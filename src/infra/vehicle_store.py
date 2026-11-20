# ruff: noqa: E501
"""車両 yaml（vehicles/<車ID>/vehicle.yaml）の一覧・読み・作成・更新・削除と、選んでいる車。

RPD_Phase_20261115_PortResearch.md §3 段4。DB の profiles（VehicleProfile）に代わる。

- 車の ID ＝ フォルダ名。1 台 1 フォルダ（``vehicle.yaml``・``models/``・``data/``・``state.json``）
- **人が決める値**（``vehicle.*``）だけ書ける（``buttons.*`` は段11 で共通設定へ移した。読むだけ）。学習で決まる ``feedforward.*`` は読むだけ
  （手順 2・12〜14 がランナー経由で書く）
- 書き込みは core の ``update_yaml_values``（コメントを保つ）。書く前に、共通設定と合成して
  core の ``validate_config`` を通す（通らなければ ``VehicleInvalid`` に日本語の理由）
- 新しい車は共通の雛形（``TEMPLATE``）から作る。学習の値は無し（``model_path`` は null）、min は仮値で
  ``state.json`` の ``min_measured`` が false（「未測定」の印。ジョグで測って save-min すると true）
- 選んでいる車は ``vehicles/.active`` に ID を 1 行で持つ（再起動後も残る。サーバー全体で 1 台）

state.json は Web 側の付属情報（yaml には書かない値）。無い車（手順外で取り込んだ車）は min を測定済みとみなす。
"""

from __future__ import annotations

import copy
import json
import os
import re
import shutil
import tempfile
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml

from src.robot import config_store
from src.robot.core.config import ConfigError, load_config, update_yaml_values, validate_config

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VEHICLES_DIR = REPO_ROOT / "vehicles"
DEFAULT_COMMON = REPO_ROOT / "config" / "robot_common.yaml"

ACTIVE_FILE = ".active"
STATE_FILE = "state.json"

#: 人が決める vehicle.* のキー（書ける）。型は書き込みの検査に使う
VEHICLE_EDITABLE: dict[str, type | tuple[type, ...]] = {
    "name": str,
    "max_speed_kmh": (int, float),
    "max_decel_g": (int, float),
    "min_accel_opening_pct": (int, float),
    "max_accel_opening_pct": (int, float),
    "min_brake_opening_pct": (int, float),
    "max_brake_opening_pct": (int, float),
    "stop_deviation_threshold_kmh": (int, float),
    "stop_deviation_duration_s": (int, float),
    "test_mass_kg": (int, float, type(None)),
    "road_load_a0_n": (int, float, type(None)),
    "road_load_a1_n_per_kmh": (int, float, type(None)),
    "road_load_a2_n_per_kmh2": (int, float, type(None)),
    "road_load_source": str,
}

_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")

#: 新しい車の雛形。値は「仮」（実車で測る・手順 2 が書く）。feedforward のキーは手順 2・12〜14 の保存先
#: （config_store.sync_saved_to_vehicle は車両 yaml に無いキーを書けないので、全部ここに置く）
TEMPLATE = """\
# =====================================================================
# 車両 yaml（vehicles/<車ID>/vehicle.yaml）。車ごとの値だけを持つ。それ以外は config/robot_common.yaml。
#   Web の「プロファイル」画面が作った雛形。人が決める値（vehicle.）は画面から直せる。ボタンの設定は共通設定（段11）。
#   feedforward の値は手順 2・12〜14（学習運転）が書く。手で直さない。
# =====================================================================

# ── 車両 ─────────────────────────────────────────────────────────────
vehicle:
  name: {name}
  max_speed_kmh: {max_speed_kmh}             # 最高速 [km/h]
  max_decel_g: {max_decel_g}                 # 最大減速G [G]
  # ペダルに触れた位置 [%]（待機位置・効く開度の原点）。**仮値**: キャリブレーション画面のジョグで測って保存する
  min_accel_opening_pct: {min_accel_opening_pct}
  max_accel_opening_pct: {max_accel_opening_pct}    # アクセル開度上限 [%]（機構保護）
  min_brake_opening_pct: {min_brake_opening_pct}
  max_brake_opening_pct: {max_brake_opening_pct}    # ブレーキ開度上限 [%]（機構保護）
  stop_deviation_threshold_kmh: 2.0           # 異常停止判定の偏差しきい値 [km/h]
  stop_deviation_duration_s: 9999             # 上記が続いたら停止 [s]
  # 運転指標（ER・EER）だけに使う。分からなければ null（その指標は出さない）
  test_mass_kg: null
  road_load_a0_n: null
  road_load_a1_n_per_kmh: null
  road_load_a2_n_per_kmh2: null
  road_load_source: ""
  road_load_file: null
# ── Feedforward（手順 2・12〜14 が書く値だけ。他の feedforward の値は共通） ──
feedforward:
  model_path: null    # 手順 2 が作る pkl（models/ の下の名前）。null ＝ 未学習
  creep_speed_kmh: 5.0
  creep_rate_kmhs: 0.5
  stop_brake_offset_pct: 15.0
  engine_brake_decel_kmhs: 1.6
  coast_decel_speeds_kmh: []
  coast_decel_kmhs: []
  # クリープ加速カーブの**仮の 2 点**（core の validate_config が空を許さないため。手順 2-0 が測り直して置き換える）
  creep_accel_speeds_kmh: [0.5, 5.0]
  creep_accel_kmhs: [1.0, 0.0]
  pedal_gain_speeds_kmh: []
  accel_gain_kmhs_per_pct: []
  brake_gain_kmhs_per_pct: []
"""

#: 新しい車の既定値（仮）
NEW_VEHICLE_DEFAULTS: dict[str, Any] = {
    "max_speed_kmh": 140.0,
    "max_decel_g": 0.4,
    "min_accel_opening_pct": 5.0,
    "max_accel_opening_pct": 80.0,
    "min_brake_opening_pct": 5.0,
    "max_brake_opening_pct": 80.0,
}


class VehicleStoreError(Exception):
    """車両 API の誤りの基底（日本語の案内つき）。"""


class VehicleNotFound(VehicleStoreError):  # noqa: N818
    pass


class VehicleExists(VehicleStoreError):  # noqa: N818
    pass


class VehicleInvalid(VehicleStoreError):  # noqa: N818
    """値の検証に通らない。``problems`` は日本語の理由の一覧。"""

    def __init__(self, problems: list[str]) -> None:
        super().__init__("; ".join(problems))
        self.problems = problems


@dataclass(frozen=True)
class VehicleInfo:
    """読んだ車の値（API の応答の元）。"""

    id: str
    name: str
    vehicle: dict[str, Any]
    buttons: dict[str, Any]
    feedforward: dict[str, Any]
    min_accel_measured: bool
    min_brake_measured: bool
    learned: bool
    model_path: str | None
    learned_at: datetime | None
    candidate: str | None
    updated_at: datetime | None
    error: str | None = None


def slugify(text: str) -> str:
    """名前から車 ID（フォルダ名）の候補を作る。英数字・_・- 以外は _ にして詰める。"""
    slug = re.sub(r"[^A-Za-z0-9_-]+", "_", text.strip()).strip("_-")
    return slug[:40]


class VehicleStore:
    def __init__(
        self,
        vehicles_dir: Path = DEFAULT_VEHICLES_DIR,
        common_path: Path = DEFAULT_COMMON,
    ) -> None:
        self.vehicles_dir = Path(vehicles_dir)
        self.common_path = Path(common_path)

    # ── パス ───────────────────────────────────────────────────────

    def _dir(self, vehicle_id: str) -> Path:
        if not _ID_RE.match(vehicle_id):
            raise VehicleNotFound(f"車 ID は英数字・_・- だけです: {vehicle_id!r}")
        return self.vehicles_dir / vehicle_id

    def _yaml(self, vehicle_id: str) -> Path:
        return self._dir(vehicle_id) / "vehicle.yaml"

    def exists(self, vehicle_id: str) -> bool:
        try:
            return self._yaml(vehicle_id).is_file()
        except VehicleNotFound:
            return False

    # ── 一覧・読み ─────────────────────────────────────────────────

    def list_ids(self) -> list[str]:
        if not self.vehicles_dir.is_dir():
            return []
        return sorted(
            p.name
            for p in self.vehicles_dir.iterdir()
            if p.is_dir() and _ID_RE.match(p.name) and (p / "vehicle.yaml").is_file()
        )

    def list_all(self) -> list[VehicleInfo]:
        out: list[VehicleInfo] = []
        for vehicle_id in self.list_ids():
            try:
                out.append(self.read(vehicle_id))
            except VehicleStoreError as exc:  # 壊れた yaml も一覧には出す（画面で気付けるように）
                out.append(
                    VehicleInfo(
                        id=vehicle_id, name=vehicle_id, vehicle={}, buttons={}, feedforward={},
                        min_accel_measured=False, min_brake_measured=False, learned=False,
                        model_path=None, learned_at=None, candidate=None, updated_at=None,
                        error=str(exc),
                    )
                )
        return out

    def _load_yaml(self, vehicle_id: str) -> dict[str, Any]:
        path = self._yaml(vehicle_id)
        if not path.is_file():
            raise VehicleNotFound(f"車 {vehicle_id!r} が見つかりません")
        try:
            return config_store._load_yaml(path)
        except config_store.VehicleConfigError as exc:
            raise VehicleStoreError(f"車両 yaml を読めません: {exc}") from exc

    def _state(self, vehicle_id: str) -> dict[str, Any]:
        path = self._dir(vehicle_id) / STATE_FILE
        if not path.is_file():
            return {}
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return {}
        return data if isinstance(data, dict) else {}

    def _write_state(self, vehicle_id: str, state: dict[str, Any]) -> None:
        (self._dir(vehicle_id) / STATE_FILE).write_text(
            json.dumps(state, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    def _common_candidate(self) -> str | None:
        try:
            common = config_store._load_yaml(self.common_path)
        except config_store.VehicleConfigError:
            return None
        value = (common.get("feedforward") or {}).get("candidate")
        return str(value) if value else None

    def _common_buttons(self) -> dict[str, Any]:
        """ボタンの設定（共通設定 buttons.*。読むだけ）。"""
        try:
            common = config_store._load_yaml(self.common_path)
        except config_store.VehicleConfigError:
            return {}
        buttons = dict(common.get("buttons") or {})
        buttons.pop("enabled", None)
        return buttons

    def read(self, vehicle_id: str) -> VehicleInfo:
        data = self._load_yaml(vehicle_id)
        vehicle = dict(data.get("vehicle") or {})
        buttons = self._common_buttons()
        feedforward = {
            k: v for k, v in (data.get("feedforward") or {}).items()
            if k in config_store.VEHICLE_FEEDFORWARD_KEYS
        }
        state = self._state(vehicle_id)
        measured = state.get("min_measured") or {}
        model_name = feedforward.get("model_path")
        pkl: Path | None = None
        if model_name:
            p = Path(str(model_name))
            pkl = p if p.is_absolute() else self._dir(vehicle_id) / "models" / p
        learned = bool(pkl and pkl.is_file())
        mtime = lambda p: datetime.fromtimestamp(p.stat().st_mtime, tz=UTC)  # noqa: E731
        return VehicleInfo(
            id=vehicle_id,
            name=str(vehicle.get("name") or vehicle_id),
            vehicle=vehicle,
            buttons=buttons,
            feedforward=feedforward,
            # state.json の無い車は測定済みとみなす（新規作成だけが false で始まる）
            min_accel_measured=bool(measured.get("accel", True)),
            min_brake_measured=bool(measured.get("brake", True)),
            learned=learned,
            model_path=str(model_name) if model_name else None,
            learned_at=mtime(pkl) if learned and pkl is not None else None,
            candidate=self._common_candidate(),
            updated_at=mtime(self._yaml(vehicle_id)),
        )

    # ── 作成 ───────────────────────────────────────────────────────

    def _unique_id(self, base: str) -> str:
        if not self.exists(base) and not (self.vehicles_dir / base).exists():
            return base
        for n in range(2, 1000):
            cand = f"{base}_{n}"
            if not (self.vehicles_dir / cand).exists():
                return cand
        raise VehicleExists(f"車 ID {base!r} の空きがありません")

    def create(
        self,
        name: str,
        *,
        vehicle_id: str | None = None,
        values: dict[str, Any] | None = None,
    ) -> VehicleInfo:
        """共通の雛形から新しい車を作る。学習の値なし・min は仮値（未測定）。

        ``vehicle_id`` を省略すると名前から作る（英数字が無ければ vehicle_日時）。
        """
        name = name.strip()
        if not name:
            raise VehicleInvalid(["名前を入れてください"])
        if vehicle_id is not None:
            if not _ID_RE.match(vehicle_id):
                raise VehicleInvalid([f"車 ID は英数字・_・- だけです: {vehicle_id!r}"])
            if (self.vehicles_dir / vehicle_id).exists():
                raise VehicleExists(f"車 ID {vehicle_id!r} は既にあります")
            new_id = vehicle_id
        else:
            base = slugify(name) or f"vehicle_{datetime.now():%Y%m%d_%H%M%S}"
            new_id = self._unique_id(base)
        merged = {**NEW_VEHICLE_DEFAULTS, **(values or {})}
        text = TEMPLATE.format(
            name=config_store_format_str(name),
            **{k: merged[k] for k in NEW_VEHICLE_DEFAULTS},
        )
        root = self.vehicles_dir / new_id
        root.mkdir(parents=True)
        try:
            (root / "models").mkdir()
            (root / "data").mkdir()
            (root / "vehicle.yaml").write_text(text, encoding="utf-8")
            self._write_state(new_id, {"min_measured": {"accel": False, "brake": False}})
            self._validate(new_id, yaml.safe_load(text))
        except BaseException:
            shutil.rmtree(root, ignore_errors=True)
            raise
        return self.read(new_id)

    # ── 更新 ───────────────────────────────────────────────────────

    def _validate(self, vehicle_id: str, vehicle_data: dict[str, Any]) -> None:
        """車両 yaml の内容を共通設定と合成し、core の validate_config を通す。"""
        root = self._dir(vehicle_id)
        merged = config_store.compose(
            config_store._load_yaml(self.common_path),
            vehicle_data,
            data_dir=root / "data",
            models_dir=root / "models",
        )
        with tempfile.TemporaryDirectory(prefix="vehicle_validate_") as tmp:
            path = Path(tmp) / "merged.yaml"
            path.write_text(config_store._dump(merged), encoding="utf-8")
            try:
                cfg = load_config(path)
                problems = validate_config(cfg)
            except ConfigError as exc:
                problems = [str(exc)]
        if problems:
            raise VehicleInvalid(problems)

    def update(
        self,
        vehicle_id: str,
        *,
        vehicle: dict[str, Any] | None = None,
    ) -> VehicleInfo:
        """人が決める値（vehicle.*）だけを書き換える（コメントは保つ）。通らなければ何も書かず VehicleInvalid。

        min を書いたら、その軸は「測定済み」にする（人が決めた値なので）。
        ボタンの設定は共通設定へ移した（段11）ので、ここでは書けない。
        """
        vehicle = vehicle or {}
        problems: list[str] = []
        for key, value in vehicle.items():
            if key not in VEHICLE_EDITABLE:
                problems.append(f"vehicle.{key} は書き換えられません（学習で決まる値・または未対応）")
            elif isinstance(value, bool) and bool not in _as_tuple(VEHICLE_EDITABLE[key]):
                problems.append(f"vehicle.{key} は数値で指定してください")
            elif not isinstance(value, VEHICLE_EDITABLE[key]):
                problems.append(f"vehicle.{key} の型が違います: {value!r}")
        if problems:
            raise VehicleInvalid(problems)

        data = self._load_yaml(vehicle_id)
        after = copy.deepcopy(data)
        after.setdefault("vehicle", {}).update(vehicle)
        self._validate(vehicle_id, after)
        updates = {f"vehicle.{k}": v for k, v in vehicle.items()}
        if updates:
            try:
                update_yaml_values(self._yaml(vehicle_id), updates)
            except ConfigError as exc:
                raise VehicleInvalid([f"車両 yaml に書けません: {exc}"]) from exc
        for axis in ("accel", "brake"):
            if f"min_{axis}_opening_pct" in vehicle:
                self.mark_min_measured(vehicle_id, axis)
        return self.read(vehicle_id)

    def mark_min_measured(self, vehicle_id: str, axis: str) -> None:
        """min を測って保存した（または人が決めた）軸を「測定済み」にする。axis は accel / brake / both。"""
        state = self._state(vehicle_id)
        measured = dict(state.get("min_measured") or {})
        for name in ("accel", "brake") if axis == "both" else (axis,):
            measured[name] = True
        state["min_measured"] = measured
        self._write_state(vehicle_id, state)

    # ── 削除 ───────────────────────────────────────────────────────

    def delete(self, vehicle_id: str) -> None:
        """車のフォルダを消す（yaml・pkl・学習データごと）。確認は画面側。"""
        root = self._dir(vehicle_id)
        if not root.is_dir():
            raise VehicleNotFound(f"車 {vehicle_id!r} が見つかりません")
        shutil.rmtree(root)
        if self.get_active() == vehicle_id:
            self.set_active(None)

    # ── 選んでいる車 ───────────────────────────────────────────────

    def get_active(self) -> str | None:
        path = self.vehicles_dir / ACTIVE_FILE
        try:
            value = path.read_text(encoding="utf-8").strip()
        except OSError:
            return None
        return value if value and self.exists(value) else None

    def set_active(self, vehicle_id: str | None) -> None:
        path = self.vehicles_dir / ACTIVE_FILE
        if vehicle_id is None:
            path.unlink(missing_ok=True)
            return
        if not self.exists(vehicle_id):
            raise VehicleNotFound(f"車 {vehicle_id!r} が見つかりません")
        self.vehicles_dir.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(vehicle_id + "\n", encoding="utf-8")
        os.replace(tmp, path)


def _as_tuple(t: type | tuple[type, ...]) -> tuple[type, ...]:
    return t if isinstance(t, tuple) else (t,)


def config_store_format_str(value: str) -> str:
    """yaml の 1 行値としての文字列（core の書き方に合わせる）。"""
    from src.robot.core.config import _format_str  # noqa: PLC0415

    return _format_str(value)
