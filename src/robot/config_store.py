# ruff: noqa: E501
"""車ごとの設定（vehicles/<車ID>/vehicle.yaml）と共通設定（config/robot_common.yaml）の合成・保存。

RPD_Phase_20261115_PortResearch.md §3 段1。core（src/robot/core = tests/research の写し）は
yaml 1 本（ResearchConfig）で動くので、ここで「共通 → 車両」の順に深く重ねた合成 yaml
（vehicles/<車ID>/.work/merged.yaml）を作り、core の load_config で読む。core は直さない。

車両 yaml が持つのは次のキーだけ（定数 VEHICLE_SECTIONS・VEHICLE_FEEDFORWARD_KEYS。1 か所）。
それ以外は全部共通。

- ``vehicle.*`` 全部（走行抵抗の値も直接持つ。``road_load_file`` は使わない）
- （段11 2026-11-16 で ``buttons.*`` は共通設定へ移した。車両 yaml に ``buttons:`` が残っていても
  **無視する**（共通の値を使う。読み込みのとき警告を 1 行出す）。旧 ``buttons.enabled`` も取り除く）
- 手順 2・12〜14 が ``cfg.save()`` で書く ``feedforward.*`` のキー（VEHICLE_FEEDFORWARD_KEYS）

保存の流れ（core を直さずに済ませる）:

1. core は ``cfg.save(updates)`` で合成 yaml（.work/merged.yaml）にだけ書く
2. **ランナーが ``cfg.save()`` の直後に** ``sync_saved_to_vehicle(...)`` を呼ぶと、
   合成 yaml に書かれた車両キーの値が車両 yaml へ写る（コメントは保たれる）。
   ``install_save_sync(cfg, ...)`` で ``cfg.save`` を包めば、呼び忘れを防げる。
3. 写すキーが車両 yaml に無ければ例外。共通のキーが変わっていたら例外（共通設定は変えない）
"""

from __future__ import annotations

import copy
import dataclasses
import logging
import os
import re
from pathlib import Path
from typing import Any

import yaml

from src.robot.core.config import (
    ConfigError,
    ResearchConfig,
    load_config,
    update_yaml_values,
    validate_config,
)

#: 車両 yaml が持つセクション（中のキー全部が車両キー）
VEHICLE_SECTIONS: tuple[str, ...] = ("vehicle",)

#: 以前は車両 yaml にあり、今は共通設定が持つセクション（車両 yaml に残っていても無視する）
IGNORED_VEHICLE_SECTIONS: tuple[str, ...] = ("buttons",)

#: src 専用のキー（core の検証に渡す前に合成 yaml から取り除く）
SRC_ONLY_KEYS: tuple[str, ...] = ("buttons.ign_off_press_s", "buttons.installed", "emergency.can_lost_press_s")

#: src 専用キーだけでできているセクション（キーを取り除いて空になったら、セクションごと取り除く）
SRC_ONLY_SECTIONS: tuple[str, ...] = ("emergency",)

#: src 専用キーの既定（車両 yaml に無い車で使う）
#: buttons.installed: ボタンサーボ（SG90）が付いているか。false なら全ボタンは押したことにするだけ
#: （PCA9685 に繋がず・サーボを動かさない。ユーザー決定 2026-11-16 D-15c）
#: emergency.can_lost_press_s: 非常停止で CAN 車速を読めないとき、ブレーキを最小から最大まで踏むのにかける秒数
#: （RPD_Phase_20261119_EmergencyStop。現在位置から最大までは、残りの割合だけかける）
SRC_ONLY_DEFAULTS: dict[str, Any] = {
    "buttons.ign_off_press_s": 10.0,
    "buttons.installed": True,
    "emergency.can_lost_press_s": 20.0,
}

#: 車両 yaml が持つ feedforward のキー。手順 2（pattern_drive.FF_PARAM_KEYS の MEASURED_KEYS 以外・
#: model_path・build_ff_model の model_* の 5 キー）と手順 2-0（pedal_search.config_updates の
#: stop_brake_offset_pct・creep_speed_kmh）が cfg.save() で書くもの。
VEHICLE_FEEDFORWARD_KEYS: tuple[str, ...] = (
    "model_path",
    "creep_speed_kmh",
    "creep_rate_kmhs",
    "stop_brake_offset_pct",
    "creep_accel_speeds_kmh",
    "creep_accel_kmhs",
    "engine_brake_decel_kmhs",
    "coast_decel_speeds_kmh",
    "coast_decel_kmhs",
    "pedal_gain_speeds_kmh",
    "accel_gain_kmhs_per_pct",
    "brake_gain_kmhs_per_pct",
    # 多項式モデル（C1〜C5）の作成（_build_poly_model）だけが書く参考用のキー。ResearchConfig に
    # フィールドが無いので今の core では保存で失敗する（tests でも同じ）。車両 yaml には書かない
    "model_accel_horizons_s",
    "model_brake_horizons_s",
    "model_accel_features",
    "model_brake_features",
    "model_coef_path",
)

#: 値が pkl などのファイルの場所のキー。車両 yaml では相対（vehicles/<車ID>/models/ 基準）で持つ
MODEL_PATH_KEYS: tuple[str, ...] = ("feedforward.model_path", "feedforward.model_coef_path")

#: 置き場所のキー（合成のときに必ず上書きする。tests の設定との比較でも除く）
PLACEMENT_KEYS: tuple[str, ...] = (
    "output.results_dir",
    "output.plot",
    "vehicle.road_load_file",
    *MODEL_PATH_KEYS,
)

#: model_path が無い車（まだ手順 2 をしていない）の仮の pkl 名（models/ の下。存在しない）
UNTRAINED_MODEL_NAME = "untrained.pkl"

logger = logging.getLogger(__name__)

_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")


class VehicleConfigError(ConfigError):
    """車両設定の読み込み・保存の誤り（日本語の案内つき）。"""


def vehicle_dir(vehicle_id: str, vehicles_dir: Path) -> Path:
    if not _ID_RE.match(vehicle_id):
        raise VehicleConfigError(f"車 ID は英数字・_・- だけです: {vehicle_id!r}")
    return Path(vehicles_dir) / vehicle_id


def is_vehicle_key(dotted: str) -> bool:
    """ドット区切りキーが車両キーか。"""
    section, _, key = dotted.partition(".")
    if section in VEHICLE_SECTIONS:
        return True
    return section == "feedforward" and key in VEHICLE_FEEDFORWARD_KEYS


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise VehicleConfigError(f"設定ファイルを読めません: {path}（{exc}）") from exc
    except yaml.YAMLError as exc:
        raise VehicleConfigError(f"YAML の構文エラー: {path}\n{exc}") from exc
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise VehicleConfigError(f"設定ファイルの最上位がマッピングではありません: {path}")
    return data


def _check_vehicle_keys(vehicle: dict[str, Any], path: Path) -> None:
    """車両 yaml に車両キー以外が書かれていないことを確かめる（共通設定を上書きさせない）。"""
    bad: list[str] = []
    for section, body in vehicle.items():
        if section in VEHICLE_SECTIONS or section in IGNORED_VEHICLE_SECTIONS:
            continue
        if section == "feedforward" and isinstance(body, dict):
            bad += [
                f"feedforward.{k}" for k in body if k not in VEHICLE_FEEDFORWARD_KEYS
            ]
            continue
        bad.append(str(section))
    if bad:
        raise VehicleConfigError(
            f"車両 yaml に車ごとの値でないキーがあります: {', '.join(sorted(bad))}"
            f"（共通設定は config/robot_common.yaml で変えます。{path}）"
        )


def src_only_values(merged: dict[str, Any]) -> dict[str, Any]:
    """合成前の辞書から src 専用キーの値を読む（無いキーは既定値）。"""
    out = dict(SRC_ONLY_DEFAULTS)
    for dotted in SRC_ONLY_KEYS:
        found, value = _get(merged, dotted)
        if found and value is not None:
            out[dotted] = value
    return out


def deep_merge(base: dict[str, Any], over: dict[str, Any]) -> dict[str, Any]:
    """マッピングは再帰的に、それ以外（リスト・スカラー）は over で置き換えて重ねる。"""
    out = copy.deepcopy(base)
    for key, value in over.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def _model_abs(value: str, models_dir: Path) -> str:
    """車両 yaml の model_path 等（相対は models/ 基準）を絶対のパス文字列にする。"""
    if not value:
        return value
    p = Path(value)
    return str(p if p.is_absolute() else models_dir / p)


def _model_rel(value: str, models_dir: Path) -> str:
    """絶対パスが models/ の下なら、その下の相対名にする（車両 yaml へ戻す向き）。"""
    p = Path(value)
    if not p.is_absolute():
        return value
    try:
        return str(Path(os.path.abspath(p)).relative_to(models_dir))
    except ValueError:
        return value


def compose(
    common: dict[str, Any],
    vehicle: dict[str, Any],
    *,
    data_dir: Path,
    models_dir: Path,
) -> dict[str, Any]:
    """共通 → 車両の順に重ね、置き場所のキーを上書きした辞書（core に渡す形）を返す。"""
    # 車両 yaml の buttons: は無視（共通設定の値を使う。段11）
    merged = deep_merge(common, {k: v for k, v in vehicle.items() if k not in IGNORED_VEHICLE_SECTIONS})
    buttons = merged.get("buttons")
    if isinstance(buttons, dict):
        buttons.pop("enabled", None)  # 旧 src 専用キーの名残。core は未知のキーを拒否する
    for dotted in SRC_ONLY_KEYS:  # src 専用キーは core の検証（未知のキーを拒否）に渡さない
        section, _, key = dotted.partition(".")
        if isinstance(merged.get(section), dict):
            merged[section].pop(key, None)
    for section in SRC_ONLY_SECTIONS:
        if merged.get(section) == {}:
            del merged[section]
    ff = merged.setdefault("feedforward", {})
    # 置き場所は車ごとのフォルダ基準。キーが無い車は存在しない仮の pkl 名にする
    # （core の既定値 tests/research/... を指させない）
    ff["model_path"] = _model_abs(str(ff.get("model_path") or UNTRAINED_MODEL_NAME), models_dir)
    if ff.get("model_coef_path"):
        ff["model_coef_path"] = _model_abs(str(ff["model_coef_path"]), models_dir)
    merged.setdefault("vehicle", {})["road_load_file"] = None
    out = merged.setdefault("output", {})
    out["results_dir"] = str(data_dir)
    out["plot"] = False
    return merged


class _MergedDumper(yaml.SafeDumper):
    """update_yaml_values（行単位の書き換え）が読めるよう、マッピングは入れ子の block、
    リストは 1 行の flow（[a, b, c]）で書く。"""


def _represent_list(dumper: yaml.SafeDumper, data: list[Any]) -> yaml.Node:
    return dumper.represent_sequence("tag:yaml.org,2002:seq", data, flow_style=True)


def _represent_dict(dumper: yaml.SafeDumper, data: dict[Any, Any]) -> yaml.Node:
    return dumper.represent_mapping("tag:yaml.org,2002:map", data, flow_style=False)


_MergedDumper.add_representer(list, _represent_list)
_MergedDumper.add_representer(dict, _represent_dict)


def _dump(data: dict[str, Any]) -> str:
    return yaml.dump(
        data,
        Dumper=_MergedDumper,
        allow_unicode=True,
        sort_keys=False,
        width=10**9,
    )


def _paths(vehicle_id: str, vehicles_dir: Path) -> tuple[Path, Path, Path, Path, Path]:
    """(車のフォルダ, vehicle.yaml, .work（合成 yaml 置き場）, models/, data/（CSV 置き場）)。"""
    root = vehicle_dir(vehicle_id, Path(vehicles_dir)).resolve()
    return root, root / "vehicle.yaml", root / ".work", root / "models", root / "data"


def load_vehicle_config(
    vehicle_id: str, *, vehicles_dir: Path, common_path: Path, validate: bool = True
) -> tuple[ResearchConfig, Path]:
    """共通 → 車両の順に重ねた設定を core の ResearchConfig で返す。

    合成 yaml は ``vehicles/<車ID>/.work/merged.yaml`` に書く（core の ``cfg.save()`` の保存先）。
    ``output.results_dir`` は ``vehicles/<車ID>/data/``（学習 CSV の置き場。手順 2・12〜14 が
    後で読み直す材料。走行ログの CSV は Web 側が DB へ入れた後に消す）・``output.plot`` は
    false・相対の model_path 等は ``vehicles/<車ID>/models/`` 基準・
    ``vehicle.road_load_file`` は無効にする。
    検証は core の validate_config をそのまま使い、問題があれば VehicleConfigError。

    Returns:
        (ResearchConfig, 合成 yaml のパス)
    """
    root, vehicle_path, work, models, data = _paths(vehicle_id, vehicles_dir)
    if not vehicle_path.exists():
        raise VehicleConfigError(f"車両 yaml がありません: {vehicle_path}（車を登録してください）")
    vehicle = _load_yaml(vehicle_path)
    _check_vehicle_keys(vehicle, vehicle_path)
    common_data = _load_yaml(Path(common_path))
    if any(sec in vehicle for sec in IGNORED_VEHICLE_SECTIONS):
        logger.warning(
            "車両 yaml の buttons: は使いません（共通設定 config/robot_common.yaml の値を使います）: %s",
            vehicle_path,
        )
    src_only = src_only_values(
        deep_merge(common_data, {k: v for k, v in vehicle.items() if k not in IGNORED_VEHICLE_SECTIONS})
    )
    merged = compose(common_data, vehicle, data_dir=data, models_dir=models)
    work.mkdir(parents=True, exist_ok=True)
    data.mkdir(parents=True, exist_ok=True)
    merged_path = work / "merged.yaml"
    merged_path.write_text(
        "# vehicles/<車ID>/.work/merged.yaml（共通 + 車両の合成。config_store が毎回作り直す。"
        "手で直さない）\n" + _dump(merged),
        encoding="utf-8",
    )
    cfg = load_config(merged_path)
    cfg.src_only = src_only  # type: ignore[attr-defined]  # patches の ign_off が読む（core の検証に通さない値）
    if validate:
        problems = validate_config(cfg)
        if problems:
            raise VehicleConfigError(
                "設定に問題があります:\n- " + "\n- ".join(problems) + f"\n（{vehicle_path}）"
            )
    return cfg, merged_path


def _get(data: dict[str, Any], dotted: str) -> tuple[bool, Any]:
    node: Any = data
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return False, None
        node = node[part]
    return True, node


def _same(a: Any, b: Any) -> bool:
    """値の比較。core の保存は有効 6 桁に丸めるので、数値は相対 1e-5 までは同じとみなす。"""
    if isinstance(a, tuple | list) and isinstance(b, tuple | list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b, strict=True))
    if (
        isinstance(a, int | float)
        and isinstance(b, int | float)
        and not isinstance(a, bool)
        and not isinstance(b, bool)
    ):
        return abs(float(a) - float(b)) <= max(1e-9, abs(float(b)) * 1e-5)
    return bool(a == b)


def _leaves(data: Any, prefix: str = "") -> dict[str, Any]:
    """入れ子のマッピングを「ドット区切りキー → 葉の値」に開く（リストは 1 つの葉）。"""
    if isinstance(data, dict) and data:
        out: dict[str, Any] = {}
        for k, v in data.items():
            out.update(_leaves(v, f"{prefix}.{k}" if prefix else str(k)))
        return out
    return {prefix: data}


def sync_saved_to_vehicle(
    vehicle_id: str, *, vehicles_dir: Path, common_path: Path
) -> list[str]:
    """core が合成 yaml に保存した車両キーの値を、車両 yaml へ写す。

    **ランナーが ``cfg.save()`` の直後に呼ぶ**（手順 2・12〜14・手順 2-0 の保存のたび）。
    - 合成 yaml のうち車両キー（VEHICLE_FEEDFORWARD_KEYS）で車両 yaml と値が違うものだけを、
      ``update_yaml_values`` で書く（コメント・並びはそのまま）。
      pkl のパスは models/ の下なら相対名で書く。
    - 写すキーが車両 yaml に無ければ VehicleConfigError（キーを勝手に足さない）。
    - 共通のキー（車両キー以外）が、共通 + 車両から作り直した値と違っていたら VehicleConfigError
      （共通設定を変えない。合成 yaml が手で書き換えられた場合の検出も兼ねる）。

    Returns:
        書き換えた行の説明（``"feedforward.creep_speed_kmh: 5.0 -> 5.2"`` 形式）。
    """
    root, vehicle_path, work, models, data = _paths(vehicle_id, vehicles_dir)
    saved = _load_yaml(work / "merged.yaml")
    vehicle = _load_yaml(vehicle_path)
    expected = compose(_load_yaml(Path(common_path)), vehicle, data_dir=data, models_dir=models)

    saved_leaves, expected_leaves = _leaves(saved), _leaves(expected)
    # 共通のキーが変わっていないか（置き場所のキーは固定で上書き済みなので同じはず）
    changed_common = sorted(
        k
        for k in saved_leaves.keys() | expected_leaves.keys()
        if not is_vehicle_key(k) and not _same(saved_leaves.get(k), expected_leaves.get(k))
    )
    if changed_common:
        raise VehicleConfigError(
            "共通設定のキーが保存で変わっています（共通設定は車ごとに変えられません）: "
            + ", ".join(changed_common)
        )

    updates: dict[str, Any] = {}
    for key in VEHICLE_FEEDFORWARD_KEYS:
        dotted = f"feedforward.{key}"
        ok_s, new = _get(saved, dotted)
        ok_e, old = _get(expected, dotted)
        if not ok_s:
            continue
        if dotted in MODEL_PATH_KEYS and isinstance(new, str):
            new = _model_rel(new, models)
            old = _model_rel(str(old), models) if ok_e and isinstance(old, str) else old
        if ok_e and _same(new, old):
            continue
        # 車両 yaml に無いキーは書けない（model_path は仮の値が入るので yaml 側の有無で見る）
        if not _get(vehicle, dotted)[0]:
            raise VehicleConfigError(
                f"{dotted} が車両 yaml にありません。車両 yaml に追加してください: {vehicle_path}"
            )
        updates[dotted] = new
    if not updates:
        return []
    return update_yaml_values(vehicle_path, updates)


def install_save_sync(
    cfg: ResearchConfig, vehicle_id: str, *, vehicles_dir: Path, common_path: Path
) -> None:
    """``cfg.save`` を包み、保存の直後に sync_saved_to_vehicle を呼ぶようにする（呼び忘れ防止）。

    ResearchConfig は frozen でないのでインスタンスの save を差し替えられる。
    ただし dataclasses.replace(cfg, ...) で作った別のインスタンスには引き継がれない
    （その場合はランナーが sync_saved_to_vehicle を直接呼ぶ）。
    保存の前に、共通のキーを変える更新が混じっていないか（値が今と違うか。キーが無いものも）を確かめ、
    混じっていれば何も書かずに VehicleConfigError にする。
    """
    original = cfg.save

    def save(updates: dict[str, Any]) -> list[str]:
        now = dataclasses.asdict(cfg)
        now.pop("source_path", None)
        current = _leaves(now)
        bad = [
            k
            for k, v in updates.items()
            if not is_vehicle_key(k) and not (k in current and _same(current[k], v))
        ]
        if bad:
            raise VehicleConfigError(
                "共通設定のキーは車ごとに保存できません: " + ", ".join(sorted(bad))
            )
        changed = original(updates)
        sync_saved_to_vehicle(vehicle_id, vehicles_dir=vehicles_dir, common_path=common_path)
        return changed

    cfg.save = save  # type: ignore[method-assign]


def save_vehicle_values(
    vehicle_id: str, updates: dict[str, Any], *, vehicles_dir: Path
) -> list[str]:
    """車両 yaml の車両キー（vehicle.*・車両の feedforward.*）の値を書き換える。

    手動運転の ``save_min``（vehicle.min_accel/brake_opening_pct）など、ランナーが core の
    ``cfg.save`` を通さずに車両 yaml を直接直すときの入口。コメント・並びは保つ。
    車両キー以外・車両 yaml に無いキーは VehicleConfigError。
    """
    _, vehicle_path, _, _, _ = _paths(vehicle_id, vehicles_dir)
    bad = sorted(k for k in updates if not is_vehicle_key(k))
    if bad:
        raise VehicleConfigError("車両キーではありません: " + ", ".join(bad))
    if not vehicle_path.exists():
        raise VehicleConfigError(f"車両 yaml がありません: {vehicle_path}")
    try:
        return update_yaml_values(vehicle_path, updates)
    except ConfigError as exc:
        raise VehicleConfigError(f"{exc}（{vehicle_path}）") from exc
