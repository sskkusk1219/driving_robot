#!/usr/bin/env python3
# ruff: noqa: E501
"""tests/research の今の設定から、共通設定と最初の 1 台（試験車）の車両フォルダを作る（段1）。

    .venv/bin/python scripts/import_research_vehicle.py                # 両方作る
    .venv/bin/python scripts/import_research_vehicle.py --common-only  # config/robot_common.yaml だけ
    .venv/bin/python scripts/import_research_vehicle.py --force        # 既にあっても作り直す

作るもの:
- ``config/robot_common.yaml``: tests/research/config_testVehicle.yaml から車両キーを除いたもの
  （コメントは保つ。車両キーの定義は src/robot/config_store.py の定数 1 か所）。
- ``vehicles/<車ID>/vehicle.yaml``: 車両キー（vehicle.* 全部・buttons.* 全部 + buttons.enabled・
  手順 2 が書く feedforward のキー）。走行抵抗は tests/research/private/road_load.yaml（あれば）の値を
  vehicle.* に直接書く（road_load_file は null）。
- ``vehicles/<車ID>/models/``: yaml が指す pkl のコピー（model_path・model_coef_path と、main.py の
  hold_pkl_path・stop_pkl_path の規則の別名 pkl があれば）。vehicle.yaml の model_path は models/ 基準の相対名。

tests 側は読むだけ（1 文字も書き換えない）。vehicles/ は .gitignore（公開しない値を含む）。
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.robot.config_store import (  # noqa: E402
    MODEL_PATH_KEYS,
    VEHICLE_FEEDFORWARD_KEYS,
    VEHICLE_SECTIONS,
    deep_merge,
    vehicle_dir,
)
from src.robot.core.config import update_yaml_values  # noqa: E402

DEFAULT_SOURCE = REPO_ROOT / "tests" / "research" / "config_testVehicle.yaml"
DEFAULT_ROAD_LOAD = REPO_ROOT / "tests" / "research" / "private" / "road_load.yaml"
DEFAULT_COMMON = REPO_ROOT / "config" / "robot_common.yaml"
DEFAULT_VEHICLES = REPO_ROOT / "vehicles"
DEFAULT_RESULTS = REPO_ROOT / "tests" / "research" / "results"
ROAD_LOAD_KEYS = (
    "test_mass_kg",
    "road_load_a0_n",
    "road_load_a1_n_per_kmh",
    "road_load_a2_n_per_kmh2",
    "road_load_source",
)

_TOP_KEY = re.compile(r"^([A-Za-z_][A-Za-z0-9_]*):")
_FF_KEY = re.compile(r"^  ([A-Za-z_][A-Za-z0-9_]*):")

COMMON_HEADER = """\
# =====================================================================
# 共通設定（robot_common.yaml）。ユーザーには見せない。
#   scripts/import_research_vehicle.py が tests/research/config_testVehicle.yaml から作った
#   （車ごとの値 = vehicle.*・buttons.*・手順 2 が書く feedforward のキーは vehicles/<車ID>/vehicle.yaml へ）。
#   車両キーの一覧は src/robot/config_store.py（VEHICLE_SECTIONS・VEHICLE_FEEDFORWARD_KEYS）。
#   合成のとき output.results_dir・output.plot・vehicle.road_load_file は config_store が上書きする。
# 以下は元の yaml のヘッダ（コメントは tests/research の書き方のまま）。
# =====================================================================

"""

VEHICLE_HEADER = """\
# =====================================================================
# 車両 yaml（vehicles/<車ID>/vehicle.yaml）。車ごとの値だけを持つ。それ以外は config/robot_common.yaml。
#   scripts/import_research_vehicle.py が tests/research/config_testVehicle.yaml から作った。
#   手順 2・12〜14 が書く feedforward のキーは、ランナーが config_store.sync_saved_to_vehicle で
#   ここへ写す（コメントは保たれる）。model_path は models/ 基準の相対名。
#   公開しない値（走行抵抗など）を含むので git 管理外（vehicles/ は .gitignore）。
# =====================================================================

"""


def _blocks(lines: list[str]) -> list[tuple[str | None, int, int, int]]:
    """トップレベルのキーごとのブロック: (キー, ヘッダ開始, キー行, ブロック終端)。

    ヘッダ = キー行の直前の「空行・列 0 のコメント」の連なり（次のセクションの見出し）。
    先頭のキーより前の分は (None, 0, 先頭のキー行, ...) ではなく呼び出し側で扱う。
    """
    key_idx = [i for i, ln in enumerate(lines) if _TOP_KEY.match(ln)]
    starts: list[int] = []
    for i in key_idx:
        j = i
        while j > 0 and (lines[j - 1].strip() == "" or lines[j - 1].startswith("#")):
            j -= 1
        starts.append(j)
    out: list[tuple[str | None, int, int, int]] = []
    for n, i in enumerate(key_idx):
        end = starts[n + 1] if n + 1 < len(starts) else len(lines)
        m = _TOP_KEY.match(lines[i])
        out.append((m.group(1) if m else None, starts[n], i, end))
    return out


def split_yaml_text(text: str) -> tuple[str, str]:
    """元の yaml の文字列を (共通の文字列, 車両の文字列) に分ける。コメントは保つ。"""
    lines = text.splitlines(keepends=True)
    blocks = _blocks(lines)
    common: list[str] = []
    vehicle: list[str] = []
    pos = 0
    for key, hstart, kidx, end in blocks:
        header = lines[hstart:kidx]
        # 先頭より前（ファイルのヘッダ）と、前のブロックの末尾のコメントは共通側
        common += lines[pos:hstart]
        pos = end
        body = lines[kidx:end]
        if key in VEHICLE_SECTIONS:
            # ヘッダの最後の空行より後ろ（見出し）だけを車両側へ。それより前は共通側に残す
            last_blank = max((i for i, ln in enumerate(header) if ln.strip() == ""), default=-1)
            common += header[: last_blank + 1]
            common.append(f"# （{key}: は車ごとの値なので vehicles/<車ID>/vehicle.yaml にある）\n\n")
            vehicle += header[last_blank + 1 :]
            if key == "buttons":
                vehicle.append(body[0])
                vehicle.append(
                    "  enabled: true   # src 専用: ボタン操作（サーボ）の有無。"
                    "サーボの無い車は false（core の検証に渡す前に取り除く）\n"
                )
                vehicle += body[1:]
            else:
                vehicle += body
        elif key == "feedforward":
            common += header
            common.append(body[0])
            vehicle.append("# ── Feedforward（手順 2 が書く値だけ。他の feedforward の値は共通） ──\n")
            vehicle.append(body[0])
            owner: list[str] = common
            buf: list[str] = []
            for ln in body[1:]:
                m = _FF_KEY.match(ln)
                if m:
                    owner = vehicle if m.group(1) in VEHICLE_FEEDFORWARD_KEYS else common
                    owner += buf
                    buf = []
                    owner.append(ln)
                elif ln.strip() == "" or ln.lstrip().startswith("#"):
                    buf.append(ln)
                else:  # キーの続きの行（複数行の値）。直前の持ち主へ
                    owner += buf
                    buf = []
                    owner.append(ln)
            common += buf
            vehicle.append("\n")
        else:
            common += header + body
    common += lines[pos:]
    return "".join(common), "".join(vehicle)


def _copy_models(rel_values: dict[str, str], dst: Path) -> list[str]:
    """yaml が指す pkl と、hold/stop の別名 pkl（main.py の規則）をコピーする。"""
    copied: list[str] = []
    dst.mkdir(parents=True, exist_ok=True)
    for dotted, value in rel_values.items():
        src = Path(value)
        if not src.is_absolute():
            src = REPO_ROOT / src
        cands = [src]
        if dotted == "feedforward.model_path":
            cands += [
                src.with_name(f"{src.stem}_hold{src.suffix}"),
                src.with_name(f"{src.stem}_stop{src.suffix}"),
            ]
        for c in cands:
            if c.exists():
                shutil.copy2(c, dst / c.name)
                copied.append(c.name)
            elif c is src:
                print(f"警告: {dotted} の pkl がありません: {c}", file=sys.stderr)
    return copied


def build(
    source: Path,
    road_load: Path,
    vehicle_id: str,
    common_out: Path,
    vehicles_dir: Path,
    *,
    do_common: bool,
    do_vehicle: bool,
    force: bool,
    results: Path = DEFAULT_RESULTS,
) -> None:
    text = source.read_text(encoding="utf-8")
    original = yaml.safe_load(text)
    common_text, vehicle_text = split_yaml_text(text)
    common_data = yaml.safe_load(common_text) or {}
    vehicle_data = yaml.safe_load(vehicle_text) or {}

    # 分けた結果を重ね直すと元に戻る（buttons.enabled だけ余分）ことを確かめる
    rebuilt = deep_merge(common_data, vehicle_data)
    rebuilt.get("buttons", {}).pop("enabled", None)
    if rebuilt != original:
        diff = sorted(
            k for k in original.keys() | rebuilt.keys() if original.get(k) != rebuilt.get(k)
        )
        raise SystemExit(f"分割の検算に失敗しました（セクション: {diff}）")

    if do_common:
        if common_out.exists() and not force:
            raise SystemExit(f"{common_out} が既にあります（作り直すなら --force）")
        common_out.parent.mkdir(parents=True, exist_ok=True)
        common_out.write_text(COMMON_HEADER + common_text, encoding="utf-8")
        print(f"作った: {common_out}（{common_text.count(chr(10))} 行）")

    if not do_vehicle:
        return
    root = vehicle_dir(vehicle_id, vehicles_dir)
    vehicle_path = root / "vehicle.yaml"
    if vehicle_path.exists() and not force:
        raise SystemExit(
            f"{vehicle_path} が既にあります（学習した値を含むので上書きしません。作り直すなら --force）"
        )
    root.mkdir(parents=True, exist_ok=True)
    vehicle_path.write_text(VEHICLE_HEADER + vehicle_text, encoding="utf-8")

    updates: dict[str, Any] = {"vehicle.road_load_file": None}
    if road_load.exists():
        raw = yaml.safe_load(road_load.read_text(encoding="utf-8")) or {}
        unknown = sorted(set(raw) - set(ROAD_LOAD_KEYS))
        if unknown:
            raise SystemExit(f"走行抵抗のファイルに未知のキー: {unknown}")
        updates |= {f"vehicle.{k}": v for k, v in raw.items()}
        print(f"走行抵抗: {road_load.name} の {len(raw)} 個を vehicle.* に直接書いた（値は表示しない）")
    else:
        print("走行抵抗のファイルが無いので、yaml の値（null）のまま")

    ff = original.get("feedforward") or {}
    rel: dict[str, str] = {}
    for dotted in MODEL_PATH_KEYS:
        value = ff.get(dotted.split(".", 1)[1])
        if value:
            rel[dotted] = str(value)
            updates[dotted] = Path(str(value)).name  # models/ 基準の相対名
    update_yaml_values(vehicle_path, updates)
    copied = _copy_models(rel, root / "models")
    print(f"作った: {vehicle_path}（{vehicle_text.count(chr(10))} 行）")
    print(f"pkl をコピー: {len(copied)} 個 → {root / 'models'}")
    for name in copied:
        print(f"  {name}")
    copy_training_csvs(vehicle_id, vehicles_dir, results)


def _source_csv_names(meta: dict[str, Any]) -> list[str]:
    """pkl の meta ``sources``（'+' かカンマ区切りの文字列、または list）→ CSV 名（core main の
    _pkl_source_csvs と同じ解釈）。"""
    raw = meta.get("sources")
    if isinstance(raw, str):
        names = [n for part in raw.split("+") for n in part.split(",")]
    elif raw:
        names = [str(n) for n in raw]
    else:
        names = []
    return [n.strip() for n in names if n.strip()]


def copy_training_csvs(vehicle_id: str, vehicles_dir: Path, results: Path) -> list[str]:
    """車両の models/ の pkl が参照する学習 CSV を results から data/ へコピーする（既にあれば
    そのまま）。手順 13・14 の ``_pkl_source_csvs`` が data/（= output.results_dir）から探すため。"""
    from src.robot.core.fwd_map import load_forward_map  # noqa: PLC0415

    root = vehicle_dir(vehicle_id, vehicles_dir)
    data = root / "data"
    data.mkdir(parents=True, exist_ok=True)
    wanted: list[str] = []
    for pkl in sorted((root / "models").glob("*.pkl")):
        for name in _source_csv_names(load_forward_map(pkl).meta):
            if name not in wanted:
                wanted.append(name)
    done: list[str] = []
    for name in wanted:
        src = results / name
        dst = data / name
        if dst.exists():
            print(f"  あり（そのまま）: {name}")
        elif src.exists():
            shutil.copy2(src, dst)
            done.append(name)
            print(f"  コピー: {name}")
        else:
            print(f"  警告: {src} がありません", file=sys.stderr)
    print(f"学習 CSV: pkl が参照 {len(wanted)} 個・今回コピー {len(done)} 個 → {data}")
    return done


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--vehicle-id", default="test_vehicle")
    ap.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    ap.add_argument("--road-load", type=Path, default=DEFAULT_ROAD_LOAD)
    ap.add_argument("--common-out", type=Path, default=DEFAULT_COMMON)
    ap.add_argument("--vehicles-dir", type=Path, default=DEFAULT_VEHICLES)
    ap.add_argument("--common-only", action="store_true")
    ap.add_argument("--vehicle-only", action="store_true")
    ap.add_argument(
        "--data-only",
        action="store_true",
        help="学習 CSV（pkl の meta sources）を tests/research/results から vehicles/<車ID>/data/ へ"
        "コピーするだけ（yaml・pkl は触らない。--force は不要。既にあれば上書きしない）",
    )
    ap.add_argument("--results", type=Path, default=DEFAULT_RESULTS, help="学習 CSV の元の場所")
    ap.add_argument("--force", action="store_true", help="既にあるファイルを作り直す")
    args = ap.parse_args()
    if args.data_only:
        copy_training_csvs(args.vehicle_id, args.vehicles_dir, args.results)
        return 0
    build(
        args.source,
        args.road_load,
        args.vehicle_id,
        args.common_out,
        args.vehicles_dir,
        do_common=not args.vehicle_only,
        do_vehicle=not args.common_only,
        force=args.force,
        results=args.results,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
