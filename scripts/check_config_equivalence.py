#!/usr/bin/env python3
"""等価確認①: 共通 + 車両の合成設定が、tests の load_config(config_testVehicle.yaml) と一致するか。

    .venv/bin/python scripts/check_config_equivalence.py [--vehicle-id test_vehicle]

比べるのは ResearchConfig の全フィールド（dataclasses.asdict）。置き場所のキー
（config_store.PLACEMENT_KEYS: output.results_dir・output.plot・feedforward.model_path・
model_coef_path・vehicle.road_load_file）と source_path だけは違ってよい（別に中身を確かめる）:
- model_path は tests 側の pkl と同じ名前で、車両の models/ の下にあること
- road_load_file は無効（None）。走行抵抗の値は tests 側が private から読んだ値と同じ（下で比べる）
- results_dir は車両の data/、plot は False

tests/research と src/robot/core の両方を import するので、tests/unit ではなくここに置く
（本番とテストの隔離: tests は tests だけ、src は src だけで完結させる）。
一致しなければ終了コード 1。値（走行抵抗など）は表示しない。
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.robot import config_store  # noqa: E402
from tests.research import config as tests_config  # noqa: E402


def _flatten(data: Any, prefix: str = "") -> dict[str, Any]:
    if isinstance(data, dict):
        out: dict[str, Any] = {}
        for k, v in data.items():
            out.update(_flatten(v, f"{prefix}.{k}" if prefix else k))
        return out
    if isinstance(data, tuple):
        data = list(data)
    return {prefix: data}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vehicle-id", default="test_vehicle")
    ap.add_argument("--vehicles-dir", type=Path, default=REPO_ROOT / "vehicles")
    ap.add_argument("--common", type=Path, default=REPO_ROOT / "config" / "robot_common.yaml")
    args = ap.parse_args()

    theirs = tests_config.load_config(REPO_ROOT / "tests" / "research" / "config_testVehicle.yaml")
    ours, merged_path = config_store.load_vehicle_config(
        args.vehicle_id, vehicles_dir=args.vehicles_dir, common_path=args.common, validate=False
    )
    a = _flatten(dataclasses.asdict(theirs))
    b = _flatten(dataclasses.asdict(ours))
    a.pop("source_path", None)
    b.pop("source_path", None)

    skip = set(config_store.PLACEMENT_KEYS)
    keys = sorted(a.keys() | b.keys())
    diffs = [k for k in keys if k not in skip and a.get(k) != b.get(k)]
    n_cmp = len(keys) - len(skip)
    print(f"比べたキー: {len(keys)} 個（置き場所のキー {len(skip)} 個を除く {n_cmp} 個）")

    problems: list[str] = []
    # 置き場所のキーの確認
    models = (args.vehicles_dir / args.vehicle_id / "models").resolve()
    data = (args.vehicles_dir / args.vehicle_id / "data").resolve()
    if Path(ours.feedforward.model_path).parent != models:
        problems.append("model_path が車両の models/ の下ではありません")
    if Path(ours.feedforward.model_path).name != Path(theirs.feedforward.model_path).name:
        problems.append("model_path のファイル名が tests と違います")
    if not Path(ours.feedforward.model_path).exists():
        problems.append("model_path の pkl が車両の models/ にありません")
    if ours.vehicle.road_load_file is not None:
        problems.append("vehicle.road_load_file が無効になっていません")
    if Path(ours.output.results_dir) != data:
        problems.append("output.results_dir が車両の data/ ではありません")
    if ours.output.plot is not False:
        problems.append("output.plot が false ではありません")
    if ours.source_path != merged_path:
        problems.append("source_path が合成 yaml ではありません")

    # 検証の結果も同じか（tests は同じ model_path 以外の検証）
    v_theirs = tests_config.validate_config(theirs)
    v_ours = config_store.validate_config(ours)
    if v_theirs != v_ours:
        problems.append(f"validate_config の結果が違います: tests {v_theirs} / src {v_ours}")
    if theirs.stop_brake_opening_pct != ours.stop_brake_opening_pct:
        problems.append("stop_brake_opening_pct が違います")

    print(f"置き場所のキーの確認: {'OK' if not problems else 'NG'}")
    for p in problems:
        print(f"  NG: {p}")
    print(f"不一致のキー: {len(diffs)} 個")
    for k in diffs:
        print(f"  不一致: {k}（値は表示しない）")
    ok = not diffs and not problems
    print("等価確認①: 一致" if ok else "等価確認①: 不一致あり")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
