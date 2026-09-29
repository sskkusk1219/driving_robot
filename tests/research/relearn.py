"""既存の走行 CSV からモデル・カーブだけを作り直すオフライン入口（段2.5。ProblemReport_20260916）。

手順2 を実機で走り直すと不感帯・クリープ速度（2-0 の実測）も同時に変わってしまい、惰行カーブ
だけを直したいときに 1 変数比較にならない。既存の手順2 ログ（`results/drive_log_real_*.csv`）
をそのまま使い、`pattern_drive.build_ff_model` を呼ぶだけの薄いエントリポイント。

`pedal=None` で呼ぶため、不感帯・停車保持開度・クリープ平衡車速（`pattern_drive.MEASURED_KEYS`）は
書き戻されず、`build_vehicle_profile(cfg)` が設定ファイルの 2-0 実測値をそのまま使う。

使い方:
    .venv/bin/python -m tests.research.relearn <csv> --dry-run   # 差分表示だけ（設定不変更）
    .venv/bin/python -m tests.research.relearn <csv>             # config_testVehicle.yaml へ保存

段2（ProblemReport_20260925。学習サンプルの WLTP 重み付け）:
    .venv/bin/python -m tests.research.relearn <csv> --dry-run --ref-csv <基準車速を持つCSV> \\
        --weight on     # config を変えずに重み付けを試す（DB 不要）
    --weight off でも --ref-csv を渡せば mae_wltp は出る（重みなし／ありを同じ物差しで比較できる）。
    --ref-csv 省略時、重みが要る（--weight on、または config の sample_weight_enabled）なら
    DB の modes.wltp_mode_name を読む。
"""

from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

from tests.research.config import DEFAULT_CONFIG_PATH, ConfigError, load_config
from tests.research.hardware import HW_REAL
from tests.research.learning_patterns import LearningDataError
from tests.research.mode_drive import load_mode
from tests.research.pattern_drive import build_ff_model
from tests.research.research_types import DrivingMode
from tests.research.wltp_grid import mode_from_csv


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="既存の走行 CSV からモデル・物理定数（惰行カーブ等）を作り直す（実機不要）"
    )
    parser.add_argument("csv", type=Path, help="走行ログ CSV（tests.research.drive_log 形式）")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="config_testVehicle.yaml へ保存せず、推定値の表示だけで終える",
    )
    parser.add_argument(
        "--ref-csv", type=Path, default=None,
        help="WLTP の基準車速を持つ CSV（省略時、重みが要れば DB の modes.wltp_mode_name を読む）",
    )
    parser.add_argument(
        "--weight", choices=("config", "on", "off"), default="config",
        help="WLTP 重み付けの有効/無効を一時的に上書きする（config は変更しない。既定 config）",
    )
    args = parser.parse_args(argv)

    try:
        cfg = load_config(DEFAULT_CONFIG_PATH)
        sample_weight_enabled = {"on": True, "off": False, "config": None}[args.weight]
        enabled = (
            cfg.learning.sample_weight_enabled
            if sample_weight_enabled is None
            else sample_weight_enabled
        )
        # WLTP は「重みが有効」または「--ref-csv 指定」のどちらかで読む（off でも ref があれば
        # mae_wltp を出して比較できるように）
        wltp_mode: DrivingMode | None = None
        if args.ref_csv is not None:
            wltp_mode = mode_from_csv(args.ref_csv)
        elif enabled:
            wltp_mode = asyncio.run(load_mode(cfg, cfg.modes.wltp_mode_name))
        build_ff_model(
            cfg, args.csv, hw_mode=HW_REAL, pedal=None, write_config=not args.dry_run,
            wltp_mode=wltp_mode, sample_weight_enabled=sample_weight_enabled,
        )
    except ConfigError as exc:
        print(f"設定エラー: {exc}")
        return 2
    except LearningDataError as exc:
        print(f"モデル作成エラー: {exc}")
        return 6
    return 0


if __name__ == "__main__":
    sys.exit(main())
