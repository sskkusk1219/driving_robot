"""既存の走行 CSV からモデル・カーブだけを作り直すオフライン入口（段2.5。ProblemReport_20260916）。

手順2 を実機で走り直すと不感帯・クリープ速度（2-0 の実測）も同時に変わってしまい、惰行カーブ
だけを直したいときに 1 変数比較にならない。既存の手順2 ログ（`results/drive_log_real_*.csv`）
をそのまま使い、`pattern_drive.build_ff_model` を呼ぶだけの薄いエントリポイント。

`pedal=None` で呼ぶため、不感帯・停車保持開度・クリープ平衡車速（`pattern_drive.MEASURED_KEYS`）は
書き戻されず、`build_vehicle_profile(cfg)` が設定ファイルの 2-0 実測値をそのまま使う。

使い方:
    .venv/bin/python -m tests.research.relearn <csv> --dry-run   # 差分表示だけ（設定不変更）
    .venv/bin/python -m tests.research.relearn <csv>             # config_testVehicle.yaml へ保存
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from src.domain.learning_drive import LearningDataError
from tests.research.config import DEFAULT_CONFIG_PATH, ConfigError, load_config
from tests.research.hardware import HW_REAL
from tests.research.pattern_drive import build_ff_model


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="既存の走行 CSV からモデル・物理定数（惰行カーブ等）を作り直す（実機不要）"
    )
    parser.add_argument("csv", type=Path, help="走行ログ CSV（tests.research.drive_log 形式）")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="config_testVehicle.yaml へ保存せず、推定値の表示だけで終える",
    )
    args = parser.parse_args(argv)

    try:
        cfg = load_config(DEFAULT_CONFIG_PATH)
        build_ff_model(
            cfg, args.csv, hw_mode=HW_REAL, pedal=None, write_config=not args.dry_run,
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
