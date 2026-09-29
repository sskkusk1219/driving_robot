"""学習運転（手順2）のパターンに使う定数と、学習データ不足の例外。

もとは本番 `src/domain/learning_drive.py` の移植（`generate_patterns`）だったが、
2026-09-25 段4（ProblemReport_20260925）で手順2 のパターン列を格子ステップ走行に置き換えたため、
パターン生成クラスは削除した。残しているのは、新しいパターン列
（`pattern_drive.build_patterns`）が使う定数と、`LearningDataError`（元は同ファイルで定義）だけ。
"""

from __future__ import annotations

HOLD_DURATION_S: float = 3.0  # 各パターンの最大保持時間（プラトー/上限未達時の打ち切り）

# 専用コーストダウン本数（速度全域の減速カーブ計測）。cap→5km/h を完走する（coast_timeout_s=90）
# ため 1 本 40〜85s かかる。惰行減速カーブ同定には 2 本で十分なため 3→2 に減らし時間を相殺。
# 2026-09-27 段7a（ProblemReport_20260925 段7）: 実測（065502）で 1 本目・2 本目の惰行 a が
# ±0.07 km/h/s 以内に一致し、格子ステップが惰行の代わりにコーストダウンの実測を使うようになった
# （grid_planner.GridPlanner の coast_fn）ため、2→1 に減らして手順2 を約 90s 短縮する。
COAST_DOWN_COUNT: int = 1
COAST_DOWN_ACCEL_PCT: float = 70.0  # コーストダウンの加速の頭打ち開度（加速は目標 G で刻み踏み）


class LearningDataError(Exception):
    """ログが不足・不正でモデル構築できない場合に送出。"""
