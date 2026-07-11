from dataclasses import dataclass
from enum import StrEnum


class PatternKind(StrEnum):
    """開度パターンの種別。LearningLoop が実行時の前進条件を決めるのに使う。"""

    CREEP = "creep"  # 停車保持から段階的にブレーキを緩める（解放ステップ）
    CREEP_SETTLE = "creep_settle"  # アクセル・ブレーキ 0% で車速が安定するまで待機しクリープ計測
    # 低いアクセル開度（不感帯が疑われる域）を無ランプで一定時間保持し、開度→応答曲線から
    # アクセル不感帯（accel_deadband_pct）を推定するためのサンプルを採る。
    ACCEL_DEADBAND_PROBE = "accel_deadband_probe"
    # 固定アクセル開度で 0→0.98×max_speed（cap）まで加速し全車速域の加速サンプルを採る。
    # cap 到達/タイムアウト後はブレーキで停車まで戻す（次パターンの起点を 0 に揃える）。
    ACCEL_SWEEP = "accel_sweep"
    # 高速（cap）まで加速→固定ブレーキ開度を一定保持し定常減速を採る（加速プラトーと対称）。
    BRAKE_HOLD = "brake_hold"
    # アクセルで加速→ブレーキ無しで低速まで惰行（エンジンブレーキ減速率を計測）
    COAST_DOWN = "coast_down"
    # 高速（cap）まで加速→アクセルを微小開度（trim_opening）へ落として一定保持し、高速域×
    # 微小開度の応答（惰性減速しつつ）を採る。WLTP 巡航帯（125-140km/h×1〜5%）のモデルデータ
    # 欠損を埋めるための専用パターン（B-7-2）。cap 超過は防ぐため速度が cap に戻れば離脱する。
    CRUISE_TRIM = "cruise_trim"


@dataclass(frozen=True)
class LearningPattern:
    """学習運転で開ループ実行する1つの固定開度パターン。

    開度 [%] はアクチュエータ位置への換算前の論理値。`hold_duration_s` は
    速度プラトーや上限に達しない場合に当該パターンを打ち切る最大保持時間。
    ACCEL_SWEEP は accel_opening（加速の目標）と brake_opening（停車復帰のリセットブレーキ）、
    BRAKE_HOLD は accel_opening（cap まで上げる加速）と brake_opening（保持する定常ブレーキ）。
    CRUISE_TRIM は accel_opening（cap まで上げる加速）と trim_opening（cap 到達後に保持する
    微小アクセル開度）を使う。
    """

    kind: PatternKind
    accel_opening: float
    brake_opening: float
    hold_duration_s: float
    # CRUISE_TRIM 専用: cap 到達後に保持する微小アクセル開度 [%]（他パターンでは未使用で 0.0）。
    trim_opening: float = 0.0
