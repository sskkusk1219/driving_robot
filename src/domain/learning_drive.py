"""学習運転の開度パターン生成を担うドメインモジュール。

Phase 8（学習運転）の方針に従い、固定の開度パターンを開ループで順に実行するための
パターン列を生成する。実際の走行・ログ記録・安全監視は LearningLoop が担う。
"""

from dataclasses import dataclass, field

from src.models.learning_drive import LearningPattern, PatternKind
from src.models.profile import VehicleProfile

# 開度パターン生成の既定パラメータ
CREEP_RELEASE_STEPS: int = 5  # 停車保持→0 へブレーキを緩める段数
HOLD_DURATION_S: float = 3.0  # 各パターンの最大保持時間（プラトー/上限未達時の打ち切り）

# 全車速域 加速スイープ（ACCEL_SWEEP）: max_accel_opening に対する割合で数段振る。
# 低開度は低速プラトー、高開度は cap（0.9×max_speed）到達まで加速し、速度全域 × 加速率を採取する。
ACCEL_SWEEP_FRACS: tuple[float, ...] = (0.3, 0.5, 0.7, 1.0)
# ACCEL_SWEEP 後に停車へ戻すリセットブレーキ開度（中庸＝過G にならず素早く停車）
ACCEL_SWEEP_RESET_BRAKE_PCT: float = 30.0

# 定常ブレーキ計測（BRAKE_HOLD）: 高速まで上げてから保持するブレーキ開度を数段スイープ。
BRAKE_HOLD_OPENINGS_PCT: tuple[float, ...] = (10.0, 20.0, 30.0, 40.0)
# BRAKE_HOLD で cap まで上げる加速開度（ガバナで 0.9×上限G に収束。確実に cap へ届く高開度）
BRAKE_HOLD_ACCEL_PCT: float = 70.0

COAST_DOWN_COUNT: int = 3  # 専用コーストダウン本数（速度全域の減速カーブ計測）
COAST_DOWN_ACCEL_PCT: float = 70.0  # コーストダウンの加速アクセル開度（cap まで上げる標準値）


class LearningDataError(Exception):
    """ログが不足・不正でモデル構築できない場合に送出。"""


@dataclass
class LearningDriveConfig:
    creep_release_steps: int = field(default=CREEP_RELEASE_STEPS)
    hold_duration_s: float = field(default=HOLD_DURATION_S)
    accel_sweep_fracs: tuple[float, ...] = field(default=ACCEL_SWEEP_FRACS)
    accel_sweep_reset_brake_pct: float = field(default=ACCEL_SWEEP_RESET_BRAKE_PCT)
    brake_hold_openings_pct: tuple[float, ...] = field(default=BRAKE_HOLD_OPENINGS_PCT)
    brake_hold_accel_pct: float = field(default=BRAKE_HOLD_ACCEL_PCT)
    coast_down_count: int = field(default=COAST_DOWN_COUNT)
    coast_down_accel_pct: float = field(default=COAST_DOWN_ACCEL_PCT)


class LearningDriveManager:
    """学習運転の開度パターン列を生成するドメインクラス。"""

    _config: LearningDriveConfig

    def __init__(self, config: LearningDriveConfig | None = None) -> None:
        self._config = config if config is not None else LearningDriveConfig()

    def generate_patterns(self, profile: VehicleProfile) -> list[LearningPattern]:
        """車両プロファイルから開度パターン列を生成する。

        構成（連続軌跡として LearningLoop が順に実行する）:
          1. クリープ解放: 停車保持ブレーキ（`stop_brake_opening_pct`）から段階的にブレーキを緩める
          2. クリープ安定待ち: accel=brake=0 で車速が安定するまで待機（クリープ車速・加速率を計測）
          3. 全車速域 加速スイープ（ACCEL_SWEEP）数段: 固定アクセル開度で 0→0.9×max_speed（cap）
             まで加速し、速度全域 × 異なる加速率の加速サンプルを採る。各段は cap 到達/タイムアウト
             後にブレーキで停車へ戻し、次段の起点を 0 に揃える（「踏む→戻す→低速キープ」を解消）。
          4. 定常ブレーキ計測（BRAKE_HOLD）数段: 高速（cap）まで加速→固定ブレーキ開度を一定保持して
             定常減速を記録する（加速側プラトー保持と対称）。ブレーキ開度を数段スイープする。
          5. コーストダウン（COAST_DOWN）数本: アクセルで加速→ブレーキ無しで低速まで惰行し、
             エンジンブレーキ＋走行抵抗の減速率を速度全域で計測する

        いずれも車両プロファイルの最大開度を超えないようにスケール・クランプする。
        """
        hold = self._config.hold_duration_s
        patterns: list[LearningPattern] = []

        # 1. クリープ解放（停車保持ブレーキを段階的に緩める。0 は次の安定待ちが担うため > 0 まで）
        stop_brake = min(
            profile.feedforward_params.stop_brake_opening_pct, profile.max_brake_opening
        )
        steps = max(self._config.creep_release_steps, 1)
        for i in range(steps, 0, -1):
            patterns.append(
                LearningPattern(
                    kind=PatternKind.CREEP,
                    accel_opening=0.0,
                    brake_opening=stop_brake * (i / steps),
                    hold_duration_s=hold,
                )
            )
        # 2. クリープ安定待ち（accel=brake=0 で車速が安定するまで保持。hold は LearningLoop 側の
        #    creep_settle_* で制御するためここでは便宜上の値）
        patterns.append(
            LearningPattern(
                kind=PatternKind.CREEP_SETTLE,
                accel_opening=0.0,
                brake_opening=0.0,
                hold_duration_s=hold,
            )
        )

        # 3. 全車速域 加速スイープ（固定開度で cap まで加速 → リセットブレーキで停車復帰）
        reset_brake = min(self._config.accel_sweep_reset_brake_pct, profile.max_brake_opening)
        for frac in self._config.accel_sweep_fracs:
            accel = min(max(frac, 0.0) * profile.max_accel_opening, profile.max_accel_opening)
            if accel <= 0.0:
                continue
            patterns.append(
                LearningPattern(
                    kind=PatternKind.ACCEL_SWEEP,
                    accel_opening=accel,
                    brake_opening=reset_brake,
                    hold_duration_s=hold,
                )
            )

        # 4. 定常ブレーキ計測（cap まで加速 → 固定ブレーキ開度を一定保持して定常減速を記録）
        brake_accel = min(self._config.brake_hold_accel_pct, profile.max_accel_opening)
        if brake_accel > 0.0:
            for brake_pct in self._config.brake_hold_openings_pct:
                brake = min(max(brake_pct, 0.0), profile.max_brake_opening)
                if brake <= 0.0:
                    continue
                patterns.append(
                    LearningPattern(
                        kind=PatternKind.BRAKE_HOLD,
                        accel_opening=brake_accel,
                        brake_opening=brake,
                        hold_duration_s=hold,
                    )
                )

        # 5. コーストダウン（高速まで加速→ブレーキ無しで惰行。減速率 vs 速度の全域カーブ）
        coast_accel = min(self._config.coast_down_accel_pct, profile.max_accel_opening)
        if coast_accel > 0.0:
            for _ in range(max(self._config.coast_down_count, 0)):
                patterns.append(
                    LearningPattern(
                        kind=PatternKind.COAST_DOWN,
                        accel_opening=coast_accel,
                        brake_opening=0.0,
                        hold_duration_s=hold,
                    )
                )

        return patterns
