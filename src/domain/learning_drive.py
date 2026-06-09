from dataclasses import dataclass, field
from datetime import UTC, datetime
from typing import Protocol
from uuid import uuid4

import numpy as np

from src.models.calibration import CalibrationData
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.learning_drive import LearningLog, LearningPattern
from src.models.profile import VehicleProfile

SPEED_STEP_KMH: float = 10.0
ACCEL_STEP_KMHS: float = 1.0
ACCEL_MAX_KMHS: float = 10.0
HOLD_DURATION_S: float = 2.0
SPEED_SAMPLE_INTERVAL_S: float = 0.1

G_TO_KMHS: float = 9.81 * 3.6

# 学習用基準速度プロファイル生成パラメータ
# 各目標速度への加減速を複数レートで網羅し、逆モデル学習に必要な特徴量空間を広く覆う。
LEARNING_DWELL_S: float = 1.5  # 各サイクル間の停車保持時間
LEARNING_ACCEL_RATE_FRACTIONS: tuple[float, ...] = (0.4, 0.7, 1.0)  # accel_max_kmhs に対する割合
LEARNING_DECEL_RATE_FRACTIONS: tuple[float, ...] = (0.4, 0.7, 1.0)  # 最大減速度に対する割合


class LearningActuatorProtocol(Protocol):
    async def move_to_position(self, pos: int) -> None: ...

    async def read_position(self) -> int: ...


class LearningCANProtocol(Protocol):
    async def read_speed(self) -> float: ...


class LearningDataError(Exception):
    """ログが不足・不正でモデル構築できない場合に送出。"""


@dataclass
class LearningDriveConfig:
    speed_step_kmh: float = field(default=SPEED_STEP_KMH)
    accel_step_kmhs: float = field(default=ACCEL_STEP_KMHS)
    accel_max_kmhs: float = field(default=ACCEL_MAX_KMHS)
    hold_duration_s: float = field(default=HOLD_DURATION_S)
    speed_sample_interval_s: float = field(default=SPEED_SAMPLE_INTERVAL_S)


class LearningDriveManager:
    """学習パターンの生成・走行実行・運転モデル学習を担うドメインクラス。"""

    _config: LearningDriveConfig

    def __init__(self, config: LearningDriveConfig | None = None) -> None:
        self._config = config if config is not None else LearningDriveConfig()

    def build_learning_reference(self, profile: VehicleProfile) -> DrivingMode:
        """学習用の連続基準速度プロファイル（DrivingMode）を生成する。

        速度ステップごとに「0→目標速度→保持→0」のサイクルを構成し、加速・減速の
        各レートを複数（LEARNING_*_RATE_FRACTIONS）切り替えることで、先読み逆モデルが
        必要とする速度×加減速トレンドの特徴量空間を広く網羅する。

        生成した DrivingMode は永続化されない一時的なもので、走行セッションの mode_id は
        None のままとする（drive_sessions.mode_id は driving_modes への FK のため）。
        """
        max_decel_kmhs = max(profile.max_decel_g * G_TO_KMHS, 1.0)
        accel_max = max(self._config.accel_max_kmhs, 1.0)

        targets = np.arange(
            self._config.speed_step_kmh,
            profile.max_speed + self._config.speed_step_kmh,
            self._config.speed_step_kmh,
        )
        targets = targets[targets <= profile.max_speed + 1e-9]

        points: list[SpeedPoint] = [SpeedPoint(time_s=0.0, speed_kmh=0.0)]
        t = 0.0
        for i, target in enumerate(targets):
            target_v = float(target)
            accel_rate = (
                accel_max * LEARNING_ACCEL_RATE_FRACTIONS[i % len(LEARNING_ACCEL_RATE_FRACTIONS)]
            )
            decel_rate = (
                max_decel_kmhs
                * LEARNING_DECEL_RATE_FRACTIONS[i % len(LEARNING_DECEL_RATE_FRACTIONS)]
            )

            # 0 → 目標速度（加速）
            t += target_v / accel_rate
            points.append(SpeedPoint(time_s=t, speed_kmh=target_v))
            # 目標速度を保持
            t += self._config.hold_duration_s
            points.append(SpeedPoint(time_s=t, speed_kmh=target_v))
            # 目標速度 → 0（減速）
            t += target_v / decel_rate
            points.append(SpeedPoint(time_s=t, speed_kmh=0.0))
            # 停車保持
            t += LEARNING_DWELL_S
            points.append(SpeedPoint(time_s=t, speed_kmh=0.0))

        return DrivingMode(
            id=f"learning-{uuid4()}",
            name="learning-reference",
            description="学習走行用に自動生成した基準速度プロファイル（非永続）",
            reference_speed=points,
            total_duration=t,
            max_speed=profile.max_speed,
            created_at=datetime.now(tz=UTC),
        )

    def generate_patterns(self, profile: VehicleProfile) -> list[LearningPattern]:
        """max_opening / max_decel_g を超えるパターンを除外した学習パターンリストを返す。"""
        max_decel_kmhs = profile.max_decel_g * G_TO_KMHS

        speed_points = np.arange(
            self._config.speed_step_kmh,
            profile.max_speed + self._config.speed_step_kmh,
            self._config.speed_step_kmh,
        )
        accel_points = np.arange(
            -max_decel_kmhs,
            self._config.accel_max_kmhs + self._config.accel_step_kmhs,
            self._config.accel_step_kmhs,
        )

        patterns: list[LearningPattern] = []
        for speed in speed_points:
            for accel in accel_points:
                accel_opening, brake_opening = self._compute_initial_opening(
                    float(speed), float(accel), profile
                )
                if accel_opening > profile.max_accel_opening:
                    continue
                if brake_opening > profile.max_brake_opening:
                    continue
                if accel < 0 and abs(accel) > max_decel_kmhs + 1e-9:
                    continue
                patterns.append(
                    LearningPattern(
                        speed_kmh=float(speed),
                        accel_kmhs=float(accel),
                        accel_opening=accel_opening,
                        brake_opening=brake_opening,
                        hold_duration_s=self._config.hold_duration_s,
                    )
                )
        return patterns

    def _compute_initial_opening(
        self, speed_kmh: float, accel_kmhs: float, profile: VehicleProfile
    ) -> tuple[float, float]:
        """速度・加速度から初期開度を線形マッピングで計算する。"""
        if accel_kmhs >= 0:
            ratio = min(1.0, speed_kmh / max(profile.max_speed, 1.0))
            accel_ratio = min(1.0, accel_kmhs / max(self._config.accel_max_kmhs, 1.0))
            accel_opening = profile.max_accel_opening * (ratio * 0.5 + accel_ratio * 0.5)
            brake_opening = 0.0
        else:
            decel_ratio = min(1.0, abs(accel_kmhs) / max(profile.max_decel_g * G_TO_KMHS, 1.0))
            accel_opening = 0.0
            brake_opening = profile.max_brake_opening * decel_ratio
        return accel_opening, brake_opening

    async def run_pattern(
        self,
        pattern: LearningPattern,
        accel_driver: LearningActuatorProtocol,
        brake_driver: LearningActuatorProtocol,
        can_reader: LearningCANProtocol,
        calibration: CalibrationData,
    ) -> LearningLog:
        """パターンの開度指令を送信し、実車速を記録して LearningLog を返す。"""
        import asyncio

        accel_pulse = self._opening_to_pulse(
            pattern.accel_opening, calibration.accel_zero_pos, calibration.accel_stroke
        )
        brake_pulse = self._opening_to_pulse(
            pattern.brake_opening, calibration.brake_zero_pos, calibration.brake_stroke
        )

        await asyncio.gather(
            accel_driver.move_to_position(accel_pulse),
            brake_driver.move_to_position(brake_pulse),
        )

        speed_samples: list[float] = []
        elapsed = 0.0
        while elapsed < pattern.hold_duration_s:
            await asyncio.sleep(self._config.speed_sample_interval_s)
            speed_samples.append(await can_reader.read_speed())
            elapsed += self._config.speed_sample_interval_s

        actual_speed = sum(speed_samples) / len(speed_samples) if speed_samples else 0.0

        return LearningLog(
            pattern=pattern,
            actual_speed_kmh=actual_speed,
            accel_opening_applied=pattern.accel_opening,
            brake_opening_applied=pattern.brake_opening,
            recorded_at=datetime.now(tz=UTC),
        )

    def _opening_to_pulse(self, opening_pct: float, zero_pos: int, stroke: int) -> int:
        """開度 [%] をアクチュエータ位置 [pulse] に換算する。"""
        return zero_pos + int(opening_pct / 100.0 * stroke)
