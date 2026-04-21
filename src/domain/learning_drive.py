import pickle
from dataclasses import dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Protocol

import numpy as np
from scipy.interpolate import NearestNDInterpolator, griddata

from src.models.calibration import CalibrationData
from src.models.learning_drive import LearningLog, LearningPattern
from src.models.profile import VehicleProfile

SPEED_STEP_KMH: float = 10.0
ACCEL_STEP_KMHS: float = 1.0
ACCEL_MAX_KMHS: float = 10.0
HOLD_DURATION_S: float = 2.0
SPEED_SAMPLE_INTERVAL_S: float = 0.1
MIN_LOGS_FOR_TRAINING: int = 4

G_TO_KMHS: float = 9.81 * 3.6


def _fill_nan_nearest(
    map_: np.ndarray, grid_speed: np.ndarray, grid_accel: np.ndarray
) -> np.ndarray:
    """NaN セルを最近傍の既知値で補完する。全セル NaN の場合は 0 埋め。"""
    known = ~np.isnan(map_)
    if not np.any(known):
        return np.zeros_like(map_)
    nn = NearestNDInterpolator(
        np.column_stack([grid_speed[known], grid_accel[known]]),
        map_[known],
    )
    result = map_.copy()
    nan_mask = np.isnan(result)
    result[nan_mask] = nn(grid_speed[nan_mask], grid_accel[nan_mask])
    return result


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
    min_logs_for_training: int = field(default=MIN_LOGS_FOR_TRAINING)


class LearningDriveManager:
    """学習パターンの生成・走行実行・運転モデル学習を担うドメインクラス。"""

    _config: LearningDriveConfig

    def __init__(self, config: LearningDriveConfig | None = None) -> None:
        self._config = config if config is not None else LearningDriveConfig()

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

    def train_model(
        self,
        logs: list[LearningLog],
        profile_id: str,
        output_dir: str = "data/models",
    ) -> str:
        """収集ログから pkl モデルを構築・保存してファイルパスを返す。

        Raises:
            LearningDataError: ログが不足しモデル構築できない場合。
        """
        if len(logs) < self._config.min_logs_for_training:
            raise LearningDataError(
                f"ログが不足しています ({len(logs)} 点)。"
                f"モデル構築には最低 {self._config.min_logs_for_training} 点必要です。"
            )

        speed_vals = np.array([log.pattern.speed_kmh for log in logs])
        accel_vals = np.array([log.pattern.accel_kmhs for log in logs])
        accel_openings = np.array([log.accel_opening_applied for log in logs])
        brake_openings = np.array([log.brake_opening_applied for log in logs])

        speed_grid = np.unique(speed_vals)
        accel_grid = np.unique(accel_vals)

        if len(speed_grid) < 2 or len(accel_grid) < 2:
            raise LearningDataError(
                "グリッドが不十分です。速度・加速度それぞれ 2 点以上のログが必要です。"
            )

        grid_speed, grid_accel = np.meshgrid(speed_grid, accel_grid, indexing="ij")
        points = np.column_stack([speed_vals, accel_vals])

        accel_map = griddata(points, accel_openings, (grid_speed, grid_accel), method="linear")
        brake_map = griddata(points, brake_openings, (grid_speed, grid_accel), method="linear")

        # NaN（フィルタアウトされたグリッド点）を最近傍値で補完する。
        # 0 埋めだと FF コントローラーが誤った開度を出力するため最近傍補間を使う。
        accel_map = _fill_nan_nearest(accel_map, grid_speed, grid_accel)
        brake_map = _fill_nan_nearest(brake_map, grid_speed, grid_accel)

        safe_profile_id = Path(profile_id).name
        timestamp = datetime.now(tz=UTC).strftime("%Y%m%d_%H%M%S")
        filename = f"{safe_profile_id}_{timestamp}.pkl"

        out_dir = Path(output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        pkl_path = out_dir / filename

        model_data = {
            "speed_grid": speed_grid,
            "accel_grid": accel_grid,
            "accel_map": accel_map,
            "brake_map": brake_map,
        }
        with pkl_path.open("wb") as f:
            pickle.dump(model_data, f)

        return str(pkl_path)
