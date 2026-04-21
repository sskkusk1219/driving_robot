import asyncio
from collections import deque
from dataclasses import dataclass, field
from datetime import UTC, datetime
from typing import Protocol

from src.models.calibration import CalibrationData, CalibrationResult, ValidationResult

CALIB_MOVE_STEP_PULSE = 50
CALIB_STEP_INTERVAL_S = 0.05
CALIB_CURRENT_WINDOW = 5
CALIB_CURRENT_SPIKE_RATIO = 1.5
CALIB_MIN_STROKE_PULSE = 1000
CALIB_MAX_STROKE_PULSE = 10000
CALIB_MAX_SEARCH_PULSE = 50000


class CalibrationActuatorProtocol(Protocol):
    async def home_return(self) -> None: ...

    async def move_to_position(self, pos: int) -> None: ...

    async def read_position(self) -> int: ...

    async def read_current(self) -> float: ...


@dataclass
class CalibrationConfig:
    move_step_pulse: int = field(default=CALIB_MOVE_STEP_PULSE)
    step_interval_s: float = field(default=CALIB_STEP_INTERVAL_S)
    current_window: int = field(default=CALIB_CURRENT_WINDOW)
    current_spike_ratio: float = field(default=CALIB_CURRENT_SPIKE_RATIO)
    min_stroke_pulse: int = field(default=CALIB_MIN_STROKE_PULSE)
    max_stroke_pulse: int = field(default=CALIB_MAX_STROKE_PULSE)
    max_search_pulse: int = field(default=CALIB_MAX_SEARCH_PULSE)


class CalibrationDetectionError(Exception):
    """接触点またはフル位置を検出できなかった場合に送出。"""


class CalibrationManager:
    """アクセル・ブレーキのゼロフルキャリブレーションを実行するドメインクラス。"""

    _accel_driver: CalibrationActuatorProtocol
    _brake_driver: CalibrationActuatorProtocol
    _config: CalibrationConfig

    def __init__(
        self,
        accel_driver: CalibrationActuatorProtocol,
        brake_driver: CalibrationActuatorProtocol,
        config: CalibrationConfig | None = None,
    ) -> None:
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._config = config if config is not None else CalibrationConfig()

    async def run_calibration(self, profile_id: str) -> CalibrationResult:  # noqa: ARG002
        """両軸のゼロフルキャリブレーションを順番に実行し、バリデーション済みの結果を返す。

        TODO: バリデーション成功後に profile_id のプロファイルへ data を永続化する。
              ProfileRepository Protocol を注入して CalibrationData を保存することで
              走行前チェック #3（有効なキャリブレーションデータあり）が通るようになる。
        """
        try:
            accel_zero = await self._detect_zero(self._accel_driver)
            accel_full = await self._detect_full(self._accel_driver, accel_zero)
            await self._accel_driver.home_return()

            brake_zero = await self._detect_zero(self._brake_driver)
            brake_full = await self._detect_full(self._brake_driver, brake_zero)
            await self._brake_driver.home_return()
        except CalibrationDetectionError as e:
            return CalibrationResult(success=False, data=None, error_message=str(e))

        data = CalibrationData(
            accel_zero_pos=accel_zero,
            accel_full_pos=accel_full,
            accel_stroke=accel_full - accel_zero,
            brake_zero_pos=brake_zero,
            brake_full_pos=brake_full,
            brake_stroke=brake_full - brake_zero,
            calibrated_at=datetime.now(tz=UTC),
            is_valid=False,
        )
        validation = self._validate(data)
        data.is_valid = validation.is_valid
        return CalibrationResult(
            success=validation.is_valid,
            data=data,
            error_message=None if validation.is_valid else validation.error_message,
        )

    async def _detect_zero(self, driver: CalibrationActuatorProtocol) -> int:
        """原点から正方向に移動し、電流急増で接触点(zero)を検出する。"""
        await driver.home_return()
        return await self._probe_contact(driver, start_pos=0)

    async def _detect_full(self, driver: CalibrationActuatorProtocol, zero_pos: int) -> int:
        """接触点(zero_pos)から正方向にさらに移動し、電流急増でフル位置を検出する。"""
        await driver.move_to_position(zero_pos)
        await asyncio.sleep(self._config.step_interval_s)
        return await self._probe_contact(driver, start_pos=zero_pos)

    async def _probe_contact(self, driver: CalibrationActuatorProtocol, start_pos: int) -> int:
        """start_pos から正方向にステップ移動し、電流スパイクで接触位置を返す。

        スパイク判定: current > moving_avg + baseline * spike_ratio
        baseline は最初のウィンドウ満杯時点の移動平均（自由移動中の基準電流）。
        """
        window: deque[float] = deque(maxlen=self._config.current_window)
        baseline: float | None = None
        pos = start_pos

        while pos - start_pos < self._config.max_search_pulse:
            pos += self._config.move_step_pulse
            await driver.move_to_position(pos)
            await asyncio.sleep(self._config.step_interval_s)
            current = await driver.read_current()
            window.append(current)

            if len(window) < self._config.current_window:
                continue

            avg = sum(window) / len(window)

            if baseline is None:
                # baseline = 最初のウィンドウ平均（自由移動中の基準電流）として確定する。
                # このフレームはまだ接触前の安定した値を記録しているため判定をスキップし、
                # 次のフレーム以降でスパイク判定を開始する（保守的な設計）。
                baseline = avg
                continue

            if current > avg + baseline * self._config.current_spike_ratio:
                return await driver.read_position()

        raise CalibrationDetectionError(
            f"接触点を検出できませんでした (最大探索距離 {self._config.max_search_pulse} pulse)"
        )

    def _validate(self, data: CalibrationData) -> ValidationResult:
        """接触点 < フル位置、ストローク妥当性を検証する。"""
        if data.accel_full_pos <= data.accel_zero_pos:
            return ValidationResult(
                is_valid=False,
                error_message=(
                    f"アクセル: フル位置({data.accel_full_pos}) が"
                    f"接触点({data.accel_zero_pos}) 以下"
                ),
            )
        if data.brake_full_pos <= data.brake_zero_pos:
            return ValidationResult(
                is_valid=False,
                error_message=(
                    f"ブレーキ: フル位置({data.brake_full_pos}) が"
                    f"接触点({data.brake_zero_pos}) 以下"
                ),
            )
        if not (
            self._config.min_stroke_pulse <= data.accel_stroke <= self._config.max_stroke_pulse
        ):
            return ValidationResult(
                is_valid=False,
                error_message=(
                    f"アクセルストローク({data.accel_stroke} pulse) が"
                    f"範囲外 [{self._config.min_stroke_pulse}, {self._config.max_stroke_pulse}]"
                ),
            )
        if not (
            self._config.min_stroke_pulse <= data.brake_stroke <= self._config.max_stroke_pulse
        ):
            return ValidationResult(
                is_valid=False,
                error_message=(
                    f"ブレーキストローク({data.brake_stroke} pulse) が"
                    f"範囲外 [{self._config.min_stroke_pulse}, {self._config.max_stroke_pulse}]"
                ),
            )
        return ValidationResult(is_valid=True, error_message=None)
