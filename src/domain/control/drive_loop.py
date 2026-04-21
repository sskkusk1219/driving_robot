"""50ms 制御ループ。FF+PID 制御・安全チェック・ログ記録を担う。"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Awaitable, Callable
from typing import Protocol

from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pid import PIDController
from src.models.drive_log import DriveLogData
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile

_logger = logging.getLogger(__name__)

CONTROL_LOOP_INTERVAL_S: float = 0.05
LOG_EVERY_N_CYCLES: int = 2


class ActuatorDriverProtocol(Protocol):
    async def move_to_position(self, pos: int) -> None: ...

    async def read_current(self) -> float: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


class SafetyCheckProtocol(Protocol):
    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...

    def check_deviation(self, ref: float, actual: float, duration: float) -> bool: ...


class LogWriterProtocol(Protocol):
    async def write_log(self, session_id: str, data: DriveLogData) -> None: ...


class DriveLoop:
    """50ms 制御ループを管理するドメインコンポーネント。

    start() でループを開始し、stop() または on_complete/on_emergency コールバックで停止する。
    asyncio.sleep を使わず call_later でスケジューリングすることでジッタを ±5ms 以内に抑制する。
    """

    _running: bool
    _started_at: float
    _cycle_count: int
    _deviation_start: float | None

    def __init__(
        self,
        ff_controller: FeedforwardController,
        pid: PIDController,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        profile: VehicleProfile,
        mode: DrivingMode,
        safety_check: SafetyCheckProtocol,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        log_writer: LogWriterProtocol | None = None,
        session_id: str | None = None,
    ) -> None:
        self._ff = ff_controller
        self._pid = pid
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._mode = mode
        self._safety_check = safety_check
        self._on_complete = on_complete
        self._on_emergency = on_emergency
        self._log_writer = log_writer
        self._session_id = session_id

        self._running = False
        self._started_at = 0.0
        self._cycle_count = 0
        self._deviation_start = None

    def start(self) -> None:
        """制御ループを開始する。既に実行中の場合は何もしない。"""
        if self._running:
            return
        self._running = True
        self._pid.reset()
        self._deviation_start = None
        self._cycle_count = 0
        loop = asyncio.get_running_loop()
        self._started_at = loop.time()
        loop.call_later(CONTROL_LOOP_INTERVAL_S, self._schedule_next_cycle)

    def stop(self) -> None:
        """制御ループを停止する。進行中のサイクルは完了させる。"""
        self._running = False

    @property
    def is_running(self) -> bool:
        return self._running

    def _schedule_next_cycle(self) -> None:
        if not self._running:
            return
        asyncio.ensure_future(self._execute_one_cycle())
        asyncio.get_running_loop().call_later(CONTROL_LOOP_INTERVAL_S, self._schedule_next_cycle)

    async def _execute_one_cycle(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        elapsed_s = loop.time() - self._started_at

        if elapsed_s >= self._mode.total_duration:
            self.stop()
            await self._on_complete()
            return

        ref_speed, ref_accel = self._get_ref_speed_and_accel(elapsed_s)

        try:
            actual_speed = await self._can_reader.read_speed()
        except Exception:
            _logger.exception("CAN 車速取得失敗: 緊急停止")
            self.stop()
            await self._on_emergency()
            return

        ff_accel, ff_brake = self._ff.predict(ref_speed, ref_accel)
        pid_correction = self._pid.update(ref_speed, actual_speed)

        raw_accel = ff_accel + max(0.0, pid_correction)
        raw_brake = ff_brake + max(0.0, -pid_correction)

        # アクセル優先排他制御（functional-design.md 記載の初期実装）
        if raw_accel > 0.0 and raw_brake > 0.0:
            raw_brake = 0.0

        accel_opening = max(0.0, min(self._profile.max_accel_opening, raw_accel))
        brake_opening = max(0.0, min(self._profile.max_brake_opening, raw_brake))

        calib = self._profile.calibration
        if calib is None:
            _logger.error("キャリブレーションデータがない: 緊急停止")
            self.stop()
            await self._on_emergency()
            return

        accel_pos = self._opening_to_position(
            accel_opening, calib.accel_zero_pos, calib.accel_full_pos
        )
        brake_pos = self._opening_to_position(
            brake_opening, calib.brake_zero_pos, calib.brake_full_pos
        )

        try:
            accel_current, brake_current = await asyncio.gather(
                self._drive_accel_axis(accel_pos),
                self._drive_brake_axis(brake_pos),
            )
        except Exception:
            _logger.exception("アクチュエータ通信失敗: 緊急停止")
            self.stop()
            await self._on_emergency()
            return

        if self._safety_check.check_overcurrent(accel_current, "accel"):
            _logger.warning("アクセル過電流: %.1f mA", accel_current)
            self.stop()
            await self._on_emergency()
            return

        if self._safety_check.check_overcurrent(brake_current, "brake"):
            _logger.warning("ブレーキ過電流: %.1f mA", brake_current)
            self.stop()
            await self._on_emergency()
            return

        deviation = abs(ref_speed - actual_speed)
        threshold = self._profile.stop_config.deviation_threshold_kmh
        if deviation > threshold:
            if self._deviation_start is None:
                self._deviation_start = loop.time()
            deviation_duration = loop.time() - self._deviation_start
        else:
            self._deviation_start = None
            deviation_duration = 0.0

        if self._safety_check.check_deviation(ref_speed, actual_speed, deviation_duration):
            _logger.warning(
                "走行逸脱: ref=%.1f actual=%.1f duration=%.1fs",
                ref_speed,
                actual_speed,
                deviation_duration,
            )
            self.stop()
            await self._on_emergency()
            return

        self._cycle_count += 1
        if self._cycle_count % LOG_EVERY_N_CYCLES == 0 and self._log_writer and self._session_id:
            data = DriveLogData(
                ref_speed_kmh=ref_speed,
                actual_speed_kmh=actual_speed,
                accel_opening=accel_opening,
                brake_opening=brake_opening,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
            )
            task = asyncio.ensure_future(self._log_writer.write_log(self._session_id, data))
            task.add_done_callback(_log_write_error_callback)

    def _get_ref_speed_and_accel(self, elapsed_s: float) -> tuple[float, float]:
        """経過時間 [s] から基準車速 [km/h] と基準加速度 [km/h/s] を線形補間で返す。"""
        points = self._mode.reference_speed

        if not points:
            return 0.0, 0.0

        if elapsed_s <= points[0].time_s:
            return points[0].speed_kmh, 0.0

        if elapsed_s >= points[-1].time_s:
            return points[-1].speed_kmh, 0.0

        for i in range(len(points) - 1):
            p0 = points[i]
            p1 = points[i + 1]
            if p0.time_s <= elapsed_s <= p1.time_s:
                dt = p1.time_s - p0.time_s
                if dt == 0.0:
                    return p1.speed_kmh, 0.0
                t_frac = (elapsed_s - p0.time_s) / dt
                speed = p0.speed_kmh + t_frac * (p1.speed_kmh - p0.speed_kmh)
                accel = (p1.speed_kmh - p0.speed_kmh) / dt
                return speed, accel

        return points[-1].speed_kmh, 0.0

    def _opening_to_position(self, opening_pct: float, zero_pos: int, full_pos: int) -> int:
        """開度 [%] をアクチュエータ位置 [pulse] に変換する。"""
        return zero_pos + round((full_pos - zero_pos) * opening_pct / 100.0)

    async def _drive_accel_axis(self, pos: int) -> float:
        """アクセル軸に位置指令を送り電流値を返す（同一バス上で逐次実行）。"""
        await self._accel_driver.move_to_position(pos)
        return await self._accel_driver.read_current()

    async def _drive_brake_axis(self, pos: int) -> float:
        """ブレーキ軸に位置指令を送り電流値を返す（同一バス上で逐次実行）。"""
        await self._brake_driver.move_to_position(pos)
        return await self._brake_driver.read_current()


def _log_write_error_callback(task: asyncio.Task[None]) -> None:
    """ログ書き込みタスクの例外をログに記録する。走行は継続する。"""
    exc = task.exception()
    if exc is not None:
        _logger.exception("ログ書き込みエラー (走行継続)", exc_info=exc)
