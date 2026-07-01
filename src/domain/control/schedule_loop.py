"""タイムスケジュール（統合タイムライン）の開ループ実行ループ。

基準車速追従（DriveLoop・閉ループ）や学習パターン（LearningLoop・状態機械）とは異なり、
「経過時刻でタイムラインを引く」方式で動く。ペダル開度（アクセル・ブレーキ）を時系列で
線形補間して開ループ指令し、ボタンイベントを指定時刻で発火する。走行全体を 100ms 周期で
連続した (実車速, 開度) 軌跡として drive_logs に記録する（run_type='auto'、ref_speed=None）。

安全:
  - 非常停止型: 過電流・CAN 読取失敗・アクチュエータ失敗・サイクル例外 → on_emergency。
  - ボタンサーボの I2C 失敗は press タスク内でログのみ（ペダル制御を優先し走行は継続）。
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Awaitable, Callable
from typing import Protocol

from src.domain.control.pedal_safety import enforce_pedal_exclusion
from src.models.drive_log import DriveLogData
from src.models.profile import VehicleProfile
from src.models.system_state import RealtimeSnapshot
from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule

_logger = logging.getLogger(__name__)

SCHEDULE_LOOP_INTERVAL_S: float = 0.1  # 100ms 周期（drive_logs の記録間隔に一致）
WEDGED_CYCLE_TIMEOUT_S: float = 1.0
MAX_PENDING_LOG_TASKS: int = 100


class ActuatorDriverProtocol(Protocol):
    async def move_to_position(self, pos: int) -> None: ...

    async def read_current(self) -> float: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


class SafetyCheckProtocol(Protocol):
    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...


class ButtonServoProtocol(Protocol):
    async def press(self, channel: int, duration_s: float) -> None: ...


class LogWriterProtocol(Protocol):
    async def write_log(self, session_id: str, data: DriveLogData) -> None: ...


def interpolate_pedal(points: list[PedalPoint], t: float) -> tuple[float, float]:
    """時刻 t におけるアクセル・ブレーキ開度 [%] を線形補間して返す。

    points は time_s 昇順を前提とする。範囲外は端点で保持する（ゼロオーダー外挿）。
    points が空なら (0, 0)。
    """
    if not points:
        return 0.0, 0.0
    if t <= points[0].time_s:
        return points[0].accel_opening, points[0].brake_opening
    if t >= points[-1].time_s:
        return points[-1].accel_opening, points[-1].brake_opening
    for i in range(1, len(points)):
        p1 = points[i]
        if t <= p1.time_s:
            p0 = points[i - 1]
            span = p1.time_s - p0.time_s
            if span <= 0.0:
                return p1.accel_opening, p1.brake_opening
            frac = (t - p0.time_s) / span
            accel = p0.accel_opening + (p1.accel_opening - p0.accel_opening) * frac
            brake = p0.brake_opening + (p1.brake_opening - p0.brake_opening) * frac
            return accel, brake
    return points[-1].accel_opening, points[-1].brake_opening


def _log_emergency_error_callback(task: asyncio.Task[None]) -> None:
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        _logger.error("スケジュールループの非常停止コールバックで例外", exc_info=exc)


class ScheduleLoop:
    """タイムスケジュールを開ループ実行し連続ログを記録するドメインコンポーネント。"""

    def __init__(
        self,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        profile: VehicleProfile,
        schedule: TimeSchedule,
        safety_check: SafetyCheckProtocol,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        button_servo: ButtonServoProtocol | None = None,
        log_writer: LogWriterProtocol | None = None,
        session_id: str | None = None,
        interval_s: float = SCHEDULE_LOOP_INTERVAL_S,
    ) -> None:
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._schedule = schedule
        self._safety_check = safety_check
        self._on_complete = on_complete
        self._on_emergency = on_emergency
        self._button_servo = button_servo
        self._log_writer = log_writer
        self._session_id = session_id
        self._interval_s = interval_s

        # ボタンイベントは time_s 昇順で発火判定する
        self._events: list[ButtonEvent] = sorted(schedule.button_events, key=lambda e: e.time_s)
        self._next_event_idx = 0

        self._running = False
        self._start_time: float | None = None
        self._current_accel_opening = 0.0
        self._current_brake_opening = 0.0
        self._consecutive_skips = 0
        self._log_backlog_active = False

        self._cycle_task: asyncio.Task[None] | None = None
        self._emergency_task: asyncio.Task[None] | None = None
        self._pending_log_tasks: set[asyncio.Task[None]] = set()
        self._pending_press_tasks: set[asyncio.Task[None]] = set()
        self._last_snapshot: RealtimeSnapshot | None = None

    # ── ライフサイクル ──────────────────────────────────────────────
    def start(self) -> None:
        """ループを開始する。既に実行中なら何もしない。"""
        if self._running:
            return
        self._running = True
        self._start_time = None
        self._next_event_idx = 0
        self._consecutive_skips = 0
        loop = asyncio.get_running_loop()
        loop.call_later(self._interval_s, self._schedule_next_cycle)

    def stop(self) -> None:
        """ループを停止する。進行中サイクルは書き込み前に中断される。"""
        self._running = False

    async def stop_and_join(self, timeout_s: float = 2.0) -> None:
        """停止し、進行中のサイクルタスク完了を待つ。home_return 前に必ず呼ぶ。

        LearningLoop.stop_and_join と同じく、自タスク（サイクル）からの呼び出しは
        join せず即 return する（自タスク join でのデッドロック回避）。
        """
        self.stop()
        task = self._cycle_task
        if task is None or task.done() or task is asyncio.current_task():
            return
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout_s)
        except TimeoutError:
            _logger.warning(
                "進行中のスケジュールサイクルが %.1fs 以内に完了せずキャンセルします", timeout_s
            )
            task.cancel()
        except Exception:
            pass

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def current_accel_opening(self) -> float:
        return self._current_accel_opening

    @property
    def current_brake_opening(self) -> float:
        return self._current_brake_opening

    @property
    def current_ref_speed(self) -> float | None:
        """タイムスケジュールは基準速度を持たないため常に None。"""
        return None

    @property
    def last_snapshot(self) -> RealtimeSnapshot | None:
        return self._last_snapshot

    # ── スケジューリング（LearningLoop と同方式）────────────────────
    def _schedule_next_cycle(self) -> None:
        if not self._running:
            return
        if self._cycle_task is not None and not self._cycle_task.done():
            self._consecutive_skips += 1
            wedged_s = self._consecutive_skips * self._interval_s
            _logger.warning(
                "前回のスケジュールサイクルが %.0fms 以内に完了せずスキップ",
                self._interval_s * 1000,
            )
            if wedged_s >= WEDGED_CYCLE_TIMEOUT_S:
                _logger.error(
                    "スケジュールサイクルが %.1fs 以上完了していません: 非常停止します", wedged_s
                )
                self.stop()
                self._emergency_task = asyncio.ensure_future(self._on_emergency())
                self._emergency_task.add_done_callback(_log_emergency_error_callback)
                return
        else:
            self._consecutive_skips = 0
            self._cycle_task = asyncio.ensure_future(self._execute_one_cycle())
            self._cycle_task.add_done_callback(self._on_cycle_done)
        asyncio.get_running_loop().call_later(self._interval_s, self._schedule_next_cycle)

    def _on_cycle_done(self, task: asyncio.Task[None]) -> None:
        if task.cancelled():
            return
        exc = task.exception()
        if exc is None:
            return
        _logger.error("スケジュールサイクルで未捕捉例外: 非常停止します", exc_info=exc)
        self.stop()
        self._emergency_task = asyncio.ensure_future(self._on_emergency())
        self._emergency_task.add_done_callback(_log_emergency_error_callback)

    async def _abort_emergency(self) -> None:
        self.stop()
        await self._on_emergency()

    # ── 1 サイクル ─────────────────────────────────────────────────
    async def _execute_one_cycle(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        now = loop.time()
        if self._start_time is None:
            self._start_time = now
        t = now - self._start_time

        # 実車速を取得（取得失敗は非常停止）
        try:
            speed = await self._can_reader.read_speed()
        except Exception:
            _logger.exception("CAN 車速取得失敗: 非常停止")
            await self._abort_emergency()
            return

        calib = self._profile.calibration
        if calib is None:
            _logger.error("キャリブレーションデータがない: 非常停止")
            await self._abort_emergency()
            return

        # タイムラインからペダル開度を補間 → クランプ → 同時踏み排除
        accel_opening, brake_opening = interpolate_pedal(self._schedule.pedal_points, t)
        accel_opening = self._clamp_accel(accel_opening)
        brake_opening = self._clamp_brake(brake_opening)
        accel_opening, brake_opening = enforce_pedal_exclusion(accel_opening, brake_opening)
        self._current_accel_opening = accel_opening
        self._current_brake_opening = brake_opening

        accel_pos = self._opening_to_position(
            accel_opening, calib.accel_zero_pos, calib.accel_full_pos
        )
        brake_pos = self._opening_to_position(
            brake_opening, calib.brake_zero_pos, calib.brake_full_pos
        )

        # CAN await 中に stop が入った場合は位置指令を送らない
        if not self._running:
            return

        try:
            accel_current, brake_current = await asyncio.gather(
                self._drive_axis(self._accel_driver, accel_pos),
                self._drive_axis(self._brake_driver, brake_pos),
            )
        except Exception:
            _logger.exception("アクチュエータ通信失敗: 非常停止")
            await self._abort_emergency()
            return

        self._last_snapshot = RealtimeSnapshot(
            actual_speed_kmh=speed,
            accel_pos=accel_pos,
            brake_pos=brake_pos,
            accel_current_ma=accel_current,
            brake_current_ma=brake_current,
            captured_at=loop.time(),
        )

        # 非常停止型安全: 過電流
        if self._safety_check.check_overcurrent(accel_current, "accel"):
            _logger.warning("アクセル過電流: %.1f mA", accel_current)
            await self._abort_emergency()
            return
        if self._safety_check.check_overcurrent(brake_current, "brake"):
            _logger.warning("ブレーキ過電流: %.1f mA", brake_current)
            await self._abort_emergency()
            return

        # ボタンイベント発火（時刻到達分をまとめて発火。I2C は独立バスで fire-and-forget）
        self._fire_due_button_events(t)

        # 連続ログ記録（基準速度は持たないため ref_speed_kmh=None）
        self._enqueue_log_write(
            DriveLogData(
                ref_speed_kmh=None,
                actual_speed_kmh=speed,
                accel_opening=accel_opening,
                brake_opening=brake_opening,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
            )
        )

        # タイムライン終端: loop なら巻き戻し、そうでなければ正常終了
        if t >= self._schedule.total_duration:
            if self._schedule.loop:
                self._start_time = now
                self._next_event_idx = 0
            else:
                self.stop()
                await self._on_complete()

    async def _drive_axis(self, driver: ActuatorDriverProtocol, pos: int) -> float:
        await driver.move_to_position(pos)
        return await driver.read_current()

    def _fire_due_button_events(self, t: float) -> None:
        """経過時刻 t までに到達した未発火ボタンイベントを押下する（fire-and-forget）。"""
        if self._button_servo is None:
            self._next_event_idx = len(self._events)  # ドライバなしなら空撃ちで前進
            return
        while self._next_event_idx < len(self._events):
            event = self._events[self._next_event_idx]
            if event.time_s > t:
                break
            self._next_event_idx += 1
            self._enqueue_button_press(event)

    def _enqueue_button_press(self, event: ButtonEvent) -> None:
        assert self._button_servo is not None
        task = asyncio.ensure_future(
            self._button_servo.press(event.channel, event.press_duration_s)
        )
        self._pending_press_tasks.add(task)
        task.add_done_callback(self._on_press_done)

    def _on_press_done(self, task: asyncio.Task[None]) -> None:
        self._pending_press_tasks.discard(task)
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            _logger.error("ボタン押下エラー (走行継続)", exc_info=exc)

    def _clamp_accel(self, opening_pct: float) -> float:
        return max(0.0, min(opening_pct, self._profile.max_accel_opening))

    def _clamp_brake(self, opening_pct: float) -> float:
        return max(0.0, min(opening_pct, self._profile.max_brake_opening))

    def _opening_to_position(self, opening_pct: float, zero_pos: int, full_pos: int) -> int:
        return zero_pos + round((full_pos - zero_pos) * opening_pct / 100.0)

    def _enqueue_log_write(self, data: DriveLogData) -> None:
        if self._log_writer is None or self._session_id is None:
            return
        if len(self._pending_log_tasks) >= MAX_PENDING_LOG_TASKS:
            if not self._log_backlog_active:
                self._log_backlog_active = True
                _logger.warning(
                    "スケジュールログ書き込みが %d 件滞留: 解消まで記録をスキップします",
                    MAX_PENDING_LOG_TASKS,
                )
            return
        if self._log_backlog_active:
            self._log_backlog_active = False
            _logger.info("スケジュールログ書き込みの滞留が解消しました")
        task = asyncio.ensure_future(self._log_writer.write_log(self._session_id, data))
        self._pending_log_tasks.add(task)
        task.add_done_callback(self._on_log_write_done)

    def _on_log_write_done(self, task: asyncio.Task[None]) -> None:
        self._pending_log_tasks.discard(task)
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            _logger.error("スケジュールログ書き込みエラー (走行継続)", exc_info=exc)
