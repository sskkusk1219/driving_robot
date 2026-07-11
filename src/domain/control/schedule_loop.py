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
import bisect
import logging
from collections.abc import Awaitable, Callable
from typing import Protocol

from src.domain.control.base_loop import (
    MAX_PENDING_LOG_TASKS,
    WEDGED_CYCLE_TIMEOUT_S,
    CycleLoopBase,
    LogWriterProtocol,
)
from src.domain.control.conversions import clamp_opening, opening_to_position
from src.domain.control.pedal_safety import enforce_pedal_exclusion
from src.models.drive_log import DriveLogData
from src.models.profile import VehicleProfile
from src.models.system_state import RealtimeSnapshot
from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule

_logger = logging.getLogger(__name__)

SCHEDULE_LOOP_INTERVAL_S: float = 0.1  # 100ms 周期（drive_logs の記録間隔に一致）


class ActuatorDriverProtocol(Protocol):
    async def move_to_position(self, pos: int) -> None: ...

    async def read_current(self) -> float: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


class SafetyCheckProtocol(Protocol):
    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...


class ButtonServoProtocol(Protocol):
    async def press(self, channel: int, duration_s: float) -> None: ...


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


class ScheduleLoop(CycleLoopBase):
    """タイムスケジュールを開ループ実行し連続ログを記録するドメインコンポーネント。

    サイクル実行のライフサイクル・ウォッチドッグ・ログ書込バックログ管理は
    CycleLoopBase（DriveLoop/LearningLoop と共通）に委譲する。
    """

    _cycle_label = "スケジュールサイクル"

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
        super().__init__(
            interval_s=interval_s,
            on_complete=on_complete,
            on_emergency=on_emergency,
            log_writer=log_writer,
            session_id=session_id,
        )
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._schedule = schedule
        self._safety_check = safety_check
        self._button_servo = button_servo

        # ボタンイベントは time_s 昇順で発火判定する
        self._events: list[ButtonEvent] = sorted(schedule.button_events, key=lambda e: e.time_s)
        self._next_event_idx = 0
        # bisect 用に時刻列を前計算（interpolate_pedal は毎サイクル呼ばれるため O(log n) にする）
        self._pedal_times: list[float] = [p.time_s for p in schedule.pedal_points]

        self._start_time: float | None = None
        self._pending_press_tasks: set[asyncio.Task[None]] = set()
        # 直近指令位置（変化がない軸は move_to_position を省略する。learning_loop.py と同方式）
        self._accel_pos_cmd: int | None = None
        self._brake_pos_cmd: int | None = None

    def _reset_for_start(self) -> None:
        self._start_time = None
        self._next_event_idx = 0
        self._accel_pos_cmd = None
        self._brake_pos_cmd = None

    @property
    def current_ref_speed(self) -> float | None:
        """タイムスケジュールは基準速度を持たないため常に None。"""
        return None

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

        # タイムラインからペダル開度を補間 → クランプ。保存時バリデーション（schemas.py の
        # TimeScheduleSchema）でアクセル・ブレーキが同時に非ゼロとなる区間は reject 済みだが、
        # 既存データ・補間誤差への保険として他ループと同じ enforce_pedal_exclusion を通す
        # （機構保護のため同時踏みは常に禁止：pedal_safety.py 参照）。
        accel_opening, brake_opening = self._interpolate_pedal_at(t)
        accel_opening = clamp_opening(accel_opening, self._profile.max_accel_opening)
        brake_opening = clamp_opening(brake_opening, self._profile.max_brake_opening)
        accel_opening, brake_opening = enforce_pedal_exclusion(accel_opening, brake_opening)
        self._current_accel_opening = accel_opening
        self._current_brake_opening = brake_opening

        accel_pos = opening_to_position(accel_opening, calib.accel_zero_pos, calib.accel_full_pos)
        brake_pos = opening_to_position(brake_opening, calib.brake_zero_pos, calib.brake_full_pos)

        # CAN await 中に stop が入った場合は位置指令を送らない
        if not self._running:
            return

        try:
            # 指令位置が前サイクルと同じ軸は move_to_position を省略し電流のみ読む
            # （learning_loop.py._drive_or_sense と同方式、E3 レビュー指摘）。
            accel_current, brake_current = await self._drive_or_sense(accel_pos, brake_pos)
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

        # タイムライン終端: 正常終了
        if t >= self._schedule.total_duration:
            self.stop()
            await self._on_complete()

    def _interpolate_pedal_at(self, t: float) -> tuple[float, float]:
        """時刻 t のアクセル・ブレーキ開度 [%] を線形補間で返す（DriveLoop._ref_speed_at と
        同方式で bisect を使い、サイクル毎の毎tick先頭走査を避ける。E2 レビュー指摘）。
        """
        points = self._schedule.pedal_points
        if not points:
            return 0.0, 0.0
        if t <= points[0].time_s:
            return points[0].accel_opening, points[0].brake_opening
        if t >= points[-1].time_s:
            return points[-1].accel_opening, points[-1].brake_opening
        i = bisect.bisect_right(self._pedal_times, t) - 1
        p0 = points[i]
        p1 = points[i + 1]
        span = p1.time_s - p0.time_s
        if span <= 0.0:
            return p1.accel_opening, p1.brake_opening
        frac = (t - p0.time_s) / span
        accel = p0.accel_opening + (p1.accel_opening - p0.accel_opening) * frac
        brake = p0.brake_opening + (p1.brake_opening - p0.brake_opening) * frac
        return accel, brake

    async def _drive_or_sense(self, accel_pos: int, brake_pos: int) -> tuple[float, float]:
        """指令位置が変化した軸だけ move_to_position を発行し、変化のない軸は電流のみ読む。

        補間位置が前サイクルと同じ（区間内で開度が一定、または端点保持中）場合の無駄な
        書込を省く（learning_loop.py._drive_or_sense と同方式、E3 レビュー指摘）。
        """
        a_changed = accel_pos != self._accel_pos_cmd
        b_changed = brake_pos != self._brake_pos_cmd

        async def accel_axis() -> float:
            if a_changed:
                await self._accel_driver.move_to_position(accel_pos)
            return await self._accel_driver.read_current()

        async def brake_axis() -> float:
            if b_changed:
                await self._brake_driver.move_to_position(brake_pos)
            return await self._brake_driver.read_current()

        a, b = await asyncio.gather(accel_axis(), brake_axis())
        if a_changed:
            self._accel_pos_cmd = accel_pos
        if b_changed:
            self._brake_pos_cmd = brake_pos
        return a, b

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


__all__ = [
    "MAX_PENDING_LOG_TASKS",
    "SCHEDULE_LOOP_INTERVAL_S",
    "WEDGED_CYCLE_TIMEOUT_S",
    "ActuatorDriverProtocol",
    "ButtonServoProtocol",
    "CANReaderProtocol",
    "LogWriterProtocol",
    "SafetyCheckProtocol",
    "ScheduleLoop",
    "interpolate_pedal",
]
