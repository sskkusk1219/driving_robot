# ruff: noqa: E501
"""タイムスケジュールの再生（開ループ）。ランナーの ``--schedule`` / ``--schedule-file`` の中身。

RPD_Phase_20261115_PortResearch.md §3 段6。流れは core の ``run_steps`` と同じ（初期化 → 走行前チェック →
再生 → 必ず shutdown）で、再生の手順（``SCHEDULE_STEP_NUMBER``）だけをここで書き、core の ``Step`` として
``run_steps`` に渡す（core は直さない）。旧 ``src/domain/control/schedule_loop.py`` の考え方
（時刻でタイムラインを引く・ペダル開度を線形補間して指令・ボタンを指定時刻に押す）を、core の
アクチュエータの指令（``move_to_position``）・読み取り（``read_monitor``）・安全網（``AxisSafetyNet``）・
ボタンの押し方（``button_steps.press_button``）の上に移した。

- 開度は core の定義（原点 0%・9500 pulse 100%）。車両の max_* で頭打ち。使わないペダルは待機位置（min）。
  アクセルとブレーキの同時踏みは許す（2026-11-19 ユーザー決定）ので、片足ルール・実開度の安全網は掛けない。
- 基準車速は無い（ref なし）。記録は core の SessionLog（ランナーでは StreamingSessionLog）。区間名は SCHEDULE_DRIVE。
- 安全: 過電流・アラーム・電流ゼロの継続（axis_safety）・CAN 読み失敗・アクチュエータ通信失敗・最高速超えは
  ペダルを離して DriveError（終了コード 5）。ボタンの押下の失敗は進行のメッセージに残すだけで走行は続ける
  （旧 schedule_loop と同じ。ペダルの制御を優先）。
- 中断（SIGINT）はランナーが手順の本体を cancel する。ここの finally で押している最中のボタンを戻し、
  その後 run_steps の finally が必ず shutdown（原点復帰・サーボOFF・切断）する。
- 終わり: タイムラインの終端で車が動いていれば core の緩減速（``decelerate_to_stop``）で停車まで減速して
  停車保持し、止まっていればすぐ停車保持にする（core のモード走行の終わり ``_finish_stopped`` と同じ。段7）。
  その後 shutdown（原点復帰・サーボOFF・切断）へ進む。
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import time

from src.robot.cancel_guard import raise_if_cancelling
from src.robot.core import button_steps
from src.robot.core import main as core_main
from src.robot.core.axis_monitor import AxisMonitor
from src.robot.core.axis_safety import AxisSafetyNet
from src.robot.core.config import ResearchConfig
from src.robot.core.drive_log import SECTION_DECEL_TO_STOP, SessionLog
from src.robot.core.hardware import (
    ActuatorProtocol,
    DriveError,
    ResearchHardware,
    ensure_buttons_connected,
)
from src.robot.core.pattern_drive import _overcurrent_limit_ma, _release_pedals
from src.robot.core.research_types import VEHICLE_STOP_SPEED_KMH, DriveLogData
from src.robot.core.stop_decel import PHASE_APPROACH, PHASE_STOP_HOLD, decelerate_to_stop
from src.robot.core.term import say
from src.robot.core.vehicle import build_vehicle_profile, opening_to_pulse
from src.robot.schedule_spec import (
    BUTTON_LABELS,
    ButtonPress,
    PedalTimeline,
    ScheduleData,
    button_seconds,
    pedal_command,
)
from src.robot.streaming_log import SECTION_SCHEDULE_DRIVE

#: ランナーの手順番号（core の手順 0〜25 と重ならない値）
SCHEDULE_STEP_NUMBER = 30
SCHEDULE_STEP_TITLE = "タイムスケジュール再生（開ループ）"

#: 軸の指令の滑らかさ（core の mode_drive.AXIS_SMOOTH_DUTY / AXIS_PRE_READ_DELAY_S と同じ値）
AXIS_SMOOTH_DUTY = 0.8
AXIS_PRE_READ_DELAY_S = 0.002

PHASE_SCHEDULE = "SCHEDULE"


class _ScheduleRun:
    """1 本のスケジュールの再生。``run()`` が終端まで 50 ms 周期で回す。"""

    def __init__(
        self, hw: ResearchHardware, cfg: ResearchConfig, schedule: ScheduleData, log: SessionLog
    ) -> None:
        self.hw = hw
        self.cfg = cfg
        self.schedule = schedule
        self.log = log
        self.interval_s = cfg.control.loop_interval_s
        self.timeline = PedalTimeline(schedule.points)
        self.total_s = schedule.total_duration
        self.overcurrent_ma = _overcurrent_limit_ma(hw)
        self.safety = AxisSafetyNet(self.interval_s)
        v = cfg.vehicle
        self.limits = {
            "min_accel": v.min_accel_opening_pct,
            "min_brake": v.min_brake_opening_pct,
            "max_accel": v.max_accel_opening_pct,
            "max_brake": v.max_brake_opening_pct,
        }
        self.cycles = 0
        self.overruns = 0
        self.max_speed_seen = 0.0
        self.press_failures = 0
        self._accel_cmd: int | None = None
        self._brake_cmd: int | None = None
        self._next_button = 0
        self._press_tasks: set[asyncio.Task[None]] = set()
        self._next_print = 0.0

    # ── 本体 ──────────────────────────────────────────────────────

    async def run(self) -> float:
        """終端（総時間）まで回し、走った秒数を返す。異常は DriveError（ペダルを離してから）。"""
        loop = asyncio.get_running_loop()
        started = loop.time()
        next_tick = started
        t = 0.0
        try:
            while True:
                t = loop.time() - started
                await self._cycle(t)
                if t >= self.total_s:
                    break
                next_tick += self.interval_s
                now = loop.time()
                if now - next_tick > self.interval_s:
                    self.overruns += 1
                    next_tick = now
                await asyncio.sleep(max(0.0, next_tick - now))
            await self._wait_presses()
        except DriveError:
            await self._cancel_presses()
            await _release_pedals(self.hw)
            raise
        finally:
            await self._cancel_presses()  # 中断（cancel）でも押している最中のボタンを戻す
        return t

    async def _cycle(self, t: float) -> None:
        hw, cfg = self.hw, self.cfg
        cycle_started = time.perf_counter()
        try:
            speed = await hw.can.read_speed()
        except Exception as exc:
            raise DriveError(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）") from exc
        self.max_speed_seen = max(self.max_speed_seen, speed)

        want_accel, want_brake = self.timeline.at(t)
        accel, brake = pedal_command(want_accel, want_brake, **self.limits)
        accel_pos, brake_pos = opening_to_pulse(accel), opening_to_pulse(brake)

        try:
            monitor_accel, monitor_brake = await asyncio.gather(
                self._drive_axis(hw.accel, accel_pos, is_accel=True),
                self._drive_axis(hw.brake, brake_pos, is_accel=False),
            )
        except Exception as exc:
            raise_if_cancelling(exc)
            raise DriveError(
                f"アクチュエータ通信に失敗しました（{type(exc).__name__}: {exc}）"
            ) from exc
        cycle_ms = 1000.0 * (time.perf_counter() - cycle_started)
        alarm_accel, alarm_brake = await self.safety.poll_alarms(self.cycles, hw.accel, hw.brake)
        self._check_safety(
            t, speed, monitor_accel.current_ma, monitor_brake.current_ma,
            accel_pos, brake_pos, alarm_accel, alarm_brake,
        )

        self._fire_due_buttons(t)

        if self.cycles % cfg.control.log_every_n_cycles == 0:
            data = DriveLogData(
                ref_speed_kmh=None,
                actual_speed_kmh=speed,
                accel_opening=accel,
                brake_opening=brake,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=monitor_accel.current_ma,
                brake_current=monitor_brake.current_ma,
                phase=PHASE_SCHEDULE,
            )
            self.log.record(
                data, section=SECTION_SCHEDULE_DRIVE, phase=PHASE_SCHEDULE, pattern=self.schedule.name,
                mode_time_s=t, alarm_accel=alarm_accel, alarm_brake=alarm_brake,
                monitor_accel=monitor_accel, monitor_brake=monitor_brake, cycle_ms=cycle_ms,
            )
        self.cycles += 1
        if t >= self._next_print:
            interval = cfg.output.print_interval_s
            self._next_print = (math.floor(t / interval) + 1) * interval
            say(
                f"スケジュール {t:6.1f}/{self.total_s:.0f}s  車速 {speed:6.2f} km/h  "
                f"アクセル {accel:5.2f}%  ブレーキ {brake:5.2f}%"
            )

    async def _drive_axis(
        self, axis: ActuatorProtocol, pos: int, *, is_accel: bool
    ) -> AxisMonitor:
        """位置が変わった軸だけ指令し（core の mode_drive と同じ）、まとめ読みする。"""
        last = self._accel_cmd if is_accel else self._brake_cmd
        if pos != last:
            await axis.move_to_position(pos, smooth_over_s=self.interval_s * AXIS_SMOOTH_DUTY)
            if is_accel:
                self._accel_cmd = pos
            else:
                self._brake_cmd = pos
            await asyncio.sleep(AXIS_PRE_READ_DELAY_S)
        return await axis.read_monitor()

    def _check_safety(
        self, t: float, speed: float, accel_current: float, brake_current: float,
        accel_pos: int, brake_pos: int, alarm_accel: bool | None, alarm_brake: bool | None,
    ) -> None:
        for label, current in (("アクセル", accel_current), ("ブレーキ", brake_current)):
            if current > self.overcurrent_ma:
                raise DriveError(
                    f"{label}軸が過電流です（{current:.0f} mA > {self.overcurrent_ma:.0f} mA、t={t:.1f}s）"
                )
        reason = self.safety.check(
            t=t, accel_pos=accel_pos, brake_pos=brake_pos,
            accel_current=accel_current, brake_current=brake_current,
            alarm_accel=alarm_accel, alarm_brake=alarm_brake,
        )
        if reason is not None:
            raise DriveError(reason)
        if speed > self.cfg.vehicle.max_speed_kmh:
            raise DriveError(
                f"車速 {speed:.1f} km/h が最高速 {self.cfg.vehicle.max_speed_kmh:.1f} km/h を超えました"
            )

    # ── ボタン ────────────────────────────────────────────────────

    def _fire_due_buttons(self, t: float) -> None:
        """時刻に達した未発火のボタンイベントを押す（背景タスク。ペダルの周期を止めない）。"""
        events = self.schedule.buttons
        while self._next_button < len(events) and events[self._next_button].time_s <= t:
            event = events[self._next_button]
            self._next_button += 1
            task = asyncio.ensure_future(self._press(event, t))
            self._press_tasks.add(task)
            task.add_done_callback(self._press_tasks.discard)

    async def _press(self, event: ButtonPress, t: float) -> None:
        assert event.button is not None  # noqa: S101 - check_schedule で名前なしは弾いてある
        b = self.cfg.buttons
        seconds = button_seconds(
            event.press_duration_s, event.button,
            start_short_press_s=b.start_short_press_s, shift_press_s=b.shift_press_s,
        )
        say(f"  スケジュール {t:.1f}s: ボタン「{BUTTON_LABELS.get(event.button, event.button)}」を {seconds:g}s 押します")
        try:
            await button_steps.press_button(
                self.hw, self.cfg, event.button, seconds, poll=False, wait_after=False
            )
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - 旧 schedule_loop と同じ: ボタンの失敗でペダルの制御は止めない
            self.press_failures += 1
            say(f"警告: ボタン「{event.button}」の押下に失敗しました（{type(exc).__name__}: {exc}）。走行は続けます")

    async def _wait_presses(self) -> None:
        if self._press_tasks:
            await asyncio.gather(*list(self._press_tasks), return_exceptions=True)

    async def _cancel_presses(self) -> None:
        tasks = [t for t in self._press_tasks if not t.done()]
        for task in tasks:
            task.cancel()
        for task in tasks:
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await task


async def _finish_stopped(
    hw: ResearchHardware, cfg: ResearchConfig, log: SessionLog, run: _ScheduleRun
) -> None:
    """終端で停車保持にする（core の mode_drive._finish_stopped と同じ流れ）。

    車が動いていれば core の緩減速（decelerate_to_stop）で止めて停車保持する。止まっていれば
    アクセルを原点・ブレーキを停車保持開度にしてすぐ返す。
    """
    try:
        speed = await hw.can.read_speed()
    except Exception as exc:
        await _release_pedals(hw)
        raise DriveError(
            f"終端で CAN 車速を読めません（{type(exc).__name__}: {exc}）"
        ) from exc
    if speed >= VEHICLE_STOP_SPEED_KMH:
        say(f"終端の車速 {speed:.2f} km/h: 緩減速で停車させます …")
        log.mark(SECTION_DECEL_TO_STOP, PHASE_APPROACH)
        log.start_sampler()
        stop = await decelerate_to_stop(hw, cfg, build_vehicle_profile(cfg), log=log)
        say(f"停車保持: ブレーキ {stop.hold_pct:.2f}%")
        return
    hold_pct = cfg.stop_brake_opening_pct
    log.mark(SECTION_DECEL_TO_STOP, PHASE_STOP_HOLD)
    try:
        await run._drive_axis(hw.accel, 0, is_accel=True)  # noqa: SLF001
        await run._drive_axis(hw.brake, opening_to_pulse(hold_pct), is_accel=False)  # noqa: SLF001
    except Exception as exc:
        await _release_pedals(hw)
        raise DriveError(f"終端で停車保持へ移れません（{type(exc).__name__}: {exc}）") from exc
    log.start_sampler()
    say(f"停車しています（{speed:.2f} km/h）。停車保持: ブレーキ {hold_pct:.2f}%")


async def run_schedule_drive(
    hw: ResearchHardware, cfg: ResearchConfig, schedule: ScheduleData, *, log: SessionLog
) -> _ScheduleRun:
    """停車保持の状態からスケジュールを再生し、終端で両ペダルを待機位置にして返す。"""
    say(f"タイムスケジュール: 「{schedule.name}」（{schedule.total_duration:.1f}s・ペダル開度の点 "
        f"{len(schedule.points)}・ボタンイベント {len(schedule.buttons)}）")
    say("制御: 開ループ（基準車速なし）。ペダル開度を時刻で線形補間して "
        f"{cfg.control.loop_interval_s * 1000:.0f}ms 周期で指令します")
    v = cfg.vehicle
    say(f"ペダルの上限: アクセル {v.max_accel_opening_pct:g}%・ブレーキ {v.max_brake_opening_pct:g}%"
        f"／待機位置（min）: アクセル {v.min_accel_opening_pct:g}%・ブレーキ {v.min_brake_opening_pct:g}%"
        "（各ペダルを独立に min〜上限で頭打ち。min 未満は待機位置の min。アクセルとブレーキの同時踏みも許す）")
    say(f"中止条件: 過電流・アラーム・電流ゼロの継続・CAN 読み失敗・通信失敗・最高速 {v.max_speed_kmh:g}km/h 超え")
    if hw.is_real:
        say("*** 実機モード: 車両がスケジュールどおりに走ります。シャシダイナモ上で実施してください ***")
    if schedule.buttons:
        await ensure_buttons_connected(hw)  # 周期の中で初めて繋がないよう、先に繋ぐ

    run = _ScheduleRun(hw, cfg, schedule, log)
    await log.stop_sampler()  # 50 ms の制御ループと Modbus を取り合わない（記録はループから）
    log.mark(SECTION_SCHEDULE_DRIVE, PHASE_SCHEDULE)
    run_s = await run.run()

    say(f"スケジュール再生 完了: {run_s:.1f}s・{run.cycles} 周期"
        f"（1 周期以上の遅れ {run.overruns} 回・"
        f"ボタン押下の失敗 {run.press_failures} 回）・最高車速 {run.max_speed_seen:.1f} km/h")
    await _finish_stopped(hw, cfg, log, run)
    return run


def make_schedule_step(schedule: ScheduleData) -> core_main.Step:
    """core の Step（走行する手順・基準車速なし）。run_steps が走行前チェックを挟んで呼ぶ（手順 1 は入れない。接続・サーボ ON は run.py の quiet_connect）。"""

    async def handler(ctx: core_main.RunContext) -> None:
        hw = ctx.require_hardware()
        log = ctx.session_log
        if log is None or log.closed:  # 走行前チェックから続くログ（無ければ作る）
            log = SessionLog(ctx.config, hw, has_ref=False)
            log.start(SECTION_SCHEDULE_DRIVE, PHASE_SCHEDULE)
            ctx.session_log = log
        await run_schedule_drive(hw, ctx.config, schedule, log=log)
        await log.close()  # 終端まで記録したら保存（csv イベントが出る）

    return core_main.Step(
        SCHEDULE_STEP_NUMBER, SCHEDULE_STEP_TITLE, handler, drives=True, has_ref=False
    )
