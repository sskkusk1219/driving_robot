"""ScheduleLoop（タイムスケジュール開ループ実行）のユニットテスト。

call_later のタイミングに依存せず、_execute_one_cycle を直接呼んで検証する。
_start_time を操作して経過時刻 t を制御する。
"""

from __future__ import annotations

import asyncio
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest

from src.domain.control.schedule_loop import (
    WEDGED_CYCLE_TIMEOUT_S,
    ScheduleLoop,
    interpolate_pedal,
)
from src.models.calibration import CalibrationData
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile
from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule


def _make_calibration() -> CalibrationData:
    return CalibrationData(
        accel_zero_pos=100,
        accel_full_pos=600,
        accel_stroke=500,
        brake_zero_pos=200,
        brake_full_pos=700,
        brake_stroke=500,
        calibrated_at=datetime(2026, 1, 1),
        is_valid=True,
    )


def _make_profile() -> VehicleProfile:
    return VehicleProfile(
        id="p1",
        name="Test",
        max_accel_opening=80.0,
        max_brake_opening=80.0,
        max_speed=120.0,
        max_decel_g=0.5,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=_make_calibration(),
        model_path=None,
        created_at=datetime(2026, 1, 1),
        updated_at=datetime(2026, 1, 1),
        feedforward_params=FeedforwardParams(),
    )


def _make_schedule(
    pedal_points: list[PedalPoint] | None = None,
    button_events: list[ButtonEvent] | None = None,
    total_duration: float = 10.0,
) -> TimeSchedule:
    if pedal_points is None:
        pedal_points = [
            PedalPoint(time_s=0.0, accel_opening=0.0, brake_opening=0.0),
            PedalPoint(time_s=10.0, accel_opening=100.0, brake_opening=0.0),
        ]
    return TimeSchedule(
        id="s1",
        name="sched",
        description="",
        pedal_points=pedal_points,
        button_events=button_events or [],
        total_duration=total_duration,
        created_at=datetime.now(tz=UTC),
    )


def _make_loop(schedule: TimeSchedule) -> tuple[ScheduleLoop, dict[str, object]]:
    accel = MagicMock()
    accel.move_to_position = AsyncMock()
    accel.read_current = AsyncMock(return_value=100.0)
    brake = MagicMock()
    brake.move_to_position = AsyncMock()
    brake.read_current = AsyncMock(return_value=100.0)
    can = MagicMock()
    can.read_speed = AsyncMock(return_value=42.0)
    safety = MagicMock()
    safety.check_overcurrent = MagicMock(return_value=False)
    button = MagicMock()
    button.press = AsyncMock()
    on_complete = AsyncMock()
    on_emergency = AsyncMock()
    loop = ScheduleLoop(
        accel_driver=accel,
        brake_driver=brake,
        can_reader=can,
        profile=_make_profile(),
        schedule=schedule,
        safety_check=safety,
        on_complete=on_complete,
        on_emergency=on_emergency,
        button_servo=button,
    )
    return loop, {
        "accel": accel,
        "brake": brake,
        "safety": safety,
        "button": button,
        "on_complete": on_complete,
        "on_emergency": on_emergency,
    }


def _set_t(loop: ScheduleLoop, t: float) -> None:
    """次サイクルの経過時刻 t を強制する。"""
    loop._running = True
    loop._start_time = asyncio.get_running_loop().time() - t


# ── interpolate_pedal（純関数）─────────────────────────────────────────


def test_interpolate_empty_returns_zero() -> None:
    assert interpolate_pedal([], 3.0) == (0.0, 0.0)


def test_interpolate_before_first_point_holds_first() -> None:
    pts = [PedalPoint(2.0, 10.0, 0.0), PedalPoint(4.0, 20.0, 0.0)]
    assert interpolate_pedal(pts, 0.0) == (10.0, 0.0)


def test_interpolate_after_last_point_holds_last() -> None:
    pts = [PedalPoint(2.0, 10.0, 0.0), PedalPoint(4.0, 20.0, 0.0)]
    assert interpolate_pedal(pts, 99.0) == (20.0, 0.0)


def test_interpolate_midpoint_is_linear() -> None:
    pts = [PedalPoint(0.0, 0.0, 0.0), PedalPoint(10.0, 100.0, 0.0)]
    accel, brake = interpolate_pedal(pts, 5.0)
    assert accel == 50.0
    assert brake == 0.0


# ── 1 サイクル ──────────────────────────────────────────────────────────


async def test_cycle_drives_actuators_with_interpolated_opening() -> None:
    loop, m = _make_loop(_make_schedule())
    _set_t(loop, 5.0)  # 中点 → accel 50%
    await loop._execute_one_cycle()
    # accel 50% → zero(100) + 500*0.5 = 350
    m["accel"].move_to_position.assert_awaited_once_with(350)
    m["brake"].move_to_position.assert_awaited_once_with(200)


async def test_cycle_skips_move_to_position_when_opening_unchanged() -> None:
    """E3 回帰テスト: 補間位置が前サイクルと同じ軸は move_to_position を省略し電流のみ読む
    （learning_loop.py と同方式で無駄な書込を避ける）。"""
    pts = [
        PedalPoint(time_s=0.0, accel_opening=50.0, brake_opening=0.0),
        PedalPoint(time_s=10.0, accel_opening=50.0, brake_opening=0.0),
    ]
    loop, m = _make_loop(_make_schedule(pedal_points=pts, total_duration=20.0))
    _set_t(loop, 1.0)
    await loop._execute_one_cycle()
    m["accel"].move_to_position.assert_awaited_once_with(350)

    m["accel"].move_to_position.reset_mock()
    _set_t(loop, 2.0)  # 開度は一定(50%)なので指令位置は変化しない
    await loop._execute_one_cycle()
    m["accel"].move_to_position.assert_not_awaited()
    m["accel"].read_current.assert_awaited()


async def test_bisect_interpolation_matches_pure_function_across_segments() -> None:
    """E2 回帰テスト: bisect ベースの _interpolate_pedal_at が線形走査の interpolate_pedal と
    複数区間にわたって一致する（前計算した時刻列のインデックスがずれていないことを確認）。"""
    pts = [
        PedalPoint(time_s=0.0, accel_opening=0.0, brake_opening=0.0),
        PedalPoint(time_s=2.0, accel_opening=40.0, brake_opening=0.0),
        PedalPoint(time_s=5.0, accel_opening=40.0, brake_opening=0.0),
        PedalPoint(time_s=8.0, accel_opening=0.0, brake_opening=30.0),
    ]
    loop, _m = _make_loop(_make_schedule(pedal_points=pts, total_duration=8.0))
    for t in [0.0, 1.0, 2.0, 3.5, 5.0, 6.5, 8.0, 9.0]:
        assert loop._interpolate_pedal_at(t) == interpolate_pedal(pts, t)


async def test_cycle_fires_due_button_events() -> None:
    events = [ButtonEvent(time_s=1.0, channel=3, press_duration_s=0.5)]
    loop, m = _make_loop(_make_schedule(button_events=events))
    _set_t(loop, 2.0)  # t=2 > event t=1 → 発火
    await loop._execute_one_cycle()
    await asyncio.sleep(0)  # fire-and-forget の press タスクを走らせる
    m["button"].press.assert_awaited_once_with(3, 0.5)


async def test_cycle_does_not_fire_future_button_events() -> None:
    events = [ButtonEvent(time_s=8.0, channel=3, press_duration_s=0.5)]
    loop, m = _make_loop(_make_schedule(button_events=events))
    _set_t(loop, 2.0)  # t=2 < event t=8 → 未発火
    await loop._execute_one_cycle()
    await asyncio.sleep(0)
    m["button"].press.assert_not_awaited()


async def test_simultaneous_pedal_press_is_excluded() -> None:
    """D3 回帰テスト: 補間結果が同時に非ゼロでも enforce_pedal_exclusion で排他される
    （機構保護のため同時踏みは常に禁止。大きい方＝ブレーキが優先で残る）。"""
    pts = [PedalPoint(time_s=0.0, accel_opening=30.0, brake_opening=40.0)]
    loop, m = _make_loop(_make_schedule(pedal_points=pts, total_duration=10.0))
    _set_t(loop, 0.0)
    await loop._execute_one_cycle()
    assert loop.current_accel_opening == 0.0
    assert loop.current_brake_opening == 40.0
    m["accel"].move_to_position.assert_awaited_once_with(100)
    m["brake"].move_to_position.assert_awaited_once_with(200 + round(500 * 0.4))


async def test_completion_calls_on_complete_at_timeline_end() -> None:
    loop, m = _make_loop(_make_schedule(total_duration=10.0))
    _set_t(loop, 11.0)  # t >= total_duration
    await loop._execute_one_cycle()
    m["on_complete"].assert_awaited_once()
    assert loop.is_running is False


async def test_overcurrent_triggers_emergency() -> None:
    loop, m = _make_loop(_make_schedule())
    m["safety"].check_overcurrent = MagicMock(return_value=True)
    _set_t(loop, 5.0)
    await loop._execute_one_cycle()
    m["on_emergency"].assert_awaited_once()
    assert loop.is_running is False


async def test_can_read_failure_triggers_emergency() -> None:
    loop, m = _make_loop(_make_schedule())
    loop._can_reader.read_speed = AsyncMock(side_effect=RuntimeError("CAN down"))
    _set_t(loop, 5.0)
    await loop._execute_one_cycle()
    m["on_emergency"].assert_awaited_once()


async def test_missing_calibration_triggers_emergency() -> None:
    schedule = _make_schedule()
    loop, m = _make_loop(schedule)
    loop._profile.calibration = None
    _set_t(loop, 5.0)
    await loop._execute_one_cycle()
    m["on_emergency"].assert_awaited_once()


# ── 安全系パス（DriveLoop 相当）─────────────────────────────────────────


async def test_actuator_failure_triggers_emergency() -> None:
    loop, m = _make_loop(_make_schedule())
    m["accel"].move_to_position = AsyncMock(side_effect=RuntimeError("modbus down"))
    _set_t(loop, 5.0)
    await loop._execute_one_cycle()
    m["on_emergency"].assert_awaited_once()
    assert loop.is_running is False


async def test_stop_during_can_read_skips_actuator_command() -> None:
    loop, m = _make_loop(_make_schedule())

    async def _read_and_stop() -> float:
        loop.stop()  # CAN await 中に停止が入った状況を再現
        return 10.0

    loop._can_reader.read_speed = AsyncMock(side_effect=_read_and_stop)
    _set_t(loop, 5.0)
    await loop._execute_one_cycle()
    m["accel"].move_to_position.assert_not_awaited()
    m["brake"].move_to_position.assert_not_awaited()


async def test_wedged_cycle_triggers_emergency() -> None:
    loop, m = _make_loop(_make_schedule())
    loop._running = True
    # 前サイクルが完了していない状況を作る（wedged 監視の閾値直前まで積む）
    stuck = asyncio.ensure_future(asyncio.sleep(10))
    loop._cycle_task = stuck
    loop._consecutive_skips = int(WEDGED_CYCLE_TIMEOUT_S / loop._interval_s) - 1
    try:
        loop._schedule_next_cycle()
        await asyncio.sleep(0)  # 非常停止タスクを走らせる
        m["on_emergency"].assert_awaited_once()
        assert loop.is_running is False
    finally:
        stuck.cancel()


async def test_uncaught_cycle_exception_triggers_emergency() -> None:
    loop, m = _make_loop(_make_schedule())
    loop._running = True

    async def _boom() -> None:
        raise RuntimeError("cycle blew up")

    task = asyncio.ensure_future(_boom())
    try:
        await task
    except RuntimeError:
        pass
    loop._on_cycle_done(task)
    await asyncio.sleep(0)
    m["on_emergency"].assert_awaited_once()
    assert loop.is_running is False


async def test_start_then_stop_and_join_lifecycle() -> None:
    loop, _m = _make_loop(_make_schedule())
    loop.start()
    assert loop.is_running is True
    await loop.stop_and_join()  # 進行中サイクルタスクが無いので即 return
    assert loop.is_running is False


async def test_stall_summary_accumulates_on_resolved_skip() -> None:
    """S1 回帰テスト: CycleLoopBase 統合により ScheduleLoop でもストール計測が有効になる
    （旧実装は DriveLoop のみで計測していた）。"""
    loop, _m = _make_loop(_make_schedule())
    loop._running = True
    loop._consecutive_skips = 3

    loop._schedule_next_cycle()  # 前タスクなし → 正常起動＝直前のストールが解消
    try:
        summary = loop.stall_summary
        assert summary["stall_count"] == 1.0
        assert summary["stall_total_s"] == pytest.approx(3 * loop._interval_s)
        assert summary["stall_max_s"] == pytest.approx(3 * loop._interval_s)
    finally:
        loop.stop()
        assert loop._cycle_task is not None
        await loop._cycle_task


def test_stall_summary_is_zero_before_any_stall() -> None:
    loop, _m = _make_loop(_make_schedule())
    summary = loop.stall_summary
    assert summary == {"stall_count": 0.0, "stall_total_s": 0.0, "stall_max_s": 0.0}


async def test_button_events_fire_in_time_order() -> None:
    events = [
        ButtonEvent(time_s=3.0, channel=2, press_duration_s=0.5),
        ButtonEvent(time_s=1.0, channel=1, press_duration_s=0.5),
    ]
    loop, m = _make_loop(_make_schedule(button_events=events))
    _set_t(loop, 2.0)  # t=2 → time_s=1.0 のみ発火（3.0 は未来）
    await loop._execute_one_cycle()
    await asyncio.sleep(0)
    m["button"].press.assert_awaited_once_with(1, 0.5)
