"""DriveLoop のユニットテスト。"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.domain.control.drive_loop import DriveLoop
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pid import PIDController
from src.models.calibration import CalibrationData
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import PIDGains, StopConfig, VehicleProfile

# ---------------------------------------------------------------------------
# ヘルパー
# ---------------------------------------------------------------------------


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


def _make_profile(
    max_accel: float = 80.0,
    max_brake: float = 80.0,
    deviation_threshold: float = 2.0,
    deviation_duration: float = 4.0,
) -> VehicleProfile:
    return VehicleProfile(
        id="profile-1",
        name="Test",
        max_accel_opening=max_accel,
        max_brake_opening=max_brake,
        max_speed=120.0,
        max_decel_g=0.5,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(
            deviation_threshold_kmh=deviation_threshold,
            deviation_duration_s=deviation_duration,
        ),
        calibration=_make_calibration(),
        model_path=None,
        created_at=datetime(2026, 1, 1),
        updated_at=datetime(2026, 1, 1),
    )


def _make_mode(
    points: list[SpeedPoint] | None = None,
    total_duration: float = 10.0,
) -> DrivingMode:
    if points is None:
        points = [
            SpeedPoint(time_s=0.0, speed_kmh=0.0),
            SpeedPoint(time_s=5.0, speed_kmh=60.0),
            SpeedPoint(time_s=10.0, speed_kmh=60.0),
        ]
    return DrivingMode(
        id="mode-1",
        name="Test Mode",
        description="",
        reference_speed=points,
        total_duration=total_duration,
        max_speed=60.0,
        created_at=datetime(2026, 1, 1),
    )


def _make_ff() -> FeedforwardController:
    ff = MagicMock(spec=FeedforwardController)
    ff.predict = MagicMock(return_value=(50.0, 0.0))
    return ff


def _make_pid() -> PIDController:
    return PIDController(kp=0.0, ki=0.0, kd=0.0)


def _make_accel_driver() -> MagicMock:
    d = MagicMock()
    d.move_to_position = AsyncMock()
    d.read_current = AsyncMock(return_value=500.0)
    return d


def _make_brake_driver() -> MagicMock:
    d = MagicMock()
    d.move_to_position = AsyncMock()
    d.read_current = AsyncMock(return_value=300.0)
    return d


def _make_can_reader(speed: float = 60.0) -> MagicMock:
    r = MagicMock()
    r.read_speed = AsyncMock(return_value=speed)
    return r


def _make_safety_check(
    overcurrent: bool = False,
    deviation: bool = False,
) -> MagicMock:
    sc = MagicMock()
    sc.check_overcurrent = MagicMock(return_value=overcurrent)
    sc.check_deviation = MagicMock(return_value=deviation)
    return sc


def _make_loop(
    *,
    ff: FeedforwardController | None = None,
    pid: PIDController | None = None,
    accel_driver: MagicMock | None = None,
    brake_driver: MagicMock | None = None,
    can_reader: MagicMock | None = None,
    profile: VehicleProfile | None = None,
    mode: DrivingMode | None = None,
    safety_check: MagicMock | None = None,
    on_complete: Callable[[], Awaitable[None]] | None = None,
    on_emergency: Callable[[], Awaitable[None]] | None = None,
    log_writer: MagicMock | None = None,
    session_id: str | None = None,
) -> DriveLoop:
    return DriveLoop(
        ff_controller=ff or _make_ff(),
        pid=pid or _make_pid(),
        accel_driver=accel_driver or _make_accel_driver(),
        brake_driver=brake_driver or _make_brake_driver(),
        can_reader=can_reader or _make_can_reader(),
        profile=profile or _make_profile(),
        mode=mode or _make_mode(),
        safety_check=safety_check or _make_safety_check(),
        on_complete=on_complete or AsyncMock(),
        on_emergency=on_emergency or AsyncMock(),
        log_writer=log_writer,
        session_id=session_id,
    )


# ---------------------------------------------------------------------------
# _get_ref_speed_and_accel
# ---------------------------------------------------------------------------


class TestGetRefSpeedAndAccel:
    def test_at_start_returns_first_point_speed(self) -> None:
        dl = _make_loop(mode=_make_mode())
        speed, _ = dl._get_ref_speed_and_accel(0.0)
        assert speed == pytest.approx(0.0)

    def test_at_end_returns_last_point_speed(self) -> None:
        dl = _make_loop(mode=_make_mode())
        speed, accel = dl._get_ref_speed_and_accel(10.0)
        assert speed == pytest.approx(60.0)
        assert accel == pytest.approx(0.0)

    def test_interpolates_midpoint(self) -> None:
        dl = _make_loop(mode=_make_mode())
        speed, accel = dl._get_ref_speed_and_accel(2.5)
        assert speed == pytest.approx(30.0)
        assert accel == pytest.approx(12.0)

    def test_accel_computed_as_speed_difference_over_dt(self) -> None:
        points = [
            SpeedPoint(time_s=0.0, speed_kmh=0.0),
            SpeedPoint(time_s=4.0, speed_kmh=80.0),
        ]
        dl = _make_loop(mode=_make_mode(points=points, total_duration=4.0))
        _, accel = dl._get_ref_speed_and_accel(2.0)
        assert accel == pytest.approx(20.0)

    def test_before_first_point_returns_first_speed(self) -> None:
        points = [
            SpeedPoint(time_s=1.0, speed_kmh=30.0),
            SpeedPoint(time_s=5.0, speed_kmh=60.0),
        ]
        dl = _make_loop(mode=_make_mode(points=points, total_duration=5.0))
        speed, _ = dl._get_ref_speed_and_accel(0.0)
        assert speed == pytest.approx(30.0)

    def test_empty_points_returns_zero(self) -> None:
        mode = _make_mode(points=[], total_duration=10.0)
        dl = _make_loop(mode=mode)
        speed, accel = dl._get_ref_speed_and_accel(5.0)
        assert speed == 0.0
        assert accel == 0.0

    def test_single_point_mode(self) -> None:
        mode = _make_mode(points=[SpeedPoint(time_s=0.0, speed_kmh=50.0)], total_duration=10.0)
        dl = _make_loop(mode=mode)
        speed, _ = dl._get_ref_speed_and_accel(5.0)
        assert speed == pytest.approx(50.0)


# ---------------------------------------------------------------------------
# _opening_to_position
# ---------------------------------------------------------------------------


class TestOpeningToPosition:
    def test_zero_opening(self) -> None:
        dl = _make_loop()
        pos = dl._opening_to_position(0.0, zero_pos=100, full_pos=600)
        assert pos == 100

    def test_full_opening(self) -> None:
        dl = _make_loop()
        pos = dl._opening_to_position(100.0, zero_pos=100, full_pos=600)
        assert pos == 600

    def test_half_opening(self) -> None:
        dl = _make_loop()
        pos = dl._opening_to_position(50.0, zero_pos=100, full_pos=600)
        assert pos == 350

    def test_rounding(self) -> None:
        dl = _make_loop()
        # 1/3 of stroke 300 = 100; zero=0 → 100
        pos = dl._opening_to_position(100.0 / 3.0, zero_pos=0, full_pos=300)
        assert pos == 100


# ---------------------------------------------------------------------------
# FF+PID 合成・排他制御・クランプ
# ---------------------------------------------------------------------------


class TestOpeningComputation:
    @pytest.mark.asyncio
    async def test_accel_priority_exclusive_control(self) -> None:
        """FF がアクセル、PID 補正がブレーキ方向でも、アクセルがあればブレーキをゼロにする。"""
        ff = MagicMock(spec=FeedforwardController)
        ff.predict = MagicMock(return_value=(30.0, 10.0))
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0)

        accel_driver = _make_accel_driver()
        brake_driver = _make_brake_driver()

        dl = _make_loop(ff=ff, pid=pid, accel_driver=accel_driver, brake_driver=brake_driver)

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        # ブレーキに位置指令が出ているが開度 0% なので zero_pos に一致
        brake_call_pos = brake_driver.move_to_position.call_args[0][0]
        calib = _make_calibration()
        assert brake_call_pos == calib.brake_zero_pos

    @pytest.mark.asyncio
    async def test_clamp_to_max_opening(self) -> None:
        """FF が 200% を返しても max_accel_opening=80 にクランプされること。"""
        ff = MagicMock(spec=FeedforwardController)
        ff.predict = MagicMock(return_value=(200.0, 0.0))
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0)
        profile = _make_profile(max_accel=80.0)
        accel_driver = _make_accel_driver()

        dl = _make_loop(ff=ff, pid=pid, profile=profile, accel_driver=accel_driver)

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        accel_pos = accel_driver.move_to_position.call_args[0][0]
        calib = _make_calibration()
        expected_pos = calib.accel_zero_pos + round(
            (calib.accel_full_pos - calib.accel_zero_pos) * 80.0 / 100.0
        )
        assert accel_pos == expected_pos


# ---------------------------------------------------------------------------
# 正常完了コールバック
# ---------------------------------------------------------------------------


class TestNormalCompletion:
    @pytest.mark.asyncio
    async def test_on_complete_called_when_elapsed_exceeds_duration(self) -> None:
        on_complete = AsyncMock()
        dl = _make_loop(
            mode=_make_mode(total_duration=5.0),
            on_complete=on_complete,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 10.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        on_complete.assert_called_once()

    @pytest.mark.asyncio
    async def test_stop_called_before_on_complete(self) -> None:
        on_complete = AsyncMock()
        dl = _make_loop(
            mode=_make_mode(total_duration=5.0),
            on_complete=on_complete,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 10.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        assert not dl.is_running


# ---------------------------------------------------------------------------
# 過電流検知
# ---------------------------------------------------------------------------


class TestOvercurrentEmergency:
    @pytest.mark.asyncio
    async def test_on_emergency_called_on_accel_overcurrent(self) -> None:
        on_emergency = AsyncMock()
        safety_check = _make_safety_check()

        accel_driver = _make_accel_driver()
        accel_driver.read_current = AsyncMock(return_value=5000.0)

        def overcurrent_side_effect(current_ma: float, axis: str) -> bool:
            return current_ma > 3000.0

        safety_check.check_overcurrent = MagicMock(side_effect=overcurrent_side_effect)

        dl = _make_loop(
            accel_driver=accel_driver,
            safety_check=safety_check,
            on_emergency=on_emergency,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        on_emergency.assert_called_once()
        assert not dl.is_running

    @pytest.mark.asyncio
    async def test_on_emergency_called_on_brake_overcurrent(self) -> None:
        on_emergency = AsyncMock()
        safety_check = _make_safety_check()

        brake_driver = _make_brake_driver()
        brake_driver.read_current = AsyncMock(return_value=5000.0)

        call_count = 0

        def overcurrent_side_effect(current_ma: float, axis: str) -> bool:
            nonlocal call_count
            call_count += 1
            # 1 回目は accel (正常)、2 回目は brake (過電流)
            return call_count == 2

        safety_check.check_overcurrent = MagicMock(side_effect=overcurrent_side_effect)

        dl = _make_loop(
            brake_driver=brake_driver,
            safety_check=safety_check,
            on_emergency=on_emergency,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        on_emergency.assert_called_once()


# ---------------------------------------------------------------------------
# 逸脱超過
# ---------------------------------------------------------------------------


class TestDeviationEmergency:
    @pytest.mark.asyncio
    async def test_on_emergency_called_when_deviation_exceeds_duration(self) -> None:
        on_emergency = AsyncMock()
        safety_check = _make_safety_check(deviation=True)

        dl = _make_loop(
            safety_check=safety_check,
            on_emergency=on_emergency,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        on_emergency.assert_called_once()
        assert not dl.is_running

    @pytest.mark.asyncio
    async def test_deviation_start_reset_when_deviation_clears(self) -> None:
        """逸脱が解消したとき _deviation_start が None にリセットされること。"""
        safety_check = _make_safety_check(deviation=False)
        # elapsed=0 のとき ref_speed=0.0。can_reader も 0.0 を返すので deviation=0 < threshold
        dl = _make_loop(safety_check=safety_check, can_reader=_make_can_reader(speed=0.0))
        dl._deviation_start = 1.0

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        assert dl._deviation_start is None


# ---------------------------------------------------------------------------
# CAN エラー
# ---------------------------------------------------------------------------


class TestCANErrorEmergency:
    @pytest.mark.asyncio
    async def test_on_emergency_called_when_can_read_fails(self) -> None:
        on_emergency = AsyncMock()
        can_reader = MagicMock()
        can_reader.read_speed = AsyncMock(side_effect=OSError("CAN timeout"))

        dl = _make_loop(
            can_reader=can_reader,
            on_emergency=on_emergency,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        on_emergency.assert_called_once()
        assert not dl.is_running


# ---------------------------------------------------------------------------
# ログ書き込み
# ---------------------------------------------------------------------------


class TestLogWriting:
    @pytest.mark.asyncio
    async def test_log_written_every_two_cycles(self) -> None:
        """LOG_EVERY_N_CYCLES = 2 なので、cycle_count % 2 == 0 のときのみ書き込む。"""
        log_writer = MagicMock()
        log_writer.write_log = AsyncMock()
        session_id = "session-1"

        dl = _make_loop(
            log_writer=log_writer,
            session_id=session_id,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop, patch.object(
            asyncio, "ensure_future"
        ) as mock_ensure:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0
            dl._cycle_count = 0

            # 1 回目のサイクル: cycle_count が 1 → ログなし
            await dl._execute_one_cycle()
            assert mock_ensure.call_count == 0

            dl._running = True
            dl._cycle_count = 1

            # 2 回目のサイクル: cycle_count が 2 → LOG_EVERY_N_CYCLES=2 の倍数 → ログあり
            await dl._execute_one_cycle()
            assert mock_ensure.call_count == 1

    @pytest.mark.asyncio
    async def test_no_log_written_without_log_writer(self) -> None:
        dl = _make_loop(log_writer=None, session_id="session-1")

        with patch.object(asyncio, "get_running_loop") as mock_loop, patch.object(
            asyncio, "ensure_future"
        ) as mock_ensure:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0
            dl._cycle_count = 1

            await dl._execute_one_cycle()

        mock_ensure.assert_not_called()


# ---------------------------------------------------------------------------
# start / stop 動作
# ---------------------------------------------------------------------------


class TestStartStop:
    def test_start_sets_running_true(self) -> None:
        dl = _make_loop()
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            loop_obj.call_later = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
        assert dl.is_running

    def test_stop_sets_running_false(self) -> None:
        dl = _make_loop()
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            loop_obj.call_later = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
        dl.stop()
        assert not dl.is_running

    def test_double_start_is_idempotent(self) -> None:
        dl = _make_loop()
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            loop_obj.call_later = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
            dl.start()
            # call_later は 1 回だけ呼ばれること
            assert loop_obj.call_later.call_count == 1

    @pytest.mark.asyncio
    async def test_execute_one_cycle_noop_when_not_running(self) -> None:
        on_complete = AsyncMock()
        on_emergency = AsyncMock()
        can_reader = MagicMock()
        can_reader.read_speed = AsyncMock(return_value=0.0)

        dl = _make_loop(
            can_reader=can_reader,
            on_complete=on_complete,
            on_emergency=on_emergency,
        )
        dl._running = False

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 99.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        on_complete.assert_not_called()
        on_emergency.assert_not_called()
        can_reader.read_speed.assert_not_called()
