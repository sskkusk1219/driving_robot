"""DriveLoop のユニットテスト。"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.domain.control.drive_loop import MAX_PENDING_LOG_TASKS, WEDGED_CYCLE_TIMEOUT_S, DriveLoop
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pid import PIDController
from src.domain.model_training import LOOKAHEAD_HORIZONS_S
from src.models.calibration import CalibrationData
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile

# レートリミット・ディレイを無効化した調停定数。1 サイクルで定常開度に到達させ、
# 開度値そのものを検証するテストで使う。
FAST_ARBITER_PARAMS = FeedforwardParams(
    switch_hysteresis_pct=0.0,
    accel_reengage_dwell_s=0.0,
    accel_rate_limit_pct_s=0.0,  # 0 = 無制限
    brake_rate_limit_pct_s=0.0,
    accel_deadband_pct=0.0,
    brake_deadband_pct=0.0,
)

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
    ffp: FeedforwardParams | None = None,
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
        feedforward_params=ffp if ffp is not None else FAST_ARBITER_PARAMS,
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


def _make_ff(effort: float = 50.0) -> MagicMock:
    ff = MagicMock(spec=FeedforwardController)
    ff.predict_effort = MagicMock(return_value=effort)
    # DriveLoop は ff.horizons を反復して先読み速度を組むため、実タプルを設定する
    ff.horizons = LOOKAHEAD_HORIZONS_S
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
# _ref_speed_at（基準軌跡の補間・先読みサンプリング）
# ---------------------------------------------------------------------------


class TestRefSpeedAt:
    def test_at_start_returns_first_point_speed(self) -> None:
        dl = _make_loop(mode=_make_mode())
        assert dl._ref_speed_at(0.0) == pytest.approx(0.0)

    def test_at_end_returns_last_point_speed(self) -> None:
        dl = _make_loop(mode=_make_mode())
        assert dl._ref_speed_at(10.0) == pytest.approx(60.0)

    def test_interpolates_midpoint(self) -> None:
        dl = _make_loop(mode=_make_mode())
        assert dl._ref_speed_at(2.5) == pytest.approx(30.0)

    def test_before_first_point_returns_first_speed(self) -> None:
        points = [
            SpeedPoint(time_s=1.0, speed_kmh=30.0),
            SpeedPoint(time_s=5.0, speed_kmh=60.0),
        ]
        dl = _make_loop(mode=_make_mode(points=points, total_duration=5.0))
        assert dl._ref_speed_at(0.0) == pytest.approx(30.0)

    def test_beyond_last_point_clamps_to_last_speed(self) -> None:
        """先読み（elapsed + horizon）が軌跡末尾を超えても終端値でクランプ。"""
        dl = _make_loop(mode=_make_mode())
        assert dl._ref_speed_at(999.0) == pytest.approx(60.0)

    def test_empty_points_returns_zero(self) -> None:
        mode = _make_mode(points=[], total_duration=10.0)
        dl = _make_loop(mode=mode)
        assert dl._ref_speed_at(5.0) == 0.0

    def test_single_point_mode(self) -> None:
        mode = _make_mode(points=[SpeedPoint(time_s=0.0, speed_kmh=50.0)], total_duration=10.0)
        dl = _make_loop(mode=mode)
        assert dl._ref_speed_at(5.0) == pytest.approx(50.0)


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


class TestNoModelBootstrap:
    @pytest.mark.asyncio
    async def test_no_model_skips_predict_and_uses_pid_only(self) -> None:
        """モデル未ロード（has_model=False）では FF を呼ばず PID のみで開度を決める。

        初回学習走行のブートストラップ。ref>actual の正誤差で PID がアクセルを出すこと。
        """
        ff = _make_ff(effort=50.0)
        ff.has_model = False  # モデル未ロード
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        accel_driver = _make_accel_driver()
        # ref_speed(t=0)=60, actual=0 で正の誤差を作る（PID がアクセルを出す）
        mode = _make_mode(
            points=[SpeedPoint(time_s=0.0, speed_kmh=60.0), SpeedPoint(time_s=10.0, speed_kmh=60.0)]
        )
        can_reader = _make_can_reader(speed=0.0)

        dl = _make_loop(ff=ff, pid=pid, accel_driver=accel_driver, can_reader=can_reader, mode=mode)

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        ff.predict_effort.assert_not_called()
        # アクセルに位置指令が出ている（PID 補正分）
        accel_driver.move_to_position.assert_called_once()
        assert dl.current_accel_opening > 0.0


class TestEffortArbitration:
    @pytest.mark.asyncio
    async def test_positive_effort_drives_accel_and_zero_brake(self) -> None:
        """正の努力量（FF アクセル）ではブレーキ開度 0%（zero_pos 指令）になる。"""
        ff = _make_ff(effort=30.0)
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

        brake_call_pos = brake_driver.move_to_position.call_args[0][0]
        calib = _make_calibration()
        assert brake_call_pos == calib.brake_zero_pos
        assert dl.current_accel_opening == pytest.approx(30.0)

    @pytest.mark.asyncio
    async def test_ff_brake_not_cancelled_by_small_positive_pid(self) -> None:
        """レビュー #1 回帰テスト: FF ブレーキ中の微小正 PID でブレーキが全解除されない。

        旧実装（アクセル優先排他）は raw_accel=+0.1 でも FF ブレーキ 15% を 0 にしていた。
        努力量合成では -15 + 0.1 = -14.9 → ブレーキ維持。
        """
        ff = _make_ff(effort=-15.0)
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        # 平坦 50km/h 基準で actual=49.9 → 誤差 +0.1 → PID +0.1（旧実装ならブレーキ全解除）
        mode = _make_mode(
            points=[SpeedPoint(time_s=0.0, speed_kmh=50.0), SpeedPoint(time_s=10.0, speed_kmh=50.0)]
        )
        can_reader = _make_can_reader(speed=49.9)
        accel_driver = _make_accel_driver()
        brake_driver = _make_brake_driver()

        dl = _make_loop(
            ff=ff,
            pid=pid,
            mode=mode,
            can_reader=can_reader,
            accel_driver=accel_driver,
            brake_driver=brake_driver,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        assert dl.current_brake_opening == pytest.approx(14.9)
        assert dl.current_accel_opening == 0.0

    @pytest.mark.asyncio
    async def test_pid_braking_demand_overcomes_ff_accel(self) -> None:
        """レビュー #1 回帰テスト: FF アクセル中でも PID の減速要求が努力量を負にすれば
        ブレーキへ切り替わる（旧実装は減速権限ゼロだった）。"""
        ff = _make_ff(effort=10.0)
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_limit=50.0)
        # 平坦 50km/h 基準で actual=52 → 誤差 -2 → PID -20 → 努力量 -10 → ブレーキ
        mode = _make_mode(
            points=[SpeedPoint(time_s=0.0, speed_kmh=50.0), SpeedPoint(time_s=10.0, speed_kmh=50.0)]
        )
        can_reader = _make_can_reader(speed=52.0)

        dl = _make_loop(ff=ff, pid=pid, mode=mode, can_reader=can_reader)

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        assert dl.current_brake_opening == pytest.approx(10.0)
        assert dl.current_accel_opening == 0.0

    @pytest.mark.asyncio
    async def test_clamp_to_max_opening(self) -> None:
        """FF が 200% を返しても max_accel_opening=80 にクランプされること。"""
        ff = _make_ff(effort=200.0)
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
        # クランプされた事実が飽和フラグとして次サイクルの PID に伝わる
        assert dl._saturated_high is True


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

        with (
            patch.object(asyncio, "get_running_loop") as mock_loop,
            patch.object(asyncio, "ensure_future") as mock_ensure,
        ):
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

        with (
            patch.object(asyncio, "get_running_loop") as mock_loop,
            patch.object(asyncio, "ensure_future") as mock_ensure,
        ):
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


# ---------------------------------------------------------------------------
# current_accel_opening / current_brake_opening プロパティ
# ---------------------------------------------------------------------------


class TestCurrentOpeningProperties:
    def test_initial_values_are_zero(self) -> None:
        dl = _make_loop()
        assert dl.current_accel_opening == 0.0
        assert dl.current_brake_opening == 0.0

    @pytest.mark.asyncio
    async def test_properties_updated_after_cycle(self) -> None:
        ff = _make_ff(effort=40.0)
        can_reader = _make_can_reader(speed=50.0)

        dl = _make_loop(ff=ff, can_reader=can_reader)
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 1.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        # FF が +40 の努力量を返し PID ゲイン 0 なので accel_opening=40.0, brake_opening=0.0
        assert dl.current_accel_opening == pytest.approx(40.0)
        assert dl.current_brake_opening == pytest.approx(0.0)

    @pytest.mark.asyncio
    async def test_brake_opening_set_when_ff_predicts_brake(self) -> None:
        ff = _make_ff(effort=-30.0)
        can_reader = _make_can_reader(speed=60.0)

        dl = _make_loop(ff=ff, can_reader=can_reader)
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 1.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        assert dl.current_brake_opening == pytest.approx(30.0)


# ---------------------------------------------------------------------------
# 停止・非常停止との競合（レビュー修正 C3/C4/C8 の回帰テスト）
# ---------------------------------------------------------------------------


class TestStopDuringCycle:
    @pytest.mark.asyncio
    async def test_cycle_aborts_actuator_write_if_stopped_during_can_read(self) -> None:
        """CAN 読み取りの await 中に stop() された場合、位置指令を送らないこと。

        非常停止後の home_return と競合する位置指令（EMERGENCY 後のペダル再押下）を防ぐ。
        """
        accel_driver = _make_accel_driver()
        brake_driver = _make_brake_driver()
        can_reader = MagicMock()
        dl = _make_loop(
            accel_driver=accel_driver,
            brake_driver=brake_driver,
            can_reader=can_reader,
        )

        async def read_speed_then_stop() -> float:
            # 読み取り待機中に非常停止が drive_loop.stop() を呼んだ状況を模擬
            dl.stop()
            return 60.0

        can_reader.read_speed = read_speed_then_stop
        dl._running = True
        dl._started_at = asyncio.get_running_loop().time()

        await dl._execute_one_cycle()

        accel_driver.move_to_position.assert_not_called()
        brake_driver.move_to_position.assert_not_called()

    @pytest.mark.asyncio
    async def test_schedule_skips_when_previous_cycle_still_running(self) -> None:
        """前サイクル未完了時は新サイクルを起動しない（重複実行ガード）。"""
        dl = _make_loop()
        dl._running = True

        pending = asyncio.get_running_loop().create_future()
        dl._cycle_task = asyncio.ensure_future(_wait_forever(pending))
        try:
            with patch.object(asyncio, "ensure_future") as mock_ensure:
                with patch.object(asyncio, "get_running_loop") as mock_loop:
                    mock_loop.return_value = MagicMock()
                    dl._schedule_next_cycle()
                mock_ensure.assert_not_called()
        finally:
            pending.set_result(None)
            await dl._cycle_task

    @pytest.mark.asyncio
    async def test_cycle_task_reference_is_retained(self) -> None:
        """サイクルタスクへの強参照を保持する（GC によるタスク消失防止）。"""
        dl = _make_loop()
        dl._running = True
        dl._started_at = asyncio.get_running_loop().time()

        dl._schedule_next_cycle()
        try:
            assert dl._cycle_task is not None
            await dl._cycle_task
        finally:
            dl.stop()

    @pytest.mark.asyncio
    async def test_uncaught_cycle_exception_triggers_emergency(self) -> None:
        """サイクル内の未捕捉例外は黙殺せず停止 + 非常停止すること。"""
        on_emergency = AsyncMock()
        ff = _make_ff()
        ff.predict_effort = MagicMock(side_effect=RuntimeError("unexpected"))
        ff.has_model = True
        dl = _make_loop(ff=ff, on_emergency=on_emergency)
        dl._running = True
        dl._started_at = asyncio.get_running_loop().time()

        dl._schedule_next_cycle()
        assert dl._cycle_task is not None
        # タスク完了と done_callback → 非常停止タスクの実行を待つ
        try:
            await dl._cycle_task
        except RuntimeError:
            pass
        await asyncio.sleep(0)
        if dl._emergency_task is not None:
            await dl._emergency_task

        assert dl.is_running is False
        on_emergency.assert_awaited_once()


async def _wait_forever(fut: asyncio.Future[None]) -> None:
    await fut


# ---------------------------------------------------------------------------
# ウォッチドッグ（サイクル長時間未完了）
# ---------------------------------------------------------------------------


class TestWedgedCycleWatchdog:
    @pytest.mark.asyncio
    async def test_emergency_after_consecutive_skips_exceed_timeout(self) -> None:
        """連続スキップが WEDGED_CYCLE_TIMEOUT_S に達したら非常停止する（レビュー #16）。

        Modbus がハングするとサイクルが完了せず、逸脱・過電流チェックが一切走らない
        ままペダルが最終指令位置で凍結する。pymodbus のリトライ上限（十数秒）を
        待たずに 1 秒で安全側へ倒す。
        """
        on_emergency = AsyncMock()
        dl = _make_loop(on_emergency=on_emergency)
        dl._running = True

        pending = asyncio.get_running_loop().create_future()
        dl._cycle_task = asyncio.ensure_future(_wait_forever(pending))
        # 次のスキップで wedged_s がしきい値に到達する状態を作る
        dl._consecutive_skips = int(WEDGED_CYCLE_TIMEOUT_S / dl._interval_s) - 1
        try:
            dl._schedule_next_cycle()
            assert not dl.is_running
            assert dl._emergency_task is not None
            await dl._emergency_task
            on_emergency.assert_awaited_once()
        finally:
            pending.set_result(None)
            await dl._cycle_task

    @pytest.mark.asyncio
    async def test_skip_counter_resets_on_successful_schedule(self) -> None:
        dl = _make_loop()
        dl._running = True
        dl._consecutive_skips = 5
        dl._started_at = asyncio.get_running_loop().time()

        dl._schedule_next_cycle()  # 前タスクなし → 正常起動
        try:
            assert dl._consecutive_skips == 0
            assert dl._cycle_task is not None
            await dl._cycle_task
        finally:
            dl.stop()


# ---------------------------------------------------------------------------
# stop_and_join（飛行中サイクルの停止完了待ち）
# ---------------------------------------------------------------------------


class TestStopAndJoin:
    @pytest.mark.asyncio
    async def test_waits_for_inflight_cycle(self) -> None:
        """進行中のサイクルが完了してから戻る（home_return との競合防止、レビュー #6）。"""
        dl = _make_loop()
        finished = False

        async def slow_cycle() -> None:
            nonlocal finished
            await asyncio.sleep(0.05)
            finished = True

        dl._cycle_task = asyncio.ensure_future(slow_cycle())
        await dl.stop_and_join(timeout_s=1.0)
        assert finished
        assert not dl.is_running

    @pytest.mark.asyncio
    async def test_cancels_wedged_cycle_after_timeout(self) -> None:
        """タイムアウトしたサイクルはキャンセルし、非常停止をそれ以上遅延させない。"""
        dl = _make_loop()
        pending = asyncio.get_running_loop().create_future()
        dl._cycle_task = asyncio.ensure_future(_wait_forever(pending))

        await dl.stop_and_join(timeout_s=0.05)
        await asyncio.sleep(0)  # キャンセル伝播
        assert dl._cycle_task.cancelled()

    @pytest.mark.asyncio
    async def test_noop_when_no_cycle_task(self) -> None:
        dl = _make_loop()
        await dl.stop_and_join()
        assert not dl.is_running


# ---------------------------------------------------------------------------
# ログ保留タスク上限
# ---------------------------------------------------------------------------


class TestLogBacklogCap:
    def test_enqueue_skipped_when_backlog_full(self) -> None:
        """保留タスクが上限に達したら新規ログをスキップする（レビュー #13）。"""
        log_writer = MagicMock()
        log_writer.write_log = AsyncMock()
        dl = _make_loop(log_writer=log_writer, session_id="s1")
        dl._pending_log_tasks = {MagicMock() for _ in range(MAX_PENDING_LOG_TASKS)}

        with patch.object(asyncio, "ensure_future") as mock_ensure:
            dl._enqueue_log_write(MagicMock())

        mock_ensure.assert_not_called()
        assert dl._log_backlog_active is True

    def test_enqueue_resumes_after_backlog_clears(self) -> None:
        log_writer = MagicMock()
        log_writer.write_log = AsyncMock()
        dl = _make_loop(log_writer=log_writer, session_id="s1")
        dl._log_backlog_active = True  # 滞留中だった
        dl._pending_log_tasks = set()  # 解消

        with patch.object(asyncio, "ensure_future") as mock_ensure:
            mock_ensure.return_value = MagicMock()
            dl._enqueue_log_write(MagicMock())

        mock_ensure.assert_called_once()
        assert dl._log_backlog_active is False


# ---------------------------------------------------------------------------
# KPI 計測・スナップショット鮮度
# ---------------------------------------------------------------------------


class TestKPIIntegration:
    @pytest.mark.asyncio
    async def test_kpi_samples_collected_each_cycle(self) -> None:
        dl = _make_loop(can_reader=_make_can_reader(speed=10.0))
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 1.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        summary = dl.kpi_summary
        assert summary["n_samples"] == 1.0
        # t=1.0s の基準は 12km/h、actual=10 → 偏差 2.0 が最大偏差として記録される
        assert summary["max_abs_deviation_kmh"] == pytest.approx(2.0)

    def test_start_resets_kpi(self) -> None:
        dl = _make_loop()
        dl._kpi.update(50.0, 49.0, 0.0)
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            loop_obj.call_later = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
        assert dl.kpi_summary["n_samples"] == 0.0


class TestSnapshotFreshness:
    @pytest.mark.asyncio
    async def test_snapshot_has_captured_at(self) -> None:
        """スナップショットにイベントループ時刻が記録される（凍結検知の基盤）。"""
        dl = _make_loop()
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 1.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        assert dl.last_snapshot is not None
        assert dl.last_snapshot.captured_at == pytest.approx(1.0)
