"""DriveLoop のユニットテスト。"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.domain.control.conversions import opening_to_position
from src.domain.control.drive_loop import MAX_PENDING_LOG_TASKS, WEDGED_CYCLE_TIMEOUT_S, DriveLoop
from src.domain.control.feedforward import FeedforwardController, GainSchedule
from src.domain.control.pedal_plan import PedalPlan, PlanPhase
from src.domain.control.pid import PIDController
from src.domain.control.trim import TrimController
from src.domain.model_training import DEFAULT_FEATURE_SPEC
from src.models.calibration import CalibrationData
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import (
    DynamicsParams,
    FeedforwardParams,
    PIDGains,
    StopConfig,
    VehicleProfile,
)
from src.models.system_state import RealtimeSnapshot

LOOKAHEAD_HORIZONS_S = DEFAULT_FEATURE_SPEC.lookahead_horizons_s
PAST_HORIZONS_S = DEFAULT_FEATURE_SPEC.past_horizons_s

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
    dynamics_params: DynamicsParams | None = None,
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
        dynamics_params=dynamics_params if dynamics_params is not None else DynamicsParams(),
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
    # DriveLoop は ff.horizons / ff.past_horizons を反復して先読み・過去速度を組むため実タプルを設定
    ff.horizons = LOOKAHEAD_HORIZONS_S
    ff.past_horizons = PAST_HORIZONS_S
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
    disable_deviation_check: bool = False,
    ilc: object | None = None,
    plan: PedalPlan | None = None,
) -> DriveLoop:
    # デフォルトは plan=None＝従来経路（FF 毎サイクル＋速い補正層 PID 直結）。既存テストは
    # この従来合成（ff_effort + pid_u）を検証する。プラン経路は plan を明示指定するテストで。
    return DriveLoop(
        ff_controller=ff or _make_ff(),
        trim=TrimController(pid or _make_pid()),
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
        disable_deviation_check=disable_deviation_check,
        ilc=ilc,  # type: ignore[arg-type]
        plan=plan,
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
    """A1 レビュー指摘: 開度→パルス変換は src.domain.control.conversions に一本化済み。"""

    def test_zero_opening(self) -> None:
        pos = opening_to_position(0.0, zero_pos=100, full_pos=600)
        assert pos == 100

    def test_full_opening(self) -> None:
        pos = opening_to_position(100.0, zero_pos=100, full_pos=600)
        assert pos == 600

    def test_half_opening(self) -> None:
        pos = opening_to_position(50.0, zero_pos=100, full_pos=600)
        assert pos == 350

    def test_rounding(self) -> None:
        # 1/3 of stroke 300 = 100; zero=0 → 100
        pos = opening_to_position(100.0 / 3.0, zero_pos=0, full_pos=300)
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

    @pytest.mark.asyncio
    async def test_final_sample_logged_before_completion(self) -> None:
        """完了時、停止前に末端サンプルが 1 行ログされる。

        従来は完了分岐がログを書かずに return するため t=total_duration 相当のサンプルが
        欠落していた。ref は末端値、実測系は直近 snapshot・直近指令開度を再利用する。
        """
        log_writer = MagicMock()
        log_writer.write_log = AsyncMock()
        on_complete = AsyncMock()
        # 既定軌跡 0→60(5s)→60(10s)、total_duration=5.0 → t=5.0 の末端 ref は 60.0
        dl = _make_loop(
            mode=_make_mode(total_duration=5.0),
            on_complete=on_complete,
            log_writer=log_writer,
            session_id="s1",
        )
        dl._last_snapshot = RealtimeSnapshot(
            actual_speed_kmh=58.0,
            accel_pos=321,
            brake_pos=200,
            accel_current_ma=510.0,
            brake_current_ma=300.0,
            captured_at=4.9,
        )
        dl._current_accel_opening = 12.0
        dl._current_brake_opening = 0.0

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 10.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0
            await dl._execute_one_cycle()
            await asyncio.sleep(0)  # ログ書き込みタスクを走らせる

        log_writer.write_log.assert_awaited_once()
        _sid, written = log_writer.write_log.call_args[0]
        assert written.ref_speed_kmh == pytest.approx(60.0)  # total_duration 末端値
        assert written.actual_speed_kmh == pytest.approx(58.0)  # 直近 snapshot
        assert written.accel_opening == pytest.approx(12.0)  # 直近指令開度
        assert written.accel_pos == 321
        assert written.accel_current == pytest.approx(510.0)
        on_complete.assert_called_once()
        assert not dl.is_running

    @pytest.mark.asyncio
    async def test_no_final_sample_when_snapshot_missing(self) -> None:
        """`_last_snapshot` が無い状態で完了しても例外にならず、ログも書かれない。"""
        log_writer = MagicMock()
        log_writer.write_log = AsyncMock()
        on_complete = AsyncMock()
        dl = _make_loop(
            mode=_make_mode(total_duration=5.0),
            on_complete=on_complete,
            log_writer=log_writer,
            session_id="s1",
        )
        assert dl.last_snapshot is None

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 10.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0
            await dl._execute_one_cycle()
            await asyncio.sleep(0)

        log_writer.write_log.assert_not_awaited()
        on_complete.assert_called_once()

    @pytest.mark.asyncio
    async def test_no_final_sample_without_log_writer(self) -> None:
        """log_writer が無い完了でも従来どおり例外なく終了する（回帰）。"""
        on_complete = AsyncMock()
        dl = _make_loop(mode=_make_mode(total_duration=5.0), on_complete=on_complete)
        dl._last_snapshot = RealtimeSnapshot(
            actual_speed_kmh=58.0,
            accel_pos=321,
            brake_pos=200,
            accel_current_ma=510.0,
            brake_current_ma=300.0,
            captured_at=4.9,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 10.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        on_complete.assert_called_once()
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
    async def test_deviation_check_suppressed_when_disabled(self) -> None:
        """disable_deviation_check=True では逸脱しても非常停止しない（PID 自動適合用）。

        過電流等の他の安全網は維持されるため check_deviation 自体も呼ばれない。
        """
        on_emergency = AsyncMock()
        safety_check = _make_safety_check(deviation=True)

        dl = _make_loop(
            safety_check=safety_check,
            on_emergency=on_emergency,
            disable_deviation_check=True,
        )

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0

            await dl._execute_one_cycle()

        on_emergency.assert_not_called()
        safety_check.check_deviation.assert_not_called()
        assert dl.is_running

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
            loop_obj.call_at = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
        assert dl.is_running

    def test_stop_sets_running_false(self) -> None:
        dl = _make_loop()
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            loop_obj.call_at = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
        dl.stop()
        assert not dl.is_running

    def test_double_start_is_idempotent(self) -> None:
        dl = _make_loop()
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            loop_obj.call_at = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
            dl.start()
            # 絶対時刻グリッドの予約（call_at）は 1 回だけ呼ばれること
            assert loop_obj.call_at.call_count == 1

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
# 絶対時刻グリッドスケジューリング（ドリフト除去・catch-up バースト回避）
# ---------------------------------------------------------------------------


class TestAbsoluteGridScheduling:
    def test_start_anchors_grid_at_start_time(self) -> None:
        """start() でグリッドを開始時刻に固定し、最初のサイクルを anchor+interval に予約する。"""
        dl = _make_loop()
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 42.0
            mock_loop.return_value = loop_obj
            dl.start()
        assert dl._grid_anchor == pytest.approx(42.0)
        assert dl._grid_tick == 1
        target = loop_obj.call_at.call_args[0][0]
        assert target == pytest.approx(42.0 + dl._interval_s)

    def test_arm_next_cycle_targets_grid_point(self) -> None:
        """定時発火では次 tick の絶対グリッド時刻に予約する。"""
        dl = _make_loop()
        dl._grid_anchor = 100.0
        dl._grid_tick = 5
        loop_obj = MagicMock()
        loop_obj.time.return_value = 100.27  # tick6 の 100.30 より前（定時内）
        dl._arm_next_cycle(loop_obj)
        assert dl._grid_tick == 6
        target = loop_obj.call_at.call_args[0][0]
        assert target == pytest.approx(100.30)

    def test_grid_does_not_drift_under_callback_latency(self) -> None:
        """毎サイクルのコールバック起動遅延が累積しても予約時刻はグリッドに張り付く。

        相対 call_later ではこの遅延が積もって固定周期からドリフトし、323s 走行で
        数サイクル早く終端到達＝ログ数行欠落していた。絶対グリッドではドリフトしない。
        """
        dl = _make_loop()
        dl._grid_anchor = 0.0
        dl._grid_tick = 0
        loop_obj = MagicMock()
        for n in range(1, 201):
            # 各コールバックが前グリッドから 7ms 遅れて起動する状況を模擬
            loop_obj.time.return_value = (n - 1) * dl._interval_s + 0.007
            dl._arm_next_cycle(loop_obj)
            assert dl._grid_tick == n
            target = loop_obj.call_at.call_args[0][0]
            assert target == pytest.approx(n * dl._interval_s)

    def test_late_fire_rounds_forward_without_burst(self) -> None:
        """イベントループがグリッドを跨いで停止した場合、過去時刻に予約せず未来グリッドへ丸める。"""
        dl = _make_loop()
        dl._grid_anchor = 0.0
        dl._grid_tick = 3  # 次は tick4 → 0.20 の予定
        loop_obj = MagicMock()
        loop_obj.time.return_value = 0.63  # 0.20 を大きく過ぎている（イベントループ停止）
        dl._arm_next_cycle(loop_obj)
        target = loop_obj.call_at.call_args[0][0]
        assert target > 0.63  # 過去時刻に予約しない（catch-up バースト回避）
        assert target == pytest.approx(0.65)  # 0.63 の次の未来グリッド
        assert dl._grid_tick == 13


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
                    loop_obj = MagicMock()
                    loop_obj.time.return_value = 0.0  # 絶対時刻グリッドの予約に数値時刻が要る
                    mock_loop.return_value = loop_obj
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

    @pytest.mark.asyncio
    async def test_stall_summary_accumulates_on_resolved_skip(self) -> None:
        """連続スキップが解消するたびに 1 件のストールとして回数・時間を集計する。

        ストール切り分け（.steering/20260620-modbus-retry-cycle-stall）用の計装。
        """
        dl = _make_loop()
        dl._running = True
        dl._consecutive_skips = 3
        dl._started_at = asyncio.get_running_loop().time()

        dl._schedule_next_cycle()  # 前タスクなし → 正常起動＝直前のストールが解消
        try:
            summary = dl.stall_summary
            assert summary["stall_count"] == 1.0
            assert summary["stall_total_s"] == pytest.approx(3 * dl._interval_s)
            assert summary["stall_max_s"] == pytest.approx(3 * dl._interval_s)
        finally:
            dl.stop()
            assert dl._cycle_task is not None
            await dl._cycle_task

    def test_stall_summary_is_zero_before_any_stall(self) -> None:
        dl = _make_loop()
        summary = dl.stall_summary
        assert summary == {"stall_count": 0.0, "stall_total_s": 0.0, "stall_max_s": 0.0}


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
            loop_obj.call_at = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
        assert dl.kpi_summary["n_samples"] == 0.0


class TestPauseResume:
    """一時停止／再開: 基準速度タイムライン（経過時間）の凍結と連続再開。"""

    def test_pause_records_frozen_elapsed_and_sets_paused(self) -> None:
        dl = _make_loop()
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 7.0
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 2.0
            dl.pause()
        assert dl.is_paused
        assert dl._paused_elapsed == pytest.approx(5.0)

    def test_pause_noop_when_not_running(self) -> None:
        dl = _make_loop()
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            mock_loop.return_value = loop_obj
            dl._running = False
            dl.pause()
        assert not dl.is_paused

    def test_resume_shifts_started_at_to_continue_timeline(self) -> None:
        dl = _make_loop()
        dl._paused = True
        dl._paused_elapsed = 5.0
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 20.0
            mock_loop.return_value = loop_obj
            dl.resume()
        assert not dl.is_paused
        # elapsed が凍結時点(5.0)の続きから進むよう started_at をシフト
        assert dl._started_at == pytest.approx(15.0)
        # 次サイクルの dt スパイクを避けるため計測時刻はリセットされる
        assert dl._last_cycle_time is None

    def test_resume_noop_when_not_paused(self) -> None:
        dl = _make_loop()
        dl._started_at = 3.0
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 99.0
            mock_loop.return_value = loop_obj
            dl.resume()
        assert dl._started_at == 3.0  # 変化しない

    def test_start_resets_paused(self) -> None:
        dl = _make_loop()
        dl._paused = True
        dl._paused_elapsed = 5.0
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 0.0
            loop_obj.call_at = MagicMock()
            mock_loop.return_value = loop_obj
            dl.start()
        assert not dl.is_paused
        assert dl._paused_elapsed == 0.0

    @pytest.mark.asyncio
    async def test_cycle_uses_frozen_elapsed_when_paused(self) -> None:
        """一時停止中は loop.time() に関わらず凍結経過時間の基準速度を参照する。"""
        dl = _make_loop(can_reader=_make_can_reader(speed=10.0))
        dl._running = True
        dl._paused = True
        dl._paused_elapsed = 1.0  # 0→0, 5→60 の軌跡で t=1.0s → 12km/h

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 999.0  # 実時刻は大きく進んでいる
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        assert dl.current_ref_speed == pytest.approx(12.0)

    @pytest.mark.asyncio
    async def test_paused_does_not_auto_complete(self) -> None:
        """一時停止中は実時刻が総時間を超えても正常完了しない。"""
        on_complete = AsyncMock()
        dl = _make_loop(mode=_make_mode(total_duration=5.0), on_complete=on_complete)
        dl._running = True
        dl._paused = True
        dl._paused_elapsed = 1.0

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 999.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        on_complete.assert_not_called()
        assert dl.is_running

    @pytest.mark.asyncio
    async def test_paused_skips_kpi_and_log(self) -> None:
        """一時停止中は KPI 集計とログ書き込みを行わない（保持区間で汚さない）。"""
        log_writer = MagicMock()
        log_writer.write_log = AsyncMock()
        dl = _make_loop(
            can_reader=_make_can_reader(speed=10.0),
            log_writer=log_writer,
            session_id="s1",
        )
        dl._running = True
        dl._paused = True
        dl._paused_elapsed = 1.0
        dl._cycle_count = 1  # 一時停止でなければ次サイクルで 2 になりログ対象

        with (
            patch.object(asyncio, "get_running_loop") as mock_loop,
            patch.object(asyncio, "ensure_future") as mock_ensure,
        ):
            loop_obj = MagicMock()
            loop_obj.time.return_value = 1.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        assert dl.kpi_summary["n_samples"] == 0.0
        mock_ensure.assert_not_called()

    @pytest.mark.asyncio
    async def test_paused_still_drives_actuators_to_hold_speed(self) -> None:
        """一時停止中もアクチュエータ制御は継続し、凍結した目標車速を保持する。"""
        ff = _make_ff(effort=40.0)
        accel_driver = _make_accel_driver()
        dl = _make_loop(ff=ff, accel_driver=accel_driver, can_reader=_make_can_reader(speed=50.0))
        dl._running = True
        dl._paused = True
        dl._paused_elapsed = 1.0

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 999.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        accel_driver.move_to_position.assert_called_once()
        assert dl.current_accel_opening == pytest.approx(40.0)


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


# ---------------------------------------------------------------------------
# PID 先読み補償（pid_preview_s） — FB ループのむだ時間補償
#
# Stage A（KPI 対策）で preview を FF+PID 両方への前倒しから PID 専用ノブへ分離した。
# FF は now-frame 基準で動き（先読みはモデルの horizons が担う）、PID のみ pid_preview_s
# だけ前倒しした基準を追う。preview を FF にも掛けると FF 内蔵のむだ時間補償と二重になり
# 実車速が基準を先行する系統偏差（KPI 違反の主因）を生むため。
# ---------------------------------------------------------------------------


class TestPidPreviewCompensation:
    @pytest.mark.asyncio
    async def test_ff_receives_now_frame_pid_receives_shifted(self) -> None:
        """pid_preview_s>0 のとき、FF は now-frame、PID のみ前倒しした基準を受け取る。"""
        mode = _make_mode(
            points=[
                SpeedPoint(time_s=0.0, speed_kmh=0.0),
                SpeedPoint(time_s=10.0, speed_kmh=100.0),
            ],
            total_duration=10.0,
        )
        profile = _make_profile(dynamics_params=DynamicsParams(pid_preview_s=1.0))
        ff = _make_ff(effort=0.0)
        pid = MagicMock(spec=PIDController)
        pid.update = MagicMock(return_value=0.0)

        dl = _make_loop(ff=ff, pid=pid, mode=mode, profile=profile)
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        # FF: now-frame elapsed(2.0) → ref = 20.0（前倒しなし）
        ff_args = ff.predict_effort.call_args[0]
        assert ff_args[0] == pytest.approx(20.0)
        # PID: elapsed(2.0) + pid_preview(1.0) = 3.0 → ref = 30.0（前倒し）
        pid_args = pid.update.call_args[0]
        assert pid_args[0] == pytest.approx(30.0)

    @pytest.mark.asyncio
    async def test_ff_future_and_past_use_now_frame(self) -> None:
        """FF の先読み/過去速度も now-frame（elapsed）基準で組まれる（pid_preview は無関係）。"""
        mode = _make_mode(
            points=[
                SpeedPoint(time_s=0.0, speed_kmh=0.0),
                SpeedPoint(time_s=10.0, speed_kmh=100.0),
            ],
            total_duration=10.0,
        )
        profile = _make_profile(dynamics_params=DynamicsParams(pid_preview_s=1.0))
        ff = _make_ff(effort=0.0)
        dl = _make_loop(ff=ff, mode=mode, profile=profile)
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        # predict_effort(ref, future_speeds, past_speeds) の future/past は elapsed=2.0 基準
        _, future_speeds, past_speeds = ff.predict_effort.call_args[0]
        expected_future = [min(100.0, (2.0 + h) * 10.0) for h in ff.horizons]
        expected_past = [max(0.0, (2.0 - h) * 10.0) for h in ff.past_horizons]
        assert future_speeds == pytest.approx(expected_future)
        assert past_speeds == pytest.approx(expected_past)

    @pytest.mark.asyncio
    async def test_kpi_and_last_ref_speed_use_now_frame(self) -> None:
        """KPI・current_ref_speed は前倒しされない now-frame の基準速度で評価される。"""
        mode = _make_mode(
            points=[
                SpeedPoint(time_s=0.0, speed_kmh=0.0),
                SpeedPoint(time_s=10.0, speed_kmh=100.0),
            ],
            total_duration=10.0,
        )
        profile = _make_profile(dynamics_params=DynamicsParams(pid_preview_s=1.0))
        dl = _make_loop(mode=mode, profile=profile, can_reader=_make_can_reader(speed=20.0))
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        # now-frame: t=2.0 → ref=20.0（前倒しされない）
        assert dl.current_ref_speed == pytest.approx(20.0)
        summary = dl.kpi_summary
        assert summary["n_samples"] == 1.0
        assert summary["max_abs_deviation_kmh"] == pytest.approx(0.0)

    @pytest.mark.asyncio
    async def test_zero_preview_ff_and_pid_both_now_frame(self) -> None:
        """pid_preview_s=0.0（デフォルト）では FF・PID とも now-frame 基準で一致する（回帰）。"""
        mode = _make_mode(
            points=[
                SpeedPoint(time_s=0.0, speed_kmh=0.0),
                SpeedPoint(time_s=10.0, speed_kmh=100.0),
            ],
            total_duration=10.0,
        )
        profile = _make_profile(dynamics_params=DynamicsParams(pid_preview_s=0.0))
        ff = _make_ff(effort=0.0)
        pid = MagicMock(spec=PIDController)
        pid.update = MagicMock(return_value=0.0)
        dl = _make_loop(ff=ff, pid=pid, mode=mode, profile=profile)
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        assert ff.predict_effort.call_args[0][0] == pytest.approx(20.0)
        assert pid.update.call_args[0][0] == pytest.approx(20.0)

    @pytest.mark.asyncio
    async def test_pid_preview_beyond_trajectory_end_clamps_safely(self) -> None:
        """PID 先読みが軌跡末尾を超えても終端値でクランプされ例外を起こさない。"""
        mode = _make_mode(
            points=[
                SpeedPoint(time_s=0.0, speed_kmh=0.0),
                SpeedPoint(time_s=5.0, speed_kmh=60.0),
            ],
            total_duration=10.0,
        )
        profile = _make_profile(dynamics_params=DynamicsParams(pid_preview_s=1.0))
        pid = MagicMock(spec=PIDController)
        pid.update = MagicMock(return_value=0.0)
        dl = _make_loop(pid=pid, mode=mode, profile=profile)
        dl._running = True

        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 4.5  # elapsed+preview = 5.5 > 5.0（軌跡末尾）
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()

        assert pid.update.call_args[0][0] == pytest.approx(60.0)  # 終端値でクランプ（例外なし）


# ---------------------------------------------------------------------------
# ゲインスケジューリング（速度依存プラントゲイン正規化）— Stage B
# ---------------------------------------------------------------------------


class TestGainScheduling:
    def _accel_mode(self) -> DrivingMode:
        # 0→100 km/h の上昇ランプ（elapsed=2.0 で加速フェーズ）
        return _make_mode(
            points=[SpeedPoint(0.0, 0.0), SpeedPoint(10.0, 100.0)],
            total_duration=10.0,
        )

    def _decel_mode(self) -> DrivingMode:
        # 100→0 km/h の下降ランプ（elapsed=2.0 で減速フェーズ）
        return _make_mode(
            points=[SpeedPoint(0.0, 100.0), SpeedPoint(10.0, 0.0)],
            total_duration=10.0,
        )

    def _pid_mock(self) -> MagicMock:
        pid = MagicMock(spec=PIDController)
        pid.update = MagicMock(return_value=0.0)
        return pid

    @pytest.mark.asyncio
    async def test_no_schedule_passes_scale_one(self) -> None:
        """gain_schedule が None なら gain_scale=1.0（従来動作）。"""
        profile = _make_profile(dynamics_params=DynamicsParams(pid_preview_s=0.0, fopdt_k=2.0))
        ff = _make_ff(effort=0.0)
        ff.gain_schedule = None
        pid = self._pid_mock()
        dl = _make_loop(ff=ff, pid=pid, mode=self._accel_mode(), profile=profile)
        dl._running = True
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()
        assert pid.update.call_args.kwargs["gain_scale"] == pytest.approx(1.0)

    @pytest.mark.asyncio
    @pytest.mark.parametrize(
        "dyn",
        [
            DynamicsParams(pid_preview_s=0.0, fopdt_k=None, fopdt_tau=1.0),  # k 未同定
            DynamicsParams(pid_preview_s=0.0, fopdt_k=2.0, fopdt_tau=None),  # tau 未同定
        ],
    )
    async def test_incomplete_fopdt_passes_scale_one(self, dyn: DynamicsParams) -> None:
        """g_nominal=tau/k は k・tau 両方が必要。片方欠落なら g_nominal=None → gain_scale=1.0。"""
        profile = _make_profile(dynamics_params=dyn)
        ff = _make_ff(effort=0.0)
        ff.gain_schedule = GainSchedule(
            speeds=(0.0, 120.0), accel_gains=(1.0, 1.0), brake_gains=(1.0, 1.0)
        )
        pid = self._pid_mock()
        dl = _make_loop(ff=ff, pid=pid, mode=self._accel_mode(), profile=profile)
        dl._running = True
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()
        assert pid.update.call_args.kwargs["gain_scale"] == pytest.approx(1.0)

    @pytest.mark.asyncio
    async def test_accel_phase_uses_accel_gain(self) -> None:
        """加速フェーズでは accel_gain を使う。scale=clamp(g/g_nominal)。"""
        # g_nominal = fopdt_tau/fopdt_k = 1.0/2.0 = 0.5、accel_gain=0.6 → scale = 1.2（範囲内）
        profile = _make_profile(
            dynamics_params=DynamicsParams(pid_preview_s=0.0, fopdt_k=2.0, fopdt_tau=1.0)
        )
        ff = _make_ff(effort=0.0)
        ff.gain_schedule = GainSchedule(
            speeds=(0.0, 120.0), accel_gains=(0.6, 0.6), brake_gains=(0.1, 0.1)
        )
        pid = self._pid_mock()
        dl = _make_loop(
            ff=ff, pid=pid, mode=self._accel_mode(), profile=profile,
            can_reader=_make_can_reader(speed=20.0),
        )
        dl._running = True
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()
        assert pid.update.call_args.kwargs["gain_scale"] == pytest.approx(1.2)

    @pytest.mark.asyncio
    async def test_accel_gain_clamps_to_upper_max(self) -> None:
        """ブースト上限 1.5（B-8-3）: g/g_nominal=2.0 でも scale は 1.5 にクランプされる。"""
        # g_nominal=0.5、accel_gain=1.0 → 比 2.0 → 上限 1.5 にクランプ
        profile = _make_profile(
            dynamics_params=DynamicsParams(pid_preview_s=0.0, fopdt_k=2.0, fopdt_tau=1.0)
        )
        ff = _make_ff(effort=0.0)
        ff.gain_schedule = GainSchedule(
            speeds=(0.0, 120.0), accel_gains=(1.0, 1.0), brake_gains=(0.1, 0.1)
        )
        pid = self._pid_mock()
        dl = _make_loop(
            ff=ff, pid=pid, mode=self._accel_mode(), profile=profile,
            can_reader=_make_can_reader(speed=20.0),
        )
        dl._running = True
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()
        assert pid.update.call_args.kwargs["gain_scale"] == pytest.approx(1.5)

    @pytest.mark.asyncio
    async def test_decel_phase_uses_brake_gain_and_clamps(self) -> None:
        """減速フェーズでは brake_gain を使い、下限 0.5 にクランプされる。"""
        # g_nominal=tau/k=0.5、brake_gain=0.1 → 0.1/0.5=0.2 → clamp 下限 0.5
        profile = _make_profile(
            dynamics_params=DynamicsParams(pid_preview_s=0.0, fopdt_k=2.0, fopdt_tau=1.0)
        )
        ff = _make_ff(effort=0.0)
        ff.gain_schedule = GainSchedule(
            speeds=(0.0, 120.0), accel_gains=(1.0, 1.0), brake_gains=(0.1, 0.1)
        )
        pid = self._pid_mock()
        dl = _make_loop(
            ff=ff, pid=pid, mode=self._decel_mode(), profile=profile,
            can_reader=_make_can_reader(speed=80.0),
        )
        dl._running = True
        with patch.object(asyncio, "get_running_loop") as mock_loop:
            loop_obj = MagicMock()
            loop_obj.time.return_value = 2.0
            mock_loop.return_value = loop_obj
            dl._started_at = 0.0
            await dl._execute_one_cycle()
        assert pid.update.call_args.kwargs["gain_scale"] == pytest.approx(0.5)


# ---------------------------------------------------------------------------
# ILC 補正 effort の合成（Stage C）
# ---------------------------------------------------------------------------


class TestILCEffortSynthesis:
    """FF+PID に加えて ILC 補正 effort が調停器へ渡ることを検証する。"""

    async def _run_one_cycle_capturing_effort(self, dl: DriveLoop, t: float = 0.0) -> float:
        """1 サイクル実行し、調停器 arbitrate に渡された合成 effort を返す。"""
        captured: dict[str, float] = {}
        real_arbitrate = dl._arbiter.arbitrate

        def spy(effort: float, dt: float):  # type: ignore[no-untyped-def]
            captured["effort"] = effort
            return real_arbitrate(effort, dt)

        with (
            patch.object(asyncio, "get_running_loop") as mock_loop,
            patch.object(dl._arbiter, "arbitrate", side_effect=spy),
        ):
            loop_obj = MagicMock()
            loop_obj.time.return_value = t
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0
            await dl._execute_one_cycle()
        return captured["effort"]

    @pytest.mark.asyncio
    async def test_ilc_effort_added_to_synthesis(self) -> None:
        """ILC 補正が FF+PID に加算されて調停器に渡る（ff=30, pid=0, ilc=+5 → 35）。"""
        from src.domain.control.ilc import ILCController, ILCTable

        ff = _make_ff(effort=30.0)
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0)
        ilc = ILCController(ILCTable(efforts=[5.0, 5.0], dt_s=10.0))  # 全域 +5%
        dl = _make_loop(ff=ff, pid=pid, ilc=ilc)
        effort = await self._run_one_cycle_capturing_effort(dl)
        assert effort == pytest.approx(35.0)

    @pytest.mark.asyncio
    async def test_none_ilc_is_full_regression(self) -> None:
        """ilc=None なら合成は従来どおり FF+PID のみ（ff=30, pid=0 → 30）。"""
        ff = _make_ff(effort=30.0)
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0)
        dl = _make_loop(ff=ff, pid=pid, ilc=None)
        effort = await self._run_one_cycle_capturing_effort(dl)
        assert effort == pytest.approx(30.0)

    @pytest.mark.asyncio
    async def test_ilc_effort_sampled_at_now_frame(self) -> None:
        """ILC は elapsed_s（now-frame）で参照される（t=5s で該当補正が入る）。"""
        from src.domain.control.ilc import ILCController, ILCTable

        ff = _make_ff(effort=0.0)
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0)
        # efforts: t=0→0, t=5→8, t=10→0（グリッド 5s）。t=5 では 8% が入る。
        ilc = ILCController(ILCTable(efforts=[0.0, 8.0, 0.0], dt_s=5.0))
        dl = _make_loop(ff=ff, pid=pid, ilc=ilc)
        effort = await self._run_one_cycle_capturing_effort(dl, t=5.0)
        assert effort == pytest.approx(8.0)


# ---------------------------------------------------------------------------
# プラン＋トリム経路（新アーキテクチャ）
# ---------------------------------------------------------------------------


def _uniform_plan(effort: float, phase: PlanPhase, n: int = 400, dt: float = 0.1) -> PedalPlan:
    """一様な effort/phase のプラン（テスト用）。n×dt 秒をカバーする。"""
    return PedalPlan(dt_s=dt, efforts=[effort] * n, phases=[phase] * n)


class TestPlanPathSynthesis:
    """プランありの経路で FF を毎サイクル評価せず plan+trim+ilc を合成することを検証する。"""

    async def _run_cycle_capturing(self, dl: DriveLoop, t: float = 2.0) -> float:
        captured: dict[str, float] = {}
        real = dl._arbiter.arbitrate

        def spy(effort: float, dt: float):  # type: ignore[no-untyped-def]
            captured["effort"] = effort
            return real(effort, dt)

        with (
            patch.object(asyncio, "get_running_loop") as mock_loop,
            patch.object(dl._arbiter, "arbitrate", side_effect=spy),
        ):
            loop_obj = MagicMock()
            loop_obj.time.return_value = t
            mock_loop.return_value = loop_obj
            dl._running = True
            dl._started_at = 0.0
            await dl._execute_one_cycle()
        return captured["effort"]

    @pytest.mark.asyncio
    async def test_plan_effort_used_and_ff_not_called(self) -> None:
        """プランありなら FF.predict_effort は呼ばれず、plan.effort_at が名目 effort になる。"""
        ff = _make_ff(effort=99.0)  # 呼ばれたら 99 が混じるはず
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0)
        # 偏差 0（actual=ref）でトリムは 0。plan effort=20 のみが渡る。
        mode = _make_mode(
            points=[SpeedPoint(0.0, 60.0), SpeedPoint(10.0, 60.0)], total_duration=10.0
        )
        plan = _uniform_plan(20.0, PlanPhase.DRIVE)
        dl = _make_loop(
            ff=ff, pid=pid, mode=mode, can_reader=_make_can_reader(speed=60.0), plan=plan
        )
        effort = await self._run_cycle_capturing(dl)
        assert effort == pytest.approx(20.0)
        ff.predict_effort.assert_not_called()

    @pytest.mark.asyncio
    async def test_ilc_added_in_plan_path(self) -> None:
        """プラン経路でも ILC 補正が加算される（plan=20, trim=0, ilc=+5 → 25）。"""
        from src.domain.control.ilc import ILCController, ILCTable

        ff = _make_ff(effort=0.0)
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0)
        mode = _make_mode(
            points=[SpeedPoint(0.0, 60.0), SpeedPoint(10.0, 60.0)], total_duration=10.0
        )
        plan = _uniform_plan(20.0, PlanPhase.DRIVE)
        ilc = ILCController(ILCTable(efforts=[5.0, 5.0], dt_s=10.0))
        dl = _make_loop(
            ff=ff, pid=pid, mode=mode, can_reader=_make_can_reader(speed=60.0), plan=plan, ilc=ilc
        )
        effort = await self._run_cycle_capturing(dl)
        assert effort == pytest.approx(25.0)


class TestPhaseAuthority:
    """_apply_phase_authority: フェーズ権限クランプと速い補正層の無権限化を検証する。"""

    def _loop(self, fast_active: bool = False) -> DriveLoop:
        dl = _make_loop(plan=_uniform_plan(0.0, PlanPhase.DRIVE))
        dl._trim._fast_active = fast_active
        return dl

    def test_drive_clamps_brake_direction(self) -> None:
        dl = self._loop()
        # DRIVE で合成が負（ブレーキ）→ 0 にクランプ、制動側飽和フラグ
        effort, ch, cl = dl._apply_phase_authority(1.0, -3.0, 0.0, PlanPhase.DRIVE)
        assert effort == 0.0
        assert cl is True and ch is False

    def test_drive_keeps_positive(self) -> None:
        dl = self._loop()
        effort, ch, cl = dl._apply_phase_authority(2.0, 1.0, 0.5, PlanPhase.DRIVE)
        assert effort == pytest.approx(3.5)
        assert ch is False and cl is False

    def test_brake_clamps_accel_direction(self) -> None:
        dl = self._loop()
        effort, ch, cl = dl._apply_phase_authority(-1.0, 3.0, 0.0, PlanPhase.BRAKE)
        assert effort == 0.0
        assert ch is True and cl is False

    def test_coast_zero(self) -> None:
        dl = self._loop()
        effort, ch, cl = dl._apply_phase_authority(5.0, 2.0, 1.0, PlanPhase.COAST)
        assert effort == 0.0
        assert ch is True and cl is True

    def test_stop_hold_uses_plan_only(self) -> None:
        dl = self._loop()
        # STOP_HOLD は base（プランの停車保持）のみ、トリム/ILC を無効化
        effort, ch, cl = dl._apply_phase_authority(-19.2, 5.0, 3.0, PlanPhase.STOP_HOLD)
        assert effort == pytest.approx(-19.2)
        assert ch is True and cl is True

    def test_fast_active_bypasses_authority(self) -> None:
        dl = self._loop(fast_active=True)
        # 速い補正層アクティブなら DRIVE でもブレーキ方向を通す（max≤1.0 安全網）
        effort, ch, cl = dl._apply_phase_authority(1.0, -3.0, 0.0, PlanPhase.DRIVE)
        assert effort == pytest.approx(-2.0)
        assert ch is False and cl is False
