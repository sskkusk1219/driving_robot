import asyncio
import dataclasses
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest

from src.app.robot_controller import (
    EmergencyStillActive,
    InvalidStateTransition,
    RobotController,
)
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pid import PIDController
from src.models.calibration import CalibrationData
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import PIDGains, StopConfig, VehicleProfile
from src.models.system_state import InitStepStatus, RobotState


def make_accel_driver() -> MagicMock:
    driver = MagicMock()
    driver.connect = AsyncMock()
    driver.enable_modbus_control = AsyncMock()
    driver.home_return = AsyncMock()
    driver.servo_off = AsyncMock()
    driver.servo_on = AsyncMock()
    driver.is_alarm_active = AsyncMock(return_value=False)
    driver.reset_alarm = AsyncMock()
    driver.read_position = AsyncMock(return_value=0)
    driver.read_current = AsyncMock(return_value=0.0)
    driver.move_to_position = AsyncMock()
    driver.wait_for_position_complete = AsyncMock()
    return driver


def make_brake_driver() -> MagicMock:
    return make_accel_driver()


def make_can_reader() -> MagicMock:
    reader = MagicMock()
    reader.connect = AsyncMock()
    reader.read_speed = AsyncMock(return_value=0.0)
    return reader


def make_safety_monitor() -> MagicMock:
    monitor = MagicMock()
    monitor.start_monitoring = AsyncMock()
    monitor.stop_monitoring = AsyncMock()
    monitor.register_emergency_callback = MagicMock()
    monitor.trigger_emergency = AsyncMock()
    monitor.is_emergency_active = MagicMock(return_value=False)
    return monitor


def make_pid() -> PIDController:
    gains = PIDGains(kp=1.0, ki=0.0, kd=0.0)
    return PIDController(kp=gains.kp, ki=gains.ki, kd=gains.kd)


def make_stop_config() -> StopConfig:
    return StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0)


def make_ff_controller() -> MagicMock:
    ff = MagicMock(spec=FeedforwardController)
    ff.has_model = False
    return ff


def make_safety_check() -> MagicMock:
    safety = MagicMock()
    safety.check_overcurrent = MagicMock(return_value=False)
    safety.check_deviation = MagicMock(return_value=False)
    safety.set_stop_config = MagicMock()
    return safety


def _calibrated_profile() -> VehicleProfile:
    calib = CalibrationData(
        accel_zero_pos=100,
        accel_full_pos=5100,
        accel_stroke=5000,
        brake_zero_pos=200,
        brake_full_pos=5200,
        brake_stroke=5000,
        calibrated_at=datetime.now(tz=UTC),
        is_valid=True,
    )
    return VehicleProfile(
        id="p1",
        name="t",
        max_accel_opening=80.0,
        max_brake_opening=80.0,
        max_speed=100.0,
        max_decel_g=0.4,
        pid_gains=PIDGains(kp=1.0, ki=0.1, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=calib,
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )


def _make_mode(mode_id: str = "mode-1") -> DrivingMode:
    """start_auto_drive の precondition（W4）を満たすための最小限の走行モード。"""
    points = [SpeedPoint(time_s=0.0, speed_kmh=0.0), SpeedPoint(time_s=10.0, speed_kmh=0.0)]
    return DrivingMode(
        id=mode_id,
        name="TestMode",
        description="",
        reference_speed=points,
        total_duration=10.0,
        max_speed=100.0,
        created_at=datetime.now(tz=UTC),
    )


def make_controller(last_normal_shutdown: bool = True) -> RobotController:
    return RobotController(
        accel_driver=make_accel_driver(),
        brake_driver=make_brake_driver(),
        can_reader=make_can_reader(),
        safety_monitor=make_safety_monitor(),
        pid=make_pid(),
        last_normal_shutdown=last_normal_shutdown,
        ff_controller=make_ff_controller(),
        safety_check=make_safety_check(),
    )


async def advance_to_ready(ctrl: RobotController) -> None:
    """BOOTING → STANDBY → INITIALIZING → READY まで進める。"""
    await ctrl.start()
    await ctrl.initialize()


class TestRobotControllerInit:
    def test_initial_state_is_booting(self) -> None:
        ctrl = make_controller()
        assert ctrl.get_system_state().robot_state == RobotState.BOOTING

    def test_initial_profile_and_session_are_none(self) -> None:
        ctrl = make_controller()
        state = ctrl.get_system_state()
        assert state.active_profile_id is None
        assert state.active_session_id is None


class TestGetSystemState:
    @pytest.mark.asyncio
    async def test_returns_current_state(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        state = ctrl.get_system_state()
        assert state.robot_state == RobotState.STANDBY

    def test_updated_at_is_recent(self) -> None:
        from datetime import datetime

        ctrl = make_controller()
        state = ctrl.get_system_state()
        now = datetime.now(tz=UTC)
        diff = abs((now - state.updated_at).total_seconds())
        assert diff < 1.0


class TestTransitionValidation:
    def test_invalid_transition_from_booting_raises(self) -> None:
        ctrl = make_controller()
        with pytest.raises(InvalidStateTransition):
            ctrl._transition(RobotState.READY)

    def test_invalid_transition_from_standby_raises(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.STANDBY
        with pytest.raises(InvalidStateTransition):
            ctrl._transition(RobotState.RUNNING)

    def test_valid_transition_succeeds(self) -> None:
        ctrl = make_controller()
        ctrl._transition(RobotState.STANDBY)
        assert ctrl._state == RobotState.STANDBY

    def test_invalid_transition_does_not_change_state(self) -> None:
        ctrl = make_controller()
        try:
            ctrl._transition(RobotState.RUNNING)
        except InvalidStateTransition:
            pass
        assert ctrl._state == RobotState.BOOTING


class TestStart:
    @pytest.mark.asyncio
    async def test_start_transitions_to_standby(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        assert ctrl.get_system_state().robot_state == RobotState.STANDBY

    @pytest.mark.asyncio
    async def test_start_from_non_booting_raises(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        with pytest.raises(InvalidStateTransition):
            await ctrl.start()

    @pytest.mark.asyncio
    async def test_start_transitions_to_error_on_hardware_failure(self) -> None:
        """start() 内部で例外発生時に ERROR 状態へ遷移し、例外を再送出すること。"""
        ctrl = make_controller()
        original_transition = ctrl._transition
        call_count = 0

        def raising_transition(new_state: RobotState) -> None:
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                raise RuntimeError("通信エラー（テスト用）")
            original_transition(new_state)

        ctrl._transition = raising_transition  # type: ignore[method-assign]
        with pytest.raises(RuntimeError):
            await ctrl.start()
        assert ctrl._state == RobotState.ERROR


class TestInitialize:
    @pytest.mark.asyncio
    async def test_initialize_transitions_to_ready(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        await ctrl.initialize()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_initialize_calls_home_return_when_not_normal_shutdown(self) -> None:
        ctrl = make_controller(last_normal_shutdown=False)
        await ctrl.start()
        await ctrl.initialize()
        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_initialize_skips_home_return_when_normal_shutdown(self) -> None:
        ctrl = make_controller(last_normal_shutdown=True)
        await ctrl.start()
        await ctrl.initialize()
        ctrl._accel_driver.home_return.assert_not_called()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_not_called()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_initialize_from_booting_raises(self) -> None:
        ctrl = make_controller()
        with pytest.raises(InvalidStateTransition):
            await ctrl.initialize()


class TestStop:
    @pytest.mark.asyncio
    async def test_stop_from_running_transitions_to_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        await ctrl.stop()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_stop_from_running_calls_home_return_and_servo_off(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        ctrl._accel_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        await ctrl.stop()
        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_called_once()  # type: ignore[attr-defined]
        ctrl._accel_driver.servo_off.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.servo_off.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_stop_sets_last_normal_shutdown_true(self) -> None:
        ctrl = make_controller(last_normal_shutdown=False)
        await ctrl.start()
        await ctrl.initialize()
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        await ctrl.stop()
        assert ctrl._last_normal_shutdown is True

    @pytest.mark.asyncio
    async def test_stop_clears_session_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        await ctrl.stop()
        assert ctrl.get_system_state().active_session_id is None

    @pytest.mark.asyncio
    async def test_stop_from_ready_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.stop()

    @pytest.mark.asyncio
    async def test_stop_from_manual_transitions_to_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_manual()
        await ctrl.stop()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_stop_does_not_transition_to_ready_until_home_return_completes(self) -> None:
        """W3 回帰テスト: home_return/servo_off が完了するまで READY へ遷移しない。

        先に READY へ遷移すると、home_return がまだ飛行中のうちに次の arm/走行が
        READY を見て受理され、後から完了する home_return がブレーキ保持等を
        上書きしてしまう。"""
        import asyncio

        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

        release_gate = asyncio.Event()

        async def blocking_home_return() -> None:
            await release_gate.wait()

        ctrl._accel_driver.home_return = AsyncMock(side_effect=blocking_home_return)
        ctrl._brake_driver.home_return = AsyncMock(side_effect=blocking_home_return)

        stop_task = asyncio.ensure_future(ctrl.stop())
        await asyncio.sleep(0)  # stop() が home_return 待ちに入るまで進める

        # home_return が完了していない間は READY へ遷移していない（= 次の arm を受理しない）
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

        release_gate.set()
        await stop_task
        assert ctrl.get_system_state().robot_state == RobotState.READY


class TestEmergencyStop:
    @pytest.mark.asyncio
    async def test_emergency_stop_from_running(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY

    @pytest.mark.asyncio
    async def test_emergency_stop_from_manual(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_manual()
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY

    @pytest.mark.asyncio
    async def test_emergency_stop_from_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY

    @pytest.mark.asyncio
    async def test_emergency_stop_from_standby(self) -> None:
        """プロファイル/モード画面や復帰直後(STANDBY)でも非常停止できること。"""
        ctrl = make_controller()
        await ctrl.start()  # BOOTING → STANDBY
        assert ctrl.get_system_state().robot_state == RobotState.STANDBY
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY

    @pytest.mark.asyncio
    async def test_emergency_stop_from_calibrating(self) -> None:
        """キャリブレーション画面(CALIBRATING)でも非常停止できること。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.jog_axis("accel", 10)  # READY → CALIBRATING
        assert ctrl.get_system_state().robot_state == RobotState.CALIBRATING
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY

    @pytest.mark.asyncio
    async def test_emergency_stop_from_pre_check(self) -> None:
        """走行前チェック(PRE_CHECK)中でも非常停止できること。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        ctrl._transition(RobotState.PRE_CHECK)  # 走行開始の過渡状態を再現
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY

    @pytest.mark.asyncio
    async def test_emergency_stop_calls_home_return_on_both_axes(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        ctrl._accel_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        await ctrl.emergency_stop()
        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_emergency_stop_does_not_call_trigger_emergency(self) -> None:
        """emergency_stop は trigger_emergency を呼び返さない（一方向ディスパッチ）。

        本番配線では SafetyMonitor.trigger_emergency のコールバックとして
        emergency_stop 自身が登録されるため、呼び返すと再帰ループになる。
        """
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        ctrl._safety_monitor.trigger_emergency.assert_not_called()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_emergency_stop_from_booting_raises(self) -> None:
        ctrl = make_controller()
        with pytest.raises(InvalidStateTransition):
            await ctrl.emergency_stop()

    @pytest.mark.asyncio
    async def test_emergency_stop_clears_session_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().active_session_id is None

    @pytest.mark.asyncio
    async def test_emergency_stop_is_reentrant_no_duplicate_home_return(self) -> None:
        """多重 GPIO 発火を模した連続呼び出しで原点復帰が重複起動しないこと。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        # 2 回目・3 回目は既に EMERGENCY のため早期 return する想定
        await ctrl.emergency_stop()
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY
        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_emergency_stop_closes_session_even_if_home_return_fails(self) -> None:
        """home_return が例外でもセッションは 'emergency' で必ず閉じること。

        status='running' が残らないこと。
        実機では Modbus 失敗で home_return が例外を投げ得る。try/finally で
        _close_session に到達しないと DB に status='running' のセッションが残る。
        """
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        ctrl.select_profile(MagicMock(id="prof-1", model_path=None))  # type: ignore[arg-type]
        log_writer = MagicMock()
        log_writer.start_session = AsyncMock(return_value="sess-1")
        log_writer.write_log = AsyncMock()
        log_writer.end_session = AsyncMock()
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
            log_writer=log_writer,
        )
        ctrl._accel_driver.home_return = AsyncMock(  # type: ignore[attr-defined]
            side_effect=RuntimeError("Modbus desync")
        )

        with pytest.raises(RuntimeError):
            await ctrl.emergency_stop()

        # home_return が失敗しても: 状態は EMERGENCY、セッションは閉じられ end_session 記録済み
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY
        assert ctrl.get_system_state().active_session_id is None
        log_writer.end_session.assert_awaited_once_with("sess-1", "emergency")

    @pytest.mark.asyncio
    async def test_stop_auto_drive_closes_session_even_if_home_return_fails(self) -> None:
        """正常停止でも home_return 失敗時にセッションを 'completed' で閉じること。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        ctrl.select_profile(MagicMock(id="prof-1", model_path=None))  # type: ignore[arg-type]
        log_writer = MagicMock()
        log_writer.start_session = AsyncMock(return_value="sess-2")
        log_writer.write_log = AsyncMock()
        log_writer.end_session = AsyncMock()
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
            log_writer=log_writer,
        )
        ctrl._accel_driver.home_return = AsyncMock(  # type: ignore[attr-defined]
            side_effect=RuntimeError("Modbus desync")
        )

        with pytest.raises(RuntimeError):
            await ctrl.stop_auto_drive()

        assert ctrl.get_system_state().active_session_id is None
        log_writer.end_session.assert_awaited_once_with("sess-2", "completed")
        ctrl._brake_driver.home_return.assert_called_once()  # type: ignore[attr-defined]


class TestResetEmergency:
    @pytest.mark.asyncio
    async def test_reset_emergency_transitions_to_standby(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        await ctrl.reset_emergency()
        assert ctrl.get_system_state().robot_state == RobotState.STANDBY

    @pytest.mark.asyncio
    async def test_reset_emergency_then_reinitialize_recovers_to_ready(self) -> None:
        """非常停止リセット後、初期化を再実行して READY へ復帰できること。"""
        ctrl = make_controller(last_normal_shutdown=False)
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        await ctrl.reset_emergency()
        await ctrl.initialize()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_reset_emergency_from_ready_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.reset_emergency()

    @pytest.mark.asyncio
    async def test_reset_emergency_blocked_while_switch_active(self) -> None:
        """物理スイッチが押下中はリセットを拒否し、EMERGENCY を維持する。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        ctrl._safety_monitor.is_emergency_active.return_value = True  # type: ignore[attr-defined]
        with pytest.raises(EmergencyStillActive):
            await ctrl.reset_emergency()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY

    @pytest.mark.asyncio
    async def test_reset_emergency_allowed_after_switch_released(self) -> None:
        """スイッチ解除後はリセットできて STANDBY へ戻る。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        ctrl._safety_monitor.is_emergency_active.return_value = True  # type: ignore[attr-defined]
        with pytest.raises(EmergencyStillActive):
            await ctrl.reset_emergency()
        # スイッチを戻す
        ctrl._safety_monitor.is_emergency_active.return_value = False  # type: ignore[attr-defined]
        await ctrl.reset_emergency()
        assert ctrl.get_system_state().robot_state == RobotState.STANDBY


class TestClearError:
    @pytest.mark.asyncio
    async def test_clear_error_transitions_to_standby(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.ERROR
        await ctrl.clear_error()
        assert ctrl.get_system_state().robot_state == RobotState.STANDBY

    @pytest.mark.asyncio
    async def test_clear_error_from_ready_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.clear_error()


class TestRunCalibration:
    @pytest.mark.asyncio
    async def test_run_calibration_returns_to_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.run_calibration()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_run_calibration_from_running_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        with pytest.raises(InvalidStateTransition):
            await ctrl.run_calibration()

    @pytest.mark.asyncio
    async def test_run_calibration_without_manager_returns_not_configured(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        result = await ctrl.run_calibration()
        assert result.success is False
        assert result.error_message == "キャリブレーション未設定"
        assert result.data is None

    @pytest.mark.asyncio
    async def test_run_calibration_delegates_to_manager(self) -> None:
        from src.models.calibration import CalibrationData, CalibrationResult

        ctrl = make_controller()
        await advance_to_ready(ctrl)

        calib_data = CalibrationData(
            accel_zero_pos=100,
            accel_full_pos=5100,
            accel_stroke=5000,
            brake_zero_pos=200,
            brake_full_pos=5200,
            brake_stroke=5000,
            calibrated_at=datetime.now(tz=UTC),
            is_valid=True,
        )
        expected_result = CalibrationResult(success=True, data=calib_data, error_message=None)
        mock_manager = AsyncMock()
        mock_manager.run_calibration = AsyncMock(return_value=expected_result)
        ctrl._calibration_manager = mock_manager  # type: ignore[assignment]

        result = await ctrl.run_calibration()

        mock_manager.run_calibration.assert_awaited_once()
        assert result.success is True
        assert result.data is calib_data

    @pytest.mark.asyncio
    async def test_run_calibration_state_returns_to_ready_after_manager_failure(self) -> None:
        from src.models.calibration import CalibrationResult

        ctrl = make_controller()
        await advance_to_ready(ctrl)

        failed_result = CalibrationResult(success=False, data=None, error_message="スパイク未検出")
        mock_manager = AsyncMock()
        mock_manager.run_calibration = AsyncMock(return_value=failed_result)
        ctrl._calibration_manager = mock_manager  # type: ignore[assignment]

        result = await ctrl.run_calibration()

        assert ctrl.get_system_state().robot_state == RobotState.READY
        assert result.success is False

    @pytest.mark.asyncio
    async def test_emergency_during_calibration_does_not_raise_and_cancels_manager(self) -> None:
        """W6 回帰テスト: キャリブレーション中の emergency_stop は
        CalibrationManager.cancel() を呼び、run_calibration の finally は
        EMERGENCY を READY で上書きしない（InvalidStateTransition を送出しない）。"""
        from src.models.calibration import CalibrationResult

        ctrl = make_controller()
        await advance_to_ready(ctrl)

        mock_manager = AsyncMock()
        cancel_calls: list[bool] = []
        mock_manager.cancel = MagicMock(side_effect=lambda: cancel_calls.append(True))

        async def fake_run_calibration(profile_id: str) -> CalibrationResult:
            # 探索の途中で emergency_stop が割り込んだことを模擬する
            await ctrl.emergency_stop()
            return CalibrationResult(success=False, data=None, error_message="中断されました")

        mock_manager.run_calibration = fake_run_calibration
        ctrl._calibration_manager = mock_manager  # type: ignore[assignment]

        result = await ctrl.run_calibration()  # 例外を送出しないこと

        assert result.success is False
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY  # 上書きされない
        assert cancel_calls == [True]


class TestSaveManualCalibration:
    async def _enter_calibrating(self, ctrl: RobotController) -> None:
        """READY から jog で CALIBRATING に入り、両軸の pending を設定する。"""
        await advance_to_ready(ctrl)
        await ctrl.jog_axis("accel", 100)  # READY → CALIBRATING
        ctrl._pending_calib_zero = {"accel": 100, "brake": 200}
        ctrl._pending_calib_full = {"accel": 5100, "brake": 5200}

    @pytest.mark.asyncio
    async def test_save_manual_calibration_homes_both_axes_after_save(self) -> None:
        from src.models.calibration import CalibrationData, CalibrationResult

        ctrl = make_controller()
        await self._enter_calibrating(ctrl)
        calib_data = CalibrationData(
            accel_zero_pos=100,
            accel_full_pos=5100,
            accel_stroke=5000,
            brake_zero_pos=200,
            brake_full_pos=5200,
            brake_stroke=5000,
            calibrated_at=datetime.now(tz=UTC),
            is_valid=True,
        )
        mock_manager = AsyncMock()
        mock_manager.save_manual = AsyncMock(
            return_value=CalibrationResult(success=True, data=calib_data, error_message=None)
        )
        ctrl._calibration_manager = mock_manager  # type: ignore[assignment]
        ctrl._accel_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.reset_mock()  # type: ignore[attr-defined]

        result = await ctrl.save_manual_calibration()

        mock_manager.save_manual.assert_awaited_once()
        ctrl._accel_driver.home_return.assert_awaited_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_awaited_once()  # type: ignore[attr-defined]
        assert result.success is True
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_save_manual_calibration_failure_stays_calibrating_without_home(self) -> None:
        from src.models.calibration import CalibrationResult

        ctrl = make_controller()
        await self._enter_calibrating(ctrl)
        mock_manager = AsyncMock()
        mock_manager.save_manual = AsyncMock(
            return_value=CalibrationResult(
                success=False, data=None, error_message="ストローク範囲外"
            )
        )
        ctrl._calibration_manager = mock_manager  # type: ignore[assignment]
        ctrl._accel_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.reset_mock()  # type: ignore[attr-defined]

        result = await ctrl.save_manual_calibration()

        # 失敗時は原点復帰せず CALIBRATING を維持し、pending を保持してリトライ可能にする
        ctrl._accel_driver.home_return.assert_not_called()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_not_called()  # type: ignore[attr-defined]
        assert result.success is False
        assert ctrl.get_system_state().robot_state == RobotState.CALIBRATING
        assert ctrl._pending_calib_zero == {"accel": 100, "brake": 200}
        assert ctrl._pending_calib_full == {"accel": 5100, "brake": 5200}

    @pytest.mark.asyncio
    async def test_save_manual_calibration_exception_stays_calibrating(self) -> None:
        ctrl = make_controller()
        await self._enter_calibrating(ctrl)
        mock_manager = AsyncMock()
        mock_manager.save_manual = AsyncMock(side_effect=RuntimeError("DBエラー"))
        ctrl._calibration_manager = mock_manager  # type: ignore[assignment]
        ctrl._accel_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.reset_mock()  # type: ignore[attr-defined]

        with pytest.raises(RuntimeError):
            await ctrl.save_manual_calibration()

        # 例外時も CALIBRATING を維持しリトライ可能（原点復帰しない）
        assert ctrl.get_system_state().robot_state == RobotState.CALIBRATING
        ctrl._accel_driver.home_return.assert_not_called()  # type: ignore[attr-defined]
        assert ctrl._pending_calib_zero == {"accel": 100, "brake": 200}

    @pytest.mark.asyncio
    async def test_save_manual_calibration_from_ready_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.save_manual_calibration()


class TestJogAxis:
    @pytest.mark.asyncio
    async def test_jog_axis_waits_for_position_complete_before_reading(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.jog_axis("accel", 100)
        # 移動完了待ちを挟んでから位置を読むこと
        ctrl._accel_driver.wait_for_position_complete.assert_awaited_once()  # type: ignore[attr-defined]
        ctrl._accel_driver.move_to_position.assert_awaited_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_jog_axis_invalid_axis_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(ValueError):
            await ctrl.jog_axis("steering", 100)


class TestStartAutoDrive:
    @pytest.mark.asyncio
    async def test_start_auto_drive_transitions_to_running(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

    @pytest.mark.asyncio
    async def test_start_auto_drive_returns_session_with_mode_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        session = await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        assert session.mode_id == "mode-1"
        assert session.run_type == "auto"
        assert session.status == "running"

    @pytest.mark.asyncio
    async def test_start_auto_drive_sets_active_session_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        session = await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        assert ctrl.get_system_state().active_session_id == session.id

    @pytest.mark.asyncio
    async def test_start_auto_drive_from_running_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        with pytest.raises(InvalidStateTransition):
            await ctrl.start_auto_drive(
                mode_id="mode-2",
                mode=_make_mode(),
                profile=_calibrated_profile(),
            )

    @pytest.mark.asyncio
    async def test_missing_profile_rejected_before_transitioning_to_running(self) -> None:
        """W4 回帰テスト: profile が None のまま開始しようとすると、RUNNING へ遷移
        する前に拒否される（ファントム RUNNING を防ぐ）。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.start_auto_drive(mode_id="mode-1", mode=_make_mode(), profile=None)
        assert ctrl.get_system_state().robot_state == RobotState.READY
        assert ctrl.get_system_state().active_session_id is None
        assert ctrl._drive_loop is None

    @pytest.mark.asyncio
    async def test_missing_mode_rejected_before_transitioning_to_running(self) -> None:
        """W4 回帰テスト: mode が None のまま開始しようとすると RUNNING へ遷移する前に
        拒否される。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.start_auto_drive(mode_id="mode-1", mode=None, profile=_calibrated_profile())
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_missing_ff_controller_rejected_before_transitioning_to_running(self) -> None:
        """W4 回帰テスト: ff_controller 未配線でも RUNNING へ遷移する前に拒否される。"""
        ctrl = make_controller()
        ctrl._ff_controller = None
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.start_auto_drive(
                mode_id="mode-1", mode=_make_mode(), profile=_calibrated_profile()
            )
        assert ctrl.get_system_state().robot_state == RobotState.READY


class TestStopAutoDrive:
    @pytest.mark.asyncio
    async def test_stop_auto_drive_transitions_to_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        await ctrl.stop_auto_drive()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_stop_auto_drive_clears_session(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        await ctrl.stop_auto_drive()
        assert ctrl.get_system_state().active_session_id is None

    @pytest.mark.asyncio
    async def test_stop_auto_drive_from_ready_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.stop_auto_drive()


class TestPauseResumeAutoDrive:
    @pytest.mark.asyncio
    async def test_pause_auto_drive_transitions_running_to_paused(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        ctrl._drive_loop = MagicMock()  # テストでは start で実ループは生成されない
        await ctrl.pause_auto_drive()
        assert ctrl.get_system_state().robot_state == RobotState.PAUSED
        ctrl._drive_loop.pause.assert_called_once()

    @pytest.mark.asyncio
    async def test_resume_auto_drive_transitions_paused_to_running(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        ctrl._drive_loop = MagicMock()
        await ctrl.pause_auto_drive()
        await ctrl.resume_auto_drive()
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING
        ctrl._drive_loop.resume.assert_called_once()

    @pytest.mark.asyncio
    async def test_pause_auto_drive_from_ready_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.pause_auto_drive()

    @pytest.mark.asyncio
    async def test_resume_auto_drive_from_running_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        ctrl._drive_loop = MagicMock()
        with pytest.raises(InvalidStateTransition):
            await ctrl.resume_auto_drive()

    @pytest.mark.asyncio
    async def test_stop_from_paused_transitions_to_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        ctrl._drive_loop = MagicMock()
        ctrl._drive_loop.stop_and_join = AsyncMock()
        ctrl._drive_loop.kpi_summary = {"n_samples": 0.0}
        await ctrl.pause_auto_drive()
        await ctrl.stop()
        assert ctrl.get_system_state().robot_state == RobotState.READY


class TestStartManual:
    @pytest.mark.asyncio
    async def test_start_manual_transitions_to_manual(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_manual()
        assert ctrl.get_system_state().robot_state == RobotState.MANUAL

    @pytest.mark.asyncio
    async def test_start_manual_returns_session(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        session = await ctrl.start_manual()
        assert session.mode_id is None
        assert session.run_type == "manual"
        assert session.status == "running"

    @pytest.mark.asyncio
    async def test_start_manual_sets_active_session_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        session = await ctrl.start_manual()
        assert ctrl.get_system_state().active_session_id == session.id


class TestStopManual:
    @pytest.mark.asyncio
    async def test_stop_manual_transitions_to_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_manual()
        await ctrl.stop_manual()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_stop_manual_clears_session(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_manual()
        await ctrl.stop_manual()
        assert ctrl.get_system_state().active_session_id is None

    @pytest.mark.asyncio
    async def test_stop_manual_from_running_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        with pytest.raises(InvalidStateTransition):
            await ctrl.stop_manual()


class TestStartConnectsHardware:
    @pytest.mark.asyncio
    async def test_start_calls_connect_on_accel_driver(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        ctrl._accel_driver.connect.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_start_calls_connect_on_brake_driver(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        ctrl._brake_driver.connect.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_start_calls_connect_on_can_reader(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        ctrl._can_reader.connect.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_start_calls_start_monitoring(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        ctrl._safety_monitor.start_monitoring.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_start_transitions_to_error_when_connect_fails(self) -> None:
        ctrl = make_controller()
        ctrl._accel_driver.connect = AsyncMock(side_effect=ConnectionError("接続失敗"))  # type: ignore[attr-defined]
        with pytest.raises(ConnectionError):
            await ctrl.start()
        assert ctrl.get_system_state().robot_state == RobotState.ERROR


class TestInitializeResetsAlarmAndServosOn:
    @pytest.mark.asyncio
    async def test_initialize_calls_enable_modbus_control_on_both_drivers(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        await ctrl.initialize()
        ctrl._accel_driver.enable_modbus_control.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.enable_modbus_control.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_initialize_calls_reset_alarm_on_both_drivers(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        await ctrl.initialize()
        ctrl._accel_driver.reset_alarm.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.reset_alarm.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_initialize_calls_servo_on_on_both_drivers(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        await ctrl.initialize()
        ctrl._accel_driver.servo_on.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.servo_on.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_initialize_calls_home_return_and_servo_on_together(self) -> None:
        ctrl = make_controller(last_normal_shutdown=False)
        await ctrl.start()
        await ctrl.initialize()
        ctrl._accel_driver.reset_alarm.assert_called_once()  # type: ignore[attr-defined]
        ctrl._accel_driver.servo_on.assert_called_once()  # type: ignore[attr-defined]
        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]


class TestInitProgress:
    """初期化進捗 (init_progress) がハード操作と連動することを検証。"""

    def test_init_progress_starts_all_pending(self) -> None:
        ctrl = make_controller()
        keys = [s.key for s in ctrl.init_progress]
        assert keys == [
            "comm_brake",
            "comm_accel",
            "comm_can",
            "alarm_reset",
            "servo_on",
            "home_return",
        ]
        assert all(s.status == InitStepStatus.PENDING for s in ctrl.init_progress)

    @pytest.mark.asyncio
    async def test_init_progress_all_done_after_initialize_with_home(self) -> None:
        ctrl = make_controller(last_normal_shutdown=False)
        await ctrl.start()
        await ctrl.initialize()
        status = {s.key: s.status for s in ctrl.init_progress}
        assert status["comm_brake"] == InitStepStatus.DONE
        assert status["comm_accel"] == InitStepStatus.DONE
        assert status["comm_can"] == InitStepStatus.DONE
        assert status["alarm_reset"] == InitStepStatus.DONE
        assert status["servo_on"] == InitStepStatus.DONE
        assert status["home_return"] == InitStepStatus.DONE

    @pytest.mark.asyncio
    async def test_init_progress_home_skipped_on_normal_shutdown(self) -> None:
        ctrl = make_controller(last_normal_shutdown=True)
        await ctrl.start()
        await ctrl.initialize()
        status = {s.key: s.status for s in ctrl.init_progress}
        assert status["home_return"] == InitStepStatus.SKIPPED

    @pytest.mark.asyncio
    async def test_init_progress_marks_failing_step_error(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        ctrl._can_reader.read_speed = AsyncMock(side_effect=RuntimeError("CAN no link"))  # type: ignore[attr-defined]
        with pytest.raises(RuntimeError):
            await ctrl.initialize()
        status = {s.key: s.status for s in ctrl.init_progress}
        # CAN 通信確認の手前 (ブレーキ・アクセル) は完了済み
        assert status["comm_brake"] == InitStepStatus.DONE
        assert status["comm_accel"] == InitStepStatus.DONE
        # 失敗したステップは ERROR、後続は PENDING のまま
        assert status["comm_can"] == InitStepStatus.ERROR
        assert status["alarm_reset"] == InitStepStatus.PENDING


class TestGetRealtimeData:
    @pytest.mark.asyncio
    async def test_returns_snapshot_with_correct_values(self) -> None:
        from src.models.system_state import RealtimeSnapshot

        ctrl = make_controller()
        ctrl._can_reader.read_speed = AsyncMock(return_value=60.0)  # type: ignore[attr-defined]
        ctrl._accel_driver.read_position = AsyncMock(return_value=100)  # type: ignore[attr-defined]
        ctrl._brake_driver.read_position = AsyncMock(return_value=200)  # type: ignore[attr-defined]
        ctrl._accel_driver.read_current = AsyncMock(return_value=1500.0)  # type: ignore[attr-defined]
        ctrl._brake_driver.read_current = AsyncMock(return_value=2000.0)  # type: ignore[attr-defined]

        snapshot = await ctrl.get_realtime_data()

        assert isinstance(snapshot, RealtimeSnapshot)
        assert snapshot.actual_speed_kmh == 60.0
        assert snapshot.accel_pos == 100
        assert snapshot.brake_pos == 200
        assert snapshot.accel_current_ma == 1500.0
        assert snapshot.brake_current_ma == 2000.0

    @pytest.mark.asyncio
    async def test_raises_when_can_reader_fails(self) -> None:
        ctrl = make_controller()
        ctrl._can_reader.read_speed = AsyncMock(side_effect=OSError("CAN通信エラー"))  # type: ignore[attr-defined]

        with pytest.raises(OSError):
            await ctrl.get_realtime_data()

    @pytest.mark.asyncio
    async def test_raises_when_accel_driver_fails(self) -> None:
        ctrl = make_controller()
        ctrl._accel_driver.read_current = AsyncMock(side_effect=RuntimeError("Modbus タイムアウト"))  # type: ignore[attr-defined]

        with pytest.raises(RuntimeError):
            await ctrl.get_realtime_data()

    @pytest.mark.asyncio
    async def test_idle_reads_are_cached_within_ttl(self) -> None:
        """E1 回帰テスト: アイドル時（制御ループ非動作中）は IDLE_SNAPSHOT_TTL_S 以内の
        再呼び出しでハードウェアを再読み取りしない（WS 10Hz によるジョグ/キャリブレーション
        操作の遅延を防ぐ）。"""
        ctrl = make_controller()
        ctrl._can_reader.read_speed = AsyncMock(return_value=60.0)  # type: ignore[attr-defined]
        ctrl._accel_driver.read_position = AsyncMock(return_value=100)  # type: ignore[attr-defined]
        ctrl._brake_driver.read_position = AsyncMock(return_value=200)  # type: ignore[attr-defined]
        ctrl._accel_driver.read_current = AsyncMock(return_value=1500.0)  # type: ignore[attr-defined]
        ctrl._brake_driver.read_current = AsyncMock(return_value=2000.0)  # type: ignore[attr-defined]

        first = await ctrl.get_realtime_data()
        second = await ctrl.get_realtime_data()

        assert second is first
        ctrl._can_reader.read_speed.assert_awaited_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_idle_cache_refreshes_after_ttl_expires(self) -> None:
        from src.app.robot_controller import IDLE_SNAPSHOT_TTL_S

        ctrl = make_controller()
        ctrl._can_reader.read_speed = AsyncMock(return_value=60.0)  # type: ignore[attr-defined]
        ctrl._accel_driver.read_position = AsyncMock(return_value=100)  # type: ignore[attr-defined]
        ctrl._brake_driver.read_position = AsyncMock(return_value=200)  # type: ignore[attr-defined]
        ctrl._accel_driver.read_current = AsyncMock(return_value=1500.0)  # type: ignore[attr-defined]
        ctrl._brake_driver.read_current = AsyncMock(return_value=2000.0)  # type: ignore[attr-defined]

        await ctrl.get_realtime_data()
        assert ctrl._idle_snapshot is not None
        # キャッシュを TTL 超過扱いに書き換える
        ctrl._idle_snapshot = dataclasses.replace(
            ctrl._idle_snapshot,
            captured_at=ctrl._idle_snapshot.captured_at - (IDLE_SNAPSHOT_TTL_S + 0.1),
        )

        await ctrl.get_realtime_data()

        assert ctrl._can_reader.read_speed.await_count == 2  # type: ignore[attr-defined]


class TestPreCheckIntegration:
    """走行前チェックと RobotController の統合テスト。"""

    def make_passing_pre_check_runner(self) -> AsyncMock:
        from src.models.pre_check import PreCheckItemResult, PreCheckResult

        runner = AsyncMock()
        runner.run = AsyncMock(
            return_value=PreCheckResult(
                passed=True,
                items=[PreCheckItemResult(item_name="通信確認", passed=True)],
            )
        )
        return runner

    def make_failing_pre_check_runner(self) -> AsyncMock:
        from src.models.pre_check import PreCheckItemResult, PreCheckResult

        runner = AsyncMock()
        runner.run = AsyncMock(
            return_value=PreCheckResult(
                passed=False,
                items=[
                    PreCheckItemResult(
                        item_name="UPS残量",
                        passed=False,
                        error_message="UPS残量不足: 5.0%（20%以上必要）",
                    )
                ],
            )
        )
        return runner

    @pytest.mark.asyncio
    async def test_start_auto_drive_without_pre_check_runner_transitions_to_running(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

    @pytest.mark.asyncio
    async def test_start_auto_drive_with_passing_pre_check_transitions_to_running(self) -> None:
        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_passing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

    @pytest.mark.asyncio
    async def test_start_auto_drive_with_failing_pre_check_raises_pre_check_failed(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415

        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_failing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        with pytest.raises(PreCheckFailed) as exc_info:
            await ctrl.start_auto_drive(
                mode_id="mode-1",
                mode=_make_mode(),
                profile=_calibrated_profile(),
            )
        assert exc_info.value.result is not None
        assert not exc_info.value.result.passed

    @pytest.mark.asyncio
    async def test_start_auto_drive_with_failing_pre_check_returns_to_ready(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415

        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_failing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        with pytest.raises(PreCheckFailed):
            await ctrl.start_auto_drive(
                mode_id="mode-1",
                mode=_make_mode(),
                profile=_calibrated_profile(),
            )
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_start_manual_with_passing_pre_check_transitions_to_manual(self) -> None:
        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_passing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        await ctrl.start_manual()
        assert ctrl.get_system_state().robot_state == RobotState.MANUAL

    @pytest.mark.asyncio
    async def test_start_manual_with_failing_pre_check_raises_pre_check_failed(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415

        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_failing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        with pytest.raises(PreCheckFailed):
            await ctrl.start_manual()

    @pytest.mark.asyncio
    async def test_start_manual_with_failing_pre_check_returns_to_ready(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415

        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_failing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        with pytest.raises(PreCheckFailed):
            await ctrl.start_manual()
        assert ctrl.get_system_state().robot_state == RobotState.READY


class TestShutdown:
    @pytest.mark.asyncio
    async def test_shutdown_from_standby_calls_stop_monitoring(self) -> None:
        ctrl = make_controller()
        await ctrl.start()
        await ctrl.shutdown()
        ctrl._safety_monitor.stop_monitoring.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_shutdown_from_ready_calls_stop_monitoring(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.shutdown()
        ctrl._safety_monitor.stop_monitoring.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_shutdown_from_booting_calls_stop_monitoring(self) -> None:
        ctrl = make_controller()
        assert ctrl.get_system_state().robot_state == RobotState.BOOTING
        await ctrl.shutdown()
        ctrl._safety_monitor.stop_monitoring.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_shutdown_stops_drive_loop_if_running(self) -> None:
        from unittest.mock import MagicMock

        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )

        mock_loop = MagicMock()
        mock_loop.stop_and_join = AsyncMock()
        ctrl._drive_loop = mock_loop  # type: ignore[assignment]

        await ctrl.shutdown()

        # 飛行中サイクルの完了を待ってからペダル解放する（stop() 単発ではない）
        mock_loop.stop_and_join.assert_awaited_once()
        assert ctrl._drive_loop is None

    @pytest.mark.asyncio
    async def test_shutdown_without_drive_loop_does_not_raise(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        assert ctrl._drive_loop is None
        await ctrl.shutdown()

    @pytest.mark.asyncio
    async def test_shutdown_cancels_active_learning_task(self) -> None:
        import asyncio  # noqa: PLC0415

        ctrl = make_controller()
        mock_task = MagicMock(spec=asyncio.Task)
        ctrl._active_learning_task = mock_task

        await ctrl.shutdown()

        mock_task.cancel.assert_called_once()
        assert ctrl._active_learning_task is None


class TestSelectProfile:
    def _make_profile(self) -> object:
        from datetime import datetime  # noqa: PLC0415

        from src.models.profile import PIDGains, StopConfig, VehicleProfile  # noqa: PLC0415

        return VehicleProfile(
            id="abc-123",
            name="TestCar",
            max_accel_opening=80.0,
            max_brake_opening=60.0,
            max_speed=180.0,
            max_decel_g=0.5,
            pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
            stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
            calibration=None,
            model_path=None,
            created_at=datetime.now(tz=UTC),
            updated_at=datetime.now(tz=UTC),
        )

    @pytest.mark.asyncio
    async def test_select_profile_sets_active_profile(self) -> None:
        ctrl = make_controller()
        await ctrl.start()  # BOOTING → STANDBY
        profile = self._make_profile()
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        assert ctrl.get_active_profile() is profile

    @pytest.mark.asyncio
    async def test_select_profile_updates_active_profile_id_in_system_state(self) -> None:
        ctrl = make_controller()
        await ctrl.start()  # BOOTING → STANDBY
        profile = self._make_profile()
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        assert ctrl.get_system_state().active_profile_id == "abc-123"

    def test_get_active_profile_returns_none_initially(self) -> None:
        ctrl = make_controller()
        assert ctrl.get_active_profile() is None

    def test_select_profile_raises_in_booting_state(self) -> None:
        from src.app.robot_controller import InvalidStateTransition  # noqa: PLC0415

        ctrl = make_controller()
        profile = self._make_profile()
        with pytest.raises(InvalidStateTransition):
            ctrl.select_profile(profile)  # type: ignore[arg-type]

    def _make_controller_with_ff(self, ff: object) -> RobotController:
        return RobotController(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            safety_monitor=make_safety_monitor(),
            pid=make_pid(),
            last_normal_shutdown=True,
            ff_controller=ff,  # type: ignore[arg-type]
        )

    @pytest.mark.asyncio
    async def test_select_profile_loads_model_when_path_present(self) -> None:
        from src.domain.control.feedforward import FeedforwardController  # noqa: PLC0415

        ff = MagicMock(spec=FeedforwardController)
        ctrl = self._make_controller_with_ff(ff)
        await ctrl.start()
        profile = self._make_profile()
        profile.model_path = "data/models/x.pkl"  # type: ignore[attr-defined]
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        ff.load_model.assert_called_once_with("data/models/x.pkl")

    @pytest.mark.asyncio
    async def test_select_profile_sets_feedforward_params(self) -> None:
        from src.domain.control.feedforward import FeedforwardController  # noqa: PLC0415

        ff = MagicMock(spec=FeedforwardController)
        ctrl = self._make_controller_with_ff(ff)
        await ctrl.start()
        profile = self._make_profile()  # model_path=None でも params は適用される
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        ff.set_params.assert_called_once_with(profile.feedforward_params)  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_select_profile_without_model_path_skips_load(self) -> None:
        from src.domain.control.feedforward import FeedforwardController  # noqa: PLC0415

        ff = MagicMock(spec=FeedforwardController)
        ctrl = self._make_controller_with_ff(ff)
        await ctrl.start()
        profile = self._make_profile()  # model_path=None
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        ff.load_model.assert_not_called()

    @pytest.mark.asyncio
    async def test_select_profile_load_failure_does_not_raise(self) -> None:
        from src.domain.control.feedforward import FeedforwardController  # noqa: PLC0415

        ff = MagicMock(spec=FeedforwardController)
        ff.load_model.side_effect = FileNotFoundError("missing")
        ctrl = self._make_controller_with_ff(ff)
        await ctrl.start()
        profile = self._make_profile()
        profile.model_path = "data/models/missing.pkl"  # type: ignore[attr-defined]
        # ロード失敗でも例外を送出せず、プロファイル選択自体は成功する
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        assert ctrl.get_active_profile() is profile

    @pytest.mark.asyncio
    async def test_select_profile_can_be_overwritten(self) -> None:
        from datetime import datetime  # noqa: PLC0415

        from src.models.profile import PIDGains, StopConfig, VehicleProfile  # noqa: PLC0415

        ctrl = make_controller()
        await ctrl.start()  # BOOTING → STANDBY
        p1 = VehicleProfile(
            id="p1",
            name="Car1",
            max_accel_opening=80.0,
            max_brake_opening=60.0,
            max_speed=180.0,
            max_decel_g=0.5,
            pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
            stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
            calibration=None,
            model_path=None,
            created_at=datetime.now(tz=UTC),
            updated_at=datetime.now(tz=UTC),
        )
        p2 = VehicleProfile(
            id="p2",
            name="Car2",
            max_accel_opening=70.0,
            max_brake_opening=50.0,
            max_speed=150.0,
            max_decel_g=0.4,
            pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
            stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
            calibration=None,
            model_path=None,
            created_at=datetime.now(tz=UTC),
            updated_at=datetime.now(tz=UTC),
        )
        ctrl.select_profile(p1)
        ctrl.select_profile(p2)
        assert ctrl.get_active_profile() is p2


async def _arm_ready_learning_controller() -> tuple[RobotController, object]:
    """学習運転を arm 済み（PRE_CHECK）の状態まで進めたコントローラとプロファイルを返す。"""
    ctrl = _make_controller_for_logging()
    await advance_to_ready(ctrl)
    profile = _make_profile_with_calibration()
    ctrl.select_profile(profile)  # type: ignore[arg-type]
    return ctrl, profile


class TestArmLearningDrive:
    """arm_learning_drive() の状態遷移とブレーキ保持。"""

    @pytest.mark.asyncio
    async def test_arm_transitions_to_pre_check(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_learning_drive()
        assert ctrl.get_system_state().robot_state == RobotState.PRE_CHECK

    @pytest.mark.asyncio
    async def test_arm_applies_stop_brake_hold(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_learning_drive()
        # 停車保持ブレーキ（既定 20%、brake stroke 0..5000 → 1000）まで踏み込む
        ctrl._brake_driver.move_to_position.assert_awaited()  # type: ignore[attr-defined]
        assert ctrl._brake_driver.move_to_position.await_args.args[0] == 1000  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_arm_two_phase_precheck_brakes_between(self) -> None:
        """踏込前チェック(車速除外)→ブレーキ踏込→踏込後チェック(位置除外)の順であること。"""
        from src.domain.pre_check import (  # noqa: PLC0415
            ITEM_ACTUATOR_POSITION,
            ITEM_VEHICLE_STOPPED,
        )
        from src.models.pre_check import PreCheckItemResult, PreCheckResult  # noqa: PLC0415

        ctrl, _ = await _arm_ready_learning_controller()
        order: list[str] = []
        excludes: list[frozenset[str]] = []

        orig_move = ctrl._brake_driver.move_to_position

        async def rec_move(pos: int) -> None:
            order.append("brake")
            await orig_move(pos)

        ctrl._brake_driver.move_to_position = AsyncMock(side_effect=rec_move)  # type: ignore[method-assign]

        async def rec_run(exclude: frozenset[str] = frozenset()) -> PreCheckResult:
            order.append("precheck")
            excludes.append(exclude)
            return PreCheckResult(
                passed=True, items=[PreCheckItemResult(item_name="x", passed=True)]
            )

        runner = MagicMock()
        runner.run = AsyncMock(side_effect=rec_run)
        ctrl._pre_check_runner = runner  # type: ignore[assignment]

        await ctrl.arm_learning_drive()
        assert order == ["precheck", "brake", "precheck"]
        assert ITEM_VEHICLE_STOPPED in excludes[0]  # 踏込前は車速確認を除外
        assert ITEM_ACTUATOR_POSITION in excludes[1]  # 踏込後はアクチュエータ位置を除外
        assert ctrl.get_system_state().robot_state == RobotState.PRE_CHECK

    @pytest.mark.asyncio
    async def test_arm_pre_check_failed_returns_to_ready(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415
        from src.models.pre_check import PreCheckItemResult, PreCheckResult  # noqa: PLC0415

        ctrl, _ = await _arm_ready_learning_controller()
        runner = MagicMock()
        runner.run = AsyncMock(
            return_value=PreCheckResult(
                passed=False,
                items=[
                    PreCheckItemResult(
                        item_name="車速確認", passed=False, error_message="車速が 0 ではありません"
                    )
                ],
            )
        )
        ctrl._pre_check_runner = runner  # type: ignore[assignment]
        with pytest.raises(PreCheckFailed):
            await ctrl.arm_learning_drive()
        assert ctrl.get_system_state().robot_state == RobotState.READY


class TestStartLearningDrive:
    """start_learning_drive() の状態遷移・返り値テスト（arm 後）。"""

    @pytest.mark.asyncio
    async def test_transitions_to_running(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_learning_drive()
        await ctrl.start_learning_drive()
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

    @pytest.mark.asyncio
    async def test_returns_learning_drive_session(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_learning_drive()
        session = await ctrl.start_learning_drive()
        assert session.run_type == "learning"
        assert session.mode_id is None
        assert session.status == "running"

    @pytest.mark.asyncio
    async def test_active_session_id_is_set(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_learning_drive()
        session = await ctrl.start_learning_drive()
        assert ctrl.get_system_state().active_session_id == session.id

    @pytest.mark.asyncio
    async def test_start_without_arm_raises(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        # arm せず READY のまま start → InvalidStateTransition
        with pytest.raises(InvalidStateTransition):
            await ctrl.start_learning_drive()
        assert ctrl.get_system_state().robot_state == RobotState.READY


class TestCancelLearningDrive:
    """cancel_learning_drive() の状態遷移とブレーキリリース。"""

    @pytest.mark.asyncio
    async def test_cancel_returns_to_ready_and_releases_brake(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_learning_drive()
        await ctrl.cancel_learning_drive()
        assert ctrl.get_system_state().robot_state == RobotState.READY
        ctrl._brake_driver.home_return.assert_awaited()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_cancel_without_arm_raises(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        with pytest.raises(InvalidStateTransition):
            await ctrl.cancel_learning_drive()


class TestStopLearningDrive:
    """stop_learning_drive() の緩減速・停止確認・停車保持。"""

    @pytest.mark.asyncio
    async def test_already_stopped_applies_brake_hold_and_ready(self) -> None:
        """学習終了時に既に停止していれば即停車保持ブレーキを適用し READY へ。"""
        ctrl, profile = await _arm_ready_learning_controller()
        ctrl._active_profile = profile  # type: ignore[assignment]
        ctrl._state = RobotState.RUNNING
        ctrl._can_reader.read_speed = AsyncMock(return_value=0.0)  # type: ignore[attr-defined]

        await ctrl.stop_learning_drive()

        assert ctrl.get_system_state().robot_state == RobotState.READY
        # 停車保持ブレーキ（20% → 1000）を適用。原点復帰はしない（停止保持を維持）。
        assert ctrl._brake_driver.move_to_position.await_args.args[0] == 1000  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_not_called()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_decelerates_until_stopped_then_holds(self) -> None:
        """転動中なら停止が確認できるまで減速し、その後に停車保持ブレーキを適用する。"""
        ctrl, profile = await _arm_ready_learning_controller()
        ctrl._active_profile = profile  # type: ignore[assignment]
        ctrl._state = RobotState.RUNNING
        # 10 → 5 → 0 km/h と収束。最後の 0 で停止確認しループを抜ける。
        ctrl._can_reader.read_speed = AsyncMock(side_effect=[10.0, 5.0, 0.0])  # type: ignore[attr-defined]

        await ctrl.stop_learning_drive()

        assert ctrl.get_system_state().robot_state == RobotState.READY
        # 減速中に複数回ブレーキ位置を指令し、最後は停車保持ブレーキ（1000）。
        positions = [c.args[0] for c in ctrl._brake_driver.move_to_position.await_args_list]  # type: ignore[attr-defined]
        assert len(positions) >= 2
        assert positions[-1] == 1000
        ctrl._brake_driver.home_return.assert_not_called()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_bridge_deceleration_exposes_brake_opening_for_ui(self) -> None:
        """橋渡し減速中は current_openings が指令ブレーキ開度を返す（UI 表示用）。

        減速はループ停止後に走るため、以前は current_openings が (0,0) を返し
        減速中のブレーキ開度が UI に出なかった（回帰防止）。
        """
        ctrl, profile = await _arm_ready_learning_controller()
        ctrl._active_profile = profile  # type: ignore[assignment]
        ctrl._state = RobotState.RUNNING
        ctrl._can_reader.read_speed = AsyncMock(side_effect=[10.0, 5.0, 0.0])  # type: ignore[attr-defined]

        # 各ブレーキ指令時点の current_openings を記録する
        openings_seen: list[tuple[float, float]] = []
        orig_move = ctrl._brake_driver.move_to_position

        async def rec_move(pos: int) -> None:
            await orig_move(pos)
            openings_seen.append(ctrl.current_openings)

        ctrl._brake_driver.move_to_position = AsyncMock(side_effect=rec_move)  # type: ignore[method-assign]

        await ctrl.stop_learning_drive()

        # 減速中の少なくとも1点でブレーキ開度が UI に見えている（accel は常に 0）
        assert any(brake > 0.0 for _, brake in openings_seen)
        assert all(accel == 0.0 for accel, _ in openings_seen)
        # READY 遷移後は橋渡し表示を終了し (0,0) へ戻す
        assert ctrl.current_openings == (0.0, 0.0)


class TestArmAutoDrive:
    """arm_auto_drive() の状態遷移とブレーキ保持（学習運転 arm と共通実装）。"""

    @pytest.mark.asyncio
    async def test_arm_transitions_to_pre_check(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_auto_drive()
        assert ctrl.get_system_state().robot_state == RobotState.PRE_CHECK

    @pytest.mark.asyncio
    async def test_arm_applies_stop_brake_hold(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_auto_drive()
        # 停車保持ブレーキ（既定 20%、brake stroke 0..5000 → 1000）まで踏み込む
        ctrl._brake_driver.move_to_position.assert_awaited()  # type: ignore[attr-defined]
        assert ctrl._brake_driver.move_to_position.await_args.args[0] == 1000  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_arm_two_phase_precheck_brakes_between(self) -> None:
        """踏込前チェック(車速除外)→ブレーキ踏込→踏込後チェック(位置除外)の順であること。"""
        from src.domain.pre_check import (  # noqa: PLC0415
            ITEM_ACTUATOR_POSITION,
            ITEM_VEHICLE_STOPPED,
        )
        from src.models.pre_check import PreCheckItemResult, PreCheckResult  # noqa: PLC0415

        ctrl, _ = await _arm_ready_learning_controller()
        order: list[str] = []
        excludes: list[frozenset[str]] = []

        orig_move = ctrl._brake_driver.move_to_position

        async def rec_move(pos: int) -> None:
            order.append("brake")
            await orig_move(pos)

        ctrl._brake_driver.move_to_position = AsyncMock(side_effect=rec_move)  # type: ignore[method-assign]

        async def rec_run(exclude: frozenset[str] = frozenset()) -> PreCheckResult:
            order.append("precheck")
            excludes.append(exclude)
            return PreCheckResult(
                passed=True, items=[PreCheckItemResult(item_name="x", passed=True)]
            )

        runner = MagicMock()
        runner.run = AsyncMock(side_effect=rec_run)
        ctrl._pre_check_runner = runner  # type: ignore[assignment]

        await ctrl.arm_auto_drive()
        assert order == ["precheck", "brake", "precheck"]
        assert ITEM_VEHICLE_STOPPED in excludes[0]  # 踏込前は車速確認を除外
        assert ITEM_ACTUATOR_POSITION in excludes[1]  # 踏込後はアクチュエータ位置を除外
        assert ctrl.get_system_state().robot_state == RobotState.PRE_CHECK

    @pytest.mark.asyncio
    async def test_arm_pre_check_failed_returns_to_ready(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415
        from src.models.pre_check import PreCheckItemResult, PreCheckResult  # noqa: PLC0415

        ctrl, _ = await _arm_ready_learning_controller()
        runner = MagicMock()
        runner.run = AsyncMock(
            return_value=PreCheckResult(
                passed=False,
                items=[
                    PreCheckItemResult(
                        item_name="車速確認", passed=False, error_message="車速が 0 ではありません"
                    )
                ],
            )
        )
        ctrl._pre_check_runner = runner  # type: ignore[assignment]
        with pytest.raises(PreCheckFailed):
            await ctrl.arm_auto_drive()
        assert ctrl.get_system_state().robot_state == RobotState.READY


class TestStartAutoDriveAfterArm:
    """start_auto_drive() を arm 後（PRE_CHECK）から呼ぶ確認ポップアップ「はい」経路。"""

    @pytest.mark.asyncio
    async def test_start_after_arm_transitions_to_running(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_auto_drive()
        session = await ctrl.start_auto_drive(
            mode_id="mode-1",
            mode=_make_mode(),
            profile=_calibrated_profile(),
        )
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING
        assert session.run_type == "auto"
        assert ctrl.get_system_state().active_session_id == session.id


class TestCancelAutoDrive:
    """cancel_auto_drive() の状態遷移とブレーキリリース（学習運転 cancel と共通実装）。"""

    @pytest.mark.asyncio
    async def test_cancel_returns_to_ready_and_releases_brake(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        await ctrl.arm_auto_drive()
        await ctrl.cancel_auto_drive()
        assert ctrl.get_system_state().robot_state == RobotState.READY
        ctrl._brake_driver.home_return.assert_awaited()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_cancel_without_arm_raises(self) -> None:
        ctrl, _ = await _arm_ready_learning_controller()
        with pytest.raises(InvalidStateTransition):
            await ctrl.cancel_auto_drive()


def _make_profile_with_calibration() -> object:
    from src.models.calibration import CalibrationData  # noqa: PLC0415
    from src.models.profile import PIDGains, StopConfig, VehicleProfile  # noqa: PLC0415

    return VehicleProfile(
        id="prof-uuid-1",
        name="LogCar",
        max_accel_opening=80.0,
        max_brake_opening=60.0,
        max_speed=120.0,
        max_decel_g=0.5,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=CalibrationData(
            accel_zero_pos=0,
            accel_full_pos=5000,
            accel_stroke=5000,
            brake_zero_pos=0,
            brake_full_pos=5000,
            brake_stroke=5000,
            calibrated_at=datetime.now(tz=UTC),
            is_valid=True,
        ),
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )


def _make_patterns() -> object:
    from src.models.learning_drive import LearningPattern, PatternKind  # noqa: PLC0415

    return [
        LearningPattern(
            kind=PatternKind.ACCEL_SWEEP,
            accel_opening=30.0,
            brake_opening=30.0,
            hold_duration_s=1.0,
        )
    ]


def _make_controller_for_logging() -> RobotController:
    """ff_controller / safety_check / learning_manager を備えた、DriveLoop を起動できる構成。"""
    from src.domain.control.feedforward import FeedforwardController  # noqa: PLC0415

    accel = make_accel_driver()
    accel.move_to_position = AsyncMock()
    brake = make_brake_driver()
    brake.move_to_position = AsyncMock()
    ff = MagicMock(spec=FeedforwardController)
    ff.has_model = False  # ペダルプランを生成せず従来経路（plan=None）で DriveLoop を起動
    learning_manager = MagicMock()
    learning_manager.generate_patterns = MagicMock(return_value=_make_patterns())
    return RobotController(
        accel_driver=accel,
        brake_driver=brake,
        can_reader=make_can_reader(),
        safety_monitor=make_safety_monitor(),
        pid=make_pid(),
        last_normal_shutdown=True,
        ff_controller=ff,
        safety_check=MagicMock(),
        learning_manager=learning_manager,  # type: ignore[arg-type]
    )


class TestDriveSessionLogging:
    """走行ログ収集の配線（セッション採番・DriveLoop 起動・end_session 記録）。"""

    @pytest.mark.asyncio
    async def test_auto_drive_with_log_writer_opens_session_and_builds_loop(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]

        log_writer = MagicMock()
        log_writer.start_session = AsyncMock(return_value="db-sess-1")
        log_writer.end_session = AsyncMock()

        session = await ctrl.start_auto_drive(
            "mode-uuid-1",
            mode=_make_mode(),
            profile=profile,
            log_writer=log_writer,  # type: ignore[arg-type]
        )

        log_writer.start_session.assert_awaited_once_with(
            "prof-uuid-1", "mode-uuid-1", "auto", cycle_id=None
        )
        assert session.id == "db-sess-1"
        assert ctrl._drive_loop is not None

        await ctrl.stop_auto_drive()
        log_writer.end_session.assert_awaited_once_with("db-sess-1", "completed")
        assert ctrl._drive_loop is None

    @pytest.mark.asyncio
    async def test_auto_drive_emergency_ends_session_with_emergency_status(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]

        log_writer = MagicMock()
        log_writer.start_session = AsyncMock(return_value="db-sess-2")
        log_writer.end_session = AsyncMock()

        await ctrl.start_auto_drive(
            "mode-uuid-1",
            mode=_make_mode(),
            profile=profile,
            log_writer=log_writer,  # type: ignore[arg-type]
        )
        await ctrl.emergency_stop()
        log_writer.end_session.assert_awaited_once_with("db-sess-2", "emergency")

    @pytest.mark.asyncio
    async def test_learning_drive_with_log_writer_opens_session_and_builds_loop(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]

        log_writer = MagicMock()
        log_writer.start_session = AsyncMock(return_value="learn-sess-1")
        log_writer.end_session = AsyncMock()
        log_writer.start_cycle = AsyncMock(return_value="cycle-1")

        await ctrl.arm_learning_drive()
        session = await ctrl.start_learning_drive(log_writer=log_writer)

        log_writer.start_cycle.assert_awaited_once_with("prof-uuid-1")
        log_writer.start_session.assert_awaited_once_with(
            "prof-uuid-1", None, "learning", cycle_id="cycle-1"
        )
        assert session.run_type == "learning"
        assert session.mode_id is None
        assert session.id == "learn-sess-1"
        ctrl._learning_manager.generate_patterns.assert_called_once_with(profile)  # type: ignore[attr-defined]
        assert ctrl._learning_loop is not None

        # 手動停止（/stop）で RUNNING→READY・セッション completed 終了
        await ctrl.stop()
        log_writer.end_session.assert_awaited_once_with("learn-sess-1", "completed")
        assert ctrl._learning_loop is None


class TestActiveCycleIdWiring:
    """学習運転で開設した学習サイクルへ後続の PID 適合走行が参加すること。"""

    @pytest.mark.asyncio
    async def test_tuning_drive_inherits_active_cycle_id(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]

        log_writer = MagicMock()
        log_writer.start_session = AsyncMock(side_effect=["learn-sess-1", "tune-sess-1"])
        log_writer.end_session = AsyncMock()
        log_writer.start_cycle = AsyncMock(return_value="cycle-1")

        await ctrl.arm_learning_drive()
        await ctrl.start_learning_drive(log_writer=log_writer)
        assert ctrl._active_cycle_id == "cycle-1"

        # 学習終了→READY（保持ブレーキのまま）。cycle_id は維持されたまま次の走行へ引き継ぐ。
        await ctrl.stop_learning_drive()
        assert ctrl._active_cycle_id == "cycle-1"

        def fake_build(
            mode: object,
            profile: object,
            lw: object,
            session_id: object,
            on_complete: object = None,
            disable_deviation_check: bool = False,
        ) -> None:
            ctrl._state = RobotState.READY
            ctrl._last_kpi_summary = {"n_samples": 1.0}
            ctrl._drive_complete.set()

        ctrl._build_and_start_drive_loop = fake_build  # type: ignore[assignment]
        await ctrl._run_tuning_drive(profile, log_writer)  # type: ignore[arg-type]

        second_call = log_writer.start_session.await_args_list[1]
        assert second_call.args[2] == "tuning"  # run_type
        assert second_call.kwargs["cycle_id"] == "cycle-1"

    @pytest.mark.asyncio
    async def test_active_cycle_id_property_and_session_cycle_id(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        assert ctrl.active_cycle_id is None

        log_writer = MagicMock()
        log_writer.start_session = AsyncMock(return_value="learn-sess-1")
        log_writer.end_session = AsyncMock()
        log_writer.start_cycle = AsyncMock(return_value="cycle-9")

        await ctrl.arm_learning_drive()
        session = await ctrl.start_learning_drive(log_writer=log_writer)

        assert ctrl.active_cycle_id == "cycle-9"
        assert session.cycle_id == "cycle-9"  # 返り値の DriveSession にも反映される

    @pytest.mark.asyncio
    async def test_select_profile_clears_active_cycle_id(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        ctrl._active_cycle_id = "leftover-cycle"

        ctrl.select_profile(profile)  # type: ignore[arg-type]

        assert ctrl._active_cycle_id is None

    @pytest.mark.asyncio
    async def test_start_learning_drive_without_log_writer_uses_local_uuid(self) -> None:
        import uuid

        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]

        await ctrl.arm_learning_drive()
        await ctrl.start_learning_drive(log_writer=None)

        assert ctrl._active_cycle_id is not None
        uuid.UUID(ctrl._active_cycle_id)  # UUID として解析できれば形式が正しい


class TestLearningCompleteEvent:
    """`_learning_complete` が学習サイクルオーケストレータの完了待ちとして機能すること。"""

    @pytest.mark.asyncio
    async def test_cleared_at_start_and_set_after_stop_learning_drive(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]
        ctrl._learning_complete.set()  # 事前にセットされていても start でクリアされること

        await ctrl.arm_learning_drive()
        await ctrl.start_learning_drive(log_writer=None)

        assert not ctrl._learning_complete.is_set()

        await ctrl.stop_learning_drive()

        assert ctrl._learning_complete.is_set()

    @pytest.mark.asyncio
    async def test_set_on_emergency_stop(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]

        await ctrl.arm_learning_drive()
        await ctrl.start_learning_drive(log_writer=None)
        assert not ctrl._learning_complete.is_set()

        await ctrl.emergency_stop()

        assert ctrl._learning_complete.is_set()

    @pytest.mark.asyncio
    async def test_set_on_generic_stop(self) -> None:
        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]

        await ctrl.arm_learning_drive()
        await ctrl.start_learning_drive(log_writer=None)
        assert not ctrl._learning_complete.is_set()

        await ctrl.stop()

        assert ctrl._learning_complete.is_set()

    @pytest.mark.asyncio
    async def test_learning_drive_without_log_writer_uses_local_uuid(self) -> None:
        from uuid import UUID  # noqa: PLC0415

        ctrl = _make_controller_for_logging()
        await advance_to_ready(ctrl)
        profile = _make_profile_with_calibration()
        ctrl.select_profile(profile)  # type: ignore[arg-type]

        await ctrl.arm_learning_drive()
        session = await ctrl.start_learning_drive(log_writer=None)
        UUID(session.id)  # ローカル採番の UUID として解析できる
        assert ctrl._log_writer is None
        await ctrl.stop()


# ---------------------------------------------------------------------------
# プロファイル⇔制御スタック整合（コードレビュー 2026-06-11 指摘 #4/#10）
# ---------------------------------------------------------------------------


def _make_safety_check() -> MagicMock:
    sc = MagicMock()
    sc.check_overcurrent = MagicMock(return_value=False)
    sc.check_deviation = MagicMock(return_value=False)
    sc.set_stop_config = MagicMock()
    return sc


def _make_ctrl_with_ff() -> "RobotController":
    from src.domain.control.feedforward import FeedforwardController  # noqa: PLC0415

    return RobotController(
        accel_driver=make_accel_driver(),
        brake_driver=make_brake_driver(),
        can_reader=make_can_reader(),
        safety_monitor=make_safety_monitor(),
        pid=make_pid(),
        last_normal_shutdown=True,
        ff_controller=FeedforwardController(),
        safety_check=_make_safety_check(),
    )


def _profile(profile_id: str, model_path: str | None = None, **ffp_kwargs) -> object:
    from src.models.profile import (  # noqa: PLC0415
        FeedforwardParams,
        PIDGains,
        StopConfig,
        VehicleProfile,
    )

    return VehicleProfile(
        id=profile_id,
        name=f"Car-{profile_id}",
        max_accel_opening=80.0,
        max_brake_opening=60.0,
        max_speed=180.0,
        max_decel_g=0.5,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None,
        model_path=model_path,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
        feedforward_params=FeedforwardParams(**ffp_kwargs),
    )


class TestProfileModelLifecycle:
    @pytest.mark.asyncio
    async def test_switching_to_profile_without_model_unloads_previous(self, tmp_path) -> None:
        """レビュー #4: model_path なしプロファイルへの切替で前車両のモデルが残留しない。"""
        from tests.unit.test_feedforward import make_model_file  # noqa: PLC0415

        ctrl = _make_ctrl_with_ff()
        await ctrl.start()  # STANDBY
        model_path = str(make_model_file(tmp_path))

        ctrl.select_profile(_profile("vehicle-a", model_path=model_path))  # type: ignore[arg-type]
        assert ctrl._ff_controller.has_model is True  # type: ignore[union-attr]

        ctrl.select_profile(_profile("vehicle-b", model_path=None))  # type: ignore[arg-type]
        assert ctrl._ff_controller.has_model is False  # type: ignore[union-attr]

    @pytest.mark.asyncio
    async def test_failed_model_load_leaves_no_stale_model(self, tmp_path) -> None:
        """ロード失敗時も前車両のモデルが残留せず、警告のみで選択は成功する。"""
        from tests.unit.test_feedforward import make_model_file  # noqa: PLC0415

        ctrl = _make_ctrl_with_ff()
        await ctrl.start()
        ctrl.select_profile(  # type: ignore[arg-type]
            _profile("vehicle-a", model_path=str(make_model_file(tmp_path)))
        )

        ctrl.select_profile(  # type: ignore[arg-type]
            _profile("vehicle-b", model_path="/nonexistent/model.pkl")
        )
        assert ctrl._ff_controller.has_model is False  # type: ignore[union-attr]
        assert ctrl.get_active_profile().id == "vehicle-b"  # type: ignore[union-attr]

    @pytest.mark.asyncio
    async def test_pid_output_limit_applied_from_profile(self) -> None:
        ctrl = _make_ctrl_with_ff()
        await ctrl.start()
        ctrl.select_profile(_profile("p1", pid_output_limit_pct=30.0))  # type: ignore[arg-type]
        assert ctrl._pid._output_limit == pytest.approx(30.0)


class TestRefreshActiveProfile:
    @pytest.mark.asyncio
    async def test_refresh_applies_new_model_and_params(self, tmp_path) -> None:
        """レビュー #10: 学習後の refresh でモデルと物理定数が即時反映される。"""
        from tests.unit.test_feedforward import make_model_file  # noqa: PLC0415

        ctrl = _make_ctrl_with_ff()
        await ctrl.start()
        ctrl.select_profile(_profile("p1", model_path=None))  # type: ignore[arg-type]
        assert ctrl._ff_controller.has_model is False  # type: ignore[union-attr]

        trained = _profile("p1", model_path=str(make_model_file(tmp_path)), creep_speed_kmh=9.0)
        assert ctrl.refresh_active_profile(trained) is True  # type: ignore[arg-type]
        assert ctrl._ff_controller.has_model is True  # type: ignore[union-attr]
        assert ctrl._ff_controller._params.creep_speed_kmh == 9.0  # type: ignore[union-attr]
        assert ctrl.get_active_profile() is trained

    @pytest.mark.asyncio
    async def test_refresh_ignores_non_active_profile(self) -> None:
        ctrl = _make_ctrl_with_ff()
        await ctrl.start()
        ctrl.select_profile(_profile("p1"))  # type: ignore[arg-type]
        assert ctrl.refresh_active_profile(_profile("other")) is False  # type: ignore[arg-type]
        assert ctrl.get_active_profile().id == "p1"  # type: ignore[union-attr]

    @pytest.mark.asyncio
    async def test_refresh_rejected_while_running(self) -> None:
        """走行中の制御パラメータ差し替えは反映しない（挙動が不定になるため）。"""
        ctrl = _make_ctrl_with_ff()
        await ctrl.start()
        ctrl.select_profile(_profile("p1"))  # type: ignore[arg-type]
        ctrl._state = RobotState.RUNNING
        assert ctrl.refresh_active_profile(_profile("p1")) is False  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# 非常停止経路（コードレビュー 2026-06-11 指摘 #6/#15）
# ---------------------------------------------------------------------------


class TestEmergencyDispatch:
    @pytest.mark.asyncio
    async def test_dispatch_routes_through_safety_monitor(self) -> None:
        """内部起因の非常停止が SafetyMonitor ディスパッチャを経由する（指摘 #15）。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        # 実構成では trigger_emergency → 登録コールバック（emergency_stop）が発火する
        ctrl._safety_monitor.trigger_emergency = AsyncMock(side_effect=ctrl.emergency_stop)  # type: ignore[attr-defined]

        await ctrl._dispatch_emergency()

        ctrl._safety_monitor.trigger_emergency.assert_awaited_once()  # type: ignore[attr-defined]
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY
        # フォールバックで二重に home_return しない（冪等）
        assert ctrl._accel_driver.home_return.await_count == 1  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_dispatch_falls_back_when_no_callback_registered(self) -> None:
        """コールバック未登録の DI 構成でも emergency_stop に倒れる（安全フォールバック）。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        # trigger_emergency が何もしないスタブ → フォールバックが効く
        await ctrl._dispatch_emergency()

        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY
        assert ctrl._accel_driver.home_return.await_count == 1  # type: ignore[attr-defined]


class TestEmergencyStopJoinsCycle:
    @pytest.mark.asyncio
    async def test_stop_and_join_completes_before_home_return(self) -> None:
        """レビュー #6: 飛行中サイクルの完了待ちが home_return より先に行われる。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)

        order: list[str] = []

        mock_loop = MagicMock()
        mock_loop.stop = MagicMock()
        mock_loop.kpi_summary = {"n_samples": 0.0}

        async def _join(timeout_s: float = 2.0) -> None:
            order.append("join")

        async def _home() -> None:
            order.append("home")

        mock_loop.stop_and_join = _join
        ctrl._drive_loop = mock_loop  # type: ignore[assignment]
        ctrl._accel_driver.home_return = _home  # type: ignore[attr-defined]

        await ctrl.emergency_stop()

        assert order.index("join") < order.index("home")
        assert ctrl._drive_loop is None


# ---------------------------------------------------------------------------
# テレメトリ鮮度（コードレビュー 2026-06-11 指摘 #16）
# ---------------------------------------------------------------------------


class TestRealtimeSnapshotFreshness:
    @pytest.mark.asyncio
    async def test_fresh_snapshot_returned_from_cache(self) -> None:
        import asyncio as _asyncio  # noqa: PLC0415

        from src.models.system_state import RealtimeSnapshot  # noqa: PLC0415

        ctrl = make_controller()
        snap = RealtimeSnapshot(
            actual_speed_kmh=42.0,
            accel_pos=10,
            brake_pos=20,
            accel_current_ma=1.0,
            brake_current_ma=2.0,
            captured_at=_asyncio.get_running_loop().time(),
        )
        mock_loop = MagicMock()
        mock_loop.last_snapshot = snap
        ctrl._drive_loop = mock_loop  # type: ignore[assignment]

        assert await ctrl.get_realtime_data() is snap
        ctrl._can_reader.read_speed.assert_not_called()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_stale_snapshot_falls_back_to_hardware_read(self) -> None:
        """サイクル凍結で古くなったキャッシュは配信せず直接読み取る（障害マスキング防止）。"""
        import asyncio as _asyncio  # noqa: PLC0415

        from src.models.system_state import RealtimeSnapshot  # noqa: PLC0415

        ctrl = make_controller()
        stale = RealtimeSnapshot(
            actual_speed_kmh=42.0,
            accel_pos=10,
            brake_pos=20,
            accel_current_ma=1.0,
            brake_current_ma=2.0,
            captured_at=_asyncio.get_running_loop().time() - 10.0,
        )
        mock_loop = MagicMock()
        mock_loop.last_snapshot = stale
        ctrl._drive_loop = mock_loop  # type: ignore[assignment]

        result = await ctrl.get_realtime_data()

        assert result is not stale
        ctrl._can_reader.read_speed.assert_awaited_once()  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# KPI サマリ記録（コードレビュー 2026-06-11 指摘 #7）
# ---------------------------------------------------------------------------


class TestKpiSummaryRecording:
    @pytest.mark.asyncio
    async def test_summary_recorded_on_stop_auto_drive(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        ctrl._state = RobotState.RUNNING

        summary = {
            "n_samples": 100.0,
            "max_abs_deviation_kmh": 0.4,
            "p95_kmh": 0.15,
            "reversal_max_per_5s": 1.0,
            "hard_limit_violations": 0.0,
        }
        mock_loop = MagicMock()
        mock_loop.stop_and_join = AsyncMock()
        mock_loop.kpi_summary = summary
        mock_loop.stall_summary = {
            "stall_count": 0.0,
            "stall_total_s": 0.0,
            "stall_max_s": 0.0,
        }
        ctrl._drive_loop = mock_loop  # type: ignore[assignment]

        await ctrl.stop_auto_drive()

        assert ctrl.last_kpi_summary == summary

    @pytest.mark.asyncio
    async def test_empty_run_does_not_overwrite_summary(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        ctrl._state = RobotState.RUNNING
        ctrl._last_kpi_summary = {"n_samples": 10.0}

        mock_loop = MagicMock()
        mock_loop.stop_and_join = AsyncMock()
        mock_loop.kpi_summary = {"n_samples": 0.0}
        ctrl._drive_loop = mock_loop  # type: ignore[assignment]

        await ctrl.stop_auto_drive()

        assert ctrl.last_kpi_summary == {"n_samples": 10.0}


class TestPidTuningOrchestration:
    @pytest.mark.asyncio
    async def test_validation_requires_ready_state(self) -> None:
        ctrl = make_controller()  # BOOTING
        with pytest.raises(InvalidStateTransition):
            await ctrl.run_pid_validation(_calibrated_profile())

    @pytest.mark.asyncio
    async def test_validation_requires_calibration(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()
        uncalibrated = VehicleProfile(**{**prof.__dict__, "calibration": None})
        with pytest.raises(InvalidStateTransition):
            await ctrl.run_pid_validation(uncalibrated)

    @pytest.mark.asyncio
    async def test_validation_restores_active_profile_gains_afterward(self) -> None:
        """A4 回帰テスト: 検証走行後、ライブ PID は検証対象プロファイルのゲインではなく
        アクティブプロファイルのゲインに戻る（両者が異なる場合の乖離防止）。"""
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        active_prof = _calibrated_profile()
        ctrl._active_profile = active_prof
        other_prof = dataclasses.replace(
            active_prof, id="other", pid_gains=PIDGains(kp=9.0, ki=0.9, kd=0.0)
        )

        async def fake_drive(
            profile: object, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            return {
                "n_samples": 10.0,
                "p95_kmh": 0.1,
                "max_abs_deviation_kmh": 0.2,
                "reversal_max_per_5s": 0.0,
                "hard_limit_violations": 0.0,
            }

        ctrl._run_tuning_drive = fake_drive  # type: ignore[assignment]

        await ctrl.run_pid_validation(other_prof)

        # 検証中は other_prof のゲイン(9.0)を使ったはずだが、終了後は
        # アクティブプロファイル(active_prof)のゲインに戻っている
        assert ctrl._pid._kp == pytest.approx(active_prof.pid_gains.kp)
        assert ctrl._pid._kp != pytest.approx(other_prof.pid_gains.kp)

    @pytest.mark.asyncio
    async def test_tuning_session_drives_runs_and_applies_best(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()

        # _run_tuning_drive をモックし、kp が大きいほど良い（コスト低い）KPI を返す。
        async def fake_drive(
            profile: object, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            kp = ctrl._pid._kp  # 直近に set_gains された候補
            # p95 が kp とともに改善する単調なダミー応答
            p95 = max(0.05, 1.0 - 0.1 * kp)
            return {
                "n_samples": 500.0,
                "p95_kmh": p95,
                "max_abs_deviation_kmh": p95 * 2,
                "reversal_max_per_5s": 1.0,
                "hard_limit_violations": 0.0,
            }

        ctrl._run_tuning_drive = fake_drive  # type: ignore[assignment]

        best, history = await ctrl.run_pid_tuning_session(prof, None, max_runs=12)

        assert len(history) >= 2
        assert best.kp >= prof.pid_gains.kp  # kp を上げる方向に改善している

    @pytest.mark.asyncio
    async def test_tuning_session_restores_active_profile_gains_not_best(self) -> None:
        """A4 回帰テスト: 探索終了後、ライブ PID は best ではなくアクティブプロファイルの
        ゲインへ戻る（呼び出し元が persist+refresh するまで best を保持し続けない。
        ライブ PID と永続 VehicleProfile の二重所有権を解消する）。"""
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()
        ctrl._active_profile = prof  # 探索対象と同一プロファイルがアクティブ

        async def fake_drive(
            profile: object, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            kp = ctrl._pid._kp
            p95 = max(0.05, 1.0 - 0.1 * kp)
            return {
                "n_samples": 500.0,
                "p95_kmh": p95,
                "max_abs_deviation_kmh": p95 * 2,
                "reversal_max_per_5s": 1.0,
                "hard_limit_violations": 0.0,
            }

        ctrl._run_tuning_drive = fake_drive  # type: ignore[assignment]

        best, _ = await ctrl.run_pid_tuning_session(prof, None, max_runs=12)

        assert best.kp > prof.pid_gains.kp  # 探索は改善している
        # ライブ PID は best ではなく、アクティブプロファイル（未永続化）のゲインに戻る
        assert ctrl._pid._kp == pytest.approx(prof.pid_gains.kp)

    @pytest.mark.asyncio
    async def test_tuning_session_injects_candidate_preview_into_drive_profile(self) -> None:
        """候補の pid_preview_s が差し替えられたプロファイルで各走行が実行される。"""
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()
        seen_previews: list[float] = []

        async def fake_drive(
            profile: VehicleProfile, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            seen_previews.append(profile.dynamics_params.pid_preview_s)
            preview = profile.dynamics_params.pid_preview_s
            cost_p95 = max(0.05, 1.0 - abs(preview - 0.75))
            return {
                "n_samples": 500.0,
                "p95_kmh": cost_p95,
                "max_abs_deviation_kmh": cost_p95 * 2,
                "reversal_max_per_5s": 1.0,
                "hard_limit_violations": 0.0,
            }

        ctrl._run_tuning_drive = fake_drive  # type: ignore[assignment]

        best, history = await ctrl.run_pid_tuning_session(prof, None, max_runs=12)

        # 元プロファイルの pid_preview_s(0.0)以外の値も走行に渡っている
        assert any(p != 0.0 for p in seen_previews)
        assert best.pid_preview_s in seen_previews
        assert all("pid_preview_s" in h for h in history)
        # 呼び出し元の元プロファイルは不変（dataclasses.replace で新規オブジェクトを渡す）
        assert prof.dynamics_params.pid_preview_s == 0.0

    @pytest.mark.asyncio
    async def test_tuning_drive_aborts_on_emergency(self) -> None:
        from src.app.robot_controller import PidTuningAborted

        ctrl = make_controller()
        ctrl._state = RobotState.READY
        ctrl._active_profile = _calibrated_profile()
        prof = ctrl._active_profile

        def fake_build(
            mode: object,
            profile: object,
            log_writer: object,
            session_id: object,
            on_complete: object = None,
            disable_deviation_check: bool = False,
        ) -> None:
            # 走行開始直後に非常停止が発火したのを模擬
            ctrl._state = RobotState.EMERGENCY
            ctrl._drive_complete.set()

        ctrl._build_and_start_drive_loop = fake_build  # type: ignore[assignment]

        with pytest.raises(PidTuningAborted):
            await ctrl._run_tuning_drive(prof, None)

    @pytest.mark.asyncio
    async def test_manual_stop_during_tuning_drive_raises_aborted_not_success(self) -> None:
        """W2 回帰テスト: チューニング走行中に手動 stop() が割り込んだ場合も、
        状態が READY に戻るだけの「正常完了」とは区別して PidTuningAborted を送出する
        （さもないと不完全な走行の KPI を正常完了として採用してしまう）。"""
        import asyncio

        from src.app.robot_controller import PidTuningAborted

        ctrl = make_controller()
        ctrl._state = RobotState.READY
        ctrl._active_profile = _calibrated_profile()
        prof = ctrl._active_profile

        def fake_build(
            mode: object,
            profile: object,
            log_writer: object,
            session_id: object,
            on_complete: object = None,
            disable_deviation_check: bool = False,
        ) -> None:
            # DriveLoop を起動せず、代わりに手動 stop() が割り込んだことを模擬する
            asyncio.ensure_future(ctrl.stop())

        ctrl._build_and_start_drive_loop = fake_build  # type: ignore[assignment]

        with pytest.raises(PidTuningAborted):
            await ctrl._run_tuning_drive(prof, None)

    @pytest.mark.asyncio
    async def test_run_pid_validation_skips_release_stop_hold_when_manually_stopped(self) -> None:
        """W2 回帰テスト: 手動 stop() が既に home_return 済みの場合、
        release_stop_hold を二重に送出しない（干渉防止）。"""
        import asyncio

        from src.app.robot_controller import PidTuningAborted

        ctrl = make_controller()
        ctrl._state = RobotState.READY
        ctrl._active_profile = _calibrated_profile()
        prof = ctrl._active_profile

        def fake_build(
            mode: object,
            profile: object,
            log_writer: object,
            session_id: object,
            on_complete: object = None,
            disable_deviation_check: bool = False,
        ) -> None:
            asyncio.ensure_future(ctrl.stop())

        ctrl._build_and_start_drive_loop = fake_build  # type: ignore[assignment]
        ctrl._accel_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.reset_mock()  # type: ignore[attr-defined]

        with pytest.raises(PidTuningAborted):
            await ctrl.run_pid_validation(prof)

        # stop() 内で1回 home_return が呼ばれ、release_stop_hold からは呼ばれない（二重防止）
        assert ctrl._accel_driver.home_return.call_count == 1  # type: ignore[attr-defined]
        assert ctrl._brake_driver.home_return.call_count == 1  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_tuning_drive_starts_without_precheck(self) -> None:
        # 仕様: PID最適化は走行前チェック/arm をせず、停止保持状態から直接走行する。
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        ctrl._active_profile = _calibrated_profile()
        prof = ctrl._active_profile
        ctrl.arm_auto_drive = AsyncMock()  # type: ignore[assignment]
        ctrl._pre_check_runner = MagicMock()
        ctrl._pre_check_runner.run = AsyncMock()

        def fake_build(
            mode: object,
            profile: object,
            log_writer: object,
            session_id: object,
            on_complete: object = None,
            disable_deviation_check: bool = False,
        ) -> None:
            # 走行完了を模擬（停止保持して READY へ）
            ctrl._state = RobotState.READY
            ctrl._last_kpi_summary = {"n_samples": 100.0, "p95_kmh": 0.1}
            ctrl._drive_complete.set()

        ctrl._build_and_start_drive_loop = fake_build  # type: ignore[assignment]

        kpi = await ctrl._run_tuning_drive(prof, None)

        assert kpi["n_samples"] == 100.0
        ctrl.arm_auto_drive.assert_not_called()  # arm しない
        ctrl._pre_check_runner.run.assert_not_called()  # 走行前チェックしない

    @pytest.mark.asyncio
    async def test_tuning_drive_transitions_directly_ready_to_running(self) -> None:
        """A3 回帰テスト: READY→RUNNING は状態機械の第一級遷移として直接行われ、
        PRE_CHECK を経由する空遷移ではない。"""
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        ctrl._active_profile = _calibrated_profile()
        prof = ctrl._active_profile

        def fake_build(
            mode: object,
            profile: object,
            log_writer: object,
            session_id: object,
            on_complete: object = None,
            disable_deviation_check: bool = False,
        ) -> None:
            ctrl._state = RobotState.READY
            ctrl._last_kpi_summary = {"n_samples": 1.0}
            ctrl._drive_complete.set()

        ctrl._build_and_start_drive_loop = fake_build  # type: ignore[assignment]

        transitions: list[RobotState] = []
        original_transition = ctrl._transition

        def spy_transition(new_state: RobotState) -> None:
            transitions.append(new_state)
            original_transition(new_state)

        ctrl._transition = spy_transition  # type: ignore[method-assign]

        await ctrl._run_tuning_drive(prof, None)

        assert transitions.count(RobotState.PRE_CHECK) == 0
        assert transitions.count(RobotState.RUNNING) == 1

    @pytest.mark.asyncio
    async def test_tuning_drive_opens_session_with_null_mode_id(self) -> None:
        # 回帰防止: drive_sessions.mode_id は UUID。規定パターンは永続モードでないため
        # mode_id=None で開始する（"pid-tune" を渡すと DB DataError→500）。
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        ctrl._active_profile = _calibrated_profile()
        prof = ctrl._active_profile
        log_writer = MagicMock()
        log_writer.start_session = AsyncMock(return_value="sess-1")
        log_writer.end_session = AsyncMock()

        def fake_build(
            mode: object,
            profile: object,
            lw: object,
            session_id: object,
            on_complete: object = None,
            disable_deviation_check: bool = False,
        ) -> None:
            ctrl._state = RobotState.READY
            ctrl._last_kpi_summary = {"n_samples": 1.0}
            ctrl._drive_complete.set()

        ctrl._build_and_start_drive_loop = fake_build  # type: ignore[assignment]

        await ctrl._run_tuning_drive(prof, log_writer)

        log_writer.start_session.assert_awaited_once()
        # start_session(profile_id, mode_id, run_type) → mode_id は None
        assert log_writer.start_session.await_args.args[1] is None

    @pytest.mark.asyncio
    async def test_tuning_drive_disables_deviation_check(self) -> None:
        # 適合中は逸脱による非常停止を無効化して走行する（未適合ゲインでも適合できる）。
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        ctrl._active_profile = _calibrated_profile()
        prof = ctrl._active_profile
        captured: dict[str, object] = {}

        def fake_build(
            mode: object,
            profile: object,
            lw: object,
            session_id: object,
            on_complete: object = None,
            disable_deviation_check: bool = False,
        ) -> None:
            captured["disable_deviation_check"] = disable_deviation_check
            ctrl._state = RobotState.READY
            ctrl._last_kpi_summary = {"n_samples": 1.0}
            ctrl._drive_complete.set()

        ctrl._build_and_start_drive_loop = fake_build  # type: ignore[assignment]

        await ctrl._run_tuning_drive(prof, None)

        assert captured["disable_deviation_check"] is True


class TestReleaseOnFinishAndOnRun:
    """2段階学習フロー向けの release_on_finish・on_run コールバック。"""

    @pytest.mark.asyncio
    async def test_release_on_finish_true_releases_after_normal_completion(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()

        async def fake_drive(
            profile: object, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            return {
                "n_samples": 500.0,
                "p95_kmh": 0.05,
                "max_abs_deviation_kmh": 0.1,
                "reversal_max_per_5s": 0.0,
                "hard_limit_violations": 0.0,
            }

        ctrl._run_tuning_drive = fake_drive  # type: ignore[assignment]

        await ctrl.run_pid_tuning_session(prof, None, max_runs=2, release_on_finish=True)

        ctrl._accel_driver.home_return.assert_awaited()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_release_on_finish_false_skips_release_after_normal_completion(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()

        async def fake_drive(
            profile: object, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            return {
                "n_samples": 500.0,
                "p95_kmh": 0.05,
                "max_abs_deviation_kmh": 0.1,
                "reversal_max_per_5s": 0.0,
                "hard_limit_violations": 0.0,
            }

        ctrl._run_tuning_drive = fake_drive  # type: ignore[assignment]

        await ctrl.run_pid_tuning_session(prof, None, max_runs=2, release_on_finish=False)

        ctrl._accel_driver.home_return.assert_not_awaited()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_release_on_finish_false_still_releases_on_exception(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()

        async def failing_drive(
            profile: object, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            raise RuntimeError("boom")

        ctrl._run_tuning_drive = failing_drive  # type: ignore[assignment]

        with pytest.raises(RuntimeError):
            await ctrl.run_pid_tuning_session(prof, None, max_runs=2, release_on_finish=False)

        ctrl._accel_driver.home_return.assert_awaited()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_on_run_called_with_run_index_gains_and_cost(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()
        calls: list[tuple[int, object, float]] = []

        async def fake_drive(
            profile: object, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            kp = ctrl._pid._kp
            p95 = max(0.05, 1.0 - 0.1 * kp)
            return {
                "n_samples": 500.0,
                "p95_kmh": p95,
                "max_abs_deviation_kmh": p95 * 2,
                "reversal_max_per_5s": 1.0,
                "hard_limit_violations": 0.0,
            }

        ctrl._run_tuning_drive = fake_drive  # type: ignore[assignment]

        def on_run(run_index: int, gains: object, cost: float) -> None:
            calls.append((run_index, gains, cost))

        await ctrl.run_pid_tuning_session(prof, None, max_runs=3, on_run=on_run)

        assert len(calls) == 3
        assert [c[0] for c in calls] == [1, 2, 3]

    @pytest.mark.asyncio
    async def test_on_run_exception_aborts_and_releases(self) -> None:
        ctrl = make_controller()
        ctrl._state = RobotState.READY
        prof = _calibrated_profile()

        async def fake_drive(
            profile: object, log_writer: object, mode: object = None
        ) -> dict[str, float]:
            return {
                "n_samples": 500.0,
                "p95_kmh": 0.05,
                "max_abs_deviation_kmh": 0.1,
                "reversal_max_per_5s": 0.0,
                "hard_limit_violations": 0.0,
            }

        ctrl._run_tuning_drive = fake_drive  # type: ignore[assignment]

        class Aborted(Exception):
            pass

        def on_run(run_index: int, gains: object, cost: float) -> None:
            raise Aborted()

        with pytest.raises(Aborted):
            await ctrl.run_pid_tuning_session(prof, None, max_runs=3, on_run=on_run)

        ctrl._accel_driver.home_return.assert_awaited()  # type: ignore[attr-defined]


# ── タイムスケジュール走行の統合（release_all の保証を含む）─────────────


def _make_button_servo() -> MagicMock:
    servo = MagicMock()
    servo.connect = AsyncMock()
    servo.press = AsyncMock()
    servo.release_all = AsyncMock()
    return servo


def _make_time_schedule(with_button: bool = False) -> object:
    from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule  # noqa: PLC0415

    return TimeSchedule(
        id="sched-uuid-1",
        name="Sched",
        description="",
        pedal_points=[PedalPoint(0.0, 0.0, 0.0), PedalPoint(5.0, 40.0, 0.0)],
        button_events=[ButtonEvent(0.5, 0, 1.0)] if with_button else [],
        total_duration=5.0,
        created_at=datetime.now(tz=UTC),
    )


def _make_controller_with_button_servo() -> tuple[RobotController, MagicMock]:
    button = _make_button_servo()
    safety = MagicMock()
    safety.check_overcurrent = MagicMock(return_value=False)
    safety.check_deviation = MagicMock(return_value=False)
    safety.set_stop_config = MagicMock()
    ctrl = RobotController(
        accel_driver=make_accel_driver(),
        brake_driver=make_brake_driver(),
        can_reader=make_can_reader(),
        safety_monitor=make_safety_monitor(),
        pid=make_pid(),
        last_normal_shutdown=True,
        safety_check=safety,
        button_servo=button,
    )
    return ctrl, button


class TestScheduleDrive:
    @pytest.mark.asyncio
    async def test_start_connects_button_servo_on_start(self) -> None:
        ctrl, button = _make_controller_with_button_servo()
        await ctrl.start()
        button.connect.assert_awaited_once()

    @pytest.mark.asyncio
    async def test_start_schedule_drive_builds_and_starts_loop(self) -> None:
        ctrl, button = _make_controller_with_button_servo()
        await advance_to_ready(ctrl)
        ctrl.select_profile(_make_profile_with_calibration())
        session = await ctrl.start_schedule_drive(
            _make_time_schedule(), profile=ctrl.get_active_profile()
        )
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING
        assert ctrl._schedule_loop is not None
        assert ctrl._schedule_loop.is_running is True
        assert session.run_type == "auto"
        assert session.mode_id is None
        await ctrl.stop_schedule_drive()  # クリーンアップ

    @pytest.mark.asyncio
    async def test_missing_profile_rejected_before_transitioning_to_running(self) -> None:
        """W4 回帰テスト: profile が None のまま開始しようとすると RUNNING へ遷移する
        前に拒否される（ファントム RUNNING を防ぐ）。"""
        ctrl, button = _make_controller_with_button_servo()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.start_schedule_drive(_make_time_schedule(), profile=None)
        assert ctrl.get_system_state().robot_state == RobotState.READY
        assert ctrl.get_system_state().active_session_id is None
        assert ctrl._schedule_loop is None

    @pytest.mark.asyncio
    async def test_start_schedule_runs_precheck_with_button_flag(self) -> None:
        from src.models.pre_check import PreCheckResult  # noqa: PLC0415

        ctrl, button = _make_controller_with_button_servo()
        runner = AsyncMock()
        runner.run = AsyncMock(return_value=PreCheckResult(passed=True, items=[]))
        runner.set_profile = MagicMock()
        ctrl._pre_check_runner = runner
        await advance_to_ready(ctrl)
        ctrl.select_profile(_make_profile_with_calibration())
        await ctrl.start_schedule_drive(
            _make_time_schedule(with_button=True), profile=ctrl.get_active_profile()
        )
        runner.run.assert_awaited_once()
        assert runner.run.await_args.kwargs.get("include_button_servo") is True
        await ctrl.stop_schedule_drive()

    @pytest.mark.asyncio
    async def test_stop_schedule_drive_releases_and_homes(self) -> None:
        ctrl, button = _make_controller_with_button_servo()
        await advance_to_ready(ctrl)
        mock_loop = MagicMock()
        mock_loop.stop_and_join = AsyncMock()
        ctrl._schedule_loop = mock_loop
        ctrl._state = RobotState.RUNNING
        await ctrl.stop_schedule_drive()
        mock_loop.stop_and_join.assert_awaited_once()
        button.release_all.assert_awaited_once()
        ctrl._accel_driver.home_return.assert_awaited()
        ctrl._brake_driver.home_return.assert_awaited()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_emergency_stop_releases_button_servo(self) -> None:
        ctrl, button = _make_controller_with_button_servo()
        await advance_to_ready(ctrl)
        mock_loop = MagicMock()
        mock_loop.stop = MagicMock()
        mock_loop.stop_and_join = AsyncMock()
        ctrl._schedule_loop = mock_loop
        ctrl._state = RobotState.RUNNING
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY
        mock_loop.stop.assert_called_once()
        button.release_all.assert_awaited()

    @pytest.mark.asyncio
    async def test_stop_releases_button_servo_during_schedule(self) -> None:
        ctrl, button = _make_controller_with_button_servo()
        await advance_to_ready(ctrl)
        mock_loop = MagicMock()
        mock_loop.stop_and_join = AsyncMock()
        ctrl._schedule_loop = mock_loop
        ctrl._state = RobotState.RUNNING
        await ctrl.stop()
        button.release_all.assert_awaited()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_shutdown_releases_button_servo(self) -> None:
        ctrl, button = _make_controller_with_button_servo()
        await advance_to_ready(ctrl)
        await ctrl.shutdown()
        button.release_all.assert_awaited()


# ---------------------------------------------------------------------------
# ILC 配線（Stage C: prepare で補正ロード・正常完了で学習）
# ---------------------------------------------------------------------------


def _make_controller_with_ilc() -> tuple[RobotController, AsyncMock]:
    from src.domain.control.ilc import ILCController, ILCTable

    ilc_service = AsyncMock()
    # prepare は補正コントローラを返す（sentinel）。learn_from_session は AsyncMock。
    ilc_service.prepare = AsyncMock(return_value=ILCController(ILCTable(efforts=[1.0, 1.0])))
    ilc_service.learn_from_session = AsyncMock()
    ctrl = RobotController(
        accel_driver=make_accel_driver(),
        brake_driver=make_brake_driver(),
        can_reader=make_can_reader(),
        safety_monitor=make_safety_monitor(),
        pid=make_pid(),
        last_normal_shutdown=True,
        ff_controller=make_ff_controller(),
        safety_check=make_safety_check(),
        ilc_service=ilc_service,
    )
    return ctrl, ilc_service


class TestILCWiring:
    @pytest.mark.asyncio
    async def test_prepare_result_passed_to_drive_loop(self) -> None:
        """start_auto_drive が prepare の ILCController を DriveLoop に渡す。"""
        ctrl, ilc_service = _make_controller_with_ilc()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1", mode=_make_mode(), profile=_calibrated_profile()
        )
        ilc_service.prepare.assert_awaited_once()
        assert ctrl._drive_loop is not None
        assert ctrl._drive_loop._ilc is ilc_service.prepare.return_value

    @pytest.mark.asyncio
    async def test_natural_completion_triggers_learning(self) -> None:
        """DriveLoop の正常完了コールバックで learn_from_session が起動する。"""
        ctrl, ilc_service = _make_controller_with_ilc()
        await advance_to_ready(ctrl)
        session = await ctrl.start_auto_drive(
            mode_id="mode-1", mode=_make_mode(), profile=_calibrated_profile()
        )
        # DriveLoop 自然完了と同じく on_complete を呼ぶ
        on_complete = ctrl._drive_loop._on_complete  # type: ignore[union-attr]
        await on_complete()
        await asyncio.sleep(0)  # fire-and-forget タスクを走らせる
        ilc_service.learn_from_session.assert_awaited_once()
        assert ilc_service.learn_from_session.await_args.args[0] == session.id

    @pytest.mark.asyncio
    async def test_manual_stop_does_not_learn(self) -> None:
        """手動停止（stop）では learn_from_session を呼ばない。"""
        ctrl, ilc_service = _make_controller_with_ilc()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(
            mode_id="mode-1", mode=_make_mode(), profile=_calibrated_profile()
        )
        await ctrl.stop()
        await asyncio.sleep(0)
        ilc_service.learn_from_session.assert_not_awaited()


class TestPedalPlanBuild:
    """_build_pedal_plan: FF モデルの有無でプラン生成／None を切り替える（P-5）。"""

    def test_returns_none_when_no_model(self) -> None:
        ctrl = make_controller()  # make_ff_controller は has_model=False
        plan = ctrl._build_pedal_plan(_make_mode(), _calibrated_profile())
        assert plan is None

    def test_returns_none_when_ff_controller_absent(self) -> None:
        ctrl = make_controller()
        ctrl._ff_controller = None
        assert ctrl._build_pedal_plan(_make_mode(), _calibrated_profile()) is None

    def test_builds_plan_when_model_loaded(self) -> None:
        from src.domain.control.pedal_plan import PedalPlan

        ctrl = make_controller()
        ff = MagicMock(spec=FeedforwardController)
        ff.has_model = True
        ctrl._ff_controller = ff
        sentinel = PedalPlan()
        with pytest.MonkeyPatch.context() as mp:
            called: dict[str, object] = {}

            def fake_build(mode, ffc, params, **kw):  # type: ignore[no-untyped-def]
                called["mode"] = mode
                called["ff"] = ffc
                return sentinel

            mp.setattr("src.app.robot_controller.PedalPlanner.build", staticmethod(fake_build))
            mode = _make_mode()
            profile = _calibrated_profile()
            plan = ctrl._build_pedal_plan(mode, profile)
        assert plan is sentinel
        assert called["mode"] is mode
        assert called["ff"] is ff
