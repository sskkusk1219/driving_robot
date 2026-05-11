from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest

from src.app.robot_controller import (
    InvalidStateTransition,
    RobotController,
)
from src.domain.control.pid import PIDController
from src.models.profile import PIDGains, StopConfig
from src.models.system_state import RobotState


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
    return monitor


def make_pid() -> PIDController:
    gains = PIDGains(kp=1.0, ki=0.0, kd=0.0)
    return PIDController(kp=gains.kp, ki=gains.ki, kd=gains.kd)


def make_stop_config() -> StopConfig:
    return StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0)


def make_controller(last_normal_shutdown: bool = True) -> RobotController:
    return RobotController(
        accel_driver=make_accel_driver(),
        brake_driver=make_brake_driver(),
        can_reader=make_can_reader(),
        safety_monitor=make_safety_monitor(),
        pid=make_pid(),
        last_normal_shutdown=last_normal_shutdown,
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
        await ctrl.start_auto_drive(mode_id="mode-1")
        await ctrl.stop()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_stop_from_running_calls_home_return_and_servo_off(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
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
        await ctrl.start_auto_drive(mode_id="mode-1")
        await ctrl.stop()
        assert ctrl._last_normal_shutdown is True

    @pytest.mark.asyncio
    async def test_stop_clears_session_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
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


class TestEmergencyStop:
    @pytest.mark.asyncio
    async def test_emergency_stop_from_running(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
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
    async def test_emergency_stop_calls_home_return_on_both_axes(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
        ctrl._accel_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.reset_mock()  # type: ignore[attr-defined]
        await ctrl.emergency_stop()
        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_emergency_stop_triggers_safety_monitor(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        ctrl._safety_monitor.trigger_emergency.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_emergency_stop_from_booting_raises(self) -> None:
        ctrl = make_controller()
        with pytest.raises(InvalidStateTransition):
            await ctrl.emergency_stop()

    @pytest.mark.asyncio
    async def test_emergency_stop_clears_session_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
        await ctrl.emergency_stop()
        assert ctrl.get_system_state().active_session_id is None


class TestResetEmergency:
    @pytest.mark.asyncio
    async def test_reset_emergency_transitions_to_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.emergency_stop()
        await ctrl.reset_emergency()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_reset_emergency_from_ready_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.reset_emergency()


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
        await ctrl.start_auto_drive(mode_id="mode-1")
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

        failed_result = CalibrationResult(
            success=False, data=None, error_message="スパイク未検出"
        )
        mock_manager = AsyncMock()
        mock_manager.run_calibration = AsyncMock(return_value=failed_result)
        ctrl._calibration_manager = mock_manager  # type: ignore[assignment]

        result = await ctrl.run_calibration()

        assert ctrl.get_system_state().robot_state == RobotState.READY
        assert result.success is False


class TestStartAutoDrive:
    @pytest.mark.asyncio
    async def test_start_auto_drive_transitions_to_running(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

    @pytest.mark.asyncio
    async def test_start_auto_drive_returns_session_with_mode_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        session = await ctrl.start_auto_drive(mode_id="mode-1")
        assert session.mode_id == "mode-1"
        assert session.run_type == "auto"
        assert session.status == "running"

    @pytest.mark.asyncio
    async def test_start_auto_drive_sets_active_session_id(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        session = await ctrl.start_auto_drive(mode_id="mode-1")
        assert ctrl.get_system_state().active_session_id == session.id

    @pytest.mark.asyncio
    async def test_start_auto_drive_from_running_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
        with pytest.raises(InvalidStateTransition):
            await ctrl.start_auto_drive(mode_id="mode-2")


class TestStopAutoDrive:
    @pytest.mark.asyncio
    async def test_stop_auto_drive_transitions_to_ready(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
        await ctrl.stop_auto_drive()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_stop_auto_drive_clears_session(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
        await ctrl.stop_auto_drive()
        assert ctrl.get_system_state().active_session_id is None

    @pytest.mark.asyncio
    async def test_stop_auto_drive_from_ready_raises(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        with pytest.raises(InvalidStateTransition):
            await ctrl.stop_auto_drive()


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
        await ctrl.start_auto_drive(mode_id="mode-1")
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
        await ctrl.start_auto_drive(mode_id="mode-1")
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

    @pytest.mark.asyncio
    async def test_start_auto_drive_with_passing_pre_check_transitions_to_running(self) -> None:
        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_passing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        await ctrl.start_auto_drive(mode_id="mode-1")
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

    @pytest.mark.asyncio
    async def test_start_auto_drive_with_failing_pre_check_raises_pre_check_failed(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415

        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_failing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        with pytest.raises(PreCheckFailed) as exc_info:
            await ctrl.start_auto_drive(mode_id="mode-1")
        assert exc_info.value.result is not None
        assert not exc_info.value.result.passed

    @pytest.mark.asyncio
    async def test_start_auto_drive_with_failing_pre_check_returns_to_ready(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415

        ctrl = make_controller()
        ctrl._pre_check_runner = self.make_failing_pre_check_runner()  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        with pytest.raises(PreCheckFailed):
            await ctrl.start_auto_drive(mode_id="mode-1")
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
        await ctrl.start_auto_drive(mode_id="mode-1")

        mock_loop = MagicMock()
        ctrl._drive_loop = mock_loop  # type: ignore[assignment]

        await ctrl.shutdown()

        mock_loop.stop.assert_called_once()
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


class TestStartLearningDrive:
    """start_learning_drive() の状態遷移・返り値テスト。"""

    @pytest.mark.asyncio
    async def test_transitions_to_running(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        await ctrl.start_learning_drive()
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING

    @pytest.mark.asyncio
    async def test_returns_learning_drive_session(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        session = await ctrl.start_learning_drive()
        assert session.run_type == "learning"
        assert session.mode_id is None
        assert session.status == "running"

    @pytest.mark.asyncio
    async def test_active_session_id_is_set(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        session = await ctrl.start_learning_drive()
        assert ctrl.get_system_state().active_session_id == session.id

    @pytest.mark.asyncio
    async def test_pre_check_failed_returns_to_ready(self) -> None:
        from src.app.robot_controller import PreCheckFailed  # noqa: PLC0415
        from src.models.pre_check import PreCheckItemResult, PreCheckResult  # noqa: PLC0415

        runner = MagicMock()
        runner.run = AsyncMock(
            return_value=PreCheckResult(
                passed=False,
                items=[
                    PreCheckItemResult(
                        item_name="UPS残量",
                        passed=False,
                        error_message="UPS残量不足: 5.0%",
                    )
                ],
            )
        )
        ctrl = make_controller()
        ctrl._pre_check_runner = runner  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        with pytest.raises(PreCheckFailed):
            await ctrl.start_learning_drive()
        assert ctrl.get_system_state().robot_state == RobotState.READY

    @pytest.mark.asyncio
    async def test_with_passing_pre_check_transitions_to_running(self) -> None:
        from src.models.pre_check import PreCheckItemResult, PreCheckResult  # noqa: PLC0415

        runner = MagicMock()
        runner.run = AsyncMock(
            return_value=PreCheckResult(
                passed=True,
                items=[PreCheckItemResult(item_name="通信確認", passed=True)],
            )
        )
        ctrl = make_controller()
        ctrl._pre_check_runner = runner  # type: ignore[assignment]
        await advance_to_ready(ctrl)
        await ctrl.start_learning_drive()
        assert ctrl.get_system_state().robot_state == RobotState.RUNNING
