from datetime import UTC
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
