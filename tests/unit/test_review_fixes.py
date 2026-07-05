"""コードレビュー修正（.steering/20260610-fix-code-review-findings）の回帰テスト。

非常停止ディスパッチの一元化・初期化失敗からの復旧・シャットダウン時のペダル解放・
走行開始中の非常停止競合・プロファイル制御パラメータの反映を検証する。
"""

from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest

from src.app.robot_controller import InvalidStateTransition, RobotController
from src.domain.control.pid import PIDController
from src.domain.safety_monitor import SafetyMonitor
from src.models.profile import PIDGains, StopConfig, VehicleProfile
from src.models.system_state import RobotState
from tests.unit.test_robot_controller import (
    advance_to_ready,
    make_accel_driver,
    make_brake_driver,
    make_can_reader,
    make_controller,
    make_safety_monitor,
    make_stop_config,
)


def _make_profile(profile_id: str = "profile-1") -> VehicleProfile:
    return VehicleProfile(
        id=profile_id,
        name="TestCar",
        max_accel_opening=80.0,
        max_brake_opening=60.0,
        max_speed=180.0,
        max_decel_g=0.5,
        pid_gains=PIDGains(kp=2.5, ki=0.3, kd=0.1),
        stop_config=StopConfig(deviation_threshold_kmh=5.0, deviation_duration_s=3.0),
        calibration=None,
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )


class TestEmergencyDispatch:
    """SafetyMonitor を単一ディスパッチャとする非常停止配線（C1 修正）。"""

    def _make_controller_with_real_monitor(self) -> tuple[RobotController, SafetyMonitor]:
        monitor = SafetyMonitor(stop_config=make_stop_config())
        ctrl = RobotController(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            safety_monitor=monitor,  # type: ignore[arg-type]
            pid=PIDController(kp=1.0, ki=0.0, kd=0.0),
            last_normal_shutdown=True,
        )
        # factory.build_real_controller と同じ配線
        monitor.register_emergency_callback(ctrl.emergency_stop)
        return ctrl, monitor

    @pytest.mark.asyncio
    async def test_ac_power_loss_stops_the_robot(self) -> None:
        """AC断 → handle_ac_power_loss → trigger_emergency → emergency_stop が走ること。"""
        ctrl, monitor = self._make_controller_with_real_monitor()
        await advance_to_ready(ctrl)

        await monitor.handle_ac_power_loss()

        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY
        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_trigger_emergency_does_not_recurse(self) -> None:
        """emergency_stop は trigger_emergency を呼び返さないため再帰しないこと。"""
        ctrl, monitor = self._make_controller_with_real_monitor()
        await advance_to_ready(ctrl)

        # 再帰があればここで RecursionError / 多重 home_return になる
        await monitor.trigger_emergency()
        await monitor.trigger_emergency()

        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY
        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_emergency_from_error_state_is_allowed(self) -> None:
        """ERROR 状態でも物理非常停止が InvalidStateTransition にならないこと（C10 修正）。"""
        ctrl = make_controller(last_normal_shutdown=False)
        await ctrl.start()
        ctrl._brake_driver.enable_modbus_control = AsyncMock(side_effect=TimeoutError("modbus"))
        with pytest.raises(TimeoutError):
            await ctrl.initialize()
        assert ctrl.get_system_state().robot_state == RobotState.ERROR

        await ctrl.emergency_stop()
        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY


class TestInitializeFailureRecovery:
    """初期化失敗時の ERROR 遷移と再試行（C9 修正）。"""

    @pytest.mark.asyncio
    async def test_initialize_failure_transitions_to_error(self) -> None:
        ctrl = make_controller(last_normal_shutdown=False)
        await ctrl.start()
        ctrl._brake_driver.enable_modbus_control = AsyncMock(side_effect=TimeoutError("modbus"))

        with pytest.raises(TimeoutError):
            await ctrl.initialize()

        assert ctrl.get_system_state().robot_state == RobotState.ERROR

    @pytest.mark.asyncio
    async def test_initialize_retry_from_error_recovers_to_ready(self) -> None:
        """ERROR からの initialize 再呼び出し（再試行ボタン）で READY へ復帰できること。"""
        ctrl = make_controller(last_normal_shutdown=False)
        await ctrl.start()
        ctrl._brake_driver.enable_modbus_control = AsyncMock(
            side_effect=[TimeoutError("transient"), None]
        )

        with pytest.raises(TimeoutError):
            await ctrl.initialize()
        await ctrl.initialize()

        assert ctrl.get_system_state().robot_state == RobotState.READY


class TestShutdownNeutralizesActuators:
    """shutdown() でのペダル解放とサーボOFF（C2 修正）。"""

    @pytest.mark.asyncio
    async def test_shutdown_from_ready_releases_pedals_and_servo_off(self) -> None:
        ctrl = make_controller()
        await advance_to_ready(ctrl)

        await ctrl.shutdown()

        ctrl._accel_driver.home_return.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.home_return.assert_called_once()  # type: ignore[attr-defined]
        ctrl._accel_driver.servo_off.assert_called_once()  # type: ignore[attr-defined]
        ctrl._brake_driver.servo_off.assert_called_once()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_shutdown_from_standby_does_not_touch_actuators(self) -> None:
        """サーボON前（STANDBY）はハード操作しない。"""
        ctrl = make_controller()
        await ctrl.start()

        await ctrl.shutdown()

        ctrl._accel_driver.home_return.assert_not_called()  # type: ignore[attr-defined]
        ctrl._accel_driver.servo_off.assert_not_called()  # type: ignore[attr-defined]

    @pytest.mark.asyncio
    async def test_shutdown_continues_when_hardware_fails(self) -> None:
        """ペダル解放に失敗してもシャットダウンは完走する（監視停止まで到達）。"""
        ctrl = make_controller()
        await advance_to_ready(ctrl)
        ctrl._accel_driver.home_return = AsyncMock(side_effect=ConnectionError("bus down"))

        await ctrl.shutdown()  # 例外が伝播しないこと

        ctrl._safety_monitor.stop_monitoring.assert_called_once()  # type: ignore[attr-defined]


class TestStartDriveEmergencyRace:
    """走行開始処理中の非常停止競合（C5 修正）。"""

    @pytest.mark.asyncio
    async def test_emergency_during_session_open_aborts_drive_start(self) -> None:
        """_open_session の await 中に非常停止 → DriveLoop を起動せず中断すること。"""
        ctrl = make_controller()
        await ctrl.start()
        ctrl.select_profile(_make_profile())
        await ctrl.initialize()

        log_writer = MagicMock()

        async def start_session_with_estop(
            profile_id: str, mode_id: str | None, run_type: str, cycle_id: str | None = None
        ) -> str:
            # DB INSERT の await 中に GPIO 非常停止が割り込んだ状況を模擬
            await ctrl.emergency_stop()
            return "session-x"

        log_writer.start_session = AsyncMock(side_effect=start_session_with_estop)
        log_writer.end_session = AsyncMock()

        with pytest.raises(InvalidStateTransition):
            await ctrl.start_auto_drive("mode-1", log_writer=log_writer)

        assert ctrl.get_system_state().robot_state == RobotState.EMERGENCY
        assert ctrl._drive_loop is None
        # 開いてしまったセッションは emergency で閉じる
        log_writer.end_session.assert_awaited_once_with("session-x", "emergency")


class TestSelectProfileAppliesControlParams:
    """プロファイルの pid_gains / stop_config が制御スタックへ反映される（C15 修正）。"""

    @pytest.mark.asyncio
    async def test_select_profile_applies_pid_gains(self) -> None:
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        ctrl = RobotController(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            safety_monitor=make_safety_monitor(),
            pid=pid,
            last_normal_shutdown=True,
        )
        await ctrl.start()

        ctrl.select_profile(_make_profile())

        assert pid._kp == 2.5
        assert pid._ki == 0.3
        assert pid._kd == 0.1

    @pytest.mark.asyncio
    async def test_select_profile_applies_stop_config_to_safety_check(self) -> None:
        safety_check = MagicMock()
        ctrl = RobotController(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            safety_monitor=make_safety_monitor(),
            pid=PIDController(kp=1.0, ki=0.0, kd=0.0),
            safety_check=safety_check,
            last_normal_shutdown=True,
        )
        await ctrl.start()
        profile = _make_profile()

        ctrl.select_profile(profile)

        safety_check.set_stop_config.assert_called_once_with(profile.stop_config)
