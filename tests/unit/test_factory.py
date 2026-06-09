from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.app.factory import build_real_controller
from src.app.robot_controller import RobotController
from src.domain.safety_monitor import SafetyMonitor
from src.infra.settings import (
    AppSettings,
    CanSettings,
    DatabaseSettings,
    GpioSettings,
    SafetySettings,
    SerialSettings,
)


def make_settings(
    accel_port: str = "/dev/ttyUSB0",
    brake_port: str = "/dev/ttyUSB1",
    baud_rate: int = 38400,
    can_interface: str = "kvaser",
    can_channel: int = 0,
    can_dbc_path: str = "config/can/MEIDEN_MEIDACS.dbc",
    emergency_pin: int = 17,
    ac_detect_pin: int = 27,
    db_dsn: str = "postgresql://localhost/test",
) -> AppSettings:
    settings = AppSettings()
    settings.serial = SerialSettings(
        accel_port=accel_port,
        brake_port=brake_port,
        baud_rate=baud_rate,
    )
    settings.can = CanSettings(interface=can_interface, channel=can_channel, dbc_path=can_dbc_path)
    settings.gpio = GpioSettings(
        emergency_stop_pin=emergency_pin,
        ac_detect_pin=ac_detect_pin,
    )
    settings.database = DatabaseSettings(dsn=db_dsn)
    return settings


class TestBuildRealController:
    @pytest.mark.asyncio
    async def test_returns_robot_controller_instance(self) -> None:
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver") as mock_actuator,
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.NutUPSMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_actuator.return_value = MagicMock()
            ctrl, _ = await build_real_controller(settings)

        assert isinstance(ctrl, RobotController)

    @pytest.mark.asyncio
    async def test_accel_driver_uses_accel_port_and_slave_id_1(self) -> None:
        settings = make_settings(accel_port="/dev/ttyUSB0", baud_rate=38400)
        with (
            patch("src.app.factory.ActuatorDriver") as mock_actuator,
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            await build_real_controller(settings)

        first_call = mock_actuator.call_args_list[0]
        assert first_call.kwargs["port"] == "/dev/ttyUSB0"
        assert first_call.kwargs["slave_id"] == 1
        assert first_call.kwargs["baud_rate"] == 38400

    @pytest.mark.asyncio
    async def test_brake_driver_uses_brake_port_and_slave_id_1(self) -> None:
        settings = make_settings(brake_port="/dev/ttyUSB1", baud_rate=38400)
        with (
            patch("src.app.factory.ActuatorDriver") as mock_actuator,
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            await build_real_controller(settings)

        second_call = mock_actuator.call_args_list[1]
        assert second_call.kwargs["port"] == "/dev/ttyUSB1"
        assert second_call.kwargs["slave_id"] == 1  # 各軸が独立した RS-485 バスのため両軸とも 1
        assert second_call.kwargs["baud_rate"] == 38400

    @pytest.mark.asyncio
    async def test_can_reader_uses_correct_interface_and_channel(self) -> None:
        settings = make_settings(can_interface="kvaser", can_channel=0)
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader") as mock_can,
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            await build_real_controller(settings)

        mock_can.assert_called_once_with(
            interface="kvaser",
            channel=0,
            bitrate=500000,
            dbc_path="config/can/MEIDEN_MEIDACS.dbc",
        )

    @pytest.mark.asyncio
    async def test_gpio_monitor_uses_correct_pins(self) -> None:
        settings = make_settings(emergency_pin=17, ac_detect_pin=27)
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            await build_real_controller(settings)

        mock_gpio.assert_called_once_with(emergency_pin=17, ac_detect_pin=27)

    @pytest.mark.asyncio
    async def test_gpio_emergency_callback_registered_to_controller(self) -> None:
        """GPIO非常停止コールバックが controller.emergency_stop に直接配線されること。
        trigger_emergency 経由にすると循環呼び出しが生じるため、コントローラを直接登録する。
        """
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio_cls,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.NutUPSMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio = MagicMock()
            mock_gpio_cls.return_value = mock_gpio
            ctrl, _ = await build_real_controller(settings)

        mock_gpio.register_emergency_callback.assert_called_once_with(
            ctrl.emergency_stop
        )

    @pytest.mark.asyncio
    async def test_nut_ac_loss_callback_registered_to_safety_monitor(self) -> None:
        """NutUPSMonitor の AC断コールバックが SafetyMonitor.handle_ac_power_loss に配線されること。
        APC Smart-UPS 750 は NC/NO 接点なし → GPIO ではなく NUT ポーリングで AC断を検知する。
        """
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio_cls,
            patch("src.app.factory.SafetyMonitor") as mock_monitor_cls,
            patch("src.app.factory.NutUPSMonitor") as mock_ups_cls,
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio_cls.return_value = MagicMock()
            mock_monitor = MagicMock()
            mock_monitor_cls.return_value = mock_monitor
            mock_ups_monitor = MagicMock()
            mock_ups_cls.return_value = mock_ups_monitor
            await build_real_controller(settings)

        mock_ups_monitor.register_ac_loss_callback.assert_called_once_with(
            mock_monitor.handle_ac_power_loss
        )

    @pytest.mark.asyncio
    @pytest.mark.parametrize(
        ("accel_port", "brake_port", "baud_rate"),
        [
            ("/dev/ttyUSB0", "/dev/ttyUSB1", 38400),
            ("/dev/ttyACM0", "/dev/ttyACM1", 9600),
        ],
    )
    async def test_baud_rate_propagated_to_both_drivers(
        self, accel_port: str, brake_port: str, baud_rate: int
    ) -> None:
        settings = make_settings(
            accel_port=accel_port, brake_port=brake_port, baud_rate=baud_rate
        )
        with (
            patch("src.app.factory.ActuatorDriver") as mock_actuator,
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            await build_real_controller(settings)

        for call in mock_actuator.call_args_list:
            assert call.kwargs["baud_rate"] == baud_rate

    @pytest.mark.asyncio
    async def test_build_real_controller_creates_pool_with_dsn(self) -> None:
        """create_pool が settings.database.dsn で呼ばれること。"""
        settings = make_settings(db_dsn="postgresql://localhost/driving_robot")
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock) as mock_create_pool,
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            await build_real_controller(settings)

        mock_create_pool.assert_awaited_once_with("postgresql://localhost/driving_robot")

    @pytest.mark.asyncio
    async def test_profile_repository_injected_into_calibration_manager(self) -> None:
        """ProfileRepository が CalibrationManager に渡されること。"""
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock) as mock_create_pool,
            patch("src.app.factory.ProfileRepository") as mock_repo_cls,
            patch("src.app.factory.CalibrationManager") as mock_calib_cls,
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            mock_pool = MagicMock()
            mock_create_pool.return_value = mock_pool
            mock_repo = MagicMock()
            mock_repo_cls.return_value = mock_repo

            await build_real_controller(settings)

        mock_repo_cls.assert_called_once_with(mock_pool)
        calib_kwargs = mock_calib_cls.call_args.kwargs
        assert calib_kwargs["profile_repo"] is mock_repo

    @pytest.mark.asyncio
    async def test_robot_controller_has_safety_check(self) -> None:
        """RobotController に safety_check（SafetyMonitor）が渡されていること。"""
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.NutUPSMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            ctrl, _ = await build_real_controller(settings)

        assert ctrl._safety_check is not None
        assert isinstance(ctrl._safety_check, SafetyMonitor)

    @pytest.mark.asyncio
    async def test_bench_gpio_only_stubs_actuators_and_can_but_keeps_real_gpio(self) -> None:
        """bench_gpio_only=True ではアクチュエータ/CAN を生成せず、GPIO は実機のまま。"""
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver") as mock_actuator,
            patch("src.app.factory.CANReader") as mock_can,
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.NutUPSMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            ctrl, _ = await build_real_controller(settings, bench_gpio_only=True)

        assert isinstance(ctrl, RobotController)
        # 実アクチュエータ/CAN は生成されない（スタブに置換）
        mock_actuator.assert_not_called()
        mock_can.assert_not_called()
        # 非常停止スイッチ(GPIO)は実機のまま生成・配線される
        mock_gpio.assert_called_once_with(emergency_pin=17, ac_detect_pin=27)
        mock_gpio.return_value.register_emergency_callback.assert_called_once_with(
            ctrl.emergency_stop
        )

    @pytest.mark.asyncio
    async def test_safety_monitor_uses_overcurrent_limit_from_settings(self) -> None:
        """SafetyMonitor が settings.safety.overcurrent_limit_ma を使うこと。"""
        settings = make_settings()
        settings.safety = SafetySettings(overcurrent_limit_ma=1500.0)
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.NutUPSMonitor"),
            patch("src.app.factory.create_pool", new_callable=AsyncMock),
            patch("src.app.factory.ProfileRepository"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            ctrl, _ = await build_real_controller(settings)

        assert isinstance(ctrl._safety_check, SafetyMonitor)
        assert ctrl._safety_check._overcurrent_limit_ma == 1500.0
