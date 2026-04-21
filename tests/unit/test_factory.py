from unittest.mock import MagicMock, patch

import pytest

from src.app.factory import build_real_controller
from src.app.robot_controller import RobotController
from src.infra.settings import AppSettings, CanSettings, GpioSettings, SerialSettings


def make_settings(
    accel_port: str = "/dev/ttyUSB0",
    brake_port: str = "/dev/ttyUSB1",
    baud_rate: int = 38400,
    can_interface: str = "kvaser",
    can_channel: int = 0,
    emergency_pin: int = 17,
    ac_detect_pin: int = 27,
) -> AppSettings:
    settings = AppSettings()
    settings.serial = SerialSettings(
        accel_port=accel_port,
        brake_port=brake_port,
        baud_rate=baud_rate,
    )
    settings.can = CanSettings(interface=can_interface, channel=can_channel)
    settings.gpio = GpioSettings(
        emergency_stop_pin=emergency_pin,
        ac_detect_pin=ac_detect_pin,
    )
    return settings


class TestBuildRealController:
    def test_returns_robot_controller_instance(self) -> None:
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver") as mock_actuator,
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            mock_actuator.return_value = MagicMock()
            ctrl = build_real_controller(settings)

        assert isinstance(ctrl, RobotController)

    def test_accel_driver_uses_accel_port_and_slave_id_1(self) -> None:
        settings = make_settings(accel_port="/dev/ttyUSB0", baud_rate=38400)
        with (
            patch("src.app.factory.ActuatorDriver") as mock_actuator,
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            build_real_controller(settings)

        first_call = mock_actuator.call_args_list[0]
        assert first_call.kwargs["port"] == "/dev/ttyUSB0"
        assert first_call.kwargs["slave_id"] == 1
        assert first_call.kwargs["baud_rate"] == 38400

    def test_brake_driver_uses_brake_port_and_slave_id_2(self) -> None:
        settings = make_settings(brake_port="/dev/ttyUSB1", baud_rate=38400)
        with (
            patch("src.app.factory.ActuatorDriver") as mock_actuator,
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            build_real_controller(settings)

        second_call = mock_actuator.call_args_list[1]
        assert second_call.kwargs["port"] == "/dev/ttyUSB1"
        assert second_call.kwargs["slave_id"] == 2
        assert second_call.kwargs["baud_rate"] == 38400

    def test_can_reader_uses_correct_interface_and_channel(self) -> None:
        settings = make_settings(can_interface="kvaser", can_channel=0)
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader") as mock_can,
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            build_real_controller(settings)

        mock_can.assert_called_once_with(interface="kvaser", channel=0)

    def test_gpio_monitor_uses_correct_pins(self) -> None:
        settings = make_settings(emergency_pin=17, ac_detect_pin=27)
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio,
            patch("src.app.factory.SafetyMonitor"),
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            build_real_controller(settings)

        mock_gpio.assert_called_once_with(emergency_pin=17, ac_detect_pin=27)

    def test_gpio_emergency_callback_registered_to_safety_monitor(self) -> None:
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio_cls,
            patch("src.app.factory.SafetyMonitor") as mock_monitor_cls,
        ):
            mock_gpio = MagicMock()
            mock_gpio_cls.return_value = mock_gpio
            mock_monitor = MagicMock()
            mock_monitor_cls.return_value = mock_monitor
            build_real_controller(settings)

        mock_gpio.register_emergency_callback.assert_called_once_with(
            mock_monitor.trigger_emergency
        )

    def test_gpio_ac_loss_callback_registered_to_safety_monitor(self) -> None:
        settings = make_settings()
        with (
            patch("src.app.factory.ActuatorDriver"),
            patch("src.app.factory.CANReader"),
            patch("src.app.factory.GPIOMonitor") as mock_gpio_cls,
            patch("src.app.factory.SafetyMonitor") as mock_monitor_cls,
        ):
            mock_gpio = MagicMock()
            mock_gpio_cls.return_value = mock_gpio
            mock_monitor = MagicMock()
            mock_monitor_cls.return_value = mock_monitor
            build_real_controller(settings)

        mock_gpio.register_ac_loss_callback.assert_called_once_with(
            mock_monitor.handle_ac_power_loss
        )

    @pytest.mark.parametrize(
        ("accel_port", "brake_port", "baud_rate"),
        [
            ("/dev/ttyUSB0", "/dev/ttyUSB1", 38400),
            ("/dev/ttyACM0", "/dev/ttyACM1", 9600),
        ],
    )
    def test_baud_rate_propagated_to_both_drivers(
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
        ):
            mock_gpio.return_value.register_emergency_callback = MagicMock()
            mock_gpio.return_value.register_ac_loss_callback = MagicMock()
            build_real_controller(settings)

        for call in mock_actuator.call_args_list:
            assert call.kwargs["baud_rate"] == baud_rate
