"""GPIOMonitor のユニットテスト（RPi.GPIO はモック化）。"""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.infra.gpio_monitor import GPIOMonitor


def _make_gpio_mock() -> MagicMock:
    gpio = MagicMock()
    gpio.BCM = "BCM"
    gpio.IN = "IN"
    gpio.FALLING = "FALLING"
    gpio.PUD_UP = "PUD_UP"
    return gpio


def _make_sys_modules_patch(mock_gpio: MagicMock) -> dict[str, object]:
    """RPi と RPi.GPIO の sys.modules パッチ用辞書を作成する。

    import RPi.GPIO as GPIO は IMPORT_FROM 経由で mock_rpi.GPIO を参照するため、
    mock_rpi.GPIO を mock_gpio に明示的にバインドする必要がある。
    """
    mock_rpi = MagicMock()
    mock_rpi.GPIO = mock_gpio
    return {"RPi": mock_rpi, "RPi.GPIO": mock_gpio}


class TestStartMonitoring:
    @pytest.mark.asyncio
    async def test_setup_called_for_both_pins(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_gpio = _make_gpio_mock()

        with patch.dict("sys.modules", _make_sys_modules_patch(mock_gpio)):
            with patch("asyncio.get_event_loop", return_value=asyncio.get_event_loop()):
                await monitor.start_monitoring()

        mock_gpio.setmode.assert_called_once_with(mock_gpio.BCM)
        setup_calls = mock_gpio.setup.call_args_list
        pins = [c.args[0] for c in setup_calls]
        assert 17 in pins
        assert 27 in pins

    @pytest.mark.asyncio
    async def test_event_detect_registered_for_both_pins(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_gpio = _make_gpio_mock()

        with patch.dict("sys.modules", _make_sys_modules_patch(mock_gpio)):
            with patch("asyncio.get_event_loop", return_value=asyncio.get_event_loop()):
                await monitor.start_monitoring()

        detect_calls = mock_gpio.add_event_detect.call_args_list
        detected_pins = [c.args[0] for c in detect_calls]
        assert 17 in detected_pins
        assert 27 in detected_pins


class TestCallbacks:
    def test_register_and_fire_emergency_callback(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_loop = MagicMock()
        monitor._loop = mock_loop

        cb = AsyncMock()
        monitor.register_emergency_callback(cb)
        monitor._on_emergency(channel=17)

        mock_loop.is_running.assert_called()

    def test_register_and_fire_ac_loss_callback(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_loop = MagicMock()
        monitor._loop = mock_loop

        cb = AsyncMock()
        monitor.register_ac_loss_callback(cb)
        monitor._on_ac_loss(channel=27)

        mock_loop.is_running.assert_called()

    def test_fire_without_loop_does_not_raise(self) -> None:
        monitor = GPIOMonitor()
        monitor._loop = None

        cb = AsyncMock()
        monitor.register_emergency_callback(cb)
        monitor._on_emergency(channel=17)  # ループが None でもクラッシュしない

    @pytest.mark.asyncio
    async def test_callback_invoked_via_run_coroutine_threadsafe(self) -> None:
        loop = asyncio.get_event_loop()
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        monitor._loop = loop

        called = asyncio.Event()

        async def cb() -> None:
            called.set()

        monitor.register_emergency_callback(cb)

        with patch("asyncio.run_coroutine_threadsafe") as mock_threadsafe:
            monitor._on_emergency(channel=17)
            mock_threadsafe.assert_called_once()


class TestStopMonitoring:
    def test_cleanup_called(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_gpio = _make_gpio_mock()

        with patch.dict("sys.modules", _make_sys_modules_patch(mock_gpio)):
            monitor.stop_monitoring()

        mock_gpio.remove_event_detect.assert_any_call(17)
        mock_gpio.remove_event_detect.assert_any_call(27)
        mock_gpio.cleanup.assert_called_once_with([17, 27])
