"""GPIOMonitor のユニットテスト（lgpio はモック化）。"""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.infra.gpio_monitor import GPIOMonitor


def _make_lgpio_mock() -> MagicMock:
    lgpio = MagicMock()
    lgpio.RISING_EDGE = 1
    lgpio.FALLING_EDGE = 2
    lgpio.SET_PULL_UP = 1
    lgpio.gpiochip_open.return_value = 0
    return lgpio


class TestStartMonitoring:
    @pytest.mark.asyncio
    async def test_setup_called_for_both_pins(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_lgpio = _make_lgpio_mock()

        with patch.dict("sys.modules", {"lgpio": mock_lgpio}):
            with patch("asyncio.get_event_loop", return_value=asyncio.get_event_loop()):
                await monitor.start_monitoring()

        claim_calls = mock_lgpio.gpio_claim_alert.call_args_list
        claimed_pins = [c.args[1] for c in claim_calls]
        assert 17 in claimed_pins
        assert 27 in claimed_pins

    @pytest.mark.asyncio
    async def test_event_detect_registered_for_both_pins(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_lgpio = _make_lgpio_mock()

        with patch.dict("sys.modules", {"lgpio": mock_lgpio}):
            with patch("asyncio.get_event_loop", return_value=asyncio.get_event_loop()):
                await monitor.start_monitoring()

        cb_calls = mock_lgpio.callback.call_args_list
        cb_pins = [c.args[1] for c in cb_calls]
        assert 17 in cb_pins
        assert 27 in cb_pins


class TestCallbacks:
    def test_register_and_fire_emergency_callback(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_loop = MagicMock()
        monitor._loop = mock_loop

        cb = AsyncMock()
        monitor.register_emergency_callback(cb)
        monitor._on_emergency(0, 17, 1, 0)

        mock_loop.is_running.assert_called()

    def test_register_and_fire_ac_loss_callback(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_loop = MagicMock()
        monitor._loop = mock_loop

        cb = AsyncMock()
        monitor.register_ac_loss_callback(cb)
        monitor._on_ac_loss(0, 27, 0, 0)

        mock_loop.is_running.assert_called()

    def test_fire_without_loop_does_not_raise(self) -> None:
        monitor = GPIOMonitor()
        monitor._loop = None

        cb = AsyncMock()
        monitor.register_emergency_callback(cb)
        monitor._on_emergency(0, 17, 1, 0)  # ループが None でもクラッシュしない

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
            monitor._on_emergency(0, 17, 1, 0)
            mock_threadsafe.assert_called_once()


class TestStopMonitoring:
    def test_cleanup_called(self) -> None:
        monitor = GPIOMonitor(emergency_pin=17, ac_detect_pin=27)
        mock_lgpio = _make_lgpio_mock()
        mock_cb = MagicMock()
        monitor._handle = 0
        monitor._cb_emergency = mock_cb  # type: ignore[assignment]
        monitor._cb_ac = mock_cb  # type: ignore[assignment]

        with patch.dict("sys.modules", {"lgpio": mock_lgpio}):
            monitor.stop_monitoring()

        assert mock_cb.cancel.call_count == 2
        mock_lgpio.gpiochip_close.assert_called_once_with(0)
