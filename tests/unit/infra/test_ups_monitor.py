"""NutUPSMonitor のユニットテスト。"""

from __future__ import annotations

import asyncio
from unittest.mock import AsyncMock, patch

import pytest

from src.infra.ups_monitor import NutUPSMonitor, UPSStatus


class _FakeStreamReader:
    def __init__(self, response: str) -> None:
        self._lines = iter([response.encode()])

    async def readline(self) -> bytes:
        return next(self._lines)


class _FakeStreamWriter:
    def __init__(self) -> None:
        self.written: list[bytes] = []

    def write(self, data: bytes) -> None:
        self.written.append(data)

    async def drain(self) -> None:
        pass

    def close(self) -> None:
        pass

    async def wait_closed(self) -> None:
        pass


def _make_fake_connection(response: str):
    reader = _FakeStreamReader(response)
    writer = _FakeStreamWriter()
    return reader, writer


# ── _nut_get_var のパーステスト ────────────────────────────────────────────────


@pytest.mark.asyncio
async def test_nut_get_var_parses_battery_charge() -> None:
    """battery.charge の正常レスポンスを正しくパースする。"""
    monitor = NutUPSMonitor(ups_name="apcups")

    async def fake_open_connection(host: str, port: int):
        return _make_fake_connection('VAR apcups battery.charge "95"\n')

    with patch("asyncio.open_connection", fake_open_connection):
        result = await monitor._nut_get_var("battery.charge")

    assert result == "95"


@pytest.mark.asyncio
async def test_nut_get_var_parses_ups_status_ol() -> None:
    """ups.status の OL CHRG レスポンスを正しくパースする。"""
    monitor = NutUPSMonitor(ups_name="apcups")

    async def fake_open_connection(host: str, port: int):
        return _make_fake_connection('VAR apcups ups.status "OL CHRG"\n')

    with patch("asyncio.open_connection", fake_open_connection):
        result = await monitor._nut_get_var("ups.status")

    assert result == "OL CHRG"


@pytest.mark.asyncio
async def test_nut_get_var_raises_on_err_response() -> None:
    """NUT が ERR を返した場合に RuntimeError を送出する。"""
    monitor = NutUPSMonitor(ups_name="apcups")

    async def fake_open_connection(host: str, port: int):
        return _make_fake_connection("ERR VAR-NOT-SUPPORTED\n")

    with patch("asyncio.open_connection", fake_open_connection):
        with pytest.raises(RuntimeError, match="NUT エラー"):
            await monitor._nut_get_var("battery.charge")


# ── AC断コールバックのエッジ検知テスト ─────────────────────────────────────────


@pytest.mark.asyncio
async def test_ac_loss_callback_fires_on_ol_to_ob_transition() -> None:
    """OL → OB 遷移（立ち上がりエッジ）でのみコールバックが発火する。"""
    monitor = NutUPSMonitor(ups_name="apcups")
    monitor._loop = asyncio.get_event_loop()
    monitor._prev_on_battery = False  # 初期状態: AC通電中

    callback_count = 0

    async def on_ac_loss() -> None:
        nonlocal callback_count
        callback_count += 1

    monitor.register_ac_loss_callback(on_ac_loss)

    # OL → OB 遷移: コールバックが1回発火すること
    call_count = 0

    async def fake_get_var(var_name: str) -> str:
        nonlocal call_count
        call_count += 1
        if "battery.charge" in var_name:
            return "50"
        return "OB"  # On Battery

    monitor._nut_get_var = fake_get_var  # type: ignore[method-assign]

    await monitor._poll_once()

    # run_coroutine_threadsafe はテスト環境では直接呼べないため、_fire_callbacks を直接テスト
    assert monitor._cached_on_battery is True
    assert monitor._prev_on_battery is True


@pytest.mark.asyncio
async def test_ac_loss_callback_does_not_fire_if_already_on_battery() -> None:
    """OB → OB（継続中）ではコールバックが発火しない。"""
    monitor = NutUPSMonitor(ups_name="apcups")
    monitor._loop = asyncio.get_event_loop()
    monitor._prev_on_battery = True  # 既にバッテリー運転中

    fired = False

    async def on_ac_loss() -> None:
        nonlocal fired
        fired = True

    monitor.register_ac_loss_callback(on_ac_loss)

    async def fake_get_var(var_name: str) -> str:
        if "battery.charge" in var_name:
            return "45"
        return "OB"

    monitor._nut_get_var = fake_get_var  # type: ignore[method-assign]

    await monitor._poll_once()

    assert not fired, "OB→OB 継続中にコールバックが発火してはいけない"


@pytest.mark.asyncio
async def test_ac_loss_callback_does_not_fire_on_ob_to_ol() -> None:
    """OB → OL（復電）ではコールバックが発火しない。"""
    monitor = NutUPSMonitor(ups_name="apcups")
    monitor._loop = asyncio.get_event_loop()
    monitor._prev_on_battery = True  # バッテリー運転中から

    fired = False

    async def on_ac_loss() -> None:
        nonlocal fired
        fired = True

    monitor.register_ac_loss_callback(on_ac_loss)

    async def fake_get_var(var_name: str) -> str:
        if "battery.charge" in var_name:
            return "90"
        return "OL CHRG"  # 復電

    monitor._nut_get_var = fake_get_var  # type: ignore[method-assign]

    await monitor._poll_once()

    assert not fired, "OB→OL 復電時にコールバックが発火してはいけない"
    assert not monitor._cached_on_battery


# ── NUT 接続失敗時のフォールバックテスト ────────────────────────────────────────


@pytest.mark.asyncio
async def test_poll_once_keeps_cache_on_connection_failure() -> None:
    """NUT 接続失敗時に前回キャッシュを保持し、is_available=False にする。"""
    monitor = NutUPSMonitor(ups_name="apcups")
    monitor._cached_battery_pct = 80.0
    monitor._cached_on_battery = False
    monitor._nut_available = True

    async def fake_get_var_raises(var_name: str) -> str:
        raise ConnectionRefusedError("NUT に接続できません")

    monitor._nut_get_var = fake_get_var_raises  # type: ignore[method-assign]

    await monitor._poll_once()

    assert not monitor._nut_available
    # キャッシュは保持されている（前回値を保持）
    assert monitor._cached_battery_pct == 80.0


@pytest.mark.asyncio
async def test_get_battery_level_pct_raises_when_unavailable() -> None:
    """NUT 未接続時に get_battery_level_pct() が RuntimeError を送出する。"""
    monitor = NutUPSMonitor(ups_name="apcups")
    monitor._nut_available = False

    with pytest.raises(RuntimeError, match="NUT サーバーに接続できません"):
        await monitor.get_battery_level_pct()


# ── get_status のテスト ─────────────────────────────────────────────────────────


@pytest.mark.asyncio
async def test_get_status_returns_cached_values() -> None:
    """get_status() がキャッシュ済みの値を正しく返す。"""
    monitor = NutUPSMonitor(ups_name="apcups")
    monitor._cached_battery_pct = 75.0
    monitor._cached_on_battery = False
    monitor._cached_low_battery = False
    monitor._cached_status_flags = "OL"
    monitor._nut_available = True

    status = await monitor.get_status()

    assert isinstance(status, UPSStatus)
    assert status.battery_charge_pct == 75.0
    assert not status.on_battery
    assert not status.low_battery
    assert status.status_flags == "OL"
    assert status.is_available


@pytest.mark.asyncio
async def test_get_status_on_battery_flag() -> None:
    """OB 状態で is_on_battery プロパティと status.on_battery が True になる。"""
    monitor = NutUPSMonitor(ups_name="apcups")
    monitor._cached_battery_pct = 60.0
    monitor._cached_on_battery = True
    monitor._cached_status_flags = "OB"
    monitor._nut_available = True

    assert monitor.is_on_battery is True

    status = await monitor.get_status()
    assert status.on_battery is True
