"""ボタンサーボ（button_servo.py）と hardware 組み込みのテスト（2026-11-12 段1）。"""

from __future__ import annotations

import asyncio

import pytest

from tests.research import button_servo as bs
from tests.research import config as cfgmod
from tests.research import hardware as hwmod


class FakeBus:
    """I2C バスの代わり。write_i2c_block_data の履歴（チャンネル→最後のカウント）を残す。"""

    def __init__(self) -> None:
        self.counts: dict[int, int] = {}
        self.writes: list[tuple[int, int]] = []  # (channel, count)
        self.closed = False
        self.fail_release = False

    def write_byte_data(self, addr: int, reg: int, value: int) -> None:
        pass

    def read_byte_data(self, addr: int, reg: int) -> int:
        return 0

    def write_i2c_block_data(self, addr: int, reg: int, data: list[int]) -> None:
        ch = (reg - 0x06) // 4
        count = data[2] | (data[3] << 8)
        self.writes.append((ch, count))
        self.counts[ch] = count

    def close(self) -> None:
        self.closed = True


def _driver(bus: FakeBus) -> bs.ButtonServoDriver:
    return bs.ButtonServoDriver(rest_angle=60.0, press_angle=110.0, bus_factory=lambda _n: bus)


REST = bs.angle_to_count(60.0, 50)
PRESS = bs.angle_to_count(110.0, 50)


def test_angle_to_count_known_values_and_clamp() -> None:
    # 0° = 0.5ms / 20ms * 4096 = 102、180° = 2.5ms → 512
    assert bs.angle_to_count(0.0, 50) == 102
    assert bs.angle_to_count(180.0, 50) == 512
    assert bs.angle_to_count(-10.0, 50) == 102  # 範囲外は端に丸める
    assert bs.angle_to_count(999.0, 50) == 512
    assert bs.angle_to_count(60.0, 50) < bs.angle_to_count(110.0, 50)


def test_channel_constants_and_names() -> None:
    assert (bs.CH_START, bs.CH_P, bs.CH_N, bs.CH_D) == (0, 1, 2, 3)
    assert bs.button_name(bs.CH_N) == "N"
    assert bs.button_name(9) == "ch9"


async def test_connect_sets_all_channels_to_rest() -> None:
    bus = FakeBus()
    drv = _driver(bus)
    await drv.connect()
    assert all(bus.counts[ch] == REST for ch in range(16))


async def test_press_ends_at_rest_angle() -> None:
    bus = FakeBus()
    drv = _driver(bus)
    await drv.connect()
    bus.writes.clear()
    await drv.press(bs.CH_N, 0.01)
    assert bus.writes == [(bs.CH_N, PRESS), (bs.CH_N, REST)]
    assert bus.counts[bs.CH_N] == REST


async def test_press_cancel_still_returns_to_rest() -> None:
    bus = FakeBus()
    drv = _driver(bus)
    await drv.connect()
    task = asyncio.create_task(drv.press(bs.CH_START, 5.0))
    await asyncio.sleep(0.1)  # 押下角へ動いて待機に入るまで
    assert bus.counts[bs.CH_START] == PRESS
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    await asyncio.sleep(0.05)  # 解放タスクの完了待ち
    assert bus.counts[bs.CH_START] == REST


async def test_press_rejects_bad_channel_and_unconnected() -> None:
    drv = _driver(FakeBus())
    with pytest.raises(RuntimeError, match="connect"):
        await drv.press(0, 0.0)
    await drv.connect()
    with pytest.raises(ValueError):
        await drv.press(16, 0.0)


async def test_release_all_and_close() -> None:
    bus = FakeBus()
    drv = _driver(bus)
    await drv.release_all()  # 未接続は no-op
    await drv.connect()
    bus.counts[2] = PRESS
    await drv.release_all()
    assert bus.counts[2] == REST
    await drv.close()
    assert bus.closed


class _Listener:
    def __init__(self) -> None:
        self.events: list[tuple[str, int]] = []

    def on_press_start(self, channel: int, now: float) -> None:
        self.events.append(("start", channel))

    def on_press_end(self, channel: int, now: float) -> None:
        self.events.append(("end", channel))


async def test_stub_records_history_and_notifies_listener() -> None:
    listener = _Listener()
    stub = bs.StubButtonServo(listener=listener)
    with pytest.raises(RuntimeError):
        await stub.press(bs.CH_N, 0.0)
    await stub.connect()
    await stub.press(bs.CH_N, 0.02)
    await stub.press(bs.CH_START, 0.0)
    assert [(r.channel, r.duration_s) for r in stub.history] == [(2, 0.02), (0, 0.0)]
    assert listener.events == [("start", 2), ("end", 2), ("start", 0), ("end", 0)]
    await stub.release_all()
    assert stub.released_all_count == 1


def _stub_hw() -> hwmod.ResearchHardware:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    return hwmod.build_hardware(cfg, hwmod.HW_STUB)


async def test_stub_hw_has_buttons_not_connected_by_init() -> None:
    hw = _stub_hw()
    assert isinstance(hw.buttons, bs.StubButtonServo)
    await hwmod.run_initialize(hw)
    assert hw.buttons_connected is False  # 手順 1 では PCA9685 に接続しない
    assert hw.buttons.connected is False  # type: ignore[union-attr]


async def test_ensure_buttons_connected_then_shutdown_releases() -> None:
    hw = _stub_hw()
    await hwmod.run_initialize(hw)
    await hwmod.ensure_buttons_connected(hw)
    await hwmod.ensure_buttons_connected(hw)  # 2 回目は何もしない
    assert hw.buttons_connected is True
    buttons = hw.buttons
    assert isinstance(buttons, bs.StubButtonServo)
    assert buttons.connected
    await hwmod.shutdown(hw)
    assert buttons.released_all_count == 1
    assert buttons.connected is False
    assert hw.buttons_connected is False


async def test_shutdown_without_buttons_connected_skips_release() -> None:
    hw = _stub_hw()
    await hwmod.run_initialize(hw)
    buttons = hw.buttons
    assert isinstance(buttons, bs.StubButtonServo)
    await hwmod.shutdown(hw)
    assert buttons.released_all_count == 0
