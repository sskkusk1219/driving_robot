"""ButtonServoDriver（PCA9685 + SG90）のユニットテスト。

実 I2C ハードウェアなしで検証するためフェイクバスを注入する。
"""

from __future__ import annotations

import pytest

from src.infra.button_servo_driver import (
    CH_ENGINE_START,
    ButtonServoDriver,
    angle_to_count,
)


class FakeI2CBus:
    """write_i2c_block_data / read_byte_data を記録するフェイク I2C バス。"""

    def __init__(self) -> None:
        self.block_writes: list[tuple[int, int, list[int]]] = []
        self.byte_writes: list[tuple[int, int, int]] = []
        self.closed = False

    def write_byte_data(self, addr: int, reg: int, value: int) -> None:
        self.byte_writes.append((addr, reg, value))

    def read_byte_data(self, addr: int, reg: int) -> int:
        return 0x20  # MODE1 AI

    def write_i2c_block_data(self, addr: int, reg: int, data: list[int]) -> None:
        self.block_writes.append((addr, reg, list(data)))

    def close(self) -> None:
        self.closed = True


def _make_driver(bus: FakeI2CBus) -> ButtonServoDriver:
    return ButtonServoDriver(
        rest_angle=60.0, press_angle=110.0, bus_factory=lambda _num: bus
    )


# ── 角度→カウント変換（純関数）─────────────────────────────────────────


def test_angle_to_count_endpoints() -> None:
    # 50Hz: 1 周期 20000us → 4096 count。0.5ms=~102, 2.5ms=~512
    assert angle_to_count(0.0, 50) == pytest.approx(102, abs=1)
    assert angle_to_count(180.0, 50) == pytest.approx(512, abs=1)


def test_angle_to_count_clamps_out_of_range() -> None:
    assert angle_to_count(-30.0, 50) == angle_to_count(0.0, 50)
    assert angle_to_count(999.0, 50) == angle_to_count(180.0, 50)


# ── connect / press / release_all ──────────────────────────────────────


async def test_connect_moves_all_channels_to_rest() -> None:
    bus = FakeI2CBus()
    driver = _make_driver(bus)
    await driver.connect()
    # 16ch すべてに待機角の PWM を書く
    assert len(bus.block_writes) == 16


async def test_press_writes_press_then_rest_angle() -> None:
    bus = FakeI2CBus()
    driver = _make_driver(bus)
    await driver.connect()
    bus.block_writes.clear()
    await driver.press(CH_ENGINE_START, duration_s=0.0)
    # 押下角 → 待機角 の2回。ch0 の LED0 レジスタ(0x06)宛
    assert len(bus.block_writes) == 2
    assert all(w[1] == 0x06 for w in bus.block_writes)


async def test_press_rejects_out_of_range_channel() -> None:
    bus = FakeI2CBus()
    driver = _make_driver(bus)
    await driver.connect()
    with pytest.raises(ValueError):
        await driver.press(16, duration_s=0.0)


async def test_press_before_connect_raises() -> None:
    driver = _make_driver(FakeI2CBus())
    with pytest.raises(RuntimeError):
        await driver.press(0, duration_s=0.0)


async def test_release_all_before_connect_is_noop() -> None:
    driver = _make_driver(FakeI2CBus())
    await driver.release_all()  # 未接続でも例外を出さない


async def test_check_connection_true_when_bus_ok() -> None:
    driver = _make_driver(FakeI2CBus())
    await driver.connect()
    assert await driver.check_connection() is True


async def test_check_connection_false_when_not_connected() -> None:
    driver = _make_driver(FakeI2CBus())
    assert await driver.check_connection() is False
