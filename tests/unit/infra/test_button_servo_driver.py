"""ButtonServoDriver（PCA9685 + SG90）のユニットテスト。

実 I2C ハードウェアなしで検証するためフェイクバスを注入する。
"""

from __future__ import annotations

import asyncio

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


class FlakyI2CBus(FakeI2CBus):
    """write_i2c_block_data を任意回数だけ OSError で失敗させられるフェイクバス。"""

    def __init__(self) -> None:
        super().__init__()
        self.fail_next = 0

    def write_i2c_block_data(self, addr: int, reg: int, data: list[int]) -> None:
        if self.fail_next > 0:
            self.fail_next -= 1
            raise OSError("I2C error (simulated)")
        super().write_i2c_block_data(addr, reg, data)


def _make_driver(bus: FakeI2CBus) -> ButtonServoDriver:
    return ButtonServoDriver(rest_angle=60.0, press_angle=110.0, bus_factory=lambda _num: bus)


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


async def test_press_cancelled_during_hold_still_releases_channel() -> None:
    """I1 回帰テスト: 保持中にキャンセルされてもボタンを待機角へ戻す（押しっぱなし防止）。"""
    bus = FakeI2CBus()
    driver = _make_driver(bus)
    await driver.connect()
    bus.block_writes.clear()
    task = asyncio.ensure_future(driver.press(CH_ENGINE_START, duration_s=10.0))
    # 押下角の書込みは asyncio.to_thread（実スレッド）のため、sleep(10.0) に入るまで
    # 実時間で待つ必要がある（sleep(0) では executor の完了を保証できない）。
    await asyncio.sleep(0.05)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    # 押下角の書込み + キャンセルされても解放（待機角）の書込みが行われる
    assert len(bus.block_writes) == 2


async def test_release_channel_retries_once_on_i2c_error() -> None:
    """I1 回帰テスト: 解放が1回 OSError で失敗しても即座にリトライして成功させる。"""
    bus = FlakyI2CBus()
    driver = _make_driver(bus)
    await driver.connect()
    bus.block_writes.clear()
    bus.fail_next = 1
    await driver._release_channel(CH_ENGINE_START)  # 例外は送出されない
    assert len(bus.block_writes) == 1  # 1回失敗（未記録）+ リトライで1回成功（記録）


async def test_release_channel_logs_and_does_not_raise_when_persistently_failing() -> None:
    """I1 回帰テスト: 解放が繰り返し失敗しても press()/finally 経路を止めない
    （例外を送出せずログのみ残す）。"""
    bus = FlakyI2CBus()
    driver = _make_driver(bus)
    await driver.connect()
    bus.fail_next = 999  # 常に失敗させる
    await driver._release_channel(CH_ENGINE_START)  # 例外を送出しない


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
