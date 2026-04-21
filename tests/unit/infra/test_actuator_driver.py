"""ActuatorDriver のユニットテスト（pymodbus はモック化）。"""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.infra.actuator_driver import ActuatorDriver, _from_signed32, _to_signed32

# ---------------------------------------------------------------------------
# ヘルパー
# ---------------------------------------------------------------------------


def _make_reg_result(*values: int) -> MagicMock:
    """pymodbus の read_holding_registers 成功レスポンスを模倣する。"""
    result = MagicMock()
    result.isError.return_value = False
    result.registers = list(values)
    return result


def _make_error_result() -> MagicMock:
    result = MagicMock()
    result.isError.return_value = True
    return result


def _make_driver() -> tuple[ActuatorDriver, MagicMock]:
    """ActuatorDriver とモッククライアントのペアを返す。

    connect() 後の状態（_client がセット済み）を再現するため、
    _client を直接差し替えてテストに使用する。
    """
    driver = ActuatorDriver(port="/dev/ttyUSB0", slave_id=1)
    mock_client = MagicMock()
    mock_client.connect = AsyncMock(return_value=True)
    mock_client.close = MagicMock()
    mock_client.write_coil = AsyncMock()
    mock_client.write_registers = AsyncMock()
    mock_client.read_holding_registers = AsyncMock()
    # connect() を経由せず直接セット（pyserial 未インストール環境向け）
    driver._client = mock_client
    return driver, mock_client


# ---------------------------------------------------------------------------
# ユーティリティ関数テスト
# ---------------------------------------------------------------------------


class TestSignedConversion:
    def test_positive_32bit(self) -> None:
        assert _to_signed32(0x0000, 0x0064) == 100

    def test_negative_32bit(self) -> None:
        hi = 0xFFFF
        lo = 0xFF9C
        assert _to_signed32(hi, lo) == -100

    def test_zero(self) -> None:
        assert _to_signed32(0, 0) == 0

    def test_roundtrip_positive(self) -> None:
        hi, lo = _from_signed32(12345)
        assert _to_signed32(hi, lo) == 12345

    def test_roundtrip_negative(self) -> None:
        hi, lo = _from_signed32(-9999)
        assert _to_signed32(hi, lo) == -9999


# ---------------------------------------------------------------------------
# ActuatorDriver テスト
# ---------------------------------------------------------------------------


class TestConnect:
    @pytest.mark.asyncio
    async def test_connect_success(self) -> None:
        driver = ActuatorDriver(port="/dev/ttyUSB0", slave_id=1)
        mock_client = MagicMock()
        mock_client.connect = AsyncMock(return_value=True)

        with patch(
            "src.infra.actuator_driver.AsyncModbusSerialClient",
            return_value=mock_client,
        ):
            await driver.connect()

        mock_client.connect.assert_awaited_once()

    @pytest.mark.asyncio
    async def test_connect_failure_raises(self) -> None:
        driver = ActuatorDriver(port="/dev/ttyUSB0", slave_id=1)
        mock_client = MagicMock()
        mock_client.connect = AsyncMock(return_value=False)

        with patch(
            "src.infra.actuator_driver.AsyncModbusSerialClient",
            return_value=mock_client,
        ):
            with pytest.raises(ConnectionError):
                await driver.connect()


class TestServoControl:
    @pytest.mark.asyncio
    async def test_servo_on(self) -> None:
        driver, mock_client = _make_driver()
        await driver.servo_on()
        mock_client.write_coil.assert_awaited_once_with(
            address=0x0403, value=True, device_id=1
        )

    @pytest.mark.asyncio
    async def test_servo_off(self) -> None:
        driver, mock_client = _make_driver()
        await driver.servo_off()
        mock_client.write_coil.assert_awaited_once_with(
            address=0x0403, value=False, device_id=1
        )


class TestResetAlarm:
    @pytest.mark.asyncio
    async def test_reset_alarm_sends_edge(self) -> None:
        driver, mock_client = _make_driver()
        with patch("asyncio.sleep", AsyncMock()):
            await driver.reset_alarm()

        calls = mock_client.write_coil.await_args_list
        assert len(calls) == 2
        assert calls[0].kwargs["value"] is True
        assert calls[1].kwargs["value"] is False
        assert calls[0].kwargs["address"] == 0x0407


class TestHomeReturn:
    @pytest.mark.asyncio
    async def test_home_return_success(self) -> None:
        driver, mock_client = _make_driver()
        dss1_with_hend = 0x0010  # bit4 = HEND

        mock_client.read_holding_registers.return_value = _make_reg_result(dss1_with_hend)

        with patch("asyncio.sleep", AsyncMock()):
            await driver.home_return()

        mock_client.write_coil.assert_awaited_once_with(
            address=0x040B, value=True, device_id=1
        )

    @pytest.mark.asyncio
    async def test_home_return_timeout(self) -> None:
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.return_value = _make_reg_result(0x0000)

        call_count = 0

        def fake_time() -> float:
            nonlocal call_count
            call_count += 1
            return call_count * 100.0

        with (
            patch("asyncio.sleep", AsyncMock()),
            patch("asyncio.get_event_loop") as mock_loop_fn,
        ):
            mock_loop = MagicMock()
            mock_loop.time.side_effect = fake_time
            mock_loop_fn.return_value = mock_loop

            with pytest.raises(TimeoutError):
                await driver.home_return()


class TestMoveToPosition:
    @pytest.mark.asyncio
    async def test_move_to_position_calls_write_registers(self) -> None:
        driver, mock_client = _make_driver()
        await driver.move_to_position(pos=1000, speed_mm_s=50, accel_mm_s2=1000)

        mock_client.write_registers.assert_awaited_once()
        call_kwargs = mock_client.write_registers.await_args.kwargs
        assert call_kwargs["address"] == 0x9900
        assert call_kwargs["device_id"] == 1

        regs = call_kwargs["values"]
        # PCMD = 1000 → 0x000003E8
        assert regs[0] == 0x0000  # hi
        assert regs[1] == 0x03E8  # lo
        # CTLF = 0x0002
        assert regs[8] == 0x0002


class TestReadPosition:
    @pytest.mark.asyncio
    async def test_read_position_positive(self) -> None:
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.return_value = _make_reg_result(0x0000, 0x03E8)
        pos = await driver.read_position()
        assert pos == 1000

    @pytest.mark.asyncio
    async def test_read_position_negative(self) -> None:
        driver, mock_client = _make_driver()
        hi, lo = _from_signed32(-500)
        mock_client.read_holding_registers.return_value = _make_reg_result(hi, lo)
        pos = await driver.read_position()
        assert pos == -500

    @pytest.mark.asyncio
    async def test_read_position_error_raises(self) -> None:
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.return_value = _make_error_result()
        with pytest.raises(IOError):
            await driver.read_position()


class TestReadCurrent:
    @pytest.mark.asyncio
    async def test_read_current(self) -> None:
        driver, mock_client = _make_driver()
        # 電流 250 mA
        mock_client.read_holding_registers.return_value = _make_reg_result(0x0000, 0x00FA)
        current = await driver.read_current()
        assert current == 250.0

    @pytest.mark.asyncio
    async def test_read_current_error_raises(self) -> None:
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.return_value = _make_error_result()
        with pytest.raises(IOError):
            await driver.read_current()


class TestIsAlarmActive:
    @pytest.mark.asyncio
    async def test_no_alarm(self) -> None:
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.return_value = _make_reg_result(0)
        assert await driver.is_alarm_active() is False

    @pytest.mark.asyncio
    async def test_alarm_active(self) -> None:
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.return_value = _make_reg_result(0x0050)
        assert await driver.is_alarm_active() is True

    @pytest.mark.asyncio
    async def test_alarm_error_raises(self) -> None:
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.return_value = _make_error_result()
        with pytest.raises(IOError):
            await driver.is_alarm_active()
