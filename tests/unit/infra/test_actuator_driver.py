"""ActuatorDriver のユニットテスト（pymodbus はモック化）。"""

from __future__ import annotations

from unittest.mock import AsyncMock, MagicMock, call, patch

import pytest

from src.infra.actuator_driver import (
    ActuatorDriver,
    _from_signed32,
    _to_signed32,
    acmd_for_move,
    min_speed_for_lead,
)

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


def _make_driver(lead_mm: float = 0.0) -> tuple[ActuatorDriver, MagicMock]:
    """ActuatorDriver とモッククライアントのペアを返す。

    connect() 後の状態（_client がセット済み）を再現するため、
    _client を直接差し替えてテストに使用する。lead_mm=0 は「settings.toml に
    リード長が未記入」＝フォールバック下限（12.5mm/s）を使う既定の状態。
    """
    driver = ActuatorDriver(port="/dev/ttyUSB0", slave_id=1, lead_mm=lead_mm)
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
        mock_client.write_coil.assert_awaited_once_with(address=0x0403, value=True, device_id=1)

    @pytest.mark.asyncio
    async def test_servo_off(self) -> None:
        driver, mock_client = _make_driver()
        await driver.servo_off()
        mock_client.write_coil.assert_awaited_once_with(address=0x0403, value=False, device_id=1)


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

        calls = mock_client.write_coil.await_args_list
        assert len(calls) == 2
        assert calls[0] == call(address=0x040B, value=False, device_id=1)
        assert calls[1] == call(address=0x040B, value=True, device_id=1)

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


class TestEnableModbusControl:
    @pytest.mark.asyncio
    async def test_enable_modbus_control_writes_pmsl(self) -> None:
        driver, mock_client = _make_driver()
        await driver.enable_modbus_control()
        mock_client.write_coil.assert_awaited_once_with(address=0x0427, value=True, device_id=1)


class TestMoveToPosition:
    @pytest.mark.asyncio
    async def test_move_to_position_calls_write_registers(self) -> None:
        driver, mock_client = _make_driver()
        await driver.move_to_position(pos=1000, speed_mm_s=50, accel=30)

        mock_client.write_registers.assert_awaited_once()
        call_kwargs = mock_client.write_registers.await_args.kwargs
        assert call_kwargs["address"] == 0x9900
        assert call_kwargs["device_id"] == 1

        regs = call_kwargs["values"]
        # PCMD = 1000 → 0x000003E8
        assert regs[0] == 0x0000  # hi
        assert regs[1] == 0x03E8  # lo
        # VCMD = 50mm/s → 5000 (0.01mm/s単位) → 0x00001388
        assert regs[4] == 0x0000  # hi
        assert regs[5] == 0x1388  # lo (5000)
        # CTLF = 0x0000 (絶対位置移動)
        assert regs[8] == 0x0000


class TestMoveToPositionTimed:
    @pytest.mark.asyncio
    async def test_speed_computed_from_distance_and_duration(self) -> None:
        driver, mock_client = _make_driver()
        # 距離 7500pulse=75.0mm を 1.5s → 50mm/s → VCMD 5000(0.01mm/s) = 0x1388
        await driver.move_to_position_timed(target_pos=7500, current_pos=0, duration_s=1.5)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[0] == 0x0000 and regs[1] == 0x1D4C  # PCMD = 7500
        assert regs[4] == 0x0000 and regs[5] == 0x1388  # VCMD = 5000 (50mm/s)

    @pytest.mark.asyncio
    async def test_speed_clamped_to_floor(self) -> None:
        """最低速度（RCP6-ROD 仕様: リード長÷0.8）を下回る要求は下限へ引き上げる。

        仕様に「最低速度以下の速度は設定しないでください。設定した速度では動きません」と
        明記されており、旧実装の下限 1mm/s はどのリードでもこれを下回る仕様違反だった
        （学習運転の微小移動指令が黙って無視され、同定データが汚染されうる）。
        """
        driver, mock_client = _make_driver()
        # 0.1mm を 10s → 0.01mm/s → 下限 12.5mm/s → VCMD 1250 = 0x04E2
        await driver.move_to_position_timed(target_pos=10, current_pos=0, duration_s=10.0)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[4] == 0x0000 and regs[5] == 0x04E2

    @pytest.mark.asyncio
    async def test_speed_clamped_to_ceiling(self) -> None:
        driver, mock_client = _make_driver()
        # 1000mm を 0.1s → 10000mm/s → 上限 100mm/s → VCMD 10000 = 0x2710
        await driver.move_to_position_timed(target_pos=100000, current_pos=0, duration_s=0.1)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[4] == 0x0000 and regs[5] == 0x2710

    @pytest.mark.asyncio
    async def test_nonpositive_duration_uses_max_speed(self) -> None:
        driver, mock_client = _make_driver()
        # duration<=0（安全フリーズ経路）→ 許容最速 100mm/s で目標へ
        await driver.move_to_position_timed(target_pos=7500, current_pos=0, duration_s=0.0)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[1] == 0x1D4C  # PCMD = 7500（目標は届く）
        assert regs[5] == 0x2710  # VCMD = 10000 (100mm/s)

    @pytest.mark.asyncio
    async def test_zero_distance_uses_max_speed(self) -> None:
        driver, mock_client = _make_driver()
        await driver.move_to_position_timed(target_pos=500, current_pos=500, duration_s=1.0)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[5] == 0x2710  # 距離0 → 最速フォールバック


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


class TestTransactionInstrumentation:
    """ストール切り分け計装（.steering/20260620-modbus-retry-cycle-stall）のテスト。"""

    @pytest.mark.asyncio
    async def test_retries_on_response_logged_as_warning(
        self, caplog: pytest.LogCaptureFixture
    ) -> None:
        """pymodbus が付与する response.retries > 0 を再送検知として WARNING ログする。"""
        driver, mock_client = _make_driver()
        result = _make_reg_result(0x0000, 0x00FA)
        result.retries = 2
        mock_client.read_holding_registers.return_value = result

        with caplog.at_level("WARNING", logger="src.infra.actuator_driver"):
            await driver.read_current()

        assert any(
            "Modbus再送検知" in r.message and "retries=2" in r.message for r in caplog.records
        )

    @pytest.mark.asyncio
    async def test_no_retries_does_not_log_warning(self, caplog: pytest.LogCaptureFixture) -> None:
        driver, mock_client = _make_driver()
        result = _make_reg_result(0x0000, 0x00FA)
        result.retries = 0
        mock_client.read_holding_registers.return_value = result

        with caplog.at_level("WARNING", logger="src.infra.actuator_driver"):
            await driver.read_current()

        assert not any("Modbus再送検知" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_mock_response_without_int_retries_does_not_crash(self) -> None:
        """MagicMock 応答（テストダブル）は retries 属性が自動生成された Mock になるため、
        int でない場合は 0 扱いにしてフォーマット時の例外を防ぐ（回帰防止）。
        """
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.return_value = _make_reg_result(0x0000, 0x00FA)
        current = await driver.read_current()
        assert current == 250.0

    @pytest.mark.asyncio
    async def test_exhausted_retries_logs_warning_and_reraises(
        self, caplog: pytest.LogCaptureFixture
    ) -> None:
        """全再送を使い切って pymodbus が例外を送出した場合も計装ログを残し、例外は伝播する。"""
        driver, mock_client = _make_driver()
        mock_client.read_holding_registers.side_effect = TimeoutError("no response")

        with caplog.at_level("WARNING", logger="src.infra.actuator_driver"):
            with pytest.raises(TimeoutError):
                await driver.read_current()

        assert any("Modbus再送上限到達" in r.message for r in caplog.records)

    @pytest.mark.asyncio
    async def test_axis_name_appears_in_log(self, caplog: pytest.LogCaptureFixture) -> None:
        driver = ActuatorDriver(port="/dev/ttyUSB0", slave_id=1, axis_name="accel")
        mock_client = MagicMock()
        mock_client.read_holding_registers = AsyncMock()
        result = _make_reg_result(0x0000, 0x00FA)
        result.retries = 1
        mock_client.read_holding_registers.return_value = result
        driver._client = mock_client

        with caplog.at_level("WARNING", logger="src.infra.actuator_driver"):
            await driver.read_current()

        assert any("axis=accel" in r.message for r in caplog.records)


class TestAcmdForMove:
    """移動ごとの加減速度算出（acmd_for_move）。

    実機 9eee549b の全サイクル実測では、1 サイクル移動量は accel 中央値 0.082mm /
    p95 0.215mm / max 0.630mm、brake 中央値 0.040mm / max 2.244mm。100mm/s・0.3G の
    三角プロファイル境界 3.4mm を一度も超えない＝所要時間は VCMD ではなく ACMD だけで
    決まり、0.3G 固定では 0.082mm が 10.6ms で終わって**50ms サイクルの 79〜85% を
    アクチュエータが停止して過ごす**（ペダルが 20Hz の階段状に動く）。
    """

    def test_tiny_move_uses_minimum_accel(self) -> None:
        """中央値相当の微小移動は最小近傍まで落ちる（周期いっぱいかけて動く）。"""
        assert acmd_for_move(0.082, 0.04) <= 3

    def test_large_move_keeps_default_accel(self) -> None:
        """実測最大級の移動は従来どおり 0.3G（上限）で速く動く。"""
        assert acmd_for_move(2.244, 0.04) == 30

    def test_acmd_scales_with_distance(self) -> None:
        """a = 4d/t² なので距離に比例する。"""
        # 0.2mm → 5.1→5、0.8mm → 20.4→20（小さい整数域の丸め差を避けて比較する）
        assert acmd_for_move(0.8, 0.04) == pytest.approx(
            acmd_for_move(0.2, 0.04) * 4, rel=0.1
        )

    def test_clamped_to_spec_range(self) -> None:
        """MODBUS 5-132 の設定範囲 1〜300 内、かつ従来の 0.3G を超えない。"""
        for d in (0.0, 1e-6, 0.5, 5.0, 1000.0):
            assert 1 <= acmd_for_move(d, 0.04) <= 30

    def test_zero_or_negative_duration_uses_default(self) -> None:
        assert acmd_for_move(0.5, 0.0) == 30
        assert acmd_for_move(0.0, 0.04) == 30


class TestMoveSmoothing:
    """move_to_position の smooth_over_s（前回指令位置からの距離で ACMD を決める）。"""

    @pytest.mark.asyncio
    async def test_first_move_uses_default_accel(self) -> None:
        """前回指令位置が無い初回は既定 0.3G（距離が分からないため）。"""
        driver, mock_client = _make_driver()
        await driver.move_to_position(1000, smooth_over_s=0.04)
        assert mock_client.write_registers.await_args.kwargs["values"][6] == 30

    @pytest.mark.asyncio
    async def test_small_step_lowers_accel(self) -> None:
        """2 回目以降は前回指令位置からの距離で ACMD が下がる。"""
        driver, mock_client = _make_driver()
        await driver.move_to_position(1000, smooth_over_s=0.04)
        await driver.move_to_position(1008, smooth_over_s=0.04)  # 0.08mm
        assert mock_client.write_registers.await_args.kwargs["values"][6] <= 3

    @pytest.mark.asyncio
    async def test_explicit_accel_wins(self) -> None:
        """accel を明示したら平滑化しない（既存呼び出しの後方互換）。"""
        driver, mock_client = _make_driver()
        await driver.move_to_position(1000, smooth_over_s=0.04)
        await driver.move_to_position(1008, accel=7, smooth_over_s=0.04)
        assert mock_client.write_registers.await_args.kwargs["values"][6] == 7

    @pytest.mark.asyncio
    async def test_without_smoothing_keeps_default(self) -> None:
        """smooth_over_s 未指定なら従来どおり 0.3G 固定。"""
        driver, mock_client = _make_driver()
        await driver.move_to_position(1000)
        await driver.move_to_position(1008)
        assert mock_client.write_registers.await_args.kwargs["values"][6] == 30

    @pytest.mark.asyncio
    async def test_absolute_positioning_flags(self) -> None:
        """CTLF=0（絶対位置移動・台形パターン）。INC ビットを立てない。

        相対移動は Modbus の応答欠落が起きたときペダル位置が恒久的にずれる。
        絶対なら次サイクルの指令で自己回復する。
        """
        driver, mock_client = _make_driver()
        await driver.move_to_position(1000)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[8] == 0x0000


class TestMinSpeedForLead:
    """RCP6-ROD 1.2.1「最低速度 = リード長 ÷ 800 ÷ 0.001秒」の実装。

    実機（docs/hardware.md）は accel が RA6R リード 6mm、brake が RA7R リード 8mm。
    """

    def test_accel_lead_6mm(self) -> None:
        assert min_speed_for_lead(6.0) == pytest.approx(7.5)

    def test_brake_lead_8mm(self) -> None:
        assert min_speed_for_lead(8.0) == pytest.approx(10.0)

    def test_unset_lead_falls_back(self) -> None:
        """リード長未記入（0）なら従来のフォールバック 12.5mm/s。"""
        assert min_speed_for_lead(0.0) == pytest.approx(12.5)
        assert min_speed_for_lead(-1.0) == pytest.approx(12.5)


class TestPerAxisSpeedFloor:
    """時間指定移動の下限速度が軸ごとのリード長から決まること。"""

    @pytest.mark.asyncio
    async def test_accel_axis_floor_is_7_5(self) -> None:
        driver, mock_client = _make_driver(lead_mm=6.0)
        # 0.1mm を 10s → 0.01mm/s → 下限 7.5mm/s → VCMD 750 = 0x02EE
        await driver.move_to_position_timed(target_pos=10, current_pos=0, duration_s=10.0)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[4] == 0x0000 and regs[5] == 0x02EE

    @pytest.mark.asyncio
    async def test_brake_axis_floor_is_10(self) -> None:
        driver, mock_client = _make_driver(lead_mm=8.0)
        # 下限 10.0mm/s → VCMD 1000 = 0x03E8
        await driver.move_to_position_timed(target_pos=10, current_pos=0, duration_s=10.0)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[4] == 0x0000 and regs[5] == 0x03E8

    @pytest.mark.asyncio
    async def test_lead_does_not_affect_speeds_above_floor(self) -> None:
        """下限より速い要求はリード長に関係なくそのまま通る。"""
        driver, mock_client = _make_driver(lead_mm=6.0)
        # 75.0mm を 1.5s → 50mm/s → VCMD 5000 = 0x1388
        await driver.move_to_position_timed(target_pos=7500, current_pos=0, duration_s=1.5)
        regs = mock_client.write_registers.await_args.kwargs["values"]
        assert regs[4] == 0x0000 and regs[5] == 0x1388
