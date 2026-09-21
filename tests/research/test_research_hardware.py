"""研究開発用ハーネス 手順 1（初期化）のユニットテスト。

実機には触らず、スタブ HW で初期化シーケンスの順序と合否判定を確認する。
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

import pytest

from src.domain.pre_check import HOME_POSITION_TOLERANCE_PULSE, UPS_MIN_BATTERY_PCT
from src.infra.actuator_driver import acmd_for_move
from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import main as mainmod
from tests.research.axis_monitor import (
    DSS1_PEND,
    DSS1_SV,
    DSSE_MOVE,
    MONITOR_REGISTER_COUNT,
    AxisMonitor,
)
from tests.research.research_actuator import ResearchActuatorDriver


def _stub_hw() -> hwmod.ResearchHardware:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    return hwmod.build_hardware(cfg, hwmod.HW_STUB)


def test_build_hardware_stub_uses_stubs() -> None:
    hw = _stub_hw()
    assert isinstance(hw.accel, hwmod.StubActuator)
    assert isinstance(hw.brake, hwmod.StubActuator)
    assert isinstance(hw.can, hwmod.StubCANReader)
    assert isinstance(hw.ups, hwmod.StubUPSMonitor)
    assert hw.is_real is False
    assert hw.settings is None
    # 構築しただけでは接続しない（run_initialize で初めて開く）
    assert hw.connected is False
    assert hw.initialized is False


def test_build_hardware_rejects_unknown_mode() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    with pytest.raises(hwmod.InitializationError, match="未知のハードウェアモード"):
        hwmod.build_hardware(cfg, "bench")


async def test_run_initialize_all_items_pass() -> None:
    hw = _stub_hw()
    report = await hwmod.run_initialize(hw)

    assert report.passed
    assert hw.initialized and hw.connected
    # 順序は本番 RobotController.start() + initialize() と同じでなければならない
    assert [item.name for item in report.items] == [
        "接続 (アクセル/ブレーキ/CAN)",
        "Modbus 操作権 (ブレーキ→アクセル)",
        "サーボ通信確認 (現在位置読み出し)",
        "エラー消去 (アラームリセット)",
        "サーボON (両軸)",
        "CAN 通信チェック (車速読み出し)",
        "UPS 通信チェック (残量)",
        "アクチュエータ初期位置 (原点復帰)",
    ]
    # サーボONと原点復帰が実際に効いている
    assert hw.accel.servo_on_state and hw.brake.servo_on_state
    assert hw.accel.homed and hw.brake.homed


async def test_initialize_fails_when_alarm_does_not_clear() -> None:
    """リセットしても消えないアラームは走行に進ませない。"""
    hw = _stub_hw()

    async def stuck_reset() -> None:
        pass  # アラームを消さない

    hw.accel.alarm = True
    hw.accel.reset_alarm = stuck_reset  # type: ignore[method-assign]

    with pytest.raises(hwmod.InitializationError, match="アラームが残っています"):
        await hwmod.run_initialize(hw)
    assert hw.initialized is False
    # サーボONまで進んでいないこと（アラーム段で止まる）
    assert hw.accel.servo_on_state is False


async def test_initialize_does_not_require_zero_speed() -> None:
    """初期化は CAN が読めればよい。車速が 0 でなくても通す（本番 initialize() と同じ）。"""
    hw = _stub_hw()
    hw.can.speed_kmh = 30.0

    report = await hwmod.run_initialize(hw)

    assert report.passed
    assert hw.accel.homed and hw.brake.homed


async def test_initialize_fails_when_can_is_silent() -> None:
    """CAN が読めなければ NG（バス無音・DBC 不一致など）。"""
    hw = _stub_hw()

    async def silent_read() -> float:
        raise TimeoutError("CAN Speed フレームを 2s 以内に受信できません")

    hw.can.read_speed = silent_read  # type: ignore[method-assign]

    with pytest.raises(hwmod.InitializationError, match="CAN 通信チェック"):
        await hwmod.run_initialize(hw)
    assert hw.accel.homed is False


async def test_initialize_fails_on_low_ups_battery() -> None:
    hw = _stub_hw()
    hw.ups.battery_pct = UPS_MIN_BATTERY_PCT - 1.0

    with pytest.raises(hwmod.InitializationError, match="UPS 残量不足"):
        await hwmod.run_initialize(hw)


async def test_initialize_fails_when_ups_unreachable() -> None:
    hw = _stub_hw()
    hw.ups.available = False

    with pytest.raises(hwmod.InitializationError, match="UPS 通信チェック"):
        await hwmod.run_initialize(hw)


async def test_initialize_fails_when_axis_not_at_home() -> None:
    """原点復帰しても位置が許容外なら NG。"""
    hw = _stub_hw()
    off = HOME_POSITION_TOLERANCE_PULSE + 1

    async def fake_home() -> None:
        hw.brake.position = off
        hw.brake.homed = True

    hw.brake.home_return = fake_home  # type: ignore[method-assign]

    with pytest.raises(hwmod.InitializationError, match="原点から離れています"):
        await hwmod.run_initialize(hw)


async def test_run_initialize_skips_ups_when_disabled() -> None:
    """checks.init_ups: false なら NUT ポーリングを一切開始せず [SKIP] 扱いになる。"""
    hw = _stub_hw()
    calls: list[str] = []
    original_poll = hw.ups.start_polling

    async def record_poll() -> None:
        calls.append("start_polling")
        await original_poll()

    hw.ups.start_polling = record_poll  # type: ignore[method-assign]
    checks = cfgmod.ChecksSection(init_ups=False)

    report = await hwmod.run_initialize(hw, checks)

    assert report.passed
    assert calls == []  # NUT には一切接続しない
    skip_item = next(i for i in report.items if i.name == "UPS 通信チェック")
    assert skip_item.skipped and skip_item.passed
    assert skip_item.detail == "checks.init_ups: false"
    assert report.skip_count == 1
    assert report.ok_count == len(report.items) - 1


async def test_run_initialize_skips_home_return_when_disabled() -> None:
    """checks.init_home_return: false なら原点復帰を呼ばない（開度 0% の基準は保証されない）。"""
    hw = _stub_hw()
    checks = cfgmod.ChecksSection(init_home_return=False)

    report = await hwmod.run_initialize(hw, checks)

    assert report.passed
    assert hw.accel.homed is False and hw.brake.homed is False
    skip_item = next(i for i in report.items if i.name == "アクチュエータ初期位置")
    assert skip_item.skipped
    assert skip_item.detail == "checks.init_home_return: false"


async def test_shutdown_homes_and_servo_off() -> None:
    hw = _stub_hw()
    await hwmod.run_initialize(hw)
    hw.accel.position = 500

    await hwmod.shutdown(hw)

    assert hw.accel.position == 0
    assert hw.accel.servo_on_state is False and hw.brake.servo_on_state is False
    assert hw.accel.connected is False and hw.can.connected is False
    assert hw.connected is False and hw.initialized is False


async def test_shutdown_continues_after_a_failure() -> None:
    """1 手順が失敗してもサーボOFF・切断まで必ず到達する（踏んだまま終わらせない）。"""
    hw = _stub_hw()
    await hwmod.run_initialize(hw)

    async def broken_home() -> None:
        raise OSError("Modbus 応答なし")

    hw.accel.home_return = broken_home  # type: ignore[method-assign]

    await hwmod.shutdown(hw)

    assert hw.accel.servo_on_state is False
    assert hw.accel.connected is False


async def test_shutdown_closes_ports_after_partial_connect() -> None:
    """片方だけ接続できた場合でも掴んだポートを必ず閉じる。"""
    hw = _stub_hw()

    async def broken_connect() -> None:
        raise ConnectionError("Modbus RTU 接続失敗")

    hw.brake.connect = broken_connect  # type: ignore[method-assign]

    with pytest.raises(hwmod.InitializationError, match="接続"):
        await hwmod.run_initialize(hw)
    assert hw.accel.connected is True  # アクセル側は開いてしまっている

    await hwmod.shutdown(hw)
    assert hw.accel.connected is False
    assert hw.can.connected is False


# ── CLI 経由 ─────────────────────────────────────────────────────────


def test_step1_via_cli_succeeds_in_stub_mode(capsys: pytest.CaptureFixture[str]) -> None:
    assert mainmod.main(["--only", "1"]) == 0
    out = capsys.readouterr().out
    assert "ハードウェア: スタブ" in out
    assert "手順 1 完了" in out
    # 終了処理まで到達している
    assert "終了処理: 完了" in out


def test_step0_and_1_run_together(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
        }
    )
    assert mainmod.main(["--upto", "1", "--config", str(path)]) == 0
    out = capsys.readouterr().out
    assert "手順 0 完了" in out
    assert "手順 1 完了" in out
    assert "完了（手順 0, 1）" in out


def test_require_hardware_raises_before_step1() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    ctx = mainmod.RunContext(config=cfg, hw_mode=hwmod.HW_STUB, started_at=0.0)
    with pytest.raises(hwmod.InitializationError, match="手順 1 を先に実行"):
        ctx.require_hardware()


def test_initialization_error_exit_code_is_4(capsys: pytest.CaptureFixture[str]) -> None:
    """初期化 NG は専用の終了コードで返す（設定エラー 2 / 未実装 3 と区別する）。"""
    original = hwmod.build_hardware

    def build_silent_can(cfg: cfgmod.ResearchConfig, hw_mode: str) -> hwmod.ResearchHardware:
        hw = original(cfg, hw_mode)

        async def silent_read() -> float:
            raise TimeoutError("CAN 車速が 3.0s 更新されていません（バス無音の疑い）")

        hw.can.read_speed = silent_read  # type: ignore[method-assign]
        return hw

    mainmod.build_hardware = build_silent_can  # type: ignore[assignment]
    try:
        assert mainmod.main(["--only", "1"]) == 4
    finally:
        mainmod.build_hardware = original  # type: ignore[assignment]
    assert "初期化エラー" in capsys.readouterr().out



# ── A7: まとめ読み（実位置・電流・ステータス）と指令位置の記憶 ─────────────


def _monitor_registers(
    *, pos: int, current: int, alarm: int = 0, dss1: int = 0, dsse: int = 0
) -> list[int]:
    regs = [0] * MONITOR_REGISTER_COUNT
    regs[0], regs[1] = (pos >> 16) & 0xFFFF, pos & 0xFFFF
    regs[2], regs[5], regs[7] = alarm, dss1, dsse
    regs[12], regs[13] = (current >> 16) & 0xFFFF, current & 0xFFFF
    return regs


def test_axis_monitor_decodes_registers() -> None:
    regs = _monitor_registers(
        pos=-25, current=412, alarm=0x0E8, dss1=DSS1_SV | DSS1_PEND, dsse=DSSE_MOVE
    )
    m = AxisMonitor.from_registers(regs)
    assert m == AxisMonitor(
        position_pulse=-25, current_ma=412.0, alarm_code=0x0E8,
        servo_on=True, moving=True, pos_done=True,
    )
    off = AxisMonitor.from_registers(_monitor_registers(pos=9500, current=0))
    assert (off.position_pulse, off.servo_on, off.moving, off.pos_done) == (
        9500, False, False, False
    )
    with pytest.raises(ValueError, match="14 レジスタ"):
        AxisMonitor.from_registers(regs[:2])


async def test_stub_actuator_monitor_follows_command() -> None:
    axis = hwmod.StubActuator(axis_name="brake")
    await axis.connect()
    assert axis.last_command_pos is None
    await axis.servo_on()
    await axis.home_return()
    assert axis.last_command_pos == 0
    await axis.move_to_position(1200)
    m = await axis.read_monitor()
    assert axis.last_command_pos == m.position_pulse == 1200
    assert (m.servo_on, m.alarm_code) == (True, 0)
    await axis.move_to_position_timed(800, 1200, 0.1)
    assert axis.last_command_pos == 800
    axis.alarm = True
    assert (await axis.read_monitor()).alarm_code != 0


def _research_driver() -> tuple[ResearchActuatorDriver, MagicMock]:
    driver = ResearchActuatorDriver(port="/dev/ttyUSB0", slave_id=1)
    client = MagicMock()
    ok = MagicMock()
    ok.isError.return_value = False
    client.write_registers = AsyncMock(return_value=ok)
    client.read_holding_registers = AsyncMock()
    driver._client = client
    return driver, client


async def test_research_driver_reads_monitor_in_one_request() -> None:
    driver, client = _research_driver()
    result = MagicMock()
    result.isError.return_value = False
    result.registers = _monitor_registers(pos=3100, current=250, dss1=DSS1_SV)
    client.read_holding_registers.return_value = result

    m = await driver.read_monitor()

    client.read_holding_registers.assert_awaited_once_with(
        address=0x9000, count=14, device_id=1
    )
    assert (m.position_pulse, m.current_ma, m.servo_on) == (3100, 250.0, True)

    error = MagicMock()
    error.isError.return_value = True
    client.read_holding_registers.return_value = error
    with pytest.raises(OSError, match="read_monitor"):
        await driver.read_monitor()


async def test_research_driver_remembers_last_command() -> None:
    driver, client = _research_driver()
    assert driver.last_command_pos is None
    await driver.move_to_position(1500, smooth_over_s=0.04)
    assert driver.last_command_pos == 1500
    await driver.move_to_position_timed(900, 1500, 0.1)  # 本番は move_to_position に委ねる
    assert driver.last_command_pos == 900
    assert client.write_registers.await_count == 2


# ── 段2a: 0A7「指令減速度異常」対策 ─────────────────────────────────────
# 移動中に前回とほぼ同じ位置を再指令すると、ACMD が「前回指令との差」だけで決まる
# 従来ロジックでは下限（1 = 0.01G）に落ちてアラームになる（ProblemReport_20260916）。
# ResearchActuatorDriver は残距離（実位置→目標）との大きい方で ACMD を決め直す。


async def _read_monitor_into(
    driver: ResearchActuatorDriver, client: MagicMock, registers: list[int]
) -> None:
    """read_monitor() を 1 回呼んで driver._last_monitor を更新する。"""
    result = MagicMock()
    result.isError.return_value = False
    result.registers = registers
    client.read_holding_registers.return_value = result
    await driver.read_monitor()


def _last_written_accel(client: MagicMock) -> int:
    return client.write_registers.await_args.kwargs["values"][6]


def _pct_to_pulse(pct: float) -> int:
    """開度[%] → pulse（100% = 9500 pulse、実ログの換算に合わせる）。"""
    return round(pct / 100.0 * 9500)


async def test_research_driver_acmd_unchanged_when_axis_at_rest() -> None:
    """静止時（pos_done かつ非 moving）は従来どおり前回指令との差で ACMD が決まる。"""
    driver, client = _research_driver()
    driver.last_command_pos = 1000
    await _read_monitor_into(
        driver, client, _monitor_registers(pos=1000, current=0, dss1=DSS1_PEND)
    )

    await driver.move_to_position(1010, smooth_over_s=0.04)  # 微小移動 0.10mm

    assert _last_written_accel(client) == acmd_for_move(0.10, 0.04)


async def test_research_driver_acmd_uses_remaining_distance_when_moving() -> None:
    """移動中に前回指令とほぼ同じ位置を再指令しても ACMD は下限 1 に落ちない（30 になる）。"""
    driver, client = _research_driver()
    driver.last_command_pos = 1000
    # moving=1・pos_done=0、実位置は目標（1001 付近）の約 9mm 手前
    await _read_monitor_into(
        driver, client, _monitor_registers(pos=100, current=25, dsse=DSSE_MOVE)
    )

    await driver.move_to_position(1001, smooth_over_s=0.04)  # 前回指令との差はわずか 0.01mm

    assert _last_written_accel(client) == 30


async def test_research_driver_acmd_uses_last_command_when_monitor_unread() -> None:
    """read_monitor() を一度も呼んでいなければ従来どおり前回指令との差で決まる。"""
    driver, client = _research_driver()
    driver.last_command_pos = 1000
    assert driver._last_monitor is None

    await driver.move_to_position(1010, smooth_over_s=0.04)

    assert _last_written_accel(client) == acmd_for_move(0.10, 0.04)


async def test_research_driver_acmd_explicit_value_passes_through() -> None:
    """呼び出し側が accel= を明示したら残距離計算を経由せずそのまま送られる。"""
    driver, client = _research_driver()
    driver.last_command_pos = 1000
    await _read_monitor_into(
        driver, client, _monitor_registers(pos=100, current=25, dsse=DSSE_MOVE)
    )

    await driver.move_to_position(1001, accel=15, smooth_over_s=0.04)

    assert _last_written_accel(client) == 15


async def test_research_driver_acmd_reproduces_real_log_case() -> None:
    """実ログ再現: 28.420%→17.099% の移動中に 17.101% を再指令しても ACMD は 1 にならない。

    drive_log_real_20260918_041125.csv mode_time 136.60→136.65s。従来ロジックでは
    前回指令(17.099%)との差が 0.002% しかないため ACMD が下限 1 に落ち、0A7 アラームに
    至った場面。
    """
    driver, client = _research_driver()
    driver.last_command_pos = _pct_to_pulse(17.099)
    # このモニタは 136.65s 時点の実位置（26.589%）で、まだ目標まで動いている途中
    await _read_monitor_into(
        driver,
        client,
        _monitor_registers(pos=_pct_to_pulse(26.589), current=25, dsse=DSSE_MOVE),
    )

    await driver.move_to_position(_pct_to_pulse(17.101), smooth_over_s=0.04)

    assert _last_written_accel(client) != 1
