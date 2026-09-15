"""研究開発用ハーネス 走行ログ（drive_log.py）のユニットテスト。

走行前チェック → ペダル探索 → パターン走行 → 緩減速〜停車保持 が 1 本の CSV に区間つきで残ること、
FF モデルの学習はパターン走行の行だけを読むこと、異常終了でもログが残ることを確かめる。
A7（KAIZEN 表5-5 順6）からは全区間で同じ列（指令・実開度・ステータス）になり、旧形式も読めること。
"""

from __future__ import annotations

import asyncio
import csv
from datetime import UTC, datetime
from pathlib import Path

import pytest

from src.models.drive_log import DriveLogData
from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import hardware as hwmod
from tests.research import main as mainmod
from tests.research import pattern_drive as pdmod
from tests.research import pedal_search as psmod
from tests.research import pre_drive_check as pcmod
from tests.research.axis_monitor import AxisMonitor
from tests.research.test_research_pattern_drive import FAST_LOOP, SHORT_PATTERNS

FAST = {
    "pedal_search.step_mm": 1.0,
    "pedal_search.dwell_s": 0.3,
    "pedal_search.onset_margin_kmh": 0.05,  # スタブは車速ノイズが無い
    "pedal_search.creep_stable_kmh": 0.05,
    "pedal_search.creep_timeout_s": 10.0,
    "pedal_search.stop_hold_margin_pct": 3.0,
    "feedforward.creep_rate_kmhs": 3.0,
    "decel_stop.step_mm": 1.0,
    "decel_stop.dwell_s": 0.2,
    "decel_stop.slope_window_s": 0.2,
}


def _tmp_cfg(tmp_path: Path, **extra: object) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
            "output.plot": False,
            **FAST,
            **extra,
        }
    )
    return cfgmod.load_config(path)


def _rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _data(speed: float) -> DriveLogData:
    return DriveLogData(
        ref_speed_kmh=None,
        actual_speed_kmh=speed,
        accel_opening=0.0,
        brake_opening=10.0,
        accel_pos=0,
        brake_pos=950,
        accel_current=0.0,
        brake_current=0.0,
    )


def test_steps_1_2_write_one_log_through_stop_hold(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _tmp_cfg(tmp_path)
    trained: list[Path] = []
    orig_check, orig_search, orig_drive = (
        pcmod.run_pre_drive_check, psmod.run_pedal_search, pdmod.run_pattern_drive
    )

    async def fast_check(hw: hwmod.ResearchHardware, cfg_: object, **kw: object) -> int:
        hw.can.vehicle.brake_gain_kmhs_per_pct = 1.5  # type: ignore[attr-defined]  # 早く止める
        return await orig_check(hw, cfg_, **kw)  # type: ignore[arg-type]

    async def fast_search(hw: object, cfg_: object, **kw: object) -> object:
        return await orig_search(hw, cfg_, window_s=0.6, **kw)  # type: ignore[arg-type]

    async def short_drive(hw: object, cfg_: object, **kw: object) -> object:
        return await orig_drive(  # type: ignore[arg-type]
            hw, cfg_, patterns=SHORT_PATTERNS, loop_config=FAST_LOOP, **kw
        )

    def fake_build(cfg_: object, csv_path: Path, **kw: object) -> None:
        trained.append(csv_path)

    monkeypatch.setattr(mainmod, "run_pre_drive_check", fast_check)
    monkeypatch.setattr(mainmod, "run_pedal_search", fast_search)
    monkeypatch.setattr(mainmod, "run_pattern_drive", short_drive)
    monkeypatch.setattr(mainmod, "build_ff_model", fake_build)

    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 0

    logs = list((tmp_path / "results").glob("drive_log_stub_*.csv"))
    assert len(logs) == 1
    assert trained == logs  # 保存した CSV でモデルを作る
    rows = _rows(logs[0])
    order: list[str] = []
    for row in rows:
        if not order or order[-1] != row["section"]:
            order.append(row["section"])
    assert order == [
        dlmod.SECTION_PRE_DRIVE_CHECK,
        dlmod.SECTION_PEDAL_SEARCH,
        dlmod.SECTION_PATTERN_DRIVE,
        dlmod.SECTION_DECEL_TO_STOP,
    ]
    elapsed = [float(r["elapsed_s"]) for r in rows]
    assert elapsed == sorted(elapsed)
    phases = {(r["section"], r["phase"]) for r in rows}
    assert (dlmod.SECTION_PRE_DRIVE_CHECK, pcmod.PHASE_BRAKE_STEP) in phases
    assert (dlmod.SECTION_PEDAL_SEARCH, psmod.PHASE_BRAKE_SEARCH) in phases
    assert all(r["pattern"] for r in rows if r["section"] == dlmod.SECTION_PATTERN_DRIVE)
    assert "手順 2 完了" in capsys.readouterr().out


def test_log_is_saved_after_home_return_on_drive_error(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _tmp_cfg(tmp_path)

    async def fake_check(hw: object, cfg_: object, **kw: object) -> int:
        await asyncio.sleep(0.35)  # サンプラーが数行記録する
        return 0

    async def failing_search(hw: object, cfg_: object, **kw: object) -> None:
        raise hwmod.DriveError("テスト用の探索失敗")

    monkeypatch.setattr(mainmod, "run_pre_drive_check", fake_check)
    monkeypatch.setattr(mainmod, "run_pedal_search", failing_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 5

    logs = list((tmp_path / "results").glob("drive_log_stub_*.csv"))
    assert len(logs) == 1
    rows = _rows(logs[0])
    assert rows and {r["section"] for r in rows} == {dlmod.SECTION_PRE_DRIVE_CHECK}
    out = capsys.readouterr().out
    assert out.index("終了処理: 完了") < out.index("走行ログ CSV")  # ペダルを離してから保存


def test_read_drive_logs_uses_only_pattern_drive_rows(tmp_path: Path) -> None:
    samples = [
        dlmod.DriveSample(elapsed_s=i * 0.1, timestamp=datetime.now(tz=UTC),
                          data=_data(float(i)), section=section, phase="X")
        for i, section in enumerate([
            dlmod.SECTION_PRE_DRIVE_CHECK,
            dlmod.SECTION_PATTERN_DRIVE,
            dlmod.SECTION_PATTERN_DRIVE,
            dlmod.SECTION_DECEL_TO_STOP,
        ])
    ]
    path = tmp_path / "log.csv"
    dlmod.write_csv(samples, path)

    logs = dlmod.read_drive_logs(path)
    assert [log.actual_speed_kmh for log in logs] == [1.0, 2.0]
    assert [log.id for log in logs] == [0, 1]


def test_read_drive_logs_label_actual_uses_pnow_and_skips_unread_rows(tmp_path: Path) -> None:
    """D1: label="actual" は実開度（PNOW 由来）をラベルにし、読めなかった行は除外する。"""
    from tests.research.vehicle import opening_to_pulse

    def _cmd(speed: float, accel_cmd: float, brake_cmd: float) -> DriveLogData:
        return DriveLogData(
            ref_speed_kmh=None, actual_speed_kmh=speed,
            accel_opening=accel_cmd, brake_opening=brake_cmd,
            accel_pos=opening_to_pulse(accel_cmd), brake_pos=opening_to_pulse(brake_cmd),
            accel_current=0.0, brake_current=0.0,
        )

    monitor = AxisMonitor(
        position_pulse=opening_to_pulse(12.0), current_ma=0.0, alarm_code=0,
        servo_on=True, moving=False, pos_done=True,
    )
    samples = [
        # 実開度が指令とずれている行（まとめ読みあり）
        dlmod.DriveSample(
            elapsed_s=0.0, timestamp=datetime.now(tz=UTC), data=_cmd(10.0, 20.0, 0.0),
            section=dlmod.SECTION_PATTERN_DRIVE, phase="ACCEL",
            monitor_accel=monitor, monitor_brake=monitor,
        ),
        # まとめ読みが無い行（読めなかった） → label="actual" では除外
        dlmod.DriveSample(
            elapsed_s=0.1, timestamp=datetime.now(tz=UTC), data=_cmd(11.0, 30.0, 0.0),
            section=dlmod.SECTION_PATTERN_DRIVE, phase="ACCEL",
        ),
    ]
    path = tmp_path / "log.csv"
    dlmod.write_csv(samples, path)

    cmd_logs = dlmod.read_drive_logs(path, label="cmd")
    assert [log.accel_opening for log in cmd_logs] == pytest.approx([20.0, 30.0], abs=1e-2)

    actual_logs = dlmod.read_drive_logs(path, label="actual")
    assert len(actual_logs) == 1  # まとめ読みの無い行は除外
    assert actual_logs[0].accel_opening == pytest.approx(12.0, abs=1e-2)


def test_write_csv_includes_governor_and_alarm_columns(tmp_path: Path) -> None:
    """2026-09-13 手順3 のブレーキ脱落を後から気づけるよう、Gガバナーとアラームも CSV に残す。"""
    samples = [
        dlmod.DriveSample(
            elapsed_s=0.0, timestamp=datetime.now(tz=UTC), data=_data(10.0),
            section=dlmod.SECTION_MODE_DRIVE, phase="ACCEL",
            governor_active=False, alarm_accel=False, alarm_brake=False,
        ),
        dlmod.DriveSample(
            elapsed_s=0.1, timestamp=datetime.now(tz=UTC), data=_data(9.0),
            section=dlmod.SECTION_MODE_DRIVE, phase="BRAKE_GOV",
            governor_active=True, alarm_accel=False, alarm_brake=False,
        ),
        dlmod.DriveSample(
            elapsed_s=0.2, timestamp=datetime.now(tz=UTC), data=_data(8.0),
            section=dlmod.SECTION_MODE_DRIVE, phase="BRAKE",
            governor_active=False, alarm_accel=False, alarm_brake=True,
        ),
        # 走行前チェックなど、アラーム未確認の区間は空欄のまま（既定値）
        dlmod.DriveSample(
            elapsed_s=0.3, timestamp=datetime.now(tz=UTC), data=_data(0.0),
            section=dlmod.SECTION_PRE_DRIVE_CHECK, phase="PRE_CHECK",
        ),
    ]
    path = tmp_path / "log.csv"
    dlmod.write_csv(samples, path)

    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert {"governor_active", "alarm_accel", "alarm_brake"} <= set(rows[0].keys())
    assert [r["governor_active"] for r in rows] == ["0", "1", "0", "0"]
    assert [r["alarm_accel"] for r in rows] == ["0", "0", "0", ""]
    assert [r["alarm_brake"] for r in rows] == ["0", "0", "1", ""]


def test_read_drive_logs_reads_all_rows_of_legacy_csv(tmp_path: Path) -> None:
    """section 列の無い旧形式（pattern_drive_*.csv）は全行を読む。"""
    path = tmp_path / "legacy.csv"
    # 旧形式の列（section・mode_time_s・deviation_kmh・effort 列を追加する前）
    columns = [
        "timestamp", "elapsed_s", "ref_speed_kmh", "actual_speed_kmh", "accel_opening",
        "brake_opening", "accel_pos", "brake_pos", "accel_current", "brake_current",
        "pattern", "phase",
    ]
    lines = [",".join(columns)] + [
        f"2026-09-11 06:00:00,{i * 0.1:.3f},,{i}.0,0.0,0.0,0,0,0.0,0.0,1:CREEP,DRIVE"
        for i in range(3)
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    assert len(dlmod.read_drive_logs(path)) == 3


async def test_close_saves_png_when_plot_enabled(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"output.plot": True})
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    log = dlmod.SessionLog(cfg, hw)
    log.start(dlmod.SECTION_PRE_DRIVE_CHECK, "PRE_CHECK")
    await asyncio.sleep(0.35)

    assert await log.close() == log.csv_path
    assert log.samples
    assert log.figure_path is not None and log.figure_path.stat().st_size > 1000
    assert await log.close() == log.csv_path  # 2 回目は何もしない
    await hwmod.shutdown(hw)


async def test_close_returns_while_sampler_is_in_modbus_read(tmp_path: Path) -> None:
    """実機で手順 2 の停車保持の後に close() が戻らなかった件の再発防止。

    pymodbus の execute() は応答待ち中に CancelledError を受けると ModbusIOException に変えて
    投げ直す。読み取り中のサンプラーを cancel で止めると、_read() がその例外を握って
    サンプラーが回り続け、close() が戻らなかった。
    """
    cfg = _tmp_cfg(tmp_path)
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    original = hw.accel.read_monitor

    async def modbus_like_read() -> AxisMonitor:
        try:
            await asyncio.sleep(0.3)  # 応答待ち
        except asyncio.CancelledError as exc:
            raise OSError("Request cancelled outside library.") from exc  # pymodbus と同じ
        return await original()

    hw.accel.read_monitor = modbus_like_read  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw)
    log.start(dlmod.SECTION_DECEL_TO_STOP, "STOP_HOLD")
    await asyncio.sleep(0.5)  # サンプラーは読み取りの途中

    assert await asyncio.wait_for(log.close(), timeout=3.0) == log.csv_path
    assert log.samples[-1].section == dlmod.SECTION_DECEL_TO_STOP
    await hwmod.shutdown(hw)


# ── A7: 手順 1 から共通の列 ─────────────────────────────────────────────


def _monitor(pos: int, current: float = 120.0, **kw: object) -> AxisMonitor:
    fields: dict[str, object] = {
        "alarm_code": 0, "servo_on": True, "moving": False, "pos_done": True, **kw
    }
    return AxisMonitor(position_pulse=pos, current_ma=current, **fields)  # type: ignore[arg-type]


def test_csv_columns_replace_signed_effort_and_split_command_actual() -> None:
    cols = set(dlmod.CSV_COLUMNS)
    assert len(cols) == len(dlmod.CSV_COLUMNS)
    removed = {"ff_effort_pct", "pid_effort_pct", "effort_pct",
               "accel_opening", "brake_opening", "accel_pos", "brake_pos"}
    assert not cols & removed
    for axis in ("accel", "brake"):
        assert {
            f"{axis}_ff_pct", f"{axis}_cmd_pct", f"{axis}_cmd_mm", f"{axis}_actual_pct",
            f"{axis}_actual_mm", f"{axis}_current", f"{axis}_servo_on", f"{axis}_moving",
            f"{axis}_pos_done", f"{axis}_alarm_code", f"alarm_{axis}",
        } <= cols
    assert {"cycle_ms", "mode_time_s", "deviation_kmh", "section", "pattern", "phase",
            "governor_active"} <= cols


def test_write_csv_writes_ff_command_actual_and_status(tmp_path: Path) -> None:
    data = DriveLogData(
        ref_speed_kmh=30.0, actual_speed_kmh=29.5, accel_opening=12.5, brake_opening=5.0,
        accel_pos=1188, brake_pos=475, accel_current=0.0, brake_current=0.0,
    )
    samples = [
        dlmod.DriveSample(
            elapsed_s=1.0, timestamp=datetime.now(tz=UTC), data=data,
            section=dlmod.SECTION_MODE_DRIVE, phase="ACCEL", pattern="Low", mode_time_s=0.5,
            accel_ff_pct=12.5, brake_ff_pct=0.0,
            monitor_accel=_monitor(950, current=310.0, moving=True, pos_done=False),
            monitor_brake=_monitor(475, alarm_code=0x0E8), cycle_ms=23.46,
        ),
        # 読み取りに失敗した行（モニター無し）は実開度・ステータスが空欄
        dlmod.DriveSample(
            elapsed_s=1.1, timestamp=datetime.now(tz=UTC), data=_data(0.0),
            section=dlmod.SECTION_PRE_DRIVE_CHECK, phase="PRE_CHECK",
        ),
    ]
    path = tmp_path / "log.csv"
    dlmod.write_csv(samples, path)

    rows = _rows(path)
    assert list(rows[0].keys()) == list(dlmod.CSV_COLUMNS)
    r = rows[0]
    assert (r["cycle_ms"], r["accel_ff_pct"], r["brake_ff_pct"]) == ("23.5", "12.500", "0.000")
    assert (r["accel_cmd_pct"], r["accel_cmd_mm"]) == ("12.500", "11.88")
    assert r["brake_cmd_mm"] == "4.75"
    assert (r["accel_actual_pct"], r["accel_actual_mm"]) == ("10.000", "9.50")
    assert r["accel_current"] == "0.0"  # 電流は DriveLogData の値（呼び出し側がモニターから入れる）
    assert (r["accel_servo_on"], r["accel_moving"], r["accel_pos_done"]) == ("1", "1", "0")
    assert (r["accel_alarm_code"], r["brake_alarm_code"]) == ("0", str(0x0E8))
    blank = rows[1]
    assert blank["accel_cmd_pct"] == "0.000" and blank["brake_cmd_mm"] == "9.50"
    assert all(blank[k] == "" for k in (
        "cycle_ms", "accel_ff_pct", "accel_actual_pct", "brake_actual_mm", "brake_servo_on",
        "brake_alarm_code",
    ))


def test_readers_accept_new_and_legacy_columns(tmp_path: Path) -> None:
    new = {"accel_cmd_pct": "12.5", "accel_cmd_mm": "11.88", "accel_actual_pct": "10.0",
           "accel_ff_pct": "0.000", "brake_ff_pct": "8.250"}
    old = {"accel_opening": "7.0", "accel_pos": "665", "ff_effort_pct": "-3.5",
           "pid_effort_pct": "0.2", "effort_pct": "-3.3"}
    assert dlmod.cmd_opening(new, "accel") == 12.5
    assert dlmod.cmd_pulse(new, "accel") == 1188
    assert dlmod.actual_opening(new, "accel") == 10.0
    assert dlmod.ff_effort(new) == pytest.approx(-8.25)
    assert dlmod.pid_effort(new) is None and dlmod.total_effort(new) is None
    assert dlmod.ff_effort({"accel_ff_pct": "", "brake_ff_pct": ""}) is None  # モード走行以外
    assert dlmod.cmd_opening(old, "accel") == 7.0
    assert dlmod.cmd_pulse(old, "accel") == 665
    assert dlmod.actual_opening(old, "accel") is None
    assert dlmod.ff_effort(old) == -3.5
    assert (dlmod.pid_effort(old), dlmod.total_effort(old)) == (0.2, -3.3)

    # 書き出し → 学習用の読み取りで、指令の開度・位置が戻る（学習ラベルは指令のまま）
    sample = dlmod.DriveSample(
        elapsed_s=0.0, timestamp=datetime.now(tz=UTC), data=_data(5.0),
        section=dlmod.SECTION_PATTERN_DRIVE, phase="X", monitor_brake=_monitor(900),
    )
    path = tmp_path / "log.csv"
    dlmod.write_csv([sample], path)
    (log,) = dlmod.read_drive_logs(path)
    assert (log.brake_opening, log.brake_pos) == (10.0, 950)


async def test_sampler_rows_have_command_actual_and_status(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    await hw.brake.move_to_position(1900)
    log = dlmod.SessionLog(cfg, hw)
    log.start(dlmod.SECTION_PRE_DRIVE_CHECK, "PRE_CHECK")
    await asyncio.sleep(0.35)
    await log.close()
    await hwmod.shutdown(hw)

    rows = _rows(log.csv_path)
    assert rows
    for r in rows:
        assert r["brake_cmd_mm"] == r["brake_actual_mm"] == "19.00"
        assert r["accel_cmd_mm"] == "0.00"
        assert r["brake_cmd_pct"] == r["brake_actual_pct"] == "20.000"
        assert (r["brake_servo_on"], r["brake_pos_done"], r["brake_alarm_code"]) == ("1", "1", "0")
        assert float(r["cycle_ms"]) >= 0.0
        assert r["accel_ff_pct"] == ""  # FF はモード走行だけ
