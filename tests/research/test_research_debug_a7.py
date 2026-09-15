"""debug_a7（A7 の走行ログの確かめ）のユニットテスト。合成 CSV で表と関門を確かめる。"""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

import pytest

from src.models.drive_log import DriveLogData
from tests.research import debug_a7 as a7
from tests.research import drive_log as dlmod
from tests.research.axis_monitor import AxisMonitor
from tests.research.vehicle import opening_to_pulse


def _sample(
    t: float,
    *,
    brake_cmd: float,
    brake_actual: float | None,
    section: str = dlmod.SECTION_PATTERN_DRIVE,
    cycle_ms: float | None = 30.0,
    alarm_code: int = 0,
    servo_on: bool = True,
) -> dlmod.DriveSample:
    data = DriveLogData(
        ref_speed_kmh=None, actual_speed_kmh=40.0, accel_opening=0.0, brake_opening=brake_cmd,
        accel_pos=0, brake_pos=opening_to_pulse(brake_cmd), accel_current=0.0, brake_current=0.0,
    )

    def monitor(pct: float) -> AxisMonitor:
        return AxisMonitor(position_pulse=opening_to_pulse(pct), current_ma=100.0,
                           alarm_code=alarm_code, servo_on=servo_on,
                           moving=abs(pct - brake_cmd) > 0.01, pos_done=True)

    return dlmod.DriveSample(
        elapsed_s=t, timestamp=datetime.now(tz=UTC), data=data, section=section,
        phase="BRAKE_HOLD", pattern="30:BRAKE_HOLD",
        monitor_accel=monitor(0.0) if brake_actual is not None else None,
        monitor_brake=monitor(brake_actual) if brake_actual is not None else None,
        cycle_ms=cycle_ms,
    )


def _csv(tmp_path: Path) -> Path:
    # ブレーキ指令が t=0.3 で 0 → 20%。実開度は 10%（t=0.3）→ 19.8%（t=0.5）で追いつく
    samples = [
        _sample(0.0, brake_cmd=0.0, brake_actual=0.0, cycle_ms=20.0),
        _sample(0.1, brake_cmd=0.0, brake_actual=0.0, cycle_ms=25.0),
        _sample(0.2, brake_cmd=0.0, brake_actual=0.0, cycle_ms=30.0),
        _sample(0.3, brake_cmd=20.0, brake_actual=10.0, cycle_ms=40.0),
        _sample(0.4, brake_cmd=20.0, brake_actual=18.0, cycle_ms=120.0),
        _sample(0.5, brake_cmd=20.0, brake_actual=19.8, cycle_ms=35.0),
        _sample(0.6, brake_cmd=20.0, brake_actual=None, cycle_ms=None),  # 読み取り失敗
        # モード走行: 指令 5 → 0%（戻し）に追いつかないまま次の指令が来る
        _sample(10.0, brake_cmd=5.0, brake_actual=5.0, section=dlmod.SECTION_MODE_DRIVE),
        _sample(10.1, brake_cmd=0.0, brake_actual=4.0, section=dlmod.SECTION_MODE_DRIVE,
                alarm_code=0x0E8),
        _sample(10.2, brake_cmd=3.0, brake_actual=3.0, section=dlmod.SECTION_MODE_DRIVE,
                servo_on=False),
    ]
    path = tmp_path / "drive_log_stub_20260913_000000.csv"
    dlmod.write_csv(samples, path)
    return path


def test_cycle_stats_and_gate(tmp_path: Path) -> None:
    rows = a7.read_rows(_csv(tmp_path))
    periods = a7.section_periods_ms(0.05, 0.1)
    assert periods[dlmod.SECTION_PATTERN_DRIVE] == pytest.approx(100.0)
    assert periods[dlmod.SECTION_MODE_DRIVE] == pytest.approx(50.0)
    cycles = {c.section: c for c in a7.cycle_stats(rows, periods)}
    pattern = cycles[dlmod.SECTION_PATTERN_DRIVE]
    assert (pattern.rows, len(pattern.measured), pattern.over) == (7, 6, 1)
    statuses = a7.status_stats(rows)
    gate = a7.a7_gate_table(list(cycles.values()), statuses)
    # パターン走行 p95 = 120ms > 100ms で ×、モード走行 30ms ≤ 50ms で ○
    assert "| PATTERN_DRIVE の 1 周期の処理時間 p95 ≤ 100ms | p95 120.0 ms" in gate
    assert "（超えた行 1） | × |" in gate
    assert "| MODE_DRIVE の 1 周期の処理時間 p95 ≤ 50ms | p95 30.0 ms・最大 30.0 ms" in gate
    assert "（超えた行 0） | ○ |" in gate
    assert "| 実開度・ステータスが読めなかった行（全区間・両軸） | 2 | 記録 |" in gate
    assert "| アラームコードが 0 以外の行（全区間・両軸） | 2 | × |" in gate


def test_tracking_and_lag(tmp_path: Path) -> None:
    rows = a7.read_rows(_csv(tmp_path))
    stats = {(s.section, s.axis, s.motion): s for s in a7.tracking_stats(rows)}
    press = stats[(dlmod.SECTION_PATTERN_DRIVE, "brake", a7.MOTION_PRESS)]
    assert press.errors == pytest.approx([-10.0])
    hold = stats[(dlmod.SECTION_PATTERN_DRIVE, "brake", a7.MOTION_HOLD)]
    assert hold.errors == pytest.approx([0.0, 0.0, -2.0, -0.2], abs=0.02)

    steps = a7.find_steps(rows)
    brake_steps = [s for s in steps if s.axis == "brake"]
    assert [(s.section, s.cmd_from, s.cmd_to) for s in brake_steps] == [
        (dlmod.SECTION_PATTERN_DRIVE, 0.0, 20.0),
        (dlmod.SECTION_MODE_DRIVE, 5.0, 0.0),
        (dlmod.SECTION_MODE_DRIVE, 0.0, 3.0),
    ]
    assert brake_steps[0].lag_s == pytest.approx(0.2)
    assert brake_steps[1].lag_s is None  # 追いつく前に次の指令
    assert brake_steps[2].lag_s == pytest.approx(0.0)
    table = a7.lag_table(steps)
    assert "| PATTERN_DRIVE | ブレーキ | 踏み込み | 1 | 20.0 | 0.20 | 0.20 | 0.20 | 0 |" in table
    assert "| MODE_DRIVE | ブレーキ | 戻し | 1 | 5.0 | — | — | — | 1 |" in table


def test_status_table_counts_servo_off_and_alarm_codes(tmp_path: Path) -> None:
    rows = a7.read_rows(_csv(tmp_path))
    stats = {(s.section, s.axis): s for s in a7.status_stats(rows)}
    mode_brake = stats[(dlmod.SECTION_MODE_DRIVE, "brake")]
    assert (mode_brake.rows, mode_brake.read_rows, mode_brake.servo_off) == (3, 3, 1)
    assert mode_brake.alarm_codes == {0x0E8: 1}
    pattern_brake = stats[(dlmod.SECTION_PATTERN_DRIVE, "brake")]
    assert (pattern_brake.rows, pattern_brake.read_rows, pattern_brake.moving) == (7, 6, 3)
    table = a7.status_table(list(stats.values()))
    assert "0x0E8（1 行）" in table


def test_run_writes_figure(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    path = _csv(tmp_path)
    assert a7.main(["--csv", str(path), "--out", str(tmp_path / "out")]) == 0
    out = capsys.readouterr().out
    assert "### 関門" in out and "### ステータス" in out
    assert (tmp_path / "out" / "a7_tracking.png").stat().st_size > 1000
