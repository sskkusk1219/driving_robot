"""debug_a6（A6 の手順 2 を前の手順 2 と比べる解析）の純粋関数のテスト。車両には触らない。"""

from __future__ import annotations

import csv
from pathlib import Path

import pytest

from tests.research import debug_a6 as a6

COLUMNS = ("elapsed_s", "section", "pattern", "phase", "actual_speed_kmh", "accel_opening",
           "brake_opening", "governor_active")
LogRow = tuple[float, str, str, float, float, float, int]


def _write(path: Path, rows: list[LogRow]) -> Path:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        w.writerow([0.0, "PRE_DRIVE_CHECK", "", "PRE_CHECK", 0.0, 0.0, 0.0, 0])  # 読み飛ばす
        for t, pattern, phase, v, a, b, g in rows:
            w.writerow([t, "PATTERN_DRIVE", pattern, phase, v, a, b, g])
    return path


def _sweep(t0: float, name: str, *, latched: bool) -> list[LogRow]:
    rows = []
    t = t0
    for v, a in ((0.0, 20.0), (30.0, 40.0), (80.0, 40.0), (130.0, 40.0)):
        rows.append((t, name, "DRIVE_ACCEL", v, a, 0.0, 0))
        t += 0.1
    brake = (30.0, 20.0, 8.0, 8.0) if latched else (30.0, 26.0, 28.0, 30.0)
    speeds = (130.0, 100.0, 80.0, 60.0) if latched else (130.0, 60.0, 10.0, 0.0)
    for v, b in zip(speeds, brake, strict=True):
        rows.append((t, name, "DRIVE_BRAKE", v, 0.0, b, int(b < 30.0 and not latched)))
        t += 0.1
    return rows


def test_gates_and_governor_stats(tmp_path: Path) -> None:
    rows = [(0.0, "1:CREEP", "MEASURE", 0.0, 0.0, 20.0, 0),
            *_sweep(0.1, "2:ACCEL_SWEEP", latched=True),
            *_sweep(1.0, "3:ACCEL_SWEEP", latched=False)]
    data = a6.read_pattern_rows(_write(tmp_path / "log.csv", rows))
    assert len(data) == len(rows)

    gates = a6.pattern_gates(data)
    # CREEP は関門に入れない
    assert [g.pattern for g in gates] == ["2:ACCEL_SWEEP", "3:ACCEL_SWEEP"]
    assert gates[0].v_accel_end == 130.0 and gates[0].v_end == 60.0
    assert gates[1].v_end == 0.0
    summary = a6.gate_summary(data, gates, 900.0)
    assert "1 / 2 本" in summary  # 停車して終わったのは 3 だけ

    stats = {(s.kind, s.phase): s for s in a6.governor_stats(data)}
    brake = stats[("ACCEL_SWEEP", "DRIVE_BRAKE")]
    assert brake.count == 2
    assert brake.ended_lowered == 1  # 2 は 8% のまま終わった、3 は 30% に戻した
    assert brake.min_lowered == pytest.approx(8.0)
    assert brake.governor_s == pytest.approx(0.2)
    assert stats[("ACCEL_SWEEP", "DRIVE_ACCEL")].min_lowered is None


def test_effective_row_bins_count_lowered_rows(tmp_path: Path) -> None:
    log = _write(tmp_path / "log.csv", _sweep(0.0, "1:ACCEL_SWEEP", latched=True))
    data = a6.read_pattern_rows(log)
    brake = a6.effective_row_bins(data, 13.0, accel=False)
    assert brake == {20: (1, 1), 30: (1, 0)}  # 20% は下げた行、8% は不感帯未満で数えない
    accel = a6.effective_row_bins(data, 10.0, accel=True)
    assert accel == {20: (1, 0), 40: (3, 0)}
    table = a6.rows_table(brake, brake, "ブレーキ")
    assert "**合計**" in table


def test_figure_is_written(tmp_path: Path) -> None:
    log = _write(tmp_path / "log.csv", _sweep(0.0, "1:ACCEL_SWEEP", latched=True))
    data = a6.read_pattern_rows(log)
    path = a6.fig_compare(tmp_path / "out", data, data)
    assert path.exists()
