"""debug_a2a5（A2・A5 の手順 2 を A6 と比べる解析）の純粋関数のテスト。車両にも DB にも触らない。"""

from __future__ import annotations

import csv
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import debug_a2a5 as mod

COLUMNS = ("elapsed_s", "section", "pattern", "phase", "actual_speed_kmh", "accel_opening",
           "brake_opening", "governor_active")


def _write(path: Path, rows: list[tuple[float, str, str, float, float, float, int]]) -> Path:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for t, pattern, phase, v, a, b, g in rows:
            w.writerow([t, "PATTERN_DRIVE", pattern, phase, v, a, b, g])
    return path


def test_added_patterns_follow_config_order() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    added = mod.added_patterns(cfg)
    assert added.sweeps == ("12:ACCEL_SWEEP", "13:ACCEL_SWEEP", "14:ACCEL_SWEEP")
    assert added.low_holds == ("27:BRAKE_HOLD", "28:BRAKE_HOLD", "29:BRAKE_HOLD")


def test_sweep_and_low_hold_stats(tmp_path: Path) -> None:
    rows = []
    t = 0.0
    # 12% の ACCEL_SWEEP: 10s 加速して 40 km/h で一定 → 停車復帰 2s
    for i in range(101):
        rows.append((t, "12:ACCEL_SWEEP", "DRIVE_ACCEL", min(40.0, i * 0.8), 12.0, 0.0, 0))
        t += 0.1
    for i in range(21):
        rows.append((t, "12:ACCEL_SWEEP", "DRIVE_BRAKE", 40.0 - i * 2.0, 0.0, 30.0, 0))
        t += 0.1
    # 60 km/h からの BRAKE_HOLD: 加速 → 保持 6s で停車
    for i in range(51):
        rows.append((t, "27:BRAKE_HOLD", "DRIVE_ACCEL", i * 1.2, 70.0, 0.0, int(i < 20)))
        t += 0.1
    for i in range(61):
        rows.append((t, "27:BRAKE_HOLD", "BRAKE_HOLD", max(0.0, 62.0 - i * 1.1),
                     0.0, 12.0 if i < 5 else 14.0, 0))
        t += 0.1
    data = mod.read_pattern_rows(_write(tmp_path / "log.csv", rows))

    (sweep,) = mod.sweep_stats(data, ["12:ACCEL_SWEEP", "99:ACCEL_SWEEP"])  # 無い段は飛ばす
    assert sweep.opening == 12.0
    assert sweep.accel_s == pytest.approx(10.0)
    assert sweep.v_end == pytest.approx(40.0)
    assert sweep.v_change_end == pytest.approx(0.0)  # 終わり 5s は一定
    assert sweep.stop_return_s == pytest.approx(2.0)
    assert "12:ACCEL_SWEEP" in mod.sweep_table([sweep])

    (hold,) = mod.low_hold_stats(data, ["27:BRAKE_HOLD"], brake_deadband_pct=13.16)
    assert hold.v_hold_start == pytest.approx(62.0)
    assert hold.hold_s == pytest.approx(6.0)
    assert hold.v_hold_end == 0.0
    assert hold.effective_rows == 56  # 最初の 5 行は不感帯未満
    assert "| ○ |" in mod.low_hold_table([hold])


def test_hole_count_and_gate_table() -> None:
    grid = [[(0.5, 0), (2.0, 0), (20.0, 49)], [(20.0, 50), (0.0, 3), (1.0, 1)]]
    assert mod.hole_count(grid) == 2  # 要る 1s 以上で 0 行 / 要る 10s 以上で 50 行未満
    table = mod.a2a5_gate_table(span_s=880.0, holes_old=(8, 6), holes_new=(5, 6),
                                v_max=139.4, max_speed_kmh=140.0)
    lines = table.splitlines()
    assert lines[2].endswith("| ○ |")  # 880s ≤ 900s
    assert "8 → 5 個" in lines[3] and lines[3].endswith("| ○ |")
    assert lines[4].endswith("| × |")  # ブレーキの空白は減っていない
    assert lines[5].endswith("| ○ |")
