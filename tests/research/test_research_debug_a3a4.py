"""debug_a3a4（A3・A4 の手順 2 を A2・A5 と比べる解析）の純粋関数のテスト。

車両にも DB にも触らない。
"""

from __future__ import annotations

import csv
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import debug_a3a4 as mod

COLUMNS = ("elapsed_s", "section", "pattern", "phase", "actual_speed_kmh", "accel_opening",
           "brake_opening", "governor_active")
CAP = 137.2


def _write(path: Path, rows: list[tuple[float, str, str, float, float, float, int]]) -> Path:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for t, pattern, phase, v, a, b, g in rows:
            w.writerow([t, "PATTERN_DRIVE", pattern, phase, v, a, b, g])
    return path


def test_a3a4_patterns_follow_config_order() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    added = mod.a3a4_patterns(cfg)
    assert added.hard_holds == ("30:BRAKE_HOLD", "31:BRAKE_HOLD", "32:BRAKE_HOLD", "33:BRAKE_HOLD")
    assert added.stairs == ("39:CRUISE_TRIM", "40:CRUISE_TRIM", "41:CRUISE_TRIM")
    db = cfg.feedforward.accel_deadband_pct
    assert added.stair_steps_pct == pytest.approx((db + 8.0, db + 5.0, db + 2.0))


def test_stair_and_hard_hold_stats(tmp_path: Path) -> None:
    rows = []
    t = 0.0

    def add(pattern: str, phase: str, v: float, a: float, b: float, g: int = 0) -> None:
        nonlocal t
        rows.append((t, pattern, phase, v, a, b, g))
        t += 0.1

    # トリム階段: 120 km/h まで加速 → 18% が cap で 2s → 15% を 8s → 12% を 8s → 停車復帰 3s
    for i in range(20):
        add("39:CRUISE_TRIM", "DRIVE_ACCEL", i * 6.0 + 6.0, 70.0, 0.0)
    for i in range(20):
        add("39:CRUISE_TRIM", "CRUISE_TRIM", 130.0 + i * 0.4, 18.0, 0.0)  # 最後の行 137.6 ≥ cap
    for i in range(80):
        add("39:CRUISE_TRIM", "CRUISE_TRIM", 130.0 - i * 0.2, 15.0, 0.0)
    for i in range(80):
        add("39:CRUISE_TRIM", "CRUISE_TRIM", 114.0 - i * 0.2, 12.0, 0.0)
    for i in range(30):
        add("39:CRUISE_TRIM", "DRIVE_BRAKE", 98.0 - i * 3.0, 0.0, 30.0)
    # 20 km/h から 50% で停車: 加速 5s → 保持 3s（ガバナー 5 行）
    for i in range(50):
        add("33:BRAKE_HOLD", "DRIVE_ACCEL", i * 0.44, 18.0, 0.0)
    for i in range(30):
        add("33:BRAKE_HOLD", "BRAKE_HOLD", max(0.0, 22.0 - i * 0.8), 0.0,
            min(50.0, 10.0 * (i + 1)), int(10 <= i < 15))
    data = mod.read_pattern_rows(_write(tmp_path / "log.csv", rows))

    (stair,) = mod.stair_stats(
        data, ["39:CRUISE_TRIM", "40:CRUISE_TRIM"], accel_deadband_pct=10.0, step_s=8.0,
        cap_kmh=CAP, max_speed_kmh=140.0, stop_kmh=5.0,
    )  # 無い段は飛ばす
    assert stair.v_accel_end == pytest.approx(120.0)
    assert stair.stop_return_s == pytest.approx(2.9)  # 最初と最後の行の差
    assert not stair.overspeed
    assert [(s.opening, s.reason) for s in stair.steps] == [
        (18.0, mod.REASON_CAP), (15.0, mod.REASON_STEP), (12.0, mod.REASON_STEP)
    ]
    assert stair.steps[0].hold_s == pytest.approx(2.0)
    assert stair.steps[1].band_rows == {120: 51, 100: 29}
    assert "| 39:CRUISE_TRIM | 1 | 18.0 | 2.0 |" in mod.step_table([stair])
    assert "120〜140: 51 / 100〜120: 29" in mod.step_table([stair])
    assert "39:CRUISE_TRIM" in mod.stair_table([stair])

    (hold,) = mod.hard_hold_stats(data, ["33:BRAKE_HOLD"], brake_deadband_pct=13.16)
    assert hold.v_accel_end == pytest.approx(21.56)
    assert hold.v_hold_start == pytest.approx(22.0)
    assert hold.hold_s == pytest.approx(2.9)
    assert hold.v_hold_end == 0.0
    assert hold.effective_rows == 29  # 最初の 1 行（10%）は不感帯未満
    assert hold.hard_rows == 27  # 20 km/h 未満 × 40% 以上（i = 3〜29）
    assert hold.governor_rows == 5
    assert "| ○ |" in mod.hard_hold_table([hold])


@pytest.mark.parametrize(
    ("v_end", "hold_s", "reason"),
    [(141.0, 1.0, mod.REASON_OVERSPEED), (4.9, 3.0, mod.REASON_SLOW),
     (CAP, 1.0, mod.REASON_CAP), (CAP, 8.0, mod.REASON_STEP), (100.0, 8.0, mod.REASON_STEP)],
)
def test_step_reason(v_end: float, hold_s: float, reason: str) -> None:
    run = [mod.Row(t=0.0, pattern="39:CRUISE_TRIM", phase="CRUISE_TRIM", v=v_end, accel=12.0,
                   brake=0.0, governor=False)]
    assert mod._step_reason(run, hold_s, step_s=8.0, cap_kmh=CAP, max_speed_kmh=140.0,
                            stop_kmh=5.0) == reason


def test_a3a4_gate_table() -> None:
    table = mod.a3a4_gate_table(span_s=1030.0, timeout_s=1200.0, holes_old=(8, 2),
                                holes_new=(3, 2), v_max=139.0, max_speed_kmh=140.0)
    lines = table.splitlines()
    assert "8 → 3 個" in lines[2] and lines[2].endswith("| ○ |")
    assert lines[3].endswith("| × |")  # ブレーキの空白は減っていない
    assert lines[4].endswith("| ○ |")
    assert "1200s に対し +170.0s" in lines[5] and lines[5].endswith("| ○ |")
    assert "+130.0s" in lines[6] and "見積り 1020s との差 +10.0s" in lines[6]
    assert lines[6].endswith("| 記録 |")
