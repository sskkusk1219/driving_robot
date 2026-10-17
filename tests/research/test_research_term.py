"""ターミナル表示（term.drive_status_line）のテスト。"""

from __future__ import annotations

from tests.research.term import drive_status_line


def test_status_line_shows_per_pedal_gains_and_short_pedal_labels() -> None:
    line = drive_status_line(
        t_s=1511.7, ref_kmh=12.34, actual_kmh=12.10,
        accel_kp=3.6, accel_ki=0.45, accel_kd=0.0,
        brake_kp=0.5, brake_ki=0.0625, brake_kd=0.0,
        accel_pct=4.2, brake_pct=8.5, extra="11/11 G_RAMP RAMP_UP",
    )
    assert line == (
        "t=1511.7s 基準= 12.34 実= 12.10 偏差= -0.24 km/h "
        "acc Kp=3.6 Ki=0.45 Kd=0 brk Kp=0.5 Ki=0.0625 Kd=0 "
        "| acc=  4.2% brk=  8.5% | 11/11 G_RAMP RAMP_UP"
    )


def test_status_line_without_reference_uses_dashes_and_rounds_gains_to_three_digits() -> None:
    line = drive_status_line(
        t_s=3.0, ref_kmh=None, actual_kmh=5.0,
        accel_kp=1.23456, accel_ki=0.0, accel_kd=0.0,
        brake_kp=0.0, brake_ki=0.012345, brake_kd=0.0,
        accel_pct=0.0, brake_pct=0.0,
    )
    assert "基準=   ---" in line and "偏差=   ---" in line
    assert "acc Kp=1.23 Ki=0 Kd=0 brk Kp=0 Ki=0.0123 Kd=0" in line
