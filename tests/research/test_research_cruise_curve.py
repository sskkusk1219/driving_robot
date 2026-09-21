"""cruise_curve（定速階段の保持窓 → 実測テーブル）のユニットテスト。

合成の dict 行（0.1s 刻み。csv.DictReader が返すのと同じ、値はすべて文字列）で
`_advance_cruise_hold`（`pattern_loop.py`）の判定を確かめる。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from tests.research import cruise_curve as cc
from tests.research.config import LearningSection
from tests.research.drive_log import SECTION_PATTERN_DRIVE

# ─────────────────────────────────────────────────────────────────────
# 合成 CSV 行
# ─────────────────────────────────────────────────────────────────────


def _row(
    t: float,
    speed: float,
    *,
    opening: float | None = 0.0,
    phase: str = "CRUISE_HOLD",
    section: str = SECTION_PATTERN_DRIVE,
    pattern: str = "18:CRUISE_TRIM",
) -> dict[str, str]:
    return {
        "section": section,
        "phase": phase,
        "pattern": pattern,
        "elapsed_s": f"{t:.1f}",
        "actual_speed_kmh": f"{speed:.3f}",
        "accel_actual_pct": "" if opening is None else f"{opening:.3f}",
    }


def _constant_speed_rows(
    start_i: int,
    count: int,
    speed: float,
    *,
    opening: float = 12.0,
    pattern: str = "18:CRUISE_TRIM",
) -> list[dict[str, str]]:
    return [
        _row(round((start_i + i) * 0.1, 1), speed, opening=opening, pattern=pattern)
        for i in range(count)
    ]


# ─────────────────────────────────────────────────────────────────────
# extract_hold_steps: 段の再現
# ─────────────────────────────────────────────────────────────────────


def test_extract_hold_steps_reaches_held() -> None:
    learning = LearningSection(
        cruise_hold_speeds_kmh=[30.0],
        cruise_hold_settle_tol_kmh=1.0,
        cruise_hold_settle_s=0.3,
        cruise_hold_hold_s=0.5,
        cruise_hold_step_timeout_s=5.0,
    )
    rows = _constant_speed_rows(0, 12, 30.0)  # t=0.0〜1.1、目標どおりの車速で一定

    steps = cc.extract_hold_steps(rows, learning)

    assert len(steps) == 1
    step = steps[0]
    assert step.target_kmh == 30.0
    assert step.outcome is cc.HoldOutcome.HELD
    # settle_s=0.3 で t=0.3 に保持タイマー開始、hold_s=0.5 で t=0.8 に保持完了
    assert [r["elapsed_s"] for r in step.rows] == ["0.3", "0.4", "0.5", "0.6", "0.7", "0.8"]


def test_extract_hold_steps_times_out_when_never_settles() -> None:
    learning = LearningSection(
        cruise_hold_speeds_kmh=[30.0],
        cruise_hold_settle_tol_kmh=1.0,
        cruise_hold_settle_s=0.3,
        cruise_hold_hold_s=0.5,
        cruise_hold_step_timeout_s=0.4,
    )
    # 目標 30 km/h から大きく外れたまま（許容 ±1.0 km/h の外）
    rows = _constant_speed_rows(0, 8, 10.0)

    steps = cc.extract_hold_steps(rows, learning)

    assert len(steps) == 1
    step = steps[0]
    assert step.outcome is cc.HoldOutcome.TIMED_OUT
    assert step.rows == ()  # 保持タイマーが一度も始まっていない


def test_extract_hold_steps_incomplete_when_rows_run_out() -> None:
    learning = LearningSection(
        cruise_hold_speeds_kmh=[30.0, 40.0],
        cruise_hold_settle_tol_kmh=1.0,
        cruise_hold_settle_s=0.3,
        cruise_hold_hold_s=0.5,
        cruise_hold_step_timeout_s=5.0,
    )
    # 30 km/h の段は保持完了（t=0.8）まで進むが、40 km/h の段の行が無いまま CSV が終わる
    rows = _constant_speed_rows(0, 9, 30.0)  # t=0.0〜0.8

    steps = cc.extract_hold_steps(rows, learning)

    assert len(steps) == 2
    assert steps[0].target_kmh == 30.0
    assert steps[0].outcome is cc.HoldOutcome.HELD
    assert steps[1].target_kmh == 40.0
    assert steps[1].outcome is cc.HoldOutcome.INCOMPLETE
    assert steps[1].rows == ()


def test_extract_hold_steps_groups_by_pattern_column() -> None:
    """1 つの CSV に定速階段パターンが 2 つあれば pattern 列ごとに独立して再現する。"""
    learning = LearningSection(
        cruise_hold_speeds_kmh=[30.0],
        cruise_hold_settle_tol_kmh=1.0,
        cruise_hold_settle_s=0.3,
        cruise_hold_hold_s=0.5,
        cruise_hold_step_timeout_s=5.0,
    )
    rows = _constant_speed_rows(0, 12, 30.0, pattern="18:CRUISE_TRIM")
    rows += _constant_speed_rows(0, 12, 30.0, pattern="25:CRUISE_TRIM")

    steps = cc.extract_hold_steps(rows, learning)

    assert len(steps) == 2
    assert all(s.outcome is cc.HoldOutcome.HELD for s in steps)


def test_extract_hold_steps_ignores_other_sections_and_phases() -> None:
    learning = LearningSection(cruise_hold_speeds_kmh=[30.0])
    rows = [
        _row(0.0, 30.0, section="PEDAL_SEARCH"),
        _row(0.1, 30.0, phase="DRIVE_ACCEL"),
    ]

    assert cc.extract_hold_steps(rows, learning) == []


def test_extract_hold_steps_empty_targets_returns_empty() -> None:
    learning = LearningSection(cruise_hold_speeds_kmh=[])
    rows = _constant_speed_rows(0, 5, 30.0)
    assert cc.extract_hold_steps(rows, learning) == []


# ─────────────────────────────────────────────────────────────────────
# build_cruise_curve
# ─────────────────────────────────────────────────────────────────────


def test_build_cruise_curve_medians_and_unreadable_rows_excluded_from_opening() -> None:
    learning = LearningSection(
        cruise_hold_speeds_kmh=[30.0, 40.0],
        cruise_hold_settle_tol_kmh=1.0,
        cruise_hold_settle_s=0.3,
        cruise_hold_hold_s=0.5,
        cruise_hold_step_timeout_s=5.0,
    )
    rows = _constant_speed_rows(0, 9, 30.0)  # t=0.0〜0.8、保持窓は t=0.3〜0.8（6 行）
    # 保持窓 6 行の実開度を個別に上書き（1 行は読めない行 = 空欄）
    hold_window_openings = {"0.3": 10.0, "0.4": 11.0, "0.5": 12.0, "0.6": None, "0.7": 13.0,
                             "0.8": 14.0}
    for row in rows:
        if row["elapsed_s"] in hold_window_openings:
            v = hold_window_openings[row["elapsed_s"]]
            row["accel_actual_pct"] = "" if v is None else f"{v:.3f}"
    rows += _constant_speed_rows(9, 12, 40.0, opening=20.0)  # t=0.9〜2.0、40 km/h の段

    curve = cc.build_cruise_curve(rows, learning)

    assert curve.speeds_kmh == pytest.approx((30.0, 40.0))
    # 読めた 5 値 [10,11,12,13,14] の中央値 = 12.0
    assert curve.openings_pct[0] == pytest.approx(12.0)
    assert curve.openings_pct[1] == pytest.approx(20.0)
    assert curve.n_rows[0] == 6  # 読めない行も含め、保持窓の行数
    assert curve.n_rows[1] >= 2


def test_build_cruise_curve_excludes_timed_out_steps() -> None:
    learning = LearningSection(
        cruise_hold_speeds_kmh=[30.0, 40.0, 50.0],
        cruise_hold_settle_tol_kmh=1.0,
        cruise_hold_settle_s=0.3,
        cruise_hold_hold_s=0.5,
        cruise_hold_step_timeout_s=1.0,
    )
    # 30 km/h の段は保持完了（t=0.8）。40 km/h の段は車速が 30 のままで一度も収束せず打ち切り
    # （t=0.8 から timeout=1.0 で t=1.8 に打ち切り）。50 km/h の段は t=1.9 から保持完了する。
    rows = _constant_speed_rows(0, 19, 30.0, opening=12.65)  # t=0.0〜1.8
    rows += _constant_speed_rows(19, 12, 50.0, opening=18.76)  # t=1.9〜3.0

    steps = cc.extract_hold_steps(rows, learning)
    assert [s.outcome for s in steps] == [
        cc.HoldOutcome.HELD,
        cc.HoldOutcome.TIMED_OUT,
        cc.HoldOutcome.HELD,
    ]

    curve = cc.build_cruise_curve(rows, learning)
    assert curve.speeds_kmh == pytest.approx((30.0, 50.0))
    assert curve.openings_pct == pytest.approx((12.65, 18.76), abs=0.01)


def test_build_cruise_curve_raises_for_fewer_than_two_usable_steps() -> None:
    learning = LearningSection(
        cruise_hold_speeds_kmh=[30.0],
        cruise_hold_settle_tol_kmh=1.0,
        cruise_hold_settle_s=0.3,
        cruise_hold_hold_s=0.5,
        cruise_hold_step_timeout_s=5.0,
    )
    rows = _constant_speed_rows(0, 12, 30.0)

    with pytest.raises(ValueError, match="足りません"):
        cc.build_cruise_curve(rows, learning)


def test_build_cruise_curve_raises_when_no_rows() -> None:
    learning = LearningSection(cruise_hold_speeds_kmh=[30.0, 40.0])
    with pytest.raises(ValueError):
        cc.build_cruise_curve([], learning)


# ─────────────────────────────────────────────────────────────────────
# CruiseCurve: 検証・opening_at・to_dict/from_dict
# ─────────────────────────────────────────────────────────────────────


def test_cruise_curve_post_init_rejects_length_mismatch() -> None:
    with pytest.raises(ValueError):
        cc.CruiseCurve(speeds_kmh=(30.0, 40.0), openings_pct=(12.0,), n_rows=(6, 6))


def test_cruise_curve_post_init_rejects_fewer_than_two_points() -> None:
    with pytest.raises(ValueError):
        cc.CruiseCurve(speeds_kmh=(30.0,), openings_pct=(12.0,), n_rows=(6,))


def test_cruise_curve_post_init_rejects_non_ascending() -> None:
    with pytest.raises(ValueError):
        cc.CruiseCurve(speeds_kmh=(40.0, 30.0), openings_pct=(14.0, 12.0), n_rows=(6, 6))
    with pytest.raises(ValueError):  # 重複も不可
        cc.CruiseCurve(speeds_kmh=(30.0, 30.0), openings_pct=(12.0, 12.0), n_rows=(6, 6))


def _sample_curve() -> cc.CruiseCurve:
    return cc.CruiseCurve(
        speeds_kmh=(30.0, 60.0, 90.0),
        openings_pct=(12.65, 16.96, 16.82),
        n_rows=(6, 6, 6),
    )


def test_opening_at_interpolates_within_range() -> None:
    curve = _sample_curve()
    # 45 km/h は 30〜60 の中点
    expected = (12.65 + 16.96) / 2
    assert curve.opening_at(45.0, floor_pct=0.0) == pytest.approx(expected)


def test_opening_at_extrapolates_below_and_above() -> None:
    curve = _sample_curve()
    low_slope = (16.96 - 12.65) / (60.0 - 30.0)
    expected_low = 12.65 + low_slope * (20.0 - 30.0)
    assert curve.opening_at(20.0, floor_pct=0.0) == pytest.approx(expected_low)

    high_slope = (16.82 - 16.96) / (90.0 - 60.0)
    expected_high = 16.82 + high_slope * (140.0 - 90.0)
    assert curve.opening_at(140.0, floor_pct=0.0) == pytest.approx(expected_high)


def test_opening_at_floors_but_does_not_ceiling() -> None:
    curve = _sample_curve()
    # 下側への直線延長がアクセル不感帯を下回る場合は floor_pct で底打ちする
    low_slope = (16.96 - 12.65) / (60.0 - 30.0)
    raw_at_zero = 12.65 + low_slope * (0.0 - 30.0)
    floor_pct = 9.0
    assert raw_at_zero < floor_pct  # floor より下がることを確認した上でテストする
    assert curve.opening_at(0.0, floor_pct=floor_pct) == pytest.approx(floor_pct)
    # 上限は無いので、非常に大きい値でもクランプされない
    high_slope = (16.82 - 16.96) / (90.0 - 60.0)
    expected_high = 16.82 + high_slope * (300.0 - 90.0)
    assert curve.opening_at(300.0, floor_pct=floor_pct) == pytest.approx(expected_high)


def test_opening_at_scalar_returns_float_array_returns_array() -> None:
    curve = _sample_curve()
    result_scalar = curve.opening_at(60.0, floor_pct=0.0)
    assert isinstance(result_scalar, float)
    assert result_scalar == pytest.approx(16.96)

    result_array = curve.opening_at(np.array([30.0, 60.0, 90.0]), floor_pct=0.0)
    assert isinstance(result_array, np.ndarray)
    assert result_array == pytest.approx([12.65, 16.96, 16.82])


def test_to_dict_from_dict_roundtrip() -> None:
    curve = _sample_curve()
    restored = cc.CruiseCurve.from_dict(curve.to_dict())
    assert restored == curve
    d = curve.to_dict()
    assert all(isinstance(v, list) for v in d.values())
    assert all(isinstance(v, float) for v in d["speeds_kmh"])
    assert all(isinstance(v, int) for v in d["n_rows"])


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


def _write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    import csv

    fieldnames = ["section", "phase", "pattern", "elapsed_s", "actual_speed_kmh",
                  "accel_actual_pct"]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def test_main_returns_zero_and_prints_table(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    # 既定 config の cruise_hold_settle_s=3.0・hold_s=8.0 に余裕を持たせ、
    # 既定 config の先頭 2 段（10・20 km/h）が確実に保持完了するだけの行数を用意する
    rows: list[dict[str, str]] = []
    t = 0.0
    for speed in (10.0, 20.0):
        for _ in range(121):  # 0.1s 刻みで 12.1s 分（settle 3.0 + hold 8.0 = 11.0s に余裕）
            rows.append(_row(round(t, 1), speed, opening=speed / 2.0))
            t = round(t + 0.1, 1)

    csv_path = tmp_path / "drive_log_stub_20260915_000000.csv"
    _write_csv(csv_path, rows)

    assert cc.main([str(csv_path)]) == 0
    out = capsys.readouterr().out
    assert "| 目標 [km/h] | 終わり方 |" in out
    assert "実測テーブルの点数: 2" in out


def test_main_returns_one_when_no_cruise_hold_rows(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    rows = [_row(0.0, 30.0, phase="DRIVE_ACCEL"), _row(0.1, 30.0, phase="DRIVE_ACCEL")]
    csv_path = tmp_path / "drive_log_stub_20260915_000001.csv"
    _write_csv(csv_path, rows)

    assert cc.main([str(csv_path)]) == 1
    out = capsys.readouterr().out
    assert "保持段なし" in out
