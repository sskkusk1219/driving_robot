"""grid_settle（落ち着き待ち・ステップの当てはめの集計）と、PatternLoop の記録のテスト。"""

from __future__ import annotations

import math

import pytest

from tests.research import grid_settle as gs
from tests.research.grid_planner import Step, StepKind, StepResult
from tests.research.pattern_loop import _Phase
from tests.research.test_research_pattern_loop import _loop
from tests.research.test_research_pattern_loop_grid import _plan, _settle, _station


def _rec(**kw: object) -> gs.SettleRecord:
    base: dict[str, object] = {
        "station_kmh": 25.0, "target_kmh": 25.0, "approach": False, "wait_s": 5.0,
        "inside_frac": 0.5, "u_mean_pct": 7.0, "u_std_pct": 0.2, "timed_out": False,
    }
    base.update(kw)
    return gs.SettleRecord(**base)  # type: ignore[arg-type]


def _result(points: int, resid: float, approach: float = float("nan")) -> StepResult:
    return StepResult(
        speed_kmh=25.0, kind=StepKind.ACCEL, target_a_kmhs=1.0, accel_pct=8.0, brake_pct=0.0,
        a_meas_kmhs=1.0, gain_before=2.0, gain_after=2.0, verdict="OK", tries=1,
        approach_kmh=approach, fit_points=points, fit_resid_kmh=resid,
    )


def test_table_marks_approach_timeout_and_missing_opening() -> None:
    text = gs.settle_table([
        _rec(),
        _rec(approach=True, target_kmh=15.0),
        _rec(timed_out=True, u_mean_pct=math.nan, u_std_pct=math.nan),
    ])
    lines = text.splitlines()
    assert "中心" in lines[1] and "助走" in lines[2]
    assert "打切り" in lines[3] and "—" in lines[3]


def test_report_summarizes_wait_by_kind_and_fit() -> None:
    records = [
        _rec(wait_s=4.0), _rec(wait_s=8.0, u_std_pct=0.4),
        _rec(approach=True, wait_s=12.0),
        _rec(timed_out=True, wait_s=40.0, u_mean_pct=math.nan, u_std_pct=math.nan),
    ]
    text = gs.settle_report(
        records, [_result(25, 0.2), _result(10, 0.5, approach=15.0)],
        tol_kmh=1.0, settle_s=3.0, duration_s=100.0,
    )
    assert "待ち時間の合計 64s" in text and "パターン走行 100s の 64%" in text
    assert "中心: 3 回（打切り 1）" in text and "助走: 1 回（打切り 0）" in text
    assert "点数 中央 18 / 最小 10" in text  # (25, 10) の中央
    assert "うち助走つき 1 本: 点数 中央 10" in text


def test_report_is_empty_without_records() -> None:
    assert gs.settle_report([], [], tol_kmh=1.0, settle_s=3.0, duration_s=10.0) == ""
    assert "記録なし" in gs.fit_summary([])


# ── PatternLoop が記録する ────────────────────────────────────────────


def test_loop_records_settle_wait_inside_fraction_and_window_opening() -> None:
    pattern = _station(_plan(25.0))
    loop, *_ = _loop([pattern])
    got: list[gs.SettleRecord] = []
    loop._on_grid_settle = got.append
    loop._grid_enter_pattern(pattern, 0.0)
    loop._advance_grid_hold(pattern, 10.0, 0.0)  # 許容幅の外 1 周期
    for i, t in enumerate([0.25, 0.5, 0.75, 1.0, 1.25]):  # 許容幅内 5 周期 (1.0s 以上)
        loop._current_accel_opening = 6.0 + (i % 2)
        loop._advance_grid_hold(pattern, 25.2, t)
    assert loop._phase is _Phase.HOLD_STEP
    (rec,) = loop.grid_settles
    assert got == [rec]
    assert not rec.approach and not rec.timed_out
    assert rec.station_kmh == 25.0 and rec.target_kmh == 25.0
    assert rec.wait_s == pytest.approx(1.25)
    assert rec.inside_frac == pytest.approx(5 / 6)
    assert rec.u_mean_pct == pytest.approx(6.4) and rec.u_std_pct > 0.0


def test_loop_records_timeout_as_timed_out() -> None:
    pattern = _station(settle_timeout_s=2.0)
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    for t in (0.0, 1.0, 2.1):
        loop._advance_grid_hold(pattern, 10.0, t)
    (rec,) = loop.grid_settles
    assert rec.timed_out and math.isnan(rec.u_mean_pct) and rec.inside_frac == 0.0


def test_step_result_carries_fit_points_and_residual() -> None:
    pattern = _station(_plan(25.0))
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    t = _settle(loop, pattern, 7.0)
    step = loop._grid_step
    assert isinstance(step, Step)
    for k in range(1, 12):  # 傾き 0.5 km/h/s の直線（窓 1.0s で終わる）
        if loop._phase is not _Phase.HOLD_STEP:
            break
        loop._advance_grid_step(pattern, 25.0 + 0.5 * 0.1 * k, t + 0.1 * k)
    result = loop.grid_planner.results[0]  # type: ignore[union-attr]
    assert result.fit_points >= 3
    assert result.fit_resid_kmh == pytest.approx(0.0, abs=1e-6)


def test_table_and_report_show_window_slope() -> None:
    text = gs.settle_table([_rec(slope_kmhs=0.31), _rec(slope_kmhs=-0.05)])
    assert "窓の傾き" in text and "+0.31" in text and "-0.05" in text
    report = gs.settle_report(
        [_rec(slope_kmhs=0.31), _rec(slope_kmhs=-0.05)], [],
        tol_kmh=1.0, settle_s=3.0, duration_s=100.0,
    )
    assert "窓の傾き |中央| 0.18 / 最大 0.31 km/h/s" in report
