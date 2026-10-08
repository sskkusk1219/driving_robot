"""G ランプ走行の段の計画（ramp_planner。段2k 段A）のユニットテスト。"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research import ramp_planner as rp
from tests.research.research_types import G_TO_KMHS

LEVELS = (0.02, 0.1, 0.2)  # 下端: 0.01, 0.06, 0.15（|a| = G × G_TO_KMHS）


def triangle(g: float, peak_kmh: float, start_kmh: float = 10.0) -> tuple[np.ndarray, np.ndarray]:
    """一定の G で start → peak へ上がり、同じ G で start へ下がる三角形（1 秒刻み）。"""
    a = g * G_TO_KMHS
    n = int(round((peak_kmh - start_kmh) / a))
    up = start_kmh + a * np.arange(n + 1)
    v = np.concatenate([up, up[-2::-1]])
    return np.arange(len(v), dtype=float), v


def test_band_assignment_boundaries_and_lower_edge_inclusive() -> None:
    assert rp.level_lower_edges(LEVELS) == pytest.approx([0.01, 0.06, 0.15])
    # 加速度ちょうど下端（0.06G）は段 1（0.1G）の帯に入る。直前は段 0
    for g, col in ((0.0600001, 1), (0.0599, 0), (0.0100001, 0), (0.30, 2)):
        t = np.array([0.0, 1.0])
        v = np.array([20.0, 20.0 + g * G_TO_KMHS])
        c = rp.count_bands([(t, v)], LEVELS)
        assert c.accel_s[:, col].sum() == 1.0
        assert c.accel_s.sum() == 1.0
    # 最下端未満は cruise
    c = rp.count_bands([(np.array([0.0, 1.0]), np.array([20.0, 20.0 + 0.005 * G_TO_KMHS]))], LEVELS)
    assert c.cruise_s == 1.0 and c.accel_s.sum() == 0.0


def test_accel_and_decel_counted_separately() -> None:
    c = rp.count_bands([triangle(0.1, 50.0)], LEVELS)
    assert c.accel_s[:, 1].sum() > 0 and c.decel_s[:, 1].sum() > 0
    assert c.accel_s[:, 1].sum() == c.decel_s[:, 1].sum()
    assert c.accel_s[:, [0, 2]].sum() == 0 and c.decel_s[:, [0, 2]].sum() == 0


def test_stopped_segments_not_counted() -> None:
    # 中点 0.4 km/h の区間（0 → 0.8）は数えない。0.8 → 0.8 は中点 0.8 でほぼ定速
    t = np.array([0.0, 1.0, 2.0])
    v = np.array([0.0, 0.8, 0.8])
    c = rp.count_bands([(t, v)], LEVELS)
    assert c.accel_s.sum() == 0.0 and c.decel_s.sum() == 0.0
    assert c.cruise_s == 1.0


def test_interpolates_to_one_second_grid() -> None:
    # 2 秒間隔の点でも 1 秒刻みに補間して 2 区間を数える
    g = 0.1
    t = np.array([0.0, 2.0])
    v = np.array([20.0, 20.0 + 2 * g * G_TO_KMHS])
    c = rp.count_bands([(t, v)], LEVELS)
    assert c.accel_s[:, 1].sum() == 2.0


def test_top_speed_ignores_bins_below_min_s_and_caps_at_max_speed() -> None:
    c = rp.count_bands([triangle(0.1, 85.0)], LEVELS)  # 段 1 は 10〜85 km/h を往復
    plan = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=200.0, budget_s=1e9)
    lv = {x.g: x for x in plan.levels}
    assert lv[0.1].top_kmh == 90.0  # 85 を含む帯 80–90 の上端
    # 80–90 の帯の秒数が少ないとき min_s を上げると無視される
    sec = c.accel_s[8, 1]
    plan2 = rp.plan_ramps(c, mode_min_s=sec + 0.5, max_speed_kmh=200.0, budget_s=1e9)
    assert {x.g: x for x in plan2.levels}[0.1].top_kmh == 80.0
    # 最高速で頭打ち
    plan3 = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=60.0, budget_s=1e9)
    assert {x.g: x for x in plan3.levels}[0.1].top_kmh == 60.0


def test_level_without_data_is_skipped() -> None:
    c = rp.count_bands([triangle(0.1, 50.0)], LEVELS)
    plan = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=1e9)
    assert plan.skipped == (0.02, 0.2)
    assert [x.g for x in plan.levels] == [0.1]
    assert "走らない" in rp.format_plan_table(plan)


def test_estimate_formula() -> None:
    expected = 2 * 60.0 / (0.1 * G_TO_KMHS) + rp.RAMP_OVERHEAD_S
    assert rp.estimate_s(0.1, 60.0) == pytest.approx(expected)


def _two_level_counts() -> rp.BandCounts:
    # 段 1（0.1G）の方がモードの秒数が長い。段 2（0.2G）は短い
    s1 = triangle(0.1, 60.0)
    s2 = triangle(0.2, 30.0)
    return rp.count_bands([s1, s2], LEVELS)


def test_second_pass_goes_to_level_with_most_mode_seconds() -> None:
    c = _two_level_counts()
    base = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=0.0)
    e = {x.g: x.est_s for x in base.levels}
    # 段 1 の 2 周目だけが入る枠
    budget = sum(e.values()) + e[0.1] + 0.1
    plan = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=budget)
    passes = {x.g: x.passes for x in plan.levels}
    assert passes == {0.1: 2, 0.2: 1}
    assert not plan.over_budget
    assert plan.total_est_s == pytest.approx(2 * e[0.1] + e[0.2])


def test_second_pass_skips_level_that_does_not_fit_and_tries_next() -> None:
    c = _two_level_counts()
    base = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=0.0)
    e = {x.g: x.est_s for x in base.levels}
    assert e[0.1] > e[0.2]
    # 段 1 の 2 周目は入らず、段 2 の 2 周目は入る枠
    budget = sum(e.values()) + e[0.2] + 0.1
    plan = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=budget)
    assert {x.g: x.passes for x in plan.levels} == {0.1: 1, 0.2: 2}


def test_over_budget_keeps_first_pass() -> None:
    c = _two_level_counts()
    plan = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=10.0)
    assert plan.over_budget
    assert [x.passes for x in plan.levels] == [1, 1]
    assert len(plan.levels) == 2
    assert "警告" in rp.format_plan_table(plan)


def test_execution_order_pass1_then_pass2_ascending() -> None:
    c = rp.count_bands([triangle(0.02, 40.0), triangle(0.1, 60.0), triangle(0.2, 30.0)], LEVELS)
    plan = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=1e9)
    assert all(x.passes == rp.MAX_PASSES for x in plan.levels)
    order = plan.execution_order()
    gs = [g for g, _ in order]
    assert gs == [0.02, 0.1, 0.2, 0.02, 0.1, 0.2]
    assert order[1] == (0.1, plan.levels[1].top_kmh)


def test_table_formatting_runs() -> None:
    c = _two_level_counts()
    assert "加速側" in rp.format_band_table(c)
    assert "合計の見込み" in rp.format_plan_table(
        rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=1200.0)
    )


# ── 上端の車速（2026-10-07 段2l-4）───────────────────────────────────────


def _mode(max_speed: float):  # noqa: ANN202
    from datetime import UTC, datetime  # noqa: PLC0415

    from tests.research.research_types import DrivingMode  # noqa: PLC0415

    return DrivingMode(
        id="m", name="m", description="", reference_speed=[], total_duration=0.0,
        max_speed=max_speed, created_at=datetime.now(UTC),
    )


def test_top_speed_cap_is_the_highest_mode_speed_plus_margin() -> None:
    modes = [_mode(131.3), _mode(129.2)]
    assert rp.top_speed_cap_kmh(modes, 5.0, 150.0) == pytest.approx(136.3)


def test_top_speed_cap_is_limited_by_the_vehicle_max_speed() -> None:
    assert rp.top_speed_cap_kmh([_mode(131.3)], 5.0, 134.0) == pytest.approx(134.0)


def test_top_speed_cap_with_zero_margin_and_no_modes() -> None:
    assert rp.top_speed_cap_kmh([_mode(100.0)], 0.0, 150.0) == pytest.approx(100.0)
    assert rp.top_speed_cap_kmh([], 5.0, 150.0) == pytest.approx(150.0)


def test_plan_ramps_top_is_capped_by_the_given_top_speed() -> None:
    t = np.arange(0.0, 200.0)
    v = np.minimum(t, 120.0)  # 1 km/h/s ずっと加速して 120 で頭打ち
    counts = rp.count_bands([(t, v)], [0.02, 0.045])
    plan = rp.plan_ramps(counts, mode_min_s=2.0, max_speed_kmh=60.0, budget_s=1200.0)
    assert all(lv.top_kmh <= 60.0 for lv in plan.levels)
