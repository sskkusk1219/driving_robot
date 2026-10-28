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


def test_max_passes_one_gives_single_pass_even_with_huge_budget() -> None:
    """2026-10-17 段2o-8: max_passes=1 なら枠が余っても 2 周目を足さない。"""
    c = rp.count_bands([triangle(0.02, 40.0), triangle(0.1, 60.0), triangle(0.2, 30.0)], LEVELS)
    kw = dict(mode_min_s=1.0, max_speed_kmh=140.0, budget_s=1e9)
    one = rp.plan_ramps(c, max_passes=1, **kw)
    assert all(x.passes == 1 for x in one.levels)
    assert [g for g, *_ in one.execution_order()] == [0.02, 0.1, 0.2]
    # 既定（2）は max_passes=2 と同じ計画
    default = rp.plan_ramps(c, **kw)
    two = rp.plan_ramps(c, max_passes=2, **kw)
    assert default == two
    assert all(x.passes == 2 for x in default.levels)
    assert len(default.execution_order()) == 6


def test_execution_order_pass1_then_pass2_ascending() -> None:
    c = rp.count_bands([triangle(0.02, 40.0), triangle(0.1, 60.0), triangle(0.2, 30.0)], LEVELS)
    plan = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=1e9)
    assert all(x.passes == rp.MAX_PASSES for x in plan.levels)
    order = plan.execution_order()
    gs = [g for g, *_ in order]
    assert gs == [0.02, 0.1, 0.2, 0.02, 0.1, 0.2]
    assert order[1] == (0.1, plan.levels[1].top_kmh, (), False, ())
    assert all(len(item) == 5 for item in order)  # 段2o-5: 5 つ目は戻す車速


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


def test_estimate_includes_hold_and_two_turns_when_given() -> None:
    base = rp.estimate_s(0.1, 60.0)
    assert rp.estimate_s(0.1, 60.0, hold_s=7.0, turn_s=2.0) == pytest.approx(base + 7.0 + 2 * 2.0)
    assert rp.RAMP_OVERHEAD_S == 15.0  # 変えない


def test_plan_ramps_passes_hold_and_turn_into_the_estimate() -> None:
    c = rp.count_bands([triangle(0.1, 50.0), triangle(0.2, 30.0)], LEVELS)
    plain = rp.plan_ramps(c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=1e9)
    longer = rp.plan_ramps(
        c, mode_min_s=1.0, max_speed_kmh=140.0, budget_s=1e9, hold_s=7.0, turn_s=2.0
    )
    for a, b in zip(plain.levels, longer.levels, strict=True):
        assert b.est_s == pytest.approx(a.est_s + 7.0 + 4.0)


# ── 最小の上端（2026-10-10 段2l-11）──────────────────────────────────

_MIN_KW = {"min_at_g_s": 3.25, "launch_s": 1.0, "turn_s": 2.0, "edge_g": 0.05}


def _wltp_like_counts() -> rp.BandCounts:
    """0.25G の段は 40–50 km/h 帯まで、0.35G の段は 20–30 km/h 帯までモードに秒数がある。"""
    accel = np.zeros((6, 2))
    accel[4, 0] = 3.0  # 0.25G: 40–50 km/h 帯 → 上端 50
    accel[2, 1] = 3.0  # 0.35G: 20–30 km/h 帯 → 上端 30
    return rp.BandCounts((0.25, 0.35), 10.0, accel, np.zeros((6, 2)), 0.0)


def test_min_top_formula_and_rounding_up_to_the_speed_bin() -> None:
    # 0.35G: 立ち上げ (0.05+0.35)/2×35.3×1 ＋ 0.35×35.3×(3.25+1) の切り上げ ＝ 59.6 → 60
    assert rp.min_top_kmh(0.35, **_MIN_KW) == pytest.approx(60.0)
    # 3.3 秒だと計算値が 60.2 km/h になり 70 に切り上がる（3.25 を選んだ理由）
    assert rp.min_top_kmh(0.35, **{**_MIN_KW, "min_at_g_s": 3.3}) == pytest.approx(70.0)
    # 0 以下なら使わない
    assert rp.min_top_kmh(0.35, **{**_MIN_KW, "min_at_g_s": 0.0}) == 0.0
    # 狙いの G が edge_g より小さい段は (G + G) ÷ 2 ＝ G
    got = rp.min_top_kmh(0.03, **_MIN_KW)
    assert got == pytest.approx(10.0 * np.ceil((0.03 * G_TO_KMHS * (1.0 + 3.25 + 1.0)) / 10 - 1e-9))


def test_min_top_does_not_step_up_on_a_float_boundary() -> None:
    # 計算値がちょうど 10 の倍数のとき、誤差で 1 段上がらない
    g = 20.0 / (G_TO_KMHS * 2.0)  # 立ち上げ 0 ＋ G×35.3×(1+1) ＝ 20 km/h ちょうど
    got = rp.min_top_kmh(g, min_at_g_s=1.0, launch_s=0.0, turn_s=2.0, edge_g=0.0)
    assert got == pytest.approx(20.0)


def test_plan_ramps_raises_the_top_of_the_strong_level_only() -> None:
    counts = _wltp_like_counts()
    base = rp.plan_ramps(counts, mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9)
    plan = rp.plan_ramps(counts, mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9, **_MIN_KW)
    by_g = {lv.g: lv for lv in plan.levels}
    base_by_g = {lv.g: lv for lv in base.levels}
    assert base_by_g[0.35].top_kmh == pytest.approx(30.0)
    assert by_g[0.35].top_kmh == pytest.approx(60.0) and by_g[0.35].raised
    # 0.25G はモードの 50 のまま（最小の上端 43 → 50 より低い）
    assert by_g[0.25].top_kmh == base_by_g[0.25].top_kmh and not by_g[0.25].raised
    # 見込み秒数は引き上げ後の上端で計算する
    assert by_g[0.35].est_s == pytest.approx(rp.estimate_s(0.35, 60.0, turn_s=2.0))


def test_plan_ramps_min_at_g_zero_gives_the_same_result_as_before() -> None:
    counts = _wltp_like_counts()
    base = rp.plan_ramps(counts, mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9)
    zero = rp.plan_ramps(
        counts, mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9,
        min_at_g_s=0.0, launch_s=1.0, turn_s=0.0, edge_g=0.05,
    )
    assert zero == base


def test_plan_ramps_min_top_is_still_capped_by_the_max_speed() -> None:
    plan = rp.plan_ramps(
        _wltp_like_counts(), mode_min_s=2.0, max_speed_kmh=45.0, budget_s=1e9, **_MIN_KW
    )
    assert all(lv.top_kmh <= 45.0 for lv in plan.levels)
    # 最小の上端 60 は頭打ち 45 で丸める（モードの上端 30 からは引き上がっている）
    top = {lv.g: lv for lv in plan.levels}[0.35]
    assert top.top_kmh == pytest.approx(45.0) and top.raised
    # 頭打ちがモードの上端以下なら「引き上げた」とは表示しない
    low = rp.plan_ramps(
        _wltp_like_counts(), mode_min_s=2.0, max_speed_kmh=25.0, budget_s=1e9, **_MIN_KW
    )
    assert not any(lv.raised for lv in low.levels)


def test_plan_table_marks_the_raised_level() -> None:
    plan = rp.plan_ramps(
        _wltp_like_counts(), mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9, **_MIN_KW
    )
    text = rp.format_plan_table(plan)
    assert "| 0.35 | 60* |" in text and "| 0.25 | 50 |" in text
    assert "最小の上端" in text
    plain = rp.format_plan_table(
        rp.plan_ramps(_wltp_like_counts(), mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9)
    )
    assert "*" not in plain


# ── 定速の保持の車速（2026-10-13 段2o-2）───────────────────────────────


def _cruise_counts(speeds_kmh: list[float], sec: int = 3) -> rp.BandCounts:
    """指定の車速で `sec` 秒ずつ完全な定速にいる基準（車速帯 × ほぼ定速の秒数を作る）。"""
    series = []
    for v in speeds_kmh:
        series.append((np.arange(sec + 1, dtype=float), np.full(sec + 1, v)))
    return rp.count_bands(series, LEVELS)


def test_cruise_by_speed_counted_per_bin() -> None:
    c = _cruise_counts([22.0, 47.0])
    assert c.cruise_by_speed_s[2] == 3.0 and c.cruise_by_speed_s[4] == 3.0
    assert c.cruise_by_speed_s.sum() == c.cruise_s == 6.0


def test_plan_cruise_stations_centres_coarse_and_limits() -> None:
    # 4（クリープ域）, 12, 22, 47, 97, 105・125・133（細かい刻みの上限の上）, 185（上端以上）
    c = _cruise_counts([4.0, 12.0, 22.0, 47.0, 97.0, 105.0, 125.0, 133.0, 185.0])
    got = rp.plan_cruise_stations(
        c, mode_min_s=2.0, fine_max_kmh=100.0, coarse_step_kmh=20.0, min_kmh=10.0, top_kmh=150.0
    )
    # 4 km/h（中心 5）は min 未満で外れる。100 超は 20 刻みの帯の中心（110, 130）。185 は上端以上
    assert got == (15.0, 25.0, 45.0, 95.0, 110.0, 130.0)


def test_plan_cruise_stations_needs_mode_min_seconds() -> None:
    c = _cruise_counts([22.0], sec=1)
    assert rp.plan_cruise_stations(
        c, mode_min_s=2.0, fine_max_kmh=100.0, coarse_step_kmh=20.0, min_kmh=10.0, top_kmh=150.0
    ) == ()


def test_stations_only_on_weakest_level_and_estimate_includes_hold() -> None:
    c = rp.count_bands(
        [triangle(0.02, 40.0), triangle(0.1, 60.0),
         (np.arange(4, dtype=float), np.full(4, 22.0)),
         (np.arange(4, dtype=float), np.full(4, 32.0))],
        LEVELS,
    )
    kw = dict(mode_min_s=1.0, max_speed_kmh=140.0, budget_s=1e9)
    plain = rp.plan_ramps(c, **kw)
    held = rp.plan_ramps(
        c, **kw, turn_s=2.0, cruise_hold_s=7.0, cruise_fine_max_kmh=100.0,
        cruise_coarse_step_kmh=20.0, cruise_min_kmh=10.0,
    )
    assert plain.levels[0].stations == ()
    assert held.levels[0].stations and not held.levels[0].stations_up_only
    assert all(lv.stations_up_only == bool(lv.stations) for lv in held.levels[1:])
    n = len(held.levels[0].stations)
    # 見込み = 保持なしの見込み(切り替え分込み) + n × (7 + 2×2 + 3) × 2
    base = rp.estimate_s(held.levels[0].g, held.levels[0].top_kmh, turn_s=2.0)
    assert held.levels[0].est_s == pytest.approx(base + n * (7.0 + 4.0 + 3.0) * 2.0)
    assert held.execution_order()[0][2] == held.levels[0].stations
    assert "保持" in rp.format_plan_table(held)


# ---- 2026-10-14 段2o-4: 走行中から始まる加速・上りだけの保持 ----


def test_accel_start_excludes_from_stop_and_counts_from_cruise() -> None:
    # 停車(0)から +2 km/h/s で 5 秒加速 → 数えない
    from_stop = (np.arange(6, dtype=float), np.arange(6, dtype=float) * 2.0)
    # 20 km/h の定速 3 秒 → 毎秒 +2 km/h(0.0567G → 段 0) で 5 秒加速
    t = np.arange(9, dtype=float)
    v = np.array([20.0, 20.0, 20.0, 20.0, 22.0, 24.0, 26.0, 28.0, 30.0])
    c0 = rp.count_bands([from_stop], LEVELS)
    assert c0.accel_start_s.sum() == 0.0
    c = rp.count_bands([(t, v)], LEVELS)
    assert c.accel_start_s.shape == c.accel_s.shape
    # 2 km/h/s = 0.0567G は下端 0.01〜0.06 の段 0。始まりの車速は 20 km/h → 帯 20〜30 (index 2)
    assert c.accel_start_s[2, 0] == 5.0 and c.accel_start_s.sum() == 5.0
    # 既存の値は変わらない（中点が 21..29 → 帯 2 に 5 秒、定速 3 秒）
    assert c.accel_s[2, 0] == 5.0 and c.accel_s.sum() == 5.0
    assert c.cruise_s == 3.0 and c.decel_s.sum() == 0.0


def _start_counts(secs: dict[int, float]) -> rp.BandCounts:
    n = 16
    arr = np.zeros((n, 3))
    for k, s in secs.items():
        arr[k, 1] = s
    return rp.BandCounts(
        LEVELS, 10.0, np.zeros((n, 3)), np.zeros((n, 3)), 0.0, np.zeros(n), arr
    )


KW = dict(mode_min_s=2.0, fine_max_kmh=100.0, coarse_step_kmh=20.0, min_kmh=10.0, top_kmh=150.0)


def test_plan_accel_stations_min_seconds_and_min_kmh() -> None:
    c = _start_counts({0: 5.0, 1: 2.0, 2: 1.0, 4: 3.0})  # 帯 0 は 5 km/h(<min) で外れる
    got = rp.plan_accel_stations(c, 1, min_gap_kmh=0.0, **KW)
    assert got == (15.0, 45.0)  # 帯 2 は 1 秒で不足
    assert rp.plan_accel_stations(rp.BandCounts(LEVELS, 10.0, np.zeros((0, 3)),
                                  np.zeros((0, 3)), 0.0), 1, min_gap_kmh=0.0, **KW) == ()


def test_plan_accel_stations_min_gap_keeps_lower_and_top_gap() -> None:
    c = _start_counts({1: 3.0, 2: 3.0, 3: 3.0, 4: 3.0})  # 15, 25, 35, 45
    got = rp.plan_accel_stations(c, 1, min_gap_kmh=15.0, **KW)
    assert got == (15.0, 35.0)  # 25 は 10 しか離れず外れ、35 は 20 離れて残る。45 は 10 で外れる
    # 上端までの間隔: top 50 → 45 は 5 < 15 で外れ、35 も 15 ちょうどは残る
    got2 = rp.plan_accel_stations(c, 1, min_gap_kmh=15.0, **{**KW, "top_kmh": 50.0})
    assert got2 == (15.0, 35.0)
    got3 = rp.plan_accel_stations(c, 1, min_gap_kmh=15.0, **{**KW, "top_kmh": 40.0})
    assert got3 == (15.0,)


def test_plan_accel_stations_coarse_above_100() -> None:
    c = _start_counts({10: 3.0, 12: 3.0})  # 100〜110 → 110 の帯、120〜130 → 130 の帯
    got = rp.plan_accel_stations(c, 1, min_gap_kmh=0.0, **{**KW, "top_kmh": 200.0})
    assert got == (110.0, 130.0)


def test_plan_ramps_up_only_stations_on_other_levels() -> None:
    # 定速 20 km/h → 各 G で加速する列を 3 段分つなぐ
    def run(g: float) -> tuple[np.ndarray, np.ndarray]:
        a = g * G_TO_KMHS
        v = np.concatenate([np.full(4, 20.0), 20.0 + a * np.arange(1, 6)])
        return np.arange(len(v), dtype=float), v
    c = rp.count_bands([run(0.02), run(0.1), run(0.2), triangle(0.02, 40.0)], LEVELS)
    plan = rp.plan_ramps(
        c, mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9, turn_s=2.0, cruise_hold_s=7.0,
        cruise_min_kmh=10.0,
    )
    assert plan.levels[0].stations_up_only is False
    others = plan.levels[1:]
    assert any(lv.stations for lv in others)
    assert all(lv.stations_up_only == bool(lv.stations) for lv in others)
    assert all(len(item) == 5 for item in plan.execution_order())  # 段2o-5: 5 つ目は戻す車速
    lv = next(lv for lv in others if lv.stations)
    base = rp.estimate_s(lv.g, lv.top_kmh, turn_s=2.0)
    assert lv.est_s == pytest.approx(base + len(lv.stations) * (7.0 + 4.0 + 3.0))


# ---- 2026-10-14 段2o-5: 下りの戻し ----


def _decel_run(start_kmh: float, gs: list[float], hold: int = 2) -> tuple[np.ndarray, np.ndarray]:
    """start_kmh で hold 秒の定速 → 1 秒ごとに gs[i] [G] で減速する列（1 秒刻み）。"""
    v = [start_kmh] * hold
    for g in gs:
        v.append(v[-1] - g * G_TO_KMHS)
    return np.arange(len(v), dtype=float), np.asarray(v)


# 段 1（0.1G。帯 0.06〜0.15G）の減速。いちばん強い 1 秒は 3 本目（0.12G）
_RUN_GS = [0.07, 0.1, 0.12, 0.1, 0.1, 0.1]


def _release_counts() -> rp.BandCounts:
    """60 km/h から約 20 km/h まで、走行中に終わる減速を 2 本。停車まで下りる減速を 1 本。"""
    to_stop = _decel_run(30.0, [0.1] * 9)
    return rp.count_bands(
        [_decel_run(60.0, _RUN_GS), _decel_run(60.0, _RUN_GS), to_stop,
         _decel_run(60.0, [0.02] * 6), _decel_run(60.0, [0.02] * 6)], LEVELS,
        release_min_kmh=15.0,
    )


def test_release_counts_from_strongest_second_to_the_end_of_the_run() -> None:
    """走行中に終わる減速の区間だけ、いちばん強い 1 秒から終わりまでの秒数を、その 1 秒の車速帯・
    段に足す。release_min_kmh 未満（停車）で終わる区間は数えない。"""
    t, v = _decel_run(60.0, _RUN_GS)
    c = rp.count_bands([(t, v)], LEVELS, release_min_kmh=10.0)
    assert c.release_s.shape == c.decel_s.shape
    # いちばん強い 1 秒は 3 本目の減速（v[3] から v[4]）。そこから終わりまで 4 秒（3〜6 本目）
    assert c.release_s.sum() == 4.0
    v_peak = float(v[3])
    assert c.release_s[int(v_peak // rp.SPEED_BIN_KMH), 1] == 4.0
    # 停車まで下りる区間は数えない
    c2 = rp.count_bands([_decel_run(20.0, [0.1] * 9)], LEVELS, release_min_kmh=10.0)
    assert c2.release_s.sum() == 0.0
    # 既定のしきい値は停車（0.5 km/h）。走行中に終わるので数える
    assert rp.count_bands([(t, v)], LEVELS).release_s.sum() == 4.0


def test_release_drop_matches_the_default_flow_of_one_point_seven_five_seconds() -> None:
    """既定の流れ（0.5 → 0、移動 1 秒、保持 1.5・2 秒）: 1×0.75 ＋ 1.5×0.5 ＋ 1×0.25 ＋ 2×0。"""
    g = 0.1
    drop = rp.release_drop_kmh(g, (0.5, 0.0), (1.5, 2.0), 1.0)
    assert drop == pytest.approx(g * G_TO_KMHS * 1.75)
    assert rp.release_est_s(2, fracs=(0.5, 0.0), hold_s=(1.5, 2.0), move_s=1.0, turn_s=2.0) == (
        pytest.approx(2 * 7.5)
    )


def test_plan_release_stations_drop_gap_and_top_rules() -> None:
    """外す規則: 車速 − drop が min 未満・直前に採った点との間隔不足（低い側を残す）・上端までの
    間隔不足。"""
    arr = np.zeros((14, 3))
    for k in (1, 2, 3, 6, 12):
        arr[k, 1] = 5.0
    c = rp.BandCounts(
        LEVELS, 10.0, np.zeros((14, 3)), np.zeros((14, 3)), 0.0, np.zeros(14),
        np.zeros((0, 0)), arr,
    )
    kw = dict(mode_min_s=2.0, fine_max_kmh=100.0, coarse_step_kmh=20.0, top_kmh=140.0)
    # 候補 15, 25, 35, 65, 125→130（粗い帯）。drop 6: 15 − 6 = 9 < 10 で外れる
    out = rp.plan_release_stations(c, 1, min_kmh=10.0, min_gap_kmh=0.0, drop_kmh=6.0, **kw)
    assert 15.0 not in out and 25.0 in out
    # 間隔 15 km/h: 25 → 35 は 10 で外れ、25 の次は 65
    out = rp.plan_release_stations(c, 1, min_kmh=10.0, min_gap_kmh=15.0, drop_kmh=0.0, **kw)
    assert out[:2] == (15.0, 35.0)
    assert all(b - a >= 15.0 - 1e-9 for a, b in zip(out, out[1:], strict=False))
    # 上端までの間隔: 130 は top 140 − 130 = 10 < 15 で外れる
    assert 130.0 not in out
    # 段の番号が範囲外・release_s が空なら空
    assert rp.plan_release_stations(c, 5, min_kmh=0.0, min_gap_kmh=0.0, drop_kmh=0.0, **kw) == ()
    empty = rp.count_bands([], LEVELS)
    got = rp.plan_release_stations(
        empty, 1, min_kmh=0.0, min_gap_kmh=0.0, drop_kmh=0.0, **kw
    )
    assert got == ()


def test_plan_ramps_release_levels_estimate_and_execution_order() -> None:
    """いちばん弱い段以外に戻しが付き、見込み秒数に戻しの分が足され、execution_order の 5 つ目に
    出る。fracs が空なら今までどおり。"""
    c = _release_counts()
    kw = dict(
        mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9, turn_s=1.0, cruise_min_kmh=15.0,
        cruise_fine_max_kmh=100.0, cruise_coarse_step_kmh=20.0,
    )
    plain = rp.plan_ramps(c, **kw)
    rel = rp.plan_ramps(
        c, **kw, release_fracs=(0.5, 0.0), release_move_s=1.0, release_hold_s=(1.5, 2.0)
    )
    assert all(lv.release_stations == () for lv in plain.levels)
    assert rel.levels[0].g == 0.02 and rel.levels[0].release_stations == ()  # 弱い段は対象外
    withrel = [lv for lv in rel.levels if lv.release_stations]
    assert withrel
    lv = withrel[0]
    base = next(x for x in plain.levels if x.g == lv.g)
    assert lv.est_s == (
        pytest.approx(base.est_s + len(lv.release_stations) * 6.5)  # 移動 2 ＋ 保持 3.5 ＋ 切替 1
    )
    item = next(x for x in rel.execution_order() if x[0] == lv.g)
    assert item[4] == lv.release_stations
    assert "戻し" in rp.format_plan_table(rel)
    assert "戻し" in rp.format_band_table(c)


def test_plan_ramps_dither_estimate_is_added_only_when_enabled_and_for_weak_levels() -> None:
    """2026-10-28 段2: dither の見込み（上限）。off なら今と同じ。on なら目標減速度が
    dither_max_g 以下の段にブレーキの測る車速の数、保持のある段にアクセル min(max_n, 保持の数) 回、
    1 回 ＝ 4 × hold ＋ turn を足す。dither_s は周回を含む合計。"""
    c = _release_counts()
    kw = dict(
        mode_min_s=2.0, max_speed_kmh=140.0, budget_s=1e9, turn_s=1.0, cruise_min_kmh=15.0,
        cruise_fine_max_kmh=100.0, cruise_coarse_step_kmh=20.0, cruise_hold_s=7.0,
    )
    plain = rp.plan_ramps(c, **kw)
    off = rp.plan_ramps(c, **kw, dither_enabled=False, dither_brake_n=3, dither_accel_max_n=3)
    assert [lv.est_s for lv in off.levels] == [lv.est_s for lv in plain.levels]
    assert off.dither_s == 0.0 and plain.dither_s == 0.0
    on = rp.plan_ramps(
        c, **kw, dither_enabled=True, dither_hold_s=1.5, dither_brake_n=3, dither_max_g=0.1,
        dither_accel_max_n=3,
    )
    per = 4.0 * 1.5 + 1.0
    total = 0.0
    for lv, base in zip(on.levels, plain.levels, strict=True):
        want_brake = 3 if lv.g <= 0.1 else 0
        want_accel = min(3, len(lv.stations))
        assert (lv.dither_brake_n, lv.dither_accel_n) == (want_brake, want_accel)
        assert lv.est_s == pytest.approx(base.est_s + (want_brake + want_accel) * per)
        total += (want_brake + want_accel) * per * lv.passes
    assert total > 0.0 and on.dither_s == pytest.approx(total)
    assert on.total_est_s > plain.total_est_s
    text = rp.format_plan_table(on)
    assert "dither" in text and "dither" not in rp.format_plan_table(plain)


def test_plan_ramps_dither_can_push_the_first_pass_over_budget() -> None:
    """dither を足した結果 1 周目が枠を超えたら over_budget になり、表示にも出る。"""
    c = _release_counts()
    kw = dict(
        mode_min_s=2.0, max_speed_kmh=140.0, turn_s=1.0, cruise_min_kmh=15.0,
        cruise_fine_max_kmh=100.0, cruise_coarse_step_kmh=20.0, cruise_hold_s=7.0,
    )
    base = rp.plan_ramps(c, **kw, budget_s=1e9, max_passes=1)
    budget = base.total_est_s + 1.0
    assert not rp.plan_ramps(c, **kw, budget_s=budget, max_passes=1).over_budget
    on = rp.plan_ramps(
        c, **kw, budget_s=budget, max_passes=1, dither_enabled=True, dither_hold_s=1.5,
        dither_brake_n=3, dither_max_g=0.1, dither_accel_max_n=3,
    )
    assert on.over_budget and "枠を超えている" in rp.format_plan_table(on)
