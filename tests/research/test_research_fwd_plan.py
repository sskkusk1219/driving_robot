"""fwd_plan（基準車速 → 開度の列）のテスト。合成の表・基準だけを使う。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tests.research.fwd_map import POINT_FILLED, POINT_MEASURED, POINT_OUT, ForwardTable
from tests.research.fwd_plan import (
    FLAG_COAST,
    FLAG_FILLED,
    FLAG_OUT,
    FLAG_STOP,
    ForwardPlan,
    build_plan,
    window_slope_vec,
)
from tests.research.pedal_select import window_slope_kmhs

AD, BD, STOP_PCT = 4.0, 7.0, 18.0
ACCEL_SLOPE, BRAKE_SLOPE = 0.3, 0.6  # km/h/s per %


def linear_table(coast_at_zero: float = -2.0, v_coef: float = -0.01) -> ForwardTable:
    """a = coast0 + v_coef·v + (u≥0: 0.3u / u<0: 0.6u)。u の格子は −30..30（刻み 1）。"""
    rep = np.array([0.0, 50.0, 100.0, 150.0])
    grid = np.arange(-30.0, 31.0, 1.0)
    base = coast_at_zero + v_coef * rep[:, None]
    accel = base + np.where(grid >= 0, ACCEL_SLOPE * grid, BRAKE_SLOPE * grid)[None, :]
    kind = np.full(accel.shape, POINT_MEASURED, dtype=int)
    return ForwardTable(
        speed_edges=np.array([-25.0, 25.0, 75.0, 125.0, 175.0]),
        speed_rep=rep,
        u_grid=grid,
        accel_grid=accel,
        seconds=np.full(accel.shape, 10.0),
        point_kind=kind,
        meas_u_min=np.full(4, -30.0),
        meas_u_max=np.full(4, 30.0),
    )


def plan_for(ref, dur, table=None, **kw):
    kwargs = dict(
        accel_deadband_pct=AD,
        brake_deadband_pct=BD,
        stop_brake_opening_pct=STOP_PCT,
        stop_horizon_s=0.5,
    )
    kwargs.update(kw)
    return build_plan(ref, dur, table or linear_table(), **kwargs)


def idx(plan: ForwardPlan, t: float) -> int:
    return int(round(t / plan.period_s))


def test_constant_speed_cancels_coast_decel() -> None:
    p = plan_for(lambda t: 50.0, 2.0)
    coast = -2.0 - 0.01 * 50.0
    expected = AD + (0.0 - coast) / ACCEL_SLOPE
    assert p.effort_pct[idx(p, 1.0)] == pytest.approx(expected)
    assert p.need_accel_kmhs[idx(p, 1.0)] == pytest.approx(0.0)


def test_strong_decel_gives_brake() -> None:
    p = plan_for(lambda t: 80.0 - 10.0 * t, 4.0)
    t = 1.0
    coast = -2.0 - 0.01 * (80.0 - 10.0 * t)  # ブレーキは 0 秒先の車速
    expected = -(BD + abs((-10.0 - coast) / BRAKE_SLOPE))
    assert p.effort_pct[idx(p, t)] == pytest.approx(expected)


def test_exact_coast_gives_zero() -> None:
    table = linear_table(coast_at_zero=0.0, v_coef=0.0)
    p = plan_for(lambda t: 50.0, 1.0, table)
    assert np.all(p.effort_pct == 0.0)
    assert np.all(p.flag == FLAG_COAST)


def test_stop_hold_and_release_half_second_before_launch() -> None:
    def ref(t: float) -> float:
        return 0.0 if t <= 5.0 else min(2.0 * (t - 5.0), 10.0)

    p = plan_for(ref, 8.0)
    assert p.effort_pct[idx(p, 2.0)] == -STOP_PCT
    assert p.flag[idx(p, 2.0)] == FLAG_STOP
    assert p.effort_pct[idx(p, 4.4)] == -STOP_PCT  # ref(4.9) = 0 → まだ保持
    assert p.flag[idx(p, 4.6)] != FLAG_STOP  # ref(5.1) > 停車速度 → 保持が外れる
    assert p.effort_pct[idx(p, 4.6)] != -STOP_PCT


def test_speed_is_ref_now_and_need_is_window_at_t_plus_L() -> None:
    # 表の定義: 時刻 t の車速 ref(t) × 指令 → 時刻 t+L の加速度。車速は ref(t)、傾きは t+L 中心
    p = plan_for(lambda t: 20.0 + 5.0 * t, 3.0, accel_lag_s=0.30, brake_lag_s=0.0)
    t = 1.0
    coast = -2.0 - 0.01 * (20.0 + 5.0 * t)  # 車速は ref(t)（t+0.3 ではない）
    assert p.effort_pct[idx(p, t)] == pytest.approx(AD + (5.0 - coast) / ACCEL_SLOPE)
    seen: list[float] = []

    def rec(x: float) -> float:
        seen.append(round(x, 6))
        return 50.0 - 10.0 * x

    plan_for(rec, 0.0, accel_lag_s=0.30, brake_lag_s=0.0, need_window_s=1.0, stop_horizon_s=0.7)
    assert 0.0 in seen and 0.7 in seen
    assert -0.2 in seen and 0.8 in seen  # アクセルの窓 [t+0.3−0.5, t+0.3+0.5]
    assert -0.5 in seen and 0.5 in seen  # ブレーキの窓 [t−0.5, t+0.5]


def test_plan_uses_lag_stored_in_table_and_can_be_overridden() -> None:
    table = linear_table()
    table.meta.update({"accel_lag_s": 0.5, "brake_lag_s": 0.2})
    seen: list[float] = []

    def rec(x: float) -> float:
        seen.append(round(x, 6))
        return 50.0

    plan_for(rec, 0.0, table)
    assert 0.0 in seen and 1.0 in seen  # アクセルの窓 [0.5−0.5, 0.5+0.5]
    assert -0.3 in seen and 0.7 in seen  # ブレーキの窓 [0.2−0.5, 0.2+0.5]
    seen.clear()
    plan_for(rec, 0.0, table, accel_lag_s=0.0, brake_lag_s=0.0)
    assert 1.0 not in seen and 0.5 in seen


def test_coast_band_inside_is_zero_and_band_zero_matches_plain_rule() -> None:
    def ref(t: float) -> float:
        return 60.0 + 8.0 * math.sin(t / 2.0)

    table = linear_table()
    # 帯の内側: 要る加速度が惰行の加速度から ±0.25 以内 → 0
    coast = -2.0 - 0.01 * 50.0
    flat = plan_for(lambda t: 50.0 + coast * t, 0.5, coast_band_kmhs=0.25, accel_lag_s=0.0)
    assert np.all(flat.effort_pct == 0.0)
    # band=0: u の符号だけで決める従来の規則（遅れが同じとき）と一致
    p0 = plan_for(ref, 20.0, table, coast_band_kmhs=0.0, accel_lag_s=0.2, brake_lag_s=0.2)
    t = p0.t_s
    v = np.array([ref(x) for x in t])
    need = window_slope_vec(ref, t, 0.2, 1.0)
    u, _ = table.invert(v, need)
    expect = np.where(u > 0, AD + u, np.where(u < 0, -(BD + np.abs(u)), 0.0))
    assert np.allclose(p0.effort_pct, expect)
    # 帯を広げると 0 の点が増える
    p1 = plan_for(ref, 20.0, table, coast_band_kmhs=0.5, accel_lag_s=0.2, brake_lag_s=0.2)
    assert np.count_nonzero(p1.effort_pct == 0.0) > np.count_nonzero(p0.effort_pct == 0.0)


def test_simulation_with_lookahead_returns_to_ref_and_without_lags_by_L() -> None:
    from tests.research.fwd_plan import effective_u_series, simulate_speed

    ref = lambda t: 20.0 + 5.0 * min(max(t - 2.0, 0.0), 3.0)  # noqa: E731 - 2〜5 秒で加速
    L = 0.3
    table = linear_table()
    table.meta.update({"accel_lag_s": L, "brake_lag_s": L})
    dur = 8.0
    for look, expect_lag in ((True, 0.0), (False, L)):
        p = plan_for(ref, dur, table, accel_lag_s=L if look else 0.0,
                     brake_lag_s=L if look else 0.0, coast_band_kmhs=0.0)
        u = effective_u_series(p.effort_pct, p.flag, AD, BD)
        vs = simulate_speed(table, p.t_s, u, 0, len(p.t_s), ref(0.0), L, L)
        r = np.array([ref(x) for x in p.t_s])
        mid = (p.t_s > 3.5) & (p.t_s < 4.8)
        if look:
            assert np.max(np.abs(vs - r)[(p.t_s > 0.5)]) < 0.6  # 窓 1 秒のなまし分
        else:
            # 先読みなし: 車速は基準より約 L 秒ぶん遅れる（傾き 5 km/h/s なら 5 × L）
            assert np.mean((r - vs)[mid]) == pytest.approx(5.0 * expect_lag, abs=0.3)


def test_vector_window_slope_matches_scalar() -> None:
    def ref(t: float) -> float:
        return 40.0 + 15.0 * math.sin(t / 3.0) + (3.0 if t > 7.35 else 0.0)

    ts = np.array([0.0, 0.05, 1.25, 7.0, 7.2, 12.34, 30.0])
    for center, width in ((0.30, 1.0), (0.0, 1.0), (0.5, 3.0)):
        vec = window_slope_vec(ref, ts, center, width)
        one = [window_slope_kmhs(ref, float(t), center, width) for t in ts]
        assert np.allclose(vec, one, atol=1e-9)


def test_aggregates_and_switch_count() -> None:
    dt = 0.05
    effort = np.array([10.0, 0.0, -12.0, -12.0, 5.0, -18.0, -18.0, 7.0])
    flag = np.array([0, FLAG_COAST, FLAG_OUT, FLAG_FILLED, 0, FLAG_STOP, FLAG_STOP, FLAG_OUT])
    p = ForwardPlan(
        t_s=np.arange(len(effort)) * dt,
        effort_pct=effort,
        need_accel_kmhs=np.zeros(len(effort)),
        flag=flag,
        period_s=dt,
    )
    assert p.out_of_range_s() == pytest.approx(2 * dt)
    assert p.filled_s() == pytest.approx(1 * dt)
    # A, (C), B, B, A, B(停車), B(停車), A → 切替 A→B, B→A, A→B, B→A = 4
    assert p.pedal_switches() == 4
    # 停車保持を飛ばすと A, B, B, A, A → 2
    assert p.pedal_switches(include_stop=False) == 2
    s = p.pedal_seconds()
    assert s["アクセル"] == pytest.approx(3 * dt)
    assert s["ブレーキ"] == pytest.approx(2 * dt)
    assert s["惰行"] == pytest.approx(1 * dt)
    assert s["停車保持"] == pytest.approx(2 * dt)


def test_at_returns_nearest_sample() -> None:
    p = ForwardPlan(
        t_s=np.array([0.0, 0.05, 0.10]),
        effort_pct=np.array([1.0, 2.0, 3.0]),
        need_accel_kmhs=np.zeros(3),
        flag=np.zeros(3, dtype=int),
    )
    assert p.at(0.0) == 1.0
    assert p.at(0.04) == 2.0
    assert p.at(0.074) == 2.0
    assert p.at(0.08) == 3.0
    assert p.at(-1.0) == 1.0 and p.at(99.0) == 3.0


def test_out_of_range_and_filled_flags_come_from_table() -> None:
    t = linear_table()
    t.point_kind[:, :] = POINT_FILLED
    p = plan_for(lambda x: 50.0, 1.0, t)
    assert p.filled_s() > 0.0 and p.out_of_range_s() == 0.0
    p = plan_for(lambda x: 50.0 - 10.0 * x, 1.0, linear_table(), accel_lag_s=0.3)
    assert p.out_of_range_s() == 0.0
    # 要る加速度が表の範囲を超える（−1000 km/h/s）→ 範囲外
    p = plan_for(lambda x: 100.0 - 1000.0 * x, 0.2)
    assert p.out_of_range_s() > 0.0
    assert POINT_OUT == FLAG_OUT
