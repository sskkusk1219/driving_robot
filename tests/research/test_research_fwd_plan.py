"""fwd_plan（基準車速 → 開度の列）のテスト。合成の表・基準だけを使う。"""

from __future__ import annotations

import math
import types

import numpy as np
import pytest

from tests.research.fwd_map import POINT_FILLED, POINT_MEASURED, POINT_OUT, ForwardTable
from tests.research.fwd_plan import (
    FLAG_COAST,
    FLAG_FILLED,
    FLAG_MEASURED,
    FLAG_OUT,
    FLAG_PREPOS,
    FLAG_STOP,
    SIDE_LAUNCH,
    SIDE_MAIN,
    SIDE_RELEASE,
    SIDE_STOP,
    SIDE_STOP_RELEASE,
    ForwardPlan,
    build_plan,
    coast_assist_effort,
    deviation_offset,
    deviation_offset_detail,
    low_speed_sides,
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
    kwargs.setdefault("return_gap_enabled", False)  # 段2o-9 のすき間の確保は別のテストで見る
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


def _launch_variant() -> ForwardTable:
    """発進側の配列: v=0 の行を全体に +6 km/h/s 持ち上げる（車速 10 では重み 0.8 で効く）。"""
    t = linear_table()
    t.launch_accel = t.accel_grid.copy()
    t.launch_accel[0] += 6.0
    return t


def test_launch_accel_none_gives_identical_plan() -> None:
    ref = lambda t: max(0.0, 3.0 * (t - 1.0)) if t < 5.0 else 12.0  # noqa: E731
    base = plan_for(ref, 6.0)
    t = linear_table()
    t.launch_accel = None
    same = plan_for(ref, 6.0, t)
    assert np.array_equal(base.effort_pct, same.effort_pct)
    assert np.array_equal(base.flag, same.flag)


def test_launch_table_used_only_when_reference_rises() -> None:
    # 基準が 10 km/h から緩く上がり、その後同じ傾きで下がる
    def ref(t: float) -> float:
        return 10.0 + 0.5 * t if t < 2.0 else 11.0 - 0.5 * (t - 2.0)

    base = plan_for(ref, 4.0)
    plan = plan_for(ref, 4.0, _launch_variant())
    up, down = idx(plan, 1.0), idx(plan, 3.0)
    assert base.effort_pct[up] > 0.0  # 本体: 上がる基準はアクセル
    assert plan.effort_pct[up] < 0.0  # 発進側は惰行の加速度が高いのでブレーキ
    assert plan.effort_pct[down] == pytest.approx(base.effort_pct[down])  # 下がる所は本体


# ───────── 2026-10-18 段2o-10: 停止側の表 ─────────


def _stop_variant(max_speed: float = 25.0) -> ForwardTable:
    """停止側の配列: v=0 の行を全体に +6 km/h/s 持ち上げる。引く車速の上限は max_speed。"""
    t = linear_table()
    t.stop_accel = t.accel_grid.copy()
    t.stop_accel[0] += 6.0
    t.meta["stop_max_speed_kmh"] = max_speed
    return t


def _decel_to_stop(t: float) -> float:
    return max(0.0, 10.0 - 2.0 * t)  # t = 5 s で 0。stop_horizon_s=0.5 なので 4.75 s から停車保持


def test_stop_accel_none_gives_identical_plan() -> None:
    base = plan_for(_decel_to_stop, 6.0)
    t = linear_table()
    t.stop_accel = None
    t.meta["stop_max_speed_kmh"] = 25.0
    same = plan_for(_decel_to_stop, 6.0, t)
    assert np.array_equal(base.effort_pct, same.effort_pct)
    assert np.array_equal(base.flag, same.flag)


def test_stop_table_used_only_when_decelerating_to_stop_within_range() -> None:
    base = plan_for(_decel_to_stop, 6.0)
    plan = plan_for(_decel_to_stop, 6.0, _stop_variant())
    near, far = idx(plan, 4.5), idx(plan, 2.0)  # 4.5 s は ref(5.0)=0 で停車へ向かう、2 s は違う
    assert plan.stops_ahead is not None and plan.stops_ahead[near] and not plan.stops_ahead[far]
    assert plan.effort_pct[far] == pytest.approx(base.effort_pct[far])  # 停車の先が遠い所は本体
    assert plan.effort_pct[near] < base.effort_pct[near]  # 停止側は惰行が高く、ブレーキが要る
    assert plan.flag[idx(plan, 5.5)] == FLAG_STOP  # 停車保持は本体のまま
    # 基準車速が停止側の表の範囲（上限）以上なら本体
    out = plan_for(_decel_to_stop, 6.0, _stop_variant(max_speed=0.5))
    assert out.effort_pct[near] == pytest.approx(base.effort_pct[near])
    # 基準が上がっている所は停止側を引かない（発進側の領分）
    up = plan_for(lambda t: min(2.0 * t, 10.0), 6.0, _stop_variant())
    up_base = plan_for(lambda t: min(2.0 * t, 10.0), 6.0)
    assert np.array_equal(up.effort_pct, up_base.effort_pct)


def test_deviation_uses_the_same_stop_table_as_build_plan() -> None:
    t = linear_table()
    t.stop_accel = t.accel_grid.copy()
    cols = t.u_grid < 0
    base_col = (-2.0 - 0.01 * t.speed_rep)[:, None]
    t.stop_accel[:, cols] = (t.accel_grid[:, cols] - base_col) * 2.0 + base_col  # 傾き 2 倍
    t.meta["stop_max_speed_kmh"] = 25.0
    plain = _dev(linear_table(), effort=-15.0, need=-3.0, ref=10.0, dev=1.0)
    stop = _dev(t, effort=-15.0, need=-3.0, ref=10.0, dev=1.0, stops_ahead=True)
    assert stop - (-15.0) == pytest.approx((plain - (-15.0)) / 2.0)  # 傾き 2 倍なので上乗せは半分
    # 停車へ向かわない・範囲外の車速・減速でない・stops_ahead 省略は本体
    assert _dev(t, effort=-15.0, need=-3.0, ref=10.0, dev=1.0) == pytest.approx(plain)
    assert _dev(t, effort=-15.0, need=-3.0, ref=40.0, dev=1.0, stops_ahead=True) == pytest.approx(
        _dev(linear_table(), effort=-15.0, need=-3.0, ref=40.0, dev=1.0)
    )
    # low_speed_sides は build_plan と同じ判定を返す
    launch, stop_m = low_speed_sides(t, np.array([-3.0, 3.0]), np.array([10.0, 10.0]), True)
    assert not launch.any() and list(stop_m) == [True, False]


# ───────── 2026-10-14 段2o-5: 戻し量を使った解き直し ─────────


def _release_ref(t: float) -> float:
    """80 km/h から −10 km/h/s で 2 秒（強いブレーキ）→ −3 km/h/s で 8 秒（浅くして戻す）。"""
    return 80.0 - 10.0 * t if t < 2.0 else 60.0 - 3.0 * (t - 2.0)


def _release_plan(table):
    return plan_for(_release_ref, 8.0, table)


def test_plan_without_release_table_is_unchanged() -> None:
    from tests.research.test_research_fwd_map import release_table  # noqa: PLC0415

    plain = release_table(with_release=False)
    base = _release_plan(plain)
    # 戻しの列が付いていても release_grid が None なら同じ（付かなかった表と同じ）
    tb = release_table()
    tb.release_grid = None
    tb.release_accel = None
    same = _release_plan(tb)
    assert np.array_equal(same.effort_pct, base.effort_pct)
    assert np.array_equal(same.flag, base.flag)
    # 頭打ちが 0 なら戻し量は常に 0 ＝ 解き直しても結果は変わらない
    zero = release_table()
    zero.meta["release_max_pct"] = 0.0
    res = _release_plan(zero)
    assert np.allclose(res.effort_pct, base.effort_pct, atol=1e-9)
    assert np.array_equal(res.flag, base.flag)


def test_plan_with_weaker_braking_on_release_brakes_deeper_when_releasing() -> None:
    from tests.research.test_research_fwd_map import REL_SLOPE, release_table  # noqa: PLC0415

    base = _release_plan(release_table(with_release=False))
    tb = release_table()
    tb.meta["release_max_pct"] = 8.0
    plan = _release_plan(tb)
    # 山の間（戻す前）は同じ
    for t in (0.5, 1.0, 1.4):
        assert plan.effort_pct[idx(plan, t)] == pytest.approx(base.effort_pct[idx(base, t)])
    # 戻している間は、戻しで効きが弱まる表なので踏みが深い（effort がより負）
    for t in (3.0, 3.5, 4.0):
        i = idx(plan, t)
        assert plan.effort_pct[i] < base.effort_pct[i] - 0.3, t
        # 解き直した u が表を満たす: accel(v, u, release = u − m) = 要る加速度
        w = int(round(3.0 / plan.period_s))
        u_all = np.where(plan.effort_pct < -BD, plan.effort_pct + BD, 0.0)
        m = float(u_all[max(i - w, 0) : i].min())
        u = float(u_all[i])
        assert u > m  # 戻している
        got = tb.accel(_release_ref(t), u, release=u - m)
        assert got == pytest.approx(plan.need_accel_kmhs[i], abs=0.02)
    assert REL_SLOPE > 0
    # 戻しに入らない時刻（アクセル側）は影響されない
    up = plan_for(lambda t: 30.0 + 3.0 * t, 4.0, tb)
    up0 = plan_for(lambda t: 30.0 + 3.0 * t, 4.0, release_table(with_release=False))
    assert np.array_equal(up.effort_pct, up0.effort_pct)


# ─── 先置き（2026-10-16 段2o-6） ────────────────────────────────────────

DT = 0.05


def _pre(effort_list, accel_pre=5.0, brake_pre=8.0, lead=0.2, clear=0.1, flag=None):
    from tests.research.fwd_plan import FLAG_PREPOS, ReturnTime, preposition  # noqa: PLC0415

    effort = np.array(effort_list, dtype=float)
    fl = np.full(len(effort), FLAG_COAST, dtype=int) if flag is None else np.array(flag)
    # 戻る時間 T が深さによらず clear 秒になる ReturnTime（T = 遅れ + 0 + 1 周期 = clear）
    ret = ReturnTime(
        accel_min_pct=0.0, brake_min_pct=0.0, accel_delay_s=clear - DT, brake_delay_s=clear - DT,
        accel_rate_pct_s=1e12, brake_rate_pct_s=1e12, accel_default=False, brake_default=False,
    )
    st = preposition(
        effort, fl, accel_pre_pct=accel_pre, brake_pre_pct=brake_pre,
        lead_s=lead, ret=ret, period_s=DT,
    )
    return effort, fl, st, FLAG_PREPOS


def test_preposition_starts_exactly_lead_before_press() -> None:
    # 0 が 10 点続いたあとアクセル 12% → 4 点（0.2 秒）前から 5% を置く
    e, fl, st, PRE = _pre([0.0] * 10 + [12.0, 12.0])
    assert list(e[:6]) == [0.0] * 6
    assert list(e[6:10]) == [5.0] * 4
    assert list(fl[6:10]) == [PRE] * 4 and fl[5] == FLAG_COAST
    assert (st.n_placed, st.n_short, st.n_no_time) == (1, 0, 0)
    assert st.prepos_s == pytest.approx(0.2)


def test_preposition_brake_sign() -> None:
    e, _, st, _ = _pre([0.0] * 10 + [-12.0])
    assert list(e[6:10]) == [-8.0] * 4 and st.n_placed == 1


def test_preposition_waits_clear_after_other_pedal_release() -> None:
    # ブレーキ指令が 2 点前に min になった: 0 は 3 点、clear=0.1(2点) 待つと 1 点だけ
    e, _, st, _ = _pre([-10.0, -10.0, 0.0, 0.0, 0.0, 12.0])
    assert list(e) == [-10.0, -10.0, 0.0, 0.0, 5.0, 12.0]
    assert (st.n_placed, st.n_short, st.n_no_time) == (0, 1, 0)
    assert st.prepos_s == pytest.approx(DT)


def test_preposition_no_time_when_no_room() -> None:
    # 停車保持（ブレーキ）→ 発進（アクセル）は 0 を挟まない: 置く場所なし
    e, _, st, _ = _pre([-18.0, -18.0, 12.0], flag=[FLAG_STOP, FLAG_STOP, 0])
    assert list(e) == [-18.0, -18.0, 12.0] and st.n_no_time == 1 and st.prepos_s == 0.0
    # 0 が 1 点だけで clear に食われる場合も間に合わず
    e, _, st, _ = _pre([-10.0, 0.0, 12.0])
    assert list(e) == [-10.0, 0.0, 12.0] and st.n_no_time == 1


def test_preposition_no_onset_and_not_needed_leave_effort_unchanged() -> None:
    base = [0.0] * 10 + [12.0, 12.0]
    e, _, st, _ = _pre(base, accel_pre=None)
    assert list(e) == base and st.n_no_onset == 1
    e, _, st, _ = _pre(base, accel_pre=12.0)  # 置く開度が踏み出し以上
    assert list(e) == base and st.n_not_needed == 1


def test_preposition_does_not_overwrite_nonzero_and_start_of_series() -> None:
    # 系列の先頭から 0 が 3 点: lead 4 点だが先頭までしか遡らない
    e, _, st, _ = _pre([0.0, 0.0, 0.0, 12.0])
    assert list(e) == [5.0, 5.0, 5.0, 12.0] and st.n_short == 1 and st.n_placed == 0


def _wltp_like_ref(t: float) -> float:
    return 30.0 + 10.0 * math.sin(t / 3.0)


def test_build_plan_preposition_disabled_is_identical_and_switches_unchanged() -> None:
    table = linear_table()
    table.meta["accel_onset_u_pct"] = 3.0
    table.meta["brake_onset_u_pct"] = 2.0
    off = plan_for(_wltp_like_ref, 40.0, table)
    on = plan_for(_wltp_like_ref, 40.0, table, preposition_enabled=True)
    assert off.preposition is None and on.preposition is not None
    assert on.preposition.prepos_s == pytest.approx(on.pedal_seconds()["先置き"])
    # 先置きの行以外は同じ列
    from tests.research.fwd_plan import FLAG_PREPOS  # noqa: PLC0415

    keep = on.flag != FLAG_PREPOS
    assert np.array_equal(on.effort_pct[keep], off.effort_pct[keep])
    assert on.pedal_switches() == off.pedal_switches()
    # 効き始めが無ければ有効でも同じ
    t2 = linear_table()
    same = plan_for(_wltp_like_ref, 40.0, t2, preposition_enabled=True)
    assert np.array_equal(same.effort_pct, off.effort_pct)
    assert same.preposition.n_no_onset > 0


# ─── 切り替えのすき間（2026-10-17 段2o-9） ───────────────────────────────

def _ret(delay=0.1, rate=100.0, amin=4.0, bmin=4.0):
    from tests.research.fwd_plan import ReturnTime  # noqa: PLC0415

    return ReturnTime(
        accel_min_pct=amin, brake_min_pct=bmin, accel_delay_s=delay, accel_rate_pct_s=rate,
        brake_delay_s=delay, brake_rate_pct_s=rate, accel_default=False, brake_default=False,
    )


def _gap(effort_list, flag=None, ret=None):
    from tests.research.fwd_plan import ensure_return_gap  # noqa: PLC0415

    effort = np.array(effort_list, dtype=float)
    fl = np.full(len(effort), FLAG_COAST, dtype=int) if flag is None else np.array(flag)
    st = ensure_return_gap(effort, fl, ret=ret or _ret(), period_s=DT)
    return effort, fl, st


def test_return_time_depends_on_depth_and_includes_one_period() -> None:
    r = _ret()
    # 深さ 0（min ちょうど）: 0.1 + 0 + 0.05 = 0.15s → 3 行
    assert r.seconds(4.0, DT) == pytest.approx(0.15) and r.gap_rows(4.0, DT) == 3
    # 深さ 20%: 0.1 + 0.2 + 0.05 = 0.35s → 7 行（浮動小数の端数で 8 にならない）
    assert r.seconds(24.0, DT) == pytest.approx(0.35) and r.gap_rows(24.0, DT) == 7
    assert r.gap_rows(-24.0, DT) == 7  # ブレーキも同じ式


def test_return_time_of_uses_meta_or_defaults() -> None:
    from tests.research.fwd_plan import (  # noqa: PLC0415
        DEFAULT_RETURN_DELAY_S,
        DEFAULT_RETURN_RATE_PCT_S,
        return_time_of,
    )

    t = linear_table()
    r = return_time_of(t, AD, BD)
    assert r.defaults_used() == ("アクセル", "ブレーキ")
    assert r.accel_delay_s == DEFAULT_RETURN_DELAY_S
    assert r.brake_rate_pct_s == DEFAULT_RETURN_RATE_PCT_S
    t.meta.update({"accel_return_delay_s": 0.1, "accel_return_rate_pct_s": 99.0})
    r = return_time_of(t, AD, BD)
    assert r.defaults_used() == ("ブレーキ",) and r.accel_rate_pct_s == 99.0
    # 片方のキーだけでは測定値とみなさない
    t.meta.pop("accel_return_rate_pct_s")
    assert return_time_of(t, AD, BD).defaults_used() == ("アクセル", "ブレーキ")


def test_gap_direct_switch_cuts_leaving_tail() -> None:
    # アクセル 24%（深さ 20 → T 7 行）の直後にブレーキ。すき間 0 → 戻す側の最後を前倒しで 0 に
    acc = [24.0] * 20
    e, fl, st = _gap(acc + [-24.0] * 5)
    assert st.n_switches == 1 and st.n_cut == 1 and st.n_delayed == 0 and st.n_violations == 0
    assert st.cut_s == pytest.approx(7 * DT)
    assert np.all(e[13:20] == 0.0) and np.all(fl[13:20] == FLAG_COAST)
    assert np.all(e[:13] == 24.0) and np.all(e[20:] == -24.0)


def test_gap_short_coast_is_extended_by_cutting() -> None:
    # 惰行 2 行しかない: あと 5 行ぶん前倒し
    e, _, st = _gap([24.0] * 20 + [0.0] * 2 + [-24.0] * 5)
    assert st.n_cut == 1 and st.cut_s == pytest.approx(5 * DT) and st.n_violations == 0
    assert np.all(e[15:22] == 0.0) and e[14] == 24.0


def test_gap_recomputes_depth_while_cutting() -> None:
    # 末尾が浅い（深さ 5 → T 0.1+0.05+0.05=0.2s=4 行）ので 4 行ぶん削れば足りる
    # 深い所（24%）まで食い込まない
    e, _, st = _gap([24.0] * 10 + [9.0] * 6 + [-24.0] * 5)
    assert st.n_cut == 1 and st.n_violations == 0
    assert np.all(e[:10] == 24.0)
    assert np.count_nonzero(e[10:16] == 0.0) == 4 and st.cut_s == pytest.approx(4 * DT)


def test_gap_sufficient_is_untouched() -> None:
    base = [24.0] * 5 + [0.0] * 7 + [-24.0] * 5
    e, _, st = _gap(base)
    assert list(e) == base and st.n_switches == 1 and st.n_cut == 0 and st.n_delayed == 0
    # 同じペダルの間の惰行は切り替えではない
    e, _, st = _gap([24.0] * 3 + [0.0] + [24.0] * 3)
    assert st.n_switches == 0


def test_gap_stop_hold_to_launch_delays_entering_pedal() -> None:
    # 停車保持（ブレーキ −18、深さ 14 → T = 0.1+0.14+0.05 = 0.29s → 6 行）→ 発進。停車保持は
    # 早く離さず、アクセルを遅らせる
    e, fl, st = _gap([-18.0] * 10 + [12.0] * 10, flag=[FLAG_STOP] * 10 + [FLAG_COAST] * 10)
    assert np.all(e[:10] == -18.0) and np.all(fl[:10] == FLAG_STOP)
    assert np.all(e[10:16] == 0.0) and np.all(e[16:] == 12.0)
    assert st.n_delayed == 1 and st.n_cut == 0 and st.delayed_s == pytest.approx(6 * DT)
    assert st.n_violations == 0


def test_gap_delays_when_leaving_run_too_short_to_cut() -> None:
    # 戻す側が 2 行しかなく、先頭 1 行を残すと足りない → 次のペダルを遅らせる
    e, _, st = _gap([0.0, 24.0, 24.0, -24.0, -24.0] + [-24.0] * 10)
    assert np.all(e[1:3] == 24.0)
    assert st.n_delayed == 1 and st.n_cut == 0 and st.n_violations == 0
    assert np.all(e[3:10] == 0.0) and e[10] == -24.0


def test_gap_entering_run_shorter_than_delay_disappears_cleanly() -> None:
    # 停車保持→発進が 2 行だけ: 遅らせる行数が連なりを超える → 連なりは消え、違反は残らない
    e, _, st = _gap([-18.0] * 4 + [12.0] * 2 + [0.0] * 3, flag=[FLAG_STOP] * 4 + [FLAG_COAST] * 5)
    assert np.all(e[4:] == 0.0) and st.n_violations == 0


def test_gap_handles_chain_of_switches_and_counts_zero_violations() -> None:
    from tests.research.fwd_plan import count_gap_violations  # noqa: PLC0415

    seq = ([24.0] * 6 + [-20.0] * 6 + [30.0] * 6 + [0.0] + [-10.0] * 6) * 3
    e, _, st = _gap(seq)
    assert st.n_switches > 0 and st.n_violations == 0
    assert count_gap_violations(e, ret=_ret(), period_s=DT) == 0
    assert count_gap_violations(np.array(seq), ret=_ret(), period_s=DT) > 0


def test_build_plan_gap_default_on_and_disabled_flag() -> None:
    table = linear_table()
    table.meta["accel_onset_u_pct"] = 3.0
    table.meta["brake_onset_u_pct"] = 2.0
    on = build_plan(
        _wltp_like_ref, 40.0, table, accel_deadband_pct=AD, brake_deadband_pct=BD,
        stop_brake_opening_pct=STOP_PCT, stop_horizon_s=0.5, preposition_enabled=True,
    )
    off = plan_for(_wltp_like_ref, 40.0, table, preposition_enabled=True)
    assert on.return_gap is not None and on.return_gap.n_violations == 0
    assert off.return_gap is None
    assert "仮値" in on.return_gap.summary()  # meta に戻る時間が無い
    # 先置きは戻る時間の後にだけ置かれる（すき間不足の切り替えは残らない）
    from tests.research.fwd_plan import count_gap_violations, return_time_of  # noqa: PLC0415

    ret = return_time_of(table, AD, BD)
    assert count_gap_violations(on.effort_pct, ret=ret, period_s=DT) == 0


# ─── C10: 偏差の上乗せ（deviation_offset。2026-10-17 段2p） ─────────────────


def _dev(table, *, effort, need=0.0, ref=50.0, row=None, dev=0.0, flag=FLAG_MEASURED,
         tau_a=5.0, tau_b=5.0, **kw):
    return deviation_offset(
        table, effort=effort, flag=flag, need_kmhs=need, ref_kmh=ref,
        row_kmh=ref if row is None else row, dev_kmh=dev, tau_accel_s=tau_a, tau_brake_s=tau_b,
        accel_min_pct=AD, brake_min_pct=BD, **kw,
    )


def test_deviation_zero_returns_effort_unchanged_for_both_cases() -> None:
    t = linear_table()
    for e in (12.0, -15.0):
        assert _dev(t, effort=e, need=0.5) == e
        # 案B（実車速の行）でも dev=0 なら実車速＝基準車速なので同じ
        assert _dev(t, effort=e, need=0.5, ref=47.0, row=47.0) == e


def test_deviation_behind_adds_throttle_ahead_reduces_and_stops_at_min() -> None:
    t = linear_table()
    # dev=+2 km/h・τ=5s → Δa=0.4 km/h/s → Δu=0.4/0.3
    assert _dev(t, effort=12.0, dev=2.0) == pytest.approx(12.0 + 0.4 / ACCEL_SLOPE)
    # 進んでいれば減る（Δu=−1.0/0.3）。深く進むと min（AD）で止まりブレーキ側へ行かない
    assert _dev(t, effort=12.0, dev=-1.5) == pytest.approx(12.0 - 0.3 / ACCEL_SLOPE)
    assert _dev(t, effort=6.0, dev=-20.0) == AD
    _, clamped = deviation_offset_detail(
        t, effort=6.0, flag=FLAG_MEASURED, need_kmhs=0.0, ref_kmh=50.0, row_kmh=50.0,
        dev_kmh=-20.0, tau_accel_s=5.0, tau_brake_s=5.0, accel_min_pct=AD, brake_min_pct=BD,
    )
    assert clamped


def test_deviation_brake_row_behind_makes_brake_shallower_and_stops_at_min() -> None:
    t = linear_table()
    e = -15.0
    out = _dev(t, effort=e, need=-3.0, dev=1.0)  # 遅れている → ブレーキを浅く（0.2/0.6 浅く）
    assert out == pytest.approx(e + 0.2 / BRAKE_SLOPE)
    assert _dev(t, effort=e, need=-3.0, dev=-1.0) == pytest.approx(e - 0.2 / BRAKE_SLOPE)  # 深く
    assert _dev(t, effort=-9.0, need=-3.0, dev=50.0) == -BD  # アクセルには移らない


def test_deviation_skips_coast_stop_prepos_zero_and_missing_tau() -> None:
    t = linear_table()
    for flag in (FLAG_COAST, FLAG_STOP, FLAG_PREPOS):
        assert _dev(t, effort=12.0, dev=3.0, flag=flag) == 12.0
    assert _dev(t, effort=0.0, dev=3.0) == 0.0
    assert _dev(t, effort=12.0, dev=3.0, tau_a=None) == 12.0  # アクセルの τ なし
    assert _dev(t, effort=-12.0, need=-3.0, dev=3.0, tau_b=None) == -12.0
    assert _dev(t, effort=12.0, dev=3.0, tau_b=None) != 12.0  # 反対のペダルの τ は関係ない


def test_deviation_uses_launch_table_when_need_positive() -> None:
    t = linear_table()
    # 発進側の表は傾きが 2 倍（ブレーキ側の「発進」と「停止へ」の違いを模す）
    t.launch_accel = t.accel_grid.copy()
    t.launch_accel[:, t.u_grid >= 0] = (
        t.launch_accel[:, t.u_grid >= 0] - (-2.0 - 0.01 * t.speed_rep[:, None])
    ) * 2.0 + (-2.0 - 0.01 * t.speed_rep[:, None])
    plain = _dev(linear_table(), effort=12.0, need=0.5, dev=2.0)
    launch = _dev(t, effort=12.0, need=0.5, dev=2.0)
    assert launch - 12.0 == pytest.approx((plain - 12.0) / 2.0)  # 傾き 2 倍なので上乗せは半分
    # need <= 0 のときは本体の表（発進側を引かない）
    assert _dev(t, effort=12.0, need=-0.5, dev=2.0) == pytest.approx(plain)


def test_deviation_case_b_lower_speed_row_gives_smaller_offset() -> None:
    # 速度が高いほど踏み込みの効きが大きい表（傾き 0.3·(1+v/100)）: 同じ Δa を出すのに要る
    # 開度の差は、効きが大きい高い車速の行ほど小さい。案B で実車速が低い（遅れている）と、
    # 表の行が低い車速になり、案A より上乗せが大きくなる（向きの確認。行が違うと惰行の差も
    # 入るので、近い車速どうしで比べる）。
    t = linear_table()
    slope = 0.3 * (1.0 + t.speed_rep / 100.0)[:, None]
    base = (-2.0 - 0.01 * t.speed_rep)[:, None]
    t.accel_grid = base + np.where(t.u_grid >= 0, slope * t.u_grid, 0.6 * t.u_grid)
    a_case = _dev(t, effort=12.0, need=0.5, ref=100.0, row=100.0, dev=2.0)
    b_slow = _dev(t, effort=12.0, need=0.5, ref=100.0, row=97.0, dev=2.0)
    assert 12.0 < a_case < b_slow  # 低い車速の行のほうが効きが小さい → 上乗せは大きい


# ─── C11: DOB の分（deviation_offset の dob_kmhs。2026-10-18 段2q） ───────────


def test_deviation_dob_zero_is_identical_to_c10() -> None:
    t = linear_table()
    for e, need in ((12.0, 0.5), (-15.0, -3.0)):
        base = _dev(t, effort=e, need=need, dev=1.0)
        assert _dev(t, effort=e, need=need, dev=1.0, dob_kmhs=0.0) == base


def test_deviation_dob_negative_accel_row_presses_more() -> None:
    t = linear_table()
    # 表より 0.3 km/h/s 加速していない（d=−0.3）→ 要る加速度に +0.3 → Δu=0.3/0.3=+1%
    assert _dev(t, effort=12.0, dev=0.0, dob_kmhs=-0.3) == pytest.approx(12.0 + 0.3 / ACCEL_SLOPE)
    # 逆（表より出ている）なら減る。深く出ていれば min で止まりブレーキへ移らない
    assert _dev(t, effort=12.0, dev=0.0, dob_kmhs=0.3) == pytest.approx(12.0 - 0.3 / ACCEL_SLOPE)
    assert _dev(t, effort=6.0, dev=0.0, dob_kmhs=50.0) == AD


def test_deviation_dob_positive_brake_row_goes_deeper() -> None:
    t = linear_table()
    # ブレーキ行で表より減速が浅い（d>0）→ need − d が負に深くなる → 深く踏む
    out = _dev(t, effort=-15.0, need=-3.0, dev=0.0, dob_kmhs=0.3)
    assert out == pytest.approx(-15.0 - 0.3 / BRAKE_SLOPE)
    # 表より減速が深い（d<0）なら浅くなり、-BD で止まってアクセルへ移らない
    assert _dev(t, effort=-15.0, need=-3.0, dev=0.0, dob_kmhs=-0.3) == pytest.approx(
        -15.0 + 0.3 / BRAKE_SLOPE
    )
    assert _dev(t, effort=-9.0, need=-3.0, dev=0.0, dob_kmhs=-50.0) == -BD


def test_deviation_dob_adds_to_p_and_skips_coast_and_missing_tau() -> None:
    t = linear_table()
    both = _dev(t, effort=12.0, dev=2.0, dob_kmhs=-0.3)
    assert both == pytest.approx(12.0 + (0.4 + 0.3) / ACCEL_SLOPE)
    for flag in (FLAG_COAST, FLAG_STOP, FLAG_PREPOS):
        assert _dev(t, effort=12.0, dob_kmhs=-0.3, flag=flag) == 12.0
    assert _dev(t, effort=0.0, dob_kmhs=-0.3) == 0.0
    assert _dev(t, effort=12.0, dob_kmhs=-0.3, tau_a=None) == 12.0  # τ なしのペダルは何もしない
    # τ=0（deviation_c=0）でも DOB は効き、偏差の項は 0
    assert _dev(t, effort=12.0, dev=5.0, dob_kmhs=-0.3, tau_a=0.0) == pytest.approx(
        12.0 + 0.3 / ACCEL_SLOPE
    )


# ─── C12: 惰行の補助の開度（coast_assist_effort。2026-10-18 段2r） ─────────────


def _coast(table, *, is_accel, dev, need=-2.5, ref=50.0, row=None, tau=5.0, **kw):
    return coast_assist_effort(
        table, is_accel=is_accel, need_kmhs=need, ref_kmh=ref,
        row_kmh=ref if row is None else row, dev_kmh=dev, tau_s=tau,
        accel_min_pct=AD, brake_min_pct=BD, **kw,
    )


def test_coast_assist_effort_signs_and_amounts() -> None:
    t = linear_table()
    # 遅れ +2 km/h・τ=5s → Δa=0.4 → アクセル min ＋ 0.4/0.3（正）
    assert _coast(t, is_accel=True, dev=2.0) == pytest.approx(AD + 0.4 / ACCEL_SLOPE)
    # 速すぎ −2 km/h → Δa=−0.4 → ブレーキ −(min ＋ 0.4/0.6)（負）
    assert _coast(t, is_accel=False, dev=-2.0) == pytest.approx(-(BD + 0.4 / BRAKE_SLOPE))


def test_coast_assist_effort_stops_at_min_and_never_switches_pedal() -> None:
    t = linear_table()
    # アクセルを選んだのに u が負になる（進んでいる）→ min で止まる（ブレーキ側へ移らない）
    assert _coast(t, is_accel=True, dev=-20.0) == AD
    assert _coast(t, is_accel=False, dev=50.0) == -BD


def test_coast_assist_effort_dob_and_zero_tau_and_clip() -> None:
    t = linear_table()
    # 表より加速していない（dob < 0）→ アクセルは踏み増す
    base = _coast(t, is_accel=True, dev=2.0)
    assert _coast(t, is_accel=True, dev=2.0, dob_kmhs=-0.3) == pytest.approx(
        base + 0.3 / ACCEL_SLOPE
    )
    # τ ≦ 0 なら偏差の項は 0（DOB だけ）
    assert _coast(t, is_accel=True, dev=9.0, tau=0.0) == pytest.approx(AD)
    assert _coast(t, is_accel=True, dev=9.0, tau=0.0, dob_kmhs=-0.3) == pytest.approx(
        AD + 0.3 / ACCEL_SLOPE
    )
    # 100% で頭打ち
    assert abs(_coast(t, is_accel=True, dev=5000.0, tau=0.5)) <= 100.0


def test_coast_assist_effort_onset_floor() -> None:
    """段2r ②: onset_u_pct を渡すと u の大きさが効き始め以上になる。None は今まで通り。"""
    t = linear_table()
    shallow = 0.4 / BRAKE_SLOPE  # 効き始めより浅い u
    on = shallow + 1.0
    base = _coast(t, is_accel=False, dev=-2.0)
    assert base == pytest.approx(-(BD + shallow))
    assert _coast(t, is_accel=False, dev=-2.0, onset_u_pct=None) == base
    assert _coast(t, is_accel=False, dev=-2.0, onset_u_pct=on) == pytest.approx(-(BD + on))
    # 効き始めより深ければそのまま
    assert _coast(t, is_accel=False, dev=-2.0, onset_u_pct=0.1) == base
    # アクセル側も同じ
    assert _coast(t, is_accel=True, dev=2.0, onset_u_pct=5.0) == pytest.approx(AD + 5.0)
    # 逆の符号で min に止めた場合も、効き始めに置く
    assert _coast(t, is_accel=True, dev=-20.0, onset_u_pct=3.0) == pytest.approx(AD + 3.0)
    assert _coast(t, is_accel=False, dev=50.0, onset_u_pct=3.0) == pytest.approx(-(BD + 3.0))


def test_coast_assist_effort_uses_same_effort_u_relation_as_build_plan() -> None:
    """effort ＝ ±(min ＋ |u|)。build_plan が u を逆に引いて出す effort と同じ関係で、
    同じ要る加速度なら coast_assist_effort（dev=0）は build_plan の effort と一致する。"""
    t = linear_table()
    # 要る加速度が惰行より +1.0 km/h/s（アクセル）／−1.0（ブレーキ）の行を、dev=0 で引き直す
    coast_a = float(t.coast_accel(50.0))
    ua, _ = t.invert(50.0, coast_a + 1.0)
    ub, _ = t.invert(50.0, coast_a - 1.0)
    assert _coast(t, is_accel=True, dev=0.0, need=coast_a + 1.0) == pytest.approx(AD + float(ua))
    assert _coast(t, is_accel=False, dev=0.0, need=coast_a - 1.0) == pytest.approx(
        -(BD + abs(float(ub)))
    )


# ───────── 2026-10-20 段4-3: 停止側の戻し・アクセルの戻しを使った解き直し ─────────


def _stop_release_variant(strength: float = 0.05) -> ForwardTable:
    """停止側の表（linear_table と同じ）に、u < 0 の列へ戻し量 × strength × |u| を足す層を付ける."""
    from tests.research.fwd_map import RELEASE_GRID_PCT  # noqa: PLC0415

    t = linear_table()
    t.stop_accel = t.accel_grid.copy()
    t.meta["stop_max_speed_kmh"] = 25.0
    t.meta["stop_release_max_pct"] = 8.0
    g = t.u_grid
    t.stop_release_grid = np.array(RELEASE_GRID_PCT, dtype=float)
    t.stop_release_accel = np.stack(
        [t.stop_accel + np.where(g < 0, strength * r * -g, 0.0)[None, :] for r in RELEASE_GRID_PCT]
    )
    return t


def _stop_release_ref(t: float) -> float:
    """20 km/h から −5 km/h/s で 2 秒（強いブレーキ）→ −4 km/h/s（浅くして戻す）。"""
    return 20.0 - 5.0 * t if t < 2.0 else max(10.0 - 4.0 * (t - 2.0), 0.0)


def test_stop_release_none_gives_identical_plan() -> None:
    base_t = _stop_release_variant()
    base_t.stop_release_grid = None
    base_t.stop_release_accel = None
    only_stop = _stop_release_variant()
    only_stop.stop_release_accel = None
    only_stop.stop_release_grid = None
    a = plan_for(_stop_release_ref, 12.0, base_t, stop_horizon_s=10.0)
    b = plan_for(_stop_release_ref, 12.0, only_stop, stop_horizon_s=10.0)
    assert np.array_equal(a.effort_pct, b.effort_pct) and np.array_equal(a.flag, b.flag)
    # 頭打ちが 0 なら戻し量は常に 0 ＝ 解き直しても結果は変わらない
    zero = _stop_release_variant()
    zero.meta["stop_release_max_pct"] = 0.0
    z = plan_for(_stop_release_ref, 12.0, zero, stop_horizon_s=10.0)
    assert np.allclose(z.effort_pct, a.effort_pct, atol=1e-9)


def test_stop_side_times_are_resolved_with_the_stop_release_layers() -> None:
    plain = _stop_release_variant()
    plain.stop_release_grid = None
    plain.stop_release_accel = None
    tb = _stop_release_variant()
    base = plan_for(_stop_release_ref, 12.0, plain, stop_horizon_s=10.0)
    plan = plan_for(_stop_release_ref, 12.0, tb, stop_horizon_s=10.0)
    assert plan.stops_ahead is not None and plan.stops_ahead[idx(plan, 3.0)]
    # 強いブレーキの間（戻す前）は同じ
    for t in (0.5, 1.0, 1.4):
        assert plan.effort_pct[idx(plan, t)] == pytest.approx(base.effort_pct[idx(base, t)])
    # 戻している間は、戻しで効きが弱まる表なので踏みが深い（effort がより負）
    u_all = np.where(plan.effort_pct < -BD, plan.effort_pct + BD, 0.0)
    w = int(round(3.0 / plan.period_s))
    checked = 0
    for t in (2.5, 3.0, 3.5):
        i = idx(plan, t)
        assert plan.effort_pct[i] < base.effort_pct[i] - 0.3, t
        m = float(u_all[max(i - w, 0) : i].min())
        u = float(u_all[i])
        assert u > m
        got = tb.accel(_stop_release_ref(t), u, stop=True, release=u - m)
        assert got == pytest.approx(plan.need_accel_kmhs[i], abs=0.02)
        checked += 1
    assert checked == 3
    # 停止側の判定でない時刻（停車の先が遠い・範囲外の車速）は影響されない
    far = plan_for(lambda t: 40.0 - 4.0 * t if t < 3.0 else 28.0 - (t - 3.0), 5.0, tb)
    far0 = plan_for(lambda t: 40.0 - 4.0 * t if t < 3.0 else 28.0 - (t - 3.0), 5.0, plain)
    assert np.array_equal(far.effort_pct, far0.effort_pct)


def _accel_release_variant(strength: float = 0.04, with_layers: bool = True) -> ForwardTable:
    """linear_table に、u > 0 の列から strength × 戻し量 × u を引くアクセルの戻しの層を付ける。"""
    from tests.research.fwd_map import RELEASE_GRID_PCT, attach_accel_release  # noqa: PLC0415

    t = linear_table()
    if not with_layers:
        return t
    g = t.u_grid
    layers = np.stack(
        [t.accel_grid - np.where(g > 0, strength * r * g, 0.0)[None, :] for r in RELEASE_GRID_PCT]
    )
    t = attach_accel_release(
        t, layers, release_pct=np.full(1000, 1.0), dt_row_s=np.full(1000, 0.05)
    )
    t.meta["accel_release_max_pct"] = 8.0
    return t


def _accel_release_ref(t: float) -> float:
    """30 km/h から +8 km/h/s で 2 秒（強い加速）→ +2 km/h/s（弱くして戻す）。"""
    return 30.0 + 8.0 * t if t < 2.0 else 46.0 + 2.0 * (t - 2.0)


def test_accel_release_none_gives_identical_plan() -> None:
    plain = _accel_release_variant(with_layers=False)
    base = plan_for(_accel_release_ref, 8.0, plain)
    tb = _accel_release_variant()
    tb.accel_release_grid = None
    tb.accel_release_accel = None
    same = plan_for(_accel_release_ref, 8.0, tb)
    assert np.array_equal(same.effort_pct, base.effort_pct)
    assert np.array_equal(same.flag, base.flag)
    zero = _accel_release_variant()
    zero.meta["accel_release_max_pct"] = 0.0  # 戻し量は常に 0 ＝ 解き直しても変わらない
    res = plan_for(_accel_release_ref, 8.0, zero)
    assert np.allclose(res.effort_pct, base.effort_pct, atol=1e-9)


def test_accel_times_are_resolved_in_time_order_with_the_planned_release() -> None:
    base = plan_for(_accel_release_ref, 8.0, _accel_release_variant(with_layers=False))
    tb = _accel_release_variant()
    plan = plan_for(_accel_release_ref, 8.0, tb)
    # 山（強い加速）の間は同じ
    for t in (0.5, 1.0, 1.4):
        assert plan.effort_pct[idx(plan, t)] == pytest.approx(base.effort_pct[idx(base, t)])
    # 戻している間は、戻しで効きが弱まる表なので踏みが深い（effort がより正）
    u_all = np.where(plan.effort_pct > AD, plan.effort_pct - AD, 0.0)
    w = int(round(3.0 / plan.period_s))
    for t in (3.0, 3.5, 4.0):
        i = idx(plan, t)
        assert plan.effort_pct[i] > base.effort_pct[i] + 0.1, t
        m = float(u_all[max(i - w, 0) : i].max())  # 計画済みの列（時刻順に解き直した後）の山
        u = float(u_all[i])
        assert u < m  # 戻している
        got = tb.accel(_accel_release_ref(t), u, accel_release=m - u)
        assert got == pytest.approx(plan.need_accel_kmhs[i], abs=0.02)
    # 戻しに入らない時刻（ブレーキ側・ずっと同じ傾きの加速）は影響されない
    dn = plan_for(lambda t: 80.0 - 5.0 * t, 4.0, tb)
    dn0 = plan_for(lambda t: 80.0 - 5.0 * t, 4.0, _accel_release_variant(with_layers=False))
    assert np.array_equal(dn.effort_pct, dn0.effort_pct)
    up = plan_for(lambda t: 30.0 + 2.0 * t, 6.0, tb)
    up0 = plan_for(lambda t: 30.0 + 2.0 * t, 6.0, _accel_release_variant(with_layers=False))
    assert np.array_equal(up.effort_pct, up0.effort_pct)


def _legacy_launch_release_table(with_layers: bool = True) -> ForwardTable:
    """アクセルの戻しの層つき（または無し）・発進側の表は本体と同じ値・低速の上の車速 7.5 km/h。"""
    t = _accel_release_variant(with_layers=with_layers)
    t.launch_accel = t.accel_grid.copy()
    t.meta["stop_max_speed_kmh"] = 7.5
    return t


def test_legacy_accel_release_unchanged_without_release_layers() -> None:
    # 戻しの表なし: 計画もラベルも、発進側の表なしの計画と変わらない（本体と同じ値の発進側の表）
    with_l = plan_for(_accel_release_ref, 8.0, _legacy_launch_release_table(with_layers=False))
    plain = plan_for(_accel_release_ref, 8.0, _accel_release_variant(with_layers=False))
    assert np.array_equal(with_l.effort_pct, plain.effort_pct)
    assert np.array_equal(with_l.flag, plain.flag)
    again = plan_for(_accel_release_ref, 8.0, _legacy_launch_release_table(with_layers=False))
    assert np.array_equal(with_l.side_accel, again.side_accel)
    # need > 0 の行は従来どおり SIDE_LAUNCH のまま
    assert (with_l.side_accel[with_l.need_accel_kmhs > 0.0] == SIDE_LAUNCH).all()


def test_legacy_accel_release_resolves_above_top_but_not_below() -> None:
    tb = _legacy_launch_release_table()
    plan = plan_for(_accel_release_ref, 8.0, tb)
    # 発進側の表を持たない同じ戻しの層の計画（7.5 km/h 以上は解き直される）が、7.5 km/h 以上の正解
    ref_tb = _accel_release_variant()
    ref_plan = plan_for(_accel_release_ref, 8.0, ref_tb)
    for t in (3.0, 3.5, 4.0):
        i = idx(plan, t)
        assert plan.effort_pct[i] == pytest.approx(ref_plan.effort_pct[i])
        zero = plan_for(_accel_release_ref, 8.0, _accel_release_variant(with_layers=False))
        assert plan.effort_pct[i] > zero.effort_pct[i] + 0.1  # 戻し量 0 の解と変わる
        assert plan.side_accel[i] == SIDE_MAIN
    # 7.5 km/h 未満で発進側の行は解き直さない（戻し量 0 の解のまま・SIDE_LAUNCH）
    def slow(t: float) -> float:
        return 1.0 + 1.0 * t if t < 3.0 else 4.0 + 0.2 * (t - 3.0)

    low = plan_for(slow, 6.0, tb)
    low0 = plan_for(slow, 6.0, _legacy_launch_release_table(with_layers=False))
    assert np.array_equal(low.effort_pct, low0.effort_pct)
    assert (low.side_accel[low.need_accel_kmhs > 0.0] == SIDE_LAUNCH).all()


# ───────── 2026-10-22 段5-2: 低速の表の引き方（low_speed_side_rule） ─────────


def _pedal_table(dl: float = 2.0, ds: float = 2.4, breakaway: float | None = None) -> ForwardTable:
    """発進側・停止側を持つ表。発進側は本体より dl、停止側は ds だけ全体に持ち上げる
    （惰行の加速度が本体 −2.0 に対し、発進側 −2+dl・停止側 −2+ds）。低速は 7.5 km/h 未満。"""
    t = linear_table()
    t.launch_accel = t.accel_grid + dl
    t.stop_accel = t.accel_grid + ds
    t.meta["stop_max_speed_kmh"] = 7.5
    if breakaway is not None:
        t.meta["launch_breakaway_u_pct"] = breakaway
    return t


def _launch_then_constant(t: float) -> float:
    """停車保持 → 2〜3 s で 1.6 km/h まで発進 → 8 s まで一定（need = 0）。"""
    return 0.0 if t < 2.0 else min(1.6 * (t - 2.0), 1.6)


def test_side_rule_legacy_is_the_default_and_matches_explicit_legacy() -> None:
    for tb in (_pedal_table(), _launch_variant(), _stop_variant()):
        base = plan_for(_launch_then_constant, 8.0, tb)
        same = plan_for(_launch_then_constant, 8.0, tb, low_speed_side_rule="legacy")
        assert np.array_equal(base.effort_pct, same.effort_pct)
        assert np.array_equal(base.flag, same.flag)
        assert base.side_rule == "legacy" and base.side_for(10, False) is None
        assert base.side_accel is not None and base.side_brake is not None


def test_side_arrays_in_legacy_come_from_low_speed_sides() -> None:
    tb = _pedal_table()
    plan = plan_for(_decel_to_stop, 6.0, tb)
    near, far = idx(plan, 4.5), idx(plan, 2.0)
    assert plan.side_brake[near] == SIDE_STOP and plan.side_brake[far] == SIDE_MAIN
    rise = plan_for(lambda t: min(2.0 * t, 10.0), 6.0, tb)
    assert rise.side_accel[idx(rise, 1.0)] == SIDE_LAUNCH  # need > 0 → 発進側


def test_pedal_rule_without_launch_or_stop_table_is_legacy() -> None:
    for tb in (_launch_variant(), _stop_variant()):  # 片方しか持たない
        base = plan_for(_launch_then_constant, 8.0, tb)
        same = plan_for(_launch_then_constant, 8.0, tb, low_speed_side_rule="pedal")
        assert np.array_equal(base.effort_pct, same.effort_pct)
        assert same.side_rule == "legacy"


def test_pedal_rule_constant_after_launch_uses_the_stop_table() -> None:
    tb = _pedal_table()
    legacy = plan_for(_launch_then_constant, 8.0, tb)
    plan = plan_for(_launch_then_constant, 8.0, tb, low_speed_side_rule="pedal")
    assert plan.side_rule == "pedal"
    for t in (4.0, 6.0, 7.5):
        i = idx(plan, t)
        assert plan.need_accel_kmhs[i] == pytest.approx(0.0, abs=1e-9)
        assert plan.side_brake[i] == SIDE_STOP
        assert plan.side_accel[i] == SIDE_LAUNCH
        assert plan.effort_pct[i] < 0.0  # 停止側の惰行 +0.38 に対し need 0 → ブレーキ
        # legacy は need = 0 だと本体（惰行 −2.08）になり、アクセルを踏む
        assert legacy.side_brake[i] == SIDE_MAIN and legacy.effort_pct[i] > 0.0
    # 発進のアクセル（need > 0）は発進側
    assert plan.side_accel[idx(plan, 2.5)] == SIDE_LAUNCH
    assert plan.effort_pct[idx(plan, 2.5)] > 0.0


def _release_low_table() -> tuple[ForwardTable, ForwardTable]:
    import copy  # noqa: PLC0415

    from tests.research.test_research_fwd_map import release_table  # noqa: PLC0415

    tb = release_table()
    tb.meta["release_max_pct"] = 8.0
    tb.launch_accel = tb.accel_grid.copy()
    tb.stop_accel = tb.accel_grid.copy()
    tb.meta["stop_max_speed_kmh"] = 7.5
    plain = copy.deepcopy(tb)
    plain.release_grid = None
    plain.release_accel = None
    return tb, plain


def _decel_then_ease(t: float) -> float:
    """14 から −4 km/h/s で 1.5 s（強いブレーキ、8 km/h）→ −2.5 km/h/s（浅くして戻す）。"""
    return 14.0 - 4.0 * t if t < 1.5 else max(0.0, 8.0 - 2.5 * (t - 1.5))


def test_pedal_rule_low_speed_decel_uses_stop_table_not_release_table() -> None:
    tb, plain = _release_low_table()
    plan = plan_for(_decel_then_ease, 5.0, tb, low_speed_side_rule="pedal")
    base = plan_for(_decel_then_ease, 5.0, plain, low_speed_side_rule="pedal")
    legacy = plan_for(_decel_then_ease, 5.0, tb)
    v = np.array([_decel_then_ease(float(x)) for x in plan.t_s])
    low = (v < 7.5) & (plan.flag != FLAG_STOP) & (plan.effort_pct < 0.0)
    rows = np.flatnonzero(low & (plan.t_s > 2.0))
    assert len(rows) > 10
    # 停車の 0.5 s より前でも停止側。戻しの表は引かない（戻しの層を持たない表と同じ開度）
    assert np.all(plan.side_brake[rows] == SIDE_STOP)
    assert np.allclose(plan.effort_pct[rows], base.effort_pct[rows])
    # legacy はこの行を本体＋戻しの表で解き、戻しの分だけ深くなる行がある
    assert np.any(legacy.side_brake[rows] == SIDE_RELEASE)
    assert np.any(legacy.effort_pct[rows] < plan.effort_pct[rows] - 0.1)
    # 7.5 km/h 以上の戻しは今まで通り（戻しの表）
    hi = np.flatnonzero((v >= 7.5) & (plan.flag != FLAG_STOP) & (plan.t_s > 1.5))
    assert np.any(plan.side_brake[hi] == SIDE_RELEASE)
    assert np.allclose(plan.effort_pct[hi], legacy.effort_pct[hi])


def _slow_launch(t: float) -> float:
    """停車保持 → 2〜6 s で 0.5 km/h/s で 2.0 まで → 6〜7 s で −1.0 km/h/s → 7.5 s まで 1.0 一定
    → 7.5〜9.5 s で 10 km/h まで加速。"""
    if t < 2.0:
        return 0.0
    if t < 6.0:
        return 0.5 * (t - 2.0)
    if t < 7.0:
        return 2.0 - (t - 6.0)
    if t < 7.5:
        return 1.0
    return min(1.0 + 4.5 * (t - 7.5), 10.0)


def test_pedal_rule_brake_during_launch_is_launch_side_unless_deeper_than_breakaway() -> None:
    bk = 3.0
    tb = _pedal_table(dl=3.0, ds=2.4, breakaway=bk)
    plan = plan_for(_slow_launch, 10.0, tb, low_speed_side_rule="pedal")
    v = np.array([_slow_launch(float(x)) for x in plan.t_s])
    stop = plan.flag == FLAG_STOP
    first_accel = int(np.flatnonzero((plan.effort_pct > 0.0) & ~stop)[0])
    low = v < 7.5
    # 停車保持の直後（ref ≤ 0.5）と動き出して浅い間: 発進側
    for t in (1.9, 3.0, 4.0):
        i = idx(plan, t)
        assert not stop[i] and i < first_accel
        assert plan.effort_pct[i] < 0.0, t
        assert plan.side_brake[i] == SIDE_LAUNCH, t
    # 発進の途中で、発進側で解いた深さが動き出す開度より深い行は停止側。浅い行は発進側
    mid = np.arange(len(v)) < first_accel
    mid &= ~stop & low
    n_stop = n_launch = 0
    for i in np.flatnonzero(mid):
        u_l = abs(float(tb.invert(v[i], plan.need_accel_kmhs[i], launch=True)[0]))
        if v[i] > 0.5 and u_l > bk:
            assert plan.side_brake[i] == SIDE_STOP, i
            n_stop += 1
        else:
            assert plan.side_brake[i] == SIDE_LAUNCH, i
            n_launch += 1
    assert n_stop > 0 and n_launch > 0
    # アクセルを使い始めた後の低速のブレーキ側は停止側
    after = np.flatnonzero(low & ~stop & (np.arange(len(v)) > first_accel))
    assert len(after) > 0 and np.all(plan.side_brake[after] == SIDE_STOP)
    # 実機の pkl の meta は符号つき（ブレーキ側 ＝ 負）。大きさで比べるので同じ結果
    neg = _pedal_table(dl=3.0, ds=2.4, breakaway=-bk)
    plan_neg = plan_for(_slow_launch, 10.0, neg, low_speed_side_rule="pedal")
    assert np.array_equal(plan_neg.side_brake, plan.side_brake)
    # 動き出す開度が meta に無ければこの判定はしない（深い行も発進側のまま）
    tb2 = _pedal_table(dl=3.0, ds=2.4, breakaway=None)
    plan2 = plan_for(_slow_launch, 10.0, tb2, low_speed_side_rule="pedal")
    assert np.all(plan2.side_brake[mid] == SIDE_LAUNCH)


def test_launch_midway_covers_every_launch() -> None:
    """発進の途中は、停車保持を抜けるたびに始まる（モードの最初の発進だけではない）。"""
    from tests.research.fwd_plan import _launch_midway

    stop = np.array([1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0], dtype=bool)
    accel = np.array([0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0], dtype=bool)
    got = _launch_midway(stop, accel)
    assert got.tolist() == [0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0]
    # 最初が停車保持でないなら、最初の区間は発進の途中ではない。アクセルが出なければ次の停車保持まで
    stop2 = np.array([0, 0, 1, 0, 0, 1, 0], dtype=bool)
    got2 = _launch_midway(stop2, np.zeros(7, dtype=bool))
    assert got2.tolist() == [0, 0, 0, 1, 1, 0, 1]


def test_pedal_rule_body_and_release_unchanged_at_or_above_top() -> None:
    tb, plain = _release_low_table()
    plan = plan_for(_decel_then_ease, 5.0, tb, low_speed_side_rule="pedal")
    legacy = plan_for(_decel_then_ease, 5.0, tb)
    v = np.array([_decel_then_ease(float(x)) for x in plan.t_s])
    hi = v >= 7.5
    assert np.array_equal(plan.effort_pct[hi], legacy.effort_pct[hi])
    assert np.array_equal(plan.side_accel[hi], legacy.side_accel[hi])


def test_side_argument_selects_the_table_in_deviation_and_coast_assist() -> None:
    t = linear_table()
    t.stop_accel = t.accel_grid.copy()
    cols = t.u_grid < 0
    base_col = (-2.0 - 0.01 * t.speed_rep)[:, None]
    t.stop_accel[:, cols] = (t.accel_grid[:, cols] - base_col) * 2.0 + base_col  # 傾き 2 倍
    t.launch_accel = t.accel_grid.copy()
    t.launch_accel[:, cols] = (t.accel_grid[:, cols] - base_col) * 0.5 + base_col  # 傾き 半分
    t.meta["stop_max_speed_kmh"] = 25.0
    kw = dict(effort=-15.0, need=-3.0, ref=10.0, dev=1.0)
    plain = _dev(linear_table(), **kw)
    stop = _dev(t, stops_ahead=True, **kw)  # low_speed_sides で停止側
    assert _dev(t, **kw) == pytest.approx(plain)  # stops_ahead なしは本体（side=None は今まで通り）
    # side を渡すとその表（stops_ahead・need の符号は見ない）
    assert _dev(t, side=SIDE_STOP, **kw) == pytest.approx(stop)
    assert _dev(t, side=SIDE_STOP, stops_ahead=False, **kw) == pytest.approx(stop)
    assert _dev(t, side=SIDE_MAIN, stops_ahead=True, **kw) == pytest.approx(plain)
    assert _dev(t, side=SIDE_RELEASE, stops_ahead=True, **kw) == pytest.approx(plain)
    launch = _dev(t, side=SIDE_LAUNCH, **kw)
    assert launch - (-15.0) == pytest.approx(2.0 * (plain - (-15.0)))  # 傾き半分 → 上乗せ 2 倍
    # 惰行の補助も同じ
    ck = dict(is_accel=False, dev=1.0, need=-3.0, ref=10.0)
    assert _coast(t, side=SIDE_STOP, **ck) == pytest.approx(_coast(t, stops_ahead=True, **ck))
    assert _coast(t, side=SIDE_MAIN, stops_ahead=True, **ck) == pytest.approx(_coast(t, **ck))
    assert _coast(t, side=SIDE_LAUNCH, **ck) != pytest.approx(_coast(t, **ck))


def test_plan_side_for_returns_side_only_for_pedal_rule() -> None:
    tb = _pedal_table()
    plan = plan_for(_launch_then_constant, 8.0, tb, low_speed_side_rule="pedal")
    i = idx(plan, 6.0)
    assert plan.side_for(i, False) == SIDE_STOP and plan.side_for(i, True) == SIDE_LAUNCH


def test_release_table_min_kmh_skips_release_below_the_speed() -> None:
    tb, plain = _release_low_table()
    default = plan_for(_decel_then_ease, 5.0, tb)
    zero = plan_for(_decel_then_ease, 5.0, tb, release_table_min_kmh=0.0)
    assert np.array_equal(default.effort_pct, zero.effort_pct)  # 既定は今まで
    assert np.any(default.side_brake == SIDE_RELEASE)
    # 100 km/h 未満は戻しの表で解かない ＝ 戻しの表なしの表と同じ開度
    off = plan_for(_decel_then_ease, 5.0, tb, release_table_min_kmh=100.0)
    base = plan_for(_decel_then_ease, 5.0, plain)
    assert not np.any(off.side_brake == SIDE_RELEASE)
    assert np.allclose(off.effort_pct, base.effort_pct)
    # しきい値が 10 km/h のとき、10 km/h 未満の基準の行は戻しにならず、以上の行は戻しのまま
    mid = plan_for(_decel_then_ease, 5.0, tb, release_table_min_kmh=10.0)
    v = np.array([_decel_then_ease(float(x)) for x in mid.t_s])
    assert not np.any(mid.side_brake[v < 10.0] == SIDE_RELEASE)
    assert np.any(mid.side_brake[v >= 10.0] == SIDE_RELEASE)


# ───────── 2026-10-22 段5-5c: 切り替えの先読み（switch_lead） ─────────


def _lead_table() -> ForwardTable:
    tb = linear_table()
    tb.meta.update(
        accel_return_delay_s=0.2, accel_return_rate_pct_s=20.0,
        brake_return_delay_s=0.3, brake_return_rate_pct_s=20.0,
        brake_onset_u_pct=2.0,
    )
    return tb


def _vee(t: float) -> float:
    """ブレーキ（−4 km/h/s）→ 4 s で底 → アクセル（+4 km/h/s）。"""
    return 50.0 + 4.0 * abs(t - 4.0)


def _lead_kw() -> dict:
    return dict(accel_lag_s=0.2, brake_lag_s=0.3, need_window_s=0.5)


def _first_accel_row(plan: ForwardPlan) -> int:
    return int(np.flatnonzero(plan.effort_pct > 0.0)[0])


def test_switch_lead_default_false_is_identical() -> None:
    tb = _lead_table()
    base = plan_for(_vee, 8.0, tb, **_lead_kw())
    same = plan_for(_vee, 8.0, tb, switch_lead=False, **_lead_kw())
    assert base.switch_lead_s is None and same.switch_lead_s is None
    assert np.array_equal(base.effort_pct, same.effort_pct)
    assert np.array_equal(base.flag, same.flag)
    assert np.array_equal(base.need_accel_kmhs, same.need_accel_kmhs)


def test_switch_lead_brake_to_accel_switch_comes_earlier_by_about_lead() -> None:
    tb = _lead_table()
    base = plan_for(_vee, 8.0, tb, **_lead_kw())
    lead = plan_for(_vee, 8.0, tb, switch_lead=True, **_lead_kw())
    assert base.pedal_switches() == lead.pedal_switches() == 1
    dt = (_first_accel_row(base) - _first_accel_row(lead)) * base.period_s
    # lead = ブレーキの戻る時間 0.3 + 深さ/20 + アクセルの遅れ 0.2（0.5〜1.0 s 程度）
    assert lead.switch_lead_s is not None and lead.switch_lead_s.max() > 0.0
    expected = float(lead.switch_lead_s.max())
    assert 0.5 <= expected <= 1.2
    assert dt == pytest.approx(expected, abs=0.25)
    # 先読みの行は切り替えの前だけ
    assert lead.switch_lead_s[: idx(lead, 1.0)].max() == 0.0
    assert lead.switch_lead_s[idx(lead, 6.5):].max() == 0.0


def test_switch_lead_events_pick_up_switch_and_deep_release() -> None:
    from tests.research.fwd_plan import switch_lead_events  # noqa: PLC0415

    tb = _lead_table()
    plan = plan_for(_vee, 8.0, tb, **_lead_kw())
    ev = switch_lead_events(plan, tb, la=0.2, lb=0.3, accel_min_pct=AD, brake_min_pct=BD)
    kinds = [e[2] for e in ev]
    assert kinds.count("switch") == 1
    k, lead, _ = next(e for e in ev if e[2] == "switch")
    assert k == _first_accel_row(plan)
    depth = -float(plan.effort_pct[k - 1]) - BD if plan.effort_pct[k - 1] < 0 else None
    if depth is not None:
        assert lead == pytest.approx(0.3 + depth / 20.0 + 0.2)


def test_switch_lead_deep_brake_release_comes_earlier() -> None:
    tb = _lead_table()

    tb.meta["brake_onset_u_pct"] = 0.5  # 戻す直前の行の深さ（min からの量 0.71）より浅い効き始め

    def decel_then_coast(t: float) -> float:
        return 100.0 - 6.0 * t if t < 3.0 else 82.0 - 2.8 * (t - 3.0)

    base = plan_for(decel_then_coast, 8.0, tb, **_lead_kw())
    lead = plan_for(decel_then_coast, 8.0, tb, switch_lead=True, **_lead_kw())
    from tests.research.fwd_plan import switch_lead_events  # noqa: PLC0415

    ev = switch_lead_events(base, tb, la=0.2, lb=0.3, accel_min_pct=AD, brake_min_pct=BD)
    assert [e[2] for e in ev] == ["release"]

    def last_brake(p: ForwardPlan) -> int:
        return int(np.flatnonzero(p.effort_pct < 0.0)[-1])

    assert last_brake(lead) < last_brake(base)  # ブレーキの戻しが早まる
    assert lead.switch_lead_s is not None and lead.switch_lead_s.max() == pytest.approx(
        ev[0][1]
    )


def test_switch_lead_ignores_stop_hold_entry_and_exit() -> None:
    tb = _lead_table()

    def stop_and_go(t: float) -> float:  # 減速 → 停車 → 発進
        if t < 3.0:
            return max(20.0 - 7.0 * t, 0.0)
        return 0.0 if t < 6.0 else min(5.0 * (t - 6.0), 20.0)

    base = plan_for(stop_and_go, 12.0, tb, **_lead_kw())
    lead = plan_for(stop_and_go, 12.0, tb, switch_lead=True, **_lead_kw())
    stop = base.flag == FLAG_STOP
    assert stop.any()
    assert np.array_equal(base.flag == FLAG_STOP, lead.flag == FLAG_STOP)
    assert np.array_equal(base.effort_pct[stop], lead.effort_pct[stop])
    assert lead.switch_lead_s is not None
    assert lead.switch_lead_s[stop].max() == 0.0
    # 停車保持をまたぐブレーキ → アクセルは出来事に数えない
    from tests.research.fwd_plan import switch_lead_events  # noqa: PLC0415

    ev = switch_lead_events(base, tb, la=0.2, lb=0.3, accel_min_pct=AD, brake_min_pct=BD)
    first, last = int(np.flatnonzero(stop)[0]), int(np.flatnonzero(stop)[-1])
    assert all(k < first or k > last + 40 for k, _, _ in ev)  # 停車保持の中・抜けた直後は無し


# ───────── 2026-10-23 段5-6: low_speed_side_rule="dir" ─────────


def test_dir_rule_without_launch_or_stop_table_is_legacy() -> None:
    for tb in (_launch_variant(), _stop_variant()):
        base = plan_for(_launch_then_constant, 8.0, tb)
        same = plan_for(_launch_then_constant, 8.0, tb, low_speed_side_rule="dir")
        assert np.array_equal(base.effort_pct, same.effort_pct)
        assert same.side_rule == "legacy"


def test_dir_rule_leaves_legacy_and_pedal_untouched_and_reports_side_for() -> None:
    tb = _pedal_table()
    legacy = plan_for(_decel_to_stop, 6.0, tb)
    again = plan_for(_decel_to_stop, 6.0, tb, low_speed_side_rule="legacy")
    assert np.array_equal(legacy.effort_pct, again.effort_pct)
    plan = plan_for(_launch_then_constant, 8.0, tb, low_speed_side_rule="dir")
    assert plan.side_rule == "dir"
    i = idx(plan, 2.5)
    assert plan.side_for(i, True) == SIDE_LAUNCH  # 低速のアクセルは発進側
    assert plan.side_for(idx(plan, 6.0), False) == SIDE_STOP  # 発進の途中を抜けた後
    assert legacy.side_for(i, True) is None


def test_dir_rule_coast_decision_uses_stop_side_coast_accel() -> None:
    tb = _pedal_table()
    plan = plan_for(_launch_then_constant, 8.0, tb, low_speed_side_rule="dir")
    legacy = plan_for(_launch_then_constant, 8.0, tb)
    for t in (4.0, 6.0, 7.5):
        i = idx(plan, t)
        # 停止側の惰行は +0.38、要る加速度 0 → 0 − 0.38 ≤ −帯 でブレーキ
        assert plan.effort_pct[i] < 0.0
        # 本体の惰行（−2.08）を使う legacy はアクセル
        assert legacy.effort_pct[i] > 0.0
    # 7.5 km/h 以上の惰行判定は本体のまま
    hi = lambda t: 30.0 + 0.0 * t  # noqa: E731
    a = plan_for(hi, 4.0, tb, low_speed_side_rule="dir")
    b = plan_for(hi, 4.0, tb)
    assert np.array_equal(a.effort_pct, b.effort_pct)


def test_dir_rule_low_decel_then_ease_uses_stop_then_release_when_shallower() -> None:
    tb, _ = _release_low_table()
    plan = plan_for(_decel_then_ease, 5.0, tb, low_speed_side_rule="dir")
    legacy = plan_for(_decel_then_ease, 5.0, tb)
    v = np.array([_decel_then_ease(float(x)) for x in plan.t_s])
    brake = (plan.flag != FLAG_STOP) & (plan.effort_pct < 0.0)
    low_rows = np.flatnonzero(brake & (v < 7.5) & (plan.t_s > 1.5))
    assert len(low_rows) > 10
    sides = set(plan.side_brake[low_rows].tolist())
    assert sides <= {SIDE_STOP, SIDE_RELEASE} and SIDE_RELEASE in sides
    # 7.5 km/h 以上は legacy と同じ
    hi = np.flatnonzero((v >= 7.5) & (plan.flag != FLAG_STOP))
    assert np.array_equal(plan.effort_pct[hi], legacy.effort_pct[hi])
    # 戻した行は、その行を本体の戻しの効きで解いた値（戻し量 m からの浅さ）になる
    u = np.where(plan.effort_pct < -BD, plan.effort_pct + BD, 0.0)
    w = int(round(3.0 / plan.period_s))
    for i in low_rows:
        if plan.side_brake[i] == SIDE_RELEASE:
            assert u[i] > float(u[max(i - w, 0) : i].min()) - 1e-9


def _release_low_table_with_stop_layer() -> ForwardTable:
    """_release_low_table に、停止側の戻しの層（戻し量 × 0.2 × |u| で効きが弱まる）を付けた表。"""
    from tests.research.fwd_map import RELEASE_GRID_PCT  # noqa: PLC0415

    tb, _ = _release_low_table()
    tb.meta["stop_release_max_pct"] = 8.0
    g = tb.u_grid
    tb.stop_release_grid = np.array(RELEASE_GRID_PCT, dtype=float)
    tb.stop_release_accel = np.stack(
        [tb.stop_accel + np.where(g < 0, 0.2 * r * -g, 0.0)[None, :] for r in RELEASE_GRID_PCT]
    )
    return tb


def test_low_speed_release_stop_layer_solves_low_rows_with_the_stop_layer() -> None:
    tb = _release_low_table_with_stop_layer()
    kw = dict(low_speed_side_rule="dir")
    base = plan_for(_decel_then_ease, 5.0, tb, **kw)
    off = plan_for(_decel_then_ease, 5.0, tb, low_speed_release_stop_layer=False, **kw)
    on = plan_for(_decel_then_ease, 5.0, tb, low_speed_release_stop_layer=True, **kw)
    # off は今までと完全に同じ
    assert np.array_equal(base.effort_pct, off.effort_pct)
    assert np.array_equal(base.side_brake, off.side_brake)
    assert not np.any(base.side_brake == SIDE_STOP_RELEASE)
    v = np.array([_decel_then_ease(float(x)) for x in on.t_s])
    low = v < 7.5
    # 低速の行で層で解いた行は SIDE_STOP_RELEASE になる（本体の戻し SIDE_RELEASE は低速から消える）
    layer_rows = np.flatnonzero(on.side_brake == SIDE_STOP_RELEASE)
    assert len(layer_rows) > 5 and np.all(low[layer_rows])
    assert not np.any(on.side_brake[low] == SIDE_RELEASE)
    # 層の行は、停止側＋戻し量で解いた値（accel(stop=True, release=m からの浅さ)が要る加速度）
    u = np.where(on.effort_pct < -BD, on.effort_pct + BD, 0.0)
    w = int(round(3.0 / on.period_s))
    for i in layer_rows[:10]:
        m = float(u[max(i - w, 0) : i].min())
        got = tb.accel(float(v[i]), float(u[i]), stop=True, release=float(u[i]) - m)
        assert got == pytest.approx(on.need_accel_kmhs[i], abs=0.05)
    # 7.5 km/h 以上は変わらない
    hi = np.flatnonzero(~low)
    assert np.array_equal(on.effort_pct[hi], off.effort_pct[hi])
    assert np.array_equal(on.side_brake[hi], off.side_brake[hi])
    # 層を持たない表では、キー on でも今と同じ
    tb2, _ = _release_low_table()
    a = plan_for(_decel_then_ease, 5.0, tb2, **kw)
    b = plan_for(_decel_then_ease, 5.0, tb2, low_speed_release_stop_layer=True, **kw)
    assert np.array_equal(a.effort_pct, b.effort_pct)
    assert np.array_equal(a.side_brake, b.side_brake)


def test_side_stop_release_picks_the_stop_table_in_deviation_and_coast_assist() -> None:
    t = linear_table()
    t.stop_accel = t.accel_grid.copy()
    cols = t.u_grid < 0
    base_col = (-2.0 - 0.01 * t.speed_rep)[:, None]
    t.stop_accel[:, cols] = (t.accel_grid[:, cols] - base_col) * 2.0 + base_col
    t.meta["stop_max_speed_kmh"] = 25.0
    kw = dict(effort=-15.0, need=-3.0, ref=10.0, dev=1.0)
    stop = _dev(t, side=SIDE_STOP, **kw)
    assert stop != pytest.approx(_dev(t, side=SIDE_RELEASE, **kw))
    assert _dev(t, side=SIDE_STOP_RELEASE, **kw) == pytest.approx(stop)
    ck = dict(is_accel=False, dev=1.0, need=-3.0, ref=10.0)
    assert _coast(t, side=SIDE_STOP_RELEASE, **ck) == pytest.approx(_coast(t, side=SIDE_STOP, **ck))


def test_gain_ratio_divides_the_offset_before_the_min_clamp() -> None:
    t = linear_table()
    plain = _dev(t, effort=12.0, dev=2.0)
    half = _dev(t, effort=12.0, dev=2.0, gain_ratio=2.0)
    assert half - 12.0 == pytest.approx((plain - 12.0) / 2.0)
    assert _dev(t, effort=12.0, dev=2.0, gain_ratio=1.0) == plain
    # 減らす向き: 割ってから min で止める（割る前なら止まる量でも、割ると止まらない）
    du = 2.0 / 5.0 / ACCEL_SLOPE  # 割る前の減らす量
    assert _dev(t, effort=AD + 1.0, dev=-2.0) == AD and du > 1.0
    assert _dev(t, effort=AD + 1.0, dev=-2.0, gain_ratio=4.0) == pytest.approx(AD + 1.0 - du / 4.0)
    _, clamped = deviation_offset_detail(
        t, effort=AD + 1.0, flag=FLAG_MEASURED, need_kmhs=0.0, ref_kmh=50.0, row_kmh=50.0,
        dev_kmh=-2.0, tau_accel_s=5.0, tau_brake_s=5.0, accel_min_pct=AD, brake_min_pct=BD,
        gain_ratio=4.0,
    )
    assert not clamped


def test_no_deeper_brake_in_midway_helper() -> None:
    from tests.research.fwd_plan import FLAG_COAST, FLAG_MEASURED, _no_deeper_brake_in_midway

    eff = np.array([0.0, -5.0, -2.0, 0.0, -8.0, -3.0, 4.0, -9.0, -1.0])
    flag = np.array([FLAG_STOP, 0, 0, FLAG_COAST, 0, 0, 0, 0, 0])
    side = np.array([0, 1, 1, 0, 3, 1, 0, 0, 0])
    mid = np.array([0, 1, 1, 1, 1, 1, 0, 1, 1], dtype=bool)
    _no_deeper_brake_in_midway(eff, flag, side, mid)
    assert eff.tolist() == [0.0, -8.0, -8.0, -8.0, -8.0, -3.0, 4.0, -9.0, -1.0]
    assert flag[3] == flag[4] == FLAG_MEASURED and side[1] == side[2] == side[3] == 3
    # 区間をまたいで伝わらない（4.0 の行は区間の外）
    assert eff[7] == -9.0


def test_dir_rule_launch_midway_never_deepens_the_brake() -> None:
    tb = _pedal_table(dl=3.0, ds=2.4, breakaway=3.0)
    plan = plan_for(_slow_launch, 10.0, tb, low_speed_side_rule="dir")
    stop = plan.flag == FLAG_STOP
    first_accel = int(np.flatnonzero((plan.effort_pct > 0.0) & ~stop)[0])
    first_move = int(np.flatnonzero(~stop)[0])
    seg = plan.effort_pct[first_move:first_accel]
    assert len(seg) > 10
    depth = np.maximum(-seg, 0.0)
    assert np.all(np.diff(depth) <= 1e-9)
    # pedal の同じ区間は途中で深くなる行がある（(ロ) の効き）


def _accel_push_variant(strength: float = 0.05, with_release: bool = False) -> ForwardTable:
    """linear_table に、u > 0 の列へ strength × 踏み増し量 × u を足すアクセルの踏み増しの層を付ける
    （with_release なら戻しの層も付ける）。"""
    from tests.research.fwd_map import RELEASE_GRID_PCT, attach_accel_push  # noqa: PLC0415

    t = _accel_release_variant() if with_release else linear_table()
    g = t.u_grid
    layers = np.stack(
        [t.accel_grid + np.where(g > 0, strength * p * g, 0.0)[None, :] for p in RELEASE_GRID_PCT]
    )
    t = attach_accel_push(t, layers, push_pct=np.full(1000, 1.0), dt_row_s=np.full(1000, 0.05))
    t.meta["accel_push_max_pct"] = 8.0
    return t


def _push_ref(t: float) -> float:
    """30 km/h から +2 km/h/s で 2 秒 → +8 km/h/s（踏み増す）。"""
    return 30.0 + 2.0 * t if t < 2.0 else 34.0 + 8.0 * (t - 2.0)


def test_accel_push_none_gives_identical_plan() -> None:
    base = plan_for(_push_ref, 6.0, linear_table())
    tb = _accel_push_variant()
    tb.accel_push_grid = None
    tb.accel_push_accel = None
    same = plan_for(_push_ref, 6.0, tb)
    assert np.array_equal(same.effort_pct, base.effort_pct)
    assert np.array_equal(same.flag, base.flag)
    # 戻しの表だけの表（踏み増しの軸なし）は、踏み増しの軸を外した表と同じ計画
    both = _accel_push_variant(with_release=True)
    both.accel_push_grid = None
    both.accel_push_accel = None
    assert np.array_equal(
        plan_for(_accel_release_ref, 8.0, both).effort_pct,
        plan_for(_accel_release_ref, 8.0, _accel_release_variant()).effort_pct,
    )


def test_accel_push_changes_plan_only_where_pushing() -> None:
    base = plan_for(_push_ref, 6.0, linear_table())
    tb = _accel_push_variant()
    plan = plan_for(_push_ref, 6.0, tb)
    # 踏み増さない（一定の速度を保つ）計画は同じ
    flat = plan_for(lambda t: 50.0, 6.0, tb)
    flat0 = plan_for(lambda t: 50.0, 6.0, linear_table())
    assert np.allclose(flat.effort_pct, flat0.effort_pct, atol=1e-9)
    u_all = np.where(plan.effort_pct > AD, plan.effort_pct - AD, 0.0)
    w = int(round(3.0 / plan.period_s))
    seen = 0
    for t in (2.5, 3.0, 3.5, 4.0):
        i = idx(plan, t)
        mn = float(np.maximum(u_all[max(i - w, 0) : i], 0.0).min())
        u = float(u_all[i])
        assert u > mn
        # 踏み増し量 0 の解と違い、踏み増しで効きが強まる表なので浅い（effort が小さい）
        assert plan.effort_pct[i] < base.effort_pct[i] - 0.05, t
        got = tb.accel(_push_ref(t), u, accel_push=u - mn)
        assert got == pytest.approx(plan.need_accel_kmhs[i], abs=0.02)
        seen += 1
    assert seen == 4


def test_accel_push_with_release_axis_solves_both_and_release_times_unchanged() -> None:
    tb = _accel_push_variant(with_release=True)
    plan = plan_for(_accel_release_ref, 8.0, tb)
    u_all = np.where(plan.effort_pct > AD, plan.effort_pct - AD, 0.0)
    w = int(round(3.0 / plan.period_s))
    for t in (3.0, 3.5, 4.0):
        i = idx(plan, t)
        win = u_all[max(i - w, 0) : i]
        mx, mn, u = float(win.max()), float(np.maximum(win, 0.0).min()), float(u_all[i])
        got = tb.accel(
            _accel_release_ref(t), u, accel_release=max(mx - u, 0.0), accel_push=max(u - mn, 0.0)
        )
        assert got == pytest.approx(plan.need_accel_kmhs[i], abs=0.02)
    # ブレーキ側の減速は軸に影響されない
    dn = plan_for(lambda t: 80.0 - 5.0 * t, 4.0, tb)
    dn0 = plan_for(lambda t: 80.0 - 5.0 * t, 4.0, _accel_release_variant(with_layers=False))
    assert np.array_equal(dn.effort_pct, dn0.effort_pct)


# ───────── 2026-10-23 段5-6 追加 A1・A2: 動き出す開度の頭打ち・床 ─────────


def test_cap_midway_brake_helper_only_midway_and_only_shallower() -> None:
    from tests.research.fwd_plan import FLAG_COAST, FLAG_MEASURED, _cap_midway_brake

    eff = np.array([-9.0, -9.0, -6.0, 0.0, 3.0, -9.0, -9.0])
    flag = np.array([FLAG_STOP, FLAG_MEASURED, FLAG_MEASURED, FLAG_COAST, FLAG_MEASURED,
                     FLAG_MEASURED, FLAG_MEASURED])
    mid = np.array([1, 1, 1, 1, 0, 0, 1], dtype=bool)
    _cap_midway_brake(eff, flag, mid, 4.2, {"launch_breakaway_u_pct": -4.3})
    cap = -(4.2 + 4.3)
    # 停車保持の行・発進の途中でない行・浅い行・アクセルは変えない。深い行は cap まで
    assert eff.tolist() == [-9.0, cap, -6.0, 0.0, 3.0, -9.0, cap]
    for meta in ({}, {"launch_breakaway_u_pct": float("nan")}, {"launch_breakaway_u_pct": None}):
        e2 = np.array([-9.0, -9.0])
        _cap_midway_brake(e2, np.array([FLAG_MEASURED] * 2), np.ones(2, dtype=bool), 4.2, meta)
        assert e2.tolist() == [-9.0, -9.0]


def test_launch_midway_brake_cap_in_plan() -> None:
    tb = _pedal_table(dl=3.0, ds=2.4, breakaway=-1.0)  # cap ＝ −(BD + 1.0) ＝ −8
    off = plan_for(_slow_launch, 10.0, tb, low_speed_side_rule="dir")
    on = plan_for(_slow_launch, 10.0, tb, low_speed_side_rule="dir", launch_midway_brake_cap=True)
    stop = off.flag == FLAG_STOP
    first_accel = int(np.flatnonzero((off.effort_pct > 0.0) & ~stop)[0])
    first_move = int(np.flatnonzero(~stop)[0])
    cap = -(BD + 1.0)
    assert off.effort_pct[first_move:first_accel].min() < cap - 1e-9  # 前提: 頭打ちが要る計画
    assert on.effort_pct[first_move:first_accel].min() >= cap - 1e-9
    # 発進の途中より後・手前は同じ。途中も浅くなる向きだけ
    assert np.array_equal(on.effort_pct[first_accel:], off.effort_pct[first_accel:])
    assert np.array_equal(on.effort_pct[:first_move], off.effort_pct[:first_move])
    assert np.all(on.effort_pct[first_move:first_accel] >= off.effort_pct[first_move:first_accel])
    # 既定（False）は今と同じ、meta が無い表では on でも同じ
    dflt = plan_for(
        _slow_launch, 10.0, tb, low_speed_side_rule="dir", launch_midway_brake_cap=False
    )
    assert np.array_equal(dflt.effort_pct, off.effort_pct)
    nob = _pedal_table(dl=3.0, ds=2.4)
    a = plan_for(_slow_launch, 10.0, nob, low_speed_side_rule="dir")
    b = plan_for(_slow_launch, 10.0, nob, low_speed_side_rule="dir", launch_midway_brake_cap=True)
    assert np.array_equal(a.effort_pct, b.effort_pct)


def test_launch_midway_flag_in_plan() -> None:
    """段6 C: dir の計画だけ行ごとの「発進の途中」を持つ。停車保持の行は対象外。"""
    tb = _pedal_table(dl=3.0, ds=2.4)
    plan = plan_for(_slow_launch, 10.0, tb, low_speed_side_rule="dir")
    assert plan.launch_midway is not None and plan.launch_midway.any()
    idx = np.flatnonzero(plan.launch_midway & (plan.flag != FLAG_STOP))
    assert len(idx) > 0 and all(plan.is_launch_midway(int(i)) for i in idx)
    stop_idx = np.flatnonzero(plan.flag == FLAG_STOP)
    assert not any(plan.is_launch_midway(int(i)) for i in stop_idx)
    legacy = plan_for(_slow_launch, 10.0, tb, low_speed_side_rule="legacy")
    assert legacy.launch_midway is None and not legacy.is_launch_midway(0)


def test_launch_breakaway_floor_in_table_lookup() -> None:
    base = _pedal_table(dl=12.0, ds=2.4, breakaway=-4.0)
    fl = base.with_launch_breakaway_floor(True)
    assert base.launch_breakaway_floor is False and fl.launch_breakaway_floor is True
    v = np.array([0.0, 0.3, 0.6, 0.0, 0.0])
    u = np.array([-9.0, -9.0, -9.0, -3.0, 5.0])
    raw = np.asarray(base.accel(v, u, launch=True))
    out = np.asarray(fl.accel(v, u, launch=True))
    assert raw[0] > 0.0 and raw[1] > 0.0  # 前提: 発進側の表は深いブレーキで加速と言う
    # 止まっている範囲（0.5 以下）で深い（|u| > 4）行だけ min(表, 0)
    assert out[0] == 0.0 and out[1] == 0.0
    assert out[2] == raw[2] and out[3] == raw[3] and out[4] == raw[4]
    # launch=False（本体・停止側）は触らない
    assert np.array_equal(fl.accel(v, u), base.accel(v, u))
    assert np.array_equal(fl.accel(v, u, stop=True), base.accel(v, u, stop=True))
    # スカラー入力
    assert fl.accel(0.0, -9.0, launch=True) == 0.0
    # invert: 止まっている範囲で動き出す開度より深い解は -4 に頭打ち。動いていれば触らない
    ui, _ = fl.invert(np.array([0.0, 2.0]), np.array([-5.0, -5.0]), launch=True)
    ur, _ = base.invert(np.array([0.0, 2.0]), np.array([-5.0, -5.0]), launch=True)
    assert ur[0] < -4.0 and ui[0] == -4.0 and ui[1] == ur[1]
    # meta が無い・非有限なら何もしない
    nob = _pedal_table(dl=12.0, ds=2.4).with_launch_breakaway_floor(True)
    assert np.array_equal(
        nob.accel(v, u, launch=True), _pedal_table(dl=12.0, ds=2.4).accel(v, u, launch=True)
    )


def test_launch_breakaway_floor_off_is_identical_plan() -> None:
    tb = _pedal_table(dl=3.0, ds=2.4, breakaway=-1.0)
    a = plan_for(_slow_launch, 10.0, tb, low_speed_side_rule="dir")
    tb_off = tb.with_launch_breakaway_floor(False)
    b = plan_for(_slow_launch, 10.0, tb_off, low_speed_side_rule="dir")
    assert np.array_equal(a.effort_pct, b.effort_pct)


# ---- 段7-b: 基準の動き出しに合わせてブレーキを離す ----

BK_U = -1.0  # meta launch_breakaway_u_pct（符号つき）。bk ＝ BD + 1.0 ＝ 8.0
BK = BD + abs(BK_U)
PRE = BK + 2.0
LAG = {"brake_lag_s": 1.0, "accel_lag_s": 0.1}
LB = LAG["brake_lag_s"]  # アクセルの窓（約 0.4 s 前から立ち上がりを拾う）より前に t_go が来る値


def _bk_table() -> ForwardTable:
    tb = linear_table()
    tb.meta["launch_breakaway_u_pct"] = BK_U
    return tb


def _stop_then_go(stop_s: float):
    def ref(t: float) -> float:
        return 0.0 if t <= stop_s else min(2.0 * (t - stop_s), 20.0)

    return ref


def _t0_row(plan: ForwardPlan, ref) -> int:
    v = np.array([ref(float(x)) for x in plan.t_s])
    return int(np.flatnonzero(v > 0.1)[0]) if (v > 0.1).any() else -1


def test_release_ramp_long_stop_ramps_from_pre_to_bk() -> None:
    ref = _stop_then_go(12.0)
    on = plan_for(ref, 16.0, _bk_table(), **LAG, launch_release_ramp=True)
    t0 = _t0_row(on, ref)
    i_go = t0 - int(round(LB / on.period_s))
    i_r = i_go - int(round(2.0 / on.period_s))
    assert on.effort_pct[i_r - 1] == pytest.approx(-STOP_PCT)  # 窓の手前は停車保持のまま
    assert on.effort_pct[i_r] == pytest.approx(-PRE)  # t_go − 2 秒: 手前の開度
    assert on.effort_pct[i_go] == pytest.approx(-BK)  # t_go: bk
    mid = (i_r + i_go) // 2
    frac = (mid - i_r) / (i_go - i_r)
    assert on.effort_pct[mid] == pytest.approx(-(PRE + (BK - PRE) * frac))  # 間は直線
    assert on.launch_prep is not None
    assert on.launch_prep[i_r : i_go + 1].all() and not on.launch_prep[: i_r - 1].any()
    assert np.all(on.effort_pct[i_r : i_go + 1] <= -BK + 1e-9)
    assert np.all(on.flag[i_r : i_go + 1] == FLAG_STOP)


def test_release_ramp_pre_never_deeper_than_stop_hold() -> None:
    ref = _stop_then_go(12.0)
    on = plan_for(
        ref, 16.0, _bk_table(), **LAG, launch_release_ramp=True,
        stop_brake_opening_pct=9.0,  # 停車保持 9% ＜ 手前の開度 10%
    )
    i_go = _t0_row(on, ref) - int(round(LB / on.period_s))
    i_r = i_go - int(round(2.0 / on.period_s))
    assert on.effort_pct[i_r] == pytest.approx(-9.0)  # 停車保持より深くしない


def test_release_ramp_short_stop_waits_at_bk() -> None:
    ref = _stop_then_go(3.0)
    on = plan_for(ref, 8.0, _bk_table(), **LAG, launch_release_ramp=True)
    off = plan_for(ref, 8.0, _bk_table(), **LAG)
    t0 = _t0_row(on, ref)
    i_go = t0 - int(round(LB / on.period_s))
    assert np.all(off.effort_pct[: i_go - 5] == -STOP_PCT)
    assert np.all(on.effort_pct[: i_go + 1] == pytest.approx(-BK))  # 停車保持の行も bk
    assert np.all(on.flag[on.flag == FLAG_STOP] == FLAG_STOP)
    assert on.launch_prep is not None and on.launch_prep[: i_go + 1].all()


def test_release_ramp_brake_never_shallower_before_t_go() -> None:
    """型B: 基準が 1 秒目から急に立ち上がる。off では t_go の前に戻しが入るが、on では入らない。"""
    def ref(t: float) -> float:
        return 0.0 if t <= 8.0 else min(8.0 * (t - 8.0), 30.0)

    on = plan_for(
        ref, 14.0, _bk_table(), **LAG, launch_release_ramp=True
    )
    i_go = _t0_row(on, ref) - int(round(LB / on.period_s))
    assert np.all(on.effort_pct[: i_go + 1] <= -BK + 1e-9)


def test_release_ramp_off_is_identical_and_no_meta_is_noop() -> None:
    ref = _stop_then_go(12.0)
    base = plan_for(ref, 16.0, _bk_table(), **LAG)
    off = plan_for(ref, 16.0, _bk_table(), **LAG, launch_release_ramp=False)
    assert base.launch_prep is None and off.launch_prep is None
    assert np.array_equal(base.effort_pct, off.effort_pct)
    assert np.array_equal(base.flag, off.flag)
    nometa = plan_for(ref, 16.0, linear_table(), **LAG)
    nm_on = plan_for(ref, 16.0, linear_table(), **LAG, launch_release_ramp=True)
    assert np.array_equal(nometa.effort_pct, nm_on.effort_pct)
    assert nm_on.launch_prep is not None and not nm_on.launch_prep.any()


def test_release_ramp_tiny_stops_do_not_crash() -> None:
    for stop_s in (0.0, 0.05, 0.1, 0.2):
        ref = _stop_then_go(stop_s)
        p = plan_for(ref, 6.0, _bk_table(), **LAG, launch_release_ramp=True)
        assert len(p.effort_pct) == len(p.t_s) and np.all(np.isfinite(p.effort_pct))
    # 停車のまま終わる・停車の途中で始まる（基準が最初から動いている）場合も壊れない
    p = plan_for(lambda t: 0.0, 3.0, _bk_table(), launch_release_ramp=True)
    assert not p.launch_prep.any()
    p = plan_for(lambda t: 20.0, 3.0, _bk_table(), launch_release_ramp=True)
    assert not p.launch_prep.any()


def test_release_ramp_also_in_switch_lead_two_pass() -> None:
    ref = _stop_then_go(12.0)
    on = plan_for(
        ref, 16.0, _bk_table(), **LAG, launch_release_ramp=True, switch_lead=True
    )
    assert on.launch_prep is not None and on.launch_prep.any()


# ---- 段7-c: すき間で失う分の補償 ----

def _gap_table() -> ForwardTable:
    tb = linear_table()
    for p in ("accel", "brake"):
        tb.meta[f"{p}_return_delay_s"] = 0.3
        tb.meta[f"{p}_return_rate_pct_s"] = 20.0
    return tb


def _valley(t: float) -> float:
    """50 km/h から 3 秒間 −4 km/h/s で減速して、そのまま +4 km/h/s で加速（谷）。"""
    return 50.0 - 4.0 * t if t < 3.0 else 38.0 + 4.0 * (t - 3.0)


def _hill(t: float) -> float:
    """+4 km/h/s で 3 秒加速して、そのまま −4 km/h/s で減速（山）。"""
    return 50.0 + 4.0 * t if t < 3.0 else 62.0 - 4.0 * (t - 3.0)


def _gap_plan(ref, comp: bool, **kw):
    return plan_for(
        ref, 6.0, _gap_table(), return_gap_enabled=True, switch_gap_compensate=comp,
        brake_lag_s=0.1, accel_lag_s=0.1, **kw,
    )


def test_gap_compensation_valley_deepens_brake_before_gap() -> None:
    off, on = _gap_plan(_valley, False), _gap_plan(_valley, True)
    assert off.return_gap.n_cut >= 1  # 前提: 谷で戻す側が削られている
    assert on.return_gap.comp_events >= 1 and on.return_gap.comp_dv_kmh > 0.0
    brake = (off.effort_pct < 0.0) & (on.effort_pct < 0.0)
    assert np.any(on.effort_pct[brake] < off.effort_pct[brake] - 1e-9)  # 深くなった
    assert np.all(on.effort_pct[brake] <= off.effort_pct[brake] + 1e-9)
    assert on.return_gap.n_violations == 0
    # 計画が持つ要る加速度は上乗せ前のまま
    assert np.allclose(on.need_accel_kmhs, off.need_accel_kmhs, atol=1e-9)


def test_gap_compensation_adds_dv_to_need_of_rows_before_gap() -> None:
    from tests.research.fwd_plan import _gap_compensation, return_time_of

    tb = _gap_table()
    aux: dict = {}
    base = plan_for(
        _valley, 6.0, tb, return_gap_enabled=False, brake_lag_s=0.1, accel_lag_s=0.1,
        _aux_out=aux,
    )
    ret = return_time_of(tb, AD, BD)
    add, dv, n_ev, trimmed = _gap_compensation(base, aux, ret, base.period_s)
    assert n_ev >= 1 and dv > 0.0 and trimmed.any()
    rows = np.flatnonzero(add != 0.0)
    assert np.all(add[rows] < 0.0)  # 谷: 減速が足りない → 要る加速度を下げる（ブレーキを深く）
    assert np.all(base.effort_pct[rows] < 0.0) and not np.any(trimmed & (add != 0.0))
    # 上乗せの合計 × 周期 ＝ −Σ|Δv|（削った行の（要る加速度 − 惰行の加速度）× 周期の合計）
    assert float(add.sum()) * base.period_s == pytest.approx(
        -sum(
            abs(float(np.sum(base.need_accel_kmhs[trimmed] - aux["coast_b"][trimmed])))
            * base.period_s
            for _ in [0]
        ),
        rel=1e-6,
    )


def test_gap_compensation_hill_deepens_accel() -> None:
    off, on = _gap_plan(_hill, False), _gap_plan(_hill, True)
    assert off.return_gap.n_cut >= 1
    assert on.return_gap.comp_events >= 1
    accel = (off.effort_pct > 0.0) & (on.effort_pct > 0.0)
    assert np.any(on.effort_pct[accel] > off.effort_pct[accel] + 1e-9)
    assert np.all(on.effort_pct[accel] >= off.effort_pct[accel] - 1e-9)
    assert on.return_gap.n_violations == 0


def test_gap_compensation_off_is_identical_and_no_double_pedal() -> None:
    off = _gap_plan(_valley, False)
    dflt = plan_for(
        _valley, 6.0, _gap_table(), return_gap_enabled=True, brake_lag_s=0.1, accel_lag_s=0.1
    )
    assert np.array_equal(off.effort_pct, dflt.effort_pct)
    assert off.return_gap.comp_events == 0 and "補償" not in off.return_gap.summary()
    on = _gap_plan(_valley, True)
    assert "すき間の補償" in on.return_gap.summary()
    # 一定速度の基準では切り替えが無いので何も変わらない
    c_off = plan_for(lambda t: 50.0, 3.0, _gap_table(), return_gap_enabled=True)
    c_on = plan_for(
        lambda t: 50.0, 3.0, _gap_table(), return_gap_enabled=True, switch_gap_compensate=True
    )
    assert np.array_equal(c_off.effort_pct, c_on.effort_pct)


def test_gap_compensation_with_preposition_and_switch_lead() -> None:
    for kw in ({"preposition_enabled": True}, {"switch_lead": True}):
        on = _gap_plan(_valley, True, **kw)
        assert on.return_gap.n_violations == 0
        assert len(on.effort_pct) == len(on.t_s) and np.all(np.isfinite(on.effort_pct))


# ---- 段9-b: すき間を前と後ろの損の合計が最小で取る ----

def _gap_ml(effort_list, need, coast_a=0.0, coast_b=0.0, flag=None):
    from tests.research.fwd_plan import ensure_return_gap  # noqa: PLC0415

    effort = np.array(effort_list, dtype=float)
    n = len(effort)
    fl = np.full(n, FLAG_COAST, dtype=int) if flag is None else np.array(flag)
    st = ensure_return_gap(
        effort, fl, ret=_ret(), period_s=DT, need_accel=np.asarray(need, dtype=float),
        coast_a=np.full(n, coast_a), coast_b=np.full(n, coast_b),
    )
    return effort, fl, st


def test_gap_min_loss_none_is_identical_to_before() -> None:
    eff = [24.0] * 20 + [-24.0] * 10
    e0, f0, s0 = _gap(eff)
    e1, f1, s1 = _gap_ml(eff, np.zeros(30))  # 損が全部 0 ＝ 同点 ＝ 前で削る
    assert np.array_equal(e0, e1) and np.array_equal(f0, f1)
    assert (s1.n_cut, s1.n_delayed, s1.n_violations) == (s0.n_cut, s0.n_delayed, 0)
    assert s1.cut_s == pytest.approx(s0.cut_s)


def test_gap_min_loss_delays_back_when_front_loss_is_large() -> None:
    # 前（アクセル）の行は要る加速度が大きい（削ると損が大きい）、後ろ（ブレーキ）は惰行と同じ
    need = [5.0] * 20 + [0.0] * 10
    e, fl, st = _gap_ml([24.0] * 20 + [-24.0] * 10, need)
    assert st.n_cut == 0 and st.n_delayed == 1 and st.delayed_s == pytest.approx(7 * DT)
    assert np.all(e[:20] == 24.0) and np.all(e[20:27] == 0.0) and np.all(e[27:] == -24.0)
    assert np.all(fl[20:27] == FLAG_COAST) and st.n_violations == 0


def test_gap_min_loss_splits_front_and_back() -> None:
    # 前は 1 行 0.05 の損・後ろは先頭 3 行だけ損が無く、その先は大きい → 後ろ 3 行 ＋ 前 4 行
    need = [1.0] * 20 + [0.0] * 3 + [-5.0] * 7
    e, _, st = _gap_ml([24.0] * 20 + [-24.0] * 10, need)
    assert st.n_cut == 1 and st.cut_s == pytest.approx(4 * DT)
    assert st.n_delayed == 1 and st.delayed_s == pytest.approx(3 * DT)
    assert np.all(e[16:23] == 0.0) and e[15] == 24.0 and e[23] == -24.0


def test_gap_min_loss_stop_hold_and_prepos_keep_old_behaviour() -> None:
    need = [5.0] * 20 + [0.0] * 10
    fl = [FLAG_COAST] * 20 + [FLAG_PREPOS] * 10
    e0, f0, _ = _gap([24.0] * 20 + [-24.0] * 10, flag=fl)
    e1, f1, _ = _gap_ml([24.0] * 20 + [-24.0] * 10, need, flag=fl)
    assert np.array_equal(e0, e1) and np.array_equal(f0, f1)
    fl2 = [FLAG_STOP] * 20 + [FLAG_COAST] * 10
    e2, _, _ = _gap([-24.0] * 20 + [24.0] * 10, flag=fl2)
    e3, _, _ = _gap_ml([-24.0] * 20 + [24.0] * 10, [-5.0] * 20 + [0.0] * 10, flag=fl2)
    assert np.array_equal(e2, e3)


def test_gap_min_loss_off_plan_is_identical_and_compensates_front_only() -> None:
    off = _gap_plan(_valley, True)
    dflt = _gap_plan(_valley, True, switch_gap_min_loss=False)
    assert np.array_equal(off.effort_pct, dflt.effort_pct)
    on = _gap_plan(_valley, True, switch_gap_min_loss=True)
    assert on.return_gap.n_violations == 0
    assert on.return_gap.n_cut == 1 and on.return_gap.n_delayed == 1  # 谷: 前と後ろに分ける
    assert on.return_gap.comp_events == 1
    # 補償の Δv は前で削った行の分だけ（後ろで遅らせた行は含まない）
    from tests.research.fwd_plan import _gap_compensation, return_time_of  # noqa: PLC0415

    tb = _gap_table()
    aux: dict = {}
    base = plan_for(
        _valley, 6.0, tb, return_gap_enabled=False, brake_lag_s=0.1, accel_lag_s=0.1,
        _aux_out=aux,
    )
    ret = return_time_of(tb, AD, BD)
    add, dv, n_ev, trimmed = _gap_compensation(base, aux, ret, base.period_s, min_loss=True)
    cut_rows = trimmed & (base.effort_pct < 0.0)  # 谷: 前はブレーキ
    front_dv = abs(float(np.sum(base.need_accel_kmhs[cut_rows] - aux["coast_b"][cut_rows])))
    assert n_ev == 1 and dv == pytest.approx(front_dv * base.period_s)
    assert trimmed.sum() > cut_rows.sum()  # 後ろで遅らせた行もある
    # 山は全部後ろで遅らせる → 補償するものが無い
    hill = _gap_plan(_hill, True, switch_gap_min_loss=True)
    assert hill.return_gap.n_cut == 0 and hill.return_gap.comp_events == 0


# ---- 段9-c: アクセルの戻しの層を学習の範囲の中だけで使う ----

def _in_range_table(max_pct: float = 8.0, peak: float | None = 100.0) -> ForwardTable:
    t = _accel_release_variant()
    t.meta["accel_release_max_pct"] = max_pct
    if peak is not None:
        t.meta["accel_release_peak_max_pct"] = peak
    return t


def test_accel_release_in_range_only_off_or_wide_range_is_identical() -> None:
    t = _in_range_table(max_pct=1000.0)
    base = plan_for(_accel_release_ref, 8.0, t)
    wide = plan_for(_accel_release_ref, 8.0, t, accel_release_in_range_only=True)
    assert np.array_equal(base.effort_pct, wide.effort_pct)  # 範囲が十分広ければ同じ


def test_accel_release_in_range_only_peak_beyond_range_uses_body() -> None:
    body = plan_for(_accel_release_ref, 8.0, _accel_release_variant(with_layers=False))
    layered = plan_for(_accel_release_ref, 8.0, _in_range_table())
    # 山（直近の最大 u）が範囲の外 → 層を使わない ＝ 本体のまま
    narrow = plan_for(
        _accel_release_ref, 8.0, _in_range_table(peak=0.5), accel_release_in_range_only=True
    )
    assert np.allclose(narrow.effort_pct, body.effort_pct, atol=1e-9)
    assert not np.allclose(layered.effort_pct, body.effort_pct, atol=1e-6)  # 前提: 層は効く


def test_accel_release_in_range_only_amount_beyond_cap_uses_body_at_that_point() -> None:
    body = plan_for(_accel_release_ref, 8.0, _accel_release_variant(with_layers=False))
    # 戻し量の頭打ち 0.1% より戻し量が大きい点は本体の表（戻し量 0）で引く
    capped = plan_for(
        _accel_release_ref, 8.0, _in_range_table(max_pct=0.1), accel_release_in_range_only=True
    )
    off = plan_for(_accel_release_ref, 8.0, _in_range_table(max_pct=0.1))
    # off は頭打ち 0.1 の層で解く ＝ 本体からずれる行がある。on は戻し量 > 0.1 の行で本体
    assert not np.allclose(off.effort_pct, body.effort_pct, atol=1e-6)
    d_on = np.abs(capped.effort_pct - body.effort_pct)
    d_off = np.abs(off.effort_pct - body.effort_pct)
    assert d_on.max() <= d_off.max() + 1e-9


def test_accel_release_in_range_only_without_meta_raises() -> None:
    with pytest.raises(ValueError, match="accel_release_peak_max_pct"):
        plan_for(
            _accel_release_ref, 8.0, _in_range_table(peak=None), accel_release_in_range_only=True
        )
    # 戻しの層が無い表はメタが無くてもエラーにしない
    plan_for(
        _accel_release_ref, 8.0, _accel_release_variant(with_layers=False),
        accel_release_in_range_only=True,
    )


# ---- 段10-1: ペダルの実際の位置を追う部品 ----

def _pt(**kw):
    from tests.research.fwd_plan import PedalTrack  # noqa: PLC0415

    return PedalTrack(**kw)


def _track(effort, track=None, **kw):
    from tests.research.fwd_plan import track_pedal_position  # noqa: PLC0415

    return track_pedal_position(
        np.asarray(effort, dtype=float), period_s=DT, track=track or _pt(), accel_min_pct=AD,
        brake_min_pct=BD, **kw,
    )


def test_pedal_track_defaults_are_the_measured_values() -> None:
    t = _pt()
    accel = (t.accel_delay_s, t.accel_max_rate_pct_s, t.accel_max_accel_pct_s2)
    brake = (t.brake_delay_s, t.brake_max_rate_pct_s, t.brake_max_accel_pct_s2)
    assert accel == (0.02, 100.0, 2000.0) and brake == (0.02, 90.0, 1400.0)
    assert (t.accel_effect_lag_s, t.brake_effect_lag_s) == (0.10, 0.10)


def _ref_follow(cmd, p0, delay_s, vmax, amax, mn):
    """独立に書いた参照実装（式をそのまま）。"""
    n = len(cmd)
    idx = np.arange(n, dtype=float)
    c = np.maximum(np.interp(idx - delay_s / DT, idx, cmd), mn)
    x, v, h = p0, 0.0, DT / 10
    pos = [x]
    for k in range(1, n):
        for _ in range(10):
            e = c[k - 1] - x
            vd = np.sign(e) * min(vmax, np.sqrt(2 * amax * abs(e)))
            v += np.clip(vd - v, -amax * h, amax * h)
            x += v * h
            if abs(c[k - 1] - x) < 0.05 and abs(v) < 2 * amax * h:
                x, v = c[k - 1], 0.0
        pos.append(x)
    return np.array(pos)


def test_track_pedal_position_matches_reference_formula() -> None:
    t = _pt()
    eff = [0.0] * 4 + [60.0] * 12 + [20.0] * 6 + [0.0] * 14
    pa, pb = _track(eff)
    cmd = np.maximum(np.where(np.asarray(eff) > 0.0, eff, AD), AD)
    ref = _ref_follow(
        cmd, AD, t.accel_delay_s, t.accel_max_rate_pct_s, t.accel_max_accel_pct_s2, AD
    )
    assert np.allclose(pa, ref, atol=1e-9) and np.all(pb == BD)


def test_track_pedal_position_short_move_is_slow_long_move_caps_at_vmax() -> None:
    t = _pt(accel_delay_s=0.0)
    # 短い移動（+3 %）: 1 周期で 3 % には着かない（加速度の上限で平均が遅い。100 %/s 一定なら着く）
    pa, _ = _track([0.0] * 3 + [AD + 3.0] * 10, t)
    assert AD + 1.0 < pa[4] < AD + 3.0 - 0.1  # 行 3 の指令へ 1 周期後（5 % ＝ 100 %/s なら着く）
    assert pa[5] == AD + 3.0  # 着いたら止まる
    # 長い移動（+60 %）: 速さは vmax（100 %/s ＝ 1 周期 5 %）を超えない
    pa, _ = _track([0.0] * 3 + [64.0] * 30, t)
    assert np.all(np.diff(pa) <= 100.0 * DT + 1e-9)
    assert pa[4] - pa[3] < 100.0 * DT * 0.7  # 動き出しは加速度の上限で遅い
    assert np.max(np.diff(pa)) == pytest.approx(100.0 * DT, rel=0.05)  # 頭打ちになる
    # 十分な時間があれば着いて止まる（min のわずか上には残らない）
    assert pa[-1] == 64.0
    pa, _ = _track([0.0] * 3 + [64.0] * 12 + [0.0] * 30, t)
    assert pa[-1] == AD and np.all(pa >= AD - 0.5)  # 戻って min で止まる（行き過ぎは 0.5 % 未満）


def test_follow_resume_equals_full_tracking() -> None:
    from tests.research.fwd_plan import _follow_pedal, _follow_resume  # noqa: PLC0415

    rng = np.random.default_rng(3)
    cmd_base = np.maximum(rng.uniform(0.0, 50.0, 60), AD)
    cmd_new = cmd_base.copy()
    cmd_new[25:40] = AD  # 途中から違う
    vlog = np.zeros(60)
    pos_base = _follow_pedal(cmd_base, None, 0.02, 100.0, 2000.0, AD, DT, vlog=vlog)
    got = _follow_resume(cmd_new, cmd_base, pos_base, vlog, 0.02, 100.0, 2000.0, AD, DT)
    want = _follow_pedal(cmd_new, None, 0.02, 100.0, 2000.0, AD, DT)
    assert np.array_equal(got, want)
    assert np.array_equal(_follow_resume(cmd_base, cmd_base, pos_base, vlog, 0.02, 100.0,
                                         2000.0, AD, DT), pos_base)


def test_track_pedal_position_delay_and_brake_side() -> None:
    eff = [0.0] * 4 + [-30.0] * 30
    t0 = _pt(brake_delay_s=0.0)
    t1 = _pt(brake_delay_s=0.10)
    _, p0 = _track(eff, t0)
    _, p1 = _track(eff, t1)
    r0 = int(np.flatnonzero(p0 > BD + 1e-9)[0])
    r1 = int(np.flatnonzero(p1 > BD + 1e-9)[0])
    assert r1 - r0 == 2  # 遅れ 0.10 s ＝ 2 周期
    assert np.all(np.diff(p0) <= 90.0 * DT + 1e-9)  # ブレーキの速さの上限は 90 %/s
    pa, _ = _track(eff, t0)
    assert np.all(pa == AD)


def test_track_pedal_position_fractional_delay_interpolates_and_p0() -> None:
    t = _pt(accel_delay_s=0.025)
    t0 = _pt(accel_delay_s=0.0)
    pa, _ = _track([0.0, 0.0, 6.0, 6.0, 6.0, 6.0], t)
    pa0, _ = _track([0.0, 0.0, 6.0, 6.0, 6.0, 6.0], t0)
    # 半周期の遅れ（直線補間）: 行 2 の指令は (4 + 6) / 2 ＝ 5 になる。遅れなしより遅い
    assert pa[3] < pa0[3] and pa[3] <= 5.0 + 0.3 and pa[3] > AD
    pa, _ = _track([0.0, 0.0], t, accel_p0=30.0)
    assert pa[0] == 30.0 and pa[1] < 30.0  # 開始位置は指定できる


def _pos_table():
    return linear_table(coast_at_zero=-1.0, v_coef=0.0)


def _pred(table, v, pa, pb, **kw):
    from tests.research.fwd_plan import predicted_accel  # noqa: PLC0415

    kw.setdefault("track", _pt())
    return predicted_accel(
        table, np.asarray(v, dtype=float), np.asarray(pa, dtype=float),
        np.asarray(pb, dtype=float), accel_min_pct=AD, brake_min_pct=BD, period_s=DT, **kw,
    )


def test_predicted_accel_coast_when_both_at_min_and_sides() -> None:
    tb = _pos_table()
    n = 6
    v = np.full(n, 50.0)
    a = _pred(tb, v, np.full(n, AD), np.full(n, BD))
    assert np.allclose(a, tb.coast_accel(50.0))  # 両方 min は惰行
    a = _pred(tb, v, np.full(n, AD + 10.0), np.full(n, BD))
    assert np.allclose(a, tb.accel(50.0, 10.0))  # アクセルは u ＝ 位置 − min
    a = _pred(tb, v, np.full(n, AD), np.full(n, BD + 10.0))
    assert np.allclose(a, tb.accel(50.0, -10.0))  # ブレーキは u を負に


def test_predicted_accel_effect_lag_shifts_position() -> None:
    tb = _pos_table()
    n = 10
    v = np.full(n, 50.0)
    pa = np.full(n, AD)
    pa[4:] = AD + 10.0
    a0 = _pred(tb, v, pa, np.full(n, BD), track=_pt(accel_effect_lag_s=0.0))
    a1 = _pred(tb, v, pa, np.full(n, BD), track=_pt(accel_effect_lag_s=0.10))
    up = tb.accel(50.0, 10.0)
    assert a0[4] == pytest.approx(up) and a1[4] == pytest.approx(tb.coast_accel(50.0))
    assert a1[6] == pytest.approx(up)  # λ 0.10 s ＝ 2 周期あとに効く
    a2 = _pred(tb, v, pa, np.full(n, BD), track=_pt(accel_effect_lag_s=0.025))
    assert tb.coast_accel(50.0) < a2[4] < up  # 周期の端数は補間


def test_position_return_row() -> None:
    from tests.research.fwd_plan import position_return_row  # noqa: PLC0415

    pos = np.array([30.0, 20.0, 9.0, 4.04, 4.0, 4.0])
    assert position_return_row(pos, 0, 4.0) == 3  # min + 0.05 以下になる最初の行
    assert position_return_row(pos, 4, 4.0) == 4
    assert position_return_row(np.array([30.0, 20.0]), 0, 4.0) is None
    assert position_return_row(np.array([4.0, 20.0, 4.0]), 1, 4.0) == 2  # start より前は見ない
    assert position_return_row(np.array([4.06, 4.2]), 0, 4.0) is None  # tol を超える


# ---- 段10-2: 案 R（switch_gap_rule） ----

def _rule_ctx(n, ref_acc, coast=-2.5, v=50.0, table=None):
    from tests.research.fwd_plan import GapContext, PedalTrack  # noqa: PLC0415

    return GapContext(
        table=table or linear_table(), v=np.full(n, v), ref_acc=np.asarray(ref_acc, dtype=float),
        coast_a=np.full(n, coast), coast_b=np.full(n, coast), track=PedalTrack(),
        accel_min_pct=AD, brake_min_pct=BD, brake_lag_s=0.1,
    )


def _rule_gap(effort_list, ctx, flag=None):
    from tests.research.fwd_plan import ensure_return_gap  # noqa: PLC0415

    effort = np.array(effort_list, dtype=float)
    fl = np.full(len(effort), FLAG_COAST, dtype=int) if flag is None else np.array(flag)
    st = ensure_return_gap(effort, fl, ret=_ret(amin=AD, bmin=BD), period_s=DT, rule_ctx=ctx)
    return effort, fl, st


def test_rule_peak_stops_accel_before_apex_and_brake_starts_at_position_min() -> None:
    from tests.research.fwd_plan import position_return_row  # noqa: PLC0415

    n = 100
    eff = np.zeros(n)
    eff[20:56] = 24.0  # アクセル（last ＝ 55）
    eff[58:80] = -24.0  # ブレーキ（i ＝ 58。すき間 2 行）
    ref_acc = np.where(np.arange(n) < 60, 1.0, -5.0)  # 頂点 k_v ＝ 60（惰行 −2.5 を初めて下回る）
    e, fl, st = _rule_gap(eff, _rule_ctx(n, ref_acc))
    s = int(np.flatnonzero(e[20:56] > 0.0)[-1]) + 21  # アクセルの残った最後の行の次 ＝ 戻し始め
    assert 21 <= s < 60 and np.all(e[s:56] == 0.0) and np.all(fl[s:56] == FLAG_COAST)
    assert st.n_rule_peak == 1 and st.n_rule_valley == 0 and st.rule_peak_spans == [(s, 56)]
    # ブレーキはアクセルの位置が min に着いた行 r の次の行（r + 1）から（前へは動かさない）。
    # 安全網は前の周期の実開度で見るので 1 周期の余裕を取る（旧 ReturnTime の ＋1 周期と同じ）
    pa, _ = _track(e)
    r = position_return_row(pa, s, AD)
    b = r + 1
    first_brake = int(np.flatnonzero(e[56:] < 0.0)[0]) + 56
    assert b > 58 and first_brake == b and np.all(e[58:b] == 0.0)
    # 変わり目の真ん中は面積で決める（段10-2'）: 独立に書いた採点（|cum| 最小・同点は遅い方）
    tb = _rule_ctx(n, ref_acc).table
    cands = []
    for ss in range(21, 57):
        ee = eff.copy()
        ee[ss:56] = 0.0
        p, _ = _track(ee)
        bb = position_return_row(p, ss, AD)
        if bb is not None:
            cands.append((ss, bb + 1))
    r0 = min(c[0] for c in cands) - 10
    r1 = min(max(c[1] for c in cands) + round(0.5 / DT), n - 1)
    cums = {}
    for ss, bb in cands:
        ee = eff.copy()
        ee[ss:56] = 0.0
        ee[58 : max(bb, 58)] = 0.0
        pa_, pb_ = _track(ee)
        pr = _pred(tb, np.full(n, 50.0), pa_, pb_)
        cums[ss] = abs(float(np.sum((pr[r0 : r1 + 1] - ref_acc[r0 : r1 + 1]) * DT)))
    best = min(cums.values())
    assert s == max(ss for ss, c in cums.items() if c <= best + 1e-9)


def test_rule_peak_searches_only_near_the_end_of_a_long_accel_run() -> None:
    """長いアクセルの連なりでも、戻し始める行は連なりの最後から RULE_PEAK_RANGE_S 以内。"""
    from tests.research import fwd_plan  # noqa: PLC0415

    n = 400
    eff = np.zeros(n)
    eff[20:300] = 24.0  # 14 秒の連なり（last ＝ 299）
    eff[302:340] = -24.0
    ref_acc = np.where(np.arange(n) < 305, 1.0, -5.0)
    _, _, st = _rule_gap(eff, _rule_ctx(n, ref_acc))
    s = st.rule_peak_spans[0][0]
    assert s >= 300 - round(fwd_plan.RULE_PEAK_RANGE_S / DT)


def _corner_s_lo(eff, k_pk, last, a):
    """段11-4 の下限を独立に書く: max(a+1, k_pk − m)、m ＝（r − k_pk）＋ 1 ＋ ceil(遅れ / 周期)。"""
    from tests.research.fwd_plan import PedalTrack, position_return_row  # noqa: PLC0415

    e = np.array(eff, dtype=float)
    e[k_pk : last + 1] = 0.0
    pa, _ = _track(e)
    r = position_return_row(pa, k_pk, AD)
    lag = math.ceil(PedalTrack().accel_effect_lag_s / DT)
    m = lag + 1 if r is None else (r - k_pk) + 1 + lag
    return max(a + 1, k_pk - m)


def _corner_ctx(n, ref_acc, on):
    ctx = _rule_ctx(n, ref_acc)
    ctx.peak_from_corner = on
    return ctx


def test_rule_peak_from_corner_off_is_unchanged() -> None:
    """段11-4: 既定（False）は固定 RULE_PEAK_RANGE_S の下限のまま。"""
    n = 400
    eff = np.zeros(n)
    eff[20:300] = 24.0
    eff[302:340] = -24.0
    ref_acc = np.where(np.arange(n) < 305, 1.0, -5.0)
    assert _rule_ctx(n, ref_acc).peak_from_corner is False
    e0, _, st0 = _rule_gap(eff, _rule_ctx(n, ref_acc))
    e1, _, st1 = _rule_gap(eff, _corner_ctx(n, ref_acc, False))
    assert np.array_equal(e0, e1) and st0.rule_peak_spans == st1.rule_peak_spans


def test_rule_peak_from_corner_lower_bound_is_local_max_minus_return_rows() -> None:
    """段11-4: 頂点の手前の最後の極大 k_pk（270）− m から探す。長い連なりの頭の水準は拾わない。"""
    n = 400
    eff = np.zeros(n)
    eff[20:300] = 24.0  # last ＝ 299
    eff[302:340] = -24.0
    idx = np.arange(n)
    # 270 までは 1.0 で（270 が最後の極大）、その後ゆっくり下がって 306 で惰行 −2.5 を下回る
    ref_acc = np.where(idx <= 270, 1.0, 1.0 - 0.1 * (idx - 270))
    _, _, st = _rule_gap(eff, _corner_ctx(n, ref_acc, True))
    s = st.rule_peak_spans[0][0]
    s_lo = _corner_s_lo(eff, 270, 299, 20)
    assert s_lo < 270 and s >= s_lo and s <= 300


def test_rule_peak_from_corner_plateau_stops_at_vertex_side_end() -> None:
    """段11-4: 基準が平ら（等しい）なら k_pk は頂点側の端（頂点の 1 行手前）。"""
    n = 400
    eff = np.zeros(n)
    eff[20:300] = 24.0
    eff[302:340] = -24.0
    ref_acc = np.where(np.arange(n) < 305, 1.0, -5.0)  # k_v ＝ 305、平らな 1.0 が 304 まで
    e, _, _ = _rule_gap(eff, _corner_ctx(n, ref_acc, True))
    s = int(np.flatnonzero(e[20:300] > 0.0)[-1]) + 21  # 戻し始め（切らなければ 300）
    s_lo = _corner_s_lo(eff, 304, 299, 20)
    assert s >= s_lo
    # 平らの頭（20 行目）まで戻る解釈なら下限は a + 1 ＝ 21。そうはなっていない
    assert s_lo > 21


def test_rule_peak_brake_already_late_is_not_moved_forward() -> None:
    n = 100
    eff = np.zeros(n)
    eff[20:56] = 24.0
    eff[75:90] = -24.0  # ブレーキはずっと後ろ
    ref_acc = np.where(np.arange(n) < 60, 1.0, -5.0)
    e, _, _ = _rule_gap(eff, _rule_ctx(n, ref_acc))
    assert np.all(e[75:90] == -24.0)  # b ≤ i ならブレーキはそのまま


def test_rule_valley_cuts_brake_tail_and_keeps_accel() -> None:
    from tests.research.fwd_plan import position_return_row  # noqa: PLC0415

    n = 100
    eff = np.zeros(n)
    eff[20:60] = -24.0
    eff[62:80] = 24.0
    e, fl, st = _rule_gap(eff, _rule_ctx(n, np.zeros(n)))
    assert np.all(e[62:80] == 24.0)  # アクセルは遅らせない
    c = int(np.count_nonzero((eff < 0.0) & (e == 0.0)))
    assert c >= 1 and e[20] == -24.0 and st.n_rule_valley == 1 and st.n_delayed == 0
    assert st.rule_valley_spans == [(60 - c, 60)] and np.all(fl[60 - c : 60] == FLAG_COAST)
    _, pb = _track(e)
    # 着く行の次の行が i 以下（安全網は前の周期の実開度で見る。1 周期の余裕）
    assert position_return_row(pb, 60 - c + 1, BD) + 1 <= 62
    # 最小の c: 1 行少なければ間に合わない
    e1 = eff.copy()
    e1[60 - (c - 1) : 60] = 0.0
    _, pb1 = _track(e1)
    r1 = position_return_row(pb1, 60 - (c - 1) + 1, BD)
    assert r1 is None or r1 + 1 > 62


def test_rule_valley_already_in_time_does_nothing() -> None:
    n = 100
    eff = np.zeros(n)
    eff[20:60] = -24.0
    eff[75:80] = 24.0  # 十分遠い
    e, _, st = _rule_gap(eff, _rule_ctx(n, np.zeros(n)))
    assert np.array_equal(e, eff) and st.n_rule_valley == 0 and st.n_cut == 0


def test_rule_ignores_nonnegative_coast_and_stop_prepos() -> None:
    n = 100
    ref_acc = np.where(np.arange(n) < 60, 1.0, -5.0)
    for eff in (
        np.r_[np.zeros(20), np.full(36, 24.0), np.zeros(2), np.full(22, -24.0), np.zeros(20)],
        np.r_[np.zeros(20), np.full(40, -24.0), np.zeros(2), np.full(18, 24.0), np.zeros(20)],
    ):
        e0, f0, _ = _gap(eff.tolist())
        e1, f1, st = _rule_gap(eff, _rule_ctx(n, ref_acc, coast=0.5))  # 惰行 ≥ 0 は対象外
        assert np.array_equal(e0, e1) and np.array_equal(f0, f1)
        assert st.n_rule_peak == 0 and st.n_rule_valley == 0
        nz = np.flatnonzero(eff != 0.0)
        last_row = int(nz[np.flatnonzero(np.diff(nz) > 1)[0]])  # 戻す側の最後の行
        flags = []
        for kind, row in (
            (FLAG_PREPOS, last_row + 3), (FLAG_STOP, last_row + 3), (FLAG_STOP, last_row)
        ):
            fl = [FLAG_COAST] * n
            fl[row] = kind
            flags.append(fl)
        for fl in flags:
            e2, f2, _ = _gap(eff.tolist(), flag=fl)
            e3, f3, st3 = _rule_gap(eff, _rule_ctx(n, ref_acc), flag=fl)
            assert np.array_equal(e2, e3) and np.array_equal(f2, f3)
            assert st3.n_rule_peak == 0 and st3.n_rule_valley == 0


def test_rule_peak_without_apex_falls_back() -> None:
    n = 100
    eff = np.zeros(n)
    eff[20:56] = 24.0
    eff[56:80] = -24.0
    e0, _, _ = _gap(eff.tolist())
    e1, _, st = _rule_gap(eff, _rule_ctx(n, np.full(n, 1.0)))  # 基準の加速度が惰行を下回らない
    assert np.array_equal(e0, e1) and st.n_rule_peak == 0


def test_rule_off_plan_is_identical_and_summary_has_counts() -> None:
    off = _gap_plan(_valley, True)
    dflt = _gap_plan(_valley, True, switch_gap_rule=False)
    assert np.array_equal(off.effort_pct, dflt.effort_pct)
    assert "案 R" not in off.return_gap.summary()
    on = _gap_plan(_hill, True, switch_gap_rule=True)
    assert on.return_gap.n_rule_peak + on.return_gap.n_rule_valley >= 1
    assert "案 R で置き直し" in on.return_gap.summary()
    assert np.all(np.isfinite(on.effort_pct))


def test_rule_compensation_valley_dv_is_smaller_than_coast_version() -> None:
    from tests.research.fwd_plan import (  # noqa: PLC0415
        GapContext,
        PedalTrack,
        _gap_compensation,
        return_time_of,
    )

    tb = _gap_table()
    aux: dict = {}
    base = plan_for(
        _valley, 6.0, tb, return_gap_enabled=False, brake_lag_s=0.1, accel_lag_s=0.1,
        switch_gap_rule=True, _aux_out=aux,
    )
    ret = return_time_of(tb, AD, BD)
    ctx = GapContext(
        table=tb, v=aux["v"], ref_acc=aux["ref_acc"], coast_a=aux["coast_a"],
        coast_b=aux["coast_b"], track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD,
        brake_lag_s=0.1,
    )
    add_r, dv_r, n_r, trimmed_r = _gap_compensation(base, aux, ret, base.period_s, rule_ctx=ctx)
    # 同じ削り（R の削り）を惰行の加速度で数えた Δv ＝ 今の式
    assert n_r >= 1 and trimmed_r.any()
    cut = trimmed_r & (base.effort_pct < 0.0)
    coast_dv = abs(float(np.sum(base.need_accel_kmhs[cut] - aux["coast_b"][cut]))) * base.period_s
    assert dv_r < coast_dv  # 戻る途中のブレーキが効く分、失う Δv は小さい


def test_rule_peak_ignores_decelerating_head_of_accel_run() -> None:
    n = 100
    eff = np.zeros(n)
    eff[20:56] = 24.0
    eff[58:80] = -24.0
    k = np.arange(n)
    # 連なりの頭（谷の立ち上がり）は基準がまだ減速中。頂点はその後の 60 行目
    ref_acc = np.where(k < 25, -5.0, np.where(k < 60, 1.0, -5.0))
    e, _, st = _rule_gap(eff, _rule_ctx(n, ref_acc))
    s = st.rule_peak_spans[0][0]
    assert s > 30 and np.all(e[20:s] == 24.0)  # 連なりの頭（25 行目まで）を削ったりしない


def _valley_boost_setup(ref_shift=0.0):
    from tests.research.fwd_plan import GapContext, PedalTrack, predicted_accel  # noqa: PLC0415

    n = 100
    eff, need = _peak_valley(n)
    tb = linear_table()
    v = np.full(n, 50.0)
    pa, pb = _track(eff)
    ref = predicted_accel(
        tb, v, pa, pb, track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD, period_s=DT
    ) + ref_shift
    after = eff.copy()
    after[58:60] = 0.0  # c = 2 行削った
    ctx = GapContext(
        table=tb, v=v, ref_acc=ref, coast_a=np.full(n, -2.5), coast_b=np.full(n, -2.5),
        track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD, brake_lag_s=0.1, need=need,
    )
    base = types.SimpleNamespace(need_accel_kmhs=need)
    return n, eff, need, after, ctx, base, ref


def _boost_cum(f, add0, n, need, after, ctx, ref, k, s, r0, r1):
    """独立に書いた採点: M 行を表で解き直した列で両ペダルの位置を追い、r0〜r1 の面積。"""
    e = after.copy()
    u, _ = ctx.table.invert(ctx.v[k:s], need[k:s] + f * add0)
    u = np.asarray(u, dtype=float)
    e[k:s] = np.where(u < 0.0, -(BD - u), e[k:s])
    pa, pb = _track(e)
    pr = _pred(ctx.table, ctx.v, pa, pb)
    return float(np.sum((pr[r0 : r1 + 1] - ref[r0 : r1 + 1]) * DT))


def test_valley_boost_gain_balances_area_to_zero() -> None:
    from tests.research.fwd_plan import _valley_boost_gain  # noqa: PLC0415

    n, _eff, need, after, ctx, base, ref = _valley_boost_setup()
    k, s, nxt, m = 52, 58, 62, 6
    r0, r1 = s - m - 10, nxt + round(0.5 / DT)
    add0 = -2.0
    # 削ると減速が足りない
    assert _boost_cum(0.0, add0, n, need, after, ctx, ref, k, s, r0, r1) > 0.0
    f = _valley_boost_gain(base, after, ctx, k, s, nxt, m, add0, DT)
    assert 0.0 < f < 4.0
    assert abs(_boost_cum(f, add0, n, need, after, ctx, ref, k, s, r0, r1)) < 1e-3
    # 量を半分にすると倍率は倍（f × add0 が同じ）
    f2 = _valley_boost_gain(base, after, ctx, k, s, nxt, m, add0 / 2.0, DT)
    assert f2 * add0 / 2.0 == pytest.approx(f * add0, rel=0.02)


def test_valley_boost_gain_clamps_to_0_and_max() -> None:
    from tests.research.fwd_plan import VALLEY_BOOST_F_MAX, _valley_boost_gain  # noqa: PLC0415

    n, _eff, need, after, ctx, base, _ref = _valley_boost_setup()
    k, s, nxt, m = 52, 58, 62, 6
    assert _valley_boost_gain(base, after, ctx, k, s, nxt, m, -0.02, DT) == VALLEY_BOOST_F_MAX
    # 基準が強い加速 → 削っても足りている
    _, _, _, _, ctx0, base0, _ = _valley_boost_setup(ref_shift=5.0)
    assert _valley_boost_gain(base0, after, ctx0, k, s, nxt, m, -2.0, DT) == 0.0


def test_rule_valley_compensation_spreads_over_max_c_or_0_3_s_and_logs_f() -> None:
    from tests.research.fwd_plan import (  # noqa: PLC0415
        GapContext,
        PedalTrack,
        _gap_compensation,
        return_time_of,
    )

    tb = _gap_table()
    aux: dict = {}
    base = plan_for(
        _valley, 6.0, tb, return_gap_enabled=False, brake_lag_s=0.1, accel_lag_s=0.1,
        switch_gap_rule=True, _aux_out=aux,
    )
    ret = return_time_of(tb, AD, BD)
    ctx = GapContext(
        table=tb, v=aux["v"], ref_acc=aux["ref_acc"], coast_a=aux["coast_a"],
        coast_b=aux["coast_b"], track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD,
        brake_lag_s=0.1, need=base.need_accel_kmhs,
    )
    add, _dv, n_ev, trimmed = _gap_compensation(base, aux, ret, base.period_s, rule_ctx=ctx)
    assert n_ev >= 1 and len(ctx.comp_f_log) >= 1
    assert all(0.0 <= f <= 4.0 for f in ctx.comp_f_log)
    c = int(np.count_nonzero(trimmed & (base.effort_pct < 0.0)))
    rows = int(np.count_nonzero(add != 0.0))
    assert rows <= max(c, round(0.3 / base.period_s))
    assert np.all(add <= 1e-12)  # ブレーキ側なので減速を増やす向き（負）か 0


# ---- 段10-3: 案 C（switch_gap_min_peak） ----

def _peak_ctx(n, need, ref_acc, *, comp=False, window_s=2.0, coast=-2.5, v=50.0):
    from tests.research.fwd_plan import GapContext, PedalTrack  # noqa: PLC0415

    return GapContext(
        table=linear_table(), v=np.full(n, v), ref_acc=np.asarray(ref_acc, dtype=float),
        coast_a=np.full(n, coast), coast_b=np.full(n, coast), track=PedalTrack(),
        accel_min_pct=AD, brake_min_pct=BD, brake_lag_s=0.1, use_rule=False, min_peak=True,
        need=np.asarray(need, dtype=float), accel_lag_s=0.1, comp=comp, peak_window_s=window_s,
    )


def _peak_valley(n=100):
    """ブレーキ [20, 60)・アクセル [62, 80)。要る加速度はそれぞれ表どおり（−4.0・+3.0）。"""
    eff = np.zeros(n)
    eff[20:60] = -9.5
    eff[62:80] = 22.3
    need = np.where(eff < 0.0, -4.0, np.where(eff > 0.0, 3.0, -2.5))
    return eff, need


def _peak_err(e, ctx, i, window_s=2.0):
    """独立に書いた採点: 位置を追い、予測 − 基準の傾きを T − 窓から 0 で積み上げた最大の大きさ。"""
    from tests.research.fwd_plan import predicted_accel  # noqa: PLC0415

    pa, pb = _track(e)
    pred = predicted_accel(
        ctx.table, ctx.v, pa, pb, track=ctx.track, accel_min_pct=AD, brake_min_pct=BD, period_s=DT,
    )
    w = round(window_s / DT)
    sl = slice(i - w, i + w + 1)
    return float(np.max(np.abs(np.r_[0.0, np.cumsum((pred[sl] - ctx.ref_acc[sl]) * DT)])))


def test_min_peak_defaults_off_plan_is_identical() -> None:
    off = _gap_plan(_valley, True)
    dflt = _gap_plan(_valley, True, switch_gap_min_peak=False, switch_gap_peak_window_s=3.0)
    assert np.array_equal(off.effort_pct, dflt.effort_pct)
    assert off.return_gap.n_min_peak == 0 and "案 C" not in off.return_gap.summary()


def test_min_peak_choice_is_minimum_among_position_feasible_pairs() -> None:
    from tests.research.fwd_plan import position_return_row  # noqa: PLC0415

    n = 100
    eff, need = _peak_valley(n)
    ref = np.where(np.arange(n) < 60, -4.0, 2.0)
    ctx = _peak_ctx(n, need, ref)
    e = eff.copy()
    st = _gap_with(e, ctx)
    c = int(np.count_nonzero((eff < 0.0) & (e == 0.0)))
    d = int(np.count_nonzero((eff > 0.0) & (e == 0.0)))
    assert st.n_min_peak == 1 and e[20] == -9.5  # 先頭の 1 行は残る
    # すき間の条件は位置で決まる: 戻す側が min に着く行 ≤ 次のペダルの最初の行
    _, pb = _track(e)
    assert position_return_row(pb, 60 - c + 1, BD) + 1 <= 62 + d
    # 独立に数えた全部の組（条件を満たすもの）の中で、最大の大きさが最小
    best = None
    for cc in range(0, 40):
        for dd in range(0, 18):
            ee = eff.copy()
            if cc:
                ee[60 - cc : 60] = 0.0
            ee[62 : 62 + dd] = 0.0
            _, pbb = _track(ee)
            r = position_return_row(pbb, 60 - cc, BD)
            if r is None or r + 1 > 62 + dd:
                continue
            err = _peak_err(ee, ctx, 62)
            if best is None or err < best - 1e-9:
                best = err
    assert _peak_err(e, ctx, 62) == pytest.approx(best, abs=1e-6)


def _gap_with(effort, ctx, ret=None):
    from tests.research.fwd_plan import ensure_return_gap  # noqa: PLC0415

    fl = np.full(len(effort), FLAG_COAST, dtype=int)
    return ensure_return_gap(
        effort, fl, ret=ret or _ret(amin=AD, bmin=BD), period_s=DT, rule_ctx=ctx,
    )


def test_min_peak_skips_switch_when_coast_is_not_negative() -> None:
    """切り替え先の惰行の加速度が 0 以上（停車前後の低速など）の切り替えは C の対象外。"""
    n = 100
    eff, need = _peak_valley(n)
    ref = np.where(np.arange(n) < 60, -4.0, 2.0)
    e_neg = eff.copy()
    st_neg = _gap_with(e_neg, _peak_ctx(n, need, ref))  # 惰行 −2.5: C が掛かる
    assert st_neg.n_min_peak == 1
    e = eff.copy()
    st = _gap_with(e, _peak_ctx(n, need, ref, coast=0.5))  # 惰行 ≥ 0: 今までの手順
    e0 = eff.copy()
    st0 = _gap_with(e0, None)
    assert st.n_min_peak == 0 and np.array_equal(e, e0) and st.n_cut == st0.n_cut


def test_min_peak_valley_picks_a_different_pair_from_min_loss() -> None:
    from tests.research.fwd_plan import ensure_return_gap  # noqa: PLC0415

    n = 100
    eff, need = _peak_valley(n)
    ref = np.where(np.arange(n) < 60, -4.0, 2.0)
    coast = np.full(n, -2.5)
    e9 = eff.copy()
    ensure_return_gap(
        e9, np.full(n, FLAG_COAST, dtype=int), ret=_ret(amin=AD, bmin=BD), period_s=DT,
        need_accel=need, coast_a=coast, coast_b=coast,
    )
    ec = eff.copy()
    _gap_with(ec, _peak_ctx(n, need, ref))
    assert not np.array_equal(e9, ec)  # 損の最小と予測した偏差の最小は別の組


def test_min_peak_gap_ok_by_old_rule_but_position_late_is_redone() -> None:
    n = 100
    eff = np.zeros(n)
    eff[20:60] = -24.0  # 深いブレーキ（戻るのに数行）
    eff[62:80] = 24.0
    need = np.where(eff < 0.0, -12.7, np.where(eff > 0.0, 4.0, -2.5))
    fast = _ret(delay=0.0, rate=1e9, amin=AD, bmin=BD)  # 従来の見方ではすき間 2 行で足りる
    e0 = eff.copy()
    from tests.research.fwd_plan import ensure_return_gap  # noqa: PLC0415

    ensure_return_gap(e0, np.full(n, FLAG_COAST, dtype=int), ret=fast, period_s=DT)
    assert np.array_equal(e0, eff)
    e1 = eff.copy()
    st = _gap_with(e1, _peak_ctx(n, need, need.copy()), ret=fast)
    assert not np.array_equal(e1, eff) and st.n_min_peak == 1


def test_min_peak_position_already_in_time_changes_nothing() -> None:
    n = 100
    eff, need = _peak_valley(n)
    eff[62:80] = 0.0
    eff[75:80] = 22.3  # 十分遠い
    e = eff.copy()
    st = _gap_with(e, _peak_ctx(n, need, need.copy()))
    assert np.array_equal(e, eff) and st.n_min_peak == 0 and st.n_cut == 0


def test_min_peak_ignores_stop_and_prepos_rows() -> None:
    n = 100
    eff, need = _peak_valley(n)
    for row, kind in ((59, FLAG_STOP), (62, FLAG_PREPOS)):
        fl = np.full(n, FLAG_COAST, dtype=int)
        fl[row] = kind
        e0, e1 = eff.copy(), eff.copy()
        from tests.research.fwd_plan import ensure_return_gap  # noqa: PLC0415

        ensure_return_gap(e0, fl.copy(), ret=_ret(amin=AD, bmin=BD), period_s=DT)
        st = ensure_return_gap(
            e1, fl.copy(), ret=_ret(amin=AD, bmin=BD), period_s=DT,
            rule_ctx=_peak_ctx(n, need, need.copy()),
        )
        assert np.array_equal(e0, e1) and st.n_min_peak == 0


def test_min_peak_on_plan_runs_and_counts() -> None:
    on = _gap_plan(_valley, True, switch_gap_min_peak=True)
    assert np.all(np.isfinite(on.effort_pct))
    assert on.return_gap.n_min_peak >= 1
    assert "案 C" in on.return_gap.summary()
    assert abs(on.effort_pct).max() <= 100.0


def test_gap_violations_by_position_matches_rule_and_old_count_unchanged() -> None:
    """段10-6: rule_ctx があるとき n_violations は位置の物差し（R の置いた切り替えは 0）。
    rule_ctx が無ければ今までの数え方のまま。"""
    from tests.research.fwd_plan import count_gap_violations  # noqa: PLC0415

    n = 100
    eff = np.zeros(n)
    eff[20:56] = 24.0
    eff[58:80] = -24.0
    ref_acc = np.where(np.arange(n) < 60, 1.0, -5.0)
    e, _, st = _rule_gap(eff, _rule_ctx(n, ref_acc))
    assert st.n_rule_peak == 1 and st.by_position and st.n_violations == 0
    assert "位置で数えた" in st.summary()
    # 旧い数え方（指令の最後の値の戻る時間）では、R が置いた切り替えは不足と数える
    assert count_gap_violations(e, ret=_ret(amin=AD, bmin=BD), period_s=DT) > 0
    # rule_ctx なし（off）は今のまま
    e2, _, st2 = _gap([24.0] * 6 + [-20.0] * 6)
    assert not st2.by_position and st2.n_violations == 0
    assert "位置で数えた" not in st2.summary()


# ---- 段11-5: R のまま／後ろで取る を予測の偏差で選ぶ（switch_gap_choose） ----

def _choose_valley_case():
    """ブレーキ [20,60) → アクセル [64,110) の谷。基準を「後ろで取る」列の予測、または R の列の
    予測にそろえ、選び直しが点数の小さい方を選ぶことを見る。"""
    from tests.research.fwd_plan import position_return_row  # noqa: PLC0415

    n = 140
    eff = np.zeros(n)
    eff[20:60] = -24.0
    eff[64:110] = 15.0
    tb = _rule_ctx(n, np.zeros(n)).table
    _, pb = _track(eff)
    r = position_return_row(pb, 60, BD)
    eb = eff.copy()
    eb[64 : r + 1] = 0.0  # 後ろで取る: ブレーキは要る間踏み、アクセルは r + 1 から
    pa_, pb_ = _track(eb)
    ref_back = _pred(tb, np.full(n, 50.0), pa_, pb_)
    e_r, _, _ = _rule_gap(eff, _rule_ctx(n, ref_back))  # 今までの R
    pa_, pb_ = _track(e_r)
    ref_r = _pred(tb, np.full(n, 50.0), pa_, pb_)
    return n, eff, r, eb, e_r, ref_back, ref_r


def test_choose_valley_picks_back_when_it_tracks_the_reference() -> None:
    n, eff, r, eb, e_r, ref_back, _ = _choose_valley_case()
    assert not np.array_equal(e_r, eb)  # 前提: R は後ろで取る列と違う
    ctx = _rule_ctx(n, ref_back)
    ctx.choose = True
    e, fl, st = _rule_gap(eff, ctx)
    assert st.n_choose_back_valley == 1 and st.n_choose_back_peak == 0
    i, kind, score_r, score_b, choice = st.choose_log[0]
    assert (i, kind, choice) == (64, "valley", "back") and score_b < score_r
    assert np.array_equal(e, eb) and np.all(fl[64 : r + 1] == FLAG_COAST)
    from tests.research.fwd_plan import ReturnGapStats, count_gap_violations_by_position  # noqa

    assert count_gap_violations_by_position(e, ctx=ctx, period_s=DT) == 0
    assert isinstance(st, ReturnGapStats) and "後ろを選んだ" in st.summary()


def test_choose_keeps_r_when_r_tracks_better_and_on_tie() -> None:
    n, eff, _, _, e_r, _, ref_r = _choose_valley_case()
    ctx = _rule_ctx(n, ref_r)
    ctx.choose = True
    e, _, st = _rule_gap(eff, ctx)
    assert np.array_equal(e, e_r) and st.n_choose_back_valley == 0
    assert st.choose_log[0][4] == "R" and st.choose_log[0][2] <= st.choose_log[0][3]
    # 同点（R と後ろで取るが同じ列になる＝すき間が足りていて R が削らない）は R
    eff2 = np.zeros(n)
    eff2[20:60] = -24.0
    eff2[90:110] = 15.0
    ctx2 = _rule_ctx(n, np.zeros(n))
    ctx2.choose = True
    e2, _, st2 = _rule_gap(eff2, ctx2)
    assert np.array_equal(e2, eff2) and st2.n_choose_back_valley == 0


def test_choose_off_is_identical_in_ensure_and_build_plan() -> None:
    n, eff, _, _, _, ref_back, _ = _choose_valley_case()
    e0, f0, s0 = _rule_gap(eff, _rule_ctx(n, ref_back))
    assert _rule_ctx(n, ref_back).choose is False
    assert s0.choose_log == [] and s0.n_choose_back_valley == 0
    off = _gap_plan(_hill, True, switch_gap_rule=True)
    dflt = _gap_plan(_hill, True, switch_gap_rule=True, switch_gap_choose=False)
    assert np.array_equal(off.effort_pct, dflt.effort_pct)
    on = _gap_plan(_hill, True, switch_gap_rule=True, switch_gap_choose=True)
    assert on.return_gap.n_violations == 0 and np.all(np.isfinite(on.effort_pct))
    assert len(on.return_gap.choose_log) >= 1
    # R が false なら choose は効かない
    no_r = _gap_plan(_hill, True, switch_gap_choose=True)
    assert np.array_equal(no_r.effort_pct, _gap_plan(_hill, True).effort_pct)


def test_choose_back_rows_removes_valley_boost_from_brake_rows() -> None:
    """谷の戻す側はブレーキ（負）。後ろで取る列は 7-c の上乗せ行を上乗せなしの値へ戻す。"""
    from tests.research.fwd_plan import ReturnGapStats, _choose_back_rows  # noqa: PLC0415

    n = 140
    alt = np.zeros(n)
    alt[20:60] = -15.0
    alt[64:110] = 15.0
    boosted = alt.copy()
    boosted[50:58] = -21.3  # 上乗せ（深くなっている）
    ctx = _rule_ctx(n, np.zeros(n))
    ctx.alt_effort = alt
    e = boosted.copy()
    e[58:60] = 0.0  # R が削った行
    f = np.full(n, FLAG_COAST, dtype=int)
    rec = ("valley", 20, 59, 64, 110, 58, 64)
    _choose_back_rows(e, f, 0, rec, 66, boosted.copy(), f.copy(), ctx)
    assert np.all(e[50:58] == -15.0) and np.all(e[58:60] == boosted[58:60])
    assert np.all(e[64:66] == 0.0) and ReturnGapStats()


# ---- 段12: R の山の採点の頑健化（switch_gap_peak_robust） ----

def _robust_ctx(n, ref_acc, on):
    ctx = _rule_ctx(n, ref_acc)
    ctx.peak_robust = on
    return ctx


def _robust_case():
    n = 400
    eff = np.zeros(n)
    eff[20:300] = 24.0
    eff[302:340] = -24.0
    idx = np.arange(n)
    ref_acc = np.where(idx <= 270, 1.0, 1.0 - 0.1 * (idx - 270))
    return n, eff, ref_acc


def test_rule_peak_robust_off_is_unchanged() -> None:
    """段12: 既定（False）は今までと同じ計画。"""
    n, eff, ref_acc = _robust_case()
    assert _rule_ctx(n, ref_acc).peak_robust is False
    e0, _, st0 = _rule_gap(eff, _rule_ctx(n, ref_acc))
    e1, _, st1 = _rule_gap(eff, _robust_ctx(n, ref_acc, False))
    assert np.array_equal(e0, e1) and st0.rule_peak_spans == st1.rule_peak_spans


def test_rule_peak_robust_on_returns_candidate_inside_range() -> None:
    """段12: True でも動き、戻し始めは last 以下の候補の行、ブレーキは b 以降から。"""
    n, eff, ref_acc = _robust_case()
    e, _, st = _rule_gap(eff, _robust_ctx(n, ref_acc, True))
    assert st.n_rule_peak == 1
    s, end = st.rule_peak_spans[0]
    assert 21 <= s <= 300 and end == 300
    assert np.all(e[s:300] == 0.0)


def test_rule_peak_robust_skips_level_offset_before_return() -> None:
    """段12 案 Y: 戻す前の加速中だけ予測が基準より速い（一定の片寄り）とき、True の戻し始めは
    False より遅い（同じを含む）。片寄りを積み上げに数えないので早めに戻して相殺しない。"""
    n = 400
    eff = np.zeros(n)
    eff[20:300] = 24.0
    eff[302:340] = -24.0
    idx = np.arange(n)
    # 基準は加速中ずっと低め（予測との差が一定）、頂点 306 で惰行を下回る
    ref_acc = np.where(idx <= 270, -1.0, -1.0 - 0.1 * (idx - 270))
    _, _, st0 = _rule_gap(eff, _robust_ctx(n, ref_acc, False))
    _, _, st1 = _rule_gap(eff, _robust_ctx(n, ref_acc, True))
    assert st0.rule_peak_spans and st1.rule_peak_spans
    assert st1.rule_peak_spans[0][0] >= st0.rule_peak_spans[0][0]


# ---- 段12 谷: R の谷の上乗せの頑健化（switch_gap_valley_robust） ----

def test_valley_robust_default_false() -> None:
    """段12 谷: 既定（False）。False のとき上乗せ倍率は今の二分法 `_valley_boost_gain` のまま。"""
    from tests.research.fwd_plan import GapContext  # noqa: PLC0415

    _, _, _, _, ctx, _, _ = _valley_boost_setup()
    assert ctx.valley_robust is False
    assert GapContext.__dataclass_fields__["valley_robust"].default is False


def test_valley_boost_gain_robust_inside_range() -> None:
    """段12 谷: True の f は 0〜上限に入る。"""
    from tests.research.fwd_plan import (  # noqa: PLC0415
        VALLEY_BOOST_F_MAX,
        _valley_boost_gain_robust,
    )

    n, _eff, need, after, ctx, base, _ref = _valley_boost_setup()
    k, s, nxt, m = 56, 58, 62, 4
    for add0 in (-2.0, -0.02, 0.0, 0.5):
        f = _valley_boost_gain_robust(base, after, ctx, k, s, nxt, m, add0, DT)
        assert 0.0 <= f <= VALLEY_BOOST_F_MAX


def test_valley_boost_gain_robust_not_worse_than_grid_min() -> None:
    """段12 谷: 黄金分割で詰めた f の点数は、0.1 刻みの格子の最小以下。"""
    from dataclasses import replace  # noqa: PLC0415

    from tests.research.fwd_plan import (  # noqa: PLC0415
        PEAK_ROBUST_LAG_ROWS,
        VALLEY_BOOST_F_MAX,
        VALLEY_ROBUST_GRID_F,
        PedalTrack,
        _valley_boost_gain_robust,
        predicted_accel,
    )

    n, _eff, need, after, ctx, base, ref = _valley_boost_setup()
    k, s, nxt, m, add0 = 56, 58, 62, 4, -0.3
    f = _valley_boost_gain_robust(base, after, ctx, k, s, nxt, m, add0, DT)

    def score(ff: float) -> float:  # 独立に書いた採点（局所でなく全体の列。w0 ＝ k−1 から r1 まで）
        e = after.copy()
        u, _ = ctx.table.invert(ctx.v[k:s], need[k:s] + ff * add0)
        u = np.asarray(u, dtype=float)
        e[k:s] = np.where(u < 0.0, -(BD - u), e[k:s])
        pa, pb = _track(e)
        worst = 0.0
        for dr in (-PEAK_ROBUST_LAG_ROWS, 0, PEAK_ROBUST_LAG_ROWS):
            tr = replace(
                PedalTrack(),
                accel_effect_lag_s=max(PedalTrack().accel_effect_lag_s + dr * DT, 0.0),
                brake_effect_lag_s=max(PedalTrack().brake_effect_lag_s + dr * DT, 0.0),
            )
            pr = predicted_accel(
                ctx.table, ctx.v, pa, pb, track=tr, accel_min_pct=AD, brake_min_pct=BD, period_s=DT
            )
            r1 = min(nxt + round(0.5 / DT), n - 1)
            c = np.cumsum((pr[k - 1 : r1 + 1] - ref[k - 1 : r1 + 1]) * DT)
            worst = max(worst, float(np.max(np.abs(c))))
        return worst

    grid = np.arange(0.0, VALLEY_BOOST_F_MAX + 1e-9, VALLEY_ROBUST_GRID_F)
    assert score(f) <= min(score(float(g)) for g in grid) + 1e-6


# ───────── 2026-10-27 段13: 低速の予測の表の側・発進のずらし ─────────

_L13 = {"brake_lag_s": 0.4, "accel_lag_s": 0.2}
_K13 = dict(
    low_speed_side_rule="dir", launch_release_ramp=True, return_gap_enabled=True,
    switch_gap_rule=True, **_L13,
)


def _table13() -> ForwardTable:
    return _pedal_table(breakaway=-1.0)


def test_predicted_accel_side_flags_change_only_flagged_rows() -> None:
    from tests.research.fwd_plan import PedalTrack, predicted_accel  # noqa: PLC0415

    tb = _table13()
    n = 20
    v = np.full(n, 2.0)
    pa = np.full(n, AD)
    pb = np.full(n, BD)
    kw = dict(track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD, period_s=DT)
    base = predicted_accel(tb, v, pa, pb, **kw)
    none = predicted_accel(tb, v, pa, pb, launch_a=None, stop_b=None, **kw)
    assert np.array_equal(base, none)  # 既定は今と完全に同じ
    mask = np.zeros(n, dtype=bool)
    mask[5:9] = True
    la = predicted_accel(tb, v, pa, pb, launch_b=mask, **kw)
    assert np.array_equal(la[~mask], base[~mask])
    assert np.all(la[mask] > base[mask] + 1.0)  # 発進側の惰行は +2.0 持ち上がっている
    st = predicted_accel(tb, v, pa, pb, stop_b=mask, **kw)
    assert np.array_equal(st[~mask], base[~mask]) and np.all(st[mask] > la[mask])


def test_predict_low_speed_sides_off_is_identical_and_legacy_unaffected() -> None:
    ref = _stop_then_go(8.0)
    kw = dict(_K13)
    off = plan_for(ref, 14.0, _table13(), **kw)
    off2 = plan_for(ref, 14.0, _table13(), predict_low_speed_sides=False, **kw)
    assert np.array_equal(off.effort_pct, off2.effort_pct)
    assert off.launch_shift is None
    kw_leg = dict(kw, low_speed_side_rule="legacy")  # 側が決まらない規則は今と同じ
    a = plan_for(ref, 14.0, _table13(), **kw_leg)
    b = plan_for(ref, 14.0, _table13(), predict_low_speed_sides=True, **kw_leg)
    assert np.array_equal(a.effort_pct, b.effort_pct)
    on = plan_for(ref, 14.0, _table13(), predict_low_speed_sides=True, **kw)
    assert on.effort_pct.shape == off.effort_pct.shape and on.side_rule == "dir"


def test_gap_context_side_kw_slices_rows() -> None:
    from tests.research.fwd_plan import GapContext, PedalTrack  # noqa: PLC0415

    arrs = tuple(np.arange(10) % (k + 2) == 0 for k in range(4))
    ctx = GapContext(
        table=_table13(), v=np.zeros(10), ref_acc=np.zeros(10), coast_a=np.zeros(10),
        coast_b=np.zeros(10), track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD,
        brake_lag_s=0.4, lp_sides=arrs,
    )
    kw = ctx.side_kw(2, 7)
    assert set(kw) == {"launch_a", "stop_a", "launch_b", "stop_b"}
    assert all(len(x) == 5 for x in kw.values())
    assert np.array_equal(kw["stop_a"], arrs[1][2:7])
    assert GapContext(
        table=_table13(), v=np.zeros(3), ref_acc=np.zeros(3), coast_a=np.zeros(3),
        coast_b=np.zeros(3), track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD,
        brake_lag_s=0.4,
    ).side_kw() == {}


def test_shift_back_moves_window_and_fills_front_with_previous_value() -> None:
    from tests.research.fwd_plan import _shift_back  # noqa: PLC0415

    arr = np.arange(10, dtype=float)
    out = _shift_back(arr, 3, 8, 2)
    assert out.tolist() == [0, 1, 2, 2, 2, 3, 4, 5, 6, 9]
    assert _shift_back(arr, 3, 8, 0).tolist() == arr.tolist()
    assert _shift_back(None, 3, 8, 2) is None


def test_launch_shift_off_is_identical_and_on_picks_delta_in_range_without_double_press() -> None:
    from tests.research.fwd_plan import (  # noqa: PLC0415
        LAUNCH_SHIFT_MAX_ROWS,
        PedalTrack,
        track_pedal_position,
    )

    ref = _stop_then_go(8.0)
    kw = dict(_K13, predict_low_speed_sides=True)
    base = plan_for(ref, 16.0, _table13(), **kw)
    off = plan_for(ref, 16.0, _table13(), launch_shift_choose=False, **kw)
    assert np.array_equal(base.effort_pct, off.effort_pct) and base.launch_shift is None
    on = plan_for(ref, 16.0, _table13(), launch_shift_choose=True, **kw)
    ls = on.launch_shift
    assert ls is not None and ls.n_launch == 1 and len(ls.deltas) == 1
    assert all(0 <= d <= LAUNCH_SHIFT_MAX_ROWS for d in ls.deltas)
    assert "発進 1 回" in ls.summary()
    # 選んだ δ の採点は δ=0 以下（同点は δ=0）
    assert ls.scores[0][1] <= ls.scores[0][0] + 1e-9
    for arr_on, arr_off in (
        (on.flag, base.flag), (on.side_accel, base.side_accel), (on.side_brake, base.side_brake),
    ):
        assert arr_on.shape == arr_off.shape
    assert np.array_equal(on.need_accel_kmhs, base.need_accel_kmhs)  # 基準から決まる列は動かさない

    def double_press(e: np.ndarray) -> int:
        pa, pb = track_pedal_position(
            e, period_s=on.period_s, track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD
        )
        return int(np.count_nonzero((pa > AD + 0.05) & (pb > BD + 0.05)))

    assert double_press(on.effort_pct) <= double_press(base.effort_pct)


def _early_accel_plan(lead_rows: int):
    """ブレーキ保持の後、基準の動き出し t0 の lead_rows 行前にアクセルを踏み始める手組みの計画。"""
    n = 400
    tb = linear_table(coast_at_zero=0.0)  # クリープなし。アクセルが早いとそのぶん車が先に動く
    eff = np.zeros(n)
    flag = np.full(n, FLAG_MEASURED)
    t0 = int(round(8.0 / DT)) + 1
    a0 = t0 - lead_rows
    eff[:a0], flag[:a0] = -STOP_PCT, FLAG_STOP
    eff[a0 : t0 + 100] = AD + 6.0
    eff[t0 + 100 :] = AD + 3.0
    plan = ForwardPlan(
        t_s=np.arange(n) * DT, effort_pct=eff, need_accel_kmhs=np.zeros(n), flag=flag, period_s=DT
    )
    return plan, tb, a0


def test_launch_shift_delays_early_accel_and_keeps_one_pedal_at_a_time() -> None:
    from tests.research.fwd_plan import (  # noqa: PLC0415
        PedalTrack,
        apply_launch_shift,
        track_pedal_position,
    )

    def ref(x: float) -> float:
        return 0.0 if x <= 8.0 else min(2.0 * (x - 8.0), 20.0)

    for lead, expect_shift in ((0, False), (8, True), (12, True)):
        plan, tb, a0 = _early_accel_plan(lead)
        out = apply_launch_shift(
            plan, ref, tb, stop_ref_at=None, accel_min_pct=AD, brake_min_pct=BD,
            track=PedalTrack(),
        )
        d = out.launch_shift.deltas[0]
        assert 0 <= d <= 8
        assert (d > 0) == expect_shift
        assert out.launch_shift.scores[0][1] <= out.launch_shift.scores[0][0] + 1e-9
        first_acc = int(np.flatnonzero(out.effort_pct > 0.0)[0])
        assert first_acc == a0 + d  # アクセルの立ち上がりが δ 行後ろへ移る
        assert np.all(out.effort_pct[a0 : a0 + d] == -STOP_PCT)  # 前の空いた行はブレーキで埋める
        assert out.flag[a0 : a0 + d].tolist() == [FLAG_STOP] * d  # flag も同じに移る
        # 手組みの計画はブレーキ → アクセルが直接つながり（すき間なし）元から同時踏みがある。
        # ずらしで増えてはならない（遅らせるほど減る）
        dp = []
        for p in (plan, out):
            pa, pb = track_pedal_position(
                p.effort_pct, period_s=DT, track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD
            )
            dp.append(int(np.count_nonzero((pa > AD + 0.05) & (pb > BD + 0.05))))
        assert dp[1] <= dp[0]


def test_predicted_accel_creep_curve_only_both_min_and_below_last_speed() -> None:
    # 2026-10-28 段14 L1: 両ペダル min・車速が曲線の最後未満の行だけクリープの曲線で引く
    from tests.research.fwd_plan import PedalTrack, _creep_of, predicted_accel  # noqa: PLC0415

    tb = _table13()
    n = 6
    v = np.array([0.0, 0.5, 1.5, 4.0, 5.0, 8.0])
    kw = dict(track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD, period_s=DT)
    creep = _creep_of(True, [0.5, 1.5, 2.5, 3.5, 5.0], [2.22, 1.72, 1.4, 0.91, 0.0])
    assert _creep_of(False, [0.5], [2.0]) is None and _creep_of(True, [], []) is None
    pa, pb = np.full(n, AD), np.full(n, BD)
    base = predicted_accel(tb, v, pa, pb, **kw)
    assert np.array_equal(base, predicted_accel(tb, v, pa, pb, creep=None, **kw))
    on = predicted_accel(tb, v, pa, pb, creep=creep, **kw)
    assert on[0] == pytest.approx(2.22) and on[1] == pytest.approx(2.22)  # 0.5 未満は先頭の値
    assert on[2] == pytest.approx(1.72)
    assert on[3] == pytest.approx(np.interp(4.0, creep[0], creep[1]))
    assert np.array_equal(on[4:], base[4:])  # 5.0 km/h 以上は今まで通り
    # 片方でも min 超なら今まで通り
    on_a = predicted_accel(tb, v, np.full(n, AD + 5.0), pb, creep=creep, **kw)
    assert np.array_equal(on_a, predicted_accel(tb, v, np.full(n, AD + 5.0), pb, **kw))
    on_b = predicted_accel(tb, v, pa, np.full(n, BD + 5.0), creep=creep, **kw)
    assert np.array_equal(on_b, predicted_accel(tb, v, pa, np.full(n, BD + 5.0), **kw))


def test_predict_creep_curve_off_identical_and_gap_context_passes_curve() -> None:
    from tests.research.fwd_plan import GapContext, PedalTrack  # noqa: PLC0415

    ref = _stop_then_go(8.0)
    kw = dict(_K13, predict_low_speed_sides=True, launch_shift_choose=True)
    base = plan_for(ref, 16.0, _table13(), **kw)
    # off ならカーブを渡しても今までと同じ
    off = plan_for(
        ref, 16.0, _table13(), predict_creep_curve=False,
        creep_accel_speeds_kmh=[0.5, 5.0], creep_accel_kmhs=[2.2, 0.0], **kw,
    )
    assert np.array_equal(base.effort_pct, off.effort_pct)
    on = plan_for(
        ref, 16.0, _table13(), predict_creep_curve=True,
        creep_accel_speeds_kmh=[0.5, 5.0], creep_accel_kmhs=[2.2, 0.0], **kw,
    )
    assert on.launch_shift is not None and on.effort_pct.shape == base.effort_pct.shape
    ctx = GapContext(
        table=_table13(), v=np.zeros(4), ref_acc=np.zeros(4), coast_a=np.zeros(4),
        coast_b=np.zeros(4), track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD,
        brake_lag_s=0.4, creep_curve=(np.array([0.5, 5.0]), np.array([2.2, 0.0])),
    )
    assert set(ctx.side_kw()) == {"creep"}


# ---- 2026-10-30 段1 案 B: coast_band_release_at_zero ----------------------------------------


def _band_ref(t: float) -> float:
    """区間ごとの傾き（惰行の加速度 −2 に対する差 = 傾き + 2）: 0〜2 s は +2.5（アクセル）、
    2〜4 s は +0.1（帯の内側）、4〜6 s は −0.1（帯の内側で符号が変わる）、6〜8 s は −0.6
    （ブレーキ）、8〜10 s は −0.2（帯の内側）。"""
    knots = [(0.0, 2.5), (2.0, -1.9), (4.0, -2.1), (6.0, -2.6), (8.0, -2.2)]
    v, t0, s = 50.0, 0.0, knots[0][1]
    for k, sl in knots:
        if t <= k:
            break
        v += s * (k - t0)
        t0, s = k, sl
    return v + s * (t - t0)


def _band_plan(**kw):
    table = linear_table(coast_at_zero=-2.0, v_coef=0.0)
    return plan_for(
        _band_ref, 10.0, table, accel_lag_s=0.0, brake_lag_s=0.0, need_window_s=0.1, **kw
    )


def test_band_release_default_false_is_identical() -> None:
    a = _band_plan()
    b = _band_plan(coast_band_release_at_zero=False)
    assert np.array_equal(a.effort_pct, b.effort_pct) and np.array_equal(a.flag, b.flag)
    assert a.effort_pct[idx(a, 3.0)] == 0.0  # 帯の内側はふつうの惰行


def test_band_release_keeps_pedal_inside_band_and_releases_on_sign_change() -> None:
    off = _band_plan()
    on = _band_plan(coast_band_release_at_zero=True)
    # 入る行は同じ（アクセル 1 s・ブレーキ 7 s）
    assert on.effort_pct[idx(on, 1.0)] > 0.0 and off.effort_pct[idx(off, 1.0)] > 0.0
    assert on.effort_pct[idx(on, 7.0)] < 0.0 and off.effort_pct[idx(off, 7.0)] < 0.0
    # 帯の内側（差 +0.1）でも前がアクセルなら続く。開度は表を逆に引いた値
    assert off.effort_pct[idx(off, 3.0)] == 0.0 and off.flag[idx(off, 3.0)] == FLAG_COAST
    assert on.effort_pct[idx(on, 3.0)] == pytest.approx(AD + 0.1 / ACCEL_SLOPE)
    # 差の符号が変わると離す（差 −0.1 は反対の帯の内側＝惰行）
    assert on.effort_pct[idx(on, 5.0)] == 0.0 and on.flag[idx(on, 5.0)] == FLAG_COAST
    # ブレーキも帯の内側（差 −0.2）で続く
    assert off.effort_pct[idx(off, 9.0)] == 0.0
    assert on.effort_pct[idx(on, 9.0)] == pytest.approx(-(BD + 0.2 / BRAKE_SLOPE))


def test_band_hysteresis_pure_function() -> None:
    from tests.research.fwd_plan import _band_hysteresis

    ok = np.ones(6, dtype=bool)
    no_reset = np.zeros(6, dtype=bool)
    da = np.array([0.5, 0.1, -0.1, 0.1, -0.6, -0.1])
    db = da.copy()
    a, b = _band_hysteresis(da, db, ok, ok, 0.25, no_reset)
    assert a.tolist() == [True, True, False, False, False, False]
    assert b.tolist() == [False, False, False, False, True, True]
    # u が踏める向きでなくなったら続かない
    ok_a = np.array([True, False, True, True, True, True])
    a, _ = _band_hysteresis(da, db, ok_a, ok, 0.25, no_reset)
    assert a.tolist() == [True, False, False, False, False, False]
    # 反対の帯を越えていれば、離した行からすぐ反対のペダル
    a, b = _band_hysteresis(
        np.array([0.5, -0.5]), np.array([0.5, -0.5]), ok[:2], ok[:2], 0.25, no_reset[:2]
    )
    assert a.tolist() == [True, False] and b.tolist() == [False, True]


def test_band_hysteresis_resets_on_stop_rows() -> None:
    from tests.research.fwd_plan import _band_hysteresis

    ok = np.ones(4, dtype=bool)
    d = np.array([0.5, 0.1, 0.1, 0.1])
    reset = np.array([False, True, False, False])
    a, _ = _band_hysteresis(d, d, ok, ok, 0.25, reset)
    # 行 1 は停車保持（後で上書き）。その後は「前のペダルなし」なので帯の内側は惰行
    assert a.tolist() == [True, True, False, False]
    a2, _ = _band_hysteresis(d, d, ok, ok, 0.25, np.zeros(4, dtype=bool))
    assert a2.tolist() == [True, True, True, True]
