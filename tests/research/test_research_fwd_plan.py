"""fwd_plan（基準車速 → 開度の列）のテスト。合成の表・基準だけを使う。"""

from __future__ import annotations

import math

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
