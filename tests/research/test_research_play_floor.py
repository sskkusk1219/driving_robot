"""車速ごとの遊びの下限（play_floor）・C12 の handover・先読みの P のテスト。

2026-11-08 ProblemReport_20261108_アクセルを効き始めで待つ 段1〜3。
合成の表（アクセルの遊び 6.5%・ブレーキの遊び 2%）を使う。src/ は import しない。
"""

from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import mode_drive as mdmod
from tests.research.c10_integral import C10Integral
from tests.research.coast_assist import CoastAssist
from tests.research.dob import PEDAL_ACCEL, PEDAL_BRAKE, PedalDisturbanceObserver
from tests.research.fwd_map import POINT_MEASURED, ForwardTable, save_table
from tests.research.fwd_plan import (
    FLAG_COAST,
    FLAG_MEASURED,
    FLAG_PREPOS,
    coast_assist_effort_detail,
)
from tests.research.play_floor import (
    PlayFloor,
    apply_floor,
    onset_u_pct,
    play_floor_pct,
)
from tests.research.test_research_mode_drive import _mode, _tmp_cfg

ACCEL_PLAY, BRAKE_PLAY = 6.5, 2.0
KA, KB = 0.2, 0.5  # 遊びの先の効き [km/h/s per %]
AMIN, BMIN = 4.0, 7.0
MARGIN = 0.5
RISE = 0.05


def _table(*, flat: bool = False, meta: dict[str, float] | None = None) -> ForwardTable:
    """a(v, u) = −2 − 0.01 v ＋（アクセル: KA·max(0, u − 6.5)／ブレーキ: −KB·max(0, −u − 2)）。
    flat=True は遊びの先も効かない表（効き始めが求まらない）。"""
    rep = np.array([0.0, 50.0, 100.0, 150.0])
    grid = np.arange(-30.0, 30.5, 0.5)
    if flat:
        eff = np.zeros_like(grid)
    else:
        eff = np.where(
            grid >= 0.0, KA * np.maximum(0.0, grid - ACCEL_PLAY),
            -KB * np.maximum(0.0, -grid - BRAKE_PLAY),
        )
    accel = (-2.0 - 0.01 * rep[:, None]) + eff[None, :]
    m = {
        "accel_deadband_pct": AMIN, "brake_deadband_pct": BMIN,
        "accel_lag_s": 0.05, "brake_lag_s": 0.05,
        "stop_max_speed_kmh": 7.5,
    }
    m.update(meta or {})
    return ForwardTable(
        speed_edges=np.array([-25.0, 25.0, 75.0, 125.0, 175.0]),
        speed_rep=rep, u_grid=grid, accel_grid=accel,
        seconds=np.full(accel.shape, 10.0),
        point_kind=np.full(accel.shape, POINT_MEASURED, dtype=int),
        meas_u_min=np.full(4, -30.0), meas_u_max=np.full(4, 30.0),
        kind="model", meta=m,
    )


# ─── 段1: 下限の位置 ────────────────────────────────────────────────


def test_onset_is_play_edge_plus_rise_over_slope() -> None:
    t = _table()
    # アクセル: 6.5 + 0.05/0.2 = 6.75、ブレーキ: 2 + 0.05/0.5 = 2.1（格子の間は直線）
    assert onset_u_pct(t, 60.0, is_accel=True, rise_kmhs=RISE) == pytest.approx(6.75)
    assert onset_u_pct(t, 60.0, is_accel=False, rise_kmhs=RISE) == pytest.approx(2.1)
    # しきい値が大きいほど深い
    assert onset_u_pct(t, 60.0, is_accel=True, rise_kmhs=0.2) == pytest.approx(7.5)


def test_floor_is_min_plus_onset_minus_margin() -> None:
    t = _table()
    a = play_floor_pct(
        t, 84.0, is_accel=True, rise_kmhs=RISE, margin_pct=MARGIN, min_pct=AMIN
    )
    b = play_floor_pct(
        t, 84.0, is_accel=False, rise_kmhs=RISE, margin_pct=MARGIN, min_pct=BMIN
    )
    assert a == pytest.approx(AMIN + 6.75 - MARGIN)
    assert b == pytest.approx(BMIN + 2.1 - MARGIN)
    # 余白が効き始めより大きいときは min（それより浅くはならない）
    c = play_floor_pct(t, 84.0, is_accel=False, rise_kmhs=RISE, margin_pct=5.0, min_pct=BMIN)
    assert c == pytest.approx(BMIN)


def test_floor_falls_back_to_pkl_onset_then_none() -> None:
    flat = _table(flat=True)
    assert onset_u_pct(flat, 60.0, is_accel=True, rise_kmhs=RISE) is None
    got = play_floor_pct(
        flat, 60.0, is_accel=True, rise_kmhs=RISE, margin_pct=MARGIN, min_pct=AMIN,
        fallback_u_pct=8.75,
    )
    assert got == pytest.approx(AMIN + 8.75 - MARGIN)
    assert play_floor_pct(
        flat, 60.0, is_accel=True, rise_kmhs=RISE, margin_pct=MARGIN, min_pct=AMIN
    ) is None


def test_play_floor_class_caches_per_bin_and_signs_brake_negative() -> None:
    t = _table(meta={"accel_onset_u_pct": 99.0, "brake_onset_u_pct": 99.0})
    pf = PlayFloor(t, rise_kmhs=RISE, margin_pct=MARGIN, accel_min_pct=AMIN, brake_min_pct=BMIN)
    fa = pf.floor_effort(60.1, is_accel=True)
    fb = pf.floor_effort(60.1, is_accel=False)
    assert fa == pytest.approx(AMIN + 6.75 - MARGIN) and fb == pytest.approx(-(BMIN + 2.1 - MARGIN))
    n = len(pf._cache)  # noqa: SLF001
    assert pf.floor_effort(60.3, is_accel=True) == fa  # 同じ 0.5 km/h の刻みはキャッシュ
    assert len(pf._cache) == n  # noqa: SLF001


def test_apply_floor_never_deeper_than_ff_and_keeps_shallow_ff() -> None:
    # FF が下限以上: C10 が下限を割ったら下限で止める（FF より深くはならない）
    total, floored = apply_floor(15.0, -8.0, 10.25)
    assert floored and total == pytest.approx(10.25) and abs(total) <= 15.0
    total, floored = apply_floor(-20.0, 14.0, -8.6)  # ブレーキ（負）
    assert floored and total == pytest.approx(-8.6)
    # 下限以上なら何もしない
    assert apply_floor(15.0, -2.0, 10.25) == (13.0, False)
    # FF が下限より浅い行は触らない（計画の判断は変えない）
    assert apply_floor(8.0, -3.0, 10.25) == (5.0, False)
    # force（handover の行）は FF の深さに関係なく下限を掛ける
    assert apply_floor(4.0, 0.0, 10.25, force=True) == (pytest.approx(10.25), True)
    # 下限なし・FF ＝ 0 は何もしない
    assert apply_floor(15.0, -8.0, None) == (7.0, False)
    assert apply_floor(0.0, 3.0, 10.25) == (3.0, False)


# ─── モード走行の中（_play_floor_row / _coast_handover） ───────────────────


def _cfg(
    tmp_path: Path, *, pred_p: bool = False, play_floor: bool = True,
    fm: dict[str, object] | None = None,
) -> cfgmod.ResearchConfig:
    cfg = _tmp_cfg(tmp_path)
    # 本体 yaml の設定（保持の場面・発進のブレーキの帯など、合成 pkl に無い meta を要るもの）に
    # 依存しないよう、forward_map はコードの既定値から始める
    cfg.forward_map = cfgmod.ForwardMapSection()
    cfg.feedforward.candidate = "C12"
    cfg.vehicle.min_accel_opening_pct = AMIN
    cfg.vehicle.min_brake_opening_pct = BMIN
    cfg.feedforward.model_path = str(tmp_path / "fwd_table.pkl")
    cfg.forward_map.deviation_c = 20.0  # τ ＝ 20 × 0.05 s ＝ 1 s
    cfg.forward_map.dob_tq_coef = 40.0
    cfg.forward_map.preposition_margin_pct = MARGIN
    cfg.forward_map.play_floor_rise_kmhs = RISE
    cfg.forward_map.c10_play_floor = play_floor
    cfg.forward_map.coast_on_kmh = 0.2
    cfg.forward_map.coast_off_kmh = 0.1
    cfg.forward_map.coast_assist_handover = True
    cfg.forward_map.coast_assist_pred_p = pred_p
    cfg.forward_map.c10_integral_accel = True
    cfg.forward_map.c10_integral_brake = True
    cfg.forward_map.c10_integral_coast_assist = False  # handover は関係なく貯める
    for key, val in (fm or {}).items():
        setattr(cfg.forward_map, key, val)
    cfg.mode_drive.pedal_standby = False
    save_table(_table(), Path(cfg.feedforward.model_path))
    return cfg


N_COAST = 80


def _ho_run(
    tmp_path: Path, *, pred_p: bool = False, play_floor: bool = True,
    tail: str = "accel", prepos: float | None = None, fm: dict[str, object] | None = None,
) -> mdmod._ModeRun:
    """惰行の行 N_COAST 行（要る加速度 ＝ 惰行）の後に、tail の行が続く計画で、_c10_offset と
    _coast_handover だけを試せる最小の _ModeRun。"""
    cfg = _cfg(tmp_path, pred_p=pred_p, play_floor=play_floor, fm=fm)
    mode = _mode([(0.0, 0.0), (2.0, 0.0), (8.0, 60.0), (60.0, 60.0), (64.0, 0.0), (66.0, 0.0)])
    setup = mdmod.prepare_mode_drive_from(cfg, mode)
    ff = setup.ff
    plan = ff.plan
    i0 = 200
    n = len(plan.t_s)
    plan.flag[:] = FLAG_MEASURED
    plan.effort_pct[:] = 12.0
    plan.need_accel_kmhs[:] = -2.6  # 60 km/h の惰行の加速度
    plan.flag[i0 : i0 + N_COAST] = FLAG_COAST
    plan.effort_pct[i0 : i0 + N_COAST] = 0.0
    if tail == "brake":
        plan.effort_pct[i0 + N_COAST :] = -20.0
    assert i0 + N_COAST + 5 < n
    if prepos is not None:  # 惰行の区間の最後の 0.2 s（4 行）が先置き
        plan.flag[i0 + N_COAST - 4 : i0 + N_COAST] = FLAG_PREPOS
        plan.effort_pct[i0 + N_COAST - 4 : i0 + N_COAST] = prepos
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)
    run.cfg = cfg
    run.ff = ff
    run.c10 = mdmod.C10Stats(period_s=0.05)
    run._c10_prev_clamped = False
    run.dob = PedalDisturbanceObserver(
        ff.table, tq_accel_s=ff.tq_accel_s, tq_brake_s=ff.tq_brake_s,
        lag_accel_s=ff.dob_lag_accel_s, lag_brake_s=ff.dob_lag_brake_s,
    )
    run.dob_stats = mdmod.DobStats(period_s=0.05)
    run._dob_effort = None
    run.c10_int = C10Integral(brake_on=True, accel_on=True, ti_coef=4.0, low_speed="off")
    run._ci_pct = run._ci_kmhs = None
    run._ci_prev_t = None
    run._ci_dt_s = 0.0
    run._ci_active = run._ci_clamped = run._ci_deep = False
    run._stop_speed_win = deque()
    run._b1_rows = []
    run.coast = CoastAssist(plan.effort_pct, plan.flag, plan.t_s, **mdmod.coast_params(ff, cfg))
    run._pf_on = cfg.forward_map.c10_play_floor
    run._ho_on = True
    run._pf = PlayFloor(
        ff.table, rise_kmhs=RISE, margin_pct=MARGIN, accel_min_pct=AMIN, brake_min_pct=BMIN,
    )
    run._pf_row = None
    run._ho_win = deque()
    win_s = cfg.decel_stop.slope_window_s
    run._ho_tp = {
        PEDAL_ACCEL: ff.dob_lag_accel_s + win_s / 2.0,
        PEDAL_BRAKE: ff.dob_lag_brake_s + win_s / 2.0,
    }
    run._ho_at_floor = False
    run._ho_now = 0
    run._ho_p_now = None
    run._coast_effort = 0.0
    run.i0 = i0  # type: ignore[attr-defined]
    return run


def _t(run: mdmod._ModeRun, k: int) -> float:
    return float(run.ff.plan.t_s[run.i0 + k])  # type: ignore[attr-defined]


def _floor_accel() -> float:
    return AMIN + 6.75 - MARGIN


def test_handover_enters_once_and_holds_with_floor(tmp_path: Path) -> None:
    """1760 s 型: 惰行が強すぎる区間で 1 回だけ入り、遅れが消えても区間の終わりまで保つ。"""
    run = _ho_run(tmp_path)
    totals = []
    for k in range(40):
        dev = 0.4 if k < 20 else 0.0  # 途中で遅れが 0 に戻る（今の C12 なら離す所）
        total, active = run._coast_handover(_t(run, k), 60.0, 60.0, 60.0 - dev, 0.0)  # noqa: SLF001
        assert active, k
        totals.append(total)
        assert total >= _floor_accel() - 1e-9  # 下限より浅くならない（min まで戻らない）
    st = run.coast.stats
    assert st.accel_entries == 1 and st.handover_section_max == 1
    assert st.handover_released_a == st.handover_released_b == 0
    # 偏差 0 でも P は 0 だが I が残り、FF（＝ min）ではなく下限以上で待つ
    assert totals[-1] >= _floor_accel() - 1e-9
    assert run._ho_now == 1  # noqa: SLF001


def test_handover_release_b_then_reenter(tmp_path: Path) -> None:
    run = _ho_run(tmp_path)
    for k in range(10):
        run._coast_handover(_t(run, k), 60.0, 60.0, 59.6, 0.0)  # noqa: SLF001
    # 下限にいて、速すぎ（p ＜ −off）が続く → (b) で離す
    released = False
    for k in range(10, 40):
        _, active = run._coast_handover(_t(run, k), 60.0, 60.0, 60.5, 0.0)  # noqa: SLF001
        if not active:
            released = True
            break
    assert released and run.coast.stats.handover_released_b == 1
    # 同じ区間で再び遅れて |p| ≥ on になったら入り直してよい
    for k in range(40, 60):
        _, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.5, 0.0)  # noqa: SLF001
    assert active
    assert run.coast.stats.accel_entries == 2 and run.coast.stats.handover_section_max == 2


def test_handover_section_end_continues_integral_into_accel_row(tmp_path: Path) -> None:
    """1295 s 型: 区間の終わりで次がアクセルなら離して min へ戻さず、I も続く。"""
    run = _ho_run(tmp_path)
    for k in range(N_COAST):
        _, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.6, 0.0)  # noqa: SLF001
        assert active
    i_before = run.c10_int.i_accel
    assert i_before > 0.0
    plan = run.ff.plan
    k = N_COAST
    ff_e = float(plan.effort_pct[run.i0 + k])  # type: ignore[attr-defined]
    assert ff_e > 0.0 and int(plan.flag[run.i0 + k]) != FLAG_COAST  # type: ignore[attr-defined]
    # 次の周期は惰行でない行: 補助は数えず離すだけ（(a)(b) の回数は増えない）
    total, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.6, ff_e)  # noqa: SLF001
    assert not active
    st = run.coast.stats
    assert st.handover_released_a == st.handover_released_b == 0
    # 計画の行の C10 は I を捨てずに続ける（_cycle の c10_int.reset() は ff＝0 の行だけ）
    assert run.c10_int.i_accel >= i_before - 1e-9
    assert isinstance(total, float)  # c10 の上乗せ（差）。値の大きさは問わない


def test_prepos_row_keeps_integral_only_when_handover_on(tmp_path: Path) -> None:
    run = _ho_run(tmp_path)
    for k in range(20):
        run._coast_handover(_t(run, k), 60.0, 60.0, 59.6, 0.0)  # noqa: SLF001
    i0 = run.c10_int.i_accel
    assert i0 > 0.0
    t = _t(run, 20)
    run._c10_offset(t, 60.0, 59.6, 9.0, flag_override=FLAG_PREPOS)  # noqa: SLF001
    assert run.c10_int.i_accel == pytest.approx(i0)  # 先置きの行で I を捨てない
    run._ho_on = False  # noqa: SLF001
    run._c10_offset(_t(run, 21), 60.0, 59.6, 9.0, flag_override=FLAG_PREPOS)  # noqa: SLF001
    assert run.c10_int.i_accel == 0.0  # off は今まで通り捨てる


def test_handover_release_a_when_next_opposite_pedal_is_close(tmp_path: Path) -> None:
    run = _ho_run(tmp_path, tail="brake")
    released_at = None
    for k in range(N_COAST):
        _, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.6, 0.0)  # noqa: SLF001
        if not active and released_at is None:
            released_at = k
    st = run.coast.stats
    assert released_at is not None and st.handover_released_a == 1
    assert released_at < N_COAST  # 次のブレーキ（行 N_COAST）の手前で離す
    # (a) で離した区間は終わりまで入り直さない
    assert st.accel_entries == 1


def test_handover_pred_p_changes_only_the_p_amount(tmp_path: Path) -> None:
    """on/off の差は P の分だけ（I・DOB は今の偏差）。"""
    (tmp_path / "off").mkdir()
    (tmp_path / "on").mkdir()
    off = _ho_run(tmp_path / "off", pred_p=False)
    on = _ho_run(tmp_path / "on", pred_p=True)
    # 傾きのある入力: 速度が下がり続ける（先読みの偏差 p は今の偏差より大きい）
    outs = {}
    for name, run in (("off", off), ("on", on)):
        res = []
        for k in range(30):
            speed = 60.0 - 0.01 * k
            res.append(run._coast_handover(_t(run, k), 60.0, 60.0, speed, 0.0)[0])  # noqa: SLF001
        outs[name] = (res[-1], run)
    (e_off, _), (e_on, r_on) = outs["off"], outs["on"]
    p = r_on._ho_p_now  # noqa: SLF001
    dev = 60.0 - (60.0 - 0.01 * 29)
    assert p is not None and p > dev  # 先読みは今の偏差より大きい（まだ下がっている）
    assert e_on > e_off
    # I・DOB は同じ入力なので同じ（P の分だけ違う）。差は p と dev の P の差だけ
    ia = off.c10_int.accel_kmhs(True, off.ff.tau_accel_s)
    ib = on.c10_int.accel_kmhs(True, on.ff.tau_accel_s)
    assert ia == pytest.approx(ib)
    kw = dict(
        is_accel=True, need_kmhs=-2.6, ref_kmh=60.0, row_kmh=60.0, tau_s=on.ff.tau_accel_s,
        accel_min_pct=AMIN, brake_min_pct=BMIN, onset_u_pct=_floor_accel() - AMIN,
        integral_kmhs=ia,
    )
    n_dev = coast_assist_effort_detail(off.ff.table, dev_kmh=dev, **kw)[0]  # type: ignore[arg-type]
    n_p = coast_assist_effort_detail(on.ff.table, dev_kmh=p, **kw)[0]  # type: ignore[arg-type]
    assert e_off == pytest.approx(n_dev) and e_on == pytest.approx(n_p)


def _weak_need_run(tmp_path: Path) -> mdmod._ModeRun:
    """要る加速度が惰行より弱い（invert(need) がアクセル側＝ u0 ＞ 0）惰行の行。W 1276 s の型。"""
    run = _ho_run(tmp_path)
    plan = run.ff.plan
    plan.need_accel_kmhs[:] = -2.0  # 60 km/h の惰行は −2.6
    return run


def _direct(run: mdmod._ModeRun, *, is_accel: bool, dev: float) -> float:
    return coast_assist_effort_detail(
        run.ff.table, is_accel=is_accel, need_kmhs=-2.0, ref_kmh=60.0, row_kmh=60.0,
        dev_kmh=dev, tau_s=run.ff.tau_accel_s if is_accel else run.ff.tau_brake_s,
        accel_min_pct=AMIN, brake_min_pct=BMIN,
        onset_u_pct=(
            _floor_accel() - AMIN if is_accel else (BMIN + 2.1 - MARGIN) - BMIN
        ),
    )[0]


def test_handover_brake_entry_is_direct_not_a_delta_jump(tmp_path: Path) -> None:
    """W 1276 s: 要る加速度が惰行より弱い行でブレーキで入った最初の周期。直接引いた値
    （min ＋ 数%）になり、Δu の飛び越え（10% 超）にならない。"""
    run = _weak_need_run(tmp_path)
    total, active = run._coast_handover(_t(run, 20), 60.0, 60.0, 60.3, 0.0)  # noqa: SLF001
    assert active and total < 0.0
    assert total == pytest.approx(_direct(run, is_accel=False, dev=-0.3))
    assert abs(total) - BMIN < 3.0  # min ＋ 数%（以前は −13.9%）


def test_handover_accel_entry_is_direct(tmp_path: Path) -> None:
    run = _weak_need_run(tmp_path)
    total, active = run._coast_handover(_t(run, 0), 60.0, 60.0, 59.7, 0.0)  # noqa: SLF001
    assert active and total > 0.0
    assert total == pytest.approx(_direct(run, is_accel=True, dev=0.3))


def test_handover_does_not_double_count_integral_dt(tmp_path: Path) -> None:
    run = _weak_need_run(tmp_path)
    for k in range(10):
        run._coast_handover(_t(run, k), 60.0, 60.0, 59.7, 0.0)  # noqa: SLF001
    # 1 周期 1 回: 0.05 s × 9 周期 × 偏差 0.3（最初の周期は dt ＝ 0）
    assert run.c10_int.i_accel == pytest.approx(0.3 * 0.05 * 9)


# ─── _play_floor_row（C10 の行の下限） ───────────────────────────────────


def _row_run(tmp_path: Path) -> mdmod._ModeRun:
    run = _ho_run(tmp_path)
    plan = run.ff.plan
    plan.flag[:] = FLAG_MEASURED
    plan.effort_pct[:] = 15.0
    run._ci_active = False  # noqa: SLF001
    return run


def test_play_floor_row_accel_clamps_to_floor_and_marks_integral(tmp_path: Path) -> None:
    run = _row_run(tmp_path)
    t = _t(run, 0)
    run._ci_active = True  # noqa: SLF001
    out = run._play_floor_row(t, 60.0, 60.0, 15.0, -8.0)  # 15 − 8 = 7 ＜ 下限 10.25  # noqa: SLF001
    assert 15.0 + out == pytest.approx(_floor_accel())
    assert run._ci_clamped is True  # 浅く切られた向きへは I を貯めない  # noqa: SLF001
    ps = run._pf_row  # noqa: SLF001
    assert ps is not None and ps[0] is True and ps[2] is True
    # 下限より深ければ何もしない
    assert run._play_floor_row(t, 60.0, 60.0, 15.0, -2.0) == -2.0  # noqa: SLF001
    # FF が下限より浅い行は触らない
    assert run._play_floor_row(t, 60.0, 60.0, 8.0, -3.0) == -3.0  # noqa: SLF001


def test_play_floor_row_brake_and_exclusions(tmp_path: Path) -> None:
    run = _row_run(tmp_path)
    plan = run.ff.plan
    t = _t(run, 0)
    floor_b = BMIN + 2.1 - MARGIN
    out = run._play_floor_row(t, 60.0, 60.0, -20.0, 14.0)  # −6 は下限 −8.6 より浅い  # noqa: SLF001
    assert -20.0 + out == pytest.approx(-floor_b)
    # 7.5 km/h 未満（基準・実車速の小さい方）は掛けない
    assert run._play_floor_row(t, 5.0, 5.0, -20.0, 14.0) == 14.0  # noqa: SLF001
    assert run._play_floor_row(t, 60.0, 3.0, -20.0, 14.0) == 14.0  # noqa: SLF001
    assert run._pf.stats.brake_skipped == 2  # noqa: SLF001
    # 停C の区間（τ_end あり）
    run._stop_end_tau_s = 1.0  # noqa: SLF001
    assert run._play_floor_row(t, 60.0, 60.0, -20.0, 14.0) == 14.0  # noqa: SLF001
    run._stop_end_tau_s = None  # noqa: SLF001
    # 停車前の踏み増し
    plan.stop_tail = np.zeros(len(plan.t_s), dtype=bool)
    plan.stop_tail[:] = True
    assert run._play_floor_row(t, 60.0, 60.0, -20.0, 14.0) == 14.0  # noqa: SLF001
    plan.stop_tail = None
    # 発進のブレーキの帯
    plan.launch_band = np.ones(len(plan.t_s), dtype=bool)
    assert run._play_floor_row(t, 60.0, 60.0, -20.0, 14.0) == 14.0  # noqa: SLF001
    plan.launch_band = None
    # 先置き・停車保持の行は FF がそのペダルの行ではない
    plan.flag[:] = FLAG_PREPOS
    assert run._play_floor_row(t, 60.0, 60.0, 15.0, -8.0) == -8.0  # noqa: SLF001


def test_play_floor_row_accel_is_not_excluded_by_brake_rules(tmp_path: Path) -> None:
    run = _row_run(tmp_path)
    run.ff.plan.launch_band = np.ones(len(run.ff.plan.t_s), dtype=bool)
    out = run._play_floor_row(_t(run, 0), 60.0, 60.0, 15.0, -8.0)  # noqa: SLF001
    assert 15.0 + out == pytest.approx(_floor_accel())
    run.ff.plan.launch_band = None


# ─── 段6a: アクセルは下限なし・ブレーキは下限で待つ・埋めの行の C10 ──────────────


def test_accel_no_floor_goes_to_min_when_answer_is_shallower_than_onset(tmp_path: Path) -> None:
    """要る加速度が惰行より弱くない（表の答えが遊びの中）→ キー on は min、off は下限。"""
    for key, expect in ((True, AMIN), (False, _floor_accel())):
        sub = tmp_path / str(key)
        sub.mkdir()
        run = _ho_run(sub, fm={"coast_assist_accel_no_floor": key})
        run.ff.plan.need_accel_kmhs[:] = -3.0  # 惰行 −2.6 より強い減速 ＝ 答えは遊びの中
        total, active = run._coast_handover(_t(run, 0), 60.0, 60.0, 59.7, 0.0)  # noqa: SLF001
        assert active and total == pytest.approx(expect)
        assert run._ho_at_floor is True  # noqa: SLF001  on: min にいる／off: 下限にいる
        assert (run._pf_row is None) == key  # noqa: SLF001  on: アクセルの下限の行は出さない


def test_accel_no_floor_leaves_brake_floor_alone(tmp_path: Path) -> None:
    run = _ho_run(tmp_path, fm={"coast_assist_accel_no_floor": True})
    run.ff.plan.need_accel_kmhs[:] = -2.0
    total, _ = run._coast_handover(_t(run, 20), 60.0, 60.0, 60.3, 0.0)  # noqa: SLF001
    assert total < 0.0 and run._pf_row is not None  # noqa: SLF001


def _brake_run(tmp_path: Path, key: bool) -> mdmod._ModeRun:
    run = _ho_run(tmp_path, fm={"coast_assist_brake_wait_floor": key})
    run._dob_estimate = lambda *_a, **_k: 1.0  # type: ignore[method-assign]  # DOB は深くする向き
    return run


def test_brake_wait_floor_caps_total_only_while_predicted_slow(tmp_path: Path) -> None:
    fl_b = BMIN + 2.1 - MARGIN
    for key in (True, False):
        sub = tmp_path / str(key)
        sub.mkdir()
        run = _brake_run(sub, key)
        # 速すぎ（p_brake ＜ −off）でブレーキに入る: DOB が深くしてよい（切らない）
        for k in range(20, 25):
            total, active = run._coast_handover(_t(run, k), 60.0, 60.0, 60.3, 0.0)  # noqa: SLF001
            assert active and total < -fl_b - 1.0
        deep = total
        # 遅れの見込み（p_brake ＞ off）に変わった: on は下限で切る
        run._ho_win.clear()  # noqa: SLF001  車速の段差が傾きに出ないよう窓を空にする
        for k in range(25, 28):
            total, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.85, 0.0)  # noqa: SLF001
        assert active  # p_accel ＝ 0.15 ＜ on ＝ 0.2: アクセルが要るほどではない（on は待つ）
        if key:
            assert total == pytest.approx(-fl_b)
            assert run._ci_deep is True  # noqa: SLF001  I はそれ以上深く貯めない
            assert run._dob_effort == pytest.approx(0.0)  # noqa: SLF001  下限で切られ DOB は効かない
            assert run._ho_at_floor is True  # noqa: SLF001
            # p_accel ≥ on になったら離す（アクセルが要る見込み）
            run._ho_win.clear()  # noqa: SLF001
            _, active = run._coast_handover(_t(run, 28), 60.0, 60.0, 59.7, 0.0)  # noqa: SLF001
            assert not active and run.coast.stats.handover_switched == 1
        else:
            assert total < -fl_b - 1.0 and total != deep
            assert run._ci_deep is False  # noqa: SLF001


def test_brake_wait_floor_off_by_default_in_config() -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.coast_assist_brake_wait_floor is False and fm.coast_assist_accel_no_floor is False
    assert fm.coast_fill_same_pedal is False


def _fill_run(
    tmp_path: Path, *, key: bool, accel: bool = True, extra: dict[str, object] | None = None
) -> mdmod._ModeRun:
    """行 i0 から N_COAST 行が FLAG_COAST_FILL の計画（FF は効き始めで保つ開度）。"""
    from tests.research.fwd_plan import FLAG_COAST_FILL

    fm = {"coast_fill_same_pedal": key, "coast_fill_max_s": 2.0, **(extra or {})}
    run = _ho_run(tmp_path, fm=fm)
    plan = run.ff.plan
    run._ho_on = False  # noqa: SLF001  _c10_offset を直接呼ぶ
    plan.flag[run.i0 : run.i0 + N_COAST] = FLAG_COAST_FILL  # type: ignore[attr-defined]
    plan.effort_pct[run.i0 : run.i0 + N_COAST] = (  # type: ignore[attr-defined]
        AMIN + 6.75 if accel else -(BMIN + 2.1)
    )
    return run


def test_fill_row_accel_stays_at_floor_when_need_is_stronger_than_coast(tmp_path: Path) -> None:
    """10-31 の −9% 跳ね: 要る加速度が惰行より強い減速で、小さい正の偏差。通常の C10 は
    u0 ＝ 表のアクセル側・u1 ＝ ブレーキ側で大きく負に跳ぶ。キー on は下限に止まる。"""
    ff_e = AMIN + 6.75
    outs = {}
    for key in (False, True):
        sub = tmp_path / str(key)
        sub.mkdir()
        run = _fill_run(sub, key=key)
        run.ff.plan.need_accel_kmhs[:] = -4.0  # 惰行 −2.6 より強く減速
        outs[key] = run._c10_offset(_t(run, 5), 60.0, 59.9, ff_e)  # noqa: SLF001
        if key:
            total = ff_e + outs[key]
            assert total == pytest.approx(_floor_accel())  # 下限（アクセルのまま・負にならない）
            assert total > 0.0
    assert outs[False] != pytest.approx(outs[True])  # off は今までの式（上乗せ ＝ Δu）のまま


def test_fill_row_accel_adds_on_top_of_floor_when_slow(tmp_path: Path) -> None:
    ff_e = AMIN + 6.75
    run = _fill_run(tmp_path, key=True)
    run.ff.plan.need_accel_kmhs[:] = -2.6
    delta = run._c10_offset(_t(run, 5), 60.0, 59.0, ff_e)  # noqa: SLF001  偏差 1.0（τ ＝ 1 s）
    expect = coast_assist_effort_detail(
        run.ff.table, is_accel=True, need_kmhs=-2.6, ref_kmh=60.0, row_kmh=60.0, dev_kmh=1.0,
        tau_s=run.ff.tau_accel_s, accel_min_pct=AMIN, brake_min_pct=BMIN,
        onset_u_pct=_floor_accel() - AMIN,
    )[0]
    assert ff_e + delta == pytest.approx(expect) and ff_e + delta > _floor_accel()


def test_fill_row_brake_never_turns_positive(tmp_path: Path) -> None:
    ff_e = -(BMIN + 2.1)
    run = _fill_run(tmp_path, key=True, accel=False)
    run.ff.plan.need_accel_kmhs[:] = -1.0  # 惰行より弱い減速（表を引くとアクセル側）
    delta = run._c10_offset(_t(run, 5), 60.0, 59.9, ff_e)  # noqa: SLF001
    assert ff_e + delta < 0.0
    assert ff_e + delta == pytest.approx(-(BMIN + 2.1 - MARGIN))


# ─── 段6e: アクセルは効き始めより浅ければ min・埋めの行のブレーキは下限で待つ ──────


def _fake_answer(monkeypatch: pytest.MonkeyPatch, state: dict[str, float]) -> None:
    """handover の「表から引いた答え」を state["v"] に差し替える（効き始め手前の答えを作る）。"""
    import tests.research.fwd_plan as fwd_plan

    monkeypatch.setattr(
        fwd_plan, "coast_assist_effort_detail",
        lambda _tbl, *, dob_kmhs=0.0, integral_kmhs=0.0, **_kw: (state["v"], False, False),
    )


def test_accel_no_play_parks_at_min_with_hysteresis(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """答えが効き始め（余白 0）未満 → min。効き始め ＋ 0.5 未満では min のまま・以上で戻る。"""
    run = _ho_run(
        tmp_path, fm={"coast_assist_accel_no_floor": True, "coast_assist_accel_no_play": True}
    )
    onset = run._pf.onset_mag(60.0, is_accel=True)  # noqa: SLF001
    assert onset is not None and onset > AMIN + 1.0
    unpark = run.cfg.forward_map.coast_assist_accel_unpark_pct
    state = {"v": AMIN + 1.0}
    _fake_answer(monkeypatch, state)
    seq = [  # (答え, 出る開度, min にいるか)
        (AMIN + 1.0, AMIN, True),  # 効き始め未満 → min
        (onset + unpark - 0.1, AMIN, True),  # 効き始め ＋ 0.5 未満は min のまま
        (onset + unpark, onset + unpark, False),  # 以上で戻る
        (onset - 0.1, AMIN, True),  # また効き始め未満 → min
    ]
    for k, (ans, expect, parked) in enumerate(seq):
        state["v"] = ans
        total, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.7, 0.0)  # noqa: SLF001
        assert active and total == pytest.approx(expect)
        assert run._ho_accel_parked is parked  # noqa: SLF001
        if parked:
            assert run._dob_effort == pytest.approx(0.0)  # noqa: SLF001
            assert run._ci_pct == pytest.approx(0.0)  # noqa: SLF001
            assert run._ci_clamped is True  # noqa: SLF001  I は浅い向きへ貯めない
    assert run.coast.stats.handover_parked == 3
    assert "アクセルを min にした 0.2s" in run.coast.stats.handover_summary()


def test_accel_no_play_off_and_brake_unaffected(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # キー off（accel_no_floor だけ on）: 効き始め未満の答えもそのまま出す
    run = _ho_run(tmp_path, fm={"coast_assist_accel_no_floor": True})
    state = {"v": AMIN + 1.0}
    _fake_answer(monkeypatch, state)
    total, _ = run._coast_handover(_t(run, 0), 60.0, 60.0, 59.7, 0.0)  # noqa: SLF001
    assert total == pytest.approx(AMIN + 1.0)
    assert run._ho_accel_parked is False  # noqa: SLF001
    # ブレーキ側には効かない（浅い答えもそのまま）
    (tmp_path / "b").mkdir()
    run = _ho_run(
        tmp_path / "b", fm={"coast_assist_accel_no_floor": True, "coast_assist_accel_no_play": True}
    )
    state["v"] = -(BMIN + 1.0)
    total, _ = run._coast_handover(_t(run, 20), 60.0, 60.0, 60.3, 0.0)  # noqa: SLF001
    assert total == pytest.approx(-(BMIN + 1.0)) and run._ho_accel_parked is False  # noqa: SLF001


def test_accel_no_play_resets_flag_when_not_accel(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    run = _ho_run(
        tmp_path, fm={"coast_assist_accel_no_floor": True, "coast_assist_accel_no_play": True}
    )
    _fake_answer(monkeypatch, {"v": AMIN + 1.0})
    run._coast_handover(_t(run, 0), 60.0, 60.0, 59.7, 0.0)  # noqa: SLF001
    assert run._ho_accel_parked is True  # noqa: SLF001
    # 惰行の区間の外（計画が惰行でない行・pedal なし）でリセット
    run._coast_handover(_t(run, N_COAST + 10), 60.0, 60.0, 60.0, 12.0)  # noqa: SLF001
    assert run._ho_accel_parked is False  # noqa: SLF001


def _fill_brake_total(tmp_path: Path, key: bool, speed: float) -> tuple[float, mdmod._ModeRun]:
    run = _fill_run(
        tmp_path, key=True, accel=False, extra={"coast_fill_brake_wait_floor": key}
    )
    run._ho_on = True  # noqa: SLF001  p_brake を出す
    run._dob_estimate = lambda *_a, **_k: 1.0  # type: ignore[method-assign]  # DOB は深くする向き
    ff_e = -(BMIN + 2.1)
    run.ff.plan.need_accel_kmhs[:] = -2.6
    delta = run._c10_offset(_t(run, 5), 60.0, speed, ff_e)  # noqa: SLF001
    return ff_e + delta, run


def test_fill_brake_wait_floor_caps_only_while_predicted_slow(tmp_path: Path) -> None:
    fl_b = BMIN + 2.1 - MARGIN
    for k, sp in enumerate((59.85, 60.3)):  # p_brake ＝ 0.15 ＞ off（遅れの見込み）／−0.3
        sub = tmp_path / str(k)
        sub.mkdir()
        (sub / "on").mkdir()
        (sub / "off").mkdir()
        on, run_on = _fill_brake_total(sub / "on", True, sp)
        off, _ = _fill_brake_total(sub / "off", False, sp)
        if sp < 60.0:
            assert on == pytest.approx(-fl_b)  # 下限で止まる
            assert off < -fl_b - 0.5  # off は DOB でそれより深い
            assert run_on._ci_deep is True  # noqa: SLF001
            assert run_on._dob_effort == pytest.approx(0.0)  # noqa: SLF001
            assert run_on.coast.stats.fill_wait_capped == 1
        else:
            assert on == pytest.approx(off)  # p ≤ off なら今と同じ
            assert run_on.coast.stats.fill_wait_capped == 0


def test_fill_brake_wait_floor_leaves_accel_fill_row(tmp_path: Path) -> None:
    ff_e = AMIN + 6.75
    outs = []
    for key in (True, False):
        sub = tmp_path / str(key)
        sub.mkdir()
        run = _fill_run(sub, key=True, extra={"coast_fill_brake_wait_floor": key})
        run._ho_on = True  # noqa: SLF001
        run.ff.plan.need_accel_kmhs[:] = -2.6
        outs.append(run._c10_offset(_t(run, 5), 60.0, 59.0, ff_e))  # noqa: SLF001
    assert outs[0] == pytest.approx(outs[1])


def test_play_and_fill_wait_options_default_off_and_validated() -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.coast_assist_accel_no_play is False and fm.coast_fill_brake_wait_floor is False
    assert fm.coast_assist_accel_unpark_pct == 0.5
    cfg = cfgmod.load_config()
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.coast_assist_accel_no_floor = False  # accel_no_play（本体 on）が要る
    assert any("coast_assist_accel_no_play" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.coast_assist_accel_no_floor = True
    cfg.forward_map.coast_assist_accel_unpark_pct = 0.0
    assert any("coast_assist_accel_unpark_pct" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.coast_assist_accel_unpark_pct = 0.5
    cfg.forward_map.coast_fill_same_pedal = False  # fill_brake_wait_floor（本体 on）が要る
    assert any("coast_fill_brake_wait_floor" in p for p in cfgmod.validate_config(cfg))


# ─── 設定 ──────────────────────────────────────────────────────────────


def test_config_defaults_are_off_and_validated() -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.c10_play_floor is False and fm.play_floor_rise_kmhs == 0.05
    assert fm.coast_assist_handover is False and fm.coast_assist_pred_p is False
    cfg = cfgmod.load_config()
    # 本体 yaml: 下限・handover は on、先読み P は off（段6h ユーザー決定 2026-11-09）
    assert cfg.forward_map.c10_play_floor is True
    assert cfg.forward_map.coast_assist_handover is True
    assert cfg.forward_map.coast_assist_pred_p is False
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.coast_assist_pred_p = True
    cfg.forward_map.coast_assist_handover = False
    assert [p for p in cfgmod.validate_config(cfg) if "coast_assist_handover" in p]
    cfg.forward_map.coast_assist_handover = True
    cfg.forward_map.coast_assist_pred_p = False
    cfg.forward_map.play_floor_rise_kmhs = 0.0
    assert [p for p in cfgmod.validate_config(cfg) if "play_floor_rise_kmhs" in p]


def test_prepos_row_same_pedal_keeps_handover_opening(tmp_path: Path) -> None:
    """惰行(handover アクセル) → 先置き(アクセル 10%) → アクセル: 先置きの行で min・先置きへ
    落ちず、handover の開度を保つ。次のアクセルの行は通常の C10 で I が続く。"""
    run = _ho_run(tmp_path, prepos=10.0)
    ff = run.ff
    ff_pre = 10.0
    last = None
    for k in range(N_COAST - 4):
        last, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.6, 0.0)  # noqa: SLF001
        assert active
    assert last is not None and last > ff_pre
    for k in range(N_COAST - 4, N_COAST):
        delta, active = run._coast_handover(  # noqa: SLF001
            _t(run, k), 60.0, 60.0, 59.6, ff_pre
        )
        assert active
        final = ff_pre + delta  # _cycle が出す開度
        assert final >= last - 0.2  # 先置き開度（10%）へ落ちない
        assert final >= ff_pre
    st = run.coast.stats
    assert st.accel_entries == 1 and st.handover_released_a == st.handover_released_b == 0
    i_before = run.c10_int.i_accel
    assert i_before > 0.0
    # 次のアクセルの行（通常の C10）。段差の大きさだけ確かめる（判断はしない）
    k = N_COAST
    plan = ff.plan
    e = float(plan.effort_pct[run.i0 + k])  # type: ignore[attr-defined]
    delta, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.6, e)  # noqa: SLF001
    assert not active
    step = (e + delta) - final
    assert abs(step) < 100.0 and run.c10_int.i_accel >= i_before - 1e-9
    print("handover 最後の開度", final, "→ 次のアクセルの行", e + delta, "段差", step)


def test_prepos_row_opposite_pedal_releases(tmp_path: Path) -> None:
    run = _ho_run(tmp_path, prepos=-9.0)
    released = False
    for k in range(N_COAST - 4):
        _, active = run._coast_handover(_t(run, k), 60.0, 60.0, 59.6, 0.0)  # noqa: SLF001
        released = released or not active
    assert run.coast.pedal is None  # 先置きのブレーキの前に離している
    for k in range(N_COAST - 4, N_COAST):
        delta, active = run._coast_handover(  # noqa: SLF001
            _t(run, k), 60.0, 60.0, 59.6, -9.0
        )
        assert not active
    assert released
