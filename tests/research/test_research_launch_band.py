"""2026-10-31 発進のブレーキの帯（案 B。forward_map.launch_brake_band）:
hold_scene.launch_band_intervals・fwd_plan.build_plan(launch_band=)・mode_drive.launch_band_for。
ProblemReport_20261031_発進でブレーキを離すのが早い 段3。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import mode_drive as mdmod
from tests.research.fwd_plan import (
    FLAG_STOP,
    LaunchBandSpec,
    PedalTrack,
    apply_launch_shift,
    track_pedal_position,
)
from tests.research.hold_scene import (
    HoldInterval,
    launch_band_intervals,
    lead_in_ff_at,
    lead_in_ff_pct,
)
from tests.research.test_research_fwd_plan import AD, BD, DT, _pedal_table, plan_for
from tests.research.test_research_hold_scene import _hold_cfg, _meta
from tests.research.test_research_mode_drive import _mode

CREEP_X = [0.5, 1.5, 2.5, 3.5, 5.0]
CREEP_Y = [2.2, 1.7, 1.33, 0.85, 0.0]
LAG = 0.155
STOP_KMH = 0.02
CREEP_KMH = 5.0


def _weak_launch(t: float) -> float:
    """停車 → 1.6 km/h/s で 1 秒（t=3〜4）→ 5.3 km/h/s（UDDS 346 s と同じ形）。"""
    if t <= 3.0:
        return 0.0
    if t <= 4.0:
        return 1.6 * (t - 3.0)
    return 1.6 + 5.3 * (t - 4.0)


def _strong_launch(t: float) -> float:
    return 0.0 if t <= 3.0 else 4.5 * (t - 3.0)


def _bands(ref, dur=10.0, *, ret_s=0.2, ff=8.0, exclude=()):
    return launch_band_intervals(
        ref, dur, DT, LAG, stop_kmh=STOP_KMH, creep_kmh=CREEP_KMH,
        creep_speeds_kmh=CREEP_X, creep_accel_kmhs=CREEP_Y,
        ff_pct_at=lambda t: ff, return_seconds=lambda depth: ret_s, exclude=list(exclude),
    )


def test_band_starts_at_t_go_and_ends_before_slope_exceeds_curve() -> None:
    (iv,) = _bands(_weak_launch)
    # 2026-11-01 修正 2: 基準が 0.02 を超える最初の行（3.05 s）の L（0.155 s → 3 行）前から始める
    assert iv.start_s == pytest.approx(3.05 - 3 * DT)
    # 4.0 s で傾きが 1.6 → 5.3（曲線は 1.6 km/h 付近で約 1.7）。L 先の傾きが超える手前で終わる
    assert 3.6 < iv.end_s < 4.0
    ahead = iv.end_s - DT + LAG
    slope = (_weak_launch(ahead + DT) - _weak_launch(ahead)) / DT
    assert slope <= np.interp(_weak_launch(ahead), CREEP_X, CREEP_Y) + 1e-9
    nxt = iv.end_s + LAG  # 区間の次の周期は L 先の傾きが曲線を上回る
    assert (_weak_launch(nxt + DT) - _weak_launch(nxt)) / DT > np.interp(
        _weak_launch(nxt), CREEP_X, CREEP_Y
    )


def test_strong_launch_has_no_band() -> None:
    assert _bands(_strong_launch) == []


def test_band_is_limited_to_creep_speed() -> None:
    def slow(t: float) -> float:  # 傾き 0.5 km/h/s でずっと上がる → 5 km/h 手前で終わる
        return 0.0 if t <= 3.0 else 0.5 * (t - 3.0)

    (iv,) = _bands(slow, 20.0)
    ahead = iv.end_s + LAG  # 次の周期: L 先の傾き 0.5 が曲線（5 km/h に近づくほど小さい）を上回る
    assert slow(ahead) < CREEP_KMH and 0.5 > np.interp(slow(ahead), CREEP_X, CREEP_Y)
    assert iv.end_s < 3.0 + CREEP_KMH / 0.5


def test_band_skipped_when_shorter_than_return_time_plus_lag() -> None:
    (iv,) = _bands(_weak_launch, ret_s=0.2)
    # 長さの判定は伸ばす前（t_go から終わりまで）の長さ。伸ばした分（3 行）は数えない
    base = iv.length_s - 3 * DT
    assert _bands(_weak_launch, ret_s=base - LAG + 0.01) == []  # 長さ < 戻る時間 + L
    assert len(_bands(_weak_launch, ret_s=base - LAG)) == 1  # ちょうどは残す


def test_band_skipped_when_overlapping_existing_interval() -> None:
    (iv,) = _bands(_weak_launch)
    lead = HoldInterval(iv.start_s, 0.5, 0.5)  # 上りが先頭に重なる
    assert _bands(_weak_launch, exclude=[lead]) == []
    far = HoldInterval(iv.end_s, 2.0, 1.6)  # 接するだけ（重ならない）は残す
    assert len(_bands(_weak_launch, exclude=[far])) == 1


def test_band_not_made_for_ref_that_starts_moving_at_t0() -> None:
    assert _bands(lambda t: 1.0 + 0.1 * t, 10.0) == []  # 停車からの発進ではない


def test_lead_in_ff_at_equals_lead_in_ff_pct_plus_offset() -> None:
    pts = [(0.0, 8.9), (2.0, 7.6), (5.0, 4.2)]
    t = 3.5
    ahead = t + LAG
    r0 = _weak_launch(ahead)
    sl = (_weak_launch(ahead + DT) - r0) / DT
    want = lead_in_ff_pct(pts, r0, sl, 6.8, 4.2, 18.0) + 0.3
    assert lead_in_ff_at(_weak_launch, t, LAG, DT, pts, 6.8, 4.2, 18.0, 0.3) == pytest.approx(want)


# ───────── build_plan ─────────

BK_U = -1.0  # _pedal_table(breakaway=-1.0) → bk ＝ BD + 1.0


_PLAN_KW = dict(
    low_speed_side_rule="dir", launch_release_ramp=True, return_gap_enabled=True,
    brake_lag_s=LAG, accel_lag_s=0.2,
)


def _tb(dl: float = 3.5):
    """発進側の惰行の加速度が +1.5（クリープ向き）の表。dl=2.0 は 0（基準が動く前から踏む）。"""
    return _pedal_table(dl=dl, breakaway=BK_U)


def _plan(ref, spec=None, table=None, **kw):
    return plan_for(ref, 16.0, table or _tb(), **dict(_PLAN_KW, **kw), launch_band=spec)


def _weak(t: float) -> float:
    return _weak_launch(t - 5.0)  # 停車 8 s の後に発進


def test_plan_off_is_identical() -> None:
    for extra in ({}, {"launch_gap_tight": True},
                  {"predict_low_speed_sides": True, "launch_shift_choose": True}):
        a = plan_for(_weak, 16.0, _tb(), **dict(_PLAN_KW, **extra))
        b = _plan(_weak, None, **extra)
        assert np.array_equal(a.effort_pct, b.effort_pct) and np.array_equal(a.flag, b.flag)
        assert np.array_equal(a.launch_prep, b.launch_prep) and b.launch_band is None


def test_plan_band_rows_are_brake_only_and_accel_follows() -> None:
    # _weak は t=8.0 から動く。区間は _weak の基準で求める
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    off = _plan(_weak)
    on = _plan(_weak, spec)
    i0 = int(round(ivs[0].start_s / DT))
    i1 = i0 + int(round(ivs[0].length_s / DT))
    # 区間の中にアクセルの行が無い。ブレーキの行は FF そのもの（-8.5）か、後ろのすき間で 0
    band = on.effort_pct[i0:i1]
    assert (band <= 0.0).all() and band[0] == pytest.approx(-8.5)
    # 2026-11-01 修正: 印はブレーキの行だけ（終わりのすき間の 0 の行は印なし）
    assert on.launch_band is not None
    assert np.array_equal(on.launch_band[i0:i1], on.effort_pct[i0:i1] < 0.0)
    assert on.launch_band[i0]
    assert not on.launch_band[:i0].any() and not on.launch_band[i1:].any()
    assert on.launch_prep[i0:i1].all()
    # 離しの時刻: off は t_go − L の行で min へ切るが、on は帯の終わり近くまでブレーキ
    last_b_off = int(np.flatnonzero(off.effort_pct[: i1 + 20] < 0.0)[-1])
    last_b_on = int(np.flatnonzero(on.effort_pct[: i1 + 20] < 0.0)[-1])
    assert last_b_off < last_b_on and i0 <= last_b_on
    assert on.return_gap.n_violations == 0
    # アクセルは帯の後
    assert int(np.flatnonzero(on.effort_pct > 0.0)[0]) >= i1 - 1


def test_plan_before_band_holds_bk_and_leaves_other_rows() -> None:
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    on = _plan(_weak, spec)
    off = _plan(_weak)
    i0 = int(round(ivs[0].start_s / DT))
    bk = BD + abs(BK_U)
    i_go = int(round(i0 - LAG / DT))
    assert (on.effort_pct[i_go : i0] <= -bk + 1e-9).all()  # 帯の前は bk 以上（止まっている）
    # 帯の 2 秒より前と、帯の終わりの 4 秒より後は変わらない
    i1 = i0 + int(round(ivs[0].length_s / DT))
    far = np.r_[np.arange(0, i_go - 40), np.arange(i1 + 80, len(on.effort_pct))]
    assert np.array_equal(on.effort_pct[far], off.effort_pct[far])


def test_plan_one_foot_rule_holds() -> None:
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    for p in (_plan(_weak), _plan(_weak, spec), _plan(_weak, spec, launch_gap_tight=True)):
        pa, pb = track_pedal_position(
            p.effort_pct, period_s=DT, track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD
        )
        both = (pa > AD + 0.05) & (pb > BD + 0.05)
        assert int(np.count_nonzero(both)) == 0


def test_plan_strong_launch_has_no_band_rows() -> None:
    def strong(t: float) -> float:
        return _strong_launch(t - 5.0)

    ivs = _bands(strong, 16.0)
    assert ivs == []
    spec = LaunchBandSpec((), lambda t: 8.5)
    a, b = _plan(strong), _plan(strong, spec)
    assert np.array_equal(a.effort_pct, b.effort_pct) and not b.launch_band.any()


def test_plan_launch_shift_skips_band_launch() -> None:
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    kw = {"predict_low_speed_sides": True, "launch_shift_choose": True}
    on = _plan(_weak, spec, **kw)
    assert on.launch_shift.deltas == [0]
    base = _plan(_weak, spec)
    res = apply_launch_shift(
        base, _weak, _tb(), stop_ref_at=None, accel_min_pct=AD,
        brake_min_pct=BD, track=PedalTrack(),
    )
    assert res.launch_shift.deltas == [0] and np.array_equal(res.effort_pct, base.effort_pct)
    assert FLAG_STOP in on.flag  # 停車保持は今のまま


def test_plan_launch_band_needs_release_ramp() -> None:
    spec = LaunchBandSpec(((8.05, 0.5),), lambda t: 8.5)
    with pytest.raises(ValueError, match="launch_release_ramp"):
        plan_for(_weak, 16.0, _tb(), launch_band=spec)


def _band_gap_rows(tight: bool):
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    on = _plan(_weak, spec, launch_gap_tight=tight)
    i1 = int(round(ivs[0].start_s / DT)) + int(round(ivs[0].length_s / DT))
    return on, i1


def test_plan_band_gap_rows_have_no_mark_and_zero_effort() -> None:
    # 2026-11-01 修正 (a): tight on・off とも、帯の終わりのすき間（ブレーキの後・アクセルの前）は
    # 印なしで effort 0
    for tight in (False, True):
        on, i1 = _band_gap_rows(tight)
        last_b = int(np.flatnonzero(on.launch_band)[-1])
        first_a = int(np.flatnonzero(on.effort_pct[last_b:] > 0.0)[0]) + last_b
        gap = np.arange(last_b + 1, first_a)
        assert len(gap) >= 1, tight
        assert (on.effort_pct[gap] == 0.0).all() and not on.launch_band[gap].any()


def test_plan_band_mark_is_brake_only_and_survives_shift() -> None:
    # (b) 最終の計画で 印 かつ effort >= 0 の行は無い。他の発進のずらしを通しても同じ
    for tight in (False, True):
        on, _ = _band_gap_rows(tight)
        assert int(np.count_nonzero(on.launch_band & (on.effort_pct >= 0.0))) == 0
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    kw = {"predict_low_speed_sides": True, "launch_shift_choose": True}
    for tight in (False, True):
        p = _plan(_weak, spec, launch_gap_tight=tight, **kw)
        assert int(np.count_nonzero(p.launch_band & (p.effort_pct >= 0.0))) == 0


def test_plan_band_row_lookup_in_mode_drive() -> None:
    # (c) 区間の中でも計画の印のない行は False（hold_scene 3 にならず計画の FF のまま）
    from types import SimpleNamespace

    from tests.research.ff_candidate import ForwardMapFeedforward

    on, i1 = _band_gap_rows(True)
    ff = ForwardMapFeedforward.__new__(ForwardMapFeedforward)
    ff._plan = on  # noqa: SLF001
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)  # noqa: SLF001
    run.ff = ff
    run._b1_rows = []  # noqa: SLF001  2026-11-08 段20 B-1: 伸ばした周期（無し）
    last_b = int(np.flatnonzero(on.launch_band)[-1])
    assert run._plan_band_row(last_b * DT) is True  # noqa: SLF001
    assert run._plan_band_row((last_b + 1) * DT) is False  # noqa: SLF001  すき間の行
    # 計画を持たない FF は区間の条件だけ
    run.ff = SimpleNamespace()
    assert run._plan_band_row(1.0) is True  # noqa: SLF001


# ───────── 設定・mode_drive ─────────


def test_launch_brake_band_default_false_and_config_checks() -> None:
    assert cfgmod.ForwardMapSection().launch_brake_band is False
    cfg = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    # 2026-11-01 段4 で採用: 本体 yaml は true（依存する設定が揃っている）
    assert cfg.forward_map.launch_brake_band is True
    assert cfgmod.validate_config(cfg) == []
    for name in ("launch_release_ramp", "hold_pi_enabled", "hold_ff_enabled"):
        c2 = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
        c2.forward_map.launch_brake_band = True
        c2.forward_map.launch_move_hold = False
        c2.forward_map.launch_gap_tight = False
        c2.forward_map.hold_lead_in_enabled = False
        c2.forward_map.hold_accel_pred_enabled = False
        setattr(c2.forward_map, name, False)
        assert any("launch_brake_band" in p for p in cfgmod.validate_config(c2)), name
    c3 = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    c3.forward_map.launch_brake_band = True
    c3.feedforward.creep_accel_speeds_kmh = []
    c3.feedforward.creep_accel_kmhs = []
    c3.forward_map.predict_creep_curve = False
    assert any("launch_brake_band" in p for p in cfgmod.validate_config(c3))


def _band_cfg(tmp_path: Path, *, with_opening: bool = True, lead_in: bool = False):
    from tests.research.fwd_map import load_forward_map, save_table

    cfg = _hold_cfg(tmp_path, with_hold_pi=True, with_opening=with_opening)
    path = Path(cfg.feedforward.model_path)
    table = load_forward_map(path)
    table.meta["hold_pi"].update({"K_kmh_per_pct": 47.9, "T_s": 7.05})
    table.meta["launch_breakaway_u_pct"] = -1.0
    save_table(table, path)
    cfg.forward_map.hold_ff_enabled = True
    cfg.forward_map.hold_lead_in_enabled = lead_in
    cfg.forward_map.launch_release_ramp = True
    cfg.forward_map.launch_brake_band = True
    cfg.feedforward.creep_accel_speeds_kmh = list(CREEP_X)
    cfg.feedforward.creep_accel_kmhs = list(CREEP_Y)
    return cfg


def _mode_weak():
    return _mode([(0.0, 0.0), (3.0, 0.0), (4.0, 1.6), (4.05, 1.865), (8.0, 20.0), (9.0, 20.0)])


def test_launch_band_for_off_is_none_and_plan_args_have_no_key(tmp_path: Path) -> None:
    cfg = _band_cfg(tmp_path)
    cfg.forward_map.launch_brake_band = False
    mode = _mode_weak()
    ff = mdmod.load_feedforward(cfg)
    assert mdmod.launch_band_for(cfg, mode, ff.table) is None
    _, _, kw = mdmod.forward_map_plan_args(cfg, mode, stop_end=False, table=ff.table)
    assert "launch_band" not in kw


def test_launch_band_for_on_gives_spec_and_plan_args(tmp_path: Path) -> None:
    cfg = _band_cfg(tmp_path)
    mode = _mode_weak()
    ff = mdmod.load_feedforward(cfg)
    spec, bands = mdmod.launch_band_for(cfg, mode, ff.table)
    assert len(bands) == 1 and bands[0].start_s == pytest.approx(3.0)  # t_go − L（1 行）
    assert spec.intervals == ((bands[0].start_s, bands[0].length_s),)
    # FF は上りと同じ式（保持の開度 − 傾き ÷ c）
    t = bands[0].start_s
    assert spec.ff_at(t) == pytest.approx(
        lead_in_ff_at(
            mdmod.ReferenceSpeed(mode).at, t, 0.05, 0.05,
            __import__("tests.research.hold_scene", fromlist=["x"]).hold_opening_points(_meta()),
            47.9 / 7.05, cfg.vehicle.min_brake_opening_pct, cfg.stop_brake_opening_pct, 0.0,
        )
    )
    _, _, kw = mdmod.forward_map_plan_args(cfg, mode, stop_end=False, table=ff.table)
    assert kw["launch_band"].intervals == spec.intervals


def test_launch_band_for_overlap_with_lead_in_is_dropped(tmp_path: Path) -> None:
    # UDDS 1251 s と同じ形（0 → 1.6 を 1 秒で上がり 6 秒一定）: 上りが先に取る
    cfg = _band_cfg(tmp_path, lead_in=True)
    mode = _mode([(0.0, 0.0), (3.0, 0.0), (4.0, 1.6), (10.0, 1.6)])
    ff = mdmod.load_feedforward(cfg)
    _, bands = mdmod.launch_band_for(cfg, mode, ff.table)
    assert bands == []


def test_launch_band_for_missing_meta_or_curve_is_config_error(tmp_path: Path) -> None:
    cfg = _band_cfg(tmp_path, with_opening=False)
    mode = _mode_weak()
    ff = mdmod.load_feedforward(cfg)
    with pytest.raises(cfgmod.ConfigError, match="hold_opening"):
        mdmod.launch_band_for(cfg, mode, ff.table)
    cfg2 = _band_cfg(tmp_path)
    ff2 = mdmod.load_feedforward(cfg2)
    cfg2.feedforward.creep_accel_speeds_kmh = []
    with pytest.raises(cfgmod.ConfigError, match="creep_accel"):
        mdmod.launch_band_for(cfg2, mode, ff2.table)
    cfg3 = _band_cfg(tmp_path)
    cfg3.forward_map.hold_ff_enabled = False
    with pytest.raises(cfgmod.ConfigError, match="hold_ff_enabled"):
        mdmod.launch_band_for(cfg3, mode, mdmod.load_feedforward(cfg3).table)


def test_plan_band_starts_at_t_go_minus_lag_and_has_no_accel() -> None:
    # 2026-11-01 修正 2: 帯は t_go − L の行から。発進側の表が「基準が動く前からアクセル」と言う
    # 計画でも、帯の中にアクセルは無く、両ペダル同時もない
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    on = _plan(_weak, spec, table=_tb(2.0))
    i0 = int(round(ivs[0].start_s / DT))
    i1 = i0 + int(round(ivs[0].length_s / DT))
    t_go_row = int(round(8.05 / DT))
    assert i0 == t_go_row - int(round(LAG / DT))
    assert (on.effort_pct[i0:i1] <= 0.0).all()
    pa, pb = track_pedal_position(
        on.effort_pct, period_s=DT, track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD
    )
    assert int(np.count_nonzero((pa > AD + 0.05) & (pb > BD + 0.05))) == 0


def test_plan_first_band_row_ff_is_near_bk() -> None:
    # 区間の最初の行の FF は ref(t_go)（≒ 0）から計算される → bk（動き出す開度）付近。
    # 上りと同じ式で「− 傾き ÷ c」が引かれる（傾き 1.6 km/h/s ÷ c 6.8 ≒ 0.24%）ぶんだけ浅い
    bk = BD + abs(BK_U)
    ivs = _bands(_weak, 16.0)
    ref_at = _weak
    pts = [(0.0, bk), (5.0, bk - 3.0)]  # 車速 0 で bk、上がるほど浅い保持の開度
    c = 6.8

    def ff(t: float) -> float:
        return lead_in_ff_at(ref_at, t, LAG, DT, pts, c, BD, 18.0, 0.0)

    t0 = ivs[0].start_s
    assert ff(t0) == pytest.approx(bk - 1.6 / c, abs=0.1)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), ff)
    on = _plan(_weak, spec)
    i0 = int(round(t0 / DT))
    assert -on.effort_pct[i0] == pytest.approx(bk - 1.6 / c, abs=0.1)


def test_plan_band_end_gap_not_tightened_by_launch_gap_tight() -> None:
    # 2026-11-01 修正 2: 帯の終わり → アクセルのすき間は launch_gap_tight on でも off と同じ行数
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    a, b = _plan(_weak, spec, launch_gap_tight=False), _plan(_weak, spec, launch_gap_tight=True)
    assert np.array_equal(a.effort_pct, b.effort_pct)
    assert b.return_gap.n_violations == 0


def test_plan_strong_launch_gap_still_tightened() -> None:
    # 強い発進（帯なし）は今どおり launch_gap_tight で詰まる（帯の入力を渡しても同じ）
    def strong(t: float) -> float:
        return _strong_launch(t - 5.0)

    spec = LaunchBandSpec((), lambda t: 8.5)
    on = _plan(strong, spec, launch_gap_tight=True)
    assert on.return_gap.n_violations == 0
    ref = plan_for(strong, 16.0, _tb(), **dict(_PLAN_KW, launch_gap_tight=True))
    assert np.array_equal(ref.effort_pct, on.effort_pct)


# ───────── 2026-11-07 段16 B-2: 帯の発進のずらし（band_shift） ─────────


def _band_base_plan():
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    return _plan(_weak, spec, predict_low_speed_sides=True), ivs


def _shift(base, **kw):
    return apply_launch_shift(
        base, _weak, _tb(), stop_ref_at=None, accel_min_pct=AD, brake_min_pct=BD,
        track=PedalTrack(), **kw,
    )


def test_band_shift_false_is_identical_to_default() -> None:
    base, _ = _band_base_plan()
    a = _shift(base)
    b = _shift(base, band_shift=False)
    assert a.launch_shift.deltas == b.launch_shift.deltas == [0]
    assert a.launch_shift.band_events == []
    assert np.array_equal(a.effort_pct, b.effort_pct)
    assert a.launch_shift.summary() == b.launch_shift.summary()


def test_band_shift_true_moves_only_rows_after_band_end() -> None:
    base, ivs = _band_base_plan()
    band_rows = np.flatnonzero(base.launch_band)
    a = int(band_rows[-1]) + 1  # 帯の終わり（印の最後の行の次）
    res = _shift(base, band_shift=True, deltas=[3])
    assert res.launch_shift.deltas == [3]
    assert res.launch_shift.band_events == [(a, 3)]
    assert "帯の発進 1 回・δ 3 行×1" in res.launch_shift.summary()
    # 帯の始まり〜帯の最後の行までは動かない
    assert np.array_equal(res.effort_pct[:a], base.effort_pct[:a])
    # 空いた 3 行は帯の最後のブレーキの値・帯の印も 3 行伸びる
    assert np.all(res.effort_pct[a : a + 3] == base.effort_pct[a - 1])
    assert int(res.launch_band.sum()) == int(base.launch_band.sum()) + 3
    assert np.array_equal(np.flatnonzero(res.launch_band)[: len(band_rows)], band_rows)
    # アクセルの始まりが 3 行後ろへずれる
    first = lambda x: int(np.flatnonzero(x.effort_pct > 0)[0])  # noqa: E731
    assert first(res) - first(base) == 3


def test_band_shift_chosen_delta_is_in_range_and_band_events_match_deltas() -> None:
    base, _ = _band_base_plan()
    res = _shift(base, band_shift=True)
    (a, d) = res.launch_shift.band_events[0]
    assert res.launch_shift.deltas == [d] and 0 <= d <= 8 and a > 0


def test_launch_dev_peak_v0_starts_integration_at_reference() -> None:
    from tests.research import fwd_plan as fp  # noqa: PLC0415

    n = 30
    ref = np.full(n, 5.0)
    pa = np.full(n, AD)
    pb = np.full(n, BD)
    args = (_tb(), ref, pa, pb, PedalTrack(), AD, BD, DT, None, None, 7.5, 0, 0, 5)
    zero = fp._launch_dev_peak(*args)
    from_ref = fp._launch_dev_peak(*args, v0=5.0)
    assert zero >= 5.0 - 1e-9 and from_ref < 5.0  # 既定 v0 ＝ 0 は今と同じ
