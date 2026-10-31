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
    assert iv.start_s == pytest.approx(3.05)  # 基準が 0.02 を超える最初の行
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
    assert _bands(_weak_launch, ret_s=iv.length_s - LAG + 0.01) == []  # 長さ < 戻る時間 + L
    assert len(_bands(_weak_launch, ret_s=iv.length_s - LAG)) == 1  # ちょうどは残す


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
    assert on.launch_band is not None and on.launch_band[i0:i1].all()
    assert not on.launch_band[:i0].any() and not on.launch_band[i1:].any()
    assert on.launch_prep[i0:i1].all()
    # 離しの時刻: off は動き出しの前（t0 の前の行）に min へ切るが、on は帯の終わり近くまでブレーキ
    last_b_off = int(np.flatnonzero(off.effort_pct[: i1 + 20] < 0.0)[-1])
    last_b_on = int(np.flatnonzero(on.effort_pct[: i1 + 20] < 0.0)[-1])
    assert last_b_off < i0 <= last_b_on
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


# ───────── 設定・mode_drive ─────────


def test_launch_brake_band_default_false_and_config_checks() -> None:
    assert cfgmod.ForwardMapSection().launch_brake_band is False
    cfg = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    assert cfg.forward_map.launch_brake_band is False
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.launch_brake_band = True
    assert cfgmod.validate_config(cfg) == []  # 本体 yaml は依存する設定が揃っている
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
    assert len(bands) == 1 and bands[0].start_s == pytest.approx(3.05)
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


def test_plan_accel_rows_before_band_start_are_replaced_by_bk() -> None:
    # 発進側の表が「基準が動く前からアクセル」と言う計画でも、帯の前の行（t_go の次〜帯の始め）は
    # bk のブレーキで、帯の中にアクセルは無い
    ivs = _bands(_weak, 16.0)
    spec = LaunchBandSpec(tuple((iv.start_s, iv.length_s) for iv in ivs), lambda t: 8.5)
    off = _plan(_weak, table=_tb(2.0))
    on = _plan(_weak, spec, table=_tb(2.0))
    i0 = int(round(ivs[0].start_s / DT))
    i1 = i0 + int(round(ivs[0].length_s / DT))
    i_go = int(round(i0 - LAG / DT))
    assert (off.effort_pct[i_go + 1 : i0] > 0.0).any()  # 前提: off は帯の前にアクセルがある
    assert (on.effort_pct[i_go + 1 : i1] <= 0.0).all()
    assert (on.effort_pct[i_go + 1 : i0] <= -(BD + abs(BK_U)) + 1e-9).all()
    pa, pb = track_pedal_position(
        on.effort_pct, period_s=DT, track=PedalTrack(), accel_min_pct=AD, brake_min_pct=BD
    )
    assert int(np.count_nonzero((pa > AD + 0.05) & (pb > BD + 0.05))) == 0
