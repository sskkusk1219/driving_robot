"""停車前の踏み増し（2026-11-03 ProblemReport_20261103_停車前の這う基準 段1）のテスト。
fwd_plan._stop_tail_ramp（区間・狙い・書き換え）、build_plan の通し、mode_drive の P の向きの絞り・
plan args、config の既定値と検証。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import mode_drive as mdmod
from tests.research.fwd_plan import (
    FLAG_COAST,
    FLAG_MEASURED,
    FLAG_STOP,
    StopTailStats,
    _stop_tail_ramp,
)
from tests.research.test_research_fwd_plan import BK, LAG, STOP_PCT, _bk_table, plan_for
from tests.research.test_research_launch_band import _band_cfg, _mode_weak

DT = 0.05
N = 200
HOLD = 9.0  # 停車保持（単体テストの値）
BKP = 8.0  # bk
KW = dict(period_s=DT, bk_pct=BKP, hold_pct=HOLD, short_stop_s=1.0, tail_kmh=0.5, brake_min_pct=3.0)


def _series(stop_rows: tuple[int, int], lead_rows: int, plan_pct: float = 4.0):
    """停車 [a, b) を持つ系列。stop_now は停車の外が 5・中が 0、L 先は lead_rows 行先。"""
    a, b = stop_rows
    stop_now = np.full(N, 5.0)
    stop_now[a:b] = 0.0
    lead = np.full(N, 5.0)
    lead[: N - lead_rows] = stop_now[lead_rows:]
    effort = np.full(N, -plan_pct)
    flag = np.full(N, FLAG_MEASURED, dtype=int)
    effort[a:b] = -STOP_PCT
    flag[a:b] = FLAG_STOP
    return effort, flag, stop_now, lead


def test_long_stop_ramps_linearly_to_stop_hold() -> None:
    effort, flag, now, lead = _series((100, 140), lead_rows=20)  # 停車 2.0 s ＞ 境目 1.0 s
    rows = _stop_tail_ramp(effort, flag, now, lead, **KW)
    assert np.flatnonzero(rows)[0] == 80 and np.flatnonzero(rows)[-1] == 99
    assert effort[99] == pytest.approx(-HOLD)  # 停車保持の手前の行で狙いに着く
    u0 = 4.0
    assert effort[89] == pytest.approx(-(u0 + (HOLD - u0) * 10 / 20))  # 直線
    assert effort[79] == pytest.approx(-4.0) and effort[100] == pytest.approx(-STOP_PCT)
    assert np.all(flag[80:100] == FLAG_MEASURED)


def test_short_stop_ramps_to_bk() -> None:
    effort, flag, now, lead = _series((100, 140), lead_rows=20)
    _stop_tail_ramp(effort, flag, now, lead, **{**KW, "short_stop_s": 5.0})  # 2.0 s ≤ 5.0 s
    assert effort[99] == pytest.approx(-BKP)


def test_plan_deeper_rows_are_kept() -> None:
    effort, flag, now, lead = _series((100, 140), lead_rows=20)
    effort[85:90] = -15.0
    _stop_tail_ramp(effort, flag, now, lead, **KW)
    assert np.all(effort[85:90] == -15.0)
    assert effort[99] == pytest.approx(-HOLD)


def test_start_moves_with_lag() -> None:
    first = []
    for lag_rows in (10, 20):
        effort, flag, now, lead = _series((100, 140), lead_rows=lag_rows)
        rows = _stop_tail_ramp(effort, flag, now, lead, **KW)
        first.append(int(np.flatnonzero(rows)[0]))
    assert first == [90, 80]  # L ÷ 周期 行ぶん早く始まる


def test_stop_lead_rows_are_not_in_interval_and_end_there() -> None:
    effort, flag, now, lead = _series((100, 140), lead_rows=20)
    flag[95:100] = FLAG_STOP  # 停A: 停車保持を 5 行早めた
    effort[95:100] = -STOP_PCT
    rows = _stop_tail_ramp(effort, flag, now, lead, **KW)
    assert np.flatnonzero(rows)[-1] == 94
    assert effort[94] == pytest.approx(-HOLD)


def test_launching_region_is_excluded() -> None:
    effort, flag, now, lead = _series((100, 140), lead_rows=20)
    now[60:100] = 0.3  # 這っている（停車速度より上）
    lead[60:100] = 0.45  # L 先は今より上がる ＝ 発進へ向かう
    before = effort.copy()
    rows = _stop_tail_ramp(effort, flag, now, lead, **KW)
    assert not rows.any() and np.array_equal(effort, before)


def test_leading_stop_of_series_is_skipped() -> None:
    effort, flag, now, lead = _series((0, 40), lead_rows=20)
    before = effort.copy()
    assert not _stop_tail_ramp(effort, flag, now, lead, **KW).any()
    assert np.array_equal(effort, before)


def test_does_not_reach_back_before_previous_stop() -> None:
    effort, flag, now, lead = _series((100, 140), lead_rows=100)
    now[20:40] = 0.0  # 前の停車
    flag[20:40] = FLAG_STOP
    effort[20:40] = -STOP_PCT
    lead[:] = 0.0  # どこも L 先は 0
    lead[20:40] = 0.0
    before = effort[20:40].copy()
    rows = _stop_tail_ramp(effort, flag, now, lead, **KW)
    assert rows[40] and not rows[20:40].any()  # 後の停車の区間は前の停車の次の行から
    assert np.array_equal(effort[20:40], before)


def test_accel_and_coast_rows_become_brake_with_fb() -> None:
    effort, flag, now, lead = _series((100, 140), lead_rows=20)
    effort[80:90] = 5.0  # アクセル
    effort[90:100] = 0.0  # 惰行
    flag[90:100] = FLAG_COAST
    stats = StopTailStats()
    rows = _stop_tail_ramp(effort, flag, now, lead, stats=stats, **KW)
    assert rows[80:100].all() and np.all(effort[80:100] < 0.0)
    assert np.all(flag[80:100] == FLAG_MEASURED)
    assert effort[80] == pytest.approx(-(3.0 + (HOLD - 3.0) / 20))  # u0 ＝ brake min（アクセル行）
    assert stats.intervals == 1 and stats.rows == 20 and stats.mean_length_s == pytest.approx(1.0)
    assert stats.deepen_max_pct > 0.0 and "区間 1 本" in stats.summary()


# ---- build_plan の通し ----

def _ref_decel_stop(stop_s: float):
    """5 km/h から 1 km/h/s で 0 へ（t=2〜7）→ 停止 stop_s → 2 km/h/s で発進。"""
    def ref(t: float) -> float:
        if t < 2.0:
            return 5.0
        if t < 7.0:
            return 5.0 - (t - 2.0)
        t_go = 7.0 + stop_s
        return 0.0 if t <= t_go else min(2.0 * (t - t_go), 20.0)

    return ref


def test_build_plan_off_is_identical_and_on_marks_brake_rows() -> None:
    ref = _ref_decel_stop(12.0)
    kw = dict(**LAG, launch_release_ramp=True)
    base = plan_for(ref, 24.0, _bk_table(), **kw)
    none = plan_for(ref, 24.0, _bk_table(), stop_tail_kmh=None, **kw)
    assert base.stop_tail is None and base.stop_tail_stats is None
    assert np.array_equal(base.effort_pct, none.effort_pct)
    assert np.array_equal(base.flag, none.flag)
    on = plan_for(ref, 24.0, _bk_table(), stop_tail_kmh=0.5, **kw)
    assert on.stop_tail is not None and on.stop_tail.any()
    assert np.all(on.effort_pct[on.stop_tail] < 0.0)
    assert on.stop_tail_stats is not None and on.stop_tail_stats.intervals == 1
    first = int(np.flatnonzero(on.stop_tail)[0])
    # 基準が 0.5 km/h 未満になる t = 6.5 の L（1.0 s）手前から ＝ t = 5.5
    assert first * on.period_s == pytest.approx(5.5, abs=2 * on.period_s)
    # 長い停車: 停車保持の最初の行の手前は停車保持の開度
    i1 = int(np.flatnonzero(on.flag == FLAG_STOP)[0])
    assert on.effort_pct[i1 - 1] == pytest.approx(-STOP_PCT)
    # 区間より前は変わらない
    assert np.array_equal(on.effort_pct[: first - 1], base.effort_pct[: first - 1])
    # 短い停車は bk へ
    short = plan_for(_ref_decel_stop(2.0), 14.0, _bk_table(), stop_tail_kmh=0.5, **kw)
    j1 = int(np.flatnonzero(short.flag == FLAG_STOP)[0])
    assert short.effort_pct[j1 - 1] == pytest.approx(-BK)


def test_build_plan_requires_launch_release_ramp_and_meta() -> None:
    ref = _ref_decel_stop(12.0)
    with pytest.raises(ValueError):
        plan_for(ref, 24.0, _bk_table(), stop_tail_kmh=0.5, **LAG)
    from tests.research.test_research_fwd_plan import linear_table

    with pytest.raises(ValueError):
        plan_for(ref, 24.0, linear_table(), stop_tail_kmh=0.5, launch_release_ramp=True, **LAG)


# ---- mode_drive ----

def test_clip_stop_tail_delta_keeps_only_deeper_direction() -> None:
    assert mdmod.clip_stop_tail_delta(0.4) == 0.0  # ブレーキの行で正 ＝ 浅くする向き
    assert mdmod.clip_stop_tail_delta(-0.4) == -0.4
    assert mdmod.clip_stop_tail_delta(0.0) == 0.0


def test_plan_args_pass_stop_tail_only_when_enabled(tmp_path: Path) -> None:
    cfg = _band_cfg(tmp_path)
    cfg.forward_map.launch_brake_band = False
    cfg.forward_map.stop_tail_ramp_enabled = False  # 本体 yaml は on（2026-11-03 採用）
    mode = _mode_weak()
    ff = mdmod.load_feedforward(cfg)
    _, _, kw = mdmod.forward_map_plan_args(cfg, mode, stop_end=False, table=ff.table)
    assert kw["stop_tail_kmh"] is None
    cfg.forward_map.stop_tail_ramp_enabled = True
    cfg.forward_map.stop_tail_kmh = 0.4
    _, _, kw = mdmod.forward_map_plan_args(cfg, mode, stop_end=False, table=ff.table)
    assert kw["stop_tail_kmh"] == pytest.approx(0.4)


# ---- config ----

def test_config_defaults_and_validate() -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.stop_tail_ramp_enabled is False and fm.stop_tail_kmh == 0.5
    cfg = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    assert cfgmod.validate_config(cfg) == []
    cfg.forward_map.launch_release_ramp = False
    cfg.forward_map.stop_tail_ramp_enabled = True
    assert any("stop_tail_ramp_enabled" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.launch_release_ramp = True
    assert not any("stop_tail" in p for p in cfgmod.validate_config(cfg))
    for bad in (0.0, -1.0, float("nan"), float("inf")):
        cfg.forward_map.stop_tail_kmh = bad
        assert any("stop_tail_kmh" in p for p in cfgmod.validate_config(cfg)), bad
