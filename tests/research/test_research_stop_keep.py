"""停車へ向かう所でブレーキを戻さない（2026-11-04 ProblemReport_20261103_停車前の這う基準 段5）の
テスト。fwd_plan._stop_approach_keep（区間・浅くしない）、build_plan の通し（tail との組み合わせ・
off で今と同じ）、mode_drive の走行中の判定 hold_stop_keep_brake・plan args、
config の既定値と検証。"""

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
    _stop_approach_keep,
)
from tests.research.test_research_fwd_plan import LAG, STOP_PCT, _bk_table, plan_for
from tests.research.test_research_launch_band import _band_cfg, _mode_weak
from tests.research.test_research_stop_tail import _ref_decel_stop

DT = 0.05
N = 200
KEEP = 1.0
KW = dict(period_s=DT, keep_kmh=KEEP, brake_min_pct=3.0)


def _series(a: int = 100, b: int = 140, start: int = 80):
    """停車 [a, b)。L 先の基準は start 行から 1.0 以下（先頭で 0.9、停車の手前で 0.2 へ下がる）。"""
    stop_now = np.full(N, 5.0)
    stop_now[a:b] = 0.0
    lead = np.full(N, 5.0)
    lead[start:a] = np.linspace(0.9, 0.2, a - start)
    lead[a:b] = 0.0
    effort = np.full(N, -4.0)
    flag = np.full(N, FLAG_MEASURED, dtype=int)
    effort[a:b] = -STOP_PCT
    flag[a:b] = FLAG_STOP
    return effort, flag, stop_now, lead


def test_zone_starts_where_lead_ref_is_below_keep_and_ends_before_stop_hold() -> None:
    effort, flag, now, lead = _series()
    rows = _stop_approach_keep(effort, flag, now, lead, **KW)
    idx = np.flatnonzero(rows)
    assert idx[0] == 80 and idx[-1] == 99
    lead2 = lead.copy()
    lead2[80] = 1.2  # 1 km/h を超える行は区間の外
    effort, flag, now, _ = _series()
    rows = _stop_approach_keep(effort, flag, now, lead2, **KW)
    assert np.flatnonzero(rows)[0] == 81
    flag[95:100] = FLAG_STOP  # 停A
    effort[95:100] = -STOP_PCT
    rows = _stop_approach_keep(effort, flag, now, lead, **KW)
    assert np.flatnonzero(rows)[-1] == 94


def test_launching_region_and_leading_stop_are_excluded() -> None:
    effort, flag, now, lead = _series()
    now[60:100] = 0.3
    lead[60:100] = 0.45  # 発進へ向かう
    before = effort.copy()
    assert not _stop_approach_keep(effort, flag, now, lead, **KW).any()
    assert np.array_equal(effort, before)
    effort, flag, now, lead = _series(0, 40, start=0)
    before = effort.copy()
    assert not _stop_approach_keep(effort, flag, now, lead, **KW).any()
    assert np.array_equal(effort, before)


def test_ff_never_gets_shallower_and_deeper_plan_rows_stay() -> None:
    effort, flag, now, lead = _series()
    effort[80:100] = np.linspace(-8.69, -7.62, 20)  # 戻していく FF
    effort[90] = -12.0  # 計画の方が深い
    rows = _stop_approach_keep(effort, flag, now, lead, **KW)
    seg = -effort[80:100]
    assert np.all(np.diff(seg) >= -1e-12)
    assert effort[90] == -12.0 and np.all(effort[91:100] == -12.0)
    assert effort[80] == pytest.approx(-8.69) and rows[80:100].all()


def test_entry_row_is_compared_with_the_row_before_zone() -> None:
    effort, flag, now, lead = _series()
    effort[79] = -8.69
    effort[80:100] = -8.47  # 入りの行の FF はもう戻している
    _stop_approach_keep(effort, flag, now, lead, **KW)
    assert np.all(effort[80:100] == pytest.approx(-8.69))


def test_accel_and_coast_rows_become_brake_with_measured_flag() -> None:
    effort, flag, now, lead = _series()
    effort[79] = 5.0  # 1 つ前がアクセル -> min
    effort[80:90] = 5.0
    effort[90:100] = 0.0
    flag[90:100] = FLAG_COAST
    stats = StopTailStats()
    rows = _stop_approach_keep(effort, flag, now, lead, stats=stats, **KW)
    assert rows[80:100].all() and np.all(effort[80:100] == pytest.approx(-3.0))
    assert np.all(flag[80:100] == FLAG_MEASURED)
    assert stats.intervals == 1 and stats.rows == 20 and stats.mean_length_s == pytest.approx(1.0)
    assert stats.deepen_max_pct == pytest.approx(3.0) and "区間 1 本" in stats.summary()


# ---- build_plan の通し ----

def _kw():
    return dict(**LAG, launch_release_ramp=True)


def test_build_plan_off_is_identical_and_on_keeps_brake_deep() -> None:
    ref = _ref_decel_stop(12.0)
    base = plan_for(ref, 24.0, _bk_table(), stop_tail_kmh=0.5, **_kw())
    none = plan_for(ref, 24.0, _bk_table(), stop_tail_kmh=0.5, stop_keep_kmh=None, **_kw())
    assert base.stop_keep is None and base.stop_keep_stats is None
    assert np.array_equal(base.effort_pct, none.effort_pct)
    assert np.array_equal(base.flag, none.flag)
    on = plan_for(ref, 24.0, _bk_table(), stop_tail_kmh=0.5, stop_keep_kmh=1.0, **_kw())
    assert on.stop_keep is not None and on.stop_keep.any()
    assert np.all(on.effort_pct[on.stop_keep] < 0.0)
    assert on.stop_keep_stats is not None and on.stop_keep_stats.intervals == 1
    first = int(np.flatnonzero(on.stop_keep)[0])
    # 基準が 1 km/h 以下になる t = 6.0 の L（1.0 s）手前から ＝ t = 5.0
    assert first * on.period_s == pytest.approx(5.0, abs=2 * on.period_s)
    # 区間の外は同じ
    assert np.array_equal(on.effort_pct[: first - 1], base.effort_pct[: first - 1])
    # tail の直線は保った値の上に来る（tail は keep の後に掛かる: 深さは単調）
    seg = -on.effort_pct[first : int(np.flatnonzero(on.flag == FLAG_STOP)[0])]
    assert np.all(np.diff(seg) >= -1e-9)
    assert np.all(seg >= -base.effort_pct[first : first + len(seg)] - 1e-9)
    i1 = int(np.flatnonzero(on.flag == FLAG_STOP)[0])
    assert on.effort_pct[i1 - 1] == pytest.approx(-STOP_PCT)


def test_build_plan_keep_requires_stop_tail() -> None:
    with pytest.raises(ValueError):
        plan_for(_ref_decel_stop(12.0), 24.0, _bk_table(), stop_keep_kmh=1.0, **_kw())


# ---- mode_drive ----

def _hold(prev, new, **over):
    kw = dict(in_zone=True, first=False, speed_kmh=1.0, ref_kmh=1.0, release_dev_kmh=0.5)
    kw.update(over)
    return mdmod.hold_stop_keep_brake(prev, new, **kw)


def test_hold_does_not_make_brake_shallower() -> None:
    assert _hold(8.0, 7.0) == (8.0, True, False)
    assert _hold(8.0, 9.0) == (9.0, False, False)  # 深い向きはそのまま
    assert _hold(8.0, 8.0) == (8.0, False, False)


def test_hold_releases_when_moving_and_too_slow() -> None:
    assert _hold(8.0, 7.0, speed_kmh=0.4, ref_kmh=1.0) == (7.0, False, True)  # 偏差 −0.6
    assert _hold(8.0, 7.0, speed_kmh=0.6, ref_kmh=1.0) == (8.0, True, False)  # 偏差 −0.4
    assert _hold(8.0, 7.0, speed_kmh=0.5, ref_kmh=1.2) == (7.0, False, True)  # 偏差 −0.7


def test_hold_does_not_release_when_stopped() -> None:
    from tests.research.ff_model import STOP_SPEED_KMH

    got = _hold(8.0, 7.0, speed_kmh=STOP_SPEED_KMH, ref_kmh=STOP_SPEED_KMH + 1.0)
    assert got == (8.0, True, False)  # 止まっていたら戻さない


def test_hold_first_cycle_and_outside_zone() -> None:
    assert _hold(8.0, 7.0, first=True) == (7.0, False, False)
    assert _hold(None, 7.0) == (7.0, False, False)
    assert _hold(8.0, 7.0, in_zone=False) == (7.0, False, False)


def test_plan_args_pass_stop_keep_only_when_enabled(tmp_path: Path) -> None:
    cfg = _band_cfg(tmp_path)
    cfg.forward_map.launch_brake_band = False
    cfg.forward_map.stop_tail_ramp_enabled = True  # 本体 yaml は on だが明示する
    cfg.forward_map.stop_approach_no_release = False
    mode = _mode_weak()
    ff = mdmod.load_feedforward(cfg)
    _, _, kw = mdmod.forward_map_plan_args(cfg, mode, stop_end=False, table=ff.table)
    assert kw["stop_keep_kmh"] is None
    cfg.forward_map.stop_approach_no_release = True
    cfg.forward_map.stop_approach_no_release_kmh = 0.8
    _, _, kw = mdmod.forward_map_plan_args(cfg, mode, stop_end=False, table=ff.table)
    assert kw["stop_keep_kmh"] == pytest.approx(0.8)


# ---- config ----

def test_config_defaults_and_validate() -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.stop_approach_no_release is False
    assert fm.stop_approach_no_release_kmh == 1.0
    assert fm.stop_approach_release_dev_kmh == 0.5
    cfg = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    cfg.forward_map.stop_tail_ramp_enabled = True
    cfg.forward_map.launch_release_ramp = True
    cfg.forward_map.stop_approach_no_release = True
    assert not any("stop_approach" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.stop_tail_ramp_enabled = False
    assert any("stop_approach_no_release" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.stop_tail_ramp_enabled = True
    for key in ("stop_approach_no_release_kmh", "stop_approach_release_dev_kmh"):
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            old = getattr(cfg.forward_map, key)
            setattr(cfg.forward_map, key, bad)
            assert any(key in p for p in cfgmod.validate_config(cfg)), (key, bad)
            setattr(cfg.forward_map, key, old)
    cfg.forward_map.stop_tail_kmh = 0.5
    cfg.forward_map.stop_approach_no_release_kmh = 0.4  # stop_tail_kmh 未満は NG
    assert any("stop_tail_kmh 以上" in p for p in cfgmod.validate_config(cfg))
