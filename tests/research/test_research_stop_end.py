"""C15（停A 停車保持を早める・停C 停車まぎわの P）のテスト（2026-10-20 段4-2）。

stop_end.py と fwd_plan.build_plan(stop_lead) を見る。"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research.creep_floor import StopArrivals
from tests.research.ff_candidate import CoastAssistFeedforward, make_candidate
from tests.research.ff_model import STOP_SPEED_KMH
from tests.research.fwd_plan import FLAG_STOP, build_plan
from tests.research.stop_end import StopEndP, StopLeadStats
from tests.research.test_research_fwd_plan import AD, BD, STOP_PCT, linear_table

BRAKE_LAG_S = 0.6
HORIZON_S = 0.5


def _ref(t: float) -> float:
    """10 km/h から 2 km/h/s で 5 s に停車 → 8 s まで停車 → 2 km/h/s で発進。"""
    if t <= 5.0:
        return 10.0 - 2.0 * t
    return 0.0 if t <= 8.0 else min(2.0 * (t - 8.0), 10.0)


def _plan(stop_lead: bool):
    return build_plan(
        _ref, 11.0, linear_table(),
        accel_deadband_pct=AD, brake_deadband_pct=BD, stop_brake_opening_pct=STOP_PCT,
        stop_horizon_s=HORIZON_S, brake_lag_s=BRAKE_LAG_S, return_gap_enabled=False,
        stop_lead=stop_lead,
    )


def _first_last_stop(plan) -> tuple[int, int]:
    rows = np.flatnonzero(plan.flag == FLAG_STOP)
    return int(rows[0]), int(rows[-1])


def test_stop_lead_enters_stop_hold_brake_lag_earlier_and_keeps_launch_exit() -> None:
    base, lead = _plan(False), _plan(True)
    b_first, b_last = _first_last_stop(base)
    l_first, l_last = _first_last_stop(lead)
    # 入る時刻: 基準が停車速度以下になる時刻 → ブレーキの遅れ先の基準が停車速度以下の時刻
    assert (b_first - l_first) * base.period_s == pytest.approx(BRAKE_LAG_S, abs=base.period_s)
    assert _ref(lead.t_s[l_first] + BRAKE_LAG_S) <= STOP_SPEED_KMH
    # 発進で抜ける時刻は変わらない
    assert l_last == b_last
    # 早めた行は停A の印が付き、停車保持の開度が出る。それ以外の行は停A が無い列と同じ
    mask = lead.stop_lead
    assert mask is not None and int(mask.sum()) == b_first - l_first
    assert np.all(lead.effort_pct[mask] == -STOP_PCT)
    assert np.array_equal(lead.effort_pct[~mask], base.effort_pct[~mask])
    assert base.stop_lead is not None and not base.stop_lead.any()


def test_stop_lead_off_is_identical_to_default() -> None:
    off = _plan(False)
    default = build_plan(
        _ref, 11.0, linear_table(),
        accel_deadband_pct=AD, brake_deadband_pct=BD, stop_brake_opening_pct=STOP_PCT,
        stop_horizon_s=HORIZON_S, brake_lag_s=BRAKE_LAG_S, return_gap_enabled=False,
    )
    assert np.array_equal(off.effort_pct, default.effort_pct)
    assert np.array_equal(off.flag, default.flag)


# 基準: 0 → 10 km/h、10 → 3 → 0 と停車（20 s）、30 s から再発進
_TIMES = [0.0, 5.0, 10.0, 18.0, 20.0, 30.0, 35.0]
_SPEEDS = [0.0, 10.0, 10.0, 3.0, 0.0, 0.0, 10.0]


def _end_p(top: float | None = 7.5, lag: float | None = 0.6) -> StopEndP:
    return StopEndP(
        top_kmh=top, lead_s=5.0, brake_lag_s=lag, hold_pct=17.88, brake_min_pct=7.0,
        arrivals=StopArrivals(_TIMES, _SPEEDS),
    )


def test_tau_is_remaining_time_but_not_shorter_than_brake_lag() -> None:
    p = _end_p()
    assert p.tau_s(16.0, 4.5, False) == pytest.approx(4.0)  # 20 s に 0 になるまで 4.0 s
    assert p.tau_s(19.0, 1.0, False) == pytest.approx(1.0)
    assert p.tau_s(19.7, 0.3, False) == pytest.approx(0.6)  # 残り 0.3 s はブレーキの遅れ 0.6 s


def test_tau_only_in_the_section() -> None:
    p = _end_p()
    assert p.tau_s(14.0, 6.0, False) is None  # 次の停車まで 6 s > 5 s
    assert p.tau_s(16.0, 8.0, False) is None  # 基準が停止側の表の上の車速より速い
    assert p.tau_s(19.0, 1.0, True) is None  # 停車保持の行（停A で入れた行も同じ）は除く
    assert p.tau_s(25.0, 0.0, False) is None  # 停車中で次の停車は先
    assert p.tau_s(40.0, 5.0, False) is None  # 次の停車が無い


def test_clamp_keeps_brake_between_min_and_stop_hold_and_never_goes_to_accel() -> None:
    p = _end_p()
    assert p.clamp(-12.0, -10.0) == pytest.approx(-12.0)  # 範囲内はそのまま
    assert p.clamp(-3.0, -10.0) == pytest.approx(-7.0)  # min より浅くしない
    assert p.clamp(+5.0, -10.0) == pytest.approx(-7.0)  # アクセル側へ移らない
    assert p.clamp(-30.0, -10.0) == pytest.approx(-17.88)  # 停車保持より深くしない
    assert p.clamp(-30.0, -20.0) == pytest.approx(-20.0)  # 計画がすでに深ければその深さまで


def test_disabled_without_meta_gives_no_section() -> None:
    assert _end_p(top=None).tau_s(19.0, 1.0, False) is None
    assert _end_p(lag=None).tau_s(19.0, 1.0, False) is None
    assert "停Cなし" in _end_p(top=None).settings_text()


def test_stats() -> None:
    lead = StopLeadStats(period_s=0.5)
    for a in (False, True, True, False, True):
        lead.add(a)
    assert (lead.entries, lead.cycles) == (2, 3)
    p = _end_p()
    for active, d in ((False, 0.0), (True, 1.0), (True, -3.0), (False, 0.0)):
        p.stats.add(active, d)
    s = p.stats
    assert (s.entries, s.cycles) == (1, 2)
    assert s.delta_sum_pct == pytest.approx(-2.0) and s.delta_abs_max_pct == pytest.approx(3.0)
    assert "平均 -1.00%" in s.summary() and "最大" in s.summary()


def test_c15_is_a_c12_and_config_knows_it() -> None:
    ff = make_candidate("C15")
    assert isinstance(ff, CoastAssistFeedforward) and ff.candidate == "C15"
    assert ff.expected_kind == "model" and ff.stop_lead_enabled and ff.stop_end_p_enabled
    ff.set_stop_end(lead_enabled=False, p_enabled=False, lead_s=3.0)
    assert not ff.stop_lead_enabled and not ff.stop_end_p_enabled and ff.stop_end_lead_s == 3.0
    assert "C15" in cfgmod.CANDIDATE_NAMES
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.forward_map.stop_lead_enabled is True and cfg.forward_map.stop_end_p_enabled is True
    cfg.feedforward.candidate = "C15"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))


def test_stop_end_accel_pred_default_false_in_config_and_yaml() -> None:
    # 2026-10-31 段3: 停C の先読みは既定 false・本体 yaml も false
    assert cfgmod.ForwardMapSection().stop_end_accel_pred_enabled is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.forward_map.stop_end_accel_pred_enabled is False
