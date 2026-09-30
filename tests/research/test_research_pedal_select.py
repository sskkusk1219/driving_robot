"""ペダル選択の部品（pedal_select.py）とペダル予定表（pedal_schedule.py）のテスト。

ProblemReport_20260929 段1。src は import しない。
"""

from __future__ import annotations

import pytest

from tests.research import config as cfgmod
from tests.research.pedal_arbiter import plan_accel_kmhs
from tests.research.pedal_schedule import (
    STOP,
    Switch,
    compute_schedules,
    summarize_schedule,
    switch_timing_diffs,
    to_runs,
)
from tests.research.pedal_select import (
    PEDAL_ACCEL,
    PEDAL_BRAKE,
    PEDAL_COAST,
    point_accel_kmhs,
    select_pedal,
    window_slope_kmhs,
)


def _wavy(t: float) -> float:
    """折れ線の基準車速（傾きが途中で変わる）。"""
    return max(0.0, 2.0 * t) if t < 10.0 else max(0.0, 20.0 - 1.5 * (t - 10.0))


# ── 傾きの計算 ────────────────────────────────────────────────────────


def test_window_slope_of_straight_line_is_the_slope_for_any_window() -> None:
    def line(t: float) -> float:
        return 3.0 + 1.25 * t

    for center, width in ((0.5, 3.0), (0.0, 2.0), (1.5, 6.0)):
        assert window_slope_kmhs(line, 4.0, center, width) == pytest.approx(1.25)
    assert point_accel_kmhs(line, 4.0, 1.0) == pytest.approx(1.25)


def test_window_slope_matches_plan_accel_when_center_is_half_width() -> None:
    for t in (0.0, 3.3, 9.0, 12.7):
        for width in (2.0, 3.0, 5.0):
            assert window_slope_kmhs(_wavy, t, width / 2.0, width) == pytest.approx(
                plan_accel_kmhs(_wavy, t, width)
            )


def test_window_slope_uses_the_window_position() -> None:
    # t=10 中心 0.5・幅 3 → 窓は [9.0, 12.0]。折れ点(10)をまたぐので傾きは 2 と −1.5 の中間
    slope = window_slope_kmhs(_wavy, 10.0, 0.5, 3.0)
    assert -1.5 < slope < 2.0
    # 窓が折れ点より前だけなら 2.0
    assert window_slope_kmhs(_wavy, 2.0, 0.5, 3.0) == pytest.approx(2.0)


def test_point_accel_is_forward_difference() -> None:
    assert point_accel_kmhs(_wavy, 8.0, 1.0) == pytest.approx(2.0)
    assert point_accel_kmhs(_wavy, 10.0, 2.0) == pytest.approx(-1.5)


# ── ペダル選択の規則 ──────────────────────────────────────────────────


def test_select_pedal_boundaries_match_predict_effort() -> None:
    coast, band = -2.0, 0.5
    assert select_pedal(-2.0, coast, band) == PEDAL_COAST  # 中心
    assert select_pedal(-1.51, coast, band) == PEDAL_COAST
    assert select_pedal(-1.5, coast, band) == PEDAL_ACCEL  # 差 = 帯 は帯の外（`<`）
    assert select_pedal(-2.5, coast, band) == PEDAL_BRAKE  # 差 = 帯 は帯の外
    assert select_pedal(-2.49, coast, band) == PEDAL_COAST
    assert select_pedal(3.0, coast, band) == PEDAL_ACCEL
    assert select_pedal(-6.0, coast, band) == PEDAL_BRAKE


def test_select_pedal_zero_band_equal_is_accel() -> None:
    """帯 0（無効）で要求 = 惰行のときは `>=` でアクセル（predict_effort と同じ）。"""
    assert select_pedal(-1.0, -1.0, 0.0) == PEDAL_ACCEL
    assert select_pedal(-1.0001, -1.0, 0.0) == PEDAL_BRAKE


# ── 集計 ──────────────────────────────────────────────────────────────


def test_to_runs_and_summary_counts_switches_and_short_runs() -> None:
    a, c, b = PEDAL_ACCEL, PEDAL_COAST, PEDAL_BRAKE
    # 刻み 0.5s: アクセル 3 区間(1.5s)・惰行 1 区間(0.5s)・アクセル・ブレーキ・停車
    seq = [a, a, a, c, a, a, b, b, STOP, STOP]
    runs = to_runs(seq, 0.5)
    assert [r.value for r in runs] == [a, c, a, b, STOP]
    assert runs[1].duration_s == pytest.approx(0.5)
    s = summarize_schedule(seq, 0.5)
    assert s.switch_count == 3  # a→c, c→a, a→b（b→停車は数えない）
    assert s.direct_accel_brake == 1
    assert s.short_runs == 1
    assert s.short_patterns == {"+1→0→+1": 1}
    assert s.shares[a] == pytest.approx(0.5)
    assert s.shares[STOP] == pytest.approx(0.2)
    # 停車を除いた時間 4s で割る
    assert s.switches_per_s == pytest.approx(3 / 4.0)


def test_summary_short_run_pattern_labels_edges_and_stop() -> None:
    seq = [PEDAL_BRAKE, STOP, STOP, PEDAL_BRAKE, PEDAL_COAST, PEDAL_COAST, PEDAL_COAST]
    s = summarize_schedule(seq, 0.1)
    assert s.short_patterns == {"端→-1→停": 1, "停→-1→0": 1, "-1→0→端": 1}


def test_switch_timing_diffs_pairs_same_direction_one_to_one() -> None:
    base = [Switch(10.0, 1, 0), Switch(20.0, 0, 1), Switch(30.0, 1, -1)]
    other = [Switch(9.95, 1, 0), Switch(20.1, 0, 1), Switch(50.0, 1, -1)]  # 3 つ目は遠すぎ
    diffs = switch_timing_diffs(base, other, window_s=2.0)
    assert diffs == pytest.approx([-0.05, 0.1])
    # 同じ向きの切替が近くに 1 つしか無ければ、2 つの base に同じ other を使い回さない
    assert switch_timing_diffs([Switch(1.0, 1, 0), Switch(1.2, 1, 0)], [Switch(1.1, 1, 0)]) == (
        pytest.approx([0.1])
    )


def test_compute_schedules_marks_stop_and_selects() -> None:
    def ref(t: float) -> float:
        return min(max(0.0, 10.0 * (t - 2.0)), 50.0)  # 2s まで停車 → 加速

    times, a_seq, g_seq = compute_schedules(
        ref, 10.0, lambda v: -2.0, band_kmhs=0.5, point_s=1.0, stop_horizon_s=0.5,
        center_s=0.5, width_s=3.0,
    )
    assert len(times) == len(a_seq) == len(g_seq)
    assert a_seq[0] == g_seq[0] == STOP  # t=0（ref(0.5)=0 も停車）
    i = 30  # t=1.5: ref(t)=0 だが ref(t+0.5)=0 → まだ停車
    assert a_seq[i] == STOP
    i = 100  # t=5.0: 加速中
    assert a_seq[i] == g_seq[i] == PEDAL_ACCEL


# ── config ────────────────────────────────────────────────────────────


def _load_default() -> cfgmod.ResearchConfig:
    return cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)


def test_pedal_select_config_defaults_and_validation() -> None:
    cfg = _load_default()
    assert cfg.feedforward.pedal_select_center_s == pytest.approx(0.5)
    assert cfg.feedforward.pedal_select_width_s == pytest.approx(3.0)
    assert cfg.feedforward.pedal_select_mode == "point"
    assert cfgmod.validate_config(cfg) == []

    for bad in (0.0, -0.5, 3.1):
        cfg.feedforward.pedal_select_point_s = bad
        assert any("pedal_select_point_s" in p for p in cfgmod.validate_config(cfg))
    for ok in (0.5, 1.0, 3.0):
        cfg.feedforward.pedal_select_point_s = ok
        assert cfgmod.validate_config(cfg) == []
    cfg.feedforward.pedal_select_point_s = 1.0

    for bad in (0.0, -0.1, 2.1):
        cfg.feedforward.pedal_select_center_s = bad
        assert any("pedal_select_center_s" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.pedal_select_center_s = 2.0  # 上限は含む
    assert cfgmod.validate_config(cfg) == []

    for bad in (1.9, 6.1):
        cfg.feedforward.pedal_select_width_s = bad
        assert any("pedal_select_width_s" in p for p in cfgmod.validate_config(cfg))
    for ok in (2.0, 6.0):
        cfg.feedforward.pedal_select_width_s = ok
        assert cfgmod.validate_config(cfg) == []


def test_pedal_select_mode_is_validated_and_conflicts_with_reach() -> None:
    cfg = _load_default()
    cfg.feedforward.pedal_select_mode = "window"
    assert cfgmod.validate_config(cfg) == []
    cfg.feedforward.pedal_select_mode = "bogus"
    assert any("pedal_select_mode" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.pedal_select_mode = "window"
    cfg.feedforward.reach_horizons_s = [1.0]
    assert any("reach_horizons_s" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.pedal_select_mode = "point"  # point なら併用可
    assert cfgmod.validate_config(cfg) == []


def test_compute_schedules_a_uses_point_s() -> None:
    """A の先読みは point_s。基準が振動していれば 1s 先と 3s 先で選択が変わる。"""
    import math  # noqa: PLC0415

    def ref(t: float) -> float:
        return 60.0 + 20.0 * math.sin(t)

    kw = dict(band_kmhs=0.5, stop_horizon_s=0.5, center_s=0.5, width_s=3.0)
    _, a1, g1 = compute_schedules(ref, 20.0, lambda v: -2.0, point_s=1.0, **kw)
    _, a3, g3 = compute_schedules(ref, 20.0, lambda v: -2.0, point_s=3.0, **kw)
    assert a1 != a3  # point_s が実際に効いている
    assert g1 == g3  # G（窓）は point_s の影響を受けない


def test_pedal_schedule_main_point_s_overrides_config(capsys, tmp_path) -> None:
    """--point-s は config の pedal_select_point_s を一時的に上書きして表示に出る。"""
    from tests.research import pedal_schedule as psmod  # noqa: PLC0415

    args = ["--mode", "01_WLTP_Low,Mid,Hi,ExHi", "--out", str(tmp_path)]
    assert psmod.main([*args, "--point-s", "2.0"]) == 0
    assert "point=2s" in capsys.readouterr().out
    assert psmod.main(args) == 0
    assert "point=1s" in capsys.readouterr().out
