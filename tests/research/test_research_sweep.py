"""通し掃引（段6c）: 計画（sweep_planner）・走行中の網羅カウンタ・PatternLoop の掃引。"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research.coverage_live import LiveCoverage
from tests.research.ff_model import DEFAULT_FEATURE_SPEC
from tests.research.pattern_loop import (
    GridSweepPattern,
    PatternLoop,
    PatternLoopConfig,
    _Phase,
)
from tests.research.research_types import G_TO_KMHS, PatternKind
from tests.research.sweep_planner import SweepCell, SweepPlan, plan_sweeps
from tests.research.test_research_pattern_loop import FakeAxis, FakeCAN, Recorder
from tests.research.vehicle import build_vehicle_profile
from tests.research.wltp_grid import GridEdges, _histogram, _v0_and_a_req

SPEED_EDGES = [float(v) for v in range(0, 150, 10)]
ACCEL_EDGES = [-14.0, -7.0, -3.0, -1.5, -0.5, 0.5, 1.5, 3.0, 7.0, 14.0]
CAP = 0.3 * G_TO_KMHS  # 上限 G の予測の目標 [km/h/s]（≒ 10.6）


def _means(value: float = -5.0) -> np.ndarray:
    return np.full((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1), value)


# ── 計画 ───────────────────────────────────────────────────────


def test_decel_sweep_covers_holes_and_the_bins_between_from_the_top_down() -> None:
    means = _means()
    means[1, 0], means[2, 0], means[4, 0] = -9.0, -10.0, -8.0
    plans = plan_sweeps([(1, 0), (4, 0)], means, SPEED_EDGES, ACCEL_EDGES, CAP)
    assert len(plans) == 1
    plan = plans[0]
    assert plan.decel and [c.i for c in plan.cells] == [4, 3, 2, 1]  # 高い車速から下る
    assert (plan.start_kmh, plan.end_kmh) == (50.0, 10.0)
    # 穴でない間のビン（3）は、モードの平均（−5）でも列の内側の端（−7）へ頭打ち上げ
    assert plan.cells[1].a_kmhs == pytest.approx(-7.0)
    assert plan.cells[0].a_kmhs == pytest.approx(-8.0)


def test_accel_sweep_goes_from_the_bottom_up() -> None:
    means = _means(9.0)
    plans = plan_sweeps([(0, 8), (2, 8)], means, SPEED_EDGES, ACCEL_EDGES, CAP)
    plan = plans[0]
    assert not plan.decel and [c.i for c in plan.cells] == [0, 1, 2]
    assert (plan.start_kmh, plan.end_kmh) == (0.0, 30.0)


def test_targets_are_capped_at_the_g_limit_and_too_strong_columns_are_skipped() -> None:
    means = _means(-13.0)
    plan = plan_sweeps([(2, 0)], means, SPEED_EDGES, ACCEL_EDGES, CAP)[0]
    assert plan.cells[0].a_kmhs == pytest.approx(-CAP)  # −13 は上限 G（≒ −10.6）で頭打ち
    assert plan_sweeps([(2, 0)], means, SPEED_EDGES, ACCEL_EDGES, 5.0) == []  # 列の内側 7 > 5


def test_cruise_column_is_never_swept() -> None:
    assert plan_sweeps([(2, 4)], _means(0.0), SPEED_EDGES, ACCEL_EDGES, CAP) == []


def test_columns_are_ordered_by_demand() -> None:
    seconds = np.zeros((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1))
    seconds[2, 1], seconds[3, 8] = 3.0, 8.0
    plans = plan_sweeps([(2, 1), (3, 8)], _means(), SPEED_EDGES, ACCEL_EDGES, CAP, seconds)
    assert [p.j for p in plans] == [8, 1]


def test_nan_mean_falls_back_to_the_column_center() -> None:
    means = _means()
    means[2, 0] = np.nan
    plan = plan_sweeps([(2, 0)], means, SPEED_EDGES, ACCEL_EDGES, CAP)[0]
    assert plan.cells[0].a_kmhs == pytest.approx(-10.5)


def test_cell_at_clamps_to_the_nearest_end() -> None:
    plan = SweepPlan(0, True, (SweepCell(4, 0, 40, 50, -8.0), SweepCell(3, 0, 30, 40, -8.0)))
    assert plan.cell_at(45.0).i == 4 and plan.cell_at(35.0).i == 3
    assert plan.cell_at(80.0).i == 4 and plan.cell_at(5.0).i == 3


# ── 走行中の網羅カウンタ（事後の data_cells と同じ数え方） ────────────────


def _speeds() -> np.ndarray:
    t = np.arange(0.0, 60.0, 0.1)
    return np.clip(40.0 + 30.0 * np.sin(t / 7.0) + 8.0 * np.sin(t / 2.3), 0.0, None)


def test_live_coverage_matches_post_hoc_histogram() -> None:
    speed = _speeds()
    t = np.arange(len(speed)) * 0.1
    stamps = [datetime(2000, 1, 1, tzinfo=UTC) + timedelta(seconds=float(s)) for s in t]
    v0, a_req, _ = _v0_and_a_req(speed, stamps, DEFAULT_FEATURE_SPEC)
    expected = _histogram(v0, a_req, GridEdges(tuple(SPEED_EDGES), tuple(ACCEL_EDGES)), 0.1)
    live = LiveCoverage(SPEED_EDGES, ACCEL_EDGES)
    # 事後の数え方は、先頭の過去ホライズン（1.0 s）と末尾の先読み（最大 3.0 s）の行を数えない。
    # 走行中は末尾を 1.0 s 先までしか要らないので、同じ範囲に揃えて比べる
    for k in range(10, len(speed) - 20):
        live.push(float(t[k]), float(speed[k]))
    assert live.seconds.sum() > 30.0
    assert np.allclose(live.seconds, expected, atol=1e-6)


def test_live_coverage_ignores_standstill_and_needs_one_second_lookahead() -> None:
    live = LiveCoverage(SPEED_EDGES, ACCEL_EDGES)
    for k in range(30):
        live.push(k * 0.1, 0.0)
    assert live.seconds.sum() == 0.0  # 停車は数えない
    live2 = LiveCoverage(SPEED_EDGES, ACCEL_EDGES)
    for k in range(9):
        live2.push(k * 0.1, 20.0)
    assert live2.seconds.sum() == 0.0  # 1.0 s 先が来るまで確定しない
    live2.push(1.0, 20.0)
    assert live2.seconds.sum() == pytest.approx(0.1)


def test_live_coverage_holes_use_the_same_rule_as_find_holes() -> None:
    live = LiveCoverage(SPEED_EDGES, ACCEL_EDGES)
    wltp = np.zeros(live.shape)
    wltp[2, 1], wltp[3, 1], wltp[4, 1] = 9.0, 9.0, 1.0  # 4 は需要が小さい
    live.seconds[3, 1] = 5.0  # 3 はデータが足りている
    assert live.holes(wltp, 2.5, 2.0) == [(2, 1)]


# ── PatternLoop の掃引 ─────────────────────────────────────────


def _sweep_pattern(seconds: np.ndarray, means: np.ndarray, **kw: float) -> GridSweepPattern:
    return GridSweepPattern(
        PatternKind.GRID_SWEEP, accel_opening=70.0, brake_opening=0.0, hold_duration_s=3.0,
        wltp_seconds=seconds, wltp_mean=means, speed_edges=tuple(SPEED_EDGES),
        accel_edges=tuple(ACCEL_EDGES), min_s=2.5, data_max_s=2.0,
        max_passes=int(kw.get("passes", 3)),
    )


def _sweep_loop(
    pattern: GridSweepPattern, config: PatternLoopConfig | None = None
) -> tuple[PatternLoop, list[str]]:
    cfg = cfgmod.load_config(Path("tests/research/config_testVehicle.yaml"))
    rec, logs = Recorder(), []
    loop = PatternLoop(
        accel_driver=FakeAxis(), brake_driver=FakeAxis(), can_reader=FakeCAN(0.0),
        profile=build_vehicle_profile(cfg), patterns=[pattern], overcurrent_limit_ma=5000.0,
        on_complete=rec.on_complete, on_emergency=rec.on_emergency, on_sample=rec.on_sample,
        config=config, interval_s=0.02, on_sweep_log=logs.append,
    )
    return loop, logs


def _fill_map(loop: PatternLoop) -> None:
    """G 校正・格子ステップで測れた後の状態（ブレーキ 0.8・アクセル 0.5 km/h/s/%）。"""
    ff = loop._profile.feedforward_params
    for v in range(5, 150, 10):
        loop.glimit.add_coast(float(v), 1.6)
        loop.glimit.add("brake", float(v), ff.brake_deadband_pct + 4.0, 1.6 + 4.0 * 0.8)
        loop.glimit.add("brake", float(v), ff.brake_deadband_pct + 8.0, 1.6 + 8.0 * 0.8)
        loop.glimit.add("accel", float(v), ff.accel_deadband_pct + 6.0, -1.6 + 6.0 * 0.5)
        loop.glimit.add("accel", float(v), ff.accel_deadband_pct + 12.0, -1.6 + 12.0 * 0.5)


def test_sweep_without_holes_is_skipped_and_logged() -> None:
    seconds = np.zeros((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1))
    loop, logs = _sweep_loop(_sweep_pattern(seconds, _means()))
    pattern = loop._patterns[0]
    assert loop._sweep_enter_pattern(pattern, 0.0)  # とばした
    assert loop._phase is _Phase.DONE
    assert any("掃引 0 本" in line for line in logs)


def test_sweep_disabled_when_max_passes_is_zero() -> None:
    seconds = np.full((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1), 9.0)
    loop, logs = _sweep_loop(_sweep_pattern(seconds, _means(), passes=0))
    assert loop._sweep_enter_pattern(loop._patterns[0], 0.0)
    assert any("無効" in line for line in logs)


def test_openings_come_from_the_map_and_are_capped_at_the_fastest_edge_of_each_cell() -> None:
    seconds = np.zeros((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1))
    seconds[3, 0], seconds[2, 0] = 6.0, 6.0
    loop, _ = _sweep_loop(_sweep_pattern(seconds, _means(-9.0)))
    _fill_map(loop)
    plan = plan_sweeps([(3, 0), (2, 0)], _means(-9.0), SPEED_EDGES, ACCEL_EDGES, CAP)[0]
    openings = loop._sweep_open_for(plan)
    assert openings is not None and len(openings) == len(plan.cells)
    ff = loop._profile.feedforward_params
    for accel, brake in openings:
        assert accel == 0.0 and brake > ff.brake_deadband_pct  # 減速はブレーキだけ
    # −9 km/h/s は (9 − 1.6) / 0.8 ≒ 9.25% 不感帯より深い
    assert openings[0][1] == pytest.approx(ff.brake_deadband_pct + 9.25, abs=0.5)


def test_sweep_is_skipped_when_openings_cannot_be_predicted() -> None:
    seconds = np.zeros((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1))
    seconds[3, 0] = 6.0
    loop, logs = _sweep_loop(_sweep_pattern(seconds, _means(-9.0)))  # 地図が空
    assert loop._sweep_enter_pattern(loop._patterns[0], 0.0)
    assert any("予測できない" in line for line in logs)
    assert loop._phase is _Phase.DONE


# ── 掃引の走行（簡易プラントで 1 本を通す） ──────────────────────────────


class _Plant:
    """簡易プラント [km/h/s]。アクセル: a = 0.5 ×（開度 − 不感帯）− 1.6、
    ブレーキ: a = −1.6 − 0.8 ×（開度 − 不感帯）。"""

    def __init__(self, loop: PatternLoop) -> None:
        ff = loop._profile.feedforward_params
        self.adb, self.bdb = ff.accel_deadband_pct, ff.brake_deadband_pct
        self.v = 0.0

    def accel(self, ua: float, ub: float) -> float:
        if ub > self.bdb:
            return -1.6 - 0.8 * (ub - self.bdb)
        if ua > self.adb:
            return 0.5 * (ua - self.adb) - 1.6
        return -1.6 if self.v > 5.0 else 0.0

    def step(self, ua: float, ub: float, dt: float) -> float:
        self.v = max(0.0, self.v + self.accel(ua, ub) * dt)
        return self.v


class _WeakBrakePlant(_Plant):
    """ブレーキが効かない（0.05 km/h/s/%）プラント。狙いのセルに減速が入らず穴が残る。"""

    def accel(self, ua: float, ub: float) -> float:
        if ub > self.bdb:
            return -1.6 - 0.05 * (ub - self.bdb)
        return super().accel(ua, ub)


def _run(loop: PatternLoop, pattern: GridSweepPattern, plant: _Plant, t_end: float) -> float:
    t = 0.0
    loop._sweep_enter_pattern(pattern, t)
    phases: list[str] = []
    while t < t_end and loop._phase is not _Phase.DONE:
        loop._last_speed = plant.v
        if loop.coverage is not None:
            loop.coverage.push(t, plant.v)
        ua, ub = loop._command_openings(loop._patterns[min(loop._pattern_idx, 0)], t)
        loop._current_accel_opening, loop._current_brake_opening = ua, ub
        a = plant.accel(ua, ub)
        loop._advance(loop._patterns[0], plant.v, a, t)
        plant.step(ua, ub, 0.1)
        phases.append(loop._phase.name)
        t = round(t + 0.1, 6)
    _run.phases = phases  # type: ignore[attr-defined]
    return t


def test_decel_sweep_runs_up_sweeps_down_stops_and_fills_the_cell() -> None:
    seconds = np.zeros((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1))
    seconds[3, 1] = 8.0  # 30〜40 km/h × −7〜−3 km/h/s
    means = _means(-5.0)
    pattern = _sweep_pattern(seconds, means, passes=4)
    loop, logs = _sweep_loop(pattern, PatternLoopConfig(coast_timeout_s=200.0))
    _fill_map(loop)
    plant = _Plant(loop)
    t = _run(loop, pattern, plant, 400.0)
    phases = set(_run.phases)  # type: ignore[attr-defined]
    assert {"SWEEP_UP", "SWEEP", "DRIVE_BRAKE"} <= phases
    assert loop._phase is _Phase.DONE, (t, logs)
    assert loop.coverage is not None
    assert loop.coverage.seconds[3, 1] >= 2.0  # 穴が埋まった（回数が足りていれば途中で止まる）
    assert any("回目" in line for line in logs)
    assert not [p for p in loop._sw_plans]


def test_sweep_repeats_until_max_passes_when_the_cell_cannot_be_filled() -> None:
    seconds = np.zeros((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1))
    seconds[3, 1] = 8.0
    pattern = _sweep_pattern(seconds, _means(-5.0), passes=2)
    loop, logs = _sweep_loop(pattern, PatternLoopConfig(coast_timeout_s=200.0))
    _fill_map(loop)
    # 加速度がセルに入らないプラント（ブレーキが弱すぎる）にして、穴が残るようにする
    weak = _WeakBrakePlant(loop)
    _run(loop, pattern, weak, 600.0)
    passes = [line for line in logs if "回目" in line]
    assert len(passes) == 2  # max_passes で打ち切り
    assert loop._phase is _Phase.DONE


def test_accel_sweep_starts_from_standstill_without_a_run_up() -> None:
    seconds = np.zeros((len(SPEED_EDGES) - 1, len(ACCEL_EDGES) - 1))
    seconds[1, 8] = 6.0  # 10〜20 km/h × +7〜+14 km/h/s
    pattern = _sweep_pattern(seconds, _means(9.0), passes=3)
    loop, logs = _sweep_loop(pattern, PatternLoopConfig(coast_timeout_s=200.0))
    _fill_map(loop)
    plant = _Plant(loop)
    _run(loop, pattern, plant, 300.0)
    phases = _run.phases  # type: ignore[attr-defined]
    assert phases[0] == "SWEEP_UP"  # 開始車速（10 km/h）の手前まで G 比例で上がる
    assert "SWEEP" in phases and loop._phase is _Phase.DONE
    assert loop.coverage is not None and loop.coverage.seconds[1, 8] > 0.0
    assert any("加速" in line for line in logs)
