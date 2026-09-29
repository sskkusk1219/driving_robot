"""grid_planner（格子ステップ走行の狙い・開度の決め方）のテスト。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tests.research import grid_planner as gp
from tests.research.grid_planner import (
    GridPlanner,
    GridSettings,
    StationPlan,
    StepKind,
    Target,
)

SPEED_EDGES = [0.0, 10.0, 20.0, 30.0]
# 列: −3〜−1.5 / −1.5〜−0.5 / 定速(±0.5) / 0.5〜1.5 / 1.5〜3
ACCEL_EDGES = [-3.0, -1.5, -0.5, 0.5, 1.5, 3.0]


def _arrays(seconds: list[list[float]], means: list[list[float]]):
    sec = np.array(seconds, dtype=float)
    mean = np.array(means, dtype=float)
    return sec, mean, np.array([1.0, 1.4, 1.6]), np.array([-1.0, -2.0, -2.5])


def _station() -> StationPlan:
    return StationPlan(
        speed_kmh=25.0, speed_lo_kmh=20.0, speed_hi_kmh=30.0,
        decel=(Target(-1.0, -1.5, -0.5), Target(-2.2, -3.0, -1.5)),
        accel=(Target(1.0, 0.5, 1.5), Target(2.0, 1.5, 3.0)),
        a_max_kmhs=2.2, a_min_kmhs=-2.5,
    )


def _planner(**kw: float) -> GridPlanner:
    settings = GridSettings(**kw) if kw else GridSettings()
    return GridPlanner(
        settings, accel_deadband_pct=4.0, brake_deadband_pct=6.0,
        max_accel_pct=40.0, max_brake_pct=50.0,
    )


# ── 計画 ────────────────────────────────────────────────────────


def test_plan_stations_skips_low_band_and_sparse_band_and_orders_targets() -> None:
    nan = float("nan")
    sec, mean, amax, amin = _arrays(
        [[9, 9, 9, 9, 9], [8, 9, 9, 9, 6], [0.5, 0.5, 0.5, 0.5, 0.5]],
        [[-2, -1, 0, 1, 2], [-2.2, -1.0, 0.0, 0.9, 2.1], [nan] * 5],
    )
    plans = gp.plan_stations(sec, mean, amax, amin, SPEED_EDGES, ACCEL_EDGES, GridSettings())
    assert [p.speed_kmh for p in plans] == [15.0]  # 0〜10 は station_min 未満、20〜30 は 5s 未満
    plan = plans[0]
    assert [t.a_kmhs for t in plan.decel] == [-1.0, -2.2]  # 緩い順
    assert [t.a_kmhs for t in plan.accel] == [0.9, 2.1]  # 小さい順
    assert plan.a_max_kmhs == 1.4 and plan.a_min_kmhs == -2.0


def test_plan_stations_excludes_cells_below_wltp_min_and_constant_column() -> None:
    sec, mean, amax, amin = _arrays(
        [[0] * 5, [2, 9, 9, 9, 1], [0] * 5],
        [[float("nan")] * 5, [-2, -1, 0, 1, 2], [float("nan")] * 5],
    )
    (plan,) = gp.plan_stations(sec, mean, amax, amin, SPEED_EDGES, ACCEL_EDGES, GridSettings())
    assert [t.a_kmhs for t in plan.decel] == [-1.0]
    assert [t.a_kmhs for t in plan.accel] == [1.0]


def test_plan_launch_aggregates_low_bands_weighted_by_seconds() -> None:
    edges = [0.0, 10.0, 20.0, 30.0]
    sec = np.array([[0, 0, 0, 6, 0], [0, 0, 0, 4, 0], [0, 0, 0, 50, 0]], dtype=float)
    mean = np.array([[0, 0, 0, 0.6, 0], [0, 0, 0, 1.1, 0], [0, 0, 0, 1.0, 0]], dtype=float)
    plan = gp.plan_launch(
        sec, mean, np.array([1.0, 1.4, 9.0]), np.array([-1.0, -2.0, -9.0]), edges, ACCEL_EDGES,
        GridSettings(),
    )
    assert len(plan.accel) == 1
    assert plan.accel[0].a_kmhs == pytest.approx((6 * 0.6 + 4 * 1.1) / 10)  # 20〜30 は含めない
    assert plan.a_max_kmhs == 1.4  # 0〜20 の最大
    assert plan.a_min_kmhs == -2.0


# ── 純関数 ───────────────────────────────────────────────────────


def test_fit_slope() -> None:
    assert gp.fit_slope([0.0, 1.0, 2.0], [10.0, 12.0, 14.0]) == pytest.approx(2.0)
    assert gp.fit_slope([0.0], [1.0]) == 0.0
    assert gp.fit_slope([1.0, 1.0], [1.0, 2.0]) == 0.0


def test_pi_hold_step_rate_limit_winds_integral_back() -> None:
    base, integral = 4.0, 4.0
    for _ in range(100):  # 目標 +50 km/h の間ずっとレート制限で頭打ち
        base, integral = gp.pi_hold_step(
            base, integral, 60.0, 10.0, kp=0.3, ki=0.05, min_pct=4.0, max_pct=40.0,
            max_rate_pct_s=1.0, dt=0.1,
        )
    assert base == pytest.approx(4.0 + 100 * 0.1)  # レート上限どおりに上がる
    assert integral < 40.0  # 積分が飽和のまま伸び続けない（back-calculation）
    # 目標を超えたら即座に下がる
    lowered, _ = gp.pi_hold_step(
        base, integral, 10.0, 60.0, kp=0.3, ki=0.05, min_pct=4.0, max_pct=40.0,
        max_rate_pct_s=1.0, dt=0.1,
    )
    assert lowered < base


# ── プランナー ────────────────────────────────────────────────────


def _drive_to_targets(planner: GridPlanner) -> None:
    planner.begin_station(_station(), u0_pct=10.0)
    hold = planner.next_step()
    assert hold is not None and hold.kind is StepKind.HOLD and hold.accel_pct == 10.0
    planner.record(hold, 0.0)
    coast = planner.next_step()
    assert coast is not None and coast.kind is StepKind.COAST
    assert (coast.accel_pct, coast.brake_pct) == (0.0, 0.0)
    planner.record(coast, -1.6)


def test_steps_start_with_hold_then_coast_then_decel_then_accel() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    kinds = []
    while (step := planner.next_step()) is not None:
        kinds.append(step.kind)
        planner.record(step, step.target.a_kmhs if step.target else 0.0)
    assert kinds == [
        StepKind.DECEL_ACCEL, StepKind.DECEL_BRAKE, StepKind.ACCEL, StepKind.ACCEL,
    ]  # −1.0 は惰行(−1.6)より緩い→アクセルを絞る、−2.2 は強い→ブレーキ


def test_gentle_decel_interpolates_between_coast_and_hold() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    step = planner.next_step()
    assert step is not None and step.kind is StepKind.DECEL_ACCEL
    # 惰行 −1.6 は不感帯 4%、定速 0 は u0=10%。−1.0 はその 6 割 → 4 + 6 × 0.6/1.6 = 6.25
    assert step.accel_pct == pytest.approx(4.0 + 6.0 * (0.6 / 1.6))
    assert step.brake_pct == 0.0


def test_strong_decel_uses_brake_from_deadband_and_gain() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    planner.record(planner.next_step(), -1.0)  # 緩い減速を 1 つ消化
    step = planner.next_step()
    assert step is not None and step.kind is StepKind.DECEL_BRAKE
    # ブレーキ不感帯 6% + (惰行 −1.6 − 狙い −2.2 の差 0.6) / g_init 2.0
    assert step.brake_pct == pytest.approx(6.0 + 0.6 / 2.0)
    assert step.accel_pct == 0.0


def test_accel_opening_uses_initial_gain_and_secant_update() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    step = planner.next_step()
    assert step is not None
    assert step.accel_pct == pytest.approx(10.0 + 1.0 / 2.0)  # u0 + a / g_init
    result = planner.record(step, 0.6)  # 本当の g は 0.6/0.5 = 1.2
    assert planner.g_accel == pytest.approx(1.2)
    assert result.gain_before == 2.0 and result.gain_after == pytest.approx(1.2)
    assert result.verdict == "OK"  # 0.6 は狙いのセル 0.5〜1.5 に入る



def test_hit_inside_target_cell_is_ok() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    step = planner.next_step()
    assert step is not None
    assert planner.record(step, 0.9).verdict == "OK"


def test_miss_retries_once_with_updated_gain_then_gives_up() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL][:1]
    first = planner.next_step()
    assert first is not None
    r1 = planner.record(first, 0.3)  # 0.5 に届かない（g = 0.3/0.5 = 0.6 に更新）
    assert r1.verdict == "やり直し"
    second = planner.next_step()
    assert second is not None and second.tries == 2 and second.target == first.target
    assert second.accel_pct == pytest.approx(10.0 + 1.0 / 0.6)  # 更新した g で深く踏む
    r2 = planner.record(second, 0.35)
    assert r2.verdict == "未達"
    assert planner.next_step() is None


def test_overshoot_beyond_wltp_max_drops_remaining_accel_steps() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    first = planner.next_step()
    assert first is not None
    result = planner.record(first, 2.2 * 1.2 + 0.1)  # WLTP 最大 2.2 の 1.2 倍を超えた
    assert result.verdict == "打切り"
    assert planner.next_step() is None  # 残りの加速（+2.0）はとばす


def test_governor_activation_drops_direction_but_keeps_other_direction() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    first = planner.next_step()  # 緩い減速
    assert first is not None
    planner.record(first, -1.0)
    brake = planner.next_step()
    assert brake is not None and brake.kind is StepKind.DECEL_BRAKE
    assert planner.record(brake, -2.2, governed=True).verdict == "打切り"
    nxt = planner.next_step()
    assert nxt is not None and nxt.kind is StepKind.ACCEL  # 加速側は残る


def test_measurement_landing_in_another_cell_marks_it_done() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    first = planner.next_step()  # 狙い +1.0（セル 0.5〜1.5）
    assert first is not None
    result = planner.record(first, 2.0)  # 別のセル（1.5〜3）に入った
    assert result.also_hit == 1
    assert result.verdict == "やり直し"  # 自分のセルには届いていない
    # 残りの +2.0 の狙いは消え、やり直しだけが残る
    assert planner.remaining == 1
    retry = planner.next_step()
    assert retry is not None and retry.target is not None and retry.target.a_kmhs == 1.0


def test_gain_is_clamped() -> None:
    planner = _planner(gain_max=3.0)
    _drive_to_targets(planner)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    step = planner.next_step()
    assert step is not None
    planner.record(step, 1.4)  # 0.5% で +1.4 → g = 2.8 は上限内
    assert planner.g_accel == pytest.approx(2.8)
    planner.g_accel = 1.0
    step2 = planner.next_step()  # 残りは +2.0 の狙い
    assert step2 is not None
    planner.record(step2, 2.9)  # 2.9/(u−u0) が上限 3.0 を超える設定にして確認
    assert planner.g_accel <= 3.0


def test_launch_alternates_launch_and_stop_and_uses_nearest_coast() -> None:
    planner = _planner()
    _drive_to_targets(planner)  # 25 km/h の惰行 −1.6 を覚える
    plan = gp.LaunchPlan(
        end_kmh=20.0,
        accel=(Target(1.0, 0.5, 1.5), Target(2.0, 1.5, 3.0)),
        decel=(Target(-2.0, -3.0, -1.5),),
        a_max_kmhs=2.5, a_min_kmhs=-2.5,
    )
    planner.begin_launch(plan)
    kinds = []
    steps = []
    while (step := planner.next_step()) is not None:
        kinds.append(step.kind)
        steps.append(step)
        planner.record(step, step.target.a_kmhs if step.target else 0.0)
    assert kinds == [StepKind.LAUNCH, StepKind.STOP, StepKind.LAUNCH]
    # 不感帯 + (狙い a − 最寄りの惰行 −1.6) / g（不感帯では走行抵抗で減速するので、その分を上乗せ）
    assert steps[0].accel_pct == pytest.approx(4.0 + (1.0 + 1.6) / 2.0)
    # 停車は最寄りの惰行 −1.6 を基準にする: 不感帯 6% + (−1.6 − (−2.0)) / g_brake
    assert steps[1].brake_pct == pytest.approx(6.0 + 0.4 / 2.0)


def test_status_line_mentions_key_fields() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    step = planner.next_step()
    assert step is not None
    line = gp.status_line(planner.record(step, -1.0))
    assert "25.0km/h" in line and "decel_accel" in line and "OK" in line
    assert not math.isnan(planner.results[-1].a_meas_kmhs)


def test_no_response_halves_gain_so_the_retry_presses_deeper() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL][:1]
    first = planner.next_step()
    assert first is not None
    result = planner.record(first, 0.0)  # 踏んだのに加速度が変わらない
    assert result.verdict == "やり直し"
    assert planner.g_accel == pytest.approx(1.0)  # 2.0 の半分
    second = planner.next_step()
    assert second is not None and second.accel_pct > first.accel_pct


# ── 強いステップの助走（段4b） ───────────────────────────────────


def _strong_station(speed: float = 55.0, *, accel: float = 9.0, decel: float = -8.0) -> StationPlan:
    return StationPlan(
        speed_kmh=speed, speed_lo_kmh=speed - 5.0, speed_hi_kmh=speed + 5.0,
        decel=(Target(decel, -14.0, -7.0),), accel=(Target(accel, 7.0, 14.0),),
        a_max_kmhs=12.0, a_min_kmhs=-10.0,
    )


def _to_strong(planner: GridPlanner, plan: StationPlan, u0: float = 10.0) -> None:
    """定速・惰行を消化して、強いステップが先頭に来る状態にする。"""
    planner.begin_station(plan, u0)
    hold = planner.next_step()
    assert hold is not None
    planner.record(hold, 0.0)
    coast = planner.next_step()
    assert coast is not None
    planner.record(coast, -1.6)


def test_is_strong_boundary_follows_band_lag_and_min_fit() -> None:
    planner = _planner()  # 帯 10 / 頭の除外 0.5s / 最短 1.0s → 10/a − 0.5 < 1.0 ⇔ a > 6.67
    assert not planner.is_strong(Target(6.6, 3.0, 7.0))
    assert planner.is_strong(Target(6.7, 3.0, 7.0))
    assert planner.is_strong(Target(-8.0, -14.0, -7.0))  # 減速も同じ
    assert not planner.is_strong(Target(0.0, -0.5, 0.5))


def test_approach_is_lower_edge_for_accel_and_upper_edge_for_decel() -> None:
    planner = _planner(station_min_kmh=10.0)
    _to_strong(planner, _strong_station(55.0))
    assert planner.peek_approach_kmh() == 65.0  # 先頭は減速（緩い順の先）。上側の端
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    assert planner.peek_approach_kmh() == 45.0  # 加速は下側の端


def test_approach_is_clamped_and_skipped_when_it_is_the_center() -> None:
    low = GridPlanner(
        GridSettings(station_min_kmh=10.0), accel_deadband_pct=4.0, brake_deadband_pct=6.0,
        max_accel_pct=40.0, max_brake_pct=50.0, max_speed_kmh=128.0,
    )
    _to_strong(low, _strong_station(15.0))
    low._queue = [it for it in low._queue if it.kind is StepKind.ACCEL]
    assert low.peek_approach_kmh() == 10.0  # 15 − 10 = 5 は station_min(10) 未満 → 10
    high = GridPlanner(
        GridSettings(), accel_deadband_pct=4.0, brake_deadband_pct=6.0,
        max_accel_pct=40.0, max_brake_pct=50.0, max_speed_kmh=128.0,
    )
    _to_strong(high, _strong_station(125.0))
    assert high.peek_approach_kmh() == 128.0  # 135 は最高車速の手前 128 に頭打ち
    capped = GridPlanner(
        GridSettings(), accel_deadband_pct=4.0, brake_deadband_pct=6.0,
        max_accel_pct=40.0, max_brake_pct=50.0, max_speed_kmh=125.5,
    )
    _to_strong(capped, _strong_station(125.0))
    assert capped.peek_approach_kmh() is None  # 中心とほぼ同じ（差が許容幅未満）なら助走なし


def test_no_approach_for_ordinary_targets() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    assert planner.peek_approach_kmh() is None
    step = planner.next_step()
    assert step is not None and step.approach_kmh is None


def test_strong_accel_uses_approach_base_opening_and_gain_update_uses_it() -> None:
    planner = _planner()
    _to_strong(planner, _strong_station(55.0), u0=10.0)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    step = planner.next_step(u0_pct=6.0)  # 助走の車速で落ち着いた開度（中心の u0=10 より低い）
    assert step is not None
    assert step.approach_kmh == 45.0 and step.base_pct == 6.0
    assert step.accel_pct == pytest.approx(6.0 + 9.0 / 2.0)  # u0' + a / g_init
    result = planner.record(step, 8.0)
    assert planner.g_accel == pytest.approx(8.0 / 4.5)  # 踏み増しは u0'（6.0）からの 4.5%
    assert result.approach_kmh == 45.0


def test_strong_decel_is_brake_step_with_approach() -> None:
    planner = _planner()
    _to_strong(planner, _strong_station(55.0))
    step = planner.next_step(u0_pct=6.0)
    assert step is not None and step.kind is StepKind.DECEL_BRAKE
    assert step.approach_kmh == 65.0 and step.base_pct is None
    assert step.brake_pct == pytest.approx(6.0 + (8.0 - 1.6) / 2.0)  # 惰行 −1.6 から −8.0 まで


def test_status_line_shows_approach_speed() -> None:
    planner = _planner()
    _to_strong(planner, _strong_station(55.0))
    step = planner.next_step(u0_pct=6.0)
    assert step is not None
    assert "助走 65km/h" in gp.status_line(planner.record(step, -8.0))


# ── 段5b: 定速の実測での基準補正・ブレーキ上限・発進停車の感度 ─────────


def test_accel_opening_is_corrected_by_measured_hold_accel() -> None:
    """定速ステップが +0.3 で走ったなら、その分だけ加速の開度を浅くする（基準は実測点）。"""
    planner = _planner()
    planner.begin_station(_station(), u0_pct=10.0)
    hold = planner.next_step()
    assert hold is not None
    planner.record(hold, 0.3)
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    step = planner.next_step()
    assert step is not None and step.kind is StepKind.ACCEL
    assert step.accel_pct == pytest.approx(10.0 + (1.0 - 0.3) / 2.0)  # g_init 2.0
    assert step.base_a_kmhs == pytest.approx(0.3)
    # 感度の更新も (u0, 0.3) を基準にした割線
    planner.record(step, 1.0)
    assert planner.g_accel == pytest.approx((1.0 - 0.3) / (step.accel_pct - 10.0))


def test_gentle_decel_interpolates_toward_measured_hold_accel() -> None:
    planner = _planner()
    planner.begin_station(_station(), u0_pct=10.0)
    hold = planner.next_step()
    assert hold is not None
    planner.record(hold, 0.4)  # u0 のまま +0.4 で走った
    coast = planner.next_step()
    assert coast is not None
    planner.record(coast, -1.6)
    step = planner.next_step()
    assert step is not None and step.kind is StepKind.DECEL_ACCEL
    # 惰行 (4%, −1.6) と定速 (10%, +0.4) の直線。−1.0 は 0.6/2.0 の位置
    assert step.accel_pct == pytest.approx(4.0 + 6.0 * (0.6 / 2.0))


def test_brake_depth_is_capped_by_stop_brake_then_twice_tried_max() -> None:
    """15 km/h の実機の並び: ブレーキが効かず g が極小になっても開度は上限 80% まで伸びない。"""
    planner = GridPlanner(
        GridSettings(), accel_deadband_pct=4.0, brake_deadband_pct=7.05,
        max_accel_pct=40.0, max_brake_pct=80.0, stop_brake_pct=18.32,
    )
    plan = StationPlan(
        speed_kmh=15.0, speed_lo_kmh=10.0, speed_hi_kmh=20.0,
        decel=(Target(-8.0, -14.0, -7.0),), accel=(), a_max_kmhs=2.0, a_min_kmhs=-9.0,
    )
    planner.begin_station(plan, u0_pct=7.4)
    for step, a in ((planner.next_step(), -0.1), (planner.next_step(), -1.47)):
        assert step is not None
        planner.record(step, a)
    planner.g_brake = 0.05  # 割線で極小になった状態
    first = planner.next_step()
    assert first is not None and first.kind is StepKind.DECEL_BRAKE
    assert first.brake_pct == pytest.approx(18.32)  # 未試行: 停車保持開度まで
    planner.record(first, -1.5)  # ほぼ惰行のまま
    planner.g_brake = 0.05
    retry = planner.next_step()
    assert retry is not None
    assert retry.brake_pct - 7.05 <= 2.0 * (18.32 - 7.05) + 1e-9  # 試した最大の 2 倍まで
    assert retry.brake_pct < 80.0


def test_launch_starts_from_nearest_station_gains() -> None:
    from tests.research.grid_planner import LaunchPlan

    planner = _planner()
    planner.begin_station(_station(), u0_pct=10.0)
    planner.g_accel, planner.g_brake = 0.4, 0.08
    planner.record(planner.next_step(), 0.0)  # 定速
    planner.record(planner.next_step(), -1.6)  # 惰行
    while (step := planner.next_step()) is not None:  # 25 km/h ステーションの最後の g を残す
        planner.record(step, step.target.a_kmhs if step.target else 0.0)
    g_accel, g_brake = planner.g_accel, planner.g_brake
    planner.g_accel = planner.g_brake = 9.9  # 別の車速のステーションで大きく変わった想定
    planner.begin_launch(LaunchPlan(20.0, (Target(1.0, 0.5, 1.5),), (), 2.0, -2.0))
    assert (planner.g_accel, planner.g_brake) == (g_accel, g_brake)


# ── 上限 G の予測による開度の頭打ち（段6a） ──────────────────────


def _capped_planner(brake_cap: float | None, accel_cap: float | None = None) -> GridPlanner:
    calls: list[tuple[str, float]] = []

    def cap_fn(pedal: str, speed: float) -> float | None:
        calls.append((pedal, speed))
        return brake_cap if pedal == "brake" else accel_cap

    planner = GridPlanner(
        GridSettings(), accel_deadband_pct=4.0, brake_deadband_pct=6.0,
        max_accel_pct=40.0, max_brake_pct=50.0, cap_fn=cap_fn,
    )
    planner.cap_calls = calls  # type: ignore[attr-defined]
    return planner


def test_cap_limits_brake_opening_and_asks_at_the_fast_end_of_the_band() -> None:
    planner = _capped_planner(brake_cap=6.2)
    _drive_to_targets(planner)
    planner.record(planner.next_step(), -1.0)
    step = planner.next_step()
    assert step is not None and step.kind is StepKind.DECEL_BRAKE
    assert step.brake_pct == pytest.approx(6.2) and step.capped
    # ブレーキは速いほど効く: ステーション 25 km/h の帯の上端（+10 = 35 km/h）で上限を引く
    assert ("brake", 35.0) in planner.cap_calls  # type: ignore[attr-defined]


def test_cap_asks_accel_at_the_slow_end_of_the_band() -> None:
    planner = _capped_planner(brake_cap=None, accel_cap=5.0)
    _drive_to_targets(planner)
    while (step := planner.next_step()) is not None and step.kind is not StepKind.ACCEL:
        planner.record(step, step.target.a_kmhs if step.target else 0.0)
    assert step is not None and step.accel_pct == pytest.approx(5.0) and step.capped
    assert ("accel", 15.0) in planner.cap_calls  # type: ignore[attr-defined]


def test_cap_above_the_needed_opening_changes_nothing() -> None:
    planner = _capped_planner(brake_cap=40.0)
    _drive_to_targets(planner)
    planner.record(planner.next_step(), -1.0)
    step = planner.next_step()
    assert step is not None and not step.capped
    assert step.brake_pct == pytest.approx(6.0 + 0.6 / 2.0)


def test_capped_miss_is_verdict_cap_and_drops_the_direction_without_retry() -> None:
    planner = _capped_planner(brake_cap=6.2)
    _drive_to_targets(planner)
    planner.record(planner.next_step(), -1.0)
    step = planner.next_step()
    assert step is not None and step.capped
    result = planner.record(step, -1.4)  # 狙い −2.2（−3〜−1.5）のセルに届かない
    assert result.verdict == "上限G"
    kinds = []
    while (nxt := planner.next_step()) is not None:
        kinds.append(nxt.kind)
        planner.record(nxt, nxt.target.a_kmhs if nxt.target else 0.0)
    assert StepKind.DECEL_BRAKE not in kinds  # やり直しも残りの減速も無い
    assert StepKind.ACCEL in kinds  # 加速の向きは残る


def test_capped_step_that_still_hits_the_cell_is_ok() -> None:
    planner = _capped_planner(brake_cap=6.2)
    _drive_to_targets(planner)
    planner.record(planner.next_step(), -1.0)
    step = planner.next_step()
    assert step is not None
    assert planner.record(step, -2.2).verdict == "OK"


def test_no_cap_fn_is_the_old_behavior() -> None:
    planner = _planner()
    _drive_to_targets(planner)
    planner.record(planner.next_step(), -1.0)
    step = planner.next_step()
    assert step is not None and not step.capped


# ── コーストダウンの実測から惰行を引く（段7a） ──────────────────────


def _coast_planner(coast_fn) -> GridPlanner:  # type: ignore[no-untyped-def]
    return GridPlanner(
        GridSettings(), accel_deadband_pct=4.0, brake_deadband_pct=6.0,
        max_accel_pct=40.0, max_brake_pct=50.0, coast_fn=coast_fn,
    )


def test_coast_fn_skips_the_coast_step_and_seeds_a_coast() -> None:
    planner = _coast_planner(lambda v: -1.6 if v == 25.0 else None)
    planner.begin_station(_station(), u0_pct=10.0)
    hold = planner.next_step()
    assert hold is not None and hold.kind is StepKind.HOLD
    planner.record(hold, 0.0)
    step = planner.next_step()
    assert step is not None and step.kind is not StepKind.COAST  # 惰行ステップを入れない
    assert step.kind is StepKind.DECEL_ACCEL
    # 惰行 −1.6（coast_fn）と定速 u0=10% の間を直線で結ぶ（_drive_to_targets と同じ式）
    assert step.accel_pct == pytest.approx(4.0 + 6.0 * (0.6 / 1.6))


def test_coast_fn_returning_none_falls_back_to_the_coast_step() -> None:
    planner = _coast_planner(lambda v: None)
    planner.begin_station(_station(), u0_pct=10.0)
    hold = planner.next_step()
    assert hold is not None
    planner.record(hold, 0.0)
    step = planner.next_step()
    assert step is not None and step.kind is StepKind.COAST  # 従来どおりのフォールバック


def test_coast_fn_prefers_the_midpoint_for_launch_over_nearest_station() -> None:
    calls: list[float] = []

    def coast_fn(v: float) -> float | None:
        calls.append(v)
        return -1.2 if v == 10.0 else None

    planner = _coast_planner(coast_fn)
    _drive_to_targets(planner)  # 25 km/h の惰行 −1.6 を（COAST ステップから）覚える
    plan = gp.LaunchPlan(
        end_kmh=20.0, accel=(Target(1.0, 0.5, 1.5),), decel=(), a_max_kmhs=2.5, a_min_kmhs=-2.5,
    )
    planner.begin_launch(plan)
    step = planner.next_step()
    assert step is not None
    # coast_fn(10.0) = −1.2 を優先する（測ったステーション 25 の −1.6 ではなく）
    assert step.accel_pct == pytest.approx(4.0 + (1.0 + 1.2) / 2.0)
    assert 10.0 in calls


# ── 感度の最初の種まき（段7c） ────────────────────────────────────────


def _gain_planner(gain_fn) -> GridPlanner:  # type: ignore[no-untyped-def]
    return GridPlanner(
        GridSettings(), accel_deadband_pct=4.0, brake_deadband_pct=6.0,
        max_accel_pct=40.0, max_brake_pct=50.0, gain_fn=gain_fn,
    )


def test_gain_fn_seeds_accel_gain_after_hold_step() -> None:
    """段7c: HOLD の実測直後に、まだ測っていないアクセルの感度を gain_fn の実測点から引く。"""
    planner = _gain_planner(lambda pedal, v: (20.0, 4.0) if pedal == gp.PEDAL_ACCEL else None)
    planner.begin_station(_station(), u0_pct=10.0)
    hold = planner.next_step()
    assert hold is not None and hold.kind is StepKind.HOLD
    planner.record(hold, 0.5)  # a_hold = 0.5（base = u0=10.0, a_hold=0.5）
    # (20.0, 4.0) と (10.0, 0.5) の割線 = (4.0−0.5) / (20.0−10.0)
    assert planner.g_accel == pytest.approx((4.0 - 0.5) / 10.0)


def test_gain_fn_seeds_brake_gain_after_coast_step() -> None:
    """惰行ステップの実測直後に、ブレーキの感度を引く。`gain_fn` はペダルの向きを正（減速が正）で
    返すが、惰行の実測 a_coast は車速の傾きそのまま（負）なので、符号をそろえて割線を作る。
    """
    planner = _gain_planner(lambda pedal, v: (12.0, 5.4) if pedal == gp.PEDAL_BRAKE else None)
    planner.begin_station(_station(), u0_pct=10.0)
    hold = planner.next_step()
    assert hold is not None
    planner.record(hold, 0.0)
    coast = planner.next_step()
    assert coast is not None and coast.kind is StepKind.COAST
    planner.record(coast, -1.6)  # a_coast = −1.6（不感帯 6.0 で 1.6 の減速）
    # (12.0, 5.4) と (不感帯 6.0, 1.6) の割線 = (5.4−1.6) / (12.0−6.0)
    assert planner.g_brake == pytest.approx((5.4 - 1.6) / 6.0)


def test_gain_fn_seeds_brake_gain_via_coast_fn_at_begin_station() -> None:
    """coast_fn（段7a。惰行ステップを省く）があっても begin_station の時点で種をまく。"""
    planner = GridPlanner(
        GridSettings(), accel_deadband_pct=4.0, brake_deadband_pct=6.0,
        max_accel_pct=40.0, max_brake_pct=50.0,
        coast_fn=lambda v: -1.6,
        gain_fn=lambda pedal, v: (12.0, 5.4) if pedal == gp.PEDAL_BRAKE else None,
    )
    planner.begin_station(_station(), u0_pct=10.0)
    assert planner.g_brake == pytest.approx((5.4 - 1.6) / 6.0)


def test_gain_fn_seed_is_used_once_then_overwritten_by_a_real_measurement() -> None:
    """種をまいた感度は最初の 1 回だけ使われ、実測すれば `_update_gain` が上書きする。"""
    planner = _gain_planner(lambda pedal, v: (20.0, 4.0) if pedal == gp.PEDAL_ACCEL else None)
    planner.begin_station(_station(), u0_pct=10.0)
    hold = planner.next_step()
    assert hold is not None and hold.kind is StepKind.HOLD
    planner.record(hold, 0.0)  # a_hold = 0.0
    coast = planner.next_step()
    assert coast is not None and coast.kind is StepKind.COAST
    planner.record(coast, -1.6)
    assert planner.g_accel == pytest.approx(0.4)  # (20.0, 4.0) と (10.0, 0.0) の割線（種まき）
    planner._queue = [it for it in planner._queue if it.kind is StepKind.ACCEL]
    step = planner.next_step()
    assert step is not None
    assert step.accel_pct == pytest.approx(10.0 + 1.0 / 0.4)  # 種をまいた感度で開度を決める
    planner.record(step, 0.6)  # 実測: 本当の g は 0.6 / (step.accel_pct − 10.0)
    assert planner.g_accel == pytest.approx(0.6 / (1.0 / 0.4))
