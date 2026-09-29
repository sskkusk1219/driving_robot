"""格子ステップ走行（pattern_loop の GridStationPattern / GridLaunchPattern / HOLD_STEP）のテスト。

プランナーの中身（開度の決め方・打ち切り）は test_research_grid_planner.py。ここは走行の状態機械
（PI で落ち着く → 開度固定のステップ → u0 に戻る → 止まらず次のステーションへ、発進・停車の流れ、
ガバナ・安全側の遷移）を見る。
"""

from __future__ import annotations

import asyncio

import pytest

from tests.research import pattern_loop as plmod
from tests.research.grid_planner import (
    GridSettings,
    LaunchPlan,
    StationPlan,
    StepKind,
    StepResult,
    Target,
)
from tests.research.pattern_loop import PatternLoop, PatternLoopConfig, _Phase
from tests.research.research_types import G_TO_KMHS, PatternKind
from tests.research.test_research_pattern_loop import FakeAxis, _loop, _run_until_done


def _settings(**kw: float) -> GridSettings:
    base = {
        "settle_s": 1.0, "settle_timeout_s": 10.0, "step_window_s": 1.0, "step_lag_s": 0.2,
        "settle_tol_kmh": 1.0, "step_band_max_kmh": 10.0,
    }
    base.update(kw)
    return GridSettings(**base)  # type: ignore[arg-type]


def _plan(speed: float = 25.0, *, decel: bool = True, accel: bool = True) -> StationPlan:
    return StationPlan(
        speed_kmh=speed, speed_lo_kmh=speed - 5.0, speed_hi_kmh=speed + 5.0,
        decel=(Target(-1.0, -1.5, -0.5),) if decel else (),
        accel=(Target(1.0, 0.5, 1.5),) if accel else (),
        a_max_kmhs=2.0, a_min_kmhs=-2.5,
    )


def _station(plan: StationPlan | None = None, **settings: float) -> plmod.GridStationPattern:
    return plmod.GridStationPattern(
        PatternKind.GRID_STEP, accel_opening=0.0, brake_opening=0.0, hold_duration_s=1.0,
        plan=plan or _plan(), settings=_settings(**settings),
    )


def _launch(**settings: float) -> plmod.GridLaunchPattern:
    return plmod.GridLaunchPattern(
        PatternKind.GRID_LAUNCH, accel_opening=0.0, brake_opening=0.0, hold_duration_s=1.0,
        plan=LaunchPlan(
            end_kmh=20.0, accel=(Target(1.0, 0.5, 1.5), Target(2.0, 1.5, 3.0)),
            decel=(Target(-2.0, -3.0, -1.5),), a_max_kmhs=2.5, a_min_kmhs=-2.5,
        ),
        settings=_settings(**settings),
    )


def _settle(
    loop: PatternLoop, pattern: plmod.GridStationPattern, u: float, t0: float = 0.0
) -> float:
    """許容幅内に settle_s いさせて落ち着かせる。戻り値は落ち着いた時刻。"""
    assert pattern.plan is not None
    t, idx = t0, loop._pattern_idx
    while loop._phase is _Phase.CRUISE_HOLD and loop._pattern_idx == idx:
        loop._current_accel_opening = u
        loop._advance_grid_hold(pattern, pattern.plan.speed_kmh, t)
        t += 0.25
    return t - 0.25


def _finish_step(loop: PatternLoop, pattern: plmod.GridStationPattern, t0: float,
                 slope: float = 0.0, window: float = 1.0) -> float:
    """開度固定のステップを、車速が一定の傾きで動く想定で終わらせる。戻り値は終了時刻。"""
    assert pattern.plan is not None
    t = t0
    while loop._phase is _Phase.HOLD_STEP:
        t += 0.1
        loop._advance_grid_step(pattern, pattern.plan.speed_kmh + slope * (t - t0), t)
        assert t - t0 < window + 1.0
    return t


# ── 状態機械（同期）──────────────────────────────────────────────────


def test_station_pattern_starts_in_cruise_hold_and_launch_in_hold_step() -> None:
    loop, *_ = _loop([_station(), _launch()])
    assert loop._phase is _Phase.CRUISE_HOLD
    assert loop._initial_phase(1) is _Phase.HOLD_STEP
    assert loop.grid_planner is not None


def test_pi_starts_from_deadband_and_rises_at_the_rate_limit() -> None:
    pattern = _station(hold_max_rate_pct_s=2.0, hold_kp_norm=0.5)
    loop, *_ = _loop([pattern])
    db = loop._profile.feedforward_params.accel_deadband_pct
    loop._last_speed = 0.0
    first = loop._grid_hold_opening(pattern)
    assert first == pytest.approx(db + 2.0 * loop._interval_s)  # レート上限（g では割らない）
    loop._last_speed = 40.0  # 目標超過 → 下げる（不感帯が下限）
    assert loop._grid_hold_opening(pattern) <= first


def test_settled_window_mean_becomes_u0_and_starts_hold_step() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    for i, t in enumerate([0.0, 0.5, 1.0]):
        loop._current_accel_opening = 6.0 + i
        loop._advance_grid_hold(pattern, 25.5, t)
    assert loop._phase is _Phase.HOLD_STEP
    assert loop._grid_step is not None and loop._grid_step.kind is StepKind.HOLD
    assert loop._grid_step.accel_pct == pytest.approx(7.0)  # (6 + 7 + 8) / 3
    assert loop.grid_planner.u0 == pytest.approx(7.0)  # type: ignore[union-attr]


def test_leaving_tolerance_restarts_the_settle_window() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    loop._advance_grid_hold(pattern, 25.0, 0.0)
    loop._advance_grid_hold(pattern, 30.0, 0.6)  # 許容幅の外へ
    assert loop._grid_settle_since is None and loop._grid_window == []
    loop._advance_grid_hold(pattern, 25.0, 0.7)
    loop._advance_grid_hold(pattern, 25.0, 1.6)  # 0.7 から 0.9s: まだ settle_s(1.0) に届かない
    assert loop._phase is _Phase.CRUISE_HOLD


def test_step_measures_slope_records_and_returns_to_pi_at_u0() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    results: list[StepResult] = []
    loop._on_grid_result = results.append
    loop._grid_enter_pattern(pattern, 0.0)
    t = _settle(loop, pattern, 7.0)
    _finish_step(loop, pattern, t, slope=0.5)  # 定速ステップ（車速が +0.5 km/h/s で動いた）
    assert results[0].kind is StepKind.HOLD
    assert results[0].a_meas_kmhs == pytest.approx(0.5, abs=0.02)
    assert loop._phase is _Phase.CRUISE_HOLD  # 開度を u0 に戻して PI へ
    assert loop._grid_opening == pytest.approx(7.0) and loop._grid_integral == pytest.approx(7.0)
    assert loop._grid_step is None


def test_step_ends_when_speed_leaves_band_or_reaches_floor() -> None:
    pattern = _station(_plan(25.0))
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    t = _settle(loop, pattern, 7.0)
    loop._advance_grid_step(pattern, 25.0, t + 0.1)
    loop._advance_grid_step(pattern, 36.0, t + 0.2)  # station + 10 を超えた → 窓の途中でも終わる
    assert loop._phase is _Phase.GRID_RETURN  # 目標（25）から離れている（段7b）→ 固定開度で戻る
    assert len(loop.grid_planner.results) == 1  # type: ignore[union-attr]


def test_speed_dropping_to_floor_keeps_the_remaining_steps() -> None:
    """低いステーションで強い減速のあとに車速が底まで落ちても、続くステップ（加速）は捨てない。"""
    pattern = _station(_plan(15.0, decel=False, accel=True))
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    t = _settle(loop, pattern, 7.0)
    loop._advance_grid_step(pattern, 2.0, t + 0.1)  # 底（coast_down_stop_speed_kmh）以下で終わる
    assert loop._phase is _Phase.GRID_RETURN  # 停車復帰へ行かず、固定開度で目標へ戻る（段7b）
    assert loop.grid_planner.remaining == 2  # type: ignore[union-attr]  # 惰行・加速が残る


def test_all_steps_then_next_station_without_stopping_then_stop_return() -> None:
    p1 = _station(_plan(15.0, decel=False, accel=False))
    p2 = _station(_plan(25.0, decel=False, accel=False))
    loop, *_ = _loop([p1, p2])
    loop._grid_enter_pattern(p1, 0.0)
    t = _settle(loop, p1, 6.0)
    for _ in range(2):  # 定速・惰行
        t = _finish_step(loop, p1, t)
        t = _settle(loop, p1, 6.0, t + 0.25) if loop._phase is _Phase.CRUISE_HOLD else t
    assert loop._pattern_idx == 1 and loop._phase is _Phase.CRUISE_HOLD  # 止まらずに次へ
    assert loop._grid_opening is not None  # PI の状態を引き継ぐ
    assert loop._grid_seen_idx == 0  # 次の周期の入場処理でタイマーがリセットされる
    loop._grid_enter_pattern(p2, t)
    loop._last_speed = 25.0
    t = _settle(loop, p2, 6.0, t)
    for _ in range(2):
        t = _finish_step(loop, p2, t)
        if loop._phase is _Phase.CRUISE_HOLD:
            t = _settle(loop, p2, 6.0, t + 0.25)
    assert loop._phase is _Phase.DRIVE_BRAKE  # 最後のステーションの後は停車復帰
    assert loop._pattern_idx == 1  # 停車したら _advance_pattern が進める


def test_settle_timeout_skips_station_and_moves_on() -> None:
    p1, p2 = _station(settle_timeout_s=2.0), _station(_plan(35.0))
    loop, *_ = _loop([p1, p2])
    loop._grid_enter_pattern(p1, 0.0)
    for t in (0.0, 1.0, 2.1):  # 許容幅に入らないまま打ち切り
        loop._advance_grid_hold(p1, 10.0, t)
    assert loop._pattern_idx == 1 and loop._phase is _Phase.CRUISE_HOLD
    assert loop.grid_planner.remaining == 0  # type: ignore[union-attr]


def test_overspeed_in_grid_hold_and_step_enters_recovery_brake() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    loop._advance_grid_hold(pattern, loop._profile.max_speed + 1.0, 0.1)
    assert loop._phase is _Phase.DRIVE_BRAKE and loop._overspeed_recovery
    loop2, *_ = _loop([pattern])
    loop2._grid_enter_pattern(pattern, 0.0)
    t = _settle(loop2, pattern, 7.0)
    loop2._advance_grid_step(pattern, loop2._profile.max_speed + 1.0, t + 0.1)
    assert loop2._phase is _Phase.DRIVE_BRAKE and loop2._grid_step is None


def test_hold_step_command_ramps_over_the_lag_and_governor_caps_it() -> None:
    """段6b: 踏むペダルは step_lag_s かけて上げる（一気に出さない）。もう一方は即座に離す。"""
    pattern = _station()
    loop, *_ = _loop([pattern])
    lag = pattern.settings.step_lag_s
    loop._grid_enter_pattern(pattern, 0.0)
    t = _settle(loop, pattern, 7.0)
    # 定速ステップ → 惰行ステップ
    t = _finish_step(loop, pattern, t)
    t = _settle(loop, pattern, 7.0, t + 0.25)
    assert loop._grid_step is not None
    assert loop._grid_step.kind is StepKind.COAST  # 惰行（両ペダル 0%）
    assert loop._grid_step_openings(t, lag) == (0.0, 0.0)
    # 加速ステップを仕込む。始点は始めたときの開度（アクセル 5%）→ 12% へ lag かけて上がる
    from tests.research.grid_planner import Step

    loop._current_accel_opening = 5.0
    loop._grid_start_step(Step(StepKind.ACCEL, 12.0, 0.0, Target(1.0, 0.5, 1.5)), t)
    assert loop._grid_step_openings(t, lag) == (pytest.approx(5.0), 0.0)
    assert loop._grid_step_openings(t + lag / 2, lag)[0] == pytest.approx(8.5)
    assert loop._grid_step_openings(t + lag, lag) == (pytest.approx(12.0), 0.0)
    loop._current_accel_opening = 12.0
    loop._update_governor(loop._g_limit_kmhs + 1.0)  # 初回は現在開度で頭打ち
    loop._update_governor(loop._g_limit_kmhs + 1.0)  # 以降は 1 周期ごとに下げる
    accel, _ = loop._grid_step_openings(t + lag, lag)
    assert accel < 12.0 and loop._governor_limiting()
    assert G_TO_KMHS > 0.0


def test_brake_step_releases_the_accel_at_once_and_ramps_the_brake() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    lag = pattern.settings.step_lag_s
    loop._grid_enter_pattern(pattern, 0.0)
    from tests.research.grid_planner import Step

    loop._current_accel_opening, loop._current_brake_opening = 9.0, 0.0
    loop._grid_start_step(Step(StepKind.DECEL_BRAKE, 0.0, 20.0, Target(-8.0, -14.0, -7.0)), 1.0)
    accel, brake = loop._grid_step_openings(1.0 + lag / 2, lag)
    assert accel == 0.0 and brake == pytest.approx(10.0)


def test_launch_runs_launch_then_stop_then_stop_return_then_next_launch() -> None:
    pattern = _launch(settle_timeout_s=60.0)
    loop, *_ = _loop([pattern])
    results: list[StepResult] = []
    loop._on_grid_result = results.append
    assert loop._grid_enter_pattern(pattern, 0.0) is False
    assert loop._phase is _Phase.HOLD_STEP
    assert loop._grid_step is not None and loop._grid_step.kind is StepKind.LAUNCH
    # 発進: 0 → 20 km/h（傾き 1 km/h/s = 最初の狙い +1.0 のセルに入る）
    t = 0.0
    while loop._grid_step is not None and loop._grid_step.kind is StepKind.LAUNCH:
        t += 0.1
        loop._advance_grid_step(pattern, min(1.0 * t, 20.0), t)
    assert results[0].kind is StepKind.LAUNCH and results[0].verdict == "OK"
    assert results[0].a_meas_kmhs == pytest.approx(1.0, abs=0.1)
    assert loop._grid_step is not None and loop._grid_step.kind is StepKind.STOP  # 止まらずに停車へ
    assert loop._grid_step.brake_pct > 0.0 and loop._grid_step.accel_pct == 0.0
    # 停車: 20 → 0 km/h。停車で終わったら（走っていないので）停車復帰を挟まず次の発進へ
    t0 = t
    while loop._grid_step is not None and loop._grid_step.kind is StepKind.STOP:
        t += 0.1
        loop._advance_grid_step(pattern, max(20.0 - 2.0 * (t - t0), 0.0), t)
    assert results[1].kind is StepKind.STOP and results[1].verdict == "OK"
    assert results[1].a_meas_kmhs == pytest.approx(-2.0, abs=0.1)
    assert loop._phase is _Phase.HOLD_STEP
    assert loop._grid_step is not None and loop._grid_step.kind is StepKind.LAUNCH
    # 最後の発進（+2.0）が終わると、停車ステップは残っていないので停車復帰し、パターンが終わる
    t2 = t
    while loop._grid_step is not None:
        t2 += 0.1
        loop._advance_grid_step(pattern, min(2.0 * (t2 - t), 20.0), t2)
    assert results[2].kind is StepKind.LAUNCH and results[2].verdict == "OK"
    assert loop._phase is _Phase.DRIVE_BRAKE
    loop._advance(pattern, 0.0, 0.0, t2 + 3.0)
    assert loop._pattern_idx == 1  # パターン完了


def test_stop_step_that_times_out_while_moving_goes_through_stop_return() -> None:
    pattern = _launch(settle_timeout_s=25.0)
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    t = 0.0
    while loop._grid_step is not None and loop._grid_step.kind is StepKind.LAUNCH:
        t += 0.1
        loop._advance_grid_step(pattern, min(20.0, 1.0 * t), t)  # 傾き +1 で 20 km/h へ
    assert loop._grid_step is not None and loop._grid_step.kind is StepKind.STOP
    t0 = t
    while loop._grid_step is not None:
        t += 0.1
        loop._advance_grid_step(pattern, 15.0, t)  # 制動が弱く、打ち切りまで走ったまま
        assert t - t0 < 30.0
    assert loop._phase is _Phase.DRIVE_BRAKE  # 走っているので停車復帰 → 次の発進
    loop._advance(pattern, 0.0, 0.0, t + 3.0)
    assert loop._phase is _Phase.HOLD_STEP
    assert loop._grid_step is not None and loop._grid_step.kind is StepKind.LAUNCH


def test_launch_without_targets_is_skipped() -> None:
    empty = plmod.GridLaunchPattern(
        PatternKind.GRID_LAUNCH, accel_opening=0.0, brake_opening=0.0, hold_duration_s=1.0,
        plan=LaunchPlan(20.0, (), (), 0.0, 0.0), settings=_settings(),
    )
    loop, *_ = _loop([empty])
    assert loop._grid_enter_pattern(empty, 0.0) is True
    assert loop._pattern_idx == 1


def test_step_slope_uses_samples_after_lag_and_ignores_standstill_for_launch() -> None:
    pattern = _launch()
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    step = loop._grid_step
    assert step is not None
    # 0〜1s は停車（速度 0）、その後 4 km/h/s で加速。停車の区間は傾きに入れない
    loop._grid_samples = [
        (0.1 * i, 0.0 if i < 10 else 4.0 * (0.1 * i - 1.0) + 2.0) for i in range(25)
    ]
    slope, points, _ = loop._grid_measured_accel(step, 2.5, pattern.settings)
    assert slope == pytest.approx(4.0, abs=0.05) and points > 3


# ── 強いステップの助走（段4b） ───────────────────────────────────────


def _strong_plan(speed: float = 55.0, *, accel: bool = True, decel: bool = False) -> StationPlan:
    return StationPlan(
        speed_kmh=speed, speed_lo_kmh=speed - 5.0, speed_hi_kmh=speed + 5.0,
        decel=(Target(-8.0, -14.0, -7.0),) if decel else (),
        accel=(Target(9.0, 7.0, 14.0),) if accel else (),
        a_max_kmhs=12.0, a_min_kmhs=-10.0,
    )


def _through_hold_and_coast(
    loop: PatternLoop, pattern: plmod.GridStationPattern, u: float = 7.0
) -> float:
    """定速・惰行のステップを終え、強いステップの助走へ移った CRUISE_HOLD にする。戻り値は時刻。

    助走の目標は中心から離れているため段7b の GRID_RETURN を経由する。戻りにかかる時間は
    このテストの本質ではないので、目標にちょうど着いたことにして即 CRUISE_HOLD へ切り替える。
    """
    assert pattern.plan is not None
    loop._grid_enter_pattern(pattern, 0.0)
    t = _settle(loop, pattern, u)  # 定速ステップが始まる
    window = pattern.settings.step_window_s
    t = _finish_step(loop, pattern, t, window=window)
    t = _settle(loop, pattern, u, t + 0.25)  # 惰行ステップが始まる
    t = _finish_step(loop, pattern, t, window=window) + 0.25
    while loop._phase is _Phase.CRUISE_HOLD and loop._grid_approach_kmh is None:
        loop._current_accel_opening = u  # 中心で落ち着く → 強いステップなので助走へ移る
        loop._advance_grid_hold(pattern, pattern.plan.speed_kmh, t)
        t += 0.25
    if loop._phase is _Phase.GRID_RETURN:  # 助走の目標へ固定開度で戻る区間（段7b）
        loop._advance_grid_return(pattern, loop._grid_target_kmh(pattern), t)
        t += 0.25
    return t


def test_strong_accel_moves_to_approach_speed_then_steps_from_there() -> None:
    pattern = _station(_strong_plan(55.0), step_lag_s=0.5)
    loop, *_ = _loop([pattern])
    t = _through_hold_and_coast(loop, pattern)
    # 落ち着いた直後に助走へ: 目標車速が帯の下の端になり、落ち着き判定をやり直す
    assert loop._phase is _Phase.CRUISE_HOLD
    assert loop._grid_approach_kmh == 45.0
    assert loop._grid_target_kmh(pattern) == 45.0
    assert loop._grid_settle_since is None and loop._grid_window == []
    for k in range(6):  # 45 km/h（開度 8%）で落ち着く
        loop._current_accel_opening = 8.0
        loop._advance_grid_hold(pattern, 45.0, t + 0.5 + 0.25 * k)
    step = loop._grid_step
    assert loop._phase is _Phase.HOLD_STEP and step is not None
    assert step.kind is StepKind.ACCEL and step.approach_kmh == 45.0
    assert step.base_pct == pytest.approx(8.0)  # 助走の車速で落ち着いた開度 u0'（中心の u0 は 7）
    assert loop._grid_approach_kmh is None  # 助走は済んだ


def test_approach_step_ends_only_when_it_leaves_the_band_on_the_far_side() -> None:
    pattern = _station(_strong_plan(55.0), step_window_s=5.0, step_lag_s=0.5)
    loop, *_ = _loop([pattern])
    t = _through_hold_and_coast(loop, pattern)
    for k in range(6):
        loop._current_accel_opening = 8.0
        loop._advance_grid_hold(pattern, 45.0, t + 0.5 + 0.25 * k)
    assert loop._phase is _Phase.HOLD_STEP
    now = t + 3.0
    loop._advance_grid_step(pattern, 44.0, now)  # 助走の端は帯の外（下側）。ここでは終わらない
    loop._advance_grid_step(pattern, 60.0, now + 0.1)
    assert loop._phase is _Phase.HOLD_STEP
    loop._advance_grid_step(pattern, 66.0, now + 0.2)  # 帯の上の端（65）を超えた
    assert loop._phase is not _Phase.HOLD_STEP
    result = loop.grid_planner.results[-1]  # type: ignore[union-attr]
    assert result.kind is StepKind.ACCEL and result.approach_kmh == 45.0
    assert loop._grid_approach_kmh is None  # 中心へ戻る


def test_strong_decel_approaches_from_above() -> None:
    pattern = _station(
        _strong_plan(55.0, accel=False, decel=True), step_window_s=5.0, step_lag_s=0.5
    )
    loop, *_ = _loop([pattern])
    t = _through_hold_and_coast(loop, pattern)
    assert loop._grid_approach_kmh == 65.0
    for k in range(6):
        loop._current_accel_opening = 9.0
        loop._advance_grid_hold(pattern, 65.0, t + 0.5 + 0.25 * k)
    step = loop._grid_step
    assert step is not None and step.kind is StepKind.DECEL_BRAKE and step.approach_kmh == 65.0
    now = t + 3.0
    loop._advance_grid_step(pattern, 60.0, now)  # 中心（55）を越える前は続く
    assert loop._phase is _Phase.HOLD_STEP
    loop._advance_grid_step(pattern, 44.0, now + 0.1)  # 帯の下の端（45）を下回った
    assert loop._phase is not _Phase.HOLD_STEP


def test_approach_that_never_settles_skips_only_that_step() -> None:
    pattern = _station(
        _strong_plan(55.0, accel=True, decel=True), settle_timeout_s=2.0, step_lag_s=0.5
    )
    loop, *_ = _loop([pattern])
    t = _through_hold_and_coast(loop, pattern)
    planner = loop.grid_planner
    assert planner is not None and loop._grid_approach_kmh == 65.0  # 先頭は減速
    loop._advance_grid_hold(pattern, 30.0, t + 0.5)  # 許容幅の外のまま
    loop._advance_grid_hold(pattern, 30.0, t + 3.0)  # settle_timeout_s（2.0）超え
    assert loop._grid_approach_kmh is None and loop._pattern_idx == 0  # ステーションは続く
    assert loop._phase is _Phase.CRUISE_HOLD
    assert planner.peek_approach_kmh() == 45.0  # 減速だけ捨てて、次の加速へ
    assert loop._grid_target_kmh(pattern) == 55.0  # 目標は中心へ戻った


def test_grid_planner_gets_max_speed_below_profile_max() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    planner = loop.grid_planner
    assert planner is not None
    assert planner._max_speed == pytest.approx(loop._profile.max_speed - 2.0)  # 許容幅 1.0 の 2 倍


# ── ステップ後の戻り: GRID_RETURN（段7b） ────────────────────────────


def test_return_opening_presses_toward_target_when_slower() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)  # u0 = 9.0、a_hold は 0.0 にリセットされる
    loop.grid_planner.g_accel = 2.0
    loop._grid_station_begun = True
    # 段7c: GRID_RETURN に入るときに _grid_begin_return が始点を決める
    # (u0 + (grid_return_accel_kmhs − a_hold) / g_accel = 9 + (2.0 − 0.0) / 2.0 = 10.0)
    loop._grid_begin_return(pattern, 0.0, pattern.plan.speed_kmh, 20.0, is_approach=False)
    assert loop._phase is _Phase.GRID_RETURN
    assert loop._gr_opening == pytest.approx(10.0)
    loop._last_speed = 20.0  # 目標（25）より遅い
    assert loop._grid_return_opening(pattern) == pytest.approx(10.0)


def test_return_opening_uses_deadband_when_faster_than_target() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)
    loop._last_speed = 30.0  # 目標（25）より速い → 惰行でよい
    db = loop._profile.feedforward_params.accel_deadband_pct
    assert loop._grid_return_opening(pattern) == pytest.approx(db)


def test_return_opening_is_floored_at_u0_and_capped_by_glimit() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)
    loop.grid_planner.g_accel = 0.2  # 小さい g → 大きく踏む開度になる
    loop._grid_station_begun = True
    # 段7c: 始点の計算（cap 無し）は _grid_begin_return が行う
    loop._grid_begin_return(pattern, 0.0, pattern.plan.speed_kmh, 0.0, is_approach=False)
    raw = loop._gr_opening
    assert raw >= 9.0  # 下限は u0
    loop._glimit_cap = lambda pedal, speed: raw - 1.0  # type: ignore[method-assign]
    loop._grid_begin_return(pattern, 0.0, pattern.plan.speed_kmh, 0.0, is_approach=False)
    assert loop._gr_opening == pytest.approx(raw - 1.0)


def test_step_grid_return_presses_further_when_measured_accel_is_short() -> None:
    """段7c: 始点の開度（感度の当て推量）が実測の加速度を狙いまで出せていなければ、
    `_step_grid_return` が実測で補正して踏み足す。20260928 実機ではここが無く、初期の感度が
    実測の 1/8 しかなかったため開度が狙いの半分未満で止まり、釣り合って 190 s 動かなくなった。
    """
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)
    loop._grid_station_begun = True
    loop._grid_begin_return(pattern, 0.0, pattern.plan.speed_kmh, 20.0, is_approach=False)
    start = loop._gr_opening
    t, speed = 0.0, 20.0
    for _ in range(15):  # 実測の加速度 ≈ 0.25 km/h/s ≪ 狙い（既定 2.0 km/h/s）
        t += 0.2
        speed += 0.05
        loop._advance_grid_return(pattern, speed, t)
    assert loop._phase is _Phase.GRID_RETURN  # まだ目標（25）に届いていない
    assert loop._gr_opening > start


def test_advance_grid_return_switches_to_cruise_hold_within_tolerance() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)
    loop._grid_station_begun = True
    loop._phase = _Phase.GRID_RETURN
    switch = pattern.settings.grid_return_switch_kmh
    loop._advance_grid_return(pattern, pattern.plan.speed_kmh - switch, 0.0)  # ちょうど許容幅
    assert loop._phase is _Phase.CRUISE_HOLD
    assert loop._grid_opening == pytest.approx(9.0) and loop._grid_integral == pytest.approx(9.0)


def test_advance_grid_return_keeps_returning_while_still_far() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)
    loop._grid_station_begun = True
    loop._phase = _Phase.GRID_RETURN
    loop._advance_grid_return(pattern, 0.0, 0.0)  # まだ目標（25）から離れている
    assert loop._phase is _Phase.GRID_RETURN


def test_advance_grid_return_times_out_and_skips_station_when_target_unreachable() -> None:
    """段7c（バグ修正）: 釣り合い車速が目標に届かないと、GRID_RETURN も settle_timeout_s で
    打ち切ってステーションの残りを捨て、次へ進む（20260928 実機で 190 s 止まったバグ。
    以前は打ち切りが CRUISE_HOLD 側にしか無く、GRID_RETURN では永久に待ち続けていた）。
    """
    p1, p2 = _station(settle_timeout_s=2.0), _station(_plan(35.0))
    loop, *_ = _loop([p1, p2])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(p1.plan, 9.0)
    loop._grid_station_begun = True
    loop._grid_settle_started = 0.0
    loop._phase = _Phase.GRID_RETURN
    for t in (0.0, 1.0, 2.1):  # 車速が動かない（釣り合ってしまい目標に届かない）まま打ち切り
        loop._advance_grid_return(p1, 0.0, t)
    assert loop._pattern_idx == 1 and loop._phase is _Phase.CRUISE_HOLD
    assert loop.grid_planner.remaining == 0


def test_advance_grid_return_overspeed_enters_recovery_brake() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)
    loop._phase = _Phase.GRID_RETURN
    loop._advance_grid_return(pattern, loop._profile.max_speed + 1.0, 0.1)
    assert loop._phase is _Phase.DRIVE_BRAKE and loop._overspeed_recovery


def test_begin_return_stays_in_pi_when_station_not_yet_begun() -> None:
    """u0 未定（ステーションの最初）は、目標から離れていても GRID_RETURN に入らない。"""
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)
    loop._grid_station_begun = False
    loop._phase = _Phase.CRUISE_HOLD
    loop._grid_begin_return(pattern, 0.0, pattern.plan.speed_kmh, 0.0, is_approach=False)
    assert loop._phase is _Phase.CRUISE_HOLD


def test_step_end_far_from_center_returns_via_grid_return_then_settles() -> None:
    """ステップの後、目標から離れていれば GRID_RETURN を経て CRUISE_HOLD の落ち着き判定へ戻る。"""
    pattern = _station(_plan(25.0))
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    t = _settle(loop, pattern, 7.0)
    loop._advance_grid_step(pattern, 40.0, t + 0.1)  # 帯の外まで離れて終わる
    assert loop._phase is _Phase.GRID_RETURN
    switch = pattern.settings.grid_return_switch_kmh
    loop._advance_grid_return(pattern, 25.0 - switch, t + 0.2)  # 目標近くまで戻ってきた
    assert loop._phase is _Phase.CRUISE_HOLD
    # 打ち切りタイマーは戻り試行の頭（ステップ終了時）から進み続ける
    assert loop._grid_settle_started == pytest.approx(t + 0.1)


def test_governor_applies_in_grid_return_and_caps_the_command() -> None:
    pattern = _station()
    loop, *_ = _loop([pattern])
    assert loop.grid_planner is not None
    loop.grid_planner.begin_station(pattern.plan, 9.0)
    loop.grid_planner.g_accel = 0.2  # 小さい g → 大きく踏む開度になる
    loop._grid_station_begun = True
    # 段7c: GRID_RETURN の始点は _grid_begin_return が決める（_grid_return_opening は
    # 以降 _gr_opening をそのまま返すだけになったため）
    loop._grid_begin_return(pattern, 0.0, pattern.plan.speed_kmh, 0.0, is_approach=False)
    assert loop._phase is _Phase.GRID_RETURN
    loop._phase_started_at = 0.0
    loop._last_speed = 0.0  # 目標より遅い → アクセルを踏む
    raw, _ = loop._command_openings(pattern, 0.0)  # _accel_request を残す（頭打ち前）
    loop._current_accel_opening = raw
    loop._update_governor(loop._g_limit_kmhs + 1.0)  # 初回は現在開度で頭打ち
    loop._update_governor(loop._g_limit_kmhs + 1.0)  # 以降は 1 周期ごとに下げる
    accel, brake = loop._command_openings(pattern, 0.0)
    assert brake == 0.0 and accel < raw
    assert loop._governor_limiting()




# ── プラントつきの完走 ─────────────────────────────────────────────────


class _PlantCAN:
    """開度指令から車速を積分する簡易プラント（時間は実時間。ゲインは実車の 10 倍速の世界）。

    a = G × (アクセル − 遊び)+ − GB × (ブレーキ − 遊び)+ − C（走行中）[km/h/s]
    """

    G, GB, C = 12.0, 12.0, 8.0

    def __init__(self, accel: FakeAxis, brake: FakeAxis, play_a: float, play_b: float) -> None:
        from tests.research.vehicle import pulse_to_opening

        self._accel, self._brake, self._to_opening = accel, brake, pulse_to_opening
        self.PLAY_A, self.PLAY_B = play_a, play_b  # プロファイルの不感帯（PI の下限）に合わせる
        self.speed_kmh = 0.0
        self._t: float | None = None

    async def read_speed(self) -> float:
        now = asyncio.get_running_loop().time()
        dt = 0.0 if self._t is None else min(now - self._t, 0.2)
        self._t = now
        ua, ub = self._to_opening(self._accel.position), self._to_opening(self._brake.position)
        a = self.G * max(0.0, ua - self.PLAY_A) - self.GB * max(0.0, ub - self.PLAY_B)
        if self.speed_kmh > 0.5:
            a -= self.C
        self.speed_kmh = max(0.0, self.speed_kmh + a * dt)
        return self.speed_kmh


@pytest.mark.asyncio
async def test_grid_runs_station_and_launch_to_completion_on_a_plant() -> None:
    settings = _settings(
        settle_s=0.4, settle_timeout_s=8.0, step_window_s=0.4, step_lag_s=0.1,
        gain_init=12.0, brake_gain_init=12.0, hold_kp_norm=4.0, hold_ki_norm=4.0,
        hold_max_rate_pct_s=1.0, settle_tol_kmh=1.5, step_band_max_kmh=8.0,
    )
    station = plmod.GridStationPattern(
        PatternKind.GRID_STEP, accel_opening=0.0, brake_opening=0.0, hold_duration_s=1.0,
        plan=StationPlan(
            25.0, 20.0, 30.0, decel=(Target(-3.0, -12.0, -1.0),),
            accel=(Target(6.0, 1.0, 40.0),), a_max_kmhs=30.0, a_min_kmhs=-30.0,
        ),
        settings=settings,
    )
    launch = plmod.GridLaunchPattern(
        PatternKind.GRID_LAUNCH, accel_opening=0.0, brake_opening=0.0, hold_duration_s=1.0,
        plan=LaunchPlan(
            20.0, (Target(8.0, 1.0, 40.0),), (Target(-12.0, -40.0, -1.0),), 30.0, -30.0
        ),
        settings=settings,
    )
    accel, brake = FakeAxis(), FakeAxis()
    # 簡易プラントは応答の遅れが無いので、先読み（gov_lead_s）は切る
    config = PatternLoopConfig(accel_ramp_time_s=0.0, brake_ramp_time_s=0.0, gov_lead_s=0.0)
    loop, rec, _, *_ = _loop([station, launch], config=config, accel=accel, brake=brake,
                             interval_s=0.02)
    ff = loop._profile.feedforward_params
    loop._can_reader = _PlantCAN(  # type: ignore[assignment]
        accel, brake, ff.accel_deadband_pct, ff.brake_deadband_pct
    )
    await _run_until_done(loop, rec, timeout_s=60.0)
    assert rec.completed and not rec.emergency, loop.abort_reason
    planner = loop.grid_planner
    assert planner is not None
    kinds = [r.kind for r in planner.results]
    assert kinds[:2] == [StepKind.HOLD, StepKind.COAST]
    assert StepKind.ACCEL in kinds and StepKind.LAUNCH in kinds and StepKind.STOP in kinds
    coast = next(r for r in planner.results if r.kind is StepKind.COAST)
    assert coast.a_meas_kmhs == pytest.approx(-_PlantCAN.C, rel=0.3)  # 惰行の実測
    phases = {phase for _, _, phase, *_ in rec.rows}
    assert {"CRUISE_HOLD", "HOLD_STEP"} <= phases
    # 感度がプラントの真値（12）へ寄る
    assert planner.g_accel == pytest.approx(_PlantCAN.G, rel=0.35)


# ── 段5b: 停車ステップのクリープ平衡・当てはめ・窓の傾き ───────────────


def _to_stop_step(loop: PatternLoop, pattern: plmod.GridLaunchPattern) -> float:
    """発進を 20 km/h まで走らせて、停車ステップの先頭に来る。戻り値は現在時刻。"""
    loop._grid_enter_pattern(pattern, 0.0)
    t = 0.0
    while loop._grid_step is not None and loop._grid_step.kind is StepKind.LAUNCH:
        t += 0.1
        loop._advance_grid_step(pattern, min(1.0 * t, 20.0), t)
    assert loop._grid_step is not None and loop._grid_step.kind is StepKind.STOP
    return t


def test_stop_step_ends_when_it_settles_at_creep_speed() -> None:
    pattern = _launch(settle_timeout_s=60.0)
    loop, *_ = _loop([pattern])
    creep = loop._profile.feedforward_params.creep_speed_kmh
    t = _to_stop_step(loop, pattern)
    t0 = t
    while loop._grid_step is not None and loop._grid_step.kind is StepKind.STOP:
        t += 0.1
        # 20 km/h から −2 km/h/s で下がり、クリープ速度で釣り合って動かなくなる
        loop._advance_grid_step(pattern, max(20.0 - 2.0 * (t - t0), creep), t)
        assert t - t0 < 30.0  # 打ち切り 60s まで待たない
    assert loop.grid_planner.results[1].kind is StepKind.STOP  # type: ignore[union-attr]


def test_launch_stop_fit_ignores_creep_range_samples() -> None:
    pattern = _launch(settle_timeout_s=60.0)
    loop, *_ = _loop([pattern])
    creep = loop._profile.feedforward_params.creep_speed_kmh
    t = _to_stop_step(loop, pattern)
    t0 = t
    while loop._grid_step is not None and loop._grid_step.kind is StepKind.STOP:
        t += 0.1
        loop._advance_grid_step(pattern, max(20.0 - 2.0 * (t - t0), creep), t)
    result = loop.grid_planner.results[1]  # type: ignore[union-attr]
    # クリープの平らな区間を含めると傾きが 0 寄りになる。除いているので −2 のまま
    assert result.a_meas_kmhs == pytest.approx(-2.0, abs=0.1)


def test_settle_record_carries_slope_of_the_settled_window() -> None:
    pattern = _station(_plan(25.0))
    loop, *_ = _loop([pattern])
    loop._grid_enter_pattern(pattern, 0.0)
    for i, t in enumerate([0.0, 0.25, 0.5, 0.75, 1.0]):
        loop._current_accel_opening = 7.0
        loop._advance_grid_hold(pattern, 24.5 + 0.4 * t, t)  # 近づいている途中: +0.4 km/h/s
    (rec,) = loop.grid_settles
    assert rec.slope_kmhs == pytest.approx(0.4, abs=1e-6)
