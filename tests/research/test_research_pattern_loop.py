"""研究用パターン走行ループ（pattern_loop.PatternLoop）の A6 変更と安全網のユニットテスト。

2026-09-13 A6（KAIZEN 表5-5 順3）で入れたもの:
    - ガバナーの解除（上限 × gov_release_frac を下回ったら頭打ちを戻し、指令に届いたら解除）
    - 全運転パターンの後の停車復帰（DRIVE_BRAKE。停車してから次のパターンへ）
    - DRIVE_BRAKE の打ち切り = 停車しなければ中断する安全上限
    - ブレーキのランプをフェーズに入ったときの開度から始める
    - 走行中の安全網（axis_safety.AxisSafetyNet: アラーム確認・電流ゼロの継続）

2026-09-25 段4: ACCEL_SWEEP・BRAKE_HOLD・CRUISE_TRIM・各階段パターンは削除した。ここでは残った
COAST_DOWN・クリープ発進（ブレーキ保持は BRAKE_HOLD フェーズ）で同じ機構（ガバナー・停車復帰・
ランプ・安全網）を確かめる。G ランプ走行は test_research_pattern_loop_ramp.py。
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import pattern_loop as plmod
from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.pattern_loop import PatternLoop, PatternLoopConfig, _Phase
from tests.research.research_types import G_TO_KMHS, DriveLogData, LearningPattern, PatternKind
from tests.research.vehicle import build_vehicle_profile

CFG_PATH = Path("tests/research/config_testVehicle.yaml")


class FakeAxis:
    """位置指令を覚え、決めた電流・アラームを返すアクチュエータ。"""

    def __init__(self, current_ma: float = 0.0) -> None:
        self.position = 0
        # 送り方の記録: ("timed" | "smooth", 目標位置, 時間引数)
        self.calls: list[tuple[str, int, float | None]] = []
        self.current_ma = current_ma
        self.alarm = False
        self.current_script: list[float] | None = None

    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None:
        self.position = target_pos
        self.calls.append(("timed", target_pos, duration_s))

    async def move_to_position(
        self, pos: int, *, smooth_over_s: float | None = None
    ) -> None:
        self.position = pos
        self.calls.append(("smooth", pos, smooth_over_s))

    async def read_monitor(self) -> AxisMonitor:
        current = self.current_script.pop(0) if self.current_script else self.current_ma
        return AxisMonitor(
            position_pulse=self.position, current_ma=current, alarm_code=1 if self.alarm else 0,
            servo_on=True, moving=False, pos_done=True,
        )

    async def is_alarm_active(self) -> bool:
        return self.alarm


class FakeCAN:
    def __init__(self, speed_kmh: float) -> None:
        self.speed_kmh = speed_kmh

    async def read_speed(self) -> float:
        return self.speed_kmh


class Recorder:
    def __init__(self) -> None:
        self.rows: list[tuple[DriveLogData, int, str, bool, bool | None, bool | None]] = []
        self.monitors: list[tuple[AxisMonitor, AxisMonitor, float]] = []
        self.completed = False
        self.emergency = False

    def on_sample(
        self,
        data: DriveLogData,
        idx: int,
        phase: str,
        governed: bool,
        alarm_accel: bool | None,
        alarm_brake: bool | None,
        monitor_accel: AxisMonitor,
        monitor_brake: AxisMonitor,
        cycle_ms: float,
    ) -> None:
        self.rows.append((data, idx, phase, governed, alarm_accel, alarm_brake))
        self.monitors.append((monitor_accel, monitor_brake, cycle_ms))

    async def on_complete(self) -> None:
        self.completed = True

    async def on_emergency(self) -> None:
        self.emergency = True


def _loop(
    patterns: list[LearningPattern],
    *,
    speed_kmh: float = 0.0,
    config: PatternLoopConfig | None = None,
    accel: FakeAxis | None = None,
    brake: FakeAxis | None = None,
    interval_s: float = 0.02,
) -> tuple[PatternLoop, Recorder, FakeCAN, FakeAxis, FakeAxis]:
    cfg = cfgmod.load_config(CFG_PATH)
    profile = build_vehicle_profile(cfg)
    rec = Recorder()
    can = FakeCAN(speed_kmh)
    accel = accel or FakeAxis()
    brake = brake or FakeAxis()
    loop = PatternLoop(
        accel_driver=accel,
        brake_driver=brake,
        can_reader=can,
        profile=profile,
        patterns=patterns,
        overcurrent_limit_ma=5000.0,
        on_complete=rec.on_complete,
        on_emergency=rec.on_emergency,
        on_sample=rec.on_sample,
        config=config,
        interval_s=interval_s,
    )
    return loop, rec, can, accel, brake


def _pattern(
    kind: PatternKind, *, accel: float = 0.0, brake: float = 0.0, **kw: float
) -> LearningPattern:
    return LearningPattern(kind, accel_opening=accel, brake_opening=brake,
                           hold_duration_s=kw.pop("hold", 0.3), **kw)


async def _run_until_done(loop: PatternLoop, rec: Recorder, timeout_s: float = 5.0) -> None:
    loop.start()
    try:
        async with asyncio.timeout(timeout_s):
            while not (rec.completed or rec.emergency):
                await asyncio.sleep(0.01)
    finally:
        await loop.stop_and_join()


# ── ガバナーの解除 ─────────────────────────────────────────────────────


def test_governor_reduces_holds_raises_and_releases() -> None:
    loop, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=40.0)])
    limit = loop._g_limit_kmhs
    assert limit == pytest.approx(0.4 * G_TO_KMHS * 0.98)
    cfg = loop._config

    # 上限以上: 最初は現在開度で頭打ち → 以降 2%/0.1 秒（この周期 0.02 秒なら 0.4%/周期）で下げる
    cap = loop._next_gov_cap(None, limit + 1.0, 0.0, current=30.0, request=30.0)
    assert cap == pytest.approx(30.0)
    cap = loop._next_gov_cap(cap, limit + 1.0, 0.0, current=30.0, request=30.0)
    assert cap == pytest.approx(30.0 - cfg.gov_reduce_step_pct * 0.02 / 0.1)
    # 上限 × 0.7〜1.0: 保持
    held = loop._next_gov_cap(cap, limit * 0.8, 0.0, current=28.0, request=30.0)
    assert held == pytest.approx(cap)
    # 上限 × 0.7 未満: 0.5%/0.1 秒（この周期 0.02 秒なら 0.1%/周期）で戻す
    raised = loop._next_gov_cap(cap, limit * 0.5, 0.0, current=28.0, request=30.0)
    assert raised == pytest.approx(cap + cfg.gov_raise_step_pct * 0.02 / 0.1)
    # 指令に届いたら解除
    assert loop._next_gov_cap(29.8, limit * 0.5, 0.0, current=29.8, request=30.0) is None
    # 頭打ちが無く上限未満なら何もしない
    assert loop._next_gov_cap(None, limit * 0.5, 0.0, current=30.0, request=30.0) is None
    # 下限は 0%
    assert loop._next_gov_cap(1.0, limit + 1.0, 0.0, current=1.0, request=30.0) == 0.0


def test_governor_steps_are_per_second_regardless_of_the_cycle() -> None:
    """段2l-11: 下げ・戻しの刻みは 0.1 秒周期での 1 周期ぶん。周期 0.05 秒と 0.1 秒で
    1 秒あたりの量が同じ（下げ 20 %/秒・戻し 5 %/秒）。"""
    per_second = {}
    for interval in (0.05, 0.1):
        loop, *_ = _loop(
            [_pattern(PatternKind.COAST_DOWN, accel=40.0)], interval_s=interval
        )
        limit = loop._g_limit_kmhs
        n = round(1.0 / interval)
        cap: float | None = 30.0
        for _ in range(n):
            cap = loop._next_gov_cap(cap, limit + 1.0, 0.0, current=30.0, request=30.0)
        assert cap is not None
        down = 30.0 - cap
        for _ in range(n):
            cap = loop._next_gov_cap(cap, limit * 0.5, 0.0, current=10.0, request=60.0)
        assert cap is not None
        per_second[interval] = (down, cap - (30.0 - down))
    assert per_second[0.05][0] == pytest.approx(20.0)
    assert per_second[0.1][0] == pytest.approx(20.0)
    assert per_second[0.05][1] == pytest.approx(5.0)
    assert per_second[0.1][1] == pytest.approx(5.0)


def test_update_governor_uses_brake_deceleration_in_drive_brake() -> None:
    loop, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=40.0)])
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    loop._current_brake_opening = 30.0
    loop._brake_request = 30.0
    limit = loop._g_limit_kmhs
    loop._update_governor(-(limit + 1.0))  # 減速が上限超え
    assert loop._brake_gov_cap == pytest.approx(30.0)
    loop._update_governor(-(limit + 1.0))
    assert loop._brake_gov_cap == pytest.approx(30.0 - 0.4)  # 2%/0.1 秒 × 周期 0.02 秒
    assert loop._governor_limiting()
    loop._update_governor(-1.0)  # 減速が十分小さい → 戻す
    assert loop._brake_gov_cap == pytest.approx(30.0 - 0.4 + 0.1)
    assert loop._accel_gov_cap is None  # ブレーキ中はアクセル側を触らない


def test_governor_predicts_ahead_and_freezes_the_press_before_the_limit() -> None:
    """段6b: 見込み = 今 + 増える速さ × 0.6 s。上限 × 0.85 に届く見込みなら踏み増しを止める。"""
    loop, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=40.0)])
    limit, cfg = loop._g_limit_kmhs, loop._config
    soft = limit * cfg.gov_soft_frac
    # 今は上限の半分でも、増える速さで 0.6 s 後に soft に届く → 現在開度で頭打ち（踏み増し停止）
    rate = (soft - 0.5 * limit) / cfg.gov_lead_s + 0.1
    assert loop._next_gov_cap(None, 0.5 * limit, rate, current=12.0, request=30.0) == 12.0
    # 頭打ち済みなら保持（下げない・戻さない）
    assert loop._next_gov_cap(12.0, 0.5 * limit, rate, current=12.0, request=30.0) == 12.0
    # 増える速さが小さければ従来どおり（頭打ちなし）
    assert loop._next_gov_cap(None, 0.5 * limit, 0.0, current=12.0, request=30.0) is None
    # 見込みが上限に届くなら下げる（頭打ち済み）
    over = (limit - 0.5 * limit) / cfg.gov_lead_s + 0.1
    assert loop._next_gov_cap(12.0, 0.5 * limit, over, current=12.0, request=30.0) == pytest.approx(
        12.0 - cfg.gov_reduce_step_pct * 0.02 / 0.1
    )
    # G が減っていく（負の速さ）ときは見込みを増やさない
    assert loop._next_gov_cap(None, 0.5 * limit, -50.0, current=12.0, request=30.0) is None


def test_accel_rate_is_the_slope_of_the_recent_smoothed_accel() -> None:
    loop, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=40.0)])
    for i in range(10):  # 0.1 s 周期で加速度が 10 km/h/s² で増える
        loop._update_accel_rate(i * 0.1, 10.0 * i * 0.1)
    assert loop._accel_rate == pytest.approx(10.0, rel=0.01)


# ── コーストダウン加速（G の余裕に比例して踏む） ───────────────────────


def _coast_loop(
    accel: float = 70.0, config: PatternLoopConfig | None = None
) -> tuple[PatternLoop, LearningPattern, float]:
    """(ループ, パターン, 接近開度 [%])。DRIVE_ACCEL を now=0 で始めた状態。"""
    loop, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=accel)], config=config)
    cfg = loop._config
    deadband = loop._profile.feedforward_params.accel_deadband_pct
    approach = min(accel, max(0.0, deadband - cfg.coast_accel_approach_margin_pct))
    loop._enter_phase(_Phase.DRIVE_ACCEL, 0.0)
    return loop, loop._patterns[0], approach


def _feed(loop: PatternLoop, pattern: LearningPattern, t: float, speed: float) -> None:
    """時刻 t・車速 speed の 1 周期分（指令 → 前進判定）。"""
    loop._command_openings(pattern, t)
    loop._advance(pattern, speed, 0.0, t)


def _run_ramp(
    loop: PatternLoop, pattern: LearningPattern, t_from: float, t_to: float, kmhs: float,
    v0: float,
) -> float:
    """t_from〜t_to を 0.1s 周期で、車速が v0 から kmhs [km/h/s] で増えるとして流す。"""
    i = round(t_from * 10)
    while i <= round(t_to * 10):  # 0.1s 周期。時刻は丸めて、判定時刻の取りこぼしを避ける
        t = round(i * 0.1, 6)
        _feed(loop, pattern, t, v0 + kmhs * t)
        i += 1
    return loop._command_openings(pattern, t_to)[0]


def test_coast_accel_first_ramps_quickly_to_just_below_deadband() -> None:
    loop, pattern, approach = _coast_loop()
    cfg = loop._config
    assert approach > 0.0
    assert loop._command_openings(pattern, 0.0)[0] == pytest.approx(0.0)
    half = loop._command_openings(pattern, cfg.accel_ramp_time_s / 2)[0]
    assert half == pytest.approx(approach / 2)
    assert loop._command_openings(pattern, cfg.accel_ramp_time_s)[0] == pytest.approx(approach)


def test_coast_accel_presses_in_proportion_to_g_margin() -> None:
    loop, pattern, approach = _coast_loop()
    cfg = loop._config
    t0 = cfg.accel_ramp_time_s
    _run_ramp(loop, pattern, 0.0, t0, 0.0, 10.0)  # 接近まで（車速一定）
    # 加速していない（G=0）: 1 s で rate_gain × target_g [%] 上がる
    top = _run_ramp(loop, pattern, t0 + 0.1, t0 + 1.0, 0.0, 10.0)
    assert top == pytest.approx(approach + cfg.coast_accel_rate_gain * cfg.coast_accel_target_g)
    # 目標の半分の G なら、踏む速さも半分
    half_g = 0.5 * cfg.coast_accel_target_g * G_TO_KMHS
    loop2, pattern2, approach2 = _coast_loop()
    _run_ramp(loop2, pattern2, 0.0, t0, half_g, 10.0)
    before = loop2._command_openings(pattern2, t0)[0]
    after = _run_ramp(loop2, pattern2, t0 + 0.1, t0 + 1.0, half_g, 10.0)
    assert after - before == pytest.approx(
        cfg.coast_accel_rate_gain * 0.5 * cfg.coast_accel_target_g, rel=0.15
    )


def test_coast_accel_holds_within_margin_and_backs_off_above_target() -> None:
    loop, pattern, approach = _coast_loop()
    cfg = loop._config
    t0 = cfg.accel_ramp_time_s
    on_target = cfg.coast_accel_target_g * G_TO_KMHS
    _run_ramp(loop, pattern, 0.0, t0 + 2.0, on_target, 10.0)  # 目標ちょうど → 保持
    assert loop._ca_opening == pytest.approx(approach)
    # 目標超えでも接近位置より浅くはしない
    too_fast = (cfg.coast_accel_target_g + 0.1) * G_TO_KMHS
    _run_ramp(loop, pattern, t0 + 2.1, t0 + 4.0, too_fast, 10.0)
    assert loop._ca_opening == pytest.approx(approach)
    # 踏み増した後に目標超えなら戻る
    loop._ca_opening = approach + 10.0
    before = loop._ca_opening
    _run_ramp(loop, pattern, t0 + 4.1, t0 + 5.0, too_fast, 10.0)
    assert loop._ca_opening < before


def test_coast_accel_command_waits_for_lagging_actual_opening() -> None:
    loop, pattern, approach = _coast_loop()
    cfg = loop._config
    t0 = cfg.accel_ramp_time_s
    _run_ramp(loop, pattern, 0.0, t0, 0.0, 10.0)
    loop._accel_actual_pct = approach  # 実開度が動かない（遅れている）
    _run_ramp(loop, pattern, t0 + 0.1, t0 + 10.0, 0.0, 10.0)  # G=0 で長く流す
    assert loop._ca_opening <= approach + cfg.ramp_actuator_lag_tol_pct + 1e-9
    assert loop._ca_opening > approach  # 許容ずれぶんまでは踏む


def test_coast_accel_not_limited_when_actual_follows_command() -> None:
    loop, pattern, approach = _coast_loop()
    cfg = loop._config
    t0 = cfg.accel_ramp_time_s
    _run_ramp(loop, pattern, 0.0, t0, 0.0, 10.0)
    loop._accel_actual_pct = loop._ca_opening
    t = t0 + 0.1
    while t <= t0 + 1.0 + 1e-9:
        _feed(loop, pattern, round(t, 6), 10.0)
        loop._accel_actual_pct = loop._ca_opening  # 実開度が指令に追従
        t += 0.1
    assert loop._ca_opening == pytest.approx(
        approach + cfg.coast_accel_rate_gain * cfg.coast_accel_target_g, rel=0.15
    )


def test_coast_accel_limit_does_not_hold_back_or_lower_command() -> None:
    loop, pattern, approach = _coast_loop()
    cfg = loop._config
    t0 = cfg.accel_ramp_time_s
    _run_ramp(loop, pattern, 0.0, t0, 0.0, 10.0)
    loop._accel_actual_pct = approach
    start = approach + cfg.ramp_actuator_lag_tol_pct + 10.0  # 許容ずれより先行済み
    # G 不足: 踏み増しは止まるが、指令は下がらない（据え置き）
    loop._ca_opening = start
    _run_ramp(loop, pattern, t0 + 0.1, t0 + 2.0, 0.0, 10.0)
    assert loop._ca_opening == pytest.approx(start)
    # G 超過: 従来どおり下がる
    too_fast = (cfg.coast_accel_target_g + 0.1) * G_TO_KMHS
    _run_ramp(loop, pattern, t0 + 2.1, t0 + 3.0, too_fast, 10.0)
    assert loop._ca_opening < start


def test_coast_accel_never_exceeds_pattern_accel_opening() -> None:
    config = PatternLoopConfig(accel_full_range_timeout_s=100.0)  # 頭打ちまで踏み進める時間
    loop, pattern, _ = _coast_loop(accel=25.0, config=config)
    cfg = loop._config
    t = 0.0
    top = 0.0
    while t < 200.0:  # 加速が出ないまま踏み続ける
        top = max(top, loop._command_openings(pattern, t)[0])
        loop._advance(pattern, 10.0, 0.0, t)
        if loop._phase is not _Phase.DRIVE_ACCEL:
            break
        t += 0.1
    assert top == pytest.approx(25.0)
    assert loop._ca_opening == pytest.approx(25.0)
    # 上限で待機したまま、accel_full_range_timeout_s で打ち切って惰行へ進む
    assert loop._phase is _Phase.COAST
    assert t >= cfg.accel_full_range_timeout_s - 0.2


def test_coast_accel_reaching_speed_cap_goes_to_coast() -> None:
    loop, pattern, _ = _coast_loop()
    cap = loop._accel_speed_cap
    _feed(loop, pattern, 0.0, 5.0)
    assert loop._phase is _Phase.DRIVE_ACCEL
    _feed(loop, pattern, 0.1, cap + 0.1)
    assert loop._phase is _Phase.COAST


def test_coast_accel_defaults_match_decel_stop_section() -> None:
    d = cfgmod.load_config(CFG_PATH).decel_stop
    c = PatternLoopConfig()
    assert (c.coast_accel_target_g, c.coast_accel_press_margin_g,
            c.coast_accel_slope_window_s, c.coast_accel_approach_margin_pct) == (
        d.target_decel_g, d.press_margin_g, d.slope_window_s, d.approach_margin_pct)


# ── 停車復帰・ランプ・打ち切り ─────────────────────────────────────────


def test_brake_ramp_starts_from_opening_at_phase_entry() -> None:
    # コーストダウン以外（徐々に踏むのはコーストダウンだけ。2026-10-08 段2l-6b）
    loop, *_ = _loop([_pattern(PatternKind.CREEP, brake=15.0)])
    loop._current_brake_opening = 15.0  # BRAKE_HOLD（クリープ域ブレーキ保持）で保持していた開度
    loop._enter_phase(_Phase.DRIVE_BRAKE, 10.0)
    pattern = loop._patterns[0]
    accel, brake = loop._command_openings(pattern, 10.0)
    assert (accel, brake) == (0.0, pytest.approx(15.0))  # 0% に抜けない
    ramp = loop._config.brake_ramp_time_s
    _, brake = loop._command_openings(pattern, 10.0 + ramp)
    # 停車復帰の目標は停車保持開度（パターンの保持開度ではない）
    assert brake == pytest.approx(loop._stop_return_brake_pct)
    assert loop._stop_return_brake_pct > 15.0


@pytest.mark.parametrize("phase", [_Phase.BRAKE_HOLD, _Phase.COAST])
def test_driving_patterns_end_with_stop_return(phase: _Phase) -> None:
    config = PatternLoopConfig(brake_hold_timeout_s=1.0, coast_timeout_s=1.0)
    patterns = [_pattern(PatternKind.COAST_DOWN, accel=60.0, brake=15.0, hold=1.0),
                _pattern(PatternKind.COAST_DOWN, accel=30.0)]
    loop, *_ = _loop(patterns, config=config)
    loop._enter_phase(phase, 0.0)
    # 保持・惰行が終わっても車速が残っていれば、次へ進まず停車復帰に入る
    loop._advance(patterns[0], 40.0, 0.0, 2.0)
    assert (loop._pattern_idx, loop._phase) == (0, _Phase.DRIVE_BRAKE)
    # 停車しても、停車保持開度に届くまで（徐々に踏む。段2l-6b）は次へ進まない
    loop._command_openings(patterns[0], 3.0)
    loop._advance(patterns[0], 0.0, 0.0, 3.0)
    assert (loop._pattern_idx, loop._phase) == (0, _Phase.DRIVE_BRAKE)
    # 届いたら次のパターンへ
    t_reach = 3.0 + loop._stop_return_brake_pct / loop._stop_return_press_rate_pctps() + 1.0
    loop._command_openings(patterns[0], t_reach)
    loop._advance(patterns[0], 0.0, 0.0, t_reach)
    assert (loop._pattern_idx, loop._phase) == (1, _Phase.DRIVE_ACCEL)


def test_brake_hold_that_already_stopped_goes_straight_to_next() -> None:
    patterns = [_pattern(PatternKind.COAST_DOWN, accel=60.0, brake=20.0),
                _pattern(PatternKind.COAST_DOWN, accel=60.0)]
    loop, *_ = _loop(patterns)
    loop._enter_phase(_Phase.BRAKE_HOLD, 0.0)
    loop._advance(patterns[0], 0.0, 0.0, 1.0)
    assert (loop._pattern_idx, loop._phase) == (1, _Phase.DRIVE_ACCEL)


# ── クリープ発進・クリープ域ブレーキ保持（段1。2026-09-17。ProblemReport_20260916 課題#2） ──


def _creep_launch(
    *, target_kmh: float = 5.0, timeout_s: float = 20.0,
    hold_after: bool = False, brake_opening: float = 0.0,
) -> plmod.CreepLaunchPattern:
    return plmod.CreepLaunchPattern(
        PatternKind.CREEP_SETTLE, accel_opening=0.0, brake_opening=brake_opening,
        hold_duration_s=timeout_s, target_kmh=target_kmh, timeout_s=timeout_s,
        hold_after=hold_after,
    )


def test_initial_phase_for_creep_launch_pattern_is_creep_launch() -> None:
    loop, *_ = _loop([_creep_launch()])
    assert loop._initial_phase(0) is _Phase.CREEP_LAUNCH


def test_creep_launch_commands_both_pedals_released() -> None:
    pattern = _creep_launch()
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)
    assert loop._command_openings(pattern, 0.0) == (0.0, 0.0)
    assert loop._command_openings(pattern, 5.0) == (0.0, 0.0)  # 時間が経っても常に両ペダル解放


def test_creep_launch_reaches_target_goes_to_drive_brake_when_not_holding() -> None:
    pattern = _creep_launch(target_kmh=5.0, hold_after=False)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)
    loop._advance(pattern, 4.9, 0.0, 1.0)
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 5.0, 0.0, 2.0)  # 目標到達
    assert loop._phase is _Phase.DRIVE_BRAKE
    assert not loop._overspeed_recovery


def test_creep_launch_reaches_target_goes_to_brake_hold_when_holding() -> None:
    pattern = _creep_launch(target_kmh=5.0, hold_after=True, brake_opening=15.0)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)
    loop._advance(pattern, 5.0, 0.0, 1.0)  # 目標到達
    assert loop._phase is _Phase.BRAKE_HOLD
    _, brake = loop._command_openings(pattern, 1.0 + loop._config.brake_ramp_time_s)
    assert brake == pytest.approx(15.0)  # BRAKE_HOLD は pattern.brake_opening を保持する


def test_creep_launch_timeout_advances_even_below_target() -> None:
    pattern = _creep_launch(target_kmh=100.0, timeout_s=2.0, hold_after=False)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)
    loop._advance(pattern, 3.0, 0.0, 1.9)
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 3.0, 0.0, 2.0)  # timeout_s 経過 → 目標未到達でも打ち切り
    assert loop._phase is _Phase.DRIVE_BRAKE


def test_creep_launch_overspeed_enters_drive_brake_recovery() -> None:
    pattern = _creep_launch()
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)
    loop._advance(pattern, loop._profile.max_speed + 1.0, 0.0, 1.0)
    assert loop._phase is _Phase.DRIVE_BRAKE and loop._overspeed_recovery


async def test_creep_launch_runs_to_completion_via_timeout() -> None:
    """速度が動かないスタブ CAN でも timeout_s で打ち切られ、停車済みなのでそのまま完了する。"""
    config = PatternLoopConfig(brake_ramp_time_s=0.0, brake_stop_timeout_s=2.0)
    pattern = _creep_launch(target_kmh=100.0, timeout_s=0.2, hold_after=False)
    loop, rec, _, *_ = _loop([pattern], speed_kmh=0.0, config=config, interval_s=0.02)
    await _run_until_done(loop, rec)
    assert rec.completed and not rec.emergency
    phases = {phase for _, _, phase, *_ in rec.rows}
    assert phases == {"CREEP_LAUNCH"}


async def test_creep_launch_hold_runs_to_completion_via_timeout() -> None:
    config = PatternLoopConfig(brake_ramp_time_s=0.0, brake_hold_timeout_s=2.0)
    pattern = _creep_launch(target_kmh=100.0, timeout_s=0.2, hold_after=True, brake_opening=15.0)
    loop, rec, _, *_ = _loop([pattern], speed_kmh=0.0, config=config, interval_s=0.02)
    await _run_until_done(loop, rec)
    assert rec.completed and not rec.emergency
    phases = {phase for _, _, phase, *_ in rec.rows}
    assert "BRAKE_HOLD" in phases


def test_defaults_are_a6_values() -> None:
    cfg = PatternLoopConfig()
    assert cfg.accel_full_range_timeout_s == 60.0
    assert cfg.brake_stop_timeout_s == 60.0
    assert (cfg.gov_release_frac, cfg.gov_raise_step_pct) == (0.7, 0.5)


# ── クリープ発進の終了条件（段1b。2026-09-17。ProblemReport_20260916 ユーザー決定） ──


def test_creep_launch_ends_when_slope_settles() -> None:
    """target_kmh 到達を待たず、車速の傾きが settle_kmhs 未満で settle_s 続いたら終了する。

    settle_min_s は 0 にして最短時間ガードを無効化し、傾き収束の判定だけを見る。
    """
    config = PatternLoopConfig(
        creep_launch_settle_kmhs=0.1, creep_launch_settle_s=0.06, creep_launch_settle_min_s=0.0
    )
    pattern = _creep_launch(target_kmh=100.0, timeout_s=100.0, hold_after=False)
    loop, *_ = _loop([pattern], config=config, interval_s=0.02)
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)

    loop._advance(pattern, 4.90, 0.05, 0.02)  # 傾き 0.05 < 0.1 だが継続時間がまだ足りない
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 4.95, 0.05, 0.04)
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 4.97, 0.05, 0.06)  # 3周期 × 0.02s = 0.06s >= settle_s → 平衡到達
    assert loop._phase is _Phase.DRIVE_BRAKE


def test_creep_launch_slope_settle_resets_on_non_settled_sample() -> None:
    """途中で傾きがしきい値を超えたら、継続時間のカウントが振り出しに戻る。"""
    config = PatternLoopConfig(
        creep_launch_settle_kmhs=0.1, creep_launch_settle_s=0.06, creep_launch_settle_min_s=0.0
    )
    pattern = _creep_launch(target_kmh=100.0, timeout_s=100.0, hold_after=False)
    loop, *_ = _loop([pattern], config=config, interval_s=0.02)
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)

    loop._advance(pattern, 4.90, 0.05, 0.02)  # 安定 1 周期目
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 5.00, 0.50, 0.04)  # 傾きがしきい値を超える → カウントリセット
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 5.02, 0.05, 0.06)  # 安定 1 周期目（リセット後）。まだ足りない
    assert loop._phase is _Phase.CREEP_LAUNCH


# ── クリープ発進の誤判定修正（2026-09-17。停止直後を「平衡」と誤判定していたバグ） ──


def test_creep_launch_stationary_does_not_settle_even_past_settle_s() -> None:
    """回帰テスト: 修正前は、停車状態（速度0・傾き0）が settle_s を超えて続いただけで

    「平衡到達」と誤判定していた（停車保持を解放した直後は車速0・傾き0で
    abs(accel_kmhs) < creep_launch_settle_kmhs が最初から成立してしまうため）。
    車速が creep_launch_min_speed_kmh 未満の間は安定カウントを積まないよう直したので、
    動き出す前は settle_s をいくら超えても終了しない。
    """
    config = PatternLoopConfig(
        creep_launch_min_speed_kmh=1.0,
        creep_launch_settle_kmhs=0.1,
        creep_launch_settle_s=0.06,
        creep_launch_settle_min_s=0.0,
    )
    pattern = _creep_launch(target_kmh=100.0, timeout_s=100.0, hold_after=False)
    loop, *_ = _loop([pattern], config=config, interval_s=0.02)
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)

    # 速度0・傾き0が settle_s（0.06s）をとうに超えて続いても、動き出していないので終了しない
    for now in (0.02, 0.04, 0.06, 0.08, 0.10):
        loop._advance(pattern, 0.0, 0.0, now)
        assert loop._phase is _Phase.CREEP_LAUNCH


def test_creep_launch_settles_after_moving_starts() -> None:
    """動き出して（車速が creep_launch_min_speed_kmh 以上になって）から傾きが収束したら終了する。"""
    config = PatternLoopConfig(
        creep_launch_min_speed_kmh=1.0,
        creep_launch_settle_kmhs=0.1,
        creep_launch_settle_s=0.06,
        creep_launch_settle_min_s=0.0,
    )
    pattern = _creep_launch(target_kmh=100.0, timeout_s=100.0, hold_after=False)
    loop, *_ = _loop([pattern], config=config, interval_s=0.02)
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)

    # 停車保持解放直後: 動いていないので、傾きが小さくても安定カウントは積まれない
    loop._advance(pattern, 0.0, 0.0, 0.02)
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 0.5, 0.05, 0.04)  # まだ min_speed_kmh 未満
    assert loop._phase is _Phase.CREEP_LAUNCH
    # 動き出した（車速が min_speed_kmh 以上）後、傾きが収束し settle_s 続いたら終了
    loop._advance(pattern, 1.10, 0.05, 0.06)
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 1.15, 0.05, 0.08)
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 1.20, 0.05, 0.10)  # 3周期 × 0.02s = 0.06s >= settle_s → 平衡到達
    assert loop._phase is _Phase.DRIVE_BRAKE


def test_creep_launch_settle_min_s_blocks_early_finish() -> None:
    """動き出して傾きが収束していても、elapsed が settle_min_s 未満なら終了しない。"""
    config = PatternLoopConfig(
        creep_launch_min_speed_kmh=1.0,
        creep_launch_settle_kmhs=0.1,
        creep_launch_settle_s=0.02,
        creep_launch_settle_min_s=5.0,
    )
    pattern = _creep_launch(target_kmh=100.0, timeout_s=100.0, hold_after=False)
    loop, *_ = _loop([pattern], config=config, interval_s=0.02)
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)

    # 動いていて傾きも収束済み（settle_s は満たす）だが、elapsed(0.02s) < settle_min_s(5.0s)
    loop._advance(pattern, 1.10, 0.05, 0.02)
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 1.12, 0.05, 0.04)
    assert loop._phase is _Phase.CREEP_LAUNCH


def test_creep_launch_target_kmh_is_a_safety_cap() -> None:
    """平衡に達していなくても target_kmh に達したら安全上限として打ち切る。"""
    config = PatternLoopConfig(creep_launch_settle_kmhs=0.1, creep_launch_settle_s=100.0)
    pattern = _creep_launch(target_kmh=5.0, timeout_s=100.0, hold_after=False)
    loop, *_ = _loop([pattern], config=config, interval_s=0.02)
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)

    loop._advance(pattern, 4.9, 1.0, 0.02)  # 傾き 1.0（settle 条件は満たさない）
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 5.0, 1.0, 0.04)  # target_kmh 到達 → 安全上限で打ち切り
    assert loop._phase is _Phase.DRIVE_BRAKE


def test_creep_launch_settle_timeout_still_cuts_off() -> None:
    """平衡にもtarget_kmhにも達さなくても timeout_s で打ち切られる（従来どおり）。"""
    config = PatternLoopConfig(creep_launch_settle_kmhs=0.1, creep_launch_settle_s=100.0)
    pattern = _creep_launch(target_kmh=100.0, timeout_s=2.0, hold_after=False)
    loop, *_ = _loop([pattern], config=config, interval_s=0.02)
    loop._enter_phase(_Phase.CREEP_LAUNCH, 0.0)

    loop._advance(pattern, 3.0, 1.0, 1.9)
    assert loop._phase is _Phase.CREEP_LAUNCH
    loop._advance(pattern, 3.0, 1.0, 2.0)  # timeout_s 経過
    assert loop._phase is _Phase.DRIVE_BRAKE


async def test_stop_return_timeout_aborts_with_reason() -> None:
    """停車復帰が brake_stop_timeout_s を過ぎても停車しなければ、次へ進まず中断する。"""
    config = PatternLoopConfig(accel_full_range_timeout_s=0.1, coast_timeout_s=0.1,
                               brake_stop_timeout_s=0.3, accel_ramp_time_s=0.0,
                               brake_ramp_time_s=0.0)
    loop, rec, *_ = _loop(
        [_pattern(PatternKind.COAST_DOWN, accel=30.0)], speed_kmh=50.0, config=config
    )
    await _run_until_done(loop, rec)
    assert rec.emergency and not rec.completed
    assert loop.abort_reason is not None
    assert "停車しません" in loop.abort_reason and "DRIVE_BRAKE" in loop.abort_reason
    assert {r[2] for r in rec.rows} == {"DRIVE_ACCEL", "COAST", "DRIVE_BRAKE"}


async def test_loop_completes_after_stop_return() -> None:
    # calib_dwell_s を小さくして、停車保持開度まで徐々に踏む時間を短くする（段2l-6b）
    config = PatternLoopConfig(accel_full_range_timeout_s=0.1, coast_timeout_s=0.1,
                               accel_ramp_time_s=0.0, calib_dwell_s=0.01)
    loop, rec, can, *_ = _loop(
        [_pattern(PatternKind.COAST_DOWN, accel=30.0)], speed_kmh=50.0, config=config
    )
    loop.start()
    async with asyncio.timeout(5.0):
        while not any(r[2] == "DRIVE_BRAKE" for r in rec.rows):
            await asyncio.sleep(0.01)
        can.speed_kmh = 0.0
        while not (rec.completed or rec.emergency):
            await asyncio.sleep(0.01)
    await loop.stop_and_join()
    assert rec.completed and loop.abort_reason is None


# ── 安全網 ───────────────────────────────────────────────────────────


async def test_alarm_aborts_pattern_loop() -> None:
    brake = FakeAxis()
    brake.alarm = True
    loop, rec, *_ = _loop([_pattern(PatternKind.CREEP, brake=20.0, hold=5.0)], brake=brake)
    await _run_until_done(loop, rec)
    assert rec.emergency
    assert loop.abort_reason is not None and "ブレーキ軸にアラーム" in loop.abort_reason
    assert rec.rows == []  # 異常の周期はログに渡す前に止める


async def test_zero_current_after_having_moved_aborts_pattern_loop() -> None:
    brake = FakeAxis()
    brake.current_script = [300.0, 300.0]  # 以降は 0 mA
    loop, rec, *_ = _loop([_pattern(PatternKind.CREEP, brake=20.0, hold=5.0)], brake=brake)
    await _run_until_done(loop, rec)
    assert rec.emergency
    assert loop.abort_reason is not None and "ブレーキ軸の電流" in loop.abort_reason
    every = loop._safety.zero_current_abort_cycles
    assert len(rec.rows) == 2 + every - 1


async def test_can_failure_reason_is_kept() -> None:
    loop, rec, can, *_ = _loop([_pattern(PatternKind.CREEP, brake=20.0, hold=5.0)])

    async def broken() -> float:
        raise TimeoutError("CAN 車速が更新されていません")

    can.read_speed = broken  # type: ignore[method-assign]
    await _run_until_done(loop, rec)
    assert loop.abort_reason is not None and "CAN 車速を読めません" in loop.abort_reason


async def test_sample_carries_governor_and_alarm_flags() -> None:
    loop, rec, *_ = _loop([_pattern(PatternKind.CREEP, brake=20.0, hold=0.1)])
    await _run_until_done(loop, rec)
    assert rec.completed
    _, _, phase, governed, alarm_accel, alarm_brake = rec.rows[0]
    assert (phase, governed, alarm_accel, alarm_brake) == ("MEASURE", False, False, False)


async def test_sample_carries_monitors_and_cycle_time() -> None:
    """A7: 両軸のまとめ読み（実位置・電流・ステータス）と周期の処理時間が on_sample に渡る。"""
    brake = FakeAxis(current_ma=250.0)
    loop, rec, *_ = _loop([_pattern(PatternKind.CREEP, brake=20.0, hold=0.1)], brake=brake)
    await _run_until_done(loop, rec)
    assert rec.completed
    assert len(rec.monitors) == len(rec.rows)
    data = rec.rows[-1][0]
    monitor_accel, monitor_brake, cycle_ms = rec.monitors[-1]
    assert monitor_brake.position_pulse == data.brake_pos == brake.position
    assert monitor_brake.current_ma == data.brake_current == 250.0
    assert monitor_accel.position_pulse == data.accel_pos
    assert cycle_ms >= 0.0


async def test_axis_safety_polls_alarm_once_per_interval() -> None:
    calls = 0

    class Counting:
        async def is_alarm_active(self) -> bool:
            nonlocal calls
            calls += 1
            return False

    net = AxisSafetyNet(0.1)
    assert net.alarm_check_every_cycles == 10
    for cycle in range(25):
        await net.poll_alarms(cycle, Counting(), Counting())
    assert calls == 2 * 3  # cycle 0 / 10 / 20 × 両軸


def test_axis_safety_zero_current_needs_prior_current_and_position() -> None:
    net = AxisSafetyNet(0.1)
    kw = {"t": 1.0, "accel_pos": 0, "alarm_accel": False, "alarm_brake": False,
          "accel_current": 0.0}
    for _ in range(20):  # 一度も電流が流れていない軸は数えない（スタブ相当）
        assert net.check(brake_pos=500, brake_current=0.0, **kw) is None
    assert net.check(brake_pos=500, brake_current=200.0, **kw) is None
    for _ in range(net.zero_current_abort_cycles - 1):
        assert net.check(brake_pos=500, brake_current=0.0, **kw) is None
    assert net.check(brake_pos=0, brake_current=0.0, **kw) is None  # 原点なら 0 mA で正常
    for _ in range(net.zero_current_abort_cycles - 1):
        assert net.check(brake_pos=500, brake_current=0.0, **kw) is None
    reason = net.check(brake_pos=500, brake_current=0.0, **kw)
    assert reason is not None and "ブレーキ軸の電流が 1s 0mA" in reason


def test_pattern_loop_protocol_includes_alarm() -> None:
    assert hasattr(plmod.ActuatorDriverProtocol, "is_alarm_active")
    assert hasattr(hwmod.StubActuator, "is_alarm_active")


# ── 高い車速からの停車復帰の G 比例ブレーキ（段6a・6b。DRIVE_BRAKE の `_db_gprop`） ──


def _calib_loop(
    config: PatternLoopConfig | None = None,
) -> tuple[PatternLoop, LearningPattern]:
    loop, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=70.0)], config=config)
    return loop, loop._patterns[0]


def _enter_gprop_brake(loop: PatternLoop, speed_kmh: float) -> None:
    """高い車速から停車復帰（DRIVE_BRAKE）に入る。G 比例ブレーキ（`_db_gprop`）になる。"""
    loop._last_speed = speed_kmh
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    assert loop._db_gprop


def _brake_plant(loop: PatternLoop, pattern: LearningPattern, *, v0: float, t_end: float,
                 gain_g_per_pct: float = 0.0085, coast_g: float = 0.05) -> tuple[float, float]:
    """G 比例ブレーキの停車復帰（DRIVE_BRAKE・`_db_gprop`）を 0.1s 周期で流す。

    減速 = 惰行 + 感度 ×（不感帯からの踏み込み）。

    戻り値は (最大減速 [G], 終了時刻)。車速が下がるほどブレーキが効かなくなる素朴なプラント
    （効き = gain × v/25。実機の 25 km/h で 0.3・65 km/h で約 1.1 km/h/s/% に近い）。
    """
    db = loop._profile.feedforward_params.brake_deadband_pct
    v, t, peak = v0, 0.0, 0.0
    while t < t_end and loop._db_gprop:
        brake = loop._command_openings(pattern, t)[1]
        loop._current_brake_opening = brake
        g = coast_g + gain_g_per_pct * (v / 25.0) * max(0.0, brake - db)
        peak = max(peak, g)
        loop._advance(pattern, v, 0.0, t)
        v = max(0.0, v - g * G_TO_KMHS * 0.1)
        t = round(t + 0.1, 6)
    return peak, t


def test_gprop_brake_ramps_to_just_below_deadband_then_steps_toward_target_g() -> None:
    loop, pattern = _calib_loop()
    cfg = loop._config
    ff = loop._profile.feedforward_params
    _enter_gprop_brake(loop, 100.0)
    approach = min(
        loop._stop_return_brake_pct,
        max(0.0, ff.brake_deadband_pct - cfg.coast_accel_approach_margin_pct),
    )
    assert loop._command_openings(pattern, 0.0)[1] == pytest.approx(0.0)
    half = loop._command_openings(pattern, cfg.brake_ramp_time_s / 2)[1]
    assert half == pytest.approx(approach / 2)
    assert loop._command_openings(pattern, cfg.brake_ramp_time_s)[1] == pytest.approx(approach)


def test_gprop_brake_holds_near_target_g_and_never_exceeds_it_much() -> None:
    loop, pattern = _calib_loop()
    _enter_gprop_brake(loop, 120.0)
    peak, _ = _brake_plant(loop, pattern, v0=120.0, t_end=40.0)
    cfg = loop._config
    # 目標 0.2G。刻みが小さいので release_above_g（0.3G）どころか 0.25G も超えない
    assert peak < cfg.coast_accel_target_g + 0.05
    assert peak > cfg.coast_accel_target_g - 0.05


def test_gprop_brake_records_opening_and_decel_points_into_the_map() -> None:
    loop, pattern = _calib_loop()
    _enter_gprop_brake(loop, 120.0)
    for v in range(5, 130, 10):  # 手順2 ではこの前に走るコーストダウンが惰行の減速を測っている
        loop.glimit.add_coast(float(v), 0.05 * G_TO_KMHS)
    _brake_plant(loop, pattern, v0=120.0, t_end=40.0)
    assert loop.glimit.has_data("brake")
    ff = loop._profile.feedforward_params
    cap = loop.glimit.cap_pct(
        "brake", 60.0, 0.3 * G_TO_KMHS, ff.brake_deadband_pct
    )
    assert cap is not None and cap > ff.brake_deadband_pct
    # プラントは開度に線形（感度 0.0085G/% × v/25）なので割線の外挿が当たる。
    # 60〜70 km/h の帯（平均 約 65 km/h）で惰行 0.05G → 0.3G に届く開度 = 不感帯 + 0.25 / 0.0221
    assert cap == pytest.approx(ff.brake_deadband_pct + 0.25 / (0.0085 * 65.0 / 25.0), rel=0.3)


def test_coast_phase_feeds_coast_decel_after_the_measure_delay() -> None:
    loop, pattern = _calib_loop()
    loop._enter_phase(_Phase.COAST, 0.0)
    delay = loop._config.coast_measure_delay_s
    loop._advance(pattern, 80.0, -1.7, delay / 2)  # 離した直後は数えない
    assert loop.glimit._coast == {}
    loop._advance(pattern, 80.0, -1.7, delay + 0.1)
    assert loop.glimit._coast == {8: [1.7]}


# ── コーストダウンの終了はクリープ平衡車速から（段7c） ──────────────────


def test_coast_finishes_at_creep_equilibrium_speed_not_a_fixed_5kmh() -> None:
    """クリープ平衡が固定値 5.0 より高い車では、平衡に達した時点で終わる
    （従来の固定 5.0 のままだと届かず coast_timeout_s まで待っていた）。"""
    loop, pattern = _calib_loop()
    loop._profile.feedforward_params.creep_speed_kmh = 8.0
    loop._enter_phase(_Phase.COAST, 0.0)
    loop._advance(pattern, 8.0, -1.0, 5.0)  # 固定 5.0 の判定なら終わらないはずの車速
    assert loop._phase is _Phase.DRIVE_BRAKE  # 惰行が終わって停車復帰へ


def test_coast_does_not_end_at_5kmh_when_creep_speed_is_lower() -> None:
    """クリープ平衡が 5.0 より低い車では、5.0 で終わらず平衡近くまで惰行を続ける。"""
    loop, pattern = _calib_loop()
    loop._profile.feedforward_params.creep_speed_kmh = 3.0
    loop._enter_phase(_Phase.COAST, 0.0)
    loop._advance(pattern, 5.0, -1.0, 5.0)  # 5.0 はクリープ平衡(3.0)よりまだ上
    assert loop._phase is _Phase.COAST


def test_coast_finishes_when_settled_near_creep_speed_even_if_not_exactly_at_it() -> None:
    """惰行は平衡へ漸近するだけでちょうど届く保証が無い。クリープの影響が残る車速
    （平衡 + grid_settle_tol_kmh）以下で車速の傾きが落ち着いたら終える。"""
    loop, pattern = _calib_loop()
    loop._profile.feedforward_params.creep_speed_kmh = 5.0
    loop._enter_phase(_Phase.COAST, 0.0)
    # 平衡(5.0)の少し上で釣り合っている（傾き ≈ 0）。毎周期 _advance を呼ぶ。
    # 段2l-11: 判定専用の履歴 _coast_hist に creep_launch_settle_s（2 秒）ぶん溜まってから終わる
    # （_speed_hist は 0.4 秒で刈られるので使えなかった）
    window = loop._config.creep_launch_settle_s
    for i in range(30):
        now = i * 0.1
        loop._advance(pattern, 5.5, 0.0, now)
        if now < window - 1e-9:
            assert loop._phase is _Phase.COAST  # 2 秒ぶん溜まる前は終わらない
        if loop._phase is _Phase.DRIVE_BRAKE:
            break
    assert loop._phase is _Phase.DRIVE_BRAKE
    assert now >= window - 1e-9


def test_coast_settle_history_is_separate_from_speed_hist_and_cleared_on_entry() -> None:
    """段2l-11: 終了判定は判定専用の履歴で見る。_speed_hist は 0.4 秒で刈られても 2 秒ぶん溜まる。
    COAST に入り直したら空にする。"""
    loop, pattern = _calib_loop()
    loop._profile.feedforward_params.creep_speed_kmh = 5.0
    loop._enter_phase(_Phase.COAST, 0.0)
    for i in range(10):
        loop._speed_hist.append((i * 0.1, 5.5))
        loop._advance(pattern, 5.5, 0.0, i * 0.1)
    assert len(loop._coast_hist) > 0 and loop._coast_hist[0][0] >= 0.0
    loop._enter_phase(_Phase.COAST, 5.0)
    assert len(loop._coast_hist) == 0


def test_coast_does_not_finish_when_speed_is_still_dropping_near_creep() -> None:
    """平衡の近くでも傾きが 0.1 km/h/s 以上ある間は終わらない。"""
    loop, pattern = _calib_loop()
    loop._profile.feedforward_params.creep_speed_kmh = 3.0
    loop._enter_phase(_Phase.COAST, 0.0)
    for i in range(30):  # 天井(4.0)以下から −0.3 km/h/s で下がる（まだ平衡 3.0 に届かない）
        loop._advance(pattern, 4.0 - 0.3 * i * 0.1, 0.0, i * 0.1)
    assert loop._phase is _Phase.COAST


def test_coast_finishes_with_a_non_dividing_cycle_after_the_window_fills() -> None:
    """周期が窓（2 秒）を割り切らなくても（0.07 秒）、窓ぶん溜まったら終わる
    （窓の端をまたぐ点を残さないと先頭がいつも窓より新しく、成立しない）。"""
    loop, pattern = _calib_loop()
    loop._profile.feedforward_params.creep_speed_kmh = 5.0
    loop._enter_phase(_Phase.COAST, 0.0)
    window = loop._config.creep_launch_settle_s
    finished_at = None
    for i in range(100):
        now = i * 0.07
        loop._advance(pattern, 5.5, 0.0, now)
        if loop._phase is _Phase.DRIVE_BRAKE:
            finished_at = now
            break
    assert finished_at is not None and window <= finished_at <= window + 0.07 + 1e-9


def test_coast_does_not_finish_above_the_creep_ceiling_even_if_settled() -> None:
    """クリープの影響が残る車速の上端より高ければ、傾きが落ち着いていても終わらない。"""
    loop, pattern = _calib_loop()
    loop._profile.feedforward_params.creep_speed_kmh = 5.0
    loop._enter_phase(_Phase.COAST, 0.0)
    for i in range(30):
        # 天井よりずっと上。傾き 0 でも終わらない
        loop._advance(pattern, 20.0, 0.0, i * 0.1)
    assert loop._phase is _Phase.COAST


def test_coast_accel_records_points_via_the_lagged_actual_opening_even_while_moving() -> None:
    """段7d: 開度が動き続けていても、遅れ gov_lead_s ぶんずらした窓の実開度平均で記録する。

    実機（20260928_050944）の 70〜110 km/h は、G が目標に届かないまま開度が上がり続け、
    段7c の「振れが小さい間だけ」の条件でも 1 点も取れなかった（60〜70 km/h の値を借用して
    平らになっていた）。振れの条件を無くし、常に遅れずらしの平均で記録するので、開度が
    速く動いていても点が取れる。
    """
    loop, pattern, _ = _coast_loop()
    cfg = loop._config
    t0 = cfg.accel_ramp_time_s
    # G=0（目標に遠い）: 開度は rate_gain いっぱいで踏み増し続ける。段7c 以前はこの
    # 「動き続ける」ケースで 1 点も取れなかった
    end = t0 + cfg.coast_accel_slope_window_s + cfg.gov_lead_s + 2.0
    _run_ramp(loop, pattern, 0.0, end, 0.0, 10.0)
    assert loop.glimit.has_data("accel")


def test_coast_accel_records_the_real_actual_opening_not_the_command() -> None:
    """段7d: 実開度（monitor から）が指令とずれていれば、実開度の方で記録する。

    低速の踏み込みは指令 [%/s] より実アクチュエータの動きが遅く、指令開度のまま記録すると
    上限開度が高め（危険側）に出ていた（実機 050944 の 10〜30 km/h）。
    """
    loop, pattern, _ = _coast_loop()
    cfg = loop._config
    t0 = cfg.accel_ramp_time_s
    real_opening = 20.0  # 指令（_ca_opening は接近位置止まり）とは全く違う実開度
    end = t0 + cfg.coast_accel_slope_window_s + cfg.gov_lead_s + 1.5
    i = 0
    while i <= round(end * 10):
        t = round(i * 0.1, 6)
        loop._accel_actual_pct = real_opening
        loop._command_openings(pattern, t)
        loop._advance(pattern, 10.0, 0.0, t)  # 車速は一定（傾き 0）
        i += 1
    assert loop.glimit.has_data("accel")
    point = loop.glimit.strongest_point("accel", 10.0)
    assert point is not None
    assert point[0] == pytest.approx(real_opening)  # 指令（接近位置）ではなく実開度


def test_ca_opening_hist_does_not_carry_over_across_a_phase_change() -> None:
    """段7d: 前のパターンの開度の履歴が次のパターンに残って偽の点になっていたバグの修正。

    実機（050944）ではコーストダウン加速の指令 70% が残ったまま次の G 校正の踏み始めに
    積まれ、(70%, 低加速度) という偽の点（0〜10 km/h で 227.3%）ができていた。
    """
    loop, _pattern, _ = _coast_loop()
    loop._ca_opening_hist.append((0.0, 70.0))
    loop._enter_phase(_Phase.DRIVE_ACCEL, 1.0)
    assert len(loop._ca_opening_hist) == 0


# ── 停車復帰の G 比例（段6b） ──────────────────────────────────────────


def test_stop_return_from_speed_is_g_proportional_not_a_jump_to_the_hold_opening() -> None:
    loop, pattern = _calib_loop()
    loop._last_speed = 100.0
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    assert loop._db_gprop
    ff = loop._profile.feedforward_params
    # 1.5 s 後（従来なら停車保持開度に届く時刻）でも、不感帯の手前までしか踏まない
    brake = loop._command_openings(pattern, loop._config.brake_ramp_time_s)[1]
    assert brake < ff.brake_deadband_pct
    assert brake < loop._stop_return_brake_pct


def test_stop_return_from_creep_speed_keeps_the_old_ramp_for_non_coast_patterns() -> None:
    """G ランプなどコーストダウン以外は従来どおり brake_ramp_time_s のランプ・停車で即前進。"""
    p = plmod.GRampPattern(PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0,
                           hold_duration_s=0.0, target_g=0.1, top_kmh=60.0)
    loop, *_ = _loop([p, _pattern(PatternKind.COAST_DOWN, accel=30.0)])
    pattern = loop._patterns[0]
    loop._last_speed = 4.0
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    assert not loop._db_gprop and not loop._db_creep_press
    brake = loop._command_openings(pattern, loop._config.brake_ramp_time_s)[1]
    assert brake == pytest.approx(loop._stop_return_brake_pct)
    # 停車したら、停車保持開度に届く前でも次へ進む
    loop._enter_phase(_Phase.DRIVE_BRAKE, 10.0)
    loop._command_openings(pattern, 10.0)
    loop._advance(pattern, 0.0, 0.0, 10.0)
    assert loop._pattern_idx == 1


def test_stop_return_gprop_starts_from_the_calibrated_opening_and_hands_over_at_low_speed() -> None:
    loop, pattern = _calib_loop()
    for v in (95.0, 105.0):  # G 校正で「0.2G は 12%（傾き 0.5 km/h/s/%）」と分かっている
        loop.glimit.add("brake", v, 10.0, 0.2 * G_TO_KMHS - 1.0)
        loop.glimit.add("brake", v, 12.0, 0.2 * G_TO_KMHS)
        loop.glimit.add_coast(v, 1.6)
    loop._last_speed = 100.0
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    t = loop._config.brake_ramp_time_s
    loop._command_openings(pattern, 0.0)
    assert loop._command_openings(pattern, t)[1] == pytest.approx(12.0, abs=0.3)
    # 低速まで下りたら G 比例をやめ、今の開度から停車保持開度へランプする
    loop._current_brake_opening = 12.0
    loop._advance(pattern, loop._config.coast_down_stop_speed_kmh - 1.0, 0.0, t + 5.0)
    assert not loop._db_gprop
    assert loop._brake_ramp_from == pytest.approx(12.0)
    assert loop._command_openings(pattern, t + 5.0)[1] == pytest.approx(12.0)
    assert loop._db_creep_press  # コーストダウンなので徐々に踏む（段2l-6b）
    rate = loop._stop_return_press_rate_pctps()
    one_s = loop._command_openings(pattern, t + 6.0)[1]
    assert one_s == pytest.approx(12.0 + rate)
    end = loop._command_openings(pattern, t + 5.0 + 100.0)[1]
    assert end == pytest.approx(loop._stop_return_brake_pct)


def test_step_ramp_and_predictive_lead_defaults_are_documented_values() -> None:
    c = PatternLoopConfig()
    assert (c.gov_lead_s, c.gov_soft_frac) == (0.6, 0.85)


# ── 使わない側のペダルを待機位置（min）に置く（2026-10-07 段2l-4、段2l-6 で min そのものに）──


async def _one_cycle_openings(
    phase: _Phase, *, standby: bool, speed_kmh: float = 50.0,
    pattern: LearningPattern | None = None, launch_done: bool = False,
) -> tuple[float, float]:
    """phase に入れて 1 周期だけ走らせ、記録された (アクセル, ブレーキ) 指令開度を返す。

    launch_done=True は G ランプの発進（ブレーキを離す場面）を終えた状態（u ＝ 0）にする。"""
    pat = pattern or _pattern(PatternKind.COAST_DOWN, accel=30.0)
    config = PatternLoopConfig(pedal_standby=standby, accel_ramp_time_s=0.0,
                               brake_ramp_time_s=0.0)
    loop, rec, *_ = _loop([pat], speed_kmh=speed_kmh, config=config)
    loop._running = True
    loop._enter_phase(phase, asyncio.get_running_loop().time())
    if launch_done:  # 2026-10-09 段2l-8: ブレーキを離し終えた状態（u ＝ 0・アクセル側）
        loop._rp_u = 0.0
        loop._rp_accel_reached = True
    await loop._execute_one_cycle()
    data = rec.rows[-1][0]
    return data.accel_opening, data.brake_opening


def _standby() -> tuple[float, float]:
    """待機位置 ＝ ペダルに触れた位置 min（プロファイルの名前は不感帯のまま）。"""
    ff = build_vehicle_profile(cfgmod.load_config(CFG_PATH)).feedforward_params
    return ff.accel_deadband_pct, ff.brake_deadband_pct


async def test_coast_command_is_standby_on_both_pedals() -> None:
    a, b = await _one_cycle_openings(_Phase.COAST, standby=True)
    sa, sb = _standby()
    assert (a, b) == (pytest.approx(sa), pytest.approx(sb))
    assert sa > 0.0 and sb > 0.0


async def test_brake_command_during_accel_is_standby() -> None:
    a, b = await _one_cycle_openings(_Phase.DRIVE_ACCEL, standby=True)
    assert b == pytest.approx(_standby()[1])
    assert a >= _standby()[0]


async def test_ramp_up_brake_command_is_standby_after_the_launch_stage() -> None:
    p = plmod.GRampPattern(PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0,
                           hold_duration_s=0.0, target_g=0.1, top_kmh=60.0)
    a, b = await _one_cycle_openings(
        _Phase.RAMP_UP, standby=True, speed_kmh=20.0, pattern=p, launch_done=True
    )
    assert b == pytest.approx(_standby()[1])
    assert a >= _standby()[0]


async def test_ramp_up_launch_stage_holds_the_brake_and_keeps_accel_at_standby() -> None:
    """発進の段階: ブレーキは止まった開度（stop_confirm_pct）から始め、アクセルは min。"""
    p = plmod.GRampPattern(PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0,
                           hold_duration_s=0.0, target_g=0.1, top_kmh=60.0)
    config = PatternLoopConfig(pedal_standby=True, stop_confirm_pct=12.0, brake_ramp_time_s=0.0)
    loop, rec, *_ = _loop([p], speed_kmh=0.0, config=config)
    loop._running = True
    loop._enter_phase(_Phase.RAMP_UP, asyncio.get_running_loop().time())
    await loop._execute_one_cycle()
    data = rec.rows[-1][0]
    assert data.brake_opening == pytest.approx(12.0)
    assert data.accel_opening == pytest.approx(_standby()[0])


async def test_accel_command_during_stop_hold_is_standby() -> None:
    pat = _pattern(PatternKind.COAST_DOWN, accel=30.0, brake=15.0)
    a, b = await _one_cycle_openings(_Phase.BRAKE_HOLD, standby=True, speed_kmh=0.0, pattern=pat)
    assert a == pytest.approx(_standby()[0])
    assert b >= _standby()[1]


async def test_creep_launch_command_is_standby_on_both_pedals() -> None:
    a, b = await _one_cycle_openings(
        _Phase.CREEP_LAUNCH, standby=True, speed_kmh=0.0, pattern=_creep_launch()
    )
    assert (a, b) == (pytest.approx(_standby()[0]), pytest.approx(_standby()[1]))


async def test_no_standby_keeps_the_unused_pedals_at_zero() -> None:
    assert await _one_cycle_openings(_Phase.COAST, standby=False) == (0.0, 0.0)
    a, b = await _one_cycle_openings(_Phase.DRIVE_ACCEL, standby=False)
    assert b == 0.0


async def test_standby_is_output_only_internal_opening_stays_unfloored() -> None:
    config = PatternLoopConfig(pedal_standby=True)
    loop, rec, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=30.0)], speed_kmh=50.0,
                          config=config)
    loop._running = True
    loop._enter_phase(_Phase.COAST, asyncio.get_running_loop().time())
    await loop._execute_one_cycle()
    assert loop._current_accel_opening == 0.0 and loop._current_brake_opening == 0.0
    assert rec.rows[-1][0].accel_opening > 0.0


def test_standby_margin_pct_is_gone_from_the_loop_config() -> None:
    assert not hasattr(PatternLoopConfig(), "standby_margin_pct")
    assert PatternLoopConfig().pedal_standby is False
    assert PatternLoopConfig().stop_confirm_pct is None


# ── コーストダウンの上端（2026-10-07 段2l-4）────────────────────────────────


def test_top_speed_cap_lowers_the_coast_accel_speed_cap() -> None:
    loop, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=30.0)],
                     config=PatternLoopConfig(top_speed_cap_kmh=100.0))
    assert loop._accel_speed_cap == pytest.approx(100.0)


def test_top_speed_cap_above_the_vehicle_limit_is_ignored_and_none_is_unchanged() -> None:
    plain, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=30.0)])
    high, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=30.0)],
                     config=PatternLoopConfig(top_speed_cap_kmh=9999.0))
    assert plain._accel_speed_cap == pytest.approx(high._accel_speed_cap)


def test_coast_accel_ends_near_the_top_speed_cap() -> None:
    loop, *_ = _loop([_pattern(PatternKind.COAST_DOWN, accel=30.0)],
                     config=PatternLoopConfig(top_speed_cap_kmh=100.0))
    pattern = loop._patterns[0]
    loop._enter_phase(_Phase.DRIVE_ACCEL, 0.0)
    _feed(loop, pattern, 0.0, 99.0)
    assert loop._phase is _Phase.DRIVE_ACCEL
    _feed(loop, pattern, 0.1, 100.1)
    assert loop._phase is _Phase.COAST


# ── コーストダウンの停車復帰: クリープ域は停車保持開度まで徐々に踏む（2026-10-08 段2l-6b） ──


def _creep_press_loop(creep_kmh: float = 5.0) -> tuple[PatternLoop, LearningPattern]:
    loop, pattern = _calib_loop()
    loop._profile.feedforward_params.creep_speed_kmh = creep_kmh
    return loop, pattern


def test_creep_domain_stop_return_presses_monotonically_at_the_calib_step_rate() -> None:
    loop, pattern = _creep_press_loop()
    loop._last_speed = 3.0
    loop._current_brake_opening = 4.0
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    assert loop._db_creep_press and not loop._db_gprop
    cfg = loop._config
    rate = cfg.calib_step_mm * 100.0 * 100.0 / plmod.ACTUATOR_PULSE_MAX / cfg.calib_dwell_s
    assert loop._stop_return_press_rate_pctps() == pytest.approx(rate)
    prev, t = None, 0.0
    while t < 60.0:
        b = loop._command_openings(pattern, t)[1]
        if prev is not None:
            assert b >= prev - 1e-12  # 途中で下げない
            assert b - prev <= rate * 0.1 + 1e-9  # 1 周期あたりの増分は速さ以下
        prev, t = b, round(t + 0.1, 6)
    assert prev == pytest.approx(loop._stop_return_brake_pct)
    assert loop._command_openings(pattern, 0.0)[1] == pytest.approx(4.0)


def test_creep_press_does_not_advance_until_the_hold_opening_is_reached() -> None:
    loop, pattern = _creep_press_loop()
    loop._last_speed = 0.0
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    assert loop._db_creep_press
    loop._command_openings(pattern, 5.0)
    loop._advance(pattern, 0.0, 0.0, 5.0)  # 停車しているが、まだ踏み途中
    assert (loop._pattern_idx, loop._phase) == (0, _Phase.DRIVE_BRAKE)
    t_reach = loop._stop_return_brake_pct / loop._stop_return_press_rate_pctps()
    loop._command_openings(pattern, t_reach + 0.1)
    loop._advance(pattern, 0.0, 0.0, t_reach + 0.1)
    assert loop._pattern_idx == 1 and loop._pending_abort is None


def test_creep_press_timeout_counts_from_reaching_the_hold_opening() -> None:
    loop, pattern = _creep_press_loop()
    loop._config = PatternLoopConfig(brake_stop_timeout_s=10.0)
    cfg = loop._config
    loop._last_speed = 2.0
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    t_reach = loop._stop_return_brake_pct / loop._stop_return_press_rate_pctps()
    assert t_reach > cfg.brake_stop_timeout_s  # 踏み上げだけで時間切れ秒を超える
    loop._command_openings(pattern, t_reach - 1.0)
    loop._advance(pattern, 2.0, 0.0, t_reach - 1.0)  # 動いたまま。届く前は中断しない
    assert loop._pending_abort is None
    loop._command_openings(pattern, t_reach + 0.1)
    loop._advance(pattern, 2.0, 0.0, t_reach + 0.1)  # 届いた時刻
    assert loop._pending_abort is None
    loop._command_openings(pattern, t_reach + 0.2 + cfg.brake_stop_timeout_s)
    loop._advance(pattern, 2.0, 0.0, t_reach + 0.2 + cfg.brake_stop_timeout_s)
    assert loop._pending_abort is not None and "停車しません" in loop._pending_abort


def test_stop_return_boundary_follows_the_creep_speed_of_the_vehicle() -> None:
    """クリープ車速 8.0 km/h の車では境目が 8.0 + 1.0 = 9.0 km/h。5 km/h 固定ではない。"""
    loop, pattern = _creep_press_loop(creep_kmh=8.0)
    assert loop._stop_return_low_speed_kmh() == pytest.approx(9.0)
    loop._enter_phase(_Phase.COAST, 0.0)
    loop._advance(pattern, 8.5, -1.0, 5.0)  # 8.5 は天井 9.0 以下
    loop._last_speed = 8.5
    loop._enter_phase(_Phase.DRIVE_BRAKE, 5.0)
    assert loop._db_creep_press and not loop._db_gprop  # G 比例ではなく徐々に踏む
    loop._last_speed = 9.5
    loop._enter_phase(_Phase.DRIVE_BRAKE, 5.0)
    assert loop._db_gprop and not loop._db_creep_press


def test_overspeed_recovery_in_coast_down_keeps_the_old_behavior() -> None:
    loop, pattern = _creep_press_loop()
    loop._last_speed = 3.0
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0, overspeed_recovery=True)
    assert not loop._db_creep_press and not loop._db_gprop
    assert loop._stop_return_low_speed_kmh() == loop._config.coast_down_stop_speed_kmh
    brake = loop._command_openings(pattern, loop._config.brake_ramp_time_s)[1]
    assert brake >= loop._config.overspeed_recovery_brake_pct - 1e-9


def test_creep_press_falls_back_to_the_ramp_when_calib_dwell_is_not_positive() -> None:
    loop, pattern = _calib_loop(PatternLoopConfig(calib_dwell_s=0.0))
    loop._last_speed = 3.0
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    assert not loop._db_creep_press
    brake = loop._command_openings(pattern, loop._config.brake_ramp_time_s)[1]
    assert brake == pytest.approx(loop._stop_return_brake_pct)


# ── 2026-10-10 段2l-10: 周期・送り方・書き込み後の待ち ──────────────────────


async def test_ramp_phase_sends_with_smooth_over_interval_times_duty() -> None:
    """G ランプの区間は手順3 と同じ move_to_position(smooth_over_s=周期×0.8) で送る。"""
    p = plmod.GRampPattern(PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0,
                           hold_duration_s=0.0, target_g=0.1, top_kmh=60.0)
    config = PatternLoopConfig(pedal_standby=True, stop_confirm_pct=12.0, brake_ramp_time_s=0.0)
    loop, _rec, _can, accel, brake = _loop([p], speed_kmh=0.0, config=config, interval_s=0.05)
    loop._running = True
    loop._enter_phase(_Phase.RAMP_UP, asyncio.get_running_loop().time())
    await loop._execute_one_cycle()
    calls = accel.calls + brake.calls
    assert calls, "位置が変わる周期のはず"
    assert all(kind == "smooth" for kind, _pos, _arg in calls)
    assert all(arg == pytest.approx(0.05 * plmod.AXIS_SMOOTH_DUTY) for _k, _p, arg in calls)


async def test_non_ramp_phase_still_sends_with_move_to_position_timed() -> None:
    pat = _pattern(PatternKind.COAST_DOWN, accel=30.0)
    config = PatternLoopConfig(pedal_standby=True, accel_ramp_time_s=0.0, brake_ramp_time_s=0.0)
    loop, _rec, _can, accel, brake = _loop([pat], speed_kmh=50.0, config=config)
    loop._running = True
    loop._enter_phase(_Phase.DRIVE_ACCEL, asyncio.get_running_loop().time())
    await loop._execute_one_cycle()
    calls = accel.calls + brake.calls
    assert calls
    assert all(kind == "timed" for kind, _pos, _arg in calls)


async def test_wait_between_write_and_read_only_when_position_changed(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    waits: list[float] = []
    real_sleep = asyncio.sleep

    async def spy_sleep(delay: float, *a: object, **k: object) -> None:
        waits.append(delay)
        await real_sleep(0)

    pat = _pattern(PatternKind.COAST_DOWN, accel=30.0)
    config = PatternLoopConfig(pedal_standby=True, accel_ramp_time_s=0.0, brake_ramp_time_s=0.0)
    loop, *_ = _loop([pat], speed_kmh=50.0, config=config)
    monkeypatch.setattr(plmod.asyncio, "sleep", spy_sleep)
    loop._running = True
    loop._enter_phase(_Phase.DRIVE_ACCEL, asyncio.get_running_loop().time())
    await loop._execute_one_cycle()
    assert plmod.AXIS_PRE_READ_DELAY_S in waits  # 位置が変わった周期は 2 ms 待つ
    assert plmod.AXIS_PRE_READ_DELAY_S == pytest.approx(0.002)
    # 位置が変わらない周期（同じ指令）は読むだけで待たない
    waits.clear()
    await loop._drive_or_sense(loop._accel_pos_cmd, loop._brake_pos_cmd)
    assert waits == []


def test_skip_consecutive_required_keeps_its_length_in_seconds() -> None:
    """設定は 0.1 秒周期での周期数。50 ms 周期では 2 倍の回数にして秒を保つ。"""
    pat = _pattern(PatternKind.COAST_DOWN, accel=30.0)
    for interval, expect in ((0.1, 2), (0.05, 4)):
        loop, *_ = _loop([pat], interval_s=interval)
        assert loop._skip_required == expect
