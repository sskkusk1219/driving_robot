"""研究用パターン走行ループ（pattern_loop.PatternLoop）の A6 変更と安全網のユニットテスト。

2026-09-13 A6（KAIZEN 表5-5 順3）で入れたもの:
    - ガバナーの解除（上限 × gov_release_frac を下回ったら頭打ちを戻し、指令に届いたら解除）
    - 全運転パターンの後の停車復帰（DRIVE_BRAKE。停車してから次のパターンへ）
    - DRIVE_BRAKE の打ち切り = 停車しなければ中断する安全上限
    - ブレーキのランプをフェーズに入ったときの開度から始める
    - 走行中の安全網（axis_safety.AxisSafetyNet: アラーム確認・電流ゼロの継続）

2026-09-13 A5: 目標車速つきパターン（SpeedTargetPattern）は accel_target_kmh で加速を終える。
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest

from src.domain.control.conversions import G_TO_KMHS
from src.models.drive_log import DriveLogData
from src.models.learning_drive import LearningPattern, PatternKind
from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import pattern_loop as plmod
from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.pattern_loop import PatternLoop, PatternLoopConfig, _Phase
from tests.research.vehicle import build_vehicle_profile

CFG_PATH = Path("tests/research/config_testVehicle.yaml")


class FakeAxis:
    """位置指令を覚え、決めた電流・アラームを返すアクチュエータ。"""

    def __init__(self, current_ma: float = 0.0) -> None:
        self.position = 0
        self.current_ma = current_ma
        self.alarm = False
        self.current_script: list[float] | None = None

    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None:
        self.position = target_pos

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
    loop, *_ = _loop([_pattern(PatternKind.ACCEL_SWEEP, accel=40.0, brake=30.0)])
    limit = loop._g_limit_kmhs
    assert limit == pytest.approx(0.4 * G_TO_KMHS * 0.98)
    cfg = loop._config

    # 上限以上: 最初は現在開度で頭打ち → 以降 2%/周期で下げる（本番と同じ）
    cap = loop._next_gov_cap(None, limit + 1.0, current=30.0, request=30.0)
    assert cap == pytest.approx(30.0)
    cap = loop._next_gov_cap(cap, limit + 1.0, current=30.0, request=30.0)
    assert cap == pytest.approx(30.0 - cfg.gov_reduce_step_pct)
    # 上限 × 0.7〜1.0: 保持
    assert loop._next_gov_cap(cap, limit * 0.8, current=28.0, request=30.0) == pytest.approx(cap)
    # 上限 × 0.7 未満: 0.5%/周期で戻す
    raised = loop._next_gov_cap(cap, limit * 0.5, current=28.0, request=30.0)
    assert raised == pytest.approx(cap + cfg.gov_raise_step_pct)
    # 指令に届いたら解除
    assert loop._next_gov_cap(29.8, limit * 0.5, current=29.8, request=30.0) is None
    # 頭打ちが無く上限未満なら何もしない
    assert loop._next_gov_cap(None, limit * 0.5, current=30.0, request=30.0) is None
    # 下限は 0%
    assert loop._next_gov_cap(1.0, limit + 1.0, current=1.0, request=30.0) == 0.0


def test_update_governor_uses_brake_deceleration_in_drive_brake() -> None:
    loop, *_ = _loop([_pattern(PatternKind.ACCEL_SWEEP, accel=40.0, brake=30.0)])
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    loop._current_brake_opening = 30.0
    loop._brake_request = 30.0
    limit = loop._g_limit_kmhs
    loop._update_governor(-(limit + 1.0))  # 減速が上限超え
    assert loop._brake_gov_cap == pytest.approx(30.0)
    loop._update_governor(-(limit + 1.0))
    assert loop._brake_gov_cap == pytest.approx(28.0)
    assert loop._governor_limiting()
    loop._update_governor(-1.0)  # 減速が十分小さい → 戻す
    assert loop._brake_gov_cap == pytest.approx(28.5)
    assert loop._accel_gov_cap is None  # ブレーキ中はアクセル側を触らない


# ── 停車復帰・ランプ・打ち切り ─────────────────────────────────────────


def test_brake_ramp_starts_from_opening_at_phase_entry() -> None:
    loop, *_ = _loop([_pattern(PatternKind.BRAKE_HOLD, accel=70.0, brake=15.0)])
    loop._current_brake_opening = 15.0  # BRAKE_HOLD で保持していた開度
    loop._enter_phase(_Phase.DRIVE_BRAKE, 10.0)
    pattern = loop._patterns[0]
    accel, brake = loop._command_openings(pattern, 10.0)
    assert (accel, brake) == (0.0, pytest.approx(15.0))  # 0% に抜けない
    ramp = loop._config.brake_ramp_time_s
    _, brake = loop._command_openings(pattern, 10.0 + ramp)
    # BRAKE_HOLD の停車復帰の目標は停車保持開度（パターンの保持開度ではない）
    assert brake == pytest.approx(loop._stop_return_brake_pct)
    assert loop._stop_return_brake_pct > 15.0


def test_accel_sweep_stop_return_keeps_reset_brake() -> None:
    loop, *_ = _loop([_pattern(PatternKind.ACCEL_SWEEP, accel=40.0, brake=25.0)])
    loop._enter_phase(_Phase.DRIVE_BRAKE, 0.0)
    _, brake = loop._command_openings(loop._patterns[0], loop._config.brake_ramp_time_s)
    assert brake == pytest.approx(25.0)


@pytest.mark.parametrize(
    ("phase", "kind", "kw"),
    [
        (_Phase.BRAKE_HOLD, PatternKind.BRAKE_HOLD, {}),
        (_Phase.COAST, PatternKind.COAST_DOWN, {}),
        (_Phase.CRUISE_TRIM, PatternKind.CRUISE_TRIM, {"trim_opening": 12.0}),
    ],
)
def test_driving_patterns_end_with_stop_return(
    phase: _Phase, kind: PatternKind, kw: dict[str, float]
) -> None:
    config = PatternLoopConfig(brake_hold_timeout_s=1.0, coast_timeout_s=1.0)
    patterns = [_pattern(kind, accel=60.0, brake=15.0, hold=1.0, **kw),
                _pattern(PatternKind.ACCEL_SWEEP, accel=30.0, brake=30.0)]
    loop, *_ = _loop(patterns, config=config)
    loop._enter_phase(phase, 0.0)
    # 保持・惰行が終わっても車速が残っていれば、次へ進まず停車復帰に入る
    loop._advance(patterns[0], 40.0, 0.0, 2.0)
    assert (loop._pattern_idx, loop._phase) == (0, _Phase.DRIVE_BRAKE)
    # 停車したら次のパターンへ
    loop._advance(patterns[0], 0.0, 0.0, 3.0)
    assert (loop._pattern_idx, loop._phase) == (1, _Phase.DRIVE_ACCEL)


def test_brake_hold_that_already_stopped_goes_straight_to_next() -> None:
    patterns = [_pattern(PatternKind.BRAKE_HOLD, accel=60.0, brake=20.0),
                _pattern(PatternKind.COAST_DOWN, accel=60.0)]
    loop, *_ = _loop(patterns)
    loop._enter_phase(_Phase.BRAKE_HOLD, 0.0)
    loop._advance(patterns[0], 0.0, 0.0, 1.0)
    assert (loop._pattern_idx, loop._phase) == (1, _Phase.DRIVE_ACCEL)


@pytest.mark.parametrize(
    ("speed", "phase"), [(59.9, _Phase.DRIVE_ACCEL), (60.0, _Phase.BRAKE_HOLD)]
)
def test_speed_target_brake_hold_exits_accel_at_target(speed: float, phase: _Phase) -> None:
    """A5: 目標車速つきの BRAKE_HOLD は accel_target_kmh で保持に移る（cap の先読み無し）。"""
    pattern = plmod.SpeedTargetPattern(
        PatternKind.BRAKE_HOLD, accel_opening=70.0, brake_opening=14.0, hold_duration_s=3.0,
        accel_target_kmh=60.0,
    )
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.DRIVE_ACCEL, 0.0)
    loop._advance(pattern, speed, 11.0, 1.0)  # 加速 11 km/h/s でも先読みで手前に切り替えない
    assert loop._phase is phase


def test_plain_brake_hold_still_exits_accel_near_cap() -> None:
    pattern = _pattern(PatternKind.BRAKE_HOLD, accel=70.0, brake=14.0)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.DRIVE_ACCEL, 0.0)
    loop._advance(pattern, 60.0, 11.0, 1.0)
    assert loop._phase is _Phase.DRIVE_ACCEL
    exit_speed = loop._accel_speed_cap - 11.0 * loop._config.overspeed_lead_s
    loop._advance(pattern, exit_speed, 11.0, 2.0)
    assert loop._phase is _Phase.BRAKE_HOLD


def _stair(target_kmh: float = 120.0) -> plmod.TrimStairPattern:
    return plmod.TrimStairPattern(
        PatternKind.CRUISE_TRIM, accel_opening=70.0, brake_opening=0.0, hold_duration_s=24.0,
        trim_opening=18.0, accel_target_kmh=target_kmh, trim_steps_pct=(18.0, 15.0, 12.0),
        step_hold_s=8.0,
    )


def _trim_command(loop: PatternLoop, pattern: LearningPattern, now: float) -> float:
    accel, _ = loop._command_openings(pattern, now)
    return accel


def test_trim_stair_exits_accel_at_target_and_steps_down_every_step_hold() -> None:
    """A3: 目標車速で CRUISE_TRIM に入り、8s ごとに 18 → 15 → 12%、最後の段の後に停車復帰。"""
    pattern = _stair()
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.DRIVE_ACCEL, 0.0)
    loop._advance(pattern, 119.9, 11.0, 1.0)
    assert loop._phase is _Phase.DRIVE_ACCEL
    loop._advance(pattern, 120.0, 11.0, 2.0)  # 先読みなしで目標車速に切り替える
    assert loop._phase is _Phase.CRUISE_TRIM
    assert _trim_command(loop, pattern, 2.1) == 18.0

    speeds_and_commands = []
    for now, speed in ((9.9, 125.0), (10.0, 125.0), (17.9, 120.0), (18.0, 118.0)):
        loop._advance(pattern, speed, 0.0, now)
        speeds_and_commands.append((loop._phase, _trim_command(loop, pattern, now)))
    assert speeds_and_commands == [
        (_Phase.CRUISE_TRIM, 18.0), (_Phase.CRUISE_TRIM, 15.0),
        (_Phase.CRUISE_TRIM, 15.0), (_Phase.CRUISE_TRIM, 12.0),
    ]
    loop._advance(pattern, 100.0, 0.0, 25.9)
    assert loop._phase is _Phase.CRUISE_TRIM
    loop._advance(pattern, 100.0, 0.0, 26.0)
    assert loop._phase is _Phase.DRIVE_BRAKE
    assert not loop._overspeed_recovery


def test_trim_stair_cap_steps_down_and_last_step_keeps_holding() -> None:
    """cap に達したら低い段へ下げる。最後の段は cap に達しても保持を続ける。"""
    pattern = _stair()
    loop, *_ = _loop([pattern])
    cap = loop._accel_speed_cap
    loop._enter_phase(_Phase.CRUISE_TRIM, 0.0)
    loop._advance(pattern, cap, 0.0, 0.1)
    assert (loop._phase, _trim_command(loop, pattern, 0.1)) == (_Phase.CRUISE_TRIM, 15.0)
    loop._advance(pattern, cap, 0.0, 0.2)
    assert (loop._phase, _trim_command(loop, pattern, 0.2)) == (_Phase.CRUISE_TRIM, 12.0)
    loop._advance(pattern, cap + 1.0, 0.0, 0.3)
    assert (loop._phase, _trim_command(loop, pattern, 0.3)) == (_Phase.CRUISE_TRIM, 12.0)
    loop._advance(pattern, loop._profile.max_speed + 0.1, 0.0, 0.4)  # 最高速超えは今と同じ回復
    assert loop._phase is _Phase.DRIVE_BRAKE and loop._overspeed_recovery


def test_trim_stair_ends_at_coast_stop_speed() -> None:
    pattern = _stair(50.0)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_TRIM, 0.0)
    loop._advance(pattern, loop._config.coast_down_stop_speed_kmh, 0.0, 1.0)
    assert loop._phase is _Phase.DRIVE_BRAKE
    assert _trim_command(loop, pattern, 1.0) == 0.0


def test_plain_cruise_trim_still_ends_at_cap() -> None:
    pattern = _pattern(PatternKind.CRUISE_TRIM, accel=70.0, trim_opening=11.0, hold=8.0)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_TRIM, 0.0)
    loop._advance(pattern, loop._accel_speed_cap, 0.0, 0.5)
    assert loop._phase is _Phase.DRIVE_BRAKE


async def test_trim_stair_runs_to_completion() -> None:
    config = PatternLoopConfig(accel_ramp_time_s=0.0, brake_ramp_time_s=0.0)
    pattern = plmod.TrimStairPattern(
        PatternKind.CRUISE_TRIM, accel_opening=70.0, brake_opening=0.0, hold_duration_s=0.2,
        trim_opening=18.0, accel_target_kmh=40.0, trim_steps_pct=(18.0, 15.0), step_hold_s=0.1,
    )
    loop, rec, can, *_ = _loop([pattern], speed_kmh=40.0, config=config)
    task = asyncio.ensure_future(_run_until_done(loop, rec))
    while loop._phase is not _Phase.DRIVE_BRAKE and not task.done():
        await asyncio.sleep(0.01)
    can.speed_kmh = 0.0
    await task
    assert rec.completed and not rec.emergency
    trims = sorted({data.accel_opening for data, _, phase, *_ in rec.rows
                    if phase == "CRUISE_TRIM"})
    assert trims == [15.0, 18.0]


# ── 定速階段（段2。2026-09-14） ─────────────────────────────────────────


def _cruise(
    hold_speeds_kmh: tuple[float, ...] = (30.0, 40.0), **kw: float
) -> plmod.CruiseStairPattern:
    return plmod.CruiseStairPattern(
        PatternKind.CRUISE_TRIM, accel_opening=70.0, brake_opening=0.0,
        hold_duration_s=kw.pop("hold_duration_s", 22.0), hold_speeds_kmh=hold_speeds_kmh,
        settle_tol_kmh=kw.pop("settle_tol_kmh", 1.0), settle_s=kw.pop("settle_s", 3.0),
        hold_s=kw.pop("hold_s", 8.0), step_timeout_s=kw.pop("step_timeout_s", 30.0),
        kp=kw.pop("kp", 0.3), ki=kw.pop("ki", 0.05),
        max_rate_pct_per_s=kw.pop("max_rate_pct_per_s", 1.0),
        initial_offset_pct=kw.pop("initial_offset_pct", 7.0),
    )


def test_cruise_hold_pi_raises_below_target_and_lowers_above() -> None:
    """PI の向き: 目標未満なら開度を上げ、目標超過なら下げる（速度偏差 [km/h] → 開度 [%]）。"""
    pattern = _cruise()
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_HOLD, 0.0)
    initial = loop._profile.feedforward_params.accel_deadband_pct + pattern.initial_offset_pct

    loop._last_speed = 20.0  # 目標(30)未満 → 上げる
    raised = loop._cruise_hold_opening(pattern)
    assert raised > initial
    assert raised - initial == pytest.approx(
        pattern.max_rate_pct_per_s * loop._interval_s, abs=1e-6
    )  # 1 周期の変化量はレート制限どおり

    loop._last_speed = 60.0  # 目標(30)超過 → 下げる
    lowered = loop._cruise_hold_opening(pattern)
    assert lowered < raised


def test_cruise_hold_clamps_to_deadband_and_max_opening() -> None:
    """大きな偏差でも [アクセル不感帯, max_accel_opening] にクランプする。"""
    pattern = _cruise(kp=100.0, ki=0.0, max_rate_pct_per_s=100000.0)  # 1 周期で飽和させる
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_HOLD, 0.0)

    loop._last_speed = 0.0  # 大幅に不足 → 上限に張り付く
    assert loop._cruise_hold_opening(pattern) == pytest.approx(loop._profile.max_accel_opening)

    loop._cruise_opening = None  # 開度を初期化し直して逆方向も確認
    loop._last_speed = 1000.0  # 大幅に超過 → 不感帯に張り付く
    assert loop._cruise_hold_opening(pattern) == pytest.approx(
        loop._profile.feedforward_params.accel_deadband_pct
    )


def test_cruise_hold_settles_holds_then_advances_to_next_speed() -> None:
    """許容幅に settle_s 続けて入ったら保持タイマー開始、hold_s 経過で次の車速へ。

    積分は次の車速でも引き継ぐ（settle/hold タイマーだけリセットする）。
    """
    pattern = _cruise((30.0, 40.0), settle_tol_kmh=1.0, settle_s=1.0, hold_s=2.0)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_HOLD, 0.0)

    loop._advance_cruise_hold(30.5, 0.0)
    assert loop._cruise_settle_since == 0.0 and loop._cruise_hold_start is None
    loop._advance_cruise_hold(30.5, 1.0)  # settle_s 経過 → 保持タイマー開始
    assert loop._cruise_hold_start == 1.0
    assert loop._trim_step == 0
    loop._advance_cruise_hold(30.5, 2.9)
    assert loop._trim_step == 0  # まだ hold_s に届かない
    loop._cruise_integral = 3.3  # 積分状態を仕込んで、次の車速でも引き継ぐことを確認する
    loop._advance_cruise_hold(30.5, 3.0)  # hold_s 経過 → 次の車速へ
    assert loop._trim_step == 1
    assert loop._cruise_settle_since is None and loop._cruise_hold_start is None  # タイマーだけ
    assert loop._cruise_integral == pytest.approx(3.3)  # リセットする


def test_cruise_hold_settle_timer_resets_if_out_of_tolerance_before_hold_starts() -> None:
    pattern = _cruise((30.0, 40.0), settle_tol_kmh=1.0, settle_s=1.0, hold_s=2.0)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_HOLD, 0.0)

    loop._advance_cruise_hold(30.5, 0.0)
    assert loop._cruise_settle_since == 0.0
    loop._advance_cruise_hold(35.0, 0.5)  # 許容幅を外れる（保持タイマーはまだ始まっていない）
    assert loop._cruise_settle_since is None
    loop._advance_cruise_hold(30.5, 0.6)  # やり直し
    assert loop._cruise_settle_since == 0.6


def test_cruise_hold_step_timeout_advances_without_settling() -> None:
    pattern = _cruise((30.0, 40.0), step_timeout_s=5.0, settle_s=100.0, hold_s=100.0)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_HOLD, 0.0)

    loop._advance_cruise_hold(20.0, 4.9)  # 全然保持できていない
    assert loop._trim_step == 0
    loop._advance_cruise_hold(20.0, 5.0)  # step_timeout_s 経過 → 保持できなくても次へ
    assert loop._trim_step == 1


def test_cruise_hold_finishes_with_stop_return_after_last_speed() -> None:
    pattern = _cruise((30.0, 40.0), settle_s=0.1, hold_s=0.1)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_HOLD, 0.0)
    loop._trim_step = 1  # 最後の車速

    loop._advance_cruise_hold(40.5, 0.0)
    loop._advance_cruise_hold(40.5, 0.1)  # 保持タイマー開始
    loop._advance_cruise_hold(40.5, 0.3)  # hold_s 経過 → 最後なので停車復帰
    assert loop._phase is _Phase.DRIVE_BRAKE
    assert not loop._overspeed_recovery


def test_cruise_hold_overspeed_enters_drive_brake_recovery() -> None:
    pattern = _cruise((30.0, 40.0))
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_HOLD, 0.0)
    loop._advance_cruise_hold(loop._profile.max_speed + 1.0, 1.0)
    assert loop._phase is _Phase.DRIVE_BRAKE and loop._overspeed_recovery


def test_cruise_hold_low_speed_finishes_like_trim_stair() -> None:
    pattern = _cruise((30.0, 40.0))
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_HOLD, 0.0)
    loop._advance_cruise_hold(loop._config.coast_down_stop_speed_kmh, 1.0)
    assert loop._phase is _Phase.DRIVE_BRAKE  # 停車していないので停車復帰へ


def test_drive_accel_exits_at_first_cruise_hold_speed() -> None:
    """DRIVE_ACCEL は hold_speeds_kmh の最初の値で終える（_accel_target_kmh）。"""
    pattern = _cruise((30.0, 40.0))
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.DRIVE_ACCEL, 0.0)
    loop._advance(pattern, 29.9, 5.0, 1.0)
    assert loop._phase is _Phase.DRIVE_ACCEL
    loop._advance(pattern, 30.0, 5.0, 2.0)  # 先読みなしで目標車速に切り替える
    assert loop._phase is _Phase.CRUISE_HOLD


async def test_cruise_stair_runs_to_completion() -> None:
    config = PatternLoopConfig(accel_ramp_time_s=0.0, brake_ramp_time_s=0.0)
    pattern = plmod.CruiseStairPattern(
        PatternKind.CRUISE_TRIM, accel_opening=70.0, brake_opening=0.0, hold_duration_s=1.0,
        hold_speeds_kmh=(30.0, 40.0), settle_tol_kmh=5.0, settle_s=0.05, hold_s=0.05,
        step_timeout_s=0.3, kp=0.3, ki=0.05, max_rate_pct_per_s=100.0, initial_offset_pct=7.0,
    )
    loop, rec, can, *_ = _loop([pattern], speed_kmh=30.0, config=config, interval_s=0.02)
    task = asyncio.ensure_future(_run_until_done(loop, rec))
    while loop._phase is not _Phase.DRIVE_BRAKE and not task.done():
        await asyncio.sleep(0.01)
    can.speed_kmh = 0.0
    await task
    assert rec.completed and not rec.emergency
    phases = {phase for _, _, phase, *_ in rec.rows}
    assert "CRUISE_HOLD" in phases  # CSV の phase 列で CRUISE_TRIM と見分けられる


# ── 低開度階段（段3-1。2026-09-19。ProblemReport_20260919 候補(c)） ────────────────────


def _low_stair(
    min_speed_kmh: float = 2.0, target_kmh: float = 4.5,
) -> plmod.LowOpenStairPattern:
    return plmod.LowOpenStairPattern(
        PatternKind.CRUISE_TRIM, accel_opening=6.8, brake_opening=0.0, hold_duration_s=24.0,
        trim_opening=6.8, accel_target_kmh=target_kmh, trim_steps_pct=(6.8, 7.3, 7.8),
        step_hold_s=8.0, min_speed_kmh=min_speed_kmh,
    )


def test_low_open_stair_uses_pattern_min_speed_not_coast_stop_speed() -> None:
    """低速終了判定は min_speed_kmh を使う（トリム階段の coast_down_stop_speed_kmh=5.0 のままだと
    低速の運転域の真ん中に来て 1 段目で即終了してしまう）。"""
    pattern = _low_stair(min_speed_kmh=2.0)
    loop, *_ = _loop([pattern])
    assert loop._config.coast_down_stop_speed_kmh == 5.0  # 前提確認
    loop._enter_phase(_Phase.CRUISE_TRIM, 0.0)
    loop._advance(pattern, 4.0, 0.0, 1.0)  # coast_down_stop_speed_kmh(5.0) より低いが終了しない
    assert loop._phase is _Phase.CRUISE_TRIM


def test_low_open_stair_finishes_below_its_own_min_speed() -> None:
    pattern = _low_stair(min_speed_kmh=2.0)
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_TRIM, 0.0)
    loop._advance(pattern, 1.5, 0.0, 1.0)  # min_speed_kmh(2.0) を下回った
    assert loop._phase is _Phase.DRIVE_BRAKE
    assert _trim_command(loop, pattern, 1.0) == 0.0


def test_low_open_stair_holds_each_step_constant_without_ramp() -> None:
    """CRUISE_TRIM の指令は trim_steps_pct[i] そのもの（ランプもガバナーも掛からない）。"""
    pattern = _low_stair()
    loop, *_ = _loop([pattern])
    loop._enter_phase(_Phase.CRUISE_TRIM, 0.0)
    assert _trim_command(loop, pattern, 0.1) == pattern.trim_steps_pct[0]
    loop._trim_step = 1
    assert _trim_command(loop, pattern, 1.1) == pattern.trim_steps_pct[1]
    loop._trim_step = 2
    assert _trim_command(loop, pattern, 2.1) == pattern.trim_steps_pct[2]


async def test_low_open_stair_runs_to_completion() -> None:
    config = PatternLoopConfig(accel_ramp_time_s=0.0, brake_ramp_time_s=0.0)
    pattern = plmod.LowOpenStairPattern(
        PatternKind.CRUISE_TRIM, accel_opening=6.8, brake_opening=0.0, hold_duration_s=0.2,
        trim_opening=6.8, accel_target_kmh=4.5, trim_steps_pct=(6.8, 7.3), step_hold_s=0.1,
        min_speed_kmh=2.0,
    )
    loop, rec, can, *_ = _loop([pattern], speed_kmh=4.5, config=config)
    task = asyncio.ensure_future(_run_until_done(loop, rec))
    while loop._phase is not _Phase.DRIVE_BRAKE and not task.done():
        await asyncio.sleep(0.01)
    can.speed_kmh = 0.0
    await task
    assert rec.completed and not rec.emergency
    trims = sorted({data.accel_opening for data, _, phase, *_ in rec.rows
                    if phase == "CRUISE_TRIM"})
    assert trims == [6.8, 7.3]


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
    assert cfg.accel_full_range_timeout_s == 30.0
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
    config = PatternLoopConfig(accel_full_range_timeout_s=0.1, brake_stop_timeout_s=0.3,
                               accel_ramp_time_s=0.0, brake_ramp_time_s=0.0)
    loop, rec, *_ = _loop(
        [_pattern(PatternKind.ACCEL_SWEEP, accel=30.0, brake=30.0)], speed_kmh=50.0, config=config
    )
    await _run_until_done(loop, rec)
    assert rec.emergency and not rec.completed
    assert loop.abort_reason is not None
    assert "停車しません" in loop.abort_reason and "DRIVE_BRAKE" in loop.abort_reason
    assert {r[2] for r in rec.rows} == {"DRIVE_ACCEL", "DRIVE_BRAKE"}


async def test_loop_completes_after_stop_return() -> None:
    config = PatternLoopConfig(accel_full_range_timeout_s=0.1, accel_ramp_time_s=0.0)
    loop, rec, can, *_ = _loop(
        [_pattern(PatternKind.ACCEL_SWEEP, accel=30.0, brake=30.0)], speed_kmh=50.0, config=config
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
