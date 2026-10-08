"""G ランプ走行（pattern_loop の GRampPattern / RAMP_UP / RAMP_DOWN）のテスト（段2k 段B）。

走行の状態機械（同期。`_command_openings` と `_advance` を直接呼ぶ）を見る: 実測 G と狙いの差で
効く開度 u が動く・上端で減速へ・u の符号でペダルが切り替わる・弱い減速でアクセルを踏んだまま・
ガバナーで頭打ち・停車で次のパターンへ・実測があれば最初の u を見積もる。
フェイク（アクチュエータ・CAN・記録）は test_research_pattern_loop.py と同じ書き方で持ち込んだ。
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import pattern_loop as plmod
from tests.research.axis_monitor import AxisMonitor
from tests.research.g_limit import PEDAL_ACCEL, PEDAL_BRAKE
from tests.research.pattern_loop import PatternLoop, PatternLoopConfig, _Phase
from tests.research.research_types import G_TO_KMHS, DriveLogData, LearningPattern, PatternKind
from tests.research.vehicle import build_vehicle_profile

CFG_PATH = Path("tests/research/config_testVehicle.yaml")


class FakeAxis:
    """位置指令を覚えるアクチュエータ。"""

    def __init__(self) -> None:
        self.position = 0

    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None:
        self.position = target_pos

    async def read_monitor(self) -> AxisMonitor:
        return AxisMonitor(
            position_pulse=self.position, current_ma=0.0, alarm_code=0,
            servo_on=True, moving=False, pos_done=True,
        )

    async def is_alarm_active(self) -> bool:
        return False


class FakeCAN:
    def __init__(self, speed_kmh: float) -> None:
        self.speed_kmh = speed_kmh

    async def read_speed(self) -> float:
        return self.speed_kmh


class Recorder:
    def __init__(self) -> None:
        self.completed = False
        self.emergency = False

    def on_sample(
        self, data: DriveLogData, idx: int, phase: str, governed: bool,
        alarm_accel: bool | None, alarm_brake: bool | None,
        monitor_accel: AxisMonitor, monitor_brake: AxisMonitor, cycle_ms: float,
    ) -> None:
        pass

    async def on_complete(self) -> None:
        self.completed = True

    async def on_emergency(self) -> None:
        self.emergency = True


def _loop(
    patterns: list[LearningPattern], *, config: PatternLoopConfig | None = None,
    launched: bool = True,
) -> PatternLoop:
    """launched=True は RAMP_UP の発進の段階（段2l-6）を終えた状態から始める。発進そのものを
    見るテストは launched=False にする。"""
    cfg = cfgmod.load_config(CFG_PATH)
    rec = Recorder()
    loop = PatternLoop(
        accel_driver=FakeAxis(), brake_driver=FakeAxis(), can_reader=FakeCAN(0.0),
        profile=build_vehicle_profile(cfg), patterns=patterns, overcurrent_limit_ma=5000.0,
        on_complete=rec.on_complete, on_emergency=rec.on_emergency, on_sample=rec.on_sample,
        config=config, interval_s=0.1,
    )
    if launched:
        loop._rp_launch = False
    return loop


def _ramp(target_g: float = 0.1, top_kmh: float = 60.0) -> plmod.GRampPattern:
    return plmod.GRampPattern(
        PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.0,
        target_g=target_g, top_kmh=top_kmh,
    )


def _next_pattern() -> LearningPattern:
    return LearningPattern(PatternKind.COAST_DOWN, 0.0, 0.0, 0.3)


def _cycle(loop: PatternLoop, pattern: plmod.GRampPattern, speed: float, t: float) -> tuple[
    float, float
]:
    """1 周期: 指令開度を決め（`_execute_one_cycle` と同じ順）、状態を進める。"""
    if loop._phase_started_at is None:
        loop._phase_started_at = t
    loop._last_speed = speed
    loop._update_governor(0.0)
    a, b = loop._command_openings(pattern, t)
    loop._current_accel_opening, loop._current_brake_opening = a, b
    loop._advance(pattern, speed, 0.0, t)
    return a, b


def _run(
    loop: PatternLoop, pattern: plmod.GRampPattern, speed_of: Callable[[float], float],
    t0: float, t1: float, dt: float = 0.1,
) -> float:
    t = t0
    while t < t1:
        _cycle(loop, pattern, speed_of(t), t)
        t += dt
    return t


class _Plant:
    """u から加速度を作る簡単な車（惰行 −coast、感度 ga・gb はアクセル・ブレーキ）。"""

    def __init__(self, loop: PatternLoop, speed: float, coast: float = 1.0,
                 ga: float = 0.2, gb: float = 0.4) -> None:
        self.loop, self.speed, self.coast, self.ga, self.gb = loop, speed, coast, ga, gb

    def step(self, dt: float = 0.1) -> None:
        u = self.loop._rp_u or 0.0
        a = -self.coast + (self.ga * u if u >= 0.0 else self.gb * u)
        self.speed = max(0.0, self.speed + a * dt)


def _deadbands(loop: PatternLoop) -> tuple[float, float]:
    ff = loop._profile.feedforward_params
    return ff.accel_deadband_pct, ff.brake_deadband_pct


# ── 始まり方 ────────────────────────────────────────────────────────


def test_ramp_pattern_starts_in_ramp_up_with_phase_name() -> None:
    loop = _loop([_ramp(), _next_pattern()])
    assert loop._phase is _Phase.RAMP_UP
    assert loop._phase.name == "RAMP_UP" and _Phase.RAMP_DOWN.name == "RAMP_DOWN"
    assert PatternKind.G_RAMP.value == "g_ramp"
    # u が決まる前の最初の指令は u ＝ 0（見積もれない）→ 両ペダル 0
    p = _ramp()
    assert _cycle(loop, p, 0.0, 0.0) == (0.0, 0.0)
    assert loop._rp_u == 0.0


# ── u の動き ────────────────────────────────────────────────────────


def test_u_rises_when_measured_g_is_below_target() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    _run(loop, p, lambda t: 10.0, 0.0, 2.0)  # 車速が動かない（G ＝ 0）
    assert loop._rp_u is not None and loop._rp_u > 1.0


def test_u_falls_when_measured_g_is_above_target() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = 20.0
    _run(loop, p, lambda t: 10.0 + 0.3 * G_TO_KMHS * t, 0.1, 2.0)  # 0.3G で加速中
    # 2026-10-07 段2l-2: 下がるが 0 より下（ブレーキ側）へは行かない（下限 0）
    assert 0.0 <= loop._rp_u < 20.0


def test_u_holds_inside_the_margin() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    cfg = loop._config
    margin = max(0.1 * cfg.ramp_hold_margin_frac, cfg.ramp_hold_margin_min_g)
    slope = (0.1 - 0.5 * margin) * G_TO_KMHS  # 狙いより保持幅の半分だけ低い
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = 7.0
    _run(loop, p, lambda t: 10.0 + slope * t, 0.1, 3.0)
    assert loop._rp_u == pytest.approx(7.0)


def test_hold_margin_has_a_floor_for_small_targets() -> None:
    """0.02G の段: 狙い × 10% ＝ 0.002G は下限 0.005G に切り上がる。"""
    p = _ramp(0.02)
    loop = _loop([p])
    slope = (0.02 - 0.004) * G_TO_KMHS  # 差 0.004G ＜ 下限 0.005G
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = 3.0
    _run(loop, p, lambda t: 10.0 + slope * t, 0.1, 3.0)
    assert loop._rp_u == pytest.approx(3.0)


# ── 上端で減速へ ─────────────────────────────────────────────────────


def test_reaching_top_speed_switches_to_ramp_down() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _run(loop, p, lambda t: 30.0, 0.0, 0.5)
    assert loop._phase is _Phase.RAMP_UP
    _cycle(loop, p, 60.5, 0.6)
    assert loop._phase is _Phase.RAMP_DOWN
    assert loop._pattern_idx == 0


def test_ramp_up_times_out_into_ramp_down_when_top_is_not_reached() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    limit = 60.0 / (0.1 * G_TO_KMHS)  # 2026-10-08 段2l-6: 見込み秒数は 0 km/h までで計算
    timeout = limit * plmod.RAMP_LEG_TIMEOUT_FACTOR + plmod.RAMP_LEG_TIMEOUT_MARGIN_S
    _cycle(loop, p, 20.0, 0.0)
    _cycle(loop, p, 20.0, timeout - 1.0)
    assert loop._phase is _Phase.RAMP_UP
    _cycle(loop, p, 20.0, timeout + 0.1)
    assert loop._phase is _Phase.RAMP_DOWN


# ── u の符号とペダル ───────────────────────────────────────────────────


def test_u_sign_selects_pedal_with_deadband_offset() -> None:
    p = _ramp()
    loop = _loop([p])
    adb, bdb = _deadbands(loop)
    _cycle(loop, p, 0.0, 0.0)
    loop._rp_u = 5.0
    assert loop._command_openings(p, 0.1) == pytest.approx((adb + 5.0, 0.0))
    loop._rp_u = -5.0
    assert loop._command_openings(p, 0.2) == pytest.approx((0.0, bdb + 5.0))
    loop._rp_u = 0.0
    assert loop._command_openings(p, 0.3) == (0.0, 0.0)


# ── 弱い減速はアクセルを踏んだまま ──────────────────────────────────────


def test_weak_deceleration_keeps_the_accelerator_pressed() -> None:
    p = _ramp(0.02, top_kmh=100.0)  # 0.02G ＝ 0.71 km/h/s ＜ 惰行 1.0 km/h/s
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_u = 0.0
    plant = _Plant(loop, 100.0)
    us: list[float] = []
    t = 0.0
    while t < 40.0:
        _cycle(loop, p, plant.speed, t)
        plant.step()
        us.append(loop._rp_u or 0.0)
        t += 0.1
    adb, _ = _deadbands(loop)
    assert min(us) >= -1e-9  # ブレーキ側（u ＜ 0）へ入らない
    assert us[-1] > 0.5  # アクセルを少し踏んだまま
    assert loop._current_accel_opening == pytest.approx(adb + us[-1], abs=1.0)
    assert loop._current_brake_opening == 0.0
    assert -0.03 < (plant.speed - 100.0) / 40.0 / G_TO_KMHS < -0.01  # 狙い付近の減速


def test_strong_deceleration_crosses_zero_into_the_brake_continuously() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_u = 5.0  # 加速側から切り替わった想定（見積もりなし）
    plant = _Plant(loop, 100.0)
    us: list[float] = []
    t = 0.0
    while t < 8.0:
        _cycle(loop, p, plant.speed, t)
        plant.step()
        us.append(loop._rp_u or 0.0)
        t += 0.1
    assert any(u > 0.0 for u in us[:3]) and us[-1] < -5.0
    assert loop._rp_brake_latched  # ブレーキ側に入った印が立つ（2026-10-07 段2l-2）
    # u は 1 周期ごとの小さな動きで 0 をまたぐ（飛ばない）
    assert max(abs(b - a) for a, b in zip(us, us[1:], strict=False)) < 3.5
    assert loop._current_accel_opening == 0.0 and loop._current_brake_opening > 0.0


# ── ガバナー ────────────────────────────────────────────────────────


def test_governor_caps_the_pedal_in_use_in_ramp_up_and_ramp_down() -> None:
    p = _ramp(0.35)
    loop = _loop([p])
    limit = loop._g_limit_kmhs
    _cycle(loop, p, 20.0, 0.0)
    loop._rp_u = 20.0
    a, _ = loop._command_openings(p, 0.1)
    loop._current_accel_opening = a
    loop._update_governor(limit + 1.0)  # 上限超え: 現在開度で頭打ち
    assert loop._accel_gov_cap == pytest.approx(a)
    loop._update_governor(limit + 1.0)  # 以降は 1 周期ごとに下げる
    assert loop._accel_gov_cap == pytest.approx(a - loop._config.gov_reduce_step_pct)
    assert loop._governor_limiting()
    capped, _ = loop._command_openings(p, 0.2)
    assert capped <= loop._accel_gov_cap + 1e-9 < a

    # 減速側: ブレーキが頭打ちされ、使っていないアクセル側の古い頭打ちは持ち越さない
    loop._enter_phase(_Phase.RAMP_DOWN, 1.0)
    loop._rp_u = -15.0
    _, b = loop._command_openings(p, 1.1)
    loop._current_brake_opening = b
    loop._update_governor(-(limit + 1.0))
    assert loop._brake_gov_cap == pytest.approx(b)
    assert loop._accel_gov_cap is None
    assert loop._command_openings(p, 1.2)[1] <= b


def test_governor_stops_u_from_growing_while_it_is_limiting() -> None:
    p = _ramp(0.35)
    loop = _loop([p])
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = 10.0
    loop._accel_request = 14.0
    loop._accel_gov_cap = 5.0  # 削られている
    loop._current_accel_opening = 5.0
    u0 = loop._rp_u
    _run(loop, p, lambda t: 10.0, 0.1, 1.5)  # G ＜ 狙い（本来は u を増やす）
    assert loop._rp_u <= u0 + 1e-9


# ── 停車復帰と次のパターン ───────────────────────────────────────────────


def test_ramp_down_continues_to_standstill_then_hands_over_to_the_next_pattern() -> None:
    """2026-10-08 段2l-6: 5 km/h（coast_down_stop_speed_kmh）では終わらず、停車まで続ける。
    停車を見たらすぐ _finish_pattern（停車していれば次のパターンへ）。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p, _next_pattern()])
    _cycle(loop, p, 60.5, 0.0)  # 上端
    _cycle(loop, p, 60.5, 0.1)
    assert loop._phase is _Phase.RAMP_DOWN
    t = 0.2
    for v in (40.0, 20.0, 8.0, loop._config.coast_down_stop_speed_kmh, 2.0, 0.5):
        _cycle(loop, p, v, t)
        t += 1.0
        assert loop._phase is _Phase.RAMP_DOWN  # 5 km/h でも 0.5 km/h でも続く
    assert loop._pattern_idx == 0
    _cycle(loop, p, 0.0, t)  # 停車
    assert loop._pattern_idx == 1  # 次のパターンへ（停車しているので DRIVE_BRAKE を経ない）
    assert loop._phase is not _Phase.DRIVE_BRAKE
    assert len(loop.ramp_results) == 1


def test_ramp_down_times_out_into_stop_return() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _cycle(loop, p, 60.5, 0.0)
    assert loop._phase is _Phase.RAMP_DOWN
    leg = 60.0 / (0.1 * G_TO_KMHS)  # 0 km/h まで
    timeout = leg * plmod.RAMP_LEG_TIMEOUT_FACTOR + plmod.RAMP_LEG_TIMEOUT_MARGIN_S
    _cycle(loop, p, 50.0, 0.1)
    _cycle(loop, p, 50.0, 0.1 + timeout + 0.5)
    assert loop._phase is _Phase.DRIVE_BRAKE


def test_overspeed_in_ramp_recovers_with_brake() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p])
    over = loop._profile.max_speed + 5.0
    # 最高速超えが続く間: まず上端扱いで RAMP_DOWN へ、そこで連続して超えれば停車復帰（回復）へ
    for i in range(2 * loop._config.skip_consecutive_required):
        _cycle(loop, p, over, 0.1 * i)
    assert loop._phase is _Phase.DRIVE_BRAKE and loop._overspeed_recovery


# ── 結果の記録 ──────────────────────────────────────────────────────────


def test_result_records_durations_median_g_and_top_speed() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    g = 0.1 * G_TO_KMHS
    t = 0.0
    speed = 0.0
    while speed < 60.0:  # 0.1G で加速
        _cycle(loop, p, speed, t)
        speed += g * 0.1
        t += 0.1
    up_end = t
    while speed > 0.0:  # 0.1G で停車まで減速（段2l-6: 5 km/h ではなく停車まで）
        _cycle(loop, p, speed, t)
        speed = max(0.0, speed - g * 0.1)
        t += 0.1
    _cycle(loop, p, speed, t)
    assert len(loop.ramp_results) == 1
    r = loop.ramp_results[0]
    assert r.target_g == 0.1 and r.top_kmh == 60.0
    assert r.up_s == pytest.approx(up_end, abs=0.3)
    assert r.down_s == pytest.approx(t - up_end, abs=0.3)
    assert r.accel_g_median == pytest.approx(0.1, abs=0.01)
    assert r.decel_g_median == pytest.approx(0.1, abs=0.01)
    assert r.max_speed_kmh >= 60.0


# ── 上限 G の予測マップとの連携 ─────────────────────────────────────────


def test_first_u_of_ramp_up_comes_from_glimit_estimate() -> None:
    p = _ramp(0.08)
    loop = _loop([p])
    adb, _ = _deadbands(loop)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 6.0, 1.0)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 10.0, 3.0)
    target = 0.08 * G_TO_KMHS
    expected_opening = 6.0 + (target - 1.0) / 2.0 * 4.0
    a, b = _cycle(loop, p, 0.0, 0.0)
    assert loop._rp_u == pytest.approx(expected_opening - adb)
    assert a == pytest.approx(expected_opening) and b == 0.0


def test_first_u_of_ramp_down_comes_from_glimit_estimate_brake() -> None:
    p = _ramp(0.25, top_kmh=60.0)
    loop = _loop([p])
    _, bdb = _deadbands(loop)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 12.0, 4.0)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 20.0, 9.0)
    _cycle(loop, p, 0.0, 0.0)
    loop._rp_u = 30.0  # 加速側の u
    target = 0.25 * G_TO_KMHS
    expected_opening = 12.0 + (target - 4.0) / 5.0 * 8.0
    loop._ramp_switch_down(55.0, 5.0)
    assert loop._phase is _Phase.RAMP_DOWN
    assert loop._rp_u == pytest.approx(-(expected_opening - bdb))


def test_first_u_of_weak_ramp_down_uses_accel_side_when_below_coast() -> None:
    p = _ramp(0.02, top_kmh=60.0)
    loop = _loop([p])
    adb, _ = _deadbands(loop)
    loop.glimit.add_coast(55.0, 1.0)  # 惰行の減速 1.0 km/h/s
    loop.glimit.add(PEDAL_ACCEL, 55.0, adb + 6.0, 1.0)  # この開度で +1.0 km/h/s
    _cycle(loop, p, 0.0, 0.0)
    loop._rp_u = 30.0
    loop._ramp_switch_down(55.0, 5.0)
    # 狙い −0.02G ＝ −0.706 km/h/s は惰行（−1.0）と +1.0 の間 → アクセル側（u ＞ 0）
    assert loop._rp_u is not None and 0.0 < loop._rp_u < 6.0


def test_without_glimit_data_ramp_down_continues_from_current_u() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _cycle(loop, p, 0.0, 0.0)
    loop._rp_u = 12.0
    loop._ramp_switch_down(55.0, 5.0)
    assert loop._rp_u == pytest.approx(12.0)


def test_ramp_measurements_feed_the_glimit_map() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    _cycle(loop, p, 20.0, 0.0)
    loop._rp_u = 10.0
    _run(loop, p, lambda t: 20.0 + 0.5 * t, 0.1, 4.0)  # 実開度（指令で代用）で加速中
    assert loop.glimit.has_data(PEDAL_ACCEL)
    assert not loop.glimit.has_data(PEDAL_BRAKE)


# ── 見積もりは外挿しない（2026-10-08 段2l-6）──────────────────────────────


def test_first_u_estimate_does_not_extrapolate_beyond_the_strongest_point() -> None:
    """狙いが貯めた点のどれより強いときは、いちばん効いた点の開度から始める（外挿しない）。"""
    p = _ramp(0.25)
    loop = _loop([p])
    adb, _ = _deadbands(loop)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 6.0, 1.0)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 10.0, 3.0)  # いちばん効いた点: 10% で 3.0 km/h/s
    assert 0.25 * G_TO_KMHS > 3.0  # 狙いは貯めた点より強い
    a, _ = _cycle(loop, p, 0.0, 0.0)
    assert loop._rp_u == pytest.approx(10.0 - adb)
    assert a == pytest.approx(10.0)


def test_ramp_down_estimate_does_not_extrapolate_either() -> None:
    p = _ramp(0.35, top_kmh=60.0)
    loop = _loop([p])
    _, bdb = _deadbands(loop)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 12.0, 4.0)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 20.0, 9.0)  # いちばん効いた点: 20% で 9.0 km/h/s
    assert 0.35 * G_TO_KMHS > 9.0
    _cycle(loop, p, 0.0, 0.0)
    loop._rp_u = 30.0
    loop._ramp_switch_down(55.0, 5.0)
    assert loop._rp_u == pytest.approx(-(20.0 - bdb))


# ── RAMP_UP の発進: ブレーキを少しずつ離し、踏み直さない（2026-10-08 段2l-6）──────────


def _launch_loop(target_g: float = 0.1, **cfg_kw: float) -> tuple[PatternLoop, plmod.GRampPattern]:
    p = _ramp(target_g, top_kmh=200.0)
    config = PatternLoopConfig(pedal_standby=True, **cfg_kw)
    return _loop([p], config=config, launched=False), p


def test_launch_starts_at_stop_confirm_opening_with_accel_at_zero_u() -> None:
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    assert loop._rp_launch and loop._rp_u is None
    a, b = _cycle(loop, p, 0.0, 0.0)
    assert (a, b) == (0.0, pytest.approx(14.0))  # アクセル 0（出力側 min）・ブレーキ=止まった開度
    assert loop._rp_u is None  # 最初の u の見積もりは保留


def test_launch_without_stop_confirm_starts_from_the_stop_hold_opening() -> None:
    loop, p = _launch_loop()
    hold = loop._profile.feedforward_params.stop_brake_opening_pct
    _, b = _cycle(loop, p, 0.0, 0.0)
    assert b == pytest.approx(min(hold, loop._profile.max_brake_opening))


def test_launch_brake_only_goes_down_to_min_then_hands_over_to_accel() -> None:
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    _, bdb = _deadbands(loop)
    brakes: list[float] = []
    t = 0.0
    for _ in range(80):  # 車は動かない（G ＝ 0 ＜ 狙い）ので、離し続ける
        _, b = _cycle(loop, p, 0.0, t)
        brakes.append(b)
        t += 0.1
        if not loop._rp_launch:
            break
    assert not loop._rp_launch
    # 単調に下がる（上がる周期が 1 つも無い）。止まった開度から min まで
    assert all(b2 <= b1 + 1e-9 for b1, b2 in zip(brakes, brakes[1:], strict=False))
    assert brakes[0] == pytest.approx(14.0) and brakes[-1] > bdb - 1e-9
    assert loop._rp_launch_brake_pct == pytest.approx(bdb)
    # 渡した後: u を見積もる（実測が無いので u ＝ 0）。以後ブレーキには戻らない
    a, b = _cycle(loop, p, 0.0, t)
    assert (a, b) == (0.0, 0.0)  # 渡した最初の指令は u ＝ 0（見積もれない）。ブレーキは出ない
    assert loop._rp_u is not None
    assert loop._phase is _Phase.RAMP_UP
    for i in range(40):
        t += 0.1
        _, b = _cycle(loop, p, 0.0, t)
        assert b == 0.0  # u ≥ 0 なのでブレーキ指令は出ない
    assert loop._rp_u is not None and loop._rp_u > 0.0  # G 不足でアクセルの u が上がっていく


def test_launch_holds_the_brake_position_when_g_reaches_the_target() -> None:
    """実測 G が狙い以上なら、その位置で止めて待つ（さらに離さない）。踏み直しもしない。"""
    loop, p = _launch_loop(0.1, stop_confirm_pct=14.0)
    g = 0.1 * G_TO_KMHS
    t = 0.0
    # 車速が最初から 0.1G で上がっている → 離す必要が無い
    brakes = []
    for i in range(30):
        _, b = _cycle(loop, p, 2.0 + g * t, t)
        brakes.append(b)
        t += 0.1
    assert loop._rp_launch
    assert brakes[-1] == pytest.approx(14.0)
    assert max(brakes) == pytest.approx(14.0)


def test_launch_never_raises_the_brake_even_if_g_overshoots_then_falls() -> None:
    loop, p = _launch_loop(0.1, stop_confirm_pct=14.0)
    g = 0.1 * G_TO_KMHS
    t = 0.0
    brakes = []
    speed = 0.0
    for i in range(60):
        # 前半は止まったまま（離す）、中盤は狙いを大きく超えて加速、終盤は減速（G が負）
        if i < 8:
            rate = 0.0
        elif i < 30:
            rate = 3.0 * g
        else:
            rate = -2.0 * g
        speed = max(0.0, speed + rate * 0.1)
        _, b = _cycle(loop, p, speed, t)
        brakes.append(b)
        t += 0.1
        if not loop._rp_launch:
            break
    assert all(b2 <= b1 + 1e-9 for b1, b2 in zip(brakes, brakes[1:], strict=False))


def test_launch_end_estimates_u_from_the_glimit_map() -> None:
    loop, p = _launch_loop(0.08, stop_confirm_pct=5.0)
    adb, _ = _deadbands(loop)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 6.0, 1.0)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 10.0, 3.0)
    t = 0.0
    while loop._rp_launch and t < 10.0:
        _cycle(loop, p, 2.0, t)
        t += 0.1
    assert not loop._rp_launch and loop._rp_u is None  # 渡す時点で見積もる（まだ保留）
    a, b = _cycle(loop, p, 2.0, t)  # 渡した最初の指令が見積もりの開度
    expected_opening = 6.0 + (0.08 * G_TO_KMHS - 1.0) / 2.0 * 4.0
    assert a == pytest.approx(expected_opening) and b == 0.0
    assert loop._rp_u is not None and loop._rp_u >= expected_opening - adb


def test_launch_does_not_press_again_when_the_brake_is_already_below_stop_confirm() -> None:
    """前の減速が止まった開度より低いブレーキで停車したなら、そこから始める（踏み直さない）。"""
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    calib = loop._profile.calibration
    assert calib is not None
    loop._brake_pos_cmd = int(10.0 / 100.0 * calib.brake_full_pos)  # 実際の位置は約 10%
    loop._rp_launch = False
    loop._enter_phase(_Phase.RAMP_UP, 5.0)
    assert loop._rp_launch_brake_pct == pytest.approx(10.0, abs=0.02)
    _, b = _cycle(loop, p, 0.0, 5.0)
    assert b == pytest.approx(10.0, abs=0.02)


def test_launch_stage_resets_for_every_ramp_up_and_ends_at_ramp_down() -> None:
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    loop._rp_launch = False
    loop._enter_phase(_Phase.RAMP_UP, 5.0)  # 次の G ランプの頭: 発進の段階から
    assert loop._rp_launch and loop._rp_launch_brake_pct == pytest.approx(14.0)
    loop._enter_phase(_Phase.RAMP_DOWN, 6.0)  # 発進中に打ち切られても RAMP_DOWN には持ち越さない
    assert not loop._rp_launch


def test_launch_keeps_the_governor_working_on_the_brake() -> None:
    """発進の段階のブレーキ指令にも G ガバナー（頭打ち）が効く（安全系はそのまま）。"""
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    _cycle(loop, p, 0.0, 0.0)
    loop._brake_gov_cap = 5.0
    loop._brake_request = 14.0
    a, b = loop._command_openings(p, 0.1)
    assert b <= 5.0 + 1e-9


def test_launch_records_brake_points_only_when_deceleration_is_positive() -> None:
    """発進の段階でアクセルが min のまま加速している間は、ブレーキの点（負の減速）を貯めない。"""
    loop, p = _launch_loop(0.1, stop_confirm_pct=14.0)
    ff = loop._profile.feedforward_params
    t = 0.0
    for _ in range(40):
        loop._brake_actual_pct = ff.brake_deadband_pct + 8.0
        loop._accel_actual_pct = ff.accel_deadband_pct
        _cycle(loop, p, 0.5 * t, t)  # 加速中 → ブレーキ側の減速は負
        t += 0.1
        if not loop._rp_launch:
            break
    assert not loop.glimit.has_data(PEDAL_BRAKE)


# ── ゲインを実測の感度から決める（2026-10-07 段2l-2）──────────────────────────


def _one_step_u(loop: PatternLoop, p: plmod.GRampPattern, u0: float, speed: float,
                measured_g: float, t0: float = 0.0) -> float:
    """u を u0 にし、実測 G が measured_g になる車速を 3 周期流して最後の u の増分を返す。"""
    _cycle(loop, p, speed, t0)
    loop._rp_u = u0
    dv = measured_g * G_TO_KMHS * 0.1
    _cycle(loop, p, speed + dv, t0 + 0.1)
    _cycle(loop, p, speed + 2 * dv, t0 + 0.2)
    before = loop._rp_u
    _cycle(loop, p, speed + 3 * dv, t0 + 0.3)
    assert before is not None and loop._rp_u is not None
    return loop._rp_u - before


def test_gain_is_ramp_rate_gain_when_sensitivity_is_not_measured() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    du = _one_step_u(loop, p, 5.0, 20.0, 0.0)  # 実測 G ＝ 0、狙い 0.1G
    assert du == pytest.approx(loop._config.ramp_rate_gain * 0.1 * 0.1)


def test_gain_shrinks_with_measured_brake_sensitivity() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _, bdb = _deadbands(loop)
    sens = 1.0  # km/h/s per %（ブレーキ。開度 bdb+ の点を直線で）
    for k in range(5):
        loop.glimit.add(PEDAL_BRAKE, 60.0, bdb + 2.0 * k + 1.0, sens * (2.0 * k + 1.0))
    loop._brake_actual_pct = bdb + 4.0
    cfg = loop._config
    expected_gain = min(cfg.ramp_rate_gain, 1.0 / ((sens / G_TO_KMHS) * cfg.ramp_settle_time_s))
    assert expected_gain < cfg.ramp_rate_gain
    # 狙い −0.3G、実測 0G（減速が足りない）→ u は負の向きへ gain × 0.3 × dt
    du = _one_step_u(loop, p, -3.0, 60.0, 0.0)
    assert du == pytest.approx(-expected_gain * 0.3 * 0.1, rel=0.02)


def test_gain_never_exceeds_ramp_rate_gain() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    adb, _ = _deadbands(loop)
    for k in range(5):  # ほぼ効かないアクセル（感度 0.001 km/h/s per %）
        loop.glimit.add(PEDAL_ACCEL, 20.0, adb + 2.0 * k + 1.0, 0.001 * (2.0 * k + 1.0))
    loop._accel_actual_pct = adb + 5.0
    du = _one_step_u(loop, p, 5.0, 20.0, 0.0)
    assert du == pytest.approx(loop._config.ramp_rate_gain * 0.1 * 0.1)


# ── 加速はブレーキへ行かない・減速は一度ブレーキに入ったらアクセルへ戻らない ──────────


def test_ramp_up_does_not_go_into_the_brake_even_when_g_is_above_target() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p])
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = 1.0
    # 0.3G でずっと加速（狙い 0.1G より強い）→ u は 0 で止まる
    _run(loop, p, lambda t: 10.0 + 0.3 * G_TO_KMHS * t, 0.1, 5.0)
    assert loop._phase is _Phase.RAMP_UP
    assert loop._rp_u == 0.0
    assert loop._command_openings(p, 5.1) == (0.0, 0.0)


def test_ramp_down_never_returns_to_the_accelerator_after_entering_the_brake() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_u = -5.0
    loop._rp_brake_latched = True
    # 狙い 0.3G より強い減速（0.6G）がずっと続く → u は上がるが 0 を超えない
    _run(loop, p, lambda t: 100.0 - 0.6 * G_TO_KMHS * t, 0.1, 6.0)
    assert loop._rp_u is not None and loop._rp_u <= 0.0
    assert loop._rp_u == 0.0
    a, _ = loop._command_openings(p, 6.1)
    assert a == 0.0


def test_ramp_down_latch_is_set_by_stepping_into_the_brake() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_u = 0.5
    assert not loop._rp_brake_latched
    _run(loop, p, lambda t: 100.0, 0.1, 1.5)  # 減速が足りない → u は負へ
    assert loop._rp_u < 0.0 and loop._rp_brake_latched


def test_ramp_down_weak_deceleration_can_still_use_the_accelerator_before_latch() -> None:
    p = _ramp(0.02, top_kmh=100.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_u = 0.0
    assert not loop._rp_brake_latched
    lo, hi = loop._ramp_u_limits()
    assert hi > 0.0 and lo < 0.0  # 印が立つまでは両側が使える
    plant = _Plant(loop, 100.0)
    t = 0.0
    while t < 40.0:
        _cycle(loop, p, plant.speed, t)
        plant.step()
        t += 0.1
    assert loop._rp_u is not None and loop._rp_u > 0.5
    assert not loop._rp_brake_latched


def test_latch_is_set_by_negative_estimate_on_switch_and_cleared_on_next_ramp_up() -> None:
    p = _ramp(0.25, top_kmh=60.0)
    loop = _loop([p])
    loop.glimit.add(PEDAL_BRAKE, 55.0, 12.0, 4.0)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 20.0, 9.0)
    _cycle(loop, p, 0.0, 0.0)
    loop._rp_u = 30.0
    loop._ramp_switch_down(55.0, 5.0)
    assert loop._rp_u < 0.0 and loop._rp_brake_latched
    # 見積もりが無い（アクセル側へ戻る可能性のある）切り替えでは印が下りている
    loop2 = _loop([_ramp(0.1, top_kmh=60.0)])
    _cycle(loop2, _ramp(0.1, top_kmh=60.0), 0.0, 0.0)
    loop2._rp_brake_latched = True
    loop2._rp_u = 12.0
    loop2._ramp_switch_down(55.0, 5.0)
    assert not loop2._rp_brake_latched
    # 次の G ランプの RAMP_UP に入ったら下りる
    loop._rp_brake_latched = True
    loop._enter_phase(_Phase.RAMP_UP, 10.0)
    assert not loop._rp_brake_latched and loop._rp_u is None


# ── ブレーキの領域では 0% へ戻さず待機位置（min）で止める（2026-10-07 段2l-3、段2l-6 で min）──


def test_latched_brake_stops_at_min_and_never_returns_to_zero() -> None:
    """ラッチ後は u ≤ 0 でブレーキ ＝ min ＋ |u|。強い減速でも u は 0 で止まる。"""
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True))
    _, brake_db = _deadbands(loop)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_u = -5.0
    loop._rp_brake_latched = True
    # 狙い 0.3G より強い減速（0.6G）がずっと続く → u は上限 0 で止まる
    _run(loop, p, lambda t: 100.0 - 0.6 * G_TO_KMHS * t, 0.1, 3.0)
    assert loop._phase is _Phase.RAMP_DOWN
    assert loop._rp_u == pytest.approx(0.0)
    a, b = loop._command_openings(p, 3.1)
    assert a == 0.0
    assert b == pytest.approx(brake_db)  # min ちょうど（0% へ戻さない）
    assert b > 0.0


def test_latched_brake_command_is_min_plus_abs_u() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p])
    _, brake_db = _deadbands(loop)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_brake_latched = True
    for u in (0.0, -1.5, -7.0):
        loop._rp_u = u
        a, b = loop._command_openings(p, 0.1)
        assert a == 0.0
        assert b == pytest.approx(brake_db + abs(u))


def test_latching_moment_starts_the_brake_from_min() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True))
    _, brake_db = _deadbands(loop)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_u = 0.5
    assert not loop._rp_brake_latched
    t = 0.1
    while not loop._rp_brake_latched and t < 5.0:  # 減速が足りない → u は負へ → ラッチ
        _cycle(loop, p, 100.0, t)
        t += 0.1
    assert loop._rp_brake_latched
    # ラッチの瞬間に u は 0 へ置き直され、ブレーキ指令は min ちょうど（0% から空走させない）
    assert loop._rp_u == pytest.approx(0.0)
    a, b = loop._command_openings(p, t)
    assert a == 0.0
    assert b == pytest.approx(brake_db)


def test_latched_upper_bound_of_u_is_zero_regardless_of_standby_setting() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    for standby in (True, False):
        loop = _loop([p], config=PatternLoopConfig(pedal_standby=standby))
        loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
        loop._rp_brake_latched = True
        assert loop._ramp_u_limits()[1] == 0.0


def test_latched_governor_does_not_let_u_fall_further() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True))
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_brake_latched = True
    loop._rp_u = -1.0
    loop._brake_request = 10.0
    loop._brake_gov_cap = 1.0  # 削られている（指令 min ＋ 1 より低い）
    # 減速が足りない（G 不足 → u は減る向き＝踏み増し）が、ガバナー中は積まない
    for i in range(1, 6):
        _cycle(loop, p, 100.0, 0.1 * i)
        loop._brake_request, loop._brake_gov_cap = 10.0, 1.0
    assert loop._rp_u == pytest.approx(-1.0)


def test_ramp_up_keeps_the_brake_command_at_zero_after_the_launch_stage() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True))
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = 0.0
    assert loop._command_openings(p, 0.1) == (0.0, 0.0)  # 待機位置の下限は出力側で掛ける


# ── 両ペダルが待機位置でも実測点が貯まる（2026-10-07 段2l-4）─────────────────────


def test_ramp_measurements_accumulate_with_the_opposite_pedal_at_standby() -> None:
    p = _ramp(0.1)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True))
    ff = loop._profile.feedforward_params
    _cycle(loop, p, 20.0, 0.0)
    loop._rp_u = 10.0
    # 実開度: アクセルは min より上、反対側のブレーキは待機位置（min ちょうど。実位置のずれ
    # 0.02% を含む）。2026-10-08 段2l-6: 「min ＋ 幅 以下」なら踏んでいない側として数える
    loop._accel_actual_pct = ff.accel_deadband_pct + 10.0
    loop._brake_actual_pct = ff.brake_deadband_pct + 0.02
    t = 0.1
    while t < 4.0:
        _cycle(loop, p, 20.0 + 0.5 * t, t)
        loop._accel_actual_pct = ff.accel_deadband_pct + 10.0
        loop._brake_actual_pct = ff.brake_deadband_pct + 0.02
        t += 0.1
    assert loop.glimit.has_data(PEDAL_ACCEL)
    assert not loop.glimit.has_data(PEDAL_BRAKE)


def test_ramp_brake_measurements_accumulate_with_accelerator_at_standby() -> None:
    p = _ramp(0.1)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True))
    ff = loop._profile.feedforward_params
    _cycle(loop, p, 50.0, 0.0)
    loop._rp_u = -5.0
    t = 0.1
    while t < 4.0:
        loop._accel_actual_pct = ff.accel_deadband_pct + 0.02
        loop._brake_actual_pct = ff.brake_deadband_pct + 5.0
        _cycle(loop, p, 50.0 - 0.5 * t, t)
        t += 0.1
    assert loop.glimit.has_data(PEDAL_BRAKE)
    assert not loop.glimit.has_data(PEDAL_ACCEL)
