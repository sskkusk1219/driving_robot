"""G ランプ走行（pattern_loop の GRampPattern / RAMP_UP / RAMP_DOWN）のテスト。

走行の状態機械（同期。`_command_openings` と `_advance` を直接呼ぶ）を見る。2026-10-09 段2l-8:
u は「基準車速への追従」（基準車速 v_ref と実車速の偏差 e で、増分形の式 1 つ）で動く。見るもの:
基準に追従して u が動く・感度が無ければ最小刻み 1 個 / 周期・感度があれば Kp ＝ 1 ÷（感度 × T）・
折り返しで u が飛ばない・止められている間は基準が先に行かない・発進でブレーキが単調に離れる・
上端で減速へ・段の終わりは停車・ガバナー・結果の記録。
フェイク（アクチュエータ・CAN・記録）は test_research_pattern_loop.py と同じ書き方で持ち込んだ。
"""

from __future__ import annotations

import dataclasses
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
        # 送り方の記録: ("timed" | "smooth", 目標位置, 時間引数)
        self.calls: list[tuple[str, int, float | None]] = []

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
    launched: bool = True, deadband: float = 0.0, lag_tol: float = 1e9, two_way: bool = False,
) -> PatternLoop:
    """launched=True は RAMP_UP の発進（ブレーキを離す場面）を終えた状態（u ＝ 0・アクセル側）
    から始める。発進そのものを見るテストは launched=False にする。

    2026-10-10 段2l-10: 偏差の不感帯は既定 0（式そのものを見る既存のテストを変えないため）。
    不感帯を見るテストは `deadband` で指定する。アクチュエータの遅れで基準を待たせる許容
    （ア）も既定では効かない大きさにする（`_cycle` は実開度を動かさないため）。見るテストは
    `lag_tol` で指定する。

    2026-10-12 段2l-12: 片側調整（`ramp_one_sided`）が既定で入る。両方向の式そのもの
    （踏み戻し・ブレーキを緩める向きなど）を見るテストは `two_way=True`（ramp_one_sided=False）。"""
    config = dataclasses.replace(
        config or PatternLoopConfig(),
        ramp_dev_deadband_kmh=deadband, ramp_actuator_lag_tol_pct=lag_tol,
    )
    if two_way:
        config = dataclasses.replace(config, ramp_one_sided=False)
    cfg = cfgmod.load_config(CFG_PATH)
    rec = Recorder()
    loop = PatternLoop(
        accel_driver=FakeAxis(), brake_driver=FakeAxis(), can_reader=FakeCAN(0.0),
        profile=build_vehicle_profile(cfg), patterns=patterns, overcurrent_limit_ma=5000.0,
        on_complete=rec.on_complete, on_emergency=rec.on_emergency, on_sample=rec.on_sample,
        config=config, interval_s=0.1,
    )
    if launched:
        loop._rp_u = 0.0
        loop._rp_accel_reached = True
    return loop


# 保持なし・切り替え 0 秒（UP の次の周期で DOWN に入る。区間の遷移そのものを見ないテスト用）
_NO_HOLD = PatternLoopConfig(ramp_turn_time_s=0.0, ramp_hold_time_s=0.0)


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
        u = self.loop._rp_u
        a = -self.coast + (self.ga * u if u >= 0.0 else self.gb * u)
        self.speed = max(0.0, self.speed + a * dt)


def _deadbands(loop: PatternLoop) -> tuple[float, float]:
    ff = loop._profile.feedforward_params
    return ff.accel_deadband_pct, ff.brake_deadband_pct


def _seed_sensitivity(loop: PatternLoop, pedal: str, sens: float, speed: float,
                      *, actual_pct: float | None = None) -> None:
    """そのペダルの感度（開度 1% あたり km/h/s）が sens になる点を貯める（直線）。"""
    adb, bdb = _deadbands(loop)
    db = adb if pedal == PEDAL_ACCEL else bdb
    for k in range(5):
        loop.glimit.add(pedal, speed, db + 2.0 * k + 1.0, sens * (2.0 * k + 1.0))
    if actual_pct is None:
        actual_pct = db + 5.0
    if pedal == PEDAL_ACCEL:
        loop._accel_actual_pct = actual_pct
    else:
        loop._brake_actual_pct = actual_pct


def _min_step(loop: PatternLoop) -> float:
    calib = loop._profile.calibration
    assert calib is not None
    return 100.0 / abs(calib.accel_full_pos - calib.accel_zero_pos)


def _seed_plant_table(loop: PatternLoop, coast: float = 1.0, ga: float = 0.2,
                      gb: float = 0.4) -> None:
    """`_Plant` と同じ車の実測を、全車速帯の上限 G マップ（表）に貯める。"""
    adb, bdb = _deadbands(loop)
    for v in range(5, 150, 10):
        loop.glimit.add_coast(float(v), coast)
        for u in (2.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 40.0):
            loop.glimit.add(PEDAL_ACCEL, float(v), adb + u, -coast + ga * u)
        for u in (1.0, 2.0, 5.0, 10.0, 20.0, 30.0):
            loop.glimit.add(PEDAL_BRAKE, float(v), bdb + u, coast + gb * u)
    loop._accel_actual_pct = adb + 10.0
    loop._brake_actual_pct = bdb + 5.0


def _drive_plant(loop: PatternLoop, p: plmod.GRampPattern, plant: _Plant, t0: float,
                 t1: float, dt: float = 0.1) -> list[tuple[float, float, float]]:
    """車を回しながら走らせ、(時刻, u, 偏差 e) を返す。"""
    rows: list[tuple[float, float, float]] = []
    t = t0
    while t < t1:
        _cycle(loop, p, plant.speed, t)
        plant.step(dt)
        rows.append((t, loop._rp_u, loop._rp_vref - plant.speed))
        t += dt
    return rows


def _up_then_follow(loop: PatternLoop, p: plmod.GRampPattern, t: float, g: float) -> None:
    """1 周期: RAMP_UP の間は狙いどおり（+G）に加速する車（最低 5 km/h）、それ以外は基準に
    0.2 km/h 遅れて付いてくる車。実車速で保持へ入る判定（2026-10-10 段2l-11）を見るテスト用。"""
    speed = max(5.0, g * G_TO_KMHS * t) if loop._phase is _Phase.RAMP_UP else loop._rp_vref - 0.2
    _cycle(loop, p, speed, t)


# ── 始まり方 ────────────────────────────────────────────────────────


def test_ramp_pattern_starts_in_ramp_up_from_zero_reference_with_brake_released_u() -> None:
    loop = _loop([_ramp(), _next_pattern()], launched=False)
    assert loop._phase is _Phase.RAMP_UP
    assert loop._phase.name == "RAMP_UP" and _Phase.RAMP_DOWN.name == "RAMP_DOWN"
    assert PatternKind.G_RAMP.value == "g_ramp"
    adb, bdb = _deadbands(loop)
    hold = min(loop._profile.feedforward_params.stop_brake_opening_pct,
               loop._profile.max_brake_opening)
    # 基準車速は 0 km/h から。u は「今のブレーキ指令開度 − min」の負の値（ブレーキを離す前から）
    assert loop._rp_vref == 0.0
    # 開始開度は今のブレーキ位置（パルスに丸まる）と stop 開度の小さいほう
    assert loop._rp_u == pytest.approx(-(hold - bdb), abs=0.01)
    assert not loop._rp_accel_reached
    u0 = loop._rp_u
    a, b = _cycle(loop, _ramp(), 0.0, 0.0)
    assert a == 0.0 and b == pytest.approx(hold, abs=0.01)
    assert loop._rp_u == u0  # 最初の周期は動かさない


def test_launched_loop_starts_with_u_zero_on_accel_side() -> None:
    p = _ramp()
    loop = _loop([p])
    assert loop._rp_u == 0.0 and loop._rp_accel_reached
    assert _cycle(loop, p, 0.0, 0.0) == (0.0, 0.0)


# ── 基準車速 ────────────────────────────────────────────────────────


def test_reference_rises_at_target_g_from_the_first_cycle_without_rounding() -> None:
    """加速は踏み始めの丸みなしに最初から +G。v_ref ＋＝ g_ref × 35.3 × dt。"""
    p = _ramp(0.2, top_kmh=200.0)
    loop = _loop([p])
    _cycle(loop, p, 0.0, 0.0)
    assert loop._rp_vref == 0.0
    _cycle(loop, p, 0.0, 0.1)
    assert loop._rp_vref == pytest.approx(0.2 * G_TO_KMHS * 0.1)
    for i in range(2, 11):
        _cycle(loop, p, 0.0, 0.1 * i)
    assert loop._rp_vref == pytest.approx(0.2 * G_TO_KMHS * 1.0)


def test_phases_go_up_hold_down_and_reference_stops_at_the_top() -> None:
    """RAMP_UP → RAMP_HOLD → RAMP_DOWN。基準は上端の手前（+G → 0 の間に進むぶん）で保持へ入り、
    上端で止まる。保持の長さ ＝ 切り替え ＋ 保持。その後 -G で下がり 0 km/h で止まる。"""
    g = 0.2
    p = _ramp(g, top_kmh=20.0)
    loop = _loop([p])
    # 感度が無いと基準が cap 以上先へ行かない（2026-10-09）ので、感度を貯めて基準だけを見る
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 5.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 0.4, 5.0)
    cfg = loop._config
    turn, hold = cfg.ramp_turn_time_s, cfg.ramp_hold_time_s
    t = 0.0
    while loop._phase is _Phase.RAMP_UP and t < 30.0:
        _up_then_follow(loop, p, t, g)  # 狙いどおりに加速する車（実車速で保持へ入る。段2l-11）
        t += 0.1
    assert loop._phase is _Phase.RAMP_HOLD  # 実車速が上端の手前に着いたら保持を始める
    lead = g * G_TO_KMHS * turn / 2.0
    step = g * G_TO_KMHS * 0.1
    assert 20.0 - lead - 1e-6 <= loop._rp_vref < 20.0 - lead + step + 1e-6
    assert loop._rp_hold_g0 == pytest.approx(g, abs=0.01)  # 実測の加速度 ÷ G_TO_KMHS
    t_hold0 = loop._rp_hold_started
    vs = []
    while loop._phase is _Phase.RAMP_HOLD and t < t_hold0 + 60.0:
        _up_then_follow(loop, p, t, g)
        vs.append(loop._rp_vref)
        t += 0.1
    assert loop._phase is _Phase.RAMP_DOWN
    assert t - 0.1 - t_hold0 == pytest.approx(turn + hold, abs=0.15)
    # 最後の 1 周期は RAMP_DOWN へ切り替えて基準が実車速へ置き直された値なので、その前を見る
    assert max(vs[:-1]) <= 20.0 + step + 0.1  # 上端で止まる（越えない）
    assert vs[-2] == pytest.approx(20.0, abs=step + 0.5)
    assert vs[-2] == vs[-3]  # g_ref ＝ 0 の間は進まない
    assert loop._rp_turn_g0 == pytest.approx(0.0, abs=1e-9)  # 保持を終えた後は 0
    # 減速: -G で下がり、0 km/h で止まる
    for _ in range(100):
        _cycle(loop, p, 5.0, t)  # 実車速は停車させない（打ち切りの前まで）
        t += 0.1
    assert loop._phase is _Phase.RAMP_DOWN
    assert loop._rp_vref == 0.0


def test_gref_is_continuous_through_up_hold_down() -> None:
    """g_ref が +G → 0（切り替え時間）→ 0（保持）→ -G（切り替え時間）で、段差なく変わる。"""
    g = 0.2
    p = _ramp(g, top_kmh=40.0)
    loop = _loop([p])
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 5.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 0.4, 5.0)
    turn = loop._config.ramp_turn_time_s
    greffs: list[tuple[str, float]] = []
    t = 0.0
    while loop._phase is not _Phase.DRIVE_BRAKE and t < 80.0 and len(greffs) < 700:
        if loop._phase is _Phase.RAMP_DOWN and loop._rp_vref < 15.0:
            break  # 停止の式が効く前まで見る
        _up_then_follow(loop, p, t, g)
        greffs.append((loop._phase.name, loop._ramp_gref(p, t)))
        t += 0.1
    values = [v for _, v in greffs]
    g_min, launch = loop._config.ramp_edge_g, loop._config.ramp_launch_time_s
    # 1 周期の変化は、切り替え（G ÷ 時間）か発進の立ち上げ（(G − g_min) ÷ 時間）の傾き以内
    bound = max(g / turn, (g - g_min) / launch) * 0.1
    assert max(abs(b - a) for a, b in zip(values, values[1:], strict=False)) <= bound + 1e-9
    names = [n for n, _ in greffs]
    assert names.index("RAMP_HOLD") < names.index("RAMP_DOWN")
    hold_values = [v for n, v in greffs if n == "RAMP_HOLD"]
    assert hold_values[0] > 0.0 and hold_values[-1] == pytest.approx(0.0, abs=1e-9)
    assert values[names.index("RAMP_HOLD") - 1] == pytest.approx(g)  # UP の終わりは +G
    # 保持の始まりは実測の加速度 ÷ G_TO_KMHS（狙いどおりに加速する車なら +G とほぼ同じ）
    assert hold_values[0] == pytest.approx(g, abs=0.01)
    assert min(values) == pytest.approx(-g, abs=1e-9)  # 0 → -G まで下りきる


def test_zero_turn_and_zero_hold_turns_back_without_holding() -> None:
    g = 0.2
    p = _ramp(g, top_kmh=20.0)
    loop = _loop([p], config=_NO_HOLD)
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 5.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 0.4, 5.0)
    t = 0.0
    names = []
    for _ in range(120):
        speed = max(5.0, g * G_TO_KMHS * t) if loop._phase is _Phase.RAMP_UP else 5.0
        _cycle(loop, p, speed, t)
        names.append(loop._phase.name)
        t += 0.1
    assert names.index("RAMP_HOLD") < names.index("RAMP_DOWN")
    assert names.count("RAMP_HOLD") == 1  # 1 周期だけ通って折り返す
    assert loop._phase is _Phase.RAMP_DOWN and loop._rp_vref == 0.0


def test_gref_changes_linearly_over_turn_time() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_vref = 30.0  # 停止の式（基準車速が低いとき）に掛からない高さ
    assert loop._ramp_gref(p, 0.0) == -0.1  # 折り返しを始めていない RAMP_DOWN は −G
    loop._rp_turn_started = 10.0
    assert loop._ramp_gref(p, 10.0) == pytest.approx(0.1)
    assert loop._ramp_gref(p, 10.0 + 0.5 * loop._config.ramp_turn_time_s) == pytest.approx(0.0)
    assert loop._ramp_gref(p, 10.0 + loop._config.ramp_turn_time_s) == pytest.approx(-0.1)
    assert loop._ramp_gref(p, 99.0) == pytest.approx(-0.1)


def test_actual_speed_reaching_top_starts_the_hold_and_resets_reference() -> None:
    """安全網: 実車速が上端の頭打ちに着いたら、その時点で保持へ（基準は実車速へ置き直す）。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _run(loop, p, lambda t: 30.0, 0.0, 0.5)
    vref_before = loop._rp_vref
    assert 0.0 < vref_before < 60.0
    _cycle(loop, p, 60.5, 0.6)
    assert loop._phase is _Phase.RAMP_HOLD
    assert loop._rp_hold_started == pytest.approx(0.6)
    # 切り替えた周期に基準は実車速へ置き直す（2026-10-10 段2l-11）
    assert loop._rp_vref == pytest.approx(60.5) and loop._rp_e_prev == 0.0


# ── (a) 基準車速に追従して u が動く ─────────────────────────────────────


def test_u_rises_to_follow_reference_when_speed_does_not_move() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    loop._rp_vref = 10.0
    _run(loop, p, lambda t: 10.0, 0.0, 2.0)  # 車速が動かない（基準だけ上がる → e ＞ 0）
    assert loop._rp_u > 0.0


def test_u_tracks_reference_with_measured_sensitivity() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p])
    _seed_plant_table(loop)
    plant = _Plant(loop, 0.0)
    rows = _drive_plant(loop, p, plant, 0.0, 14.0)
    # 表は「変化」だけを足す（水準そのものは PI が作る）ので、立ち上がりは基準から遅れ、
    # その遅れを PI が詰めていく（偏差は縮む向き）。約 22%（(3.53 + 惰行 1.0) ÷ 0.2）へ向かう
    es = [e for _, _, e in rows]
    assert max(es) < 8.0 and es[-1] < max(es) * 0.5
    assert rows[-1][1] > 15.0
    assert plant.speed > 0.7 * loop._rp_vref


def test_u_tracks_reference_through_the_turn_and_down_to_standstill() -> None:
    p = _ramp(0.1, top_kmh=30.0)
    loop = _loop([p, _next_pattern()])
    _seed_plant_table(loop)
    plant = _Plant(loop, 0.0)
    rows = _drive_plant(loop, p, plant, 0.0, 60.0)
    assert loop._pattern_idx == 1  # 停車して次のパターンへ（段の終わりは停車）
    assert len(loop.ramp_results) == 1
    r = loop.ramp_results[0]
    assert r.max_speed_kmh > 20.0
    assert r.accel_g_median == pytest.approx(0.1, abs=0.02)
    assert r.decel_g_median == pytest.approx(0.1, abs=0.03)
    assert max(abs(e) for _, _, e in rows) < 8.0


# ── (b) 感度なしでは 1 周期に最小刻み 1 個だけ ────────────────────────────


def test_without_sensitivity_u_moves_by_one_actuator_step_per_cycle() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    step = _min_step(loop)
    assert step == pytest.approx(100.0 / 9500.0)
    _cycle(loop, p, 0.0, 0.0)
    us = []
    for i in range(1, 8):
        _cycle(loop, p, 0.0, 0.1 * i)  # 実車速 0・基準は上がる → e ＞ 0
        us.append(loop._rp_u)
    diffs = [b - a for a, b in zip([0.0, *us], us, strict=False)]
    assert all(d == pytest.approx(step) for d in diffs)
    assert loop.ramp_status() == plmod.RampStatus(loop._rp_vref, 0.0, 0.0, 0.0, 0.0)


def test_without_sensitivity_u_moves_toward_the_sign_of_e_and_zero_error_does_not_move() -> None:
    p = _ramp(0.1, top_kmh=100.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)  # 上限 0 ではまだ無い（印が立つ前）
    step = _min_step(loop)
    _cycle(loop, p, 50.0, 0.0)
    loop._rp_vref = 50.0
    loop._rp_e_prev = 0.0
    _cycle(loop, p, 50.0, 0.1)  # 基準 ≒ 実車速（−G で少し下がる）→ e ≒ −0.35 ＜ 0
    assert loop._rp_u == pytest.approx(-step)
    loop._rp_vref = 50.0
    before = loop._rp_u
    # e ＝ 0 ちょうどなら動かさない（−G で下がるぶんを実車速に合わせる）
    loop._rp_vref = 50.0 + 0.1 * G_TO_KMHS * 0.1
    _cycle(loop, p, 50.0, 0.2)
    assert loop._rp_u == pytest.approx(before)


def test_min_step_comes_from_the_calibration_stroke_and_has_a_fallback() -> None:
    loop = _loop([_ramp()])
    calib = loop._profile.calibration
    assert calib is not None
    assert loop._ramp_min_step_pct(False) == pytest.approx(
        100.0 / abs(calib.accel_full_pos - calib.accel_zero_pos)
    )
    assert loop._ramp_min_step_pct(True) == pytest.approx(
        100.0 / abs(calib.brake_full_pos - calib.brake_zero_pos)
    )
    loop._profile = dataclasses.replace(loop._profile, calibration=None)
    assert loop._ramp_min_step_pct(False) == plmod.RAMP_MIN_STEP_FALLBACK_PCT
    assert plmod.RAMP_MIN_STEP_FALLBACK_PCT == pytest.approx(100.0 / 9500.0)


# ── (c) 感度ありで Kp = 1 ÷（感度 × T）に切り替わる ──────────────────────────


def test_gain_switches_to_kp_from_measured_sensitivity_and_status_shows_it() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    cfg = loop._config
    t_settle = cfg.ramp_settle_time_s
    sens = 0.2  # km/h/s per %（アクセル）
    _cycle(loop, p, 0.0, 0.0)
    # 感度なし: 1 周期に最小刻み 1 個
    _cycle(loop, p, 0.0, 0.1)
    assert loop._rp_u == pytest.approx(_min_step(loop))
    st = loop.ramp_status()
    assert st is not None and (st.accel_kp, st.accel_ki) == (0.0, 0.0)
    # 感度が貯まる → 次の周期から Kp・Ki
    _seed_sensitivity(loop, PEDAL_ACCEL, sens, 0.0)
    u0, e_prev = loop._rp_u, loop._rp_e_prev
    assert e_prev is not None
    _cycle(loop, p, 0.0, 0.2)
    kp = 1.0 / (sens * t_settle)
    ki = kp / (plmod.RAMP_TI_RATIO * t_settle)
    e = loop._rp_vref - 0.0
    assert loop._rp_u - u0 == pytest.approx(kp * (e - e_prev) + ki * e * 0.1)
    st = loop.ramp_status()
    assert st is not None
    assert st.accel_kp == pytest.approx(kp) and st.accel_ki == pytest.approx(ki)
    assert st.accel_ki == pytest.approx(st.accel_kp / (4.0 * t_settle))
    assert (st.brake_kp, st.brake_ki) == (0.0, 0.0)  # ブレーキの感度はまだ無い


def test_brake_gain_uses_brake_sensitivity_and_is_separate_from_accelerator() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 60.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 1.0, 60.0)
    loop._rp_u = -3.0
    loop._rp_brake_latched = True
    loop._rp_vref = 60.0
    _cycle(loop, p, 60.0, 0.0)
    _cycle(loop, p, 60.0, 0.1)
    st = loop.ramp_status()
    t = loop._config.ramp_settle_time_s
    assert st is not None
    assert st.brake_kp == pytest.approx(1.0 / (1.0 * t))
    assert st.brake_ki == pytest.approx(st.brake_kp / (4.0 * t))


def test_increasing_e_increases_u_for_both_pedals() -> None:
    """e が正なら u を増やす（アクセルは踏む・ブレーキは緩める）。同じ式。"""
    for u0, pedal in ((5.0, PEDAL_ACCEL), (-5.0, PEDAL_BRAKE)):
        p = _ramp(0.1, top_kmh=100.0)
        loop = _loop([p])
        _seed_sensitivity(loop, pedal, 0.5, 20.0)
        _cycle(loop, p, 20.0, 0.0)
        loop._rp_u = u0
        loop._rp_vref = 20.0
        loop._rp_e_prev = 0.0
        loop._rp_accel_reached = u0 >= 0.0
        _cycle(loop, p, 19.0, 0.1)  # 実車速が基準より遅れた → e ＞ 0
        assert loop._rp_u > u0


# ── (d) 折り返しで u が飛ばない ───────────────────────────────────────────


def test_u_does_not_jump_through_hold_and_turn_with_a_table() -> None:
    """表（全車速帯）があっても、保持の頭・減速の頭・帯の境目で u は 1 周期に飛ばない
    （段2l-10: 表の項は無く、寄せは決めた時間で等速に足す）。"""
    p = _ramp(0.1, top_kmh=40.0)
    loop = _loop([p, _next_pattern()])
    _seed_plant_table(loop)
    plant = _Plant(loop, 0.0)
    rows = _drive_plant(loop, p, plant, 0.0, 70.0)
    assert loop._pattern_idx == 1  # 停車して次のパターンへ
    # 発進の寄せ（最初の ramp_turn_time_s ＝ 2 秒。段2l-11）は除く
    us = [u for t, u, _ in rows if t > 2.1]
    steps = [abs(b - a) for a, b in zip(us, us[1:], strict=False)]
    # 段2l-11: 寄せが 2 秒になり車の軌跡が変わった。減速の途中の連続した踏み増しは 1.1 %/周期
    # 程度まで出る（飛びではない）ので上限を 1.5 にする
    assert max(steps) < 1.5
    assert max(us) > 10.0 and min(us) < 0.0  # アクセルからブレーキ側まで動いた


def test_switch_down_does_not_overwrite_u_with_an_estimate() -> None:
    p = _ramp(0.25, top_kmh=60.0)
    loop = _loop([p])
    loop.glimit.add(PEDAL_BRAKE, 55.0, 12.0, 4.0)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 20.0, 9.0)
    _cycle(loop, p, 0.0, 0.0)
    loop._rp_u = 30.0
    loop._ramp_switch_down(5.0)
    assert loop._phase is _Phase.RAMP_DOWN
    assert loop._rp_u == 30.0 and not loop._rp_brake_latched
    loop2 = _loop([_ramp(0.1)])
    loop2._rp_u = -2.0
    loop2._ramp_switch_down(5.0)
    assert loop2._rp_u == -2.0 and loop2._rp_brake_latched  # ブレーキ側なら印が立つ


def _seed_accel_points(loop: PatternLoop) -> None:
    loop.glimit.add(PEDAL_ACCEL, 2.0, 6.0, 1.0)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 10.0, 3.0)


def test_ramp_up_start_does_not_jump_to_the_table_estimate() -> None:
    p = _ramp(0.08)
    loop = _loop([p])
    _seed_accel_points(loop)
    prev = loop._rp_u
    for i in range(2):
        _cycle(loop, p, 0.0, 0.1 * i)
        assert loop._rp_u - prev < 2.0  # 見積もりの開度（約 7%）へ一気に代入しない
        prev = loop._rp_u


def test_moving_to_accel_blends_u_toward_table_over_blend_time_without_jumping() -> None:
    """アクセルに移ると `_ramp_launch_blend_time_s`（段の G で決まる。0.08G は 0.5 秒。段2l-11）
    かけて、段の G が出る表の開度へ寄る。寄せの間は基準 ＝ 実車速で偏差の項が 0 なので
    （2026-10-12 段2l-13）、1 周期の増分は gap×dt/時間、寄せが終わった時点の u は gap。"""
    p = _ramp(0.08)
    loop = _loop([p])
    _seed_accel_points(loop)
    gap = loop._ramp_estimate_u(0.0, 0.08)
    assert gap is not None and gap > 3.0
    dt = 0.1
    blend_per_cycle = gap * dt / loop._ramp_launch_blend_time_s(p)
    prev = loop._rp_u
    i = 0
    _cycle(loop, p, 0.0, 0.0)
    while loop._rp_blend_left != 0.0:
        i += 1
        _cycle(loop, p, 0.0, dt * i)
        assert loop._rp_u - prev <= blend_per_cycle * 1.05 + 1e-9
        prev = loop._rp_u
        assert i < 20
    assert i * dt == pytest.approx(loop._ramp_launch_blend_time_s(p))
    assert loop._rp_u == pytest.approx(gap, rel=0.05)  # 寄せが終わった時点で表の開度ぶん
    assert loop._rp_u <= gap + 1e-9  # 足した合計が gap を超えない


def test_no_table_point_means_no_blend() -> None:
    p = _ramp(0.08)
    base = _loop([p])
    base._rp_blend_decided = True
    loop = _loop([p])
    for i in range(8):
        _cycle(base, p, 0.0, 0.1 * i)
        _cycle(loop, p, 0.0, 0.1 * i)
        assert loop._rp_u == pytest.approx(base._rp_u)
    assert loop._rp_blend_decided and loop._rp_blend_gap == 0.0
    # 途中で表が引けるようになっても寄せ直さない
    _seed_accel_points(loop)
    _seed_accel_points(base)
    for i in range(8, 14):
        _cycle(base, p, 0.0, 0.1 * i)
        _cycle(loop, p, 0.0, 0.1 * i)
    assert loop._rp_blend_left == 0.0


def test_table_values_changing_every_cycle_do_not_move_u(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """表の返す値が周期ごとに変わっても、寄せを決める周期以外は開度が動かない（偏差の項は
    e ＝ 0 にして切り離す。発進の寄せは決め済みにして、表を引く周期を作らない）。"""
    p = _ramp(0.1, top_kmh=200.0)
    # 発進の立ち上げ（0 → +G）があると基準の進みが変わり e ＝ 0 にならないので、時間 0（丸めない）
    loop = _loop([p], config=PatternLoopConfig(ramp_launch_time_s=0.0))
    loop._rp_blend_decided = True
    calls: list[int] = []
    seq = iter([None, 5.0, 7.5, None, 4.0, 6.0, 30.0, -8.0])

    def spy(speed: float, g: float) -> float | None:
        calls.append(1)
        return next(seq)

    monkeypatch.setattr(loop, "_ramp_estimate_u", spy)
    dt = 0.125  # 2 進数で正確な周期（e ＝ 0 ちょうどにするため）
    for i in range(8):
        speed = loop._rp_vref + (0.1 * G_TO_KMHS * dt if i else 0.0)
        _cycle(loop, p, speed, dt * i)  # e ＝ 0（偏差の項は 0）
        assert loop._rp_u == 0.0
    assert calls == []  # 毎周期の表引きはしない


# ── 動き出しの置き直し・感度なしの先行頭打ち（2026-10-09） ────────────────────


def test_reference_is_reanchored_once_when_speed_first_exceeds_stop_speed() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p], launched=False)
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 2.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 0.4, 2.0)
    t = 0.0
    for _ in range(10):  # ブレーキを離している間（車は動かない）。基準は進み続ける
        _cycle(loop, p, 0.0, t)
        t += 0.1
    assert loop._rp_vref == pytest.approx(0.1 * G_TO_KMHS * 0.9)
    assert not loop._rp_vref_anchored
    u_before = loop._rp_u
    speed = plmod.STOP_SPEED_KMH + 0.5
    _cycle(loop, p, speed, t)  # 初めて停車判定の速度を超えた周期
    t += 0.1
    assert loop._rp_vref_anchored
    assert loop._rp_vref == pytest.approx(speed)
    assert loop._rp_e_prev == pytest.approx(0.0)
    assert loop._rp_u == pytest.approx(u_before)  # この周期は開度を動かさない
    # 以後は置き直さない: 基準は発進の g_ref（置き直しの周期から 1 秒で g_min → +G。段2l-9）で進み、
    # 実車速が動かなければ偏差が付く。経過 k × 0.1 秒の g_ref ＝ g_min ＋ (G − g_min) × 0.1k
    g, g_min = 0.1, loop._config.ramp_edge_g
    _cycle(loop, p, speed, t)
    assert loop._rp_vref == pytest.approx(speed + (g_min + (g - g_min) * 0.1) * G_TO_KMHS * 0.1)
    for _ in range(5):
        t += 0.1
        _cycle(loop, p, speed, t)
    g_sum = sum(g_min + (g - g_min) * 0.1 * k for k in range(1, 7))
    assert loop._rp_vref == pytest.approx(speed + g_sum * G_TO_KMHS * 0.1)


def test_reference_is_not_reanchored_in_ramp_down_without_having_moved() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p], launched=False)
    _cycle(loop, p, 0.0, 0.0)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.1)
    loop._rp_vref = 3.0
    _cycle(loop, p, 0.0, 0.2)
    _cycle(loop, p, plmod.STOP_SPEED_KMH + 1.0, 0.3)
    assert not loop._rp_vref_anchored
    assert loop._rp_vref != pytest.approx(plmod.STOP_SPEED_KMH + 1.0)


def test_reference_is_anchored_in_the_first_cycle_when_already_moving() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p])
    _cycle(loop, p, 12.0, 0.0)
    assert loop._rp_vref_anchored
    assert loop._rp_vref == pytest.approx(12.0)


def test_without_sensitivity_reference_stays_within_cap_of_actual_speed() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p])
    cap = 0.1 * G_TO_KMHS * loop._config.ramp_settle_time_s
    _cycle(loop, p, 10.0, 0.0)
    for i in range(1, 100):
        _cycle(loop, p, 10.0, 0.1 * i)  # 車速が動かない（弱い踏みでは効かない）
        assert abs(loop._rp_vref - 10.0) <= cap + 1e-9
    assert loop._rp_vref == pytest.approx(10.0 + cap)
    assert loop._rp_e_prev == pytest.approx(cap)


def test_without_sensitivity_cap_applies_to_ramp_down_brake_side_too() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p])
    cap = 0.1 * G_TO_KMHS * loop._config.ramp_settle_time_s
    _cycle(loop, p, 50.0, 0.0)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.1)
    loop._rp_vref = 20.0  # 実車速よりずっと下（e ＜ 0）
    _cycle(loop, p, 50.0, 0.2)
    assert loop._rp_vref == pytest.approx(50.0 - cap)


def test_with_sensitivity_the_cap_does_not_apply() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p])
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 10.0)
    cap = 0.1 * G_TO_KMHS * loop._config.ramp_settle_time_s
    _cycle(loop, p, 10.0, 0.0)
    for i in range(1, 40):
        _cycle(loop, p, 10.0, 0.1 * i)
    assert loop._rp_vref - 10.0 > cap  # 感度があれば基準は cap を超えて先へ行く


# ── (e) 止められている間は基準が先に行かない ──────────────────────────────────


def test_reference_does_not_run_ahead_while_u_is_at_the_upper_limit() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p])
    _, hi = loop._ramp_u_limits()
    loop._rp_vref = 10.0
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = hi
    es = []
    for i in range(1, 60):
        _cycle(loop, p, 10.0, 0.1 * i)  # 車速が動かない・基準だけ上がる
        es.append(loop._rp_vref - 10.0)
    assert loop._rp_u == pytest.approx(hi)
    assert max(es) < 1e-9  # 頭打ちの間、偏差は広がらない（基準を実車速側へ戻す）


def test_reference_does_not_run_ahead_while_governor_blocks_pressing() -> None:
    p = _ramp(0.35)
    loop = _loop([p])
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = 10.0
    loop._accel_request = 14.0
    loop._accel_gov_cap = 5.0  # 削られている
    loop._current_accel_opening = 5.0
    es = []
    for i in range(1, 15):
        _cycle(loop, p, 10.0, 0.1 * i)
        es.append(loop._rp_vref - 10.0)
    assert loop._rp_u <= 10.0 + 1e-9  # 踏み増しを積まない
    assert max(es) < 0.35 * G_TO_KMHS * 0.1 + 1e-9  # 偏差は最初の周期の分から広がらない


def test_reference_waits_for_creep_faster_than_target_in_ramp_up() -> None:
    """クリープで狙いより速く加速する区間: u は下限 0（両ペダル min）のまま、基準は遅れない。"""
    p = _ramp(0.02, top_kmh=100.0)
    loop = _loop([p])
    _cycle(loop, p, 0.0, 0.0)
    speed = 0.0
    es = []
    for i in range(1, 80):
        speed += 1.4 * 0.1  # クリープで 1.4 km/h/s（狙い 0.7 km/h/s より速い）
        _cycle(loop, p, speed, 0.1 * i)
        es.append(loop._rp_vref - speed)
    assert loop._rp_u == 0.0
    assert loop._command_openings(p, 8.0) == (0.0, 0.0)
    assert min(es) > -0.3  # 偏差が広がり続けない（引き戻している）


# ── 上端で減速へ ─────────────────────────────────────────────────────


def test_reaching_top_speed_switches_to_hold_then_ramp_down() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _run(loop, p, lambda t: 30.0, 0.0, 0.5)
    assert loop._phase is _Phase.RAMP_UP
    _cycle(loop, p, 60.5, 0.6)
    assert loop._phase is _Phase.RAMP_HOLD
    assert loop._pattern_idx == 0
    cfg = loop._config
    # 切り替え ＋ 保持が経つまで保持のまま（実車速が上端を超え続けない限り）
    _cycle(loop, p, 30.0, 0.7)
    _cycle(loop, p, 30.0, 0.6 + cfg.ramp_turn_time_s + cfg.ramp_hold_time_s - 0.1)
    assert loop._phase is _Phase.RAMP_HOLD
    _cycle(loop, p, 30.0, 0.6 + cfg.ramp_turn_time_s + cfg.ramp_hold_time_s)
    assert loop._phase is _Phase.RAMP_DOWN


def test_hold_goes_down_when_speed_keeps_exceeding_the_top_cap() -> None:
    """保持中に実車速が保持の境目を skip_consecutive_required 周期続けて超えたら減速へ。
    1 周期だけなら保持のまま。2026-10-10 段2l-11: 境目は 上端 ＋ G × ramp_turn_time_s ÷ 2
    （ここでは 60 ＋ 0.1 × 35.3 × 1 ≒ 63.5 km/h）。上端を少し越えただけでは打ち切らない。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _cycle(loop, p, 60.5, 0.0)
    assert loop._phase is _Phase.RAMP_HOLD
    for k in range(loop._skip_required + 2):  # 上端を少し越えたまま続いても保持
        _cycle(loop, p, 60.8, 0.05 + 0.05 * k)
        assert loop._phase is _Phase.RAMP_HOLD
    t0 = 0.05 + 0.05 * (loop._skip_required + 2)
    _cycle(loop, p, 64.0, t0)  # 1 周期だけ超えた
    assert loop._phase is _Phase.RAMP_HOLD
    _cycle(loop, p, 30.0, t0 + 0.05)  # 戻れば数え直し
    for k in range(loop._skip_required):
        assert loop._phase is _Phase.RAMP_HOLD
        _cycle(loop, p, 64.0, t0 + 0.1 + 0.05 * k)
    assert loop._phase is _Phase.RAMP_DOWN


def test_ramp_up_times_out_into_hold_when_top_is_not_reached() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    limit = 60.0 / (0.1 * G_TO_KMHS)  # 2026-10-08 段2l-6: 見込み秒数は 0 km/h までで計算
    timeout = limit * plmod.RAMP_LEG_TIMEOUT_FACTOR + plmod.RAMP_LEG_TIMEOUT_MARGIN_S
    _cycle(loop, p, 20.0, 0.0)
    _cycle(loop, p, 20.0, timeout - 1.0)
    assert loop._phase is _Phase.RAMP_UP
    _cycle(loop, p, 20.0, timeout + 0.1)
    assert loop._phase is _Phase.RAMP_HOLD


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
    loop = _loop([p], two_way=True)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_u = 0.0
    loop._rp_vref = 100.0
    _seed_plant_table(loop)
    plant = _Plant(loop, 100.0)
    us: list[float] = []
    t = 0.0
    while t < 40.0:
        _cycle(loop, p, plant.speed, t)
        plant.step()
        us.append(loop._rp_u)
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
    loop._rp_u = 5.0  # 加速側から切り替わった想定
    loop._rp_vref = 100.0
    _seed_plant_table(loop)
    plant = _Plant(loop, 100.0)
    us: list[float] = []
    t = 0.0
    while t < 8.0:
        _cycle(loop, p, plant.speed, t)
        plant.step()
        us.append(loop._rp_u)
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
    _run(loop, p, lambda t: 10.0, 0.1, 1.5)  # 基準 ＞ 実車速（本来は u を増やす）
    assert loop._rp_u <= u0 + 1e-9


# ── 停車復帰と次のパターン ───────────────────────────────────────────────


def test_ramp_down_continues_to_standstill_then_hands_over_to_the_next_pattern() -> None:
    """2026-10-08 段2l-6: 5 km/h（coast_down_stop_speed_kmh）では終わらず、停車まで続ける。
    停車を見たらすぐ _finish_pattern（停車していれば次のパターンへ）。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p, _next_pattern()], config=_NO_HOLD)
    _cycle(loop, p, 60.5, 0.0)  # 上端 → 保持（0 秒）
    _cycle(loop, p, 60.5, 0.1)  # → 減速
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
    loop = _loop([p], config=_NO_HOLD)
    _cycle(loop, p, 60.5, 0.0)
    _cycle(loop, p, 60.5, 0.05)
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
    # 最高速超えが続く間: まず上端扱いで RAMP_HOLD へ、そこで連続して超えれば停車復帰（回復）へ
    for i in range(2 * loop._config.skip_consecutive_required):
        _cycle(loop, p, over, 0.1 * i)
    assert loop._phase is _Phase.DRIVE_BRAKE and loop._overspeed_recovery


# ── 結果の記録 ──────────────────────────────────────────────────────────


def test_result_records_durations_median_g_hold_seconds_and_top_speed() -> None:
    """加速秒 ＝ RAMP_UP、保持秒 ＝ RAMP_HOLD、減速秒 ＝ RAMP_DOWN。G 中央値は UP・DOWN だけ
    （保持の G ≒ 0 は入れない。保持を長くして、入れていれば中央値が 0 に寄る状況にする）。"""
    p = _ramp(0.1, top_kmh=60.0)
    hold_s = 60.0
    loop = _loop([p], config=PatternLoopConfig(ramp_hold_time_s=hold_s))
    turn = loop._config.ramp_turn_time_s
    g = 0.1 * G_TO_KMHS
    t = 0.0
    speed = 0.0
    while loop._phase is _Phase.RAMP_UP:  # 0.1G で加速（基準が上端の手前に着くと保持へ）
        _cycle(loop, p, speed, t)
        speed += g * 0.1
        t += 0.1
    up_end = t
    assert loop._phase is _Phase.RAMP_HOLD
    speed = 55.0
    while loop._phase is _Phase.RAMP_HOLD:  # 保持（車速は一定 ＝ 実測 G ≒ 0）
        _cycle(loop, p, speed, t)
        t += 0.1
    hold_end = t
    while speed > 0.0:  # 0.1G で停車まで減速（段2l-6: 5 km/h ではなく停車まで）
        _cycle(loop, p, speed, t)
        speed = max(0.0, speed - g * 0.1)
        t += 0.1
    _cycle(loop, p, speed, t)
    assert len(loop.ramp_results) == 1
    r = loop.ramp_results[0]
    assert r.target_g == 0.1 and r.top_kmh == 60.0
    assert r.up_s == pytest.approx(up_end, abs=0.3)
    assert r.hold_s == pytest.approx(turn + hold_s, abs=0.3)
    assert r.hold_s == pytest.approx(hold_end - up_end, abs=0.3)
    assert r.down_s == pytest.approx(t - hold_end, abs=0.3)
    assert r.accel_g_median == pytest.approx(0.1, abs=0.01)
    assert r.decel_g_median == pytest.approx(0.1, abs=0.01)
    assert r.max_speed_kmh >= 55.0


# ── 上限 G の予測マップとの連携 ─────────────────────────────────────────


def test_estimate_u_for_accel_target_comes_from_glimit() -> None:
    p = _ramp(0.08)
    loop = _loop([p])
    adb, _ = _deadbands(loop)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 6.0, 1.0)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 10.0, 3.0)
    expected_opening = 6.0 + (0.08 * G_TO_KMHS - 1.0) / 2.0 * 4.0
    assert loop._ramp_estimate_u(2.0, 0.08) == pytest.approx(expected_opening - adb)


def test_estimate_u_for_brake_target_comes_from_glimit() -> None:
    p = _ramp(0.25, top_kmh=60.0)
    loop = _loop([p])
    _, bdb = _deadbands(loop)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 12.0, 4.0)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 20.0, 9.0)
    expected_opening = 12.0 + (0.25 * G_TO_KMHS - 4.0) / 5.0 * 8.0
    assert loop._ramp_estimate_u(55.0, -0.25) == pytest.approx(-(expected_opening - bdb))


def test_estimate_u_of_weak_decel_uses_accel_side_when_below_coast() -> None:
    p = _ramp(0.02, top_kmh=60.0)
    loop = _loop([p])
    adb, _ = _deadbands(loop)
    loop.glimit.add_coast(55.0, 1.0)  # 惰行の減速 1.0 km/h/s
    loop.glimit.add(PEDAL_ACCEL, 55.0, adb + 6.0, 1.0)  # この開度で +1.0 km/h/s
    # 狙い −0.02G ＝ −0.706 km/h/s は惰行（−1.0）と +1.0 の間 → アクセル側（u ＞ 0）
    est = loop._ramp_estimate_u(55.0, -0.02)
    assert est is not None and 0.0 < est < 6.0


def test_ramp_measurements_feed_the_glimit_map() -> None:
    p = _ramp(0.1)
    loop = _loop([p])
    _cycle(loop, p, 20.0, 0.0)
    loop._rp_u = 10.0
    loop._rp_vref = 20.0  # 基準が実車速に近い間は u が下がらない
    _run(loop, p, lambda t: 20.0 + 0.5 * t, 0.1, 4.0)  # 実開度（指令で代用）で加速中
    assert loop.glimit.has_data(PEDAL_ACCEL)
    assert not loop.glimit.has_data(PEDAL_BRAKE)


# ── 見積もりは外挿しない（2026-10-08 段2l-6）──────────────────────────────


def test_estimate_does_not_extrapolate_beyond_the_strongest_point() -> None:
    """狙いが貯めた点のどれより強いときは、いちばん効いた点の開度を返す（外挿しない）。"""
    p = _ramp(0.25)
    loop = _loop([p])
    adb, _ = _deadbands(loop)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 6.0, 1.0)
    loop.glimit.add(PEDAL_ACCEL, 2.0, 10.0, 3.0)  # いちばん効いた点: 10% で 3.0 km/h/s
    assert 0.25 * G_TO_KMHS > 3.0  # 狙いは貯めた点より強い
    assert loop._ramp_estimate_u(2.0, 0.25) == pytest.approx(10.0 - adb)


def test_brake_estimate_does_not_extrapolate_either() -> None:
    p = _ramp(0.35, top_kmh=60.0)
    loop = _loop([p])
    _, bdb = _deadbands(loop)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 12.0, 4.0)
    loop.glimit.add(PEDAL_BRAKE, 55.0, 20.0, 9.0)  # いちばん効いた点: 20% で 9.0 km/h/s
    assert 0.35 * G_TO_KMHS > 9.0
    assert loop._ramp_estimate_u(55.0, -0.35) == pytest.approx(-(20.0 - bdb))


# ── (f) 発進: ブレーキが単調に離れ、踏み直さない ──────────────────────────────


def _launch_loop(target_g: float = 0.1, **cfg_kw: float) -> tuple[PatternLoop, plmod.GRampPattern]:
    p = _ramp(target_g, top_kmh=200.0)
    config = PatternLoopConfig(pedal_standby=True, **cfg_kw)
    return _loop([p], config=config, launched=False), p


def test_launch_starts_at_stop_confirm_opening_with_accel_at_standby() -> None:
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    _, bdb = _deadbands(loop)
    assert loop._rp_u == pytest.approx(-(14.0 - bdb))
    a, b = _cycle(loop, p, 0.0, 0.0)
    assert (a, b) == (0.0, pytest.approx(14.0))  # アクセル 0（出力側 min）・ブレーキ=止まった開度


def test_launch_without_stop_confirm_starts_from_the_stop_hold_opening() -> None:
    loop, p = _launch_loop()
    hold = loop._profile.feedforward_params.stop_brake_opening_pct
    _, b = _cycle(loop, p, 0.0, 0.0)
    assert b == pytest.approx(min(hold, loop._profile.max_brake_opening), abs=0.01)


def test_launch_does_not_press_again_when_the_brake_is_already_below_stop_confirm() -> None:
    """前の減速が止まった開度より低いブレーキで停車したなら、そこから始める（踏み直さない）。"""
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    calib = loop._profile.calibration
    assert calib is not None
    loop._brake_pos_cmd = int(10.0 / 100.0 * calib.brake_full_pos)  # 実際の位置は約 10%
    loop._enter_phase(_Phase.RAMP_UP, 5.0)
    _, b = _cycle(loop, p, 0.0, 5.0)
    assert b == pytest.approx(10.0, abs=0.02)


def test_launch_without_sensitivity_releases_brake_one_step_per_cycle() -> None:
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    step = _min_step(loop)
    _cycle(loop, p, 0.0, 0.0)
    u0 = loop._rp_u
    for i in range(1, 6):
        _cycle(loop, p, 0.0, 0.1 * i)
    assert loop._rp_u - u0 == pytest.approx(5 * step)  # 感度なし（ブレーキ）→ 最小刻み
    assert loop._rp_u < 0.0  # まだブレーキ側


def test_launch_brake_releases_monotonically_then_hands_over_to_accel() -> None:
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    _, bdb = _deadbands(loop)
    _seed_sensitivity(loop, PEDAL_BRAKE, 1.0, 0.0, actual_pct=bdb + 5.0)
    brakes: list[float] = []
    us: list[float] = []
    t = 0.0
    for _ in range(200):  # 車は動かさない（実車速 0・基準は上がる）→ ブレーキが離れていく
        _, b = _cycle(loop, p, 0.0, t)
        brakes.append(b)
        us.append(loop._rp_u)
        loop._brake_actual_pct = max(b, bdb) if b > 0 else bdb
        t += 0.1
        if loop._rp_accel_reached:
            break
    assert loop._rp_accel_reached and loop._rp_u >= 0.0
    # 単調に離れる（ブレーキ指令が上がる周期・u が下がる周期が 1 つも無い）
    nonzero = [b for b in brakes if b > 0.0]
    assert all(b2 <= b1 + 1e-9 for b1, b2 in zip(nonzero, nonzero[1:], strict=False))
    assert all(u2 >= u1 - 1e-9 for u1, u2 in zip(us, us[1:], strict=False))
    assert brakes[0] == pytest.approx(14.0)
    # アクセル側に入ったあとブレーキへ戻らない
    for i in range(40):
        t += 0.1
        _, b = _cycle(loop, p, 0.0, t)
        assert b == 0.0  # u ≥ 0 なのでブレーキ指令は出ない
    assert loop._rp_u > 0.0


def test_launch_never_presses_the_brake_again_even_if_speed_overshoots_reference() -> None:
    loop, p = _launch_loop(0.1, stop_confirm_pct=14.0)
    _, bdb = _deadbands(loop)
    _seed_sensitivity(loop, PEDAL_BRAKE, 1.0, 0.0, actual_pct=bdb + 5.0)
    g = 0.1 * G_TO_KMHS
    t = 0.0
    speed = 0.0
    us = []
    for i in range(80):
        # 前半は止まったまま、中盤は狙いを大きく超えて加速（クリープ相当）、終盤は減速
        rate = 0.0 if i < 10 else (3.0 * g if i < 40 else -2.0 * g)
        speed = max(0.0, speed + rate * 0.1)
        _cycle(loop, p, speed, t)
        us.append(loop._rp_u)
        t += 0.1
    assert all(u2 >= u1 - 1e-9 for u1, u2 in zip(us, us[1:], strict=False))


def test_launch_keeps_the_governor_working_on_the_brake() -> None:
    """発進のブレーキ指令にも G ガバナー（頭打ち）が効く（安全系はそのまま）。"""
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    _cycle(loop, p, 0.0, 0.0)
    loop._brake_gov_cap = 5.0
    loop._brake_request = 14.0
    _, b = loop._command_openings(p, 0.1)
    assert b <= 5.0 + 1e-9


def test_launch_records_brake_points_only_when_deceleration_is_positive() -> None:
    """発進（ブレーキを離している間）にアクセルが min のまま加速している間は、ブレーキの点
    （負の減速）を貯めない。"""
    loop, p = _launch_loop(0.1, stop_confirm_pct=14.0)
    ff = loop._profile.feedforward_params
    t = 0.0
    for _ in range(40):
        loop._brake_actual_pct = ff.brake_deadband_pct + 8.0
        loop._accel_actual_pct = ff.accel_deadband_pct
        _cycle(loop, p, 0.5 * t, t)  # 加速中 → ブレーキ側の減速は負
        t += 0.1
    assert loop._rp_u < 0.0  # まだ離している最中
    assert not loop.glimit.has_data(PEDAL_BRAKE)


def test_ramp_up_state_resets_for_every_ramp_up() -> None:
    loop, p = _launch_loop(stop_confirm_pct=14.0)
    _, bdb = _deadbands(loop)
    loop._rp_u = 7.0
    loop._rp_accel_reached = True
    loop._rp_vref = 33.0
    loop._rp_turn_started = 1.0
    loop._rp_brake_latched = True
    loop._enter_phase(_Phase.RAMP_UP, 5.0)  # 次の G ランプの頭
    assert loop._rp_u == pytest.approx(-(14.0 - bdb))
    assert not loop._rp_accel_reached and not loop._rp_brake_latched
    assert loop._rp_vref == 0.0 and loop._rp_turn_started is None
    assert loop._rp_e_prev is None and loop._rp_hold_over_count == 0


# ── 加速はブレーキへ行かない・減速は一度ブレーキに入ったらアクセルへ戻らない ──────────


def test_ramp_up_does_not_go_into_the_brake_even_when_speed_is_above_reference() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p], two_way=True)
    _cycle(loop, p, 10.0, 0.0)
    loop._rp_u = 0.2
    # 0.3G でずっと加速（基準 0.1G より速い）→ u は 0 で止まる
    _run(loop, p, lambda t: 10.0 + 0.3 * G_TO_KMHS * t, 0.1, 5.0)
    assert loop._phase is _Phase.RAMP_UP
    assert loop._rp_u == 0.0
    assert loop._command_openings(p, 5.1) == (0.0, 0.0)


def test_ramp_down_never_returns_to_the_accelerator_after_entering_the_brake() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p], two_way=True)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_u = -5.0
    loop._rp_brake_latched = True
    loop._rp_vref = 100.0
    _seed_sensitivity(loop, PEDAL_BRAKE, 1.0, 80.0)
    # 狙い 0.3G より強い減速（0.6G）がずっと続く → u は上がるが 0 を超えない
    _run(loop, p, lambda t: 100.0 - 0.6 * G_TO_KMHS * t, 0.1, 3.0)
    assert loop._rp_u == 0.0
    a, _ = loop._command_openings(p, 3.1)
    assert a == 0.0


def test_ramp_down_latch_is_set_by_stepping_into_the_brake() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_u = 0.5
    loop._rp_vref = 100.0
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 100.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 0.4, 100.0)
    assert not loop._rp_brake_latched
    _run(loop, p, lambda t: 100.0, 0.1, 2.5)  # 減速が足りない（基準 ＜ 実車速）→ u は負へ
    assert loop._rp_u < 0.0 and loop._rp_brake_latched


def test_ramp_down_weak_deceleration_can_still_use_the_accelerator_before_latch() -> None:
    p = _ramp(0.02, top_kmh=100.0)
    loop = _loop([p], two_way=True)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_u = 0.0
    loop._rp_vref = 100.0
    _seed_plant_table(loop)
    assert not loop._rp_brake_latched
    lo, hi = loop._ramp_u_limits()
    assert hi > 0.0 and lo < 0.0  # 印が立つまでは両側が使える
    plant = _Plant(loop, 100.0)
    t = 0.0
    while t < 40.0:
        _cycle(loop, p, plant.speed, t)
        plant.step()
        t += 0.1
    assert loop._rp_u > 0.5
    assert not loop._rp_brake_latched


def test_latch_is_set_on_switch_when_u_is_on_the_brake_side_and_cleared_on_next_ramp_up() -> None:
    p = _ramp(0.25, top_kmh=60.0)
    loop = _loop([p])
    _cycle(loop, p, 0.0, 0.0)
    loop._rp_u = -4.0
    loop._ramp_switch_down(5.0)
    assert loop._rp_brake_latched
    loop._enter_phase(_Phase.RAMP_UP, 10.0)  # 次の G ランプの RAMP_UP に入ったら下りる
    assert not loop._rp_brake_latched


# ── ブレーキの領域では 0% へ戻さず待機位置（min）で止める（2026-10-07 段2l-3、段2l-6 で min）──


def test_latched_brake_stops_at_min_and_never_returns_to_zero() -> None:
    """ラッチ後は u ≤ 0 でブレーキ ＝ min ＋ |u|。強い減速でも u は 0 で止まる。"""
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True), two_way=True)
    _, brake_db = _deadbands(loop)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_u = -5.0
    loop._rp_brake_latched = True
    loop._rp_vref = 100.0
    _seed_sensitivity(loop, PEDAL_BRAKE, 1.0, 80.0)
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


def test_latching_moment_keeps_u_continuous() -> None:
    """2026-10-09 段2l-8: ラッチした瞬間に u を 0 へ置き直さない（u は常に前の値から連続）。"""
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True))
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_u = 0.5
    loop._rp_vref = 100.0
    t = 0.1
    prev = loop._rp_u
    while not loop._rp_brake_latched and t < 20.0:  # 減速が足りない → u は負へ → ラッチ
        _cycle(loop, p, 100.0, t)
        assert abs(loop._rp_u - prev) <= 2.0 * _min_step(loop) + 1e-9  # 最小刻みの動き
        prev = loop._rp_u
        t += 0.1
    assert loop._rp_brake_latched
    assert loop._rp_u < 0.0 and loop._rp_u > -2.0 * _min_step(loop)


def test_latched_upper_bound_of_u_is_zero_regardless_of_standby_setting() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    for standby in (True, False):
        loop = _loop([p], config=PatternLoopConfig(pedal_standby=standby))
        loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
        loop._rp_brake_latched = True
        assert loop._ramp_u_limits()[1] == 0.0


def test_ramp_up_lower_limit_is_current_u_until_accel_reached_then_zero() -> None:
    loop, _ = _launch_loop(stop_confirm_pct=14.0)
    lo, _hi = loop._ramp_u_limits()
    assert lo == pytest.approx(loop._rp_u)  # 離す向きにしか動かさない（踏み直さない）
    loop._rp_u = -3.0
    assert loop._ramp_u_limits()[0] == -3.0
    loop._rp_u = 0.0
    loop._rp_accel_reached = True
    assert loop._ramp_u_limits()[0] == 0.0


def test_latched_governor_does_not_let_u_fall_further() -> None:
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p], config=PatternLoopConfig(pedal_standby=True))
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 100.0, 0.0)
    loop._rp_brake_latched = True
    loop._rp_u = -1.0
    loop._rp_vref = 100.0
    loop._brake_request = 10.0
    loop._brake_gov_cap = 1.0  # 削られている（指令 min ＋ 1 より低い）
    # 減速が足りない（基準 ＜ 実車速 → u は減る向き＝踏み増し）が、ガバナー中は積まない
    for i in range(1, 6):
        _cycle(loop, p, 100.0, 0.1 * i)
        loop._brake_request, loop._brake_gov_cap = 10.0, 1.0
    assert loop._rp_u == pytest.approx(-1.0)


def test_ramp_up_keeps_the_brake_command_at_zero_after_the_launch() -> None:
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
    loop._rp_vref = 20.0
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
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _cycle(loop, p, 50.0, 0.0)
    loop._rp_u = -5.0
    loop._rp_brake_latched = True
    loop._rp_vref = 50.0
    t = 0.1
    while t < 4.0:
        loop._accel_actual_pct = ff.accel_deadband_pct + 0.02
        loop._brake_actual_pct = ff.brake_deadband_pct + 5.0
        _cycle(loop, p, 50.0 - 0.5 * t, t)
        t += 0.1
    assert loop.glimit.has_data(PEDAL_BRAKE)


# ── CSV の基準車速と表示 ────────────────────────────────────────────────────


def test_csv_ref_speed_is_filled_during_ramp_and_none_otherwise() -> None:
    import asyncio

    seen: list[tuple[str, float | None]] = []

    def on_sample(
        data: DriveLogData, idx: int, phase: str, governed: bool,
        alarm_accel: bool | None, alarm_brake: bool | None,
        monitor_accel: AxisMonitor, monitor_brake: AxisMonitor, cycle_ms: float,
    ) -> None:
        seen.append((phase, data.ref_speed_kmh))

    cfg = cfgmod.load_config(CFG_PATH)
    rec = Recorder()
    p = _ramp(0.1)
    loop = PatternLoop(
        accel_driver=FakeAxis(), brake_driver=FakeAxis(), can_reader=FakeCAN(0.0),
        profile=build_vehicle_profile(cfg), patterns=[p, _next_pattern()],
        overcurrent_limit_ma=5000.0, on_complete=rec.on_complete,
        on_emergency=rec.on_emergency, on_sample=on_sample, interval_s=0.05,
    )

    async def go() -> None:
        loop._running = True
        for _ in range(5):
            await loop._execute_one_cycle()
            await asyncio.sleep(0.05)
        loop._enter_phase(_Phase.COAST, asyncio.get_running_loop().time())
        await loop._execute_one_cycle()

    asyncio.run(go())
    assert all(phase == "RAMP_UP" and ref is not None for phase, ref in seen[:5])
    assert seen[-1] == ("COAST", None)
    assert loop.ramp_status() is None


# ── 発進・停止の狙い G の立ち上げ・戻し（2026-10-09 段2l-9） ────────────────────


def _zero_times() -> PatternLoopConfig:
    return PatternLoopConfig(ramp_launch_time_s=0.0, ramp_turn_time_s=0.0, ramp_stop_time_s=0.0)


def test_gref_is_plus_g_before_the_vehicle_starts_moving() -> None:
    g = 0.1
    p = _ramp(g)
    loop = _loop([p], launched=False)
    for i in range(5):
        _cycle(loop, p, 0.0, 0.1 * i)  # 動かない間
        assert loop._ramp_gref(p, 0.1 * i) == pytest.approx(g)
    assert loop._rp_launch_started is None


def test_gref_rises_from_zero_over_launch_time_from_the_cycle_the_vehicle_moves() -> None:
    g = 0.2
    p = _ramp(g)
    loop = _loop([p], launched=False)
    launch_s = loop._config.ramp_launch_time_s
    assert launch_s > 0.0
    _cycle(loop, p, 0.0, 0.0)
    assert loop._ramp_gref(p, 0.0) == pytest.approx(g)  # 動き出す前
    t0 = 1.0
    _cycle(loop, p, plmod.STOP_SPEED_KMH + 1.0, t0)  # 動き出した周期
    assert loop._rp_launch_started == pytest.approx(t0)
    g_min = loop._config.ramp_edge_g
    assert loop._ramp_gref(p, t0) == pytest.approx(g_min)  # その周期の g_ref は下げ止め（0.05G）
    assert loop._ramp_gref(p, t0 + 0.5 * launch_s) == pytest.approx(0.5 * (g_min + g))
    assert loop._ramp_gref(p, t0 + launch_s) == pytest.approx(g)
    assert loop._ramp_gref(p, t0 + 5.0 * launch_s) == pytest.approx(g)


def test_opening_does_not_jump_in_the_cycle_after_the_reanchor(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """置き直しの周期は開度を動かさず、次の周期も表の値（どんなに大きくても）で飛ばない
    （段2l-10: 表は毎周期は引かない。発進の寄せを決めるのは u ≥ 0 になった周期だけ）。"""
    p = _ramp(0.2)
    loop = _loop([p], launched=False)
    monkeypatch.setattr(loop, "_ramp_estimate_u", lambda speed, g_ref: 10.0 + 500.0 * g_ref)
    speed = plmod.STOP_SPEED_KMH + 1.0
    _cycle(loop, p, 0.0, 0.0)
    _cycle(loop, p, 0.0, 0.1)
    u_before = loop._rp_u
    _cycle(loop, p, speed, 0.2)  # 置き直しの周期
    assert loop._rp_u == pytest.approx(u_before)
    u_anchor = loop._rp_u
    _cycle(loop, p, speed, 0.3)  # 次の周期（感度なしは最小刻み 1 個）
    assert abs(loop._rp_u - u_anchor) <= _min_step(loop) * 2.0


def test_stop_gref_is_minus_g_above_v_stop_and_sqrt_below() -> None:
    g = 0.2
    p = _ramp(g)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    v_stop = g * G_TO_KMHS * loop._config.ramp_stop_time_s / 2.0
    loop._rp_vref = 2.0 * v_stop
    assert loop._ramp_gref(p, 0.0) == pytest.approx(-g)
    loop._rp_vref = v_stop
    assert loop._ramp_gref(p, 0.0) == pytest.approx(-g)
    loop._rp_vref = 0.25 * v_stop
    assert loop._ramp_gref(p, 0.0) == pytest.approx(-g * 0.5)
    loop._rp_vref = 0.0
    assert loop._ramp_gref(p, 0.0) == pytest.approx(-loop._config.ramp_edge_g)  # 下げ止め
    loop._rp_vref = 0.01 * v_stop  # √ の式が g_min を下回る低さでも −g_min
    assert loop._ramp_gref(p, 0.0) == pytest.approx(-loop._config.ramp_edge_g)


def test_stop_reference_reaches_zero_kmh() -> None:
    g = 0.2
    p = _ramp(g, top_kmh=200.0)
    loop = _loop([p])
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 5.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 0.4, 5.0)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_vref = 10.0
    t = 0.0
    prev = loop._rp_vref
    for _ in range(100):
        _cycle(loop, p, 5.0, t)  # 実車速は停車させない
        assert loop._rp_vref <= prev + 1e-9  # 戻らず単調に下がる
        prev = loop._rp_vref
        t += 0.1
    assert loop._phase is _Phase.RAMP_DOWN
    assert loop._rp_vref == 0.0


def test_gref_is_continuous_when_turning_in_the_middle_of_the_launch() -> None:
    g = 0.1
    p = _ramp(g)
    loop = _loop([p], launched=False)
    loop._rp_vref = 10.0  # 停止の式に掛からない高さ
    loop._rp_vref_anchored = True
    loop._rp_launch_started = 1.0
    loop._ramp_switch_down(1.4, 10.0)  # 発進の途中（経過 0.4 秒 ＝ 0.4G）で折り返し
    assert loop._phase is _Phase.RAMP_DOWN
    g_min = min(loop._config.ramp_edge_g, g)
    g_mid = g_min + 0.4 * (g - g_min)
    assert loop._rp_turn_g0 == pytest.approx(g_mid)
    assert loop._ramp_gref(p, 1.4) == pytest.approx(g_mid)  # 直前と同じ値
    turn_s = loop._config.ramp_turn_time_s
    assert loop._ramp_gref(p, 1.4 + 0.5 * turn_s) == pytest.approx(0.5 * (g_mid - g))
    assert loop._ramp_gref(p, 1.4 + turn_s) == pytest.approx(-g)


def test_gref_is_the_old_shape_when_the_three_times_are_zero() -> None:
    g = 0.15
    p = _ramp(g)
    loop = _loop([p], launched=False, config=_zero_times())
    _cycle(loop, p, 0.0, 0.0)
    assert loop._ramp_gref(p, 0.0) == pytest.approx(g)
    _cycle(loop, p, plmod.STOP_SPEED_KMH + 1.0, 1.0)  # 動き出した周期も +G
    assert loop._ramp_gref(p, 1.0) == pytest.approx(g)
    loop._rp_vref = 30.0
    loop._ramp_switch_down(2.0)
    assert loop._ramp_gref(p, 2.0) == pytest.approx(-g)  # 折り返しは即 −G
    loop._rp_vref = 0.0
    assert loop._ramp_gref(p, 3.0) == pytest.approx(-g)  # 停止も −G のまま


def test_gref_has_a_floor_edge_g_at_launch_and_stop() -> None:
    """下げ止め（2026-10-09）: 狙いの G が ramp_edge_g より小さい段は丸めない（ずっと G）。"""
    g = 0.03
    p = _ramp(g)
    loop = _loop([p], launched=False)
    assert loop._config.ramp_edge_g > g
    _cycle(loop, p, plmod.STOP_SPEED_KMH + 1.0, 1.0)
    assert loop._ramp_gref(p, 1.0) == pytest.approx(g)
    assert loop._ramp_gref(p, 1.5) == pytest.approx(g)
    loop._enter_phase(_Phase.RAMP_DOWN, 2.0)
    loop._rp_vref = 0.0
    assert loop._ramp_gref(p, 3.0) == pytest.approx(-g)


def test_gref_goes_down_to_zero_when_edge_g_is_zero() -> None:
    g = 0.2
    p = _ramp(g)
    loop = _loop([p], launched=False, config=PatternLoopConfig(ramp_edge_g=0.0))
    _cycle(loop, p, plmod.STOP_SPEED_KMH + 1.0, 1.0)
    assert loop._ramp_gref(p, 1.0) == pytest.approx(0.0)
    assert loop._ramp_gref(p, 1.5) == pytest.approx(0.5 * g)
    loop._enter_phase(_Phase.RAMP_DOWN, 2.0)
    loop._rp_vref = 0.0
    assert loop._ramp_gref(p, 3.0) == pytest.approx(0.0)


# ── 段2l-10: 偏差の不感帯 ────────────────────────────────────────────────


def _hold_loop(**kw: float) -> tuple[PatternLoop, plmod.GRampPattern]:
    """RAMP_HOLD の途中（g_ref ＝ 0 で基準が動かない）の状態。偏差を手で作るために使う。"""
    p = _ramp(0.1, top_kmh=200.0)
    loop = _loop([p], **kw)  # type: ignore[arg-type]
    loop._enter_phase(_Phase.RAMP_HOLD, 0.0)
    loop._rp_hold_started = 0.0
    loop._rp_hold_g0 = 0.0
    loop._rp_vref_anchored = True
    loop._rp_blend_decided = True
    return loop, p


def test_dead_helper_is_zero_inside_and_continuous_at_the_edge() -> None:
    d = PatternLoop._dead
    assert d(0.0, 0.5) == 0.0 and d(0.5, 0.5) == 0.0 and d(-0.5, 0.5) == 0.0
    assert d(0.51, 0.5) == pytest.approx(0.01) and d(-0.9, 0.5) == pytest.approx(-0.4)
    assert d(0.3, 0.0) == 0.3


def test_deviation_inside_the_deadband_does_not_move_u_and_edge_is_continuous() -> None:
    loop, p = _hold_loop(deadband=0.5)
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 20.0)
    loop._rp_u = 5.0
    t_settle = loop._config.ramp_settle_time_s
    kp = 1.0 / (0.2 * t_settle)
    ki = kp / (plmod.RAMP_TI_RATIO * t_settle)
    speed = 20.0
    loop._rp_vref = speed
    _cycle(loop, p, speed, 0.0)  # dt ＝ 0 の最初の周期
    e_prev_dead = 0.0
    for k in range(1, 20):
        e = 0.05 * k  # 0.05 … 0.95 km/h（0.5 をまたぐ）
        loop._rp_vref = speed + e
        u0 = loop._rp_u
        _cycle(loop, p, speed, 0.1 * k)
        e_dead = max(0.0, e - 0.5)
        expected = kp * (e_dead - e_prev_dead) + ki * e_dead * 0.1
        assert loop._rp_u - u0 == pytest.approx(expected, abs=1e-9)
        if e <= 0.5:
            assert loop._rp_u == u0  # 不感帯の内側（境目を含む）は動かない
        e_prev_dead = e_dead
    # 境目をまたぐ周期の増分は Kp × 1 刻みぶん程度（飛ばない）
    assert kp * 0.05 + ki * 0.5 * 0.1 < 1.0


def test_without_sensitivity_zero_e_dead_does_not_move_and_outside_moves_one_step() -> None:
    loop, p = _hold_loop(deadband=0.5)
    step = _min_step(loop)
    speed = 20.0
    loop._rp_vref = speed + 0.4
    _cycle(loop, p, speed, 0.0)
    for k in range(1, 4):
        loop._rp_vref = speed + 0.4
        _cycle(loop, p, speed, 0.1 * k)
        assert loop._rp_u == 0.0  # e′ ＝ 0
    loop._rp_vref = speed + 0.6
    _cycle(loop, p, speed, 0.5)
    assert loop._rp_u == pytest.approx(step)  # e′ ＞ 0 の向きへ最小刻み 1 個


def test_deadband_is_not_used_in_ramp_down_after_the_reference_reached_zero() -> None:
    """基準が 0 km/h に着いた後（STOP_SPEED_KMH 未満）は不感帯なし: 実車速 0.3 km/h でも踏み増す。
    基準が着く前は、偏差が不感帯の内側（基準が 1 周期で 0.35 下がるぶん）なら動かない。"""
    p = _ramp(0.1, top_kmh=60.0)
    for vref, speed, moves in ((0.0, 0.3, True), (5.0, 5.0, False)):
        loop = _loop([p], deadband=0.5)
        loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
        loop._rp_vref_anchored = True
        loop._rp_vref = vref
        _cycle(loop, p, speed, 0.0)
        loop._rp_vref = vref
        _cycle(loop, p, speed, 0.1)
        assert (loop._rp_u != 0.0) is moves


def test_deadband_is_not_used_in_ramp_up_before_the_reanchor() -> None:
    """発進で置き直しの前（ブレーキを離している間）は、小さな偏差でも今までどおり開度を動かす。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p], launched=False, deadband=0.5)
    u0 = loop._rp_u
    _cycle(loop, p, 0.0, 0.0)
    _cycle(loop, p, 0.0, 0.1)  # 基準 0.35 km/h ＜ 不感帯 0.5 だが動かす
    assert loop._rp_u > u0


def test_deadband_switch_does_not_make_u_jump() -> None:
    """RAMP_DOWN で基準が STOP_SPEED_KMH を下回り不感帯が 0 に切り替わる周期に、開度が飛ばない
    （前周期の偏差を今周期の不感帯で引き直す）。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p], deadband=0.5)
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 2.0)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_vref_anchored = True
    speed = 3.0
    loop._rp_vref = plmod.STOP_SPEED_KMH + 0.02  # 次の周期で 0.5 未満に入る
    loop._rp_ctl_t = None
    _cycle(loop, p, speed, 0.0)
    u_before = loop._rp_u
    _cycle(loop, p, speed, 0.1)
    kp = 1.0 / (0.2 * loop._config.ramp_settle_time_s)
    # 偏差の差 ≒ 基準の下がり幅 ＋ 不感帯の有無で変わるぶんを含まない（Kp × 数 km/h にならない）
    assert abs(loop._rp_u - u_before) < kp * 0.2


# ── 段2l-10: 表の寄せ ────────────────────────────────────────────────────


def test_blend_is_signed_never_exceeds_the_remainder_and_zero_time_is_one_shot() -> None:
    loop = _loop([_ramp()])
    loop._ramp_blend_start(-6.0, 4.0, 1.0)  # gap ＝ −10 を 1 秒で
    total = 0.0
    for _ in range(4):
        du = loop._ramp_blend_step(0.3)
        assert du <= 0.0
        total += du
    assert total == pytest.approx(-10.0)  # 残りを超えない（最後の周期は残りぶんだけ）
    assert loop._ramp_blend_step(0.3) == 0.0
    loop._ramp_blend_start(7.0, 2.0, 0.0)  # 時間 0 は次の周期に全部
    assert loop._ramp_blend_step(0.1) == pytest.approx(5.0)
    assert loop._ramp_blend_step(0.1) == 0.0
    loop._ramp_blend_start(None, 2.0, 1.0)  # 引けなければ寄せない
    assert loop._ramp_blend_step(0.1) == 0.0


def test_blend_over_time_adds_the_gap_in_equal_steps() -> None:
    loop = _loop([_ramp()])
    loop._ramp_blend_start(10.0, 0.0, 2.0)
    steps = [loop._ramp_blend_step(0.1) for _ in range(20)]
    assert all(d == pytest.approx(0.5) for d in steps)
    assert sum(steps) == pytest.approx(10.0)
    assert loop._ramp_blend_step(0.1) == 0.0


def test_launch_blend_only_increases_u() -> None:
    loop = _loop([_ramp()])
    loop._ramp_blend_start(1.0, 5.0, 0.5, increase_only=True)  # 表の開度が今の u より低い
    assert loop._ramp_blend_step(0.1) == 0.0
    loop._ramp_blend_start(9.0, 5.0, 0.5, increase_only=True)
    assert loop._ramp_blend_step(0.5) == pytest.approx(4.0)


def test_hold_head_blends_toward_the_zero_g_opening_once(monkeypatch: pytest.MonkeyPatch) -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _seed_plant_table(loop)
    calls: list[tuple[float, float]] = []
    real = loop._ramp_estimate_u

    def spy(speed: float, g: float) -> float | None:
        calls.append((speed, g))
        return real(speed, g)

    monkeypatch.setattr(loop, "_ramp_estimate_u", spy)
    loop._rp_u = 3.0
    loop._ramp_switch_hold(10.0, 45.0)
    assert loop._phase is _Phase.RAMP_HOLD
    assert calls == [(45.0, 0.0)]  # 実車速・0G で 1 回だけ
    target = real(45.0, 0.0)
    assert target is not None and target > 3.0
    assert loop._rp_blend_gap == pytest.approx(target - 3.0)
    assert loop._rp_blend_time == pytest.approx(loop._config.ramp_turn_time_s)
    # 保持の間、表が変わっても引き直さない
    for i in range(1, 40):
        _cycle(loop, p, 45.0, 10.0 + 0.1 * i)
    assert calls == [(45.0, 0.0)]


def test_hold_head_without_table_or_with_brake_side_target(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    loop._rp_u = 3.0
    loop._ramp_switch_hold(10.0, 45.0)  # 表が空 → 寄せない
    assert loop._rp_blend_left == 0.0
    loop2 = _loop([p])
    monkeypatch.setattr(loop2, "_ramp_estimate_u", lambda speed, g: -5.0)  # ブレーキ側 → 0 に丸める
    loop2._rp_u = 3.0
    loop2._ramp_switch_hold(10.0, 45.0)
    assert loop2._rp_blend_gap == pytest.approx(-3.0)
    loop3 = _loop([p], launched=False)  # 発進が終わらないまま（u ＜ 0）は寄せない
    monkeypatch.setattr(loop3, "_ramp_estimate_u", lambda speed, g: 8.0)
    loop3._ramp_switch_hold(10.0, 45.0)
    assert loop3._rp_u < 0.0 and loop3._rp_blend_left == 0.0


def test_descent_head_rule1_milder_than_coast_uses_the_accel_side_estimate() -> None:
    p = _ramp(0.02, top_kmh=60.0)  # 0.02G ＝ 0.71 km/h/s ＜ 惰行 1.0
    loop = _loop([p])
    _seed_plant_table(loop)
    loop._rp_u = 8.0
    loop._ramp_switch_down(20.0, 45.0)
    assert loop._phase is _Phase.RAMP_DOWN
    target = loop._ramp_estimate_u(45.0, -0.02)
    assert target is not None and target > 0.0
    assert loop._rp_blend_gap == pytest.approx(target - 8.0)
    assert loop._rp_blend_time == pytest.approx(loop._config.ramp_turn_time_s)


def test_descent_head_rule2_interpolates_in_own_band_only() -> None:
    p = _ramp(0.1, top_kmh=60.0)  # 0.1G ＝ 3.53 km/h/s ＞ 惰行 1.0 → ブレーキ側
    loop = _loop([p])
    _seed_plant_table(loop)
    _, bdb = _deadbands(loop)
    loop._rp_u = 2.0
    loop._ramp_switch_down(20.0, 45.0)
    opening = loop.glimit.interp_pct(PEDAL_BRAKE, 45.0, 0.1 * G_TO_KMHS, bdb)
    assert opening is not None
    # 2026-10-10 段2l-11: 1 つ目はアクセルを離す（u ＝ 0 へ）だけ。ブレーキ側は 2 つ目（下のテスト）
    assert loop._rp_blend_gap == pytest.approx(-2.0)
    assert loop._rp_down_brake_blend_pending
    assert loop._rp_brake_latched is False


def test_descent_head_rule3_releases_the_accelerator_only() -> None:
    p = _ramp(0.5, top_kmh=60.0)  # 狙いが貯めた点のどれより強い → 内挿できない
    loop = _loop([p])
    _seed_plant_table(loop)
    loop._rp_u = 8.0
    loop._ramp_switch_down(20.0, 45.0)
    assert loop._rp_blend_gap == pytest.approx(-8.0)  # u ＝ 0 へ
    # u ≤ 0 なら寄せない（ブレーキを踏み増す向きに動かさない）
    loop2 = _loop([p])
    _seed_plant_table(loop2)
    loop2._rp_u = -3.0
    loop2._ramp_switch_down(20.0, 45.0)
    assert loop2._rp_blend_left == 0.0 and loop2._rp_brake_latched
    # 自分の帯に点が無く、他の帯にしかない → 借用せず rule 3
    loop3 = _loop([_ramp(0.1, top_kmh=60.0)])
    for k in range(5):
        loop3.glimit.add(PEDAL_BRAKE, 5.0, 8.0 + 2.0 * k, 3.0 + k)
    loop3._rp_u = 4.0
    loop3._ramp_switch_down(20.0, 45.0)
    assert loop3._rp_blend_gap == pytest.approx(-4.0)


def test_switching_phase_discards_the_previous_blend_remainder() -> None:
    p = _ramp(0.5, top_kmh=60.0)
    loop = _loop([p])
    loop._ramp_blend_start(20.0, 0.0, 5.0)
    loop._ramp_blend_step(0.1)
    assert loop._rp_blend_left > 0.0
    loop._rp_u = 0.0  # rule 3 かつ u ＝ 0 → 寄せない
    loop._ramp_switch_down(1.0, 30.0)
    assert loop._rp_blend_left == 0.0


# ── 段2l-10 ア: アクチュエータが追いつかない間は基準を待たせる ─────────────────────


def _lag_scenario(cmd_minus_actual: float, e_sign: float = 1.0, *, actual_known: bool = True,
                  tol: float = 2.0) -> tuple[float, float]:
    """RAMP_UP の u ＝ 10%（アクセル）で、指令 − 実開度 ＝ cmd_minus_actual のまま 1.5 秒走らせて
    (u の増分, 基準が実車速より先に行った量) を返す。"""
    p = _ramp(0.2, top_kmh=200.0)
    loop = _loop([p], lag_tol=tol, two_way=True)
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 20.0)
    adb, _ = _deadbands(loop)
    loop._rp_u = 10.0
    loop._rp_vref_anchored = True
    loop._rp_blend_decided = True
    speed = 20.0
    loop._rp_vref = speed
    _cycle(loop, p, speed, 0.0)
    for i in range(1, 16):
        u = loop._rp_u
        loop._accel_actual_pct = (adb + max(u, 0.0) - cmd_minus_actual) if actual_known else None
        if e_sign < 0:
            loop._rp_vref = speed - 1.0  # 基準が実車速より遅れている（開度は下げたい）
        _cycle(loop, p, speed, 0.1 * i)
    return loop._rp_u - 10.0, loop._rp_vref - speed


def test_reference_waits_when_actuator_lags_and_command_moves_further_away() -> None:
    du_lag, e_lag = _lag_scenario(5.0)  # 実開度が指令より 5%（＞ 2%）遅れている
    du_ok, e_ok = _lag_scenario(1.0)  # 1% は許容内
    assert du_lag > 0.0  # 開度 u は止めない
    assert e_lag < e_ok  # 基準だけが待つ（偏差が広がらない）
    assert e_ok > 3.0  # 許容内なら基準は先へ進む（狙いの G ぶん）
    assert e_lag < 0.5


def test_actuator_lag_guard_is_off_without_actual_opening_or_when_moving_closer() -> None:
    _, e_none = _lag_scenario(5.0, actual_known=False)
    assert e_none > 3.0  # 実開度が読めていない（指令値にフォールバック）ときは働かない
    # 開度を下げる向き（実開度に近づく）の周期は待たせない: 基準が遅れていて u が減る
    du, _ = _lag_scenario(5.0, e_sign=-1.0)
    assert du < 0.0


# ── 段2l-11（A）: 感度の下限・発進の寄せ 2 秒・停止の直前 ─────────────────────


def test_sensitivity_from_points_with_tiny_accel_difference_is_not_used_for_kp() -> None:
    """点の開度の幅 × 傾き（加速度の差）が不感帯 ÷ T 未満なら感度なし → 最小刻み 1 個/周期。
    そのままなら Kp ＝ 1 ÷（0.01 × T）と大きな値になり、ブレーキが跳ねる。"""
    p = _ramp(0.3, top_kmh=100.0)
    loop = _loop([p], deadband=0.5, two_way=True)
    cfg = loop._config
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _, bdb = _deadbands(loop)
    for u, a in ((bdb + 1.0, 1.0), (bdb + 2.0, 1.01), (bdb + 3.0, 1.02)):  # 傾き 0.01、差 0.02
        loop.glimit.add(PEDAL_BRAKE, 60.0, u, a)
    loop._brake_actual_pct = bdb + 2.0
    loop._rp_u = -3.0
    loop._rp_brake_latched = True
    loop._rp_blend_decided = True
    loop._rp_vref_anchored = True
    loop._rp_vref = 60.0
    _cycle(loop, p, 59.0, 0.0)
    u0 = loop._rp_u
    loop._rp_vref = 61.0  # 偏差が不感帯の外へ
    _cycle(loop, p, 59.0, 0.1)
    st = loop.ramp_status()
    assert st is not None and (st.brake_kp, st.brake_ki) == (0.0, 0.0)
    assert abs(loop._rp_u - u0) == pytest.approx(_min_step(loop))
    assert cfg.ramp_dev_deadband_kmh / cfg.ramp_settle_time_s > 0.02
    # 差が十分な点が貯まれば Kp を使う
    _seed_sensitivity(loop, PEDAL_BRAKE, 1.0, 60.0)
    _cycle(loop, p, 59.0, 0.2)
    st = loop.ramp_status()
    assert st is not None and st.brake_kp > 0.0


def test_launch_blend_takes_ramp_turn_time_and_targets_the_step_g() -> None:
    """発進の寄せは `_ramp_launch_blend_time_s`（0.08G は 0.5 秒）かけ、
    寄せ先は段の G が出る開度（g_ref ではない）。"""
    p = _ramp(0.08)
    loop = _loop([p], launched=False)
    _seed_accel_points(loop)
    loop._rp_u = 0.0
    loop._rp_accel_reached = False
    loop._rp_vref_anchored = True
    loop._rp_launch_started = 0.0
    loop._rp_e_prev = 0.0
    target = loop._ramp_estimate_u(1.0, p.target_g)
    assert target is not None and target > 3.0
    _cycle(loop, p, 1.0, 0.0)
    u_before = loop._rp_u
    _cycle(loop, p, 1.0, 0.1)  # u ≥ 0 でアクセル側に移った周期に寄せを決める
    assert loop._rp_blend_decided
    assert loop._rp_blend_time == pytest.approx(loop._ramp_launch_blend_time_s(p))
    assert loop._rp_blend_gap == pytest.approx(target - u_before, abs=0.5)
    assert loop._rp_blend_time == pytest.approx(0.5)
    assert not hasattr(plmod, "RAMP_TABLE_BLEND_TIME_S")


def test_launch_blend_at_the_first_cycle_also_takes_ramp_turn_time() -> None:
    p = _ramp(0.08)
    loop = _loop([p])  # 最初から u ≥ 0
    _seed_accel_points(loop)
    _cycle(loop, p, 0.0, 0.0)
    assert loop._rp_blend_decided
    assert loop._rp_blend_time == pytest.approx(loop._ramp_launch_blend_time_s(p))


def test_near_stop_brake_press_is_not_slower_than_one_step() -> None:
    """基準が 0 km/h に着き、実車速が停止速度より上、感度がある間は、偏差が小さくても
    ブレーキの踏み増し（u が減る向き）は最小刻み 1 個/周期以上。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 1.0, 1.0)
    loop._rp_u = -3.0
    loop._rp_brake_latched = True
    loop._rp_blend_decided = True
    loop._rp_vref_anchored = True
    speed = plmod.STOP_SPEED_KMH + 0.1  # 偏差 −0.1 km/h 程度。Ki の項だけでは 0.01 %/秒
    loop._rp_vref = 0.0
    _cycle(loop, p, speed, 0.0)
    step = _min_step(loop)
    for k in range(1, 6):
        u0 = loop._rp_u
        loop._rp_vref = 0.0
        _cycle(loop, p, speed, 0.1 * k)
        assert loop._rp_u - u0 <= -step + 1e-12
    st = loop.ramp_status()
    assert st is not None and st.brake_kp > 0.0  # 感度ありの周期


def test_near_stop_floor_is_not_applied_once_the_car_has_stopped() -> None:
    """実車速が停止速度以下なら下限は効かない（偏差の項のまま）。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 1.0, 0.0)
    loop._rp_u = -3.0
    loop._rp_brake_latched = True
    loop._rp_blend_decided = True
    loop._rp_vref_anchored = True
    loop._rp_vref = 0.0
    _cycle(loop, p, 0.0, 0.0)
    u0 = loop._rp_u
    loop._rp_vref = 0.0
    _cycle(loop, p, 0.0, 0.1)
    assert abs(loop._rp_u - u0) < _min_step(loop)


# ── 段2l-11 B: 実車速での保持への切り替えと基準の置き直し ──────────────────


def test_ramp_up_continues_while_the_car_lags_behind_the_reference() -> None:
    """基準車速が上端を越えても、実車速が上端の手前に着くまで RAMP_UP が続く。"""
    p = _ramp(0.2, top_kmh=60.0)
    loop = _loop([p])
    loop._rp_vref = 80.0  # 基準は上端をとうに越えている（車は遅れている）
    for i in range(30):
        _cycle(loop, p, 59.9, 0.1 * i)  # 実車速は上端の手前で動かない（測った加速度 0）
        loop._rp_vref = 80.0
    assert loop._phase is _Phase.RAMP_UP  # 加速度 0 なら上端ちょうどまで保持へ入らない
    loop2 = _loop([p])
    for i in range(30):
        _cycle(loop2, p, 60.0, 0.1 * i)
        if loop2._phase is _Phase.RAMP_HOLD:
            break
    assert loop2._phase is _Phase.RAMP_HOLD


def test_ramp_up_to_hold_threshold_uses_the_measured_acceleration() -> None:
    """実車速 ≥ 上端 − max(0, 実測の加速度) × ramp_turn_time_s ÷ 2（5 km/h/s・T=2 s → 手前 5）。"""
    p = _ramp(0.2, top_kmh=60.0)
    loop = _loop([p])
    accel = 5.0
    turn = loop._config.ramp_turn_time_s
    t, speed = 0.0, 40.0
    while loop._phase is _Phase.RAMP_UP and t < 10.0:
        speed = 40.0 + accel * t
        _cycle(loop, p, speed, t)
        t += 0.1
    assert loop._phase is _Phase.RAMP_HOLD
    lead = accel * turn / 2.0
    assert 60.0 - lead <= speed < 60.0 - lead + accel * 0.1 + 1e-6
    # 切り替えた周期: 基準 ＝ 実車速・e_前 ＝ 0・保持の g_ref の始まり ＝ 実測の加速度 ÷ G_TO_KMHS
    assert loop._rp_vref == pytest.approx(speed)
    assert loop._rp_e_prev == 0.0
    assert loop._rp_hold_g0 == pytest.approx(accel / G_TO_KMHS, abs=1e-6)


def test_switch_hold_g0_is_clamped_between_zero_and_the_target_g() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    for accel, expected in ((1.0 * G_TO_KMHS * 0.05, 0.05), (5.0, 0.1), (-2.0, 0.0)):
        loop = _loop([p])
        loop._ramp_switch_hold(10.0, 45.0, accel)
        assert loop._rp_hold_g0 == pytest.approx(expected)
        assert loop._rp_vref == 45.0 and loop._rp_e_prev == 0.0
    # 測れていない（None）ときは今までどおり入った時点の g_ref
    loop = _loop([p])
    expected = loop._ramp_gref(p, 10.0)
    loop._ramp_switch_hold(10.0, 45.0)
    assert loop._rp_hold_g0 == pytest.approx(expected) and expected > 0.0


def test_opening_does_not_jump_at_the_switch_to_hold() -> None:
    """基準が車から離れていても、置き直しの周期に偏差の項で開度が飛ばない（変化分の式）。"""
    g, accel = 0.2, 5.0
    p = _ramp(g, top_kmh=60.0)
    loop = _loop([p])
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 55.0)
    loop._rp_vref = 90.0  # 基準が車より 35 km/h 先（置き直し前の e_前 は大きな値）
    t, speed = 0.0, 40.0
    while loop._phase is _Phase.RAMP_UP and t < 10.0:
        speed = 40.0 + accel * t
        loop._rp_vref = max(loop._rp_vref, speed + 35.0)
        _cycle(loop, p, speed, t)
        t += 0.1
    assert loop._phase is _Phase.RAMP_HOLD
    loop._rp_blend_left = 0.0  # 寄せは別に見る。偏差の項だけを見る
    u_before = loop._rp_u
    _cycle(loop, p, speed + accel * 0.1, t)  # 車は同じ加速度で進む
    assert abs(loop._rp_u - u_before) < 0.3  # 置き直さなければ kp × 35 km/h ≈ 90% 飛ぶ


def test_switch_down_resets_reference_to_the_actual_speed() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    loop._rp_vref = 61.0
    loop._rp_e_prev = 3.0
    loop._ramp_switch_down(20.0, 58.0)
    assert loop._phase is _Phase.RAMP_DOWN
    assert loop._rp_vref == 58.0 and loop._rp_e_prev == 0.0


# ── 段2l-11 B: 減速の頭の寄せを 2 つに ────────────────────────────────────


def test_descent_head_brake_blend_starts_after_the_accelerator_release(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    p = _ramp(0.1, top_kmh=60.0)  # 0.1G ＞ 惰行 → ブレーキ側
    loop = _loop([p], two_way=True)
    _seed_plant_table(loop)
    _, bdb = _deadbands(loop)
    calls: list[float] = []
    real = loop._ramp_descent_brake_target

    def spy(pattern: plmod.GRampPattern, speed: float) -> float | None:
        calls.append(speed)
        return real(pattern, speed)

    monkeypatch.setattr(loop, "_ramp_descent_brake_target", spy)
    loop._rp_u = 2.0
    loop._ramp_switch_down(20.0, 45.0)
    assert loop._rp_blend_gap == pytest.approx(-2.0) and loop._rp_down_brake_blend_pending
    expected = real(p, 45.0)
    assert expected is not None and expected < 0.0
    t, flipped_at = 20.0, None
    for i in range(60):
        _cycle(loop, p, 45.0, t)
        if not loop._rp_down_brake_blend_pending and flipped_at is None:
            flipped_at = i
            # 1 つ目の寄せが終わった周期（残りなし）か u ≤ 0 になった周期
            gap_after = loop._rp_blend_gap
            u_now = loop._rp_u
        t += 0.1
    assert flipped_at is not None and flipped_at >= 15  # 2 秒かけて離し終えるまで始まらない
    assert len(calls) == 1  # 段ごとに 1 回だけ
    assert gap_after == pytest.approx(expected - u_now)  # その周期の u からの差
    assert bdb > 0.0


def test_descent_head_brake_blend_starts_early_when_u_reaches_zero() -> None:
    """1 つ目（アクセルを離す）が終わる前に u が 0 以下になったら、その周期にブレーキ側の寄せを
    始める。1 つ目の残りは捨てる（2026-10-10 段2l-11。残りをブレーキ側へ持ち込まない）。"""
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _seed_plant_table(loop)
    loop._rp_u = 0.3
    loop._ramp_switch_down(20.0, 45.0)
    assert loop._rp_down_brake_blend_pending and loop._rp_blend_release
    target = loop._ramp_descent_brake_target(p, 45.0)
    assert target is not None and target < -1.0
    loop._rp_vref = 44.0  # 偏差の項が u を少し下げる（1 つ目の寄せが終わる前に u ≤ 0）
    t = 20.0
    for _ in range(40):
        loop._rp_vref = min(loop._rp_vref, 44.0)
        _cycle(loop, p, 45.0, t)
        if not loop._rp_down_brake_blend_pending:
            break
        t += 0.1
    assert not loop._rp_down_brake_blend_pending
    assert target < loop._rp_u <= 0.0
    assert not loop._rp_blend_release  # 今の寄せはブレーキ側のもの
    assert loop._rp_blend_left < 0.0
    assert loop._rp_blend_time == pytest.approx(loop._config.ramp_turn_time_s)


def _down_with_pending(
    loop: PatternLoop, p: plmod.GRampPattern, u: float, monkeypatch: pytest.MonkeyPatch,
    target: float | None,
) -> None:
    """RAMP_DOWN でブレーキ側にいて 2 つ目の寄せが待ちの状態を作る（寄せ先は差し替える）。"""
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    loop._rp_vref = 30.0
    loop._rp_u = u
    loop._rp_brake_latched = True
    loop._rp_down_brake_blend_pending = True
    monkeypatch.setattr(loop, "_ramp_descent_brake_target", lambda pattern, speed: target)


def test_brake_blend_is_skipped_when_the_target_is_shallower_than_u(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    _down_with_pending(loop, p, -4.0, monkeypatch, -2.0)  # 寄せ先がいまの u より浅い
    _cycle(loop, p, 30.0, 1.0)
    assert not loop._rp_down_brake_blend_pending
    assert loop._rp_blend_left == 0.0
    loop2 = _loop([p])
    _down_with_pending(loop2, p, -4.0, monkeypatch, -6.0)  # 深い → その周期の u からの差
    _cycle(loop2, p, 30.0, 1.0)
    assert loop2._rp_blend_gap == pytest.approx(-6.0 - loop2._rp_u)
    assert loop2._rp_blend_time == pytest.approx(loop2._config.ramp_turn_time_s)


def test_brake_blend_is_skipped_when_the_estimate_cannot_be_drawn(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])  # 表が空 → 自分の車速帯で内挿できない
    loop._rp_u = 3.0
    loop._ramp_switch_down(20.0, 45.0)
    assert loop._rp_down_brake_blend_pending
    for i in range(40):
        _cycle(loop, p, 45.0, 20.0 + 0.1 * i)
    assert not loop._rp_down_brake_blend_pending
    assert loop._rp_blend_left == 0.0  # 偏差の項に任せる


def test_descent_head_first_blend_variants_set_the_pending_flag() -> None:
    # 惰行より緩く、アクセル側の値が引ける: そのまま 1 回。2 つ目は無し
    p = _ramp(0.02, top_kmh=60.0)
    loop = _loop([p])
    _seed_plant_table(loop)
    loop._rp_u = 8.0
    loop._ramp_switch_down(20.0, 45.0)
    assert not loop._rp_down_brake_blend_pending and loop._rp_blend_left != 0.0
    # u ≤ 0 で入った段（発進が終わらないまま来た）: 1 つ目も 2 つ目も無し
    p2 = _ramp(0.5, top_kmh=60.0)
    loop2 = _loop([p2])
    _seed_plant_table(loop2)
    loop2._rp_u = -3.0
    loop2._ramp_switch_down(20.0, 45.0)
    assert not loop2._rp_down_brake_blend_pending and loop2._rp_blend_left == 0.0
    # u ＞ 0 でブレーキ側: 待ち
    loop3 = _loop([p2])
    _seed_plant_table(loop3)
    loop3._rp_u = 8.0
    loop3._ramp_switch_down(20.0, 45.0)
    assert loop3._rp_down_brake_blend_pending


def test_pending_flag_is_cleared_when_a_new_ramp_starts_or_the_section_changes() -> None:
    p = _ramp(0.1, top_kmh=60.0)
    loop = _loop([p])
    loop._rp_down_brake_blend_pending = True
    loop._ramp_switch_hold(1.0, 40.0)  # 別の区間へ切り替えたら消える
    assert not loop._rp_down_brake_blend_pending
    loop._rp_down_brake_blend_pending = True
    loop._enter_phase(_Phase.RAMP_UP, 2.0)  # 次の段の始まり
    assert not loop._rp_down_brake_blend_pending


def test_launch_blend_time_follows_the_target_g() -> None:
    """発進の寄せの時間は (G − 発進の G) ÷ ramp_launch_g_rate_gps、最短 ramp_launch_blend_min_s
    （2026-10-10 段2l-11。弱い段は 0.5 秒、0.35G で 2 秒）。rate 0 なら全段 ramp_turn_time_s。"""
    loop = _loop([_ramp(0.08)])
    for g, want in ((0.02, 0.5), (0.08, 0.5), (0.125, 0.5), (0.175, 0.125 / 0.15),
                    (0.25, 0.2 / 0.15), (0.35, 2.0)):
        assert loop._ramp_launch_blend_time_s(_ramp(g)) == pytest.approx(want)
    loop._config = dataclasses.replace(loop._config, ramp_launch_g_rate_gps=0.0)
    assert loop._ramp_launch_blend_time_s(_ramp(0.08)) == loop._config.ramp_turn_time_s


def test_release_blend_leftover_is_dropped_when_u_reaches_zero() -> None:
    """減速の頭の「アクセルを離す」寄せは、u が 0 以下になったら残りを捨てる（2026-10-10 段2l-11。
    実機 114602: 残りがブレーキ側へ持ち込まれ、0.3 秒でブレーキを 5% 踏んだ）。"""
    p = _ramp(0.125, top_kmh=120.0)
    loop = _loop([p])
    loop._enter_phase(_Phase.RAMP_HOLD, 0.0)
    loop._rp_u = 30.0
    loop._ramp_switch_down(0.0, 119.0)
    assert loop._rp_blend_release and loop._rp_blend_left == pytest.approx(-30.0)
    loop._rp_u = -0.2  # 偏差の項が先に u を 0 以下へ動かした
    loop._rp_ctl_t = 0.0
    loop._rp_blend_left = -5.0
    _cycle(loop, p, 119.0, 0.05)
    assert not loop._rp_blend_release
    assert loop._rp_blend_left == 0.0 or loop._rp_blend_left > -5.0 + 1e-9
    u1 = loop._rp_u
    _cycle(loop, p, 119.0, 0.10)
    # 次の周期に寄せの残り（15 %/秒 × 0.05 秒 ＝ 0.75%）ぶんの踏み増しが乗らない
    assert u1 - loop._rp_u < 0.5


def test_hold_at_the_speed_cap_is_not_cut_by_the_lookahead() -> None:
    """上端 ＝ 最高速の頭打ちの段でも、頭打ちそのものを超えない限り保持を続ける（2026-10-12
    段2l-11。実機 114602: 0.02G の段が「頭打ち − 加速度 × 先読み」で 0.3 秒で打ち切られた）。
    頭打ちを超え続けたら減速へ。"""
    p = _ramp(0.02, top_kmh=136.3)
    loop = _loop([p])
    loop._accel_speed_cap = 136.3
    t, v = 0.0, 134.0
    while loop._phase is _Phase.RAMP_UP:  # 0.02G ＝ 0.706 km/h/s で加速して保持へ
        _cycle(loop, p, v, t)
        t += 0.05
        v += 0.02 * G_TO_KMHS * 0.05
        assert t < 10.0
    assert loop._phase is _Phase.RAMP_HOLD and v < 136.3
    for _ in range(loop._skip_required + 5):  # まだ加速度が残っていても、頭打ち未満なら保持
        v = min(v + 0.02 * G_TO_KMHS * 0.05, 136.2)
        _cycle(loop, p, v, t)
        t += 0.05
        assert loop._phase is _Phase.RAMP_HOLD
    for _ in range(loop._skip_required):
        assert loop._phase is _Phase.RAMP_HOLD
        _cycle(loop, p, 136.6, t)
        t += 0.05
    assert loop._phase is _Phase.RAMP_DOWN



# ── 段2l-12: 片側調整（踏み増し側だけで調整し、実測 G が上端を超えたら戻す）──────────────

LEVELS = (0.02, 0.045, 0.08, 0.125)


def _ctl(loop: PatternLoop, p: plmod.GRampPattern, speed: float, t: float,
         accel_kmhs: float | None = None) -> None:
    """`_ramp_control` を 1 周期だけ呼ぶ（実開度は貯めた点のペダルの位置のまま）。"""
    adb, bdb = _deadbands(loop)
    a = loop._accel_actual_pct if loop._accel_actual_pct is not None else adb
    b = loop._brake_actual_pct if loop._brake_actual_pct is not None else bdb
    loop._ramp_control(p, speed, t, a, b, accel_kmhs)


def _one_sided_up(*, two_way: bool = False, g: float = 0.045, sens: float | None = 0.2,
                  u: float = 10.0, speed: float = 50.0) -> tuple[PatternLoop, plmod.GRampPattern]:
    """置き直しの後の RAMP_UP（アクセル側）。基準 ＝ 実車速、1 周期目を済ませた状態。"""
    p = _ramp(g, top_kmh=200.0)
    config = PatternLoopConfig(ramp_g_levels=LEVELS)
    loop = _loop([p], config=config, two_way=two_way)
    if sens is not None:
        _seed_sensitivity(loop, PEDAL_ACCEL, sens, speed)
    loop._rp_u = u
    loop._rp_vref_anchored = True
    loop._rp_blend_decided = True
    loop._rp_vref = speed
    _ctl(loop, p, speed, 0.0)  # dt ＝ 0 の最初の周期
    loop._rp_e_prev = 0.0
    return loop, p


def _one_sided_down(*, two_way: bool = False, g: float = 0.045, sens: float | None = 0.4,
                    u: float = -5.0, speed: float = 50.0) -> tuple[PatternLoop, plmod.GRampPattern]:
    """ブレーキ側に入った後の RAMP_DOWN（停止の直前より前）。基準 ＝ 実車速。"""
    p = _ramp(g, top_kmh=200.0)
    config = PatternLoopConfig(ramp_g_levels=LEVELS)
    loop = _loop([p], config=config, two_way=two_way)
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    if sens is not None:
        _seed_sensitivity(loop, PEDAL_BRAKE, sens, speed)
    loop._rp_u = u
    loop._rp_brake_latched = True
    loop._rp_blend_decided = True
    loop._rp_vref = speed
    _ctl(loop, p, speed, 0.0)
    loop._rp_e_prev = 0.0
    return loop, p


def test_ramp_upper_g_is_midpoint_to_the_next_stronger_level() -> None:
    loop = _loop([_ramp()], config=PatternLoopConfig(ramp_g_levels=LEVELS))
    assert loop._ramp_upper_g(_ramp(0.02)) == pytest.approx(0.0325)
    assert loop._ramp_upper_g(_ramp(0.045)) == pytest.approx(0.0625)
    assert loop._ramp_upper_g(_ramp(0.08)) == pytest.approx(0.1025)
    assert loop._ramp_upper_g(_ramp(0.125)) is None  # いちばん強い段
    assert _loop([_ramp()])._ramp_upper_g(_ramp(0.045)) is None  # 一覧が空


def test_one_sided_dir_by_phase() -> None:
    p = _ramp(0.045)
    loop, _ = _one_sided_up()
    assert loop._ramp_one_sided_dir(p) == 1
    loop._rp_vref_anchored = False  # 置き直しの前は両方向
    assert loop._ramp_one_sided_dir(p) is None
    loop._enter_phase(_Phase.RAMP_HOLD, 1.0)
    assert loop._ramp_one_sided_dir(p) is None
    loop, _ = _one_sided_down()
    assert loop._ramp_one_sided_dir(p) == -1
    v_stop = p.target_g * G_TO_KMHS * loop._config.ramp_stop_time_s / 2.0
    loop._rp_vref = v_stop - 0.01  # 停止の直前
    assert loop._ramp_one_sided_dir(p) is None
    loop._rp_vref = 0.0  # 基準が 0 km/h に着いた後
    assert loop._ramp_one_sided_dir(p) is None
    loop._config = dataclasses.replace(loop._config, ramp_stop_time_s=0.0)  # v_stop ＝ 0
    loop._rp_vref = 5.0
    assert loop._ramp_one_sided_dir(p) == -1
    loop._rp_vref = 0.0
    assert loop._ramp_one_sided_dir(p) is None
    loop._config = dataclasses.replace(loop._config, ramp_one_sided=False)
    loop._rp_vref = 50.0
    assert loop._ramp_one_sided_dir(p) is None


def test_one_sided_up_overshoot_keeps_u_and_moves_reference_to_speed() -> None:
    """加速で車が基準より速い: u は動かさず、基準を実車速に置き直す。両方向なら u を減らす。"""
    loop, p = _one_sided_up()
    u0 = loop._rp_u
    _ctl(loop, p, 52.0, 0.1)  # 基準はほぼ 50 のまま、車は 52
    assert loop._rp_u == pytest.approx(u0)
    assert loop._rp_vref == pytest.approx(52.0)
    assert loop._rp_e_prev == pytest.approx(0.0)
    two, p2 = _one_sided_up(two_way=True)
    _ctl(two, p2, 52.0, 0.1)
    assert two._rp_u < u0 - 0.1


def test_one_sided_down_overshoot_keeps_u_and_moves_reference_to_speed() -> None:
    """減速で車が基準より遅い: u は動かさず、基準を実車速に置き直す。"""
    loop, p = _one_sided_down()
    u0 = loop._rp_u
    _ctl(loop, p, 48.0, 0.1)
    assert loop._rp_u == pytest.approx(u0)
    assert loop._rp_vref == pytest.approx(48.0)
    two, p2 = _one_sided_down(two_way=True)
    _ctl(two, p2, 48.0, 0.1)
    assert two._rp_u > u0 + 0.01


def test_one_sided_forward_direction_moves_u_by_the_same_amount_as_two_way() -> None:
    for build, speed, sign in ((_one_sided_up, 49.0, 1.0), (_one_sided_down, 51.0, -1.0)):
        one, p = build()
        two, p2 = build(two_way=True)
        u0 = one._rp_u
        for i in range(1, 6):
            _ctl(one, p, speed, 0.1 * i)
            _ctl(two, p2, speed, 0.1 * i)
        assert sign * (one._rp_u - u0) > 0.0
        assert one._rp_u == pytest.approx(two._rp_u)


def test_one_sided_up_does_not_pull_back_while_deviation_shrinks() -> None:
    """遅れが縮む途中（Kp の項が負）でも戻さない。両方向では Kp × (e′ − e′_前) で戻す。"""
    loop, p = _one_sided_up()
    two, p2 = _one_sided_up(two_way=True)
    loop._rp_e_prev = two._rp_e_prev = 3.0  # 前周期は 3 km/h 遅れ → 今周期は縮んだ
    du_one = _delta_u(loop, p, 49.5)
    du_two = _delta_u(two, p2, 49.5)
    assert du_one >= 0.0
    assert du_two < du_one


def _delta_u(loop: PatternLoop, p: plmod.GRampPattern, speed: float) -> float:
    u0 = loop._rp_u
    _ctl(loop, p, speed, 0.1)
    return loop._rp_u - u0


def test_one_sided_up_returns_when_measured_g_exceeds_the_upper_edge() -> None:
    """実測 G が上端（0.045G → 0.0625G）を超えたら、Kp ×（実測の加速度 − 狙い）× dt だけ逆へ。"""
    loop, p = _one_sided_up()
    kp_inv = 0.2 * loop._config.ramp_settle_time_s  # Kp ＝ 1 ÷（感度 × T）
    a = 0.1 * G_TO_KMHS
    u0 = loop._rp_u
    _ctl(loop, p, 49.5, 0.1, accel_kmhs=a)  # 車は遅れていても（順方向でも）戻す
    expected = -(a - 0.045 * G_TO_KMHS) * 0.1 / kp_inv
    assert loop._rp_u - u0 == pytest.approx(expected, rel=1e-6)
    # 上端を下回れば戻さない（順方向の動きに戻る）
    u1 = loop._rp_u
    _ctl(loop, p, 49.5, 0.2, accel_kmhs=0.05 * G_TO_KMHS)
    assert loop._rp_u >= u1


def test_one_sided_down_returns_toward_release_when_decel_exceeds_the_upper_edge() -> None:
    loop, p = _one_sided_down()
    kp_inv = 0.4 * loop._config.ramp_settle_time_s
    a = -0.1 * G_TO_KMHS  # 減速度 0.1G ＞ 上端 0.0625G
    u0 = loop._rp_u
    _ctl(loop, p, 50.5, 0.1, accel_kmhs=a)
    expected = (0.1 - 0.045) * G_TO_KMHS * 0.1 / kp_inv  # ブレーキを緩める（u が増える）
    assert loop._rp_u - u0 == pytest.approx(expected, rel=1e-6)
    # 減速度が上端を下回れば止まる
    u1 = loop._rp_u
    _ctl(loop, p, 50.5, 0.2, accel_kmhs=-0.05 * G_TO_KMHS)
    assert loop._rp_u <= u1


def test_one_sided_return_without_sensitivity_uses_one_min_step() -> None:
    loop, p = _one_sided_up(sens=None)
    u0 = loop._rp_u
    _ctl(loop, p, 49.5, 0.1, accel_kmhs=0.1 * G_TO_KMHS)
    assert loop._rp_u - u0 == pytest.approx(-_min_step(loop))
    down, pd = _one_sided_down(sens=None)
    u0 = down._rp_u
    _ctl(down, pd, 50.5, 0.1, accel_kmhs=-0.1 * G_TO_KMHS)
    assert down._rp_u - u0 == pytest.approx(
        down._ramp_min_step_pct(True)
    )


def test_one_sided_does_not_return_for_the_strongest_level_or_unmeasured_g() -> None:
    loop, p = _one_sided_up(g=0.125)  # いちばん強い段は上端なし
    u0 = loop._rp_u
    _ctl(loop, p, 49.5, 0.1, accel_kmhs=0.5 * G_TO_KMHS)
    assert loop._rp_u >= u0
    loop, p = _one_sided_up()
    u0 = loop._rp_u
    _ctl(loop, p, 49.5, 0.1, accel_kmhs=None)  # 実測の加速度が測れていない
    assert loop._rp_u >= u0


def test_hold_and_launch_before_anchor_stay_two_way() -> None:
    # RAMP_HOLD: 車が基準より速ければ u を減らす
    p = _ramp(0.045, top_kmh=200.0)
    loop = _loop([p], config=PatternLoopConfig(ramp_g_levels=LEVELS))
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 50.0)
    loop._enter_phase(_Phase.RAMP_HOLD, 0.0)
    loop._rp_u = 10.0
    loop._rp_blend_decided = True
    loop._rp_vref = 50.0
    _ctl(loop, p, 50.0, 0.0)
    loop._rp_e_prev = 0.0
    u0 = loop._rp_u
    _ctl(loop, p, 52.0, 0.1)
    assert loop._rp_u < u0 - 0.1
    # 置き直しの前の RAMP_UP: 基準が先へ進むのでブレーキを離す向き（両方向の式のまま）
    loop2 = _loop([p], launched=False, config=PatternLoopConfig(ramp_g_levels=LEVELS))
    _seed_sensitivity(loop2, PEDAL_BRAKE, 0.4, 1.0)
    loop2._rp_u = -10.0
    loop2._rp_blend_decided = True
    assert loop2._ramp_one_sided_dir(p) is None


def test_one_sided_near_stop_keeps_the_two_way_step_floor() -> None:
    """停止の直前（基準が 0 km/h に着いた後）は今のまま: 車が基準より速ければ 1 周期に最小刻み
    以上ブレーキを踏み増す。"""
    p = _ramp(0.045, top_kmh=200.0)
    loop = _loop([p], config=PatternLoopConfig(ramp_g_levels=LEVELS))
    loop._enter_phase(_Phase.RAMP_DOWN, 0.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 0.4, 5.0)
    loop._rp_u = -5.0
    loop._rp_brake_latched = True
    loop._rp_blend_decided = True
    loop._rp_vref = 0.0
    _ctl(loop, p, 3.0, 0.0)
    loop._rp_e_prev = -3.0
    u0 = loop._rp_u
    _ctl(loop, p, 3.0, 0.1)
    assert loop._rp_u - u0 <= -loop._ramp_min_step_pct(True) + 1e-9


# ───────── 段2l-13（2026-10-12）: 発進の寄せの間は基準を実車速に合わせる ─────────


def test_reference_follows_actual_speed_during_the_launch_blend() -> None:
    """発進の寄せの間は v_ref ＝ 実車速（偏差の項 0）。u の増分は寄せの分だけ。"""
    p = _ramp(0.08)
    loop = _loop([p])
    _seed_accel_points(loop)
    loop._rp_vref_anchored = True
    loop._rp_launch_started = 0.0
    gap = loop._ramp_estimate_u(0.0, 0.08)
    assert gap is not None and gap > 3.0
    dt = 0.1
    per_cycle = gap * dt / loop._ramp_launch_blend_time_s(p)
    _cycle(loop, p, 1.0, 0.0)  # 最初の周期に寄せを決める（dt ＝ 0 なので u は動かない）
    assert loop._rp_blend_left > 0.0
    prev = loop._rp_u
    for i in range(1, 4):  # 寄せ 0.5 秒の途中。車は基準よりずっと遅い（1 km/h のまま）
        _cycle(loop, p, 1.0, dt * i)
        assert loop._rp_vref == pytest.approx(1.0)
        assert loop._rp_u - prev == pytest.approx(per_cycle)
        prev = loop._rp_u


def test_reference_advances_from_actual_speed_after_the_launch_blend() -> None:
    """寄せが終わった周期から、基準は実車速から進む（寄せの間のぶんを持ち越さない）。"""
    p = _ramp(0.08)
    loop = _loop([p])
    _seed_accel_points(loop)
    loop._rp_vref_anchored = True
    loop._rp_launch_started = 0.0
    dt = 0.1
    i = 0
    _cycle(loop, p, 1.0, 0.0)
    while loop._rp_blend_left != 0.0:
        i += 1
        _cycle(loop, p, 1.0, dt * i)
        assert i < 20
    assert loop._rp_vref == pytest.approx(1.0)  # 寄せの最後の周期まで実車速
    _cycle(loop, p, 1.0, dt * (i + 1))
    assert 1.0 < loop._rp_vref <= 1.0 + 0.08 * plmod.G_TO_KMHS * dt + 1e-9


def test_reference_is_not_held_when_there_is_no_launch_blend() -> None:
    """表が引けず寄せが無い段は今までどおり基準が進む。"""
    p = _ramp(0.08)
    loop = _loop([p])
    loop._rp_vref_anchored = True
    loop._rp_launch_started = 0.0
    _cycle(loop, p, 1.0, 0.0)
    loop._rp_vref = 1.0
    loop._rp_e_prev = 0.0
    _cycle(loop, p, 1.0, 0.1)
    assert loop._rp_blend_left == 0.0
    assert loop._rp_vref > 1.0


# ── 途中の定速保持（RAMP_CRUISE。2026-10-13 段2o-2）──────────────────────


def _cruise_ramp(
    g: float = 0.1, top_kmh: float = 40.0, stations: tuple[float, ...] = (15.0, 25.0),
    up_only: bool = False,
) -> plmod.GRampPattern:
    return plmod.GRampPattern(
        PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.0,
        target_g=g, top_kmh=top_kmh, stations_kmh=stations,
        stations_up_only=up_only,
    )


class _Kine:
    """区間で車速を決める簡単な車: RAMP_UP は +G・RAMP_DOWN は −G で変わり、保持（RAMP_CRUISE・
    RAMP_HOLD）は一定。`drift` を渡すと保持中も車速をその速さ [km/h/s] で動かす
    （落ち着かない車）。"""

    def __init__(self, loop: PatternLoop, g: float, v0: float = 5.0, drift: float = 0.0) -> None:
        self.loop, self.g, self.v, self.drift = loop, g, v0, drift

    def step(self, dt: float = 0.1) -> None:
        ph = self.loop._phase
        if ph is _Phase.RAMP_UP:
            self.v += self.g * G_TO_KMHS * dt
        elif ph is _Phase.RAMP_DOWN:
            self.v = max(0.0, self.v - self.g * G_TO_KMHS * dt)
        elif ph is _Phase.RAMP_CRUISE:
            self.v += self.drift * dt


def _run_kine(
    loop: PatternLoop, p: plmod.GRampPattern, kine: _Kine, *, t_max: float = 200.0,
    stop_when: Callable[[PatternLoop], bool] | None = None,
) -> tuple[float, list[tuple[float, str, float]]]:
    """結果が記録される（段の終わり）か stop_when まで回す。
    戻り値は (終わりの時刻, [(t, phase 名, 車速)])。"""
    rows: list[tuple[float, str, float]] = []
    t = 0.0
    while t < t_max and not loop.ramp_results:
        _cycle(loop, p, kine.v, t)
        rows.append((t, loop._phase.name, kine.v))
        if stop_when is not None and stop_when(loop):
            break
        kine.step()
        t += 0.1
    return t, rows


def _segments(rows: list[tuple[float, str, float]], name: str) -> list[list[tuple[float, float]]]:
    """phase 名が name の連続区間ごとの [(t, 車速)]。"""
    out: list[list[tuple[float, float]]] = []
    prev = ""
    for t, ph, v in rows:
        if ph == name:
            if prev != name:
                out.append([])
            out[-1].append((t, v))
        prev = ph
    return out


def test_cruise_without_stations_is_the_old_behaviour() -> None:
    """stations_kmh が空なら RAMP_CRUISE に入らず、結果の保持の列は 0。"""
    p = _cruise_ramp(stations=())
    loop = _loop([p])
    _, rows = _run_kine(loop, p, _Kine(loop, 0.1))
    assert not any(ph == "RAMP_CRUISE" for _, ph, _ in rows)
    r = loop.ramp_results[0]
    assert (r.cruise_s, r.cruise_n, r.cruise_done) == (0.0, 0, 0)


def test_cruise_enters_up_and_down_at_each_station_and_resumes() -> None:
    """上り下りの各点で 1 回ずつ入り、実測の加速度が帯に入った時間が hold_s に達して出る。
    上りは `_ramp_up_begin` を通さずに RAMP_UP へ戻る。結果の秒数は積算で、合計が経過と合う。"""
    p = _cruise_ramp()
    loop = _loop([p])
    begins: list[int] = []
    orig = loop._ramp_up_begin
    loop._ramp_up_begin = lambda: (begins.append(1), orig())[1]  # type: ignore[method-assign]
    cfg = loop._config
    t_end, rows = _run_kine(loop, p, _Kine(loop, 0.1))
    segs = _segments(rows, "RAMP_CRUISE")
    assert len(segs) == 4  # 15・25 の上りと下り
    # 保持の車速は点の近く（上りは手前から、下りは点を越えて入る）で、保持中は一定
    for seg, station in zip(segs, (15.0, 25.0, 25.0, 15.0), strict=True):
        assert abs(seg[0][1] - station) <= 4.0
        assert seg[0][1] == seg[-1][1]
        length = seg[-1][0] - seg[0][0] + 0.1
        assert cfg.ramp_cruise_hold_s <= length <= cfg.ramp_cruise_hold_s + 2.0
    assert begins == []  # 上りへ戻るとき状態を作り直さない（初期化時の 1 回は loop 作成前）
    r = loop.ramp_results[0]
    assert (r.cruise_n, r.cruise_done) == (4, 4)
    assert r.cruise_s == pytest.approx(sum(len(s) * 0.1 for s in segs), abs=0.3)
    # 差し引きでなく積算: 上り・保持・下り・途中の保持の合計が、段の始めから終わりまでと合う
    assert r.up_s + r.hold_s + r.down_s + r.cruise_s == pytest.approx(t_end, abs=0.3)
    assert r.up_s == pytest.approx(
        sum(1 for _, ph, _ in rows if ph == "RAMP_UP") * 0.1, abs=0.3
    )


def test_cruise_up_only_enters_on_the_way_up_and_never_on_the_way_down() -> None:
    """2026-10-14 段2o-4: stations_up_only=True は上りだけ保持に入り、下りでは入らない。"""
    p = _cruise_ramp(up_only=True)
    loop = _loop([p])
    _, rows = _run_kine(loop, p, _Kine(loop, 0.1))
    segs = _segments(rows, "RAMP_CRUISE")
    assert len(segs) == 2  # 15・25 の上りだけ
    first_down = next(i for i, (_, ph, _) in enumerate(rows) if ph == "RAMP_DOWN")
    assert not any(ph == "RAMP_CRUISE" for _, ph, _ in rows[first_down:])
    assert loop.ramp_results[0].cruise_n == 2


def test_cruise_with_a_single_station_at_low_g_returns_to_the_target_g() -> None:
    """狙いの G が 0.125 で保持点 1 つ（上りだけ）: 入り、±band が hold_s に達して RAMP_UP に戻り、
    その後 g_ref は狙いの G に戻る。"""
    p = _cruise_ramp(g=0.125, top_kmh=40.0, stations=(20.0,), up_only=True)
    loop = _loop([p])
    kine = _Kine(loop, 0.125)
    t, _ = _run_kine(loop, p, kine, stop_when=lambda lp: lp._phase is _Phase.RAMP_CRUISE)
    assert loop._phase is _Phase.RAMP_CRUISE
    while loop._phase is _Phase.RAMP_CRUISE:
        t += 0.1
        _cycle(loop, p, kine.v, t)
        kine.step()
    assert loop._phase is _Phase.RAMP_UP
    assert loop._rp_cruise_done_n == 1
    resume = loop._rp_resume_started
    assert resume is not None
    turn = loop._config.ramp_turn_time_s
    assert loop._ramp_gref(p, resume + turn) == pytest.approx(0.125)
    assert loop._ramp_gref(p, resume + 10.0) == pytest.approx(0.125)


def test_cruise_up_resume_ramps_gref_from_zero_and_keeps_vref_continuous() -> None:
    p = _cruise_ramp(stations=(15.0,))
    loop = _loop([p])
    kine = _Kine(loop, 0.1)
    t, _ = _run_kine(loop, p, kine, stop_when=lambda lp: lp._phase is _Phase.RAMP_CRUISE)
    # 保持の間、g_ref は入った時点の実測 → 0（その後 0）
    while loop._phase is _Phase.RAMP_CRUISE:
        t += 0.1
        _cycle(loop, p, kine.v, t)
    assert loop._phase is _Phase.RAMP_UP and loop._rp_resume_started is not None
    resume = loop._rp_resume_started
    assert loop._rp_vref == kine.v and loop._rp_e_prev == 0.0
    turn = loop._config.ramp_turn_time_s
    assert loop._ramp_gref(p, resume) == pytest.approx(0.0)
    assert loop._ramp_gref(p, resume + turn / 2) == pytest.approx(0.05)
    assert loop._ramp_gref(p, resume + turn) == pytest.approx(0.1)
    assert loop._ramp_gref(p, resume + 10.0) == pytest.approx(0.1)


def test_cruise_times_out_when_speed_never_settles_and_is_not_retried() -> None:
    """車速が落ち着かない（±0.005G の帯に入らない）と ramp_turn_time_s ＋ timeout_s で打ち切り。
    その点は保持済みで、同じ上りでは入り直さない。"""
    p = _cruise_ramp(stations=(15.0,))
    loop = _loop([p])
    cfg = loop._config
    kine = _Kine(loop, 0.1, drift=1.0)  # 1 km/h/s ≒ 0.028G で動き続ける
    _, rows = _run_kine(loop, p, kine)
    segs = _segments(rows, "RAMP_CRUISE")
    assert len(segs) == 2  # 上りと下りで 1 回ずつ（打ち切った点を上りで入り直さない）
    for seg in segs:
        length = seg[-1][0] - seg[0][0] + 0.1
        assert length == pytest.approx(cfg.ramp_turn_time_s + cfg.ramp_cruise_timeout_s, abs=0.3)
    r = loop.ramp_results[0]
    assert (r.cruise_n, r.cruise_done) == (2, 0)


def test_cruise_does_not_count_cycles_without_a_measured_acceleration() -> None:
    """加速度が測れない周期（None）で落ちず、数えない。"""
    p = _cruise_ramp(stations=(15.0,))
    loop = _loop([p])
    kine = _Kine(loop, 0.1)
    t, _ = _run_kine(loop, p, kine, stop_when=lambda lp: lp._phase is _Phase.RAMP_CRUISE)
    loop._rp_cruise_in_band_s = 0.0
    loop._advance_ramp_cruise(p, kine.v, t + 0.1, None)
    assert loop._rp_cruise_in_band_s == 0.0 and loop._phase is _Phase.RAMP_CRUISE


def test_cruise_entered_on_the_brake_keeps_the_brake_and_exits_to_down() -> None:
    """下りでブレーキ（u ＜ 0）を踏んだまま入っても、下限が今の u なので 1 周期でブレーキが
    抜けない。出たら RAMP_DOWN へ（ブレーキ側のラッチを付け直す）。_rp_down_started は
    最初の 1 回。"""
    p = _cruise_ramp(stations=(15.0,))
    loop = _loop([p])
    kine = _Kine(loop, 0.1)
    t, _ = _run_kine(loop, p, kine, stop_when=lambda lp: lp._phase is _Phase.RAMP_DOWN)
    down_started = loop._rp_down_started
    loop._rp_u = -5.0
    loop._rp_brake_latched = True
    assert loop._rp_accel_reached
    while loop._phase is _Phase.RAMP_DOWN:
        _cycle(loop, p, kine.v, t)
        kine.step()
        t += 0.1
        loop._rp_u = min(loop._rp_u, -5.0)  # 偏差の項に左右されないよう、ブレーキを保つ
    assert loop._phase is _Phase.RAMP_CRUISE and loop._rp_cruise_dir < 0
    lo, hi = loop._ramp_u_limits()
    assert lo == pytest.approx(min(0.0, loop._rp_u)) and hi > 0.0
    _, bdb = _deadbands(loop)
    a, b = _cycle(loop, p, kine.v, t)
    assert a == 0.0 and b >= bdb + 4.0  # ブレーキが抜けていない
    # 落ち着かせて出る: RAMP_DOWN に戻り、最初の記録は残る
    while loop._phase is _Phase.RAMP_CRUISE:
        t += 0.1
        _cycle(loop, p, kine.v, t)
    assert loop._phase is _Phase.RAMP_DOWN and loop._rp_down_started == down_started
    assert loop._rp_brake_latched == (loop._rp_u < 0.0)


def test_cruise_timeouts_count_the_leg_without_the_cruise_time() -> None:
    """時間切れは保持を除いた通算。保持の分長くかかっても、上り・下りが打ち切られない。"""
    p = _cruise_ramp(g=0.1, top_kmh=40.0, stations=(15.0, 25.0))
    loop = _loop([p])
    kine = _Kine(loop, 0.1)
    _, rows = _run_kine(loop, p, kine)
    # 上り・下りの時間切れ（見込み × 3 + 10 s）より、保持を含む区間の長さのほうが短い設定でも、
    # 区間の経過は保持を除く: 打ち切りで終わらず、停車（速度 0）まで走って結果が残る
    assert len(loop.ramp_results) == 1
    assert kine.v == pytest.approx(0.0, abs=0.5)


# ── 遅れの測定のステップ（HOLD_STEP。2026-10-13 段2o-2）──────────────────


def _lag_ramp(g: float = 0.125, top_kmh: float = 30.0) -> plmod.GRampPattern:
    return plmod.GRampPattern(
        PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.0,
        target_g=g, top_kmh=top_kmh, lag_step=True,
    )


def _run_lag(
    loop: PatternLoop, p: plmod.GRampPattern, t_max: float = 120.0
) -> tuple[list[tuple[float, str, float, float, float]], float]:
    """保持の間と HOLD_STEP の間は車速一定の車で回す。[(t, phase 名, 車速, アクセル指令, ブレーキ
    指令)] と終わりの時刻を返す。"""
    kine = _Kine(loop, p.target_g, v0=5.0)
    rows: list[tuple[float, str, float, float, float]] = []
    t = 0.0
    while t < t_max and not loop.ramp_results:
        a, b = _cycle(loop, p, kine.v, t)
        rows.append((t, loop._phase.name, kine.v, a, b))
        kine.step()
        t += 0.1
    return rows, t


def test_lag_step_runs_five_fixed_openings_after_the_top_hold() -> None:
    p = _lag_ramp()
    loop = _loop([p])
    _seed_plant_table(loop)
    rows, _ = _run_lag(loop, p)
    names = [ph for _, ph, _, _, _ in rows]
    assert "HOLD_STEP" in names
    # RAMP_HOLD は途中の保持と同じ出方（合計 7 秒の定速）を経て HOLD_STEP、その後 RAMP_DOWN
    assert names.index("RAMP_HOLD") < names.index("HOLD_STEP") < names.index("RAMP_DOWN")
    step = [(a, b) for _, ph, _, a, b in rows if ph == "HOLD_STEP"]
    hold_s = loop._config.ramp_lag_step_hold_s
    n_per = round(hold_s / 0.1)
    # 開度は 1 つにつき ramp_lag_step_hold_s 秒、5 つ（先頭は phase が変わった周期の指令が前の
    # 開度なので、5 つ目の最後で数える）
    plateaus: list[tuple[float, float]] = []
    for pair in step:
        if not plateaus or plateaus[-1] != pair:
            plateaus.append(pair)
    assert 5 <= len(plateaus) <= 6  # 先頭に保持の開度が 1 周期だけ残る
    five = plateaus[-5:]
    (a1, b1), (a2, b2), (a3, b3), (a4, b4), (a5, b5) = five
    assert a1 > a2 > 0.0 and b1 == b2 == 0.0  # 1: 踏み増し  2: 保持の開度
    assert a3 == 0.0 and b3 == 0.0  # 3: 両ペダルを離す（待機位置）
    assert a4 == 0.0 and b4 > 0.0  # 4: ブレーキ踏み増し
    assert a5 == 0.0 and b5 == 0.0  # 5: 戻し
    assert len(step) == pytest.approx(5 * n_per, abs=3)
    # 終わったら RAMP_DOWN へ。結果の保持秒には HOLD_STEP が入り、G 中央値には入らない
    assert loop.ramp_results[0].hold_s >= 5 * hold_s


def test_lag_step_is_skipped_when_no_step_size_is_available() -> None:
    """表も感度も無いときは HOLD_STEP を作らず、そのまま RAMP_DOWN へ。"""
    p = _lag_ramp()
    loop = _loop([p])
    rows, _ = _run_lag(loop, p)
    names = [ph for _, ph, _, _, _ in rows]
    assert "HOLD_STEP" not in names and "RAMP_DOWN" in names


def test_lag_step_skips_brake_steps_when_brake_size_unknown() -> None:
    p = _lag_ramp()
    loop = _loop([p])
    adb, _ = _deadbands(loop)
    for v in range(5, 60, 10):  # アクセルの表だけ（ブレーキは点も感度も無い）
        loop.glimit.add_coast(float(v), 1.0)
        for u in (2.0, 5.0, 10.0, 15.0, 20.0):
            loop.glimit.add(PEDAL_ACCEL, float(v), adb + u, -1.0 + 0.2 * u)
    loop._accel_actual_pct = adb + 10.0
    targets = loop._ramp_step_targets(30.0, 3.0)
    assert targets is not None and len(targets) == 3  # 踏み増し・戻し・離す だけ
    assert targets[1] == 3.0 and targets[2] == 0.0 and targets[0] > 3.0


def test_lag_step_targets_use_table_then_sensitivity() -> None:
    p = _lag_ramp()
    loop = _loop([p])
    _seed_plant_table(loop)
    kmhs = loop._config.ramp_lag_step_g * G_TO_KMHS
    t = loop._ramp_step_targets(30.0, 4.0)
    assert t is not None and len(t) == 5
    # 表（惰行 1.0・アクセル ga=0.2・ブレーキ gb=0.4 の車）: +0.05G は 惰行 1.0 を打ち消す分も
    # 要るので、踏み増しの開度は u_hold より大きい。ブレーキは「減速の合計が 惰行 ＋ 0.05G になる
    # 開度」（表のブレーキの点は惰行を含む合計）で 0.05G ÷ 0.4
    assert t[0] > 4.0 and t[1] == 4.0 and t[2] == 0.0 and t[4] == 0.0
    assert t[3] < 0.0 and abs(t[3]) == pytest.approx(kmhs / 0.4, abs=0.5)


def test_hold_step_is_part_of_the_ramp_phase_set_and_names_match_pedal_lag() -> None:
    from tests.research import pedal_lag

    assert _Phase.HOLD_STEP.name == pedal_lag.PHASE_HOLD_STEP
    assert _Phase.HOLD_STEP in plmod._RAMP_PHASES and _Phase.RAMP_CRUISE in plmod._RAMP_PHASES


# ── 下りの戻し（RAMP_RELEASE。2026-10-14 段2o-5）──────────────────────────


def _release_ramp(
    stations: tuple[float, ...] = (25.0,), g: float = 0.1, top_kmh: float = 40.0
) -> plmod.GRampPattern:
    return plmod.GRampPattern(
        PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.0,
        target_g=g, top_kmh=top_kmh, release_stations=stations,
    )


def _release_config(**kw: object) -> PatternLoopConfig:
    base = PatternLoopConfig(
        ramp_release_fracs=(0.5, 0.0), ramp_release_move_s=1.0, ramp_release_hold_s=(1.5, 2.0),
        ramp_release_min_kmh=10.0,
    )
    return dataclasses.replace(base, **kw)  # type: ignore[arg-type]


class _ReleaseKine(_Kine):
    """RAMP_RELEASE の間は −G の 0.4 倍で減速する車（戻すと減速が弱まる）。"""

    def step(self, dt: float = 0.1) -> None:
        if self.loop._phase is _Phase.RAMP_RELEASE:
            self.v = max(0.0, self.v - 0.4 * self.g * G_TO_KMHS * dt)
        else:
            super().step(dt)


def _run_release(
    loop: PatternLoop, p: plmod.GRampPattern, kine: _Kine, *, u_down: float = -6.0,
    stop_when: Callable[[PatternLoop], bool] | None = None, t_max: float = 120.0,
) -> tuple[float, list[tuple[float, str, float, float]]]:
    """RAMP_DOWN の間は u を u_down 以下（ブレーキ側）に保って回す。
    戻り値は (終わりの時刻, [(t, phase 名, 車速, u)])。結果が記録されるか stop_when で止める。"""
    rows: list[tuple[float, str, float, float]] = []
    t = 0.0
    while t < t_max and not loop.ramp_results:
        if loop._phase is _Phase.RAMP_DOWN and kine.v > 0.0:
            loop._rp_u = min(loop._rp_u, u_down)
            loop._rp_brake_latched = True
        _cycle(loop, p, kine.v, t)
        rows.append((t, loop._phase.name, kine.v, loop._rp_u))
        if stop_when is not None and stop_when(loop):
            break
        kine.step()
        t += 0.1
    return t, rows


def test_release_without_stations_is_the_old_behaviour() -> None:
    """release_stations が空（または fracs が空）なら RAMP_RELEASE に入らず、結果の戻しは 0。"""
    for stations, cfg in (((), _release_config()), ((25.0,), PatternLoopConfig())):
        p = _release_ramp(stations=stations)
        loop = _loop([p], config=cfg)
        _, rows = _run_release(loop, p, _ReleaseKine(loop, 0.1))
        assert not any(ph == "RAMP_RELEASE" for _, ph, _, _ in rows)
        r = loop.ramp_results[0]
        assert (r.release_s, r.release_n, r.release_skipped) == (0.0, 0, 0)


def test_release_enters_at_the_station_and_moves_u_linearly() -> None:
    """戻す車速に着くと RAMP_RELEASE に入り、u が u_peak → 0.5 u_peak → 0 と直線で動く
    （1 秒で移動・1.5 秒保持・1 秒で移動・2 秒保持）。指令はアクセル 0・ブレーキは min まで。"""
    p = _release_ramp()
    loop = _loop([p], config=_release_config())
    kine = _ReleaseKine(loop, 0.1)
    _, rows = _run_release(loop, p, kine)
    seg = [(t, u, v) for t, ph, v, u in rows if ph == "RAMP_RELEASE"]
    assert seg, "RAMP_RELEASE に入っていない"
    t0, u_peak = seg[0][0], seg[0][1]
    assert u_peak == pytest.approx(-6.0, abs=0.5) and seg[0][2] <= 25.0
    for t, u, _v in seg:
        dt = t - t0
        if dt < 1.0:
            want = u_peak * (1.0 - 0.5 * dt)
        elif dt < 2.5:
            want = 0.5 * u_peak
        elif dt < 3.5:
            want = 0.5 * u_peak * (1.0 - (dt - 2.5))
        else:
            want = 0.0
        assert u == pytest.approx(want, abs=0.05 * abs(u_peak) + 1e-6)
    assert seg[-1][1] == pytest.approx(0.0, abs=0.05 * abs(u_peak) + 1e-6)
    assert seg[-1][0] - t0 == pytest.approx(5.4, abs=0.2)  # 1 + 1.5 + 1 + 2 秒
    r = loop.ramp_results[0]
    assert (r.release_n, r.release_skipped) == (1, 0)
    assert r.release_s == pytest.approx(5.5, abs=0.3)


def test_release_standby_command_is_brake_min_and_accel_zero() -> None:
    """流れの最後（u ＝ 0）の指令はアクセル 0・ブレーキ min（0% へ飛ばさない）。"""
    p = _release_ramp()
    loop = _loop([p], config=_release_config())
    kine = _ReleaseKine(loop, 0.1)
    _, bdb = _deadbands(loop)
    t = 0.0
    last: tuple[float, float] = (0.0, 0.0)
    while t < 100.0 and not loop.ramp_results:
        if loop._phase is _Phase.RAMP_DOWN:
            loop._rp_u = min(loop._rp_u, -6.0)
            loop._rp_brake_latched = True
        last = _cycle(loop, p, kine.v, t)
        if loop._phase is _Phase.RAMP_RELEASE and loop._rp_u == 0.0:
            break
        kine.step()
        t += 0.1
    assert loop._phase is _Phase.RAMP_RELEASE and loop._rp_u == 0.0
    a, b = _cycle(loop, p, kine.v, t + 0.1)
    assert a == 0.0 and b == pytest.approx(bdb)
    assert last[0] == 0.0
    assert loop._ramp_on_brake()
    lo, hi = loop._ramp_u_limits()
    assert hi == 0.0 and lo < 0.0


def test_release_does_not_feed_glimit_or_down_g_median() -> None:
    """戻しの間は glimit に点を足さず、`_rp_down_g` にも入れない（制御も呼ばない）。"""
    p = _release_ramp()
    loop = _loop([p], config=_release_config())
    _, bdb = _deadbands(loop)
    added: list[str] = []
    orig_add = loop.glimit.add

    def spy(pedal: str, speed: float, opening: float, accel: float) -> None:
        added.append(loop._phase.name)
        orig_add(pedal, speed, opening, accel)

    loop.glimit.add = spy  # type: ignore[method-assign]
    controls: list[str] = []
    orig_ctl = loop._ramp_control

    def ctl(*args: object, **kw: object) -> None:
        controls.append(loop._phase.name)
        orig_ctl(*args, **kw)  # type: ignore[arg-type]

    loop._ramp_control = ctl  # type: ignore[method-assign]
    kine = _ReleaseKine(loop, 0.1)
    down_len_at_entry: list[int] = []
    t = 0.0
    while t < 120.0 and not loop.ramp_results:
        loop._brake_actual_pct = bdb + 6.0
        loop._accel_actual_pct = 0.0
        if loop._phase is _Phase.RAMP_DOWN:
            loop._rp_u = min(loop._rp_u, -6.0)
            loop._rp_brake_latched = True
        was = loop._phase
        _cycle(loop, p, kine.v, t)
        if was is _Phase.RAMP_DOWN and loop._phase is _Phase.RAMP_RELEASE:
            down_len_at_entry.append(len(loop._rp_down_g))
        kine.step()
        t += 0.1
    assert down_len_at_entry
    assert "RAMP_RELEASE" not in added and "RAMP_RELEASE" not in controls
    assert "RAMP_DOWN" in added  # 戻しの外では貯めている（テストの前提）
    assert loop.ramp_results[0].release_n == 1


def test_release_exit_returns_to_down_with_gref_from_zero_and_vref_at_speed() -> None:
    """出た後は RAMP_DOWN。g_ref は 0 → −G を ramp_turn_time_s で直線、v_ref ＝ 実車速、
    前の偏差 ＝ 0、u は待機位置から。下りの時間切れから戻しの時間が除かれる。"""
    p = _release_ramp()
    loop = _loop([p], config=_release_config())
    kine = _ReleaseKine(loop, 0.1)
    t, _ = _run_release(
        loop, p, kine,
        stop_when=lambda lp: lp._phase is _Phase.RAMP_RELEASE,
    )
    prior_before = loop._rp_down_prior_s
    assert prior_before > 0.0  # 戻しに入るまでの下りの時間が足してある
    # 戻しを最後まで回す
    while loop._phase is _Phase.RAMP_RELEASE:
        kine.step()
        t += 0.1
        _cycle(loop, p, kine.v, t)
    assert loop._phase is _Phase.RAMP_DOWN
    assert loop._rp_vref == kine.v and loop._rp_e_prev == 0.0
    assert loop._rp_turn_g0 == 0.0 and loop._rp_turn_started == pytest.approx(t)
    assert loop._rp_u == 0.0 and loop._rp_brake_latched
    assert loop._ramp_gref(p, t) == pytest.approx(0.0)
    turn = loop._config.ramp_turn_time_s
    assert loop._ramp_gref(p, t + turn / 2.0) == pytest.approx(-0.05, abs=0.005)
    assert loop._ramp_gref(p, t + turn) == pytest.approx(-0.1, abs=0.005)
    assert loop._rp_down_prior_s == prior_before  # 戻しの時間は足さない
    assert loop._phase_started_at == pytest.approx(t)  # 経過は 0 から（戻しを含まない）
    # 2026-10-14 段2o-5: 踏み直しは u_peak へ ramp_turn_time_s で寄せる（深くする向き）
    assert loop._rp_blend_left == pytest.approx(loop._rp_release_u_peak)
    assert loop._rp_blend_time_left == pytest.approx(turn)
    # 踏み直し: 片側調整で u がブレーキ側へ動き始める（アクセル側へは行かない）
    for _ in range(40):
        kine.step()
        t += 0.1
        _cycle(loop, p, kine.v, t)
        assert loop._rp_u <= 0.0
    assert loop._rp_u < 0.0


def test_release_is_skipped_when_u_is_not_on_the_brake() -> None:
    """着いた点で u ≥ 0（惰行・アクセルで −G が出ている）なら飛ばし、数える。"""
    p = _release_ramp(stations=(25.0, 15.0))
    loop = _loop([p], config=_release_config())
    loop._ramp_control = lambda *a, **k: None  # type: ignore[method-assign, assignment]
    kine = _ReleaseKine(loop, 0.1)
    t = 0.0
    while t < 120.0 and not loop.ramp_results:
        if loop._phase is _Phase.RAMP_DOWN:
            loop._rp_u = max(loop._rp_u, 0.0)  # アクセル側（ブレーキを踏んでいない）
            loop._rp_brake_latched = False
        _cycle(loop, p, kine.v, t)
        kine.step()
        t += 0.1
    r = loop.ramp_results[0]
    assert (r.release_n, r.release_skipped) == (0, 2)
    assert r.release_s == 0.0


def test_release_is_cut_off_in_the_creep_range_and_returns_to_down() -> None:
    """流れの途中で実車速が ramp_release_min_kmh 以下になったら RAMP_DOWN へ戻す。
    着いた時点で既に安全網より下なら入らず飛ばす。"""
    p = _release_ramp(stations=(14.0,))
    loop = _loop([p], config=_release_config())
    kine = _ReleaseKine(loop, 0.1)
    t, rows = _run_release(
        loop, p, kine, stop_when=lambda lp: lp._phase is _Phase.RAMP_RELEASE
    )
    assert kine.v <= 14.0
    t_in = t
    while loop._phase is _Phase.RAMP_RELEASE:
        kine.step()
        t += 0.1
        _cycle(loop, p, kine.v, t)
    assert loop._phase is _Phase.RAMP_DOWN
    assert kine.v <= 10.0 + 1e-9
    assert t - t_in < 5.4  # 流れの全体（5.5 秒）の前に打ち切られた
    assert loop._rp_vref == kine.v
    # 既に安全網より下で着いた点は飛ばす
    q = _release_ramp(stations=(9.0,))
    loop2 = _loop([q], config=_release_config())
    _run_release(loop2, q, _ReleaseKine(loop2, 0.1))
    assert (loop2.ramp_results[0].release_n, loop2.ramp_results[0].release_skipped) == (0, 1)


def test_release_time_is_not_counted_in_the_down_leg_timeout() -> None:
    """戻しの分長くかかっても、下りの時間切れで打ち切られない（停車まで走って結果が残る）。"""
    p = _release_ramp(stations=(35.0, 25.0, 15.0))
    cfg = _release_config(ramp_release_min_kmh=0.0)
    loop = _loop([p], config=cfg)
    kine = _ReleaseKine(loop, 0.1)
    _run_release(loop, p, kine)
    assert len(loop.ramp_results) == 1
    assert kine.v == pytest.approx(0.0, abs=0.5)
    assert loop.ramp_results[0].release_n >= 1


def test_release_phase_is_part_of_the_ramp_phase_set() -> None:
    assert _Phase.RAMP_RELEASE in plmod._RAMP_PHASES
    assert _Phase.RAMP_RELEASE.name == "RAMP_RELEASE"


# ── 効き始め（ペダルの遊びの終わり。2026-10-16 段2o-6）────────────────────


def _fill_onset_buffer(loop: PatternLoop, accel_onset_u: float = 5.0,
                       brake_onset_u: float = 3.0, coast: float = 1.0) -> tuple[float, float]:
    """アクセルを min から踏み込む走り → ブレーキを min から踏み込む走り、を時系列バッファに
    入れる。遅れ 0 の車: 加速度 ＝ −惰行 ± 0.5 × max(0, u − 効き始め)。惰行は GLimitMap に貯める。
    戻り値は (アクセルの効き始め, ブレーキの効き始め)（絶対 [%]）。"""
    amin, bmin = _deadbands(loop)
    for v in range(0, 300, 10):
        loop.glimit.add_coast(float(v) + 5.0, coast)
    t, v = 0.0, 60.0
    for i in range(600):  # アクセル 0.03 %/周期 ＝ 0.6 %/s（0.05 s 周期）
        u = amin + 0.03 * i
        a = -coast + 0.5 * max(0.0, u - (amin + accel_onset_u))
        loop._onset_buf.append((t, v, u, bmin))
        v = max(1.0, v + a * 0.05)
        t += 0.05
    t += 5.0
    v = 200.0
    for i in range(600):
        u = bmin + 0.03 * i
        a = -coast - 0.5 * max(0.0, u - (bmin + brake_onset_u))
        loop._onset_buf.append((t, v, amin, u))
        v = max(1.0, v + a * 0.05)
        t += 0.05
    return amin + accel_onset_u, bmin + brake_onset_u


def test_onsets_are_computed_when_a_g_ramp_finishes_and_listed_in_the_result() -> None:
    p = _ramp()
    loop = _loop([p])
    want_a, want_b = _fill_onset_buffer(loop)
    assert loop.onset_history == [] and loop._onset_u(PEDAL_BRAKE) is None
    loop._ramp_record_result(p, 100.0)
    assert loop._onset_pct[PEDAL_ACCEL] == pytest.approx(want_a, abs=0.6)
    assert loop._onset_pct[PEDAL_BRAKE] == pytest.approx(want_b, abs=0.6)
    r = loop.ramp_results[0]
    assert r.accel_onset_pct == pytest.approx(want_a, abs=0.6)
    assert r.brake_onset_pct == pytest.approx(want_b, abs=0.6)
    assert len(loop.onset_history) == 1
    idx, hist_a, hist_b = loop.onset_history[0]
    assert (idx, hist_a, hist_b) == (loop._pattern_idx, r.accel_onset_pct, r.brake_onset_pct)
    u_on = loop._onset_u(PEDAL_BRAKE)
    assert u_on is not None and u_on == pytest.approx(3.0, abs=0.6)


def test_onsets_are_not_computed_without_a_coast_curve_and_keep_old_values() -> None:
    p = _ramp()
    loop = _loop([p])
    _fill_onset_buffer(loop)
    loop.glimit._coast.clear()  # 惰行カーブが無い
    loop._onset_pct[PEDAL_BRAKE] = 7.0  # 前の段の値
    loop._ramp_record_result(p, 100.0)
    assert loop._onset_pct == {PEDAL_BRAKE: 7.0}
    assert loop.ramp_results[0].accel_onset_pct is None
    assert loop.ramp_results[0].brake_onset_pct == 7.0


def test_onset_u_is_none_when_missing_or_not_above_min() -> None:
    loop = _loop([_ramp()])
    amin, bmin = _deadbands(loop)
    assert loop._onset_u(PEDAL_ACCEL) is None
    loop._onset_pct[PEDAL_ACCEL] = amin  # 0 以下は使わない
    loop._onset_pct[PEDAL_BRAKE] = bmin + 2.5
    assert loop._onset_u(PEDAL_ACCEL) is None
    assert loop._onset_u(PEDAL_BRAKE) == pytest.approx(2.5)


def test_hold_step_cycles_are_not_buffered_for_the_onset() -> None:
    p = _lag_ramp()
    loop = _loop([p])
    _seed_plant_table(loop)
    rows, _ = _run_lag(loop, p)
    n_step = sum(1 for _, ph, _, _, _ in rows if ph == "HOLD_STEP")
    assert n_step > 100
    assert len(loop._onset_buf) == len(rows) - n_step


def test_step_targets_start_brake_steps_at_the_onset_when_known() -> None:
    p = _lag_ramp()
    loop = _loop([p])
    _seed_plant_table(loop)
    _, bmin = _deadbands(loop)
    base = loop._ramp_step_targets(30.0, 4.0)
    assert base is not None and len(base) == 5
    delta_b = -base[3]
    loop._onset_pct[PEDAL_BRAKE] = bmin + 3.0
    t = loop._ramp_step_targets(30.0, 4.0)
    assert t is not None and len(t) == 7
    assert t[:3] == base[:3]
    assert t[3] == pytest.approx(-3.0) and t[4] == pytest.approx(-(3.0 + delta_b))
    assert t[5] == pytest.approx(-3.0) and t[6] == 0.0


def test_step_targets_warn_and_count_when_accel_start_is_shallower_than_onset() -> None:
    p = _lag_ramp()
    loop = _loop([p])
    _seed_plant_table(loop)
    amin, _ = _deadbands(loop)
    loop._onset_pct[PEDAL_ACCEL] = amin + 6.0
    base_targets = loop._ramp_step_targets(30.0, 4.0)  # 起点 4 ＜ 6
    assert loop.step_accel_below_onset_n == 1
    assert base_targets is not None and len(base_targets) == 5  # 列は変えない
    loop._ramp_step_targets(30.0, 8.0)  # 起点 8 ≥ 6
    assert loop.step_accel_below_onset_n == 1


def test_release_target_uses_the_onset_formula_when_the_peak_is_deeper() -> None:
    loop = _loop([_ramp()], config=_release_config())
    _, bmin = _deadbands(loop)
    loop._rp_release_u_peak = -8.0
    # 従来: 0.5 × (−8) ＝ −4、最後は 0
    assert loop._ramp_release_u(1.0) == pytest.approx(-4.0)
    loop._onset_pct[PEDAL_BRAKE] = bmin + 3.0
    # 効き始めあり: −3 + 0.5 × (−8 + 3) ＝ −5.5（f=0 は待機位置）
    assert loop._ramp_release_u(0.0) == pytest.approx(-8.0)
    assert loop._ramp_release_u(1.0) == pytest.approx(-5.5)
    assert loop._ramp_release_u(2.0) == pytest.approx(-5.5)
    assert loop._ramp_release_u(3.5) == pytest.approx(0.0)
    # 山が効き始めより浅いときは今の式
    loop._rp_release_u_peak = -2.0
    assert loop._ramp_release_u(1.0) == pytest.approx(-1.0)


def test_release_enabled_false_never_enters_ramp_release() -> None:
    p = _release_ramp()
    loop = _loop([p], config=_release_config(ramp_release_enabled=False))
    _, rows = _run_release(loop, p, _ReleaseKine(loop, 0.1))
    assert not any(ph == "RAMP_RELEASE" for _, ph, _, _ in rows)
    r = loop.ramp_results[0]
    assert (r.release_s, r.release_n, r.release_skipped) == (0.0, 0, 0)
    assert loop._rp_release_done == set()  # 済み・飛ばしの管理にも触らない


# ── 発進のゆっくり離し（段2o-10 A） ─────────────────────────────────────


def _lr_loop(slow: tuple[float, ...] = (0.25, 0.5), stop: float | None = 5.0) -> PatternLoop:
    cfg = PatternLoopConfig(
        ramp_launch_fast_rate_pctps=2.0, ramp_launch_release_margin_pct=2.0,
        ramp_launch_slow_rates_pctps=slow, stop_confirm_pct=stop,
        creep_launch_min_speed_kmh=1.0,
    )
    return _loop([_ramp(0.1, top_kmh=200.0)], config=cfg, launched=False)


def test_launch_release_fast_then_slow_and_phase_name() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _lr_loop()
    bdb = _deadbands(loop)[1]
    assert loop._rp_lr_active
    target_u = -(5.0 + 2.0 - bdb)
    u0 = loop._rp_u
    assert u0 < target_u
    # 最初の周期は動かさない。① は 2.0 %/s
    _cycle(loop, p, 0.0, 0.0)
    assert loop._rp_u == pytest.approx(u0)
    _cycle(loop, p, 0.0, 0.1)
    assert loop._rp_u == pytest.approx(u0 + 0.2)
    assert loop._rp_vref == 0.0  # 基準は進めない
    # ① が終わるまで回す
    t = 0.2
    while loop._rp_u < target_u - 1e-9:
        _cycle(loop, p, 0.0, t)
        t += 0.1
        assert loop._rp_lr_active
    assert loop._rp_u == pytest.approx(target_u)
    # ② 1 本目は 0.25 %/s
    _cycle(loop, p, 0.0, t)
    u1 = loop._rp_u
    _cycle(loop, p, 0.0, t + 0.1)
    assert loop._rp_u - u1 == pytest.approx(0.025)
    assert loop._rp_vref == 0.0 and loop._rp_lr_active
    assert plmod.RAMP_LAUNCH_RELEASE_PHASE == "RAMP_LAUNCH_RELEASE"


def test_launch_release_slow_rates_alternate_per_launch() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    rates = []
    for _ in range(3):
        loop = _lr_loop()
        rates.append(loop._rp_lr_slow)
    assert rates == [0.25, 0.25, 0.25]  # 別の PatternLoop は通し番号を持ち越さない
    loop = _lr_loop()
    loop._ramp_up_begin()  # 2 本目の発進
    loop._ramp_up_begin()  # 3 本目
    assert loop._rp_lr_slow == 0.25 and loop._rp_launch_seq == 3
    loop2 = _lr_loop()
    loop2._ramp_up_begin()
    assert loop2._rp_lr_slow == 0.5
    assert p is not None


def test_launch_release_moving_hands_over_with_current_opening() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _lr_loop()
    t = 0.0
    for _ in range(4):  # ① の途中
        _cycle(loop, p, 0.0, t)
        t += 0.1
    assert loop._rp_lr_active
    u_before = loop._rp_u
    speed = 1.2  # creep_launch_min_speed_kmh 以上
    _cycle(loop, p, speed, t)
    assert not loop._rp_lr_active
    assert loop._phase is _Phase.RAMP_UP
    assert loop._rp_u == pytest.approx(u_before + 0.0, abs=0.21)  # その周期の開度から
    u_hand = loop._rp_u
    assert loop._rp_vref_anchored and loop._rp_vref == pytest.approx(speed)
    assert loop._rp_launch_started == pytest.approx(t)
    # 以後は今の制御（偏差で離す・踏み直さない）
    _seed_sensitivity(loop, PEDAL_ACCEL, 0.2, 2.0)
    _seed_sensitivity(loop, PEDAL_BRAKE, 0.4, 2.0)
    prev = u_hand
    for k in range(1, 15):
        _cycle(loop, p, speed, t + 0.1 * k)
        assert loop._rp_u >= prev - 1e-9
        prev = loop._rp_u


def test_launch_release_without_creep_goes_to_current_flow_at_min() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _lr_loop(slow=(0.5,))
    t = 0.0
    n = 0
    while loop._rp_lr_active and n < 5000:
        _cycle(loop, p, 0.0, t)
        t += 0.1
        n += 1
    assert not loop._rp_lr_active and loop._rp_u == pytest.approx(0.0)
    assert loop._phase is _Phase.RAMP_UP and loop._rp_vref == 0.0
    # 今と同じ流れ: 次の周期から基準が +G で進む
    _cycle(loop, p, 0.0, t)
    t += 0.1
    _cycle(loop, p, 0.0, t)
    assert loop._rp_vref > 0.0


def test_launch_release_slow_phase_is_capped_by_time_limit() -> None:
    p = _ramp(0.1, top_kmh=200.0)
    loop = _lr_loop(slow=(0.25,))
    bdb = _deadbands(loop)[1]
    limit = (5.0 + 2.0 - bdb) / 0.25 + 5.0
    assert loop._rp_lr_limit_s == pytest.approx(limit)
    # ① を終えた状態から ② を回し、上限を超えたら ③ へ（min に着く前でも）
    loop._rp_u = loop._rp_lr_target_u
    loop._rp_lr_limit_s = 1.0
    t = 0.0
    while loop._rp_lr_active and t < 10.0:
        _cycle(loop, p, 0.0, t)
        t += 0.1
    assert not loop._rp_lr_active and loop._rp_u < 0.0 and t < 2.0


def test_launch_release_disabled_by_empty_rates_keeps_old_flow() -> None:
    loop = _lr_loop(slow=())
    assert not loop._rp_lr_active


# ── 停止前の戻し B-1（2026-10-23 段5-8）──────────────────────────────────


def _stop_config(**kw: object) -> PatternLoopConfig:
    base = PatternLoopConfig(
        stop_release_enabled=True, stop_release_start_kmh=7.5, stop_release_fracs=(0.25, 0.5),
        stop_release_rate_pct_s=2.0, stop_release_escape_s=0.5, stop_release_max_g=0.2,
    )
    return dataclasses.replace(base, **kw)  # type: ignore[arg-type]


class _StopKine(_Kine):
    """停止前の戻しの間の車: decel_ratio × G で減速（0 なら一定）。他の区間は _Kine と同じ。"""

    def __init__(self, loop: PatternLoop, g: float, decel_ratio: float = 0.4) -> None:
        super().__init__(loop, g)
        self.ratio = decel_ratio

    def step(self, dt: float = 0.1) -> None:
        if self.loop._phase is _Phase.RAMP_RELEASE:
            self.v = max(0.0, self.v - self.ratio * self.g * G_TO_KMHS * dt)
        else:
            super().step(dt)


def _run_stop(
    loop: PatternLoop, p: plmod.GRampPattern, kine: _Kine, *, u_down: float = -6.0,
    t_max: float = 150.0,
) -> list[tuple[float, str, float, float, float, float]]:
    """結果が記録されるまで回し、(t, CSV の phase 名, 車速, u, アクセル指令, ブレーキ指令) を返す。
    記録された周期の次の 1 周期も入れる（終わった後の phase を見るため）。"""
    rows = []
    t = 0.0
    done_at = None
    while t < t_max:
        if loop._phase is _Phase.RAMP_DOWN and kine.v > 0.0:
            loop._rp_u = min(loop._rp_u, u_down)
            loop._rp_brake_latched = True
        a, b = _cycle(loop, p, kine.v, t)
        rows.append((t, loop._csv_phase_name(), kine.v, loop._rp_u, a, b))
        if loop.ramp_results and done_at is None:
            done_at = t
        if done_at is not None and t > done_at:
            break
        kine.step()
        t += 0.1
    return rows


def test_stop_release_default_off_is_the_same_run() -> None:
    """stop_release_enabled が false（既定）なら、他の stop_release_* を変えても今と同じ走り。"""
    p = _release_ramp(stations=(), g=0.1)
    a = _loop([p], config=_release_config())
    b = _loop([p], config=_release_config(stop_release_fracs=(0.9,), stop_release_max_g=9.0))
    ra = _run_stop(a, p, _StopKine(a, 0.1))
    rb = _run_stop(b, p, _StopKine(b, 0.1))
    assert ra == rb
    assert not any(r[1] == "RAMP_STOP_RELEASE" for r in ra)
    assert PatternLoopConfig().stop_release_enabled is False


def test_stop_release_moves_u_linearly_holds_and_ends_at_stop() -> None:
    """7.5 km/h を切ると RAMP_STOP_RELEASE（CSV の名前）。u は山 −6 から 2%/s で山 × 0.25 だけ
    （−6 → −4.5）戻して保ち、停車（0.5 km/h 以下）で結果を記録して終わる。アクセルは常に 0。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_stop_config())
    _, bdb = _deadbands(loop)
    rows = _run_stop(loop, p, _StopKine(loop, 0.1))
    seg = [r for r in rows if r[1] == "RAMP_STOP_RELEASE"]
    assert seg
    t0, v0, u0 = seg[0][0], seg[0][2], seg[0][3]
    # 山 ＝ 入る前の直近 3 秒の u の最深（制御が踏み増すので −6 より深くなりうる）
    peak = min([r[3] for r in rows if r[1] == "RAMP_DOWN" and t0 - 3.0 <= r[0] < t0] + [u0])
    assert v0 < 7.5 and peak <= -6.0 and u0 >= peak - 1e-9
    target = 0.75 * peak
    for t, _name, _v, u, a, b in seg:
        want = min(target, u0 + 2.0 * (t - t0))
        assert u == pytest.approx(want, abs=0.25)
        assert a == 0.0 and b > bdb  # 片足だけ（ブレーキは min より上。指令は u の更新の前の値）
    assert seg[-1][3] == pytest.approx(target, abs=0.01)  # 戻した開度のまま保つ
    assert len(loop.ramp_results) == 1
    assert seg[-1][2] <= 0.5 + 1e-9 or seg[-1][2] < 1.0  # 停車まで保った
    assert rows[-1][1] not in ("RAMP_STOP_RELEASE", "RAMP_RELEASE")
    assert loop._csv_phase_name() == loop._phase.name  # 終わった後は普通の名前に戻る


def test_stop_release_alternates_fractions_per_step() -> None:
    """段ごとに割合を交互に使う（2 回目は 0.5 ＝ 山の半分）。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_stop_config())
    loop._rp_stop_release_n = 1
    rows = _run_stop(loop, p, _StopKine(loop, 0.1))
    seg = [r for r in rows if r[1] == "RAMP_STOP_RELEASE"]
    peak = min(r[3] for r in rows if r[1] == "RAMP_DOWN")
    assert seg[-1][3] == pytest.approx(0.5 * peak, abs=0.01)
    assert loop._rp_stop_release_n == 2


def test_stop_release_escape_when_deceleration_stops() -> None:
    """戻した開度で減速が止まった（実加速度 ≧ 0 が 0.5 秒続いた）ら停車保持へ踏む（結果を記録して
    DRIVE_BRAKE ＝ 停車復帰）。車はまだ走っている。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_stop_config())
    kine = _StopKine(loop, 0.1, decel_ratio=-0.1)  # 戻すと減速が止まり、わずかに加速する
    rows = _run_stop(loop, p, kine)
    seg = [r for r in rows if r[1] == "RAMP_STOP_RELEASE"]
    assert seg
    assert seg[-1][0] - seg[0][0] <= loop._config.coast_accel_slope_window_s + 1.5
    assert seg[-1][2] > 1.0  # 停車していないのに終わった
    assert len(loop.ramp_results) == 1
    assert loop._phase is _Phase.DRIVE_BRAKE


def test_stop_release_escape_when_speed_is_constant_slightly_negative() -> None:
    """速度が一定（実加速度が 0 付近でわずかに負）でも escape_s 後に停車保持へ踏む。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_stop_config())
    kine = _StopKine(loop, 0.1, decel_ratio=0.0)  # 戻すと速度が一定
    orig = loop._advance_stop_release

    def noisy(pattern, speed, now, measured_g):  # type: ignore[no-untyped-def]
        # 傾きの丸め・ノイズで 0 をわずかに下回る（-0.02 km/h/s）
        return orig(pattern, speed, now, None if measured_g is None else -0.02 / G_TO_KMHS)

    loop._advance_stop_release = noisy  # type: ignore[method-assign]
    rows = _run_stop(loop, p, kine)
    seg = [r for r in rows if r[1] == "RAMP_STOP_RELEASE"]
    assert seg
    assert seg[-1][0] - seg[0][0] <= loop._config.coast_accel_slope_window_s + 1.5
    assert seg[-1][2] > 1.0
    assert len(loop.ramp_results) == 1
    assert loop._phase is _Phase.DRIVE_BRAKE
    # 閾値 0（旧動作）なら居座る
    loop2 = _loop([p], config=_stop_config(stop_release_escape_decel_kmhs=0.0))
    orig2 = loop2._advance_stop_release
    loop2._advance_stop_release = lambda pt, sp, n, g: orig2(  # type: ignore[method-assign]
        pt, sp, n, None if g is None else -0.02 / G_TO_KMHS
    )
    rows2 = _run_stop(loop2, p, _StopKine(loop2, 0.1, decel_ratio=0.0), t_max=60.0)
    seg2 = [r for r in rows2 if r[1] == "RAMP_STOP_RELEASE"]
    assert seg2[-1][0] - seg2[0][0] >= 25.0  # 安全網（30 秒）まで居座る


def test_stop_release_skips_strong_steps_and_non_brake() -> None:
    """目標減速度が stop_release_max_g を超える段（0.25G）には入らない。0.2G ちょうどは入る。
    u ＜ 0 でない（ブレーキを踏んでいない）下りも入らない。"""
    strong = _release_ramp(stations=(), g=0.25)
    loop = _loop([strong], config=_stop_config())
    rows = _run_stop(loop, strong, _StopKine(loop, 0.25))
    assert not any(r[1] == "RAMP_STOP_RELEASE" for r in rows)
    edge = _release_ramp(stations=(), g=0.2)
    loop = _loop([edge], config=_stop_config())
    rows = _run_stop(loop, edge, _StopKine(loop, 0.2))
    assert any(r[1] == "RAMP_STOP_RELEASE" for r in rows)
    weak = _release_ramp(stations=(), g=0.1)
    loop = _loop([weak], config=_stop_config())
    t, v = 0.0, 40.0
    loop._phase = _Phase.RAMP_DOWN
    loop._rp_u, loop._rp_brake_latched = 0.0, False
    assert not loop._stop_release_try_enter(weak, 5.0, t)
    assert v > 0.0


def test_stop_release_never_presses_both_pedals() -> None:
    """停止前の戻しの全周期でアクセル指令は 0・ブレーキ指令は正（片足）。u は 0 以下。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_stop_config())
    rows = _run_stop(loop, p, _StopKine(loop, 0.1))
    seg = [r for r in rows if r[1] == "RAMP_STOP_RELEASE"]
    assert seg and all(r[4] == 0.0 and r[5] > 0.0 for r in seg)
    assert all(r[3] <= 0.0 for r in seg)
    lo, hi = loop._ramp_u_limits() if loop._phase is _Phase.RAMP_RELEASE else (0.0, 0.0)
    assert hi == 0.0 and lo <= 0.0


# ── 効き始めの少し先での小さな動き（RAMP_BRAKE_DITHER・RAMP_ACCEL_DITHER。2026-10-28 段2）──


def _dither_config(**kw: object) -> PatternLoopConfig:
    base = PatternLoopConfig(
        ramp_dither_enabled=True, ramp_dither_brake_speeds_kmh=(30.0,),
        ramp_dither_deltas_pct=(0.5, 0.25), ramp_dither_hold_s=0.5,
        ramp_dither_brake_u0_from_onset_pct=(0.5, 1.5, 2.5), ramp_dither_max_g=0.13,
        ramp_dither_accel_band_pct=10.0, ramp_dither_accel_max_n=3,
        ramp_dither_brake_onset_u_pct=6.0, ramp_dither_accel_onset_u_pct=0.0,
        ramp_release_min_kmh=10.0,
    )
    return dataclasses.replace(base, **kw)  # type: ignore[arg-type]


class _DitherKine(_Kine):
    """RAMP_BRAKE_DITHER の間は −0.4G で減速する車（それ以外は _Kine と同じ。保持・アクセルの
    dither は一定）。"""

    def step(self, dt: float = 0.1) -> None:
        if self.loop._phase is _Phase.RAMP_BRAKE_DITHER:
            self.v = max(0.0, self.v - 0.4 * self.g * G_TO_KMHS * dt)
        else:
            super().step(dt)


def _run_dither_down(
    loop: PatternLoop, p: plmod.GRampPattern, *, u_down: float = -7.0, t_max: float = 120.0
) -> list[tuple[float, str, float, float, float, float]]:
    """RAMP_DOWN の間は u を u_down 以下（ブレーキ側）に保って回す。
    [(t, phase 名, 車速, u, アクセル指令, ブレーキ指令)]。結果が記録されたら止める。"""
    kine = _DitherKine(loop, p.target_g, v0=40.0)
    loop._phase = _Phase.RAMP_DOWN
    loop._phase_started_at = 0.0
    loop._rp_brake_latched = True
    loop._rp_u = u_down
    rows: list[tuple[float, str, float, float, float, float]] = []
    t = 0.0
    while t < t_max and not loop.ramp_results:
        if loop._phase is _Phase.RAMP_DOWN and kine.v > 0.0:
            loop._rp_u = min(loop._rp_u, u_down)
            loop._rp_brake_latched = True
        a, b = _cycle(loop, p, kine.v, t)
        rows.append((t, loop._phase.name, kine.v, loop._rp_u, a, b))
        kine.step()
        t += 0.1
    return rows


def test_dither_off_is_the_old_behaviour() -> None:
    """既定（off）、または on でも効き始めが読めない（None）なら、phase の並びは同じで
    dither の列は 0。"""
    p = _release_ramp(stations=(), g=0.1)
    names: list[list[str]] = []
    for cfg in (
        PatternLoopConfig(ramp_release_min_kmh=10.0),
        _dither_config(ramp_dither_enabled=False),
        _dither_config(ramp_dither_brake_onset_u_pct=None),
    ):
        loop = _loop([p], config=cfg)
        rows = _run_dither_down(loop, p)
        names.append([r[1] for r in rows])
        assert not any("DITHER" in r[1] for r in rows)
        r0 = loop.ramp_results[0]
        assert (r0.dither_brake_s, r0.dither_brake_n, r0.dither_brake_skipped) == (0.0, 0, 0)
        assert (r0.dither_accel_s, r0.dither_accel_n) == (0.0, 0)
    assert names[0] == names[1] == names[2]


def test_brake_dither_enters_inside_ramp_down_and_steps_u0_pm_delta() -> None:
    """30 km/h を切ると RAMP_DOWN の中で RAMP_BRAKE_DITHER に入り、u0 ＝ −（効き始め 6.0 ＋ 0.5）
    ＝ −6.5（今の u −7.0 ではない）。u は u0−Δ → u0 → u0+Δ → u0（深くする向きが +。u は負が
    ブレーキ。Δ ＝ 0.5・各 0.5 秒・ステップ）。出たら RAMP_DOWN に戻る。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_dither_config())
    rows = _run_dither_down(loop, p)
    names = [r[1] for r in rows]
    i0 = names.index("RAMP_BRAKE_DITHER")
    assert names[i0 - 1] == "RAMP_DOWN"
    seg = [r for r in rows if r[1] == "RAMP_BRAKE_DITHER"]
    assert rows[i0][2] <= 30.0  # 測る車速を切った周期
    # 入った周期の u が u0。以後 ステップで 4 つの開度
    plateaus: list[float] = []
    for r in seg:
        if not plateaus or abs(plateaus[-1] - r[3]) > 1e-9:
            plateaus.append(r[3])
    u0 = loop._rp_dither_u0
    assert u0 == pytest.approx(-6.5)  # 効き始め 6.0 ＋ 0.5（今の u −7.0 ではない）
    assert seg[0][3] == pytest.approx(-6.5)  # 入った周期にステップで u0 へ移る
    assert plateaus[-4:] == pytest.approx([u0 - 0.5, u0, u0 + 0.5, u0])
    assert seg[-1][0] - seg[0][0] == pytest.approx(2.0, abs=0.2)  # 4 × 0.5 秒
    assert "RAMP_DOWN" in names[names.index("RAMP_BRAKE_DITHER"):]
    r0 = loop.ramp_results[0]
    assert (r0.dither_brake_n, r0.dither_brake_skipped) == (1, 0)
    assert r0.dither_brake_s == pytest.approx(2.0, abs=0.3)


def test_brake_dither_alternates_delta_between_runs() -> None:
    """Δ は測る車速ごとに交互（0.5 → 0.25）。2 本目の流れは Δ ＝ 0.25。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_dither_config())
    _run_dither_down(loop, p)
    assert loop._rp_dither_delta == 0.5 and loop._rp_dither_seq_brake == 1
    loop.ramp_results.clear()
    loop._rp_dither_brake_done.clear()
    rows = _run_dither_down(loop, p)
    seg = [r[3] for r in rows if r[1] == "RAMP_BRAKE_DITHER"]
    u0 = loop._rp_dither_u0
    assert min(seg) == pytest.approx(u0 - 0.25) and max(seg) == pytest.approx(u0 + 0.25)


def test_brake_dither_enters_even_when_opening_is_outside_the_old_band() -> None:
    """今の u が浅い（−2.0）・深い（−12.0）でも入る。u0 は今の u に関係なく −6.5。"""
    p = _release_ramp(stations=(), g=0.1)
    for u_down in (-2.0, -12.0):
        loop = _loop([p], config=_dither_config())
        rows = _run_dither_down(loop, p, u_down=u_down)
        assert any(r[1] == "RAMP_BRAKE_DITHER" for r in rows)
        r0 = loop.ramp_results[0]
        assert (r0.dither_brake_n, r0.dither_brake_skipped) == (1, 0)
        assert loop._rp_dither_u0 == pytest.approx(-6.5)


def test_brake_dither_u0_cycles_through_offsets_by_entry_number() -> None:
    """u0 ＝ −（効き始め ＋ 0.5 / 1.5 / 2.5）を入るたびに順に使い、4 回目は 0.5 に戻る。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_dither_config())
    got: list[float] = []
    for _ in range(4):
        loop.ramp_results.clear()
        loop._rp_dither_brake_done.clear()
        _run_dither_down(loop, p)
        got.append(loop._rp_dither_u0)
    assert got == pytest.approx([-6.5, -7.5, -8.5, -6.5])


def test_brake_dither_u0_and_steps_are_clamped_to_min_and_limit() -> None:
    """u0 が深すぎれば下限 lo、浅すぎれば min（u ＝ 0）に切る。u0±Δ も範囲内。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_dither_config(ramp_dither_brake_u0_from_onset_pct=(500.0,)))
    rows = _run_dither_down(loop, p)
    lo, _ = loop._ramp_u_limits()
    assert loop._rp_dither_u0 == pytest.approx(lo)
    assert all(r[3] >= lo - 1e-9 for r in rows if r[1] == "RAMP_BRAKE_DITHER")
    loop = _loop([p], config=_dither_config(
        ramp_dither_brake_onset_u_pct=0.1, ramp_dither_brake_u0_from_onset_pct=(0.0,),
        ramp_dither_deltas_pct=(0.5,),
    ))
    rows = _run_dither_down(loop, p)
    seg = [r[3] for r in rows if r[1] == "RAMP_BRAKE_DITHER"]
    assert loop._rp_dither_u0 == pytest.approx(-0.1)
    assert max(seg) <= 0.0 and min(seg) == pytest.approx(-0.6)  # 浅い側は min（0）を越えない


def test_brake_dither_max_g_includes_0125g_but_not_stronger() -> None:
    """既定の max_g 0.13 では 0.125G の段に入り、0.14G の段には入らない。"""
    assert PatternLoopConfig().ramp_dither_max_g == pytest.approx(0.13)
    assert PatternLoopConfig().ramp_dither_brake_speeds_kmh == (100.0, 75.0, 50.0, 30.0)
    for g, enters in ((0.125, True), (0.14, False)):
        p = _release_ramp(stations=(), g=g)
        loop = _loop([p], config=_dither_config())
        rows = _run_dither_down(loop, p)
        assert any(r[1] == "RAMP_BRAKE_DITHER" for r in rows) is enters


def test_brake_dither_does_not_enter_during_ramp_release() -> None:
    """RAMP_RELEASE 中は入らない（RAMP_RELEASE → RAMP_BRAKE_DITHER の直接の遷移はない）。
    戻しが終わって RAMP_DOWN に戻ってから入る。"""
    p = _release_ramp(stations=(35.0,), g=0.1)
    loop = _loop([p], config=_dither_config(**dataclasses.asdict(_release_config())))
    kine = _ReleaseKine(loop, p.target_g, v0=40.0)
    loop._phase, loop._phase_started_at = _Phase.RAMP_DOWN, 0.0
    loop._rp_brake_latched, loop._rp_u = True, -7.0
    names: list[str] = []
    t = 0.0
    while t < 120.0 and not loop.ramp_results:
        if loop._phase is _Phase.RAMP_DOWN and kine.v > 0.0:
            loop._rp_u = min(loop._rp_u, -7.0)
            loop._rp_brake_latched = True
        _cycle(loop, p, kine.v, t)
        names.append(loop._phase.name)
        kine.step()
        t += 0.1
    assert "RAMP_RELEASE" in names
    pairs = list(zip(names, names[1:], strict=False))
    assert ("RAMP_RELEASE", "RAMP_BRAKE_DITHER") not in pairs


def test_brake_dither_does_not_enter_on_strong_g_steps() -> None:
    """目標減速度が ramp_dither_max_g を超える段には入らない（飛ばした数にも数えない）。"""
    p = _release_ramp(stations=(), g=0.25)
    loop = _loop([p], config=_dither_config())
    rows = _run_dither_down(loop, p)
    assert not any(r[1] == "RAMP_BRAKE_DITHER" for r in rows)
    r0 = loop.ramp_results[0]
    assert (r0.dither_brake_n, r0.dither_brake_skipped) == (0, 0)
    assert loop._rp_dither_brake_done == set()


def test_brake_dither_never_presses_both_pedals_and_stays_on_the_brake_side() -> None:
    """全周期でアクセル指令 0・ブレーキ指令 ≧ min、u ≦ 0。上下限（hi ＝ 0）も守る。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_dither_config())
    _, bdb = _deadbands(loop)
    rows = _run_dither_down(loop, p)
    seg = [r for r in rows if r[1] == "RAMP_BRAKE_DITHER"]
    assert seg
    assert all(r[4] == 0.0 and r[5] >= bdb for r in seg)
    assert all(r[3] <= 0.0 for r in seg)
    loop._phase = _Phase.RAMP_BRAKE_DITHER
    lo, hi = loop._ramp_u_limits()
    assert hi == 0.0 and lo < 0.0 and loop._ramp_on_brake()


def test_brake_dither_does_not_feed_glimit_or_down_g_or_control() -> None:
    """dither の間は glimit に点を足さず、`_rp_down_g` にも入れず、制御の項も呼ばない。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_dither_config())
    _, bdb = _deadbands(loop)
    added: list[str] = []
    orig_add = loop.glimit.add

    def spy(pedal: str, speed: float, opening: float, accel: float) -> None:
        added.append(loop._phase.name)
        orig_add(pedal, speed, opening, accel)

    loop.glimit.add = spy  # type: ignore[method-assign]
    controls: list[str] = []
    orig_ctl = loop._ramp_control

    def ctl(*args: object, **kw: object) -> None:
        controls.append(loop._phase.name)
        orig_ctl(*args, **kw)  # type: ignore[arg-type]

    loop._ramp_control = ctl  # type: ignore[method-assign]
    loop._brake_actual_pct = bdb + 7.0
    loop._accel_actual_pct = 0.0
    rows = _run_dither_down(loop, p)
    assert any(r[1] == "RAMP_BRAKE_DITHER" for r in rows)
    assert "RAMP_BRAKE_DITHER" not in added and "RAMP_BRAKE_DITHER" not in controls
    assert "RAMP_DOWN" in controls  # 外では制御している（テストの前提）


def test_brake_dither_exit_returns_to_down_with_vref_at_speed_and_excludes_time() -> None:
    """出た後は RAMP_DOWN: v_ref ＝ 実車速・前の偏差 ＝ 0・g_ref は 0 から。下りの時間切れから
    dither の時間が除かれる（`_rp_down_prior_s` に入る前の分だけ。dither の間は数えない）。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_dither_config())
    rows = _run_dither_down(loop, p)
    # dither の次の周期（RAMP_DOWN に戻った直後）を作り直して確かめる
    loop2 = _loop([p], config=_dither_config())
    kine = _DitherKine(loop2, 0.1, v0=40.0)
    loop2._phase, loop2._phase_started_at = _Phase.RAMP_DOWN, 0.0
    loop2._rp_brake_latched, loop2._rp_u = True, -7.0
    t = 0.0
    while loop2._phase is not _Phase.RAMP_BRAKE_DITHER and t < 60.0:
        loop2._rp_u = min(loop2._rp_u, -7.0)
        _cycle(loop2, p, kine.v, t)
        kine.step()
        t += 0.1
    prior = loop2._rp_down_prior_s
    assert prior > 0.0
    while loop2._phase is _Phase.RAMP_BRAKE_DITHER and t < 60.0:
        _cycle(loop2, p, kine.v, t)
        kine.step()
        t += 0.1
    assert loop2._phase is _Phase.RAMP_DOWN
    assert loop2._rp_vref == pytest.approx(kine.v, abs=0.5)
    assert loop2._rp_turn_g0 == 0.0 and loop2._rp_e_prev == 0.0
    assert loop2._rp_down_prior_s == pytest.approx(prior)  # dither の間は数えない
    assert rows  # 上の通し走行も終わっている


def test_brake_dither_safety_exit_at_low_speed() -> None:
    """流れの途中でも実車速が安全網（ramp_release_min_kmh）以下になったら RAMP_DOWN へ戻る。"""
    p = _release_ramp(stations=(), g=0.1)
    loop = _loop([p], config=_dither_config(ramp_dither_hold_s=3.0, ramp_release_min_kmh=29.0))
    rows = _run_dither_down(loop, p)
    seg = [r for r in rows if r[1] == "RAMP_BRAKE_DITHER"]
    # 入ってから 12 秒の流れだが、車速が 29 km/h 以下になった時点で出る（−0.4G は 約 1.4 km/h/s）
    assert seg and seg[-1][0] - seg[0][0] < 6.0


def _run_dither_up(
    loop: PatternLoop, p: plmod.GRampPattern, *, u_cruise: float | None = 3.0,
    t_max: float = 200.0,
) -> list[tuple[float, str, float, float, float, float]]:
    """保持（RAMP_CRUISE）の間は u を u_cruise に保って回す。
    [(t, phase 名, 車速, u, アクセル指令, ブレーキ指令)]。結果が記録されたら止める。"""
    kine = _DitherKine(loop, p.target_g, v0=5.0)
    rows: list[tuple[float, str, float, float, float, float]] = []
    t = 0.0
    while t < t_max and not loop.ramp_results:
        if u_cruise is not None and loop._phase is _Phase.RAMP_CRUISE:
            loop._rp_u = u_cruise
        a, b = _cycle(loop, p, kine.v, t)
        rows.append((t, loop._phase.name, kine.v, loop._rp_u, a, b))
        kine.step()
        t += 0.1
    return rows


def test_accel_dither_follows_ramp_cruise_and_steps_u0_pm_delta() -> None:
    """保持が完了したとき u が効き始め 〜 ＋ band の中なら RAMP_ACCEL_DITHER（RAMP_CRUISE の後・
    RAMP_UP の前）。u は u0＋Δ → u0 → u0−Δ → u0。両踏みにならない（ブレーキ指令 0）。"""
    p = _cruise_ramp(g=0.1, top_kmh=40.0, stations=(20.0,), up_only=True)
    loop = _loop([p], config=_dither_config(
        ramp_dither_accel_onset_u_pct=1.0, ramp_dither_brake_speeds_kmh=()
    ))
    rows = _run_dither_up(loop, p, u_cruise=3.0)
    names = [r[1] for r in rows]
    i = names.index("RAMP_ACCEL_DITHER")
    assert names[i - 1] == "RAMP_CRUISE" and names[names.index("RAMP_ACCEL_DITHER", i):][-1] != ""
    j = max(k for k, n in enumerate(names) if n == "RAMP_ACCEL_DITHER")
    assert names[j + 1] == "RAMP_UP"
    seg = [r for r in rows if r[1] == "RAMP_ACCEL_DITHER"]
    plateaus: list[float] = []
    for r in seg:
        if not plateaus or abs(plateaus[-1] - r[3]) > 1e-9:
            plateaus.append(r[3])
    d = loop._rp_dither_delta
    u0 = loop._rp_dither_u0
    assert d == 0.5 and u0 == pytest.approx(3.0, abs=0.5)
    assert plateaus[-4:] == pytest.approx([u0 + d, u0, u0 - d, u0])
    assert all(r[5] == 0.0 for r in seg)  # ブレーキ指令 0（両踏みにならない）
    r0 = loop.ramp_results[0]
    assert (r0.dither_accel_n, r0.dither_brake_n) == (1, 0)
    assert r0.dither_accel_s == pytest.approx(2.0, abs=0.3)
    assert r0.cruise_done == 1


def test_accel_dither_clamps_u_minus_delta_to_min_and_uses_no_control() -> None:
    """u0 − Δ が min（u ＝ 0）未満になる場合は min にクランプ。制御の項は呼ばない。"""
    p = _cruise_ramp(g=0.1, top_kmh=40.0, stations=(20.0,), up_only=True)
    loop = _loop([p], config=_dither_config(ramp_dither_deltas_pct=(0.5,)))
    controls: list[str] = []
    orig_ctl = loop._ramp_control

    def ctl(*args: object, **kw: object) -> None:
        controls.append(loop._phase.name)
        orig_ctl(*args, **kw)  # type: ignore[arg-type]

    loop._ramp_control = ctl  # type: ignore[method-assign]
    rows = _run_dither_up(loop, p, u_cruise=0.2)  # u0 ≈ 0.2: u0−Δ < 0
    seg = [r for r in rows if r[1] == "RAMP_ACCEL_DITHER"]
    assert seg and min(r[3] for r in seg) >= 0.0
    assert min(r[3] for r in seg) == 0.0
    assert "RAMP_ACCEL_DITHER" not in controls


def test_accel_dither_skips_outside_band_and_respects_max_n() -> None:
    """帯の外（効き始めより浅い・band より深い）は入らず skipped に数える。1 本あたり
    ramp_dither_accel_max_n 回まで。"""
    p = _cruise_ramp(g=0.1, top_kmh=40.0, stations=(15.0, 25.0), up_only=True)
    for u in (-0.5, 15.0):
        loop = _loop([p], config=_dither_config(
        ramp_dither_accel_onset_u_pct=1.0, ramp_dither_brake_speeds_kmh=()
    ))
        rows = _run_dither_up(loop, p, u_cruise=u)
        assert not any(r[1] == "RAMP_ACCEL_DITHER" for r in rows)
        r0 = loop.ramp_results[0]
        assert r0.dither_accel_n == 0 and r0.dither_accel_skipped == 2
    loop = _loop(
        [p], config=_dither_config(ramp_dither_accel_onset_u_pct=1.0, ramp_dither_accel_max_n=1)
    )
    _run_dither_up(loop, p, u_cruise=3.0)
    r0 = loop.ramp_results[0]
    assert r0.dither_accel_n == 1 and r0.cruise_n == 2
    # 既定どおり 2 回まで入れる（保持が 2 つ）
    loop = _loop([p], config=_dither_config(
        ramp_dither_accel_onset_u_pct=1.0, ramp_dither_brake_speeds_kmh=()
    ))
    _run_dither_up(loop, p, u_cruise=3.0)
    assert loop.ramp_results[0].dither_accel_n == 2


def test_accel_dither_time_is_excluded_from_the_up_leg_timeout() -> None:
    """dither の時間は保持の開始から数えるので、上りの時間切れの経過から除かれる
    （`_rp_up_cruise_s` に dither の秒数が入る）。"""
    p = _cruise_ramp(g=0.1, top_kmh=40.0, stations=(20.0,), up_only=True)
    base = _loop([p])
    _run_dither_up(base, p, u_cruise=3.0)
    with_d = _loop([p], config=_dither_config(
        ramp_dither_accel_onset_u_pct=1.0, ramp_dither_brake_speeds_kmh=()
    ))
    _run_dither_up(with_d, p, u_cruise=3.0)
    assert with_d._rp_up_cruise_s - base._rp_up_cruise_s == pytest.approx(2.0, abs=0.4)


def test_dither_phases_are_part_of_the_ramp_phase_set() -> None:
    assert _Phase.RAMP_BRAKE_DITHER in plmod._RAMP_PHASES
    assert _Phase.RAMP_ACCEL_DITHER in plmod._RAMP_PHASES
    assert _Phase.RAMP_BRAKE_DITHER.name == "RAMP_BRAKE_DITHER"
    assert _Phase.RAMP_ACCEL_DITHER.name == "RAMP_ACCEL_DITHER"
