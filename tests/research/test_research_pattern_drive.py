"""研究開発用ハーネス 手順 2-1/2-2（パターン走行 → FF モデル作成）のユニットテスト。

スタブ HW（StubVehicle 付き）で研究用の状態機械 PatternLoop（本番 LearningLoop のアルゴリズムを
移植した自前実装）を短いパターン列で回し、走行後の停車保持・非常停止時のペダル解放・CSV・
モデル作成と YAML 書き戻しを確認する。
2-0 のペダル探索は test_research_pedal_search.py で確認し、ここでは結果を与えて始める。
"""

from __future__ import annotations

from dataclasses import replace
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import hardware as hwmod
from tests.research import main as mainmod
from tests.research import pattern_drive as pdmod
from tests.research import step2_check
from tests.research.learning_patterns import COAST_DOWN_COUNT, LearningDataError
from tests.research.live_plot import PlotSample, save_drive_figure
from tests.research.pattern_loop import (
    CreepLaunchPattern,
    GRampPattern,
    PatternLoopConfig,
    RampStatus,
)
from tests.research.pedal_search import PedalSearchResult
from tests.research.research_types import (
    VEHICLE_STOP_SPEED_KMH,
    DriveLogData,
    DrivingMode,
    LearningPattern,
    PatternKind,
    SpeedPoint,
)
from tests.research.vehicle import (
    STROKE_LIMIT_PULSE,
    build_vehicle_profile,
    feedforward_params,
    opening_to_pulse,
)

# ペダルに触れた位置 min（yaml の vehicle.min_*_opening_pct。スタブの真の遊び 6%/8% に近い値）。
# 2026-10-08 段2l-6: 不感帯の測定をやめたので 2-0 の結果ではなく設定の値（_tmp_cfg で書く）
MIN_ACCEL_PCT = 6.3
MIN_BRAKE_PCT = 8.4
# 2-0 のペダル探索が終わった想定の結果
PEDAL = PedalSearchResult(
    creep_speed_kmh=5.0,
    stop_confirm_pct=12.0,
    stop_brake_opening_pct=22.0,
)

# 本番の既定（数分）では単体テストにならないので、状態機械はそのままに時間だけ縮める
FAST_LOOP = PatternLoopConfig(
    accel_ramp_time_s=0.2,
    brake_ramp_time_s=0.2,
    creep_settle_min_s=0.3,
    creep_settle_stable_duration_s=0.2,
    creep_settle_timeout_s=1.0,
    accel_full_range_timeout_s=1.5,
    brake_stop_timeout_s=4.0,
    coast_timeout_s=1.0,
    coast_accel_rate_gain=1000.0,  # 短い試験でも開度が上がるように踏む速さを上げる
)
SHORT_PATTERNS = [
    LearningPattern(PatternKind.CREEP, accel_opening=0.0, brake_opening=22.0, hold_duration_s=0.3),
    LearningPattern(
        PatternKind.CREEP_SETTLE, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.3
    ),
    LearningPattern(
        PatternKind.COAST_DOWN, accel_opening=40.0, brake_opening=0.0, hold_duration_s=0.3
    ),
]


# 段の (狙いの G, 上端の車速)。2 周目（0.08 の 2 本目）を含む実行順
RAMP_ORDER = [(0.02, 100.0), (0.08, 90.0), (0.25, 60.0), (0.08, 90.0)]


def _tmp_cfg(tmp_path: Path) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
            "output.plot": False,
            # 段2l-6 から手順2 の対象は C7・C8 だけ（C1〜C5 は ConfigError）。C7 に固定
            # （既定 yaml は C7/C8 のこともある）
            "feedforward.candidate": "C7",
            "vehicle.min_accel_opening_pct": MIN_ACCEL_PCT,
            "vehicle.min_brake_opening_pct": MIN_BRAKE_PCT,
            # 走行後の緩減速を短くする（判定ロジックは既定と同じ）
            "decel_stop.step_mm": 1.0,
            "decel_stop.dwell_s": 0.2,
            "decel_stop.slope_window_s": 0.2,
        }
    )
    return cfgmod.load_config(path)


async def _held_stub(cfg: cfgmod.ResearchConfig) -> hwmod.ResearchHardware:
    """初期化済みで、2-0 の終わりと同じく停車保持開度で止まっているスタブ。"""
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    await hw.brake.move_to_position(opening_to_pulse(PEDAL.stop_brake_opening_pct))
    return hw


def _synthetic_samples(cfg: cfgmod.ResearchConfig) -> list[dlmod.DriveSample]:
    """スタブ車両モデルを時間指定で回した、加速・制動・惰行を含む 0.1s 刻みのログ。"""
    accel = hwmod.StubActuator("accel", connected=True)
    brake = hwmod.StubActuator("brake", connected=True)
    # スタブ車両（hardware.StubVehicle: アクセル 0.25・ブレーキ 0.5 km/h/s/% 固定）と釣り合う
    # 惰行にする。実機 config の 9.2〜10.8 km/h/s だとアクセル 25% でも 14 km/h で釣り合い、
    # ブレーキ区間がクリープ以下に沈んでブレーキゲイン推定のサンプルが採れない
    # （config_testVehicle.yaml はユーザーが実機に合わせて書き換える値なので決め打ちしない）
    params = replace(
        feedforward_params(cfg),
        coast_decel_speeds_kmh=(),
        coast_decel_kmhs=(),
        engine_brake_decel_kmhs=1.6,
        creep_speed_kmh=5.0,
        creep_rate_kmhs=0.5,
    )
    vehicle = hwmod.StubVehicle(accel=accel, brake=brake, params=params)
    wall0 = datetime(2026, 9, 10, tzinfo=UTC)
    samples: list[dlmod.DriveSample] = []
    speed = vehicle.advance(0.0, now=0.0)
    t = 0.0
    for accel_pct, brake_pct, seconds in (
        (25.0, 0.0, 8.0), (0.0, 18.0, 6.0), (45.0, 0.0, 5.0), (0.0, 0.0, 10.0),
        (0.0, 38.0, 4.0), (15.0, 0.0, 10.0), (0.0, 12.0, 10.0), (65.0, 0.0, 4.0), (0.0, 0.0, 15.0),
    ):
        accel.position = opening_to_pulse(accel_pct)
        brake.position = opening_to_pulse(brake_pct)
        for _ in range(round(seconds / 0.1)):
            t += 0.1
            speed = vehicle.advance(speed, now=t)
            samples.append(
                dlmod.DriveSample(
                    elapsed_s=t,
                    timestamp=wall0 + timedelta(seconds=t),
                    data=DriveLogData(
                        ref_speed_kmh=None,
                        actual_speed_kmh=speed,
                        accel_opening=accel_pct,
                        brake_opening=brake_pct,
                        accel_pos=accel.position,
                        brake_pos=brake.position,
                        accel_current=0.0,
                        brake_current=0.0,
                    ),
                    section=dlmod.SECTION_PATTERN_DRIVE,
                    phase="TEST",
                    pattern="1:TEST",
                )
            )
    return samples


# ── 開度の定義・プロファイル・パターン ────────────────────────────────


def test_opening_is_home_to_stroke_limit() -> None:
    assert opening_to_pulse(0.0) == 0
    assert opening_to_pulse(100.0) == STROKE_LIMIT_PULSE == 9500
    profile = build_vehicle_profile(cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH))
    assert profile.calibration is not None
    assert profile.calibration.accel_zero_pos == 0
    assert profile.calibration.brake_full_pos == STROKE_LIMIT_PULSE


def test_build_vehicle_profile_maps_yaml() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = build_vehicle_profile(cfg)
    assert profile.max_speed == cfg.vehicle.max_speed_kmh
    assert profile.max_decel_g == cfg.vehicle.max_decel_g
    ffp = profile.feedforward_params
    assert ffp.stop_brake_opening_pct == cfg.stop_brake_opening_pct
    # 2026-10-08 段2l-6: 名前は不感帯のまま、値は u の原点 min
    assert ffp.brake_deadband_pct == cfg.vehicle.min_brake_opening_pct
    assert ffp.accel_deadband_pct == cfg.vehicle.min_accel_opening_pct


def _profile(cfg: cfgmod.ResearchConfig) -> Any:
    return PEDAL.apply_to_profile(build_vehicle_profile(cfg))


def test_build_patterns_layout_is_coast_then_ramps_only() -> None:
    """段2l-6: コーストダウン → G ランプ（実行順どおり）だけ。クリープ域ブレーキ保持・
    クリープ発進・格子ステップ・発進停車・G 校正・通し掃引は入らない。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    patterns = pdmod.build_patterns(cfg, _profile(cfg), RAMP_ORDER)

    head = patterns[:COAST_DOWN_COUNT]
    ramps = patterns[COAST_DOWN_COUNT:]
    assert len(patterns) == COAST_DOWN_COUNT + len(RAMP_ORDER)
    assert [p.kind for p in head] == [PatternKind.COAST_DOWN] * COAST_DOWN_COUNT
    assert all(isinstance(p, GRampPattern) and p.kind is PatternKind.G_RAMP for p in ramps)
    # 段の G と上端の車速は渡した順のまま
    assert [(p.target_g, p.top_kmh) for p in ramps if isinstance(p, GRampPattern)] == RAMP_ORDER
    assert [(g, top) for g, top, _st, _lag in pdmod.ramp_order_of(patterns)] == RAMP_ORDER
    assert not any(isinstance(p, CreepLaunchPattern) for p in patterns)
    assert not any(p.kind.name in {"ACCEL_SWEEP", "BRAKE_HOLD", "CRUISE_TRIM", "CREEP_SETTLE"}
                   for p in patterns)


def test_build_patterns_passes_stations_up_only_from_the_fourth_element() -> None:
    """2026-10-14 段2o-4: 4 つ目があれば stations_up_only に渡り、3 要素・2 要素は False。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    order = [(0.08, 60.0, (10.0, 20.0), True), (0.1, 60.0, (10.0,), False),
             (0.125, 60.0, (10.0,)), (0.15, 60.0)]
    ramps = [p for p in pdmod.build_patterns(cfg, _profile(cfg), order)
             if isinstance(p, GRampPattern) and not p.lag_step]
    assert [p.stations_up_only for p in ramps] == [True, False, False, False]
    assert ramps[0].stations_kmh == (10.0, 20.0)


def test_build_patterns_matches_ramp_plan_execution_order() -> None:
    """RampPlan.execution_order() をそのまま渡すと、段の G・上端の車速が RampPlan と一致する。"""
    from tests.research.ramp_planner import RampLevel, RampPlan

    levels = (
        RampLevel(g=0.02, top_kmh=100.0, mode_accel_s=10.0, mode_decel_s=10.0, est_s=300.0,
                  passes=2),
        RampLevel(g=0.25, top_kmh=60.0, mode_accel_s=5.0, mode_decel_s=5.0, est_s=50.0, passes=1),
    )
    plan = RampPlan(levels=levels, skipped=(0.35,), total_est_s=650.0, budget_s=1200.0,
                    over_budget=False, cruise_s=0.0)
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    patterns = pdmod.build_patterns(cfg, _profile(cfg), plan.execution_order())
    ramps = [p for p in patterns if isinstance(p, GRampPattern)]
    assert [(p.target_g, p.top_kmh) for p in ramps] == [(0.02, 100.0), (0.25, 60.0), (0.02, 100.0)]
    # 1 周目は G 昇順、2 周目は後ろに回る
    assert [p.kind for p in patterns[COAST_DOWN_COUNT:COAST_DOWN_COUNT + 3]] == [
        PatternKind.G_RAMP
    ] * 3


def test_build_patterns_passes_release_stations_and_defaults_to_empty() -> None:
    """2026-10-14 段2o-5: execution_order の 5 つ目が release_stations に入る。省略は空。
    loop_config は learning.ramp_release_* と安全網の車速（クリープ＋5）を渡す。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    order = [(0.08, 90.0, (), True, (15.0, 35.0)), (0.125, 60.0), (0.25, 50.0, (20.0,))]
    ramps = [
        p for p in pdmod.build_patterns(cfg, _profile(cfg), order) if isinstance(p, GRampPattern)
    ]
    assert [p.release_stations for p in ramps] == [(15.0, 35.0), (), ()]
    lc = pdmod.loop_config_from_config(cfg)
    assert lc.ramp_release_fracs == tuple(cfg.learning.ramp_release_fracs)
    assert lc.ramp_release_hold_s == tuple(cfg.learning.ramp_release_hold_s)
    assert lc.ramp_release_move_s == cfg.learning.ramp_release_move_s
    assert lc.ramp_release_min_kmh == pytest.approx(cfg.feedforward.creep_speed_kmh + 5.0)


def test_result_table_shows_release_columns() -> None:
    from tests.research.pattern_loop import RampResult

    results = [RampResult(0, 0.08, 90.0, 40.0, 35.0, 0.081, 0.079, 90.5, release_s=16.5,
                          release_n=3, release_skipped=1)]
    text = pdmod.result_table([(0.08, 90.0)], results, 120.0)
    assert "戻し 秒" in text and "3/1" in text


def test_result_table_lists_each_ramp_and_marks_unrun() -> None:
    from tests.research.pattern_loop import RampResult

    results = [RampResult(0, 0.08, 90.0, 40.0, 35.0, 0.081, 0.079, 90.5, hold_s=9.0)]
    text = pdmod.result_table([(0.08, 90.0), (0.25, 60.0)], results, 120.0)
    assert "0.081" in text and "90.5" in text
    assert "走っていない" in text  # 2 本目は結果が無い
    assert "120 s" in text
    # 2026-10-10 段2l-10: 保持秒の列が加速秒と減速秒の間にあり、合計に含まれる
    lines = text.splitlines()
    head = next(ln for ln in lines if ln.startswith("段 |"))
    names = [c.strip() for c in head.split("|")]
    assert names[names.index("加速秒") + 1] == "保持秒"
    assert names[names.index("保持秒") + 1] == "減速秒"
    row = next(ln for ln in lines if ln.startswith(" 1 |"))
    cells = [c.strip() for c in row.split("|")]
    assert len(cells) == len(names)
    assert float(cells[names.index("保持秒")]) == pytest.approx(9.0)
    assert float(cells[names.index("合計秒")]) == pytest.approx(40.0 + 9.0 + 35.0)


def test_result_table_estimate_includes_hold_and_two_turns() -> None:
    plain = pdmod.result_table([(0.08, 90.0)], [], 10.0)
    longer = pdmod.result_table([(0.08, 90.0)], [], 10.0, hold_s=7.0, turn_s=2.0)
    est = lambda text: float(text.splitlines()[2].split("|")[3])  # noqa: E731 - 見込み秒の列
    assert est(longer) - est(plain) == pytest.approx(7.0 + 2 * 2.0, abs=1.0)


def test_build_patterns_coast_down_is_vehicle_independent_and_clamped() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = _profile(cfg)
    patterns = pdmod.build_patterns(cfg, profile, RAMP_ORDER)
    for p in patterns[:COAST_DOWN_COUNT]:
        assert p.accel_opening == min(70.0, profile.max_accel_opening)
        assert p.brake_opening == 0.0


def test_build_patterns_ignores_creep_settings() -> None:
    """クリープ発進・クリープ域ブレーキ保持は列に入らない（段2l-6）。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    patterns = pdmod.build_patterns(cfg, _profile(cfg), RAMP_ORDER)
    assert not any(isinstance(p, CreepLaunchPattern) for p in patterns)
    assert len(patterns) == COAST_DOWN_COUNT + len(RAMP_ORDER)


def test_loop_config_carries_standby_and_stop_confirm() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    loop_cfg = pdmod.loop_config_from_config(cfg, stop_confirm_pct=12.3)
    assert loop_cfg.pedal_standby is cfg.mode_drive.pedal_standby
    assert loop_cfg.stop_confirm_pct == 12.3
    cfg.mode_drive.pedal_standby = False
    assert pdmod.loop_config_from_config(cfg).pedal_standby is False


def test_loop_config_carries_ramp_one_sided_and_levels() -> None:
    """2026-10-12 段2l-12: 片側調整の有無と、上端を決める段の一覧を PatternLoopConfig へ渡す。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    loop_cfg = pdmod.loop_config_from_config(cfg)
    assert loop_cfg.ramp_one_sided is True
    assert loop_cfg.ramp_g_levels == tuple(cfg.learning.ramp_g_levels)
    cfg.learning.ramp_one_sided = False
    assert pdmod.loop_config_from_config(cfg).ramp_one_sided is False


async def test_run_pattern_drive_requires_patterns_or_ramp_order(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    with pytest.raises(cfgmod.ConfigError, match="ramp_order"):
        await pdmod.run_pattern_drive(hw, cfg, pedal=PEDAL)
    await hwmod.shutdown(hw)


# ── スタブ車両 ───────────────────────────────────────────────────────


def test_stub_vehicle_accelerates_and_brake_hold_stops() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    accel = hwmod.StubActuator("accel", connected=True)
    brake = hwmod.StubActuator("brake", connected=True)
    vehicle = hwmod.StubVehicle(accel=accel, brake=brake, params=feedforward_params(cfg))
    accel.position = opening_to_pulse(40.0)
    speed = vehicle.advance(0.0, now=0.0)
    for i in range(1, 21):
        speed = vehicle.advance(speed, now=i * 0.1)
    assert speed > 10.0

    accel.position = 0
    brake.position = opening_to_pulse(30.0)
    for i in range(21, 61):
        speed = vehicle.advance(speed, now=i * 0.1)
    assert speed == 0.0


def test_stub_vehicle_ignores_pedal_inside_play() -> None:
    """遊びの中の踏み込みは車速に効かない（ペダル探索が測る対象）。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    accel = hwmod.StubActuator("accel", connected=True)
    brake = hwmod.StubActuator("brake", connected=True)
    free = hwmod.StubVehicle(accel=accel, brake=brake, params=feedforward_params(cfg))
    pressed = hwmod.StubVehicle(
        accel=hwmod.StubActuator("accel", position=opening_to_pulse(5.0), connected=True),
        brake=brake,
        params=feedforward_params(cfg),
    )
    free.advance(3.0, now=0.0)
    pressed.advance(3.0, now=0.0)
    assert free.advance(3.0, now=0.1) == pressed.advance(3.0, now=0.1)


# ── 2-1. パターン走行 ────────────────────────────────────────────────


async def test_pattern_drive_runs_pattern_loop_and_holds_stop(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)

    result = await pdmod.run_pattern_drive(
        hw, cfg, pedal=PEDAL, patterns=SHORT_PATTERNS, loop_config=FAST_LOOP
    )

    lines = result.csv_path.read_text(encoding="utf-8").splitlines()
    assert lines[0].split(",") == list(dlmod.CSV_COLUMNS)
    column = dlmod.CSV_COLUMNS.index("section")
    sections = [line.split(",")[column] for line in lines[1:]]
    assert sections.count(dlmod.SECTION_PATTERN_DRIVE) == len(result.samples) >= 20
    assert max(s.data.actual_speed_kmh for s in result.samples) > 5.0
    kinds = {s.pattern.split(":")[1] for s in result.samples}
    assert kinds >= {"CREEP", "CREEP_SETTLE", "COAST_DOWN"}
    # 走行後の緩減速〜停車保持も同じ CSV の後ろに残る
    assert sections[-1] == dlmod.SECTION_DECEL_TO_STOP
    assert result.stop is not None
    assert not result.csv_path.with_suffix(".png").exists()  # output.plot=false
    # 走行後: アクセル 0%・停車保持開度（2-0 の値）・停車
    assert hw.accel.position == 0
    assert hw.brake.position == opening_to_pulse(PEDAL.stop_brake_opening_pct)
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_pattern_drive_emergency_releases_pedals(tmp_path: Path) -> None:
    """CAN が途中で読めなくなったら PatternLoop が非常停止し、ペダルを離して止まる。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    original = hw.can.read_speed
    calls = 0

    async def flaky_read() -> float:
        nonlocal calls
        calls += 1
        if calls > 15:
            raise TimeoutError("CAN 車速が 0.2s 更新されていません")
        return await original()

    hw.can.read_speed = flaky_read  # type: ignore[method-assign]

    with pytest.raises(pdmod.DriveError, match="非常停止"):
        await pdmod.run_pattern_drive(
            hw, cfg, pedal=PEDAL, patterns=SHORT_PATTERNS, loop_config=FAST_LOOP
        )
    assert hw.accel.position == 0 and hw.brake.position == 0  # 原点復帰済み
    # 途中までのログも残す（原因調査用）
    assert list((tmp_path / "results").glob("drive_log_stub_*.csv"))
    await hwmod.shutdown(hw)


def _ramp_monitor(
    cfg: cfgmod.ResearchConfig, rs: RampStatus | None
) -> tuple[pdmod._DriveMonitor, list[dict[str, Any]]]:
    """記録された record の引数を集める偽の走行ログを持つモニター。"""
    calls: list[dict[str, Any]] = []

    class _FakeLog:
        def record(self, data: DriveLogData, **kw: Any) -> dlmod.DriveSample:
            calls.append(kw)
            return dlmod.DriveSample(
                elapsed_s=0.0, timestamp=datetime.now(tz=UTC), data=data,
                section="PATTERN_DRIVE", phase="X",
                accel_kp=kw["accel_kp"], accel_ki=kw["accel_ki"],
                brake_kp=kw["brake_kp"], brake_ki=kw["brake_ki"],
            )

    monitor = pdmod._DriveMonitor(cfg, [], _FakeLog())  # type: ignore[arg-type]
    monitor.ramp_status = lambda: rs
    return monitor, calls


def test_on_sample_records_ramp_gains_or_none(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    data = DriveLogData(
        ref_speed_kmh=30.0, actual_speed_kmh=29.5, accel_opening=1.0, brake_opening=0.0,
        accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
    )
    rs = RampStatus(ref_kmh=30.0, accel_kp=0.25, accel_ki=0.03125, brake_kp=0.5, brake_ki=0.0625)
    monitor, _ = _ramp_monitor(cfg, rs)
    monitor.on_sample(data, 0, "RAMP_UP")
    s = monitor.samples[-1]
    assert (s.accel_kp, s.accel_ki, s.brake_kp, s.brake_ki) == (0.25, 0.03125, 0.5, 0.0625)

    monitor, _ = _ramp_monitor(cfg, None)
    monitor.on_sample(data, 0, "MEASURE")
    s = monitor.samples[-1]
    assert (s.accel_kp, s.accel_ki, s.brake_kp, s.brake_ki) == (None, None, None, None)


# ── CSV・図 ──────────────────────────────────────────────────────────


def test_csv_roundtrip_to_drive_logs(tmp_path: Path) -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    samples = _synthetic_samples(cfg)[:30]
    path = tmp_path / "drive.csv"
    dlmod.write_csv(samples, path)

    logs = dlmod.read_drive_logs(path)
    assert len(logs) == 30
    assert logs[1].timestamp - logs[0].timestamp == timedelta(seconds=0.1)
    assert logs[5].accel_opening == pytest.approx(samples[5].data.accel_opening)
    assert logs[5].actual_speed_kmh == pytest.approx(samples[5].data.actual_speed_kmh, abs=1e-3)
    assert logs[0].ref_speed_kmh is None


def test_save_drive_figure_writes_png(tmp_path: Path) -> None:
    samples = [PlotSample(i * 0.1, None, i * 0.5, 20.0, 0.0) for i in range(50)]
    path = save_drive_figure(
        samples, tmp_path / "fig.png", title="テスト", max_speed_kmh=140.0, has_ref=False
    )
    assert path.stat().st_size > 1000


# ── 2-2. モデル作成 ──────────────────────────────────────────────────


def _fake_table_model(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """C7 の表づくり（fwd_tool）を差し替える。ここで見るのは物理定数の推定と保存の流れ。"""

    def fake(profile, csv_path, models_dir, hw_mode, *, announce=True, lags=None):  # type: ignore[no-untyped-def]
        path = tmp_path / "results" / "models" / f"fwd_table_fake_{hw_mode}.pkl"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"x")
        return pdmod._BuiltModel(model_path=str(path), metrics={})

    monkeypatch.setattr(pdmod, "_build_table_model", fake)
    # 偽の pkl（中身 b"x"）は学習結果のチェックでは読めない。チェックそのものは
    # test_research_step2_check.py で確認するので、ここでは空（NG なし）にする
    monkeypatch.setattr(pdmod, "_run_step2_checks", lambda **kwargs: [])
    monkeypatch.setattr(
        step2_check, "check_model_path",
        lambda cfg, saved: step2_check.CheckItem("C2", "model_path", step2_check.STATUS_OK),
    )


def test_build_ff_model_real_saves_model_but_not_measured_values(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _tmp_cfg(tmp_path)
    _fake_table_model(monkeypatch, tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    saved = cfgmod.load_config(cfg.source_path)
    assert saved.feedforward.model_path == result.model_path
    assert saved.feedforward.is_model_trained
    assert saved.feedforward.engine_brake_decel_kmhs == pytest.approx(
        result.params.engine_brake_decel_kmhs, rel=1e-5
    )
    # 停車保持開度・クリープ平衡車速は 2-0 の実測が正。推定値では上書きしない
    assert pdmod.MEASURED_KEYS == {"stop_brake_opening_pct", "creep_speed_kmh"}
    for key in pdmod.MEASURED_KEYS:
        # 2026-10-08 段2l-7: yaml のキーは stop_brake_offset_pct（min からの量）
        yaml_key = "stop_brake_offset_pct" if key == "stop_brake_opening_pct" else key
        assert getattr(saved.feedforward, yaml_key) == getattr(cfg.feedforward, yaml_key)
        assert not any(line.startswith(f"feedforward.{yaml_key}:") for line in result.changed)
    # 不感帯の推定値は yaml に書かない（キー自体が無い。u の原点は vehicle.min_*_opening_pct）
    assert "accel_deadband_pct" not in pdmod.FF_PARAM_KEYS
    assert "brake_deadband_pct" not in pdmod.FF_PARAM_KEYS
    assert not any("deadband" in line for line in result.changed)
    # ペダルゲインは研究側の推定（min + 0.5% 以上）で両側とも同定し、グリッドと同じ点数で書き戻す
    ff = saved.feedforward
    assert len(ff.pedal_gain_speeds_kmh) >= 2
    assert len(ff.accel_gain_kmhs_per_pct) == len(ff.pedal_gain_speeds_kmh)
    assert len(ff.brake_gain_kmhs_per_pct) == len(ff.pedal_gain_speeds_kmh)
    assert not [p for p in cfgmod.validate_config(saved) if "pedal_gain" in p]
    # ユーザーが編集するファイルなのでコメントは消さない
    assert "# 2次多項式 Ridge 逆モデル" in cfg.source_path.read_text(encoding="utf-8")


def test_build_ff_model_order_and_reference_basis_for_coast_creep_gain(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段2.5（ProblemReport_20260916）: 推定順は 惰行 → クリープ → ペダルゲイン。

    クリープ加速カーブ・ペダルゲインの基準（reference）は、estimate_dynamics_params が返す
    生の `after`（u の原点の推定値が表示専用の粗い値）ではなく、**before の u の原点（min）・
    停車保持・creep_speed_kmh を保ったまま惰行カーブだけ差し替えたもの**であること。
    2026-10-08 段2l-6: before の u の原点は yaml の vehicle.min_*_opening_pct。
    """
    cfg = _tmp_cfg(tmp_path)
    _fake_table_model(monkeypatch, tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    calls: list[str] = []
    captured: dict[str, Any] = {}
    original_dynamics = pdmod.estimate_dynamics_params
    original_coast = pdmod._estimate_research_coast_curve
    original_creep = pdmod._estimate_research_creep_curve
    original_gain = pdmod._estimate_research_pedal_gains

    def spy_dynamics(logs, current):  # type: ignore[no-untyped-def]
        captured["before"] = current
        return original_dynamics(logs, current)

    def spy_coast(cfg_, logs, reference, target):  # type: ignore[no-untyped-def]
        calls.append("coast")
        captured["coast_reference"] = reference
        result = original_coast(cfg_, logs, reference, target)
        captured["coast_result"] = result
        return result

    def spy_creep(cfg_, logs, params, before):  # type: ignore[no-untyped-def]
        calls.append("creep")
        captured["creep_params"] = params
        return original_creep(cfg_, logs, params, before)

    def spy_gain(cfg_, logs, before, after, research):  # type: ignore[no-untyped-def]
        calls.append("gain")
        captured["gain_before"] = before
        captured["gain_after"] = after
        return original_gain(cfg_, logs, before, after, research)

    monkeypatch.setattr(pdmod, "estimate_dynamics_params", spy_dynamics)
    monkeypatch.setattr(pdmod, "_estimate_research_coast_curve", spy_coast)
    monkeypatch.setattr(pdmod, "_estimate_research_creep_curve", spy_creep)
    monkeypatch.setattr(pdmod, "_estimate_research_pedal_gains", spy_gain)

    pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    assert calls == ["coast", "creep", "gain"]

    before = captured["before"]
    coast_result = captured["coast_result"]

    # 惰行カーブ同定そのもののサンプル抽出条件は before を渡す
    assert captured["coast_reference"] is before

    # クリープ・ゲインの基準は before の値そのもの（estimate_dynamics_params の粗い推定値
    # ではない）を保っている。u の原点は yaml の min
    for ref in (captured["creep_params"], captured["gain_before"]):
        assert ref.accel_deadband_pct == pytest.approx(MIN_ACCEL_PCT)
        assert ref.brake_deadband_pct == pytest.approx(MIN_BRAKE_PCT)
        assert ref.accel_deadband_pct == pytest.approx(before.accel_deadband_pct)
        assert ref.brake_deadband_pct == pytest.approx(before.brake_deadband_pct)
        assert ref.creep_speed_kmh == pytest.approx(before.creep_speed_kmh)
        assert ref.stop_brake_opening_pct == pytest.approx(before.stop_brake_opening_pct)
        # 惰行カーブだけは今回同定した結果と一致する
        assert ref.coast_decel_speeds_kmh == coast_result.coast_decel_speeds_kmh
        assert ref.coast_decel_kmhs == coast_result.coast_decel_kmhs

    assert captured["gain_after"] is coast_result


def test_build_ff_model_does_not_estimate_stop_brake_floor(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段2l-6: クリープ域ブレーキ保持を列から外したので、stop_brake_floor は推定せず、
    stop_brake_floor_offset_pct は yaml にも書き戻しにも出ない（2026-10-09 に yaml から削除）。"""
    cfg = _tmp_cfg(tmp_path)
    _fake_table_model(monkeypatch, tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    def must_not_call(*args: object, **kwargs: object) -> None:
        raise AssertionError("stop_brake_floor の推定は呼ばない")

    monkeypatch.setattr(pdmod, "_estimate_research_stop_brake_floor", must_not_call)
    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    assert result.research_params.stop_brake_floor_offset_pct == 0.0
    assert "stop_brake_floor_offset_pct" not in pdmod.FF_PARAM_KEYS
    assert not any("stop_brake_floor_offset_pct" in line for line in result.changed)


def test_build_ff_model_stub_does_not_touch_yaml(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _tmp_cfg(tmp_path)
    _fake_table_model(monkeypatch, tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    before = cfg.source_path.read_text(encoding="utf-8")

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_STUB)

    assert cfg.source_path.read_text(encoding="utf-8") == before
    assert "_stub" in Path(result.model_path).name
    assert result.changed == []


def _ng_item(item_id: str = "B5", **metrics: float) -> step2_check.CheckItem:
    return step2_check.CheckItem(
        item_id, "テスト項目", step2_check.STATUS_NG, "値", "相手", "理由", dict(metrics)
    )


def test_build_ff_model_check_ng_real_keeps_yaml_and_raises(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """チェックが NG なら、実機でも yaml を書き換えず Step2CheckError（pkl は残る）。"""
    cfg = _tmp_cfg(tmp_path)
    _fake_table_model(monkeypatch, tmp_path)
    monkeypatch.setattr(pdmod, "_run_step2_checks", lambda **kwargs: [_ng_item("B5")])
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    before = cfg.source_path.read_bytes()

    with pytest.raises(step2_check.Step2CheckError) as info:
        pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    assert cfg.source_path.read_bytes() == before
    assert [i.id for i in info.value.items] == ["B5"]
    assert (tmp_path / "results" / "models" / "fwd_table_fake_real.pkl").exists()


def test_build_ff_model_check_ng_stub_does_not_raise(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """スタブは NG でも止めない（表示のみ）。yaml は今までどおり書き換えない。"""
    cfg = _tmp_cfg(tmp_path)
    _fake_table_model(monkeypatch, tmp_path)
    monkeypatch.setattr(pdmod, "_run_step2_checks", lambda **kwargs: [_ng_item("B5")])
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    before = cfg.source_path.read_bytes()

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_STUB)

    assert cfg.source_path.read_bytes() == before
    assert result.changed == [] and [c.id for c in result.checks] == ["B5"]
    assert "スタブのため止めません" in capsys.readouterr().out


def test_build_ff_model_post_write_c2_ng_raises(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """yaml を書いた後に読み直した C2 が NG なら Step2CheckError。"""
    cfg = _tmp_cfg(tmp_path)
    _fake_table_model(monkeypatch, tmp_path)
    monkeypatch.setattr(
        step2_check, "check_model_path",
        lambda c, saved: step2_check.CheckItem(
            "C2", "model_path", step2_check.STATUS_NG, "x", "y", "違う", {"path_diff": 1.0}
        ),
    )
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    with pytest.raises(step2_check.Step2CheckError) as info:
        pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    assert [i.id for i in info.value.items] == ["C2"]


def _c8_cfg(tmp_path: Path) -> cfgmod.ResearchConfig:
    cfg = _tmp_cfg(tmp_path)
    cfg.save({"feedforward.candidate": "C8"})
    return cfgmod.load_config(cfg.source_path)


def _seed_outlier_setup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, check_results: list[list[Any]]
) -> tuple[list[tuple[int, ...]], list[int]]:
    """C8 の学習（_build_forward_model）とチェックを差し替える。学習に渡った乱数の列と、
    チェックを呼んだ回数を返す（学習の中身はモック）。"""
    cfg = _c8_cfg(tmp_path)
    seeds_seen: list[tuple[int, ...]] = []
    check_calls: list[int] = []

    def fake_forward(  # type: ignore[no-untyped-def]
        profile, csv_path, models_dir, hw_mode, *, workers, seeds, max_iter, lags=None,
        creep_launch_min_speed_kmh=None, side_table_opening_source="actual",
        stop_release_layer_enabled=False, accel_release_input_enabled=False,
        launch_accel_side_source="extrapolate", side_table_accel_window_s=0.5,
    ):
        seeds_seen.append(tuple(seeds))
        path = tmp_path / "results" / "models" / f"fwd_model_fake_{len(seeds_seen)}.pkl"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"x")
        return pdmod._BuiltModel(model_path=str(path), metrics={})

    def fake_checks(**kwargs: Any) -> list[Any]:
        check_calls.append(1)
        return check_results[min(len(check_calls), len(check_results)) - 1]

    monkeypatch.setattr(pdmod, "_build_forward_model", fake_forward)
    monkeypatch.setattr(pdmod, "_run_step2_checks", fake_checks)
    monkeypatch.setattr(pdmod, "_outlier_seeds", lambda model_path: [1])
    monkeypatch.setattr(
        step2_check, "check_model_path",
        lambda c, saved: step2_check.CheckItem("C2", "model_path", step2_check.STATUS_OK),
    )
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    return seeds_seen, check_calls


def test_build_ff_model_seed_outlier_retrains_once_then_ok(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    outlier = _ng_item("B1", seed_outlier_count=1.0)
    seeds_seen, check_calls = _seed_outlier_setup(tmp_path, monkeypatch, [[outlier], []])
    cfg = _c8_cfg(tmp_path)
    result = pdmod.build_ff_model(
        cfg, tmp_path / "drive.csv", hw_mode=hwmod.HW_REAL, pedal=PEDAL, forward_seeds=(0, 1, 2)
    )
    # 学習は 1 回目（_build_forward_model）と、外れた乱数 1 を 101 に替えた 1 回だけ
    assert seeds_seen == [(0, 1, 2), (0, 101, 2)]
    assert len(check_calls) == 2
    assert result.model_path.endswith("fwd_model_fake_2.pkl")
    assert result.changed  # yaml は書き換えた


def test_build_ff_model_seed_outlier_still_ng_raises_without_third_training(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    outlier = _ng_item("B1", seed_outlier_count=1.0)
    seeds_seen, check_calls = _seed_outlier_setup(tmp_path, monkeypatch, [[outlier], [outlier]])
    cfg = _c8_cfg(tmp_path)
    before = cfg.source_path.read_bytes()
    with pytest.raises(step2_check.Step2CheckError):
        pdmod.build_ff_model(
            cfg, tmp_path / "drive.csv", hw_mode=hwmod.HW_REAL, pedal=PEDAL,
            forward_seeds=(0, 1, 2),
        )
    assert len(check_calls) == 2 and len(seeds_seen) == 2  # 学習し直しは 1 回だけ
    assert seeds_seen[1] == (0, 101, 2)
    assert cfg.source_path.read_bytes() == before


def test_build_ff_model_other_ng_does_not_retrain(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """B1 以外にも NG があるとき、または B1 でも乱数の外れが原因でないときは学習し直さない。"""
    for results in (
        [_ng_item("B1", seed_outlier_count=1.0), _ng_item("B5")],
        [_ng_item("B1", seed_outlier_count=0.0)],
    ):
        seeds_seen, check_calls = _seed_outlier_setup(tmp_path, monkeypatch, [results])
        cfg = _c8_cfg(tmp_path)
        with pytest.raises(step2_check.Step2CheckError):
            pdmod.build_ff_model(
                cfg, tmp_path / "drive.csv", hw_mode=hwmod.HW_REAL, pedal=PEDAL,
                forward_seeds=(0, 1, 2),
            )
        assert len(seeds_seen) == 1 and len(check_calls) == 1


def test_replace_seeds_avoids_collisions() -> None:
    assert pdmod._replace_seeds((0, 1, 2), [1]) == (0, 101, 2)
    assert pdmod._replace_seeds((0, 100, 200), [0]) == (300, 100, 200)
    assert pdmod._replace_seeds((0, 1), []) == (0, 1)


@pytest.mark.parametrize("candidate", ["C1", "C2", "C3", "C4", "C5"])
def test_build_ff_model_rejects_polynomial_candidates(tmp_path: Path, candidate: str) -> None:
    """段2l-6: C1〜C5 は手順2 の対象外。CSV を読む前に ConfigError。"""
    cfg = _tmp_cfg(tmp_path)
    cfg.save({"feedforward.candidate": candidate})
    cfg = cfgmod.load_config(cfg.source_path)
    with pytest.raises(cfgmod.ConfigError, match="手順 2 の対象外"):
        pdmod.build_ff_model(cfg, tmp_path / "missing.csv", hw_mode=hwmod.HW_STUB)


def _tiny_wltp_mode() -> DrivingMode:
    return DrivingMode(
        id="wltp", name="wltp", description="", total_duration=20.0, max_speed=40.0,
        created_at=datetime(2026, 9, 25, tzinfo=UTC), is_system=False,
        reference_speed=[
            SpeedPoint(time_s=0.0, speed_kmh=0.0), SpeedPoint(time_s=20.0, speed_kmh=40.0),
        ],
    )


# ── CLI 経由 ─────────────────────────────────────────────────────────


async def _fake_search(hw: object, cfg: object, **kwargs: object) -> PedalSearchResult:
    return PEDAL


async def _fake_pre_check(hw: object, cfg: object, **kwargs: object) -> int:
    """走行前チェックは test_research_pre_drive_check.py で確認する。"""
    return 0


async def _fake_load_mode(cfg: object, name: str) -> DrivingMode:
    """手順2 は走る前に WLTP の基準車速を DB から読む。テストでは DB を使わない。"""
    return _tiny_wltp_mode()


def test_step2_requires_step1(capsys: pytest.CaptureFixture[str]) -> None:
    assert mainmod.main(["--only", "2"]) == 4
    assert "手順 1 を先に実行" in capsys.readouterr().out


def test_step2_with_polynomial_candidate_stops_before_driving(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """段2l-6: C1〜C5 は走る前（走行前チェックの前）に「手順2 の対象外」で止まる。"""

    async def never(hw: object, cfg: object, **kwargs: object) -> None:
        raise AssertionError("ハードを動かしてはいけない")

    cfg = _tmp_cfg(tmp_path)
    cfg.save({"feedforward.candidate": "C1"})
    monkeypatch.setattr(mainmod, "run_pre_drive_check", never)
    monkeypatch.setattr(mainmod, "load_mode", never)
    monkeypatch.setattr(mainmod, "run_pedal_search", never)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 2
    out = capsys.readouterr().out
    assert "手順 2 の対象外" in out and "C1" in out


def test_drive_error_exit_code_is_5(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def failing_search(hw: object, cfg: object, **kwargs: object) -> None:
        raise pdmod.DriveError("テスト用の探索失敗")

    cfg = _tmp_cfg(tmp_path)  # 走行ログを tmp に書く
    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", _fake_load_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", failing_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 5
    out = capsys.readouterr().out
    assert "走行エラー: テスト用の探索失敗" in out
    assert "終了処理: 完了" in out  # 失敗しても原点復帰・サーボOFF まで行く


def test_step2_loads_every_coverage_mode_before_the_pedal_search(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """手順2 は modes.coverage_mode_names の全モードを走る前に読む（重複して読まない）。"""
    loaded: list[str] = []
    events: list[str] = []

    async def recording_load_mode(cfg: object, name: str) -> DrivingMode:
        loaded.append(name)
        return _tiny_wltp_mode()

    async def failing_search(hw: object, cfg: object, **kwargs: object) -> None:
        events.append("pedal_search")
        raise pdmod.DriveError("テスト用の探索失敗")

    cfg = _tmp_cfg(tmp_path)
    wltp = cfg.modes.wltp_mode_name
    cfg.save({"modes.coverage_mode_names": [wltp, "09_US06"]})
    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", recording_load_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", failing_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 5
    assert loaded == [wltp, "09_US06"]  # WLTP は 1 回だけ（MAE 用と共用）
    assert events == ["pedal_search"]
    out = capsys.readouterr().out
    assert "上端の車速" in out  # 走る前に G ランプの段の表を出す


def test_step2_stops_before_driving_when_a_coverage_mode_is_missing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def missing_mode(cfg: object, name: str) -> DrivingMode:
        if name == "09_US06":
            raise cfgmod.ConfigError("走行モード '09_US06' が driving_modes にありません")
        return _tiny_wltp_mode()

    async def never_search(hw: object, cfg: object, **kwargs: object) -> None:
        raise AssertionError("ペダル探索まで進んではいけない")

    cfg = _tmp_cfg(tmp_path)
    cfg.save({"modes.coverage_mode_names": [cfg.modes.wltp_mode_name, "09_US06"]})
    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", missing_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", never_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) != 0
    assert "09_US06" in capsys.readouterr().out


def test_model_error_exit_code_is_6(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def fake_drive(hw: object, cfg: object, **kwargs: object) -> pdmod.PatternDriveResult:
        return pdmod.PatternDriveResult(csv_path=tmp_path / "x.csv", samples=[], duration_s=0.0)

    def failing_build(cfg: object, csv_path: object, **kwargs: object) -> None:
        raise LearningDataError("学習サンプルが不足しています (3 点)")

    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", _fake_load_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", _fake_search)
    monkeypatch.setattr(mainmod, "run_pattern_drive", fake_drive)
    monkeypatch.setattr(mainmod, "build_ff_model", failing_build)
    cfg = _tmp_cfg(tmp_path)  # 走行ログを tmp に書く
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 6
    assert "モデル作成エラー" in capsys.readouterr().out


def test_step2_check_error_exit_code_is_8(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def fake_drive(hw: object, cfg: object, **kwargs: object) -> pdmod.PatternDriveResult:
        return pdmod.PatternDriveResult(csv_path=tmp_path / "x.csv", samples=[], duration_s=0.0)

    seen: dict[str, object] = {}

    def failing_build(cfg: object, csv_path: object, **kwargs: object) -> None:
        seen.update(kwargs)
        raise step2_check.Step2CheckError([_ng_item("B5")])

    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", _fake_load_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", _fake_search)
    monkeypatch.setattr(mainmod, "run_pattern_drive", fake_drive)
    monkeypatch.setattr(mainmod, "build_ff_model", failing_build)
    cfg = _tmp_cfg(tmp_path)
    # 手順3 まで指定しても、NG なら手順3 には進まない
    assert mainmod.main(["--steps", "1,2,3", "--config", str(cfg.source_path)]) == 8
    out = capsys.readouterr().out
    assert "学習結果のチェックエラー" in out and "B5" in out
    assert "手順 3:" not in out
    assert seen["ramp_results"] == []  # 走行の結果を渡している


async def test_pattern_drive_passes_yaml_loop_interval_to_pattern_loop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """手順2 も yaml の control.loop_interval_ms（50 ms）の周期で回す（段2l-10）。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    seen: dict[str, object] = {}

    class Stop(Exception):
        pass

    def spy(*args: object, **kwargs: object) -> object:
        seen.update(kwargs)
        raise Stop

    monkeypatch.setattr(pdmod, "PatternLoop", spy)
    with pytest.raises(Stop):
        await pdmod.run_pattern_drive(
            hw, cfg, pedal=PEDAL, patterns=SHORT_PATTERNS, loop_config=FAST_LOOP
        )
    assert seen["interval_s"] == pytest.approx(cfg.control.loop_interval_s)
    assert cfg.control.loop_interval_s == pytest.approx(0.05)
    await hwmod.shutdown(hw)


def test_retry_summary_uses_the_difference_from_the_start() -> None:
    start = {"アクセル": ({"read:MONITOR": 2}, {"read:MONITOR": 100}), "ブレーキ": ({}, {})}
    end = {
        "アクセル": ({"read:MONITOR": 5, "move_to_position": 1},
                  {"read:MONITOR": 1100, "move_to_position": 50}),
        "ブレーキ": ({}, {"read:MONITOR": 900}),
    }
    line = pdmod._retry_summary(start, end)
    assert "アクセル move_to_position 1/50 回・read:MONITOR 3/1000 回" in line
    assert "ブレーキ read:MONITOR 0/900 回" in line  # 0 回でも出す
    assert pdmod._retry_summary({}, {"アクセル": ({}, {})}).endswith("アクセル 通信なし")


# ── 遅れの測定の段・途中の保持（2026-10-13 段2o-2）────────────────────────


def test_build_patterns_adds_lag_step_ramps_only_when_asked() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    plain = pdmod.build_patterns(cfg, _profile(cfg), RAMP_ORDER)
    with_lag = pdmod.build_patterns(cfg, _profile(cfg), RAMP_ORDER, with_lag=True)
    lr = cfg.learning
    assert len(with_lag) == len(plain) + len(lr.ramp_lag_speeds_kmh)
    assert not any(isinstance(p, GRampPattern) and p.lag_step for p in plain)
    tail = with_lag[len(plain):]
    assert all(isinstance(p, GRampPattern) and p.lag_step for p in tail)
    assert [p.top_kmh for p in tail if isinstance(p, GRampPattern)] == [
        float(v) for v in lr.ramp_lag_speeds_kmh
    ]
    assert {p.target_g for p in tail if isinstance(p, GRampPattern)} == {lr.ramp_lag_approach_g}
    assert [lag for *_, lag in pdmod.ramp_order_of(with_lag)][-len(tail):] == [True] * len(tail)


def test_build_patterns_passes_stations_from_three_element_order() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    order = [(0.02, 100.0, (15.0, 25.0)), (0.08, 90.0, ())]
    patterns = pdmod.build_patterns(cfg, _profile(cfg), order)
    ramps = [p for p in patterns if isinstance(p, GRampPattern)]
    assert [p.stations_kmh for p in ramps] == [(15.0, 25.0), ()]


def test_loop_config_carries_cruise_and_lag_settings() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    lc = pdmod.loop_config_from_config(cfg)
    lr = cfg.learning
    assert (lc.ramp_cruise_band_g, lc.ramp_cruise_hold_s, lc.ramp_cruise_timeout_s) == (
        lr.ramp_cruise_band_g, lr.ramp_cruise_hold_s, lr.ramp_cruise_timeout_s
    )
    assert (lc.ramp_lag_step_g, lc.ramp_lag_step_hold_s) == (
        lr.ramp_lag_step_g, lr.ramp_lag_step_hold_s
    )


def test_result_table_shows_cruise_and_lag_columns() -> None:
    from tests.research.pattern_loop import RampResult

    r = RampResult(
        pattern_idx=0, target_g=0.02, top_kmh=100.0, up_s=300.0, down_s=290.0,
        accel_g_median=0.02, decel_g_median=0.02, max_speed_kmh=100.0, hold_s=9.0,
        cruise_s=150.0, cruise_n=20, cruise_done=18,
    )
    text = pdmod.result_table(
        [(0.02, 100.0, (15.0,), False), (0.125, 30.0, (), True)], [r], 800.0, hold_s=7.0,
        turn_s=2.0,
    )
    assert "途中保持" in text and "18/20" in text and "30L" in text
