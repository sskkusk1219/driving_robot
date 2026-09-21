"""研究開発用ハーネス 手順 2-1/2-2（パターン走行 → FF モデル作成）のユニットテスト。

スタブ HW（StubVehicle 付き）で研究用の状態機械 PatternLoop（本番 LearningLoop のアルゴリズムを
移植した自前実装）を短いパターン列で回し、走行後の停車保持・非常停止時のペダル解放・CSV・
モデル作成と YAML 書き戻しを確認する。
2-0 のペダル探索は test_research_pedal_search.py で確認し、ここでは結果を与えて始める。
"""

from __future__ import annotations

import copy
import pickle
from dataclasses import replace
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

import pytest

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import MODEL_TYPE
from src.models.drive_log import DriveLogData
from src.models.learning_drive import LearningPattern, PatternKind
from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import hardware as hwmod
from tests.research import main as mainmod
from tests.research import pattern_drive as pdmod
from tests.research.live_plot import PlotSample, save_drive_figure
from tests.research.pattern_loop import (
    CreepLaunchPattern,
    CruiseStairPattern,
    LowOpenStairPattern,
    PatternLoopConfig,
    SpeedTargetPattern,
    TrimStairPattern,
)
from tests.research.pedal_search import PedalSearchResult
from tests.research.vehicle import (
    STROKE_LIMIT_PULSE,
    build_vehicle_profile,
    feedforward_params,
    opening_to_pulse,
)

# 2-0 のペダル探索が終わった想定の結果（スタブの真の遊び 6%/8% に近い値）
PEDAL = PedalSearchResult(
    creep_speed_kmh=5.0,
    accel_deadband_pct=6.3,
    brake_deadband_pct=8.4,
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
)
SHORT_PATTERNS = [
    LearningPattern(PatternKind.CREEP, accel_opening=0.0, brake_opening=22.0, hold_duration_s=0.3),
    LearningPattern(
        PatternKind.CREEP_SETTLE, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.3
    ),
    LearningPattern(
        PatternKind.ACCEL_SWEEP, accel_opening=40.0, brake_opening=30.0, hold_duration_s=0.3
    ),
]


def _tmp_cfg(tmp_path: Path) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
            "output.plot": False,
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
    assert ffp.stop_brake_opening_pct == cfg.feedforward.stop_brake_opening_pct
    assert ffp.brake_deadband_pct == cfg.feedforward.brake_deadband_pct


def test_build_patterns_uses_deadband_plus_offsets() -> None:
    """ペダルの固定開度は本番の絶対値ではなく、2-0 の不感帯 + YAML の offset。"""
    cfg = _without_a3a4(cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH))
    patterns = pdmod.build_patterns(cfg, PEDAL.apply_to_profile(build_vehicle_profile(cfg)))
    lr = cfg.learning

    def openings(kind: PatternKind, attr: str) -> list[float]:
        # 定速階段（CruiseStairPattern）は kind が同じ CRUISE_TRIM だが trim_opening を使わない
        return [
            getattr(p, attr) for p in patterns
            if p.kind is kind and not isinstance(p, CruiseStairPattern)
        ]

    def above(deadband: float, offsets: list[float]) -> list[float]:
        return [deadband + offset for offset in offsets]

    assert openings(PatternKind.ACCEL_DEADBAND_PROBE, "accel_opening") == pytest.approx(
        above(PEDAL.accel_deadband_pct, lr.accel_deadband_probe_offsets_pct)
    )
    assert openings(PatternKind.CRUISE_TRIM, "trim_opening") == pytest.approx(
        above(PEDAL.accel_deadband_pct, lr.cruise_trim_offsets_pct)
    )
    assert openings(PatternKind.BRAKE_HOLD, "brake_opening") == pytest.approx(
        above(PEDAL.brake_deadband_pct, lr.brake_hold_offsets_pct + lr.brake_hold_low_offsets_pct)
    )


def _without_a3a4(cfg: cfgmod.ResearchConfig) -> cfgmod.ResearchConfig:
    plain = copy.deepcopy(cfg)
    plain.learning.trim_stair_start_kmh = []
    plain.learning.brake_hold_hard_offsets_pct = []
    # 2026-09-19 低開度階段（段3-1）も TrimStairPattern の継承で本番の段と紛れるため、
    # ここで一緒に無効化する（別途 test_build_patterns_adds_low_open_stair_* で確認する）
    plain.learning.low_open_stair_offsets_pct = []
    return plain


def _without_additions(cfg: cfgmod.ResearchConfig) -> cfgmod.ResearchConfig:
    plain = _without_a3a4(cfg)
    plain.learning.accel_sweep_add_offsets_pct = []
    plain.learning.brake_hold_low_offsets_pct = []
    return plain


def test_build_patterns_adds_low_sweeps_and_low_speed_brake_holds() -> None:
    """A2・A5: 本番の段は残し、低開度の ACCEL_SWEEP と 60 km/h からの BRAKE_HOLD を足す。"""
    cfg = _without_a3a4(cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH))
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    base = pdmod.build_patterns(_without_additions(cfg), profile)
    lr = cfg.learning
    n_sweep, n_low = len(lr.accel_sweep_add_offsets_pct), len(lr.brake_hold_low_offsets_pct)
    assert len(patterns) == len(base) + n_sweep + n_low

    sweeps = [p.accel_opening for p in patterns if p.kind is PatternKind.ACCEL_SWEEP]
    added = [PEDAL.accel_deadband_pct + o for o in lr.accel_sweep_add_offsets_pct]
    assert sweeps[:n_sweep] == pytest.approx(added)  # 本番の段（上限の割合）の前
    assert sweeps[n_sweep:] == pytest.approx([0.3 * 80.0, 0.5 * 80.0, 0.7 * 80.0, 80.0])
    first_sweep = next(i for i, p in enumerate(patterns) if p.kind is PatternKind.ACCEL_SWEEP)
    assert patterns[first_sweep - 1].kind is PatternKind.ACCEL_DEADBAND_PROBE

    holds = [i for i, p in enumerate(patterns) if p.kind is PatternKind.BRAKE_HOLD]
    low = [i for i in holds if isinstance(patterns[i], SpeedTargetPattern)]
    assert low == holds[-n_low:]  # cap からの BRAKE_HOLD の後
    assert all(
        getattr(patterns[i], "accel_target_kmh", None) == lr.brake_hold_low_start_kmh for i in low
    )
    assert patterns[low[-1] + 1].kind is PatternKind.COAST_DOWN
    # 追加分を除けば今までと同じ並び
    rest = [p for i, p in enumerate(patterns)
            if i not in low and not (p.kind is PatternKind.ACCEL_SWEEP
                                     and p.accel_opening in added)]
    assert rest == base


def test_build_patterns_adds_hard_brake_holds_and_trim_stairs() -> None:
    """A3・A4: A5 の段の後に 20 km/h からの高ブレーキ 4 本、末尾にトリム階段 3 本（41 本）。

    定速階段（段2）・クリープ発進/ブレーキ保持（段1）・低開度階段（段3-1）は本テストの対象外
    なので無効化する（別途 test_build_patterns_adds_cruise_stair /
    test_build_patterns_adds_creep_launches / test_build_patterns_adds_low_open_stair_*
    で確認する）。
    """
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.cruise_hold_speeds_kmh = []
    cfg.learning.creep_launch_count = 0
    cfg.learning.creep_brake_hold_offsets_pct = []
    cfg.learning.low_open_stair_offsets_pct = []
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    base = pdmod.build_patterns(_without_a3a4(cfg), profile)
    assert len(base) == 34 and len(patterns) == 41
    assert patterns[:29] == base[:29]  # 1〜29 は A2・A5 と同じ番号

    hard = patterns[29:33]
    assert all(isinstance(p, SpeedTargetPattern) and p.kind is PatternKind.BRAKE_HOLD
               for p in hard)
    assert [p.brake_opening for p in hard] == pytest.approx(
        [PEDAL.brake_deadband_pct + o for o in (7.0, 17.0, 27.0, 37.0)]
    )
    assert [p.accel_opening for p in hard] == pytest.approx([PEDAL.accel_deadband_pct + 8.0] * 4)
    assert {getattr(p, "accel_target_kmh", None) for p in hard} == {20.0}
    assert patterns[33:38] == base[29:]  # COAST_DOWN・CRUISE_TRIM はそのまま

    stairs = patterns[38:]
    assert all(isinstance(p, TrimStairPattern) and p.kind is PatternKind.CRUISE_TRIM
               for p in stairs)
    assert [getattr(p, "accel_target_kmh", None) for p in stairs] == [120.0, 90.0, 50.0]
    steps = tuple(PEDAL.accel_deadband_pct + o for o in (8.0, 5.0, 2.0))
    for p in stairs:
        assert isinstance(p, TrimStairPattern)
        assert p.trim_steps_pct == pytest.approx(steps)
        assert (p.trim_opening, p.step_hold_s, p.hold_duration_s) == (steps[0], 8.0, 24.0)
        assert p.accel_opening == 70.0
    label = " → ".join(f"{v:.1f}" for v in steps)
    assert f"トリム階段 {label}% 各 8s（120 km/h まで加速）" in pdmod._describe(stairs[0])


def test_build_patterns_clamps_a3a4_openings_to_max() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.brake_hold_hard_offsets_pct = [7.0, 90.0]
    cfg.learning.trim_stair_offsets_pct = [95.0, 2.0]
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    hard = [p for p in patterns if isinstance(p, SpeedTargetPattern)
            and getattr(p, "accel_target_kmh", None) == cfg.learning.brake_hold_hard_start_kmh]
    assert hard[-1].brake_opening == profile.max_brake_opening
    stair = next(p for p in patterns if isinstance(p, TrimStairPattern))
    assert stair.trim_steps_pct[0] == profile.max_accel_opening


def test_build_patterns_adds_cruise_stair_at_end() -> None:
    """2026-09-14 定速階段（段2）: トリム階段の後に 1 本足す。

    クリープ発進・クリープ域ブレーキ保持（段1）・低開度階段（段3-1）は定速階段よりさらに後ろに
    足すので、本テストの対象外として無効化する（別途
    test_build_patterns_adds_creep_launches_and_holds_at_end /
    test_build_patterns_adds_low_open_stair_* で確認）。
    """
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.creep_launch_count = 0
    cfg.learning.creep_brake_hold_offsets_pct = []
    cfg.learning.low_open_stair_offsets_pct = []
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    without_cruise = copy.deepcopy(cfg)
    without_cruise.learning.cruise_hold_speeds_kmh = []
    base = pdmod.build_patterns(without_cruise, profile)
    assert len(patterns) == len(base) + 1
    assert patterns[:-1] == base

    stair = patterns[-1]
    assert isinstance(stair, CruiseStairPattern) and stair.kind is PatternKind.CRUISE_TRIM
    lr = cfg.learning
    assert stair.hold_speeds_kmh == pytest.approx(tuple(lr.cruise_hold_speeds_kmh))
    assert (stair.settle_tol_kmh, stair.settle_s, stair.hold_s, stair.step_timeout_s) == (
        lr.cruise_hold_settle_tol_kmh, lr.cruise_hold_settle_s, lr.cruise_hold_hold_s,
        lr.cruise_hold_step_timeout_s,
    )
    assert (stair.kp, stair.ki, stair.max_rate_pct_per_s, stair.initial_offset_pct) == (
        lr.cruise_hold_kp, lr.cruise_hold_ki, lr.cruise_hold_max_rate_pct_per_s,
        lr.cruise_hold_initial_offset_pct,
    )
    assert stair.accel_opening == pytest.approx(min(70.0, profile.max_accel_opening))
    label = " → ".join(f"{v:g}" for v in stair.hold_speeds_kmh)
    assert f"定速階段 {label} km/h" in pdmod._describe(stair)


def test_build_patterns_without_cruise_hold_speeds_has_no_cruise_stair() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.cruise_hold_speeds_kmh = []
    patterns = pdmod.build_patterns(cfg, PEDAL.apply_to_profile(build_vehicle_profile(cfg)))
    assert not any(isinstance(p, CruiseStairPattern) for p in patterns)


# ── 低開度階段（段3-1。2026-09-19。ProblemReport_20260919 候補(c)） ────────────────────


def test_build_patterns_adds_low_open_stair_before_creep_launches() -> None:
    """定速階段の後・クリープ発進の前に 1 本足す。往復（descend）で offsets 10 個 → 19 段。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    without_low_open = copy.deepcopy(cfg)
    without_low_open.learning.low_open_stair_offsets_pct = []
    base = pdmod.build_patterns(without_low_open, profile)
    assert len(patterns) == len(base) + 1

    lr = cfg.learning
    n_creep = lr.creep_launch_count + len(lr.creep_brake_hold_offsets_pct)
    idx = len(base) - n_creep  # 定速階段の直後・クリープ発進の前の挿入位置
    assert patterns[:idx] == base[:idx]
    assert patterns[idx + 1:] == base[idx:]

    stair = patterns[idx]
    assert isinstance(stair, LowOpenStairPattern) and stair.kind is PatternKind.CRUISE_TRIM
    ff = profile.feedforward_params
    steps_up = tuple(round(ff.accel_deadband_pct + o, 2) for o in lr.low_open_stair_offsets_pct)
    expected_steps = tuple(
        min(v, profile.max_accel_opening) for v in (steps_up + steps_up[-2::-1])
    )
    assert len(lr.low_open_stair_offsets_pct) == 10 and len(expected_steps) == 19
    assert stair.trim_steps_pct == pytest.approx(expected_steps)
    assert stair.min_speed_kmh == pytest.approx(lr.low_open_stair_min_speed_kmh)
    assert stair.step_hold_s == pytest.approx(lr.low_open_stair_step_s)
    assert stair.accel_target_kmh == pytest.approx(lr.low_open_stair_start_kmh)
    assert stair.hold_duration_s == pytest.approx(lr.low_open_stair_step_s * len(expected_steps))
    label = " → ".join(f"{v:.2f}" for v in stair.trim_steps_pct)
    assert f"低開度階段 {label}%" in pdmod._describe(stair)


def test_build_patterns_low_open_stair_starts_at_first_step_opening() -> None:
    """DRIVE_ACCEL の加速用開度は 1 段目と同じ開度にする（段差を作らない）。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    stair = next(p for p in patterns if isinstance(p, LowOpenStairPattern))
    assert stair.accel_opening == pytest.approx(stair.trim_steps_pct[0])
    assert stair.trim_opening == pytest.approx(stair.trim_steps_pct[0])


def test_build_patterns_without_low_open_offsets_has_no_low_open_stair() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.low_open_stair_offsets_pct = []
    patterns = pdmod.build_patterns(cfg, PEDAL.apply_to_profile(build_vehicle_profile(cfg)))
    assert not any(isinstance(p, LowOpenStairPattern) for p in patterns)


def test_build_patterns_low_open_stair_descend_false_is_one_way() -> None:
    """descend=False なら折り返さず、offsets と同じ本数のまま（昇順）。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.low_open_stair_descend = False
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    stair = next(p for p in patterns if isinstance(p, LowOpenStairPattern))
    assert len(stair.trim_steps_pct) == len(cfg.learning.low_open_stair_offsets_pct)
    assert list(stair.trim_steps_pct) == sorted(stair.trim_steps_pct)


def test_build_patterns_clamps_low_open_stair_to_max() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.low_open_stair_offsets_pct = [0.5, 90.0]  # 90 は max_accel_opening を超える
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    stair = next(p for p in patterns if isinstance(p, LowOpenStairPattern))
    assert max(stair.trim_steps_pct) == profile.max_accel_opening


def test_build_patterns_adds_creep_launches_and_holds_at_end() -> None:
    """2026-09-17 クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2）:

    定速階段の後、パターン列の末尾に足す（手順3のモード走行と同じ暖機状態でクリープを測るため）。
    """
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = PEDAL.apply_to_profile(build_vehicle_profile(cfg))
    patterns = pdmod.build_patterns(cfg, profile)
    without_creep = copy.deepcopy(cfg)
    without_creep.learning.creep_launch_count = 0
    without_creep.learning.creep_brake_hold_offsets_pct = []
    base = pdmod.build_patterns(without_creep, profile)

    lr = cfg.learning
    n_new = lr.creep_launch_count + len(lr.creep_brake_hold_offsets_pct)
    assert len(patterns) == len(base) + n_new
    assert patterns[:-n_new] == base

    tail = patterns[-n_new:]
    launches = tail[:lr.creep_launch_count]
    holds = tail[lr.creep_launch_count:]

    assert len(launches) == lr.creep_launch_count
    for p in launches:
        assert isinstance(p, CreepLaunchPattern) and p.kind is PatternKind.CREEP_SETTLE
        assert not p.hold_after
        assert (p.accel_opening, p.brake_opening) == (0.0, 0.0)
        assert p.target_kmh == pytest.approx(lr.creep_launch_target_kmh)
        assert p.timeout_s == pytest.approx(lr.creep_launch_timeout_s)
        assert "クリープ発進" in pdmod._describe(p) and "停車復帰" in pdmod._describe(p)

    assert len(holds) == len(lr.creep_brake_hold_offsets_pct)
    ff = profile.feedforward_params
    expected_openings = [
        min(round(ff.brake_deadband_pct + offset, 2), profile.max_brake_opening)
        for offset in lr.creep_brake_hold_offsets_pct
    ]
    for p, expected in zip(holds, expected_openings, strict=True):
        assert isinstance(p, CreepLaunchPattern) and p.kind is PatternKind.CREEP_SETTLE
        assert p.hold_after
        assert p.accel_opening == 0.0
        assert p.brake_opening == pytest.approx(expected)
        assert p.target_kmh == pytest.approx(lr.creep_launch_target_kmh)
        assert "ブレーキ保持" in pdmod._describe(p)


def test_build_patterns_without_creep_settings_has_no_creep_launch() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.creep_launch_count = 0
    cfg.learning.creep_brake_hold_offsets_pct = []
    patterns = pdmod.build_patterns(cfg, PEDAL.apply_to_profile(build_vehicle_profile(cfg)))
    assert not any(isinstance(p, CreepLaunchPattern) for p in patterns)


def test_build_patterns_without_additions_has_no_speed_target() -> None:
    cfg = _without_additions(cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH))
    patterns = pdmod.build_patterns(cfg, PEDAL.apply_to_profile(build_vehicle_profile(cfg)))
    assert sum(p.kind is PatternKind.ACCEL_SWEEP for p in patterns) == 4
    assert sum(p.kind is PatternKind.BRAKE_HOLD for p in patterns) == len(
        cfg.learning.brake_hold_offsets_pct
    )
    assert not any(isinstance(p, SpeedTargetPattern | TrimStairPattern) for p in patterns)


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
    assert kinds >= {"CREEP", "CREEP_SETTLE", "ACCEL_SWEEP"}
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


def test_build_ff_model_real_saves_model_but_not_measured_values(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    saved = cfgmod.load_config(cfg.source_path)
    assert saved.feedforward.model_path == result.model_path
    assert saved.feedforward.is_model_trained
    with Path(result.model_path).open("rb") as f:
        assert pickle.load(f)["model_type"] == MODEL_TYPE
    assert set(result.metrics) == {"accel", "brake"}
    assert saved.feedforward.engine_brake_decel_kmhs == pytest.approx(
        result.params.engine_brake_decel_kmhs, rel=1e-5
    )
    # 不感帯・停車保持開度は 2-0 の実測が正。推定値では上書きしない
    for key in pdmod.MEASURED_KEYS:
        assert getattr(saved.feedforward, key) == getattr(cfg.feedforward, key)
        assert not any(line.startswith(f"feedforward.{key}:") for line in result.changed)
    # ペダルゲインは研究側の推定（不感帯 + 0.5% 以上）で両側とも同定し、グリッドと同じ点数で書き戻す
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
    生の `after`（不感帯が表示専用の粗い推定値。実測 8.53/12.00% に対し推定 3.0/3.0% 相当）
    ではなく、**before の実測不感帯・停車保持・creep_speed_kmh を保ったまま惰行カーブだけ
    差し替えたもの**であること。これを取り違えると「不感帯以下＝ペダルオフ」判定とゲインの
    分母（開度−不感帯）が壊れ、惰行サンプルが 995→361 点まで減る不具合が実機ログの relearn
    dry-run で確認された回帰。
    """
    cfg = _tmp_cfg(tmp_path)
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

    # 惰行カーブ同定そのもののサンプル抽出条件は before（2-0 実測）を渡す
    assert captured["coast_reference"] is before

    # クリープ・ゲインの基準は before の実測値そのもの（estimate_dynamics_params の粗い
    # 推定値ではない）を保っている
    for ref in (captured["creep_params"], captured["gain_before"]):
        assert ref.accel_deadband_pct == pytest.approx(PEDAL.accel_deadband_pct)
        assert ref.brake_deadband_pct == pytest.approx(PEDAL.brake_deadband_pct)
        assert ref.accel_deadband_pct == pytest.approx(before.accel_deadband_pct)
        assert ref.brake_deadband_pct == pytest.approx(before.brake_deadband_pct)
        assert ref.creep_speed_kmh == pytest.approx(before.creep_speed_kmh)
        assert ref.stop_brake_opening_pct == pytest.approx(before.stop_brake_opening_pct)
        # 惰行カーブだけは今回同定した結果と一致する
        assert ref.coast_decel_speeds_kmh == coast_result.coast_decel_speeds_kmh
        assert ref.coast_decel_kmhs == coast_result.coast_decel_kmhs

    assert captured["gain_after"] is coast_result


def test_build_ff_model_stop_brake_floor_uses_reference_basis_and_saves(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段4改訂（ProblemReport_20260916 クリープ域ブレーキの下限）: 下限も段2.5 と同じ形で
    reference（before 基準）で同定し、FF_PARAM_KEYS 経由で保存される。

    `test_build_ff_model_order_and_reference_basis_for_coast_creep_gain` と同じ形で、基準に
    渡された params の不感帯・停車保持開度が before（2-0 実測）の値であることをアサートする。
    """
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    captured: dict[str, Any] = {}
    original_stop_brake = pdmod._estimate_research_stop_brake_floor

    def spy_stop_brake(cfg_, logs, params, before):  # type: ignore[no-untyped-def]
        captured["params"] = params
        result = original_stop_brake(cfg_, logs, params, before)
        captured["result"] = result
        return result

    monkeypatch.setattr(pdmod, "_estimate_research_stop_brake_floor", spy_stop_brake)

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    # サンプル抽出条件の基準は before（2-0 実測）そのもの。estimate_dynamics_params の粗い
    # 推定値ではない
    params = captured["params"]
    assert params.accel_deadband_pct == pytest.approx(PEDAL.accel_deadband_pct)
    assert params.brake_deadband_pct == pytest.approx(PEDAL.brake_deadband_pct)
    assert params.stop_brake_opening_pct == pytest.approx(PEDAL.stop_brake_opening_pct)

    # 同定結果がそのまま research_params に乗り、FF_PARAM_KEYS 経由で YAML に保存される
    floor_result = captured["result"]
    assert (
        result.research_params.stop_brake_floor_offset_pct
        == floor_result.stop_brake_floor_offset_pct
    )

    saved = cfgmod.load_config(cfg.source_path)
    assert saved.feedforward.stop_brake_floor_offset_pct == pytest.approx(
        floor_result.stop_brake_floor_offset_pct
    )
    # brake_trim_max_kmh・brake_trim_ref_kmh は人が決める値なので自動保存の対象外
    # （期待値は走行前の設定値。_tmp_cfg は config_testVehicle.yaml のコピーなので
    # dataclass 既定値 0.0 とは限らない）
    assert saved.feedforward.brake_trim_max_kmh == pytest.approx(
        cfg.feedforward.brake_trim_max_kmh
    )
    assert saved.feedforward.brake_trim_ref_kmh == pytest.approx(cfg.feedforward.brake_trim_ref_kmh)


def test_build_ff_model_stub_does_not_touch_yaml(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    before = cfg.source_path.read_text(encoding="utf-8")

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_STUB)

    assert cfg.source_path.read_text(encoding="utf-8") == before
    assert "_stub_" in Path(result.model_path).name
    assert result.changed == []


def test_build_ff_model_raises_on_too_few_samples(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg)[:10], csv_path)
    with pytest.raises(LearningDataError):
        pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL)


# ── CLI 経由 ─────────────────────────────────────────────────────────


async def _fake_search(hw: object, cfg: object, **kwargs: object) -> PedalSearchResult:
    return PEDAL


async def _fake_pre_check(hw: object, cfg: object, **kwargs: object) -> int:
    """走行前チェックは test_research_pre_drive_check.py で確認する。"""
    return 0


def test_step2_requires_step1(capsys: pytest.CaptureFixture[str]) -> None:
    assert mainmod.main(["--only", "2"]) == 4
    assert "手順 1 を先に実行" in capsys.readouterr().out


def test_drive_error_exit_code_is_5(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def failing_search(hw: object, cfg: object, **kwargs: object) -> None:
        raise pdmod.DriveError("テスト用の探索失敗")

    cfg = _tmp_cfg(tmp_path)  # 走行ログを tmp に書く
    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "run_pedal_search", failing_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 5
    out = capsys.readouterr().out
    assert "走行エラー: テスト用の探索失敗" in out
    assert "終了処理: 完了" in out  # 失敗しても原点復帰・サーボOFF まで行く


def test_model_error_exit_code_is_6(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def fake_drive(hw: object, cfg: object, **kwargs: object) -> pdmod.PatternDriveResult:
        return pdmod.PatternDriveResult(csv_path=tmp_path / "x.csv", samples=[], duration_s=0.0)

    def failing_build(cfg: object, csv_path: object, **kwargs: object) -> None:
        raise LearningDataError("学習サンプルが不足しています (3 点)")

    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "run_pedal_search", _fake_search)
    monkeypatch.setattr(mainmod, "run_pattern_drive", fake_drive)
    monkeypatch.setattr(mainmod, "build_ff_model", failing_build)
    cfg = _tmp_cfg(tmp_path)  # 走行ログを tmp に書く
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 6
    assert "モデル作成エラー" in capsys.readouterr().out
