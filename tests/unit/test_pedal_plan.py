"""ペダルプラン（pedal_plan）ドメインのユニットテスト。

フェーズ分類（クリープ発進=COAST・緩減速=DRIVE・急減速=BRAKE・停止=STOP_HOLD）、
micro-phase マージ、フェーズ整合クランプ、effort_at/phase_at 補間・端点、モデル未ロード時の
0 effort を検証する。
"""

from datetime import UTC, datetime

import numpy as np
import pytest

from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pedal_plan import (
    MIN_PHASE_S,
    PedalPlan,
    PedalPlanner,
    PlanPhase,
    classify_phases,
    coast_accel,
    merge_micro_phases,
    required_accel,
)
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import FeedforwardParams

PARAMS = FeedforwardParams(
    creep_speed_kmh=7.0,
    creep_rate_kmhs=0.5,
    engine_brake_decel_kmhs=1.6,
    stop_brake_opening_pct=20.0,
    brake_deadband_pct=1.5,
)


def _mode(points: list[tuple[float, float]]) -> DrivingMode:
    ref = [SpeedPoint(time_s=t, speed_kmh=v) for t, v in points]
    return DrivingMode(
        id="test",
        name="test",
        description="",
        reference_speed=ref,
        total_duration=points[-1][0],
        max_speed=max(v for _, v in points),
        created_at=datetime.now(tz=UTC),
    )


class TestCoastAccel:
    def test_creep_below_creep_speed(self) -> None:
        assert coast_accel(3.0, PARAMS) == pytest.approx(0.5)

    def test_engine_brake_above_creep_speed(self) -> None:
        assert coast_accel(60.0, PARAMS) == pytest.approx(-1.6)


class TestClassifyPhases:
    def test_stop_hold_below_stop_speed(self) -> None:
        speeds = np.array([0.0, 0.2])
        accels = np.array([0.0, 0.0])
        phases = classify_phases(speeds, accels, PARAMS)
        assert phases == [PlanPhase.STOP_HOLD, PlanPhase.STOP_HOLD]

    def test_drive_when_accel_exceeds_coast(self) -> None:
        # 60km/h で +1.0 km/h/s 要求 → a_coast=-1.6 を大きく超え DRIVE
        speeds = np.array([60.0])
        accels = np.array([1.0])
        assert classify_phases(speeds, accels, PARAMS) == [PlanPhase.DRIVE]

    def test_coast_when_matches_engine_brake(self) -> None:
        # 60km/h で -1.6 km/h/s（エンジンブレーキちょうど）→ COAST
        speeds = np.array([60.0])
        accels = np.array([-1.6])
        assert classify_phases(speeds, accels, PARAMS) == [PlanPhase.COAST]

    def test_gentle_decel_is_drive(self) -> None:
        # 60km/h で -0.5 km/h/s（エンジンブレーキ未満の緩減速）→ アクセルで調整 = DRIVE
        speeds = np.array([60.0])
        accels = np.array([-0.5])
        assert classify_phases(speeds, accels, PARAMS) == [PlanPhase.DRIVE]

    def test_hard_decel_is_brake(self) -> None:
        # 60km/h で -4.0 km/h/s（エンジンブレーキ超）→ BRAKE
        speeds = np.array([60.0])
        accels = np.array([-4.0])
        assert classify_phases(speeds, accels, PARAMS) == [PlanPhase.BRAKE]

    def test_creep_launch_is_coast(self) -> None:
        # 3km/h でクリープ加速率ちょうど（+0.5）→ クリープで足りる = COAST
        speeds = np.array([3.0])
        accels = np.array([0.5])
        assert classify_phases(speeds, accels, PARAMS) == [PlanPhase.COAST]

    def test_below_creep_slower_than_creep_needs_brake(self) -> None:
        # 3km/h で +0.2（クリープ率 0.5 未満）→ クリープを抑えるため BRAKE
        speeds = np.array([3.0])
        accels = np.array([0.2])
        assert classify_phases(speeds, accels, PARAMS) == [PlanPhase.BRAKE]


class TestMergeMicroPhases:
    def test_short_phase_absorbed_into_longer_neighbor(self) -> None:
        dt = 0.1
        # DRIVE 30サンプル(3s) → BRAKE 5サンプル(0.5s<2s) → DRIVE 30サンプル(3s)
        phases = (
            [PlanPhase.DRIVE] * 30 + [PlanPhase.BRAKE] * 5 + [PlanPhase.DRIVE] * 30
        )
        merged = merge_micro_phases(phases, dt, min_phase_s=MIN_PHASE_S)
        assert all(p == PlanPhase.DRIVE for p in merged)

    def test_stop_hold_never_merged(self) -> None:
        dt = 0.1
        # STOP_HOLD 5サンプル(0.5s)は保持され続ける
        phases = [PlanPhase.DRIVE] * 30 + [PlanPhase.STOP_HOLD] * 5 + [PlanPhase.DRIVE] * 30
        merged = merge_micro_phases(phases, dt, min_phase_s=MIN_PHASE_S)
        assert PlanPhase.STOP_HOLD in merged
        assert merged.count(PlanPhase.STOP_HOLD) == 5

    def test_long_phases_unchanged(self) -> None:
        dt = 0.1
        phases = [PlanPhase.DRIVE] * 30 + [PlanPhase.BRAKE] * 30
        merged = merge_micro_phases(phases, dt, min_phase_s=MIN_PHASE_S)
        assert merged == phases


class TestRequiredAccel:
    def test_constant_speed_zero_accel(self) -> None:
        speeds = np.full(50, 60.0)
        a = required_accel(speeds, 0.1)
        assert np.allclose(a, 0.0)

    def test_linear_ramp_constant_accel(self) -> None:
        # 0.1s刻みで +0.2km/h/サンプル = +2.0 km/h/s
        speeds = np.arange(50, dtype=float) * 0.2
        a = required_accel(speeds, 0.1)
        # 端は平滑窓で歪むため中央を確認
        assert a[25] == pytest.approx(2.0, abs=0.05)


class TestPedalPlanContainer:
    def test_effort_at_interpolation_and_clamp(self) -> None:
        plan = PedalPlan(dt_s=1.0, efforts=[0.0, 2.0, 4.0], phases=[PlanPhase.DRIVE] * 3)
        assert plan.effort_at(0.5) == pytest.approx(1.0)
        assert plan.effort_at(-5.0) == pytest.approx(0.0)  # 端点クランプ
        assert plan.effort_at(100.0) == pytest.approx(4.0)

    def test_phase_at_nearest_grid(self) -> None:
        plan = PedalPlan(
            dt_s=1.0,
            efforts=[0.0, 0.0, 0.0],
            phases=[PlanPhase.DRIVE, PlanPhase.COAST, PlanPhase.BRAKE],
        )
        assert plan.phase_at(0.0) == PlanPhase.DRIVE
        assert plan.phase_at(1.4) == PlanPhase.COAST
        assert plan.phase_at(100.0) == PlanPhase.BRAKE  # 末尾クランプ

    def test_empty_plan(self) -> None:
        plan = PedalPlan()
        assert plan.effort_at(1.0) == 0.0
        assert plan.phase_at(1.0) == PlanPhase.COAST


class TestPedalPlannerBuild:
    def test_no_model_yields_zero_effort_with_phases(self) -> None:
        # 0→60(加速)→60保持→0(減速)→停止
        mode = _mode([(0.0, 0.0), (20.0, 60.0), (30.0, 60.0), (50.0, 0.0), (56.0, 0.0)])
        ff = FeedforwardController()  # モデル未ロード
        ff.set_params(PARAMS)
        plan = PedalPlanner.build(mode, ff, PARAMS)
        assert not ff.has_model
        # モデル未ロードなら DRIVE/COAST/BRAKE の名目 effort は 0（STOP_HOLD のみ保持ブレーキ）
        for e, ph in zip(plan.efforts, plan.phases):
            if ph != PlanPhase.STOP_HOLD:
                assert e == 0.0
        # 加速区間に DRIVE、末尾停止に STOP_HOLD が含まれる
        assert PlanPhase.DRIVE in plan.phases
        assert PlanPhase.STOP_HOLD in plan.phases

    def test_launch_is_coast_then_drive(self) -> None:
        # 緩やかな発進: 0→10km/h を10秒(=1km/h/s)。低速はクリープ→加速へ
        mode = _mode([(0.0, 0.0), (10.0, 10.0), (20.0, 10.0)])
        ff = FeedforwardController()
        ff.set_params(PARAMS)
        plan = PedalPlanner.build(mode, ff, PARAMS)
        # 発進直後（v<creep_speed かつ加速がクリープ内）に STOP_HOLD/COAST、その後 DRIVE
        assert plan.phases[0] in (PlanPhase.STOP_HOLD, PlanPhase.COAST)

    def test_stop_hold_effort_is_brake(self) -> None:
        mode = _mode([(0.0, 0.0), (10.0, 40.0), (20.0, 0.0), (26.0, 0.0)])
        ff = FeedforwardController()
        ff.set_params(PARAMS)
        plan = PedalPlanner.build(mode, ff, PARAMS)
        # STOP_HOLD の effort は停車保持ブレーキ（負）
        for e, ph in zip(plan.efforts, plan.phases):
            if ph == PlanPhase.STOP_HOLD:
                assert e == pytest.approx(-PARAMS.stop_brake_opening_pct)

    def test_empty_reference(self) -> None:
        mode = DrivingMode(
            id="e",
            name="e",
            description="",
            reference_speed=[],
            total_duration=0.0,
            max_speed=0.0,
            created_at=datetime.now(tz=UTC),
        )
        ff = FeedforwardController()
        plan = PedalPlanner.build(mode, ff, PARAMS)
        assert plan.efforts == []
        assert plan.phases == []
