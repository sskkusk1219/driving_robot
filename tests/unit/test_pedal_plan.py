"""ペダルプラン（pedal_plan）ドメインのユニットテスト。

フェーズ分類（クリープ発進=COAST・緩減速=DRIVE・急減速=BRAKE・停止=STOP_HOLD）、
micro-phase マージ、フェーズ整合クランプ、effort_at/phase_at 補間・端点、モデル未ロード時の
0 effort を検証する。
"""

from datetime import UTC, datetime
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from src.domain.control import pedal_plan
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pedal_plan import (
    MIN_PHASE_S,
    PedalPlan,
    PedalPlanner,
    PlanPhase,
    analytic_efforts,
    blend_model_and_analytic,
    clamp_effort_by_phase,
    classify_phases,
    coast_accel,
    fold_times,
    merge_micro_phases,
    required_accel,
    snap_efforts_to_deadband,
    zero_phase_lowpass,
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


# 惰行減速カーブ同定済みのパラメータ（sample_004 実機相当: 低速 3.0 / 高速 1.5 km/h/s）
PARAMS_CURVE = FeedforwardParams(
    creep_speed_kmh=7.0,
    creep_rate_kmhs=0.5,
    engine_brake_decel_kmhs=1.6,  # 旧定数（カーブがあれば使われない）
    coast_decel_speeds_kmh=(20.0, 60.0, 100.0, 130.0),
    coast_decel_kmhs=(3.0, 2.7, 1.5, 1.5),
    stop_brake_opening_pct=20.0,
    brake_deadband_pct=1.5,
)


class TestCoastAccel:
    def test_creep_below_creep_speed(self) -> None:
        assert coast_accel(3.0, PARAMS) == pytest.approx(0.5)

    def test_engine_brake_above_creep_speed(self) -> None:
        assert coast_accel(60.0, PARAMS) == pytest.approx(-1.6)

    def test_curve_interpolation_when_identified(self) -> None:
        """惰行減速カーブ同定済みなら速度依存の補間値を使う（端点はクランプ）。"""
        assert coast_accel(60.0, PARAMS_CURVE) == pytest.approx(-2.7)
        assert coast_accel(80.0, PARAMS_CURVE) == pytest.approx(-2.1)  # 60-100 の中点補間
        assert coast_accel(140.0, PARAMS_CURVE) == pytest.approx(-1.5)  # 上端クランプ
        assert coast_accel(10.0, PARAMS_CURVE) == pytest.approx(-3.0)  # 下端クランプ

    def test_curve_creep_still_wins_below_creep_speed(self) -> None:
        assert coast_accel(3.0, PARAMS_CURVE) == pytest.approx(0.5)


class TestClassifyPhases:
    def test_stop_hold_below_stop_speed(self) -> None:
        speeds = np.array([0.0, 0.01])
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

    def test_strong_engine_brake_vehicle_gentle_decel_is_drive(self) -> None:
        """T8 実機再現: 惰行減速が強い車（45km/h で -3.0）では -2.0km/h/s の減速要求は
        惰行より弱い＝アクセルで支える必要 → DRIVE。

        単一定数（engine_brake=1.6）では同じ要求が BRAKE と真逆に誤分類され、
        clamp_by_phase がプランの正 effort を封じていた（sample_004 p95=4.05 の主因）。
        """
        speeds = np.array([45.0])
        accels = np.array([-2.0])
        assert classify_phases(speeds, accels, PARAMS) == [PlanPhase.BRAKE]  # 旧: 誤分類
        assert classify_phases(speeds, accels, PARAMS_CURVE) == [PlanPhase.DRIVE]  # 新: 正分類


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


class TestSnapEffortsToDeadband:
    """PedalArbiter._apply_deadband と同一規則の量子化（プラン↔実効値の整合、F2 対策）。"""

    def test_positive_below_half_deadband_snaps_to_zero(self) -> None:
        assert snap_efforts_to_deadband([0.4], PARAMS) == [0.0]  # accel_deadband_pct=1.0

    def test_positive_above_half_deadband_snaps_to_deadband_floor(self) -> None:
        assert snap_efforts_to_deadband([0.6], PARAMS) == [pytest.approx(1.0)]

    def test_positive_above_deadband_is_unchanged(self) -> None:
        assert snap_efforts_to_deadband([5.0], PARAMS) == [pytest.approx(5.0)]

    def test_negative_below_half_deadband_snaps_to_zero(self) -> None:
        # brake_deadband_pct=1.5 → 半分 0.75
        assert snap_efforts_to_deadband([-0.5], PARAMS) == [0.0]

    def test_negative_above_half_deadband_snaps_to_deadband_floor(self) -> None:
        assert snap_efforts_to_deadband([-1.0], PARAMS) == [pytest.approx(-1.5)]

    def test_negative_above_deadband_is_unchanged(self) -> None:
        assert snap_efforts_to_deadband([-5.0], PARAMS) == [pytest.approx(-5.0)]

    def test_zero_stays_zero(self) -> None:
        assert snap_efforts_to_deadband([0.0], PARAMS) == [0.0]

    def test_sample_004_reproduction_gentle_brake_request_snaps_to_zero(self) -> None:
        """2026-07-14 実機 t=93-99s: -2.0km/h/s 要求でプラン -2.4% → brake_deadband=6.0
        では制動ゼロ（アービタの物理死帯）。プランもこれを 0（惰行）と認識すべき。"""
        params = FeedforwardParams(brake_deadband_pct=6.0, accel_deadband_pct=4.5)
        assert snap_efforts_to_deadband([-2.4], params) == [0.0]

    def test_does_not_flip_sign(self) -> None:
        efforts = [-5.0, -0.5, 0.0, 0.4, 5.0]
        snapped = snap_efforts_to_deadband(efforts, PARAMS)
        for e, s in zip(efforts, snapped):
            assert not (e > 0 and s < 0)
            assert not (e < 0 and s > 0)

    def test_preserves_phase_authority_after_clamp(self) -> None:
        """clamp_effort_by_phase 後に snap しても DRIVE≥0/BRAKE≤0/COAST=0 の権限を保つ。"""
        efforts = np.array([3.0, 0.3, -3.0, -0.5, 0.0])
        phases = [
            PlanPhase.DRIVE,
            PlanPhase.DRIVE,
            PlanPhase.BRAKE,
            PlanPhase.BRAKE,
            PlanPhase.COAST,
        ]
        clamped = clamp_effort_by_phase(efforts, phases, PARAMS)
        snapped = snap_efforts_to_deadband(clamped, PARAMS)
        for e, ph in zip(snapped, phases):
            if ph == PlanPhase.DRIVE:
                assert e >= 0.0
            elif ph == PlanPhase.BRAKE:
                assert e <= 0.0
            elif ph == PlanPhase.COAST:
                assert e == 0.0


# 惰行カーブ＋ペダルゲイン同定済み（実機 3eebcff3 の同定値相当）
PARAMS_GAIN = FeedforwardParams(
    creep_speed_kmh=5.0,
    creep_rate_kmhs=0.15,
    engine_brake_decel_kmhs=1.6,
    coast_decel_speeds_kmh=(5.0, 25.0, 45.0, 55.0, 65.0, 115.0),
    coast_decel_kmhs=(1.60, 2.59, 3.58, 3.56, 3.13, 3.10),
    pedal_gain_speeds_kmh=(25.0, 45.0, 55.0, 65.0, 115.0),
    accel_gain_kmhs_per_pct=(0.53, 0.34, 0.29, 0.26, 0.34),
    brake_gain_kmhs_per_pct=(0.25, 0.42, 0.54, 0.68, 0.98),
    accel_deadband_pct=0.5,
    brake_deadband_pct=3.5,
    stop_brake_opening_pct=16.0,
)


class TestAnalyticEfforts:
    """惰行カーブ基準の解析 effort（低開度域＝モデル外挿域の置き換え）。"""

    def test_gentle_decel_needs_positive_effort(self) -> None:
        """惰行より緩い減速要求にはアクセル側の正 effort が要る（WLTP 減速の 75%）。"""
        # 55km/h で -2.0km/h/s 要求。惰行は -3.56 なので Δa=+1.56 をアクセルで作る
        out = analytic_efforts(np.array([55.0]), np.array([-2.0]), PARAMS_GAIN)
        assert out[0] > 0.0
        # 1.56 / 0.29 + 不感帯 0.5 ≒ 5.9%
        assert out[0] == pytest.approx(1.56 / 0.29 + 0.5, rel=0.02)

    def test_coasting_requires_zero_effort(self) -> None:
        """Δa=0（惰行そのまま）なら effort=0（原点通過が構造的に保証される）。"""
        out = analytic_efforts(np.array([55.0]), np.array([-3.56]), PARAMS_GAIN)
        assert out[0] == pytest.approx(0.0, abs=1e-6)

    def test_hard_decel_needs_brake_effort(self) -> None:
        """惰行より強い減速要求はブレーキ側の負 effort。"""
        out = analytic_efforts(np.array([55.0]), np.array([-6.0]), PARAMS_GAIN)
        assert out[0] < 0.0
        assert out[0] == pytest.approx(-((6.0 - 3.56) / 0.54 + 3.5), rel=0.02)

    def test_unidentified_gain_returns_nan(self) -> None:
        out = analytic_efforts(np.array([55.0]), np.array([-2.0]), PARAMS)
        assert np.isnan(out[0])

    def test_stopped_returns_nan(self) -> None:
        """停車域は clamp_effort_by_phase の停車保持に委ねる。"""
        out = analytic_efforts(np.array([0.0]), np.array([0.0]), PARAMS_GAIN)
        assert np.isnan(out[0])


class TestBlendModelAndAnalytic:
    def test_low_effort_uses_analytic(self) -> None:
        out = blend_model_and_analytic(np.array([99.0]), np.array([3.0]))
        assert out[0] == pytest.approx(3.0)

    def test_high_effort_uses_model(self) -> None:
        out = blend_model_and_analytic(np.array([99.0]), np.array([20.0]))
        assert out[0] == pytest.approx(99.0)

    def test_midrange_is_linear_mix(self) -> None:
        # |analytic|=10 → LO=5, HI=15 の中点なので 50:50
        out = blend_model_and_analytic(np.array([20.0]), np.array([10.0]))
        assert out[0] == pytest.approx(15.0)

    def test_nan_analytic_falls_back_to_model(self) -> None:
        out = blend_model_and_analytic(np.array([7.0]), np.array([np.nan]))
        assert out[0] == pytest.approx(7.0)


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

    def test_build_snaps_sub_deadband_effort_to_zero(self) -> None:
        """build() は clamp 後に snap を適用し、不感帯未満の名目 effort を 0 にする。"""
        mode = _mode([(0.0, 20.0), (30.0, 20.0)])  # 定速巡航のみ（全区間 DRIVE/COAST 想定）
        ff = MagicMock(spec=FeedforwardController)
        ff.has_model = True
        ff.horizons = ()
        ff.past_horizons = ()
        ff.predict_effort = MagicMock(return_value=0.3)  # accel_deadband_pct=1.0 の半分未満
        plan = PedalPlanner.build(mode, ff, PARAMS)
        for e, ph in zip(plan.efforts, plan.phases):
            if ph == PlanPhase.DRIVE:
                assert e == 0.0

    def test_gain_curve_lifts_gentle_decel_plan(self) -> None:
        """ペダルゲイン同定済みなら、緩減速区間に最初から正の effort が入る。"""
        # 68→52km/h を 8 秒（-2.0km/h/s）。惰行は -3.1〜-3.6 なのでアクセルが要る
        mode = _mode([(0.0, 68.0), (8.0, 52.0), (16.0, 52.0)])
        ff = MagicMock(spec=FeedforwardController)
        ff.has_model = True
        ff.horizons = ()
        ff.past_horizons = ()
        ff.predict_effort = MagicMock(return_value=0.0)  # モデルは外挿できず 0 を返す想定
        plan = PedalPlanner.build(mode, ff, PARAMS_GAIN)
        drive = [e for e, ph in zip(plan.efforts, plan.phases) if ph == PlanPhase.DRIVE]
        assert drive, "緩減速区間は DRIVE 分類のはず"
        assert max(drive) > 3.0

    def test_unidentified_gain_keeps_model_output(self) -> None:
        """ペダルゲイン未同定なら従来どおりモデル出力がそのまま使われる（後方互換）。"""
        mode = _mode([(0.0, 60.0), (30.0, 60.0)])
        ff = MagicMock(spec=FeedforwardController)
        ff.has_model = True
        ff.horizons = ()
        ff.past_horizons = ()
        ff.predict_effort = MagicMock(return_value=8.0)
        plan = PedalPlanner.build(mode, ff, PARAMS)  # ゲイン未同定
        drive = [e for e, ph in zip(plan.efforts, plan.phases) if ph == PlanPhase.DRIVE]
        assert drive
        assert max(drive) == pytest.approx(8.0, rel=0.05)

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


class TestZeroPhaseLowpass:
    """ilc.py から移設したゼロ位相ローパス（プラン平滑・プラン更新で共用）。"""

    def test_symmetric_bump_keeps_peak_position(self) -> None:
        """ゼロ位相ローパスは対称な山のピーク位置を動かさない（位相遅れ 0）。"""
        n = 101
        x = np.exp(-((np.arange(n) - 50.0) ** 2) / (2 * 8.0**2))  # 中心 50 のガウス山
        y = zero_phase_lowpass(x, cutoff_hz=0.5, dt_s=0.1)
        assert int(np.argmax(y)) == 50

    def test_short_series_returned_asis(self) -> None:
        x = np.array([3.0])
        assert np.array_equal(zero_phase_lowpass(x, 0.3, 0.1), x)

    def test_zero_cutoff_is_passthrough(self) -> None:
        x = np.array([1.0, 5.0, 2.0])
        assert np.array_equal(zero_phase_lowpass(x, 0.0, 0.1), x)


class TestCornerPreservation:
    """コーナーでプランが必要 effort を保つこと（実機 9eee549b の回帰）。

    旧定数（PLAN_LOWPASS_HZ=0.25）はゼロ位相ローパス（forward-backward, RC=0.64s を
    両方向）の非因果性でコーナーを前後 ~1.3s ににじませ、必要な踏み替えを 2〜3 秒早く
    始めていた。実測（検証パターン __verify_pattern__、実機プロファイル 3937e81b）の
    解析 effort に対する保持率:
      減速コーナー t=79-83s  旧 76.6% → 新 89.1%
      軌跡頂点     t=143-147s 旧 77.7% → 新 89.0%
      全区間平均              旧 88.4% → 新 89.6%（ランプ主体なので変わらないのが正しい）

    なお PLAN_ACCEL_SMOOTH_S（センタリング移動平均）は 1.0 のまま据え置いた。当初は
    こちらもにじみの一因と見て 0.3 へ縮める案を検討したが、閉ループ模擬で 1.0 のほうが
    一貫して良かった（同定数の docstring 参照）。にじみはローパスがほぼ全てだった。
    """

    @staticmethod
    def _corner_mode() -> DrivingMode:
        """−5.0km/h/s で減速し、t=10s から −2.0km/h/s へ緩む折れ点を持つ軌跡。"""
        pts = [SpeedPoint(0.0, 100.0)]
        v = 100.0
        for i in range(1, 201):
            t = i * 0.1
            v += (-5.0 if t <= 10.0 else -2.0) * 0.1
            pts.append(SpeedPoint(t, max(0.0, v)))
        return DrivingMode(
            id="corner",
            name="corner",
            description="",
            reference_speed=pts,
            total_duration=20.0,
            max_speed=100.0,
            created_at=datetime(2026, 1, 1, tzinfo=UTC),
        )

    def _plan_vs_analytic(self, smooth_s: float, lowpass_hz: float) -> float:
        """折れ点直前 1 秒でプランが解析 effort の何割を保っているか。"""
        mode = self._corner_mode()
        ff = MagicMock(spec=FeedforwardController)
        ff.has_model = False
        with (
            patch.object(pedal_plan, "PLAN_ACCEL_SMOOTH_S", smooth_s),
            patch.object(pedal_plan, "PLAN_LOWPASS_HZ", lowpass_hz),
        ):
            plan = PedalPlanner.build(mode, ff, PARAMS_GAIN, dt_s=0.1)
            grid_t = np.arange(len(plan.efforts)) * 0.1
            grid_v = np.interp(
                grid_t,
                [p.time_s for p in mode.reference_speed],
                [p.speed_kmh for p in mode.reference_speed],
            )
            a_req = pedal_plan.required_accel(grid_v, 0.1, smooth_s)
            analytic = pedal_plan.analytic_efforts(grid_v, a_req, PARAMS_GAIN)
        efforts = np.array(plan.efforts, dtype=float)
        window = slice(90, 100)  # t=9.0-9.9s（折れ点 t=10.0 の直前）
        want = np.abs(analytic[window])
        got = np.abs(efforts[window])
        assert np.isfinite(want).all() and want.sum() > 0.0
        return float(got.sum() / want.sum())

    def test_current_constants_preserve_corner(self) -> None:
        """現行定数では折れ点直前で必要 effort の 8 割以上を保つ。"""
        assert self._plan_vs_analytic(
            pedal_plan.PLAN_ACCEL_SMOOTH_S, pedal_plan.PLAN_LOWPASS_HZ
        ) >= 0.8

    def test_old_constants_lost_the_corner(self) -> None:
        """旧定数（1.0s / 0.25Hz）は同じ地点で明確に削れていた（回帰の証拠）。"""
        old = self._plan_vs_analytic(1.0, 0.25)
        new = self._plan_vs_analytic(
            pedal_plan.PLAN_ACCEL_SMOOTH_S, pedal_plan.PLAN_LOWPASS_HZ
        )
        assert old < new
        assert old < 0.8


class TestFoldTimes:
    """折返し点（駆動⇄制動の入れ替わり）の検出。"""

    def test_detects_sign_reversal(self) -> None:
        efforts = [10.0] * 5 + [-5.0] * 5
        phases = [PlanPhase.DRIVE] * 5 + [PlanPhase.BRAKE] * 5
        assert fold_times(efforts, phases, 0.1) == pytest.approx([0.5])

    def test_zero_gap_between_same_sign_is_not_a_fold(self) -> None:
        """惰行帯（0）を挟んで同じ向きへ戻るのは折返しではない。"""
        efforts = [10.0, 10.0, 0.0, 0.0, 10.0, 10.0]
        assert fold_times(efforts, [PlanPhase.DRIVE] * 6, 0.1) == []

    def test_zero_gap_between_opposite_signs_is_a_fold(self) -> None:
        efforts = [10.0, 10.0, 0.0, 0.0, -5.0, -5.0]
        phases = [PlanPhase.DRIVE] * 2 + [PlanPhase.COAST] * 2 + [PlanPhase.BRAKE] * 2
        assert fold_times(efforts, phases, 0.1) == pytest.approx([0.4])

    def test_phase_change_without_sign_change_is_a_fold(self) -> None:
        """effort が 0 のままでも DRIVE⇄BRAKE のフェーズ境界は折返しとして拾う。"""
        efforts = [0.0] * 6
        phases = [PlanPhase.DRIVE] * 3 + [PlanPhase.BRAKE] * 3
        assert fold_times(efforts, phases, 0.1) == pytest.approx([0.3])

    def test_monotonic_plan_has_no_fold(self) -> None:
        assert fold_times([float(i) for i in range(10)], [PlanPhase.DRIVE] * 10, 0.1) == []
