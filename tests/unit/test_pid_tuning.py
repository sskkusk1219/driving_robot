"""PID 自動適合（pid_tuning）のユニットテスト。"""

import dataclasses
import math
from datetime import UTC, datetime, timedelta

import pytest

from src.domain.pid_tuning import (
    FOPDT,
    INVALID_COST,
    KP_MAX,
    PID_PREVIEW_MAX_S,
    CoordinateDescentTuner,
    TuningParams,
    build_tuning_trajectory,
    build_tuning_trajectory_from_mode,
    compute_pid_gains_simc,
    identify_fopdt,
    initial_preview_from_fopdt,
    tuning_cost,
)
from src.models.drive_log import DriveLog
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile

DT_S = 0.1


def make_profile(kp: float = 1.0, ki: float = 0.1) -> VehicleProfile:
    return VehicleProfile(
        id="p1",
        name="t",
        max_accel_opening=80.0,
        max_brake_opening=80.0,
        max_speed=120.0,
        max_decel_g=0.4,
        pid_gains=PIDGains(kp=kp, ki=ki, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None,
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )


def _step_speeds(u: float, k: float, tau: float, theta: float, v0: float, n: int) -> list[float]:
    """既知 FOPDT のステップ応答車速系列を生成する。v_steady = v0 + k·u。"""
    v_steady = v0 + k * u
    speeds: list[float] = []
    for i in range(n):
        t = i * DT_S
        if t < theta:
            v = v0
        else:
            v = v_steady - (v_steady - v0) * math.exp(-(t - theta) / tau)
        speeds.append(v)
    return speeds


def _build_logs(
    segments: list[tuple[float, float]],  # (accel_opening, brake_opening) per sample
    speeds: list[float],
    session_id: str = "s1",
) -> list[DriveLog]:
    t0 = datetime.now(tz=UTC)
    logs: list[DriveLog] = []
    for i, ((accel, brake), speed) in enumerate(zip(segments, speeds, strict=True)):
        logs.append(
            DriveLog(
                id=i,
                session_id=session_id,
                timestamp=t0 + timedelta(seconds=DT_S * i),
                ref_speed_kmh=None,
                actual_speed_kmh=speed,
                accel_opening=accel,
                brake_opening=brake,
                accel_pos=0,
                brake_pos=0,
                accel_current=0.0,
                brake_current=0.0,
            )
        )
    return logs


def _make_identifiable_logs(k: float, tau: float, theta: float) -> list[DriveLog]:
    """ブレーキオフギャップで区切った 2 つのアクセル保持区間を持つログを生成する。"""
    seg1 = _step_speeds(u=30.0, k=k, tau=tau, theta=theta, v0=0.0, n=80)
    gap = [seg1[-1]] * 10  # ブレーキオン（保持区間を分割）
    seg2 = _step_speeds(u=40.0, k=k, tau=tau, theta=theta, v0=10.0, n=80)

    speeds = seg1 + gap + seg2
    pedals: list[tuple[float, float]] = (
        [(30.0, 0.0)] * len(seg1) + [(0.0, 40.0)] * len(gap) + [(40.0, 0.0)] * len(seg2)
    )
    return _build_logs(pedals, speeds)


class TestIdentifyFopdt:
    def test_recovers_known_fopdt(self) -> None:
        k, tau, theta = 0.6, 2.0, 0.3
        logs = _make_identifiable_logs(k, tau, theta)
        fopdt = identify_fopdt(logs, make_profile())
        assert fopdt is not None
        assert fopdt.k == pytest.approx(k, rel=0.15)
        assert fopdt.tau == pytest.approx(tau, rel=0.3)
        assert fopdt.theta == pytest.approx(theta, abs=0.6)

    def test_brake_deadband_zero_still_finds_accel_segments(self) -> None:
        """D5 回帰テスト: brake_deadband_pct=0.0（合法値）でも strict < の falsy-zero で
        セグメントが0件になり None が返るバグの回帰。"""
        k, tau, theta = 0.6, 2.0, 0.3
        logs = _make_identifiable_logs(k, tau, theta)
        profile = dataclasses.replace(
            make_profile(),
            feedforward_params=FeedforwardParams(brake_deadband_pct=0.0),
        )
        fopdt = identify_fopdt(logs, profile)
        assert fopdt is not None
        assert fopdt.k == pytest.approx(k, rel=0.15)

    def test_insufficient_segments_returns_none(self) -> None:
        # アクセルをほぼ踏まない（保持区間なし）→ None
        speeds = [0.0] * 50
        pedals = [(0.0, 0.0)] * 50
        logs = _build_logs(pedals, speeds)
        assert identify_fopdt(logs, make_profile()) is None

    def test_no_speed_rise_returns_none(self) -> None:
        # アクセルは踏むが車速が上がらない（MIN_RISE 未満）→ 区間棄却 → None
        speeds = [5.0] * 80
        pedals = [(30.0, 0.0)] * 80
        logs = _build_logs(pedals, speeds)
        assert identify_fopdt(logs, make_profile()) is None


class TestComputePidGainsSimc:
    def test_basic_gains_positive_kd_zero(self) -> None:
        gains = compute_pid_gains_simc(FOPDT(k=0.5, tau=2.0, theta=0.3), make_profile())
        assert gains.kp > 0.0
        assert gains.ki > 0.0
        assert gains.kd == 0.0

    def test_larger_tau_c_factor_reduces_kp(self) -> None:
        fopdt = FOPDT(k=0.5, tau=4.0, theta=0.3)
        tight = compute_pid_gains_simc(fopdt, make_profile(), tau_c_factor=0.3)
        robust = compute_pid_gains_simc(fopdt, make_profile(), tau_c_factor=1.0)
        assert robust.kp < tight.kp

    def test_small_gain_clamps_kp(self) -> None:
        # 極小プラントゲイン → 巨大な Kc → KP_MAX にクランプ
        gains = compute_pid_gains_simc(FOPDT(k=0.001, tau=2.0, theta=0.3), make_profile())
        assert gains.kp == pytest.approx(KP_MAX)

    def test_nonphysical_gain_keeps_existing(self) -> None:
        prof = make_profile(kp=3.3, ki=0.7)
        gains = compute_pid_gains_simc(FOPDT(k=0.0, tau=2.0, theta=0.3), prof)
        assert gains == prof.pid_gains


class TestInitialPreviewFromFopdt:
    """Stage A: PID 先読み初期値は θ に依らず常に 0.0（FF 内蔵補償との二重補償回避）。"""

    def test_returns_zero_regardless_of_theta(self) -> None:
        assert initial_preview_from_fopdt(FOPDT(k=0.5, tau=2.0, theta=0.8)) == pytest.approx(0.0)
        assert initial_preview_from_fopdt(FOPDT(k=0.5, tau=2.0, theta=10.0)) == pytest.approx(0.0)
        assert initial_preview_from_fopdt(FOPDT(k=0.5, tau=2.0, theta=-0.5)) == pytest.approx(0.0)


class TestBuildTuningTrajectory:
    def test_within_max_speed_and_returns_to_zero(self) -> None:
        prof = make_profile()
        prof.max_speed = 100.0
        mode = build_tuning_trajectory(prof)
        speeds = [p.speed_kmh for p in mode.reference_speed]
        times = [p.time_s for p in mode.reference_speed]
        assert all(s <= 100.0 + 1e-9 for s in speeds)
        assert times == sorted(times)  # 時間軸は単調増加
        assert speeds[0] == 0.0
        assert speeds[-1] == 0.0
        assert max(speeds) > 0.0  # 加速を含む
        assert mode.total_duration == times[-1]
        assert mode.max_speed == 100.0

    def test_includes_cruise_hold_near_max_speed(self) -> None:
        """0.95×max_speed の巡航保持を含む（WLTP 巡航帯の微小開度データ供給・B-7-1）。"""
        prof = make_profile()
        prof.max_speed = 140.0
        mode = build_tuning_trajectory(prof)
        pts = mode.reference_speed
        peak = max(p.speed_kmh for p in pts)
        assert peak == pytest.approx(133.0, abs=1e-6)  # 0.95 × 140
        # 巡航速度で一定保持する区間が存在する（同一速度が連続する2点）
        cruise_holds = [
            b.time_s - a.time_s
            for a, b in zip(pts[:-1], pts[1:], strict=True)
            if a.speed_kmh == pytest.approx(peak) and b.speed_kmh == pytest.approx(peak)
        ]
        assert cruise_holds and max(cruise_holds) >= 10.0 - 1e-9

    def test_varied_ramp_rates(self) -> None:
        """加減速レートが単一でなく複数の傾きを含む（多様な過渡・B-7-1）。"""
        prof = make_profile()
        prof.max_speed = 140.0
        prof.max_decel_g = 0.4
        mode = build_tuning_trajectory(prof)
        pts = mode.reference_speed
        rates = {
            round(abs(b.speed_kmh - a.speed_kmh) / (b.time_s - a.time_s), 3)
            for a, b in zip(pts[:-1], pts[1:], strict=True)
            if abs(b.speed_kmh - a.speed_kmh) > 1e-9  # 加減速区間のみ
        }
        assert len(rates) >= 2  # 少なくとも高率/低率の2種

    @pytest.mark.parametrize(
        ("max_speed", "max_decel_g"),
        [(100.0, 0.3), (120.0, 0.5), (60.0, 0.2), (80.0, 0.1), (140.0, 0.4)],
    )
    def test_ramp_rate_within_max_g(self, max_speed: float, max_decel_g: float) -> None:
        """加速・減速いずれのレートも安全包絡（≤0.8×max_decel_g、床 MIN_RATE 考慮）以内。"""
        from src.domain.pid_tuning import G_TO_KMHS, MIN_RATE_KMHS

        prof = make_profile()
        prof.max_speed = max_speed
        prof.max_decel_g = max_decel_g
        mode = build_tuning_trajectory(prof)
        pts = mode.reference_speed

        # 全区間レートは 0.8×上限G か床 MIN_RATE の大きい方を超えない（区間別係数は全て ≤0.8）。
        envelope_kmhs = max(0.8 * max_decel_g * G_TO_KMHS, MIN_RATE_KMHS)
        for a, b in zip(pts[:-1], pts[1:], strict=True):
            dt = b.time_s - a.time_s
            assert dt > 0.0
            if abs(b.speed_kmh - a.speed_kmh) > 1e-9:  # 加減速区間
                rate = abs(a.speed_kmh - b.speed_kmh) / dt
                assert rate <= envelope_kmhs + 1e-9
            assert b.speed_kmh <= max_speed + 1e-9


class TestBuildVerificationTrajectory:
    """P-7: 学習サイクル VERIFY 用の検証専用パターン（登録全モードの包絡）。"""

    def _mode(self, points: list[tuple[float, float]], max_speed: float) -> DrivingMode:
        return DrivingMode(
            id="m",
            name="m",
            description="",
            reference_speed=[SpeedPoint(t, v) for t, v in points],
            total_duration=points[-1][0],
            max_speed=max_speed,
            created_at=datetime.now(tz=UTC),
        )

    def _modes(self) -> list[DrivingMode]:
        # 低速モード(80巡航) と 高速モード(130巡航) の 2 本
        return [
            self._mode([(0.0, 0.0), (20.0, 80.0), (60.0, 80.0), (80.0, 0.0)], 80.0),
            self._mode([(0.0, 0.0), (40.0, 130.0), (120.0, 130.0), (160.0, 0.0)], 130.0),
        ]

    def test_safety_envelope(self) -> None:
        from src.domain.pid_tuning import G_TO_KMHS, MIN_RATE_KMHS, build_verification_trajectory

        prof = make_profile()
        prof.max_speed = 140.0
        prof.max_decel_g = 0.4
        mode = build_verification_trajectory(self._modes(), prof)
        pts = mode.reference_speed
        times = [p.time_s for p in pts]
        cap = min(130.0, 140.0)
        assert times == sorted(times)
        assert pts[0].speed_kmh == 0.0
        assert pts[-1].speed_kmh == 0.0
        envelope = max(0.8 * prof.max_decel_g * G_TO_KMHS, MIN_RATE_KMHS)
        for a, b in zip(pts[:-1], pts[1:], strict=True):
            dt = b.time_s - a.time_s
            assert dt > 0.0
            assert b.speed_kmh <= cap + 1e-9
            if abs(b.speed_kmh - a.speed_kmh) > 1e-9:
                assert abs(b.speed_kmh - a.speed_kmh) / dt <= envelope + 1e-9

    def test_reaches_highest_cruise_and_stops(self) -> None:
        from src.domain.pid_tuning import build_verification_trajectory

        prof = make_profile()
        prof.max_speed = 140.0
        mode = build_verification_trajectory(self._modes(), prof)
        speeds = [p.speed_kmh for p in mode.reference_speed]
        # 最高速（cap=130）付近まで到達
        assert max(speeds) >= 130.0 - 2.0
        # 完全停止（0）を途中に含む（再発進の検証）
        assert any(s < 0.5 for s in speeds[1:-1])

    def test_cap_clip_for_low_max_speed(self) -> None:
        """max_speed=100 なら cap 超の巡航点（130）が除外され最高速巡航=100・全点≤100。"""
        from src.domain.pid_tuning import build_verification_trajectory

        prof = make_profile()
        prof.max_speed = 100.0
        mode = build_verification_trajectory(self._modes(), prof)
        speeds = [p.speed_kmh for p in mode.reference_speed]
        assert all(s <= 100.0 + 1e-9 for s in speeds)
        assert max(speeds) >= 100.0 - 2.0
        assert mode.max_speed == pytest.approx(100.0)

    def test_within_budget(self) -> None:
        from src.domain.pid_tuning import build_verification_trajectory

        prof = make_profile()
        prof.max_speed = 140.0
        mode = build_verification_trajectory(self._modes(), prof, budget_s=180.0)
        assert mode.total_duration <= 180.0 + 1e-6


class TestBuildTuningTrajectoryFromMode:
    """Stage B: 本番モードから代表区間トラジェクトリを切り出す。"""

    def _mode(self, points: list[tuple[float, float]], max_speed: float) -> DrivingMode:
        return DrivingMode(
            id="m1",
            name="WLTP_ExHi_like",
            description="",
            reference_speed=[SpeedPoint(time_s=t, speed_kmh=v) for t, v in points],
            total_duration=points[-1][0],
            max_speed=max_speed,
            created_at=datetime.now(tz=UTC),
        )

    def _long_mode(self) -> DrivingMode:
        # 300s、0→130→60→130→0 の起伏（最高速度が末尾寄り、加減速多め）
        pts = [
            (0.0, 0.0), (40.0, 130.0), (90.0, 130.0), (140.0, 60.0),
            (170.0, 60.0), (220.0, 131.0), (270.0, 131.0), (300.0, 0.0),
        ]
        return self._mode(pts, max_speed=131.3)

    def test_starts_and_ends_at_zero(self) -> None:
        traj = build_tuning_trajectory_from_mode(self._long_mode(), max_duration_s=120.0)
        speeds = [p.speed_kmh for p in traj.reference_speed]
        assert speeds[0] == pytest.approx(0.0)
        assert speeds[-1] == pytest.approx(0.0)

    def test_within_max_speed_and_monotonic_time(self) -> None:
        mode = self._long_mode()
        traj = build_tuning_trajectory_from_mode(mode, max_duration_s=120.0)
        times = [p.time_s for p in traj.reference_speed]
        speeds = [p.speed_kmh for p in traj.reference_speed]
        assert times == sorted(times)
        assert all(s <= mode.max_speed + 1e-9 for s in speeds)
        assert all(s >= -1e-9 for s in speeds)

    def test_duration_within_budget(self) -> None:
        traj = build_tuning_trajectory_from_mode(self._long_mode(), max_duration_s=120.0)
        assert traj.total_duration <= 120.0 + 1e-6

    def test_includes_high_speed_region(self) -> None:
        """最高速度窓を含むので、トラジェクトリの最高速度は元モードの高速域に達する。"""
        mode = self._long_mode()
        traj = build_tuning_trajectory_from_mode(mode, max_duration_s=120.0)
        peak = max(p.speed_kmh for p in traj.reference_speed)
        assert peak >= 0.8 * mode.max_speed

    def test_short_mode_passthrough(self) -> None:
        """既に max_duration_s 以下のモードは代表区間化せずそのまま使う。"""
        short = self._mode([(0.0, 0.0), (20.0, 80.0), (40.0, 0.0)], max_speed=80.0)
        traj = build_tuning_trajectory_from_mode(short, max_duration_s=120.0)
        assert [(p.time_s, p.speed_kmh) for p in traj.reference_speed] == [
            (0.0, 0.0), (20.0, 80.0), (40.0, 0.0)
        ]

    def test_ramp_rate_within_mode_envelope(self) -> None:
        """追加ランプの傾きは元モードの最大 |dv/dt| 以内（安全包絡内）。"""
        mode = self._long_mode()
        src_rates = [
            abs(b.speed_kmh - a.speed_kmh) / (b.time_s - a.time_s)
            for a, b in zip(mode.reference_speed[:-1], mode.reference_speed[1:], strict=True)
        ]
        max_src_rate = max(src_rates)
        traj = build_tuning_trajectory_from_mode(mode, max_duration_s=120.0)
        pts = traj.reference_speed
        for a, b in zip(pts[:-1], pts[1:], strict=True):
            dt = b.time_s - a.time_s
            assert dt > 0.0
            rate = abs(b.speed_kmh - a.speed_kmh) / dt
            assert rate <= max_src_rate + 1e-6


class TestTuningCost:
    def _kpi(self, **kw: float) -> dict[str, float]:
        base = {
            "n_samples": 1000.0,
            "p95_kmh": 0.1,
            "max_abs_deviation_kmh": 0.3,
            "reversal_max_per_5s": 1.0,
            "hard_limit_violations": 0.0,
        }
        base.update(kw)
        return base

    def test_no_samples_is_invalid(self) -> None:
        assert tuning_cost(self._kpi(n_samples=0.0)) == INVALID_COST

    def test_hard_violation_adds_bounded_step(self) -> None:
        """Stage B: ハード違反は 1.0 の離散段差を足す（旧 100×回数の支配を廃止）。
        僅かな超過(積分ゼロ近傍)では段差 ~1.0 のみで、局所解の壁を作らない。"""
        good = tuning_cost(self._kpi())
        bad = tuning_cost(self._kpi(hard_limit_violations=1.0))
        assert bad - good == pytest.approx(1.0)

    def test_over_limit_integral_gives_gradient(self) -> None:
        """Stage B: 回数が同じ(hard=1)でも超過量×時間の積分が大きいほどコストが単調増加し、
        座標降下に勾配を与える（旧 100×回数では積分に無反応で停滞した）。"""
        base = self._kpi(hard_limit_violations=1.0)
        c_small = tuning_cost({**base, "over_limit_integral_kmhs": 0.5})
        c_mid = tuning_cost({**base, "over_limit_integral_kmhs": 2.0})
        c_large = tuning_cost({**base, "over_limit_integral_kmhs": 5.0})
        assert c_small < c_mid < c_large
        # 連続項の効き（10×積分）が段差(1.0)より支配的になりうる
        assert c_large - c_small == pytest.approx(10.0 * (5.0 - 0.5))

    def test_zero_violation_beats_any_violation(self) -> None:
        """違反ゼロ達成は、僅かでも違反が残る走行より必ず低コスト（段差インセンティブ）。"""
        clean = tuning_cost(self._kpi(hard_limit_violations=0.0, over_limit_integral_kmhs=0.0))
        barely = tuning_cost(
            self._kpi(hard_limit_violations=1.0, over_limit_integral_kmhs=0.001)
        )
        assert clean < barely

    def test_better_kpi_lowers_cost(self) -> None:
        worse = tuning_cost(self._kpi(p95_kmh=0.4, max_abs_deviation_kmh=0.8))
        better = tuning_cost(self._kpi(p95_kmh=0.1, max_abs_deviation_kmh=0.2))
        assert better < worse

    def test_reversal_at_kpi_limit_contributes_full_weight(self) -> None:
        """B-6: KPI 上限ちょうど(reversal=1)の走行は reversal 項として重み 1.0 を計上する
        （0.2→1.0 に引き上げ。振動 KPI 超過を p95 と同格でペナルティする）。"""
        base = tuning_cost(self._kpi(reversal_max_per_5s=0.0))
        at_limit = tuning_cost(self._kpi(reversal_max_per_5s=1.0))
        assert at_limit - base == pytest.approx(1.0)

    def test_oscillatory_reversal_outweighs_small_p95_gain(self) -> None:
        """B-6 回帰テスト: p95 がわずかに良くても reversal が多い(振動的な)候補は
        コストが高くなり、座標降下がその候補を採用しない（重み1.0で強化）。"""
        smooth = tuning_cost(self._kpi(p95_kmh=0.10, reversal_max_per_5s=0.0))
        oscillatory = tuning_cost(self._kpi(p95_kmh=0.08, reversal_max_per_5s=5.0))
        assert oscillatory > smooth

    def test_reversal_dominates_tiny_p95_improvement(self) -> None:
        """B-6: 実機ゲートBの失敗パターン（p95微改善だが振動大）を高コスト化する。
        reversal 14回/5s は 0.2 重みでは p95 項に埋もれたが 1.0 重みでは支配的になる。"""
        # 実機ゲートB規定パターン相当: p95=1.28, reversal=14 の候補が
        # p95=0.5, reversal=2 の候補より高コスト（＝選ばれない）であること。
        oscillatory = tuning_cost(self._kpi(p95_kmh=1.28, reversal_max_per_5s=14.0))
        calm = tuning_cost(self._kpi(p95_kmh=0.5, reversal_max_per_5s=2.0))
        assert oscillatory > calm

    def test_pedal_activity_adds_monotonic_penalty(self) -> None:
        """B-7-4: アクセル ON-OFF 回数/min が多いほどコストが単調増加する。"""
        from src.domain.pid_tuning import PEDAL_ACTIVITY_WEIGHT

        smooth = tuning_cost(self._kpi(accel_on_per_min=5.0))
        hunting = tuning_cost(self._kpi(accel_on_per_min=20.0))
        assert hunting > smooth
        assert hunting - smooth == pytest.approx(PEDAL_ACTIVITY_WEIGHT * (20.0 - 5.0))

    def test_pedal_activity_absent_is_zero(self) -> None:
        """ペダル情報が無い summary（accel_on_per_min 欠損）では影響しない。"""
        base = self._kpi()  # accel_on_per_min を含まない
        with_smooth = tuning_cost({**base, "accel_on_per_min": 0.0})
        assert tuning_cost(base) == pytest.approx(with_smooth)

    def test_pedal_activity_does_not_dominate_reversal(self) -> None:
        """B-7-4: ペダル項は支配項（reversal）を上書きしない校正になっている。
        実機ハンチング(20回/min)の寄与でも reversal KPI 超過(21回)より小さい。"""
        pedal_contrib = tuning_cost(self._kpi(accel_on_per_min=20.0)) - tuning_cost(self._kpi())
        reversal_contrib = tuning_cost(self._kpi(reversal_max_per_5s=21.0)) - tuning_cost(
            self._kpi(reversal_max_per_5s=0.0)
        )
        assert pedal_contrib < reversal_contrib

    def test_seesaw_adds_monotonic_penalty(self) -> None:
        """B-8-4: アクセル⇔ブレーキ交互踏み(回/min)が多いほどコストが単調増加する。"""
        from src.domain.pid_tuning import PEDAL_SEESAW_WEIGHT

        smooth = tuning_cost(self._kpi(pedal_switch_per_min=2.0))
        seesaw = tuning_cost(self._kpi(pedal_switch_per_min=20.0))
        assert seesaw > smooth
        assert seesaw - smooth == pytest.approx(PEDAL_SEESAW_WEIGHT * (20.0 - 2.0))

    def test_seesaw_absent_is_zero(self) -> None:
        """交互踏み情報が無い summary（pedal_switch_per_min 欠損）では影響しない。"""
        base = self._kpi()
        with_zero = tuning_cost({**base, "pedal_switch_per_min": 0.0})
        assert tuning_cost(base) == pytest.approx(with_zero)


class TestCoordinateDescentTuner:
    def test_converges_on_convex_cost(self) -> None:
        # 既知の凸コスト: 最適点 (kp,ki,kd) = (2.0, 1.0, 0.5)。pid_preview_s は無関係。
        def cost(g: TuningParams) -> float:
            return (g.kp - 2.0) ** 2 + (g.ki - 1.0) ** 2 + (g.kd - 0.5) ** 2

        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=200)
        while (cand := tuner.next_candidate()) is not None:
            tuner.report(cand, cost(cand))
        best = tuner.best
        assert best.kp == pytest.approx(2.0, abs=0.3)
        assert best.ki == pytest.approx(1.0, abs=0.3)
        assert best.kd == pytest.approx(0.5, abs=0.3)

    def test_stops_at_max_runs(self) -> None:
        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=5)
        count = 0
        while (cand := tuner.next_candidate()) is not None:
            tuner.report(cand, 1.0)  # 改善しないコスト
            count += 1
        assert count <= 5

    def test_keeps_best_gains(self) -> None:
        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=20)
        # 最初の候補（ベースライン）だけ良コスト、以降は悪コスト
        first = True
        while (cand := tuner.next_candidate()) is not None:
            tuner.report(cand, 0.0 if first else 10.0)
            first = False
        assert tuner.best == TuningParams(kp=1.0, ki=0.1, kd=0.0)
        assert tuner.best_cost == 0.0

    def test_kd_candidate_evaluated_within_budget(self) -> None:
        """kp/ki が改善し続ける形状でも、遅くとも7走行目までに kd 候補が評価される。"""

        def cost(g: TuningParams) -> float:
            return -(g.kp + g.ki)  # kp/ki は大きいほど無制限に改善し続ける

        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=15)
        kd_seen_at_run: int | None = None
        run_idx = 0
        while (cand := tuner.next_candidate()) is not None:
            run_idx += 1
            tuner.report(cand, cost(cand))
            if cand.kd != 0.0 and kd_seen_at_run is None:
                kd_seen_at_run = run_idx
        assert kd_seen_at_run is not None
        assert kd_seen_at_run <= 7

    def test_kd_improves_when_beneficial(self) -> None:
        """kd のみ改善が効く凸コストでは best.kd が実際に更新される。"""

        def cost(g: TuningParams) -> float:
            return (g.kd - 2.0) ** 2  # kp/ki/preview は無関係（コストに影響しない）

        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=200)
        while (cand := tuner.next_candidate()) is not None:
            tuner.report(cand, cost(cand))
        assert tuner.best.kd > 0.0
        assert tuner.best.kd == pytest.approx(2.0, abs=0.3)

    def test_pid_preview_s_improves_when_beneficial(self) -> None:
        """pid_preview_s のみ改善が効く凸コストでは best.pid_preview_s が実際に更新される。"""

        def cost(g: TuningParams) -> float:
            return (g.pid_preview_s - 0.8) ** 2  # kp/ki/kd は無関係（[0,1.0] 内の最適点）

        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=200)
        while (cand := tuner.next_candidate()) is not None:
            tuner.report(cand, cost(cand))
        assert tuner.best.pid_preview_s == pytest.approx(0.8, abs=0.2)

    def test_pid_preview_s_clamped_to_max(self) -> None:
        """コストが pid_preview_s の増加を際限なく好む場合でも PID_PREVIEW_MAX_S を超えない。"""

        def cost(g: TuningParams) -> float:
            return -g.pid_preview_s

        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=200)
        while (cand := tuner.next_candidate()) is not None:
            tuner.report(cand, cost(cand))
            assert cand.pid_preview_s <= PID_PREVIEW_MAX_S
        assert tuner.best.pid_preview_s == pytest.approx(PID_PREVIEW_MAX_S)

    def test_no_duplicate_clamped_candidates(self) -> None:
        """kd=0 初期でクランプにより best と同値になる候補（−側等）は生成されない。"""
        initial = TuningParams(kp=0.0, ki=0.0, kd=0.0)
        tuner = CoordinateDescentTuner(initial, max_runs=50)
        seen: list[TuningParams] = []
        while (cand := tuner.next_candidate()) is not None:
            seen.append(cand)
            tuner.report(cand, 1.0)  # 常に据え置き（ベースラインだけ採用される）
        # ベースライン以降の候補はどれか1軸が非ゼロになるため、initial と同値にはならない
        for cand in seen[1:]:
            assert cand != initial

    def test_step_halves_after_full_cycle_without_improvement(self) -> None:
        """1巡（kp→ki→kd→pid_preview_s）で改善がなければステップが半減する。"""
        tuner = CoordinateDescentTuner(
            TuningParams(kp=0.0, ki=0.0, kd=0.0),
            max_runs=50,
            init_step_frac=0.3,
            min_step_frac=0.05,
        )
        baseline = tuner.next_candidate()
        tuner.report(baseline, 1.0)

        kp_plus = tuner.next_candidate()
        assert kp_plus is not None
        assert kp_plus.kp == pytest.approx(0.3)  # 0.3 * (0 + _BASE["kp"]=1.0)
        tuner.report(kp_plus, 1.0)  # 改善なし（kp- は best と同値になりスキップされる）

        ki_plus = tuner.next_candidate()
        assert ki_plus is not None
        assert ki_plus.ki == pytest.approx(0.03)  # 0.3 * (0 + _BASE["ki"]=0.1)
        tuner.report(ki_plus, 1.0)

        kd_plus = tuner.next_candidate()
        assert kd_plus is not None
        assert kd_plus.kd == pytest.approx(0.15)  # 0.3 * (0 + _BASE["kd"]=0.5)
        tuner.report(kd_plus, 1.0)  # 改善なし（kd- は best と同値になりスキップされる）

        preview_plus = tuner.next_candidate()
        assert preview_plus is not None
        assert preview_plus.pid_preview_s == pytest.approx(0.075)  # 0.3*(0+_BASE["preview"]=0.25)
        tuner.report(preview_plus, 1.0)  # 1巡改善なしで完了 → ステップ半減

        second_cycle_kp_plus = tuner.next_candidate()
        assert second_cycle_kp_plus is not None
        assert second_cycle_kp_plus.kp == pytest.approx(0.15)  # 半減後: 0.15 * 1.0
