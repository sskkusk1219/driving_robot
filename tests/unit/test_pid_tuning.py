"""PID 自動適合（pid_tuning）のユニットテスト。"""

import math
from datetime import UTC, datetime, timedelta

import pytest

from src.domain.pid_tuning import (
    FOPDT,
    INVALID_COST,
    KP_MAX,
    PREVIEW_MAX_S,
    CoordinateDescentTuner,
    TuningParams,
    build_tuning_trajectory,
    compute_pid_gains_simc,
    identify_fopdt,
    initial_preview_from_fopdt,
    tuning_cost,
)
from src.models.drive_log import DriveLog
from src.models.profile import PIDGains, StopConfig, VehicleProfile

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
    def test_uses_theta_directly_within_range(self) -> None:
        assert initial_preview_from_fopdt(FOPDT(k=0.5, tau=2.0, theta=0.8)) == pytest.approx(0.8)

    def test_clamps_to_preview_max_s(self) -> None:
        # L_MAX_S(=PREVIEW_MAX_S)=3.0 を超えるむだ時間はクランプされる
        assert initial_preview_from_fopdt(
            FOPDT(k=0.5, tau=2.0, theta=10.0)
        ) == pytest.approx(PREVIEW_MAX_S)

    def test_clamps_to_zero_for_nonpositive_theta(self) -> None:
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

    @pytest.mark.parametrize(
        ("max_speed", "max_decel_g"),
        [(100.0, 0.3), (120.0, 0.5), (60.0, 0.2), (80.0, 0.1)],
    )
    def test_decel_rate_within_max_g(self, max_speed: float, max_decel_g: float) -> None:
        from src.domain.pid_tuning import G_TO_KMHS

        prof = make_profile()
        prof.max_speed = max_speed
        prof.max_decel_g = max_decel_g
        mode = build_tuning_trajectory(prof)
        pts = mode.reference_speed

        limit_kmhs = max_decel_g * G_TO_KMHS  # 上限G [km/h/s]
        for a, b in zip(pts[:-1], pts[1:], strict=True):
            dt = b.time_s - a.time_s
            assert dt > 0.0
            if b.speed_kmh < a.speed_kmh:  # 減速区間
                decel_rate = (a.speed_kmh - b.speed_kmh) / dt
                assert decel_rate <= limit_kmhs + 1e-9
            assert b.speed_kmh <= max_speed + 1e-9


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

    def test_hard_violation_dominates(self) -> None:
        good = tuning_cost(self._kpi())
        bad = tuning_cost(self._kpi(hard_limit_violations=1.0))
        assert bad > good + 50.0

    def test_better_kpi_lowers_cost(self) -> None:
        worse = tuning_cost(self._kpi(p95_kmh=0.4, max_abs_deviation_kmh=0.8))
        better = tuning_cost(self._kpi(p95_kmh=0.1, max_abs_deviation_kmh=0.2))
        assert better < worse


class TestCoordinateDescentTuner:
    def test_converges_on_convex_cost(self) -> None:
        # 既知の凸コスト: 最適点 (kp,ki,kd) = (2.0, 1.0, 0.5)。preview_time_s は無関係。
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

    def test_preview_time_s_improves_when_beneficial(self) -> None:
        """preview_time_s のみ改善が効く凸コストでは best.preview_time_s が実際に更新される。"""

        def cost(g: TuningParams) -> float:
            return (g.preview_time_s - 1.2) ** 2  # kp/ki/kd は無関係

        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=200)
        while (cand := tuner.next_candidate()) is not None:
            tuner.report(cand, cost(cand))
        assert tuner.best.preview_time_s == pytest.approx(1.2, abs=0.3)

    def test_preview_time_s_clamped_to_max(self) -> None:
        """コストが preview_time_s の増加を際限なく好む場合でも PREVIEW_MAX_S を超えない。"""

        def cost(g: TuningParams) -> float:
            return -g.preview_time_s

        tuner = CoordinateDescentTuner(TuningParams(kp=1.0, ki=0.1, kd=0.0), max_runs=200)
        while (cand := tuner.next_candidate()) is not None:
            tuner.report(cand, cost(cand))
            assert cand.preview_time_s <= PREVIEW_MAX_S
        assert tuner.best.preview_time_s == pytest.approx(PREVIEW_MAX_S)

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
        """1巡（kp→ki→kd→preview_time_s）で改善がなければステップが半減する。"""
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
        assert preview_plus.preview_time_s == pytest.approx(0.15)  # 0.3*(0+_BASE["preview"]=0.5)
        tuner.report(preview_plus, 1.0)  # 1巡改善なしで完了 → ステップ半減

        second_cycle_kp_plus = tuner.next_candidate()
        assert second_cycle_kp_plus is not None
        assert second_cycle_kp_plus.kp == pytest.approx(0.15)  # 半減後: 0.15 * 1.0
