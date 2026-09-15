"""update_plan（エピソード型プラン更新則）のユニットテスト。

収束性は FOPDT（1次遅れ+むだ時間、実機同定値 k=2.16/τ=2.08/θ=0.30）閉ループ
シミュレーションで検証する（test_trim.py と同じプラント）。
"""

import numpy as np
import pytest

from src.domain.control.pedal_plan import PedalPlan, PlanPhase
from src.domain.control.plan_update import (
    MIN_LOGS_FOR_UPDATE,
    l_gain_at,
    l_gain_from_fopdt,
    lead_time_from_fopdt,
    update_plan,
)
from src.models.profile import FeedforwardParams

# 実機同定 FOPDT（test_trim と共用）。v_ss = k·u。
FOPDT_K = 2.16
FOPDT_TAU = 2.08
FOPDT_THETA = 0.30
DT = 0.1


def _simulate_fopdt(
    efforts: list[float],
    dt: float = DT,
    *,
    k: float = FOPDT_K,
    tau: float = FOPDT_TAU,
    theta: float = FOPDT_THETA,
    v0: float,
) -> list[float]:
    """applied effort 列から FOPDT プラントの実車速列を生成する。v_ss = k·u。"""
    n = len(efforts)
    delay = int(round(theta / dt))
    v = v0
    out: list[float] = []
    for i in range(n):
        u_delayed = efforts[i - delay] if i - delay >= 0 else efforts[0]
        v += (k * u_delayed - v) / tau * dt
        out.append(v)
    return out


def _p95(errors: list[float]) -> float:
    return float(np.percentile(np.abs(errors), 95))


class TestConvergence:
    """反復でプラン更新すると追従誤差が単調に縮む。"""

    def test_cruise_tracking_converges(self) -> None:
        # 40km/h 定常巡航（全 DRIVE）。FF由来プランは必要 effort(≈18.5%) より 5% 低い 17.6%。
        n = 301  # 30s / 0.1s
        cruise = 40.0
        ref = [cruise] * n
        times = [i * DT for i in range(n)]
        phases = [PlanPhase.DRIVE] * n
        ff_base = PedalPlan(dt_s=DT, efforts=[17.6] * n, phases=list(phases))
        params = FeedforwardParams()
        l_gain = l_gain_from_fopdt(FOPDT_K)

        plan = ff_base
        p95s: list[float] = []
        for _ in range(6):
            u = list(plan.efforts)
            actual = _simulate_fopdt(u, v0=FOPDT_K * u[0])
            errors = [r - a for r, a in zip(ref, actual)]
            p95s.append(_p95(errors))
            updated = update_plan(
                ff_base, times, u, ref, actual, params,
                l_gain=l_gain, delta_s=FOPDT_THETA,
            )
            assert updated is not None
            plan = updated

        # p95 は単調非増加、最終は初回の半分未満。
        for prev, cur in zip(p95s, p95s[1:]):
            assert cur <= prev + 1e-9
        assert p95s[-1] < p95s[0] / 2.0


class TestPhasePreservation:
    """更新後もフェーズ列は固定され、フェーズ整合クランプが効く。"""

    def _mixed_plan(self, n: int) -> PedalPlan:
        # DRIVE / COAST / BRAKE / STOP_HOLD を各 1/4 ずつ。
        phases: list[PlanPhase] = []
        efforts: list[float] = []
        for i in range(n):
            if i < n // 4:
                phases.append(PlanPhase.DRIVE)
                efforts.append(15.0)
            elif i < n // 2:
                phases.append(PlanPhase.COAST)
                efforts.append(0.0)
            elif i < 3 * n // 4:
                phases.append(PlanPhase.BRAKE)
                efforts.append(-10.0)
            else:
                phases.append(PlanPhase.STOP_HOLD)
                efforts.append(-20.0)
        return PedalPlan(dt_s=DT, efforts=efforts, phases=phases)

    def test_phases_and_clamps_preserved(self) -> None:
        n = 100
        base = self._mixed_plan(n)
        params = FeedforwardParams()
        times = [i * DT for i in range(n)]
        # 逸脱を誘うノイズ的な applied（正負を跨ぐ）と残差。
        applied = [5.0 * (-1) ** i for i in range(n)]
        ref = [30.0] * n
        actual = [30.0 + (-1) ** i for i in range(n)]
        updated = update_plan(
            base, times, applied, ref, actual, params, l_gain=0.2, delta_s=0.3
        )
        assert updated is not None
        # フェーズ列は不変。
        assert updated.phases == base.phases
        stop_effort = -max(params.stop_brake_opening_pct, params.brake_deadband_pct)
        for e, ph in zip(updated.efforts, updated.phases):
            if ph == PlanPhase.DRIVE:
                assert e >= 0.0
            elif ph == PlanPhase.BRAKE:
                assert e <= 0.0
            elif ph == PlanPhase.COAST:
                assert e == 0.0
            else:  # STOP_HOLD
                assert e == stop_effort


class TestDeltaClamp:
    """FF由来プランとの差分が ±10% にクランプされる。"""

    def test_large_applied_clamped_to_10pct(self) -> None:
        n = 100
        base = PedalPlan(dt_s=DT, efforts=[20.0] * n, phases=[PlanPhase.DRIVE] * n)
        params = FeedforwardParams()
        times = [i * DT for i in range(n)]
        # 極端に大きい applied → 差分は +10% で頭打ち → 30% になるはず。
        applied = [100.0] * n
        ref = [40.0] * n
        actual = [40.0] * n  # 残差 0 なので L·e 項は無し
        updated = update_plan(
            base, times, applied, ref, actual, params, l_gain=0.2, delta_s=0.3
        )
        assert updated is not None
        for e in updated.efforts:
            assert e <= 30.0 + 1e-6
            assert e >= 20.0 - 1e-6


class TestDeadbandSnap:
    """更新後 effort が不感帯未満なら 0（or ±deadband 以上）に量子化される（F2 対策）。"""

    def test_sub_deadband_update_snaps_to_zero(self) -> None:
        n = 100
        base = PedalPlan(dt_s=DT, efforts=[2.0] * n, phases=[PlanPhase.DRIVE] * n)
        params = FeedforwardParams()  # accel_deadband_pct=1.0（半分=0.5）
        times = [i * DT for i in range(n)]
        applied = [0.3] * n  # 不感帯未満（db/2=0.5 未満）
        ref = [30.0] * n
        actual = [30.0] * n  # 残差 0 → filtered ≈ applied
        updated = update_plan(base, times, applied, ref, actual, params, l_gain=0.2, delta_s=0.3)
        assert updated is not None
        for e in updated.efforts:
            assert e == 0.0


class TestGuards:
    def _base(self, n: int) -> PedalPlan:
        return PedalPlan(dt_s=DT, efforts=[10.0] * n, phases=[PlanPhase.DRIVE] * n)

    def test_too_few_logs_returns_none(self) -> None:
        n = MIN_LOGS_FOR_UPDATE - 1
        base = self._base(200)
        times = [i * DT for i in range(n)]
        assert update_plan(
            base, times, [10.0] * n, [30.0] * n, [30.0] * n, FeedforwardParams(),
            l_gain=0.2, delta_s=0.3,
        ) is None

    def test_insufficient_coverage_returns_none(self) -> None:
        # プランは 30s 分だがログは前半 10s しかない → 完走していない。
        base = self._base(301)
        n = 101  # 10s
        times = [i * DT for i in range(n)]
        assert update_plan(
            base, times, [10.0] * n, [30.0] * n, [30.0] * n, FeedforwardParams(),
            l_gain=0.2, delta_s=0.3,
        ) is None

    def test_empty_base_returns_none(self) -> None:
        base = PedalPlan(dt_s=DT, efforts=[], phases=[])
        n = 60
        times = [i * DT for i in range(n)]
        assert update_plan(
            base, times, [10.0] * n, [30.0] * n, [30.0] * n, FeedforwardParams(),
            l_gain=0.2, delta_s=0.3,
        ) is None

    def test_length_mismatch_returns_none(self) -> None:
        base = self._base(200)
        n = 60
        times = [i * DT for i in range(n)]
        assert update_plan(
            base, times, [10.0] * (n - 1), [30.0] * n, [30.0] * n, FeedforwardParams(),
            l_gain=0.2, delta_s=0.3,
        ) is None


class TestLearningGainFromPlant:
    """ILC 学習ゲインを積分系プラント（ペダルゲイン曲線）から求める（実機 9eee549b）。

    旧実装 L = 0.25/fopdt_k は fopdt_k=3.51 から L=0.071 %/(km/h) となり、4km/h の誤差に
    対してプラン補正が 0.28% しか出ない＝実質学習していなかった（±8% のクランプに対して
    桁が違う）。しかも fopdt_k は学習運転がプラトーに達しないと保持区間長を測るだけの値。
    """

    @staticmethod
    def _params() -> FeedforwardParams:
        return FeedforwardParams(
            pedal_gain_speeds_kmh=(0.0, 100.0),
            accel_gain_kmhs_per_pct=(0.5, 0.25),
            brake_gain_kmhs_per_pct=(0.25, 1.0),
        )

    def test_gain_is_inverse_of_pedal_gain_and_lead(self) -> None:
        """L = factor / (k'(v)·Δ)。"""
        got = l_gain_at(self._params(), 0.0, 2.0, is_accel=True, factor=0.25)
        assert got == pytest.approx(0.25 / (0.5 * 2.0))

    def test_gain_varies_with_speed_and_direction(self) -> None:
        p = self._params()
        assert l_gain_at(p, 100.0, 1.0, is_accel=True) == pytest.approx(
            l_gain_at(p, 0.0, 1.0, is_accel=True) * 2.0
        )
        # 同じ 100km/h でも制動側はゲインが 4 倍大きい → L は 1/4
        assert l_gain_at(p, 100.0, 1.0, is_accel=False) == pytest.approx(
            l_gain_at(p, 100.0, 1.0, is_accel=True) / 4.0
        )

    def test_much_larger_than_old_fopdt_gain(self) -> None:
        """実機の同定値で旧実装より桁が上がる（0.071 → 0.5 以上）。"""
        assert l_gain_from_fopdt(3.5115) == pytest.approx(0.0712, abs=1e-4)
        assert l_gain_at(self._params(), 0.0, 1.0, is_accel=True) > 0.4

    def test_falls_back_when_unidentified(self) -> None:
        """ペダルゲイン未同定・Δ≤0 は fallback をそのまま返す（従来動作）。"""
        assert l_gain_at(FeedforwardParams(), 50.0, 1.0, is_accel=True, fallback=0.9) == 0.9
        assert l_gain_at(self._params(), 50.0, 0.0, is_accel=True, fallback=0.9) == 0.9


class TestLeadTime:
    """むだ時間シフト Δ から τ 項を外した（τ が無効な同定値のため）。"""

    def test_delta_is_theta_plus_response(self) -> None:
        assert lead_time_from_fopdt(0.5, response_s=0.5) == pytest.approx(1.0)

    def test_tau_is_ignored(self) -> None:
        """旧シグネチャ互換で tau を受けるが結果に影響しない。"""
        assert lead_time_from_fopdt(0.5, 99.0, response_s=0.5) == pytest.approx(1.0)

    def test_unidentified_theta_uses_response_only(self) -> None:
        assert lead_time_from_fopdt(None, response_s=0.5) == pytest.approx(0.5)
