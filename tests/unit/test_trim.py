"""トリム制御（TrimController）のユニットテスト。

3 層（凍結／低速トリム／速い補正層）の遷移・レート制限・量子化・ヒステリシス・
バンプレス切替・アンチワインドアップ・reset・FOPDT シミュレーションでの収束を検証する。
"""

import numpy as np
import pytest

from src.domain.control.pedal_plan import PlanPhase
from src.domain.control.pid import PIDController
from src.domain.control.trim import (
    FAST_ENGAGE_KMH,
    TRIM_RATE_PCT_S,
    TRIM_STEP_PCT,
    TrimController,
)


def _fast_pid() -> PIDController:
    return PIDController(kp=3.9, ki=0.88, kd=0.27, dt=0.05, output_limit=50.0)


class TestHoldBand:
    def test_freezes_output_within_hold_band(self) -> None:
        trim = TrimController(_fast_pid())
        # まず低速層で少し出力を作る
        for _ in range(20):
            trim.update(60.3, 60.0, 0.05, phase=PlanPhase.DRIVE)
        held = trim._output
        # 偏差を凍結帯内に入れると出力が変わらない
        out = trim.update(60.05, 60.0, 0.05, phase=PlanPhase.DRIVE)
        assert out == pytest.approx(held)

    def test_stop_hold_phase_freezes(self) -> None:
        trim = TrimController(_fast_pid())
        for _ in range(10):
            trim.update(60.3, 60.0, 0.05, phase=PlanPhase.DRIVE)
        held = trim._output
        out = trim.update(60.3, 60.0, 0.05, phase=PlanPhase.STOP_HOLD)
        assert out == pytest.approx(held)


class TestSlowLayer:
    def test_rate_limit_caps_output_change(self) -> None:
        trim = TrimController(_fast_pid())
        # 大きめの偏差（ただし FAST_ENGAGE 未満）で 1 サイクルの変化がレート上限以内
        dt = 0.05
        trim.update(60.4, 60.0, dt, phase=PlanPhase.DRIVE)
        first = trim._raw
        assert abs(first) <= TRIM_RATE_PCT_S * dt + 1e-9

    def test_quantization_holds_below_step(self) -> None:
        trim = TrimController(_fast_pid())
        # ごく小さな偏差では確定出力が量子化ステップ未満で動かない
        out0 = trim.update(60.15, 60.0, 0.05, phase=PlanPhase.DRIVE)
        assert out0 == 0.0  # 1 サイクルでは STEP 未満
        # 内部連続値は動いている（積分作用は失われない）
        assert trim._raw > 0.0

    def test_integral_action_nulls_steady_error(self) -> None:
        trim = TrimController(_fast_pid())
        # 一定偏差を与え続けると出力が単調に増えて（積分）補正しにいく
        outs = [trim.update(60.3, 60.0, 0.05, phase=PlanPhase.DRIVE) for _ in range(60)]
        assert outs[-1] > outs[10] > 0.0


class TestFastLayerHysteresis:
    def test_engages_at_threshold(self) -> None:
        trim = TrimController(_fast_pid())
        assert not trim.is_fast_active
        trim.update(60.0 + FAST_ENGAGE_KMH + 0.1, 60.0, 0.05, phase=PlanPhase.DRIVE)
        assert trim.is_fast_active

    def test_stays_engaged_until_release(self) -> None:
        trim = TrimController(_fast_pid())
        trim.update(61.0, 60.0, 0.05, phase=PlanPhase.DRIVE)  # 大偏差で介入
        assert trim.is_fast_active
        # 0.3<|dev|<0.5 では維持（離脱しない）
        trim.update(60.4, 60.0, 0.05, phase=PlanPhase.DRIVE)
        assert trim.is_fast_active
        # 0.3 未満で離脱
        trim.update(60.2, 60.0, 0.05, phase=PlanPhase.DRIVE)
        assert not trim.is_fast_active

    def test_fast_layer_large_output(self) -> None:
        trim = TrimController(_fast_pid())
        out = trim.update(62.0, 60.0, 0.05, phase=PlanPhase.DRIVE)  # 2km/h 偏差
        assert out > 1.0  # 速い層は大きく補正


class TestBumpless:
    def test_no_jump_on_fast_to_slow(self) -> None:
        trim = TrimController(_fast_pid())
        # 速い層を数サイクル
        trim.update(61.0, 60.0, 0.05, phase=PlanPhase.DRIVE)
        out_fast = trim.update(60.6, 60.0, 0.05, phase=PlanPhase.DRIVE)
        # 偏差が縮んで低速層へ落ちる（0.3未満）— 出力が段差なく引き継がれる
        out_slow = trim.update(60.2, 60.0, 0.05, phase=PlanPhase.DRIVE)
        assert abs(out_slow - out_fast) <= TRIM_RATE_PCT_S * 0.05 + TRIM_STEP_PCT + 1e-9


class TestAntiWindup:
    def test_saturated_high_stops_positive_growth(self) -> None:
        trim = TrimController(_fast_pid())
        for _ in range(40):
            trim.update(60.3, 60.0, 0.05, phase=PlanPhase.DRIVE, saturated_high=True)
        # 飽和方向へは _raw が伸びない（正の偏差＝加速方向）
        assert trim._raw == pytest.approx(0.0, abs=1e-9)


class TestReset:
    def test_reset_clears_state(self) -> None:
        trim = TrimController(_fast_pid())
        for _ in range(30):
            trim.update(60.4, 60.0, 0.05, phase=PlanPhase.DRIVE)
        trim.reset()
        assert trim._raw == 0.0
        assert trim._output == 0.0
        assert not trim.is_fast_active


class TestFOPDTConvergence:
    def test_converges_below_kpi_with_ff_residual(self) -> None:
        """FF 残差 2% 開度（定常バイアス）に対し定常偏差 < 0.2km/h に収束する（ILC なし）。"""
        k, tau, theta = 2.16, 2.08, 0.30
        dt = 0.05
        resid = 2.0  # FF が 2% 少なく出す想定
        trim = TrimController(_fast_pid())
        delay = int(round(theta / dt))
        ubuf = [0.0] * (delay + 1)
        v = 0.0
        errs = []
        for _ in range(int(60.0 / dt)):
            u_trim = trim.update(0.0, v, dt, phase=PlanPhase.DRIVE)
            ubuf.append(-resid + u_trim)
            u_del = ubuf.pop(0)
            v += (k * u_del - v) / tau * dt
            errs.append(abs(0.0 - v))
        tail = np.array(errs[int(len(errs) * 0.6) :])
        assert tail.max() < 0.2
