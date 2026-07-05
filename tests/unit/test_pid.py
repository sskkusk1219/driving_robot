import pytest

from src.domain.control.pid import DERIV_FILTER_TAU_S, PIDController

DT = 0.05  # 50ms 制御ループ周期


class TestPIDControllerInit:
    def test_default_dt(self) -> None:
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        assert pid._dt == pytest.approx(0.05)

    def test_custom_dt(self) -> None:
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.1)
        assert pid._dt == pytest.approx(0.1)


class TestPIDControllerProportional:
    def test_p_only_positive_error(self) -> None:
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, dt=DT)
        output = pid.update(setpoint=60.0, measurement=50.0)
        assert output == pytest.approx(20.0)  # 2.0 * 10.0

    def test_p_only_negative_error(self) -> None:
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, dt=DT)
        output = pid.update(setpoint=50.0, measurement=60.0)
        assert output == pytest.approx(-20.0)

    def test_p_only_zero_error(self) -> None:
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, dt=DT)
        output = pid.update(setpoint=60.0, measurement=60.0)
        assert output == pytest.approx(0.0)


class TestPIDControllerIntegral:
    def test_integral_accumulates(self) -> None:
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        pid.update(setpoint=60.0, measurement=50.0)  # error=10, integral=10*0.05=0.5
        output = pid.update(setpoint=60.0, measurement=50.0)  # integral=1.0
        assert output == pytest.approx(1.0)

    def test_integral_resets(self) -> None:
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        pid.update(setpoint=60.0, measurement=50.0)
        pid.reset()
        output = pid.update(setpoint=60.0, measurement=50.0)
        assert output == pytest.approx(0.5)  # 積分がリセットされ最初の1ステップ分のみ


class TestPIDControllerDerivative:
    """微分項は1次LPF（DERIV_FILTER_TAU_S）を通るため、後退差分の生値ではなく
    フィルタ後の値がkdに乗る。kd=0時は出力に影響しないため既存挙動と完全一致する
    （TestPIDControllerIntegral等の他クラスはkd=0のため無修正で通る）。
    """

    def test_d_first_step(self) -> None:
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT, output_limit=500.0)
        # prev_error=0, d_filt=0 → error=10 → d_raw=(10-0)/0.05=200
        # d_filt = 0 + (200-0)*0.05/(0.2+0.05) = 40
        output = pid.update(setpoint=60.0, measurement=50.0)
        assert output == pytest.approx(40.0)

    def test_d_constant_error(self) -> None:
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT)
        pid.update(setpoint=60.0, measurement=50.0)  # d_filt=40 (上と同じ)
        # error が同じなら d_raw=0 だが、フィルタ残留のため即座には0にならない
        # d_filt = 40 + (0-40)*0.2 = 32
        output = pid.update(setpoint=60.0, measurement=50.0)
        assert output == pytest.approx(32.0)

    def test_d_decreasing_error(self) -> None:
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT)
        pid.update(setpoint=60.0, measurement=50.0)  # prev_error=10, d_filt=40
        # error が 5 に減少 → d_raw=(5-10)/0.05=-100
        # d_filt = 40 + (-100-40)*0.2 = 12
        output = pid.update(setpoint=60.0, measurement=55.0)
        assert output == pytest.approx(12.0)

    def test_derivative_lpf_attenuates_spike(self) -> None:
        """1サンプル誤差ジャンプの微分項出力は未フィルタ理論値の約 dt/(τf+dt) 倍に減衰する"""
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT, output_limit=500.0)
        unfiltered_theoretical = 10.0 / DT  # (error-0)/dt = 200
        output = pid.update(setpoint=60.0, measurement=50.0)
        expected_ratio = DT / (DERIV_FILTER_TAU_S + DT)
        assert output == pytest.approx(unfiltered_theoretical * expected_ratio)
        assert output < 0.25 * unfiltered_theoretical

    def test_reset_clears_derivative_filter(self) -> None:
        """reset() 後の微分はフィルタ残留の影響を受けず、新規フィルタ状態から始まる。"""
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT, output_limit=500.0)
        pid.update(setpoint=60.0, measurement=0.0)  # 大きな誤差でフィルタ状態を積む
        pid.reset()
        # reset 後は prev_error=0, d_filt=0 に戻るため、単独の初回ステップと一致する
        output = pid.update(setpoint=60.0, measurement=50.0)
        assert output == pytest.approx(40.0)


class TestPIDControllerCombined:
    def test_step_response(self) -> None:
        """ステップ入力に対してP項が即座に反応することを確認。"""
        pid = PIDController(kp=1.0, ki=0.1, kd=0.0, dt=DT)
        output = pid.update(setpoint=100.0, measurement=0.0)
        assert output > 0.0

    def test_reset_clears_state(self) -> None:
        pid = PIDController(kp=1.0, ki=1.0, kd=1.0, dt=DT)
        for _ in range(10):
            pid.update(setpoint=100.0, measurement=0.0)
        pid.reset()
        # リセット後は積分・前回偏差・微分フィルタ状態が0になる
        assert pid._integral == pytest.approx(0.0)
        assert pid._prev_error == pytest.approx(0.0)
        assert pid._d_filt == pytest.approx(0.0)

    def test_integral_grows_while_unsaturated(self) -> None:
        """飽和していない間は偏差が続くほど I 項出力が増加する。"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        outputs = [pid.update(setpoint=100.0, measurement=0.0) for _ in range(10)]
        assert outputs[-1] > outputs[0]


class TestPIDOutputLimit:
    def test_output_clamped_to_limit(self) -> None:
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, dt=DT, output_limit=30.0)
        assert pid.update(setpoint=100.0, measurement=0.0) == pytest.approx(30.0)
        assert pid.update(setpoint=0.0, measurement=100.0) == pytest.approx(-30.0)

    def test_set_output_limit(self) -> None:
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, dt=DT, output_limit=30.0)
        pid.set_output_limit(50.0)
        assert pid.update(setpoint=100.0, measurement=0.0) == pytest.approx(50.0)


class TestPIDAntiWindup:
    def test_integral_stops_when_pushing_into_saturation_high(self) -> None:
        """正の誤差 + 加速側飽和では積分が増えない（条件付き積分）。"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        first = pid.update(setpoint=100.0, measurement=0.0, saturated_high=True)
        second = pid.update(setpoint=100.0, measurement=0.0, saturated_high=True)
        assert first == pytest.approx(0.0)
        assert second == pytest.approx(0.0)

    def test_integral_resumes_when_error_leaves_saturation_direction(self) -> None:
        """飽和方向と逆向きの誤差では飽和フラグが立っていても積分する（巻き戻し可能）。"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        output = pid.update(setpoint=0.0, measurement=10.0, saturated_high=True)
        assert output == pytest.approx(-0.5)  # error=-10 → integral=-0.5

    def test_integral_stops_when_pushing_into_saturation_low(self) -> None:
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        output = pid.update(setpoint=0.0, measurement=10.0, saturated_low=True)
        assert output == pytest.approx(0.0)

    def test_integral_magnitude_clamped_by_output_limit(self) -> None:
        """I 項単独で出力上限を超えない（|integral| ≤ limit/ki）。"""
        pid = PIDController(kp=0.0, ki=2.0, kd=0.0, dt=DT, output_limit=10.0)
        for _ in range(1000):
            pid.update(setpoint=100.0, measurement=0.0)
        assert pid._integral == pytest.approx(5.0)  # 10.0 / ki=2.0
        # 偏差が反転すれば即座に巻き戻り始める
        out_after_reversal = pid.update(setpoint=0.0, measurement=100.0)
        assert out_after_reversal < 10.0


class TestPIDMeasuredDt:
    def test_measured_dt_used_for_derivative(self) -> None:
        """計測 dt を渡すと微分(フィルタ入力)が実経過時間で計算される（スキップ時のスパイク防止）"""
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT, output_limit=500.0)
        pid.update(setpoint=60.0, measurement=50.0, dt=DT)  # prev_error=10, d_filt=40
        # 100ms 経過（1 サイクルスキップ）で誤差 10→5: d_raw=(5-10)/0.1=-50
        # d_filt = 40 + (-50-40)*0.1/(0.2+0.1) = 40 - 30 = 10
        output = pid.update(setpoint=60.0, measurement=55.0, dt=0.1)
        assert output == pytest.approx(10.0)

    def test_measured_dt_used_for_integral(self) -> None:
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        output = pid.update(setpoint=60.0, measurement=50.0, dt=0.1)
        assert output == pytest.approx(1.0)  # 10 * 0.1

    def test_dt_clamped_against_outliers(self) -> None:
        """異常に大きい dt は公称の 4 倍にクランプされる。"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        output = pid.update(setpoint=60.0, measurement=50.0, dt=10.0)
        assert output == pytest.approx(10.0 * 4 * DT)  # error * 4dt

    def test_none_dt_falls_back_to_nominal(self) -> None:
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        output = pid.update(setpoint=60.0, measurement=50.0)
        assert output == pytest.approx(0.5)
