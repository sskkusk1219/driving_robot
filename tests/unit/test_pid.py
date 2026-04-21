import pytest

from src.domain.control.pid import PIDController

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
    def test_d_first_step(self) -> None:
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT)
        # prev_error=0 → error=10 → derivative=(10-0)/0.05=200
        output = pid.update(setpoint=60.0, measurement=50.0)
        assert output == pytest.approx(200.0)

    def test_d_constant_error(self) -> None:
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT)
        pid.update(setpoint=60.0, measurement=50.0)
        # error が同じなら derivative=0
        output = pid.update(setpoint=60.0, measurement=50.0)
        assert output == pytest.approx(0.0)

    def test_d_decreasing_error(self) -> None:
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, dt=DT)
        pid.update(setpoint=60.0, measurement=50.0)  # prev_error=10
        # error が 5 に減少 → derivative=(5-10)/0.05=-100
        output = pid.update(setpoint=60.0, measurement=55.0)
        assert output == pytest.approx(-100.0)


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
        # リセット後は積分・前回偏差が0になる
        assert pid._integral == pytest.approx(0.0)
        assert pid._prev_error == pytest.approx(0.0)

    def test_integral_grows_without_clamp(self) -> None:
        """積分クランプなしでは偏差が続くほど出力が増加することを確認（設計上クランプなし、呼び出し元でクランプする）。"""
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, dt=DT)
        outputs = [pid.update(setpoint=100.0, measurement=0.0) for _ in range(10)]
        assert outputs[-1] > outputs[0]
