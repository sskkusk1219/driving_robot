class PIDController:
    """離散時間PIDコントローラ（後退差分）。dt = 制御ループ周期 (デフォルト 50ms)。"""

    _kp: float
    _ki: float
    _kd: float
    _dt: float
    _integral: float
    _prev_error: float

    def __init__(self, kp: float, ki: float, kd: float, dt: float = 0.05) -> None:
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._dt = dt
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, setpoint: float, measurement: float) -> float:
        """偏差を入力してPID出力を返す。呼び出し周期は dt と一致させること。"""
        error = setpoint - measurement
        self._integral += error * self._dt
        derivative = (error - self._prev_error) / self._dt
        self._prev_error = error
        return self._kp * error + self._ki * self._integral + self._kd * derivative

    def reset(self) -> None:
        """積分器と前回偏差をリセットする。走行開始・停止時に呼ぶ。"""
        self._integral = 0.0
        self._prev_error = 0.0
