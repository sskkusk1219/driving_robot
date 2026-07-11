# 微分項1次ローパスフィルタの時定数 [s]。公称 dt=50ms の4倍で、CAN 車速の量子化に
# よる1サンプルスパイクを dt/(τf+dt)=0.2 倍に抑制しつつ、FOPDT 同定の τ（秒オーダー）
# より十分速く位相遅れは実用上無視できる。
DERIV_FILTER_TAU_S: float = 0.2


class PIDController:
    """離散時間PIDコントローラ（後退差分）。dt = 制御ループ周期 (デフォルト 50ms)。

    アンチワインドアップ:
    - 条件付き積分: 下流（PedalArbiter / 開度クランプ）が飽和している方向の誤差では
      積分を停止する。飽和は呼び出し元が前サイクルの飽和フラグとして渡す。
    - 積分量クランプ: I 項単独で出力上限を超えないよう |integral| ≤ output_limit/ki。
    - 出力クランプ: 戻り値を ±output_limit に制限（FF の補助という役割を機構で保証）。

    dt はサイクルスキップ（バス遅延）時の微分スパイク・積分過小評価を防ぐため、
    呼び出し元が計測値を渡せる。公称 dt の 0.5〜4 倍にクランプして外れ値を防ぐ。
    """

    _kp: float
    _ki: float
    _kd: float
    _dt: float
    _output_limit: float
    _integral: float
    _prev_error: float
    _d_filt: float

    def __init__(
        self, kp: float, ki: float, kd: float, dt: float = 0.05, output_limit: float = 100.0
    ) -> None:
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._dt = dt
        self._output_limit = abs(output_limit)
        self._integral = 0.0
        self._prev_error = 0.0
        self._d_filt = 0.0

    def update(
        self,
        setpoint: float,
        measurement: float,
        dt: float | None = None,
        *,
        saturated_high: bool = False,
        saturated_low: bool = False,
        gain_scale: float = 1.0,
    ) -> float:
        """偏差を入力してPID出力を返す。

        Args:
            setpoint: 基準値（基準車速 [km/h]）
            measurement: 計測値（実車速 [km/h]）
            dt: 前回更新からの計測経過時間 [s]。None なら公称周期を使う。
            saturated_high: 前サイクルで正方向（加速側）の出力が飽和していた
            saturated_low: 前サイクルで負方向（制動側）の出力が飽和していた
            gain_scale: 速度依存プラントゲイン正規化係数（>0）。P/I/D 合成出力を一律に
                この係数で乗算する。プラントゲインが速度で大きく変わる（高速域は開度あたりの
                加速が小さい）ため、FF モデルの局所勾配から求めた比で PID の実効ゲインを
                速度域で一定に保ち、高速域の振動・低速域の応答不足を抑える。1.0 で従来動作。
                積分器は誤差単位のまま保持し、積分クランプのみスケール整合させる（緩変化する
                スケールに対して安全）。
        """
        if dt is None or dt <= 0.0:
            dt_eff = self._dt
        else:
            # スケジューラジッタ・サイクルスキップの外れ値を抑える
            dt_eff = max(0.5 * self._dt, min(4.0 * self._dt, dt))

        scale = gain_scale if gain_scale > 0.0 else 1.0
        error = setpoint - measurement
        # 飽和方向へ誤差が押している間は積分しない（ワインドアップ防止）
        pushing_into_saturation = (error > 0.0 and saturated_high) or (
            error < 0.0 and saturated_low
        )
        if not pushing_into_saturation:
            self._integral += error * dt_eff
            if self._ki > 0.0:
                # 出力段で scale を掛けるため、I 項単独の出力上限は output_limit/(ki*scale)。
                integral_limit = self._output_limit / (self._ki * scale)
                self._integral = max(-integral_limit, min(integral_limit, self._integral))

        d_raw = (error - self._prev_error) / dt_eff
        self._prev_error = error
        self._d_filt += (d_raw - self._d_filt) * dt_eff / (DERIV_FILTER_TAU_S + dt_eff)
        output = scale * (self._kp * error + self._ki * self._integral + self._kd * self._d_filt)
        return max(-self._output_limit, min(self._output_limit, output))

    def reset(self) -> None:
        """積分器・前回偏差・微分フィルタ状態をリセットする。走行開始・停止時に呼ぶ。"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._d_filt = 0.0

    def set_gains(self, kp: float, ki: float, kd: float) -> None:
        """ゲインを更新し内部状態をリセットする。プロファイル選択時に呼ぶ。"""
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self.reset()

    def set_output_limit(self, limit: float) -> None:
        """出力権限上限 [%] を更新する。プロファイル選択時に pid_output_limit_pct を反映する。"""
        self._output_limit = abs(limit)
