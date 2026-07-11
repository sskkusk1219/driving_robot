"""トリム制御（TrimController）のドメインモジュール。

ペダルプランが名目 effort を供給する構成では、閉ループ補正は「大きく速く追う PID」ではなく
「人間のように小さくゆっくり直すトリム」で十分であり、そのほうがペダルが滑らかになる。
TrimController は偏差の大きさで 3 層に切り替える:

  - |偏差| ≤ HOLD_BAND_KMH        : 出力凍結（ペダルを動かさない）
  - HOLD_BAND < |偏差| < FAST_ENGAGE: 低速トリム（速度形 PI＋スルーレート制限＋量子化）
  - |偏差| ≥ FAST_ENGAGE           : 速い補正層（プロファイルの PID×ゲインスケジュール）

速い補正層は max≤1.0km/h を初回走行から守る安全網。低速トリムは ILC なしで p95≤0.2 を
出せるよう校正する（TRIM_SLOW_* 参照）。層の切替はヒステリシス（介入 FAST_ENGAGE /
離脱 FAST_RELEASE）でチャタらせず、速度形 PI により層をまたいでも出力段差が出ない
（バンプレス）。effort の符号は FF/PID/ILC と同じ（+加速/−制動 [%]）。
"""

from __future__ import annotations

from src.domain.control.pedal_plan import PlanPhase
from src.domain.control.pid import PIDController

# ── トリム 3 層のしきい値 ─────────────────────────────────────────────────
HOLD_BAND_KMH: float = 0.1  # この幅以内はペダルを動かさない（ユーザー要求）
FAST_ENGAGE_KMH: float = 0.5  # これ以上で速い補正層に入る（max≤1.0 の安全網を起動）
FAST_RELEASE_KMH: float = 0.3  # これ未満に戻るまで速い補正層を維持（層チャタ防止）

# ── 低速トリムのゲイン・整形 ───────────────────────────────────────────────
# 速度形 PI: Δoutput = KP·Δe + KI·e·dt を毎サイクル積み、レート制限・量子化して出力する。
# FOPDT(k=2.16,τ=2.08,θ=0.30) のシミュレーションで、FF 残差 2% 開度（モデル MAE 相当）を
# 定常バイアスとして与えたとき、定常偏差 0.03km/h（≪0.2）に収束し、かつ整定後はペダルが
# 動かない（滑らか）ことを机上確認して選んだ（2026-07-11、全ゲイン候補を比較）。KI が積分
# 作用で定常残差を消し、KP が過渡を抑える。量子化 0.25% は内部連続値 _raw を保持するため
# 定常精度を落とさず（プラント τ が量子化ディザを平滑化）、サーボ微振動だけ抑える。
TRIM_SLOW_KP: float = 2.0  # 低速トリム比例ゲイン [%/(km/h)]
TRIM_SLOW_KI: float = 1.2  # 低速トリム積分ゲイン [%/(km/h·s)]
TRIM_RATE_PCT_S: float = 3.0  # 低速トリム出力のスルーレート上限 [%/s]
TRIM_STEP_PCT: float = 0.25  # 低速トリム出力の量子化ステップ [%]（サーボ微振動抑制）


class TrimController:
    """偏差の大きさで凍結／低速トリム／速い補正層を切り替える閉ループ補正器。

    速い補正層はコンストラクタで渡す PIDController（プロファイルの kp/ki/kd）を内包し、
    ゲインスケジュールの gain_scale をそのまま渡す。低速トリムは速度形 PI で内部状態を持つ。
    """

    def __init__(self, fast_pid: PIDController) -> None:
        self._fast = fast_pid
        # 連続内部トリム出力（量子化前）。返り値 _output はこれを量子化した確定値。
        self._raw = 0.0
        self._output = 0.0
        self._prev_error = 0.0
        self._fast_active = False

    def reset(self) -> None:
        """内部状態と内包 PID をリセットする。走行開始・停止時に呼ぶ。"""
        self._fast.reset()
        self._raw = 0.0
        self._output = 0.0
        self._prev_error = 0.0
        self._fast_active = False

    @property
    def is_fast_active(self) -> bool:
        """速い補正層がアクティブか（drive_loop のフェーズ権限判定に使う）。"""
        return self._fast_active

    @property
    def fast_pid(self) -> PIDController:
        """内包する速い補正層 PID。プランなし（ブートストラップ）経路が直結で使う。"""
        return self._fast

    def update(
        self,
        ref: float,
        actual: float,
        dt: float,
        *,
        phase: PlanPhase = PlanPhase.DRIVE,
        saturated_high: bool = False,
        saturated_low: bool = False,
        gain_scale: float = 1.0,
    ) -> float:
        """トリム effort [%] を返す（+加速/−制動）。

        Args:
            ref: PID 基準速度 [km/h]（pid_preview_s 前倒し済みを呼び出し元が渡す）。
            actual: 実車速 [km/h]。
            dt: 前サイクルからの経過時間 [s]。
            phase: プランの現フェーズ。STOP_HOLD では低速トリムを無効化しプランの停車保持に
                委ねる（速い補正層は安全網として残す）。
            saturated_high/low: 調停・フェーズ権限で加速側/制動側が飽和したか（積分停止）。
            gain_scale: 速い補正層に渡すゲインスケジュール正規化係数。
        """
        error = ref - actual
        abs_e = abs(error)

        # 層ヒステリシス: FAST_ENGAGE で入り、FAST_RELEASE を切るまで維持。
        if abs_e >= FAST_ENGAGE_KMH:
            self._fast_active = True
        elif abs_e < FAST_RELEASE_KMH:
            self._fast_active = False

        if self._fast_active:
            out = self._fast.update(
                ref,
                actual,
                dt,
                saturated_high=saturated_high,
                saturated_low=saturated_low,
                gain_scale=gain_scale,
            )
            # バンプレス: 低速層へ戻るときのため内部状態を速い層の出力に揃えておく。
            self._raw = out
            self._output = out
            self._prev_error = error
            return out

        # 凍結帯: ペダルを動かさない（積分＝内部状態も凍結）。再入時の微分キックを避けるため
        # prev_error だけ更新する。STOP_HOLD もプランの停車保持へ委ねるので凍結扱い。
        if abs_e <= HOLD_BAND_KMH or phase == PlanPhase.STOP_HOLD:
            self._prev_error = error
            return self._output

        # 低速トリム（速度形 PI）: Δoutput を積み、レート制限・量子化する。
        dt_eff = dt if dt > 0.0 else 0.0
        delta = TRIM_SLOW_KP * (error - self._prev_error) + TRIM_SLOW_KI * error * dt_eff
        self._prev_error = error
        max_step = TRIM_RATE_PCT_S * dt_eff
        if max_step > 0.0:
            delta = max(-max_step, min(max_step, delta))
        # アンチワインドアップ: 飽和方向へさらに押す変化は積まない。
        pushing = (delta > 0.0 and saturated_high) or (delta < 0.0 and saturated_low)
        if not pushing:
            self._raw += delta
        # 量子化: 内部連続値と確定出力の差が 1 ステップ以上開いたら確定値を更新する
        # （積分作用は内部連続値 _raw で保持されるため量子化で失われない）。
        if abs(self._raw - self._output) >= TRIM_STEP_PCT:
            self._output = self._raw
        return self._output


__all__ = [
    "FAST_ENGAGE_KMH",
    "FAST_RELEASE_KMH",
    "HOLD_BAND_KMH",
    "TRIM_RATE_PCT_S",
    "TRIM_SLOW_KI",
    "TRIM_SLOW_KP",
    "TRIM_STEP_PCT",
    "TrimController",
]
