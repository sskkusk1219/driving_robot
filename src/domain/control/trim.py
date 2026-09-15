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
from src.models.profile import FeedforwardParams, robust_kp_at

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
TRIM_SLOW_KP: float = 2.0  # 低速トリム比例ゲイン [%/(km/h)]（実効ゲイン未指定時のみ使う）
TRIM_SLOW_KI: float = 1.2  # 低速トリム積分ゲイン [%/(km/h·s)]（同上）
TRIM_RATE_PCT_S: float = 3.0  # 低速トリム出力のスルーレート上限 [%/s]
TRIM_STEP_PCT: float = 0.25  # 低速トリム出力の量子化ステップ [%]（サーボ微振動抑制）
# 低速トリムの比例ゲインを「速い補正層の実効 kp」に対する比として決める（2026-09-09 追加）。
#
# **層の権限逆転を構造的に禁じるための仕組み。** 旧実装は低速層 KP=2.0 固定に対し速い層の
# 実効 kp が 0.80 しかなく、偏差が FAST_ENGAGE(0.5km/h) を超えた瞬間に比例ゲインが 2.5 倍
# **下がって**いた。実機 9eee549b の実測でも誤差帯 0.5-1.0km/h で実効 kp 0.953 に対し
# 3.0-10.0km/h では 0.484 と、誤差が大きいほど戻せなくなっていた。
# 比 1.0 なら層をまたいでも比例ゲインが連続し、速い層より強くなることが原理的に無い。
# 小偏差帯での穏やかさはゲインではなくスルーレート制限 3%/s と量子化 0.25% が担う。
TRIM_SLOW_KP_RATIO: float = 1.0
# 低速トリムの積分時間 [s]。KI = KP / TRIM_SLOW_TI_S。旧固定値の比 2.0/1.2 を踏襲する。
TRIM_SLOW_TI_S: float = TRIM_SLOW_KP / TRIM_SLOW_KI

# ── 速い補正層のむだ時間安定ゲインキャップ ─────────────────────────────────
# 速い補正層はプロファイルの PID ゲインをそのまま使うが、そのゲインは座標降下が
# tuning_cost 最小化で選んだ値で、プラントのむだ時間 θ に対して過大になりうる。実機
# sample_003 では高速巡航で閉ループ限界サイクル（偏差±0.5〜1.0km/h ⇄ effort±4〜8%、
# 周期≈0.35Hz、アクセル⇔ブレーキ踏み替え多発）を生んだ。SIMC の整定則で実効比例ゲインの
# 安定上限を与え、これを超える分だけ gain_scale を絞る。
#
# **2026-09-09 変更: 自己制御系 FOPDT → 積分系 SIMC。**
# 旧実装は kp_robust = τ/(k·θ·(1+τc係数)) を使っていたが、この k（= 車速上昇量 ÷ 保持開度）
# は学習運転がプラトーに達しないと保持区間の長さを測るだけの値になる（実機 9eee549b:
# 同一車で 15 倍のばらつき）。結果 kp_robust=0.80 %/(km/h) となり、誤差 4.70km/h に対して
# トリムが最大 2.79% しか出せなかった。本系のプラントは「開度→加速度」が静的で車速はその
# 積分＝積分系＋むだ時間なので、同定済みペダルゲイン k'(v) から
# Kc = 1/(k'(v)·(θ+τc)) を速度・向きごとに求める（models.profile.robust_kp_at）。
SIMC_TAU_C_FACTOR: float = 1.5  # 閉ループ時定数 τc = θ×この係数
# （1.0→1.5: τc=θ だと 75km/h 駆動側で Kc=3.38 まで上がり、むだ時間入り閉ループ模擬の
# 発散点 8.0 に対する余裕が薄い。1.5 で全域 0.83〜3.38 に収まり、模擬の最適点 2.4〜3.2 と
# 整合する。pid_tuning.SIMC_INTEGRATING_TAU_C_FACTOR と同値に保つこと）

# ── フェーズ内符号反転時の積分ブリード（F3: イントラフェーズ・ワインドアップ対策） ──────
# 同一フェーズ内で持続偏差（プラン effort 不足等）が続くと速い層の積分・低速トリム出力が
# 大きく溜まる。ref が平坦化する等で偏差の符号が反転すると、この蓄積が逆方向へ数秒残留し
# オーバーシュートを長引かせる（実機 2026-07-14 t=119-139s: 8 秒の -3.8km/h 偏差の後、
# 反転後も +4.4km/h が持続）。notify_phase_change の全クリアはフェーズ境界専用のため、
# フェーズ内の符号反転はこの部分ブリードで扱う。FOPDT 閉ループシミュレーション
# （k=2.14/τ=2.88/θ=0.30・kp=1.30/ki=0.38、7/14 同定値。8 秒の持続偏差→符号反転の再現
# シナリオ）で反転後 |偏差|≤0.5km/h への整定時間を比較し、factor=1.0（ブリードなし）12.0s
# → 0.15 で 2.6s に短縮すると確認（2026-07-14、比較表は tasklist T3 参照）。factor=0 の
# 全クリアは整定 3.8s とかえって悪化する（積分を急に失う段差でトリムが一時的に過小補正になり
# 二次的な振動を生む）ため、緩やかな残留を残す 0.15 を採用。
ZERO_CROSS_BLEED_FACTOR: float = 0.15
# ブリードを起動する誤差の最小振幅 [km/h]。ゼロ近傍のノイズ的な符号反転で不要にブリード
# しないよう、凍結帯 HOLD_BAND_KMH と同じ閾値を使う。
ZERO_CROSS_MIN_ERROR_KMH: float = HOLD_BAND_KMH


def fast_gain_scale_cap(
    params: FeedforwardParams,
    v_kmh: float,
    theta_s: float | None,
    kp: float,
    *,
    is_accel: bool,
    tau_c_factor: float = SIMC_TAU_C_FACTOR,
) -> float:
    """速い補正層に許す gain_scale の上限を返す（実 gain_scale との小さい方を使う）。

    実効比例ゲイン kp×gain_scale が積分系 SIMC のロバスト安定ゲイン
    Kc = 1/(k'(v)·(θ+τc)) を超えないよう、上限 = Kc/kp を返す。ペダルゲイン曲線 k'(v) が
    未同定、θ が未同定/非正、または kp≤0 のときは制限なし（+inf）＝従来動作。

    速度と向き（駆動/制動）ごとに変わるため、**毎サイクル評価すること**。旧実装は
    FOPDT から起動時に 1 度だけ算出した定数だった。
    """
    if kp <= 0.0:
        return float("inf")
    robust = robust_kp_at(
        params, v_kmh, theta_s, is_accel=is_accel, tau_c_factor=tau_c_factor
    )
    if robust is None:
        return float("inf")
    return robust / kp


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

    def notify_phase_change(self) -> None:
        """プランのフェーズ切替を通知する。補正の持ち越し（積分・トリム出力）をクリアする。

        前フェーズで蓄積した補正は「そのフェーズのプラン誤差の補償」であり、フェーズが変わると
        プラン effort も不連続に変わるため、持ち越すと切替頭で踏み抜く（実機 sample_004: BRAKE
        中の +10% 補償が DRIVE 切替頭で applied 36% → +9.4km/h オーバーシュート）。速い層は
        積分のみクリア（prev_error・微分フィルタは保持＝微分キック回避）、低速トリムは出力を
        0 から再構築する（レート 3%/s で滑らかに立ち上がるためバンプは小さい）。
        """
        self._fast.reset_integral()
        self._raw = 0.0
        self._output = 0.0

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
        fast_kp_effective: float | None = None,
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
            fast_kp_effective: 速い補正層の実効比例ゲイン kp×gain_scale [%/(km/h)]。
                低速トリムの比例ゲインをこれに追随させ、層をまたいだ権限逆転を防ぐ
                （TRIM_SLOW_KP_RATIO 参照）。None なら TRIM_SLOW_KP/KI 定数を使う（従来動作）。
        """
        error = ref - actual
        abs_e = abs(error)

        # ゼロクロス・ブリード: 持続偏差で溜まった補正が符号反転後も残留してオーバーシュートを
        # 長引かせるのを防ぐ（F3）。ノイズ的な反転で誤発火しないよう、直前誤差が
        # ZERO_CROSS_MIN_ERROR_KMH 以上あった場合のみ発火する。
        if (
            abs(self._prev_error) >= ZERO_CROSS_MIN_ERROR_KMH
            and (error > 0.0) != (self._prev_error > 0.0)
        ):
            self._fast.bleed_integral(ZERO_CROSS_BLEED_FACTOR)
            self._raw *= ZERO_CROSS_BLEED_FACTOR
            self._output *= ZERO_CROSS_BLEED_FACTOR

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
        if fast_kp_effective is not None and fast_kp_effective > 0.0:
            slow_kp = TRIM_SLOW_KP_RATIO * fast_kp_effective
            slow_ki = slow_kp / TRIM_SLOW_TI_S if TRIM_SLOW_TI_S > 0.0 else 0.0
        else:
            slow_kp, slow_ki = TRIM_SLOW_KP, TRIM_SLOW_KI
        delta = slow_kp * (error - self._prev_error) + slow_ki * error * dt_eff
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
    "SIMC_TAU_C_FACTOR",
    "TRIM_RATE_PCT_S",
    "TRIM_SLOW_KI",
    "TRIM_SLOW_KP",
    "TRIM_SLOW_KP_RATIO",
    "TRIM_SLOW_TI_S",
    "TRIM_STEP_PCT",
    "ZERO_CROSS_BLEED_FACTOR",
    "ZERO_CROSS_MIN_ERROR_KMH",
    "TrimController",
    "fast_gain_scale_cap",
]
