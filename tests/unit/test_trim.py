"""トリム制御（TrimController）のユニットテスト。

3 層（凍結／低速トリム／速い補正層）の遷移・レート制限・量子化・ヒステリシス・
バンプレス切替・アンチワインドアップ・reset・FOPDT シミュレーションでの収束を検証する。
"""

import numpy as np
import pytest

import src.domain.control.trim as trim_mod
from src.domain.control.pedal_plan import PlanPhase
from src.domain.control.pid import PIDController
from src.domain.control.trim import (
    FAST_ENGAGE_KMH,
    SIMC_TAU_C_FACTOR,
    TRIM_RATE_PCT_S,
    TRIM_SLOW_KP,
    TRIM_SLOW_KP_RATIO,
    TRIM_SLOW_TI_S,
    TRIM_STEP_PCT,
    ZERO_CROSS_MIN_ERROR_KMH,
    TrimController,
    fast_gain_scale_cap,
)
from src.models.profile import FeedforwardParams


def _fast_pid() -> PIDController:
    return PIDController(kp=3.9, ki=0.88, kd=0.27, dt=0.05, output_limit=50.0)


# モジュール既定値を import 時点で固定キャプチャ（テスト内で trim_mod.ZERO_CROSS_BLEED_FACTOR
# を書き換えるため、実行順序に依らず「実際の既定値」を参照できるようにする）。
_DEFAULT_ZERO_CROSS_BLEED_FACTOR = trim_mod.ZERO_CROSS_BLEED_FACTOR


def _simulate_fopdt(
    *,
    k: float,
    tau: float,
    theta: float,
    kp: float,
    ki: float,
    kd: float,
    gain_scale: float,
    kick: float,
    quant: float = 0.1,
    dt: float = 0.05,
    dur_s: float = 60.0,
) -> tuple[np.ndarray, np.ndarray]:
    """トリム＋FOPDT プラント（むだ時間 theta）の閉ループを回し (偏差, トリム出力) を返す。

    基準を 0 とし v=実車速−基準の偏差ダイナミクスを積分する。初期偏差 kick[km/h] を与え、
    CAN 量子化 quant[km/h] で計測を丸める。プランは定常保持済みと仮定しトリムのみが v を動かす。
    """
    trim = TrimController(PIDController(kp, ki, kd, dt=dt, output_limit=50.0))
    delay = int(round(theta / dt))
    ubuf = [0.0] * (delay + 1)
    v = kick
    errs: list[float] = []
    us: list[float] = []
    for _ in range(int(dur_s / dt)):
        meas = round(v / quant) * quant if quant > 0 else v
        u = trim.update(0.0, meas, dt, phase=PlanPhase.DRIVE, gain_scale=gain_scale)
        ubuf.append(u)
        u_del = ubuf.pop(0)
        v += (k * u_del - v) / tau * dt
        errs.append(v)
        us.append(u)
    return np.array(errs), np.array(us)


# 実機 sample_003 の同定値。高ゲイン kp=4.5 と大むだ時間 θ=0.80s の組み合わせが
# 高速巡航の閉ループ限界サイクル（ペダル踏み替え多発）の原因（2026-07-13 実機解析）。
_S003 = {"k": 0.9228, "tau": 1.3662, "theta": 0.8000, "kp": 4.4992, "ki": 1.2137, "kd": 0.15}
# _simulate_fopdt のプラント v += (k*u - v)/tau*dt は、ステップ u に対する初期加速度が
# k/tau·u [km/h/s] になる。積分系モデルのペダルゲイン k' はこの初期勾配そのものなので、
# 同じプラントを新 API（積分系 SIMC）で評価するときは k' = k/tau を使う。
_S003_PEDAL_GAIN = _S003["k"] / _S003["tau"]


def _params_with_gain(gain: float) -> FeedforwardParams:
    """全速度で一定のペダルゲインを持つ FeedforwardParams（キャップ算出用）。"""
    return FeedforwardParams(
        pedal_gain_speeds_kmh=(0.0, 200.0),
        accel_gain_kmhs_per_pct=(gain, gain),
        brake_gain_kmhs_per_pct=(gain, gain),
    )


def _cap_s003() -> float:
    """sample_003 プラントに対する新 API のロバスト上限。"""
    return fast_gain_scale_cap(
        _params_with_gain(_S003_PEDAL_GAIN), 100.0, _S003["theta"], _S003["kp"], is_accel=True
    )


def _sign_flips(u: np.ndarray, *, floor: float = 0.05) -> int:
    """トリム出力の符号反転（アクセル⇔ブレーキ踏み替え）回数。floor 未満は中立扱い。"""
    s = np.sign(u)
    s[np.abs(u) < floor] = 0
    return int(np.sum(np.abs(np.diff(s)) > 1))


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


class TestPhaseChangeNotification:
    """フェーズ切替時の補正持ち越しクリア（T8: ワインドアップ事故対策）。"""

    def test_clears_fast_integral_and_slow_output(self) -> None:
        trim = TrimController(_fast_pid())
        # 速い層をアクティブにして積分を蓄積（持続偏差 +1.0km/h）
        for _ in range(60):
            trim.update(61.0, 60.0, 0.05, phase=PlanPhase.BRAKE)
        assert trim.fast_pid._integral > 0.0
        trim.notify_phase_change()
        assert trim.fast_pid._integral == 0.0
        assert trim._raw == 0.0
        assert trim._output == 0.0

    def test_windup_not_carried_across_phase_flip(self) -> None:
        """T8 実機再現: BRAKE 中に溜めた補正がフェーズ切替直後の出力に持ち越されない。

        sample_004 では BRAKE 中の +10% 補償（大半が積分）が DRIVE 切替頭で plan と重なり
        applied 36% → +9.4km/h オーバーシュートを起こした。切替通知後の初回出力は
        積分持ち越しなしの小さい値になる。
        """
        trim = TrimController(_fast_pid())
        # BRAKE フェーズで持続偏差により積分を大きく蓄積
        out_before = 0.0
        for _ in range(200):
            out_before = trim.update(45.0, 44.0, 0.05, phase=PlanPhase.BRAKE)
        assert out_before > 3.0  # 積分主体の大きな補正が立っている

        # フェーズ切替（BRAKE→DRIVE）: 通知後の初回出力は偏差比例分のみ（積分なし）
        trim.notify_phase_change()
        out_after = trim.update(45.0, 44.8, 0.05, phase=PlanPhase.DRIVE)
        assert abs(out_after) < out_before / 2  # 持ち越しが消えている

    def test_no_derivative_kick_after_notification(self) -> None:
        """積分のみクリア（prev_error 保持）のため、切替直後に微分キックが出ない。"""
        trim = TrimController(_fast_pid())
        for _ in range(20):
            trim.update(61.0, 60.0, 0.05, phase=PlanPhase.BRAKE)
        trim.notify_phase_change()
        # 同じ偏差（error 変化なし）→ D 項はほぼゼロ、P 項のみの穏やかな出力
        out = trim.update(61.0, 60.0, 0.05, phase=PlanPhase.DRIVE)
        kp = 3.9
        assert abs(out) < kp * 1.0 * 1.5  # P 項オーダー（キックで跳ねていない）


class TestZeroCrossBleed:
    """フェーズ内の偏差符号反転時の積分ブリード（F3: イントラフェーズ・ワインドアップ対策）。"""

    def test_bleeds_fast_integral_on_sign_flip(self) -> None:
        trim = TrimController(_fast_pid())
        # 持続偏差（正）で速い層の積分を蓄積
        for _ in range(60):
            trim.update(61.0, 60.0, 0.05, phase=PlanPhase.DRIVE)
        integral_before = trim.fast_pid._integral
        assert integral_before > 0.0
        # 符号反転（実測が基準を超える）
        trim.update(60.0, 61.0, 0.05, phase=PlanPhase.DRIVE)
        # ブリード後に今回分の PID 更新（error=-1.0）が乗った値と一致する
        expected = integral_before * trim_mod.ZERO_CROSS_BLEED_FACTOR + (-1.0) * 0.05
        assert trim.fast_pid._integral == pytest.approx(expected, abs=1e-6)

    def test_bleeds_slow_trim_output_on_sign_flip(self) -> None:
        trim = TrimController(_fast_pid())
        # 低速トリム帯（0.1<|e|<0.5）で持続偏差により出力を蓄積
        for _ in range(60):
            trim.update(60.3, 60.0, 0.05, phase=PlanPhase.DRIVE)
        raw_before = trim._raw
        assert raw_before > 0.0
        trim.update(60.0, 60.3, 0.05, phase=PlanPhase.DRIVE)  # 符号反転
        # ブリードで大きく縮小している（今回分の増分を含めても反転前の半分未満）
        assert abs(trim._raw) < raw_before * 0.5

    def test_no_bleed_when_below_hold_band(self) -> None:
        """ZERO_CROSS_MIN_ERROR_KMH は凍結帯 HOLD_BAND_KMH と同値のため、凍結帯内で
        符号が反転しても（元々ペダルを動かさない帯域なので）速い層の積分は変化しない。"""
        trim = TrimController(_fast_pid())
        trim.update(60.0 + ZERO_CROSS_MIN_ERROR_KMH / 2.0, 60.0, 0.05, phase=PlanPhase.DRIVE)
        trim.update(60.0 - ZERO_CROSS_MIN_ERROR_KMH / 2.0, 60.0, 0.05, phase=PlanPhase.DRIVE)
        assert trim.fast_pid._integral == pytest.approx(0.0, abs=1e-9)


def _simulate_ramp_then_flat(
    *,
    bleed_factor: float,
    k: float = 2.1436,
    tau: float = 2.8772,
    theta: float = 0.30,
    kp: float = 1.3036,
    ki: float = 0.3788,
    kd: float = 0.0,
    dt: float = 0.05,
    ramp_kmhs: float = 2.0,
    ramp_dur_s: float = 8.0,
    deficiency_pct: float = 50.0,
    dur_s: float = 30.0,
    v0: float = 80.0,
) -> tuple[np.ndarray, np.ndarray]:
    """実機 2026-07-14 t=119-139s の再現: プラン effort 不足で持続偏差 → ref 平坦化で符号反転。

    ref を ramp_kmhs で ramp_dur_s だけ下げた後平坦化する。その間プランは理想 FF
    （FOPDT の厳密フィードフォワード u=(τ·ref_dot+ref)/k）から deficiency_pct 分だけ不足させ、
    トリムが不足を埋めるため積分が蓄積する。平坦化後は理想プランに戻り、蓄積した補正が
    符号反転後にどれだけ速く抜けるかを (偏差, トリム出力) で返す（FOPDT 閉ループ、実機同定
    k=2.14/τ=2.88/θ=0.30・kp=1.30/ki=0.38 相当）。
    """
    trim_mod.ZERO_CROSS_BLEED_FACTOR = bleed_factor
    trim = TrimController(PIDController(kp, ki, kd, dt=dt, output_limit=50.0))
    delay = int(round(theta / dt))
    ubuf = [0.0] * (delay + 1)
    v = v0
    n = int(dur_s / dt)
    errs: list[float] = []
    trims: list[float] = []
    for i in range(n):
        t = i * dt
        if t < ramp_dur_s:
            ref = v0 - ramp_kmhs * t
            ref_dot = -ramp_kmhs
        else:
            ref = v0 - ramp_kmhs * ramp_dur_s
            ref_dot = 0.0
        plan_ideal = (tau * ref_dot + ref) / k
        plan = plan_ideal * (1.0 - deficiency_pct / 100.0) if t < ramp_dur_s else plan_ideal
        trim_u = trim.update(ref, v, dt, phase=PlanPhase.DRIVE, gain_scale=1.0)
        applied = plan + trim_u
        ubuf.append(applied)
        u_del = ubuf.pop(0)
        v += (k * u_del - v) / tau * dt
        errs.append(v - ref)
        trims.append(trim_u)
    return np.array(errs), np.array(trims)


def _settle_time_s(errs: np.ndarray, flip_idx: int, dt: float, *, band: float = 0.5) -> float:
    """flip_idx 以降で |err|<=band に入ってから最後まで維持される最初の時刻 [s]。"""
    post = np.abs(errs[flip_idx:]) <= band
    for i in range(len(post)):
        if post[i:].all():
            return float(i * dt)
    return float("inf")


class TestIntraPhaseWindupRecovery:
    """WS3 T3/T4: ゼロクロス・ブリードによる符号反転後の残留時間短縮（実機 7/14 再現）。"""

    def test_bleed_shortens_settle_time_after_sign_flip(self) -> None:
        dt = 0.05
        ramp_dur_s = 8.0
        flip_idx = int(ramp_dur_s / dt)
        try:
            errs_no_bleed, _ = _simulate_ramp_then_flat(bleed_factor=1.0, dt=dt)
            errs_bleed, _ = _simulate_ramp_then_flat(
                bleed_factor=_DEFAULT_ZERO_CROSS_BLEED_FACTOR, dt=dt
            )
        finally:
            trim_mod.ZERO_CROSS_BLEED_FACTOR = _DEFAULT_ZERO_CROSS_BLEED_FACTOR
        settle_no_bleed = _settle_time_s(errs_no_bleed, flip_idx, dt)
        settle_bleed = _settle_time_s(errs_bleed, flip_idx, dt)
        # 2026-07-14 実機同定値でのシム比較: ブリード無し 12.0s → 既定値(0.15) 2.6s。
        assert settle_bleed < settle_no_bleed * 0.5
        assert settle_bleed < 4.0

    def test_bleed_does_not_worsen_overshoot_peak(self) -> None:
        dt = 0.05
        ramp_dur_s = 8.0
        flip_idx = int(ramp_dur_s / dt)
        try:
            errs_no_bleed, _ = _simulate_ramp_then_flat(bleed_factor=1.0, dt=dt)
            errs_bleed, _ = _simulate_ramp_then_flat(
                bleed_factor=_DEFAULT_ZERO_CROSS_BLEED_FACTOR, dt=dt
            )
        finally:
            trim_mod.ZERO_CROSS_BLEED_FACTOR = _DEFAULT_ZERO_CROSS_BLEED_FACTOR
        assert errs_bleed[flip_idx:].max() <= errs_no_bleed[flip_idx:].max() + 1e-9


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


class TestFastGainScaleCap:
    """速い補正層のロバストゲインキャップ（fast_gain_scale_cap・積分系 SIMC）。"""

    def test_unidentified_returns_inf(self) -> None:
        """ペダルゲイン未同定・θ 未同定/非正・kp≤0 は制限なし（従来動作）。"""
        p = _params_with_gain(0.5)
        assert fast_gain_scale_cap(FeedforwardParams(), 50.0, 0.3, 4.5, is_accel=True) == float(
            "inf"
        )
        assert fast_gain_scale_cap(p, 50.0, None, 4.5, is_accel=True) == float("inf")
        assert fast_gain_scale_cap(p, 50.0, 0.0, 4.5, is_accel=True) == float("inf")
        assert fast_gain_scale_cap(p, 50.0, 0.3, 0.0, is_accel=True) == float("inf")

    def test_cap_equals_integrating_simc_ratio(self) -> None:
        """cap = Kc/kp、Kc = 1/(k'·(θ+τc))、τc = SIMC_TAU_C_FACTOR·θ。"""
        cap = _cap_s003()
        theta = _S003["theta"]
        kc = 1.0 / (_S003_PEDAL_GAIN * (theta + SIMC_TAU_C_FACTOR * theta))
        assert cap == pytest.approx(kc / _S003["kp"])
        assert cap < 1.0  # sample_003 は過大ゲイン → 絞られる

    def test_robust_profile_not_limited(self) -> None:
        """kp が既に安定値以下なら上限は 1 以上（gain_scale を実質制限しない）。"""
        # k'=0.2, θ=0.2 → Kc = 1/(0.2*0.5) = 10.0 ≫ kp=0.5
        cap = fast_gain_scale_cap(_params_with_gain(0.2), 50.0, 0.2, 0.5, is_accel=True)
        assert cap > 1.0

    def test_larger_dead_time_tightens_cap(self) -> None:
        p = _params_with_gain(0.5)
        c_small = fast_gain_scale_cap(p, 50.0, 0.2, 3.0, is_accel=True)
        c_large = fast_gain_scale_cap(p, 50.0, 0.6, 3.0, is_accel=True)
        assert c_large < c_small  # θ 大ほど強く絞る

    def test_larger_pedal_gain_tightens_cap(self) -> None:
        """プラントゲインが大きい速度域ほど上限は小さくなる（積分系 SIMC の反比例）。"""
        c_low = fast_gain_scale_cap(_params_with_gain(0.25), 50.0, 0.5, 3.0, is_accel=True)
        c_high = fast_gain_scale_cap(_params_with_gain(1.00), 50.0, 0.5, 3.0, is_accel=True)
        assert c_high == pytest.approx(c_low / 4.0)

    def test_cap_varies_with_speed_and_direction(self) -> None:
        """速度・駆動/制動でペダルゲインが違えば上限も変わる（旧実装は単一定数だった）。"""
        params = FeedforwardParams(
            pedal_gain_speeds_kmh=(0.0, 100.0),
            accel_gain_kmhs_per_pct=(0.5, 0.25),
            brake_gain_kmhs_per_pct=(0.25, 1.0),
        )
        at_0 = fast_gain_scale_cap(params, 0.0, 0.5, 1.0, is_accel=True)
        at_100 = fast_gain_scale_cap(params, 100.0, 0.5, 1.0, is_accel=True)
        assert at_100 == pytest.approx(at_0 * 2.0)  # ゲイン半減 → 上限倍増
        brake_100 = fast_gain_scale_cap(params, 100.0, 0.5, 1.0, is_accel=False)
        assert brake_100 < at_100  # 同じ速度でも制動側はゲインが大きく上限が小さい


class TestHighSpeedLimitCycle:
    """高速巡航の閉ループ限界サイクル（実機 sample_003）の再現とキャップによる解消。"""

    def test_current_gain_oscillates(self) -> None:
        """キャップ無し（gain_scale=1.0）では発散的な限界サイクル（踏み替え多発）になる。"""
        errs, us = _simulate_fopdt(**_S003, gain_scale=1.0, kick=1.0)
        tail = slice(len(errs) // 2, len(errs))
        # 偏差が大きく発散し、符号反転（ペダル踏み替え）が多発する
        assert np.abs(errs[tail]).max() > 2.0
        assert _sign_flips(us[tail]) * 2 > 20  # >20 回/min（実機 35-42/min 相当）

    def test_gain_cap_kills_limit_cycle(self) -> None:
        """SIMC キャップ適用で限界サイクルが消失し、偏差が凍結帯へ収束・ペダル静止。"""
        cap = _cap_s003()
        errs, us = _simulate_fopdt(**_S003, gain_scale=min(1.0, cap), kick=1.0)
        tail = slice(len(errs) // 2, len(errs))
        assert np.abs(errs[tail]).max() < 0.25  # KPI p95≤0.2 圏へ収束
        assert _sign_flips(us[tail]) < 3  # 踏み替えほぼ消失

    def test_gain_cap_preserves_max_protection(self) -> None:
        """2km/h の大偏差ステップに対し発散せず収束する（max≤1.0 安全網が機能）。"""
        cap = _cap_s003()
        errs, _ = _simulate_fopdt(**_S003, gain_scale=min(1.0, cap), kick=2.0)
        # 15s 後には十分収束している（発散しない）
        assert abs(errs[int(15.0 / 0.05)]) < 0.3
        assert np.abs(errs[len(errs) // 2 :]).max() < 0.25


class TestLayerAuthorityNoInversion:
    """層をまたいだ権限逆転の回帰テスト（実機 9eee549b）。

    旧実装は低速トリム KP=2.0 固定に対し速い層の実効 kp が 0.80 しかなく、偏差が
    FAST_ENGAGE(0.5km/h) を超えた瞬間に比例ゲインが 2.5 倍**下がって**いた。実測でも
    誤差帯 0.5-1.0km/h で実効 kp 0.953 に対し 3.0-10.0km/h では 0.484 と、誤差が
    大きいほど戻せなくなっていた（max|trim| 2.79% vs max|err| 4.70km/h）。
    """

    @staticmethod
    def _fast_gain(error: float, kp_eff: float) -> float:
        """速い補正層の実効比例ゲイン（ki=0 で P 項のみを見る）。"""
        pid = PIDController(kp=kp_eff, ki=0.0, kd=0.0, dt=0.05, output_limit=50.0)
        trim = TrimController(pid)
        return trim.update(error, 0.0, 0.05, fast_kp_effective=kp_eff) / error

    @staticmethod
    def _slow_output(error: float, kp_eff: float | None, cycles: int = 10) -> float:
        """低速トリム層に一定偏差を与え続けたときの出力（レート制限・量子化込み）。"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0, dt=0.05, output_limit=50.0)
        trim = TrimController(pid)
        out = 0.0
        for _ in range(cycles):
            out = trim.update(error, 0.0, 0.05, fast_kp_effective=kp_eff)
        return out

    def test_fast_layer_gain_flat_across_error_sizes(self) -> None:
        """誤差 1→5km/h で実効ゲインが目減りしない（実測 0.953→0.484 の逆転を禁じる）。"""
        kp_eff = 2.8  # 積分系 SIMC の公称値相当
        gains = [self._fast_gain(e, kp_eff) for e in (1.0, 2.0, 3.0, 5.0)]
        assert all(g == pytest.approx(kp_eff) for g in gains)

    def test_fast_layer_output_grows_with_error(self) -> None:
        kp_eff = 2.8
        outs = [self._fast_gain(e, kp_eff) * e for e in (1.0, 2.0, 3.0, 5.0)]
        assert all(b > a for a, b in zip(outs, outs[1:], strict=False))

    def test_slow_layer_tracks_fast_layer_gain(self) -> None:
        """低速トリム層の出力が速い層の実効ゲインに追随する（固定 2.0 ではない）。"""
        weak = self._slow_output(0.4, 0.8)
        strong = self._slow_output(0.4, 3.4)
        assert strong > weak  # ロバスト値が大きい速度域では低速層も強くなる

    def test_slow_layer_never_exceeds_fast_layer(self) -> None:
        """境界直下（低速層）の応答が境界直上（速い層）を上回らない＝権限逆転しない。

        旧実装は kp_eff=0.8 のとき低速層 KP=2.0 が速い層の 2.5 倍で、偏差が 0.5km/h を
        超えた瞬間に補正が弱くなっていた。両層に同じ PI（低速層が導出するのと同じ積分時間）
        を与え、同じ窓で出力を比べる。低速層はレート制限・量子化があるぶん必ず小さくなる。
        """
        for kp_eff in (0.8, 1.5, 3.4):
            slow = self._slow_output(0.49, kp_eff, cycles=40)
            pid = PIDController(
                kp=kp_eff, ki=kp_eff / TRIM_SLOW_TI_S, kd=0.0, dt=0.05, output_limit=50.0
            )
            trim = TrimController(pid)
            fast = 0.0
            for _ in range(40):
                fast = trim.update(0.51, 0.0, 0.05, fast_kp_effective=kp_eff)
            assert slow <= fast + 1e-9

    def test_falls_back_to_constants_without_effective_gain(self) -> None:
        """fast_kp_effective 未指定なら従来の TRIM_SLOW_KP 定数と同一挙動（後方互換）。"""
        assert self._slow_output(0.4, None) == pytest.approx(
            self._slow_output(0.4, TRIM_SLOW_KP / TRIM_SLOW_KP_RATIO)
        )
