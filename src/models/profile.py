from dataclasses import dataclass, field
from datetime import datetime

from .calibration import CalibrationData


@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float


@dataclass
class StopConfig:
    deviation_threshold_kmh: float
    deviation_duration_s: float


@dataclass
class FeedforwardParams:
    """Ridge 逆FFモデルが推論できない領域を補う車両固有の物理定数。

    停車保持・クリープ・惰行・ペダル不感帯（遊び）は滑らかな線形モデルでは
    表現できないため、これらを定数で補完する。デフォルトは AT 車の標準的な値。
    """

    creep_speed_kmh: float = 7.0  # クリープ車速（AT 車典型 5〜10 km/h）
    creep_rate_kmhs: float = 0.5  # クリープ加速率
    engine_brake_decel_kmhs: float = 1.0  # ペダル未操作時のエンジンブレーキ減速量
    # ── 惰行減速カーブ（速度依存） ─────────────────────────────────────────
    # 学習運転のコーストダウンから同定する「速度 v での惰行減速量（正値）[km/h/s]」の折れ線。
    # 実車の惰行減速は速度依存（低速ほどエンジンブレーキが効く車もある）で、単一定数
    # engine_brake_decel_kmhs の近似ではフェーズ分類・FF レジーム合成が減速区間で
    # 真逆の判定（BRAKE↔DRIVE）になりうる（sample_004 実機 p95=4.05 の主因）。
    # 空タプル＝未同定で engine_brake_decel_kmhs 定数へフォールバック（coast_decel_at 参照）。
    coast_decel_speeds_kmh: tuple[float, ...] = ()  # 速度グリッド [km/h]（昇順）
    coast_decel_kmhs: tuple[float, ...] = ()  # 各速度での惰行減速量（正値）[km/h/s]
    # ── ペダルゲイン曲線（速度依存） ───────────────────────────────────────
    # 学習運転から同定する「開度 1% あたり惰行からどれだけ加速度が変わるか [km/h/s per %]」。
    # 逆FFモデルは学習運転の開度分布（10% 以上に集中。0.5-10% は速度域あたり 0.7-6.2 秒しか
    # 無い）に強く依存し、低開度域は外挿になる。ところが WLTP は減速時間の 75% が「惰行より
    # 緩い減速」＝アクセルを少し踏みながら減速する領域で、まさにこの外挿域を使う
    # （実機 3ca20d43: 該当区間で最大 -3.9km/h の追従誤差）。
    # Δa = a_req − a_coast(v) を effort へ換算するこのゲインは Δa=0 で effort=0 が構造的に
    # 保証されるため、低開度域が「外挿」ではなく「原点との内挿」になる（pedal_plan.
    # analytic_efforts 参照）。空タプル＝未同定で従来どおりモデル出力のみを使う。
    pedal_gain_speeds_kmh: tuple[float, ...] = ()  # 速度グリッド [km/h]（昇順）
    accel_gain_kmhs_per_pct: tuple[float, ...] = ()  # アクセル側ゲイン（正値）
    brake_gain_kmhs_per_pct: tuple[float, ...] = ()  # ブレーキ側ゲイン（正値）
    stop_brake_opening_pct: float = 20.0  # 停車保持に要するブレーキ開度
    brake_deadband_pct: float = 1.0  # これ未満では制動力が出ないブレーキ遊び
    accel_deadband_pct: float = 1.0  # これ未満では駆動力が出ないアクセル遊び
    # ── ペダル調停（PedalArbiter）定数 ──────────────────────────────────
    # 振動抑制 KPI（偏差符号反転 ≤1 回/5 秒）をゲイン調整でなく機構で支えるための定数群。
    switch_hysteresis_pct: float = 0.5  # ペダル切替ヒステリシス半幅（努力量 ±この幅は惰行）
    accel_reengage_dwell_s: float = (
        0.3  # ブレーキ解放後のアクセル再踏込ディレイ（制動側は遅延なし）
    )
    accel_rate_limit_pct_s: float = 200.0  # アクセル開度レートリミット
    brake_rate_limit_pct_s: float = 300.0  # ブレーキ開度レートリミット
    pid_output_limit_pct: float = 50.0  # PID 出力権限上限（FF の補助に留める）
    # coast（惰行）遷移でアクセルを 0 へ戻す解放レート [%/s]（B-7-3）。定常巡航で努力量が
    # ヒステリシス帯を跨ぐたびにアクセルが即 0 解放され ON-OFF パルス化する（実機で高速域
    # 36回/min のハンチングを観測）のを防ぐ。ブレーキ要求（effort<−h）による解放は減速権限を
    # 遅延させないため対象外（即解放を維持）。
    accel_release_rate_pct_s: float = 10.0
    # アクセル開度の量子化しきい値 [%]（B-7-3）。要求開度の変化がこの幅未満なら前回開度を保持し、
    # 微小変動によるサーボの震えと ON-OFF チャタを抑える。
    accel_min_step_pct: float = 0.2


def coast_decel_at(params: FeedforwardParams, v_kmh: float) -> float:
    """速度 v での惰行減速量（正値）[km/h/s] を返す。

    惰行減速カーブ（coast_decel_speeds_kmh / coast_decel_kmhs）が同定済み（2点以上）なら
    線形補間（範囲外は端点クランプ）、未同定なら engine_brake_decel_kmhs 定数（従来動作）。
    フェーズ分類（pedal_plan.coast_accel）と FF レジーム合成（feedforward.predict_effort の
    スロットルテーパ判定）の両方が単一ソースとしてこの関数を使う。numpy 非依存
    （models 層は軽量に保つ）。
    """
    decel = _interp_curve(params.coast_decel_speeds_kmh, params.coast_decel_kmhs, v_kmh)
    return decel if decel is not None else params.engine_brake_decel_kmhs


def _interp_curve(
    speeds: tuple[float, ...], values: tuple[float, ...], v_kmh: float
) -> float | None:
    """速度グリッド上の折れ線を線形補間する（範囲外は端点クランプ）。

    有効点が 2 未満なら None（＝未同定）。coast_decel_at / pedal_gain_at の共通実装。
    numpy 非依存（models 層は軽量に保つ）。
    """
    n = min(len(speeds), len(values))
    if n < 2:
        return None
    if v_kmh <= speeds[0]:
        return values[0]
    if v_kmh >= speeds[n - 1]:
        return values[n - 1]
    for i in range(1, n):
        if v_kmh <= speeds[i]:
            span = speeds[i] - speeds[i - 1]
            if span <= 0.0:
                return values[i]
            frac = (v_kmh - speeds[i - 1]) / span
            return values[i - 1] + frac * (values[i] - values[i - 1])
    return values[n - 1]  # pragma: no cover - 端点クランプで到達しない


def pedal_gain_at(params: FeedforwardParams, v_kmh: float, *, is_accel: bool) -> float | None:
    """速度 v でのペダルゲイン [km/h/s per %]（正値）を返す。未同定なら None。

    「開度 1% あたり、惰行状態からどれだけ加速度が変わるか」。アクセル側は駆動方向、
    ブレーキ側は制動方向の大きさで、どちらも正値で持つ。同定は model_training.
    _estimate_pedal_gain_curve、利用は pedal_plan.analytic_efforts。
    """
    values = params.accel_gain_kmhs_per_pct if is_accel else params.brake_gain_kmhs_per_pct
    gain = _interp_curve(params.pedal_gain_speeds_kmh, values, v_kmh)
    if gain is None or gain <= 0.0:
        return None
    return gain


# 速い補正層 PID の公称ゲインを算出する代表速度 [km/h]。速度依存はランタイムの
# gain_scale（drive_loop._gain_scale × trim.fast_gain_scale_cap）が担うため、公称点は
# 1 つで足りる。pid_tuning.compute_pid_gains_simc と drive_loop の g_nominal が
# **同じ点**を使わないと、ゲイン上限が常に張り付くか常に緩むかのどちらかになる。
SIMC_NOMINAL_SPEED_KMH: float = 60.0


def robust_kp_at(
    params: FeedforwardParams,
    v_kmh: float,
    theta_s: float | None,
    *,
    is_accel: bool,
    tau_c_factor: float = 1.0,
) -> float | None:
    """積分系＋むだ時間プラントの SIMC ロバスト比例ゲイン [%/(km/h)] を返す。

    本系のプラントは「ペダル開度 → 加速度」が静的（＝同定済みペダルゲイン k'(v)）で、
    車速はその積分である。したがって開度→車速は **積分系＋むだ時間** G(s)=k'·e^(−θs)/s であり、
    自己制御系 FOPDT ではない。SIMC(Skogestad) の積分系整定則:

        Kc = 1 / (k'(v) · (θ + τc)),   τc = tau_c_factor · θ

    **なぜ FOPDT を使わないか**（2026-09-09 実機 9eee549b で確定）:
    旧実装は pid_tuning.identify_fopdt の定常ゲイン k = 車速上昇量 ÷ 保持開度 を使っていたが、
    この式は「アクセル一定保持で車速がプラトーに達する」ことを前提にしている。実車は
    学習運転のアクセル保持 17 区間すべてでプラトーに達しておらず（区間末の加速度 +0.38〜
    +54.96 km/h/s）、k は定常ゲインではなく **保持区間の長さ** を測っていた（同一車で
    2.3-3.1s の区間 k=0.28-0.46 に対し 18.6-19.9s の区間 k=3.51-4.30 と 15 倍の開き）。
    その k から出る kp_robust=τ/(k·θ·2)=0.80 %/(km/h) がフィードバック権限を 3〜4 倍
    過小に抑え、誤差 4.70km/h に対しトリムが最大 2.79% しか出せない状態を作っていた。
    120km/h 上限の本システムでは真のプラトー（終端速度）に達する学習運転は原理的に
    組めないため、FOPDT を同定し直すのではなくモデル構造を積分系へ替える。

    θ（むだ時間）は identify_fopdt が「区間開始→車速が 0.3km/h 上昇するまで」で測っており、
    プラトー到達を前提としないため引き続き有効。

    Returns:
        ロバスト比例ゲイン。ペダルゲイン未同定・θ 未同定/非正なら None（＝制限なし＝従来動作）。
    """
    if theta_s is None or theta_s <= 0.0:
        return None
    gain = pedal_gain_at(params, v_kmh, is_accel=is_accel)
    if gain is None or gain <= 0.0:
        return None
    tau_c = max(0.0, tau_c_factor) * theta_s
    denom = gain * (theta_s + tau_c)
    if denom <= 0.0:
        return None
    return 1.0 / denom


@dataclass
class DynamicsParams:
    """FOPDT同定と適合走行で得た動特性パラメータ(学習成果メタデータ)。

    pid_preview_s は PID フィードバックのみに適用する基準軌跡の時間シフト量[s]。
    PID が参照する基準速度サンプリングをこの秒数だけ前倒しし、フィードバックループ側の
    むだ時間を補償する。FF は now-frame（前倒しなし）で動き、先読みはモデル自身の
    horizons 特徴量が担う（FF への前倒しは二重補償となり系統偏差を生むため行わない）。
    0.0 で前倒しなし。

    注: 旧フィールド preview_time_s（FF+PID 両方を前倒し）は本フィールドへ改名された。
    既存 DB の JSONB に残る preview_time_s キーは profile_repository のロード時に
    dataclass フィールド外として無視され、pid_preview_s は既定 0.0 に補完される
    （暗黙リセット。むだ時間 θ は fopdt_theta に別途保存済み）。
    """

    pid_preview_s: float = 0.0
    fopdt_k: float | None = None
    fopdt_tau: float | None = None
    fopdt_theta: float | None = None


@dataclass
class VehicleProfile:
    id: str
    name: str
    max_accel_opening: float
    max_brake_opening: float
    max_speed: float
    max_decel_g: float
    pid_gains: PIDGains
    stop_config: StopConfig
    calibration: CalibrationData | None
    model_path: str | None
    created_at: datetime
    updated_at: datetime
    feedforward_params: FeedforwardParams = field(default_factory=FeedforwardParams)
    dynamics_params: DynamicsParams = field(default_factory=DynamicsParams)
