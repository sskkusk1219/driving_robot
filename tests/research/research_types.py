"""研究環境の型・定数・小さな純関数（本番 `src/` から持ち込んだもの）。

`tests/` は `tests/` だけで完結する（`src/` を丸ごと消しても動く。ProblemReport_20260924）ため、
研究ハーネスが使う本番の型と定数をここへ写した。**ロジックは本番と同じ**（変えない）。使わない部分
（DB 用の DriveSession・LearningCycle、SIMC の robust_kp_at など）は持ち込んでいない。

移植元（2026-09-24 時点）:
    src/models/profile.py        … FeedforwardParams ほか車両の型・補間関数
    src/models/calibration.py    … CalibrationData
    src/models/drive_log.py      … DriveLog・DriveLogData
    src/models/driving_mode.py   … DrivingMode・SpeedPoint
    src/models/learning_drive.py … LearningPattern・PatternKind
    src/models/pre_check.py      … PreCheckResult・PreCheckItemResult
    src/domain/control/conversions.py / pedal_safety.py … 換算・定数・同時踏み禁止ガード
    src/utils/time.py            … to_jst_naive
    src/app/robot_controller.py  … ACTUATOR_PULSE_MAX（定数1個）
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from enum import StrEnum
from zoneinfo import ZoneInfo

# 開度 100% のアクチュエータ位置 [pulse]（ハードウェア機械端。src/app/robot_controller.py の
# _ACTUATOR_PULSE_MAX と同値）
ACTUATOR_PULSE_MAX: int = 9500

# ── conversions（src/domain/control/conversions.py） ────────────────────

# これ未満/以下を「停車」とみなす車速しきい値 [km/h]。
# 従来 robot_controller.py・learning_loop.py・pre_check.py・model_training.py で
# それぞれ独立に 0.5 と定義され、コメントで「同一であること」を頼りに揃えていた。
VEHICLE_STOP_SPEED_KMH: float = 0.02

# 1G を km/h/s へ換算する係数（9.81 m/s^2 * 3.6 km/h/(m/s)）。
# 従来 learning_loop.py・robot_controller.py・pid_tuning.py でそれぞれ独立定義されていた。
G_TO_KMHS: float = 9.81 * 3.6


def opening_to_position(opening_pct: float, zero_pos: int, full_pos: int) -> int:
    """開度 [%] をアクチュエータ位置 [pulse] に変換する。"""
    return zero_pos + round((full_pos - zero_pos) * opening_pct / 100.0)


def position_to_opening(pos_pulse: int, zero_pos: int, full_pos: int) -> float:
    """アクチュエータ位置 [pulse] を開度 [%] に変換する（`opening_to_position` の逆）。

    2026-09-28 段7d: G 校正の上限開度表で「実際に効いていた開度」を記録するのに使う
    （指令開度のままだと、指令が実アクチュエータより速く動く低速の踏み込みで開度を高く見誤る）。
    """
    span = full_pos - zero_pos
    if span == 0:
        return 0.0
    return (pos_pulse - zero_pos) * 100.0 / span


def clamp_opening(opening_pct: float, max_opening_pct: float) -> float:
    """開度 [%] を [0, max_opening_pct] にクランプする。"""
    return max(0.0, min(opening_pct, max_opening_pct))


def enforce_pedal_exclusion(accel_opening: float, brake_opening: float) -> tuple[float, float]:
    """アクセル・ブレーキが同時に非ゼロなら小さい方を 0 にして排他を強制する。

    同値のときはブレーキを優先（安全側）に残す。通常運用では片方が常に 0 のため no-op。
    """
    if accel_opening > 0.0 and brake_opening > 0.0:
        if brake_opening >= accel_opening:
            return 0.0, brake_opening
        return accel_opening, 0.0
    return accel_opening, brake_opening


# ── utils.time（src/utils/time.py） ──────────────────────────────

JST = ZoneInfo("Asia/Tokyo")
_UTC = ZoneInfo("UTC")


def to_jst_naive(dt: datetime) -> datetime:
    """aware/naive datetime を JST に変換し、tzinfo を外した naive datetime を返す。

    naive（tzinfo なし）が渡された場合は UTC とみなす（DB の TIMESTAMPTZ は
    asyncpg から UTC aware で返るが、防御的に扱う）。
    """
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=_UTC)
    return dt.astimezone(JST).replace(tzinfo=None)


# ── models.calibration ─────────────────────────────────────────

@dataclass
class CalibrationData:
    accel_zero_pos: int
    accel_full_pos: int
    accel_stroke: int
    brake_zero_pos: int
    brake_full_pos: int
    brake_stroke: int
    calibrated_at: datetime
    is_valid: bool


# ── models.profile ─────────────────────────────────────────────

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


# ── models.drive_log ───────────────────────────────────────────

@dataclass
class DriveLog:
    id: int
    session_id: str
    timestamp: datetime
    ref_speed_kmh: float | None
    actual_speed_kmh: float
    accel_opening: float
    brake_opening: float
    accel_pos: int
    brake_pos: int
    accel_current: float
    brake_current: float
    # effort 内訳（エピソード型プラン学習）。自動走行のみ非 None。学習運転・スケジュール走行・
    # 旧セッションは None（後方互換）。applied はフェーズ権限クランプ後・調停器前の合成値。
    plan_effort_pct: float | None = None
    trim_effort_pct: float | None = None
    applied_effort_pct: float | None = None
    phase: str | None = None


@dataclass
class DriveLogData:
    """LogWriter が 100ms 周期で DB に書き込む転送オブジェクト。id・timestamp は DB 側で生成。"""

    ref_speed_kmh: float | None
    actual_speed_kmh: float
    accel_opening: float
    brake_opening: float
    accel_pos: int
    brake_pos: int
    accel_current: float
    brake_current: float
    # effort 内訳（プラン学習・トリム寄与率の可観測化）。デフォルト None で学習運転・
    # スケジュール走行は無変更（それらの LogWriter 呼び出しは既存のまま通る）。
    plan_effort_pct: float | None = None
    trim_effort_pct: float | None = None
    applied_effort_pct: float | None = None
    phase: str | None = None


# ── models.driving_mode ────────────────────────────────────────

@dataclass
class SpeedPoint:
    time_s: float
    speed_kmh: float


@dataclass
class DrivingMode:
    id: str
    name: str
    description: str
    reference_speed: list[SpeedPoint]
    total_duration: float
    max_speed: float
    created_at: datetime
    # 学習サイクルが内部生成する網羅検証パターン（プラン学習の保存先）は is_system=True。
    # ユーザー向けモード一覧から除外し、WebUI 編集・削除も拒否する。既存呼び出しは False 既定。
    is_system: bool = False


# ── models.learning_drive ──────────────────────────────────────

class PatternKind(StrEnum):
    """開度パターンの種別。LearningLoop が実行時の前進条件を決めるのに使う。"""

    CREEP = "creep"  # 停車保持から段階的にブレーキを緩める（解放ステップ）
    CREEP_SETTLE = "creep_settle"  # アクセル・ブレーキ 0% で車速が安定するまで待機しクリープ計測
    # アクセルで加速→ブレーキ無しで低速まで惰行（エンジンブレーキ減速率を計測）
    COAST_DOWN = "coast_down"
    # 格子ステップ走行（ProblemReport_20260925 段3。研究側の追加）: WLTP の車速×加速度の格子を
    # 狙い、開度固定のステップで測る。GRID_STEP は車速ステーション 1 つ、GRID_LAUNCH は
    # 0〜20km/h の発進・停車セル。CSV の pattern 列で見分けるための種別
    GRID_STEP = "grid_step"
    GRID_LAUNCH = "grid_launch"
    # G 校正（段6a・門①。研究側の追加）: G 比例加速で cap まで上げ、0.2G 狙いの G 比例ブレーキ減速で
    # 低速まで下りる。各車速で「0.2G を出すブレーキ開度」を測り、上限 G に届く開度の予測に使う
    G_CALIB = "g_calib"
    # 通し掃引（段6c。研究側の追加）: 格子ステップで埋まらなかった強い加減速のセルを、
    # 車速のセルごとに開度を切り替えながら 1 回で複数の車速帯を通り抜けて測る
    # （穴が無ければ何もしない）
    GRID_SWEEP = "grid_sweep"


@dataclass(frozen=True)
class LearningPattern:
    """学習運転で開ループ実行する1つの固定開度パターン。

    開度 [%] はアクチュエータ位置への換算前の論理値。`hold_duration_s` は
    速度プラトーや上限に達しない場合に当該パターンを打ち切る最大保持時間。
    COAST_DOWN は accel_opening（cap まで上げる加速）を使う。格子ステップ走行・クリープ発進は
    固定開度を使わない（クリープ域ブレーキ保持だけ brake_opening）。旧 ACCEL_SWEEP・BRAKE_HOLD・
    CRUISE_TRIM は 2026-09-25 段4 で削除した。`trim_opening` は旧 CRUISE_TRIM の名残で未使用。
    """

    kind: PatternKind
    accel_opening: float
    brake_opening: float
    hold_duration_s: float
    # CRUISE_TRIM 専用: cap 到達後に保持する微小アクセル開度 [%]（他パターンでは未使用で 0.0）。
    trim_opening: float = 0.0


# ── models.pre_check ───────────────────────────────────────────

@dataclass
class PreCheckItemResult:
    item_name: str
    passed: bool
    error_message: str | None = None


@dataclass
class PreCheckResult:
    passed: bool
    items: list[PreCheckItemResult] = field(default_factory=list)

    @property
    def failed_items(self) -> list[PreCheckItemResult]:
        return [i for i in self.items if not i.passed]
