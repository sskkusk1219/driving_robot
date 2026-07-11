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
