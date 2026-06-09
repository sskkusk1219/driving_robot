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
