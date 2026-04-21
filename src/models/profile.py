from dataclasses import dataclass
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
