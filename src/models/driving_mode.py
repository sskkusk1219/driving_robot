from dataclasses import dataclass
from datetime import datetime


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
