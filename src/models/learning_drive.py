from dataclasses import dataclass
from datetime import datetime


@dataclass
class LearningPattern:
    speed_kmh: float
    accel_kmhs: float
    accel_opening: float
    brake_opening: float
    hold_duration_s: float


@dataclass
class LearningLog:
    pattern: LearningPattern
    actual_speed_kmh: float
    accel_opening_applied: float
    brake_opening_applied: float
    recorded_at: datetime
