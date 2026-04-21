from dataclasses import dataclass
from datetime import datetime


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


@dataclass
class ValidationResult:
    is_valid: bool
    error_message: str | None


@dataclass
class CalibrationResult:
    success: bool
    data: CalibrationData | None
    error_message: str | None
