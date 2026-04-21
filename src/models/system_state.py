from dataclasses import dataclass
from datetime import datetime
from enum import StrEnum


class RobotState(StrEnum):
    BOOTING = "BOOTING"
    STANDBY = "STANDBY"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    CALIBRATING = "CALIBRATING"
    PRE_CHECK = "PRE_CHECK"
    RUNNING = "RUNNING"
    MANUAL = "MANUAL"
    EMERGENCY = "EMERGENCY"
    ERROR = "ERROR"


@dataclass
class SystemState:
    robot_state: RobotState
    active_profile_id: str | None
    active_session_id: str | None
    last_normal_shutdown: bool
    updated_at: datetime


@dataclass
class RealtimeSnapshot:
    """ハードウェアから取得したリアルタイム計測値。"""

    actual_speed_kmh: float
    accel_pos: int
    brake_pos: int
    accel_current_ma: float
    brake_current_ma: float
