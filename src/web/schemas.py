from datetime import datetime

from pydantic import BaseModel

from src.models.system_state import RobotState


class SystemStateResponse(BaseModel):
    robot_state: RobotState
    active_profile_id: str | None
    active_session_id: str | None
    last_normal_shutdown: bool
    updated_at: datetime


class StartDriveRequest(BaseModel):
    mode_id: str


class DriveSessionResponse(BaseModel):
    id: str
    profile_id: str
    mode_id: str | None
    run_type: str
    started_at: datetime
    ended_at: datetime | None
    status: str


class RealtimeData(BaseModel):
    timestamp: str
    robot_state: RobotState
    actual_speed_kmh: float
    ref_speed_kmh: float | None
    accel_opening: float
    brake_opening: float
    accel_current_ma: float
    brake_current_ma: float


class ErrorResponse(BaseModel):
    detail: str
