from dataclasses import dataclass
from datetime import datetime


@dataclass
class DriveSession:
    id: str
    profile_id: str
    mode_id: str | None
    run_type: str  # 'auto' | 'manual' | 'learning'
    started_at: datetime
    ended_at: datetime | None
    status: str  # 'running' | 'completed' | 'error' | 'emergency'


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
