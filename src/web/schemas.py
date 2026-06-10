from datetime import datetime

from pydantic import BaseModel, field_validator

from src.models.system_state import InitStepStatus, RobotState

# ── Calibration ──────────────────────────────────────────────────────────────

class CalibrationDataResponse(BaseModel):
    accel_zero_pos: int
    accel_full_pos: int
    accel_stroke: int
    brake_zero_pos: int
    brake_full_pos: int
    brake_stroke: int
    calibrated_at: datetime
    is_valid: bool


class CalibrationResultResponse(BaseModel):
    success: bool
    error_message: str | None
    data: CalibrationDataResponse | None


# ── System state ──────────────────────────────────────────────────────────────

class SystemStateResponse(BaseModel):
    robot_state: RobotState
    active_profile_id: str | None
    active_session_id: str | None
    last_normal_shutdown: bool
    updated_at: datetime


# ── Drive session ─────────────────────────────────────────────────────────────

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


class FeedforwardParamsSchema(BaseModel):
    """Ridge 逆FFモデルを補う車両物理定数。デフォルトは AT 標準値。"""

    creep_speed_kmh: float = 7.0
    creep_rate_kmhs: float = 0.5
    engine_brake_decel_kmhs: float = 1.0
    stop_brake_opening_pct: float = 20.0
    brake_deadband_pct: float = 1.0
    accel_deadband_pct: float = 1.0


class TrainModelRequest(BaseModel):
    profile_id: str
    session_ids: list[str] | None = None


class TrainModelResponse(BaseModel):
    model_path: str
    metrics: dict[str, dict[str, float]]
    feedforward_params: FeedforwardParamsSchema


class SelectProfileRequest(BaseModel):
    profile_id: str


class JogRequest(BaseModel):
    axis: str  # "accel" or "brake"
    step: int  # 移動量 [pulse]


class AxisRequest(BaseModel):
    axis: str  # "accel" or "brake"


class JogResponse(BaseModel):
    position: int  # 操作後の実位置 [pulse]


# ── UPS ──────────────────────────────────────────────────────────────────────

class UPSStatusResponse(BaseModel):
    battery_charge_pct: float
    on_battery: bool
    low_battery: bool
    status_flags: str
    is_available: bool


# ── Realtime WebSocket ────────────────────────────────────────────────────────

class InitStepSchema(BaseModel):
    """初期化シーケンス各ステップの進捗。フロント初期化画面の表示と連動する。"""

    key: str
    label: str
    status: InitStepStatus


class RealtimeData(BaseModel):
    timestamp: str
    robot_state: RobotState
    actual_speed_kmh: float
    ref_speed_kmh: float | None
    accel_opening: float
    brake_opening: float
    accel_current_ma: float
    brake_current_ma: float
    ups_battery_pct: float | None = None
    ups_on_battery: bool = False
    init_steps: list[InitStepSchema] = []


# ── Vehicle Profile ───────────────────────────────────────────────────────────

class PIDGainsSchema(BaseModel):
    kp: float
    ki: float
    kd: float


class StopConfigSchema(BaseModel):
    deviation_threshold_kmh: float
    deviation_duration_s: float


class ProfileCreateRequest(BaseModel):
    name: str
    max_accel_opening: float
    max_brake_opening: float
    max_speed: float
    max_decel_g: float
    pid_gains: PIDGainsSchema
    stop_config: StopConfigSchema
    model_path: str | None = None
    feedforward_params: FeedforwardParamsSchema | None = None

    @field_validator("max_accel_opening", "max_brake_opening")
    @classmethod
    def opening_range(cls, v: float) -> float:
        if not (0.0 <= v <= 100.0):
            raise ValueError("開度は 0〜100% の範囲で指定してください")
        return v

    @field_validator("max_speed")
    @classmethod
    def speed_positive(cls, v: float) -> float:
        if v <= 0:
            raise ValueError("最高車速は正の値を指定してください")
        return v


class ProfileUpdateRequest(BaseModel):
    name: str | None = None
    max_accel_opening: float | None = None
    max_brake_opening: float | None = None
    max_speed: float | None = None
    max_decel_g: float | None = None
    pid_gains: PIDGainsSchema | None = None
    stop_config: StopConfigSchema | None = None
    model_path: str | None = None
    feedforward_params: FeedforwardParamsSchema | None = None

    @field_validator("max_accel_opening", "max_brake_opening")
    @classmethod
    def opening_range(cls, v: float | None) -> float | None:
        if v is not None and not (0.0 <= v <= 100.0):
            raise ValueError("開度は 0〜100% の範囲で指定してください")
        return v

    @field_validator("max_speed")
    @classmethod
    def speed_positive(cls, v: float | None) -> float | None:
        if v is not None and v <= 0:
            raise ValueError("最高車速は正の値を指定してください")
        return v


class ProfileResponse(BaseModel):
    id: str
    name: str
    max_accel_opening: float
    max_brake_opening: float
    max_speed: float
    max_decel_g: float
    pid_gains: PIDGainsSchema
    stop_config: StopConfigSchema
    calibration: CalibrationDataResponse | None
    model_path: str | None
    feedforward_params: FeedforwardParamsSchema
    created_at: datetime
    updated_at: datetime


# ── Driving Mode ──────────────────────────────────────────────────────────────

class SpeedPointSchema(BaseModel):
    time_s: float
    speed_kmh: float


class ModeResponse(BaseModel):
    id: str
    name: str
    description: str
    total_duration: float
    max_speed: float
    point_count: int
    created_at: datetime


class ModeDetailResponse(BaseModel):
    id: str
    name: str
    description: str
    reference_speed: list[SpeedPointSchema]
    total_duration: float
    max_speed: float
    created_at: datetime


class ModeUpdateRequest(BaseModel):
    name: str | None = None
    description: str | None = None


# ── Session / Log ─────────────────────────────────────────────────────────────

class SessionResponse(BaseModel):
    id: str
    profile_id: str
    mode_id: str | None
    run_type: str
    started_at: datetime
    ended_at: datetime | None
    status: str


class LogResponse(BaseModel):
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


# ── Error ─────────────────────────────────────────────────────────────────────

class ErrorResponse(BaseModel):
    detail: str
