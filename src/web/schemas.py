from datetime import datetime
from typing import Literal

from pydantic import BaseModel, Field, field_validator

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
    # ペダル調停（PedalArbiter）定数
    switch_hysteresis_pct: float = 0.5
    accel_reengage_dwell_s: float = 0.3
    accel_rate_limit_pct_s: float = 200.0
    brake_rate_limit_pct_s: float = 300.0
    pid_output_limit_pct: float = 50.0


class PIDGainsSchema(BaseModel):
    kp: float
    ki: float
    kd: float


class TrainModelRequest(BaseModel):
    profile_id: str
    session_ids: list[str] | None = None


class TrainModelResponse(BaseModel):
    model_path: str
    metrics: dict[str, dict[str, float]]
    feedforward_params: FeedforwardParamsSchema
    # 学習ログから解析的に自動適合した PID ゲイン。同定不能（区間不足）なら None。
    pid_gains: PIDGainsSchema | None = None
    pid_auto_tuned: bool = False


class PidValidateRequest(BaseModel):
    profile_id: str


class PidValidateResponse(BaseModel):
    """規定パターン 1 回走行の追従性能。"""

    kpi_summary: dict[str, float]
    cost: float
    pid_gains: PIDGainsSchema


class PidRefineRequest(BaseModel):
    profile_id: str
    max_runs: int = Field(default=15, ge=1, le=50)


class PidRefineResponse(BaseModel):
    """閉ループ絞り込みの最良ゲインと反復履歴。"""

    pid_gains: PIDGainsSchema
    best_cost: float
    history: list[dict[str, float]]


class SelectProfileRequest(BaseModel):
    profile_id: str


class JogRequest(BaseModel):
    axis: Literal["accel", "brake"]
    # 移動量 [pulse]。UI のドラッグスライダー最大量(±5000)を上限とし、
    # 任意クライアントが巨大な step でアクチュエータを機械端まで叩き込むのを防ぐ。
    step: int = Field(ge=-5000, le=5000)


class AxisRequest(BaseModel):
    axis: Literal["accel", "brake"]


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


# ── Time schedule（統合タイムライン）─────────────────────────────────────────────


class PedalPointSchema(BaseModel):
    time_s: float = Field(ge=0.0)
    accel_opening: float = Field(ge=0.0, le=100.0)
    brake_opening: float = Field(ge=0.0, le=100.0)


class ButtonEventSchema(BaseModel):
    time_s: float = Field(ge=0.0)
    channel: int = Field(ge=0, le=15)
    press_duration_s: float = Field(gt=0.0)


class ScheduleResponse(BaseModel):
    id: str
    name: str
    description: str
    total_duration: float
    pedal_point_count: int
    button_event_count: int
    loop: bool
    created_at: datetime


class ScheduleDetailResponse(BaseModel):
    id: str
    name: str
    description: str
    pedal_points: list[PedalPointSchema]
    button_events: list[ButtonEventSchema]
    total_duration: float
    loop: bool
    created_at: datetime


class ScheduleCreateRequest(BaseModel):
    name: str = Field(min_length=1)
    description: str = ""
    pedal_points: list[PedalPointSchema] = Field(default_factory=list)
    button_events: list[ButtonEventSchema] = Field(default_factory=list)
    loop: bool = False

    @field_validator("pedal_points")
    @classmethod
    def _pedal_points_monotonic(
        cls, v: list[PedalPointSchema]
    ) -> list[PedalPointSchema]:
        prev = -1.0
        for p in v:
            if p.time_s <= prev:
                raise ValueError("pedal_points の time_s は単調増加である必要があります")
            prev = p.time_s
        return v

    @field_validator("button_events")
    @classmethod
    def _button_events_sorted(
        cls, v: list[ButtonEventSchema]
    ) -> list[ButtonEventSchema]:
        prev = -1.0
        for e in v:
            if e.time_s < prev:
                raise ValueError("button_events の time_s は昇順である必要があります")
            prev = e.time_s
        return v


class ScheduleUpdateRequest(BaseModel):
    name: str | None = Field(default=None, min_length=1)
    description: str | None = None
    pedal_points: list[PedalPointSchema] | None = None
    button_events: list[ButtonEventSchema] | None = None
    loop: bool | None = None

    @field_validator("pedal_points")
    @classmethod
    def _pedal_points_monotonic(
        cls, v: list[PedalPointSchema] | None
    ) -> list[PedalPointSchema] | None:
        if v is None:
            return v
        prev = -1.0
        for p in v:
            if p.time_s <= prev:
                raise ValueError("pedal_points の time_s は単調増加である必要があります")
            prev = p.time_s
        return v


class StartScheduleRequest(BaseModel):
    schedule_id: str


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
