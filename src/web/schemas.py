# ruff: noqa: E501
from datetime import datetime
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from src.models.system_state import InitStepStatus, RobotState

# ── System state ──────────────────────────────────────────────────────────────


class JobSnapshotSchema(BaseModel):
    """JobManager のジョブ状態（src/app/job_manager.py の JobStatus.to_dict）。形は README。"""

    model_config = ConfigDict(extra="allow")

    job_id: str
    kind: str  # learning / mode_drive / interactive / init_only / schedule / ign_off
    vehicle_id: str
    purpose: str | None = None
    steps: str | None = None
    mode_id: str | None = None
    mode_name: str | None = None
    schedule_id: str | None = None
    schedule_name: str | None = None
    sequence: dict[str, Any] | None = None  # シーケンスの 1 段として走っているとき、その全体の進み具合
    run_type: str | None = None
    session_id: str | None = None
    phase: str
    step: int | None = None
    step_title: str | None = None
    init_steps: list[dict[str, str]] = []
    sample: dict[str, Any] | None = None
    interactive_status: dict[str, Any] | None = None
    started_at: str
    elapsed_s: float
    exit_code: int | None = None
    final_status: str | None = None
    stop_requested: bool = False
    emergency: bool = False
    # 非常停止の流れ（減速して停車 → N → IGN-OFF → 原点復帰）の今の段と詳細。止まらずブレーキを踏んだまま終えたか
    emergency_stage: str | None = None
    emergency_detail: str | None = None
    brake_kept: bool = False
    buttons_unavailable: bool = False
    result: dict[str, Any] | None = None
    ended_at: str | None = None
    paused: bool = False  # 一時停止中（モード走行）
    # ボタン押下 [{button: start/P/N/D, seconds, start_s, end_s|None}]（時刻はサンプルの mode_time_s ?? t_s と同じ軸）
    button_events: list[dict[str, Any]] = []
    dev_stats: dict[str, Any] | None = None  # モード走行の |偏差| の {max, p95, n}（一時停止中の行を除く）


class SystemStateResponse(BaseModel):
    robot_state: RobotState
    active_profile_id: str | None  # 選んでいる車の ID（フォルダ名）
    active_session_id: str | None
    last_normal_shutdown: bool
    updated_at: datetime
    manager_state: str = "standby"  # standby / running / emergency
    # 初期化ページの初期化が済んでいるか（モード走行・学習を始める条件。サーバー起動・非常停止・エラー終了で false）
    initialized: bool = False
    # 初期化ページが「なし（設定で off）」と出すための、機器ごとの有無（ups・gpio・buttons。true が「ある」）
    devices: dict[str, bool] = {}
    job: JobSnapshotSchema | None = None
    last_job: JobSnapshotSchema | None = None
    # シーケンスの実行中の進み具合と、直前に終わったシーケンス（段・回数・結果。形は job_manager.SequenceRun.to_dict）
    sequence: dict[str, Any] | None = None
    last_sequence: dict[str, Any] | None = None
    # 走行の準備（開始ポップアップの間の計画作成）。形は job_manager.PrepareRun.to_dict。無ければ None
    prepare: dict[str, Any] | None = None


class JobStartResponse(BaseModel):
    """ジョブ開始の応答。session_id は DB が無い環境・セッションを作らないジョブでは None。"""

    job_id: str
    kind: str
    vehicle_id: str
    steps: str | None = None
    run_type: str | None = None
    session_id: str | None = None
    started_at: datetime


class LogLineSchema(BaseModel):
    seq: int
    t: float
    time: str
    level: str
    logger: str
    message: str


class DriveLogResponse(BaseModel):
    lines: list[LogLineSchema]
    last_seq: int


# ── Drive session ─────────────────────────────────────────────────────────────


class StartDriveRequest(BaseModel):
    mode_id: str
    # 開発・動作確認用: モード走行を先頭 limit_s 秒で打ち切る（ランナーの --limit-s）。通常は指定しない
    limit_s: float | None = Field(default=None, gt=0, le=36000)


class DriveSessionResponse(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    id: str
    profile_id: str
    mode_id: str | None
    run_type: str
    started_at: datetime
    ended_at: datetime | None
    status: str


class LearningStartRequest(BaseModel):
    """学習の種類: full ＝ 手順 2（探索 → G ランプ → モデル作成 → チェック）、
    fel ＝ 手順 12・hold ＝ 手順 13・stop ＝ 手順 14（今の pkl への部分的な学習のやり直し）。"""

    kind: Literal["full", "fel", "hold", "stop"] = "full"


class SaveMinRequest(BaseModel):
    axis: Literal["accel", "brake", "both"] = "both"


class SaveMaxRequest(BaseModel):
    axis: Literal["accel", "brake"]


class SaveMaxResponse(BaseModel):
    message: str
    accel_pct: float | None = None
    brake_pct: float | None = None


class SaveMinResponse(BaseModel):
    message: str
    accel_pct: float | None = None
    brake_pct: float | None = None
    min_accel_measured: bool
    min_brake_measured: bool


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
    # 初期化ページの初期化が済んでいるか（false の間はモード走行・学習を始められない。D-15c）
    initialized: bool = False
    # 実行中のジョブの状態（段3: cycle_progress の置き換え。手順・phase・section など）
    job: JobSnapshotSchema | None = None
    # 実行中のシーケンスの進み具合（段の切れ目は job が None でもここは残る）
    sequence: dict[str, Any] | None = None
    # 走行の準備（計画作成）の状態。無ければ None
    prepare: dict[str, Any] | None = None
    # 非常停止のきっかけ（screen / switch / ac_loss / auto。非常停止中だけ）
    emergency_cause: str | None = None
    emergency_cause_detail: str | None = None  # auto のときのランナーの理由


# ── Vehicle Profile ───────────────────────────────────────────────────────────


class ButtonsSchema(BaseModel):
    """ボタン操作（サーボ）の値。servo_map は {ボタン名: {channel, offset_deg}}。"""

    model_config = ConfigDict(extra="allow")

    start_short_press_s: float | None = None
    start_long_press_s: float | None = None
    ign_off_press_s: float | None = None
    installed: bool | None = None  # ボタンサーボ（SG90）が付いているか。false は押したことにするだけ
    shift_press_s: float | None = None
    after_press_wait_s: float | None = None
    accel_tap_opening_pct: float | None = None
    accel_tap_on_s: float | None = None
    accel_tap_off_s: float | None = None
    maintenance_tap_count: int | None = None
    n_hold_gap_s: float | None = None
    d_press_at_s: float | None = None
    no_speed_abort_s: float | None = None
    no_speed_kmh: float | None = None
    servo_rest_angle_deg: float | None = None
    servo_map: dict[str, Any] | None = None


class _VehicleValues(BaseModel):
    """人が決める vehicle.* の値（更新では送ったキーだけ書く。作成では省略すると仮値）。

    知らないキー（学習で決まる feedforward など）は黙って無視せず 422 にする。
    """

    model_config = ConfigDict(extra="forbid")

    max_speed_kmh: float | None = None
    max_decel_g: float | None = None
    min_accel_opening_pct: float | None = None
    max_accel_opening_pct: float | None = None
    min_brake_opening_pct: float | None = None
    max_brake_opening_pct: float | None = None
    stop_deviation_threshold_kmh: float | None = None
    stop_deviation_duration_s: float | None = None
    test_mass_kg: float | None = None
    road_load_a0_n: float | None = None
    road_load_a1_n_per_kmh: float | None = None
    road_load_a2_n_per_kmh2: float | None = None
    road_load_source: str | None = None


class ProfileCreateRequest(_VehicleValues):
    """車の登録。id（フォルダ名）を省略すると名前から作る。新しい車は学習の値なし・min は仮値（未測定）。"""

    name: str
    id: str | None = None


class ProfileUpdateRequest(_VehicleValues):
    """人が決める値の更新。学習で決まる feedforward.* は書けない。送ったキーだけ書く。"""

    name: str | None = None


class ProfileResponse(BaseModel):
    """車（vehicles/<車ID>/vehicle.yaml）の値。id はフォルダ名。"""

    id: str
    name: str
    error: str | None = None  # yaml が壊れている車（一覧にだけ出る）
    max_speed_kmh: float | None = None
    max_decel_g: float | None = None
    min_accel_opening_pct: float | None = None
    max_accel_opening_pct: float | None = None
    min_brake_opening_pct: float | None = None
    max_brake_opening_pct: float | None = None
    min_accel_measured: bool = False  # false ＝ 仮値（ジョグで測って save-min する）
    min_brake_measured: bool = False
    stop_deviation_threshold_kmh: float | None = None
    stop_deviation_duration_s: float | None = None
    test_mass_kg: float | None = None
    road_load_a0_n: float | None = None
    road_load_a1_n_per_kmh: float | None = None
    road_load_a2_n_per_kmh2: float | None = None
    road_load_source: str | None = None
    buttons: ButtonsSchema = ButtonsSchema()  # 共通設定の値（読むだけ）
    learned: bool = False  # model_path があり pkl が存在する
    model_path: str | None = None
    learned_at: datetime | None = None  # pkl の更新時刻
    candidate: str | None = None  # 走る制御の候補（共通設定。読むだけ）
    feedforward: dict[str, Any] = {}  # 学習で決まる値（読むだけ）
    updated_at: datetime | None = None


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
    """ボタン押下イベント。新しいイベントはボタン名（button）で指す。

    channel は旧データ（ch だけを持つ）の読み込み用で、button が無いイベントは「名前なし（要再設定）」。
    保存し直しで button を付ける。press_duration_s が無ければ車両 yaml の既定秒数で押す。
    """

    time_s: float = Field(ge=0.0)
    button: Literal["start", "P", "N", "D"] | None = None
    channel: int | None = Field(default=None, ge=0, le=15)
    press_duration_s: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def _button_or_channel(self) -> "ButtonEventSchema":
        if self.button is None and self.channel is None:
            raise ValueError("ボタン名（button: start / P / N / D）を指定してください")
        return self


class ScheduleResponse(BaseModel):
    id: str
    name: str
    description: str
    total_duration: float
    pedal_point_count: int
    button_event_count: int
    created_at: datetime


class ScheduleDetailResponse(BaseModel):
    id: str
    name: str
    description: str
    pedal_points: list[PedalPointSchema]
    button_events: list[ButtonEventSchema]
    total_duration: float
    created_at: datetime


class ScheduleCreateRequest(BaseModel):
    name: str = Field(min_length=1)
    description: str = ""
    pedal_points: list[PedalPointSchema] = Field(default_factory=list)
    button_events: list[ButtonEventSchema] = Field(default_factory=list)

    @field_validator("pedal_points")
    @classmethod
    def _pedal_points_monotonic(cls, v: list[PedalPointSchema]) -> list[PedalPointSchema]:
        prev = -1.0
        for p in v:
            if p.time_s <= prev:
                raise ValueError("pedal_points の time_s は単調増加である必要があります")
            prev = p.time_s
        return v

    @field_validator("button_events")
    @classmethod
    def _button_events_sorted(cls, v: list[ButtonEventSchema]) -> list[ButtonEventSchema]:
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


class PrepareRequest(BaseModel):
    """走行の準備（開始ポップアップの間に計画を作る）。kind に応じた ID（モード・スケジュール・シーケンス）。"""

    kind: Literal["mode", "schedule", "sequence"]
    id: str
    # 開発・動作確認用: モードの段を先頭 limit_s 秒で打ち切る（ランナーの --limit-s）。通常は指定しない
    limit_s: float | None = Field(default=None, gt=0, le=36000)


class StartScheduleRequest(BaseModel):
    schedule_id: str


class StartSequenceRequest(BaseModel):
    sequence_id: str
    # 開発・動作確認用: シーケンスの走行モードの段を先頭 limit_s 秒で打ち切る（ランナーの --limit-s）。通常は指定しない
    limit_s: float | None = Field(default=None, gt=0, le=36000)


# ── Session / Log ─────────────────────────────────────────────────────────────


class SessionResponse(BaseModel):
    id: str
    profile_id: str | None  # 旧プロファイル時代の行だけ。新しい走行は vehicle_id
    vehicle_id: str | None = None
    vehicle_name: str | None = None
    mode_id: str | None
    run_type: str
    started_at: datetime
    ended_at: datetime | None
    status: str
    cycle_id: str | None = None


class CycleSummaryResponse(BaseModel):
    """学習サイクルの一覧表示用サマリ（ログ画面のグループ表示に使う）。"""

    id: str
    profile_id: str
    status: str
    started_at: datetime
    ended_at: datetime | None
    session_count: int
    detail: dict[str, Any]


class MessageResponse(BaseModel):
    """進行のメッセージ 1 行（drive_session_messages）。"""

    seq: int
    elapsed_s: float
    time: datetime
    level: str
    message: str


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


# ── Sequence ─────────────────────────────────────────────────────────────────


class SequenceStepSchema(BaseModel):
    """シーケンスの1段（リクエスト用）。"""

    kind: Literal["mode", "schedule"]
    ref_id: str = Field(min_length=1)
    repeat: int = Field(default=1, ge=1)


class SequenceStepResponse(BaseModel):
    """シーケンスの1段（参照先を解決した応答用）。見つからない場合は missing=True・長さ 0。"""

    kind: Literal["mode", "schedule"]
    ref_id: str
    repeat: int
    name: str
    duration_s: float
    missing: bool


class SequenceResponse(BaseModel):
    id: str
    name: str
    description: str
    steps: list[SequenceStepResponse]
    repeat: int
    total_duration: float
    has_missing: bool
    created_at: datetime


class SequenceCreateRequest(BaseModel):
    name: str = Field(min_length=1)
    description: str = ""
    steps: list[SequenceStepSchema] = Field(min_length=1)
    repeat: int = Field(default=1, ge=1)


class SequenceUpdateRequest(BaseModel):
    name: str = Field(min_length=1)
    description: str = ""
    steps: list[SequenceStepSchema] = Field(min_length=1)
    repeat: int = Field(default=1, ge=1)
