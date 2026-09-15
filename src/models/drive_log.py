from dataclasses import dataclass, field
from datetime import datetime
from typing import Any


@dataclass
class DriveSession:
    id: str
    profile_id: str
    mode_id: str | None
    run_type: str  # 'auto' | 'manual' | 'learning' | 'tuning'
    started_at: datetime
    ended_at: datetime | None
    status: str  # 'running' | 'completed' | 'error' | 'emergency'
    cycle_id: str | None = None


@dataclass
class LearningCycle:
    id: str
    profile_id: str
    status: str  # 'running' | 'completed' | 'error' | 'aborted'
    started_at: datetime
    ended_at: datetime | None
    detail: dict[str, Any] = field(default_factory=dict)
    session_count: int = 0


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
    # effort 内訳（エピソード型プラン学習）。自動走行のみ非 None。学習運転・スケジュール走行・
    # 旧セッションは None（後方互換）。applied はフェーズ権限クランプ後・調停器前の合成値。
    plan_effort_pct: float | None = None
    trim_effort_pct: float | None = None
    applied_effort_pct: float | None = None
    phase: str | None = None


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
    # effort 内訳（プラン学習・トリム寄与率の可観測化）。デフォルト None で学習運転・
    # スケジュール走行は無変更（それらの LogWriter 呼び出しは既存のまま通る）。
    plan_effort_pct: float | None = None
    trim_effort_pct: float | None = None
    applied_effort_pct: float | None = None
    phase: str | None = None
