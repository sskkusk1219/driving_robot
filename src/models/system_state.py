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


class InitStepStatus(StrEnum):
    """初期化シーケンス各ステップの進捗状態。フロント表示と連動する。"""

    PENDING = "pending"  # 未実施 (—)
    RUNNING = "running"  # 実行中 (⟳)
    DONE = "done"  # 完了 (✓)
    SKIPPED = "skipped"  # スキップ（前回正常終了で原点復帰不要等）
    ERROR = "error"  # 失敗 (✗)


@dataclass
class InitStep:
    """初期化シーケンスの 1 ステップ。key は実ハード操作との対応キー。"""

    key: str
    label: str
    status: InitStepStatus = InitStepStatus.PENDING


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
