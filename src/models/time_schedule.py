"""タイムスケジュール（統合タイムライン）のドメインモデル。

基準車速を使わず、時系列でペダル開度（アクセル・ブレーキ）と物理ボタン押下
（ボタンサーボ）を開ループで再生する。ペダル動作とボタンイベントを同一タイムライン上で
管理する（`docs/functional-design.md` の TimeSchedule 定義に準拠）。
"""

from dataclasses import dataclass
from datetime import datetime


@dataclass
class PedalPoint:
    """時刻ごとのアクセル・ブレーキ開度 [%]。"""

    time_s: float
    accel_opening: float  # アクセル開度 [%] 0-100
    brake_opening: float  # ブレーキ開度 [%] 0-100


@dataclass
class ButtonEvent:
    """時刻ごとのボタン押下イベント。"""

    time_s: float  # 押下開始時刻 [s]
    channel: int  # PCA9685 チャンネル（0-15、ButtonServoDriver のマッピング参照）
    press_duration_s: float  # 押下時間 [s]（ボタンごとに設定）


@dataclass
class TimeSchedule:
    """ペダル開度とボタンイベントを統合した1本のタイムライン。"""

    id: str
    name: str
    description: str
    pedal_points: list[PedalPoint]
    button_events: list[ButtonEvent]
    total_duration: float  # 総時間 [s]
    loop: bool  # ループ再生の有無
    created_at: datetime
