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
    # 旧データだけが持つ PCA9685 のチャンネル（0-15）。新しいイベントはボタン名（button）で指す。
    # button が無い旧イベントは「名前なし（要再設定）」で、実行時はエラーにする（推測で押さない）
    channel: int | None = None
    press_duration_s: float | None = None  # 押下時間 [s]。None なら車両 yaml の既定秒数
    button: str | None = None  # ボタン名: start / P / N / D（車両 yaml の servo_map のキー）


@dataclass
class TimeSchedule:
    """ペダル開度とボタンイベントを統合した1本のタイムライン。"""

    id: str
    name: str
    description: str
    pedal_points: list[PedalPoint]
    button_events: list[ButtonEvent]
    total_duration: float  # 総時間 [s]
    created_at: datetime
