from dataclasses import dataclass
from datetime import datetime


@dataclass
class SpeedPoint:
    time_s: float
    speed_kmh: float


@dataclass
class DrivingMode:
    id: str
    name: str
    description: str
    reference_speed: list[SpeedPoint]
    total_duration: float
    max_speed: float
    created_at: datetime
    # 学習サイクルが内部生成する網羅検証パターン（プラン学習の保存先）は is_system=True。
    # ユーザー向けモード一覧から除外し、WebUI 編集・削除も拒否する。既存呼び出しは False 既定。
    is_system: bool = False
