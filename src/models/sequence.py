"""シーケンス（走行モードとタイムスケジュールを順に並べたもの）のドメインモデル。

各段は走行モードまたはタイムスケジュールを ID で参照する（外部キーにはしない）。
実行（自動運転画面から走らせる）は対象外で、作成・編集・一覧・削除・保存のみ扱う。
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Literal


@dataclass(frozen=True)
class SequenceStep:
    """シーケンスの1段。"""

    kind: Literal["mode", "schedule"]  # mode=走行モード / schedule=タイムスケジュール
    ref_id: str  # driving_modes.id または time_schedules.id
    repeat: int = 1  # この段の繰り返し回数（>=1）


@dataclass(frozen=True)
class Sequence:
    """段を順につないだシーケンス。"""

    id: str
    name: str
    description: str
    steps: list[SequenceStep]
    repeat: int  # 全体の繰り返し回数（>=1）
    created_at: datetime
