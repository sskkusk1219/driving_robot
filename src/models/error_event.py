# ruff: noqa: E501
"""エラーページに残す「できごと 1 件」（error_events）のモデル。"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Any

#: 水準（エラーは赤・警告は黄）
LEVEL_ERROR = "ERROR"
LEVEL_WARNING = "WARNING"

#: きっかけ（source）
SOURCE_JOB = "job"
SOURCE_EMERGENCY = "emergency"
SOURCE_IDLE = "idle"
SOURCE_PREPARE = "prepare"
SOURCE_SEQUENCE = "sequence"
SOURCE_SERVER = "server"
SOURCE_BROWSER = "browser"
SOURCES = (
    SOURCE_JOB,
    SOURCE_EMERGENCY,
    SOURCE_IDLE,
    SOURCE_PREPARE,
    SOURCE_SEQUENCE,
    SOURCE_SERVER,
    SOURCE_BROWSER,
)

#: 1 件に付ける直前の進行メッセージの最大行数
CONTEXT_MESSAGES_MAX = 50


@dataclass
class NewErrorEvent:
    """これから記録するできごと。``dedupe`` が真なら、同じ文が 60 秒以内なら回数を足す（記録係が判断する）。"""

    level: str
    source: str
    title: str
    detail: str = ""
    context: dict[str, Any] = field(default_factory=dict)
    occurred_at: datetime | None = None
    job_id: str | None = None
    job_kind: str | None = None
    vehicle_id: str | None = None
    session_id: str | None = None
    sequence_name: str | None = None
    dedupe: bool = False


@dataclass
class ErrorEvent:
    """保存済みのできごと 1 件。"""

    id: int
    first_at: datetime
    last_at: datetime
    count: int
    level: str
    source: str
    title: str
    detail: str
    context: dict[str, Any]
    job_id: str | None = None
    job_kind: str | None = None
    vehicle_id: str | None = None
    session_id: str | None = None
    sequence_name: str | None = None
    acknowledged_at: datetime | None = None
