# ruff: noqa: E501
"""PgDriveRecorder.start_session が schedule_id・schedule_name を INSERT に渡す（O-5）。DB 不要（偽のプール）。"""

from __future__ import annotations

import uuid
from typing import Any

from src.infra.drive_recorder import PgDriveRecorder, ensure_session_schema


class _Pool:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple[Any, ...]]] = []

    async def execute(self, sql: str, *args: Any) -> None:
        self.calls.append((sql, args))


async def test_start_session_passes_schedule_values() -> None:
    pool = _Pool()
    sid = await PgDriveRecorder(pool).start_session(  # type: ignore[arg-type]
        vehicle_id="v1", mode_id=None, run_type="auto",
        schedule_id="00000000-0000-0000-0000-0000000000b2", schedule_name="メンテナンスモード_v2",
    )
    sql, args = pool.calls[0]
    assert "schedule_id, schedule_name" in sql and uuid.UUID(sid) == args[0]
    assert args[-4] == uuid.UUID("00000000-0000-0000-0000-0000000000b2") and args[-3] == "メンテナンスモード_v2"


async def test_start_session_schedule_defaults_to_null_and_bad_id_is_ignored() -> None:
    pool = _Pool()
    rec = PgDriveRecorder(pool)  # type: ignore[arg-type]
    await rec.start_session(vehicle_id="v1", mode_id=None, run_type="auto")
    await rec.start_session(vehicle_id="v1", mode_id=None, run_type="auto", schedule_id="", schedule_name="")
    await rec.start_session(vehicle_id="v1", mode_id=None, run_type="auto", schedule_id="x", schedule_name="S")
    assert [a[-4:-2] for _, a in pool.calls] == [(None, None), (None, None), (None, "S")]


async def test_ensure_session_schema_runs_alter_if_not_exists() -> None:
    pool = _Pool()
    await ensure_session_schema(pool)  # type: ignore[arg-type]
    joined = "\n".join(sql for sql, _ in pool.calls)
    assert "ADD COLUMN IF NOT EXISTS schedule_id UUID" in joined and "ADD COLUMN IF NOT EXISTS schedule_name TEXT" in joined


async def test_start_session_passes_sequence_values() -> None:
    pool = _Pool()
    rid = "00000000-0000-0000-0000-0000000000e1"
    await PgDriveRecorder(pool).start_session(  # type: ignore[arg-type]
        vehicle_id="v1", mode_id=None, run_type="auto", sequence_run_id=rid, sequence_name="夜間シーケンス"
    )
    await PgDriveRecorder(pool).start_session(vehicle_id="v1", mode_id=None, run_type="auto")  # type: ignore[arg-type]
    sql, args = pool.calls[0]
    assert "sequence_run_id, sequence_name" in sql
    assert args[-2:] == (uuid.UUID(rid), "夜間シーケンス") and pool.calls[1][1][-2:] == (None, None)
    await ensure_session_schema(pool)  # type: ignore[arg-type]
    joined = "\n".join(q for q, _ in pool.calls)
    assert "ADD COLUMN IF NOT EXISTS sequence_run_id UUID" in joined and "ADD COLUMN IF NOT EXISTS sequence_name TEXT" in joined
