# ruff: noqa: E501
"""エラーのできごと（error_events）を PostgreSQL へ永続化する。"""

from __future__ import annotations

import json
from datetime import datetime
from typing import Any

import asyncpg

from src.models.error_event import ErrorEvent, NewErrorEvent

#: テーブルの作成文（scripts/setup_db.py と同じ内容。起動時にも流し、setup_db を流し忘れても書けるようにする）
DDL_STATEMENTS = [
    """
    CREATE TABLE IF NOT EXISTS error_events (
        id              BIGSERIAL PRIMARY KEY,
        first_at        TIMESTAMPTZ NOT NULL,
        last_at         TIMESTAMPTZ NOT NULL,
        count           INTEGER NOT NULL DEFAULT 1,
        level           TEXT NOT NULL,
        source          TEXT NOT NULL,
        title           TEXT NOT NULL,
        detail          TEXT NOT NULL DEFAULT '',
        context         JSONB NOT NULL DEFAULT '{}'::jsonb,
        job_id          TEXT,
        job_kind        TEXT,
        vehicle_id      TEXT,
        session_id      TEXT,
        sequence_name   TEXT,
        acknowledged_at TIMESTAMPTZ
    )
    """,
    """
    CREATE INDEX IF NOT EXISTS idx_error_events_last_at
        ON error_events (last_at DESC)
    """,
]


def _row_to_event(row: asyncpg.Record) -> ErrorEvent:
    ctx = row["context"]
    if isinstance(ctx, str):
        ctx = json.loads(ctx)
    return ErrorEvent(
        id=row["id"],
        first_at=row["first_at"],
        last_at=row["last_at"],
        count=row["count"],
        level=row["level"],
        source=row["source"],
        title=row["title"],
        detail=row["detail"],
        context=ctx or {},
        job_id=row["job_id"],
        job_kind=row["job_kind"],
        vehicle_id=row["vehicle_id"],
        session_id=row["session_id"],
        sequence_name=row["sequence_name"],
        acknowledged_at=row["acknowledged_at"],
    )


def _filters(level: str | None, source: str | None, unacknowledged_only: bool) -> tuple[str, list[Any]]:
    conds: list[str] = []
    args: list[Any] = []
    if level:
        args.append(level)
        conds.append(f"level = ${len(args)}")
    if source:
        args.append(source)
        conds.append(f"source = ${len(args)}")
    if unacknowledged_only:
        conds.append("acknowledged_at IS NULL")
    return (" WHERE " + " AND ".join(conds)) if conds else "", args


class ErrorRepository:
    """error_events テーブルのリポジトリ。"""

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def ensure_schema(self) -> None:
        """テーブルが無ければ作る（冪等）。"""
        async with self._pool.acquire() as conn:
            for stmt in DDL_STATEMENTS:
                await conn.execute(stmt)

    async def insert(self, ev: NewErrorEvent, *, at: datetime) -> int:
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                INSERT INTO error_events
                    (first_at, last_at, count, level, source, title, detail, context,
                     job_id, job_kind, vehicle_id, session_id, sequence_name)
                VALUES ($1, $1, 1, $2, $3, $4, $5, $6::jsonb, $7, $8, $9, $10, $11)
                RETURNING id
                """,
                at, ev.level, ev.source, ev.title, ev.detail,
                json.dumps(ev.context, ensure_ascii=False, default=str),
                ev.job_id, ev.job_kind, ev.vehicle_id, ev.session_id, ev.sequence_name,
            )
        return int(row["id"])

    async def bump(self, event_id: int, *, at: datetime) -> bool:
        """同じ文がまた起きた: 回数を足し、最後の日時を更新し、確認済みを外す。無ければ False。"""
        async with self._pool.acquire() as conn:
            res = await conn.execute(
                "UPDATE error_events SET count = count + 1, last_at = $2, acknowledged_at = NULL WHERE id = $1",
                event_id, at,
            )
        return res.endswith(" 1")

    async def append_detail(
        self, event_id: int, text: str, *, at: datetime, context_patch: dict[str, Any] | None = None
    ) -> bool:
        """本文の末尾へ書き足す（非常停止の件への終了処理の結果）。context_patch は context へ上書きで足す。"""
        async with self._pool.acquire() as conn:
            res = await conn.execute(
                """
                UPDATE error_events
                   SET detail = CASE WHEN detail = '' THEN $2 ELSE detail || E'\\n' || $2 END,
                       last_at = $3,
                       context = context || $4::jsonb
                 WHERE id = $1
                """,
                event_id, text, at, json.dumps(context_patch or {}, ensure_ascii=False, default=str),
            )
        return res.endswith(" 1")

    async def list_events(
        self,
        *,
        level: str | None = None,
        source: str | None = None,
        unacknowledged_only: bool = False,
        limit: int = 50,
        offset: int = 0,
    ) -> tuple[list[ErrorEvent], int]:
        """新しい順（last_at 降順）。(該当の一覧, 該当の総数)。"""
        where, args = _filters(level, source, unacknowledged_only)
        async with self._pool.acquire() as conn:
            total = await conn.fetchval(f"SELECT count(*) FROM error_events{where}", *args)
            rows = await conn.fetch(
                f"SELECT * FROM error_events{where} ORDER BY last_at DESC, id DESC "
                f"LIMIT ${len(args) + 1} OFFSET ${len(args) + 2}",
                *args, limit, offset,
            )
        return [_row_to_event(r) for r in rows], int(total)

    async def get(self, event_id: int) -> ErrorEvent | None:
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow("SELECT * FROM error_events WHERE id = $1", event_id)
        return _row_to_event(row) if row is not None else None

    async def set_acknowledged(self, event_id: int, acknowledged: bool, *, at: datetime) -> bool:
        async with self._pool.acquire() as conn:
            res = await conn.execute(
                "UPDATE error_events SET acknowledged_at = $2 WHERE id = $1",
                event_id, at if acknowledged else None,
            )
        return res.endswith(" 1")

    async def acknowledge_all(self, *, at: datetime) -> int:
        async with self._pool.acquire() as conn:
            res = await conn.execute(
                "UPDATE error_events SET acknowledged_at = $1 WHERE acknowledged_at IS NULL", at
            )
        return int(res.split()[-1])

    async def delete(self, event_id: int) -> bool:
        async with self._pool.acquire() as conn:
            res = await conn.execute("DELETE FROM error_events WHERE id = $1", event_id)
        return res.endswith(" 1")

    async def delete_acknowledged(self) -> int:
        async with self._pool.acquire() as conn:
            res = await conn.execute("DELETE FROM error_events WHERE acknowledged_at IS NOT NULL")
        return int(res.split()[-1])
