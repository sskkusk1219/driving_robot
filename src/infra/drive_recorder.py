# ruff: noqa: E501
"""ジョブ（ランナーのサブプロセス）の記録を DB へ書く。JobManager が使う唯一の DB 口。

- ``drive_sessions``: ジョブの開始で作り（vehicle_id・run_type・mode_id・スケジュールのときは schedule_id/名前）、終了で status を更新する
- ``drive_session_messages``: ランナーの標準出力の「進行のメッセージ」
- ``drive_logs``: ランナーが保存した CSV を取り込む（9 列＋残りは detail JSONB。対応は drive_csv.py）

DB が無い環境（DATABASE_URL 未設定）では ``NullDriveRecorder``（何も書かない）を使う。
"""

from __future__ import annotations

import logging
import uuid
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import Any, Protocol

import asyncpg

from src.infra import drive_csv

logger = logging.getLogger(__name__)

#: CSV 取り込みで 1 回に書く行数（メモリと往復回数の釣り合い）
IMPORT_CHUNK_ROWS = 5000

#: 進行のメッセージ 1 行（seq・経過秒・時刻・level・本文）
MessageRow = tuple[int, float, datetime, str, str]


#: 起動時にも流す DDL（scripts/setup_db.py と同じ内容。setup_db を流し忘れても書けるようにする。IF NOT EXISTS で何度でも安全）
SESSION_DDL_STATEMENTS = [
    "ALTER TABLE drive_sessions ADD COLUMN IF NOT EXISTS schedule_id UUID",
    "ALTER TABLE drive_sessions ADD COLUMN IF NOT EXISTS schedule_name TEXT",
    """
    DO $$
    BEGIN
        IF EXISTS (
            SELECT 1 FROM pg_constraint
            WHERE conname = 'drive_sessions_status_check'
              AND conrelid = 'drive_sessions'::regclass
        ) THEN
            ALTER TABLE drive_sessions DROP CONSTRAINT drive_sessions_status_check;
            ALTER TABLE drive_sessions
                ADD CONSTRAINT drive_sessions_status_check
                CHECK (status IN ('running', 'completed', 'stopped', 'error', 'emergency'));
        END IF;
    END $$;
    """,
]


async def ensure_session_schema(pool: asyncpg.Pool) -> None:
    """drive_sessions の追加の列・制約を用意する（起動時）。"""
    for stmt in SESSION_DDL_STATEMENTS:
        await pool.execute(stmt)


def _uuid_or_none(value: str | None) -> uuid.UUID | None:
    """UUID 形式の文字列だけ UUID にする（空・形式違いは None。記録のために走行を止めない）。"""
    if not value:
        return None
    try:
        return uuid.UUID(value)
    except ValueError:
        return None


class DriveRecorder(Protocol):
    async def start_session(
        self,
        *,
        vehicle_id: str,
        mode_id: str | None,
        run_type: str,
        schedule_id: str | None = None,
        schedule_name: str | None = None,
    ) -> str | None: ...

    async def add_messages(self, session_id: str, rows: Sequence[MessageRow]) -> None: ...

    async def import_csv(self, session_id: str, path: Path) -> int: ...

    async def end_session(self, session_id: str, status: str) -> None: ...


class NullDriveRecorder:
    """DB が無いときの記録先。セッションを作らない（None）ので、他の操作は呼ばれない。"""

    async def start_session(
        self,
        *,
        vehicle_id: str,  # noqa: ARG002
        mode_id: str | None,  # noqa: ARG002
        run_type: str,  # noqa: ARG002
        schedule_id: str | None = None,  # noqa: ARG002
        schedule_name: str | None = None,  # noqa: ARG002
    ) -> str | None:
        return None

    async def add_messages(self, session_id: str, rows: Sequence[MessageRow]) -> None:
        return None

    async def import_csv(self, session_id: str, path: Path) -> int:
        return 0

    async def end_session(self, session_id: str, status: str) -> None:
        return None


class PgDriveRecorder:
    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def start_session(
        self,
        *,
        vehicle_id: str,
        mode_id: str | None,
        run_type: str,
        schedule_id: str | None = None,
        schedule_name: str | None = None,
    ) -> str | None:
        """schedule_id・schedule_name はスケジュールのジョブだけ（名前はその時点の写し。後で消えても残る）。"""
        session_id = str(uuid.uuid4())
        await self._pool.execute(
            """
            INSERT INTO drive_sessions (id, profile_id, mode_id, run_type, started_at, status, vehicle_id, schedule_id, schedule_name)
            VALUES ($1, NULL, $2, $3, NOW(), 'running', $4, $5, $6)
            """,
            uuid.UUID(session_id),
            uuid.UUID(mode_id) if mode_id else None,
            run_type,
            vehicle_id,
            _uuid_or_none(schedule_id),
            schedule_name or None,
        )
        return session_id

    async def add_messages(self, session_id: str, rows: Sequence[MessageRow]) -> None:
        if not rows:
            return
        sid = uuid.UUID(session_id)
        await self._pool.executemany(
            """
            INSERT INTO drive_session_messages (session_id, seq, elapsed_s, time, level, message)
            VALUES ($1, $2, $3, $4, $5, $6)
            """,
            [(sid, seq, elapsed, when, level, text) for seq, elapsed, when, level, text in rows],
        )

    async def import_csv(self, session_id: str, path: Path) -> int:
        """CSV を drive_logs へ入れ、入れた行数を返す。1 本のトランザクション（途中で失敗したら 0 行）。"""
        total = 0
        async with self._pool.acquire() as conn, conn.transaction():
            chunk: list[tuple[Any, ...]] = []
            for row in drive_csv.csv_to_db_rows(session_id, drive_csv.read_csv_rows(path)):
                chunk.append((uuid.UUID(session_id), *row[1:]))
                if len(chunk) >= IMPORT_CHUNK_ROWS:
                    await self._copy(conn, chunk)
                    total += len(chunk)
                    chunk = []
            if chunk:
                await self._copy(conn, chunk)
                total += len(chunk)
        return total

    @staticmethod
    async def _copy(conn: asyncpg.Connection, rows: list[tuple[Any, ...]]) -> None:
        await conn.copy_records_to_table(
            "drive_logs", records=rows, columns=list(drive_csv.DB_COLUMNS)
        )

    async def end_session(self, session_id: str, status: str) -> None:
        await self._pool.execute(
            "UPDATE drive_sessions SET ended_at = NOW(), status = $2 WHERE id = $1",
            uuid.UUID(session_id),
            status,
        )
