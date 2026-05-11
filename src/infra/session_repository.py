"""DriveSession・DriveLog を PostgreSQL から読み取る読み取り専用リポジトリ。"""

from __future__ import annotations

import uuid

import asyncpg

from src.models.drive_log import DriveLog, DriveSession


class SessionRepository:
    """drive_sessions / drive_logs の読み取りを担うリポジトリ。"""

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def list_all(self, limit: int = 100) -> list[DriveSession]:
        """走行セッション一覧を開始日時降順で返す。"""
        async with self._pool.acquire() as conn:
            rows = await conn.fetch(
                "SELECT * FROM drive_sessions ORDER BY started_at DESC LIMIT $1",
                limit,
            )
        return [_row_to_session(row) for row in rows]

    async def get_by_id(self, session_id: str) -> DriveSession | None:
        """ID でセッションを取得する。存在しない場合は None。"""
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM drive_sessions WHERE id = $1",
                uuid.UUID(session_id),
            )
        if row is None:
            return None
        return _row_to_session(row)

    async def list_logs(self, session_id: str, limit: int = 1000) -> list[DriveLog]:
        """セッションに紐づくログを時刻昇順で返す。"""
        async with self._pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT * FROM drive_logs
                WHERE session_id = $1
                ORDER BY timestamp ASC
                LIMIT $2
                """,
                uuid.UUID(session_id),
                limit,
            )
        return [_row_to_log(row) for row in rows]


def _row_to_session(row: asyncpg.Record) -> DriveSession:
    return DriveSession(
        id=str(row["id"]),
        profile_id=str(row["profile_id"]),
        mode_id=str(row["mode_id"]) if row["mode_id"] is not None else None,
        run_type=row["run_type"],
        started_at=row["started_at"],
        ended_at=row["ended_at"],
        status=row["status"],
    )


def _row_to_log(row: asyncpg.Record) -> DriveLog:
    return DriveLog(
        id=row["id"],
        session_id=str(row["session_id"]),
        timestamp=row["timestamp"],
        ref_speed_kmh=row["ref_speed_kmh"],
        actual_speed_kmh=row["actual_speed_kmh"],
        accel_opening=row["accel_opening"],
        brake_opening=row["brake_opening"],
        accel_pos=row["accel_pos"],
        brake_pos=row["brake_pos"],
        accel_current=row["accel_current"],
        brake_current=row["brake_current"],
    )
