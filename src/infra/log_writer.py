"""100ms 周期で PostgreSQL に走行ログを書き込むインフラコンポーネント。"""

import uuid

import asyncpg

from src.models.drive_log import DriveLogData


class LogWriter:
    """走行セッションとログデータを PostgreSQL に永続化する。

    asyncpg.Connection を外部から受け取る設計（接続プール管理は呼び出し元が担当）。
    """

    def __init__(self, conn: asyncpg.Connection) -> None:
        self._conn = conn

    async def start_session(
        self,
        profile_id: str,
        mode_id: str | None,
        run_type: str,
    ) -> str:
        """drive_sessions に INSERT し、生成したセッション ID (UUID 文字列) を返す。

        Args:
            profile_id: 使用する車両プロファイルの UUID 文字列
            mode_id: 走行モードの UUID 文字列（手動運転・学習運転時は None）
            run_type: 'auto' | 'manual' | 'learning'
        """
        session_id = str(uuid.uuid4())
        await self._conn.execute(
            """
            INSERT INTO drive_sessions
                (id, profile_id, mode_id, run_type, started_at, status)
            VALUES
                ($1, $2, $3, $4, NOW(), 'running')
            """,
            session_id,
            profile_id,
            mode_id,
            run_type,
        )
        return session_id

    async def write_log(self, session_id: str, data: DriveLogData) -> None:
        """drive_logs に 1 レコードを INSERT する。timestamp は DB 側 NOW() を使用。

        100ms 周期で呼ばれることを前提とし、5ms 以内の完了を目標とする。
        """
        await self._conn.execute(
            """
            INSERT INTO drive_logs
                (session_id, timestamp, ref_speed_kmh, actual_speed_kmh,
                 accel_opening, brake_opening, accel_pos, brake_pos,
                 accel_current, brake_current)
            VALUES
                ($1, NOW(), $2, $3, $4, $5, $6, $7, $8, $9)
            """,
            session_id,
            data.ref_speed_kmh,
            data.actual_speed_kmh,
            data.accel_opening,
            data.brake_opening,
            data.accel_pos,
            data.brake_pos,
            data.accel_current,
            data.brake_current,
        )

    async def end_session(self, session_id: str, status: str) -> None:
        """drive_sessions.ended_at と status を UPDATE してセッションを終了する。

        Args:
            session_id: 終了するセッションの UUID 文字列
            status: 'completed' | 'error' | 'emergency'
        """
        await self._conn.execute(
            """
            UPDATE drive_sessions
            SET ended_at = NOW(), status = $2
            WHERE id = $1
            """,
            session_id,
            status,
        )
