"""DrivingMode を PostgreSQL へ永続化する。"""

from __future__ import annotations

import json
import uuid

import asyncpg

from src.models.driving_mode import DrivingMode, SpeedPoint


def _row_to_mode(row: asyncpg.Record) -> DrivingMode:
    ref_speed_raw = json.loads(row["reference_speed"])
    ref_speed = [SpeedPoint(time_s=p["time_s"], speed_kmh=p["speed_kmh"]) for p in ref_speed_raw]
    return DrivingMode(
        id=str(row["id"]),
        name=row["name"],
        description=row["description"],
        reference_speed=ref_speed,
        total_duration=row["total_duration"],
        max_speed=row["max_speed"],
        created_at=row["created_at"],
    )


class ModeRepository:
    """driving_modes テーブルの CRUD を担うリポジトリ。"""

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def list_all(self) -> list[DrivingMode]:
        """全走行モードを作成日時降順で返す。"""
        async with self._pool.acquire() as conn:
            rows = await conn.fetch(
                "SELECT * FROM driving_modes ORDER BY created_at DESC"
            )
        return [_row_to_mode(row) for row in rows]

    async def get_by_id(self, mode_id: str) -> DrivingMode | None:
        """ID で走行モードを取得する。存在しない場合は None。"""
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM driving_modes WHERE id = $1",
                uuid.UUID(mode_id),
            )
        if row is None:
            return None
        return _row_to_mode(row)

    async def create(self, mode: DrivingMode) -> DrivingMode:
        """走行モードを新規作成する。"""
        mode_id = mode.id if mode.id else str(uuid.uuid4())
        ref_speed_json = json.dumps(
            [{"time_s": p.time_s, "speed_kmh": p.speed_kmh} for p in mode.reference_speed]
        )
        async with self._pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO driving_modes
                    (id, name, description, reference_speed, total_duration, max_speed, created_at)
                VALUES ($1, $2, $3, $4::jsonb, $5, $6, $7)
                """,
                uuid.UUID(mode_id),
                mode.name,
                mode.description,
                ref_speed_json,
                mode.total_duration,
                mode.max_speed,
                mode.created_at,
            )
        return DrivingMode(
            id=mode_id,
            name=mode.name,
            description=mode.description,
            reference_speed=mode.reference_speed,
            total_duration=mode.total_duration,
            max_speed=mode.max_speed,
            created_at=mode.created_at,
        )

    async def delete(self, mode_id: str) -> bool:
        """走行モードを削除する。削除できた場合 True。"""
        async with self._pool.acquire() as conn:
            result = await conn.execute(
                "DELETE FROM driving_modes WHERE id = $1",
                uuid.UUID(mode_id),
            )
        return str(result) != "DELETE 0"
