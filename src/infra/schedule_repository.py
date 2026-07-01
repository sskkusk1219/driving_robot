"""TimeSchedule を PostgreSQL へ永続化する。"""

from __future__ import annotations

import json
import uuid

import asyncpg

from src.infra.db import DuplicateNameError
from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule


def _pedal_points_to_json(points: list[PedalPoint]) -> str:
    return json.dumps(
        [
            {
                "time_s": p.time_s,
                "accel_opening": p.accel_opening,
                "brake_opening": p.brake_opening,
            }
            for p in points
        ]
    )


def _button_events_to_json(events: list[ButtonEvent]) -> str:
    return json.dumps(
        [
            {
                "time_s": e.time_s,
                "channel": e.channel,
                "press_duration_s": e.press_duration_s,
            }
            for e in events
        ]
    )


def _row_to_schedule(row: asyncpg.Record) -> TimeSchedule:
    pedal_raw = json.loads(row["pedal_points"])
    button_raw = json.loads(row["button_events"])
    return TimeSchedule(
        id=str(row["id"]),
        name=row["name"],
        description=row["description"],
        pedal_points=[
            PedalPoint(
                time_s=p["time_s"],
                accel_opening=p["accel_opening"],
                brake_opening=p["brake_opening"],
            )
            for p in pedal_raw
        ],
        button_events=[
            ButtonEvent(
                time_s=e["time_s"],
                channel=e["channel"],
                press_duration_s=e["press_duration_s"],
            )
            for e in button_raw
        ],
        total_duration=row["total_duration"],
        loop=row["loop"],
        created_at=row["created_at"],
    )


class ScheduleRepository:
    """time_schedules テーブルの CRUD を担うリポジトリ。"""

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def list_all(self) -> list[TimeSchedule]:
        """全タイムスケジュールを作成日時降順で返す。"""
        async with self._pool.acquire() as conn:
            rows = await conn.fetch("SELECT * FROM time_schedules ORDER BY created_at DESC")
        return [_row_to_schedule(row) for row in rows]

    async def get_by_id(self, schedule_id: str) -> TimeSchedule | None:
        """ID でタイムスケジュールを取得する。存在しない場合は None。"""
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM time_schedules WHERE id = $1",
                uuid.UUID(schedule_id),
            )
        if row is None:
            return None
        return _row_to_schedule(row)

    async def create(self, schedule: TimeSchedule) -> TimeSchedule:
        """タイムスケジュールを新規作成する。"""
        schedule_id = schedule.id if schedule.id else str(uuid.uuid4())
        async with self._pool.acquire() as conn:
            try:
                await conn.execute(
                    """
                    INSERT INTO time_schedules
                        (id, name, description, pedal_points, button_events,
                         total_duration, loop, created_at)
                    VALUES ($1, $2, $3, $4::jsonb, $5::jsonb, $6, $7, $8)
                    """,
                    uuid.UUID(schedule_id),
                    schedule.name,
                    schedule.description,
                    _pedal_points_to_json(schedule.pedal_points),
                    _button_events_to_json(schedule.button_events),
                    schedule.total_duration,
                    schedule.loop,
                    schedule.created_at,
                )
            except asyncpg.UniqueViolationError as e:
                raise DuplicateNameError(
                    f"スケジュール名 {schedule.name!r} は既に使用されています"
                ) from e
        return TimeSchedule(
            id=schedule_id,
            name=schedule.name,
            description=schedule.description,
            pedal_points=schedule.pedal_points,
            button_events=schedule.button_events,
            total_duration=schedule.total_duration,
            loop=schedule.loop,
            created_at=schedule.created_at,
        )

    async def update(self, schedule: TimeSchedule) -> TimeSchedule | None:
        """タイムスケジュールを更新する。存在しない場合は None。名称重複は DuplicateNameError。"""
        async with self._pool.acquire() as conn:
            try:
                result = await conn.execute(
                    """
                    UPDATE time_schedules
                    SET name = $1, description = $2, pedal_points = $3::jsonb,
                        button_events = $4::jsonb, total_duration = $5, loop = $6
                    WHERE id = $7
                    """,
                    schedule.name,
                    schedule.description,
                    _pedal_points_to_json(schedule.pedal_points),
                    _button_events_to_json(schedule.button_events),
                    schedule.total_duration,
                    schedule.loop,
                    uuid.UUID(schedule.id),
                )
            except asyncpg.UniqueViolationError as e:
                raise DuplicateNameError(
                    f"スケジュール名 {schedule.name!r} は既に使用されています"
                ) from e
        if str(result) == "UPDATE 0":
            return None
        return schedule

    async def delete(self, schedule_id: str) -> bool:
        """タイムスケジュールを削除する。削除できた場合 True。"""
        async with self._pool.acquire() as conn:
            result = await conn.execute(
                "DELETE FROM time_schedules WHERE id = $1",
                uuid.UUID(schedule_id),
            )
        return str(result) != "DELETE 0"
