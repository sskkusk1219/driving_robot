"""Sequence を PostgreSQL へ永続化する。"""

from __future__ import annotations

import json
import uuid
from typing import Literal, cast

import asyncpg

from src.infra.db import DuplicateNameError
from src.models.sequence import Sequence, SequenceStep


def _steps_to_json(steps: list[SequenceStep]) -> str:
    return json.dumps([{"kind": s.kind, "ref_id": s.ref_id, "repeat": s.repeat} for s in steps])


def _row_to_sequence(row: asyncpg.Record) -> Sequence:
    raw = json.loads(row["steps"])
    return Sequence(
        id=str(row["id"]),
        name=row["name"],
        description=row["description"],
        steps=[
            SequenceStep(
                kind=cast(Literal["mode", "schedule"], s["kind"]),
                ref_id=s["ref_id"],
                repeat=s["repeat"],
            )
            for s in raw
        ],
        repeat=row["repeat"],
        created_at=row["created_at"],
    )


class SequenceRepository:
    """sequences テーブルの CRUD を担うリポジトリ。"""

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def list_all(self) -> list[Sequence]:
        """全シーケンスを作成日時降順で返す。"""
        async with self._pool.acquire() as conn:
            rows = await conn.fetch("SELECT * FROM sequences ORDER BY created_at DESC")
        return [_row_to_sequence(row) for row in rows]

    async def get_by_id(self, sequence_id: str) -> Sequence | None:
        """ID でシーケンスを取得する。存在しない場合は None。"""
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM sequences WHERE id = $1", uuid.UUID(sequence_id)
            )
        if row is None:
            return None
        return _row_to_sequence(row)

    async def create(self, sequence: Sequence) -> Sequence:
        """シーケンスを新規作成する。名称重複は DuplicateNameError。"""
        sequence_id = sequence.id if sequence.id else str(uuid.uuid4())
        async with self._pool.acquire() as conn:
            try:
                await conn.execute(
                    """
                    INSERT INTO sequences (id, name, description, steps, repeat, created_at)
                    VALUES ($1, $2, $3, $4::jsonb, $5, $6)
                    """,
                    uuid.UUID(sequence_id),
                    sequence.name,
                    sequence.description,
                    _steps_to_json(sequence.steps),
                    sequence.repeat,
                    sequence.created_at,
                )
            except asyncpg.UniqueViolationError as e:
                raise DuplicateNameError(
                    f"シーケンス名 {sequence.name!r} は既に使用されています"
                ) from e
        return Sequence(
            id=sequence_id,
            name=sequence.name,
            description=sequence.description,
            steps=sequence.steps,
            repeat=sequence.repeat,
            created_at=sequence.created_at,
        )

    async def update(self, sequence: Sequence) -> Sequence | None:
        """シーケンスを更新する。存在しない場合は None。名称重複は DuplicateNameError。"""
        async with self._pool.acquire() as conn:
            try:
                result = await conn.execute(
                    """
                    UPDATE sequences
                    SET name = $1, description = $2, steps = $3::jsonb, repeat = $4
                    WHERE id = $5
                    """,
                    sequence.name,
                    sequence.description,
                    _steps_to_json(sequence.steps),
                    sequence.repeat,
                    uuid.UUID(sequence.id),
                )
            except asyncpg.UniqueViolationError as e:
                raise DuplicateNameError(
                    f"シーケンス名 {sequence.name!r} は既に使用されています"
                ) from e
        if str(result) == "UPDATE 0":
            return None
        return sequence

    async def delete(self, sequence_id: str) -> bool:
        """シーケンスを削除する。削除できた場合 True。"""
        async with self._pool.acquire() as conn:
            result = await conn.execute(
                "DELETE FROM sequences WHERE id = $1", uuid.UUID(sequence_id)
            )
        return str(result) != "DELETE 0"
