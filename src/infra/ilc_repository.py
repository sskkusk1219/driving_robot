"""反復学習制御（ILC）テーブルを PostgreSQL へ永続化する。

profile×mode 複合キーで時刻別補正 effort（ILCTable）と有効フラグ・KPI 履歴を保持する。
ILCLearner が走行後に更新した ILCTable を upsert し、走行開始時に get でロードする。
"""

from __future__ import annotations

import json
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from typing import Any

import asyncpg

from src.domain.control.ilc import ILC_DT_S, ILCTable


@dataclass
class ILCRecord:
    """ilc_tables の1行（ILCTable ＋ 永続化メタデータ）。"""

    profile_id: str
    mode_id: str
    enabled: bool
    table: ILCTable
    kpi_history: list[dict[str, Any]] = field(default_factory=list)
    updated_at: datetime | None = None


def _row_to_record(row: asyncpg.Record) -> ILCRecord:
    efforts_raw = json.loads(row["efforts"]) if isinstance(row["efforts"], str) else row["efforts"]
    history_raw = (
        json.loads(row["kpi_history"])
        if isinstance(row["kpi_history"], str)
        else row["kpi_history"]
    )
    table = ILCTable(
        efforts=[float(v) for v in efforts_raw],
        dt_s=float(row["dt_s"]),
        iteration=int(row["iteration"]),
        best_p95_kmh=(None if row["best_p95_kmh"] is None else float(row["best_p95_kmh"])),
    )
    return ILCRecord(
        profile_id=str(row["profile_id"]),
        mode_id=str(row["mode_id"]),
        enabled=bool(row["enabled"]),
        table=table,
        kpi_history=list(history_raw),
        updated_at=row["updated_at"],
    )


class ILCRepository:
    """ilc_tables の get / upsert / reset / set_enabled を担うリポジトリ。"""

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def get(self, profile_id: str, mode_id: str) -> ILCRecord | None:
        """profile×mode の ILC レコードを取得する。無ければ None。"""
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM ilc_tables WHERE profile_id = $1 AND mode_id = $2",
                uuid.UUID(profile_id),
                uuid.UUID(mode_id),
            )
        return None if row is None else _row_to_record(row)

    async def upsert(
        self,
        profile_id: str,
        mode_id: str,
        table: ILCTable,
        kpi_history: list[dict[str, Any]],
    ) -> None:
        """ILCTable と KPI 履歴を保存する。enabled は既存値を保持（挿入時は既定 TRUE）。"""
        async with self._pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO ilc_tables
                    (profile_id, mode_id, enabled, iteration, dt_s, efforts,
                     best_p95_kmh, kpi_history, updated_at)
                VALUES ($1, $2, TRUE, $3, $4, $5::jsonb, $6, $7::jsonb, $8)
                ON CONFLICT (profile_id, mode_id) DO UPDATE SET
                    iteration = EXCLUDED.iteration,
                    dt_s = EXCLUDED.dt_s,
                    efforts = EXCLUDED.efforts,
                    best_p95_kmh = EXCLUDED.best_p95_kmh,
                    kpi_history = EXCLUDED.kpi_history,
                    updated_at = EXCLUDED.updated_at
                """,
                uuid.UUID(profile_id),
                uuid.UUID(mode_id),
                table.iteration,
                table.dt_s,
                json.dumps(table.efforts),
                table.best_p95_kmh,
                json.dumps(kpi_history),
                datetime.now(tz=UTC),
            )

    async def reset(self, profile_id: str, mode_id: str) -> None:
        """補正テーブルを削除する（次回走行は補正なし＝反復 0 からやり直し）。"""
        async with self._pool.acquire() as conn:
            await conn.execute(
                "DELETE FROM ilc_tables WHERE profile_id = $1 AND mode_id = $2",
                uuid.UUID(profile_id),
                uuid.UUID(mode_id),
            )

    async def reset_for_mode(self, mode_id: str) -> None:
        """指定モードの全 profile の補正テーブルを削除する（モード編集時に呼ぶ）。"""
        async with self._pool.acquire() as conn:
            await conn.execute(
                "DELETE FROM ilc_tables WHERE mode_id = $1",
                uuid.UUID(mode_id),
            )

    async def set_enabled(self, profile_id: str, mode_id: str, enabled: bool) -> None:
        """ILC の有効/無効を設定する。行が無ければ空テーブルで作成して設定を永続化する。"""
        async with self._pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO ilc_tables
                    (profile_id, mode_id, enabled, iteration, dt_s, efforts,
                     best_p95_kmh, kpi_history, updated_at)
                VALUES ($1, $2, $3, 0, $4, '[]'::jsonb, NULL, '[]'::jsonb, $5)
                ON CONFLICT (profile_id, mode_id) DO UPDATE SET
                    enabled = EXCLUDED.enabled,
                    updated_at = EXCLUDED.updated_at
                """,
                uuid.UUID(profile_id),
                uuid.UUID(mode_id),
                enabled,
                ILC_DT_S,
                datetime.now(tz=UTC),
            )


__all__ = ["ILCRecord", "ILCRepository"]
