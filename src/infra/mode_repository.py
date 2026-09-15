"""DrivingMode を PostgreSQL へ永続化する。"""

from __future__ import annotations

import json
import uuid

import asyncpg

from src.infra.db import DuplicateNameError
from src.models.driving_mode import DrivingMode, SpeedPoint

# 学習サイクルが内部生成する網羅検証パターン（システムモード）の予約名。この名前の行は
# is_system=TRUE で永続化し、ユーザー向け一覧・編集から隔離する。名前が一意制約のため、
# ON CONFLICT (name) でサイクル毎に同一 id を保ったまま軌跡だけ更新できる。
RESERVED_SYSTEM_MODE_NAME = "__verify_pattern__"


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
        is_system=bool(row["is_system"]),
    )


class ModeRepository:
    """driving_modes テーブルの CRUD を担うリポジトリ。"""

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def list_all(self) -> list[DrivingMode]:
        """ユーザー登録の走行モードを作成日時降順で返す（システムモードは除外）。

        システムモード（網羅検証パターン）を含めると、WebUI 一覧・自動運転のモード選択に
        現れ、さらに検証パターン生成（build_verification_trajectory）の入力に自己参照が
        混入してしまう。ユーザー向け列挙は常に is_system=FALSE に限定する。
        """
        async with self._pool.acquire() as conn:
            rows = await conn.fetch(
                "SELECT * FROM driving_modes WHERE is_system = FALSE ORDER BY created_at DESC"
            )
        return [_row_to_mode(row) for row in rows]

    async def get_by_id(self, mode_id: str) -> DrivingMode | None:
        """ID で走行モードを取得する（システムモードも取得可）。存在しない場合は None。

        システムモードもプラン学習フェーズ・ログ画面のモード名解決で参照するため、
        list_all と異なりフィルタしない。
        """
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM driving_modes WHERE id = $1",
                uuid.UUID(mode_id),
            )
        if row is None:
            return None
        return _row_to_mode(row)

    async def get_system_mode(self) -> DrivingMode | None:
        """システムモード（網羅検証パターン）を予約名で取得する（無ければ None）。

        upsert_system_mode の前に呼び、軌跡が前世代から変わるかを判定する
        （変わる場合は旧軌跡で学習した保存プランをリセットする。learning_cycle 参照）。
        """
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM driving_modes WHERE name = $1 AND is_system = TRUE",
                RESERVED_SYSTEM_MODE_NAME,
            )
        if row is None:
            return None
        return _row_to_mode(row)

    async def upsert_system_mode(self, mode: DrivingMode) -> DrivingMode:
        """システムモード（網羅検証パターン）を予約名で INSERT または UPDATE する。

        名前を予約名 `__verify_pattern__` に固定し、ON CONFLICT (name) で既存行の id を
        保ったまま軌跡（reference_speed / total_duration / max_speed）だけ更新する。id が
        安定するため pedal_plans の FK（profile×mode）が世代をまたいで有効に保たれる。
        戻り値は永続化された（＝安定 id を持つ）DrivingMode。
        """
        ref_speed_json = json.dumps(
            [{"time_s": p.time_s, "speed_kmh": p.speed_kmh} for p in mode.reference_speed]
        )
        # INSERT 用の候補 id は常に新規採番する。build_verification_trajectory が渡す mode.id は
        # 合成値（"verify" など非 UUID）で DB の実 id ではないため、UUID パースしてはならない。
        # 既存行があれば ON CONFLICT (name) が既存 id を保持し RETURNING で返る（id 安定）。
        new_id = uuid.uuid4()
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                INSERT INTO driving_modes
                    (id, name, description, reference_speed,
                     total_duration, max_speed, created_at, is_system)
                VALUES ($1, $2, $3, $4::jsonb, $5, $6, $7, TRUE)
                ON CONFLICT (name) DO UPDATE SET
                    description = EXCLUDED.description,
                    reference_speed = EXCLUDED.reference_speed,
                    total_duration = EXCLUDED.total_duration,
                    max_speed = EXCLUDED.max_speed,
                    is_system = TRUE
                RETURNING *
                """,
                new_id,
                RESERVED_SYSTEM_MODE_NAME,
                mode.description,
                ref_speed_json,
                mode.total_duration,
                mode.max_speed,
                mode.created_at,
            )
        assert row is not None  # INSERT ... RETURNING は常に 1 行返す
        return _row_to_mode(row)

    async def create(self, mode: DrivingMode) -> DrivingMode:
        """走行モードを新規作成する。"""
        mode_id = mode.id if mode.id else str(uuid.uuid4())
        ref_speed_json = json.dumps(
            [{"time_s": p.time_s, "speed_kmh": p.speed_kmh} for p in mode.reference_speed]
        )
        async with self._pool.acquire() as conn:
            try:
                await conn.execute(
                    """
                    INSERT INTO driving_modes
                        (id, name, description, reference_speed,
                         total_duration, max_speed, created_at)
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
            except asyncpg.UniqueViolationError as e:
                raise DuplicateNameError(
                    f"走行モード名 {mode.name!r} は既に使用されています"
                ) from e
        return DrivingMode(
            id=mode_id,
            name=mode.name,
            description=mode.description,
            reference_speed=mode.reference_speed,
            total_duration=mode.total_duration,
            max_speed=mode.max_speed,
            created_at=mode.created_at,
        )

    async def update(self, mode: DrivingMode) -> DrivingMode | None:
        """走行モードを更新する。存在しない場合は None。"""
        ref_speed_json = json.dumps(
            [{"time_s": p.time_s, "speed_kmh": p.speed_kmh} for p in mode.reference_speed]
        )
        async with self._pool.acquire() as conn:
            try:
                result = await conn.execute(
                    """
                    UPDATE driving_modes
                    SET name = $1, description = $2, reference_speed = $3::jsonb,
                        total_duration = $4, max_speed = $5
                    WHERE id = $6
                    """,
                    mode.name,
                    mode.description,
                    ref_speed_json,
                    mode.total_duration,
                    mode.max_speed,
                    uuid.UUID(mode.id),
                )
            except asyncpg.UniqueViolationError as e:
                # create と同じ変換（I5 レビュー指摘: update だけ未変換で 500 になっていた）
                raise DuplicateNameError(
                    f"走行モード名 {mode.name!r} は既に使用されています"
                ) from e
        if str(result) == "UPDATE 0":
            return None
        return mode

    async def delete(self, mode_id: str) -> bool:
        """走行モードを削除する。削除できた場合 True。"""
        async with self._pool.acquire() as conn:
            result = await conn.execute(
                "DELETE FROM driving_modes WHERE id = $1",
                uuid.UUID(mode_id),
            )
        return str(result) != "DELETE 0"
