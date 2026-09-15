"""エピソード型プラン学習の保存プランを PostgreSQL へ永続化する。

profile×mode 複合キーで、次回走行に使う候補プラン（efforts＋phases）と、これまでの最良
reward を出した走行のプラン（best_efforts）・reward 履歴・生成に使った FF モデルパスを保持する。
PedalPlanService が走行後に更新したプランを upsert し、走行開始時に get でロードする。
ilc_repository.py（ILCRecord/ILCRepository）の後継。
"""

from __future__ import annotations

import json
import uuid
from dataclasses import dataclass, field
from datetime import UTC, datetime
from typing import Any

import asyncpg

from src.domain.control.pedal_plan import PedalPlan, PlanPhase


@dataclass
class PedalPlanRecord:
    """pedal_plans の1行（候補プラン＋最良プラン＋学習メタデータ）。"""

    profile_id: str
    mode_id: str
    enabled: bool
    plan: PedalPlan  # 次回走行に使う候補プラン
    iteration: int
    best_efforts: list[float]  # 最良 reward を出した走行のプラン（ロールバック先）
    best_reward: float | None
    reward_history: list[dict[str, Any]] = field(default_factory=list)
    model_path: str | None = None
    updated_at: datetime | None = None


def _loads(value: Any) -> Any:
    return json.loads(value) if isinstance(value, str) else value


def _row_to_record(row: asyncpg.Record) -> PedalPlanRecord:
    efforts = [float(v) for v in _loads(row["efforts"])]
    phases = [PlanPhase(str(v)) for v in _loads(row["phases"])]
    best_efforts = [float(v) for v in _loads(row["best_efforts"])]
    history = list(_loads(row["reward_history"]))
    plan = PedalPlan(dt_s=float(row["dt_s"]), efforts=efforts, phases=phases)
    return PedalPlanRecord(
        profile_id=str(row["profile_id"]),
        mode_id=str(row["mode_id"]),
        enabled=bool(row["enabled"]),
        plan=plan,
        iteration=int(row["iteration"]),
        best_efforts=best_efforts,
        best_reward=(None if row["best_reward"] is None else float(row["best_reward"])),
        reward_history=history,
        model_path=row["model_path"],
        updated_at=row["updated_at"],
    )


class PedalPlanRepository:
    """pedal_plans の get / upsert / reset / reset_for_mode / set_enabled を担うリポジトリ。"""

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def get(self, profile_id: str, mode_id: str) -> PedalPlanRecord | None:
        """profile×mode の保存プランを取得する。無ければ None。"""
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                "SELECT * FROM pedal_plans WHERE profile_id = $1 AND mode_id = $2",
                uuid.UUID(profile_id),
                uuid.UUID(mode_id),
            )
        return None if row is None else _row_to_record(row)

    async def upsert(
        self,
        profile_id: str,
        mode_id: str,
        plan: PedalPlan,
        *,
        iteration: int,
        best_efforts: list[float],
        best_reward: float | None,
        reward_history: list[dict[str, Any]],
        model_path: str | None,
    ) -> None:
        """候補プラン・最良プラン・reward 履歴を保存する。enabled は既存値を保持（挿入時 TRUE）。"""
        async with self._pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO pedal_plans
                    (profile_id, mode_id, enabled, iteration, dt_s, efforts, phases,
                     best_efforts, best_reward, reward_history, model_path, updated_at)
                VALUES ($1, $2, TRUE, $3, $4, $5::jsonb, $6::jsonb, $7::jsonb, $8,
                        $9::jsonb, $10, $11)
                ON CONFLICT (profile_id, mode_id) DO UPDATE SET
                    iteration = EXCLUDED.iteration,
                    dt_s = EXCLUDED.dt_s,
                    efforts = EXCLUDED.efforts,
                    phases = EXCLUDED.phases,
                    best_efforts = EXCLUDED.best_efforts,
                    best_reward = EXCLUDED.best_reward,
                    reward_history = EXCLUDED.reward_history,
                    model_path = EXCLUDED.model_path,
                    updated_at = EXCLUDED.updated_at
                """,
                uuid.UUID(profile_id),
                uuid.UUID(mode_id),
                iteration,
                plan.dt_s,
                json.dumps(list(plan.efforts)),
                json.dumps([p.value for p in plan.phases]),
                json.dumps(list(best_efforts)),
                best_reward,
                json.dumps(reward_history),
                model_path,
                datetime.now(tz=UTC),
            )

    async def reset(self, profile_id: str, mode_id: str) -> None:
        """保存プランを削除する（次回走行は FF 由来プランから学習をやり直す）。"""
        async with self._pool.acquire() as conn:
            await conn.execute(
                "DELETE FROM pedal_plans WHERE profile_id = $1 AND mode_id = $2",
                uuid.UUID(profile_id),
                uuid.UUID(mode_id),
            )

    async def reset_for_mode(self, mode_id: str) -> None:
        """指定モードの全 profile の保存プランを削除する（モード基準軌跡編集時に呼ぶ）。"""
        async with self._pool.acquire() as conn:
            await conn.execute(
                "DELETE FROM pedal_plans WHERE mode_id = $1",
                uuid.UUID(mode_id),
            )

    async def set_enabled(self, profile_id: str, mode_id: str, enabled: bool) -> None:
        """プラン学習の有効/無効を設定する。行が無ければ空プランで作成して設定を永続化する。"""
        async with self._pool.acquire() as conn:
            await conn.execute(
                """
                INSERT INTO pedal_plans
                    (profile_id, mode_id, enabled, iteration, dt_s, efforts, phases,
                     best_efforts, best_reward, reward_history, model_path, updated_at)
                VALUES ($1, $2, $3, 0, $4, '[]'::jsonb, '[]'::jsonb, '[]'::jsonb, NULL,
                        '[]'::jsonb, NULL, $5)
                ON CONFLICT (profile_id, mode_id) DO UPDATE SET
                    enabled = EXCLUDED.enabled,
                    updated_at = EXCLUDED.updated_at
                """,
                uuid.UUID(profile_id),
                uuid.UUID(mode_id),
                enabled,
                PedalPlan().dt_s,
                datetime.now(tz=UTC),
            )


__all__ = ["PedalPlanRecord", "PedalPlanRepository"]
