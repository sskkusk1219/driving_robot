"""PedalPlanRepository のユニットテスト（asyncpg 接続をモック）。ilc_repository の後継。"""

from contextlib import asynccontextmanager
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock
from uuid import uuid4

import pytest

from src.domain.control.pedal_plan import PedalPlan, PlanPhase
from src.infra.pedal_plan_repository import PedalPlanRepository, _row_to_record

PROFILE_ID = str(uuid4())
MODE_ID = str(uuid4())


def make_mock_pool() -> tuple[MagicMock, AsyncMock]:
    conn = AsyncMock()
    pool = MagicMock()

    @asynccontextmanager
    async def _acquire():  # type: ignore[no-untyped-def]
        yield conn

    pool.acquire = _acquire
    return pool, conn


def make_row() -> dict[str, object]:
    return {
        "profile_id": PROFILE_ID,
        "mode_id": MODE_ID,
        "enabled": True,
        "iteration": 3,
        "dt_s": 0.1,
        "efforts": "[0.0, 1.5, -2.0]",
        "phases": '["drive", "coast", "brake"]',
        "best_efforts": "[0.0, 1.4, -2.1]",
        "best_reward": -0.42,
        "reward_history": '[{"reward": -0.5}, {"reward": -0.42}]',
        "model_path": "/models/x.pkl",
        "updated_at": datetime.now(tz=UTC),
    }


class TestRowToRecord:
    def test_parses_jsonb_strings(self) -> None:
        rec = _row_to_record(make_row())  # type: ignore[arg-type]
        assert rec.enabled is True
        assert rec.iteration == 3
        assert rec.plan.efforts == [0.0, 1.5, -2.0]
        assert rec.plan.phases == [PlanPhase.DRIVE, PlanPhase.COAST, PlanPhase.BRAKE]
        assert rec.best_efforts == [0.0, 1.4, -2.1]
        assert rec.best_reward == pytest.approx(-0.42)
        assert rec.reward_history == [{"reward": -0.5}, {"reward": -0.42}]
        assert rec.model_path == "/models/x.pkl"

    def test_parses_native_jsonb(self) -> None:
        row = make_row()
        row["efforts"] = [1.0, 2.0]  # asyncpg が list を返すケース
        row["phases"] = ["drive", "drive"]
        row["best_efforts"] = [1.0, 2.0]
        row["reward_history"] = []
        row["best_reward"] = None
        rec = _row_to_record(row)  # type: ignore[arg-type]
        assert rec.plan.efforts == [1.0, 2.0]
        assert rec.best_reward is None
        assert rec.reward_history == []


class TestGet:
    @pytest.mark.asyncio
    async def test_returns_none_when_missing(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = None
        repo = PedalPlanRepository(pool)
        assert await repo.get(PROFILE_ID, MODE_ID) is None

    @pytest.mark.asyncio
    async def test_returns_record(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = make_row()
        repo = PedalPlanRepository(pool)
        rec = await repo.get(PROFILE_ID, MODE_ID)
        assert rec is not None
        assert rec.iteration == 3


class TestUpsert:
    @pytest.mark.asyncio
    async def test_executes_insert_on_conflict(self) -> None:
        pool, conn = make_mock_pool()
        repo = PedalPlanRepository(pool)
        plan = PedalPlan(dt_s=0.1, efforts=[0.0, 1.0], phases=[PlanPhase.DRIVE, PlanPhase.DRIVE])
        await repo.upsert(
            PROFILE_ID,
            MODE_ID,
            plan,
            iteration=2,
            best_efforts=[0.0, 1.0],
            best_reward=-0.19,
            reward_history=[{"reward": -0.19}],
            model_path="/models/x.pkl",
        )
        conn.execute.assert_awaited_once()
        sql = conn.execute.await_args.args[0]
        assert "ON CONFLICT" in sql
        # enabled は更新句に含めない（既存値を保持）
        assert "enabled = EXCLUDED.enabled" not in sql


class TestResetAndEnable:
    @pytest.mark.asyncio
    async def test_reset_deletes_row(self) -> None:
        pool, conn = make_mock_pool()
        repo = PedalPlanRepository(pool)
        await repo.reset(PROFILE_ID, MODE_ID)
        sql = conn.execute.await_args.args[0]
        assert "DELETE FROM pedal_plans" in sql
        assert "profile_id" in sql and "mode_id" in sql

    @pytest.mark.asyncio
    async def test_reset_for_mode_deletes_all_profiles(self) -> None:
        pool, conn = make_mock_pool()
        repo = PedalPlanRepository(pool)
        await repo.reset_for_mode(MODE_ID)
        sql = conn.execute.await_args.args[0]
        assert "DELETE FROM pedal_plans WHERE mode_id" in sql

    @pytest.mark.asyncio
    async def test_set_enabled_upserts_flag(self) -> None:
        pool, conn = make_mock_pool()
        repo = PedalPlanRepository(pool)
        await repo.set_enabled(PROFILE_ID, MODE_ID, False)
        sql = conn.execute.await_args.args[0]
        assert "enabled = EXCLUDED.enabled" in sql
        assert conn.execute.await_args.args[3] is False
