"""ModeRepository のユニットテスト。"""

from contextlib import asynccontextmanager
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock
from uuid import UUID, uuid4

import asyncpg
import pytest

from src.infra.db import DuplicateNameError
from src.infra.mode_repository import RESERVED_SYSTEM_MODE_NAME, ModeRepository
from src.models.driving_mode import DrivingMode, SpeedPoint


def make_row(*, name: str = "M1", is_system: bool = False) -> dict:
    """asyncpg.Record 相当（dict でも _row_to_mode は動作する）。"""
    return {
        "id": MODE_UUID,
        "name": name,
        "description": "",
        "reference_speed": '[{"time_s": 0.0, "speed_kmh": 0.0}]',
        "total_duration": 10.0,
        "max_speed": 60.0,
        "created_at": datetime.now(tz=UTC),
        "is_system": is_system,
    }

MODE_ID = str(uuid4())
MODE_UUID = UUID(MODE_ID)


def make_mode(name: str = "WLTP_Class3", mode_id: str = "") -> DrivingMode:
    points = [SpeedPoint(time_s=0.0, speed_kmh=0.0), SpeedPoint(time_s=10.0, speed_kmh=60.0)]
    return DrivingMode(
        id=mode_id or str(uuid4()),
        name=name,
        description="テスト走行モード",
        reference_speed=points,
        total_duration=10.0,
        max_speed=60.0,
        created_at=datetime.now(tz=UTC),
    )


def make_mock_pool() -> tuple[MagicMock, AsyncMock]:
    conn = AsyncMock()
    pool = MagicMock()

    @asynccontextmanager
    async def _acquire():
        yield conn

    pool.acquire = _acquire
    return pool, conn


class TestModeRepositoryCreate:
    @pytest.mark.asyncio
    async def test_create_executes_insert(self) -> None:
        """create が conn.execute を1回呼ぶこと。"""
        pool, conn = make_mock_pool()
        repo = ModeRepository(pool)
        mode = make_mode()

        result = await repo.create(mode)

        conn.execute.assert_awaited_once()
        assert result.name == "WLTP_Class3"

    @pytest.mark.asyncio
    async def test_create_assigns_id_when_empty(self) -> None:
        """mode.id が空の場合、新しい id が割り当てられること。"""
        pool, conn = make_mock_pool()
        repo = ModeRepository(pool)
        mode = make_mode(mode_id="")

        result = await repo.create(mode)

        assert result.id != ""


class TestModeRepositoryUpdate:
    @pytest.mark.asyncio
    async def test_update_converts_unique_violation_to_duplicate_name_error(self) -> None:
        """I5 回帰テスト: update 時の一意制約違反を DuplicateNameError（→409）に変換する
        （create は既に変換済みだが update だけ未変換で 500 になっていたバグの回帰）。"""
        pool, conn = make_mock_pool()
        conn.execute.side_effect = asyncpg.UniqueViolationError("duplicate key")
        repo = ModeRepository(pool)
        mode = make_mode(name="既存モード名", mode_id=MODE_ID)

        with pytest.raises(DuplicateNameError):
            await repo.update(mode)


class TestModeRepositoryDelete:
    @pytest.mark.asyncio
    async def test_delete_returns_true_when_deleted(self) -> None:
        pool, conn = make_mock_pool()
        conn.execute.return_value = "DELETE 1"
        repo = ModeRepository(pool)

        result = await repo.delete(MODE_ID)

        assert result is True
        conn.execute.assert_awaited_once_with(
            "DELETE FROM driving_modes WHERE id = $1",
            MODE_UUID,
        )

    @pytest.mark.asyncio
    async def test_delete_returns_false_when_not_found(self) -> None:
        pool, conn = make_mock_pool()
        conn.execute.return_value = "DELETE 0"
        repo = ModeRepository(pool)

        result = await repo.delete(MODE_ID)

        assert result is False


class TestModeRepositoryGetById:
    @pytest.mark.asyncio
    async def test_get_by_id_returns_none_when_not_found(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = None
        repo = ModeRepository(pool)

        result = await repo.get_by_id(MODE_ID)

        assert result is None

    @pytest.mark.asyncio
    async def test_get_by_id_queries_with_uuid(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = None
        repo = ModeRepository(pool)

        await repo.get_by_id(MODE_ID)

        conn.fetchrow.assert_awaited_once()
        args = conn.fetchrow.call_args[0]
        assert args[1] == MODE_UUID


class TestModeRepositoryListAll:
    @pytest.mark.asyncio
    async def test_list_all_returns_empty_when_no_modes(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = ModeRepository(pool)

        result = await repo.list_all()

        assert result == []

    @pytest.mark.asyncio
    async def test_list_all_excludes_system_modes(self) -> None:
        """list_all はシステムモードを除外するクエリを発行する。"""
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = ModeRepository(pool)

        await repo.list_all()

        sql = conn.fetch.call_args[0][0]
        assert "is_system = FALSE" in sql


class TestModeRepositorySystemMode:
    @pytest.mark.asyncio
    async def test_get_by_id_reads_is_system(self) -> None:
        """get_by_id はフィルタせず is_system を DrivingMode に反映する。"""
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = make_row(name="__verify_pattern__", is_system=True)
        repo = ModeRepository(pool)

        result = await repo.get_by_id(MODE_ID)

        assert result is not None
        assert result.is_system is True
        # get_by_id は is_system でフィルタしない（システムモードも取得可）
        assert "is_system" not in conn.fetchrow.call_args[0][0]

    @pytest.mark.asyncio
    async def test_upsert_system_mode_forces_reserved_name_and_flag(self) -> None:
        """upsert_system_mode は予約名・is_system=TRUE を強制し、返り行を DrivingMode 化する。"""
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = make_row(name=RESERVED_SYSTEM_MODE_NAME, is_system=True)
        repo = ModeRepository(pool)
        # 呼び出し側の name は無視され、予約名で永続化される
        mode = make_mode(name="ignored")

        result = await repo.upsert_system_mode(mode)

        assert result.name == RESERVED_SYSTEM_MODE_NAME
        assert result.is_system is True
        sql, *params = conn.fetchrow.call_args[0]
        assert "ON CONFLICT (name) DO UPDATE" in sql
        assert RESERVED_SYSTEM_MODE_NAME in params  # name パラメータは予約名

    @pytest.mark.asyncio
    async def test_upsert_system_mode_accepts_non_uuid_synthetic_id(self) -> None:
        """回帰: build_verification_trajectory が渡す合成 id（"verify" 等・非UUID）でも
        UUID パースで落ちず、INSERT 候補 id は新規採番される（id は ON CONFLICT が保持）。"""
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = make_row(name=RESERVED_SYSTEM_MODE_NAME, is_system=True)
        repo = ModeRepository(pool)
        mode = make_mode(name="verify", mode_id="verify")  # ← 非UUID id

        result = await repo.upsert_system_mode(mode)  # 例外を送出しない

        assert result.is_system is True
        # 渡した候補 id は新規 UUID（合成 "verify" ではない）
        candidate_id = conn.fetchrow.call_args[0][1]
        assert isinstance(candidate_id, UUID)
