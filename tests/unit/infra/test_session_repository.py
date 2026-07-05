"""SessionRepository のユニットテスト。"""

from contextlib import asynccontextmanager
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock
from uuid import UUID, uuid4

import pytest

from src.infra.session_repository import SessionRepository

SESSION_ID = str(uuid4())
SESSION_UUID = UUID(SESSION_ID)

_NOW = datetime.now(tz=UTC)


def make_session_row(**kwargs: object) -> MagicMock:
    defaults = {
        "id": SESSION_UUID,
        "profile_id": uuid4(),
        "mode_id": uuid4(),
        "run_type": "auto",
        "started_at": _NOW,
        "ended_at": None,
        "status": "completed",
        "cycle_id": None,
    }
    defaults.update(kwargs)
    row = MagicMock()
    row.__getitem__ = lambda self, key: defaults[key]
    return row


def make_cycle_row(**kwargs: object) -> MagicMock:
    defaults = {
        "id": uuid4(),
        "profile_id": uuid4(),
        "status": "completed",
        "started_at": _NOW,
        "ended_at": _NOW,
        "detail": "{}",
        "session_count": 3,
    }
    defaults.update(kwargs)
    row = MagicMock()
    row.__getitem__ = lambda self, key: defaults[key]
    return row


def make_log_row(**kwargs: object) -> MagicMock:
    defaults = {
        "id": 1,
        "session_id": SESSION_UUID,
        "timestamp": _NOW,
        "ref_speed_kmh": 60.0,
        "actual_speed_kmh": 58.5,
        "accel_opening": 30.0,
        "brake_opening": 0.0,
        "accel_pos": 1500,
        "brake_pos": 800,
        "accel_current": 1.2,
        "brake_current": 0.0,
    }
    defaults.update(kwargs)
    row = MagicMock()
    row.__getitem__ = lambda self, key: defaults[key]
    return row


def make_mock_pool() -> tuple[MagicMock, AsyncMock]:
    conn = AsyncMock()
    pool = MagicMock()

    @asynccontextmanager
    async def _acquire():
        yield conn

    pool.acquire = _acquire
    return pool, conn


class TestListAll:
    @pytest.mark.asyncio
    async def test_returns_empty_when_no_sessions(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = SessionRepository(pool)

        result = await repo.list_all()

        assert result == []

    @pytest.mark.asyncio
    async def test_returns_sessions_mapped_from_rows(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = [make_session_row()]
        repo = SessionRepository(pool)

        result = await repo.list_all()

        assert len(result) == 1
        assert result[0].id == SESSION_ID
        assert result[0].run_type == "auto"

    @pytest.mark.asyncio
    async def test_passes_limit_to_query(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = SessionRepository(pool)

        await repo.list_all(limit=50)

        args = conn.fetch.call_args[0]
        assert 50 in args


class TestGetById:
    @pytest.mark.asyncio
    async def test_returns_none_when_not_found(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = None
        repo = SessionRepository(pool)

        result = await repo.get_by_id(SESSION_ID)

        assert result is None

    @pytest.mark.asyncio
    async def test_returns_session_when_found(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = make_session_row()
        repo = SessionRepository(pool)

        result = await repo.get_by_id(SESSION_ID)

        assert result is not None
        assert result.id == SESSION_ID
        assert result.status == "completed"

    @pytest.mark.asyncio
    async def test_queries_with_uuid(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = None
        repo = SessionRepository(pool)

        await repo.get_by_id(SESSION_ID)

        args = conn.fetchrow.call_args[0]
        assert SESSION_UUID in args

    @pytest.mark.asyncio
    async def test_raises_value_error_for_invalid_uuid(self) -> None:
        pool, conn = make_mock_pool()
        repo = SessionRepository(pool)

        with pytest.raises(ValueError):
            await repo.get_by_id("not-a-uuid")


class TestListLogs:
    @pytest.mark.asyncio
    async def test_returns_empty_when_no_logs(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = SessionRepository(pool)

        result = await repo.list_logs(SESSION_ID)

        assert result == []

    @pytest.mark.asyncio
    async def test_returns_logs_mapped_from_rows(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = [make_log_row()]
        repo = SessionRepository(pool)

        result = await repo.list_logs(SESSION_ID)

        assert len(result) == 1
        assert result[0].session_id == SESSION_ID
        assert result[0].actual_speed_kmh == 58.5

    @pytest.mark.asyncio
    async def test_queries_with_session_uuid(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = SessionRepository(pool)

        await repo.list_logs(SESSION_ID)

        args = conn.fetch.call_args[0]
        assert SESSION_UUID in args

    @pytest.mark.asyncio
    async def test_mode_id_none_when_null(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = make_session_row(mode_id=None)
        repo = SessionRepository(pool)

        result = await repo.get_by_id(SESSION_ID)

        assert result is not None
        assert result.mode_id is None

    @pytest.mark.asyncio
    async def test_cycle_id_none_when_null(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = make_session_row(cycle_id=None)
        repo = SessionRepository(pool)

        result = await repo.get_by_id(SESSION_ID)

        assert result is not None
        assert result.cycle_id is None

    @pytest.mark.asyncio
    async def test_cycle_id_mapped_when_present(self) -> None:
        cycle_uuid = uuid4()
        pool, conn = make_mock_pool()
        conn.fetchrow.return_value = make_session_row(cycle_id=cycle_uuid)
        repo = SessionRepository(pool)

        result = await repo.get_by_id(SESSION_ID)

        assert result is not None
        assert result.cycle_id == str(cycle_uuid)


class TestListSessionIdsForCycle:
    @pytest.mark.asyncio
    async def test_returns_empty_when_no_sessions(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = SessionRepository(pool)

        result = await repo.list_session_ids_for_cycle(str(uuid4()))

        assert result == []

    @pytest.mark.asyncio
    async def test_returns_session_ids(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = [{"id": SESSION_UUID}]
        repo = SessionRepository(pool)

        result = await repo.list_session_ids_for_cycle(str(uuid4()))

        assert result == [SESSION_ID]


class TestListCycles:
    @pytest.mark.asyncio
    async def test_returns_empty_when_no_cycles(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = SessionRepository(pool)

        result = await repo.list_cycles()

        assert result == []

    @pytest.mark.asyncio
    async def test_returns_cycles_mapped_from_rows(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = [make_cycle_row()]
        repo = SessionRepository(pool)

        result = await repo.list_cycles()

        assert len(result) == 1
        assert result[0].status == "completed"
        assert result[0].session_count == 3
        assert result[0].detail == {}

    @pytest.mark.asyncio
    async def test_parses_dict_detail(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = [make_cycle_row(detail='{"stage1_gains": {"kp": 1.0}}')]
        repo = SessionRepository(pool)

        result = await repo.list_cycles()

        assert result[0].detail == {"stage1_gains": {"kp": 1.0}}

    @pytest.mark.asyncio
    async def test_filters_by_profile_id(self) -> None:
        pool, conn = make_mock_pool()
        conn.fetch.return_value = []
        repo = SessionRepository(pool)

        await repo.list_cycles(profile_id=SESSION_ID)

        args = conn.fetch.call_args[0]
        assert UUID(SESSION_ID) in args
