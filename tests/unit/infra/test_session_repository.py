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
