# ruff: noqa: E501
"""ErrorRepository ↔ PostgreSQL 統合テスト（error_events）。

実行には TEST_DATABASE_URL 環境変数が必要（本番 DB は使わない）。CI 対象外 (pytest.mark.integration)。
接続できない環境ではスキップする。

  export TEST_DATABASE_URL=postgresql://localhost/driving_robot_test
  pytest tests/integration/test_error_repository_db.py -v -m integration
"""

import os
from collections.abc import AsyncGenerator
from datetime import UTC, datetime, timedelta

import asyncpg
import pytest

from src.infra.error_repository import ErrorRepository
from src.models.error_event import NewErrorEvent

TEST_DATABASE_URL = os.environ.get("TEST_DATABASE_URL", "postgresql://localhost/driving_robot_test")

pytestmark = pytest.mark.integration

T0 = datetime(2026, 11, 20, 9, 0, 0, tzinfo=UTC)


@pytest.fixture
async def repo() -> AsyncGenerator[ErrorRepository]:
    try:
        pool = await asyncpg.create_pool(TEST_DATABASE_URL, min_size=1, max_size=2)
    except (OSError, asyncpg.PostgresError) as exc:
        pytest.skip(f"テスト用 DB に接続できません: {exc}")
    assert pool is not None
    r = ErrorRepository(pool)
    await r.ensure_schema()
    await r.ensure_schema()  # 冪等
    async with pool.acquire() as conn:
        await conn.execute("DELETE FROM error_events")
    yield r
    async with pool.acquire() as conn:
        await conn.execute("DELETE FROM error_events")
    await pool.close()


def new(title: str, level: str = "ERROR", source: str = "job", **kw: object) -> NewErrorEvent:
    return NewErrorEvent(level=level, source=source, title=title, detail="本文", **kw)  # type: ignore[arg-type]


async def test_insert_get_list_filter_and_order(repo: ErrorRepository) -> None:
    a = await repo.insert(new("a", context={"messages": [{"text": "x"}]}, session_id="S", job_id="j"), at=T0)
    b = await repo.insert(new("b", "WARNING", "idle"), at=T0 + timedelta(minutes=1))
    e = await repo.get(a)
    assert e is not None and e.context == {"messages": [{"text": "x"}]} and e.count == 1 and e.session_id == "S"
    items, total = await repo.list_events()
    assert [i.id for i in items] == [b, a] and total == 2
    assert [i.id for i in (await repo.list_events(level="WARNING"))[0]] == [b]
    assert [i.id for i in (await repo.list_events(source="job"))[0]] == [a]
    page, total = await repo.list_events(limit=1, offset=1)
    assert [i.id for i in page] == [a] and total == 2


async def test_bump_append_ack_delete(repo: ErrorRepository) -> None:
    a = await repo.insert(new("a"), at=T0)
    assert await repo.set_acknowledged(a, True, at=T0)
    later = T0 + timedelta(seconds=30)
    assert await repo.bump(a, at=later)
    e = await repo.get(a)
    assert e is not None and e.count == 2 and e.last_at == later and e.acknowledged_at is None
    assert await repo.append_detail(a, "終了処理の結果", at=later, context_patch={"job_end": {"exit_code": 130}})
    e = await repo.get(a)
    assert e is not None and e.detail == "本文\n終了処理の結果" and e.context["job_end"]["exit_code"] == 130
    assert await repo.acknowledge_all(at=later) == 1
    assert await repo.delete_acknowledged() == 1
    assert await repo.delete(a) is False and await repo.bump(a, at=later) is False
