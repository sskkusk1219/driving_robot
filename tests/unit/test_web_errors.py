# ruff: noqa: E501
"""エラーページ API（/api/v1/errors）のユニットテスト。PostgreSQL 不要（in-memory リポジトリ）。"""

from __future__ import annotations

from collections.abc import AsyncIterator
from datetime import UTC, datetime, timedelta

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.error_recorder import ErrorRecorder
from src.app.inmemory_repos import InMemoryErrorRepository
from src.models.error_event import LEVEL_ERROR, LEVEL_WARNING, NewErrorEvent
from src.web.app import app

T0 = datetime(2026, 11, 20, 9, 0, 0, tzinfo=UTC)


def new(title: str, level: str = LEVEL_ERROR, source: str = "job", **kw: object) -> NewErrorEvent:
    return NewErrorEvent(level=level, source=source, title=title, detail=f"{title} の本文", **kw)  # type: ignore[arg-type]


@pytest.fixture
async def client() -> AsyncIterator[tuple[AsyncClient, InMemoryErrorRepository, ErrorRecorder]]:
    repo = InMemoryErrorRepository()
    recorder = ErrorRecorder(repo)
    recorder.start()
    app.state.error_repo = repo
    app.state.error_recorder = recorder
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        yield c, repo, recorder
    await recorder.stop()


async def seed(repo: InMemoryErrorRepository) -> list[int]:
    ids = [
        await repo.insert(new("a", LEVEL_ERROR, "job", session_id="S1", job_id="j1", context={"messages": [{"level": "ERROR", "text": "x", "time": "t"}]}), at=T0),
        await repo.insert(new("b", LEVEL_WARNING, "idle"), at=T0 + timedelta(minutes=1)),
        await repo.insert(new("c", LEVEL_ERROR, "browser"), at=T0 + timedelta(minutes=2)),
    ]
    return ids


async def test_list_orders_newest_first_and_filters(client) -> None:
    c, repo, _ = client
    a, b, cc = await seed(repo)
    body = (await c.get("/api/v1/errors")).json()
    assert [i["id"] for i in body["items"]] == [cc, b, a] and body["total"] == 3
    assert "detail" not in body["items"][0] and body["items"][0]["acknowledged_at"] is None
    assert [i["id"] for i in (await c.get("/api/v1/errors", params={"level": "WARNING"})).json()["items"]] == [b]
    assert [i["id"] for i in (await c.get("/api/v1/errors", params={"source": "job"})).json()["items"]] == [a]
    page = (await c.get("/api/v1/errors", params={"limit": 1, "offset": 1})).json()
    assert [i["id"] for i in page["items"]] == [b] and page["total"] == 3
    assert (await c.get("/api/v1/errors", params={"source": "nope"})).status_code == 422
    assert (await c.get("/api/v1/errors", params={"level": "INFO"})).status_code == 422


async def test_detail_acknowledge_unacknowledge_and_unacknowledged_filter(client) -> None:
    c, repo, _ = client
    a, b, cc = await seed(repo)
    d = (await c.get(f"/api/v1/errors/{a}")).json()
    assert d["detail"] == "a の本文" and d["session_id"] == "S1" and d["job_id"] == "j1" and d["context"]["messages"][0]["text"] == "x"
    r = await c.post(f"/api/v1/errors/{a}/acknowledge")
    assert r.status_code == 200 and r.json()["acknowledged_at"] is not None
    ids = [i["id"] for i in (await c.get("/api/v1/errors", params={"unacknowledged": "true"})).json()["items"]]
    assert ids == [cc, b]
    r = await c.post(f"/api/v1/errors/{a}/unacknowledge")
    assert r.json()["acknowledged_at"] is None
    assert (await c.post("/api/v1/errors/999/acknowledge")).status_code == 404
    assert (await c.get("/api/v1/errors/999")).status_code == 404


async def test_acknowledge_all_and_delete(client) -> None:
    c, repo, _ = client
    a, b, cc = await seed(repo)
    assert (await c.post("/api/v1/errors/acknowledge-all")).json() == {"count": 3}
    assert (await c.post("/api/v1/errors/acknowledge-all")).json() == {"count": 0}
    await c.post(f"/api/v1/errors/{b}/unacknowledge")
    assert (await c.delete("/api/v1/errors/acknowledged")).json() == {"count": 2}  # 確認済み（a・c）だけ消える
    assert [i["id"] for i in (await c.get("/api/v1/errors")).json()["items"]] == [b]
    assert (await c.delete(f"/api/v1/errors/{b}")).status_code == 204
    assert (await c.delete(f"/api/v1/errors/{b}")).status_code == 404


async def test_client_post_records_browser_event_and_merges_repeats(client) -> None:
    c, repo, recorder = client
    body = {"message": "開始できません\n次にすること: 初期化", "kind": "toast", "url": "http://x/#init"}
    assert (await c.post("/api/v1/errors/client", json=body)).status_code == 204
    assert (await c.post("/api/v1/errors/client", json=body)).status_code == 204
    await c.post("/api/v1/errors/client", json={"message": "TypeError: x is undefined", "kind": "onerror", "stack": "at foo (app.js:1)"})
    await recorder.flush()
    items, total = await repo.list_events()
    assert total == 2
    toast = next(e for e in items if e.title == "開始できません")
    assert toast.source == "browser" and toast.level == LEVEL_ERROR and toast.count == 2
    assert toast.context == {"kind": "toast", "url": "http://x/#init"}
    err = next(e for e in items if e.title.startswith("TypeError"))
    assert "at foo (app.js:1)" in err.detail and err.context["kind"] == "onerror"


async def test_client_post_validates_and_survives_without_recorder(client) -> None:
    c, repo, _ = client
    assert (await c.post("/api/v1/errors/client", json={"kind": "toast"})).status_code == 422
    assert (await c.post("/api/v1/errors/client", json={"message": "x", "kind": "weird"})).status_code == 422
    del app.state.error_recorder
    assert (await c.post("/api/v1/errors/client", json={"message": "x"})).status_code == 204  # 記録係が無くても失敗にしない
    assert (await repo.list_events())[1] == 0
