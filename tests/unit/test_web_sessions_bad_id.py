"""セッション API に UUID でない id を渡したら 400（500 にしない）。PostgreSQL 不要。"""

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.inmemory_repos import InMemorySessionRepository
from src.infra.vehicle_store import VehicleStore
from src.web.app import app

VALID_MISSING = "00000000-0000-0000-0000-0000000000ff"
SUFFIXES = ["", "/messages", "/logs", "/logs.csv", "/analysis"]
NOT_FOUND_SUFFIXES = ["", "/messages", "/logs.csv", "/analysis"]  # /logs は 404 を持たず空配列


class _StrictSessionRepo(InMemorySessionRepository):
    """DB 層と同じく UUID 変換で ValueError を出す偽物。"""

    async def get_by_id(self, session_id):
        import uuid

        uuid.UUID(session_id)
        return None

    async def list_logs(self, session_id, limit=1000):
        import uuid

        uuid.UUID(session_id)
        return []

    async def list_cycles(self, profile_id=None, limit=100):
        import uuid

        if profile_id is not None:
            uuid.UUID(profile_id)
        return []


@pytest.fixture
async def client(tmp_path):
    app.state.session_repo = _StrictSessionRepo()
    app.state.vehicle_store = VehicleStore(tmp_path)
    async with AsyncClient(
        transport=ASGITransport(app=app, raise_app_exceptions=False), base_url="http://test"
    ) as c:
        yield c


@pytest.mark.parametrize("suffix", SUFFIXES)
async def test_bad_session_id_is_400(client, suffix):
    r = await client.get(f"/api/v1/sessions/abc{suffix}")
    assert r.status_code == 400
    assert "UUID" in r.json()["detail"]


@pytest.mark.parametrize("suffix", NOT_FOUND_SUFFIXES)
async def test_valid_missing_session_id_is_404(client, suffix):
    r = await client.get(f"/api/v1/sessions/{VALID_MISSING}{suffix}")
    assert r.status_code == 404


async def test_cycles_bad_profile_id_is_400(client):
    r = await client.get("/api/v1/sessions/cycles", params={"profile_id": "abc"})
    assert r.status_code == 400


async def test_cycles_valid_profile_id_ok(client):
    r = await client.get("/api/v1/sessions/cycles", params={"profile_id": VALID_MISSING})
    assert r.status_code == 200


async def test_logs_csv_has_bom_buttons_and_errors(tmp_path):
    from datetime import UTC, datetime, timedelta
    from types import SimpleNamespace

    from src.app.inmemory_repos import InMemoryErrorRepository
    from src.models.error_event import NewErrorEvent

    sid = "00000000-0000-0000-0000-000000000001"
    t0 = datetime(2026, 1, 1, tzinfo=UTC)

    def log(sec):
        return SimpleNamespace(
            timestamp=t0 + timedelta(seconds=sec), ref_speed_kmh=1.0, actual_speed_kmh=1.0,
            accel_opening=0.0, brake_opening=0.0, accel_pos=0, brake_pos=0,
            accel_current=0.0, brake_current=0.0, detail={},
        )

    class Repo(InMemorySessionRepository):
        async def get_by_id(self, session_id):
            return SimpleNamespace(started_at=t0)

        async def list_messages(self, session_id, limit=100_000):
            msg = '{"button":"D","seconds":1.5}'
            return [SimpleNamespace(level="BUTTON", time=t0 + timedelta(seconds=10), message=msg)]

        async def iter_logs_for_csv(self, session_id, chunk=2000):
            yield [log(0), log(1)]

    errors = InMemoryErrorRepository()
    ev = lambda title, sid_: NewErrorEvent(  # noqa: E731
        level="ERROR", source="s", title=title, detail="", context={}, session_id=sid_
    )
    await errors.insert(ev("mine", sid), at=t0 + timedelta(seconds=0.5))
    await errors.insert(ev("other", "zzz"), at=t0)
    app.state.session_repo = Repo()
    app.state.error_repo = errors
    app.state.vehicle_store = VehicleStore(tmp_path)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        r = await c.get(f"/api/v1/sessions/{sid}/logs.csv")
    assert r.status_code == 200
    text = r.content.decode("utf-8")
    assert text.startswith("﻿")
    lines = text.lstrip("﻿").splitlines()
    assert lines[0].endswith("btn_start,btn_P,btn_N,btn_D,error_level,error_title")
    assert len(lines) == 1 + 2 + 1  # 見出し＋ログ 2 行＋範囲の外のボタン 1 行
    assert lines[2].endswith(",0,0,0,0,ERROR,mine")
    assert lines[3].endswith(",0,0,0,1,,")
