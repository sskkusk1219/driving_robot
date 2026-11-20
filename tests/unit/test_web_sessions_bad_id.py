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
