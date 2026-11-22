"""操作権（ControlLock）・変更系 API のミドルウェア・WS の役割通知のテスト。"""

import pytest
from httpx import ASGITransport, AsyncClient
from starlette.testclient import TestClient

from src.web.app import app
from src.web.control import GRACE_S, STALE_AFTER_S, ControlLock


class FakeClock:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def clock() -> FakeClock:
    return FakeClock()


@pytest.fixture
def lock(clock: FakeClock) -> ControlLock:
    return ControlLock(clock=clock)


def _take(lock: ControlLock, cid: str, iid: str) -> object:
    ws = object()
    lock.on_connect(cid, iid, ws)
    assert lock.claim(cid, iid)
    return ws


def test_claim_requires_connection(lock: ControlLock) -> None:
    assert lock.claim("a", "1") is False
    lock.on_connect("a", "1", object())
    assert lock.claim("a", "1") is True
    assert lock.is_operator("a", "1")
    assert lock.status("a", "1") == {"held": True, "mine": True, "waiting": False}


def test_busy_and_viewer(lock: ControlLock) -> None:
    _take(lock, "a", "1")
    lock.on_connect("b", "2", object())
    assert lock.claim("b", "2") is False
    assert not lock.is_operator("b", "2")
    assert lock.status("b", "2") == {"held": True, "mine": False, "waiting": False}


def test_reload_rebinds_within_grace(lock: ControlLock, clock: FakeClock) -> None:
    ws = _take(lock, "a", "1")
    lock.on_disconnect(ws)
    assert lock.status("a", "1")["waiting"] is True
    clock.t += GRACE_S - 1
    assert lock.on_connect("a", "9", object()) == "operator"
    assert lock.is_operator("a", "9")
    assert not lock.is_operator("a", "1")


def test_duplicate_when_holder_alive(lock: ControlLock) -> None:
    _take(lock, "a", "1")
    assert lock.on_connect("a", "2", object()) == "duplicate"
    assert not lock.is_operator("a", "2")
    assert lock.is_operator("a", "1")


def test_release_after_grace(lock: ControlLock, clock: FakeClock) -> None:
    ws = _take(lock, "a", "1")
    lock.on_disconnect(ws)
    clock.t += GRACE_S + 0.1
    assert lock.held is False
    assert not lock.is_operator("a", "1")
    # 空いたあとの同じ client_id は普通の閲覧扱い
    assert lock.on_connect("a", "2", object()) == "viewer"


def test_ping_keeps_alive_and_stale_releases(lock: ControlLock, clock: FakeClock) -> None:
    ws = _take(lock, "a", "1")
    clock.t += STALE_AFTER_S - 1
    lock.on_ping(ws)
    clock.t += STALE_AFTER_S - 1
    assert lock.held is True and lock.status("a", "1")["waiting"] is False
    clock.t += 2  # 最後の ping から STALE 超え → 待ち
    assert lock.status("a", "1")["waiting"] is True
    clock.t += GRACE_S
    assert lock.held is False


def test_old_ws_disconnect_after_rebind_keeps_holder(lock: ControlLock) -> None:
    old = _take(lock, "a", "1")
    assert lock.on_connect("a", "1", new := object()) == "operator"
    lock.on_disconnect(old)
    assert lock.status("a", "1")["waiting"] is False
    lock.on_ping(new)


# ── ミドルウェア ──
@pytest.fixture
def app_lock():
    lock = ControlLock()
    app.state.control = lock
    yield lock
    del app.state.control


def _client() -> AsyncClient:
    return AsyncClient(transport=ASGITransport(app=app, raise_app_exceptions=False), base_url="http://test")


async def test_middleware_blocks_viewer(app_lock: ControlLock) -> None:
    async with _client() as c:
        r = await c.post("/api/v1/drive/emergency")
        assert r.status_code == 403
        assert r.json()["detail"] == "閲覧用のウィンドウでは操作できません"
        r = await c.delete("/api/v1/errors/1")
        assert r.status_code == 403
        r = await c.delete("/api/v1/errors/acknowledged")
        assert r.status_code == 403


async def test_middleware_allows_get_and_error_ack(app_lock: ControlLock) -> None:
    async with _client() as c:
        assert (await c.get("/api/v1/ups/status")).status_code != 403
        assert (await c.post("/api/v1/errors/acknowledge-all")).status_code != 403
        assert (await c.post("/api/v1/errors/5/acknowledge")).status_code != 403
        assert (await c.post("/api/v1/errors/5/unacknowledge")).status_code != 403
        assert (await c.post("/api/v1/errors/client", json={})).status_code != 403
        r = await c.get("/api/v1/control", params={"client_id": "a", "instance_id": "1"})
        assert r.status_code == 200
        assert r.json() == {"held": False, "mine": False, "waiting": False}


async def test_middleware_operator_passes(app_lock: ControlLock) -> None:
    app_lock.on_connect("a", "1", object())
    assert app_lock.claim("a", "1")
    async with _client() as c:
        h = {"X-Client-Id": "a", "X-Instance-Id": "1"}
        assert (await c.post("/api/v1/drive/emergency", headers=h)).status_code != 403
        bad = {"X-Client-Id": "a", "X-Instance-Id": "2"}
        assert (await c.post("/api/v1/drive/emergency", headers=bad)).status_code == 403


async def test_claim_api(app_lock: ControlLock) -> None:
    async with _client() as c:
        body = {"client_id": "a", "instance_id": "1"}
        r = await c.post("/api/v1/control/claim", json=body)
        assert r.status_code == 409
        assert "接続を確認できません" in r.json()["detail"]
        app_lock.on_connect("a", "1", object())
        r = await c.post("/api/v1/control/claim", json=body)
        assert r.status_code == 200 and r.json() == {"role": "operator"}
        app_lock.on_connect("b", "2", object())
        r = await c.post("/api/v1/control/claim", json={"client_id": "b", "instance_id": "2"})
        assert r.status_code == 409
        assert "使用中" in r.json()["detail"]


# ── WebSocket ──
def test_ws_role_message(app_lock: ControlLock) -> None:
    client = TestClient(app)
    with client.websocket_connect("/ws/realtime?client_id=a&instance_id=1") as ws:
        assert ws.receive_json() == {"type": "control", "role": "viewer"}
        ws.send_text("ping")
        assert app_lock.has_connection("a", "1")
        assert app_lock.claim("a", "1")
    # 切断 → 待ちに入る
    assert app_lock.status("a", "1")["waiting"] is True
    with client.websocket_connect("/ws/realtime?client_id=a&instance_id=2") as ws:
        assert ws.receive_json() == {"type": "control", "role": "operator"}
    with client.websocket_connect("/ws/realtime") as ws:
        assert ws.receive_json() == {"type": "control", "role": "viewer"}
