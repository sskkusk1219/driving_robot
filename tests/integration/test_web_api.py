"""Web レイヤー 統合テスト（FastAPI アプリ全体・モック Controller）。"""

from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.robot_controller import RobotController
from src.models.system_state import RobotState, SystemState
from src.web.app import app


def _make_stub_controller() -> MagicMock:
    ctrl = MagicMock(spec=RobotController)
    ctrl.get_system_state.return_value = SystemState(
        robot_state=RobotState.STANDBY,
        active_profile_id=None,
        active_session_id=None,
        last_normal_shutdown=False,
        updated_at=datetime.now(tz=UTC),
    )
    ctrl.initialize = AsyncMock()
    ctrl.emergency_stop = AsyncMock()
    ctrl.reset_emergency = AsyncMock()
    ctrl.stop = AsyncMock()
    ctrl.stop_manual = AsyncMock()
    ctrl.start_auto_drive = AsyncMock()
    ctrl.start_manual = AsyncMock()
    return ctrl


@pytest.fixture(autouse=True)
def inject_controller() -> MagicMock:
    ctrl = _make_stub_controller()
    app.state.controller = ctrl
    return ctrl


@pytest.mark.asyncio
async def test_status_endpoint_returns_state() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/drive/status")
    assert res.status_code == 200
    assert res.json()["robot_state"] == "STANDBY"


@pytest.mark.asyncio
async def test_profiles_list_empty() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/profiles/")
    assert res.status_code == 200
    assert res.json() == []


@pytest.mark.asyncio
async def test_profiles_get_404() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/profiles/nonexistent")
    assert res.status_code == 404


@pytest.mark.asyncio
async def test_modes_list_empty() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/modes/")
    assert res.status_code == 200
    assert res.json() == []


@pytest.mark.asyncio
async def test_sessions_list_empty() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/sessions/")
    assert res.status_code == 200
    assert res.json() == []


@pytest.mark.asyncio
async def test_sessions_logs_empty() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/sessions/some-id/logs")
    assert res.status_code == 200
    assert res.json() == []


@pytest.mark.asyncio
async def test_docs_endpoint() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/docs")
    assert res.status_code == 200


@pytest.mark.asyncio
async def test_websocket_connect() -> None:
    from starlette.testclient import TestClient

    with TestClient(app) as client:
        with client.websocket_connect("/ws/realtime") as ws:
            # 接続直後はサーバーからのメッセージをすぐには受け取れないが接続は成功する
            assert ws is not None
