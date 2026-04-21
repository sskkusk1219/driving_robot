"""Web レイヤー 走行制御エンドポイントのユニットテスト。"""

from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.robot_controller import InvalidStateTransition, PreCheckFailed, RobotController
from src.models.drive_log import DriveSession
from src.models.system_state import RobotState, SystemState
from src.web.app import app


def _make_stub_controller(state: RobotState = RobotState.READY) -> MagicMock:
    ctrl = MagicMock(spec=RobotController)
    ctrl.get_system_state.return_value = SystemState(
        robot_state=state,
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

    stub_session = DriveSession(
        id="session-001",
        profile_id="",
        mode_id="mode-001",
        run_type="auto",
        started_at=datetime.now(tz=UTC),
        ended_at=None,
        status="running",
    )
    ctrl.start_auto_drive = AsyncMock(return_value=stub_session)

    stub_manual_session = DriveSession(
        id="session-002",
        profile_id="",
        mode_id=None,
        run_type="manual",
        started_at=datetime.now(tz=UTC),
        ended_at=None,
        status="running",
    )
    ctrl.start_manual = AsyncMock(return_value=stub_manual_session)
    return ctrl


@pytest.fixture
def stub_controller() -> MagicMock:
    return _make_stub_controller()


@pytest.mark.asyncio
async def test_get_status(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/drive/status")
    assert res.status_code == 200
    data = res.json()
    assert data["robot_state"] == "READY"
    assert data["active_profile_id"] is None


@pytest.mark.asyncio
async def test_initialize_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/initialize")
    assert res.status_code == 200
    stub_controller.initialize.assert_awaited_once()


@pytest.mark.asyncio
async def test_initialize_invalid_state(stub_controller: MagicMock) -> None:
    stub_controller.initialize.side_effect = InvalidStateTransition("不正な遷移")
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/initialize")
    assert res.status_code == 409
    assert "不正な遷移" in res.json()["detail"]


@pytest.mark.asyncio
async def test_start_drive_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/start", json={"mode_id": "mode-001"})
    assert res.status_code == 200
    data = res.json()
    assert data["id"] == "session-001"
    assert data["run_type"] == "auto"
    stub_controller.start_auto_drive.assert_awaited_once_with("mode-001")


@pytest.mark.asyncio
async def test_start_drive_invalid_state(stub_controller: MagicMock) -> None:
    stub_controller.start_auto_drive.side_effect = InvalidStateTransition("READY 以外不可")
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/start", json={"mode_id": "mode-001"})
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_stop_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/stop")
    assert res.status_code == 200
    stub_controller.stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_stop_invalid_state(stub_controller: MagicMock) -> None:
    stub_controller.stop.side_effect = InvalidStateTransition("RUNNING 以外不可")
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/stop")
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_emergency_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/emergency")
    assert res.status_code == 200
    stub_controller.emergency_stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_manual_start_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/manual/start")
    assert res.status_code == 200
    data = res.json()
    assert data["run_type"] == "manual"
    stub_controller.start_manual.assert_awaited_once()


@pytest.mark.asyncio
async def test_manual_stop_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/manual/stop")
    assert res.status_code == 200
    stub_controller.stop_manual.assert_awaited_once()


@pytest.mark.asyncio
async def test_start_drive_precheck_failed_returns_422(stub_controller: MagicMock) -> None:
    stub_controller.start_auto_drive.side_effect = PreCheckFailed("キャリブレーション未完了")
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/start", json={"mode_id": "mode-001"})
    assert res.status_code == 422
    assert "キャリブレーション未完了" in res.json()["detail"]


@pytest.mark.asyncio
async def test_manual_start_precheck_failed_returns_422(stub_controller: MagicMock) -> None:
    stub_controller.start_manual.side_effect = PreCheckFailed("サーボ未ON")
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/manual/start")
    assert res.status_code == 422
