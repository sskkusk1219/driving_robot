"""WebSocket 配信ループのテスト。

非常停止の核心的不具合「HW 読み取りがストールすると robot_state が配信されず、
フロントが非常停止画面に遷移しない」の回帰を防ぐ。
"""

import asyncio
import json
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest

from src.models.system_state import RobotState, SystemState
from src.web import ws


class _FakeWS:
    """send_text を記録するだけの最小 WebSocket スタブ。"""

    def __init__(self) -> None:
        self.sent: list[str] = []

    async def send_text(self, data: str) -> None:
        self.sent.append(data)


def _make_app(controller: MagicMock) -> MagicMock:
    app = MagicMock()
    app.state.controller = controller
    ups_status = MagicMock()
    ups_status.is_available = False
    ups_monitor = MagicMock()
    ups_monitor.get_status = AsyncMock(return_value=ups_status)
    app.state.ups_monitor = ups_monitor
    return app


def _make_controller(realtime_impl) -> MagicMock:
    controller = MagicMock()
    controller.get_system_state.return_value = SystemState(
        robot_state=RobotState.EMERGENCY,
        active_profile_id=None,
        active_session_id=None,
        last_normal_shutdown=False,
        updated_at=datetime.now(tz=UTC),
    )
    controller.get_realtime_data = realtime_impl
    controller.current_openings = (0.0, 0.0)
    return controller


@pytest.fixture
def fake_connection():
    """manager に接続を1つ登録し、テスト後に必ず取り除く。"""
    conn = _FakeWS()
    ws.manager._connections.append(conn)  # type: ignore[arg-type]
    yield conn
    ws.manager.disconnect(conn)  # type: ignore[arg-type]


async def _run_until_message(app: MagicMock, conn: _FakeWS, timeout_s: float = 2.0) -> dict:
    task = asyncio.create_task(ws.broadcast_loop(app))
    try:
        deadline = asyncio.get_event_loop().time() + timeout_s
        while not conn.sent:
            if asyncio.get_event_loop().time() > deadline:
                raise AssertionError("broadcast_loop がメッセージを配信しなかった")
            await asyncio.sleep(0.01)
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
    return json.loads(conn.sent[0])


@pytest.mark.asyncio
async def test_broadcast_emits_state_even_when_realtime_read_hangs(
    fake_connection: _FakeWS, monkeypatch: pytest.MonkeyPatch
) -> None:
    """get_realtime_data がハングしても robot_state は配信される（非常停止の核心修正）。"""
    monkeypatch.setattr(ws, "GET_REALTIME_TIMEOUT_S", 0.05)

    async def _hang() -> None:
        await asyncio.sleep(3600)

    app = _make_app(_make_controller(_hang))
    data = await _run_until_message(app, fake_connection)

    assert data["robot_state"] == "EMERGENCY"
    # HW 読み取りはタイムアウトしフォールバック値（0.0）になる
    assert data["actual_speed_kmh"] == 0.0


@pytest.mark.asyncio
async def test_broadcast_emits_realtime_values_on_normal_read(
    fake_connection: _FakeWS,
) -> None:
    """正常時は計測値をそのまま配信する。"""
    from src.models.system_state import RealtimeSnapshot

    async def _ok() -> RealtimeSnapshot:
        return RealtimeSnapshot(
            actual_speed_kmh=12.5,
            accel_pos=100,
            brake_pos=0,
            accel_current_ma=500.0,
            brake_current_ma=10.0,
        )

    app = _make_app(_make_controller(_ok))
    data = await _run_until_message(app, fake_connection)

    assert data["robot_state"] == "EMERGENCY"
    assert data["actual_speed_kmh"] == 12.5
    assert data["accel_current_ma"] == 500.0


@pytest.mark.asyncio
async def test_broadcast_isolates_stalled_client(monkeypatch: pytest.MonkeyPatch) -> None:
    """1 クライアントの送信ストールが他クライアントの配信を阻害しない（レビュー #14）。

    旧実装は逐次 await のため、TCP バックプレッシャで固まったクライアントが
    broadcast_loop 全体（EMERGENCY 表示含む）を凍結させていた。
    """
    monkeypatch.setattr(ws, "SEND_TIMEOUT_S", 0.05)

    class _StalledWS:
        async def send_text(self, data: str) -> None:
            await asyncio.sleep(3600)

    stalled = _StalledWS()
    healthy = _FakeWS()
    ws.manager._connections.extend([stalled, healthy])  # type: ignore[list-item]
    try:
        await asyncio.wait_for(ws.manager.broadcast("payload"), timeout=1.0)
    finally:
        ws.manager.disconnect(stalled)  # type: ignore[arg-type]
        ws.manager.disconnect(healthy)  # type: ignore[arg-type]

    # 健全なクライアントへは配信され、ストールしたクライアントは切断扱いになる
    assert healthy.sent == ["payload"]
    assert stalled not in ws.manager._connections
