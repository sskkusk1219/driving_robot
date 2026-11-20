"""WebSocket 配信ループのテスト。

配信の元は JobManager の状態（HW には触らない）。非常停止の状態が配信されること、
値が不変なら再送を省くこと、新しい接続には必ず 1 回送ること、
1 クライアントのストールが他を止めないことを確かめる。
"""

import asyncio
import json
from unittest.mock import AsyncMock, MagicMock

import pytest

from src.web import ws


class _FakeWS:
    """send_text を記録するだけの最小 WebSocket スタブ。"""

    def __init__(self) -> None:
        self.sent: list[str] = []

    async def accept(self) -> None:
        pass

    async def send_text(self, data: str) -> None:
        self.sent.append(data)


def _make_app(robot_state: str = "EMERGENCY") -> MagicMock:
    app = MagicMock()
    jobs = MagicMock()
    jobs.snapshot.return_value = {
        "state": "emergency", "initialized": False, "robot_state": robot_state, "job": None,
        "last_job": None, "sequence": None, "last_sequence": None,
    }
    jobs.live_values.return_value = {
        "speed_kmh": 0.0, "ref_kmh": None, "accel_pct": 0.0, "brake_pct": 0.0,
    }
    app.state.job_manager = jobs
    ups_status = MagicMock()
    ups_status.is_available = False
    ups_monitor = MagicMock()
    ups_monitor.get_status = AsyncMock(return_value=ups_status)
    app.state.ups_monitor = ups_monitor
    return app


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
async def test_broadcast_emits_robot_state_from_job_manager(fake_connection: _FakeWS) -> None:
    msg = await _run_until_message(_make_app("EMERGENCY"), fake_connection)
    assert msg["robot_state"] == "EMERGENCY"
    assert msg["job"] is None and msg["accel_opening"] == 0.0


@pytest.mark.asyncio
async def test_broadcast_skips_redundant_send_and_forces_for_new_client(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(ws, "WS_BROADCAST_INTERVAL_S", 0.01)
    first = _FakeWS()
    ws.manager._connections.append(first)  # type: ignore[arg-type]
    ws.manager.needs_full_send = True
    task = asyncio.create_task(ws.broadcast_loop(_make_app("READY")))
    try:
        await asyncio.sleep(0.3)
        assert len(first.sent) == 1  # 値が不変なら 2 回目以降は送らない
        second = _FakeWS()
        await ws.manager.connect(second)  # type: ignore[arg-type]
        await asyncio.sleep(0.2)
        assert len(second.sent) >= 1  # 新しい接続には変化がなくても送る
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        ws.manager.disconnect(first)  # type: ignore[arg-type]
        ws.manager.disconnect(second)  # type: ignore[arg-type]


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
