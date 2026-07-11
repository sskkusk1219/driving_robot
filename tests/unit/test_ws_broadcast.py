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

    async def accept(self) -> None:
        pass

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
    # cycle_orchestrator 未設定（None）時のフォールバック経路を通す
    app.state.cycle_orchestrator = None
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
async def test_broadcast_includes_cycle_progress_when_orchestrator_present(
    fake_connection: _FakeWS,
) -> None:
    """cycle_orchestrator が設定されていれば cycle_progress が配信に含まれること。"""
    from src.app.learning_cycle import CyclePhase, CycleProgress
    from src.models.system_state import RealtimeSnapshot

    async def _ok() -> RealtimeSnapshot:
        return RealtimeSnapshot(
            actual_speed_kmh=0.0,
            accel_pos=0,
            brake_pos=0,
            accel_current_ma=0.0,
            brake_current_ma=0.0,
        )

    app = _make_app(_make_controller(_ok))
    orchestrator = MagicMock()
    orchestrator.progress = CycleProgress(
        cycle_id="cycle-1",
        phase=CyclePhase.REFINE_1,
        run_index=2,
        run_total=10,
        best_cost=0.42,
        message="PID適合を実行しています",
        started_at=datetime.now(tz=UTC),
    )
    app.state.cycle_orchestrator = orchestrator

    data = await _run_until_message(app, fake_connection)

    assert data["cycle_progress"]["cycle_id"] == "cycle-1"
    assert data["cycle_progress"]["phase"] == "REFINE_1"
    assert data["cycle_progress"]["run_index"] == 2
    assert data["cycle_progress"]["run_total"] == 10
    assert data["cycle_progress"]["best_cost"] == 0.42


@pytest.mark.asyncio
async def test_broadcast_cycle_progress_none_without_orchestrator(
    fake_connection: _FakeWS,
) -> None:
    from src.models.system_state import RealtimeSnapshot

    async def _ok() -> RealtimeSnapshot:
        return RealtimeSnapshot(
            actual_speed_kmh=0.0,
            accel_pos=0,
            brake_pos=0,
            accel_current_ma=0.0,
            brake_current_ma=0.0,
        )

    app = _make_app(_make_controller(_ok))
    data = await _run_until_message(app, fake_connection)

    assert data["cycle_progress"] is None


@pytest.mark.asyncio
async def test_broadcast_skips_redundant_send_when_unchanged(
    fake_connection: _FakeWS, monkeypatch: pytest.MonkeyPatch
) -> None:
    """E4 回帰テスト: timestamp 以外が不変なら再送を省略する（アイドル時の無駄な送信削減）。

    同一の broadcast_loop インスタンスを継続稼働させ、値が不変のまま複数ティック
    経過しても送信件数が 1 件目から増えないことを確認する
    （last_payload は broadcast_loop 呼び出し毎にリセットされるため、新しいループ
    インスタンスを作ると常に1件目が送信されてしまい検証にならない）。
    """
    from src.models.system_state import RealtimeSnapshot

    monkeypatch.setattr(ws, "WS_BROADCAST_INTERVAL_S", 0.01)
    ws.manager.needs_full_send = False  # 直前のテストの残留状態をリセット

    async def _ok() -> RealtimeSnapshot:
        return RealtimeSnapshot(
            actual_speed_kmh=0.0,
            accel_pos=0,
            brake_pos=0,
            accel_current_ma=0.0,
            brake_current_ma=0.0,
        )

    app = _make_app(_make_controller(_ok))
    task = asyncio.create_task(ws.broadcast_loop(app))
    try:
        deadline = asyncio.get_event_loop().time() + 2.0
        while not fake_connection.sent:
            if asyncio.get_event_loop().time() > deadline:
                raise AssertionError("broadcast_loop がメッセージを配信しなかった")
            await asyncio.sleep(0.01)
        # 初回送信（needs_full_send=False でも last_payload は None→値で変化するため送信される）
        assert len(fake_connection.sent) == 1

        # 値が完全に不変のまま同一ループで複数ティック経過させる
        await asyncio.sleep(0.1)
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    assert len(fake_connection.sent) == 1  # 追加送信は発生しない


@pytest.mark.asyncio
async def test_broadcast_forces_send_for_newly_connected_client(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """E4 回帰テスト: ループ稼働中に新規接続したクライアントは、値が不変でも
    次ティックで必ず現在値を受け取る（変更検知による送信スキップで無表示にならない）。"""
    from src.models.system_state import RealtimeSnapshot

    monkeypatch.setattr(ws, "WS_BROADCAST_INTERVAL_S", 0.01)

    async def _ok() -> RealtimeSnapshot:
        return RealtimeSnapshot(
            actual_speed_kmh=0.0,
            accel_pos=0,
            brake_pos=0,
            accel_current_ma=0.0,
            brake_current_ma=0.0,
        )

    app = _make_app(_make_controller(_ok))
    first = _FakeWS()
    ws.manager._connections.append(first)  # type: ignore[arg-type]
    second = _FakeWS()
    try:
        task = asyncio.create_task(ws.broadcast_loop(app))
        try:
            # 1件目が送信され値が安定するのを待つ（needs_full_send を消費させる）
            deadline = asyncio.get_event_loop().time() + 2.0
            while not first.sent:
                if asyncio.get_event_loop().time() > deadline:
                    raise AssertionError("初回配信がされなかった")
                await asyncio.sleep(0.01)
            await asyncio.sleep(0.03)  # 値不変のまま数ティック経過させる

            # 稼働中に新規クライアントが接続する
            await ws.manager.connect(second)  # type: ignore[arg-type]
            deadline = asyncio.get_event_loop().time() + 2.0
            while not second.sent:
                if asyncio.get_event_loop().time() > deadline:
                    raise AssertionError("新規接続したクライアントへ配信されなかった")
                await asyncio.sleep(0.01)
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
    finally:
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
