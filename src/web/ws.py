import asyncio
import logging
from datetime import UTC, datetime

from fastapi import WebSocket
from starlette.applications import Starlette

from src.models.system_state import RobotState
from src.web.schemas import RealtimeData

logger = logging.getLogger(__name__)

# WebSocket配信は100ms周期で±5msのジッタ許容のため asyncio.sleep を使用（制御ループとは別基準）
WS_BROADCAST_INTERVAL_S = 0.1


class ConnectionManager:
    def __init__(self) -> None:
        self._connections: list[WebSocket] = []

    @property
    def has_connections(self) -> bool:
        return bool(self._connections)

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._connections.append(ws)

    def disconnect(self, ws: WebSocket) -> None:
        if ws in self._connections:
            self._connections.remove(ws)

    async def broadcast(self, data: str) -> None:
        dead: list[WebSocket] = []
        for ws in self._connections:
            try:
                await ws.send_text(data)
            except Exception as exc:
                logger.debug("WebSocket送信失敗、切断として処理: %s", exc)
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


manager = ConnectionManager()


async def realtime_ws(ws: WebSocket) -> None:
    await manager.connect(ws)
    try:
        while True:
            await ws.receive_text()
    except Exception as exc:
        logger.debug("WebSocket接続終了: %s", exc)
        manager.disconnect(ws)


async def broadcast_loop(app: Starlette) -> None:
    while True:
        await asyncio.sleep(WS_BROADCAST_INTERVAL_S)
        if not manager.has_connections:
            continue
        controller = app.state.controller
        state = controller.get_system_state()
        try:
            snapshot = await controller.get_realtime_data()
            actual_speed = snapshot.actual_speed_kmh
            accel_current = snapshot.accel_current_ma
            brake_current = snapshot.brake_current_ma
        except Exception as exc:
            logger.debug("get_realtime_data 失敗（フォールバック）: %s", exc)
            actual_speed = 0.0
            accel_current = 0.0
            brake_current = 0.0
        data = RealtimeData(
            timestamp=datetime.now(tz=UTC).isoformat(),
            robot_state=RobotState(state.robot_state),
            actual_speed_kmh=actual_speed,
            ref_speed_kmh=None,
            accel_opening=0.0,
            brake_opening=0.0,
            accel_current_ma=accel_current,
            brake_current_ma=brake_current,
        )
        await manager.broadcast(data.model_dump_json())
