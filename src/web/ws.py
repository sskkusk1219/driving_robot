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

# get_realtime_data（Modbus 読み取り）の上限。これを超えたら計測値はフォールバックし、
# robot_state の配信を止めない。非常停止中に home_return がバスを占有しても
# 状態（EMERGENCY）がフロントへ確実に届くようにするための分離措置。
GET_REALTIME_TIMEOUT_S = 0.5


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
            # HW 読み取りがストールしても robot_state の配信を止めないよう上限を設ける。
            snapshot = await asyncio.wait_for(
                controller.get_realtime_data(), timeout=GET_REALTIME_TIMEOUT_S
            )
            actual_speed = snapshot.actual_speed_kmh
            accel_current = snapshot.accel_current_ma
            brake_current = snapshot.brake_current_ma
        except Exception as exc:
            # TimeoutError も Exception のサブクラスのためここで捕捉される。
            logger.debug("get_realtime_data 失敗/タイムアウト（フォールバック）: %s", exc)
            actual_speed = 0.0
            accel_current = 0.0
            brake_current = 0.0
        accel_opening, brake_opening = controller.current_openings

        ups_battery_pct: float | None = None
        ups_on_battery: bool = False
        try:
            ups_monitor = app.state.ups_monitor
            ups_status = await ups_monitor.get_status()
            if ups_status.is_available:
                ups_battery_pct = ups_status.battery_charge_pct
                ups_on_battery = ups_status.on_battery
        except Exception as exc:
            logger.debug("UPS 状態取得失敗: %s", exc)

        data = RealtimeData(
            timestamp=datetime.now(tz=UTC).isoformat(),
            robot_state=RobotState(state.robot_state),
            actual_speed_kmh=actual_speed,
            ref_speed_kmh=None,
            accel_opening=accel_opening,
            brake_opening=brake_opening,
            accel_current_ma=accel_current,
            brake_current_ma=brake_current,
            ups_battery_pct=ups_battery_pct,
            ups_on_battery=ups_on_battery,
        )
        await manager.broadcast(data.model_dump_json())
