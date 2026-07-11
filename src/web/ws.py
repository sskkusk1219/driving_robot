import asyncio
import logging
from datetime import UTC, datetime

from fastapi import WebSocket
from starlette.applications import Starlette

from src.models.system_state import RobotState
from src.web.schemas import CycleProgressSchema, InitStepSchema, RealtimeData

logger = logging.getLogger(__name__)

# WebSocket配信は100ms周期で±5msのジッタ許容のため asyncio.sleep を使用（制御ループとは別基準）
WS_BROADCAST_INTERVAL_S = 0.1

# get_realtime_data（Modbus 読み取り）の上限。これを超えたら計測値はフォールバックし、
# robot_state の配信を止めない。非常停止中に home_return がバスを占有しても
# 状態（EMERGENCY）がフロントへ確実に届くようにするための分離措置。
GET_REALTIME_TIMEOUT_S = 0.5

# クライアント毎の送信タイムアウト。1 クライアントの TCP バックプレッシャ（送信
# バッファ満杯）が全クライアントの監視表示（EMERGENCY 含む）を凍結させないための上限。
# 超過したクライアントは切断として扱う（コードレビュー 2026-06-11 指摘 #14）。
SEND_TIMEOUT_S = 0.5


class ConnectionManager:
    def __init__(self) -> None:
        self._connections: list[WebSocket] = []
        # 新規接続時に True にする。broadcast_loop の変更検知スキップ（E4）を1回だけ
        # バイパスし、接続直後のクライアントが値変化まで無表示にならないようにする。
        self.needs_full_send = True

    @property
    def has_connections(self) -> bool:
        return bool(self._connections)

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self._connections.append(ws)
        self.needs_full_send = True

    def disconnect(self, ws: WebSocket) -> None:
        if ws in self._connections:
            self._connections.remove(ws)

    async def broadcast(self, data: str) -> None:
        """全クライアントへ並列送信する。

        逐次送信だと 1 クライアントのストールが broadcast_loop 全体を止めるため、
        タイムアウト付きで並列に送り、失敗・超過したクライアントは切断する。
        """
        dead: list[WebSocket] = []

        async def _send(ws: WebSocket) -> None:
            try:
                await asyncio.wait_for(ws.send_text(data), timeout=SEND_TIMEOUT_S)
            except Exception as exc:
                logger.debug("WebSocket送信失敗、切断として処理: %s", exc)
                dead.append(ws)

        await asyncio.gather(*(_send(ws) for ws in list(self._connections)))
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
    # 直近送信したペイロード（timestamp 除外）。値が完全に不変なら再送を省略し、アイドル中の
    # 無駄な JSON 直列化・全クライアント送信を避ける（E4 レビュー指摘）。状態変化（EMERGENCY
    # 遷移等）は必ず内容差として検知されるため、安全に関わる更新の遅延は発生しない。
    last_payload: tuple[object, ...] | None = None
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

        init_steps = [
            InitStepSchema(key=s.key, label=s.label, status=s.status)
            for s in controller.init_progress
        ]

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

        cycle_progress: CycleProgressSchema | None = None
        orchestrator = getattr(app.state, "cycle_orchestrator", None)
        if orchestrator is not None:
            cycle_progress = CycleProgressSchema.model_validate(orchestrator.progress)

        # timestamp を除いた内容で変化検知する（送信スキップ判定専用。実送信は下の RealtimeData）。
        payload = (
            state.robot_state,
            actual_speed,
            controller.current_ref_speed,
            accel_opening,
            brake_opening,
            accel_current,
            brake_current,
            ups_battery_pct,
            ups_on_battery,
            tuple((s.key, s.status) for s in init_steps),
            cycle_progress.model_dump_json() if cycle_progress is not None else None,
        )
        if payload == last_payload and not manager.needs_full_send:
            continue
        last_payload = payload
        manager.needs_full_send = False

        data = RealtimeData(
            timestamp=datetime.now(tz=UTC).isoformat(),
            robot_state=RobotState(state.robot_state),
            actual_speed_kmh=actual_speed,
            # 制御ループが実際に追従している基準車速を配信する（走行中以外は None）
            ref_speed_kmh=controller.current_ref_speed,
            accel_opening=accel_opening,
            brake_opening=brake_opening,
            accel_current_ma=accel_current,
            brake_current_ma=brake_current,
            ups_battery_pct=ups_battery_pct,
            ups_on_battery=ups_on_battery,
            init_steps=init_steps,
            cycle_progress=cycle_progress,
        )
        await manager.broadcast(data.model_dump_json())
