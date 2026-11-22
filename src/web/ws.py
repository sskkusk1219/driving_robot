# ruff: noqa: E501
import asyncio
import json
import logging
from datetime import UTC, datetime

from fastapi import WebSocket
from starlette.applications import Starlette

from src.models.system_state import RobotState
from src.web.schemas import InitStepSchema, JobSnapshotSchema, RealtimeData

logger = logging.getLogger(__name__)

# WebSocket配信は100ms周期で±5msのジッタ許容のため asyncio.sleep を使用（制御ループとは別基準）
WS_BROADCAST_INTERVAL_S = 0.1

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
    """リアルタイム配信。クエリの client_id / instance_id で操作権の持ち主と結び付ける（無ければ閲覧のみ）。

    接続直後に {"type":"control","role":"operator|duplicate|viewer"} を 1 通送る
    （配信本体のメッセージには "type" が無い）。クライアントは 2 s ごとに "ping" を送る。
    """
    await manager.connect(ws)
    lock = getattr(ws.app.state, "control", None)
    client_id = ws.query_params.get("client_id")
    instance_id = ws.query_params.get("instance_id")
    role = "viewer"
    if lock is not None and client_id and instance_id:
        role = lock.on_connect(client_id, instance_id, ws)
    try:
        await ws.send_text(json.dumps({"type": "control", "role": role}))
        while True:
            msg = await ws.receive_text()
            if lock is not None and msg == "ping":
                lock.on_ping(ws)
    except Exception as exc:
        logger.debug("WebSocket接続終了: %s", exc)
    finally:
        manager.disconnect(ws)
        if lock is not None:
            lock.on_disconnect(ws)


async def broadcast_loop(app: Starlette) -> None:
    """JobManager の状態（ランナーのイベントから作った robot_state・車速・基準・開度・init_steps）と UPS を配信する。

    RealtimeData の形は段3 以前のまま。変えたのは cycle_progress → job（実行中のジョブの状態。
    手順・phase・最新 sample など）と、電流（accel_current_ma / brake_current_ma。ランナーの
    sample に無いので常に 0.0）。HW は触らない（ランナーのプロセスだけが触る）。
    """
    # 直近送信したペイロード（timestamp 除外）。値が完全に不変なら再送を省略し、アイドル中の
    # 無駄な JSON 直列化・全クライアント送信を避ける。状態変化（EMERGENCY 遷移等）は必ず内容差として
    # 検知されるため、安全に関わる更新の遅延は発生しない。
    last_payload: tuple[object, ...] | None = None
    while True:
        await asyncio.sleep(WS_BROADCAST_INTERVAL_S)
        control = getattr(app.state, "control", None)
        if control is not None:
            control.tick()  # 接続が無くても期限切れで操作権を空ける
        if not manager.has_connections:
            continue
        jobs = app.state.job_manager
        snap = jobs.snapshot()
        live = jobs.live_values()
        job = snap["job"]
        robot_state = RobotState(snap["robot_state"])

        init_steps = [
            InitStepSchema(key=s["key"], label=s["label"], status=s["status"])
            for s in (job["init_steps"] if job else [])
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

        job_schema = JobSnapshotSchema.model_validate(job) if job else None

        control_held = control.held if control is not None else False

        # timestamp・経過秒を除いた内容で変化検知する（送信スキップ判定専用。実送信は下の RealtimeData）。
        prepare = snap.get("prepare")
        payload = (
            robot_state,
            live["speed_kmh"],
            live["ref_kmh"],
            live["accel_pct"],
            live["brake_pct"],
            ups_battery_pct,
            ups_on_battery,
            tuple((s.key, s.status) for s in init_steps),
            (job["phase"], job["step"], job["sample"] and job["sample"].get("section")) if job else None,
            (job["paused"], len(job["button_events"]), job["dev_stats"] and job["dev_stats"]["n"]) if job else None,
            (snap["sequence"]["run_no"], snap["sequence"]["state"]) if snap["sequence"] else None,
            snap["initialized"],
            (prepare["id"], prepare["state"], prepare["done"], prepare["message"]) if prepare else None,
            snap.get("emergency_cause"),
            control_held,
        )
        if payload == last_payload and not manager.needs_full_send:
            continue
        last_payload = payload
        manager.needs_full_send = False

        data = RealtimeData(
            timestamp=datetime.now(tz=UTC).isoformat(),
            robot_state=robot_state,
            actual_speed_kmh=live["speed_kmh"],
            # ランナーの sample の基準車速（モード走行以外は None）
            ref_speed_kmh=live["ref_kmh"],
            accel_opening=live["accel_pct"],
            brake_opening=live["brake_pct"],
            accel_current_ma=0.0,
            brake_current_ma=0.0,
            ups_battery_pct=ups_battery_pct,
            ups_on_battery=ups_on_battery,
            init_steps=init_steps,
            initialized=snap["initialized"],
            job=job_schema,
            sequence=snap["sequence"],
            prepare=prepare,
            emergency_cause=snap.get("emergency_cause"),
            emergency_cause_detail=snap.get("emergency_cause_detail"),
            control_held=control_held,
        )
        await manager.broadcast(data.model_dump_json())
