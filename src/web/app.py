# ruff: noqa: E501
import asyncio
import logging
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint
from starlette.responses import Response

from src.app.inmemory_repos import (
    InMemoryModeRepository,
    InMemoryScheduleRepository,
    InMemorySequenceRepository,
    InMemorySessionRepository,
    StubUPSMonitor,
)
from src.app.job_manager import JobError, JobManager, hw_mode_from_env
from src.infra.db import DuplicateNameError
from src.infra.drive_recorder import DriveRecorder, NullDriveRecorder
from src.infra.log_buffer import LogBufferHandler
from src.infra.vehicle_store import (
    DEFAULT_COMMON,
    DEFAULT_VEHICLES_DIR,
    VehicleExists,
    VehicleInvalid,
    VehicleNotFound,
    VehicleStore,
    VehicleStoreError,
)
from src.web.routers import drive, modes, profiles, schedules, sequences, sessions, ups
from src.web.ws import broadcast_loop, realtime_ws

#: ジョブの外でランナーが出す行（待機中の見張り・非常停止の後とアプリ終了の終了処理）を残すファイル
DEFAULT_RUNNER_IDLE_LOG = Path(__file__).resolve().parents[2] / "logs" / "runner_idle.log"


async def _build_repos(app: FastAPI) -> DriveRecorder:
    """DB が利用可能なら DB バックエンド、そうでなければ in-memory リポジトリを設定する。

    走行の記録先（DriveRecorder）を返す。DB が無ければ何も書かない NullDriveRecorder。
    """
    db_url = os.environ.get("DATABASE_URL")
    if db_url:
        from src.infra.db import create_pool  # noqa: PLC0415
        from src.infra.drive_recorder import PgDriveRecorder  # noqa: PLC0415
        from src.infra.mode_repository import ModeRepository  # noqa: PLC0415
        from src.infra.schedule_repository import ScheduleRepository  # noqa: PLC0415
        from src.infra.sequence_repository import SequenceRepository  # noqa: PLC0415
        from src.infra.session_repository import SessionRepository  # noqa: PLC0415

        pool = await create_pool(db_url)
        app.state.db_pool = pool
        app.state.mode_repo = ModeRepository(pool)
        app.state.session_repo = SessionRepository(pool)
        app.state.schedule_repo = ScheduleRepository(pool)
        app.state.sequence_repo = SequenceRepository(pool)

        # 前回プロセスの異常終了で 'running' のまま残った孤児セッションを是正する。
        from src.infra.log_writer import LogWriter  # noqa: PLC0415

        reaped = await LogWriter(pool).reap_interrupted_sessions()
        if reaped:
            logging.getLogger(__name__).warning(
                "起動時に未終了セッション %d 件を 'error' で終了済みに是正しました", reaped
            )
        return PgDriveRecorder(pool)
    app.state.db_pool = None
    app.state.mode_repo = InMemoryModeRepository()
    app.state.session_repo = InMemorySessionRepository()
    app.state.schedule_repo = InMemoryScheduleRepository()
    app.state.sequence_repo = InMemorySequenceRepository()
    return NullDriveRecorder()


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    # サーバーログ・ランナーの進行のメッセージを WebUI の「現在ステータス」へ流すバッファを src ロガーに付ける
    log_buffer = LogBufferHandler(maxlen=500)
    src_logger = logging.getLogger("src")
    src_logger.addHandler(log_buffer)
    src_logger.setLevel(logging.INFO)
    app.state.log_buffer = log_buffer

    recorder = await _build_repos(app)

    # 車両フォルダ・共通設定の場所（開発・検証で本物の vehicles/ を触らないよう環境変数で差し替えられる）
    vehicles_dir = Path(os.environ.get("DRIVING_ROBOT_VEHICLES_DIR") or DEFAULT_VEHICLES_DIR)
    common_path = Path(os.environ.get("DRIVING_ROBOT_COMMON_CONFIG") or DEFAULT_COMMON)
    store = VehicleStore(vehicles_dir, common_path)
    app.state.vehicle_store = store

    gpio_monitor = None
    if os.environ.get("DRIVING_ROBOT_USE_REAL_HW") == "1":
        from src.infra.gpio_monitor import GPIOMonitor  # noqa: PLC0415
        from src.infra.settings import load_settings  # noqa: PLC0415
        from src.infra.ups_monitor import NutUPSMonitor  # noqa: PLC0415

        # 実機・ベンチ（DRIVING_ROBOT_BENCH_GPIO_ONLY=1）: 非常停止スイッチ(GPIO)と UPS は実機。
        # アクチュエータ・CAN はランナー側（--hw real か stub。hw_mode_from_env が決める）
        settings = load_settings()
        gpio_off_env = os.environ.get("DRIVING_ROBOT_GPIO_DISABLED") == "1"
        if settings.web_gpio.enabled and not gpio_off_env:
            gpio_monitor = GPIOMonitor(
                emergency_pin=settings.gpio.emergency_stop_pin,
                ac_detect_pin=settings.gpio.ac_detect_pin,
            )
        else:
            # 非常停止スイッチ未接続の機体用。見張りを作らない＝emergency_probe も無く、解除はピンを見ない
            logging.getLogger(__name__).warning(
                "非常停止スイッチ(GPIO)の見張りは無効です（%s）。画面の非常停止ボタンだけで止められます",
                "環境変数 DRIVING_ROBOT_GPIO_DISABLED=1" if gpio_off_env else "settings.toml の [web_gpio] enabled = false",
            )
        ups_monitor: NutUPSMonitor | StubUPSMonitor = NutUPSMonitor(
            nut_host=settings.ups.nut_host,
            nut_port=settings.ups.nut_port,
            ups_name=settings.ups.ups_name,
            poll_interval_s=settings.ups.poll_interval_s,
        )
    else:
        ups_monitor = StubUPSMonitor()

    manager = JobManager(
        vehicles_dir=vehicles_dir,
        common_path=common_path,
        hw_mode=hw_mode_from_env(),
        recorder=recorder,
        emergency_probe=gpio_monitor.is_emergency_active if gpio_monitor is not None else None,
        on_min_saved=store.mark_min_measured,
        idle_log_path=Path(os.environ.get("DRIVING_ROBOT_RUNNER_IDLE_LOG") or DEFAULT_RUNNER_IDLE_LOG),
    )
    app.state.job_manager = manager
    app.state.gpio_enabled = gpio_monitor is not None  # 初期化ページの「非常停止(GPIO)」の表示用
    app.state.ups_monitor = ups_monitor
    if not isinstance(ups_monitor, StubUPSMonitor):
        # AC 断 → JobManager.emergency（実機の UPS。GPIO の見張りが無くても繋ぐ）
        ups_monitor.register_ac_loss_callback(lambda: manager.emergency("ac_loss"))  # type: ignore[union-attr]
    if gpio_monitor is not None:
        # 非常停止（GPIO）→ JobManager.emergency（ランナーへ SIGINT。終わらなければ SIGKILL）
        gpio_monitor.register_emergency_callback(lambda: manager.emergency("switch"))
        await gpio_monitor.start_monitoring()
    await ups_monitor.start_polling()

    task = asyncio.create_task(broadcast_loop(app))
    try:
        yield
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        await manager.shutdown()
        if gpio_monitor is not None:
            gpio_monitor.stop_monitoring()
        await ups_monitor.stop_polling()
        if app.state.db_pool is not None:
            await app.state.db_pool.close()
        src_logger.removeHandler(log_buffer)


app = FastAPI(
    title="driving_robot API",
    description="シャシダイナモ向け自動運転ロボットシステム REST API",
    version="0.2.0",
    lifespan=lifespan,
)


# app レベル例外ハンドラ: 各ルーターで同じ try/except を書かずに済むよう、ここで一本化する。
@app.exception_handler(JobError)
async def _job_error_handler(request: Request, exc: JobError) -> JSONResponse:
    # 409 実行中・非常停止中 / 423 非常停止スイッチが押されたまま / 503 DB・ランナーを使えない
    return JSONResponse(status_code=exc.status_code, content={"detail": str(exc)})


@app.exception_handler(VehicleStoreError)
async def _vehicle_error_handler(request: Request, exc: VehicleStoreError) -> JSONResponse:
    if isinstance(exc, VehicleNotFound):
        status = 404
    elif isinstance(exc, VehicleExists):
        status = 409
    elif isinstance(exc, VehicleInvalid):
        status = 422
    else:
        status = 500
    return JSONResponse(status_code=status, content={"detail": str(exc)})


@app.exception_handler(DuplicateNameError)
async def _duplicate_name_error_handler(request: Request, exc: DuplicateNameError) -> JSONResponse:
    return JSONResponse(status_code=409, content={"detail": str(exc)})


app.include_router(drive.router)
app.include_router(profiles.router)
app.include_router(modes.router)
app.include_router(schedules.router)
app.include_router(sequences.router)
app.include_router(sessions.router)
app.include_router(ups.router)


class _NoCacheStaticMiddleware(BaseHTTPMiddleware):
    """/static 配下に `Cache-Control: no-cache` を付与する。

    ブラウザに毎回 ETag/Last-Modified での再検証を強制し（変更が無ければ 304）、
    UI（JS/CSS）を更新した際に古いキャッシュが使われ続ける事故を防ぐ。
    """

    async def dispatch(self, request: Request, call_next: RequestResponseEndpoint) -> Response:
        response = await call_next(request)
        if request.url.path.startswith("/static/"):
            response.headers["Cache-Control"] = "no-cache, must-revalidate"
        return response


app.add_middleware(_NoCacheStaticMiddleware)

_static_dir = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(_static_dir)), name="static")


@app.get("/", include_in_schema=False)
async def root() -> RedirectResponse:
    return RedirectResponse(url="/static/index.html")


@app.websocket("/ws/realtime")
async def ws_realtime(ws: WebSocket) -> None:
    await realtime_ws(ws)
