import asyncio
import logging
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.base import BaseHTTPMiddleware, RequestResponseEndpoint
from starlette.responses import Response

from src.app.stubs import (
    InMemoryModeRepository,
    InMemoryProfileRepository,
    InMemoryScheduleRepository,
    InMemorySessionRepository,
    _StubUPSMonitor,
    build_stub_controller,
)
from src.app.training_service import feature_spec_from_settings
from src.domain.model_training import DEFAULT_FEATURE_SPEC, FeatureSpec
from src.infra.settings import LearningSettings
from src.web.routers import drive, modes, profiles, schedules, sessions, ups
from src.web.ws import broadcast_loop, realtime_ws


def _load_feature_spec_and_learning_settings() -> tuple[FeatureSpec, LearningSettings]:
    """`config/settings.toml` から特徴量構成・学習サイクル設定を読み込む。

    設定ファイルが無い開発・スタブ環境ではデフォルト値にフォールバックする
    （本番の特徴量セット変更は config/settings.toml 経由でのみ行う）。
    """
    from src.infra.settings import load_settings  # noqa: PLC0415

    try:
        settings = load_settings()
    except FileNotFoundError:
        return DEFAULT_FEATURE_SPEC, LearningSettings()
    return feature_spec_from_settings(settings.model), settings.learning


async def _build_repos(app: FastAPI) -> None:
    """DB が利用可能なら DB バックエンド、そうでなければ in-memory リポジトリを設定する。"""
    app.state.feature_spec, app.state.learning_settings = (
        _load_feature_spec_and_learning_settings()
    )
    db_url = os.environ.get("DATABASE_URL")
    if db_url:
        from src.infra.db import create_pool  # noqa: PLC0415
        from src.infra.mode_repository import ModeRepository  # noqa: PLC0415
        from src.infra.profile_repository import ProfileRepository  # noqa: PLC0415
        from src.infra.schedule_repository import ScheduleRepository  # noqa: PLC0415
        from src.infra.session_repository import SessionRepository  # noqa: PLC0415

        pool = await create_pool(db_url)
        app.state.db_pool = pool
        app.state.profile_repo = ProfileRepository(pool)
        app.state.mode_repo = ModeRepository(pool)
        app.state.session_repo = SessionRepository(pool)
        app.state.schedule_repo = ScheduleRepository(pool)

        # 前回プロセスの異常終了で 'running' のまま残った孤児セッションを是正する。
        from src.infra.log_writer import LogWriter  # noqa: PLC0415

        reaped = await LogWriter(pool).reap_interrupted_sessions()
        if reaped:
            logging.getLogger(__name__).warning(
                "起動時に未終了セッション %d 件を 'error' で終了済みに是正しました", reaped
            )
    else:
        app.state.db_pool = None
        app.state.profile_repo = InMemoryProfileRepository()
        app.state.mode_repo = InMemoryModeRepository()
        app.state.session_repo = InMemorySessionRepository()
        app.state.schedule_repo = InMemoryScheduleRepository()


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    await _build_repos(app)

    if os.environ.get("DRIVING_ROBOT_USE_REAL_HW") == "1":
        from src.app.factory import build_real_controller  # noqa: PLC0415
        from src.infra.settings import load_settings  # noqa: PLC0415

        # ベンチ検証モード: 非常停止スイッチ(GPIO)のみ実機、アクチュエータ/CAN はスタブ
        bench_gpio_only = os.environ.get("DRIVING_ROBOT_BENCH_GPIO_ONLY") == "1"
        controller, ups_monitor = await build_real_controller(
            load_settings(), bench_gpio_only=bench_gpio_only
        )
        await ups_monitor.start_polling()
    else:
        controller = build_stub_controller()
        ups_monitor = _StubUPSMonitor()

    await controller.start()
    app.state.controller = controller
    app.state.ups_monitor = ups_monitor

    from src.app.learning_cycle import LearningCycleOrchestrator  # noqa: PLC0415

    cycle_log_writer = None
    if app.state.db_pool is not None:
        from src.infra.log_writer import LogWriter  # noqa: PLC0415

        cycle_log_writer = LogWriter(app.state.db_pool)
    app.state.cycle_orchestrator = LearningCycleOrchestrator(
        controller=controller,
        profile_repo=app.state.profile_repo,
        session_repo=app.state.session_repo,
        log_writer=cycle_log_writer,
        learning_timeout_s=app.state.learning_settings.learning_timeout_s,
    )

    task = asyncio.create_task(broadcast_loop(app))
    try:
        yield
    finally:
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        await controller.shutdown()
        await ups_monitor.stop_polling()
        if app.state.db_pool is not None:
            await app.state.db_pool.close()


app = FastAPI(
    title="driving_robot API",
    description="シャシダイナモ向け自動運転ロボットシステム REST API",
    version="0.1.0",
    lifespan=lifespan,
)

app.include_router(drive.router)
app.include_router(profiles.router)
app.include_router(modes.router)
app.include_router(schedules.router)
app.include_router(sessions.router)
app.include_router(ups.router)

class _NoCacheStaticMiddleware(BaseHTTPMiddleware):
    """/static 配下に `Cache-Control: no-cache` を付与する。

    ブラウザに毎回 ETag/Last-Modified での再検証を強制し（変更が無ければ 304）、
    UI（JS/CSS）を更新した際に古いキャッシュが使われ続ける事故を防ぐ。
    """

    async def dispatch(
        self, request: Request, call_next: RequestResponseEndpoint
    ) -> Response:
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
