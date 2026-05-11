import asyncio
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles

from src.app.robot_controller import RobotController
from src.app.stubs import (
    InMemoryModeRepository,
    InMemoryProfileRepository,
    InMemorySessionRepository,
    build_stub_controller,
)
from src.web.routers import drive, modes, profiles, sessions
from src.web.ws import broadcast_loop, realtime_ws


async def _build_controller() -> RobotController:
    if os.environ.get("DRIVING_ROBOT_USE_REAL_HW") == "1":
        from src.app.factory import build_real_controller  # noqa: PLC0415
        from src.infra.settings import load_settings  # noqa: PLC0415

        return await build_real_controller(load_settings())
    return build_stub_controller()


async def _build_repos(app: FastAPI) -> None:
    """DB が利用可能なら DB バックエンド、そうでなければ in-memory リポジトリを設定する。"""
    db_url = os.environ.get("DATABASE_URL")
    if db_url:
        from src.infra.db import create_pool  # noqa: PLC0415
        from src.infra.mode_repository import ModeRepository  # noqa: PLC0415
        from src.infra.profile_repository import ProfileRepository  # noqa: PLC0415
        from src.infra.session_repository import SessionRepository  # noqa: PLC0415

        pool = await create_pool(db_url)
        app.state.db_pool = pool
        app.state.profile_repo = ProfileRepository(pool)
        app.state.mode_repo = ModeRepository(pool)
        app.state.session_repo = SessionRepository(pool)
    else:
        app.state.db_pool = None
        app.state.profile_repo = InMemoryProfileRepository()
        app.state.mode_repo = InMemoryModeRepository()
        app.state.session_repo = InMemorySessionRepository()


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    await _build_repos(app)
    controller = await _build_controller()
    await controller.start()
    app.state.controller = controller
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
app.include_router(sessions.router)

_static_dir = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=str(_static_dir)), name="static")


@app.get("/", include_in_schema=False)
async def root() -> RedirectResponse:
    return RedirectResponse(url="/static/index.html")


@app.websocket("/ws/realtime")
async def ws_realtime(ws: WebSocket) -> None:
    await realtime_ws(ws)
