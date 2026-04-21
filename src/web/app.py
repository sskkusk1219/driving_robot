import asyncio
import os
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles

from src.app.robot_controller import RobotController
from src.app.stubs import build_stub_controller
from src.web.routers import drive, modes, profiles, sessions
from src.web.ws import broadcast_loop, realtime_ws


def _build_controller() -> RobotController:
    if os.environ.get("DRIVING_ROBOT_USE_REAL_HW") == "1":
        from src.app.factory import build_real_controller
        from src.infra.settings import load_settings

        return build_real_controller(load_settings())
    return build_stub_controller()


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    controller = _build_controller()
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
