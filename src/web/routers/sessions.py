from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter(prefix="/api/v1/sessions", tags=["sessions"])


class SessionStub(BaseModel):
    id: str
    status: str


class LogStub(BaseModel):
    session_id: str
    actual_speed_kmh: float


@router.get("/", response_model=list[SessionStub])
async def list_sessions() -> list[SessionStub]:
    return []


@router.get("/{session_id}", response_model=SessionStub)
async def get_session(session_id: str) -> SessionStub:
    raise HTTPException(status_code=404, detail=f"Session {session_id!r} not found")


@router.get("/{session_id}/logs", response_model=list[LogStub])
async def get_session_logs(session_id: str) -> list[LogStub]:
    return []
