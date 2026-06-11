from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException

from src.models.drive_log import DriveSession
from src.web.deps import SessionRepoProtocol, get_session_repo
from src.web.schemas import LogResponse, SessionResponse

router = APIRouter(prefix="/api/v1/sessions", tags=["sessions"])

SessionRepo = Annotated[SessionRepoProtocol, Depends(get_session_repo)]


def _to_response(session: DriveSession) -> SessionResponse:
    return SessionResponse(
        id=session.id,
        profile_id=session.profile_id,
        mode_id=session.mode_id,
        run_type=session.run_type,
        started_at=session.started_at,
        ended_at=session.ended_at,
        status=session.status,
    )


@router.get("/", response_model=list[SessionResponse])
async def list_sessions(repo: SessionRepo) -> list[SessionResponse]:
    sessions = await repo.list_all()
    return [_to_response(s) for s in sessions]


@router.get("/{session_id}", response_model=SessionResponse)
async def get_session(session_id: str, repo: SessionRepo) -> SessionResponse:
    session = await repo.get_by_id(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail=f"セッション {session_id!r} が見つかりません")
    return _to_response(session)


@router.get("/{session_id}/logs", response_model=list[LogResponse])
async def get_session_logs(session_id: str, repo: SessionRepo) -> list[LogResponse]:
    logs = await repo.list_logs(session_id)
    return [
        LogResponse(
            id=log.id,
            session_id=log.session_id,
            timestamp=log.timestamp,
            ref_speed_kmh=log.ref_speed_kmh,
            actual_speed_kmh=log.actual_speed_kmh,
            accel_opening=log.accel_opening,
            brake_opening=log.brake_opening,
            accel_pos=log.accel_pos,
            brake_pos=log.brake_pos,
            accel_current=log.accel_current,
            brake_current=log.brake_current,
        )
        for log in logs
    ]
