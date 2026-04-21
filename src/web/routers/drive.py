from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException

from src.app.robot_controller import InvalidStateTransition, PreCheckFailed, RobotController
from src.web.deps import get_controller
from src.web.schemas import DriveSessionResponse, StartDriveRequest, SystemStateResponse

router = APIRouter(prefix="/api/v1/drive", tags=["drive"])

Controller = Annotated[RobotController, Depends(get_controller)]


@router.get("/status", response_model=SystemStateResponse)
async def get_status(controller: Controller) -> SystemStateResponse:
    state = controller.get_system_state()
    return SystemStateResponse(
        robot_state=state.robot_state,
        active_profile_id=state.active_profile_id,
        active_session_id=state.active_session_id,
        last_normal_shutdown=state.last_normal_shutdown,
        updated_at=state.updated_at,
    )


@router.post("/initialize", status_code=200)
async def initialize(controller: Controller) -> dict[str, str]:
    try:
        await controller.initialize()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "ok"}


@router.post("/start", response_model=DriveSessionResponse)
async def start_drive(req: StartDriveRequest, controller: Controller) -> DriveSessionResponse:
    try:
        session = await controller.start_auto_drive(req.mode_id)
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    return DriveSessionResponse(
        id=session.id,
        profile_id=session.profile_id,
        mode_id=session.mode_id,
        run_type=session.run_type,
        started_at=session.started_at,
        ended_at=session.ended_at,
        status=session.status,
    )


@router.post("/stop", status_code=200)
async def stop_drive(controller: Controller) -> dict[str, str]:
    try:
        await controller.stop()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "ok"}


@router.post("/emergency", status_code=200)
async def emergency_stop(controller: Controller) -> dict[str, str]:
    try:
        await controller.emergency_stop()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "ok"}


@router.post("/reset-emergency", status_code=200)
async def reset_emergency(controller: Controller) -> dict[str, str]:
    try:
        await controller.reset_emergency()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "ok"}


@router.post("/manual/start", response_model=DriveSessionResponse)
async def start_manual(controller: Controller) -> DriveSessionResponse:
    try:
        session = await controller.start_manual()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    return DriveSessionResponse(
        id=session.id,
        profile_id=session.profile_id,
        mode_id=session.mode_id,
        run_type=session.run_type,
        started_at=session.started_at,
        ended_at=session.ended_at,
        status=session.status,
    )


@router.post("/manual/stop", status_code=200)
async def stop_manual(controller: Controller) -> dict[str, str]:
    try:
        await controller.stop_manual()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "ok"}
