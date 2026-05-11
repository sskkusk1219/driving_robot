from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException

from src.app.robot_controller import InvalidStateTransition, PreCheckFailed, RobotController
from src.web.deps import ProfileRepoProtocol, get_controller, get_profile_repo
from src.web.schemas import (
    CalibrationDataResponse,
    CalibrationResultResponse,
    DriveSessionResponse,
    SelectProfileRequest,
    StartDriveRequest,
    SystemStateResponse,
)

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


@router.post("/calibrate", response_model=CalibrationResultResponse)
async def run_calibration(controller: Controller) -> CalibrationResultResponse:
    try:
        result = await controller.run_calibration()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return CalibrationResultResponse(
        success=result.success,
        error_message=result.error_message,
        data=CalibrationDataResponse(
            accel_zero_pos=result.data.accel_zero_pos,
            accel_full_pos=result.data.accel_full_pos,
            accel_stroke=result.data.accel_stroke,
            brake_zero_pos=result.data.brake_zero_pos,
            brake_full_pos=result.data.brake_full_pos,
            brake_stroke=result.data.brake_stroke,
            calibrated_at=result.data.calibrated_at,
            is_valid=result.data.is_valid,
        )
        if result.data is not None
        else None,
    )


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


@router.post("/learning/start", response_model=DriveSessionResponse)
async def start_learning_drive(controller: Controller) -> DriveSessionResponse:
    """学習走行を開始する。READY → PRE_CHECK → RUNNING 遷移。"""
    try:
        session = await controller.start_learning_drive()
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


@router.post("/select-profile", status_code=200)
async def select_profile(
    req: SelectProfileRequest,
    controller: Controller,
    profile_repo: Annotated[ProfileRepoProtocol, Depends(get_profile_repo)],
) -> dict[str, str]:
    """アクティブプロファイルを選択する。プロファイルが存在しない場合は 404。"""
    profile = await profile_repo.get_by_id(req.profile_id)
    if profile is None:
        raise HTTPException(
            status_code=404, detail=f"プロファイル {req.profile_id!r} が見つかりません"
        )
    controller.select_profile(profile)
    return {"status": "ok", "profile_id": profile.id}
