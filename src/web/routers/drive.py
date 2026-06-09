from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException

from src.app.robot_controller import (
    EmergencyStillActive,
    InvalidStateTransition,
    LogWriterProtocol,
    PreCheckFailed,
    RobotController,
)
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import estimate_dynamics_params, train_inverse_model
from src.web.deps import (
    ModeRepoProtocol,
    ProfileRepoProtocol,
    SessionRepoProtocol,
    get_controller,
    get_log_writer,
    get_mode_repo,
    get_profile_repo,
    get_session_repo,
)
from src.web.schemas import (
    AxisRequest,
    CalibrationDataResponse,
    CalibrationResultResponse,
    DriveSessionResponse,
    FeedforwardParamsSchema,
    JogRequest,
    JogResponse,
    SelectProfileRequest,
    StartDriveRequest,
    SystemStateResponse,
    TrainModelRequest,
    TrainModelResponse,
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
async def start_drive(
    req: StartDriveRequest,
    controller: Controller,
    mode_repo: Annotated[ModeRepoProtocol, Depends(get_mode_repo)],
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> DriveSessionResponse:
    """自動走行を開始する。

    走行モード・選択中プロファイル・LogWriter を解決して DriveLoop を起動し、
    走行ログを `drive_logs` に記録する（DB 利用時）。
    """
    mode = await mode_repo.get_by_id(req.mode_id)
    profile = controller.get_active_profile()
    try:
        session = await controller.start_auto_drive(
            req.mode_id, mode=mode, profile=profile, log_writer=log_writer
        )
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
    except EmergencyStillActive as e:
        # 物理スイッチ未解除。423 Locked で「まだ解除できない」ことを表す。
        raise HTTPException(status_code=423, detail=str(e)) from e
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


@router.post("/calib/jog", response_model=JogResponse)
async def calib_jog(req: JogRequest, controller: Controller) -> JogResponse:
    """キャリブレーション中に軸をジョグ移動する。READY から CALIBRATING へ自動遷移。"""
    try:
        pos = await controller.jog_axis(req.axis, req.step)
    except (InvalidStateTransition, ValueError) as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/calib/home", response_model=JogResponse)
async def calib_home(req: AxisRequest, controller: Controller) -> JogResponse:
    """キャリブレーション中に軸を原点復帰する。READY から CALIBRATING へ自動遷移。"""
    try:
        pos = await controller.home_axis(req.axis)
    except (InvalidStateTransition, ValueError) as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/calib/set-zero", response_model=JogResponse)
async def calib_set_zero(req: AxisRequest, controller: Controller) -> JogResponse:
    """現在位置をゼロ点として記録する。CALIBRATING 状態でのみ呼べる。"""
    try:
        pos = await controller.calib_set_zero(req.axis)
    except (InvalidStateTransition, ValueError) as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/calib/set-full", response_model=JogResponse)
async def calib_set_full(req: AxisRequest, controller: Controller) -> JogResponse:
    """現在位置をフル点として記録する。CALIBRATING 状態でのみ呼べる。"""
    try:
        pos = await controller.calib_set_full(req.axis)
    except (InvalidStateTransition, ValueError) as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/calib/save", response_model=CalibrationResultResponse)
async def save_calibration(controller: Controller) -> CalibrationResultResponse:
    """手動設定したゼロ/フル点でキャリブレーションを保存する。CALIBRATING → READY。"""
    try:
        result = await controller.save_manual_calibration()
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


@router.post("/manual/jog", response_model=JogResponse)
async def manual_jog(req: JogRequest, controller: Controller) -> JogResponse:
    """手動運転中に軸をジョグ移動する。MANUAL 状態でのみ呼べる。"""
    try:
        pos = await controller.jog_axis(req.axis, req.step)
    except (InvalidStateTransition, ValueError) as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/manual/home", response_model=JogResponse)
async def manual_home(req: AxisRequest, controller: Controller) -> JogResponse:
    """手動運転中に軸を原点復帰する。MANUAL 状態でのみ呼べる。"""
    try:
        pos = await controller.home_axis(req.axis)
    except (InvalidStateTransition, ValueError) as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


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
async def start_learning_drive(
    controller: Controller,
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> DriveSessionResponse:
    """学習走行を開始する。READY → PRE_CHECK → RUNNING 遷移。

    学習用基準速度プロファイルを DriveLoop で走行し、走行ログを `drive_logs` に
    記録する（DB 利用時）。
    """
    try:
        session = await controller.start_learning_drive(log_writer=log_writer)
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


@router.post("/learning/train", response_model=TrainModelResponse)
async def train_learning_model(
    req: TrainModelRequest,
    profile_repo: Annotated[ProfileRepoProtocol, Depends(get_profile_repo)],
    session_repo: Annotated[SessionRepoProtocol, Depends(get_session_repo)],
) -> TrainModelResponse:
    """連続走行ログから先読み Ridge 逆モデルを学習し、プロファイルに紐づけて保存する。

    再学習も同一エンドポイントで実行できる（model_path を上書き更新）。
    """
    profile = await profile_repo.get_by_id(req.profile_id)
    if profile is None:
        raise HTTPException(
            status_code=404, detail=f"プロファイル {req.profile_id!r} が見つかりません"
        )

    logs = await session_repo.list_logs_for_training(req.profile_id, req.session_ids)
    try:
        model_path, metrics = train_inverse_model(logs, profile)
    except LearningDataError as e:
        raise HTTPException(status_code=422, detail=str(e)) from e

    # 観測可能な物理定数のみログから推定して上書き（不足項目は既存値を保持）
    new_params = estimate_dynamics_params(logs, profile.feedforward_params)

    profile.model_path = model_path
    profile.feedforward_params = new_params
    await profile_repo.update(profile)

    return TrainModelResponse(
        model_path=model_path,
        metrics=metrics,
        feedforward_params=FeedforwardParamsSchema(
            creep_speed_kmh=new_params.creep_speed_kmh,
            creep_rate_kmhs=new_params.creep_rate_kmhs,
            engine_brake_decel_kmhs=new_params.engine_brake_decel_kmhs,
            stop_brake_opening_pct=new_params.stop_brake_opening_pct,
            brake_deadband_pct=new_params.brake_deadband_pct,
            accel_deadband_pct=new_params.accel_deadband_pct,
        ),
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
