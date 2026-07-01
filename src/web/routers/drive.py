import asyncio
from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException

from src.app.robot_controller import (
    EmergencyStillActive,
    InvalidStateTransition,
    LogWriterProtocol,
    PidTuningAborted,
    PreCheckFailed,
    RobotController,
)
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import estimate_dynamics_params, train_inverse_model
from src.domain.pid_tuning import compute_pid_gains_simc, identify_fopdt, tuning_cost
from src.models.calibration import CalibrationResult
from src.models.drive_log import DriveSession
from src.models.system_state import RobotState
from src.web.deps import (
    ModeRepoProtocol,
    ProfileRepoProtocol,
    ScheduleRepoProtocol,
    SessionRepoProtocol,
    get_controller,
    get_log_writer,
    get_mode_repo,
    get_profile_repo,
    get_schedule_repo,
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
    PIDGainsSchema,
    PidRefineRequest,
    PidRefineResponse,
    PidValidateRequest,
    PidValidateResponse,
    SelectProfileRequest,
    StartDriveRequest,
    StartScheduleRequest,
    SystemStateResponse,
    TrainModelRequest,
    TrainModelResponse,
)

router = APIRouter(prefix="/api/v1/drive", tags=["drive"])

Controller = Annotated[RobotController, Depends(get_controller)]


def _to_session_response(session: DriveSession) -> DriveSessionResponse:
    return DriveSessionResponse(
        id=session.id,
        profile_id=session.profile_id,
        mode_id=session.mode_id,
        run_type=session.run_type,
        started_at=session.started_at,
        ended_at=session.ended_at,
        status=session.status,
    )


def _to_calibration_result_response(result: CalibrationResult) -> CalibrationResultResponse:
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


@router.post("/arm", status_code=200)
async def arm_drive(controller: Controller) -> dict[str, str]:
    """自動走行を準備する。READY → PRE_CHECK 遷移。

    走行前チェック（車速0含む）を実施し、合格したら停車保持ブレーキまで踏み込んで
    確認待ち（PRE_CHECK）にする。フロントは確認ポップアップを表示し、「はい」で
    /start、「いいえ」で /cancel を呼ぶ（学習運転と同じ arm フロー）。
    """
    try:
        await controller.arm_auto_drive()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    return {"status": "armed"}


@router.post("/cancel", status_code=200)
async def cancel_drive(controller: Controller) -> dict[str, str]:
    """自動走行を中止する（確認ポップアップ「いいえ」）。PRE_CHECK → READY。

    arm でかけた保持ブレーキをリリースする。セッションは未開始のためログは残らない。
    """
    try:
        await controller.cancel_auto_drive()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "cancelled"}


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
    if mode is None:
        # mode 不在のまま start_auto_drive を呼ぶと RUNNING 遷移後に DriveLoop が
        # 起動しない／DB の FK 違反で 500 になるため、状態遷移前に拒否する。
        raise HTTPException(status_code=404, detail=f"走行モード {req.mode_id!r} が見つかりません")
    profile = controller.get_active_profile()
    try:
        session = await controller.start_auto_drive(
            req.mode_id, mode=mode, profile=profile, log_writer=log_writer
        )
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    return _to_session_response(session)


@router.post("/pause", status_code=200)
async def pause_drive(controller: Controller) -> dict[str, str]:
    """自動走行を一時停止する。RUNNING → PAUSED。

    基準速度タイムラインを凍結し、一時停止した瞬間の目標車速を保持して走り続ける。
    フロントの「一時停止」ボタンから呼ぶ。
    """
    try:
        await controller.pause_auto_drive()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "paused"}


@router.post("/resume", status_code=200)
async def resume_drive(controller: Controller) -> dict[str, str]:
    """一時停止中の自動走行を再開する。PAUSED → RUNNING。フロントの「走行再開」ボタンから呼ぶ。"""
    try:
        await controller.resume_auto_drive()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "running"}


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
    return _to_calibration_result_response(result)


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
    return _to_calibration_result_response(result)


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
async def start_manual(
    controller: Controller,
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> DriveSessionResponse:
    try:
        session = await controller.start_manual(log_writer=log_writer)
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    return _to_session_response(session)


@router.post("/manual/stop", status_code=200)
async def stop_manual(controller: Controller) -> dict[str, str]:
    try:
        await controller.stop_manual()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "ok"}


@router.post("/learning/arm", status_code=200)
async def arm_learning_drive(controller: Controller) -> dict[str, str]:
    """学習運転を準備する。READY → PRE_CHECK 遷移。

    走行前チェック（車速0含む）を実施し、合格したら停車保持ブレーキまで踏み込んで
    確認待ち（PRE_CHECK）にする。フロントは確認ポップアップを表示し、「はい」で
    /learning/start、「いいえ」で /learning/cancel を呼ぶ。
    """
    try:
        await controller.arm_learning_drive()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    return {"status": "armed"}


@router.post("/learning/start", response_model=DriveSessionResponse)
async def start_learning_drive(
    controller: Controller,
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> DriveSessionResponse:
    """学習走行を開始する（確認ポップアップ「はい」）。arm 後（PRE_CHECK）→ RUNNING。

    開度パターン列を LearningLoop で開ループ実行し、走行ログを `drive_logs` に
    連続記録する（DB 利用時）。
    """
    try:
        session = await controller.start_learning_drive(log_writer=log_writer)
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    return _to_session_response(session)


@router.post("/learning/cancel", status_code=200)
async def cancel_learning_drive(controller: Controller) -> dict[str, str]:
    """学習運転を中止する（確認ポップアップ「いいえ」）。PRE_CHECK → READY。

    arm でかけた保持ブレーキをリリースする。セッションは未開始のためログは残らない。
    """
    try:
        await controller.cancel_learning_drive()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "cancelled"}


@router.post("/learning/train", response_model=TrainModelResponse)
async def train_learning_model(
    req: TrainModelRequest,
    controller: Controller,
    profile_repo: Annotated[ProfileRepoProtocol, Depends(get_profile_repo)],
    session_repo: Annotated[SessionRepoProtocol, Depends(get_session_repo)],
) -> TrainModelResponse:
    """直近の学習走行ログから先読み 多項式Ridge 逆モデルを学習し、プロファイルに紐づけて保存する。

    再学習も同一エンドポイントで実行できる（model_path を上書き更新）。
    学習は CPU 負荷の高い同期処理のため to_thread で実行し、50ms 制御ループ・
    WebSocket 配信・非常停止コールバックが動くイベントループをブロックしない。
    """
    state = controller.get_system_state().robot_state
    if state in (
        RobotState.RUNNING,
        RobotState.PAUSED,
        RobotState.MANUAL,
        RobotState.CALIBRATING,
        RobotState.PRE_CHECK,
    ):
        raise HTTPException(
            status_code=409,
            detail=f"走行・操作中はモデル学習を実行できません (現在: {state})",
        )

    profile = await profile_repo.get_by_id(req.profile_id)
    if profile is None:
        raise HTTPException(
            status_code=404, detail=f"プロファイル {req.profile_id!r} が見つかりません"
        )

    # 学習対象は「直近に実施した学習走行」のみを既定とする（過去の旧データ混在を避ける）。
    # session_ids を明示指定した場合はそれを優先する（再学習・複数選択の用途）。
    session_ids = req.session_ids
    if not session_ids:
        latest = await session_repo.latest_learning_session_id(req.profile_id)
        if latest is None:
            raise HTTPException(
                status_code=422,
                detail="学習走行のログがありません。先に学習走行を実施してください。",
            )
        session_ids = [latest]

    logs = await session_repo.list_logs_for_training(req.profile_id, session_ids)
    try:
        model_path, metrics = await asyncio.to_thread(train_inverse_model, logs, profile)
    except LearningDataError as e:
        raise HTTPException(status_code=422, detail=str(e)) from e

    # 観測可能な物理定数のみログから推定して上書き（不足項目は既存値を保持）
    new_params = await asyncio.to_thread(estimate_dynamics_params, logs, profile.feedforward_params)

    # 学習ログのアクセル保持区間から FOPDT を同定し SIMC で PID を自動適合する。
    # 同定不能（区間不足）なら既存ゲインを保持する。CPU 同期処理のため to_thread。
    fopdt = await asyncio.to_thread(identify_fopdt, logs, profile)
    pid_auto_tuned = False
    if fopdt is not None:
        new_gains = compute_pid_gains_simc(fopdt, profile)
        if new_gains != profile.pid_gains:
            profile.pid_gains = new_gains
            pid_auto_tuned = True

    profile.model_path = model_path
    profile.feedforward_params = new_params
    updated = await profile_repo.update(profile)

    # 学習結果をアクティブプロファイルの制御スタックへ即時反映する。
    # DB だけ更新して in-memory を放置すると、学習直後の走行が旧モデル
    # （または FF なし）で実行される（コードレビュー 2026-06-11 指摘 #10）。
    controller.refresh_active_profile(updated if updated is not None else profile)

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
            switch_hysteresis_pct=new_params.switch_hysteresis_pct,
            accel_reengage_dwell_s=new_params.accel_reengage_dwell_s,
            accel_rate_limit_pct_s=new_params.accel_rate_limit_pct_s,
            brake_rate_limit_pct_s=new_params.brake_rate_limit_pct_s,
            pid_output_limit_pct=new_params.pid_output_limit_pct,
        ),
        pid_gains=PIDGainsSchema(
            kp=profile.pid_gains.kp,
            ki=profile.pid_gains.ki,
            kd=profile.pid_gains.kd,
        ),
        pid_auto_tuned=pid_auto_tuned,
    )


@router.post("/pid-tune/validate", response_model=PidValidateResponse)
async def pid_tune_validate(
    req: PidValidateRequest,
    controller: Controller,
    profile_repo: Annotated[ProfileRepoProtocol, Depends(get_profile_repo)],
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> PidValidateResponse:
    """規定パターンを 1 回走行し、現在の PID ゲインの追従 KPI とコストを返す。"""
    profile = await profile_repo.get_by_id(req.profile_id)
    if profile is None:
        raise HTTPException(
            status_code=404, detail=f"プロファイル {req.profile_id!r} が見つかりません"
        )
    try:
        kpi = await controller.run_pid_validation(profile, log_writer)
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    except PidTuningAborted as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return PidValidateResponse(
        kpi_summary=kpi,
        cost=tuning_cost(kpi),
        pid_gains=PIDGainsSchema(
            kp=profile.pid_gains.kp, ki=profile.pid_gains.ki, kd=profile.pid_gains.kd
        ),
    )


@router.post("/pid-tune/refine", response_model=PidRefineResponse)
async def pid_tune_refine(
    req: PidRefineRequest,
    controller: Controller,
    profile_repo: Annotated[ProfileRepoProtocol, Depends(get_profile_repo)],
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> PidRefineResponse:
    """規定パターンを反復走行し座標降下で PID を絞り込み、最良ゲインを保存する。"""
    profile = await profile_repo.get_by_id(req.profile_id)
    if profile is None:
        raise HTTPException(
            status_code=404, detail=f"プロファイル {req.profile_id!r} が見つかりません"
        )
    try:
        best, history = await controller.run_pid_tuning_session(
            profile, log_writer, max_runs=req.max_runs
        )
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    except PidTuningAborted as e:
        raise HTTPException(status_code=409, detail=str(e)) from e

    # 最良ゲインを永続化し、アクティブプロファイルの制御スタックへ反映する。
    profile.pid_gains = best
    updated = await profile_repo.update(profile)
    controller.refresh_active_profile(updated if updated is not None else profile)

    return PidRefineResponse(
        pid_gains=PIDGainsSchema(kp=best.kp, ki=best.ki, kd=best.kd),
        best_cost=min((h["cost"] for h in history), default=0.0),
        history=history,
    )


@router.post("/schedule/start", response_model=DriveSessionResponse)
async def start_schedule_drive(
    req: StartScheduleRequest,
    controller: Controller,
    schedule_repo: Annotated[ScheduleRepoProtocol, Depends(get_schedule_repo)],
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> DriveSessionResponse:
    """タイムスケジュール走行を開始する。

    ペダル開度とボタンイベントの統合タイムラインを ScheduleLoop で開ループ実行する。
    """
    try:
        schedule = await schedule_repo.get_by_id(req.schedule_id)
    except ValueError as e:
        raise HTTPException(
            status_code=400, detail="schedule_id が UUID 形式ではありません"
        ) from e
    if schedule is None:
        raise HTTPException(
            status_code=404, detail=f"スケジュール {req.schedule_id!r} が見つかりません"
        )
    profile = controller.get_active_profile()
    try:
        session = await controller.start_schedule_drive(
            schedule, profile=profile, log_writer=log_writer
        )
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except PreCheckFailed as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    return _to_session_response(session)


@router.post("/schedule/stop", status_code=200)
async def stop_schedule_drive(controller: Controller) -> dict[str, str]:
    """タイムスケジュール走行を停止する。RUNNING → READY。"""
    try:
        await controller.stop_schedule_drive()
    except InvalidStateTransition as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return {"status": "ok"}


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
