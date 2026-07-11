from dataclasses import replace
from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException

from src.app.learning_cycle import CycleBusyError, LearningCycleOrchestrator
from src.app.robot_controller import (
    EmergencyStillActive,
    LogWriterProtocol,
    RobotController,
)
from src.app.training_service import train_and_apply
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import FeatureSpec
from src.domain.pid_tuning import build_tuning_trajectory_from_mode, tuning_cost
from src.infra.settings import LearningSettings
from src.models.calibration import CalibrationResult
from src.models.drive_log import DriveSession
from src.models.system_state import RobotState
from src.web.deps import (
    ILCRepoProtocol,
    ModeRepoProtocol,
    ProfileRepoProtocol,
    ScheduleRepoProtocol,
    SessionRepoProtocol,
    get_controller,
    get_cycle_orchestrator,
    get_feature_spec,
    get_ilc_repo,
    get_learning_settings,
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
    CycleProgressSchema,
    DriveSessionResponse,
    DynamicsParamsSchema,
    FeedforwardParamsSchema,
    ILCStatusResponse,
    JogRequest,
    JogResponse,
    LearningCycleAbortResponse,
    LearningCycleStartRequest,
    LearningCycleStartResponse,
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
    return DriveSessionResponse.model_validate(session)


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
    await controller.initialize()
    return {"status": "ok"}


@router.post("/arm", status_code=200)
async def arm_drive(controller: Controller) -> dict[str, str]:
    """自動走行を準備する。READY → PRE_CHECK 遷移。

    走行前チェック（車速0含む）を実施し、合格したら停車保持ブレーキまで踏み込んで
    確認待ち（PRE_CHECK）にする。フロントは確認ポップアップを表示し、「はい」で
    /start、「いいえ」で /cancel を呼ぶ（学習運転と同じ arm フロー）。
    """
    await controller.arm_auto_drive()
    return {"status": "armed"}


@router.post("/cancel", status_code=200)
async def cancel_drive(controller: Controller) -> dict[str, str]:
    """自動走行を中止する（確認ポップアップ「いいえ」）。PRE_CHECK → READY。

    arm でかけた保持ブレーキをリリースする。セッションは未開始のためログは残らない。
    """
    await controller.cancel_auto_drive()
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
    session = await controller.start_auto_drive(
        req.mode_id, mode=mode, profile=profile, log_writer=log_writer
    )
    return _to_session_response(session)


@router.post("/pause", status_code=200)
async def pause_drive(controller: Controller) -> dict[str, str]:
    """自動走行を一時停止する。RUNNING → PAUSED。

    基準速度タイムラインを凍結し、一時停止した瞬間の目標車速を保持して走り続ける。
    フロントの「一時停止」ボタンから呼ぶ。
    """
    await controller.pause_auto_drive()
    return {"status": "paused"}


@router.post("/resume", status_code=200)
async def resume_drive(controller: Controller) -> dict[str, str]:
    """一時停止中の自動走行を再開する。PAUSED → RUNNING。フロントの「走行再開」ボタンから呼ぶ。"""
    await controller.resume_auto_drive()
    return {"status": "running"}


@router.post("/stop", status_code=200)
async def stop_drive(controller: Controller) -> dict[str, str]:
    await controller.stop()
    return {"status": "ok"}


@router.post("/emergency", status_code=200)
async def emergency_stop(controller: Controller) -> dict[str, str]:
    await controller.emergency_stop()
    return {"status": "ok"}


@router.post("/reset-emergency", status_code=200)
async def reset_emergency(controller: Controller) -> dict[str, str]:
    try:
        await controller.reset_emergency()
    except EmergencyStillActive as e:
        # 物理スイッチ未解除。423 Locked で「まだ解除できない」ことを表す。
        raise HTTPException(status_code=423, detail=str(e)) from e
    return {"status": "ok"}


@router.post("/calibrate", response_model=CalibrationResultResponse)
async def run_calibration(controller: Controller) -> CalibrationResultResponse:
    result = await controller.run_calibration()
    return _to_calibration_result_response(result)


@router.post("/calib/jog", response_model=JogResponse)
async def calib_jog(req: JogRequest, controller: Controller) -> JogResponse:
    """キャリブレーション中に軸をジョグ移動する。READY から CALIBRATING へ自動遷移。"""
    try:
        pos = await controller.jog_axis(req.axis, req.step)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/calib/home", response_model=JogResponse)
async def calib_home(req: AxisRequest, controller: Controller) -> JogResponse:
    """キャリブレーション中に軸を原点復帰する。READY から CALIBRATING へ自動遷移。"""
    try:
        pos = await controller.home_axis(req.axis)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/calib/set-zero", response_model=JogResponse)
async def calib_set_zero(req: AxisRequest, controller: Controller) -> JogResponse:
    """現在位置をゼロ点として記録する。CALIBRATING 状態でのみ呼べる。"""
    try:
        pos = await controller.calib_set_zero(req.axis)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/calib/set-full", response_model=JogResponse)
async def calib_set_full(req: AxisRequest, controller: Controller) -> JogResponse:
    """現在位置をフル点として記録する。CALIBRATING 状態でのみ呼べる。"""
    try:
        pos = await controller.calib_set_full(req.axis)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/calib/save", response_model=CalibrationResultResponse)
async def save_calibration(controller: Controller) -> CalibrationResultResponse:
    """手動設定したゼロ/フル点でキャリブレーションを保存する。CALIBRATING → READY。"""
    result = await controller.save_manual_calibration()
    return _to_calibration_result_response(result)


@router.post("/manual/jog", response_model=JogResponse)
async def manual_jog(req: JogRequest, controller: Controller) -> JogResponse:
    """手動運転中に軸をジョグ移動する。MANUAL 状態でのみ呼べる。"""
    try:
        pos = await controller.jog_axis(req.axis, req.step)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/manual/home", response_model=JogResponse)
async def manual_home(req: AxisRequest, controller: Controller) -> JogResponse:
    """手動運転中に軸を原点復帰する。MANUAL 状態でのみ呼べる。"""
    try:
        pos = await controller.home_axis(req.axis)
    except ValueError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return JogResponse(position=pos)


@router.post("/manual/start", response_model=DriveSessionResponse)
async def start_manual(
    controller: Controller,
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> DriveSessionResponse:
    session = await controller.start_manual(log_writer=log_writer)
    return _to_session_response(session)


@router.post("/manual/stop", status_code=200)
async def stop_manual(controller: Controller) -> dict[str, str]:
    await controller.stop_manual()
    return {"status": "ok"}


@router.post("/learning/arm", status_code=200)
async def arm_learning_drive(controller: Controller) -> dict[str, str]:
    """学習運転を準備する。READY → PRE_CHECK 遷移。

    走行前チェック（車速0含む）を実施し、合格したら停車保持ブレーキまで踏み込んで
    確認待ち（PRE_CHECK）にする。フロントは確認ポップアップを表示し、「はい」で
    /learning/start、「いいえ」で /learning/cancel を呼ぶ。
    """
    await controller.arm_learning_drive()
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
    session = await controller.start_learning_drive(log_writer=log_writer)
    return _to_session_response(session)


@router.post("/learning/cancel", status_code=200)
async def cancel_learning_drive(controller: Controller) -> dict[str, str]:
    """学習運転を中止する（確認ポップアップ「いいえ」）。PRE_CHECK → READY。

    arm でかけた保持ブレーキをリリースする。セッションは未開始のためログは残らない。
    """
    await controller.cancel_learning_drive()
    return {"status": "cancelled"}


@router.post("/learning/train", response_model=TrainModelResponse)
async def train_learning_model(
    req: TrainModelRequest,
    controller: Controller,
    profile_repo: Annotated[ProfileRepoProtocol, Depends(get_profile_repo)],
    session_repo: Annotated[SessionRepoProtocol, Depends(get_session_repo)],
    feature_spec: Annotated[FeatureSpec, Depends(get_feature_spec)],
) -> TrainModelResponse:
    """直近の学習走行ログから先読み 多項式Ridge 逆モデルを学習し、プロファイルに紐づけて保存する。

    再学習も同一エンドポイントで実行できる（model_path を上書き更新）。
    学習は CPU 負荷の高い同期処理のため to_thread で実行し、50ms 制御ループ・
    WebSocket 配信・非常停止コールバックが動くイベントループをブロックしない。
    実処理は training_service.train_and_apply に委譲する（2段階学習フローの
    オーケストレータとロジックを共有する）。
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

    # session_ids を明示指定した場合はそれを優先する（再学習・複数選択の用途）。
    # 次に cycle_id 指定（サイクル内全セッションで学習）。いずれも無ければ
    # 「直近に実施した学習走行」のみを既定とする（過去の旧データ混在を避ける）。
    session_ids = req.session_ids
    if not session_ids and req.cycle_id:
        session_ids = await session_repo.list_session_ids_for_cycle(req.cycle_id)
        if not session_ids:
            raise HTTPException(
                status_code=422,
                detail=f"学習サイクル {req.cycle_id!r} にセッションがありません。",
            )
    if not session_ids:
        latest = await session_repo.latest_learning_session_id(req.profile_id)
        if latest is None:
            raise HTTPException(
                status_code=422,
                detail="学習走行のログがありません。先に学習走行を実施してください。",
            )
        session_ids = [latest]

    try:
        result = await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=req.profile_id,
            session_ids=session_ids,
            feature_spec=feature_spec,
        )
    except LearningDataError as e:
        raise HTTPException(status_code=422, detail=str(e)) from e
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e

    return TrainModelResponse(
        model_path=result.model_path,
        metrics=result.metrics,
        feedforward_params=FeedforwardParamsSchema.model_validate(result.feedforward_params),
        pid_gains=PIDGainsSchema.model_validate(result.pid_gains),
        pid_auto_tuned=result.pid_auto_tuned,
        dynamics_params=DynamicsParamsSchema.model_validate(result.dynamics_params),
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
    kpi = await controller.run_pid_validation(profile, log_writer)
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
    mode_repo: Annotated[ModeRepoProtocol, Depends(get_mode_repo)],
    log_writer: Annotated[LogWriterProtocol | None, Depends(get_log_writer)],
) -> PidRefineResponse:
    """規定パターン（または指定モード代表区間）を反復走行し座標降下で PID を絞り込む。"""
    profile = await profile_repo.get_by_id(req.profile_id)
    if profile is None:
        raise HTTPException(
            status_code=404, detail=f"プロファイル {req.profile_id!r} が見つかりません"
        )
    # mode_id 指定時は本番モードの代表区間で評価する（適合結果の転移性向上）。
    tuning_mode = None
    if req.mode_id is not None:
        mode = await mode_repo.get_by_id(req.mode_id)
        if mode is None:
            raise HTTPException(
                status_code=404, detail=f"モード {req.mode_id!r} が見つかりません"
            )
        tuning_mode = build_tuning_trajectory_from_mode(mode)
    best, history = await controller.run_pid_tuning_session(
        profile, log_writer, max_runs=req.max_runs, mode=tuning_mode
    )

    # 最良ゲイン・PID 先読み補償秒数を永続化し、アクティブプロファイルの制御スタックへ反映する。
    profile.pid_gains = best.gains
    profile.dynamics_params = replace(profile.dynamics_params, pid_preview_s=best.pid_preview_s)
    updated = await profile_repo.update(profile)
    controller.refresh_active_profile(updated if updated is not None else profile)

    return PidRefineResponse(
        pid_gains=PIDGainsSchema(kp=best.kp, ki=best.ki, kd=best.kd),
        pid_preview_s=best.pid_preview_s,
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
        raise HTTPException(status_code=400, detail="schedule_id が UUID 形式ではありません") from e
    if schedule is None:
        raise HTTPException(
            status_code=404, detail=f"スケジュール {req.schedule_id!r} が見つかりません"
        )
    profile = controller.get_active_profile()
    session = await controller.start_schedule_drive(
        schedule, profile=profile, log_writer=log_writer
    )
    return _to_session_response(session)


@router.post("/schedule/stop", status_code=200)
async def stop_schedule_drive(controller: Controller) -> dict[str, str]:
    """タイムスケジュール走行を停止する。RUNNING → READY。"""
    await controller.stop_schedule_drive()
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


def _to_cycle_progress_schema(orchestrator: LearningCycleOrchestrator) -> CycleProgressSchema:
    return CycleProgressSchema.model_validate(orchestrator.progress)


@router.post("/learning-cycle/arm", status_code=200)
async def arm_learning_cycle(
    controller: Controller,
    orchestrator: Annotated[LearningCycleOrchestrator, Depends(get_cycle_orchestrator)],
) -> dict[str, str]:
    """学習サイクルを準備する(自動運転と同じ arm フロー)。READY → PRE_CHECK。

    停車保持ブレーキ踏込・車速0収束待ち・走行前チェックを実施し、合格したら確認待ち
    (PRE_CHECK)にする。フロントは確認ポップアップを表示し、「はい」で
    /learning-cycle/start、「いいえ」で /learning-cycle/cancel を呼ぶ。
    """
    profile = controller.get_active_profile()
    if profile is None:
        raise HTTPException(status_code=404, detail="プロファイルが選択されていません")
    try:
        await orchestrator.arm(profile.id)
    except CycleBusyError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    except ValueError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    return {"status": "armed"}


@router.post("/learning-cycle/cancel", status_code=200)
async def cancel_learning_cycle(
    orchestrator: Annotated[LearningCycleOrchestrator, Depends(get_cycle_orchestrator)],
) -> dict[str, str]:
    """arm 済みの学習サイクルを中止する(確認ポップアップ「いいえ」)。PRE_CHECK → READY。"""
    await orchestrator.cancel()
    return {"status": "cancelled"}


@router.post("/learning-cycle/start", response_model=LearningCycleStartResponse, status_code=202)
async def start_learning_cycle(
    req: LearningCycleStartRequest,
    orchestrator: Annotated[LearningCycleOrchestrator, Depends(get_cycle_orchestrator)],
    learning_settings: Annotated[LearningSettings, Depends(get_learning_settings)],
    feature_spec: Annotated[FeatureSpec, Depends(get_feature_spec)],
) -> LearningCycleStartResponse:
    """arm 済みの学習サイクル(学習運転→訓練→PID適合→再学習→PID適合)を開始する
    (確認ポップアップ「はい」)。

    以降のフェーズはバックグラウンドで進行し、進捗は `GET /learning-cycle/status` または
    WebSocket(`cycle_progress`)で参照する。
    """
    stage1 = req.refine_runs_stage1 or learning_settings.refine_runs_stage1
    stage2 = req.refine_runs_stage2 or learning_settings.refine_runs_stage2
    cycle_id = await orchestrator.start(
        stage1, stage2, feature_spec=feature_spec, target_mode_id=req.target_mode_id
    )
    return LearningCycleStartResponse(cycle_id=cycle_id, status="started")


@router.post("/learning-cycle/abort", response_model=LearningCycleAbortResponse, status_code=200)
async def abort_learning_cycle(
    orchestrator: Annotated[LearningCycleOrchestrator, Depends(get_cycle_orchestrator)],
) -> LearningCycleAbortResponse:
    """実行中の学習サイクルを中断する。実行中でない場合は 409。"""
    await orchestrator.abort()
    return LearningCycleAbortResponse(status="aborting")


@router.get("/learning-cycle/status", response_model=CycleProgressSchema)
async def get_learning_cycle_status(
    orchestrator: Annotated[LearningCycleOrchestrator, Depends(get_cycle_orchestrator)],
) -> CycleProgressSchema:
    return _to_cycle_progress_schema(orchestrator)


# ── ILC（反復学習制御） ────────────────────────────────────────────────
ILCRepo = Annotated[ILCRepoProtocol, Depends(get_ilc_repo)]


def _to_ilc_status(profile_id: str, mode_id: str, rec: object | None) -> ILCStatusResponse:
    """ILCRecord（または None）を状態レスポンスへ写像する。未学習は既定値。"""
    if rec is None:
        return ILCStatusResponse(
            profile_id=profile_id,
            mode_id=mode_id,
            enabled=True,  # 未登録は既定で有効（走行完了時に学習開始）
            iteration=0,
            has_table=False,
            best_p95_kmh=None,
            kpi_history=[],
        )
    table = rec.table  # type: ignore[attr-defined]
    return ILCStatusResponse(
        profile_id=profile_id,
        mode_id=mode_id,
        enabled=rec.enabled,  # type: ignore[attr-defined]
        iteration=table.iteration,
        has_table=bool(table.efforts),
        best_p95_kmh=table.best_p95_kmh,
        kpi_history=list(rec.kpi_history),  # type: ignore[attr-defined]
    )


@router.get("/ilc/{profile_id}/{mode_id}", response_model=ILCStatusResponse)
async def get_ilc_status(profile_id: str, mode_id: str, ilc_repo: ILCRepo) -> ILCStatusResponse:
    """profile×mode の ILC 状態（反復回数・有効フラグ・最良p95・履歴）を返す。"""
    rec = await ilc_repo.get(profile_id, mode_id)
    return _to_ilc_status(profile_id, mode_id, rec)


@router.post("/ilc/{profile_id}/{mode_id}/enable", response_model=ILCStatusResponse)
async def enable_ilc(profile_id: str, mode_id: str, ilc_repo: ILCRepo) -> ILCStatusResponse:
    """ILC を有効化する。"""
    await ilc_repo.set_enabled(profile_id, mode_id, True)
    return _to_ilc_status(profile_id, mode_id, await ilc_repo.get(profile_id, mode_id))


@router.post("/ilc/{profile_id}/{mode_id}/disable", response_model=ILCStatusResponse)
async def disable_ilc(profile_id: str, mode_id: str, ilc_repo: ILCRepo) -> ILCStatusResponse:
    """ILC を無効化する（補正を適用せず学習もしない）。"""
    await ilc_repo.set_enabled(profile_id, mode_id, False)
    return _to_ilc_status(profile_id, mode_id, await ilc_repo.get(profile_id, mode_id))


@router.post("/ilc/{profile_id}/{mode_id}/reset", response_model=ILCStatusResponse)
async def reset_ilc(profile_id: str, mode_id: str, ilc_repo: ILCRepo) -> ILCStatusResponse:
    """補正テーブルを削除して反復 0 からやり直す。"""
    await ilc_repo.reset(profile_id, mode_id)
    return _to_ilc_status(profile_id, mode_id, await ilc_repo.get(profile_id, mode_id))
