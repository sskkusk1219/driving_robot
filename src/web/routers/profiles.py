from dataclasses import fields
from datetime import UTC, datetime
from typing import Annotated
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException

from src.app.robot_controller import RobotController
from src.models.profile import (
    DynamicsParams,
    FeedforwardParams,
    PIDGains,
    StopConfig,
    VehicleProfile,
)
from src.web.deps import ProfileRepoProtocol, get_controller, get_profile_repo
from src.web.schemas import (
    CalibrationDataResponse,
    DynamicsParamsSchema,
    FeedforwardParamsSchema,
    PIDGainsSchema,
    ProfileCreateRequest,
    ProfileResponse,
    ProfileUpdateRequest,
    StopConfigSchema,
)

router = APIRouter(prefix="/api/v1/profiles", tags=["profiles"])

ProfileRepo = Annotated[ProfileRepoProtocol, Depends(get_profile_repo)]
Controller = Annotated[RobotController, Depends(get_controller)]


def _ffp_from_schema(s: FeedforwardParamsSchema) -> FeedforwardParams:
    return FeedforwardParams(**{f.name: getattr(s, f.name) for f in fields(FeedforwardParams)})


def _dyn_from_schema(s: DynamicsParamsSchema) -> DynamicsParams:
    return DynamicsParams(**{f.name: getattr(s, f.name) for f in fields(DynamicsParams)})


def _to_response(p: VehicleProfile) -> ProfileResponse:
    calib = None
    if p.calibration is not None:
        calib = CalibrationDataResponse(
            accel_zero_pos=p.calibration.accel_zero_pos,
            accel_full_pos=p.calibration.accel_full_pos,
            accel_stroke=p.calibration.accel_stroke,
            brake_zero_pos=p.calibration.brake_zero_pos,
            brake_full_pos=p.calibration.brake_full_pos,
            brake_stroke=p.calibration.brake_stroke,
            calibrated_at=p.calibration.calibrated_at,
            is_valid=p.calibration.is_valid,
        )
    return ProfileResponse(
        id=p.id,
        name=p.name,
        max_accel_opening=p.max_accel_opening,
        max_brake_opening=p.max_brake_opening,
        max_speed=p.max_speed,
        max_decel_g=p.max_decel_g,
        pid_gains=PIDGainsSchema(kp=p.pid_gains.kp, ki=p.pid_gains.ki, kd=p.pid_gains.kd),
        stop_config=StopConfigSchema(
            deviation_threshold_kmh=p.stop_config.deviation_threshold_kmh,
            deviation_duration_s=p.stop_config.deviation_duration_s,
        ),
        calibration=calib,
        model_path=p.model_path,
        feedforward_params=FeedforwardParamsSchema.model_validate(p.feedforward_params),
        dynamics_params=DynamicsParamsSchema.model_validate(p.dynamics_params),
        created_at=p.created_at,
        updated_at=p.updated_at,
    )


@router.get("/", response_model=list[ProfileResponse])
async def list_profiles(repo: ProfileRepo) -> list[ProfileResponse]:
    profiles = await repo.list_all()
    return [_to_response(p) for p in profiles]


@router.post("/", response_model=ProfileResponse, status_code=201)
async def create_profile(req: ProfileCreateRequest, repo: ProfileRepo) -> ProfileResponse:
    now = datetime.now(tz=UTC)
    profile = VehicleProfile(
        id=str(uuid4()),
        name=req.name,
        max_accel_opening=req.max_accel_opening,
        max_brake_opening=req.max_brake_opening,
        max_speed=req.max_speed,
        max_decel_g=req.max_decel_g,
        pid_gains=PIDGains(kp=req.pid_gains.kp, ki=req.pid_gains.ki, kd=req.pid_gains.kd),
        stop_config=StopConfig(
            deviation_threshold_kmh=req.stop_config.deviation_threshold_kmh,
            deviation_duration_s=req.stop_config.deviation_duration_s,
        ),
        calibration=None,
        model_path=req.model_path,
        created_at=now,
        updated_at=now,
        feedforward_params=(
            _ffp_from_schema(req.feedforward_params)
            if req.feedforward_params is not None
            else FeedforwardParams()
        ),
        dynamics_params=(
            _dyn_from_schema(req.dynamics_params)
            if req.dynamics_params is not None
            else DynamicsParams()
        ),
    )
    created = await repo.create(profile)
    return _to_response(created)


@router.get("/{profile_id}", response_model=ProfileResponse)
async def get_profile(profile_id: str, repo: ProfileRepo) -> ProfileResponse:
    try:
        profile = await repo.get_by_id(profile_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="profile_id が UUID 形式ではありません")
    if profile is None:
        raise HTTPException(status_code=404, detail=f"プロファイル {profile_id!r} が見つかりません")
    return _to_response(profile)


@router.put("/{profile_id}", response_model=ProfileResponse)
async def update_profile(
    profile_id: str, req: ProfileUpdateRequest, repo: ProfileRepo, controller: Controller
) -> ProfileResponse:
    try:
        existing = await repo.get_by_id(profile_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="profile_id が UUID 形式ではありません")
    if existing is None:
        raise HTTPException(status_code=404, detail=f"プロファイル {profile_id!r} が見つかりません")
    merged = VehicleProfile(
        id=existing.id,
        name=req.name if req.name is not None else existing.name,
        max_accel_opening=(
            req.max_accel_opening
            if req.max_accel_opening is not None
            else existing.max_accel_opening
        ),
        max_brake_opening=(
            req.max_brake_opening
            if req.max_brake_opening is not None
            else existing.max_brake_opening
        ),
        max_speed=req.max_speed if req.max_speed is not None else existing.max_speed,
        max_decel_g=req.max_decel_g if req.max_decel_g is not None else existing.max_decel_g,
        pid_gains=(
            PIDGains(kp=req.pid_gains.kp, ki=req.pid_gains.ki, kd=req.pid_gains.kd)
            if req.pid_gains is not None
            else existing.pid_gains
        ),
        stop_config=(
            StopConfig(
                deviation_threshold_kmh=req.stop_config.deviation_threshold_kmh,
                deviation_duration_s=req.stop_config.deviation_duration_s,
            )
            if req.stop_config is not None
            else existing.stop_config
        ),
        calibration=existing.calibration,
        model_path=req.model_path if req.model_path is not None else existing.model_path,
        created_at=existing.created_at,
        updated_at=existing.updated_at,
        feedforward_params=(
            _ffp_from_schema(req.feedforward_params)
            if req.feedforward_params is not None
            else existing.feedforward_params
        ),
        dynamics_params=(
            _dyn_from_schema(req.dynamics_params)
            if req.dynamics_params is not None
            else existing.dynamics_params
        ),
    )
    updated = await repo.update(merged)
    if updated is None:
        raise HTTPException(status_code=404, detail=f"プロファイル {profile_id!r} が見つかりません")
    # 編集が「アクティブプロファイル」なら制御スタック（in-memory）へ即時反映する。
    # これをしないと stop_brake_opening_pct 等を編集しても arm のブレーキ踏込量が
    # 旧値のまま変わらない（コントローラが古い _active_profile を保持し続けるため）。
    controller.refresh_active_profile(updated)
    return _to_response(updated)


@router.delete("/{profile_id}", status_code=204)
async def delete_profile(profile_id: str, repo: ProfileRepo) -> None:
    try:
        deleted = await repo.delete(profile_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="profile_id が UUID 形式ではありません")
    if not deleted:
        raise HTTPException(status_code=404, detail=f"プロファイル {profile_id!r} が見つかりません")
