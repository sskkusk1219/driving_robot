from datetime import UTC, datetime
from typing import Annotated
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException

from src.infra.db import DuplicateNameError
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile
from src.web.deps import ProfileRepoProtocol, get_profile_repo
from src.web.schemas import (
    CalibrationDataResponse,
    FeedforwardParamsSchema,
    PIDGainsSchema,
    ProfileCreateRequest,
    ProfileResponse,
    ProfileUpdateRequest,
    StopConfigSchema,
)

router = APIRouter(prefix="/api/v1/profiles", tags=["profiles"])

ProfileRepo = Annotated[ProfileRepoProtocol, Depends(get_profile_repo)]


def _ffp_to_schema(ffp: FeedforwardParams) -> FeedforwardParamsSchema:
    return FeedforwardParamsSchema(
        creep_speed_kmh=ffp.creep_speed_kmh,
        creep_rate_kmhs=ffp.creep_rate_kmhs,
        engine_brake_decel_kmhs=ffp.engine_brake_decel_kmhs,
        stop_brake_opening_pct=ffp.stop_brake_opening_pct,
        brake_deadband_pct=ffp.brake_deadband_pct,
        accel_deadband_pct=ffp.accel_deadband_pct,
    )


def _ffp_from_schema(s: FeedforwardParamsSchema) -> FeedforwardParams:
    return FeedforwardParams(
        creep_speed_kmh=s.creep_speed_kmh,
        creep_rate_kmhs=s.creep_rate_kmhs,
        engine_brake_decel_kmhs=s.engine_brake_decel_kmhs,
        stop_brake_opening_pct=s.stop_brake_opening_pct,
        brake_deadband_pct=s.brake_deadband_pct,
        accel_deadband_pct=s.accel_deadband_pct,
    )


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
        feedforward_params=_ffp_to_schema(p.feedforward_params),
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
    )
    try:
        created = await repo.create(profile)
    except DuplicateNameError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
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
    profile_id: str, req: ProfileUpdateRequest, repo: ProfileRepo
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
    )
    updated = await repo.update(merged)
    if updated is None:
        raise HTTPException(status_code=404, detail=f"プロファイル {profile_id!r} が見つかりません")
    return _to_response(updated)


@router.delete("/{profile_id}", status_code=204)
async def delete_profile(profile_id: str, repo: ProfileRepo) -> None:
    try:
        deleted = await repo.delete(profile_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="profile_id が UUID 形式ではありません")
    if not deleted:
        raise HTTPException(status_code=404, detail=f"プロファイル {profile_id!r} が見つかりません")
