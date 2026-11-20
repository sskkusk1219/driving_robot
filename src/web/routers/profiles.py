# ruff: noqa: E501
"""車（プロファイル）API。段4（RPD_Phase_20261115_PortResearch）で DB の profiles から車両 yaml に置き換えた。

パス（``/api/v1/profiles/*``）は画面の呼び先のまま。ID は車のフォルダ名（vehicles/<ID>/）。
人が決める値（vehicle.*）だけ書ける（ボタンの設定は共通設定へ移した。段11）。学習で決まる feedforward.* は読むだけ。
"""

from __future__ import annotations

from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException

from src.app.job_manager import JobManager
from src.infra.vehicle_store import NEW_VEHICLE_DEFAULTS, VehicleInfo, VehicleStore
from src.web.deps import get_job_manager, get_vehicle_store
from src.web.schemas import (
    ButtonsSchema,
    ProfileCreateRequest,
    ProfileResponse,
    ProfileUpdateRequest,
)

router = APIRouter(prefix="/api/v1/profiles", tags=["profiles"])

Store = Annotated[VehicleStore, Depends(get_vehicle_store)]
Manager = Annotated[JobManager, Depends(get_job_manager)]


def to_response(info: VehicleInfo) -> ProfileResponse:
    v = info.vehicle
    return ProfileResponse(
        id=info.id,
        name=info.name,
        error=info.error,
        max_speed_kmh=v.get("max_speed_kmh"),
        max_decel_g=v.get("max_decel_g"),
        min_accel_opening_pct=v.get("min_accel_opening_pct"),
        max_accel_opening_pct=v.get("max_accel_opening_pct"),
        min_brake_opening_pct=v.get("min_brake_opening_pct"),
        max_brake_opening_pct=v.get("max_brake_opening_pct"),
        min_accel_measured=info.min_accel_measured,
        min_brake_measured=info.min_brake_measured,
        stop_deviation_threshold_kmh=v.get("stop_deviation_threshold_kmh"),
        stop_deviation_duration_s=v.get("stop_deviation_duration_s"),
        test_mass_kg=v.get("test_mass_kg"),
        road_load_a0_n=v.get("road_load_a0_n"),
        road_load_a1_n_per_kmh=v.get("road_load_a1_n_per_kmh"),
        road_load_a2_n_per_kmh2=v.get("road_load_a2_n_per_kmh2"),
        road_load_source=v.get("road_load_source"),
        buttons=ButtonsSchema(**info.buttons),
        learned=info.learned,
        model_path=info.model_path,
        learned_at=info.learned_at,
        candidate=info.candidate,
        feedforward=info.feedforward,
        updated_at=info.updated_at,
    )


@router.get("/", response_model=list[ProfileResponse])
async def list_profiles(store: Store) -> list[ProfileResponse]:
    return [to_response(i) for i in store.list_all()]


@router.post("/", response_model=ProfileResponse, status_code=201)
async def create_profile(req: ProfileCreateRequest, store: Store) -> ProfileResponse:
    values = req.model_dump(exclude={"name", "id"}, exclude_none=True)
    # 作成で渡せるのは雛形の仮値を置き換える数値（諸元・開度）だけ。走行抵抗などは作成後に更新で入れる

    template_values = {k: v for k, v in values.items() if k in NEW_VEHICLE_DEFAULTS}
    info = store.create(
        req.name,
        vehicle_id=req.id,
        values=template_values,
    )
    rest_vehicle = {k: v for k, v in values.items() if k not in NEW_VEHICLE_DEFAULTS}
    if rest_vehicle:
        info = store.update(info.id, vehicle=rest_vehicle)
    return to_response(info)


@router.get("/{profile_id}", response_model=ProfileResponse)
async def get_profile(profile_id: str, store: Store) -> ProfileResponse:
    return to_response(store.read(profile_id))


@router.put("/{profile_id}", response_model=ProfileResponse)
async def update_profile(
    profile_id: str, req: ProfileUpdateRequest, store: Store, manager: Manager
) -> ProfileResponse:
    """人が決める値の更新（送ったキーだけ）。値が core の検証に通らなければ 422（日本語の理由）。"""
    current = manager.current
    if current is not None and current.spec.vehicle_id == profile_id:
        raise HTTPException(status_code=409, detail="この車のジョブが実行中のため変更できません")
    vehicle = req.model_dump(exclude_none=True)
    return to_response(store.update(profile_id, vehicle=vehicle))


@router.delete("/{profile_id}", status_code=204)
async def delete_profile(profile_id: str, store: Store, manager: Manager) -> None:
    """車のフォルダ（yaml・pkl・学習データ）を消す。確認は画面側。この車のジョブが走っていれば 409。"""
    current = manager.current
    if current is not None and current.spec.vehicle_id == profile_id:
        raise HTTPException(status_code=409, detail="この車のジョブが実行中のため削除できません")
    store.delete(profile_id)
