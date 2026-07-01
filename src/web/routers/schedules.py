"""タイムスケジュール（統合タイムライン）CRUD ルーター。

走行モード管理（modes.py）と同じ CRUD パターンに従う。ペダル開度とボタンイベントを
含む統合タイムラインを1エンティティとして扱う。作成・更新は JSON ボディで受ける。
"""

from datetime import UTC, datetime
from typing import Annotated
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException

from src.infra.db import DuplicateNameError
from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule
from src.web.deps import ScheduleRepoProtocol, get_schedule_repo
from src.web.schemas import (
    ButtonEventSchema,
    PedalPointSchema,
    ScheduleCreateRequest,
    ScheduleDetailResponse,
    ScheduleResponse,
    ScheduleUpdateRequest,
)

router = APIRouter(prefix="/api/v1/schedules", tags=["schedules"])

ScheduleRepo = Annotated[ScheduleRepoProtocol, Depends(get_schedule_repo)]


def _total_duration(pedal_points: list[PedalPoint], button_events: list[ButtonEvent]) -> float:
    """タイムラインの総時間を末尾のペダル点・ボタンイベント（押下終了）から算出する。"""
    pedal_end = pedal_points[-1].time_s if pedal_points else 0.0
    button_end = max((e.time_s + e.press_duration_s for e in button_events), default=0.0)
    return max(pedal_end, button_end)


def _to_response(s: TimeSchedule) -> ScheduleResponse:
    return ScheduleResponse(
        id=s.id,
        name=s.name,
        description=s.description,
        total_duration=s.total_duration,
        pedal_point_count=len(s.pedal_points),
        button_event_count=len(s.button_events),
        loop=s.loop,
        created_at=s.created_at,
    )


def _to_detail_response(s: TimeSchedule) -> ScheduleDetailResponse:
    return ScheduleDetailResponse(
        id=s.id,
        name=s.name,
        description=s.description,
        pedal_points=[
            PedalPointSchema(
                time_s=p.time_s,
                accel_opening=p.accel_opening,
                brake_opening=p.brake_opening,
            )
            for p in s.pedal_points
        ],
        button_events=[
            ButtonEventSchema(
                time_s=e.time_s,
                channel=e.channel,
                press_duration_s=e.press_duration_s,
            )
            for e in s.button_events
        ],
        total_duration=s.total_duration,
        loop=s.loop,
        created_at=s.created_at,
    )


def _pedal_points_from_schema(points: list[PedalPointSchema]) -> list[PedalPoint]:
    return [
        PedalPoint(time_s=p.time_s, accel_opening=p.accel_opening, brake_opening=p.brake_opening)
        for p in points
    ]


def _button_events_from_schema(events: list[ButtonEventSchema]) -> list[ButtonEvent]:
    return [
        ButtonEvent(time_s=e.time_s, channel=e.channel, press_duration_s=e.press_duration_s)
        for e in events
    ]


@router.get("/", response_model=list[ScheduleResponse])
async def list_schedules(repo: ScheduleRepo) -> list[ScheduleResponse]:
    schedules = await repo.list_all()
    return [_to_response(s) for s in schedules]


@router.post("/", response_model=ScheduleDetailResponse, status_code=201)
async def create_schedule(req: ScheduleCreateRequest, repo: ScheduleRepo) -> ScheduleDetailResponse:
    """タイムスケジュールを新規作成する。名称重複は 409。"""
    pedal_points = _pedal_points_from_schema(req.pedal_points)
    button_events = _button_events_from_schema(req.button_events)
    schedule = TimeSchedule(
        id=str(uuid4()),
        name=req.name.strip(),
        description=req.description,
        pedal_points=pedal_points,
        button_events=button_events,
        total_duration=_total_duration(pedal_points, button_events),
        loop=req.loop,
        created_at=datetime.now(tz=UTC),
    )
    try:
        created = await repo.create(schedule)
    except DuplicateNameError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    return _to_detail_response(created)


@router.get("/{schedule_id}", response_model=ScheduleDetailResponse)
async def get_schedule(schedule_id: str, repo: ScheduleRepo) -> ScheduleDetailResponse:
    try:
        schedule = await repo.get_by_id(schedule_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail="schedule_id が UUID 形式ではありません") from e
    if schedule is None:
        raise HTTPException(
            status_code=404, detail=f"スケジュール {schedule_id!r} が見つかりません"
        )
    return _to_detail_response(schedule)


@router.put("/{schedule_id}", response_model=ScheduleDetailResponse)
async def update_schedule(
    schedule_id: str, req: ScheduleUpdateRequest, repo: ScheduleRepo
) -> ScheduleDetailResponse:
    """タイムスケジュールを更新する。省略フィールドは既存値を維持する。名称重複は 409。"""
    try:
        existing = await repo.get_by_id(schedule_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail="schedule_id が UUID 形式ではありません") from e
    if existing is None:
        raise HTTPException(
            status_code=404, detail=f"スケジュール {schedule_id!r} が見つかりません"
        )
    pedal_points = (
        _pedal_points_from_schema(req.pedal_points)
        if req.pedal_points is not None
        else existing.pedal_points
    )
    button_events = (
        _button_events_from_schema(req.button_events)
        if req.button_events is not None
        else existing.button_events
    )
    merged = TimeSchedule(
        id=existing.id,
        name=req.name.strip() if req.name is not None else existing.name,
        description=req.description if req.description is not None else existing.description,
        pedal_points=pedal_points,
        button_events=button_events,
        total_duration=_total_duration(pedal_points, button_events),
        loop=req.loop if req.loop is not None else existing.loop,
        created_at=existing.created_at,
    )
    try:
        updated = await repo.update(merged)
    except DuplicateNameError as e:
        raise HTTPException(status_code=409, detail=str(e)) from e
    if updated is None:
        raise HTTPException(
            status_code=404, detail=f"スケジュール {schedule_id!r} が見つかりません"
        )
    return _to_detail_response(updated)


@router.delete("/{schedule_id}", status_code=204)
async def delete_schedule(schedule_id: str, repo: ScheduleRepo) -> None:
    try:
        deleted = await repo.delete(schedule_id)
    except ValueError as e:
        raise HTTPException(status_code=400, detail="schedule_id が UUID 形式ではありません") from e
    if not deleted:
        raise HTTPException(
            status_code=404, detail=f"スケジュール {schedule_id!r} が見つかりません"
        )
