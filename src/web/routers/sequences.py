"""シーケンス（走行モードとタイムスケジュールを順に並べたもの）CRUD ルーター。

スケジュール管理（schedules.py）と同じ CRUD パターンに従う。各段の参照先（走行モード／
タイムスケジュール）を解決して名前・長さを付けて返す。参照先が削除されていれば
`missing=True`・長さ 0 とし、見つからない段があるままの作成・更新は 422 で拒否する。
実行は対象外（作成・編集・一覧・削除のみ）。
"""

from datetime import UTC, datetime
from typing import Annotated
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException

from src.models.sequence import Sequence, SequenceStep
from src.web.deps import (
    ModeRepoProtocol,
    ScheduleRepoProtocol,
    SequenceRepoProtocol,
    get_mode_repo,
    get_schedule_repo,
    get_sequence_repo,
)
from src.web.schemas import (
    SequenceCreateRequest,
    SequenceResponse,
    SequenceStepResponse,
    SequenceStepSchema,
    SequenceUpdateRequest,
)

router = APIRouter(prefix="/api/v1/sequences", tags=["sequences"])

SequenceRepo = Annotated[SequenceRepoProtocol, Depends(get_sequence_repo)]
ModeRepo = Annotated[ModeRepoProtocol, Depends(get_mode_repo)]
ScheduleRepo = Annotated[ScheduleRepoProtocol, Depends(get_schedule_repo)]


async def _resolve_step(
    step: SequenceStep, mode_repo: ModeRepoProtocol, schedule_repo: ScheduleRepoProtocol
) -> SequenceStepResponse:
    """段の参照先を解決する。見つからない（削除済み・ID 不正）場合は missing=True・長さ 0。"""
    name = ""
    duration = 0.0
    missing = True
    try:
        if step.kind == "mode":
            mode = await mode_repo.get_by_id(step.ref_id)
            if mode is not None:
                name, duration, missing = mode.name, mode.total_duration, False
        else:
            schedule = await schedule_repo.get_by_id(step.ref_id)
            if schedule is not None:
                name, duration, missing = schedule.name, schedule.total_duration, False
    except ValueError:
        # UUID 形式でない ID は「見つからない」として扱う
        pass
    return SequenceStepResponse(
        kind=step.kind,
        ref_id=step.ref_id,
        repeat=step.repeat,
        name=name,
        duration_s=duration,
        missing=missing,
    )


async def _to_response(
    seq: Sequence, mode_repo: ModeRepoProtocol, schedule_repo: ScheduleRepoProtocol
) -> SequenceResponse:
    steps = [await _resolve_step(s, mode_repo, schedule_repo) for s in seq.steps]
    one_pass = sum(s.duration_s * s.repeat for s in steps)
    return SequenceResponse(
        id=seq.id,
        name=seq.name,
        description=seq.description,
        steps=steps,
        repeat=seq.repeat,
        total_duration=one_pass * seq.repeat,
        has_missing=any(s.missing for s in steps),
        created_at=seq.created_at,
    )


def _steps_from_schema(steps: list[SequenceStepSchema]) -> list[SequenceStep]:
    return [SequenceStep(kind=s.kind, ref_id=s.ref_id, repeat=s.repeat) for s in steps]


async def _validate_request(
    name: str,
    steps: list[SequenceStep],
    mode_repo: ModeRepoProtocol,
    schedule_repo: ScheduleRepoProtocol,
) -> None:
    """名前の空白のみ・参照先が見つからない段を 422 で拒否する。"""
    if not name.strip():
        raise HTTPException(status_code=422, detail="シーケンス名を入力してください")
    for i, step in enumerate(steps, start=1):
        resolved = await _resolve_step(step, mode_repo, schedule_repo)
        if resolved.missing:
            label = "走行モード" if step.kind == "mode" else "タイムスケジュール"
            raise HTTPException(
                status_code=422,
                detail=f"{i} 段目の{label}（{step.ref_id}）が見つかりません",
            )


def _bad_id() -> HTTPException:
    return HTTPException(status_code=400, detail="sequence_id が UUID 形式ではありません")


def _not_found(sequence_id: str) -> HTTPException:
    return HTTPException(status_code=404, detail=f"シーケンス {sequence_id!r} が見つかりません")


@router.get("/", response_model=list[SequenceResponse])
async def list_sequences(
    repo: SequenceRepo, mode_repo: ModeRepo, schedule_repo: ScheduleRepo
) -> list[SequenceResponse]:
    sequences = await repo.list_all()
    return [await _to_response(s, mode_repo, schedule_repo) for s in sequences]


@router.post("/", response_model=SequenceResponse, status_code=201)
async def create_sequence(
    req: SequenceCreateRequest,
    repo: SequenceRepo,
    mode_repo: ModeRepo,
    schedule_repo: ScheduleRepo,
) -> SequenceResponse:
    """シーケンスを新規作成する。参照先が見つからなければ 422・名称重複は 409。"""
    steps = _steps_from_schema(req.steps)
    await _validate_request(req.name, steps, mode_repo, schedule_repo)
    seq = Sequence(
        id=str(uuid4()),
        name=req.name.strip(),
        description=req.description,
        steps=steps,
        repeat=req.repeat,
        created_at=datetime.now(tz=UTC),
    )
    created = await repo.create(seq)
    return await _to_response(created, mode_repo, schedule_repo)


@router.get("/{sequence_id}", response_model=SequenceResponse)
async def get_sequence(
    sequence_id: str, repo: SequenceRepo, mode_repo: ModeRepo, schedule_repo: ScheduleRepo
) -> SequenceResponse:
    try:
        seq = await repo.get_by_id(sequence_id)
    except ValueError as e:
        raise _bad_id() from e
    if seq is None:
        raise _not_found(sequence_id)
    return await _to_response(seq, mode_repo, schedule_repo)


@router.put("/{sequence_id}", response_model=SequenceResponse)
async def update_sequence(
    sequence_id: str,
    req: SequenceUpdateRequest,
    repo: SequenceRepo,
    mode_repo: ModeRepo,
    schedule_repo: ScheduleRepo,
) -> SequenceResponse:
    """シーケンスを更新する（全項目置き換え）。参照先が見つからなければ 422・名称重複は 409。"""
    try:
        existing = await repo.get_by_id(sequence_id)
    except ValueError as e:
        raise _bad_id() from e
    if existing is None:
        raise _not_found(sequence_id)
    steps = _steps_from_schema(req.steps)
    await _validate_request(req.name, steps, mode_repo, schedule_repo)
    merged = Sequence(
        id=existing.id,
        name=req.name.strip(),
        description=req.description,
        steps=steps,
        repeat=req.repeat,
        created_at=existing.created_at,
    )
    updated = await repo.update(merged)
    if updated is None:
        raise _not_found(sequence_id)
    return await _to_response(updated, mode_repo, schedule_repo)


@router.delete("/{sequence_id}", status_code=204)
async def delete_sequence(sequence_id: str, repo: SequenceRepo) -> None:
    try:
        deleted = await repo.delete(sequence_id)
    except ValueError as e:
        raise _bad_id() from e
    if not deleted:
        raise _not_found(sequence_id)
