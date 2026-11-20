# ruff: noqa: E501
"""エラーページ（できごとの記録）API（/api/v1/errors）。RPD_Phase_20261120_ErrorLog。

一覧（水準・きっかけ・未確認のみで絞る。新しい順・ページ送り）・1 件の詳細・確認済みにする／戻す・
すべて確認済みにする・1 件の削除・確認済みをまとめて削除、ブラウザからの記録（POST /client）。
"""

from __future__ import annotations

from datetime import UTC, datetime
from typing import Annotated, Any, Literal

from fastapi import APIRouter, Depends, HTTPException, Query, Request
from pydantic import BaseModel, Field

from src.models.error_event import (
    LEVEL_ERROR,
    SOURCE_BROWSER,
    SOURCES,
    ErrorEvent,
    NewErrorEvent,
)
from src.web.deps import ErrorRepoProtocol, get_error_repo

router = APIRouter(prefix="/api/v1/errors", tags=["errors"])

ErrorRepo = Annotated[ErrorRepoProtocol, Depends(get_error_repo)]

TITLE_MAX = 120


class ErrorSummary(BaseModel):
    id: int
    first_at: datetime
    last_at: datetime
    count: int
    level: str
    source: str
    title: str
    job_kind: str | None = None
    vehicle_id: str | None = None
    session_id: str | None = None
    sequence_name: str | None = None
    acknowledged_at: datetime | None = None


class ErrorDetail(ErrorSummary):
    detail: str
    job_id: str | None = None
    context: dict[str, Any]


class ErrorListResponse(BaseModel):
    items: list[ErrorSummary]
    total: int
    limit: int
    offset: int


class AcknowledgeResult(BaseModel):
    count: int


class ClientErrorRequest(BaseModel):
    """ブラウザ側のエラー（赤いトースト・window.onerror・unhandledrejection）。"""

    message: str = Field(max_length=4000)
    kind: Literal["toast", "onerror", "unhandledrejection"] = "toast"
    stack: str | None = Field(default=None, max_length=8000)
    url: str | None = Field(default=None, max_length=500)


def _summary(e: ErrorEvent) -> ErrorSummary:
    return ErrorSummary(
        id=e.id, first_at=e.first_at, last_at=e.last_at, count=e.count, level=e.level, source=e.source,
        title=e.title, job_kind=e.job_kind, vehicle_id=e.vehicle_id, session_id=e.session_id,
        sequence_name=e.sequence_name, acknowledged_at=e.acknowledged_at,
    )


def _detail(e: ErrorEvent) -> ErrorDetail:
    return ErrorDetail(**_summary(e).model_dump(), detail=e.detail, job_id=e.job_id, context=e.context)


@router.get("", response_model=ErrorListResponse)
async def list_errors(
    repo: ErrorRepo,
    level: Annotated[Literal["ERROR", "WARNING"] | None, Query()] = None,
    source: Annotated[str | None, Query()] = None,
    unacknowledged: Annotated[bool, Query()] = False,
    limit: Annotated[int, Query(ge=1, le=200)] = 50,
    offset: Annotated[int, Query(ge=0)] = 0,
) -> ErrorListResponse:
    """新しい順（最後に起きた日時）。``unacknowledged=true`` で未確認のみ。"""
    if source is not None and source not in SOURCES:
        raise HTTPException(status_code=422, detail=f"source は {', '.join(SOURCES)} のどれかです")
    items, total = await repo.list_events(
        level=level, source=source, unacknowledged_only=unacknowledged, limit=limit, offset=offset
    )
    return ErrorListResponse(items=[_summary(e) for e in items], total=total, limit=limit, offset=offset)


@router.post("/acknowledge-all", response_model=AcknowledgeResult)
async def acknowledge_all(repo: ErrorRepo) -> AcknowledgeResult:
    """未確認のものをすべて確認済みにする。"""
    return AcknowledgeResult(count=await repo.acknowledge_all(at=datetime.now(tz=UTC)))


@router.delete("/acknowledged", response_model=AcknowledgeResult)
async def delete_acknowledged(repo: ErrorRepo) -> AcknowledgeResult:
    """確認済みのものをまとめて削除する。"""
    return AcknowledgeResult(count=await repo.delete_acknowledged())


@router.post("/client", status_code=204)
async def record_client_error(body: ClientErrorRequest, request: Request) -> None:
    """ブラウザ側のエラーを記録する。記録係へ渡すだけで待たない（60 秒以内の同じ文は回数を足す）。"""
    recorder = getattr(request.app.state, "error_recorder", None)
    if recorder is None:
        return
    message = body.message.strip()
    first = message.splitlines()[0] if message else "(空のエラー)"
    detail = message
    if body.stack:
        detail += f"\n{body.stack}"
    ctx: dict[str, Any] = {"kind": body.kind}
    if body.url:
        ctx["url"] = body.url
    recorder.record(
        NewErrorEvent(
            level=LEVEL_ERROR,
            source=SOURCE_BROWSER,
            title=first[:TITLE_MAX],
            detail=detail,
            context=ctx,
            dedupe=True,
        )
    )


@router.get("/{event_id}", response_model=ErrorDetail)
async def get_error(event_id: int, repo: ErrorRepo) -> ErrorDetail:
    e = await repo.get(event_id)
    if e is None:
        raise HTTPException(status_code=404, detail="エラーの記録が見つかりません")
    return _detail(e)


@router.post("/{event_id}/acknowledge", response_model=ErrorDetail)
async def acknowledge(event_id: int, repo: ErrorRepo) -> ErrorDetail:
    """確認済みにする。"""
    if not await repo.set_acknowledged(event_id, True, at=datetime.now(tz=UTC)):
        raise HTTPException(status_code=404, detail="エラーの記録が見つかりません")
    return _detail(await _must_get(repo, event_id))


@router.post("/{event_id}/unacknowledge", response_model=ErrorDetail)
async def unacknowledge(event_id: int, repo: ErrorRepo) -> ErrorDetail:
    """確認済みを戻す。"""
    if not await repo.set_acknowledged(event_id, False, at=datetime.now(tz=UTC)):
        raise HTTPException(status_code=404, detail="エラーの記録が見つかりません")
    return _detail(await _must_get(repo, event_id))


@router.delete("/{event_id}", status_code=204)
async def delete_error(event_id: int, repo: ErrorRepo) -> None:
    if not await repo.delete(event_id):
        raise HTTPException(status_code=404, detail="エラーの記録が見つかりません")


async def _must_get(repo: ErrorRepoProtocol, event_id: int) -> ErrorEvent:
    e = await repo.get(event_id)
    if e is None:
        raise HTTPException(status_code=404, detail="エラーの記録が見つかりません")
    return e

