import csv
import io
from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException
from fastapi.responses import Response

from src.models.drive_log import DriveSession, LearningCycle
from src.utils.time import to_jst_naive
from src.web.deps import SessionRepoProtocol, get_session_repo
from src.web.schemas import CycleSummaryResponse, LogResponse, SessionResponse

router = APIRouter(prefix="/api/v1/sessions", tags=["sessions"])

SessionRepo = Annotated[SessionRepoProtocol, Depends(get_session_repo)]

# ArchiveManager._export_to_csv と同一の列順（アーカイブ CSV と互換）。
_CSV_FIELDNAMES = [
    "timestamp",
    "ref_speed_kmh",
    "actual_speed_kmh",
    "accel_opening",
    "brake_opening",
    "accel_pos",
    "brake_pos",
    "accel_current",
    "brake_current",
]


def _to_response(session: DriveSession) -> SessionResponse:
    return SessionResponse(
        id=session.id,
        profile_id=session.profile_id,
        mode_id=session.mode_id,
        run_type=session.run_type,
        started_at=session.started_at,
        ended_at=session.ended_at,
        status=session.status,
        cycle_id=session.cycle_id,
    )


def _to_cycle_response(cycle: LearningCycle) -> CycleSummaryResponse:
    return CycleSummaryResponse(
        id=cycle.id,
        profile_id=cycle.profile_id,
        status=cycle.status,
        started_at=cycle.started_at,
        ended_at=cycle.ended_at,
        session_count=cycle.session_count,
        detail=cycle.detail,
    )


@router.get("/", response_model=list[SessionResponse])
async def list_sessions(repo: SessionRepo) -> list[SessionResponse]:
    sessions = await repo.list_all()
    return [_to_response(s) for s in sessions]


@router.get("/cycles", response_model=list[CycleSummaryResponse])
async def list_cycles(
    repo: SessionRepo, profile_id: str | None = None
) -> list[CycleSummaryResponse]:
    """学習サイクル一覧を返す（ログ画面のサイクル単位グループ表示に使う）。"""
    cycles = await repo.list_cycles(profile_id=profile_id)
    return [_to_cycle_response(c) for c in cycles]


@router.get("/{session_id}", response_model=SessionResponse)
async def get_session(session_id: str, repo: SessionRepo) -> SessionResponse:
    session = await repo.get_by_id(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail=f"セッション {session_id!r} が見つかりません")
    return _to_response(session)


@router.get("/{session_id}/logs.csv")
async def download_session_logs_csv(session_id: str, repo: SessionRepo) -> Response:
    """セッションの走行ログを CSV ファイルとしてダウンロードさせる。

    列構成はアーカイブ CSV（ArchiveManager）と一致させる。長時間セッションでも
    欠落しないよう取得上限を大きく取る。
    """
    session = await repo.get_by_id(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail=f"セッション {session_id!r} が見つかりません")

    logs = await repo.list_logs(session_id, limit=1_000_000)

    buf = io.StringIO()
    writer = csv.writer(buf)
    writer.writerow(_CSV_FIELDNAMES)
    for log in logs:
        writer.writerow(
            [
                to_jst_naive(log.timestamp).isoformat(sep=" "),
                log.ref_speed_kmh,
                log.actual_speed_kmh,
                log.accel_opening,
                log.brake_opening,
                log.accel_pos,
                log.brake_pos,
                log.accel_current,
                log.brake_current,
            ]
        )

    filename = f"session_{session_id}_{to_jst_naive(session.started_at):%Y%m%d_%H%M%S}.csv"
    return Response(
        content=buf.getvalue(),
        media_type="text/csv",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )


@router.get("/{session_id}/logs", response_model=list[LogResponse])
async def get_session_logs(session_id: str, repo: SessionRepo) -> list[LogResponse]:
    # 既定の limit ではセッション開始～終了の全体が取れず、グラフ・テーブルが途中で
    # 途切れてしまう。CSV ダウンロードと同じく取得上限を大きく取り、全区間を返す。
    logs = await repo.list_logs(session_id, limit=1_000_000)
    return [
        LogResponse(
            id=log.id,
            session_id=log.session_id,
            timestamp=log.timestamp,
            ref_speed_kmh=log.ref_speed_kmh,
            actual_speed_kmh=log.actual_speed_kmh,
            accel_opening=log.accel_opening,
            brake_opening=log.brake_opening,
            accel_pos=log.accel_pos,
            brake_pos=log.brake_pos,
            accel_current=log.accel_current,
            brake_current=log.brake_current,
        )
        for log in logs
    ]
