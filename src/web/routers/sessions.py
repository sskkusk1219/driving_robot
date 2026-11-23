# ruff: noqa: E501
import csv
import io
import uuid
from collections.abc import AsyncIterator
from typing import Annotated, Any

from fastapi import APIRouter, Depends, HTTPException, Request
from fastapi.responses import Response, StreamingResponse

from src.infra import drive_csv
from src.infra.button_events import LEVEL_BUTTON, parse_button_presses
from src.infra.vehicle_store import VehicleStore
from src.models.drive_log import DriveSession, LearningCycle
from src.robot import config_store
from src.utils.time import to_jst_naive
from src.web.analysis import build_analysis, build_figure
from src.web.deps import ErrorRepoProtocol, SessionRepoProtocol, get_session_repo, get_vehicle_store
from src.web.schemas import CycleSummaryResponse, LogResponse, MessageResponse, SessionResponse

router = APIRouter(prefix="/api/v1/sessions", tags=["sessions"])

SessionRepo = Annotated[SessionRepoProtocol, Depends(get_session_repo)]
Store = Annotated[VehicleStore, Depends(get_vehicle_store)]


def _optional_error_repo(request: Request) -> ErrorRepoProtocol | None:
    """エラーの保存先（無い環境では None。CSV はエラー列を空にして出す）。"""
    repo: ErrorRepoProtocol | None = getattr(request.app.state, "error_repo", None)
    return repo


ErrorRepo = Annotated[ErrorRepoProtocol | None, Depends(_optional_error_repo)]


def _require_uuid(value: str, label: str = "session_id") -> None:
    """UUID 形式でなければ 400（DB 層の UUID 変換で 500 にしない）。"""
    try:
        uuid.UUID(value)
    except ValueError:
        raise HTTPException(status_code=400, detail=f"{label} が UUID 形式ではありません") from None


def _vehicle_name(store: VehicleStore, vehicle_id: str | None, cache: dict[str, str | None]) -> str | None:
    """車の名前（今の vehicle.yaml の name）。車が消されていれば None（画面は ID を出す）。"""
    if vehicle_id is None:
        return None
    if vehicle_id not in cache:
        try:
            cache[vehicle_id] = store.read(vehicle_id).name
        except Exception:  # noqa: BLE001 - 消えた・壊れた車は名前なし
            cache[vehicle_id] = None
    return cache[vehicle_id]


def _to_response(
    session: DriveSession, store: VehicleStore, cache: dict[str, str | None] | None = None
) -> SessionResponse:
    return SessionResponse(
        id=session.id,
        profile_id=session.profile_id,
        vehicle_id=session.vehicle_id,
        vehicle_name=_vehicle_name(store, session.vehicle_id, {} if cache is None else cache),
        mode_id=session.mode_id,
        run_type=session.run_type,
        started_at=session.started_at,
        ended_at=session.ended_at,
        status=session.status,
        cycle_id=session.cycle_id,
        schedule_id=session.schedule_id,
        schedule_name=session.schedule_name,
        sequence_run_id=session.sequence_run_id,
        sequence_name=session.sequence_name,
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
async def list_sessions(repo: SessionRepo, store: Store) -> list[SessionResponse]:
    sessions = await repo.list_all()
    cache: dict[str, str | None] = {}
    return [_to_response(s, store, cache) for s in sessions]


@router.get("/cycles", response_model=list[CycleSummaryResponse])
async def list_cycles(
    repo: SessionRepo, profile_id: str | None = None
) -> list[CycleSummaryResponse]:
    """学習サイクル一覧を返す（ログ画面のサイクル単位グループ表示に使う）。"""
    if profile_id is not None:
        _require_uuid(profile_id, "profile_id")
    cycles = await repo.list_cycles(profile_id=profile_id)
    return [_to_cycle_response(c) for c in cycles]


@router.get("/{session_id}", response_model=SessionResponse)
async def get_session(session_id: str, repo: SessionRepo, store: Store) -> SessionResponse:
    _require_uuid(session_id)
    session = await repo.get_by_id(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail=f"セッション {session_id!r} が見つかりません")
    return _to_response(session, store)


@router.get("/{session_id}/messages", response_model=list[MessageResponse])
async def get_session_messages(session_id: str, repo: SessionRepo) -> list[MessageResponse]:
    """走行の進行のメッセージ（ランナーの標準出力の行）を古い順に返す。"""
    _require_uuid(session_id)
    session = await repo.get_by_id(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail=f"セッション {session_id!r} が見つかりません")
    messages = await repo.list_messages(session_id)
    # level BUTTON はボタン押下の機械可読な記録（走行グラフ用）。同じ押下の通常の文が別にあるので出さない
    return [
        MessageResponse(seq=m.seq, elapsed_s=m.elapsed_s, time=m.time, level=m.level, message=m.message)
        for m in messages
        if m.level != LEVEL_BUTTON
    ]


@router.get("/{session_id}/logs.csv")
async def download_session_logs_csv(
    session_id: str, repo: SessionRepo, error_repo: ErrorRepo
) -> Response:
    """セッションの走行ログを CSV でダウンロードさせる。

    列は研究ハーネス（tests/research）の CSV と同じ（core の drive_log.CSV_COLUMNS の順）。
    drive_logs の 9 列＋detail JSONB を展開して全列を書く。旧セッション（detail なし）の列は空欄。
    末尾にボタン（btn_start・btn_P・btn_N・btn_D）とエラー（error_level・error_title）の列を足し、
    先頭に BOM を付ける（Excel の文字化け対策）。ログの範囲の外のボタン・エラーは時刻順に行を足す。
    数万行でもメモリに全部載せないよう、DB のカーソルから流しながら返す（ボタン・エラーは数件なので先に読む）。
    """
    _require_uuid(session_id)
    session = await repo.get_by_id(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail=f"セッション {session_id!r} が見つかりません")

    header = drive_csv.csv_header(await repo.log_detail_keys(session_id))
    presses = parse_button_presses(await repo.list_messages(session_id))
    errors = await error_repo.list_by_session(session_id) if error_repo is not None else []
    overlay = drive_csv.CsvOverlay(presses, errors)

    def encode(rows: list[list[str]]) -> str:
        buf = io.StringIO()
        csv.writer(buf, lineterminator="\n").writerows(rows)
        return buf.getvalue()

    async def body() -> AsyncIterator[str]:
        yield "\ufeff" + encode([header])
        first = True
        last_ts = None
        async for batch in repo.iter_logs_for_csv(session_id):
            rows: list[list[str]] = []
            for r in batch:
                if first:
                    first = False
                    rows.extend(drive_csv.extra_row(header, t, c) for t, c in overlay.rows_before(r.timestamp))
                last_ts = r.timestamp
                cells = drive_csv.db_to_csv_cells(
                    timestamp=r.timestamp,
                    ref_speed_kmh=r.ref_speed_kmh,
                    actual_speed_kmh=r.actual_speed_kmh,
                    accel_opening=r.accel_opening,
                    brake_opening=r.brake_opening,
                    accel_pos=r.accel_pos,
                    brake_pos=r.brake_pos,
                    accel_current=r.accel_current,
                    brake_current=r.brake_current,
                    detail=r.detail,
                    columns=header,
                )
                rows.append(drive_csv.with_extra(cells, overlay.cells_for(r.timestamp)))
            yield encode(rows)
        tail = overlay.rows_only() if last_ts is None else overlay.rows_after(last_ts)
        if tail:
            yield encode([drive_csv.extra_row(header, t, c) for t, c in tail])

    filename = f"session_{session_id}_{to_jst_naive(session.started_at):%Y%m%d_%H%M%S}.csv"
    return StreamingResponse(
        body(),
        media_type="text/csv",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )


@router.get("/{session_id}/logs", response_model=list[LogResponse])
async def get_session_logs(session_id: str, repo: SessionRepo) -> list[LogResponse]:
    _require_uuid(session_id)
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


@router.get("/{session_id}/analysis")
async def get_session_analysis(session_id: str, repo: SessionRepo, store: Store) -> dict[str, Any]:
    """セッションの走行グラフ（``figure``）と、モード走行なら運転指標・偏差の分析（段11）。

    ``figure`` は車速・ペダル開度・ボタン押下の区間（壁時計を x に直したもの）。x はモード走行ならモード時間、
    学習などは経過時間。運転指標（EER・IWR・ASCR ほか）と偏差の KPI は auto のモード走行だけ
    （無ければ ``{"available": false}``）。質量・走行抵抗は車の vehicle.yaml（vehicle.*）から読む。
    合否・基準の線は出さない。
    """
    _require_uuid(session_id)
    session = await repo.get_by_id(session_id)
    if session is None:
        raise HTTPException(status_code=404, detail=f"セッション {session_id!r} が見つかりません")
    rows: list[Any] = []
    async for batch in repo.iter_logs_for_csv(session_id):
        rows.extend(batch)
    presses = parse_button_presses(await repo.list_messages(session_id))
    figure = build_figure(rows, is_auto=session.run_type == "auto", presses=presses)
    if session.run_type != "auto":
        return {"available": False, "figure": figure}
    vehicle: dict[str, Any] = {}
    if session.vehicle_id and store.exists(session.vehicle_id):
        try:
            vehicle = store.read(session.vehicle_id).vehicle
        except Exception:  # noqa: BLE001 - 壊れた車でも指標（ER・EER 以外）は出す
            vehicle = {}
    try:
        common = config_store._load_yaml(store.common_path)  # noqa: SLF001
        segment_names = list((common.get("modes") or {}).get("segment_names") or [])
    except Exception:  # noqa: BLE001
        segment_names = []
    analysis = build_analysis(
        rows,
        mass_kg=vehicle.get("test_mass_kg"),
        a0=vehicle.get("road_load_a0_n"),
        a1=vehicle.get("road_load_a1_n_per_kmh"),
        a2=vehicle.get("road_load_a2_n_per_kmh2"),
        segment_names=segment_names,
    )
    return {**analysis, "figure": figure}
