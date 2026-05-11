import csv
import io
from datetime import UTC, datetime
from typing import Annotated
from uuid import uuid4

from fastapi import APIRouter, Depends, HTTPException, UploadFile

from src.models.driving_mode import DrivingMode, SpeedPoint
from src.web.deps import ModeRepoProtocol, get_mode_repo
from src.web.schemas import ModeDetailResponse, ModeResponse, SpeedPointSchema

router = APIRouter(prefix="/api/v1/modes", tags=["modes"])

ModeRepo = Annotated[ModeRepoProtocol, Depends(get_mode_repo)]


def _to_response(m: DrivingMode) -> ModeResponse:
    return ModeResponse(
        id=m.id,
        name=m.name,
        description=m.description,
        total_duration=m.total_duration,
        max_speed=m.max_speed,
        point_count=len(m.reference_speed),
        created_at=m.created_at,
    )


def _to_detail_response(m: DrivingMode) -> ModeDetailResponse:
    return ModeDetailResponse(
        id=m.id,
        name=m.name,
        description=m.description,
        reference_speed=[
            SpeedPointSchema(time_s=p.time_s, speed_kmh=p.speed_kmh) for p in m.reference_speed
        ],
        total_duration=m.total_duration,
        max_speed=m.max_speed,
        created_at=m.created_at,
    )


def _parse_csv(content: bytes) -> list[SpeedPoint]:
    """CSV (time_s,speed_kmh) をパースして SpeedPoint リストを返す。

    ヘッダー行が time_s, speed_kmh であることを検証する。
    時刻の単調増加と速度の非負を検証する。
    """
    text = content.decode("utf-8-sig")
    reader = csv.DictReader(io.StringIO(text))
    if reader.fieldnames is None or set(reader.fieldnames) < {"time_s", "speed_kmh"}:
        raise ValueError("CSV ヘッダーに time_s, speed_kmh が必要です")
    points: list[SpeedPoint] = []
    prev_time = -1.0
    for i, row in enumerate(reader):
        try:
            t = float(row["time_s"])
            s = float(row["speed_kmh"])
        except (KeyError, ValueError) as e:
            raise ValueError(f"行 {i + 2}: 数値変換エラー ({e})") from e
        if t <= prev_time:
            raise ValueError(f"行 {i + 2}: time_s が単調増加していません ({t} <= {prev_time})")
        if s < 0:
            raise ValueError(f"行 {i + 2}: speed_kmh が負の値です ({s})")
        points.append(SpeedPoint(time_s=t, speed_kmh=s))
        prev_time = t
    if not points:
        raise ValueError("CSV にデータ行がありません")
    return points


@router.get("/", response_model=list[ModeResponse])
async def list_modes(repo: ModeRepo) -> list[ModeResponse]:
    modes = await repo.list_all()
    return [_to_response(m) for m in modes]


@router.post("/upload", response_model=ModeResponse, status_code=201)
async def upload_mode(
    file: UploadFile,
    repo: ModeRepo,
    name: str = "",
    description: str = "",
) -> ModeResponse:
    """基準車速 CSV をアップロードして走行モードを作成する。

    CSV フォーマット: ヘッダー行 `time_s,speed_kmh` + データ行。
    name が空の場合はファイル名（拡張子除く）を使用する。
    """
    _MAX_CSV_BYTES = 10 * 1024 * 1024  # 10 MB
    if file.content_type not in ("text/csv", "text/plain", "application/octet-stream", None):
        raise HTTPException(status_code=415, detail="CSV ファイルをアップロードしてください")
    content = await file.read()
    if len(content) > _MAX_CSV_BYTES:
        raise HTTPException(status_code=413, detail="CSV ファイルが大きすぎます（上限 10MB）")
    try:
        points = _parse_csv(content)
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e)) from e

    mode_name = name.strip() or (
        file.filename.rsplit(".", 1)[0] if file.filename else "mode"
    )
    total_duration = points[-1].time_s
    max_speed = max(p.speed_kmh for p in points)

    mode = DrivingMode(
        id=str(uuid4()),
        name=mode_name,
        description=description,
        reference_speed=points,
        total_duration=total_duration,
        max_speed=max_speed,
        created_at=datetime.now(tz=UTC),
    )
    created = await repo.create(mode)
    return _to_response(created)


@router.get("/{mode_id}", response_model=ModeDetailResponse)
async def get_mode(mode_id: str, repo: ModeRepo) -> ModeDetailResponse:
    try:
        mode = await repo.get_by_id(mode_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="mode_id が UUID 形式ではありません")
    if mode is None:
        raise HTTPException(status_code=404, detail=f"走行モード {mode_id!r} が見つかりません")
    return _to_detail_response(mode)


@router.delete("/{mode_id}", status_code=204)
async def delete_mode(mode_id: str, repo: ModeRepo) -> None:
    try:
        deleted = await repo.delete(mode_id)
    except ValueError:
        raise HTTPException(status_code=400, detail="mode_id が UUID 形式ではありません")
    if not deleted:
        raise HTTPException(status_code=404, detail=f"走行モード {mode_id!r} が見つかりません")
