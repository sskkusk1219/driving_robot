import csv
import io
from datetime import UTC, datetime
from typing import Annotated
from uuid import uuid4

from fastapi import APIRouter, Depends, Form, HTTPException, UploadFile

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


import re as _re


def _normalize_col(name: str) -> str:
    """カラム名から記号・空白を除去して小文字化する。例: 'Time [s]' → 'times'"""
    return _re.sub(r"[^a-z0-9]", "", name.lower())


_TIME_NORMS = {"times", "timesec", "timesecs", "time", "t", "elapsed", "elapseds", "ts"}
_SPEED_NORMS = {"speedkmh", "speedkm", "speed", "vkmh", "v", "vel", "velocity", "kmh", "refspeed"}


def _detect_columns(fieldnames: list[str]) -> tuple[str, str]:
    """時刻列・速度列のカラム名を検出する。

    正規化マッチを優先し、見つからない場合は列位置（1列目=時刻、2列目=速度）で代替する。
    """
    pairs = [(_normalize_col(f), f) for f in fieldnames]

    time_col = next((orig for norm, orig in pairs if norm in _TIME_NORMS), None)
    speed_col = next((orig for norm, orig in pairs if norm in _SPEED_NORMS), None)

    if time_col is None and len(fieldnames) >= 1:
        time_col = fieldnames[0]
    if speed_col is None and len(fieldnames) >= 2:
        speed_col = fieldnames[1]

    if time_col is None or speed_col is None:
        raise ValueError("CSV に 2 列以上のデータが必要です")

    return time_col, speed_col


def _parse_csv(content: bytes) -> list[SpeedPoint]:
    """CSV をパースして SpeedPoint リストを返す。

    カラム名は柔軟に検出する（time_s/speed_kmh 以外も可）。
    時刻の単調増加と速度の非負を検証する。
    """
    text = content.decode("utf-8-sig")
    reader = csv.DictReader(io.StringIO(text))
    if not reader.fieldnames:
        raise ValueError("CSV にヘッダー行がありません")
    time_col, speed_col = _detect_columns(list(reader.fieldnames))
    points: list[SpeedPoint] = []
    prev_time = -1.0
    for i, row in enumerate(reader):
        try:
            t = float(row[time_col])
            s = float(row[speed_col])
        except (KeyError, ValueError) as e:
            raise ValueError(f"行 {i + 2}: 数値変換エラー ({e})") from e
        if t <= prev_time:
            raise ValueError(f"行 {i + 2}: 時刻列が単調増加していません ({t} <= {prev_time})")
        if s < 0:
            raise ValueError(f"行 {i + 2}: 速度列に負の値があります ({s})")
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
    name: Annotated[str, Form()] = "",
    description: Annotated[str, Form()] = "",
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
