"""シーケンス API（/api/v1/sequences）のユニットテスト。PostgreSQL 不要（in-memory リポジトリ）。"""

from datetime import UTC, datetime

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.stubs import (
    InMemoryModeRepository,
    InMemoryScheduleRepository,
    InMemorySequenceRepository,
)
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.time_schedule import TimeSchedule
from src.web.app import app

MODE_ID = "00000000-0000-0000-0000-0000000000a1"
SCHED_ID = "00000000-0000-0000-0000-0000000000b1"


@pytest.fixture
async def client():
    app.state.mode_repo = InMemoryModeRepository()
    app.state.schedule_repo = InMemoryScheduleRepository()
    app.state.sequence_repo = InMemorySequenceRepository()
    now = datetime.now(tz=UTC)
    await app.state.mode_repo.create(
        DrivingMode(
            id=MODE_ID,
            name="モードA",
            description="",
            reference_speed=[SpeedPoint(time_s=0.0, speed_kmh=0.0)],
            total_duration=100.0,
            max_speed=0.0,
            created_at=now,
        )
    )
    await app.state.schedule_repo.create(
        TimeSchedule(
            id=SCHED_ID,
            name="スケジュールB",
            description="",
            pedal_points=[],
            button_events=[],
            total_duration=30.0,
            created_at=now,
        )
    )
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        yield c


def _body(name: str = "seq1", **over):
    body = {
        "name": name,
        "description": "説明",
        "steps": [
            {"kind": "mode", "ref_id": MODE_ID, "repeat": 2},
            {"kind": "schedule", "ref_id": SCHED_ID, "repeat": 1},
        ],
        "repeat": 3,
    }
    body.update(over)
    return body


@pytest.mark.asyncio
async def test_crud_and_total_duration(client: AsyncClient) -> None:
    res = await client.post("/api/v1/sequences/", json=_body())
    assert res.status_code == 201
    data = res.json()
    # (100*2 + 30*1) * 3
    assert data["total_duration"] == pytest.approx(690.0)
    assert data["has_missing"] is False
    assert data["steps"][0]["name"] == "モードA"
    assert data["steps"][0]["duration_s"] == 100.0
    assert data["steps"][1]["missing"] is False
    sid = data["id"]

    res = await client.get("/api/v1/sequences/")
    assert [s["id"] for s in res.json()] == [sid]

    res = await client.get(f"/api/v1/sequences/{sid}")
    assert res.status_code == 200 and res.json()["name"] == "seq1"

    res = await client.put(
        f"/api/v1/sequences/{sid}",
        json=_body("seq2", steps=[{"kind": "mode", "ref_id": MODE_ID}], repeat=1),
    )
    assert res.status_code == 200
    assert res.json()["name"] == "seq2"
    assert res.json()["total_duration"] == pytest.approx(100.0)

    assert (await client.delete(f"/api/v1/sequences/{sid}")).status_code == 204
    assert (await client.get(f"/api/v1/sequences/{sid}")).status_code == 404
    assert (await client.delete(f"/api/v1/sequences/{sid}")).status_code == 404
    assert (await client.put(f"/api/v1/sequences/{sid}", json=_body())).status_code == 404


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "over",
    [
        {"steps": []},
        {"repeat": 0},
        {"name": ""},
        {"name": "   "},
        {"steps": [{"kind": "mode", "ref_id": MODE_ID, "repeat": 0}]},
        {"steps": [{"kind": "other", "ref_id": MODE_ID}]},
        {"steps": [{"kind": "mode", "ref_id": "00000000-0000-0000-0000-00000000ffff"}]},
        {"steps": [{"kind": "schedule", "ref_id": MODE_ID}]},  # 種類違いの ID
        {"steps": [{"kind": "mode", "ref_id": "not-a-uuid"}]},
    ],
)
async def test_create_422(client: AsyncClient, over: dict) -> None:
    res = await client.post("/api/v1/sequences/", json=_body(**over))
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_missing_ref_detail_is_japanese(client: AsyncClient) -> None:
    res = await client.post(
        "/api/v1/sequences/",
        json=_body(steps=[{"kind": "mode", "ref_id": "00000000-0000-0000-0000-00000000ffff"}]),
    )
    assert res.status_code == 422
    assert "見つかりません" in res.json()["detail"]


@pytest.mark.asyncio
async def test_update_422_missing_ref(client: AsyncClient) -> None:
    sid = (await client.post("/api/v1/sequences/", json=_body())).json()["id"]
    res = await client.put(
        f"/api/v1/sequences/{sid}",
        json=_body(steps=[{"kind": "schedule", "ref_id": "00000000-0000-0000-0000-00000000ffff"}]),
    )
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_duplicate_name_409(client: AsyncClient) -> None:
    assert (await client.post("/api/v1/sequences/", json=_body("dup"))).status_code == 201
    assert (await client.post("/api/v1/sequences/", json=_body("dup"))).status_code == 409
    other = (await client.post("/api/v1/sequences/", json=_body("other"))).json()["id"]
    res = await client.put(f"/api/v1/sequences/{other}", json=_body("dup"))
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_list_shows_missing_after_ref_deleted(client: AsyncClient) -> None:
    sid = (await client.post("/api/v1/sequences/", json=_body())).json()["id"]
    await app.state.schedule_repo.delete(SCHED_ID)
    res = await client.get("/api/v1/sequences/")
    seq = res.json()[0]
    assert seq["has_missing"] is True
    step = seq["steps"][1]
    assert step["missing"] is True and step["duration_s"] == 0.0
    # 見つからない段は 0 秒: (100*2) * 3
    assert seq["total_duration"] == pytest.approx(600.0)
    detail = (await client.get(f"/api/v1/sequences/{sid}")).json()
    assert detail["has_missing"] is True
