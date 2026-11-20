"""走行モード・スケジュールの名前が空・空白だけなら 422。

ProblemReport_20261120_src全動作チェック B-1。

PostgreSQL 不要（in-memory リポジトリ）。シーケンスの同じ検査は test_web_sequences.py。
"""

from datetime import UTC, datetime

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.inmemory_repos import InMemoryModeRepository, InMemoryScheduleRepository
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.time_schedule import TimeSchedule
from src.web.app import app

MODE_ID = "00000000-0000-0000-0000-0000000000c1"
SCHED_ID = "00000000-0000-0000-0000-0000000000d1"


@pytest.fixture
async def client():
    app.state.mode_repo = InMemoryModeRepository()
    app.state.schedule_repo = InMemoryScheduleRepository()
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


@pytest.mark.parametrize("name", ["", "  ", "　"])
async def test_mode_patch_blank_name_422(client: AsyncClient, name: str) -> None:
    res = await client.patch(f"/api/v1/modes/{MODE_ID}", json={"name": name})
    assert res.status_code == 422
    after = await client.get(f"/api/v1/modes/{MODE_ID}")
    assert after.json()["name"] == "モードA"


async def test_mode_patch_name_is_stripped(client: AsyncClient) -> None:
    res = await client.patch(f"/api/v1/modes/{MODE_ID}", json={"name": "  モードC  "})
    assert res.status_code == 200
    assert res.json()["name"] == "モードC"


async def test_mode_patch_description_only_keeps_name(client: AsyncClient) -> None:
    res = await client.patch(f"/api/v1/modes/{MODE_ID}", json={"description": "説明"})
    assert res.status_code == 200
    assert res.json()["name"] == "モードA"


@pytest.mark.parametrize("name", ["  ", "　"])
async def test_schedule_post_blank_name_422(client: AsyncClient, name: str) -> None:
    res = await client.post("/api/v1/schedules/", json={"name": name})
    assert res.status_code == 422
    assert res.json()["detail"] == "スケジュール名を入力してください"


async def test_schedule_put_blank_name_422(client: AsyncClient) -> None:
    res = await client.put(f"/api/v1/schedules/{SCHED_ID}", json={"name": "  "})
    assert res.status_code == 422
    assert res.json()["detail"] == "スケジュール名を入力してください"
