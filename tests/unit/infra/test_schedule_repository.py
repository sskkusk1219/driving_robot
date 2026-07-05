"""InMemoryScheduleRepository と ScheduleRepository の行変換のユニットテスト。"""

from __future__ import annotations

import json
from datetime import UTC, datetime

import pytest

from src.app.stubs import InMemoryScheduleRepository
from src.infra.db import DuplicateNameError
from src.infra.schedule_repository import (
    _button_events_to_json,
    _pedal_points_to_json,
    _row_to_schedule,
)
from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule


def _make_schedule(name: str = "sched", sid: str = "s1") -> TimeSchedule:
    return TimeSchedule(
        id=sid,
        name=name,
        description="desc",
        pedal_points=[PedalPoint(0.0, 0.0, 0.0), PedalPoint(5.0, 40.0, 0.0)],
        button_events=[ButtonEvent(0.5, 0, 1.0)],
        total_duration=5.0,
        created_at=datetime.now(tz=UTC),
    )


# ── in-memory リポジトリ ───────────────────────────────────────────────


async def test_create_and_get() -> None:
    repo = InMemoryScheduleRepository()
    created = await repo.create(_make_schedule())
    fetched = await repo.get_by_id(created.id)
    assert fetched is not None
    assert fetched.name == "sched"
    assert len(fetched.pedal_points) == 2


async def test_duplicate_name_raises() -> None:
    repo = InMemoryScheduleRepository()
    await repo.create(_make_schedule(name="dup", sid="a"))
    with pytest.raises(DuplicateNameError):
        await repo.create(_make_schedule(name="dup", sid="b"))


async def test_update_missing_returns_none() -> None:
    repo = InMemoryScheduleRepository()
    assert await repo.update(_make_schedule(sid="ghost")) is None


async def test_update_rejects_name_collision() -> None:
    repo = InMemoryScheduleRepository()
    await repo.create(_make_schedule(name="first", sid="a"))
    await repo.create(_make_schedule(name="second", sid="b"))
    collide = _make_schedule(name="first", sid="b")
    with pytest.raises(DuplicateNameError):
        await repo.update(collide)


async def test_delete() -> None:
    repo = InMemoryScheduleRepository()
    created = await repo.create(_make_schedule())
    assert await repo.delete(created.id) is True
    assert await repo.delete(created.id) is False


async def test_list_all_newest_first() -> None:
    repo = InMemoryScheduleRepository()
    old = _make_schedule(name="old", sid="a")
    old.created_at = datetime(2020, 1, 1, tzinfo=UTC)
    new = _make_schedule(name="new", sid="b")
    new.created_at = datetime(2026, 1, 1, tzinfo=UTC)
    await repo.create(old)
    await repo.create(new)
    listed = await repo.list_all()
    assert [s.name for s in listed] == ["new", "old"]


# ── 行変換（JSONB ⇄ dataclass）─────────────────────────────────────────


def test_row_to_schedule_roundtrip() -> None:
    s = _make_schedule()
    row = {
        "id": s.id,
        "name": s.name,
        "description": s.description,
        "pedal_points": _pedal_points_to_json(s.pedal_points),
        "button_events": _button_events_to_json(s.button_events),
        "total_duration": s.total_duration,
        "created_at": s.created_at,
    }
    restored = _row_to_schedule(row)
    assert restored.name == s.name
    assert restored.pedal_points[1].accel_opening == 40.0
    assert restored.button_events[0].press_duration_s == 1.0
    # JSON 化された内容が想定どおり
    assert json.loads(row["pedal_points"])[0]["time_s"] == 0.0
