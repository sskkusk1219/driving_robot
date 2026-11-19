# ruff: noqa: E501
"""タイムスケジュール再生の純粋な部分（src/robot/schedule_spec.py）とモデル・保存・スキーマのボタン名。"""

from __future__ import annotations

import json
from datetime import UTC, datetime

import pytest
from pydantic import ValidationError

from src.infra.schedule_repository import _button_events_to_json
from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule
from src.robot import schedule_spec as sp
from src.web.schemas import ButtonEventSchema, ScheduleCreateRequest

LIM = {"min_accel": 4.0, "min_brake": 3.0, "max_accel": 60.0, "max_brake": 40.0}


def pts(*rows: tuple[float, float, float]) -> list[sp.PedalPoint]:
    return [sp.PedalPoint(t, a, b) for t, a, b in rows]


# ── 補間 ─────────────────────────────────────────────────────


def test_timeline_interpolates_and_holds_ends() -> None:
    tl = sp.PedalTimeline(pts((1.0, 0, 0), (3.0, 20, 0), (5.0, 20, 0), (7.0, 0, 10)))
    assert tl.at(0.0) == (0.0, 0.0)  # 先頭より前は先頭を保持
    assert tl.at(1.0) == (0.0, 0.0)
    assert tl.at(2.0) == (10.0, 0.0)  # 線形補間
    assert tl.at(4.0) == (20.0, 0.0)
    assert tl.at(6.0) == (10.0, 5.0)
    assert tl.at(7.0) == (0.0, 10.0)
    assert tl.at(99.0) == (0.0, 10.0)  # 終端より後は終端を保持


def test_timeline_empty() -> None:
    assert sp.PedalTimeline([]).at(3.0) == (0.0, 0.0)


# ── ペダルの決め方（頭打ち・待機位置・同時踏み） ────────────


def test_pedal_command_standby_and_clamp() -> None:
    assert sp.pedal_command(0.0, 0.0, **LIM) == (4.0, 3.0)  # 使わない側は待機位置（min）
    assert sp.pedal_command(2.0, 0.0, **LIM) == (4.0, 3.0)  # min 未満は min
    assert sp.pedal_command(30.0, 0.0, **LIM) == (30.0, 3.0)
    assert sp.pedal_command(99.0, 0.0, **LIM) == (60.0, 3.0)  # max で頭打ち
    assert sp.pedal_command(0.0, 99.0, **LIM) == (4.0, 40.0)
    assert sp.pedal_command(-5.0, -5.0, **LIM) == (4.0, 3.0)


def test_pedal_command_allows_simultaneous_press() -> None:
    # 同時踏みは許す（2026-11-19）: 両ペダルをそれぞれ頭打ち・min 持ち上げだけで通す
    assert sp.pedal_command(20.0, 5.0, **LIM) == (20.0, 5.0)
    assert sp.pedal_command(5.0, 20.0, **LIM) == (5.0, 20.0)
    assert sp.pedal_command(14.0, 13.0, **LIM) == (14.0, 13.0)
    assert sp.pedal_command(99.0, 99.0, **LIM) == (60.0, 40.0)
    assert sp.pedal_command(2.0, 50.0, **LIM) == (4.0, 40.0)


# ── ボタン ───────────────────────────────────────────────────


def sched(buttons: list[dict], points: list[dict] | None = None) -> sp.ScheduleData:
    return sp.schedule_from_dict({
        "id": "s", "name": "試験",
        "pedal_points": points if points is not None else [
            {"time_s": 0, "accel_opening": 0, "brake_opening": 0},
            {"time_s": 5, "accel_opening": 10, "brake_opening": 0},
        ],
        "button_events": buttons,
    })


def test_check_schedule_ok_and_button_rules() -> None:
    assert sp.check_schedule(sched([])) is None
    ok = [{"time_s": 1.0, "button": "D", "press_duration_s": None}]
    assert sp.check_schedule(sched(ok)) is None
    # 名前なし（旧データ: ch だけ）→ エラー（推測で押さない）
    legacy = [{"time_s": 1.0, "channel": 0, "press_duration_s": 1.0}, {"time_s": 2.0, "button": "N"}]
    msg = sp.check_schedule(sched(legacy))
    assert msg is not None and "ボタン名のないイベントが 1 件" in msg and "保存し直し" in msg
    # 知らない名前
    msg = sp.check_schedule(sched([{"time_s": 1.0, "button": "X"}]))
    assert msg is not None and "知らないボタン名" in msg
    # 点もイベントも無い・時刻が増えない
    assert sp.check_schedule(sched([], points=[])) is not None
    bad = [{"time_s": 1, "accel_opening": 0, "brake_opening": 0}, {"time_s": 1, "accel_opening": 5, "brake_opening": 0}]
    assert "増えていません" in (sp.check_schedule(sched([], points=bad)) or "")


def test_button_events_sorted_and_total_duration() -> None:
    s = sched([{"time_s": 8.0, "button": "D", "press_duration_s": 2.0}, {"time_s": 1.0, "button": "start"}])
    assert [b.time_s for b in s.buttons] == [1.0, 8.0]
    assert s.total_duration == 10.0  # 最後のボタンの押し終わり（8 + 2）が末尾のペダル点（5）より後


def test_button_seconds_default_from_vehicle() -> None:
    kw = {"start_short_press_s": 1.0, "shift_press_s": 1.2}
    assert sp.button_seconds(2.5, "D", **kw) == 2.5
    assert sp.button_seconds(None, "start", **kw) == 1.0
    assert sp.button_seconds(None, "N", **kw) == 1.2
    assert sp.button_seconds(0.0, "P", **kw) == 1.2


# ── モデル・保存・スキーマ ───────────────────────────────────


def test_repo_json_keeps_button_and_legacy_channel() -> None:
    events = [
        ButtonEvent(time_s=1.0, button="D", press_duration_s=1.2),
        ButtonEvent(0.5, 3, 1.0),  # 旧データ（positional）: ch だけ
        ButtonEvent(time_s=2.0, button="start"),  # 押す時間なし
    ]
    raw = json.loads(_button_events_to_json(events))
    assert raw[0] == {"time_s": 1.0, "button": "D", "press_duration_s": 1.2}
    assert raw[1] == {"time_s": 0.5, "channel": 3, "press_duration_s": 1.0}
    assert raw[2] == {"time_s": 2.0, "button": "start"}


def test_schedule_to_dict_roundtrip_through_spec() -> None:
    s = TimeSchedule(
        id="i", name="n", description="",
        pedal_points=[PedalPoint(0.0, 0.0, 0.0), PedalPoint(4.0, 10.0, 0.0)],
        button_events=[ButtonEvent(time_s=1.0, button="P"), ButtonEvent(0.5, 2, 1.0)],
        total_duration=4.0, created_at=datetime.now(UTC),
    )
    data = sp.schedule_from_dict(json.loads(json.dumps(sp.schedule_to_dict(s))))
    assert [(b.time_s, b.button, b.legacy_channel) for b in data.buttons] == [(0.5, None, 2), (1.0, "P", None)]
    assert sp.check_schedule(data) is not None  # 名前なしが残っている


def test_schema_button_name_or_legacy_channel() -> None:
    assert ButtonEventSchema(time_s=0, button="D").press_duration_s is None
    assert ButtonEventSchema(time_s=0, channel=1, press_duration_s=1.0).button is None  # 旧データの読み込み
    with pytest.raises(ValidationError):
        ButtonEventSchema(time_s=0)  # 名前も ch も無い
    with pytest.raises(ValidationError):
        ButtonEventSchema(time_s=0, button="X")
    with pytest.raises(ValidationError):
        ButtonEventSchema(time_s=0, channel=99, press_duration_s=1.0)
    req = ScheduleCreateRequest(name="a", button_events=[{"time_s": 1, "button": "N"}])
    assert req.button_events[0].button == "N"


def test_schema_rejects_negative_time() -> None:
    with pytest.raises(ValidationError):
        ScheduleCreateRequest(
            name="a", pedal_points=[{"time_s": -1, "accel_opening": 0, "brake_opening": 0}]
        )
    with pytest.raises(ValidationError):
        ButtonEventSchema(time_s=-0.5, button="D")
