"""TimeSchedule ドメインモデルのユニットテスト。"""

from datetime import UTC, datetime

from src.models.time_schedule import ButtonEvent, PedalPoint, TimeSchedule


def test_time_schedule_holds_pedal_and_button_timelines() -> None:
    sched = TimeSchedule(
        id="s1",
        name="test",
        description="desc",
        pedal_points=[
            PedalPoint(time_s=0.0, accel_opening=0.0, brake_opening=0.0),
            PedalPoint(time_s=5.0, accel_opening=40.0, brake_opening=0.0),
        ],
        button_events=[ButtonEvent(time_s=0.5, channel=0, press_duration_s=1.0)],
        total_duration=5.0,
        created_at=datetime.now(tz=UTC),
    )
    assert len(sched.pedal_points) == 2
    assert sched.button_events[0].channel == 0
    assert sched.pedal_points[1].accel_opening == 40.0
