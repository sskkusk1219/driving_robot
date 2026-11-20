# ruff: noqa: E501
"""段11: ログ画面の走行グラフ（3 段の図）。ボタン押下の保存（level BUTTON）・旧メッセージ文の読み直し・図の点列。"""

import time
from datetime import UTC, datetime, timedelta
from types import SimpleNamespace

import pytest

from src.app import job_manager as jm
from src.infra.button_events import (
    LEVEL_BUTTON,
    button_message,
    parse_button_presses,
    parse_button_text,
    presses_to_intervals,
)
from src.web.analysis import build_figure

T0 = datetime(2026, 11, 16, 3, 0, 0, tzinfo=UTC)


def _msg(seq: int, sec: float, text: str, level: str = "INFO") -> SimpleNamespace:
    return SimpleNamespace(seq=seq, elapsed_s=sec, time=T0 + timedelta(seconds=sec), level=level, message=text)


def test_parse_legacy_lines() -> None:
    assert parse_button_text("  ボタンなし: 「N」を押したことにします（1.2s 待ちます）") == ("N", 1.2)
    assert parse_button_text("  ボタン「D」を ch1 175° で 1.2s 押します …") == ("D", 1.2)
    assert parse_button_text("  ボタンなし: 「スタート」を押したことにします（0.5s 待ちます）") == ("start", 0.5)
    assert parse_button_text("車速 0.0 km/h") is None


def test_parse_presses_prefers_button_rows_and_sorts() -> None:
    legacy = [_msg(1, 5.0, "  ボタンなし: 「N」を押したことにします（1.2s 待ちます）"), _msg(2, 9.0, "  ボタン「D」を ch1 175° で 1.2s 押します …")]
    assert [(p["button"], p["seconds"]) for p in parse_button_presses(legacy)] == [("N", 1.2), ("D", 1.2)]
    mixed = [*legacy, _msg(3, 5.0, button_message("N", 1.2), LEVEL_BUTTON)]
    got = parse_button_presses(mixed)
    assert len(got) == 1 and got[0]["button"] == "N" and got[0]["seconds"] == 1.2


def test_presses_to_intervals_interpolates_and_extrapolates() -> None:
    times = [T0 + timedelta(seconds=s) for s in (10.0, 20.0, 30.0)]
    xs = [0.0, 10.0, 20.0]
    presses = [{"button": "D", "seconds": 1.2, "time": T0 + timedelta(seconds=15.0)}, {"button": "N", "seconds": 1.0, "time": T0 + timedelta(seconds=8.0)}]
    out = presses_to_intervals(presses, times, xs)
    assert out[0] == {"button": "D", "seconds": 1.2, "start_s": 5.0, "end_s": 6.2}
    assert out[1]["start_s"] == -2.0 and out[1]["end_s"] == -1.0
    assert presses_to_intervals(presses, times[:1], xs[:1]) == []


def _row(i: int, section: str = "MODE_DRIVE", ref: float | None = 10.0) -> SimpleNamespace:
    return SimpleNamespace(
        timestamp=T0 + timedelta(seconds=100 + i * 0.1),
        ref_speed_kmh=ref,
        actual_speed_kmh=10.5,
        accel_opening=20.0,
        brake_opening=0.0,
        detail={"section": section, "mode_time_s": str(i * 0.1)},
    )


def test_build_figure_auto_uses_mode_time_and_maps_buttons() -> None:
    rows = [_row(i) for i in range(50)]
    press = {"button": "D", "seconds": 1.2, "time": T0 + timedelta(seconds=102.0)}
    fig = build_figure(rows, is_auto=True, presses=[press])
    assert fig is not None and fig["x_label"] == "モード時間 [s]"
    assert fig["t"][-1] == 4.9 and fig["ref"][0] == 10.0 and fig["act"][0] == 10.5
    assert fig["buttons"] == [{"button": "D", "seconds": 1.2, "start_s": 2.0, "end_s": 3.2}]


def test_build_figure_learning_uses_elapsed_and_no_ref() -> None:
    rows = [_row(i, section="X", ref=None) for i in range(20)]
    fig = build_figure(rows, is_auto=False)
    assert fig is not None and fig["x_label"] == "経過時間 [s]" and fig["ref"] is None and fig["t"][1] == 0.1
    assert build_figure(rows[:1], is_auto=False) is None


def test_job_manager_records_button_start_as_button_row() -> None:
    job = SimpleNamespace(
        status=jm.JobStatus(job_id="j", spec=jm.JobSpec(kind=jm.KIND_MODE_DRIVE, vehicle_id="v"), started_at=datetime.now(tz=UTC), t0=time.monotonic()),
        seq=4, pending_messages=[], pending={}, ready=SimpleNamespace(set=lambda: None),
    )
    jm.JobManager._dispatch(None, job, {"type": "button", "button": "N", "phase": "start", "seconds": 1.2})  # type: ignore[arg-type]
    jm.JobManager._dispatch(None, job, {"type": "button", "button": "N", "phase": "end", "seconds": 1.2})  # type: ignore[arg-type]
    assert len(job.pending_messages) == 1
    seq, _elapsed, when, level, text = job.pending_messages[0]
    assert seq == 5 and level == LEVEL_BUTTON and when.tzinfo is not None
    assert text == '{"button": "N", "seconds": 1.2}'
    assert parse_button_presses([_msg(1, 0.0, text, level)])[0]["button"] == "N"


@pytest.mark.parametrize("name", ["スタート", "P", "N", "D"])
def test_all_display_names_map_to_keys(name: str) -> None:
    got = parse_button_text(f"  ボタンなし: 「{name}」を押したことにします（1s 待ちます）")
    assert got is not None and got[0] == ("start" if name == "スタート" else name)
