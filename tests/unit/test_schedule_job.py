# ruff: noqa: E501
"""タイムスケジュールの再生（src/robot/schedule_job.py）をスタブ HW でプロセス内で確かめる。"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from src.robot import config_store as cs
from src.robot import run, schedule_job, schedule_spec
from src.robot.core import hardware as core_hardware
from src.robot.core import stop_decel as core_stop_decel
from src.robot.core.config import ConfigError
from src.robot.core.hardware import DriveError
from src.robot.core.research_types import VEHICLE_STOP_SPEED_KMH, DriveLogData
from src.robot.core.vehicle import opening_to_pulse


class FakeLog:
    """SessionLog の代わり（record だけ控える）。"""

    def __init__(self) -> None:
        self.rows: list[tuple[DriveLogData, dict[str, Any]]] = []
        self.marks: list[tuple[str, str]] = []

    async def stop_sampler(self) -> None:  # noqa: D401
        return None

    def start_sampler(self) -> None:
        return None

    def mark(self, section: str, phase: str) -> None:
        self.marks.append((section, phase))

    def record(self, data: DriveLogData, **kw: Any) -> None:
        self.rows.append((data, kw))


@pytest.fixture(autouse=True)
def fast_decel(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    """終端の緩減速（実時間で 20 秒かかる）を、呼ばれた車速を控えるだけの偽物に差し替える。本物は専用テストで通す。"""
    calls: list[float] = []

    async def fake(hw: Any, cfg: Any, profile: Any, *, log: Any = None) -> Any:
        calls.append(await hw.can.read_speed())
        return SimpleNamespace(hold_pct=cfg.stop_brake_opening_pct)

    monkeypatch.setattr(schedule_job, "decelerate_to_stop", fake)
    return calls


def make_cfg(tmp_path: Path) -> Any:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    cfg = cs.load_config(common)
    v = cfg.vehicle
    v.min_accel_opening_pct, v.max_accel_opening_pct = 4.0, 60.0
    v.min_brake_opening_pct, v.max_brake_opening_pct = 3.0, 40.0
    return cfg


async def make_hw(cfg: Any) -> Any:
    hw = core_hardware.build_hardware(cfg, core_hardware.HW_STUB)
    await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    return hw


def schedule(points: list[tuple[float, float, float]], buttons: list[dict[str, Any]] | None = None) -> schedule_spec.ScheduleData:
    return schedule_spec.schedule_from_dict({
        "id": "s", "name": "試験",
        "pedal_points": [{"time_s": t, "accel_opening": a, "brake_opening": b} for t, a, b in points],
        "button_events": buttons or [],
    })


async def test_playback_commands_follow_timeline_with_limits(tmp_path: Path, capsys: pytest.CaptureFixture[str], fast_decel: list[float]) -> None:
    cfg = make_cfg(tmp_path)
    hw = await make_hw(cfg)
    log = FakeLog()
    s = schedule([(0.0, 0.0, 0.0), (0.5, 30.0, 0.0), (1.0, 99.0, 0.0), (1.5, 0.0, 0.0)])
    run_ = await schedule_job.run_schedule_drive(hw, cfg, s, log=log)  # type: ignore[arg-type]
    assert run_.cycles >= 25
    assert log.marks[0] == ("SCHEDULE_DRIVE", "SCHEDULE")
    openings = [d.accel_opening for d, _ in log.rows]
    assert min(openings) == 4.0  # 0% は待機位置（min）
    assert max(openings) <= 60.0 and max(openings) > 50.0  # max で頭打ち（99% を指令しても 60%）
    assert all(d.brake_opening == 3.0 for d, _ in log.rows)  # 使わないブレーキは待機位置
    assert all(d.ref_speed_kmh is None for d, _ in log.rows)  # 基準車速なし
    assert "スケジュール再生 完了" in capsys.readouterr().out
    # 終端で車が動いていれば停車まで減速する（偽の緩減速が呼ばれ、停車保持の記録区間が付く）
    assert len(fast_decel) == 1 and fast_decel[0] >= VEHICLE_STOP_SPEED_KMH
    assert ("DECEL_TO_STOP", "APPROACH") in log.marks


async def test_end_of_schedule_decelerates_to_stop_with_real_core_function(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """core の本物の decelerate_to_stop で、動いたまま終わった車が止まり停車保持になる（スタブ）。"""
    cfg = make_cfg(tmp_path)
    cfg.decel_stop.dwell_s = 0.2
    cfg.decel_stop.slope_window_s = 0.4
    monkeypatch.setattr(schedule_job, "decelerate_to_stop", core_stop_decel.decelerate_to_stop)
    hw = await make_hw(cfg)
    s = schedule([(0.0, 0.0, 0.0), (0.4, 30.0, 0.0), (0.8, 0.0, 0.0)])
    await schedule_job.run_schedule_drive(hw, cfg, s, log=FakeLog())  # type: ignore[arg-type]
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    assert hw.accel.last_command_pos == 0
    assert hw.brake.last_command_pos >= opening_to_pulse(cfg.stop_brake_opening_pct) - 1


async def test_end_of_schedule_when_already_stopped_holds_immediately(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, fast_decel: list[float]
) -> None:
    cfg = make_cfg(tmp_path)
    hw = await make_hw(cfg)

    async def zero() -> float:
        return 0.0

    monkeypatch.setattr(hw.can, "read_speed", zero)
    log = FakeLog()
    await schedule_job.run_schedule_drive(hw, cfg, schedule([(0.0, 0.0, 0.0), (0.3, 0.0, 0.0)]), log=log)  # type: ignore[arg-type]
    assert fast_decel == []  # 減速は呼ばない
    assert log.marks[-1] == ("DECEL_TO_STOP", "STOP_HOLD")
    assert hw.accel.last_command_pos == 0
    assert hw.brake.last_command_pos == opening_to_pulse(cfg.stop_brake_opening_pct)


async def test_button_pressed_at_time_by_name_with_default_seconds(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = make_cfg(tmp_path)
    cfg.buttons.start_short_press_s, cfg.buttons.shift_press_s = 1.0, 1.2
    hw = await make_hw(cfg)
    pressed: list[tuple[str, float, float]] = []
    clock = asyncio.get_running_loop().time

    async def fake_press(_hw: Any, _cfg: Any, button: str, seconds: float, **kw: Any) -> None:
        pressed.append((button, seconds, clock() - t0))
        assert kw == {"poll": False, "wait_after": False}

    async def fake_connect(_hw: Any) -> None:
        return None

    monkeypatch.setattr(schedule_job.button_steps, "press_button", fake_press)
    monkeypatch.setattr(schedule_job, "ensure_buttons_connected", fake_connect)
    s = schedule(
        [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
        [{"time_s": 0.3, "button": "start"}, {"time_s": 0.6, "button": "D", "press_duration_s": 2.5}, {"time_s": 0.7, "button": "N"}],
    )
    t0 = clock()
    await schedule_job.run_schedule_drive(hw, cfg, s, log=FakeLog())  # type: ignore[arg-type]
    assert [(b, sec) for b, sec, _ in pressed] == [("start", 1.0), ("D", 2.5), ("N", 1.2)]
    assert [round(t, 1) for _, _, t in pressed] == [0.3, 0.6, 0.7]


async def test_button_failure_does_not_stop_pedals(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    cfg = make_cfg(tmp_path)
    hw = await make_hw(cfg)

    async def boom(*_a: Any, **_k: Any) -> None:
        raise OSError("I2C が応答しない")

    async def fake_connect(_hw: Any) -> None:
        return None

    monkeypatch.setattr(schedule_job.button_steps, "press_button", boom)
    monkeypatch.setattr(schedule_job, "ensure_buttons_connected", fake_connect)
    s = schedule([(0.0, 0.0, 0.0), (0.6, 10.0, 0.0)], [{"time_s": 0.1, "button": "D"}])
    run_ = await schedule_job.run_schedule_drive(hw, cfg, s, log=FakeLog())  # type: ignore[arg-type]
    assert run_.press_failures == 1
    assert "ボタン押下の失敗 1 回" in capsys.readouterr().out


async def test_simultaneous_press_is_allowed_during_crossover(tmp_path: Path) -> None:
    cfg = make_cfg(tmp_path)
    hw = await make_hw(cfg)
    log = FakeLog()
    # クロスオーバーの区間 [1.0, 1.2] の補間は両方が非ゼロになる。同時踏みは許すので止めずに出す
    s = schedule([(0.0, 0.0, 0.0), (0.4, 20.0, 0.0), (1.0, 20.0, 0.0), (1.2, 0.0, 20.0), (1.6, 0.0, 0.0)])
    await schedule_job.run_schedule_drive(hw, cfg, s, log=log)  # type: ignore[arg-type]
    assert any(d.accel_opening > 4.0 and d.brake_opening > 3.0 for d, _ in log.rows)


async def test_can_failure_releases_pedals_and_raises(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    cfg = make_cfg(tmp_path)
    hw = await make_hw(cfg)
    released: list[bool] = []

    async def boom() -> float:
        raise OSError("CAN が止まった")

    async def fake_release(_hw: Any) -> None:
        released.append(True)

    monkeypatch.setattr(hw.can, "read_speed", boom)
    monkeypatch.setattr(schedule_job, "_release_pedals", fake_release)
    with pytest.raises(DriveError, match="CAN 車速を読めません"):
        await schedule_job.run_schedule_drive(hw, cfg, schedule([(0.0, 5.0, 0.0), (1.0, 5.0, 0.0)]), log=FakeLog())  # type: ignore[arg-type]
    assert released == [True]


async def test_over_max_speed_aborts(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    cfg = make_cfg(tmp_path)
    hw = await make_hw(cfg)

    async def fast() -> float:
        return cfg.vehicle.max_speed_kmh + 5.0

    async def fake_release(_hw: Any) -> None:
        return None

    monkeypatch.setattr(hw.can, "read_speed", fast)
    monkeypatch.setattr(schedule_job, "_release_pedals", fake_release)
    with pytest.raises(DriveError, match="最高速"):
        await schedule_job.run_schedule_drive(hw, cfg, schedule([(0.0, 5.0, 0.0), (1.0, 5.0, 0.0)]), log=FakeLog())  # type: ignore[arg-type]


async def test_cancel_mid_playback_cancels_button_tasks(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """中断（cancel）で、押している最中のボタンのタスクも止まる（サーボは基準の角度へ戻る側が処理する）。"""
    cfg = make_cfg(tmp_path)
    hw = await make_hw(cfg)
    state = {"started": False, "cancelled": False}

    async def slow_press(*_a: Any, **_k: Any) -> None:
        state["started"] = True
        try:
            await asyncio.sleep(30)
        except asyncio.CancelledError:
            state["cancelled"] = True
            raise

    async def fake_connect(_hw: Any) -> None:
        return None

    monkeypatch.setattr(schedule_job.button_steps, "press_button", slow_press)
    monkeypatch.setattr(schedule_job, "ensure_buttons_connected", fake_connect)
    s = schedule([(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)], [{"time_s": 0.1, "button": "D"}])
    task = asyncio.create_task(schedule_job.run_schedule_drive(hw, cfg, s, log=FakeLog()))  # type: ignore[arg-type]
    await asyncio.sleep(0.5)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    assert state == {"started": True, "cancelled": True}


# ── ランナーの引数・読み込み ─────────────────────────────────


def ns(**kw: Any) -> argparse.Namespace:
    base = {"schedule": None, "schedule_file": None}
    return argparse.Namespace(**{**base, **kw})


async def test_load_schedule_from_file_and_db_and_errors(tmp_path: Path) -> None:
    cfg = make_cfg(tmp_path)
    f = tmp_path / "s.json"
    f.write_text(json.dumps({"id": "i", "name": "ファイル", "pedal_points": [{"time_s": 0, "accel_opening": 1, "brake_opening": 0}], "button_events": []}), encoding="utf-8")
    assert (await run.load_schedule(ns(schedule_file=f), cfg)).name == "ファイル"
    with pytest.raises(ConfigError, match="読めません"):
        await run.load_schedule(ns(schedule_file=tmp_path / "none.json"), cfg)

    seen: list[tuple[str, str]] = []

    async def fetch(url: str, text: str) -> dict[str, Any] | None:
        seen.append((url, text))
        if text == "ある":
            return {"id": "x", "name": "ある", "pedal_points": [], "button_events": [{"time_s": 1, "channel": 0, "press_duration_s": 1.0}]}
        return None

    got = await run.load_schedule(ns(schedule="ある"), cfg, fetch=fetch)
    assert got.name == "ある" and got.buttons[0].button is None
    with pytest.raises(ConfigError, match="見つかりません"):
        await run.load_schedule(ns(schedule="ない"), cfg, fetch=fetch)
    assert [t for _, t in seen] == ["ある", "ない"]


def test_runner_parser_schedule_options_are_exclusive() -> None:
    p = run.build_parser()
    a = p.parse_args(["--vehicle", "v", "--schedule", "S1"])
    assert a.schedule == "S1" and a.steps is None and not a.interactive
    b = p.parse_args(["--vehicle", "v", "--schedule-file", "/tmp/x.json"])
    assert str(b.schedule_file) == "/tmp/x.json"
    with pytest.raises(SystemExit):
        p.parse_args(["--vehicle", "v", "--schedule", "S1", "--steps", "1,3"])
