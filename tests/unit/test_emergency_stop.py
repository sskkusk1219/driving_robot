# ruff: noqa: E501
"""非常停止の流れ（src/robot/emergency_stop.py・patches の差し替え 10）。偽の HW で順序・開度・秒数を確かめる。"""

from __future__ import annotations

import asyncio
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from src.robot import config_store as cs
from src.robot import emergency_stop as es
from src.robot import events, patches
from src.robot.core import button_steps
from src.robot.core import mode_drive as core_mode_drive
from src.robot.core import pattern_drive as core_pattern_drive
from src.robot.core.button_servo import BUTTON_N, BUTTON_START
from src.robot.core.vehicle import opening_to_pulse

_real_sleep = asyncio.sleep


class FakeAxis:
    def __init__(self, name: str, log: list[Any]) -> None:
        self.name = name
        self.log = log
        self.position = 0
        self.alarm = False
        self.monitor_error = False
        self.monitor_fail_times = 0  # 先頭からこの回数だけ read_monitor が例外（以降は成功）
        self.home_error = False

    async def read_monitor(self) -> Any:
        if self.monitor_fail_times > 0:
            self.monitor_fail_times -= 1
            raise ValueError("モニターは 14 レジスタ要ります（0 個）")
        if self.monitor_error:
            raise OSError("通信断")
        return SimpleNamespace()

    async def is_alarm_active(self) -> bool:
        return self.alarm

    async def read_position(self) -> int:
        return self.position

    async def move_to_position(self, pos: int, *, smooth_over_s: float | None = None) -> None:
        self.position = pos
        self.log.append((self.name, "move", pos))

    async def move_to_position_timed(self, target: int, current: int, duration: float) -> None:
        self.position = target
        self.log.append((self.name, "timed", target))

    async def home_return(self) -> None:
        if self.home_error:
            raise OSError("原点復帰できない")
        self.position = 0
        self.log.append((self.name, "home"))

    async def servo_off(self) -> None:
        self.log.append((self.name, "servo_off"))

    async def close(self) -> None:
        self.log.append((self.name, "close"))


class FakeCan:
    def __init__(self, hw_log: list[Any], speed: float | None) -> None:
        self.speed = speed  # None なら読めない
        self.log = hw_log

    async def read_speed(self) -> float:
        if self.speed is None:
            raise TimeoutError("CAN 車速が更新されていません")
        return self.speed

    async def close(self) -> None:
        self.log.append(("can", "close"))


class FakeButtons:
    def __init__(self, log: list[Any]) -> None:
        self.log = log

    async def release_all(self) -> None:
        self.log.append(("buttons", "release_all"))

    async def close(self) -> None:
        self.log.append(("buttons", "close"))


class FakeUps:
    def __init__(self, log: list[Any]) -> None:
        self.log = log

    async def stop_polling(self) -> None:
        self.log.append(("ups", "stop"))


def _make_hw(speed: float | None) -> Any:
    log: list[Any] = []
    return SimpleNamespace(
        log=log,
        accel=FakeAxis("accel", log),
        brake=FakeAxis("brake", log),
        can=FakeCan(log, speed),
        buttons=FakeButtons(log),
        ups=FakeUps(log),
        buttons_connected=True,
        connected=True,
        initialized=True,
        vehicle_status=None,
        is_real=False,
    )


def _cfg(tmp_path: Path, **src_only: Any) -> Any:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    cfg = cs.load_config(common)
    cfg.src_only = dict(src_only)
    return cfg


@pytest.fixture
def env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Any:
    """偽の HW・速い待ち・ボタンと緩減速の記録つきの環境。"""
    events_seen: list[dict[str, Any]] = []
    monkeypatch.setattr(events, "emit", lambda t, **f: events_seen.append({"type": t, **f}))

    async def fast_sleep(s: float, *a: Any) -> None:
        await _real_sleep(0)

    monkeypatch.setattr(asyncio, "sleep", fast_sleep)
    monkeypatch.setattr(es, "CAN_LOST_HOLD_S", 0.0)
    monkeypatch.setattr(es, "PROBE_TIMEOUT_S", 1.0)
    monkeypatch.setattr(es, "PROBE_RETRY_WAIT_S", 0.0)

    cfg = _cfg(tmp_path, **{"buttons.ign_off_press_s": 10.0, "emergency.can_lost_press_s": 20.0})
    ctx = SimpleNamespace(config=cfg, pre_drive_checked=True)
    es.configure(ctx, enabled=True)
    e = SimpleNamespace(cfg=cfg, ctx=ctx, events=events_seen, hw=None, stop_at_n=1, decel_stops=True, decel_calls=0, steps=[])

    def attach(hw: Any) -> Any:
        e.hw = hw
        n_count = {"n": 0}

        async def fake_press(h: Any, c: Any, button: str, seconds: float, **kw: Any) -> None:
            h.log.append(("press", button, seconds, h.brake.position))
            if button == BUTTON_N:
                n_count["n"] += 1
                if h.can.speed is not None and n_count["n"] >= e.stop_at_n:
                    h.can.speed = 0.0

        async def fake_set_brake(h: Any, c: Any, *, on: bool) -> None:
            h.log.append(("set_brake", on))
            h.brake.position = opening_to_pulse(c.stop_brake_opening_pct) if on else opening_to_pulse(c.vehicle.min_brake_opening_pct)

        async def fake_decel(h: Any, c: Any, profile: Any, **kw: Any) -> Any:
            e.decel_calls += 1
            h.log.append(("decel",))
            h.accel.position = 0
            h.brake.position = opening_to_pulse(c.stop_brake_opening_pct)
            if not e.decel_stops:
                raise RuntimeError("緩減速で 60s 以内に停車しません")
            h.can.speed = 0.0

        async def fake_step(axis: Any, start: int, target: int, *, step_pulse: int, dwell_s: float) -> int:
            e.steps.append((start, target, step_pulse, dwell_s))
            axis.position = target
            axis.log.append(("brake", "stepped", target))
            return target

        monkeypatch.setattr(button_steps, "press_button", fake_press)
        monkeypatch.setattr(button_steps, "set_brake", fake_set_brake)
        monkeypatch.setattr(es, "decelerate_to_stop", fake_decel)
        monkeypatch.setattr(es, "step_to_position", fake_step)
        return hw

    e.attach = attach
    yield e
    es.reset()


def _presses(hw: Any) -> list[tuple[str, float]]:
    return [(x[1], x[2]) for x in hw.log if x[0] == "press"]


def _stages(env: Any) -> list[str]:
    return [x["stage"] for x in env.events if x.get("phase") == "emergency"]


async def test_moving_decel_then_n_release_start_home(env: Any) -> None:
    hw = env.attach(_make_hw(speed=40.0))
    await es.run(hw, "SIGINT による中断")
    assert env.decel_calls == 1
    assert _presses(hw) == [(BUTTON_N, env.cfg.buttons.shift_press_s), (BUTTON_START, 10.0)]
    order = [x for x in hw.log if x[0] in ("decel", "press", "set_brake") or x[1:2] == ("home",)]
    kinds = [x[0] if x[0] != "press" else x[1] for x in order]
    assert kinds[:4] == ["decel", BUTTON_N, "set_brake", BUTTON_START]
    assert ("accel", "home") in hw.log and ("brake", "home") in hw.log
    assert ("set_brake", False) in hw.log
    assert not es.brake_kept
    stages = _stages(env)
    assert stages.index("ign_off") < stages.index("home") < stages.index("done")


async def test_stopped_presses_n_first_without_decel(env: Any) -> None:
    hw = env.attach(_make_hw(speed=0.0))
    await es.run(hw, "")
    assert env.decel_calls == 0
    assert _presses(hw)[0][0] == BUTTON_N
    assert _presses(hw)[-1] == (BUTTON_START, 10.0)
    # N を押したときのブレーキは停車保持開度
    assert [x for x in hw.log if x[0] == "press" and x[1] == BUTTON_N][0][3] == opening_to_pulse(env.cfg.stop_brake_opening_pct)


async def test_never_stops_four_levels_then_brake_kept_and_shutdown_skips_home(env: Any) -> None:
    env.stop_at_n = 99  # N を押しても止まらない
    env.decel_stops = False  # 緩減速でも止まらない
    hw = env.attach(_make_hw(speed=40.0))
    await es.run(hw, "SIGINT による中断")
    n_positions = [x[3] for x in hw.log if x[0] == "press" and x[1] == BUTTON_N]
    hold = opening_to_pulse(env.cfg.stop_brake_opening_pct)
    mx = opening_to_pulse(env.cfg.vehicle.max_brake_opening_pct)
    assert len(n_positions) == 4
    assert n_positions == sorted(n_positions)
    assert n_positions[0] == hold and n_positions[-1] == mx
    assert abs((n_positions[1] - hold) - (mx - hold) / 3) <= 1
    assert BUTTON_START not in [b for b, _ in _presses(hw)]
    assert es.brake_kept and hw.brake.position == mx
    assert "brake_kept" in _stages(env)
    assert ("brake", "home") not in hw.log
    # shutdown の差し替え: 原点復帰・サーボOFF をしない
    patches.install_state_events()
    await button_steps_core_shutdown(hw)
    assert ("brake", "servo_off") not in hw.log and ("accel", "servo_off") not in hw.log
    assert ("brake", "home") not in hw.log
    assert ("ups", "stop") in hw.log and ("brake", "close") in hw.log and ("buttons", "release_all") in hw.log


async def button_steps_core_shutdown(hw: Any) -> None:
    from src.robot.core import main as core_main  # noqa: PLC0415

    await core_main.shutdown(hw)


async def test_can_lost_steps_to_max_then_n_release_start(env: Any) -> None:
    hw = env.attach(_make_hw(speed=None))
    await es.run(hw, "CAN 車速を読めません")
    assert env.decel_calls == 0
    mx = opening_to_pulse(env.cfg.vehicle.max_brake_opening_pct)
    assert len(env.steps) == 1
    start, target, step_pulse, dwell = env.steps[0]
    assert target == mx
    # 最小から最大までを 20 秒で踏む: 刻み数 × 刻みの待ち ≒ 20 秒
    n_steps = -(-(mx - start) // step_pulse)
    assert abs(n_steps * dwell - 20.0) <= 1.0
    kinds = [x[1] if x[0] == "press" else x[0] for x in hw.log if x[0] in ("press", "set_brake")]
    assert kinds == [BUTTON_N, "set_brake", BUTTON_START]
    assert ("brake", "home") in hw.log
    assert "can_lost" in _stages(env)


async def test_can_lost_press_time_is_proportional_to_remaining(env: Any) -> None:
    hw = env.attach(_make_hw(speed=None))
    min_p, _hold, mx = es._brake_pulses(env.cfg)  # noqa: SLF001
    hw.brake.position = (min_p + mx) // 2  # すでに半分踏んでいる
    await es.run(hw, "")
    start, target, step_pulse, dwell = env.steps[0]
    n_steps = -(-(target - start) // step_pulse)
    assert abs(n_steps * dwell - 10.0) <= 1.0


async def test_brake_axis_fault_homes_then_start(env: Any) -> None:
    hw = env.attach(_make_hw(speed=40.0))
    hw.brake.alarm = True
    await es.run(hw, "")
    assert env.decel_calls == 0
    assert _presses(hw) == [(BUTTON_START, 10.0)]
    assert ("accel", "home") in hw.log and ("brake", "home") in hw.log
    assert "brake_fault" in _stages(env)


async def test_brake_axis_fault_even_if_home_fails_still_presses_start(env: Any) -> None:
    hw = env.attach(_make_hw(speed=40.0))
    hw.brake.monitor_error = True
    hw.brake.home_error = True
    await es.run(hw, "")
    assert _presses(hw) == [(BUTTON_START, 10.0)]
    assert ("accel", "home") in hw.log


async def test_brake_monitor_fails_once_then_ok_is_not_a_fault(env: Any) -> None:
    """中断で取り消された通信のずれ（1 回だけ読めない）は異常にしない。停車中なら通常どおり N から。"""
    hw = env.attach(_make_hw(speed=0.0))
    hw.brake.monitor_fail_times = 1
    await es.run(hw, "SIGINT による中断")
    assert "brake_fault" not in _stages(env)
    assert [b for b, _ in _presses(hw)] == [BUTTON_N, BUTTON_START]
    diag = await es.classify(_make_hw(speed=0.0), "")
    assert not diag.brake_faulty


async def test_brake_monitor_always_fails_is_still_a_fault(env: Any) -> None:
    hw = env.attach(_make_hw(speed=0.0))
    hw.brake.monitor_error = True
    diag = await es.classify(hw, "")
    assert diag.brake_faulty
    assert f"{es.PROBE_ATTEMPTS} 回読んで失敗" in diag.notes[0]
    await es.run(hw, "")
    assert "brake_fault" in _stages(env)
    assert _presses(hw) == [(BUTTON_START, 10.0)]


async def test_accel_only_fault_uses_brake_flow(env: Any) -> None:
    hw = env.attach(_make_hw(speed=40.0))
    hw.accel.alarm = True
    await es.run(hw, "")
    assert ("accel", "home") in hw.log
    assert env.decel_calls == 1
    assert [b for b, _ in _presses(hw)] == [BUTTON_N, BUTTON_START]


async def test_accel_fault_and_can_lost_goes_to_can_lost_flow(env: Any) -> None:
    hw = env.attach(_make_hw(speed=None))
    hw.accel.monitor_error = True
    await es.run(hw, "")
    assert len(env.steps) == 1
    assert [b for b, _ in _presses(hw)] == [BUTTON_N, BUTTON_START]


async def test_decel_fails_because_can_dropped_goes_to_can_lost(env: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    hw = env.attach(_make_hw(speed=30.0))

    async def broken_decel(h: Any, c: Any, profile: Any, **kw: Any) -> Any:
        h.can.speed = None
        raise RuntimeError("緩減速中に CAN 車速を読めません")

    monkeypatch.setattr(es, "decelerate_to_stop", broken_decel)
    await es.run(hw, "")
    assert len(env.steps) == 1  # 最大開度まで刻んで踏んだ
    assert [b for b, _ in _presses(hw)] == [BUTTON_N, BUTTON_START]


async def test_decel_timeout_goes_to_n_loop(env: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    hw = env.attach(_make_hw(speed=30.0))

    async def timeout_decel(h: Any, c: Any, profile: Any, **kw: Any) -> Any:
        raise RuntimeError("緩減速で 60s 以内に停車しません")

    monkeypatch.setattr(es, "decelerate_to_stop", timeout_decel)
    await es.run(hw, "")
    assert env.steps == []
    assert [b for b, _ in _presses(hw)] == [BUTTON_N, BUTTON_START]


async def test_second_call_awaits_same_task_and_survives_cancel(env: Any) -> None:
    hw = env.attach(_make_hw(speed=40.0))
    first = asyncio.ensure_future(es.run(hw, "自動の非常停止"))
    await _real_sleep(0)
    first.cancel()  # SIGINT の cancel（流れは止まらない）
    with pytest.raises(asyncio.CancelledError):
        await first
    await es.run(hw, "SIGINT")  # 2 回目: 同じ流れの終わりを待つだけ
    assert env.decel_calls == 1
    assert [b for b, _ in _presses(hw)] == [BUTTON_N, BUTTON_START]
    assert _stages(env).count("start") == 1


@pytest.mark.parametrize(
    ("reason", "expected"),
    [
        ("ブレーキ軸が過電流です（t=3.0s）", (False, True)),
        ("アクセル軸にアラームが発生しました（t=1.0s）", (True, False)),
        ("アクセル軸の電流が 1s 0mA のままです（指令位置 100、t=2.0s）", (True, False)),
        ("アクチュエータ通信に失敗しました（OSError: x）", (False, False)),
        ("車速が目標から外れました", (False, False)),
        ("", (False, False)),
    ],
)
def test_reason_classification(reason: str, expected: tuple[bool, bool]) -> None:
    assert es.faulty_axes_from_reason(reason) == expected


async def test_classify_reads_alarm_and_reason(env: Any) -> None:
    hw = _make_hw(speed=12.0)
    diag = await es.classify(hw, "ブレーキ軸が過電流です")
    assert diag.can_ok and diag.brake_faulty and not diag.accel_faulty
    hw.can.speed = None
    hw.accel.monitor_error = True
    diag = await es.classify(hw, "")
    assert not diag.can_ok and diag.accel_faulty


# ── patches の差し替え 10 ─────────────────────────────────────────────


@pytest.fixture
def stop() -> Any:
    from src.robot.interrupt import StopController  # noqa: PLC0415

    s = StopController()
    patches.install_all(s)
    patches._stop = s  # noqa: SLF001 - 2 回目以降の install_all でも更新される
    return s


async def test_release_pedals_patch_runs_flow_with_exception_reason(env: Any, stop: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    hw = env.attach(_make_hw(speed=40.0))
    reasons: list[str] = []
    orig_flow = es._flow_body  # noqa: SLF001

    async def spy(h: Any, reason: str) -> None:
        reasons.append(reason)
        await orig_flow(h, reason)

    monkeypatch.setattr(es, "_flow_body", spy)
    try:
        raise core_mode_drive.DriveError("ブレーキ軸が過電流です")
    except core_mode_drive.DriveError:
        await core_mode_drive._release_pedals(hw)  # noqa: SLF001
    assert "ブレーキ軸が過電流です" in reasons[0]
    assert es.handled
    # 流れが済んだ後の N は押さない
    before = len(_presses(hw))
    await button_steps.press_shift_n_after_drive(hw, env.cfg)
    assert len(_presses(hw)) == before


async def test_release_pedals_patch_noop_when_stop_requested_then_shutdown_runs_flow(env: Any, stop: Any) -> None:
    hw = env.attach(_make_hw(speed=40.0))
    stop.requested = True
    stop.signal_name = "SIGINT"
    try:
        await core_pattern_drive._release_pedals(hw)  # noqa: SLF001
        assert not es.handled
        from src.robot.core import main as core_main  # noqa: PLC0415

        await core_main.shutdown(hw)
        assert env.decel_calls == 1
        assert _presses(hw)[-1] == (BUTTON_START, 10.0)
        assert ("brake", "servo_off") in hw.log  # 通常の終了処理（原点復帰・サーボOFF）も続く
    finally:
        stop.requested = False
        stop.signal_name = ""


async def test_sigint_before_pre_check_does_not_run_flow(env: Any, stop: Any) -> None:
    hw = env.attach(_make_hw(speed=0.0))
    env.ctx.pre_drive_checked = False
    stop.requested = True
    try:
        from src.robot.core import main as core_main  # noqa: PLC0415

        await core_main.shutdown(hw)
        assert not es.handled and _presses(hw) == []
    finally:
        stop.requested = False


async def test_not_enabled_uses_original_release(env: Any, stop: Any) -> None:
    hw = env.attach(_make_hw(speed=40.0))
    es.set_enabled(False)
    await core_pattern_drive._release_pedals(hw)  # noqa: SLF001
    assert ("accel", "home") in hw.log and ("brake", "home") in hw.log
    assert _presses(hw) == []


async def test_stopped_but_button_press_fails_keeps_hold_brake_without_start(env: Any) -> None:
    """停車中で N の押下が例外 → 押下は 1 回だけ・ブレーキは停車保持・buttons_unavailable・start は押さない。"""
    hw = env.attach(_make_hw(speed=0.0))
    presses: list[str] = []

    async def failing_press(h: Any, c: Any, button: str, seconds: float, **kw: Any) -> None:
        presses.append(button)
        raise OSError(121, "Remote I/O error")

    button_steps.press_button = failing_press  # env の monkeypatch が teardown で元に戻す
    await es.run(hw, "SIGINT による中断")
    hold = opening_to_pulse(env.cfg.stop_brake_opening_pct)
    assert presses == [BUTTON_N]
    assert hw.brake.position == hold
    assert es.buttons_unavailable and not es.brake_kept
    stages = _stages(env)
    assert "buttons_unavailable" in stages and "brake_kept" not in stages
    assert "ign_off" not in stages and "done" not in stages
    assert ("brake", "home") not in hw.log
    # shutdown の差し替え: 原点復帰・サーボ OFF をしない
    patches.install_state_events()
    await button_steps_core_shutdown(hw)
    assert ("brake", "servo_off") not in hw.log and ("brake", "home") not in hw.log
    es.reset()
    assert not es.buttons_unavailable


async def test_hold_and_keep_presses_no_button_and_keeps_servo(env: Any) -> None:
    """待機中の見張りの異常: ボタンは押さない・ブレーキは停車保持（今より浅くしない）・アクセル原点・brake_kept。"""
    hw = env.attach(_make_hw(speed=0.0))
    presses: list[str] = []

    async def any_press(h: Any, c: Any, button: str, seconds: float, **kw: Any) -> None:
        presses.append(button)

    button_steps.press_button = any_press
    await es.hold_and_keep(hw, "待機中の見張りで異常: x")
    hold = opening_to_pulse(env.cfg.stop_brake_opening_pct)
    assert presses == [] and hw.brake.position >= hold
    assert ("accel", "home") not in hw.log and es.brake_kept
    stages = _stages(env)
    assert "start" not in stages and "ign_off" not in stages
    patches.install_state_events()
    await button_steps_core_shutdown(hw)
    assert ("brake", "servo_off") not in hw.log and ("brake", "home") not in hw.log
    es.reset()


async def test_moving_and_button_press_fails_goes_on_to_deeper_levels(env: Any) -> None:
    """車速 > no_speed で押下が失敗したときは今までどおり次の段へ（4 回で brake_kept）。"""
    hw = env.attach(_make_hw(speed=40.0))
    presses: list[str] = []

    async def failing_press(h: Any, c: Any, button: str, seconds: float, **kw: Any) -> None:
        presses.append(button)
        raise OSError(121, "Remote I/O error")

    button_steps.press_button = failing_press
    env.decel_stops = False  # 緩減速でも止まらない
    await es.run(hw, "SIGINT による中断")
    assert presses == [BUTTON_N] * 4
    assert es.brake_kept and not es.buttons_unavailable
