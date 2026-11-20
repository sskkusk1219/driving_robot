# ruff: noqa: E501
"""学習の手順（2・12〜14）の Ready・D・N（learning_buttons。D-14b）。偽のボタン操作・偽の手順で順序と回数を確かめる。"""

from __future__ import annotations

import asyncio
import io
import json
from types import SimpleNamespace
from typing import Any

import pytest

from src.robot import events, learning_buttons, run
from src.robot.core import button_steps as core_button_steps
from src.robot.core import main as core_main
from src.robot.interrupt import Interrupted, StopController


class FakeOps:
    def __init__(self, log: list[str], *, fail_on: str | None = None) -> None:
        self.log = log
        self.fail_on = fail_on

    async def _do(self, name: str) -> None:
        self.log.append(name)
        if self.fail_on == name:
            raise RuntimeError(f"{name} 失敗")

    async def ready(self, ctx: Any) -> None:
        await self._do("ready")

    async def d(self, ctx: Any) -> None:
        await self._do("d")

    async def n(self, ctx: Any) -> None:
        await self._do("n")


def make_step(number: int, log: list[str], *, exc: BaseException | None = None) -> core_main.Step:
    async def handler(ctx: Any) -> None:
        log.append(f"step{number}")
        if exc is not None:
            raise exc

    return core_main.Step(number, f"手順{number}", handler, drives=number != 1)


async def run_all(steps: list[core_main.Step]) -> None:
    for step in steps:
        await step.handler(None)


def build(numbers: list[int], log: list[str], *, ops: Any = None, exc_at: dict[int, BaseException] | None = None):
    steps = [make_step(n, log, exc=(exc_at or {}).get(n)) for n in numbers]
    state = SimpleNamespace(error="")
    out = learning_buttons.add_button_ops(steps, StopController(), state, ops=ops or FakeOps(log))
    return out, state


async def test_single_learning_step_order() -> None:
    log: list[str] = []
    steps, _ = build([1, 2], log)
    await run_all(steps)
    assert log == ["step1", "ready", "d", "step2", "n"]


async def test_consecutive_learning_steps_press_once() -> None:
    log: list[str] = []
    steps, _ = build([1, 13, 14], log)
    await run_all(steps)
    assert log == ["step1", "ready", "d", "step13", "step14", "n"]


async def test_no_learning_step_changes_nothing() -> None:
    """手順 24（モード走行）などの handler は包まない（Step がそのまま返る）。"""
    log: list[str] = []
    steps = [make_step(n, log) for n in (1, 24)]
    out = learning_buttons.add_button_ops(steps, StopController(), SimpleNamespace(error=""), ops=FakeOps(log))
    assert [s.handler for s in out] == [s.handler for s in steps]
    await run_all(out)
    assert log == ["step1", "step24"]


async def test_error_in_step_presses_n_and_reraises() -> None:
    log: list[str] = []
    steps, state = build([1, 13, 14], log, exc_at={13: RuntimeError("壊れた")})
    with pytest.raises(RuntimeError):
        await run_all(steps)
    assert log == ["step1", "ready", "d", "step13", "n"]
    assert "壊れた" in state.error


async def test_error_in_last_step_presses_n_once() -> None:
    log: list[str] = []
    steps, _ = build([1, 2], log, exc_at={2: RuntimeError("x")})
    with pytest.raises(RuntimeError):
        await run_all(steps)
    assert log.count("n") == 1


async def test_interrupted_does_not_press_n() -> None:
    log: list[str] = []
    steps, _ = build([1, 2], log, exc_at={2: Interrupted()})
    with pytest.raises(Interrupted):
        await run_all(steps)
    assert log == ["step1", "ready", "d", "step2"]


async def test_stop_requested_during_ready_does_not_press_n() -> None:
    log: list[str] = []
    steps = [make_step(1, log), make_step(2, log)]
    stop = StopController()
    stop.requested = True  # すでに中断要求
    state = SimpleNamespace(error="")
    out = learning_buttons.add_button_ops(steps, stop, state, ops=FakeOps(log))
    await out[0].handler(None)
    with pytest.raises(Interrupted):
        await out[1].handler(None)
    assert "n" not in log and "d" not in log


async def test_ready_failure_presses_n_and_skips_step() -> None:
    log: list[str] = []
    steps, state = build([1, 2], log, ops=FakeOps(log, fail_on="ready"))
    with pytest.raises(RuntimeError):
        await run_all(steps)
    assert log == ["step1", "ready", "n"]  # Ready の前後の状態が分からないので N で戻す。学習の走行はしない
    assert "ready 失敗" in state.error


async def test_n_failure_after_error_keeps_original_error() -> None:
    log: list[str] = []
    steps, _ = build([1, 2], log, ops=FakeOps(log, fail_on="n"), exc_at={2: ValueError("元のエラー")})
    with pytest.raises(ValueError, match="元のエラー"):
        await run_all(steps)


async def test_n_failure_after_success_raises() -> None:
    log: list[str] = []
    steps, state = build([1, 2], log, ops=FakeOps(log, fail_on="n"))
    with pytest.raises(RuntimeError):
        await run_all(steps)
    assert "N を押せませんでした" in state.error


async def test_state_events_emitted() -> None:
    buf = io.StringIO()
    events.set_sink(events.EventSink(stream=buf))
    try:
        log: list[str] = []
        steps, _ = build([1, 2], log)
        await run_all(steps)
    finally:
        events.set_sink(None)
    phases = [json.loads(line[len(events.EVENT_PREFIX):]).get("phase") for line in buf.getvalue().splitlines() if line.startswith(events.EVENT_PREFIX)]
    assert phases == ["ready", "d", "n"]


# ── 実際の押し方（core の関数を偽物に差し替えて順序と待ち時間を見る） ──────────


async def test_core_ops_sequence_and_waits(monkeypatch: pytest.MonkeyPatch) -> None:
    log: list[str] = []
    waits: list[float] = []

    async def set_brake(hw: Any, cfg: Any, *, on: bool) -> None:
        log.append(f"brake_{'on' if on else 'off'}")

    async def press_ready(hw: Any, cfg: Any) -> None:
        log.append("press_ready")
        await asyncio.sleep(0.05)

    async def press_d(hw: Any, cfg: Any) -> None:
        log.append("press_d")

    async def press_n(hw: Any, cfg: Any) -> None:
        log.append("press_n")

    async def wait_polling(hw: Any, seconds: float) -> None:
        waits.append(seconds)

    monkeypatch.setattr(core_button_steps, "set_brake", set_brake)
    monkeypatch.setattr(core_button_steps, "press_ready_in_mode", press_ready)
    monkeypatch.setattr(core_button_steps, "press_shift_d_in_mode", press_d)
    monkeypatch.setattr(core_button_steps, "press_shift_n_after_drive", press_n)
    monkeypatch.setattr(core_button_steps, "wait_polling", wait_polling)
    cfg = SimpleNamespace(
        stop_brake_opening_pct=18.0,
        buttons=SimpleNamespace(d_press_at_s=3.0, after_press_wait_s=1.0),
    )
    ctx = SimpleNamespace(require_hardware=lambda: object(), config=cfg)
    ops = learning_buttons.CoreButtonOps()
    await ops.ready(ctx)
    await ops.d(ctx)
    await ops.n(ctx)
    assert log == ["brake_on", "press_ready", "press_d", "press_n"]
    # D の前の待ちは「d_press_at_s − Ready の押下にかかった時間」、D の後は after_press_wait_s
    assert 2.8 < waits[0] <= 3.0 and waits[1] == 1.0


# ── check_job ────────────────────────────────────────────────


@pytest.mark.parametrize("numbers", [[1, 2, 24], [1, 13, 23], [1, 12, 24]])
def test_check_job_rejects_learning_with_ready_steps(numbers: list[int]) -> None:
    assert "同じ実行にできません" in (run.check_job(numbers, mode=None, limit_s=None) or "")


def test_check_job_allows_learning_with_25() -> None:
    assert run.check_job([1, 2, 25], mode=None, limit_s=None) is None


