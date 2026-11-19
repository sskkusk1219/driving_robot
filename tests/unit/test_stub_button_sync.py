# ruff: noqa: E501
"""差し替え 5（patches.install_stub_button_sync）: スタブのときだけ、ボタンを押す直前に車速を読んで車両の状態へ今のペダルを渡す。"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

import pytest

from src.robot import config_store as cs
from src.robot import patches
from src.robot.core import button_steps
from src.robot.core import hardware as core_hardware
from src.robot.core import main as core_main
from src.robot.core.vehicle import opening_to_pulse


def make_cfg(tmp_path: Path) -> Any:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    return cs.load_config(common)


async def stub_hw(cfg: Any) -> Any:
    hw = core_hardware.build_hardware(cfg, core_hardware.HW_STUB)
    await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    await hw.brake.move_to_position(opening_to_pulse(18.0))  # 停車保持（走行前チェックの後と同じ）
    return hw


async def test_stub_ready_press_with_brake_held_reaches_ready(tmp_path: Path) -> None:
    """手順 24 の最初の押下（有効化の直後・車速を読む前）でも、ブレーキを踏んでいれば Ready になる。"""
    patches.install_stub_button_sync()
    cfg = make_cfg(tmp_path)
    hw = await stub_hw(cfg)
    await button_steps.press_button(hw, cfg, "start", 0.05, poll=False, wait_after=False)
    assert hw.vehicle_status.state.value == "Ready"


async def test_original_function_misses_the_brake(tmp_path: Path) -> None:
    """（原因の確認）core の元の関数では、有効化直後の押下はブレーキ 0 と見なされ Ready にならない。"""
    cfg = make_cfg(tmp_path)
    hw = await stub_hw(cfg)
    await patches._orig_press_button(hw, cfg, "start", 0.05, poll=False, wait_after=False)  # noqa: SLF001
    assert hw.vehicle_status.state.value == "IGN-ON(RLY-ON)"


async def test_real_hw_is_untouched(monkeypatch: pytest.MonkeyPatch) -> None:
    """実機（hw.is_real）では元の関数をそのまま呼ぶ。有効化も車速の読み取りも足さない。"""
    patches.install_stub_button_sync()
    calls: list[str] = []

    class FakeCan:
        async def read_speed(self) -> float:
            calls.append("read_speed")
            return 0.0

    class FakeHw:
        is_real = True
        vehicle_status = object()  # 実機では None だが、あっても実機なら触らない
        can = FakeCan()

    async def fake_connect(_hw: Any) -> None:
        calls.append("ensure")

    async def fake_orig(_hw: Any, _cfg: Any, button: str, seconds: float, **kw: Any) -> None:
        calls.append(f"orig:{button}:{seconds}:{sorted(kw)}")

    monkeypatch.setattr(core_hardware, "ensure_buttons_connected", fake_connect)
    monkeypatch.setattr(patches, "_orig_press_button", fake_orig)
    await button_steps.press_button(FakeHw(), None, "D", 1.2, poll=False)
    assert calls == ["orig:D:1.2:['poll']"]


async def test_step24_wrapper_presses_brake_before_handler_and_press_button_does_not(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """手順 24 の包みは handler の前にブレーキ ON。press_button 自体はブレーキを触らない。"""
    patches.install_stub_button_sync()
    monkeypatch.setattr(patches, "_buttons_installed", True)
    calls: list[str] = []

    class FakeHw:
        is_real = True
        vehicle_status = None
        can = None

    class FakeCtx:
        config = None

        def require_hardware(self) -> FakeHw:
            return FakeHw()

    async def fake_set_brake(_hw: Any, _cfg: Any, *, on: bool) -> None:
        calls.append(f"brake:{on}")

    async def fake_orig(_hw: Any, _cfg: Any, button: str, seconds: float, **kw: Any) -> None:
        calls.append(f"orig:{button}")

    async def handler(_ctx: Any) -> None:
        calls.append("handler")

    monkeypatch.setattr(button_steps, "set_brake", fake_set_brake)
    monkeypatch.setattr(patches, "_orig_press_button", fake_orig)
    step = patches.brake_on_before_step24(core_main.Step(24, "x", handler))
    await step.handler(FakeCtx())
    assert calls == ["brake:True", "handler"]
    calls.clear()
    await button_steps.press_button(FakeHw(), None, "start", 1.0, poll=False, wait_after=False)
    assert calls == ["orig:start"]  # ブレーキは触らない

    # ボタンなしでは踏まない
    calls.clear()
    monkeypatch.setattr(patches, "_buttons_installed", False)
    await step.handler(FakeCtx())
    assert calls == ["handler"]
