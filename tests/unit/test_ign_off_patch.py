# ruff: noqa: E501
"""差し替え 7（patches.ign_off）: 手順 25 はスタートを buttons.ign_off_press_s 秒押す。停車していなければ押さない。"""

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


async def _stub_hw(cfg: Any) -> Any:
    hw = core_hardware.build_hardware(cfg, core_hardware.HW_STUB)
    await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    await hw.brake.move_to_position(opening_to_pulse(18.0))
    return hw


def _cfg(tmp_path: Path, press_s: float | None) -> Any:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    cfg = cs.load_config(common)
    if press_s is not None:
        cfg.src_only = {"buttons.ign_off_press_s": press_s}
    return cfg


async def test_ign_off_presses_start_for_configured_seconds(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    patches.install_ign_off()
    assert button_steps.ign_off is patches.ign_off  # core_main は属性で呼ぶので代入で効く
    assert core_main.button_steps is button_steps
    pressed: list[tuple[str, float]] = []

    async def fake_press(hw: Any, cfg: Any, button: str, seconds: float, **kw: Any) -> None:
        pressed.append((button, seconds))

    monkeypatch.setattr(button_steps, "press_button", fake_press)
    cfg = _cfg(tmp_path, 7.5)
    patches.configure(cfg)  # ランナーは設定を読んだ後に 1 回呼ぶ（秒数はここで決まる）
    await button_steps.ign_off(await _stub_hw(cfg), cfg)
    assert pressed == [("start", 7.5)]


async def test_ign_off_default_is_10s(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pressed: list[float] = []

    async def fake_press(hw: Any, cfg: Any, button: str, seconds: float, **kw: Any) -> None:
        pressed.append(seconds)

    monkeypatch.setattr(button_steps, "press_button", fake_press)
    cfg = _cfg(tmp_path, None)
    patches.configure(cfg)
    await patches.ign_off(await _stub_hw(cfg), cfg)
    assert pressed == [10.0]


async def test_ign_off_not_pressed_when_moving(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    pressed: list[float] = []

    async def fake_press(hw: Any, cfg: Any, button: str, seconds: float, **kw: Any) -> None:
        pressed.append(seconds)

    monkeypatch.setattr(button_steps, "press_button", fake_press)
    cfg = _cfg(tmp_path, 10.0)
    hw = await _stub_hw(cfg)

    async def moving() -> float:
        return 5.0

    monkeypatch.setattr(hw.can, "read_speed", moving)
    with pytest.raises(button_steps.DriveError):
        await patches.ign_off(hw, cfg)
    assert pressed == []


async def test_buttons_off_survives_config_reload(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """手順2 は保存後に設定を読み直す（src_only が付かない）。それでも ボタンなし のまま（11-16 段10-4）。"""
    from src.robot import schedule_job
    from src.robot.core import hardware as core_hardware

    # 差し替えはモジュールの属性を書き換えるので、テストの後に元へ戻す
    monkeypatch.setattr(button_steps, "press_button", button_steps.press_button)
    monkeypatch.setattr(button_steps, "ensure_buttons_connected", button_steps.ensure_buttons_connected)
    monkeypatch.setattr(core_hardware, "ensure_buttons_connected", core_hardware.ensure_buttons_connected)
    monkeypatch.setattr(schedule_job, "ensure_buttons_connected", schedule_job.ensure_buttons_connected)
    patches.install_buttons_off()
    cfg = _cfg(tmp_path, None)
    cfg.src_only = {"buttons.installed": False}
    patches.configure(cfg)
    reloaded = _cfg(tmp_path, None)
    if hasattr(reloaded, "src_only"):
        del reloaded.src_only
    try:
        hw = await _stub_hw(reloaded)
        await button_steps.press_button(hw, reloaded, "N", 0.01, poll=False, wait_after=False)
    finally:
        patches.configure(_cfg(tmp_path, None))  # 後のテストのために既定（ボタンあり）へ戻す
