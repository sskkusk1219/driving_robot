# ruff: noqa: E501
"""手動運転のスイッチの押し続け（src/robot/button_hold.py）と、差し替え 9 の hold イベント。"""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any

import pytest

from src.robot import button_hold, events, patches
from src.robot import config_store as cs
from src.robot.core import button_steps
from src.robot.core import hardware as core_hardware
from src.robot.core.vehicle import opening_to_pulse


@pytest.fixture()
def cfg(tmp_path: Path) -> Any:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    return cs.load_config(common)


@pytest.fixture()
def emitted(monkeypatch: pytest.MonkeyPatch) -> list[dict[str, Any]]:
    """差し替え 5・8・9 を入れ（終われば戻す）、button イベントを集める。"""
    from src.robot import schedule_job

    monkeypatch.setattr(button_steps, "press_button", button_steps.press_button)
    monkeypatch.setattr(button_steps, "say", button_steps.say)
    monkeypatch.setattr(button_steps, "ensure_buttons_connected", button_steps.ensure_buttons_connected)
    monkeypatch.setattr(core_hardware, "ensure_buttons_connected", core_hardware.ensure_buttons_connected)
    monkeypatch.setattr(schedule_job, "ensure_buttons_connected", schedule_job.ensure_buttons_connected)
    monkeypatch.setattr(patches, "_buttons_installed", True)
    patches.install_stub_button_sync()
    patches.install_buttons_off()
    patches.install_button_events()
    out: list[dict[str, Any]] = []
    monkeypatch.setattr(events, "emit", lambda kind, **f: out.append({"event": kind, **f}))
    return out


async def _hw(cfg: Any) -> Any:
    hw = core_hardware.build_hardware(cfg, core_hardware.HW_STUB)
    await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    await hw.brake.move_to_position(opening_to_pulse(18.0))
    return hw


async def test_down_up_presses_until_released(cfg: Any, emitted: list[dict[str, Any]]) -> None:
    hw = await _hw(cfg)
    hold = button_hold.ButtonHold(hw, cfg)
    ok, _ = await hold.down("start")
    assert ok
    await asyncio.sleep(0.35)
    assert hold.pressed == "start"
    assert len(hw.buttons.history) == 1  # 無限の秒数で 1 回だけ押している（inf で sleep できる）
    assert hw.vehicle_status.state.value != "IGN-OFF"  # 押している間
    ok, _ = await hold.up("start")
    assert ok and hold.pressed is None
    kinds = [(e["phase"], e.get("hold"), e["seconds"]) for e in emitted if e["event"] == "button"]
    assert kinds[0] == ("start", True, None)
    assert kinds[1][0] == "end" and kinds[1][1] is True and kinds[1][2] >= 0.3
    json.dumps(emitted)  # inf を含まず JSON にできる


async def test_first_press_wins_and_same_button_only_refreshes(cfg: Any, emitted: list[dict[str, Any]]) -> None:
    hw = await _hw(cfg)
    hold = button_hold.ButtonHold(hw, cfg)
    assert (await hold.down("start"))[0]
    ok, msg = await hold.down("P")
    assert not ok and "スタート" in msg
    assert (await hold.down("start"))[0]  # 送り直し
    await asyncio.sleep(0.2)
    assert len(hw.buttons.history) == 1
    # 先を離しても後の P は押されない
    await hold.up("start")
    assert hold.pressed is None
    assert len(hw.buttons.history) == 1
    # 押していない側の up は何もしない（OK）
    assert (await hold.up("P")) == (True, "押していません")


async def test_watchdog_releases_without_keepalive(cfg: Any, emitted: list[dict[str, Any]]) -> None:
    hw = await _hw(cfg)
    hold = button_hold.ButtonHold(hw, cfg, timeout_s=0.3)
    await hold.down("N")
    for _ in range(3):
        await asyncio.sleep(0.15)
        await hold.down("N")  # 送り直している間は離さない
    assert hold.pressed == "N"
    await asyncio.sleep(0.6)  # 送り直しを止める
    assert hold.pressed is None
    ends = [e for e in emitted if e["event"] == "button" and e["phase"] == "end"]
    assert len(ends) == 1


async def test_release_all(cfg: Any, emitted: list[dict[str, Any]]) -> None:
    hw = await _hw(cfg)
    hold = button_hold.ButtonHold(hw, cfg)
    await hold.release_all()  # 押していなくても例外にならない
    await hold.down("D")
    await asyncio.sleep(0.15)
    await hold.release_all()
    assert hold.pressed is None
    assert any(e["phase"] == "end" for e in emitted if e["event"] == "button")


async def test_buttons_off_setting_also_holds(cfg: Any, emitted: list[dict[str, Any]], monkeypatch: pytest.MonkeyPatch) -> None:
    """ボタンなし（差し替え 8）: サーボは動かさず、押したことにして待ち、離せば終わる。"""
    hw = await _hw(cfg)
    monkeypatch.setattr(patches, "_buttons_installed", False)
    hold = button_hold.ButtonHold(hw, cfg)
    await hold.down("P")
    await asyncio.sleep(0.25)
    assert hold.pressed == "P"
    assert hw.buttons.history == []
    await hold.up("P")
    assert hold.pressed is None


async def test_finite_press_event_unchanged(cfg: Any, emitted: list[dict[str, Any]]) -> None:
    hw = await _hw(cfg)
    await button_steps.press_button(hw, cfg, "start", 0.05, poll=False, wait_after=False)
    await asyncio.sleep(0.1)
    ev = [e for e in emitted if e["event"] == "button"]
    assert ev[0] == {"event": "button", "button": "start", "phase": "start", "seconds": 0.05}
    assert ev[1]["phase"] == "end" and ev[1]["seconds"] == 0.05 and "hold" not in ev[1]


async def test_hold_line_says_hold_not_infs(
    cfg: Any, emitted: list[dict[str, Any]], capsys: pytest.CaptureFixture[str]
) -> None:
    """押し続けのログは「infs 押します」ではなく「押し続けます（離すまで）」と書く。"""
    hw = await _hw(cfg)
    hold = button_hold.ButtonHold(hw, cfg)
    await hold.down("start")
    await asyncio.sleep(0.2)
    await hold.up("start")
    out = capsys.readouterr().out
    assert "infs" not in out and "押し続けます（離すまで）" in out
    from src.infra.button_events import parse_button_text

    assert parse_button_text("  ボタン「P」を ch0 145° で 押し続けます（離すまで） …") is None  # 秒数が無い行は読まない（BUTTON 行を使う）
