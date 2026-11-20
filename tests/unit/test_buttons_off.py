# ruff: noqa: E501
"""ボタンなし（車両 yaml の src 専用キー buttons.installed: false）と、手順 1 を含まないジョブの最小の接続（D-15c）。"""

from __future__ import annotations

import argparse
import asyncio
import time
from pathlib import Path
from typing import Any

import pytest
import yaml

from src.infra import vehicle_store as vs
from src.robot import config_store as cs
from src.robot import patches, quiet_connect, run
from src.robot.core import button_steps
from src.robot.core import hardware as core_hardware
from src.robot.core import main as core_main
from src.robot.core.vehicle import opening_to_pulse

COMMON = Path(__file__).resolve().parents[2] / "config" / "robot_common.yaml"


def _cfg(tmp_path: Path, installed: bool | None) -> Any:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    cfg = cs.load_config(common)
    if installed is not None:
        cfg.src_only = {"buttons.installed": installed}
    return cfg


@pytest.fixture()
def patched(monkeypatch: pytest.MonkeyPatch) -> None:
    """差し替え 8 を入れ、テストの後で元に戻す。"""
    from src.robot import schedule_job

    monkeypatch.setattr(button_steps, "press_button", button_steps.press_button)
    monkeypatch.setattr(button_steps, "ensure_buttons_connected", button_steps.ensure_buttons_connected)
    monkeypatch.setattr(core_hardware, "ensure_buttons_connected", core_hardware.ensure_buttons_connected)
    monkeypatch.setattr(schedule_job, "ensure_buttons_connected", schedule_job.ensure_buttons_connected)
    monkeypatch.setattr(patches, "_buttons_installed", True)
    patches.install_buttons_off()


async def _hw(cfg: Any) -> Any:
    hw = core_hardware.build_hardware(cfg, core_hardware.HW_STUB)
    await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    await hw.brake.move_to_position(opening_to_pulse(18.0))
    return hw


# ── 設定 ──────────────────────────────────────────────────────


def test_src_only_installed_default_true_and_removed_before_core() -> None:
    assert cs.src_only_values({})["buttons.installed"] is True
    assert cs.src_only_values({"buttons": {"installed": False}})["buttons.installed"] is False
    assert "buttons.installed" in cs.SRC_ONLY_KEYS


def test_common_installed_false_loads_through_core(tmp_path: Path) -> None:
    """ボタンの有無は共通設定の buttons.installed（段11）。車両 yaml の buttons: は無視される。"""
    store = vs.VehicleStore(tmp_path / "vehicles", COMMON)
    store.create("ボタンなし車", vehicle_id="nobtn")
    cfg, _ = cs.load_vehicle_config("nobtn", vehicles_dir=store.vehicles_dir, common_path=COMMON)  # core の検証を通る
    common_installed = yaml.safe_load(COMMON.read_text(encoding="utf-8"))["buttons"]["installed"]
    assert cfg.src_only["buttons.installed"] is common_installed
    assert patches.buttons_installed_in(cfg) is common_installed
    assert store.read("nobtn").buttons["installed"] is common_installed


# ── ボタンなしの押下 ───────────────────────────────────────────


async def test_press_is_pretended_without_connecting(patched: None, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    cfg = _cfg(tmp_path, False)
    patches.configure(cfg)
    hw = await _hw(cfg)
    t0 = time.monotonic()
    await button_steps.press_button(hw, cfg, "start", 0.3, poll=False, wait_after=False)
    assert time.monotonic() - t0 >= 0.25  # 押す秒数は待つ（制御の流れを合わせる）
    assert hw.buttons_connected is False  # PCA9685 に繋がない
    assert hw.vehicle_status is None or not hw.vehicle_status.active
    out = capsys.readouterr().out
    assert "ボタンなし" in out and "押したことにします" in out


async def test_every_button_step_skips_connection(patched: None, tmp_path: Path) -> None:
    cfg = _cfg(tmp_path, False)
    cfg.buttons.after_press_wait_s = 0.0
    cfg.buttons.start_short_press_s = 0.0
    cfg.buttons.shift_press_s = 0.0
    patches.configure(cfg)
    hw = await _hw(cfg)
    await button_steps.press_ready_in_mode(hw, cfg)
    await button_steps.press_shift_d_in_mode(hw, cfg)
    await button_steps.press_shift_n_after_drive(hw, cfg)  # ブレーキ ON は元のまま
    hw.can.speed_kmh = 0.0  # 走り終えて停車している（スタブ CAN の初期値はクリープ車速）
    await patches.ign_off(hw, cfg)
    assert hw.buttons_connected is False
    await core_hardware.ensure_buttons_connected(hw)  # 直接呼んでも繋がない
    assert hw.buttons_connected is False


async def test_installed_true_still_uses_original_press(patched: None, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    cfg = _cfg(tmp_path, True)
    patches.configure(cfg)
    hw = await _hw(cfg)
    await button_steps.press_button(hw, cfg, "start", 0.05, poll=False, wait_after=False)
    assert hw.buttons_connected is True  # 元の関数が PCA9685（スタブ）に繋いだ


async def test_press_emits_button_start_and_end_events_even_without_buttons(
    patched: None, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段11: 最も外側の press_button がボタンの押し始め・離し（seconds 後）を button イベントにする（ボタンなしでも）。"""
    import io

    from src.robot import events

    monkeypatch.setattr(button_steps, "press_button", button_steps.press_button)
    patches.install_button_events()
    cfg = _cfg(tmp_path, False)
    cfg.buttons.after_press_wait_s = 0.0
    patches.configure(cfg)
    hw = await _hw(cfg)
    sink = io.StringIO()
    events.set_sink(events.EventSink(stream=sink))
    try:
        await button_steps.press_button(hw, cfg, "D", 0.2, poll=False, wait_after=False)
        await asyncio.sleep(0.1)  # end は start から seconds 後
        got = [events.decode_event_line(ln) for ln in sink.getvalue().splitlines()]
        assert [(e["button"], e["phase"], e["seconds"]) for e in got if e and e["type"] == "button"] == [
            ("D", "start", 0.2), ("D", "end", 0.2),
        ]
    finally:
        events.set_sink(None)


# ── 手順 1 を含まないジョブの最小の接続 ─────────────────────────


async def test_connect_quietly_makes_hardware_ready_without_report(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    cfg = _cfg(tmp_path, None)
    ctx = core_main.RunContext(config=cfg, hw_mode=core_hardware.HW_STUB, started_at=time.monotonic())
    hw = await quiet_connect.prepare_hardware(ctx)
    try:
        assert ctx.hardware is hw and hw.initialized and hw.connected
        assert ctx.require_hardware() is hw  # core の手順がそのまま使える
        assert hw.buttons_connected is False  # ボタンはここでは繋がない
        out = capsys.readouterr().out
        assert "初期化結果" not in out and "[OK" not in out  # 初期化の表は出さない
    finally:
        await core_main.shutdown(hw)


def test_needs_quiet_connect_only_for_steps_without_step1() -> None:
    def args(interactive: bool = False) -> argparse.Namespace:
        return argparse.Namespace(interactive=interactive)

    assert run.needs_quiet_connect(args(), [24, 25]) is True
    assert run.needs_quiet_connect(args(), [2, 25]) is True
    assert run.needs_quiet_connect(args(), [25]) is True
    assert run.needs_quiet_connect(args(), []) is True  # スケジュール（手順 1 を入れない）
    assert run.needs_quiet_connect(args(), [1]) is False  # 初期化ページのジョブ
    assert run.needs_quiet_connect(args(), [1, 24, 25]) is False  # 手順 1 を自分で持つ
    assert run.needs_quiet_connect(args(interactive=True), [1]) is False
