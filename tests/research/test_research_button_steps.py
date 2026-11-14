"""ボタンの手順（20〜25、button_steps.py・main.py）のテスト（2026-11-12 段4・5）。

スタブ HW に車両の状態（VehicleStatus）をつなぎ、手順を流したあとの状態を確かめる。
ペダルの移動の待ちだけ速くして（実機の速度の仮定を外して）時間を縮める。
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from tests.research import button_steps as bsmod
from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import main as mainmod
from tests.research import mode_drive as mdmod
from tests.research.vehicle_status import State


@pytest.fixture(autouse=True)
def _fast_pedals(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(bsmod, "DIRECT_PRESS_SPEED_MM_S", 2000.0)
    monkeypatch.setattr(bsmod, "_SETTLE_S", 0.02)
    monkeypatch.setattr(bsmod, "_TAP_SMOOTH_S", 0.02)


def _cfg() -> cfgmod.ResearchConfig:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    b = cfg.buttons
    b.start_short_press_s = 0.1
    b.start_long_press_s = 2.1  # 表 2 の 2 s を超える最短（検証は通さず直接設定）
    b.shift_press_s = 0.1
    b.after_press_wait_s = 0.1
    b.accel_tap_on_s = 0.15
    b.accel_tap_off_s = 0.15
    b.n_hold_gap_s = 0.1
    return cfg


async def _hw(cfg: cfgmod.ResearchConfig) -> hwmod.ResearchHardware:
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    return hw


async def test_steps_20_to_25_chain_reaches_expected_states() -> None:
    cfg = _cfg()
    hw = await _hw(cfg)
    status = hw.vehicle_status
    assert status is not None

    await bsmod.ign_on_rly_off(hw, cfg)
    assert (status.state, status.range) == (State.IGN_ON_RLY_OFF, "P")
    assert status.active and hw.buttons_connected

    status.activate()  # 手順 21 は先頭で手順 20 をやり直すので、IGN-OFF から始め直す
    await bsmod.maintenance_mode(hw, cfg)
    assert status.state is State.IGN_ON_RLY_OFF
    assert status.range == "P" and status.maintenance is True

    await bsmod.n_range_hold(hw, cfg)
    assert status.range == "N" and status.n_hold is True

    await bsmod.ready(hw, cfg)
    assert status.state is State.READY and status.range == "N"
    assert status.n_hold and status.maintenance  # Ready になっても続く

    await bsmod.ign_off(hw, cfg)
    assert status.state is State.IGN_OFF
    assert status.range == "N" and status.n_hold is True and status.maintenance is False

    await hwmod.shutdown(hw)


async def test_step_21_alone_from_ign_off_sets_maintenance_within_60s() -> None:
    cfg = _cfg()
    hw = await _hw(cfg)
    await bsmod.maintenance_mode(hw, cfg)
    assert hw.vehicle_status is not None and hw.vehicle_status.maintenance is True
    await hwmod.shutdown(hw)


async def test_step_25_aborts_when_not_stopped() -> None:
    cfg = _cfg()
    hw = await _hw(cfg)
    await hwmod.ensure_buttons_connected(hw)
    hw.can.speed_kmh = 5.0  # type: ignore[union-attr]
    hw.can.vehicle = None  # type: ignore[union-attr]  # 車速を 5 km/h のまま返す
    with pytest.raises(hwmod.DriveError, match="停車していません"):
        await bsmod.ign_off(hw, cfg)
    assert hw.buttons.history == []  # type: ignore[union-attr]  # 押していない
    await hwmod.shutdown(hw)


async def test_buttons_not_connected_by_steps_without_buttons() -> None:
    """ボタンを使わない（手順 2・3 だけの）実行では PCA9685 に接続せず、車両の状態も無効のまま。"""
    cfg = _cfg()
    hw = await _hw(cfg)
    assert hw.buttons_connected is False
    assert hw.vehicle_status is not None and hw.vehicle_status.active is False
    await hwmod.shutdown(hw)


def test_steps_are_registered_and_listed() -> None:
    nums = {s.number: s for s in mainmod.STEPS}
    for n in (20, 21, 22, 23, 25):
        assert n in nums and nums[n].drives is False
    assert nums[24].drives and nums[24].has_ref and nums[24].prepare is not None
    assert mainmod.MAX_STEP == 25
    text = mainmod._steps_help()  # noqa: SLF001
    assert "手順:" in text and " 24 " in text


async def test_buttons_use_yaml_channel_and_angle() -> None:
    """スタートと P はどちらも ch0（10° と 170°）、N と D は ch1（10° と 170°）。"""
    cfg = _cfg()
    hw = await _hw(cfg)
    for button in ("start", "P", "N", "D"):
        await bsmod.press_button(hw, cfg, button, 0.0)
    pressed = [(r.channel, r.angle) for r in hw.buttons.history]  # type: ignore[union-attr]
    assert pressed == [(0, 32.0), (0, 180.0), (1, 3.0), (1, 175.0)]
    assert bsmod.button_target(cfg, "P") == (0, 180.0)
    await hwmod.shutdown(hw)


# ── 手順 24（D を押す・N を押す・中止） ──────────────────────────────────
class _FakeLog:
    csv_path = None
    closed = False

    async def close(self) -> None:
        self.closed = True


async def _run_step24(
    monkeypatch: pytest.MonkeyPatch, *, completed: bool, ignore_d: bool = False
) -> tuple[hwmod.ResearchHardware, dict[str, object]]:
    cfg = _cfg()
    hw = await _hw(cfg)
    seen: dict[str, object] = {}
    if ignore_d:  # スタブ車両が D を無視する（押した履歴は残るが D が入らない車）
        status = hw.vehicle_status
        assert status is not None
        orig = status.on_press_start

        def on_press_start(button: str, now: float) -> None:
            if button != "D":
                orig(button, now)

        monkeypatch.setattr(status, "on_press_start", on_press_start)

    async def fake_run(hw_, cfg_, setup_, **kwargs):  # type: ignore[no-untyped-def]
        seen["no_speed_abort"] = kwargs["no_speed_abort"]
        plan = kwargs["button_plan"]
        seen["plan"] = plan
        # モード中の押下を順に呼ぶ（Ready は 0 秒・D は d_press_at_s。時刻は mode_drive 側のテスト）
        pressed_before = len(hw_.buttons.history)
        await plan.ready_press()
        seen["state_after_ready"] = hw_.vehicle_status.state
        await plan.d_press()
        seen["pressed_in_run"] = [
            (r.channel, r.angle) for r in hw_.buttons.history[pressed_before:]
        ]
        seen["range_at_run"] = hw_.vehicle_status.range  # D を押した後の状態
        seen["brake_pos_at_run"] = await hw_.brake.read_position()
        # 実際の中止（run_mode_drive）はペダルを離して返す
        await hw_.brake.move_to_position(0)
        return mdmod.ModeDriveResult(
            mode_name="m", mode_duration_s=60.0, run_duration_s=3.0, completed=completed,
            abort_reason="" if completed else "D に入っていない可能性",
        )

    monkeypatch.setattr(mainmod, "run_mode_drive", fake_run)
    monkeypatch.setattr(mainmod, "_session_log_for_drive", lambda *a, **k: _FakeLog())
    ctx = mainmod.RunContext(config=cfg, hw_mode=hwmod.HW_STUB, started_at=0.0, hardware=hw)
    mode = SimpleNamespace(
        total_duration=60.0, reference_speed=[SimpleNamespace(time_s=0.0, speed_kmh=0.0),
                                              SimpleNamespace(time_s=1.0, speed_kmh=0.0),
                                              SimpleNamespace(time_s=5.0, speed_kmh=10.0)],
    )
    ctx.mode_setup = SimpleNamespace(mode=mode, ff=None)
    # 手順 24 の前（手順 21・22）を済ませた状態: IGN-ON・N。Ready は手順 24 の中で押す。
    # 走行前チェックは停車保持の開度にする
    await bsmod.maintenance_mode(hw, cfg)
    await bsmod.n_range_hold(hw, cfg)
    await bsmod.set_brake(hw, cfg, on=True)
    monkeypatch.setattr(mainmod, "write_mode_report", lambda *a, **k: None)
    monkeypatch.setattr(mainmod, "rows_from_samples", lambda s: [])
    monkeypatch.setattr(mainmod, "save_config_snapshot", lambda *a, **k: (None, None, None))
    monkeypatch.setattr(mainmod, "_fel_text_of", lambda *a, **k: None)
    monkeypatch.setattr(mainmod, "controller_text", lambda c: "")
    try:
        await mainmod.step24_mode_drive_buttons(ctx)
    finally:
        seen["ctx"] = ctx
    return hw, seen


async def test_step24_presses_ready_and_d_in_mode_then_n(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    hw, seen = await _run_step24(monkeypatch, completed=True)
    assert seen["no_speed_abort"] == (3.0, 0.2)  # yaml の既定（手順 3 は None）
    plan = seen["plan"]
    assert (plan.d_press_at_s, plan.after_d_wait_s) == (3.0, 0.1)  # type: ignore[attr-defined]
    # 走る前に D は押さない。モードの中で Ready（スタート ch0 10°）→ D（ch1 170°）
    assert seen["pressed_in_run"] == [(0, 32.0), (1, 175.0)]
    assert seen["state_after_ready"] is State.READY
    pressed = [(r.channel, r.angle) for r in hw.buttons.history]  # type: ignore[union-attr]
    assert pressed[-1] == (1, 3.0)  # 最後は N＝ch1 3°
    assert seen["range_at_run"] == "D"  # 走る間は D
    assert hw.vehicle_status is not None and hw.vehicle_status.range == "N"
    # 終わりは停車保持の開度のまま N を押した
    brake_pos = await hw.brake.read_position()
    assert brake_pos == bsmod.opening_to_pulse(_cfg().stop_brake_opening_pct)


async def test_step24_abort_presses_n_with_stop_hold_then_raises(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """D が入らず中止になっても、停車保持の開度にして N を押してから DriveError。"""
    holder: dict[str, hwmod.ResearchHardware] = {}
    orig_hw = hwmod.build_hardware

    def spy(cfg, mode):  # type: ignore[no-untyped-def]
        holder["hw"] = orig_hw(cfg, mode)
        return holder["hw"]

    monkeypatch.setattr(hwmod, "build_hardware", spy)
    with pytest.raises(hwmod.DriveError, match="D に入っていない"):
        await _run_step24(monkeypatch, completed=False, ignore_d=True)
    hw = holder["hw"]
    history = hw.buttons.history  # type: ignore[union-attr]
    pressed = [(r.channel, r.angle) for r in history]
    assert pressed[-3:] == [(0, 32.0), (1, 175.0), (1, 3.0)]  # Ready・D（入らない）→ N
    assert hw.vehicle_status is not None
    assert hw.vehicle_status.range == "N"  # D は入っていない
    brake_pos = await hw.brake.read_position()
    assert brake_pos == bsmod.opening_to_pulse(_cfg().stop_brake_opening_pct)


async def test_step21_presses_p_first_even_if_start_range_is_n() -> None:
    """最初のレンジが N でも、先に P を押してからアクセルを踏む（メンテナンスモードに入る）。"""
    cfg = _cfg()
    hw = await _hw(cfg)
    await hwmod.ensure_buttons_connected(hw)
    status = hw.vehicle_status
    assert status is not None
    status.activate()
    status.range = "N"  # 最初のレンジが P とは限らない
    await bsmod.maintenance_mode(hw, cfg)
    pressed = [(r.channel, r.angle) for r in hw.buttons.history]  # type: ignore[union-attr]
    # スタート長押し → P（ch0 170°）→ …（N → P）
    assert pressed[:2] == [(0, 32.0), (0, 180.0)]
    assert status.maintenance is True and status.range == "P"
    await hwmod.shutdown(hw)


def test_steps_23_and_24_in_one_run_is_a_config_error(
    capsys: pytest.CaptureFixture[str],
) -> None:
    steps = {s.number: s for s in mainmod.STEPS}
    assert mainmod.button_step_conflict([steps[1], steps[23], steps[24]]) is not None
    chain = [steps[n] for n in (1, 21, 22, 24, 25)]
    assert mainmod.button_step_conflict(chain) is None
    assert mainmod.button_step_conflict([steps[1], steps[23]]) is None
    assert mainmod.main(["--steps", "1,23,24"]) == 2
    assert "手順23 と同じ実行にできません" in capsys.readouterr().out


async def _order_of_run(numbers: tuple[int, ...], monkeypatch: pytest.MonkeyPatch) -> list[str]:
    """手順を空の handler/prepare に差し替えて run_steps を流し、呼ばれた順を返す。"""
    from dataclasses import replace

    events: list[str] = []

    def fake(step: mainmod.Step) -> mainmod.Step:
        async def handler(ctx: object, n: int = step.number) -> None:
            events.append(f"handler{n}")

        async def prepare(ctx: object, n: int = step.number) -> None:
            events.append(f"prepare{n}")

        return replace(
            step, handler=handler, prepare=prepare if step.prepare is not None else None
        )

    async def fake_check(ctx: object, **kwargs: object) -> None:
        events.append("check")
        ctx.pre_drive_checked = True  # type: ignore[attr-defined]

    monkeypatch.setattr(mainmod, "pre_drive_check", fake_check)
    monkeypatch.setattr(mainmod, "validate_config_once", lambda ctx: None)
    ctx = SimpleNamespace(
        require_hardware=lambda: None, pre_drive_checked=False, session_log=None, hardware=None
    )
    by_number = {s.number: s for s in mainmod.STEPS}
    code = await mainmod.run_steps(ctx, [fake(by_number[n]) for n in numbers])  # type: ignore[arg-type]
    assert code == 0
    return events


async def test_prepare_runs_once_before_first_button_step(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    events = await _order_of_run((1, 21, 22, 24, 25), monkeypatch)
    assert events == [
        "handler1", "prepare24", "handler21", "handler22", "check", "handler24", "handler25",
    ]


async def test_order_unchanged_without_button_steps(monkeypatch: pytest.MonkeyPatch) -> None:
    events = await _order_of_run((1, 3), monkeypatch)
    assert events == ["handler1", "prepare3", "check", "handler3"]
