# 移植元: tests/research/button_steps.py（scripts/port_research.py が 2026-11-19 に写した。tests の commit 4c80595+未コミットの変更あり）。手で直さない  # noqa: E501
"""スタートスイッチ・P/N/D ボタンを押す手順（手順 20〜25）の部品（ProblemReport_20261112）。

押す流れ（どのボタンをどの順に押すか）はここに書く。回数・秒数・開度は yaml の `buttons:` 節。
ボタンを押しても車の状態（Ready か・今のレンジ）は読めない（開ループ）。車速（CAN）だけ読める。
スタブでは車両の状態（VehicleStatus）を各手順の最後に表示し、机上で流れを確かめる。

ペダルは両方を同時に踏まない（アクセルを踏むときはブレーキを待機位置へ、逆も同じ）。
ブレーキ ON の開度は新しい値を作らず stop_brake_opening_pct（手順 2 が書く停車保持の開度）。
"""

from __future__ import annotations

import asyncio
import contextlib
import time

from src.robot.core.button_servo import (
    BUTTON_D,
    BUTTON_N,
    BUTTON_P,
    BUTTON_START,
    button_name,
)
from src.robot.core.config import ResearchConfig
from src.robot.core.hardware import (
    ActuatorProtocol,
    DriveError,
    ResearchHardware,
    ensure_buttons_connected,
)
from src.robot.core.mode_drive import standby_openings
from src.robot.core.pre_drive_check import DIRECT_PRESS_SPEED_MM_S
from src.robot.core.term import say
from src.robot.core.vehicle import opening_to_pulse

_POLL_S = 0.1  # 待つ間に CAN 車速を読む間隔 [s]
_PULSE_PER_MM = 100.0  # 1 mm = 100 pulse（pre_drive_check と同じ）
_SETTLE_S = 0.3  # ペダルの移動の後の余裕 [s]
_TAP_SMOOTH_S = 0.2  # アクセル踏みの移動にかける時間 [s]


async def _poll_speed(hw: ResearchHardware) -> float | None:
    """CAN 車速を 1 回読む。読めなければ None（IGN-OFF の間は CAN が止まることがある）。"""
    try:
        return await hw.can.read_speed()
    except Exception:  # noqa: BLE001 - 待つ間の読みは失敗しても続ける（鮮度を保つだけ）
        return None


async def wait_polling(hw: ResearchHardware, seconds: float) -> None:
    """seconds 待つ。待つ間は 0.1 秒ごとに車速を読む（スタブの状態が進み、実機は CAN が新しい）。"""
    loop = asyncio.get_running_loop()
    end = loop.time() + max(0.0, seconds)
    while True:
        await _poll_speed(hw)
        remaining = end - loop.time()
        if remaining <= 0.0:
            return
        await asyncio.sleep(min(_POLL_S, remaining))


def button_target(cfg: ResearchConfig, button: str) -> tuple[int, float]:
    """ボタンの名前から (ch, 指令する角度) を求める。角度 ＝ 基準の角度 ＋ offset_deg。"""
    item = cfg.buttons.servo_map[button]
    return int(item["channel"]), cfg.buttons.servo_rest_angle_deg + float(item["offset_deg"])


async def press_button(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    button: str,
    seconds: float,
    *,
    poll: bool = True,
    wait_after: bool = True,
) -> None:
    """ボタンを押し、`after_press_wait_s` 待つ。ボタン名・ch・角度・秒数を表示する。

    `poll=False` は押す間・押した後に車速を読まない（モード走行の周期が読んでいる最中に使う）。
    `wait_after=False` は押した後の待ちを呼び出し側が持つとき（手順 24 の D・Ready）。

    スタブのときは押す前後に車両の状態（VehicleStatus）へ押下を伝える（同じ ch に 2 つの
    ボタンがあるので、ch 番号ではなくボタンの名前で伝える）。
    """
    await ensure_buttons_connected(hw)
    assert hw.buttons is not None
    ch, angle = button_target(cfg, button)
    status = hw.vehicle_status
    if poll:
        await _poll_speed(hw)  # 押す直前の車速・ペダルを車両の状態へ渡す
    loop = asyncio.get_running_loop()
    say(
        f"  ボタン「{button_name(button)}」を ch{ch} {angle:g}° で {seconds:g}s 押します …"
        f"（t={loop.time():.1f}）"
    )
    if status is not None:
        status.on_press_start(button, time.monotonic())
    press = asyncio.create_task(hw.buttons.press(ch, angle, seconds))
    try:
        while not press.done():
            if poll:
                await _poll_speed(hw)
            await asyncio.sleep(_POLL_S)
        await press
    finally:
        if not press.done():
            press.cancel()  # 中断時は基準の角度へ戻る（press の finally）
            with contextlib.suppress(asyncio.CancelledError):
                await press
        if status is not None:
            status.on_press_end(button, time.monotonic())
    if wait_after:
        await wait_polling(hw, cfg.buttons.after_press_wait_s)


async def _move_pedal(axis: ActuatorProtocol, target_pulse: int) -> None:
    """ペダルを目標位置へ DIRECT_PRESS_SPEED_MM_S で動かし、動き終わるまで待つ。"""
    current = await axis.read_position()
    distance_mm = abs(target_pulse - current) / _PULSE_PER_MM
    duration_s = distance_mm / DIRECT_PRESS_SPEED_MM_S
    await axis.move_to_position_timed(target_pulse, current, duration_s)
    await asyncio.sleep(duration_s + _SETTLE_S)


async def set_brake(hw: ResearchHardware, cfg: ResearchConfig, *, on: bool) -> None:
    """ブレーキ ON（停車保持の開度）／OFF（待機位置 min）。アクセルは待機位置へ置く。"""
    accel_min, brake_min = standby_openings(cfg)
    await _move_pedal(hw.accel, opening_to_pulse(accel_min))
    pct = cfg.stop_brake_opening_pct if on else brake_min
    say(f"  ブレーキを {'ON' if on else 'OFF（待機位置）'}: {pct:.2f}%")
    await _move_pedal(hw.brake, opening_to_pulse(pct))
    await wait_polling(hw, 0.0)


async def release_brake(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    await set_brake(hw, cfg, on=False)


async def accel_taps(hw: ResearchHardware, cfg: ResearchConfig, n: int) -> None:
    """ブレーキを待機位置にしてから、アクセルを踏んで離すことを n 回行う（両踏みしない）。"""
    b = cfg.buttons
    accel_min, brake_min = standby_openings(cfg)
    await _move_pedal(hw.brake, opening_to_pulse(brake_min))
    say(f"  アクセル踏み × {n}（{b.accel_tap_opening_pct:g}% を {b.accel_tap_on_s:g}s ・"
        f"離して {b.accel_tap_off_s:g}s）")
    for _ in range(n):
        await hw.accel.move_to_position(
            opening_to_pulse(b.accel_tap_opening_pct), smooth_over_s=_TAP_SMOOTH_S
        )
        await wait_polling(hw, b.accel_tap_on_s)
        await hw.accel.move_to_position(opening_to_pulse(accel_min), smooth_over_s=_TAP_SMOOTH_S)
        await wait_polling(hw, b.accel_tap_off_s)


async def return_pedals_to_origin(hw: ResearchHardware) -> None:
    """両ペダルを原点（0 pulse）へ戻す。

    走行前チェックの「アクチュエータ位置」は原点にいることを確かめる。ボタンの手順はペダルを
    待機位置（min）に置いて終わるので、続けて走行する手順 24 の前にここで原点へ戻す。
    """
    say("  走行前チェックのため、両ペダルを原点へ戻します")
    await _move_pedal(hw.accel, 0)
    await _move_pedal(hw.brake, 0)


def show_stub_status(hw: ResearchHardware) -> None:
    """スタブの車両の状態を表示する（机上確認用。実機では何もしない）。"""
    status = hw.vehicle_status
    if status is None or not status.active:
        return
    speed = status.speed_kmh
    say(f"  [スタブ車両] {status.describe()}  Speed={speed:.1f}km/h")


async def _begin(hw: ResearchHardware) -> None:
    """ボタンを使う手順の最初: PCA9685 に接続する（スタブでは車両の状態も有効にする）。"""
    await ensure_buttons_connected(hw)


# ── 手順 ────────────────────────────────────────────────────────────


async def ign_on_rly_off(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 20: ブレーキを待機位置 → スタートを長押し（IGN-OFF → IGN-ON(RLY-OFF)）。"""
    await _begin(hw)
    await set_brake(hw, cfg, on=False)
    await press_button(hw, cfg, BUTTON_START, cfg.buttons.start_long_press_s)
    show_stub_status(hw)


async def maintenance_mode(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 21: 手順 20 → P を押す → P でアクセル踏み → N でアクセル踏み → P でアクセル踏み。

    メンテナンスモードは RLY-OFF から 60 秒以内に完了する必要があるため、先頭で手順 20 も行う。
    最初のレンジが P とは限らないので、最初に P を押してから踏む（ユーザー決定 2026-11-14）。
    P を押すのは停車中だけ（手順 20 の時点で車は止まっている）。
    """
    n = cfg.buttons.maintenance_tap_count
    await ign_on_rly_off(hw, cfg)
    await press_button(hw, cfg, BUTTON_P, cfg.buttons.shift_press_s)
    await accel_taps(hw, cfg, n)  # P × n
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, BUTTON_N, cfg.buttons.shift_press_s)
    await release_brake(hw, cfg)
    await accel_taps(hw, cfg, n)  # N × n
    await press_button(hw, cfg, BUTTON_P, cfg.buttons.shift_press_s)
    await accel_taps(hw, cfg, n)  # P × n
    show_stub_status(hw)


async def n_range_hold(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 22: ブレーキ ON → N → 待つ → N → ブレーキを離す（IGN-ON・P から N を 2 回）。"""
    await _begin(hw)
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, BUTTON_N, cfg.buttons.shift_press_s)
    await wait_polling(hw, cfg.buttons.n_hold_gap_s)
    await press_button(hw, cfg, BUTTON_N, cfg.buttons.shift_press_s)
    await release_brake(hw, cfg)
    show_stub_status(hw)


async def ready(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 23: ブレーキ ON → スタートを短押し → ブレーキを離す（→ Ready）。"""
    await _begin(hw)
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, BUTTON_START, cfg.buttons.start_short_press_s)
    await release_brake(hw, cfg)
    show_stub_status(hw)


async def ign_off(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 25: 停車中を確かめる → ブレーキ ON → スタートを短押し（→ IGN-OFF）。"""
    await _begin(hw)
    try:
        speed = await hw.can.read_speed()
    except Exception as exc:
        raise DriveError(
            f"IGN-OFF の前に CAN 車速を読めません（{type(exc).__name__}: {exc}）"
        ) from exc
    kmh = cfg.buttons.no_speed_kmh
    if speed > kmh:
        raise DriveError(
            f"停車していません（車速 {speed:.2f} km/h > {kmh:g} km/h）。IGN-OFF を押しません"
        )
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, BUTTON_START, cfg.buttons.start_short_press_s)
    show_stub_status(hw)


# ── 手順 24（モード走行）の中・後の押下 ───────────────────────────────


async def press_ready_in_mode(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 24: モード開始（0 秒）にスタートを短押しして Ready にする（停車保持済み）。

    押す間は車速を読まない（モード走行の周期が読んでいる）。押した後の待ちは持たない。
    """
    await _begin(hw)
    await press_button(
        hw, cfg, BUTTON_START, cfg.buttons.start_short_press_s, poll=False, wait_after=False
    )


async def press_shift_d_in_mode(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 24: モード開始から `d_press_at_s` 後に D を押す。押した後の待ち
    （`after_press_wait_s`）はモード走行側が持つ（待つ間にモードの時刻を止めるかを決めるため）。"""
    await _begin(hw)
    await press_button(hw, cfg, BUTTON_D, cfg.buttons.shift_press_s, poll=False, wait_after=False)


async def press_shift_n_after_drive(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 24: モードの後（中止でも）ブレーキを停車保持の開度にしてから N を押す。"""
    await _begin(hw)
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, BUTTON_N, cfg.buttons.shift_press_s)
    show_stub_status(hw)
