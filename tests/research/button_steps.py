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

from tests.research.button_servo import CH_D, CH_N, CH_P, CH_START, button_name
from tests.research.config import ResearchConfig
from tests.research.hardware import (
    ActuatorProtocol,
    DriveError,
    ResearchHardware,
    ensure_buttons_connected,
)
from tests.research.mode_drive import standby_openings
from tests.research.pre_drive_check import DIRECT_PRESS_SPEED_MM_S
from tests.research.term import say
from tests.research.vehicle import opening_to_pulse

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


async def press_button(hw: ResearchHardware, cfg: ResearchConfig, ch: int, seconds: float) -> None:
    """ボタンを押し、`after_press_wait_s` 待つ。押した時刻・ボタン名・秒数を表示する。"""
    await ensure_buttons_connected(hw)
    assert hw.buttons is not None
    await _poll_speed(hw)  # 押す直前の車速・ペダルを車両の状態へ渡す
    loop = asyncio.get_running_loop()
    say(f"  ボタン「{button_name(ch)}」を {seconds:g}s 押します …（t={loop.time():.1f}）")
    press = asyncio.create_task(hw.buttons.press(ch, seconds))
    try:
        while not press.done():
            await _poll_speed(hw)
            await asyncio.sleep(_POLL_S)
        await press
    finally:
        if not press.done():
            press.cancel()  # 中断時は待機角へ戻る（press の finally）
            with contextlib.suppress(asyncio.CancelledError):
                await press
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
    await press_button(hw, cfg, CH_START, cfg.buttons.start_long_press_s)
    show_stub_status(hw)


async def maintenance_mode(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 21: 手順 20 → P でアクセル踏み → N でアクセル踏み → P でアクセル踏み。

    メンテナンスモードは RLY-OFF から 60 秒以内に完了する必要があるため、先頭で手順 20 も行う。
    """
    n = cfg.buttons.maintenance_tap_count
    await ign_on_rly_off(hw, cfg)
    await accel_taps(hw, cfg, n)  # P × n
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, CH_N, cfg.buttons.shift_press_s)
    await release_brake(hw, cfg)
    await accel_taps(hw, cfg, n)  # N × n
    await press_button(hw, cfg, CH_P, cfg.buttons.shift_press_s)
    await accel_taps(hw, cfg, n)  # P × n
    show_stub_status(hw)


async def n_range_hold(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 22: ブレーキ ON → N → 待つ → N → ブレーキを離す（IGN-ON・P から N を 2 回）。"""
    await _begin(hw)
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, CH_N, cfg.buttons.shift_press_s)
    await wait_polling(hw, cfg.buttons.n_hold_gap_s)
    await press_button(hw, cfg, CH_N, cfg.buttons.shift_press_s)
    await release_brake(hw, cfg)
    show_stub_status(hw)


async def ready(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 23: ブレーキ ON → スタートを短押し → ブレーキを離す（→ Ready）。"""
    await _begin(hw)
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, CH_START, cfg.buttons.start_short_press_s)
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
    await press_button(hw, cfg, CH_START, cfg.buttons.start_short_press_s)
    show_stub_status(hw)


# ── 手順 24（モード走行）の前後の押下 ─────────────────────────────────


async def press_shift_d(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 24: 走行前チェックの後（停車保持の開度のまま）D を押す。"""
    await _begin(hw)
    await press_button(hw, cfg, CH_D, cfg.buttons.shift_press_s)
    show_stub_status(hw)


async def press_shift_n_after_drive(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """手順 24: モードの後（中止でも）ブレーキを停車保持の開度にしてから N を押す。"""
    await _begin(hw)
    await set_brake(hw, cfg, on=True)
    await press_button(hw, cfg, CH_N, cfg.buttons.shift_press_s)
    show_stub_status(hw)
