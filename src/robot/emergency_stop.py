# ruff: noqa: E501
"""走行するジョブの非常停止の流れ（RPD_Phase_20261119_EmergencyStop）。

以前は「その場で両ペダルを原点復帰 → サーボOFF → 切断」で終わっていた。走行中だと車は惰行で
減速し、手順 24 では離した後にいきなり停車保持の開度まで踏んで N を押し、車は Ready・D のまま残った。
これを次の流れに変える（core は直さない。差し替えは patches.py の差し替え 10）。

きっかけ: SIGINT（非常停止スイッチ・画面の非常停止・停止ボタン・AC 断）と、走行中の自動の非常停止（DriveError）。

通常（CAN を読めて、アクチュエータも正常）:
    アクセルを原点へ → 車速 > no_speed_kmh なら 0.2G で停車（core の decelerate_to_stop）
    → 停車保持開度でブレーキを踏んで N → 車速が no_speed_kmh 以下か確かめる
    → 以下でなければ、停車保持〜最大開度を 3 等分して 1 段ずつ深くして N（計 4 回）
    → 以下になったらブレーキを離す → スタートを 10 秒押す（IGN-OFF）→ 両軸を原点復帰
    4 回でも止まらなければ、ブレーキを最大開度で踏んだまま終える（スタートは押さない・原点復帰も
    サーボ OFF もしない。サーボ ON のまま接続だけ閉じる）。警告を出す。
    N の押下で例外が出ても車速は読む。車速が no_speed_kmh 以下（停車）なのにボタンを押せないと分かったら、
    残りの回は押さない（ボタンの基板が応答しないとき、最大開度まで踏んでも N に入れないため）。ブレーキは
    停車保持の開度（今より浅くはしない）で止めたまま、サーボ ON で終える（スタートは押さない・原点復帰も
    しない）。stage=buttons_unavailable で知らせる（2026-11-20 実機: PCA9685 が I2C で応答せず、停車・
    IGN-OFF なのにブレーキ最大で残った）。車速 > no_speed_kmh のときは今までどおり次の段へ進む。
CAN を読めない:
    アクセルを原点へ → ブレーキを ``emergency.can_lost_press_s`` 秒（最小〜最大の通し）かけて最大開度まで
    刻んで踏む → 10 秒保つ → N → 離す → スタート 10 秒 → 原点復帰
アクチュエータの異常:
    ブレーキ軸の異常: 両軸をすぐ原点復帰 → スタート 10 秒。
    アクセル軸だけの異常: アクセルを原点へ戻し（戻せなくても進む）、ブレーキで上の流れ。

流れは 1 本の task で走らせ、``asyncio.shield`` で包む（中断の cancel の繰り返しでも止まらない。
2 回目の呼び出しは同じ task を待つだけ）。段ごとに state イベント（phase=emergency）を出す。
"""

from __future__ import annotations

import asyncio
import copy
import math
from dataclasses import dataclass, field
from typing import Any

from src.robot import events
from src.robot.core import button_steps as core_button_steps
from src.robot.core.button_servo import BUTTON_N, BUTTON_START
from src.robot.core.mode_drive import standby_openings
from src.robot.core.pre_drive_check import DIRECT_PRESS_SPEED_MM_S
from src.robot.core.stop_decel import decelerate_to_stop, step_to_position
from src.robot.core.term import say
from src.robot.core.vehicle import build_vehicle_profile, opening_to_pulse

CAN_LOST_PRESS_DEFAULT_S = 20.0
START_PRESS_DEFAULT_S = 10.0
#: 軸の状態を 1 回読むときの待ち時間の上限 [s]
PROBE_TIMEOUT_S = 3.0
#: 軸の状態を読む回数（読み取り失敗のときだけやり直す）と、やり直す前の待ち [s]。
#: 中断で取り消された通信のずれ（2026-11-19 実機 #2）を軸の異常と誤判定しないため。
#: 仮の値（根拠: 1 回のやり直しで戻るのに十分な短さ）
PROBE_ATTEMPTS = 3
PROBE_RETRY_WAIT_S = 0.3
#: CAN を読めないときの max までの踏み込みの 1 刻みの待ち [s]
CAN_LOST_STEP_DWELL_S = 0.5
#: max まで踏んだ後に保つ秒数 [s]（CAN を読めないとき。車が止まるのを待つ）
CAN_LOST_HOLD_S = 10.0
#: 停車確認の N を何段に分けるか（停車保持〜最大開度を 3 等分 → 開度は 4 つ）
N_LEVELS = 4

#: ブレーキを動かす速さ [mm/s]（button_steps の set_brake と同じ）
_brake_speed_mm_s = DIRECT_PRESS_SPEED_MM_S

# ── プロセスで 1 個の状態 ────────────────────────────────────────────
enabled = False  # 走行するジョブか（configure が決める）
brake_kept = False  # ブレーキを最大開度で踏んだまま終えたか
buttons_unavailable = False  # 停車しているのにボタンを押せず、ブレーキを停車保持の開度で止めたまま終えたか
handled = False  # 流れを始めたか（始めたら N の二重押しなどをしない）
_ctx: Any = None
_task: asyncio.Task[None] | None = None
_start_press_s = START_PRESS_DEFAULT_S
_can_lost_press_s = CAN_LOST_PRESS_DEFAULT_S


def configure(ctx: Any, *, enabled: bool) -> None:
    """ランナーが設定を読んだ後・手順の前に 1 回呼ぶ。状態を初期化する。

    ``ctx.config`` は手順 2 の保存で読み直されることがあるので、ctx は呼ぶたびに ``ctx.config`` を引く。
    src 専用キー（``emergency.can_lost_press_s``・``buttons.ign_off_press_s``）は読み直した設定には
    付かないので、ここで起動時の値を取っておく。
    """
    global _ctx, _start_press_s, _can_lost_press_s  # noqa: PLW0603
    reset()
    _ctx = ctx
    set_enabled(bool(enabled))
    src_only = getattr(getattr(ctx, "config", None), "src_only", {}) or {}
    _start_press_s = float(src_only.get("buttons.ign_off_press_s", START_PRESS_DEFAULT_S))
    _can_lost_press_s = float(src_only.get("emergency.can_lost_press_s", CAN_LOST_PRESS_DEFAULT_S))


def reset() -> None:
    """状態を初期化する（テスト・configure 用）。"""
    global _ctx, _task, brake_kept, buttons_unavailable, handled  # noqa: PLW0603
    _ctx = None
    _task = None
    brake_kept = False
    buttons_unavailable = False
    handled = False
    set_enabled(False)


def set_enabled(value: bool) -> None:
    global enabled  # noqa: PLW0603
    enabled = value


def armed() -> bool:
    """走行前チェックが済んでいるか（車が走れる状態になってから）。

    済む前の中断（手順 1 の途中など）では、車は IGN-OFF のままのことがある。そこでスタートを押すと
    IGN-ON になってしまうので、非常停止の流れは走らせない（従来どおり原点復帰・サーボ OFF だけ）。
    """
    return _ctx is not None and bool(getattr(_ctx, "pre_drive_checked", False))


def _emit(stage: str, detail: str = "") -> None:
    events.emit("state", phase="emergency", stage=stage, detail=detail)


# ── 異常の見分け ─────────────────────────────────────────────────────


@dataclass
class Diagnosis:
    can_ok: bool = True
    speed_kmh: float | None = None
    accel_faulty: bool = False
    brake_faulty: bool = False
    notes: list[str] = field(default_factory=list)


_FAULT_WORDS = ("過電流", "アラーム", "電流")


def faulty_axes_from_reason(reason: str) -> tuple[bool, bool]:
    """中断理由の文言から (アクセル軸の異常, ブレーキ軸の異常) を見分ける。

    「アクセル軸」「ブレーキ軸」と、過電流・アラーム・電流（0mA 継続）の語が両方あれば、その軸の異常。
    「アクチュエータ通信」は文言では見分けず、軸ごとの読み取り（classify）で見る。
    """
    text = reason or ""
    has_fault = any(w in text for w in _FAULT_WORDS)
    return (has_fault and "アクセル軸" in text, has_fault and "ブレーキ軸" in text)


async def _probe_axis(axis: Any, label: str = "軸") -> str | None:
    """軸の状態を読む。読めない・アラームなら理由、正常なら None。

    読み取りの失敗（例外）だけ ``PROBE_ATTEMPTS`` 回までやり直す。アラームは即 "アラーム" を返す。
    ``asyncio.CancelledError`` は Exception ではないので飲み込まない。
    """
    last_error = ""
    first_error = ""
    for attempt in range(1, PROBE_ATTEMPTS + 1):
        try:
            await asyncio.wait_for(axis.read_monitor(), PROBE_TIMEOUT_S)
            if await asyncio.wait_for(axis.is_alarm_active(), PROBE_TIMEOUT_S):
                return "アラーム"
        except Exception as exc:  # noqa: BLE001
            last_error = f"{type(exc).__name__}: {exc}"
            first_error = first_error or last_error
            if attempt < PROBE_ATTEMPTS:
                await asyncio.sleep(PROBE_RETRY_WAIT_S)
            continue
        if attempt > 1:
            say(f"  異常の確認: {label} 1 回目は読めず（{first_error}）、{attempt} 回目で読めました")
        return None
    return f"{last_error}（{PROBE_ATTEMPTS} 回読んで失敗）"


async def classify(hw: Any, reason: str) -> Diagnosis:
    """CAN 車速・各軸の状態・中断理由の文言から、どの流れにするかの材料を集める。

    軸の読み取りは失敗したときだけ数回やり直す。中断（SIGINT）が通信の最中に届くと pymodbus の
    応答がずれて 1 回だけ読めないことがあり、それを軸の異常と誤判定しないため（2026-11-19 実機）。
    """
    diag = Diagnosis()
    try:
        diag.speed_kmh = float(await hw.can.read_speed())
    except Exception as exc:  # noqa: BLE001
        diag.can_ok = False
        diag.notes.append(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）")
    accel_bad, brake_bad = await asyncio.gather(_probe_axis(hw.accel, "アクセル軸"), _probe_axis(hw.brake, "ブレーキ軸"))
    if accel_bad:
        diag.accel_faulty = True
        diag.notes.append(f"アクセル軸: {accel_bad}")
    if brake_bad:
        diag.brake_faulty = True
        diag.notes.append(f"ブレーキ軸: {brake_bad}")
    by_text_accel, by_text_brake = faulty_axes_from_reason(reason)
    if by_text_accel and not diag.accel_faulty:
        diag.accel_faulty = True
        diag.notes.append("アクセル軸の異常（中断理由の文言）")
    if by_text_brake and not diag.brake_faulty:
        diag.brake_faulty = True
        diag.notes.append("ブレーキ軸の異常（中断理由の文言）")
    return diag


# ── 流れ ─────────────────────────────────────────────────────────────


async def run(hw: Any, reason: str) -> None:
    """非常停止の流れを走らせて、終わるまで待つ。2 回目以降の呼び出しは同じ流れを待つだけ。

    呼び出し側が cancel されても流れは止まらない（shield）。
    """
    global _task, handled  # noqa: PLW0603
    handled = True
    if _task is None:
        _task = asyncio.ensure_future(_flow(hw, reason))
    await asyncio.shield(_task)


async def hold_and_keep(hw: Any, reason: str) -> None:
    """待機中の見張りで異常を見つけたとき（ユーザー決定 2026-11-20）。非常停止の流れ（``run``）は使わない。

    ボタン（N・スタートなど）は一切押さない。ブレーキ軸が正常なら停車保持の開度（``_brake_pulses`` の hold。
    今の位置より浅くしない）まで踏み、アクセルは原点へ（失敗しても続ける）。ブレーキを保ったまま終える扱い
    （``brake_kept``）にして、終了処理は原点復帰・サーボ OFF をせずサーボ ON のまま切る（``shutdown_keep_servo``）。
    """
    global brake_kept  # noqa: PLW0603
    cfg = _ctx.config
    diag = await classify(hw, reason)
    for note in diag.notes:
        say(f"  異常の確認: {note}")
    await _try(hw.accel.move_to_position(0), "アクセルを原点へ")
    if not diag.brake_faulty:
        _, hold_pulse, _ = _brake_pulses(cfg)
        target = hold_pulse
        try:
            target = max(hold_pulse, int(await hw.brake.read_position()))
        except Exception as exc:  # noqa: BLE001
            say(f"  警告: ブレーキの位置を読めません（{type(exc).__name__}: {exc}）。停車保持の開度へ動かします")
        await _try(_move_brake(hw, target), "ブレーキを停車保持の開度へ")
    brake_kept = True  # 終了処理は原点復帰・サーボ OFF をしない（shutdown_keep_servo）


async def wait_if_running() -> None:
    """流れが走っていれば、終わるまで待つ（shutdown の前に呼ぶ）。"""
    if _task is not None and not _task.done():
        await asyncio.shield(_task)


async def _flow(hw: Any, reason: str) -> None:
    """流れの本体。例外は外へ出さない（出しても終了処理は続けたいので、警告を出して終える）。"""
    try:
        await _flow_body(hw, reason)
    except asyncio.CancelledError:
        raise
    except Exception as exc:  # noqa: BLE001
        say(f"  警告: 非常停止の流れが途中で失敗しました（{type(exc).__name__}: {exc}）。終了処理へ進みます")
        _emit("error", f"{type(exc).__name__}: {exc}")


async def _flow_body(hw: Any, reason: str) -> None:
    cfg = _ctx.config
    say("=" * 60)
    say(f"非常停止の流れを始めます（理由: {reason or '中断要求'}）")
    _emit("start", reason or "")
    diag = await classify(hw, reason)
    for note in diag.notes:
        say(f"  異常の確認: {note}")
    if diag.brake_faulty:
        _emit("brake_fault", "ブレーキ軸の異常: すぐ原点復帰してスタートを押します")
        say("ブレーキ軸の異常: 両軸をすぐ原点復帰 → スタートを押します")
        await _home_both(hw)
        await _press_start(hw, cfg)
        _emit("done", "ブレーキ軸の異常のため原点復帰とスタート押下だけ行いました")
        return
    if diag.accel_faulty:
        _emit("accel_fault", "アクセル軸の異常: アクセルを原点へ戻します")
        say("アクセル軸の異常: アクセルを原点復帰します（戻せなくても進みます）")
        await _try(hw.accel.home_return(), "アクセル原点復帰")
    if not diag.can_ok:
        await _can_lost_flow(hw, cfg, accel_done=diag.accel_faulty)
        return
    stopped = (diag.speed_kmh or 0.0) <= cfg.buttons.no_speed_kmh
    if not stopped:
        outcome = await _decelerate(hw, cfg, accel_faulty=diag.accel_faulty)
        if outcome == "can_lost":
            await _can_lost_flow(hw, cfg, accel_done=True)
            return
    else:
        if not diag.accel_faulty:
            await _try(hw.accel.move_to_position(0), "アクセルを原点へ")
        say(f"停車中（{diag.speed_kmh:.2f} km/h）: 先に N を押します")
    await _n_loop(hw, cfg)


async def _try(coro: Any, label: str) -> bool:
    """失敗しても流れを止めずに 1 つ実行する。"""
    try:
        await coro
    except Exception as exc:  # noqa: BLE001
        say(f"  警告: {label}に失敗しました（{type(exc).__name__}: {exc}）")
        return False
    return True


class _NoopAccel:
    """アクセル軸が異常のとき、緩減速の「アクセルを原点へ」を何もしないことにする置き物。"""

    async def move_to_position(self, pos: int, *, smooth_over_s: float | None = None) -> None:
        return None


async def _decelerate(hw: Any, cfg: Any, *, accel_faulty: bool = False) -> str:
    """0.2G で停車させる。'ok' / 'timeout'（止まらず）/ 'can_lost'（途中で CAN が読めなくなった）。"""
    _emit("decel", "0.2G で停車させます")
    say("走行中: アクセルを原点へ戻し、0.2G で停車させます")
    try:
        target = hw
        if accel_faulty:  # アクセル軸が動かせない。ブレーキだけで止める
            target = copy.copy(hw)
            target.accel = _NoopAccel()
        await decelerate_to_stop(target, cfg, build_vehicle_profile(cfg))
    except Exception as exc:  # noqa: BLE001
        say(f"  緩減速が最後までいきませんでした（{type(exc).__name__}: {exc}）")
        try:
            await hw.can.read_speed()
        except Exception:  # noqa: BLE001
            return "can_lost"
        return "timeout"
    return "ok"


def _brake_pulses(cfg: Any) -> tuple[int, int, int]:
    """(最小（待機位置）, 停車保持, 最大) のブレーキ位置 [pulse]。"""
    _, brake_min_pct = standby_openings(cfg)
    min_pulse = opening_to_pulse(brake_min_pct)
    max_pulse = opening_to_pulse(cfg.vehicle.max_brake_opening_pct)
    hold_pulse = min(max_pulse, max(min_pulse, opening_to_pulse(cfg.stop_brake_opening_pct)))
    return min_pulse, hold_pulse, max_pulse


async def _move_brake(hw: Any, target_pulse: int) -> None:
    """ブレーキを目標位置へ _brake_speed_mm_s で動かし、動き終わるまで待つ。"""
    current = await hw.brake.read_position()
    duration_s = abs(target_pulse - current) / 100.0 / _brake_speed_mm_s
    await hw.brake.move_to_position_timed(target_pulse, current, duration_s)
    await asyncio.sleep(duration_s + 0.3)


async def _read_speed(hw: Any) -> float | None:
    try:
        return float(await hw.can.read_speed())
    except Exception:  # noqa: BLE001
        return None


async def _n_loop(hw: Any, cfg: Any) -> None:
    """停車保持の開度から最大開度まで 4 段に深くしながら N を押し、停車を確かめる。"""
    min_pulse, hold_pulse, max_pulse = _brake_pulses(cfg)
    levels = [round(hold_pulse + (max_pulse - hold_pulse) * i / (N_LEVELS - 1)) for i in range(N_LEVELS)]
    no_speed = cfg.buttons.no_speed_kmh
    for i, level in enumerate(levels, start=1):
        try:
            current = await hw.brake.read_position()
        except Exception:  # noqa: BLE001
            current = 0
        target = max(level, current)  # 今より浅くはしない（まだ動いているかもしれない）
        _emit("n_press", f"{i}/{len(levels)} 回目: ブレーキ {target} pulse で N")
        say(f"N の確認 {i}/{len(levels)}: ブレーキを {target} pulse（{target / 100.0:.1f}mm）まで踏んで N を押します")
        press_failed = False
        try:
            await _move_brake(hw, target)
            try:
                await core_button_steps.press_button(hw, cfg, BUTTON_N, cfg.buttons.shift_press_s)
            except Exception as exc:  # noqa: BLE001
                press_failed = True
                say(f"  警告: N の操作に失敗しました（{type(exc).__name__}: {exc}）")
        except Exception as exc:  # noqa: BLE001
            say(f"  警告: N の操作に失敗しました（{type(exc).__name__}: {exc}）")
            continue
        speed = await _read_speed(hw)  # N を押せなくても車速は読む
        if press_failed:
            if speed is not None and speed <= no_speed:
                await _stay_buttons_unavailable(speed, no_speed)
                return
            say(f"  車速 {'読めません' if speed is None else f'{speed:.2f} km/h'}: 止まっていないので、次の段へ進みます")
            continue
        if speed is not None and speed <= no_speed:
            say(f"  車速 {speed:.2f} km/h ≦ {no_speed:g} km/h: 停車を確認しました")
            await _release_and_ign_off(hw, cfg)
            return
        say(f"  車速 {'読めません' if speed is None else f'{speed:.2f} km/h'}: まだ止まっていません")
    await _keep_brake(hw, max_pulse)


async def _stay_buttons_unavailable(speed: float, no_speed: float) -> None:
    """停車しているのにボタンを押せない: 残りの回は押さず、ブレーキは今の開度（停車保持以上）で止めたまま終える。

    スタートは押さない・原点復帰もしない（サーボ ON のまま。shutdown は ``shutdown_keep_servo`` を使う）。
    """
    global buttons_unavailable  # noqa: PLW0603
    buttons_unavailable = True
    msg = "ボタンを押せません（停車保持のまま）"
    say(f"  車速 {speed:.2f} km/h ≦ {no_speed:g} km/h: 停車していますが、ボタンを押せません")
    say(f"*** 警告: {msg} ***")
    _emit("buttons_unavailable", msg)


async def _keep_brake(hw: Any, max_pulse: int) -> None:
    """止まらなかった: ブレーキを最大開度で踏んだまま終える（スタート・原点復帰・サーボ OFF はしない）。"""
    global brake_kept  # noqa: PLW0603
    brake_kept = True
    await _try(_move_brake(hw, max_pulse), "ブレーキを最大開度へ")
    msg = "N を 4 回押しても停車を確認できません。ブレーキを最大開度で踏んだままにします（サーボ ON のまま）"
    say(f"*** 警告: {msg} ***")
    _emit("brake_kept", msg)


async def _release_and_ign_off(hw: Any, cfg: Any) -> None:
    """停車を確認した後: ブレーキを離す → スタート 10 秒 → 両軸を原点復帰。"""
    _emit("release", "ブレーキを離します")
    await _try(core_button_steps.set_brake(hw, cfg, on=False), "ブレーキを離す")
    await _press_start(hw, cfg)
    _emit("home", "両軸を原点復帰します")
    await _home_both(hw)
    _emit("done", "停車 → N → IGN-OFF → 原点復帰まで済ませました")


async def _press_start(hw: Any, cfg: Any) -> None:
    """スタートを押す（IGN-OFF。ブレーキは離した状態）。押し終えたら stage=ign_off。"""
    say(f"スタートを {_start_press_s:g}s 押します（IGN-OFF）")
    ok = await _try(
        core_button_steps.press_button(hw, cfg, BUTTON_START, _start_press_s), "スタートの押下"
    )
    if ok:
        _emit("ign_off", f"スタートを {_start_press_s:g}s 押しました")
        core_button_steps.show_stub_status(hw)


async def _home_both(hw: Any) -> None:
    """両軸を原点復帰する（片方が失敗してももう片方は行う）。"""
    await asyncio.gather(
        _try(hw.accel.home_return(), "アクセル原点復帰"),
        _try(hw.brake.home_return(), "ブレーキ原点復帰"),
    )


async def _can_lost_flow(hw: Any, cfg: Any, *, accel_done: bool) -> None:
    """CAN を読めない: max まで刻んで踏む → 10 秒保つ → N → 離す → スタート → 原点復帰。"""
    _emit("can_lost", "CAN 車速を読めません: ブレーキを刻んで最大開度まで踏みます")
    say("CAN 車速を読めません: アクセルを原点へ戻し、ブレーキを最大開度まで刻んで踏みます")
    if not accel_done:
        await _try(hw.accel.move_to_position(0), "アクセルを原点へ")
    min_pulse, _hold, max_pulse = _brake_pulses(cfg)
    try:
        current = await hw.brake.read_position()
    except Exception:  # noqa: BLE001
        current = min_pulse
    span = max(1, max_pulse - min_pulse)
    seconds = max(0.0, _can_lost_press_s * max(0, max_pulse - current) / span)
    n_steps = max(1, math.ceil(seconds / CAN_LOST_STEP_DWELL_S))
    step_pulse = max(1, math.ceil(max(0, max_pulse - current) / n_steps))
    say(f"  ブレーキ {current} → {max_pulse} pulse を約 {seconds:.1f}s かけて踏みます")
    await _try(
        step_to_position(hw.brake, current, max_pulse, step_pulse=step_pulse, dwell_s=CAN_LOST_STEP_DWELL_S),
        "ブレーキを刻んで踏む",
    )
    say(f"  最大開度で {CAN_LOST_HOLD_S:g}s 保ちます")
    await asyncio.sleep(CAN_LOST_HOLD_S)
    _emit("n_press", "最大開度で N を押します")
    await _try(core_button_steps.press_button(hw, cfg, BUTTON_N, cfg.buttons.shift_press_s), "N の押下")
    await _release_and_ign_off(hw, cfg)


# ── 終了処理（ブレーキを踏んだまま終えるとき） ─────────────────────────


async def shutdown_keep_servo(hw: Any) -> None:
    """core の shutdown から「原点復帰」と「サーボ OFF」を除いたもの（サーボ ON のまま切る）。

    ブレーキを踏んだまま終えるとき（最大開度で保つとき・停車していてボタンを押せないとき）用。
    ここで原点復帰・サーボ OFF をすると車が動き出す。
    """
    if not hw.connected:
        return
    say("終了処理（ブレーキ保持）: ボタン解放 → UPS 停止 → 切断（原点復帰・サーボOFF はしません）…")
    steps: list[tuple[str, Any]] = []
    if hw.buttons is not None and hw.buttons_connected:
        steps.append(("ボタン解放", hw.buttons.release_all))
    steps.append(("UPS ポーリング停止", hw.ups.stop_polling))
    steps.append(("切断", lambda: asyncio.gather(hw.accel.close(), hw.brake.close(), hw.can.close())))
    if hw.buttons is not None and hw.buttons_connected:
        steps.append(("ボタン切断", hw.buttons.close))
    for label, make in steps:
        try:
            await make()
        except Exception as exc:  # noqa: BLE001
            say(f"  警告: {label}に失敗しました（{type(exc).__name__}: {exc}）")
    hw.connected = False
    hw.initialized = False
    hw.buttons_connected = False
    say("終了処理（ブレーキ保持）: 完了")
