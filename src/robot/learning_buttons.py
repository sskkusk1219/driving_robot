# ruff: noqa: E501
"""学習の手順（2・12〜14）の前後に、手順 24 と同じ Ready・D・N のボタン操作を足す（ユーザー決定 D-14b、2026-11-15）。

core の学習の手順にはボタン操作が無い（tests/research の手順 2 は手で Ready・D にしてから走る前提）ので、
ランナーが各 ``Step`` の handler を包んで足す（core は直さない）。

順序（1 回のジョブで学習の手順が続くときは、最初の前に 1 回・最後の後に 1 回）:
  走行前チェック（run_steps が済ませる）→ ブレーキを停車保持の開度に → スタート短押しで Ready →
  Ready から ``buttons.d_press_at_s`` 後に D（待つ間は車速を読み続ける）→ ``after_press_wait_s`` 待つ →
  学習の handler … → ``press_shift_n_after_drive``（ブレーキ ON で N）

- 学習の手順が失敗したとき（例外）も N を押す（押す前に失敗したら何もしない）。N が失敗しても元のエラーを優先する。
- SIGINT / SIGTERM の中断（Interrupted）では N を押さない（shutdown を優先。手順 24 と同じ）。
- D に入ったかの検出は足さない（学習にはモードの基準車速が無い。core の手順 2 の判定に任せる）。
"""

from __future__ import annotations

import asyncio
import dataclasses
from collections.abc import Sequence
from typing import Any, Protocol

from src.robot import events
from src.robot.core import button_steps as core_button_steps
from src.robot.core import main as core_main
from src.robot.core.term import say
from src.robot.interrupt import Interrupted, StopController

#: ボタン操作を足す手順（手順 2・12〜14）
LEARNING_STEPS = frozenset({2, 12, 13, 14})


class ButtonOps(Protocol):
    """3 つの操作。ctx を受けて HW を動かす（テストでは偽物に差し替える）。"""

    async def ready(self, ctx: core_main.RunContext) -> None: ...
    async def d(self, ctx: core_main.RunContext) -> None: ...
    async def n(self, ctx: core_main.RunContext) -> None: ...


class CoreButtonOps:
    """core の button_steps の関数（手順 24 と同じ押し方）で押す。

    関数は呼ぶたびにモジュールの属性から引く（patches の差し替え 5＝スタブのペダル同期が効くように）。
    """

    def __init__(self) -> None:
        self._ready_started_at: float | None = None

    async def ready(self, ctx: core_main.RunContext) -> None:
        hw, cfg = ctx.require_hardware(), ctx.config
        say(f"[学習] Ready のためブレーキ {cfg.stop_brake_opening_pct:.2f}% を踏みます（停車保持の開度）")
        await core_button_steps.set_brake(hw, cfg, on=True)
        say("[学習] Ready: スタートを短押しします")
        self._ready_started_at = asyncio.get_running_loop().time()
        await core_button_steps.press_ready_in_mode(hw, cfg)

    async def d(self, ctx: core_main.RunContext) -> None:
        hw, cfg = ctx.require_hardware(), ctx.config
        loop = asyncio.get_running_loop()
        started = self._ready_started_at if self._ready_started_at is not None else loop.time()
        wait_s = max(0.0, cfg.buttons.d_press_at_s - (loop.time() - started))
        say(f"[学習] Ready から {cfg.buttons.d_press_at_s:g} 秒後（あと {wait_s:.1f} 秒）に D を押します")
        await core_button_steps.wait_polling(hw, wait_s)
        say("[学習] D を押します")
        await core_button_steps.press_shift_d_in_mode(hw, cfg)
        await core_button_steps.wait_polling(hw, cfg.buttons.after_press_wait_s)
        say("[学習] D 済み。学習の走行を始めます")

    async def n(self, ctx: core_main.RunContext) -> None:
        hw, cfg = ctx.require_hardware(), ctx.config
        say("[学習] 終わり: ブレーキを停車保持の開度にして N を押します")
        await core_button_steps.press_shift_n_after_drive(hw, cfg)


@dataclasses.dataclass
class _Session:
    """1 回のジョブの Ready・D・N の進み具合。"""

    started: bool = False  # Ready を押し始めた（N を押す理由ができた）
    ended: bool = False  # N を押した（または押そうとした）


def add_button_ops(
    steps: Sequence[core_main.Step],
    stop: StopController,
    state: Any,
    *,
    ops: ButtonOps | None = None,
) -> list[core_main.Step]:
    """学習の手順の handler を包み、最初の前に Ready・D、最後の後に N を足した手順の並びを返す。

    steps は run.wrap_step で包んだ後のもの（state・中断はそこで付いている）。学習の手順が無ければそのまま返す。
    state は run.RunState（Ready・D・N の失敗の文を ``error`` に残す）。
    """
    idx = [i for i, s in enumerate(steps) if s.number in LEARNING_STEPS]
    out = list(steps)
    if not idx:
        return out
    first, last = idx[0], idx[-1]
    the_ops: ButtonOps = ops if ops is not None else CoreButtonOps()
    session = _Session()

    async def press_n(ctx: core_main.RunContext) -> None:
        session.ended = True
        events.emit("state", phase="n")
        await stop.run(lambda: the_ops.n(ctx))

    def wrap(step: core_main.Step, i: int) -> core_main.Step:
        inner = step.handler

        async def handler(ctx: core_main.RunContext) -> None:
            try:
                if i == first:
                    session.started = True
                    events.emit("state", phase="ready")
                    await stop.run(lambda: the_ops.ready(ctx))
                    events.emit("state", phase="d")
                    await stop.run(lambda: the_ops.d(ctx))
                await inner(ctx)
            except Interrupted:
                raise  # 中断では N を押さない（shutdown を優先）
            except Exception as exc:
                if not state.error:
                    state.error = f"{type(exc).__name__}: {exc}"
                if session.started and not session.ended:
                    try:
                        await press_n(ctx)
                    except Interrupted:
                        raise
                    except Exception as n_exc:  # noqa: BLE001 - 元のエラーを優先する
                        say(f"[学習] N を押せませんでした（{type(n_exc).__name__}: {n_exc}）")
                raise
            if i == last:
                try:
                    await press_n(ctx)
                except Interrupted:
                    raise
                except Exception as exc:
                    state.error = f"N を押せませんでした（{type(exc).__name__}: {exc}）"
                    raise

        return dataclasses.replace(step, handler=handler)

    for i in idx:
        out[i] = wrap(steps[i], i)
    return out

