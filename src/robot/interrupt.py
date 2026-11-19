# ruff: noqa: E501
"""中断（SIGINT / SIGTERM）の扱い。

core の run_steps は「どこで例外が出ても finally で原点復帰・サーボOFF・切断」を必ず行う作り。
ランナーはこの finally を邪魔しないよう、**run_steps 全体は cancel せず**、手順の本体（handler・
prepare・走行前チェック）だけを内側のタスクで走らせ、中断要求が来たらその内側のタスクだけを cancel して
`Interrupted` を投げる。run_steps の except 節は `Interrupted` を拾わないので finally（終了処理）へ
そのまま進み、終了処理中は cancel されない。2 回目以降の中断要求は無視する。

pymodbus は応答待ち中の cancel を握りつぶして別の例外にすることがある（メモリ: 読み取り中の
task.cancel() は止まらない）。そのため内側のタスクが終わるまで 0.5 s ごとに cancel を繰り返す。
"""

from __future__ import annotations

import asyncio
import threading
from collections.abc import Awaitable, Callable
from typing import TypeVar

from src.robot.core.term import say

T = TypeVar("T")

#: 内側のタスクへ cancel を繰り返す間隔 [s]
RECANCEL_INTERVAL_S = 0.5


class Interrupted(BaseException):  # noqa: N818 - 中断を表す。Exception にしない（core の except に拾わせない）
    """中断要求（SIGINT / SIGTERM）で手順を打ち切った。"""


class StopController:
    """中断要求の状態と、手順の本体を中断できる形で走らせる入口。"""

    def __init__(self) -> None:
        self.requested = False
        self.signal_name = ""
        self._inner: asyncio.Task[object] | None = None
        self._threads: list[threading.Thread] = []

    def request(self, signal_name: str) -> None:
        """シグナルハンドラから呼ぶ。最初の 1 回だけ受け付け、実行中の内側のタスクを cancel する。"""
        if self.requested:
            say(f"{signal_name}: 中断処理中のため無視します（終了処理を邪魔しない）")
            return
        self.requested = True
        self.signal_name = signal_name
        say(f"{signal_name} を受けました。手順を打ち切って終了処理へ進みます（走行するジョブは停止の流れ: 停車 → N → IGN-OFF → 原点復帰）")
        if self._inner is not None and not self._inner.done():
            self._inner.cancel()

    @property
    def orphan_alive(self) -> bool:
        """中断で見捨てた作業スレッドがまだ動いているか（その間 Python の通常の終了は待たされる）。"""
        return any(t.is_alive() for t in self._threads)

    async def in_thread(self, factory: Callable[[], Awaitable[T]]) -> T:
        """factory() を**別スレッドの別のイベントループ**で走らせて結果を待つ。

        core の準備（prepare_mode_drive など）は同期の重い計算（WLTP で約 2 分）を含み、メインの
        イベントループを止める。止まっている間はシグナルも受け付けられないので、準備は別スレッドへ
        逃がし、メインのループは中断要求を受けられるようにする。準備は HW を触らない・ctx に結果を
        書くだけ。中断したらスレッドは見捨てる（デーモン。ランナーの main が os._exit で終わる）。
        """
        loop = asyncio.get_running_loop()
        fut: asyncio.Future[T] = loop.create_future()

        def deliver(result: T | None, exc: BaseException | None) -> None:
            if fut.cancelled():
                return
            if exc is not None:
                fut.set_exception(exc)
            else:
                fut.set_result(result)  # type: ignore[arg-type]

        def work() -> None:
            try:
                result = asyncio.run(factory())  # type: ignore[arg-type]
            except BaseException as exc:  # noqa: BLE001
                loop.call_soon_threadsafe(deliver, None, exc)
            else:
                loop.call_soon_threadsafe(deliver, result, None)

        thread = threading.Thread(target=work, daemon=True, name="robot-prepare")
        self._threads.append(thread)
        thread.start()
        return await fut

    @property
    def exit_code(self) -> int:
        return 143 if self.signal_name == "SIGTERM" else 130

    async def run(self, factory: Callable[[], Awaitable[T]]) -> T:
        """factory() の結果を待つ。中断要求があれば内側のタスクを cancel して Interrupted を投げる。"""
        if self.requested:
            raise Interrupted
        task: asyncio.Task[T] = asyncio.ensure_future(factory())
        self._inner = task  # type: ignore[assignment]
        try:
            while True:
                done, _ = await asyncio.wait({task}, timeout=RECANCEL_INTERVAL_S)
                if done:
                    break
                if self.requested:
                    task.cancel()
        finally:
            self._inner = None
        if task.cancelled():
            raise Interrupted
        exc = task.exception()
        if exc is not None:
            if self.requested:
                # cancel の余波の例外（通信の失敗など）。中断として扱い、後の手順に進まない
                say(f"中断中に手順が例外で終わりました（{type(exc).__name__}: {exc}）")
                raise Interrupted from exc
            raise exc
        if self.requested:
            raise Interrupted
        return task.result()
