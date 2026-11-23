"""手動運転のスイッチ（スタート・P・N・D）を「押している間だけ押す」ための押し続け管理。

core の ``press_button`` に無限の秒数を渡して押し、離すときにその押下タスクを取り消す。
取り消されるとドライバの finally（shield 付き）が基準の角度へ戻すので、ドライバ・スタブ車両の
状態（VehicleStatus）・ボタンなし設定（差し替え 8）・button イベント（差し替え 9）をそのまま使える。
``src/robot/core`` は写しなので、ここ（src/robot 直下）に置く。

ルール（2026-11-23 ユーザー決定）:
- 押しているボタンは最大 1 つ。先に押した方が優先で、別のボタンの down は NG。
- 先のボタンを離しても、後から押していた側は押されない（押し直しが要る）。
- 押している間は呼び出し側が ``down`` を送り直す（約 0.5 s ごと）。最後の down から
  ``KEEPALIVE_TIMEOUT_S`` 経っても来なければ離す（Web 側が落ちた・通信が切れたときの安全網）。
"""

from __future__ import annotations

import asyncio
import contextlib
import math
import time
from typing import Any

from src.robot.core import button_steps as core_button_steps
from src.robot.core.button_servo import button_name
from src.robot.core.term import say

#: keepalive（送り直し）の down への ack の文の末尾。進行のメッセージに溜めない目印
KEEPALIVE_SUFFIX = "」を押し続けています"

#: 見張りの時間切れ [s]。送り直しは 0.5 s ごと → 2 回取りこぼしても離さない幅（仮の値）
KEEPALIVE_TIMEOUT_S = 1.5
#: 見張りが時刻を見る間隔 [s]
_WATCH_INTERVAL_S = 0.1


class ButtonHold:
    """押しているスイッチ（最大 1 つ）の管理。"""

    def __init__(self, hw: Any, cfg: Any, *, timeout_s: float = KEEPALIVE_TIMEOUT_S) -> None:
        self._hw = hw
        self._cfg = cfg
        self._timeout_s = timeout_s
        self._button: str | None = None
        self._task: asyncio.Task[None] | None = None
        self._watch: asyncio.Task[None] | None = None
        self._last_down = 0.0
        self._lock = asyncio.Lock()

    @property
    def pressed(self) -> str | None:
        """今押しているボタン名（無ければ None）。押す処理が終わっていたら None。"""
        if self._task is not None and self._task.done():
            self._clear()
        return self._button

    def _clear(self) -> None:
        task = self._task
        watch = self._watch
        self._button = None
        self._task = None
        self._watch = None
        if watch is not None and watch is not asyncio.current_task():
            watch.cancel()
        if task is not None and task.done() and not task.cancelled():
            exc = task.exception()
            if exc is not None:
                say(f"  警告: スイッチ押下が異常で終わりました（{type(exc).__name__}: {exc}）")

    async def down(self, button: str) -> tuple[bool, str]:
        """押す（または押し続けの送り直し）。(ok, message)。"""
        async with self._lock:
            now = time.monotonic()
            current = self.pressed
            if current is not None:
                if current == button:
                    self._last_down = now
                    return True, f"「{button_name(button)}{KEEPALIVE_SUFFIX}"
                return False, f"スイッチ「{button_name(current)}」を押しています"
            self._button = button
            self._last_down = now
            self._task = asyncio.create_task(
                core_button_steps.press_button(
                    self._hw, self._cfg, button, math.inf, wait_after=False
                )
            )
            self._watch = asyncio.create_task(self._watchdog(button))
            # 押す処理が最初のポイントまで進むのを 1 回譲って待つ（接続失敗などをすぐ返すため）
            await asyncio.sleep(0)
            return True, f"「{button_name(button)}」を押します"

    async def up(self, button: str) -> tuple[bool, str]:
        """離す。そのボタンを押していなければ何もしない（OK）。"""
        async with self._lock:
            if self.pressed != button:
                return True, "押していません"
            await self._release()
            return True, f"「{button_name(button)}」を離しました"

    async def release_all(self) -> None:
        """押していれば離す（対話の終わり・中断の finally で呼ぶ）。"""
        async with self._lock:
            if self.pressed is not None:
                await self._release()

    async def _release(self) -> None:
        button = self._button
        task = self._task
        watch = self._watch
        self._watch = None
        if watch is not None and watch is not asyncio.current_task():
            watch.cancel()
        if task is not None and not task.done():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task  # ドライバの finally が基準の角度へ戻すまで待つ
        say(f"  スイッチ「{button_name(button or '')}」を離しました")
        self._clear()

    async def _watchdog(self, button: str) -> None:
        while True:
            await asyncio.sleep(_WATCH_INTERVAL_S)
            if time.monotonic() - self._last_down <= self._timeout_s:
                continue
            say(
                f"  警告: スイッチ「{button_name(button)}」の「まだ押している」が "
                f"{self._timeout_s:g} 秒届かないので離します"
            )
            # 自分自身（_watch）は _release で取り消さない。ロックは up/down と共有
            async with self._lock:
                if self._button == button:
                    await self._release()
            return
