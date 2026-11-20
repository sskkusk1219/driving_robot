# ruff: noqa: E501
"""エラーのできごとの記録係（RPD_Phase_20261120_ErrorLog §3 の規則）。

``JobManager``・サーバーのログのハンドラ・ブラウザからの POST が ``record()`` などへ渡し、
書き込みは asyncio のキュー越しに 1 本のワーカーが行う。

**安全の約束**: 記録は制御を止めない・遅らせない。呼び出しは待たない（キューへ入れるだけ。満杯なら捨てる）。
書き込みに失敗しても例外は出さず、ログに 1 回だけ出す。このモジュールと ``error_repository`` のログは
サーバーログのハンドラ（``ErrorLogHandler``）が拾わない（無限ループ防止）。

規則（ここで決めるもの）:
- ``dedupe`` つきのできごと（待機中・サーバー・ブラウザ）は、同じ（きっかけ・水準・題名）が 60 秒以内に続いたら
  新しい件を作らず、回数を足す（最後の日時も更新。確認済みは外す）
- 非常停止の件（``source=emergency``）は ``emergency_id`` に覚える。そのときのジョブが終わったら
  ``append_emergency`` で終了処理の結果を本文へ書き足す（ジョブの件は別に作らない）
"""

from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from typing import Any, Protocol

from src.models.error_event import SOURCE_EMERGENCY, NewErrorEvent

logger = logging.getLogger(__name__)

#: 同じ文をまとめる時間 [s]
DEDUPE_WINDOW_S = 60.0
#: キューの上限（超えたら捨てる。制御を止めないため待たない）
QUEUE_MAX = 1000
#: 止めるとき、溜まった分を書き切るまで待つ時間 [s]
STOP_DRAIN_TIMEOUT_S = 3.0

class ErrorRepoProtocol(Protocol):
    async def insert(self, ev: NewErrorEvent, *, at: datetime) -> int: ...
    async def bump(self, event_id: int, *, at: datetime) -> bool: ...
    async def append_detail(
        self, event_id: int, text: str, *, at: datetime, context_patch: dict[str, Any] | None = None
    ) -> bool: ...


class NullErrorRecorder:
    """何も記録しない記録係（JobManager の既定。テスト・記録不要の場面）。"""

    def record(self, ev: NewErrorEvent) -> None:
        return None

    def append_emergency(
        self,
        text: str,
        *,
        close: bool = False,
        fallback: NewErrorEvent | None = None,
        context_patch: dict[str, Any] | None = None,
    ) -> None:
        return None


@dataclass
class _Op:
    kind: str  # "event" / "append"
    event: NewErrorEvent | None = None
    text: str = ""
    close: bool = False
    context_patch: dict[str, Any] | None = None
    at: datetime | None = None


class ErrorRecorder:
    """キュー越しにできごとを書く記録係。``start()`` でワーカーを起動し、``stop()`` で書き切って止める。"""

    def __init__(
        self,
        repo: ErrorRepoProtocol,
        *,
        dedupe_window_s: float = DEDUPE_WINDOW_S,
        clock: Callable[[], float] = time.monotonic,
        queue_max: int = QUEUE_MAX,
    ) -> None:
        self._repo = repo
        self._window = dedupe_window_s
        self._clock = clock
        self._queue_max = queue_max
        self._loop: asyncio.AbstractEventLoop | None = None
        self._queue: asyncio.Queue[_Op] | None = None
        self._task: asyncio.Task[None] | None = None
        self._recent: dict[tuple[str, str, str], tuple[int, float]] = {}
        self.emergency_id: int | None = None  # 書き足し先の非常停止の件
        self._failure_logged = False
        self._drop_logged = False

    # ── 起動・停止 ─────────────────────────────────────────

    def start(self) -> None:
        """ワーカーを起動する（実行中のイベントループの中で呼ぶ）。"""
        if self._task is not None:
            return
        self._loop = asyncio.get_running_loop()
        self._queue = asyncio.Queue()
        self._task = asyncio.create_task(self._worker(), name="error-recorder")

    async def stop(self) -> None:
        """溜まった分を（短い時間だけ）書き切ってから止める。"""
        task, queue = self._task, self._queue
        if task is None or queue is None:
            return
        try:
            await asyncio.wait_for(queue.join(), STOP_DRAIN_TIMEOUT_S)
        except (TimeoutError, asyncio.CancelledError):
            pass
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        self._task = None
        self._loop = None
        self._queue = None

    async def flush(self) -> None:
        """今キューにある分を書き切るまで待つ（テスト・終了処理用。制御からは呼ばない）。"""
        queue = self._queue
        if queue is None:
            return
        await asyncio.sleep(0)  # call_soon_threadsafe で渡された分がキューに入るのを待つ
        await queue.join()

    # ── 呼び出し側（待たない・例外を出さない・どのスレッドからでも） ─────

    def record(self, ev: NewErrorEvent) -> None:
        if ev.occurred_at is None:
            ev.occurred_at = datetime.now(tz=UTC)
        self._submit(_Op(kind="event", event=ev, at=ev.occurred_at))

    def append_emergency(
        self,
        text: str,
        *,
        close: bool = False,
        fallback: NewErrorEvent | None = None,
        context_patch: dict[str, Any] | None = None,
    ) -> None:
        """非常停止の件の本文へ書き足す。``close`` なら以後の書き足し先を外す（終了処理の結果）。
        件が無い（記録に失敗した等）とき ``fallback`` があればそれを普通の件として記録する。"""
        self._submit(
            _Op(kind="append", event=fallback, text=text, close=close, context_patch=context_patch,
                at=datetime.now(tz=UTC))
        )

    def _submit(self, op: _Op) -> None:
        loop = self._loop
        if loop is None:
            return
        try:
            loop.call_soon_threadsafe(self._enqueue, op)
        except RuntimeError:  # ループが閉じている（終了中）
            return

    def _enqueue(self, op: _Op) -> None:
        queue = self._queue
        if queue is None:
            return
        if queue.qsize() >= self._queue_max:
            if not self._drop_logged:
                self._drop_logged = True
                logger.warning("エラーの記録が溜まりすぎているため、以後の記録を捨てます")
            return
        queue.put_nowait(op)

    # ── ワーカー ───────────────────────────────────────────

    async def _worker(self) -> None:
        assert self._queue is not None  # noqa: S101
        while True:
            op = await self._queue.get()
            try:
                await self.process(op)
            except Exception as exc:  # noqa: BLE001 - 記録の失敗で何も止めない
                if not self._failure_logged:
                    self._failure_logged = True
                    logger.warning("エラーの記録を書けません（以後は黙ります）: %s: %s", type(exc).__name__, exc)
            finally:
                self._queue.task_done()

    async def process(self, op: _Op) -> None:
        """1 件の処理（テストからも直接呼べる）。"""
        at = op.at or datetime.now(tz=UTC)
        if op.kind == "append":
            await self._process_append(op, at)
            return
        ev = op.event
        assert ev is not None  # noqa: S101
        await self._process_event(ev, at)

    async def _process_event(self, ev: NewErrorEvent, at: datetime) -> None:
        if ev.dedupe:
            key = (ev.source, ev.level, ev.title)
            now = self._clock()
            prev = self._recent.get(key)
            if prev is not None and now - prev[1] <= self._window:
                if await self._repo.bump(prev[0], at=at):
                    self._recent[key] = (prev[0], now)
                    return
            # 窓の外・件が消された: 新しい件を作る。古い印は捨てる
            self._prune(now)
            event_id = await self._repo.insert(ev, at=at)
            self._recent[key] = (event_id, now)
            return
        event_id = await self._repo.insert(ev, at=at)
        if ev.source == SOURCE_EMERGENCY:
            self.emergency_id = event_id

    async def _process_append(self, op: _Op, at: datetime) -> None:
        event_id = self.emergency_id
        done = False
        if event_id is not None:
            done = await self._repo.append_detail(event_id, op.text, at=at, context_patch=op.context_patch)
        if op.close:
            self.emergency_id = None
        if not done and op.event is not None:
            await self._repo.insert(op.event, at=at)

    def _prune(self, now: float) -> None:
        for key in [k for k, (_, t) in self._recent.items() if now - t > self._window]:
            del self._recent[key]
