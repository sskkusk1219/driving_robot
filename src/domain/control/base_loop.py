"""制御・学習・スケジュールの3ループに共通するサイクル実行基盤。

DriveLoop・LearningLoop・ScheduleLoop はいずれも「固定周期でサイクルを実行し、
前サイクル未完了が続いたらウォッチドッグで非常停止する」という同じループ機構を
コピペで共有していた。修正（stop_and_join の self-join デッドロック回避、
サイクルストール計測等）が1ループにしか適用されない事故を防ぐため、
共通のライフサイクル・ウォッチドッグ・ログ書込バックログ管理を本モジュールへ
一本化する（S1 レビュー指摘）。

サブクラスは `_execute_one_cycle()` を実装し、開始時に固有状態のリセットが
必要なら `_reset_for_start()` をオーバーライドする。
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Awaitable, Callable
from typing import Protocol

from src.models.drive_log import DriveLogData
from src.models.system_state import RealtimeSnapshot

_logger = logging.getLogger(__name__)

# サイクルウォッチドッグ: 前サイクル未完了によるスキップがこの時間続いたら非常停止する。
# pymodbus のタイムアウト×リトライ（最大十数秒）を待つ間、ペダルが最終指令位置で
# 凍結したまま安全チェックが走らない時間を 1 秒に短縮する。
WEDGED_CYCLE_TIMEOUT_S: float = 1.0
# ログ書き込み保留タスクの上限。DB ストール時に無制限に積み上がるのを防ぎ、
# 長時間走行のメモリとイベントループ負荷を一定に保つ（走行継続をログより優先）。
MAX_PENDING_LOG_TASKS: int = 100


class LogWriterProtocol(Protocol):
    async def write_log(self, session_id: str, data: DriveLogData) -> None: ...


def _log_emergency_error_callback(task: asyncio.Task[None]) -> None:
    """非常停止コールバックタスクの例外をログに記録する（黙殺防止）。"""
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        _logger.error("非常停止コールバックが失敗しました", exc_info=exc)


class CycleLoopBase:
    """固定周期でサイクルを実行するループの共通基盤。

    start() でループを開始し、stop() または on_complete/on_emergency コールバックで
    停止する。asyncio.sleep を使わず、開始時刻を基準にした絶対時刻グリッド（call_at）で
    サイクルをスケジューリングすることで、相対 call_later のドリフトを避けつつジッタを抑制する。
    """

    # ログメッセージに使うループ種別名。「制御サイクル」「学習サイクル」等、
    # 「サイクル」を含む名詞としてサブクラスでオーバーライドする。
    _cycle_label: str = "サイクル"

    def __init__(
        self,
        interval_s: float,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        log_writer: LogWriterProtocol | None = None,
        session_id: str | None = None,
    ) -> None:
        self._interval_s = interval_s
        self._on_complete = on_complete
        self._on_emergency = on_emergency
        self._log_writer = log_writer
        self._session_id = session_id

        self._running = False
        # ウォッチドッグ: 前サイクル未完了による連続スキップ数
        self._consecutive_skips = 0
        # ストール計測（.steering/20260620-modbus-retry-cycle-stall）: 連続スキップが
        # 解消するたびに 1 件のストールとして回数・累積時間・最大継続時間を集計する。
        self._stall_count = 0
        self._stall_total_s = 0.0
        self._stall_max_s = 0.0
        self._log_backlog_active = False
        self._current_accel_opening: float = 0.0
        self._current_brake_opening: float = 0.0
        # イベントループはタスクを弱参照でしか保持しないため、GC でサイクルタスクが
        # 実行途中に破棄されないよう強参照を保持する。
        self._cycle_task: asyncio.Task[None] | None = None
        self._emergency_task: asyncio.Task[None] | None = None
        self._pending_log_tasks: set[asyncio.Task[None]] = set()
        # WS 配信用のサイクル計測値キャッシュ（走行中の追加 Modbus/CAN 読み取りを回避）。
        self._last_snapshot: RealtimeSnapshot | None = None
        # 絶対時刻スケジューリング: サイクル tick n を `_grid_anchor + n×interval` に予約し、
        # 相対 call_later のコールバック起動遅延の累積（ドリフト）を除去する。start() で確定。
        self._grid_anchor: float = 0.0
        self._grid_tick: int = 0

    def _reset_for_start(self) -> None:
        """start() から呼ばれるサブクラス固有の状態リセット。既定は no-op。"""

    def start(self) -> None:
        """ループを開始する。既に実行中の場合は何もしない。"""
        if self._running:
            return
        self._running = True
        self._consecutive_skips = 0
        self._stall_count = 0
        self._stall_total_s = 0.0
        self._stall_max_s = 0.0
        self._reset_for_start()
        loop = asyncio.get_running_loop()
        # サイクルグリッドを開始時刻に固定する（以降 tick n は _grid_anchor + n×interval）。
        self._grid_anchor = loop.time()
        self._grid_tick = 0
        self._arm_next_cycle(loop)

    def stop(self) -> None:
        """ループを停止する。進行中サイクルは書き込み前に中断される。"""
        self._running = False

    async def stop_and_join(self, timeout_s: float = 2.0) -> None:
        """停止し、進行中のサイクルタスク完了を待つ。home_return 前に必ず呼ぶ。

        飛行中の位置指令（move_to_position の await 中）と原点復帰シーケンスが
        同一軸のバスでインターリーブし、非常停止中にペダル指令が再適用されるのを防ぐ
        （コードレビュー 2026-06-11 指摘 #6）。

        タイムアウト時はタスクをキャンセルする（バスが完全に固まっている場合、
        非常停止をこれ以上遅延させない）。

        on_complete はサイクルタスク内から await されるため、その経路から本メソッドが
        呼ばれると自タスクを join しようとして wait_for がタイムアウト→自タスクを
        cancel し、呼び出し元の後続 await（home_return）が CancelledError で中断される。
        `current_task() is サイクルタスク` のときは既にサイクルが終了処理中のため join
        不要として即 return する（正常完了でのハング＋原点未復帰を防ぐ）。
        """
        self.stop()
        task = self._cycle_task
        if task is None or task.done() or task is asyncio.current_task():
            return
        try:
            # shield: タイムアウトしてもサイクルタスク自体は cancel せず明示的に扱う
            await asyncio.wait_for(asyncio.shield(task), timeout_s)
        except TimeoutError:
            _logger.warning(
                "進行中の%sが %.1fs 以内に完了せずキャンセルします",
                self._cycle_label,
                timeout_s,
            )
            task.cancel()
        except Exception:
            # サイクル内の例外は _on_cycle_done が回収・処理済み（ここでは無視してよい）
            pass

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def current_accel_opening(self) -> float:
        return self._current_accel_opening

    @property
    def current_brake_opening(self) -> float:
        return self._current_brake_opening

    @property
    def last_snapshot(self) -> RealtimeSnapshot | None:
        """直近サイクルの計測値。WS 配信が走行中の追加バス読み取りを避けるために使う。"""
        return self._last_snapshot

    @property
    def stall_summary(self) -> dict[str, float]:
        """走行中〜終了時点のサイクルストール集計（回数・累積時間・最大継続時間）。

        ストール＝前サイクル未完了（バス再送等）により後続サイクルが連続スキップされた
        1 エピソード。ウォッチドッグ（非常停止）に至らず解消したものも含む。
        """
        return {
            "stall_count": float(self._stall_count),
            "stall_total_s": self._stall_total_s,
            "stall_max_s": self._stall_max_s,
        }

    def _arm_next_cycle(self, loop: asyncio.AbstractEventLoop) -> None:
        """次サイクルを絶対時刻グリッド（_grid_anchor + tick×interval）上に予約する。

        相対 call_later はコールバック起動遅延が毎回累積し固定周期からドリフトする
        （実測 +0.07ms/サイクル → 323s 走行で約 10 サイクル早く終端到達＝ログ数行欠落）。
        グリッドを絶対時刻に固定してこのドリフトを除去する。

        遅延発火でグリッド点を跨いで過去になった場合は、次の未来グリッドまで tick を進めて
        catch-up バースト発火（過去時刻の call_at を連続発火）を避ける。丸めても 1 tick=interval
        の刻みは保たれるため、スキップ計数・stall_summary の集計単位は現行と互換になる。
        """
        self._grid_tick += 1
        now = loop.time()
        target = self._grid_anchor + self._grid_tick * self._interval_s
        if target <= now:
            # 発火が遅れてグリッドを跨いだ: now より未来の最小グリッドまで tick を丸める
            self._grid_tick = int((now - self._grid_anchor) // self._interval_s) + 1
            target = self._grid_anchor + self._grid_tick * self._interval_s
        loop.call_at(target, self._schedule_next_cycle)

    def _schedule_next_cycle(self) -> None:
        if not self._running:
            return
        loop = asyncio.get_running_loop()
        # 重複実行ガード: 前サイクルが未完了（バス遅延等）の場合は新サイクルを起動しない。
        # 並行サイクルは古い位置指令が新しい指令を上書きする危険があるためスキップする。
        if self._cycle_task is not None and not self._cycle_task.done():
            self._consecutive_skips += 1
            wedged_s = self._consecutive_skips * self._interval_s
            _logger.warning(
                "前回の%sが %.0fms 以内に完了せずスキップ",
                self._cycle_label,
                self._interval_s * 1000,
            )
            # ウォッチドッグ: スキップが続く間は安全チェックが一切走らずペダルが
            # 最終指令位置で保持され続けるため、上限で非常停止に倒す。
            if wedged_s >= WEDGED_CYCLE_TIMEOUT_S:
                _logger.error(
                    "%sが %.1fs 以上完了していません: ウォッチドッグ非常停止します",
                    self._cycle_label,
                    wedged_s,
                )
                self.stop()
                self._emergency_task = asyncio.ensure_future(self._on_emergency())
                self._emergency_task.add_done_callback(_log_emergency_error_callback)
                return
        else:
            if self._consecutive_skips > 0:
                stall_duration_s = self._consecutive_skips * self._interval_s
                self._stall_count += 1
                self._stall_total_s += stall_duration_s
                self._stall_max_s = max(self._stall_max_s, stall_duration_s)
                _logger.warning(
                    "%sストール解消: 継続時間=%.2fs (累計 %d 回 / %.2fs)",
                    self._cycle_label,
                    stall_duration_s,
                    self._stall_count,
                    self._stall_total_s,
                )
            self._consecutive_skips = 0
            self._cycle_task = asyncio.ensure_future(self._execute_one_cycle())
            self._cycle_task.add_done_callback(self._on_cycle_done)
        self._arm_next_cycle(loop)

    def _on_cycle_done(self, task: asyncio.Task[None]) -> None:
        """サイクルタスクの未捕捉例外を回収する。黙殺すると安全チェック抜けが見えなくなる。"""
        if task.cancelled():
            return
        exc = task.exception()
        if exc is None:
            return
        _logger.error("%sで未捕捉例外: 非常停止します", self._cycle_label, exc_info=exc)
        self.stop()
        self._emergency_task = asyncio.ensure_future(self._on_emergency())
        self._emergency_task.add_done_callback(_log_emergency_error_callback)

    async def _abort_emergency(self) -> None:
        """ループを停止して非常停止コールバックを呼ぶ。サイクル内の異常検知から使う。"""
        self.stop()
        await self._on_emergency()

    async def _execute_one_cycle(self) -> None:
        """1 サイクル分の処理。サブクラスが実装する。"""
        raise NotImplementedError

    def _enqueue_log_write(self, data: DriveLogData) -> None:
        """ログ書き込みタスクを上限付きで起動する。滞留時はログを捨てて走行を優先する。"""
        if self._log_writer is None or self._session_id is None:
            return
        if len(self._pending_log_tasks) >= MAX_PENDING_LOG_TASKS:
            if not self._log_backlog_active:
                self._log_backlog_active = True
                _logger.warning(
                    "%sのログ書き込みが %d 件滞留: 解消まで記録をスキップします (DB 遅延の疑い)",
                    self._cycle_label,
                    MAX_PENDING_LOG_TASKS,
                )
            return
        if self._log_backlog_active:
            self._log_backlog_active = False
            _logger.info("%sのログ書き込みの滞留が解消しました", self._cycle_label)
        task = asyncio.ensure_future(self._log_writer.write_log(self._session_id, data))
        # GC によるタスク消失防止のため完了まで強参照を保持する
        self._pending_log_tasks.add(task)
        task.add_done_callback(self._on_log_write_done)

    def _on_log_write_done(self, task: asyncio.Task[None]) -> None:
        """ログ書き込みタスクの参照を解放し、例外をログに記録する。走行は継続する。"""
        self._pending_log_tasks.discard(task)
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            _logger.error("%sのログ書き込みエラー (走行継続)", self._cycle_label, exc_info=exc)
