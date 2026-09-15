"""走行グラフ（matplotlib）。

本番の自動走行画面（src/web/static/js/screens/auto-drive.js）と同じ 2 段・同じ色で描く。

    1 段目: 車速 [km/h]  基準車速（灰・破線。基準がある走行のみ）/ 実車速（橙）
    2 段目: 開度 [%]     アクセル（水色）/ ブレーキ（赤）

走行中の描画は**別プロセス**で行う。制御ループ（100ms 周期）と同じプロセスで再描画すると
1 回数百 ms かかってサイクルが詰まり、ループのウォッチドッグ（1s）で非常停止しかねないため。
DISPLAY があればウィンドウに、無ければ（SSH 接続など）PNG を数秒ごとに上書きする。
"""

from __future__ import annotations

import asyncio
import multiprocessing as mp
import os
import queue
import time
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from tests.research.term import say

COLOR_REF = "#8a8a8a"
COLOR_ACTUAL = "#c8922a"
COLOR_ACCEL = "#78c8f0"
COLOR_BRAKE = "#f07070"
FONT_FAMILY = ["BIZ UDGothic", "Droid Sans Fallback", "DejaVu Sans"]
FIGSIZE = (12.0, 7.0)
LIVE_REDRAW_INTERVAL_S = 2.0
_QUEUE_MAX = 20_000


@dataclass(frozen=True)
class PlotSample:
    t_s: float
    ref_kmh: float | None
    actual_kmh: float
    accel_pct: float
    brake_pct: float


def _build_axes(fig: Any, title: str, max_speed_kmh: float, has_ref: bool) -> dict[str, Any]:
    ax_speed, ax_open = fig.subplots(2, 1, sharex=True)
    fig.suptitle(title)
    ref = None
    if has_ref:
        (ref,) = ax_speed.plot([], [], color=COLOR_REF, linestyle="--", linewidth=1.5,
                               label="基準車速")
    (actual,) = ax_speed.plot([], [], color=COLOR_ACTUAL, linewidth=2.0, label="実車速")
    ax_speed.set_ylabel("車速 [km/h]")
    ax_speed.set_ylim(0.0, max_speed_kmh * 1.05)
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend(loc="upper right")

    (accel,) = ax_open.plot([], [], color=COLOR_ACCEL, linewidth=1.8, label="アクセル")
    (brake,) = ax_open.plot([], [], color=COLOR_BRAKE, linewidth=1.8, label="ブレーキ")
    ax_open.set_ylabel("開度 [%]")
    ax_open.set_xlabel("経過時間 [s]")
    ax_open.set_ylim(0.0, 100.0)
    ax_open.grid(True, alpha=0.3)
    ax_open.legend(loc="upper right")
    return {"ax_open": ax_open, "ref": ref, "actual": actual, "accel": accel, "brake": brake}


def _update(parts: dict[str, Any], samples: Sequence[PlotSample]) -> None:
    t = [s.t_s for s in samples]
    parts["actual"].set_data(t, [s.actual_kmh for s in samples])
    if parts["ref"] is not None:
        with_ref = [s for s in samples if s.ref_kmh is not None]
        parts["ref"].set_data([s.t_s for s in with_ref], [s.ref_kmh for s in with_ref])
    parts["accel"].set_data(t, [s.accel_pct for s in samples])
    parts["brake"].set_data(t, [s.brake_pct for s in samples])
    parts["ax_open"].set_xlim(0.0, max(t[-1], 1.0) if t else 1.0)


def save_drive_figure(
    samples: Sequence[PlotSample],
    path: Path,
    *,
    title: str,
    max_speed_kmh: float,
    has_ref: bool,
) -> Path:
    """走行全体の図を PNG に保存する（走行後に呼ぶ）。

    pyplot を使わないので、呼び出し側プロセスの描画状態を汚さない。
    """
    import matplotlib  # noqa: PLC0415
    from matplotlib.figure import Figure  # noqa: PLC0415

    with matplotlib.rc_context({"font.family": FONT_FAMILY}):
        fig = Figure(figsize=FIGSIZE, dpi=100, layout="constrained")
        parts = _build_axes(fig, title, max_speed_kmh, has_ref)
        _update(parts, samples)
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path)
    return path


def _drain(q: Any, samples: list[PlotSample]) -> bool:
    """キューを空にしてサンプルに積む。終了合図（None）を受け取ったら True。"""
    try:
        item = q.get(timeout=0.1)
    except queue.Empty:
        return False
    while True:
        if item is None:
            return True
        samples.append(item)
        try:
            item = q.get_nowait()
        except queue.Empty:
            return False


def _live_worker(
    q: Any, png_path: str, title: str, max_speed_kmh: float, has_ref: bool, interactive: bool
) -> None:
    """描画プロセスの本体。数秒ごとにウィンドウ（または PNG）を更新する。"""
    import matplotlib  # noqa: PLC0415

    matplotlib.use("TkAgg" if interactive else "Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    matplotlib.rcParams["font.family"] = FONT_FAMILY
    fig = plt.figure(figsize=FIGSIZE, layout="constrained")
    parts = _build_axes(fig, title, max_speed_kmh, has_ref)
    if interactive:
        plt.show(block=False)

    samples: list[PlotSample] = []
    last_draw = 0.0
    finished = False
    while not finished:
        finished = _drain(q, samples)
        now = time.monotonic()
        if finished or now - last_draw >= LIVE_REDRAW_INTERVAL_S:
            _update(parts, samples)
            if interactive:
                fig.canvas.draw_idle()
            else:
                fig.savefig(png_path)
            last_draw = now
        if interactive:
            fig.canvas.flush_events()
    plt.close(fig)


class LivePlot:
    """走行中グラフの別プロセス。`push` は非ブロッキングで、詰まったら間引く。"""

    def __init__(
        self,
        png_path: Path,
        *,
        title: str,
        max_speed_kmh: float,
        has_ref: bool,
        enabled: bool,
    ) -> None:
        self.png_path = png_path
        self.title = title
        self.max_speed_kmh = max_speed_kmh
        self.has_ref = has_ref
        self.enabled = enabled
        self._queue: Any = None
        self._process: Any = None

    @property
    def interactive(self) -> bool:
        return bool(os.environ.get("DISPLAY"))

    def start(self) -> None:
        if not self.enabled:
            return
        # fork はシリアル/CAN を掴んだプロセスの複製になるため spawn を使う
        ctx = mp.get_context("spawn")
        self._queue = ctx.Queue(maxsize=_QUEUE_MAX)
        self._process = ctx.Process(
            target=_live_worker,
            args=(self._queue, str(self.png_path), self.title, self.max_speed_kmh,
                  self.has_ref, self.interactive),
            name="research-live-plot",
            daemon=True,
        )
        self._process.start()
        if self.interactive:
            say("走行グラフ: ウィンドウに表示します")
        else:
            say(f"走行グラフ: {self.png_path}（{LIVE_REDRAW_INTERVAL_S:g}s ごとに更新）")

    def push(self, sample: PlotSample) -> None:
        if self._queue is None:
            return
        try:
            self._queue.put_nowait(sample)
        except queue.Full:
            pass  # 描画が遅れているだけ。制御側は待たない

    async def aclose(self, timeout_s: float = 3.0) -> None:
        if self._process is None:
            return
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        await asyncio.to_thread(self._process.join, timeout_s)
        if self._process.is_alive():
            self._process.terminate()
        # 描画プロセスが先に落ちていてもキューの送信スレッドで終了が固まらないように
        self._queue.cancel_join_thread()
        self._queue = None
        self._process = None
