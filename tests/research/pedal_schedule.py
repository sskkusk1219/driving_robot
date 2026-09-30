"""ペダル予定表: 基準車速だけから、方式 A（今の 1 点の傾き）と方式 G（窓の傾き）の
ペダル選択（アクセル／惰行／ブレーキ）を並べて比べる（走らない・車両に触らない）。

ProblemReport_20260929 段1。使い方:
    python -m tests.research.pedal_schedule [--mode モード名] [--center-s L] [--width-s H]
        [--point-s P]

- A: 傾き = (ref(t+P) − ref(t)) / P（P は config の pedal_select_point_s。1.0 で今の
  predict_effort と同じ）、惰行加速度は ref(t) での値
- G: 傾き = ref の [t+L−H/2, t+L+H/2] の最小二乗の傾き、惰行加速度は ref(t+L) での値
- 停車（ref(t) と ref(t+停車ホライズン) がともに STOP_SPEED_KMH 以下）は選択の対象外（STOP）
"""

from __future__ import annotations

import argparse
import asyncio
from collections import Counter
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from statistics import median
from typing import Any

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.ff_model import STOP_SPEED_KMH, FeedforwardModel
from tests.research.ff_params import free_accel_at, research_ff_params
from tests.research.live_plot import COLOR_ACCEL, COLOR_BRAKE, COLOR_REF, FONT_FAMILY
from tests.research.mode_drive import ReferenceSpeed, load_mode
from tests.research.pedal_select import (
    PEDAL_ACCEL,
    PEDAL_BRAKE,
    PEDAL_COAST,
    point_accel_kmhs,
    select_pedal,
    window_slope_kmhs,
)
from tests.research.vehicle import feedforward_params

STOP = 2  # 停車（選択の対象外）を表す値。ペダル値（+1/0/−1）と重ならない
STEP_S = 0.05  # 予定表の刻み [s]（制御周期と同じ）
ROW_S = 300.0  # PNG の 1 段（行）の長さ [s]
SHORT_RUN_S = 1.0  # これ未満の区間を「短い区間」として数える
MATCH_WINDOW_S = 2.0  # A と G の切替を「対応する」とみなす最大の時刻差 [s]
COLOR_COAST = "#2e7d32"
COLOR_STOP = "#9e9e9e"
LABELS = {PEDAL_ACCEL: "+1", PEDAL_COAST: "0", PEDAL_BRAKE: "-1", STOP: "停"}
NAMES = {PEDAL_ACCEL: "アクセル", PEDAL_COAST: "惰行", PEDAL_BRAKE: "ブレーキ", STOP: "停車"}
COLORS = {
    PEDAL_ACCEL: COLOR_ACCEL,
    PEDAL_COAST: COLOR_COAST,
    PEDAL_BRAKE: COLOR_BRAKE,
    STOP: COLOR_STOP,
}


@dataclass(frozen=True)
class Run:
    """同じ選択が続いた区間。start_s は先頭の時刻、duration_s は次の区間の頭までの長さ。"""

    value: int
    start_s: float
    duration_s: float


@dataclass(frozen=True)
class Switch:
    """選択の切替（停車をはさまない隣り合う区間の境目）。time_s は切替後の区間の頭。"""

    time_s: float
    before: int
    after: int


@dataclass(frozen=True)
class ScheduleStats:
    """1 つの予定表の集計。"""

    switches: tuple[Switch, ...]
    switch_count: int
    switches_per_s: float  # 回/s（停車を除いた時間で割る）
    short_runs: int  # 1s 未満の区間の数（両端・停車の隣も含む）
    short_patterns: dict[str, int]  # 前→区間→後 の並びごとの数
    direct_accel_brake: int  # アクセル⇔ブレーキの直接切替（惰行をはさまない）
    shares: dict[int, float]  # 各選択の割合（全時間に対して。停車も含む）


def to_runs(sequence: Sequence[int], step_s: float = STEP_S) -> list[Run]:
    """選択の列を、同じ値が続く区間にまとめる。"""
    runs: list[Run] = []
    start = 0
    for i in range(1, len(sequence) + 1):
        if i == len(sequence) or sequence[i] != sequence[start]:
            runs.append(Run(sequence[start], start * step_s, (i - start) * step_s))
            start = i
    return runs


def summarize_schedule(sequence: Sequence[int], step_s: float = STEP_S) -> ScheduleStats:
    """切替回数・1s 未満の区間・アクセル⇔ブレーキ直接切替・割合を集計する（純関数）。"""
    runs = to_runs(sequence, step_s)
    switches = tuple(
        Switch(cur.start_s, prev.value, cur.value)
        for prev, cur in zip(runs, runs[1:], strict=False)
        if STOP not in (prev.value, cur.value)
    )
    active_s = sum(r.duration_s for r in runs if r.value != STOP)
    patterns: Counter[str] = Counter()
    short = 0
    for i, r in enumerate(runs):
        if r.value == STOP or r.duration_s >= SHORT_RUN_S - 1e-9:
            continue
        short += 1
        before = LABELS[runs[i - 1].value] if i > 0 else "端"
        after = LABELS[runs[i + 1].value] if i + 1 < len(runs) else "端"
        patterns[f"{before}→{LABELS[r.value]}→{after}"] += 1
    total = len(sequence)
    counts = Counter(sequence)
    return ScheduleStats(
        switches=switches,
        switch_count=len(switches),
        switches_per_s=len(switches) / active_s if active_s > 0 else 0.0,
        short_runs=short,
        short_patterns=dict(patterns.most_common()),
        direct_accel_brake=sum(
            1 for s in switches if {s.before, s.after} == {PEDAL_ACCEL, PEDAL_BRAKE}
        ),
        shares={v: counts.get(v, 0) / total if total else 0.0 for v in NAMES},
    )


def switch_timing_diffs(
    base: Sequence[Switch], other: Sequence[Switch], window_s: float = MATCH_WINDOW_S
) -> list[float]:
    """base の各切替に、同じ向き（前→後が同じ）で最も近い other の切替を対応づけ、時刻差
    （other − base）[s] を返す。窓 ±window_s 内に無い切替は対応なしとして捨てる。

    other 側の切替は 1 回しか使わない（時刻順に、まだ使っていない中から最も近いものを取る）。
    """
    used: set[int] = set()
    diffs: list[float] = []
    for b in base:
        best: tuple[float, int] | None = None
        for j, o in enumerate(other):
            if j in used or (o.before, o.after) != (b.before, b.after):
                continue
            d = o.time_s - b.time_s
            if abs(d) <= window_s and (best is None or abs(d) < abs(best[0])):
                best = (d, j)
        if best is not None:
            used.add(best[1])
            diffs.append(best[0])
    return diffs


def compute_schedules(
    ref_at: Callable[[float], float],
    duration_s: float,
    coast_at: Callable[[float], float],
    *,
    band_kmhs: float,
    point_s: float,
    stop_horizon_s: float,
    center_s: float,
    width_s: float,
    speed_clip_max: float | None = None,
    step_s: float = STEP_S,
) -> tuple[np.ndarray, list[int], list[int]]:
    """各時刻の A・G の選択を返す（times, A, G）。停車の時刻は両方 STOP。"""
    times = np.round(np.arange(0.0, duration_s + step_s / 2, step_s), 3)
    clip = (lambda v: v) if speed_clip_max is None else (lambda v: min(v, speed_clip_max))
    a_seq: list[int] = []
    g_seq: list[int] = []
    for t in times:
        t = float(t)
        if ref_at(t) <= STOP_SPEED_KMH and ref_at(t + stop_horizon_s) <= STOP_SPEED_KMH:
            a_seq.append(STOP)
            g_seq.append(STOP)
            continue
        a_seq.append(select_pedal(
            point_accel_kmhs(ref_at, t, point_s), coast_at(clip(ref_at(t))), band_kmhs
        ))
        g_seq.append(select_pedal(
            window_slope_kmhs(ref_at, t, center_s, width_s),
            coast_at(clip(ref_at(t + center_s))),
            band_kmhs,
        ))
    return times, a_seq, g_seq


def format_report(
    a: ScheduleStats,
    g: ScheduleStats,
    diffs: Sequence[float],
    *,
    center_s: float,
    width_s: float,
    point_s: float = 1.0,
) -> str:
    """標準出力用の表（Markdown）。"""
    lines = [
        f"| 項目 | A（1 点の傾き point={point_s:g}s） | "
        f"G（窓の傾き L={center_s:g}s H={width_s:g}s） |",
        "|---|---|---|",
        f"| 切替回数 | {a.switch_count} | {g.switch_count} |",
        f"| 切替 回/s（停車を除く） | {a.switches_per_s:.4f} | {g.switches_per_s:.4f} |",
        f"| 1s 未満の区間 | {a.short_runs} | {g.short_runs} |",
        f"| アクセル⇔ブレーキ直接切替 | {a.direct_accel_brake} | {g.direct_accel_brake} |",
    ]
    for v in (PEDAL_ACCEL, PEDAL_COAST, PEDAL_BRAKE, STOP):
        lines.append(f"| 割合 {NAMES[v]} | {a.shares[v]:.1%} | {g.shares[v]:.1%} |")
    lines.append("")
    for label, s in (("A", a), ("G", g)):
        detail = "、".join(f"{k} {n}" for k, n in s.short_patterns.items()) or "なし"
        lines.append(f"- {label} の 1s 未満の区間（前→区間→後。停=停車・端=モードの端）: {detail}")
    lines.append("")
    if diffs:
        lines.append(
            f"- A と G の切替タイミングの差（G − A。同じ向きの切替を ±{MATCH_WINDOW_S:g}s 以内で"
            f"1 対 1 に対応づけ）: 対応 {len(diffs)} 組・中央値 {median(diffs):+.3f}s・"
            f"平均 {float(np.mean(diffs)):+.3f}s・最小 {min(diffs):+.2f}s・最大 {max(diffs):+.2f}s"
        )
    else:
        lines.append("- A と G の切替タイミングの差: 対応する切替なし")
    lines.append(f"- A のみ・G のみの切替（対応なし）: A {a.switch_count - len(diffs)}・"
                 f"G {g.switch_count - len(diffs)}")
    return "\n".join(lines)


def _plt() -> Any:
    import matplotlib  # noqa: PLC0415 - 図を出すときだけ読み込む

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    plt.rcParams["font.family"] = FONT_FAMILY
    return plt


def fig_schedule(
    times: np.ndarray,
    ref_kmh: np.ndarray,
    a_seq: Sequence[int],
    g_seq: Sequence[int],
    path: Path,
    *,
    title: str,
    point_s: float = 1.0,
    step_s: float = STEP_S,
) -> None:
    """A と G を上下に並べ、基準車速の線に選択を背景色で重ねる。ROW_S ごとに段を分ける。"""
    from matplotlib.patches import Patch  # noqa: PLC0415

    plt = _plt()
    n_rows = max(1, int(np.ceil((times[-1] + step_s) / ROW_S)))
    fig, axes = plt.subplots(n_rows * 2, 1, figsize=(16, 2.4 * n_rows * 2), squeeze=False)
    for row in range(n_rows):
        t0, t1 = row * ROW_S, (row + 1) * ROW_S
        mask = (times >= t0) & (times <= t1)
        labeled = ((f"A（1 点の傾き point={point_s:g}s）", a_seq), ("G（窓の傾き）", g_seq))
        for k, (label, seq) in enumerate(labeled):
            ax = axes[row * 2 + k][0]
            for r in to_runs(seq, step_s):
                if r.start_s + r.duration_s < t0 or r.start_s > t1:
                    continue
                ax.axvspan(r.start_s, r.start_s + r.duration_s, color=COLORS[r.value],
                           alpha=0.28, linewidth=0)
            ax.plot(times[mask], ref_kmh[mask], color=COLOR_REF, linewidth=1.2)
            ax.set_xlim(t0, min(t1, float(times[-1]) + step_s))
            ax.set_ylabel(f"{label}\n基準車速 [km/h]", fontsize=8)
            ax.grid(alpha=0.3)
    axes[-1][0].set_xlabel("モード経過時間 [s]")
    axes[0][0].legend(
        handles=[Patch(color=COLORS[v], alpha=0.5, label=NAMES[v]) for v in NAMES],
        loc="upper right", ncol=4, fontsize=9,
    )
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--mode", default=None, help="走行モード名（既定: modes.wltp_mode_name）")
    ap.add_argument("--center-s", type=float, default=None, help="L を一時的に上書き [s]")
    ap.add_argument("--width-s", type=float, default=None, help="H を一時的に上書き [s]")
    ap.add_argument("--point-s", type=float, default=None,
                    help="A（point）の先読みを一時的に上書き [s]")
    ap.add_argument("--out", type=Path, default=None,
                    help="PNG の保存先（既定: results/report<今日>_pedalSchedule/）")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    p = feedforward_params(cfg)
    research = research_ff_params(cfg)
    center_s = args.center_s if args.center_s is not None else research.pedal_select_center_s
    width_s = args.width_s if args.width_s is not None else research.pedal_select_width_s
    point_s = args.point_s if args.point_s is not None else research.pedal_select_point_s
    band = research.coast_band_kmhs

    # 停車ホライズン・学習域クリップは pkl から読む（predict_effort と同じ）
    ff = FeedforwardModel()
    ff.load_model(cfg.feedforward.model_path)
    stop_horizon_s = ff.stop_horizon_s
    clip = ff._speed_clip_max  # noqa: SLF001 - 公開プロパティが無い

    mode_name = args.mode or cfg.modes.wltp_mode_name
    mode = asyncio.run(load_mode(cfg, mode_name))
    ref = ReferenceSpeed(mode)
    print(f"# ペダル予定表: {mode.name}"
          f"（{mode.total_duration:.0f}s・最高 {mode.max_speed:.1f} km/h）")
    print(f"- L={center_s:g}s・H={width_s:g}s・帯 ±{band:g} km/h/s・point={point_s:g}s・"
          f"停車ホライズン {stop_horizon_s:g}s・刻み {STEP_S:g}s・学習域クリップ {clip}")

    times, a_seq, g_seq = compute_schedules(
        ref.at, mode.total_duration, lambda v: free_accel_at(p, research, v),
        band_kmhs=band, point_s=point_s, stop_horizon_s=stop_horizon_s,
        center_s=center_s, width_s=width_s, speed_clip_max=clip,
    )
    a, g = summarize_schedule(a_seq), summarize_schedule(g_seq)
    diffs = switch_timing_diffs(a.switches, g.switches)
    print()
    print(format_report(a, g, diffs, center_s=center_s, width_s=width_s, point_s=point_s))

    out_dir = args.out or cfg.results_path / f"report{datetime.now():%Y%m%d}_pedalSchedule"
    out_dir.mkdir(parents=True, exist_ok=True)
    png = out_dir / "pedal_schedule.png"
    fig_schedule(
        times, np.array([ref.at(float(t)) for t in times]), a_seq, g_seq, png,
        title=(f"ペダル予定表 {mode.name}（point={point_s:g}s L={center_s:g}s "
               f"H={width_s:g}s 帯±{band:g}）"),
        point_s=point_s,
    )
    print()
    print(f"- 図: {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
