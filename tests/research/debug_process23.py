"""手順 2・3 デバッグのまとめ（report20260913_debag_process2,3.md）の表と図を CSV から作る。

車両・アクチュエータには触らない。9/11〜9/13 の走行ログ CSV を読み、表（Markdown）を
ターミナルに出し、図を --out に保存する。

    .venv/bin/python -m tests.research.debug_process23

解析すること（レポートの章と対応）:
    runs      4 本の手順 3 の KPI と補助指標（完全停止回数・ガバナー作動時間など）
    switch    アクセル → ブレーキ（とその逆）に切り替えた瞬間の開度の跳びと、最大減速・最低車速
    onset     ブレーキの立ち上がり方（手順 2・9/11 C0 はなだらか、9/13 C1 は 1 周期）
    decel     手順 2 のパターン走行での「開度 × 車速帯」ごとの減速度（手順 3 の一気踏みと比べる）
    governor  Gガバナーの作動区間と、下限（0% / 待機位置）に張り付いた秒数
    dropout   ブレーキ電流 0 mA が 1s 以上続き始めた時刻と、その直前の開度
    runaway   終盤の 140 km/h 超え直前（アクセル一定なのに加速が増える）
"""

from __future__ import annotations

import argparse
import csv
import pickle
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np

from tests.research.config import load_config
from tests.research.drive_log import cmd_opening, ff_effort
from tests.research.kpi import compute_kpi
from tests.research.live_plot import COLOR_ACCEL, COLOR_ACTUAL, COLOR_BRAKE, COLOR_REF, FONT_FAMILY

RESULTS = Path("tests/research/results")
DEFAULT_CONFIG_PATH = Path("tests/research/config_testVehicle.yaml")
DEFAULT_OUT = RESULTS / "report20260913_debag_process2,3"

MODE_DRIVE = "MODE_DRIVE"
MOVING_REF_KMH = 5.0  # 切り替えを数える基準車速の下限 [km/h]
STOPPED_KMH = 1.0  # これ未満になったら「完全停止」[km/h]
SLOPE_WINDOW_S = 0.4  # 減速度・加速度を出す窓（ガバナーと同じ）[s]
LOOKAHEAD_S = 1.5  # 切り替え後に最大減速・最低車速を探す長さ [s]
GOVERNOR_LIMIT_KMHS = 0.4 * 35.30394 * 0.98  # 0.4G × 0.98 [km/h/s]
ZERO_CURRENT_S = 1.0  # 電流 0 mA がこの長さ続いたら脱落とみなす [s]


@dataclass(frozen=True)
class RunSpec:
    label: str
    csv: str
    brake_deadband_pct: float
    brake_standby_pct: float  # 待機位置（無しなら 0）


RUNS: tuple[RunSpec, ...] = (
    RunSpec("9/11 C0", "drive_log_real_20260911_171637.csv", 13.68, 0.0),
    RunSpec("9/13 06:05 C1", "drive_log_real_20260913_060556.csv", 13.68, 0.0),
    RunSpec("9/13 07:27 C1", "drive_log_real_20260913_072716.csv", 13.16, 0.0),
    RunSpec("9/13 10:32 C1+待機位置", "drive_log_real_20260913_103244.csv", 13.16, 11.16),
)
STEP2_CSV = "drive_log_real_20260913_071502.csv"  # 9/13 07:15 手順 2 やり直し
MODELS = (
    ("9/11 手順2", "test_vehicle_20260912_142144.pkl"),
    ("9/13 07:15 手順2", "test_vehicle_20260912_222641.pkl"),
)


@dataclass(frozen=True)
class Row:
    """CSV の 1 行（解析に使う列だけ）。"""

    t: float  # MODE_DRIVE は mode_time_s、それ以外は elapsed_s
    section: str
    phase: str
    ref: float
    v: float
    accel: float
    brake: float
    ff: float
    brake_current: float
    governor: bool


def read_rows(path: Path, sections: Sequence[str] = (MODE_DRIVE,)) -> list[Row]:
    def num(text: str | None) -> float:
        return float(text) if text else 0.0

    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["section"] not in sections:
                continue
            gov = r.get("governor_active")
            rows.append(Row(
                t=num(r["mode_time_s"]) if r["section"] == MODE_DRIVE else num(r["elapsed_s"]),
                section=r["section"],
                phase=r["phase"],
                ref=num(r["ref_speed_kmh"]),
                v=num(r["actual_speed_kmh"]),
                accel=cmd_opening(r, "accel"),
                brake=cmd_opening(r, "brake"),
                ff=ff_effort(r) or 0.0,
                brake_current=num(r["brake_current"]),
                governor=(gov == "1") if gov else r["phase"] == "BRAKE_GOV",
            ))
    return rows


def _is_brake(r: Row) -> bool:
    return r.phase in ("BRAKE", "BRAKE_GOV")


def _max_slope(seg: Sequence[Row], window_s: float, *, decel: bool) -> float:
    """seg の中で window_s 窓の車速の傾きの最大（decel=True なら減速を正）。"""
    best = 0.0
    j = 0
    for i in range(len(seg)):
        while j < len(seg) and seg[j].t - seg[i].t < window_s - 1e-6:
            j += 1
        if j >= len(seg):
            break
        dv = seg[i].v - seg[j].v if decel else seg[j].v - seg[i].v
        best = max(best, dv / (seg[j].t - seg[i].t))
    return best


# ─────────────────────────────────────────────────────────────────────
# 切り替え
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class SwitchEvent:
    t: float
    v0: float
    before_pct: float  # 切り替え直前に指令していた、切り替え先のペダルの開度
    after_pct: float  # 切り替え後 1 周期目の開度
    peak_kmhs: float  # LOOKAHEAD_S 内の最大減速（ブレーキ）/ 最大加速（アクセル）
    v_min: float


def switch_events(
    rows: Sequence[Row], *, to_brake: bool, t_max: float | None = None
) -> list[SwitchEvent]:
    """ACCEL ↔ BRAKE の切り替え（基準 > MOVING_REF_KMH）ごとの跳びと応答。"""
    out = []
    for i in range(1, len(rows)):
        prev, cur = rows[i - 1], rows[i]
        if t_max is not None and cur.t > t_max:
            break
        switched = (prev.phase == "ACCEL" and _is_brake(cur)) if to_brake else (
            _is_brake(prev) and cur.phase == "ACCEL"
        )
        if not switched or cur.ref <= MOVING_REF_KMH:
            continue
        seg = [r for r in rows[i : i + 40] if r.t - cur.t <= LOOKAHEAD_S]
        out.append(SwitchEvent(
            t=cur.t,
            v0=cur.v,
            before_pct=prev.brake if to_brake else prev.accel,
            after_pct=cur.brake if to_brake else cur.accel,
            peak_kmhs=_max_slope(seg, SLOPE_WINDOW_S, decel=to_brake),
            v_min=min(r.v for r in seg),
        ))
    return out


@dataclass(frozen=True)
class SwitchSummary:
    n: int
    full_stops: int  # 最低車速が STOPPED_KMH 未満
    over_limit: int  # 最大減速がガバナーの上限以上
    median_peak_kmhs: float


def summarize_switches(events: Sequence[SwitchEvent]) -> SwitchSummary:
    peaks = [e.peak_kmhs for e in events]
    return SwitchSummary(
        n=len(events),
        full_stops=sum(e.v_min < STOPPED_KMH for e in events),
        over_limit=sum(e.peak_kmhs >= GOVERNOR_LIMIT_KMHS for e in events),
        median_peak_kmhs=float(np.median(peaks)) if peaks else 0.0,
    )


# ─────────────────────────────────────────────────────────────────────
# ブレーキの立ち上がり・開度ごとの効き
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Onset:
    t: float
    v0: float
    first_pct: float  # 立ち上がり 1 行目の開度
    peak_pct: float  # 3s 以内の最大開度
    t90_s: float  # 最大開度の 90% に届くまで
    max_decel_kmhs: float


def onset_ramp(rows: Sequence[Row], floor_pct: float = 0.0, min_v0: float = 5.0) -> list[Onset]:
    """ブレーキ開度が floor_pct 以下から上がった区間ごとの立ち上がり方（3s まで）。"""
    out = []
    i = 1
    while i < len(rows):
        if not (rows[i - 1].brake <= floor_pct + 1e-3 < rows[i].brake):
            i += 1
            continue
        t0 = rows[i].t
        j = i
        while j < len(rows) and rows[j].brake > floor_pct + 1e-3 and rows[j].t - t0 <= 3.0:
            j += 1
        seg = rows[i:j]
        if rows[i - 1].v > min_v0 and seg:
            peak = max(r.brake for r in seg)
            t90 = next(r.t - t0 for r in seg if r.brake >= 0.9 * peak)
            out.append(Onset(t0, rows[i - 1].v, seg[0].brake, peak, t90,
                             _max_slope(seg, SLOPE_WINDOW_S, decel=True)))
        i = max(j, i + 1)
    return out


def decel_by_opening(
    rows: Sequence[Row], window_s: float = SLOPE_WINDOW_S
) -> dict[tuple[int, int], float]:
    """開度が窓の間ほぼ一定（±1%）の行で、(開度 1% 刻み, 車速 10 km/h 帯) → 減速度の中央値。"""
    cells: dict[tuple[int, int], list[float]] = {}
    j = 0
    for i in range(len(rows)):
        while j < i and rows[i].t - rows[j].t > window_s + 1e-6:
            j += 1
        a, b = rows[j], rows[i]
        steady = abs(b.brake - a.brake) <= 1.0 and b.t - a.t >= window_s - 0.06
        if b.brake <= 0.0 or b.v < 3.0 or not steady:
            continue
        key = (int(b.brake), int(b.v // 10) * 10)
        cells.setdefault(key, []).append((a.v - b.v) / (b.t - a.t))
    return {k: float(np.median(v)) for k, v in cells.items()}


# ─────────────────────────────────────────────────────────────────────
# ガバナー・脱落・暴走
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class GovernorEpisode:
    t0: float
    duration_s: float
    v_start: float
    v_end: float
    brake_start: float
    brake_end: float
    latched_s: float  # 開度が下限（floor）に張り付き、FF はそれより深いブレーキを出していた時間


def governor_episodes(
    rows: Sequence[Row], floor_pct: float, dt: float = 0.1
) -> list[GovernorEpisode]:
    out = []
    cur: list[Row] = []
    for r in [*rows, None]:
        if r is not None and r.governor:
            cur.append(r)
            continue
        if cur:
            floor = floor_pct + 0.05
            latched = sum(dt for x in cur if x.brake <= floor and -x.ff > floor)
            out.append(GovernorEpisode(cur[0].t, len(cur) * dt, cur[0].v, cur[-1].v,
                                       cur[0].brake, cur[-1].brake, latched))
            cur = []
    return out


@dataclass(frozen=True)
class Dropout:
    t: float
    brake_max_before: float  # 直前 1s のブレーキ開度の最大
    brake_at: float  # 0 mA になった行の開度
    governor_before: bool  # 直前 1s にガバナーが作動していたか


def axis_dropouts(rows: Sequence[Row]) -> list[Dropout]:
    """ブレーキ電流 0 mA が ZERO_CURRENT_S 以上続き始めた時刻。"""
    out = []
    i = 0
    while i < len(rows):
        if rows[i].brake_current != 0.0:
            i += 1
            continue
        j = i
        while j < len(rows) and rows[j].brake_current == 0.0:
            j += 1
        if rows[j - 1].t - rows[i].t >= ZERO_CURRENT_S:
            before = [r for r in rows[:i] if rows[i].t - r.t <= 1.0]
            out.append(Dropout(rows[i].t, max((r.brake for r in before), default=0.0),
                               rows[i].brake, any(r.governor for r in before)))
        i = j
    return out


@dataclass(frozen=True)
class Runaway:
    t0: float
    t1: float
    v0: float
    v1: float
    ref1: float
    accel_min: float
    accel_max: float
    first_accel_kmhs: float  # 最初の 2s の平均加速度
    last_accel_kmhs: float  # 最後の 2s の平均加速度


def runaway_window(rows: Sequence[Row], t0: float, t1: float) -> Runaway:
    seg = [r for r in rows if t0 <= r.t <= t1]
    head = [r for r in seg if r.t <= seg[0].t + 2.0]
    tail = [r for r in seg if r.t >= seg[-1].t - 2.0]
    return Runaway(
        seg[0].t, seg[-1].t, seg[0].v, seg[-1].v, seg[-1].ref,
        min(r.accel for r in seg), max(r.accel for r in seg),
        (head[-1].v - head[0].v) / (head[-1].t - head[0].t),
        (tail[-1].v - tail[0].v) / (tail[-1].t - tail[0].t),
    )


def b3_clamped_rows(rows: Sequence[Row], brake_deadband_pct: float) -> tuple[int, int]:
    """(走行中に FF がブレーキを出した行数, そのうち B3 で不感帯ちょうどに切り上げた行数)。"""
    brake = [r for r in rows if r.ff < 0.0 and r.ref > 0.5]
    return len(brake), sum(abs(-r.ff - brake_deadband_pct) < 0.02 for r in brake)


def search_onsets(rows: Sequence[Row], margin_kmh: float = 0.3) -> dict[str, float]:
    """手順 2-0 のペダル探索で、探索開始の車速から margin 以上動いた最初の開度（記録値とは別）。"""
    out = {}
    for phase, key, sign in (("ACCEL_SEARCH", "accel", 1.0), ("BRAKE_SEARCH", "brake", -1.0)):
        seg = [r for r in rows if r.phase == phase]
        if not seg:
            continue
        base = seg[0].v
        hit = next((r for r in seg if sign * (r.v - base) > margin_kmh), None)
        if hit is not None:
            out[key] = hit.accel if key == "accel" else hit.brake
    return out


# ─────────────────────────────────────────────────────────────────────
# 表
# ─────────────────────────────────────────────────────────────────────


def md_table(header: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(lines)


def _print(title: str, body: str) -> None:
    print(f"\n### {title}\n\n{body}")


# ─────────────────────────────────────────────────────────────────────
# 図（pyplot を使わない Figure）
# ─────────────────────────────────────────────────────────────────────


def _figure(nrows: int, height: float, **kwargs: Any) -> tuple[Any, Any]:
    from matplotlib.figure import Figure  # noqa: PLC0415

    fig = Figure(figsize=(12.0, height), dpi=100, layout="constrained")
    return fig, fig.subplots(nrows, 1, **kwargs)


def _speed_pedal_axes(axs: Any, rows: Sequence[Row], title: str, lines: dict[str, float]) -> None:
    t = [r.t for r in rows]
    ax_v, ax_p = axs
    ax_v.plot(t, [r.ref for r in rows], color=COLOR_REF, linewidth=2.0, label="基準車速")
    ax_v.plot(t, [r.v for r in rows], color=COLOR_ACTUAL, linewidth=1.8, label="実車速")
    ax_v.set_ylabel("車速 [km/h]")
    ax_v.set_title(title)
    ax_v.legend(loc="upper right")
    ax_p.plot(t, [r.accel for r in rows], color=COLOR_ACCEL, linewidth=1.8, label="アクセル開度")
    ax_p.plot(t, [r.brake for r in rows], color=COLOR_BRAKE, linewidth=1.8, label="ブレーキ開度")
    ax_p.plot(t, [max(0.0, -r.ff) for r in rows], color=COLOR_BRAKE, linestyle="--", linewidth=1.0,
              label="FF のブレーキ指令")
    for label, y in lines.items():
        ax_p.axhline(y, color="#555555", linestyle=":", linewidth=1.0)
        ax_p.text(t[0], y, f" {label}", va="bottom", fontsize=9, color="#555555")
    for ax in axs:
        for r0, r1 in zip(rows, rows[1:], strict=False):
            if r0.governor:
                ax.axvspan(r0.t, r1.t, color="#ffd27f", alpha=0.35, linewidth=0)
        ax.grid(True, alpha=0.3)
    ax_p.set_ylabel("開度 [%]")
    ax_p.set_xlabel("モード経過時間 [s]（黄帯 = Gガバナー作動）")
    ax_p.legend(loc="upper right")


def fig_runs(out: Path, labels: Sequence[str], values: Mapping[str, Sequence[float]]) -> None:
    fig, axs = _figure(len(values), 2.2 * len(values), sharex=True)
    x = np.arange(len(labels))
    for ax, (name, ys) in zip(axs, values.items(), strict=True):
        ax.bar(x, ys, color=COLOR_ACTUAL)
        for xi, y in zip(x, ys, strict=True):
            ax.text(xi, y, f"{y:g}", ha="center", va="bottom", fontsize=9)
        ax.set_ylabel(name, fontsize=9)
        ax.margins(y=0.2)
        ax.grid(True, axis="y", alpha=0.3)
    axs[-1].set_xticks(x, labels)
    fig.savefig(out / "runs_kpi.png")


def fig_onsets(out: Path, series: dict[str, list[tuple[list[float], list[float]]]]) -> None:
    fig, axs = _figure(1, 4.5)
    colors = {0: "#2f7ed8", 1: "#8bbc21", 2: "#d0342c"}
    for k, (name, curves) in enumerate(series.items()):
        for n, (tt, bb) in enumerate(curves):
            axs.plot(tt, bb, color=colors[k % 3], alpha=0.55, linewidth=1.2,
                     label=name if n == 0 else None)
    axs.set_xlabel("ブレーキを踏み始めてからの時間 [s]")
    axs.set_ylabel("ブレーキ開度 [%]")
    axs.set_title("ブレーキの立ち上がり方（最大開度 13% 以上の立ち上がりを重ね描き）")
    axs.grid(True, alpha=0.3)
    axs.legend(loc="lower right")
    fig.savefig(out / "onset_c0_vs_c1.png")


def fig_decel(out: Path, step2: dict[tuple[int, int], float], step3: Sequence[SwitchEvent]) -> None:
    fig, axs = _figure(1, 4.5)
    bands = sorted({b for (_, b) in step2 if 10 <= b <= 70})
    for band in bands:
        pts = sorted((p, d) for (p, b), d in step2.items() if b == band and 12 <= p <= 18)
        if pts:
            axs.plot([p + 0.5 for p, _ in pts], [d for _, d in pts], marker="o", linewidth=1.0,
                     label=f"手順2（1.5s ランプ）{band}〜{band + 10} km/h")
    axs.scatter([e.after_pct for e in step3], [e.peak_kmhs for e in step3], color=COLOR_BRAKE,
                marker="x", s=40, label="手順3 10:32（1 周期で踏む）切り替え直後の最大減速")
    axs.axhline(GOVERNOR_LIMIT_KMHS, color="#555555", linestyle=":", linewidth=1.0)
    axs.text(0.01, GOVERNOR_LIMIT_KMHS, " Gガバナーの上限", va="bottom", fontsize=9,
             transform=axs.get_yaxis_transform())
    axs.set_xlabel("ブレーキ開度 [%]")
    axs.set_ylabel("減速度 [km/h/s]")
    axs.set_title("同じブレーキ開度でも、一気に踏むと効き方が違う")
    axs.grid(True, alpha=0.3)
    axs.legend(loc="upper right", fontsize=8, ncol=2)
    fig.savefig(out / "decel_step2_vs_step3.png")


def fig_runaway(out: Path, runs: dict[str, Sequence[Row]], t0: float, t1: float) -> None:
    fig, axs = _figure(3, 8.0, sharex=True)
    for k, (name, rows) in enumerate(runs.items()):
        seg = [r for r in rows if t0 <= r.t <= t1]
        t = np.array([r.t for r in seg])
        v = np.array([r.v for r in seg])
        color = ("#2f7ed8", "#d0342c")[k % 2]
        if k == 0:
            axs[0].plot(t, [r.ref for r in seg], color=COLOR_REF, linewidth=2.0, label="基準車速")
        axs[0].plot(t, v, color=color, linewidth=1.6, label=f"実車速 {name}")
        axs[1].plot(t, [r.accel for r in seg], color=color, linewidth=1.6,
                    label=f"アクセル開度 {name}")
        acc = np.gradient(v, t) if len(t) > 2 else np.zeros_like(v)
        kernel = np.ones(10) / 10  # 1s 移動平均（CAN 車速の刻みをならす）
        smooth = np.convolve(acc, kernel, mode="same")
        axs[2].plot(t, smooth, color=color, linewidth=1.6, label=name)
    axs[0].set_ylabel("車速 [km/h]")
    axs[1].set_ylabel("開度 [%]")
    axs[2].set_ylabel("加速度 [km/h/s]（1s 平均）")
    axs[2].set_xlabel("モード経過時間 [s]")
    axs[0].set_title("終盤の 140 km/h 超え: アクセル一定なのに加速が増えていく（2 本とも同じ形）")
    for ax in axs:
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper left", fontsize=9)
    fig.savefig(out / "runaway_1216s.png")


# ─────────────────────────────────────────────────────────────────────
# 実行
# ─────────────────────────────────────────────────────────────────────


def run(cfg_path: Path, out: Path) -> int:  # noqa: PLR0914, PLR0915 - レポート 1 本分の表と図を順に出す
    cfg = load_config(cfg_path)
    out.mkdir(parents=True, exist_ok=True)
    matplotlib.rcParams["font.family"] = FONT_FAMILY
    runs = {spec.label: read_rows(RESULTS / spec.csv) for spec in RUNS}

    # runs: KPI と補助指標
    table: list[list[str]] = []
    values: dict[str, list[float]] = {
        k: [] for k in ("最大逸脱 [km/h]", "p95 [km/h]", "完全停止 [回]", "ガバナー [s]")
    }
    for spec in RUNS:
        rows = runs[spec.label]
        kpi = compute_kpi([r.t for r in rows], [r.v - r.ref for r in rows], cfg.kpi)
        sw = summarize_switches(switch_events(rows, to_brake=True))
        gov_s = sum(0.1 for r in rows if r.governor)
        table.append([
            spec.label, f"{rows[-1].t:.1f}", f"{kpi.max_abs_kmh:.2f}", f"{kpi.p95_kmh:.2f}",
            str(kpi.reversal_max_per_window), str(sw.n), str(sw.full_stops), str(sw.over_limit),
            f"{sw.median_peak_kmhs:.1f}", f"{gov_s:.1f}", f"{rows[-1].v:.1f}",
        ])
        for key, val in zip(values, (round(kpi.max_abs_kmh, 1), round(kpi.p95_kmh, 1),
                                     sw.full_stops, round(gov_s, 1)), strict=True):
            values[key].append(val)
    _print("runs: 4 本の KPI と補助指標（アクセル→ブレーキ切り替え、基準 > 5 km/h）", md_table(
        ["走行", "走行時間 [s]", "最大逸脱", "p95", "符号反転", "切替", "完全停止", "上限超え",
         "最大減速の中央値", "ガバナー [s]", "終端車速"], table))
    fig_runs(out, [s.label for s in RUNS], values)

    # switch: 同じ区間（0〜953s）で 07:27 と 10:32 を比べる
    cmp_rows = []
    for spec in RUNS[2:]:
        sw = summarize_switches(switch_events(runs[spec.label], to_brake=True, t_max=953.0))
        cmp_rows.append([spec.label, str(sw.n), str(sw.full_stops), str(sw.over_limit),
                         f"{sw.median_peak_kmhs:.1f}"])
    _print("switch: 0〜953s で比べた急ブレーキ（待機位置の効果）", md_table(
        ["走行", "切替", "完全停止", "上限超え", "最大減速の中央値 [km/h/s]"], cmp_rows))
    b3 = [[s.label, *map(str, b3_clamped_rows(runs[s.label], s.brake_deadband_pct))]
          for s in RUNS[1:]]
    _print("switch: B3（不感帯への切り上げ）が効いた行", md_table(
        ["走行", "FF のブレーキ行（基準 > 0.5 km/h）", "不感帯ちょうどの行"], b3))
    new = runs[RUNS[3].label]
    ev_b = switch_events(new, to_brake=True)
    _print("switch: 10:32 アクセル→ブレーキの各切り替え", md_table(
        ["t [s]", "車速", "開度 前→後 [%]", "最大減速 [km/h/s]", "最低車速"],
        [[f"{e.t:.1f}", f"{e.v0:.1f}", f"{e.before_pct:.2f}→{e.after_pct:.2f}",
          f"{e.peak_kmhs:.1f}", f"{e.v_min:.1f}"] for e in ev_b]))
    ev_a = switch_events(new, to_brake=False)
    jumps = [e.after_pct - e.before_pct for e in ev_a]
    _print("switch: 10:32 ブレーキ→アクセル", (
        f"{len(ev_a)} 回。アクセル開度の跳び 最大 {max(jumps):.2f}%・"
        f"中央値 {np.median(jumps):.2f}%、"
        f"最大加速 {max(e.peak_kmhs for e in ev_a):.1f} km/h/s"))
    zoom = [r for r in new if 430.0 <= r.t <= 446.0]
    fig, axs = _figure(2, 7.0, sharex=True)
    _speed_pedal_axes(axs, zoom,
                      "10:32（待機位置あり）t=430〜446s: ブレーキが 1 周期で待機位置から指令開度へ",
                      {"ブレーキ不感帯 13.16%": 13.16, "ブレーキ待機位置 11.16%": 11.16})
    fig.savefig(out / "brake_switch_zoom.png")

    # onset: 立ち上がり方
    step2 = read_rows(RESULTS / STEP2_CSV, ("PATTERN_DRIVE", "DECEL_TO_STOP"))
    sources = {
        "手順2 07:15（パターン走行）": (step2, 0.0),
        "手順3 9/11 C0": (runs[RUNS[0].label], 0.0),
        "手順3 9/13 07:27 C1": (runs[RUNS[2].label], 0.0),
    }
    onset_rows, curves = [], {}
    for name, (rows, floor) in sources.items():
        ons = [o for o in onset_ramp(rows, floor) if o.peak_pct >= 13.0]
        onset_rows.append([
            name, str(len(ons)),
            f"{min(o.first_pct for o in ons):.1f}〜{max(o.first_pct for o in ons):.1f}",
            f"{np.median([o.t90_s for o in ons]):.2f}",
            f"{min(o.max_decel_kmhs for o in ons):.1f}〜{max(o.max_decel_kmhs for o in ons):.1f}",
        ])
        curves[name] = []
        for o in ons:
            seg = [r for r in rows if o.t - 0.2 <= r.t <= o.t + 3.0]
            curves[name].append(([r.t - o.t for r in seg], [r.brake for r in seg]))
    _print("onset: ブレーキの立ち上がり方（最大開度 13% 以上、踏み始め車速 > 5 km/h）", md_table(
        ["走行", "回数", "1 行目の開度 [%]", "90% 到達の中央値 [s]", "最大減速 [km/h/s]"],
        onset_rows))
    fig_onsets(out, curves)

    # decel: 手順 2 の開度 × 車速帯
    dec = decel_by_opening(step2)
    bands = (10, 20, 30, 40, 50, 60)
    _print("decel: 手順2 07:15 のブレーキ開度 × 車速帯の減速度の中央値 [km/h/s]", md_table(
        ["開度 [%]", *[f"{b}〜{b + 10}" for b in bands]],
        [[str(p), *[f"{dec[(p, b)]:.1f}" if (p, b) in dec else "—" for b in bands]]
         for p in range(14, 18)]))
    in_band = [e for e in ev_b if 20.0 <= e.v0 < 40.0 and 15.0 <= e.after_pct < 18.0]
    if in_band:
        _print("decel: 手順3 10:32 で 15〜18%・20〜40 km/h に切り替えた回", (
            f"{len(in_band)} 回、最大減速 {min(e.peak_kmhs for e in in_band):.1f}〜"
            f"{max(e.peak_kmhs for e in in_band):.1f} km/h/s"))
    fig_decel(out, dec, ev_b)

    # governor
    for spec in RUNS[1:]:
        eps = governor_episodes(runs[spec.label], spec.brake_standby_pct)
        long = [e for e in eps if e.latched_s >= 3.0]
        title = f"governor: {spec.label}（作動 {len(eps)} 区間）下限に 3s 以上張り付いた区間"
        _print(title, md_table(
            ["開始 t [s]", "長さ [s]", "車速 始→終", "開度 始→終 [%]", "張り付き [s]"],
            [[f"{e.t0:.1f}", f"{e.duration_s:.1f}", f"{e.v_start:.1f}→{e.v_end:.1f}",
              f"{e.brake_start:.2f}→{e.brake_end:.2f}", f"{e.latched_s:.1f}"] for e in long]))
    latch = [r for r in new if 430.0 <= r.t <= 515.0]
    fig, axs = _figure(2, 7.0, sharex=True)
    _speed_pedal_axes(axs, latch, "10:32 t=430〜515s: ガバナーが待機位置に張り付き、止まれない",
                      {"ブレーキ待機位置 11.16%": 11.16, "停車保持 30%": 30.0})
    fig.savefig(out / "governor_latch.png")

    # dropout
    drop_rows = []
    for spec in RUNS[1:]:
        for d in axis_dropouts(runs[spec.label]):
            drop_rows.append([spec.label, f"{d.t:.1f}", f"{d.brake_max_before:.2f}",
                              f"{d.brake_at:.2f}",
                              "あり" if d.governor_before else "なし"])
    _print("dropout: ブレーキ電流 0 mA が 1s 以上続き始めた時刻", md_table(
        ["走行", "t [s]", "直前 1s の最大開度 [%]", "その時の開度 [%]", "直前のガバナー"],
        drop_rows)
        if drop_rows else "なし")

    # runaway
    rw_rows = []
    for spec in (RUNS[1], RUNS[3]):
        rows = runs[spec.label]
        w = runaway_window(rows, 1195.0, rows[-1].t)
        rw_rows.append([spec.label, f"{w.t0:.1f}〜{w.t1:.1f}", f"{w.v0:.1f}→{w.v1:.1f}",
                        f"{w.ref1:.1f}",
                        f"{w.accel_min:.1f}〜{w.accel_max:.1f}", f"{w.first_accel_kmhs:.2f}",
                        f"{w.last_accel_kmhs:.2f}"])
    _print("runaway: 終盤（t ≥ 1195s）", md_table(
        ["走行", "t [s]", "車速", "終端の基準", "アクセル開度 [%]",
         "最初 2s の加速", "最後 2s の加速"],
        rw_rows))
    fig_runaway(out, {s.label: runs[s.label] for s in (RUNS[1], RUNS[3])}, 1190.0, 1219.0)

    # 手順 2 のペダル探索とモデル
    search = search_onsets(read_rows(RESULTS / STEP2_CSV, ("PEDAL_SEARCH",)))
    _print("step2: 07:15 ペダル探索で車速が 0.3 km/h 動いた最初の開度",
           f"アクセル {search.get('accel', float('nan')):.2f}%・"
           f"ブレーキ {search.get('brake', float('nan')):.2f}%")
    model_rows = []
    for name, pkl in MODELS:
        with (RESULTS / "models" / pkl).open("rb") as f:
            payload = pickle.load(f)  # noqa: S301 - 自分で保存した研究用 pkl
        m, db = payload["metrics"], payload["deadbands_pct"]
        model_rows.append([
            name, f"{db['accel']:.2f} / {db['brake']:.2f}",
            f"{m['accel']['n']:.0f} / {m['brake']['n']:.0f}",
            f"{m['accel']['mae']:.2f} / {m['brake']['mae']:.2f}",
            f"{m['accel']['r2']:.3f} / {m['brake']['r2']:.3f}",
            f"{100 * m['accel']['below_deadband']:.1f} / {100 * m['brake']['below_deadband']:.1f}",
        ])
    _print("step2: A1 モデル（アクセル / ブレーキ）", md_table(
        ["手順2", "不感帯 [%]", "学習行", "MAE [%]", "R²", "不感帯未満の予測 [%]"], model_rows))
    print(f"\n図: {out}/")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT, help="図の保存先")
    args = ap.parse_args(argv)
    return run(args.config, args.out)


if __name__ == "__main__":
    raise SystemExit(main())
