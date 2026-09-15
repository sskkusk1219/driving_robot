"""モード走行のレポート（reportYYYYMMDD_Run<ラベル>.md ＋ 図）を作る（手順 3/5/7/9）。

走行 CSV の MODE_DRIVE の行（0.1s 刻み）だけから作るので、走行後に CSV から作り直せる:

    .venv/bin/python -m tests.research.mode_report \
        tests/research/results/drive_log_real_XXXX.csv --label FF

中身:
    1. 結論（プライマリー KPI の判定表）
    2. 実施条件
    3. 走行全体の図（本番の自動走行画面と同じ 2 段）と偏差の図
    4. どこで外れたか（WLTP 区間別・走行状態別・速度帯別・1.0 km/h 超えの区間・
       最大逸脱付近の拡大図）
    5. ペダル指令の特徴（不感帯の中に落ちた指令など）
    6. 所見（データから機械的に抜き出した事実）
    7. 考察・次の手順（走行後に追記する欄）
"""

from __future__ import annotations

import argparse
import csv
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from src.models.profile import FeedforwardParams, coast_decel_at
from tests.research.config import ResearchConfig, load_config
from tests.research.drive_log import (
    SECTION_MODE_DRIVE,
    DriveSample,
    cmd_opening,
    ff_effort,
    pid_effort,
    total_effort,
)
from tests.research.kpi import KpiResult, compute_kpi, sample_interval_s
from tests.research.live_plot import (
    COLOR_ACCEL,
    COLOR_ACTUAL,
    COLOR_BRAKE,
    COLOR_REF,
    FONT_FAMILY,
    PlotSample,
    save_drive_figure,
)
from tests.research.mode_drive import standby_label
from tests.research.term import say
from tests.research.vehicle import feedforward_params

# 走行状態の分類: 基準車速の ±REF_SLOPE_HALF_WINDOW_S の傾き [km/h/s] で分ける
REF_SLOPE_HALF_WINDOW_S = 0.5
STATE_SLOPE_KMHS = 0.3
STATE_STOP = "停車"
STATE_ACCEL = "加速"
STATE_CRUISE = "定速"
STATE_DECEL = "減速"
STATES = (STATE_STOP, STATE_ACCEL, STATE_CRUISE, STATE_DECEL)
SPEED_BANDS_KMH = (0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 1000.0)
ZOOM_HALF_WINDOW_S = 20.0
TOP_EPISODES = 10
COLOR_DEV = "#5a6fd6"
COLOR_LIMIT = "#d64545"
COLOR_P95 = "#e0a030"


@dataclass(frozen=True)
class ModeRow:
    """モード走行の 1 行（0.1s 刻み）。"""

    t_s: float
    ref_kmh: float
    actual_kmh: float
    accel_pct: float
    brake_pct: float
    ff_effort_pct: float
    pid_effort_pct: float
    effort_pct: float
    segment: str
    phase: str

    @property
    def deviation_kmh(self) -> float:
        return self.actual_kmh - self.ref_kmh


@dataclass
class RunInfo:
    """レポートの「実施条件」。CSV から作り直すときは分からない項目が None のまま。"""

    label: str  # ファイル名のラベル（FF / FF&Kp …）
    title: str  # 見出し（手順 3: FF のみでモード走行 など）
    controller: str  # 制御構成の説明
    csv_path: Path
    hw_mode: str
    mode_name: str
    started_at: datetime
    mode_duration_s: float | None = None
    run_duration_s: float | None = None
    completed: bool | None = None
    limited: bool = False  # --limit-s でモードの先頭だけ走った
    abort_reason: str = ""
    cycles: int | None = None
    overruns: int | None = None
    notes: list[str] = field(default_factory=list)


# ─────────────────────────────────────────────────────────────────────
# 行の読み込み
# ─────────────────────────────────────────────────────────────────────


def rows_from_samples(samples: Sequence[DriveSample]) -> list[ModeRow]:
    rows = []
    for s in samples:
        d = s.data
        if s.section != SECTION_MODE_DRIVE or s.mode_time_s is None or d.ref_speed_kmh is None:
            continue
        rows.append(ModeRow(
            t_s=s.mode_time_s,
            ref_kmh=d.ref_speed_kmh,
            actual_kmh=d.actual_speed_kmh,
            accel_pct=d.accel_opening,
            brake_pct=d.brake_opening,
            ff_effort_pct=d.plan_effort_pct or 0.0,
            pid_effort_pct=d.trim_effort_pct or 0.0,
            effort_pct=d.applied_effort_pct or 0.0,
            segment=s.pattern,
            phase=s.phase,
        ))
    return rows


def rows_from_csv(path: Path) -> list[ModeRow]:
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("section") != SECTION_MODE_DRIVE or not r.get("mode_time_s"):
                continue
            # A7 以降の CSV は PID・合成の列を持たない（PID は手順 5 まで 0、合成 = FF + PID）
            ff = ff_effort(r) or 0.0
            pid = pid_effort(r) or 0.0
            total = total_effort(r)
            rows.append(ModeRow(
                t_s=float(r["mode_time_s"]),
                ref_kmh=float(r["ref_speed_kmh"]),
                actual_kmh=float(r["actual_speed_kmh"]),
                accel_pct=cmd_opening(r, "accel"),
                brake_pct=cmd_opening(r, "brake"),
                ff_effort_pct=ff,
                pid_effort_pct=pid,
                effort_pct=ff + pid if total is None else total,
                segment=r["pattern"],
                phase=r["phase"],
            ))
    return rows


# ─────────────────────────────────────────────────────────────────────
# 集計
# ─────────────────────────────────────────────────────────────────────


def driving_states(rows: Sequence[ModeRow]) -> list[str]:
    """基準車速の傾き（前後 0.5s）で 停車 / 加速 / 定速 / 減速 に分ける。"""
    t = np.array([r.t_s for r in rows])
    ref = np.array([r.ref_kmh for r in rows])
    if len(rows) == 0:
        return []
    ahead = np.interp(t + REF_SLOPE_HALF_WINDOW_S, t, ref)
    behind = np.interp(t - REF_SLOPE_HALF_WINDOW_S, t, ref)
    slope = (ahead - behind) / (2.0 * REF_SLOPE_HALF_WINDOW_S)
    states = []
    for v, a in zip(ref, slope, strict=True):
        if v < 0.1 and abs(a) < STATE_SLOPE_KMHS:
            states.append(STATE_STOP)
        elif a >= STATE_SLOPE_KMHS:
            states.append(STATE_ACCEL)
        elif a <= -STATE_SLOPE_KMHS:
            states.append(STATE_DECEL)
        else:
            states.append(STATE_CRUISE)
    return states


@dataclass(frozen=True)
class GroupStats:
    name: str
    duration_s: float
    share: float
    p95_kmh: float
    max_abs_kmh: float
    mean_kmh: float  # 平均偏差（+: 実車速が速い / −: 遅い）
    over_limit_s: float


def group_stats(
    rows: Sequence[ModeRow], keys: Sequence[str], order: Sequence[str], limit_kmh: float
) -> list[GroupStats]:
    dt = sample_interval_s([r.t_s for r in rows])
    total = len(rows)
    out = []
    for name in order:
        dev = np.array([r.deviation_kmh for r, k in zip(rows, keys, strict=True) if k == name])
        if dev.size == 0:
            continue
        out.append(GroupStats(
            name=name,
            duration_s=dev.size * dt,
            share=dev.size / total,
            p95_kmh=float(np.percentile(np.abs(dev), 95)),
            max_abs_kmh=float(np.max(np.abs(dev))),
            mean_kmh=float(np.mean(dev)),
            over_limit_s=float(np.count_nonzero(np.abs(dev) > limit_kmh)) * dt,
        ))
    return out


def speed_band_label(ref_kmh: float) -> str:
    for lo, hi in zip(SPEED_BANDS_KMH, SPEED_BANDS_KMH[1:], strict=False):
        if ref_kmh < hi:
            return f"{lo:.0f}〜{hi:.0f}" if hi < 1000 else f"{lo:.0f}〜"
    return ""


def speed_band_order() -> list[str]:
    return [speed_band_label(lo) for lo in SPEED_BANDS_KMH[:-1]]


# ブレーキ寄与の判定（2026-09-13 手順3 でブレーキ軸の電流が脱落し、指令は出続けたが制動が
# 効いていなかった事故の再発を、レポートだけからでも後で気づけるようにするための指標）
BRAKE_CONTRIB_WINDOW_S = 0.4  # 実測減速度を出す窓（減速G ガバナーの判定窓と同じ）
BRAKE_CONTRIB_MARGIN_KMHS = 0.5  # 惰行カーブ + この値 未満ならブレーキが効いていないとみなす


@dataclass(frozen=True)
class PedalStats:
    accel_active_s: float
    accel_in_deadband_s: float  # 0 < アクセル指令 < アクセル不感帯
    brake_active_moving_s: float  # 走行中（基準 > 停車）にブレーキ指令があった時間
    brake_in_deadband_moving_s: float  # 走行中に 0 < ブレーキ指令 < ブレーキ不感帯
    accel_max_pct: float
    brake_max_moving_pct: float
    switches: int  # アクセル ⇔ ブレーキの切り替え回数
    governor_s: float
    brake_ineffective_s: float  # 不感帯以上のブレーキ指令なのに惰行カーブ相当しか効いていない時間


def _brake_ineffective_s(
    rows: Sequence[ModeRow], params: FeedforwardParams, brake_db: float, dt: float
) -> float:
    window = max(1, round(BRAKE_CONTRIB_WINDOW_S / dt))
    total = 0.0
    for i in range(window, len(rows)):
        r = rows[i]
        if r.phase != "BRAKE" or r.brake_pct < brake_db or r.ref_kmh <= VEHICLE_STOP_SPEED_KMH:
            continue
        measured = (rows[i - window].actual_kmh - r.actual_kmh) / (window * dt)
        if measured <= coast_decel_at(params, r.actual_kmh) + BRAKE_CONTRIB_MARGIN_KMHS:
            total += dt
    return total


def _uses_accel(r: ModeRow) -> bool:
    """FF がアクセルを選んだ行。待機位置で開度 > 0 のまま待つので、開度ではなく phase で見る。"""
    return r.phase == "ACCEL" and r.accel_pct > 0.0


def _uses_brake(r: ModeRow) -> bool:
    """FF がブレーキを選んだ行（ガバナーで 0% に頭打ちした行は除く。待機位置導入前と同じ）。"""
    return r.phase in ("BRAKE", "BRAKE_GOV") and r.brake_pct > 0.0


def pedal_stats(
    rows: Sequence[ModeRow], accel_db: float, brake_db: float, params: FeedforwardParams
) -> PedalStats:
    dt = sample_interval_s([r.t_s for r in rows])
    accel_rows = [r for r in rows if _uses_accel(r)]
    brake_moving = [r for r in rows if _uses_brake(r) and r.ref_kmh > VEHICLE_STOP_SPEED_KMH]
    switches = 0
    last = ""
    for r in rows:
        pedal = "a" if _uses_accel(r) else "b" if _uses_brake(r) else ""
        if pedal and last and pedal != last:
            switches += 1
        last = pedal or last
    return PedalStats(
        accel_active_s=dt * len(accel_rows),
        accel_in_deadband_s=sum(dt for r in accel_rows if r.accel_pct < accel_db),
        brake_active_moving_s=dt * len(brake_moving),
        brake_in_deadband_moving_s=sum(dt for r in brake_moving if r.brake_pct < brake_db),
        accel_max_pct=max((r.accel_pct for r in accel_rows), default=0.0),
        brake_max_moving_pct=max((r.brake_pct for r in brake_moving), default=0.0),
        switches=switches,
        governor_s=sum(dt for r in rows if r.phase == "BRAKE_GOV"),
        brake_ineffective_s=_brake_ineffective_s(rows, params, brake_db, dt),
    )


# ─────────────────────────────────────────────────────────────────────
# 図
# ─────────────────────────────────────────────────────────────────────


def _figure(nrows: int, height: float, **kwargs: Any) -> tuple[Any, Any]:
    """pyplot を使わない Figure（呼び出し側プロセスの描画状態を汚さない）。"""
    from matplotlib.figure import Figure  # noqa: PLC0415

    fig = Figure(figsize=(12.0, height), dpi=100, layout="constrained")
    return fig, fig.subplots(nrows, 1, **kwargs)


def _segment_lines(ax: Any, cfg: ResearchConfig, t_end: float) -> None:
    for bound in cfg.modes.segment_bounds_s:
        if bound < t_end:
            ax.axvline(bound, color="#999999", linewidth=0.8, linestyle=":")


def plot_overview(rows: Sequence[ModeRow], cfg: ResearchConfig, path: Path, title: str) -> Path:
    samples = [PlotSample(r.t_s, r.ref_kmh, r.actual_kmh, r.accel_pct, r.brake_pct) for r in rows]
    return save_drive_figure(
        samples, path, title=title, max_speed_kmh=cfg.vehicle.max_speed_kmh, has_ref=True
    )


def plot_deviation(
    rows: Sequence[ModeRow], cfg: ResearchConfig, kpi: KpiResult, path: Path
) -> Path:
    t = [r.t_s for r in rows]
    dev = [r.deviation_kmh for r in rows]
    k = cfg.kpi
    fig, ax = _figure(1, 4.0)
    ax.plot(t, dev, color=COLOR_DEV, linewidth=0.8, label="偏差（実車速 − 基準車速）")
    for y, color, label in (
        (k.max_abs_deviation_kmh, COLOR_LIMIT, f"±{k.max_abs_deviation_kmh:g} km/h（最大逸脱）"),
        (k.p95_deviation_kmh, COLOR_P95, f"±{k.p95_deviation_kmh:g} km/h（p95 の基準）"),
    ):
        ax.axhline(y, color=color, linestyle="--", linewidth=1.0, label=label)
        ax.axhline(-y, color=color, linestyle="--", linewidth=1.0)
    for ep in kpi.episodes:
        ax.axvspan(ep.start_s, ep.end_s + 0.1, color=COLOR_LIMIT, alpha=0.15, linewidth=0)
    _segment_lines(ax, cfg, t[-1] if t else 0.0)
    lim = max(2.0, min(10.0, kpi.max_abs_kmh * 1.1))
    ax.set_ylim(-lim, lim)
    ax.set_xlim(t[0] if t else 0.0, t[-1] if t else 1.0)
    ax.set_xlabel("モード経過時間 [s]")
    ax.set_ylabel("偏差 [km/h]")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    fig.savefig(path)
    return path


def plot_zoom(
    rows: Sequence[ModeRow], cfg: ResearchConfig, center_s: float, path: Path
) -> Path:
    lo, hi = center_s - ZOOM_HALF_WINDOW_S, center_s + ZOOM_HALF_WINDOW_S
    part = [r for r in rows if lo <= r.t_s <= hi]
    t = [r.t_s for r in part]
    ff = cfg.feedforward
    k = cfg.kpi
    fig, (ax_v, ax_d, ax_p) = _figure(3, 9.0, sharex=True)
    ax_v.plot(t, [r.ref_kmh for r in part], color=COLOR_REF, linestyle="--", label="基準車速")
    ax_v.plot(t, [r.actual_kmh for r in part], color=COLOR_ACTUAL, linewidth=2.0, label="実車速")
    ax_v.set_ylabel("車速 [km/h]")
    ax_v.legend(loc="upper right")
    ax_d.plot(t, [r.deviation_kmh for r in part], color=COLOR_DEV, label="偏差")
    for y in (k.max_abs_deviation_kmh, -k.max_abs_deviation_kmh):
        ax_d.axhline(y, color=COLOR_LIMIT, linestyle="--", linewidth=1.0)
    ax_d.axvline(center_s, color=COLOR_LIMIT, linewidth=0.8, linestyle=":")
    ax_d.set_ylabel("偏差 [km/h]")
    ax_p.plot(t, [r.accel_pct for r in part], color=COLOR_ACCEL, linewidth=1.8, label="アクセル")
    ax_p.plot(t, [r.brake_pct for r in part], color=COLOR_BRAKE, linewidth=1.8, label="ブレーキ")
    ax_p.axhline(ff.accel_deadband_pct, color=COLOR_ACCEL, linestyle=":", linewidth=1.0,
                 label=f"アクセル不感帯 {ff.accel_deadband_pct:.1f}%")
    ax_p.axhline(ff.brake_deadband_pct, color=COLOR_BRAKE, linestyle=":", linewidth=1.0,
                 label=f"ブレーキ不感帯 {ff.brake_deadband_pct:.1f}%")
    ax_p.set_ylabel("開度 [%]")
    ax_p.set_xlabel("モード経過時間 [s]")
    ax_p.legend(loc="upper right", fontsize=9)
    for ax in (ax_v, ax_d, ax_p):
        ax.grid(True, alpha=0.3)
    if t:
        ax_p.set_xlim(t[0], t[-1])
    fig.savefig(path)
    return path


def plot_distribution(
    rows: Sequence[ModeRow], cfg: ResearchConfig, bands: Sequence[GroupStats], path: Path
) -> Path:
    k = cfg.kpi
    abs_dev = np.abs([r.deviation_kmh for r in rows])
    fig, (ax_h, ax_b) = _figure(2, 7.0)
    ax_h.hist(abs_dev, bins=np.arange(0.0, max(2.0, float(abs_dev.max()) + 0.1), 0.05),
              color=COLOR_DEV)
    ax_h.axvline(k.p95_deviation_kmh, color=COLOR_P95, linestyle="--",
                 label=f"p95 の基準 {k.p95_deviation_kmh:g}")
    ax_h.axvline(k.max_abs_deviation_kmh, color=COLOR_LIMIT, linestyle="--",
                 label=f"最大逸脱の基準 {k.max_abs_deviation_kmh:g}")
    ax_h.axvline(float(np.percentile(abs_dev, 95)), color="#333333",
                 label=f"今回の p95 {float(np.percentile(abs_dev, 95)):.2f}")
    ax_h.set_xlabel("|偏差| [km/h]")
    ax_h.set_ylabel("行数（0.1s 刻み）")
    ax_h.set_yscale("log")
    ax_h.legend(loc="upper right")
    x = np.arange(len(bands))
    ax_b.bar(x - 0.2, [b.p95_kmh for b in bands], width=0.4, color=COLOR_P95, label="p95")
    ax_b.bar(x + 0.2, [b.max_abs_kmh for b in bands], width=0.4, color=COLOR_LIMIT, label="最大")
    ax_b.axhline(k.p95_deviation_kmh, color=COLOR_P95, linestyle="--", linewidth=1.0)
    ax_b.axhline(k.max_abs_deviation_kmh, color=COLOR_LIMIT, linestyle="--", linewidth=1.0)
    ax_b.set_xticks(x, [b.name for b in bands])
    ax_b.set_xlabel("基準車速の帯 [km/h]")
    ax_b.set_ylabel("|偏差| [km/h]")
    ax_b.legend(loc="upper left")
    for ax in (ax_h, ax_b):
        ax.grid(True, alpha=0.3)
    fig.savefig(path)
    return path


# ─────────────────────────────────────────────────────────────────────
# Markdown
# ─────────────────────────────────────────────────────────────────────


def _ok(passed: bool) -> str:
    return "✅ 合格" if passed else "❌ 不合格"


def _table(header: Sequence[str], body: Sequence[Sequence[object]]) -> list[str]:
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    lines += ["| " + " | ".join(str(c) for c in row) + " |" for row in body]
    return lines


def _stats_table(stats: Sequence[GroupStats], first: str) -> list[str]:
    return _table(
        (first, "時間 [s]", "割合", "p95 [km/h]", "最大 [km/h]", "平均偏差 [km/h]", "1.0 超え [s]"),
        [
            (s.name, f"{s.duration_s:.0f}", f"{100 * s.share:.0f}%", f"{s.p95_kmh:.2f}",
             f"{s.max_abs_kmh:.2f}", f"{s.mean_kmh:+.2f}", f"{s.over_limit_s:.1f}")
            for s in stats
        ],
    )


def unique_report_path(results_dir: Path, label: str, started_at: datetime) -> Path:
    """reportYYYYMMDD_Run<label>.md。同じ日に 2 本目以降は _2, _3 … を付けて上書きしない。"""
    base = f"report{started_at:%Y%m%d}_Run{label}"
    path = results_dir / f"{base}.md"
    n = 2
    while path.exists():
        path = results_dir / f"{base}_{n}.md"
        n += 1
    return path


def _observations(
    kpi: KpiResult,
    states: Sequence[GroupStats],
    segments: Sequence[GroupStats],
    pedal: PedalStats,
    rows: Sequence[ModeRow],
    state_of: Callable[[float], str],
    cfg: ResearchConfig,
) -> list[str]:
    ff = cfg.feedforward
    lines = []
    worst = next(r for r in rows if r.t_s == kpi.max_abs_t_s)
    lines.append(
        f"最大逸脱 {kpi.max_abs_kmh:.2f} km/h は t={kpi.max_abs_t_s:.1f}s"
        f"（区間 {worst.segment}・基準 {worst.ref_kmh:.1f} km/h・{state_of(worst.t_s)}中、"
        f"実車速が基準より{'速い' if worst.deviation_kmh > 0 else '遅い'}）。"
    )
    for s in states:
        if s.name == STATE_STOP:
            continue
        if abs(s.mean_kmh) < 0.1:
            tendency = "偏りはほぼない"
        elif s.mean_kmh < 0:
            tendency = "実車速が基準より遅れる（踏み不足か応答の遅れ）"
        else:
            tendency = "実車速が基準より速い（戻し不足・制動不足か応答の遅れ）"
        lines.append(
            f"{s.name}中（{100 * s.share:.0f}%）: 平均偏差 {s.mean_kmh:+.2f} km/h、"
            f"p95 {s.p95_kmh:.2f}・最大 {s.max_abs_kmh:.2f} km/h → {tendency}。"
        )
    if segments:
        worst_seg = max(segments, key=lambda s: s.p95_kmh)
        lines.append(
            f"区間別の p95 が最も大きいのは {worst_seg.name}（{worst_seg.p95_kmh:.2f} km/h）。"
        )
    if pedal.brake_active_moving_s > 0:
        lines.append(
            f"走行中のブレーキ指令 {pedal.brake_active_moving_s:.0f}s のうち "
            f"{pedal.brake_in_deadband_moving_s:.0f}s"
            f"（{100 * pedal.brake_in_deadband_moving_s / pedal.brake_active_moving_s:.0f}%）は"
            f"ブレーキ不感帯 {ff.brake_deadband_pct:.2f}% より浅く、制動が出ていない指令だった。"
        )
    if pedal.accel_active_s > 0:
        lines.append(
            f"アクセル指令 {pedal.accel_active_s:.0f}s のうち {pedal.accel_in_deadband_s:.0f}s"
            f"（{100 * pedal.accel_in_deadband_s / pedal.accel_active_s:.0f}%）は"
            f"アクセル不感帯 {ff.accel_deadband_pct:.2f}% より浅かった。"
        )
    if pedal.governor_s > 0:
        lines.append(
            f"減速G ガバナーが {pedal.governor_s:.1f}s 作動した"
            "（その間はブレーキを FF 指令より浅くしている）。"
        )
    lines.append(
        f"1.0 km/h を超えたのは {len(kpi.episodes)} 回・合計 {kpi.time_over_limit_s:.1f}s"
        f"（走行時間の {100 * kpi.time_over_limit_s / max(rows[-1].t_s, 1e-9):.1f}%）。"
    )
    return lines


def write_mode_report(
    rows: Sequence[ModeRow], cfg: ResearchConfig, info: RunInfo, results_dir: Path
) -> Path:
    """図と Markdown を書き、レポートのパスを返す。"""
    if not rows:
        raise ValueError("モード走行の行がありません（section=MODE_DRIVE）")
    results_dir.mkdir(parents=True, exist_ok=True)
    report_path = unique_report_path(results_dir, info.label, info.started_at)
    fig_dir = results_dir / report_path.stem
    fig_dir.mkdir(parents=True, exist_ok=True)
    k = cfg.kpi
    ff = cfg.feedforward

    t = [r.t_s for r in rows]
    kpi = compute_kpi(t, [r.deviation_kmh for r in rows], k)
    state_keys = driving_states(rows)
    states = group_stats(rows, state_keys, STATES, k.max_abs_deviation_kmh)
    segments = group_stats(
        rows, [r.segment for r in rows], cfg.modes.segment_names, k.max_abs_deviation_kmh
    )
    bands = group_stats(
        rows, [speed_band_label(r.ref_kmh) for r in rows], speed_band_order(),
        k.max_abs_deviation_kmh,
    )
    pedal = pedal_stats(
        rows, ff.accel_deadband_pct, ff.brake_deadband_pct, feedforward_params(cfg)
    )
    state_at = dict(zip(t, state_keys, strict=True))

    say(f"レポートの図を作成します: {fig_dir}/ …")
    import matplotlib  # noqa: PLC0415

    with matplotlib.rc_context({"font.family": FONT_FAMILY}):
        figs = {
            "overview": plot_overview(rows, cfg, fig_dir / "overview.png",
                                      f"{info.title}（{info.mode_name}）"),
            "deviation": plot_deviation(rows, cfg, kpi, fig_dir / "deviation.png"),
            "zoom": plot_zoom(rows, cfg, kpi.max_abs_t_s, fig_dir / "worst_zoom.png"),
            "distribution": plot_distribution(rows, cfg, bands, fig_dir / "distribution.png"),
        }
    rel = {name: f"{fig_dir.name}/{p.name}" for name, p in figs.items()}

    run_s = info.run_duration_s if info.run_duration_s is not None else t[-1]
    if info.completed is False:
        status = f"中断: {info.abort_reason}"
    elif info.limited:
        status = f"動作確認のため先頭 {info.mode_duration_s or run_s:.0f}s で打ち切り"
    else:
        status = "完走"
    reversal_at = (
        f"（t={kpi.reversal_max_t_s:.1f}s）" if kpi.reversal_max_t_s is not None else ""
    )

    md: list[str] = [
        f"# {info.title} — {info.started_at:%Y-%m-%d %H:%M}",
        "",
        "`docs/Problem/ProblemReport_20260910.md` の手順に沿った走行結果。"
        f"制御構成: **{info.controller}**。",
        "",
        "## 1. 結論（プライマリー KPI）",
        "",
        f"{info.mode_name} を {run_s:.0f}s 走り（{status}）、"
        f"**最大逸脱 {kpi.max_abs_kmh:.2f} km/h・p95 {kpi.p95_kmh:.2f} km/h・"
        f"符号反転 最大 {kpi.reversal_max_per_window} 回/{k.reversal_window_s:g}s**。"
        f"プライマリー KPI 3 項目のうち **{kpi.passed_count} 項目**を満たした"
        f"（総合 {_ok(kpi.passed)}）。",
        "",
        *_table(
            ("KPI", "基準", "結果", "判定"),
            [
                ("最大逸脱（例外なし）", f"≤ {k.max_abs_deviation_kmh:g} km/h",
                 f"{kpi.max_abs_kmh:.2f} km/h（t={kpi.max_abs_t_s:.1f}s）", _ok(kpi.max_ok)),
                ("偏差 p95", f"≤ {k.p95_deviation_kmh:g} km/h", f"{kpi.p95_kmh:.2f} km/h",
                 _ok(kpi.p95_ok)),
                (f"符号反転（±{k.reversal_band_kmh:g} km/h を両側で超えた往復）",
                 f"≤ {k.reversal_limit_per_window:g} 回/{k.reversal_window_s:g}s",
                 f"{kpi.reversal_max_per_window} 回{reversal_at}", _ok(kpi.reversal_ok)),
            ],
        ),
        "",
        f"- 判定は MODE_DRIVE の 0.1s 刻みの行（{kpi.n_samples} 行、CSV と同じ）で計算"
        "（`tests/research/kpi.py`、定義は本番 `kpi_monitor.py` と同じ）。",
        "",
        "## 2. 実施条件",
        "",
        *_table(
            ("項目", "値"),
            [
                ("実施日時", f"{info.started_at:%Y-%m-%d %H:%M:%S}"),
                ("ハードウェア", info.hw_mode),
                ("走行モード", info.mode_name),
                ("走行時間", f"{run_s:.1f}s（{status}）"
                 + (f"・予定 {info.mode_duration_s:.0f}s" if info.mode_duration_s else "")),
                ("制御構成", info.controller),
                ("FF モデル", f"`{ff.model_path}`"),
                ("不感帯（アクセル / ブレーキ）",
                 f"{ff.accel_deadband_pct:.2f}% / {ff.brake_deadband_pct:.2f}%（手順 2-0 の実測）"),
                ("停車保持開度", f"{ff.stop_brake_opening_pct:.2f}%"),
                ("待機位置（アクセル / ブレーキ）", standby_label(cfg)),
                ("開度上限（アクセル / ブレーキ）",
                 f"{cfg.vehicle.max_accel_opening_pct:.0f}% / "
                 f"{cfg.vehicle.max_brake_opening_pct:.0f}%"),
                ("制御周期 / ログ周期",
                 f"{cfg.control.loop_interval_ms} ms / {cfg.control.log_interval_ms} ms"
                 + (f"（{info.cycles} 周期、1 周期以上の遅れ {info.overruns} 回）"
                    if info.cycles is not None else "")),
                ("減速G ガバナー（安全網）",
                 f"{'有効' if cfg.mode_drive.decel_governor else '無効'}"
                 f"（{cfg.vehicle.max_decel_g:g}G × 0.98）・作動 {pedal.governor_s:.1f}s"),
                ("走行ログ CSV", f"`{info.csv_path}`"),
            ],
        ),
        "",
        *[f"- {note}" for note in info.notes],
        "",
        "## 3. 走行全体",
        "",
        f"![走行全体]({rel['overview']})",
        "",
        "図1: 本番の自動走行画面と同じレイアウト"
        "（上: 基準車速と実車速、下: アクセル・ブレーキ開度）。",
        "",
        f"![偏差]({rel['deviation']})",
        "",
        f"図2: 偏差（実車速 − 基準車速）。赤破線 = ±{k.max_abs_deviation_kmh:g} km/h、"
        f"橙破線 = ±{k.p95_deviation_kmh:g} km/h、赤帯 = ±{k.max_abs_deviation_kmh:g} km/h を"
        "超えていた区間、点線 = WLTP の区間境界。",
        "",
        "## 4. どこで外れたか",
        "",
        "### 4.1 WLTP 区間別",
        "",
        *_stats_table(segments, "区間"),
        "",
        "### 4.2 走行状態別",
        "",
        f"基準車速の前後 {REF_SLOPE_HALF_WINDOW_S:g}s の傾きで分類"
        f"（±{STATE_SLOPE_KMHS:g} km/h/s 以上を"
        "加速 / 減速、基準 0 km/h 付近を停車）。平均偏差が − なら実車速が基準より遅い。",
        "",
        *_stats_table(states, "状態"),
        "",
        "### 4.3 基準車速の帯別",
        "",
        *_stats_table(bands, "基準車速 [km/h]"),
        "",
        f"![分布]({rel['distribution']})",
        "",
        "図3: 上 = |偏差| の分布（縦軸は対数）、下 = 基準車速の帯ごとの p95 と最大。",
        "",
        f"### 4.4 {k.max_abs_deviation_kmh:g} km/h を超えた区間"
        f"（ピークの大きい順に最大 {TOP_EPISODES} 件）",
        "",
    ]
    if kpi.episodes:
        top = sorted(kpi.episodes, key=lambda e: abs(e.peak_kmh), reverse=True)[:TOP_EPISODES]
        ref_at = {r.t_s: r for r in rows}
        md += _table(
            ("#", "開始 [s]", "長さ [s]", "ピーク偏差 [km/h]", "ピーク時刻 [s]", "区間",
             "基準車速 [km/h]", "状態"),
            [
                (i, f"{e.start_s:.1f}", f"{e.duration_s + 0.1:.1f}", f"{e.peak_kmh:+.2f}",
                 f"{e.peak_t_s:.1f}", ref_at[e.peak_t_s].segment,
                 f"{ref_at[e.peak_t_s].ref_kmh:.1f}", state_at[e.peak_t_s])
                for i, e in enumerate(top, start=1)
            ],
        )
        md.append(f"\n合計 {len(kpi.episodes)} 回・{kpi.time_over_limit_s:.1f}s。")
    else:
        md.append("なし。")
    md += [
        "",
        f"![最大逸脱付近]({rel['zoom']})",
        "",
        f"図4: 最大逸脱（t={kpi.max_abs_t_s:.1f}s）の前後 {ZOOM_HALF_WINDOW_S:g}s。"
        "下段の点線はアクセル / ブレーキの不感帯（これより浅い指令はペダルが効かない）。",
        "",
        "## 5. ペダル指令の特徴",
        "",
        *_table(
            ("項目", "値"),
            [
                ("アクセル指令があった時間", f"{pedal.accel_active_s:.0f}s"),
                (f"　うち不感帯 {ff.accel_deadband_pct:.2f}% より浅い",
                 f"{pedal.accel_in_deadband_s:.0f}s"),
                ("走行中にブレーキ指令があった時間", f"{pedal.brake_active_moving_s:.0f}s"),
                (f"　うち不感帯 {ff.brake_deadband_pct:.2f}% より浅い",
                 f"{pedal.brake_in_deadband_moving_s:.0f}s"),
                ("アクセル開度の最大", f"{pedal.accel_max_pct:.1f}%"),
                ("走行中のブレーキ開度の最大", f"{pedal.brake_max_moving_pct:.1f}%"),
                ("アクセル ⇔ ブレーキの切り替え", f"{pedal.switches} 回"),
                ("減速G ガバナー作動", f"{pedal.governor_s:.1f}s"),
                ("ブレーキ寄与ほぼ0（指令はあるが惰行カーブ相当しか効いていない）",
                 f"{pedal.brake_ineffective_s:.1f}s"),
            ],
        ),
        "",
        "## 6. 所見（データから抜き出した事実）",
        "",
        *[f"- {line}" for line in _observations(
            kpi, states, segments, pedal, rows, lambda x: state_at[x], cfg
        )],
        "",
        "## 7. 考察・次の手順",
        "",
        "- 考察: （走行結果を見て追記する）",
        "- 次の手順と比べるときは、1 章の KPI 表と 4 章の表を基準にする。",
        "",
    ]
    report_path.write_text("\n".join(md), encoding="utf-8")
    say(f"レポート: {report_path}")
    return report_path


# ─────────────────────────────────────────────────────────────────────
# CSV から作り直す
# ─────────────────────────────────────────────────────────────────────


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m tests.research.mode_report",
        description="モード走行の CSV（section=MODE_DRIVE の行）からレポートを作り直す",
    )
    parser.add_argument("csv", type=Path, help="走行ログ CSV（drive_log_<hw>_<日時>.csv）")
    parser.add_argument("--label", default="FF", help="ファイル名のラベル（FF / FF&Kp …）")
    parser.add_argument("--title", default="手順 3: FF のみでモード走行", help="見出し")
    parser.add_argument(
        "--config", type=Path, default=Path("tests/research/config_testVehicle.yaml")
    )
    args = parser.parse_args(argv)
    cfg = load_config(args.config)
    rows = rows_from_csv(args.csv)
    stem = args.csv.stem  # drive_log_<hw>_<YYYYmmdd>_<HHMMSS>
    parts = stem.split("_")
    try:
        started = datetime.strptime("_".join(parts[-2:]), "%Y%m%d_%H%M%S")
        hw_mode = parts[-3]
    except (ValueError, IndexError):
        started, hw_mode = datetime.now(), "?"
    info = RunInfo(
        label=args.label,
        title=args.title,
        controller="CSV から作り直し（制御構成は元の走行のレポートを参照）",
        csv_path=args.csv,
        hw_mode=hw_mode,
        mode_name=cfg.modes.wltp_mode_name,
        started_at=started,
    )
    write_mode_report(rows, cfg, info, args.csv.parent)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
