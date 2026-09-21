"""KAIZEN 5.8.1 の順位基準で、複数本のモード走行 CSV を並べて比べる。

車両には触らない。CSV（MODE_DRIVE の行）を読むだけ。D1（C1 を指令ラベル/実開度ラベルの
2 モデルで比べる）と、5.8 節の C1〜C5 本走行の両方にこのスクリプトを使う。
2026-09-15: 各候補 3 本まとめて走る比較（C1×3→C5×3→C4×3 など）向けに、同じラベルが
複数本あるときの集計表と、基準車速帯別の偏差表を追加した。

    .venv/bin/python -m tests.research.compare_runs \
        tests/research/results/drive_log_real_A.csv tests/research/results/drive_log_real_B.csv \
        --labels "C1 指令ラベル" "C1 実開度ラベル"

    # 各候補 3 本ずつ（--labels 省略。CSV の candidate 列からラベルが付く）
    .venv/bin/python -m tests.research.compare_runs \
        tests/research/results/drive_log_real_<C1_1>.csv \
        tests/research/results/drive_log_real_<C1_2>.csv \
        tests/research/results/drive_log_real_<C1_3>.csv \
        tests/research/results/drive_log_real_<C5_1>.csv \
        ... tests/research/results/drive_log_real_<C4_3>.csv

順位（5.8.1、上から順に見る。同点なら次の基準）:
    1. 走破したか（中断した案は中断時刻の早い順に下。走行ループは duration_s 到達時点で
       抜けるため最後の記録行は 1 周期手前になるので、走破の許容幅は 2 周期分見る）
    2. 最大逸脱（小さいほど良い）
    3. |偏差| p95（小さいほど良い）
    4. 符号反転（少ないほど良い）
    5. 同着ならペダル切替回数・指令の動き p95（実機の振動・摩耗に効く。小さいほど良い）

同じラベルが 2 本以上あるときは、上の 1 本ごとの表に続けて「候補ごとの集計」表
（本数・走破本数・各 KPI の中央値（最小〜最大）。並びは上と同じ順位基準を中央値に当てる）
を出す。基準車速帯別の偏差表（0〜40 / 40〜80 / 80〜120 / 120〜 km/h、平均偏差と |偏差| p95）は
常に出す。
"""

from __future__ import annotations

import argparse
import csv
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, ResearchConfig, load_config
from tests.research.drive_log import SECTION_MODE_DRIVE
from tests.research.kpi import KpiResult, compute_kpi, sample_interval_s
from tests.research.mode_report import ModeRow, PedalStats, pedal_stats, rows_from_csv
from tests.research.vehicle import feedforward_params


@dataclass
class RunResult:
    label: str
    csv_path: Path
    n_rows: int
    reached_s: float
    completed: bool
    kpi: KpiResult
    pedal: PedalStats
    command_rate_p95: float  # |Δ(アクセル% − ブレーキ%)| / dt の p95 [%/s]
    rows: list[ModeRow] = field(default_factory=list, repr=False)

    @property
    def rank_key(self) -> tuple[float, ...]:
        return (
            0.0 if self.completed else 1.0,
            -self.reached_s if not self.completed else 0.0,
            self.kpi.max_abs_kmh,
            self.kpi.p95_kmh,
            float(self.kpi.reversal_max_per_window),
            float(self.pedal.switches),
            self.command_rate_p95,
        )


def command_rate_p95(rows: list[ModeRow]) -> float:
    """指令の動き（アクセル/ブレーキを符号付きにした値の変化率）p95 [%/s]。

    報告書 表 3-9 と同じ量。
    """
    if len(rows) < 2:
        return 0.0
    signed = np.array([r.accel_pct - r.brake_pct for r in rows])
    dt = sample_interval_s([r.t_s for r in rows])
    if dt <= 0.0:
        return 0.0
    rate = np.abs(np.diff(signed)) / dt
    return float(np.percentile(rate, 95)) if len(rate) else 0.0


#: 走破とみなす許容幅は 2 周期。走行ループは duration_s 到達時点で抜けるため最後の行は
#: 1 周期手前になり、そこに周期遅れ 1 回（実測 0.198〜0.228s）が乗ることがある
COMPLETION_TOLERANCE_CYCLES = 2.0


def completion_threshold_s(duration_s: float, t: Sequence[float]) -> float:
    """走破とみなす到達時刻のしきい値 [s]。"""
    return duration_s - COMPLETION_TOLERANCE_CYCLES * sample_interval_s(t)


def evaluate_run(
    csv_path: Path, label: str, cfg: ResearchConfig, duration_s: float
) -> RunResult:
    rows = rows_from_csv(csv_path)
    if not rows:
        raise ValueError(f"{csv_path}: MODE_DRIVE の行がありません")
    t = [r.t_s for r in rows]
    deviation = [r.deviation_kmh for r in rows]
    kpi = compute_kpi(t, deviation, cfg.kpi)
    reached = max(t)
    completed = reached >= completion_threshold_s(duration_s, t)
    pedal = pedal_stats(
        rows, cfg.feedforward.accel_deadband_pct, cfg.feedforward.brake_deadband_pct,
        feedforward_params(cfg),
    )
    return RunResult(
        label=label, csv_path=csv_path, n_rows=len(rows), reached_s=reached,
        completed=completed, kpi=kpi, pedal=pedal, command_rate_p95=command_rate_p95(rows),
        rows=rows,
    )


def rank(results: list[RunResult]) -> list[RunResult]:
    return sorted(results, key=lambda r: r.rank_key)


def summary_table(results: list[RunResult]) -> str:
    header = (
        "| 順位 | ラベル | 走破 | 到達[s] | 最大逸脱[km/h] | \\|偏差\\|p95[km/h] | "
        "符号反転(窓内最大) | ペダル切替[回] | 指令p95[%/s] |"
    )
    sep = "|---|---|---|---|---|---|---|---|---|"
    lines = [header, sep]
    for i, r in enumerate(rank(results), start=1):
        lines.append(
            f"| {i} | {r.label} | {'○' if r.completed else '×'} | {r.reached_s:.1f} | "
            f"{r.kpi.max_abs_kmh:.2f} | {r.kpi.p95_kmh:.2f} | {r.kpi.reversal_max_per_window} | "
            f"{r.pedal.switches} | {r.command_rate_p95:.2f} |"
        )
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────
# 既定ラベル（--labels 省略時。CSV の candidate 列 → ファイル名の順で決める）
# ─────────────────────────────────────────────────────────────────────


def candidate_label_from_csv(csv_path: Path) -> str | None:
    """MODE_DRIVE 行の candidate 列を見る。ちょうど 1 種類の空でない値なら返し、
    列が無い・空・複数種が混ざっているなら None（呼び出し側でファイル名にフォールバック）。

    ModeRow は candidate を持たない（手順 3/5/7/9 の順位判定に不要なため）ので、
    ModeRow を広げるのではなく CSV を直接読む（最小限の変更）。
    """
    values: set[str] = set()
    with csv_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("section") != SECTION_MODE_DRIVE:
                continue
            v = (row.get("candidate") or "").strip()
            if v:
                values.add(v)
    return next(iter(values)) if len(values) == 1 else None


def default_label(csv_path: Path) -> str:
    return candidate_label_from_csv(csv_path) or csv_path.stem


# ─────────────────────────────────────────────────────────────────────
# 候補ごとの集計（同じラベルが 2 本以上あるとき）
# ─────────────────────────────────────────────────────────────────────


@dataclass
class RunGroup:
    label: str
    runs: list[RunResult]


def group_by_label(results: Sequence[RunResult]) -> list[RunGroup]:
    """ラベルごとにまとめる（最初に出てきた順）。"""
    groups: dict[str, RunGroup] = {}
    order: list[str] = []
    for r in results:
        if r.label not in groups:
            groups[r.label] = RunGroup(r.label, [])
            order.append(r.label)
        groups[r.label].runs.append(r)
    return [groups[label] for label in order]


def group_sort_key(g: RunGroup) -> tuple[float, ...]:
    """RunResult.rank_key と同じ並びを中央値に当てる。走破しなかった本がある候補は下、
    走破しなかった候補どうしでは走破本数が少ないほど下。"""
    n_completed = sum(1 for r in g.runs if r.completed)
    all_completed = n_completed == len(g.runs)
    med = [float(np.median(vals)) for vals in zip(*(r.rank_key for r in g.runs), strict=True)]
    return (
        0.0 if all_completed else 1.0,
        0.0 if all_completed else -float(n_completed),
        med[2], med[3], med[4], med[5], med[6],
    )


def rank_groups(groups: Sequence[RunGroup]) -> list[RunGroup]:
    return sorted(groups, key=group_sort_key)


def _med_range(values: Sequence[float], fmt: str) -> str:
    if len(values) == 1:
        return fmt.format(values[0])
    return (
        f"{fmt.format(float(np.median(values)))}"
        f"（{fmt.format(min(values))}〜{fmt.format(max(values))}）"
    )


def aggregate_table(groups: Sequence[RunGroup]) -> str:
    header = (
        "| ラベル | 本数 | 走破本数 | 最大逸脱[km/h] | \\|偏差\\|p95[km/h] | "
        "符号反転(窓内最大) | ペダル切替[回] | 指令p95[%/s] |"
    )
    sep = "|---|---|---|---|---|---|---|---|"
    lines = [header, sep]
    for g in rank_groups(groups):
        n_completed = sum(1 for r in g.runs if r.completed)
        lines.append(
            f"| {g.label} | {len(g.runs)} | {n_completed} | "
            f"{_med_range([r.kpi.max_abs_kmh for r in g.runs], '{:.2f}')} | "
            f"{_med_range([r.kpi.p95_kmh for r in g.runs], '{:.2f}')} | "
            f"{_med_range([float(r.kpi.reversal_max_per_window) for r in g.runs], '{:.0f}')} | "
            f"{_med_range([float(r.pedal.switches) for r in g.runs], '{:.0f}')} | "
            f"{_med_range([r.command_rate_p95 for r in g.runs], '{:.2f}')} |"
        )
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────
# 基準車速帯別の偏差（常に出す）
# ─────────────────────────────────────────────────────────────────────


#: 基準車速の帯 (下限, 上限, 表示名)。上限は含まない（[lo, hi)）。最後の帯だけ上限なし
SPEED_BANDS: tuple[tuple[float, float, str], ...] = (
    (0.0, 40.0, "0〜40"),
    (40.0, 80.0, "40〜80"),
    (80.0, 120.0, "80〜120"),
    (120.0, float("inf"), "120〜"),
)


@dataclass(frozen=True)
class BandStat:
    n: int
    mean_kmh: float | None  # 平均偏差（実車速 − 基準車速）。帯にデータが無ければ None
    p95_abs_kmh: float | None  # |偏差| の p95


def compute_band_stats(rows: Sequence[ModeRow]) -> dict[str, BandStat]:
    """基準車速の帯ごとに、偏差（実車速 − 基準車速）の平均と |偏差| の p95 を出す。"""
    out: dict[str, BandStat] = {}
    for lo, hi, name in SPEED_BANDS:
        dev = [r.deviation_kmh for r in rows if lo <= r.ref_kmh < hi]
        if not dev:
            out[name] = BandStat(0, None, None)
            continue
        arr = np.asarray(dev, dtype=float)
        out[name] = BandStat(
            n=len(dev), mean_kmh=float(np.mean(arr)),
            p95_abs_kmh=float(np.percentile(np.abs(arr), 95)),
        )
    return out


def median_band_stats(stats_list: Sequence[dict[str, BandStat]]) -> dict[str, BandStat]:
    """同じラベルの複数本ぶんの帯別統計から、帯ごとの中央値を出す（データが無い本は除く）。"""
    out: dict[str, BandStat] = {}
    for _, _, name in SPEED_BANDS:
        means = [m for s in stats_list if (m := s[name].mean_kmh) is not None]
        p95s = [q for s in stats_list if (q := s[name].p95_abs_kmh) is not None]
        if not means:
            out[name] = BandStat(0, None, None)
        else:
            out[name] = BandStat(
                n=len(means), mean_kmh=float(np.median(means)), p95_abs_kmh=float(np.median(p95s))
            )
    return out


def _band_cell(stat: BandStat) -> str:
    if stat.mean_kmh is None:
        return "—"
    return f"{stat.mean_kmh:+.2f} / {stat.p95_abs_kmh:.2f}"


def band_table(results: Sequence[RunResult]) -> str:
    """行 = 本（同じラベルが 2 本以上なら末尾に中央値の行）、列 = 基準車速の帯。

    セルは「平均偏差 / |偏差|p95」（km/h）。帯にデータが無ければ「—」。
    """
    header = "| 本 | " + " | ".join(f"{name}[km/h]" for _, _, name in SPEED_BANDS) + " |"
    sep = "|" + "|".join("---" for _ in range(len(SPEED_BANDS) + 1)) + "|"
    lines = [header, sep]
    for g in group_by_label(results):
        multi = len(g.runs) >= 2
        per_run_stats = [compute_band_stats(r.rows) for r in g.runs]
        for i, (r, stats) in enumerate(zip(g.runs, per_run_stats, strict=True), start=1):
            row_label = f"{r.label} #{i}" if multi else r.label
            cells = " | ".join(_band_cell(stats[name]) for _, _, name in SPEED_BANDS)
            lines.append(f"| {row_label} | {cells} |")
        if multi:
            med_stats = median_band_stats(per_run_stats)
            cells = " | ".join(_band_cell(med_stats[name]) for _, _, name in SPEED_BANDS)
            lines.append(f"| {g.label} 中央値 | {cells} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("csvs", nargs="+", type=Path, help="比べるモード走行 CSV（2 本以上）")
    ap.add_argument(
        "--labels", nargs="*", default=None,
        help="表示ラベル（CSV と同じ数。省略時は CSV の candidate 列、無ければファイル名）",
    )
    ap.add_argument(
        "--duration", type=float, default=1800.0, help="モードの総時間 [s]（走破判定。既定 WLTP）"
    )
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    args = ap.parse_args(argv)

    if args.labels is not None and len(args.labels) != len(args.csvs):
        ap.error("--labels は --csv と同じ数だけ指定してください")
    labels = args.labels or [default_label(p) for p in args.csvs]

    cfg = load_config(args.config)
    results = [
        evaluate_run(path, label, cfg, args.duration)
        for path, label in zip(args.csvs, labels, strict=True)
    ]
    print(summary_table(results))
    winner = rank(results)[0]
    print(f"\n1 位: {winner.label}（{winner.csv_path}）")

    groups = group_by_label(results)
    if any(len(g.runs) >= 2 for g in groups):
        print("\n## 候補ごとの集計\n")
        print(aggregate_table(groups))
        top = rank_groups(groups)[0]
        print(f"\n1 位（中央値）: {top.label}")

    print("\n## 基準車速帯別の偏差\n")
    print(band_table(results))
    print(
        "\n各セル = 平均偏差（実車速 − 基準車速） / |偏差| p95 [km/h]。"
        "「—」= その帯に行が無い。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
