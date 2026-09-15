"""A6（打ち切りの見直し＋ガバナーの解除＋全パターン後の停車復帰）の手順 2 を、前の手順 2 と比べる。

車両・アクチュエータには触らない。手順 2 の走行ログ CSV（PATTERN_DRIVE 区間）を 2 本読み、
表（Markdown）をターミナルに出し、図を --out に保存する。

    .venv/bin/python -m tests.research.debug_a6 \
        --csv tests/research/results/drive_log_real_<日時>.csv

出すもの（KAIZEN 表5-5 順3 の関門と、A6 で変えたところ）:
    gate      関門 3 つ: 各パターンの開始車速・加速を終えた車速（cap との差）・
              所要時間（900s との差）
    phase     系統 × フェーズの所要時間と打ち切りに張り付いた本数（kaizen.phase_stats。
              上限は A6 の既定値なので新しい CSV だけに出す）
    governor  系統 × フェーズごとに、開度を最大値から下げていた時間・下げたまま終わった本数・
              governor_active の秒数（前の手順 2 は列が無いので「下げた」だけで数える）
    rows      学習に効く行（開度 ≥ 不感帯）を 5% 刻みで。うちガバナーで下げていた行
    figure    ACCEL_SWEEP の停車復帰と COAST_DOWN の加速を新旧で重ねた車速・開度
"""

from __future__ import annotations

import argparse
import csv
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

from tests.research.config import load_config
from tests.research.debug_process23 import md_table
from tests.research.drive_log import SECTION_PATTERN_DRIVE, cmd_opening
from tests.research.kaizen import phase_stats, phase_table
from tests.research.live_plot import COLOR_ACCEL, COLOR_ACTUAL, COLOR_BRAKE, FONT_FAMILY
from tests.research.pattern_loop import STOP_SPEED_KMH, PatternLoopConfig

RESULTS = Path("tests/research/results")
DEFAULT_CONFIG_PATH = Path("tests/research/config_testVehicle.yaml")
OLD_CSV = RESULTS / "drive_log_real_20260913_071502.csv"  # 9/13 07:15 手順 2（A6 前）
DRIVING_KINDS = ("ACCEL_SWEEP", "BRAKE_HOLD", "COAST_DOWN", "CRUISE_TRIM")
LOWERED_TOL_PCT = 0.5  # 最大開度よりこれ以上浅ければ「下げていた」とみなす [%]
BIN_PCT = 5.0  # 学習行の開度の刻み [%]
DT_S = 0.1  # CSV の行間隔 [s]


@dataclass(frozen=True)
class Row:
    t: float
    pattern: str
    phase: str
    v: float
    accel: float
    brake: float
    governor: bool

    @property
    def kind(self) -> str:
        return self.pattern.split(":", 1)[-1]


@dataclass(frozen=True)
class Block:
    """同じ pattern・phase が続いた行のまとまり。"""

    rows: tuple[Row, ...]

    @property
    def pattern(self) -> str:
        return self.rows[0].pattern

    @property
    def kind(self) -> str:
        return self.rows[0].kind

    @property
    def phase(self) -> str:
        return self.rows[0].phase

    @property
    def duration_s(self) -> float:
        return self.rows[-1].t - self.rows[0].t

    def opening(self, r: Row) -> float:
        """このフェーズで動かすペダルの開度。

        CRUISE_HOLD は 2026-09-14 定速階段（段2）の追加フェーズで、CRUISE_TRIM と同じく
        アクセル側を動かす。
        """
        return r.accel if self.phase in ("DRIVE_ACCEL", "CRUISE_TRIM", "CRUISE_HOLD") else r.brake

    def lowered_flags(self) -> list[bool]:
        """各行が「それまでの最大開度から下げていた」か（ガバナーの頭打ちの跡）。"""
        peak = 0.0
        out = []
        for r in self.rows:
            x = self.opening(r)
            peak = max(peak, x)
            out.append(x < peak - LOWERED_TOL_PCT)
        return out


def read_pattern_rows(path: Path) -> list[Row]:
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r["section"] != SECTION_PATTERN_DRIVE:
                continue
            rows.append(Row(
                t=float(r["elapsed_s"]),
                pattern=r["pattern"],
                phase=r["phase"],
                v=float(r["actual_speed_kmh"]),
                accel=cmd_opening(r, "accel"),
                brake=cmd_opening(r, "brake"),
                governor=r.get("governor_active") == "1",
            ))
    return rows


def blocks_of(rows: Sequence[Row]) -> list[Block]:
    out: list[Block] = []
    start = 0
    for i in range(1, len(rows) + 1):
        if i < len(rows) and (rows[i].pattern, rows[i].phase) == (
            rows[start].pattern, rows[start].phase
        ):
            continue
        out.append(Block(tuple(rows[start:i])))
        start = i
    return out


# ─────────────────────────────────────────────────────────────────────
# 関門
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class PatternGate:
    pattern: str
    v_start: float  # パターンの最初の行の車速
    v_accel_end: float | None  # DRIVE_ACCEL の最後の行の車速（無ければ None）
    accel_s: float | None
    v_max: float
    stop_return_s: float | None  # 停車復帰（DRIVE_BRAKE）の所要
    v_end: float  # パターンの最後の行の車速


def pattern_gates(rows: Sequence[Row]) -> list[PatternGate]:
    blocks = blocks_of(rows)
    out = []
    for name in dict.fromkeys(b.pattern for b in blocks):
        mine = [b for b in blocks if b.pattern == name]
        if mine[0].kind not in DRIVING_KINDS:
            continue
        accel = next((b for b in mine if b.phase == "DRIVE_ACCEL"), None)
        brakes = [b for b in mine if b.phase == "DRIVE_BRAKE"]
        out.append(PatternGate(
            pattern=name,
            v_start=mine[0].rows[0].v,
            v_accel_end=accel.rows[-1].v if accel else None,
            accel_s=accel.duration_s if accel else None,
            v_max=max(r.v for b in mine for r in b.rows),
            stop_return_s=brakes[-1].duration_s if brakes else None,
            v_end=mine[-1].rows[-1].v,
        ))
    return out


def gate_table(gates: Sequence[PatternGate], cap_kmh: float) -> str:
    def opt(x: float | None, fmt: str) -> str:
        return "—" if x is None else format(x, fmt)

    body = [[
        g.pattern, f"{g.v_start:.1f}", opt(g.v_accel_end, ".1f"),
        opt(None if g.v_accel_end is None else g.v_accel_end - cap_kmh, "+.1f"),
        opt(g.accel_s, ".1f"), f"{g.v_max:.1f}", opt(g.stop_return_s, ".1f"), f"{g.v_end:.2f}",
    ] for g in gates]
    return md_table(
        ["パターン", "開始車速[km/h]", "加速を終えた車速[km/h]", "cap との差[km/h]", "加速[s]",
         "最高速[km/h]", "停車復帰[s]", "終わりの車速[km/h]"],
        body,
    )


def gate_summary(rows: Sequence[Row], gates: Sequence[PatternGate], timeout_s: float) -> str:
    span = rows[-1].t - rows[0].t if rows else 0.0
    starts = [g.v_start for g in gates]
    ends = [g.v_accel_end for g in gates if g.v_accel_end is not None]
    stopped = sum(1 for g in gates if g.v_end <= STOP_SPEED_KMH)
    body = [
        ["パターン走行の所要時間", f"{span:.1f}s",
         f"learning.timeout_s {timeout_s:.0f}s に対し余裕 {timeout_s - span:+.1f}s"],
        ["運転パターンの開始車速", f"{min(starts):.2f}〜{max(starts):.2f} km/h" if starts else "—",
         f"{len(starts)} 本"],
        ["加速を終えた車速", f"{min(ends):.1f}〜{max(ends):.1f} km/h" if ends else "—",
         f"{len(ends)} 本"],
        ["停車して終わった運転パターン", f"{stopped} / {len(gates)} 本",
         f"車速 ≤ {STOP_SPEED_KMH:g} km/h"],
    ]
    return md_table(["項目", "値", "補足"], body)


# ─────────────────────────────────────────────────────────────────────
# ガバナー・学習行
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class GovernorStat:
    kind: str
    phase: str
    count: int
    lowered_s: float  # 最大開度から下げていた時間の合計
    ended_lowered: int  # 下げたまま終わった本数
    governor_s: float  # governor_active の秒数（列が無い CSV は 0）
    min_lowered: float | None  # 下げていた行の最低開度（下げた行が無ければ None）


def governor_stats(rows: Sequence[Row]) -> list[GovernorStat]:
    groups: dict[tuple[str, str], list[Block]] = {}
    for b in blocks_of(rows):
        if b.phase in ("DRIVE_ACCEL", "DRIVE_BRAKE", "BRAKE_HOLD"):
            groups.setdefault((b.kind, b.phase), []).append(b)
    out = []
    for (kind, phase), bs in sorted(groups.items()):
        lowered_s = ended = 0.0
        gov_s = 0.0
        lows: list[float] = []
        for b in bs:
            flags = b.lowered_flags()
            lowered_s += sum(flags) * DT_S
            ended += int(flags[-1])
            gov_s += sum(r.governor for r in b.rows) * DT_S
            lows += [b.opening(r) for r, f in zip(b.rows, flags, strict=True) if f]
        out.append(GovernorStat(
            kind, phase, len(bs), lowered_s, int(ended), gov_s, min(lows) if lows else None
        ))
    return out


def governor_table(stats_old: Sequence[GovernorStat], stats_new: Sequence[GovernorStat]) -> str:
    old = {(s.kind, s.phase): s for s in stats_old}
    new = {(s.kind, s.phase): s for s in stats_new}
    body = []
    for key in sorted(set(old) | set(new)):
        cells = [key[0], key[1]]
        for s in (old.get(key), new.get(key)):
            if s is None:
                cells += ["—", "—", "—"]
            else:
                low = "—" if s.min_lowered is None else f"{s.min_lowered:.1f}"
                cells += [f"{s.ended_lowered} / {s.count}", f"{s.lowered_s:.1f}", low]
        g = new.get(key)
        cells.append("—" if g is None else f"{g.governor_s:.1f}")
        body.append(cells)
    return md_table(
        ["系統", "フェーズ",
         "前: 下げたまま終わった本数", "前: 下げていた[s]", "前: 下げた最低開度[%]",
         "新: 下げたまま終わった本数", "新: 下げていた[s]", "新: 下げた最低開度[%]",
         "新: ガバナー作動[s]"],
        body,
    )


def effective_row_bins(
    rows: Sequence[Row], deadband_pct: float, *, accel: bool
) -> dict[int, tuple[int, int]]:
    """開度 ≥ 不感帯の行を BIN_PCT 刻みで数える。値は (行数, うちガバナーで下げていた行)。"""
    phase = "DRIVE_ACCEL" if accel else None
    out: dict[int, list[int]] = {}
    for b in blocks_of(rows):
        flags = b.lowered_flags()
        for r, f in zip(b.rows, flags, strict=True):
            x = r.accel if accel else r.brake
            if x < deadband_pct:
                continue
            lo = int(x // BIN_PCT * BIN_PCT)
            cell = out.setdefault(lo, [0, 0])
            cell[0] += 1
            driving_phase = b.phase == phase if accel else b.phase in ("DRIVE_BRAKE", "BRAKE_HOLD")
            cell[1] += int(f and driving_phase)
    return {k: (v[0], v[1]) for k, v in sorted(out.items())}


def rows_table(
    old: dict[int, tuple[int, int]], new: dict[int, tuple[int, int]], label: str
) -> str:
    body = []
    for lo in sorted(set(old) | set(new)):
        o, n = old.get(lo, (0, 0)), new.get(lo, (0, 0))
        body.append([f"{lo}〜{lo + BIN_PCT:g}", str(o[0]), str(o[1]), str(n[0]), str(n[1])])
    body.append(["**合計**", f"**{sum(v[0] for v in old.values())}**",
                 f"**{sum(v[1] for v in old.values())}**", f"**{sum(v[0] for v in new.values())}**",
                 f"**{sum(v[1] for v in new.values())}**"])
    return md_table(
        [f"{label}開度[%]", "前: 行数", "前: うち下げていた", "新: 行数", "新: うち下げていた"],
        body,
    )


# ─────────────────────────────────────────────────────────────────────
# 図
# ─────────────────────────────────────────────────────────────────────


def _first_block(rows: Sequence[Row], kind: str, phase: str) -> Block | None:
    return next((b for b in blocks_of(rows) if b.kind == kind and b.phase == phase), None)


def fig_compare(out: Path, old: Sequence[Row], new: Sequence[Row]) -> Path:
    from matplotlib.figure import Figure  # noqa: PLC0415

    matplotlib.rcParams["font.family"] = FONT_FAMILY
    fig = Figure(figsize=(12.0, 7.0), dpi=100, layout="constrained")
    axs: Any = fig.subplots(2, 2, sharex="col")
    panels = (("ACCEL_SWEEP", "DRIVE_BRAKE", "ブレーキ", True),
              ("COAST_DOWN", "DRIVE_ACCEL", "アクセル", False))
    for col, (kind, phase, pedal, is_brake) in enumerate(panels):
        color = COLOR_BRAKE if is_brake else COLOR_ACCEL
        for rows, label, style in ((old, "前（A6 前）", "--"), (new, "新（A6）", "-")):
            b = _first_block(rows, kind, phase)
            if b is None:
                continue
            t = [r.t - b.rows[0].t for r in b.rows]
            axs[0, col].plot(t, [r.v for r in b.rows], color=COLOR_ACTUAL, linestyle=style,
                             linewidth=1.8, label=f"実車速 {label}")
            axs[1, col].plot(t, [r.brake if is_brake else r.accel for r in b.rows], color=color,
                             linestyle=style, linewidth=1.8, label=f"{pedal}開度 {label}")
        axs[0, col].set_title(f"最初の {kind} の {phase}")
        axs[0, col].set_ylabel("車速 [km/h]")
        axs[1, col].set_ylabel("開度 [%]")
        axs[1, col].set_xlabel("フェーズ開始からの時間 [s]")
        for ax in axs[:, col]:
            ax.grid(True, alpha=0.3)
            if ax.get_legend_handles_labels()[0]:
                ax.legend(loc="best")
    out.mkdir(parents=True, exist_ok=True)
    path = out / "a6_compare.png"
    fig.savefig(path)
    return path


# ─────────────────────────────────────────────────────────────────────


def run(new_csv: Path, old_csv: Path, cfg_path: Path, out: Path) -> int:
    cfg = load_config(cfg_path)
    loop_cfg = PatternLoopConfig()
    cap = cfg.vehicle.max_speed_kmh * loop_cfg.accel_speed_cap_frac
    ff = cfg.feedforward
    old, new = read_pattern_rows(old_csv), read_pattern_rows(new_csv)
    print(f"前: {old_csv}（{len(old)} 行）\n新: {new_csv}（{len(new)} 行）")
    print(f"cap = {cap:.1f} km/h / 不感帯 アクセル {ff.accel_deadband_pct:g}% "
          f"ブレーキ {ff.brake_deadband_pct:g}%（{cfg_path} の値。前後で共通に使う）")

    for label, rows in (("前", old), ("新", new)):
        gates = pattern_gates(rows)
        print(f"\n### 関門（{label}）\n")
        print(gate_summary(rows, gates, cfg.learning.timeout_s))
        print()
        print(gate_table(gates, cap))
    print("\n### フェーズの所要時間（新。上限は A6 の既定値）\n")
    print(phase_table(phase_stats(new_csv)))

    print("\n### ガバナー（前後）\n")
    print(governor_table(governor_stats(old), governor_stats(new)))
    for label, accel, db in (("アクセル", True, ff.accel_deadband_pct),
                             ("ブレーキ", False, ff.brake_deadband_pct)):
        print(f"\n### 学習に効く{label}行（開度 ≥ 不感帯 {db:g}%）\n")
        print(rows_table(effective_row_bins(old, db, accel=accel),
                         effective_row_bins(new, db, accel=accel), label))
    path = fig_compare(out, old, new)
    print(f"\n図: {path}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--csv", type=Path, required=True, help="A6 の手順 2 の走行ログ CSV")
    ap.add_argument("--old", type=Path, default=OLD_CSV, help="比べる前の手順 2 の CSV")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument(
        "--out", type=Path, default=None, help="図の保存先（既定: CSV と同じ名前のフォルダ）"
    )
    args = ap.parse_args(argv)
    out = args.out or args.csv.with_name(f"{args.csv.stem}_a6")
    return run(args.csv, args.old, args.config, out)


if __name__ == "__main__":
    raise SystemExit(main())
