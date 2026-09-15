"""A7（実開度 PNOW とデバイスステータスの記録）の走行ログを確かめる。

車両・アクチュエータには触らない。A7 以降の走行ログ CSV（手順 1 からすべての区間で同じ列）を
1 本読み、表（Markdown）をターミナルに出し、図を --out に保存する。

    .venv/bin/python -m tests.research.debug_a7 \
        --csv tests/research/results/drive_log_real_<日時>.csv

出すもの（KAIZEN 表5-5 順6 の関門と、A7 で取れるようになったもの）:
    gate      関門: 1 周期の処理時間（cycle_ms）がパターン走行 100ms・モード走行 50ms を超えないか。
              実開度・ステータスが読めなかった行・アラームコードが 0 以外の行
    cycle     区間ごとの cycle_ms の平均 / p95 / 最大と、周期を超えた行
    tracking  区間 × ペダル × 指令の動き（踏み込み / 戻し / 保持）ごとの「実開度 − 指令」[%]
    lag       指令が 1 行で STEP_PCT 以上変わった所から、実開度が指令の TOL_PCT 以内に
              入るまでの時間（次に指令が変わるまでに入らなければ未到達）
    status    区間 × 軸ごとの サーボ OFF の行・移動中の割合・位置決め完了の割合・アラームコード
    figure    指令と実開度の差がいちばん大きかった所の前後の 指令 / 実開度 / 車速
"""

from __future__ import annotations

import argparse
import csv
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib

from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.debug_process23 import md_table
from tests.research.drive_log import (
    SECTION_DECEL_TO_STOP,
    SECTION_MODE_DRIVE,
    SECTION_PATTERN_DRIVE,
    SECTION_PEDAL_SEARCH,
    SECTION_PRE_DRIVE_CHECK,
    actual_opening,
    cmd_opening,
)
from tests.research.live_plot import COLOR_ACCEL, COLOR_ACTUAL, COLOR_BRAKE, FONT_FAMILY
from tests.research.pattern_loop import PATTERN_LOOP_INTERVAL_S

SECTIONS = (
    SECTION_PRE_DRIVE_CHECK, SECTION_PEDAL_SEARCH, SECTION_PATTERN_DRIVE, SECTION_MODE_DRIVE,
    SECTION_DECEL_TO_STOP,
)
AXES = (("accel", "アクセル"), ("brake", "ブレーキ"))
STEP_PCT = 2.0  # 指令が 1 行でこれ以上変わったら「指令が変わった」とみなす [%]
TOL_PCT = 0.5  # 実開度が指令のこの範囲に入ったら「追いついた」とみなす [%]
HOLD_TOL_PCT = 0.05  # 指令の変化がこれ未満なら「保持」[%]
FIG_WINDOW_S = 5.0  # 図に出す前後の秒数

MOTION_PRESS = "踏み込み"
MOTION_RELEASE = "戻し"
MOTION_HOLD = "保持"
MOTIONS = (MOTION_PRESS, MOTION_RELEASE, MOTION_HOLD)


@dataclass(frozen=True)
class AxisRow:
    cmd: float  # 最終指令 [%]
    actual: float | None  # 実開度 [%]（読めなかった行は None）
    servo_on: bool | None
    moving: bool | None
    pos_done: bool | None
    alarm_code: int | None


@dataclass(frozen=True)
class Row:
    t: float  # elapsed_s
    section: str
    pattern: str
    phase: str
    v: float
    cycle_ms: float | None
    accel: AxisRow
    brake: AxisRow

    def axis(self, name: str) -> AxisRow:
        return self.accel if name == "accel" else self.brake


def _flag(text: str | None) -> bool | None:
    return None if text in (None, "") else text == "1"


def _axis_row(r: dict[str, str], axis: str) -> AxisRow:
    code = r.get(f"{axis}_alarm_code", "")
    return AxisRow(
        cmd=cmd_opening(r, axis),
        actual=actual_opening(r, axis),
        servo_on=_flag(r.get(f"{axis}_servo_on")),
        moving=_flag(r.get(f"{axis}_moving")),
        pos_done=_flag(r.get(f"{axis}_pos_done")),
        alarm_code=int(code) if code else None,
    )


def read_rows(path: Path) -> list[Row]:
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            rows.append(Row(
                t=float(r["elapsed_s"]),
                section=r["section"],
                pattern=r.get("pattern", ""),
                phase=r.get("phase", ""),
                v=float(r["actual_speed_kmh"]),
                cycle_ms=float(r["cycle_ms"]) if r.get("cycle_ms") else None,
                accel=_axis_row(r, "accel"),
                brake=_axis_row(r, "brake"),
            ))
    return rows


def by_section(rows: Sequence[Row]) -> dict[str, list[Row]]:
    out: dict[str, list[Row]] = {}
    for r in rows:
        out.setdefault(r.section, []).append(r)
    return out


def _pct(values: Sequence[float], q: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, max(0, math.ceil(q * len(ordered)) - 1))]


def _stats_text(values: Sequence[float], digits: int = 1) -> list[str]:
    if not values:
        return ["—", "—", "—"]
    return [f"{sum(values) / len(values):.{digits}f}", f"{_pct(values, 0.95):.{digits}f}",
            f"{max(values):.{digits}f}"]


# ── 周期 ─────────────────────────────────────────────────────────────


def section_periods_ms(control_interval_s: float, csv_interval_s: float) -> dict[str, float]:
    """区間ごとの周期 [ms]。パターン走行・モード走行は制御周期、他はサンプラーの間隔。"""
    periods = {s: 1000.0 * csv_interval_s for s in SECTIONS}
    periods[SECTION_PATTERN_DRIVE] = 1000.0 * PATTERN_LOOP_INTERVAL_S
    periods[SECTION_MODE_DRIVE] = 1000.0 * control_interval_s
    return periods


@dataclass(frozen=True)
class CycleStat:
    section: str
    rows: int
    measured: list[float]
    period_ms: float

    @property
    def over(self) -> int:
        return sum(1 for ms in self.measured if ms > self.period_ms)


def cycle_stats(rows: Sequence[Row], periods_ms: dict[str, float]) -> list[CycleStat]:
    groups = by_section(rows)
    return [
        CycleStat(s, len(groups[s]), [r.cycle_ms for r in groups[s] if r.cycle_ms is not None],
                  periods_ms.get(s, math.inf))
        for s in SECTIONS if s in groups
    ]


def cycle_table(stats: Sequence[CycleStat]) -> str:
    body = [
        [s.section, str(s.rows), str(len(s.measured)), *_stats_text(s.measured),
         f"{s.period_ms:.0f}", str(s.over)]
        for s in stats
    ]
    return md_table(["区間", "行", "cycle_ms のある行", "平均[ms]", "p95[ms]", "最大[ms]",
                     "周期[ms]", "周期を超えた行"], body)


# ── 指令と実開度 ───────────────────────────────────────────────────────


def _motion(prev_cmd: float, cmd: float) -> str:
    if cmd - prev_cmd > HOLD_TOL_PCT:
        return MOTION_PRESS
    if prev_cmd - cmd > HOLD_TOL_PCT:
        return MOTION_RELEASE
    return MOTION_HOLD


@dataclass(frozen=True)
class TrackingStat:
    section: str
    axis: str
    motion: str
    errors: list[float]  # 実開度 − 指令 [%]


def tracking_stats(rows: Sequence[Row]) -> list[TrackingStat]:
    out = []
    for section, group in by_section(rows).items():
        for axis, _ in AXES:
            errs: dict[str, list[float]] = {m: [] for m in MOTIONS}
            for prev, cur in zip(group, group[1:], strict=False):
                a = cur.axis(axis)
                if a.actual is None:
                    continue
                errs[_motion(prev.axis(axis).cmd, a.cmd)].append(a.actual - a.cmd)
            out += [TrackingStat(section, axis, m, errs[m]) for m in MOTIONS if errs[m]]
    order = {s: i for i, s in enumerate(SECTIONS)}
    return sorted(out, key=lambda s: (order.get(s.section, 99), s.axis != "accel",
                                      MOTIONS.index(s.motion)))


def tracking_table(stats: Sequence[TrackingStat]) -> str:
    names = dict(AXES)
    body = []
    for s in stats:
        body.append([
            s.section, names[s.axis], s.motion, str(len(s.errors)),
            f"{sum(s.errors) / len(s.errors):+.2f}",
            *_stats_text([abs(e) for e in s.errors], 2),
        ])
    return md_table(["区間", "ペダル", "指令の動き", "行", "平均（実開度−指令）[%]",
                     "|差| 平均[%]", "|差| p95[%]", "|差| 最大[%]"], body)


@dataclass(frozen=True)
class Step:
    section: str
    axis: str
    t: float
    cmd_from: float
    cmd_to: float
    lag_s: float | None  # 追いつかなければ None


def find_steps(rows: Sequence[Row]) -> list[Step]:
    """指令が 1 行で STEP_PCT 以上変わった所と、実開度が追いつくまでの時間。"""
    steps = []
    for section, group in by_section(rows).items():
        for axis, _ in AXES:
            for i in range(1, len(group)):
                prev, cur = group[i - 1].axis(axis), group[i].axis(axis)
                if abs(cur.cmd - prev.cmd) < STEP_PCT or cur.actual is None:
                    continue
                lag = None
                for later in group[i:]:
                    a = later.axis(axis)
                    if abs(a.cmd - cur.cmd) > HOLD_TOL_PCT:
                        break  # 追いつく前に次の指令が来た
                    if a.actual is not None and abs(a.actual - a.cmd) <= TOL_PCT:
                        lag = later.t - group[i].t
                        break
                steps.append(Step(section, axis, group[i].t, prev.cmd, cur.cmd, lag))
    return steps


def lag_table(steps: Sequence[Step]) -> str:
    names = dict(AXES)
    groups: dict[tuple[str, str, str], list[Step]] = {}
    for s in steps:
        motion = MOTION_PRESS if s.cmd_to > s.cmd_from else MOTION_RELEASE
        groups.setdefault((s.section, s.axis, motion), []).append(s)
    order = {s: i for i, s in enumerate(SECTIONS)}
    body = []
    for (section, axis, motion), group in sorted(
        groups.items(), key=lambda kv: (order.get(kv[0][0], 99), kv[0][1] != "accel", kv[0][2])
    ):
        lags = [s.lag_s for s in group if s.lag_s is not None]
        sizes = [abs(s.cmd_to - s.cmd_from) for s in group]
        body.append([
            section, names[axis], motion, str(len(group)),
            f"{sum(sizes) / len(sizes):.1f}",
            *(["—", "—", "—"] if not lags else
              [f"{_pct(lags, 0.5):.2f}", f"{_pct(lags, 0.95):.2f}", f"{max(lags):.2f}"]),
            str(len(group) - len(lags)),
        ])
    return md_table(["区間", "ペダル", "指令の向き", "回数", "指令の変化 平均[%]",
                     "遅れ 中央値[s]", "遅れ p95[s]", "遅れ 最大[s]", "未到達"], body)


# ── ステータス ─────────────────────────────────────────────────────────


@dataclass(frozen=True)
class StatusStat:
    section: str
    axis: str
    rows: int
    read_rows: int  # ステータスが読めた行
    servo_off: int
    moving: int
    pos_done: int
    alarm_codes: dict[int, int]  # 0 以外のアラームコード → 行数


def status_stats(rows: Sequence[Row]) -> list[StatusStat]:
    groups = by_section(rows)
    out = []
    for section in SECTIONS:
        if section not in groups:
            continue
        for axis, _ in AXES:
            items = [r.axis(axis) for r in groups[section]]
            read = [a for a in items if a.servo_on is not None]
            codes: dict[int, int] = {}
            for a in items:
                if a.alarm_code:
                    codes[a.alarm_code] = codes.get(a.alarm_code, 0) + 1
            out.append(StatusStat(
                section, axis, len(items), len(read),
                sum(1 for a in read if not a.servo_on),
                sum(1 for a in read if a.moving),
                sum(1 for a in read if a.pos_done),
                codes,
            ))
    return out


def status_table(stats: Sequence[StatusStat]) -> str:
    names = dict(AXES)

    def share(n: int, total: int) -> str:
        return "—" if total == 0 else f"{100.0 * n / total:.1f}%"

    body = [
        [s.section, names[s.axis], str(s.rows), str(s.rows - s.read_rows), str(s.servo_off),
         share(s.moving, s.read_rows), share(s.pos_done, s.read_rows),
         "なし" if not s.alarm_codes else
         "・".join(f"0x{c:03X}（{n} 行）" for c, n in sorted(s.alarm_codes.items()))]
        for s in stats
    ]
    return md_table(["区間", "軸", "行", "読めなかった行", "サーボ OFF の行", "移動中",
                     "位置決め完了", "アラームコード"], body)


# ── 関門 ─────────────────────────────────────────────────────────────


def a7_gate_table(cycles: Sequence[CycleStat], statuses: Sequence[StatusStat]) -> str:
    body = []
    for c in cycles:
        if c.section not in (SECTION_PATTERN_DRIVE, SECTION_MODE_DRIVE):
            continue
        if not c.measured:
            body.append([f"{c.section} の 1 周期の処理時間", "cycle_ms の行なし", "—"])
            continue
        p95 = _pct(c.measured, 0.95)
        body.append([
            f"{c.section} の 1 周期の処理時間 p95 ≤ {c.period_ms:.0f}ms",
            f"p95 {p95:.1f} ms・最大 {max(c.measured):.1f} ms（超えた行 {c.over}）",
            "○" if p95 <= c.period_ms else "×",
        ])
    unread = sum(s.rows - s.read_rows for s in statuses)
    body.append(["実開度・ステータスが読めなかった行（全区間・両軸）", str(unread),
                 "○" if unread == 0 else "記録"])
    alarms = sum(sum(s.alarm_codes.values()) for s in statuses)
    body.append(["アラームコードが 0 以外の行（全区間・両軸）", str(alarms),
                 "○" if alarms == 0 else "×"])
    return md_table(["項目", "実測", "判定"], body)


# ── 図 ─────────────────────────────────────────────────────────────


def worst_point(rows: Sequence[Row]) -> tuple[Row, str] | None:
    """実開度と指令の差がいちばん大きかった行と軸（パターン走行・モード走行の中から）。"""
    best: tuple[float, Row, str] | None = None
    for r in rows:
        if r.section not in (SECTION_PATTERN_DRIVE, SECTION_MODE_DRIVE):
            continue
        for axis, _ in AXES:
            a = r.axis(axis)
            if a.actual is None:
                continue
            err = abs(a.actual - a.cmd)
            if best is None or err > best[0]:
                best = (err, r, axis)
    return None if best is None else (best[1], best[2])


def fig_tracking(out: Path, rows: Sequence[Row]) -> Path | None:
    from matplotlib.figure import Figure  # noqa: PLC0415

    point = worst_point(rows)
    if point is None:
        return None
    center, _ = point
    window = [r for r in rows if abs(r.t - center.t) <= FIG_WINDOW_S]
    matplotlib.rcParams["font.family"] = FONT_FAMILY
    fig = Figure(figsize=(12.0, 7.0), dpi=100, layout="constrained")
    axs: Any = fig.subplots(2, 1, sharex=True)
    t = [r.t for r in window]
    axs[0].plot(t, [r.v for r in window], color=COLOR_ACTUAL, linewidth=1.8, label="実車速")
    axs[0].set_ylabel("車速 [km/h]")
    axs[0].set_title(f"指令と実開度の差が最大の所（{center.section} {center.pattern} "
                     f"t={center.t:.1f}s の前後 {FIG_WINDOW_S:g}s）")
    for axis, name in AXES:
        color = COLOR_ACCEL if axis == "accel" else COLOR_BRAKE
        axs[1].plot(t, [r.axis(axis).cmd for r in window], color=color, linestyle="--",
                    linewidth=1.5, label=f"{name} 指令")
        axs[1].plot(t, [math.nan if r.axis(axis).actual is None else r.axis(axis).actual
                        for r in window],
                    color=color, linewidth=1.8, label=f"{name} 実開度")
    axs[1].set_ylabel("開度 [%]")
    axs[1].set_xlabel("走行開始からの時間 [s]")
    for ax in axs:
        ax.grid(True, alpha=0.3)
        ax.axvline(center.t, color="gray", linewidth=0.8)
        ax.legend(loc="best")
    out.mkdir(parents=True, exist_ok=True)
    path = out / "a7_tracking.png"
    fig.savefig(path)
    return path


# ─────────────────────────────────────────────────────────────────────


def run(csv_path: Path, cfg_path: Path, out: Path) -> int:
    cfg = load_config(cfg_path)
    rows = read_rows(csv_path)
    print(f"CSV: {csv_path}（{len(rows)} 行）")
    if not any(r.accel.actual is not None or r.brake.actual is not None for r in rows):
        print("実開度の列がありません（A7 より前の CSV）。指令だけの集計になります。")
    periods = section_periods_ms(cfg.control.loop_interval_s, cfg.output.csv_interval_s)
    cycles = cycle_stats(rows, periods)
    statuses = status_stats(rows)
    print("\n### 関門\n")
    print(a7_gate_table(cycles, statuses))
    print("\n### 1 周期の処理時間（区間ごと）\n")
    print(cycle_table(cycles))
    print(f"\n### 指令と実開度の差（指令の動きの判定: 前の行から {HOLD_TOL_PCT:g}% 以上）\n")
    print(tracking_table(tracking_stats(rows)))
    print(f"\n### 指令が変わってから実開度が追いつくまで（変化 ≥ {STEP_PCT:g}%・"
          f"追いついた = 差 ≤ {TOL_PCT:g}%）\n")
    print(lag_table(find_steps(rows)))
    print("\n### ステータス\n")
    print(status_table(statuses))
    path = fig_tracking(out, rows)
    print(f"\n図: {path}" if path else "\n図: 実開度の行が無いため作りません")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--csv", type=Path, required=True, help="A7 以降の走行ログ CSV")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument(
        "--out", type=Path, default=None, help="図の保存先（既定: CSV と同じ名前のフォルダ）"
    )
    args = ap.parse_args(argv)
    out = args.out or args.csv.with_name(f"{args.csv.stem}_a7")
    return run(args.csv, args.config, out)


if __name__ == "__main__":
    raise SystemExit(main())
