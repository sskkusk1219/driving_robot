#!/usr/bin/env python3
# ruff: noqa: E501
"""等価確認②: 走行 CSV を突き合わせて、src のランナーが tests/research の main と同じペダル指令・車速を出すか確かめる。

    .venv/bin/python scripts/check_runner_equivalence.py \\
        --tests A.csv B.csv --src C.csv [D.csv ...]

- ``--tests``: tests/research/main.py の走行（2 本以上。同じ条件で 2 回走らせたもの）。1 本目と 2 本目の差が
  「時刻のゆらぎだけで出る差」の物差しになる（スタブ車両に乱数は無く、ずれるのは壁時計で車速を積分することによる）。
- ``--src``: src.robot.run の走行（1 本以上）。
- 突き合わせるのはモード走行の行（section=MODE_DRIVE）を ``mode_time_s``（50 ms 刻み）で。
  列は accel_cmd_pct・brake_cmd_pct（ペダル指令）・actual_speed_kmh（車速）・ref_speed_kmh（基準車速。
  同じなら 0 差）。行の数・最大差・p95 差・平均を出す。
- 判断: tests 同士の差（ゆらぎ）と、tests–src の差を並べ、src の差がゆらぎの範囲（最大差・p95 差とも
  ゆらぎの 1.5 倍＋小さな余裕以内）なら「時刻のゆらぎで説明できる」とする。

tests も src も import しない（CSV を読むだけ）。
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path

COLUMNS = ("accel_cmd_pct", "brake_cmd_pct", "actual_speed_kmh", "ref_speed_kmh")
STEP_S = 0.05  # モード走行の制御周期
#: 判断の余裕: ゆらぎの 1.5 倍に、この絶対値を足したものまでをゆらぎ内とする
SLACK = {"accel_cmd_pct": 0.05, "brake_cmd_pct": 0.05, "actual_speed_kmh": 0.02, "ref_speed_kmh": 1e-6}


def load(path: Path) -> tuple[dict[int, dict[str, float]], dict[str, int]]:
    """MODE_DRIVE の行を mode_time_s（STEP_S 単位の整数）→ 列の値 にする。区間ごとの行数も返す。"""
    rows: dict[int, dict[str, float]] = {}
    counts: dict[str, int] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            counts[row["section"]] = counts.get(row["section"], 0) + 1
            if row["section"] != "MODE_DRIVE" or not row["mode_time_s"]:
                continue
            key = round(float(row["mode_time_s"]) / STEP_S)
            try:
                rows[key] = {c: float(row[c]) if row[c] != "" else float("nan") for c in COLUMNS}
            except KeyError as exc:
                raise SystemExit(f"{path} に列 {exc} がありません") from exc
    return rows, counts


def diff_stats(
    a: dict[int, dict[str, float]], b: dict[int, dict[str, float]]
) -> dict[str, tuple[int, float, float, float]]:
    """列ごとの (突き合わせた行数, 最大差, p95 差, 平均差)。"""
    out: dict[str, tuple[int, float, float, float]] = {}
    keys = sorted(a.keys() & b.keys())
    for col in COLUMNS:
        diffs = sorted(
            abs(a[k][col] - b[k][col])
            for k in keys
            if a[k][col] == a[k][col] and b[k][col] == b[k][col]
        )
        if not diffs:
            out[col] = (0, float("nan"), float("nan"), float("nan"))
            continue
        p95 = diffs[min(len(diffs) - 1, int(0.95 * len(diffs)))]
        out[col] = (len(diffs), diffs[-1], p95, statistics.fmean(diffs))
    return out


def print_pair(title: str, stats: dict[str, tuple[int, float, float, float]]) -> None:
    print(f"\n{title}")
    print(f"  {'列':<18}{'行数':>6}{'最大差':>10}{'p95差':>10}{'平均差':>10}")
    for col, (n, mx, p95, mean) in stats.items():
        print(f"  {col:<18}{n:>6}{mx:>10.4f}{p95:>10.4f}{mean:>10.4f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--tests", nargs="+", type=Path, required=True, help="tests 側の走行 CSV（2 本以上）")
    ap.add_argument("--src", nargs="+", type=Path, required=True, help="src のランナーの走行 CSV")
    args = ap.parse_args()
    if len(args.tests) < 2:
        raise SystemExit("--tests は 2 本以上（時刻のゆらぎの物差しに使う）")

    t_rows, s_rows = [], []
    print("区間ごとの行数（行数の違いは走行前チェック・停車保持の長さの違い）:")
    for label, paths, store in (("tests", args.tests, t_rows), ("src", args.src, s_rows)):
        for p in paths:
            rows, counts = load(p)
            store.append(rows)
            print(f"  {label} {p.name}: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))

    jitter = diff_stats(t_rows[0], t_rows[1])
    print_pair(f"ゆらぎ（tests 同士: {args.tests[0].name} vs {args.tests[1].name}）", jitter)

    ok = True
    for i, s in enumerate(s_rows):
        for j, t in enumerate(t_rows):
            st = diff_stats(t, s)
            print_pair(f"tests[{j}] vs src[{i}]（{args.tests[j].name} vs {args.src[i].name}）", st)
            for col in COLUMNS:
                n, mx, p95, _ = st[col]
                jn, jmx, jp95, _ = jitter[col]
                lim_max = 1.5 * jmx + SLACK[col]
                lim_p95 = 1.5 * jp95 + SLACK[col]
                if n == 0 or mx > lim_max or p95 > lim_p95:
                    ok = False
                    print(f"  × {col}: 最大差 {mx:.4f}（許容 {lim_max:.4f}）・p95 差 {p95:.4f}（許容 {lim_p95:.4f}）")
    print("\n判断: " + ("tests–src の差は tests 同士の時刻のゆらぎの範囲（1.5 倍＋余裕）に収まる" if ok else "ゆらぎを超える差がある（上の × を調べる）"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
