"""|偏差| がしきい値を超えた場面を走行ごとに並べ、同じ場面（時刻が近い）を走行どうしで突き合わせる（分析専用）。

使い方（リポジトリ直下）:
    .venv/bin/python .claude/skills/research-run-analysis/scripts/episodes.py 6c:082652 6e:130454
    … --thr 1.0 --gap 1.5 --before 3 --flags coast_handover,play_floor_active,hold_scene

1 場面 ＝ |偏差| > thr の行が gap 秒以内に続くまとまり。出すもの: 時刻の範囲・最大（符号つき。正 ＝ 速すぎ）・
その時刻の基準車速・直前 before 秒に 0 以外だった印の列（--flags、無い列は飛ばす）。
最後に「場面 × 走行」の表を出す（時刻が match 秒以内なら同じ場面）。片方にしか無い場面が、新しい
仕組みのせいか日のばらつきかを見る入口になる（印が直前に立っていなければ、その仕組みは関係ない）。
"""

from __future__ import annotations

import argparse

import numpy as np
from _runs import dt_of, load, parse_run

DEFAULT_FLAGS = (
    "coast_handover,play_floor_active,hold_scene,stop_tail_active,launch_step_code,governor_active"
)


def episodes(d, thr: float, gap: float) -> list[tuple[float, float, float, float, int]]:
    t = d.mode_time_s.to_numpy()
    a = np.abs(d.deviation_kmh.to_numpy())
    idx = np.flatnonzero(a > thr)
    if not len(idx):
        return []
    out = []
    for g in np.split(idx, np.flatnonzero(np.diff(t[idx]) > gap) + 1):
        k = g[np.argmax(a[g])]
        out.append((float(t[g[0]]), float(t[g[-1]]), float(d.deviation_kmh.iloc[k]), float(d.ref_speed_kmh.iloc[k]), int(k)))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--thr", type=float, default=0.5, help="|偏差| のしきい値 [km/h]（既定 0.5）")
    ap.add_argument("--gap", type=float, default=1.5, help="同じ場面とみなす切れ目 [s]")
    ap.add_argument("--before", type=float, default=3.0, help="印を見る直前の秒数")
    ap.add_argument("--match", type=float, default=3.0, help="走行どうしで同じ場面とみなす時刻の差 [s]")
    ap.add_argument("--flags", default=DEFAULT_FLAGS, help="直前に立っていたか見る列（カンマ区切り）")
    args = ap.parse_args()
    flags = [f for f in args.flags.split(",") if f]
    table: dict[str, list] = {}
    for arg in args.runs:
        lab, path = parse_run(arg)
        d = load(path)
        dt = dt_of(d)
        t = d.mode_time_s.to_numpy()
        eps = episodes(d, args.thr, args.gap)
        print(f"== {lab}（{path.stem}）: {len(eps)} 場面・|偏差|>{args.thr:g} 合計 {(np.abs(d.deviation_kmh) > args.thr).sum() * dt:.2f} s")
        rows = []
        for t0, t1, peak, ref, k in eps:
            i0 = int(np.searchsorted(t, t0 - args.before))
            on = []
            for f in flags:
                if f in d:
                    v = d[f].iloc[i0 : k + 1].fillna(0).to_numpy()
                    n = int((v != 0).sum())
                    if n:
                        on.append(f"{f}={n * dt:.2f}s")
            print(f"  {t0:7.1f}〜{t1:7.1f} s  最大 {peak:+.2f}（基準 {ref:5.1f} km/h）  直前: {', '.join(on) or '印なし'}")
            rows.append((t0, peak))
        table[lab] = rows
    if len(table) < 2:
        return
    # 場面 × 走行
    times = sorted({round(t0, 1) for rows in table.values() for t0, _ in rows})
    groups: list[float] = []
    for tt in times:
        if not groups or tt - groups[-1] > args.match:
            groups.append(tt)
    labs = list(table)
    print()
    print("| 場面 [s] | " + " | ".join(labs) + " |")
    print("|---|" + "---|" * len(labs))
    for g in groups:
        cells = []
        for lab in labs:
            hit = [p for t0, p in table[lab] if abs(t0 - g) <= args.match]
            cells.append(f"{max(hit, key=abs):+.2f}" if hit else "—")
        print(f"| {g:.1f} | " + " | ".join(cells) + " |")


if __name__ == "__main__":
    main()
