"""2 本の走行で、アクセル（またはブレーキ）指令の往復がどの時間帯で増えた・減ったかを出す（分析専用）。

使い方（リポジトリ直下）:
    .venv/bin/python .claude/skills/research-run-analysis/scripts/reversal_windows.py 085936 133740
    … --pedal brake --win 60 --step 20 --top 8 --flags coast_handover,play_floor_active

往復 ＝ kpi.pedal_reversals と同じ数え方（山・谷から hyst 以上戻ったら 1 回。既定 0.1%）。
窓の数字は「回数」（KPI の回/s はアクセルを操作している時間で割るので、窓の回数 ÷ 60 とは合わない。
KPI の全体・60 s 最大は最初に別に出す）。窓（win 秒、step 秒ずつずらす）ごとに A → B の回数を出し、
増えた順に top 個。各窓で --flags の列が
0 以外だった秒数を B の側で付ける（新しい仕組みが働いていない所で増えていれば、その仕組みのせいではない）。
"""

from __future__ import annotations

import argparse

import numpy as np
from _runs import dt_of, load, parse_run


def reversal_times(t: np.ndarray, x: np.ndarray, hyst: float) -> np.ndarray:
    """kpi.pedal_reversals と同じ規則で、数えた時刻の配列。"""
    out = []
    direction, extreme = 0, None
    for ti, v in zip(t, x):
        if extreme is None:
            extreme = v
        elif direction == 0:
            if abs(v - extreme) >= hyst:
                direction, extreme = (1 if v > extreme else -1), v
        elif direction == 1:
            if v > extreme:
                extreme = v
            elif extreme - v >= hyst:
                direction, extreme = -1, v
                out.append(ti)
        else:
            if v < extreme:
                extreme = v
            elif v - extreme >= hyst:
                direction, extreme = 1, v
                out.append(ti)
    return np.array(out)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--pedal", choices=("accel", "brake"), default="accel")
    ap.add_argument("--hyst", type=float, default=0.1)
    ap.add_argument("--win", type=float, default=60.0)
    ap.add_argument("--step", type=float, default=20.0)
    ap.add_argument("--top", type=int, default=8)
    ap.add_argument("--flags", default="coast_handover,play_floor_active,hold_scene")
    a = ap.parse_args()
    la, pa = parse_run(a.a)
    lb, pb = parse_run(a.b)
    da, db = load(pa), load(pb)
    c = f"{a.pedal}_cmd_pct"
    ra = reversal_times(da.mode_time_s.to_numpy(), da[c].to_numpy(), a.hyst)
    rb = reversal_times(db.mode_time_s.to_numpy(), db[c].to_numpy(), a.hyst)
    T = float(min(da.mode_time_s.iloc[-1], db.mode_time_s.iloc[-1]))
    print(f"{a.pedal} の往復の回数（モード全体・hyst {a.hyst:g}%）: {la} {len(ra)} → {lb} {len(rb)}")
    if a.pedal == "accel":
        from _runs import run_config

        from tests.research.kpi import chatter_metrics

        for lab, d, p in ((la, da, pa), (lb, db, pb)):
            m = chatter_metrics(
                d.mode_time_s.tolist(), d.ref_speed_kmh.tolist(), d.actual_speed_kmh.tolist(),
                d.accel_cmd_pct.tolist(), d.brake_cmd_pct.tolist(), d.phase.fillna("").tolist(),
                run_config(p).kpi,
            )
            print(f"  KPI（アクセルを操作している時間で割る・レポートと同じ）{lab}: 全体 "
                  f"{m.pedal_reversal_per_s:.2f}・60 s 最大 {m.pedal_reversal_window_max_per_s:.2f} 回/s")
    starts = np.arange(0.0, max(T - a.win, 0.0) + 1e-9, a.step)
    ca = np.array([((ra >= s) & (ra < s + a.win)).sum() for s in starts])
    cb = np.array([((rb >= s) & (rb < s + a.win)).sum() for s in starts])
    flags = [f for f in a.flags.split(",") if f and f in db]
    dt = dt_of(db)
    tb = db.mode_time_s.to_numpy()
    print(f"| 窓 [s] | {la} | {lb} | 差 | {lb} の平均基準 [km/h] | " + " | ".join(f"{f} [s]" for f in flags) + " |")
    print("|---|---|---|---|---|" + "---|" * len(flags))
    for k in np.argsort(-(cb - ca), kind="stable")[: a.top]:
        s = starts[k]
        m = (tb >= s) & (tb < s + a.win)
        fl = [f"{(db[f][m].fillna(0) != 0).sum() * dt:.1f}" for f in flags]
        print(f"| {s:.0f}〜{s + a.win:.0f} | {ca[k]} | {cb[k]} | {cb[k] - ca[k]:+d} | {db.ref_speed_kmh[m].mean():.1f} | " + " | ".join(fl) + " |")


if __name__ == "__main__":
    main()
