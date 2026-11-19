"""走行ごとの KPI を 1 行ずつ並べた Markdown の表（分析専用）。

使い方（リポジトリ直下）:
    .venv/bin/python .claude/skills/research-run-analysis/scripts/kpi_table.py 6c:082652 6e:130454

列: 最大逸脱・p95・|偏差|>0.5 / >1 の秒数（>1 は 速い/遅い）・アクセルの往復 全体 / 60 s 最大
（kpi.chatter_metrics。走行のレポート 5 章と同じ定義・走行時の yaml の kpi 設定）・
IWR（SAE J2951。kpi.iwr_j2951、括弧は定速の寄与）・1 秒未満のアクセルのオフ回数（実開度が min ＋ 0.8% 未満へ落ちて 1 秒未満で戻る。--offs で時刻と原因）。
偏差 ＝ 実車速 − 基準（正 ＝ 速すぎ）。
"""

from __future__ import annotations

import argparse

import numpy as np
from _runs import dt_of, load, parse_run, run_config


def short_offs(d, amin: float, max_s: float = 1.0) -> list[tuple[float, float, str]]:
    """アクセルの実開度が min ＋ 0.8% 未満へ落ち、max_s 未満で戻った (時刻, 長さ, 原因)。
    原因は落ちる直前 0.6 s で指令がいちばん大きく落ちた行で分ける: その前の行が惰行の補助
    （coast_effort_pct > 0）なら「C12 の離し」、計画の FF がアクセルなら「C10 が min へ」、
    それ以外は「計画の短い惰行」（ProblemReport_20261108 の表と同じ数え方）。"""
    on = amin + 0.8
    t = d.mode_time_s.to_numpy()
    aa = d.accel_actual_pct.to_numpy()
    ac = d.accel_cmd_pct.to_numpy()
    af = d.accel_ff_pct.fillna(0).to_numpy()
    ce = d.coast_effort_pct.fillna(0).to_numpy() if "coast_effort_pct" in d else np.zeros(len(d))
    out = []
    i, n = 1, len(d)
    while i < n:
        if aa[i] < on <= aa[i - 1]:
            j = i
            while j < n and aa[j] < on:
                j += 1
            if j < n and t[j] - t[i] < max_s:
                lo = max(1, i - 12)
                k = lo + int(np.argmax(ac[lo - 1 : i] - ac[lo : i + 1]))
                c = "C12 の離し" if ce[k - 1] > 0 else ("C10 が min へ" if af[k] > amin + 0.01 else "計画の短い惰行")
                out.append((float(t[i]), float(t[j] - t[i]), c))
            i = j
        else:
            i += 1
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="ラベル:トークン（例 6e:130454）")
    ap.add_argument("--offs", action="store_true", help="1 秒未満のアクセルのオフの時刻も出す")
    args = ap.parse_args()
    from tests.research.kpi import chatter_metrics, iwr_j2951

    print("| 走行 | CSV | 時間 [s] | 最大 | p95 | \\|偏差\\|>0.5 [s] | >1 [s]（速/遅） | 往復 全体 | 往復 60s 最大 | 短いオフ [回] | IWR [%]（定速） |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    notes = []
    for arg in args.runs:
        lab, path = parse_run(arg)
        d = load(path)
        cfg = run_config(path)
        dt = dt_of(d)
        dv = d.deviation_kmh.to_numpy()
        m = chatter_metrics(
            d.mode_time_s.tolist(), d.ref_speed_kmh.tolist(), d.actual_speed_kmh.tolist(),
            d.accel_cmd_pct.tolist(), d.brake_cmd_pct.tolist(), d.phase.fillna("").tolist(), cfg.kpi,
        )
        ok = d.mode_time_s.notna() & d.ref_speed_kmh.notna()
        iwr = iwr_j2951(
            d.mode_time_s[ok].tolist(), d.ref_speed_kmh[ok].tolist(), d.actual_speed_kmh[ok].tolist()
        )
        iwr_txt = "—" if iwr is None else f"{iwr.iwr_pct:+.2f}（{iwr.cruise_pct:+.2f}）"
        offs = short_offs(d, cfg.vehicle.min_accel_opening_pct)
        dur = d.mode_time_s.iloc[-1] - d.mode_time_s.iloc[0]
        print(
            f"| {lab} | {path.stem.replace('drive_log_', '')} | {dur:.0f} | {np.nanmax(np.abs(dv)):.2f} | "
            f"{np.nanquantile(np.abs(dv), 0.95):.2f} | {(np.abs(dv) > 0.5).sum() * dt:.2f} | "
            f"{(np.abs(dv) > 1).sum() * dt:.2f}（{(dv > 1).sum() * dt:.2f}/{(dv < -1).sum() * dt:.2f}） | "
            f"{m.pedal_reversal_per_s:.2f} | {m.pedal_reversal_window_max_per_s:.2f} | {len(offs)} | {iwr_txt} |"
        )
        if args.offs and offs:
            notes.append(f"- {lab} 短いオフ: " + ", ".join(f"{t:.1f} s（{s:.2f} s・{c}）" for t, s, c in offs))
    if notes:
        print()
        print("\n".join(notes))


if __name__ == "__main__":
    main()
