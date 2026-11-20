"""場面ごとの比較図（PNG）: 列 ＝ 場面（時刻の窓）、行 ＝ 走行。各走行は「偏差」と「ペダル開度」の 2 段（分析専用）。

使い方（リポジトリ直下）:
    .venv/bin/python .claude/skills/research-run-analysis/scripts/plot_scenes.py \\
        --runs "段6c:082652" "段6e:130454" \\
        --scene "W 1758〜1768 s:1758:1768" --scene "W 318〜323 s:318:323" \\
        --title "段6g 実機: 段6c と 段6e" --note "左・中: C12 のアクセル" --out <scratchpad>/stage6g.png

場面ごとに走行を変えたいとき（W と US06 を並べる等）は、--scene の 4 つ目に走行を「,」で並べる:
    --scene "US06 474〜480 s:474:480:6c=092405,6e=140206"   （ラベル=トークン。--runs のラベルと同じ並び）
--cmd で指令（点線）も重ねる。--ylim-dev で偏差の縦軸、--ylim-pedal で開度の縦軸（既定は自動）。
点線: 偏差 ±0.5 km/h・開度 min。偏差 ＝ 実車速 − 基準（正 ＝ 速すぎ）。
"""

from __future__ import annotations

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from _runs import load, parse_run, run_config  # noqa: E402

A, B, INK, MUTED, SURF, GRID = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#fcfcfb", "#e5e4e0"
plt.rcParams["font.family"] = "BIZ UDPGothic"
plt.rcParams["axes.unicode_minus"] = False


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", nargs="+", required=True, help="ラベル:トークン（行の並び）")
    ap.add_argument("--scene", action="append", required=True, help="題:t0:t1[:ラベル=トークン,…]")
    ap.add_argument("--title", default="場面ごとの比較")
    ap.add_argument("--note", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--cmd", action="store_true", help="指令も点線で重ねる")
    ap.add_argument("--ylim-dev", type=float, default=0.0, help="偏差の縦軸 ±（0 で自動）")
    ap.add_argument("--ylim-pedal", type=float, default=0.0, help="開度の縦軸の上（0 で自動）")
    a = ap.parse_args()
    runs = [parse_run(r) for r in a.runs]
    scenes = []
    for s in a.scene:
        parts = s.split(":")
        title, t0, t1 = parts[0], float(parts[1]), float(parts[2])
        if len(parts) > 3:
            over = [parse_run(x.replace("=", ":", 1)) for x in parts[3].split(",")]
        else:
            over = runs
        scenes.append((title, t0, t1, over))
    nr, nc = len(runs), len(scenes)
    fig, ax = plt.subplots(2 * nr, nc, figsize=(5.4 * nc, 2.6 * nr * 2), sharex="col", squeeze=False,
                           gridspec_kw={"height_ratios": [0.8, 1.2] * nr}, facecolor=SURF)
    cache: dict = {}
    for c, (title, t0, t1, rs) in enumerate(scenes):
        dmax, pmax = 0.0, 0.0
        for r, (lab, path) in enumerate(rs):
            if path not in cache:
                cache[path] = (load(path), run_config(path).vehicle)
            d, veh = cache[path]
            d = d[(d.mode_time_s >= t0) & (d.mode_time_s <= t1)]
            a1, a2 = ax[2 * r][c], ax[2 * r + 1][c]
            for x in (a1, a2):
                x.set_facecolor(SURF)
                x.grid(color=GRID, lw=0.6)
                for sp in ("top", "right"):
                    x.spines[sp].set_visible(False)
            a1.plot(d.mode_time_s, d.deviation_kmh, color=INK, lw=1.5)
            a1.axhline(0, color=MUTED, lw=0.8)
            for y in (0.5, -0.5):
                a1.axhline(y, color=MUTED, lw=0.8, ls=":")
            a1.set_title(f"{lab}（{path.stem.split('_')[-1]}）　{title}", loc="left", color=INK, fontsize=10.5)
            a2.plot(d.mode_time_s, d.accel_actual_pct, color=A, lw=2, label="アクセル 実開度")
            a2.plot(d.mode_time_s, d.brake_actual_pct, color=B, lw=2, label="ブレーキ 実開度")
            if a.cmd:
                a2.plot(d.mode_time_s, d.accel_cmd_pct, color=A, lw=1, ls="--", label="アクセル 指令")
                a2.plot(d.mode_time_s, d.brake_cmd_pct, color=B, lw=1, ls="--", label="ブレーキ 指令")
            a2.axhline(veh.min_accel_opening_pct, color=MUTED, lw=0.8, ls=":")
            if c == 0:
                a1.set_ylabel("偏差 [km/h]\n実車速−基準", color=MUTED)
                a2.set_ylabel("開度 [%]", color=MUTED)
            if r == 0 and c == 0:
                a2.legend(loc="upper right", frameon=False, fontsize=9)
            if len(d):
                dmax = max(dmax, float(d.deviation_kmh.abs().max()))
                pmax = max(pmax, float(d[["accel_actual_pct", "brake_actual_pct"]].max().max()))
        for r in range(nr):
            yd = a.ylim_dev or max(0.8, round(dmax + 0.2, 1))
            ax[2 * r][c].set_ylim(-yd, yd)
            ax[2 * r + 1][c].set_ylim(0, a.ylim_pedal or max(10.0, pmax * 1.15))
        ax[2 * nr - 1][c].set_xlabel("モード時刻 [s]", color=MUTED)
    fig.suptitle(a.title, x=0.01, y=0.995, ha="left", fontsize=14, color=INK)
    h = fig.get_size_inches()[1]
    fig.text(0.01, 1 - 0.42 / h, "点線: 偏差 ±0.5 km/h・開度 min。" + a.note, color=MUTED, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 1 - 0.7 / h))
    fig.savefig(a.out, dpi=100, facecolor=SURF)
    print(a.out)


if __name__ == "__main__":
    main()
