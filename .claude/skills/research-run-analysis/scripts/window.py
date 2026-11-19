"""1 本または複数の走行の、時刻の窓の中の列を表で出す（分析専用）。

使い方（リポジトリ直下）:
    .venv/bin/python .claude/skills/research-run-analysis/scripts/window.py 140206 476.6 479
    … 082652 130454 1762 1766 --every 0.15 --cols +coast_pred_dev_kmh,play_floor_pct
    … 140206 476 479 --preset coast

位置引数は「走行…（1 本以上） t0 t1」。--every は間引きの秒数（既定 0.15 s ≒ 3 周期）。
--preset で列の組（base・coast・dob・stop・launch・all）。--cols で列を足す（先頭 + で足す、無しで置き換え）。
無い列は黙って飛ばす（古いログでも動く）。
"""

from __future__ import annotations

import argparse

import pandas as pd
from _runs import dt_of, load, parse_run

BASE = ["mode_time_s", "ref_speed_kmh", "deviation_kmh", "accel_ff_pct", "brake_ff_pct",
        "accel_cmd_pct", "brake_cmd_pct", "accel_actual_pct", "brake_actual_pct",
        "c10_effort_pct", "dob_effort_pct"]
PRESETS = {
    "base": BASE,
    "coast": BASE + ["coast_effort_pct", "coast_handover", "coast_pred_dev_kmh", "play_floor_pct",
                     "play_floor_active", "c10_integral_pct"],
    "dob": BASE + ["dob_accel_kmhs", "dob_brake_kmhs", "dob_brake_low_kmhs", "c10_integral_pct",
                   "c10_integral_kmhs"],
    "stop": BASE + ["stop_end_tau_s", "stop_pred_dev_kmh", "stop_tail_active", "hold_scene",
                    "hold_pi_effort_pct", "creep_floor_pct"],
    "launch": BASE + ["launch_step_code", "launch_obs_pct", "band_late_rows", "hold_scene"],
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("args", nargs="+", help="走行… t0 t1")
    ap.add_argument("--every", type=float, default=0.15, help="間引きの秒数（0 で全行）")
    ap.add_argument("--preset", default="base", choices=[*PRESETS, "all"])
    ap.add_argument("--cols", default="", help="列（カンマ区切り）。先頭 + で preset に足す")
    a = ap.parse_args()
    *runs, t0, t1 = a.args
    t0, t1 = float(t0), float(t1)
    pd.set_option("display.width", 300)
    pd.set_option("display.max_rows", 2000)
    pd.set_option("display.max_columns", 100)
    for arg in runs:
        lab, path = parse_run(arg)
        d = load(path)
        if a.preset == "all":
            cols = list(d.columns)
        else:
            cols = list(PRESETS[a.preset])
        if a.cols:
            extra = a.cols.lstrip("+").split(",")
            cols = cols + extra if a.cols.startswith("+") else ["mode_time_s", *extra]
        cols = [c for c in dict.fromkeys(cols) if c in d]
        w = d[(d.mode_time_s >= t0) & (d.mode_time_s <= t1)]
        if a.every > 0:
            w = w.iloc[:: max(1, round(a.every / dt_of(d)))]
        print(f"== {lab}（{path.stem}）")
        print(w[cols].round(3).to_string(index=False))


if __name__ == "__main__":
    main()
