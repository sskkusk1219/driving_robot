"""手順3 と同じ作り方で計画（開度の列・行の印）を作り直し、時刻の窓を出す（分析専用・机上）。

使い方（リポジトリ直下）:
    .venv/bin/python .claude/skills/research-run-analysis/scripts/plan_at.py <yaml> 1270 1300
    … 130454 1270 1300            （走行のトークンなら、その走行の yaml とモードで作り、ログの FF と突き合わせる）
    … <yaml> 470 480 --mode 09_US06 --set coast_fill_max_s=0 --set coast_fill_same_pedal=false

作り方は mode_drive.forward_map_plan_args（実行時と同じ引数）→ fwd_plan.build_plan。表は
apply_table_options を通す。走行のトークンを渡したときは、作った計画の effort とログの accel_ff_pct /
brake_ff_pct の差の最大を最初に出す（0.01% 程度なら実行時と同じ計画を再現できている。大きければ
このスクリプトか、計画を作るコードが変わっている → 机上の結論に使う前に原因を見る）。
--set は forward_map の値を上書きする（案の机上比較用。ログとの突き合わせは上書き前の計画で行う）。
"""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path

import numpy as np
from _runs import REPO, find_csv, load


def _val(s: str):
    lo = s.lower()
    if lo in ("true", "false"):
        return lo == "true"
    try:
        return float(s) if "." in s or "e" in lo else int(s)
    except ValueError:
        return s


def build(cfg, mode_name: str):
    from tests.research import fwd_plan, mode_drive
    from tests.research.fwd_map import load_forward_map

    mode = asyncio.run(mode_drive.load_mode(cfg, mode_name))
    mp = Path(cfg.feedforward.model_path)
    table = mode_drive.apply_table_options(cfg, load_forward_map(mp if mp.is_absolute() else REPO / mp))
    from tests.research.ff_candidate import CANDIDATE_CLASSES, StopEndFeedforward

    cls = CANDIDATE_CLASSES.get(str(cfg.feedforward.candidate))
    stop_end = cls is not None and issubclass(cls, StopEndFeedforward)
    ref_at, dur, kw = mode_drive.forward_map_plan_args(cfg, mode, stop_end=stop_end, table=table)
    return fwd_plan.build_plan(ref_at, dur, table, **kw)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", help="yaml のパス、または走行のトークン")
    ap.add_argument("t0", type=float)
    ap.add_argument("t1", type=float)
    ap.add_argument("--mode", default=None, help="モード名（既定は yaml の modes.wltp_mode_name）")
    ap.add_argument("--set", action="append", default=[], help="forward_map.KEY=VALUE（何度でも）")
    ap.add_argument("--every", type=int, default=1, help="行の間引き")
    a = ap.parse_args()
    from tests.research.config import load_config
    from tests.research.fwd_plan import FLAG_LABELS

    log = None
    p = Path(a.src)
    if p.suffix in (".yaml", ".yml") and p.exists():
        cfg = load_config(p)
    else:
        csv = find_csv(a.src)
        cfg = load_config(csv.with_suffix(".yaml"))
        log = load(csv)
    name = a.mode or cfg.modes.wltp_mode_name
    plan = build(cfg, name)
    t = np.asarray(plan.t_s)
    eff = np.asarray(plan.effort_pct)
    if log is not None:
        lt = log.mode_time_s.to_numpy()
        ff = log.accel_ff_pct.fillna(0).to_numpy() - log.brake_ff_pct.fillna(0).to_numpy()
        pe = eff[np.clip(np.searchsorted(t, lt, side="right") - 1, 0, len(t) - 1)]
        m = (lt >= a.t0) & (lt <= a.t1)
        err = np.abs(pe - ff)
        print(f"ログの FF との差: 窓の中 最大 {err[m].max():.3f}%・0.01% 以内の行 {np.mean(err[m] < 0.01):.0%}"
              f"／モード全体で 0.01% 以内の行 {np.mean(err < 0.01):.0%}"
              "（実行時に書き換わる行（先置き・安全網・発進の帯など）は差が出る。窓の中が大きく外れたら原因を見る）")
    if a.set:
        for kv in a.set:
            k, v = kv.split("=", 1)
            if not hasattr(cfg.forward_map, k):
                raise SystemExit(f"forward_map に {k} が無い")
            setattr(cfg.forward_map, k, _val(v))
        plan = build(cfg, name)
        t, eff = np.asarray(plan.t_s), np.asarray(plan.effort_pct)
        print("上書き: " + ", ".join(a.set))
    flag = np.asarray(plan.flag)
    need = np.asarray(plan.need_accel_kmhs)
    print(f"モード {name}・周期 {plan.period_s:g} s（effort 正 ＝ アクセル・負 ＝ ブレーキ）")
    print("  時刻 [s]   effort [%]  要る加速度 [km/h/s]  印")
    idx = np.flatnonzero((t >= a.t0) & (t <= a.t1))[:: a.every]
    for i in idx:
        print(f"  {t[i]:8.2f}  {eff[i]:+9.3f}  {need[i]:+9.3f}  {FLAG_LABELS.get(int(flag[i]), flag[i])}")


if __name__ == "__main__":
    main()
