"""走行 CSV から「保持の区間」（基準がほぼ一定の低速）を取り出して、追従を判定する CLI。

2026-10-30 UDDS 1252 s の「基準 1.6 km/h 一定」でブレーキが往復する問題の確認用
（ProblemReport_20261030）。

保持の区間の行: 基準の傾きの大きさ ≤ ramp_cruise_band_g × 9.80665 × 3.6 [km/h/s]、かつ
停車速度 < 基準 < クリープ車速。連続した行のかたまり（行間 GAP_S 未満）を 1 区間とし、区間ごとに
  (a) |偏差| ≤ tol だった時間の合計が hold_s に達した時刻（区間の開始からの秒数も。無ければ
      「到達せず」）。数え方は G ランプの定速の保持（pattern_loop の ramp_cruise_hold_s。帯に入って
      いた時間の合計。途切れても 0 に戻さない）と同じ（2026-10-30 ユーザー決定）
  (b) |偏差| の p95・最大・≤ tol の行の割合
  (c) ブレーキ指令の往復回数（kpi.pedal_reversals。0.1% 戻りで 1 回）と回/s
  (d) ブレーキ実開度の平均・最小・最大
を出す。

使い方: .venv/bin/python -m tests.research.hold_check <csv> [--t0 S --t1 S]
計算の本体は純関数（`find_hold_segments`・`judge_segment`）。src/ は import しない。
"""

from __future__ import annotations

import argparse
import csv
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.ff_model import STOP_SPEED_KMH
from tests.research.kpi import pedal_reversals
from tests.research.term import say

G_MS2 = 9.80665
# 区間の行が途切れたとみなす間隔 [s]（local_gain.GAP_S と同じ値。独立に持つ）
GAP_S = 0.5
# 到達判定の既定の偏差幅 [km/h]（議論の決定。--tol-kmh で変える）
DEFAULT_TOL_KMH = 0.3
# ブレーキ指令の往復を数える戻り幅 [%]（議論の決定）
REVERSAL_HYST_PCT = 0.1


@dataclass(frozen=True)
class HoldSegment:
    """保持の区間 1 つ分の時系列（絶対時刻 t_s）。"""

    t_s: np.ndarray
    dev_kmh: np.ndarray
    brake_cmd_pct: np.ndarray
    brake_actual_pct: np.ndarray


@dataclass(frozen=True)
class SegmentJudge:
    """区間 1 つの判定。reach_t_s・reach_after_s は到達しなければ None。"""

    t_start_s: float
    t_end_s: float
    length_s: float
    reach_t_s: float | None
    reach_after_s: float | None
    abs_dev_p95: float
    abs_dev_max: float
    within_tol_ratio: float
    reversals: int
    reversals_per_s: float
    actual_mean: float
    actual_min: float
    actual_max: float


def slope_threshold_kmhs(band_g: float) -> float:
    """基準の傾きのしきい値 [km/h/s]（band_g [G] を換算）。"""
    return band_g * G_MS2 * 3.6


def find_hold_segments(
    t_s: np.ndarray,
    ref_kmh: np.ndarray,
    dev_kmh: np.ndarray,
    brake_cmd_pct: np.ndarray,
    brake_actual_pct: np.ndarray,
    *,
    slope_max_kmhs: float,
    creep_kmh: float,
    stop_kmh: float = STOP_SPEED_KMH,
    gap_s: float = GAP_S,
) -> list[HoldSegment]:
    """保持の区間（基準の傾き ≤ slope_max_kmhs かつ stop_kmh < 基準 < creep_kmh）を切り出す。

    基準の傾きは np.gradient（時刻 t_s で）。行が飛んだ所（gap_s 以上）・マスクが切れた所で区切る。
    """
    t = np.asarray(t_s, dtype=float)
    ref = np.asarray(ref_kmh, dtype=float)
    if len(t) < 2:
        return []
    slope = np.abs(np.gradient(ref, t))
    mask = (slope <= slope_max_kmhs) & (ref > stop_kmh) & (ref < creep_kmh)
    mask &= np.isfinite(ref) & np.isfinite(dev_kmh) & np.isfinite(t)
    segs: list[HoldSegment] = []
    start: int | None = None
    for i in range(len(t) + 1):
        ok = i < len(t) and bool(mask[i])
        gap = i < len(t) and i > 0 and t[i] - t[i - 1] >= gap_s
        if start is not None and (not ok or gap):  # 区間が終わる
            sl = slice(start, i)
            segs.append(HoldSegment(t[sl], dev_kmh[sl], brake_cmd_pct[sl], brake_actual_pct[sl]))
            start = None
        if ok and start is None:
            start = i
    return segs


def first_reach(
    t_s: np.ndarray, dev_kmh: np.ndarray, tol_kmh: float, hold_s: float
) -> float | None:
    """|偏差| ≤ tol_kmh だった時間の合計が hold_s に達した時刻。無ければ None。

    行の時間は次の行までの間隔（行間 GAP_S 以上は数えない）。G ランプの定速の保持と同じく、
    帯の外の行があっても合計は 0 に戻さない。
    """
    ok = np.abs(dev_kmh) <= tol_kmh
    total = 0.0
    for i in range(len(t_s) - 1):
        dt = float(t_s[i + 1] - t_s[i])
        if ok[i] and 0.0 < dt < GAP_S:
            total += dt
            if total >= hold_s - 1e-9:
                return float(t_s[i + 1])
    return None


def judge_segment(seg: HoldSegment, *, tol_kmh: float, hold_s: float) -> SegmentJudge:
    """区間 1 つを判定する。"""
    t = seg.t_s
    length = float(t[-1] - t[0])
    adev = np.abs(seg.dev_kmh)
    reach = first_reach(t, seg.dev_kmh, tol_kmh, hold_s)
    rev = pedal_reversals(seg.brake_cmd_pct, hyst=REVERSAL_HYST_PCT)
    act = seg.brake_actual_pct[np.isfinite(seg.brake_actual_pct)]
    nan = math.nan
    return SegmentJudge(
        t_start_s=float(t[0]),
        t_end_s=float(t[-1]),
        length_s=length,
        reach_t_s=reach,
        reach_after_s=None if reach is None else reach - float(t[0]),
        abs_dev_p95=float(np.percentile(adev, 95)),
        abs_dev_max=float(adev.max()),
        within_tol_ratio=float(np.mean(adev <= tol_kmh)),
        reversals=rev,
        reversals_per_s=rev / length if length > 0 else nan,
        actual_mean=float(act.mean()) if len(act) else nan,
        actual_min=float(act.min()) if len(act) else nan,
        actual_max=float(act.max()) if len(act) else nan,
    )


def _f(x: str | None) -> float:
    try:
        v = float(x) if x not in (None, "") else math.nan
    except ValueError:
        return math.nan
    return v


def read_columns(csv_path: Path) -> dict[str, np.ndarray]:
    """CSV から必要な列を読む。時刻は mode_time_s（空なら elapsed_s）。"""
    cols: dict[str, list[float]] = {k: [] for k in ("t", "ref", "dev", "cmd", "act")}
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            t = _f(r.get("mode_time_s"))
            if not math.isfinite(t):
                t = _f(r.get("elapsed_s"))
            cols["t"].append(t)
            cols["ref"].append(_f(r.get("ref_speed_kmh")))
            cols["dev"].append(_f(r.get("deviation_kmh")))
            cols["cmd"].append(_f(r.get("brake_cmd_pct")))
            cols["act"].append(_f(r.get("brake_actual_pct")))
    return {k: np.array(v, dtype=float) for k, v in cols.items()}


def check_csv(
    csv_path: Path,
    *,
    slope_max_kmhs: float,
    creep_kmh: float,
    tol_kmh: float,
    hold_s: float,
    t0: float | None = None,
    t1: float | None = None,
) -> list[SegmentJudge]:
    """CSV を読み、mode_time_s の範囲 [t0, t1] で絞って、保持の区間を判定する。"""
    c = read_columns(csv_path)
    keep = np.isfinite(c["t"]) & np.isfinite(c["ref"])
    if t0 is not None:
        keep &= c["t"] >= t0
    if t1 is not None:
        keep &= c["t"] <= t1
    segs = find_hold_segments(
        c["t"][keep], c["ref"][keep], c["dev"][keep], c["cmd"][keep], c["act"][keep],
        slope_max_kmhs=slope_max_kmhs, creep_kmh=creep_kmh,
    )
    return [judge_segment(s, tol_kmh=tol_kmh, hold_s=hold_s) for s in segs]


def format_table(judges: Sequence[SegmentJudge], tol_kmh: float, hold_s: float) -> list[str]:
    """判定の表（行のリスト）。"""
    lines = [
        f"| 開始 s | 終了 s | 長さ s | |偏差|≤{tol_kmh:g} の合計が {hold_s:g}s に達した時刻 "
        "| 区間開始から s | |偏差| p95 | |偏差| 最大 | ≤tol の割合 "
        "| ブレーキ指令の往復 | 回/s | 実開度 平均 | 最小 | 最大 |",
        "|" + "---|" * 13,
    ]
    for j in judges:
        reach = "到達せず" if j.reach_t_s is None else f"{j.reach_t_s:.2f}"
        after = "-" if j.reach_after_s is None else f"{j.reach_after_s:.2f}"
        lines.append(
            f"| {j.t_start_s:.2f} | {j.t_end_s:.2f} | {j.length_s:.2f} | {reach} | {after} "
            f"| {j.abs_dev_p95:.3f} | {j.abs_dev_max:.3f} | {j.within_tol_ratio:.1%} "
            f"| {j.reversals} | {j.reversals_per_s:.3f} "
            f"| {j.actual_mean:.2f} | {j.actual_min:.2f} | {j.actual_max:.2f} |"
        )
    return lines


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="hold_check", description="走行 CSV の保持の区間を判定する")
    p.add_argument("csv", help="走行 CSV")
    p.add_argument("--config", default=str(DEFAULT_CONFIG_PATH))
    p.add_argument("--t0", type=float, default=None, help="mode_time_s の始まり [s]")
    p.add_argument("--t1", type=float, default=None, help="mode_time_s の終わり [s]")
    p.add_argument(
        "--creep-kmh", type=float, default=None, help="省略: feedforward.creep_speed_kmh"
    )
    p.add_argument("--hold-s", type=float, default=None, help="省略: learning.ramp_cruise_hold_s")
    p.add_argument("--tol-kmh", type=float, default=DEFAULT_TOL_KMH)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = load_config(Path(args.config))
    creep = float(args.creep_kmh) if args.creep_kmh is not None else cfg.feedforward.creep_speed_kmh
    hold_s = float(args.hold_s) if args.hold_s is not None else cfg.learning.ramp_cruise_hold_s
    slope = slope_threshold_kmhs(cfg.learning.ramp_cruise_band_g)
    judges = check_csv(
        Path(args.csv), slope_max_kmhs=slope, creep_kmh=creep, tol_kmh=args.tol_kmh,
        hold_s=hold_s, t0=args.t0, t1=args.t1,
    )
    say(f"保持の区間: 基準の傾き ≤ {slope:.3f} km/h/s・{STOP_SPEED_KMH:g} < 基準 < {creep:g} km/h"
        f"（{len(judges)} 区間）")
    if not judges:
        return 1
    for line in format_table(judges, args.tol_kmh, hold_s):
        say(line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
