"""G ランプ走行の段の計画（段2k 段A。車は走らせない純ロジック）。

手順2 の学習運転は「狙いの G で加速 → 同じ G で減速」を G の段ごとに繰り返す。
どの G の段をどの車速まで走るかは、選んだ走行モード（modes.coverage_mode_names）の
「車速 × 加速度」の秒数から自動で決める。このモジュールはその計算と表示だけを行う。

定義:
- 加速度: 基準車速を 1 秒刻みに線形補間し、隣り合う点の差 [km/h/s]。G = 加速度 / G_TO_KMHS。
  車速は区間の中点。中点が STOP_SPEED_KMH 未満（停車）の区間は数えない。
- 車速帯: SPEED_BIN_KMH 幅（0–10, 10–20, …）。
- G の帯: 段の値 levels（昇順）に対し、段 i の帯は |a| が [下端_i, 上端_i)。境界は隣り合う段の
  中点、いちばん下の下端は levels[0] / 2、いちばん上の上端は無限大。加速側（a > 0）と
  減速側（a < 0）は別々に数える。|a| が最下端未満（ほぼ定速）は cruise_s に合計だけ持つ。
- 段の上端の車速: その段の帯で、加速側または減速側の秒数が mode_min_s 以上ある車速帯のうち
  いちばん高い帯の上端 [km/h]（max_speed_kmh で頭打ち。呼ぶ側は top_speed_cap_kmh の上端を渡す）。
  該当が無い段は「走らない」。
- 1 本の見込み秒数: 2 × 上端 / (g × G_TO_KMHS) + RAMP_OVERHEAD_S。
- 周回: 走る段すべてを 1 周（合計が枠 budget_s を超えても 1 周目は削らず over_budget=True）。
  残りの枠に、モードの秒数（加速側＋減速側）が多い段から 2 周目を足す
  （見込み秒数が残りに入る段だけ。入らない段はとばす）。1 段あたり最大 MAX_PASSES 周。

使い方:
    .venv/bin/python -m tests.research.ramp_planner [--config PATH]
  config の modes.coverage_mode_names の各モードを DB から読み、(a) 車速帯 × G の帯の秒数、
  (b) 段の表を出す。調整値は config の learning.ramp_g_levels / ramp_mode_min_s /
  ramp_time_budget_s。
"""

from __future__ import annotations

import argparse
import asyncio
import math
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from tests.research.research_types import G_TO_KMHS, DrivingMode  # G_TO_KMHS = 9.81 × 3.6

SPEED_BIN_KMH = 10.0  # 車速帯の幅 [km/h]（走行モードの車速の見やすさから決めた値。実測根拠なし）
STOP_SPEED_KMH = 0.5  # 区間の中点の車速がこれ未満なら停車として数えない [km/h]
# 仮値。データの根拠なし。2l の実機で決め直す。
# 1 本あたりの停車復帰・つなぎの見込み [s]。段ごとの見込み秒数に足す（周回の割り当てに効く）
RAMP_OVERHEAD_S = 15.0
MAX_PASSES = 2  # 1 段あたりの最大周回数（仮値。枠に余りがあれば 2 周目を足す）


@dataclass(frozen=True)
class BandCounts:
    """車速帯 × G の帯の秒数。accel_s / decel_s の形は (車速帯の数, 段の数)。"""

    levels: tuple[float, ...]
    speed_bin_kmh: float
    accel_s: np.ndarray
    decel_s: np.ndarray
    cruise_s: float  # |a| が最下端未満（ほぼ定速）の合計秒数


@dataclass(frozen=True)
class RampLevel:
    g: float  # 狙いの G
    top_kmh: float  # 上端の車速 [km/h]
    mode_accel_s: float  # モードがこの段の帯で加速していた秒数
    mode_decel_s: float  # 同・減速
    est_s: float  # 1 本の見込み秒数
    passes: int  # 周回数


@dataclass(frozen=True)
class RampPlan:
    levels: tuple[RampLevel, ...]  # 走る段（G 昇順）
    skipped: tuple[float, ...]  # 走らない段の G
    total_est_s: float
    budget_s: float
    over_budget: bool  # 1 周目だけで枠を超えた
    cruise_s: float

    def execution_order(self) -> list[tuple[float, float]]:
        """走る順: 1 周目を G の昇順 → 2 周目を G の昇順。要素は (g, top_kmh)。"""
        order: list[tuple[float, float]] = []
        for p in range(1, MAX_PASSES + 1):
            order.extend((lv.g, lv.top_kmh) for lv in self.levels if lv.passes >= p)
        return order


def level_lower_edges(levels: Sequence[float]) -> list[float]:
    """段 i の帯の下端 [G]（|a| / G_TO_KMHS）。levels[0]/2 と隣り合う段の中点。"""
    edges = [levels[0] / 2.0]
    edges.extend((a + b) / 2.0 for a, b in zip(levels, levels[1:], strict=False))
    return edges


def count_bands(
    speeds: Sequence[tuple[np.ndarray, np.ndarray]], levels: Sequence[float]
) -> BandCounts:
    """(時刻 [s], 車速 [km/h]) の組のリスト（モード複数）から車速帯 × G の帯の秒数を数える。"""
    lower = np.asarray(level_lower_edges(levels))
    n_lv = len(levels)
    series: list[tuple[np.ndarray, np.ndarray]] = []
    max_mid = 0.0
    for t_arr, v_arr in speeds:
        t = np.asarray(t_arr, dtype=float)
        v = np.asarray(v_arr, dtype=float)
        if len(t) < 2:
            continue
        grid = np.arange(t[0], t[-1] + 1e-9, 1.0)
        vg = np.interp(grid, t, v)
        mid = (vg[:-1] + vg[1:]) / 2.0
        acc = np.diff(vg)  # 1 秒刻みなので [km/h/s]
        keep = mid >= STOP_SPEED_KMH
        series.append((mid[keep], acc[keep]))
        if keep.any():
            max_mid = max(max_mid, float(mid[keep].max()))
    n_sp = max(1, int(math.floor(max_mid / SPEED_BIN_KMH)) + 1)
    accel = np.zeros((n_sp, n_lv))
    decel = np.zeros((n_sp, n_lv))
    cruise = 0.0
    for mid, acc in series:
        g = np.abs(acc) / G_TO_KMHS
        sp = np.minimum((mid // SPEED_BIN_KMH).astype(int), n_sp - 1)
        lv = np.searchsorted(lower, g, side="right") - 1  # 下端を含む。-1 は最下端未満
        for s_i, l_i, a in zip(sp, lv, acc, strict=True):
            if l_i < 0:
                cruise += 1.0
            elif a > 0:
                accel[s_i, l_i] += 1.0
            else:
                decel[s_i, l_i] += 1.0
    return BandCounts(tuple(levels), SPEED_BIN_KMH, accel, decel, cruise)


def count_bands_for_modes(modes: Sequence[DrivingMode], levels: Sequence[float]) -> BandCounts:
    """DrivingMode（reference_speed が SpeedPoint のリスト）を受ける薄いラッパー。"""
    series = [
        (
            np.array([p.time_s for p in m.reference_speed], dtype=float),
            np.array([p.speed_kmh for p in m.reference_speed], dtype=float),
        )
        for m in modes
    ]
    return count_bands(series, levels)


def top_speed_cap_kmh(
    modes: Sequence[DrivingMode], margin_kmh: float, vehicle_max_kmh: float
) -> float:
    """コーストダウンと G ランプの上端の車速 [km/h]。

    上端 = min(選んだモードの最高車速の最大 + 余裕, 車両の最高速)。2026-10-07 段2l-4
    （ユーザー指定: 最高速は選んだモードの最高車速 + 5 km/h で十分）。モードが空なら車両の最高速。
    """
    if not modes:
        return vehicle_max_kmh
    return min(max(m.max_speed for m in modes) + margin_kmh, vehicle_max_kmh)


def estimate_s(g: float, top_kmh: float) -> float:
    """1 本の見込み秒数: 往復の走行 2 × 上端 / (g × G_TO_KMHS) + つなぎ。"""
    return 2.0 * top_kmh / (g * G_TO_KMHS) + RAMP_OVERHEAD_S


def plan_ramps(
    counts: BandCounts, *, mode_min_s: float, max_speed_kmh: float, budget_s: float
) -> RampPlan:
    """段ごとの上端の車速・見込み秒数・周回数を決める。"""
    runs: list[RampLevel] = []
    skipped: list[float] = []
    for i, g in enumerate(counts.levels):
        enough = np.flatnonzero(
            (counts.accel_s[:, i] >= mode_min_s) | (counts.decel_s[:, i] >= mode_min_s)
        )
        if len(enough) == 0:
            skipped.append(g)
            continue
        top = min((int(enough.max()) + 1) * counts.speed_bin_kmh, max_speed_kmh)
        runs.append(
            RampLevel(
                g=g,
                top_kmh=top,
                mode_accel_s=float(counts.accel_s[:, i].sum()),
                mode_decel_s=float(counts.decel_s[:, i].sum()),
                est_s=estimate_s(g, top),
                passes=1,
            )
        )
    remaining = budget_s - sum(r.est_s for r in runs)
    over_budget = remaining < 0.0
    passes = {r.g: 1 for r in runs}
    by_mode_s = sorted(runs, key=lambda r: (-(r.mode_accel_s + r.mode_decel_s), r.g))
    for _ in range(2, MAX_PASSES + 1):
        for r in by_mode_s:
            if passes[r.g] < MAX_PASSES and r.est_s <= remaining:
                passes[r.g] += 1
                remaining -= r.est_s
    final = tuple(
        RampLevel(r.g, r.top_kmh, r.mode_accel_s, r.mode_decel_s, r.est_s, passes[r.g])
        for r in runs
    )
    return RampPlan(
        levels=final,
        skipped=tuple(skipped),
        total_est_s=sum(r.est_s * r.passes for r in final),
        budget_s=budget_s,
        over_budget=over_budget,
        cruise_s=counts.cruise_s,
    )


def format_band_table(counts: BandCounts) -> str:
    """(a) 車速帯 × G の帯の秒数（加速側・減速側）のマークダウン表。"""
    head = "| 車速帯 [km/h] | " + " | ".join(f"{g:g}G" for g in counts.levels) + " |"
    sep = "|---|" + "---|" * len(counts.levels)
    lower = level_lower_edges(counts.levels)
    out = [
        f"帯の下端 [G]: {', '.join(f'{x:.4f}' for x in lower)}"
        f"（上端は次の段の下端。最後は無限大）。ほぼ定速（最下端未満）: {counts.cruise_s:.0f} 秒",
        "",
    ]
    for title, arr in (("加速側", counts.accel_s), ("減速側", counts.decel_s)):
        out += [f"{title}の秒数（形式: 秒）", "", head, sep]
        for k in range(arr.shape[0]):
            lo, hi = k * counts.speed_bin_kmh, (k + 1) * counts.speed_bin_kmh
            cells = " | ".join(f"{x:.0f}" if x else "-" for x in arr[k])
            out.append(f"| {lo:g}–{hi:g} | {cells} |")
        out.append("")
    return "\n".join(out)


def format_plan_table(plan: RampPlan) -> str:
    """(b) 段の表と合計・枠・超過の警告のマークダウン。"""
    out = [
        "| 狙いの G | 上端の車速 [km/h] | モード秒数 加速 | モード秒数 減速 | 1 本の見込み [s] "
        "| 周回数 | 小計 [s] |",
        "|---|---|---|---|---|---|---|",
    ]
    rows = {lv.g: lv for lv in plan.levels}
    for g in sorted([*rows, *plan.skipped]):
        if g in rows:
            lv = rows[g]
            out.append(
                f"| {g:g} | {lv.top_kmh:g} | {lv.mode_accel_s:.0f} | {lv.mode_decel_s:.0f} "
                f"| {lv.est_s:.0f} | {lv.passes} | {lv.est_s * lv.passes:.0f} |"
            )
        else:
            out.append(f"| {g:g} | 走らない | - | - | - | - | - |")
    out += [
        "",
        f"合計の見込み: {plan.total_est_s:.0f} 秒 / 枠: {plan.budget_s:.0f} 秒"
        f"（走行 {plan.total_est_s / 60.0:.1f} 分）",
    ]
    if plan.over_budget:
        out.append(
            "警告: 1 周目だけで枠を超えています（1 周目は削らない）。"
            "learning.ramp_time_budget_s か段の設定を見直してください。"
        )
    return "\n".join(out)


async def _run(config_path: Path) -> int:
    from tests.research.config import load_config  # noqa: PLC0415 - CLI のときだけ
    from tests.research.mode_drive import load_mode  # noqa: PLC0415 - 循環と重い import を避ける

    cfg = load_config(config_path)
    names = cfg.modes.coverage_mode_names
    modes = [await load_mode(cfg, n) for n in names]
    lr = cfg.learning
    counts = count_bands_for_modes(modes, lr.ramp_g_levels)
    top_cap = top_speed_cap_kmh(modes, lr.ramp_top_margin_kmh, cfg.vehicle.max_speed_kmh)
    plan = plan_ramps(
        counts,
        mode_min_s=lr.ramp_mode_min_s,
        max_speed_kmh=top_cap,
        budget_s=lr.ramp_time_budget_s,
    )
    print(f"モード: {', '.join(names)}  最高速 {top_cap:g} km/h"
          f"（モードの最高車速 + {lr.ramp_top_margin_kmh:g}・車両 {cfg.vehicle.max_speed_kmh:g}"
          f" で頭打ち）")
    print(f"最小秒数 {lr.ramp_mode_min_s:g} s / 1 本のつなぎ {RAMP_OVERHEAD_S:g} s（仮値）\n")
    print("## (a) 車速帯 × G の帯の秒数\n")
    print(format_band_table(counts))
    print("## (b) 段の表\n")
    print(format_plan_table(plan))
    round_trip = sum(lv.est_s - RAMP_OVERHEAD_S for lv in plan.levels)
    print(f"\n1 周の往復秒数の合計（つなぎ抜き）: {round_trip:.0f} 秒")
    order = ", ".join(f"({g:g}, {t:g})" for g, t in plan.execution_order())
    print(f"実行順 (G, 上端 km/h): {order}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    from tests.research.config import DEFAULT_CONFIG_PATH  # noqa: PLC0415

    ap = argparse.ArgumentParser(description="G ランプ走行の段の計画（モードの秒数から）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    args = ap.parse_args(argv)
    return asyncio.run(_run(args.config))


if __name__ == "__main__":
    raise SystemExit(main())
