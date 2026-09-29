"""通し掃引の計画（ProblemReport_20260925 段6c）。

格子ステップ走行は 1 ステップが開度固定で最大 3 s・±10 km/h の帯を出たら終わるので、強い加減速の
セル（|a| > 約 3 km/h/s。帯を 1〜2 s で抜ける）は 1 ステップでは学習データ 2 s に届かない。
通し掃引は、**車速のセルごとに開度を切り替えながら 1 回で複数の車速帯を通り抜ける**:
同じ加速度の列の穴を 1 本にまとめ、各セルの開度は「そのセルの車速で狙いの加速度が出る開度」
（G 校正・格子ステップの実測から）にする。

このモジュールは穴 → 掃引の計画（純関数）。開度の決定と走行は `pattern_loop.PatternLoop`。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SweepCell:
    i: int  # 車速ビン
    j: int  # 加速度ビン
    speed_lo_kmh: float
    speed_hi_kmh: float
    a_kmhs: float  # 狙いの加速度（モードのそのセルの平均。上限 G で頭打ち）


@dataclass(frozen=True)
class SweepPlan:
    """1 本の掃引。同じ加速度の列の、穴を含む連続した車速帯。`cells` は通る順。"""

    j: int
    decel: bool  # True = 減速（高い車速から下る）、False = 加速（低い車速から上る）
    cells: tuple[SweepCell, ...]

    @property
    def start_kmh(self) -> float:
        """掃引を始める車速: 減速は最初のセルの上端、加速は最初のセルの下端。"""
        first = self.cells[0]
        return first.speed_hi_kmh if self.decel else first.speed_lo_kmh

    @property
    def end_kmh(self) -> float:
        """掃引を終える車速: 減速は最後のセルの下端、加速は最後のセルの上端。"""
        last = self.cells[-1]
        return last.speed_lo_kmh if self.decel else last.speed_hi_kmh

    def cell_at(self, speed_kmh: float) -> SweepCell:
        """その車速を含むセル。範囲外は、通る順で一番近い端のセル。"""
        for cell in self.cells:
            if cell.speed_lo_kmh <= speed_kmh < cell.speed_hi_kmh:
                return cell
        first, last = self.cells[0], self.cells[-1]
        if self.decel:  # 上から下へ: 上端より上は最初、下端より下は最後
            return first if speed_kmh >= first.speed_hi_kmh else last
        return first if speed_kmh < first.speed_lo_kmh else last


def plan_sweeps(
    holes: list[tuple[int, int]],
    wltp_mean: np.ndarray,
    speed_edges: list[float] | tuple[float, ...],
    accel_edges: list[float] | tuple[float, ...],
    a_cap_kmhs: float,
    wltp_seconds: np.ndarray | None = None,
) -> list[SweepPlan]:
    """穴（(車速ビン, 加速度ビン)）から掃引を作る。需要（穴の WLTP 秒の合計）が大きい列から。

    - 加速度の列ごとに 1 本。穴の一番低い車速ビン〜一番高い車速ビンの間は、穴でないビンも通る
    - 定速の列（0 を含む列）は掃引できないので作らない
    - 狙いの加速度 = そのセルのモードの平均。上限 `a_cap_kmhs`（G の上限）を超える列は、
      その内側の端が上限以内なら上限で頭打ち、内側の端が上限を超えるなら作らない
    """
    by_col: dict[int, list[int]] = {}
    for i, j in holes:
        by_col.setdefault(j, []).append(i)
    plans: list[tuple[float, SweepPlan]] = []
    for j, rows in by_col.items():
        lo_a, hi_a = float(accel_edges[j]), float(accel_edges[j + 1])
        if lo_a < 0.0 < hi_a:
            continue  # 定速の列
        decel = hi_a <= 0.0
        inner = min(abs(lo_a), abs(hi_a))
        if inner >= a_cap_kmhs:
            continue  # 上限 G より強い列は測れない
        cells: list[SweepCell] = []
        for i in range(min(rows), max(rows) + 1):
            mean = float(wltp_mean[i, j])
            if not math.isfinite(mean):
                mean = 0.5 * (lo_a + hi_a)  # モードがそのセルを使わない（通るだけ）: 列の中央
            mag = min(max(abs(mean), inner), a_cap_kmhs)
            cells.append(
                SweepCell(
                    i, j, float(speed_edges[i]), float(speed_edges[i + 1]),
                    -mag if decel else mag,
                )
            )
        if decel:
            cells.reverse()  # 高い車速から下る
        demand = (
            float(sum(wltp_seconds[i, j] for i in rows)) if wltp_seconds is not None
            else float(len(rows))
        )
        plans.append((demand, SweepPlan(j, decel, tuple(cells))))
    plans.sort(key=lambda p: -p[0])
    return [p for _, p in plans]


__all__ = ["SweepCell", "SweepPlan", "plan_sweeps"]
