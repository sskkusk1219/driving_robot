"""C14（C12 ＋ 停車の最後のブレーキの下限）の判断（2026-10-19 段3a）。

基準が停車（0 km/h）へ向かう最後に、FF がブレーキを戻しすぎると、クリープで車速が上がり直す
（094422 の最大逸脱の上位 4 件）。クリープが効く車速帯で基準が停車へ向かっているときだけ、
ブレーキを「動き出す開度」（手順2 で測った ブレーキの min ＋ |launch_breakaway_u_pct|）より
浅くしない。
深くするのは自由（下限だけ）。測った値だけを使い、試験車の値は置かない。

このモジュールは判断だけの純粋なロジック（ハードウェア・mode_drive に依存しない）。
"""

from __future__ import annotations

import bisect
import math
from collections.abc import Sequence
from dataclasses import dataclass

from tests.research.ff_model import STOP_SPEED_KMH


def creep_floor_opening_pct(
    meta: dict, brake_min_pct: float, hold_pct: float
) -> float | None:
    """下限のブレーキ開度 [%] ＝ min ＋ |meta の launch_breakaway_u_pct|。停車保持より深くしない。
    meta に動き出す開度が無ければ None（下限なし）。"""
    u = meta.get("launch_breakaway_u_pct")
    if u is None or not math.isfinite(float(u)):
        return None
    return min(brake_min_pct + abs(float(u)), hold_pct)


class StopArrivals:
    """基準の点列から「次に基準が停車へ着く時刻」を引く（元の基準。ぼかさない）。"""

    def __init__(self, times: Sequence[float], speeds: Sequence[float]) -> None:
        self._arrivals: list[float] = []
        prev_moving = False
        for t, v in zip(times, speeds, strict=True):
            moving = v > STOP_SPEED_KMH
            if prev_moving and not moving:
                self._arrivals.append(float(t))
            prev_moving = moving

    def next_after(self, t_s: float) -> float | None:
        i = bisect.bisect_left(self._arrivals, t_s)
        return self._arrivals[i] if i < len(self._arrivals) else None


@dataclass
class CreepFloorStats:
    """下限の集計。"""

    period_s: float = 0.05
    cycles: int = 0  # 下限が指令を上げた周期数
    entries: int = 0  # 上げ始めた回数
    raise_sum_pct: float = 0.0
    raise_max_pct: float = 0.0
    _active: bool = False

    def add(self, raised_pct: float) -> None:
        if raised_pct > 0.0:
            self.cycles += 1
            self.raise_sum_pct += raised_pct
            self.raise_max_pct = max(self.raise_max_pct, raised_pct)
            if not self._active:
                self.entries += 1
            self._active = True
        else:
            self._active = False

    def summary(self) -> str:
        mean = self.raise_sum_pct / self.cycles if self.cycles else 0.0
        return (
            f"掛けた {self.entries} 回・{self.cycles * self.period_s:.1f}s"
            f"（{self.cycles} 周期）・上げた量 平均 {mean:.2f}% / 最大 {self.raise_max_pct:.2f}%"
        )


class CreepFloor:
    """C14 の下限。`apply` は符号付き effort（負＝ブレーキ）に下限を掛けて返す。"""

    def __init__(
        self,
        *,
        floor_pct: float | None,
        lead_s: float,
        speed_limit_kmh: float,
        arrivals: StopArrivals,
        accel_min_pct: float,
        period_s: float = 0.05,
        accel_tol_pct: float = 0.3,
    ) -> None:
        self.floor_pct = floor_pct
        self.lead_s = lead_s
        self.speed_limit_kmh = speed_limit_kmh  # creep_speed_kmh ＋ margin
        self.arrivals = arrivals
        self.accel_min_pct = accel_min_pct
        self.accel_tol_pct = accel_tol_pct
        self.stats = CreepFloorStats(period_s=period_s)

    def active(
        self, t_s: float, ref_kmh: float, speed_kmh: float, accel_actual_pct: float | None
    ) -> bool:
        """条件 (1)〜(3) とアクセルが min のとき True。(4)（ブレーキ側か）は effort の符号。"""
        if self.floor_pct is None:
            return False
        if ref_kmh <= STOP_SPEED_KMH:  # 停車中は停車保持の行
            return False
        arrival = self.arrivals.next_after(t_s)
        if arrival is None or arrival - t_s > self.lead_s:
            return False
        if ref_kmh > self.speed_limit_kmh or speed_kmh > self.speed_limit_kmh:
            return False
        return not (
            accel_actual_pct is not None
            and accel_actual_pct > self.accel_min_pct + self.accel_tol_pct
        )

    def raise_effort(self, effort: float, on: bool) -> float:
        """effort が負（ブレーキ）で、深さが下限より浅いときだけ下限に上げる。"""
        if on and self.floor_pct is not None and effort < 0.0 and -effort < self.floor_pct:
            return -self.floor_pct
        return effort

    def apply(
        self, t_s: float, ref_kmh: float, speed_kmh: float, effort: float,
        accel_actual_pct: float | None, *, record: bool = True,
    ) -> float:
        out = self.raise_effort(
            effort, self.active(t_s, ref_kmh, speed_kmh, accel_actual_pct)
        )
        if record:
            self.stats.add(-out + effort if out != effort else 0.0)
        return out

    def settings_text(self) -> str:
        if self.floor_pct is None:
            return "下限なし（pkl の meta に launch_breakaway_u_pct が無い。C12 と同じ指令）"
        return (
            f"下限 ブレーキ {self.floor_pct:.2f}%"
            "（min ＋ 動き出す開度の大きさ。停車保持より深くしない）"
            f"・条件 次の停車まで {self.lead_s:g}s 以内かつ"
            f" 基準・実車速 ≤ {self.speed_limit_kmh:.2f} km/h"
            "（クリープ車速 ＋ 低速の余裕）かつ ブレーキ側でアクセルは min"
        )
