"""C15（C12 ＋ 停A ＋ 停C）の停車まぎわの判断（2026-10-20 段4-2）。

C12 は基準が停車（0 km/h）になった瞬間に停車保持の開度へ踏むが、ブレーキが効くまでに遅れがあり、
その 2 秒前のブレーキは浅いので、基準 0 の時点で実車がまだ動いている（0.3〜2.8 km/h）。
- 停A（停車保持を早める）: 停車保持の開度を、ブレーキの遅れ分だけ早く指令する。開度の列を
  作る `fwd_plan.build_plan(stop_lead=True)` が行う。ここは集計（`StopLeadStats`）だけ持つ。
- 停C（停車まぎわの P）: 区間は、基準 ≤ 停止側の表の上の車速（meta の stop_max_speed_kmh）で、
  基準が creep_floor_lead_s 以内に 0 になる行（停車保持の行は除く）。この区間では C10 の
  P のブレーキ側の τ を「基準が 0 になるまでの残り時間（ブレーキの遅れより短くしない）」に
  置き換え、残り時間で偏差を消す強さにする。ブレーキ min より浅くせず、停車保持より深くせず、
  アクセルへは移らない（`clamp`）。

値はすべて測った値（pkl の meta・既存の yaml のキー）から。試験車の値は置かない。
このモジュールは判断と集計だけの純粋なロジック（ハードウェア・mode_drive に依存しない）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from tests.research.creep_floor import StopArrivals


@dataclass
class StopLeadStats:
    """停A の集計: 停車保持へ早めに入れた周期の数・回数（連続した周期のまとまり）。"""

    period_s: float = 0.05
    cycles: int = 0
    entries: int = 0
    _active: bool = False

    def add(self, active: bool) -> None:
        if active:
            self.cycles += 1
            if not self._active:
                self.entries += 1
        self._active = active

    def summary(self) -> str:
        return f"働いた {self.entries} 回・{self.cycles * self.period_s:.1f}s（{self.cycles} 周期）"


@dataclass
class StopEndStats:
    """停C の集計: 働いた周期の数・回数と、ブレーキ開度の変化（停C の指令 − 通常の τ の指令。
    正＝深くなった）の平均・最大。"""

    period_s: float = 0.05
    cycles: int = 0
    entries: int = 0
    delta_sum_pct: float = 0.0
    delta_abs_sum_pct: float = 0.0
    delta_abs_max_pct: float = 0.0
    _active: bool = False

    def add(self, active: bool, delta_pct: float = 0.0) -> None:
        if active:
            self.cycles += 1
            self.delta_sum_pct += delta_pct
            self.delta_abs_sum_pct += abs(delta_pct)
            self.delta_abs_max_pct = max(self.delta_abs_max_pct, abs(delta_pct))
            if not self._active:
                self.entries += 1
        self._active = active

    def summary(self) -> str:
        n = max(self.cycles, 1)
        return (
            f"働いた {self.entries} 回・{self.cycles * self.period_s:.1f}s（{self.cycles} 周期）"
            f"・ブレーキ開度の変化（深い向きが正） 平均 {self.delta_sum_pct / n:+.2f}%"
            f"（大きさの平均 {self.delta_abs_sum_pct / n:.2f}%）/ 大きさの最大 "
            f"{self.delta_abs_max_pct:.2f}%"
        )


class StopEndP:
    """停C。`tau_s` が区間内なら τ を返し、`clamp` が結果の開度を範囲に収める。"""

    def __init__(
        self,
        *,
        top_kmh: float | None,
        lead_s: float,
        brake_lag_s: float | None,
        hold_pct: float,
        brake_min_pct: float,
        arrivals: StopArrivals,
        period_s: float = 0.05,
    ) -> None:
        # 停止側の表の上の車速（meta の stop_max_speed_kmh）。None ＝ 停Cなし
        self.top_kmh = top_kmh
        self.lead_s = lead_s  # 基準が 0 になるまでこの秒数以内を区間とする（creep_floor_lead_s）
        self.brake_lag_s = brake_lag_s  # τ の下限。None ＝ 停Cなし
        self.hold_pct = hold_pct  # 停車保持の開度（これより深くしない）
        self.brake_min_pct = brake_min_pct
        self.arrivals = arrivals
        self.stats = StopEndStats(period_s=period_s)

    @property
    def enabled(self) -> bool:
        return (
            self.top_kmh is not None and math.isfinite(self.top_kmh) and self.top_kmh > 0.0
            and self.brake_lag_s is not None and math.isfinite(self.brake_lag_s)
        )

    def tau_s(self, t_s: float, ref_kmh: float, is_stop_row: bool) -> float | None:
        """区間内なら τ ＝ max(基準が 0 になるまでの残り時間, ブレーキの遅れ)、区間外は None。
        ref_kmh は元の基準（ぼかさない）。is_stop_row は計画が停車保持の行（停A の行も含む）。"""
        if not self.enabled or is_stop_row:
            return None
        assert self.top_kmh is not None and self.brake_lag_s is not None
        if ref_kmh > self.top_kmh:
            return None
        arrival = self.arrivals.next_after(t_s)
        if arrival is None or arrival - t_s > self.lead_s:
            return None
        return max(arrival - t_s, self.brake_lag_s)

    def clamp(self, effort_pct: float, plan_effort_pct: float) -> float:
        """停C の結果の effort（負＝ブレーキ）を範囲に収める。min より浅くせず（0 側へは
        行かない＝アクセルへ移らない）、停車保持より深くしない。計画の列がすでに停車保持より
        深いときは、その深さまでは許す（列より浅くなる向きに丸めない）。"""
        deepest = min(-self.hold_pct, plan_effort_pct)
        return min(-self.brake_min_pct, max(effort_pct, deepest))

    def settings_text(self) -> str:
        if not self.enabled:
            return (
                "停Cなし（pkl の meta に stop_max_speed_kmh か brake_lag_s が無い。C12 と同じ指令）"
            )
        return (
            f"区間 基準 ≤ {self.top_kmh:.2f} km/h（停止側の表の上）かつ {self.lead_s:g}s 以内に"
            f"基準が 0 へ（停車保持の行を除く）・τ ＝ max(残り時間, ブレーキの遅れ "
            f"{self.brake_lag_s:.3f}s)・開度の範囲 ブレーキ {self.brake_min_pct:.2f}% 〜 "
            f"停車保持 {self.hold_pct:.2f}%"
        )


def cap_stop_end_tau(
    tau_end_s: float | None, normal_tau_s: float | None, enabled: bool
) -> float | None:
    """停C の τ の上限（2026-10-23 段6-2 (c)）。enabled なら min(普段の τ, 停C の τ)。
    区間外（None）・普段の τ なし・enabled=False は、そのまま返す（今までと同じ）。"""
    if not enabled or tau_end_s is None or normal_tau_s is None:
        return tau_end_s
    return min(tau_end_s, float(normal_tau_s))
