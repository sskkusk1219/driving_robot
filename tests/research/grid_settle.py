"""格子ステップ走行の「落ち着き待ち」と「傾きの当てはめ」の集計（ProblemReport_20260925 段4b）。

`learning.grid_settle_tol_kmh`（±1 km/h）・`grid_settle_s`（3 s）・`grid_step_window_s`（3 s）は、
段3 で置いた仮の値でデータの根拠が無い。実機の 1 本目でこれらを決め直せるように、走行中に
次の 2 つを記録して、2-1 の終わりに表で出す。

    落ち着き待ち: 車速を保つ PI が目標に落ち着くまでの待ち時間・許容幅の中にいた割合・
                 落ち着いた窓の開度のばらつき（この平均が u0 = 加速度 0 の開度になる。
                 ばらつきが大きいと踏み増しがずれる）
    当てはめ    : ステップ 1 本の傾きを何点・どれだけの残差で当てはめたか（窓が短すぎないかの目安）

`pattern_loop.PatternLoop` が走行中に `SettleRecord` を作る（`GridPlanner.results` の
`StepResult` は当てはめ情報を持つ）。ここは集計と表示だけ（ハード非依存）。
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass

from tests.research.grid_planner import StepResult


@dataclass(frozen=True)
class SettleRecord:
    """PI が目標に落ち着くのを待った 1 回ぶん。"""

    station_kmh: float  # ステーションの中心車速
    target_kmh: float  # PI の目標（助走なら帯の端の車速）
    approach: bool  # 助走の車速で待ったか
    wait_s: float  # 待ち始めから落ち着く（または打ち切り）まで
    inside_frac: float  # 待っている間、許容幅の中にいた周期の割合
    u_mean_pct: float  # 落ち着いた窓の開度の平均（打ち切りは nan）
    u_std_pct: float  # 同じ窓の開度の標準偏差（打ち切りは nan）
    timed_out: bool  # 落ち着かず打ち切ったか
    slope_kmhs: float = float("nan")  # 落ち着いた窓の車速の傾き [km/h/s]


def _median(values: list[float]) -> float:
    return statistics.median(values) if values else float("nan")


def _fmt(value: float, spec: str = ".1f") -> str:
    return "—" if math.isnan(value) else format(value, spec)


def settle_table(records: list[SettleRecord]) -> str:
    """落ち着き待ちを 1 回 1 行で。"""
    lines = ["  車速   目標  種別   待ち[s]  幅内[%]  開度 平均[%] (σ)   窓の傾き   結果"]
    for r in records:
        u = "—" if math.isnan(r.u_mean_pct) else f"{r.u_mean_pct:6.2f} ({r.u_std_pct:.2f})"
        lines.append(
            f"  {r.station_kmh:5.1f} {r.target_kmh:5.1f}  {'助走' if r.approach else '中心'}  "
            f"{r.wait_s:7.1f}  {100 * r.inside_frac:6.0f}  {u:>18}  "
            f"{_fmt(r.slope_kmhs, '+.2f'):>8}   "
            f"{'打切り' if r.timed_out else '落ち着いた'}"
        )
    return "\n".join(lines)


def _group_summary(label: str, records: list[SettleRecord]) -> str:
    if not records:
        return f"  {label}: なし"
    waits = [r.wait_s for r in records]
    inside = [100 * r.inside_frac for r in records]
    stds = [r.u_std_pct for r in records if not math.isnan(r.u_std_pct)]
    timeouts = sum(r.timed_out for r in records)
    slopes = [abs(r.slope_kmhs) for r in records if not math.isnan(r.slope_kmhs)]
    return (
        f"  {label}: {len(records)} 回（打切り {timeouts}）、待ち 中央 {_median(waits):.1f}s / "
        f"最大 {max(waits):.1f}s、幅内 中央 {_median(inside):.0f}%、"
        f"開度の窓内 σ 中央 {_fmt(_median(stds), '.2f')}% / "
        f"最大 {_fmt(max(stds, default=math.nan), '.2f')}%、"
        f"窓の傾き |中央| {_fmt(_median(slopes), '.2f')} / 最大 "
        f"{_fmt(max(slopes, default=math.nan), '.2f')} km/h/s"
    )


def fit_summary(results: list[StepResult]) -> str:
    """ステップの傾きの当てはめ（点数と残差）。定速・惰行・狙いのあるステップすべて。"""
    fitted = [r for r in results if r.fit_points > 0]
    if not fitted:
        return "  ステップの当てはめ: 記録なし"
    points = [float(r.fit_points) for r in fitted]
    resid = [r.fit_resid_kmh for r in fitted if not math.isnan(r.fit_resid_kmh)]
    strong = [r for r in fitted if not math.isnan(r.approach_kmh)]
    text = (
        f"  ステップの当てはめ: {len(fitted)} 本、点数 中央 {_median(points):.0f} / "
        f"最小 {min(points):.0f}、車速の残差 σ 中央 {_fmt(_median(resid), '.2f')} / "
        f"最大 {_fmt(max(resid, default=math.nan), '.2f')} km/h"
    )
    if strong:
        s_points = [float(r.fit_points) for r in strong]
        text += (
            f"\n  うち助走つき {len(strong)} 本: 点数 中央 {_median(s_points):.0f} / "
            f"最小 {min(s_points):.0f}"
        )
    return text


def settle_report(
    records: list[SettleRecord], results: list[StepResult], *, tol_kmh: float, settle_s: float,
    duration_s: float,
) -> str:
    """2-1 の終わりに出す落ち着き判定の集計（段5 で値を決め直す材料）。"""
    if not records:
        return ""
    total_wait = sum(r.wait_s for r in records)
    center = [r for r in records if not r.approach]
    approach = [r for r in records if r.approach]
    return "\n".join([
        f"── 落ち着き判定の集計（許容幅 ±{tol_kmh:g} km/h・{settle_s:g} s。決め直す材料） ──",
        settle_table(records),
        f"  待ち時間の合計 {total_wait:.0f}s（パターン走行 {duration_s:.0f}s の "
        f"{100 * total_wait / max(duration_s, 1e-9):.0f}%。最初は停車からの加速を含む）",
        _group_summary("中心", center),
        _group_summary("助走", approach),
        fit_summary(results),
    ])


__all__ = ["SettleRecord", "fit_summary", "settle_report", "settle_table"]
