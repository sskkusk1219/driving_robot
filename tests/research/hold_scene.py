"""保持の場面（基準が低速で平ら）の PI と保持の開度の FF
（2026-10-30 ProblemReport_20261030 段4・7・8・11）。

UDDS 1251〜1256 s（基準 0→1.6 km/h、その後 4 s 一定）で、FF のブレーキが動き出す開度で頭打ちのまま、
C10 の P と DOB の上乗せが車速の谷で跳ね、ブレーキが 0.5 s 周期で往復する。保持の場面だけ、
P・DOB・惰行の補助・launch_move_hold の足しを止めて、
開度の単位の PI（偏差 e ＝ 基準 − 実車速）に替える。

  effort ＝ FF ＋ Kp·e ＋ I（I += Ki·e·dt）。effort はブレーキが負（その絶対値が開度）。
  e > 0（遅い）→ effort が正の向き＝ブレーキが浅くなる。

pkl の meta:
  hold_pi: fwd_tool attach-hold-pi が作る
    （kp_pct_per_kmh・ki_pct_per_kmh_s・L_s など。hold_pi.py）。
  hold_opening: 手順13 が作る予定の保持の開度（段6）。形は
    {"points": [{"speed_kmh": float, "opening_pct": float, "held": bool, ...}],
     "creep_kmh": float, "breakaway_pct": float, "min_pct": float, "source_csv": str}
    補間に使うのは held の点と両端（0 km/h → breakaway_pct、creep_kmh → min_pct）。
    opening_pct は絶対開度（ブレーキ。正）。

上り（段11・案 A）: 保持の区間の直前で、基準が停車速度〜クリープ車速の中にあり保持の車速へ
単調に向かう行（UDDS 1251〜1252 s の 0 → 1.6 km/h）も保持の場面にする（hold_lead_in_intervals）。
上りの FF ＝ 保持の開度（L 先の基準）− L 先の基準の傾き ÷ c（c ＝ K ÷ T。lead_in_ff_pct）。
PI は上りから保持の区間へ続けて使う（境目で I を捨てない）。

先読みの P（段13・案 B）: 加速中に保持の区間へ入ると、今の偏差だけを見る P は偏差の符号が変わるまで
反転せず行き過ぎる（1.6 → 2.2 km/h、FF が +0.3% ずれると 0.75〜2.5 km/h を約 4 s 周期で往復）。
P の偏差だけを「Tp 秒先の基準 −（今の車速 ＋ 実測の加速度 × Tp）」に替える（predicted_deviation）。
Tp ＝ L ＋ 窓 ÷ 2（加速度を測ってから効くまでの遅れ）。
加速度は帯（±ramp_cruise_band_g）の中を 0 とする（banded_accel）。
積分・不感帯・巻き上がり防止は今の偏差のまま。

純関数と小さな状態機械だけ。src/ は import しない。
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

G_MS2 = 9.80665


def slope_threshold_kmhs(band_g: float) -> float:
    """基準の傾きのしきい値 [km/h/s]（band_g [G] を換算。hold_check と同じ値）。"""
    return band_g * G_MS2 * 3.6


def is_hold_scene(
    ref_now_kmh: float,
    ref_ahead_kmh: float,
    lag_s: float,
    *,
    slope_max_kmhs: float,
    stop_kmh: float,
    creep_kmh: float,
) -> bool:
    """保持の場面か。基準（ぼかす前）の t と t＋L の差の大きさ ÷ L ≤ しきい値、かつ
    停車速度 < ref(t) < クリープ車速。L ≤ 0 は判定できないので False。"""
    if not lag_s > 0.0:
        return False
    slope = abs(ref_ahead_kmh - ref_now_kmh) / lag_s
    return slope <= slope_max_kmhs and stop_kmh < ref_now_kmh < creep_kmh


# 区間の長さの下限 ＝ この倍率 ×（λ ＋ L）。PI の応答時間 λ＋L の 2 倍より短い平らな区間では、
# PI が落ち着く前に区間が終わるので、保持の場面にせず今の P・DOB に任せる。
# 2026-10-30 UDDS 060815 で 727 s（0.9 s）・763 s（0.9 s）が場面に入り、
# 0.44 → 1.10・0.91 に悪化した。
MIN_SCENE_RESPONSE_MULT = 2.0


def min_scene_length_s(lambda_s: float, lag_s: float) -> float:
    """保持の場面にする平らな区間の長さの下限 [s] ＝ 2 ×（λ ＋ L）。"""
    return MIN_SCENE_RESPONSE_MULT * (lambda_s + lag_s)


@dataclass(frozen=True)
class HoldInterval:
    """保持の場面の区間。start_s ＝ 最初の周期の時刻、length_s ＝ 最後 − 最初の時刻 ＋ 刻み。"""

    start_s: float
    length_s: float
    mean_kmh: float

    @property
    def end_s(self) -> float:
        """区間の終わり（含まない）。"""
        return self.start_s + self.length_s


def hold_scene_intervals(
    ref_at: Callable[[float], float],
    duration_s: float,
    step_s: float,
    lag_s: float,
    *,
    slope_max_kmhs: float,
    stop_kmh: float,
    creep_kmh: float,
    min_length_s: float,
) -> tuple[list[HoldInterval], int]:
    """走行前に基準全体から保持の場面の区間を求める。時刻 k·step_s（0〜duration_s）ごとに
    is_hold_scene を調べ、連続した区間のうち長さ ≥ min_length_s のものを返す
    （(残した区間, 外した区間の数)）。長さ ＝ 最後の時刻 − 最初の時刻 ＋ 刻み。"""
    kept: list[HoldInterval] = []
    dropped = 0
    n = int(math.floor(duration_s / step_s + 1e-9)) + 1
    run_start: int | None = None
    speeds: list[float] = []

    def close(last: int) -> None:
        nonlocal dropped, run_start, speeds
        assert run_start is not None
        length = (last - run_start) * step_s + step_s
        # 境目ちょうど（浮動小数の丸め）は残す側に倒す
        if length >= min_length_s - 1e-9:
            kept.append(HoldInterval(run_start * step_s, length, sum(speeds) / len(speeds)))
        else:
            dropped += 1
        run_start, speeds = None, []

    for k in range(n):
        t = k * step_s
        r = ref_at(t)
        if is_hold_scene(
            r, ref_at(t + lag_s), lag_s,
            slope_max_kmhs=slope_max_kmhs, stop_kmh=stop_kmh, creep_kmh=creep_kmh,
        ):
            if run_start is None:
                run_start = k
            speeds.append(r)
        elif run_start is not None:
            close(k - 1)
    if run_start is not None:
        close(n - 1)
    return kept, dropped


def in_hold_interval(intervals: list[HoldInterval], t_s: float, step_s: float) -> bool:
    """t が残った区間のどれかに入っているか（最初の周期から最後の周期まで）。"""
    tol = 0.5 * step_s
    return any(iv.start_s - tol <= t_s <= iv.end_s - step_s + tol for iv in intervals)


def hold_lead_in_intervals(
    ref_at: Callable[[float], float],
    intervals: list[HoldInterval],
    step_s: float,
    *,
    stop_kmh: float,
    creep_kmh: float,
) -> list[HoldInterval]:
    """残った保持の区間ごとに、直前の上り（lead-in）の区間を求める（段11）。区間の手前の周期から
    過去へ歩き、停車速度 < 基準 < クリープ車速、かつ基準が区間の車速（mean_kmh）へ単調に近づく
    （上りも下りも可）間だけ含める。前の保持の区間には入らない。1 周期も無ければ省く。
    start_s ＝ 最初の周期の時刻、length_s ＝ 周期数 × 刻み、mean_kmh ＝ 向かう先の保持の車速。"""
    out: list[HoldInterval] = []
    for i, iv in enumerate(intervals):
        prev_end = intervals[i - 1].end_s if i > 0 else 0.0
        k = int(round(iv.start_s / step_s)) - 1
        first_sign = 0.0
        next_gap: float | None = None  # 1 周期先の |mean − ref|
        n = 0
        while k >= 0:
            t = k * step_s
            if t < prev_end - 0.5 * step_s:
                break
            r = ref_at(t)
            if not stop_kmh < r < creep_kmh:
                break
            gap = iv.mean_kmh - r
            if n == 0:
                first_sign = math.copysign(1.0, gap) if gap != 0.0 else 0.0
            elif next_gap is not None and abs(gap) < next_gap - 1e-9:
                break  # 保持の車速から遠ざかる
            if first_sign != 0.0 and gap != 0.0 and math.copysign(1.0, gap) != first_sign:
                break
            next_gap = abs(gap)
            n += 1
            k -= 1
        if n > 0:
            out.append(HoldInterval((k + 1) * step_s, n * step_s, iv.mean_kmh))
    return out


def lead_in_ff_pct(
    points: list[tuple[float, float]],
    ref_ahead_kmh: float,
    slope_ahead_kmhs: float,
    c_kmhs_per_pct: float,
    lo_pct: float,
    hi_pct: float,
) -> float:
    """上りの FF（ブレーキ開度。正 [%]）＝ 保持の開度（L 先の基準）− 傾き ÷ c を
    [lo, hi] に収めた値。
    基準が上がる（傾き > 0）ほどブレーキを浅くする。c ≤ 0・有限でなければ傾きの項は 0。"""
    base = interp_hold_opening(points, ref_ahead_kmh)
    slope_term = 0.0
    if math.isfinite(c_kmhs_per_pct) and c_kmhs_per_pct > 0.0 and math.isfinite(slope_ahead_kmhs):
        slope_term = slope_ahead_kmhs / c_kmhs_per_pct
    return min(max(base - slope_term, lo_pct), hi_pct)


def hold_opening_points(meta: Any) -> list[tuple[float, float]] | None:
    """meta["hold_opening"] の補間に使う点（車速 [km/h]・ブレーキ開度 [%]。車速の昇順）。
    held の点と両端（0 → breakaway_pct、creep_kmh → min_pct）。meta が無い・壊れていれば None。"""
    info = meta.get("hold_opening") if hasattr(meta, "get") else None
    if not info:
        return None
    try:
        creep = float(info["creep_kmh"])
        pts = [(0.0, float(info["breakaway_pct"]))]
        for p in info.get("points", []):
            v = float(p["speed_kmh"])
            if p.get("held") and 0.0 < v < creep:
                pts.append((v, float(p["opening_pct"])))
        pts.append((creep, float(info["min_pct"])))
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(x) for pt in pts for x in pt):
        return None
    return sorted(pts)


def interp_hold_opening(points: list[tuple[float, float]], speed_kmh: float) -> float:
    """保持の開度（ブレーキ。正 [%]）を基準車速で線形補間（両端の外は端の値）。"""
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return float(np.interp(speed_kmh, xs, ys))


def predicted_deviation(
    ref_ahead_kmh: float, speed_kmh: float, accel_kmhs: float, tp_s: float
) -> float:
    """段13: P に使う先読みの偏差 ＝ Tp 秒先の基準 −（今の車速 ＋ 加速度 × Tp）[km/h]。"""
    return ref_ahead_kmh - (speed_kmh + accel_kmhs * tp_s)


def banded_accel(accel_kmhs: float | None, band_kmhs: float) -> float:
    """段13: 実測の加速度 [km/h/s]。None・|a| ≤ 帯 は 0（段に止まっている）、それ以外は a。"""
    if accel_kmhs is None or abs(accel_kmhs) <= band_kmhs:
        return 0.0
    return accel_kmhs


@dataclass
class HoldScenePI:
    """保持の場面の PI。1 周期ずつ step() を呼ぶ。場面の外の周期は reset()（I を捨てる）。"""

    kp: float  # [%/(km/h)]
    ki: float  # [%/(km/h·s)]
    dt_s: float
    lo_pct: float  # ブレーキ側の浅い端（min のブレーキ開度。正）
    hi_pct: float  # ブレーキ側の深い端（停車保持の開度。正）
    integral: float = 0.0
    active: bool = False
    # |偏差| ≤ これ [km/h] の周期は積分を進めない（P の項はそのまま）。0 ＝ 不感帯なし（今と同じ）
    deadband_kmh: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.active = False

    def step(
        self, dev_kmh: float, ff_effort: float, p_dev_kmh: float | None = None
    ) -> tuple[float, float]:
        """(最終の effort, PI の上乗せ) を返す。effort は [−hi, −lo] に収める。
        収めたとき、さらに外へ向かう積分は進めない（巻き上がり防止）。
        場面に入った最初の周期は I ＝ 0（active が False のとき）。
        |偏差| ≤ deadband_kmh の周期は積分を進めない。
        p_dev_kmh（段13）が None でなければ P の項だけその偏差を使う（積分・不感帯は dev_kmh）。"""
        p_dev = dev_kmh if p_dev_kmh is None else p_dev_kmh
        if not self.active:
            self.integral = 0.0
            self.active = True
        in_deadband = self.deadband_kmh > 0.0 and abs(dev_kmh) <= self.deadband_kmh
        step_i = 0.0 if in_deadband else self.ki * dev_kmh * self.dt_s
        raw = ff_effort + self.kp * p_dev + self.integral + step_i
        lo, hi = -self.hi_pct, -self.lo_pct
        out_high, out_low = raw > hi, raw < lo
        if not ((out_high and step_i > 0.0) or (out_low and step_i < 0.0)):
            self.integral += step_i
        effort = min(max(ff_effort + self.kp * p_dev + self.integral, lo), hi)
        return effort, effort - ff_effort
