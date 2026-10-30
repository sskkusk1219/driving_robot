"""手順2 の BRAKE_SEARCH から、低速の保持用の PI の値（Kp・Ki）を求める。

2026-10-30 UDDS 1252 s の「基準 1.6 km/h 一定」でブレーキが往復する問題（ProblemReport_20261030）。
BRAKE_SEARCH はクリープ車速（5.0 km/h）からブレーキを 0.1% ずつ踏み込んだ記録で、段（指令が
変わった所で切る）ごとに車速が「むだ時間 L ＋ 1 次遅れ」で落ちる。

求め方:
  1. 段: brake_cmd_pct が STEP_JUMP_PCT 以上変わった所で切る（行間 GAP_S 以上空いても切る）。
     各段の t0（始まりの時刻）・Δu（直前の段からの指令の変化 [%]。深くすると正）・
     v0（始まりの車速）。
     最初の段は Δu が決まらないので使わない。
  2. 各段に v(t) = v_end + (v0 − v_end)·exp(−(t − t0 − L)/T)（t ≥ t0 + L。それ以前は v0）を当てる。
     L は pkl meta の brake_lag_s（引数で上書き可）。v_end・T は scipy.optimize.curve_fit。
     R² は t ≥ t0 + L の行で出す。
  3. 使う段: 0 < v0 < クリープ車速、|v0 − v_end| ≥ MIN_CHANGE_KMH、R² ≥ MIN_R2、
     段の長さ ≥ L + MIN_AFTER_LAG_S、
     K_i > 0（向きがそろっている）、0 ≤ v_end（止まった先が負にならない）、T < T_MAX_S（境界に
     張り付いていない。062805 の 83.83 s の段が v_end −50・T 60 で R² 0.87 を通ってしまった）。
     K_i = (v0 − v_end)/Δu [km/h/%]。K・T は使った段の中央値。
  4. PI（SIMC。新しい調整値は作らない）: λ = deviation_c × L、Kp = T / (K × (λ + L))、
     Ti = min(T, 4 × (λ + L))、Ki = Kp / Ti。

     Ti の min は Skogestad の SIMC（Simple IMC）の積分時間の決め方。T が長い（積分器に近い）車で
     Ti = T にすると積分が強すぎて約 6 s 周期で上下した（20261030_063905）ので、T が長いときは
     4 × (λ + L) で頭打ちにする。T = 0.3 s など短いときは min で Ti = T（今までと同じ値）。

もう 1 つの測り方（LOW_HOLD。2026-10-30、手順13 の 4 点の保持のログ）:
  BRAKE_SEARCH の約 1 s の段からは T が短く（0.3 s）見えるが、低速の車はほぼ積分器だった。
  点（elapsed_s の POINT_GAP_S 以上の切れ目で分ける）ごとに、始まりから SKIP_S 秒以降
  （停車保持から動き出す開度へ寄せる間を除く）を RESAMPLE_DT_S 刻みに直し、u は
  brake_actual_pct を L 遅らせ、dv/dt は DIFF_S 差分、v・u はその窓の平均にして
  dv/dt = a + b·v + c·u を最小二乗で当てる。T = −1/b、K = |c/b|。
  使う点: R² ≥ MIN_R2・T > 0・K > 0。K・T は使った点の中央値。MIN_HOLD_POINTS 未満ならエラー。

使える段（点）が足りなければエラー（HoldPiError）。純関数が中心。src/ は import しない。
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

# CSV の phase 名（pattern_loop の _Phase の名前と同じ。持ち込み）
PHASE_BRAKE_SEARCH = "BRAKE_SEARCH"
PHASE_CREEP_WAIT = "CREEP_WAIT"
PHASE_LOW_HOLD = "LOW_HOLD"

# 指令がこれ以上変わったら段を切る [%]（BRAKE_SEARCH の刻みは約 0.1%。ノイズなし）
STEP_JUMP_PCT = 0.02
# 行間がこれ以上空いたら別の塊 [s]（local_gain.GAP_S と同じ値）
GAP_S = 0.5
# 使う段の条件（議論の決定）
MIN_CHANGE_KMH = 0.1
MIN_R2 = 0.6  # 局所の傾きの解析で使った基準
MIN_AFTER_LAG_S = 0.3
MIN_STEPS = 3
# T の探索範囲 [s]（当てはめの境界。仮値）
T_MIN_S = 0.05
T_MAX_S = 60.0
# SIMC の積分時間の上限 Ti ≤ 4 × (λ + L)（Skogestad の既定の係数）
TI_FACTOR = 4.0
# LOW_HOLD（4 点の保持）の測り方
POINT_GAP_S = 1.0  # elapsed_s がこれ以上空いたら別の点
SKIP_S = 5.0  # 点の始まりからこの間は使わない（動き出す開度へ寄せる間）
RESAMPLE_DT_S = 0.05
DIFF_S = 0.5  # dv/dt の差分の幅
# 使う点の最小数: 4 点のうち半分
MIN_HOLD_POINTS = 2
# クリープ車速の既定: CREEP_WAIT の最後のこの秒数の平均 [s]
CREEP_TAIL_S = 2.0


class HoldPiError(Exception):
    """使える段が足りないなど、値を作れない。"""


@dataclass(frozen=True)
class Series:
    """CSV の 1 つの区間（BRAKE_SEARCH）の時系列。"""

    t_s: np.ndarray
    speed_kmh: np.ndarray
    cmd_pct: np.ndarray


@dataclass
class Step:
    """段 1 つ。i0・i1 は Series の行（半開区間）。"""

    i0: int
    i1: int
    t0_s: float
    du_pct: float
    v0_kmh: float


def _f(x: str | None) -> float:
    try:
        return float(x) if x not in (None, "") else math.nan
    except ValueError:
        return math.nan


def read_phase(csv_path: Path, phase: str) -> Series:
    """CSV の phase の行を読む（時刻は elapsed_s）。"""
    t, v, u = [], [], []
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("phase") != phase:
                continue
            t.append(_f(r.get("elapsed_s")))
            v.append(_f(r.get("actual_speed_kmh")))
            u.append(_f(r.get("brake_cmd_pct")))
    a = np.array([t, v, u], dtype=float)
    ok = np.isfinite(a).all(axis=0)
    return Series(a[0][ok], a[1][ok], a[2][ok])


def creep_from_csv(csv_path: Path) -> float:
    """CREEP_WAIT の最後の CREEP_TAIL_S 秒の平均車速 [km/h]。無ければ HoldPiError。"""
    s = read_phase(csv_path, PHASE_CREEP_WAIT)
    if len(s.t_s) == 0:
        raise HoldPiError("CSV に CREEP_WAIT の行がない（--creep-kmh で渡す）")
    return float(s.speed_kmh[s.t_s >= s.t_s[-1] - CREEP_TAIL_S].mean())


def split_steps(s: Series) -> list[Step]:
    """指令が STEP_JUMP_PCT 以上変わった所（と行間 GAP_S 以上の飛び）で段に切る。

    最初の段（直前が無い）は Δu が決まらないので返さない。
    """
    n = len(s.t_s)
    if n < 2:
        return []
    starts = [0]
    for i in range(1, n):
        jump = abs(s.cmd_pct[i] - s.cmd_pct[i - 1]) >= STEP_JUMP_PCT
        if jump or s.t_s[i] - s.t_s[i - 1] >= GAP_S:
            starts.append(i)
    ends = starts[1:] + [n]
    steps: list[Step] = []
    for k in range(1, len(starts)):
        i0, i1 = starts[k], ends[k]
        if s.t_s[i0] - s.t_s[i0 - 1] >= GAP_S:
            continue  # 塊の先頭: 直前の段が別の塊で Δu が決まらない
        du = float(s.cmd_pct[i0] - s.cmd_pct[i0 - 1])
        steps.append(Step(i0, i1, float(s.t_s[i0]), du, float(s.speed_kmh[i0])))
    return steps


def fit_step(
    t_s: np.ndarray, v_kmh: np.ndarray, t0: float, v0: float, lag_s: float
) -> tuple[float, float, float] | None:
    """段に「むだ時間 + 1 次遅れ」を当てて (v_end, T, R²) を返す。当たらなければ None。

    当てる行は t ≥ t0 + lag_s。v0 は固定。R² は同じ行で。
    """
    from scipy.optimize import curve_fit  # noqa: PLC0415

    m = t_s >= t0 + lag_s
    if m.sum() < 3:
        return None
    x, y = t_s[m] - t0 - lag_s, v_kmh[m]

    def model(xx: np.ndarray, v_end: float, tau: float) -> np.ndarray:
        return v_end + (v0 - v_end) * np.exp(-xx / tau)

    try:
        popt, _ = curve_fit(
            model, x, y, p0=(float(y[-1]), 2.0),
            bounds=([-np.inf, T_MIN_S], [np.inf, T_MAX_S]),
        )
    except (RuntimeError, ValueError):
        return None
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    if ss_tot <= 0.0:
        return None
    r2 = 1.0 - float(np.sum((y - model(x, *popt)) ** 2)) / ss_tot
    return float(popt[0]), float(popt[1]), r2


def compute_hold_pi(
    s: Series,
    *,
    lag_s: float,
    creep_kmh: float,
    deviation_c: float,
    source_csv: str = "",
) -> dict[str, Any]:
    """BRAKE_SEARCH の時系列から K・T と PI の値を求める。使える段が MIN_STEPS 未満ならエラー。"""
    steps = split_steps(s)
    per_step: list[dict[str, Any]] = []
    for st in steps:
        t, v = s.t_s[st.i0 : st.i1], s.speed_kmh[st.i0 : st.i1]
        row: dict[str, Any] = {
            "t0_s": st.t0_s, "v0_kmh": st.v0_kmh, "du_pct": st.du_pct,
            "len_s": float(t[-1] - t[0]), "v_end_kmh": math.nan, "K": math.nan,
            "T_s": math.nan, "r2": math.nan, "used": False,
        }
        fit = None
        if st.du_pct > 0 and t[-1] - t[0] >= lag_s + MIN_AFTER_LAG_S:
            fit = fit_step(t, v, st.t0_s, st.v0_kmh, lag_s)
        if fit is not None:
            v_end, tau, r2 = fit
            row.update(v_end_kmh=v_end, T_s=tau, r2=r2, K=(st.v0_kmh - v_end) / st.du_pct)
            row["used"] = bool(
                0.0 < st.v0_kmh < creep_kmh
                and abs(st.v0_kmh - v_end) >= MIN_CHANGE_KMH
                and r2 >= MIN_R2
                and row["K"] > 0.0
                and v_end >= 0.0
                and tau < T_MAX_S
            )
        per_step.append(row)
    used = [r for r in per_step if r["used"]]
    if len(used) < MIN_STEPS:
        raise HoldPiError(
            f"使える段が {len(used)} 件（全 {len(per_step)} 段）で {MIN_STEPS} 件に満たない。"
            "値を作らない"
        )
    k = float(np.median([r["K"] for r in used]))
    tau = float(np.median([r["T_s"] for r in used]))
    return {
        **pi_values(k, tau, lag_s, deviation_c),
        "source": PHASE_BRAKE_SEARCH,
        "steps_used": len(used), "steps_total": len(per_step),
        "creep_kmh": creep_kmh, "per_step": per_step, "source_csv": source_csv,
    }


def pi_values(k: float, tau: float, lag_s: float, deviation_c: float) -> dict[str, Any]:
    """K・T・L から SIMC の PI の値を作る（両方の測り方で共通）。

    Skogestad の SIMC: λ = deviation_c × L、Kp = T / (K × (λ + L))、
    Ti = min(T, 4 × (λ + L))、Ki = Kp / Ti。積分器に近い車（T が長い）で Ti = T だと積分が
    強すぎて約 6 s 周期で上下した（063905）ので T が長いときは 4 × (λ + L) で頭打ちにする。
    T = 0.3 s のように短ければ min は T で、今までと同じ値になる。
    """
    lam = deviation_c * lag_s
    kp = tau / (k * (lam + lag_s))
    ti = min(tau, TI_FACTOR * (lam + lag_s))
    return {
        "K_kmh_per_pct": k, "T_s": tau, "L_s": lag_s, "lambda_s": lam,
        "kp_pct_per_kmh": kp, "Ti_s": ti, "ki_pct_per_kmh_s": kp / ti,
    }


def split_points(t_s: np.ndarray) -> list[tuple[int, int]]:
    """時刻が POINT_GAP_S 以上空いた所で点に分ける（行の半開区間のリスト）。"""
    if len(t_s) == 0:
        return []
    cuts = [0] + [i for i in range(1, len(t_s)) if t_s[i] - t_s[i - 1] >= POINT_GAP_S]
    return list(zip(cuts, cuts[1:] + [len(t_s)], strict=True))


def fit_point(
    t_s: np.ndarray, v_kmh: np.ndarray, u_pct: np.ndarray, lag_s: float
) -> tuple[float, float, float] | None:
    """1 点の dv/dt = a + b·v + c·u を最小二乗で当てて (K, T, R²) を返す。足りなければ None。

    t_s は点の始まりが先頭。SKIP_S 以降を RESAMPLE_DT_S 刻みに直し、u は lag_s 遅らせる
    （t − L の値）。dv/dt は DIFF_S 差分、v・u はその窓の平均。b == 0 なら None。
    """
    t0 = t_s[0] + SKIP_S
    t_end = t_s[-1]
    if t_end - t0 < DIFF_S * 4:
        return None
    grid = np.arange(t0, t_end, RESAMPLE_DT_S)
    v = np.interp(grid, t_s, v_kmh)
    u = np.interp(grid - lag_s, t_s, u_pct)
    n = int(round(DIFF_S / RESAMPLE_DT_S))
    if len(grid) <= n + 3:
        return None
    dv = (v[n:] - v[:-n]) / DIFF_S
    cs_v = np.concatenate([[0.0], np.cumsum(v)])
    cs_u = np.concatenate([[0.0], np.cumsum(u)])
    vm = (cs_v[n + 1 :] - cs_v[: -n - 1]) / (n + 1)
    um = (cs_u[n + 1 :] - cs_u[: -n - 1]) / (n + 1)
    x = np.column_stack([np.ones_like(vm), vm, um])
    coef, *_ = np.linalg.lstsq(x, dv, rcond=None)
    ss_tot = float(np.sum((dv - dv.mean()) ** 2))
    if ss_tot <= 0.0 or coef[1] == 0.0:
        return None
    r2 = 1.0 - float(np.sum((dv - x @ coef) ** 2)) / ss_tot
    b, c = float(coef[1]), float(coef[2])
    return abs(c / b), -1.0 / b, r2


def compute_hold_pi_low_hold(
    s: Series, *, lag_s: float, deviation_c: float, source_csv: str = ""
) -> dict[str, Any]:
    """LOW_HOLD の時系列（cmd_pct の所に brake_actual_pct を入れた Series）から K・T・PI を求める。

    使える点が MIN_HOLD_POINTS 未満ならエラー。
    """
    per_point: list[dict[str, Any]] = []
    for i0, i1 in split_points(s.t_s):
        t, v, u = s.t_s[i0:i1], s.speed_kmh[i0:i1], s.cmd_pct[i0:i1]
        row: dict[str, Any] = {
            "t0_s": float(t[0]), "len_s": float(t[-1] - t[0]), "v_mean_kmh": float(v.mean()),
            "u_mean_pct": float(u.mean()), "K": math.nan, "T_s": math.nan, "r2": math.nan,
            "used": False,
        }
        fit = fit_point(t, v, u, lag_s)
        if fit is not None:
            k, tau, r2 = fit
            row.update(K=k, T_s=tau, r2=r2)
            row["used"] = bool(r2 >= MIN_R2 and tau > 0.0 and k > 0.0)
        per_point.append(row)
    used = [r for r in per_point if r["used"]]
    if len(used) < MIN_HOLD_POINTS:
        raise HoldPiError(
            f"使える点が {len(used)} 件（全 {len(per_point)} 点）で "
            f"{MIN_HOLD_POINTS} 件に満たない。値を作らない"
        )
    k = float(np.median([r["K"] for r in used]))
    tau = float(np.median([r["T_s"] for r in used]))
    return {
        **pi_values(k, tau, lag_s, deviation_c),
        "source": PHASE_LOW_HOLD,
        "steps_used": len(used), "steps_total": len(per_point),
        "per_step": per_point, "source_csv": source_csv,
    }


def read_low_hold(csv_path: Path) -> Series:
    """CSV の LOW_HOLD の行を読む。cmd_pct の所に brake_actual_pct（実開度）を入れる。"""
    t, v, u = [], [], []
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("phase") != PHASE_LOW_HOLD:
                continue
            t.append(_f(r.get("elapsed_s")))
            v.append(_f(r.get("actual_speed_kmh")))
            u.append(_f(r.get("brake_actual_pct")))
    a = np.array([t, v, u], dtype=float)
    ok = np.isfinite(a).all(axis=0)
    return Series(a[0][ok], a[1][ok], a[2][ok])


def hold_pi_low_hold_from_csv(
    csv_path: Path, *, lag_s: float, deviation_c: float
) -> dict[str, Any]:
    """手順13 の CSV（LOW_HOLD）から求める。"""
    s = read_low_hold(csv_path)
    if len(s.t_s) == 0:
        raise HoldPiError("CSV に LOW_HOLD の行がない")
    return compute_hold_pi_low_hold(
        s, lag_s=lag_s, deviation_c=deviation_c, source_csv=Path(csv_path).name
    )


def attach_hold_pi_low_hold(
    meta: dict[str, Any], csv_path: Path, *, deviation_c: float
) -> dict[str, Any]:
    """CSV の LOW_HOLD から PI を求めて meta["hold_pi"] に入れる（fwd_tool attach-hold-pi
    --phase LOW_HOLD と同じ。L は meta の brake_lag_s）。手順2 の自動付与が使う。
    brake_lag_s が無い・有限でない・LOW_HOLD の行が無い・使える点が足りないときは HoldPiError。"""
    lag = meta.get("brake_lag_s")
    if lag is None or not math.isfinite(float(lag)):
        raise HoldPiError("brake_lag_s が pkl の meta に無い")
    res = hold_pi_low_hold_from_csv(Path(csv_path), lag_s=float(lag), deviation_c=deviation_c)
    meta["hold_pi"] = res
    return res


def point_table_lines(per_point: list[dict[str, Any]]) -> list[str]:
    """点ごとの表（Markdown の行）。"""
    lines = [
        "| t0 s | 長さ s | v 平均 km/h | u 平均 % | K km/h/% | T s | R² | 使った |",
        "|" + "---|" * 8,
    ]
    for r in per_point:
        lines.append(
            f"| {r['t0_s']:.2f} | {r['len_s']:.2f} | {r['v_mean_kmh']:.3f} "
            f"| {r['u_mean_pct']:.3f} | {r['K']:.2f} | {r['T_s']:.2f} | {r['r2']:.2f} "
            f"| {'○' if r['used'] else '-'} |"
        )
    return lines


def hold_pi_from_csv(
    csv_path: Path, *, lag_s: float, deviation_c: float, creep_kmh: float | None = None
) -> dict[str, Any]:
    """手順2 の CSV から求める（creep_kmh が None なら CREEP_WAIT の最後の 2 s の平均）。"""
    s = read_phase(csv_path, PHASE_BRAKE_SEARCH)
    if len(s.t_s) == 0:
        raise HoldPiError("CSV に BRAKE_SEARCH の行がない")
    creep = creep_from_csv(csv_path) if creep_kmh is None else float(creep_kmh)
    return compute_hold_pi(
        s, lag_s=lag_s, creep_kmh=creep, deviation_c=deviation_c, source_csv=Path(csv_path).name
    )


def step_table_lines(per_step: list[dict[str, Any]]) -> list[str]:
    """段ごとの表（Markdown の行）。"""
    lines = [
        "| t0 s | v0 km/h | Δu % | 長さ s | v_end km/h | K_i km/h/% | T_i s | R² | 使った |",
        "|" + "---|" * 9,
    ]
    for r in per_step:
        lines.append(
            f"| {r['t0_s']:.2f} | {r['v0_kmh']:.3f} | {r['du_pct']:.3f} | {r['len_s']:.2f} "
            f"| {r['v_end_kmh']:.3f} | {r['K']:.3f} | {r['T_s']:.2f} | {r['r2']:.2f} "
            f"| {'○' if r['used'] else '-'} |"
        )
    return lines
