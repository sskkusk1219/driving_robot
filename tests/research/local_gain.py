"""手順2 の dither（効き始めの少し先での小さな開度の動き）から、局所の効きと効くまでの遅れを測る。

2026-10-28 段3（ProblemReport_20261028）。議論で、効き始め〜＋約 3%（ブレーキ）では場面の中の
小さな動きの効きが表（G ランプの傾き）の 2.1〜2.7 倍あると分かった。C10 は「足りない加速度 ÷
表の傾き」で踏む量を決めるので、この帯では踏みすぎる。そこで手順2 の dither（段2）の走りから
「局所の効き ÷ 表の傾き」の倍率を、効き始めからの開度 × 車速の粗い表にして pkl の meta に残す。
段4 で FB の上乗せを割る（引く側 `local_gain_at`）。

求め方:
  1. 区間: CSV の phase が RAMP_BRAKE_DITHER（または RAMP_ACCEL_DITHER）の連続区間 1 つ = 1 回。
     中には指令の開度の段（u0±Δ, u0 など）があるので、指令の変化点で段に切る。入った直後の
     u0 への移り（入る前の u から u0 へ跳ぶ。区間の外の行との間）は段に数えない。区間の先頭の
     短い u0 の行も捨て、4 段（u0−Δ, u0, u0+Δ, u0）だけで当てる（`dither_steps`）。
  2. 各段の加速度: 段の後半（段の始まり + 遅れの見込み DELAY_ASSUME_S 〜 段の終わり）の車速の直線の
     傾き [km/h/s]。その段の実開度の平均と、段の平均車速も取る。
  3. 表の予測: 同じ (段の平均車速, 段の実開度) の ForwardTable の加速度。u の符号はブレーキが負
     （u = −(開度 − brake min)）、アクセルが正（u = +(開度 − accel min)）。
  4. 1 回の倍率 = (実測の加速度を実開度に直線で当てた傾き) ÷ (表の予測を同じ実開度に当てた傾き)。
     表の傾きが小さすぎる・実開度の幅が小さすぎる回は捨てる。
  5. マス: 行 = 区間の平均車速の代表（ブレーキは dither の測る車速、アクセルは 20 km/h 刻み）、
     列 = 「u0 − 効き始め」の COL_STEP_PCT 刻み。マスごとに中央値と回数。中央値は
     [GAIN_MIN, GAIN_MAX] に切る。測っていないマスは 1（回数 0）。
  6. 遅れ: 段の切り替えで「指令の切り替え時刻 → 加速度の変化が 50% に届く時刻」（pedal_lag の
     HOLD_STEP と同じ考え方）。ペダル別の中央値と回数。

純関数が中心（config・ハードウェアには触らない）。src/ は import しない。
"""

from __future__ import annotations

import csv
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from tests.research import pedal_lag

# CSV の phase 名（pattern_loop の _Phase の名前と同じ。pattern_loop に依らないよう持ち込み）
PHASE_BRAKE_DITHER = "RAMP_BRAKE_DITHER"
PHASE_ACCEL_DITHER = "RAMP_ACCEL_DITHER"
PEDAL_BRAKE = "brake"
PEDAL_ACCEL = "accel"

# 倍率の clip [1, 4]。1 未満は 1（FB を今より強くしないため）。4 は机上（ProblemReport_20261028）の
# 最大の仮値。いつ決めるか: 段5 の実機の手順2 の後、マスごとの生の中央値の分布を見て。
GAIN_MIN = 1.0
GAIN_MAX = 4.0
# 引く側の車速の下限 [km/h]。これ未満は 1（議論の決定。低速は停止側・戻しの層で扱う）
MIN_SPEED_KMH = 7.5
# 段の中の遅れの見込み [s]。加速度は段の始まり + これ 〜 段の終わりで測る。仮値（議論の当てはめで
# 0.20〜0.25 s・meta は 0.125 s。余裕を見て 0.4 s）。段5 で、段の中の加速度の立ち上がりを
# 見て決める。
DELAY_ASSUME_S = 0.4
# 加速度を測る窓の最小の長さ [s]。これ未満の段は使わない。仮値（50 ms 周期で 10 点）。
MIN_WINDOW_S = 0.5
# 指令が 1 周期でこれ以上動いたら段を切る [%]。dither の段は 0.25% 以上の階段（仮値。ノイズなし）。
STEP_JUMP_PCT = 0.02
# 区間の行が途切れたとみなす間隔 [s]（fwd_data.SEGMENT_GAP_S と同じ値。独立に持つ）
GAP_S = 0.5
# 表の傾きの絶対値の下限 [km/h/s per %]。仮値（議論の指定）。割る数が 0 に近い回を捨てる
MIN_TABLE_SLOPE = 0.05
# 実開度の幅（区間の中の段の実開度の最大 − 最小）の下限 [%]。仮値（議論の指定）。
MIN_OPENING_SPAN_PCT = 0.15
# 1 回に必要な段の数
MIN_STEPS_PER_RUN = 3
# 1 回の dither の段の数（u0−Δ, u0, u0+Δ, u0）
DITHER_STEPS = 4
# マスの列の刻み [%]（議論の指定）
COL_STEP_PCT = 0.5
# アクセルの車速の行の刻み [km/h]（議論の指定: データから決める。仮値）
ACCEL_SPEED_STEP_KMH = 20.0
# 遅れ: 切り替えの前後の加速度の差の下限 [km/h/s]。仮値。dither の Δ は 0.25〜0.5% で、HOLD_STEP の
# 3% より小さいので、pedal_lag.MIN_ACCEL_CHANGE_KMHS（1.0）より緩める。いつ決めるか: 段5。
LAG_MIN_ACCEL_CHANGE_KMHS = 0.1
# 遅れ: 指令の変化量の下限 [%]
LAG_MIN_CMD_CHANGE_PCT = 0.2
# 遅れの有効範囲 (0, これ] [s]。pedal_lag.MAX_LAG_S と同じ
LAG_MAX_S = pedal_lag.MAX_LAG_S


@dataclass(frozen=True)
class DitherRun:
    """dither の区間 1 回分の時系列。cmd・actual は指令・実開度（絶対 [%]）。"""

    pedal: str
    t_s: np.ndarray
    speed_kmh: np.ndarray
    cmd_pct: np.ndarray
    actual_pct: np.ndarray  # 読めない行は cmd で埋めてある


@dataclass(frozen=True)
class StepStat:
    """区間の中の 1 段。"""

    i0: int  # 区間の中の行番号（半開区間 [i0, i1)）
    i1: int
    accel_kmhs: float
    opening_pct: float
    speed_kmh: float


@dataclass
class LocalGainFit:
    """1 ペダルの結果。entry は meta に入れる dict（採れなければ None）。"""

    entry: dict[str, Any] | None
    n_runs: int = 0  # dither の区間の数
    n_used: int = 0  # 倍率が出た区間の数
    n_cells: int = 0  # n ≥ 1 のマスの数
    lag_s: float | None = None
    lag_n: int = 0
    reason: str = ""
    cell_medians: list[float] = field(default_factory=list)  # 測れたマスの倍率（clip 後）


def _num(x: str | None) -> float | None:
    if x is None or x == "":
        return None
    try:
        v = float(x)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def read_dither_runs(csv_path: Path) -> list[DitherRun]:
    """CSV から dither の区間を読む（phase が dither で、行間が GAP_S 未満の連続区間 1 つ = 1 回）。

    指令は `*_cmd_pct`（旧形式は `*_opening`）、実開度は `*_actual_pct`（読めない行は指令）。
    車速は `actual_speed_kmh`（無ければ `speed_kmh`）。
    """
    runs: list[DitherRun] = []
    cur: list[tuple[float, float, float, float]] = []
    cur_pedal = ""
    prev_t = math.nan

    def flush() -> None:
        nonlocal cur
        if len(cur) >= 3:
            a = np.array(cur, dtype=float)
            runs.append(DitherRun(cur_pedal, a[:, 0], a[:, 1], a[:, 2], a[:, 3]))
        cur = []

    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            phase = r.get("phase", "") or ""
            pedal = (
                PEDAL_BRAKE if phase == PHASE_BRAKE_DITHER
                else PEDAL_ACCEL if phase == PHASE_ACCEL_DITHER else ""
            )
            t = _num(r.get("elapsed_s"))
            if not pedal or t is None:
                flush()
                prev_t = math.nan
                continue
            axis = pedal
            cmd = _num(r.get(f"{axis}_cmd_pct"))
            if cmd is None:
                cmd = _num(r.get(f"{axis}_opening"))
            v = _num(r.get("actual_speed_kmh"))
            if v is None:
                v = _num(r.get("speed_kmh"))
            if cmd is None or v is None:
                continue
            act = _num(r.get(f"{axis}_actual_pct"))
            if cur and (pedal != cur_pedal or not (0.0 < t - prev_t < GAP_S)):
                flush()
            cur_pedal = pedal
            prev_t = t
            cur.append((t, v, cmd, cmd if act is None else act))
    flush()
    return runs


def split_steps(cmd_pct: np.ndarray, jump_pct: float = STEP_JUMP_PCT) -> list[tuple[int, int]]:
    """指令が 1 周期で jump_pct 以上動いた行で段を切る。半開区間 [i0, i1) の並び。"""
    n = len(cmd_pct)
    if n == 0:
        return []
    cuts = [0] + [k for k in range(1, n) if abs(cmd_pct[k] - cmd_pct[k - 1]) >= jump_pct] + [n]
    return [(a, b) for a, b in zip(cuts, cuts[1:], strict=False) if b > a]


def dither_steps(run: DitherRun, jump_pct: float = STEP_JUMP_PCT) -> list[tuple[int, int]]:
    """1 回の dither の段（u0−Δ, u0, u0+Δ, u0 の 4 段）。入った直後の 1〜2 行（中心 u0 での
    ごく短い区間。入る前の u から u0 へ跳んだ直後で、段ではない）は段に数えない。
    段の数が 4 を超えるときだけ、先頭の MIN_WINDOW_S + DELAY_ASSUME_S に満たない区間を捨てる。"""
    steps = split_steps(run.cmd_pct, jump_pct)
    if len(steps) > DITHER_STEPS:
        i0, i1 = steps[0]
        if run.t_s[i1 - 1] - run.t_s[i0] < MIN_WINDOW_S + DELAY_ASSUME_S:
            steps = steps[1:]
    return steps


def step_stats(
    run: DitherRun, steps: Sequence[tuple[int, int]],
    *, delay_s: float = DELAY_ASSUME_S, min_window_s: float = MIN_WINDOW_S,
) -> list[StepStat]:
    """各段の加速度（段の始まり + delay_s 〜 終わりの車速の傾き）・実開度の平均・車速の平均。
    窓が min_window_s に満たない・傾きが出ない段は返さない。"""
    out: list[StepStat] = []
    t = run.t_s
    for i0, i1 in steps:
        sel = (t >= t[i0] + delay_s) & (np.arange(len(t)) < i1) & (np.arange(len(t)) >= i0)
        if not sel.any() or t[sel][-1] - t[sel][0] < min_window_s - 1e-9:
            continue
        a = pedal_lag._slope(t[sel], run.speed_kmh[sel])  # noqa: SLF001 - 同じ傾きの定義を使い回す
        if not math.isfinite(a):
            continue
        out.append(StepStat(
            i0, i1, a, float(np.mean(run.actual_pct[sel])), float(np.mean(run.speed_kmh[sel]))
        ))
    return out


def u_of(pedal: str, opening_pct: float, min_pct: float) -> float:
    """表の u。ブレーキは −(開度 − min)、アクセルは +(開度 − min)。"""
    d = opening_pct - min_pct
    return -d if pedal == PEDAL_BRAKE else d


def run_ratio(
    pedal: str, stats: Sequence[StepStat], table: Any, min_pct: float,
    *, min_table_slope: float = MIN_TABLE_SLOPE, min_span_pct: float = MIN_OPENING_SPAN_PCT,
) -> tuple[float | None, float | None, float]:
    """1 回の倍率。(倍率, 区間の平均車速, 区間の平均の u) を返す。捨てる回は倍率 None。"""
    if len(stats) < MIN_STEPS_PER_RUN:
        return None, None, math.nan
    op = np.array([s.opening_pct for s in stats])
    acc = np.array([s.accel_kmhs for s in stats])
    vs = np.array([s.speed_kmh for s in stats])
    u = np.array([u_of(pedal, o, min_pct) for o in op])
    v_mean = float(np.mean(vs))
    u_mean = float(np.mean(u))
    if float(op.max() - op.min()) < min_span_pct:
        return None, v_mean, u_mean
    pred = np.array(
        [float(np.asarray(table.accel(float(v), float(uu)))) for v, uu in zip(vs, u, strict=True)]
    )
    k_meas = float(np.polyfit(op, acc, 1)[0])
    k_tab = float(np.polyfit(op, pred, 1)[0])
    if not (math.isfinite(k_meas) and math.isfinite(k_tab)) or abs(k_tab) < min_table_slope:
        return None, v_mean, u_mean
    return k_meas / k_tab, v_mean, u_mean


def nearest_rep(v_kmh: float, reps: Sequence[float]) -> float:
    return float(min(reps, key=lambda r: abs(r - v_kmh)))


def accel_speed_rep(v_kmh: float, step_kmh: float = ACCEL_SPEED_STEP_KMH) -> float:
    """アクセルの区間の平均車速を step_kmh 刻みの代表に丸める（最小は 1 刻み）。"""
    return max(step_kmh, round(v_kmh / step_kmh) * step_kmh)


def boundary_lag_s(
    run: DitherRun, prev: StepStat, cur: StepStat,
    *, min_accel_change: float = LAG_MIN_ACCEL_CHANGE_KMHS,
    min_cmd_change_pct: float = LAG_MIN_CMD_CHANGE_PCT,
) -> float | None:
    """段の切り替え 1 つの遅れ。指令の切り替え時刻 → 加速度の変化が 50% に届く時刻 [s]。
    pedal_lag._step_lag と同じ考え方（指令は階段なので指令の 50% 点は切り替え時刻そのもの）。
    条件を満たさなければ None。"""
    t, v = run.t_s, run.speed_kmh
    if prev.i1 != cur.i0:  # 間に捨てた段がある
        return None
    if abs(run.cmd_pct[cur.i0] - run.cmd_pct[cur.i0 - 1]) < min_cmd_change_pct:
        return None
    a_pre, a_post = prev.accel_kmhs, cur.accel_kmhs
    if abs(a_post - a_pre) < min_accel_change:
        return None
    half = pedal_lag.LOCAL_HALF_WINDOW_S
    local = []
    for k in range(cur.i0, cur.i1):
        m = (t >= t[k] - half) & (t <= t[k] + half)
        local.append(pedal_lag._slope(t[m], v[m]))  # noqa: SLF001
    loc = np.array(local, dtype=float)
    loc = np.where(np.isfinite(loc), loc, a_pre)  # 測れない点は変化前扱い（越えない）
    sign = 1.0 if a_post > a_pre else -1.0
    k_acc = pedal_lag._first_crossing(loc, (a_pre + a_post) / 2.0, sign)  # noqa: SLF001
    if k_acc is None:
        return None
    lag = float(t[cur.i0 + k_acc] - t[cur.i0])
    return lag if 0.0 <= lag <= LAG_MAX_S else None


def fit_pedal(
    runs: Sequence[DitherRun], pedal: str, table: Any, *,
    pedal_min_pct: float, onset_u_pct: float | None,
    speed_reps_kmh: Sequence[float] | None = None,
) -> LocalGainFit:
    """1 ペダルの倍率の表と遅れ。speed_reps_kmh（ブレーキ: dither の測る車速）が None なら
    アクセルの丸め（ACCEL_SPEED_STEP_KMH 刻み）。onset_u_pct が None なら表は作らない
    （遅れだけ測る）。"""
    mine = [r for r in runs if r.pedal == pedal]
    fit = LocalGainFit(entry=None, n_runs=len(mine))
    lags: list[float] = []
    cells: list[tuple[float, int, float]] = []  # (行の代表車速, 列の番号, 倍率)
    for run in mine:
        stats = step_stats(run, dither_steps(run))
        for prev, cur in zip(stats, stats[1:], strict=False):
            lag = boundary_lag_s(run, prev, cur)
            if lag is not None:
                lags.append(lag)
        ratio, v_mean, u_mean = run_ratio(pedal, stats, table, pedal_min_pct)
        if ratio is None or v_mean is None or v_mean < MIN_SPEED_KMH or onset_u_pct is None:
            continue
        x = abs(u_mean) - onset_u_pct
        rep = (
            accel_speed_rep(v_mean) if not speed_reps_kmh else nearest_rep(v_mean, speed_reps_kmh)
        )
        cells.append((rep, int(round(x / COL_STEP_PCT)), ratio))
        fit.n_used += 1
    if lags:
        fit.lag_s = float(np.median(lags))
        fit.lag_n = len(lags)
    if onset_u_pct is None:
        fit.reason = "効き始めが meta に無い"
        return fit
    if not cells:
        fit.reason = f"倍率が出た区間なし（区間 {fit.n_runs}）"
        return fit
    reps = sorted({c[0] for c in cells})
    k_lo, k_hi = min(c[1] for c in cells), max(c[1] for c in cells)
    cols = [k * COL_STEP_PCT for k in range(k_lo, k_hi + 1)]
    ratio_m = [[1.0] * len(cols) for _ in reps]
    n_m = [[0] * len(cols) for _ in reps]
    for ri, rep in enumerate(reps):
        for ci, k in enumerate(range(k_lo, k_hi + 1)):
            vals = [c[2] for c in cells if c[0] == rep and c[1] == k]
            if vals:
                g = float(min(GAIN_MAX, max(GAIN_MIN, float(np.median(vals)))))
                ratio_m[ri][ci] = g
                n_m[ri][ci] = len(vals)
                fit.cell_medians.append(g)
    fit.n_cells = len(fit.cell_medians)
    fit.entry = {
        "speed_rep_kmh": [float(r) for r in reps],
        "u_from_onset_pct": [float(c) for c in cols],
        "ratio": ratio_m,
        "n": n_m,
    }
    return fit


def local_gain_at(
    entry: Mapping[str, Any] | None, v_kmh: float, u_from_onset_pct: float,
    *, min_speed_kmh: float = MIN_SPEED_KMH,
) -> float:
    """倍率を引く。各行は測ったマス（n>0）だけで開度を直線補間し、その行の測った開度の範囲の外なら
    その行は使わない。使える行どうしを車速で直線補間し、車速の範囲外は端の行の値のまま
    （2026-10-28 段4 確認: 机上で倍率は 15〜140 km/h のどの帯でも 2.1〜2.7 倍とほぼ同じ。
    測る車速の外（7.5〜30・100 超）を 1 にすると、効かせたい帯の多くが抜けるため）。
    2026-10-28 段5b: 測っていないマスを 1.0 として補間すると、u0 を 0.5・1.5・2.5% に巡回させて測る
    （1.0・2.0% の列は測らない）ため、倍率が測った列の上だけの細い山になる（`111335`）。そのため
    測っていないマスは補間に入れない。
    entry なし・min_speed_kmh 未満・どの行も開度の範囲外は 1.0。"""
    if not entry or v_kmh < min_speed_kmh:
        return 1.0
    reps = np.asarray(entry["speed_rep_kmh"], dtype=float)
    cols = np.asarray(entry["u_from_onset_pct"], dtype=float)
    if len(reps) == 0 or len(cols) == 0:
        return 1.0
    ratio = np.asarray(entry["ratio"], dtype=float)
    n = np.asarray(entry["n"], dtype=float)
    use_v: list[float] = []
    use_g: list[float] = []
    for i, rep in enumerate(reps):
        m = n[i] > 0
        if not m.any():
            continue
        c = cols[m]
        if c[0] <= u_from_onset_pct <= c[-1]:
            use_v.append(float(rep))
            use_g.append(float(np.interp(u_from_onset_pct, c, ratio[i][m])))
    if not use_v:
        return 1.0
    out = float(np.interp(v_kmh, use_v, use_g))
    return float(min(GAIN_MAX, max(GAIN_MIN, out)))


def attach_to_meta(
    meta: dict[str, Any], brake: LocalGainFit, accel: LocalGainFit, source: str
) -> bool:
    """meta に local_gain_brake・local_gain_accel・dither_*_lag_s（と _n）・local_gain_source を
    入れる。何も入らなければ False（meta は変えない）。"""
    any_put = False
    for pedal, fit in ((PEDAL_BRAKE, brake), (PEDAL_ACCEL, accel)):
        if fit.entry is not None:
            meta[f"local_gain_{pedal}"] = fit.entry
            any_put = True
        if fit.lag_s is not None:
            meta[f"dither_{pedal}_lag_s"] = float(fit.lag_s)
            meta[f"dither_{pedal}_lag_n"] = int(fit.lag_n)
            any_put = True
    if any_put:
        meta["local_gain_source"] = source
    return any_put
