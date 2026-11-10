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

2026-11-10 ProblemReport_20261109 段7: アクセルの定速の保持（G ランプの RAMP_CRUISE・
RAMP_HOLD）からも測る（`fit_cruise`）。dither は手順2 では off で、
定速の開度（例 128 km/h で 34 %）の局所の効きは測れていなかった。
保持の PI が開度を少しずつ動かすので、その区間の「実開度 → 加速度」の傾き
（実開度は遅れ L だけずらす）を表の傾きで割って倍率にする。結果は別のキー
`local_gain_accel_cruise` に残す（dither の `local_gain_accel` は書き換えない。形は同じなので
`local_gain_at` がそのまま引ける）。閉ループの中からの推定なので、手順2 の遅れの測定の
HOLD_STEP（開度固定 ＝ 開ループ）のアクセルのステップで同じ倍率を検算できる（`hold_step_ratios`）。

2026-11-10 ProblemReport_20261109 段10c: 手順2 の定速の保持の正弦波（RAMP_CRUISE_SINE。保持の PI の
出力に A·sin(2πft) を足す）から、1 Hz 付近の効きを測る（`fit_cruise_sine`）。保持ごとに最初の 1
周期を
捨て、車速と実開度（accel_actual_pct）に 3 次のトレンドと f の正弦波を同時に当てはめて（excite.py の
解析と同じ式）|P|・位相・SNR を出す。倍率 r ＝ |P|·2πf ÷ 表の感度 S（その保持の平均の車速・開度で
中心差分）。結果は meta `local_gain_accel_hf`（車速 20 km/h 刻みの中央値）に残す（段7 の
`local_gain_accel_cruise` とは別。ゆっくり・大きな変化の倍率と 1 Hz 付近の倍率は別の数）。

純関数が中心（config・ハードウェアには触らない）。src/ は import しない。
"""

from __future__ import annotations

import cmath
import csv
import math
import re
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


# ── 段7: 定速の保持からの測定（2026-11-10 ProblemReport_20261109）──────────
PHASE_RAMP_CRUISE = "RAMP_CRUISE"
PHASE_RAMP_HOLD = "RAMP_HOLD"
PHASE_HOLD_STEP = "HOLD_STEP"
# 加速度の窓 [s]（その時刻を中心とする直線の傾き）。仮値・半分データ由来（W 1690〜1720 s の
# 実機で遅れ 0.14 s・残差 0.020 km/h/s）。効く先: 倍率のばらつき。
# いつ決めるか: 段10 の実機の手順2 の後。
CRUISE_ACCEL_WINDOW_S = 0.3
# 窓に必要な点数の下限（50 ms 周期で窓 0.3 s なら 7 点前後）
CRUISE_WINDOW_MIN_POINTS = 4
# 当てはまりの下限 R²。仮値（10-28 のブレーキの局所の傾きの机上と同じ）。効く先: 捨てる区間の数。
# いつ決めるか: 段10 の実機の手順2 の後、R² の分布を見て。
CRUISE_MIN_R2 = 0.6
# 区間の実開度（遅れをずらした後）の幅の下限 [%]。仮値。
# 保持の PI の動きが小さすぎる区間は傾きがノイズで決まる。
CRUISE_MIN_SPAN_PCT = 0.1
# 区間の行数の下限（遅れをずらした後。50 ms 周期で 1 秒）。仮値
CRUISE_MIN_ROWS = 20
# 遅れが meta に無いときの代わり [s]（半分データ由来: W 1690〜1720 s の 0.14 s）
CRUISE_LAG_FALLBACK_S = 0.14
# 表の傾きを引く開度の幅 [%]（u ± これの差分。表の局所の傾き。仮値）
CRUISE_TABLE_H_PCT = 0.25
# 列の刻み [%]（粗い表。仮値。保持の開度は車速ごとに 1 点なので細かく切ると点が散る）
CRUISE_COL_STEP_PCT = 5.0
# ブレーキが min とみなす幅 [%]（actual のゆれ。pedal_lag.AT_MIN_TOL_PCT と同じ値）
CRUISE_BRAKE_AT_MIN_TOL_PCT = 0.2
# 検算（HOLD_STEP）: 前後の窓 [s]・実開度の変化の下限 [%]（pedal_lag と同じ考え方。仮値）
HOLD_STEP_PRE_S = 1.0
HOLD_STEP_POST_START_S = 1.2
HOLD_STEP_MIN_OPENING_CHANGE_PCT = 1.0


@dataclass(frozen=True)
class CruiseSegment:
    """定速の保持の区間 1 回分（アクセルだけの連続行）。開度は actual（読めない行は cmd）。"""

    t_s: np.ndarray
    speed_kmh: np.ndarray
    accel_pct: np.ndarray


@dataclass
class CruiseFit:
    """定速の保持からの測定結果。entry は meta に入れる dict（採れなければ None）。"""

    entry: dict[str, Any] | None
    n_segments: int = 0  # 読めた区間の数
    n_used: int = 0  # 倍率が出た区間の数
    n_cells: int = 0
    reason: str = ""
    lag_s: float = CRUISE_LAG_FALLBACK_S
    ratios: list[float] = field(default_factory=list)  # 区間ごとの倍率（clip 前）
    r2: list[float] = field(default_factory=list)  # 採った区間の R²
    by_speed: dict[float, list[float]] = field(default_factory=dict)  # 行の車速 → 倍率（clip 前）
    reject: dict[str, int] = field(default_factory=dict)  # 捨てた理由 → 数


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
    *, min_speed_kmh: float = MIN_SPEED_KMH, edge_hold_pct: float = 0.0,
) -> float:
    """倍率を引く。各行は測ったマス（n>0）だけで開度を直線補間し、その行の測った開度の範囲の外なら
    その行は使わない。使える行どうしを車速で直線補間し、車速の範囲外は端の行の値のまま
    （2026-10-28 段4 確認: 机上で倍率は 15〜140 km/h のどの帯でも 2.1〜2.7 倍とほぼ同じ。
    測る車速の外（7.5〜30・100 超）を 1 にすると、効かせたい帯の多くが抜けるため）。
    2026-10-28 段5b: 測っていないマスを 1.0 として補間すると、u0 を 0.5・1.5・2.5% に巡回させて測る
    （1.0・2.0% の列は測らない）ため、倍率が測った列の上だけの細い山になる（`111335`）。そのため
    測っていないマスは補間に入れない。
    entry なし・min_speed_kmh 未満・どの行も開度の範囲外は 1.0。
    2026-11-10 段9: edge_hold_pct > 0 なら、
    測った開度の範囲の両端から edge_hold_pct までは端の値のまま
    使う（定速の保持の表は列が 5% 刻みで 1 行に 1〜2 列しか無く、範囲の中だけでは引けないため。
    既定 0 は今までと同じ）。"""
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
        if c[0] - edge_hold_pct <= u_from_onset_pct <= c[-1] + edge_hold_pct:
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


# ── 段7: 定速の保持（RAMP_CRUISE・RAMP_HOLD）からの局所の効き ──────────────


def _read_rows(csv_path: Path) -> list[dict[str, Any]]:
    """CSV の行（時刻・実車速・アクセル/ブレーキの実開度（読めなければ指令）・phase）。"""
    rows: list[dict[str, Any]] = []
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            t = _num(r.get("elapsed_s"))
            v = _num(r.get("actual_speed_kmh"))
            if v is None:
                v = _num(r.get("speed_kmh"))
            if t is None or v is None:
                continue
            acc = _num(r.get("accel_actual_pct"))
            if acc is None:
                acc = _num(r.get("accel_cmd_pct"))
            if acc is None:
                acc = _num(r.get("accel_opening"))
            brk = _num(r.get("brake_actual_pct"))
            if brk is None:
                brk = _num(r.get("brake_cmd_pct"))
            if brk is None:
                brk = _num(r.get("brake_opening"))
            if acc is None or brk is None:
                continue
            rows.append({"t": t, "v": v, "acc": acc, "brk": brk, "phase": r.get("phase", "") or ""})
    return rows


def read_cruise_segments(
    csv_path: Path, *, accel_min_pct: float, brake_min_pct: float,
    min_speed_kmh: float = MIN_SPEED_KMH,
) -> list[CruiseSegment]:
    """CSV から定速の保持の区間を読む（phase が RAMP_CRUISE か RAMP_HOLD で、行間が GAP_S 未満・
    phase が同じ連続区間 1 つ = 1 回）。アクセルだけの行（アクセルの実開度 > min・ブレーキの実開度が
    min 付近）で車速が min_speed_kmh 以上の行だけ。条件を外れた行で区間を切る。"""
    segs: list[CruiseSegment] = []
    cur: list[tuple[float, float, float]] = []
    prev_t = math.nan
    prev_phase = ""

    def flush() -> None:
        nonlocal cur
        if len(cur) >= 3:
            a = np.array(cur, dtype=float)
            segs.append(CruiseSegment(a[:, 0], a[:, 1], a[:, 2]))
        cur = []

    for r in _read_rows(csv_path):
        ok = (
            r["phase"] in (PHASE_RAMP_CRUISE, PHASE_RAMP_HOLD)
            and r["acc"] > accel_min_pct
            and r["brk"] <= brake_min_pct + CRUISE_BRAKE_AT_MIN_TOL_PCT
            and r["v"] >= min_speed_kmh
        )
        if not ok:
            flush()
            prev_t = math.nan
            continue
        if cur and (r["phase"] != prev_phase or not (0.0 < r["t"] - prev_t < GAP_S)):
            flush()
        prev_t, prev_phase = r["t"], r["phase"]
        cur.append((r["t"], r["v"], r["acc"]))
    flush()
    return segs


def windowed_accel(
    t_s: np.ndarray, v_kmh: np.ndarray, window_s: float = CRUISE_ACCEL_WINDOW_S
) -> np.ndarray:
    """各行を中心とする窓 window_s（±半分）の車速の直線の傾き [km/h/s]。点が足りない行は nan。"""
    out = np.full(len(t_s), math.nan)
    half = window_s / 2.0
    for k in range(len(t_s)):
        m = (t_s >= t_s[k] - half - 1e-9) & (t_s <= t_s[k] + half + 1e-9)
        if int(m.sum()) >= CRUISE_WINDOW_MIN_POINTS:
            out[k] = pedal_lag._slope(t_s[m], v_kmh[m])  # noqa: SLF001
    return out


def table_slope_at(
    table: Any, v_kmh: float, u_pct: float, h_pct: float = CRUISE_TABLE_H_PCT
) -> float:
    """表の加速度の開度方向の傾き [km/h/s per %]（(v, u) のまわり ±h の差分。u の下は 0）。"""
    lo = max(0.0, u_pct - h_pct)
    hi = u_pct + h_pct
    a_lo = float(np.asarray(table.accel(float(v_kmh), float(lo))))
    a_hi = float(np.asarray(table.accel(float(v_kmh), float(hi))))
    return (a_hi - a_lo) / (hi - lo)


def cruise_segment_ratio(
    seg: CruiseSegment, table: Any, accel_min_pct: float, lag_s: float,
    *, min_r2: float = CRUISE_MIN_R2, min_span_pct: float = CRUISE_MIN_SPAN_PCT,
    min_rows: int = CRUISE_MIN_ROWS, min_table_slope: float = MIN_TABLE_SLOPE,
) -> tuple[float | None, float, float, float, str]:
    """区間 1 回の倍率。(倍率, 平均車速, 平均の u, R², 捨てた理由) を返す。
    倍率 None のときは理由が入る（"rows"・"span"・"r2"・"table_slope"）。
    加速度（窓の中心）を、遅れ L だけ前の実開度（直線補間）に直線で当てた傾き ÷ 表の傾き。"""
    t, v = seg.t_s, seg.speed_kmh
    acc = windowed_accel(t, v)
    t_shift = t - lag_s
    ok = np.isfinite(acc) & (t_shift >= t[0])
    v_mean = float(np.mean(v))
    if int(ok.sum()) < min_rows:
        return None, v_mean, math.nan, math.nan, "rows"
    op = np.interp(t_shift[ok], t, seg.accel_pct)
    a = acc[ok]
    u_mean = float(np.mean(op)) - accel_min_pct
    if float(op.max() - op.min()) < min_span_pct:
        return None, v_mean, u_mean, math.nan, "span"
    slope, intercept = np.polyfit(op, a, 1)
    resid = a - (slope * op + intercept)
    ss_tot = float(np.sum((a - np.mean(a)) ** 2))
    r2 = 1.0 - float(np.sum(resid**2)) / ss_tot if ss_tot > 0 else 0.0
    if r2 < min_r2:
        return None, v_mean, u_mean, r2, "r2"
    k_tab = table_slope_at(table, v_mean, u_mean)
    if not math.isfinite(k_tab) or abs(k_tab) < min_table_slope:
        return None, v_mean, u_mean, r2, "table_slope"
    return float(slope) / k_tab, v_mean, u_mean, r2, ""


def fit_cruise(
    segments: Sequence[CruiseSegment], table: Any, *, accel_min_pct: float,
    onset_u_pct: float | None, lag_s: float | None,
) -> CruiseFit:
    """アクセルの定速の保持の区間から倍率の表を作る。行 = accel_speed_rep（20 km/h 刻み）、
    列 = (u − 効き始め) の CRUISE_COL_STEP_PCT 刻み。マスは中央値（[GAIN_MIN, GAIN_MAX] に clip）。
    形は dither の entry と同じ（`local_gain_at` がそのまま引ける）。"""
    fit = CruiseFit(entry=None, n_segments=len(segments))
    if lag_s is not None and math.isfinite(lag_s) and lag_s > 0:
        fit.lag_s = float(lag_s)
    if onset_u_pct is None:
        fit.reason = "効き始めが meta に無い"
        return fit
    cells: list[tuple[float, int, float]] = []
    for seg in segments:
        ratio, v_mean, u_mean, r2, why = cruise_segment_ratio(
            seg, table, accel_min_pct, fit.lag_s
        )
        if ratio is None:
            fit.reject[why] = fit.reject.get(why, 0) + 1
            continue
        rep = accel_speed_rep(v_mean)
        cells.append((rep, int(round((u_mean - onset_u_pct) / CRUISE_COL_STEP_PCT)), ratio))
        fit.ratios.append(ratio)
        fit.r2.append(r2)
        fit.by_speed.setdefault(rep, []).append(ratio)
        fit.n_used += 1
    if not cells:
        fit.reason = f"倍率が出た区間なし（区間 {fit.n_segments}・捨てた理由 {fit.reject}）"
        return fit
    reps = sorted({c[0] for c in cells})
    k_lo, k_hi = min(c[1] for c in cells), max(c[1] for c in cells)
    cols = [k * CRUISE_COL_STEP_PCT for k in range(k_lo, k_hi + 1)]
    ratio_m = [[1.0] * len(cols) for _ in reps]
    n_m = [[0] * len(cols) for _ in reps]
    for ri, rep in enumerate(reps):
        for ci, k in enumerate(range(k_lo, k_hi + 1)):
            vals = [c[2] for c in cells if c[0] == rep and c[1] == k]
            if vals:
                ratio_m[ri][ci] = float(min(GAIN_MAX, max(GAIN_MIN, float(np.median(vals)))))
                n_m[ri][ci] = len(vals)
                fit.n_cells += 1
    fit.entry = {
        "speed_rep_kmh": [float(r) for r in reps],
        "u_from_onset_pct": [float(c) for c in cols],
        "ratio": ratio_m,
        "n": n_m,
    }
    return fit


def attach_cruise_to_meta(meta: dict[str, Any], fit: CruiseFit, source: str) -> bool:
    """meta に local_gain_accel_cruise（と _source・_n_segments・_n_used）を入れる。
    dither の local_gain_accel は書き換えない。倍率の表が無ければ False（meta は変えない）。"""
    if fit.entry is None:
        return False
    meta["local_gain_accel_cruise"] = fit.entry
    meta["local_gain_accel_cruise_source"] = source
    meta["local_gain_accel_cruise_n_segments"] = int(fit.n_segments)
    meta["local_gain_accel_cruise_n_used"] = int(fit.n_used)
    return True


def hold_step_ratios(
    csv_path: Path, table: Any, *, accel_min_pct: float, brake_min_pct: float,
) -> list[tuple[float, float]]:
    """検算: 手順2 の遅れの測定（HOLD_STEP。開度固定 ＝ 開ループ）のアクセルのステップの倍率。
    ステップの前の窓（直前 HOLD_STEP_PRE_S 秒）と後の窓（始まり + HOLD_STEP_POST_START_S 〜
    終わり）の
    加速度・実開度の平均の差から (Δ加速度 ÷ Δ開度) ÷ 表の同じ 2 点の傾き。[(車速の平均, 倍率), …]。
    実開度の変化が HOLD_STEP_MIN_OPENING_CHANGE_PCT 未満・ブレーキが min 以外・
    車速が MIN_SPEED_KMH 未満のステップは捨てる。ステップは phase が HOLD_STEP の連続区間を、
    アクセルの実開度の 1 周期の飛びが 1.0% 以上の所で切ったもの。"""
    rows = _read_rows(csv_path)
    if not rows:
        return []
    t = np.array([r["t"] for r in rows])
    v = np.array([r["v"] for r in rows])
    acc = np.array([r["acc"] for r in rows])
    brk = np.array([r["brk"] for r in rows])
    hold = np.array([r["phase"] == PHASE_HOLD_STEP for r in rows])
    out: list[tuple[float, float]] = []
    n = len(rows)
    i = 0
    while i < n:
        if not hold[i]:
            i += 1
            continue
        j = i
        while j < n and hold[j] and (j == i or 0.0 < t[j] - t[j - 1] < GAP_S):
            j += 1
        start = i
        for k in range(i + 1, j + 1):
            if k == j or abs(acc[k] - acc[k - 1]) >= 1.0:
                if k - start >= 2 and start >= 1:
                    _hold_step_one(
                        t, v, acc, brk, start, k, table, accel_min_pct, brake_min_pct, out
                    )
                start = k
        i = max(j, i + 1)
    return out


def _hold_step_one(
    t: np.ndarray, v: np.ndarray, acc: np.ndarray, brk: np.ndarray, i0: int, i1: int,
    table: Any, accel_min_pct: float, brake_min_pct: float, out: list[tuple[float, float]],
) -> None:
    pre = (t >= t[i0] - HOLD_STEP_PRE_S) & (t < t[i0])
    post = (t >= t[i0] + HOLD_STEP_POST_START_S) & (t <= t[i1 - 1])
    if pre.sum() < 3 or post.sum() < 3:
        return
    if (brk[pre | post] > brake_min_pct + CRUISE_BRAKE_AT_MIN_TOL_PCT).any():
        return
    a0 = pedal_lag._slope(t[pre], v[pre])  # noqa: SLF001
    a1 = pedal_lag._slope(t[post], v[post])  # noqa: SLF001
    o0, o1 = float(np.mean(acc[pre])), float(np.mean(acc[post]))
    v_mean = float(np.mean(v[pre | post]))
    if not (math.isfinite(a0) and math.isfinite(a1)) or v_mean < MIN_SPEED_KMH:
        return
    if abs(o1 - o0) < HOLD_STEP_MIN_OPENING_CHANGE_PCT or min(o0, o1) <= accel_min_pct:
        return
    u0, u1 = o0 - accel_min_pct, o1 - accel_min_pct
    k_tab = (
        float(np.asarray(table.accel(v_mean, u1))) - float(np.asarray(table.accel(v_mean, u0)))
    ) / (u1 - u0)
    if not math.isfinite(k_tab) or abs(k_tab) < MIN_TABLE_SLOPE:
        return
    out.append((v_mean, ((a1 - a0) / (o1 - o0)) / k_tab))


# ── 段10c: 定速の保持の正弦波（RAMP_CRUISE_SINE）からの 1 Hz 付近の効き ──────
# 2026-11-10 ProblemReport_20261109 段10c
PHASE_RAMP_CRUISE_SINE = "RAMP_CRUISE_SINE"
# 3 次のトレンド（excite.py の _TREND_DEGREE と同じ。理由は ProblemReport_20260921: 1 次だと
# 到達フェーズの整定の尾の曲がりが正弦波成分に漏れ、ゲイン・位相にバイアスが乗る）
SINE_TREND_DEGREE = 3
# SNR の下限。仮の値（段10b の加振で SNR 4〜8 が取れた。3
# 未満はその周波数成分が他の変動に埋もれる）。
# 効く先: 捨てる保持の数。決める時期: 段10e（SNR の分布を見て）
SINE_MIN_SNR = 3.0
# 表の感度を引く開度の幅 [%]（loop_margin.operating_point の du と同じ 0.5。中心差分）
SINE_TABLE_H_PCT = 0.5
# 当てはめに必要な行数の下限（計画行列の列数 トレンド 4 ＋ cos/sin 2 に余裕を足した値）
SINE_MIN_ROWS = 10
# 加振 CSV（excite.py）のブロック名 `<車速>kmh_<周波数>Hz`（_brake なしだけ読む）
EXCITE_BLOCK_RE = re.compile(r"^(?P<v>\d+(?:\.\d+)?)kmh_(?P<f>\d+(?:\.\d+)?)Hz$")
# 正弦波の周波数の既定 [Hz]（learning.ramp_cruise_sine_hz の既定と同じ。CSV には周波数が残らないので
# 付ける側が渡す。渡されなければこれ）
SINE_HZ_DEFAULT = 1.2
EXCITE_SKIP_S = 2.0  # excite.analyze の既定（先頭の過渡）


@dataclass(frozen=True)
class SineSegment:
    """正弦波の区間 1 回分（RAMP_CRUISE_SINE の連続行）。開度は絶対 [%]（accel_actual_pct と
    accel_cmd_pct。読めない行は互いで埋める）。"""

    t_s: np.ndarray
    speed_kmh: np.ndarray
    accel_actual_pct: np.ndarray
    accel_cmd_pct: np.ndarray


@dataclass(frozen=True)
class SineResponse:
    """正弦波の当てはめの結果（1 区間）。"""

    n_rows: int
    abs_p_kmh_per_pct: float  # 車速の振幅 ÷ 開度の振幅
    phase_deg: float  # 車速が開度に対して遅れる向きを負で表す
    snr: float  # 車速の振幅 ÷ 残差（トレンドと f の成分を除いた後）の RMS
    speed_amp_kmh: float
    opening_amp_pct: float
    v_mean_kmh: float
    u_mean_pct: float  # 開度の平均（絶対 [%]）


@dataclass
class SineFit:
    """保持の正弦波からの測定結果。entry は meta に入れる dict（採れなければ None）。"""

    entry: dict[str, Any] | None
    hz: float
    n_segments: int = 0
    n_used: int = 0
    reason: str = ""
    reject: dict[str, int] = field(default_factory=dict)
    per_segment: list[dict[str, float]] = field(default_factory=list)  # 採った区間ごとの値
    by_speed: dict[float, list[float]] = field(default_factory=dict)  # 行の車速 → r


def _sine_design(t: np.ndarray, hz: float) -> np.ndarray:
    span = float(np.ptp(t))
    tt = np.zeros_like(t) if span <= 0.0 else (t - t.mean()) / (span / 2.0)
    cols = [tt**k for k in range(SINE_TREND_DEGREE + 1)]
    return np.column_stack(
        [*cols, np.cos(2.0 * np.pi * hz * t), np.sin(2.0 * np.pi * hz * t)]
    )


def _fit_sine(t: np.ndarray, x: np.ndarray, hz: float) -> tuple[complex, np.ndarray]:
    """3 次のトレンドと hz の正弦波を 1 回の最小二乗で同時に当てはめる
    （excite._fit_sinusoid と同じ）。複素成分は離散フーリエ和の規約
    （2·|big|/n が振幅・angle(big) が位相）。残差も返す。"""
    design = _sine_design(t, hz)
    coeffs, *_ = np.linalg.lstsq(design, x, rcond=None)
    big = 0.5 * len(t) * complex(coeffs[-2], -coeffs[-1])
    return big, x - design @ coeffs


def sine_response(
    t_s: np.ndarray, v_kmh: np.ndarray, u_pct: np.ndarray, hz: float, *, skip_s: float,
) -> SineResponse | None:
    """区間の先頭 skip_s（過渡）を捨て、車速 y と開度 u に正弦波を当てて |P|・位相・SNR を出す
    （excite.analyze と同じ式）。行が足りない・開度の振幅が 0 なら None。"""
    m = (t_s - t_s[0]) >= skip_s
    if int(m.sum()) < SINE_MIN_ROWS:
        return None
    t, y, u = t_s[m], v_kmh[m], u_pct[m]
    big_u, _ = _fit_sine(t, u, hz)
    big_y, res_y = _fit_sine(t, y, hz)
    if abs(big_u) <= 0.0:
        return None
    n = len(t)
    speed_amp = 2.0 * abs(big_y) / n
    rms = float(np.sqrt(np.mean(res_y**2)))
    return SineResponse(
        n_rows=n, abs_p_kmh_per_pct=abs(big_y) / abs(big_u),
        phase_deg=math.degrees(cmath.phase(big_y / big_u)),
        snr=speed_amp / rms if rms > 0.0 else math.inf,
        speed_amp_kmh=speed_amp, opening_amp_pct=2.0 * abs(big_u) / n,
        v_mean_kmh=float(np.mean(y)), u_mean_pct=float(np.mean(u)),
    )


def read_sine_segments(csv_path: Path) -> list[SineSegment]:
    """CSV から正弦波の区間を読む（phase が RAMP_CRUISE_SINE で、行間が GAP_S 未満の連続区間
    1 つ = 1 回）。開度は accel_actual_pct（読めなければ accel_cmd_pct）。"""
    segs: list[SineSegment] = []
    cur: list[tuple[float, float, float, float]] = []
    prev_t = math.nan

    def flush() -> None:
        nonlocal cur
        if len(cur) >= 3:
            a = np.array(cur, dtype=float)
            segs.append(SineSegment(a[:, 0], a[:, 1], a[:, 2], a[:, 3]))
        cur = []

    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if (r.get("phase") or "") != PHASE_RAMP_CRUISE_SINE:
                flush()
                prev_t = math.nan
                continue
            t = _num(r.get("elapsed_s"))
            v = _num(r.get("actual_speed_kmh"))
            act = _num(r.get("accel_actual_pct"))
            cmd = _num(r.get("accel_cmd_pct"))
            if act is None:
                act = cmd
            if cmd is None:
                cmd = act
            if t is None or v is None or act is None or cmd is None:
                continue
            if cur and not (0.0 < t - prev_t < GAP_S):
                flush()
            prev_t = t
            cur.append((t, v, act, cmd))
    flush()
    return segs


def sine_ratio(
    resp: SineResponse, table: Any, accel_min_pct: float, hz: float,
) -> tuple[float | None, float]:
    """倍率 r ＝ |P|·2πf ÷ 表の感度 S と S を返す。
    S は (区間の平均の車速, 平均の u ＝ 平均開度 − min) のまわり ±SINE_TABLE_H_PCT の
    中心差分（u の下は 0）。S が有限の正でなければ r は None。"""
    u_mean = resp.u_mean_pct - accel_min_pct
    sens = table_slope_at(table, resp.v_mean_kmh, u_mean, SINE_TABLE_H_PCT)
    if not math.isfinite(sens) or sens <= MIN_TABLE_SLOPE:
        return None, sens
    return resp.abs_p_kmh_per_pct * 2.0 * math.pi * hz / sens, sens


def fit_cruise_sine(
    segments: Sequence[SineSegment], table: Any, *, accel_min_pct: float, hz: float,
    min_snr: float = SINE_MIN_SNR, use: str = "actual",
) -> SineFit:
    """保持の正弦波の区間から、車速ごとの倍率 r の表を作る。区間ごとに最初の 1 周期（1 ÷ hz 秒）を
    捨てて当てはめ、SNR が min_snr 未満・表の感度が無効な区間は捨てる（`reject`: rows・snr・sens）。
    use は開度の列（"actual" ＝ accel_actual_pct・"cmd" ＝ accel_cmd_pct）。
    表（entry）: speed_rep_kmh（accel_speed_rep の 20 km/h 刻み）ごとの r の中央値（ratio）・
    数（n）・
    |P| の中央値（abs_p）・位相の中央値（phase_deg）。clip はしない（C10 が使う側で扱う）。"""
    fit = SineFit(entry=None, hz=float(hz), n_segments=len(segments))
    rows: list[tuple[float, float, float, float]] = []  # (rep, r, |P|, phase)
    for seg in segments:
        u = seg.accel_actual_pct if use == "actual" else seg.accel_cmd_pct
        resp = sine_response(seg.t_s, seg.speed_kmh, u, hz, skip_s=1.0 / hz)
        if resp is None:
            fit.reject["rows"] = fit.reject.get("rows", 0) + 1
            continue
        if not resp.snr >= min_snr:
            fit.reject["snr"] = fit.reject.get("snr", 0) + 1
            continue
        ratio, sens = sine_ratio(resp, table, accel_min_pct, hz)
        if ratio is None:
            fit.reject["sens"] = fit.reject.get("sens", 0) + 1
            continue
        rep = accel_speed_rep(resp.v_mean_kmh)
        rows.append((rep, ratio, resp.abs_p_kmh_per_pct, resp.phase_deg))
        fit.n_used += 1
        fit.by_speed.setdefault(rep, []).append(ratio)
        fit.per_segment.append({
            "v_mean_kmh": resp.v_mean_kmh, "u_mean_pct": resp.u_mean_pct - accel_min_pct,
            "abs_p": resp.abs_p_kmh_per_pct, "phase_deg": resp.phase_deg, "snr": resp.snr,
            "sens": sens, "ratio": ratio,
        })
    if not rows:
        fit.reason = f"倍率が出た区間なし（区間 {fit.n_segments}・捨てた理由 {fit.reject}）"
        return fit
    reps = sorted({r[0] for r in rows})
    fit.entry = {
        "speed_rep_kmh": [float(r) for r in reps],
        "ratio": [float(np.median([x[1] for x in rows if x[0] == r])) for r in reps],
        "n": [sum(1 for x in rows if x[0] == r) for r in reps],
        "abs_p": [float(np.median([x[2] for x in rows if x[0] == r])) for r in reps],
        "phase_deg": [float(np.median([x[3] for x in rows if x[0] == r])) for r in reps],
    }
    return fit


def attach_sine_to_meta(meta: dict[str, Any], fit: SineFit, source: str) -> bool:
    """meta に local_gain_accel_hf（と _hz・_source・_n_segments・_n_used）を入れる。
    倍率の表が無ければ False（meta は変えない）。"""
    if fit.entry is None:
        return False
    meta["local_gain_accel_hf"] = fit.entry
    meta["local_gain_accel_hf_hz"] = float(fit.hz)
    meta["local_gain_accel_hf_source"] = source
    meta["local_gain_accel_hf_n_segments"] = int(fit.n_segments)
    meta["local_gain_accel_hf_n_used"] = int(fit.n_used)
    return True


def hf_ratio_at(entry: Mapping[str, Any] | None, v_kmh: float) -> float | None:
    """meta local_gain_accel_hf の倍率 r(v)（車速で直線補間・測った範囲の外は端の値）。
    entry が無い・空なら None（呼ぶ側で k ＝ 1）。"""
    if not entry:
        return None
    reps = np.asarray(entry.get("speed_rep_kmh", []), dtype=float)
    ratio = np.asarray(entry.get("ratio", []), dtype=float)
    if len(reps) == 0 or len(reps) != len(ratio):
        return None
    return float(np.interp(float(v_kmh), reps, ratio))


@dataclass(frozen=True)
class ExciteBlock:
    """加振 CSV のブロック（アクセル。`<車速>kmh_<周波数>Hz`）の時系列。"""

    pattern: str
    v_star_kmh: float
    hz: float
    t_s: np.ndarray
    speed_kmh: np.ndarray
    accel_actual_pct: np.ndarray
    accel_cmd_pct: np.ndarray


def read_excite_blocks(csv_path: Path, hz: float | None = None) -> list[ExciteBlock]:
    """加振 CSV（section == EXCITE）のアクセルのブロックを読む。hz を渡すとその周波数だけ。
    時刻は mode_time_s（excite.analyze と同じ）。"""
    groups: dict[str, list[tuple[float, float, float, float]]] = {}
    order: list[str] = []
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("section") != "EXCITE":
                continue
            pat = r.get("pattern", "") or ""
            m = EXCITE_BLOCK_RE.match(pat)
            if m is None or (hz is not None and abs(float(m.group("f")) - hz) > 1e-9):
                continue
            t = _num(r.get("mode_time_s"))
            v = _num(r.get("actual_speed_kmh"))
            cmd = _num(r.get("accel_cmd_pct"))
            act = _num(r.get("accel_actual_pct"))
            if act is None:
                act = cmd
            if cmd is None:
                cmd = act
            if t is None or v is None or act is None or cmd is None:
                continue
            if pat not in groups:
                groups[pat] = []
                order.append(pat)
            groups[pat].append((t, v, act, cmd))
    out: list[ExciteBlock] = []
    for pat in order:
        a = np.array(groups[pat], dtype=float)
        m = EXCITE_BLOCK_RE.match(pat)
        assert m is not None
        out.append(
            ExciteBlock(
                pat, float(m.group("v")), float(m.group("f")),
                a[:, 0], a[:, 1], a[:, 2], a[:, 3],
            )
        )
    return out
