"""ペダルの遊びの終わり（効き始めの開度）を、G ランプ走行の時系列から折れ点で求める。

2026-10-16 段A: 待機位置 min から踏み込んでも、「効き始め」の開度までは加速度が変わらない
（ペダルの遊び。試験車の模擬では アクセル約10%・ブレーキ約6.5%）。この効き始めを
「惰行との差 d」が 0 から立ち上がる折れ目として求める。

求め方:
  1. 加速度 a: 各サンプルの前後 ±SLOPE_HALF_WINDOW_S の最小二乗の傾き（中心窓。遅れ 0 として扱う。
     遅れを入れるともう少し浅い側に出るが、遅れ 0 のほうが深い側＝安全側に出る）。
  2. d = sign × (a + 惰行減速量(v))。アクセルは +1、ブレーキは −1（どちらも「効いた向き」が正）。
  3. 使う行: 踏み込み中（開度が減っていない）・反対のペダルが待機位置・走行中・有限な a。
  4. 開度を BIN_PCT 幅でまとめ、d の中央値をとる（BIN の行数が MIN_BIN_SAMPLES 未満は捨てる）。
  5. d = k·max(0, u − u0) を折れ目 u0 の格子探索＋最小二乗で当てはめ、深い側にずれないよう
     u0 + REFIT_SPAN_PCT までのまとまりだけで一度当て直す。
  6. k > 0 かつ折れ目の手前・先に十分なまとまりがあるときだけ採用する。

純関数のみ（config・ハードウェアには触らない）。src/ は import しない。
"""

from __future__ import annotations

import argparse
import csv
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# 0.4 秒の中心窓。054131 の検算と同じ。
SLOPE_HALF_WINDOW_S = 0.2
# 開度のまとめ幅 [%]。仮値（054131 の検算と同じ）。
BIN_PCT = 0.5
# まとまりを採る最小行数。50 ms 周期で 0.25 秒ぶん。仮値。
MIN_BIN_SAMPLES = 5
# 折れ目 u0 の探索刻み [%]。BIN_PCT より十分細かくする。
KNEE_STEP_PCT = 0.05
# 当て直しで使う範囲（u0 から先 何 %）。強く踏むほど効きが急に増えるため、全域で当てると
# 折れ目が深い側にずれる。それを防ぐ仮値。054131 のブレーキで 6.5〜8.75% の範囲から 6.6%。
REFIT_SPAN_PCT = 2.0
# 当て直しの最大回数。折れ目が動かなくなったら途中で止める。
REFIT_MAX_ITER = 8
# 折れ目の手前（効かない側）に最低限ほしいまとまりの数。仮値。
MIN_FLAT_BINS = 2
# 折れ目の先（効く側）に最低限ほしいまとまりの数。仮値。
MIN_EFFECT_BINS = 2
# 反対ペダル・自ペダル下限の判定の許容 [%]。アクチュエータの位置の揺れ。
OTHER_MIN_TOL_PCT = 0.2
# 効き始めの計算に使わない phase。HOLD_STEP は遅れの測定の階段、dither（2026-10-28 段3）は
# 効き始めの先での小さな動き（開度を意図的に往復させる行で、折れ点の当てはめを乱す）。
# CSV の phase 名（pattern_loop の _Phase の名前と同じ。tests 内で独立に持つ）
EXCLUDED_PHASES = frozenset({"HOLD_STEP", "RAMP_BRAKE_DITHER", "RAMP_ACCEL_DITHER"})
# これ以下は停車とみなして捨てる [km/h]。fwd_data.STOP_SPEED_KMH と同じ値。
STOP_SPEED_KMH = 0.5


@dataclass(frozen=True)
class OnsetResult:
    """効き始めの求めた結果。"""

    onset_pct: float | None  # 絶対開度 [%]。採用できなければ None
    k: float | None  # 折れ目より先の傾き [km/h/s per %]（効く向きが正）
    bins: tuple[tuple[float, float, int], ...]  # (まとまりの中心開度, d の中央値, 行数)
    reason: str  # None のときの理由（採用時は ""）


def coast_fn_from_curve(
    speeds_kmh: Sequence[float], decel_kmhs: Sequence[float]
) -> Callable[[np.ndarray], np.ndarray]:
    """惰行カーブ（速度→減速量。正値）を補間する関数を返す。両端は端の値で頭打ち。"""
    xs = np.asarray(speeds_kmh, dtype=float)
    ys = np.asarray(decel_kmhs, dtype=float)

    def fn(v: np.ndarray) -> np.ndarray:
        return np.interp(np.asarray(v, dtype=float), xs, ys)

    return fn


def centered_slope(t_s: np.ndarray, v: np.ndarray, half_window_s: float) -> np.ndarray:
    """各サンプルの ±half_window_s の最小二乗の傾き。3 点未満は NaN。累積和でベクトル化。"""
    t = np.asarray(t_s, dtype=float) - float(t_s[0])
    y = np.asarray(v, dtype=float)
    n_all = len(t)
    lo = np.searchsorted(t, t - half_window_s, side="left")
    hi = np.searchsorted(t, t + half_window_s, side="right")  # 半開区間 [lo, hi)

    def cs(x: np.ndarray) -> np.ndarray:
        return np.concatenate(([0.0], np.cumsum(x)))

    c_t, c_y, c_tt, c_ty = cs(t), cs(y), cs(t * t), cs(t * y)

    def rng(c: np.ndarray) -> np.ndarray:
        return c[hi] - c[lo]

    n = (hi - lo).astype(float)
    s_t, s_y, s_tt, s_ty = rng(c_t), rng(c_y), rng(c_tt), rng(c_ty)
    den = n * s_tt - s_t * s_t
    with np.errstate(divide="ignore", invalid="ignore"):
        slope = (n * s_ty - s_t * s_y) / den
    slope[(n < 3) | ~np.isfinite(slope)] = np.nan
    return slope[:n_all]


def _hinge_fit(
    centers: np.ndarray, d: np.ndarray
) -> tuple[float, float, bool] | None:
    """d = k·max(0, u − u0) を u0 の格子探索で当てる。(u0, k, 格子の端に張り付いたか) を返す。"""
    if len(centers) < MIN_FLAT_BINS + MIN_EFFECT_BINS:
        return None
    best: tuple[float, float, float] | None = None  # (err, u0, k)
    # 格子の両端を、手前・先に最低限のまとまりが残る範囲に絞る。絞らないと、最後の 1 点だけに
    # 折れ目を合わせる縮退解（誤差が小さく見える）を拾ってしまう（054131 のアクセルで発生）。
    lo = centers[MIN_FLAT_BINS - 1] + 1e-9
    hi = centers[-MIN_EFFECT_BINS] - 1e-9
    for u0 in np.arange(lo, hi, KNEE_STEP_PCT):
        x = np.maximum(0.0, centers - u0)
        xx = float(x @ x)
        if xx == 0.0:
            continue
        k = float(x @ d) / xx
        err = float(np.sum((d - k * x) ** 2))
        if best is None or err < best[0]:
            best = (err, float(u0), k)
    if best is None:
        return None
    return best[1], best[2], (best[1] <= lo + KNEE_STEP_PCT or best[1] >= hi - KNEE_STEP_PCT)


def _refit(
    all_centers: np.ndarray, all_dv: np.ndarray, knee: float
) -> tuple[tuple[float, float, bool] | None, np.ndarray, np.ndarray]:
    """折れ目 knee + span 以下のまとまりで当て直す。(当て結果, 使ったcenters, 使ったd) を返す。

    2026-10-16 段2o-7: アクセルは効きが段のように急に立ち上がり、全域の最初の当て込みが手前に
    出ると、REFIT_SPAN_PCT の範囲に効いた所がほとんど入らず、平らな所だけで当てて端に張り付く
    （None）か惰行基準の ±0.03 の差で誤った値（135724 で C8 の表の惰行基準だとアクセルが
    5.40%）になった。結果が使えないとき（当てられない・端に張り付く・折れ目より先のまとまりが
    MIN_EFFECT_BINS 未満・k ≤ 0）は span を BIN_PCT ずつ広げ、全部のまとまりを含むまで直らなければ
    その結果のまま返す（後段の判定が None と理由にする）。
    """
    span = REFIT_SPAN_PCT
    while True:
        keep = all_centers <= knee + span
        centers, dv = all_centers[keep], all_dv[keep]
        fit = _hinge_fit(centers, dv)
        bad = (
            fit is None
            or fit[2]
            or int(np.sum(centers > fit[0])) < MIN_EFFECT_BINS
            or fit[1] <= 0.0
        )
        if not bad or bool(keep.all()):
            return fit, centers, dv
        span += BIN_PCT


def onset_from_series(
    t_s: np.ndarray,
    v_kmh: np.ndarray,
    u_pedal_pct: np.ndarray,
    u_other_pct: np.ndarray,
    *,
    pedal_min_pct: float,
    other_min_pct: float,
    coast_fn: Callable[[np.ndarray], np.ndarray],
    sign: float,
) -> OnsetResult:
    """時系列から効き始めの開度（絶対 [%]）を求める。sign はアクセル +1・ブレーキ −1。"""
    t = np.asarray(t_s, dtype=float)
    v = np.asarray(v_kmh, dtype=float)
    u = np.asarray(u_pedal_pct, dtype=float)
    uo = np.asarray(u_other_pct, dtype=float)
    if len(t) < 3:
        return OnsetResult(None, None, (), "行数が足りない")

    a = centered_slope(t, v, SLOPE_HALF_WINDOW_S)
    d = sign * (a + coast_fn(v))
    du = np.gradient(u, t)
    mask = (
        (du >= 0.0)
        & (uo <= other_min_pct + OTHER_MIN_TOL_PCT)
        & (v > STOP_SPEED_KMH)
        & (u >= pedal_min_pct - OTHER_MIN_TOL_PCT)
        & np.isfinite(d)
    )
    idx = np.floor((u[mask] - pedal_min_pct) / BIN_PCT).astype(int)
    dm = d[mask]
    rows: list[tuple[float, float, int]] = []
    for b in np.unique(idx):
        sel = idx == b
        n = int(sel.sum())
        if n < MIN_BIN_SAMPLES:
            continue
        center = pedal_min_pct + (float(b) + 0.5) * BIN_PCT
        rows.append((center, float(np.median(dm[sel])), n))
    bins = tuple(rows)
    if len(bins) < MIN_FLAT_BINS + MIN_EFFECT_BINS:
        return OnsetResult(None, None, bins, f"まとまりが足りない（{len(bins)} 個）")

    centers = np.array([r[0] for r in bins])
    dv = np.array([r[1] for r in bins])
    first = _hinge_fit(centers, dv)
    if first is None:
        return OnsetResult(None, None, bins, "折れ目を当てられない")
    all_centers, all_dv = centers, dv
    # 全域で当てると効きが急に増える分で折れ目が手前にずれる。ずれた折れ目から REFIT_SPAN_PCT
    # までで当て直すと、その範囲がまた狭すぎて端に張り付くことがある（054131 で確認）ため、
    # 折れ目が動かなくなるまで（最大 REFIT_MAX_ITER 回）当て直しを繰り返す。
    fit = first
    for _ in range(REFIT_MAX_ITER):
        new_fit, centers, dv = _refit(all_centers, all_dv, fit[0])
        if new_fit is None:
            return OnsetResult(None, None, bins, "当て直しで折れ目を当てられない")
        converged = abs(new_fit[0] - fit[0]) < KNEE_STEP_PCT / 2
        fit = new_fit
        if converged:
            break
    u0, k, at_edge = fit
    if at_edge:
        return OnsetResult(None, k, bins, f"折れ目が探索範囲の端に張り付いた（{u0:.2f}%）")
    n_flat = int(np.sum(centers < u0))
    n_eff = int(np.sum(centers > u0))
    if k <= 0.0:
        return OnsetResult(None, k, bins, f"傾きが正でない（k={k:.3f}）")
    if n_flat < MIN_FLAT_BINS:
        return OnsetResult(None, k, bins, f"折れ目の手前のまとまりが足りない（{n_flat} 個）")
    if n_eff < MIN_EFFECT_BINS:
        return OnsetResult(None, k, bins, f"折れ目の先のまとまりが足りない（{n_eff} 個）")
    return OnsetResult(u0, k, bins, "")


def _read_csv(path: Path, pattern_contains: str) -> dict[str, np.ndarray]:
    """G ランプ行（pattern に部分一致・EXCLUDED_PHASES 以外）だけ読む。"""
    cols: dict[str, list[float]] = {
        "t": [],
        "v": [],
        "acc": [],
        "brk": [],
    }
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            if pattern_contains not in row.get("pattern", ""):
                continue
            if row.get("phase", "") in EXCLUDED_PHASES:
                continue
            try:
                vals = (
                    float(row["elapsed_s"]),
                    float(row["actual_speed_kmh"]),
                    float(row["accel_actual_pct"]),
                    float(row["brake_actual_pct"]),
                )
            except (ValueError, KeyError):
                continue
            for key, val in zip(("t", "v", "acc", "brk"), vals, strict=True):
                cols[key].append(val)
    return {k: np.asarray(x) for k, x in cols.items()}


def onsets_from_csv(
    csv_path: Path,
    *,
    accel_min: float,
    brake_min: float,
    coast_fn: Callable[[np.ndarray], np.ndarray],
    pattern_contains: str = "G_RAMP",
) -> tuple[OnsetResult, OnsetResult]:
    """G ランプ走行の CSV からアクセル・ブレーキの効き始めを求める（2026-10-16 段2o-6。
    CLI とモデル作成（pkl の meta）が共用）。戻り値は (アクセル, ブレーキ)。"""
    data = _read_csv(csv_path, pattern_contains)
    acc = onset_from_series(
        data["t"], data["v"], data["acc"], data["brk"],
        pedal_min_pct=accel_min, other_min_pct=brake_min, coast_fn=coast_fn, sign=1.0,
    )  # fmt: skip
    brk = onset_from_series(
        data["t"], data["v"], data["brk"], data["acc"],
        pedal_min_pct=brake_min, other_min_pct=accel_min, coast_fn=coast_fn, sign=-1.0,
    )  # fmt: skip
    return acc, brk


def main(argv: Sequence[str] | None = None) -> int:
    from tests.research.config import DEFAULT_CONFIG_PATH, load_config
    from tests.research.term import say

    ap = argparse.ArgumentParser(description="G ランプ走行の CSV からペダルの効き始めを求める")
    ap.add_argument("csv", type=Path)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--patterns-contains", default="G_RAMP")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    data = _read_csv(args.csv, args.patterns_contains)
    say(f"使う行: {len(data['t'])}")
    coast = coast_fn_from_curve(
        cfg.feedforward.coast_decel_speeds_kmh, cfg.feedforward.coast_decel_kmhs
    )
    amin = cfg.vehicle.min_accel_opening_pct
    bmin = cfg.vehicle.min_brake_opening_pct
    for name, u, uo, pmin, omin, sign in (
        ("アクセル", data["acc"], data["brk"], amin, bmin, 1.0),
        ("ブレーキ", data["brk"], data["acc"], bmin, amin, -1.0),
    ):
        res = onset_from_series(
            data["t"], data["v"], u, uo,
            pedal_min_pct=pmin, other_min_pct=omin, coast_fn=coast, sign=sign,
        )  # fmt: skip
        onset = "なし" if res.onset_pct is None else f"{res.onset_pct:.2f}%"
        k = "-" if res.k is None else f"{res.k:.3f}"
        say(f"[{name}] 効き始め={onset}  k={k} km/h/s per %  理由={res.reason or '-'}")
        for center, med, n in res.bins:
            say(f"    開度 {center:6.2f}%  d中央値 {med:+.3f}  行数 {n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
