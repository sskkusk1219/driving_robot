"""4 点の保持（LOW_HOLD）の「加速度の符号が変わる瞬間」から、点の開度・車速を求める（段10-1 A）。

シミュレータのブレーキの読みは 1 目盛り ≒ 0.22% で、1 目盛りで加速度が約 3 km/h/s 跳ぶため、
「加速度 ±0.005 G の帯に合計 5 s」は当たり外れになる。そこで点ごとに、保持中に加速度が
+→− と −→+ に変わる瞬間（車が止まっている 0.3 km/h 未満は除く）を集め、その瞬間の
ブレーキの実開度（ブレーキの遅れ L だけ前）の平均を点の開度、瞬間の車速の平均を点の車速にする。
瞬間が 2 回以上なら「保てた」（帯の判定と OR。pedal_search.apply_crossings）。

純関数だけ（オンライン・CSV のどちらからも使える）。tests/research だけで完結する。
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PHASE_LOW_HOLD = "LOW_HOLD"
ACCEL_WINDOW_S = 0.4  # 加速度 ＝ 車速の中心差分の幅 [s]
SKIP_S = 3.0  # 点の始まりからこの間は数えない（基準を上げて寄せる間）[s]
MIN_SPEED_KMH = 0.3  # これ未満（停車）の瞬間は数えない [km/h]
POINT_GAP_S = 1.0  # elapsed_s がこれ以上空いたら別の点 [s]
MIN_CROSSINGS = 2  # 「保てた」になる瞬間の数


@dataclass(frozen=True)
class Crossing:
    t_s: float
    speed_kmh: float
    opening_pct: float  # t − L のブレーキの実開度
    direction: int  # +1 ＝ −→+、−1 ＝ +→−


@dataclass(frozen=True)
class CrossingSummary:
    count: int
    opening_pct: float | None  # 瞬間の開度の平均
    speed_kmh: float | None  # 瞬間の車速の平均
    n_up: int = 0
    n_down: int = 0
    spread_pct: float | None = None  # 開度の標準偏差（点の中のばらつき）


def central_accel(
    t_s: np.ndarray, v_kmh: np.ndarray, window_s: float = ACCEL_WINDOW_S
) -> np.ndarray:
    """車速の中心差分の加速度 [km/h/s]。両端（窓が時刻の範囲外）は NaN。"""
    half = window_s / 2.0
    a = np.full(len(t_s), np.nan)
    if len(t_s) < 2:
        return a
    ok = (t_s - half >= t_s[0]) & (t_s + half <= t_s[-1])
    hi = np.interp(t_s[ok] + half, t_s, v_kmh)
    lo = np.interp(t_s[ok] - half, t_s, v_kmh)
    a[ok] = (hi - lo) / window_s
    return a


def find_crossings(
    t_s: np.ndarray,
    v_kmh: np.ndarray,
    u_pct: np.ndarray,
    lag_s: float,
    *,
    skip_s: float = SKIP_S,
    min_speed_kmh: float = MIN_SPEED_KMH,
    window_s: float = ACCEL_WINDOW_S,
) -> list[Crossing]:
    """1 点（時刻が昇順）の中の、加速度の符号が変わる瞬間の一覧。

    加速度が 0 ちょうどの行は飛ばし、0 でない直前の符号と比べる。瞬間の時刻は 2 行の間を
    線形補間で求める。点の始まりから skip_s 以内・その瞬間の車速が min_speed_kmh 未満のものは除く。
    開度は t − lag_s の u_pct（線形補間。範囲外は端の値）。
    """
    t = np.asarray(t_s, dtype=float)
    v = np.asarray(v_kmh, dtype=float)
    u = np.asarray(u_pct, dtype=float)
    if len(t) < 3:
        return []
    a = central_accel(t, v, window_s)
    out: list[Crossing] = []
    j = -1
    for i in range(len(t)):
        if not np.isfinite(a[i]) or a[i] == 0.0:
            continue
        if j >= 0 and (a[j] > 0.0) != (a[i] > 0.0):
            tc = t[j] + (t[i] - t[j]) * a[j] / (a[j] - a[i])
            speed = float(np.interp(tc, t, v))
            if tc - t[0] >= skip_s and speed >= min_speed_kmh:
                out.append(
                    Crossing(
                        float(tc), speed, float(np.interp(tc - lag_s, t, u)),
                        1 if a[i] > 0.0 else -1,
                    )
                )
        j = i
    return out


def summarize(crossings: list[Crossing]) -> CrossingSummary:
    if not crossings:
        return CrossingSummary(0, None, None)
    op = np.array([c.opening_pct for c in crossings])
    return CrossingSummary(
        count=len(crossings),
        opening_pct=float(op.mean()),
        speed_kmh=float(np.mean([c.speed_kmh for c in crossings])),
        n_up=sum(1 for c in crossings if c.direction > 0),
        n_down=sum(1 for c in crossings if c.direction < 0),
        spread_pct=float(op.std()),
    )


def split_segments(t_s: np.ndarray, gap_s: float = POINT_GAP_S) -> list[tuple[int, int]]:
    """時刻が gap_s 以上空いた所で点に分ける（行の半開区間）。"""
    if len(t_s) == 0:
        return []
    cuts = [0] + [i for i in range(1, len(t_s)) if t_s[i] - t_s[i - 1] >= gap_s]
    return list(zip(cuts, cuts[1:] + [len(t_s)], strict=True))


def read_low_hold_rows(csv_path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """CSV の LOW_HOLD の行 (elapsed_s, actual_speed_kmh, brake_actual_pct)。欠けた行は除く。"""
    rows: list[tuple[float, float, float]] = []
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("phase") != PHASE_LOW_HOLD:
                continue
            try:
                rows.append(
                    (float(r["elapsed_s"]), float(r["actual_speed_kmh"]),
                     float(r["brake_actual_pct"]))
                )
            except (KeyError, TypeError, ValueError):
                continue
    a = np.array(rows, dtype=float).reshape(-1, 3)
    ok = np.isfinite(a).all(axis=1)
    return a[ok, 0], a[ok, 1], a[ok, 2]


def crossings_from_csv(csv_path: Path, lag_s: float) -> list[list[Crossing]]:
    """CSV の LOW_HOLD を点ごとに分け、点ごとの瞬間の一覧を返す（机上・オンラインで同じ関数）。"""
    t, v, u = read_low_hold_rows(csv_path)
    return [find_crossings(t[i0:i1], v[i0:i1], u[i0:i1], lag_s) for i0, i1 in split_segments(t)]


def main(argv: list[str] | None = None) -> int:
    """机上: CSV の LOW_HOLD から点ごとの瞬間の数・開度・車速を表にする
    （python -m tests.research.hold_crossing CSV --lag 0.155）。"""
    import argparse  # noqa: PLC0415

    ap = argparse.ArgumentParser(description="4 点の保持の瞬間の判定（机上）")
    ap.add_argument("csv", type=Path)
    ap.add_argument("--lag", type=float, required=True, help="ブレーキの遅れ L [s]")
    args = ap.parse_args(argv)
    print("点 | 瞬間 | +→− | −→+ | 開度 % | 車速 km/h | 開度の標準偏差 %")
    for i, cs in enumerate(crossings_from_csv(args.csv, args.lag), 1):
        s = summarize(cs)
        if s.count == 0:
            print(f"{i} | 0 | - | - | - | - | -")
            continue
        print(
            f"{i} | {s.count} | {s.n_down} | {s.n_up} | {s.opening_pct:.3f} | "
            f"{s.speed_kmh:.3f} | {s.spread_pct:.3f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
