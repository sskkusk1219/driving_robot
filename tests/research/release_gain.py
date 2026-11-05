"""手順2・手順14 の既存の走行ログから、ブレーキを戻したときの効き（戻す側の効き）を測る。

2026-11-05 段14（ProblemReport_20261103_停車前の這う基準）。停車へ向かう 1 km/h 以下で遅すぎのとき
ブレーキを少しずつ戻す案（段15）は、1 回に戻す量を「要る加速度の変化 ÷ 効き」で決める。効きは
「ブレーキを 1% 戻したとき L 後に加速度が何 km/h/s 変わるか」で、車ごとに違うので、手順2・手順14 の
走行ログの「減速中の戻し」から pkl を作る計算で測り、meta に残す（手順2 の走り方・測定は
増やさない）。

求め方（1 つの山 = 実開度が下がり始めた所）:
  1. 行: section が PATTERN_DRIVE、行間が GAP_S 未満の連続区間。実開度 `brake_actual_pct`、車速は
     `actual_speed_kmh`。加速度は車速の微分（3 行の移動平均の中心差分）。
  2. 山の始まり i: 実開度が下がり始めた行（前の行までは下がっていない）で、
     始まり前の行（i−1）の車速が 0.5〜max_speed_kmh・加速度が accel_max_kmhs（−1.0）以下
     （減速中）・開度が min ＋ 効き始めより深い（効き始めより浅いと戻しても効かない）。
  3. 開度が始まり前の行から amount_pct（0.3%）下がった最初の行 j まで、実開度が下がり続けている
     （上がらない）こと。j から遅れ L 後の行 k で、車速が止まっていない（> 0.05 km/h）こと。
  4. 効き = （k の加速度 − 始まり前の行の加速度）÷ amount_pct。全部の山の中央値。山が
     MIN_PEAKS 未満なら測れず。

純関数（config・ハードウェアには触らない）。src/ は import しない。
"""

from __future__ import annotations

import csv
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# CSV の section 名（drive_log.SECTION_PATTERN_DRIVE と同じ。独立に持つ）
SECTION_PATTERN_DRIVE = "PATTERN_DRIVE"
# 測る戻し量 [%]。半分データ由来（0.1% はばらつきで使えない・W 386 に要る量 0.4% に近い）。
# 段14 の机上で 0.5% と並べて表示。
AMOUNT_PCT = 0.3
# 減速中とみる始まりの加速度 [km/h/s]。仮の値（根拠なし）。段14 で山の数を表示。
ACCEL_MAX_KMHS = -1.0
# 山を数える車速の下限 [km/h]（停車間際は除く）
MIN_SPEED_KMH = 0.5
# 山を数える車速の上限の既定 [km/h]（learning.stop_release_start_kmh と同じ 7.5。呼ぶ側が渡せる）
MAX_SPEED_KMH = 7.5
# 遅れ L の後でも止まっていないとみる車速 [km/h]
MOVING_KMH = 0.05
# 区間の行が途切れたとみなす間隔 [s]（local_gain.GAP_S と同じ値。独立に持つ）
GAP_S = 0.5
# 効きを出すのに要る山の数
MIN_PEAKS = 3
# 開度が「下がっていない」とみる許容 [%]
OPEN_EPS_PCT = 1e-9


@dataclass
class ReleaseGain:
    """戻す側の効き。gain_kmhs_pct が None なら測れず（reason に理由）。"""

    gain_kmhs_pct: float | None
    n_peaks: int = 0
    values: list[float] = field(default_factory=list)  # 山ごとの効き
    csv_names: list[str] = field(default_factory=list)  # 山ごとの CSV 名
    per_csv: dict[str, list[float]] = field(default_factory=dict)  # CSV 名 → 山ごとの効き
    amount_pct: float = AMOUNT_PCT
    reason: str = ""


def _num(x: str | None) -> float | None:
    if x is None or x == "":
        return None
    try:
        v = float(x)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def read_segments(csv_path: Path) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """PATTERN_DRIVE の連続区間ごとの (時刻, 車速, ブレーキ実開度)。値が読めない行・
    行間が GAP_S 以上の所で区間を切る。"""
    segs: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    cur: list[tuple[float, float, float]] = []
    prev_t = math.nan

    def flush() -> None:
        nonlocal cur
        if len(cur) >= 5:
            a = np.array(cur, dtype=float)
            segs.append((a[:, 0], a[:, 1], a[:, 2]))
        cur = []

    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            t = _num(r.get("elapsed_s"))
            v = _num(r.get("actual_speed_kmh"))
            if v is None:
                v = _num(r.get("speed_kmh"))
            b = _num(r.get("brake_actual_pct"))
            if r.get("section", "") != SECTION_PATTERN_DRIVE or t is None or v is None or b is None:
                flush()
                prev_t = math.nan
                continue
            if cur and not (0.0 < t - prev_t < GAP_S):
                flush()
            prev_t = t
            cur.append((t, v, b))
    flush()
    return segs


def accel_of(t: np.ndarray, v: np.ndarray) -> np.ndarray:
    """加速度 [km/h/s]。車速の 3 行の移動平均（端は元のまま）の中心差分。"""
    sm = v.copy()
    if len(v) >= 3:
        sm[1:-1] = (v[:-2] + v[1:-1] + v[2:]) / 3.0
    return np.gradient(sm, t)


def peaks_in_segment(
    t: np.ndarray, v: np.ndarray, b: np.ndarray, *,
    depth_floor_pct: float, lag_s: float, max_speed_kmh: float,
    amount_pct: float = AMOUNT_PCT, accel_max_kmhs: float = ACCEL_MAX_KMHS,
) -> list[float]:
    """1 区間の山ごとの効き [km/h/s per %]。depth_floor_pct は山の始まりの実開度がこれより深い
    こと（min ＋ 効き始め）。"""
    a = accel_of(t, v)
    n = len(t)
    out: list[float] = []
    for i in range(2, n):
        p = i - 1
        # 山の始まり: i で下がり始め、その前（p−1 → p）は下がっていない
        if not (b[i] < b[p] - OPEN_EPS_PCT and b[p] >= b[p - 1] - OPEN_EPS_PCT):
            continue
        if not (MIN_SPEED_KMH <= v[p] <= max_speed_kmh) or a[p] > accel_max_kmhs:
            continue
        if b[p] <= depth_floor_pct:
            continue
        # 0.3% 下がるまで下がり続けること
        j = -1
        for m in range(i, n):
            if b[m] > b[m - 1] + OPEN_EPS_PCT:
                break
            if b[p] - b[m] >= amount_pct:
                j = m
                break
        if j < 0:
            continue
        ks = np.nonzero(t[j:] >= t[j] + lag_s)[0]
        if len(ks) == 0:
            continue
        k = j + int(ks[0])
        if v[k] <= MOVING_KMH:
            continue
        out.append(float((a[k] - a[p]) / amount_pct))
    return out


def measure_stop_release_gain(
    csv_paths: Sequence[Path], *, brake_min_pct: float, onset_u_pct: float, lag_s: float,
    max_speed_kmh: float = MAX_SPEED_KMH, amount_pct: float = AMOUNT_PCT,
    accel_max_kmhs: float = ACCEL_MAX_KMHS,
) -> ReleaseGain:
    """CSV の山から戻す側の効き（中央値）を測る。山が MIN_PEAKS 未満なら測れず。"""
    res = ReleaseGain(None, amount_pct=amount_pct)
    floor = brake_min_pct + onset_u_pct
    for p in csv_paths:
        name = Path(p).name
        try:
            segs = read_segments(Path(p))
        except OSError:
            continue
        for t, v, b in segs:
            vals = peaks_in_segment(
                t, v, b, depth_floor_pct=floor, lag_s=lag_s, max_speed_kmh=max_speed_kmh,
                amount_pct=amount_pct, accel_max_kmhs=accel_max_kmhs,
            )
            for g in vals:
                res.values.append(g)
                res.csv_names.append(name)
                res.per_csv.setdefault(name, []).append(g)
    res.n_peaks = len(res.values)
    if res.n_peaks < MIN_PEAKS:
        res.reason = f"山が {res.n_peaks} 個（{MIN_PEAKS} 個未満）"
        return res
    res.gain_kmhs_pct = float(np.median(res.values))
    return res


def attach_to_meta(meta: dict[str, Any], res: ReleaseGain) -> bool:
    """meta に stop_release_gain_*（効き・山の数・山ごとの値・CSV 名・測った戻し量）を入れる。
    測れていなければ何も入れず False。"""
    if res.gain_kmhs_pct is None:
        return False
    meta["stop_release_gain_kmhs_pct"] = float(res.gain_kmhs_pct)
    meta["stop_release_gain_n"] = int(res.n_peaks)
    meta["stop_release_gain_list"] = [float(x) for x in res.values]
    meta["stop_release_gain_csv"] = list(dict.fromkeys(res.csv_names))
    meta["stop_release_gain_amount_pct"] = float(res.amount_pct)
    return True
