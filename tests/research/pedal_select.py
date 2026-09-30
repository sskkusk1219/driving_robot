"""ペダル選択（アクセル／惰行／ブレーキ）に使う加速度の計算（純関数）。

ProblemReport_20260929: ペダル選択を「1 点の傾き」から「区間の傾き G」に置き換える案の部品。
`pedal_schedule`（ペダル予定表）と `mode_drive._cycle`（制御。段3）が同じ関数を使う。

- 方式 A（今の方式）: (基準(t+h1) − 基準(t)) / h1 の 1 点の傾き
- 方式 G（新方式）: 基準の「今から L 秒先」を中心にした幅 H 秒の窓の最小二乗の傾き
"""

from __future__ import annotations

from collections.abc import Callable

PEDAL_ACCEL = 1
PEDAL_COAST = 0
PEDAL_BRAKE = -1


def window_slope_kmhs(
    ref_at: Callable[[float], float],
    t_s: float,
    center_s: float,
    width_s: float,
    step_s: float = 0.1,
) -> float:
    """窓 [t+center−width/2, t+center+width/2] の基準車速の最小二乗の傾き [km/h/s]。

    `pedal_arbiter.plan_accel_kmhs` と同じ計算（step_s 刻みでサンプルして直線を当てる）で、
    窓の位置だけを一般化した。center=width/2 なら窓が [t, t+width] になり一致する。
    """
    n = max(1, round(width_s / step_s))
    start = t_s + center_s - width_s / 2.0
    ts = [k * step_s for k in range(n + 1)]
    vs = [ref_at(start + t) for t in ts]
    t_mean = sum(ts) / len(ts)
    v_mean = sum(vs) / len(vs)
    denom = sum((t - t_mean) ** 2 for t in ts)
    return sum((t - t_mean) * (v - v_mean) for t, v in zip(ts, vs, strict=True)) / denom


def point_accel_kmhs(ref_at: Callable[[float], float], t_s: float, h1_s: float) -> float:
    """方式 A: (基準(t+h1) − 基準(t)) / h1 [km/h/s]（`predict_effort` の desired_accel と同じ）。"""
    return (ref_at(t_s + h1_s) - ref_at(t_s)) / h1_s


def select_pedal(accel_kmhs: float, coast_kmhs: float, band_kmhs: float) -> int:
    """要求加速度と惰行加速度からペダルを選ぶ（+1 アクセル / 0 惰行 / −1 ブレーキ）。

    `ff_candidate.CandidateFeedforward.predict_effort` の規則と同じ比較（等号の扱いも同じ）:
    |要求 − 惰行| < 帯 なら惰行、それ以外は 要求 >= 惰行 でアクセル、下ならブレーキ。
    """
    if abs(accel_kmhs - coast_kmhs) < band_kmhs:
        return PEDAL_COAST
    return PEDAL_ACCEL if accel_kmhs >= coast_kmhs else PEDAL_BRAKE
