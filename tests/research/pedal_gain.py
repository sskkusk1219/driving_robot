"""手順 2-2 ペダルゲイン推定（本番 estimate_dynamics_params のゲイン部分の研究用置き換え）。

ペダルゲイン = 開度 1% あたり、惰行からどれだけ加速度が変わるか [km/h/s per %]（正値）。
本番は「不感帯 + PEDAL_GAIN_MIN_OPENING_PCT（5%）以上」のサンプルだけで推定する。研究ハーネスの
車両はブレーキの効きが急（不感帯 13.68% → 16% ≈ 0.2G → 20% ≈ 0.41G）で、+5% ≈ 0.38G は減速G
ガバナーの上限ぎりぎりのため定常サンプルが採れず、ブレーキゲインが未同定になった。

ここでは「不感帯 + 何% 以上」だけを YAML（learning.accel/brake_gain_min_offset_pct）で差し替え、
それ以外は本番の補助関数で同じ計算をする:
    使うサンプル … もう片方のペダルが不感帯以下・開度 − 不感帯 ≥ min_offset・クリープより速い・
                   開度が直近 1s ほぼ一定（本番 _steady_opening_mask）
    ゲイン       … 惰行基準（前回同定の惰行カーブ）からの加速度差 ÷ (開度 − 不感帯)
    曲線         … 10 km/h 幅の速度帯ごとの中央値（8 点以上の帯が 2 つ以上で同定。本番
                   _estimate_pedal_gain_curve）
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from src.domain.model_training import (
    DEFAULT_DT_S,
    _estimate_pedal_gain_curve,
    _group_by_session,
    _merge_pedal_gain_curves,
    _steady_opening_mask,
)
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams, coast_decel_at


@dataclass(frozen=True)
class GainCurve:
    speeds_kmh: tuple[float, ...]
    gains: tuple[float, ...]
    samples: int  # 条件を満たしたサンプル数（ノイズで非正になったゲインを捨てる前）

    @property
    def identified(self) -> bool:
        return bool(self.speeds_kmh)


def estimate_gain_curve(
    logs: list[DriveLog],
    params: FeedforwardParams,
    *,
    is_accel: bool,
    min_offset_pct: float,
) -> GainCurve:
    """走行ログからアクセル側またはブレーキ側のゲイン曲線を推定する（不感帯・惰行は params）。"""
    speeds: list[float] = []
    gains: list[float] = []
    for session_logs in _group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([lg.actual_speed_kmh for lg in session_logs], dtype=float), 0.0, None
        )
        accel = np.array([lg.accel_opening for lg in session_logs], dtype=float)
        brake = np.array([lg.brake_opening for lg in session_logs], dtype=float)
        dt = _sample_interval_s(session_logs)
        dv = np.diff(speed) / dt  # i→i+1 の加速度 [km/h/s]
        sp = speed[:-1]
        if is_accel:
            opening, db = accel, params.accel_deadband_pct
            other, other_db = brake, params.brake_deadband_pct
        else:
            opening, db = brake, params.brake_deadband_pct
            other, other_db = accel, params.accel_deadband_pct
        mask = (
            (other[:-1] <= other_db)
            & (opening[:-1] - db >= min_offset_pct)
            & (sp > params.creep_speed_kmh)
            & _steady_opening_mask(opening, dt)[:-1]
        )
        if not np.any(mask):
            continue
        a_coast = np.array([-coast_decel_at(params, float(v)) for v in sp[mask]], dtype=float)
        effect = dv[mask] - a_coast if is_accel else a_coast - dv[mask]
        speeds.extend(sp[mask].tolist())
        gains.extend((effect / (opening[:-1][mask] - db)).tolist())
    curve_speeds, curve_gains = _estimate_pedal_gain_curve(np.array(speeds), np.array(gains))
    return GainCurve(speeds_kmh=curve_speeds, gains=curve_gains, samples=len(speeds))


def apply_pedal_gains(
    params: FeedforwardParams, *, accel: GainCurve, brake: GainCurve
) -> FeedforwardParams:
    """同定できた側の曲線で params のペダルゲインを置き換える（共通の速度グリッドに載せ直す）。

    同定できなかった側は params の値（本番推定の結果）を使う。
    """
    grid = params.pedal_gain_speeds_kmh

    def side(curve: GainCurve, values: tuple[float, ...]) -> tuple[tuple[float, ...], ...]:
        if curve.identified:
            return curve.speeds_kmh, curve.gains
        if len(grid) >= 2 and len(values) == len(grid):
            return grid, values
        return (), ()

    speeds_a, gains_a = side(accel, params.accel_gain_kmhs_per_pct)
    speeds_b, gains_b = side(brake, params.brake_gain_kmhs_per_pct)
    merged_grid, merged_a, merged_b = _merge_pedal_gain_curves(
        speeds_a, gains_a, speeds_b, gains_b
    )
    if not merged_grid:
        return params
    return replace(
        params,
        pedal_gain_speeds_kmh=merged_grid,
        accel_gain_kmhs_per_pct=merged_a,
        brake_gain_kmhs_per_pct=merged_b,
    )


def _sample_interval_s(session_logs: list[DriveLog]) -> float:
    """ログ周期 [s]（時刻差の中央値。本番 estimate_dynamics_params と同じ）。"""
    epochs = np.array([lg.timestamp.timestamp() for lg in session_logs])
    d = np.diff(epochs)
    d = d[d > 0.0]
    dt = float(np.median(d)) if len(d) > 0 else DEFAULT_DT_S
    return dt if dt > 0.0 else DEFAULT_DT_S
