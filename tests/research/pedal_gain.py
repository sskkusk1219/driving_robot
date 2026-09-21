"""手順 2-2 ペダルゲイン推定(本番 estimate_dynamics_params のゲイン部分の研究用置き換え)。

ペダルゲイン = 開度 1% あたり、惰行からどれだけ加速度が変わるか [km/h/s per %](正値)。
本番は「不感帯 + PEDAL_GAIN_MIN_OPENING_PCT（5%）以上」のサンプルだけで推定する。研究ハーネスの
車両はブレーキの効きが急（不感帯 13.68% → 16% ≈ 0.2G → 20% ≈ 0.41G）で、+5% ≈ 0.38G は減速G
ガバナーの上限ぎりぎりのため定常サンプルが採れず、ブレーキゲインが未同定になった。

ここでは「不感帯 + 何% 以上」だけを YAML（learning.accel/brake_gain_min_offset_pct）で差し替え、
それ以外は本番の補助関数で同じ計算をする:
    使うサンプル … もう片方のペダルが不感帯以下・開度 − 不感帯 ≥ min_offset・
                   開度が直近 1s ほぼ一定（本番 _steady_opening_mask）
    ゲイン       … 基準（free_accel_at。惰行域は惰行カーブ、クリープ域はクリープ加速カーブ）
                   からの加速度差 ÷ (開度 − 不感帯)
    曲線         … 10 km/h 幅の速度帯ごとの中央値（8 点以上の帯が 2 つ以上で同定。本番
                   _estimate_pedal_gain_curve）

2026-09-17（ProblemReport_20260916 課題#2）で変えたところ:
    - 基準を `-coast_decel_at(params, v)`（v<creep_speed_kmh では端点クランプの値を誤って使って
      いた。符号ごと間違っていた）から `free_accel_at(params, research, v)`（クリープ域は
      +creep_accel_at、惰行域は従来どおり -coast_decel_at）へ置き換えた。
    - ブレーキ側のみ `sp > params.creep_speed_kmh` の下限制約を `sp > STOP_SPEED_KMH`
      （停車済み・速度クリップで dv≈0 になるだけのサンプルの除外）へ緩めた。アクセル側は変更しない
      （クリープ域はクリープ加速の引き戻し力が支配的でアクセルを踏む意味のあるサンプルが薄く、
      対象を広げると母集団がクリープ引き戻しと交絡するため。ブレーキ側は段1でクリープ域ブレーキ
      保持パターン `pattern_loop.CreepLaunchPattern(hold_after=True)` を新設し、低速のブレーキ
      ゲインを狙って測るため、除外すると新設パターンのサンプルが丸ごと捨てられてしまう）。
    - クリープ域（速度 < creep_speed_kmh）は速度レンジが 0〜5km/h 程度と狭く、本番の 10 km/h
      ビン（`_estimate_pedal_gain_curve`）では 1 ビンに潰れるため、`creep_bin_kmh`
      （`learning.creep_curve_bin_kmh`）の細いビンで別カーブを作り、高速側カーブと連結する
      （`_join_curves_by_speed`）。速度レンジが重ならないため補間は不要で、accel/brake という
      別変数を共通グリッドへ載せる `_merge_pedal_gain_curves` は用途が違い再利用できなかった。
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from src.domain.model_training import (
    DEFAULT_DT_S,
    STOP_SPEED_KMH,
    _estimate_pedal_gain_curve,
    _group_by_session,
    _merge_pedal_gain_curves,
    _steady_opening_mask,
)
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research.ff_params import ResearchFFParams, free_accel_at


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
    research: ResearchFFParams,
    *,
    is_accel: bool,
    min_offset_pct: float,
    creep_bin_kmh: float,
    creep_min_bin_samples: int,
) -> GainCurve:
    """走行ログからアクセル側またはブレーキ側のゲイン曲線を推定する（不感帯・惰行は params）。

    ブレーキ側はクリープ域（速度 < params.creep_speed_kmh）も対象にする（アクセル側は対象外。
    モジュール docstring 参照）。クリープ域のサンプルは `creep_bin_kmh` 幅の細いビンで別カーブに
    し、高速側カーブ（本番と同じ 10 km/h 幅）と連結する。
    """
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
            & _steady_opening_mask(opening, dt)[:-1]
        )
        if is_accel:  # クリープ域はアクセル側の対象外（モジュール docstring 参照）
            mask = mask & (sp > params.creep_speed_kmh)
        else:  # 停車済み（速度が 0 にクリップされ dv≈0 になる）サンプルは物理的に無意味なので除外
            mask = mask & (sp > STOP_SPEED_KMH)
        if not np.any(mask):
            continue
        a_free = np.array(
            [free_accel_at(params, research, float(v)) for v in sp[mask]], dtype=float
        )
        effect = dv[mask] - a_free if is_accel else a_free - dv[mask]
        speeds.extend(sp[mask].tolist())
        gains.extend((effect / (opening[:-1][mask] - db)).tolist())

    speeds_arr = np.array(speeds)
    gains_arr = np.array(gains)
    is_low = speeds_arr < params.creep_speed_kmh
    low_speeds, low_gains = _low_speed_gain_curve(
        speeds_arr[is_low], gains_arr[is_low],
        bin_kmh=creep_bin_kmh, min_bin_samples=creep_min_bin_samples,
    )
    high_speeds, high_gains = _estimate_pedal_gain_curve(speeds_arr[~is_low], gains_arr[~is_low])
    curve_speeds, curve_gains = _join_curves_by_speed(
        low_speeds, low_gains, high_speeds, high_gains
    )
    return GainCurve(speeds_kmh=curve_speeds, gains=curve_gains, samples=len(speeds))


def _low_speed_gain_curve(
    speeds: np.ndarray, gains: np.ndarray, *, bin_kmh: float, min_bin_samples: int
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """クリープ域（速度 < creep_speed_kmh）のゲインを `bin_kmh` 幅のビンで推定する。

    非正のゲイン（ノイズで符号が反転したサンプル）を捨てるのは本番 `_estimate_pedal_gain_curve`
    と同じ。ビン化そのものは `_estimate_pedal_gain_curve` と同形だがビン幅だけ変える
    （速度レンジが狭いクリープ域では本番の 10 km/h 幅だと 1 ビンに潰れるため）。
    """
    if len(speeds) == 0:
        return (), ()
    valid = gains > 0.0
    speeds = speeds[valid]
    gains = gains[valid]
    if len(speeds) == 0:
        return (), ()
    v_max = float(speeds.max())
    n_bins = int(v_max / bin_kmh) + 1
    out_speeds: list[float] = []
    out_gains: list[float] = []
    for i in range(n_bins):
        lo = i * bin_kmh
        hi = lo + bin_kmh
        mask = (speeds >= lo) & (speeds < hi)
        if int(np.count_nonzero(mask)) < min_bin_samples:
            continue
        out_speeds.append(lo + bin_kmh / 2.0)
        out_gains.append(float(np.median(gains[mask])))
    if len(out_speeds) < 2:
        return (), ()
    return tuple(out_speeds), tuple(out_gains)


def _join_curves_by_speed(
    low_speeds: tuple[float, ...],
    low_gains: tuple[float, ...],
    high_speeds: tuple[float, ...],
    high_gains: tuple[float, ...],
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """速度レンジが重ならない低速・高速の2本のカーブを1本へ連結する（速度昇順）。

    accel/brake という別々の変数を共通グリッドへ補間する `_merge_pedal_gain_curves` とは目的が
    違う（同一変数＝ブレーキゲインを速度レンジで分けて別ビン幅で推定したものを連結するだけ）ため
    再利用できず、ここに素朴な連結関数を書いた。速度レンジが重ならない前提なので補間は不要。
    """
    combined = sorted(
        [*zip(low_speeds, low_gains, strict=True), *zip(high_speeds, high_gains, strict=True)],
        key=lambda pair: pair[0],
    )
    if not combined:
        return (), ()
    speeds, gains = zip(*combined, strict=True)
    return speeds, gains


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
