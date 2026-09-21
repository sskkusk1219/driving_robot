"""研究段階のみの FF パラメータ（`src/models/profile.py:FeedforwardParams` を補う薄い器）。

ProblemReport_20260916 の課題#2（クリープ加速カーブと低速ブレーキゲインの再同定）に対応する。
`FeedforwardParams` は本番コードであり変更しないため、そこに無いパラメータ（クリープ加速カーブ・
惰行帯）はここに置く。段1時点では `coast_band_kmhs` はまだどこからも参照しない（段2で惰行レジームの
帯判定に使う）。

`free_accel_at` は「今ペダルを離したときの加速度 [km/h/s]」の単一ソース。クリープ域（v <
creep_speed_kmh）は `+creep_accel_at(v)`、それ以上は `-coast_decel_at(v)`（既存の惰行減速カーブ）。
低速ブレーキゲインの基準（`pedal_gain.py`）と、段2以降の惰行レジーム判定が同じ関数を使うことで、
基準のずれ（境界での符号違い）を防ぐ。
"""

from __future__ import annotations

from dataclasses import dataclass

from src.models.profile import FeedforwardParams, _interp_curve, coast_decel_at
from tests.research.config import ResearchConfig


@dataclass(frozen=True)
class ResearchFFParams:
    """`FeedforwardParams` を補う研究段階のみのパラメータ（`feedforward` セクションの残り）。"""

    # ── クリープ加速カーブ（速度依存） ─────────────────────────────────
    # 0〜creep_speed_kmh のクリープ加速度 a_creep(v)（正値）[km/h/s]。段1で
    # tests.research.creep_curve.estimate_creep_accel_curve が同定する。空タプル＝未同定。
    creep_accel_speeds_kmh: tuple[float, ...] = ()  # 速度グリッド [km/h]（昇順）
    creep_accel_kmhs: tuple[float, ...] = ()  # 各速度でのクリープ加速度（正値）[km/h/s]
    # 惰行とみなす要求加速度の帯（半幅）[km/h/s]。|a_req - free_accel| < この値なら惰行。
    # 段2（`ff_candidate.predict_effort` の惰行レジーム）で使う。段1では未参照。
    coast_band_kmhs: float = 0.0
    # 段3 到達可能性判定に使う先読みホライズン [s]。空タプル＝段3 無効（従来どおり
    # regime_horizon 1 点だけで判定する）。モデルの lookahead_horizons_s の部分集合であること
    reach_horizons_s: tuple[float, ...] = ()
    # v_free(t+h) の数値積分の刻み [s]
    reach_step_s: float = 0.05
    # ── 停止ブレーキ境界（段4改訂。ProblemReport_20260916） ─────────────────────
    # クリープ域でブレーキを保持して実際に停車できた最小の「不感帯からの超過」[%]。
    # tests.research.stop_brake_floor.estimate_stop_brake_floor が手順2 で同定する。0.0＝未同定
    stop_brake_floor_offset_pct: float = 0.0
    # この車速以下でブレーキの下限を効かせる [km/h]。0.0 で無効（人が決める値・自動保存の対象外）
    brake_trim_max_kmh: float = 0.0
    # 先読み車速（最短ホライズン=0.5s 先の基準）がこの値以下のときだけ下限を掛ける [km/h]
    # （人が決める値・自動保存の対象外）
    brake_trim_ref_kmh: float = 0.3


def creep_accel_at(research: ResearchFFParams, v_kmh: float) -> float | None:
    """速度 v でのクリープ加速度（正値）[km/h/s]。未同定（有効点2未満）なら None を返す。

    線形補間・端点クランプの流儀は `src.models.profile.coast_decel_at` と同じ
    （共通実装 `_interp_curve` を再利用する）。
    """
    return _interp_curve(research.creep_accel_speeds_kmh, research.creep_accel_kmhs, v_kmh)


def free_accel_at(params: FeedforwardParams, research: ResearchFFParams, v_kmh: float) -> float:
    """今ペダルを離したときの加速度 [km/h/s]（正=加速、負=減速）の単一ソース。

    v < params.creep_speed_kmh はクリープ域として +creep_accel_at(v)（カーブ未同定なら
    +params.creep_rate_kmhs にフォールバック）、それ以上は惰行域として -coast_decel_at(v)。
    """
    if v_kmh < params.creep_speed_kmh:
        accel = creep_accel_at(research, v_kmh)
        return accel if accel is not None else params.creep_rate_kmhs
    return -coast_decel_at(params, v_kmh)


def research_ff_params(cfg: ResearchConfig) -> ResearchFFParams:
    """feedforward セクション → ResearchFFParams（`vehicle.feedforward_params` と同じ流儀）。"""
    ff = cfg.feedforward
    return ResearchFFParams(
        creep_accel_speeds_kmh=tuple(ff.creep_accel_speeds_kmh),
        creep_accel_kmhs=tuple(ff.creep_accel_kmhs),
        coast_band_kmhs=ff.coast_band_kmhs,
        reach_horizons_s=tuple(ff.reach_horizons_s),
        reach_step_s=ff.reach_step_s,
        stop_brake_floor_offset_pct=ff.stop_brake_floor_offset_pct,
        brake_trim_max_kmh=ff.brake_trim_max_kmh,
        brake_trim_ref_kmh=ff.brake_trim_ref_kmh,
    )


__all__ = [
    "ResearchFFParams",
    "creep_accel_at",
    "free_accel_at",
    "research_ff_params",
]
