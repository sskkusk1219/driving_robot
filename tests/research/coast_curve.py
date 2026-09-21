"""手順 2-2 惰行減速カーブの低速端を実測に合わせる（段2.5。ProblemReport_20260916）。

段2 までの実機結果で、9/18 の逸脱の主因が「基準 0〜20 km/h 帯」に集中していることが分かった。
`src.domain.model_training._estimate_coast_decel_curve` の `COAST_CURVE_BIN_KMH = 10.0` は
0〜10 km/h を 1 ビンにまとめ、その中央値をビン中心 5.0 km/h に置く。実測を 1 km/h 幅で割り直すと
5〜10 km/h の惰行減速量は 1.74→3.15 km/h/s と滑らかに変化しており、1 本の値（2.235）では
表現できない。

さらに悪いことに、クリープ平衡速度（`creep_speed_kmh`。クリープだけで到達して静止する速度＝
惰行加速度が 0 になる速度）とこの惰行カーブの間には接続点が無く、`ff_params.free_accel_at` は
`v=4.79 → +0.10`（クリープカーブの端点クランプ）から `v=4.80 → -2.235`（惰行カーブの端点クランプ）
へ 1 ステップで 2.3 km/h/s も飛んでいた。実機では 5.29 km/h で惰行に入っても実測は 0.85 秒
5.00 km/h に張り付き（証拠は ProblemReport_20260916 段2.5 の Context 参照）、この土台のずれが
段3（到達可能性の先読み）・段4（ブレーキトリム）双方の前提を壊す。

この推定器は本番 `estimate_dynamics_params` の惰行サンプル条件（`m_eng`: 両ペダル不感帯以下・
`sp > creep_speed_kmh`・`dv < 0`）をそのまま使い、ビン化だけを変える:
    低速側（creep_speed_kmh 〜 low_max_kmh）… `low_bin_kmh` 幅の細ビン
    高速側（low_max_kmh 以上）            … 本番と同じ `COAST_CURVE_BIN_KMH`（10.0）幅

ビン化そのものは `creep_curve._bin_median`（`start_kmh` 引数を追加して再利用。ビン化の実装を
2本に増やさない）と同形: サンプル不足のビンは捨て、有効ビン（低速＋高速の合計）が2個未満なら
未同定（既存カーブへフォールバック）。

**先頭に必ず `(creep_speed_kmh, 0.0)` を置く**。クリープ平衡速度の定義そのもの（クリープだけで
到達して静止する速度＝惰行加速度 0 の速度）であり、これが `creep_curve.estimate_creep_accel_curve`
の末尾点（同じく `(creep_speed_kmh, 0.0)`）と一致することで、`free_accel_at` は特別扱いの
コード無しに構造として連続になる。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.domain.model_training import (
    COAST_CURVE_BIN_KMH,
    COAST_CURVE_MIN_BIN_SAMPLES,
    _group_by_session,
)
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research.creep_curve import _bin_median
from tests.research.pedal_gain import _sample_interval_s


@dataclass(frozen=True)
class CoastDecelCurve:
    speeds_kmh: tuple[float, ...]
    decels_kmh: tuple[float, ...]  # 各速度での惰行減速量（正値）[km/h/s]
    samples: int  # 条件を満たしたサンプル数（ビン化前）

    @property
    def identified(self) -> bool:
        return bool(self.speeds_kmh)


def estimate_coast_decel_curve(
    logs: list[DriveLog],
    params: FeedforwardParams,
    *,
    low_bin_kmh: float,
    low_max_kmh: float,
    low_min_bin_samples: int,
) -> CoastDecelCurve:
    """走行ログから惰行減速カーブ（creep_speed_kmh 以上の -dv(v)、正値）を推定する。

    低速端（creep_speed_kmh〜low_max_kmh）は細ビンで推定する。

    サンプル条件は本番 `estimate_dynamics_params` の `m_eng` と同じ: 両ペダルが不感帯以下
    （accel<=accel_deadband_pct・brake<=brake_deadband_pct）かつ `sp > params.creep_speed_kmh`
    かつ `dv < 0`。低速側（creep_speed_kmh 〜 low_max_kmh）は `low_bin_kmh` 幅、高速側
    （low_max_kmh 以上）は本番と同じ `COAST_CURVE_BIN_KMH` 幅・`COAST_CURVE_MIN_BIN_SAMPLES`
    点以上で、ビン中心が `low_max_kmh` 以上のものだけを高速側として採る（低速側と重複させない）。
    単調性の強制はしない（実測をそのまま出す）。
    """
    speeds: list[float] = []
    decels: list[float] = []
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
        pedal_off = (accel[:-1] <= params.accel_deadband_pct) & (
            brake[:-1] <= params.brake_deadband_pct
        )
        mask = pedal_off & (sp > params.creep_speed_kmh) & (dv < 0.0)
        if not np.any(mask):
            continue
        speeds.extend(sp[mask].tolist())
        decels.extend((-dv[mask]).tolist())

    speeds_arr = np.array(speeds)
    decels_arr = np.array(decels)

    # 低速側: creep_speed_kmh を起点にした細ビン。low_max_kmh 未満のサンプルだけを渡し、
    # 万一 bin_kmh の半端でビン中心が low_max_kmh に達しても後段の filter で高速側と重複させない
    is_low = speeds_arr < low_max_kmh
    low_speeds_all, low_decels_all = _bin_median(
        speeds_arr[is_low], decels_arr[is_low],
        bin_kmh=low_bin_kmh, min_bin_samples=low_min_bin_samples,
        start_kmh=params.creep_speed_kmh,
    )
    low_bins = [
        (v, d) for v, d in zip(low_speeds_all, low_decels_all, strict=True) if v < low_max_kmh
    ]

    # 高速側: 本番と同じビン境界（0 起点・COAST_CURVE_BIN_KMH 幅）。低速側と重複させないため
    # ビン中心が low_max_kmh 以上のものだけを採る
    high_speeds_all, high_decels_all = _bin_median(
        speeds_arr, decels_arr,
        bin_kmh=COAST_CURVE_BIN_KMH, min_bin_samples=COAST_CURVE_MIN_BIN_SAMPLES,
    )
    high_bins = [
        (v, d) for v, d in zip(high_speeds_all, high_decels_all, strict=True) if v >= low_max_kmh
    ]

    bins = low_bins + high_bins
    if len(bins) < 2:
        return CoastDecelCurve(speeds_kmh=(), decels_kmh=(), samples=len(speeds))

    # 先頭にクリープ平衡点を置く（モジュール docstring 参照。creep_curve.estimate_creep_accel_curve
    # の末尾点と一致させ、free_accel_at を構造として連続にする）
    combined = [(params.creep_speed_kmh, 0.0), *bins]
    out_speeds, out_decels = zip(*combined, strict=True)
    return CoastDecelCurve(speeds_kmh=out_speeds, decels_kmh=out_decels, samples=len(speeds))


__all__ = ["CoastDecelCurve", "estimate_coast_decel_curve"]
