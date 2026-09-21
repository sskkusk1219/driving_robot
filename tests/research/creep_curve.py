"""手順 2-2 クリープ加速カーブの推定（ProblemReport_20260916 課題#2）。

`src.domain.model_training.estimate_dynamics_params` の `creep_rate_kmhs`（定数1個）は「ペダルオフ・
低速・加速中」の**全サンプルの中央値**を取る。手順2はクリープ安定まで待つ
（`pedal_search.creep_settle_kmhs` / `creep_timeout_s`）ため母集団が定常側（dv≒0）に偏り、実測
（中央値 2.39 km/h/s、範囲 1.96〜3.44）の 1/12（0.19）になっていた。さらにクリープ加速は車速の
関数なのに定数1個で表している（惰行側は既に `coast_decel_speeds_kmh`/`coast_decel_kmhs` の
カーブになっている）。

新設のクリープ発進パターン（両ペダル 0% で自走、`pattern_loop.CreepLaunchPattern`）を足すことで、
定常待ちを挟まない加速中サンプルを狙って採る。ビン化は `src.domain.model_training.
_estimate_coast_decel_curve` と同形（速度ビンごとの中央値、最少サンプル数未満のビンは捨てる、
有効ビン2未満は未同定）だが、低速域なのでビン幅は `bin_kmh`（learning.creep_curve_bin_kmh、
既定 1.0 km/h。惰行カーブの 10 km/h 幅では 0〜5 km/h の域が 1 ビンに潰れる）。

2026-09-18（段2.5。ProblemReport_20260916）: 同定できたときは結果の**末尾**に必ず
`(params.creep_speed_kmh, 0.0)` を足す。理由は `estimate_creep_accel_curve` の実装コメント、
惰行側の接続点は `coast_curve.estimate_coast_decel_curve` を参照。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from src.domain.model_training import STOP_SPEED_KMH, _group_by_session
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research.pedal_gain import _sample_interval_s


@dataclass(frozen=True)
class CreepAccelCurve:
    speeds_kmh: tuple[float, ...]
    accel_kmhs: tuple[float, ...]
    samples: int  # 条件を満たしたサンプル数（ビン化前）

    @property
    def identified(self) -> bool:
        return bool(self.speeds_kmh)


def estimate_creep_accel_curve(
    logs: list[DriveLog],
    params: FeedforwardParams,
    *,
    bin_kmh: float,
    min_bin_samples: int,
) -> CreepAccelCurve:
    """走行ログからクリープ加速カーブ（0〜creep_speed_kmh の a_creep(v)、正値）を推定する。

    サンプル条件: 両ペダルが不感帯以下（accel<=accel_deadband_pct・brake<=brake_deadband_pct）
    かつ STOP_SPEED_KMH < v < params.creep_speed_kmh かつ dv > 0。段階解放中の行（本番
    `src/domain/learning_drive.py` の CREEP_RELEASE_STEPS）は両ペダル不感帯以下という条件を
    満たさないため自然に除外される。単調性の強制はしない（実測をそのまま出す）。
    """
    speeds: list[float] = []
    accels: list[float] = []
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
        mask = pedal_off & (sp > STOP_SPEED_KMH) & (sp < params.creep_speed_kmh) & (dv > 0.0)
        if not np.any(mask):
            continue
        speeds.extend(sp[mask].tolist())
        accels.extend(dv[mask].tolist())
    curve_speeds, curve_accels = _bin_median(
        np.array(speeds), np.array(accels), bin_kmh=bin_kmh, min_bin_samples=min_bin_samples
    )
    if curve_speeds:
        # クリープ平衡速度（creep_speed_kmh）はクリープだけで到達して静止する速度＝惰行加速度が
        # 0 になる速度の定義そのもの。現行は最終ビンの端点クランプ（例: 4.5km/h→+0.10）で
        # creep_speed_kmh（4.793）まで押し出すため、free_accel_at は 4.79 で +0.10 → 4.80 で
        # （惰行側）-2.235 と 1 ステップで飛んでいた（ProblemReport_20260916 段2.5）。末尾に
        # (creep_speed_kmh, 0.0) を足せば、惰行カーブ側の先頭点（coast_curve.
        # estimate_coast_decel_curve が同じ値を置く）と共有され、free_accel_at は特別扱いの
        # コード無しに構造として連続になる。
        curve_speeds = (*curve_speeds, params.creep_speed_kmh)
        curve_accels = (*curve_accels, 0.0)
    return CreepAccelCurve(speeds_kmh=curve_speeds, accel_kmhs=curve_accels, samples=len(speeds))


def _bin_median(
    speeds: np.ndarray,
    values: np.ndarray,
    *,
    bin_kmh: float,
    min_bin_samples: int,
    start_kmh: float = 0.0,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """`src.domain.model_training._estimate_coast_decel_curve` と同形のビン化（ビン幅は可変）。

    `start_kmh` はビン境界の起点（既定 0.0 は本番・従来のクリープ加速カーブと同じ）。
    `coast_curve.estimate_coast_decel_curve` が低速側のビンを `creep_speed_kmh` 起点で
    切るために再利用する（ビン化の実装を2本に増やさない）。
    有効ビンが 2 個未満なら空タプル（未同定＝呼び出し元が既存値を据え置く）。
    """
    if len(speeds) == 0:
        return (), ()
    v_max = float(speeds.max())
    span = v_max - start_kmh
    if span < 0.0:
        return (), ()
    n_bins = int(span / bin_kmh) + 1
    out_speeds: list[float] = []
    out_values: list[float] = []
    for i in range(n_bins):
        lo = start_kmh + i * bin_kmh
        hi = lo + bin_kmh
        mask = (speeds >= lo) & (speeds < hi)
        if int(np.count_nonzero(mask)) < min_bin_samples:
            continue
        out_speeds.append(lo + bin_kmh / 2.0)
        out_values.append(float(np.median(values[mask])))
    if len(out_speeds) < 2:
        return (), ()
    return tuple(out_speeds), tuple(out_values)


__all__ = ["CreepAccelCurve", "estimate_creep_accel_curve"]
