"""プライマリー KPI の計算（手順 3 以降のモード走行・手順 4/6/8 の適合で使う）。

定義は docs/product-requirements.md のプライマリー KPI と本番 src/domain/control/kpi_monitor.py に
合わせ、しきい値は config_testVehicle.yaml の kpi セクションを使う。

    最大逸脱   … |実車速 − 基準車速| の最大（例外なし）
    p95        … |実車速 − 基準車速| の 95 パーセンタイル
    符号反転   … 偏差が ±reversal_band_kmh を両側で超えて入れ替わった回数（帯の中は直前の
                 符号を保持）。任意の reversal_window_s 窓での最大回数

本番 KPIMonitor との違い:
    - 走行中に逐次集計するのではなく、記録した 0.1s 刻みの行（CSV と同じ行）からまとめて計算する。
      CSV から計算し直しても同じ値になるようにするため。
    - p95 は numpy の線形補間パーセンタイル（本番は 0.01 km/h ビンの上端で、最大 +0.01 保守側）。
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from tests.research.config import KpiSection


@dataclass(frozen=True)
class DeviationEpisode:
    """|偏差| が最大逸脱のしきい値を連続で超えた 1 区間。"""

    start_s: float
    end_s: float  # 最後に超えていた行の時刻
    peak_kmh: float  # 符号付き（+: 実車速が速い / −: 遅い）
    peak_t_s: float

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


@dataclass(frozen=True)
class KpiResult:
    n_samples: int
    max_abs_kmh: float
    max_abs_t_s: float
    p95_kmh: float
    reversal_max_per_window: int
    reversal_max_t_s: float | None  # 最大回数に達した時刻（反転が 0 回なら None）
    time_over_limit_s: float  # |偏差| > 最大逸脱しきい値 だった時間
    episodes: tuple[DeviationEpisode, ...]
    limits: KpiSection

    @property
    def max_ok(self) -> bool:
        return self.max_abs_kmh <= self.limits.max_abs_deviation_kmh

    @property
    def p95_ok(self) -> bool:
        return self.p95_kmh <= self.limits.p95_deviation_kmh

    @property
    def reversal_ok(self) -> bool:
        return self.reversal_max_per_window <= self.limits.reversal_limit_per_window

    @property
    def passed(self) -> bool:
        return self.n_samples > 0 and self.max_ok and self.p95_ok and self.reversal_ok

    @property
    def passed_count(self) -> int:
        return sum((self.max_ok, self.p95_ok, self.reversal_ok))


def sample_interval_s(t_s: Sequence[float]) -> float:
    """行の時間間隔の代表値（中央値）。1 行以下なら 0.1s とみなす。"""
    if len(t_s) < 2:
        return 0.1
    return float(np.median(np.diff(np.asarray(t_s, dtype=float))))


def reversal_max(
    t_s: Sequence[float], deviation: Sequence[float], *, band_kmh: float, window_s: float
) -> tuple[int, float | None]:
    """任意の window_s 窓での符号反転の最大回数と、その回数に達した時刻を返す。

    数え方は本番 KPIMonitor と同じ。
    """
    last_sign = 0
    times: deque[float] = deque()
    best = 0
    best_t: float | None = None
    for t, dev in zip(t_s, deviation, strict=True):
        sign = 1 if dev > band_kmh else -1 if dev < -band_kmh else 0
        if sign == 0:
            continue
        if last_sign != 0 and sign != last_sign:
            times.append(t)
            while times and t - times[0] > window_s:
                times.popleft()
            if len(times) > best:
                best, best_t = len(times), t
        last_sign = sign
    return best, best_t


def find_episodes(
    t_s: Sequence[float], deviation: Sequence[float], *, limit_kmh: float
) -> list[DeviationEpisode]:
    """|偏差| > limit_kmh が連続した区間を時刻順に返す。"""
    episodes: list[DeviationEpisode] = []
    start: int | None = None
    for i, dev in enumerate([*deviation, 0.0]):  # 番兵で最後の区間を閉じる
        if abs(dev) > limit_kmh:
            if start is None:
                start = i
            continue
        if start is None:
            continue
        seg = range(start, i)
        peak = max(seg, key=lambda j: abs(deviation[j]))
        episodes.append(
            DeviationEpisode(
                start_s=t_s[start],
                end_s=t_s[i - 1],
                peak_kmh=deviation[peak],
                peak_t_s=t_s[peak],
            )
        )
        start = None
    return episodes


def compute_kpi(t_s: Sequence[float], deviation: Sequence[float], limits: KpiSection) -> KpiResult:
    """時刻 [s] と偏差（実車速 − 基準車速）[km/h] の列からプライマリー KPI を計算する。"""
    if len(t_s) != len(deviation):
        raise ValueError("t_s と deviation の長さが一致しません")
    if not deviation:
        return KpiResult(0, 0.0, 0.0, 0.0, 0, None, 0.0, (), limits)
    abs_dev = np.abs(np.asarray(deviation, dtype=float))
    i_max = int(np.argmax(abs_dev))
    reversals, reversal_t = reversal_max(
        t_s, deviation, band_kmh=limits.reversal_band_kmh, window_s=limits.reversal_window_s
    )
    over = int(np.count_nonzero(abs_dev > limits.max_abs_deviation_kmh))
    return KpiResult(
        n_samples=len(deviation),
        max_abs_kmh=float(abs_dev[i_max]),
        max_abs_t_s=float(t_s[i_max]),
        p95_kmh=float(np.percentile(abs_dev, 95)),
        reversal_max_per_window=reversals,
        reversal_max_t_s=reversal_t,
        time_over_limit_s=over * sample_interval_s(t_s),
        episodes=tuple(
            find_episodes(t_s, deviation, limit_kmh=limits.max_abs_deviation_kmh)
        ),
        limits=limits,
    )
