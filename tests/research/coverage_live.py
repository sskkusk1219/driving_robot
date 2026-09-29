"""走行中に数える網羅カウンタ（ProblemReport_20260925 段6c）。

`wltp_grid.data_cells`（走行後に CSV から数える）と同じ定義で、PatternLoop の各周期の
(時刻, 車速, 開度) から、車速 × 加速度の格子の各セルに学習データが何秒あるかを数える:

- 行の v0 = その時刻の車速、a_req = 1.0 s 先の車速との差（regime ホライズン 1.0 s と同じ）。
  1.0 s 先が来るまで確定しない（1 s 遅れて数える）
- 停車（v0 < STOP_SPEED_KMH）は数えない
- 1 行の重みは、その行から次の行までの時間（周期）

`wltp_grid` は import しない（wltp_grid → mode_drive → pattern_drive → pattern_loop の循環を
避けるため）。格子の境界・停車の判定は同じ値を持ち、事後の網羅表と一致することはテストで確認している。
"""

from __future__ import annotations

from collections import deque

import numpy as np

from tests.research.ff_model import STOP_SPEED_KMH

REGIME_HORIZON_S = 1.0  # ff_model.FeatureSpec.regime_horizon_s と同じ（加速度の定義の窓）
_EPS_S = 1e-6  # 時刻の浮動小数の丸め（0.1 s 刻みで 1.0 s 先を取りこぼさない）


class LiveCoverage:
    """車速 × 加速度の格子ごとの学習データの秒数（走行中に加算）。"""

    def __init__(self, speed_edges_kmh: list[float], accel_edges_kmhs: list[float]) -> None:
        self._sp = np.asarray(speed_edges_kmh, dtype=float)
        self._ac = np.asarray(accel_edges_kmhs, dtype=float)
        self.seconds = np.zeros((len(self._sp) - 1, len(self._ac) - 1))
        self._rows: deque[tuple[float, float]] = deque()  # (時刻, 車速)。確定前の行

    @property
    def shape(self) -> tuple[int, int]:
        return self.seconds.shape

    def push(self, t: float, speed_kmh: float) -> None:
        """1 周期分の (時刻, 車速)。1.0 s 先が揃った行から順に確定してセルへ加算する。"""
        rows = self._rows
        rows.append((t, max(speed_kmh, 0.0)))
        while len(rows) >= 2 and rows[-1][0] - rows[0][0] >= REGIME_HORIZON_S - _EPS_S:
            t0, v0 = rows[0]
            # 1.0 s 先の行（t0 + 1.0 s 以上で最初の行）
            future = next((v for tt, v in rows if tt >= t0 + REGIME_HORIZON_S - _EPS_S), None)
            if future is None:
                break
            dt = rows[1][0] - t0
            self._add(v0, (future - v0) / REGIME_HORIZON_S, dt)
            rows.popleft()

    def _add(self, v0: float, a_req: float, dt_s: float) -> None:
        if v0 < STOP_SPEED_KMH:
            return
        i = int(np.searchsorted(self._sp, v0, side="right")) - 1
        j = int(np.searchsorted(self._ac, a_req, side="right")) - 1
        if i == len(self._sp) - 1 and v0 == self._sp[-1]:
            i -= 1  # np.histogram2d と同じ: 最後の境界は最後のビンに入れる
        if j == len(self._ac) - 1 and a_req == self._ac[-1]:
            j -= 1
        if 0 <= i < self.seconds.shape[0] and 0 <= j < self.seconds.shape[1]:
            self.seconds[i, j] += dt_s

    def holes(
        self, wltp_seconds: np.ndarray, wltp_min_s: float, data_max_s: float
    ) -> list[tuple[int, int]]:
        """(車速ビン, 加速度ビン)。モードが `wltp_min_s` 以上要るのにデータが `data_max_s` 未満。

        `wltp_grid.find_holes` と同じ判定。需要の大きい順。
        """
        cells = [
            (i, j)
            for i in range(wltp_seconds.shape[0])
            for j in range(wltp_seconds.shape[1])
            if wltp_seconds[i, j] >= wltp_min_s and self.seconds[i, j] < data_max_s
        ]
        return sorted(cells, key=lambda c: -wltp_seconds[c])


__all__ = ["LiveCoverage"]
