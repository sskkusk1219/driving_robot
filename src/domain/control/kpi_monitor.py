"""プライマリー KPI（車速追従精度・振動抑制）の実行時計測モニタ。

product-requirements.md のプライマリー KPI:
- 速度偏差絶対値の 95 パーセンタイルが 0.2 km/h 以内
- 速度偏差絶対値が 1.0 km/h を超えない（例外なし）
- 速度偏差の符号反転が 5 秒に 1 回以下

走行終了後のオフライン解析（しかもログは 2 サイクルに 1 回へ間引かれる）では
KPI 違反が観測すらできないため、制御サイクル内で全サンプルを集計する
（コードレビュー 2026-06-11 指摘 #7）。

メモリは固定: ヒストグラムは固定長ビン、反転時刻は 5 秒窓 deque のみ保持するため、
10 時間 × 20Hz = 72 万サンプルでも増加しない。
"""

from __future__ import annotations

import logging
from collections import deque

_logger = logging.getLogger(__name__)

KPI_P95_LIMIT_KMH: float = 0.2
KPI_HARD_LIMIT_KMH: float = 1.0
# ハード違反の解除しきい値。境界での warning 連発を防ぐリリースヒステリシス。
_HARD_LIMIT_RELEASE_KMH: float = 0.9
KPI_REVERSAL_WINDOW_S: float = 5.0
# 符号判定のノイズフロア。CAN 車速の量子化ノイズで偏差 ±0.0x km/h の符号が
# チャタつくのを反転としてカウントしないための不感帯。
SIGN_NOISE_FLOOR_KMH: float = 0.05

_BIN_WIDTH_KMH: float = 0.01
_MAX_BIN_KMH: float = 10.0


class KPIMonitor:
    """速度偏差の P95・最大値・符号反転率を逐次集計するドメインコンポーネント。"""

    def __init__(self) -> None:
        n_bins = int(_MAX_BIN_KMH / _BIN_WIDTH_KMH) + 1  # 最終ビンはオーバーフロー
        self._bins: list[int] = [0] * n_bins
        self._n: int = 0
        self._max_abs_deviation: float = 0.0
        self._last_sign: int = 0
        self._reversal_times: deque[float] = deque()
        self._reversal_max_per_window: int = 0
        self._hard_violations: int = 0
        self._in_hard_violation: bool = False

    def update(self, ref_kmh: float, actual_kmh: float, now_s: float) -> None:
        """1 サイクル分の偏差を集計する。

        Args:
            ref_kmh: 基準車速 [km/h]
            actual_kmh: 実車速 [km/h]
            now_s: 単調増加時刻 [s]（イベントループ時刻）
        """
        deviation = actual_kmh - ref_kmh
        abs_dev = abs(deviation)

        self._n += 1
        if abs_dev > self._max_abs_deviation:
            self._max_abs_deviation = abs_dev
        bin_idx = min(int(abs_dev / _BIN_WIDTH_KMH), len(self._bins) - 1)
        self._bins[bin_idx] += 1

        # 符号反転（ノイズフロア超のみ）。任意 5 秒窓の最大反転回数を追跡する。
        if deviation > SIGN_NOISE_FLOOR_KMH:
            sign = 1
        elif deviation < -SIGN_NOISE_FLOOR_KMH:
            sign = -1
        else:
            sign = 0
        if sign != 0:
            if self._last_sign != 0 and sign != self._last_sign:
                self._reversal_times.append(now_s)
                while (
                    self._reversal_times and now_s - self._reversal_times[0] > KPI_REVERSAL_WINDOW_S
                ):
                    self._reversal_times.popleft()
                self._reversal_max_per_window = max(
                    self._reversal_max_per_window, len(self._reversal_times)
                )
            self._last_sign = sign

        # ハード上限（1.0 km/h、例外なし）の即時警告。違反継続中の連発はしない。
        if abs_dev > KPI_HARD_LIMIT_KMH and not self._in_hard_violation:
            self._in_hard_violation = True
            self._hard_violations += 1
            _logger.warning(
                "KPI ハード上限違反: |偏差| %.2f km/h > %.1f km/h (ref=%.1f actual=%.1f)",
                abs_dev,
                KPI_HARD_LIMIT_KMH,
                ref_kmh,
                actual_kmh,
            )
        elif abs_dev < _HARD_LIMIT_RELEASE_KMH:
            self._in_hard_violation = False

    def p95_kmh(self) -> float:
        """偏差絶対値の 95 パーセンタイル [km/h]（ビン上端値、保守側）を返す。"""
        if self._n == 0:
            return 0.0
        target = 0.95 * self._n
        cumulative = 0
        for i, count in enumerate(self._bins):
            cumulative += count
            if cumulative >= target:
                return (i + 1) * _BIN_WIDTH_KMH
        return _MAX_BIN_KMH

    def summary(self) -> dict[str, float]:
        """走行全体の KPI サマリを返す。走行終了時にログ・公開する。"""
        return {
            "n_samples": float(self._n),
            "max_abs_deviation_kmh": self._max_abs_deviation,
            "p95_kmh": self.p95_kmh(),
            "reversal_max_per_5s": float(self._reversal_max_per_window),
            "hard_limit_violations": float(self._hard_violations),
        }
