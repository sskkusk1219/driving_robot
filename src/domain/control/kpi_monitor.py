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
# プライマリー KPI の上限値（この回数を超えたら違反）。窓の長さ(KPI_REVERSAL_WINDOW_S)とは
# 別物であり、コスト関数等で正規化に使う場合はこちらを使うこと（窓長で割ると単位が
# 秒になり「回数の上限で正規化」という意図と食い違う）。
KPI_REVERSAL_LIMIT_PER_WINDOW: float = 1.0
# 符号判定のノイズフロア。CAN 車速の量子化ノイズで偏差 ±0.0x km/h の符号が
# チャタつくのを反転としてカウントしないための不感帯。
SIGN_NOISE_FLOOR_KMH: float = 0.05

_BIN_WIDTH_KMH: float = 0.01
_MAX_BIN_KMH: float = 10.0
# 超過積分の dt クランプ上限 [s]。pause 復帰やサイクルスキップの大ギャップ対策。
_DT_CLAMP_S: float = 0.5
# ペダル「踏んでいる」とみなすアクセル開度のしきい値 [%]（ON-OFF 立ち上がりカウント用）。
# scripts/analyze_session.py のハンチング解析と揃える。
_PEDAL_ON_THRESHOLD_PCT: float = 0.5
# アクセル⇔ブレーキの交互踏み（シーソー）とみなす切替の時間窓 [s]。前にアクティブだった
# ペダルと異なるペダルがこの秒数以内に踏まれたら「不要切替」の候補としてカウントする。
_PEDAL_SWITCH_WINDOW_S: float = 2.0


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
        # ハード上限超過の連続量。回数(_hard_violations)の不連続ペナルティでは座標降下が
        # 局所解で停滞するため、tuning_cost へ勾配を与える「超過量×時間」の積分を保持する。
        self._over_limit_integral_kmhs: float = 0.0
        self._time_over_limit_s: float = 0.0
        self._last_now_s: float | None = None
        self._first_now_s: float | None = None
        # ペダル活動度（ハンチング指標）。アクセルの 0→非0 立ち上がり回数と総移動量[%]。
        # tuning_cost が「滑らかな操作」を最適化対象に含めるために集計する（B-7-4）。
        self._accel_on_count: int = 0
        self._pedal_travel_pct: float = 0.0
        self._prev_accel_on: bool | None = None
        self._prev_accel_opening: float | None = None
        # アクセル⇔ブレーキ交互踏み（シーソー）回数。前にアクティブだったペダルと異なる
        # ペダルが _PEDAL_SWITCH_WINDOW_S 以内に踏まれた回数を数える（不要切替の実測。
        # 軌跡が要求する切替との差が「不要切替」＝滑らかさの逆指標）。
        self._pedal_switch_count: int = 0
        self._last_active_pedal: int = 0  # 1=accel, -1=brake, 0=どちらも踏んでいない
        self._last_active_pedal_s: float | None = None

    def update(
        self,
        ref_kmh: float,
        actual_kmh: float,
        now_s: float,
        accel_opening: float | None = None,
        brake_opening: float | None = None,
    ) -> None:
        """1 サイクル分の偏差（と任意でペダル開度）を集計する。

        Args:
            ref_kmh: 基準車速 [km/h]
            actual_kmh: 実車速 [km/h]
            now_s: 単調増加時刻 [s]（イベントループ時刻）
            accel_opening: 調停後のアクセル開度 [%]。渡すとペダル活動度（ON-OFF 立ち上がり
                回数・総移動量）を集計する。None なら偏差 KPI のみ（後方互換）。
            brake_opening: 調停後のブレーキ開度 [%]。accel_opening と両方渡すと
                アクセル⇔ブレーキ交互踏み（不要切替）を集計する。None なら集計しない。
        """
        if self._first_now_s is None:
            self._first_now_s = now_s
        deviation = actual_kmh - ref_kmh
        abs_dev = abs(deviation)

        self._n += 1
        if abs_dev > self._max_abs_deviation:
            self._max_abs_deviation = abs_dev
        bin_idx = min(int(abs_dev / _BIN_WIDTH_KMH), len(self._bins) - 1)
        self._bins[bin_idx] += 1

        # 超過量×時間の積分。dt は前回サンプルからの経過で、pause/サイクルスキップ由来の
        # 大ギャップが積分を膨らませないよう [0, _DT_CLAMP_S] にクランプする。
        if self._last_now_s is not None:
            dt = now_s - self._last_now_s
            if dt < 0.0:
                dt = 0.0
            elif dt > _DT_CLAMP_S:
                dt = _DT_CLAMP_S
            over = abs_dev - KPI_HARD_LIMIT_KMH
            if over > 0.0:
                self._over_limit_integral_kmhs += over * dt
                self._time_over_limit_s += dt
        self._last_now_s = now_s

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

        # ペダル活動度（ハンチング指標）。アクセルの 0→非0 立ち上がりと総移動量を集計する。
        if accel_opening is not None:
            on = accel_opening > _PEDAL_ON_THRESHOLD_PCT
            if self._prev_accel_on is not None and on and not self._prev_accel_on:
                self._accel_on_count += 1
            self._prev_accel_on = on
            if self._prev_accel_opening is not None:
                self._pedal_travel_pct += abs(accel_opening - self._prev_accel_opening)
            self._prev_accel_opening = accel_opening

            # アクセル⇔ブレーキ交互踏み（シーソー）。今アクティブなペダルが、直前にアクティブ
            # だった別ペダルと _PEDAL_SWITCH_WINDOW_S 以内で切り替わったらカウント。
            if brake_opening is not None:
                if accel_opening > _PEDAL_ON_THRESHOLD_PCT:
                    current_pedal = 1
                elif brake_opening > _PEDAL_ON_THRESHOLD_PCT:
                    current_pedal = -1
                else:
                    current_pedal = 0
                if current_pedal != 0:
                    if (
                        self._last_active_pedal != 0
                        and current_pedal != self._last_active_pedal
                        and self._last_active_pedal_s is not None
                        and now_s - self._last_active_pedal_s <= _PEDAL_SWITCH_WINDOW_S
                    ):
                        self._pedal_switch_count += 1
                    self._last_active_pedal = current_pedal
                    self._last_active_pedal_s = now_s

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
        duration_s = 0.0
        if self._first_now_s is not None and self._last_now_s is not None:
            duration_s = max(0.0, self._last_now_s - self._first_now_s)
        accel_on_per_min = (
            self._accel_on_count / (duration_s / 60.0) if duration_s > 0.0 else 0.0
        )
        pedal_switch_per_min = (
            self._pedal_switch_count / (duration_s / 60.0) if duration_s > 0.0 else 0.0
        )
        return {
            "n_samples": float(self._n),
            "max_abs_deviation_kmh": self._max_abs_deviation,
            "p95_kmh": self.p95_kmh(),
            "reversal_max_per_5s": float(self._reversal_max_per_window),
            "hard_limit_violations": float(self._hard_violations),
            "over_limit_integral_kmhs": self._over_limit_integral_kmhs,
            "time_over_limit_s": self._time_over_limit_s,
            "accel_on_count": float(self._accel_on_count),
            "accel_on_per_min": accel_on_per_min,
            "pedal_travel_pct": self._pedal_travel_pct,
            "pedal_switch_count": float(self._pedal_switch_count),
            "pedal_switch_per_min": pedal_switch_per_min,
        }
