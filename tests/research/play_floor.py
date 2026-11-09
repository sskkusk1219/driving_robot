"""車速ごとのペダルの遊びの端（下限）と、C10 の行にその下限を掛ける規則。

2026-11-08 ProblemReport_20261108_アクセルを効き始めで待つ 段1。

遊び ＝ 待機位置 min から踏み込んでも加速度が惰行と変わらない範囲。効き始め ＝ 遊びの端。
遊びの中では min でも効き始めの手前でも減速は同じなので、C10 の P が遊びを飛び越えて min まで
戻してもペダルが往復するだけで減速は増えない。そこで C10 の行の開度を「効き始め − 余白」より
浅くしない。

効き始めは表から車速ごとに引く: 「|加速度(v, u) − 加速度(v, 0)| が rise_kmhs を超える最小の |u|」
を u の格子で探し（格子の間は直線で補間）、アクセルは u > 0・ブレーキは u < 0 の側で探す。
求まらないとき（表が最後まで立ち上がらない）は pkl の meta の accel_onset_u_pct・
brake_onset_u_pct へ戻す。それも無ければ下限なし（None）。

純粋なロジックだけ（表は `accel(v, u, launch=, stop=)` と `u_grid` を持つものならよい）。
mode_drive が走行中に引く。src/ は import しない。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

# 車速のキャッシュの刻み [km/h]。下限は車速にゆるやかにしか依存しないので、この刻みの中心で引く
SPEED_BIN_KMH = 0.5


def onset_u_pct(
    table: Any,
    v_kmh: float,
    *,
    is_accel: bool,
    rise_kmhs: float,
    launch: bool = False,
    stop: bool = False,
) -> float | None:
    """車速 v での効き始め（min からの量 [%]、正）。rise_kmhs を超えるほど立ち上がらなければ None。

    アクセルは u > 0 の格子点を小さい方から、ブレーキは u < 0 の格子点を 0 に近い方から調べ、
    惰行（u = 0）の加速度との差の大きさが最初に rise_kmhs を超える区間を直線で補間して u を出す。
    """
    ug = np.asarray(table.u_grid, dtype=float)
    pts = np.sort(ug[ug > 0.0]) if is_accel else np.sort(-ug[ug < 0.0])
    if pts.size == 0:
        return None
    sign = 1.0 if is_accel else -1.0
    a0 = float(table.accel(v_kmh, 0.0, launch=launch, stop=stop))
    prev_u, prev_d = 0.0, 0.0
    for u in pts:
        a = float(table.accel(v_kmh, sign * float(u), launch=launch, stop=stop))
        d = abs(a - a0)
        if d > rise_kmhs:
            if d == prev_d:  # 平らな区間には来ない（d > rise ≥ prev_d）が、念のため
                return float(u)
            return float(prev_u + (rise_kmhs - prev_d) / (d - prev_d) * (float(u) - prev_u))
        prev_u, prev_d = float(u), d
    return None


def play_floor_pct(
    table: Any,
    v_kmh: float,
    *,
    is_accel: bool,
    rise_kmhs: float,
    margin_pct: float,
    min_pct: float,
    fallback_u_pct: float | None = None,
    launch: bool = False,
    stop: bool = False,
) -> float | None:
    """車速 v でのそのペダルの下限の開度の大きさ [%] ＝ min ＋ max(効き始め − margin, 0)。

    効き始めは `onset_u_pct`。求まらなければ fallback_u_pct（pkl の効き始め）、それも無ければ None
    （下限なし）。返す値は正の大きさ（符号をつけるのは呼ぶ側。ブレーキは負の effort にする）。
    """
    u_on = onset_u_pct(
        table, v_kmh, is_accel=is_accel, rise_kmhs=rise_kmhs, launch=launch, stop=stop
    )
    if u_on is None:
        if fallback_u_pct is None or not math.isfinite(float(fallback_u_pct)):
            return None
        u_on = float(fallback_u_pct)
    return float(min_pct + max(u_on - margin_pct, 0.0))


def apply_floor(
    base_effort: float, delta: float, floor_mag: float | None, *, force: bool = False
) -> tuple[float, bool]:
    """C10 の行の開度に下限を掛ける。(新しい合計の effort, 下限で止めたか) を返す。

    base_effort ＝ FF の effort（正＝アクセル・負＝ブレーキ）、delta ＝ C10 の上乗せ、
    floor_mag ＝ 下限の開度（大きさ。符号つきでも可。None なら掛けない）。
    合計 raw ＝ base ＋ delta の大きさを max(|raw|, floor) にする。ただし FF そのものが下限より
    浅い行（|base| < floor）は触らない（計画の判断は変えない）。force（惰行の補助の handover の
    行）は FF が計画の列ではなく表から引いた値なので、FF の深さに関係なく下限を掛ける。
    """
    raw = base_effort + delta
    if floor_mag is None or base_effort == 0.0:
        return raw, False
    floor_mag = abs(floor_mag)  # 符号つき（ブレーキ負）の下限もそのまま渡してよい
    if not force and abs(base_effort) < floor_mag:
        return raw, False
    if abs(raw) >= floor_mag:
        return raw, False
    return math.copysign(floor_mag, base_effort), True


@dataclass
class PlayFloorStats:
    """下限で止めた周期の集計（ペダル別）。"""

    period_s: float = 0.05
    accel_floored: int = 0  # 下限で止めた周期数
    brake_floored: int = 0
    accel_raise_pct: float = 0.0  # 持ち上げた量（下限 − 下限がなければの開度）の合計
    brake_raise_pct: float = 0.0
    accel_rows: int = 0  # 下限を掛ける行だった周期数（止めなかった周期も含む）
    brake_rows: int = 0
    brake_skipped: int = 0  # ブレーキで除外した行の周期数（7.5 km/h 未満・保持・帯・踏み増し・停C）

    def add(self, *, is_accel: bool, floored: bool, raise_pct: float = 0.0) -> None:
        if is_accel:
            self.accel_rows += 1
            if floored:
                self.accel_floored += 1
                self.accel_raise_pct += raise_pct
        else:
            self.brake_rows += 1
            if floored:
                self.brake_floored += 1
                self.brake_raise_pct += raise_pct

    def summary(self) -> str:
        p = self.period_s
        return (
            f"下限で止めた アクセル {self.accel_floored} 周期（{self.accel_floored * p:.1f}s・"
            f"持ち上げた量の合計 {self.accel_raise_pct:.1f}%）／"
            f"ブレーキ {self.brake_floored} 周期（{self.brake_floored * p:.1f}s・"
            f"持ち上げた量の合計 {self.brake_raise_pct:.1f}%）・"
            f"掛けた行 アクセル {self.accel_rows * p:.1f}s／ブレーキ {self.brake_rows * p:.1f}s・"
            f"ブレーキで除外した行 {self.brake_skipped * p:.1f}s"
        )


class PlayFloor:
    """車速ごとの下限を引く（車速の刻みでキャッシュ）。1 周期 50 ms の中で安く引くため。"""

    def __init__(
        self,
        table: Any,
        *,
        rise_kmhs: float,
        margin_pct: float,
        accel_min_pct: float,
        brake_min_pct: float,
        period_s: float = 0.05,
    ) -> None:
        self.table = table
        self.rise_kmhs = rise_kmhs
        self.margin_pct = margin_pct
        self.accel_min_pct = accel_min_pct
        self.brake_min_pct = brake_min_pct
        self.stats = PlayFloorStats(period_s=period_s)
        self._cache: dict[tuple[int, bool, bool, bool], float | None] = {}
        self._cache0: dict[tuple[int, bool, bool, bool], float | None] = {}  # 余白 0（効き始め）

    def _fallback(self, is_accel: bool) -> float | None:
        meta = getattr(self.table, "meta", {}) or {}
        x = meta.get("accel_onset_u_pct" if is_accel else "brake_onset_u_pct")
        return None if x is None or not math.isfinite(float(x)) else float(x)

    def floor_mag(
        self, v_kmh: float, *, is_accel: bool, launch: bool = False, stop: bool = False
    ) -> float | None:
        """下限の開度の大きさ [%]（None ＝ 掛けない）。"""
        key = (int(math.floor(max(v_kmh, 0.0) / SPEED_BIN_KMH)), is_accel, launch, stop)
        if key not in self._cache:
            v_c = (key[0] + 0.5) * SPEED_BIN_KMH
            self._cache[key] = play_floor_pct(
                self.table, v_c, is_accel=is_accel, rise_kmhs=self.rise_kmhs,
                margin_pct=self.margin_pct,
                min_pct=self.accel_min_pct if is_accel else self.brake_min_pct,
                fallback_u_pct=self._fallback(is_accel), launch=launch, stop=stop,
            )
        return self._cache[key]

    def onset_mag(
        self, v_kmh: float, *, is_accel: bool, launch: bool = False, stop: bool = False
    ) -> float | None:
        """効き始めの開度の大きさ [%] ＝ min ＋ 効き始め（余白を引かない。None ＝ 求まらない）。"""
        key = (int(math.floor(max(v_kmh, 0.0) / SPEED_BIN_KMH)), is_accel, launch, stop)
        if key not in self._cache0:
            v_c = (key[0] + 0.5) * SPEED_BIN_KMH
            self._cache0[key] = play_floor_pct(
                self.table, v_c, is_accel=is_accel, rise_kmhs=self.rise_kmhs, margin_pct=0.0,
                min_pct=self.accel_min_pct if is_accel else self.brake_min_pct,
                fallback_u_pct=self._fallback(is_accel), launch=launch, stop=stop,
            )
        return self._cache0[key]

    def floor_effort(
        self, v_kmh: float, *, is_accel: bool, launch: bool = False, stop: bool = False
    ) -> float | None:
        """下限の effort（アクセル正・ブレーキ負。None ＝ 掛けない）。"""
        m = self.floor_mag(v_kmh, is_accel=is_accel, launch=launch, stop=stop)
        return None if m is None else (m if is_accel else -m)
