"""段8-a: 基準 0 で車が動き出したら、発進前のブレーキの離しを止めて待つ（2026-10-24）。

対象は計画の launch_prep の行（発進前の離しの行と、短い停車の bk の行。段7-b）。そこで実車速が
`MOVE_KMH` を超えたら:
 (ア) その周期の FF のブレーキで離しを止める（以降の行は止めた値）。
 (イ) 深くする向きだけの積分 I [%]（I += Ki × 車速 × dt。車速が MOVE_KMH 以下の周期は足さず、
      その値で待つ）。ブレーキの指令 ＝ 止めた FF ＋ I（深い側）。
 (ウ) Ki ＝ 1 ÷ (τ² × k)（`launch_hold_gain`）。
基準が動き出して launch_prep の行が終わったら、「止めた開度 ＋ I − bk」（≧ 0）を、発進の途中の
ブレーキの行に足し、計画のブレーキが min に戻ったらやめる。発進（launch_prep の連なり）ごとに
状態を戻す。アクセルの行・惰行の行には何もしない（ペダルの同時踏みを作らない）。

tests/research の中で完結する（src は import しない）。"""

from __future__ import annotations

from typing import Any

import numpy as np

MOVE_KMH = 0.05  # 動いたとみなす実車速の下限 [km/h]（ユーザー指定 2026-10-24）
_SLOPE_HALF_PCT = 0.5  # k を出す差分の幅の半分 [%]


def launch_hold_gain(
    table: Any, *, tau_brake_s: float | None
) -> tuple[float | None, float | None, str]:
    """(Ki, k, 説明)。働かないとき Ki は None（説明に理由）。

    k ＝ A2 なしの発進側の表の最も低い車速の行（v ≒ 0）で、動き出す開度の前後 ±0.5% の傾き
    [km/h/s ÷ %]（開度 u はブレーキが負。浅い側が +）。Ki ＝ 1 ÷ (τ² × k) [% ÷ (km/h·s)]。"""
    if tau_brake_s is None or not np.isfinite(tau_brake_s) or tau_brake_s <= 0.0:
        return None, None, f"τ が使えない（{tau_brake_s}）"
    bk = table.meta.get("launch_breakaway_u_pct")
    if bk is None or not np.isfinite(bk):
        return None, None, "pkl の meta に launch_breakaway_u_pct が無い"
    u0 = -abs(float(bk))
    free = table.with_launch_breakaway_floor(False)
    hi = float(free.accel(0.0, u0 + _SLOPE_HALF_PCT, launch=True))
    lo = float(free.accel(0.0, u0 - _SLOPE_HALF_PCT, launch=True))
    k = (hi - lo) / (2.0 * _SLOPE_HALF_PCT)
    if not np.isfinite(k) or k <= 0.0:
        return None, k, f"k ＝ {k:+.4f} km/h/s/% が正でない"
    ki = 1.0 / (tau_brake_s**2 * k)
    return ki, k, f"τ ＝ {tau_brake_s:.3f}s・k ＝ {k:.4f} km/h/s/%・Ki ＝ {ki:.4f} %/(km/h·s)"


class LaunchMoveHold:
    """発進ごとの状態（止めた FF・積分 I・受け渡しのずれ）。`step` を毎周期呼ぶ。"""

    def __init__(self, *, ki: float, bk_pct: float, brake_min_pct: float, dt_s: float) -> None:
        self.ki = float(ki)
        self.bk_pct = float(bk_pct)  # 動き出す開度（絶対値。min ＋ |meta|）
        self.brake_min_pct = float(brake_min_pct)
        self.dt_s = float(dt_s)
        self._in_block = False
        self.frozen: float | None = None  # 止めた FF（負）
        self.integral = 0.0  # I [%]（正。ブレーキを深くする量）
        self._offset = 0.0  # 受け渡しのずれ（深さ。≧ 0）
        self._handover = False

    def _reset(self) -> None:
        self.frozen = None
        self.integral = 0.0
        self._offset = 0.0
        self._handover = False

    def step(self, *, prep: bool, speed_kmh: float, ff_effort: float, midway: bool) -> float:
        """今の周期に足すブレーキの深さ [%]（≧ 0。足さなければ 0）。指令は ff_effort − 返り値。"""
        if prep:
            if not self._in_block:  # 新しい発進
                self._reset()
            self._in_block = True
            if ff_effort >= 0.0:  # アクセル・惰行の行には何もしない
                return 0.0
            moving = speed_kmh > MOVE_KMH
            if self.frozen is None:
                if not moving:
                    return 0.0
                self.frozen = ff_effort  # (ア) この周期の FF で止める
            if moving:  # (イ) 動いている間だけ深くする向きに足す
                self.integral += self.ki * speed_kmh * self.dt_s
            target = min(ff_effort, self.frozen) - self.integral
            return max(ff_effort - target, 0.0)
        if self._in_block:  # launch_prep の連なりが終わった周期: 受け渡しを決める
            self._in_block = False
            if self.frozen is not None:
                self._offset = max(-self.frozen + self.integral - self.bk_pct, 0.0)
                self._handover = self._offset > 0.0
        if not self._handover:
            return 0.0
        if ff_effort < 0.0 and -ff_effort > self.brake_min_pct + 1e-6 and midway:
            return self._offset
        self._handover = False  # ブレーキが min に戻った（アクセル・惰行・途中の終わり）
        return 0.0
