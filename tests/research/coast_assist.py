"""C12（惰行の補助 = coast assist）の入る・離すの判断（2026-10-18 段2r）。

惰行の行（計画がペダルを踏まない行 `FLAG_COAST`）では、C10・C11 は何もしない。長い惰行で偏差が
積み上がっても（`142728` は約 20 秒の惰行で −1.65 km/h）詰められない。そこで、惰行の行でも
偏差が on_kmh 以上に積み上がったら、偏差を詰める向きのペダルを浅く踏み、効果が出るまで保持し、
偏差が off_kmh 以下に戻るか符号が逆になったら離す。

このモジュールは「いつどのペダルで補助するか」だけを決める純粋なロジック（ハードウェア・
mode_drive に依存しない）。踏む量は `fwd_plan.coast_assist_effort`、呼ぶのは走行ループ。

守る規則:
- 1 つの惰行の区間では片方のペダルだけ使う（入る・離すでアクセル⇔ブレーキを往復しない）
- 前の踏む行が反対のペダルなら、そのペダルが min に戻ってから入る（段2o-9 の戻り待ち）
- 次の踏む行が反対のペダルなら、そのペダルが間に合って min に戻れるように離す
- 離してすぐに入り直さない（保持時間 hold_p だけ空ける）
- 補助するのは基準も実車速も停車速度より上の行だけ
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from tests.research.dob import PEDAL_ACCEL, PEDAL_BRAKE
from tests.research.ff_model import STOP_SPEED_KMH
from tests.research.fwd_plan import FLAG_COAST, ReturnTime


@dataclass
class CoastStats:
    """惰行の補助の集計（周期数は入れなかった行・離した回数のほかは回数）。"""

    period_s: float = 0.05
    accel_entries: int = 0  # 入った回数（ペダル別）
    brake_entries: int = 0
    accel_s: float = 0.0  # 補助していた秒数（ペダル別）
    brake_s: float = 0.0
    # 入る条件（偏差 ≥ on）は満たしたが入れなかった周期数。最初に外れた理由で数える
    blocked_section: int = 0  # この惰行の区間で反対のペダルを使った
    blocked_reentry: int = 0  # 離してから hold_p たっていない
    blocked_prev_return: int = 0  # 前の反対のペダルが min に戻っていない
    blocked_next_return: int = 0  # 次の反対のペダルまでに保持して戻る時間がない
    released_next: int = 0  # (a) 次の反対のペダルに間に合うよう、保持の途中で離した回数

    def add_active(self, pedal: str) -> None:
        if pedal == PEDAL_ACCEL:
            self.accel_s += self.period_s
        else:
            self.brake_s += self.period_s

    def summary(self) -> str:
        p = self.period_s
        return (
            f"入った アクセル {self.accel_entries} 回（{self.accel_s:.1f}s）／"
            f"ブレーキ {self.brake_entries} 回（{self.brake_s:.1f}s）・"
            f"入れなかった（周期）区間の反対ペダル {self.blocked_section * p:.1f}s・"
            f"入り直しの間隔 {self.blocked_reentry * p:.1f}s・"
            f"前の戻り待ち {self.blocked_prev_return * p:.1f}s・"
            f"次の戻り待ち {self.blocked_next_return * p:.1f}s・"
            f"次のペダルのため保持の途中で離した {self.released_next} 回"
        )


def coast_settings_text(
    on_kmh: float, off_kmh: float, hold_coef: float, hold_accel_s: float, hold_brake_s: float,
    floor_onset: bool = False,
) -> str:
    """C12 の設定の表示（走行前とレポートで同じ文）。"""
    return (
        f"入る {on_kmh:g} km/h・離す {off_kmh:g} km/h・保持係数 {hold_coef:g}"
        f"（保持 アクセル {hold_accel_s:.2f}s / ブレーキ {hold_brake_s:.2f}s）"
        f"・効き始めの下限 {'あり' if floor_onset else 'なし'}"
    )


class CoastAssist:
    """惰行の行の補助の状態機械。plan の配列は初期化のときに一度だけ走査する。

    plan_effort は絶対開度（正＝アクセル・負＝ブレーキ。min を含む）。「踏む行」は effort ≠ 0 の
    行（先置き・停車保持のブレーキも含む）。accel_onset_effort・brake_onset_effort は効き始めの
    絶対開度（アクセル正・ブレーキ負。保持のあと次のペダルまでに戻れるかの計算に使う）。
    """

    def __init__(
        self,
        plan_effort: np.ndarray,
        plan_flag: np.ndarray,
        plan_t: np.ndarray,
        *,
        on_kmh: float,
        off_kmh: float,
        hold_accel_s: float,
        hold_brake_s: float,
        ret: ReturnTime,
        period_s: float,
        accel_onset_effort: float,
        brake_onset_effort: float,
    ) -> None:
        self.flag = np.asarray(plan_flag)
        self.on_kmh = on_kmh
        self.off_kmh = off_kmh
        self.hold_s = {PEDAL_ACCEL: hold_accel_s, PEDAL_BRAKE: hold_brake_s}
        self.onset_effort = {PEDAL_ACCEL: accel_onset_effort, PEDAL_BRAKE: brake_onset_effort}
        self.ret = ret
        self.period_s = period_s
        self.stats = CoastStats(period_s=period_s)
        effort = np.asarray(plan_effort, dtype=float)
        t = np.asarray(plan_t, dtype=float)
        n = len(effort)
        # 各行の直前の踏む行の (effort, 終わった時刻) と、次の踏む行の (effort, 始まる時刻)。
        # 無ければ effort ＝ 0・時刻 ＝ nan
        self.prev_effort = np.zeros(n)
        self.prev_end = np.full(n, math.nan)
        self.next_effort = np.zeros(n)
        self.next_start = np.full(n, math.nan)
        last_e, last_end = 0.0, math.nan
        for i in range(n):
            self.prev_effort[i], self.prev_end[i] = last_e, last_end
            if effort[i] != 0.0:
                last_e, last_end = float(effort[i]), float(t[i]) + period_s
        nxt_e, nxt_t = 0.0, math.nan
        for i in range(n - 1, -1, -1):
            self.next_effort[i], self.next_start[i] = nxt_e, nxt_t
            if effort[i] != 0.0:
                nxt_e, nxt_t = float(effort[i]), float(t[i])
        # 状態
        self.pedal: str | None = None  # 今使っているペダル
        self.t_on = -math.inf
        self.t_off = -math.inf
        self.section_pedal: str | None = None  # この惰行の区間で使ったペダル
        self._effort = 0.0  # 今の補助の effort（mode_drive が note_effort で渡す）

    def note_effort(self, effort: float) -> None:
        """今の周期で出した補助の effort（(a) の戻り時間の計算に使う）。"""
        self._effort = effort

    def _release(self, t: float) -> None:
        self.pedal = None
        self.t_off = t

    def _next_opposite(self, i: int, pedal: str) -> bool:
        e = self.next_effort[i]
        return e != 0.0 and (e < 0.0) == (pedal == PEDAL_ACCEL)

    def _prev_opposite(self, i: int, pedal: str) -> bool:
        e = self.prev_effort[i]
        return e != 0.0 and (e < 0.0) == (pedal == PEDAL_ACCEL)

    def step(
        self, *, t: float, i: int, dev_kmh: float, speed_kmh: float, ref_kmh: float
    ) -> str | None:
        """今の周期で補助に使うペダル（PEDAL_ACCEL / PEDAL_BRAKE）か None を返す。
        dev_kmh ＝ 基準 − 実車速（正＝遅れている → アクセル、負＝速すぎる → ブレーキ）。"""
        is_coast = int(self.flag[i]) == FLAG_COAST
        if not is_coast:
            self.section_pedal = None  # 惰行の区間が終わった
        if not (is_coast and ref_kmh > STOP_SPEED_KMH and speed_kmh > STOP_SPEED_KMH):
            if self.pedal is not None:
                self._release(t)
            return None

        if self.pedal is not None:
            pedal = self.pedal
            hold = self.hold_s[pedal]
            # (a) 次の反対のペダルまでに戻れる時間しか残っていない: 保持の途中でも離す
            if self._next_opposite(i, pedal):
                effort = self._effort if self._effort != 0.0 else self.onset_effort[pedal]
                if self.next_start[i] - t <= self.ret.seconds(effort, self.period_s):
                    self.stats.released_next += 1
                    self._release(t)
                    return None
            # (c) 保持が済み、偏差が off 以下に戻った（または符号が逆になった）
            wrong_sign = (dev_kmh < 0.0) if pedal == PEDAL_ACCEL else (dev_kmh > 0.0)
            if t - self.t_on >= hold - 1e-9 and (abs(dev_kmh) <= self.off_kmh or wrong_sign):
                self._release(t)
                return None
            self.stats.add_active(pedal)
            return pedal

        if abs(dev_kmh) < self.on_kmh:
            return None
        pedal = PEDAL_ACCEL if dev_kmh > 0.0 else PEDAL_BRAKE
        hold = self.hold_s[pedal]
        st = self.stats
        if self.section_pedal is not None and self.section_pedal != pedal:
            st.blocked_section += 1
            return None
        if t - self.t_off < hold - 1e-9:
            st.blocked_reentry += 1
            return None
        if self._prev_opposite(i, pedal) and (
            t - self.prev_end[i] < self.ret.seconds(float(self.prev_effort[i]), self.period_s)
        ):
            st.blocked_prev_return += 1
            return None
        if self._next_opposite(i, pedal) and (
            self.next_start[i] - t
            < hold + self.ret.seconds(self.onset_effort[pedal], self.period_s)
        ):
            st.blocked_next_return += 1
            return None
        self.pedal, self.t_on, self.section_pedal = pedal, t, pedal
        self._effort = 0.0
        if pedal == PEDAL_ACCEL:
            st.accel_entries += 1
        else:
            st.brake_entries += 1
        st.add_active(pedal)
        return pedal
