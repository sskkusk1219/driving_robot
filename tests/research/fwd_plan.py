"""基準車速 → 開度の列（走る前に解く）。順モデルの表（C7）を逆に引いて作る（段2g）。

表の定義（Apollo の形）は「時刻 t の車速 × 時刻 t の指令の開度 → 時刻 t + L の加速度」。
だから時刻 t の指令は invert(v = ref(t), a = 要る加速度(t + L)) で出す。L はペダル別
（accel_lag_s・brake_lag_s）で、表を作ったときの値（pkl の meta）を既定で使う。

制御周期の各時刻 t について（符号は既存の FF と同じ: 正＝アクセル開度、負＝ブレーキ開度）:
    1. 停車保持: ref(t) <= 停車速度 かつ ref(t + stop_horizon_s) <= 停車速度
       → −stop_brake_opening_pct（停車速度と規則は `ff_candidate.predict_effort` の
       「1. 停車レジーム」と同じ）。
    2. アクセル: 要る加速度 need_a = 基準の「t + accel_lag_s を中心にした幅 need_window_s の窓」
       の傾き、車速 v = ref(t)。惰行の加速度 c = coast_accel(v)。
       need_a − c >= coast_band_kmhs のときだけ
       アクセル: 表を逆に引いて u > 0 なら effort = アクセル不感帯 + u（u <= 0 なら 0 扱い）。
    3. そうでなく need_b − c <= −coast_band_kmhs のときだけブレーキ
       （need_b は t + brake_lag_s の窓）:
       u < 0 なら effort = −(ブレーキ不感帯 + |u|)。
    4. どちらでもなければ 0（惰行）。
effort は ±100 にクリップする。

発進側（2026-10-14 段2o-4）: 表が launch_accel を持つとき、基準車速が上がっている時刻
（need_a > 0 のアクセル判定、need_b > 0 のブレーキ判定）だけ、惰行の加速度 c と逆引きを発進側の
配列で行う（低車速のブレーキ側は、同じ開度でも「発進」と「停止へ」で加速度が違うため）。
基準が下がっている・変わらない時刻と、停車保持・クリップは本体のまま。launch_accel が None の
表は今までと同じ計算。

停止側（2026-10-18 段2o-10）: 表が stop_accel を持つとき、停車へ向かう減速の時刻（need < 0・
基準車速が停止側の表の範囲内・その先 stop_horizon_s 以内に基準が停車速度以下になる）は、発進側と
同じやり方で停止側の配列を引く。判定は `low_speed_sides`（deviation_offset_detail と共有）。
stop_accel が None の表は今までと同じ計算。

戻し（2026-10-14 段2o-5）: 表が release_grid を持つとき、ブレーキの時刻を上の計算の後に時刻順に
解き直す。時刻 i の「直近 RELEASE_WINDOW_S 秒（i を含まない）に計画済みの u の最小」を m とし、
戻し量 0 で解いた u0 が m 以下（深い）ならそのまま。浅い（u0 > m）ときは u ∈ (m, 0] で
accel(v, u, release = u − m) = 要る加速度 を解く（u の格子点で評価して交差を直線補間。外れたら
端 m／0 に止める）。m は解き直した後の u の列で数える（直前の戻しも反映）。停車保持の時刻の
深いブレーキも m に入る（学習の戻し量が全時系列で計算されているのと同じ）。発進側の表を引いた
時刻（基準が上がっているブレーキ）は戻しではないので解き直さない。release_grid が None の表は
今までと完全に同じ（ベクトル化のまま）。
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from tests.research.ff_bias import PEDAL_ACCEL, PEDAL_BRAKE, PEDAL_COAST, count_pedal_switches
from tests.research.ff_model import STOP_SPEED_KMH
from tests.research.fwd_data import DEFAULT_ACCEL_LAG_S as _DATA_ACCEL_LAG_S
from tests.research.fwd_data import DEFAULT_BRAKE_LAG_S as _DATA_BRAKE_LAG_S
from tests.research.fwd_data import RELEASE_WINDOW_S
from tests.research.fwd_map import POINT_FILLED, POINT_MEASURED, POINT_OUT, ForwardTable

DEFAULT_CONTROL_PERIOD_S = 0.05
# 遅れ L の既定は fwd_data と同じ（データ由来。fwd_data の定数のコメント参照）。
# 実際は表を作ったときの L（pkl の meta）が優先される
DEFAULT_ACCEL_LAG_S = _DATA_ACCEL_LAG_S
DEFAULT_BRAKE_LAG_S = _DATA_BRAKE_LAG_S
# ユーザー決定（2026-10-03）: 要る加速度が惰行の加速度から ±この値以上離れるまで惰行（0）にする。
# 開度が惰行の付近でアクセル⇔ブレーキを往復するのを防ぐ帯（仮の値、実機で見る）
DEFAULT_COAST_BAND_KMHS = 0.25
# 仮値: 要る加速度の窓（学習側の測り方にそろえた。段2h の実機後に決める）
DEFAULT_NEED_WINDOW_S = 1.0
NEED_STEP_S = 0.1  # 窓の中のサンプル刻み（`pedal_select.window_slope_kmhs` の既定と同じ）
EFFORT_LIMIT_PCT = 100.0

# 2026-10-17 段2o-9: ペダルが min へ戻るまでの時間 T = 遅れ + 戻す量 ÷ 戻る速さ + 1 周期の余裕。
# 遅れ・戻る速さは手順2 の CSV から測って pkl の meta に入る（{accel,brake}_return_delay_s・
# _return_rate_pct_s。`pedal_lag.estimate_pedal_return`）。meta に無い古い pkl は次の仮値を使い、
# 表示で知らせる。仮値は測れた値（遅れ約 0.1 秒・約 100 %/秒）より遅い側（待つ時間が長くなる側）
DEFAULT_RETURN_DELAY_S = 0.15
DEFAULT_RETURN_RATE_PCT_S = 50.0

# flag: 0〜2 は fwd_map の POINT_*（実測・穴埋め・範囲外）
FLAG_MEASURED = POINT_MEASURED
FLAG_FILLED = POINT_FILLED
FLAG_OUT = POINT_OUT
FLAG_COAST = 3
FLAG_STOP = 4
FLAG_PREPOS = 5  # 2026-10-16 段2o-6: 次に踏むペダルを min から効き始めの手前へ先に置いている区間
FLAG_LABELS = {
    FLAG_MEASURED: "実測",
    FLAG_FILLED: "穴埋め",
    FLAG_OUT: "範囲外",
    FLAG_COAST: "惰行",
    FLAG_STOP: "停車保持",
    FLAG_PREPOS: "先置き",
}


def _eval_ref(ref_at: Callable[[float], float], times: np.ndarray) -> np.ndarray:
    """スカラー関数 ref_at を時刻の配列で評価する（同じ時刻は 1 回だけ呼ぶ）。"""
    flat = np.round(times.reshape(-1), 9)
    uniq, inv = np.unique(flat, return_inverse=True)
    vals = np.array([ref_at(float(x)) for x in uniq], dtype=float)
    return vals[inv].reshape(times.shape)


def window_slope_vec(
    ref_at: Callable[[float], float],
    t_s: np.ndarray,
    center_s: float,
    width_s: float,
    step_s: float = NEED_STEP_S,
) -> np.ndarray:
    """`pedal_select.window_slope_kmhs` のベクトル版（同じ窓・同じ最小二乗の傾き）。"""
    n = max(1, round(width_s / step_s))
    ks = np.arange(n + 1) * step_s
    start = np.asarray(t_s, dtype=float) + center_s - width_s / 2.0
    vs = _eval_ref(ref_at, start[:, None] + ks[None, :])
    kc = ks - ks.mean()
    return ((vs - vs.mean(axis=1, keepdims=True)) @ kc) / float(np.dot(kc, kc))


def switch_times_of(t_s: np.ndarray, phases: list[str]) -> list[float]:
    """フェーズ列（PEDAL_ACCEL / PEDAL_BRAKE / PEDAL_COAST）から切替の時刻を出す。"""
    out: list[float] = []
    last: str | None = None
    for t, p in zip(t_s.tolist(), phases, strict=True):
        if p not in (PEDAL_ACCEL, PEDAL_BRAKE):
            continue
        if last is not None and p != last:
            out.append(t)
        last = p
    return out


@dataclass
class PrepositionStats:
    """先置き（`preposition`）の集計（2026-10-16 段2o-6）。回数は「踏み出し」の数。"""

    n_placed: int = 0  # 置けた（lead_s ぶん全部）
    n_short: int = 0  # 時間が足りず、lead_s より短く置いた（部分的）
    n_no_time: int = 0  # 間に合わない（置く場所が 0 点。停車保持→発進の直接切替を含む）
    n_not_needed: int = 0  # 置く開度が踏み出しの開度以上で、置く意味がない
    n_no_onset: int = 0  # そのペダルの効き始めが無い（pkl の meta に無い・margin で 0 以下）
    prepos_s: float = 0.0  # 先置きにした合計秒数

    def summary(self) -> str:
        return (
            f"置いた {self.n_placed} 回・時間が足りず短く {self.n_short} 回・"
            f"間に合わず {self.n_no_time} 回・不要 {self.n_not_needed} 回・"
            f"効き始めなし {self.n_no_onset} 回・先置き合計 {self.prepos_s:.1f}s"
        )


@dataclass(frozen=True)
class ReturnTime:
    """ペダルが min へ戻るまでの時間（2026-10-17 段2o-9）。T = 遅れ + 戻す量 ÷ 戻る速さ + 1 周期。

    戻す量 = 指令の開度 − そのペダルの min。1 周期の余裕は、実開度の読みが 1 周期遅れて見えるぶん。
    *_default が True のペダルは、pkl の meta に測った値が無く仮値（DEFAULT_RETURN_*）を使っている。
    """

    accel_min_pct: float
    brake_min_pct: float
    accel_delay_s: float = DEFAULT_RETURN_DELAY_S
    accel_rate_pct_s: float = DEFAULT_RETURN_RATE_PCT_S
    brake_delay_s: float = DEFAULT_RETURN_DELAY_S
    brake_rate_pct_s: float = DEFAULT_RETURN_RATE_PCT_S
    accel_default: bool = True
    brake_default: bool = True

    def seconds(self, effort: float, period_s: float) -> float:
        """指令 effort（正＝アクセル・負＝ブレーキ）のペダルが min へ戻るまでの時間 T [s]。"""
        if effort > 0.0:
            depth = max(effort - self.accel_min_pct, 0.0)
            return self.accel_delay_s + depth / self.accel_rate_pct_s + period_s
        depth = max(-effort - self.brake_min_pct, 0.0)
        return self.brake_delay_s + depth / self.brake_rate_pct_s + period_s

    def gap_rows(self, effort: float, period_s: float) -> int:
        """切り替えで挟む「戻す側が min の行」の最小数（T を周期で切り上げ）。"""
        return max(math.ceil(self.seconds(effort, period_s) / period_s - 1e-9), 0)

    def defaults_used(self) -> tuple[str, ...]:
        out = []
        if self.accel_default:
            out.append("アクセル")
        if self.brake_default:
            out.append("ブレーキ")
        return tuple(out)


def return_time_of(table: ForwardTable, accel_min_pct: float, brake_min_pct: float) -> ReturnTime:
    """表の meta から ReturnTime を作る。遅れと速さの両方が揃ったペダルだけ測定値、
    そうでなければ仮値（*_default=True）。"""

    def pick(prefix: str) -> tuple[float, float, bool]:
        d = table.meta.get(f"{prefix}_return_delay_s")
        r = table.meta.get(f"{prefix}_return_rate_pct_s")
        if (
            d is not None and r is not None and np.isfinite(d) and np.isfinite(r)
            and float(d) >= 0.0 and float(r) > 0.0
        ):
            return float(d), float(r), False
        return DEFAULT_RETURN_DELAY_S, DEFAULT_RETURN_RATE_PCT_S, True

    ad, ar, adef = pick("accel")
    bd, br, bdef = pick("brake")
    return ReturnTime(
        accel_min_pct=accel_min_pct, brake_min_pct=brake_min_pct,
        accel_delay_s=ad, accel_rate_pct_s=ar, brake_delay_s=bd, brake_rate_pct_s=br,
        accel_default=adef, brake_default=bdef,
    )


@dataclass
class ReturnGapStats:
    """切り替えのすき間の確保（`ensure_return_gap`）の集計（2026-10-17 段2o-9）。"""

    n_switches: int = 0  # アクセル⇔ブレーキの切り替えの数
    n_cut: int = 0  # 戻す側の最後を前倒しで min にした切り替えの数
    cut_s: float = 0.0  # 前倒しで min にした合計秒数
    n_delayed: int = 0  # 次のペダルを遅らせた切り替えの数
    delayed_s: float = 0.0  # 遅らせた合計秒数
    n_violations: int = 0  # 処理の後に残ったすき間不足の数（0 のはず）
    defaults_used: tuple[str, ...] = ()  # 測定値が無く仮値を使ったペダル

    def summary(self) -> str:
        text = (
            f"切り替え {self.n_switches} 回・戻す側を前倒し {self.n_cut} 回（{self.cut_s:.2f}s）・"
            f"次を遅らせ {self.n_delayed} 回（{self.delayed_s:.2f}s）・"
            f"すき間不足の残り {self.n_violations} 回"
        )
        if self.defaults_used:
            text += f"・戻る時間は仮値（{'・'.join(self.defaults_used)}。pkl に測定値なし）"
        return text


@dataclass
class ForwardPlan:
    """開度の列。配列はすべて同じ長さ（制御周期の刻み）。"""

    t_s: np.ndarray
    effort_pct: np.ndarray  # 正＝アクセル開度、負＝ブレーキ開度
    need_accel_kmhs: np.ndarray
    flag: np.ndarray  # FLAG_*
    period_s: float = DEFAULT_CONTROL_PERIOD_S
    preposition: PrepositionStats | None = None  # 先置きを有効にしたときだけ入る
    return_gap: ReturnGapStats | None = None  # 切り替えのすき間の確保（段2o-9）
    # 各時刻で「その先 stop_horizon_s 以内に基準が停車速度以下になる」か（停止側の表を引く条件の
    # 一部。段2o-10。C10 の上乗せが build_plan と同じ表を引くために持つ）。None ＝ 持たない
    stops_ahead: np.ndarray | None = None

    def index_at(self, t_s: float) -> int:
        """時刻 t に最も近い行の番号（`at` と同じ最近傍）。"""
        return int(np.clip(round((t_s - self.t_s[0]) / self.period_s), 0, len(self.t_s) - 1))

    def at(self, t_s: float) -> float:
        """時刻 t の effort。最近傍の点を返す。

        アクセル⇔ブレーキの境で 0 を経由しないよう、補間はしない。
        """
        return float(self.effort_pct[self.index_at(t_s)])

    def seconds_of(self, flag: int) -> float:
        return float(np.count_nonzero(self.flag == flag)) * self.period_s

    def out_of_range_s(self) -> float:
        return self.seconds_of(FLAG_OUT)

    def filled_s(self) -> float:
        return self.seconds_of(FLAG_FILLED)

    def pedal_seconds(self) -> dict[str, float]:
        """アクセル・ブレーキ・惰行・停車保持の秒数（停車保持はブレーキに含めない）。"""
        stop = self.flag == FLAG_STOP
        pre = self.flag == FLAG_PREPOS  # 先置きは別に数える（アクセル・ブレーキ・惰行に含めない）
        other = ~stop & ~pre
        e = self.effort_pct
        return {
            "アクセル": float(np.count_nonzero((e > 0.0) & other)) * self.period_s,
            "ブレーキ": float(np.count_nonzero((e < 0.0) & other)) * self.period_s,
            "惰行": float(np.count_nonzero((e == 0.0) & other)) * self.period_s,
            "停車保持": float(np.count_nonzero(stop)) * self.period_s,
            "先置き": float(np.count_nonzero(pre)) * self.period_s,
        }

    def _phases(self, include_stop: bool) -> list[str]:
        out = []
        for e, f in zip(self.effort_pct.tolist(), self.flag.tolist(), strict=True):
            if (f == FLAG_STOP and not include_stop) or f == FLAG_PREPOS:
                out.append(PEDAL_COAST)  # 先置きは惰行扱い（切替の数え方を変えない）
            else:
                out.append(PEDAL_ACCEL if e > 0.0 else PEDAL_BRAKE if e < 0.0 else PEDAL_COAST)
        return out

    def switch_times(self, include_stop: bool = True) -> list[float]:
        """アクセル⇔ブレーキの切替が起きた時刻 [s]（切替先のペダルが最初に出た点の時刻。
        数え方は `count_pedal_switches` と同じ: 惰行は飛ばして直前のペダルと比べる）。"""
        return switch_times_of(self.t_s, self._phases(include_stop))

    def pedal_switches(self, include_stop: bool = True) -> int:
        """アクセル⇔ブレーキの切替回数（`ff_bias.count_pedal_switches` と同じ数え方。
        include_stop=False なら停車保持の区間を飛ばして数える）。"""
        return count_pedal_switches(self._phases(include_stop))


def build_plan(
    ref_at: Callable[[float], float],
    duration_s: float,
    table: ForwardTable,
    *,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    stop_brake_opening_pct: float,
    stop_horizon_s: float,
    period_s: float = DEFAULT_CONTROL_PERIOD_S,
    accel_lag_s: float | None = None,
    brake_lag_s: float | None = None,
    need_window_s: float = DEFAULT_NEED_WINDOW_S,
    coast_band_kmhs: float = DEFAULT_COAST_BAND_KMHS,
    preposition_enabled: bool = False,
    preposition_margin_pct: float = 0.5,
    preposition_lead_s: float = 0.2,
    return_gap_enabled: bool = True,
) -> ForwardPlan:
    """基準車速から開度の列を作る（走る前に全時刻ぶんを解く）。

    accel_lag_s・brake_lag_s が None なら表の meta（表を作ったときの L）、それも無ければ既定値。
    preposition_enabled なら、最後に `preposition` で次に踏むペダルを先に置く
    （効き始めは表の meta の accel_onset_u_pct・brake_onset_u_pct。2026-10-16 段2o-6）。
    2026-10-17 段2o-9: 先置きの待ちと、全部の切り替えのすき間を「戻す側が min に戻るまでの時間」
    （表の meta の戻る時間。`ReturnTime`）にそろえる。return_gap_enabled なら最後に
    `ensure_return_gap` ですき間不足を直す。
    """
    la = _lag(accel_lag_s, table, "accel_lag_s", DEFAULT_ACCEL_LAG_S)
    lb = _lag(brake_lag_s, table, "brake_lag_s", DEFAULT_BRAKE_LAG_S)
    n = int(np.floor(duration_s / period_s + 1e-9)) + 1
    t = np.arange(n) * period_s

    # 1. 停車保持
    ref_now = _eval_ref(ref_at, t)
    ref_stop = _eval_ref(ref_at, t + stop_horizon_s)
    stop = (ref_now <= STOP_SPEED_KMH) & (ref_stop <= STOP_SPEED_KMH)

    # 表の定義: 時刻 t の車速 ref(t)・指令 → 時刻 t + L の加速度。だから車速は ref(t)、
    # 要る加速度は t + L を中心にした窓の傾き
    v = ref_now
    coast = np.asarray(table.coast_accel(v), dtype=float)
    need_a = window_slope_vec(ref_at, t, la, need_window_s)
    need_b = window_slope_vec(ref_at, t, lb, need_window_s)
    u_a, kind_a = table.invert(v, need_a)
    u_b, kind_b = table.invert(v, need_b)

    coast_a = coast_b = coast
    stops_ahead = ref_stop <= STOP_SPEED_KMH
    launch_a, stop_a = low_speed_sides(table, need_a, v, stops_ahead)
    launch_b, stop_b = low_speed_sides(table, need_b, v, stops_ahead)
    # 発進側（基準が上がっている時刻）・停止側（停車へ向かう減速の時刻）だけ、その表で解く
    for use_launch, side_a, side_b in ((True, launch_a, launch_b), (False, stop_a, stop_b)):
        if not (side_a.any() or side_b.any()):
            continue
        kw = {"launch": True} if use_launch else {"stop": True}
        coast_s = np.asarray(table.coast_accel(v, **kw), dtype=float)
        u_a_s, kind_a_s = table.invert(v, need_a, **kw)
        u_b_s, kind_b_s = table.invert(v, need_b, **kw)
        coast_a = np.where(side_a, coast_s, coast_a)
        coast_b = np.where(side_b, coast_s, coast_b)
        u_a, kind_a = np.where(side_a, u_a_s, u_a), np.where(side_a, kind_a_s, kind_a)
        u_b, kind_b = np.where(side_b, u_b_s, u_b), np.where(side_b, kind_b_s, kind_b)

    want_accel = need_a - coast_a >= coast_band_kmhs - 1e-12
    want_brake = ~want_accel & (need_b - coast_b <= -coast_band_kmhs + 1e-12)
    is_accel = want_accel & (u_a > 0.0)
    is_brake = want_brake & (u_b < 0.0)
    effort = np.zeros(n)
    effort[is_accel] = accel_deadband_pct + u_a[is_accel]
    effort[is_brake] = -(brake_deadband_pct + np.abs(u_b[is_brake]))
    flag = np.full(n, FLAG_COAST, dtype=int)
    flag[is_accel] = kind_a[is_accel]
    flag[is_brake] = kind_b[is_brake]
    need = np.where(want_brake, need_b, need_a)

    effort[stop] = -stop_brake_opening_pct
    flag[stop] = FLAG_STOP
    if table.release_grid is not None and table.release_accel is not None:
        redo = is_brake & ~stop
        redo &= ~(launch_b | stop_b)  # 発進側・停止側の表で解いた時刻は戻しではない
        _resolve_releases(
            effort, redo, u_b, v, need_b, table, brake_deadband_pct, accel_deadband_pct, period_s
        )
    effort = np.clip(effort, -EFFORT_LIMIT_PCT, EFFORT_LIMIT_PCT)
    stats = None
    ret = return_time_of(table, accel_deadband_pct, brake_deadband_pct)
    if preposition_enabled:
        m = preposition_margin_pct
        accel_pre = _pre_opening(table, "accel_onset_u_pct", accel_deadband_pct, m)
        brake_pre = _pre_opening(table, "brake_onset_u_pct", brake_deadband_pct, m)
        stats = preposition(
            effort,
            flag,
            accel_pre_pct=accel_pre,
            brake_pre_pct=brake_pre,
            lead_s=preposition_lead_s,
            ret=ret,
            period_s=period_s,
        )
    gap_stats = (
        ensure_return_gap(effort, flag, ret=ret, period_s=period_s) if return_gap_enabled else None
    )
    return ForwardPlan(
        t_s=t, effort_pct=effort, need_accel_kmhs=need, flag=flag, period_s=period_s,
        preposition=stats, return_gap=gap_stats, stops_ahead=stops_ahead,
    )


def low_speed_sides(
    table: ForwardTable,
    need_kmhs: np.ndarray | float,
    ref_kmh: np.ndarray | float,
    stops_ahead: np.ndarray | bool,
) -> tuple[np.ndarray, np.ndarray]:
    """発進側・停止側のどちらの表を引くかの判定 (launch, stop)（bool の配列。2026-10-18 段2o-10）。

    build_plan と deviation_offset_detail が同じ条件で同じ表を引くための共有関数。
    - launch: 表が launch_accel を持ち、要る加速度 need > 0（基準が上がっている）。
    - stop: 表が stop_accel を持ち、need < 0（減速）で、基準車速が停止側の表の車速の範囲
      （meta の stop_max_speed_kmh 未満）で、その先 stop_horizon_s 以内に基準が停車速度以下に
      なる（stops_ahead。停車保持の判定 ref(t + stop_horizon_s) <= 停車速度 と同じ）。
    表が持たない側は全部 False（今までと同じ計算になる）。
    """
    need = np.asarray(need_kmhs, dtype=float)
    ref = np.asarray(ref_kmh, dtype=float)
    ahead = np.asarray(stops_ahead, dtype=bool)
    launch = (need > 0.0) if table.launch_accel is not None else np.zeros(need.shape, dtype=bool)
    if table.stop_accel is not None:
        top = float(table.meta.get("stop_max_speed_kmh", 0.0))
        stop = (need < 0.0) & ahead & (ref < top)
    else:
        stop = np.zeros(np.broadcast(need, ref, ahead).shape, dtype=bool)
    return launch, stop


def deviation_offset(table: ForwardTable, **kwargs: Any) -> float:
    """`deviation_offset_detail` の上乗せ後 effort [%] だけ返す（引数は同じ）。"""
    return deviation_offset_detail(table, **kwargs)[0]


def deviation_offset_detail(
    table: ForwardTable,
    *,
    effort: float,
    flag: int,
    need_kmhs: float,
    ref_kmh: float,
    row_kmh: float,
    dev_kmh: float,
    tau_accel_s: float | None,
    tau_brake_s: float | None,
    accel_min_pct: float,
    brake_min_pct: float,
    stops_ahead: bool = False,
    dob_kmhs: float = 0.0,
) -> tuple[float, bool]:
    """C10: 開度の列の 1 行（effort）に、偏差の分だけ上乗せした effort [%] を返す（段2p）。

    偏差 dev ＝ 基準車速 − 実車速（遅れていれば正）。dev を τ 秒で 0 に戻す加速度 Δa ＝ dev ÷ τ を
    要る加速度に足して表を引き直し、基準のままの開度との差 Δu を effort に足す:
        Δu ＝ invert(row_kmh, need ＋ Δa) − invert(ref_kmh, need)
    row_kmh は表を引く車速（案A は基準車速 ref、案B は実車速）。
    - アクセルの行は max(effort ＋ Δu, accel_min)、ブレーキの行は min(effort ＋ Δu, −brake_min)。
      減らすときは min で止め、反対のペダルには移らない（新しい切り替えを作らない）。
    - 惰行・停車保持・先置きの行、effort ＝ 0 の行、その行のペダルの τ が None の行は
      何もしない（effort をそのまま返す）。
    返す値は (上乗せ後の effort [%], min で止めたか)。止めたとは、減らす向きの差が min を
    越えるため min に止めたこと（集計用）。
    - 表の引き方は build_plan と同じ（`low_speed_sides`。need ＞ 0 で表が launch_accel を持つなら
      発進側、停車へ向かう減速なら停止側）。stops_ahead は ForwardPlan.stops_ahead のその時刻の値
      （省略は False ＝ 停止側を引かない）。基準車速の判定には ref_kmh を使う。
      戻し（release）の引き直しは上乗せの差には使わない。局所の差だけを取る近似。
    - dob_kmhs（C11。2026-10-18 段2q）: その行のペダルの外乱の推定 d ＝ 実測 − 表の予測 [km/h/s]。
      表が予測するより加速度が出ていない（d ＜ 0）なら多く踏む必要があるので、要る加速度から d を
      引いて表を引く: invert(row, need ＋ Δa − d)。ブレーキの行は d ＞ 0（減速が浅い）で
      need − d がより負になり、深く踏む。既定 0 は C10 と同じ出力。
      τ が None の行（遅れが測れない）は dob_kmhs ≠ 0 でも何もしない。
      τ ≦ 0（deviation_c ＝ 0）のときは偏差の項 Δa ＝ 0 で DOB だけ効く。
    """
    if flag in (FLAG_COAST, FLAG_STOP, FLAG_PREPOS) or effort == 0.0:
        return effort, False
    is_accel = effort > 0.0
    tau = tau_accel_s if is_accel else tau_brake_s
    if tau is None or (tau <= 0.0 and dob_kmhs == 0.0):
        return effort, False
    lv, sv = low_speed_sides(table, need_kmhs, ref_kmh, stops_ahead)
    launch, stop = bool(lv), bool(sv)
    d_a = dev_kmh / tau if tau > 0.0 else 0.0
    u0, _ = table.invert(ref_kmh, need_kmhs, launch=launch, stop=stop)
    u1, _ = table.invert(row_kmh, need_kmhs + d_a - dob_kmhs, launch=launch, stop=stop)
    du = float(u1) - float(u0)
    raw = effort + du
    if is_accel:
        new, clamped = max(raw, accel_min_pct), raw < accel_min_pct
    else:
        new, clamped = min(raw, -brake_min_pct), raw > -brake_min_pct
    return float(np.clip(new, -EFFORT_LIMIT_PCT, EFFORT_LIMIT_PCT)), clamped


def _pre_opening(
    table: ForwardTable, key: str, min_pct: float, margin_pct: float
) -> float | None:
    """先置きの絶対開度 [%] ＝ min ＋ (効き始め − margin)。効き始めが無い・0 以下なら None。"""
    on = table.meta.get(key)
    if on is None or not np.isfinite(on):
        return None
    u_pre = float(on) - margin_pct
    return min_pct + u_pre if u_pre > 0.0 else None


def preposition(
    effort: np.ndarray,
    flag: np.ndarray,
    *,
    accel_pre_pct: float | None,
    brake_pre_pct: float | None,
    lead_s: float,
    ret: ReturnTime,
    period_s: float,
) -> PrepositionStats:
    """次に踏むペダルを、踏む直前に min から効き始めの手前へ先に置く（effort・flag を書き換える）。

    2026-10-16 段2o-6。片足のルール（片方が min より上なら、もう片方は min）を守りながら、
    人が次のペダルへ足を置くのと同じく「ちょうど間に合う」ように動かす。ロボットは開度の列を
    先に全部持っているので、踏み出しの lead_s 前から置きにいく。
    - effort の正がアクセル・負がブレーキ・0 が両方 min（accel_pre_pct・brake_pre_pct は絶対開度、
      None ならそのペダルは先置きしない）
    - 置けるのは、踏み出しの直前に続く 0（惰行）の連なりの中だけ（0 でない点は書き換えない）。
      もう一方のペダルの指令が min になってから、そのペダルが min に戻るまでの時間 T（`ReturnTime`。
      戻す量は直前の指令の深さ。2026-10-17 段2o-9）がたつまでは置かない
    - 停車保持→発進のように 0 を挟まず直接切り替わるときは置く場所が無い（間に合わず）
    """
    n = len(effort)
    stats = PrepositionStats()
    orig = effort.copy()
    lead_n = int(round(lead_s / period_s))
    n_pre = 0
    for i in range(n):
        e = orig[i]
        if e == 0.0:
            continue
        if i == 0:
            continue  # 系列の先頭は踏み出しではない（途中から始まっているだけ）
        prev = orig[i - 1]
        if prev != 0.0 and (prev > 0.0) == (e > 0.0):
            continue  # 同じペダルが続いている（踏み出しではない）
        pre = accel_pre_pct if e > 0.0 else brake_pre_pct
        if pre is None:
            stats.n_no_onset += 1
            continue
        if pre >= abs(e):
            stats.n_not_needed += 1
            continue
        s = i  # 直前の 0 の連なりの先頭
        while s > 0 and orig[s - 1] == 0.0:
            s -= 1
        earliest = s
        if s > 0 and (orig[s - 1] > 0.0) != (e > 0.0):
            # もう一方が min になってから、そのペダルが min に戻るまでの時間 T 後
            earliest = s + ret.gap_rows(float(orig[s - 1]), period_s)
        j0 = i - lead_n
        j = max(j0, earliest, s)
        if j >= i:
            stats.n_no_time += 1
            continue
        effort[j:i] = pre if e > 0.0 else -pre
        flag[j:i] = FLAG_PREPOS
        n_pre += i - j
        if j > j0:
            stats.n_short += 1
        else:
            stats.n_placed += 1
    stats.prepos_s = n_pre * period_s
    return stats


def ensure_return_gap(
    effort: np.ndarray,
    flag: np.ndarray,
    *,
    ret: ReturnTime,
    period_s: float,
) -> ReturnGapStats:
    """アクセル⇔ブレーキの全部の切り替えで、すき間（戻す側の指令が 0 の行数）を戻る時間 T 以上にする
    （effort・flag を書き換える。2026-10-17 段2o-9）。

    すき間 = 次のペダルの最初の行 − 戻す側の最後の行 − 1。T は戻す側の最後の指令の深さで決まる
    （`ReturnTime.gap_rows`）。足りないときは左から順に:
    - 戻す側が停車保持（FLAG_STOP）、または前倒しで足りる前に戻す側の連なりが無くなるとき:
      次のペダルの最初の行から 0 に書き換えて遅らせる（停車保持を早く離すとクリープで動き出す）
    - それ以外: 戻す側の最後の行を 1 行ずつ 0（惰行）にして前倒しする（削るたびに深さを測り直す。
      最後の部分はたいてい遊びの中）
    """
    n = len(effort)
    stats = ReturnGapStats(defaults_used=ret.defaults_used())
    last = -1  # 直前の 0 でない行
    i = 0
    while i < n:
        if effort[i] == 0.0:
            i += 1
            continue
        if last >= 0 and (effort[last] > 0.0) != (effort[i] > 0.0):
            stats.n_switches += 1
            gap = i - last - 1
            need = ret.gap_rows(float(effort[last]), period_s)
            if gap < need:
                a = last  # 戻す側の連なり（同じ向きの 0 でない行）の先頭
                leaving_accel = effort[last] > 0.0
                while a > 0 and effort[a - 1] != 0.0 and (effort[a - 1] > 0.0) == leaving_accel:
                    a -= 1
                cut = None
                if flag[last] != FLAG_STOP:
                    for c in range(1, last - a + 1):  # 先頭の 1 行は残す
                        if gap + c >= ret.gap_rows(float(effort[last - c]), period_s):
                            cut = c
                            break
                if cut is not None:
                    effort[last - cut + 1 : last + 1] = 0.0
                    flag[last - cut + 1 : last + 1] = FLAG_COAST
                    stats.n_cut += 1
                    stats.cut_s += cut * period_s
                    last -= cut
                else:
                    k = i  # 次のペダルの最初の行から 0 にして遅らせる
                    sign = effort[i] > 0.0
                    d = 0
                    while (
                        gap + d < need and k < n and effort[k] != 0.0
                        and (effort[k] > 0.0) == sign
                    ):
                        effort[k] = 0.0
                        flag[k] = FLAG_COAST
                        k += 1
                        d += 1
                    stats.n_delayed += 1
                    stats.delayed_s += d * period_s
                    i = k  # 0 にした行の後から数え直す（連なりが尽きたら次の連なりが相手になる）
                    continue
        last = i
        i += 1
    stats.n_violations = count_gap_violations(effort, ret=ret, period_s=period_s)
    return stats


def count_gap_violations(effort: np.ndarray, *, ret: ReturnTime, period_s: float) -> int:
    """アクセル⇔ブレーキの切り替えで、すき間が戻る時間 T に足りない数。"""
    count = 0
    last = -1
    for i in np.flatnonzero(effort != 0.0).tolist():
        if last >= 0 and (effort[last] > 0.0) != (effort[i] > 0.0):
            if i - last - 1 < ret.gap_rows(float(effort[last]), period_s):
                count += 1
        last = i
    return count


def _resolve_releases(
    effort: np.ndarray,
    redo: np.ndarray,
    u_b0: np.ndarray,
    v: np.ndarray,
    need_b: np.ndarray,
    table: ForwardTable,
    brake_deadband_pct: float,
    accel_deadband_pct: float,
    period_s: float,
    window_s: float = RELEASE_WINDOW_S,
) -> None:
    """戻しの表があるとき、redo の時刻のブレーキを時刻順に解き直して effort を書き換える
    （2026-10-14 段2o-5。手順はモジュールの docstring）。u_b0 は戻し量 0 で解いた u。"""
    w = max(int(round(window_s / period_s)), 1)
    # 計画済みの効く開度 u（アクセル＋・ブレーキ−。停車保持の深いブレーキも含む）
    u = np.where(
        effort < -brake_deadband_pct,
        effort + brake_deadband_pct,
        np.where(effort > accel_deadband_pct, effort - accel_deadband_pct, 0.0),
    )
    grid = table.u_grid
    for i in np.flatnonzero(redo):
        m = float(u[max(i - w, 0) : i].min()) if i > 0 else 0.0
        if m >= 0.0 or float(u_b0[i]) <= m:
            continue  # 戻していない（直近の最小が 0 以上）か、戻し量 0 の解がすでに m より深い
        pts = np.concatenate([[m], grid[(grid > m) & (grid < 0.0)], [0.0]])
        f = np.asarray(table.accel(float(v[i]), pts, release=pts - m), dtype=float) - need_b[i]
        hit = np.flatnonzero(f >= 0.0)
        if len(hit) == 0:
            continue  # 完全に離しても要る加速度に届かない: 戻し量 0 の解のまま
        j = int(hit[0])
        if j == 0:
            u_new = m
        else:
            f0, f1 = f[j - 1], f[j]
            frac = 0.0 if f1 == f0 else float(np.clip(-f0 / (f1 - f0), 0.0, 1.0))
            u_new = float(pts[j - 1] + frac * (pts[j] - pts[j - 1]))
        u_new = min(u_new, 0.0)
        u[i] = u_new
        effort[i] = -(brake_deadband_pct + abs(u_new))


def _lag(given: float | None, table: ForwardTable, key: str, default: float) -> float:
    if given is not None:
        return float(given)
    v = table.meta.get(key)
    return float(v) if v is not None and np.isfinite(v) else default


def effective_u_series(
    effort_pct: np.ndarray, flag: np.ndarray, accel_deadband_pct: float, brake_deadband_pct: float
) -> np.ndarray:
    """開度の列（effort）から、不感帯を引いた効く開度 u（アクセル＋・ブレーキ−）を出す。
    不感帯以下・惰行・停車保持は 0。"""
    u = np.zeros(len(effort_pct))
    skip = (flag == FLAG_STOP) | (flag == FLAG_PREPOS)  # 先置きは効き始めの手前（u=0 扱い）
    acc = (effort_pct > accel_deadband_pct) & ~skip
    brk = (effort_pct < -brake_deadband_pct) & ~skip
    u[acc] = effort_pct[acc] - accel_deadband_pct
    u[brk] = effort_pct[brk] + brake_deadband_pct
    return u


def lagged_input(
    table_t: np.ndarray, u: np.ndarray, tau: float, la: float, lb: float
) -> tuple[float, float]:
    """時刻 τ の加速度に効く (u, 遅れ L)。表の定義（時刻 t の u → 時刻 t + L の加速度）の向きに
    合わせて、時刻 τ の加速度には τ − L の u を使う。u(τ − la) >= 0 ならそれ（遅れ la）、
    そうでなければ u(τ − lb) が負ならそれ（遅れ lb）、どちらでもなければ（0, la）。"""
    ia = max(int(np.searchsorted(table_t, tau - la + 1e-9, side="right")) - 1, 0)
    ua = float(u[ia])
    if ua >= 0.0:
        return ua, la
    ib = max(int(np.searchsorted(table_t, tau - lb + 1e-9, side="right")) - 1, 0)
    ub = float(u[ib])
    return (ub, lb) if ub < 0.0 else (0.0, la)


def simulate_speed(
    table: ForwardTable,
    t: np.ndarray,
    u: np.ndarray,
    i0: int,
    i1: int,
    v0: float,
    accel_lag_s: float,
    brake_lag_s: float,
    hold: np.ndarray | None = None,
) -> np.ndarray:
    """表が正しいと仮定して、開度 u から車速を積み上げる（添字 i0..i1-1）。

    v_sim(τ+dt) = v_sim(τ) + accel(v_sim(τ − L), u(τ − L)) × dt（表の定義どおり。
    τ − L が i0 より前なら v0 を使う）。hold が True の点は v_sim を 0 に保つ（停車保持）。
    """
    vs = np.empty(i1 - i0)
    vs[0] = 0.0 if (hold is not None and hold[i0]) else v0
    for k in range(1, i1 - i0):
        i = i0 + k
        if hold is not None and hold[i]:
            vs[k] = 0.0
            continue
        tau = float(t[i - 1])
        uu, lag = lagged_input(t, u, tau, accel_lag_s, brake_lag_s)
        j = max(int(np.searchsorted(t, tau - lag + 1e-9, side="right")) - 1, i0)
        v_lag = vs[j - i0] if j - i0 < k else vs[k - 1]
        vs[k] = max(vs[k - 1] + float(table.accel(v_lag, uu)) * float(t[i] - t[i - 1]), 0.0)
    return vs
