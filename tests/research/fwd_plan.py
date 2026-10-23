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
2026-10-20 段4-3: 停止側の表を引いた時刻も、表が stop_release_accel（停止側の戻しの層）を持つとき
は同じ手順で解き直す（その時刻は停止側の表と層で引く）。アクセルは、表が accel_release_grid を持つ
とき、直近 RELEASE_WINDOW_S 秒の計画済みの u の最大 m からの戻り m − u を戻し量として、同じ
やり方で時刻順に解き直す（`_resolve_accel_releases`）。
2026-10-23 段5-7b: 表が accel_push_grid（アクセルの踏み増しの表）も持つとき、同じ解き直しで、直近の
計画済みの u の最小 n からの増え u − n を踏み増し量として引く（戻し量と踏み増し量を同時に入れて
accel ≧ 要る加速度になる最初の u を探す。戻しの軸だけ・踏み増しの軸だけ・両方のどれでも動く）。
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
from tests.research.fwd_map import (
    LAUNCH_MOVING_KMH,
    POINT_FILLED,
    POINT_MEASURED,
    POINT_OUT,
    ForwardTable,
)

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
# 2026-10-22 段5-2: 行ごとに引いた表（ForwardPlan.side_accel / side_brake。int）。
# 戻しの表（SIDE_RELEASE）は、ブレーキ側で戻し量を見て解き直した行だけに入る
SIDE_MAIN = 0
SIDE_LAUNCH = 1
SIDE_STOP = 2
SIDE_RELEASE = 3
LOW_SPEED_SIDE_RULES = ("legacy", "pedal", "dir")

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
    shift_s: np.ndarray | None = None,
) -> np.ndarray:
    """`pedal_select.window_slope_kmhs` のベクトル版（同じ窓・同じ最小二乗の傾き）。
    shift_s（行ごと。段5-5c）は窓の中心を行ごとにその秒数ずらす。None ＝ ずらさない。"""
    n = max(1, round(width_s / step_s))
    ks = np.arange(n + 1) * step_s
    start = np.asarray(t_s, dtype=float) + center_s - width_s / 2.0
    if shift_s is not None:
        start = start + np.asarray(shift_s, dtype=float)
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
    # 停A（C15。2026-10-20 段4-2）で停車保持へ早めに入れた時刻（停A が無ければ全部 False、
    # None ＝ 持たない）。flag は FLAG_STOP のまま、通常の入る時刻より前の行だけ True
    stop_lead: np.ndarray | None = None
    # 2026-10-22 段5-2: 行ごとに引いた表（SIDE_*。アクセル側・ブレーキ側それぞれ）。
    # deviation_offset_detail・coast_assist_effort の side に渡して、計画と同じ表で P・惰行の補助を
    # 引く。None ＝ 持たない（その場合は low_speed_sides で決める）
    side_accel: np.ndarray | None = None
    side_brake: np.ndarray | None = None
    # 表の引き方の規則（build_plan の low_speed_side_rule）。"legacy" のときは side_for が None を
    # 返し、呼び出し側は今までどおり low_speed_sides で決める（計算が変わらないことを保証するため）
    side_rule: str = "legacy"
    # 2026-10-22 段5-5c: 切り替えの先読み。行ごとに「要る加速度を見る時刻」を追加で早めた秒数
    # （switch_lead=False なら None）
    switch_lead_s: np.ndarray | None = None
    # 2026-10-23 段6 C: 行ごとの「発進の途中」か（dir の規則だけ持つ。それ以外は None）。
    # mode_drive が forward_map.launch_midway_fb_no_deeper で FB のブレーキ踏み増しを抑える行を引く
    launch_midway: np.ndarray | None = None
    # 2026-10-24 段7-b: 発進前のブレーキを基準の動き出しに合わせて離す行（離しの行と、短い停車の
    # bk の行）。stop_lead と同じ持ち方（launch_release_ramp=False なら None）。
    # flag は FLAG_STOP のまま
    launch_prep: np.ndarray | None = None

    def side_for(self, i: int, is_accel: bool) -> int | None:
        """行 i のペダル（アクセル側かブレーキ側か）で計画が引いた表 SIDE_*。
        "pedal"・"dir"（段5-6）の規則の計画だけ値を返す
        （legacy・配列なしは None）。P・惰行の補助・DOB の予測が渡す。"""
        arr = self.side_accel if is_accel else self.side_brake
        if self.side_rule not in ("pedal", "dir") or arr is None:
            return None
        return int(arr[i])

    def is_launch_midway(self, i: int) -> bool:
        """行 i が発進の途中（停車保持の行は除く）か。持たない計画は False。"""
        m = self.launch_midway
        return bool(m is not None and m[i] and self.flag[i] != FLAG_STOP)

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
    stop_ref_at: Callable[[float], float] | None = None,
    stop_lead: bool = False,
    low_speed_side_rule: str = "legacy",
    release_table_min_kmh: float = 0.0,
    switch_lead: bool = False,
    launch_midway_brake_cap: bool = False,
    launch_release_ramp: bool = False,
    launch_release_lead_s: float = 2.0,
    launch_release_pre_pct: float = 2.0,
    launch_short_stop_s: float = 5.0,
    _lead_extra: np.ndarray | None = None,
) -> ForwardPlan:
    """基準車速から開度の列を作る（走る前に全時刻ぶんを解く）。

    2026-10-24 段7-b: `launch_release_ramp`。False は今までと完全に同じ。True は停車からの発進の
    ブレーキを基準の動き出しに合わせて離す（`_launch_release_ramp`）。t0 ＝ 停車の後に基準（停車の
    判定と同じ基準）が停車速度を超える最初の行、t_go ＝ t0 − ブレーキの遅れ、bk ＝ min ＋
    |meta launch_breakaway_u_pct|。停車が launch_short_stop_s 秒より長いなら、t_go −
    launch_release_lead_s に「bk ＋ launch_release_pre_pct（停車保持の開度が浅ければそれまで）」へ、
    t_go まで直線で bk へ。以下なら停車の区間を bk で待つ。どちらも t_go までのブレーキ・惰行の行は
    浅くしない。meta が無ければ何もしない。A1 の後・np.clip の前（先置き・すき間の確保はその後に
    今までどおり掛かる）。

    2026-10-22 段5-5c: `switch_lead`。False は今までと完全に同じ。True は 2 回解く。1 回目は先置き・
    すき間の確保の前の計画（要る加速度から素直に解いた effort。先置き・すき間の確保は切り替えの
    時刻を動かすので、拾うのは動かす前の「要る加速度が決めた切り替え」）。そこから (a) アクセル ⇔
    ブレーキの切り替え、(b) 深いブレーキ（効き始め meta brake_onset_u_pct より深い）を惰行へ戻す所、
    を拾い、出来事の前の行で要る加速度を見る時刻を lead（今のペダルが min へ戻る時間 ＋ 次のペダルの
    遅れ）だけ早める（`switch_lead_extra`）。2 回目はその要る加速度で、通常どおり最後まで解く。

    2026-10-23 段5-6: `low_speed_side_rule="dir"`（表が launch_accel と
    stop_accel の両方を持つときだけ。片方でも無ければ legacy）。
    "legacy"・"pedal" の計算は変えない。低速（基準 < meta
    stop_max_speed_kmh）の引き方を、7.5 km/h 以上と同じ「踏み込み・保持／戻し」の判定にそろえる:
      判定2: アクセル・惰行・ブレーキの判定に使う「ペダルを離した加速度」は、低速は停止側の表
        （`coast_accel(v, stop=True)`）、それ以外は本体。停車 0.5 秒前だけ停止側、の条件は使わない。
      判定3: ブレーキ。低速で発進の途中は発進側。低速で発進の途中でなければ、まず停止側で解き、直近
        RELEASE_WINDOW_S の一番深いブレーキより浅い行は本体の戻しの効き（stop_side=None）で解き直す
        （SIDE_RELEASE。段5-2 の「動き出す開度より深ければ停止側」は使わない）。
        7.5 km/h 以上は今までと同じ。
      判定4: アクセル。低速は発進側、7.5 km/h 以上は本体（pedal と同じ）。
      (ロ) 発進の途中は踏み増さない: 発進の途中の連続区間ごとに、各行のブレーキを「その行から区間の
        終わりまでの最深」にする（ブレーキは戻す向きにしか動かない）。惰行の行も後ろにブレーキがあれば
        そのブレーキになる。値の出どころの行の flag・表（side_brake）を引き継ぐ。

    2026-10-22 段5-3: `release_table_min_kmh`。基準がこの車速未満のブレーキの行は、戻しの表（本体の
    戻し）で解き直さない（本体のまま）。0.0 は今までと完全に同じ。停止側の戻しの層には効かない。

    2026-10-22 段5-2: `low_speed_side_rule`。"legacy" は今までと完全に同じ（`low_speed_sides`）。
    "pedal"（表が launch_accel と stop_accel の両方を持つときだけ。片方でも無ければ legacy）は、
    基準車速が meta stop_max_speed_kmh 未満の行を、ペダルごとに次の表で解く: アクセル＝いつでも
    発進側。ブレーキ＝発進の途中（停車保持を抜けてから、初めてアクセルを使うまで）は発進側、
    それ以外は停止側（need の符号・停車の先読みは問わない）。ただし発進の途中でも、基準が停車速度
    より上で、発進側で解いた u_b の深さが meta launch_breakaway_u_pct（動き出す開度）より深い行は
    停止側で解き直す。この車速では本体・戻しの表は使わない（上の車速は今までと同じ）。

    2026-10-20 段4-2 停A: `stop_lead` が True のとき、停車保持に入る時刻を「ブレーキの遅れ
    （brake_lag_s）先の基準が停車速度以下」に早める（遅れの分だけ早く停車保持の開度を指令する）。
    抜ける時刻（発進側）は変えない: 早めた入りは、基準が上がっていて（発進）その先 stop_horizon_s
    が停車速度を超えるときは入れない。False は今までと完全に同じ。

    2026-10-19 段3: `stop_ref_at` は停車保持・停止側の表の判定（stop・stops_ahead）だけに使う
    基準。`ref_at` をぼかした基準にするとき、停車の始まり・終わりがずれないよう元の基準を渡す。
    None なら `ref_at` と同じ（今までと完全に同じ計算）。

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
    if switch_lead and _lead_extra is None:
        common = {
            "accel_deadband_pct": accel_deadband_pct, "brake_deadband_pct": brake_deadband_pct,
            "stop_brake_opening_pct": stop_brake_opening_pct, "stop_horizon_s": stop_horizon_s,
            "period_s": period_s, "accel_lag_s": accel_lag_s, "brake_lag_s": brake_lag_s,
            "need_window_s": need_window_s, "coast_band_kmhs": coast_band_kmhs,
            "stop_ref_at": stop_ref_at, "stop_lead": stop_lead,
            "low_speed_side_rule": low_speed_side_rule,
            "release_table_min_kmh": release_table_min_kmh,
            "launch_midway_brake_cap": launch_midway_brake_cap,
            "launch_release_ramp": launch_release_ramp,
            "launch_release_lead_s": launch_release_lead_s,
            "launch_release_pre_pct": launch_release_pre_pct,
            "launch_short_stop_s": launch_short_stop_s,
        }
        first = build_plan(
            ref_at, duration_s, table, **common,
            preposition_enabled=False, return_gap_enabled=False,
        )
        extra = switch_lead_extra(
            first, table, la=la, lb=lb, accel_min_pct=accel_deadband_pct,
            brake_min_pct=brake_deadband_pct, need_window_s=need_window_s,
        )
        return build_plan(
            ref_at, duration_s, table, **common,
            preposition_enabled=preposition_enabled, preposition_margin_pct=preposition_margin_pct,
            preposition_lead_s=preposition_lead_s, return_gap_enabled=return_gap_enabled,
            _lead_extra=extra,
        )

    # 1. 停車保持
    ref_now = _eval_ref(ref_at, t)
    if stop_ref_at is None:
        stop_now, ref_stop = ref_now, _eval_ref(ref_at, t + stop_horizon_s)
    else:  # 停車の判定は元の基準（ぼかした基準では判定しない）
        stop_now = _eval_ref(stop_ref_at, t)
        ref_stop = _eval_ref(stop_ref_at, t + stop_horizon_s)
    stop = (stop_now <= STOP_SPEED_KMH) & (ref_stop <= STOP_SPEED_KMH)
    lead_mask = np.zeros(n, dtype=bool)
    if stop_lead:  # 停A: 遅れ lb 先の基準が停車速度以下なら、停車保持に早く入る
        ref_lead = _eval_ref(stop_ref_at if stop_ref_at is not None else ref_at, t + lb)
        # 抜ける側（発進）は変えない: その先 stop_horizon_s の基準が停車速度を超え、かつ今より
        # 上がっている（発進）なら入れない。減速で停車へ向かう間は、その先が今より下がっている
        # ので入れる（stop_horizon_s が遅れより短くても、遅れ分早く入れるため）
        not_launching = (ref_stop <= STOP_SPEED_KMH) | (ref_stop <= stop_now)
        lead_mask = (ref_lead <= STOP_SPEED_KMH) & not_launching & ~stop
        stop = stop | lead_mask

    # 表の定義: 時刻 t の車速 ref(t)・指令 → 時刻 t + L の加速度。だから車速は ref(t)、
    # 要る加速度は t + L を中心にした窓の傾き
    v = ref_now
    coast = np.asarray(table.coast_accel(v), dtype=float)
    need_a = window_slope_vec(ref_at, t, la, need_window_s, shift_s=_lead_extra)
    need_b = window_slope_vec(ref_at, t, lb, need_window_s, shift_s=_lead_extra)
    u_a, kind_a = table.invert(v, need_a)
    u_b, kind_b = table.invert(v, need_b)

    stops_ahead = ref_stop <= STOP_SPEED_KMH
    pedal_rule = (
        low_speed_side_rule == "pedal"
        and table.launch_accel is not None
        and table.stop_accel is not None
    )
    dir_rule = (
        low_speed_side_rule == "dir"
        and table.launch_accel is not None
        and table.stop_accel is not None
    )
    if dir_rule:
        top = float(table.meta.get("stop_max_speed_kmh", 0.0))
        low = v < top
        # 判定2: ペダルを離した加速度。低速は停止側の表（クリープを知っている）、それ以外は本体
        coast_low = np.where(low, np.asarray(table.coast_accel(v, stop=True), dtype=float), coast)
        launch_a, stop_a = low, np.zeros(n, dtype=bool)  # 判定4: 低速のアクセルは発進側
        _, u_a, kind_a = _solve_sides(table, v, need_a, launch_a, stop_a, coast, u_a, kind_a)
        coast_a = coast_b = coast_low
        want_accel = need_a - coast_a >= coast_band_kmhs - 1e-12
        accel_now = want_accel & (u_a > 0.0)
        midway = _launch_midway(stop, accel_now)
        # 判定3: 低速で発進の途中は発進側、そうでなければ停止側（戻しは後で解き直す）
        launch_b = low & midway
        stop_b = low & ~midway
        _, u_b, kind_b = _solve_sides(table, v, need_b, launch_b, stop_b, coast, u_b, kind_b)
    elif not pedal_rule:
        coast_a = coast_b = coast
        launch_a, stop_a = low_speed_sides(table, need_a, v, stops_ahead)
        launch_b, stop_b = low_speed_sides(table, need_b, v, stops_ahead)
        # 発進側（基準が上がっている時刻）・停止側（停車へ向かう減速の時刻）だけ、その表で解く
        coast_a, u_a, kind_a = _solve_sides(
            table, v, need_a, launch_a, stop_a, coast_a, u_a, kind_a
        )
        coast_b, u_b, kind_b = _solve_sides(
            table, v, need_b, launch_b, stop_b, coast_b, u_b, kind_b
        )
        want_accel = need_a - coast_a >= coast_band_kmhs - 1e-12
    else:
        top = float(table.meta.get("stop_max_speed_kmh", 0.0))
        low = v < top
        # アクセル側: 低速はいつでも発進側。判定（want_accel・is_accel）はアクセル側の値だけ
        launch_a, stop_a = low, np.zeros(n, dtype=bool)
        coast_a, u_a, kind_a = _solve_sides(table, v, need_a, launch_a, stop_a, coast, u_a, kind_a)
        want_accel = need_a - coast_a >= coast_band_kmhs - 1e-12
        accel_now = want_accel & (u_a > 0.0)
        midway = _launch_midway(stop, accel_now)
        # ブレーキ側: 発進の途中は発進側、それ以外は停止側。発進の途中でも、動き出す開度より深い
        # 行（基準が LAUNCH_MOVING_KMH より上）は停止側で解き直す
        launch_b = low & midway
        stop_b = low & ~midway
        breakaway = table.meta.get("launch_breakaway_u_pct")
        if launch_b.any() and breakaway is not None and np.isfinite(breakaway):
            u_b_l, _ = table.invert(v, need_b, launch=True)
            # 実機の pkl の meta は符号つき（ブレーキ側 ＝ 負）なので、大きさで比べる
            deep = (
                launch_b & (v > LAUNCH_MOVING_KMH)
                & (np.abs(np.asarray(u_b_l)) > abs(float(breakaway)))
            )
            launch_b, stop_b = launch_b & ~deep, stop_b | deep
        coast_b, u_b, kind_b = _solve_sides(table, v, need_b, launch_b, stop_b, coast, u_b, kind_b)

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
    released = np.zeros(n, dtype=bool)  # ブレーキで戻しの表（本体）に解き直した行
    body_release = table.release_grid is not None and table.release_accel is not None
    # 2026-10-20 段4-3: 停止側の表で解いた時刻も、停止側の戻しの層があれば戻し量を見て解き直す
    stop_release = table.stop_release_accel is not None and table.stop_release_grid is not None
    if dir_rule:
        # 段5-6: 低速で発進の途中でない行（停止側で解いた行）も、7.5 km/h 以上と同じく本体の戻しの
        # （stop_side=None）で解き直す。停止側の戻しの層は使わない
        if body_release:
            body_redo = is_brake & ~stop & ~launch_b
            if release_table_min_kmh > 0.0:  # 段5-3: 低速は dir が決める
                body_redo &= (v >= release_table_min_kmh) | low
            released = _resolve_releases(
                effort, body_redo, u_b, v, need_b, table, brake_deadband_pct, accel_deadband_pct,
                period_s, stop_side=None,
            )
    elif body_release or stop_release:
        redo = np.zeros(n, dtype=bool)
        if body_release:
            # 発進側の表で解いた時刻は戻しではない
            body_redo = is_brake & ~stop & ~(launch_b | stop_b)
            if release_table_min_kmh > 0.0:  # 段5-3: 低い車速は戻しの表で解き直さない
                body_redo &= v >= release_table_min_kmh
            redo |= body_redo
        if stop_release:
            redo |= is_brake & ~stop & stop_b & ~launch_b
        released = _resolve_releases(
            effort, redo, u_b, v, need_b, table, brake_deadband_pct, accel_deadband_pct, period_s,
            stop_side=stop_b,
        )
    relabel_a = np.zeros(n, dtype=bool)
    accel_rel_on = table.accel_release_grid is not None and table.accel_release_accel is not None
    accel_push_on = table.accel_push_grid is not None and table.accel_push_accel is not None
    if accel_rel_on or accel_push_on:
        # 2026-10-23 段5-7b: アクセルの踏み増しの表だけでも同じ解き直しに入る
        # 2026-10-20 段4-3: アクセルの戻し量（直近の u の最大からの戻り）を見て解き直す。
        # 2026-10-23 段5-7a: 外すのは低速（stop_max_speed_kmh 未満）で発進側の表を引いた時刻と停止側
        # の時刻だけ。それ以上の車速では発進側の表は本体と同じ値なので、legacy でも解き直す
        low_a = v < float(table.meta.get("stop_max_speed_kmh", 0.0))
        redo_a = is_accel & ~stop & ~((launch_a & low_a) | stop_a)
        _resolve_accel_releases(
            effort, redo_a, u_a, v, need_a, table, brake_deadband_pct, accel_deadband_pct, period_s
        )
        # 解き直した行（7.5 km/h 以上の legacy の発進側）は本体の表なので、ラベルも本体にする
        relabel_a = redo_a & launch_a
    side_brake_dir = None
    if dir_rule:
        # 戻しで解き直した行（低速の停止側だった行を含む）は SIDE_RELEASE、それ以外は上の表のまま
        side_brake_dir = np.where(
            launch_b, SIDE_LAUNCH,
            np.where(released, SIDE_RELEASE, np.where(stop_b, SIDE_STOP, SIDE_MAIN)),
        )
        # (ロ) 発進の途中は踏み増さない（区間ごとに後ろからの最深に置き換える）
        _no_deeper_brake_in_midway(effort, flag, side_brake_dir, midway)
        if launch_midway_brake_cap:  # 段5-6 追加 A1: (ロ) の後。浅くする向きだけ
            _cap_midway_brake(effort, flag, midway, brake_deadband_pct, table.meta)
    launch_prep = None
    if launch_release_ramp:  # 段7-b: A1 の後・clip の前
        launch_prep = np.zeros(n, dtype=bool)
        bkm = table.meta.get("launch_breakaway_u_pct")
        if bkm is not None and np.isfinite(bkm):
            launch_prep = _launch_release_ramp(
                effort, flag, stop_now, period_s=period_s, brake_lag_s=lb,
                bk_pct=brake_deadband_pct + abs(float(bkm)),
                pre_pct=launch_release_pre_pct, lead_s=launch_release_lead_s,
                short_stop_s=launch_short_stop_s, hold_pct=stop_brake_opening_pct,
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
    side_accel = np.where(launch_a, SIDE_LAUNCH, np.where(stop_a, SIDE_STOP, SIDE_MAIN))
    side_accel = np.where(relabel_a, SIDE_MAIN, side_accel)
    side_brake = np.where(launch_b, SIDE_LAUNCH, np.where(stop_b, SIDE_STOP, SIDE_MAIN))
    side_brake = np.where(released & ~stop_b & ~launch_b, SIDE_RELEASE, side_brake)
    if side_brake_dir is not None:
        side_brake = side_brake_dir
    return ForwardPlan(
        t_s=t, effort_pct=effort, need_accel_kmhs=need, flag=flag, period_s=period_s,
        preposition=stats, return_gap=gap_stats, stops_ahead=stops_ahead,
        stop_lead=lead_mask, side_accel=side_accel.astype(int), side_brake=side_brake.astype(int),
        side_rule="dir" if dir_rule else ("pedal" if pedal_rule else "legacy"),
        switch_lead_s=_lead_extra,
        launch_midway=np.asarray(midway, dtype=bool).copy() if dir_rule else None,
        launch_prep=launch_prep,
    )


def _launch_release_ramp(
    effort: np.ndarray,
    flag: np.ndarray,
    stop_ref: np.ndarray,
    *,
    period_s: float,
    brake_lag_s: float,
    bk_pct: float,
    pre_pct: float,
    lead_s: float,
    short_stop_s: float,
    hold_pct: float,
) -> np.ndarray:
    """段7-b: 停車からの発進のブレーキを基準の動き出しに合わせて離す（effort・flag をその場で
    書き換え、書き換えた離しの行・短い停車の bk の行の bool 配列を返す）。

    停車 ＝ stop_ref が停車速度以下の連なり（系列の先頭からの停車も含む）。t0 ＝ その直後の最初の
    行、t_go ＝ t0 − brake_lag_s。bk_pct は動き出す開度（絶対値。min ＋ |meta|）。ブレーキの effort
    は負。
    - 停車が short_stop_s 秒より長い: 停車保持の開度（hold）で待ち、t_go − lead_s の行で
      min(hold, bk ＋ pre_pct) → t_go の行で bk まで直線。停車保持の行がこの窓の中なら、この直線に
      置き換える。窓の中のブレーキ・惰行の行はこの値より浅くしない（アクセルの行は触らない）。
      停車保持の後ろから窓の始めまでのブレーキ・惰行の行は hold で保つ（型B の早い戻しを止める）
    - 短い: 停車保持の行（停A の早めた行を含む）と、t_go までのブレーキ・惰行の行を bk より
      浅くしない
      （停車保持の行は bk にする。直線は作らない）
    惰行の行を深くしたときは FLAG_STOP にする（FB が効かない行）。ブレーキの行の flag はそのまま。
    """
    n = len(effort)
    prep = np.zeros(n, dtype=bool)
    low = stop_ref <= STOP_SPEED_KMH
    lead_n = max(int(round(lead_s / period_s)), 1)
    prev_end = 0
    i = 0

    def deepen(r: int, depth: float, *, exact: bool) -> None:
        """行 r を深さ depth のブレーキにする（exact なら浅くなっても置き換える）。"""
        if effort[r] > 0.0:
            return  # アクセルの行は触らない
        if exact or -effort[r] < depth:
            effort[r] = -depth
        if flag[r] == FLAG_COAST:
            flag[r] = FLAG_STOP
        prep[r] = True

    while i < n:
        if not low[i]:
            i += 1
            continue
        s = i
        while i < n and low[i]:
            i += 1
        t0 = i
        if t0 >= n:
            break
        length_s = (t0 - s) * period_s
        i_go = int(np.clip(round(t0 - brake_lag_s / period_s), 0, t0))
        # 停車保持の連なり（停A の早めた行を含む）。t0 の手前で最後の FLAG_STOP の行から前へ
        j_end = t0 - 1
        while j_end >= prev_end and flag[j_end] != FLAG_STOP:
            j_end -= 1
        b0 = j_end
        if j_end >= prev_end:
            while b0 - 1 >= prev_end and flag[b0 - 1] == FLAG_STOP:
                b0 -= 1
        has_hold = j_end >= prev_end
        # アクセルが t_go より前に始まる計画（窓が基準の立ち上がりを先に拾う）では、その手前までで
        # 止める（ブレーキと同時踏みにしない）
        acc = np.flatnonzero((effort[s:] > 0.0) & (flag[s:] != FLAG_STOP))
        i_end = min(i_go, s + int(acc[0]) - 1) if len(acc) else i_go
        if length_s > short_stop_s + 1e-9:
            hold = -float(effort[j_end]) if has_hold else hold_pct
            pre = min(hold, bk_pct + pre_pct)
            i_r = max(i_go - lead_n, 0)
            span = max(i_go - i_r, 1)
            if has_hold:  # 停車保持の後ろから窓の始めまで hold で保つ
                for r in range(j_end + 1, min(i_r, i_end + 1)):
                    deepen(r, hold, exact=False)
            for r in range(i_r, min(i_end, n - 1) + 1):
                val = pre + (bk_pct - pre) * (r - i_r) / span
                stop_row = flag[r] == FLAG_STOP
                deepen(r, val, exact=bool(stop_row))
        else:
            lo = b0 if has_hold else s
            for r in range(lo, min(i_end, n - 1) + 1):
                deepen(r, bk_pct, exact=bool(flag[r] == FLAG_STOP))
        prev_end = t0
    return prep



def switch_lead_events(
    plan: ForwardPlan,
    table: ForwardTable,
    *,
    la: float,
    lb: float,
    accel_min_pct: float,
    brake_min_pct: float,
) -> list[tuple[int, float, str]]:
    """切り替えの先読み（段5-5c）の出来事 [(行 k, lead [s], 種類 "switch"/"release")]。

    switch: 停車保持・先置きを除いたアクセル ⇔ ブレーキの切り替え。k ＝ 切り替え先のペダルが最初に
    出た行。lead ＝ 今のペダルが min へ戻る時間（meta の *_return_delay_s ＋ 戻す直前の深さ ÷
    *_return_rate_pct_s）＋ 切り替え先のペダルの遅れ。
    release: ブレーキが続いたあと惰行（停車保持でない 0）へ戻る行 k で、戻す直前の深さ
    （min からの量）が meta brake_onset_u_pct（大きさ）より深いもの。
    lead ＝ ブレーキの戻る時間（次のペダルは無いので
    遅れは 0）。meta brake_onset_u_pct が無ければ release は拾わない。
    """
    ret = return_time_of(table, accel_min_pct, brake_min_pct)
    onset = table.meta.get("brake_onset_u_pct")
    onset_ok = onset is not None and np.isfinite(onset)
    eff = plan.effort_pct
    stop = (plan.flag == FLAG_STOP) | (plan.flag == FLAG_PREPOS)
    out: list[tuple[int, float, str]] = []

    def back_s(i: int) -> float:  # 行 i のペダルが min へ戻る時間（周期の余裕は入れない）
        if eff[i] > 0.0:
            depth = max(float(eff[i]) - accel_min_pct, 0.0)
            return ret.accel_delay_s + depth / ret.accel_rate_pct_s
        depth = max(-float(eff[i]) - brake_min_pct, 0.0)
        return ret.brake_delay_s + depth / ret.brake_rate_pct_s

    last_i: int | None = None  # 直前のペダルの最後の行
    for k in range(len(eff)):
        if stop[k]:
            last_i = None  # 停車保持への出入りをまたぐ切り替えは対象外
            continue
        if eff[k] == 0.0:
            continue
        if last_i is not None and (eff[k] > 0.0) != (eff[last_i] > 0.0):
            out.append((k, back_s(last_i) + (la if eff[k] > 0.0 else lb), "switch"))
        last_i = k
    for k in range(1, len(eff)):
        if stop[k] or eff[k] != 0.0 or stop[k - 1] or eff[k - 1] >= 0.0 or not onset_ok:
            continue
        if -float(eff[k - 1]) - brake_min_pct > abs(float(onset)):
            out.append((k, back_s(k - 1), "release"))
    return out


def switch_lead_extra(
    plan: ForwardPlan,
    table: ForwardTable,
    *,
    la: float,
    lb: float,
    accel_min_pct: float,
    brake_min_pct: float,
    need_window_s: float,
) -> np.ndarray:
    """行ごとの追加の先読み extra[i]。出来事 k ごとに行 [k − ceil(lead/周期) − round(窓/2/周期), k)
    を lead にする（重なる所は大きい方）。それ以外は 0。"""
    p = plan.period_s
    extra = np.zeros(len(plan.t_s))
    for k, lead, _kind in switch_lead_events(
        plan, table, la=la, lb=lb, accel_min_pct=accel_min_pct, brake_min_pct=brake_min_pct
    ):
        lo = max(k - math.ceil(lead / p - 1e-9) - round(need_window_s / 2.0 / p), 0)
        extra[lo:k] = np.maximum(extra[lo:k], lead)
    return extra


def _solve_sides(
    table: ForwardTable,
    v: np.ndarray,
    need: np.ndarray,
    launch: np.ndarray,
    stop: np.ndarray,
    coast: np.ndarray,
    u: np.ndarray,
    kind: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """発進側・停止側の行だけ、その表で惰行の加速度 c と逆引き u を置き換える（残りは本体のまま）。
    戻り値は (coast, u, kind)。launch・stop が全部 False なら入力をそのまま返す。"""
    for use_launch, side in ((True, launch), (False, stop)):
        if not side.any():
            continue
        kw = {"launch": True} if use_launch else {"stop": True}
        coast_s = np.asarray(table.coast_accel(v, **kw), dtype=float)
        u_s, kind_s = table.invert(v, need, **kw)
        coast = np.where(side, coast_s, coast)
        u, kind = np.where(side, u_s, u), np.where(side, kind_s, kind)
    return coast, u, kind


# 段5-2: LAUNCH_MOVING_KMH（0.5 km/h）は fwd_map で定義（ProblemReport 段5 の表を引く条件の ①② の
# 境目）。停車保持の判定の STOP_SPEED_KMH（0.02）とは別。これ以下（① まだ止まっている所）は
# 動き出す開度より深くても発進側のまま


def _launch_midway(stop: np.ndarray, accel_now: np.ndarray) -> np.ndarray:
    """発進の途中の行（段5-2）: 停車保持を抜けるたびに、抜けた最初の行から、初めてアクセルを使う
    行（またはその前に停車保持に戻る行）の直前まで。アクセルが出ないときは最後まで。"""
    n = len(stop)
    out = np.zeros(n, dtype=bool)
    if n == 0:
        return out
    starts = np.flatnonzero(stop[:-1] & ~stop[1:]) + 1
    ends = np.flatnonzero(accel_now | stop)
    for i0 in starts.tolist():
        k = np.searchsorted(ends, i0)
        j = n if k >= len(ends) else int(ends[k])
        out[i0:j] = True
    return out


def _no_deeper_brake_in_midway(
    effort: np.ndarray, flag: np.ndarray, side_brake: np.ndarray, midway: np.ndarray
) -> None:
    """(ロ)（段5-6）発進の途中の連続区間ごとに、ブレーキを「その行から区間の終わりまでの最深」にする
    （effort・flag・side_brake をその場で書き換える）。区間の中でブレーキは戻す向きにしか動かない。
    惰行（0）の行も、後ろにブレーキがあればそのブレーキになる。flag・side_brake は値の出どころの行
    のものを引き継ぐ。アクセルの行（effort > 0）と停車保持の行は触らない。"""
    n = len(effort)
    i = n - 1
    while i >= 0:
        if not midway[i]:
            i -= 1
            continue
        best_e, best_j = 0.0, -1  # 後ろ（区間の終わりまで）の最深のブレーキ（負）とその行
        while i >= 0 and midway[i]:
            if flag[i] != FLAG_STOP and effort[i] <= 0.0:
                if effort[i] < best_e:
                    best_e, best_j = float(effort[i]), i
                elif best_j >= 0:  # 自分より後ろに深いブレーキがある → それに置き換える
                    effort[i], flag[i], side_brake[i] = best_e, flag[best_j], side_brake[best_j]
            i -= 1


def _cap_midway_brake(
    effort: np.ndarray, flag: np.ndarray, midway: np.ndarray, brake_deadband_pct: float,
    meta: dict[str, Any],
) -> None:
    """段5-6 追加 A1: 発進の途中の行のブレーキを、動き出す開度（meta launch_breakaway_u_pct の
    大きさ。min からの量）に min を足した深さで頭打ちにする（effort をその場で書き換える。
    浅くする向きだけ。停車保持・アクセル・惰行の行は触らない）。meta が無い／非有限なら何もしない。
    余裕は足さない。"""
    bk = meta.get("launch_breakaway_u_pct")
    if bk is None or not np.isfinite(bk):
        return
    cap = -(brake_deadband_pct + abs(float(bk)))
    hit = midway & (flag != FLAG_STOP) & (effort < cap)
    effort[hit] = cap


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


def _side_flags(
    table: ForwardTable,
    side: int | None,
    need_kmhs: float,
    ref_kmh: float,
    stops_ahead: bool,
) -> tuple[bool, bool]:
    """表の引き方 (launch, stop)。side があればそれ（SIDE_LAUNCH / SIDE_STOP / それ以外は本体）、
    None なら `low_speed_sides`（今までと同じ）。"""
    if side is None:
        lv, sv = low_speed_sides(table, need_kmhs, ref_kmh, stops_ahead)
        return bool(lv), bool(sv)
    return side == SIDE_LAUNCH, side == SIDE_STOP


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
    side: int | None = None,
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
    - side（2026-10-22 段5-2）: 計画がその行のペダルで引いた表（ForwardPlan.side_accel /
      side_brake。SIDE_LAUNCH ＝ 発進側・SIDE_STOP ＝ 停止側・それ以外 ＝ 本体）。None なら上の
      `low_speed_sides`
      で決める（今までと同じ）。
    """
    if flag in (FLAG_COAST, FLAG_STOP, FLAG_PREPOS) or effort == 0.0:
        return effort, False
    is_accel = effort > 0.0
    tau = tau_accel_s if is_accel else tau_brake_s
    if tau is None or (tau <= 0.0 and dob_kmhs == 0.0):
        return effort, False
    launch, stop = _side_flags(table, side, need_kmhs, ref_kmh, stops_ahead)
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


def coast_assist_effort(
    table: ForwardTable,
    *,
    is_accel: bool,
    need_kmhs: float,
    ref_kmh: float,
    row_kmh: float,
    dev_kmh: float,
    tau_s: float,
    accel_min_pct: float,
    brake_min_pct: float,
    stops_ahead: bool = False,
    dob_kmhs: float = 0.0,
    onset_u_pct: float | None = None,
    side: int | None = None,
) -> float:
    """C12: 惰行の行で補助に使うペダルの effort [%] を返す（段2r。正＝アクセル・負＝ブレーキ）。

    惰行の行は計画が 0 なので、`deviation_offset_detail` のような「列に足す」ことができない。
    代わりに、偏差 dev を τ 秒で 0 に戻す加速度を要る加速度に足して、表を直接逆に引く:
        u1 ＝ invert(row_kmh, need ＋ dev ÷ τ − dob)
    アクセル（遅れている側）なら effort ＝ accel_min ＋ max(u1, 0)、ブレーキなら
    effort ＝ −(brake_min ＋ max(−u1, 0))。u1 が反対の符号なら min で止め、反対のペダルには
    移らない。u と effort の関係（effort ＝ ±(min ＋ |u|)）は build_plan と同じ。
    τ ≦ 0 なら偏差の項は 0。表の引き方（発進側・停止側）は deviation_offset_detail と同じ
    （`low_speed_sides`。ref_kmh と stops_ahead で決める）。row_kmh は表を引く車速。
    onset_u_pct（そのペダルの効き始め。min からの量）を渡すと、u の大きさをそれ以上にする
    （段2r ②。遊びの中に入って効かないのを防ぐ。逆の符号で min に止めた場合も効き始めに置く）。
    None なら下限なし。
    side（2026-10-22 段5-2）: 計画がその行のペダルで引いた表（SIDE_*）。None なら
    `low_speed_sides`。
    """
    launch, stop = _side_flags(table, side, need_kmhs, ref_kmh, stops_ahead)
    d_a = dev_kmh / tau_s if tau_s > 0.0 else 0.0
    u1, _ = table.invert(row_kmh, need_kmhs + d_a - dob_kmhs, launch=launch, stop=stop)
    u = float(u1)
    floor = 0.0 if onset_u_pct is None or not np.isfinite(onset_u_pct) else float(onset_u_pct)
    if is_accel:
        effort = accel_min_pct + max(u, floor, 0.0)
    else:
        effort = -(brake_min_pct + max(-u, floor, 0.0))
    return float(np.clip(effort, -EFFORT_LIMIT_PCT, EFFORT_LIMIT_PCT))


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
    stop_side: np.ndarray | None = None,
) -> np.ndarray:
    """戻しの表があるとき、redo の時刻のブレーキを時刻順に解き直して effort を書き換える
    （2026-10-14 段2o-5。手順はモジュールの docstring）。u_b0 は戻し量 0 で解いた u。
    stop_side（2026-10-20 段4-3）: True の時刻は停止側の表（と停止側の戻しの層）で引く。
    None なら全部本体（今までと同じ）。
    戻り値（段5-2）: 解き直して effort を書いた行の bool 配列。"""
    changed = np.zeros(len(effort), dtype=bool)
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
        on_stop = stop_side is not None and bool(stop_side[i])
        f = (
            np.asarray(table.accel(float(v[i]), pts, release=pts - m, stop=on_stop), dtype=float)
            - need_b[i]
        )
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
        changed[i] = True
    return changed


def _resolve_accel_releases(
    effort: np.ndarray,
    redo: np.ndarray,
    u_a0: np.ndarray,
    v: np.ndarray,
    need_a: np.ndarray,
    table: ForwardTable,
    brake_deadband_pct: float,
    accel_deadband_pct: float,
    period_s: float,
    window_s: float = RELEASE_WINDOW_S,
) -> None:
    """アクセルの戻しの表・踏み増しの表があるとき、redo の時刻のアクセルを時刻順に解き直して
    effort を書き換える（2026-10-20 段4-3。`_resolve_releases` のアクセル版。2026-10-23 段5-7b で
    踏み増しの軸を追加）。u_a0 は戻し量・踏み増し量 0 で解いた u。

    時刻 i の戻し量 ＝ 直近 window_s 秒（i の手前まで）の計画済み u の最大 mx − 新しい u（0 以上）、
    踏み増し量 ＝ 新しい u − 直近の u の最小 mn（0 以上）。
    解き直す条件: 戻しの表があり mx > 0 かつ u_a0 < mx（戻している）、または踏み増しの表があり
    u_a0 > mn（踏み増している。i = 0 は除く）。どちらでもなければそのまま。
    解き直すときは u を 0 から上限まで動かし（上限 ＝ 戻しの軸があれば mx、踏み増しの軸があれば
    u_a0 のうち大きい方）、accel(v, u, accel_release=…, accel_push=…) ≧ need に最初になる u
    （節の間は直線補間）を新しい u にする。届かないとき（上限でも届かない）は戻し量・踏み増し量 0 の
    解のまま。u = 0 で届くときもそのまま（アクセルの行を 0 にしない）。
    """
    rel_on = table.accel_release_grid is not None and table.accel_release_accel is not None
    push_on = table.accel_push_grid is not None and table.accel_push_accel is not None
    w = max(int(round(window_s / period_s)), 1)
    u = np.where(
        effort < -brake_deadband_pct,
        effort + brake_deadband_pct,
        np.where(effort > accel_deadband_pct, effort - accel_deadband_pct, 0.0),
    )
    grid = table.u_grid
    for i in np.flatnonzero(redo):
        win = u[max(i - w, 0) : i]
        mx = float(win.max()) if i > 0 else 0.0
        mn = float(np.maximum(win, 0.0).min()) if i > 0 else 0.0
        u0 = float(u_a0[i])
        do_rel = rel_on and mx > 0.0 and u0 < mx
        do_push = push_on and i > 0 and u0 > mn + 1e-9
        if not (do_rel or do_push):
            continue  # 戻していない・踏み増していない
        top = max(mx if do_rel else 0.0, u0 if do_push else 0.0)
        if top <= 0.0:
            continue
        kinks = [0.0, top]
        if rel_on and 0.0 < mx < top:
            kinks.append(mx)  # 戻し量 max(mx − u, 0) の折れ点
        if push_on and 0.0 < mn < top:
            kinks.append(mn)  # 踏み増し量 max(u − mn, 0) の折れ点
        pts = np.unique(np.concatenate([kinks, grid[(grid > 0.0) & (grid < top)]]))
        f = (
            np.asarray(
                table.accel(
                    float(v[i]),
                    pts,
                    accel_release=np.maximum(mx - pts, 0.0) if rel_on else None,
                    accel_push=np.maximum(pts - mn, 0.0) if push_on else None,
                ),
                dtype=float,
            )
            - need_a[i]
        )
        hit = np.flatnonzero(f >= 0.0)
        if len(hit) == 0:
            continue  # 戻し量・踏み増し量 0 の解のまま
        j = int(hit[0])
        if j == 0:
            continue  # u = 0 でも届く: 戻し量 0 の解のまま（アクセルの行を 0 にしない）
        f0, f1 = f[j - 1], f[j]
        frac = 0.0 if f1 == f0 else float(np.clip(-f0 / (f1 - f0), 0.0, 1.0))
        u_new = float(pts[j - 1] + frac * (pts[j] - pts[j - 1]))
        u_new = max(u_new, 0.0)
        u[i] = u_new
        effort[i] = accel_deadband_pct + u_new


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
