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
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, replace
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


# 2026-10-26 段10-1: ペダルの実際の位置を追う部品（ProblemReport_20261026）。計画は「指令した開度に
# ペダルがすぐ着く」と見ていたが、実際は速さの上限と加速度の上限で動く（短い移動は平均が遅く、
# 長い移動は上限の速さで頭打ち）。値はデータ由来: 段10-1 の 24 件（AMA 谷 6 か所 × 2 本・
# US06 山 2 か所 × 6 本）の当てはめ（stage10_pedal_track.py。位置の RMSE アクセル 0.67 → 0.38 %、
# ブレーキ 0.41 → 0.28 %）。delay は指令 → 位置の遅れ（位置は 1 周期遅れて指令へ動くので、実効は
# delay ＋ 1 周期 ≒ 0.07 s）、effect_lag は位置 → 車の加速度に効くまで
DEFAULT_PEDAL_ACCEL_DELAY_S = 0.02
DEFAULT_PEDAL_ACCEL_MAX_RATE_PCT_S = 100.0
DEFAULT_PEDAL_ACCEL_MAX_ACCEL_PCT_S2 = 2000.0
DEFAULT_PEDAL_BRAKE_DELAY_S = 0.02
DEFAULT_PEDAL_BRAKE_MAX_RATE_PCT_S = 90.0
DEFAULT_PEDAL_BRAKE_MAX_ACCEL_PCT_S2 = 1400.0
DEFAULT_PEDAL_ACCEL_EFFECT_LAG_S = 0.10
DEFAULT_PEDAL_BRAKE_EFFECT_LAG_S = 0.10
# min に着いたとみなす幅。mode_drive の INTERLOCK_TOL_PCT（同時踏み防止の判定）と同じ値
POSITION_RETURN_TOL_PCT = 0.05
# 2026-10-26 段10-2': R の谷の上乗せ（7-c）を配る行数の下限 [s] と、倍率 f の上限
VALLEY_BOOST_MIN_S = 0.3
VALLEY_BOOST_F_MAX = 4.0
VALLEY_BOOST_ITERS = 20
# 2026-10-26 段10-2': R の山で戻し始める行 s を探す範囲 [秒]。アクセルの連なりの最後（last + 1）から
# 遡る長さ。None なら連なりの頭まで全部。長い連なり（US06 は 32 秒）で頭まで探すと、積み上がりの
# 面積に連なり全体の水準のずれ（A1）が入り、それを打ち消す方へ s が動いてしまうため、
# 変わり目の近くに限る。
# 2026-10-26 段11-4: これは仮の値（データ由来でない）。switch_gap_peak_from_corner が true のときは
# 使わず、基準の加速度が頂点へ下がり始める行から下限を決める（`_rule_peak`）。false なら今まで通り
RULE_PEAK_RANGE_S: float | None = 2.0

# 2026-10-27 段12: switch_gap_peak_robust が true のとき、`_rule_peak` の採点で
# 「ペダルが効くまでの遅れ」をこの行数だけ前後にずらした 3 通り（−1・0・+1 行）の最大を取る。
# 仮の値（モードの走りから。2026-10-27 US06 13 s の実測 154602・081033 で当てはまりのよい遅れが
# 0.075〜0.10 s と 1 行ぶれた）。手順2（151009）で動いた所ごとに当てはめると、遅れが決まったのは
# アクセル 8・ブレーキ 11 か所で 25〜75% が 0.119〜0.175 s・0.113〜0.200 s＝±1 行と食い違わない
# （ユーザー決定 2026-10-27: ±1 行のまま）。
# いつ決め直すか: 急な踏み替えを含む手順2 の走りができたら。
# 山（_rule_peak）と谷（_valley_boost_gain_robust）で使う
PEAK_ROBUST_LAG_ROWS: int = 1
# 2026-10-27 段12 谷: switch_gap_valley_robust が true のとき、谷の倍率 f をまず粗くこの刻みで
# 全部採点し、最小の格子点のまわりを黄金分割探索で詰める。
# 仮の値（根拠なし）。粗く見る刻み。2026-10-27 机上で AMA 12・US06 16・UDDS 9 の谷のうち点数の極小が
# 2 つあったのは US06 27.7 s の 1 件（f 0 と 2.1）だけ＝0.1 刻みで取り違えない。
# 細かさは黄金分割で詰める
VALLEY_ROBUST_GRID_F: float = 0.1

# 位置を追うときの 1 周期の分割数（加速度の上限つきの動きを細かく積分する）
PEDAL_TRACK_SUBSTEPS = 10
# 着いたとみなす位置の幅 [%]（着いたら止める。min のわずか上に残ると「踏んでいる」扱いになるため）
PEDAL_TRACK_ARRIVE_TOL_PCT = 0.05


@dataclass(frozen=True)
class PedalTrack:
    """ペダルの実際の位置の動き方（2026-10-26 段10-1）。既定値はデータ由来（上の定数）。
    速さの上限 max_rate [%/s] と加速度の上限 max_accel [%/s^2] で動く。"""

    accel_delay_s: float = DEFAULT_PEDAL_ACCEL_DELAY_S
    accel_max_rate_pct_s: float = DEFAULT_PEDAL_ACCEL_MAX_RATE_PCT_S
    accel_max_accel_pct_s2: float = DEFAULT_PEDAL_ACCEL_MAX_ACCEL_PCT_S2
    brake_delay_s: float = DEFAULT_PEDAL_BRAKE_DELAY_S
    brake_max_rate_pct_s: float = DEFAULT_PEDAL_BRAKE_MAX_RATE_PCT_S
    brake_max_accel_pct_s2: float = DEFAULT_PEDAL_BRAKE_MAX_ACCEL_PCT_S2
    accel_effect_lag_s: float = DEFAULT_PEDAL_ACCEL_EFFECT_LAG_S
    brake_effect_lag_s: float = DEFAULT_PEDAL_BRAKE_EFFECT_LAG_S


def _pedal_cmd(cmd: np.ndarray, delay_s: float, mn: float, period_s: float) -> np.ndarray:
    """指令の列を delay だけ遅らせ（直線補間）、min で下限にする。"""
    idx = np.arange(len(cmd), dtype=float)
    return np.maximum(np.interp(idx - delay_s / period_s, idx, cmd), mn)


def _follow_core(
    c: np.ndarray, pos: np.ndarray, vlog: np.ndarray, k0: int, x: float, v: float,
    period_s: float, vmax: float, amax: float,
) -> None:
    """行 k0 以降の位置 pos[k]（と速さ vlog[k]）を書く。x・v ＝ 行 k0 − 1 の位置・速さ。
    行 k の位置は、行 k − 1 の（遅らせた）指令 c[k − 1] へ 1 周期動いた後。動きは 1 周期を
    PEDAL_TRACK_SUBSTEPS 個に分け、止まれる速さ min(vmax, sqrt(2・amax・|差|)) へ加速度の上限 amax
    で近づく。着いたら（差が 0.05 % 未満で速さも小さい）止める。"""
    h = period_s / PEDAL_TRACK_SUBSTEPS
    dvmax = amax * h
    v_stop = 2.0 * amax * h
    sqrt = math.sqrt
    cl = c.tolist()
    for k in range(k0, len(c)):
        tgt = cl[k - 1]
        if v != 0.0 or x != tgt:  # 静止して着いている行は動かない
            for _ in range(PEDAL_TRACK_SUBSTEPS):
                e = tgt - x
                vd = min(vmax, sqrt(2.0 * amax * abs(e)))
                if e < 0.0:
                    vd = -vd
                dv = vd - v
                v += -dvmax if dv < -dvmax else (dvmax if dv > dvmax else dv)
                x += v * h
                if abs(tgt - x) < PEDAL_TRACK_ARRIVE_TOL_PCT and abs(v) < v_stop:
                    x = tgt
                    v = 0.0
        pos[k] = x
        vlog[k] = v


def _follow_pedal(
    cmd: np.ndarray, p0: float | None, delay_s: float, vmax: float, amax: float, mn: float,
    period_s: float, *, vlog: np.ndarray | None = None,
) -> np.ndarray:
    """指令の列 → 位置の列（`stage10_pedal_track` の当てはめと同じ）。指令は delay だけ遅らせ
    （直線補間）、min で下限。動き方は `_follow_core`。vlog を渡すと各行の速さも書く。"""
    n = len(cmd)
    c = _pedal_cmd(cmd, delay_s, mn, period_s)
    pos = np.empty(n)
    pos[0] = float(max(cmd[0], mn)) if p0 is None else float(p0)
    vl = np.zeros(n) if vlog is None else vlog
    vl[0] = 0.0
    _follow_core(c, pos, vl, 1, float(pos[0]), 0.0, period_s, vmax, amax)
    return pos


def _follow_resume(
    cmd_new: np.ndarray, cmd_base: np.ndarray, pos_base: np.ndarray, vlog_base: np.ndarray,
    delay_s: float, vmax: float, amax: float, mn: float, period_s: float,
) -> np.ndarray:
    """cmd_base で追った結果（pos_base・vlog_base。p0 は同じ）を使い、cmd_new の位置の列を作る。
    遅らせた指令が最初に違う行 kd までは同じなので、行 kd の状態から続きだけを追う
    （`_follow_pedal(cmd_new, ...)` と同じ値。候補ごとに列の途中だけが違うときの高速化）。"""
    c_new = _pedal_cmd(cmd_new, delay_s, mn, period_s)
    c_base = _pedal_cmd(cmd_base, delay_s, mn, period_s)
    diff = np.flatnonzero(c_new != c_base)
    if diff.size == 0:
        return pos_base
    kd = int(diff[0])
    pos = pos_base.copy()
    if kd + 1 < len(pos):
        _follow_core(
            c_new, pos, vlog_base.copy(), kd + 1, float(pos_base[kd]), float(vlog_base[kd]),
            period_s, vmax, amax,
        )
    return pos


def track_pedal_position(
    effort: np.ndarray,
    *,
    period_s: float,
    track: PedalTrack,
    accel_min_pct: float,
    brake_min_pct: float,
    accel_p0: float | None = None,
    brake_p0: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """計画の列 effort（正＝アクセル・負＝ブレーキ・0＝両方 min）から、各ペダルの実際の位置を
    周期ごとに作る（2026-10-26 段10-1）。そのペダルでない行・0 の行の指令は min。遅れ → 速さの上限
    ＋ 加速度の上限で追う（`_follow_pedal`）。p0 省略時は最初の行の指令（min 以上）。"""
    e = np.asarray(effort, dtype=float)
    cmd_a = np.maximum(np.where(e > 0.0, e, accel_min_pct), accel_min_pct)
    cmd_b = np.maximum(np.where(e < 0.0, -e, brake_min_pct), brake_min_pct)
    pa = _follow_pedal(
        cmd_a, accel_p0, track.accel_delay_s, track.accel_max_rate_pct_s,
        track.accel_max_accel_pct_s2, accel_min_pct, period_s,
    )
    pb = _follow_pedal(
        cmd_b, brake_p0, track.brake_delay_s, track.brake_max_rate_pct_s,
        track.brake_max_accel_pct_s2, brake_min_pct, period_s,
    )
    return pa, pb


def _creep_of(
    enabled: bool, speeds: Sequence[float], accels: Sequence[float]
) -> tuple[np.ndarray, np.ndarray] | None:
    """2026-10-28 段14 L1: predict_creep_curve が true で曲線があるとき (速度, 加速度) の配列、
    それ以外は None（今までと同じ）。"""
    if not enabled or len(speeds) == 0:
        return None
    return np.asarray(speeds, dtype=float), np.asarray(accels, dtype=float)


def predicted_accel(
    table: ForwardTable,
    v: np.ndarray,
    accel_pos: np.ndarray,
    brake_pos: np.ndarray,
    *,
    track: PedalTrack,
    accel_min_pct: float,
    brake_min_pct: float,
    period_s: float,
    launch_a: np.ndarray | None = None,
    stop_a: np.ndarray | None = None,
    launch_b: np.ndarray | None = None,
    stop_b: np.ndarray | None = None,
    creep: tuple[np.ndarray, np.ndarray] | None = None,
) -> np.ndarray:
    """車の時刻 j の加速度 [km/h/s]（2026-10-26 段10-1）。位置をペダルごとに effect_lag だけ後ろへ
    ずらし（直線補間）、アクセルが min 超ならアクセルの表、そうでなくブレーキが min 超ならブレーキ側
    （u を負にして同じ表）、両方 min なら惰行。本体の表だけ（発進・停止・戻しの層は使わない）。

    2026-10-27 段13: launch_a・stop_a（アクセルの行）、launch_b・stop_b（ブレーキと惰行の行）は
    行ごとの bool 配列（v と同じ長さ。行 j の値は計画の側の配列の行 j）。True の行はその層の表
    （発進側 launch=True・停止側 stop=True）で引く。全部 None（既定）は今までと完全に同じ。

    2026-10-28 段14 L1: creep ＝ (速度 [km/h] の昇順, クリープ加速度 [km/h/s])。None 以外なら、
    両ペダルが min 以下（位置 − min ≦ 0）で車速が速度の最後より小さい行の
    加速度を np.interp(v, 速度, 加速度) にする（先頭より小さい速度は先頭の値）。
    None（既定）は今までと完全に同じ。"""
    pa = _shift_pos(accel_pos, track.accel_effect_lag_s / period_s)
    pb = _shift_pos(brake_pos, track.brake_effect_lag_s / period_s)
    return _accel_by_pos(
        table, v, pa, pb, accel_min_pct, brake_min_pct, launch_a, stop_a, launch_b, stop_b, creep
    )


def _shift_pos(pos: np.ndarray, lag_rows: float) -> np.ndarray:
    """位置を lag_rows 行だけ後ろへずらす（直線補間）。"""
    idx = np.arange(len(pos), dtype=float)
    return np.interp(idx - lag_rows, idx, pos)


def _accel_by_pos(
    table: ForwardTable,
    v: np.ndarray,
    pa: np.ndarray,
    pb: np.ndarray,
    accel_min_pct: float,
    brake_min_pct: float,
    launch_a: np.ndarray | None,
    stop_a: np.ndarray | None,
    launch_b: np.ndarray | None,
    stop_b: np.ndarray | None,
    creep: tuple[np.ndarray, np.ndarray] | None = None,
) -> np.ndarray:
    """`predicted_accel` の表を引く部分（pa・pb は効くまでの遅れを入れ済みの位置）。"""
    ua = pa - accel_min_pct
    ub = pb - brake_min_pct
    vv = np.asarray(v, dtype=float)
    a_acc = np.asarray(table.accel(vv, np.maximum(ua, 0.0)), dtype=float)
    a_brk = np.asarray(table.accel(vv, -np.maximum(ub, 0.0)), dtype=float)
    a_coast = np.asarray(table.coast_accel(vv), dtype=float)
    # 段13: 低速の行だけ発進側・停止側の表で引き直す（None の組は何もしない）
    for la, sa, is_acc in ((launch_a, stop_a, True), (launch_b, stop_b, False)):
        for flag_arr, kw in ((sa, {"stop": True}), (la, {"launch": True})):
            if flag_arr is None or not np.any(flag_arr):
                continue
            m = np.asarray(flag_arr, dtype=bool)
            if is_acc:
                a_acc = np.where(m, table.accel(vv, np.maximum(ua, 0.0), **kw), a_acc)
            else:
                a_brk = np.where(m, table.accel(vv, -np.maximum(ub, 0.0), **kw), a_brk)
                a_coast = np.where(m, table.coast_accel(vv, **kw), a_coast)
    # 段14 L1: 両ペダル min 以下の低速の行は手順2 が測ったクリープの曲線で引く
    if creep is not None and len(creep[0]) > 0:
        cs, ck = creep
        both_min = (ua <= 0.0) & (ub <= 0.0) & (vv < float(cs[-1]))
        a_coast = np.where(both_min, np.interp(vv, cs, ck), a_coast)
    return np.where(ua > 0.0, a_acc, np.where(ub > 0.0, a_brk, a_coast))


def position_return_row(
    pos: np.ndarray, start: int, mn: float, tol: float = POSITION_RETURN_TOL_PCT
) -> int | None:
    """pos[start:] で初めて pos <= mn + tol になる行（無ければ None）。tol 0.05 は mode_drive の
    INTERLOCK_TOL_PCT と同じ（実機が「min に着いた」と見る幅）。"""
    hit = np.flatnonzero(np.asarray(pos)[start:] <= mn + tol)
    return None if hit.size == 0 else int(start + hit[0])


@dataclass
class GapContext:
    """案 R（`switch_gap_rule`）がすき間を決めるのに使う文脈（2026-10-26 段10-2）。
    v ＝ 行ごとの基準車速、ref_acc ＝ 車の時刻の基準の傾き、coast_a・coast_b ＝ 行ごとの惰行の
    加速度。"""

    table: ForwardTable
    v: np.ndarray
    ref_acc: np.ndarray
    coast_a: np.ndarray
    coast_b: np.ndarray
    track: PedalTrack
    accel_min_pct: float
    brake_min_pct: float
    brake_lag_s: float
    # 2026-10-26 段10-3: 案 C（switch_gap_min_peak）の文脈。R だけのときは使わない（既定のまま）
    use_rule: bool = True  # False なら案 R の山・谷の置き直しはしない（C だけ）
    min_peak: bool = False  # 案 C を使う
    need: np.ndarray | None = None  # 行ごとの要る加速度（上乗せ前）
    accel_lag_s: float = 0.0
    comp: bool = False  # 候補の列に 7-c の上乗せを入れる（上乗せ済みの列を評価するときは False）
    # 2026-10-26 段10-2': R の谷の上乗せの倍率 f の記録（`_gap_compensation` が足す）
    comp_f_log: list = field(default_factory=list)
    peak_window_s: float = 2.0
    # 2026-10-26 段11-4: True なら R の山の戻し始めを探す下限を、固定 RULE_PEAK_RANGE_S でなく
    # 基準の加速度が頂点へ下がり始める行から決める（`_rule_peak`）
    peak_from_corner: bool = False
    # 2026-10-27 段12: True なら R の山の採点を、戻す前の水準のずれを数えず（案 Y）、遅れのぶれを
    # 見込んだ（案 X）積み上がりの最大にする（`_rule_peak`）
    peak_robust: bool = False
    # 2026-10-27 段12 谷: True なら R の谷の上乗せの倍率 f を、数え始めを変わり目の直前へ遅らせ
    # （案 Y）、遅れのぶれを見込んで（案 X）、粗く見てから細かく詰めて選ぶ
    # （`_valley_boost_gain_robust`）
    valley_robust: bool = False
    # 2026-10-26 段11-5: True なら R の後に `_choose_pass`
    # （山・谷ごとに R のまま／後ろで取るを選ぶ）。
    # alt_* は 7-c の上乗せなしの effort・flag（谷の「後ろで取る」で上乗せを外す。
    # 無ければ上乗せなし）
    choose: bool = False
    alt_effort: np.ndarray | None = None
    alt_flag: np.ndarray | None = None
    # 2026-10-27 段13: predict_low_speed_sides が true のとき、予測に渡す低速の行の表の側
    # （全行の launch_a, stop_a, launch_b, stop_b の bool 配列）。None なら今までと同じ
    lp_sides: tuple | None = None
    # 2026-10-28 段14 L1: predict_creep_curve が true のときの (速度, クリープ加速度)。
    # None なら使わない
    creep_curve: tuple[np.ndarray, np.ndarray] | None = None

    def side_kw(self, lo: int = 0, hi: int | None = None) -> dict[str, Any]:
        """`predicted_accel` に渡す低速の表の側（行 lo〜hi の切り出し）とクリープの曲線。
        持たなければ空。"""
        kw: dict[str, Any] = {}
        if self.lp_sides is not None:
            names = ("launch_a", "stop_a", "launch_b", "stop_b")
            kw = {k: a[lo:hi] for k, a in zip(names, self.lp_sides, strict=True)}
        if self.creep_curve is not None:
            kw["creep"] = self.creep_curve
        return kw


@dataclass
class ReturnGapStats:
    """切り替えのすき間の確保（`ensure_return_gap`）の集計（2026-10-17 段2o-9）。"""

    n_switches: int = 0  # アクセル⇔ブレーキの切り替えの数
    n_cut: int = 0  # 戻す側の最後を前倒しで min にした切り替えの数
    cut_s: float = 0.0  # 前倒しで min にした合計秒数
    n_delayed: int = 0  # 次のペダルを遅らせた切り替えの数
    delayed_s: float = 0.0  # 遅らせた合計秒数
    n_violations: int = 0  # 処理の後に残ったすき間不足の数（0 のはず）
    # 2026-10-26 段10-6: True なら n_violations は戻す側の位置（PedalTrack）で数えた
    by_position: bool = False
    defaults_used: tuple[str, ...] = ()  # 測定値が無く仮値を使ったペダル
    # 2026-10-24 段7-c: すき間で失う分の補償（switch_gap_compensate）。補償しなければ全部 0
    comp_events: int = 0  # 補償した切り替えの数
    comp_dv_kmh: float = 0.0  # 上乗せした Δv の絶対値の合計 [km/h]
    comp_new_trim_rows: int = 0  # 上乗せ後の 2 回目の確保で新しく惰行に削られた行数
    # 2026-10-26 段10-2: 案 R（switch_gap_rule）で置き直した切り替えの数。使わなければ 0
    n_rule_peak: int = 0  # 山（アクセル → ブレーキ）
    n_rule_valley: int = 0  # 谷（ブレーキ → アクセル）で前を削った数
    # R が削った行の範囲 [始め, 終わり)。7-c の補償が山（補償しない）と谷（予測で測る）を見分ける
    rule_peak_spans: list = field(default_factory=list)
    rule_valley_spans: list = field(default_factory=list)
    # 2026-10-26 段10-3: 案 C（switch_gap_min_peak）で、前を削る・後ろを遅らせる組を選んだ
    # 切り替えの数と、削った行の範囲 [始め, 終わり)（7-c の補償が予測で Δv を測る）
    n_min_peak: int = 0
    min_peak_spans: list = field(default_factory=list)
    # 2026-10-26 段10-2': R の谷の上乗せ（`_gap_compensation`）の数と、倍率 f が上限に張り付いた数
    n_valley_boost: int = 0
    n_valley_f_capped: int = 0
    # 2026-10-26 段11-5: R が置いた切り替えの記録 (kind, a, last, i, j, cut0, zr)（kind は "peak"・
    # "valley"。cut0 ＝ R が削った前の行の始め、zr ＝ R が遅らせた後ろの行の終わり）と、
    # 選び直しの結果
    rule_switches: list = field(default_factory=list)
    n_choose_back_peak: int = 0
    n_choose_back_valley: int = 0
    # (行 i, 種類, R の点数, 後ろで取る点数（作れなければ None）, 選んだ "R"・"back")
    choose_log: list = field(default_factory=list)

    def summary(self) -> str:
        text = (
            f"切り替え {self.n_switches} 回・戻す側を前倒し {self.n_cut} 回（{self.cut_s:.2f}s）・"
            f"次を遅らせ {self.n_delayed} 回（{self.delayed_s:.2f}s）・"
            f"すき間不足の残り {self.n_violations} 回"
            + ("（位置で数えた）" if self.by_position else "")
        )
        if self.defaults_used:
            text += f"・戻る時間は仮値（{'・'.join(self.defaults_used)}。pkl に測定値なし）"
        if self.comp_events:
            text += (
                f"・すき間の補償 {self.comp_events} 回（Δv 計 {self.comp_dv_kmh:.2f} km/h・"
                f"2 回目で新しく削られた {self.comp_new_trim_rows} 行）"
            )
        if self.n_rule_peak or self.n_rule_valley:
            text += f"・案 R で置き直し 山 {self.n_rule_peak} 回・谷 {self.n_rule_valley} 回"
        if self.n_valley_boost:
            text += (
                f"・R の谷の上乗せ {self.n_valley_boost} 回（倍率 f が上限に張り付き "
                f"{self.n_valley_f_capped} 回）"
            )
        if self.choose_log:
            text += (
                f"・R/後ろの選び直し 山 {self.n_choose_back_peak} 回・"
                f"谷 {self.n_choose_back_valley} 回"
                f"（後ろを選んだ。対象 {len(self.choose_log)} 回）"
            )
        if self.n_min_peak:
            text += f"・案 C で予測の最大偏差が最小の組を選んだ {self.n_min_peak} 回"
        return text


# 2026-10-27 段13: 発進のずらし（launch_shift_choose）の定数。ずらす行数 0〜8（0.05 s 刻みで 0.4 s
# まで）は仮の値（ProblemReport_20261027 段2 の机上で最良は 1〜2 行。上限は余裕を見た仮置き）。
# いつ決め直すか: 実機で選ばれた δ の分布が上限に張り付いたら。
LAUNCH_SHIFT_MAX_ROWS = 8
# ずらす範囲: 停車保持の終わりの 2.5 s 前〜基準の動き出しの 2.0 s 後（依頼 2026-10-27。仮の値）
LAUNCH_SHIFT_BEFORE_HOLD_END_S = 2.5
LAUNCH_SHIFT_AFTER_GO_S = 2.0
# 採点: 車速を基準の動き出しの 2.0 s 前から v=0 で積み、動き出しの 1.0 s 前〜1.5 s 後の |偏差| の
# 最大で比べる（依頼 2026-10-27。仮の値）
LAUNCH_SHIFT_INTEGRATE_FROM_S = 2.0
LAUNCH_SHIFT_SCORE_FROM_S = 1.0
LAUNCH_SHIFT_SCORE_TO_S = 1.5


@dataclass
class LaunchShiftStats:
    """発進のずらし（`apply_launch_shift`）の集計（2026-10-27 段13）。"""

    n_launch: int = 0  # 停車からの発進の数
    deltas: list = field(default_factory=list)  # 発進ごとに選んだ δ [行]
    scores: list = field(default_factory=list)  # (δ=0 の最悪点, 選んだ δ の最悪点) [km/h]
    n_rejected: int = 0  # 同時踏みが増えるため候補から外した (発進, δ) の数

    def summary(self) -> str:
        """走行前の表示用。例: 「あり・発進 3 回・δ 0 行×1・2 行×2」"""
        if not self.deltas:
            return f"なし・発進 {self.n_launch} 回"
        counts: dict[int, int] = {}
        for d in self.deltas:
            counts[d] = counts.get(d, 0) + 1
        parts = "・".join(f"{d} 行×{counts[d]}" for d in sorted(counts))
        shifted = sum(1 for d in self.deltas if d > 0)
        head = "あり" if shifted else "なし"
        return f"{head}・発進 {self.n_launch} 回・δ {parts}"


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
    # 2026-10-27 段13: 発進のずらし（launch_shift_choose）の集計。使わなければ None
    launch_shift: LaunchShiftStats | None = None

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
    switch_gap_compensate: bool = False,
    switch_gap_min_loss: bool = False,
    switch_gap_rule: bool = False,
    switch_gap_min_peak: bool = False,
    switch_gap_peak_window_s: float = 2.0,
    switch_gap_peak_from_corner: bool = False,
    switch_gap_peak_robust: bool = False,
    switch_gap_valley_robust: bool = False,
    switch_gap_choose: bool = False,
    pedal_track: PedalTrack | None = None,
    accel_release_in_range_only: bool = False,
    predict_low_speed_sides: bool = False,
    launch_shift_choose: bool = False,
    predict_creep_curve: bool = False,
    creep_accel_speeds_kmh: Sequence[float] = (),
    creep_accel_kmhs: Sequence[float] = (),
    _lead_extra: np.ndarray | None = None,
    _need_add: np.ndarray | None = None,
    _aux_out: dict[str, np.ndarray] | None = None,
    _choose_alt: tuple[np.ndarray, np.ndarray] | None = None,
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

    2026-10-24 段7-c: `switch_gap_compensate`。False は今までと完全に同じ。True は、アクセル ⇔
    ブレーキの切り替え（停車保持・先置きをまたぐものは除く。`switch_lead_events` と同じ対象）で
    `ensure_return_gap` が戻す側の最後の行を惰行に削って失う速度
    Δv ＝ Σ（その行の要る加速度 − 惰行の加速度）× 周期 を、削られた行の直前の同じペダルの同じ行数に
    均等に上乗せして（要る加速度 ＋ Δv ÷ その秒数）、同じ表・同じ側で解き直す（`_need_add`）。
    解き直した後のすき間の確保で新しく削られた行数は ReturnGapStats.comp_new_trim_rows に残す
    （追わない）。新しい調整値は無い。

    2026-10-25 段9-b: `switch_gap_min_loss`。False は今までと完全に同じ。True は、アクセル ⇔
    ブレーキの切り替え（停車保持・先置きが絡むものは今までどおり）のすき間の確保で、前（戻す側）で
    c 行削る・後ろ（次のペダル）で d 行遅らせる組み合わせのうち、すき間の条件を満たし、損の合計
    Σ|要る加速度 − 惰行の加速度|×周期（惰行にした行）が最小のものを選ぶ（`ensure_return_gap`）。
    後ろで遅らせた分は補償しない（段7-c の補償は前で削った分だけ）。新しい調整値は無い。

    2026-10-26 段10-2: `switch_gap_rule`（案 R）。False は今までと完全に同じ。True は、9-b と同じ
    対象のアクセル ⇔ ブレーキの切り替えのうち、切り替え先の最初の行の惰行の加速度が負のものを、
    ペダルの実際の位置（`pedal_track`。None なら PedalTrack の既定）で置き直す（`ensure_return_gap`
    の rule_ctx）。山（アクセル → ブレーキ）は車の加速度が下がる区間の真ん中を頂点に置き、ブレーキは
    アクセルの位置が min に着いた行から。谷（ブレーキ → アクセル）はすき間を前で取る。7-c の補償は
    谷の前の削りだけ（山の削りは補償しない）で、Δv は戻る途中のブレーキの効きを入れる。9-b も on
    なら、R の対象の切り替えは R、それ以外は 9-b。

    2026-10-26 段11-5: `switch_gap_choose`（switch_gap_rule が True のときだけ効く）。
    False は今までと
    完全に同じ。True は、R が扱う山・谷ごとに「R のまま」と「後ろで取る」の列を作り、切り替えの前後
    switch_gap_peak_window_s の予測した積み上げの偏差の最大の大きさが小さい方を選ぶ
    （`_choose_pass`。同点は R）。`_choose_alt` は内部用（7-c の上乗せなしの effort・flag）。

    2026-10-27 段12: `switch_gap_peak_robust`（switch_gap_rule が True のときだけ効く）。False は
    今までと完全に同じ。True は R の山の採点だけを変える（`_rule_peak`）: 戻す前の加速中にたまった
    予測の片寄り（実機に無い水準のずれ）を数えないよう積み上げの数え始めを遅らせ（案 Y）、ペダルが
    効くまでの遅れを PEAK_ROBUST_LAG_ROWS 行ずらした 3 通りの積み上がりの最大の大きさで選ぶ
    （案 X）。

    2026-10-27 段12（谷）: `switch_gap_valley_robust`（switch_gap_rule が True のときだけ効く）。
    False は今までと完全に同じ。True は R の谷の上乗せの倍率 f の決め方だけを変える
    （`_valley_boost_gain_robust`）: 積み上がりの数え始めを上乗せを配る最初の行の 1 行前へ遅らせ
    （案 Y）、ペダルが効くまでの遅れを PEAK_ROBUST_LAG_ROWS 行ずらした 3 通りの積み上がりの
    最大の大きさが最小の f を、0.1 刻みで粗く見てから黄金分割で詰めて選ぶ（案 X）。

    2026-10-26 段10-3: `switch_gap_min_peak`（案 C）。False は今までと完全に同じ。True は、9-b と
    同じ対象のすき間の取り方を、損の最小から「予測した最大偏差」の最小に直す（`_min_peak_split`）。
    前を c 行削る・後ろを d 行遅らせる組のうち、戻す側の実際の位置が min に着く行 ≤ 次のペダルの
    最初の行を満たすものを、切り替え（元の位置）の前後 `switch_gap_peak_window_s` 秒の
    「予測した加速度 − 基準の傾き」の積み上げの最大の大きさで比べる。c > 0 で
    `switch_gap_compensate` が on なら 7-c と同じ上乗せを入れた列で比べる。on のときは
    switch_gap_min_loss の値に関係なく C の比べ方を使う（組が無ければ今までの手順）。
    `switch_gap_rule` と同時には使えない。

    2026-10-25 段9-c: `accel_release_in_range_only`。False は今までと完全に同じ。True は、アクセルの
    戻しの層を学習データの範囲の中だけで使う（`_resolve_accel_releases`。戻し量が meta
    accel_release_max_pct を超える点は本体・直近の最大 u が meta accel_release_peak_max_pct を超える
    時刻は層なし）。meta が無い表ではエラー。

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
        終わりまでの最深」にする（ブレーキは戻す向きにしか動かない）。
        惰行の行も後ろにブレーキがあれば
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
    if launch_shift_choose:
        # 2026-10-27 段13: 発進のずらしは、できあがった計画（すき間の処理まで済み）に後から掛ける。
        # 内側の呼び出しは同じ引数でずらしだけ off（再帰の common には渡さない＝ずらしは 1 回だけ）
        kw_in = dict(locals())
        kw_in["launch_shift_choose"] = False
        return apply_launch_shift(
            build_plan(**kw_in), ref_at, table, stop_ref_at=stop_ref_at,
            accel_min_pct=accel_deadband_pct, brake_min_pct=brake_deadband_pct,
            track=pedal_track or PedalTrack(),
            creep=_creep_of(predict_creep_curve, creep_accel_speeds_kmh, creep_accel_kmhs),
        )
    creep = _creep_of(predict_creep_curve, creep_accel_speeds_kmh, creep_accel_kmhs)
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
            "switch_gap_compensate": switch_gap_compensate,
            "switch_gap_min_loss": switch_gap_min_loss,
            "switch_gap_rule": switch_gap_rule, "pedal_track": pedal_track,
            "switch_gap_min_peak": switch_gap_min_peak,
            "switch_gap_peak_window_s": switch_gap_peak_window_s,
            "switch_gap_peak_from_corner": switch_gap_peak_from_corner,
            "switch_gap_peak_robust": switch_gap_peak_robust,
            "switch_gap_valley_robust": switch_gap_valley_robust,
            "switch_gap_choose": switch_gap_choose,
            "accel_release_in_range_only": accel_release_in_range_only,
            "predict_low_speed_sides": predict_low_speed_sides,
            "predict_creep_curve": predict_creep_curve,
            "creep_accel_speeds_kmh": creep_accel_speeds_kmh,
            "creep_accel_kmhs": creep_accel_kmhs,
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

    if switch_gap_compensate and _need_add is None and return_gap_enabled:
        # 段7-c: 1 回目（先置き・すき間の確保なし）で削られる行と Δv を調べ、上乗せして解き直す
        common_c = {
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
            "switch_lead": switch_lead, "_lead_extra": _lead_extra,
            "switch_gap_min_loss": switch_gap_min_loss,
            "switch_gap_rule": switch_gap_rule, "pedal_track": pedal_track,
            "switch_gap_min_peak": switch_gap_min_peak,
            "switch_gap_peak_window_s": switch_gap_peak_window_s,
            "switch_gap_peak_from_corner": switch_gap_peak_from_corner,
            "switch_gap_peak_robust": switch_gap_peak_robust,
            "switch_gap_valley_robust": switch_gap_valley_robust,
            "switch_gap_choose": switch_gap_choose,
            "accel_release_in_range_only": accel_release_in_range_only,
            "predict_low_speed_sides": predict_low_speed_sides,
            "predict_creep_curve": predict_creep_curve,
            "creep_accel_speeds_kmh": creep_accel_speeds_kmh,
            "creep_accel_kmhs": creep_accel_kmhs,
            "preposition_enabled": False, "return_gap_enabled": False,
        }
        aux: dict[str, np.ndarray] = {}
        base = build_plan(ref_at, duration_s, table, **common_c, _aux_out=aux)
        ret_c = return_time_of(table, accel_deadband_pct, brake_deadband_pct)
        ctx_c = (
            GapContext(
                table=table, v=aux["v"], ref_acc=aux["ref_acc"], coast_a=aux["coast_a"],
                coast_b=aux["coast_b"], track=pedal_track or PedalTrack(),
                accel_min_pct=accel_deadband_pct, brake_min_pct=brake_deadband_pct,
                brake_lag_s=lb, use_rule=switch_gap_rule, min_peak=switch_gap_min_peak,
                need=base.need_accel_kmhs, accel_lag_s=la, comp=True,
                peak_window_s=switch_gap_peak_window_s,
                peak_from_corner=switch_gap_peak_from_corner,
                peak_robust=switch_gap_peak_robust,
                valley_robust=switch_gap_valley_robust,
                lp_sides=(
                    tuple(aux[k] for k in ("lp_la", "lp_sa", "lp_lb", "lp_sb"))
                    if "lp_la" in aux else None
                ),
                creep_curve=creep,
            )
            if (switch_gap_rule or switch_gap_min_peak)
            else None
        )
        # 上乗せ後の 2 回目の確保は、列に上乗せが入っているので候補に重ねて上乗せしない
        ctx_c2 = replace(ctx_c, comp=False) if ctx_c is not None else None
        add, dv_total, n_ev, trimmed0 = _gap_compensation(
            base, aux, ret_c, period_s, min_loss=switch_gap_min_loss, rule_ctx=ctx_c
        )
        if n_ev > 0:
            comped = build_plan(
                ref_at, duration_s, table, **common_c, _need_add=add, _aux_out={}
            )
            e1 = comped.effort_pct.copy()
            if switch_gap_min_loss:
                ensure_return_gap(
                    e1, comped.flag.copy(), ret=ret_c, period_s=period_s,
                    need_accel=base.need_accel_kmhs, coast_a=aux["coast_a"], coast_b=aux["coast_b"],
                    rule_ctx=ctx_c2,
                )
            else:
                ensure_return_gap(
                    e1, comped.flag.copy(), ret=ret_c, period_s=period_s, rule_ctx=ctx_c2
                )
            trimmed1 = (comped.effort_pct != 0.0) & (e1 == 0.0)
            new_rows = int(np.count_nonzero(trimmed1 & ~trimmed0))
            choose_alt = None
            if switch_gap_choose and switch_gap_rule:  # 段11-5: 上乗せなしの effort・flag
                nb = build_plan(
                    ref_at, duration_s, table, **{
                        **common_c, "preposition_enabled": preposition_enabled,
                        "return_gap_enabled": False,
                    },
                    preposition_margin_pct=preposition_margin_pct,
                    preposition_lead_s=preposition_lead_s,
                )
                choose_alt = (nb.effort_pct.copy(), nb.flag.copy())
            plan2 = build_plan(
                ref_at, duration_s, table, **{
                    **common_c, "preposition_enabled": preposition_enabled,
                    "return_gap_enabled": return_gap_enabled,
                },
                preposition_margin_pct=preposition_margin_pct,
                preposition_lead_s=preposition_lead_s, _need_add=add, _choose_alt=choose_alt,
            )
            if plan2.return_gap is not None:
                plan2.return_gap.comp_events = n_ev
                plan2.return_gap.comp_dv_kmh = dv_total
                plan2.return_gap.comp_new_trim_rows = new_rows
                if ctx_c is not None:  # 段10-2': R の谷の上乗せの倍率 f
                    plan2.return_gap.n_valley_boost = len(ctx_c.comp_f_log)
                    plan2.return_gap.n_valley_f_capped = sum(
                        1 for f in ctx_c.comp_f_log if f >= VALLEY_BOOST_F_MAX - 1e-9
                    )
            return plan2

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
    if _need_add is not None:  # 段7-c: すき間で失う分の上乗せ（要る加速度に足して解き直す）
        need_a = need_a + _need_add
        need_b = need_b + _need_add
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
    if _need_add is not None:
        # 計画が持つ「要る加速度」は上乗せ前（P・DOB の予測は元の基準で見る）
        need = need - _need_add
    if _aux_out is not None:  # 段7-c: 惰行の加速度（惰行の判定に使った表の値）を渡す
        _aux_out["coast_a"] = np.broadcast_to(np.asarray(coast_a, dtype=float), (n,)).copy()
        _aux_out["coast_b"] = np.broadcast_to(np.asarray(coast_b, dtype=float), (n,)).copy()
        if switch_gap_rule or switch_gap_min_peak:  # 段10-2・3: 案 R・C の文脈
            _aux_out["v"] = np.asarray(v, dtype=float).copy()
            _aux_out["ref_acc"] = window_slope_vec(ref_at, t, 0.0, need_window_s)

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
            effort, redo_a, u_a, v, need_a, table, brake_deadband_pct, accel_deadband_pct, period_s,
            in_range_only=accel_release_in_range_only,
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
    side_accel = np.where(launch_a, SIDE_LAUNCH, np.where(stop_a, SIDE_STOP, SIDE_MAIN))
    side_accel = np.where(relabel_a, SIDE_MAIN, side_accel)
    side_brake = np.where(launch_b, SIDE_LAUNCH, np.where(stop_b, SIDE_STOP, SIDE_MAIN))
    side_brake = np.where(released & ~stop_b & ~launch_b, SIDE_RELEASE, side_brake)
    if side_brake_dir is not None:
        side_brake = side_brake_dir
    # 2026-10-27 段13: predict_low_speed_sides が true で表の側を持つ規則（pedal・dir）のときだけ、
    # すき間の処理の予測（predicted_accel）に低速の行の表の側を渡す。車速は基準車速 v で
    # meta stop_max_speed_kmh（無ければ 7.5）未満の行。legacy は側が決まらないので今までと同じ
    lp_sides = None
    if predict_low_speed_sides and (dir_rule or pedal_rule):
        lowv = np.asarray(v, dtype=float) < float(table.meta.get("stop_max_speed_kmh", 7.5))
        lp_sides = (
            lowv & (side_accel == SIDE_LAUNCH), lowv & (side_accel == SIDE_STOP),
            lowv & (side_brake == SIDE_LAUNCH), lowv & (side_brake == SIDE_STOP),
        )
        if _aux_out is not None:
            for key, arr in zip(("lp_la", "lp_sa", "lp_lb", "lp_sb"), lp_sides, strict=True):
                _aux_out[key] = arr.copy()
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
    gap_stats = None
    if return_gap_enabled:
        rule_ctx = None
        if switch_gap_rule or switch_gap_min_peak:  # 段10-2・3: 案 R・C（ペダルの実際の位置）
            rule_ctx = GapContext(
                table=table, v=np.asarray(v, dtype=float),
                ref_acc=window_slope_vec(ref_at, t, 0.0, need_window_s),
                coast_a=np.broadcast_to(np.asarray(coast_a, dtype=float), (n,)),
                coast_b=np.broadcast_to(np.asarray(coast_b, dtype=float), (n,)),
                track=pedal_track or PedalTrack(), accel_min_pct=accel_deadband_pct,
                brake_min_pct=brake_deadband_pct, brake_lag_s=lb,
                use_rule=switch_gap_rule, min_peak=switch_gap_min_peak, need=need,
                accel_lag_s=la, comp=switch_gap_compensate and _need_add is None,
                peak_window_s=switch_gap_peak_window_s,
                peak_from_corner=switch_gap_peak_from_corner,
                peak_robust=switch_gap_peak_robust,
                valley_robust=switch_gap_valley_robust,
                choose=switch_gap_choose and switch_gap_rule,
                alt_effort=None if _choose_alt is None else _choose_alt[0],
                alt_flag=None if _choose_alt is None else _choose_alt[1],
                lp_sides=lp_sides,
                creep_curve=creep,
            )
        if switch_gap_min_loss:  # 段9-b: 損の合計が最小の組（要る加速度・惰行の加速度を渡す）
            gap_stats = ensure_return_gap(
                effort, flag, ret=ret, period_s=period_s, need_accel=need,
                coast_a=np.broadcast_to(np.asarray(coast_a, dtype=float), (n,)),
                coast_b=np.broadcast_to(np.asarray(coast_b, dtype=float), (n,)),
                rule_ctx=rule_ctx,
            )
        else:
            gap_stats = ensure_return_gap(
                effort, flag, ret=ret, period_s=period_s, rule_ctx=rule_ctx
            )
    return ForwardPlan(
        t_s=t, effort_pct=effort, need_accel_kmhs=need, flag=flag, period_s=period_s,
        preposition=stats, return_gap=gap_stats, stops_ahead=stops_ahead,
        stop_lead=lead_mask, side_accel=side_accel.astype(int), side_brake=side_brake.astype(int),
        side_rule="dir" if dir_rule else ("pedal" if pedal_rule else "legacy"),
        switch_lead_s=_lead_extra,
        launch_midway=np.asarray(midway, dtype=bool).copy() if dir_rule else None,
        launch_prep=launch_prep,
    )


def _launch_events(
    stop_now: np.ndarray, flag: np.ndarray
) -> list[tuple[int, int, int]]:
    """停車からの発進 (停車の始め s, 基準の動き出し t0, 停車保持の終わり hold_end)。
    t0 ＝ 停車（基準が停車速度以下の連なり）の直後の最初の行（`_launch_release_ramp` と同じ）。
    hold_end ＝ t0 の手前の最後の FLAG_STOP の行の次（前の発進の t0 以降で探す。無ければ t0）。"""
    n = len(stop_now)
    low = stop_now <= STOP_SPEED_KMH
    out: list[tuple[int, int, int]] = []
    prev_end = 0
    i = 0
    while i < n:
        if not low[i]:
            i += 1
            continue
        s = i
        while i < n and low[i]:
            i += 1
        if i >= n:
            break
        t0 = i
        j = t0 - 1
        while j >= prev_end and flag[j] != FLAG_STOP:
            j -= 1
        out.append((s, t0, j + 1 if j >= prev_end else t0))
        prev_end = t0
    return out


def _shift_back(arr: np.ndarray | None, a: int, b: int, d: int) -> np.ndarray | None:
    """行 a〜b の中身を d 行後ろへ移す（前の空いた d 行は直前の行 a−1 の値）。コピーを返す。"""
    if arr is None:
        return None
    out = arr.copy()
    if d > 0:
        out[a + d : b + 1] = arr[a : b + 1 - d]
        out[a : a + d] = arr[a - 1]
    return out


def apply_launch_shift(
    plan: ForwardPlan,
    ref_at: Callable[[float], float],
    table: ForwardTable,
    *,
    stop_ref_at: Callable[[float], float] | None,
    accel_min_pct: float,
    brake_min_pct: float,
    track: PedalTrack,
    creep: tuple[np.ndarray, np.ndarray] | None = None,
) -> ForwardPlan:
    """停車からの各発進で、発進の並びを δ 行（0〜LAUNCH_SHIFT_MAX_ROWS）後ろへずらす
    （2026-10-27 段13。すき間の処理まで済んだ計画に掛ける）。

    動き出し t0 の 2.0 s 前から車速を v=0 で積み（v は 0 未満にしない）、予測した偏差
    （車速 − 基準車速）の t0 の 1.0 s 前〜1.5 s 後の |偏差| の最大で採点する。加速度は
    `_accel_by_pos`（ペダル位置は `track_pedal_position`。低速の行は計画の side_accel・side_brake
    の表。車速が stop_max_speed_kmh（無ければ 7.5）未満の行）。効くまでの遅れ（アクセル・ブレーキ）
    を ±PEAK_ROBUST_LAG_ROWS 行ずらした 3 通りの最悪で比べ（段12 の switch_gap_peak_robust と同じ
    考え方）、最悪が一番小さい δ（同点は小さい δ）を選ぶ。
    ずらす範囲は停車保持の終わりの 2.5 s 前〜t0 の 2.0 s 後（前の発進のずらした範囲の後ろから）。
    effort・flag・launch_prep・launch_midway・side_accel・side_brake を同じに移し、前の空いた行は
    直前の行の値。need_accel（基準から決まる）・stops_ahead・stop_lead は動かさない。
    ずらした結果、両ペダルの位置が同時に min 超になる行が δ=0 より増える候補は外す
    （片足ルール・すき間を壊さない）。"""
    n = len(plan.effort_pct)
    dt = plan.period_s
    t = plan.t_s
    ref = _eval_ref(ref_at, t)
    stop_now = ref if stop_ref_at is None else _eval_ref(stop_ref_at, t)
    events = _launch_events(stop_now, plan.flag)
    stats = LaunchShiftStats(n_launch=len(events))
    top = float(table.meta.get("stop_max_speed_kmh", 7.5))
    eff = plan.effort_pct.copy()
    flag = plan.flag.copy()
    prep = None if plan.launch_prep is None else plan.launch_prep.copy()
    mid = None if plan.launch_midway is None else plan.launch_midway.copy()
    sa = None if plan.side_accel is None else plan.side_accel.copy()
    sb = None if plan.side_brake is None else plan.side_brake.copy()
    trs = [
        replace(
            track,
            accel_effect_lag_s=max(track.accel_effect_lag_s + dr * dt, 0.0),
            brake_effect_lag_s=max(track.brake_effect_lag_s + dr * dt, 0.0),
        )
        for dr in (-PEAK_ROBUST_LAG_ROWS, 0, PEAK_ROBUST_LAG_ROWS)
    ]
    tol = POSITION_RETURN_TOL_PCT
    last_b = 0
    for _s, t0, hold_end in events:
        a = max(hold_end - round(LAUNCH_SHIFT_BEFORE_HOLD_END_S / dt), last_b + 1, 1)
        b = min(t0 + round(LAUNCH_SHIFT_AFTER_GO_S / dt), n - 1)
        r_a = max(t0 - round(LAUNCH_SHIFT_INTEGRATE_FROM_S / dt), 0)
        k_sc0 = max(t0 - round(LAUNCH_SHIFT_SCORE_FROM_S / dt), 0)
        k_end = min(t0 + round(LAUNCH_SHIFT_SCORE_TO_S / dt), n - 1)
        if a + LAUNCH_SHIFT_MAX_ROWS >= b or k_end <= k_sc0:
            stats.deltas.append(0)
            stats.scores.append((0.0, 0.0))
            continue
        st = max(min(a, r_a) - 40, 0)
        en = max(b, k_end) + 1
        best: tuple[float, int] | None = None
        base_viol = 0
        score0 = 0.0
        for d in range(LAUNCH_SHIFT_MAX_ROWS + 1):
            e_c = _shift_back(eff, a, b, d)[st:en]
            sa_c = None if sa is None else _shift_back(sa, a, b, d)[st:en]
            sb_c = None if sb is None else _shift_back(sb, a, b, d)[st:en]
            pa, pb = track_pedal_position(
                e_c, period_s=dt, track=track, accel_min_pct=accel_min_pct,
                brake_min_pct=brake_min_pct,
            )
            viol = int(np.count_nonzero((pa > accel_min_pct + tol) & (pb > brake_min_pct + tol)))
            if d == 0:
                base_viol = viol
            elif viol > base_viol:
                stats.n_rejected += 1
                continue
            worst = max(
                _launch_dev_peak(
                    table, ref[st:en], pa, pb, trv, accel_min_pct, brake_min_pct, dt, sa_c, sb_c,
                    top, r_a - st, k_sc0 - st, k_end - st, creep,
                )
                for trv in trs
            )
            if d == 0:
                score0 = worst
            if best is None or worst < best[0] - 1e-9:
                best = (worst, d)
        assert best is not None
        d_best = best[1]
        stats.deltas.append(d_best)
        stats.scores.append((score0, best[0]))
        if d_best > 0:
            eff = _shift_back(eff, a, b, d_best)
            flag = _shift_back(flag, a, b, d_best)
            prep = _shift_back(prep, a, b, d_best)
            mid = _shift_back(mid, a, b, d_best)
            sa = _shift_back(sa, a, b, d_best)
            sb = _shift_back(sb, a, b, d_best)
            last_b = b
    return replace(
        plan, effort_pct=eff, flag=flag, launch_prep=prep, launch_midway=mid, side_accel=sa,
        side_brake=sb, launch_shift=stats,
    )


def _launch_dev_peak(
    table: ForwardTable,
    ref: np.ndarray,
    pa: np.ndarray,
    pb: np.ndarray,
    trv: PedalTrack,
    accel_min_pct: float,
    brake_min_pct: float,
    period_s: float,
    side_a: np.ndarray | None,
    side_b: np.ndarray | None,
    top: float,
    k_from: int,
    k_sc0: int,
    k_end: int,
    creep: tuple[np.ndarray, np.ndarray] | None = None,
) -> float:
    """行 k_from から v=0 で車速を積み（v[k+1] = max(v[k] + 予測した加速度[k] × 周期, 0)）、行
    k_sc0〜k_end の |v − 基準| の最大 [km/h]。位置 pa・pb は遅れ trv を入れて引く。"""
    qa = _shift_pos(pa, trv.accel_effect_lag_s / period_s)
    qb = _shift_pos(pb, trv.brake_effect_lag_s / period_s)
    v = 0.0
    worst = 0.0
    for k in range(k_from, k_end + 1):
        if k >= k_sc0:
            worst = max(worst, abs(v - float(ref[k])))
        if side_a is None or side_b is None:
            fl = (None, None, None, None)
        else:
            low = v < top
            fl = (
                np.array([low and side_a[k] == SIDE_LAUNCH]),
                np.array([low and side_a[k] == SIDE_STOP]),
                np.array([low and side_b[k] == SIDE_LAUNCH]),
                np.array([low and side_b[k] == SIDE_STOP]),
            )
        a = _accel_by_pos(
            table, np.array([v]), qa[k : k + 1], qb[k : k + 1], accel_min_pct, brake_min_pct, *fl,
            creep,
        )
        v = max(v + float(a[0]) * period_s, 0.0)
    return worst


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


def _gap_compensation(
    base: ForwardPlan,
    aux: dict[str, np.ndarray],
    ret: ReturnTime,
    period_s: float,
    *,
    min_loss: bool = False,
    rule_ctx: GapContext | None = None,
) -> tuple[np.ndarray, float, int, np.ndarray]:
    """段7-c: ensure_return_gap が戻す側の最後を惰行に削る切り替えを拾い、上乗せする分を返す。

    base ＝ 先置き・すき間の確保なしの計画。その effort を `ensure_return_gap` に掛けて、削られた行
    （戻す側の連なりの末尾。先頭の 1 行は残る）を調べる。対象外: 停車保持・先置きの行が絡むもの
    （戻す側の行・切り替え先の最初の行が FLAG_STOP・FLAG_PREPOS）。
    削られた行ごとの失う分 Δv ＝ Σ（要る加速度 − 惰行の加速度）× 周期。
    上乗せ ＝ Δv ÷（行数 × 周期）を、
    削られた行の直前の同じペダルの行（連なりの中・停車保持でない行だけ。足りなければある分）へ。
    戻り値: (行ごとの上乗せ [km/h/s], Σ|Δv|, 補償した切り替えの数, 削られた行の bool 配列)

    2026-10-26 段10-2: rule_ctx（案 R）があるとき、R が置いた山のアクセルの削り（行 s〜last）は補償
    しない。R が置いた谷の前の削りは、Δv ＝ Σ（要る加速度[k] − 予測した加速度[k + ブレーキの遅れ]）
    × 周期（削った列でペダルの位置を追い、戻る途中のブレーキの効きを入れる。`predicted_accel`）。
    R の対象外の切り替えは今までどおり。

    2026-10-26 段10-3: 案 C（rule_ctx.min_peak）で前を削った切り替えも、Δv を同じ式で数える
    （削った行だけ。後ろで遅らせた分は補償しない）。遅れは削ったペダルの遅れ（la・lb）。
    """
    n = len(base.effort_pct)
    before = base.effort_pct
    after = before.copy()
    if min_loss:  # 段9-b: 同じ取り方で削る。後ろで遅らせた行は下の判定で補償の対象から外れる
        gap_st = ensure_return_gap(
            after, base.flag.copy(), ret=ret, period_s=period_s,
            need_accel=base.need_accel_kmhs, coast_a=aux["coast_a"], coast_b=aux["coast_b"],
            rule_ctx=rule_ctx,
        )
    else:
        gap_st = ensure_return_gap(
            after, base.flag.copy(), ret=ret, period_s=period_s, rule_ctx=rule_ctx
        )
    peak_starts = {sp[0] for sp in gap_st.rule_peak_spans}
    valley_starts = {sp[0] for sp in gap_st.rule_valley_spans}
    min_peak_starts = {sp[0] for sp in gap_st.min_peak_spans}
    pred = None
    if rule_ctx is not None and (valley_starts or min_peak_starts):
        pa, pb = track_pedal_position(
            after, period_s=period_s, track=rule_ctx.track,
            accel_min_pct=rule_ctx.accel_min_pct, brake_min_pct=rule_ctx.brake_min_pct,
        )
        pred = predicted_accel(
            rule_ctx.table, rule_ctx.v, pa, pb, track=rule_ctx.track,
            accel_min_pct=rule_ctx.accel_min_pct, brake_min_pct=rule_ctx.brake_min_pct,
            period_s=period_s, **rule_ctx.side_kw(),
        )
    trimmed = (before != 0.0) & (after == 0.0)
    add = np.zeros(n)
    dv_total = 0.0
    n_ev = 0
    skip = (base.flag == FLAG_STOP) | (base.flag == FLAG_PREPOS)
    i = 0
    while i < n:
        if not trimmed[i]:
            i += 1
            continue
        s = i
        while i < n and trimmed[i] and (before[i] > 0.0) == (before[s] > 0.0):
            i += 1
        e = i  # 削られた行 [s, e)
        i = max(e, s + 1)
        if s in peak_starts:  # 案 R の山のアクセルの削りは補償しない
            continue
        # 戻す側の連なりの末尾の削り（直前に同じペダルの行が残る）だけ。次のペダルを遅らせた行は除く
        if s == 0 or after[s - 1] == 0.0 or (after[s - 1] > 0.0) != (before[s] > 0.0):
            continue
        nxt = e
        while nxt < n and after[nxt] == 0.0:
            nxt += 1
        if nxt >= n or (after[nxt] > 0.0) == (before[s] > 0.0) or skip[nxt] or skip[s : e].any():
            continue
        is_valley = pred is not None and s in valley_starts and s not in min_peak_starts
        m_rows = e - s
        if is_valley:  # 段10-2': 配る行数は max(削った行数, 0.3 秒)
            m_rows = max(m_rows, round(VALLEY_BOOST_MIN_S / period_s))
        # 直前の同じペダルの行（削った行と同じ行数まで。
        # 停車保持・先置きの行・符号が変わる所で止める）
        k = s
        while (
            k > 0 and s - k < m_rows and after[k - 1] != 0.0
            and (after[k - 1] > 0.0) == (before[s] > 0.0) and not skip[k - 1]
        ):
            k -= 1
        if k == s:
            continue
        gain = 1.0
        if pred is not None and s in min_peak_starts:  # 案 C: 谷と同じ式。遅れは削ったペダルの遅れ
            lag_s = rule_ctx.accel_lag_s if before[s] > 0.0 else rule_ctx.brake_lag_s
            nl = round(lag_s / period_s)
            idx = np.minimum(np.arange(s, e) + nl, n - 1)
            dv = float(np.sum(base.need_accel_kmhs[s:e] - pred[idx])) * period_s
        elif is_valley:  # 案 R の谷: 戻る途中のブレーキの効きを入れる
            nb = round(rule_ctx.brake_lag_s / period_s)
            idx = np.minimum(np.arange(s, e) + nb, n - 1)
            dv = float(np.sum(base.need_accel_kmhs[s:e] - pred[idx])) * period_s
            gain_fn = (
                _valley_boost_gain_robust if rule_ctx.valley_robust else _valley_boost_gain
            )
            gain = gain_fn(
                base, after, rule_ctx, k, s, nxt, m_rows, dv / ((s - k) * period_s), period_s
            )
            rule_ctx.comp_f_log.append(gain)
        else:
            coast = aux["coast_a"][s:e] if before[s] > 0.0 else aux["coast_b"][s:e]
            dv = float(np.sum(base.need_accel_kmhs[s:e] - coast)) * period_s
        add[k:s] += gain * dv / ((s - k) * period_s)
        dv_total += abs(dv)
        n_ev += 1
    return add, dv_total, n_ev, trimmed


def _valley_boost_gain(
    base: ForwardPlan,
    after: np.ndarray,
    ctx: GapContext,
    k: int,
    s: int,
    nxt: int,
    m_target: int,
    add0: float,
    period_s: float,
) -> float:
    """R の谷の上乗せの倍率 f ∈ [0, VALLEY_BOOST_F_MAX]（2026-10-26 段10-2'）。

    谷（ブレーキ → アクセル）で前の c 行を削ると、ブレーキの効きがその分弱くなる。それを同じ行数の
    直前に配ると、ブレーキは 1 周期で数 % しか深くならず届かない。そこで配る行を max(c, 0.3 秒) 行
    にし、上乗せの量は「予測した車速のずれが 0 に戻る」ように決める。add0 ＝ 今の式の Δv ÷
    （配る行数 × 周期）[km/h/s]。f を二分法（20 回）で決める: 局所の列（削った列の行 k〜s−1 を、
    本体の表で `table.invert(v, 要る加速度 + f × add0)` で解き直す。u ≥ 0 の行は元のまま）で
    両ペダルの位置を追い、cum ＝ Σ_{r0..r1}（予測した加速度 − 基準の加速度）× 周期（r0 ＝ 削った行の
    先頭 − m_target − 10、r1 ＝ 次のアクセルの最初の行 nxt ＋ round(0.5/周期)）が 0 になる f。
    f が大きいほど減速が増えて cum は下がる。f = 上限でも cum > 0 なら上限、
    f = 0 で cum < 0 なら 0。
    """
    n = len(after)
    r0 = max(s - m_target - 10, 0)
    r1 = min(nxt + round(0.5 / period_s), n - 1)
    st = max(r0 - 40, 0)
    en = r1 + 1
    e0 = after[st:en].copy()
    v_loc, need_loc = ctx.v[st:en], base.need_accel_kmhs[st:en]
    acc_loc = ctx.ref_acc[st:en]
    pa = _track_one(e0, True, ctx, period_s)  # アクセルは f に依らない
    ks, ss = k - st, s - st

    def cum_of(f: float) -> float:
        e = e0.copy()
        u, _ = ctx.table.invert(v_loc[ks:ss], need_loc[ks:ss] + f * add0)
        u = np.asarray(u, dtype=float)
        e[ks:ss] = np.where(u < 0.0, -(ctx.brake_min_pct - u), e[ks:ss])
        np.clip(e, -EFFORT_LIMIT_PCT, EFFORT_LIMIT_PCT, out=e)
        pb = _track_one(e, False, ctx, period_s)
        pred = predicted_accel(
            ctx.table, v_loc, pa, pb, track=ctx.track, accel_min_pct=ctx.accel_min_pct,
            brake_min_pct=ctx.brake_min_pct, period_s=period_s, **ctx.side_kw(st, en),
        )
        return float(np.sum((pred[r0 - st :] - acc_loc[r0 - st :]) * period_s))

    if cum_of(0.0) <= 0.0:
        return 0.0
    if cum_of(VALLEY_BOOST_F_MAX) > 0.0:
        return VALLEY_BOOST_F_MAX
    lo, hi = 0.0, VALLEY_BOOST_F_MAX
    for _ in range(VALLEY_BOOST_ITERS):
        mid = 0.5 * (lo + hi)
        if cum_of(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def _valley_boost_gain_robust(
    base: ForwardPlan,
    after: np.ndarray,
    ctx: GapContext,
    k: int,
    s: int,
    nxt: int,
    m_target: int,
    add0: float,
    period_s: float,
) -> float:
    """R の谷の上乗せの倍率 f の頑健版（2026-10-27 段12 谷。switch_gap_valley_robust）。

    局所の列の作り方は `_valley_boost_gain` の cum_of と同じ。違いは点数だけ:
    案 Y（数え始め）: 積み上がりを上乗せを配る最初の行 k の 1 行前（w0）から r1 まで数える
    （それより前の水準のずれは f で直せないので数えない）。
    案 X: ペダルが効くまでの遅れ（アクセル・ブレーキ両方）を PEAK_ROBUST_LAG_ROWS 行ずらした
    3 通り（−・0・+）の PedalTrack で予測し、点数 ＝ 3 通りのうち最大の
    「[w0, r1] の running cum の絶対値の最大」。位置の追跡は ctx.track のまま。
    探し方: (1) f を 0〜VALLEY_BOOST_F_MAX の VALLEY_ROBUST_GRID_F 刻みで全部採点して最小の格子点 j
    （同点は小さい f）、(2) 区間 [grid[j−1], grid[j+1]] を黄金分割探索で
    VALLEY_BOOST_ITERS 回詰める、
    (3) 詰めた点の点数が格子の最小より悪ければ格子の点を返す。
    """
    n = len(after)
    r0 = max(s - m_target - 10, 0)
    r1 = min(nxt + round(0.5 / period_s), n - 1)
    st = max(r0 - 40, 0)
    en = r1 + 1
    e0 = after[st:en].copy()
    v_loc, need_loc = ctx.v[st:en], base.need_accel_kmhs[st:en]
    acc_loc = ctx.ref_acc[st:en]
    pa = _track_one(e0, True, ctx, period_s)  # アクセルは f に依らない
    ks, ss = k - st, s - st
    w0 = max(k - 1, 0)
    trs = [
        replace(
            ctx.track,
            accel_effect_lag_s=max(ctx.track.accel_effect_lag_s + dr * period_s, 0.0),
            brake_effect_lag_s=max(ctx.track.brake_effect_lag_s + dr * period_s, 0.0),
        )
        for dr in (-PEAK_ROBUST_LAG_ROWS, 0, PEAK_ROBUST_LAG_ROWS)
    ]

    def score(f: float) -> float:
        e = e0.copy()
        u, _ = ctx.table.invert(v_loc[ks:ss], need_loc[ks:ss] + f * add0)
        u = np.asarray(u, dtype=float)
        e[ks:ss] = np.where(u < 0.0, -(ctx.brake_min_pct - u), e[ks:ss])
        np.clip(e, -EFFORT_LIMIT_PCT, EFFORT_LIMIT_PCT, out=e)
        pb = _track_one(e, False, ctx, period_s)
        worst = 0.0
        for trv in trs:
            pred = predicted_accel(
                ctx.table, v_loc, pa, pb, track=trv, accel_min_pct=ctx.accel_min_pct,
                brake_min_pct=ctx.brake_min_pct, period_s=period_s, **ctx.side_kw(st, en),
            )
            c = np.cumsum((pred[w0 - st :] - acc_loc[w0 - st :]) * period_s)
            worst = max(worst, float(np.max(np.abs(c))))
        return worst

    grid = np.arange(0.0, VALLEY_BOOST_F_MAX + 1e-9, VALLEY_ROBUST_GRID_F)
    grid = np.minimum(grid, VALLEY_BOOST_F_MAX)
    gs = [score(float(f)) for f in grid]
    j = int(np.argmin(gs))  # 同点は先（小さい f）
    best_f, best_s = float(grid[j]), gs[j]
    lo = float(grid[max(j - 1, 0)])
    hi = float(grid[min(j + 1, len(grid) - 1)])
    inv = (math.sqrt(5.0) - 1.0) / 2.0
    c, d = hi - inv * (hi - lo), lo + inv * (hi - lo)
    fc, fd = score(c), score(d)
    for _ in range(VALLEY_BOOST_ITERS):
        if fc <= fd:
            hi, d, fd = d, c, fc
            c = hi - inv * (hi - lo)
            fc = score(c)
        else:
            lo, c, fc = c, d, fd
            d = lo + inv * (hi - lo)
            fd = score(d)
    f_g = 0.5 * (lo + hi)
    if score(f_g) <= best_s:
        return min(max(f_g, 0.0), VALLEY_BOOST_F_MAX)
    return best_f


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
    need_accel: np.ndarray | None = None,
    coast_a: np.ndarray | None = None,
    coast_b: np.ndarray | None = None,
    rule_ctx: GapContext | None = None,
) -> ReturnGapStats:
    """アクセル⇔ブレーキの全部の切り替えで、すき間（戻す側の指令が 0 の行数）を戻る時間 T 以上にする
    （effort・flag を書き換える。2026-10-17 段2o-9）。

    すき間 = 次のペダルの最初の行 − 戻す側の最後の行 − 1。T は戻す側の最後の指令の深さで決まる
    （`ReturnTime.gap_rows`）。足りないときは左から順に:
    - 戻す側が停車保持（FLAG_STOP）、または前倒しで足りる前に戻す側の連なりが無くなるとき:
      次のペダルの最初の行から 0 に書き換えて遅らせる（停車保持を早く離すとクリープで動き出す）
    - それ以外: 戻す側の最後の行を 1 行ずつ 0（惰行）にして前倒しする（削るたびに深さを測り直す。
      最後の部分はたいてい遊びの中）

    2026-10-25 段9-b: need_accel・coast_a・coast_b（行ごとの要る加速度・アクセル側／ブレーキ側の
    惰行の加速度）を渡すと、停車保持・先置きが絡まないアクセル ⇔ ブレーキの切り替えは、前で c 行
    削り（先頭の 1 行は残す）・後ろで d 行遅らせる（次のペダルの連なりの中で、1 行は残す）組み合わせ
    のうち gap + c + d ≥ gap_rows(前の残った最後の行) を満たし、損 Σ|要る加速度 − 惰行の加速度|
    ×周期（惰行にした行）が最小のものを選ぶ（同点は d の小さい方・c の小さい方＝今までの前で削る）。
    条件を満たす組が無ければ今までの手順。None は今までと完全に同じ。

    2026-10-26 段10-2: rule_ctx（案 R。`switch_gap_rule`）を渡すと、9-b と同じ対象（戻す側の最後の
    行・切り替え先の最初の行が FLAG_STOP・FLAG_PREPOS でない）で、切り替え先の最初の行の
    惰行の加速度が負の切り替えを、ペダルの実際の位置で置き直す（すき間が足りていても処理する）。
    山（アクセル → ブレーキ）は `_rule_peak`、谷（ブレーキ → アクセル）は `_rule_valley`。
    置けなければ今までの手順（9-b が on なら 9-b）。None は今までと完全に同じ。

    2026-10-26 段10-3: rule_ctx.min_peak（案 C。`switch_gap_min_peak`）のとき、9-b と同じ対象で、
    切り替え先の最初の行の惰行の加速度（切り替え先の側）が負の切り替え（停車前後の低速で惰行が
    負でない所は C の対象外＝今までの手順。2026-10-26 追加）は `_min_peak_split` で (c, d) を
    選ぶ（すき間が足りていても、戻す側の実際の位置が次のペダルの最初の行に間に合わないなら
    選び直す）。組が無ければ今までの手順
    （9-b が on なら 9-b）。
    """
    n = len(effort)
    min_loss = need_accel is not None and coast_a is not None and coast_b is not None
    stats = ReturnGapStats(defaults_used=ret.defaults_used())
    do_choose = rule_ctx is not None and rule_ctx.use_rule and rule_ctx.choose
    before = effort.copy() if do_choose else None  # 段11-5: 選び直しの「元」
    flag0 = flag.copy() if do_choose else None
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
            a = last  # 戻す側の連なり（同じ向きの 0 でない行）の先頭
            leaving_accel = effort[last] > 0.0
            while a > 0 and effort[a - 1] != 0.0 and (effort[a - 1] > 0.0) == leaving_accel:
                a -= 1
            if (
                rule_ctx is not None
                and rule_ctx.use_rule
                and flag[last] not in (FLAG_STOP, FLAG_PREPOS)
                and flag[i] not in (FLAG_STOP, FLAG_PREPOS)
                and float((rule_ctx.coast_b if leaving_accel else rule_ctx.coast_a)[i]) < 0.0
            ):
                done = (
                    _rule_peak(effort, flag, a, last, i, rule_ctx, period_s, stats)
                    if leaving_accel
                    else _rule_valley(effort, flag, a, last, i, rule_ctx, period_s, stats)
                )
                if done is not None:  # R が置いた。続きは (last, i) から
                    last, i = done
                    continue
            peak_pick = None
            if (
                rule_ctx is not None and rule_ctx.min_peak
                and float((rule_ctx.coast_b if leaving_accel else rule_ctx.coast_a)[i]) < 0.0
            ):  # 段10-3: 案 C（9-b と同じ対象。R と同じく切り替え先の惰行の加速度が負のものだけ）
                peak_pick = _min_peak_split(
                    effort, flag, a, last, i, gap >= need, period_s, rule_ctx
                )
                if peak_pick is not None and (peak_pick[0] > 0 or peak_pick[1] > 0):
                    stats.n_min_peak += 1
                    if peak_pick[0] > 0:
                        stats.min_peak_spans.append((last - peak_pick[0] + 1, last + 1))
            if gap < need or peak_pick is not None:  # C は選んだ組を適用（(0, 0) は無変更）
                pick = peak_pick
                if pick is None and gap < need and min_loss:
                    pick = _min_loss_split(
                        effort, flag, a, last, i, gap, ret, period_s,
                        need_accel, coast_a, coast_b,
                    )
                if pick is not None:  # 段9-b・C: 前で c 行削り、後ろで d 行遅らせる
                    c, d = pick
                    if c > 0:
                        effort[last - c + 1 : last + 1] = 0.0
                        flag[last - c + 1 : last + 1] = FLAG_COAST
                        stats.n_cut += 1
                        stats.cut_s += c * period_s
                        last -= c
                    if d > 0:
                        effort[i : i + d] = 0.0
                        flag[i : i + d] = FLAG_COAST
                        stats.n_delayed += 1
                        stats.delayed_s += d * period_s
                        i += d
                    last = i
                    i += 1
                    continue
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
    if do_choose and stats.rule_switches:  # 段11-5
        _choose_pass(effort, flag, before, flag0, rule_ctx, period_s, stats)
    if rule_ctx is not None:  # 段10-6: R の判定と同じ位置の物差しで数える
        stats.n_violations = count_gap_violations_by_position(
            effort, ctx=rule_ctx, period_s=period_s
        )
        stats.by_position = True
    else:
        stats.n_violations = count_gap_violations(effort, ret=ret, period_s=period_s)
    return stats


def _rule_peak(
    effort: np.ndarray,
    flag: np.ndarray,
    a: int,
    last: int,
    i: int,
    ctx: GapContext,
    period_s: float,
    stats: ReturnGapStats,
) -> tuple[int, int] | None:
    """案 R の山（アクセル [a, last] → ブレーキ i〜。2026-10-26 段10-2）。

    頂点 k_v ＝ a から i + 1 秒までで、基準の加速度（車の時刻）が一度惰行以上になった後に惰行を
    初めて下回る行（連なりの頭でまだ減速中の行は頂点にしない）。頂点が無いときは None
    （今までの手順）。

    アクセルを戻し始める行 s ∈ [max(a+1, last+1−RANGE), last+1]（アクセルは延ばさない。
    RANGE ＝ RULE_PEAK_RANGE_S。連なりの頭まで探すと水準のずれが面積に入るため、
    変わり目の近くに限る。2026-10-26）は、
    (c) 変わり目の真ん中で選ぶ。候補 s ごとに局所の列を作り（行 s〜last のアクセルを 0、
    アクセルの位置が min に着く行 r（`position_return_row`）の次の行 b ＝ r + 1 より前の
    ブレーキ行 i〜b−1 を 0。b ≤ i ならブレーキはそのまま。実機の安全網（実開度の判定）は
    前の周期に読んだ実開度で見るので、次のペダルは r の 1 周期後の行から。旧 `ReturnTime` の
    ＋1 周期と同じ理由）、
    両ペダルの位置を追って `predicted_accel` を出す。r0 ＝（候補の s の最小）− 10 行、
    r1 ＝（候補の b の最大）＋ round(0.5/周期) 行（n−1 で切る）を全候補で同じにして、
    cum_s ＝ Σ_{r0..r1}（予測した加速度 − 基準の加速度）× 周期 の |cum_s| が最小の s（同点は
    遅い方）。(c) の真ん中は、ブレーキが効くまでを含めた変わり目の、面積で見た真ん中（Claude の
    解釈 2026-10-26。議論の見積もりは惰行までしか見ていなかった）。変わり目を通り過ぎた後で、
    予測車速と基準のずれの積み上がりが 0 に戻る位置。
    アクセルの位置が min に戻らない候補は使わない（全部そうなら None）。
    2026-10-26 段11-4（ProblemReport_20261026_2）: ctx.peak_from_corner が True なら、下限を
    固定の RANGE でなく基準から決める。k_pk ＝ 頂点 k_v の手前から、基準の加速度が（後ろ向きに
    見て）厳密に高い間さかのぼった行（頂点の手前の最後の極大。平らなら頂点側の端）。
    下限 s_lo ＝ max(a+1, k_pk − m)、m ＝（k_pk で戻したときアクセルの位置が min に着くまでの行数
    r − k_pk ＋ 1 周期）＋ ceil(アクセルが効くまでの遅れ / 周期)。着かないときは
    m ＝ ceil(遅れ / 周期) ＋ 1。データ由来（基準とペダルの値から決まる）。
    2026-10-27 段12: ctx.peak_robust が True なら採点だけを変える（候補の選び方は同じ）。
    案 Y（数え始め）: s_last ＝ 候補の s の最大、b_last ＝ その b、R0 ＝ s_last −（b_last − s_last）
    − ceil(アクセルが効くまでの遅れ / 周期）。候補 s の数え始め w0 ＝ max(min(s, R0) − 1, r0)
    （戻す前の加速中にたまった水準のずれを数えない）。案 X（遅れのずれ）: 効くまでの遅れ
    （アクセル・ブレーキ両方）を PEAK_ROBUST_LAG_ROWS 行ずつ前後にずらした 3 通りで予測し、
    点数 ＝ 3 通りの [w0, r1] の積み上がり（running cum）の絶対値の最大のうち最大。最小の s
    （同点は遅い方）。
    適用: s〜last を 0、ブレーキを b から。戻り値は呼び出し側が続きを数える (last, i)。
    """
    n = len(effort)
    k_end = min(n - 1, i + round(1.0 / period_s))
    below_all = ctx.ref_acc[a : k_end + 1] < ctx.coast_a[a : k_end + 1]
    # 連なりの頭で基準がまだ減速中（谷の立ち上がり）の行は頂点ではない。一度惰行以上になってから探す
    above = np.flatnonzero(~below_all)
    if above.size == 0:
        return None
    if np.flatnonzero(below_all[above[0] :]).size == 0:
        return None
    sign_skip = (FLAG_STOP, FLAG_PREPOS)
    j = i  # ブレーキの連なりの終わり（停車保持・先置きの行の手前まで）
    while j < n and effort[j] < 0.0 and flag[j] not in sign_skip:
        j += 1
    st = max(a - 40, 0)
    en1 = min(n, last + 1 + round(2.5 / period_s))
    base_a = _track_base(effort[st:en1], True, ctx, period_s)  # 候補は途中から違うだけ
    cands: list[tuple[int, int]] = []  # (s, accel の位置が min に着く行 b)
    s_lo = a + 1
    if ctx.peak_from_corner:
        # 段11-4: 頂点 k_v の手前の最後の極大 k_pk（平らなら頂点側の端）の、戻る行数＋遅れ手前から
        k_v = a + int(above[0]) + int(np.flatnonzero(below_all[above[0] :])[0])
        k_pk = k_v - 1
        while k_pk - 1 >= a and ctx.ref_acc[k_pk - 1] > ctx.ref_acc[k_pk] + 1e-6:
            k_pk -= 1
        lag_rows = math.ceil(ctx.track.accel_effect_lag_s / period_s)
        e = effort[st:en1].copy()
        e[max(k_pk, a) - st : last + 1 - st] = 0.0
        pos = _track_from(base_a, e, True, ctx, period_s)
        r = position_return_row(pos, k_pk - st, ctx.accel_min_pct)
        m = lag_rows + 1 if r is None else (r + st - k_pk) + 1 + lag_rows
        s_lo = max(s_lo, k_pk - m)
        s_lo = min(s_lo, last + 1)
    elif RULE_PEAK_RANGE_S is not None:
        s_lo = max(s_lo, last + 1 - round(RULE_PEAK_RANGE_S / period_s))
    for s in range(s_lo, last + 2):
        e = effort[st:en1].copy()
        e[s - st : last + 1 - st] = 0.0
        pos = _track_from(base_a, e, True, ctx, period_s)
        r = position_return_row(pos, s - st, ctx.accel_min_pct)
        if r is not None:
            # 安全網は前の周期の実開度で判定する。行 r で着くなら次のペダルは行 r + 1 から
            cands.append((s, r + 1 + st))
    if not cands:
        return None
    r0 = max(min(s for s, _ in cands) - 10, 0)
    r1 = min(max(b for _, b in cands) + round(0.5 / period_s), n - 1)
    en2 = r1 + 1
    st = max(r0 - 40, 0)
    v_loc, acc_loc = ctx.v[st:en2], ctx.ref_acc[st:en2]
    e_full = effort[st:en2]
    base_a = _track_base(e_full, True, ctx, period_s)
    base_b = _track_base(e_full, False, ctx, period_s)
    kw = {
        "track": ctx.track, "accel_min_pct": ctx.accel_min_pct,
        "brake_min_pct": ctx.brake_min_pct, "period_s": period_s,
        **ctx.side_kw(st, en2),
    }
    full = predicted_accel(ctx.table, v_loc, base_a.pos, base_b.pos, **kw) - acc_loc
    prefix = np.concatenate([[0.0], np.cumsum(full * period_s)])  # prefix[k] ＝ Σ_{行 < k}
    lag_max = max(ctx.track.accel_effect_lag_s, ctx.track.brake_effect_lag_s)
    margin = math.ceil(lag_max / period_s) + 2
    best: tuple[float, int, int] | None = None
    if ctx.peak_robust:
        # 段12: 案 Y（数え始め）と案 X（遅れのずれ 3 通り）。位置の追跡は ctx.track のまま
        s_last = max(s for s, _ in cands)
        b_last = next(bb for ss, bb in cands if ss == s_last)
        lag_rows_y = math.ceil(ctx.track.accel_effect_lag_s / period_s)
        big_r0 = s_last - (b_last - s_last) - lag_rows_y
        kws = []
        for dr in (-PEAK_ROBUST_LAG_ROWS, 0, PEAK_ROBUST_LAG_ROWS):
            trv = replace(
                ctx.track,
                accel_effect_lag_s=max(ctx.track.accel_effect_lag_s + dr * period_s, 0.0),
                brake_effect_lag_s=max(ctx.track.brake_effect_lag_s + dr * period_s, 0.0),
            )
            kws.append(dict(kw, track=trv))
        for s, b in cands:
            e = e_full.copy()
            e[s - st : last + 1 - st] = 0.0
            z = min(max(b, i), j)
            if z > i:
                e[i - st : z - st] = 0.0
            pa = _track_from(base_a, e, True, ctx, period_s)
            pb = _track_from(base_b, e, False, ctx, period_s) if z > i else base_b.pos
            w0 = max(min(s, big_r0) - 1, r0)
            score = 0.0
            for kwv in kws:
                pr = predicted_accel(ctx.table, v_loc, pa, pb, **kwv) - acc_loc
                c = np.cumsum(pr[w0 - st : r1 - st + 1] * period_s)
                score = max(score, float(np.max(np.abs(c))))
            if best is None or score <= best[0] + 1e-9:
                best = (score, s, b)
    for s, b in [] if ctx.peak_robust else cands:  # 段12 の採点は上で済み
        e = e_full.copy()
        e[s - st : last + 1 - st] = 0.0
        z = min(max(b, i), j)
        if z > i:
            e[i - st : z - st] = 0.0
        pa = _track_from(base_a, e, True, ctx, period_s)
        pb = _track_from(base_b, e, False, ctx, period_s) if z > i else base_b.pos
        # 行 s より前は全候補で同じ（位置が違うのは行 s + 1 から）。続きだけ表を引く
        ks = s - st
        k1 = max(ks - margin, 0)
        kw_t = dict(kw, **ctx.side_kw(st + k1, en2))
        tail = predicted_accel(ctx.table, v_loc[k1:], pa[k1:], pb[k1:], **kw_t)[ks - k1 :]
        cum = float(prefix[ks] - prefix[r0 - st] + np.sum((tail - acc_loc[ks:]) * period_s))
        if best is None or abs(cum) <= best[0] + 1e-9:
            best = (abs(cum), s, b)
    assert best is not None
    _, s, b = best
    if last + 1 > s:
        effort[s : last + 1] = 0.0
        flag[s : last + 1] = FLAG_COAST
        stats.n_cut += 1
        stats.cut_s += (last + 1 - s) * period_s
        stats.rule_peak_spans.append((s, last + 1))
    z = min(max(b, i), j)
    if z > i:
        effort[i:z] = 0.0
        flag[i:z] = FLAG_COAST
        stats.n_delayed += 1
        stats.delayed_s += (z - i) * period_s
    stats.n_rule_peak += 1
    stats.rule_switches.append(("peak", a, last, i, j, min(s, last + 1), z))
    if z >= j:  # ブレーキの連なりが全部消えた
        return s - 1, j
    return z, z + 1


def _rule_valley(
    effort: np.ndarray,
    flag: np.ndarray,
    a: int,
    last: int,
    i: int,
    ctx: GapContext,
    period_s: float,
    stats: ReturnGapStats,
) -> tuple[int, int] | None:
    """案 R の谷（ブレーキ [a, last] → アクセル i〜。2026-10-26 段10-2）。すき間は前（アクセルは
    遅らせない）。

    ブレーキの連なりの最後から c 行ずつ 0 にし（先頭の 1 行は残す）、ブレーキの実際の位置が min に
    着く行 r の次の行（r + 1。安全網は前の周期の実開度で見るので 1 周期の余裕を取る。
    旧 `ReturnTime` の ＋1 周期と同じ）が i 以下になる最小の c を選ぶ。もともと i 以下（c = 0）
    なら何もしない。先頭の 1 行を
    残しても足りなければ None（今までの手順）。
    """
    n = len(effort)
    st = max(a - 40, 0)
    en = min(n, i + 1)
    base_b = _track_base(effort[st:en], False, ctx, period_s)
    for c in range(0, last - a + 1):
        e = effort[st:en].copy()
        if c > 0:
            e[last - c + 1 - st : last + 1 - st] = 0.0
        pos = _track_from(base_b, e, False, ctx, period_s)
        r = position_return_row(pos, last - c + 1 - st, ctx.brake_min_pct)
        if r is not None and r + 1 + st <= i:
            if c > 0:
                jn = i  # アクセルの連なりの終わり（選び直しの記録用）
                while jn < n and effort[jn] > 0.0 and flag[jn] not in (FLAG_STOP, FLAG_PREPOS):
                    jn += 1
                stats.rule_switches.append(("valley", a, last, i, jn, last - c + 1, i))
                effort[last - c + 1 : last + 1] = 0.0
                flag[last - c + 1 : last + 1] = FLAG_COAST
                stats.n_cut += 1
                stats.cut_s += c * period_s
                stats.n_rule_valley += 1
                stats.rule_valley_spans.append((last - c + 1, last + 1))
            return i, i + 1
    return None


def _choose_score(
    e_loc: np.ndarray, lo: int, w0: int, w1: int, ctx: GapContext, period_s: float
) -> float:
    """局所の列 e_loc（行 lo〜）の、行 w0〜w1 の予測した積み上げの偏差の最大の大きさ（2026-10-26
    段11-5）。両ペダルの位置を追い、
    車の時刻の（予測した加速度 − 基準の傾き）× 周期を w0 から積み上げ、
    その |積み上げ| の最大。案 C（`_min_peak_split`）と同じ物差し。"""
    hi = lo + len(e_loc)
    pa = _track_one(e_loc, True, ctx, period_s)
    pb = _track_one(e_loc, False, ctx, period_s)
    pred = predicted_accel(
        ctx.table, ctx.v[lo:hi], pa, pb, track=ctx.track, accel_min_pct=ctx.accel_min_pct,
        brake_min_pct=ctx.brake_min_pct, period_s=period_s, **ctx.side_kw(lo, hi),
    )
    d = (pred - ctx.ref_acc[lo:hi]) * period_s
    cum = np.cumsum(d[w0 - lo : w1 - lo + 1])
    return float(np.max(np.abs(cum))) if cum.size else 0.0


def _choose_back_rows(
    e: np.ndarray,
    f: np.ndarray,
    off: int,
    rec: tuple,
    z: int,
    before: np.ndarray,
    flag0: np.ndarray,
    ctx: GapContext,
) -> None:
    """「後ろで取る」の書き換えを e・f（行 off〜）に適用する（2026-10-26 段11-5）。
    前のペダル: R が削った行 [cut0, last] を元（before）へ戻す。
    谷は 7-c の上乗せも外す（上乗せなしの
    列 ctx.alt_effort があれば、いま踏んでいる同じ向きの行 [a, cut0) をそれへ）。
    後ろのペダル: 行 i〜z−1 を惰行にし、R が遅らせた行 [z, zr) は元へ戻す。"""
    kind, a, last, i, _j, cut0, zr = rec
    n_loc = len(e)

    def put(r0: int, r1: int, src_e: np.ndarray, src_f: np.ndarray) -> None:
        r0, r1 = max(r0, off), min(r1, off + n_loc)
        if r0 < r1:
            e[r0 - off : r1 - off] = src_e[r0:r1]
            f[r0 - off : r1 - off] = src_f[r0:r1]

    put(cut0, last + 1, before, flag0)
    if kind == "valley" and ctx.alt_effort is not None:
        r0, r1 = max(a, off), min(cut0, off + n_loc)
        for r in range(r0, r1):
            cur = e[r - off]
            alt = ctx.alt_effort[r]
            if cur < 0.0 and alt < 0.0:  # 谷の戻す側はブレーキ（負）。山の R は上乗せしない
                e[r - off] = alt
    if z < zr:
        put(z, zr, before, flag0)
    r0, r1 = max(i, off), min(z, off + n_loc)
    if r0 < r1:
        e[r0 - off : r1 - off] = 0.0
        f[r0 - off : r1 - off] = FLAG_COAST


def _choose_pass(
    effort: np.ndarray,
    flag: np.ndarray,
    before: np.ndarray,
    flag0: np.ndarray,
    ctx: GapContext,
    period_s: float,
    stats: ReturnGapStats,
) -> None:
    """R が置いた山・谷ごとに「R のまま」と「後ろで取る」を選ぶ（2026-10-26 段11-5。
    ProblemReport_20261026_2）。ensure_return_gap の最後（R・9-b・C の処理が全部済んだ列）で呼ぶ。

    後ろで取る ＝ 前のペダル（R が削った行）を元（R の前の列 `before`）へ戻して要る間踏み、
    次のペダルを
    前のペダルの位置が min に着く行 r の次の行（z ＝ r + 1。安全網は前の周期の実開度で見る）まで
    遅らせる（9-最終の形）。谷は 7-c の上乗せも外す（`ctx.alt_effort`）。
    位置が着かない・次のペダルの
    連なりが全部消える（z ≥ 連なりの終わり）・停車保持／先置きの行を書き換えるときは作れない
    （R のまま）。
    点数 ＝ 切り替えの最初の行 i の前後 switch_gap_peak_window_s 秒（行 w0 = i − W 〜 i + W）の、
    予測した（加速度 − 基準の傾き）× 周期を w0 から積み上げた |積み上げ| の最大（`_choose_score`）。
    後ろが厳密に小さいときだけ後ろ（同点は R）。時刻順に、
    いまの列（先の選び直しを反映）で点数を出す。
    """
    n = len(effort)
    skip = (FLAG_STOP, FLAG_PREPOS)
    w = max(int(round(ctx.peak_window_s / period_s)), 1)
    warm = 80
    for rec in sorted(stats.rule_switches, key=lambda r: r[3]):
        kind, a, last, i, j, cut0, zr = rec
        leaving_accel = kind == "peak"
        mn = ctx.accel_min_pct if leaving_accel else ctx.brake_min_pct
        w0, w1 = max(i - w, 0), min(n - 1, i + w)
        lo = max(min(w0, cut0, a) - warm, 0)
        # 1. 前のペダルを元へ戻した列で、位置が min に着く行を探す
        hz = min(n, max(w1 + 1, last + 1 + round(4.0 / period_s)) + 1)
        e_pre = effort[lo:hz].copy()
        e_pre[max(cut0, lo) - lo : last + 1 - lo] = before[max(cut0, lo) : last + 1]
        pos = _track_one(e_pre, leaving_accel, ctx, period_s)
        r = position_return_row(pos, last + 1 - lo, mn)
        score_r = None
        score_b = None
        choice = "R"
        z = None if r is None else max(r + 1 + lo, i)
        ok = z is not None and z < j and not (
            (flag0[cut0 : last + 1] == FLAG_STOP).any()
            or (flag0[cut0 : last + 1] == FLAG_PREPOS).any()
            or any(int(flag[k]) in skip for k in range(i, min(z, n)))
        )
        hi = min(n, max(w1 + 1, (z or 0) + 1))
        e_r = effort[lo:hi].copy()
        score_r = _choose_score(e_r, lo, w0, w1, ctx, period_s)
        if ok:
            e_b, f_b = e_r.copy(), flag[lo:hi].copy()
            _choose_back_rows(e_b, f_b, lo, rec, z, before, flag0, ctx)
            score_b = _choose_score(e_b, lo, w0, w1, ctx, period_s)
            if score_b < score_r - 1e-9:
                choice = "back"
                _choose_back_rows(effort, flag, 0, rec, z, before, flag0, ctx)
                if leaving_accel:
                    stats.n_choose_back_peak += 1
                else:
                    stats.n_choose_back_valley += 1
        stats.choose_log.append((i, kind, score_r, score_b, choice))


def _track_one(e: np.ndarray, accel: bool, ctx: GapContext, period_s: float) -> np.ndarray:
    """局所の列 e から片方のペダルの実際の位置を作る（2026-10-26 段10-3）。"""
    tr = ctx.track
    if accel:
        cmd = np.maximum(np.where(e > 0.0, e, ctx.accel_min_pct), ctx.accel_min_pct)
        return _follow_pedal(
            cmd, None, tr.accel_delay_s, tr.accel_max_rate_pct_s, tr.accel_max_accel_pct_s2,
            ctx.accel_min_pct, period_s,
        )
    cmd = np.maximum(np.where(e < 0.0, -e, ctx.brake_min_pct), ctx.brake_min_pct)
    return _follow_pedal(
        cmd, None, tr.brake_delay_s, tr.brake_max_rate_pct_s, tr.brake_max_accel_pct_s2,
        ctx.brake_min_pct, period_s,
    )


@dataclass
class _TrackBase:
    """局所の列の片方のペダルを追った結果（`_follow_resume` の元）。"""

    cmd: np.ndarray
    pos: np.ndarray
    vlog: np.ndarray


def _track_base(e: np.ndarray, accel: bool, ctx: GapContext, period_s: float) -> _TrackBase:
    """`_track_one` と同じ位置の列を、途中から続けるための状態つきで作る（2026-10-26 段10-2'）。"""
    if accel:
        cmd = np.maximum(np.where(e > 0.0, e, ctx.accel_min_pct), ctx.accel_min_pct)
    else:
        cmd = np.maximum(np.where(e < 0.0, -e, ctx.brake_min_pct), ctx.brake_min_pct)
    vlog = np.zeros(len(e))
    tr = ctx.track
    if accel:
        pos = _follow_pedal(
            cmd, None, tr.accel_delay_s, tr.accel_max_rate_pct_s, tr.accel_max_accel_pct_s2,
            ctx.accel_min_pct, period_s, vlog=vlog,
        )
    else:
        pos = _follow_pedal(
            cmd, None, tr.brake_delay_s, tr.brake_max_rate_pct_s, tr.brake_max_accel_pct_s2,
            ctx.brake_min_pct, period_s, vlog=vlog,
        )
    return _TrackBase(cmd, pos, vlog)


def _track_from(
    base: _TrackBase, e_new: np.ndarray, accel: bool, ctx: GapContext, period_s: float
) -> np.ndarray:
    """base の列と途中から違う列 e_new の位置（`_track_one(e_new, ...)` と同じ値）。"""
    tr = ctx.track
    if accel:
        cmd = np.maximum(np.where(e_new > 0.0, e_new, ctx.accel_min_pct), ctx.accel_min_pct)
        return _follow_resume(
            cmd, base.cmd, base.pos, base.vlog, tr.accel_delay_s, tr.accel_max_rate_pct_s,
            tr.accel_max_accel_pct_s2, ctx.accel_min_pct, period_s,
        )
    cmd = np.maximum(np.where(e_new < 0.0, -e_new, ctx.brake_min_pct), ctx.brake_min_pct)
    return _follow_resume(
        cmd, base.cmd, base.pos, base.vlog, tr.brake_delay_s, tr.brake_max_rate_pct_s,
        tr.brake_max_accel_pct_s2, ctx.brake_min_pct, period_s,
    )


def _min_peak_split(
    effort: np.ndarray,
    flag: np.ndarray,
    a: int,
    last: int,
    i: int,
    gap_ok: bool,
    period_s: float,
    ctx: GapContext,
) -> tuple[int, int] | None:
    """案 C（2026-10-26 段10-3）: 前を c 行削り・後ろを d 行遅らせる組 (c, d) を、予測した最大偏差が
    最小になるように選ぶ（9-b の `_min_loss_split` の比べ方を直したもの）。

    前の連なりは [a, last]（先頭の 1 行は残す。c ≤ min(last − a, 窓の行数)）、後ろの連なりは
    i からの同じ向きの 0 でない行（停車保持・先置きの手前まで。1 行は残す）。すき間の条件は
    「戻す側の実際の位置（PedalTrack で追う）が min に着く行 r の次の行 r + 1 ≤ i + d」
    （安全網は前の周期の実開度で見るので 1 周期の余裕を取る。旧 `ReturnTime` の ＋1 周期と同じ）。
    候補は、各 c について
    条件を満たす最小の d と、各 d について条件を満たす最小の c の和集合（d は窓の行数 + 1 まで。
    それより後ろは窓の外で予測が変わらず同じ点になり、同点は d の小さい方が選ばれるため）。
    各候補の列を i の前後だけの局所に作り（c > 0 で ctx.comp なら 7-c と同じ上乗せを、
    Δv ＝ Σ（要る加速度[k] − 予測[k + 遅れ]）× 周期、削った行の直前の同じペダルの最大 c 行に
    均等に配って、本体の表で解き直して入れる）、
    車の時刻の予測の加速度 − 基準の傾きを T − 窓から 0 で積み上げた最大の大きさで比べる。
    同点（1e-9 以内）は d の小さい方・c の小さい方。

    gap_ok（従来の見方ですき間が足りている）で、c = 0 の位置がすでに間に合うなら (0, 0)。
    停車保持・先置きが絡むとき・条件を満たす候補が無いときは None（今までの手順に任せる）。
    """
    skip = (FLAG_STOP, FLAG_PREPOS)
    if flag[last] in skip or flag[i] in skip or ctx.need is None:
        return None
    n = len(effort)
    leaving_accel = bool(effort[last] > 0.0)
    sign = effort[i] > 0.0
    j = i
    while j < n and effort[j] != 0.0 and (effort[j] > 0.0) == sign and flag[j] not in skip:
        j += 1
    w = max(int(round(ctx.peak_window_s / period_s)), 1)
    d_max = min(j - i - 1, w + 1)
    c_max = min(last - a, w)
    lo, hi = max(i - w, 0), min(n, i + w + 1)
    st = max(min(lo, last - 2 * c_max) - 20, 0)  # 位置を追う助走を前に取る
    base_loc = effort[st:hi].copy()
    v_loc, need_loc, acc_loc = ctx.v[st:hi], ctx.need[st:hi], ctx.ref_acc[st:hi]
    li, ii = last - st, i - st
    lag_rows = round((ctx.accel_lag_s if leaving_accel else ctx.brake_lag_s) / period_s)
    mn_l = ctx.accel_min_pct if leaving_accel else ctx.brake_min_pct

    def pred_of(e_loc: np.ndarray, pa: np.ndarray, pb: np.ndarray) -> np.ndarray:
        return predicted_accel(
            ctx.table, v_loc, pa, pb, track=ctx.track, accel_min_pct=ctx.accel_min_pct,
            brake_min_pct=ctx.brake_min_pct, period_s=period_s, **ctx.side_kw(st, hi),
        )

    def column(c: int) -> np.ndarray:  # c 行削った列（c > 0 で comp なら上乗せ入り）
        e = base_loc.copy()
        if c == 0:
            return e
        s0 = li - c + 1
        e[s0 : li + 1] = 0.0
        if not ctx.comp:
            return e
        pred = pred_of(e, _track_one(e, True, ctx, period_s), _track_one(e, False, ctx, period_s))
        idx = np.minimum(np.arange(s0, li + 1) + lag_rows, len(e) - 1)
        dv = float(np.sum(need_loc[s0 : li + 1] - pred[idx])) * period_s
        k0 = s0
        while (
            k0 > 0 and s0 - k0 < c and e[k0 - 1] != 0.0 and (e[k0 - 1] > 0.0) == leaving_accel
            and flag[st + k0 - 1] not in skip
        ):
            k0 -= 1
        if k0 < s0:
            u, _ = ctx.table.invert(v_loc[k0:s0], need_loc[k0:s0] + dv / ((s0 - k0) * period_s))
            u = np.asarray(u, dtype=float)
            if leaving_accel:
                e[k0:s0] = np.where(u > 0.0, ctx.accel_min_pct + u, e[k0:s0])
            else:
                e[k0:s0] = np.where(u < 0.0, -(ctx.brake_min_pct - u), e[k0:s0])
            np.clip(e, -EFFORT_LIMIT_PCT, EFFORT_LIMIT_PCT, out=e)
        return e

    cols: dict[int, tuple[np.ndarray, np.ndarray, int | None]] = {}
    for c in range(0, c_max + 1):
        e = column(c)
        pos = _track_one(e, leaving_accel, ctx, period_s)
        r = position_return_row(pos, li - c + 1, mn_l)
        d_c = None if r is None else max(r + 1 - ii, 0)  # r + 1: 前の周期の実開度で見る
        cols[c] = (e, pos, d_c)
        if c == 0 and gap_ok and d_c == 0:
            return 0, 0
    cand: set[tuple[int, int]] = set()
    for c, (_e, _p, d_c) in cols.items():
        if d_c is not None and d_c <= d_max:
            cand.add((c, d_c))
    for d in range(d_max + 1):
        for c in range(c_max + 1):
            d_c = cols[c][2]
            if d_c is not None and d_c <= d:
                cand.add((c, d))
                break
    if not cand:
        return None
    sl = slice(lo - st, hi - st)
    best: tuple[float, int, int] | None = None
    for c, d in sorted(cand):
        e, pos_l, _ = cols[c]
        if d > 0:
            e = e.copy()
            e[ii : ii + d] = 0.0
        pos_o = _track_one(e, not leaving_accel, ctx, period_s)
        pa, pb = (pos_l, pos_o) if leaving_accel else (pos_o, pos_l)
        pred = pred_of(e, pa, pb)
        cum = np.concatenate([[0.0], np.cumsum((pred[sl] - acc_loc[sl]) * period_s)])
        key = (float(np.max(np.abs(cum))), d, c)
        if best is None or key[0] < best[0] - 1e-9 or (
            abs(key[0] - best[0]) <= 1e-9 and key[1:] < best[1:]
        ):
            best = key
    return None if best is None else (best[2], best[1])


def _min_loss_split(
    effort: np.ndarray,
    flag: np.ndarray,
    a: int,
    last: int,
    i: int,
    gap: int,
    ret: ReturnTime,
    period_s: float,
    need_accel: np.ndarray,
    coast_a: np.ndarray,
    coast_b: np.ndarray,
) -> tuple[int, int] | None:
    """段9-b: すき間不足の切り替えで、前を c 行削り・後ろを d 行遅らせる組 (c, d) のうち、
    gap + c + d ≥ gap_rows(effort[last − c]) を満たし損が最小のものを返す。

    前の連なりは [a, last]（先頭の 1 行は残すので c ≤ last − a）、後ろの連なりは i からの同じ向きの
    0 でない行（停車保持・先置きの行の手前まで。1 行は残すので d ≤ 連なりの長さ − 1）。
    損 ＝ Σ|要る加速度 − 惰行の加速度| × 周期（前はそのペダル側、後ろは次のペダル側の惰行）。
    同点は d の小さい方・c の小さい方。停車保持・先置きが絡むとき・組が無いときは None
    （今までの手順に任せる）。
    """
    skip = (FLAG_STOP, FLAG_PREPOS)
    if flag[last] in skip or flag[i] in skip:
        return None
    n = len(effort)
    sign = effort[i] > 0.0
    j = i
    while j < n and effort[j] != 0.0 and (effort[j] > 0.0) == sign and flag[j] not in skip:
        j += 1
    d_max = j - i - 1
    front_coast = coast_a if effort[last] > 0.0 else coast_b
    back_coast = coast_b if effort[last] > 0.0 else coast_a
    front_loss = np.concatenate(
        [[0.0], np.cumsum(np.abs(need_accel[last:a:-1] - front_coast[last:a:-1]))]
    ) * period_s  # front_loss[c] ＝ 後ろから c 行削った損（先頭 a は残す）
    back_loss = np.concatenate(
        [[0.0], np.cumsum(np.abs(need_accel[i : i + d_max] - back_coast[i : i + d_max]))]
    ) * period_s
    best: tuple[float, int, int] | None = None
    for c in range(0, last - a + 1):
        for d in range(0, d_max + 1):
            if gap + c + d < ret.gap_rows(float(effort[last - c]), period_s):
                continue
            key = (float(front_loss[c] + back_loss[d]), d, c)
            if best is None or key[0] < best[0] - 1e-12 or (
                abs(key[0] - best[0]) <= 1e-12 and key[1:] < best[1:]
            ):
                best = key
    return None if best is None else (best[2], best[1])


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


def count_gap_violations_by_position(
    effort: np.ndarray, *, ctx: GapContext, period_s: float
) -> int:
    """2026-10-26 段10-6: 案 R と同じ物差しのすき間不足の数。アクセル⇔ブレーキの切り替えで、
    戻す側の位置（PedalTrack で追う）が min＋0.05 に着く行 ＋ 1 が次のペダルの最初の行より
    後なら不足。
    `count_gap_violations`（指令の最後の値で決める戻る時間）は R が置いた切り替えを不足と数える。"""
    pa, pb = track_pedal_position(
        effort, period_s=period_s, track=ctx.track,
        accel_min_pct=ctx.accel_min_pct, brake_min_pct=ctx.brake_min_pct,
    )
    count = 0
    last = -1
    for i in np.flatnonzero(effort != 0.0).tolist():
        if last >= 0 and (effort[last] > 0.0) != (effort[i] > 0.0):
            leaving_accel = effort[last] > 0.0
            r = position_return_row(
                pa if leaving_accel else pb, last,
                ctx.accel_min_pct if leaving_accel else ctx.brake_min_pct,
            )
            if r is None or r + 1 > i:
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
    in_range_only: bool = False,
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

    2026-10-25 段9-c: in_range_only=True のとき、戻しの層を学習データの範囲の中だけで使う。
    戻し量 mx − u が meta accel_release_max_pct を超える点は戻し量 0（本体の表）で引き、
    直近の最大 u（mx）が meta accel_release_peak_max_pct を超える時刻は戻しの層を使わない
    （戻し量 0 ＝ 本体のまま。踏み増しの層は別）。戻しの表があるのに meta が無ければエラー。
    """
    rel_on = table.accel_release_grid is not None and table.accel_release_accel is not None
    push_on = table.accel_push_grid is not None and table.accel_push_accel is not None
    rel_cap = peak_cap = None
    if in_range_only and rel_on:
        rel_cap = table.meta.get("accel_release_max_pct")
        peak_cap = table.meta.get("accel_release_peak_max_pct")
        if rel_cap is None or peak_cap is None or not (
            np.isfinite(rel_cap) and np.isfinite(peak_cap)
        ):
            raise ValueError(
                "accel_release_in_range_only が true ですが、表の meta に"
                " accel_release_max_pct / accel_release_peak_max_pct がありません"
                "（`python -m tests.research.fwd_final add-release-peak` で付けてください）"
            )
        rel_cap, peak_cap = float(rel_cap), float(peak_cap)
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
        if do_rel and peak_cap is not None and mx > peak_cap:
            do_rel = False  # 段9-c: 直近の最大 u が学習の範囲の外は戻しの層なし
        do_push = push_on and i > 0 and u0 > mn + 1e-9
        if not (do_rel or do_push):
            continue  # 戻していない・踏み増していない
        top = max(mx if do_rel else 0.0, u0 if do_push else 0.0)
        if top <= 0.0:
            continue
        kinks = [0.0, top]
        use_rel = rel_on and not (peak_cap is not None and mx > peak_cap)
        if use_rel and 0.0 < mx < top:
            kinks.append(mx)  # 戻し量 max(mx − u, 0) の折れ点
        if push_on and 0.0 < mn < top:
            kinks.append(mn)  # 踏み増し量 max(u − mn, 0) の折れ点
        pts = np.unique(np.concatenate([kinks, grid[(grid > 0.0) & (grid < top)]]))
        f = (
            np.asarray(
                table.accel(
                    float(v[i]),
                    pts,
                    accel_release=(
                        _release_amount_in_range(mx - pts, rel_cap) if use_rel else None
                    ),
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


def _release_amount_in_range(amount: np.ndarray, cap: float | None) -> np.ndarray:
    """戻し量（負は 0 に）。cap があるとき、cap を超える点は 0（本体）にする（段9-c）。"""
    r = np.maximum(amount, 0.0)
    return r if cap is None else np.where(r > cap, 0.0, r)


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
