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
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from tests.research.ff_bias import PEDAL_ACCEL, PEDAL_BRAKE, PEDAL_COAST, count_pedal_switches
from tests.research.ff_model import STOP_SPEED_KMH
from tests.research.fwd_data import DEFAULT_ACCEL_LAG_S as _DATA_ACCEL_LAG_S
from tests.research.fwd_data import DEFAULT_BRAKE_LAG_S as _DATA_BRAKE_LAG_S
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

# flag: 0〜2 は fwd_map の POINT_*（実測・穴埋め・範囲外）
FLAG_MEASURED = POINT_MEASURED
FLAG_FILLED = POINT_FILLED
FLAG_OUT = POINT_OUT
FLAG_COAST = 3
FLAG_STOP = 4
FLAG_LABELS = {
    FLAG_MEASURED: "実測",
    FLAG_FILLED: "穴埋め",
    FLAG_OUT: "範囲外",
    FLAG_COAST: "惰行",
    FLAG_STOP: "停車保持",
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
class ForwardPlan:
    """開度の列。配列はすべて同じ長さ（制御周期の刻み）。"""

    t_s: np.ndarray
    effort_pct: np.ndarray  # 正＝アクセル開度、負＝ブレーキ開度
    need_accel_kmhs: np.ndarray
    flag: np.ndarray  # FLAG_*
    period_s: float = DEFAULT_CONTROL_PERIOD_S

    def at(self, t_s: float) -> float:
        """時刻 t の effort。最近傍の点を返す。

        アクセル⇔ブレーキの境で 0 を経由しないよう、補間はしない。
        """
        i = int(np.clip(round((t_s - self.t_s[0]) / self.period_s), 0, len(self.t_s) - 1))
        return float(self.effort_pct[i])

    def seconds_of(self, flag: int) -> float:
        return float(np.count_nonzero(self.flag == flag)) * self.period_s

    def out_of_range_s(self) -> float:
        return self.seconds_of(FLAG_OUT)

    def filled_s(self) -> float:
        return self.seconds_of(FLAG_FILLED)

    def pedal_seconds(self) -> dict[str, float]:
        """アクセル・ブレーキ・惰行・停車保持の秒数（停車保持はブレーキに含めない）。"""
        stop = self.flag == FLAG_STOP
        e = self.effort_pct
        return {
            "アクセル": float(np.count_nonzero((e > 0.0) & ~stop)) * self.period_s,
            "ブレーキ": float(np.count_nonzero((e < 0.0) & ~stop)) * self.period_s,
            "惰行": float(np.count_nonzero((e == 0.0) & ~stop)) * self.period_s,
            "停車保持": float(np.count_nonzero(stop)) * self.period_s,
        }

    def _phases(self, include_stop: bool) -> list[str]:
        out = []
        for e, f in zip(self.effort_pct.tolist(), self.flag.tolist(), strict=True):
            if f == FLAG_STOP and not include_stop:
                out.append(PEDAL_COAST)
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
) -> ForwardPlan:
    """基準車速から開度の列を作る（走る前に全時刻ぶんを解く）。

    accel_lag_s・brake_lag_s が None なら表の meta（表を作ったときの L）、それも無ければ既定値。
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

    want_accel = need_a - coast >= coast_band_kmhs - 1e-12
    want_brake = ~want_accel & (need_b - coast <= -coast_band_kmhs + 1e-12)
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
    effort = np.clip(effort, -EFFORT_LIMIT_PCT, EFFORT_LIMIT_PCT)
    return ForwardPlan(t_s=t, effort_pct=effort, need_accel_kmhs=need, flag=flag, period_s=period_s)


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
    acc = (effort_pct > accel_deadband_pct) & (flag != FLAG_STOP)
    brk = (effort_pct < -brake_deadband_pct) & (flag != FLAG_STOP)
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
