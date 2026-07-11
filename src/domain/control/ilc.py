"""反復学習制御（ILC: Iterative Learning Control）のドメインモジュール。

シャシダイナモの同一モード反復走行は再現性が高い。走行ごとに時刻別の追従残差
e_j(t)=ref−actual を蓄積し、次回走行に適用する時刻別補正 effort テーブルを学習すれば、
FF+PID では消せない「反復再現性のある誤差」（停止移行の系統偏差など）をほぼ 0 にできる。

構成（純ドメイン・I/O なし）:
  - ILCTable   : 時刻グリッド（dt_s）上の補正 effort 列と反復回数・最良 p95 のメタデータ
  - ILCController: 走行中に elapsed_s から補正 effort を線形補間して返す（±amp クランプ）
  - ILCLearner : 走行後に残差から次回テーブルを学習する
      u_{j+1}(t) = clip( Q( u_j(t) + L·e_j(t+Δ) ), ±amp )
      Q はゼロ位相ローパス（forward-backward 1次 IIR）。L=学習ゲイン、Δ=むだ時間シフト。

effort の符号は FF/PID と同じ（+: 加速 [%]、−: 制動 [%]）。amp クランプで暴走を機構的に
制限し、発散検知（直近 p95 > 最良 p95×係数）で学習をスキップする安全策を持つ。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ── ILC 既定パラメータ ─────────────────────────────────────────────────
ILC_DT_S: float = 0.1  # 補正グリッド周期 [s]（drive_logs の記録周期に一致）
ILC_AMP_LIMIT_PCT: float = 10.0  # 補正 effort の振幅上限 [%]（暴走の機構的制限）
ILC_CUTOFF_HZ: float = 0.3  # Q（ゼロ位相ローパス）のカットオフ [Hz]（PID 帯域より低く役割分離）
# 学習ゲイン L = ILC_L_GAIN_FACTOR / fopdt_k（プラント定常ゲインで正規化）。
ILC_L_GAIN_FACTOR: float = 0.4
# 発散検知係数: 直近走行 p95 が 最良 p95 × この値を超えたら学習をスキップ（テーブルは最良を保持）。
ILC_DIVERGENCE_FACTOR: float = 1.2


@dataclass
class ILCTable:
    """時刻グリッド上の補正 effort 列と学習メタデータ。

    efforts[i] は時刻 i×dt_s における符号付き補正 effort [%]（+加速/−制動、FF/PID と同単位）。
    iteration は学習反復回数、best_p95_kmh はこれまでの最良走行 p95（発散検知の基準）。
    """

    efforts: list[float] = field(default_factory=list)
    dt_s: float = ILC_DT_S
    iteration: int = 0
    best_p95_kmh: float | None = None

    @property
    def duration_s(self) -> float:
        """テーブルが覆う時間長 [s]。"""
        return max(0, len(self.efforts) - 1) * self.dt_s


class ILCController:
    """走行中に時刻別補正 effort を供給するコントローラ（線形補間＋振幅クランプ）。"""

    def __init__(self, table: ILCTable, amp_limit_pct: float = ILC_AMP_LIMIT_PCT) -> None:
        self._table = table
        self._amp = max(0.0, amp_limit_pct)
        # 補間の高速化のため grid 時刻と値を前計算する。
        self._times = np.arange(len(table.efforts), dtype=float) * table.dt_s
        self._efforts = np.asarray(table.efforts, dtype=float)

    @property
    def table(self) -> ILCTable:
        return self._table

    def effort_at(self, elapsed_s: float) -> float:
        """elapsed_s における補正 effort [%]。範囲外は端点クランプ、±amp でクランプ。

        Δシフトは学習時に焼き込み済みのため、ここは now-frame で参照する。テーブルが空
        （初回走行）なら 0.0 を返す。
        """
        if self._efforts.size == 0:
            return 0.0
        value = float(np.interp(elapsed_s, self._times, self._efforts))
        return max(-self._amp, min(self._amp, value))


class ILCLearner:
    """走行後の残差から次回の補正テーブルを学習する（純関数的）。"""

    def update(
        self,
        table: ILCTable,
        times: list[float],
        errors: list[float],
        *,
        l_gain: float,
        delta_s: float,
        amp_limit_pct: float = ILC_AMP_LIMIT_PCT,
        cutoff_hz: float = ILC_CUTOFF_HZ,
        dt_s: float | None = None,
        new_p95_kmh: float | None = None,
    ) -> ILCTable:
        """u_{j+1}(t) = clip( Q( u_j(t) + L·e_j(t+Δ) ), ±amp ) を計算して新テーブルを返す。

        Args:
            table: 現在の補正テーブル（初回は空 efforts）。
            times: 残差サンプルの時刻 [s]（走行開始からの経過秒、単調増加想定）。
            errors: 各 times での追従残差 e=ref−actual [km/h]。
            l_gain: 学習ゲイン L（>0。誤差が正=遅い ほど正の effort を足す）。
            delta_s: むだ時間シフト Δ [s]（e_j(t+Δ) を参照して因果を揃える）。
            amp_limit_pct: 補正振幅の上限 [%]。
            cutoff_hz: ゼロ位相ローパスのカットオフ [Hz]。
            dt_s: 出力グリッド周期 [s]（省略時は table.dt_s）。
            new_p95_kmh: 今回走行の p95。テーブルの best_p95 更新に使う（None なら据え置き）。

        Returns:
            iteration+1・新 efforts の ILCTable。times が空なら table をそのまま返す。
        """
        if not times or not errors:
            return table
        grid_dt = dt_s if dt_s is not None else table.dt_s
        t_arr = np.asarray(times, dtype=float)
        e_arr = np.asarray(errors, dtype=float)

        # 出力グリッド長: 既存テーブルがあればその長さを維持、無ければ走行時間から決める。
        if table.efforts:
            n = len(table.efforts)
            prev = np.asarray(table.efforts, dtype=float)
        else:
            n = int(round(float(t_arr[-1]) / grid_dt)) + 1
            prev = np.zeros(n, dtype=float)
        grid = np.arange(n, dtype=float) * grid_dt

        # e_j(t+Δ) を各グリッド点で補間（範囲外は端点クランプ）。
        e_shifted = np.interp(grid + delta_s, t_arr, e_arr)
        raw = prev + l_gain * e_shifted
        filtered = zero_phase_lowpass(raw, cutoff_hz, grid_dt)
        amp = max(0.0, amp_limit_pct)
        clipped = np.clip(filtered, -amp, amp)

        best = table.best_p95_kmh
        if new_p95_kmh is not None and (best is None or new_p95_kmh < best):
            best = new_p95_kmh
        return ILCTable(
            efforts=[float(v) for v in clipped],
            dt_s=grid_dt,
            iteration=table.iteration + 1,
            best_p95_kmh=best,
        )


def is_diverged(
    best_p95_kmh: float | None,
    new_p95_kmh: float | None,
    factor: float = ILC_DIVERGENCE_FACTOR,
) -> bool:
    """今回走行 p95 が最良 p95×factor を超えたら発散とみなす（学習スキップの判定）。

    最良がまだ無い（初回）か p95 が取れない場合は発散扱いしない（学習を進める）。
    """
    if best_p95_kmh is None or new_p95_kmh is None:
        return False
    return new_p95_kmh > best_p95_kmh * factor


def l_gain_from_fopdt(fopdt_k: float | None, factor: float = ILC_L_GAIN_FACTOR) -> float:
    """学習ゲイン L = factor / fopdt_k。fopdt_k 未同定なら factor をそのまま返す（保守側）。"""
    if fopdt_k is None or fopdt_k <= 0.0:
        return factor
    return factor / fopdt_k


def zero_phase_lowpass(x: np.ndarray, cutoff_hz: float, dt_s: float) -> np.ndarray:
    """1次 IIR の forward-backward 適用でゼロ位相ローパスする（位相遅れ 0）。

    cutoff_hz<=0 や短すぎる系列はそのまま返す。RC=1/(2π·fc)、α=dt/(RC+dt) の指数平滑を
    前方・後方に適用して群遅延を打ち消す。ILC の残差平滑とペダルプランの名目 effort 平滑
    （pedal_plan.py）で共用する。
    """
    if cutoff_hz <= 0.0 or dt_s <= 0.0 or x.size < 2:
        return np.asarray(x, dtype=float)
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = dt_s / (rc + dt_s)
    forward = _ewma(x, alpha)
    backward = _ewma(forward[::-1], alpha)[::-1]
    return backward


def _ewma(x: np.ndarray, alpha: float) -> np.ndarray:
    """指数加重移動平均（1次 IIR ローパス）。y[i] = y[i-1] + α(x[i]−y[i-1])。"""
    y = np.empty_like(x, dtype=float)
    acc = float(x[0])
    for i in range(x.size):
        acc += alpha * (float(x[i]) - acc)
        y[i] = acc
    return y


__all__ = [
    "ILC_AMP_LIMIT_PCT",
    "ILC_CUTOFF_HZ",
    "ILC_DIVERGENCE_FACTOR",
    "ILC_DT_S",
    "ILC_L_GAIN_FACTOR",
    "ILCController",
    "ILCLearner",
    "ILCTable",
    "is_diverged",
    "l_gain_from_fopdt",
    "zero_phase_lowpass",
]
