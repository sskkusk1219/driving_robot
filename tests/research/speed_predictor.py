"""C13 の車速のフィルタ（簡単なカルマンフィルタ）と遅れの予測器（2026-10-19 段2u）。

目的: ペダルを踏んでから車速に効くまでには遅れ L（手順2 で測った値）がある。C10 の P の上乗せは
「今の偏差」を見るので、遅れのぶんだけ後追いになる。ここでは L 秒先の車速を予測し、
偏差を「基準の L 秒先 − 予測した L 秒先の車速」で取れるようにする。

## 車速のフィルタ（状態は車速 1 つ）
- 予測: x⁻ ＝ 前の推定 ＋ a × dt。a ＝ 表（順モデル）が遅れ L 前の実指令から予測する加速度
  （DOB の `_delayed` と同じ引き方・同じ停止側／発進側／戻しの表の選び方）＋ DOB の推定外乱 d
  （押しているペダルの分。低速の行では 0）。P⁻ ＝ P ＋ Q。
- 観測: CAN 車速 y。K ＝ P⁻ ÷ (P⁻ ＋ R)、x ＝ x⁻ ＋ K (y − x⁻)、P ＝ (1 − K) P⁻。
- DOB の「実加速度」には、このフィルタの推定車速の差分 (x − 前の x) ÷ dt を使う（C11 は CAN 車速の
  後退差分）。注意: 予測に d を入れているので、DOB の読む加速度は d ＋ K × 外れ ÷ dt になり、
  d の追いつきは C11 の約 1/K 倍の時間になる（K は R と Q の比で決まる）。

## Q と R の決め方（決まり G3: フィルタの強さは測った車速の乱れから）
- R（観測雑音の分散 [(km/h)²]）: 走行の最初の停車保持のあいだの CAN 車速の乱れ。停車中は
  ゆっくりした変化が無いので、2 階差分 v[k] − 2v[k−1] + v[k−2]（白色雑音なら分散 6σ²）の
  2 乗平均 ÷ 6 で測る（10 点以上）。測れない・測った値が下限未満なら下限
  （`speed_filter_r_floor`。分解能が粗く停車中が一定値に張り付く車では 0 になるため必要）。
  yaml の `speed_filter_r` が数値ならそれで上書き。
- Q（プロセス雑音）: 表の予測加速度の外れの分散 σ_a² [(km/h/s)²] から Q ＝ σ_a² × dt²。
  σ_a は pkl の meta の `seed_mae`（学習のときの加速度の MAE。ペダル別ではなく平均）に正規分布の
  MAE→σ の換算 1.2533 を掛けた値。meta に無ければ yaml の `speed_filter_accel_sigma`。
  yaml の `speed_filter_q` が数値なら σ_a² として上書き。外れは時間的に相関する（白色ではない）
  ので、Q は積み上がる速さを小さく見積もる。その分、フィルタは観測を重く見る側に寄る（安全側）。

## 予測器
v̂(t + L) ＝ x(t) ＋ Σ a(v, u(τ − L)) × h ＋ d × L。τ は t から t + L まで（h は周期、最後は端数）。
遅れた u は履歴（DOB と同じもの）から引く。履歴が足りなければ加速度 0（予測なし）。
L ＝ 今踏んでいるペダルの遅れ（アクセル正・ブレーキ負・惰行は coast_lag_s）。

numpy は使わない（DOB と同じ。テーブルの `accel` だけ呼ぶ）。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any

#: MAE から正規分布の標準偏差への換算（σ ＝ MAE × √(π/2)）
MAE_TO_SIGMA = math.sqrt(math.pi / 2.0)
#: R を測るのに必要な最少の点数
R_MIN_SAMPLES = 10
#: R を測る点数の上限（停車保持が長いとき。20 秒ぶん）
R_MAX_SAMPLES = 400


class SpeedKalman:
    """状態が車速 1 つのカルマンフィルタ。Q はステップごとに σ_a² × dt²。"""

    def __init__(self, *, accel_var: float, r_var: float) -> None:
        self.accel_var = accel_var  # σ_a²（表の予測加速度の外れの分散）[(km/h/s)²]
        self.r_var = r_var  # 観測雑音の分散 [(km/h)²]
        self.x: float | None = None
        self.p = r_var
        self.gain = 1.0  # 直近の更新の K

    def predict(self, accel_kmhs: float, dt: float) -> None:
        if self.x is None:
            return
        self.x = max(self.x + accel_kmhs * dt, 0.0)
        self.p += self.accel_var * dt * dt

    def update(self, y: float) -> float:
        if self.x is None:
            self.x, self.p, self.gain = y, self.r_var, 1.0
            return self.x
        k = self.p / (self.p + self.r_var)
        self.x += k * (y - self.x)
        self.p = (1.0 - k) * self.p
        self.gain = k
        return self.x


def noise_var_from_second_difference(samples: list[float]) -> float | None:
    """停車中などの CAN 車速の列から観測雑音の分散を測る（2 階差分の 2 乗平均 ÷ 6）。
    R_MIN_SAMPLES 点未満は None。"""
    n = len(samples)
    if n < R_MIN_SAMPLES:
        return None
    d2 = [samples[k] - 2.0 * samples[k - 1] + samples[k - 2] for k in range(2, n)]
    return sum(d * d for d in d2) / len(d2) / 6.0


def accel_error_var_from_meta(meta: dict[str, Any], fallback_sigma: float) -> tuple[float, str]:
    """表の予測加速度の外れの分散 σ_a² と、その出どころの説明。pkl の meta の seed_mae
    （辞書か列。学習のときの加速度の MAE の平均）から。無い・使えない値なら fallback_sigma。"""
    mae = meta.get("seed_mae")
    vals: list[float] = []
    if isinstance(mae, dict):
        vals = [float(v) for v in mae.values()]
    elif isinstance(mae, (list, tuple)):
        vals = [float(v) for v in mae]
    elif isinstance(mae, (int, float)):
        vals = [float(mae)]
    vals = [v for v in vals if math.isfinite(v) and v > 0.0]
    if vals:
        sigma = MAE_TO_SIGMA * (sum(vals) / len(vals))
        mean_mae = sum(vals) / len(vals)
        return sigma * sigma, f"pkl の seed_mae 平均 {mean_mae:.3f} から σ_a {sigma:.3f}"
    return fallback_sigma * fallback_sigma, f"yaml の仮値 σ_a {fallback_sigma:g}"


@dataclass
class PredictorStats:
    """C13 の集計（2026-10-19 段2u）。推定と CAN の差、予測の当たり具合。

    予測の当たり具合 ＝ 時刻 t に出した L 秒先の予測 v̂ と、t + L の CAN 車速（CAN の 2 点を
    補間）の差の 2 乗平均の平方根。「予測なし」は、t の CAN 車速をそのまま L 秒先とみなした差。
    予測を使った周期（低速の行でない周期）だけ数える。
    """

    est_sq_sum: float = 0.0
    est_n: int = 0
    pred_sq_sum: float = 0.0
    naive_sq_sum: float = 0.0
    pred_n: int = 0
    pred_used_cycles: int = 0  # 偏差に予測を使った周期数
    total_cycles: int = 0
    period_s: float = 0.05
    _pending: deque = field(default_factory=deque)  # (目標時刻, 予測, 出した時刻の CAN 車速)
    _prev: tuple[float, float] | None = None  # 直前の周期の (時刻, CAN 車速)

    def add_cycle(self, *, t: float, can_kmh: float, est_kmh: float) -> None:
        """毎周期。推定と CAN の差を足し、期限の来た予測を採点する。"""
        self.total_cycles += 1
        self.est_sq_sum += (est_kmh - can_kmh) ** 2
        self.est_n += 1
        prev = self._prev
        while self._pending and self._pending[0][0] <= t + 1e-9:
            t_target, v_pred, v_then = self._pending.popleft()
            if prev is not None and prev[0] < t:
                w = min(max((t_target - prev[0]) / (t - prev[0]), 0.0), 1.0)
                v_at = prev[1] + w * (can_kmh - prev[1])
            else:
                v_at = can_kmh
            self.pred_sq_sum += (v_at - v_pred) ** 2
            self.naive_sq_sum += (v_at - v_then) ** 2
            self.pred_n += 1
        self._prev = (t, can_kmh)

    def add_prediction(self, *, t: float, lag_s: float, pred_kmh: float, can_kmh: float) -> None:
        """予測を偏差に使った周期の予測を、採点待ちに入れる。"""
        self.pred_used_cycles += 1
        self._pending.append((t + lag_s, pred_kmh, can_kmh))

    @property
    def est_rms(self) -> float:
        return math.sqrt(self.est_sq_sum / self.est_n) if self.est_n else 0.0

    @property
    def pred_rms(self) -> float:
        return math.sqrt(self.pred_sq_sum / self.pred_n) if self.pred_n else 0.0

    @property
    def naive_rms(self) -> float:
        return math.sqrt(self.naive_sq_sum / self.pred_n) if self.pred_n else 0.0

    def summary(self) -> str:
        p = self.period_s
        used = self.pred_used_cycles * p
        if self.pred_n == 0:
            hit = "予測の採点なし"
        else:
            ratio = self.pred_rms / self.naive_rms if self.naive_rms > 0.0 else float("nan")
            hit = (
                f"予測の当たり（L 秒後の実車速との差の RMS）予測 {self.pred_rms:.3f}・"
                f"予測なし（今の車速を L 秒先とみなす）{self.naive_rms:.3f} km/h"
                f"（比 {ratio:.2f}。{self.pred_n} 点）"
            )
        return (
            f"推定と CAN 車速の差の RMS {self.est_rms:.4f} km/h（{self.est_n} 周期）／{hit}／"
            f"偏差に予測を使った {used:.1f}s（全 {self.total_cycles * p:.1f}s）"
        )


class SpeedFilterPredictor:
    """C13: 車速のフィルタ ＋ L 秒先の予測。履歴・表・外乱の推定は DOB（`dob`）のものを使う。

    使い方（走行ループが毎周期）: step(...) → 推定車速 → predict_ahead(...) → 偏差。
    """

    def __init__(
        self,
        dob: Any,
        *,
        period_s: float,
        accel_var: float,
        r_override: float | None,
        r_floor: float,
        lag_accel_s: float | None,
        lag_brake_s: float | None,
        coast_lag_s: float | None,
        accel_var_note: str = "",
    ) -> None:
        self.dob = dob
        self.table = dob.table
        self.period_s = period_s
        self.lag_accel_s = lag_accel_s
        self.lag_brake_s = lag_brake_s
        self.coast_lag_s = coast_lag_s
        self.accel_var_note = accel_var_note
        self.r_floor = r_floor
        self.r_override = r_override
        self.r_source = "yaml の上書き" if r_override is not None else "下限（測定前）"
        self.r_measured: float | None = None
        r0 = r_override if r_override is not None else r_floor
        self.kf = SpeedKalman(accel_var=accel_var, r_var=r0)
        self._calibrating = r_override is None
        self._idle_samples: list[float] = []
        self._t: float | None = None
        self._x_prev: float | None = None
        self.accel_est = 0.0  # 推定車速の差分 [km/h/s]（DOB の実加速度）
        self.stats = PredictorStats(period_s=period_s)

    # ── 遅れ L ──────────────────────────────────────────────
    def lag_for(self, effort: float) -> float | None:
        """今踏んでいるペダル（effort の符号）の遅れ。惰行（0）は coast_lag_s。None ＝ 測れない。"""
        if effort > 0.0:
            return self.lag_accel_s
        if effort < 0.0:
            return self.lag_brake_s
        return self.coast_lag_s

    # ── 表の予測加速度 ─────────────────────────────────────
    def _accel(self, tau: float, v: float, *, launch: bool, stop: bool, held: bool) -> float:
        """時刻 τ の加速度の予測 [km/h/s] ＝ 表(v, 遅れた u) ＋ DOB の推定（低速の行は 0）。
        履歴が足りなければ 0。"""
        dl = self.dob.delayed_u(tau)
        if dl is None:
            return 0.0
        u_d = dl[0]
        low = launch or stop
        release = 0.0 if low else self.dob.release_at(tau, u_d)
        a_release = 0.0 if low else self.dob.accel_release_at(tau, u_d)
        # 段5-7b: 表が踏み増しの表を持つときだけ踏み増し量を数える（持たなければ今までと同じ）
        has_push = getattr(self.table, "accel_push_grid", None) is not None
        a_push = 0.0 if low or not has_push else self.dob.accel_push_at(tau, u_d)
        if a_release > 0.0 or a_push > 0.0:
            extra = {"accel_push": a_push} if a_push > 0.0 else {}
            a = float(
                self.table.accel(
                    v, u_d, launch=False, stop=False, accel_release=a_release, **extra
                )
            )
        elif release > 0.0:
            a = float(self.table.accel(v, u_d, launch=False, stop=False, release=release))
        else:
            a = float(self.table.accel(v, u_d, launch=launch, stop=stop))
        if held or low or u_d == 0.0:
            return a
        return a + (self.dob.d_accel if u_d > 0.0 else self.dob.d_brake)

    # ── フィルタ ────────────────────────────────────────────
    def step(
        self, *, t: float, can_kmh: float, launch: bool, stop: bool, held: bool, idle: bool
    ) -> float:
        """1 周期ぶん進めて推定車速を返す。idle ＝ 最初の停車保持の行か（R を測る間）。"""
        if self._calibrating:
            if idle and len(self._idle_samples) < R_MAX_SAMPLES:
                self._idle_samples.append(can_kmh)
            else:
                self._finish_r()
        dt = None if self._t is None else t - self._t
        if self.kf.x is not None and dt is not None and dt > 0.0:
            self.kf.predict(
                self._accel(t, self.kf.x, launch=launch, stop=stop, held=held), dt
            )
        x = self.kf.update(can_kmh)
        if self._x_prev is not None and dt is not None and dt > 0.0:
            self.accel_est = (x - self._x_prev) / dt
        self._x_prev, self._t = x, t
        self.stats.add_cycle(t=t, can_kmh=can_kmh, est_kmh=x)
        return x

    def _finish_r(self) -> None:
        self._calibrating = False
        var = noise_var_from_second_difference(self._idle_samples)
        self.r_measured = var
        if var is not None and var >= self.r_floor:
            self.kf.r_var, self.r_source = var, "最初の停車保持で測定"
        else:
            self.kf.r_var = self.r_floor
            self.r_source = (
                "下限（最初の停車保持で測れず）" if var is None
                else f"下限（測定値 {var:.2e} が下限未満）"
            )

    # ── 予測 ────────────────────────────────────────────────
    def predict_ahead(
        self, *, t: float, lag_s: float, launch: bool, stop: bool, held: bool
    ) -> float | None:
        """時刻 t の L 秒先の車速の予測 [km/h]。フィルタが始まっていなければ None。"""
        x = self.kf.x
        if x is None:
            return None
        v, tau, left = x, t, lag_s
        while left > 1e-9:
            h = min(self.period_s, left)
            tau += h
            v = max(v + h * self._accel(tau, v, launch=launch, stop=stop, held=held), 0.0)
            left -= h
        return v

    def settings_text(self) -> str:
        def lag(v: float | None) -> str:
            return f"{v:.2f}s" if v is not None else "なし"

        sigma = math.sqrt(self.kf.accel_var)
        return (
            f"L アクセル {lag(self.lag_accel_s)} / ブレーキ {lag(self.lag_brake_s)} / "
            f"惰行 {lag(self.coast_lag_s)}・Q: σ_a {sigma:.3f} km/h/s（{self.accel_var_note}）・"
            f"R: {self.kf.r_var:.2e} (km/h)²＝σ {math.sqrt(self.kf.r_var):.4f} km/h"
            f"（{self.r_source}）・定常の K ≈ {self.kf.gain:.2f}"
        )
