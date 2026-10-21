"""C11（DOB = 外乱オブザーバ）の推定器（2026-10-18 段2q）。

表（順モデル）が予測する加速度と、実車速から測った加速度の差を、ペダル別に一次遅れでならして
持つ。差 d ＝ 実測 − 予測 [km/h/s]（正＝表より加速している／減速が浅い、負＝表より遅い）。
走行ループ（mode_drive）が毎周期 `update` を呼び、開度の列を引き直すときに `fwd_plan.
deviation_offset_detail(dob_kmhs=d)` で「要る加速度から d を引いて」表を引く。

- 推定に使う開度は、実際に出した開度から min を引いた u（アクセル正・ブレーキ負・min 以下は 0）
- u は遅れ L だけ過去のもの（アクセルの u はアクセルの遅れ、ブレーキの u はブレーキの遅れ。
  `fwd_plan.lagged_input` と同じ向きと優先順）を使う
- ブレーキの予測は、計画（`fwd_plan._resolve_releases`）と同じく戻しの表を引く。戻し量 ＝ 遅れた u −
  その時刻までの直近 release_window_s 秒の u の最小（`fwd_data.release_amount` と同じ定義）。
  2026-10-18 段2q ①' `124859` の修正: 本体の表だけで予測すると、戻している低速のブレーキで
  「表ほど減速しない」（+0.2〜+0.4 km/h/s）と読み、踏み増して遅くなった
- アクセルの予測も、表がアクセルの戻しの表を持つなら同様に引く（2026-10-20 段4-3）。戻し量 ＝
  その時刻までの直近 release_window_s 秒の u の最大 − 遅れた u（`fwd_data.accel_release_amount`
  と同じ定義）。表が持たなければ今までと同じ
- 更新しない（値を保つ）条件: 遅れた u が 0（惰行・停車保持）／ガバナー作動／遅れた u が最大開度／
  発進側・停止側の表の行／そのペダルの Tq が None
- numpy 以外は import しない（fwd_plan を読むと mode_drive との循環になる。発進・停止の判定は
  呼び出し側が出して渡す。出どころは、計画の規則が legacy なら `low_speed_sides`、pedal なら計画が
  行ごとに決めた表（`ForwardPlan.side_accel` / `side_brake`。2026-10-22 段5-2））
"""

from __future__ import annotations

from collections import deque
from typing import Any

PEDAL_ACCEL = "accel"
PEDAL_BRAKE = "brake"


class PedalDisturbanceObserver:
    """ペダル別の外乱推定器（d_accel・d_brake）。表は `accel(v, u, launch=, stop=)` を持つもの。"""

    def __init__(
        self,
        table: Any,
        *,
        tq_accel_s: float | None,
        tq_brake_s: float | None,
        lag_accel_s: float | None,
        lag_brake_s: float | None,
        release_window_s: float = 0.0,
    ) -> None:
        self.table = table
        self.tq_accel_s = tq_accel_s
        self.tq_brake_s = tq_brake_s
        # 遅れが測れていないペダルは更新しない（Tq も None のはずだが、ここでも守る）
        self.lag_accel_s = lag_accel_s
        self.lag_brake_s = lag_brake_s
        self.release_window_s = release_window_s  # 0 ＝ 戻しの表を引かない
        self.d_accel = 0.0
        self.d_brake = 0.0
        self.n_accel = 0  # 推定を更新した周期数
        self.n_brake = 0
        # (時刻, u, 最大開度か, ガバナーか) の履歴
        self._hist: deque[tuple[float, float, bool, bool]] = deque()
        self._prev_t: float | None = None
        self._prev_speed: float | None = None

    def estimate(self, *, is_accel: bool) -> float:
        return self.d_accel if is_accel else self.d_brake

    def _delayed(self, tau: float) -> tuple[float, bool, bool] | None:
        """時刻 τ の加速度に効く (u, 最大開度か, ガバナーか)。履歴が足りなければ None。
        u(τ − la) > 0 ならアクセル、そうでなく u(τ − lb) < 0 ならブレーキ、
        どちらでもなければ u ＝ 0。"""
        la, lb = self.lag_accel_s, self.lag_brake_s
        if la is not None:
            e = self._at(tau - la)
            if e is None:
                return None
            if e[0] > 0.0:
                return e
        if lb is not None:
            e = self._at(tau - lb)
            if e is None:
                return None
            if e[0] < 0.0:
                return e
        return 0.0, False, False

    def _release(self, tau: float, u_d: float) -> float:
        """ブレーキの遅れた u（< 0）の戻し量 ＝ u_d − 時刻 τ − lb までの直近 release_window_s 秒の
        u の最小（その時刻を含む）。アクセル・窓 0 は 0。"""
        if u_d >= 0.0 or self.release_window_s <= 0.0 or self.lag_brake_s is None:
            return 0.0
        t1 = tau - self.lag_brake_s
        t0 = t1 - self.release_window_s
        m = min((u for ht, u, _, _ in self._hist if t0 - 1e-9 <= ht <= t1 + 1e-9), default=u_d)
        return max(u_d - m, 0.0)

    def _accel_release(self, tau: float, u_d: float) -> float:
        """アクセルの遅れた u（> 0）の戻し量 ＝ 時刻 τ − la までの直近 release_window_s 秒の u の
        最大（その時刻を含む）− u_d（2026-10-20 段4-3）。ブレーキ・窓 0 は 0。"""
        if u_d <= 0.0 or self.release_window_s <= 0.0 or self.lag_accel_s is None:
            return 0.0
        t1 = tau - self.lag_accel_s
        t0 = t1 - self.release_window_s
        m = max((u for ht, u, _, _ in self._hist if t0 - 1e-9 <= ht <= t1 + 1e-9), default=u_d)
        return max(m - u_d, 0.0)

    def accel_release_at(self, tau: float, u_d: float) -> float:
        """時刻 τ のアクセルの戻し量（`_accel_release` の公開名。段4-3）。"""
        return self._accel_release(tau, u_d)

    def delayed_u(self, tau: float) -> tuple[float, bool, bool] | None:
        """時刻 τ の加速度に効く (u, 最大開度か, ガバナーか)（`_delayed` の公開名。2026-10-19 段2u:
        C13 の車速のフィルタ・予測器が同じ引き方で表を引く）。履歴が足りなければ None。"""
        return self._delayed(tau)

    def release_at(self, tau: float, u_d: float) -> float:
        """時刻 τ のブレーキの戻し量（`_release` の公開名。段2u）。"""
        return self._release(tau, u_d)

    def _at(self, t: float) -> tuple[float, bool, bool] | None:
        """時刻 t 以前で最新の履歴の (u, 最大開度か, ガバナーか)。t が履歴より前なら None。"""
        found = None
        for ht, u, lim, gov in self._hist:
            if ht <= t + 1e-9:
                found = (u, lim, gov)
            else:
                break
        return found

    def update(
        self,
        *,
        t: float,
        speed_kmh: float,
        u_now: float,
        governed: bool,
        at_limit_now: bool,
        launch: bool,
        stop: bool,
    ) -> str | None:
        """1 周期ぶん進める。推定を更新したペダル（PEDAL_ACCEL / PEDAL_BRAKE）を返す。
        更新しなければ None。

        u_now ＝ 今の周期で出した開度 − min（アクセル正・ブレーキ負・min 以下は 0）。
        launch・stop ＝ 今の計画の行が発進側・停止側の表を引くか（`fwd_plan.low_speed_sides`、
        規則が pedal の計画は `ForwardPlan.side_for`。段5-2）。
        """
        self._hist.append((t, u_now, at_limit_now, governed))
        prev_t, prev_v = self._prev_t, self._prev_speed
        self._prev_t, self._prev_speed = t, speed_kmh
        # 履歴は最大の遅れ + 余裕より古いものを捨てる（遅れ以前の 1 点は残す）
        keep = (
            max(
                (self.lag_accel_s or 0.0) + self.release_window_s,
                (self.lag_brake_s or 0.0) + self.release_window_s,
            )
            + 0.5
        )
        while len(self._hist) > 2 and self._hist[1][0] <= t - keep:
            self._hist.popleft()
        if prev_t is None or prev_v is None:
            return None
        dt = t - prev_t
        if dt <= 0.0:
            return None
        delayed = self._delayed(t)
        if delayed is None:
            return None
        u_d, limited, gov_d = delayed
        if u_d == 0.0 or governed or gov_d or limited or launch or stop:
            return None
        is_accel = u_d > 0.0
        tq = self.tq_accel_s if is_accel else self.tq_brake_s
        if tq is None or tq <= 0.0:
            return None
        meas = (speed_kmh - prev_v) / dt
        release = self._release(t, u_d)
        a_release = self._accel_release(t, u_d)
        if a_release > 0.0:
            pred = float(
                self.table.accel(
                    speed_kmh, u_d, launch=False, stop=False, accel_release=a_release
                )
            )
        elif release > 0.0:
            pred = float(
                self.table.accel(speed_kmh, u_d, launch=False, stop=False, release=release)
            )
        else:
            pred = float(self.table.accel(speed_kmh, u_d, launch=False, stop=False))
        e = meas - pred
        alpha = min(1.0, dt / tq)
        if is_accel:
            self.d_accel += alpha * (e - self.d_accel)
            self.n_accel += 1
            return PEDAL_ACCEL
        self.d_brake += alpha * (e - self.d_brake)
        self.n_brake += 1
        return PEDAL_BRAKE
