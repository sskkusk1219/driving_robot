"""上限 G に届く開度の予測マップ（ProblemReport_20260925 段6a・門①）。

手順2 の先頭の G 校正（0.2G 狙いのブレーキ減速・G 比例加速）と、走行中に測れた
「開度と実測加速度」の組から、車速帯ごとに「そのペダルが上限 G に届くと予測される開度」を返す。
格子ステップ・発進停車セルの開度の上限として使う（超えてから頭打ちにする G ガバナーだけでは、
踏んだ直後に一度は超えてしまう。20260926 の実機で 0.47G）。

- 車速帯 `bin_kmh` ごとに点を持つ。ペダルの向き（アクセル=加速、ブレーキ=減速）の加速度 [km/h/s]。
- 惰行の減速（両ペダル 0%）も車速帯ごとに持ち、不感帯の開度での点として使う
  （ブレーキ: 開度=不感帯で減速=惰行、アクセル: 開度=不感帯で加速=−惰行）。
- 予測は帯内の点から: 上限が点の範囲内なら前後の点の内挿、範囲の外なら割線（一番効いた点と
  開度が一番低い点）で外挿する。ブレーキは踏むほど急に効く（25 km/h の実測で 12→16% が 0.21、
  16→24% が 0.34 km/h/s/%）ので、割線は実際の傾きより緩く、予測開度は大きめ（危険側）になる。
  上限に `learning.g_cap_g`（本当の上限より低い仮の値）を使うのはそのため。
- 点が無い車速帯は、そのペダルが効きやすい側（ブレーキは速い側、アクセルは遅い側）の帯を使う
  （効きやすい側の予測開度は小さくなるので安全側）。

2026-09-27 段7a（ProblemReport_20260925 段7）: `coast_decel_kmhs` で車速帯ごとの惰行の減速の
中央値を外向けにも返す。格子ステップ走行が「その車速の惰行ステップ」の代わりに、コーストダウン
で既に測れているこの値を使う（`grid_planner.GridPlanner` の `coast_fn`）ため。
"""

from __future__ import annotations

import math
from collections import deque

PEDAL_ACCEL = "accel"
PEDAL_BRAKE = "brake"
_MAX_POINTS_PER_BIN = 40
_MIN_OPENING_GAP_PCT = 1.0  # 割線の 2 点の開度がこれ未満なら傾きを作らない（雑音で不安定）
_MIN_ACCEL_GAP_KMHS = 0.05


def _extrapolate(points: list[tuple[float, float]], target: float) -> float | None:
    """(開度, 加速度) の点から、加速度が target になる開度。作れなければ None。"""
    if len(points) < 2:
        return None
    below = [p for p in points if p[1] < target]
    above = [p for p in points if p[1] >= target]
    if above and below:  # 内挿: target の直前・直後の点
        lo = max(below, key=lambda p: p[1])
        hi = min(above, key=lambda p: p[1])
        if hi[1] - lo[1] < _MIN_ACCEL_GAP_KMHS:
            return min(lo[0], hi[0])
        frac = (target - lo[1]) / (hi[1] - lo[1])
        return lo[0] + frac * (hi[0] - lo[0])
    if above:  # すべて target 以上: 一番弱い点の開度で既に届く
        return min(above, key=lambda p: p[1])[0]
    # 外挿: 加速度が一番大きい点と、開度が一番低い点の割線。近い 2 点の割線は加速度の雑音で
    # 傾きが不安定になる（1 帯に数点しか無く、開度の差は 0.5% 刻み）。遠い点（惰行の基準点＝不感帯）
    # との割線は安定。ブレーキは踏むほど急に効くので割線は実際の傾きより緩く、開度を大きく見積もる
    # 側になる（`learning.g_cap_g` を上限より低くとる理由）
    (u1, a1) = max(below, key=lambda p: p[1])
    (u0, a0) = min(below, key=lambda p: p[0])
    if u1 - u0 < _MIN_OPENING_GAP_PCT or a1 - a0 < _MIN_ACCEL_GAP_KMHS:
        return None
    return u1 + (target - a1) * (u1 - u0) / (a1 - a0)


def _fmt_cap(cap_pct: float | None) -> str:
    """上限開度表の 1 セルの表示文字列。100% を超える予測は「届かない」を明示する。"""
    if cap_pct is None:
        return "—"
    if cap_pct > 100.0:
        return "100%(届かない)"
    return f"{cap_pct:.1f}"


class GLimitMap:
    """車速帯ごとの (開度, 加速度) の点から、上限 G に届く開度を予測する。"""

    def __init__(self, bin_kmh: float = 10.0) -> None:
        self._bin = bin_kmh
        self._points: dict[tuple[str, int], deque[tuple[float, float]]] = {}
        self._coast: dict[int, list[float]] = {}  # 車速帯 → 惰行の減速 [km/h/s]（正）

    def _idx(self, speed_kmh: float) -> int:
        return math.floor(max(speed_kmh, 0.0) / self._bin)

    def add(self, pedal: str, speed_kmh: float, opening_pct: float, accel_kmhs: float) -> None:
        """実測の 1 点。`accel_kmhs` はそのペダルの向きの加速度（ブレーキなら減速を正で）。"""
        if opening_pct <= 0.0 or math.isnan(accel_kmhs):
            return
        key = (pedal, self._idx(speed_kmh))
        self._points.setdefault(key, deque(maxlen=_MAX_POINTS_PER_BIN)).append(
            (opening_pct, accel_kmhs)
        )

    def add_coast(self, speed_kmh: float, decel_kmhs: float) -> None:
        """惰行（両ペダル 0%）の減速 [km/h/s]（正）。"""
        if math.isnan(decel_kmhs):
            return
        self._coast.setdefault(self._idx(speed_kmh), []).append(decel_kmhs)

    def has_data(self, pedal: str) -> bool:
        return any(k[0] == pedal for k in self._points)

    def strongest_point(self, pedal: str, speed_kmh: float) -> tuple[float, float] | None:
        """`speed_kmh` の車速帯で一番効いた実測点 (開度, 加速度)。点が無ければ None。

        2026-09-28 段7c（ProblemReport_20260925 段7）: 格子ステップの最初の感度（未測定のペダル
        の最初の開度）を、仮定値ではなくその車速で実際に効いた実測から決めるのに使う
        （`grid_planner.GridPlanner` の `gain_fn`）。惰行の基準点は含めない（呼び出し側が
        `coast_decel_kmhs` の基準点と組み合わせて割線を作るため）。
        """
        pts = self._points.get((pedal, self._idx(speed_kmh)))
        if not pts:
            return None
        return max(pts, key=lambda p: p[1])

    def _coast_median(self, idx: int) -> float | None:
        coast = self._coast.get(idx)
        if not coast:
            return None
        return sorted(coast)[len(coast) // 2]

    def coast_decel_kmhs(self, speed_kmh: float) -> float | None:
        """その車速帯の惰行（両ペダル 0%）の減速の中央値 [km/h/s]（正）。点が無ければ None。

        段7a（ProblemReport_20260925）: 格子ステップの惰行ステップの代わりに、コーストダウンで
        既に測れているこの値を使う（`grid_planner.GridPlanner` の `coast_fn`）。
        """
        return self._coast_median(self._idx(speed_kmh))

    def _bin_cap(
        self, pedal: str, idx: int, target_kmhs: float, deadband_pct: float
    ) -> float | None:
        pts = list(self._points.get((pedal, idx), ()))
        c = self._coast_median(idx)
        if c is not None:
            pts.append((deadband_pct, c if pedal == PEDAL_BRAKE else -c))
        u = _extrapolate(pts, target_kmhs)
        if u is None:
            return None
        # 実測で「上限に届かなかった」開度は安全と分かっている。予測がそれより低いのは雑音
        safe = max((o for o, a in pts if a <= target_kmhs), default=deadband_pct)
        return max(u, safe, deadband_pct)

    def cap_pct(
        self, pedal: str, speed_kmh: float, g_cap_kmhs: float, deadband_pct: float
    ) -> float | None:
        """`speed_kmh` の車速帯で、そのペダルが `g_cap_kmhs`（正）に届くと予測される開度 [%]。

        測れていなければ None（上限をかけない）。自分の帯で作れないときは、そのペダルが
        効きやすい側の帯（ブレーキは速い側・アクセルは遅い側）を近い順に探し、無ければ反対側。
        """
        own = self._idx(speed_kmh)
        idxs = sorted({k[1] for k in self._points if k[0] == pedal} | set(self._coast))
        if not idxs:
            return None
        safe_side = [i for i in idxs if (i >= own if pedal == PEDAL_BRAKE else i <= own)]
        other = [i for i in idxs if i not in safe_side]
        order = sorted(safe_side, key=lambda i: abs(i - own)) + sorted(
            other, key=lambda i: abs(i - own)
        )
        for i in order:
            cap = self._bin_cap(pedal, i, g_cap_kmhs, deadband_pct)
            if cap is not None:
                return cap
        return None

    def table(
        self, g_cap_kmhs: float, brake_deadband_pct: float, accel_deadband_pct: float,
        max_speed_kmh: float,
    ) -> list[str]:
        """車速帯ごとの上限開度の表（ターミナル表示用）。

        2026-09-28 段7d: 予測開度が 100% を超える帯（＝機構の上限開度まで踏んでも上限 G に
        届かないと予測される帯）は数値ではなく「100%(届かない)」と表示する。頭打ちとしての
        動き（`cap_pct` の戻り値）はそのまま、100% 超えでも変えない。
        """
        lines = ["車速帯[km/h]  ブレーキ上限[%]  アクセル上限[%]"]
        top = int(max_speed_kmh // self._bin) + 1
        for i in range(top):
            v = (i + 0.5) * self._bin
            b = self.cap_pct(PEDAL_BRAKE, v, g_cap_kmhs, brake_deadband_pct)
            a = self.cap_pct(PEDAL_ACCEL, v, g_cap_kmhs, accel_deadband_pct)
            if b is None and a is None:
                continue
            lines.append(
                f"{i * self._bin:4.0f}〜{(i + 1) * self._bin:4.0f}   "
                f"{_fmt_cap(b):>12}     {_fmt_cap(a):>12}"
            )
        return lines


__all__ = ["PEDAL_ACCEL", "PEDAL_BRAKE", "GLimitMap"]
