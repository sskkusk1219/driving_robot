"""上限 G に届く開度の予測マップ（ProblemReport_20260925 段6a・門①）。

走行中に測れた「開度と実測加速度」の組（コーストダウン・G ランプ・高い車速からの停車復帰の
G 比例ブレーキ）から、車速帯ごとに「そのペダルが上限 G に届くと予測される開度」を返す。
G ランプの最初の開度の見積もりに使う。当初（段6a）は格子ステップ・発進停車セルの開度の上限
（超えてから頭打ちにする G ガバナーだけでは、踏んだ直後に一度は超えてしまう。20260926 の実機で
0.47G）に使っていた（段2k 段E で格子ステップ走行は削除）。

- 車速帯 `bin_kmh` ごとに点を持つ。ペダルの向き（アクセル=加速、ブレーキ=減速）の加速度 [km/h/s]。
- 惰行の減速（両ペダル 0%）も車速帯ごとに持ち、不感帯の開度での点として使う
  （ブレーキ: 開度=不感帯で減速=惰行、アクセル: 開度=不感帯で加速=−惰行）。
- 予測は帯内の点から: 上限が点の範囲内なら前後の点の内挿、範囲の外なら割線（一番効いた点と
  開度が一番低い点）で外挿する。ブレーキは踏むほど急に効く（25 km/h の実測で 12→16% が 0.21、
  16→24% が 0.34 km/h/s/%）ので、割線は実際の傾きより緩く、予測開度は大きめ（危険側）になる。
  上限には、本当の上限より低い仮の値を使うのがよい（`cap_pct` の `target_kmhs` に渡す）。
- 点が無い車速帯は、そのペダルが効きやすい側（ブレーキは速い側、アクセルは遅い側）の帯を使う
  （効きやすい側の予測開度は小さくなるので安全側）。

2026-09-27 段7a（ProblemReport_20260925 段7）: `coast_decel_kmhs` で車速帯ごとの惰行の減速の
中央値を外向けにも返す（G ランプの減速の見積もり `_ramp_estimate_u` が、惰行より緩い減速かを
判定するのに使う）。

2026-10-07 段2l-2: `sensitivity_kmhs_per_pct` で「開度 1% あたり加速度がどれだけ変わるか」を
返す。G ランプの制御ゲインを、車ごとの固定値ではなく実測の感度から決めるため（今の開度の近くの
点の最小二乗の傾きと、帯の全点の傾きの大きいほう。どちらも作れなければ他の帯）。感度を小さく
見積もると開度を速く動かす側（危険側）に外れるので、大きいほうを採る（`062322` に当てた確認で
極端に小さい値が消えた）。
"""

from __future__ import annotations

import math
from collections import deque

PEDAL_ACCEL = "accel"
PEDAL_BRAKE = "brake"
_MAX_POINTS_PER_BIN = 40
_MIN_OPENING_GAP_PCT = 1.0  # 割線の 2 点の開度がこれ未満なら傾きを作らない（雑音で不安定）
_MIN_ACCEL_GAP_KMHS = 0.05
# 仮値。ブレーキは踏むほど急に効くので、今の開度に近い点だけで傾きを出す。
# 段2l-2（ProblemReport_20261001）
_SENS_WINDOW_PCT = 5.0


def _interpolate(
    below: list[tuple[float, float]], above: list[tuple[float, float]], target: float
) -> float:
    """target をはさむ点（below: 加速度が target 未満、above: 以上。どちらも空でない）の内挿。"""
    lo = max(below, key=lambda p: p[1])
    hi = min(above, key=lambda p: p[1])
    if hi[1] - lo[1] < _MIN_ACCEL_GAP_KMHS:
        return min(lo[0], hi[0])
    frac = (target - lo[1]) / (hi[1] - lo[1])
    return lo[0] + frac * (hi[0] - lo[0])


def _extrapolate(
    points: list[tuple[float, float]], target: float, *, extrapolate: bool = True
) -> float | None:
    """(開度, 加速度) の点から、加速度が target になる開度。作れなければ None。

    `extrapolate=False` のときは、target が貯めた点のどれより強い場合に割線の外挿をせず、いちばん
    効いた点（加速度が最大の点）の開度を返す（2026-10-08 段2l-6。G ランプの最初の開度の見積もり
    用）。
    補間（target が点の範囲内）と「すべて target 以上」は変わらない。
    """
    if len(points) < 2:
        return None
    below = [p for p in points if p[1] < target]
    above = [p for p in points if p[1] >= target]
    if above and below:  # 内挿: target の直前・直後の点
        return _interpolate(below, above, target)
    if above:  # すべて target 以上: 一番弱い点の開度で既に届く
        return min(above, key=lambda p: p[1])[0]
    if not extrapolate:  # 外挿しない: いちばん効いた点の開度から始める
        return max(below, key=lambda p: p[1])[0]
    # 外挿: 加速度が一番大きい点と、開度が一番低い点の割線。近い 2 点の割線は加速度の雑音で
    # 傾きが不安定になる（1 帯に数点しか無く、開度の差は 0.5% 刻み）。遠い点（惰行の基準点＝不感帯）
    # との割線は安定。ブレーキは踏むほど急に効くので割線は実際の傾きより緩く、開度を大きく見積もる
    # 側になる（上限を本当の上限より低くとる理由）
    (u1, a1) = max(below, key=lambda p: p[1])
    (u0, a0) = min(below, key=lambda p: p[0])
    if u1 - u0 < _MIN_OPENING_GAP_PCT or a1 - a0 < _MIN_ACCEL_GAP_KMHS:
        return None
    return u1 + (target - a1) * (u1 - u0) / (a1 - a0)


def _lsq_slope(
    points: list[tuple[float, float]], min_points: int, min_accel_span_kmhs: float = 0.0
) -> float | None:
    """(開度, 加速度) の最小二乗の傾き（加速度を開度で回帰）。作れない・0 以下なら None。

    2026-10-10 段2l-11: `min_accel_span_kmhs` ＞ 0 のときは、傾き × 使った点の開度の幅
    （max − min）＝ その点たちが見分けた加速度の差 がこれ未満なら None（測れていない傾き）。
    既定 0 は今までと同じ。
    """
    if len(points) < min_points:
        return None
    us = [p[0] for p in points]
    if max(us) - min(us) < _MIN_OPENING_GAP_PCT:
        return None
    n = len(points)
    mean_u = sum(us) / n
    mean_a = sum(p[1] for p in points) / n
    sxx = sum((u - mean_u) ** 2 for u in us)
    sxy = sum((u - mean_u) * (a - mean_a) for u, a in points)
    slope = sxy / sxx
    if slope <= 0.0:
        return None
    if min_accel_span_kmhs > 0.0 and slope * (max(us) - min(us)) < min_accel_span_kmhs:
        return None
    return slope


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

        その車速で実際に効いた実測から最初の開度を決めるのに使う（段7c。当初は格子ステップの
        `GridPlanner` の `gain_fn` だった）。惰行の基準点は含めない（呼び出し側が
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

        コーストダウンで測れた惰行の減速。G ランプの減速の見積もり（`_ramp_estimate_u`）が使う。
        """
        return self._coast_median(self._idx(speed_kmh))

    def coast_curve(self) -> tuple[list[float], list[float]]:
        """惰行の減速のカーブ（車速帯の中心 [km/h] の昇順, その帯の中央値 [km/h/s]。正）。

        2026-10-16 段2o-6: 効き始めの折れ点（`play_onset`）が「惰行との差」を取るのに使う。
        点が無ければ空のリスト 2 つ。
        """
        speeds: list[float] = []
        decels: list[float] = []
        for idx in sorted(self._coast):
            med = self._coast_median(idx)
            if med is None:
                continue
            speeds.append((idx + 0.5) * self._bin)
            decels.append(med)
        return speeds, decels

    def _bin_cap(
        self, pedal: str, idx: int, target_kmhs: float, deadband_pct: float,
        *, extrapolate: bool = True,
    ) -> float | None:
        pts = list(self._points.get((pedal, idx), ()))
        c = self._coast_median(idx)
        if c is not None:
            pts.append((deadband_pct, c if pedal == PEDAL_BRAKE else -c))
        u = _extrapolate(pts, target_kmhs, extrapolate=extrapolate)
        if u is None:
            return None
        # 実測で「上限に届かなかった」開度は安全と分かっている。予測がそれより低いのは雑音
        safe = max((o for o, a in pts if a <= target_kmhs), default=deadband_pct)
        return max(u, safe, deadband_pct)

    def interp_pct(
        self, pedal: str, speed_kmh: float, target_kmhs: float, deadband_pct: float
    ) -> float | None:
        """自分の車速帯の中で、加速度が `target_kmhs`（正）になる開度を内挿で返す [%]。

        2026-10-10 段2l-10（ProblemReport_20261001。G ランプの減速の頭の寄せ用）。自分の帯の
        点と惰行の基準点（`_bin_cap` と同じ作り方）のうち、target 未満の点と target 以上の点の
        両方があるときだけ、`_extrapolate` の内挿の枝と同じ計算で返す。他の帯は探さない・外挿
        しない・
        「いちばん効いた点」を返さない・実測の安全側への底上げ（`_bin_cap` の safe）もしない。
        それ以外は None。
        """
        idx = self._idx(speed_kmh)
        pts = list(self._points.get((pedal, idx), ()))
        c = self._coast_median(idx)
        if c is not None:
            pts.append((deadband_pct, c if pedal == PEDAL_BRAKE else -c))
        below = [p for p in pts if p[1] < target_kmhs]
        above = [p for p in pts if p[1] >= target_kmhs]
        if not below or not above:
            return None
        return _interpolate(below, above, target_kmhs)

    def cap_pct(
        self, pedal: str, speed_kmh: float, g_cap_kmhs: float, deadband_pct: float,
        *, extrapolate: bool = True,
    ) -> float | None:
        """`speed_kmh` の車速帯で、そのペダルが `g_cap_kmhs`（正）に届くと予測される開度 [%]。

        測れていなければ None（上限をかけない）。自分の帯で作れないときは、そのペダルが
        効きやすい側の帯（ブレーキは速い側・アクセルは遅い側）を近い順に探し、無ければ反対側。

        2026-10-08 段2l-6（ProblemReport_20261001）: `extrapolate=False` なら、狙いが貯めた点の
        どれより強いとき割線の外挿をせず、いちばん効いた点の開度を返す（G ランプの最初の開度の
        見積もり用。ユーザー決定）。補間は変わらない。
        """
        for i in self._search_order(pedal, self._idx(speed_kmh)):
            cap = self._bin_cap(pedal, i, g_cap_kmhs, deadband_pct, extrapolate=extrapolate)
            if cap is not None:
                return cap
        return None

    def _search_order(self, pedal: str, own: int) -> list[int]:
        """自分の帯 `own` で作れないときに探す車速帯の順。そのペダルが効きやすい側
        （ブレーキは速い側・アクセルは遅い側）を近い順、その後に反対側を近い順。"""
        idxs = sorted({k[1] for k in self._points if k[0] == pedal} | set(self._coast))
        safe_side = [i for i in idxs if (i >= own if pedal == PEDAL_BRAKE else i <= own)]
        other = [i for i in idxs if i not in safe_side]
        return sorted(safe_side, key=lambda i: abs(i - own)) + sorted(
            other, key=lambda i: abs(i - own)
        )

    def _bin_sensitivity(
        self, pedal: str, idx: int, opening_pct: float, deadband_pct: float,
        min_accel_span_kmhs: float = 0.0,
    ) -> float | None:
        pts = list(self._points.get((pedal, idx), ()))
        c = self._coast_median(idx)
        if c is not None:
            pts.append((deadband_pct, c if pedal == PEDAL_BRAKE else -c))
        near = [p for p in pts if abs(p[0] - opening_pct) <= _SENS_WINDOW_PCT]
        # 2026-10-07 段2l-2: 近くの点の傾きと帯の全点の傾きの大きいほう。感度を小さく見積もると
        # 開度を速く動かす側（危険側）に外れるので、大きいほうを採る（062322 に当てた確認で、
        # 近くの点だけだと雑音で極端に小さい値が出ていたのが消えた）
        # 2026-10-10 段2l-11: 傾きを作った点の加速度の差が min_accel_span_kmhs 未満の候補は外す
        slopes = [
            s for s in (
                _lsq_slope(near, 3, min_accel_span_kmhs),
                _lsq_slope(pts, 2, min_accel_span_kmhs),
            ) if s is not None
        ]
        return max(slopes) if slopes else None

    def sensitivity_kmhs_per_pct(
        self, pedal: str, speed_kmh: float, opening_pct: float, deadband_pct: float,
        *, min_accel_span_kmhs: float = 0.0,
    ) -> float | None:
        """`opening_pct` あたりでの開度 1% あたりの加速度の変化（感度）[km/h/s per %]。

        そのペダルの向きの加速度（ブレーキなら減速を正で、踏み増すほど増える向き）。帯内で
        `opening_pct` から ±`_SENS_WINDOW_PCT` の点（3 点以上）の最小二乗の傾きと、帯の全点
        （＋惰行の点）の傾きを両方計算し、作れたものの大きいほう（小さく見積もると開度を速く動かす
        危険側に外れるため）。自分の帯で作れなければ `cap_pct` と同じ順で他の帯。
        どの帯でも作れなければ None（呼び出し側が既定の速さを使う）。

        2026-10-10 段2l-11（ProblemReport_20261001）: `min_accel_span_kmhs`（既定 0 ＝ 従来）。
        2 つの候補それぞれについて「傾き × その候補に使った点の開度の幅」がこれ未満なら候補から
        外す（点が近すぎて加速度の差が雑音と区別できないとき、傾きが 0 に近い正の値になり
        Kp ＝ 1 ÷（感度 × T）が跳ねるため）。両方外れた帯は None で、次の帯を探す。
        """
        for i in self._search_order(pedal, self._idx(speed_kmh)):
            sens = self._bin_sensitivity(
                pedal, i, opening_pct, deadband_pct, min_accel_span_kmhs
            )
            if sens is not None:
                return sens
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
