"""FEL の学習用パターン: モードの統計から規則で作る基準車速（毎回同じ・乱数なし）。

2026-10-19 段2s-2b。手順2 の 2-3 で、手順3 と同じ候補（FF）でこのパターンを走り、
fel.py が補正表を作る。G ランプ（一定の G で上端まで）では足りない「発進 → 加速 → 定速 →
減速 → 停車」の台形と、停車前の戻し方・車速変化・惰行を、モードが通るマスに合わせて並べる。

車に依存する値は使わない。決める材料は次の 2 つだけ:
    ① 選んだモード（modes.coverage_mode_names）の基準車速の統計（G ランプと同じ入力）
    ② 同じ手順2 で測った表の惰行の加速度（table.coast_accel。形 D・高速の惰行の基準）

統計（全モードの基準車速を 1 秒刻みに線形補間した差分。単位 G ＝ km/h/s ÷ G_TO_KMHS）:
    トリップ ＝ 停車（< STOP_SPEED_KMH）と停車の間。トリップごとの最高車速。
    g_acc_med / g_acc_hi ＝ 加速度 > accel_min_g の標本の中央値・75% 点
    g_dec_med / g_dec_hi ＝ 車速 > ease_start_kmh で減速（< −accel_min_g）の標本の |G| の
       中央値・75% 点
    g_end ＝ 各停車の直前 stop_end_window_s の |G| の中央値（停車前の G）
    どの G も learning.ramp_g_levels の最大で頭打ち。

並べ方（頭に停車 stop_hold_s、トリップの間も同じ秒数の停車）:
    ① 低速の短い走り low_trip_count 本: 上の車速 ＝ 最高車速が low_trip_max_kmh 以下のトリップの
       最高車速の (k+0.5)/本数 分位。加速 G は g_acc_med・g_acc_hi を交互、減速 G は g_dec_med・
       g_dec_hi を 2 本ずつ交互。定速 cruise_s。停車前の形は A,B,C,D の繰り返し。
    ② 中速の車速変化 1 本: 上の車速 ＝ 走っている間の基準車速で mid_speed_min_kmh 以上の標本の
       中央値。そこから −mid_step_kmh と +mid_step_kmh の間を mid_round_trips 往復（減速
       g_dec_med・加速 g_acc_med、各段で定速 cruise_s）→ 形 B で停車。
    ③ 高速 1 本: 上の車速 ＝ 走っている間の基準車速の high_speed_quantile 点（上端は G ランプと
       同じ top_speed_cap_kmh 以下）。加速 g_acc_med → 定速 → 惰行 high_coast_s → g_dec_med で
       減速 → 形 A で停車。

停車前の形:
    A 同じ G で停車（G ランプの停止と同じ。−G → 下げ止めを v_ref の平方根で絞る）
    B 車速が ease_start_kmh 以下で、G を減速の G から g_end へ車速に比例して緩めて停車
    C 減速の途中（上の車速の半分）で G を半分にして half_g_s、また戻して停車
    D 惰行 coast_s（表の惰行の加速度。表が無ければ 0G の定速で代用）してからブレーキの G で停車

G の切り替えは G ランプと同じ丸め方（learning.ramp_turn_time_s で直線。発進は ramp_edge_g から
(G − edge) ÷ ramp_launch_g_rate_gps、最短 ramp_launch_blend_min_s。停止は ramp_stop_time_s の
平方根の絞り）。基準車速は 0 未満にしない。

CLI（机上）:
    python -m tests.research.fel_pattern [--config PATH]
        DB のモードから統計とパターンを作って、各部分の秒数・合計・G を表示する。
"""

from __future__ import annotations

import argparse
import asyncio
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from tests.research.ramp_planner import STOP_SPEED_KMH
from tests.research.research_types import G_TO_KMHS, DrivingMode, SpeedPoint

SHAPES = ("A", "B", "C", "D")
SHAPE_TEXT = {
    "A": "同じ G で停車",
    "B": "ease_start 以下で G を g_end へ緩める",
    "C": "途中で G を半分にして戻す",
    "D": "惰行してからブレーキ",
}
PATTERN_MODE_NAME = "FEL学習用パターン"
MIN_TOP_KMH = 5.0  # 台形の上の車速の下限（これ未満だと台形にならない）
MAX_STEPS = 200_000  # 暴走よけ（0.1 秒刻みで 2 万秒）


class PatternError(Exception):
    """モードの統計が取れない（停車がない・加減速の標本がないなど）。"""


@dataclass(frozen=True)
class ModeStats:
    """モードの統計（G は単位 G。上限で頭打ちした後の値）。"""

    g_acc_med: float
    g_acc_hi: float
    g_dec_med: float
    g_dec_hi: float
    g_end: float
    trip_tops_kmh: tuple[float, ...]  # トリップごとの最高車速
    run_speeds_kmh: tuple[float, ...]  # 走っている間（> 0）の基準車速の標本（1 秒刻み）
    g_cap: float

    def lines(self) -> list[str]:
        return [
            f"加速の G 中央値 {self.g_acc_med:.3f} / 75% {self.g_acc_hi:.3f}・"
            f"減速の G 中央値 {self.g_dec_med:.3f} / 75% {self.g_dec_hi:.3f}・"
            f"停車前の G {self.g_end:.3f}（上限 {self.g_cap:.2f}G）",
            f"トリップ {len(self.trip_tops_kmh)} 本・最高車速 "
            f"{min(self.trip_tops_kmh):.1f}〜{max(self.trip_tops_kmh):.1f} km/h"
            if self.trip_tops_kmh else "トリップなし",
        ]


@dataclass(frozen=True)
class PatternPart:
    """パターンの 1 まとまり（表示用）。"""

    name: str
    start_s: float
    end_s: float
    stops: int  # このまとまりが終わるときの停車の数（0 か 1）
    note: str = ""

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


@dataclass
class FelPattern:
    mode: DrivingMode
    parts: list[PatternPart]
    stats: ModeStats
    coast_note: str  # 惰行の基準（表 or 0G の代用）
    tops_kmh: dict[str, float] = field(default_factory=dict)

    @property
    def total_s(self) -> float:
        return self.mode.total_duration

    @property
    def stop_count(self) -> int:
        return sum(p.stops for p in self.parts)


# ─────────────────────────────────────────────────────────────────────
# 統計
# ─────────────────────────────────────────────────────────────────────


def _grid_1s(mode: DrivingMode) -> np.ndarray:
    t = np.array([p.time_s for p in mode.reference_speed], dtype=float)
    v = np.array([p.speed_kmh for p in mode.reference_speed], dtype=float)
    if len(t) < 2:
        return v
    return np.interp(np.arange(t[0], t[-1] + 1e-9, 1.0), t, v)


def compute_stats(
    modes: Sequence[DrivingMode], fel: Any, g_levels: Sequence[float]
) -> ModeStats:
    """全モードの基準車速から統計を作る。標本が足りなければ PatternError。"""
    g_cap = max(g_levels)
    acc_g: list[np.ndarray] = []
    dec_g: list[np.ndarray] = []
    end_g: list[np.ndarray] = []
    tops: list[float] = []
    run: list[np.ndarray] = []
    win = max(1, round(fel.stop_end_window_s))
    for m in modes:
        vg = _grid_1s(m)
        if len(vg) < 2:
            continue
        g = np.diff(vg) / G_TO_KMHS
        mid = (vg[:-1] + vg[1:]) / 2.0
        moving = mid >= STOP_SPEED_KMH
        acc_g.append(g[moving & (g > fel.accel_min_g)])
        dec_g.append(-g[moving & (mid > fel.ease_start_kmh) & (g < -fel.accel_min_g)])
        run.append(vg[vg > 0.0])
        # トリップ（停車と停車の間）と各停車の直前の窓
        is_stop = vg < STOP_SPEED_KMH
        i = 0
        n = len(vg)
        while i < n:
            if is_stop[i]:
                i += 1
                continue
            j = i
            while j < n and not is_stop[j]:
                j += 1
            tops.append(float(vg[i:j].max()))
            if j < n:  # 停車で終わるトリップ: 停車の直前の窓（j-1 が最後の走行点）
                lo = max(j - win, 0)
                end_g.append(-g[lo:j][g[lo:j] < 0.0])
            i = j
    acc = np.concatenate(acc_g) if acc_g else np.zeros(0)
    dec = np.concatenate(dec_g) if dec_g else np.zeros(0)
    end = np.concatenate(end_g) if end_g else np.zeros(0)
    runs = np.concatenate(run) if run else np.zeros(0)
    if acc.size == 0 or dec.size == 0 or end.size == 0 or not tops:
        raise PatternError(
            "モードの基準車速から加速・減速・停車前の標本が取れません"
            f"（加速 {acc.size}・減速 {dec.size}・停車前 {end.size}・トリップ {len(tops)}）"
        )

    def cap(x: float) -> float:
        return float(min(x, g_cap))

    return ModeStats(
        g_acc_med=cap(np.median(acc)), g_acc_hi=cap(np.quantile(acc, 0.75)),
        g_dec_med=cap(np.median(dec)), g_dec_hi=cap(np.quantile(dec, 0.75)),
        g_end=cap(np.median(end)),
        trip_tops_kmh=tuple(tops), run_speeds_kmh=tuple(float(x) for x in runs), g_cap=g_cap,
    )


# ─────────────────────────────────────────────────────────────────────
# 基準車速の組み立て
# ─────────────────────────────────────────────────────────────────────


class _Builder:
    """時刻と車速を dt 刻みで進めて点を貯める。G（符号つき）を与えると車速を積分する。"""

    def __init__(self, fel: Any, learning: Any, coast_fn: Callable[[float], float] | None) -> None:
        self.dt = fel.pattern_dt_s
        self.fel = fel
        self.turn_s = learning.ramp_turn_time_s
        self.edge_g = learning.ramp_edge_g
        self.launch_rate = learning.ramp_launch_g_rate_gps
        self.blend_min_s = learning.ramp_launch_blend_min_s
        self.stop_time_s = learning.ramp_stop_time_s
        self.coast_fn = coast_fn  # 車速 → 加速度 [km/h/s]
        self.n = 0
        self.v = 0.0
        self.vs: list[float] = [0.0]
        self.g_now = 0.0  # 直近の G（符号つき）

    @property
    def t(self) -> float:
        return self.n * self.dt

    def step(self, g: float) -> None:
        if self.n > MAX_STEPS:
            raise PatternError("パターンが終わりません（暴走よけ）")
        self.v = max(0.0, self.v + g * G_TO_KMHS * self.dt)
        self.n += 1
        self.vs.append(self.v)
        self.g_now = g

    def ramp(self, g0: float, g1: float, dur_s: float) -> None:
        """G を g0 から g1 へ dur_s で直線に切り替える（区間の中点の G で積分）。"""
        n = max(1, round(dur_s / self.dt)) if dur_s > 0.0 else 1
        for i in range(n):
            self.step(g0 + (g1 - g0) * (i + 0.5) / n)

    def hold(self, sec: float, g: float = 0.0) -> None:
        for _ in range(max(0, round(sec / self.dt))):
            self.step(g)

    def _eff_g(self, g: float, dv: float, launch: bool) -> float:
        """切り替えの上り下りが dv に収まるよう G を下げる（収まるなら g のまま）。"""
        room = 0.9 * dv / (G_TO_KMHS * max(self.turn_s, self.dt)) if not launch else g
        return max(min(g, room), 0.01)

    def launch_to(self, vtop: float, g: float) -> None:
        """停車から vtop まで g で加速する。発進は edge_g から rate で g へ。"""
        g = self._eff_g(g, vtop, launch=False) if vtop < 2.0 * g * G_TO_KMHS * self.turn_s else g
        g_min = min(self.edge_g, g)
        blend = (
            self.turn_s if self.launch_rate <= 0.0
            else max(self.blend_min_s, (g - g_min) / self.launch_rate)
        )
        lead = g * G_TO_KMHS * self.turn_s / 2.0
        n = max(1, round(blend / self.dt))
        for i in range(n):
            if self.v + lead >= vtop:
                break
            self.step(g_min + (g - g_min) * (i + 0.5) / n)
        self._climb(vtop, g, lead)

    def _climb(self, vtop: float, g: float, lead: float) -> None:
        while self.v + lead < vtop:
            self.step(g)
        self.ramp(self.g_now if self.g_now > 0.0 else g, 0.0, self.turn_s)

    def change_to(self, vt: float, g: float) -> None:
        """定速から vt まで、|g| で加速（vt が上）か減速（vt が下）して定速に戻る。"""
        up = vt > self.v
        mag = self._eff_g(g, abs(vt - self.v), launch=False)
        s = 1.0 if up else -1.0
        lead = mag * G_TO_KMHS * self.turn_s / 2.0
        self.ramp(0.0, s * mag, self.turn_s)
        while (self.v + lead < vt) if up else (self.v - lead > vt):
            self.step(s * mag)
        self.ramp(s * mag, 0.0, self.turn_s)

    def coast(self, sec: float) -> None:
        """惰行 sec 秒。G ＝ 表の惰行の加速度（無ければ 0）を turn_s で 0 から入れる。"""
        n = max(0, round(sec / self.dt))
        for i in range(n):
            g_c = 0.0
            if self.coast_fn is not None:
                g_c = min(0.0, self.coast_fn(self.v) / G_TO_KMHS)
            frac = min(1.0, (i + 0.5) * self.dt / self.turn_s) if self.turn_s > 0.0 else 1.0
            self.step(g_c * frac)
            if self.v <= 0.0:
                break

    def descend_stop(self, g: float, shape: str, g_end: float, g0: float = 0.0) -> None:
        """減速して停車する。g0 ＝ 今の G（切り替えの起点）。A/B（と C/D の後半）の形。"""
        cfg = self.fel
        g_min = min(self.edge_g, g)
        v_stop = g * G_TO_KMHS * self.stop_time_s / 2.0
        e = 0.0
        while self.v > 0.0:
            frac = min(1.0, e / self.turn_s) if self.turn_s > 0.0 else 1.0
            turn = g0 + (-g - g0) * frac
            if shape == "B":
                mag = g if self.v > cfg.ease_start_kmh else (
                    g_end + (g - g_end) * self.v / cfg.ease_start_kmh
                )
                stop = -max(mag, 0.01)
            elif v_stop <= 0.0:
                stop = -g
            else:
                stop = -max(g_min, g * min(1.0, math.sqrt(self.v / v_stop)))
            self.step(max(turn, stop))
            e += self.dt

    def decel_until(self, g: float, v_until: float) -> None:
        """0 → −g を turn_s で切り替え、v ≤ v_until まで −g（形 C の前半）。"""
        self.ramp(0.0, -g, self.turn_s)
        while self.v > v_until:
            self.step(-g)

    def stop_hold(self, sec: float) -> None:
        for _ in range(max(0, round(sec / self.dt))):
            self.step(0.0)


def _trip(
    b: _Builder, vtop: float, g_acc: float, g_dec: float, shape: str, g_end: float, cruise_s: float
) -> None:
    b.launch_to(vtop, g_acc)
    b.hold(cruise_s)
    _stop_with_shape(b, vtop, g_dec, shape, g_end)


def _stop_with_shape(b: _Builder, vtop: float, g_dec: float, shape: str, g_end: float) -> None:
    fel = b.fel
    if shape == "C":
        b.decel_until(g_dec, vtop / 2.0)
        if b.v > 0.0:
            b.ramp(-g_dec, -g_dec / 2.0, b.turn_s)
            b.hold(fel.half_g_s, -g_dec / 2.0)
        if b.v > 0.0:
            b.ramp(-g_dec / 2.0, -g_dec, b.turn_s)
            b.descend_stop(g_dec, "A", g_end, g0=-g_dec)
    elif shape == "D":
        b.coast(fel.coast_s)
        b.descend_stop(g_dec, "A", g_end, g0=b.g_now)
    else:  # A / B
        b.descend_stop(g_dec, shape, g_end, g0=0.0)


def build_pattern(
    stats: ModeStats,
    fel: Any,
    learning: Any,
    *,
    top_cap_kmh: float,
    coast_accel_kmhs: Callable[[float], float] | None = None,
) -> FelPattern:
    """統計から学習用パターン（DrivingMode）を作る。同じ入力なら同じ列。

    coast_accel_kmhs: 車速 [km/h] → 惰行の加速度 [km/h/s]（同じ手順2 の表の惰行）。None なら 0G。
    """
    b = _Builder(fel, learning, coast_accel_kmhs)
    parts: list[PatternPart] = []
    tops: dict[str, float] = {}

    b.stop_hold(fel.stop_hold_s)
    parts.append(PatternPart("頭の停車", 0.0, b.t, 0))

    # ① 低速の短い走り
    low = np.array([x for x in stats.trip_tops_kmh if x <= fel.low_trip_max_kmh])
    if low.size == 0:
        low = np.array(stats.trip_tops_kmh)
    n_low = fel.low_trip_count
    for k in range(n_low):
        vtop = float(np.quantile(low, (k + 0.5) / n_low))
        vtop = min(max(vtop, MIN_TOP_KMH), top_cap_kmh)
        g_acc = stats.g_acc_med if k % 2 == 0 else stats.g_acc_hi
        g_dec = stats.g_dec_med if (k // 2) % 2 == 0 else stats.g_dec_hi
        shape = SHAPES[k % 4]
        t0 = b.t
        _trip(b, vtop, g_acc, g_dec, shape, stats.g_end, fel.cruise_s)
        name = f"① 低速 {k + 1}"
        tops[name] = vtop
        parts.append(PatternPart(
            name, t0, b.t, 1,
            f"{vtop:.1f} km/h・加速 {g_acc:.3f}G・減速 {g_dec:.3f}G・形 {shape}",
        ))
        b.stop_hold(fel.stop_hold_s)
        parts.append(PatternPart("停車", b.t - fel.stop_hold_s, b.t, 0))

    # ② 中速の車速変化
    mids = np.array([x for x in stats.run_speeds_kmh if x >= fel.mid_speed_min_kmh])
    v_mid = float(np.median(mids)) if mids.size else float(np.median(stats.run_speeds_kmh))
    hi = min(v_mid + fel.mid_step_kmh, top_cap_kmh)
    lo = max(v_mid - fel.mid_step_kmh, MIN_TOP_KMH)
    t0 = b.t
    b.launch_to(v_mid, stats.g_acc_med)
    b.hold(fel.cruise_s)
    for _ in range(fel.mid_round_trips):
        b.change_to(lo, stats.g_dec_med)
        b.hold(fel.cruise_s)
        b.change_to(hi, stats.g_acc_med)
        b.hold(fel.cruise_s)
    _stop_with_shape(b, hi, stats.g_dec_med, "B", stats.g_end)
    tops["② 中速"] = v_mid
    parts.append(PatternPart(
        "② 中速", t0, b.t, 1,
        f"{v_mid:.1f} km/h を中心に {lo:.1f}↔{hi:.1f} km/h を {fel.mid_round_trips} 往復・"
        f"加速 {stats.g_acc_med:.3f}G・減速 {stats.g_dec_med:.3f}G・形 B",
    ))
    b.stop_hold(fel.stop_hold_s)
    parts.append(PatternPart("停車", b.t - fel.stop_hold_s, b.t, 0))

    # ③ 高速
    v_hi = float(np.quantile(np.array(stats.run_speeds_kmh), fel.high_speed_quantile))
    v_hi = min(max(v_hi, MIN_TOP_KMH), top_cap_kmh)
    t0 = b.t
    b.launch_to(v_hi, stats.g_acc_med)
    b.hold(fel.cruise_s)
    b.coast(fel.high_coast_s)
    b.ramp(b.g_now, -stats.g_dec_med, b.turn_s)
    b.descend_stop(stats.g_dec_med, "A", stats.g_end, g0=-stats.g_dec_med)
    tops["③ 高速"] = v_hi
    parts.append(PatternPart(
        "③ 高速", t0, b.t, 1,
        f"{v_hi:.1f} km/h・加速 {stats.g_acc_med:.3f}G・惰行 {fel.high_coast_s:g}s・"
        f"減速 {stats.g_dec_med:.3f}G・形 A",
    ))

    ts = [i * b.dt for i in range(len(b.vs))]
    mode = DrivingMode(
        id="fel_pattern", name=PATTERN_MODE_NAME,
        description="FEL の学習用パターン（モードの統計から規則で作る）",
        reference_speed=[
            SpeedPoint(round(t, 6), round(v, 6)) for t, v in zip(ts, b.vs, strict=True)
        ],
        total_duration=ts[-1], max_speed=max(b.vs), created_at=datetime.now(), is_system=True,
    )
    coast_note = (
        "表の惰行の加速度（table.coast_accel）" if coast_accel_kmhs is not None
        else "表なし: 0G の定速で代用"
    )
    return FelPattern(mode=mode, parts=parts, stats=stats, coast_note=coast_note, tops_kmh=tops)


def make_pattern(
    cfg: Any,
    modes: Sequence[DrivingMode],
    *,
    top_cap_kmh: float,
    table: Any = None,
) -> FelPattern:
    """設定・モード・表（惰行）から作る。table が None なら惰行は 0G で代用。"""
    stats = compute_stats(modes, cfg.fel, cfg.learning.ramp_g_levels)
    coast_fn = None if table is None else (lambda v: float(table.coast_accel(v)))
    return build_pattern(
        stats, cfg.fel, cfg.learning, top_cap_kmh=top_cap_kmh, coast_accel_kmhs=coast_fn
    )


def format_pattern(p: FelPattern) -> str:
    """統計・各部分の秒数・停車回数・合計の表示。"""
    lines = ["モードの統計: " + "／".join(p.stats.lines())]
    lines.append(f"惰行の基準: {p.coast_note}")
    for part in p.parts:
        if part.name == "停車":
            continue
        extra = f"　{part.note}" if part.note else ""
        lines.append(f"  {part.name}: {part.duration_s:6.1f}s（{part.start_s:6.1f}〜"
                     f"{part.end_s:6.1f}s）{extra}")
    run_s = sum(x.duration_s for x in p.parts if x.name not in ("停車", "頭の停車"))
    hold_s = sum(x.duration_s for x in p.parts if x.name in ("停車", "頭の停車"))
    lines.append(
        f"  合計 {p.total_s:.1f}s（{p.total_s / 60.0:.1f} 分。走り {run_s:.1f}s ＋ 停車 "
        f"{hold_s:.1f}s）・停車 {p.stop_count} 回・最高 {p.mode.max_speed:.1f} km/h"
    )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    from tests.research.config import DEFAULT_CONFIG_PATH, load_config  # noqa: PLC0415
    from tests.research.fwd_map import load_forward_map  # noqa: PLC0415
    from tests.research.mode_drive import load_mode  # noqa: PLC0415
    from tests.research.ramp_planner import top_speed_cap_kmh  # noqa: PLC0415

    ap = argparse.ArgumentParser(description="FEL の学習用パターンを作って表示する（机上）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--model", type=Path, default=None, help="惰行の表（pkl。省略は 0G で代用）")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)

    async def _modes() -> list[DrivingMode]:
        return [await load_mode(cfg, n) for n in cfg.modes.coverage_mode_names]

    modes = asyncio.run(_modes())
    cap = top_speed_cap_kmh(modes, cfg.learning.ramp_top_margin_kmh, cfg.vehicle.max_speed_kmh)
    table = None if args.model is None else load_forward_map(args.model)
    print(format_pattern(make_pattern(cfg, modes, top_cap_kmh=cap, table=table)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
