"""P＋DOB のループの余裕を、加振で測った P(f) から机上で計算する（2026-10-19 段2t）。

目的（決まり G2）: C10 の τ ＝ c × 遅れ と C11 の DOB の Tq ＝ 係数 × 遅れ を、測った余裕
（ゲイン余裕 6 dB・位相余裕 45° が目安）から決め直し、「係数 × 遅れ」で全車共通に書けるかを
判定する。
走行はしない（加振の CSV・順モデルの pkl・yaml を読む計算だけ）。

記号:
    s ＝ j2πf、P(s) ＝ ペダルの符号つき開度 u（アクセル正・ブレーキ負）[%] → 車速 V [km/h]
    （加振で測る）
    S ＝ 順モデルの表の感度 ∂a/∂u [km/h/s per %]（運転点の中心差分）
    a_v ＝ 表の ∂a/∂v [1/s]（惰行の減速の車速依存。純積分器なら 0）
    L_d ＝ 順モデルの遅れ（pkl の meta、yaml の forward_map.*_lag_s で上書き）
    τ ＝ c × L_d（C10）、Tq ＝ k × L_d（C11）、T ＝ 制御周期

式（コードの読み取り）:
  C10（`fwd_plan.deviation_offset_detail`）: 偏差 dev ＝ 基準 − 実車速。Δa ＝ dev ÷ τ を要る加速度に
  足して表を引き直し、基準のままとの差を開度に足す。差は局所では Δu ＝ Δa ÷ S なので、
      Δu ＝ K (R − V),  K(v) ＝ 1 ÷ (τ・S)  [%/(km/h)]
  K は 1 ÷（偏差を 0 に戻す時間 τ × 表の感度）。P の経路だけの開ループ伝達は L_P(s) ＝ K・P(s)。

  C11（`dob.PedalDisturbanceObserver`）: d̂ ＝ LPF_Tq( a_meas − a_table(v, u(t − L_d)) )。
  a_meas は CAN 車速の 1 周期の後退差分 ＝ H(s)・V、H(s) ＝ (1 − e^{−sT}) ÷ T。表の予測を
  u と v で線形化すると a_table ＝ S・e^{−sL_d}・U + a_v・V。要る加速度から d̂ を引いて表を引くので
      Δu_DOB ＝ −d̂ ÷ S,   d̂ ＝ ( (H − a_v)・P − S・e^{−sL_d} ) ・ U ÷ ( 1 + sTq )
  （V ＝ P・U を代入した。表が車を完全に表すなら括弧の中が 0 で、DOB は何もしない）。

  合わせたループ: U ＝ U_ff + K (R − V) − d̂ ÷ S に V ＝ P・U を入れると
      U ・ ( 1 + G + K・P ) ＝ U_ff + K・R
      G(s) ＝ ( (H − a_v)・P − S・e^{−sL_d} ) ÷ ( S・(1 + sTq) )
  よって開ループ伝達 L(s) ＝ K・P(s) + G(s)（特性方程式 1 + L ＝ 0。外乱の増幅率 ＝ 1 ÷ |1 + L|）。
  純積分器 P ＝ S・e^{−sL_d} ÷ s（a_v ＝ 0）で T → 0 なら G ＝ 0 になり、L ＝ K・P だけ。

  ペダルごとの差は S（運転点）と L_d、P(f) の測定だけ。ブレーキは符号つき開度 u に対する P で測る
  （excite.py）ので式は同じ。運転点は、アクセルは「惰行より少し速く加速する開度」、ブレーキは
  「惰行 − brake_extra の減速になる開度」（excite の `brake_extra_decel_kmhs` と同じ）。

余裕の定義:
  交差周波数 ＝ |L| が 1 を下る周波数。位相余裕 ＝ 180° ＋ その点の ∠L（連続に巻いた位相）。
  複数あれば最小。位相交差周波数 ＝ ∠L が −180° を通る周波数。ゲイン余裕 ＝ −20 log10 |L|
  （その点）。複数あれば最小。
  |L| が測定帯で常に 1 未満なら交差なし（位相余裕は定義されない）。|L| が帯の上端でも 1 以上なら
  交差が帯の外（測定が足りない）として注記する。感度の山 Ms ＝ max 1 ÷ |1 + L|。
  P(f) は測った点の間を、対数周波数で |P| の対数と連続位相を直線補間する。帯の外は外挿しない。

判定の仮値（`--gm-db`・`--pm-deg`・`--align-ratio` で変える）:
  ゲイン余裕 6 dB・位相余裕 45°（決まり G2）。「速度ごとの必要な c がそろう」は、速度ごとの
  最小の c の最大 ÷ 最小 ≦ 2（仮値。そろわなければ止めて案を残す、が G2）。

CLI:
    .venv/bin/python -m tests.research.loop_margin <excite CSV> --pkl <fwd_model pkl>
    .venv/bin/python -m tests.research.loop_margin --past-check   # 2026-09-24 の実測で C5 を検算
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.debug_process23 import md_table
from tests.research.excite import BlockResponse, analyze
from tests.research.fwd_map import load_forward_map

PEDAL_ACCEL = "accel"
PEDAL_BRAKE = "brake"
_PEDAL_JA = {PEDAL_ACCEL: "アクセル", PEDAL_BRAKE: "ブレーキ"}

# 掃引する c と Tq の係数（c は τ ＝ c × 遅れ、k は Tq ＝ k × 遅れ）。粗い対数格子
C_GRID = (3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.5, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 70.0, 100.0)
K_GRID = (2.5, 5.0, 10.0, 20.0, 40.0, 80.0, 160.0)


# ─────────────────────────────────────────────────────────────────────
# 測った P(f)
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class PlantFrf:
    """ペダル・速度ごとの測った P(f)。位相は低周波側から連続に巻いた [deg]。"""

    pedal: str
    v_kmh: float
    freqs_hz: np.ndarray
    gain: np.ndarray  # km/h per %
    phase_deg: np.ndarray  # 連続（−180° をまたいで遅れ続ける）
    snr: np.ndarray

    @property
    def f_min(self) -> float:
        return float(self.freqs_hz[0])

    @property
    def f_max(self) -> float:
        return float(self.freqs_hz[-1])

    def at(self, f_hz: np.ndarray) -> np.ndarray:
        """測定帯の中の P(f)（複素数）。|P| の対数と連続位相を対数周波数で直線補間する。"""
        lf = np.log(self.freqs_hz)
        mag = np.exp(np.interp(np.log(f_hz), lf, np.log(self.gain)))
        ph = np.interp(np.log(f_hz), lf, np.radians(self.phase_deg))
        return mag * np.exp(1j * ph)


def build_frfs(blocks: list[BlockResponse]) -> dict[tuple[str, float], PlantFrf]:
    """`excite.analyze` の結果を (ペダル, 速度) ごとの P(f) にまとめる。

    位相は周波数順に np.unwrap して連続にする。先頭の位相が (−270°, 90°] に入るよう 360° の倍数で
    ずらす（アクセル・ブレーキとも低周波は −90° 付近から遅れる向き）。
    """
    groups: dict[tuple[str, float], list[BlockResponse]] = {}
    for b in blocks:
        if not (math.isfinite(b.gain_kmh_per_pct) and b.gain_kmh_per_pct > 0.0):
            continue
        groups.setdefault((b.pedal, b.v_star_kmh), []).append(b)
    out: dict[tuple[str, float], PlantFrf] = {}
    for (pedal, v), items in sorted(groups.items()):
        items.sort(key=lambda b: b.freq_hz)
        if len(items) < 2:
            continue
        ph = np.degrees(np.unwrap(np.radians([b.phase_deg for b in items])))
        shift = 360.0 * math.ceil((ph[0] - 90.0) / 360.0)  # ph[0] - shift が (−270, 90]
        ph = ph - shift
        out[(pedal, v)] = PlantFrf(
            pedal=pedal, v_kmh=v,
            freqs_hz=np.array([b.freq_hz for b in items]),
            gain=np.array([b.gain_kmh_per_pct for b in items]),
            phase_deg=ph, snr=np.array([b.snr for b in items]),
        )
    return out


# ─────────────────────────────────────────────────────────────────────
# 表（pkl）から運転点の感度
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class OperatingPoint:
    pedal: str
    v_kmh: float
    u_pct: float  # 運転点の符号つき開度（min からの量）
    sens: float  # S ＝ ∂a/∂u [km/h/s per %]
    a_v: float  # ∂a/∂v [1/s]
    lag_s: float  # そのペダルの遅れ


def operating_point(
    table: Any, pedal: str, v_kmh: float, lag_s: float, *,
    brake_extra_kmhs: float, accel_above_coast_kmhs: float = 0.1, du: float = 0.5,
) -> OperatingPoint:
    """表の運転点と感度 S・a_v を中心差分で求める。

    アクセル: 惰行の加速度（u ＝ 0）より `accel_above_coast_kmhs` 速く加速する開度（惰行の
    加速度が正のクリープ域でも、アクセルの領域に入る）。ブレーキ: 惰行より `brake_extra_kmhs`
    強く減速する開度。S が正でなければ ValueError（表が単調でない・平らな所）。
    """
    coast = float(table.coast_accel(v_kmh))
    if pedal == PEDAL_ACCEL:
        target = max(coast, 0.0) + accel_above_coast_kmhs
    else:
        target = coast - brake_extra_kmhs
    u0 = float(table.invert(v_kmh, target)[0])
    if pedal == PEDAL_ACCEL:
        u0 = max(u0, du)
    else:
        u0 = min(u0, -du)
    sens = float(table.accel(v_kmh, u0 + du) - table.accel(v_kmh, u0 - du)) / (2.0 * du)
    a_v = float(table.accel(v_kmh + 1.0, u0) - table.accel(v_kmh - 1.0, u0)) / 2.0
    if not sens > 0.0:
        raise ValueError(
            f"{_PEDAL_JA[pedal]} {v_kmh:g} km/h の感度 ∂a/∂u が正でありません（{sens:g}）"
        )
    return OperatingPoint(pedal, v_kmh, u0, sens, a_v, lag_s)


# ─────────────────────────────────────────────────────────────────────
# ループ伝達
# ─────────────────────────────────────────────────────────────────────


def loop_terms(
    f_hz: np.ndarray, p: np.ndarray, *, sens: float, a_v: float, lag_s: float,
    tau_s: float | None, tq_s: float | None, dt_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(L, L_P, G)。L ＝ K・P + G（モジュールの docstring の式）。

    tau_s が None（そのペダルは C10 の上乗せなし）なら K ＝ 0、tq_s が None（DOB なし）なら G ＝ 0。
    """
    s = 2j * np.pi * np.asarray(f_hz, dtype=float)
    k_gain = 0.0 if tau_s is None else 1.0 / (tau_s * sens)
    l_p = k_gain * p
    if tq_s is None:
        g = np.zeros_like(l_p)
    else:
        h = (1.0 - np.exp(-s * dt_s)) / dt_s  # CAN 車速の 1 周期後退差分
        g = ((h - a_v) * p - sens * np.exp(-s * lag_s)) / (sens * (1.0 + s * tq_s))
    return l_p + g, l_p, g


@dataclass(frozen=True)
class Margins:
    crossover_hz: float | None  # 位相余裕が最小になる交差周波数
    pm_deg: float | None
    phase_cross_hz: float | None  # ゲイン余裕が最小になる位相交差周波数
    gm_db: float | None
    ms: float  # 感度の山 max 1/|1+L|
    ms_hz: float
    note: str = ""

    def ok(self, gm_req_db: float, pm_req_deg: float) -> bool:
        gm_ok = self.gm_db is None or self.gm_db >= gm_req_db
        pm_ok = self.pm_deg is None or self.pm_deg >= pm_req_deg
        return gm_ok and pm_ok


def margins(f_hz: np.ndarray, loop: np.ndarray) -> Margins:
    """L(f)（昇順の周波数格子）から余裕を求める。定義はモジュールの docstring。"""
    mag = np.abs(loop)
    ph = np.degrees(np.unwrap(np.angle(loop)))
    lf = np.log(f_hz)
    notes: list[str] = []

    pm_best: float | None = None
    fc_best: float | None = None
    for i in range(len(f_hz) - 1):
        if mag[i] >= 1.0 > mag[i + 1]:
            w = (math.log(1.0) - math.log(mag[i])) / (math.log(mag[i + 1]) - math.log(mag[i]))
            pm = 180.0 + float(ph[i] + w * (ph[i + 1] - ph[i]))
            if pm_best is None or pm < pm_best:
                pm_best, fc_best = pm, float(np.exp(lf[i] + w * (lf[i + 1] - lf[i])))
    if pm_best is None:
        if mag[-1] >= 1.0:
            notes.append("|L|≧1 が帯の上端まで続く（交差が帯の外）")
        elif mag[0] < 1.0:
            notes.append("交差なし（帯の下端から |L|＜1）")
        else:  # 下端は ≧1 でどこかで下がるはずだが、格子の端の丸めで取れなかった
            notes.append("交差を特定できず")

    gm_best: float | None = None
    fp_best: float | None = None
    for i in range(len(f_hz) - 1):
        a, b = ph[i] + 180.0, ph[i + 1] + 180.0
        # −180° ＋ 360°n を横切る点
        for n in range(math.floor(min(a, b) / 360.0), math.floor(max(a, b) / 360.0) + 1):
            a_n, b_n = a - 360.0 * n, b - 360.0 * n
            if a_n * b_n < 0.0 or a_n == 0.0:
                w = a_n / (a_n - b_n) if a_n != b_n else 0.0
                lm = math.log(mag[i]) + w * (math.log(mag[i + 1]) - math.log(mag[i]))
                gm = -20.0 * lm / math.log(10.0)
                if gm_best is None or gm < gm_best:
                    gm_best = gm
                    fp_best = float(np.exp(lf[i] + w * (lf[i + 1] - lf[i])))
    sens = 1.0 / np.abs(1.0 + loop)
    k = int(np.argmax(sens))
    return Margins(
        fc_best, pm_best, fp_best, gm_best, float(sens[k]), float(f_hz[k]), "；".join(notes)
    )


@dataclass(frozen=True)
class LoopCase:
    """1 つの (ペダル, 速度)。測った P(f)・運転点・表の遅れを持つ。"""

    frf: PlantFrf
    op: OperatingPoint
    dt_s: float

    def margins_for(self, c: float | None, k: float | None, *, n_grid: int = 600) -> Margins:
        """τ ＝ c × 遅れ、Tq ＝ k × 遅れ のときの余裕（c または k が None ならその項なし）。"""
        f = np.geomspace(self.frf.f_min, self.frf.f_max, n_grid)
        p = self.frf.at(f)
        loop, _lp, _g = loop_terms(
            f, p, sens=self.op.sens, a_v=self.op.a_v, lag_s=self.op.lag_s,
            tau_s=None if c is None else c * self.op.lag_s,
            tq_s=None if k is None else k * self.op.lag_s, dt_s=self.dt_s,
        )
        return margins(f, loop)


def margins_for_p_only(frf: PlantFrf, k_gain: float, *, n_grid: int = 600) -> Margins:
    """測った P(f) に比例ゲイン K [%/(km/h)] だけを掛けたループの余裕（C5 などの検算用）。"""
    f = np.geomspace(frf.f_min, frf.f_max, n_grid)
    return margins(f, k_gain * frf.at(f))


# ─────────────────────────────────────────────────────────────────────
# c・k の範囲と共通判定
# ─────────────────────────────────────────────────────────────────────


def _feasible_from(values: tuple[float, ...], ok: list[bool]) -> float | None:
    """昇順の values のうち、それ以上がすべて ok になる最小値（c も k も大きいほど弱い）。"""
    best = None
    for v, good in zip(reversed(values), reversed(ok), strict=True):
        if not good:
            break
        best = v
    return best


@dataclass(frozen=True)
class Verdict:
    c_min_by_case: dict[tuple[str, float], float | None]  # k ＝ k_now のときの最小 c
    k_min_by_case: dict[tuple[str, float], float | None]  # c ＝ c_now のときの最小 k
    common_ok: np.ndarray  # (len(C_GRID), len(K_GRID)) 全ケースが合格か
    common_c_min: float | None  # k_now のときの共通の最小 c
    aligned_ratio: dict[str, float | None]  # ペダルごとの max/min（速度ごとの最小 c）
    aligned: bool


def judge(
    cases: list[LoopCase], *, c_now: float, k_now: float, gm_db: float, pm_deg: float,
    align_ratio: float,
) -> Verdict:
    """全ケースの c・k の合格範囲と、「係数 × 遅れで共通に書けるか」の判定。"""
    c_min: dict[tuple[str, float], float | None] = {}
    k_min: dict[tuple[str, float], float | None] = {}
    common = np.ones((len(C_GRID), len(K_GRID)), dtype=bool)
    common_c_ok = [True] * len(C_GRID)  # k ＝ k_now のとき全ケースが合格する c
    for case in cases:
        key = (case.frf.pedal, case.frf.v_kmh)
        common &= np.array(
            [[case.margins_for(c, k).ok(gm_db, pm_deg) for k in K_GRID] for c in C_GRID]
        )
        c_ok = [case.margins_for(c, k_now).ok(gm_db, pm_deg) for c in C_GRID]
        common_c_ok = [a and b for a, b in zip(common_c_ok, c_ok, strict=True)]
        c_min[key] = _feasible_from(C_GRID, c_ok)
        k_min[key] = _feasible_from(
            K_GRID, [case.margins_for(c_now, k).ok(gm_db, pm_deg) for k in K_GRID]
        )
    ratio: dict[str, float | None] = {}
    aligned = True
    for pedal in (PEDAL_ACCEL, PEDAL_BRAKE):
        vals = [v for (p, _), v in c_min.items() if p == pedal]
        if not vals:
            continue
        finite = [v for v in vals if v is not None]
        if len(finite) < len(vals):
            ratio[pedal] = None
            aligned = False
        else:
            r = max(finite) / min(finite)
            ratio[pedal] = r
            aligned = aligned and r <= align_ratio
    common_c_min = _feasible_from(C_GRID, common_c_ok)
    return Verdict(c_min, k_min, common, common_c_min, ratio, aligned)


# ─────────────────────────────────────────────────────────────────────
# 表示
# ─────────────────────────────────────────────────────────────────────


def _fmt(x: float | None, spec: str = ".2f", none: str = "—") -> str:
    return none if x is None else format(x, spec)


def _edge(x: float | None, grid: tuple[float, ...]) -> str:
    """格子の最小値で合格しているときは「≦」を付ける（それより小さい値も合格の可能性）。"""
    if x is None:
        return "なし"
    return f"≦{x:g}" if x == grid[0] else f"{x:g}"


def margins_table(rows: list[tuple[LoopCase, Margins]], gm_db: float, pm_deg: float) -> str:
    header = [
        "ペダル", "速度 [km/h]", "S [km/h/s per %]", "遅れ [s]", "交差 [Hz]", "位相余裕 [°]",
        "位相交差 [Hz]", "ゲイン余裕 [dB]", "感度の山 Ms（Hz）", "判定", "注記",
    ]
    body = []
    for case, m in rows:
        body.append([
            _PEDAL_JA[case.frf.pedal], f"{case.frf.v_kmh:.0f}", f"{case.op.sens:.3f}",
            f"{case.op.lag_s:.3f}", _fmt(m.crossover_hz, ".3f"), _fmt(m.pm_deg, ".1f"),
            _fmt(m.phase_cross_hz, ".2f"), _fmt(m.gm_db, ".1f"),
            f"{m.ms:.2f}（{m.ms_hz:.2f}）",
            "OK" if m.ok(gm_db, pm_deg) else "NG", m.note,
        ])
    return md_table(header, body)


def common_table(common: np.ndarray) -> str:
    header = ["c ＼ Tq 係数 k"] + [f"{k:g}" for k in K_GRID]
    body = [
        [f"{c:g}"] + ["○" if common[i, j] else "×" for j in range(len(K_GRID))]
        for i, c in enumerate(C_GRID)
    ]
    return md_table(header, body)


# ─────────────────────────────────────────────────────────────────────
# 過去の実測での検算（ProblemReport_20260921 2-9-2）
# ─────────────────────────────────────────────────────────────────────

# 2026-09-24 に前の車で測った P(f)（drive_log_real_20260924_055632.csv）。
# ゲイン [km/h per %]・位相 [°]
# （位相は連続に巻いた値）。周波数 0.2 / 0.35 / 0.5 / 0.7 / 1.0 / 1.4 / 2.0 Hz
PAST_FREQS_HZ = (0.2, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0)
PAST_P: dict[float, tuple[tuple[float, ...], tuple[float, ...]]] = {
    60.0: ((3.988, 2.027, 1.620, 1.055, 0.892, 0.674, 0.582),
           (-114.5, -130.6, -141.2, -149.8, -172.4, -195.5, -231.9)),
    100.0: ((3.386, 2.134, 1.521, 1.122, 0.805, 0.618, 0.548),
            (-105.6, -117.5, -126.9, -143.5, -165.4, -191.9, -230.0)),
}
# 前の車の C5 の実質 Kp [%/(km/h)]（ProblemReport_20260921: 9/21 は 0.97〜1.44、60 km/h は 1.08。
# 0.30 は到達 PI の Kp）
PAST_C5_KP = (0.30, 0.97, 1.08, 1.44)


def past_frf(v_kmh: float) -> PlantFrf:
    gain, phase = PAST_P[v_kmh]
    return PlantFrf(
        PEDAL_ACCEL, v_kmh, np.array(PAST_FREQS_HZ), np.array(gain), np.array(phase),
        np.full(len(gain), 10.0),
    )


def past_check_text() -> str:
    """C5（比例ゲイン K だけと見なす）で、前の車の測定がばたつき（約 1〜1.4 Hz）を説明するか。"""
    rows = []
    for v in sorted(PAST_P):
        frf = past_frf(v)
        for k_gain in PAST_C5_KP:
            m = margins_for_p_only(frf, k_gain)
            rows.append([
                f"{v:.0f}", f"{k_gain:.2f}", _fmt(m.phase_cross_hz, ".2f"), _fmt(m.gm_db, ".1f"),
                _fmt(m.crossover_hz, ".3f"), _fmt(m.pm_deg, ".1f"), f"{m.ms:.2f}（{m.ms_hz:.2f}）",
            ])
    header = ["速度 [km/h]", "実質 Kp [%/(km/h)]", "位相交差 [Hz]", "ゲイン余裕 [dB]",
              "交差 [Hz]", "位相余裕 [°]", "感度の山 Ms（Hz）"]
    return md_table(header, rows)


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


def _lags(table: Any, cfg: Any) -> dict[str, float | None]:
    fm = cfg.forward_map
    la = fm.accel_lag_s if fm.accel_lag_s is not None else table.meta.get("accel_lag_s")
    lb = fm.brake_lag_s if fm.brake_lag_s is not None else table.meta.get("brake_lag_s")
    return {
        PEDAL_ACCEL: None if la is None else float(la),
        PEDAL_BRAKE: None if lb is None else float(lb),
    }


def build_cases(
    blocks: list[BlockResponse], table: Any, cfg: Any, *, brake_extra_kmhs: float
) -> tuple[list[LoopCase], list[str]]:
    lags = _lags(table, cfg)
    cases: list[LoopCase] = []
    skipped: list[str] = []
    for (pedal, v), frf in build_frfs(blocks).items():
        lag = lags[pedal]
        if lag is None:
            skipped.append(f"{_PEDAL_JA[pedal]} {v:g} km/h: 遅れが測れていない")
            continue
        try:
            op = operating_point(table, pedal, v, lag, brake_extra_kmhs=brake_extra_kmhs)
        except ValueError as exc:
            skipped.append(str(exc))
            continue
        cases.append(LoopCase(frf, op, cfg.control.loop_interval_s))
    return cases, skipped


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m tests.research.loop_margin",
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("csv", nargs="?", type=Path, help="加振走行の CSV（手順11）")
    parser.add_argument("--pkl", type=Path, default=None, help="順モデルの pkl（fwd_model_*.pkl）")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--skip-s", type=float, default=2.0)
    parser.add_argument(
        "--c", type=float, default=None, help="C10 の c（既定は yaml の deviation_c）"
    )
    parser.add_argument(
        "--k", type=float, default=None, help="Tq の係数（既定は yaml の dob_tq_coef）"
    )
    parser.add_argument("--gm-db", type=float, default=6.0)
    parser.add_argument("--pm-deg", type=float, default=45.0)
    parser.add_argument("--align-ratio", type=float, default=2.0)
    parser.add_argument(
        "--brake-extra", type=float, default=None,
        help="ブレーキの運転点: 惰行より強い減速 [km/h/s]（既定は yaml の brake_extra_decel_kmhs）",
    )
    parser.add_argument("--past-check", action="store_true", help="前の車の実測で C5 を検算する")
    args = parser.parse_args(argv)

    if args.past_check:
        print("### 過去の実測（2026-09-24 前の車）× C5 の実質 Kp（比例のみと見なす）\n")
        print(past_check_text())
        return 0
    if args.csv is None or args.pkl is None:
        parser.error("csv と --pkl が要ります（検算だけなら --past-check）")

    cfg = load_config(args.config)
    try:
        blocks = analyze(args.csv, skip_s=args.skip_s)
        table = load_forward_map(args.pkl)
    except (OSError, ValueError) as exc:
        print(f"エラー: {exc}", file=sys.stderr)
        return 2
    if not blocks:
        print(f"EXCITE の加振ブロックが見つかりません: {args.csv}", file=sys.stderr)
        return 1
    c_now = cfg.forward_map.deviation_c if args.c is None else args.c
    k_now = cfg.forward_map.dob_tq_coef if args.k is None else args.k
    extra = cfg.excite.brake_extra_decel_kmhs if args.brake_extra is None else args.brake_extra

    cases, skipped = build_cases(blocks, table, cfg, brake_extra_kmhs=extra)
    if not cases:
        print("計算できるケースがありません: " + "；".join(skipped), file=sys.stderr)
        return 1

    print(f"### 設定: c ＝ {c_now:g}（τ ＝ c × 遅れ）・Tq 係数 ＝ {k_now:g}・"
          f"判定 ゲイン余裕 {args.gm_db:g} dB 以上・位相余裕 {args.pm_deg:g}° 以上\n")
    for msg in skipped:
        print(f"- 飛ばした: {msg}")
    print("\n### 1. 今の設定の余裕（P ＋ DOB）\n")
    rows = [(case, case.margins_for(c_now, k_now)) for case in cases]
    print(margins_table(rows, args.gm_db, args.pm_deg))
    print("\n### 2. P だけ（DOB なし）の余裕\n")
    rows_p = [(case, case.margins_for(c_now, None)) for case in cases]
    print(margins_table(rows_p, args.gm_db, args.pm_deg))

    verdict = judge(
        cases, c_now=c_now, k_now=k_now, gm_db=args.gm_db, pm_deg=args.pm_deg,
        align_ratio=args.align_ratio,
    )
    print(f"\n### 3. 余裕を満たす最小の c（k ＝ {k_now:g}）・最小の k（c ＝ {c_now:g}）\n")
    body = []
    for case in cases:
        key = (case.frf.pedal, case.frf.v_kmh)
        body.append([
            _PEDAL_JA[key[0]], f"{key[1]:.0f}", _edge(verdict.c_min_by_case[key], C_GRID),
            _edge(verdict.k_min_by_case[key], K_GRID),
        ])
    print(md_table(["ペダル", "速度 [km/h]", "最小の c", "最小の k"], body))
    print("\n### 4. 全ケース（ペダル・速度）が合格する (c, k)\n")
    print(common_table(verdict.common_ok))
    print()
    common_text = _fmt(verdict.common_c_min, "g", "なし（この格子に無い）")
    print(f"- 共通の最小 c（k ＝ {k_now:g}）: {common_text}")
    for pedal, r in verdict.aligned_ratio.items():
        ratio_text = _fmt(r, ".2f", "算出不能（合格する c が無い速度あり）")
        print(f"- {_PEDAL_JA[pedal]}: 速度ごとの最小 c の最大 ÷ 最小 ＝ {ratio_text}"
              f"（そろう目安 {args.align_ratio:g} 以下）")
    print("- 判定: " + (
        "「係数 × 遅れ」で全速度・全ペダルにそろって書ける" if verdict.aligned
        and verdict.common_c_min is not None
        else "そろわない（G2: 止めて案を残す）"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
