"""P＋DOB のループの余裕（`tests/research/loop_margin.py`）のユニットテスト。

検算の方針: 純積分器（スタブ車両に近い。メモリ stub-vehicle-is-a-different-plant）は手計算で
答えが出るので、(1) P だけの交差周波数・位相余裕・ゲイン余裕、(2) 表が車を完全に表すとき DOB の項
G が 0 になること、(3) 表が車の半分の感度のときの G の閉じた式、(4) 前の車の実測で C5 の
ばたつき（約 1〜1.4 Hz）が説明できること、を確かめる。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tests.research import loop_margin as lm
from tests.research.excite import BlockResponse

S_STUB = 0.25  # スタブ車両のアクセルゲイン [km/h/s per %]
LAG = 0.23


def _frf_from_plant(freqs: np.ndarray, plant, *, pedal: str = "accel", v: float = 60.0):
    p = plant(2j * np.pi * freqs)
    return lm.PlantFrf(
        pedal, v, freqs, np.abs(p), np.degrees(np.unwrap(np.angle(p))), np.full(len(freqs), 10.0)
    )


def _integrator_with_delay(s: np.ndarray) -> np.ndarray:
    return S_STUB * np.exp(-s * LAG) / s


def test_p_only_margins_match_hand_calculation_for_pure_integrator() -> None:
    """P ＝ S e^{-sL}/s、K ＝ 1/(τS) → L ＝ e^{-sL}/(sτ)。

    交差 ωc ＝ 1/τ、位相余裕 ＝ 90° − ωc・L(rad→°)、位相交差 ωπ ＝ π/(2L)、
    ゲイン余裕 ＝ 20log10(ωπ τ)。
    """
    freqs = np.geomspace(0.01, 2.0, 40)
    frf = _frf_from_plant(freqs, _integrator_with_delay)
    c = 25.0
    tau = c * LAG
    op = lm.OperatingPoint("accel", 60.0, 10.0, S_STUB, 0.0, LAG)
    case = lm.LoopCase(frf, op, 0.05)
    m = case.margins_for(c, None)

    wc = 1.0 / tau
    assert m.crossover_hz == pytest.approx(wc / (2 * math.pi), rel=0.02)
    assert m.pm_deg == pytest.approx(90.0 - math.degrees(wc * LAG), abs=0.5)
    w_pi = math.pi / (2.0 * LAG)
    assert m.phase_cross_hz == pytest.approx(w_pi / (2 * math.pi), rel=0.02)
    assert m.gm_db == pytest.approx(20.0 * math.log10(w_pi * tau), abs=0.3)
    assert m.ok(6.0, 45.0)


def test_gain_margin_falls_6db_when_c_is_halved() -> None:
    """K ∝ 1/c なので c を半分にするとゲイン余裕がちょうど 6.02 dB 減る。"""
    freqs = np.geomspace(0.01, 2.0, 40)
    case = lm.LoopCase(
        _frf_from_plant(freqs, _integrator_with_delay),
        lm.OperatingPoint("accel", 60.0, 10.0, S_STUB, 0.0, LAG), 0.05,
    )
    g1 = case.margins_for(25.0, None).gm_db
    g2 = case.margins_for(12.5, None).gm_db
    assert g1 is not None and g2 is not None
    assert g1 - g2 == pytest.approx(20.0 * math.log10(2.0), abs=0.2)


def test_dob_term_is_zero_when_table_matches_the_plant() -> None:
    """表が車を完全に表す（P ＝ S e^{-sL}/(s − a_v)）なら G ＝ 0（T → 0 のとき）。"""
    a_v = -0.1
    f = np.geomspace(0.02, 2.0, 50)
    s = 2j * np.pi * f
    p = S_STUB * np.exp(-s * LAG) / (s - a_v)
    _loop, l_p, g = lm.loop_terms(
        f, p, sens=S_STUB, a_v=a_v, lag_s=LAG, tau_s=5.0, tq_s=3.0, dt_s=1e-7
    )
    assert np.max(np.abs(g)) < 1e-4
    assert np.allclose(_loop, l_p, atol=1e-4)


def test_dob_term_closed_form_when_table_sensitivity_is_half_the_plant() -> None:
    """P ＝ 2S/s（遅れなし）、表は S → G(s) ＝ (s・2S/s − S)/(S(1 + sTq)) ＝ 1/(1 + sTq)。"""
    f = np.geomspace(0.02, 2.0, 50)
    s = 2j * np.pi * f
    p = 2.0 * S_STUB / s
    tq = 4.0
    loop, l_p, g = lm.loop_terms(
        f, p, sens=S_STUB, a_v=0.0, lag_s=0.0, tau_s=5.0, tq_s=tq, dt_s=1e-7
    )
    assert np.allclose(g, 1.0 / (1.0 + s * tq), atol=1e-4)
    assert np.allclose(loop, l_p + g)
    assert np.allclose(l_p, (1.0 / (5.0 * S_STUB)) * p)


def test_build_frfs_unwraps_phase_through_minus_180() -> None:
    """analyze の生の位相（+164.5° など）を −195.5° のように連続に直す（2026-09-24 の表記）。"""

    def blk(f: float, g: float, ph: float) -> BlockResponse:
        return BlockResponse(60.0, f, 100, 0.3, 0.3 * g, g, ph, 10.0)

    frfs = lm.build_frfs([blk(1.0, 0.892, -172.4), blk(1.4, 0.674, 164.5), blk(0.5, 1.62, -141.2),
                          blk(2.0, 0.582, 128.1)])
    frf = frfs[("accel", 60.0)]
    assert list(frf.freqs_hz) == [0.5, 1.0, 1.4, 2.0]
    assert frf.phase_deg == pytest.approx([-141.2, -172.4, -195.5, -231.9], abs=0.1)


def test_margins_report_no_crossover_when_loop_gain_is_below_one() -> None:
    f = np.geomspace(0.05, 2.0, 100)
    loop = 0.3 * np.exp(-1j * 2 * np.pi * f * 0.2) / (1j * f)  # |L| = 0.3/f > 1 for f<0.3
    m = lm.margins(f, 0.05 * loop)  # 0.05×0.3/f < 1 全域
    assert m.crossover_hz is None and m.pm_deg is None
    assert "交差なし" in m.note


def test_past_measurement_explains_the_c5_chatter_near_1_to_1_4_hz() -> None:
    """2026-09-24 の実測 P(f) × C5 の実質 Kp（0.97〜1.44）でばたつきを説明できる。

    - 位相交差（−180°）は 60 km/h で約 1.1 Hz、100 km/h で約 1.2 Hz（報告 2-9-2 の補間と同じ）
    - Kp ＝ 0.97〜1.44 ではゲイン余裕が 6 dB を大きく割り、感度の山が 1.0〜1.2 Hz に立つ
    - 純粋な P 制御 K ＝ 0.3 の最大増幅率は 60 km/h・0.5 Hz で約 1.45、100 km/h で約 1.32
      （報告の訂正注記）
    """
    m60 = lm.margins_for_p_only(lm.past_frf(60.0), 0.30)
    m100 = lm.margins_for_p_only(lm.past_frf(100.0), 0.30)
    assert m60.phase_cross_hz == pytest.approx(1.13, abs=0.03)
    assert m100.phase_cross_hz == pytest.approx(1.22, abs=0.03)
    assert m60.ms == pytest.approx(1.45, abs=0.05) and m60.ms_hz == pytest.approx(0.5, abs=0.05)
    assert m100.ms == pytest.approx(1.32, abs=0.05)
    assert m60.ok(6.0, 45.0) and m100.ok(6.0, 45.0)

    for v in (60.0, 100.0):
        for k in (0.97, 1.08, 1.44):
            m = lm.margins_for_p_only(lm.past_frf(v), k)
            assert m.gm_db is not None and m.gm_db < 6.0  # C5 は余裕の目安を満たさない
            assert 0.95 <= m.ms_hz <= 1.45  # 感度の山 ＝ ばたつきの周波数帯
            assert not m.ok(6.0, 45.0)
    # 100 km/h・Kp 1.44 はほぼ限界（ゲイン余裕 0 dB 付近）
    assert abs(lm.margins_for_p_only(lm.past_frf(100.0), 1.44).gm_db) < 0.5


class _LinearTable:
    """a(v, u) ＝ a0 ＋ S・u ＋ a_v・(v − 60) の線形な偽の表（感度の取り出しの検算用）。"""

    def __init__(self, sens: float, a_v: float, a0: float = -0.6) -> None:
        self.sens, self.a_v, self.a0 = sens, a_v, a0

    def accel(self, v, u):
        return self.a0 + self.sens * u + self.a_v * (v - 60.0)

    def coast_accel(self, v):
        return self.accel(v, 0.0)

    def invert(self, v, a):
        return ((a - self.a0 - self.a_v * (v - 60.0)) / self.sens, 0.0)


@pytest.mark.parametrize("pedal", ["accel", "brake"])
def test_operating_point_recovers_slope_of_the_table(pedal: str) -> None:
    table = _LinearTable(0.09, -0.004)
    op = lm.operating_point(table, pedal, 60.0, 0.2, brake_extra_kmhs=0.5)
    assert op.sens == pytest.approx(0.09)
    assert op.a_v == pytest.approx(-0.004)
    assert (op.u_pct > 0.0) if pedal == "accel" else (op.u_pct < 0.0)


def test_judge_flags_speeds_that_need_very_different_c() -> None:
    """速度ごとに必要な c が 2 倍超ばらつくと「そろわない」。そろえば共通の最小 c を返す。"""
    freqs = np.geomspace(0.01, 2.0, 40)

    def case_with_lag(v: float, extra_lag: float) -> lm.LoopCase:
        def plant(s):
            return S_STUB * np.exp(-s * (LAG + extra_lag)) / s

        return lm.LoopCase(
            _frf_from_plant(freqs, plant, v=v),
            lm.OperatingPoint("accel", v, 10.0, S_STUB, 0.0, LAG), 0.05,
        )

    same = [case_with_lag(30.0, 0.0), case_with_lag(60.0, 0.0), case_with_lag(90.0, 0.0)]
    v_same = lm.judge(same, c_now=25.0, k_now=40.0, gm_db=6.0, pm_deg=45.0, align_ratio=2.0)
    assert v_same.aligned and v_same.common_c_min is not None

    diff = [case_with_lag(30.0, 0.0), case_with_lag(60.0, 0.0), case_with_lag(90.0, 1.2)]
    v_diff = lm.judge(diff, c_now=25.0, k_now=40.0, gm_db=6.0, pm_deg=45.0, align_ratio=2.0)
    assert v_diff.aligned_ratio["accel"] is None or v_diff.aligned_ratio["accel"] > 2.0
    assert not v_diff.aligned
