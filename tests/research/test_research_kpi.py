"""kpi.py のばたつき指標（chatter_metrics 一式）のユニットテスト。

手順3（FF のみ・C5）の実機走行で見えた 1.4Hz 付近の小刻みな往復を測るために追加した
bandpass / direction_reversals / hold_durations_s / dominant_frequency / chatter_metrics を、
`test_research_stair_gain.py` と同じ流儀（合成データでの検算）で確かめる。
"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research import kpi
from tests.research.config import KpiSection


def test_bandpass_keeps_chatter_band_and_removes_low_frequency() -> None:
    """1.4Hz（帯の中）+ 0.1Hz（帯の外・大振幅）の合成波から 0.1Hz 成分が落ち、
    帯 RMS が 1.4Hz 成分の理論値 A/√2 に近くなること。"""
    dt = 0.05  # 20Hz（実機ログの周期と同じ）
    t = np.arange(0.0, 60.0, dt)
    amplitude = 0.3
    chatter = amplitude * np.sin(2 * np.pi * 1.4 * t)
    drift = 5.0 * np.sin(2 * np.pi * 0.1 * t)  # 帯の外・振幅が一桁大きい低周波
    signal = chatter + drift

    filtered = kpi.bandpass(signal, dt=dt)

    # 過渡応答が乗る前後の端を捨て、定常区間だけで RMS を見る
    steady = filtered[200:-200]
    rms = float(np.std(steady))
    expected_rms = amplitude / np.sqrt(2.0)
    assert rms == pytest.approx(expected_rms, rel=0.10)


def test_direction_reversals_counts_sign_changes_of_diff() -> None:
    assert kpi.direction_reversals([0, 1, 2, 1, 0, 1]) == 2


def test_direction_reversals_ignores_flat_steps() -> None:
    # 差分 0 の行（値が変わらない行）は符号を持たないので飛ばす
    assert kpi.direction_reversals([0, 1, 1, 0, 0, 1]) == 2


def test_hold_durations_s_on_constant_run() -> None:
    # 1,1,1,2,2,2,2,5 (dt=1, tol=0) → 保持区間は [1,1,1] [2,2,2,2] [5]
    # それぞれの経過時間は (n-1)*dt = 2, 3, 0
    durations = kpi.hold_durations_s([1, 1, 1, 2, 2, 2, 2, 5], dt=1.0, tol=0.0)
    assert durations == pytest.approx([2.0, 3.0, 0.0])


def test_hold_durations_s_respects_tolerance() -> None:
    # 保持値 0 から ±0.5 以内は「保てた」とみなす
    durations = kpi.hold_durations_s([0.0, 0.3, 0.5, 0.6, 1.2], dt=0.1, tol=0.5)
    # 0.0,0.3,0.5 は保持値 0.0 から ±0.5 以内（3 行 → (3-1)*0.1=0.2s）。
    # 0.6 で ±0.5 を超えて新しい保持値 0.6 になるが、次の 1.2 も ±0.5 を超えるので
    # 0.6 は 1 行だけの区間（(1-1)*0.1=0.0s）、続けて 1.2 も 1 行だけの区間（0.0s）
    assert durations == pytest.approx([0.2, 0.0, 0.0])


def test_dominant_frequency_of_pure_tone() -> None:
    dt = 0.05
    t = np.arange(0.0, 60.0, dt)
    signal = np.sin(2 * np.pi * 1.4 * t)

    freq = kpi.dominant_frequency(signal, dt=dt)

    assert freq == pytest.approx(1.4, abs=0.1)


def test_dominant_frequency_returns_zero_for_too_short_input() -> None:
    assert kpi.dominant_frequency([1.0], dt=0.05) == 0.0


def test_bandpass_and_lowpass_do_not_raise_on_short_input() -> None:
    # 零位相フィルタのパディング長を満たせないほど短い入力でも例外を投げず、ゼロ配列に落ちる
    short = [1.0, 2.0, 3.0]
    bp = kpi.bandpass(short, dt=0.05)
    lp = kpi.lowpass(short, dt=0.05)
    assert len(bp) == len(short)
    assert len(lp) == len(short)


# ─────────────────────────────────────────────────────────────────────
# chatter_metrics（合成した短い走行で主要フィールドを確かめる）
# ─────────────────────────────────────────────────────────────────────


def _synthetic_run(
    *, duration_s: float = 60.0, dt: float = 0.05, ref_kmh: float = 50.0,
    ripple_hz: float = 1.4, ripple_amp_kmh: float = 0.1,
) -> tuple[list[float], list[float], list[float], list[float], list[float], list[str]]:
    """基準車速一定・実車速に 1.4Hz のリップルを乗せた合成走行データ。

    アクセル指令は 0.5Hz でわずかに往復させ、方向反転・保持時間の検算に使う。
    ブレーキは使わない（全区間 ACCEL）。
    """
    n = int(round(duration_s / dt))
    t = [i * dt for i in range(n)]
    ref = [ref_kmh] * n
    actual = [ref_kmh + ripple_amp_kmh * np.sin(2 * np.pi * ripple_hz * ti) for ti in t]
    accel = [15.0 + 0.5 * np.sin(2 * np.pi * 0.5 * ti) for ti in t]
    brake = [0.0] * n
    phase = ["ACCEL"] * n
    return t, ref, actual, accel, brake, phase


def test_chatter_metrics_on_synthetic_ripple() -> None:
    t, ref, actual, accel, brake, phase = _synthetic_run()
    limits = KpiSection()

    m = kpi.chatter_metrics(t, ref, actual, accel, brake, phase, limits)

    # 実車速のリップル（振幅 0.1km/h の 1.4Hz 正弦波）の帯 RMS は理論値 0.1/√2 に近い
    assert m.speed_band_rms_kmh == pytest.approx(0.1 / np.sqrt(2.0), rel=0.15)
    # 基準車速は一定（帯の中の成分が無い）ので帯 RMS はほぼ 0
    assert m.ref_band_rms_kmh == pytest.approx(0.0, abs=1e-6)
    # 卓越周波数は 1.4Hz 付近
    assert m.dominant_hz == pytest.approx(1.4, abs=0.15)
    # 全行 ACCEL・全行 accel_pct > 0 なので継続時間は走行全体と一致
    assert m.accel_active_s == pytest.approx(60.0, abs=0.1)
    assert m.brake_active_s == pytest.approx(0.0, abs=1e-9)
    # アクセル指令は往復しているので反転回数・移動量は 0 より大きい
    assert m.accel_reversals_per_s > 0.0
    assert m.accel_travel_pct > 0.0
    # ブレーキは未使用
    assert m.brake_reversals_per_s == pytest.approx(0.0)
    assert m.brake_travel_pct == pytest.approx(0.0)
    # 保持時間は 0 以上（アクセルが往復しているので有限の値が出る）
    assert m.hold_median_s >= 0.0
    assert m.hold_p90_s >= m.hold_median_s
    # 偏差にはっきり 1.4Hz のリップルがあるので、ローパス後の符号反転は生より少ないか同じ
    assert m.reversal_smoothed <= m.reversal_raw
    assert m.p95_raw_kmh >= 0.0
    assert m.p95_smoothed_kmh >= 0.0


def test_chatter_metrics_falls_back_to_zero_for_too_few_rows() -> None:
    """行が少なすぎて窓が 1 つも取れない等のときは、例外を投げず 0.0 に落ちる。"""
    limits = KpiSection()
    m = kpi.chatter_metrics(
        [0.0], [50.0], [50.0], [15.0], [0.0], ["ACCEL"], limits,
    )
    assert m.dominant_hz == 0.0
    assert m.speed_band_rms_kmh == 0.0
    assert m.by_speed_band == {}
    assert m.reversal_raw == 0
    assert m.reversal_smoothed == 0


def test_speed_band_label_matches_mode_report_bounds() -> None:
    """速度帯の区切りは mode_report.py の 4.3 節の表と同じ（0/20/40/60/80/100/120）。"""
    assert kpi.speed_band_label(10.0) == "0〜20"
    assert kpi.speed_band_label(20.0) == "20〜40"
    assert kpi.speed_band_label(119.9) == "100〜120"
    assert kpi.speed_band_label(150.0) == "120〜"


# ── アクセル指令の往復回数（ProblemReport_20260921 手順3） ─────────────


def test_pedal_reversals_ignores_monotonic_and_small_ripple() -> None:
    assert kpi.pedal_reversals([10.0, 11.0, 12.0, 15.0], hyst=0.1) == 0  # 一方向の踏み増し
    assert kpi.pedal_reversals([15.0, 12.0, 10.0], hyst=0.1) == 0  # 一方向の戻し
    # 0.05% の上下は hyst 0.1% 未満なので数えない
    assert kpi.pedal_reversals([11.0, 11.05, 11.0, 11.05, 11.0], hyst=0.1) == 0


def test_pedal_reversals_counts_turns_beyond_hysteresis() -> None:
    # 山 11.6 から 11.4（0.2% 戻り）で 1 回、谷 10.9 から 11.5 で 2 回
    assert kpi.pedal_reversals([11.0, 11.6, 11.4, 10.9, 11.5], hyst=0.1) == 2
    # 0.73 秒周期のばたつき（10.92 ↔ 11.60）は山と谷で 1 周期 2 回
    zigzag = [10.92, 11.60] * 5
    assert kpi.pedal_reversals(zigzag, hyst=0.1) == len(zigzag) - 2


def test_pedal_reversal_rates_overall_and_window_max() -> None:
    dt = 0.1
    n = 1200  # 120s
    t = [i * dt for i in range(n)]
    ref = [50.0] * n
    phase = ["ACCEL"] * n
    # 前半 60s は一定、後半 60s は 1 秒周期で ±0.5% 往復（1 秒に 2 回）
    accel = [15.0] * 600 + [15.0 + (0.5 if (i // 5) % 2 == 0 else -0.5) for i in range(600)]
    overall, window_max = kpi.pedal_reversal_rates(
        t, accel, ref, phase, hyst_pct=0.1, window_s=60.0, min_window_active_s=15.0
    )
    assert window_max == pytest.approx(2.0, abs=0.1)  # 後半の窓
    assert overall == pytest.approx(1.0, abs=0.1)  # 全体では半分の時間だけ揺れている


def test_pedal_reversal_rates_skip_brake_slow_and_short_windows() -> None:
    dt = 0.1
    n = 1000
    t = [i * dt for i in range(n)]
    accel = [15.0 + (0.5 if i % 10 < 5 else -0.5) for i in range(n)]
    # ブレーキ相・基準 5km/h 未満の行は対象外 → 対象が無ければ (0, 0)
    assert kpi.pedal_reversal_rates(
        t, accel, [50.0] * n, ["BRAKE"] * n,
        hyst_pct=0.1, window_s=60.0, min_window_active_s=15.0,
    ) == (0.0, 0.0)
    assert kpi.pedal_reversal_rates(
        t, accel, [2.0] * n, ["ACCEL"] * n,
        hyst_pct=0.1, window_s=60.0, min_window_active_s=15.0,
    ) == (0.0, 0.0)
    # 100s 走行のうち 2 窓目は 40s（≥15s）で残る。min を 50s にすると 2 窓目は除かれ 1 窓目のみ
    _, w_all = kpi.pedal_reversal_rates(
        t, accel, [50.0] * n, ["ACCEL"] * n,
        hyst_pct=0.1, window_s=60.0, min_window_active_s=15.0,
    )
    _, w_long = kpi.pedal_reversal_rates(
        t, accel, [50.0] * n, ["ACCEL"] * n,
        hyst_pct=0.1, window_s=60.0, min_window_active_s=50.0,
    )
    assert w_all >= w_long > 0.0


def test_chatter_metrics_reports_pedal_reversal_rates() -> None:
    t, ref, actual, accel, brake, phase = _synthetic_run()
    m = kpi.chatter_metrics(t, ref, actual, accel, brake, phase, KpiSection())
    # 合成走行のアクセル指令は 0.5Hz・振幅 0.5% の正弦波 → 1 秒に約 1 回向きが変わる
    assert m.pedal_reversal_per_s == pytest.approx(1.0, abs=0.15)
    assert m.pedal_reversal_window_max_per_s == pytest.approx(1.0, abs=0.15)
