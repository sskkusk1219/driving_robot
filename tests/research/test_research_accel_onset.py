"""accel_onset（段1: アクセル不感帯直上の欠陥切り分け）のユニットテスト。

`test_research_stop_brake_floor.py` / `test_research_cruise_curve.py` と同じ流儀で、
合成の CSV 行（dict 列。値はすべて文字列。csv.DictReader が返すのと同じ形）を組み立てて確かめる。

合成ログの作り方（`_build_hinge_rows`）:
    `FeedforwardParams` の惰行減速カーブを 2 点とも 0.0 にし、`creep_speed_kmh=0.0` にして
    常に惰行側分岐を使わせることで `free_accel_at` を全域で 0 にする（テストの簡略化。
    これにより a_eff = a_obs になり、狙った `a = k·max(0, x − x0)` をそのまま検算できる）。
    アクセル開度を階段状（プラトー）に固定し、各プラトー内で目標加速度 `a` を一定にして
    前進オイラー（`v[i+1] = v[i] + a·dt`）で車速を積分する。加速度が区間内で定数なので、
    プラトー内部（境界から離れた点）では中心差分が厳密に `a` を復元する。
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
import pytest

from src.models.profile import FeedforwardParams
from tests.research import accel_onset as ao
from tests.research.drive_log import SECTION_MODE_DRIVE, SECTION_PATTERN_DRIVE
from tests.research.ff_params import ResearchFFParams

# 表2/表4 の既定と同じ開度オフセット帯（テストでもそのまま使う）
OFFSET_BINS: tuple[float, ...] = (0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 30.0)

# free_accel_at を全域で 0 にするための研究側パラメータ（未同定のまま。クリープ分岐を使わない
# 設定にしているので creep_accel_speeds_kmh 等は参照されない）
RESEARCH = ResearchFFParams()


def _params(deadband: float = 1.0, brake_deadband: float = 10.0) -> FeedforwardParams:
    return FeedforwardParams(
        accel_deadband_pct=deadband,
        brake_deadband_pct=brake_deadband,
        creep_speed_kmh=0.0,  # v>=0 は常に惰行側分岐（coast_decel_at）を使う
        coast_decel_speeds_kmh=(0.0, 200.0),
        coast_decel_kmhs=(0.0, 0.0),  # 惰行減速量を全域で 0 に固定 → free_accel_at 常に 0
    )


def _mode_row(t: float, v: float, accel: float, *, brake: float = 0.0) -> dict[str, str]:
    return {
        "section": SECTION_MODE_DRIVE,
        "mode_time_s": f"{t:.4f}",
        "actual_speed_kmh": f"{v:.4f}",
        "accel_actual_pct": f"{accel:.4f}",
        "brake_actual_pct": f"{brake:.4f}",
        "accel_cmd_pct": f"{accel:.4f}",
        "deviation_kmh": "0.0",
    }


def _build_hinge_rows(
    k: float,
    x0: float,
    offsets: Sequence[float],
    *,
    dt: float = 0.05,
    plateau_s: float = 2.0,
    v0: float = 10.0,
    deadband: float = 1.0,
) -> list[dict[str, str]]:
    """`a = k·max(0, x − x0)` を仕込んだ、開度が段々に一定なプラトー列の合成ログを作る。"""
    rows: list[dict[str, str]] = []
    t = 0.0
    v = v0
    n_per_plateau = max(1, round(plateau_s / dt))
    for offset in offsets:
        accel_pct = deadband + offset
        a = k * max(0.0, offset - x0)
        for _ in range(n_per_plateau):
            rows.append(_mode_row(t, v, accel_pct))
            v += a * dt
            t += dt
    return rows


def _fit_from_rows(
    rows: list[dict[str, str]],
    *,
    deadband: float = 1.0,
    steady_window_s: float = 0.45,
    steady_tol_pct: float = 0.3,
    accel_half_window_s: float = 0.30,
    min_bin_samples: int = 5,
    bootstrap_n: int = 0,
) -> ao.BandFit:
    """合成ログ 1 本を解析し、単一速度帯（0〜1000 km/h）でフィットする（テスト用の近道）。"""
    analysis = ao.analyze_rows(
        rows, _params(deadband), RESEARCH, source="synthetic", deadband_pct=deadband,
        steady_window_s=steady_window_s, steady_tol_pct=steady_tol_pct,
        accel_half_window_s=accel_half_window_s, min_speed_kmh=0.0, direction_lookback_s=1.45,
    )
    x = analysis.x_pct[analysis.sample_mask]
    a = analysis.a_eff[analysis.sample_mask]
    return ao.fit_band(
        x, a, speed_lo=0.0, speed_hi=1000.0, offset_bins=OFFSET_BINS,
        min_bin_samples=min_bin_samples, bootstrap_n=bootstrap_n, model_gain=None,
        rng=np.random.default_rng(0),
    )


# ─────────────────────────────────────────────────────────────────────
# x0 の復元（折れ線当てはめ）
# ─────────────────────────────────────────────────────────────────────


def test_hinge_seeded_log_recovers_x0_within_tolerance() -> None:
    """`a = k·max(0, x − x0)` を仕込んだ合成ログから x0 が ±0.1% 以内で戻る。"""
    true_k, true_x0 = 2.0, 0.8
    offsets = [0.1, 0.6, 1.2, 1.8, 2.5, 3.5, 5.0, 7.0]
    rows = _build_hinge_rows(true_k, true_x0, offsets, dt=0.05, plateau_s=2.0)

    fit = _fit_from_rows(rows)

    assert fit.identified
    assert fit.n_bins >= 3
    assert fit.x0_pct == pytest.approx(true_x0, abs=0.1)
    assert fit.k == pytest.approx(true_k, rel=0.05)


def test_dt_005_and_dt_01_logs_pooled_give_consistent_x0() -> None:
    """訂正1 の回帰テスト: dt=0.05s と dt=0.1s の CSV を混ぜても同じ x0 に戻る。"""
    true_k, true_x0 = 2.0, 0.8
    offsets = [0.1, 0.6, 1.2, 1.8, 2.5, 3.5, 5.0, 7.0]
    rows_005 = _build_hinge_rows(true_k, true_x0, offsets, dt=0.05, plateau_s=2.0)
    rows_010 = _build_hinge_rows(true_k, true_x0, offsets, dt=0.10, plateau_s=2.0)

    kwargs = dict(
        deadband_pct=1.0, steady_window_s=0.45, steady_tol_pct=0.3, accel_half_window_s=0.30,
        min_speed_kmh=0.0, direction_lookback_s=1.45,
    )
    an_005 = ao.analyze_rows(rows_005, _params(), RESEARCH, source="dt005", **kwargs)
    an_010 = ao.analyze_rows(rows_010, _params(), RESEARCH, source="dt010", **kwargs)

    assert an_005.series.dt_s == pytest.approx(0.05, abs=1e-6)
    assert an_010.series.dt_s == pytest.approx(0.10, abs=1e-6)

    _v, x, a, _dir = ao._pool_samples([an_005, an_010])
    fit = ao.fit_band(
        x, a, speed_lo=0.0, speed_hi=1000.0, offset_bins=OFFSET_BINS, min_bin_samples=5,
        bootstrap_n=0, model_gain=None, rng=np.random.default_rng(0),
    )

    assert fit.identified
    assert fit.x0_pct == pytest.approx(true_x0, abs=0.15)


def test_quantized_speed_does_not_break_x0() -> None:
    """訂正2 の回帰テスト: 実車速を 14Hz 相当に量子化（約3割を前回値）しても x0 が壊れない。"""
    true_k, true_x0 = 2.0, 0.8
    offsets = [0.1, 0.6, 1.2, 1.8, 2.5, 3.5, 5.0, 7.0]
    rows = _build_hinge_rows(true_k, true_x0, offsets, dt=0.05, plateau_s=2.0)

    # 隣接差分の約 3 割をゼロにする（CAN 更新が約14Hzで、20Hz サンプリングに対して
    # 一部の行が前回値のまま読まれることを模す）
    held = 0
    for i in range(1, len(rows)):
        if i % 10 < 3:
            rows[i]["actual_speed_kmh"] = rows[i - 1]["actual_speed_kmh"]
            held += 1
    assert held / (len(rows) - 1) == pytest.approx(0.3, abs=0.01)

    fit = _fit_from_rows(rows)

    assert fit.identified
    assert fit.x0_pct == pytest.approx(true_x0, abs=0.3)


def test_insufficient_offset_diversity_is_not_identified() -> None:
    """ビン数が最低 3 に満たなければ当てはめず `identified=False` になる。"""
    rows = _build_hinge_rows(2.0, 0.8, [0.5, 1.5], dt=0.05, plateau_s=2.0)

    fit = _fit_from_rows(rows)

    assert not fit.identified
    assert fit.x0_pct is None
    assert fit.judgement == "不定(ビン不足)"


def test_min_bin_samples_threshold_controls_identification() -> None:
    """同じログでも `--min-bin-samples` を上げるとビンが間引かれ `identified=False` になる。"""
    rows = _build_hinge_rows(
        2.0, 0.8, [0.1, 0.6, 1.2, 1.8, 2.5, 3.5, 5.0, 7.0], dt=0.05, plateau_s=0.6,
    )

    permissive = _fit_from_rows(
        rows, steady_window_s=0.1, accel_half_window_s=0.1, min_bin_samples=2,
    )
    strict = _fit_from_rows(
        rows, steady_window_s=0.1, accel_half_window_s=0.1, min_bin_samples=100,
    )

    assert permissive.identified
    assert not strict.identified


# ─────────────────────────────────────────────────────────────────────
# 3次カーブとの区別（偽陽性を出さない）
# ─────────────────────────────────────────────────────────────────────


def test_cubic_shaped_data_makes_cubic_fit_win() -> None:
    """折れ線では表せない滑らかな3次カーブを仕込むと、3次の R² が折れ線よりはっきり勝つ。"""
    x = np.linspace(0.2, 8.0, 14)
    y = 0.02 * x**3 - 0.25 * x**2 + 1.3 * x
    x0_candidates = np.arange(0.0, 8.0, 0.01)

    fit = ao._hinge_fit_no_intercept(x, y, x0_candidates)
    assert fit is not None
    _rss, x0, k = fit
    r2_line = ao._r2(y, k * np.maximum(0.0, x - x0))
    r2_cubic = ao._cubic_r2(x, y)

    assert r2_cubic is not None
    assert r2_cubic - r2_line > ao.CUBIC_GAIN_THRESHOLD


def test_pure_hinge_data_does_not_make_cubic_win() -> None:
    """純粋な折れ線データでは 3次が折れ線を有意に上回らない（偽陽性を出さない）。"""
    x = np.array([0.1, 0.4, 0.6, 0.9, 1.2, 1.8, 2.5, 3.5, 5.0, 7.0])
    true_k, true_x0 = 2.0, 0.8
    y = true_k * np.maximum(0.0, x - true_x0)
    x0_candidates = np.arange(0.0, 7.0, 0.01)

    fit = ao._hinge_fit_no_intercept(x, y, x0_candidates)
    assert fit is not None
    _rss, x0, k = fit
    r2_line = ao._r2(y, k * np.maximum(0.0, x - x0))
    r2_cubic = ao._cubic_r2(x, y)

    assert r2_cubic is not None
    assert r2_cubic - r2_line <= ao.CUBIC_GAIN_THRESHOLD


# ─────────────────────────────────────────────────────────────────────
# 表5: 応答遅れ（lag 検出）
# ─────────────────────────────────────────────────────────────────────


def test_diff_based_peak_lag_recovers_known_delay_despite_raw_autocorrelation() -> None:
    """欠陥1 の回帰テスト。

    実開度のような「ゆっくり動く」生信号は強い自己相関を持ち、生信号のまま相関を取ると
    どのラグでもほぼ同じ高い相関になって遅れを判別できない。1 階差分どうしを相関させると、
    仕込んだ既知の遅れ（0.3s）でピークが立つことを確かめる。
    """
    dt = 0.05
    lag_s = 0.3
    t = np.arange(0.0, 20.0, dt)

    # x・y は実開度のように単調に増え続ける「ゆっくり動く」トレンドに、小さな矩形パルス
    # （本当の遅れ情報を持つ変化）を重ねる。トレンドの分散がパルスよりずっと大きいので、
    # 生信号のままだとどのラグでもトレンドの強い自己相関に支配される。
    trend = t.copy()
    pulse = np.where((t > 8.0) & (t < 12.0), 0.3, 0.0)
    x = trend + pulse

    t_shifted = t - lag_s
    pulse_shifted = np.where((t_shifted > 8.0) & (t_shifted < 12.0), 0.3, 0.0)
    y = t_shifted + pulse_shifted  # x を lag_s だけ遅らせてなぞった信号

    # 欠陥1 の再現: 生信号のままだと、どのラグでも強い相関が出て遅れを判別できない
    raw_corrs = [ao._lagged_corr(x, y, lag) for lag in (0, round(lag_s / dt), round(1.0 / dt))]
    assert all(c is not None and c > 0.99 for c in raw_corrs)

    # 修正後: 1 階差分どうしなら、仕込んだ遅れでピークが立つ
    peak = ao._peak_lag(np.diff(x), np.diff(y), dt, max_lag_s=1.0)

    assert peak is not None
    lag_found, corr = peak
    assert lag_found == pytest.approx(lag_s, abs=dt)
    assert corr > 0.9


# ─────────────────────────────────────────────────────────────────────
# --section（段3-1。2026-09-19。ProblemReport_20260919 候補(c) の検証用）
# ─────────────────────────────────────────────────────────────────────


def _pattern_drive_row(t: float, v: float, accel: float, *, brake: float = 0.0) -> dict[str, str]:
    """PATTERN_DRIVE の行（mode_time_s は持たず elapsed_s だけを持つ）。"""
    return {
        "section": SECTION_PATTERN_DRIVE,
        "elapsed_s": f"{t:.4f}",
        "mode_time_s": "",
        "actual_speed_kmh": f"{v:.4f}",
        "accel_actual_pct": f"{accel:.4f}",
        "brake_actual_pct": f"{brake:.4f}",
        "accel_cmd_pct": f"{accel:.4f}",
        "deviation_kmh": "",
    }


def test_series_from_rows_reads_pattern_drive_section_with_elapsed_s() -> None:
    """`section=PATTERN_DRIVE` は mode_time_s が空でも elapsed_s にフォールバックして読める。

    既定（section 省略 = MODE_DRIVE）の挙動は変わらないことも合わせて確認する
    （PATTERN_DRIVE の行が混ざっていても、既定呼び出しは従来どおり MODE_DRIVE の行だけを拾う）。
    """
    mode_rows = [_mode_row(t, 10.0 + t, 5.0) for t in (0.0, 0.05, 0.1)]
    pattern_rows = [_pattern_drive_row(t, 2.0 + t, 6.8) for t in (0.0, 0.1, 0.2, 0.3)]
    rows = mode_rows + pattern_rows

    series = ao.series_from_rows(rows, source="synthetic", section=SECTION_PATTERN_DRIVE)
    assert series.n_mode_rows == len(pattern_rows)
    assert series.t_s.tolist() == pytest.approx([0.0, 0.1, 0.2, 0.3])
    assert series.v_kmh.tolist() == pytest.approx([2.0, 2.1, 2.2, 2.3])
    assert series.accel_pct.tolist() == pytest.approx([6.8, 6.8, 6.8, 6.8])

    default_series = ao.series_from_rows(rows, source="synthetic")  # 既定は MODE_DRIVE のまま
    assert default_series.n_mode_rows == len(mode_rows)


# ─────────────────────────────────────────────────────────────────────
# 表3b: 低開度階段の上り／下り（段3-2。ProblemReport_20260919 6章の往復の層別）
# ─────────────────────────────────────────────────────────────────────


def test_stair_leg_labels_splits_up_and_down() -> None:
    """山型（1,1,2,2,3,3,2,2,1,1。5段）の CRUISE_TRIM 指令開度: 頂点までが「上り」、
    頂点より後が「下り」になる。"""
    pattern = np.array(["P"] * 10)
    phase = np.array([ao.STAIR_PHASE] * 10)
    accel_cmd = np.array([1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 2.0, 2.0, 1.0, 1.0])

    labels = ao._stair_leg_labels(pattern, phase, accel_cmd)

    assert list(labels[:6]) == ["上り"] * 6
    assert list(labels[6:]) == ["下り"] * 4


def test_stair_leg_labels_ignores_monotonic_pattern() -> None:
    """折り返しの無い単調増の区間（山の頂点が最後の段）はすべて "" になる。"""
    pattern = np.array(["P"] * 6)
    phase = np.array([ao.STAIR_PHASE] * 6)
    accel_cmd = np.array([1.0, 1.0, 2.0, 2.0, 3.0, 3.0])

    labels = ao._stair_leg_labels(pattern, phase, accel_cmd)

    assert list(labels) == [""] * 6


def test_stair_leg_labels_separates_patterns() -> None:
    """`pattern` が変われば別区間: 一方が単調（無効）でも他方の山型判定に影響しない。"""
    pattern = np.array(["A"] * 4 + ["B"] * 10)
    phase = np.array([ao.STAIR_PHASE] * 14)
    accel_cmd = np.array(
        [1.0, 2.0, 3.0, 4.0]  # A: 単調増、折り返し無し → 無効
        + [1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 2.0, 2.0, 1.0, 1.0]  # B: 5段の山型 → 有効
    )

    labels = ao._stair_leg_labels(pattern, phase, accel_cmd)

    assert list(labels[:4]) == [""] * 4
    assert list(labels[4:10]) == ["上り"] * 6
    assert list(labels[10:14]) == ["下り"] * 4


def test_stair_leg_labels_ignores_non_cruise_trim_phase() -> None:
    """階段判定は `phase == STAIR_PHASE`（CRUISE_TRIM）の行だけが対象。

    加速掃引（DRIVE_ACCEL）やブレーキ保持（DRIVE_BRAKE）も「踏む→離す」で指令開度が
    山型になるが、phase が違うため誤って階段扱いされない（実ログでの誤判定の回帰テスト）。
    """
    stair_shape = [1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 2.0, 2.0, 1.0, 1.0]  # 5段の山型（形は有効）
    pattern = np.array(["P1"] * 10 + ["P2"] * 10)
    phase = np.array(["DRIVE_ACCEL"] * 10 + ["DRIVE_BRAKE"] * 10)
    accel_cmd = np.array(stair_shape + stair_shape)

    labels = ao._stair_leg_labels(pattern, phase, accel_cmd)

    assert list(labels) == [""] * 20


def test_stair_leg_labels_requires_five_steps() -> None:
    """phase が CRUISE_TRIM でも段が 4 段以下の山型は "" になる（m>=5 が必要）。"""
    pattern = np.array(["P"] * 4)
    phase = np.array([ao.STAIR_PHASE] * 4)
    accel_cmd = np.array([1.0, 2.0, 3.0, 2.0])  # 4段の山型（折り返しはあるが段数不足）

    labels = ao._stair_leg_labels(pattern, phase, accel_cmd)

    assert list(labels) == [""] * 4


def test_build_table3b_reports_up_minus_down() -> None:
    """上り・下り両方があるセルだけ「差(上り-下り)」が出て、片方しか無いセルは行が出ない。"""
    speed_bins = (0.0, 100.0)
    offset_bins = (0.0, 1.0, 2.0)

    v = np.array([10.0, 10.0, 10.0, 10.0, 10.0])
    x = np.array([0.5, 0.5, 0.5, 0.5, 1.5])
    a = np.array([1.0, 1.2, 0.6, 0.8, 2.0])
    leg = np.array(["上り", "上り", "下り", "下り", "上り"])

    header, rows = ao.build_table3b(v, x, a, leg, speed_bins, offset_bins)

    assert header == [
        "速度帯 [km/h]", "開度帯 [%]", "上り n", "上り a_eff", "下り n", "下り a_eff",
        "差(上り-下り)",
    ]
    # 開度帯 0〜1 は上り(n=2, median=1.1)・下り(n=2, median=0.7) の両方あり → 出る
    # 開度帯 1〜2 は上りしか無い（n=1）→ 行ごと省かれる
    assert rows == [
        ["0〜100", "0〜1", "2", "+1.10", "2", "+0.70", "+0.40"],
    ]
