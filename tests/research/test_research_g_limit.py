"""g_limit（上限 G に届く開度の予測マップ）のテスト。"""

from __future__ import annotations

import pytest

from tests.research.g_limit import PEDAL_ACCEL, PEDAL_BRAKE, GLimitMap, _lsq_slope

CAP = 0.3 * 35.316  # 0.3G [km/h/s]


def test_no_data_returns_none() -> None:
    assert GLimitMap().cap_pct(PEDAL_BRAKE, 30.0, CAP, 8.0) is None


def test_one_point_without_coast_returns_none() -> None:
    m = GLimitMap()
    m.add(PEDAL_BRAKE, 25.0, 14.0, 7.0)
    assert m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0) is None


def test_brake_extrapolates_secant_from_top_point_and_lowest_opening_point() -> None:
    m = GLimitMap()
    m.add(PEDAL_BRAKE, 25.0, 12.0, 4.0)  # 中間の点は傾きに使わない（近い点の割線は雑音）
    m.add(PEDAL_BRAKE, 25.0, 14.0, 7.0)
    m.add(PEDAL_BRAKE, 25.0, 9.0, 1.0)
    cap = m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0)
    assert cap == pytest.approx(14.0 + (CAP - 7.0) / 1.2)  # (9,1)→(14,7): 1.2 km/h/s per %


def test_no_extrapolation_returns_the_opening_of_the_strongest_point() -> None:
    """2026-10-08 段2l-6: extrapolate=False は、狙いが貯めた点のどれより強いとき割線の外挿を
    せず、いちばん効いた点の開度を返す。"""
    m = GLimitMap()
    m.add(PEDAL_BRAKE, 25.0, 12.0, 4.0)
    m.add(PEDAL_BRAKE, 25.0, 14.0, 7.0)  # いちばん効いた点
    m.add(PEDAL_BRAKE, 25.0, 9.0, 1.0)
    assert CAP > 7.0
    assert m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0, extrapolate=False) == pytest.approx(14.0)
    # 既定（外挿あり）は従来どおり。他の呼び出し元の挙動は変わらない
    assert m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0) == pytest.approx(14.0 + (CAP - 7.0) / 1.2)


def test_no_extrapolation_leaves_interpolation_and_weaker_targets_unchanged() -> None:
    m = GLimitMap()
    m.add(PEDAL_BRAKE, 25.0, 12.0, 4.0)
    m.add(PEDAL_BRAKE, 25.0, 20.0, 9.0)
    for target in (5.0, 6.5, 8.0):  # 点の範囲内: 補間
        assert m.cap_pct(PEDAL_BRAKE, 25.0, target, 8.0, extrapolate=False) == pytest.approx(
            m.cap_pct(PEDAL_BRAKE, 25.0, target, 8.0)
        )
    # すべての点が狙い以上なら、いちばん弱い点の開度（従来どおり）
    assert m.cap_pct(PEDAL_BRAKE, 25.0, 1.0, 8.0, extrapolate=False) == pytest.approx(12.0)


def test_no_extrapolation_still_needs_two_points_and_works_on_the_accel_side() -> None:
    m = GLimitMap()
    m.add(PEDAL_ACCEL, 5.0, 10.0, 3.0)
    assert m.cap_pct(PEDAL_ACCEL, 5.0, 9.0, 6.0, extrapolate=False) is None  # 点が 1 つだけ
    m.add(PEDAL_ACCEL, 5.0, 7.0, 1.0)
    assert m.cap_pct(PEDAL_ACCEL, 5.0, 9.0, 6.0, extrapolate=False) == pytest.approx(10.0)


def test_close_points_do_not_make_a_slope() -> None:
    m = GLimitMap()
    m.add(PEDAL_BRAKE, 25.0, 14.0, 7.0)
    m.add(PEDAL_BRAKE, 25.0, 14.5, 7.4)  # 開度の差 0.5% 未満…ではなく 1% 未満
    assert m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0) is None


def test_interpolates_when_target_inside_measured_range() -> None:
    m = GLimitMap()
    m.add(PEDAL_BRAKE, 25.0, 12.0, 4.0)
    m.add(PEDAL_BRAKE, 25.0, 20.0, 12.0)
    assert m.cap_pct(PEDAL_BRAKE, 25.0, 8.0, 8.0) == pytest.approx(16.0)


def test_coast_point_at_deadband_serves_as_second_point() -> None:
    m = GLimitMap()
    m.add_coast(25.0, 1.6)
    m.add(PEDAL_BRAKE, 25.0, 14.0, 7.0)
    cap = m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0)
    # (8%, 1.6) → (14%, 7.0): 0.9 km/h/s per %
    assert cap == pytest.approx(14.0 + (CAP - 7.0) / 0.9)


def test_accel_side_uses_negative_coast_as_deadband_point() -> None:
    m = GLimitMap()
    m.add_coast(25.0, 1.6)  # アクセル不感帯の開度では加速度 −1.6
    m.add(PEDAL_ACCEL, 25.0, 20.0, 5.0)
    cap = m.cap_pct(PEDAL_ACCEL, 25.0, CAP, 5.0)
    slope = (5.0 + 1.6) / 15.0
    assert cap == pytest.approx(20.0 + (CAP - 5.0) / slope)


def test_cap_never_below_deadband() -> None:
    m = GLimitMap()
    m.add(PEDAL_BRAKE, 25.0, 9.0, 20.0)
    m.add(PEDAL_BRAKE, 25.0, 10.0, 30.0)
    assert m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0) >= 8.0


def test_brake_falls_back_to_faster_bin_first() -> None:
    m = GLimitMap()
    for v, u in ((45.0, 10.0), (15.0, 20.0)):  # 速い帯ほど小さい開度で届く
        m.add(PEDAL_BRAKE, v, u - 2.0, CAP - 4.0)
        m.add(PEDAL_BRAKE, v, u, CAP - 1.0)
    # 25 km/h は測れていない: 遅い帯（15）ではなく速い帯（45）の予測を使う（安全側）
    cap = m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0)
    assert cap == pytest.approx(m.cap_pct(PEDAL_BRAKE, 45.0, CAP, 8.0))
    assert cap < m.cap_pct(PEDAL_BRAKE, 15.0, CAP, 8.0)


def test_accel_falls_back_to_slower_bin_first() -> None:
    m = GLimitMap()
    for v, u in ((15.0, 20.0), (45.0, 40.0)):  # 遅い帯ほど小さい開度で届く
        m.add(PEDAL_ACCEL, v, u - 2.0, CAP - 4.0)
        m.add(PEDAL_ACCEL, v, u, CAP - 1.0)
    cap = m.cap_pct(PEDAL_ACCEL, 25.0, CAP, 5.0)
    assert cap == pytest.approx(m.cap_pct(PEDAL_ACCEL, 15.0, CAP, 5.0))


def test_table_shows_predicted_cap_per_bin_and_dash_when_unknown() -> None:
    m = GLimitMap()
    m.add_coast(25.0, 1.6)
    m.add(PEDAL_BRAKE, 25.0, 14.0, 7.0)
    lines = m.table(CAP, 8.0, 5.0, 140.0)
    row = next(line for line in lines if line.strip().startswith("20〜"))
    assert "18.0" in row and "—" in row  # ブレーキ (14 + 3.6/0.9)≒18.0、アクセルは点が無い
    assert GLimitMap().table(CAP, 8.0, 5.0, 140.0) == [lines[0]]


def test_coast_decel_kmhs_returns_median_and_none_when_missing() -> None:
    m = GLimitMap()
    assert m.coast_decel_kmhs(25.0) is None
    m.add_coast(25.0, 1.4)
    m.add_coast(25.0, 1.6)
    m.add_coast(25.0, 1.8)
    assert m.coast_decel_kmhs(25.0) == pytest.approx(1.6)  # 中央値
    assert m.coast_decel_kmhs(45.0) is None  # 別の車速帯には無い


def test_measured_safe_opening_is_a_floor_for_a_noisy_extrapolation() -> None:
    m = GLimitMap()
    m.add(PEDAL_BRAKE, 25.0, 9.0, 1.0)
    m.add(PEDAL_BRAKE, 25.0, 12.0, 6.0)
    m.add(PEDAL_BRAKE, 25.0, 30.0, 0.5)  # 遅れで開度と対応しない点。それでも「届かなかった開度」
    assert m.cap_pct(PEDAL_BRAKE, 25.0, CAP, 8.0) >= 30.0


def test_strongest_point_returns_the_point_with_the_highest_accel_in_the_bin() -> None:
    """段7c: 格子ステップの最初の感度の種まき（gain_fn）に使う一番効いた実測点。"""
    m = GLimitMap()
    m.add(PEDAL_ACCEL, 15.0, 20.0, 3.0)
    m.add(PEDAL_ACCEL, 15.0, 40.0, 6.35)  # 一番効いた点
    m.add(PEDAL_ACCEL, 15.0, 10.0, 0.5)
    assert m.strongest_point(PEDAL_ACCEL, 15.0) == pytest.approx((40.0, 6.35))


def test_table_shows_100_percent_unreachable_instead_of_a_number_over_100() -> None:
    """段7d: 予測開度が 100% 超の帯は「100%(届かない)」と表示する（実機 050944 の 110km/h〜）。"""
    m = GLimitMap()
    m.add(PEDAL_ACCEL, 115.0, 70.0, 4.7)  # 機構上限近く踏んでも 0.13G 程度で 0.3G に届かない
    m.add_coast(115.0, 2.6)
    lines = m.table(CAP, 8.0, 5.0, 140.0)
    row = next(line for line in lines if line.strip().startswith("110〜"))
    assert "100%(届かない)" in row
    assert m.cap_pct(PEDAL_ACCEL, 115.0, CAP, 5.0) > 100.0  # 頭打ちとしての値は変えない


def test_strongest_point_is_none_without_data_and_ignores_other_bins_and_pedal() -> None:
    m = GLimitMap()
    assert m.strongest_point(PEDAL_ACCEL, 15.0) is None
    m.add(PEDAL_ACCEL, 25.0, 40.0, 6.35)  # 別の車速帯
    m.add(PEDAL_BRAKE, 15.0, 13.0, 5.4)  # 別のペダル
    assert m.strongest_point(PEDAL_ACCEL, 15.0) is None


def test_sensitivity_returns_slope_of_points_on_a_line() -> None:
    m = GLimitMap()
    for u, a in ((10.0, 1.0), (12.0, 2.0), (14.0, 3.0), (16.0, 4.0)):
        m.add(PEDAL_BRAKE, 25.0, u, a)
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 13.0, 8.0) == pytest.approx(0.5)


def test_sensitivity_uses_points_near_the_current_opening() -> None:
    """近くの点の傾きが全点の傾きより大きいときは近くの点の傾き（0.5 側を見分ける）。"""
    m = GLimitMap()
    for u in (8.0, 10.0, 12.0, 14.0):  # 傾き 0.2
        m.add(PEDAL_BRAKE, 25.0, u, 0.2 * (u - 8.0))
    for u in (16.0, 18.0, 20.0, 22.0, 24.0):  # 傾き 0.5
        m.add(PEDAL_BRAKE, 25.0, u, 1.2 + 0.5 * (u - 14.0))
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 22.0, 8.0) == pytest.approx(0.5, abs=0.05)
    # 0.2 側の近くでも、全点の傾き（0.2 と 0.5 の間）のほうが大きいのでそちらが返る
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 10.0, 8.0) > 0.25


def test_sensitivity_returns_the_larger_of_near_and_all_points_slopes() -> None:
    """近くの点の傾きが小さく出ても（雑音）、全点の傾きが大きければそれを返す（危険側を避ける）。"""
    m = GLimitMap()
    for u, a in ((10.0, 3.0), (11.0, 3.1), (12.0, 3.0), (30.0, 12.0)):
        m.add(PEDAL_BRAKE, 25.0, u, a)
    near_only = [(10.0, 3.0), (11.0, 3.1), (12.0, 3.0)]
    assert _lsq_slope(near_only, 3) is None or _lsq_slope(near_only, 3) < 0.1  # 近くはほぼ平ら
    all_slope = _lsq_slope(near_only + [(30.0, 12.0)], 2)
    assert all_slope is not None and all_slope > 0.3
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 11.0, 8.0) == pytest.approx(all_slope)


def test_sensitivity_falls_back_to_all_points_and_coast_when_window_is_sparse() -> None:
    m = GLimitMap()
    m.add_coast(25.0, 1.6)
    m.add(PEDAL_BRAKE, 25.0, 14.0, 7.0)  # 窓（±5%）の中は 1 点だけ（惰行の点 8% は 6 離れている）
    # 全点: (8, 1.6), (14, 7.0) → 0.9
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 14.0, 8.0) == pytest.approx(0.9)


def test_sensitivity_falls_back_to_other_bin_by_pedal_side() -> None:
    m = GLimitMap()
    for v, k in ((45.0, 0.8), (15.0, 0.3)):
        for u in (10.0, 14.0):
            m.add(PEDAL_BRAKE, v, u, k * (u - 8.0))
            m.add(PEDAL_ACCEL, v, u, k * (u - 8.0))
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 12.0, 8.0) == pytest.approx(0.8)
    assert m.sensitivity_kmhs_per_pct(PEDAL_ACCEL, 25.0, 12.0, 8.0) == pytest.approx(0.3)


def test_sensitivity_is_none_without_data_with_one_point_or_nonpositive_slope() -> None:
    m = GLimitMap()
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 12.0, 8.0) is None
    m.add(PEDAL_BRAKE, 25.0, 12.0, 3.0)
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 12.0, 8.0) is None
    m.add(PEDAL_BRAKE, 25.0, 16.0, 3.0)  # 傾き 0
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 12.0, 8.0) is None
    m.add(PEDAL_BRAKE, 25.0, 20.0, 1.0)  # 傾き負
    assert m.sensitivity_kmhs_per_pct(PEDAL_BRAKE, 25.0, 12.0, 8.0) is None
