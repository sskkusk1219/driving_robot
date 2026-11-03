"""運転指標（drive_index.py）のユニットテスト。人工データで定義どおりに計算されることを確かめる。"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research import drive_index as di


def _ref(n: int = 600) -> tuple[np.ndarray, np.ndarray]:
    """0.1s 刻み。0 → 60 km/h まで加速して保ち、減速する波形。"""
    t = np.arange(n) * 0.1
    v = np.interp(t, [0, 10, 40, 50, n * 0.1], [0, 0, 60, 60, 0])
    return t, v


ROAD = {"mass_kg": 1500.0, "road_load_a0_n": 100.0,
        "road_load_a1_n_per_kmh": 0.5, "road_load_a2_n_per_kmh2": 0.03}


def test_dr_is_plus_one_percent_for_scaled_speed() -> None:
    t, v = _ref()
    res = di.compute_drive_index(t, v, v * 1.01)
    assert res.total.dr_pct == pytest.approx(1.0, abs=1e-6)


def test_rmsse_equals_constant_offset() -> None:
    t, v = _ref()
    res = di.compute_drive_index(t, v + 5.0, v + 5.5)
    assert res.total.rmsse_kmh == pytest.approx(0.5, abs=1e-9)


def test_identical_series_gives_zero_indices() -> None:
    t, v = _ref()
    res = di.compute_drive_index(t, v, v, **ROAD)
    tot = res.total
    for x in (tot.dr_pct, tot.er_pct, tot.eer_pct, tot.iwr_pct, tot.ascr_pct, tot.rmsse_kmh):
        assert x == pytest.approx(0.0, abs=1e-9)


def test_iwr_does_not_depend_on_mass() -> None:
    t, v = _ref()
    act = v * 1.02
    a = di.compute_drive_index(t, v, act, mass_kg=900.0).total.iwr_pct
    b = di.compute_drive_index(t, v, act, mass_kg=2500.0).total.iwr_pct
    c = di.compute_drive_index(t, v, act).total.iwr_pct  # 未設定は M=1
    assert a == pytest.approx(b) == pytest.approx(c)
    assert a is not None and a > 0.0


def test_er_eer_none_unless_road_load_complete() -> None:
    t, v = _ref()
    none = di.compute_drive_index(t, v, v * 1.01).total
    assert none.er_pct is None and none.eer_pct is None
    partial = di.compute_drive_index(t, v, v * 1.01, mass_kg=1500.0, road_load_a0_n=100.0).total
    assert partial.er_pct is None and partial.eer_pct is None
    full = di.compute_drive_index(t, v, v * 1.01, **ROAD).total
    assert full.er_pct is not None and full.eer_pct is not None
    # EER は距離のずれを除いたエネルギーのずれ
    assert full.eer_pct == pytest.approx(
        (1 - (full.er_pct / 100 + 1) / (full.dr_pct / 100 + 1)) * 100
    )


def test_phases_use_only_their_own_rows_not_cumulative() -> None:
    t = np.arange(400) * 0.1
    v = np.full(400, 50.0)
    act = v.copy()
    act[200:] = 55.0  # 後半だけ 10% 速い
    seg = ["A"] * 200 + ["B"] * 200
    res = di.compute_drive_index(t, v, act, segments=seg)
    a, b = res.phases
    assert (a.label, b.label) == ("A", "B")
    assert a.dr_pct == pytest.approx(0.0, abs=1e-9)
    assert b.dr_pct == pytest.approx(10.0, abs=1e-6)
    assert a.distance_ref_km + b.distance_ref_km == pytest.approx(res.total.distance_ref_km)
    assert b.n_rows == 200


def test_smoothing_edges_average_available_points() -> None:
    out = di.smooth_centered(np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]))
    assert out[0] == pytest.approx(2.0)  # (1+2+3)/3
    assert out[1] == pytest.approx(2.5)  # (1+2+3+4)/4
    assert out[2] == pytest.approx(3.0)  # (1..5)/5
    assert out[-1] == pytest.approx(5.0)  # (4+5+6)/3


def test_length_mismatch_raises() -> None:
    with pytest.raises(ValueError):
        di.compute_drive_index([0.0, 0.1], [0.0], [0.0, 0.0])


def test_non_uniform_input_is_resampled_to_0p1s_grid() -> None:
    t, v = _ref()
    fine_t = np.arange(0.0, t[-1], 0.05)
    fine_v = np.interp(fine_t, t, v)
    a = di.compute_drive_index(t, v, v * 1.01).total
    b = di.compute_drive_index(fine_t, fine_v, fine_v * 1.01).total
    assert b.dr_pct == pytest.approx(a.dr_pct, abs=1e-3)
    assert b.rmsse_kmh == pytest.approx(a.rmsse_kmh, abs=0.01)
