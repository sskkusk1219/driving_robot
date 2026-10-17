"""play_onset（ペダルの効き始めの折れ点）の単体テスト。合成データのみ。"""

from __future__ import annotations

import numpy as np

from tests.research.play_onset import coast_fn_from_curve, onset_from_series

COAST = 0.6
DT = 0.05


def _coast(v: np.ndarray) -> np.ndarray:
    return np.full_like(np.asarray(v, dtype=float), COAST)


def _synth(u0: float, k: float, sign: float, pmin: float = 4.0, umax: float = 14.0,
           slow: float = 0.5):  # fmt: skip
    """開度を pmin から umax まで slow %/s で上げ、加速度 = −sign... を積分した速度を作る。"""
    n = int((umax - pmin) / slow / DT)
    t = np.arange(n) * DT
    u = pmin + slow * t
    # d = sign*(a+coast) = k*max(0,u-u0) -> a = sign*d - coast
    a = sign * k * np.maximum(0.0, u - u0) - COAST
    v = 80.0 + np.cumsum(a) * DT
    return t, v, u


def test_hinge_recovered_accel():
    t, v, u = _synth(10.0, 0.3, 1.0)
    other = np.full_like(u, 2.9)
    r = onset_from_series(t, v, u, other, pedal_min_pct=4.0, other_min_pct=2.9,
                          coast_fn=_coast, sign=1.0)  # fmt: skip
    assert r.onset_pct is not None
    assert abs(r.onset_pct - 10.0) <= 0.15
    assert abs(r.k - 0.3) < 0.05


def test_brake_sign():
    t, v, u = _synth(6.5, 0.4, -1.0, pmin=3.0, umax=10.0)
    other = np.full_like(u, 4.0)
    r = onset_from_series(t, v, u, other, pedal_min_pct=3.0, other_min_pct=4.0,
                          coast_fn=_coast, sign=-1.0)  # fmt: skip
    assert r.onset_pct is not None
    assert abs(r.onset_pct - 6.5) <= 0.15


def test_only_flat_is_none():
    t, v, u = _synth(50.0, 0.3, 1.0)
    other = np.full_like(u, 2.9)
    r = onset_from_series(t, v, u, other, pedal_min_pct=4.0, other_min_pct=2.9,
                          coast_fn=_coast, sign=1.0)  # fmt: skip
    assert r.onset_pct is None


def test_only_effective_is_none():
    t, v, u = _synth(3.0, 0.3, 1.0)
    other = np.full_like(u, 2.9)
    r = onset_from_series(t, v, u, other, pedal_min_pct=4.0, other_min_pct=2.9,
                          coast_fn=_coast, sign=1.0)  # fmt: skip
    assert r.onset_pct is None


def test_negative_k_is_none():
    t, v, u = _synth(10.0, -0.3, 1.0)
    other = np.full_like(u, 2.9)
    r = onset_from_series(t, v, u, other, pedal_min_pct=4.0, other_min_pct=2.9,
                          coast_fn=_coast, sign=1.0)  # fmt: skip
    assert r.onset_pct is None


def test_other_pedal_not_at_min_excluded():
    t, v, u = _synth(10.0, 0.3, 1.0)
    other = np.full_like(u, 20.0)
    r = onset_from_series(t, v, u, other, pedal_min_pct=4.0, other_min_pct=2.9,
                          coast_fn=_coast, sign=1.0)  # fmt: skip
    assert r.onset_pct is None
    assert r.bins == ()


def test_releasing_rows_excluded():
    t, v, u = _synth(10.0, 0.3, 1.0)
    u = u[::-1].copy()  # 戻し中だけの時系列
    other = np.full_like(u, 2.9)
    r = onset_from_series(t, v, u, other, pedal_min_pct=4.0, other_min_pct=2.9,
                          coast_fn=_coast, sign=1.0)  # fmt: skip
    assert r.onset_pct is None
    assert r.bins == ()


def test_coast_fn_clamps():
    fn = coast_fn_from_curve([0.0, 10.0], [0.0, 1.0])
    out = fn(np.array([-5.0, 5.0, 20.0]))
    assert np.allclose(out, [0.0, 0.5, 1.0])


def _step_profile(u: np.ndarray, off: float) -> np.ndarray:
    """平ら（d=0）から 10.8〜11.05% で段のように立ち上がり、先は緩い傾きで続く d（実データ形）。"""
    step = np.clip(1.6 * (u - 10.8), 0.0, 0.4)
    return off + step + np.where(u > 11.05, 0.12 * (u - 11.05), 0.0)


def test_step_onset_refit_widens_span():
    # 2026-10-16 段2o-7: 段の立ち上がりで全域の最初の当て込みが手前に出ても、当て直しの範囲を
    # 広げて効き始めが段の位置に戻ること（惰行基準の ±0.03 のずれでも変わらない）。
    n = int((50.0 - 4.2) / 0.5 / DT)
    t = np.arange(n) * DT
    u = 4.2 + 0.5 * t
    other = np.full_like(u, 2.9)
    for off in (0.0, 0.03, -0.03):
        a = _step_profile(u, off) - COAST
        v = 80.0 + np.cumsum(a) * DT
        r = onset_from_series(t, v, u, other, pedal_min_pct=4.0, other_min_pct=2.9,
                              coast_fn=_coast, sign=1.0)  # fmt: skip
        assert r.onset_pct is not None, (off, r.reason)
        assert 10.2 <= r.onset_pct <= 10.7, (off, r.onset_pct)
