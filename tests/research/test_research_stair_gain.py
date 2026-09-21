"""stair_gain（段3-3: 低開度階段からのペダルゲイン k(v)・x0 の当てはめ）のユニットテスト。

`test_research_accel_onset.py` と同じ流儀で、`fit_stair_gain` の中核（scipy 不使用の
x0 走査 + lstsq）を合成データで検算する。
"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research import stair_gain as sg

TRUE_K0 = 2.5
TRUE_K1 = -0.03
TRUE_X0 = 0.2


def _synthetic_vxa(
    *, noise_std: float = 0.0, seed: int = 0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`a = (k0 + k1*v)*(x - x0)` を仕込んだ合成データ（v と x を独立に振った格子）。"""
    v_vals = np.linspace(5.0, 25.0, 9)
    x_vals = np.linspace(0.5, 4.0, 8)
    v_grid, x_grid = np.meshgrid(v_vals, x_vals)
    v = v_grid.ravel()
    x = x_grid.ravel()
    a = (TRUE_K0 + TRUE_K1 * v) * (x - TRUE_X0)
    if noise_std > 0.0:
        rng = np.random.default_rng(seed)
        a = a + rng.normal(0.0, noise_std, size=a.shape)
    return v, x, a


def test_fit_recovers_known_parameters_without_noise() -> None:
    v, x, a = _synthetic_vxa()

    fit = sg.fit_stair_gain(v, x, a)

    assert fit is not None
    assert fit.k0 == pytest.approx(TRUE_K0, abs=0.01)
    assert fit.k1 == pytest.approx(TRUE_K1, abs=0.01)
    assert fit.x0_pct == pytest.approx(TRUE_X0, abs=0.01)


def test_fit_recovers_parameters_with_noise() -> None:
    v, x, a = _synthetic_vxa(noise_std=0.05, seed=0)

    fit = sg.fit_stair_gain(v, x, a)

    assert fit is not None
    assert fit.k0 == pytest.approx(TRUE_K0, abs=0.1)
    assert fit.k1 == pytest.approx(TRUE_K1, abs=0.1)
    assert fit.x0_pct == pytest.approx(TRUE_X0, abs=0.1)


def test_fit_returns_none_when_x_has_single_value() -> None:
    v = np.linspace(5.0, 25.0, 20)
    x = np.full(20, 1.5)
    a = (TRUE_K0 + TRUE_K1 * v) * (x - TRUE_X0)

    fit = sg.fit_stair_gain(v, x, a)

    assert fit is None


def test_fit_returns_none_when_too_few_samples() -> None:
    v = np.linspace(5.0, 25.0, 9)
    x = np.linspace(0.5, 4.0, 9)
    a = (TRUE_K0 + TRUE_K1 * v) * (x - TRUE_X0)

    fit = sg.fit_stair_gain(v, x, a)

    assert fit is None


def test_fit_ignores_nan_rows() -> None:
    v, x, a = _synthetic_vxa()
    fit_clean = sg.fit_stair_gain(v, x, a)
    assert fit_clean is not None

    v_nan = np.concatenate([v, [np.nan, 12.0, np.nan]])
    x_nan = np.concatenate([x, [1.0, np.nan, np.nan]])
    a_nan = np.concatenate([a, [np.nan, np.nan, 3.0]])

    fit_with_nan = sg.fit_stair_gain(v_nan, x_nan, a_nan)

    assert fit_with_nan is not None
    assert fit_with_nan.k0 == pytest.approx(fit_clean.k0, abs=1e-9)
    assert fit_with_nan.k1 == pytest.approx(fit_clean.k1, abs=1e-9)
    assert fit_with_nan.x0_pct == pytest.approx(fit_clean.x0_pct, abs=1e-9)
    assert fit_with_nan.n == fit_clean.n


def test_gain_at_is_linear_in_speed() -> None:
    fit = sg.StairGainFit(k0=2.0, k1=-0.05, x0_pct=0.3, r2=0.9, n=100)

    for speed in (0.0, 10.0, 20.0, 30.0):
        assert fit.gain_at(speed) == pytest.approx(2.0 + (-0.05) * speed)
