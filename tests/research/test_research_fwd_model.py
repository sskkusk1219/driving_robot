"""fwd_model（モデル版の順モデル: 学習・格子化・単調化・pkl）のテスト。合成データだけを使う。"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research.fwd_data import FwdRows, make_speed_edges
from tests.research.fwd_map import (
    KIND_MODEL,
    POINT_MEASURED,
    ForwardTable,
    build_table,
    load_forward_map,
    save_table,
)
from tests.research.fwd_model import (
    NAME_LINEAR,
    NAME_MLP,
    NAME_RIDGE,
    FittedModel,
    fit_models,
    mean_grid,
    model_to_grid,
    monotone_violation,
    raw_grid,
)

DT = 0.05


def true_accel(v, u):
    """既知の 2 次曲面（u の符号で式が違う）。"""
    v = np.asarray(v, dtype=float)
    u = np.asarray(u, dtype=float)
    return np.where(
        u >= 0,
        -1.0 - 0.01 * v + 0.30 * u - 0.002 * u * u + 0.0005 * v * u,
        -1.0 - 0.01 * v + 0.50 * u + 0.004 * u * u,
    )


def make_rows(seed: int, source: str, n: int = 6000) -> FwdRows:
    rng = np.random.default_rng(seed)
    v = rng.uniform(3.0, 78.0, n)
    u = rng.uniform(-20.0, 20.0, n)
    u[rng.random(n) < 0.05] = 0.0
    return FwdRows(
        speed_kmh=v,
        u_pct=u,
        accel_kmhs=true_accel(v, u),
        t_s=np.arange(n) * DT,
        dt_s=DT,
        source=source,
        dt_row_s=np.full(n, DT),
    )


@pytest.fixture(scope="module")
def logs() -> list[FwdRows]:
    return [make_rows(i, f"log{i}") for i in range(3)]


@pytest.fixture(scope="module")
def like(logs) -> object:
    from tests.research.fwd_data import concat_rows  # noqa: PLC0415

    return build_table(concat_rows(logs), speed_edges=make_speed_edges(80.0))


@pytest.fixture(scope="module")
def fitted(logs):
    return fit_models(logs, mlp_max_iter=60)


def test_fit_recovers_quadratic_surface(fitted):
    v = np.array([10.0, 40.0, 70.0, 25.0])
    u = np.array([5.0, 15.0, 2.0, -12.0])
    x = np.column_stack([v, u])
    for name in (NAME_LINEAR, NAME_RIDGE):
        f = fitted[name]
        side = np.where(u >= 0, f.accel_side.predict(x), f.brake_side.predict(x))
        assert np.allclose(side, true_accel(v, u), atol=0.05), name
    assert fitted[NAME_RIDGE].alpha in (0.01, 0.1, 1.0, 10.0, 100.0)
    assert fitted[NAME_MLP].hidden == (10,)
    for f in fitted.values():
        assert f.train_time_s >= 0
        assert f.sources == ["log0", "log1", "log2"]


def test_grid_copies_like_and_zero_is_mean(fitted, like):
    f = fitted[NAME_LINEAR]
    tb = model_to_grid(f, like)
    assert tb.kind == KIND_MODEL
    assert np.array_equal(tb.point_kind, like.point_kind)
    assert np.array_equal(tb.seconds, like.seconds)
    assert np.array_equal(tb.meas_u_min, like.meas_u_min)
    assert np.array_equal(tb.u_grid, like.u_grid)
    j0 = int(np.argmin(np.abs(tb.u_grid)))
    rep = tb.speed_rep
    x = np.column_stack([rep, np.zeros_like(rep)])
    mean0 = (f.accel_side.predict(x) + f.brake_side.predict(x)) / 2.0
    assert np.allclose(tb.accel_before_mono[:, j0], mean0)
    assert tb.meta["model_name"] == NAME_LINEAR


def test_grid_is_monotone_and_matches_truth(fitted, like):
    tb = model_to_grid(fitted[NAME_RIDGE], like)
    assert np.all(np.diff(tb.accel_grid, axis=1) >= -1e-9)
    vmax, vmean = monotone_violation(tb)
    assert 0.0 <= vmean <= vmax < 0.1
    assert abs(tb.accel(40.0, 10.0) - true_accel(40.0, 10.0)) < 0.1


def test_nonmonotone_raw_gives_positive_violation(fitted, like):
    tb = model_to_grid(fitted[NAME_LINEAR], like)
    r, j = np.argwhere(tb.point_kind == POINT_MEASURED)[len(tb.point_kind) // 2]
    raw = tb.accel_before_mono.copy()
    raw[r, j] += 1.0  # 1 点だけ持ち上げた非単調な配列
    tb.accel_before_mono = raw
    assert monotone_violation(tb)[0] > 0.05


def test_pkl_roundtrip_keeps_kind(fitted, like, tmp_path):
    tb = model_to_grid(fitted[NAME_MLP], like)
    path = save_table(tb, tmp_path / "m.pkl")
    back = load_forward_map(path)
    assert back.kind == KIND_MODEL
    assert np.allclose(back.accel_grid, tb.accel_grid)
    assert back.meta["model_name"] == NAME_MLP
    assert back.meta["hidden"] == (10,)


def test_default_unchanged_and_coast_from_accel(fitted, like):
    f = fitted[NAME_LINEAR]
    tb = model_to_grid(f, like)
    assert tb.meta["coast_from"] == "mean"
    assert np.array_equal(tb.accel_before_mono, raw_grid(f, like))
    tb2 = model_to_grid(f, like, coast_from="accel")
    j0 = int(np.argmin(np.abs(tb2.u_grid)))
    x = np.column_stack([tb2.speed_rep, np.zeros_like(tb2.speed_rep)])
    assert np.allclose(tb2.accel_before_mono[:, j0], f.accel_side.predict(x))
    with pytest.raises(ValueError):
        raw_grid(f, like, coast_from="x")


def test_mean_grid_averages_before_isotonic(fitted, like):
    fs = [fitted[NAME_LINEAR], fitted[NAME_RIDGE], fitted[NAME_MLP]]
    tb = mean_grid(fs, like)
    mean_raw = np.mean([raw_grid(f, like) for f in fs], axis=0)
    assert np.allclose(tb.accel_before_mono, mean_raw)
    assert tb.meta["n_models"] == 3
    assert np.all(np.diff(tb.accel_grid, axis=1) >= -1e-9)


def test_mean_grid_differs_from_mean_of_monotonized():
    """非単調な 2 本の平均: 先に平均して単調化 ≠ 単調化してから平均。"""
    ug = np.array([-1.0, 0.0, 1.0])
    like_tb = ForwardTable(
        speed_edges=np.array([0.0, 10.0]),
        speed_rep=np.array([5.0]),
        u_grid=ug,
        accel_grid=np.zeros((1, 3)),
        seconds=np.ones((1, 3)),
        point_kind=np.full((1, 3), POINT_MEASURED),
        meas_u_min=np.array([-1.0]),
        meas_u_max=np.array([1.0]),
    )

    class Const:
        def __init__(self, vals):
            self.vals = np.asarray(vals, dtype=float)

        def predict(self, x):
            return self.vals[np.searchsorted(ug, x[:, 1])]

    a = FittedModel("a", Const([0.0, 1.0, 0.0]), Const([0.0, 1.0, 0.0]))
    b = FittedModel("b", Const([0.0, 0.0, 3.0]), Const([0.0, 0.0, 3.0]))
    avg_then_mono = mean_grid([a, b], like_tb).accel_grid
    grid_a = model_to_grid(a, like_tb).accel_grid
    grid_b = model_to_grid(b, like_tb).accel_grid
    mono_then_avg = (grid_a + grid_b) / 2
    assert not np.allclose(avg_then_mono, mono_then_avg)
