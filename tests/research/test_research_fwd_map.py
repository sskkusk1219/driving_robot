"""fwd_map（順モデルの表・逆引き・pkl）のテスト。合成データだけを使う。"""

from __future__ import annotations

import pickle
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from tests.research.fwd_data import (
    Breakaway,
    FwdRows,
    concat_rows,
    make_speed_edges,
    u_cell,
)
from tests.research.fwd_map import (
    POINT_FILLED,
    POINT_MEASURED,
    POINT_OUT,
    RELEASE_GRID_PCT,
    ForwardTable,
    apply_launch_accel_measured,
    apply_side_row_speed_rep,
    attach_accel_release,
    attach_launch,
    attach_release,
    attach_stop,
    attach_stop_release,
    build_table,
    extend_launch_rows_to_stop_max,
    load_forward_map,
    refine_u_grid,
    release_bin_edges,
    release_seconds_in_band,
    save_table,
    u_grid_points,
)

DT = 0.05


def true_accel(v, u):
    """既知の関数: u<0 側は傾きを変える。"""
    u = np.asarray(u, dtype=float)
    return -2.0 - 0.01 * np.asarray(v, dtype=float) + np.where(u >= 0, 0.3 * u, 0.6 * u)


def make_rows(
    v: np.ndarray, u: np.ndarray, a: np.ndarray, dt: float = DT
) -> FwdRows:
    n = len(v)
    return FwdRows(
        speed_kmh=np.asarray(v, dtype=float),
        u_pct=np.asarray(u, dtype=float),
        accel_kmhs=np.asarray(a, dtype=float),
        t_s=np.arange(n) * dt,
        dt_s=dt,
        source="synthetic",
        dt_row_s=np.full(n, dt),
    )


@pytest.fixture(scope="module")
def synth_rows() -> FwdRows:
    rng = np.random.default_rng(0)
    n = 80000
    v = rng.uniform(3.0, 78.0, n)
    u = rng.uniform(-20.0, 20.0, n)
    u[rng.random(n) < 0.05] = 0.0  # 惰行の行
    return make_rows(v, u, true_accel(v, u))


@pytest.fixture(scope="module")
def synth_table(synth_rows):
    return build_table(synth_rows, speed_edges=make_speed_edges(80.0))


def test_accel_matches_true_function(synth_table) -> None:
    for v in (12.0, 30.0, 52.0, 70.0):
        for u in (-15.0, -6.0, 0.0, 5.0, 14.0):
            assert synth_table.accel(v, u) == pytest.approx(float(true_accel(v, u)), abs=0.06)


def test_invert_returns_original_u(synth_table) -> None:
    v = np.array([15.0, 33.0, 47.0, 66.0, 41.0])
    u = np.array([-12.0, -3.0, 8.0, 15.0, 4.0])
    a = synth_table.accel(v, u)
    back, flag = synth_table.invert(v, a)
    assert np.allclose(back, u, atol=1e-6)
    assert np.all(flag == POINT_MEASURED)


def test_median_ignores_outliers() -> None:
    n = 400
    v = np.full(n, 12.0)
    u = np.full(n, 6.0)  # 1 マスだけ（u=6 は番号 3、中心 5）
    a = np.full(n, 1.0)
    a[:30] = 50.0  # 外れ値（平均なら 4.5 になる）
    other = make_rows(np.full(n, 12.0), np.full(n, -6.0), np.full(n, -1.0))
    rows = make_rows(np.concatenate([v, other.speed_kmh]), np.concatenate([u, other.u_pct]),
                     np.concatenate([a, other.accel_kmhs]))
    t = build_table(rows, speed_edges=np.array([0.0, 5.0, 10.0, 15.0, 20.0]))
    j = int(np.argmin(np.abs(t.u_grid - 5.0)))
    assert t.accel_grid[2, j] == pytest.approx(1.0)


def test_holes_filled_linearly_between_neighbours() -> None:
    edges = np.array([0.0, 10.0, 20.0])
    # 行 1（10〜20）の実測は u = −3, −1, 0, 9, 11（各 100 行 = 5 秒）。1〜7 は穴
    centers = {-3: -3.0, -1: -1.0, 0: 0.0, 9: 9.0, 11: 11.0}
    vs, us, as_ = [], [], []
    for uc, a in centers.items():
        vs += [15.0] * 100
        us += [uc if uc != 0 else 0.0] * 100
        as_ += [0.1 * a] * 100
    t = build_table(make_rows(np.array(vs), np.array(us, dtype=float), np.array(as_)),
                    speed_edges=edges, accel_deadband_pct=1.0)
    row = 1
    kinds = t.point_kind[row]
    j = {round(float(g), 3): i for i, g in enumerate(t.u_grid)}
    # u=−3,−1,0 は実測、0 と 9 の間の穴（1〜7）は穴埋め
    assert kinds[j[-3.0]] == POINT_MEASURED
    mid = j[5.0]
    assert kinds[mid] == POINT_FILLED
    # 0(=0) と 9(=0.9) の間の直線: u=5 → 0.5
    assert t.accel_before_mono[row, mid] == pytest.approx(0.5, abs=1e-6)


def test_out_of_range_stops_at_end_value_and_flags() -> None:
    n = 200
    u = np.repeat([-4.0, -2.0, 0.0, 2.0, 4.0], n // 5)
    a = 0.5 * u  # 実測の範囲は u=−3(中心)〜+3
    rows = make_rows(np.full(n, 12.0), u, a)
    rows2 = make_rows(np.full(n, 22.0), u, a)
    from tests.research.fwd_data import concat_rows
    big = concat_rows([rows, rows2])
    # 実測は ±4 までだが、格子は ±10 まで欲しいので u=±10 のごく短い行を足す（穴になる）
    tiny = make_rows(np.array([12.0, 12.0]), np.array([10.0, -10.0]), np.array([5.0, -5.0]))
    t = build_table(concat_rows([big, tiny]), speed_edges=np.array([0.0, 10.0, 20.0, 30.0]))
    row = 1
    hi = t.meas_u_max[row]
    assert t.accel(15.0, 10.0) == pytest.approx(t.accel(15.0, hi))
    assert t.point_kind[row, -1] == POINT_OUT
    u_inv, flag = t.invert(15.0, t.accel(15.0, hi) + 5.0)
    assert u_inv == pytest.approx(hi)
    assert flag == POINT_OUT
    u_inv, flag = t.invert(15.0, -1e3)
    assert flag == POINT_OUT
    assert u_inv == pytest.approx(t.meas_u_min[row], abs=1.0001)


def test_output_is_monotone_even_for_non_monotone_input() -> None:
    rng = np.random.default_rng(1)
    n = 30000
    v = rng.uniform(10.0, 20.0, n)
    u = rng.uniform(-20.0, 20.0, n)
    a = 0.1 * u + 2.0 * np.sin(u / 3.0) + rng.normal(0, 0.1, n)  # 単調でない
    t = build_table(make_rows(v, u, a), speed_edges=np.array([0.0, 10.0, 20.0]))
    assert np.all(np.diff(t.accel_grid, axis=1) >= -1e-12)
    # 単調化の前は単調でない
    assert np.any(np.diff(t.accel_before_mono[1]) < -0.05)


def test_flat_region_returns_smaller_abs_u() -> None:
    n = 12000
    u = np.tile(np.array([-6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0]), n // 7)
    a = np.where(np.abs(u) <= 4.0, 0.0, 0.5 * u)  # −4〜+4 は平ら（加速度 0）
    t = build_table(make_rows(np.full(len(u), 12.0), u, a),
                    speed_edges=np.array([0.0, 10.0, 20.0]))
    # 平らな区間 (u 中心 −3..+3 で 0): 加速度 0 を引くと |u| の小さい側 = 0
    got, flag = t.invert(15.0, 0.0)
    assert got == pytest.approx(0.0)
    assert flag == POINT_MEASURED
    # 平らが 0 をまたがない場合: 加速度 1.0 の平らな区間が u=+2..+6 → 小さい側 2
    a2 = np.where(u >= 2.0, 1.0, np.where(u < 0.0, -1.0, 0.0))
    t2 = build_table(make_rows(np.full(len(u), 12.0), u, a2),
                     speed_edges=np.array([0.0, 10.0, 20.0]))
    got2, _ = t2.invert(15.0, 1.0)
    assert got2 == pytest.approx(2.0, abs=1.01)  # 平らの下端（u=2 の中心 1 〜 2 付近）
    assert got2 < 3.0


def test_row_with_too_few_points_copies_nearest_row() -> None:
    n = 6000
    u = np.tile(np.array([-4.0, -2.0, 0.0, 2.0, 4.0]), n // 5)
    a = 0.4 * u
    rows = make_rows(np.full(n, 12.0), u, a)  # 行 1 だけにデータ
    edges = np.array([0.0, 10.0, 20.0, 30.0, 40.0])
    one = make_rows(np.array([35.0]), np.array([0.0]), np.array([9.0]))  # 行 3 に 1 点だけ
    from tests.research.fwd_data import concat_rows
    t = build_table(concat_rows([rows, one]), speed_edges=edges)
    assert t.meta["copied_rows"] == [0, 2, 3]
    for r in (0, 2, 3):
        assert np.allclose(t.accel_grid[r], t.accel_grid[1])
        assert np.all(t.point_kind[r] == POINT_OUT)
    _, flag = t.invert(35.0, 0.2)
    assert flag == POINT_OUT


def test_uneven_speed_edges_pick_correct_rows() -> None:
    edges = make_speed_edges(30.0)  # 0,2.5,...,15,20,25,30
    rep = (edges[:-1] + edges[1:]) / 2.0
    assert rep[0] == pytest.approx(1.25) and rep[6] == pytest.approx(17.5)
    # 各行の加速度が行番号になるデータ: v 中心で作る
    vs, us, as_ = [], [], []
    for r, c in enumerate(rep):
        for uu in (-2.0, 0.0, 2.0):
            vs += [c] * 40
            us += [uu] * 40
            as_ += [float(r) + 0.1 * uu] * 40
    t = build_table(make_rows(np.array(vs), np.array(us), np.array(as_)), speed_edges=edges)
    for r, c in enumerate(rep):
        assert t.accel(c, 0.0) == pytest.approx(float(r))
    # 2.5 と 5 の混在: 代表車速の間は直線補間（行 5 の 13.75 と行 6 の 17.5 の中点）
    assert t.accel((rep[5] + rep[6]) / 2.0, 0.0) == pytest.approx(5.5)
    assert t.coast_accel(rep[3]) == pytest.approx(3.0)


def test_pkl_roundtrip_and_model_type_check(synth_table, tmp_path: Path) -> None:
    p = save_table(synth_table, tmp_path / "t.pkl")
    t2 = load_forward_map(p)
    v = np.array([10.0, 40.0, 70.0])
    u = np.array([-5.0, 3.0, 12.0])
    assert np.allclose(t2.accel(v, u), synth_table.accel(v, u))
    assert np.allclose(t2.invert(v, t2.accel(v, u))[0], synth_table.invert(v, t2.accel(v, u))[0])
    assert t2.kind == "table"
    with p.open("rb") as f:
        d = pickle.load(f)
    assert d["model_type"] == "forward_map_v1" and d["kind"] == "table"
    bad = tmp_path / "bad.pkl"
    with bad.open("wb") as f:
        pickle.dump({"model_type": "other"}, f)
    with pytest.raises(ValueError):
        load_forward_map(bad)


def test_invert_is_fast_for_whole_mode(synth_table) -> None:
    rng = np.random.default_rng(2)
    v = rng.uniform(0.0, 80.0, 36000)
    a = rng.uniform(-4.0, 3.0, 36000)
    t0 = time.perf_counter()
    u, flag = synth_table.invert(v, a)
    assert time.perf_counter() - t0 < 5.0
    assert u.shape == (36000,) and flag.shape == (36000,)


def test_refine_u_grid_keeps_range_and_inherits_cells(synth_table) -> None:
    """2026-10-14 段2o-4: 範囲は元と同じ・0 を 1 点含む・秒数と種類は元のマスの値。"""
    fine = refine_u_grid(synth_table, 0.5)
    old = synth_table.u_grid
    # 元のマスの外側の端（元の端の点 ± 刻みの半分）が同じ
    assert fine.u_grid[0] == pytest.approx(old[0] - 0.75)
    assert fine.u_grid[-1] == pytest.approx(old[-1] + 0.75)
    assert np.all(np.diff(fine.u_grid) > 0)
    assert int(np.sum(fine.u_grid == 0.0)) == 1
    assert fine.meta["u_step_pct"] == 0.5 and fine.meta["like_u_step_pct"] == 2.0
    assert fine.accel_grid.shape == fine.seconds.shape == fine.point_kind.shape
    shape = (len(fine.speed_rep), len(fine.u_grid))
    assert fine.accel_grid.shape == fine.accel_before_mono.shape == shape
    cells = u_cell(fine.u_grid, 2.0)
    col = cells - int(u_cell(old, 2.0).min())
    assert np.array_equal(fine.seconds, synth_table.seconds[:, col])
    assert np.array_equal(fine.point_kind, synth_table.point_kind[:, col])
    assert np.array_equal(fine.meas_u_min, synth_table.meas_u_min)
    # 新しい点の値は元の表の直線補間そのもの。元の格子点（折れ点）では新しい点の間を結ぶので
    # 折れ点を少し丸めるが、ずれは小さい
    for v in (12.0, 30.0, 52.0, 70.0):
        for u in old[::3]:
            a_old = synth_table.accel(v, float(u))
            assert fine.accel(v, float(u)) == pytest.approx(a_old, abs=0.2)
    r = 5
    assert np.allclose(fine.accel_grid[r], np.interp(fine.u_grid, old, synth_table.accel_grid[r]))
    assert np.all(np.diff(fine.accel_grid, axis=1) >= -1e-9)


def test_refine_u_grid_same_or_coarser_step_returns_same_table(synth_table) -> None:
    assert refine_u_grid(synth_table, 2.0) is synth_table
    assert refine_u_grid(synth_table, 3.0) is synth_table


def test_refine_u_grid_rejects_non_integer_ratio(synth_table) -> None:
    with pytest.raises(ValueError):
        refine_u_grid(synth_table, 0.7)


def _launch_setup(grid_lo: int = -32):
    """本体: v 0〜10 の 4 行（2.5 刻み）、u −16〜+4（0.5 刻み）。本体の u<0 は a = 0.5u。
    発進側の行: v 0〜2.5 の行と 2.5〜5 の行に、u<0 で a = 1.5 + 0.1u の実測を 20 秒ぶん置く。
    u = 0 のクリープの行（a = 1.5）も置く。"""
    edges = np.array([0.0, 2.5, 5.0, 7.5, 10.0])
    grid = u_grid_points(grid_lo, 8, 0.5)
    rep = (edges[:-1] + edges[1:]) / 2.0
    acc = np.tile(np.where(grid < 0, 0.5 * grid, 0.3 * grid), (4, 1))
    table = ForwardTable(
        speed_edges=edges,
        speed_rep=rep,
        u_grid=grid,
        accel_grid=acc,
        seconds=np.full(acc.shape, 5.0),
        point_kind=np.full(acc.shape, POINT_MEASURED, dtype=int),
        meas_u_min=np.full(4, grid[0]),
        meas_u_max=np.full(4, grid[-1]),
        meta={"u_step_pct": 0.5},
    )
    rng = np.random.default_rng(1)
    n = 400
    v = rng.uniform(0.6, 4.9, n)
    u = -rng.choice(np.arange(0.5, 5.01, 0.5), n)
    v = np.concatenate([v, rng.uniform(0.6, 4.9, 100)])
    u = np.concatenate([u, np.zeros(100)])
    rows = make_rows(v, u, 1.5 + 0.1 * u, dt=0.1)
    return table, rows


def test_attach_launch_too_few_rows_leaves_none() -> None:
    table, rows = _launch_setup()
    short = make_rows(rows.speed_kmh[:20], rows.u_pct[:20], rows.accel_kmhs[:20], dt=0.1)  # 2 秒
    t2 = attach_launch(table, short, creep_speed_kmh=5.0)
    assert t2.launch_accel is None
    assert t2.meta["launch_rows_s"] == pytest.approx(2.0)
    empty = make_rows(np.zeros(0), np.zeros(0), np.zeros(0))
    assert attach_launch(table, empty, creep_speed_kmh=5.0).launch_accel is None
    assert table.launch_accel is None  # 元は書き換えない


def test_zero_creep_speed_makes_neither_table() -> None:
    table, rows = _launch_setup()
    assert attach_launch(table, rows, creep_speed_kmh=0.0).launch_accel is None
    t2 = attach_stop(table, rows, creep_speed_kmh=0.0)
    assert t2.stop_accel is None and t2.stop_kind is None


def test_attach_launch_replaces_low_speed_rows_only() -> None:
    table, rows = _launch_setup()
    t2 = attach_launch(table, rows, creep_speed_kmh=5.0, accel_window_s=0.5)
    la = t2.launch_accel
    assert la is not None and la.shape == table.accel_grid.shape
    neg = (table.u_grid <= 0) & (table.u_grid >= -5.0)
    for r in (0, 1):  # 発進側の実測がある行
        assert np.allclose(la[r, neg], 1.5 + 0.1 * table.u_grid[neg], atol=0.05)
        assert np.all(np.diff(la[r]) >= -1e-9)
    # 代表車速 6.25（<= クリープ 5 ＋ 余裕 2.5）の行は実測が無いので本体のまま。高車速も本体
    assert np.array_equal(la[2:], table.accel_grid[2:])
    assert t2.meta["launch_speed_rows"] == [1.25, 3.75]
    assert t2.meta["launch_min_s"] == 0.5 and t2.meta["launch_accel_window_s"] == 0.5
    assert t2.meta["launch_u_step_pct"] == 0.25
    assert np.array_equal(t2.accel_grid, table.accel_grid)


def test_launch_deep_side_is_zero_at_breakaway_and_has_no_out_of_range() -> None:
    table, rows = _launch_setup()
    # 停車中（車速 0.1）の行: u = −8〜−12 で加速度 0（動き出す開度 −8 より深い）。
    # u = −6 の停車中の行は動き出す開度より浅いので使われない
    su = -np.arange(8.0, 12.01, 0.5)
    st_rows = make_rows(np.full(len(su) * 20, 0.1), np.repeat(su, 20), np.zeros(len(su) * 20), 0.1)
    bad = make_rows(np.full(50, 0.1), np.full(50, -6.0), np.zeros(50), 0.1)
    all_rows = concat_rows([rows, st_rows, bad], "x")
    bk = Breakaway(-8.0, "release", (-8.0,))
    t2 = attach_launch(table, all_rows, creep_speed_kmh=5.0, breakaway=bk)
    la, kind = t2.launch_accel, t2.launch_kind
    assert la is not None and kind is not None
    g = table.u_grid
    assert np.allclose(la[0, g <= -8.0], 0.0, atol=1e-9)  # 深い側は 0（-16 まで端の値ではなく 0）
    assert np.all(la[0] >= -1e-9)  # 車速 0 付近の行は 0 未満にしない
    assert np.all(np.diff(la[0]) >= -1e-9)
    assert la[0, np.argmin(np.abs(g + 3.0))] == pytest.approx(1.2, abs=0.1)  # 動き出しの後
    assert not np.any(kind[0, g <= 0.0] == POINT_OUT)  # u <= 0 に範囲外を作らない
    assert t2.meta["launch_breakaway_u_pct"] == -8.0
    assert t2.meta["launch_breakaway_source"] == "release"
    assert t2.meta["launch_breakaway_u_list"] == [-8.0]
    # 動き出す開度が無いときは 0 で埋めない（深い側は本体の傾きでつなぐ）
    t3 = attach_launch(table, rows, creep_speed_kmh=5.0)
    assert t3.launch_accel is not None and t3.meta["launch_breakaway_u_pct"] is None
    g_deep = np.argmin(np.abs(g + 8.0))
    assert t3.launch_accel[0, g_deep] < 0.0  # 本体の傾き（0.5/%）で下がっていく


def test_attach_launch_hole_is_filled_by_straight_line_on_quarter_cells() -> None:
    table, _ = _launch_setup()
    u = np.concatenate([np.full(100, -5.0), np.full(100, -1.0), np.zeros(100)])
    a = np.concatenate([np.zeros(100), np.full(100, 2.0), np.full(100, 3.0)])
    rows = make_rows(np.full(300, 1.0), u, a, 0.1)
    t2 = attach_launch(table, rows, creep_speed_kmh=5.0)
    la = t2.launch_accel
    assert la is not None
    g = table.u_grid
    assert np.interp(-3.0, g, la[0]) == pytest.approx(1.0, abs=0.1)  # 穴は直線
    assert np.interp(-1.0, g, la[0]) == pytest.approx(2.0, abs=0.3)
    assert t2.meta["launch_u_step_pct"] == 0.25


def _stop_rows():
    """停止側の行: v 0.6〜4.9 で u = −1〜−10、a = −0.2 + 0.15u。u = 0 は a = 0.8（クリープ）。"""
    rng = np.random.default_rng(2)
    n = 500
    v = rng.uniform(0.6, 4.9, n)
    u = -rng.choice(np.arange(1.0, 10.01, 0.5), n)
    v = np.concatenate([v, rng.uniform(0.6, 4.9, 100)])
    u = np.concatenate([u, np.zeros(100)])
    a = np.where(u == 0.0, 0.8, -0.2 + 0.15 * u)
    return make_rows(v, u, a, 0.1)


def test_attach_stop_joins_body_beyond_deepest_measurement() -> None:
    table, _ = _launch_setup()
    t2 = attach_stop(table, _stop_rows(), creep_speed_kmh=5.0, accel_window_s=0.5)
    sa, kind = t2.stop_accel, t2.stop_kind
    assert sa is not None and kind is not None
    g = table.u_grid
    sel = (g <= -1.0) & (g >= -10.0)
    assert np.allclose(sa[1, sel], -0.2 + 0.15 * g[sel], atol=0.1)
    assert sa[1, np.argmin(np.abs(g))] == pytest.approx(0.8, abs=0.05)  # u = 0 は実測
    assert np.all(np.diff(sa[1]) >= -1e-9)
    # 実測のいちばん深い所（−10）より深い所: 端の値で止めず、本体の傾き（0.5/%）で下がる
    i10, i14 = np.argmin(np.abs(g + 9.75)), np.argmin(np.abs(g + 14.25))
    assert sa[1, i14] == pytest.approx(sa[1, i10] - 0.5 * 4.5, abs=0.2)
    assert kind[1, i14] == POINT_OUT and kind[1, i10] == POINT_MEASURED
    assert not np.any(kind[1, (g >= -9.0) & (g <= 0.0)] == POINT_OUT)  # 範囲内は範囲外にしない
    assert t2.meta["stop_speed_rows"] == [1.25, 3.75]
    assert t2.meta["stop_max_speed_kmh"] == pytest.approx(5.0)
    assert t2.meta["stop_u_step_pct"] == 0.25
    assert np.array_equal(t2.accel_grid, table.accel_grid)


def test_launch_and_stop_flags_select_tables_and_both_is_error() -> None:
    table, rows = _launch_setup()
    t2 = attach_stop(attach_launch(table, rows, creep_speed_kmh=5.0), _stop_rows(),
                     creep_speed_kmh=5.0)
    v, u = 1.25, -3.0
    body = float(t2.accel(v, u))
    assert body == pytest.approx(-1.5)
    assert float(t2.accel(v, u, launch=True)) == pytest.approx(1.2, abs=0.1)
    assert float(t2.accel(v, u, stop=True)) == pytest.approx(-0.2 + 0.15 * u, abs=0.1)
    assert float(t2.coast_accel(v, stop=True)) == pytest.approx(0.8, abs=0.1)
    uu, kind = t2.invert(v, -0.65, stop=True)
    assert float(uu) == pytest.approx(-3.0, abs=0.3)
    with pytest.raises(ValueError):
        t2.accel(v, u, launch=True, stop=True)
    with pytest.raises(ValueError):
        t2.invert(v, 0.0, launch=True, stop=True)
    # 停止側なしの表では stop=True でも本体
    assert float(table.accel(v, u, stop=True)) == pytest.approx(float(table.accel(v, u)))


def test_launch_and_stop_survive_pkl_and_old_pkl_has_none(tmp_path: Path) -> None:
    table, rows = _launch_setup()
    t2 = attach_stop(attach_launch(table, rows, creep_speed_kmh=5.0), _stop_rows(),
                     creep_speed_kmh=5.0)
    t3 = load_forward_map(save_table(t2, tmp_path / "l.pkl"))
    assert t3.launch_accel is not None and np.allclose(t3.launch_accel, t2.launch_accel)
    assert t3.stop_accel is not None and np.allclose(t3.stop_accel, t2.stop_accel)
    assert t3.stop_kind is not None and np.array_equal(t3.stop_kind, t2.stop_kind)
    p = save_table(t2, tmp_path / "old.pkl")
    with p.open("rb") as f:
        d = pickle.load(f)
    for k in ("launch_accel", "stop_accel", "launch_kind", "stop_kind"):
        del d[k]
    with p.open("wb") as f:
        pickle.dump(d, f)
    old = load_forward_map(p)
    assert old.launch_accel is None and old.stop_accel is None and old.stop_kind is None


# ───────── 2026-10-14 段2o-5: 戻しの表 ─────────

REL_SLOPE = 0.05  # 戻し 1% あたり、u < 0 の側の加速度が |u| に比例して上がる量（ブレーキが弱まる）


def release_table(max_pct: float = 8.0, with_release: bool = True) -> ForwardTable:
    """a = −2 − 0.01v + (u>=0: 0.3u / u<0: 0.6u)。戻しの表: u<0 の列に REL_SLOPE×戻し量×|u| を足す
    （u=0 では本体と同じ。単調のまま）。"""
    rep = np.array([0.0, 50.0, 100.0])
    grid = np.arange(-20.0, 21.0, 0.5)
    base = -2.0 - 0.01 * rep[:, None] + np.where(grid >= 0, 0.3 * grid, 0.6 * grid)[None, :]
    kind = np.full(base.shape, POINT_MEASURED, dtype=int)
    tb = ForwardTable(
        speed_edges=np.array([-25.0, 25.0, 75.0, 125.0]),
        speed_rep=rep,
        u_grid=grid,
        accel_grid=base,
        seconds=np.full(base.shape, 10.0),
        point_kind=kind,
        meas_u_min=np.full(3, -20.0),
        meas_u_max=np.full(3, 20.0),
    )
    if not with_release:
        return tb
    slices = np.stack(
        [base + np.where(grid < 0, REL_SLOPE * r * -grid, 0.0)[None, :] for r in RELEASE_GRID_PCT]
    )
    rel = np.full(1000, 1.0)  # 戻し量 1% の行が 1000 行 × 0.05 s = 50 秒
    return attach_release(tb, slices, release_pct=rel, dt_row_s=np.full(1000, 0.05))


def test_release_none_or_zero_is_identical_to_main() -> None:
    tb = release_table()
    v = np.array([10.0, 40.0, 70.0])
    u = np.array([-8.0, -3.0, 5.0])
    base = tb.accel(v, u)
    assert np.array_equal(tb.accel(v, u, release=None), base)
    assert np.array_equal(tb.accel(v, u, release=0.0), base)
    a = np.array([-4.0, -3.0, 0.5])
    assert np.array_equal(tb.invert(v, a, release=0.0)[0], tb.invert(v, a)[0])
    # 戻しの表が無い表は release を渡しても変わらない
    plain = release_table(with_release=False)
    assert plain.release_grid is None
    assert np.array_equal(plain.accel(v, u, release=3.0), plain.accel(v, u))
    assert np.array_equal(plain.invert(v, a, release=3.0)[0], plain.invert(v, a)[0])


def test_release_interpolates_linearly_between_slices() -> None:
    tb = release_table()
    tb.meta["release_max_pct"] = 8.0  # 頭打ちを格子の端まで広げる（頭打ちは別のテスト）
    for r in (0.25, 0.5, 1.0, 1.5, 3.0):
        got = tb.accel(30.0, -6.0, release=r)
        assert got == pytest.approx(tb.accel(30.0, -6.0) + REL_SLOPE * r * 6.0, abs=1e-9)
    # u > 0 は戻し量に依らない
    assert tb.accel(30.0, 5.0, release=2.0) == pytest.approx(tb.accel(30.0, 5.0))
    # 配列の戻し量（サンプルごと）
    rr = np.array([0.0, 1.0, 2.0])
    got = tb.accel(np.full(3, 30.0), np.full(3, -6.0), release=rr)
    assert np.allclose(got, tb.accel(30.0, -6.0) + REL_SLOPE * rr * 6.0)


def test_release_is_clamped_to_release_max_pct() -> None:
    tb = release_table()
    assert tb.meta["release_max_pct"] == pytest.approx(1.0)
    at_max = tb.accel(30.0, -6.0, release=1.0)
    assert tb.accel(30.0, -6.0, release=5.0) == pytest.approx(at_max)  # 外挿しない
    u_max, _ = tb.invert(30.0, -3.0, release=1.0)
    u_big, _ = tb.invert(30.0, -3.0, release=7.0)
    assert u_big == pytest.approx(u_max)


def test_invert_with_release_is_inverse_of_accel_per_sample() -> None:
    tb = release_table()
    tb.meta["release_max_pct"] = 8.0
    v = np.array([20.0, 30.0, 60.0, 90.0])
    u = np.array([-12.0, -6.0, -2.5, -9.0])
    r = np.array([0.0, 1.0, 2.5, 6.0])
    a = tb.accel(v, u, release=r)
    back, flag = tb.invert(v, a, release=r)
    assert np.allclose(back, u, atol=1e-6)
    assert np.all(flag == POINT_MEASURED)


def test_launch_table_ignores_release() -> None:
    tb = release_table()
    tb.launch_accel = tb.accel_grid + 1.0
    assert tb.accel(30.0, -6.0, launch=True, release=2.0) == pytest.approx(
        tb.accel(30.0, -6.0, launch=True)
    )
    assert tb.accel(30.0, -6.0, release=2.0) != pytest.approx(tb.accel(30.0, -6.0))


def test_attach_release_needs_enough_release_seconds() -> None:
    tb = release_table(with_release=False)
    slices = np.stack([tb.accel_grid] * len(RELEASE_GRID_PCT))
    few = attach_release(tb, slices, release_pct=np.full(100, 1.0), dt_row_s=np.full(100, 0.05))
    assert few.release_grid is None and few.release_accel is None  # 5 秒 < 10 秒
    assert few.meta["release_rows_s"] == pytest.approx(5.0)
    # 戻し量が 0.3% 未満の行は秒数に数えない
    tiny = attach_release(tb, slices, release_pct=np.full(1000, 0.1), dt_row_s=np.full(1000, 0.05))
    assert tiny.release_grid is None
    rel = np.concatenate([np.full(400, 1.0), np.zeros(100), np.linspace(0.0, 4.0, 100)])
    ok = attach_release(tb, slices, release_pct=rel, dt_row_s=np.full(len(rel), 0.05))
    assert ok.release_grid is not None and ok.release_accel.shape == slices.shape
    assert ok.meta["release_max_pct"] == pytest.approx(float(np.quantile(rel, 0.95)))
    with pytest.raises(ValueError):
        attach_release(tb, slices[:2], release_pct=rel, dt_row_s=np.full(len(rel), 0.05))


def test_release_pkl_roundtrip_and_old_pkl_loads(tmp_path: Path) -> None:
    tb = release_table()
    back = load_forward_map(save_table(tb, tmp_path / "r.pkl"))
    assert np.array_equal(back.release_grid, tb.release_grid)
    assert np.array_equal(back.release_accel, tb.release_accel)
    assert back.accel(30.0, -6.0, release=1.0) == pytest.approx(tb.accel(30.0, -6.0, release=1.0))
    # 古い pkl（キーが無い）は None で、今までと同じに読める
    with (tmp_path / "r.pkl").open("rb") as f:
        d = pickle.load(f)
    del d["release_grid"], d["release_accel"]
    old_path = tmp_path / "old.pkl"
    with old_path.open("wb") as f:
        pickle.dump(d, f)
    old = load_forward_map(old_path)
    assert old.release_grid is None and old.release_accel is None
    assert old.accel(30.0, -6.0, release=1.0) == pytest.approx(old.accel(30.0, -6.0))


# ───────── 2026-10-20 段4-3: アクセルの戻しの表 ─────────

AREL_SLOPE = 0.04  # アクセルを 1% 戻した直後、u > 0 の側の加速度が u に比例して下がる量


def accel_release_table(with_release: bool = True) -> ForwardTable:
    """release_table（ブレーキの戻しの表あり）に、アクセルの戻しの表を足す。
    アクセルの戻しの層: u > 0 の列から AREL_SLOPE × 戻し量 × u を引く
    （u = 0 では本体と同じ。単調のまま）。"""
    tb = release_table()
    if not with_release:
        return tb
    grid = tb.u_grid
    slices = np.stack(
        [
            tb.accel_grid - np.where(grid > 0, AREL_SLOPE * r * grid, 0.0)[None, :]
            for r in RELEASE_GRID_PCT
        ]
    )
    rel = np.full(1000, 1.0)  # アクセルの戻し量 1% の行が 50 秒
    return attach_accel_release(tb, slices, release_pct=rel, dt_row_s=np.full(1000, 0.05))


def test_accel_release_none_or_zero_is_identical_to_main() -> None:
    tb = accel_release_table()
    v = np.array([10.0, 40.0, 70.0])
    u = np.array([-8.0, 3.0, 5.0])
    base = tb.accel(v, u)
    assert np.array_equal(tb.accel(v, u, accel_release=None), base)
    assert np.array_equal(tb.accel(v, u, accel_release=0.0), base)
    a = np.array([-4.0, 1.0, 0.5])
    assert np.array_equal(tb.invert(v, a, accel_release=0.0)[0], tb.invert(v, a)[0])
    # アクセルの戻しの表が無い表は accel_release を渡しても変わらない
    plain = accel_release_table(with_release=False)
    assert plain.accel_release_grid is None
    assert np.array_equal(plain.accel(v, u, accel_release=3.0), plain.accel(v, u))
    assert np.array_equal(plain.invert(v, a, accel_release=3.0)[0], plain.invert(v, a)[0])


def test_accel_release_interpolates_linearly_and_only_for_positive_u() -> None:
    tb = accel_release_table()
    tb.meta["accel_release_max_pct"] = 8.0
    for r in (0.25, 0.5, 1.0, 1.5, 3.0):
        got = tb.accel(30.0, 6.0, accel_release=r)
        assert got == pytest.approx(tb.accel(30.0, 6.0) - AREL_SLOPE * r * 6.0, abs=1e-9)
    # u < 0 はアクセルの戻し量に依らない
    assert tb.accel(30.0, -5.0, accel_release=2.0) == pytest.approx(tb.accel(30.0, -5.0))
    rr = np.array([0.0, 1.0, 2.0])
    got = tb.accel(np.full(3, 30.0), np.full(3, 6.0), accel_release=rr)
    assert np.allclose(got, tb.accel(30.0, 6.0) - AREL_SLOPE * rr * 6.0)
    # ブレーキの戻し（release）と同時に渡すと u の符号で使い分ける
    tb.meta["release_max_pct"] = 8.0
    both = tb.accel(np.full(2, 30.0), np.array([6.0, -6.0]), release=1.0, accel_release=1.0)
    assert both[0] == pytest.approx(tb.accel(30.0, 6.0, accel_release=1.0))
    assert both[1] == pytest.approx(tb.accel(30.0, -6.0, release=1.0))


def test_accel_release_is_clamped_and_ignored_by_side_tables() -> None:
    tb = accel_release_table()
    assert tb.meta["accel_release_max_pct"] == pytest.approx(1.0)
    at_max = tb.accel(30.0, 6.0, accel_release=1.0)
    assert tb.accel(30.0, 6.0, accel_release=5.0) == pytest.approx(at_max)  # 外挿しない
    tb.launch_accel = tb.accel_grid + 1.0
    assert tb.accel(30.0, 6.0, launch=True, accel_release=1.0) == pytest.approx(
        tb.accel(30.0, 6.0, launch=True)
    )
    tb.stop_accel = tb.accel_grid - 1.0
    assert tb.accel(30.0, 6.0, stop=True, accel_release=1.0) == pytest.approx(
        tb.accel(30.0, 6.0, stop=True)
    )


def test_invert_with_accel_release_is_inverse_of_accel_per_sample() -> None:
    tb = accel_release_table()
    tb.meta["accel_release_max_pct"] = 8.0
    v = np.array([20.0, 30.0, 60.0, 90.0])
    u = np.array([12.0, 6.0, 2.5, 9.0])
    r = np.array([0.0, 1.0, 2.5, 6.0])
    a = tb.accel(v, u, accel_release=r)
    back, flag = tb.invert(v, a, accel_release=r)
    assert np.allclose(back, u, atol=1e-6)
    assert np.all(flag == POINT_MEASURED)
    # ブレーキ側の u をブレーキの戻し量と一緒に解いても、アクセルの戻し量は影響しない
    ub = np.array([-12.0, -6.0, -2.5, -9.0])
    rb = np.array([0.0, 1.0, 2.5, 6.0])
    tb.meta["release_max_pct"] = 8.0
    ab = tb.accel(v, ub, release=rb)
    back_b, _ = tb.invert(v, ab, release=rb, accel_release=np.full(4, 3.0))
    assert np.allclose(back_b, ub, atol=1e-6)


def test_attach_accel_release_needs_enough_seconds_and_pkl_roundtrip(tmp_path: Path) -> None:
    tb = release_table(with_release=False)
    slices = np.stack([tb.accel_grid] * len(RELEASE_GRID_PCT))
    few = attach_accel_release(
        tb, slices, release_pct=np.full(100, 1.0), dt_row_s=np.full(100, 0.05)
    )
    assert few.accel_release_grid is None and few.accel_release_accel is None
    assert few.meta["accel_release_rows_s"] == pytest.approx(5.0)
    rel = np.concatenate([np.full(400, 1.0), np.zeros(100), np.linspace(0.0, 4.0, 100)])
    ok = attach_accel_release(tb, slices, release_pct=rel, dt_row_s=np.full(len(rel), 0.05))
    assert ok.accel_release_grid is not None
    assert ok.meta["accel_release_max_pct"] == pytest.approx(float(np.quantile(rel, 0.95)))
    with pytest.raises(ValueError):
        attach_accel_release(tb, slices[:2], release_pct=rel, dt_row_s=np.full(len(rel), 0.05))
    full = accel_release_table()
    back = load_forward_map(save_table(full, tmp_path / "a.pkl"))
    assert np.array_equal(back.accel_release_accel, full.accel_release_accel)
    assert back.accel(30.0, 6.0, accel_release=1.0) == pytest.approx(
        full.accel(30.0, 6.0, accel_release=1.0)
    )
    with (tmp_path / "a.pkl").open("rb") as f:
        d = pickle.load(f)
    for k in (
        "accel_release_grid", "accel_release_accel", "stop_release_grid", "stop_release_accel",
    ):
        del d[k]
    with (tmp_path / "old.pkl").open("wb") as f:
        pickle.dump(d, f)
    old = load_forward_map(tmp_path / "old.pkl")
    assert old.accel_release_grid is None and old.stop_release_accel is None
    assert old.accel(30.0, 6.0, accel_release=1.0) == pytest.approx(old.accel(30.0, 6.0))


# ───────── 2026-10-20 段4-3: 停止側の戻しの層 ─────────

STOP_REL_SLOPE = 0.02  # 停止側: ブレーキの戻し 1% あたり、u < 0 の側が |u| に比例して上がる量


def _stop_release_rows():
    """停止側のブレーキの行: 戻し量 0 / 1 / 4 % が各 200 行（0.1 秒刻み）。
    a = −0.2 + 0.15u + STOP_REL_SLOPE × 戻し量 × |u|。"""
    rng = np.random.default_rng(3)
    parts = []
    for r in (0.0, 1.0, 4.0):
        n = 200
        v = rng.uniform(0.6, 4.9, n)
        u = -rng.choice(np.arange(1.0, 10.01, 0.5), n)
        a = -0.2 + 0.15 * u + STOP_REL_SLOPE * r * -u
        rows = make_rows(v, u, a, 0.1)
        parts.append(FwdRows(**{**rows.__dict__, "release_pct": np.full(n, r)}))
    return concat_rows(parts, "stoprel")


def _stop_release_table():
    table, _ = _launch_setup()
    base = attach_stop(table, _stop_rows(), creep_speed_kmh=5.0)
    return base, attach_stop_release(base, _stop_release_rows(), creep_speed_kmh=5.0)


def test_release_bin_edges_are_midpoints_between_nodes() -> None:
    lo, hi = release_bin_edges(RELEASE_GRID_PCT)  # (0.5, 1, 2, 4, 8)
    assert lo.tolist() == [0.25, 0.75, 1.5, 3.0, 6.0]
    assert hi[:-1].tolist() == lo[1:].tolist() and np.isinf(hi[-1])


def test_stop_release_layers_fall_back_to_the_lower_layer() -> None:
    base, t2 = _stop_release_table()
    assert t2.stop_release_accel is not None and t2.stop_release_grid is not None
    assert t2.stop_release_accel.shape == (len(RELEASE_GRID_PCT), *base.accel_grid.shape)
    # 層ごとの秒数（0.1 秒 × 行数）: 戻し量 1% → 節 1.0 に 20 秒、4% → 節 4.0 に 20 秒。
    # 戻し量 0% の行はどの層にも入らない
    assert t2.meta["stop_release_layer_s"] == pytest.approx([0.0, 20.0, 0.0, 20.0, 0.0])
    assert t2.meta["stop_release_layer_built"] == [False, True, False, True, False]
    layers = t2.stop_release_accel
    assert np.array_equal(layers[0], base.stop_accel)  # 行が無い層は一つ下（stop_accel）のまま
    assert np.array_equal(layers[2], layers[1]) and np.array_equal(layers[4], layers[3])
    g = base.u_grid
    sel = (g <= -1.0) & (g >= -9.0)
    for k, r in ((1, 1.0), (3, 4.0)):
        want = -0.2 + 0.15 * g[sel] + STOP_REL_SLOPE * r * -g[sel]
        assert np.allclose(layers[k][1, sel], want, atol=0.1)
    assert t2.meta["stop_release_max_pct"] == pytest.approx(
        float(np.quantile(np.repeat([0.0, 1.0, 4.0], 200), 0.95))
    )
    # 本体・停止側は書き換えない
    assert np.array_equal(t2.stop_accel, base.stop_accel)
    assert np.array_equal(t2.accel_grid, base.accel_grid)


def test_stop_release_is_used_only_by_stop_lookups_and_none_is_identical() -> None:
    base, t2 = _stop_release_table()
    v, u = 1.25, -6.0
    t2.meta["stop_release_max_pct"] = 8.0
    a1 = float(t2.accel(v, u, stop=True, release=1.0))
    assert a1 == pytest.approx(-0.2 + 0.15 * u + STOP_REL_SLOPE * 1.0 * 6.0, abs=0.1)
    # 戻し量 2.5% は節 2.0（＝節 1.0 と同じ層）と節 4.0 の間 → 節 1.0 の値より大きい
    a25 = float(t2.accel(v, u, stop=True, release=2.5))
    assert a25 > a1
    # release を渡さない・0 は stop_accel のまま
    assert float(t2.accel(v, u, stop=True)) == pytest.approx(float(base.accel(v, u, stop=True)))
    assert float(t2.accel(v, u, stop=True, release=0.0)) == pytest.approx(
        float(base.accel(v, u, stop=True))
    )
    # 本体の引き（stop=False）は停止側の層に影響されない
    assert float(t2.accel(v, u, release=1.0)) == pytest.approx(float(base.accel(v, u, release=1.0)))
    # 層なしの表は release を渡しても stop=True の値は変わらない
    assert float(base.accel(v, u, stop=True, release=3.0)) == pytest.approx(
        float(base.accel(v, u, stop=True))
    )
    # 逆引き: 層の値から元の u に戻る。層なしは release を無視する
    back, _ = t2.invert(v, a1, stop=True, release=1.0)
    assert float(back) == pytest.approx(u, abs=0.3)
    nolayer, _ = base.invert(v, a1, stop=True, release=1.0)
    assert float(nolayer) == pytest.approx(float(base.invert(v, a1, stop=True)[0]))


def test_stop_release_is_capped_by_its_own_max_pct() -> None:
    _, t2 = _stop_release_table()
    t2.meta["stop_release_max_pct"] = 1.0
    at_max = float(t2.accel(1.25, -6.0, stop=True, release=1.0))
    assert float(t2.accel(1.25, -6.0, stop=True, release=7.0)) == pytest.approx(at_max)
    # stop 用の頭打ちが無ければ release_max_pct を使う
    del t2.meta["stop_release_max_pct"]
    t2.meta["release_max_pct"] = 1.0
    assert float(t2.accel(1.25, -6.0, stop=True, release=7.0)) == pytest.approx(at_max)


def test_attach_stop_release_not_built_without_stop_table_or_rows(tmp_path: Path) -> None:
    table, _ = _launch_setup()
    none = attach_stop_release(table, _stop_release_rows(), creep_speed_kmh=5.0)  # 停止側なし
    assert none.stop_release_accel is None and none.stop_release_grid is None
    base = attach_stop(table, _stop_rows(), creep_speed_kmh=5.0)
    rows = _stop_release_rows()
    # 戻し量 0 の行だけ（どの層にも入らない）→ 作らない
    only0 = FwdRows(**{**rows.__dict__, "release_pct": np.zeros(len(rows))})
    t0 = attach_stop_release(base, only0, creep_speed_kmh=5.0)
    assert t0.stop_release_accel is None
    assert t0.meta["stop_release_layer_built"] == [False] * 5
    # 動いていない行は使わない
    stopped = FwdRows(**{**rows.__dict__, "speed_kmh": np.full(len(rows), 0.1)})
    assert attach_stop_release(base, stopped, creep_speed_kmh=5.0).stop_release_accel is None
    # pkl を往復しても残る
    t2 = attach_stop_release(base, rows, creep_speed_kmh=5.0)
    back = load_forward_map(save_table(t2, tmp_path / "s.pkl"))
    assert np.array_equal(back.stop_release_accel, t2.stop_release_accel)
    assert np.array_equal(back.stop_release_grid, t2.stop_release_grid)


def test_release_seconds_in_band_counts_braking_release_rows() -> None:
    v = np.array([5.0, 8.0, 12.0, 25.0, 12.0, 12.0])
    u = np.array([-3.0, -3.0, -3.0, -3.0, 2.0, -3.0])
    base = make_rows(v, u, np.zeros(6), 0.1)
    r = FwdRows(**{**base.__dict__, "release_pct": np.array([1.0, 1.0, 1.0, 1.0, 1.0, 0.1])})
    # 7.5〜20 km/h・u < 0・戻し量 0.3% 以上: 添字 1・2 だけ
    assert release_seconds_in_band(r, 7.5) == pytest.approx(0.2)


# ───────── 2026-10-22 段5-3: 発進側のアクセル側を実測の点で作る ─────────


def _accel_rows(speed: float, us, accel_of, dt: float = 0.1, n_each: int = 10) -> FwdRows:
    u = np.repeat(np.asarray(us, dtype=float), n_each)
    return make_rows(np.full(len(u), speed), u, accel_of(u), dt=dt)


def test_measured_launch_accel_uses_cell_medians_and_keeps_brake_side() -> None:
    table, rows = _launch_setup()  # u の格子は −16〜+3.75（0.5 刻み）
    t1 = attach_launch(table, rows, creep_speed_kmh=5.0)
    pat = _accel_rows(3.0, [0.5, 1.5, 2.5], lambda u: 2.0 + 0.2 * u)  # 1% のマスに 1 秒ずつ
    t2 = apply_launch_accel_measured(t1, pat, cell_pct=1.0)
    assert t2.meta["launch_accel_side_source"] == "measured"
    pts = t2.meta["launch_accel_measured"]["3.75"]
    assert [round(p[0], 3) for p in pts] == [0.5, 1.5, 2.5]
    assert pts[1][1] == pytest.approx(2.3) and pts[1][2] == pytest.approx(1.0)
    la, g = t2.launch_accel, t2.u_grid
    r = 1  # 2.5〜5 km/h の行
    creep = float(np.interp(0.0, g, t1.launch_accel[r]))
    assert float(np.interp(1.5, g, la[r])) == pytest.approx(2.3, abs=1e-6)  # 点の位置
    # 浅い所は u = 0 のクリープから最初の点へ直線
    assert float(np.interp(0.25, g, la[r])) == pytest.approx(creep + 0.5 * (2.1 - creep), abs=1e-6)
    # いちばん深い点（u=2.5）より深い所は「点の値 ＋ 本体の差」
    body = t1.accel_grid[r]
    want = 2.5 + (float(np.interp(3.25, g, body)) - float(np.interp(2.5, g, body)))
    assert float(np.interp(3.25, g, la[r])) == pytest.approx(want, abs=1e-6)
    # ブレーキ側・他の行は変えない。アクセル側は単調
    neg = g <= 0.0
    assert np.array_equal(la[r, neg], t1.launch_accel[r, neg])
    assert np.array_equal(la[0, neg], t1.launch_accel[0, neg])
    assert np.array_equal(la[2:], t1.launch_accel[2:])
    assert np.all(np.diff(la[r]) >= -1e-9)
    assert t1.meta.get("launch_accel_side_source") is None  # 元は書き換えない


def test_measured_launch_accel_low_row_is_max_of_creep_and_body() -> None:
    table, rows = _launch_setup()
    t1 = attach_launch(table, rows, creep_speed_kmh=5.0)
    steep = t1.accel_grid.copy()
    steep[0] = np.where(t1.u_grid > 0, 0.8 * t1.u_grid, steep[0])  # 本体が深い側でクリープを超える
    t1 = replace(t1, accel_grid=steep)
    t2 = apply_launch_accel_measured(t1, _accel_rows(3.0, [0.5, 1.5], lambda u: 2.0 + 0.2 * u))
    g = t1.u_grid
    pos = g > 0
    creep = float(np.interp(0.0, g, t1.launch_accel[0]))
    want = np.maximum(creep, steep[0][pos])
    assert np.allclose(t2.launch_accel[0][pos], np.maximum.accumulate(want))
    assert t2.launch_accel[0][pos][0] == pytest.approx(creep)  # 浅い所はクリープ
    assert t2.launch_accel[0][pos].max() > creep + 1.0  # 深い所は本体の増え
    neg = g <= 0.0
    assert np.array_equal(t2.launch_accel[0][neg], t1.launch_accel[0][neg])


def test_measured_launch_accel_row_without_points_stays_and_short_cells_dropped() -> None:
    table, rows = _launch_setup()
    t1 = attach_launch(table, rows, creep_speed_kmh=5.0)
    # 0.3 秒しかないマスは捨てる。点が 1 つも無い行は今のまま
    short = _accel_rows(3.0, [1.0, 3.0], lambda u: 2.0 + 0.2 * u, n_each=3)
    t2 = apply_launch_accel_measured(t1, short)
    assert t2.meta["launch_accel_measured"]["3.75"] == []
    assert np.array_equal(t2.launch_accel[1], t1.launch_accel[1])
    # 車速が帯の外・u <= 0 の行は点にならない
    out = _accel_rows(8.0, [1.0, 3.0], lambda u: 2.0 + 0.2 * u)
    assert apply_launch_accel_measured(t1, out).meta["launch_accel_measured"]["3.75"] == []
    brake = _accel_rows(3.0, [-1.0, -3.0], lambda u: 2.0 + 0.2 * u)
    assert apply_launch_accel_measured(t1, brake).meta["launch_accel_measured"]["3.75"] == []


def test_measured_launch_accel_without_launch_table_changes_nothing() -> None:
    table, _ = _launch_setup()
    t2 = apply_launch_accel_measured(table, _accel_rows(3.0, [1.0], lambda u: u))
    assert t2.launch_accel is None and t2.meta["launch_accel_side_source"] == "measured"


# ───────── 2026-10-22 段5-4: 発進側を停止側の車速の行まで広げる（b）・行の代表車速（e） ─────────


def _wide_stop_rows(v_lo: float = 0.6, v_hi: float = 7.4, seed: int = 3):
    """停止側の行: v_lo〜v_hi（5〜7.5 km/h の行にも届く）、a = −0.2 + 0.15u、u = 0 は 0.8。"""
    rng = np.random.default_rng(seed)
    n = 800
    v = rng.uniform(v_lo, v_hi, n)
    u = -rng.choice(np.arange(1.0, 10.01, 0.5), n)
    v = np.concatenate([v, rng.uniform(v_lo, v_hi, 150)])
    u = np.concatenate([u, np.zeros(150)])
    return make_rows(v, u, np.where(u == 0.0, 0.8, -0.2 + 0.15 * u), 0.1)


def _both_sides():
    table, lrows = _launch_setup()  # 発進側の行は v < 4.9 → [1.25, 3.75]
    t = attach_launch(table, lrows, creep_speed_kmh=5.0)
    t = attach_stop(t, _wide_stop_rows(), creep_speed_kmh=5.0)  # 停止側は [1.25, 3.75, 6.25]
    return t


def test_extend_launch_rows_copies_stop_brake_side_and_builds_accel_side() -> None:
    t1 = _both_sides()
    assert t1.meta["launch_speed_rows"] == [1.25, 3.75]
    assert t1.meta["stop_speed_rows"] == [1.25, 3.75, 6.25]
    t2 = extend_launch_rows_to_stop_max(t1)
    assert t2.meta["launch_speed_rows"] == [1.25, 3.75, 6.25]
    assert t2.meta["launch_rows_to_stop_max"] is True
    assert t2.meta["launch_rows_to_stop_max_rows"] == [6.25]
    g = t1.u_grid
    neg, pos = g <= 0.0, g > 0.0
    r = 2
    assert np.array_equal(t2.launch_accel[r, neg], t1.stop_accel[r, neg])
    u0 = float(np.interp(0.0, g, t1.stop_accel[r]))
    # アクセル側 ＝ 停止側の開度 0 ＋ 本体の増え（本体は a = 0.3u なので傾きは 0.3）
    assert float(np.interp(3.0, g, t2.launch_accel[r])) == pytest.approx(u0 + 0.9, abs=1e-6)
    assert np.all(np.diff(t2.launch_accel[r]) >= -1e-9)
    assert t2.launch_accel[r, pos].min() >= t2.launch_accel[r, neg].max() - 1e-9
    # 5〜7.5 km/h の開度 0 が、停止側とそろう（広げる前は本体 0.0 ↔ 停止側 0.8 の食い違い）
    assert float(t2.accel(6.25, 0.0, launch=True)) == pytest.approx(
        float(t2.accel(6.25, 0.0, stop=True)))
    assert float(t1.accel(6.25, 0.0, launch=True)) != pytest.approx(
        float(t1.accel(6.25, 0.0, stop=True)))
    # 他の行・本体・停止側は変えない。元の表も書き換えない
    assert np.array_equal(t2.launch_accel[[0, 1, 3]], t1.launch_accel[[0, 1, 3]])
    assert np.array_equal(t2.accel_grid, t1.accel_grid)
    assert np.array_equal(t2.stop_accel, t1.stop_accel)
    assert t1.meta["launch_speed_rows"] == [1.25, 3.75]


def test_extend_launch_rows_then_measured_builds_accel_side_from_points() -> None:
    t2 = extend_launch_rows_to_stop_max(_both_sides())
    pat = _accel_rows(6.0, [1.5, 3.5], lambda u: 2.0 + 0.2 * u, n_each=10)
    t3 = apply_launch_accel_measured(t2, pat, cell_pct=1.0)
    assert [round(p[0], 3) for p in t3.meta["launch_accel_measured"]["6.25"]] == [1.5, 3.5]
    assert float(np.interp(2.5, t3.u_grid, t3.launch_accel[2])) == pytest.approx(2.5, abs=1e-6)
    neg = t3.u_grid <= 0.0
    assert np.array_equal(t3.launch_accel[2, neg], t3.stop_accel[2, neg])


def test_extend_launch_rows_is_noop_without_both_tables_or_extra_rows() -> None:
    table, lrows = _launch_setup()
    only_launch = attach_launch(table, lrows, creep_speed_kmh=5.0)
    t = extend_launch_rows_to_stop_max(only_launch)
    assert t.launch_accel is only_launch.launch_accel and t.meta["launch_rows_to_stop_max"] is True
    assert t.meta["launch_rows_to_stop_max_rows"] == []
    both = attach_stop(only_launch, _stop_rows(), creep_speed_kmh=5.0)  # 停止側も [1.25, 3.75]
    t = extend_launch_rows_to_stop_max(both)
    assert np.array_equal(t.launch_accel, both.launch_accel)
    assert t.meta["launch_rows_to_stop_max_rows"] == []
    assert extend_launch_rows_to_stop_max(table).launch_accel is None


def _rep_rows(v_value: float, n: int = 100, spread: float = 0.0):
    rng = np.random.default_rng(4)
    v = v_value + spread * rng.uniform(-1.0, 1.0, n)
    u = -rng.choice(np.arange(1.0, 10.01, 0.5), n)
    return make_rows(v, u, -0.2 + 0.15 * u, 0.1)


def test_side_row_speed_rep_uses_data_median_and_keeps_others() -> None:
    t1 = extend_launch_rows_to_stop_max(_both_sides())
    stop_rows = concat_rows([_rep_rows(2.0, spread=0.3), _rep_rows(4.0, spread=0.3),
                             _rep_rows(5.8, spread=0.3)], "stop")
    launch_rows = concat_rows([_rep_rows(2.2, spread=0.3), _rep_rows(4.2, spread=0.3)], "launch")
    t2 = apply_side_row_speed_rep(t1, launch_rows, stop_rows)
    assert t2.meta["side_row_speed_rep"] == "data"
    assert t2.stop_speed_rep[:3] == pytest.approx([2.0, 4.0, 5.8], abs=0.1)
    # 発進側: 元から置き換えた行は発進側のデータ、広げた行（6.25）は停止側の中央値
    assert t2.launch_speed_rep[:3] == pytest.approx([2.2, 4.2, t2.stop_speed_rep[2]], abs=0.1)
    # 置き換えていない行（7.5 km/h 以上）は本体と同じ
    assert t2.stop_speed_rep[3] == t1.speed_rep[3] and t2.launch_speed_rep[3] == t1.speed_rep[3]
    assert np.all(np.diff(t2.stop_speed_rep) > 0) and np.all(np.diff(t2.launch_speed_rep) > 0)
    assert t2.meta["stop_speed_rep_map"]["6.25"] == pytest.approx(5.8, abs=0.1)
    assert t1.stop_speed_rep is None  # 元は書き換えない


def test_side_row_speed_rep_reverts_rows_too_close_to_neighbour() -> None:
    t1 = _both_sides()
    # 2.5〜5 の行が 4.97、5〜7.5 の行が 5.0 → 近すぎるので下の行を中心 (3.75) に戻す
    stop_rows = concat_rows([_rep_rows(2.0), _rep_rows(4.97), _rep_rows(5.0)], "stop")
    t2 = apply_side_row_speed_rep(t1, concat_rows([_rep_rows(2.0)], "launch"), stop_rows,
                                  min_gap_kmh=0.5)
    assert t2.stop_speed_rep[:3] == pytest.approx([2.0, 3.75, 5.0])
    assert np.all(np.diff(t2.stop_speed_rep) > 0)


def test_side_row_speed_rep_changes_only_the_side_table_interpolation() -> None:
    t1 = _both_sides()
    t2 = replace(t1, stop_speed_rep=np.array([1.25, 3.75, 5.0, 8.75]))
    v = np.array([3.0, 4.5, 5.0, 5.6, 6.5])
    u = np.full(5, -4.0)
    # 本体・発進側は変わらない
    assert np.array_equal(t2.accel(v, u), t1.accel(v, u))
    assert np.array_equal(t2.accel(v, u, launch=True), t1.accel(v, u, launch=True))
    # 停止側は専用の代表車速で補間する: 行の値そのもの（代表車速ちょうど）を引ける
    assert float(t2.accel(5.0, -4.0, stop=True)) == pytest.approx(
        float(np.interp(-4.0, t1.u_grid, t1.stop_accel[2])))
    assert float(t2.coast_accel(5.0, stop=True)) == pytest.approx(
        float(np.interp(0.0, t1.u_grid, t1.stop_accel[2])))
    # invert も同じ代表車速で引く（accel の逆）
    a = float(t2.accel(5.4, -4.0, stop=True))
    u_back, _ = t2.invert(5.4, a, stop=True)
    assert float(u_back) == pytest.approx(-4.0, abs=0.3)
    # 既定（None）は今と完全に同じ
    t3 = replace(t1, stop_speed_rep=None, launch_speed_rep=None)
    assert np.array_equal(t3.accel(v, u, stop=True), t1.accel(v, u, stop=True))
    assert t1._rep(False, True) is t1.speed_rep and t1._rep(True) is t1.speed_rep


def test_side_row_speed_rep_roundtrips_through_pkl_and_old_pkl_is_none(tmp_path: Path) -> None:
    t1 = _both_sides()
    t2 = replace(t1, stop_speed_rep=np.array([1.5, 3.5, 5.0, 8.75]),
                 launch_speed_rep=np.array([1.5, 3.5, 6.25, 8.75]))
    back = load_forward_map(save_table(t2, tmp_path / "r.pkl"))
    assert np.array_equal(back.stop_speed_rep, t2.stop_speed_rep)
    assert np.array_equal(back.launch_speed_rep, t2.launch_speed_rep)
    plain = load_forward_map(save_table(t1, tmp_path / "p.pkl"))
    assert plain.stop_speed_rep is None and plain.launch_speed_rep is None
    # キーの無い古い pkl
    with (tmp_path / "p.pkl").open("rb") as f:
        d = pickle.load(f)
    d.pop("stop_speed_rep"), d.pop("launch_speed_rep")
    with (tmp_path / "old.pkl").open("wb") as f:
        pickle.dump(d, f)
    old = load_forward_map(tmp_path / "old.pkl")
    assert old.stop_speed_rep is None and old.launch_speed_rep is None


# ───────── 2026-10-23 段5-7b: アクセルの踏み増しの表 ─────────

APUSH_SLOPE = 0.05  # 踏み増し量 1% ごとに、u > 0 の側の加速度が u に比例して増える量


def accel_push_table(with_release: bool = False) -> ForwardTable:
    """（アクセルの戻しの表つきの）表に、アクセルの踏み増しの表を足す。層: u > 0 の列に
    APUSH_SLOPE × 踏み増し量 × u を足す（u = 0 では本体と同じ。単調のまま）。"""
    from tests.research.fwd_map import attach_accel_push  # noqa: PLC0415

    tb = accel_release_table() if with_release else release_table()
    grid = tb.u_grid
    slices = np.stack(
        [
            tb.accel_grid + np.where(grid > 0, APUSH_SLOPE * p * grid, 0.0)[None, :]
            for p in RELEASE_GRID_PCT
        ]
    )
    return attach_accel_push(tb, slices, push_pct=np.full(1000, 1.0), dt_row_s=np.full(1000, 0.05))


def test_accel_push_none_zero_or_no_table_is_identical_to_main() -> None:
    tb = accel_push_table()
    v = np.array([10.0, 40.0, 70.0])
    u = np.array([-8.0, 3.0, 5.0])
    base = tb.accel(v, u)
    assert np.array_equal(tb.accel(v, u, accel_push=None), base)
    assert np.array_equal(tb.accel(v, u, accel_push=0.0), base)
    a = np.array([-4.0, 1.0, 0.5])
    assert np.array_equal(tb.invert(v, a, accel_push=0.0)[0], tb.invert(v, a)[0])
    plain = release_table()
    assert plain.accel_push_grid is None
    assert np.array_equal(plain.accel(v, u, accel_push=3.0), plain.accel(v, u))
    assert np.array_equal(plain.invert(v, a, accel_push=3.0)[0], plain.invert(v, a)[0])


def test_accel_push_interpolates_clamps_and_only_for_positive_u() -> None:
    tb = accel_push_table()
    assert tb.meta["accel_push_max_pct"] == pytest.approx(1.0)
    tb.meta["accel_push_max_pct"] = 8.0
    for p in (0.25, 1.0, 3.0):
        got = tb.accel(30.0, 6.0, accel_push=p)
        assert got == pytest.approx(tb.accel(30.0, 6.0) + APUSH_SLOPE * p * 6.0, abs=1e-9)
    assert tb.accel(30.0, -5.0, accel_push=2.0) == pytest.approx(tb.accel(30.0, -5.0))
    tb.meta["accel_push_max_pct"] = 2.0
    assert tb.accel(30.0, 6.0, accel_push=5.0) == pytest.approx(tb.accel(30.0, 6.0, accel_push=2.0))
    tb.launch_accel = tb.accel_grid + 1.0
    assert tb.accel(30.0, 6.0, launch=True, accel_push=1.0) == pytest.approx(
        tb.accel(30.0, 6.0, launch=True)
    )


def test_accel_push_and_release_add_their_deltas_and_invert_is_inverse() -> None:
    tb = accel_push_table(with_release=True)
    tb.meta["accel_push_max_pct"] = 8.0
    tb.meta["accel_release_max_pct"] = 8.0
    base = tb.accel(30.0, 6.0)
    both = tb.accel(30.0, 6.0, accel_release=2.0, accel_push=3.0)
    d_rel = tb.accel(30.0, 6.0, accel_release=2.0) - base
    d_push = tb.accel(30.0, 6.0, accel_push=3.0) - base
    assert both == pytest.approx(base + d_rel + d_push)
    v = np.array([20.0, 30.0, 60.0, 90.0])
    u = np.array([12.0, 6.0, 2.5, 9.0])
    r = np.array([0.0, 1.0, 2.5, 6.0])
    p = np.array([2.0, 0.0, 1.5, 4.0])
    for kw in ({"accel_push": p}, {"accel_release": r, "accel_push": p}):
        a = tb.accel(v, u, **kw)
        back, flag = tb.invert(v, a, **kw)
        assert np.allclose(back, u, atol=1e-6)
        assert np.all(flag == POINT_MEASURED)


def test_accel_push_table_survives_save_and_old_pkl_has_none(tmp_path) -> None:
    from tests.research.fwd_map import load_forward_map, save_table  # noqa: PLC0415

    tb = accel_push_table()
    got = load_forward_map(save_table(tb, tmp_path / "p.pkl"))
    assert np.array_equal(got.accel_push_grid, tb.accel_push_grid)
    assert np.array_equal(got.accel_push_accel, tb.accel_push_accel)
    plain = load_forward_map(save_table(release_table(), tmp_path / "q.pkl"))
    assert plain.accel_push_grid is None and plain.accel_push_accel is None
