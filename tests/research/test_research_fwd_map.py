"""fwd_map（順モデルの表・逆引き・pkl）のテスト。合成データだけを使う。"""

from __future__ import annotations

import pickle
import time
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
    attach_launch,
    attach_release,
    attach_stop,
    build_table,
    load_forward_map,
    refine_u_grid,
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
