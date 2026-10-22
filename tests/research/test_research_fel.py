"""fel（フィードバック誤差学習の補正表）のテスト。合成データだけを使う（2026-10-19 段2s-2）。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import fel as felmod
from tests.research import mode_drive as mdmod
from tests.research.fwd_map import (
    POINT_MEASURED,
    ForwardTable,
    load_forward_map,
    save_table,
)


def _table(**extra) -> ForwardTable:
    rep = np.array([5.0, 15.0, 30.0, 60.0])
    grid = np.arange(-6.0, 6.01, 0.5)
    accel = (-0.5 - 0.01 * rep[:, None]) + 0.3 * grid[None, :]
    return ForwardTable(
        speed_edges=np.array([0.0, 10.0, 20.0, 40.0, 80.0]),
        speed_rep=rep,
        u_grid=grid,
        accel_grid=accel,
        seconds=np.full(accel.shape, 10.0),
        point_kind=np.full(accel.shape, POINT_MEASURED, dtype=int),
        meas_u_min=np.full(4, -6.0),
        meas_u_max=np.full(4, 6.0),
        meta={
            "accel_deadband_pct": 4.0, "brake_deadband_pct": 4.0,
            "accel_lag_s": 0.2, "brake_lag_s": 0.2, "stop_max_speed_kmh": 7.5,
        },
        **extra,
    )


def _rows(spec: list[tuple[float, float, int, int, float, int]]) -> felmod.FelRows:
    """spec ＝ [(v, u, pedal, rel, r, 行数), ...] から行を作る。"""
    v, u, pe, rl, r = [], [], [], [], []
    for sv, su, sp, sr, sx, n in spec:
        v += [sv] * n
        u += [su] * n
        pe += [sp] * n
        rl += [sr] * n
        r += [sx] * n
    m = len(v)
    z = np.zeros(m)
    return felmod.FelRows(
        r=np.array(r), pedal=np.array(pe), v=np.array(v), u=np.array(u), rel=np.array(rl),
        stop20=np.zeros(m, bool), acc=z, kind=np.zeros(m, dtype=int), release=z,
    )


PARAMS = felmod.FelParams(min_cell_s=2.0, c_max_kmhs=0.5)


def test_cell_recovers_known_correction_and_ignores_sparse_cells() -> None:
    rows = _rows([
        (30.0, 1.5, 1, 0, +0.2, 100),
        (30.0, 3.5, 1, 0, -0.2, 100),
        (30.0, 5.5, 1, 0, 0.0, 10),  # 0.5 秒 < min_cell_s
    ])
    model = felmod.fit(rows, PARAMS)
    c, found = model.cell_value(
        np.array([30.0, 30.0, 30.0]), np.array([1.5, 3.5, 5.5]), np.ones(3, int), np.zeros(3, int)
    )
    # offset（ペダル別の平均）を引いてから貯める: 平均は 0 に近い
    assert abs(model.offset[1]) < 0.02
    assert c[0] == pytest.approx(0.2 - model.offset[1], abs=1e-9)
    assert c[1] == pytest.approx(-0.2 - model.offset[1], abs=1e-9)
    assert list(found) == [True, True, False] and c[2] == 0.0


def test_cell_is_clipped_to_c_max() -> None:
    rows = _rows([(30.0, 1.5, 1, 0, +2.0, 100), (30.0, 3.5, 1, 0, -2.0, 100)])
    model = felmod.fit(rows, PARAMS)
    c, _ = model.cell_value(
        np.array([30.0, 30.0]), np.array([1.5, 3.5]), np.array([1, 1]), np.zeros(2, int)
    )
    assert c[0] == pytest.approx(0.5) and c[1] == pytest.approx(-0.5)


def test_nn_is_zero_far_from_data_and_deterministic() -> None:
    rng = np.random.default_rng(0)
    n = 400
    v = rng.uniform(25, 35, n)
    u = rng.uniform(1.0, 5.0, n)
    rows = felmod.FelRows(
        r=0.1 * (u - 3.0), pedal=np.ones(n, int), v=v, u=u, rel=np.zeros(n, int),
        stop20=np.zeros(n, bool), acc=np.zeros(n), kind=np.zeros(n, int), release=np.zeros(n),
    )
    a, b = felmod.fit(rows, PARAMS), felmod.fit(rows, PARAMS)
    q = (
        np.array([30.0, 100.0, 30.0]), np.array([3.0, 3.0, -3.0]),
        np.array([1, 1, -1]), np.zeros(3, int),
    )
    ca, cov = a.nn_value(*q)
    cb, _ = b.nn_value(*q)
    assert np.array_equal(ca, cb)  # seed 固定
    assert list(cov) == [True, False, False]
    assert ca[1] == 0.0 and ca[2] == 0.0  # 遠い所・データの無いペダルは 0
    assert abs(ca[0]) <= PARAMS.c_max_kmhs


def test_with_fel_without_correction_is_the_same_table() -> None:
    t = _table()
    assert t.with_fel("cell") is t and t.with_fel("nn") is t
    assert not t.has_fel()
    with pytest.raises(ValueError):
        t.with_fel("bad")


def test_with_fel_adds_correction_keeps_f_and_is_monotone() -> None:
    t = _table()
    c = np.zeros_like(t.accel_grid)
    c[:, 12] = 1.0  # 1 点だけ大きく持ち上げる → 単調化で隣が引き上げられる
    crel = np.full_like(t.accel_grid, 0.1)
    t2 = replace_fel(t, c, crel)
    out = t2.with_fel("cell")
    assert np.all(np.diff(out.accel_grid, axis=1) >= -1e-12)
    assert np.array_equal(t2.accel_grid, t.accel_grid)  # 元の f は書き換えない
    assert out.accel_grid[0, 12] > t.accel_grid[0, 12]
    assert out.meta["fel_applied"] == "cell"
    # nn の補正は無いので nn では同じ表
    assert t2.with_fel("nn") is t2


def replace_fel(t: ForwardTable, c_main, c_rel) -> ForwardTable:
    from dataclasses import replace

    return replace(t, fel_cell_main=c_main, fel_cell_rel=c_rel)


def test_grid_corrections_follow_the_cell_of_each_grid_point() -> None:
    rows = _rows([(30.0, 1.5, 1, 0, +0.3, 100), (30.0, 3.5, 1, 0, -0.3, 100)])
    model = felmod.fit(rows, PARAMS)
    t = _table()
    g = felmod.grid_corrections(model, t)
    i = int(np.argmin(np.abs(t.speed_rep - 30.0)))  # 20〜40 km/h の行
    j = int(np.argmin(np.abs(t.u_grid - 1.5)))  # u=1.5 は ビン 1（1〜2）
    assert g["cell_main"][i, j] == pytest.approx(0.3 - model.offset[1])
    assert g["cell_main"][i, 0] == 0.0  # ブレーキ側にデータは無い
    assert np.all(g["cell_rel"] == 0.0)


def test_attach_save_load_roundtrip_and_old_pkl_compat(tmp_path: Path) -> None:
    t = _table()
    old = tmp_path / "old.pkl"
    save_table(t, old)
    back = load_forward_map(old)
    assert back.fel_cell_main is None and back.fel_nn_main is None and back.fel_fit is None
    rows = _rows([(30.0, 1.5, 1, 0, +0.3, 100), (30.0, 3.5, 1, 0, -0.3, 100)])
    t2 = felmod.attach_fel(t, rows, PARAMS, source="x.csv")
    assert np.array_equal(t2.accel_grid, t.accel_grid)
    assert t2.meta["fel"]["source_csv"] == "x.csv"
    new = tmp_path / "new.pkl"
    save_table(t2, new)
    back2 = load_forward_map(new)
    assert np.array_equal(back2.fel_cell_main, t2.fel_cell_main)
    assert back2.fel_nn_main is not None and isinstance(back2.fel_fit, felmod.FelFit)
    assert back2.has_fel("cell") and back2.has_fel("nn")


def test_fel_config_defaults_validation_and_yaml() -> None:
    cfg = cfgmod.load_config()
    assert cfg.fel.enabled is False and cfg.fel.method == "cell"
    assert cfg.fel.c_max_kmhs == pytest.approx(0.5) and cfg.fel.min_cell_s == pytest.approx(2.0)
    assert [p for p in cfgmod.validate_config(cfg) if "fel." in p] == []
    p = felmod.FelParams.from_config(cfg.fel)
    assert p.speed_edges_kmh[0] == 0.3 and p.brake_u_bin_pct == 0.5
    cfg.fel.method = "x"
    assert any("fel.method" in m for m in cfgmod.validate_config(cfg))
    cfg.fel.method = "nn"
    cfg.fel.c_max_kmhs = 0.0
    assert any("fel.c_max_kmhs" in m for m in cfgmod.validate_config(cfg))
    cfg.fel.c_max_kmhs = 0.5
    cfg.fel.speed_edges_kmh = [1.0, 1.0, 2.0]
    assert any("fel.speed_edges_kmh" in m for m in cfgmod.validate_config(cfg))


def _cfg_with_pkl(tmp_path: Path, table: ForwardTable) -> cfgmod.ResearchConfig:
    cfg = cfgmod.load_config()
    cfg.feedforward.candidate = "C7"
    cfg.feedforward.model_path = str(tmp_path / "t.pkl")
    save_table(table, Path(cfg.feedforward.model_path))
    return cfg


def test_load_feedforward_uses_fel_only_when_enabled_and_available(tmp_path: Path) -> None:
    rows = _rows([(30.0, 1.5, 1, 0, +0.3, 100), (30.0, 3.5, 1, 0, -0.3, 100)])
    t = felmod.attach_fel(_table(), rows, PARAMS, source="x.csv")
    cfg = _cfg_with_pkl(tmp_path, t)
    cfg.fel.enabled = False  # 無効: 今と同じ（元の f）
    ff = mdmod.load_feedforward(cfg)
    assert np.array_equal(ff.table.accel_grid, t.accel_grid)
    assert "無効" in mdmod.fel_text(cfg, ff.table)
    cfg.fel.enabled = True
    ff = mdmod.load_feedforward(cfg)
    assert not np.array_equal(ff.table.accel_grid, t.accel_grid)  # f + c
    assert "有効" in mdmod.fel_text(cfg, ff.table) and "x.csv" in mdmod.fel_text(cfg, ff.table)
    # 補正表の無い pkl: 有効でも同じ表
    cfg2 = _cfg_with_pkl(tmp_path, _table())
    cfg2.fel.enabled = True
    ff2 = mdmod.load_feedforward(cfg2)
    assert np.array_equal(ff2.table.accel_grid, _table().accel_grid)
    assert "補正表なし" in mdmod.fel_text(cfg2, ff2.table)


def test_fel_rows_from_csv_builds_r_and_drops_stop_rows(tmp_path: Path) -> None:
    """CSV から r を作る。基準 0 の行（停車）・v ≤ 0.3 の行は捨てる。"""
    n = 400
    t = np.arange(n) * 0.05
    v = np.where(t < 10.0, 10.0 + 1.0 * t, 20.0)  # 1 km/h/s で加速してから定速
    ref = np.where(t < 1.0, 0.0, v)
    ac = np.full(n, 6.0)
    head = "timestamp,mode_time_s,ref_speed_kmh,actual_speed_kmh,accel_cmd_pct,brake_cmd_pct"
    lines = [head + ",governor_active"]
    for i in range(n):
        lines.append(f"x,{t[i]:.3f},{ref[i]:.3f},{v[i]:.3f},{ac[i]:.3f},0.0,0")
    path = tmp_path / "log.csv"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    tb = _table()
    rows = felmod.fel_rows(path, tb, PARAMS)
    assert 0 < len(rows) < n  # 基準 0 の最初の 1 秒は落ちる
    assert np.all(rows.v > 0.3) and np.all(rows.pedal == 1)
    # 加速中の実加速度 ≈ 1.0 → r ≈ 1.0 − 表(v, u=2)
    i = int(np.argmin(np.abs(rows.v - 15.0)))
    assert rows.acc[i] == pytest.approx(1.0, abs=0.05)
    assert rows.r[i] == pytest.approx(rows.acc[i] - float(tb.accel(rows.v[i], rows.u[i])), abs=1e-6)


def test_predict_passes_release_to_stop_layers_and_accel_release_to_main() -> None:
    """2026-10-20 段4-3: 停止側の行は停止側の戻しの層があるときだけ戻し量を渡し、本体の行は
    アクセルの戻し量を渡す。層・表が無い表では今までと同じ値。"""
    plain = _table()
    plain.stop_accel = plain.accel_grid.copy()
    v = np.array([3.0, 3.0, 30.0, 30.0])
    u = np.array([-3.0, -3.0, 3.0, 3.0])
    kind = np.array([felmod.KIND_STOP, felmod.KIND_STOP, felmod.KIND_MAIN, felmod.KIND_MAIN])
    rel = np.array([0.0, 2.0, 0.0, 0.0])
    arel = np.array([0.0, 0.0, 0.0, 2.0])
    base = felmod._predict(plain, v, u, kind, rel, arel)
    assert np.array_equal(base, felmod._predict(plain, v, u, kind, rel))  # 層なしは無視される
    layered = _table()
    layered.stop_accel = layered.accel_grid.copy()
    layered.stop_release_grid = np.array([1.0, 2.0])
    layered.stop_release_accel = np.stack([layered.stop_accel + 0.5, layered.stop_accel + 1.0])
    layered.accel_release_grid = np.array([1.0, 2.0])
    layered.accel_release_accel = np.stack([layered.accel_grid - 0.5, layered.accel_grid - 1.0])
    layered.meta["stop_release_max_pct"] = 2.0
    layered.meta["accel_release_max_pct"] = 2.0
    got = felmod._predict(layered, v, u, kind, rel, arel)
    assert got[0] == pytest.approx(base[0])  # 戻し量 0 は停止側のまま
    assert got[1] == pytest.approx(base[1] + 1.0)  # 戻し量 2% → 節 2.0
    assert got[2] == pytest.approx(base[2])
    assert got[3] == pytest.approx(base[3] - 1.0)  # アクセルの戻し量 2%


def test_predict_passes_accel_push_to_main_rows_only_when_table_has_axis() -> None:
    """2026-10-23 段5-7b: 本体の行に踏み増し量を渡す。踏み増しの表が無い表では無視される。"""
    plain = _table()
    v = np.array([30.0, 30.0])
    u = np.array([3.0, 3.0])
    kind = np.array([felmod.KIND_MAIN, felmod.KIND_MAIN])
    rel = np.zeros(2)
    push = np.array([0.0, 2.0])
    base = felmod._predict(plain, v, u, kind, rel)
    assert np.array_equal(base, felmod._predict(plain, v, u, kind, rel, None, push))
    layered = _table()
    layered.accel_push_grid = np.array([1.0, 2.0])
    layered.accel_push_accel = np.stack([layered.accel_grid + 0.5, layered.accel_grid + 1.0])
    layered.meta["accel_push_max_pct"] = 2.0
    got = felmod._predict(layered, v, u, kind, rel, None, push)
    assert got[0] == pytest.approx(base[0])
    assert got[1] == pytest.approx(base[1] + 1.0)
