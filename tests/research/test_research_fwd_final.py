"""fwd_final（全ログ学習の最終版）のテスト。合成データだけを使う。"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research.fwd_cv import CvConfig
from tests.research.fwd_data import (
    FwdRows,
    accel_release_amount,
    concat_rows,
    make_speed_edges,
    release_amount,
)
from tests.research.fwd_final import build_final_table, final_config
from tests.research.fwd_map import KIND_MODEL, build_table, save_table
from tests.research.test_research_fwd_model import make_rows


def test_final_table_is_mean_of_seeds_and_monotone(tmp_path):
    rows = [make_rows(i, f"log{i}") for i in range(2)]
    like = build_table(concat_rows(rows), speed_edges=make_speed_edges(80.0))
    path = save_table(like, tmp_path / "like.pkl")
    cfg = CvConfig("t", (4,), loss="absolute", seeds=(0, 1))
    tb, secs = build_final_table(cfg, rows, path, workers=1, max_iter=3)
    assert tb.kind == KIND_MODEL
    assert [s for s, _ in secs] == [0, 1]
    assert tb.meta["n_models"] == 2 and tb.meta["loss"] == "absolute"
    assert tb.meta["sources"] == "log0+log1"
    assert np.all(np.diff(tb.accel_grid, axis=1) >= -1e-9)


def test_final_table_seed_mae_is_opt_in(tmp_path):
    """seed_mae=True のとき、乱数ごとの学習データでの MAE を meta に入れる（既定は入れない）。"""
    rows = [make_rows(i, f"log{i}") for i in range(2)]
    like = build_table(concat_rows(rows), speed_edges=make_speed_edges(80.0))
    path = save_table(like, tmp_path / "like.pkl")
    cfg = CvConfig("t", (4,), loss="absolute", seeds=(0, 1))
    tb_off, _ = build_final_table(cfg, rows, path, workers=1, max_iter=3)
    assert "seed_mae" not in tb_off.meta
    tb, _ = build_final_table(cfg, rows, path, workers=1, max_iter=3, seed_mae=True)
    assert sorted(tb.meta["seed_mae"]) == [0, 1]
    assert all(np.isfinite(v) and v >= 0.0 for v in tb.meta["seed_mae"].values())
    # 表そのもの（乱数の平均）は seed_mae の有無で変わらない
    assert np.allclose(tb.accel_grid, tb_off.accel_grid)


def test_final_config_default_is_unchanged():
    cfg = final_config()
    assert cfg == CvConfig("1-1 絶対誤差", (30, 30), loss="absolute", seeds=(0, 1, 2, 3, 4))
    assert "drop_stop_approach" not in cfg.key()


def test_final_config_args_are_reflected():
    cfg = final_config((0,), hidden=(50, 50), activation="relu", drop_stop_approach=True)
    assert cfg.hidden == (50, 50) and cfg.activation == "relu" and cfg.drop_stop_approach
    assert cfg.loss == "absolute" and cfg.seeds == (0,)
    assert cfg.name == "2層50個 relu 絶対誤差 停車まぎわ除外"
    assert cfg.key().endswith("|drop_stop_approach=True")


def test_final_config_rollout_s_name_and_key():
    cfg = final_config((0,), hidden=(50, 50), activation="relu", drop_stop_approach=True,
                       rollout_s=10)
    assert cfg.name == "2層50個 relu 絶対誤差 停車まぎわ除外 積み上げ10秒"
    assert cfg.rollout_s == 10 and cfg.key().endswith("|rollout_s=10")
    assert final_config().name == "1-1 絶対誤差" and final_config().rollout_s == 0
    assert final_config(rollout_s=30).name == "1-1 絶対誤差 積み上げ30秒"


def test_missing_compare_logs_are_skipped(tmp_path, capsys):
    """固定の比較ログが無ければ、その採点をとばして 1 行表示する（落ちない）。"""
    from tests.research.fwd_final import print_blocks
    from tests.research.fwd_tool import skip_missing

    exist = tmp_path / "a.csv"
    exist.write_text("x\n", encoding="utf-8")
    gone = tmp_path / "gone.csv"
    assert skip_missing([exist, gone], "検証ログ") == [str(exist)]
    first = capsys.readouterr().out
    assert "gone.csv" in first and "とばします" in first
    print_blocks({}, None, None, [], [])  # 比較ログが空なら採点（表の読み込み）に入らない
    out = capsys.readouterr().out
    assert out.count("とばします") == 2


def test_final_config_table_u_step_pct_passes_through_and_keeps_key():
    assert final_config((0,)).table_u_step_pct is None
    assert "table_u_step_pct" not in final_config((0,)).key()
    cfg = final_config((0,), table_u_step_pct=0.5)
    assert cfg.table_u_step_pct == 0.5 and cfg.name == final_config((0,)).name


def test_final_table_with_finer_u_step(tmp_path):
    """2026-10-14 段2o-4: table_u_step_pct=0.5 で C8 の表だけ u の刻みが 0.5%（C7 は 2%）。"""
    rows = [make_rows(i, f"log{i}") for i in range(2)]
    like = build_table(concat_rows(rows), speed_edges=make_speed_edges(80.0))
    path = save_table(like, tmp_path / "like.pkl")
    cfg = CvConfig("t", (4,), loss="absolute", seeds=(0,), table_u_step_pct=0.5)
    tb, _ = build_final_table(cfg, rows, path, workers=1, max_iter=3)
    assert tb.meta["u_step_pct"] == 0.5 and tb.meta["like_u_step_pct"] == 2.0
    assert tb.accel_grid.shape == (len(like.speed_rep), len(tb.u_grid))
    assert tb.seconds.shape == tb.point_kind.shape == tb.accel_grid.shape
    assert len(tb.u_grid) > len(like.u_grid)
    assert tb.u_grid[0] == pytest.approx(like.u_grid[0] - 0.75)
    assert np.all(np.diff(tb.accel_grid, axis=1) >= -1e-9)
    u, flag = tb.invert(np.array([20.0, 40.0]), np.array([0.0, 0.5]))
    assert np.all(np.isfinite(u)) and len(flag) == 2
    # None なら今までと同じ格子
    tb0, _ = build_final_table(
        CvConfig("t", (4,), loss="absolute", seeds=(0,)), rows, path, workers=1, max_iter=3
    )
    assert np.array_equal(tb0.u_grid, like.u_grid) and tb0.meta["u_step_pct"] == 2.0


def make_release_rows(
    seed: int,
    source: str,
    n: int = 4000,
    release_gain: float = 0.0,
    accel_release_gain: float = 0.0,
) -> FwdRows:
    """u がゆっくり揺れる時系列（0.05 秒刻み）。戻し量は release_amount で全系列から計算する。
    ブレーキが弱まる効き（release_gain × 戻し量 × |u|）を加速度に足す。"""
    rng = np.random.default_rng(seed)
    t = np.arange(n) * 0.05
    u = -3.0 + 8.0 * np.sin(t / 3.0 + seed) + rng.normal(0.0, 0.2, n)
    v = rng.uniform(10.0, 70.0, n)
    rel = release_amount(t, u, 3.0)
    arel = accel_release_amount(t, u, 3.0)
    a = -2.0 - 0.01 * v + np.where(
        u >= 0, 0.3 * u - accel_release_gain * arel * u, 0.6 * u + release_gain * rel * -u
    )
    return FwdRows(
        speed_kmh=v,
        u_pct=u,
        accel_kmhs=a,
        t_s=t,
        dt_s=0.05,
        source=source,
        dt_row_s=np.full(n, 0.05),
        release_pct=rel,
        accel_release_pct=arel,
    )


def test_final_table_attaches_monotone_release_slices(tmp_path):
    """2026-10-14 段2o-5: 戻しの行が足りれば戻しの表が付き、各スライスは行ごとに単調。"""
    rows = [make_release_rows(i, f"log{i}", release_gain=0.05) for i in range(2)]
    like = build_table(concat_rows(rows), speed_edges=make_speed_edges(80.0))
    path = save_table(like, tmp_path / "like.pkl")
    cfg = CvConfig("t", (6,), loss="absolute", seeds=(0, 1))
    tb, _ = build_final_table(cfg, rows, path, workers=1, max_iter=30)
    assert tb.release_grid is not None
    assert tb.release_accel.shape == (len(tb.release_grid), *tb.accel_grid.shape)
    assert np.all(np.diff(tb.release_accel, axis=2) >= -1e-9)
    assert tb.meta["release_rows_s"] >= 10.0 and tb.meta["release_max_pct"] > 0.0
    # 本体（戻し量 0）は release_slices の有無で変わらない
    cfg_off = CvConfig("t", (6,), loss="absolute", seeds=(0, 1), release_slices=False)
    tb_off, _ = build_final_table(cfg_off, rows, path, workers=1, max_iter=30)
    assert tb_off.release_grid is None and tb_off.release_accel is None
    assert np.allclose(tb_off.accel_grid, tb.accel_grid)


def test_final_table_without_release_rows_has_no_release_table(tmp_path):
    rows = [make_rows(i, f"log{i}") for i in range(2)]  # 戻し量はすべて 0
    like = build_table(concat_rows(rows), speed_edges=make_speed_edges(80.0))
    path = save_table(like, tmp_path / "like.pkl")
    cfg = CvConfig("t", (4,), loss="absolute", seeds=(0,))
    tb, _ = build_final_table(cfg, rows, path, workers=1, max_iter=3)
    assert tb.release_grid is None and tb.meta["release_rows_s"] == 0.0


def test_final_config_release_slices_default_on_and_keeps_key():
    assert final_config().release_slices is True
    assert "release_slices" not in final_config().key()
    assert final_config(release_slices=False).key().endswith("|release_slices=False")


def test_final_table_release_stats_exclude_launch_rows(tmp_path):
    """2026-10-14 段2o-5: 発進の区間の行（launch_row）は戻しの秒数・頭打ちに数えない。"""
    import dataclasses  # noqa: PLC0415

    rows = [make_release_rows(i, f"log{i}", release_gain=0.05) for i in range(2)]
    like = build_table(concat_rows(rows), speed_edges=make_speed_edges(80.0))
    path = save_table(like, tmp_path / "like.pkl")
    cfg = CvConfig("t", (6,), loss="absolute", seeds=(0,))
    tb, _ = build_final_table(cfg, rows, path, workers=1, max_iter=5)
    flagged = [
        dataclasses.replace(r, launch_row=(r.release_pct >= 0.3) & (r.u_pct < 0)) for r in rows
    ]
    tb2, _ = build_final_table(cfg, flagged, path, workers=1, max_iter=5)
    assert tb.meta["release_rows_s"] > 10.0
    assert tb2.meta["release_rows_s"] == pytest.approx(0.0)
    assert tb2.release_grid is None
    off = CvConfig("t", (6,), loss="absolute", seeds=(0,), brake_exclude_launch=False)
    tb3, _ = build_final_table(off, flagged, path, workers=1, max_iter=5)
    assert tb3.meta["release_rows_s"] == pytest.approx(tb.meta["release_rows_s"])


def test_final_config_brake_exclude_launch_default_on():
    assert final_config().brake_exclude_launch is True
    assert "brake_exclude_launch" not in final_config().key()
    assert not final_config(brake_exclude_launch=False).brake_exclude_launch


def test_final_table_attaches_accel_release_slices(tmp_path):
    """2026-10-20 段4-3: アクセルの戻し行が足りれば、アクセルの戻しの表が付く（行ごとに単調）。
    u > 0 の列は戻し量で変わり（戻すと加速が弱い）、u < 0 の列は本体と同じ。"""
    rows = [
        make_release_rows(i, f"log{i}", release_gain=0.05, accel_release_gain=0.03)
        for i in range(2)
    ]
    like = build_table(concat_rows(rows), speed_edges=make_speed_edges(80.0))
    path = save_table(like, tmp_path / "like.pkl")
    cfg = CvConfig("t", (6,), loss="absolute", seeds=(0, 1))
    tb, _ = build_final_table(cfg, rows, path, workers=1, max_iter=30)
    assert tb.accel_release_grid is not None
    assert tb.accel_release_accel.shape == (len(tb.accel_release_grid), *tb.accel_grid.shape)
    assert np.all(np.diff(tb.accel_release_accel, axis=2) >= -1e-9)
    assert tb.meta["accel_release_rows_s"] >= 10.0 and tb.meta["accel_release_max_pct"] > 0.0
    ug = tb.u_grid
    pos = ug > 0
    assert not np.allclose(tb.accel_release_accel[-1][:, pos], tb.accel_grid[:, pos])
    assert np.allclose(tb.accel_release_accel[:, :, ug < 0], tb.accel_grid[None, :, ug < 0])
    # release_slices=False ならアクセルの戻しの表も付かず、本体は変わらない
    cfg_off = CvConfig("t", (6,), loss="absolute", seeds=(0, 1), release_slices=False)
    tb_off, _ = build_final_table(cfg_off, rows, path, workers=1, max_iter=30)
    assert tb_off.accel_release_grid is None and np.allclose(tb_off.accel_grid, tb.accel_grid)


def test_final_table_without_accel_release_rows_has_no_accel_release_table(tmp_path):
    rows = [make_rows(i, f"log{i}") for i in range(2)]  # アクセルの戻し量はすべて 0
    like = build_table(concat_rows(rows), speed_edges=make_speed_edges(80.0))
    path = save_table(like, tmp_path / "like.pkl")
    cfg = CvConfig("t", (4,), loss="absolute", seeds=(0,))
    tb, _ = build_final_table(cfg, rows, path, workers=1, max_iter=3)
    assert tb.accel_release_grid is None and tb.meta["accel_release_rows_s"] == 0.0
