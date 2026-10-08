"""fwd_final（全ログ学習の最終版）のテスト。合成データだけを使う。"""

from __future__ import annotations

import numpy as np

from tests.research.fwd_cv import CvConfig
from tests.research.fwd_data import concat_rows, make_speed_edges
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
