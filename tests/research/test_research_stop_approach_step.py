"""手順14: 停止の手前の測定（2026-10-31 段2。ProblemReport_20261031_停止間際 B）。

prepare のエラー・保存先の名前・停止側の表の作り直し（`rebuild_stop_side`）。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import main as mainmod
from tests.research import pattern_drive as pdmod
from tests.research.fwd_map import load_forward_map, save_table
from tests.research.ramp_planner import format_stop_approach, stop_approach_top_kmh
from tests.research.test_research_fwd_map import release_table

REAL_PKL = Path("tests/research/results/models/fwd_model_20261028_220204_hpi2_hold.pkl")
REAL_CSV = Path("tests/research/results/drive_log_real_20261029_062805.csv")


def _ctx(cfg: cfgmod.ResearchConfig) -> mainmod.RunContext:
    return mainmod.RunContext(config=cfg, hw_mode="stub", started_at=0.0)


def test_step14_exists_and_pkl_name() -> None:
    step = next(s for s in mainmod.STEPS if s.number == 14)
    assert step.drives and step.prepare is not None
    assert mainmod.MAX_STEP == 14
    assert mainmod.stop_pkl_path("a/b/fwd_model_x_hold.pkl") == Path(
        "a/b/fwd_model_x_hold_stop.pkl"
    )


def test_pkl_source_csvs_accepts_plus_comma_and_list() -> None:
    d = Path("res")
    assert mainmod._pkl_source_csvs({"sources": "a.csv"}, d) == [d / "a.csv"]
    assert mainmod._pkl_source_csvs({"sources": "a.csv+b.csv"}, d) == [d / "a.csv", d / "b.csv"]
    assert mainmod._pkl_source_csvs({"sources": "a.csv, b.csv"}, d) == [d / "a.csv", d / "b.csv"]
    assert mainmod._pkl_source_csvs({"sources": ["a.csv", "b.csv"]}, d) == [
        d / "a.csv", d / "b.csv",
    ]
    assert mainmod._pkl_source_csvs({}, d) == []


async def test_prepare_step14_errors_before_touching_hardware(tmp_path: Path) -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.output.results_dir = str(tmp_path)
    pkl = tmp_path / "m.pkl"
    cfg.feedforward.model_path = str(pkl)
    ctx = _ctx(cfg)
    with pytest.raises(cfgmod.ConfigError, match="pkl がありません"):
        await mainmod.prepare_step14(ctx)
    table = release_table()
    save_table(table, pkl)  # sources が無い
    with pytest.raises(cfgmod.ConfigError, match="sources"):
        await mainmod.prepare_step14(ctx)
    table.meta["sources"] = "src.csv"
    save_table(table, pkl)
    with pytest.raises(cfgmod.ConfigError, match="results にありません"):
        await mainmod.prepare_step14(ctx)
    (tmp_path / "src.csv").write_text("x\n", encoding="utf-8")
    await mainmod.prepare_step14(ctx)  # 通る
    out = mainmod.stop_pkl_path(pkl)
    out.write_bytes(b"x")
    with pytest.raises(cfgmod.ConfigError, match="既にあります"):
        await mainmod.prepare_step14(ctx)
    out.unlink()
    table.meta["launch_rows_to_stop_max"] = True  # 作り直しでは元と同じにならない pkl
    save_table(table, pkl)
    with pytest.raises(cfgmod.ConfigError, match="launch_rows_to_stop_max"):
        await mainmod.prepare_step14(ctx)


def test_stub_like_pedal_comes_from_yaml() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    pedal = mainmod._stub_like_pedal(cfg)
    hold = cfg.vehicle.min_brake_opening_pct + cfg.feedforward.stop_brake_offset_pct
    assert pedal.stop_brake_opening_pct == pytest.approx(hold)
    assert pedal.stop_confirm_pct == pytest.approx(hold - cfg.pedal_search.stop_hold_margin_pct)
    assert pedal.brake_min_pct == cfg.vehicle.min_brake_opening_pct
    assert pedal.creep_speed_kmh == cfg.feedforward.creep_speed_kmh


def test_stop_approach_top_and_planner_text() -> None:
    kw = {"release_start_kmh": 7.5, "turn_s": 2.0, "settle_s": 1.0}
    assert [stop_approach_top_kmh(g, **kw) for g in (0.045, 0.08, 0.125, 0.15)] == [
        15.0, 15.0, 20.0, 20.0]
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    text = format_stop_approach(cfg)
    assert "0.045G × 1" in text and "0.15G × 2" in text and "上端 20 km/h" in text
    cfg.learning.stop_approach_enabled = False
    assert format_stop_approach(cfg) == ""


@pytest.mark.skipif(not (REAL_PKL.exists() and REAL_CSV.exists()), reason="実機の pkl・CSV が無い")
def test_rebuild_stop_side_changes_only_the_stop_side(tmp_path: Path) -> None:
    old = load_forward_map(REAL_PKL)
    before = REAL_PKL.read_bytes()
    new = pdmod.rebuild_stop_side(
        old, source_csvs=[REAL_CSV], new_csv=REAL_CSV, creep_speed_kmh=5.0,
    )
    assert REAL_PKL.read_bytes() == before  # 元は書き換わらない
    assert new.meta["stop_approach_csv"] == REAL_CSV.name
    assert "stop_approach_csv" not in old.meta
    # 停止側以外の表・meta はそのまま
    for name in ("accel_grid", "launch_accel", "launch_kind", "point_kind", "u_grid"):
        a, b = getattr(old, name), getattr(new, name)
        assert np.array_equal(a, b), name
    for key, val in old.meta.items():
        if key.startswith("stop_") and key != "stop_approach_exclusion":
            continue
        assert str(new.meta[key]) == str(val), key
    # 同じ CSV を 2 回足したので停止側の行の秒数は 2 倍
    assert new.meta["stop_rows_s"] == pytest.approx(2.0 * old.meta["stop_rows_s"], rel=1e-6)
    # 保存して読み戻せる
    out = save_table(new, tmp_path / "x_stop.pkl")
    assert load_forward_map(out).meta["stop_approach_csv"] == REAL_CSV.name


def test_rebuild_stop_side_refuses_tables_that_depend_on_stop_rows() -> None:
    table = release_table()
    table.meta["side_row_speed_rep"] = "data"
    with pytest.raises(ValueError, match="停止側だけ"):
        pdmod.rebuild_stop_side(
            table, source_csvs=[], new_csv=Path("none.csv"), creep_speed_kmh=5.0
        )


def test_down_seconds_in_stop_band_counts_ramp_down_rows(tmp_path: Path) -> None:
    rows = ["elapsed_s,actual_speed_kmh,section,pattern,phase"]
    t = 0.0
    for v in (9.0, 7.0, 5.0, 3.0, 1.0, 0.2):  # RAMP_DOWN 1 本目: 7.0〜1.0 の 4 区間が帯の中
        rows.append(f"{t:.1f},{v},PATTERN_DRIVE,1:G_RAMP,RAMP_DOWN")
        t += 0.5
    rows.append(f"{t:.1f},4.0,PATTERN_DRIVE,2:G_RAMP,RAMP_UP")  # 上りは数えない
    path = tmp_path / "x.csv"
    path.write_text("\n".join(rows) + "\n", encoding="utf-8")
    secs = mainmod._down_seconds_in_stop_band(path, 2)
    # 帯の中の行は 7.0・5.0・3.0・1.0。各行の「前の行からの経過」0.5 s ずつ（1 行目 9.0 は前が無い）
    assert secs[1] == pytest.approx(0.5 * 4)
    assert secs[2] == 0.0
