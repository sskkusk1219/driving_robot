"""FEL の埋まり方（fel.wltp_coverage）・手順2 の 2-3 と手順 12（main）・step2_check F1 のテスト。

合成データだけを使い、DB・実機・スタブ HW は要らない（2026-10-19 段2s-2b）。
"""

from __future__ import annotations

import asyncio
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import fel as felmod
from tests.research import main as mainmod
from tests.research import step2_check as sc
from tests.research.fwd_map import load_forward_map, save_table
from tests.research.pattern_drive import DriveError
from tests.research.research_types import DrivingMode, SpeedPoint
from tests.research.test_research_fel import PARAMS, _rows, _table


def _mode(points: list[tuple[float, float]]) -> DrivingMode:
    return DrivingMode(
        id="w", name="w", description="", reference_speed=[SpeedPoint(t, v) for t, v in points],
        total_duration=points[-1][0], max_speed=max(v for _, v in points),
        created_at=datetime(2026, 1, 1),
    )


def _attached(tmp_path: Path | None = None):
    # v=30 km/h・アクセル u 2〜3% のマス（_table では定速 30 km/h に要る u≈2.7）だけを学習する
    rows = _rows([(30.0, 2.5, 1, 0, 0.1, 200)])
    return felmod.attach_fel(_table(), rows, PARAMS, source="learn.csv")


def test_coverage_counts_only_cells_with_correction() -> None:
    t = _attached()
    mode = _mode([(0.0, 30.0), (30.0, 30.0), (31.0, 25.0), (60.0, 25.0)])
    cov = felmod.wltp_coverage(t, mode, PARAMS)
    assert cov is not None
    d = {nm: (tot, c) for nm, tot, c in cov.groups}
    tot, c = d["全体"]
    assert 55.0 < tot <= 60.2  # 基準が 0 の行は無い
    assert 0.4 < c / tot < 0.6  # 30 km/h の半分は補正のあるマス、25 km/h は無い
    assert d["アクセル"][0] == pytest.approx(tot, abs=3.0)  # 25 km/h へ下げる 1 秒だけブレーキ
    assert d["ブレーキ"][0] < 3.0 and d["ブレーキ"][1] == 0.0  # 補正のあるブレーキのマスは無い
    top = cov.missing[0]
    assert "アクセル" in top.text and top.wltp_s > 20.0 and top.learn_s == 0.0
    text = felmod.format_coverage(cov)
    assert "全体" in text and "足りないマス" in text and "WLTP" in text


def test_coverage_reports_learning_seconds_of_sparse_cell() -> None:
    u25 = float(np.asarray(_table().invert(25.0, 0.0)[0]))  # 25 km/h の定速に要る u
    rows = _rows([(30.0, 2.5, 1, 0, 0.1, 200), (25.0, u25, 1, 0, 0.1, 10)])  # 後者は 0.5 秒
    t = felmod.attach_fel(_table(), rows, PARAMS, source="x")
    cov = felmod.wltp_coverage(t, _mode([(0.0, 25.0), (60.0, 25.0)]), PARAMS)
    assert cov is not None and cov.missing
    assert cov.missing[0].learn_s == pytest.approx(0.5)  # 行はあるが min_cell_s 未満


def test_coverage_none_without_correction_table() -> None:
    assert felmod.wltp_coverage(_table(), _mode([(0.0, 30.0), (10.0, 30.0)]), PARAMS) is None


def test_check_fel_coverage_item_is_notice_not_ng() -> None:
    cov = felmod.wltp_coverage(
        _attached(), _mode([(0.0, 30.0), (30.0, 30.0), (31.0, 25.0), (60.0, 25.0)]), PARAMS
    )
    item = sc.check_fel_coverage(cov)
    assert item.id == "F1" and item.status == sc.STATUS_OK
    assert "全体" in item.value and "足りないマス" in item.reason
    assert sc.ng_items([item]) == []
    assert "F1" in sc.format_checks([item])
    skip = sc.check_fel_coverage(None)
    assert skip.status == sc.STATUS_SKIP


# ── main: 2-3 / 手順 12 ───────────────────────────────────────────────


class _FakeLog:
    def __init__(self, csv_path: Path) -> None:
        self.csv_path = csv_path
        self.closed = False

    async def close(self) -> Path:
        self.closed = True
        return self.csv_path


def _write_csv(path: Path) -> None:
    n = 800
    t = np.arange(n) * 0.05
    v = 30.0 + 0.0 * t
    head = "timestamp,mode_time_s,ref_speed_kmh,actual_speed_kmh,accel_cmd_pct,brake_cmd_pct"
    lines = [head + ",governor_active"]
    lines += [f"x,{t[i]:.3f},{v[i]:.3f},{v[i]:.3f},6.5,0.0,0" for i in range(n)]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _cfg(tmp_path: Path) -> cfgmod.ResearchConfig:
    cfg = cfgmod.load_config()
    cfg.feedforward.candidate = "C12"
    cfg.feedforward.model_path = "tests/research/results/models/NOT_USED.pkl"
    cfg.fel.enabled = True
    return cfg


def test_prepare_fel_run_uses_given_pkl_candidate_and_no_fel(tmp_path, monkeypatch) -> None:
    pkl = tmp_path / "new.pkl"
    save_table(_table(), pkl)
    seen = {}

    def fake_prepare(cfg, mode):
        seen["cfg"], seen["mode"] = cfg, mode
        return SimpleNamespace(mode=mode, ff=None)

    monkeypatch.setattr(mainmod, "prepare_mode_drive_from", fake_prepare)
    cfg = _cfg(tmp_path)
    cover = [_mode_trips()]
    run = mainmod._prepare_fel_run(cfg, str(pkl), cover, 130.0)
    rc = seen["cfg"]
    assert rc.fel.enabled is False and cfg.fel.enabled is True  # 元の設定は変えない
    assert rc.feedforward.model_path == str(pkl) and rc.feedforward.candidate == "C12"
    assert seen["mode"] is run.pattern.mode and run.pattern.stop_count == cfg.fel.low_trip_count + 2


def _mode_trips() -> DrivingMode:
    from tests.research.test_research_fel_pattern import _trips  # noqa: PLC0415

    return _trips([20, 30, 40, 50, 60, 80, 100, 120, 30, 45])


def _run_setup(cfg, pkl: Path, tmp_path: Path) -> mainmod.FelRunSetup:
    return mainmod.FelRunSetup(
        cfg=cfg, model_path=str(pkl), pattern=None, setup=SimpleNamespace(),
        wltp_mode=_mode([(0.0, 30.0), (60.0, 30.0)]),
    )


def test_run_fel_learning_attaches_table_and_adds_f1(tmp_path, monkeypatch) -> None:
    pkl = tmp_path / "m.pkl"
    save_table(_table(), pkl)
    csv_path = tmp_path / "fel.csv"
    _write_csv(csv_path)
    log = _FakeLog(csv_path)
    monkeypatch.setattr(mainmod, "_session_log_for_drive", lambda *a, **k: log)
    got = {}

    async def fake_run(hw, cfg, setup, *, log, limit_s=None, section):
        got["section"], got["limit"] = section, limit_s
        return SimpleNamespace(completed=True, abort_reason="")

    monkeypatch.setattr(mainmod, "run_mode_drive", fake_run)
    cfg = _cfg(tmp_path)
    run = _run_setup(cfg, pkl, tmp_path)
    ctx = SimpleNamespace()
    checks: list = []
    asyncio.run(mainmod._run_fel_learning(ctx, None, run, run.wltp_mode, checks=checks,
                                          limit_s=12.0))
    assert got == {"section": "FEL_DRIVE", "limit": 12.0} and log.closed
    back = load_forward_map(pkl)
    assert back.has_fel("cell") and back.meta["fel"]["source_csv"] == "fel.csv"
    assert np.array_equal(back.accel_grid, _table().accel_grid)  # f は不変
    assert [c.id for c in checks] == ["F1"] and checks[0].status == sc.STATUS_OK


def test_run_fel_learning_failure_keeps_pkl_without_correction(tmp_path, monkeypatch) -> None:
    pkl = tmp_path / "m.pkl"
    save_table(_table(), pkl)
    log = _FakeLog(tmp_path / "none.csv")
    monkeypatch.setattr(mainmod, "_session_log_for_drive", lambda *a, **k: log)

    async def fake_run(hw, cfg, setup, *, log, limit_s=None, section):
        return SimpleNamespace(completed=False, abort_reason="CAN 車速を読めません")

    monkeypatch.setattr(mainmod, "run_mode_drive", fake_run)
    run = _run_setup(_cfg(tmp_path), pkl, tmp_path)
    with pytest.raises(DriveError, match="中断"):
        asyncio.run(mainmod._run_fel_learning(SimpleNamespace(), None, run, run.wltp_mode))
    assert not load_forward_map(pkl).has_fel()  # 補正表なしのまま


def test_run_fel_learning_unexpected_error_becomes_drive_error(tmp_path, monkeypatch) -> None:
    pkl = tmp_path / "m.pkl"
    save_table(_table(), pkl)
    log = _FakeLog(tmp_path / "missing.csv")  # CSV が無い → 補正表を作れない
    monkeypatch.setattr(mainmod, "_session_log_for_drive", lambda *a, **k: log)

    async def fake_run(hw, cfg, setup, *, log, limit_s=None, section):
        return SimpleNamespace(completed=True, abort_reason="")

    monkeypatch.setattr(mainmod, "run_mode_drive", fake_run)
    run = _run_setup(_cfg(tmp_path), pkl, tmp_path)
    with pytest.raises(DriveError):
        asyncio.run(mainmod._run_fel_learning(SimpleNamespace(), None, run, run.wltp_mode))
    assert not load_forward_map(pkl).has_fel()


def test_step12_is_registered_and_driving() -> None:
    step = mainmod._STEP_BY_NUMBER[12]
    assert step.drives and step.prepare is mainmod.prepare_step12 and step.has_ref
    assert "補正表" in step.title
    args = mainmod.build_parser().parse_args(["--steps", "1,12"])
    assert [s.number for s in mainmod.resolve_steps(args)] == [1, 12]


def test_fel_text_of_reports_pkl_and_setting(tmp_path) -> None:
    cfg = _cfg(tmp_path)
    t = _attached()
    ff = SimpleNamespace(table=t.with_fel("cell"))
    text = mainmod._fel_text_of(cfg, SimpleNamespace(ff=ff))
    assert "FEL: 有効・方法 cell" in text and "learn.csv" in text
    cfg.fel.enabled = False
    assert "無効" in mainmod._fel_text_of(cfg, SimpleNamespace(ff=SimpleNamespace(table=t)))
    assert mainmod._fel_text_of(cfg, SimpleNamespace(ff=SimpleNamespace())) is None


# ── 2026-10-20 段4-3: 手順2 の 2-3 を設定で走らない ─────────────────────


def _run_step2_with_stubs(monkeypatch, cfg) -> dict:
    """step2_pattern_drive を、走行・モデル作成を差し替えて最後まで通す。呼ばれた部品を返す。"""
    calls = {"prepare": 0, "fel": 0, "build": 0}
    mode = _mode([(0.0, 30.0), (60.0, 30.0)])

    async def fake_load_mode(_cfg, _name):
        return mode

    async def fake_pedal(*_a, **_k):
        return None

    async def fake_drive(*_a, **_k):
        return SimpleNamespace(csv_path=Path("x.csv"), ramp_results=[])

    def fake_build(*_a, **_k):
        calls["build"] += 1
        return SimpleNamespace(model_path="m.pkl", checks=[])

    def fake_prepare(*_a, **_k):
        calls["prepare"] += 1
        return SimpleNamespace()

    async def fake_fel(*_a, **_k):
        calls["fel"] += 1

    plan = SimpleNamespace(levels=[1], execution_order=lambda: [])
    monkeypatch.setattr(mainmod, "require_forward_candidate", lambda _c: None)
    monkeypatch.setattr(mainmod, "load_mode", fake_load_mode)
    monkeypatch.setattr(mainmod, "top_speed_cap_kmh", lambda *_a: 130.0)
    monkeypatch.setattr(mainmod, "count_bands_for_modes", lambda *_a: [])
    monkeypatch.setattr(mainmod, "cruise_kwargs", lambda _c: {"cruise_min_kmh": 10.0})
    monkeypatch.setattr(mainmod, "plan_ramps", lambda *_a, **_k: plan)
    monkeypatch.setattr(mainmod, "format_plan_table", lambda _p: "")
    monkeypatch.setattr(mainmod, "run_pedal_search", fake_pedal)
    monkeypatch.setattr(mainmod, "run_pattern_drive", fake_drive)
    monkeypatch.setattr(mainmod, "build_ff_model", fake_build)
    monkeypatch.setattr(mainmod, "_prepare_fel_run", fake_prepare)
    monkeypatch.setattr(mainmod, "_run_fel_learning", fake_fel)
    hw = SimpleNamespace(hw_mode="stub", is_real=False)
    ctx = SimpleNamespace(
        config=cfg, require_hardware=lambda: hw, session_log=None
    )
    asyncio.run(mainmod.step2_pattern_drive(ctx))
    return calls


def test_step2_runs_23_by_default_and_skips_it_when_disabled(monkeypatch, capsys) -> None:
    cfg = _cfg(Path("."))
    assert cfg.learning.fel_pattern_enabled is False  # 既定は false（2026-10-23 ユーザー決定）
    cfg.learning.fel_pattern_enabled = True
    on = _run_step2_with_stubs(monkeypatch, cfg)
    assert on == {"prepare": 1, "fel": 1, "build": 1}
    off_cfg = _cfg(Path("."))
    off_cfg.learning.fel_pattern_enabled = False
    capsys.readouterr()
    off = _run_step2_with_stubs(monkeypatch, off_cfg)
    assert off == {"prepare": 0, "fel": 0, "build": 1}  # 2-2（モデル作成）までは走る
    out = capsys.readouterr().out
    assert "2-3" in out and "走らない" in out and "手順 2 完了" in out
