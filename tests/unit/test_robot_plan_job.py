# ruff: noqa: E501
"""計画だけを作るジョブ（--plan-only）・計画ファイルの読み込み（--plan-in）・引数の検査（段4）。

計画の作成（約 30 秒）は偽物に差し替えて速くする。ハードは触らない。
"""

from __future__ import annotations

import argparse
import asyncio
from pathlib import Path
from types import SimpleNamespace

import pytest

from src.robot import events, plan_job, run
from src.robot.core.config import ConfigError


def _args(tmp_path: Path, **over) -> argparse.Namespace:
    base = dict(vehicle="car_a", mode="WLTP", plan_out=tmp_path / "p.pkl", vehicles_dir=tmp_path, common=tmp_path / "c.yaml")
    base.update(over)
    return argparse.Namespace(**base)


@pytest.fixture
def fake_planning(monkeypatch: pytest.MonkeyPatch):
    cfg = SimpleNamespace(modes=SimpleNamespace(composed_modes=[]))
    monkeypatch.setattr(plan_job.config_store, "load_vehicle_config", lambda *a, **k: (cfg, "merged"))

    async def load_mode(_cfg, name):
        return SimpleNamespace(name=name)

    monkeypatch.setattr(plan_job, "load_mode", load_mode)
    monkeypatch.setattr(plan_job, "prepare_mode_drive_from", lambda _cfg, mode: {"plan_for": mode.name})
    monkeypatch.setattr("src.robot.core.mode_drive.resolve_database_url", lambda _cfg: "postgresql://x/y")

    async def fetch(_url):
        return [("id-1", "WLTP")]

    return fetch


def _run_plan(args, fetch) -> tuple[int, list[dict]]:
    sink_lines: list[dict] = []

    class Sink(events.EventSink):
        def emit(self, type_, **fields):  # noqa: ANN001, ANN201
            sink_lines.append({"type": type_, **fields})

    events.set_sink(Sink())
    try:
        code = asyncio.run(plan_job.run_plan_only(args, pick_mode_name_fn=run.pick_mode_name, fetch_rows_fn=fetch))
    finally:
        events.set_sink(events.EventSink())
    return code, sink_lines


def test_plan_only_writes_loadable_file_and_events(tmp_path: Path, fake_planning) -> None:
    args = _args(tmp_path, mode="id-1")  # ID でも名前に解決される
    code, evs = _run_plan(args, fake_planning)
    assert code == 0 and args.plan_out.exists() and not args.plan_out.with_name("p.pkl.tmp").exists()
    phases = [e.get("phase") for e in evs if e["type"] == "state"]
    assert phases[0] == "planning" and "planned" in phases and phases[-1] == "done"
    setup, created = plan_job.read_plan_file(args.plan_out, vehicle="car_a", mode="WLTP")
    assert setup == {"plan_for": "WLTP"} and created


def test_plan_only_unknown_mode_exits_2_without_file(tmp_path: Path, fake_planning) -> None:
    args = _args(tmp_path, mode="NOPE")
    code, _ = _run_plan(args, fake_planning)
    assert code == 2 and not args.plan_out.exists()


def test_plan_only_unexpected_error_exits_5(tmp_path: Path, fake_planning, monkeypatch: pytest.MonkeyPatch) -> None:
    def boom(_cfg, _mode):
        raise RuntimeError("壊れた")

    monkeypatch.setattr(plan_job, "prepare_mode_drive_from", boom)
    args = _args(tmp_path, mode="WLTP")
    code, _ = _run_plan(args, fake_planning)
    assert code == 5 and not args.plan_out.exists()


def test_read_plan_file_rejects_mismatch(tmp_path: Path) -> None:
    out = tmp_path / "p.pkl"
    plan_job.write_plan_file(out, vehicle="car_a", mode="WLTP", setup={"x": 1})
    assert plan_job.read_plan_file(out, vehicle="car_a", mode="WLTP")[0] == {"x": 1}
    with pytest.raises(ConfigError, match="車ID"):
        plan_job.read_plan_file(out, vehicle="car_b", mode="WLTP")
    with pytest.raises(ConfigError, match="モード"):
        plan_job.read_plan_file(out, vehicle="car_a", mode="UDDS")
    with pytest.raises(ConfigError, match="読めません"):
        plan_job.read_plan_file(tmp_path / "none.pkl", vehicle="car_a", mode="WLTP")


def test_plan_in_prepare_sets_mode_setup(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "p.pkl"
    plan_job.write_plan_file(out, vehicle="car_a", mode="WLTP", setup={"x": 1})
    ctx = SimpleNamespace(mode_setup=None)
    asyncio.run(plan_job.make_plan_in_prepare(out, vehicle="car_a", mode="WLTP")(ctx))
    assert ctx.mode_setup == {"x": 1}
    assert "計画をファイルから読み込みました: WLTP（作成 " in capsys.readouterr().out
    with pytest.raises(ConfigError):
        asyncio.run(plan_job.make_plan_in_prepare(out, vehicle="car_a", mode="UDDS")(ctx))


def test_parser_options_and_main_checks(tmp_path: Path) -> None:
    p = run.build_parser().parse_args(
        ["--vehicle", "v", "--steps", "24", "--plan-in", "/x.pkl", "--no-pre-check", "--mode", "M"]
    )
    assert p.plan_in == Path("/x.pkl") and p.no_pre_check is True and p.plan_only is False
    for bad in (
        ["--vehicle", "v", "--plan-only", "--plan-out", "/x"],  # --mode が無い
        ["--vehicle", "v", "--plan-only", "--mode", "M"],  # --plan-out が無い
        ["--vehicle", "v", "--plan-only", "--mode", "M", "--plan-out", "/x", "--steps", "24"],
        ["--vehicle", "v"],  # どれも無い
        ["--vehicle", "v", "--steps", "24", "--plan-out", "/x"],
    ):
        with pytest.raises(SystemExit) as exc:
            run.main(bad)
        assert exc.value.code == 2
