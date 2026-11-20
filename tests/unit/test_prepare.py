# ruff: noqa: E501
"""走行の準備（prepare。開始ポップアップの間に計画を作る）: JobManager の状態遷移と /drive/prepare の契約。

計画プロセスは偽（tests/unit/fakes/fake_plan.py）、常駐ランナーは偽（fake_serve_runner.py）。
"""

from __future__ import annotations

import asyncio
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import pytest
from httpx import ASGITransport, AsyncClient

from src.app import job_manager as jm
from src.app.inmemory_repos import (
    InMemoryModeRepository,
    InMemoryScheduleRepository,
    InMemorySequenceRepository,
)
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.web.app import app
from tests.unit.test_job_manager import FakeRecorder, runner_commands, runner_log

FAKE_PLAN = str(Path(__file__).parent / "fakes" / "fake_plan.py")
FAKE_SERVE = str(Path(__file__).parent / "fakes" / "fake_serve_runner.py")


def sched(name: str) -> dict[str, Any]:
    return {"id": f"id-{name}", "name": name, "pedal_points": [{"time_s": 0, "accel_opening": 0, "brake_opening": 0}], "button_events": []}


async def make(tmp_path: Path, behavior: str = "ok", codes: str = "0", *, ready: bool = True) -> tuple[jm.JobManager, Path, Path]:
    """計画プロセスは偽（fake_plan.py）、ランナーは偽の常駐ランナー。ready なら初期化まで済ませる。"""
    plan_log = tmp_path / "plans.log"

    def plan_factory(vehicle: str, mode: str, out: Path) -> list[str]:
        return [sys.executable, FAKE_PLAN, str(out), mode, behavior, str(plan_log)]

    mgr = jm.JobManager(
        vehicles_dir=tmp_path / "vehicles", common_path=tmp_path / "c.yaml", recorder=FakeRecorder(),
        command_factory=lambda vehicle_id: [sys.executable, FAKE_SERVE, str(runner_log(tmp_path)), codes],
        plan_command_factory=plan_factory, kill_timeout_s=5.0,
    )
    if ready:
        await mgr.start_init_only("v1")
        await mgr.wait_idle(30)
    return mgr, plan_log, runner_log(tmp_path)


def run_lines(tmp_path: Path) -> list[str]:
    """走らせたジョブの「種類 名前|plan=..|nopre=..」（init は除く）。"""
    out = []
    for c in runner_commands(tmp_path):
        if c["cmd"] == "mode":
            out.append(f"mode_drive {c['mode']}|plan={c['plan_in'] is not None}|nopre={c['no_pre_check']}")
        elif c["cmd"] == "schedule":
            out.append(f"schedule {c['schedule']['name']}|plan=False|nopre={c['no_pre_check']}")
    return out


async def wait_state(mgr: jm.JobManager, state: str, timeout: float = 15.0) -> jm.PrepareRun:
    prep = mgr.prepare
    assert prep is not None
    for _ in range(int(timeout / 0.05)):
        if prep.state == state:
            return prep
        await asyncio.sleep(0.05)
    raise AssertionError(f"state={prep.state} (want {state})")


def lines(path: Path) -> list[str]:
    return path.read_text().splitlines() if path.exists() else []


# ── コマンドの組み立て ────────────────────────────────────────


def test_build_commands(tmp_path: Path) -> None:
    pc = jm.build_plan_command("v1", "M", tmp_path / "1.pkl", vehicles_dir=tmp_path, common_path=tmp_path / "c.yaml", python="py")
    assert pc[pc.index("--plan-only") :][:5] == ["--plan-only", "--mode", "M", "--plan-out", str(tmp_path / "1.pkl")]
    assert "--steps" not in pc


# ── 状態遷移 ─────────────────────────────────────────────────


async def test_mode_prepare_ready_then_go_starts_with_plan_in_and_cleans(tmp_path: Path) -> None:
    mgr, plan_log, run_log = await make(tmp_path)
    prep = await mgr.start_prepare_mode("v1", "mid", "WLTP")
    assert prep.state == jm.PREP_PLANNING and prep.total == 1
    assert mgr.snapshot()["prepare"]["state"] == jm.PREP_PLANNING
    with pytest.raises(jm.JobConflict):  # 準備中は他の開始を弾く
        await mgr.start_init_only("v1")
    with pytest.raises(jm.JobConflict):  # 準備は同時に 1 つ
        await mgr.start_prepare_mode("v1", "mid", "WLTP")
    await wait_state(mgr, jm.PREP_READY)
    assert prep.done == 1 and Path(prep.plan_files["WLTP"]).exists()
    state, job_id = await mgr.prepare_go()
    assert state == "started" and job_id
    assert mgr.snapshot()["prepare"] is None
    await mgr.wait_idle(15)
    assert run_lines(tmp_path) == ["mode_drive WLTP|plan=True|nopre=False"]
    assert not prep.plan_dir.exists()  # ジョブの終わりに消える


async def test_go_while_planning_waits_then_starts(tmp_path: Path) -> None:
    mgr, _, run_log = await make(tmp_path, "slow")
    prep = await mgr.start_prepare_mode("v1", "mid", "WLTP")
    state, job_id = await mgr.prepare_go()
    assert (state, job_id) == ("waiting", None)
    assert mgr.snapshot()["prepare"]["state"] == jm.PREP_WAITING
    for _ in range(300):
        if mgr.snapshot()["prepare"] is None:
            break
        await asyncio.sleep(0.05)
    assert mgr.snapshot()["prepare"] is None
    await mgr.wait_idle(15)
    assert run_lines(tmp_path) == ["mode_drive WLTP|plan=True|nopre=False"] and not prep.plan_dir.exists()


async def test_plan_failure_sets_error_and_nothing_starts(tmp_path: Path) -> None:
    mgr, _, _ = await make(tmp_path, "fail")
    prep = await mgr.start_prepare_mode("v1", "mid", "WLTP")
    await mgr.prepare_go()  # 待ち状態へ
    await wait_state(mgr, jm.PREP_ERROR)
    assert "WLTP" in (prep.message or "") and "がありません" in (prep.message or "")
    assert mgr.current is None and run_lines(tmp_path) == []
    with pytest.raises(jm.JobConflict):  # 失敗した準備は始められない
        await mgr.prepare_go()
    assert await mgr.cancel_prepare() is True and mgr.prepare is None


async def test_cancel_stops_plan_process_and_removes_files(tmp_path: Path) -> None:
    mgr, plan_log, _ = await make(tmp_path, "hang")
    prep = await mgr.start_prepare_mode("v1", "mid", "WLTP")
    for _ in range(100):  # 計画プロセスが起動するまで
        if prep.proc is not None and lines(plan_log):
            break
        await asyncio.sleep(0.05)
    proc = prep.proc
    assert proc is not None
    await mgr.cancel_prepare()
    assert proc.returncode is not None and mgr.prepare is None and not prep.plan_dir.exists()
    assert await mgr.cancel_prepare() is False  # 無ければ何もしない
    await mgr.start_init_only("v1")  # 取り消した後は始められる
    await mgr.wait_idle(15)


async def test_emergency_cancels_prepare(tmp_path: Path) -> None:
    mgr, _, _ = await make(tmp_path, "hang")
    prep = await mgr.start_prepare_mode("v1", "mid", "WLTP")
    await asyncio.sleep(0.3)
    await mgr.emergency()
    assert mgr.prepare is None and not prep.plan_dir.exists()


async def test_schedule_prepare_is_ready_immediately(tmp_path: Path) -> None:
    mgr, plan_log, run_log = await make(tmp_path)
    prep = await mgr.start_prepare_schedule("v1", sched("S"))
    assert (prep.state, prep.done, prep.total) == (jm.PREP_READY, 0, 0)
    state, job_id = await mgr.prepare_go()
    assert state == "started" and job_id
    await mgr.wait_idle(15)
    assert run_lines(tmp_path) == ["schedule S|plan=False|nopre=False"] and not plan_log.exists()


async def test_prepare_requires_init_for_mode_schedule_and_sequence(tmp_path: Path) -> None:
    mgr, _, _ = await make(tmp_path, ready=False)
    with pytest.raises(jm.JobConflict):
        await mgr.start_prepare_mode("v1", "mid", "WLTP")
    assert mgr.prepare is None
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_prepare_schedule("v1", sched("S"))
    steps = [jm.PlannedStep("schedule", "s", "S", 1, schedule=sched("S"))]
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_prepare_sequence("v1", "seq", "SEQ", steps, 1)
    assert mgr.prepare is None
    mgr, _, _ = await make(tmp_path)  # 初期化済みなら準備できる
    await mgr.start_prepare_schedule("v1", sched("S"))
    assert await mgr.cancel_prepare() is True


async def test_sequence_prepare_plans_distinct_modes_once_and_skips_pre_check_after_first(tmp_path: Path) -> None:
    mgr, plan_log, run_log = await make(tmp_path)
    steps = [
        jm.PlannedStep("mode", "m1", "WLTP", 2, mode_id="m1"),
        jm.PlannedStep("schedule", "s", "S", 1, schedule=sched("S")),
        jm.PlannedStep("mode", "m2", "UDDS", 1, mode_id="m2"),
        jm.PlannedStep("mode", "m1", "WLTP", 1, mode_id="m1"),
    ]
    prep = await mgr.start_prepare_sequence("v1", "seq1", "SEQ", steps, 1)
    assert prep.total == 2  # 違うモードごとに 1 回
    await wait_state(mgr, jm.PREP_READY)
    assert prep.done == 2 and sorted(lines(plan_log)) == ["UDDS", "WLTP"] and lines(plan_log)[0] == "WLTP"
    state, job_id = await mgr.prepare_go()
    assert state == "started" and job_id and mgr.snapshot()["prepare"] is None
    seq = mgr.sequence
    assert seq is not None and seq.task is not None
    await asyncio.wait_for(seq.task, 30)
    assert run_lines(tmp_path) == [
        "mode_drive WLTP|plan=True|nopre=False",
        "mode_drive WLTP|plan=True|nopre=True",
        "schedule S|plan=False|nopre=True",
        "mode_drive UDDS|plan=True|nopre=True",
        "mode_drive WLTP|plan=True|nopre=True",
    ]
    assert not prep.plan_dir.exists()  # シーケンスの終わりに消える


async def test_unprepared_sequence_is_unchanged(tmp_path: Path) -> None:
    mgr, _, run_log = await make(tmp_path)
    steps = [jm.PlannedStep("mode", "m1", "WLTP", 2, mode_id="m1")]
    seq, _ = await mgr.start_sequence("v1", "s1", "SEQ", steps, 1)
    assert seq.task is not None
    await asyncio.wait_for(seq.task, 30)
    assert run_lines(tmp_path) == ["mode_drive WLTP|plan=False|nopre=False"] * 2


# ── /api/v1/drive/prepare の契約 ───────────────────────────────

MODE_ID = "00000000-0000-0000-0000-0000000000a1"


class _Store:
    common_path = Path("/nonexistent/common.yaml")

    def get_active(self) -> str:
        return "v1"

    def exists(self, vehicle_id: str) -> bool:
        return False


@pytest.fixture
async def client(tmp_path: Path):
    mgr, _, _ = await make(tmp_path, "slow")
    repo = InMemoryModeRepository()
    await repo.create(
        DrivingMode(
            id=MODE_ID, name="モードA", description="",
            reference_speed=[SpeedPoint(time_s=0.0, speed_kmh=0.0)],
            total_duration=100.0, max_speed=0.0, created_at=datetime.now(tz=UTC),
        )
    )
    keys = ("job_manager", "vehicle_store", "mode_repo", "schedule_repo", "sequence_repo")
    saved = {k: getattr(app.state, k, None) for k in keys}
    app.state.job_manager, app.state.vehicle_store, app.state.mode_repo = mgr, _Store(), repo
    app.state.schedule_repo, app.state.sequence_repo = InMemoryScheduleRepository(), InMemorySequenceRepository()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        yield c
    await mgr.cancel_prepare()
    await mgr.shutdown()
    for k, v in saved.items():
        setattr(app.state, k, v)


async def test_api_prepare_flow(client: AsyncClient) -> None:
    res = await client.post("/api/v1/drive/prepare", json={"kind": "mode", "id": MODE_ID})
    assert res.status_code == 200
    body = res.json()
    assert set(body) == {"prepare_id", "state", "done", "total"} and body["state"] == "planning" and body["total"] == 1
    st = (await client.get("/api/v1/drive/status")).json()["prepare"]
    assert set(st) == {"id", "kind", "name", "state", "done", "total", "message"}
    assert st["id"] == body["prepare_id"] and st["kind"] == "mode" and st["name"] == "モードA"
    # 準備中は他の開始・二重の準備が 409
    assert (await client.post("/api/v1/drive/initialize")).status_code == 409
    assert (await client.post("/api/v1/drive/prepare", json={"kind": "mode", "id": MODE_ID})).status_code == 409
    go = await client.post("/api/v1/drive/prepare/go")
    assert go.status_code == 200 and go.json()["state"] == "waiting" and go.json()["job_id"] is None
    for _ in range(300):
        if (await client.get("/api/v1/drive/status")).json()["prepare"] is None:
            break
        await asyncio.sleep(0.05)
    assert (await client.get("/api/v1/drive/status")).json()["prepare"] is None


async def test_api_prepare_cancel_and_404(client: AsyncClient) -> None:
    assert (await client.post("/api/v1/drive/prepare", json={"kind": "mode", "id": "00000000-0000-0000-0000-0000000000ff"})).status_code == 404
    assert (await client.post("/api/v1/drive/prepare", json={"kind": "bogus", "id": "x"})).status_code == 422
    assert (await client.post("/api/v1/drive/prepare/cancel")).json() == {"status": "cancelled"}  # 無くても 200
    await client.post("/api/v1/drive/prepare", json={"kind": "mode", "id": MODE_ID})
    assert (await client.post("/api/v1/drive/prepare/cancel")).json() == {"status": "cancelled"}
    assert (await client.get("/api/v1/drive/status")).json()["prepare"] is None
    assert (await client.post("/api/v1/drive/prepare/go")).status_code == 409  # 準備が無い
