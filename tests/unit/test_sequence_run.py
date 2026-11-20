# ruff: noqa: E501
"""シーケンスの進め方（JobManager.start_sequence）と、スケジュールのジョブを偽ランナーで確かめる。"""

from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any

import pytest

from src.app import job_manager as jm
from tests.unit.test_job_manager import (  # 記録先・偽ランナーの部品
    FakeRecorder,
    make_ready,
    runner_commands,
    runner_log,
    runner_starts,
    wait_until,
)


def sched(name: str) -> dict[str, Any]:
    return {"id": f"id-{name}", "name": name, "pedal_points": [{"time_s": 0, "accel_opening": 0, "brake_opening": 0}], "button_events": []}


async def make(tmp_path: Path, codes: str, **kw: Any) -> tuple[jm.JobManager, FakeRecorder, Path]:
    """初期化まで済んだ JobManager（偽の常駐ランナー）。ログは偽ランナーが受け取ったコマンド。"""
    rec = FakeRecorder()
    mgr = await make_ready(tmp_path, codes, rec, vehicle="car", **kw)
    return mgr, rec, runner_log(tmp_path)


def runs(tmp_path: Path) -> list[str]:
    """走らせた順の「種類 名前」（init は除く）。"""
    out = []
    for c in runner_commands(tmp_path):
        if c["cmd"] == "schedule":
            out.append(f"schedule {c['schedule']['name']}")
        elif c["cmd"] == "mode":
            out.append(f"mode_drive {c['mode']}@{c['limit_s']}")
    return out


def plan() -> list[jm.PlannedStep]:
    return [
        jm.PlannedStep("schedule", "a", "A", 2, schedule=sched("A")),
        jm.PlannedStep("mode", "m", "WLTP", 1, mode_id="mode-1"),
    ]


async def wait_sequence(mgr: jm.JobManager, timeout: float = 30.0) -> jm.SequenceRun:
    seq = mgr.sequence
    assert seq is not None and seq.task is not None
    await asyncio.wait_for(seq.task, timeout)
    await mgr.wait_persisted()  # DB への記録は裏で続くので待つ
    assert mgr.last_job is not None and mgr._last_sequence is seq  # noqa: SLF001
    return seq


# ── 純粋な部分 ────────────────────────────────────────────────


def test_expand_runs_order_and_numbers() -> None:
    expanded = jm.expand_runs(plan(), 2)
    got = [(r.cycle, r.step_no, r.step_repeat, r.step.name) for r in expanded]
    assert got == [
        (1, 1, 1, "A"), (1, 1, 2, "A"), (1, 2, 1, "WLTP"),
        (2, 1, 1, "A"), (2, 1, 2, "A"), (2, 2, 1, "WLTP"),
    ]


# ── 実行 ─────────────────────────────────────────────────────


async def test_sequence_runs_all_steps_in_order_one_session_each(tmp_path: Path) -> None:
    mgr, rec, _log = await make(tmp_path, "0")
    seq, first = await mgr.start_sequence("car", "sid", "S", plan(), 2, limit_s=30.0)
    assert first.sequence is seq and mgr.is_busy
    # 始めた直後の status/ws に進み具合が出る
    snap = mgr.snapshot()
    assert snap["job"]["sequence"]["run_no"] == 1 and snap["job"]["sequence"]["runs_total"] == 6
    assert snap["sequence"]["step_no"] == 1 and snap["sequence"]["cycles"] == 2
    # 実行中の別の開始・シーケンスは 409
    with pytest.raises(jm.JobConflict):
        await mgr.start_init_only("car")
    with pytest.raises(jm.JobConflict):
        await mgr.start_sequence("car", "x", "X", plan(), 1)
    done = await wait_sequence(mgr)
    assert done.state == jm.SEQ_COMPLETED and done.message is None
    assert not mgr.is_busy and mgr.state == jm.STATE_STANDBY and mgr.sequence is None
    # 順序・回数どおり（mode の段にだけ limit_s・モード名が渡る）
    assert runs(tmp_path) == ["schedule A", "schedule A", "mode_drive WLTP@30.0"] * 2
    # 常駐ランナーは 1 本のまま（段ごとに起動しない）
    assert runner_starts(tmp_path) == 1
    # 原点復帰は全体の最後の run だけ（段の間は停車保持のまま）
    assert [c["home_at_end"] for c in runner_commands(tmp_path)] == [False] * 5 + [True]
    # セッションは回ごとに 1 つ・全部 completed・run_type auto・mode は mode の段だけ
    assert len(rec.sessions) == 6 and all(v["status"] == "completed" for v in rec.sessions.values())
    assert [v["mode_id"] for v in rec.sessions.values()] == [None, None, "mode-1"] * 2
    assert {v["run_type"] for v in rec.sessions.values()} == {"auto"}
    assert [r["final_status"] for r in done.results] == ["completed"] * 6
    d = done.to_dict()
    assert d["run_no"] == 6 and d["state"] == "completed" and len(d["session_ids"]) == 6
    assert mgr.snapshot()["last_sequence"]["state"] == "completed"
    assert mgr.initialized
    await mgr.shutdown()


async def test_sequence_stops_at_first_failure(tmp_path: Path) -> None:
    mgr, rec, _log = await make(tmp_path, "0,5")  # 2 回目が走行エラー（5）
    await mgr.start_sequence("car", "sid", "S", plan(), 2)
    done = await wait_sequence(mgr)
    assert done.state == jm.SEQ_ERROR
    assert "1 段目「A」（2/2 回目" in (done.message or "") and "終了コード 5" in (done.message or "")
    assert len(runs(tmp_path)) == 2  # 残りは走らせない
    assert [v["status"] for v in rec.sessions.values()] == ["completed", "error"]
    assert not mgr.is_busy and mgr.state == jm.STATE_STANDBY
    await mgr.shutdown()


async def test_sequence_stop_during_step(tmp_path: Path) -> None:
    mgr, rec, _log = await make(tmp_path, "0,h")  # 2 回目は stop が来るまで居座る
    await mgr.start_sequence("car", "sid", "S", plan(), 1)
    await wait_until(lambda: len(runs(tmp_path)) >= 2)
    await asyncio.sleep(0.3)
    assert mgr.current is not None
    await mgr.stop()
    done = await wait_sequence(mgr)
    assert done.state == jm.SEQ_STOPPED and "停止" in (done.message or "")
    assert len(runs(tmp_path)) == 2
    assert mgr.state == jm.STATE_STANDBY and mgr.snapshot()["emergency_cause"] is None  # stop は非常停止の状態にしない
    assert runner_starts(tmp_path) == 1 and mgr.initialized
    await mgr.shutdown()


async def test_sequence_stop_between_steps_does_not_start_next(tmp_path: Path) -> None:
    mgr, _rec, _log = await make(tmp_path, "w")
    seq, _first = await mgr.start_sequence("car", "sid", "S", plan(), 1)
    await wait_until(lambda: len(runs(tmp_path)) >= 1)
    await asyncio.sleep(0.2)
    await mgr.stop()  # 1 回目の最中に停止要求（ジョブは stop を無視して 0 で終わる → 次の段は始めない）
    done = await wait_sequence(mgr)
    assert done.state in (jm.SEQ_STOPPED,)
    assert len(runs(tmp_path)) == 1 and seq is done
    await mgr.shutdown()


async def test_emergency_stops_whole_sequence(tmp_path: Path) -> None:
    mgr, rec, _log = await make(tmp_path, "0,h")
    await mgr.start_sequence("car", "sid", "S", plan(), 1)
    await wait_until(lambda: len(runs(tmp_path)) >= 2)
    await asyncio.sleep(0.3)
    await mgr.emergency()
    done = await wait_sequence(mgr)
    assert done.state == jm.SEQ_EMERGENCY and "非常停止" in (done.message or "")
    assert len(runs(tmp_path)) == 2  # 残り（mode）は走らせない
    assert list(rec.sessions.values())[-1]["status"] == "emergency"
    assert mgr.state == jm.STATE_EMERGENCY and mgr.initialized is False
    with pytest.raises(jm.JobConflict):
        await mgr.start_sequence("car", "sid", "S", plan(), 1)  # 解除するまで始められない
    await wait_until(lambda: mgr._runner is None)  # noqa: SLF001 - SIGINT でランナーは終わった
    await mgr.reset_emergency()
    assert mgr.state == jm.STATE_STANDBY


async def test_start_failure_leaves_manager_idle(tmp_path: Path) -> None:
    mgr, rec, _log = await make(tmp_path, "0")

    async def bad_session(**_k: Any) -> str:
        raise RuntimeError("DB 停止")

    rec.start_session = bad_session  # type: ignore[method-assign]
    with pytest.raises(jm.JobUnavailable):
        await mgr.start_sequence("car", "sid", "S", plan(), 1)
    assert mgr.sequence is None and not mgr.is_busy
    with pytest.raises(jm.JobUnavailable):
        await mgr.start_schedule("car", sched("A"))
    await mgr.shutdown()


async def test_single_schedule_job(tmp_path: Path) -> None:
    mgr, rec, _log = await make(tmp_path, "0")
    status = await mgr.start_schedule("car", sched("単発"))
    assert status.spec.kind == jm.KIND_SCHEDULE and status.spec.schedule_name == "単発" and status.sequence is None
    await mgr.wait_idle(30)
    assert mgr.last_job is not None and mgr.last_job.final_status == "completed"
    assert list(rec.sessions.values()) == [{"vehicle_id": "car", "mode_id": None, "run_type": "auto", "status": "completed"}]
    cmd = runner_commands(tmp_path)[-1]
    assert cmd["cmd"] == "schedule" and cmd["schedule"]["name"] == "単発" and cmd["home_at_end"] is True  # 単独は原点復帰する
    await mgr.shutdown()
