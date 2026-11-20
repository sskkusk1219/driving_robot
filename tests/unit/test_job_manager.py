# ruff: noqa: E501
"""JobManager（src/app/job_manager.py）。偽の常駐ランナー（tests/unit/fakes/fake_serve_runner.py）でサブプロセスごと確かめる。"""

from __future__ import annotations

import asyncio
import json
import sys
import time
from collections.abc import Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

import pytest

from src.app import job_manager as jm
from src.infra.drive_recorder import MessageRow

FAKE = str(Path(__file__).parent / "fakes" / "fake_serve_runner.py")


# ── 記録先（DB の代わり） ───────────────────────────────────────


class FakeRecorder:
    def __init__(self, fail_import: bool = False) -> None:
        self.sessions: dict[str, dict[str, Any]] = {}
        self.messages: dict[str, list[MessageRow]] = {}
        self.imported: list[tuple[str, str]] = []
        self.fail_import = fail_import

    async def start_session(
        self,
        *,
        vehicle_id: str,
        mode_id: str | None,
        run_type: str,
        schedule_id: str | None = None,
        schedule_name: str | None = None,
    ) -> str | None:
        sid = f"sess{len(self.sessions) + 1}"
        self.sessions[sid] = {
            "vehicle_id": vehicle_id, "mode_id": mode_id, "run_type": run_type, "status": "running",
            "schedule_id": schedule_id, "schedule_name": schedule_name,
        }
        return sid

    async def add_messages(self, session_id: str, rows: Sequence[MessageRow]) -> None:
        self.messages.setdefault(session_id, []).extend(rows)

    async def import_csv(self, session_id: str, path: Path) -> int:
        if self.fail_import:
            raise RuntimeError("DB が落ちた")
        self.imported.append((session_id, path.name))
        return sum(1 for _ in path.open()) - 1

    async def end_session(self, session_id: str, status: str) -> None:
        self.sessions[session_id]["status"] = status


def runner_log(tmp_path: Path) -> Path:
    return tmp_path / "serve.log"


def make_manager(
    tmp_path: Path, codes: str = "", recorder: FakeRecorder | None = None, **kw: Any
) -> jm.JobManager:
    """偽の常駐ランナーを使う JobManager（まだ初期化していない）。codes は fake_serve_runner の CODES。"""
    return jm.JobManager(
        vehicles_dir=tmp_path,
        common_path=tmp_path / "common.yaml",
        recorder=recorder or FakeRecorder(),
        command_factory=lambda vehicle_id: [sys.executable, FAKE, str(runner_log(tmp_path)), codes],
        **kw,
    )


async def make_ready(
    tmp_path: Path, codes: str = "", recorder: FakeRecorder | None = None, *, vehicle: str = "v1", **kw: Any
) -> jm.JobManager:
    """初期化まで済んだ JobManager（常駐ランナーが 1 本起動している）。"""
    mgr = make_manager(tmp_path, codes, recorder, **kw)
    await mgr.start_init_only(vehicle)
    await mgr.wait_idle(30)
    assert mgr.initialized
    return mgr


def runner_lines(tmp_path: Path) -> list[str]:
    p = runner_log(tmp_path)
    return p.read_text().splitlines() if p.exists() else []


def runner_commands(tmp_path: Path) -> list[dict[str, Any]]:
    """偽ランナーが受け取ったコマンド（START の行は除く）。"""
    return [json.loads(x) for x in runner_lines(tmp_path) if x != "START"]


def runner_starts(tmp_path: Path) -> int:
    return runner_lines(tmp_path).count("START")


async def wait_until(cond: Any, timeout: float = 10.0) -> None:
    for _ in range(int(timeout / 0.05)):
        if cond():
            return
        await asyncio.sleep(0.05)
    raise AssertionError("条件が満たされません")


def write_csv(path: Path, n: int = 3) -> Path:
    path.write_text("timestamp,x\n" + "".join(f"2026-11-15 09:00:0{i},1\n" for i in range(n)), encoding="utf-8")
    path.with_suffix(".yaml").write_text("snapshot: 1\n", encoding="utf-8")
    return path


# ── 純粋な部分 ────────────────────────────────────────────────


def test_mode_drive_steps_have_no_init_step() -> None:
    # D-15c: 手順 1（初期化）は初期化ページだけ。モード走行は 24,25（Ready・D・モード・N → IGN-OFF）。手順 3 は使わない
    assert jm.MODE_DRIVE_STEPS == "24,25"


def test_end_meaning_prefers_runner_result_meaning() -> None:
    # B-3: stop コマンドの終わり（130）は、ランナーの result の meaning（中断（stop コマンド））を使う
    from types import SimpleNamespace

    stopped = SimpleNamespace(exit_code=130, result={"meaning": "中断（stop コマンド）。終了処理は済ませた"})
    assert jm.JobManager._end_meaning(stopped) == "中断（stop コマンド）。終了処理は済ませた"
    plain = SimpleNamespace(exit_code=130, result=None)
    assert "SIGINT" in jm.JobManager._end_meaning(plain)
    assert jm.JobManager._end_meaning(SimpleNamespace(exit_code=None, result=None)) == "不明"


def test_learning_steps_have_no_init_step() -> None:
    assert jm.LEARNING_STEPS == {
        "full": "2,25", "fel": "12,25", "hold": "13,25", "stop": "14,25",
    }
    ign = jm.JobManager.ign_off_spec("v1")
    assert ign.steps == "25" and ign.kind == jm.KIND_IGN_OFF
    assert jm.JobManager.init_only_spec("v1").steps == "1"  # 手順 1 を行うのはこのジョブだけ


def _ended_status(kind: str, steps: str) -> jm.JobStatus:
    from datetime import datetime

    status = jm.JobStatus(job_id="j", spec=jm.JobSpec(kind=kind, vehicle_id="v", steps=steps), started_at=datetime.now(), t0=0.0)
    return status


def test_needs_ign_off_only_when_buttons_used_and_step25_not_done() -> None:
    from datetime import datetime

    def feed(status: jm.JobStatus, *events: dict) -> None:
        for e in events:
            jm.apply_event(status, {"type": "state", **e})

    # 手順 1 だけで失敗: 車はそのまま
    s = _ended_status(jm.KIND_MODE_DRIVE, jm.MODE_DRIVE_STEPS)
    feed(s, {"phase": "step", "step": 1}, {"phase": "step_end", "step": 1, "ok": False})
    s.ended_at = datetime.now()
    assert not s.needs_ign_off
    # 手順 24 まで行ってエラー（25 を押していない）→ 要る
    s = _ended_status(jm.KIND_MODE_DRIVE, jm.MODE_DRIVE_STEPS)
    feed(s, {"phase": "step", "step": 21}, {"phase": "step_end", "step": 21, "ok": True}, {"phase": "step", "step": 24})
    assert not s.needs_ign_off  # 終わっていない間は出さない
    s.ended_at = datetime.now()
    assert s.needs_ign_off and s.to_dict()["needs_ign_off"] is True
    # 学習で Ready・D の後に中断
    s = _ended_status(jm.KIND_LEARNING, jm.LEARNING_STEPS["full"])
    feed(s, {"phase": "ready", "step": None}, {"phase": "d"})
    s.ended_at = datetime.now()
    assert s.needs_ign_off
    # 手順 25 まで正常に終わった → 要らない
    s = _ended_status(jm.KIND_LEARNING, jm.LEARNING_STEPS["full"])
    feed(s, {"phase": "step", "step": 21}, {"phase": "step", "step": 25}, {"phase": "step_end", "step": 25, "ok": True})
    s.ended_at = datetime.now()
    assert not s.needs_ign_off
    # IGN-OFF だけのジョブ・初期化・スケジュールは対象外
    for kind in (jm.KIND_IGN_OFF, jm.KIND_INIT_ONLY, jm.KIND_SCHEDULE):
        s = _ended_status(kind, "25")
        feed(s, {"phase": "step", "step": 21})
        s.ended_at = datetime.now()
        assert not s.needs_ign_off


def test_hw_mode_from_env() -> None:
    assert jm.hw_mode_from_env({}) == "stub"
    assert jm.hw_mode_from_env({"DRIVING_ROBOT_USE_REAL_HW": "1"}) == "real"
    # ベンチ（GPIO だけ実機）ではランナーは stub
    assert jm.hw_mode_from_env({"DRIVING_ROBOT_USE_REAL_HW": "1", "DRIVING_ROBOT_BENCH_GPIO_ONLY": "1"}) == "stub"
    assert jm.hw_mode_from_env({"DRIVING_ROBOT_USE_REAL_HW": "0"}) == "stub"


def test_build_serve_command_and_job_command(tmp_path: Path) -> None:
    cmd = jm.build_serve_command("v1", vehicles_dir=tmp_path, common_path=tmp_path / "c.yaml", hw="stub", python="py")
    assert cmd[:5] == ["py", "-m", "src.robot.run", "--vehicle", "v1"] and "--serve" in cmd
    assert cmd[cmd.index("--hw") + 1] == "stub"
    for gone in ("--steps", "--interactive", "--schedule-file", "--mode", "--plan-in"):
        assert gone not in cmd  # ジョブごとの指定はコマンドで送る
    mode = jm.JobSpec(jm.KIND_MODE_DRIVE, "v1", steps=jm.MODE_DRIVE_STEPS, mode_name="WLTP", limit_s=60.0, plan_file="/p.pkl", no_pre_check=True, home_at_end=False)
    assert jm.job_command(mode, "j1") == {
        "cmd": "mode", "job_id": "j1", "vehicle": "v1", "mode": "WLTP", "plan_in": "/p.pkl", "limit_s": 60.0, "no_pre_check": True, "home_at_end": False,
    }
    sched = jm.JobSpec(jm.KIND_SCHEDULE, "v1", schedule={"name": "S"})
    assert jm.job_command(sched, "j2") == {"cmd": "schedule", "job_id": "j2", "vehicle": "v1", "schedule": {"name": "S"}, "no_pre_check": False, "home_at_end": True}
    assert jm.job_command(jm.JobManager.learning_spec("v1", "full"), "j3") == {"cmd": "steps", "job_id": "j3", "vehicle": "v1", "steps": "2,25", "home_at_end": True}
    assert jm.job_command(jm.JobManager.ign_off_spec("v1"), "j4")["steps"] == "25"
    assert jm.job_command(jm.JobManager.interactive_spec("v1", "calib"), "j5") == {"cmd": "interactive", "job_id": "j5", "vehicle": "v1", "purpose": "calib"}
    assert jm.job_command(jm.JobManager.init_only_spec("v1"), "j6") == {"cmd": "init", "job_id": "j6", "vehicle": "v1"}


def test_run_type_by_kind() -> None:
    assert jm.JobSpec(jm.KIND_LEARNING, "v").run_type == "learning"
    assert jm.JobSpec(jm.KIND_MODE_DRIVE, "v").run_type == "auto"
    assert jm.JobSpec(jm.KIND_INTERACTIVE, "v", purpose="manual").run_type == "manual"
    assert jm.JobSpec(jm.KIND_INTERACTIVE, "v", purpose="calib").run_type is None
    assert jm.JobSpec(jm.KIND_INIT_ONLY, "v").run_type is None


def test_exit_to_status() -> None:
    assert jm.exit_to_status(0, emergency=False, interactive=False) == "completed"
    assert jm.exit_to_status(5, emergency=False, interactive=False) == "error"
    assert jm.exit_to_status(130, emergency=False, interactive=False) == "error"
    assert jm.exit_to_status(130, emergency=True, interactive=False) == "emergency"
    assert jm.exit_to_status(0, emergency=True, interactive=False) == "emergency"
    assert jm.exit_to_status(130, emergency=False, interactive=True) == "completed"
    assert jm.exit_to_status(4, emergency=False, interactive=True) == "error"
    assert jm.exit_to_status(None, emergency=False, interactive=False) == "error"
    # 利用者の走行終了（stop）で止めたジョブは stopped（130/143 だけ。他のコードは error・非常停止が優先）
    assert jm.exit_to_status(130, emergency=False, interactive=False, stopped_by_user=True) == "stopped"
    assert jm.exit_to_status(143, emergency=False, interactive=False, stopped_by_user=True) == "stopped"
    assert jm.exit_to_status(5, emergency=False, interactive=False, stopped_by_user=True) == "error"
    assert jm.exit_to_status(130, emergency=True, interactive=False, stopped_by_user=True) == "emergency"
    assert jm.exit_to_status(130, emergency=False, interactive=True, stopped_by_user=True) == "completed"


def _status(kind: str = jm.KIND_MODE_DRIVE, purpose: str | None = None) -> jm.JobStatus:
    from datetime import UTC

    return jm.JobStatus(
        job_id="j", spec=jm.JobSpec(kind, "v", purpose=purpose), started_at=datetime.now(UTC), t0=time.monotonic()
    )


def test_apply_event_state_init_sample_csv_result() -> None:
    st = _status()
    jm.apply_event(st, {"type": "state", "phase": "init", "step": 1, "title": "初期化"})
    assert (st.phase, st.step, st.step_title) == ("init", 1, "初期化")
    jm.apply_event(st, {"type": "init_step", "name": "通信確認", "status": "ok", "detail": ""})
    jm.apply_event(st, {"type": "init_step", "name": "原点復帰", "status": "ng", "detail": "x"})
    jm.apply_event(st, {"type": "init_step", "name": "通信確認", "status": "skip", "detail": ""})  # 同名は置き換え
    assert [(i["key"], i["status"]) for i in st.init_steps] == [("通信確認", "skipped"), ("原点復帰", "error")]
    jm.apply_event(st, {"type": "sample", "ts": 1.0, "speed_kmh": 5.0, "ref_kmh": 6.0, "section": "MODE_DRIVE"})
    assert st.sample == {"speed_kmh": 5.0, "ref_kmh": 6.0, "section": "MODE_DRIVE"}
    jm.apply_event(st, {"type": "csv", "path": "/x.csv", "kind": "drive", "rows": 10})
    assert st.csv_files == [{"path": "/x.csv", "kind": "drive", "rows": 10}]
    jm.apply_event(st, {"type": "state", "phase": "done", "exit_code": 5})
    jm.apply_event(st, {"type": "result", "exit_code": 5, "meaning": "走行エラー", "error": "boom", "elapsed_s": 3.0})
    assert st.exit_code == 5 and st.result["error"] == "boom"
    d = st.to_dict()
    assert d["result"]["meaning"] == "走行エラー" and d["kind"] == "mode_drive"


def test_derive_robot_state() -> None:
    assert jm.derive_robot_state(jm.STATE_STANDBY, None) == "READY"
    assert jm.derive_robot_state(jm.STATE_EMERGENCY, None) == "EMERGENCY"
    st = _status()
    assert jm.derive_robot_state(jm.STATE_RUNNING, st) == "INITIALIZING"  # starting
    st.phase = "init"
    assert jm.derive_robot_state(jm.STATE_RUNNING, st) == "INITIALIZING"
    st.phase = "step"
    assert jm.derive_robot_state(jm.STATE_RUNNING, st) == "RUNNING"
    st.phase = "pre_check"
    assert jm.derive_robot_state(jm.STATE_RUNNING, st) == "RUNNING"
    man, cal = _status(jm.KIND_INTERACTIVE, "manual"), _status(jm.KIND_INTERACTIVE, "calib")
    man.phase = cal.phase = "interactive"
    assert jm.derive_robot_state(jm.STATE_RUNNING, man) == "MANUAL"
    assert jm.derive_robot_state(jm.STATE_RUNNING, cal) == "CALIBRATING"
    assert jm.derive_robot_state(jm.STATE_EMERGENCY, st) == "EMERGENCY"


def test_message_level() -> None:
    assert jm.message_level("設定エラー: x") == "ERROR"
    assert jm.message_level("  [NG  ] 原点復帰") == "WARNING"
    assert jm.message_level("[  1.0s] 走行中") == "INFO"
    # 異常でない決まった文は INFO（2026-11-20）
    for text in (
        "エラー消去: アラームリセット (両軸同時) → 残留確認 …",
        "[OK  ] エラー消去 (アラームリセット)  … 残留アラームなし",
        "エラー消去 (アラームリセット)     : OK  残留アラームなし",
        "スケジュール再生 完了: 22.5s・451 周期（1 周期以上の遅れ 0 回・ボタン押下の失敗 0 回）…",
        "中止条件: 過電流・アラーム・電流ゼロの継続・CAN 読み失敗・通信失敗・最高速 150km/h 超え",
        "手順 1: 初期化（サーボ通信・エラー消去・CAN・UPS・初期位置）",
    ):
        assert jm.message_level(text) == "INFO", text
    # 本物のエラーは ERROR のまま
    for text in (
        "初期化エラー: x",
        "走行エラー: x",
        "  警告: N の操作に失敗しました（OSError: x）",
        "Traceback (most recent call last):",
        "ボタン押下の失敗 2 回",
    ):
        assert jm.message_level(text) == "ERROR", text


# ── 常駐ランナー ─────────────────────────────────────────────────


async def test_init_starts_one_resident_runner_and_second_init_sends_init_command(tmp_path: Path) -> None:
    mgr = make_manager(tmp_path)
    assert mgr.initialized is False
    st = await mgr.start_init_only("v1")
    assert st.session_id is None and mgr.state == jm.STATE_RUNNING
    await mgr.wait_idle(30)
    assert mgr.initialized is True and mgr.snapshot()["initialized"] is True and mgr.state == jm.STATE_STANDBY
    assert runner_starts(tmp_path) == 1
    last = mgr.last_job
    assert last is not None and last.exit_code == 0 and last.final_status == "completed"
    assert [i["key"] for i in last.init_steps] == ["通信確認", "原点復帰"]
    # 2 回目の初期化は新しく起動せず init コマンド
    await mgr.start_init_only("v1")
    await mgr.wait_idle(30)
    assert runner_starts(tmp_path) == 1 and [c["cmd"] for c in runner_commands(tmp_path)] == ["init"]
    assert mgr.initialized is True
    await mgr.shutdown()


async def test_init_failure_closes_runner_and_stays_uninitialized(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("FAKE_INIT_EXIT", "4")
    mgr = make_manager(tmp_path)
    await mgr.start_init_only("v1")
    await mgr.wait_idle(30)
    assert mgr.initialized is False and mgr.last_job.exit_code == 4 and mgr.last_job.final_status == "error"
    await wait_until(lambda: mgr._runner is None)  # noqa: SLF001
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_mode_drive("v1", "m", "WLTP")


async def test_mode_drive_job_completes_and_records(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rec = FakeRecorder()
    csv_path = write_csv(tmp_path / "drive_log_stub_x.csv")
    monkeypatch.setenv("FAKE_CSV", str(csv_path))
    mgr = await make_ready(tmp_path, "c", rec)
    status = await mgr.start_mode_drive("v1", "mode-id", "WLTP", 60.0)
    assert status.spec.steps == jm.MODE_DRIVE_STEPS and status.session_id == "sess1"
    assert mgr.state == jm.STATE_RUNNING and mgr.is_busy
    with pytest.raises(jm.JobConflict):  # 同時に 1 本
        await mgr.start_init_only("v1")
    await mgr.wait_idle(30)
    assert mgr.state == jm.STATE_STANDBY and not mgr.is_busy and mgr.initialized
    last = mgr.last_job
    assert last is not None and last.exit_code == 0 and last.final_status == "completed"
    assert rec.sessions["sess1"] == {"vehicle_id": "v1", "mode_id": "mode-id", "run_type": "auto", "status": "completed", "schedule_id": None, "schedule_name": None}
    texts = [m[4] for m in rec.messages["sess1"]]
    assert any("ジョブの終わり" in t for t in texts) and any("ジョブ終了" in t for t in texts)
    seqs = [m[0] for m in rec.messages["sess1"]]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs)
    assert rec.imported == [("sess1", "drive_log_stub_x.csv")]
    assert not csv_path.exists() and not csv_path.with_suffix(".yaml").exists()  # drive の CSV は取り込み後に消える
    cmd = runner_commands(tmp_path)[-1]
    assert cmd["cmd"] == "mode" and cmd["mode"] == "WLTP" and cmd["limit_s"] == 60.0 and cmd["home_at_end"] is True
    assert cmd["plan_in"] is None and cmd["no_pre_check"] is False and runner_starts(tmp_path) == 1


async def test_learning_csv_is_kept_and_uses_steps_command(tmp_path: Path) -> None:
    rec = FakeRecorder()
    mgr = await make_ready(tmp_path, "0", rec)
    s = await mgr.start_learning("v1", "hold")
    assert s.spec.steps == "13,25" and s.spec.run_type == "learning"
    await mgr.wait_idle(30)
    assert {k: v for k, v in runner_commands(tmp_path)[-1].items() if k != "job_id"} == {"cmd": "steps", "vehicle": "v1", "steps": "13,25", "home_at_end": True}
    assert mgr.mode_drive_spec("v1", "m", "M").steps == jm.MODE_DRIVE_STEPS
    with pytest.raises(jm.JobError):
        mgr.learning_spec("v1", "nope")


async def test_ign_off_job_sends_steps_25(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path, "0")
    await mgr.start_ign_off("v1")
    await mgr.wait_idle(30)
    assert runner_commands(tmp_path)[-1]["steps"] == "25" and mgr.last_job.final_status == "completed"


async def test_exit_code_nonzero_is_error_keeps_runner_and_csv_import_failure_keeps_file(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    rec = FakeRecorder(fail_import=True)
    csv_path = write_csv(tmp_path / "d.csv")
    monkeypatch.setenv("FAKE_CSV", str(csv_path))
    mgr = await make_ready(tmp_path, "5", rec)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await mgr.wait_idle(30)
    assert rec.sessions["sess1"]["status"] == "error"
    assert mgr.initialized and mgr.last_job.exit_code == 5  # ランナーが待機へ戻した（needs_init なし）ので初期化済みのまま


async def test_modes_schedule_and_sequence_do_not_start_more_runners(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path, "0,0,0,0")
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await mgr.wait_idle(30)
    await mgr.start_schedule("v1", {"id": "i", "name": "S", "pedal_points": [], "button_events": []})
    await mgr.wait_idle(30)
    steps = [jm.PlannedStep("schedule", "a", "A", 1, schedule={"id": "a", "name": "A"}), jm.PlannedStep("mode", "m", "WLTP", 1, mode_id="m")]
    seq, _ = await mgr.start_sequence("v1", "sid", "SEQ", steps, 1)
    assert seq.task is not None
    await asyncio.wait_for(seq.task, 30)
    assert seq.state == jm.SEQ_COMPLETED and runner_starts(tmp_path) == 1
    assert [c["cmd"] for c in runner_commands(tmp_path)] == ["mode", "schedule", "schedule", "mode"]
    await mgr.shutdown()


async def test_vehicle_switch_keeps_initialized_and_commands_carry_vehicle(tmp_path: Path) -> None:
    """初期化は機械の初期化: 車を切り替えても initialized は true・ランナーは起動し直さず、ジョブのコマンドは今の車で送る。"""
    mgr = await make_ready(tmp_path, "0,0,0")
    assert mgr.initialized is True and not hasattr(mgr, "initialized_for")
    await mgr.start_mode_drive("v2", "m", "WLTP")  # 車 v2 へ切り替えた後
    await mgr.wait_idle(30)
    assert mgr.initialized is True and runner_starts(tmp_path) == 1
    await mgr.start_schedule("v3", {"id": "i", "name": "S", "pedal_points": [], "button_events": []})
    await mgr.wait_idle(30)
    await mgr.start_init_only("v2")  # 車が違うだけでは起動し直さない（init コマンドを送る）
    await mgr.wait_idle(30)
    assert runner_starts(tmp_path) == 1 and mgr.initialized is True
    cmds = runner_commands(tmp_path)
    assert [(c["cmd"], c["vehicle"]) for c in cmds] == [("mode", "v2"), ("schedule", "v3"), ("init", "v2")]
    await mgr.shutdown()


def test_job_command_carries_vehicle_for_every_kind() -> None:
    for kind, steps in ((jm.KIND_MODE_DRIVE, None), (jm.KIND_SCHEDULE, None), (jm.KIND_LEARNING, "2"), (jm.KIND_INTERACTIVE, None), (jm.KIND_INIT_ONLY, "1")):
        spec = jm.JobSpec(kind=kind, vehicle_id="vX", steps=steps)
        assert jm.job_command(spec, "j1")["vehicle"] == "vX"


async def test_schedule_session_records_schedule_id_and_name(tmp_path: Path) -> None:
    """スケジュールのジョブのセッションに、スケジュールの ID・名前が渡る。モード走行は None（O-5）。"""
    rec = FakeRecorder()
    mgr = await make_ready(tmp_path, "0,0", rec)
    await mgr.start_schedule("v1", {"id": "sched-1", "name": "メンテナンスモード_v2", "pedal_points": [], "button_events": []})
    await mgr.wait_idle(30)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await mgr.wait_idle(30)
    assert rec.sessions["sess1"]["schedule_id"] == "sched-1" and rec.sessions["sess1"]["schedule_name"] == "メンテナンスモード_v2"
    assert rec.sessions["sess2"]["schedule_id"] is None and rec.sessions["sess2"]["schedule_name"] is None
    await mgr.shutdown()


async def test_stop_sends_stop_command_and_does_not_enter_emergency(tmp_path: Path) -> None:
    rec = FakeRecorder()
    mgr = await make_ready(tmp_path, "h", rec)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await wait_until(lambda: mgr.current is not None and mgr.current.sample is not None)
    assert mgr.live_values()["speed_kmh"] == 10.5 and mgr.robot_state() == "RUNNING"
    await mgr.stop()
    await mgr.wait_idle(15)
    last = mgr.last_job
    assert last.exit_code == 130 and rec.sessions["sess1"]["status"] == "stopped" and last.stop_requested
    assert last.final_status == "stopped"
    assert mgr.state == jm.STATE_STANDBY and mgr.snapshot()["emergency_cause"] is None  # 非常停止の状態に入らない
    assert last.emergency is False and last.emergency_stage == "start"  # 停止の流れの段階の表示は残る
    assert mgr.initialized is True  # ランナーは残り、待機している
    assert runner_commands(tmp_path)[-1] == {"cmd": "stop"} and runner_starts(tmp_path) == 1
    assert any("stop コマンド" in m[4] for m in rec.messages["sess1"])
    with pytest.raises(jm.JobConflict):
        await mgr.stop()
    await mgr.start_mode_drive("v1", "m", "WLTP")  # 続けて始められる（常駐ランナーのまま）
    await mgr.wait_idle(30)
    assert runner_starts(tmp_path) == 1


async def test_emergency_sends_sigint_closes_runner_and_blocks_until_reset(tmp_path: Path) -> None:
    rec = FakeRecorder()
    probe = {"pressed": True}
    mgr = await make_ready(tmp_path, "h", rec, emergency_probe=lambda: probe["pressed"])
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await wait_until(lambda: mgr.current is not None and mgr.current.sample is not None)
    assert mgr.snapshot()["emergency_cause"] is None
    await mgr.emergency("switch")
    assert mgr.snapshot()["emergency_cause"] == "switch" and mgr.initialized is False
    await mgr.emergency("ac_loss")  # 二重に来ても最初のきっかけを残す
    assert mgr.snapshot()["emergency_cause"] == "switch"
    assert mgr.state == jm.STATE_EMERGENCY and mgr.robot_state() == "EMERGENCY"
    await mgr.wait_idle(15)
    assert rec.sessions["sess1"]["status"] == "emergency" and mgr.last_job.exit_code == 130
    await wait_until(lambda: mgr._runner is None)  # noqa: SLF001 - ランナーは終了した
    assert mgr.state == jm.STATE_EMERGENCY and mgr.initialized is False
    with pytest.raises(jm.JobConflict):
        await mgr.start_init_only("v1")
    with pytest.raises(jm.EmergencyStillActive):  # 物理スイッチが押されたまま
        await mgr.reset_emergency()
    probe["pressed"] = False
    await mgr.reset_emergency()
    assert mgr.state == jm.STATE_STANDBY and mgr.robot_state() == "READY" and mgr.snapshot()["emergency_cause"] is None
    await mgr.reset_emergency()  # 解除済みでの再呼び出しは何もしない
    await mgr.start_init_only("v1")  # 解除 → 初期化: ランナーを起動し直す
    await mgr.wait_idle(30)
    assert mgr.initialized and runner_starts(tmp_path) == 2


async def test_emergency_while_idle_sends_sigint_to_resident_runner(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path)
    await mgr.emergency()
    assert mgr.state == jm.STATE_EMERGENCY and mgr.initialized is False
    await wait_until(lambda: mgr._runner is None)  # noqa: SLF001
    await mgr.reset_emergency()
    assert mgr.state == jm.STATE_STANDBY


async def test_runner_auto_emergency_sets_emergency_without_sigint(tmp_path: Path) -> None:
    """ランナーが自分で自動の非常停止に入ったら EMERGENCY にする。SIGINT は送らない（流れが走っている）。"""
    rec = FakeRecorder()
    mgr = await make_ready(tmp_path, "e", rec)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await wait_until(lambda: mgr.state == jm.STATE_EMERGENCY)
    assert mgr.robot_state() == "EMERGENCY" and mgr.initialized is False
    assert mgr.snapshot()["emergency_cause"] == "auto" and mgr.snapshot()["emergency_cause_detail"]
    assert mgr.current is not None and mgr.current.emergency and mgr.current.stop_requested
    await mgr.emergency()  # 流れの最中に画面の非常停止が押されても SIGINT は送らない
    await mgr.stop()  # 停止ボタンも同様（stop コマンドも送らない）
    await mgr.wait_idle(15)
    assert mgr.last_job.exit_code == 5  # SIGINT を受けていれば 130 になる
    assert rec.sessions["sess1"]["status"] == "emergency"
    assert any("自動の非常停止" in m[4] for m in rec.messages["sess1"])
    assert "stop" not in [c["cmd"] for c in runner_commands(tmp_path)]
    assert mgr.state == jm.STATE_EMERGENCY  # 解除まで残る
    await mgr.reset_emergency()  # ランナーは待機のまま生きている（終了処理中ではない）
    assert mgr.state == jm.STATE_STANDBY and mgr.initialized is False


async def test_needs_init_result_clears_initialized(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path, "n")
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await mgr.wait_idle(30)
    assert mgr.initialized is False and mgr.state == jm.STATE_STANDBY
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_learning("v1", "hold")
    await mgr.start_init_only("v1")  # 生きているランナーへ init
    await mgr.wait_idle(30)
    assert mgr.initialized is True and runner_starts(tmp_path) == 1


async def test_watch_error_closes_runner_and_enters_emergency_auto(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("FAKE_WATCH_AFTER", "0.5")
    mgr = await make_ready(tmp_path)
    await wait_until(lambda: mgr.state == jm.STATE_EMERGENCY, 15)
    assert mgr.snapshot()["emergency_cause"] == "auto" and "アラーム" in (mgr.snapshot()["emergency_cause_detail"] or "")
    await wait_until(lambda: mgr._runner is None)  # noqa: SLF001
    assert mgr.initialized is False
    await mgr.reset_emergency()
    assert mgr.state == jm.STATE_STANDBY


async def test_emergency_without_job_blocks_start(tmp_path: Path) -> None:
    mgr = make_manager(tmp_path)
    await mgr.emergency()
    assert mgr.state == jm.STATE_EMERGENCY
    with pytest.raises(jm.JobConflict):
        await mgr.start_init_only("v1")
    await mgr.reset_emergency()
    await mgr.start_init_only("v1")
    await mgr.wait_idle(30)
    assert mgr.initialized
    await mgr.shutdown()


async def test_sigkill_when_runner_ignores_sigint(tmp_path: Path) -> None:
    rec = FakeRecorder()
    mgr = await make_ready(tmp_path, "k", rec, kill_timeout_s=1.0)
    await mgr.start_learning("v1", "full")
    await asyncio.sleep(0.5)
    t0 = time.monotonic()
    await mgr.emergency()
    await mgr.wait_idle(20)
    assert 0.9 < time.monotonic() - t0 < 10
    assert mgr.last_job.exit_code == -9
    assert rec.sessions["sess1"]["status"] == "emergency"
    assert any("SIGKILL" in m[4] for m in rec.messages["sess1"])


async def test_stop_that_never_finishes_is_killed(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path, "k", kill_timeout_s=1.0)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await asyncio.sleep(0.3)
    await mgr.stop()
    await mgr.wait_idle(20)
    assert mgr.last_job.exit_code == -9 and mgr.initialized is False


async def test_interactive_commands_and_manual_session(tmp_path: Path) -> None:
    rec = FakeRecorder()
    saved: list[tuple[str, str]] = []
    mgr = await make_ready(tmp_path, "", rec, on_min_saved=lambda v, a: saved.append((v, a)))
    with pytest.raises(jm.JobConflict):
        await mgr.send_command({"cmd": "status"})  # 対話ジョブが無い
    status = await mgr.ensure_interactive("v1", jm.PURPOSE_MANUAL)
    assert status.interactive_ready and mgr.robot_state() == "MANUAL"
    assert (await mgr.ensure_interactive("v1", jm.PURPOSE_MANUAL)).job_id == status.job_id  # 使い回し
    with pytest.raises(jm.JobConflict):
        await mgr.ensure_interactive("v1", jm.PURPOSE_CALIB)  # 目的が違う
    with pytest.raises(jm.JobConflict):
        await mgr.ensure_interactive("v2", jm.PURPOSE_MANUAL)  # 車が違う
    reply = await mgr.send_command({"cmd": "jog", "axis": "accel", "amount_pct": 2.0})
    assert reply.ack["ok"] and reply.status is not None and reply.status["accel_pulse"] == 665
    assert mgr.live_values()["accel_pct"] == 7.0
    bad = await mgr.send_command({"cmd": "bad"})
    assert bad.ack["ok"] is False and bad.status is None
    await mgr.send_command({"cmd": "save_min", "axis": "accel"})
    assert saved == [("v1", "accel")]
    await mgr.stop()  # quit_interactive（ランナーは終わらない）
    await mgr.wait_idle(15)
    assert mgr.last_job.final_status == "completed" and rec.sessions["sess1"]["run_type"] == "manual"
    assert rec.sessions["sess1"]["status"] == "completed" and runner_starts(tmp_path) == 1 and mgr.initialized
    assert runner_commands(tmp_path)[-1] == {"cmd": "quit_interactive"}


async def test_calibration_job_makes_no_session(tmp_path: Path) -> None:
    rec = FakeRecorder()
    mgr = await make_ready(tmp_path, "", rec)
    st = await mgr.ensure_interactive("v1", jm.PURPOSE_CALIB)
    assert st.session_id is None and mgr.robot_state() == "CALIBRATING"
    await mgr.send_command({"cmd": "quit"})  # 対話の終わり（quit_interactive に読み替える）
    await mgr.wait_idle(15)
    assert rec.sessions == {} and mgr.last_job.final_status == "completed" and mgr.initialized


async def test_schedule_and_ign_off_without_init_are_409_and_start_nothing(tmp_path: Path) -> None:
    mgr = make_manager(tmp_path, "0")
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_schedule("v1", {"id": "i", "name": "S"})
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_ign_off("v1")
    steps = [jm.PlannedStep("schedule", "a", "A", 1, schedule={"id": "a", "name": "A"})]
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_sequence("v1", "sid", "SEQ", steps, 1)
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_prepare_schedule("v1", {"id": "i", "name": "S"})
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_prepare_sequence("v1", "sid", "SEQ", steps, 1)
    assert not mgr.is_busy and mgr.sequence is None and mgr.prepare is None and runner_starts(tmp_path) == 0


async def test_calibration_without_init_runs_init_first(tmp_path: Path) -> None:
    mgr = make_manager(tmp_path)
    st = await mgr.ensure_interactive("v1", jm.PURPOSE_CALIB)  # 手動・キャリブだけは内部で初期化してから始める
    assert st.interactive_ready and runner_starts(tmp_path) == 1 and mgr.initialized
    await mgr.stop()
    await mgr.wait_idle(15)
    await mgr.shutdown()


async def test_shutdown_sends_quit_and_waits_for_runner(tmp_path: Path) -> None:
    rec = FakeRecorder()
    mgr = await make_ready(tmp_path, "h", rec)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await asyncio.sleep(0.5)
    runner = mgr._runner  # noqa: SLF001
    assert runner is not None
    await mgr.shutdown()
    assert runner.exited.is_set() and not mgr.is_busy and runner_commands(tmp_path)[-1] == {"cmd": "quit"}
    assert rec.sessions["sess1"]["status"] == "error"


async def test_shutdown_idle_runner_quits(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path)
    runner = mgr._runner  # noqa: SLF001
    await mgr.shutdown()
    assert runner is not None and runner.exited.is_set() and runner.exit_code == 0
    assert runner_commands(tmp_path)[-1] == {"cmd": "quit"}


async def test_start_fails_when_session_cannot_be_created(tmp_path: Path) -> None:
    class Broken(FakeRecorder):
        async def start_session(self, **kw: Any) -> str | None:
            raise RuntimeError("DB 接続なし")

    mgr = await make_ready(tmp_path, "0", Broken())
    with pytest.raises(jm.JobUnavailable):
        await mgr.start_mode_drive("v1", "m", "WLTP")
    assert not mgr.is_busy and mgr.state == jm.STATE_STANDBY and mgr.initialized
    await mgr.shutdown()


# ── 初期化の関門（D-15c） ────────────────────────────────────────


async def test_gate_blocks_learning_and_mode_drive_until_init(tmp_path: Path) -> None:
    mgr = make_manager(tmp_path, "0")
    assert mgr.initialized is False and mgr.snapshot()["initialized"] is False
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_mode_drive("v1", "m", "WLTP")
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_learning("v1", "full")
    assert not mgr.is_busy and mgr.state == jm.STATE_STANDBY and runner_starts(tmp_path) == 0  # 何も起動していない
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_ign_off("v1")  # IGN-OFF・スケジュールも未初期化なら 409（内部で初期化しない）


async def test_normal_completion_keeps_gate_open(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path, "0,0")
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await mgr.wait_idle(30)
    assert mgr.initialized is True
    await mgr.start_learning("v1", "hold")  # 続けて始められる
    await mgr.wait_idle(30)
    assert mgr.initialized is True
    await mgr.shutdown()


async def test_emergency_closes_gate(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path)
    assert mgr.initialized is True
    await mgr.emergency()
    assert mgr.initialized is False
    await wait_until(lambda: mgr._runner is None)  # noqa: SLF001
    await mgr.reset_emergency()
    with pytest.raises(jm.JobConflict, match="初期化ページ"):
        await mgr.start_mode_drive("v1", "m", "WLTP")


def test_apply_event_emergency_stage_marks_ign_off_and_brake_kept() -> None:
    """非常停止の流れ（phase=emergency）: ign_off でスタートを押し終えた印・brake_kept で警告の印。"""
    st = _status()
    jm.apply_event(st, {"type": "state", "phase": "emergency", "stage": "decel", "detail": "0.2G"})
    assert st.phase == "emergency" and st.emergency_stage == "decel" and not st.ign_off_done
    jm.apply_event(st, {"type": "state", "phase": "emergency", "stage": "ign_off", "detail": ""})
    assert st.ign_off_done and st.to_dict()["emergency_stage"] == "ign_off"
    jm.apply_event(st, {"type": "state", "phase": "emergency", "stage": "brake_kept", "detail": "x"})
    assert st.brake_kept and st.to_dict()["brake_kept"] is True
    assert jm.KILL_TIMEOUT_S == 120.0


# ── 段の間（DB への取り込みを待たずに次のジョブを始める） ───────────────────


class SlowRecorder(FakeRecorder):
    """CSV の取り込みに時間がかかる記録先（取り込みの開始・終了の順を残す）。"""

    def __init__(self) -> None:
        super().__init__()
        self.order: list[str] = []

    async def import_csv(self, session_id: str, path: Path) -> int:
        self.order.append(f"start {session_id}")
        await asyncio.sleep(1.0)
        self.order.append(f"end {session_id}")
        return await super().import_csv(session_id, path)


async def test_next_job_starts_without_waiting_for_csv_import_and_records_do_not_mix(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    csv_a = write_csv(tmp_path / "x1.csv")
    csv_b = write_csv(tmp_path / "x2.csv")
    rec = SlowRecorder()
    monkeypatch.setenv("FAKE_CSV", str(tmp_path / "x{n}.csv"))
    mgr = await make_ready(tmp_path, "c,c", rec)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    for _ in range(200):  # 1 本目が終わった（最終状態が決まった）のに、取り込みはまだ
        if mgr.last_job is not None and mgr.last_job.session_id == "sess1" and mgr.last_job.ended_at:
            break
        await asyncio.sleep(0.01)
    assert mgr.last_job.final_status == "completed" and mgr.state == jm.STATE_STANDBY and not mgr.is_busy
    assert rec.sessions["sess1"]["status"] == "running" and rec.imported == []  # 取り込み中
    t0 = time.monotonic()
    await mgr.start_mode_drive("v1", "m", "WLTP")  # 取り込みを待たずに次を始められる
    assert time.monotonic() - t0 < 0.5
    await mgr.wait_idle(30)
    # 取り込みは終わった順に 1 本ずつ（混ざらない）。各セッションの終わりの記録も揃う
    assert rec.order == ["start sess1", "end sess1", "start sess2", "end sess2"]
    assert rec.imported == [("sess1", "x1.csv"), ("sess2", "x2.csv")]
    assert [v["status"] for v in rec.sessions.values()] == ["completed", "completed"]
    assert not csv_a.exists() and not csv_b.exists()
    assert all(any("ジョブ終了" in m[4] for m in rec.messages[sid]) for sid in rec.sessions)
    await mgr.shutdown()


def test_idle_lines_are_appended_to_idle_log(tmp_path: Path) -> None:
    """ジョブの外のランナーの行（非常停止の後・アプリ終了の終了処理）はファイルに残る。"""
    log = tmp_path / "logs" / "runner_idle.log"
    m = make_manager(tmp_path, idle_log_path=log)
    m._say_idle("原点復帰しました")
    m._say_idle("サーボ OFF", "WARNING")
    lines = log.read_text(encoding="utf-8").splitlines()
    assert len(lines) == 2
    assert lines[0].endswith("INFO    原点復帰しました")
    assert lines[1].endswith("WARNING サーボ OFF")


def test_idle_log_disabled_by_default(tmp_path: Path) -> None:
    m = make_manager(tmp_path)
    m._say_idle("原点復帰しました")
    assert not (tmp_path / "logs").exists()
