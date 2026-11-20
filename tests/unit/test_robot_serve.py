# ruff: noqa: E501
"""常駐ランナー（src/robot/serve.py・patches のジョブの終わりの切り替え・events の job_id）のテスト。

コマンドの解釈・A-D1 の判定・見張りの判定は純粋関数で、ジョブの終わりの切り替えは偽の HW で、
通しはスタブ HW（in-process。標準入力は使わず runner.queue に行を入れる）で確かめる。
"""

from __future__ import annotations

import asyncio
import io
import json
import signal
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from src.robot import config_store as cs
from src.robot import emergency_stop, events, patches, serve
from src.robot.core import main as core_main
from src.robot.core.config import ConfigError
from src.robot.interactive import CommandError
from src.robot.interrupt import StopController
from src.robot.serve import (
    AxisReading,
    CanReading,
    IdleWatcher,
    decide_after_job,
    parse_serve_command,
)

# ── コマンドの解釈 ──────────────────────────────────────────────


def test_parse_mode_defaults_and_fields() -> None:
    c = parse_serve_command(
        '{"cmd":"mode","job_id":"j1","mode":"03_WLTP_Low","plan_in":"/x/p.pkl","limit_s":30,"no_pre_check":true,"home_at_end":false}'
    )
    assert (c.cmd, c.job_id, c.mode, c.limit_s) == ("mode", "j1", "03_WLTP_Low", 30.0)
    assert c.plan_in == Path("/x/p.pkl") and c.no_pre_check is True and c.home_at_end is False
    d = parse_serve_command('{"cmd":"mode","job_id":"j2","mode":"09_US06","plan_in":null,"limit_s":null}')
    assert d.plan_in is None and d.limit_s is None and d.no_pre_check is False and d.home_at_end is True


def test_parse_schedule_steps_interactive_init_and_controls() -> None:
    s = parse_serve_command('{"cmd":"schedule","job_id":"a","schedule":{"name":"x","pedal_points":[]},"home_at_end":false}')
    assert s.schedule == {"name": "x", "pedal_points": []} and s.home_at_end is False
    t = parse_serve_command('{"cmd":"steps","job_id":"b","steps":"2,25"}')
    assert t.steps == "2,25" and t.home_at_end is True
    i = parse_serve_command('{"cmd":"interactive","job_id":"c","purpose":"calib"}')
    assert i.purpose == "calib"
    assert parse_serve_command('{"cmd":"interactive","job_id":"c"}').purpose == "manual"
    assert parse_serve_command('{"cmd":"init","job_id":"d"}').cmd == "init"
    # 制御コマンドは job_id なしでよい
    for name in ("stop", "quit", "quit_interactive"):
        assert parse_serve_command(json.dumps({"cmd": name})).cmd == name
    # 対話の操作は元の行をそのまま持つ（対話セッションが解釈する）
    j = parse_serve_command('{"cmd":"jog","axis":"accel","amount_pct":2,"id":7}')
    assert j.cmd == "jog" and j.req_id == 7 and '"amount_pct"' in j.raw


@pytest.mark.parametrize(
    "line",
    [
        "",
        "not json",
        "[1]",
        '{"cmd":"nope"}',
        '{"cmd":"mode","mode":"x"}',  # job_id なし
        '{"cmd":"mode","job_id":"","mode":"x"}',
        '{"cmd":"mode","job_id":1,"mode":"x"}',
        '{"cmd":"mode","job_id":"a"}',  # mode なし
        '{"cmd":"mode","job_id":"a","mode":"x","limit_s":0}',
        '{"cmd":"mode","job_id":"a","mode":"x","limit_s":true}',
        '{"cmd":"mode","job_id":"a","mode":"x","home_at_end":"yes"}',
        '{"cmd":"mode","job_id":"a","mode":"x","plan_in":3}',
        '{"cmd":"schedule","job_id":"a"}',
        '{"cmd":"steps","job_id":"a","steps":"1,24"}',  # 手順 1 は init
        '{"cmd":"steps","job_id":"a","steps":"3"}',  # 手順 3 は動かさない
        '{"cmd":"steps","job_id":"a","steps":"2,24"}',  # 学習と 24 は同じ実行にできない
        '{"cmd":"steps","job_id":"a"}',
        '{"cmd":"interactive","job_id":"a","purpose":"x"}',
    ],
)
def test_parse_errors(line: str) -> None:
    with pytest.raises(CommandError):
        parse_serve_command(line)


# ── A-D1: ジョブの終わりの判定 ──────────────────────────────────


def _diag(**kw: Any) -> emergency_stop.Diagnosis:
    return emergency_stop.Diagnosis(**kw)


@pytest.mark.parametrize(
    ("code", "kwargs", "close", "needs_init"),
    [
        (0, {}, False, False),
        (2, {"error": "走行モード 'x' が見つかりません"}, False, False),
        (3, {}, False, False),
        (6, {}, False, False),
        (7, {"error": "走行前チェックエラー"}, False, False),
        (8, {}, False, False),
        # 4・5: ハード語・接続の閉じ・診断
        (5, {"error": "ブレーキ軸にアラームが発生しました（t=3.0s）"}, True, False),
        (5, {"error": "アクセル軸が過電流です（5000 mA > 4000 mA、t=1.0s）"}, True, False),
        (5, {"error": "ブレーキ軸の電流が 1s 0mA のままです"}, True, False),
        (5, {"error": "CAN 車速を読めません"}, True, False),
        (5, {"error": "DriveError: アクチュエータ通信に失敗しました"}, True, False),
        (5, {"error": "TimeoutError: "}, True, False),
        (5, {"error": "ボタンサーボ接続: OSError"}, True, False),
        (5, {"error": "車速が上がりません（クリープ不足）"}, False, False),  # ハードに関係しない
        (5, {"error": "IGN-OFF の前に停車していません（車速 5.0 km/h）"}, False, False),
        (5, {"error": "", "hw_connected": False}, True, False),
        (4, {"error": "InitializationError: サーボ通信"}, True, False),
        (4, {"error": ""}, True, False),
        # 診断が優先
        (5, {"error": "x", "diag": _diag(brake_faulty=True)}, True, False),
        (5, {"error": "x", "diag": _diag(accel_faulty=True)}, True, False),
        (5, {"error": "x", "diag": _diag(can_ok=False)}, True, False),
        (5, {"error": "x", "diag": _diag()}, False, False),
        # 停止コマンド・ブレーキ保持
        (130, {"stopped": True}, False, False),
        (5, {"stopped": True, "error": "x"}, False, False),
        (5, {"kept": True, "hw_connected": False}, False, True),
        (5, {"kept": True, "diag": _diag(brake_faulty=True)}, True, False),
        # シグナルの中断は閉じる・未知は安全側
        (130, {}, True, False),
        (143, {}, True, False),
        (1, {}, True, False),
    ],
)
def test_decide_after_job(code: int, kwargs: dict[str, Any], close: bool, needs_init: bool) -> None:
    d = decide_after_job(code, **kwargs)
    assert (d.close, d.needs_init) == (close, needs_init), d.reason


def test_decide_after_job_signal_reason_is_not_unknown() -> None:
    assert decide_after_job(130).reason == "非常停止（SIGINT）で中断しました"
    assert "未知" not in decide_after_job(143).reason
    assert decide_after_job(1).reason == "未知の終了コード 1"


# ── A-D3: 待機中の見張りの判定 ──────────────────────────────────

OK_AX = AxisReading(True, "", False, 120.0, 0)
OK_CAN = CanReading(True, "", 0.0)
BAD_AX = AxisReading(False, "TimeoutError: ")
BAD_CAN = CanReading(False, "TimeoutError: ")


def _errors(findings: list[Any]) -> list[str]:
    return [f.detail for f in findings if f.level == "error"]


def test_watch_normal_has_no_findings() -> None:
    w = IdleWatcher(overcurrent_ma=4000.0)
    for _ in range(5):
        assert w.evaluate(OK_AX, OK_AX, OK_CAN) == []


def test_watch_alarm_and_overcurrent_are_immediate() -> None:
    w = IdleWatcher(overcurrent_ma=4000.0)
    e = _errors(w.evaluate(AxisReading(True, "", True, 10.0, 0), OK_AX, OK_CAN))
    assert len(e) == 1 and "アクセル軸にアラーム" in e[0]
    e = _errors(w.evaluate(OK_AX, AxisReading(True, "", False, 4500.0, 0), OK_CAN))
    assert len(e) == 1 and "ブレーキ軸が過電流" in e[0]


def test_watch_comm_failure_needs_three_in_a_row() -> None:
    w = IdleWatcher(overcurrent_ma=4000.0)
    for n in (1, 2):
        f = w.evaluate(BAD_AX, OK_AX, OK_CAN)
        assert _errors(f) == [] and f[0].level == "warn" and f"{n}/3" in f[0].detail
    assert _errors(w.evaluate(BAD_AX, OK_AX, OK_CAN))  # 3 回目
    # 途中で読めれば数え直す
    w2 = IdleWatcher(overcurrent_ma=4000.0)
    w2.evaluate(OK_AX, BAD_AX, OK_CAN)
    w2.evaluate(OK_AX, BAD_AX, OK_CAN)
    w2.evaluate(OK_AX, OK_AX, OK_CAN)
    assert _errors(w2.evaluate(OK_AX, BAD_AX, OK_CAN)) == []


def test_watch_can_failure_needs_three_in_a_row() -> None:
    w = IdleWatcher(overcurrent_ma=4000.0)
    assert _errors(w.evaluate(OK_AX, OK_AX, BAD_CAN)) == []
    assert _errors(w.evaluate(OK_AX, OK_AX, BAD_CAN)) == []
    e = _errors(w.evaluate(OK_AX, OK_AX, BAD_CAN))
    assert len(e) == 1 and "CAN" in e[0]
    w.evaluate(OK_AX, OK_AX, OK_CAN)
    assert _errors(w.evaluate(OK_AX, OK_AX, BAD_CAN)) == []


def test_watch_zero_current_only_after_current_seen() -> None:
    w = IdleWatcher(overcurrent_ma=4000.0)
    zero = AxisReading(True, "", False, 0.0, 500)
    for _ in range(5):  # 一度も電流 > 0 を返していない軸（スタブ）は異常にしない
        assert w.evaluate(zero, OK_AX, OK_CAN) == []
    w.evaluate(AxisReading(True, "", False, 80.0, 500), OK_AX, OK_CAN)
    assert _errors(w.evaluate(zero, OK_AX, OK_CAN)) == []
    assert _errors(w.evaluate(zero, OK_AX, OK_CAN)) == []
    assert "電流が 0 mA" in _errors(w.evaluate(zero, OK_AX, OK_CAN))[0]
    # 位置 0（原点）の 0 mA は異常でない
    w3 = IdleWatcher(overcurrent_ma=4000.0)
    w3.evaluate(AxisReading(True, "", False, 80.0, 500), OK_AX, OK_CAN)
    for _ in range(5):
        assert w3.evaluate(AxisReading(True, "", False, 0.0, 0), OK_AX, OK_CAN) == []


async def test_read_idle_with_stub_hardware_and_failures(tmp_path: Path) -> None:
    from src.robot.core import hardware as core_hardware  # noqa: PLC0415

    cfg = cs.load_config(_common_yaml(tmp_path))
    hw = core_hardware.build_hardware(cfg, core_hardware.HW_STUB)
    await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    accel, brake, can = await serve.read_idle(hw)
    assert accel.ok and brake.ok and can.ok and not accel.alarm
    hw.brake.alarm = True
    _, brake, _ = await serve.read_idle(hw)
    assert brake.ok and brake.alarm
    await hw.brake.close()  # 切れた軸は読めない（ok=False・error に例外）
    _, brake, _ = await serve.read_idle(hw)
    assert not brake.ok and "RuntimeError" in brake.error


# ── events: job_id を自動で付ける ───────────────────────────────


def test_events_attach_job_id_automatically() -> None:
    buf = io.StringIO()
    events.set_sink(events.EventSink(stream=buf))
    try:
        events.emit("state", phase="idle")
        events.set_job_id("j9")
        events.emit("state", phase="step", step=24)
        events.emit("ack", ok=False, job_id="other")  # 明示が優先
        events.set_job_id(None)
        events.emit("watch", level="warn", detail="x")
    finally:
        events.set_job_id(None)
        events.set_sink(None)
    got = [events.decode_event_line(ln) for ln in buf.getvalue().splitlines()]
    assert got[0] is not None and "job_id" not in got[0]
    assert got[1] is not None and got[1]["job_id"] == "j9"
    assert got[2] is not None and got[2]["job_id"] == "other"
    assert got[3] is not None and got[3]["type"] == "watch" and "job_id" not in got[3]


# ── ジョブの終わりの切り替え（patches 差し替え 4） ─────────────────


class _Axis:
    def __init__(self, name: str, log: list[str]) -> None:
        self.name, self.log = name, log

    async def home_return(self) -> None:
        self.log.append(f"{self.name}:home")

    async def servo_off(self) -> None:
        self.log.append(f"{self.name}:servo_off")

    async def close(self) -> None:
        self.log.append(f"{self.name}:close")


def _fake_hw(log: list[str]) -> Any:
    async def ups_stop() -> None:
        log.append("ups:stop")

    return SimpleNamespace(
        accel=_Axis("accel", log),
        brake=_Axis("brake", log),
        can=SimpleNamespace(close=lambda: _async_log(log, "can:close")),
        ups=SimpleNamespace(stop_polling=ups_stop),
        buttons=None,
        buttons_connected=False,
        connected=True,
        initialized=True,
    )


async def _async_log(log: list[str], text: str) -> None:
    log.append(text)


@pytest.fixture
def job_end_clean():
    patches.install_all(StopController())
    emergency_stop.reset()
    patches.set_job_end("full")
    yield
    patches.set_job_end("full")
    emergency_stop.reset()


async def test_job_end_modes_switch_core_shutdown(job_end_clean: None, capsys: pytest.CaptureFixture[str]) -> None:
    buf = io.StringIO()
    events.set_sink(events.EventSink(stream=buf))
    try:
        # keep: 何もしない（停車保持のまま）。接続・初期化のまま
        log: list[str] = []
        hw = _fake_hw(log)
        patches.set_job_end("keep")
        await core_main.shutdown(hw)
        assert log == [] and hw.connected and hw.initialized
        # home: 原点復帰だけ（サーボ OFF・切断なし）
        patches.set_job_end("home")
        await core_main.shutdown(hw)
        assert sorted(log) == ["accel:home", "brake:home"] and hw.connected and hw.initialized
        # full（既定）: core のまま 原点復帰 → サーボ OFF → 切断
        log.clear()
        patches.set_job_end("full")
        await core_main.shutdown(hw)
        assert [x for x in log if x.endswith(":home")] and any(x.endswith(":servo_off") for x in log)
        assert "can:close" in log and not hw.connected and not hw.initialized
        assert log.index("accel:servo_off") > log.index("accel:home")
    finally:
        events.set_sink(None)
    phases = [(e["phase"], e.get("mode")) for ln in buf.getvalue().splitlines() if (e := events.decode_event_line(ln))]
    assert phases == [("job_end", "keep"), ("job_end", "home"), ("shutdown", None)]
    capsys.readouterr()


async def test_job_end_keeps_brake_hold_handling(job_end_clean: None, capsys: pytest.CaptureFixture[str]) -> None:
    """ブレーキを保ったまま終えたとき（brake_kept）は、モードに関係なく今までどおり（サーボ ON のまま切断）。"""
    log: list[str] = []
    hw = _fake_hw(log)
    emergency_stop.brake_kept = True
    patches.set_job_end("home")
    await core_main.shutdown(hw)
    assert "accel:home" not in log and "accel:servo_off" not in log and "can:close" in log
    assert not hw.connected
    capsys.readouterr()


def test_set_job_end_rejects_unknown() -> None:
    with pytest.raises(ValueError):
        patches.set_job_end("none")
    assert patches.current_job_end() in patches.JOB_END_MODES


# ── スタブでの通し（in-process） ────────────────────────────────


def _common_yaml(tmp_path: Path) -> Path:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    return common


def _make_args(tmp_path: Path) -> Any:
    return SimpleNamespace(
        vehicle="car_a", hw="stub", vehicles_dir=tmp_path / "vehicles", common=tmp_path / "c.yaml",
    )


def _parse_events(buf: io.StringIO) -> list[dict[str, Any]]:
    return [e for ln in buf.getvalue().splitlines() if (e := events.decode_event_line(ln)) is not None]


@pytest.fixture
async def stub_runner(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]):
    """標準入力を読まず、設定は共通 yaml だけ（検証は省く）で、スタブ HW のランナーを作る。"""
    cfg_common = _common_yaml(tmp_path)

    def load_config(self: serve.ServeRunner):  # noqa: ANN202
        cfg = cs.load_config(cfg_common)
        cfg.vehicle.min_accel_opening_pct, cfg.vehicle.max_accel_opening_pct = 6.5, 60.0
        cfg.vehicle.min_brake_opening_pct, cfg.vehicle.max_brake_opening_pct = 2.9, 40.0
        return cfg, Path("merged.yaml")

    monkeypatch.setattr(serve.ServeRunner, "_load_config", load_config)
    monkeypatch.setattr(serve.ServeRunner, "_start_reader", lambda self, loop: None)
    monkeypatch.setattr(core_main, "validate_config_once", lambda ctx: None)
    monkeypatch.setattr("src.robot.interactive.SETTLE_S", 0.0)
    monkeypatch.setattr(serve, "WATCH_INTERVAL_S", 0.05)
    buf = io.StringIO()
    events.set_sink(events.EventSink(stream=buf))
    runner = serve.ServeRunner(_make_args(tmp_path))
    yield runner, buf
    events.set_sink(None)
    events.set_job_id(None)
    patches.set_job_end("full")
    emergency_stop.reset()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.remove_signal_handler(sig)  # 無くてもよい（False を返すだけ）
    capsys.readouterr()


def _send(runner: serve.ServeRunner, obj: dict[str, Any]) -> None:
    runner.queue.put_nowait(("line", json.dumps(obj)))


async def _wait_for(buf: io.StringIO, pred: Any, timeout: float = 20.0) -> None:
    async def poll() -> None:
        while not any(pred(e) for e in _parse_events(buf)):
            await asyncio.sleep(0.02)

    await asyncio.wait_for(poll(), timeout)


def _is(type_: str, **kw: Any):  # noqa: ANN202
    return lambda e: e["type"] == type_ and all(e.get(k) == v for k, v in kw.items())


async def test_serve_init_interactive_reuses_connection_then_quit(stub_runner: Any) -> None:
    runner, buf = stub_runner
    task = asyncio.create_task(runner.run())
    await _wait_for(buf, _is("state", phase="idle"))
    hw = runner.hw
    connects = []
    orig_connect = hw.accel.connect

    async def counting_connect() -> None:
        connects.append(1)
        await orig_connect()

    hw.accel.connect = counting_connect
    _send(runner, {"cmd": "jog", "axis": "accel", "amount_pct": 2})  # 対話の外 → NG
    _send(runner, {"cmd": "interactive", "job_id": "i1", "purpose": "manual"})
    _send(runner, {"cmd": "mode", "job_id": "m1", "mode": "x"})  # 実行中 → NG
    await _wait_for(buf, _is("state", phase="interactive", job_id="i1"))
    _send(runner, {"cmd": "jog", "axis": "accel", "amount_pct": 10, "id": 5})
    await _wait_for(buf, _is("status", id=5))
    _send(runner, {"cmd": "quit_interactive"})
    await _wait_for(buf, _is("result", job_id="i1"))
    await _wait_for(buf, lambda e: e["type"] == "state" and e.get("phase") == "idle" and e["ts"] > 0 and runner.job_task is None)
    assert hw.connected and hw.initialized and not connects  # 接続し直していない（サーボ ON のまま）
    assert hw.accel.servo_on_state
    # 対話の終わりは原点復帰だけ（ジョブの終わり home）
    assert hw.accel.position == 0
    _send(runner, {"cmd": "quit", "job_id": "q1"})
    code = await asyncio.wait_for(task, 20)
    assert code == 0 and not hw.connected and not hw.accel.servo_on_state
    ev = _parse_events(buf)
    results = [e for e in ev if e["type"] == "result"]
    assert [r["job_id"] for r in results] == ["init", "i1", "q1"]
    assert results[0]["exit_code"] == 0 and results[1]["exit_code"] == 0
    acks = [e for e in ev if e["type"] == "ack"]
    assert any(a["cmd"] == "jog" and a["ok"] is False and "対話ジョブの間だけ" in a["message"] for a in acks)
    assert any(a["cmd"] == "mode" and a["ok"] is False and a["job_id"] == "m1" and "実行中" in a["message"] for a in acks)
    assert any(a["cmd"] == "jog" and a["ok"] is True and a["job_id"] == "i1" for a in acks)
    assert [e["phase"] for e in ev if e["type"] == "state" and e.get("job_id") == "i1" and e["phase"] == "job_end"] == ["job_end"]
    assert [e["mode"] for e in ev if e["type"] == "state" and e.get("phase") == "job_end"] == ["home"]
    assert sum(1 for e in ev if e["type"] == "state" and e.get("phase") == "shutdown") == 1  # 本物の終了処理は quit の 1 回だけ


async def test_serve_config_error_returns_to_idle_and_stop_without_job(
    stub_runner: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner, buf = stub_runner

    async def bad_select(cfg: Any, text: str) -> str:
        raise ConfigError(f"走行モード {text!r} が見つかりません")

    monkeypatch.setattr(serve.run, "select_mode", bad_select)
    task = asyncio.create_task(runner.run())
    await _wait_for(buf, _is("state", phase="idle"))
    _send(runner, {"cmd": "stop"})  # ジョブなし → ack だけ
    _send(runner, {"cmd": "mode", "job_id": "m1", "mode": "zzz"})
    await _wait_for(buf, _is("result", job_id="m1"))
    await asyncio.sleep(0.1)
    assert runner.hw.connected and runner.hw.initialized and runner.job_task is None  # 閉じない
    _send(runner, {"cmd": "quit"})
    assert await asyncio.wait_for(task, 20) == 0
    ev = _parse_events(buf)
    r = next(e for e in ev if e["type"] == "result" and e.get("job_id") == "m1")
    assert r["exit_code"] == 2 and "見つかりません" in r["error"] and r["runner_exit"] is False
    assert any(e["type"] == "ack" and e["cmd"] == "stop" and e["ok"] and "ありません" in e["message"] for e in ev)
    assert sum(1 for e in ev if e["type"] == "state" and e.get("phase") == "idle") >= 2  # 初期化後・m1 の後


async def test_serve_idle_watch_abnormal_runs_emergency_then_exits_5(
    stub_runner: Any, monkeypatch: pytest.MonkeyPatch
) -> None:
    runner, buf = stub_runner
    called: list[str] = []

    async def fake_flow(hw: Any, reason: str) -> None:
        called.append(reason)
        emergency_stop.brake_kept = True

    async def never_run(hw: Any, reason: str) -> None:
        raise AssertionError("見張りの異常で非常停止の流れ（ボタンを押す）は走らせない")

    monkeypatch.setattr(emergency_stop, "hold_and_keep", fake_flow)
    monkeypatch.setattr(emergency_stop, "run", never_run)
    task = asyncio.create_task(runner.run())
    await _wait_for(buf, _is("state", phase="idle"))
    await asyncio.sleep(0.2)  # 正常の間は何も起きない
    assert not task.done()
    hw = runner.hw
    hw.brake.alarm = True  # 偽の異常
    code = await asyncio.wait_for(task, 20)
    assert code == 5 and len(called) == 1 and "ブレーキ軸にアラーム" in called[0]
    assert not hw.connected  # 本物の shutdown 済み
    ev = _parse_events(buf)
    w = [e for e in ev if e["type"] == "watch"]
    assert w and w[0]["level"] == "error" and "アラーム" in w[0]["detail"]
    last = [e for e in ev if e["type"] == "result"][-1]
    assert last["exit_code"] == 5 and last["runner_exit"] is True


async def test_serve_idle_watch_comm_failure_needs_three(stub_runner: Any, monkeypatch: pytest.MonkeyPatch) -> None:
    runner, buf = stub_runner

    async def fake_flow(hw: Any, reason: str) -> None:
        return None

    monkeypatch.setattr(emergency_stop, "hold_and_keep", fake_flow)
    task = asyncio.create_task(runner.run())
    await _wait_for(buf, _is("state", phase="idle"))

    async def boom() -> int:
        raise TimeoutError("CAN 車速が更新されていません")

    runner.hw.can.read_speed = boom
    code = await asyncio.wait_for(task, 20)
    assert code == 5
    w = [e for e in _parse_events(buf) if e["type"] == "watch"]
    assert [e["level"] for e in w] == ["warn", "warn", "error"]


async def test_serve_signal_while_idle_does_real_shutdown(stub_runner: Any) -> None:
    runner, buf = stub_runner
    task = asyncio.create_task(runner.run())
    await _wait_for(buf, _is("state", phase="idle"))
    hw = runner.hw
    runner._on_signal("SIGINT")  # noqa: SLF001
    assert await asyncio.wait_for(task, 20) == 130
    assert not hw.connected and not hw.accel.servo_on_state
    assert _parse_events(buf)[-1]["exit_code"] == 130


async def test_serve_stop_during_interactive_returns_to_idle(stub_runner: Any) -> None:
    runner, buf = stub_runner
    task = asyncio.create_task(runner.run())
    await _wait_for(buf, _is("state", phase="idle"))
    _send(runner, {"cmd": "interactive", "job_id": "i1"})
    await _wait_for(buf, _is("state", phase="interactive", job_id="i1"))
    _send(runner, {"cmd": "stop"})
    await _wait_for(buf, _is("result", job_id="i1"))
    r = next(e for e in _parse_events(buf) if e["type"] == "result" and e.get("job_id") == "i1")
    assert r["exit_code"] == 130 and r["runner_exit"] is False and "stop" in r["meaning"]
    await asyncio.sleep(0.1)
    assert runner.hw.connected  # ランナーは終わらない
    _send(runner, {"cmd": "quit"})
    assert await asyncio.wait_for(task, 20) == 0


async def test_serve_init_command_reinitializes(stub_runner: Any) -> None:
    runner, buf = stub_runner
    task = asyncio.create_task(runner.run())
    await _wait_for(buf, _is("state", phase="idle"))
    first = runner.hw
    _send(runner, {"cmd": "init", "job_id": "again"})
    await _wait_for(buf, _is("result", job_id="again"))
    await asyncio.sleep(0.1)
    assert runner.hw is not first and runner.hw.connected and runner.hw.initialized
    assert not first.connected  # 古い接続は本物の終了処理で閉じた
    _send(runner, {"cmd": "quit"})
    assert await asyncio.wait_for(task, 20) == 0
