"""src/robot のランナー周りの純粋な部分（イベント形式・引数の解釈・対話コマンド・中断・差し替え）。

tests/research は import しない。ハードウェア（スタブ）を使うのは対話コマンドの実行だけ。
"""

from __future__ import annotations

import asyncio
import io
import json
import os
from pathlib import Path

import pytest
import yaml

from src.robot import config_store as cs
from src.robot import events, patches, run
from src.robot.core import drive_log as core_drive_log
from src.robot.core import hardware as core_hardware
from src.robot.core import main as core_main
from src.robot.core import pattern_drive as core_pattern_drive
from src.robot.core.axis_monitor import AxisMonitor
from src.robot.core.config import ConfigError
from src.robot.core.research_types import DriveLogData
from src.robot.interactive import (
    CommandError,
    InteractiveSession,
    Limits,
    jog_target,
    parse_command,
    plan_move,
)
from src.robot.interrupt import Interrupted, StopController
from src.robot.streaming_log import (
    KIND_DRIVE,
    KIND_LEARNING,
    StreamingSessionLog,
    csv_kind,
    sample_fields,
)

# ── イベント形式 ────────────────────────────────────────────────


def test_event_roundtrip_with_prefix() -> None:
    line = events.encode_event(events.build_event("state", phase="step", step=3, title="日本語"))
    assert line.startswith("@@EV ") and "\n" not in line
    data = events.decode_event_line(line + "\n")
    assert data is not None
    assert data["type"] == "state" and data["step"] == 3 and data["title"] == "日本語"
    assert isinstance(data["ts"], float)


def test_decode_ignores_non_events() -> None:
    assert events.decode_event_line("[   1.0s] 初期化 OK") is None
    assert events.decode_event_line("@@EV not json") is None
    assert events.decode_event_line('@@EV {"type":"unknown"}') is None
    assert events.decode_event_line('{"type":"result","exit_code":0}', prefix="") is not None


def test_build_event_rejects_unknown_type() -> None:
    with pytest.raises(ValueError):
        events.build_event("nope")


def test_sink_stream_and_fd() -> None:
    buf = io.StringIO()
    events.EventSink(stream=buf).emit("result", exit_code=0)
    assert buf.getvalue().startswith("@@EV ") and buf.getvalue().endswith("\n")

    r, w = os.pipe()
    events.EventSink(fd=w).emit("csv", path="/x.csv", kind="drive", rows=3)
    os.close(w)
    text = os.read(r, 4096).decode("utf-8")
    os.close(r)
    data = json.loads(text)  # fd 経由は前置きなし
    assert data["type"] == "csv" and data["rows"] == 3


def test_exit_meaning() -> None:
    assert events.exit_meaning(0) == "成功"
    assert "SIGINT" in events.exit_meaning(130)
    assert "不明" in events.exit_meaning(99)


# ── ストリーミングログ ─────────────────────────────────────────


def test_csv_kind() -> None:
    assert csv_kind({"PRE_DRIVE_CHECK", "MODE_DRIVE"}) == KIND_DRIVE
    assert csv_kind({"PRE_DRIVE_CHECK", "PEDAL_SEARCH", "PATTERN_DRIVE"}) == KIND_LEARNING
    assert csv_kind(set()) == KIND_LEARNING


def test_sample_fields() -> None:
    sample = core_drive_log.DriveSample(
        elapsed_s=1.23456,
        timestamp=core_drive_log.datetime.now(tz=core_drive_log.UTC),
        data=DriveLogData(
            ref_speed_kmh=10.0, actual_speed_kmh=9.5, accel_opening=12.0, brake_opening=3.0,
            accel_pos=1140, brake_pos=285, accel_current=0.0, brake_current=0.0,
        ),
        section="MODE_DRIVE",
        phase="RUN",
        monitor_accel=AxisMonitor(
            position_pulse=950, current_ma=0.0, alarm_code=0, servo_on=True, moving=False,
            pos_done=True,
        ),
    )
    f = sample_fields(sample)
    assert f["t_s"] == 1.235 and f["ref_kmh"] == 10.0 and f["speed_kmh"] == 9.5
    assert f["accel_actual_pct"] == 10.0 and f["brake_actual_pct"] is None
    assert f["section"] == "MODE_DRIVE"


# ── 引数の解釈 ──────────────────────────────────────────────────


def test_parse_steps_same_as_core() -> None:
    assert run.parse_steps("1,3") == [1, 3]
    assert run.parse_steps(" 1, 24 ") == [1, 24]
    with pytest.raises(SystemExit):
        run.parse_steps("1,x")
    with pytest.raises(SystemExit):
        run.parse_steps("1,99")


@pytest.mark.parametrize(
    ("numbers", "mode", "limit", "expect"),
    [
        ([1, 24], None, 60.0, None),
        ([1, 2], None, None, None),
        ([1, 24], "09_US06", None, None),
        ([1, 21, 22], None, None, None),
        ([1, 0], None, None, "動かせません"),
        ([1, 4], None, None, "動かせません"),
        ([1, 11], None, None, "動かせません"),
        ([1, 3], None, None, "動かせません"),  # 手順 3 は src では使わない
        ([1, 21, 22, 24, 25], None, 60.0, None),  # D-15b 通しの流れ
        ([1, 21, 22, 13, 25], None, None, None),
        ([1, 24, 25], None, 60.0, None),
        ([1, 2, 25], None, None, None),
        ([1, 21, 22, 2, 25], None, None, None),
        ([1, 2, 24], None, None, "同じ実行にできません"),  # 学習と 23・24 は同居不可
        ([1, 13, 23], None, None, "同じ実行にできません"),
        ([1, 2, 24], "09_US06", None, "学習の手順"),
        ([1, 2], "09_US06", None, "手順 24"),
        ([1, 24], None, 0.0, "正の秒数"),
    ],
)
def test_check_job(numbers, mode, limit, expect) -> None:  # noqa: ANN001
    result = run.check_job(numbers, mode=mode, limit_s=limit)
    if expect is None:
        assert result is None
    else:
        assert result is not None and expect in result


def test_pick_mode_name() -> None:
    rows = [("id-1", "01_WLTP"), ("id-2", "09_US06")]
    assert run.pick_mode_name("09_US06", rows, []) == "09_US06"
    assert run.pick_mode_name("id-1", rows, []) == "01_WLTP"
    assert run.pick_mode_name("short", [], ["short"]) == "short"  # 合成モードは DB に無くてよい
    with pytest.raises(ConfigError):
        run.pick_mode_name("nothing", rows, [])


async def test_select_mode_writes_config(tmp_path: Path) -> None:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: 01_WLTP\n", encoding="utf-8")
    cfg = cs.load_config(common)

    async def fetch(_url: str) -> list[tuple[str, str]]:
        return [("id-2", "09_US06")]

    assert await run.select_mode(cfg, "id-2", fetch=fetch) == "09_US06"
    assert cfg.modes.wltp_mode_name == "09_US06"
    saved = yaml.safe_load(common.read_text(encoding="utf-8"))
    assert saved["modes"]["wltp_mode_name"] == "09_US06"


async def test_select_mode_prefers_database_url_env(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: 01_WLTP\n", encoding="utf-8")
    cfg = cs.load_config(common)
    urls: list[str] = []

    async def fetch(url: str) -> list[tuple[str, str]]:
        urls.append(url)
        return [("id-2", "09_US06")]

    monkeypatch.setenv("DATABASE_URL", "postgresql://localhost/copy_db")
    await run.select_mode(cfg, "id-2", fetch=fetch)
    monkeypatch.delenv("DATABASE_URL")
    await run.select_mode(cfg, "id-2", fetch=fetch)
    assert urls == ["postgresql://localhost/copy_db", cfg.hardware.database_url]


# ── 対話コマンド ────────────────────────────────────────────────


def test_parse_command() -> None:
    c = parse_command('{"cmd":"jog","axis":"accel","amount_pct":-2.5,"id":7}')
    assert (c.cmd, c.axis, c.value, c.req_id) == ("jog", "accel", -2.5, 7)
    assert parse_command('{"cmd":"move","axis":"brake","target_pct":20}').value == 20.0
    assert parse_command('{"cmd":"home"}').cmd == "home"
    assert parse_command('{"cmd":"save_min","axis":"both"}').axis == "both"


@pytest.mark.parametrize(
    "line",
    [
        "",
        "not json",
        "[1]",
        '{"cmd":"fly"}',
        '{"cmd":"jog","axis":"clutch","amount_pct":1}',
        '{"cmd":"jog","axis":"accel"}',
        '{"cmd":"jog","axis":"accel","amount_pct":0}',
        '{"cmd":"jog","axis":"accel","amount_pct":"1"}',
        '{"cmd":"jog","axis":"accel","amount_pct":1000}',
        '{"cmd":"move","axis":"accel","target_pct":-1}',
        '{"cmd":"save_min"}',
    ],
)
def test_parse_command_errors(line: str) -> None:
    with pytest.raises(CommandError):
        parse_command(line)


LIM = Limits(min_accel=6.0, max_accel=60.0, min_brake=3.0, max_brake=40.0)


def test_plan_move_clamps_and_one_pedal_rule() -> None:
    # 単純な移動
    p = plan_move("accel", 20.0, 6.0, 3.0, LIM)
    assert p.moves == (("accel", 20.0),) and not p.clamped and not p.released_other
    # max で頭打ち
    p = plan_move("accel", 99.0, 6.0, 3.0, LIM)
    assert p.target_pct == 60.0 and p.clamped
    # ブレーキを踏んだままアクセルを踏む → 先にブレーキを min へ
    p = plan_move("accel", 20.0, 6.0, 15.0, LIM)
    assert p.moves == (("brake", 3.0), ("accel", 20.0)) and p.released_other
    # アクセルを min 以下へ戻す動きではブレーキを戻さない
    p = plan_move("accel", 6.0, 20.0, 15.0, LIM)
    assert p.moves == (("accel", 6.0),)
    # 同じ軸を戻すだけならもう一方に触らない
    p = plan_move("brake", 0.0, 6.0, 15.0, LIM)
    assert p.moves == (("brake", 0.0),)
    # 負は 0 に
    assert plan_move("brake", -5.0, 6.0, 3.0, LIM).target_pct == 0.0


def test_jog_target() -> None:
    assert jog_target("accel", 2.0, 10.0, 5.0) == 12.0
    assert jog_target("brake", -9.0, 10.0, 5.0) == 0.0
    # 画面の 10 pulse（≒0.105%）を足しても 0.1% の目盛りに止まる
    assert jog_target("brake", 10 / 95, 0.0, 4.105) == 4.2
    assert jog_target("brake", -10 / 95, 0.0, 4.2) == 4.1


def test_saved_min_max_snap_to_jog_grid() -> None:
    from src.robot.interactive import on_jog_grid

    assert on_jog_grid(4494 / 95) == 47.3  # 47.305%（47.3% の行き先の pulse）→ 47.3
    assert on_jog_grid(399 / 95) == 4.2


VEHICLE_YAML = """\
vehicle:
  name: car_a
  min_accel_opening_pct: 6.5   # アクセルの min
  max_accel_opening_pct: 60.0
  min_brake_opening_pct: 2.9
  max_brake_opening_pct: 40.0
"""


async def test_interactive_session_stub(
    tmp_path: Path, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    vdir = tmp_path / "vehicles" / "car_a"
    vdir.mkdir(parents=True)
    (vdir / "vehicle.yaml").write_text(VEHICLE_YAML, encoding="utf-8")
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    cfg = cs.load_config(common)
    cfg.vehicle.min_accel_opening_pct, cfg.vehicle.max_accel_opening_pct = 6.5, 60.0
    cfg.vehicle.min_brake_opening_pct, cfg.vehicle.max_brake_opening_pct = 2.9, 40.0

    hw = core_hardware.build_hardware(cfg, core_hardware.HW_STUB)
    await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    sess = InteractiveSession(hw, cfg, vehicle_id="car_a", vehicles_dir=tmp_path / "vehicles")
    sink_buf = io.StringIO()
    events.set_sink(events.EventSink(stream=sink_buf))
    try:
        monkeypatch.setattr("src.robot.interactive.SETTLE_S", 0.0)
        jog = '{"cmd":"jog","axis":"accel","amount_pct":10,"id":1}'
        assert await sess.execute(parse_command(jog))
        a, b = await sess.read_openings()
        assert round(a, 1) == 10.0 and b == 0.0
        # アクセルが min 超の間にブレーキを踏む → 先にアクセルを min へ
        assert await sess.execute(parse_command('{"cmd":"move","axis":"brake","target_pct":20}'))
        a, b = await sess.read_openings()
        assert round(a, 1) == 6.5 and round(b, 1) == 20.0
        # max で頭打ち
        await sess.execute(parse_command('{"cmd":"move","axis":"brake","target_pct":90}'))
        assert round((await sess.read_openings())[1], 1) == 40.0
        await sess.execute(parse_command('{"cmd":"home"}'))
        assert await sess.read_openings() == (0.0, 0.0)
        # save_min: 今の位置を車両 yaml へ（コメントは保つ）
        await sess.execute(parse_command('{"cmd":"move","axis":"accel","target_pct":7.25}'))
        await sess.execute(parse_command('{"cmd":"save_min","axis":"accel"}'))
        text = (vdir / "vehicle.yaml").read_text(encoding="utf-8")
        # 保存は 0.1% のジョグの目盛りに丸める（7.25 → 7.3）
        assert "min_accel_opening_pct: 7.3" in text and "# アクセルの min" in text
        assert cfg.vehicle.min_accel_opening_pct == 7.3
        # 0 は min にできる（max 未満）が max 以上は拒否
        assert await sess.execute(parse_command('{"cmd":"status"}'))
        assert not await sess.execute(parse_command('{"cmd":"quit"}'))
    finally:
        events.set_sink(None)
    got = [events.decode_event_line(ln) for ln in sink_buf.getvalue().splitlines()]
    acks = [e for e in got if e and e["type"] == "ack"]
    assert acks[0]["id"] == 1 and acks[0]["ok"] is True
    assert any("片足ルール" in a["message"] for a in acks)
    assert any("max" in a["message"] for a in acks)
    assert any(e and e["type"] == "status" for e in got)
    capsys.readouterr()


# ── 中断 ──────────────────────────────────────────────────────


async def test_stop_controller_cancels_inner_only() -> None:
    stop = StopController()

    async def long_job() -> None:
        await asyncio.sleep(30)

    loop = asyncio.get_running_loop()
    loop.call_later(0.1, stop.request, "SIGINT")
    loop.call_later(0.2, stop.request, "SIGINT")  # 2 回目は無視される
    with pytest.raises(Interrupted):
        await stop.run(long_job)
    assert stop.requested and stop.exit_code == 130
    # 中断後は新しい手順を始めない
    with pytest.raises(Interrupted):
        await stop.run(long_job)
    # 外側のタスクは cancel されていない（終了処理を続けられる）
    await asyncio.sleep(0)


async def test_stop_controller_normal_and_error() -> None:
    stop = StopController()

    async def ok() -> int:
        return 5

    async def bad() -> None:
        raise RuntimeError("x")

    assert await stop.run(ok) == 5
    with pytest.raises(RuntimeError):
        await stop.run(bad)


async def test_stop_controller_recancels_swallowed_cancel() -> None:
    """通信の応答待ちが cancel を握りつぶしても、繰り返しの cancel で止まる。"""
    stop = StopController()
    swallowed = 0

    async def stubborn() -> None:
        nonlocal swallowed
        while True:
            try:
                await asyncio.sleep(30)
            except asyncio.CancelledError:
                swallowed += 1
                if swallowed >= 2:
                    raise

    asyncio.get_running_loop().call_later(0.05, stop.request, "SIGTERM")
    with pytest.raises(Interrupted):
        await stop.run(stubborn)
    assert swallowed >= 2 and stop.exit_code == 143


# ── 差し替え ──────────────────────────────────────────────────


def test_patch_targets_exist_in_core() -> None:
    """core を取り込み直したとき、差し替える名前が消えていないこと。"""
    assert callable(core_main.write_mode_report)
    assert callable(core_main.pre_drive_check)
    assert callable(core_main.shutdown)
    assert callable(core_hardware.InitReport.add) and callable(core_hardware.InitReport.skip)
    assert core_main.run_steps  # run_steps が上の名前をグローバルから引く


def test_install_all_replaces_session_log() -> None:
    patches.install_all(StopController())
    assert core_main.SessionLog is StreamingSessionLog
    assert core_pattern_drive.SessionLog is StreamingSessionLog
    patches.install_all(StopController())  # 2 回目は何もしない（例外にならない）


async def test_in_thread_runs_and_interrupts() -> None:
    """同期の重い準備（ループを止める）を別スレッドへ逃がし、その間も中断要求を受けられる。"""
    import time  # noqa: PLC0415

    stop = StopController()

    async def heavy() -> str:
        time.sleep(0.3)  # ループを止める同期処理の代わり
        return "done"

    assert await stop.run(lambda: stop.in_thread(heavy)) == "done"

    async def very_heavy() -> None:
        time.sleep(2.0)

    asyncio.get_running_loop().call_later(0.1, stop.request, "SIGINT")
    t0 = time.monotonic()
    with pytest.raises(Interrupted):
        await stop.run(lambda: stop.in_thread(very_heavy))
    assert time.monotonic() - t0 < 1.0  # 準備の終わり（2 秒後）を待たずに戻る
    assert stop.orphan_alive  # 見捨てたスレッドはまだ動いている


def test_learning_prepare_runs_before_button_steps() -> None:
    """D-15b: 学習の手順の準備は、手順 21・22 のボタン操作の前に済む（core の preload_target）。"""
    ns = run.argparse.Namespace
    steps = core_main.resolve_steps(ns(only=None, steps="1,21,22,13,25", upto=None))
    index21 = [s.number for s in steps].index(21)
    target = core_main.preload_target(steps, index21)
    assert target is not None and target.number == 13 and target.prepare is not None
    assert core_main.preload_target(steps, index21 + 1) is None  # 22 では再度前倒ししない
    assert core_main.button_step_conflict(steps) is None
    steps24 = core_main.resolve_steps(ns(only=None, steps="1,21,22,24,25", upto=None))
    assert core_main.preload_target(steps24, 1).number == 24  # type: ignore[union-attr]
