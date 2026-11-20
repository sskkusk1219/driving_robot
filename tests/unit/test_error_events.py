# ruff: noqa: E501
"""エラーページの記録（RPD_Phase_20261120_ErrorLog）: 記録係の規則・InMemory リポジトリ・サーバーログのハンドラ・JobManager の差し込み。"""

from __future__ import annotations

import asyncio
import logging
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import pytest

from src.app import job_manager as jm
from src.app.error_recorder import ErrorRecorder
from src.app.inmemory_repos import InMemoryErrorRepository
from src.infra.error_log_handler import RECORDED, ErrorLogHandler
from src.models.error_event import (
    LEVEL_ERROR,
    LEVEL_WARNING,
    SOURCE_EMERGENCY,
    SOURCE_IDLE,
    SOURCE_JOB,
    NewErrorEvent,
)
from tests.unit.test_job_manager import FakeRecorder, make_ready, runner_starts, wait_until

NOW = datetime(2026, 11, 20, 9, 0, 0, tzinfo=UTC)


class Clock:
    def __init__(self) -> None:
        self.t = 1000.0

    def __call__(self) -> float:
        return self.t


def ev(title: str = "t", *, source: str = SOURCE_IDLE, level: str = LEVEL_WARNING, dedupe: bool = True, **kw: Any) -> NewErrorEvent:
    return NewErrorEvent(level=level, source=source, title=title, detail=title, dedupe=dedupe, **kw)


async def started(repo: InMemoryErrorRepository | None = None, clock: Clock | None = None) -> tuple[ErrorRecorder, InMemoryErrorRepository]:
    repo = repo or InMemoryErrorRepository()
    rec = ErrorRecorder(repo, clock=clock or Clock())
    rec.start()
    return rec, repo


# ── InMemory リポジトリ ───────────────────────────────────────


async def test_inmemory_repo_list_filter_ack_delete() -> None:
    repo = InMemoryErrorRepository()
    a = await repo.insert(ev("a", level=LEVEL_ERROR, source="job"), at=datetime(2026, 11, 20, 9, 0, tzinfo=UTC))
    b = await repo.insert(ev("b", level=LEVEL_WARNING, source="idle"), at=datetime(2026, 11, 20, 9, 1, tzinfo=UTC))
    c = await repo.insert(ev("c", level=LEVEL_ERROR, source="browser"), at=datetime(2026, 11, 20, 9, 2, tzinfo=UTC))
    items, total = await repo.list_events()
    assert [e.id for e in items] == [c, b, a] and total == 3  # 新しい順
    assert [e.id for e in (await repo.list_events(level="ERROR"))[0]] == [c, a]
    assert [e.id for e in (await repo.list_events(source="idle"))[0]] == [b]
    page, total = await repo.list_events(limit=2, offset=1)
    assert [e.id for e in page] == [b, a] and total == 3
    assert await repo.set_acknowledged(b, True, at=NOW)
    assert [e.id for e in (await repo.list_events(unacknowledged_only=True))[0]] == [c, a]
    assert await repo.acknowledge_all(at=NOW) == 2
    assert await repo.delete_acknowledged() == 3 and (await repo.list_events())[1] == 0
    assert await repo.delete(a) is False and await repo.get(a) is None


async def test_inmemory_bump_resets_ack_and_append_detail() -> None:
    repo = InMemoryErrorRepository()
    i = await repo.insert(ev("x"), at=NOW)
    await repo.set_acknowledged(i, True, at=NOW)
    later = datetime(2026, 11, 20, 9, 5, tzinfo=UTC)
    assert await repo.bump(i, at=later)
    e = await repo.get(i)
    assert e is not None and e.count == 2 and e.last_at == later and e.first_at == NOW and e.acknowledged_at is None
    assert await repo.append_detail(i, "追記", at=later, context_patch={"k": 1})
    e = await repo.get(i)
    assert e is not None and e.detail == "x\n追記" and e.context == {"k": 1}
    assert await repo.bump(999, at=later) is False


# ── 記録係の規則 ─────────────────────────────────────────────


async def test_same_text_within_60s_adds_count_and_after_window_makes_new_event() -> None:
    clock = Clock()
    rec, repo = await started(clock=clock)
    rec.record(ev("同じ文"))
    rec.record(ev("同じ文"))
    clock.t += 30
    rec.record(ev("同じ文"))
    rec.record(ev("別の文"))
    await rec.flush()
    items, _ = await repo.list_events()
    by_title = {e.title: e for e in items}
    assert by_title["同じ文"].count == 3 and by_title["別の文"].count == 1
    clock.t += 61  # 最後の発生（30 秒時点）から 60 秒を超えた
    rec.record(ev("同じ文"))
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 3 and sorted(e.count for e in items if e.title == "同じ文") == [1, 3]
    await rec.stop()


async def test_non_dedupe_events_are_never_merged() -> None:
    rec, repo = await started()
    rec.record(ev("job", source=SOURCE_JOB, dedupe=False))
    rec.record(ev("job", source=SOURCE_JOB, dedupe=False))
    await rec.flush()
    assert (await repo.list_events())[1] == 2
    await rec.stop()


async def test_deleted_event_is_not_bumped_but_recreated() -> None:
    rec, repo = await started()
    rec.record(ev("消される"))
    await rec.flush()
    (e,), _ = await repo.list_events()
    await repo.delete(e.id)
    rec.record(ev("消される"))
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1 and items[0].count == 1
    await rec.stop()


async def test_emergency_gets_finish_text_appended_and_close_stops_appending() -> None:
    rec, repo = await started()
    rec.record(ev("非常停止（画面）", source=SOURCE_EMERGENCY, level=LEVEL_ERROR, dedupe=False))
    rec.append_emergency("終了処理の結果: 終了コード 130", context_patch={"job_end": {"exit_code": 130}})
    rec.append_emergency("非常停止を解除しました", close=True)
    rec.append_emergency("閉じた後の行（捨てる）")
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1
    assert items[0].detail == "非常停止（画面）\n終了処理の結果: 終了コード 130\n非常停止を解除しました"
    assert items[0].context["job_end"]["exit_code"] == 130
    await rec.stop()


async def test_append_without_emergency_uses_fallback_event_only_when_given() -> None:
    rec, repo = await started()
    rec.append_emergency("書き足し先なし")
    rec.append_emergency("書き足し先なし（代わり）", fallback=ev("ジョブの件", source=SOURCE_JOB, level=LEVEL_ERROR, dedupe=False))
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1 and items[0].title == "ジョブの件"
    await rec.stop()


async def test_repo_failure_does_not_raise_and_logs_once(caplog: pytest.LogCaptureFixture) -> None:
    class Broken(InMemoryErrorRepository):
        async def insert(self, ev: NewErrorEvent, *, at: datetime) -> int:
            raise OSError("DB が落ちた")

    rec, _ = await started(Broken())
    with caplog.at_level(logging.WARNING):
        for i in range(3):
            rec.record(ev(f"e{i}", dedupe=False))
        await rec.flush()
    msgs = [r for r in caplog.records if "エラーの記録を書けません" in r.getMessage()]
    assert len(msgs) == 1  # ログに 1 回だけ
    await rec.stop()


async def test_queue_overflow_drops_without_blocking() -> None:
    repo = InMemoryErrorRepository()
    rec = ErrorRecorder(repo, queue_max=2)
    rec.start()
    for i in range(10):  # ワーカーが動く前に溜める（record は待たない）
        rec.record(ev(f"e{i}", dedupe=False))
    await rec.flush()
    assert (await repo.list_events())[1] == 2
    await rec.stop()


async def test_record_before_start_or_after_stop_is_a_noop() -> None:
    repo = InMemoryErrorRepository()
    rec = ErrorRecorder(repo)
    rec.record(ev("開始前"))  # 例外にならない
    rec.start()
    await rec.stop()
    rec.record(ev("停止後"))
    assert (await repo.list_events())[1] == 0


# ── サーバーログのハンドラ ───────────────────────────────────


def _record(name: str, level: int, msg: str, **extra: Any) -> logging.LogRecord:
    r = logging.LogRecord(name, level, __file__, 1, msg, (), None)
    for k, v in extra.items():
        setattr(r, k, v)
    return r


def test_log_handler_records_warning_and_above_and_skips_marked_and_own_loggers() -> None:
    got: list[NewErrorEvent] = []
    h = ErrorLogHandler(got.append)
    h.emit(_record("src.web.x", logging.INFO, "情報"))
    h.emit(_record("src.web.x", logging.WARNING, "警告です\n2 行目"))
    h.emit(_record("uvicorn.error", logging.ERROR, "エラーです"))
    h.emit(_record("src.app.job_manager", logging.WARNING, "記録済み", **RECORDED))
    h.emit(_record("src.app.error_recorder", logging.WARNING, "記録係自身"))
    h.emit(_record("src.infra.error_repository", logging.ERROR, "リポジトリ自身"))
    assert [(e.level, e.title, e.source, e.dedupe) for e in got] == [
        (LEVEL_WARNING, "警告です", "server", True),
        (LEVEL_ERROR, "エラーです", "server", True),
    ]
    assert got[0].detail == "警告です\n2 行目"


def test_log_handler_includes_traceback_and_never_raises() -> None:
    got: list[NewErrorEvent] = []
    h = ErrorLogHandler(got.append)
    try:
        raise ValueError("boom")
    except ValueError:
        import sys

        r = logging.LogRecord("src.web.x", logging.ERROR, __file__, 1, "例外", (), sys.exc_info())
    h.emit(r)
    assert "Traceback" in got[0].detail and "ValueError: boom" in got[0].detail

    def bad(_: NewErrorEvent) -> None:
        raise RuntimeError("x")

    ErrorLogHandler(bad).emit(_record("src.web.x", logging.WARNING, "w"))  # 例外を外へ出さない


def test_job_manager_log_lines_carry_recorded_mark(tmp_path: Path, caplog: pytest.LogCaptureFixture) -> None:
    """ジョブ・待機中の行は JobManager が自分の経路で記録するので、ログ行には印が付く（二重記録しない）。"""
    mgr = jm.JobManager(vehicles_dir=tmp_path, common_path=tmp_path / "c.yaml")
    with caplog.at_level(logging.INFO):
        mgr._say_idle("待機中の警告", "WARNING")  # noqa: SLF001
    rows = [r for r in caplog.records if r.getMessage() == "待機中の警告"]
    assert rows and getattr(rows[0], "error_recorded", False) is True


# ── JobManager の差し込み ────────────────────────────────────


async def make_mgr(tmp_path: Path, codes: str = "", **kw: Any) -> tuple[jm.JobManager, ErrorRecorder, InMemoryErrorRepository]:
    repo = InMemoryErrorRepository()
    rec = ErrorRecorder(repo)
    rec.start()
    mgr = await make_ready(tmp_path, codes, FakeRecorder(), error_recorder=rec, **kw)
    return mgr, rec, repo


async def test_job_error_exit_makes_one_error_event_with_messages(tmp_path: Path) -> None:
    mgr, rec, repo = await make_mgr(tmp_path, "5")
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await mgr.wait_idle(30)
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1
    e = items[0]
    assert e.level == LEVEL_ERROR and e.source == SOURCE_JOB and e.job_kind == jm.KIND_MODE_DRIVE
    assert "異常終了" in e.title and "終了コード 5" in e.title
    assert e.vehicle_id == "v1" and e.session_id == "sess1" and e.job_id
    assert "理由: boom" in e.detail
    texts = [m["text"] for m in e.context["messages"]]
    assert any("ジョブ開始" in t for t in texts) and any("ジョブ終了" in t for t in texts) and len(texts) <= 50
    await mgr.shutdown()
    await rec.stop()


async def test_init_failure_without_session_is_recorded(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("FAKE_INIT_EXIT", "4")
    repo = InMemoryErrorRepository()
    rec = ErrorRecorder(repo)
    rec.start()
    from tests.unit.test_job_manager import make_manager

    mgr = make_manager(tmp_path, "", FakeRecorder(), error_recorder=rec)
    await mgr.start_init_only("v1")
    await mgr.wait_idle(30)
    await rec.flush()
    items, _ = await repo.list_events()
    assert len(items) == 1 and items[0].job_kind == jm.KIND_INIT_ONLY and items[0].session_id is None
    assert "初期化が異常終了" in items[0].title
    await rec.stop()


async def test_completed_with_warning_line_is_a_warning_event(tmp_path: Path) -> None:
    mgr, rec, repo = await make_mgr(tmp_path, "w")  # 0.5 秒走って 0 で終わる
    await mgr.start_mode_drive("v1", "m", "WLTP")
    assert mgr._job is not None  # noqa: SLF001
    mgr._say(mgr._job, "警告: 路面の傾きを読めませんでした", "WARNING")  # noqa: SLF001
    await mgr.wait_idle(30)
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1 and items[0].level == LEVEL_WARNING
    assert items[0].title.endswith("警告ありで完了（1 件）") and "路面の傾き" in items[0].detail
    await mgr.shutdown()
    await rec.stop()


async def test_clean_completion_and_user_stop_record_nothing(tmp_path: Path) -> None:
    mgr, rec, repo = await make_mgr(tmp_path, "0,h")
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await mgr.wait_idle(30)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await wait_until(lambda: mgr.current is not None and mgr.current.sample is not None)
    await mgr.stop()  # 利用者の停止（130）はエラーにしない
    await mgr.wait_idle(15)
    await rec.flush()
    assert (await repo.list_events())[1] == 0
    await mgr.shutdown()
    await rec.stop()


async def test_emergency_is_one_event_with_finish_result_and_no_job_event(tmp_path: Path) -> None:
    mgr, rec, repo = await make_mgr(tmp_path, "h")
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await wait_until(lambda: mgr.current is not None and mgr.current.sample is not None)
    await mgr.emergency("switch")
    await mgr.emergency("ac_loss")  # 二重に来ても件は増えない
    await mgr.wait_idle(15)
    await wait_until(lambda: mgr._runner is None)  # noqa: SLF001
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1
    e = items[0]
    assert e.source == SOURCE_EMERGENCY and e.level == LEVEL_ERROR and "非常停止スイッチ" in e.title
    assert e.job_kind == jm.KIND_MODE_DRIVE and e.session_id == "sess1"
    assert "終了処理の結果" in e.detail and "終了コード 130" in e.detail
    assert e.context["job_end"]["exit_code"] == 130
    await mgr.reset_emergency()
    await rec.flush()
    e2 = await repo.get(e.id)
    assert e2 is not None and e2.detail.endswith("非常停止を解除しました")
    await mgr.shutdown()
    await rec.stop()


async def test_idle_emergency_records_cause_and_reason(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("FAKE_WATCH_AFTER", "0.5")  # 待機中の見張りの error → 非常停止（auto）
    mgr, rec, repo = await make_mgr(tmp_path)
    await wait_until(lambda: mgr.state == jm.STATE_EMERGENCY, 15)
    await wait_until(lambda: mgr._runner is None)  # noqa: SLF001
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1  # 見張りの行は別の件にしない
    assert items[0].source == SOURCE_EMERGENCY and "自動" in items[0].title and "アラーム" in items[0].detail
    assert "常駐ランナーが終了しました" in items[0].detail
    await mgr.reset_emergency()
    await rec.stop()


async def test_idle_warning_lines_are_recorded_and_repeated_ones_counted(tmp_path: Path) -> None:
    mgr, rec, repo = await make_mgr(tmp_path)
    for _ in range(3):
        mgr._say_idle("CAN の応答が遅れています（警告）", "WARNING")  # noqa: SLF001
    mgr._say_idle("普通の行")  # noqa: SLF001
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1 and items[0].source == SOURCE_IDLE and items[0].count == 3 and items[0].level == LEVEL_WARNING
    await mgr.shutdown()
    await rec.stop()


async def test_job_lines_are_not_recorded_one_by_one(tmp_path: Path) -> None:
    mgr, rec, repo = await make_mgr(tmp_path, "w")
    await mgr.start_mode_drive("v1", "m", "WLTP")
    assert mgr._job is not None  # noqa: SLF001
    for i in range(5):
        mgr._say(mgr._job, f"警告 {i}", "WARNING")  # noqa: SLF001
    await rec.flush()
    assert (await repo.list_events())[1] == 0  # ジョブが終わるまで件は作らない
    await mgr.wait_idle(30)
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1 and "5 件" in items[0].title
    await mgr.shutdown()
    await rec.stop()


async def test_sequence_job_failure_carries_sequence_name_without_extra_event(tmp_path: Path) -> None:
    from tests.unit.test_sequence_run import plan

    mgr, rec, repo = await make_mgr(tmp_path, "0,5")
    await mgr.start_sequence("v1", "sid", "朝のシーケンス", plan(), 2)
    seq = mgr.sequence
    assert seq is not None and seq.task is not None
    await asyncio.wait_for(seq.task, 30)
    await mgr.wait_persisted()
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1  # 段のジョブの件だけ（シーケンスの件は別に作らない）
    assert items[0].source == SOURCE_JOB and items[0].sequence_name == "朝のシーケンス"
    assert "シーケンス: 「朝のシーケンス」1 段目" in items[0].detail
    await mgr.shutdown()
    await rec.stop()


async def test_sequence_stop_reason_without_job_makes_sequence_event(tmp_path: Path) -> None:
    mgr, rec, repo = await make_mgr(tmp_path)
    seq = jm.SequenceRun(
        sequence_id="s", name="S", vehicle_id="v1", steps=(), cycles=1,
        runs=[jm.RunItem(1, 1, 1, jm.PlannedStep("mode", "m", "WLTP"))],
    )
    mgr._record_sequence_error(seq, "2 段目を始められません: 初期化が済んでいません")  # noqa: SLF001
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1 and items[0].source == "sequence" and items[0].sequence_name == "S"
    await mgr.shutdown()
    await rec.stop()


async def test_default_manager_without_recorder_is_unaffected(tmp_path: Path) -> None:
    mgr = await make_ready(tmp_path, "5")
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await mgr.wait_idle(30)
    assert runner_starts(tmp_path) == 1
    await mgr.shutdown()


async def test_prepare_failure_makes_one_prepare_event(tmp_path: Path) -> None:
    from tests.unit.test_prepare import make, wait_state

    mgr, _, _ = await make(tmp_path, "fail")
    repo = InMemoryErrorRepository()
    rec = ErrorRecorder(repo)
    rec.start()
    mgr._errors = rec  # noqa: SLF001 - 記録係を後から差し込む
    await mgr.start_prepare_mode("v1", "mid", "WLTP")
    await mgr.prepare_go()
    await wait_state(mgr, jm.PREP_ERROR)
    await rec.flush()
    items, total = await repo.list_events()
    assert total == 1 and items[0].source == "prepare" and items[0].level == LEVEL_ERROR
    assert "WLTP" in items[0].title and "計画を作れませんでした" in items[0].detail and items[0].vehicle_id == "v1"
    await mgr.shutdown()
    await rec.stop()
