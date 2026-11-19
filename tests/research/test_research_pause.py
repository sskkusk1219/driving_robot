"""一時停止（2026-11-16 段11）のテスト: モードの時刻の凍結・保つ基準・KPI からの除外。

SIGUSR1 で一時停止・SIGUSR2 で再開。スタブ HW と偽の FF で数秒のモードを走らせ、走行中に
シグナルを送る。受信しなければ今までと同じ（印の列が 0・一時停止の行なし）ことも確かめる。
"""

from __future__ import annotations

import asyncio
import csv
import os
import signal
import time
from pathlib import Path

import pytest

from tests.research import drive_log as dlmod
from tests.research import mode_drive as mdmod
from tests.research import mode_report as mrmod
from tests.research.research_types import VEHICLE_STOP_SPEED_KMH
from tests.research.test_research_mode_drive import FakeFF, _held_stub, _mode, _tmp_cfg


def _factory(v_kmh: float) -> mdmod.ModeDriveSetup:
    return mdmod.ModeDriveSetup(mdmod.make_pause_mode(v_kmh, 30.0), FakeFF())  # type: ignore[arg-type]


def _slow_factory(build_s: float):  # noqa: ANN202
    def make(v_kmh: float) -> mdmod.ModeDriveSetup:
        time.sleep(build_s)  # 別スレッドで走るので、本体のモードは止まらない
        return _factory(v_kmh)

    return make


async def _run(
    tmp_path: Path, sends: list[tuple[float, int]], *, build_s: float = 0.0
) -> tuple[mdmod.ModeDriveResult, dlmod.SessionLog]:
    cfg = _tmp_cfg(tmp_path)
    cfg.mode_drive.pedal_standby = False
    cfg.forward_map.hold_pi_enabled = False  # 偽の FF は表の候補ではない
    hw = await _held_stub(cfg)
    mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (7.0, 6.0), (7.6, 0.0), (8.0, 0.0)])
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    loop = asyncio.get_running_loop()
    for delay, sig in sends:
        loop.call_later(delay, os.kill, os.getpid(), sig)
    result = await mdmod.run_mode_drive(
        hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()),  # type: ignore[arg-type]
        log=log, pause_setup_factory=_slow_factory(build_s),
    )
    await log.close()
    return result, log


def test_pause_hold_speed_keeps_speed_or_zero_when_stopped() -> None:
    assert mdmod.pause_hold_speed(42.5) == 42.5
    assert mdmod.pause_hold_speed(VEHICLE_STOP_SPEED_KMH) == 0.0
    assert mdmod.pause_hold_speed(0.0) == 0.0


def test_pause_mode_is_constant_reference() -> None:
    m = mdmod.make_pause_mode(37.0, 60.0)
    ref = mdmod.ReferenceSpeed(m)
    assert {ref.at(t) for t in (0.0, 13.3, 59.9, 80.0)} == {37.0}
    assert m.total_duration == 60.0 and m.max_speed == 37.0


async def test_no_signal_is_unchanged(tmp_path: Path) -> None:
    result, log = await _run(tmp_path, [])
    assert result.completed and result.pause_count == 0 and result.paused_s == 0.0
    with log.csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert {r["paused"] for r in rows} == {"0"}
    assert not any(r["section"].endswith(mdmod.PAUSE_SECTION_SUFFIX) for r in rows)
    assert result.run_duration_s == pytest.approx(8.0, abs=0.15)


async def test_pause_freezes_mode_clock_holds_reference_and_excludes_kpi(tmp_path: Path) -> None:
    result, log = await _run(tmp_path, [(3.0, signal.SIGUSR1), (5.0, signal.SIGUSR2)])
    assert result.completed
    assert result.pause_count == 1
    assert result.paused_s == pytest.approx(2.0, abs=0.2)
    # モードの時刻は止めた分だけ進まない: 走ったモードの長さは 8s のまま
    assert result.run_duration_s == pytest.approx(8.0, abs=0.2)

    with log.csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    paused = [r for r in rows if r["paused"] == "1"]
    mode_rows = [r for r in rows if r["section"] == dlmod.SECTION_MODE_DRIVE]
    assert len(paused) >= 30  # 2s ≒ 40 周期
    assert {r["section"] for r in paused} == {dlmod.SECTION_MODE_DRIVE + mdmod.PAUSE_SECTION_SUFFIX}
    # 凍結: 一時停止中のモードの時刻は止めた時刻のまま
    t_frozen = {r["mode_time_s"] for r in paused}
    assert len(t_frozen) == 1
    t_p = float(next(iter(t_frozen)))
    assert 2.5 < t_p < 3.5
    # 保つ基準: 一時停止中の基準は一定（定速）
    refs = {r["ref_speed_kmh"] for r in paused}
    assert len(refs) == 1
    # 再開は止めた時刻から続く（時刻が飛ばない）
    times = [float(r["mode_time_s"]) for r in mode_rows]
    after = [x for x in times if x >= t_p - 1e-6]
    assert min(x for x in times if x > t_p + 1e-6) - t_p < 0.2
    assert times == sorted(times)
    assert after[-1] == pytest.approx(8.0, abs=0.15)

    # KPI・レポートの入力から一時停止の行は外れる（samples・CSV の両方）
    n_paused_samples = sum(1 for s in result.samples if s.paused)
    assert n_paused_samples == len(paused)
    kept = mrmod.rows_from_samples(result.samples)
    assert len(kept) == len(mode_rows)
    from_csv = mrmod.rows_from_csv(log.csv_path)
    assert len(from_csv) == len(mode_rows)


async def test_pause_ignored_when_resume_without_pause(tmp_path: Path) -> None:
    result, _log = await _run(tmp_path, [(2.0, signal.SIGUSR2)])
    assert result.completed and result.pause_count == 0


def test_pause_not_allowed_before_d_in_button_run() -> None:
    run = object.__new__(mdmod._ModeRun)  # noqa: SLF001 - 単体テストで直接組み立てる
    run.button_plan = object()  # type: ignore[assignment]
    run._d_finished_at = None  # noqa: SLF001
    assert run._pause_allowed() is False  # noqa: SLF001
    run._d_finished_at = 1.0  # noqa: SLF001
    assert run._pause_allowed() is True  # noqa: SLF001
    run.button_plan = None
    run._d_finished_at = None  # noqa: SLF001
    assert run._pause_allowed() is True  # noqa: SLF001


async def test_clock_freezes_at_signal_cycle_even_when_building_is_slow(tmp_path: Path) -> None:
    # 実機 1 回目の直し: 制御を作るのに時間がかかっても、モードの時刻は押された周期で止まる
    # （作る間に進んで、再開で基準とずれることがない）。作る間の行も一時停止の行（paused=1）
    result, log = await _run(
        tmp_path, [(3.0, signal.SIGUSR1), (5.5, signal.SIGUSR2)], build_s=0.5
    )
    assert result.completed and result.pause_count == 1
    with log.csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    first_paused = next(i for i, r in enumerate(rows) if r["paused"] == "1")
    t_p = float(rows[first_paused]["mode_time_s"])
    prev_t = float(rows[first_paused - 1]["mode_time_s"])
    # 止めた時刻は、直前のモード行の次の周期（約 0.05 s 後）で、作った 0.5 s の分は進まない
    assert t_p - prev_t < 0.15
    frozen = {r["mode_time_s"] for r in rows if r["paused"] == "1"}
    assert frozen == {rows[first_paused]["mode_time_s"]}
    # 再開は止めた時刻から続く（基準が飛ばない）
    after = next(r for r in rows[first_paused:] if r["paused"] == "0")
    assert float(after["mode_time_s"]) - t_p < 0.15
    assert result.run_duration_s == pytest.approx(8.0, abs=0.3)
    assert result.paused_s == pytest.approx(2.5, abs=0.3)


async def test_default_pause_build_is_fast_for_every_speed() -> None:
    # 実機 1 回目: 40 秒のモードを別スレッドで作って 2.7 秒かかった（速度によっては表の引き直しが
    # 毎行に入り、時間は長さに比例して 16 倍）。短い定速モードならどの速度でも 1 秒かからない
    from tests.research.config import load_config  # noqa: PLC0415

    cfg = load_config()
    mdmod.default_pause_setup(cfg, 30.0)  # 1 回目は読み込みを含む
    for v in (0.0, 20.0, 20.37, 34.3, 100.0):
        t0 = time.perf_counter()
        setup = mdmod.default_pause_setup(cfg, v)
        assert time.perf_counter() - t0 < 1.0, v
        assert setup.mode.total_duration == mdmod.PAUSE_PLAN_S


async def test_pause_longer_than_plan_wraps_time_and_keeps_running(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # 保つ車速のモードの長さを短くして、一時停止が 1 周以上続いても時刻の巻き戻しで落ちない
    monkeypatch.setattr(mdmod, "PAUSE_PLAN_S", 5.0)
    result, log = await _run(tmp_path, [(2.5, signal.SIGUSR1), (5.0, signal.SIGUSR2)])
    assert result.completed and result.pause_count == 1
    assert result.paused_s == pytest.approx(2.5, abs=0.3)
