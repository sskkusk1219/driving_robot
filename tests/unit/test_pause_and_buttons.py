# ruff: noqa: E501
"""段11: 一時停止・再開（JobManager・API）、ボタン押下イベント、|偏差| の最大・p95（src/app/job_manager.py）。"""

from __future__ import annotations

import asyncio
import time
from datetime import UTC, datetime
from pathlib import Path

import pytest
from httpx import ASGITransport, AsyncClient

from src.app import job_manager as jm
from src.web.app import app
from tests.unit.test_job_manager import make_ready


def _status(kind: str = jm.KIND_MODE_DRIVE) -> jm.JobStatus:
    return jm.JobStatus(
        job_id="j1", spec=jm.JobSpec(kind=kind, vehicle_id="v"), started_at=datetime.now(tz=UTC), t0=time.monotonic()
    )


def _sample(t: float, ref: float | None, speed: float, section: str = "MODE_DRIVE") -> dict:
    return {"type": "sample", "t_s": t, "mode_time_s": t, "ref_kmh": ref, "speed_kmh": speed, "section": section}


# ── ボタンの押下 ──────────────────────────────────────────────


def test_button_events_use_latest_sample_time_and_pair_start_end() -> None:
    st = _status()
    jm.apply_event(st, {"type": "button", "button": "start", "phase": "start", "seconds": 1.0})
    jm.apply_event(st, _sample(3.4, 10.0, 10.0))
    jm.apply_event(st, {"type": "button", "button": "start", "phase": "end", "seconds": 1.0})
    jm.apply_event(st, {"type": "button", "button": "D", "phase": "start", "seconds": 1.2})
    assert st.button_events[0] == {"button": "start", "seconds": 1.0, "start_s": 0.0, "end_s": 3.4}  # sample 前は 0
    assert st.button_events[1]["button"] == "D" and st.button_events[1]["start_s"] == 3.4 and st.button_events[1]["end_s"] is None
    out = st.to_dict()
    assert out["button_events"][1]["end_s"] is None and out["paused"] is False


def test_button_events_fall_back_to_t_s_and_are_capped() -> None:
    st = _status(jm.KIND_LEARNING)
    jm.apply_event(st, {"type": "sample", "t_s": 7.0, "mode_time_s": None, "section": "PEDAL_SEARCH"})
    for _ in range(jm.BUTTON_EVENTS_MAX + 5):
        jm.apply_event(st, {"type": "button", "button": "P", "phase": "start", "seconds": 1.0})
    assert len(st.button_events) == jm.BUTTON_EVENTS_MAX
    assert st.button_events[-1]["start_s"] == 7.0


# ── |偏差| の最大・p95 ────────────────────────────────────────


def test_dev_stats_count_only_mode_drive_and_skip_paused_rows() -> None:
    st = _status()
    assert st.dev_stats() is None
    for i in range(100):
        jm.apply_event(st, _sample(float(i), 10.0, 10.0 + (i + 1) * 0.01))  # 偏差 0.01 … 1.00
    jm.apply_event(st, _sample(200.0, 10.0, 30.0, section="PEDAL_SEARCH"))  # モード走行でない行は除く
    jm.apply_event(st, _sample(201.0, None, 30.0))  # 基準なしは除く
    st.paused = True
    jm.apply_event(st, _sample(202.0, 10.0, 50.0))  # 一時停止中の行は除く
    st.paused = False
    jm.apply_event(st, {**_sample(203.0, 10.0, 50.0), "paused": True})  # paused 印の行も除く
    s = st.dev_stats()
    assert s is not None and s["n"] == 100
    assert s["max"] == 1.0 and s["p95"] == 0.95


# ── 一時停止・再開（偽ランナーのシグナル） ─────────────────────


async def _manager(tmp_path: Path, codes: str = "p,p,p") -> jm.JobManager:
    """初期化まで済んだ JobManager（偽の常駐ランナー。ジョブは p = 一時停止・再開を受けて stop まで走る）。"""
    return await make_ready(tmp_path, codes)


async def _wait_sample(mgr: jm.JobManager) -> None:
    for _ in range(100):
        if mgr.current and mgr.current.sample:
            return
        await asyncio.sleep(0.1)
    raise AssertionError("sample が来ない")


async def test_pause_and_resume_send_signals_and_track_flag(tmp_path: Path) -> None:
    mgr = await _manager(tmp_path)
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await _wait_sample(mgr)
    assert mgr.current.button_events and mgr.current.button_events[0]["button"] == "start"
    assert mgr.current.button_events[0]["end_s"] is not None
    await mgr.pause()
    assert mgr.snapshot()["job"]["paused"] is True
    with pytest.raises(jm.JobConflict, match="すでに一時停止中"):
        await mgr.pause()
    for _ in range(50):
        if any("GOT-USR1" in m for m in mgr.recent_messages):
            break
        await asyncio.sleep(0.1)
    assert any("GOT-USR1" in m for m in mgr.recent_messages)
    await mgr.resume()
    assert mgr.snapshot()["job"]["paused"] is False
    with pytest.raises(jm.JobConflict, match="一時停止中ではありません"):
        await mgr.resume()
    for _ in range(50):
        if any("GOT-USR2" in m for m in mgr.recent_messages):
            break
        await asyncio.sleep(0.1)
    assert any("GOT-USR2" in m for m in mgr.recent_messages)
    await mgr.stop()
    await mgr.wait_idle(15)


async def test_pause_refused_when_not_mode_drive(tmp_path: Path) -> None:
    mgr = await _manager(tmp_path)
    with pytest.raises(jm.JobConflict, match="モード走行の実行中だけ"):
        await mgr.pause()  # ジョブなし
    await mgr.start_learning("v1", "full")
    await _wait_sample(mgr)
    with pytest.raises(jm.JobConflict, match="モード走行の実行中だけ"):
        await mgr.pause()  # 学習
    with pytest.raises(jm.JobConflict):
        await mgr.resume()
    await mgr.stop()
    await mgr.wait_idle(15)


# ── API ──────────────────────────────────────────────────────


@pytest.fixture
async def client(tmp_path: Path):
    app.state.job_manager = await _manager(tmp_path)
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        yield c
    await app.state.job_manager.shutdown()


async def test_pause_resume_api_409_with_japanese_message_without_job(client: AsyncClient) -> None:
    for path in ("/api/v1/drive/pause", "/api/v1/drive/resume"):
        r = await client.post(path)
        assert r.status_code == 409
        assert "モード走行" in r.json()["detail"]


async def test_pause_resume_api_with_mode_drive(client: AsyncClient) -> None:
    mgr = app.state.job_manager
    await mgr.start_mode_drive("v1", "m", "WLTP")
    await _wait_sample(mgr)
    r = await client.post("/api/v1/drive/pause")
    assert r.status_code == 200 and r.json()["paused"] is True
    assert (await client.post("/api/v1/drive/pause")).status_code == 409
    r = await client.post("/api/v1/drive/resume")
    assert r.status_code == 200 and r.json()["paused"] is False
    await mgr.stop()
    await mgr.wait_idle(15)


# ── 読み直し用の最近の sample（2026-11-16）──────────────────────


def test_sample_history_keeps_recent_samples_with_age(monkeypatch: pytest.MonkeyPatch) -> None:
    st = _status()
    t0 = time.monotonic()
    jm.apply_event(st, {**_sample(1.0, 10.0, 9.5), "accel_actual_pct": 12.0, "brake_actual_pct": 4.2}, now=t0 - 2.0)
    jm.apply_event(st, {**_sample(2.0, 11.0, 10.5), "accel_cmd_pct": 13.0, "brake_cmd_pct": 4.2}, now=t0 - 1.0)
    monkeypatch.setattr(jm.JobManager, "current", property(lambda self: st))
    h = jm.JobManager.__new__(jm.JobManager).sample_history()
    assert h["job_id"] == "j1"
    assert [p["actual_speed_kmh"] for p in h["samples"]] == [9.5, 10.5]
    assert h["samples"][0]["accel_opening"] == 12.0 and h["samples"][1]["accel_opening"] == 13.0
    assert 1900 <= h["samples"][0]["age_ms"] <= 2600 and h["samples"][1]["age_ms"] < h["samples"][0]["age_ms"]


def test_sample_history_is_capped() -> None:
    st = _status()
    for i in range(jm.SAMPLE_HISTORY_MAX + 50):
        jm.apply_event(st, _sample(float(i), 10.0, 10.0), now=float(i))
    assert len(st.sample_history) == jm.SAMPLE_HISTORY_MAX
