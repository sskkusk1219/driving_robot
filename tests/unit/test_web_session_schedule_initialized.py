# ruff: noqa: E501
"""セッションのスケジュール名（O-5）・車を切り替えても initialized は変わらない・select-profile は壊れた車の設定を 422 で断る。PostgreSQL・ランナー不要。"""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.inmemory_repos import InMemorySessionRepository
from src.models.drive_log import DriveSession
from src.web.app import app

SESSION_ID = "00000000-0000-0000-0000-0000000000a1"
SCHEDULE_ID = "00000000-0000-0000-0000-0000000000b2"


class _Store:
    common_path = Path("/nonexistent/common.yaml")
    vehicles_dir = Path("/nonexistent/vehicles")

    def __init__(self, active: str | None) -> None:
        self.active = active

    def get_active(self) -> str | None:
        return self.active

    def exists(self, vehicle_id: str) -> bool:  # noqa: ARG002
        return False

    def read(self, vehicle_id: str) -> Any:  # noqa: ARG002
        raise KeyError


class _Manager:
    """status が見る口だけを持つ偽の JobManager（初期化済み）。"""

    initialized = True

    is_busy = False
    prepare = None

    def snapshot(self) -> dict[str, Any]:
        return {
            "robot_state": "READY", "state": "standby", "initialized": True, "job": None, "last_job": None,
            "sequence": None, "last_sequence": None, "prepare": None, "emergency_cause": None,
            "emergency_cause_detail": None,
        }


class _SessionRepo(InMemorySessionRepository):
    def __init__(self, sessions: list[DriveSession]) -> None:
        self._sessions = sessions

    async def list_all(self, limit: int = 100) -> list[DriveSession]:  # noqa: ARG002
        return self._sessions

    async def get_by_id(self, session_id: str) -> DriveSession | None:
        return next((s for s in self._sessions if s.id == session_id), None)


def _session(**kw: Any) -> DriveSession:
    return DriveSession(
        id=SESSION_ID, profile_id=None, mode_id=None, run_type="auto",
        started_at=datetime(2026, 11, 20, tzinfo=UTC), ended_at=None, status="stopped", vehicle_id=None, **kw,
    )


@pytest.fixture
async def client():
    keys = ("job_manager", "vehicle_store", "session_repo")
    saved = {k: getattr(app.state, k, None) for k in keys}
    store = _Store("v1")
    app.state.job_manager, app.state.vehicle_store = _Manager(), store
    app.state.session_repo = _SessionRepo([_session(schedule_id=SCHEDULE_ID, schedule_name="メンテナンスモード_v2")])
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        c.store = store  # type: ignore[attr-defined]
        yield c
    for k, v in saved.items():
        setattr(app.state, k, v)


async def test_status_initialized_stays_true_after_vehicle_switch(client: AsyncClient) -> None:
    assert (await client.get("/api/v1/drive/status")).json()["initialized"] is True
    client.store.active = "v2"  # type: ignore[attr-defined]  # select-profile で別の車へ
    assert (await client.get("/api/v1/drive/status")).json()["initialized"] is True
    client.store.active = None  # type: ignore[attr-defined]
    assert (await client.get("/api/v1/drive/status")).json()["initialized"] is True


async def test_select_profile_rejects_broken_vehicle_config_without_switching(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from src.infra.vehicle_store import VehicleStore  # noqa: PLC0415
    from src.robot import config_store  # noqa: PLC0415

    vdir = tmp_path / "vehicles"
    for vid in ("good", "bad", "garbled"):
        (vdir / vid).mkdir(parents=True)
        (vdir / vid / "vehicle.yaml").write_text("vehicle: {}\n", encoding="utf-8")
    (vdir / "garbled" / "vehicle.yaml").write_text("vehicle: [unclosed\n", encoding="utf-8")
    common = tmp_path / "common.yaml"
    common.write_text("pid: {}\n", encoding="utf-8")
    store = VehicleStore(vehicles_dir=vdir, common_path=common)
    store.set_active("good")
    real_load = config_store.load_vehicle_config

    def fake_load(vehicle_id: str, **kw: Any) -> Any:
        if vehicle_id == "bad":
            raise config_store.VehicleConfigError("max_speed_kmh が範囲外です")
        if vehicle_id == "good":
            return object(), Path("merged.yaml")
        return real_load(vehicle_id, **kw)

    monkeypatch.setattr(config_store, "load_vehicle_config", fake_load)
    saved = {k: getattr(app.state, k, None) for k in ("job_manager", "vehicle_store")}
    app.state.job_manager, app.state.vehicle_store = _Manager(), store
    try:
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
            for vid in ("bad", "garbled"):
                r = await c.post("/api/v1/drive/select-profile", json={"profile_id": vid})
                assert r.status_code == 422 and "設定が読めません" in r.json()["detail"]
                assert store.get_active() == "good"  # 切り替えない
            assert "max_speed_kmh が範囲外" in (await c.post("/api/v1/drive/select-profile", json={"profile_id": "bad"})).json()["detail"]
            assert (await c.post("/api/v1/drive/select-profile", json={"profile_id": "good"})).status_code == 200
            assert (await c.post("/api/v1/drive/select-profile", json={"profile_id": "nope"})).status_code == 404
    finally:
        for k, v in saved.items():
            setattr(app.state, k, v)


async def test_session_response_has_schedule_name_in_list_and_detail(client: AsyncClient) -> None:
    listed = (await client.get("/api/v1/sessions/")).json()
    assert listed[0]["schedule_id"] == SCHEDULE_ID and listed[0]["schedule_name"] == "メンテナンスモード_v2"
    assert listed[0]["status"] == "stopped"
    detail = (await client.get(f"/api/v1/sessions/{SESSION_ID}")).json()
    assert detail["schedule_id"] == SCHEDULE_ID and detail["schedule_name"] == "メンテナンスモード_v2"


async def test_session_response_schedule_fields_default_none(client: AsyncClient) -> None:
    app.state.session_repo = _SessionRepo([_session()])
    detail = (await client.get(f"/api/v1/sessions/{SESSION_ID}")).json()
    assert detail["schedule_id"] is None and detail["schedule_name"] is None


async def test_session_response_has_sequence_fields(client: AsyncClient) -> None:
    rid = "00000000-0000-0000-0000-0000000000e1"
    app.state.session_repo = _SessionRepo([_session(sequence_run_id=rid, sequence_name="夜間")])
    listed = (await client.get("/api/v1/sessions/")).json()
    assert listed[0]["sequence_run_id"] == rid and listed[0]["sequence_name"] == "夜間"
    app.state.session_repo = _SessionRepo([_session()])
    assert (await client.get("/api/v1/sessions/")).json()[0]["sequence_run_id"] is None
