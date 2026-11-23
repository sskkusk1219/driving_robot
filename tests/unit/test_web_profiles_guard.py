# ruff: noqa: E501
"""車の登録・削除の拒否（同じ名前・選択中の車の削除）。ProblemReport_20261123_src全動作チェック2 #2・#3。"""

from __future__ import annotations

from collections.abc import AsyncIterator
from pathlib import Path
from types import SimpleNamespace

import pytest
from httpx import ASGITransport, AsyncClient

from src.infra.vehicle_store import VehicleStore
from src.web.app import app

COMMON = Path(__file__).resolve().parents[2] / "config" / "robot_common.yaml"

pytestmark = pytest.mark.skipif(not COMMON.exists(), reason="config/robot_common.yaml が無い")


@pytest.fixture
async def client(tmp_path: Path) -> AsyncIterator[tuple[AsyncClient, VehicleStore]]:
    store = VehicleStore(tmp_path / "vehicles", COMMON)
    saved = {k: getattr(app.state, k, None) for k in ("job_manager", "vehicle_store")}
    app.state.job_manager = SimpleNamespace(current=None)
    app.state.vehicle_store = store
    try:
        async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
            yield c, store
    finally:
        for k, v in saved.items():
            setattr(app.state, k, v)


async def test_create_with_same_name_is_409(client: tuple[AsyncClient, VehicleStore]) -> None:
    c, store = client
    assert (await c.post("/api/v1/profiles/", json={"name": "テスト車"})).status_code == 201
    r = await c.post("/api/v1/profiles/", json={"name": " テスト車 "})
    assert r.status_code == 409 and r.json()["detail"] == "車の名前「テスト車」は既に使われています"
    assert len(store.list_ids()) == 1  # 2 台目は作られない


async def test_delete_active_vehicle_is_409(client: tuple[AsyncClient, VehicleStore]) -> None:
    c, store = client
    a = (await c.post("/api/v1/profiles/", json={"name": "車A"})).json()["id"]
    b = (await c.post("/api/v1/profiles/", json={"name": "車B"})).json()["id"]
    store.set_active(a)
    r = await c.delete(f"/api/v1/profiles/{a}")
    assert r.status_code == 409 and "選択中の車は削除できません" in r.json()["detail"]
    assert store.exists(a) and store.get_active() == a
    assert (await c.delete(f"/api/v1/profiles/{b}")).status_code == 204  # 選んでいない車は消せる
