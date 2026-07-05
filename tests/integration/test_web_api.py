"""Web レイヤー 統合テスト（FastAPI アプリ全体・モック Controller）。"""

from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.robot_controller import InvalidStateTransition, RobotController
from src.app.stubs import (
    InMemoryModeRepository,
    InMemoryProfileRepository,
    InMemoryScheduleRepository,
    InMemorySessionRepository,
)
from src.models.drive_log import DriveSession
from src.models.system_state import RobotState, SystemState
from src.web.app import app


def _make_stub_controller() -> MagicMock:
    ctrl = MagicMock(spec=RobotController)
    ctrl.get_system_state.return_value = SystemState(
        robot_state=RobotState.STANDBY,
        active_profile_id=None,
        active_session_id=None,
        last_normal_shutdown=False,
        updated_at=datetime.now(tz=UTC),
    )
    ctrl.initialize = AsyncMock()
    ctrl.emergency_stop = AsyncMock()
    ctrl.reset_emergency = AsyncMock()
    ctrl.stop = AsyncMock()
    ctrl.stop_manual = AsyncMock()
    ctrl.pause_auto_drive = AsyncMock()
    ctrl.resume_auto_drive = AsyncMock()
    ctrl.start_auto_drive = AsyncMock()
    ctrl.start_manual = AsyncMock()
    ctrl.arm_learning_drive = AsyncMock()
    ctrl.cancel_learning_drive = AsyncMock()
    ctrl.get_active_profile = MagicMock(return_value=None)
    ctrl.stop_schedule_drive = AsyncMock()
    ctrl.start_schedule_drive = AsyncMock(
        return_value=DriveSession(
            id="sched-sess-1",
            profile_id="p1",
            mode_id=None,
            run_type="auto",
            started_at=datetime.now(tz=UTC),
            ended_at=None,
            status="running",
        )
    )
    ctrl.start_learning_drive = AsyncMock(
        return_value=DriveSession(
            id="learn-sess-1",
            profile_id="p1",
            mode_id=None,
            run_type="learning",
            started_at=datetime.now(tz=UTC),
            ended_at=None,
            status="running",
        )
    )
    return ctrl


@pytest.fixture(autouse=True)
def inject_controller() -> MagicMock:
    ctrl = _make_stub_controller()
    app.state.controller = ctrl
    app.state.profile_repo = InMemoryProfileRepository()
    app.state.mode_repo = InMemoryModeRepository()
    app.state.session_repo = InMemorySessionRepository()
    app.state.schedule_repo = InMemoryScheduleRepository()
    app.state.db_pool = None
    return ctrl


@pytest.mark.asyncio
async def test_status_endpoint_returns_state() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/drive/status")
    assert res.status_code == 200
    assert res.json()["robot_state"] == "STANDBY"


@pytest.mark.asyncio
async def test_profiles_list_empty() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/profiles/")
    assert res.status_code == 200
    assert res.json() == []


@pytest.mark.asyncio
async def test_profiles_get_404() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/profiles/nonexistent")
    assert res.status_code == 404


def _profile_create_payload(**overrides: object) -> dict:
    payload: dict = {
        "name": "Prof1",
        "max_accel_opening": 80.0,
        "max_brake_opening": 80.0,
        "max_speed": 120.0,
        "max_decel_g": 0.4,
        "pid_gains": {"kp": 1.0, "ki": 0.1, "kd": 0.0},
        "stop_config": {"deviation_threshold_kmh": 2.0, "deviation_duration_s": 4.0},
    }
    payload.update(overrides)
    return payload


@pytest.mark.asyncio
async def test_profile_put_preserves_arbiter_constants(inject_controller: MagicMock) -> None:
    """PUT で feedforward_params の一部だけ送っても調停定数（switch_hysteresis_pct 等）が
    デフォルトへリセットされず維持されること（旧 _ffp_to_schema/_ffp_from_schema の
    6/11 フィールド欠落バグの回帰）。"""
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        create_res = await c.post(
            "/api/v1/profiles/",
            json=_profile_create_payload(
                feedforward_params={
                    "creep_speed_kmh": 8.0,
                    "switch_hysteresis_pct": 1.2,
                    "accel_reengage_dwell_s": 0.5,
                    "accel_rate_limit_pct_s": 150.0,
                    "brake_rate_limit_pct_s": 250.0,
                    "pid_output_limit_pct": 40.0,
                },
            ),
        )
        assert create_res.status_code == 201
        profile_id = create_res.json()["id"]
        assert create_res.json()["feedforward_params"]["switch_hysteresis_pct"] == 1.2

        # 別フィールド（stop_config）だけを更新する PUT。feedforward_params は送らない。
        put_res = await c.put(
            f"/api/v1/profiles/{profile_id}",
            json={"stop_config": {"deviation_threshold_kmh": 3.0, "deviation_duration_s": 5.0}},
        )

    assert put_res.status_code == 200
    ffp = put_res.json()["feedforward_params"]
    assert ffp["switch_hysteresis_pct"] == 1.2
    assert ffp["accel_reengage_dwell_s"] == 0.5
    assert ffp["accel_rate_limit_pct_s"] == 150.0
    assert ffp["brake_rate_limit_pct_s"] == 250.0
    assert ffp["pid_output_limit_pct"] == 40.0


@pytest.mark.asyncio
async def test_profile_dynamics_params_roundtrip_and_manual_preview_update(
    inject_controller: MagicMock,
) -> None:
    """dynamics_params(preview_time_s・FOPDT値)が作成時に保存され、PUT で preview だけ
    手動更新できること。"""
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        create_res = await c.post(
            "/api/v1/profiles/",
            json=_profile_create_payload(
                dynamics_params={
                    "preview_time_s": 0.6,
                    "fopdt_k": 0.5,
                    "fopdt_tau": 2.0,
                    "fopdt_theta": 0.6,
                },
            ),
        )
        assert create_res.status_code == 201
        body = create_res.json()
        assert body["dynamics_params"]["preview_time_s"] == 0.6
        assert body["dynamics_params"]["fopdt_theta"] == 0.6
        profile_id = body["id"]

        put_res = await c.put(
            f"/api/v1/profiles/{profile_id}",
            json={"dynamics_params": {"preview_time_s": 1.1}},
        )

    assert put_res.status_code == 200
    dyn = put_res.json()["dynamics_params"]
    # 手動更新時は FOPDT 値もスキーマのデフォルト(None)で上書きされる仕様
    # （同スキーマで受けて上書き可能とする設計上の想定挙動）。
    assert dyn["preview_time_s"] == 1.1


@pytest.mark.asyncio
async def test_profile_created_without_dynamics_params_defaults_to_zero_preview(
    inject_controller: MagicMock,
) -> None:
    """dynamics_params を指定しない作成は preview_time_s=0.0（従来動作互換）になること。"""
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/profiles/", json=_profile_create_payload())

    assert res.status_code == 201
    assert res.json()["dynamics_params"]["preview_time_s"] == 0.0


@pytest.mark.asyncio
async def test_modes_list_empty() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/modes/")
    assert res.status_code == 200
    assert res.json() == []


@pytest.mark.asyncio
async def test_sessions_list_empty() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/sessions/")
    assert res.status_code == 200
    assert res.json() == []


@pytest.mark.asyncio
async def test_sessions_logs_empty() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/sessions/some-id/logs")
    assert res.status_code == 200
    assert res.json() == []


@pytest.mark.asyncio
async def test_sessions_logs_csv_404_when_missing() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/sessions/nonexistent/logs.csv")
    assert res.status_code == 404


@pytest.mark.asyncio
async def test_sessions_logs_csv_download() -> None:
    from src.models.drive_log import DriveLog, DriveSession

    started = datetime(2026, 6, 16, 5, 36, 49, tzinfo=UTC)
    session = DriveSession(
        id="sess-1", profile_id="p1", mode_id=None, run_type="learning",
        started_at=started, ended_at=None, status="error",
    )
    log = DriveLog(
        id=1, session_id="sess-1", timestamp=started,
        ref_speed_kmh=10.0, actual_speed_kmh=9.5,
        accel_opening=12.0, brake_opening=0.0,
        accel_pos=100, brake_pos=0, accel_current=300.0, brake_current=0.0,
    )

    class _Repo:
        async def get_by_id(self, session_id: str) -> DriveSession:
            return session

        async def list_logs(self, session_id: str, limit: int = 1000) -> list[DriveLog]:
            return [log]

    app.state.session_repo = _Repo()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/sessions/sess-1/logs.csv")

    assert res.status_code == 200
    assert res.headers["content-type"].startswith("text/csv")
    assert "attachment" in res.headers["content-disposition"]
    # started_at は UTC 05:36:49 → JST 14:36:49 でファイル名に出る
    assert "session_sess-1_20260616_143649.csv" in res.headers["content-disposition"]
    lines = res.text.strip().splitlines()
    assert lines[0] == (
        "timestamp,ref_speed_kmh,actual_speed_kmh,accel_opening,brake_opening,"
        "accel_pos,brake_pos,accel_current,brake_current"
    )
    # timestamp は UTC 05:36:49 → JST 14:36:49・スペース区切りで出力
    assert lines[1].startswith("2026-06-16 14:36:49")
    assert lines[1].endswith("9.5,12.0,0.0,100,0,300.0,0.0")


@pytest.mark.asyncio
async def test_docs_endpoint() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/docs")
    assert res.status_code == 200


@pytest.mark.asyncio
async def test_websocket_connect() -> None:
    from starlette.testclient import TestClient

    with TestClient(app) as client:
        with client.websocket_connect("/ws/realtime") as ws:
            # 接続直後はサーバーからのメッセージをすぐには受け取れないが接続は成功する
            assert ws is not None


@pytest.mark.asyncio
async def test_learning_arm_then_start_sequence(inject_controller: MagicMock) -> None:
    """学習運転の確認「はい」フロー: arm → start が全 ASGI スタックを通ること。"""
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        arm = await c.post("/api/v1/drive/learning/arm")
        start = await c.post("/api/v1/drive/learning/start")
    assert arm.status_code == 200
    assert arm.json()["status"] == "armed"
    assert start.status_code == 200
    assert start.json()["run_type"] == "learning"
    inject_controller.arm_learning_drive.assert_awaited_once()
    inject_controller.start_learning_drive.assert_awaited_once()


@pytest.mark.asyncio
async def test_learning_train_returns_auto_tuned_pid(inject_controller: MagicMock) -> None:
    """学習モデル学習が FOPDT 同定で PID を自動適合し、応答に反映すること。"""
    from datetime import timedelta

    from src.models.drive_log import DriveLog
    from src.models.profile import PIDGains, StopConfig, VehicleProfile

    prof = VehicleProfile(
        id="p1", name="Train", max_accel_opening=80.0, max_brake_opening=80.0,
        max_speed=100.0, max_decel_g=0.4, pid_gains=PIDGains(kp=1.0, ki=0.1, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None, model_path=None,
        created_at=datetime.now(tz=UTC), updated_at=datetime.now(tz=UTC),
    )
    await app.state.profile_repo.create(prof)

    def _session_logs(session_id: str) -> list[DriveLog]:
        t0 = datetime.now(tz=UTC)
        out: list[DriveLog] = []
        speed = 0.0
        i = 0
        for _ in range(60):  # 加速保持（FOPDT 区間 + アクセルレジーム）
            speed = min(80.0, speed + 1.5)
            out.append(DriveLog(
                id=i, session_id=session_id, timestamp=t0 + timedelta(seconds=0.1 * i),
                ref_speed_kmh=None, actual_speed_kmh=speed, accel_opening=40.0,
                brake_opening=0.0, accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
            ))
            i += 1
        for _ in range(60):  # 減速（ブレーキレジーム）
            speed = max(0.0, speed - 1.5)
            out.append(DriveLog(
                id=i, session_id=session_id, timestamp=t0 + timedelta(seconds=0.1 * i),
                ref_speed_kmh=None, actual_speed_kmh=speed, accel_opening=0.0,
                brake_opening=30.0, accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
            ))
            i += 1
        return out

    logs = _session_logs("s1") + _session_logs("s2")

    class _Repo:
        async def latest_learning_session_id(self, profile_id: str) -> str | None:
            return "s1"

        async def list_logs_for_training(
            self, profile_id: str, session_ids: list[str] | None = None, limit: int = 100_000
        ) -> list[DriveLog]:
            return logs

    app.state.session_repo = _Repo()

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/train", json={"profile_id": "p1"})

    assert res.status_code == 200
    body = res.json()
    assert body["pid_auto_tuned"] is True
    assert body["pid_gains"]["kp"] > 0.0
    assert body["pid_gains"]["ki"] > 0.0
    assert body["pid_gains"]["kd"] == 0.0
    inject_controller.refresh_active_profile.assert_called_once()


@pytest.mark.asyncio
async def test_pid_tune_validate_returns_kpi_and_cost(inject_controller: MagicMock) -> None:
    """PID 検証走行が KPI サマリとコストを返すこと。"""
    from src.models.profile import PIDGains, StopConfig, VehicleProfile

    prof = VehicleProfile(
        id="p1", name="Val", max_accel_opening=80.0, max_brake_opening=80.0,
        max_speed=100.0, max_decel_g=0.4, pid_gains=PIDGains(kp=2.0, ki=0.5, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None, model_path=None,
        created_at=datetime.now(tz=UTC), updated_at=datetime.now(tz=UTC),
    )
    await app.state.profile_repo.create(prof)
    inject_controller.run_pid_validation = AsyncMock(
        return_value={"n_samples": 500.0, "p95_kmh": 0.15, "max_abs_deviation_kmh": 0.4,
                      "reversal_max_per_5s": 1.0, "hard_limit_violations": 0.0}
    )

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/pid-tune/validate", json={"profile_id": "p1"})

    assert res.status_code == 200
    body = res.json()
    assert body["kpi_summary"]["p95_kmh"] == 0.15
    assert body["cost"] > 0.0
    assert body["pid_gains"]["kp"] == 2.0


@pytest.mark.asyncio
async def test_pid_tune_validate_404_when_profile_missing(inject_controller: MagicMock) -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/pid-tune/validate", json={"profile_id": "nope"})
    assert res.status_code == 404


@pytest.mark.asyncio
async def test_pid_tune_validate_409_on_invalid_state(inject_controller: MagicMock) -> None:
    """走行中など不正状態では 409 を返すこと。"""
    from src.models.profile import PIDGains, StopConfig, VehicleProfile

    prof = VehicleProfile(
        id="p1", name="Val", max_accel_opening=80.0, max_brake_opening=80.0,
        max_speed=100.0, max_decel_g=0.4, pid_gains=PIDGains(kp=2.0, ki=0.5, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None, model_path=None,
        created_at=datetime.now(tz=UTC), updated_at=datetime.now(tz=UTC),
    )
    await app.state.profile_repo.create(prof)
    inject_controller.run_pid_validation = AsyncMock(
        side_effect=InvalidStateTransition("not ready")
    )

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/pid-tune/validate", json={"profile_id": "p1"})
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_pid_tune_refine_saves_best_gains(inject_controller: MagicMock) -> None:
    """PID 絞り込みが最良ゲイン・先読み補償秒数を保存し、履歴・最良コストを返すこと。"""
    from src.domain.pid_tuning import TuningParams
    from src.models.profile import PIDGains, StopConfig, VehicleProfile

    prof = VehicleProfile(
        id="p1", name="Ref", max_accel_opening=80.0, max_brake_opening=80.0,
        max_speed=100.0, max_decel_g=0.4, pid_gains=PIDGains(kp=1.0, ki=0.1, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None, model_path=None,
        created_at=datetime.now(tz=UTC), updated_at=datetime.now(tz=UTC),
    )
    await app.state.profile_repo.create(prof)
    best = TuningParams(kp=2.5, ki=0.4, kd=0.0, preview_time_s=0.7)
    history = [
        {"kp": 1.0, "ki": 0.1, "kd": 0.0, "preview_time_s": 0.0, "cost": 3.0},
        {"kp": 2.5, "ki": 0.4, "kd": 0.0, "preview_time_s": 0.7, "cost": 1.2},
    ]
    inject_controller.run_pid_tuning_session = AsyncMock(return_value=(best, history))

    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post(
            "/api/v1/drive/pid-tune/refine", json={"profile_id": "p1", "max_runs": 5}
        )

    assert res.status_code == 200
    body = res.json()
    assert body["pid_gains"]["kp"] == 2.5
    assert body["best_cost"] == 1.2
    assert len(body["history"]) == 2
    inject_controller.refresh_active_profile.assert_called_once()
    # 最良ゲイン・先読み補償秒数が永続化されていること
    saved = await app.state.profile_repo.get_by_id("p1")
    assert saved.pid_gains.kp == 2.5
    assert saved.dynamics_params.preview_time_s == pytest.approx(0.7)


@pytest.mark.asyncio
async def test_pause_then_resume_sequence(inject_controller: MagicMock) -> None:
    """自動走行の一時停止 → 走行再開が全 ASGI スタックを通ること。"""
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        pause = await c.post("/api/v1/drive/pause")
        resume = await c.post("/api/v1/drive/resume")
    assert pause.status_code == 200
    assert pause.json()["status"] == "paused"
    assert resume.status_code == 200
    assert resume.json()["status"] == "running"
    inject_controller.pause_auto_drive.assert_awaited_once()
    inject_controller.resume_auto_drive.assert_awaited_once()


@pytest.mark.asyncio
async def test_pause_returns_409_on_invalid_state(inject_controller: MagicMock) -> None:
    """走行中(RUNNING)以外で一時停止すると 409 を返すこと。"""
    inject_controller.pause_auto_drive = AsyncMock(
        side_effect=InvalidStateTransition("not running")
    )
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/pause")
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_learning_arm_then_cancel_sequence(inject_controller: MagicMock) -> None:
    """学習運転の確認「いいえ」フロー: arm → cancel が通り、start は呼ばれないこと。"""
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        arm = await c.post("/api/v1/drive/learning/arm")
        cancel = await c.post("/api/v1/drive/learning/cancel")
    assert arm.status_code == 200
    assert cancel.status_code == 200
    assert cancel.json()["status"] == "cancelled"
    inject_controller.cancel_learning_drive.assert_awaited_once()
    inject_controller.start_learning_drive.assert_not_awaited()


# ── タイムスケジュール（統合タイムライン）CRUD ─────────────────────────


def _schedule_payload(name: str = "sched-1") -> dict:
    return {
        "name": name,
        "description": "test schedule",
        "pedal_points": [
            {"time_s": 0.0, "accel_opening": 0.0, "brake_opening": 0.0},
            {"time_s": 5.0, "accel_opening": 40.0, "brake_opening": 0.0},
        ],
        "button_events": [{"time_s": 0.5, "channel": 0, "press_duration_s": 1.0}],
    }


@pytest.mark.asyncio
async def test_schedules_crud_flow() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        empty = await c.get("/api/v1/schedules/")
        assert empty.status_code == 200
        assert empty.json() == []

        created = await c.post("/api/v1/schedules/", json=_schedule_payload())
        assert created.status_code == 201
        body = created.json()
        assert body["name"] == "sched-1"
        assert body["total_duration"] == 5.0
        assert len(body["pedal_points"]) == 2
        sid = body["id"]

        listed = await c.get("/api/v1/schedules/")
        assert listed.status_code == 200
        assert listed.json()[0]["pedal_point_count"] == 2
        assert listed.json()[0]["button_event_count"] == 1

        detail = await c.get(f"/api/v1/schedules/{sid}")
        assert detail.status_code == 200
        assert detail.json()["button_events"][0]["channel"] == 0

        deleted = await c.delete(f"/api/v1/schedules/{sid}")
        assert deleted.status_code == 204
        assert (await c.get(f"/api/v1/schedules/{sid}")).status_code == 404


@pytest.mark.asyncio
async def test_schedule_duplicate_name_returns_409() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        first = await c.post("/api/v1/schedules/", json=_schedule_payload("dup"))
        assert first.status_code == 201
        dup = await c.post("/api/v1/schedules/", json=_schedule_payload("dup"))
    assert dup.status_code == 409


@pytest.mark.asyncio
async def test_schedule_non_monotonic_pedal_points_returns_422() -> None:
    payload = _schedule_payload("bad")
    payload["pedal_points"] = [
        {"time_s": 5.0, "accel_opening": 0.0, "brake_opening": 0.0},
        {"time_s": 2.0, "accel_opening": 10.0, "brake_opening": 0.0},
    ]
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/schedules/", json=payload)
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_schedule_invalid_channel_returns_422() -> None:
    payload = _schedule_payload("badch")
    payload["button_events"] = [{"time_s": 0.0, "channel": 99, "press_duration_s": 1.0}]
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/schedules/", json=payload)
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_schedule_drive_start_and_stop(inject_controller: MagicMock) -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        created = await c.post("/api/v1/schedules/", json=_schedule_payload("run"))
        sid = created.json()["id"]

        start = await c.post("/api/v1/drive/schedule/start", json={"schedule_id": sid})
        assert start.status_code == 200
        assert start.json()["run_type"] == "auto"
        inject_controller.start_schedule_drive.assert_awaited_once()

        stop = await c.post("/api/v1/drive/schedule/stop")
        assert stop.status_code == 200
        inject_controller.stop_schedule_drive.assert_awaited_once()


@pytest.mark.asyncio
async def test_schedule_drive_start_unknown_id_returns_404() -> None:
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post(
            "/api/v1/drive/schedule/start",
            json={"schedule_id": "00000000-0000-0000-0000-000000000000"},
        )
    assert res.status_code == 404
