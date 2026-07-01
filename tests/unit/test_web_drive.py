"""Web レイヤー 走行制御エンドポイントのユニットテスト。"""

from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock

import pytest
from httpx import ASGITransport, AsyncClient

from src.app.robot_controller import InvalidStateTransition, PreCheckFailed, RobotController
from src.models.drive_log import DriveSession
from src.models.system_state import RobotState, SystemState
from src.web.app import app


def _make_stub_controller(state: RobotState = RobotState.READY) -> MagicMock:
    ctrl = MagicMock(spec=RobotController)
    ctrl.get_system_state.return_value = SystemState(
        robot_state=state,
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

    stub_session = DriveSession(
        id="session-001",
        profile_id="",
        mode_id="mode-001",
        run_type="auto",
        started_at=datetime.now(tz=UTC),
        ended_at=None,
        status="running",
    )
    ctrl.start_auto_drive = AsyncMock(return_value=stub_session)

    stub_manual_session = DriveSession(
        id="session-002",
        profile_id="",
        mode_id=None,
        run_type="manual",
        started_at=datetime.now(tz=UTC),
        ended_at=None,
        status="running",
    )
    ctrl.start_manual = AsyncMock(return_value=stub_manual_session)
    return ctrl


@pytest.fixture
def stub_controller() -> MagicMock:
    ctrl = _make_stub_controller()
    from src.app.stubs import (  # noqa: PLC0415
        InMemoryModeRepository,
        InMemoryProfileRepository,
        InMemorySessionRepository,
    )

    app.state.profile_repo = InMemoryProfileRepository()
    app.state.mode_repo = InMemoryModeRepository()
    app.state.session_repo = InMemorySessionRepository()
    app.state.db_pool = None  # DB なし → ログ書き込みは無効（log_writer=None）
    return ctrl


async def _seed_mode(mode_id: str = "mode-001") -> None:
    """走行開始テスト用にモードをリポジトリへ登録する（未登録だと 404 になる）。"""
    from src.models.driving_mode import DrivingMode, SpeedPoint  # noqa: PLC0415

    mode = DrivingMode(
        id=mode_id,
        name="テストモード",
        description="",
        reference_speed=[SpeedPoint(time_s=0.0, speed_kmh=0.0)],
        total_duration=10.0,
        max_speed=0.0,
        created_at=datetime.now(tz=UTC),
    )
    await app.state.mode_repo.create(mode)


@pytest.mark.asyncio
async def test_get_status(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.get("/api/v1/drive/status")
    assert res.status_code == 200
    data = res.json()
    assert data["robot_state"] == "READY"
    assert data["active_profile_id"] is None


@pytest.mark.asyncio
async def test_initialize_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/initialize")
    assert res.status_code == 200
    stub_controller.initialize.assert_awaited_once()


@pytest.mark.asyncio
async def test_initialize_invalid_state(stub_controller: MagicMock) -> None:
    stub_controller.initialize.side_effect = InvalidStateTransition("不正な遷移")
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/initialize")
    assert res.status_code == 409
    assert "不正な遷移" in res.json()["detail"]


@pytest.mark.asyncio
async def test_start_drive_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    await _seed_mode()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/start", json={"mode_id": "mode-001"})
    assert res.status_code == 200
    data = res.json()
    assert data["id"] == "session-001"
    assert data["run_type"] == "auto"
    # mode/profile/log_writer を解決して渡す（DB なしのテストでは log_writer=None）
    stub_controller.start_auto_drive.assert_awaited_once()
    call = stub_controller.start_auto_drive.await_args
    assert call.args[0] == "mode-001"
    assert call.kwargs["mode"] is not None
    assert call.kwargs["mode"].id == "mode-001"
    assert call.kwargs["log_writer"] is None


@pytest.mark.asyncio
async def test_start_drive_unknown_mode_returns_404(stub_controller: MagicMock) -> None:
    """未登録モードでの走行開始は状態遷移前に 404 で拒否される。"""
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/start", json={"mode_id": "unknown-mode"})
    assert res.status_code == 404
    stub_controller.start_auto_drive.assert_not_awaited()


@pytest.mark.asyncio
async def test_start_drive_invalid_state(stub_controller: MagicMock) -> None:
    stub_controller.start_auto_drive.side_effect = InvalidStateTransition("READY 以外不可")
    app.state.controller = stub_controller
    await _seed_mode()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/start", json={"mode_id": "mode-001"})
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_auto_arm_ok(stub_controller: MagicMock) -> None:
    """自動走行 arm: 200 で status='armed'、arm_auto_drive が呼ばれること。"""
    stub_controller.arm_auto_drive = AsyncMock()
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/arm")
    assert res.status_code == 200
    assert res.json()["status"] == "armed"
    stub_controller.arm_auto_drive.assert_awaited_once()


@pytest.mark.asyncio
async def test_auto_arm_precheck_failed_returns_422(stub_controller: MagicMock) -> None:
    stub_controller.arm_auto_drive = AsyncMock(
        side_effect=PreCheckFailed("車速が 0 ではありません")
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/arm")
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_auto_arm_invalid_state_returns_409(stub_controller: MagicMock) -> None:
    stub_controller.arm_auto_drive = AsyncMock(side_effect=InvalidStateTransition("READY 以外不可"))
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/arm")
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_auto_cancel_ok(stub_controller: MagicMock) -> None:
    """自動走行 cancel: 200 で status='cancelled'、cancel_auto_drive が呼ばれること。"""
    stub_controller.cancel_auto_drive = AsyncMock()
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/cancel")
    assert res.status_code == 200
    assert res.json()["status"] == "cancelled"
    stub_controller.cancel_auto_drive.assert_awaited_once()


@pytest.mark.asyncio
async def test_auto_cancel_invalid_state_returns_409(stub_controller: MagicMock) -> None:
    stub_controller.cancel_auto_drive = AsyncMock(
        side_effect=InvalidStateTransition("arm 後のみ可")
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/cancel")
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_stop_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/stop")
    assert res.status_code == 200
    stub_controller.stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_stop_invalid_state(stub_controller: MagicMock) -> None:
    stub_controller.stop.side_effect = InvalidStateTransition("RUNNING 以外不可")
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/stop")
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_emergency_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/emergency")
    assert res.status_code == 200
    stub_controller.emergency_stop.assert_awaited_once()


@pytest.mark.asyncio
async def test_manual_start_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/manual/start")
    assert res.status_code == 200
    data = res.json()
    assert data["run_type"] == "manual"
    stub_controller.start_manual.assert_awaited_once()


@pytest.mark.asyncio
async def test_manual_stop_ok(stub_controller: MagicMock) -> None:
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/manual/stop")
    assert res.status_code == 200
    stub_controller.stop_manual.assert_awaited_once()


@pytest.mark.asyncio
async def test_start_drive_precheck_failed_returns_422(stub_controller: MagicMock) -> None:
    stub_controller.start_auto_drive.side_effect = PreCheckFailed("キャリブレーション未完了")
    app.state.controller = stub_controller
    await _seed_mode()
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/start", json={"mode_id": "mode-001"})
    assert res.status_code == 422
    assert "キャリブレーション未完了" in res.json()["detail"]


@pytest.mark.asyncio
async def test_manual_start_precheck_failed_returns_422(stub_controller: MagicMock) -> None:
    stub_controller.start_manual.side_effect = PreCheckFailed("サーボ未ON")
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/manual/start")
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_calibrate_ok(stub_controller: MagicMock) -> None:
    from datetime import UTC, datetime

    from src.models.calibration import CalibrationData, CalibrationResult

    calib_data = CalibrationData(
        accel_zero_pos=100,
        accel_full_pos=5100,
        accel_stroke=5000,
        brake_zero_pos=200,
        brake_full_pos=5200,
        brake_stroke=5000,
        calibrated_at=datetime(2026, 4, 22, 0, 0, 0, tzinfo=UTC),
        is_valid=True,
    )
    stub_controller.run_calibration = AsyncMock(
        return_value=CalibrationResult(success=True, data=calib_data, error_message=None)
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/calibrate")
    assert res.status_code == 200
    body = res.json()
    assert body["success"] is True
    assert body["error_message"] is None
    assert body["data"]["accel_stroke"] == 5000
    assert body["data"]["is_valid"] is True


@pytest.mark.asyncio
async def test_calibrate_returns_failure_result(stub_controller: MagicMock) -> None:
    from src.models.calibration import CalibrationResult

    stub_controller.run_calibration = AsyncMock(
        return_value=CalibrationResult(success=False, data=None, error_message="スパイク未検出")
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/calibrate")
    assert res.status_code == 200
    body = res.json()
    assert body["success"] is False
    assert body["error_message"] == "スパイク未検出"
    assert body["data"] is None


@pytest.mark.asyncio
async def test_calibrate_invalid_state_returns_409(stub_controller: MagicMock) -> None:
    stub_controller.run_calibration = AsyncMock(
        side_effect=InvalidStateTransition("READY 以外でキャリブレーション不可")
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/calibrate")
    assert res.status_code == 409
    assert "READY 以外でキャリブレーション不可" in res.json()["detail"]


@pytest.mark.asyncio
async def test_select_profile_ok(stub_controller: MagicMock) -> None:
    """プロファイルが存在する場合に select_profile が呼ばれて 200 が返ること。"""
    from datetime import datetime  # noqa: PLC0415
    from uuid import uuid4  # noqa: PLC0415

    from src.models.profile import PIDGains, StopConfig, VehicleProfile  # noqa: PLC0415

    profile_id = str(uuid4())
    profile = VehicleProfile(
        id=profile_id,
        name="TestCar",
        max_accel_opening=80.0,
        max_brake_opening=60.0,
        max_speed=180.0,
        max_decel_g=0.5,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None,
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )
    await app.state.profile_repo.create(profile)

    stub_controller.select_profile = MagicMock()
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/select-profile", json={"profile_id": profile_id})
    assert res.status_code == 200
    assert res.json()["profile_id"] == profile_id
    stub_controller.select_profile.assert_called_once()


@pytest.mark.asyncio
async def test_select_profile_not_found_returns_404(stub_controller: MagicMock) -> None:
    """存在しない profile_id に対して 404 が返ること。"""
    from uuid import uuid4  # noqa: PLC0415

    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/select-profile", json={"profile_id": str(uuid4())})
    assert res.status_code == 404


@pytest.mark.asyncio
async def test_learning_start_ok(stub_controller: MagicMock) -> None:
    """学習走行開始: 200 で DriveSession(run_type='learning') が返ること。"""
    stub_learning_session = DriveSession(
        id="session-learning-001",
        profile_id="",
        mode_id=None,
        run_type="learning",
        started_at=datetime.now(tz=UTC),
        ended_at=None,
        status="running",
    )
    stub_controller.start_learning_drive = AsyncMock(return_value=stub_learning_session)
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/start")
    assert res.status_code == 200
    data = res.json()
    assert data["id"] == "session-learning-001"
    assert data["run_type"] == "learning"
    assert data["mode_id"] is None
    stub_controller.start_learning_drive.assert_awaited_once()
    # DB なし環境では log_writer=None で呼ばれる
    assert stub_controller.start_learning_drive.await_args.kwargs["log_writer"] is None


@pytest.mark.asyncio
async def test_learning_arm_ok(stub_controller: MagicMock) -> None:
    """学習運転 arm: 200 で status='armed'、arm_learning_drive が呼ばれること。"""
    stub_controller.arm_learning_drive = AsyncMock()
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/arm")
    assert res.status_code == 200
    assert res.json()["status"] == "armed"
    stub_controller.arm_learning_drive.assert_awaited_once()


@pytest.mark.asyncio
async def test_learning_arm_precheck_failed_returns_422(stub_controller: MagicMock) -> None:
    stub_controller.arm_learning_drive = AsyncMock(
        side_effect=PreCheckFailed("車速が 0 ではありません")
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/arm")
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_learning_arm_invalid_state_returns_409(stub_controller: MagicMock) -> None:
    stub_controller.arm_learning_drive = AsyncMock(
        side_effect=InvalidStateTransition("READY 以外不可")
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/arm")
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_learning_cancel_ok(stub_controller: MagicMock) -> None:
    """学習運転 cancel: 200 で status='cancelled'、cancel_learning_drive が呼ばれること。"""
    stub_controller.cancel_learning_drive = AsyncMock()
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/cancel")
    assert res.status_code == 200
    assert res.json()["status"] == "cancelled"
    stub_controller.cancel_learning_drive.assert_awaited_once()


@pytest.mark.asyncio
async def test_learning_cancel_invalid_state_returns_409(stub_controller: MagicMock) -> None:
    stub_controller.cancel_learning_drive = AsyncMock(
        side_effect=InvalidStateTransition("arm 後のみ可")
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/cancel")
    assert res.status_code == 409


def test_get_log_writer_returns_none_without_pool() -> None:
    """db_pool が None なら get_log_writer は None を返す。"""
    from types import SimpleNamespace
    from unittest.mock import MagicMock

    from src.web.deps import get_log_writer

    req = MagicMock()
    req.app.state = SimpleNamespace(db_pool=None)
    assert get_log_writer(req) is None


def test_get_log_writer_returns_log_writer_with_pool() -> None:
    """db_pool があれば get_log_writer は LogWriter を返す。"""
    from types import SimpleNamespace
    from unittest.mock import MagicMock

    from src.infra.log_writer import LogWriter
    from src.web.deps import get_log_writer

    req = MagicMock()
    req.app.state = SimpleNamespace(db_pool=MagicMock())
    assert isinstance(get_log_writer(req), LogWriter)


async def _create_profile(profile_id: str) -> None:
    from src.models.profile import PIDGains, StopConfig, VehicleProfile  # noqa: PLC0415

    await app.state.profile_repo.create(
        VehicleProfile(
            id=profile_id,
            name="TrainCar",
            max_accel_opening=80.0,
            max_brake_opening=60.0,
            max_speed=120.0,
            max_decel_g=0.4,
            pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
            stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
            calibration=None,
            model_path=None,
            created_at=datetime.now(tz=UTC),
            updated_at=datetime.now(tz=UTC),
        )
    )


@pytest.mark.asyncio
async def test_learning_train_profile_not_found_returns_404(stub_controller: MagicMock) -> None:
    from uuid import uuid4  # noqa: PLC0415

    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/train", json={"profile_id": str(uuid4())})
    assert res.status_code == 404


@pytest.mark.asyncio
async def test_learning_train_insufficient_logs_returns_422(stub_controller: MagicMock) -> None:
    """ログが空（InMemory）の場合、LearningDataError → 422。"""
    from uuid import uuid4  # noqa: PLC0415

    profile_id = str(uuid4())
    await _create_profile(profile_id)
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/train", json={"profile_id": profile_id})
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_learning_train_ok_updates_model_path(
    stub_controller: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """学習成功時に 200 でメトリクスを返し、プロファイルの model_path が更新されること。"""
    from uuid import uuid4  # noqa: PLC0415

    profile_id = str(uuid4())
    await _create_profile(profile_id)

    fake_metrics = {"accel": {"mae": 0.1, "n": 50.0}, "brake": {"mae": 0.2, "n": 40.0}}

    def _fake_train(logs, profile, output_dir="data/models"):  # noqa: ANN001, ANN202, ARG001
        return "data/models/fake_model.pkl", fake_metrics

    monkeypatch.setattr("src.web.routers.drive.train_inverse_model", _fake_train)
    # train は最新の学習走行セッションを既定対象にする（空ログでも fake_train が成功を返す）
    monkeypatch.setattr(
        app.state.session_repo, "latest_learning_session_id", AsyncMock(return_value="sess-1")
    )

    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/train", json={"profile_id": profile_id})
    assert res.status_code == 200
    body = res.json()
    assert body["model_path"] == "data/models/fake_model.pkl"
    assert body["metrics"]["accel"]["mae"] == 0.1
    # フィードフォワード定数がレスポンスに含まれる（空ログ→デフォルト保持）
    assert body["feedforward_params"]["creep_speed_kmh"] == 7.0

    updated = await app.state.profile_repo.get_by_id(profile_id)
    assert updated.model_path == "data/models/fake_model.pkl"
    assert updated.feedforward_params is not None


@pytest.mark.asyncio
async def test_learning_train_refreshes_active_profile(
    stub_controller: MagicMock, monkeypatch: pytest.MonkeyPatch
) -> None:
    """学習成功時にコントローラの in-memory プロファイルへ即時反映される（レビュー #10）。

    DB のみ更新すると、学習直後の走行が旧モデル（または FF なし）で実行される。
    """
    from uuid import uuid4  # noqa: PLC0415

    profile_id = str(uuid4())
    await _create_profile(profile_id)

    def _fake_train(logs, profile, output_dir="data/models"):  # noqa: ANN001, ANN202, ARG001
        return "data/models/fake_model.pkl", {"accel": {"n": 1.0}, "brake": {"n": 1.0}}

    monkeypatch.setattr("src.web.routers.drive.train_inverse_model", _fake_train)
    monkeypatch.setattr(
        app.state.session_repo, "latest_learning_session_id", AsyncMock(return_value="sess-1")
    )

    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/train", json={"profile_id": profile_id})
    assert res.status_code == 200

    stub_controller.refresh_active_profile.assert_called_once()
    refreshed = stub_controller.refresh_active_profile.call_args[0][0]
    assert refreshed.id == profile_id
    assert refreshed.model_path == "data/models/fake_model.pkl"


@pytest.mark.asyncio
async def test_learning_train_no_learning_session_returns_422(
    stub_controller: MagicMock,
) -> None:
    """学習走行ログが無い（最新学習セッションが None）と 422 を返すこと。"""
    from uuid import uuid4  # noqa: PLC0415

    profile_id = str(uuid4())
    await _create_profile(profile_id)
    # InMemorySessionRepository.latest_learning_session_id は None を返す
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/train", json={"profile_id": profile_id})
    assert res.status_code == 422


@pytest.mark.asyncio
async def test_learning_start_invalid_state_returns_409(stub_controller: MagicMock) -> None:
    """InvalidStateTransition 時に 409 が返ること。"""
    stub_controller.start_learning_drive = AsyncMock(
        side_effect=InvalidStateTransition("READY 以外不可")
    )
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/start")
    assert res.status_code == 409


@pytest.mark.asyncio
async def test_learning_start_precheck_failed_returns_422(stub_controller: MagicMock) -> None:
    """PreCheckFailed 時に 422 が返ること。"""
    stub_controller.start_learning_drive = AsyncMock(side_effect=PreCheckFailed("サーボ未ON"))
    app.state.controller = stub_controller
    async with AsyncClient(transport=ASGITransport(app=app), base_url="http://test") as c:
        res = await c.post("/api/v1/drive/learning/start")
    assert res.status_code == 422
