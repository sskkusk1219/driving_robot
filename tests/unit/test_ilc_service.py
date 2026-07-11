"""ILCService（走行前ロード・走行後学習）のユニットテスト。"""

from datetime import UTC, datetime, timedelta
from unittest.mock import AsyncMock

import pytest

from src.app.ilc_service import ILCService
from src.domain.control.ilc import ILCController, ILCTable
from src.infra.ilc_repository import ILCRecord
from src.models.drive_log import DriveLog
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import (
    DynamicsParams,
    PIDGains,
    StopConfig,
    VehicleProfile,
)

PROFILE_ID = "11111111-1111-1111-1111-111111111111"
MODE_ID = "22222222-2222-2222-2222-222222222222"


def _make_profile(fopdt_k: float | None = 1.0, fopdt_theta: float | None = 0.3) -> VehicleProfile:
    return VehicleProfile(
        id=PROFILE_ID,
        name="p",
        max_accel_opening=80.0,
        max_brake_opening=80.0,
        max_speed=140.0,
        max_decel_g=0.4,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None,
        model_path=None,
        created_at=datetime(2026, 1, 1),
        updated_at=datetime(2026, 1, 1),
        dynamics_params=DynamicsParams(fopdt_k=fopdt_k, fopdt_theta=fopdt_theta),
    )


def _make_mode() -> DrivingMode:
    return DrivingMode(
        id=MODE_ID,
        name="m",
        description="",
        reference_speed=[SpeedPoint(time_s=0.0, speed_kmh=0.0)],
        total_duration=10.0,
        max_speed=140.0,
        created_at=datetime(2026, 1, 1),
    )


def _make_logs(n: int, ref: float = 100.0, actual: float = 99.0) -> list[DriveLog]:
    t0 = datetime(2026, 1, 1, tzinfo=UTC)
    return [
        DriveLog(
            id=i,
            session_id="s",
            timestamp=t0 + timedelta(seconds=i * 0.1),
            ref_speed_kmh=ref,
            actual_speed_kmh=actual,
            accel_opening=2.0,
            brake_opening=0.0,
            accel_pos=0,
            brake_pos=0,
            accel_current=0.0,
            brake_current=0.0,
        )
        for i in range(n)
    ]


def _make_service(repo: AsyncMock, logs: list[DriveLog]) -> ILCService:
    session_repo = AsyncMock()
    session_repo.list_logs = AsyncMock(return_value=logs)
    return ILCService(ilc_repo=repo, session_repo=session_repo)


class TestPrepare:
    @pytest.mark.asyncio
    async def test_none_when_no_record(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(return_value=None)
        svc = _make_service(repo, [])
        assert await svc.prepare(_make_profile(), _make_mode()) is None

    @pytest.mark.asyncio
    async def test_none_when_disabled(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(
            return_value=ILCRecord(
                PROFILE_ID, MODE_ID, enabled=False,
                table=ILCTable(efforts=[1.0, 2.0], dt_s=0.1, iteration=2),
            )
        )
        svc = _make_service(repo, [])
        assert await svc.prepare(_make_profile(), _make_mode()) is None

    @pytest.mark.asyncio
    async def test_none_when_empty_efforts(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(
            return_value=ILCRecord(PROFILE_ID, MODE_ID, enabled=True, table=ILCTable(efforts=[]))
        )
        svc = _make_service(repo, [])
        assert await svc.prepare(_make_profile(), _make_mode()) is None

    @pytest.mark.asyncio
    async def test_returns_controller_when_enabled(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(
            return_value=ILCRecord(
                PROFILE_ID, MODE_ID, enabled=True,
                table=ILCTable(efforts=[1.0, 2.0], dt_s=0.1, iteration=3),
            )
        )
        svc = _make_service(repo, [])
        ctrl = await svc.prepare(_make_profile(), _make_mode())
        assert isinstance(ctrl, ILCController)

    @pytest.mark.asyncio
    async def test_none_and_no_raise_on_repo_error(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(side_effect=RuntimeError("db down"))
        svc = _make_service(repo, [])
        assert await svc.prepare(_make_profile(), _make_mode()) is None


class TestLearnFromSession:
    @pytest.mark.asyncio
    async def test_upserts_new_iteration(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(return_value=None)  # 初回
        repo.upsert = AsyncMock()
        svc = _make_service(repo, _make_logs(100, ref=100.0, actual=99.0))
        await svc.learn_from_session("s", _make_profile(), _make_mode(), {"p95_kmh": 0.3})
        repo.upsert.assert_awaited_once()
        args = repo.upsert.await_args.args
        new_table = args[2]
        assert new_table.iteration == 1
        assert len(new_table.efforts) > 0
        # e = ref-actual = +1 なので正の補正が入る
        assert max(new_table.efforts) > 0.0

    @pytest.mark.asyncio
    async def test_skips_on_insufficient_logs(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(return_value=None)
        repo.upsert = AsyncMock()
        svc = _make_service(repo, _make_logs(10))  # < 50
        await svc.learn_from_session("s", _make_profile(), _make_mode(), {"p95_kmh": 0.3})
        repo.upsert.assert_not_awaited()

    @pytest.mark.asyncio
    async def test_skips_when_disabled(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(
            return_value=ILCRecord(
                PROFILE_ID, MODE_ID, enabled=False,
                table=ILCTable(efforts=[0.0] * 100, dt_s=0.1, iteration=1),
            )
        )
        repo.upsert = AsyncMock()
        svc = _make_service(repo, _make_logs(100))
        await svc.learn_from_session("s", _make_profile(), _make_mode(), {"p95_kmh": 0.3})
        repo.upsert.assert_not_awaited()

    @pytest.mark.asyncio
    async def test_skips_on_divergence(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(
            return_value=ILCRecord(
                PROFILE_ID, MODE_ID, enabled=True,
                table=ILCTable(efforts=[0.0] * 100, dt_s=0.1, iteration=2, best_p95_kmh=0.2),
            )
        )
        repo.upsert = AsyncMock()
        svc = _make_service(repo, _make_logs(100))
        # 今回 p95=0.5 > 0.2×1.2=0.24 → 発散
        await svc.learn_from_session("s", _make_profile(), _make_mode(), {"p95_kmh": 0.5})
        repo.upsert.assert_not_awaited()

    @pytest.mark.asyncio
    async def test_exception_does_not_propagate(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(side_effect=RuntimeError("boom"))
        svc = _make_service(repo, _make_logs(100))
        # 例外を投げず握りつぶす（fire-and-forget 前提）
        await svc.learn_from_session("s", _make_profile(), _make_mode(), {"p95_kmh": 0.3})

    @pytest.mark.asyncio
    async def test_appends_kpi_history(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(
            return_value=ILCRecord(
                PROFILE_ID, MODE_ID, enabled=True,
                table=ILCTable(efforts=[0.0] * 100, dt_s=0.1, iteration=1, best_p95_kmh=0.4),
                kpi_history=[{"iteration": 1, "p95_kmh": 0.4}],
            )
        )
        repo.upsert = AsyncMock()
        svc = _make_service(repo, _make_logs(100))
        await svc.learn_from_session(
            "s", _make_profile(), _make_mode(), {"p95_kmh": 0.35, "max_abs_deviation_kmh": 1.1}
        )
        history = repo.upsert.await_args.args[3]
        assert len(history) == 2
        assert history[-1]["iteration"] == 2
        assert history[-1]["p95_kmh"] == 0.35
