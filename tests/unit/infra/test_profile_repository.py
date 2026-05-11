"""ProfileRepository のユニットテスト。"""

from contextlib import asynccontextmanager
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import UUID, uuid4

import pytest

from src.infra.profile_repository import ProfileRepository
from src.models.calibration import CalibrationData
from src.models.profile import PIDGains, StopConfig, VehicleProfile

PROFILE_ID = str(uuid4())
PROFILE_UUID = UUID(PROFILE_ID)


def make_calibration_data(
    accel_zero: int = 1000,
    accel_full: int = 6000,
    brake_zero: int = 1000,
    brake_full: int = 6000,
) -> CalibrationData:
    return CalibrationData(
        accel_zero_pos=accel_zero,
        accel_full_pos=accel_full,
        accel_stroke=accel_full - accel_zero,
        brake_zero_pos=brake_zero,
        brake_full_pos=brake_full,
        brake_stroke=brake_full - brake_zero,
        calibrated_at=datetime.now(tz=UTC),
        is_valid=True,
    )


def make_mock_pool() -> tuple[MagicMock, AsyncMock]:
    """acquire() が AsyncMock の conn を返す Pool モック。"""
    conn = AsyncMock()
    pool = MagicMock()

    @asynccontextmanager
    async def _acquire():
        yield conn

    pool.acquire = _acquire
    return pool, conn


def make_mock_pool_with_rows(rows: list) -> tuple[MagicMock, AsyncMock]:
    """fetch/fetchrow が指定した rows を返す Pool モック。"""
    conn = AsyncMock()
    pool = MagicMock()

    @asynccontextmanager
    async def _acquire():
        yield conn

    pool.acquire = _acquire
    if len(rows) == 1:
        conn.fetchrow.return_value = rows[0]
    conn.fetch.return_value = rows
    return pool, conn


class TestSaveCalibration:
    @pytest.mark.asyncio
    async def test_save_calibration_executes_upsert(self) -> None:
        """save_calibration が conn.execute を1回呼ぶこと。"""
        pool, conn = make_mock_pool()
        repo = ProfileRepository(pool)
        data = make_calibration_data()

        await repo.save_calibration(PROFILE_ID, data)

        conn.execute.assert_awaited_once()

    @pytest.mark.asyncio
    async def test_save_calibration_passes_profile_id_as_uuid(self) -> None:
        """execute に渡される第2引数が UUID(profile_id) であること。"""
        pool, conn = make_mock_pool()
        repo = ProfileRepository(pool)
        data = make_calibration_data()

        await repo.save_calibration(PROFILE_ID, data)

        args = conn.execute.call_args[0]
        assert args[2] == PROFILE_UUID

    @pytest.mark.asyncio
    async def test_save_calibration_passes_calibration_fields(self) -> None:
        """execute に CalibrationData の各フィールドが渡されること。"""
        pool, conn = make_mock_pool()
        repo = ProfileRepository(pool)
        data = make_calibration_data(
            accel_zero=500, accel_full=5500, brake_zero=600, brake_full=5600
        )

        await repo.save_calibration(PROFILE_ID, data)

        args = conn.execute.call_args[0]
        # args: (sql, record_id, profile_id, accel_zero, accel_full, accel_stroke, ...)
        assert args[3] == 500   # accel_zero_pos
        assert args[4] == 5500  # accel_full_pos
        assert args[5] == 5000  # accel_stroke
        assert args[6] == 600   # brake_zero_pos
        assert args[7] == 5600  # brake_full_pos
        assert args[8] == 5000  # brake_stroke

    @pytest.mark.asyncio
    async def test_save_calibration_db_error_propagates_and_logs(self) -> None:
        """DB エラーが伝播し、ログが出力されること。"""
        pool, conn = make_mock_pool()
        conn.execute.side_effect = RuntimeError("DB接続エラー")
        repo = ProfileRepository(pool)
        data = make_calibration_data()

        with patch.object(
            __import__("src.infra.profile_repository", fromlist=["logger"]),
            "logger",
        ) as mock_logger:
            with pytest.raises(RuntimeError, match="DB接続エラー"):
                await repo.save_calibration(PROFILE_ID, data)

        mock_logger.exception.assert_called_once()


class TestCreateProfile:
    @pytest.mark.asyncio
    async def test_create_executes_insert(self) -> None:
        """create が conn.execute を1回呼ぶこと。"""
        pool, conn = make_mock_pool()
        repo = ProfileRepository(pool)
        profile = VehicleProfile(
            id=str(uuid4()),
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

        result = await repo.create(profile)

        conn.execute.assert_awaited_once()
        assert result.name == "TestCar"

    @pytest.mark.asyncio
    async def test_create_returns_profile_with_id(self) -> None:
        """create が id を持つ VehicleProfile を返すこと。"""
        pool, conn = make_mock_pool()
        repo = ProfileRepository(pool)
        profile = VehicleProfile(
            id="",
            name="NoCar",
            max_accel_opening=50.0,
            max_brake_opening=50.0,
            max_speed=100.0,
            max_decel_g=0.3,
            pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
            stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
            calibration=None,
            model_path=None,
            created_at=datetime.now(tz=UTC),
            updated_at=datetime.now(tz=UTC),
        )

        result = await repo.create(profile)

        assert result.id != ""


class TestDeleteProfile:
    @pytest.mark.asyncio
    async def test_delete_returns_true_when_deleted(self) -> None:
        pool, conn = make_mock_pool()
        conn.execute.return_value = "DELETE 1"
        repo = ProfileRepository(pool)

        result = await repo.delete(PROFILE_ID)

        assert result is True

    @pytest.mark.asyncio
    async def test_delete_returns_false_when_not_found(self) -> None:
        pool, conn = make_mock_pool()
        conn.execute.return_value = "DELETE 0"
        repo = ProfileRepository(pool)

        result = await repo.delete(PROFILE_ID)

        assert result is False
