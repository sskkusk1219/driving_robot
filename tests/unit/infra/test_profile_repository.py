"""ProfileRepository のユニットテスト。"""

from contextlib import asynccontextmanager
from datetime import UTC, datetime
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import UUID, uuid4

import asyncpg
import pytest

from src.infra.db import DuplicateNameError
from src.infra.profile_repository import (
    ProfileRepository,
    _dyn_from_value,
    _dyn_to_json,
    _ffp_from_value,
    _ffp_to_json,
    _row_to_profile,
)
from src.models.calibration import CalibrationData
from src.models.profile import (
    DynamicsParams,
    FeedforwardParams,
    PIDGains,
    StopConfig,
    VehicleProfile,
)

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

    @asynccontextmanager
    async def _transaction():
        yield

    conn.transaction = MagicMock(side_effect=_transaction)
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
        assert args[3] == 500  # accel_zero_pos
        assert args[4] == 5500  # accel_full_pos
        assert args[5] == 5000  # accel_stroke
        assert args[6] == 600  # brake_zero_pos
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


class TestFeedforwardParamsPersistence:
    def test_ffp_from_value_none_returns_defaults(self) -> None:
        assert _ffp_from_value(None) == FeedforwardParams()

    def test_ffp_from_value_partial_json_merges_defaults(self) -> None:
        ffp = _ffp_from_value('{"creep_speed_kmh": 9.0, "stop_brake_opening_pct": 35.0}')
        assert ffp.creep_speed_kmh == 9.0
        assert ffp.stop_brake_opening_pct == 35.0
        # 欠損キーはデフォルト
        assert ffp.brake_deadband_pct == FeedforwardParams().brake_deadband_pct

    def test_ffp_from_value_accepts_dict(self) -> None:
        ffp = _ffp_from_value({"accel_deadband_pct": 2.5})
        assert ffp.accel_deadband_pct == 2.5

    def test_ffp_arbiter_constants_default_filled_from_old_record(self) -> None:
        # 調停定数追加前の旧 JSONB レコードでもデフォルト補完で読めること
        ffp = _ffp_from_value('{"creep_speed_kmh": 9.0}')
        defaults = FeedforwardParams()
        assert ffp.switch_hysteresis_pct == defaults.switch_hysteresis_pct
        assert ffp.accel_reengage_dwell_s == defaults.accel_reengage_dwell_s
        assert ffp.pid_output_limit_pct == defaults.pid_output_limit_pct

    def test_ffp_arbiter_constants_roundtrip(self) -> None:
        ffp = FeedforwardParams(
            switch_hysteresis_pct=1.2,
            accel_reengage_dwell_s=0.5,
            accel_rate_limit_pct_s=150.0,
            brake_rate_limit_pct_s=250.0,
            pid_output_limit_pct=40.0,
        )
        assert _ffp_from_value(_ffp_to_json(ffp)) == ffp

    def test_row_to_profile_parses_feedforward_params(self) -> None:
        row = {
            "id": PROFILE_UUID,
            "name": "Car",
            "max_accel_opening": 80.0,
            "max_brake_opening": 60.0,
            "max_speed": 120.0,
            "max_decel_g": 0.4,
            "pid_gains": '{"kp": 1.0, "ki": 0.0, "kd": 0.0}',
            "stop_config": '{"deviation_threshold_kmh": 2.0, "deviation_duration_s": 4.0}',
            "model_path": None,
            "feedforward_params": '{"creep_speed_kmh": 8.5}',
            "dynamics_params": None,
            "created_at": datetime.now(tz=UTC),
            "updated_at": datetime.now(tz=UTC),
        }
        profile = _row_to_profile(row)
        assert profile.feedforward_params.creep_speed_kmh == 8.5

    def test_row_to_profile_null_feedforward_params_defaults(self) -> None:
        row = {
            "id": PROFILE_UUID,
            "name": "Car",
            "max_accel_opening": 80.0,
            "max_brake_opening": 60.0,
            "max_speed": 120.0,
            "max_decel_g": 0.4,
            "pid_gains": '{"kp": 1.0, "ki": 0.0, "kd": 0.0}',
            "stop_config": '{"deviation_threshold_kmh": 2.0, "deviation_duration_s": 4.0}',
            "model_path": None,
            "feedforward_params": None,
            "dynamics_params": None,
            "created_at": datetime.now(tz=UTC),
            "updated_at": datetime.now(tz=UTC),
        }
        profile = _row_to_profile(row)
        assert profile.feedforward_params == FeedforwardParams()
        assert profile.dynamics_params == DynamicsParams()

    @pytest.mark.asyncio
    async def test_create_serializes_feedforward_params(self) -> None:
        pool, conn = make_mock_pool()
        repo = ProfileRepository(pool)
        profile = VehicleProfile(
            id=str(uuid4()),
            name="FfpCar",
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
            feedforward_params=FeedforwardParams(creep_speed_kmh=9.0),
        )
        result = await repo.create(profile)
        # INSERT 引数のいずれかに feedforward_params JSON が含まれること
        args = conn.execute.call_args[0]
        assert any(isinstance(a, str) and "creep_speed_kmh" in a for a in args)
        assert result.feedforward_params.creep_speed_kmh == 9.0


class TestDynamicsParamsPersistence:
    def test_dyn_from_value_none_returns_defaults(self) -> None:
        assert _dyn_from_value(None) == DynamicsParams()

    def test_dyn_from_value_partial_json_merges_defaults(self) -> None:
        dyn = _dyn_from_value('{"pid_preview_s": 1.2, "fopdt_theta": 0.8}')
        assert dyn.pid_preview_s == 1.2
        assert dyn.fopdt_theta == 0.8
        # 欠損キーはデフォルト
        assert dyn.fopdt_k == DynamicsParams().fopdt_k
        assert dyn.fopdt_tau == DynamicsParams().fopdt_tau

    def test_dyn_from_value_accepts_dict(self) -> None:
        dyn = _dyn_from_value({"pid_preview_s": 0.5})
        assert dyn.pid_preview_s == 0.5

    def test_legacy_preview_time_s_key_ignored_defaults_to_zero(self) -> None:
        """Stage A 改名前 DB の旧キー preview_time_s は未知フィールドとして無視され、
        pid_preview_s は既定 0.0 に補完される（マイグレーション不要の暗黙リセット）。
        θ は別途 fopdt_theta として保持されるので情報は失われない。"""
        dyn = _dyn_from_value('{"preview_time_s": 0.5, "fopdt_theta": 0.5}')
        assert dyn.pid_preview_s == 0.0  # 旧キーは無視され既定に落ちる
        assert dyn.fopdt_theta == 0.5  # θ は保持される

    def test_dyn_roundtrip(self) -> None:
        dyn = DynamicsParams(pid_preview_s=1.5, fopdt_k=2.0, fopdt_tau=1.1, fopdt_theta=0.7)
        assert _dyn_from_value(_dyn_to_json(dyn)) == dyn

    def test_row_to_profile_parses_dynamics_params(self) -> None:
        row = {
            "id": PROFILE_UUID,
            "name": "Car",
            "max_accel_opening": 80.0,
            "max_brake_opening": 60.0,
            "max_speed": 120.0,
            "max_decel_g": 0.4,
            "pid_gains": '{"kp": 1.0, "ki": 0.0, "kd": 0.0}',
            "stop_config": '{"deviation_threshold_kmh": 2.0, "deviation_duration_s": 4.0}',
            "model_path": None,
            "feedforward_params": None,
            "dynamics_params": '{"pid_preview_s": 0.9, "fopdt_theta": 0.9}',
            "created_at": datetime.now(tz=UTC),
            "updated_at": datetime.now(tz=UTC),
        }
        profile = _row_to_profile(row)
        assert profile.dynamics_params.pid_preview_s == 0.9
        assert profile.dynamics_params.fopdt_theta == 0.9

    def test_row_to_profile_null_dynamics_params_defaults(self) -> None:
        row = {
            "id": PROFILE_UUID,
            "name": "Car",
            "max_accel_opening": 80.0,
            "max_brake_opening": 60.0,
            "max_speed": 120.0,
            "max_decel_g": 0.4,
            "pid_gains": '{"kp": 1.0, "ki": 0.0, "kd": 0.0}',
            "stop_config": '{"deviation_threshold_kmh": 2.0, "deviation_duration_s": 4.0}',
            "model_path": None,
            "feedforward_params": None,
            "dynamics_params": None,
            "created_at": datetime.now(tz=UTC),
            "updated_at": datetime.now(tz=UTC),
        }
        profile = _row_to_profile(row)
        assert profile.dynamics_params == DynamicsParams()

    @pytest.mark.asyncio
    async def test_create_serializes_dynamics_params(self) -> None:
        pool, conn = make_mock_pool()
        repo = ProfileRepository(pool)
        profile = VehicleProfile(
            id=str(uuid4()),
            name="DynCar",
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
            dynamics_params=DynamicsParams(pid_preview_s=1.3),
        )
        result = await repo.create(profile)
        args = conn.execute.call_args[0]
        assert any(isinstance(a, str) and "pid_preview_s" in a for a in args)
        assert result.dynamics_params.pid_preview_s == 1.3

    @pytest.mark.asyncio
    async def test_update_serializes_dynamics_params(self) -> None:
        pool, conn = make_mock_pool()
        conn.execute.return_value = "UPDATE 1"
        repo = ProfileRepository(pool)
        profile = VehicleProfile(
            id=PROFILE_ID,
            name="DynCar",
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
            dynamics_params=DynamicsParams(pid_preview_s=2.1, fopdt_theta=0.6),
        )

        result = await repo.update(profile)

        args = conn.execute.call_args[0]
        assert any(isinstance(a, str) and "pid_preview_s" in a for a in args)
        assert result is not None
        assert result.dynamics_params.pid_preview_s == 2.1


class TestUpdateProfile:
    @pytest.mark.asyncio
    async def test_update_converts_unique_violation_to_duplicate_name_error(self) -> None:
        """I5 回帰テスト: update 時の一意制約違反を DuplicateNameError（→409）に変換する
        （create は既に変換済みだが update だけ未変換で 500 になっていたバグの回帰）。"""
        pool, conn = make_mock_pool()
        conn.execute.side_effect = asyncpg.UniqueViolationError("duplicate key")
        repo = ProfileRepository(pool)
        profile = VehicleProfile(
            id=PROFILE_ID,
            name="既存プロファイル名",
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

        with pytest.raises(DuplicateNameError):
            await repo.update(profile)


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
