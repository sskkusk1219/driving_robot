from __future__ import annotations

from typing import Protocol

from fastapi import Request

from src.app.learning_cycle import LearningCycleOrchestrator
from src.app.robot_controller import LogWriterProtocol, RobotController
from src.domain.model_training import DEFAULT_FEATURE_SPEC, FeatureSpec
from src.infra.settings import LearningSettings
from src.infra.ups_monitor import UPSStatus
from src.models.calibration import CalibrationData
from src.models.drive_log import DriveLog, DriveSession, LearningCycle
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile
from src.models.time_schedule import TimeSchedule


class ProfileRepoProtocol(Protocol):
    async def list_all(self) -> list[VehicleProfile]: ...
    async def get_by_id(self, profile_id: str) -> VehicleProfile | None: ...
    async def create(self, profile: VehicleProfile) -> VehicleProfile: ...
    async def update(self, profile: VehicleProfile) -> VehicleProfile | None: ...
    async def delete(self, profile_id: str) -> bool: ...
    async def save_calibration(self, profile_id: str, data: CalibrationData) -> None: ...


class ModeRepoProtocol(Protocol):
    async def list_all(self) -> list[DrivingMode]: ...
    async def get_by_id(self, mode_id: str) -> DrivingMode | None: ...
    async def create(self, mode: DrivingMode) -> DrivingMode: ...
    async def update(self, mode: DrivingMode) -> DrivingMode | None: ...
    async def delete(self, mode_id: str) -> bool: ...


class ScheduleRepoProtocol(Protocol):
    async def list_all(self) -> list[TimeSchedule]: ...
    async def get_by_id(self, schedule_id: str) -> TimeSchedule | None: ...
    async def create(self, schedule: TimeSchedule) -> TimeSchedule: ...
    async def update(self, schedule: TimeSchedule) -> TimeSchedule | None: ...
    async def delete(self, schedule_id: str) -> bool: ...


class SessionRepoProtocol(Protocol):
    async def list_all(self, limit: int = 100) -> list[DriveSession]: ...
    async def get_by_id(self, session_id: str) -> DriveSession | None: ...
    async def latest_learning_session_id(self, profile_id: str) -> str | None: ...
    async def list_logs(self, session_id: str, limit: int = 1000) -> list[DriveLog]: ...
    async def list_logs_for_training(
        self,
        profile_id: str,
        session_ids: list[str] | None = None,
        limit: int = 100_000,
    ) -> list[DriveLog]: ...
    async def list_session_ids_for_cycle(self, cycle_id: str) -> list[str]: ...
    async def list_cycles(
        self, profile_id: str | None = None, limit: int = 100
    ) -> list[LearningCycle]: ...


class UPSMonitorProtocol(Protocol):
    async def get_status(self) -> UPSStatus: ...

    @property
    def is_available(self) -> bool: ...


def get_controller(request: Request) -> RobotController:
    controller: RobotController = request.app.state.controller
    return controller


def get_log_writer(request: Request) -> LogWriterProtocol | None:
    """DB プールが利用可能なら走行ログを永続化する LogWriter を返す。

    in-memory 環境（DB なし）では None を返し、走行ログ記録を無効化する。
    """
    pool = getattr(request.app.state, "db_pool", None)
    if pool is None:
        return None
    from src.infra.log_writer import LogWriter  # noqa: PLC0415

    return LogWriter(pool)


def get_profile_repo(request: Request) -> ProfileRepoProtocol:
    repo: ProfileRepoProtocol = request.app.state.profile_repo
    return repo


def get_mode_repo(request: Request) -> ModeRepoProtocol:
    repo: ModeRepoProtocol = request.app.state.mode_repo
    return repo


def get_session_repo(request: Request) -> SessionRepoProtocol:
    repo: SessionRepoProtocol = request.app.state.session_repo
    return repo


def get_schedule_repo(request: Request) -> ScheduleRepoProtocol:
    repo: ScheduleRepoProtocol = request.app.state.schedule_repo
    return repo


def get_feature_spec(request: Request) -> FeatureSpec:
    """学習時に使う特徴量構成。app.state 未設定（起動シーケンス外）時は現行9特徴を返す。"""
    spec: FeatureSpec = getattr(request.app.state, "feature_spec", DEFAULT_FEATURE_SPEC)
    return spec


def get_learning_settings(request: Request) -> LearningSettings:
    """2段階学習フローのデフォルトパラメータ。app.state 未設定時はデフォルト値を返す。"""
    settings: LearningSettings = getattr(
        request.app.state, "learning_settings", LearningSettings()
    )
    return settings


def get_cycle_orchestrator(request: Request) -> LearningCycleOrchestrator:
    orchestrator: LearningCycleOrchestrator = request.app.state.cycle_orchestrator
    return orchestrator


def get_ups_monitor(request: Request) -> UPSMonitorProtocol:
    monitor: UPSMonitorProtocol = request.app.state.ups_monitor
    return monitor
