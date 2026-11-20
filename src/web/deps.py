from __future__ import annotations

from collections.abc import AsyncIterator
from typing import Protocol

from fastapi import Request

from src.app.job_manager import JobManager
from src.infra.session_repository import LogCsvRow, SessionMessage
from src.infra.ups_monitor import UPSStatus
from src.infra.vehicle_store import VehicleStore
from src.models.drive_log import DriveLog, DriveSession, LearningCycle
from src.models.driving_mode import DrivingMode
from src.models.sequence import Sequence
from src.models.time_schedule import TimeSchedule


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


class SequenceRepoProtocol(Protocol):
    async def list_all(self) -> list[Sequence]: ...
    async def get_by_id(self, sequence_id: str) -> Sequence | None: ...
    async def create(self, sequence: Sequence) -> Sequence: ...
    async def update(self, sequence: Sequence) -> Sequence | None: ...
    async def delete(self, sequence_id: str) -> bool: ...


class SessionRepoProtocol(Protocol):
    async def list_all(self, limit: int = 100) -> list[DriveSession]: ...
    async def get_by_id(self, session_id: str) -> DriveSession | None: ...
    async def latest_learning_session_id(self, profile_id: str) -> str | None: ...
    async def list_logs(self, session_id: str, limit: int = 1000) -> list[DriveLog]: ...
    async def list_messages(
        self, session_id: str, limit: int = 100_000
    ) -> list[SessionMessage]: ...
    async def log_detail_keys(self, session_id: str) -> list[str]: ...
    def iter_logs_for_csv(
        self, session_id: str, chunk: int = 2000
    ) -> AsyncIterator[list[LogCsvRow]]: ...
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


def get_job_manager(request: Request) -> JobManager:
    manager: JobManager = request.app.state.job_manager
    return manager


def get_vehicle_store(request: Request) -> VehicleStore:
    store: VehicleStore = request.app.state.vehicle_store
    return store


def get_mode_repo(request: Request) -> ModeRepoProtocol:
    repo: ModeRepoProtocol = request.app.state.mode_repo
    return repo


def get_session_repo(request: Request) -> SessionRepoProtocol:
    repo: SessionRepoProtocol = request.app.state.session_repo
    return repo


def get_schedule_repo(request: Request) -> ScheduleRepoProtocol:
    repo: ScheduleRepoProtocol = request.app.state.schedule_repo
    return repo


def get_sequence_repo(request: Request) -> SequenceRepoProtocol:
    repo: SequenceRepoProtocol = request.app.state.sequence_repo
    return repo


def get_ups_monitor(request: Request) -> UPSMonitorProtocol:
    monitor: UPSMonitorProtocol = request.app.state.ups_monitor
    return monitor
