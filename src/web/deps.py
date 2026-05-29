from __future__ import annotations

from typing import Protocol

from fastapi import Request

from src.app.robot_controller import RobotController
from src.infra.ups_monitor import NutUPSMonitor, UPSStatus
from src.models.calibration import CalibrationData
from src.models.drive_log import DriveLog, DriveSession
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile


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


class SessionRepoProtocol(Protocol):
    async def list_all(self, limit: int = 100) -> list[DriveSession]: ...
    async def get_by_id(self, session_id: str) -> DriveSession | None: ...
    async def list_logs(self, session_id: str, limit: int = 1000) -> list[DriveLog]: ...


class UPSMonitorProtocol(Protocol):
    async def get_status(self) -> UPSStatus: ...

    @property
    def is_available(self) -> bool: ...


def get_controller(request: Request) -> RobotController:
    controller: RobotController = request.app.state.controller
    return controller


def get_profile_repo(request: Request) -> ProfileRepoProtocol:
    repo: ProfileRepoProtocol = request.app.state.profile_repo
    return repo


def get_mode_repo(request: Request) -> ModeRepoProtocol:
    repo: ModeRepoProtocol = request.app.state.mode_repo
    return repo


def get_session_repo(request: Request) -> SessionRepoProtocol:
    repo: SessionRepoProtocol = request.app.state.session_repo
    return repo


def get_ups_monitor(request: Request) -> UPSMonitorProtocol:
    monitor: UPSMonitorProtocol = request.app.state.ups_monitor
    return monitor
