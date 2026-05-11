"""実機なしで起動するためのスタブコンポーネント群。開発・テスト用。"""

from __future__ import annotations

from datetime import UTC, datetime
from uuid import uuid4

from src.app.robot_controller import RobotController
from src.domain.control.pid import PIDController
from src.models.calibration import CalibrationData
from src.models.drive_log import DriveLog, DriveSession
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile


class _StubActuator:
    async def connect(self) -> None:
        pass

    async def home_return(self) -> None:
        pass

    async def servo_off(self) -> None:
        pass

    async def servo_on(self) -> None:
        pass

    async def reset_alarm(self) -> None:
        pass

    async def is_alarm_active(self) -> bool:
        return False

    async def read_position(self) -> int:
        return 0

    async def read_current(self) -> float:
        return 0.0

    async def move_to_position(self, pos: int) -> None:  # noqa: ARG002
        pass


class _StubCANReader:
    async def connect(self) -> None:
        pass

    async def read_speed(self) -> float:
        return 0.0


class _StubSafetyMonitor:
    async def start_monitoring(self) -> None:
        pass

    async def stop_monitoring(self) -> None:
        pass

    def register_emergency_callback(self, cb: object) -> None:
        pass

    async def trigger_emergency(self) -> None:
        pass


def build_stub_controller() -> RobotController:
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    return RobotController(
        accel_driver=_StubActuator(),
        brake_driver=_StubActuator(),
        can_reader=_StubCANReader(),
        safety_monitor=_StubSafetyMonitor(),
        pid=pid,
        last_normal_shutdown=False,
    )


class InMemoryProfileRepository:
    """DB なし環境用の in-memory プロファイルリポジトリ。"""

    def __init__(self) -> None:
        self._profiles: dict[str, VehicleProfile] = {}

    async def list_all(self) -> list[VehicleProfile]:
        return sorted(self._profiles.values(), key=lambda p: p.created_at, reverse=True)

    async def get_by_id(self, profile_id: str) -> VehicleProfile | None:
        return self._profiles.get(profile_id)

    async def create(self, profile: VehicleProfile) -> VehicleProfile:
        now = datetime.now(tz=UTC)
        profile_id = profile.id if profile.id else str(uuid4())
        stored = VehicleProfile(
            id=profile_id,
            name=profile.name,
            max_accel_opening=profile.max_accel_opening,
            max_brake_opening=profile.max_brake_opening,
            max_speed=profile.max_speed,
            max_decel_g=profile.max_decel_g,
            pid_gains=profile.pid_gains,
            stop_config=profile.stop_config,
            calibration=None,
            model_path=profile.model_path,
            created_at=now,
            updated_at=now,
        )
        self._profiles[profile_id] = stored
        return stored

    async def update(self, profile: VehicleProfile) -> VehicleProfile | None:
        if profile.id not in self._profiles:
            return None
        now = datetime.now(tz=UTC)
        updated = VehicleProfile(
            id=profile.id,
            name=profile.name,
            max_accel_opening=profile.max_accel_opening,
            max_brake_opening=profile.max_brake_opening,
            max_speed=profile.max_speed,
            max_decel_g=profile.max_decel_g,
            pid_gains=profile.pid_gains,
            stop_config=profile.stop_config,
            calibration=self._profiles[profile.id].calibration,
            model_path=profile.model_path,
            created_at=self._profiles[profile.id].created_at,
            updated_at=now,
        )
        self._profiles[profile.id] = updated
        return updated

    async def delete(self, profile_id: str) -> bool:
        if profile_id not in self._profiles:
            return False
        del self._profiles[profile_id]
        return True

    async def save_calibration(self, profile_id: str, data: CalibrationData) -> None:
        if profile_id in self._profiles:
            self._profiles[profile_id].calibration = data


class InMemoryModeRepository:
    """DB なし環境用の in-memory 走行モードリポジトリ。"""

    def __init__(self) -> None:
        self._modes: dict[str, DrivingMode] = {}

    async def list_all(self) -> list[DrivingMode]:
        return sorted(self._modes.values(), key=lambda m: m.created_at, reverse=True)

    async def get_by_id(self, mode_id: str) -> DrivingMode | None:
        return self._modes.get(mode_id)

    async def create(self, mode: DrivingMode) -> DrivingMode:
        mode_id = mode.id if mode.id else str(uuid4())
        stored = DrivingMode(
            id=mode_id,
            name=mode.name,
            description=mode.description,
            reference_speed=mode.reference_speed,
            total_duration=mode.total_duration,
            max_speed=mode.max_speed,
            created_at=mode.created_at,
        )
        self._modes[mode_id] = stored
        return stored

    async def delete(self, mode_id: str) -> bool:
        if mode_id not in self._modes:
            return False
        del self._modes[mode_id]
        return True


class InMemorySessionRepository:
    """DB なし環境用の in-memory セッションリポジトリ（常に空）。"""

    async def list_all(self, limit: int = 100) -> list[DriveSession]:  # noqa: ARG002
        return []

    async def get_by_id(self, session_id: str) -> DriveSession | None:  # noqa: ARG002
        return None

    async def list_logs(self, session_id: str, limit: int = 1000) -> list[DriveLog]:  # noqa: ARG002
        return []
