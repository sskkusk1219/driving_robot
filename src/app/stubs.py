"""実機なしで起動するためのスタブコンポーネント群。開発・テスト用。"""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from datetime import UTC, datetime
from uuid import uuid4

from src.app.robot_controller import RobotController
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pid import PIDController
from src.domain.learning_drive import LearningDriveManager
from src.infra.db import DuplicateNameError
from src.infra.ups_monitor import UPSStatus
from src.models.calibration import CalibrationData
from src.models.drive_log import DriveLog, DriveSession
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile
from src.models.time_schedule import TimeSchedule


class _StubActuator:
    async def connect(self) -> None:
        pass

    async def enable_modbus_control(self) -> None:
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

    async def move_to_position_timed(  # noqa: ARG002
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None:
        pass

    async def wait_for_position_complete(self) -> None:
        pass


class _StubCANReader:
    async def connect(self) -> None:
        pass

    async def read_speed(self) -> float:
        return 0.0


class _StubUPSMonitor:
    """開発・テスト用 UPS スタブ。常に満充電・AC通電中を返す。"""

    async def get_battery_level_pct(self) -> float:
        return 100.0

    async def get_status(self) -> UPSStatus:
        return UPSStatus(
            battery_charge_pct=100.0,
            on_battery=False,
            low_battery=False,
            status_flags="OL",
            is_available=True,
        )

    def register_ac_loss_callback(self, cb: object) -> None:
        pass

    async def start_polling(self) -> None:
        pass

    async def stop_polling(self) -> None:
        pass

    @property
    def is_on_battery(self) -> bool:
        return False

    @property
    def is_available(self) -> bool:
        return True


class _StubButtonServo:
    """開発・テスト用ボタンサーボスタブ。押下・解放を no-op で受ける。"""

    async def connect(self) -> None:
        pass

    async def press(self, channel: int, duration_s: float) -> None:  # noqa: ARG002
        pass

    async def release_all(self) -> None:
        pass

    async def check_connection(self) -> bool:
        return True


class _StubSafetyMonitor:
    """実機の SafetyMonitor と同様にコールバックをディスパッチするスタブ。"""

    def __init__(self) -> None:
        self._emergency_callbacks: list[Callable[[], Awaitable[None]]] = []

    async def start_monitoring(self) -> None:
        pass

    async def stop_monitoring(self) -> None:
        pass

    def register_emergency_callback(self, cb: Callable[[], Awaitable[None]]) -> None:
        self._emergency_callbacks.append(cb)

    async def trigger_emergency(self) -> None:
        for cb in self._emergency_callbacks:
            await cb()

    def is_emergency_active(self) -> bool:
        return False


def build_stub_controller() -> RobotController:
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    safety_monitor = _StubSafetyMonitor()
    controller = RobotController(
        accel_driver=_StubActuator(),
        brake_driver=_StubActuator(),
        can_reader=_StubCANReader(),
        safety_monitor=safety_monitor,
        pid=pid,
        ff_controller=FeedforwardController(),
        last_normal_shutdown=False,
        learning_manager=LearningDriveManager(),
        button_servo=_StubButtonServo(),
    )
    # 実機 factory と同じ一方向ディスパッチ配線（monitor → controller）
    safety_monitor.register_emergency_callback(controller.emergency_stop)
    return controller


class InMemoryProfileRepository:
    """DB なし環境用の in-memory プロファイルリポジトリ。"""

    def __init__(self) -> None:
        self._profiles: dict[str, VehicleProfile] = {}

    async def list_all(self) -> list[VehicleProfile]:
        return sorted(self._profiles.values(), key=lambda p: p.created_at, reverse=True)

    async def get_by_id(self, profile_id: str) -> VehicleProfile | None:
        return self._profiles.get(profile_id)

    async def create(self, profile: VehicleProfile) -> VehicleProfile:
        # DB の UNIQUE(name) 制約と挙動を揃える
        if any(p.name == profile.name for p in self._profiles.values()):
            raise DuplicateNameError(f"プロファイル名 {profile.name!r} は既に使用されています")
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
            feedforward_params=profile.feedforward_params,
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
            feedforward_params=profile.feedforward_params,
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
        # DB の UNIQUE(name) 制約と挙動を揃える
        if any(m.name == mode.name for m in self._modes.values()):
            raise DuplicateNameError(f"走行モード名 {mode.name!r} は既に使用されています")
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

    async def update(self, mode: DrivingMode) -> DrivingMode | None:
        if mode.id not in self._modes:
            return None
        self._modes[mode.id] = mode
        return mode

    async def delete(self, mode_id: str) -> bool:
        if mode_id not in self._modes:
            return False
        del self._modes[mode_id]
        return True


class InMemoryScheduleRepository:
    """DB なし環境用の in-memory タイムスケジュールリポジトリ。"""

    def __init__(self) -> None:
        self._schedules: dict[str, TimeSchedule] = {}

    async def list_all(self) -> list[TimeSchedule]:
        return sorted(self._schedules.values(), key=lambda s: s.created_at, reverse=True)

    async def get_by_id(self, schedule_id: str) -> TimeSchedule | None:
        return self._schedules.get(schedule_id)

    async def create(self, schedule: TimeSchedule) -> TimeSchedule:
        # DB の UNIQUE(name) 制約と挙動を揃える
        if any(s.name == schedule.name for s in self._schedules.values()):
            raise DuplicateNameError(f"スケジュール名 {schedule.name!r} は既に使用されています")
        schedule_id = schedule.id if schedule.id else str(uuid4())
        stored = TimeSchedule(
            id=schedule_id,
            name=schedule.name,
            description=schedule.description,
            pedal_points=schedule.pedal_points,
            button_events=schedule.button_events,
            total_duration=schedule.total_duration,
            loop=schedule.loop,
            created_at=schedule.created_at,
        )
        self._schedules[schedule_id] = stored
        return stored

    async def update(self, schedule: TimeSchedule) -> TimeSchedule | None:
        if schedule.id not in self._schedules:
            return None
        # 別レコードとの名称重複を DB と同様に弾く
        if any(
            s.name == schedule.name and sid != schedule.id
            for sid, s in self._schedules.items()
        ):
            raise DuplicateNameError(f"スケジュール名 {schedule.name!r} は既に使用されています")
        self._schedules[schedule.id] = schedule
        return schedule

    async def delete(self, schedule_id: str) -> bool:
        if schedule_id not in self._schedules:
            return False
        del self._schedules[schedule_id]
        return True


class InMemorySessionRepository:
    """DB なし環境用の in-memory セッションリポジトリ（常に空）。"""

    async def list_all(self, limit: int = 100) -> list[DriveSession]:  # noqa: ARG002
        return []

    async def get_by_id(self, session_id: str) -> DriveSession | None:  # noqa: ARG002
        return None

    async def latest_learning_session_id(self, profile_id: str) -> str | None:  # noqa: ARG002
        return None

    async def list_logs(self, session_id: str, limit: int = 1000) -> list[DriveLog]:  # noqa: ARG002
        return []

    async def list_logs_for_training(
        self,
        profile_id: str,  # noqa: ARG002
        session_ids: list[str] | None = None,  # noqa: ARG002
        limit: int = 100_000,  # noqa: ARG002
    ) -> list[DriveLog]:
        return []
