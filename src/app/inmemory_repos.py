"""DB が無い環境（DATABASE_URL 未設定）用の in-memory リポジトリと UPS スタブ。

段3（RPD_Phase_20261115_PortResearch）で、Web が古い制御（src/app/stubs.py・robot_controller）を
import しなくて済むよう、そこから必要な部分だけを写した。stubs.py は段7 で消える。
走行モード・スケジュール・シーケンスは揮発、セッションは常に空（走行ログは DB だけに残す）。
"""

from __future__ import annotations

from collections.abc import AsyncIterator
from typing import Any
from uuid import uuid4

from src.infra.db import DuplicateNameError
from src.infra.mode_repository import RESERVED_SYSTEM_MODE_NAME
from src.infra.ups_monitor import UPSStatus
from src.models.drive_log import DriveLog, DriveSession, LearningCycle
from src.models.driving_mode import DrivingMode
from src.models.sequence import Sequence
from src.models.time_schedule import TimeSchedule


class StubUPSMonitor:
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


class InMemoryModeRepository:
    """DB なし環境用の in-memory 走行モードリポジトリ。"""

    def __init__(self) -> None:
        self._modes: dict[str, DrivingMode] = {}

    async def list_all(self) -> list[DrivingMode]:
        # システムモードはユーザー向け一覧から除外（DB リポジトリと挙動を揃える）
        return sorted(
            (m for m in self._modes.values() if not m.is_system),
            key=lambda m: m.created_at,
            reverse=True,
        )

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

    async def get_system_mode(self) -> DrivingMode | None:
        return next(
            (m for m in self._modes.values() if m.name == RESERVED_SYSTEM_MODE_NAME), None
        )

    async def upsert_system_mode(self, mode: DrivingMode) -> DrivingMode:
        # 予約名で既存システムモードを探し、あれば id を保って軌跡だけ差し替える。新規は
        # 常に採番する（渡される mode.id は "verify" 等の合成値で実 id ではない。DB 版と同じ）。
        existing = next(
            (m for m in self._modes.values() if m.name == RESERVED_SYSTEM_MODE_NAME), None
        )
        mode_id = existing.id if existing is not None else str(uuid4())
        stored = DrivingMode(
            id=mode_id,
            name=RESERVED_SYSTEM_MODE_NAME,
            description=mode.description,
            reference_speed=mode.reference_speed,
            total_duration=mode.total_duration,
            max_speed=mode.max_speed,
            created_at=mode.created_at,
            is_system=True,
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
            created_at=schedule.created_at,
        )
        self._schedules[schedule_id] = stored
        return stored

    async def update(self, schedule: TimeSchedule) -> TimeSchedule | None:
        if schedule.id not in self._schedules:
            return None
        # 別レコードとの名称重複を DB と同様に弾く
        if any(
            s.name == schedule.name and sid != schedule.id for sid, s in self._schedules.items()
        ):
            raise DuplicateNameError(f"スケジュール名 {schedule.name!r} は既に使用されています")
        self._schedules[schedule.id] = schedule
        return schedule

    async def delete(self, schedule_id: str) -> bool:
        if schedule_id not in self._schedules:
            return False
        del self._schedules[schedule_id]
        return True


class InMemorySequenceRepository:
    """DB なし環境用の in-memory シーケンスリポジトリ。"""

    def __init__(self) -> None:
        self._sequences: dict[str, Sequence] = {}

    async def list_all(self) -> list[Sequence]:
        return sorted(self._sequences.values(), key=lambda s: s.created_at, reverse=True)

    async def get_by_id(self, sequence_id: str) -> Sequence | None:
        return self._sequences.get(sequence_id)

    async def create(self, sequence: Sequence) -> Sequence:
        # DB の UNIQUE(name) 制約と挙動を揃える
        if any(s.name == sequence.name for s in self._sequences.values()):
            raise DuplicateNameError(f"シーケンス名 {sequence.name!r} は既に使用されています")
        sequence_id = sequence.id if sequence.id else str(uuid4())
        stored = Sequence(
            id=sequence_id,
            name=sequence.name,
            description=sequence.description,
            steps=sequence.steps,
            repeat=sequence.repeat,
            created_at=sequence.created_at,
        )
        self._sequences[sequence_id] = stored
        return stored

    async def update(self, sequence: Sequence) -> Sequence | None:
        if sequence.id not in self._sequences:
            return None
        if any(
            s.name == sequence.name and sid != sequence.id for sid, s in self._sequences.items()
        ):
            raise DuplicateNameError(f"シーケンス名 {sequence.name!r} は既に使用されています")
        self._sequences[sequence.id] = sequence
        return sequence

    async def delete(self, sequence_id: str) -> bool:
        if sequence_id not in self._sequences:
            return False
        del self._sequences[sequence_id]
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

    async def list_messages(self, session_id: str, limit: int = 100_000) -> list[Any]:  # noqa: ARG002
        return []

    async def log_detail_keys(self, session_id: str) -> list[str]:  # noqa: ARG002
        return []

    async def iter_logs_for_csv(
        self,
        session_id: str,  # noqa: ARG002
        chunk: int = 2000,  # noqa: ARG002
    ) -> AsyncIterator[list[Any]]:
        return
        yield []

    async def list_logs_for_training(
        self,
        profile_id: str,  # noqa: ARG002
        session_ids: list[str] | None = None,  # noqa: ARG002
        limit: int = 100_000,  # noqa: ARG002
    ) -> list[DriveLog]:
        return []

    async def list_session_ids_for_cycle(self, cycle_id: str) -> list[str]:  # noqa: ARG002
        return []

    async def list_cycles(
        self,
        profile_id: str | None = None,  # noqa: ARG002
        limit: int = 100,  # noqa: ARG002
    ) -> list[LearningCycle]:
        return []
