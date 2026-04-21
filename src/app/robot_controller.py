import asyncio
from collections.abc import Awaitable, Callable
from datetime import UTC, datetime
from typing import Protocol
from uuid import uuid4

from src.domain.control.drive_loop import DriveLoop, LogWriterProtocol
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pid import PIDController
from src.models.calibration import CalibrationData, CalibrationResult
from src.models.drive_log import DriveSession
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile
from src.models.system_state import RealtimeSnapshot, RobotState, SystemState


class InvalidStateTransition(Exception):
    """不正な状態遷移を試みた場合に送出。"""


class PreCheckFailed(Exception):
    """走行前チェックが失敗した場合に送出。"""


VALID_TRANSITIONS: dict[RobotState, frozenset[RobotState]] = {
    RobotState.BOOTING: frozenset({RobotState.STANDBY, RobotState.ERROR}),
    RobotState.STANDBY: frozenset({RobotState.INITIALIZING}),
    RobotState.INITIALIZING: frozenset({RobotState.READY}),
    RobotState.READY: frozenset(
        {RobotState.CALIBRATING, RobotState.PRE_CHECK, RobotState.EMERGENCY}
        # EMERGENCY を含むのは、物理的な非常停止スイッチが READY 中でも押される場面があるため
    ),
    RobotState.CALIBRATING: frozenset({RobotState.READY}),
    RobotState.PRE_CHECK: frozenset({RobotState.RUNNING, RobotState.MANUAL, RobotState.READY}),
    RobotState.RUNNING: frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.MANUAL: frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.EMERGENCY: frozenset({RobotState.READY}),
    RobotState.ERROR: frozenset({RobotState.STANDBY}),
}


class ActuatorDriverProtocol(Protocol):
    async def connect(self) -> None: ...

    async def home_return(self) -> None: ...

    async def servo_off(self) -> None: ...

    async def servo_on(self) -> None: ...

    async def reset_alarm(self) -> None: ...

    async def is_alarm_active(self) -> bool: ...

    async def read_position(self) -> int: ...

    async def read_current(self) -> float: ...

    async def move_to_position(self, pos: int) -> None: ...


class CANReaderProtocol(Protocol):
    async def connect(self) -> None: ...

    async def read_speed(self) -> float: ...


class SafetyMonitorProtocol(Protocol):
    async def start_monitoring(self) -> None: ...

    async def stop_monitoring(self) -> None: ...

    def register_emergency_callback(self, cb: Callable[[], Awaitable[None]]) -> None: ...

    async def trigger_emergency(self) -> None: ...


class SafetyCheckProtocol(Protocol):
    """DriveLoop に渡す安全チェック専用プロトコル。SafetyMonitor が実装する。"""

    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...

    def check_deviation(self, ref: float, actual: float, duration: float) -> bool: ...


class RobotController:
    """システム状態機械とコンポーネント協調制御を担うアプリケーションレイヤー。"""

    _state: RobotState
    _active_profile_id: str | None
    _active_session_id: str | None
    _last_normal_shutdown: bool
    _accel_driver: ActuatorDriverProtocol
    _brake_driver: ActuatorDriverProtocol
    _can_reader: CANReaderProtocol
    _safety_monitor: SafetyMonitorProtocol
    _pid: PIDController
    _ff_controller: FeedforwardController | None
    _safety_check: SafetyCheckProtocol | None
    _drive_loop: DriveLoop | None

    def __init__(
        self,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        safety_monitor: SafetyMonitorProtocol,
        pid: PIDController,
        last_normal_shutdown: bool = False,
        ff_controller: FeedforwardController | None = None,
        safety_check: SafetyCheckProtocol | None = None,
    ) -> None:
        self._state = RobotState.BOOTING
        self._active_profile_id = None
        self._active_session_id = None
        self._last_normal_shutdown = last_normal_shutdown
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._safety_monitor = safety_monitor
        self._pid = pid
        self._ff_controller = ff_controller
        self._safety_check = safety_check
        self._drive_loop = None

    def _transition(self, new_state: RobotState) -> None:
        allowed = VALID_TRANSITIONS.get(self._state, frozenset())
        if new_state not in allowed:
            raise InvalidStateTransition(f"{self._state} → {new_state} は許可されていない遷移です")
        self._state = new_state

    def get_system_state(self) -> SystemState:
        return SystemState(
            robot_state=self._state,
            active_profile_id=self._active_profile_id,
            active_session_id=self._active_session_id,
            last_normal_shutdown=self._last_normal_shutdown,
            updated_at=datetime.now(tz=UTC),
        )

    async def get_realtime_data(self) -> RealtimeSnapshot:
        """ハードウェアからリアルタイム計測値を並列取得する。"""
        speed, accel_pos, brake_pos, accel_cur, brake_cur = await asyncio.gather(
            self._can_reader.read_speed(),
            self._accel_driver.read_position(),
            self._brake_driver.read_position(),
            self._accel_driver.read_current(),
            self._brake_driver.read_current(),
        )
        return RealtimeSnapshot(
            actual_speed_kmh=speed,
            accel_pos=accel_pos,
            brake_pos=brake_pos,
            accel_current_ma=accel_cur,
            brake_current_ma=brake_cur,
        )

    async def start(self) -> None:
        """電源ON後の通信確認。成功で STANDBY、失敗で ERROR へ遷移。"""
        try:
            await asyncio.gather(
                self._accel_driver.connect(),
                self._brake_driver.connect(),
                self._can_reader.connect(),
            )
            await self._safety_monitor.start_monitoring()
            self._transition(RobotState.STANDBY)
        except Exception:
            self._transition(RobotState.ERROR)
            raise

    async def initialize(self) -> None:
        """アラームリセット・サーボON・必要に応じて原点復帰。"""
        self._transition(RobotState.INITIALIZING)
        await asyncio.gather(
            self._accel_driver.reset_alarm(),
            self._brake_driver.reset_alarm(),
        )
        await asyncio.gather(
            self._accel_driver.servo_on(),
            self._brake_driver.servo_on(),
        )
        if not self._last_normal_shutdown:
            await asyncio.gather(
                self._accel_driver.home_return(),
                self._brake_driver.home_return(),
            )
        self._transition(RobotState.READY)

    async def stop(self) -> None:
        """正常停止: 原点復帰 → サーボOFF → READY 遷移。RUNNING または MANUAL 状態でのみ呼べる。"""
        if self._state not in (RobotState.RUNNING, RobotState.MANUAL):
            raise InvalidStateTransition(
                f"stop は RUNNING または MANUAL 状態でのみ呼べます (現在: {self._state})"
            )
        if self._drive_loop is not None:
            self._drive_loop.stop()
            self._drive_loop = None
        self._transition(RobotState.READY)
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        await asyncio.gather(
            self._accel_driver.servo_off(),
            self._brake_driver.servo_off(),
        )
        self._active_session_id = None
        self._last_normal_shutdown = True

    async def emergency_stop(self) -> None:
        """非常停止: 即座に EMERGENCY へ遷移し、両軸を原点復帰させる。

        DriveLoop の on_emergency と GPIO 割り込みが同サイクルで重複呼び出しされても
        冪等に動作するよう、既に EMERGENCY の場合は遷移をスキップする。
        """
        if self._drive_loop is not None:
            self._drive_loop.stop()
            self._drive_loop = None
        if self._state != RobotState.EMERGENCY:
            self._transition(RobotState.EMERGENCY)
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        await self._safety_monitor.trigger_emergency()
        self._active_session_id = None
        self._last_normal_shutdown = False

    async def reset_emergency(self) -> None:
        """非常停止リセット: EMERGENCY → READY。"""
        self._transition(RobotState.READY)

    async def clear_error(self) -> None:
        """エラー解除: ERROR → STANDBY。"""
        self._transition(RobotState.STANDBY)

    async def run_calibration(self) -> CalibrationResult:
        """キャリブレーション実行。READY → CALIBRATING → READY。"""
        self._transition(RobotState.CALIBRATING)
        try:
            # 実機実装: CalibrationManager.run_calibration() を呼ぶ
            stub_data = CalibrationData(
                accel_zero_pos=0,
                accel_full_pos=0,
                accel_stroke=0,
                brake_zero_pos=0,
                brake_full_pos=0,
                brake_stroke=0,
                calibrated_at=datetime.now(tz=UTC),
                is_valid=False,
            )
            return CalibrationResult(success=False, data=stub_data, error_message="未実装")
        finally:
            self._transition(RobotState.READY)

    async def start_auto_drive(
        self,
        mode_id: str,
        mode: DrivingMode | None = None,
        profile: VehicleProfile | None = None,
        log_writer: LogWriterProtocol | None = None,
    ) -> DriveSession:
        """自動走行開始。READY → PRE_CHECK → RUNNING。

        mode / profile / ff_controller / safety_check が全て揃っている場合に DriveLoop を起動する。
        """
        self._transition(RobotState.PRE_CHECK)
        # 実機実装: 走行前チェック6項目を実施。NG なら _transition(READY) + raise PreCheckFailed
        self._transition(RobotState.RUNNING)
        session = DriveSession(
            id=str(uuid4()),
            profile_id=self._active_profile_id or "",
            mode_id=mode_id,
            run_type="auto",
            started_at=datetime.now(tz=UTC),
            ended_at=None,
            status="running",
        )
        self._active_session_id = session.id

        if (
            mode is not None
            and profile is not None
            and profile.calibration is not None
            and self._ff_controller is not None
            and self._safety_check is not None
        ):
            self._drive_loop = DriveLoop(
                ff_controller=self._ff_controller,
                pid=self._pid,
                accel_driver=self._accel_driver,
                brake_driver=self._brake_driver,
                can_reader=self._can_reader,
                profile=profile,
                mode=mode,
                safety_check=self._safety_check,
                on_complete=self.stop_auto_drive,
                on_emergency=self.emergency_stop,
                log_writer=log_writer,
                session_id=session.id,
            )
            self._drive_loop.start()

        return session

    async def stop_auto_drive(self) -> None:
        """自動走行停止。RUNNING → READY。"""
        if self._drive_loop is not None:
            self._drive_loop.stop()
            self._drive_loop = None
        self._transition(RobotState.READY)
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        self._active_session_id = None

    async def start_manual(self) -> DriveSession:
        """手動操作開始。READY → PRE_CHECK → MANUAL。"""
        self._transition(RobotState.PRE_CHECK)
        # 実機実装: 走行前チェック6項目を実施。NG なら _transition(READY) + raise PreCheckFailed
        self._transition(RobotState.MANUAL)
        session = DriveSession(
            id=str(uuid4()),
            profile_id=self._active_profile_id or "",
            mode_id=None,
            run_type="manual",
            started_at=datetime.now(tz=UTC),
            ended_at=None,
            status="running",
        )
        self._active_session_id = session.id
        return session

    async def stop_manual(self) -> None:
        """手動操作終了。MANUAL → READY。MANUAL 状態以外から呼ぶと InvalidStateTransition。"""
        if self._state != RobotState.MANUAL:
            raise InvalidStateTransition(
                f"stop_manual は MANUAL 状態でのみ呼べます (現在: {self._state})"
            )
        self._transition(RobotState.READY)
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        self._active_session_id = None
