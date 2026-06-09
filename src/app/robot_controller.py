import asyncio
import logging
from collections.abc import Awaitable, Callable
from datetime import UTC, datetime
from typing import Protocol
from uuid import uuid4

from src.domain.control.drive_loop import DriveLoop
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pid import PIDController
from src.models.calibration import CalibrationResult
from src.models.drive_log import DriveLogData, DriveSession
from src.models.driving_mode import DrivingMode
from src.models.pre_check import PreCheckResult
from src.models.profile import VehicleProfile
from src.models.system_state import RealtimeSnapshot, RobotState, SystemState

_logger = logging.getLogger(__name__)


class InvalidStateTransition(Exception):
    """不正な状態遷移を試みた場合に送出。"""


class EmergencyStillActive(Exception):
    """非常停止スイッチが物理的に解除されていない状態でリセットを試みた場合に送出。"""


class PreCheckFailed(Exception):
    """走行前チェックが失敗した場合に送出。"""

    def __init__(self, result: "PreCheckResult | None" = None) -> None:
        self.result = result
        super().__init__(str(result))


VALID_TRANSITIONS: dict[RobotState, frozenset[RobotState]] = {
    # 物理的な非常停止スイッチはどの運用画面でも押されうるため、起動前(BOOTING)を除く
    # 全状態から EMERGENCY への遷移を許可する（安全オーバーライド）。
    # BOOTING 中は GPIO 監視が未起動のため割り込みは発生しない。
    RobotState.BOOTING: frozenset({RobotState.STANDBY, RobotState.ERROR}),
    RobotState.STANDBY: frozenset({RobotState.INITIALIZING, RobotState.EMERGENCY}),
    RobotState.INITIALIZING: frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.READY: frozenset(
        {RobotState.CALIBRATING, RobotState.PRE_CHECK, RobotState.EMERGENCY}
    ),
    RobotState.CALIBRATING: frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.PRE_CHECK: frozenset(
        {RobotState.RUNNING, RobotState.MANUAL, RobotState.READY, RobotState.EMERGENCY}
    ),
    RobotState.RUNNING: frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.MANUAL: frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.EMERGENCY: frozenset({RobotState.STANDBY}),
    # 非常停止からの復帰は STANDBY へ戻し、初期化（アラームリセット→サーボON→原点復帰）を
    # 改めて実施させる。物理スイッチ解除後の安全な再立ち上げ手順に合わせる。
    RobotState.ERROR: frozenset({RobotState.STANDBY}),
}


class ActuatorDriverProtocol(Protocol):
    async def connect(self) -> None: ...

    async def enable_modbus_control(self) -> None: ...

    async def home_return(self) -> None: ...

    async def servo_off(self) -> None: ...

    async def servo_on(self) -> None: ...

    async def reset_alarm(self) -> None: ...

    async def is_alarm_active(self) -> bool: ...

    async def read_position(self) -> int: ...

    async def read_current(self) -> float: ...

    async def move_to_position(self, pos: int) -> None: ...

    async def wait_for_position_complete(self) -> None: ...


class CANReaderProtocol(Protocol):
    async def connect(self) -> None: ...

    async def read_speed(self) -> float: ...


class SafetyMonitorProtocol(Protocol):
    async def start_monitoring(self) -> None: ...

    async def stop_monitoring(self) -> None: ...

    def register_emergency_callback(self, cb: Callable[[], Awaitable[None]]) -> None: ...

    async def trigger_emergency(self) -> None: ...

    def is_emergency_active(self) -> bool: ...


class SafetyCheckProtocol(Protocol):
    """DriveLoop に渡す安全チェック専用プロトコル。SafetyMonitor が実装する。"""

    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...

    def check_deviation(self, ref: float, actual: float, duration: float) -> bool: ...


class CalibrationManagerProtocol(Protocol):
    async def run_calibration(self, profile_id: str) -> CalibrationResult: ...

    async def save_manual(
        self,
        profile_id: str,
        accel_zero: int | None,
        accel_full: int | None,
        brake_zero: int | None,
        brake_full: int | None,
    ) -> CalibrationResult: ...


class PreCheckRunnerProtocol(Protocol):
    async def run(self) -> PreCheckResult: ...

    def set_profile(self, profile: VehicleProfile | None) -> None: ...


class LogWriterProtocol(Protocol):
    """走行セッション・ログの永続化プロトコル。LogWriter が実装する。"""

    async def start_session(self, profile_id: str, mode_id: str | None, run_type: str) -> str: ...

    async def write_log(self, session_id: str, data: DriveLogData) -> None: ...

    async def end_session(self, session_id: str, status: str) -> None: ...


class LearningDriveManagerProtocol(Protocol):
    """学習走行マネージャーのプロトコル。"""

    def build_learning_reference(self, profile: VehicleProfile) -> DrivingMode:
        """学習用の連続基準速度プロファイル（加減速を網羅）を生成する。"""
        ...


class RobotController:
    """システム状態機械とコンポーネント協調制御を担うアプリケーションレイヤー。"""

    _state: RobotState
    _active_profile: VehicleProfile | None
    _active_session_id: str | None
    _last_normal_shutdown: bool
    _accel_driver: ActuatorDriverProtocol
    _brake_driver: ActuatorDriverProtocol
    _can_reader: CANReaderProtocol
    _safety_monitor: SafetyMonitorProtocol
    _pid: PIDController
    _ff_controller: FeedforwardController | None
    _safety_check: SafetyCheckProtocol | None
    _pre_check_runner: PreCheckRunnerProtocol | None
    _calibration_manager: CalibrationManagerProtocol | None
    _learning_manager: LearningDriveManagerProtocol | None
    _drive_loop: DriveLoop | None
    _log_writer: LogWriterProtocol | None
    _active_learning_task: asyncio.Task[None] | None
    _pending_calib_zero: dict[str, int | None]
    _pending_calib_full: dict[str, int | None]

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
        pre_check_runner: PreCheckRunnerProtocol | None = None,
        calibration_manager: CalibrationManagerProtocol | None = None,
        learning_manager: LearningDriveManagerProtocol | None = None,
    ) -> None:
        self._state = RobotState.BOOTING
        self._active_profile = None
        self._active_session_id = None
        self._last_normal_shutdown = last_normal_shutdown
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._safety_monitor = safety_monitor
        self._pid = pid
        self._ff_controller = ff_controller
        self._safety_check = safety_check
        self._pre_check_runner = pre_check_runner
        self._calibration_manager = calibration_manager
        self._learning_manager = learning_manager
        self._drive_loop = None
        self._log_writer = None
        self._active_learning_task = None
        self._pending_calib_zero: dict[str, int | None] = {"accel": None, "brake": None}
        self._pending_calib_full: dict[str, int | None] = {"accel": None, "brake": None}

    def _transition(self, new_state: RobotState) -> None:
        allowed = VALID_TRANSITIONS.get(self._state, frozenset())
        if new_state not in allowed:
            raise InvalidStateTransition(f"{self._state} → {new_state} は許可されていない遷移です")
        self._state = new_state

    def get_system_state(self) -> SystemState:
        return SystemState(
            robot_state=self._state,
            active_profile_id=self._active_profile.id if self._active_profile else None,
            active_session_id=self._active_session_id,
            last_normal_shutdown=self._last_normal_shutdown,
            updated_at=datetime.now(tz=UTC),
        )

    def select_profile(self, profile: VehicleProfile) -> None:
        """アクティブプロファイルを設定する。STANDBY/READY 状態のみ許可。"""
        if self._state not in (RobotState.STANDBY, RobotState.READY):
            raise InvalidStateTransition(
                f"select_profile は STANDBY/READY 状態でのみ呼べます (現在: {self._state})"
            )
        self._active_profile = profile
        if self._pre_check_runner is not None:
            self._pre_check_runner.set_profile(profile)
        if self._ff_controller is not None:
            # フィードフォワード物理定数を反映（モデル有無に関わらず適用）。
            self._ff_controller.set_params(profile.feedforward_params)
            # 運転モデルが紐づいていればロードする。
            # 失敗しても走行不可にはせず警告に留める（走行前チェックで別途担保）。
            if profile.model_path:
                try:
                    self._ff_controller.load_model(profile.model_path)
                except (FileNotFoundError, ValueError) as e:
                    _logger.warning(
                        "運転モデルのロードに失敗しました (profile=%s, path=%s): %s",
                        profile.id,
                        profile.model_path,
                        e,
                    )

    def get_active_profile(self) -> VehicleProfile | None:
        """現在選択中のプロファイルを返す。未選択の場合は None。"""
        return self._active_profile

    @property
    def current_openings(self) -> tuple[float, float]:
        """現在のアクセル・ブレーキ開度 [%] を返す。DriveLoop 非動作時は (0.0, 0.0)。"""
        if self._drive_loop is not None:
            return self._drive_loop.current_accel_opening, self._drive_loop.current_brake_opening
        return 0.0, 0.0

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
            self._accel_driver.enable_modbus_control(),
            self._brake_driver.enable_modbus_control(),
        )
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
        await self._close_session("completed")
        self._last_normal_shutdown = True

    async def shutdown(self) -> None:
        """グレースフルシャットダウン: 状態を問わず DriveLoop を停止し安全監視を解除する。"""
        if self._drive_loop is not None:
            self._drive_loop.stop()
            self._drive_loop = None
        if self._active_learning_task is not None:
            self._active_learning_task.cancel()
            self._active_learning_task = None
        await self._safety_monitor.stop_monitoring()

    async def emergency_stop(self) -> None:
        """非常停止: 即座に EMERGENCY へ遷移し、両軸を原点復帰させる。

        DriveLoop の on_emergency と GPIO 割り込み（多重発火含む）が重複呼び出しされても
        冪等に動作するよう、既に EMERGENCY の場合は DriveLoop 停止のみ行い早期 return する。
        これにより最大30秒かかる home_return が共有 Modbus バス上で多重起動し、
        get_realtime_data（状態配信）を長時間ブロックするのを防ぐ。

        先行コルーチンは最初の await（gather）までに同期的に EMERGENCY へ遷移するため、
        後続コルーチンは確実に EMERGENCY を観測して早期 return できる。
        """
        if self._drive_loop is not None:
            self._drive_loop.stop()
            self._drive_loop = None
        if self._state == RobotState.EMERGENCY:
            return
        self._transition(RobotState.EMERGENCY)
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        await self._safety_monitor.trigger_emergency()
        await self._close_session("emergency")
        self._last_normal_shutdown = False

    async def reset_emergency(self) -> None:
        """非常停止リセット: EMERGENCY → STANDBY。

        物理スイッチがまだ押下されている場合はリセットを拒否し EmergencyStillActive を送出する
        （スイッチ解除を強制してから復帰させる安全策）。
        復帰後は初期化画面から initialize() を実行して READY へ再立ち上げする想定。
        """
        if self._safety_monitor.is_emergency_active():
            raise EmergencyStillActive(
                "非常停止スイッチが解除されていません。物理スイッチを戻してから再度リセットしてください。"
            )
        self._transition(RobotState.STANDBY)

    async def clear_error(self) -> None:
        """エラー解除: ERROR → STANDBY。"""
        self._transition(RobotState.STANDBY)

    async def run_calibration(self) -> CalibrationResult:
        """キャリブレーション実行。READY → CALIBRATING → READY。"""
        self._transition(RobotState.CALIBRATING)
        try:
            if self._calibration_manager is not None:
                return await self._calibration_manager.run_calibration(
                    profile_id=self._active_profile.id if self._active_profile else ""
                )
            return CalibrationResult(
                success=False, data=None, error_message="キャリブレーション未設定"
            )
        finally:
            self._transition(RobotState.READY)

    async def _open_session(
        self,
        profile_id: str,
        mode_id: str | None,
        run_type: str,
        log_writer: LogWriterProtocol | None,
    ) -> str:
        """走行セッションを採番する。

        log_writer と profile_id が揃っている場合は drive_sessions に INSERT し、
        DB 採番のセッション ID を返す（以降 write_log/end_session で同 ID を使う）。
        それ以外（DB なし・プロファイル未選択）は UUID をローカル採番し、ログは永続化しない。
        """
        if log_writer is not None and profile_id:
            session_id = await log_writer.start_session(profile_id, mode_id, run_type)
            self._log_writer = log_writer
        else:
            session_id = str(uuid4())
            self._log_writer = None
        self._active_session_id = session_id
        return session_id

    async def _close_session(self, status: str) -> None:
        """アクティブセッションを終了する。永続化中の場合のみ end_session を記録する。

        status: 'completed' | 'error' | 'emergency'
        """
        if self._log_writer is not None and self._active_session_id is not None:
            try:
                await self._log_writer.end_session(self._active_session_id, status)
            except Exception:
                _logger.exception("走行セッション終了の記録に失敗しました (走行停止は継続)")
        self._log_writer = None
        self._active_session_id = None

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
        try:
            if self._pre_check_runner is not None:
                result = await self._pre_check_runner.run()
                if not result.passed:
                    raise PreCheckFailed(result)
            self._transition(RobotState.RUNNING)
        except PreCheckFailed:
            self._transition(RobotState.READY)
            raise
        except Exception:
            self._transition(RobotState.READY)
            raise
        profile_id = self._active_profile.id if self._active_profile else ""
        session_id = await self._open_session(profile_id, mode_id, "auto", log_writer)
        session = DriveSession(
            id=session_id,
            profile_id=profile_id,
            mode_id=mode_id,
            run_type="auto",
            started_at=datetime.now(tz=UTC),
            ended_at=None,
            status="running",
        )

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
        await self._close_session("completed")

    async def start_manual(self) -> DriveSession:
        """手動操作開始。READY → PRE_CHECK → MANUAL。"""
        self._transition(RobotState.PRE_CHECK)
        try:
            if self._pre_check_runner is not None:
                result = await self._pre_check_runner.run()
                if not result.passed:
                    raise PreCheckFailed(result)
            self._transition(RobotState.MANUAL)
        except PreCheckFailed:
            self._transition(RobotState.READY)
            raise
        except Exception:
            self._transition(RobotState.READY)
            raise
        session = DriveSession(
            id=str(uuid4()),
            profile_id=self._active_profile.id if self._active_profile else "",
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

    async def start_learning_drive(
        self,
        log_writer: LogWriterProtocol | None = None,
    ) -> DriveSession:
        """学習走行開始。READY → PRE_CHECK → RUNNING。

        学習用の連続基準速度プロファイル（加減速を網羅）を DriveLoop で走行し、
        100ms 周期で走行ログ（DriveLog）を `drive_logs` に記録する（run_type='learning'）。
        正常完了で on_complete=stop_auto_drive により RUNNING→READY 遷移しセッションを
        'completed' で終了する。停止/非常停止でも DriveLoop を止めセッションを終了する。
        """
        self._transition(RobotState.PRE_CHECK)
        try:
            if self._pre_check_runner is not None:
                result = await self._pre_check_runner.run()
                if not result.passed:
                    raise PreCheckFailed(result)
            self._transition(RobotState.RUNNING)
        except PreCheckFailed:
            self._transition(RobotState.READY)
            raise
        except Exception:
            self._transition(RobotState.READY)
            raise
        profile = self._active_profile
        profile_id = profile.id if profile else ""
        session_id = await self._open_session(profile_id, None, "learning", log_writer)
        session = DriveSession(
            id=session_id,
            profile_id=profile_id,
            mode_id=None,
            run_type="learning",
            started_at=datetime.now(tz=UTC),
            ended_at=None,
            status="running",
        )

        if (
            profile is not None
            and profile.calibration is not None
            and self._ff_controller is not None
            and self._safety_check is not None
            and self._learning_manager is not None
        ):
            learning_mode = self._learning_manager.build_learning_reference(profile)
            self._drive_loop = DriveLoop(
                ff_controller=self._ff_controller,
                pid=self._pid,
                accel_driver=self._accel_driver,
                brake_driver=self._brake_driver,
                can_reader=self._can_reader,
                profile=profile,
                mode=learning_mode,
                safety_check=self._safety_check,
                on_complete=self.stop_auto_drive,
                on_emergency=self.emergency_stop,
                log_writer=log_writer,
                session_id=session_id,
            )
            self._drive_loop.start()

        return session

    def _get_axis_driver(self, axis: str) -> ActuatorDriverProtocol:
        """軸名からドライバを返す。不明な軸名は ValueError。"""
        if axis == "accel":
            return self._accel_driver
        if axis == "brake":
            return self._brake_driver
        raise ValueError(f"不明な軸: {axis!r}。'accel' または 'brake' を指定してください。")

    async def jog_axis(self, axis: str, step: int) -> int:
        """軸を step [pulse] だけ相対移動して実位置を返す。

        READY 状態の場合は CALIBRATING へ自動遷移する（キャリブレーション画面の最初の操作）。
        CALIBRATING / MANUAL 状態では状態遷移なしで移動する。
        """
        if self._state == RobotState.READY:
            self._transition(RobotState.CALIBRATING)
            self._pending_calib_zero = {"accel": None, "brake": None}
            self._pending_calib_full = {"accel": None, "brake": None}
        elif self._state not in (RobotState.CALIBRATING, RobotState.MANUAL):
            raise InvalidStateTransition(
                f"jog_axis は READY/CALIBRATING/MANUAL 状態でのみ呼べます (現在: {self._state})"
            )
        driver = self._get_axis_driver(axis)
        current = await driver.read_position()
        await driver.move_to_position(current + step)
        # 移動完了を待ってから位置を読む（直後の read_position は移動途中値を返すため）
        await driver.wait_for_position_complete()
        return await driver.read_position()

    async def home_axis(self, axis: str) -> int:
        """軸を原点復帰して 0 を返す。

        READY 状態の場合は CALIBRATING へ自動遷移する。
        CALIBRATING / MANUAL 状態では状態遷移なしで原点復帰する。
        """
        if self._state == RobotState.READY:
            self._transition(RobotState.CALIBRATING)
            self._pending_calib_zero = {"accel": None, "brake": None}
            self._pending_calib_full = {"accel": None, "brake": None}
        elif self._state not in (RobotState.CALIBRATING, RobotState.MANUAL):
            raise InvalidStateTransition(
                f"home_axis は READY/CALIBRATING/MANUAL 状態でのみ呼べます (現在: {self._state})"
            )
        driver = self._get_axis_driver(axis)
        await driver.home_return()
        return 0

    async def calib_set_zero(self, axis: str) -> int:
        """現在位置をゼロ点として記録し、位置を返す。CALIBRATING 状態でのみ呼べる。"""
        if self._state != RobotState.CALIBRATING:
            raise InvalidStateTransition(
                f"calib_set_zero は CALIBRATING 状態でのみ呼べます (現在: {self._state})"
            )
        driver = self._get_axis_driver(axis)
        pos = await driver.read_position()
        self._pending_calib_zero[axis] = pos
        return pos

    async def calib_set_full(self, axis: str) -> int:
        """現在位置をフル点として記録し、位置を返す。CALIBRATING 状態でのみ呼べる。"""
        if self._state != RobotState.CALIBRATING:
            raise InvalidStateTransition(
                f"calib_set_full は CALIBRATING 状態でのみ呼べます (現在: {self._state})"
            )
        driver = self._get_axis_driver(axis)
        pos = await driver.read_position()
        self._pending_calib_full[axis] = pos
        return pos

    async def save_manual_calibration(self) -> CalibrationResult:
        """手動設定したゼロ/フル点でキャリブレーションを保存する。

        成功時のみアクセル・ブレーキを原点復帰させてペダルを解放し、
        CALIBRATING → READY へ遷移する。

        保存失敗（バリデーション不合格・DBエラー等）時は CALIBRATING を維持し、
        記録済みゼロ/フル点（pending）を保持したまま結果を返す（または例外を伝播する）。
        これによりゼロ/フル点を修正して再保存（リトライ）できる。失敗時は原点復帰しない。
        """
        if self._state != RobotState.CALIBRATING:
            raise InvalidStateTransition(
                f"save_manual_calibration は CALIBRATING 状態でのみ呼べます (現在: {self._state})"
            )
        if self._calibration_manager is not None:
            result = await self._calibration_manager.save_manual(
                profile_id=self._active_profile.id if self._active_profile else "",
                accel_zero=self._pending_calib_zero.get("accel"),
                accel_full=self._pending_calib_full.get("accel"),
                brake_zero=self._pending_calib_zero.get("brake"),
                brake_full=self._pending_calib_full.get("brake"),
            )
        else:
            result = CalibrationResult(
                success=False, data=None, error_message="キャリブレーション未設定"
            )
        if not result.success:
            # 失敗時は CALIBRATING を維持してリトライ可能にする（原点復帰しない）
            return result
        # 成功時のみ両軸を原点復帰してペダルを解放し、READY へ遷移する
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        self._transition(RobotState.READY)
        return result
