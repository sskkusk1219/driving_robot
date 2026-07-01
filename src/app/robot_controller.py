import asyncio
import logging
from collections.abc import Awaitable, Callable
from datetime import UTC, datetime
from typing import Protocol
from uuid import uuid4

from src.domain.control.drive_loop import DriveLoop
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.learning_loop import LearningLoop
from src.domain.control.pid import PIDController
from src.domain.control.schedule_loop import ScheduleLoop
from src.domain.pid_tuning import (
    CoordinateDescentTuner,
    build_tuning_trajectory,
    tuning_cost,
)
from src.domain.pre_check import ITEM_ACTUATOR_POSITION, ITEM_VEHICLE_STOPPED
from src.models.calibration import CalibrationResult
from src.models.drive_log import DriveLogData, DriveSession
from src.models.driving_mode import DrivingMode
from src.models.learning_drive import LearningPattern
from src.models.pre_check import PreCheckResult
from src.models.profile import PIDGains, StopConfig, VehicleProfile
from src.models.system_state import (
    InitStep,
    InitStepStatus,
    RealtimeSnapshot,
    RobotState,
    SystemState,
)
from src.models.time_schedule import TimeSchedule

_logger = logging.getLogger(__name__)

# DriveLoop スナップショットの許容鮮度 [s]。制御サイクルが固まった際に凍結キャッシュを
# 「現在値」として WS 配信し続けると、操作者に正常なダッシュボードを見せたまま
# ロボットが無制御になる（コードレビュー 2026-06-11 指摘 #16）。
SNAPSHOT_MAX_AGE_S: float = 0.5

# 学習運転 arm: ブレーキ踏込後に車速が 0 へ収束するのを待つパラメータ
_VEHICLE_STOP_SPEED_KMH: float = 0.5  # これ未満を停車とみなす
_VEHICLE_STOP_TIMEOUT_S: float = 10.0  # 収束待ちのタイムアウト
_VEHICLE_STOP_POLL_S: float = 0.2  # 車速ポーリング間隔

# 学習運転終了 → PID 自動適合への橋渡し減速。学習最終パターンは惰行/減速の途中で
# 終わり車両が転動したまま（最大数 km/h）のことがある。その状態で PID 適合の規定パターン
# （0km/h 始点）を走らせると追従誤差が大きく逸脱扱いになるため、緩やかに減速・停止確認して
# から適合へ移行する。
_G_TO_KMHS: float = 9.81 * 3.6  # 1G を km/h/s へ換算（learning_loop.G_TO_KMHS と同一）
_BRIDGE_DECEL_G: float = 0.1  # 目標減速 [G]（緩減速）
_BRIDGE_DECEL_POLL_S: float = 0.1  # 速度ポーリング兼ブレーキ調整間隔 [s]
_BRIDGE_DECEL_TIMEOUT_S: float = 30.0  # 減速の打ち切り（安全網）[s]
_BRIDGE_BRAKE_STEP_PCT: float = 1.0  # 1 サイクルあたりのブレーキ開度増減 [%]

# アクチュエータのハードウェア上限 [pulse]。これを超えると機械端に当たりタイムアウトになる。
_ACTUATOR_PULSE_MAX: int = 9500


class InvalidStateTransition(Exception):
    """不正な状態遷移を試みた場合に送出。"""


class EmergencyStillActive(Exception):
    """非常停止スイッチが物理的に解除されていない状態でリセットを試みた場合に送出。"""


class PreCheckFailed(Exception):
    """走行前チェックが失敗した場合に送出。"""

    def __init__(self, result: "PreCheckResult | None" = None) -> None:
        self.result = result
        super().__init__(str(result))


class PidTuningAborted(Exception):
    """PID 自動適合の走行中に非常停止が発生し、適合を中断した場合に送出。"""


# 規定パターン走行の完了待ちタイムアウト余裕 [s]（total_duration に加算）。
_PID_TUNING_DRIVE_TIMEOUT_MARGIN_S: float = 30.0


VALID_TRANSITIONS: dict[RobotState, frozenset[RobotState]] = {
    # 物理的な非常停止スイッチはどの運用画面でも押されうるため、起動前(BOOTING)を除く
    # 全状態から EMERGENCY への遷移を許可する（安全オーバーライド）。
    # BOOTING 中は GPIO 監視が未起動のため割り込みは発生しない。
    RobotState.BOOTING: frozenset({RobotState.STANDBY, RobotState.ERROR}),
    RobotState.STANDBY: frozenset({RobotState.INITIALIZING, RobotState.EMERGENCY}),
    RobotState.INITIALIZING: frozenset({RobotState.READY, RobotState.ERROR, RobotState.EMERGENCY}),
    RobotState.READY: frozenset(
        {RobotState.CALIBRATING, RobotState.PRE_CHECK, RobotState.EMERGENCY}
    ),
    RobotState.CALIBRATING: frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.PRE_CHECK: frozenset(
        {RobotState.RUNNING, RobotState.MANUAL, RobotState.READY, RobotState.EMERGENCY}
    ),
    RobotState.RUNNING: frozenset(
        {RobotState.READY, RobotState.PAUSED, RobotState.EMERGENCY}
    ),
    # 一時停止: 再開（RUNNING）・走行終了（READY）・非常停止（EMERGENCY）へ遷移できる。
    RobotState.PAUSED: frozenset(
        {RobotState.RUNNING, RobotState.READY, RobotState.EMERGENCY}
    ),
    RobotState.MANUAL: frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.EMERGENCY: frozenset({RobotState.STANDBY}),
    # 非常停止からの復帰は STANDBY へ戻し、初期化（アラームリセット→サーボON→原点復帰）を
    # 改めて実施させる。物理スイッチ解除後の安全な再立ち上げ手順に合わせる。
    RobotState.ERROR: frozenset({RobotState.STANDBY, RobotState.EMERGENCY}),
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

    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None: ...

    async def wait_for_position_complete(self) -> None: ...


class CANReaderProtocol(Protocol):
    async def connect(self) -> None: ...

    async def read_speed(self) -> float: ...


class ButtonServoProtocol(Protocol):
    """ボタンサーボ（PCA9685+SG90）ドライバのプロトコル。ButtonServoDriver が実装する。"""

    async def connect(self) -> None: ...

    async def press(self, channel: int, duration_s: float) -> None: ...

    async def release_all(self) -> None: ...


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

    def set_stop_config(self, stop_config: StopConfig) -> None: ...


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
    async def run(
        self,
        exclude: frozenset[str] = frozenset(),
        *,
        include_button_servo: bool = False,
    ) -> PreCheckResult: ...

    def set_profile(self, profile: VehicleProfile | None) -> None: ...


class LogWriterProtocol(Protocol):
    """走行セッション・ログの永続化プロトコル。LogWriter が実装する。"""

    async def start_session(self, profile_id: str, mode_id: str | None, run_type: str) -> str: ...

    async def write_log(self, session_id: str, data: DriveLogData) -> None: ...

    async def end_session(self, session_id: str, status: str) -> None: ...


class LearningDriveManagerProtocol(Protocol):
    """学習走行マネージャーのプロトコル。"""

    def generate_patterns(self, profile: VehicleProfile) -> list[LearningPattern]:
        """開ループ実行する開度パターン列を生成する。"""
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
    _learning_loop: LearningLoop | None
    _drive_loop: DriveLoop | None
    _schedule_loop: ScheduleLoop | None
    _button_servo: ButtonServoProtocol | None
    _log_writer: LogWriterProtocol | None
    _active_learning_task: asyncio.Task[None] | None
    _pending_calib_zero: dict[str, int | None]
    _pending_calib_full: dict[str, int | None]
    _init_steps: list[InitStep]

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
        button_servo: ButtonServoProtocol | None = None,
        control_interval_s: float = 0.05,
        log_every_n_cycles: int = 2,
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
        self._button_servo = button_servo
        self._drive_loop = None
        self._learning_loop = None
        self._schedule_loop = None
        self._log_writer = None
        self._active_learning_task = None
        self._last_kpi_summary: dict[str, float] | None = None
        # 自動走行が完了（stop_auto_drive）または非常停止（emergency_stop）したら set される。
        # PID 自動適合の走行完了待ちに使う。通常 UI 経路では無害な no-op。
        self._drive_complete = asyncio.Event()
        self._control_interval_s = control_interval_s
        self._log_every_n_cycles = log_every_n_cycles
        self._pending_calib_zero: dict[str, int | None] = {"accel": None, "brake": None}
        self._pending_calib_full: dict[str, int | None] = {"accel": None, "brake": None}
        self._init_steps = self._build_init_steps()

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
        self._apply_profile_to_control_stack(profile)

    def _apply_profile_to_control_stack(self, profile: VehicleProfile) -> None:
        """プロファイルの制御パラメータを制御スタックへ反映する。

        反映しないと UI で調整した PID ゲインが無言で無視され、逸脱停止の閾値も
        DriveLoop（profile.stop_config）と SafetyMonitor（構築時の固定値）で食い違う。
        """
        self._pid.set_gains(profile.pid_gains.kp, profile.pid_gains.ki, profile.pid_gains.kd)
        self._pid.set_output_limit(profile.feedforward_params.pid_output_limit_pct)
        if self._safety_check is not None:
            self._safety_check.set_stop_config(profile.stop_config)
        if self._ff_controller is not None:
            # フィードフォワード物理定数を反映（モデル有無に関わらず適用）。
            self._ff_controller.set_params(profile.feedforward_params)
            # 前プロファイルのモデルを必ず破棄してからロードする。アンロードしないと
            # model_path なし・ロード失敗のプロファイルで has_model が True のまま
            # 前車両のペダルマップが適用される（コードレビュー 2026-06-11 指摘 #4）。
            self._ff_controller.unload_model()
            # 運転モデルが紐づいていればロードする。
            # 失敗しても走行不可にはせず警告に留める（FF なし = PID 単独で走行可能）。
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

    def refresh_active_profile(self, profile: VehicleProfile) -> bool:
        """アクティブプロファイルの内容を更新し、制御スタックへ再反映する。

        モデル学習（/learning/train）後に呼ぶ。DB のプロファイルだけ更新して
        in-memory を放置すると、学習直後の走行が旧モデル（または FF なし）で
        実行される（コードレビュー 2026-06-11 指摘 #10）。

        Returns:
            反映した場合 True。アクティブでない・走行系状態のため反映しない場合 False。
        """
        if self._active_profile is None or self._active_profile.id != profile.id:
            return False
        if self._state in (
            RobotState.RUNNING,
            RobotState.MANUAL,
            RobotState.CALIBRATING,
            RobotState.PRE_CHECK,
        ):
            # 走行・操作中の制御パラメータ差し替えは挙動が不定になるため拒否する
            _logger.warning(
                "走行系状態 (%s) のためプロファイル更新を制御スタックへ反映しません", self._state
            )
            return False
        self._active_profile = profile
        if self._pre_check_runner is not None:
            self._pre_check_runner.set_profile(profile)
        self._apply_profile_to_control_stack(profile)
        return True

    def get_active_profile(self) -> VehicleProfile | None:
        """現在選択中のプロファイルを返す。未選択の場合は None。"""
        return self._active_profile

    @property
    def last_kpi_summary(self) -> dict[str, float] | None:
        """直近の走行の KPI サマリ（P95・最大偏差・符号反転率・ハード違反数）。"""
        return self._last_kpi_summary

    def _record_kpi_summary(self, drive_loop: DriveLoop) -> None:
        """走行終了時に KPI 集計をログへ残し、API/検証用に保持する。

        プライマリー KPI（P95 0.2km/h・最大 1.0km/h・符号反転 ≤1/5s）は逸脱自動停止より
        厳しく、走行が「正常完了」しても violated であり得るため必ず記録する（指摘 #7）。
        """
        summary = drive_loop.kpi_summary
        if summary.get("n_samples", 0.0) <= 0.0:
            return
        self._last_kpi_summary = summary
        _logger.info(
            "走行 KPI サマリ: P95=%.2fkm/h 最大偏差=%.2fkm/h 符号反転(最大/5s)=%.0f "
            "ハード上限(1.0km/h)違反=%.0f回 (n=%.0f)",
            summary["p95_kmh"],
            summary["max_abs_deviation_kmh"],
            summary["reversal_max_per_5s"],
            summary["hard_limit_violations"],
            summary["n_samples"],
        )

    @property
    def _realtime_loop(self) -> DriveLoop | LearningLoop | ScheduleLoop | None:
        """現在動作中のループ（走行 DriveLoop / 学習 LearningLoop / スケジュール ScheduleLoop）。
        3者は同一のリアルタイム計測 I/F（current_*_opening / current_ref_speed /
        last_snapshot）を公開する。同時には高々1つのみ動作する。"""
        if self._drive_loop is not None:
            return self._drive_loop
        if self._learning_loop is not None:
            return self._learning_loop
        return self._schedule_loop

    def _active_control_loop(self) -> DriveLoop | LearningLoop | ScheduleLoop | None:
        """現在動作中の制御ループ（stop/stop_and_join を持つ）。同時には高々1つ。"""
        return self._realtime_loop

    async def _release_button_servo(self) -> None:
        """全ボタンサーボを待機位置へ戻す。未構成・失敗時も処理は継続する。"""
        if self._button_servo is None:
            return
        try:
            await self._button_servo.release_all()
        except Exception:
            _logger.exception("ボタンサーボの解放に失敗しました（処理は継続）")

    @property
    def current_openings(self) -> tuple[float, float]:
        """現在のアクセル・ブレーキ開度 [%] を返す。ループ非動作時は (0.0, 0.0)。"""
        loop = self._realtime_loop
        if loop is not None:
            return loop.current_accel_opening, loop.current_brake_opening
        return 0.0, 0.0

    @property
    def current_ref_speed(self) -> float | None:
        """現在の基準車速 [km/h]。ループ非動作時・学習運転時は None。WebSocket 配信で使う。"""
        loop = self._realtime_loop
        if loop is not None:
            return loop.current_ref_speed
        return None

    async def get_realtime_data(self) -> RealtimeSnapshot:
        """リアルタイム計測値を返す。

        走行中（DriveLoop 動作中）は直近サイクルの計測値キャッシュを返し、
        50ms 制御ループが使う Modbus/CAN バスへ追加トランザクションを発行しない。
        キャッシュがサイクル凍結等で古くなった場合は障害を隠さないよう
        ハードウェア読み取りへフォールバックする。非走行時もハードウェアから並列取得する。
        """
        realtime_loop = self._realtime_loop
        if realtime_loop is not None:
            snapshot = realtime_loop.last_snapshot
            if snapshot is not None:
                age_s = asyncio.get_running_loop().time() - snapshot.captured_at
                if age_s <= SNAPSHOT_MAX_AGE_S:
                    return snapshot
                _logger.warning(
                    "走行中スナップショットが %.1fs 更新されていません: 直接読み取りへ切替",
                    age_s,
                )
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
            captured_at=asyncio.get_running_loop().time(),
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
            # ボタンサーボは任意構成。接続失敗はスケジュール走行のみ不可とし、起動自体は継続する。
            if self._button_servo is not None:
                try:
                    await self._button_servo.connect()
                except Exception:
                    _logger.warning(
                        "ボタンサーボの接続に失敗しました（タイムスケジュール走行は利用不可）",
                        exc_info=True,
                    )
            self._transition(RobotState.STANDBY)
        except Exception:
            self._transition(RobotState.ERROR)
            raise

    @staticmethod
    def _build_init_steps() -> list[InitStep]:
        """初期化シーケンスのステップ一覧を PENDING で生成する。

        各 key はフロント表示と実ハード操作の対応を示す。順序が表示順となる。
        """
        return [
            InitStep(key="comm_brake", label="通信確認 (ブレーキ)"),
            InitStep(key="comm_accel", label="通信確認 (アクセル)"),
            InitStep(key="comm_can", label="通信確認 (CAN)"),
            InitStep(key="alarm_reset", label="アラームリセット (両軸)"),
            InitStep(key="servo_on", label="サーボON (両軸)"),
            InitStep(key="home_return", label="原点復帰"),
        ]

    @property
    def init_progress(self) -> list[InitStep]:
        """初期化シーケンスの現在の進捗を返す。WebSocket 配信で利用する。"""
        return self._init_steps

    def _set_init_step(self, key: str, status: InitStepStatus) -> None:
        """指定キーの初期化ステップの状態を更新する。"""
        for step in self._init_steps:
            if step.key == key:
                step.status = status
                return

    def _fail_running_init_steps(self) -> None:
        """RUNNING 中の初期化ステップを ERROR にする。例外発生時に呼ぶ。"""
        for step in self._init_steps:
            if step.status == InitStepStatus.RUNNING:
                step.status = InitStepStatus.ERROR

    async def initialize(self) -> None:
        """アラームリセット・サーボON・必要に応じて原点復帰。

        各ハード操作の進捗を ``_init_steps`` に逐次反映し、WebSocket 経由で
        フロントの初期化画面と連動させる。

        失敗時は ERROR へ遷移する（INITIALIZING に留まると再試行が
        InvalidStateTransition で恒久的に拒否され、復旧手段がなくなるため）。
        ERROR からの再呼び出しは STANDBY を経由して再試行として受け付ける。
        """
        if self._state == RobotState.ERROR:
            self._transition(RobotState.STANDBY)
        self._transition(RobotState.INITIALIZING)
        self._init_steps = self._build_init_steps()
        try:
            # 通信確認: 各軸・CAN の応答を確認しながらステップを進める。
            self._set_init_step("comm_brake", InitStepStatus.RUNNING)
            await self._brake_driver.enable_modbus_control()
            self._set_init_step("comm_brake", InitStepStatus.DONE)

            self._set_init_step("comm_accel", InitStepStatus.RUNNING)
            await self._accel_driver.enable_modbus_control()
            self._set_init_step("comm_accel", InitStepStatus.DONE)

            self._set_init_step("comm_can", InitStepStatus.RUNNING)
            await self._can_reader.read_speed()
            self._set_init_step("comm_can", InitStepStatus.DONE)

            # アラームリセット（両軸同時）
            self._set_init_step("alarm_reset", InitStepStatus.RUNNING)
            await asyncio.gather(
                self._accel_driver.reset_alarm(),
                self._brake_driver.reset_alarm(),
            )
            self._set_init_step("alarm_reset", InitStepStatus.DONE)

            # サーボON（両軸同時）
            self._set_init_step("servo_on", InitStepStatus.RUNNING)
            await asyncio.gather(
                self._accel_driver.servo_on(),
                self._brake_driver.servo_on(),
            )
            self._set_init_step("servo_on", InitStepStatus.DONE)

            # 原点復帰（前回正常終了済みならスキップ）
            if not self._last_normal_shutdown:
                self._set_init_step("home_return", InitStepStatus.RUNNING)
                await asyncio.gather(
                    self._accel_driver.home_return(),
                    self._brake_driver.home_return(),
                )
                self._set_init_step("home_return", InitStepStatus.DONE)
            else:
                self._set_init_step("home_return", InitStepStatus.SKIPPED)
        except Exception:
            self._fail_running_init_steps()
            self._transition(RobotState.ERROR)
            raise
        self._transition(RobotState.READY)

    async def stop(self) -> None:
        """正常停止: 原点復帰 → サーボOFF → READY。RUNNING・PAUSED・MANUAL 状態でのみ呼べる。"""
        if self._state not in (RobotState.RUNNING, RobotState.PAUSED, RobotState.MANUAL):
            raise InvalidStateTransition(
                f"stop は RUNNING・PAUSED・MANUAL 状態でのみ呼べます (現在: {self._state})"
            )
        drive_loop = self._drive_loop
        active_loop = self._active_control_loop()
        self._drive_loop = None
        self._learning_loop = None
        self._schedule_loop = None
        if active_loop is not None:
            # 飛行中の位置指令完了を待ってから home_return（バス上のインターリーブ防止）
            await active_loop.stop_and_join()
            if drive_loop is not None:  # KPI は閉ループ走行のみ集計（学習走行には無い）
                self._record_kpi_summary(drive_loop)
        self._transition(RobotState.READY)
        # home_return / servo_off が失敗してもセッションは必ず閉じる（status='running' 残り防止）
        try:
            await self._release_button_servo()  # スケジュール走行のボタンサーボを待機位置へ
            await asyncio.gather(
                self._accel_driver.home_return(),
                self._brake_driver.home_return(),
            )
            await asyncio.gather(
                self._accel_driver.servo_off(),
                self._brake_driver.servo_off(),
            )
        finally:
            await self._close_session("completed")
            self._last_normal_shutdown = True

    async def shutdown(self) -> None:
        """グレースフルシャットダウン: DriveLoop を停止し、ペダル解放とサーボOFFを試みる。

        プロセス終了後はソフトウェア側の監視・制御が一切残らないため、ここで
        ペダルを解放しないと最終指令位置（例: アクセル40%）のままサーボONで
        保持され続ける。サーボONになり得る状態ではベストエフォートで
        原点復帰 → サーボOFF を実施する（失敗してもシャットダウンは続行）。
        """
        drive_loop = self._drive_loop
        learning_loop = self._learning_loop
        schedule_loop = self._schedule_loop
        self._drive_loop = None
        self._learning_loop = None
        self._schedule_loop = None
        if drive_loop is not None:
            await drive_loop.stop_and_join()
        if learning_loop is not None:
            await learning_loop.stop_and_join()
        if schedule_loop is not None:
            await schedule_loop.stop_and_join()
        await self._release_button_servo()
        if self._active_learning_task is not None:
            self._active_learning_task.cancel()
            self._active_learning_task = None
        if self._state in (
            RobotState.READY,
            RobotState.CALIBRATING,
            RobotState.PRE_CHECK,
            RobotState.RUNNING,
            RobotState.PAUSED,
            RobotState.MANUAL,
        ):
            try:
                await asyncio.gather(
                    self._accel_driver.home_return(),
                    self._brake_driver.home_return(),
                )
                await asyncio.gather(
                    self._accel_driver.servo_off(),
                    self._brake_driver.servo_off(),
                )
            except Exception:
                _logger.exception("シャットダウン時のペダル解放に失敗しました")
        await self._close_session("error")
        await self._safety_monitor.stop_monitoring()

    async def emergency_stop(self) -> None:
        """非常停止: 即座に EMERGENCY へ遷移し、両軸を原点復帰させる。

        DriveLoop の on_emergency と GPIO 割り込み（多重発火含む）が重複呼び出しされても
        冪等に動作するよう、既に EMERGENCY の場合は DriveLoop 停止のみ行い早期 return する。
        これにより最大30秒かかる home_return が共有 Modbus バス上で多重起動し、
        get_realtime_data（状態配信）を長時間ブロックするのを防ぐ。

        先行コルーチンは最初の await までに同期的に EMERGENCY へ遷移するため、
        後続コルーチンは確実に EMERGENCY を観測して早期 return できる。

        home_return の前に必ず stop_and_join で飛行中サイクルの完了を待つ。
        待たないと await 中の位置指令（例: アクセル40%）が原点復帰のコイル操作と
        同一軸バスでインターリーブし、非常停止中にペダルが再適用され得る
        （コードレビュー 2026-06-11 指摘 #6）。
        """
        # 走行/学習/スケジュールのいずれが動作中でも停止する（同時には動かない）
        drive_loop = self._drive_loop
        active_loop = self._active_control_loop()
        self._drive_loop = None
        self._learning_loop = None
        self._schedule_loop = None
        if active_loop is not None:
            active_loop.stop()  # 同期で停止フラグ（以降のサイクルは書き込み前に中断）
        if self._state == RobotState.EMERGENCY:
            if active_loop is not None:
                await active_loop.stop_and_join()
            await self._release_button_servo()
            # 先行呼び出しが home_return 失敗等で閉じ損ねた場合に備え、ここでも閉じる（冪等）
            await self._close_session("emergency")
            self._drive_complete.set()  # PID 自動適合の走行完了待ちを起こす（中断検知）
            return
        self._transition(RobotState.EMERGENCY)
        # home_return（実機 Modbus）が失敗してもセッションは必ず 'emergency' で閉じる。
        # try なしだと例外で _close_session に到達せず status='running' のまま残る。
        try:
            if active_loop is not None:
                await active_loop.stop_and_join()
                if drive_loop is not None:  # KPI は閉ループ走行のみ集計（学習走行には無い）
                    self._record_kpi_summary(drive_loop)
            await self._release_button_servo()  # 全ボタンサーボを待機位置へ
            await asyncio.gather(
                self._accel_driver.home_return(),
                self._brake_driver.home_return(),
            )
        finally:
            await self._close_session("emergency")
            self._last_normal_shutdown = False
            self._drive_complete.set()  # PID 自動適合の走行完了待ちを起こす（中断検知）

    async def _dispatch_emergency(self) -> None:
        """DriveLoop 内部起因（過電流・逸脱・CAN断・サイクル例外）の非常停止を発火する。

        GPIO・AC断と同じ SafetyMonitor ディスパッチャを通すことで、登録済みコールバック
        （通知等）が内部起因の非常停止でも発火する（コードレビュー 2026-06-11 指摘 #15）。
        コールバック未登録の DI 構成でも安全に倒れるよう、ディスパッチ後に EMERGENCY へ
        遷移していなければ emergency_stop を直接呼ぶ（冪等）。
        """
        await self._safety_monitor.trigger_emergency()
        if self._state != RobotState.EMERGENCY:
            await self.emergency_stop()

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

    async def _run_pre_check_and_transition(
        self, target: RobotState, *, include_button_servo: bool = False
    ) -> None:
        """READY → PRE_CHECK → target の共通プロローグ。失敗時は READY へロールバックする。

        include_button_servo=True でボタンサーボ確認（走行前チェック項目8）を追加する
        （タイムスケジュール実行時のみ）。
        """
        self._transition(RobotState.PRE_CHECK)
        try:
            if self._pre_check_runner is not None:
                result = await self._pre_check_runner.run(
                    include_button_servo=include_button_servo
                )
                if not result.passed:
                    raise PreCheckFailed(result)
            self._transition(target)
        except Exception:
            self._transition(RobotState.READY)
            raise

    async def _begin_session(
        self,
        run_type: str,
        mode_id: str | None,
        log_writer: LogWriterProtocol | None,
        expected_state: RobotState,
    ) -> DriveSession:
        """走行セッションを開始する。

        _open_session の await 中に非常停止等で状態が変わった場合は、走行を開始せず
        セッションを閉じて中断する（EMERGENCY 中の DriveLoop 起動を防ぐ）。
        """
        profile_id = self._active_profile.id if self._active_profile else ""
        session_id = await self._open_session(profile_id, mode_id, run_type, log_writer)
        if self._state != expected_state:
            await self._close_session("emergency")
            raise InvalidStateTransition(
                f"走行開始処理中に状態が {self._state} へ変化したため中断しました"
            )
        return DriveSession(
            id=session_id,
            profile_id=profile_id,
            mode_id=mode_id,
            run_type=run_type,
            started_at=datetime.now(tz=UTC),
            ended_at=None,
            status="running",
        )

    async def _run_tuning_drive(
        self, profile: VehicleProfile, log_writer: LogWriterProtocol | None
    ) -> dict[str, float]:
        """規定パターンを 1 回走行し、完了を待って KPI サマリを返す。

        **走行前チェック・arm はしない**。学習終了（停車保持ブレーキで停止）／前の最適化走行
        終了（同上）で車両は既に停止保持済みのため、DriveLoop を直接起動する。各走行の終了でも
        原点復帰せず停止保持を維持する（on_complete=_stop_tuning_drive）。最適化セッション全体の
        完了時にのみ原点復帰で解放する。

        完了（_stop_tuning_drive で _drive_complete set）または非常停止まで待つ。
        非常停止・タイムアウト時は PidTuningAborted を送出する。
        """
        mode = build_tuning_trajectory(profile)
        self._drive_complete.clear()
        # READY → PRE_CHECK → RUNNING。状態機械は PRE_CHECK を経由必須だが、走行前チェック
        # （PreCheckRunner）は実行しない（車両は停止保持済みのため）。
        self._transition(RobotState.PRE_CHECK)
        self._transition(RobotState.RUNNING)
        # mode_id=None: 規定パターンは永続化された DrivingMode ではない（drive_sessions.mode_id は
        # UUID カラムで "pid-tune" を渡すと DataError→500。学習運転と同様に None）。
        session = await self._begin_session("auto", None, log_writer, RobotState.RUNNING)
        self._build_and_start_drive_loop(
            mode,
            profile,
            log_writer,
            session.id,
            on_complete=self._stop_tuning_drive,
            disable_deviation_check=True,  # 適合中は逸脱で非常停止しない
        )
        timeout = mode.total_duration + _PID_TUNING_DRIVE_TIMEOUT_MARGIN_S
        try:
            await asyncio.wait_for(self._drive_complete.wait(), timeout)
        except TimeoutError as e:
            raise PidTuningAborted("PID 自動適合の走行がタイムアウトしました") from e
        if self._state == RobotState.EMERGENCY:
            raise PidTuningAborted("走行中に非常停止が発生したため PID 自動適合を中断しました")
        return self._last_kpi_summary or {}

    async def _stop_tuning_drive(self) -> None:
        """PID 最適化の 1 走行終了。原点復帰せず停車保持ブレーキで停止維持して READY へ。

        次の最適化走行へ停止状態のまま移行するため home_return しない。最適化セッション全体の
        完了時に呼び出し元（run_pid_validation / run_pid_tuning_session）が原点復帰で解放する。
        """
        drive_loop = self._drive_loop
        self._drive_loop = None
        if drive_loop is not None:
            await drive_loop.stop_and_join()
            self._record_kpi_summary(drive_loop)
        self._transition(RobotState.READY)
        try:
            if self._active_profile is not None:
                await self._apply_brake_hold(self._active_profile)
        finally:
            await self._close_session("completed")
            self._drive_complete.set()

    async def _release_after_tuning(self) -> None:
        """PID 最適化セッション終了時に停車保持ブレーキを解放（両軸原点復帰）する。"""
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )

    async def run_pid_validation(
        self, profile: VehicleProfile, log_writer: LogWriterProtocol | None = None
    ) -> dict[str, float]:
        """規定パターンを現在のプロファイルゲインで 1 回走行し KPI サマリを返す。"""
        self._assert_tuning_preconditions(profile)
        g = profile.pid_gains
        self._pid.set_gains(g.kp, g.ki, g.kd)
        try:
            return await self._run_tuning_drive(profile, log_writer)
        finally:
            await self._release_after_tuning()  # 停止保持を解放して終了

    async def run_pid_tuning_session(
        self,
        profile: VehicleProfile,
        log_writer: LogWriterProtocol | None = None,
        max_runs: int = 15,
    ) -> tuple[PIDGains, list[dict[str, float]]]:
        """規定パターンを反復走行し座標降下で KPI コストを最小化する。

        各走行は走行前チェックなしで停止保持状態から直接実行し、走行間も停止保持を維持する。
        セッション完了/中断時に停車保持ブレーキを解放（原点復帰）する。

        Returns:
            (最良ゲイン, 反復履歴) のタプル。履歴は各走行のゲイン・コスト・KPI を含む。
            最良ゲインは制御スタックへ反映済み（永続化は呼び出し元の責務）。
        """
        self._assert_tuning_preconditions(profile)
        tuner = CoordinateDescentTuner(profile.pid_gains, max_runs=max_runs)
        history: list[dict[str, float]] = []
        try:
            while (cand := tuner.next_candidate()) is not None:
                self._pid.set_gains(cand.kp, cand.ki, cand.kd)
                kpi = await self._run_tuning_drive(profile, log_writer)
                cost = tuning_cost(kpi)
                tuner.report(cand, cost)
                history.append(
                    {"kp": cand.kp, "ki": cand.ki, "kd": cand.kd, "cost": cost, **kpi}
                )
            best = tuner.best
            self._pid.set_gains(best.kp, best.ki, best.kd)  # 最良ゲインを制御スタックへ反映
        finally:
            await self._release_after_tuning()  # 停止保持を解放して終了（中断時も）
        return best, history

    def _assert_tuning_preconditions(self, profile: VehicleProfile) -> None:
        """PID 自動適合の前提（READY 状態・キャリブレーション済み）を検証する。"""
        if self._state != RobotState.READY:
            raise InvalidStateTransition(
                f"PID 自動適合は READY 状態でのみ開始できます (現在: {self._state})"
            )
        if profile.calibration is None:
            raise InvalidStateTransition(
                "PID 自動適合にはキャリブレーション済みプロファイルが必要です"
            )

    def _build_and_start_drive_loop(
        self,
        mode: DrivingMode | None,
        profile: VehicleProfile | None,
        log_writer: LogWriterProtocol | None,
        session_id: str,
        on_complete: Callable[[], Awaitable[None]] | None = None,
        disable_deviation_check: bool = False,
    ) -> None:
        """mode / profile / ff_controller / safety_check が揃っていれば DriveLoop を起動する。

        on_complete は走行正常終了時のコールバック。未指定なら自動走行の stop_auto_drive
        （原点復帰あり）。PID 最適化は停止保持を維持する _stop_tuning_drive を渡す。

        disable_deviation_check=True で逸脱による非常停止を無効化する（PID 自動適合用）。
        """
        if (
            mode is None
            or profile is None
            or profile.calibration is None
            or self._ff_controller is None
            or self._safety_check is None
        ):
            return
        self._drive_loop = DriveLoop(
            ff_controller=self._ff_controller,
            pid=self._pid,
            accel_driver=self._accel_driver,
            brake_driver=self._brake_driver,
            can_reader=self._can_reader,
            profile=profile,
            mode=mode,
            safety_check=self._safety_check,
            on_complete=on_complete or self.stop_auto_drive,
            on_emergency=self._dispatch_emergency,
            log_writer=log_writer,
            session_id=session_id,
            interval_s=self._control_interval_s,
            log_every_n_cycles=self._log_every_n_cycles,
            disable_deviation_check=disable_deviation_check,
        )
        self._drive_loop.start()

    async def start_auto_drive(
        self,
        mode_id: str,
        mode: DrivingMode | None = None,
        profile: VehicleProfile | None = None,
        log_writer: LogWriterProtocol | None = None,
    ) -> DriveSession:
        """自動走行開始。

        二経路に対応する:
          - PRE_CHECK（arm 済み・確認ポップアップ「はい」）: 走行前チェックは arm 済みのため、
            セッションを開始して RUNNING へ遷移するだけ（学習運転と同じ arm フロー）。
          - READY（arm なしの直接開始）: 従来どおり走行前チェックを実施して開始する（後方互換）。
        """
        if self._state == RobotState.PRE_CHECK:
            session = await self._begin_session("auto", mode_id, log_writer, RobotState.PRE_CHECK)
            self._transition(RobotState.RUNNING)
        else:
            await self._run_pre_check_and_transition(RobotState.RUNNING)
            session = await self._begin_session("auto", mode_id, log_writer, RobotState.RUNNING)
        self._build_and_start_drive_loop(mode, profile, log_writer, session.id)
        return session

    async def stop_auto_drive(self) -> None:
        """自動走行停止。RUNNING → READY。"""
        drive_loop = self._drive_loop
        self._drive_loop = None
        if drive_loop is not None:
            await drive_loop.stop_and_join()
            self._record_kpi_summary(drive_loop)
        self._transition(RobotState.READY)
        try:
            await asyncio.gather(
                self._accel_driver.home_return(),
                self._brake_driver.home_return(),
            )
        finally:
            await self._close_session("completed")
            self._drive_complete.set()  # PID 自動適合の走行完了待ちを起こす

    def _build_and_start_schedule_loop(
        self,
        schedule: TimeSchedule,
        profile: VehicleProfile | None,
        log_writer: LogWriterProtocol | None,
        session_id: str,
    ) -> None:
        """profile / calibration / safety_check が揃っていれば ScheduleLoop を起動する。

        ボタンサーボは任意（未構成でもペダル再生は動く。ボタンイベントは空撃ちで前進）。
        """
        if profile is None or profile.calibration is None or self._safety_check is None:
            return
        self._schedule_loop = ScheduleLoop(
            accel_driver=self._accel_driver,
            brake_driver=self._brake_driver,
            can_reader=self._can_reader,
            profile=profile,
            schedule=schedule,
            safety_check=self._safety_check,
            on_complete=self.stop_schedule_drive,
            on_emergency=self._dispatch_emergency,
            button_servo=self._button_servo,
            log_writer=log_writer,
            session_id=session_id,
            # 周期はモジュール定数 SCHEDULE_LOOP_INTERVAL_S(100ms=ログ周期)を単一ソースにする
            # （LearningLoop と同方針。制御ループ周期 control_interval_s とは独立）。
        )
        self._schedule_loop.start()

    async def start_schedule_drive(
        self,
        schedule: TimeSchedule,
        profile: VehicleProfile | None = None,
        log_writer: LogWriterProtocol | None = None,
    ) -> DriveSession:
        """タイムスケジュール走行開始。READY → PRE_CHECK → RUNNING。

        ペダル開度とボタンイベントの統合タイムラインを ScheduleLoop で開ループ実行し、
        走行ログを `drive_logs` に記録する（run_type='auto'、mode_id=None、ref_speed=None）。
        走行前チェックはボタンサーボ確認（項目8）を含めて実施する。
        """
        await self._run_pre_check_and_transition(
            RobotState.RUNNING, include_button_servo=bool(schedule.button_events)
        )
        session = await self._begin_session("auto", None, log_writer, RobotState.RUNNING)
        self._build_and_start_schedule_loop(schedule, profile, log_writer, session.id)
        return session

    async def stop_schedule_drive(self) -> None:
        """タイムスケジュール走行停止（on_complete / 手動停止）。RUNNING → READY。

        ループ停止 → 全ボタンサーボ待機位置へ → 両軸原点復帰 → セッション終了。
        """
        schedule_loop = self._schedule_loop
        self._schedule_loop = None
        if schedule_loop is not None:
            await schedule_loop.stop_and_join()
        self._transition(RobotState.READY)
        try:
            await self._release_button_servo()
            await asyncio.gather(
                self._accel_driver.home_return(),
                self._brake_driver.home_return(),
            )
        finally:
            await self._close_session("completed")

    async def pause_auto_drive(self) -> None:
        """自動走行を一時停止する。RUNNING → PAUSED。

        基準速度タイムラインを凍結し、一時停止した瞬間の目標車速を保持して走り続ける
        （DriveLoop.pause を参照）。RUNNING 以外、または DriveLoop が無い場合は拒否する。
        """
        if self._state != RobotState.RUNNING or self._drive_loop is None:
            raise InvalidStateTransition(
                f"pause_auto_drive は自動走行中(RUNNING)でのみ呼べます (現在: {self._state})"
            )
        self._drive_loop.pause()
        self._transition(RobotState.PAUSED)

    async def resume_auto_drive(self) -> None:
        """一時停止中の自動走行を再開する。PAUSED → RUNNING。

        凍結したタイムラインを続きから進める（DriveLoop.resume を参照）。
        PAUSED 以外、または DriveLoop が無い場合は拒否する。
        """
        if self._state != RobotState.PAUSED or self._drive_loop is None:
            raise InvalidStateTransition(
                f"resume_auto_drive は一時停止中(PAUSED)でのみ呼べます (現在: {self._state})"
            )
        self._drive_loop.resume()
        self._transition(RobotState.RUNNING)

    async def start_manual(
        self,
        log_writer: LogWriterProtocol | None = None,
    ) -> DriveSession:
        """手動操作開始。READY → PRE_CHECK → MANUAL。セッションは DB 利用時に永続化する。"""
        await self._run_pre_check_and_transition(RobotState.MANUAL)
        return await self._begin_session("manual", None, log_writer, RobotState.MANUAL)

    async def stop_manual(self) -> None:
        """手動操作終了。MANUAL → READY。MANUAL 状態以外から呼ぶと InvalidStateTransition。"""
        if self._state != RobotState.MANUAL:
            raise InvalidStateTransition(
                f"stop_manual は MANUAL 状態でのみ呼べます (現在: {self._state})"
            )
        self._transition(RobotState.READY)
        try:
            await asyncio.gather(
                self._accel_driver.home_return(),
                self._brake_driver.home_return(),
            )
        finally:
            await self._close_session("completed")

    async def arm_learning_drive(self) -> None:
        """学習運転の準備。READY → PRE_CHECK。

        共通の arm 処理（`_arm_drive`）に加えて学習マネージャの存在を必須化する。
        フロントは「はい」で start_learning_drive、「いいえ」で cancel_learning_drive を呼ぶ。
        """
        await self._arm_drive(require_learning_manager=True)

    async def arm_auto_drive(self) -> None:
        """自動走行の準備。READY → PRE_CHECK。

        学習運転の arm と同じ手順（停車保持ブレーキ踏込 → 車速0確認 → 走行前チェック）を実施する。
        フロントは「はい」で start_auto_drive、「いいえ」で cancel_auto_drive を呼ぶ。
        """
        await self._arm_drive(require_learning_manager=False)

    async def _arm_drive(self, *, require_learning_manager: bool) -> None:
        """自動走行・学習運転に共通の arm 処理。READY → PRE_CHECK。

        手順:
          1. 停車保持ブレーキ（`stop_brake_opening_pct`）まで踏み込む
          2. 車速が 0 に収束するまで待機（タイムアウトあり）
          3. 走行前チェック（車速0・通信・サーボ等）を実施
          4. 合格なら PRE_CHECK のまま待機（フロントが確認ポップアップを表示）

        車速判定はブレーキを踏んで停止させた後に行う（踏む前に車速が出ていても、踏んで
        止めてから判定する）。失敗時はブレーキを原点復帰して READY へロールバックする。
        """
        self._transition(RobotState.PRE_CHECK)
        try:
            profile = self._active_profile
            # 依存欠落は silent no-op せず明示的に弾く（ブレーキ換算に calibration が必要）
            if profile is None or profile.calibration is None:
                raise InvalidStateTransition("走行に必要な構成が不足しています")
            if require_learning_manager and self._learning_manager is None:
                raise InvalidStateTransition("学習運転に必要な構成が不足しています")
            runner = self._pre_check_runner
            # 1. 踏込前チェック: 車速確認のみ除外（この時点では車速が出ていてよい）。
            #    両ペダルが原点付近にあること等をここで確認する。
            if runner is not None:
                pre = await runner.run(exclude=frozenset({ITEM_VEHICLE_STOPPED}))
                if not pre.passed:
                    raise PreCheckFailed(pre)
            # 2. ブレーキを停車保持開度まで踏む → 3. 車速が 0 に収束するまで待機
            await self._apply_brake_hold(profile)
            await self._wait_for_vehicle_stopped()
            # 4. 踏込後チェック: アクチュエータ位置を除外（保持ブレーキで意図的に原点から離れる）。
            #    停止後に車速0・通信・サーボ等を判定する。
            if runner is not None:
                post = await runner.run(exclude=frozenset({ITEM_ACTUATOR_POSITION}))
                if not post.passed:
                    raise PreCheckFailed(post)
        except Exception:
            # 踏んだブレーキを解放してから READY へ戻す（home_return 失敗は握りつぶす）
            try:
                await asyncio.gather(
                    self._accel_driver.home_return(),
                    self._brake_driver.home_return(),
                )
            except Exception:
                _logger.exception("arm 失敗時のブレーキ原点復帰に失敗しました（処理は継続）")
            self._transition(RobotState.READY)
            raise

    async def _wait_for_vehicle_stopped(self) -> None:
        """車速が停車しきい値未満へ収束するまで待機する。

        ブレーキ踏込後に呼ぶ。タイムアウトしても例外は送出せず、後続の走行前チェック
        （車速確認）に NG 判定を委ねる。CAN 読取例外も同様に走行前チェック（通信確認）へ委ねる。
        """
        loop = asyncio.get_running_loop()
        deadline = loop.time() + _VEHICLE_STOP_TIMEOUT_S
        while loop.time() < deadline:
            try:
                speed = await self._can_reader.read_speed()
            except Exception:
                return  # 通信異常は走行前チェック（通信確認）で検出させる
            if abs(speed) < _VEHICLE_STOP_SPEED_KMH:
                return
            await asyncio.sleep(_VEHICLE_STOP_POLL_S)

    async def start_learning_drive(
        self,
        log_writer: LogWriterProtocol | None = None,
    ) -> DriveSession:
        """学習走行開始。arm 後（PRE_CHECK）→ RUNNING。

        開度パターン列を LearningLoop で開ループ実行し、100ms 周期で走行ログを
        `drive_logs` に記録する（run_type='learning'）。正常完了で
        on_complete=stop_learning_drive により RUNNING→READY 遷移しセッションを
        'completed' で終了する。停止/非常停止でも LearningLoop を止めセッションを終了する。
        """
        if self._state != RobotState.PRE_CHECK:
            raise InvalidStateTransition(
                f"start_learning_drive は arm 後（PRE_CHECK）にのみ呼べます (現在: {self._state})"
            )
        profile = self._active_profile
        if (
            profile is None
            or profile.calibration is None
            or self._learning_manager is None
            or self._safety_check is None
        ):
            self._transition(RobotState.READY)
            raise InvalidStateTransition("学習運転に必要な構成が不足しています")

        session = await self._begin_session("learning", None, log_writer, RobotState.PRE_CHECK)
        self._transition(RobotState.RUNNING)
        patterns = self._learning_manager.generate_patterns(profile)
        self._learning_loop = LearningLoop(
            accel_driver=self._accel_driver,
            brake_driver=self._brake_driver,
            can_reader=self._can_reader,
            profile=profile,
            patterns=patterns,
            safety_check=self._safety_check,
            on_complete=self.stop_learning_drive,
            on_emergency=self._dispatch_emergency,
            log_writer=log_writer,
            session_id=session.id,
        )
        self._learning_loop.start()
        return session

    async def cancel_learning_drive(self) -> None:
        """学習運転の中止（確認ポップアップ「いいえ」）。PRE_CHECK → READY。

        arm でかけた保持ブレーキをリリースする。arm 段階ではセッションを開始しないため、
        ログ・学習は発生しない。
        """
        await self._cancel_armed_drive()

    async def cancel_auto_drive(self) -> None:
        """自動走行の中止（確認ポップアップ「いいえ」）。PRE_CHECK → READY。

        arm でかけた保持ブレーキをリリースする。arm 段階ではセッションを開始しないため、
        ログは残らない。
        """
        await self._cancel_armed_drive()

    async def _cancel_armed_drive(self) -> None:
        """arm 済み（確認待ち）の走行を中止する共通処理。PRE_CHECK → READY。

        arm でかけた保持ブレーキを原点復帰でリリースする。arm 段階ではセッションを
        開始しないため、ログは残らない。
        """
        if self._state != RobotState.PRE_CHECK:
            raise InvalidStateTransition(
                f"cancel は arm 後（PRE_CHECK）にのみ呼べます (現在: {self._state})"
            )
        self._transition(RobotState.READY)
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )

    async def stop_learning_drive(self) -> None:
        """学習走行停止（on_complete / 手動停止）。RUNNING → READY。

        原点復帰せず**緩やかに減速・停止確認してから停車保持ブレーキで車両を停止保持**したまま
        READY へ遷移する。学習終了後はそのまま PID 最適化（規定パターン走行）へ移行する設計のため、
        車両を停止状態に保つ。学習最終パターンは惰行/減速の途中で終わり車両が転動したまま終わる
        ことがあるため、READY へ遷移する前に ~0.1G で減速して停止を確認する（転動したまま適合の
        規定パターンを走らせると追従誤差が大きく逸脱になる）。原点復帰するとシャシダイナモ上で
        車両がクリープし、最適化開始前に再停止・走行前チェックが必要になるため原点復帰はしない。
        停車保持ブレーキは最適化セッション完了時に解放される（_release_after_tuning）。
        """
        learning_loop = self._learning_loop
        self._learning_loop = None
        if learning_loop is not None:
            await learning_loop.stop_and_join()
        try:
            if self._active_profile is not None:
                await self._decelerate_to_stop(self._active_profile)
        finally:
            self._transition(RobotState.READY)
            await self._close_session("completed")

    async def _decelerate_to_stop(self, profile: VehicleProfile) -> None:
        """~0.1G の緩減速で車両を停止させ、停止を確認してから停車保持ブレーキへ移行する。

        閉ループでブレーキ開度を調整し、減速G を目標(~0.1G)付近に保ちながら車速が停車しきい値
        未満へ収束するまで待つ。停止確認後（またはタイムアウト時）は停車保持ブレーキ
        （`stop_brake_opening_pct`）を適用して終了する。CAN 読取例外時も停車保持ブレーキへ
        フォールバックし、停止判定は後続（PID 適合側）に委ねる。
        """
        calib = profile.calibration
        if calib is None:
            return
        loop = asyncio.get_running_loop()
        target_decel_kmhs = _BRIDGE_DECEL_G * _G_TO_KMHS
        max_brake = profile.max_brake_opening
        brake_pct = 0.0
        prev_speed: float | None = None
        prev_t: float | None = None
        deadline = loop.time() + _BRIDGE_DECEL_TIMEOUT_S
        while loop.time() < deadline:
            try:
                speed = await self._can_reader.read_speed()
            except Exception:
                break  # 通信異常は停車保持へフォールバック（後続の走行前判定に委ねる）
            if abs(speed) < _VEHICLE_STOP_SPEED_KMH:
                break  # 停止確認
            now = loop.time()
            if prev_speed is not None and prev_t is not None and now > prev_t:
                decel = (prev_speed - speed) / (now - prev_t)  # >0 が減速
                if decel < target_decel_kmhs:
                    brake_pct = min(max_brake, brake_pct + _BRIDGE_BRAKE_STEP_PCT)
                elif decel > target_decel_kmhs:
                    brake_pct = max(0.0, brake_pct - _BRIDGE_BRAKE_STEP_PCT)
            else:
                brake_pct = min(max_brake, brake_pct + _BRIDGE_BRAKE_STEP_PCT)  # 初動は踏み増し
            pos = calib.brake_zero_pos + round(
                (calib.brake_full_pos - calib.brake_zero_pos) * brake_pct / 100.0
            )
            await self._brake_driver.move_to_position(pos)
            prev_speed, prev_t = speed, now
            await asyncio.sleep(_BRIDGE_DECEL_POLL_S)
        await self._apply_brake_hold(profile)

    async def _apply_brake_hold(self, profile: VehicleProfile) -> None:
        """停車保持ブレーキ（`stop_brake_opening_pct`）までブレーキ軸を踏み込む。"""
        calib = profile.calibration
        if calib is None:
            return
        pct = max(
            0.0, min(profile.feedforward_params.stop_brake_opening_pct, profile.max_brake_opening)
        )
        pos = calib.brake_zero_pos + round(
            (calib.brake_full_pos - calib.brake_zero_pos) * pct / 100.0
        )
        await self._brake_driver.move_to_position(pos)

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
        # 下限: 原点(0)、上限: ハードウェア機械端(_ACTUATOR_PULSE_MAX)でクランプ。
        # MANUAL ではさらにキャリブレーション範囲内に制限する。
        target = max(0, min(_ACTUATOR_PULSE_MAX, current + step))
        if self._state == RobotState.MANUAL:
            target = self._clamp_to_calibrated_range(axis, target)
        await driver.move_to_position(target)
        # 移動完了を待ってから位置を読む（直後の read_position は移動途中値を返すため）
        await driver.wait_for_position_complete()
        return await driver.read_position()

    def _clamp_to_calibrated_range(self, axis: str, target: int) -> int:
        """キャリブレーション済みのゼロ/フル位置の範囲内に目標位置をクランプする。"""
        if self._active_profile is None or self._active_profile.calibration is None:
            return target
        calib = self._active_profile.calibration
        if axis == "accel":
            lo, hi = calib.accel_zero_pos, calib.accel_full_pos
        else:
            lo, hi = calib.brake_zero_pos, calib.brake_full_pos
        lo, hi = min(lo, hi), max(lo, hi)
        return max(lo, min(hi, target))

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
