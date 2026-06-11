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
from src.models.profile import StopConfig, VehicleProfile
from src.models.system_state import (
    InitStep,
    InitStepStatus,
    RealtimeSnapshot,
    RobotState,
    SystemState,
)

_logger = logging.getLogger(__name__)

# DriveLoop スナップショットの許容鮮度 [s]。制御サイクルが固まった際に凍結キャッシュを
# 「現在値」として WS 配信し続けると、操作者に正常なダッシュボードを見せたまま
# ロボットが無制御になる（コードレビュー 2026-06-11 指摘 #16）。
SNAPSHOT_MAX_AGE_S: float = 0.5


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
    RobotState.INITIALIZING: frozenset({RobotState.READY, RobotState.ERROR, RobotState.EMERGENCY}),
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
        self._drive_loop = None
        self._log_writer = None
        self._active_learning_task = None
        self._last_kpi_summary: dict[str, float] | None = None
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
    def current_openings(self) -> tuple[float, float]:
        """現在のアクセル・ブレーキ開度 [%] を返す。DriveLoop 非動作時は (0.0, 0.0)。"""
        if self._drive_loop is not None:
            return self._drive_loop.current_accel_opening, self._drive_loop.current_brake_opening
        return 0.0, 0.0

    @property
    def current_ref_speed(self) -> float | None:
        """現在の基準車速 [km/h]。DriveLoop 非動作時は None。WebSocket 配信で使う。"""
        if self._drive_loop is not None:
            return self._drive_loop.current_ref_speed
        return None

    async def get_realtime_data(self) -> RealtimeSnapshot:
        """リアルタイム計測値を返す。

        走行中（DriveLoop 動作中）は直近サイクルの計測値キャッシュを返し、
        50ms 制御ループが使う Modbus/CAN バスへ追加トランザクションを発行しない。
        キャッシュがサイクル凍結等で古くなった場合は障害を隠さないよう
        ハードウェア読み取りへフォールバックする。非走行時もハードウェアから並列取得する。
        """
        if self._drive_loop is not None:
            snapshot = self._drive_loop.last_snapshot
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
        """正常停止: 原点復帰 → サーボOFF → READY 遷移。RUNNING または MANUAL 状態でのみ呼べる。"""
        if self._state not in (RobotState.RUNNING, RobotState.MANUAL):
            raise InvalidStateTransition(
                f"stop は RUNNING または MANUAL 状態でのみ呼べます (現在: {self._state})"
            )
        drive_loop = self._drive_loop
        self._drive_loop = None
        if drive_loop is not None:
            # 飛行中の位置指令完了を待ってから home_return（バス上のインターリーブ防止）
            await drive_loop.stop_and_join()
            self._record_kpi_summary(drive_loop)
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
        """グレースフルシャットダウン: DriveLoop を停止し、ペダル解放とサーボOFFを試みる。

        プロセス終了後はソフトウェア側の監視・制御が一切残らないため、ここで
        ペダルを解放しないと最終指令位置（例: アクセル40%）のままサーボONで
        保持され続ける。サーボONになり得る状態ではベストエフォートで
        原点復帰 → サーボOFF を実施する（失敗してもシャットダウンは続行）。
        """
        drive_loop = self._drive_loop
        self._drive_loop = None
        if drive_loop is not None:
            await drive_loop.stop_and_join()
        if self._active_learning_task is not None:
            self._active_learning_task.cancel()
            self._active_learning_task = None
        if self._state in (
            RobotState.READY,
            RobotState.CALIBRATING,
            RobotState.PRE_CHECK,
            RobotState.RUNNING,
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
        drive_loop = self._drive_loop
        self._drive_loop = None
        if drive_loop is not None:
            drive_loop.stop()  # 同期で停止フラグ（以降のサイクルは書き込み前に中断）
        if self._state == RobotState.EMERGENCY:
            if drive_loop is not None:
                await drive_loop.stop_and_join()
            return
        self._transition(RobotState.EMERGENCY)
        if drive_loop is not None:
            await drive_loop.stop_and_join()
            self._record_kpi_summary(drive_loop)
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        await self._close_session("emergency")
        self._last_normal_shutdown = False

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

    async def _run_pre_check_and_transition(self, target: RobotState) -> None:
        """READY → PRE_CHECK → target の共通プロローグ。失敗時は READY へロールバックする。"""
        self._transition(RobotState.PRE_CHECK)
        try:
            if self._pre_check_runner is not None:
                result = await self._pre_check_runner.run()
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

    def _build_and_start_drive_loop(
        self,
        mode: DrivingMode | None,
        profile: VehicleProfile | None,
        log_writer: LogWriterProtocol | None,
        session_id: str,
    ) -> None:
        """mode / profile / ff_controller / safety_check が揃っていれば DriveLoop を起動する。"""
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
            on_complete=self.stop_auto_drive,
            on_emergency=self._dispatch_emergency,
            log_writer=log_writer,
            session_id=session_id,
            interval_s=self._control_interval_s,
            log_every_n_cycles=self._log_every_n_cycles,
        )
        self._drive_loop.start()

    async def start_auto_drive(
        self,
        mode_id: str,
        mode: DrivingMode | None = None,
        profile: VehicleProfile | None = None,
        log_writer: LogWriterProtocol | None = None,
    ) -> DriveSession:
        """自動走行開始。READY → PRE_CHECK → RUNNING。"""
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
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        await self._close_session("completed")

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
        await asyncio.gather(
            self._accel_driver.home_return(),
            self._brake_driver.home_return(),
        )
        await self._close_session("completed")

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
        await self._run_pre_check_and_transition(RobotState.RUNNING)
        session = await self._begin_session("learning", None, log_writer, RobotState.RUNNING)
        profile = self._active_profile
        if profile is not None and self._learning_manager is not None:
            learning_mode = self._learning_manager.build_learning_reference(profile)
            self._build_and_start_drive_loop(learning_mode, profile, log_writer, session.id)
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
        # 負方向は原点(0)でクランプ。MANUAL（走行ペダル操作）ではさらにキャリブレーション
        # 範囲内に制限し、フル位置を超えた機械端への押し込みを防ぐ。
        # CALIBRATING はゼロ/フル点を探索する工程のため上限クランプしない。
        target = max(0, current + step)
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
