"""本番環境向け RobotController ファクトリ。

実ハードウェア（ActuatorDriver / CANReader / GPIOMonitor / NutUPSMonitor）を組み合わせた
コントローラーを生成する。
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable

from src.app.robot_controller import (
    ActuatorDriverProtocol,
    CANReaderProtocol,
    RobotController,
    SafetyMonitorProtocol,
)
from src.domain.calibration import CalibrationManager
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pid import PIDController
from src.domain.learning_drive import LearningDriveManager
from src.domain.pre_check import PreCheckRunner
from src.domain.safety_monitor import SafetyMonitor
from src.infra.actuator_driver import ActuatorDriver
from src.infra.can_reader import CANReader
from src.infra.db import create_pool
from src.infra.gpio_monitor import GPIOMonitor
from src.infra.profile_repository import ProfileRepository
from src.infra.settings import AppSettings
from src.infra.ups_monitor import NutUPSMonitor
from src.models.profile import StopConfig


class _GpioSafetyAdapter:
    """SafetyMonitor + GPIOMonitor をまとめて SafetyMonitorProtocol に適合させる。

    ドメイン層の SafetyMonitor はインフラ依存を持たないため、
    アプリケーション層でこのアダプターを通して GPIO を統合する。
    AC断は NutUPSMonitor が担当するため、GPIO には非常停止のみ登録する。
    """

    def __init__(self, monitor: SafetyMonitor, gpio: GPIOMonitor) -> None:
        self._monitor = monitor
        self._gpio = gpio

    async def start_monitoring(self) -> None:
        await self._monitor.start_monitoring()
        await self._gpio.start_monitoring()

    async def stop_monitoring(self) -> None:
        self._gpio.stop_monitoring()
        await self._monitor.stop_monitoring()

    def register_emergency_callback(self, cb: Callable[[], Awaitable[None]]) -> None:
        self._monitor.register_emergency_callback(cb)

    async def trigger_emergency(self) -> None:
        await self._monitor.trigger_emergency()

    def is_emergency_active(self) -> bool:
        return self._gpio.is_emergency_active()


async def build_real_controller(
    settings: AppSettings,
    *,
    bench_gpio_only: bool = False,
) -> tuple[RobotController, NutUPSMonitor]:
    """実ハードウェアに接続する RobotController と NutUPSMonitor を生成する。

    ポート・スレーブID・GPIO ピン番号はすべて settings から取得する。
    DB 接続プールを確立し、ProfileRepository を CalibrationManager に注入する。
    NutUPSMonitor は呼び出し元（app.py lifespan）で start_polling() を呼ぶこと。

    Args:
        settings: config/settings.toml から読み込んだ AppSettings。
        bench_gpio_only: True の場合、非常停止スイッチ(GPIO)のみ実機とし、
            アクチュエータ・CAN はスタブに置き換える。アクチュエータ/CAN 未接続の
            ベンチ環境で非常停止スイッチの動作を検証するためのモード。

    Returns:
        (RobotController, NutUPSMonitor) のタプル。
        RobotController は start() で実 HW に接続する。
    """
    accel_driver: ActuatorDriverProtocol
    brake_driver: ActuatorDriverProtocol
    can_reader: CANReaderProtocol
    if bench_gpio_only:
        # ベンチ検証用: 非常停止スイッチ(GPIO)のみ実機。アクチュエータ/CAN はスタブ。
        from src.app.stubs import _StubActuator, _StubCANReader  # noqa: PLC0415

        accel_driver = _StubActuator()
        brake_driver = _StubActuator()
        can_reader = _StubCANReader()
    else:
        accel_driver = ActuatorDriver(
            port=settings.serial.accel_port,
            slave_id=1,
            baud_rate=settings.serial.baud_rate,
        )
        brake_driver = ActuatorDriver(
            port=settings.serial.brake_port,
            slave_id=1,  # 各軸が独立した RS-485 バスを持つため両軸とも slave_id=1
            baud_rate=settings.serial.baud_rate,
        )
        can_reader = CANReader(
            interface=settings.can.interface,
            channel=settings.can.channel,
            bitrate=settings.can.bitrate,
            dbc_path=settings.can.dbc_path,
            max_speed_age_s=settings.can.max_speed_age_s,
        )
    # 非常停止のみ GPIO 経由。AC断は NUT ポーリングに変更したため ac_detect_pin 未使用
    gpio_monitor = GPIOMonitor(
        emergency_pin=settings.gpio.emergency_stop_pin,
        ac_detect_pin=settings.gpio.ac_detect_pin,
    )
    # プロファイル未選択時の暫定値。select_profile() がプロファイルの stop_config で上書きし、
    # DriveLoop が参照する profile.stop_config と判定基準を一致させる
    stop_config = StopConfig(
        deviation_threshold_kmh=2.0,
        deviation_duration_s=4.0,
    )
    safety_monitor = SafetyMonitor(
        stop_config=stop_config,
        overcurrent_limit_ma=settings.safety.overcurrent_limit_ma,
    )
    safety_adapter: SafetyMonitorProtocol = _GpioSafetyAdapter(safety_monitor, gpio_monitor)

    ups_monitor = NutUPSMonitor(
        nut_host=settings.ups.nut_host,
        nut_port=settings.ups.nut_port,
        ups_name=settings.ups.ups_name,
        poll_interval_s=settings.ups.poll_interval_s,
    )
    # AC断は NUT ポーリングで検知し、SafetyMonitor のシーケンスを呼ぶ
    ups_monitor.register_ac_loss_callback(safety_monitor.handle_ac_power_loss)

    pool = await create_pool(settings.database.dsn)
    profile_repo = ProfileRepository(pool)

    # 周期は settings を単一ソースとし、PID の dt と DriveLoop の周期を一致させる
    loop_interval_s = settings.control.loop_interval_ms / 1000.0
    log_every_n_cycles = max(
        1, round(settings.control.log_interval_ms / settings.control.loop_interval_ms)
    )
    # PID ゲインはプロファイル未選択時の暫定値。select_profile() がプロファイル値で上書きする
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, dt=loop_interval_s)
    ff_controller = FeedforwardController()
    calibration_manager = CalibrationManager(
        accel_driver=accel_driver,
        brake_driver=brake_driver,
        profile_repo=profile_repo,
    )
    pre_check_runner = PreCheckRunner(
        accel_driver=accel_driver,
        brake_driver=brake_driver,
        can_reader=can_reader,
        ups_monitor=ups_monitor,
        profile=None,  # RobotController が select_profile で更新する
    )

    controller = RobotController(
        accel_driver=accel_driver,
        brake_driver=brake_driver,
        can_reader=can_reader,
        safety_monitor=safety_adapter,
        pid=pid,
        ff_controller=ff_controller,
        last_normal_shutdown=False,
        safety_check=safety_monitor,
        calibration_manager=calibration_manager,
        pre_check_runner=pre_check_runner,
        learning_manager=LearningDriveManager(),
        control_interval_s=loop_interval_s,
        log_every_n_cycles=log_every_n_cycles,
    )
    # 非常停止は SafetyMonitor を単一ディスパッチャとする:
    #   GPIO / AC断(NUT) → SafetyMonitor.trigger_emergency → controller.emergency_stop
    # controller.emergency_stop は trigger_emergency を呼び返さない（一方向ディスパッチ）。
    safety_monitor.register_emergency_callback(controller.emergency_stop)
    gpio_monitor.register_emergency_callback(safety_monitor.trigger_emergency)
    return controller, ups_monitor
