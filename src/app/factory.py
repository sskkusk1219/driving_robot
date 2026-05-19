"""本番環境向け RobotController ファクトリ。

実ハードウェア（ActuatorDriver / CANReader / GPIOMonitor）を組み合わせた
コントローラーを生成する。
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable

from src.app.robot_controller import RobotController, SafetyMonitorProtocol
from src.domain.calibration import CalibrationManager
from src.domain.control.pid import PIDController
from src.domain.safety_monitor import SafetyMonitor
from src.infra.actuator_driver import ActuatorDriver
from src.infra.can_reader import CANReader
from src.infra.db import create_pool
from src.infra.gpio_monitor import GPIOMonitor
from src.infra.profile_repository import ProfileRepository
from src.infra.settings import AppSettings
from src.models.profile import StopConfig


class _GpioSafetyAdapter:
    """SafetyMonitor + GPIOMonitor をまとめて SafetyMonitorProtocol に適合させる。

    ドメイン層の SafetyMonitor はインフラ依存を持たないため、
    アプリケーション層でこのアダプターを通して GPIO を統合する。
    """

    def __init__(self, monitor: SafetyMonitor, gpio: GPIOMonitor) -> None:
        self._monitor = monitor
        self._gpio = gpio
        # AC断のみここで登録。非常停止は build_real_controller でコントローラ生成後に登録する
        gpio.register_ac_loss_callback(monitor.handle_ac_power_loss)

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


async def build_real_controller(settings: AppSettings) -> RobotController:
    """実ハードウェアに接続する RobotController を生成する。

    ポート・スレーブID・GPIO ピン番号はすべて settings から取得する。
    DB 接続プールを確立し、ProfileRepository を CalibrationManager に注入する。

    Args:
        settings: config/settings.toml から読み込んだ AppSettings。

    Returns:
        接続前の RobotController（start() で実 HW に接続する）。
    """
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
        dbc_path=settings.can.dbc_path,
    )
    gpio_monitor = GPIOMonitor(
        emergency_pin=settings.gpio.emergency_stop_pin,
        ac_detect_pin=settings.gpio.ac_detect_pin,
    )
    stop_config = StopConfig(
        deviation_threshold_kmh=2.0,
        deviation_duration_s=4.0,
    )
    safety_monitor = SafetyMonitor(
        stop_config=stop_config,
        overcurrent_limit_ma=settings.safety.overcurrent_limit_ma,
    )
    safety_adapter: SafetyMonitorProtocol = _GpioSafetyAdapter(safety_monitor, gpio_monitor)

    pool = await create_pool(settings.database.dsn)
    profile_repo = ProfileRepository(pool)

    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
    calibration_manager = CalibrationManager(
        accel_driver=accel_driver,
        brake_driver=brake_driver,
        profile_repo=profile_repo,
    )

    controller = RobotController(
        accel_driver=accel_driver,
        brake_driver=brake_driver,
        can_reader=can_reader,
        safety_monitor=safety_adapter,
        pid=pid,
        last_normal_shutdown=False,
        safety_check=safety_monitor,
        calibration_manager=calibration_manager,
    )
    # GPIO非常停止 → controller.emergency_stop() を直接呼ぶ
    # （trigger_emergency経由にすると controller が自身を再帰呼び出しするループが生じるため）
    gpio_monitor.register_emergency_callback(controller.emergency_stop)
    return controller
