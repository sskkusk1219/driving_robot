"""本番環境向け RobotController ファクトリ。

実ハードウェア（ActuatorDriver / CANReader / GPIOMonitor）を組み合わせた
コントローラーを生成する。
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable

from src.app.robot_controller import RobotController, SafetyMonitorProtocol
from src.domain.control.pid import PIDController
from src.domain.safety_monitor import SafetyMonitor
from src.infra.actuator_driver import ActuatorDriver
from src.infra.can_reader import CANReader
from src.infra.gpio_monitor import GPIOMonitor
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
        gpio.register_emergency_callback(monitor.trigger_emergency)
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


def build_real_controller(settings: AppSettings) -> RobotController:
    """実ハードウェアに接続する RobotController を生成する。

    ポート・スレーブID・GPIO ピン番号はすべて settings から取得する。
    DBC ファイルは dbc_path=None（未指定）で動作するが、実測車速取得には
    config/can/ 以下に DBC ファイルを配置して CANReader に渡すこと。

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
        slave_id=2,
        baud_rate=settings.serial.baud_rate,
    )
    can_reader = CANReader(
        interface=settings.can.interface,
        channel=settings.can.channel,
    )
    gpio_monitor = GPIOMonitor(
        emergency_pin=settings.gpio.emergency_stop_pin,
        ac_detect_pin=settings.gpio.ac_detect_pin,
    )
    stop_config = StopConfig(
        deviation_threshold_kmh=2.0,
        deviation_duration_s=4.0,
    )
    safety_monitor = SafetyMonitor(stop_config=stop_config)
    safety_adapter: SafetyMonitorProtocol = _GpioSafetyAdapter(safety_monitor, gpio_monitor)

    pid = PIDController(kp=1.0, ki=0.0, kd=0.0)

    return RobotController(
        accel_driver=accel_driver,
        brake_driver=brake_driver,
        can_reader=can_reader,
        safety_monitor=safety_adapter,
        pid=pid,
        last_normal_shutdown=False,
    )
