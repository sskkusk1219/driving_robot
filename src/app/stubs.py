"""実機なしで起動するためのスタブコンポーネント群。開発・テスト用。"""

from src.app.robot_controller import RobotController
from src.domain.control.pid import PIDController


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
