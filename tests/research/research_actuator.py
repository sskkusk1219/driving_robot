"""研究用の実機アクチュエータドライバ（KAIZEN 表5-5 順6 A7）。

本番 `src/infra/actuator_driver.py::ActuatorDriver` を継承し、src は変えずに次の 2 つだけ足す:
    - `read_monitor()` … 0x9000 から 14 レジスタをまとめ読みし、実位置・電流・ステータスを返す
    - `last_command_pos` … 最後に送った目標位置 [pulse]（ログの「指令」列に使う）

pymodbus を要するので、`hardware.build_hardware` の実機側でだけ遅延 import する。
"""

from __future__ import annotations

from src.infra.actuator_driver import ActuatorDriver
from tests.research.axis_monitor import (
    MONITOR_REGISTER_COUNT,
    MONITOR_START_REGISTER,
    AxisMonitor,
)


class ResearchActuatorDriver(ActuatorDriver):
    """まとめ読みと指令位置の記憶を足した ActuatorDriver。"""

    last_command_pos: int | None = None

    async def home_return(self) -> None:
        await super().home_return()
        self.last_command_pos = 0

    async def move_to_position(
        self,
        pos: int,
        speed_mm_s: float | None = None,
        accel: int | None = None,
        *,
        smooth_over_s: float | None = None,
    ) -> None:
        # move_to_position_timed もここを通る（本番の実装が move_to_position に委ねている）
        if speed_mm_s is None:
            await super().move_to_position(pos, accel=accel, smooth_over_s=smooth_over_s)
        else:
            await super().move_to_position(
                pos, speed_mm_s=speed_mm_s, accel=accel, smooth_over_s=smooth_over_s
            )
        self.last_command_pos = pos

    async def read_monitor(self) -> AxisMonitor:
        client = self._require_client()
        result = await self._execute(
            "read:MONITOR",
            lambda: client.read_holding_registers(
                address=MONITOR_START_REGISTER,
                count=MONITOR_REGISTER_COUNT,
                device_id=self._slave_id,
            ),
        )
        if result.isError():
            raise OSError(f"read_monitor 失敗: slave_id={self._slave_id}")
        return AxisMonitor.from_registers(result.registers)


__all__ = ["ResearchActuatorDriver"]
