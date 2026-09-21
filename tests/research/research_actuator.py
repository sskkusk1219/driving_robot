"""研究用の実機アクチュエータドライバ（KAIZEN 表5-5 順6 A7）。

本番 `src/infra/actuator_driver.py::ActuatorDriver` を継承し、src は変えずに次を足す:
    - `read_monitor()` … 0x9000 から 14 レジスタをまとめ読みし、実位置・電流・ステータスを返す
    - `last_command_pos` … 最後に送った目標位置 [pulse]（ログの「指令」列に使う）
    - `_last_monitor` / `_acmd_for_move()` … 移動中の残距離から ACMD（加減速指令）を
      決め直す（段2a: 0A7「指令減速度異常」対策）。

pymodbus を要するので、`hardware.build_hardware` の実機側でだけ遅延 import する。

### 0A7 対策の背景

親 `ActuatorDriver.move_to_position` は ACMD を「前回送った指令位置との差」だけで
決める（`acmd_for_move`）。軸が静止していればそれで正しいが、移動が終わる前に
前回とほぼ同じ位置を再指令すると、その「差」がほぼ 0 になり ACMD が下限（0.01G）
まで落ちる。コントローラはこれを「移動中に減速度を大きく下げられた」と解釈し、
今の減速度では残距離内に止まれないと判定すると 0A7 を出す（実ログ
`drive_log_real_20260918_041125.csv` mode_time 136.60→136.65s、28.420%→17.099%
の移動が終わる前に 17.101%（前回との差 0.002%）を指令し、ACMD が 1 に落ちてアラーム
発生）。ここでは「前回指令との差」と「実位置から見た残距離」の大きい方を使うことで、
移動中の微小な再指令でも ACMD が下がりすぎないようにする。
"""

from __future__ import annotations

from src.infra.actuator_driver import ActuatorDriver, acmd_for_move
from tests.research.axis_monitor import (
    MONITOR_REGISTER_COUNT,
    MONITOR_START_REGISTER,
    AxisMonitor,
)


class ResearchActuatorDriver(ActuatorDriver):
    """まとめ読みと指令位置の記憶を足した ActuatorDriver。"""

    last_command_pos: int | None = None
    # 直近に read_monitor() で読んだ AxisMonitor（ACMD の残距離計算に使う）。
    _last_monitor: AxisMonitor | None = None

    async def home_return(self) -> None:
        await super().home_return()
        self.last_command_pos = 0
        self._last_monitor = None

    def _acmd_for_move(self, pos: int, smooth_over_s: float) -> int | None:
        """前回の移動がまだ終わっていなければ、実位置から目標までの残距離で ACMD を決める。

        親（src）は「前回指令との差」で ACMD を出す。軸が静止していればそれで正しいが、
        移動中に微小変化の再指令が来ると ACMD が下限（0.01G）に落ち、コントローラが
        「その減速度では止まれない」と判断して 0A7（指令減速度異常）を出す。

        設計上の要点:
            - 静止時の挙動は変えない。`moving` / `not pos_done` のときだけ残距離を使う。
              微小移動を周期いっぱいかけて滑らかに動かす元の設計（`acmd_for_move` の
              docstring）はそのまま。
            - `max()` を使うので ACMD は下がる方向には変わらない。アラームを避ける
              方向にしか働かない。
            - モニタは前周期の読み（約 48ms 前）なので、実際には移動が終わっているのに
              `moving` が立っている周期がありうる。そのとき残距離は位置決め完了幅
              （INP = 10 pulse = 0.1mm）ぶんだけ大きく出るが、
              `acmd_for_move(0.1mm, 0.04s)` = 3（0.03G）で、影響は微小移動がわずかに
              速くなる程度にとどまる。
        """
        last = self.last_command_pos
        if last is None:
            return None                      # 親の既定（_DEFAULT_ACCEL）に任せる
        distance_mm = abs(pos - last) / 100.0
        monitor = self._last_monitor
        if monitor is not None and (monitor.moving or not monitor.pos_done):
            remaining_mm = abs(pos - monitor.position_pulse) / 100.0
            distance_mm = max(distance_mm, remaining_mm)
        return acmd_for_move(distance_mm, smooth_over_s)

    async def move_to_position(
        self,
        pos: int,
        speed_mm_s: float | None = None,
        accel: int | None = None,
        *,
        smooth_over_s: float | None = None,
    ) -> None:
        # move_to_position_timed もここを通る（本番の実装が move_to_position に委ねている）
        if accel is None and smooth_over_s is not None:
            accel = self._acmd_for_move(pos, smooth_over_s)
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
        monitor = AxisMonitor.from_registers(result.registers)
        self._last_monitor = monitor
        return monitor


__all__ = ["ResearchActuatorDriver"]
