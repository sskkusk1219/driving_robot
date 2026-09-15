"""アクチュエータ脱落の安全網（手順 2 のパターン走行・手順 3 のモード走行で共通）。

2026-09-13 の手順 3 でブレーキ軸の電流が 0 mA のまま脱落し、検知できなかった件の対策。
もとは mode_drive.py の中にあった判定を、pattern_loop.py からも使えるように切り出した。

    アラーム確認   … ALARM_CHECK_INTERVAL_S ごとに両軸のアラームを読む
                     （Modbus を毎周期は取り合わない）
    電流ゼロの継続 … 指令位置 > 0 なのに電流 0 mA が ZERO_CURRENT_ABORT_S 続いたら異常。
                     走行中に一度でも電流 > 0 を返した軸だけ有効
                     （スタブは常に 0 mA なので誤検知しない）

過電流・最高速・偏差の判定は呼び出し側に残す（走行の種類で扱いが違うため）。
"""

from __future__ import annotations

import asyncio
from typing import Protocol

ALARM_CHECK_INTERVAL_S = 1.0  # 両軸のアラームを確認する間隔 [s]
ZERO_CURRENT_ABORT_S = 1.0  # 指令位置>0 なのに電流 0mA がこの秒数続いたら中断 [s]


class AlarmReader(Protocol):
    async def is_alarm_active(self) -> bool: ...


class AxisSafetyNet:
    """アラーム確認と電流ゼロ継続の判定。1 周期に 1 回 `poll_alarms` → `check` の順で呼ぶ。"""

    def __init__(self, interval_s: float) -> None:
        self.interval_s = interval_s
        self.alarm_check_every_cycles = max(1, round(ALARM_CHECK_INTERVAL_S / interval_s))
        self.zero_current_abort_cycles = max(1, round(ZERO_CURRENT_ABORT_S / interval_s))
        self.last_alarm_accel: bool | None = None
        self.last_alarm_brake: bool | None = None
        self._accel_ever_current = False  # この軸が走行中に一度でも電流>0 を返したか
        self._brake_ever_current = False
        self._accel_zero_run = 0  # 指令位置>0 かつ電流0mA が連続した周期数
        self._brake_zero_run = 0

    async def poll_alarms(
        self, cycle: int, accel: AlarmReader, brake: AlarmReader
    ) -> tuple[bool | None, bool | None]:
        """`cycle` が確認の周期なら両軸のアラームを読む。確認しない周期・確認に失敗した周期は

        前回の値を返す（通信断そのものは呼び出し側のアクチュエータ通信の except で扱う）。
        """
        if cycle % self.alarm_check_every_cycles != 0:
            return self.last_alarm_accel, self.last_alarm_brake
        try:
            alarm_accel, alarm_brake = await asyncio.gather(
                accel.is_alarm_active(), brake.is_alarm_active()
            )
        except Exception:
            return self.last_alarm_accel, self.last_alarm_brake
        self.last_alarm_accel, self.last_alarm_brake = alarm_accel, alarm_brake
        return alarm_accel, alarm_brake

    def check(
        self,
        *,
        t: float,
        accel_pos: int,
        brake_pos: int,
        accel_current: float,
        brake_current: float,
        alarm_accel: bool | None,
        alarm_brake: bool | None,
    ) -> str | None:
        """異常があれば中断理由の文言を、無ければ None を返す。"""
        for label, alarm in (("アクセル", alarm_accel), ("ブレーキ", alarm_brake)):
            if alarm:
                return f"{label}軸にアラームが発生しました（t={t:.1f}s）"
        if accel_current > 0.0:
            self._accel_ever_current = True
        self._accel_zero_run = (
            self._accel_zero_run + 1
            if accel_pos > 0 and accel_current == 0.0 and self._accel_ever_current
            else 0
        )
        if self._accel_zero_run >= self.zero_current_abort_cycles:
            return (
                f"アクセル軸の電流が {self.zero_current_abort_cycles * self.interval_s:g}s"
                f" 0mA のままです（指令位置 {accel_pos}、t={t:.1f}s）"
            )
        if brake_current > 0.0:
            self._brake_ever_current = True
        self._brake_zero_run = (
            self._brake_zero_run + 1
            if brake_pos > 0 and brake_current == 0.0 and self._brake_ever_current
            else 0
        )
        if self._brake_zero_run >= self.zero_current_abort_cycles:
            return (
                f"ブレーキ軸の電流が {self.zero_current_abort_cycles * self.interval_s:g}s"
                f" 0mA のままです（指令位置 {brake_pos}、t={t:.1f}s）"
            )
        return None


__all__ = ["ALARM_CHECK_INTERVAL_S", "ZERO_CURRENT_ABORT_S", "AlarmReader", "AxisSafetyNet"]
