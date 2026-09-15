"""アクチュエータ 1 軸の状態モニター（KAIZEN 表5-5 順6 A7）。

PCON-CB の監視レジスタは 0x9000〜0x9011 が連続しているので、FC03 で 0x9000 から 14 レジスタを
1 回で読めば、実位置 PNOW・アラームコード・デバイスステータス・電流 CNOW がまとめて取れる
（MODBUS 仕様書 MJ0162-12A 5.3 節。使用例 5.3.1〔5〕も 0x9000 からまとめて読んでいる）。

    オフセット  レジスタ      中身
    0, 1        0x9000–01     PNOW 現在位置 [0.01mm]（符号付き 32bit）
    2           0x9002        ALMC アラームコード（0 = アラームなし）
    5           0x9005        DSS1 ステータス1（bit12 SV サーボON / bit3 PEND 位置決め完了）
    7           0x9007        DSSE 拡張デバイスステータス（bit5 MOVE 移動中）
    12, 13      0x900C–0D     CNOW 電流 [mA]（符号付き 32bit）

ビット定義は `src/infra/actuator_driver.py` の定数と同じ値（src の private 名は import しない）。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

MONITOR_START_REGISTER = 0x9000
MONITOR_REGISTER_COUNT = 14

_OFFSET_PNOW = 0
_OFFSET_ALMC = 2
_OFFSET_DSS1 = 5
_OFFSET_DSSE = 7
_OFFSET_CNOW = 12

DSS1_SV = 1 << 12  # サーボON中
DSS1_PEND = 1 << 3  # 位置決め完了
DSSE_MOVE = 1 << 5  # 移動中


def _signed32(hi: int, lo: int) -> int:
    raw = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
    return raw - 0x100000000 if raw >= 0x80000000 else raw


@dataclass(frozen=True)
class AxisMonitor:
    """1 回のまとめ読みで取れた 1 軸の状態。"""

    position_pulse: int  # 実位置 PNOW [pulse = 0.01mm]
    current_ma: float  # 電流 CNOW [mA]
    alarm_code: int  # ALMC（0 = アラームなし）
    servo_on: bool
    moving: bool
    pos_done: bool

    @classmethod
    def from_registers(cls, registers: Sequence[int]) -> AxisMonitor:
        if len(registers) < MONITOR_REGISTER_COUNT:
            raise ValueError(
                f"モニターは {MONITOR_REGISTER_COUNT} レジスタ要ります（{len(registers)} 個）"
            )
        dss1 = registers[_OFFSET_DSS1]
        dsse = registers[_OFFSET_DSSE]
        return cls(
            position_pulse=_signed32(registers[_OFFSET_PNOW], registers[_OFFSET_PNOW + 1]),
            current_ma=float(_signed32(registers[_OFFSET_CNOW], registers[_OFFSET_CNOW + 1])),
            alarm_code=registers[_OFFSET_ALMC] & 0xFFFF,
            servo_on=bool(dss1 & DSS1_SV),
            moving=bool(dsse & DSSE_MOVE),
            pos_done=bool(dss1 & DSS1_PEND),
        )


__all__ = [
    "DSS1_PEND",
    "DSS1_SV",
    "DSSE_MOVE",
    "MONITOR_REGISTER_COUNT",
    "MONITOR_START_REGISTER",
    "AxisMonitor",
]
