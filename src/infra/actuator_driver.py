"""IAI P-CON-CB 用 Modbus RTU アクチュエータドライバ。

MJ0162-12A（Modbus 仕様書 第12版）に基づく実装。
"""

from __future__ import annotations

import asyncio
import logging

from pymodbus.client import AsyncModbusSerialClient
from pymodbus.framer import FramerType

logger = logging.getLogger(__name__)

# FC03 読み取りレジスタアドレス（HEX）
_REG_PNOW_HI = 0x9000  # 現在位置 上位 16bit
_REG_ALMC = 0x9002      # アラームコード
_REG_DSS1 = 0x9005      # デバイスステータス1
_REG_DSSE = 0x9007      # 拡張デバイスステータス
_REG_CNOW_HI = 0x900C   # 電流値 上位 16bit

# DSS1 ビット定義
_DSS1_SV = 1 << 12    # サーボON中
_DSS1_ALMH = 1 << 10  # アラームあり
_DSS1_HEND = 1 << 4   # 原点復帰完了
_DSS1_PEND = 1 << 3   # 位置決め完了

# DSSE ビット定義
_DSSE_MOVE = 1 << 5  # 移動中

# FC05 コイルアドレス（HEX）
_COIL_SON = 0x0403   # サーボON
_COIL_ALRS = 0x0407  # アラームリセット
_COIL_HOME = 0x040B  # 原点復帰

# FC10 書き込みレジスタアドレス（HEX）
_REG_PCMD_HI = 0x9900  # 目標位置 上位 16bit
_REG_VCMD_HI = 0x9904  # 速度指令 上位 16bit
_REG_ACMD = 0x9906     # 加減速指令
_REG_CTLF = 0x9908     # 制御フラグ

# デフォルト移動パラメータ
_DEFAULT_SPEED_MM_S = 50       # 速度 [mm/s]
_DEFAULT_ACCEL_MM_S2 = 1000    # 加減速 [mm/s²]

_HOME_RETURN_TIMEOUT_S = 30.0
_HOME_RETURN_POLL_INTERVAL_S = 0.1


def _to_signed32(hi: int, lo: int) -> int:
    """上位・下位 16bit ワードから符号付き 32bit 整数を生成する。"""
    raw = (hi << 16) | (lo & 0xFFFF)
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw


def _from_signed32(value: int) -> tuple[int, int]:
    """符号付き 32bit 整数を上位・下位 16bit ワードのタプルに分解する。"""
    unsigned = value & 0xFFFFFFFF
    hi = (unsigned >> 16) & 0xFFFF
    lo = unsigned & 0xFFFF
    return hi, lo


class ActuatorDriver:
    """IAI P-CON-CB 用 Modbus RTU 非同期ドライバ。

    1インスタンスが1軸（アクセルまたはブレーキ）に対応する。
    connect() を呼んでから各操作メソッドを使用すること。
    """

    def __init__(
        self,
        port: str,
        slave_id: int,
        baud_rate: int = 38400,
    ) -> None:
        self._port = port
        self._slave_id = slave_id
        self._baud_rate = baud_rate
        self._client: AsyncModbusSerialClient | None = None

    async def connect(self) -> None:
        """Modbus RTU 接続を確立する。"""
        self._client = AsyncModbusSerialClient(
            port=self._port,
            baudrate=self._baud_rate,
            bytesize=8,
            parity="N",
            stopbits=1,
            framer=FramerType.RTU,
        )
        connected = await self._client.connect()
        if not connected:
            raise ConnectionError(f"Modbus RTU 接続失敗: port={self._port}")
        logger.info("ActuatorDriver 接続完了: port=%s slave_id=%d", self._port, self._slave_id)

    def _require_client(self) -> AsyncModbusSerialClient:
        """接続済みクライアントを返す。未接続なら RuntimeError。"""
        if self._client is None:
            raise RuntimeError("connect() を先に呼んでください。")
        return self._client

    async def close(self) -> None:
        """接続を閉じる。"""
        if self._client is not None:
            self._client.close()
            self._client = None
        logger.info("ActuatorDriver 切断: port=%s", self._port)

    async def reset_alarm(self) -> None:
        """アラームをリセットする（ALRS コイルをエッジ入力）。"""
        client = self._require_client()
        await client.write_coil(address=_COIL_ALRS, value=True, device_id=self._slave_id)
        await asyncio.sleep(0.05)
        await client.write_coil(address=_COIL_ALRS, value=False, device_id=self._slave_id)
        logger.debug("reset_alarm 完了: slave_id=%d", self._slave_id)

    async def servo_on(self) -> None:
        """サーボをONにする。"""
        client = self._require_client()
        await client.write_coil(address=_COIL_SON, value=True, device_id=self._slave_id)
        logger.debug("servo_on: slave_id=%d", self._slave_id)

    async def servo_off(self) -> None:
        """サーボをOFFにする。"""
        client = self._require_client()
        await client.write_coil(address=_COIL_SON, value=False, device_id=self._slave_id)
        logger.debug("servo_off: slave_id=%d", self._slave_id)

    async def home_return(self) -> None:
        """原点復帰を実行する。DSS1 HEND ビットが立つまでポーリング。

        タイムアウト (_HOME_RETURN_TIMEOUT_S) を超えた場合は TimeoutError を送出。
        """
        client = self._require_client()
        await client.write_coil(address=_COIL_HOME, value=True, device_id=self._slave_id)
        logger.info("home_return 開始: slave_id=%d", self._slave_id)

        deadline = asyncio.get_event_loop().time() + _HOME_RETURN_TIMEOUT_S
        while asyncio.get_event_loop().time() < deadline:
            await asyncio.sleep(_HOME_RETURN_POLL_INTERVAL_S)
            result = await client.read_holding_registers(
                address=_REG_DSS1, count=1, device_id=self._slave_id
            )
            if result.isError():
                logger.warning("home_return DSS1 読み取りエラー: slave_id=%d", self._slave_id)
                continue
            dss1 = result.registers[0]
            if dss1 & _DSS1_HEND:
                logger.info("home_return 完了: slave_id=%d", self._slave_id)
                return

        raise TimeoutError(f"home_return タイムアウト: slave_id={self._slave_id}")

    async def move_to_position(
        self,
        pos: int,
        speed_mm_s: int = _DEFAULT_SPEED_MM_S,
        accel_mm_s2: int = _DEFAULT_ACCEL_MM_S2,
    ) -> None:
        """指定位置へ移動指令を送出する（FC10 直値移動）。

        Args:
            pos: 目標位置 [pulse / 0.01mm 単位]
            speed_mm_s: 移動速度 [mm/s]
            accel_mm_s2: 加減速 [mm/s²]
        """
        client = self._require_client()
        pcmd_hi, pcmd_lo = _from_signed32(pos)
        vcmd_hi, vcmd_lo = _from_signed32(speed_mm_s)

        # 9900: PCMD_HI, 9901: PCMD_LO, 9902-9903: 未使用（0）,
        # 9904: VCMD_HI, 9905: VCMD_LO, 9906: ACMD, 9907: 未使用, 9908: CTLF
        registers = [
            pcmd_hi, pcmd_lo,  # 9900-9901: PCMD
            0, 0,               # 9902-9903: 予約
            vcmd_hi, vcmd_lo,  # 9904-9905: VCMD
            accel_mm_s2,        # 9906: ACMD
            0,                  # 9907: 予約
            0x0002,             # 9908: CTLF = 直値移動有効
        ]
        await client.write_registers(
            address=_REG_PCMD_HI, values=registers, device_id=self._slave_id
        )
        logger.debug(
            "move_to_position: slave_id=%d pos=%d speed=%d",
            self._slave_id, pos, speed_mm_s,
        )

    async def read_position(self) -> int:
        """現在位置を読み取る。

        Returns:
            現在位置 [pulse / 0.01mm 単位]（符号付き 32bit）
        """
        client = self._require_client()
        result = await client.read_holding_registers(
            address=_REG_PNOW_HI, count=2, device_id=self._slave_id
        )
        if result.isError():
            raise OSError(f"read_position 失敗: slave_id={self._slave_id}")
        return _to_signed32(result.registers[0], result.registers[1])

    async def read_current(self) -> float:
        """電流値を読み取る。

        Returns:
            電流値 [mA]（符号付き 32bit）
        """
        client = self._require_client()
        result = await client.read_holding_registers(
            address=_REG_CNOW_HI, count=2, device_id=self._slave_id
        )
        if result.isError():
            raise OSError(f"read_current 失敗: slave_id={self._slave_id}")
        return float(_to_signed32(result.registers[0], result.registers[1]))

    async def is_alarm_active(self) -> bool:
        """アラームが発生しているか確認する。

        Returns:
            True: アラームあり（ALMC ≠ 0）
        """
        client = self._require_client()
        result = await client.read_holding_registers(
            address=_REG_ALMC, count=1, device_id=self._slave_id
        )
        if result.isError():
            raise OSError(f"is_alarm_active 失敗: slave_id={self._slave_id}")
        return result.registers[0] != 0
