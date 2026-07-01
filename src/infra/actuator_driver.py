"""IAI P-CON-CB 用 Modbus RTU アクチュエータドライバ。

MJ0162-12A（Modbus 仕様書 第12版）に基づく実装。
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from pymodbus.client import AsyncModbusSerialClient
from pymodbus.framer import FramerType

logger = logging.getLogger(__name__)

# FC03 読み取りレジスタアドレス（HEX）
_REG_PNOW_HI = 0x9000  # 現在位置 上位 16bit
_REG_ALMC = 0x9002  # アラームコード
_REG_DSS1 = 0x9005  # デバイスステータス1
_REG_DSSE = 0x9007  # 拡張デバイスステータス
_REG_CNOW_HI = 0x900C  # 電流値 上位 16bit

# DSS1 ビット定義
_DSS1_SV = 1 << 12  # サーボON中
_DSS1_ALMH = 1 << 10  # アラームあり
_DSS1_HEND = 1 << 4  # 原点復帰完了
_DSS1_PEND = 1 << 3  # 位置決め完了

# DSSE ビット定義
_DSSE_MOVE = 1 << 5  # 移動中

# FC05 コイルアドレス（HEX）
_COIL_SON = 0x0403  # サーボON
_COIL_ALRS = 0x0407  # アラームリセット
_COIL_HOME = 0x040B  # 原点復帰
_COIL_PMSL = 0x0427  # Modbus 操作権（PIO 無効化・Modbus 指令優先）

# FC10 書き込みレジスタアドレス（HEX）
_REG_PCMD_HI = 0x9900  # 目標位置 上位 16bit
_REG_VCMD_HI = 0x9904  # 速度指令 上位 16bit
_REG_ACMD = 0x9906  # 加減速指令
_REG_CTLF = 0x9908  # 制御フラグ

# デフォルト移動パラメータ
_DEFAULT_SPEED_MM_S = 100  # 速度 [mm/s]
_DEFAULT_ACCEL = 30  # 加減速指令（ACMD）[コントローラ固有単位 ≈ 0.01G]

# 時間指定移動（move_to_position_timed）の速度クランプ範囲 [mm/s]
_MIN_TIMED_SPEED_MM_S = 1  # 微小移動でも正の VCMD になる下限
_MAX_TIMED_SPEED_MM_S = _DEFAULT_SPEED_MM_S  # 上限＝既存の固定速度（今より速くはしない）

_HOME_RETURN_TIMEOUT_S = 30.0
_HOME_RETURN_POLL_INTERVAL_S = 0.1

_POSITION_COMPLETE_TIMEOUT_S = 10.0
_POSITION_COMPLETE_POLL_INTERVAL_S = 0.05
# 移動指令直後は PEND が前回値のまま残ることがあるため、判定開始前に短い猶予を置く
_POSITION_COMPLETE_START_DELAY_S = 0.05


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
        # retries=0 だと 1 回でも応答が遅延・分割すると即 ModbusIOException となり、
        # 遅れて届いた応答バイトが OS バッファに残って次トランザクションを汚染し、
        # 以降のトランザクションが永続的に失敗する（desync カスケード）。
        # retries>0 にすると pymodbus がタイムアウト時に recv_buffer をクリアして
        # 再送するため、単発の遅延から自動復旧できる。
        timeout: float = 0.3,
        retries: int = 3,
    ) -> None:
        self._port = port
        self._slave_id = slave_id
        self._baud_rate = baud_rate
        self._timeout = timeout
        self._retries = retries
        self._client: AsyncModbusSerialClient | None = None
        # pymodbus は execute() 内に独自ロックを持つが、ここでは「OS バッファ
        # フラッシュ → トランザクション」を不可分に行うためのロック。フラッシュと
        # 送信の間に別コルーチンのトランザクションが割り込むのを防ぐ。
        self._bus_lock = asyncio.Lock()

    def _flush_input_buffer(self) -> None:
        """OS シリアル受信バッファをクリアする。

        前トランザクションがタイムアウトした後に遅れて届いた応答バイトが
        OS 受信バッファに残ると、次トランザクションの応答先頭に混入して
        RTU フレーマーがデシンクする。各トランザクション開始前にフラッシュ
        することで、この残渣を除去して desync を断ち切る。
        pymodbus は自身の recv_buffer はクリアするが OS バッファはクリアしない。
        """
        try:
            if self._client is not None:
                transport = self._client.ctx.transport
                if hasattr(transport, "sync_serial"):
                    transport.sync_serial.reset_input_buffer()
        except Exception:
            pass

    @asynccontextmanager
    async def _bus_op(self) -> AsyncIterator[None]:
        """バス排他 + 受信バッファフラッシュを組み合わせたコンテキストマネージャ。"""
        async with self._bus_lock:
            self._flush_input_buffer()
            yield

    async def connect(self) -> None:
        """Modbus RTU 接続を確立する。"""
        self._client = AsyncModbusSerialClient(
            port=self._port,
            baudrate=self._baud_rate,
            bytesize=8,
            parity="N",
            stopbits=1,
            framer=FramerType.RTU,
            timeout=self._timeout,
            retries=self._retries,
        )
        connected = await self._client.connect()
        if not connected:
            raise ConnectionError(f"Modbus RTU 接続失敗: port={self._port}")
        # FTDI USB-RS485 アダプタのレイテンシタイマーをデフォルト 16ms から 1ms に下げる。
        # 16ms のままでは Modbus 応答だけで最大 16ms 遅れ、50ms 制御ループ予算を超過する。
        try:
            transport = self._client.ctx.transport
            if hasattr(transport, "sync_serial"):
                transport.sync_serial.set_low_latency_mode(True)
                logger.debug("FTDI low_latency 設定完了: port=%s", self._port)
        except Exception:
            logger.debug("low_latency 設定をスキップ: port=%s（非 FTDI/権限なし）", self._port)
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

    async def enable_modbus_control(self) -> None:
        """Modbus 操作権を有効化する（PMSL コイル = True）。

        PIO 入力を無効化し、Modbus による位置指令を受け付ける状態にする。
        reset_alarm() / servo_on() より前に呼ぶこと。
        """
        client = self._require_client()
        async with self._bus_op():
            await client.write_coil(address=_COIL_PMSL, value=True, device_id=self._slave_id)
        logger.debug("enable_modbus_control: slave_id=%d", self._slave_id)

    async def reset_alarm(self) -> None:
        """アラームをリセットする（ALRS コイルをエッジ入力）。"""
        client = self._require_client()
        async with self._bus_op():
            await client.write_coil(address=_COIL_ALRS, value=True, device_id=self._slave_id)
        await asyncio.sleep(0.05)
        async with self._bus_op():
            await client.write_coil(address=_COIL_ALRS, value=False, device_id=self._slave_id)
        logger.debug("reset_alarm 完了: slave_id=%d", self._slave_id)

    async def servo_on(self) -> None:
        """サーボをONにする。"""
        client = self._require_client()
        async with self._bus_op():
            await client.write_coil(address=_COIL_SON, value=True, device_id=self._slave_id)
        logger.debug("servo_on: slave_id=%d", self._slave_id)

    async def servo_off(self) -> None:
        """サーボをOFFにする。"""
        client = self._require_client()
        async with self._bus_op():
            await client.write_coil(address=_COIL_SON, value=False, device_id=self._slave_id)
        logger.debug("servo_off: slave_id=%d", self._slave_id)

    async def home_return(self) -> None:
        """原点復帰を実行する。DSS1 HEND ビットが立つまでポーリング。

        タイムアウト (_HOME_RETURN_TIMEOUT_S) を超えた場合は TimeoutError を送出。
        """
        client = self._require_client()
        # P-CON-CB はコイルの立ち上がりエッジ（False→True）で原点復帰をトリガーする
        async with self._bus_op():
            await client.write_coil(address=_COIL_HOME, value=False, device_id=self._slave_id)
        await asyncio.sleep(0.05)
        async with self._bus_op():
            await client.write_coil(address=_COIL_HOME, value=True, device_id=self._slave_id)
        logger.info("home_return 開始: slave_id=%d", self._slave_id)

        deadline = asyncio.get_event_loop().time() + _HOME_RETURN_TIMEOUT_S
        while asyncio.get_event_loop().time() < deadline:
            await asyncio.sleep(_HOME_RETURN_POLL_INTERVAL_S)
            async with self._bus_op():
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
        accel: int = _DEFAULT_ACCEL,
    ) -> None:
        """指定位置へ移動指令を送出する（FC10 絶対位置移動）。

        Args:
            pos: 目標位置 [pulse / 0.01mm 単位]
            speed_mm_s: 移動速度 [mm/s]
            accel: 加減速指令 [コントローラ固有単位 ≈ 0.01G]
        """
        client = self._require_client()
        pcmd_hi, pcmd_lo = _from_signed32(pos)
        vcmd_hi, vcmd_lo = _from_signed32(speed_mm_s * 100)  # VCMD は 0.01mm/s 単位

        # 9900: PCMD_HI, 9901: PCMD_LO, 9902: INP_HI, 9903: INP_LO,
        # 9904: VCMD_HI, 9905: VCMD_LO, 9906: ACMD, 9907: 予約, 9908: CTLF
        registers = [
            pcmd_hi,
            pcmd_lo,  # 9900-9901: PCMD（目標位置 0.01mm 単位）
            0,
            10,  # 9902-9903: INP（位置決め完了幅）
            vcmd_hi,
            vcmd_lo,  # 9904-9905: VCMD（0.01mm/s 単位）
            accel,  # 9906: ACMD
            0,  # 9907: 予約
            0x0000,  # 9908: CTLF = 絶対位置移動
        ]
        async with self._bus_op():
            await client.write_registers(
                address=_REG_PCMD_HI, values=registers, device_id=self._slave_id
            )
        logger.debug(
            "move_to_position: slave_id=%d pos=%d speed=%d",
            self._slave_id,
            pos,
            speed_mm_s,
        )

    async def move_to_position_timed(
        self,
        target_pos: int,
        current_pos: int,
        duration_s: float,
        accel: int = _DEFAULT_ACCEL,
    ) -> None:
        """目標位置まで「指定時間かけて」移動する（速度を距離÷時間で算出）。

        固定速度（mm/s）ではなく「何秒で目標へ到達するか」を指定したい用途向け。
        サーボのプロファイル速度（VCMD）を `距離[mm] / duration_s` から決めるため、
        制御ループの周期ジッタに依存せず一定時間で滑らかにランプする。

        Args:
            target_pos: 目標位置 [pulse / 0.01mm 単位]
            current_pos: 現在（直近指令）位置 [同]。距離算出に使う（Modbus 読みは挟まない）
            duration_s: 目標到達までの目標時間 [s]。<=0 または距離0 のときは許容最速で移動
            accel: 加減速指令（ACMD）
        """
        distance_mm = abs(target_pos - current_pos) / 100.0  # PCMD は 0.01mm 単位
        if duration_s <= 0.0 or distance_mm <= 0.0:
            speed_mm_s = _MAX_TIMED_SPEED_MM_S
        else:
            speed_mm_s = round(distance_mm / duration_s)
        speed_mm_s = max(_MIN_TIMED_SPEED_MM_S, min(speed_mm_s, _MAX_TIMED_SPEED_MM_S))
        await self.move_to_position(target_pos, speed_mm_s=speed_mm_s, accel=accel)

    async def wait_for_position_complete(
        self, timeout_s: float = _POSITION_COMPLETE_TIMEOUT_S
    ) -> None:
        """位置決め完了（DSS1 PEND かつ DSSE 非 MOVE）までポーリングする。

        move_to_position 直後に read_position すると移動途中値を読むため、
        ジョグなど移動完了後の確定位置が必要な場面で本メソッドを挟む。
        50ms 制御ループ（DriveLoop）では呼ばないこと（ブロックするため）。

        タイムアウト (timeout_s) を超えた場合は TimeoutError を送出する。
        """
        client = self._require_client()
        await asyncio.sleep(_POSITION_COMPLETE_START_DELAY_S)
        deadline = asyncio.get_event_loop().time() + timeout_s
        while asyncio.get_event_loop().time() < deadline:
            # DSS1(0x9005), 0x9006, DSSE(0x9007) を一括読み取り
            async with self._bus_op():
                result = await client.read_holding_registers(
                    address=_REG_DSS1, count=3, device_id=self._slave_id
                )
            if result.isError():
                logger.warning(
                    "wait_for_position_complete ステータス読み取りエラー: slave_id=%d",
                    self._slave_id,
                )
                await asyncio.sleep(_POSITION_COMPLETE_POLL_INTERVAL_S)
                continue
            dss1 = result.registers[0]
            dsse = result.registers[2]
            if (dss1 & _DSS1_PEND) and not (dsse & _DSSE_MOVE):
                return
            await asyncio.sleep(_POSITION_COMPLETE_POLL_INTERVAL_S)

        raise TimeoutError(f"wait_for_position_complete タイムアウト: slave_id={self._slave_id}")

    async def read_position(self) -> int:
        """現在位置を読み取る。

        Returns:
            現在位置 [pulse / 0.01mm 単位]（符号付き 32bit）
        """
        client = self._require_client()
        async with self._bus_op():
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
        async with self._bus_op():
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
        async with self._bus_op():
            result = await client.read_holding_registers(
                address=_REG_ALMC, count=1, device_id=self._slave_id
            )
        if result.isError():
            raise OSError(f"is_alarm_active 失敗: slave_id={self._slave_id}")
        return result.registers[0] != 0
