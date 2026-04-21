"""Kvaser USB-CAN 経由でシャシダイナモ車速を取得する CANReader。

python-can 4.x + cantools（DBC デコード）を使用する。
DBC ファイルが存在しない場合は NotImplementedError を送出する。
"""

from __future__ import annotations

import asyncio
import logging
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

_CAN_RECV_TIMEOUT_S = 0.1
_SPEED_SIGNAL_NAME = "VehicleSpeed"


class CANReader:
    """CAN bus から車速を読み取る非同期クラス。

    python-can の Bus は同期 API であるため、
    asyncio の run_in_executor 経由でスレッドプールに委譲する。
    """

    def __init__(
        self,
        interface: str = "kvaser",
        channel: int = 0,
        dbc_path: str | None = None,
    ) -> None:
        self._interface = interface
        self._channel = channel
        self._dbc_path = Path(dbc_path) if dbc_path else None
        self._bus: Any = None
        self._db: Any = None

    async def connect(self) -> None:
        """CAN バスに接続し、DBC ファイルをロードする。"""
        import can

        loop = asyncio.get_event_loop()
        self._bus = await loop.run_in_executor(
            None,
            lambda: can.Bus(interface=self._interface, channel=self._channel),
        )

        if self._dbc_path is not None:
            if not self._dbc_path.exists():
                raise FileNotFoundError(f"DBC ファイルが見つかりません: {self._dbc_path}")
            import cantools

            self._db = cantools.database.load_file(str(self._dbc_path))
            logger.info("DBC ロード完了: %s", self._dbc_path)
        else:
            logger.warning(
                "DBC ファイル未指定。read_speed() は NotImplementedError を送出します。"
            )

        logger.info(
            "CANReader 接続完了: interface=%s channel=%d", self._interface, self._channel
        )

    async def close(self) -> None:
        """CAN バスを閉じる。"""
        if self._bus is not None:
            bus = self._bus
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, bus.shutdown)
            self._bus = None
            logger.info("CANReader 切断: interface=%s", self._interface)

    async def read_speed(self) -> float:
        """CAN フレームを受信し、車速 [km/h] を返す。

        DBC ファイルが未指定の場合は NotImplementedError を送出する。
        受信タイムアウト時は最後に受信した値を返す（初回はタイムアウト例外）。

        Returns:
            車速 [km/h]

        Raises:
            NotImplementedError: DBC ファイルが未指定
            TimeoutError: CAN フレームを受信できなかった
        """
        if self._db is None:
            raise NotImplementedError(
                "DBC ファイルが未指定です。CANReader に dbc_path を渡してください。"
            )
        if self._bus is None:
            raise RuntimeError("connect() を先に呼んでください。")

        import can

        bus = self._bus
        loop = asyncio.get_event_loop()

        msg: can.Message | None = await loop.run_in_executor(
            None,
            lambda: bus.recv(timeout=_CAN_RECV_TIMEOUT_S),
        )
        if msg is None:
            raise TimeoutError("CAN フレームの受信タイムアウト")

        db = self._db
        try:
            decoded = db.decode_message(msg.arbitration_id, msg.data)
        except KeyError:
            raise ValueError(
                f"不明な CAN フレーム ID: 0x{msg.arbitration_id:X}"
            ) from None

        if _SPEED_SIGNAL_NAME not in decoded:
            raise ValueError(
                f"DBC に '{_SPEED_SIGNAL_NAME}' シグナルが見つかりません。"
            )

        speed: float = float(decoded[_SPEED_SIGNAL_NAME])
        logger.debug("read_speed: %.2f km/h", speed)
        return speed
