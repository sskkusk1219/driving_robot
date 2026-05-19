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

# MEIDEN_MEIDACS.dbc の MEIDACS_Frame0 (0x120) に定義されたシグナル名
# DBC を差し替える場合はシグナル名を合わせて更新すること
_SPEED_SIGNAL_NAME = "Speed"


class CANReader:
    """CAN bus から車速を読み取る非同期クラス。

    can.Notifier + can.AsyncBufferedReader によるイベント駆動型受信を使用する。
    フレームはバックグラウンドスレッドで受信されキューに積まれるため、
    ポーリングのタイムアウトによる誤 TimeoutError が発生しない。
    """

    def __init__(
        self,
        interface: str = "kvaser",
        channel: int = 0,
        bitrate: int = 500000,
        dbc_path: str | None = None,
    ) -> None:
        self._interface = interface
        self._channel = channel
        self._bitrate = bitrate
        self._dbc_path = Path(dbc_path) if dbc_path else None
        self._bus: Any = None
        self._db: Any = None
        self._notifier: Any = None
        self._async_reader: Any = None

    async def connect(self) -> None:
        """CAN バスに接続し、DBC ファイルをロードする。"""
        import can

        loop = asyncio.get_event_loop()

        # Kvaser の一部デバイスは canSetAcceptanceFilter が未実装 (Error Code -32) のため
        # Bus 初期化時に can.kvaser ロガーが error を出力する。機能には影響しないので抑制する。
        _kvaser_log = logging.getLogger("can.kvaser")
        _prev_level = _kvaser_log.level
        _kvaser_log.setLevel(logging.CRITICAL)
        try:
            self._bus = await loop.run_in_executor(
                None,
                lambda: can.Bus(
                    interface=self._interface,
                    channel=self._channel,
                    bitrate=self._bitrate,
                    # single_handle=True: 1ハンドルで送受信を共用する。
                    # デフォルト(False)は読み書きで別ハンドルを開くが、
                    # usbcanII などの旧デバイスで ACK 送出が不安定になる場合がある。
                    single_handle=True,
                ),
            )
        finally:
            _kvaser_log.setLevel(_prev_level)

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

        # イベント駆動型受信: Notifier がバックグラウンドスレッドでフレームを受け取り
        # AsyncBufferedReader のキューに積む
        self._async_reader = can.AsyncBufferedReader()
        self._notifier = can.Notifier(self._bus, [self._async_reader], loop=loop)

        logger.info(
            "CANReader 接続完了: interface=%s channel=%d", self._interface, self._channel
        )

    async def close(self) -> None:
        """CAN バスを閉じる。"""
        if self._notifier is not None:
            self._notifier.stop()
            self._notifier = None
        if self._bus is not None:
            bus = self._bus
            self._bus = None
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, bus.shutdown)
            logger.info("CANReader 切断: interface=%s", self._interface)

    async def read_speed(self) -> float:
        """次の Speed フレームが届くまで待機し、車速 [km/h] を返す。

        DBC ファイルが未指定の場合は NotImplementedError を送出する。
        Speed 以外の ID のフレームは読み飛ばして次を待つ。

        Returns:
            車速 [km/h]

        Raises:
            NotImplementedError: DBC ファイルが未指定
            RuntimeError: connect() 未呼び出し
        """
        if self._db is None:
            raise NotImplementedError(
                "DBC ファイルが未指定です。CANReader に dbc_path を渡してください。"
            )
        if self._async_reader is None:
            raise RuntimeError("connect() を先に呼んでください。")

        while True:
            msg = await self._async_reader.get_message()

            if msg.is_error_frame:
                logger.warning("エラーフレーム受信: ID=0x%X", msg.arbitration_id)
                continue

            try:
                decoded = self._db.decode_message(
                    msg.arbitration_id, msg.data, allow_truncated=True
                )
            except KeyError:
                logger.debug("不明な CAN フレーム ID: 0x%X (スキップ)", msg.arbitration_id)
                continue
            except Exception as e:
                logger.warning(
                    "デコードエラー: ID=0x%X len=%d (%s)", msg.arbitration_id, len(msg.data), e
                )
                continue

            if _SPEED_SIGNAL_NAME not in decoded:
                continue

            speed: float = float(decoded[_SPEED_SIGNAL_NAME])
            logger.debug("read_speed: %.2f km/h", speed)
            return speed
