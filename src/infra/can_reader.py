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

# 初回 Speed フレームの受信待ちタイムアウト [s]（初期化時の通信確認に使う）
_FIRST_FRAME_TIMEOUT_S = 2.0
# キャッシュ済み車速の許容鮮度 [s] の既定値。これを超えたらバス無音とみなし TimeoutError を
# 送出する（DriveLoop が捕捉して非常停止する）。
# 0.2s = 制御 4 周期。1.0s では凍結車速のまま最大 20 周期制御が走り、5km/h/s の減速中に
# 実偏差が KPI ハード上限（1.0km/h）を超えても逸脱チェックが検知できない
# （コードレビュー 2026-06-11 指摘 #2）。
DEFAULT_MAX_SPEED_AGE_S = 0.2


class CANReader:
    """CAN bus から車速を読み取る非同期クラス。

    can.Notifier + can.AsyncBufferedReader によるイベント駆動型受信を使用する。
    常駐コンシューマタスクがキューを常時ドレインして最新の車速のみをキャッシュするため、
    キューが無制限に成長したり、古いフレームの車速を「現在値」として返すことがない。
    read_speed() はキャッシュを返すだけでブロックしない（初回のみフレーム受信を待つ）。
    """

    def __init__(
        self,
        interface: str = "kvaser",
        channel: int = 0,
        bitrate: int = 500000,
        dbc_path: str | None = None,
        max_speed_age_s: float = DEFAULT_MAX_SPEED_AGE_S,
    ) -> None:
        self._interface = interface
        self._channel = channel
        self._bitrate = bitrate
        self._max_speed_age_s = max_speed_age_s
        self._dbc_path = Path(dbc_path) if dbc_path else None
        self._bus: Any = None
        self._db: Any = None
        self._notifier: Any = None
        self._async_reader: Any = None
        self._consume_task: asyncio.Task[None] | None = None
        self._first_frame: asyncio.Event | None = None
        self._latest_speed: float | None = None
        self._latest_at: float = 0.0

    async def connect(self) -> None:
        """CAN バスに接続し、DBC ファイルをロードする。"""
        import can

        loop = asyncio.get_event_loop()

        # DBC のロードは Bus オープンより先に行う。Bus オープン後に失敗すると Bus
        # ハンドルがクローズされずリークし、single_handle のチャネルが占有されたまま
        # 残ってしまう（次回接続がハンドル不足で失敗し得る）ため、失敗し得る処理を
        # 先に済ませておく（I6 レビュー指摘）。
        if self._dbc_path is not None:
            if not self._dbc_path.exists():
                raise FileNotFoundError(f"DBC ファイルが見つかりません: {self._dbc_path}")
            import cantools

            self._db = cantools.database.load_file(str(self._dbc_path))
            logger.info("DBC ロード完了: %s", self._dbc_path)
        else:
            logger.warning("DBC ファイル未指定。read_speed() は NotImplementedError を送出します。")

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

        # イベント駆動型受信: Notifier がバックグラウンドスレッドでフレームを受け取り
        # AsyncBufferedReader のキューに積み、常駐コンシューマが最新車速をキャッシュする
        self._async_reader = can.AsyncBufferedReader()
        self._notifier = can.Notifier(self._bus, [self._async_reader], loop=loop)
        self._latest_speed = None
        self._first_frame = asyncio.Event()
        self._consume_task = asyncio.ensure_future(self._consume_frames())
        self._consume_task.add_done_callback(_log_consume_task_failure)

        logger.info("CANReader 接続完了: interface=%s channel=%d", self._interface, self._channel)

    async def close(self) -> None:
        """CAN バスを閉じる。"""
        if self._consume_task is not None:
            self._consume_task.cancel()
            self._consume_task = None
        if self._notifier is not None:
            self._notifier.stop()
            self._notifier = None
        if self._bus is not None:
            bus = self._bus
            self._bus = None
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, bus.shutdown)
            logger.info("CANReader 切断: interface=%s", self._interface)

    async def _consume_frames(self) -> None:
        """受信キューを常時ドレインし、最新の Speed 値をキャッシュする常駐タスク。"""
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

            self._latest_speed = float(decoded[_SPEED_SIGNAL_NAME])
            self._latest_at = asyncio.get_running_loop().time()
            if self._first_frame is not None:
                self._first_frame.set()

    async def read_speed(self) -> float:
        """キャッシュされた最新の車速 [km/h] を返す。

        初回のみ Speed フレームの受信を最大 2s 待つ。キャッシュが max_speed_age_s
        （既定 0.2s）以上更新されていない場合はバス無音とみなし TimeoutError を送出する
        （呼び出し元の DriveLoop が捕捉して非常停止する。従来の「次フレームまで永久に待つ」
        挙動は、バス無音時に制御ループが凍結しペダルが踏まれたまま放置される危険が
        あるため廃止した）。

        Returns:
            車速 [km/h]

        Raises:
            NotImplementedError: DBC ファイルが未指定
            RuntimeError: connect() 未呼び出し
            TimeoutError: Speed フレームが受信できない／キャッシュが古すぎる
        """
        if self._db is None:
            raise NotImplementedError(
                "DBC ファイルが未指定です。CANReader に dbc_path を渡してください。"
            )
        if self._async_reader is None or self._first_frame is None:
            raise RuntimeError("connect() を先に呼んでください。")

        if self._latest_speed is None:
            try:
                await asyncio.wait_for(self._first_frame.wait(), _FIRST_FRAME_TIMEOUT_S)
            except TimeoutError:
                raise TimeoutError(
                    f"CAN Speed フレームを {_FIRST_FRAME_TIMEOUT_S:.0f}s 以内に受信できません"
                ) from None

        age_s = asyncio.get_running_loop().time() - self._latest_at
        if age_s > self._max_speed_age_s:
            raise TimeoutError(f"CAN 車速が {age_s:.1f}s 更新されていません（バス無音の疑い）")

        assert self._latest_speed is not None
        logger.debug("read_speed: %.2f km/h", self._latest_speed)
        return self._latest_speed


def _log_consume_task_failure(task: asyncio.Task[None]) -> None:
    """常駐コンシューマタスクの異常終了をログに記録する。

    タスクが死ぬとキャッシュが更新されなくなるが、read_speed() の鮮度チェックが
    TimeoutError を送出するため走行は安全側（非常停止）に倒れる。
    """
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        logger.error("CAN 受信コンシューマタスクが異常終了しました", exc_info=exc)
