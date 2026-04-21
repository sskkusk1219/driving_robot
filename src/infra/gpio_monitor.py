"""RPi.GPIO を使った非常停止・AC電源断 GPIO 割り込みモニタ。

GPIO17: 非常停止スイッチ（物理ピン11、プルアップ、FALLING=停止）
GPIO27: AC UPS 接点出力 AC断検知（物理ピン13、プルアップ、FALLING=AC断）
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable, Coroutine
from typing import Any

logger = logging.getLogger(__name__)

_BOUNCETIME_MS = 50  # チャタリング除去 [ms]

AsyncCallback = Callable[[], Coroutine[Any, Any, None]]


class GPIOMonitor:
    """GPIO 割り込みで非常停止・AC断を非同期コールバック経由で通知するクラス。

    RPi.GPIO の割り込みコールバックは別スレッドで実行されるため、
    asyncio.run_coroutine_threadsafe でイベントループへ投入する。
    """

    def __init__(
        self,
        emergency_pin: int = 17,
        ac_detect_pin: int = 27,
        loop: asyncio.AbstractEventLoop | None = None,
    ) -> None:
        self._emergency_pin = emergency_pin
        self._ac_detect_pin = ac_detect_pin
        self._loop = loop
        self._emergency_callbacks: list[AsyncCallback] = []
        self._ac_loss_callbacks: list[AsyncCallback] = []

    def register_emergency_callback(self, cb: AsyncCallback) -> None:
        """非常停止トリガー時に呼ばれる非同期コールバックを登録する。"""
        self._emergency_callbacks.append(cb)

    def register_ac_loss_callback(self, cb: AsyncCallback) -> None:
        """AC電源断検知時に呼ばれる非同期コールバックを登録する。"""
        self._ac_loss_callbacks.append(cb)

    async def start_monitoring(self) -> None:
        """GPIO のセットアップと割り込み登録を行う。"""
        import RPi.GPIO as GPIO  # noqa: PLC0415

        self._loop = asyncio.get_event_loop()

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._emergency_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self._ac_detect_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(
            self._emergency_pin,
            GPIO.FALLING,
            callback=self._on_emergency,
            bouncetime=_BOUNCETIME_MS,
        )
        GPIO.add_event_detect(
            self._ac_detect_pin,
            GPIO.FALLING,
            callback=self._on_ac_loss,
            bouncetime=_BOUNCETIME_MS,
        )
        logger.info(
            "GPIOMonitor 開始: emergency_pin=%d ac_detect_pin=%d",
            self._emergency_pin,
            self._ac_detect_pin,
        )

    def stop_monitoring(self) -> None:
        """GPIO 割り込みを解除してリソースをクリーンアップする。"""
        import RPi.GPIO as GPIO  # noqa: PLC0415

        GPIO.remove_event_detect(self._emergency_pin)
        GPIO.remove_event_detect(self._ac_detect_pin)
        GPIO.cleanup([self._emergency_pin, self._ac_detect_pin])
        logger.info("GPIOMonitor 停止: GPIO クリーンアップ完了")

    def _on_emergency(self, channel: int) -> None:
        """非常停止 FALLING エッジ割り込みハンドラ（別スレッドから呼ばれる）。"""
        logger.warning("非常停止スイッチ検知: channel=%d", channel)
        self._fire_callbacks(self._emergency_callbacks)

    def _on_ac_loss(self, channel: int) -> None:
        """AC断 FALLING エッジ割り込みハンドラ（別スレッドから呼ばれる）。"""
        logger.warning("AC電源断検知: channel=%d", channel)
        self._fire_callbacks(self._ac_loss_callbacks)

    def _fire_callbacks(self, callbacks: list[AsyncCallback]) -> None:
        """登録済みの非同期コールバックをイベントループに投入する。"""
        if self._loop is None or not self._loop.is_running():
            logger.error("イベントループが実行中でないため、コールバックを投入できません。")
            return
        for cb in callbacks:
            asyncio.run_coroutine_threadsafe(cb(), self._loop)
