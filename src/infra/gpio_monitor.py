"""lgpio を使った非常停止・AC電源断 GPIO 割り込みモニタ。

GPIO17: 非常停止スイッチ（物理ピン11、NC接点、プルアップ、RISING=停止）
        LOW=通常（NC接点が閉じてGNDに落ちている）
        HIGH=停止（NC接点が開いてプルアップが有効 / ケーブル断線でも停止）
GPIO27: AC UPS 接点出力 AC断検知（物理ピン13、プルアップ、FALLING=AC断）[要確認: AC UPS機種確定後に更新]
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable, Coroutine
from typing import Any

logger = logging.getLogger(__name__)

AsyncCallback = Callable[[], Coroutine[Any, Any, None]]

_CHIP = 0
_DEBOUNCE_US = 50_000  # 50ms チャタリング除去


class GPIOMonitor:
    """GPIO 割り込みで非常停止・AC断を非同期コールバック経由で通知するクラス。

    lgpio のコールバックは別スレッドで実行されるため、
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
        self._handle: int | None = None
        self._cb_emergency = None
        self._cb_ac = None

    def register_emergency_callback(self, cb: AsyncCallback) -> None:
        """非常停止トリガー時に呼ばれる非同期コールバックを登録する。"""
        self._emergency_callbacks.append(cb)

    def register_ac_loss_callback(self, cb: AsyncCallback) -> None:
        """AC電源断検知時に呼ばれる非同期コールバックを登録する。"""
        self._ac_loss_callbacks.append(cb)

    async def start_monitoring(self) -> None:
        """GPIO のセットアップと割り込み登録を行う。"""
        import lgpio  # noqa: PLC0415

        self._loop = asyncio.get_event_loop()
        self._handle = lgpio.gpiochip_open(_CHIP)

        # 非常停止: NC接点 → RISING エッジ（接点が開いてプルアップが有効になる）
        lgpio.gpio_claim_alert(
            self._handle, self._emergency_pin, lgpio.RISING_EDGE, lgpio.SET_PULL_UP
        )
        lgpio.gpio_set_debounce_micros(self._handle, self._emergency_pin, _DEBOUNCE_US)
        self._cb_emergency = lgpio.callback(
            self._handle, self._emergency_pin, lgpio.RISING_EDGE, self._on_emergency
        )

        # AC断検知: FALLING エッジ（AC断で接点が開放 → LOW になる想定、機種確定後に要確認）
        lgpio.gpio_claim_alert(
            self._handle, self._ac_detect_pin, lgpio.FALLING_EDGE, lgpio.SET_PULL_UP
        )
        lgpio.gpio_set_debounce_micros(self._handle, self._ac_detect_pin, _DEBOUNCE_US)
        self._cb_ac = lgpio.callback(
            self._handle, self._ac_detect_pin, lgpio.FALLING_EDGE, self._on_ac_loss
        )

        logger.info(
            "GPIOMonitor 開始: emergency_pin=%d ac_detect_pin=%d",
            self._emergency_pin,
            self._ac_detect_pin,
        )

    def stop_monitoring(self) -> None:
        """GPIO 割り込みを解除してリソースをクリーンアップする。"""
        import lgpio  # noqa: PLC0415

        if self._cb_emergency is not None:
            self._cb_emergency.cancel()
        if self._cb_ac is not None:
            self._cb_ac.cancel()
        if self._handle is not None:
            lgpio.gpiochip_close(self._handle)
            self._handle = None
        logger.info("GPIOMonitor 停止: GPIO クリーンアップ完了")

    def _on_emergency(self, chip: int, gpio: int, level: int, timestamp: int) -> None:  # noqa: ARG002
        """非常停止 RISING エッジ割り込みハンドラ（別スレッドから呼ばれる）。"""
        logger.warning("非常停止スイッチ検知: gpio=%d level=%d", gpio, level)
        self._fire_callbacks(self._emergency_callbacks)

    def _on_ac_loss(self, chip: int, gpio: int, level: int, timestamp: int) -> None:  # noqa: ARG002
        """AC断 FALLING エッジ割り込みハンドラ（別スレッドから呼ばれる）。"""
        logger.warning("AC電源断検知: gpio=%d level=%d", gpio, level)
        self._fire_callbacks(self._ac_loss_callbacks)

    def _fire_callbacks(self, callbacks: list[AsyncCallback]) -> None:
        """登録済みの非同期コールバックをイベントループに投入する。"""
        if self._loop is None or not self._loop.is_running():
            logger.error("イベントループが実行中でないため、コールバックを投入できません。")
            return
        for cb in callbacks:
            asyncio.run_coroutine_threadsafe(cb(), self._loop)
