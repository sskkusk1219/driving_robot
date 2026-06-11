"""lgpio を使った非常停止・AC電源断 GPIO 割り込みモニタ。

GPIO17: 非常停止スイッチ（物理ピン11、NC接点、プルアップ、RISING=停止）
        LOW=通常（NC接点が閉じてGNDに落ちている）
        HIGH=停止（NC接点が開いてプルアップが有効 / ケーブル断線でも停止）
GPIO27: AC UPS 接点出力 AC断検知（物理ピン13、プルアップ、FALLING=AC断）
        [要確認: AC UPS機種確定後に更新]
"""

from __future__ import annotations

import asyncio
import concurrent.futures
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
        # 直近のエッジから推定する非常停止スイッチ状態（HIGH=押下中）。
        # 権威ある値は is_emergency_active() の live read。ハンドル未確立時の保険。
        self._emergency_active = False

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

        # 非常停止: NC接点。押下=RISING(HIGH)、解除=FALLING(LOW)。
        # 両エッジを監視し、押下で非常停止を発火、解除はログ記録とフラグ更新に使う。
        # （解除を検知することで is_emergency_active() の判定とリセット可否に活かす）
        lgpio.gpio_claim_alert(
            self._handle, self._emergency_pin, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP
        )
        lgpio.gpio_set_debounce_micros(self._handle, self._emergency_pin, _DEBOUNCE_US)
        self._cb_emergency = lgpio.callback(
            self._handle, self._emergency_pin, lgpio.BOTH_EDGES, self._on_emergency
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
        """非常停止 両エッジ割り込みハンドラ（別スレッドから呼ばれる）。

        level=1 (RISING, 押下): 非常停止を発火。
        level=0 (FALLING, 解除): フラグを下げてログのみ（自動リセットはしない）。
        level=2 (watchdog) 等は無視する。
        """
        if level == 1:
            self._emergency_active = True
            logger.warning("非常停止スイッチ検知: gpio=%d level=%d", gpio, level)
            self._fire_callbacks(self._emergency_callbacks)
        elif level == 0:
            self._emergency_active = False
            logger.info("非常停止スイッチ解除: gpio=%d level=%d", gpio, level)

    def is_emergency_active(self) -> bool:
        """非常停止スイッチが現在押下（HIGH）されているかを返す。

        エッジ追跡ではなく GPIO の現在レベルを直接読み取る（権威ある値）。
        HIGH(1)=押下/停止、LOW(0)=解除/通常。ハンドル未確立時は追跡フラグを返す。
        """
        if self._handle is None:
            return self._emergency_active
        import lgpio  # noqa: PLC0415

        return bool(lgpio.gpio_read(self._handle, self._emergency_pin) == 1)

    def _on_ac_loss(self, chip: int, gpio: int, level: int, timestamp: int) -> None:  # noqa: ARG002
        """AC断 FALLING エッジ割り込みハンドラ（別スレッドから呼ばれる）。"""
        logger.warning("AC電源断検知: gpio=%d level=%d", gpio, level)
        self._fire_callbacks(self._ac_loss_callbacks)

    def _fire_callbacks(self, callbacks: list[AsyncCallback]) -> None:
        """登録済みの非同期コールバックをイベントループに投入する。

        Future を破棄するとコールバック内の例外（非常停止シーケンスの失敗）が
        どこにも記録されないため、必ず done_callback で例外を回収してログに残す。
        """
        if self._loop is None or not self._loop.is_running():
            logger.error("イベントループが実行中でないため、コールバックを投入できません。")
            return
        for cb in callbacks:
            future = asyncio.run_coroutine_threadsafe(cb(), self._loop)
            future.add_done_callback(_log_callback_failure)


def _log_callback_failure(future: concurrent.futures.Future[None]) -> None:
    """GPIO コールバックの例外をログに記録する（黙殺防止）。"""
    if future.cancelled():
        logger.error("GPIO コールバックがキャンセルされました。")
        return
    exc = future.exception()
    if exc is not None:
        logger.error("GPIO コールバックが失敗しました: %r", exc, exc_info=exc)
