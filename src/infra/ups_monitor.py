"""NUT (Network UPS Tools) を使った APC Smart-UPS 監視クラス。

NUT デーモン（upsd）に TCP 3493 で接続し、バッテリー残量・AC状態をポーリングする。
AC断（OL→OB 遷移）を検知したら登録済みコールバックを呼ぶ。

前提:
    - NUT が systemd サービスとして起動済み（scripts/setup_nut.sh 参照）
    - apcsmart ドライバでシリアル接続した APC Smart-UPS を管理していること
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable, Coroutine
from dataclasses import dataclass
from typing import Any

logger = logging.getLogger(__name__)

AsyncCallback = Callable[[], Coroutine[Any, Any, None]]

_NUT_LINE_TIMEOUT_S: float = 5.0
_CONNECT_TIMEOUT_S: float = 5.0
_STATUS_ON_BATTERY: str = "OB"
_STATUS_LOW_BATTERY: str = "LB"


@dataclass
class UPSStatus:
    """NUT から取得した UPS の状態スナップショット。"""

    battery_charge_pct: float
    on_battery: bool
    low_battery: bool
    status_flags: str
    is_available: bool


class NutUPSMonitor:
    """NUT socket プロトコルで UPS 状態を取得するインフラクラス。

    ポーリングループを asyncio.Task として実行し、キャッシュを常時更新する。
    UPSPreCheckProtocol を満たす get_battery_level_pct() を提供する。
    """

    def __init__(
        self,
        nut_host: str = "localhost",
        nut_port: int = 3493,
        ups_name: str = "apcups",
        poll_interval_s: float = 5.0,
    ) -> None:
        self._host = nut_host
        self._port = nut_port
        self._ups_name = ups_name
        self._poll_interval_s = poll_interval_s

        self._cached_battery_pct: float = 0.0
        self._cached_on_battery: bool = False
        self._cached_low_battery: bool = False
        self._cached_status_flags: str = ""
        self._nut_available: bool = False

        self._prev_on_battery: bool = False
        self._ac_loss_callbacks: list[AsyncCallback] = []
        self._polling_task: asyncio.Task[None] | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    # ── Public API ───────────────────────────────────────────────────────────

    def register_ac_loss_callback(self, cb: AsyncCallback) -> None:
        """AC断（OL→OB 遷移）検知時に呼ぶ非同期コールバックを登録する。"""
        self._ac_loss_callbacks.append(cb)

    async def get_battery_level_pct(self) -> float:
        """バッテリー残量 [%] を返す。UPSPreCheckProtocol 実装。

        NUT 未接続時は 0.0 を返し、走行前チェックを失敗させる。
        """
        if not self._nut_available:
            raise RuntimeError("NUT サーバーに接続できません")
        return self._cached_battery_pct

    async def get_status(self) -> UPSStatus:
        """キャッシュ済みの UPS 状態を返す。"""
        return UPSStatus(
            battery_charge_pct=self._cached_battery_pct,
            on_battery=self._cached_on_battery,
            low_battery=self._cached_low_battery,
            status_flags=self._cached_status_flags,
            is_available=self._nut_available,
        )

    @property
    def is_on_battery(self) -> bool:
        """キャッシュ済みの AC断フラグ。"""
        return self._cached_on_battery

    @property
    def is_available(self) -> bool:
        """NUT サーバーに接続できているか。"""
        return self._nut_available

    async def start_polling(self) -> None:
        """ポーリングループを asyncio.Task として起動する。"""
        self._loop = asyncio.get_event_loop()
        # 初回ポーリングを即座に実行してキャッシュを初期化
        await self._poll_once()
        self._polling_task = asyncio.create_task(self._polling_loop(), name="ups_monitor_poll")
        logger.info(
            "NutUPSMonitor ポーリング開始: host=%s:%d ups=%s interval=%.1fs",
            self._host,
            self._port,
            self._ups_name,
            self._poll_interval_s,
        )

    async def stop_polling(self) -> None:
        """ポーリングループを停止する。"""
        if self._polling_task is not None and not self._polling_task.done():
            self._polling_task.cancel()
            try:
                await self._polling_task
            except asyncio.CancelledError:
                pass
            self._polling_task = None
        logger.info("NutUPSMonitor ポーリング停止")

    # ── Internal ─────────────────────────────────────────────────────────────

    async def _polling_loop(self) -> None:
        """poll_interval_s 周期でポーリングを継続するループ。"""
        while True:
            await asyncio.sleep(self._poll_interval_s)
            await self._poll_once()

    async def _poll_once(self) -> None:
        """NUT に接続して battery.charge と ups.status を取得しキャッシュを更新する。"""
        try:
            battery_str = await self._nut_get_var("battery.charge")
            status_str = await self._nut_get_var("ups.status")

            battery_pct = float(battery_str)
            on_battery = _STATUS_ON_BATTERY in status_str.split()
            low_battery = _STATUS_LOW_BATTERY in status_str.split()

            self._cached_battery_pct = battery_pct
            self._cached_status_flags = status_str
            self._cached_low_battery = low_battery
            self._nut_available = True

            # OL → OB 遷移（立ち上がりエッジ）でコールバック発火
            if on_battery and not self._prev_on_battery:
                logger.warning(
                    "AC電源断を NUT で検知: battery=%.1f%% status=%s",
                    battery_pct,
                    status_str,
                )
                self._fire_callbacks()

            self._cached_on_battery = on_battery
            self._prev_on_battery = on_battery

        except Exception as e:
            if self._nut_available:
                logger.error("NUT ポーリング失敗（キャッシュ保持）: %s", e)
            self._nut_available = False
            # キャッシュは保持したまま（一時的な接続断を許容）

    async def _nut_get_var(self, var_name: str) -> str:
        """NUT socket プロトコルで指定変数の値を取得する。

        NUT プロトコル:
            → GET VAR <ups_name> <var_name>\\n
            ← VAR <ups_name> <var_name> "<value>"\\n
            または
            ← ERR <reason>\\n
        """
        reader, writer = await asyncio.wait_for(
            asyncio.open_connection(self._host, self._port),
            timeout=_CONNECT_TIMEOUT_S,
        )
        try:
            cmd = f"GET VAR {self._ups_name} {var_name}\n"
            writer.write(cmd.encode())
            await writer.drain()

            line_bytes = await asyncio.wait_for(reader.readline(), timeout=_NUT_LINE_TIMEOUT_S)
            line = line_bytes.decode().strip()

            writer.write(b"LOGOUT\n")
            await writer.drain()
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass

        if line.startswith("ERR"):
            raise RuntimeError(f"NUT エラー ({var_name}): {line}")

        # VAR apcups battery.charge "95"  → "95"
        parts = line.split('"')
        if len(parts) < 2:
            raise RuntimeError(f"NUT レスポンスが不正 ({var_name}): {line!r}")
        return parts[1]

    def _fire_callbacks(self) -> None:
        """登録済みの AC断コールバックをイベントループに投入する。"""
        if self._loop is None or not self._loop.is_running():
            logger.error("イベントループが実行中でないため AC断コールバックを投入できません")
            return
        for cb in self._ac_loss_callbacks:
            asyncio.run_coroutine_threadsafe(cb(), self._loop)
