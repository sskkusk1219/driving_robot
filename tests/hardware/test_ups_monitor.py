"""NutUPSMonitor 実機確認スクリプト。

以下を対話的に確認する:
    1. UPS 通信 OK か？（NUT socket 接続・変数取得）
    2. バッテリー残量はいくつか？
    3. AC断コールバック（UPS の AC ケーブルを抜いて OL→OB 遷移を確認）

接続:
    APC Smart-UPS 750（SUA750JB）
    シリアル(DB-9) ── Prolific PL2303 USB アダプタ ── /dev/ttyUSB0
    NUT apcsmart ドライバ（localhost:3493）

前提:
    sudo systemctl start nut-driver.target nut-server
    または scripts/setup_nut.sh を実行済みであること

実行方法:
    .venv/bin/python tests/hardware/test_ups_monitor.py
"""

from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from src.infra.ups_monitor import NutUPSMonitor

_POLL_INTERVAL_S = 2.0  # ハードウェアテスト中は短めに設定


async def _check_communication(monitor: NutUPSMonitor) -> bool:
    """UPS 通信チェック。接続できれば True を返す。"""
    print("=" * 60)
    print("【チェック1】UPS 通信")
    print("=" * 60)
    try:
        charge = await monitor._nut_get_var("battery.charge")
        status = await monitor._nut_get_var("ups.status")
        print("  NUT socket 接続      : OK")
        print(f"  battery.charge (raw) : {charge}")
        print(f"  ups.status (raw)     : {status}")
        return True
    except Exception as e:
        print("  NUT socket 接続      : NG")
        print(f"  エラー               : {e}")
        print()
        print("  確認事項:")
        print("    sudo systemctl status nut-server")
        print("    upsc apcups@localhost")
        return False


async def _show_battery(monitor: NutUPSMonitor) -> None:
    """バッテリー残量と現在のステータスを表示する。"""
    print()
    print("=" * 60)
    print("【チェック2】バッテリー残量・UPS ステータス")
    print("=" * 60)
    await monitor._poll_once()
    status = await monitor.get_status()
    charge = status.battery_charge_pct
    bar_len = int(charge / 5)
    bar = "█" * bar_len + "░" * (20 - bar_len)

    print(f"  バッテリー残量 : {charge:.1f}%  [{bar}]")
    print(f"  UPS ステータス : {status.status_flags}")
    print(f"  AC 通電中      : {'YES' if not status.on_battery else 'NO（バッテリー運転中）'}")
    print(f"  低残量警告     : {'YES' if status.low_battery else 'NO'}")
    print(f"  NUT 接続状態   : {'OK' if status.is_available else 'NG'}")

    if status.on_battery:
        print()
        print("  ※ 現在バッテリー運転中です（AC 断検知済み）")


async def _watch_ac_loss(monitor: NutUPSMonitor) -> None:
    """AC断コールバック動作確認。Ctrl+C で終了する。"""
    print()
    print("=" * 60)
    print("【チェック3】AC断コールバック")
    print("=" * 60)
    print("  UPS の AC ケーブルを抜くと OL→OB 遷移を検知します。")
    print("  コールバックが発火したらメッセージが表示されます。")
    print("  Ctrl+C で終了。")
    print("-" * 60)

    ac_loss_count = 0

    async def on_ac_loss() -> None:
        nonlocal ac_loss_count
        ac_loss_count += 1
        print()
        print(f"  ★★★ AC断コールバック発火！（{ac_loss_count}回目）★★★")
        print("      ups.status が OL → OB に遷移しました")
        print("      → SafetyMonitor.handle_ac_power_loss() が呼ばれます")
        print()

    monitor.register_ac_loss_callback(on_ac_loss)

    tick = 0
    try:
        while True:
            await monitor._poll_once()
            status = await monitor.get_status()
            charge = status.battery_charge_pct
            flag = status.status_flags
            on_bat = "OB（バッテリー運転中）" if status.on_battery else "OL（AC通電中）"
            print(f"\r  [{tick:4d}s] {flag:10s}  {charge:5.1f}%  {on_bat}    ", end="", flush=True)
            tick += int(_POLL_INTERVAL_S)
            await asyncio.sleep(_POLL_INTERVAL_S)
    except (KeyboardInterrupt, asyncio.CancelledError):
        print()
        print("-" * 60)
        print(f"  AC断コールバック合計発火回数: {ac_loss_count} 回")


async def main() -> None:
    monitor = NutUPSMonitor(
        ups_name="apcups",
        poll_interval_s=_POLL_INTERVAL_S,
    )

    # チェック1: 通信確認
    ok = await _check_communication(monitor)
    if not ok:
        sys.exit(1)

    # チェック2: バッテリー残量
    await _show_battery(monitor)

    # チェック3: AC断コールバック（Ctrl+C まで待機）
    await _watch_ac_loss(monitor)

    print("\n終了")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
