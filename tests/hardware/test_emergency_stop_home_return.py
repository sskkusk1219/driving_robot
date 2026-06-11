"""非常停止スイッチ → 原点復帰 エンドツーエンド確認スクリプト。

functional-design.md UC4 のシーケンスをハードウェア実機で再現する:
    SW(GPIO17) → lgpio コールバック → asyncio → home_return() 両軸並列

接続:
    GPIO17 (物理ピン11) ─── NC接点スイッチ ─── GND (物理ピン9)
    /dev/ttyUSB0  slave_id=1  アクセル軸
    /dev/ttyUSB1  slave_id=2  ブレーキ軸

注意:
    lgpio はシステムパッケージ (apt install python3-lgpio) が必要。
    .venv 環境に lgpio がない場合は ImportError になる。
    実行方法: .venv/bin/python tests/hardware/test_emergency_stop_home_return.py
    (.venv は pymodbus を持ち、システムの lgpio は Python パスに含まれる)
"""

from __future__ import annotations

import asyncio
import os
import sys

# lgpio はシステムパッケージ (/usr/lib/python3/dist-packages)。
# .venv はシステムサイトパッケージを含まないため明示的にパスを追加する。
sys.path.insert(0, "/usr/lib/python3/dist-packages")
try:
    import lgpio
except ImportError:
    print(
        "ERROR: lgpio が見つかりません。sudo apt install python3-lgpio を実行してください。",
        file=sys.stderr,
    )
    sys.exit(1)

# プロジェクトルートからのインポートのため sys.path を調整
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from src.infra.actuator_driver import ActuatorDriver

# --- 定数 ---
_ACCEL_PORT = "/dev/ttyUSB0"
_BRAKE_PORT = "/dev/ttyUSB1"
_ACCEL_SLAVE_ID = 1
_BRAKE_SLAVE_ID = 1  # 両軸とも slave_id=1（各軸が独立した RS-485 バス上）
_BAUD_RATE = 38400

_GPIO_CHIP = 0
_EMERGENCY_PIN = 17
_DEBOUNCE_US = 50_000  # 50ms チャタリング除去


async def _home_return_both_axes(accel: ActuatorDriver, brake: ActuatorDriver) -> None:
    """両軸を並列で原点復帰する。DSS1 HEND ビット確認まで待機する。"""
    print("原点復帰開始（両軸並列）...")
    try:
        await asyncio.gather(accel.home_return(), brake.home_return())
        print("両軸 原点復帰完了")
    except TimeoutError as e:
        print(f"ERROR: 原点復帰タイムアウト: {e}")
    except Exception as e:
        print(f"ERROR: 原点復帰エラー: {e}")


def _make_emergency_callback(
    loop: asyncio.AbstractEventLoop,
    accel: ActuatorDriver,
    brake: ActuatorDriver,
) -> lgpio.callback:
    """lgpio コールバック関数を生成して返す。"""

    def _on_edge(chip: int, gpio: int, level: int, timestamp: int) -> None:  # noqa: ARG001
        if level == 1:
            print(f"\n[GPIO{gpio}] 非常停止検知 → 原点復帰開始")
            asyncio.run_coroutine_threadsafe(_home_return_both_axes(accel, brake), loop)

    return _on_edge


async def main() -> None:
    accel = ActuatorDriver(port=_ACCEL_PORT, slave_id=_ACCEL_SLAVE_ID, baud_rate=_BAUD_RATE)
    brake = ActuatorDriver(port=_BRAKE_PORT, slave_id=_BRAKE_SLAVE_ID, baud_rate=_BAUD_RATE)

    # Modbus 接続
    print("Modbus 接続中...")
    try:
        await accel.connect()
        await brake.connect()
    except ConnectionError as e:
        print(f"ERROR: Modbus 接続失敗: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Modbus 接続完了（アクセル軸: {_ACCEL_PORT}, ブレーキ軸: {_BRAKE_PORT}）")

    # 起動時前処理: reset_alarm → servo_on → home_return
    # 初期化は逐次実行（pymodbus 2ポート同時リクエストの競合を回避）
    print("起動時前処理: reset_alarm → servo_on...")
    print("  アクセル軸 reset_alarm...")
    await accel.reset_alarm()
    print("  ブレーキ軸 reset_alarm...")
    await brake.reset_alarm()
    print("  アクセル軸 servo_on...")
    await accel.servo_on()
    print("  ブレーキ軸 servo_on...")
    await brake.servo_on()
    await _home_return_both_axes(accel, brake)
    print("起動時原点復帰完了")

    # GPIO17 監視開始
    loop = asyncio.get_running_loop()
    h = lgpio.gpiochip_open(_GPIO_CHIP)
    lgpio.gpio_claim_alert(h, _EMERGENCY_PIN, lgpio.RISING_EDGE, lgpio.SET_PULL_UP)
    lgpio.gpio_set_debounce_micros(h, _EMERGENCY_PIN, _DEBOUNCE_US)
    cb = lgpio.callback(
        h, _EMERGENCY_PIN, lgpio.RISING_EDGE, _make_emergency_callback(loop, accel, brake)
    )

    print(
        "非常停止スイッチ監視中（GPIO17 RISING エッジ）。"
        "スイッチを押すと原点復帰します。Ctrl+C で終了。"
    )
    print("-" * 60)

    try:
        while True:
            await asyncio.sleep(0.1)
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        cb.cancel()
        lgpio.gpiochip_close(h)
        print("\nGPIOクリーンアップ完了")
        await accel.close()
        await brake.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
