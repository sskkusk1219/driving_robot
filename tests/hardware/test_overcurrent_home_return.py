"""過電流検知 → 原点復帰 エンドツーエンド確認スクリプト。

functional-design.md エラーハンドリング「過電流検知: 制御ループ停止 → 緊急停止」を実機で再現する:
    DriveLoop: read_current() → check_overcurrent() → home_return() 両軸並列

テスト手順:
    1. スクリプトを起動する
    2. アクチュエータが往復移動しながら電流・位置・ALMCを表示する
    3. アクチュエータを手で押さえて過電流を発生させるか、
       TEST_OVERCURRENT_LIMIT_MA を実機電流より低い値に設定する
    4. 「過電流検知」が表示され原点復帰することを目視確認する

表示項目:
    電流[mA]: CNOW レジスタ（0x900C/0x900D）
    位置[pulse]: PNOW レジスタ（0x9000/0x9001）。変化すればモーターは動作中
    ALMC: アラームコード（0x9002）。0x0000 はアラームなし

接続:
    /dev/ttyUSB0  slave_id=1  アクセル軸
    /dev/ttyUSB1  slave_id=1  ブレーキ軸

実行方法: .venv/bin/python tests/hardware/test_overcurrent_home_return.py
"""

from __future__ import annotations

import asyncio
import os
import sys

# プロジェクトルートからのインポートのため sys.path を調整
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from src.infra.actuator_driver import ActuatorDriver

# --- 定数 ---
_ACCEL_PORT = "/dev/ttyUSB0"
_BRAKE_PORT = "/dev/ttyUSB1"
_ACCEL_SLAVE_ID = 1
_BRAKE_SLAVE_ID = 1  # 両軸とも slave_id=1（各軸が独立した RS-485 バス上）
_BAUD_RATE = 38400

# 過電流テスト閾値 [mA]
# 移動中の電流値を確認してから、その値より低く設定して過電流を発生させる。
# 例: 移動中が 300mA → 200 に設定
TEST_OVERCURRENT_LIMIT_MA = 300

# 往復移動の折り返し位置 [pulse / 0.01mm]
_FAR_POSITION = 5000

# 移動速度 [mm/s]
_MOVE_SPEED_MM_S = 10

# 電流監視間隔 [秒]（50ms 制御ループと同じ）
_POLL_INTERVAL_S = 0.05


async def _read_almc_code(driver: ActuatorDriver) -> int:
    """ALMC レジスタ生値を読む（デバッグ用）。0=正常、-1=読み取り失敗。"""
    try:
        client = driver._client  # noqa: SLF001
        if client is None:
            return -1
        r = await client.read_holding_registers(
            address=0x9002,
            count=1,
            device_id=driver._slave_id,  # noqa: SLF001
        )
        return -1 if r.isError() else r.registers[0]
    except Exception:
        return -1


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


async def _read_axis_state(driver: ActuatorDriver, name: str) -> tuple[float, int, int]:
    """電流値・ALMCコード・現在位置を取得する。失敗時は (0.0, -1, 0)。"""
    try:
        current = await driver.read_current()
    except Exception as e:
        print(f"  ERROR: {name} 電流値読み取り失敗: {e}")
        current = 0.0

    almc = await _read_almc_code(driver)

    try:
        position = await driver.read_position()
    except Exception as e:
        print(f"  ERROR: {name} 位置読み取り失敗: {e}")
        position = 0

    return current, almc, position


async def _poll_current_and_check(
    accel: ActuatorDriver,
    brake: ActuatorDriver,
    limit_ma: float,
    poll_count: int,
) -> bool:
    """poll_count 回だけ 50ms 間隔で電流・アラームを監視し、過電流なら home_return する。

    Returns:
        True: 過電流検知・原点復帰実行済み（制御ループを終了すべき）
        False: 正常（ループ継続）
    """
    for _ in range(poll_count):
        await asyncio.sleep(_POLL_INTERVAL_S)

        (
            (accel_cur, accel_almc, accel_pos),
            (brake_cur, brake_almc, brake_pos),
        ) = await asyncio.gather(
            _read_axis_state(accel, "accel"),
            _read_axis_state(brake, "brake"),
        )

        alarm_tags = []
        if accel_almc > 0:
            alarm_tags.append(f"ALMC:a=0x{accel_almc:04X}")
        if brake_almc > 0:
            alarm_tags.append(f"ALMC:b=0x{brake_almc:04X}")
        alarm_str = "  " + "  ".join(alarm_tags) if alarm_tags else ""

        print(
            f"  電流 a={accel_cur:7.1f}mA b={brake_cur:7.1f}mA"
            f"  位置 a={accel_pos:6d} b={brake_pos:6d}"
            f"  (閾値:{limit_ma:.0f}mA){alarm_str}"
        )

        if accel_cur > limit_ma:
            print(f"過電流検知: accel {accel_cur:.1f}mA → 原点復帰開始")
            await _home_return_both_axes(accel, brake)
            return True

        if brake_cur > limit_ma:
            print(f"過電流検知: brake {brake_cur:.1f}mA → 原点復帰開始")
            await _home_return_both_axes(accel, brake)
            return True

    return False


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
    print(f"Modbus 接続完了（アクセル軸: {_ACCEL_PORT}、ブレーキ軸: {_BRAKE_PORT}）")

    # 起動時前処理: PMSL → reset_alarm → servo_on → home_return
    print("起動時前処理: enable_modbus_control → reset_alarm → servo_on...")
    await accel.enable_modbus_control()
    await brake.enable_modbus_control()
    await accel.reset_alarm()
    await brake.reset_alarm()
    await accel.servo_on()
    await brake.servo_on()
    await _home_return_both_axes(accel, brake)

    print("起動完了: 制御ループ開始")
    print(f"過電流閾値: {TEST_OVERCURRENT_LIMIT_MA:.1f}mA  速度: {_MOVE_SPEED_MM_S}mm/s")
    print(f"往復移動: 0 ↔ {_FAR_POSITION} pulse")
    print("-" * 70)
    print("アクチュエータを手で押さえて過電流を発生させるか、")
    print("TEST_OVERCURRENT_LIMIT_MA を移動中電流より低く設定してください。")
    print("※ 位置[pulse]が変化していればモーターは動作中です")
    print("※ ALMC=0x0000 ならアラームなし。0x0000 以外はアラームコードを確認してください")
    print("Ctrl+C で終了。")
    print("-" * 70)

    try:
        going_to_far = True
        while True:
            target = _FAR_POSITION if going_to_far else 0
            going_to_far = not going_to_far

            print(f"移動指令: → {target} pulse ({_MOVE_SPEED_MM_S}mm/s)")
            try:
                await asyncio.gather(
                    accel.move_to_position(target, speed_mm_s=_MOVE_SPEED_MM_S),
                    brake.move_to_position(target, speed_mm_s=_MOVE_SPEED_MM_S),
                )
            except Exception as e:
                print(f"ERROR: 位置指令失敗: {e}")
                break

            # 移動中に 20回（約1秒）電流・アラームを監視
            overcurrent = await _poll_current_and_check(
                accel, brake, TEST_OVERCURRENT_LIMIT_MA, poll_count=20
            )
            if overcurrent:
                print("過電流検知による原点復帰を確認しました。スクリプトを終了します。")
                break

    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\nCtrl+C 検知: クリーンアップ中...")
        await _home_return_both_axes(accel, brake)
    finally:
        await accel.close()
        await brake.close()
        print("クリーンアップ完了")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
