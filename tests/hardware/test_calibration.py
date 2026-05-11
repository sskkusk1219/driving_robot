"""キャリブレーション確認スクリプト。

アクセル・ブレーキのゼロフルを実機で設定し、結果をターミナルに出力する。
キー操作でアクチュエータをジョグし、ペダル位置を目視確認しながらゼロ/フル点を記録する。

操作方法（ジョグモード）:
    +/-  : ±0.5mm 移動
    ./,  : ±0.1mm 移動
    Enter: 現在位置を確定
    q    : 中止

表示（ゼロフル記録後・最終結果）:
    acc : 5.20 mm = 0%, 72.30 mm = 100%  brk : 4.80 mm = 0%, 68.50 mm = 100%

接続:
    /dev/ttyUSB0  slave_id=1  アクセル軸
    /dev/ttyUSB1  slave_id=1  ブレーキ軸

実行方法: .venv/bin/python tests/hardware/test_calibration.py
"""

from __future__ import annotations

import asyncio
import os
import sys
import termios
import tty

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from src.infra.actuator_driver import ActuatorDriver

# --- 接続設定 ---
_ACCEL_PORT = "/dev/ttyUSB0"
_BRAKE_PORT = "/dev/ttyUSB1"
_ACCEL_SLAVE_ID = 1
_BRAKE_SLAVE_ID = 1  # 両軸とも slave_id=1（各軸が独立した RS-485 バス上）
_BAUD_RATE = 38400

# --- ジョグパラメータ ---
_JOG_STEP_LARGE = 100  # +/- キー: 移動量 [pulse] = 1.0mm
_JOG_STEP_SMALL = 50   # ./,  キー: 移動量 [pulse] = 0.1mm
_STEP_INTERVAL_S = 0.15  # ジョグ後の安定待機時間 [s]

# 電流サンプリング（中央値フィルタ）
# P-CON-CB の CNOW は PWM 瞬時電流を返すためランダムなノイズが大きい。
# N回サンプリングして中央値を取ることで PWM スパイクを除去する。
_CURRENT_SAMPLES = 9               # サンプル数（中央値抽出）
_CURRENT_SAMPLE_INTERVAL_S = 0.01  # サンプル間隔 [s]

# 安全閾値: この値を超えたら即 RuntimeError として中断する
_OVERCURRENT_LIMIT_MA = 500
_OVERCURRENT_EMERGENCY_MA = _OVERCURRENT_LIMIT_MA * 2


def _read_single_key() -> str:
    """raw モードで1文字読み取り（ブロッキング）。"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


async def _read_current_trimmed(driver: ActuatorDriver) -> float:
    """電流を _CURRENT_SAMPLES 回サンプリングして中央値を返す。

    P-CON-CB CNOW の PWM 瞬時ノイズを除去するため、中央値フィルタを使用する。
    """
    readings: list[float] = []
    for i in range(_CURRENT_SAMPLES):
        try:
            readings.append(await driver.read_current())
        except Exception as e:
            print(f"  ERROR: 電流値読み取り失敗: {e}")
        if i < _CURRENT_SAMPLES - 1:
            await asyncio.sleep(_CURRENT_SAMPLE_INTERVAL_S)
    if not readings:
        return 0.0
    if len(readings) < 3:
        return sum(readings) / len(readings)
    readings.sort()
    return readings[len(readings) // 2]


async def _read_state(driver: ActuatorDriver) -> tuple[float, int]:
    """電流値（中央値フィルタ）と現在位置 [pulse] を取得する。"""
    current = await _read_current_trimmed(driver)
    try:
        position = await driver.read_position()
    except Exception as e:
        print(f"  ERROR: 位置読み取り失敗: {e}")
        position = 0
    return current, position


async def _jog_axis(
    target: ActuatorDriver,
    other: ActuatorDriver,
    target_is_accel: bool,
    point_label: str,
) -> int:
    """キー操作でアクチュエータをジョグし、Enter で確定した位置を返す。

    e/w: ±_JOG_STEP_LARGE pulse  d/s: ±_JOG_STEP_SMALL pulse  Enter: 確定  q: 中止
    """
    loop = asyncio.get_event_loop()
    pos = await target.read_position()

    print(f"  ジョグ操作で {point_label} に合わせてください")
    print(
        f"  [e/w: ±{_JOG_STEP_LARGE}pulse ({_JOG_STEP_LARGE * 0.01:.1f}mm)]"
        f"  [d/s: ±{_JOG_STEP_SMALL}pulse ({_JOG_STEP_SMALL * 0.01:.1f}mm)]"
        f"  [Enter: 確定]  [q: 中止]"
    )

    while True:
        cur, _ = await _read_state(target)

        if cur > _OVERCURRENT_EMERGENCY_MA:
            print()
            raise RuntimeError(f"過電流上限超過: {cur:.1f}mA > {_OVERCURRENT_EMERGENCY_MA:.0f}mA")

        print(
            f"\r  位置: {pos:6d} ({pos * 0.01:.2f}mm)  電流: {cur:7.1f}mA   ",
            end="",
            flush=True,
        )

        key = await loop.run_in_executor(None, _read_single_key)

        if key in ('\r', '\n'):
            actual = await target.read_position()
            print(f"\n  → {point_label}: {actual} pulse ({actual * 0.01:.2f} mm)")
            return actual
        elif key in ('q', '\x03'):
            print()
            raise KeyboardInterrupt
        elif key == 'e':
            pos = pos + _JOG_STEP_LARGE
        elif key == 'w':
            pos = max(0, pos - _JOG_STEP_LARGE)
        elif key == 'd':
            pos = pos + _JOG_STEP_SMALL
        elif key == 's':
            pos = max(0, pos - _JOG_STEP_SMALL)
        else:
            continue

        await target.move_to_position(pos)
        await asyncio.sleep(_STEP_INTERVAL_S)


async def _calibrate_one_axis(
    target: ActuatorDriver,
    other: ActuatorDriver,
    target_is_accel: bool,
    label: str,
) -> tuple[int, int]:
    """片軸のゼロフル位置を手動ジョグで設定する。

    Returns:
        (zero_pos, full_pos): ゼロ点・フル点の位置 [pulse / 0.01mm]
    """
    print(f"\n=== {label} ゼロ点設定 ===")
    await target.home_return()
    zero_pos = await _jog_axis(target, other, target_is_accel, "ゼロ点")

    print(f"\n=== {label} フル点設定 ===")
    full_pos = await _jog_axis(target, other, target_is_accel, "フル点")

    await target.home_return()
    return zero_pos, full_pos


async def _home_return_both(accel: ActuatorDriver, brake: ActuatorDriver) -> None:
    """両軸を並列で原点復帰する。"""
    print("原点復帰（両軸並列）...")
    try:
        await asyncio.gather(accel.home_return(), brake.home_return())
        print("両軸 原点復帰完了")
    except TimeoutError as e:
        print(f"ERROR: 原点復帰タイムアウト: {e}")
    except Exception as e:
        print(f"ERROR: 原点復帰エラー: {e}")


async def main() -> None:
    accel = ActuatorDriver(port=_ACCEL_PORT, slave_id=_ACCEL_SLAVE_ID, baud_rate=_BAUD_RATE)
    brake = ActuatorDriver(port=_BRAKE_PORT, slave_id=_BRAKE_SLAVE_ID, baud_rate=_BAUD_RATE)

    print("Modbus 接続中...")
    try:
        await accel.connect()
        await brake.connect()
    except ConnectionError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"接続完了 (accel={_ACCEL_PORT}, brake={_BRAKE_PORT})")

    try:
        print("\n起動前処理: enable_modbus_control → reset_alarm → servo_on → home_return...")
        await accel.enable_modbus_control()
        await brake.enable_modbus_control()
        await accel.reset_alarm()
        await brake.reset_alarm()
        await accel.servo_on()
        await brake.servo_on()
        await _home_return_both(accel, brake)
        print("起動完了")

        print("\n" + "=" * 70)
        print("キャリブレーション開始（手動ジョグ方式）")
        print(
            f"ジョグステップ: 大={_JOG_STEP_LARGE}pulse ({_JOG_STEP_LARGE * 0.01:.1f}mm)"
            f" / 小={_JOG_STEP_SMALL}pulse ({_JOG_STEP_SMALL * 0.01:.1f}mm)"
            f"  安全閾値: {_OVERCURRENT_LIMIT_MA:.0f}mA"
        )
        print("=" * 70)

        accel_zero, accel_full = await _calibrate_one_axis(accel, brake, True, "acc")
        brake_zero, brake_full = await _calibrate_one_axis(brake, accel, False, "brk")

        print("\n" + "=" * 70)
        print(
            f"acc : {accel_zero * 0.01:.2f} mm = 0%, {accel_full * 0.01:.2f} mm = 100%"
            f"  brk : {brake_zero * 0.01:.2f} mm = 0%, {brake_full * 0.01:.2f} mm = 100%"
        )
        print("=" * 70)

    except (KeyboardInterrupt, asyncio.CancelledError):
        print("\nCtrl+C: クリーンアップ中...")
        await _home_return_both(accel, brake)
    except RuntimeError as e:
        print(f"\nERROR: {e}")
        await _home_return_both(accel, brake)
    except Exception as e:
        print(f"\nERROR (予期しないエラー): {e}")
        await _home_return_both(accel, brake)
    finally:
        await accel.close()
        await brake.close()
        print("クリーンアップ完了")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
