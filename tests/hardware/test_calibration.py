"""ペダルに触れた位置（min）と上限開度（max、機構保護）の測定スクリプト。

アクチュエータを手動で少しずつ動かし、1 軸ごとに Enter を 2 回押す。
    1 回目: ペダルに触れた位置（min）で確定
    2 回目: そのままジョグを続け、これ以上踏ませたくない位置（max）で確定
位置を開度 [%] に直してターミナルに表示する。表示された値をユーザーが
tests/research/config_testVehicle.yaml の vehicle 節
（min_accel_opening_pct・max_accel_opening_pct・min_brake_opening_pct・max_brake_opening_pct）
に書き写す。

2026-10-08 段2l-6（ProblemReport_20261001）: 手順2 の不感帯の測定をやめ、ここで測った min を
待機位置かつ効く開度 u の原点に使う（以前は max だけを測り、ブレーキは --brake のときだけだった）。
2026-10-08 段2l-7: 取り付け直したら 4 値を測って yaml に書き写せば、前のモデルのまま手順3 を走れる
（動きの比率が変わらない前提。表は u＝開度−min で持ち、停車保持は min からの量で持つため）。

開度の定義（tests/research/vehicle.py と同じ。本番のキャリブレーションは使わない）:
    0%   = 原点復帰位置（0 pulse = 0.00 mm）
    100% = ストローク限界（9500 pulse = 95.00 mm）

操作方法（ジョグモード）:
    e/w  : ±1.0mm 移動
    d/s  : ±0.5mm 移動
    Enter: 現在位置を確定（1 回目 = 触れた位置 min、2 回目 = 上限 max）
    q    : 中止

表示（最終結果。1 軸 2 行ずつ、アクセル → ブレーキ）:
    acc : 6.50 mm = 6.84%  →  min_accel_opening_pct: 6.8
    acc : 57.00 mm = 60.00%  →  max_accel_opening_pct: 60.0
    brk : ...                →  min_brake_opening_pct: ...
    brk : ...                →  max_brake_opening_pct: ...

接続:
    /dev/actuator_accel  slave_id=1  アクセル軸（udev固定シンボリックリンク FTBB7KRI）
    /dev/actuator_brake  slave_id=1  ブレーキ軸（udev固定シンボリックリンク FTAQUJJJ）

実行方法:
    .venv/bin/python tests/hardware/test_calibration.py               # アクセル → ブレーキ
    .venv/bin/python tests/hardware/test_calibration.py --accel-only  # アクセルだけ
（--brake は廃止。ブレーキも既定で測る）
"""

from __future__ import annotations

import asyncio
import math
import os
import sys
import termios
import tty

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from tests.research.actuator_driver import ActuatorDriver
from tests.research.vehicle import STROKE_LIMIT_PULSE, pulse_to_opening

# --- 接続設定 ---
_ACCEL_PORT = "/dev/actuator_accel"
_BRAKE_PORT = "/dev/actuator_brake"
_ACCEL_SLAVE_ID = 1
_BRAKE_SLAVE_ID = 1  # 両軸とも slave_id=1（各軸が独立した RS-485 バス上）
_BAUD_RATE = 38400

# --- ジョグパラメータ ---
_JOG_STEP_LARGE = 100  # e/w キー: 移動量 [pulse] = 1.0mm
_JOG_STEP_SMALL = 50  # d/s キー: 移動量 [pulse] = 0.5mm
_STEP_INTERVAL_S = 0.15  # ジョグ後の安定待機時間 [s]

# 電流サンプリング（中央値フィルタ）
# P-CON-CB の CNOW は PWM 瞬時電流を返すためランダムなノイズが大きい。
# N回サンプリングして中央値を取ることで PWM スパイクを除去する。
_CURRENT_SAMPLES = 9  # サンプル数（中央値抽出）
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
    現在位置から始めるので、続けて呼ぶと前に確定した位置からジョグを続けられる。
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
            f"\r  位置: {pos:6d} ({pos * 0.01:.2f}mm)"
            f"  開度: {pulse_to_opening(pos):6.2f}%  電流: {cur:7.1f}mA   ",
            end="",
            flush=True,
        )

        key = await loop.run_in_executor(None, _read_single_key)

        if key in ("\r", "\n"):
            actual = await target.read_position()
            print(
                f"\n  → {point_label}: {actual} pulse ({actual * 0.01:.2f} mm)"
                f" = {pulse_to_opening(actual):.2f}%"
            )
            return actual
        elif key in ("q", "\x03"):
            print()
            raise KeyboardInterrupt
        elif key == "e":
            pos = min(STROKE_LIMIT_PULSE, pos + _JOG_STEP_LARGE)
        elif key == "w":
            pos = max(0, pos - _JOG_STEP_LARGE)
        elif key == "d":
            pos = min(STROKE_LIMIT_PULSE, pos + _JOG_STEP_SMALL)
        elif key == "s":
            pos = max(0, pos - _JOG_STEP_SMALL)
        else:
            continue

        await target.move_to_position(pos)
        await asyncio.sleep(_STEP_INTERVAL_S)


async def _measure_upper_limit(
    target: ActuatorDriver,
    other: ActuatorDriver,
    target_is_accel: bool,
    label: str,
) -> tuple[int, int]:
    """片軸の触れた位置（min）と上限位置（max）を手動ジョグで決める（0% は原点復帰位置で固定）。

    原点から始め、1 回目の Enter で min、そのままジョグを続けて 2 回目の Enter で max を確定する。
    （関数名は上限だけを測っていた頃のまま。2026-10-08 段2l-6 で min も測るようにした）

    Returns:
        (触れた位置 min, 上限位置 max) [pulse / 0.01mm]
    """
    print(f"\n=== {label} 触れた位置（min）と上限（max）の設定 ===")
    await target.home_return()
    min_pos = await _jog_axis(
        target, other, target_is_accel, "触れた位置 (min)。ペダルに触れた位置で Enter"
    )
    print("  続けてジョグし、これ以上踏ませたくない位置で Enter")
    max_pos = await _jog_axis(target, other, target_is_accel, "上限 (max)")
    if max_pos <= min_pos:
        print(f"  警告: 上限({max_pos}) が触れた位置({min_pos}) 以下です。測り直してください")

    await target.home_return()
    return min_pos, max_pos


def _format_result(label: str, config_key: str, upper_pos: int) -> str:
    """位置を「mm = % → config に書く値」の1行にする（min にも max にも使う）。

    config に書く値は小数1桁へ切り捨てる（丸めで実測の位置を超えないため）。
    """
    pct = pulse_to_opening(upper_pos)
    config_pct = math.floor(pct * 10.0 + 1e-9) / 10.0
    return f"{label} : {upper_pos * 0.01:.2f} mm = {pct:.2f}%  →  {config_key}: {config_pct:.1f}"


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
    args = sys.argv[1:]
    accel_only = "--accel-only" in args
    if "--brake" in args:
        print("注意: --brake は廃止しました（ブレーキも既定で測る。アクセルだけは --accel-only）")
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
        print("触れた位置（min）・上限開度（max）の測定開始（手動ジョグ方式）")
        print(f"開度の定義: 0% = 原点（0 pulse）, 100% = {STROKE_LIMIT_PULSE} pulse")
        print(
            f"ジョグステップ: 大={_JOG_STEP_LARGE}pulse ({_JOG_STEP_LARGE * 0.01:.1f}mm)"
            f" / 小={_JOG_STEP_SMALL}pulse ({_JOG_STEP_SMALL * 0.01:.1f}mm)"
            f"  安全閾値: {_OVERCURRENT_LIMIT_MA:.0f}mA"
        )
        print("=" * 70)

        results: list[str] = []
        acc_min, acc_max = await _measure_upper_limit(accel, brake, True, "acc")
        results.append(_format_result("acc", "min_accel_opening_pct", acc_min))
        results.append(_format_result("acc", "max_accel_opening_pct", acc_max))
        if not accel_only:
            brk_min, brk_max = await _measure_upper_limit(brake, accel, False, "brk")
            results.append(_format_result("brk", "min_brake_opening_pct", brk_min))
            results.append(_format_result("brk", "max_brake_opening_pct", brk_max))

        print("\n" + "=" * 70)
        for line in results:
            print(line)
        print("上の値を tests/research/config_testVehicle.yaml の vehicle 節に反映してください")
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
