"""PCA9685 + SG90 サーボ疎通確認スクリプト。

用意した PCA9685（I2C 16ch PWM ドライバ）と SG90 サーボ 1 個が、
配線・電源・PWM 諸元どおりに動くかを実機で目視確認するための手動スクリプト。
ボタンサーボ機能（Post-MVP / ButtonServoDriver）の本実装前の最小疎通確認用。

adafruit ライブラリは使わず、既存の smbus2 で PCA9685 レジスタを直接制御する
（docs/architecture.md が PCA9685 制御で smbus2 の代替利用を許容）。

配線（docs/architecture.md L278-293 / docs/functional-design.md L395-428）:
    Raspberry Pi 5              PCA9685
    ------------------------    ---------------------------------
    GPIO2/SDA1  物理ピン3   →   SDA
    GPIO3/SCL1  物理ピン5   →   SCL
    GPIO22      物理ピン15  →   OE   (負論理。LOW=出力有効／HIGH=出力無効)
    3.3V        物理ピン1   →   VCC  (ロジック電源)
    GND         物理ピン6   →   GND  (ロジックGND。電源GNDと共通にする)
    (接続しない)            →   V+   (サーボ電源 = 外部5V。本体電源と分離)

    SG90 (ch0 の3ピンヘッダ):
    オレンジ(信号) → PWM /  赤(+) → V+(外部5V) /  茶(GND) → GND

    ※ V+ は外部5V電源から給電し、その GND を PCA9685 の GND と共通接続すること。
    ※ I2Cアドレス 0x40（A0-A5開放）、PWM 50Hz。
    ※ OE未接続の基板でも内蔵プルダウンでLOW(有効)側になるものが多いが、
      本スクリプトでのOE操作確認には GPIO22 への配線が必要。

事前準備:
    1. I2C1 を有効化: sudo raspi-config nonint do_i2c 0
       （または /boot/firmware/config.txt に dtparam=i2c_arm=on を追記して再起動）
    2. 疎通確認: i2cdetect -y 1  → アドレス 40 が見えること

操作方法:
    e / w   : ±5° 移動
    d / s   : ±1° 移動
    Enter   : +45°（180°を超えたら0°へ戻る）
    0 / 5 / 9 : 0° / 90° / 180° へ移動
    p       : 押下デモ（待機角 ↔ 押下角 を1往復）
    o       : OE=HIGH（PCA9685の全出力をハードウェア的に無効化。非常停止相当）
    i       : OE=LOW（出力を有効化。o で無効化した後の復帰）
    q       : 終了（サーボを0°に戻し、出力オフ）

実行方法:
    .venv/bin/python tests/hardware/test_servo_pca9685.py         # ch0 を使用
    .venv/bin/python tests/hardware/test_servo_pca9685.py 3       # ch3 を使用（角括弧は付けない）
"""

from __future__ import annotations

import os
import sys
import time

from smbus2 import SMBus, i2c_msg

# --- 接続設定 ---
_I2C_BUS = 1  # I2C1 (GPIO2/GPIO3)
_PCA9685_ADDR = 0x40  # 既定アドレス（A0-A5 開放）
_PWM_FREQ_HZ = 50  # SG90 標準
_OE_GPIO = 22  # PCA9685 OE端子（物理ピン15、負論理）
_GPIO_CHIP = 0

# --- 角度 ⇔ パルス幅 ---
# SG90 の一般的なパルス幅。個体差があるため必要に応じて調整する。
_MIN_PULSE_US = 500.0  # 0°
_MAX_PULSE_US = 2500.0  # 180°
_ANGLE_MIN = 0.0
_ANGLE_MAX = 180.0

# --- デモ用ポジション（本番の待機/押下相当） ---
_REST_ANGLE = 0.0  # 待機角
_PRESS_ANGLE = 90.0  # 押下角
_PRESS_HOLD_S = 0.6  # 押下保持時間

_JOG_LARGE = 5.0  # e/w キー
_JOG_SMALL = 1.0  # d/s キー
_MOVE_SETTLE_S = 0.3  # 角度指令後の安定待機

# --- PCA9685 レジスタ ---
_MODE1 = 0x00
_MODE2 = 0x01
_PRESCALE = 0xFE
_LED0_ON_L = 0x06  # 以降 ch ごとに +4
_ALL_LED_ON_L = 0xFA
_ALL_LED_ON_H = 0xFB
_ALL_LED_OFF_L = 0xFC
_ALL_LED_OFF_H = 0xFD

_MODE1_SLEEP = 0x10
_MODE1_AI = 0x20  # オートインクリメント
_MODE1_RESTART = 0x80
_MODE2_OUTDRV = 0x04


class _Pca9685:
    """PCA9685 の最小 PWM ドライバ（smbus2 直叩き）。"""

    def __init__(self, bus_num: int, address: int) -> None:
        self._addr = address
        try:
            self._bus = SMBus(bus_num)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"/dev/i2c-{bus_num} が見つかりません。I2C1 が無効です。\n"
                "  有効化: sudo raspi-config nonint do_i2c 0  （または config.txt に "
                "dtparam=i2c_arm=on を追記して再起動）"
            ) from exc

    def _write8(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self._addr, reg, value & 0xFF)

    def _read8(self, reg: int) -> int:
        return self._bus.read_byte_data(self._addr, reg)

    def init_50hz(self) -> None:
        """MODE 設定と PRESCALE 書き込みで PWM 周波数を設定する。"""
        try:
            # 一旦全出力オフ
            self._write8(_ALL_LED_ON_L, 0x00)
            self._write8(_ALL_LED_ON_H, 0x00)
            self._write8(_ALL_LED_OFF_L, 0x00)
            self._write8(_ALL_LED_OFF_H, 0x00)
            self._write8(_MODE2, _MODE2_OUTDRV)
            self._write8(_MODE1, _MODE1_AI)
            time.sleep(0.005)

            prescale = round(25_000_000.0 / (4096.0 * _PWM_FREQ_HZ)) - 1
            old_mode = self._read8(_MODE1)
            # SLEEP 中でないと PRESCALE は書けない
            self._write8(_MODE1, (old_mode & ~_MODE1_RESTART) | _MODE1_SLEEP)
            self._write8(_PRESCALE, prescale)
            self._write8(_MODE1, old_mode)
            time.sleep(0.005)
            # RESTART
            self._write8(_MODE1, old_mode | _MODE1_RESTART | _MODE1_AI)
        except OSError as exc:
            raise RuntimeError(
                f"PCA9685(0x{self._addr:02X}) と通信できません: {exc}\n"
                "  確認: i2cdetect -y 1 で 40 が見えるか / SDA・SCL・GND 配線 / VCC(3.3V) 給電"
            ) from exc

    def _set_pwm(self, channel: int, on: int, off: int) -> None:
        base = _LED0_ON_L + 4 * channel
        data = [on & 0xFF, (on >> 8) & 0x0F, off & 0xFF, (off >> 8) & 0x0F]
        # オートインクリメント書き込み
        msg = i2c_msg.write(self._addr, [base, *data])
        self._bus.i2c_rdwr(msg)

    def set_angle(self, channel: int, angle: float) -> None:
        """角度[deg] を SG90 のパルス幅に変換して出力する。"""
        angle = max(_ANGLE_MIN, min(_ANGLE_MAX, angle))
        pulse_us = _MIN_PULSE_US + (_MAX_PULSE_US - _MIN_PULSE_US) * (angle / _ANGLE_MAX)
        # 1 周期 = 1e6 / freq [us] を 4096 分割
        period_us = 1_000_000.0 / _PWM_FREQ_HZ
        count = int(round(pulse_us / period_us * 4096.0))
        count = max(0, min(4095, count))
        self._set_pwm(channel, 0, count)

    def all_off(self) -> None:
        """全チャンネルの PWM 出力を停止する（安全停止）。"""
        try:
            self._write8(_ALL_LED_ON_L, 0x00)
            self._write8(_ALL_LED_ON_H, 0x00)
            self._write8(_ALL_LED_OFF_L, 0x00)
            self._write8(_ALL_LED_OFF_H, 0x10)  # bit12=1: full-off
        except OSError:
            pass

    def close(self) -> None:
        self._bus.close()


class _OeControl:
    """PCA9685 OE端子（GPIO22）の ON/OFF 制御（lgpio 使用）。

    docs/architecture.md の非常停止ハードウェア冗長系と同じ GPIO22/物理ピン15 を使う。
    負論理: LOW=出力有効、HIGH=出力無効。
    """

    def __init__(self, gpio_pin: int) -> None:
        import lgpio  # noqa: PLC0415

        self._lgpio = lgpio
        self._pin = gpio_pin
        try:
            self._handle = lgpio.gpiochip_open(_GPIO_CHIP)
            lgpio.gpio_claim_output(self._handle, self._pin, 0)  # 初期LOW=出力有効
        except Exception as exc:
            raise RuntimeError(
                f"GPIO{gpio_pin}(OE) の初期化に失敗しました: {exc}\n"
                "  確認: GPIO22(物理ピン15)の配線 / 他プロセスがGPIOを使用していないか"
            ) from exc

    def enable(self) -> None:
        """OE=LOW: PCA9685 の全出力を有効化する。"""
        self._lgpio.gpio_write(self._handle, self._pin, 0)

    def disable(self) -> None:
        """OE=HIGH: PCA9685 の全出力をハードウェア的に無効化する。"""
        self._lgpio.gpio_write(self._handle, self._pin, 1)

    def close(self) -> None:
        try:
            self.enable()
        finally:
            self._lgpio.gpiochip_close(self._handle)


def _read_single_key() -> str:
    """raw モードで1文字読み取り（ブロッキング）。"""
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch


def _press_demo(pca: _Pca9685, channel: int) -> None:
    """待機角 → 押下角 → 待機角 の1往復（本番の押下相当）。"""
    print(f"  押下デモ: 待機 {_REST_ANGLE:.0f}° → 押下 {_PRESS_ANGLE:.0f}° → 待機")
    pca.set_angle(channel, _REST_ANGLE)
    time.sleep(_MOVE_SETTLE_S)
    pca.set_angle(channel, _PRESS_ANGLE)
    time.sleep(_PRESS_HOLD_S)
    pca.set_angle(channel, _REST_ANGLE)
    time.sleep(_MOVE_SETTLE_S)


def main() -> int:
    channel = 0
    if len(sys.argv) > 1:
        try:
            channel = int(sys.argv[1])
        except ValueError:
            print(f"チャンネル指定が不正です: {sys.argv[1]}")
            return 1
    if not 0 <= channel <= 15:
        print(f"チャンネルは 0-15 で指定してください: {channel}")
        return 1

    print(f"PCA9685(0x{_PCA9685_ADDR:02X}) / ch{channel} / {_PWM_FREQ_HZ}Hz で接続します。")
    try:
        pca = _Pca9685(_I2C_BUS, _PCA9685_ADDR)
        pca.init_50hz()
    except RuntimeError as exc:
        print(f"\n[エラー] {exc}")
        return 1

    try:
        oe = _OeControl(_OE_GPIO)
    except RuntimeError as exc:
        print(f"\n[エラー] {exc}")
        pca.all_off()
        pca.close()
        return 1

    angle = 0.0
    oe_enabled = True
    try:
        pca.set_angle(channel, angle)
        print("接続 OK。サーボを 0° に移動しました。")
        print(
            "操作: Enter=+45°  e/w=±5°  d/s=±1°  0/5/9=0/90/180°  "
            "p=押下デモ  o=OE無効化  i=OE有効化  q=終了"
        )
        while True:
            oe_label = "有効" if oe_enabled else "無効"
            print(f"\r現在角度: {angle:6.1f}°  OE:{oe_label}   > ", end="", flush=True)
            key = _read_single_key()
            if key in ("q", "\x03"):  # q or Ctrl-C
                break
            elif key in ("\r", "\n"):  # Enter
                angle = 0.0 if angle + 45.0 > _ANGLE_MAX else angle + 45.0
            elif key == "e":
                angle += _JOG_LARGE
            elif key == "w":
                angle -= _JOG_LARGE
            elif key == "d":
                angle += _JOG_SMALL
            elif key == "s":
                angle -= _JOG_SMALL
            elif key == "0":
                angle = 0.0
            elif key == "5":
                angle = 90.0
            elif key == "9":
                angle = 180.0
            elif key == "p":
                print()
                _press_demo(pca, channel)
                angle = _REST_ANGLE
                continue
            elif key == "o":
                oe.disable()
                oe_enabled = False
                print("\n  OE=HIGH: PCA9685 の全出力をハードウェア的に無効化しました。")
                continue
            elif key == "i":
                oe.enable()
                oe_enabled = True
                print("\n  OE=LOW: PCA9685 の出力を有効化しました。")
                continue
            else:
                continue
            angle = max(_ANGLE_MIN, min(_ANGLE_MAX, angle))
            pca.set_angle(channel, angle)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        oe.enable()  # OEを有効に戻してからサーボを0°へ
        try:
            pca.set_angle(channel, 0.0)
            time.sleep(_MOVE_SETTLE_S)
        except OSError:
            pass
        pca.all_off()
        pca.close()
        oe.close()
        print("\nサーボを 0° に戻し、出力をオフ・OEを有効に戻して終了しました。")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    sys.exit(main())
