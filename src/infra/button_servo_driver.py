"""ボタンサーボドライバ（PCA9685 + SG90 ×16）。

物理ボタン（エンジンスタート・シフト・オプション）を押下するための I2C PWM ドライバ。
Phase 0 の疎通確認スクリプト（tests/hardware/test_servo_pca9685.py の _Pca9685）を
本番用ドライバへ発展させたもの。16ch・待機/押下の2ポジション・全ch解放（release_all）を持つ。

用語注意: ここでの「サーボ」は SG90（ボタン押下用）を指す。IAI アクチュエータの
「サーボON/サーボ状態」（励磁）とは別物なので、本モジュールでは「ボタンサーボ」と呼ぶ。

設計方針:
- ActuatorDriver と同様、Protocol ベースの I/F（connect/press/release_all）で
  非ハード環境向けスタブ（src/app/stubs._StubButtonServo）と差し替え可能にする。
- 押下角度は全チャンネル共通のグローバル設定（待機角/押下角）。押下時間のみ呼び出し側
  （タイムスケジュール）でボタンごとに可変にする。
- PCA9685 の I2C バスはアクセル・ブレーキ用の RS-485（Modbus RTU）と独立しているため、
  50ms/100ms 制御ループとバス競合しない。同期 I2C 呼び出しは asyncio.to_thread で包む。
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable
from typing import Protocol

_logger = logging.getLogger(__name__)

# チャンネル ⇔ ボタン マッピング（docs/functional-design.md）
CH_ENGINE_START = 0
CH_SHIFT_P = 1
CH_SHIFT_N = 2
CH_SHIFT_D = 3
# ch4-15 = オプション1-12
CH_MIN = 0
CH_MAX = 15

# 角度 ⇔ パルス幅（SG90 標準。個体差があるため必要に応じて調整する）
_MIN_PULSE_US = 500.0  # 0°
_MAX_PULSE_US = 2500.0  # 180°
_ANGLE_MAX = 180.0

# PCA9685 レジスタ
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


def angle_to_count(angle: float, pwm_freq_hz: int) -> int:
    """角度[deg] を PCA9685 の 12bit デューティカウントへ変換する（純関数・テスト可能）。"""
    angle = max(0.0, min(_ANGLE_MAX, angle))
    pulse_us = _MIN_PULSE_US + (_MAX_PULSE_US - _MIN_PULSE_US) * (angle / _ANGLE_MAX)
    period_us = 1_000_000.0 / pwm_freq_hz
    count = int(round(pulse_us / period_us * 4096.0))
    return max(0, min(4095, count))


class I2CBusProtocol(Protocol):
    """smbus2.SMBus のうち本ドライバが使う操作のみを抽出したプロトコル。"""

    def write_byte_data(self, addr: int, reg: int, value: int) -> None: ...

    def read_byte_data(self, addr: int, reg: int) -> int: ...

    def write_i2c_block_data(self, addr: int, reg: int, data: list[int]) -> None: ...

    def close(self) -> None: ...


class ButtonServoDriver:
    """PCA9685 経由で 16ch のボタンサーボ（SG90）を押下・解放するドライバ。

    実 I2C バスは connect() 時に smbus2 で開く（bus_factory 差し替えでテスト可能）。
    """

    def __init__(
        self,
        i2c_bus: int = 1,
        address: int = 0x40,
        pwm_freq_hz: int = 50,
        rest_angle: float = 60.0,
        press_angle: float = 110.0,
        move_settle_s: float = 0.3,
        bus_factory: Callable[[int], I2CBusProtocol] | None = None,
    ) -> None:
        self._bus_num = i2c_bus
        self._addr = address
        self._freq = pwm_freq_hz
        self._rest_angle = rest_angle
        self._press_angle = press_angle
        self._move_settle_s = move_settle_s
        # bus_factory を渡すとテスト等でフェイク I2C バスを注入できる（既定は smbus2）。
        self._bus_factory = bus_factory
        self._bus: I2CBusProtocol | None = None

    def _open_bus(self) -> I2CBusProtocol:
        """smbus2 で I2C バスを開く。ハード無し環境ではスタブ（_StubButtonServo）を使うこと。"""
        from smbus2 import SMBus  # noqa: PLC0415

        try:
            return SMBus(self._bus_num)
        except FileNotFoundError as exc:
            raise RuntimeError(
                f"/dev/i2c-{self._bus_num} が見つかりません。I2C が無効です。"
                "（sudo raspi-config nonint do_i2c 0 で有効化）"
            ) from exc

    # ── 低レベル I2C（同期・to_thread から呼ぶ）──────────────────────
    def _write8(self, reg: int, value: int) -> None:
        assert self._bus is not None
        self._bus.write_byte_data(self._addr, reg, value & 0xFF)

    def _read8(self, reg: int) -> int:
        assert self._bus is not None
        return self._bus.read_byte_data(self._addr, reg)

    def _init_pwm(self) -> None:
        """MODE 設定と PRESCALE 書き込みで PWM 周波数を設定する。"""
        # 一旦全出力オフ
        self._write8(_ALL_LED_ON_L, 0x00)
        self._write8(_ALL_LED_ON_H, 0x00)
        self._write8(_ALL_LED_OFF_L, 0x00)
        self._write8(_ALL_LED_OFF_H, 0x00)
        self._write8(_MODE2, _MODE2_OUTDRV)
        self._write8(_MODE1, _MODE1_AI)

        prescale = round(25_000_000.0 / (4096.0 * self._freq)) - 1
        old_mode = self._read8(_MODE1)
        # SLEEP 中でないと PRESCALE は書けない
        self._write8(_MODE1, (old_mode & ~_MODE1_RESTART) | _MODE1_SLEEP)
        self._write8(_PRESCALE, prescale)
        self._write8(_MODE1, old_mode)
        self._write8(_MODE1, old_mode | _MODE1_RESTART | _MODE1_AI)

    def _set_angle(self, channel: int, angle: float) -> None:
        assert self._bus is not None
        count = angle_to_count(angle, self._freq)
        base = _LED0_ON_L + 4 * channel
        data = [0x00, 0x00, count & 0xFF, (count >> 8) & 0x0F]  # on=0, off=count
        self._bus.write_i2c_block_data(self._addr, base, data)

    def _all_rest(self) -> None:
        """全チャンネルを待機角へ。"""
        for ch in range(CH_MIN, CH_MAX + 1):
            self._set_angle(ch, self._rest_angle)

    # ── 公開 API（非同期）───────────────────────────────────────────
    async def connect(self) -> None:
        """I2C バスを開いて PCA9685 を初期化し、全chを待機角へ移動する。"""

        def _do() -> None:
            self._bus = (
                self._bus_factory(self._bus_num)
                if self._bus_factory is not None
                else self._open_bus()
            )
            self._init_pwm()
            self._all_rest()

        await asyncio.to_thread(_do)

    async def check_connection(self) -> bool:
        """PCA9685 と I2C 疎通できるか（MODE1 レジスタ読み取り）を返す。走行前チェック項目8用。"""
        if self._bus is None:
            return False

        def _probe() -> bool:
            self._read8(_MODE1)
            return True

        try:
            return await asyncio.to_thread(_probe)
        except OSError:
            return False

    async def press(self, channel: int, duration_s: float) -> None:
        """指定チャンネルを押下角へ動かし duration_s 保持してから待機角へ戻す。

        待機中のキャンセルや解放時の I2C エラーでボタンを押したまま放置しないよう、
        解放（待機角への復帰）は try/finally で必ず試みる（I1 レビュー指摘）。
        """
        if not CH_MIN <= channel <= CH_MAX:
            raise ValueError(f"channel は {CH_MIN}-{CH_MAX} で指定してください: {channel}")
        if self._bus is None:
            raise RuntimeError("connect() を先に呼んでください")
        await asyncio.to_thread(self._set_angle, channel, self._press_angle)
        try:
            await asyncio.sleep(max(0.0, duration_s))
        finally:
            # shield: press() 自体がキャンセルされても解放処理は打ち切らない
            # （キャンセルが再度伝播しても解放タスクはバックグラウンドで完了を試みる）。
            await asyncio.shield(self._release_channel(channel))

    async def _release_channel(self, channel: int) -> None:
        """指定チャンネルを待機角へ戻す。1回失敗したら1回だけリトライし、
        それでも失敗したらログのみ残す（呼び出し元の finally/キャンセル経路を止めない）。"""
        for attempt in range(2):
            try:
                await asyncio.to_thread(self._set_angle, channel, self._rest_angle)
                return
            except OSError:
                if attempt == 0:
                    continue
                _logger.exception(
                    "ボタンサーボの解放に失敗しました (channel=%d): "
                    "押下状態のまま残っている可能性があります",
                    channel,
                )

    async def release_all(self) -> None:
        """全チャンネルを待機角へ戻す（非常停止・エラー時の安全動作）。

        I2C 失敗は握り潰す（非常停止シーケンスを止めないため）。未接続時は no-op。
        """
        if self._bus is None:
            return
        try:
            await asyncio.to_thread(self._all_rest)
        except OSError:
            _logger.exception("ボタンサーボの release_all で I2C エラー（無視して継続）")

    async def close(self) -> None:
        """I2C バスを閉じる。"""
        if self._bus is not None:
            bus = self._bus
            self._bus = None
            await asyncio.to_thread(bus.close)
