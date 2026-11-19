# ruff: noqa: E501
"""手順 1（初期化）を含まないジョブの、動かすのに要る最小の接続（D-15c、2026-11-16）。

手順 1 は初期化ページ（init_only ジョブ）だけで行う。モード走行・学習・IGN-OFF のジョブは初期化の表や
確認を出さず、ここで「アクセル/ブレーキ/CAN を開く → Modbus 操作権 → （アラームが出ていれば）リセット →
サーボ ON」だけを静かに行う。原点復帰はしない（直前のジョブの終了処理が済ませている。位置が原点にあるかは
走行前チェックの「アクチュエータ位置」が確かめる）。

core の ``run_steps`` は ``ctx.hardware`` が初期化済み（``hardware.initialized``）でないと手順を動かせない
（``require_hardware``）。そこで run_steps に入る前にここで ``ctx.hardware`` を作って初期化済みにする。
失敗しても ``ctx.hardware`` は残るので、呼び出し側が ``core_main.shutdown`` で片付ける。
"""

from __future__ import annotations

import asyncio
from typing import Any

from src.robot.core.config import ResearchConfig
from src.robot.core.hardware import (
    InitializationError,
    ResearchHardware,
    build_hardware,
)
from src.robot.core.term import say


async def connect_quietly(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """接続とサーボ ON だけを行う（initialized は呼び出し側が立てる）。失敗は InitializationError。"""
    # 一部だけ開いて残りが失敗しても shutdown が閉じられるよう、試行前に立てる（core の _connect_all と同じ）
    hw.connected = True
    try:
        await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
        for axis in (hw.brake, hw.accel):  # ブレーキを先に（core の _check_servo_comm と同じ）
            await axis.enable_modbus_control()
        alarms = [
            name
            for name, axis in (("ブレーキ軸", hw.brake), ("アクセル軸", hw.accel))
            if await axis.is_alarm_active()
        ]
        if alarms:
            say(f"アラームが出ているためリセットします: {', '.join(alarms)}")
            await asyncio.gather(hw.accel.reset_alarm(), hw.brake.reset_alarm())
            remaining = [
                name
                for name, axis in (("ブレーキ軸", hw.brake), ("アクセル軸", hw.accel))
                if await axis.is_alarm_active()
            ]
            if remaining:
                raise InitializationError(f"アラームが残っています: {', '.join(remaining)}")
        await asyncio.gather(hw.accel.servo_on(), hw.brake.servo_on())
        if cfg.checks.init_ups:
            await hw.ups.start_polling()  # 走行前チェックが UPS 残量を読む構成のとき
    except InitializationError:
        raise
    except Exception as exc:
        raise InitializationError(f"接続: {type(exc).__name__}: {exc}") from exc


async def prepare_hardware(ctx: Any) -> ResearchHardware:
    """``ctx.hardware`` を作り、最小の接続を済ませて初期化済みにする（ctx は core の RunContext）。"""
    cfg: ResearchConfig = ctx.config
    hw = build_hardware(cfg, ctx.hw_mode)
    ctx.hardware = hw  # 失敗しても core_main.shutdown で片付けられるよう、構築できた時点で持たせる
    if hw.is_real:
        say("実機モード: アクチュエータが物理的に動きます。周囲の安全を確認してください")
    say("初期化ページで初期化済みの前提です。通信（サーボ・CAN）とサーボ ON だけ行います")
    await connect_quietly(hw, cfg)
    hw.initialized = True
    say("接続完了（原点は走行前チェックが確かめます）")
    return hw
