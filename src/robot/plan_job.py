# ruff: noqa: E501
"""計画だけを作るジョブ（``--plan-only``）と、作った計画を手順 24 へ渡す部分（``--plan-in``）。

モード走行の計画（``ModeDriveSetup``: FF モデルの読み込みと開度の列。作成に約 30 秒）は、設定とモードだけで作れて
ハードに触らない。Web の「開始」ポップアップの間にこのジョブで作ってファイルへ置き、走行のランナーは
``--plan-in`` で読むだけにする（読み込みは 0 秒）。core は直さない（手順 24 の ``prepare`` を run.py が差し替える）。

ファイルは pickle の辞書 ``{"version": 1, "vehicle": 車ID, "mode": モード名, "created": ISO 時刻, "setup": ModeDriveSetup}``。
自分のサーバーが作ったファイルだけを読む前提（pickle は信頼できる入力のみ）。
"""

from __future__ import annotations

import asyncio
import os
import pickle
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from src.robot import config_store, events
from src.robot.core import main as core_main
from src.robot.core.config import ConfigError
from src.robot.core.mode_drive import load_mode, prepare_mode_drive_from
from src.robot.core.term import banner, say

PLAN_VERSION = 1


def write_plan_file(path: Path, *, vehicle: str, mode: str, setup: Any) -> None:
    """計画を一時ファイルへ書いてから置き換える（読む側が書きかけを見ない）。"""
    payload = {
        "version": PLAN_VERSION,
        "vehicle": vehicle,
        "mode": mode,
        "created": datetime.now().isoformat(timespec="seconds"),
        "setup": setup,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with tmp.open("wb") as f:
        pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)
    tmp.replace(path)


def read_plan_file(path: Path, *, vehicle: str, mode: str) -> tuple[Any, str]:
    """計画ファイルを読む。車ID・モード名が違う・読めないなら ConfigError。(setup, 作成時刻) を返す。"""
    try:
        with path.open("rb") as f:
            payload = pickle.load(f)  # noqa: S301 - 自分が作ったファイルだけを読む
    except Exception as exc:  # noqa: BLE001
        raise ConfigError(f"計画ファイルを読めません（{path}、{type(exc).__name__}: {exc}）") from exc
    if not isinstance(payload, dict) or payload.get("version") != PLAN_VERSION:
        raise ConfigError(f"計画ファイルの形式が違います（{path}）")
    if payload.get("vehicle") != vehicle:
        raise ConfigError(f"計画ファイルの車ID {payload.get('vehicle')!r} が走る車 {vehicle!r} と違います")
    if payload.get("mode") != mode:
        raise ConfigError(f"計画ファイルのモード {payload.get('mode')!r} が走るモード {mode!r} と違います")
    return payload["setup"], str(payload.get("created", ""))


def make_plan_in_prepare(path: Path, *, vehicle: str, mode: str):
    """手順 24 の ``prepare`` の代わり: ファイルの計画を ``ctx.mode_setup`` に入れる。"""

    async def prepare(ctx: core_main.RunContext) -> None:
        setup, created = read_plan_file(path, vehicle=vehicle, mode=mode)
        ctx.mode_setup = setup
        say(f"計画をファイルから読み込みました: {mode}（作成 {created}）")

    return prepare


def _finish(code: int, error: str, t0: float) -> int:
    events.emit("state", phase="done", exit_code=code)
    events.emit(
        "result",
        exit_code=code,
        meaning=events.exit_meaning(code),
        error=error or None,
        elapsed_s=round(time.monotonic() - t0, 3),
    )
    return code


async def run_plan_only(args: Any, *, pick_mode_name_fn: Any, fetch_rows_fn: Any) -> int:
    """``--plan-only``: 設定とモードを読み、計画を作って ``--plan-out`` へ書く。ハードには接続しない。"""
    t0 = time.monotonic()
    out: Path = args.plan_out
    tmp = out.with_name(out.name + ".tmp")
    loop = asyncio.get_running_loop()

    def _on_signal(sig_name: str) -> None:
        # 計画は重い計算（スレッド）の途中なので、書きかけを消してすぐ終える
        for p in (tmp, out):
            p.unlink(missing_ok=True)
        say(f"中断しました（{sig_name}）。計画ファイルは消しました")
        code = 130 if sig_name == "SIGINT" else 143
        _finish(code, "", t0)
        sys.stdout.flush()
        os._exit(code)

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _on_signal, sig.name)

    banner(f"計画の作成（車 {args.vehicle} / モード {args.mode}）")
    events.emit("state", phase="planning", mode=args.mode)
    try:
        cfg, _merged = config_store.load_vehicle_config(
            args.vehicle, vehicles_dir=args.vehicles_dir, common_path=args.common
        )
        from src.robot.core.mode_drive import resolve_database_url  # noqa: PLC0415

        composed = [str(c.get("name")) for c in cfg.modes.composed_modes]
        rows = [] if args.mode in composed else await fetch_rows_fn(resolve_database_url(cfg))
        name = pick_mode_name_fn(args.mode, rows, composed)
        say(f"走行モード: {name!r}")
        mode = await load_mode(cfg, name)
        setup = await asyncio.to_thread(prepare_mode_drive_from, cfg, mode)
        write_plan_file(out, vehicle=args.vehicle, mode=name, setup=setup)
    except ConfigError as exc:
        tmp.unlink(missing_ok=True)
        say(f"設定エラー: {exc}")
        return _finish(2, str(exc), t0)
    except Exception as exc:  # noqa: BLE001
        tmp.unlink(missing_ok=True)
        err = f"{type(exc).__name__}: {exc}"
        say(f"計画の作成に失敗しました: {err}")
        return _finish(5, err, t0)
    say(f"計画を書きました: {out}（{time.monotonic() - t0:.1f} s）")
    events.emit("state", phase="planned", mode=name, path=str(out))
    return _finish(0, "", t0)
