# ruff: noqa: E501
"""ランナー: Web サーバーとは別プロセスで 1 本のジョブ（手順の並び）を走らせる。

    .venv/bin/python -m src.robot.run --vehicle test_vehicle --steps 1,24,25 --limit-s 60 --hw stub
    .venv/bin/python -m src.robot.run --vehicle test_vehicle --interactive      # 手動運転・min 測定

流れは core の ``run_steps`` そのもの（初期化 → 走行前チェック → 手順 → 必ず shutdown）。
RunContext を作って core の run_steps を呼ぶだけで、終了コードも core と同じ
（0 成功 / 2 設定 / 3 未実装 / 4 初期化 / 5 走行 / 6 モデル作成 / 7 走行前チェック / 8 学習結果チェック NG）。
ランナーが足す終了コード: 130（SIGINT で中断）・143（SIGTERM で中断）。どちらも終了処理は済んでいる。

標準出力は core の say() の行がそのまま流れる。機械が読むイベントは別に JSON 1 行
（``@@EV `` 前置き。``--events-fd N`` ならその fd）。形式は src/robot/README.md。
core は直さない。差し替えは src/robot/patches.py の 1 か所。
"""

from __future__ import annotations

import argparse
import asyncio
import dataclasses
import json
import os
import signal
import sys
import time
from collections.abc import Awaitable, Callable, Sequence
from pathlib import Path

from src.robot import (
    config_store,
    emergency_stop,
    events,
    learning_buttons,
    patches,
    plan_job,
    quiet_connect,
    schedule_job,
    schedule_spec,
)
from src.robot.core import main as core_main
from src.robot.core.config import ConfigError, ResearchConfig, update_yaml_values
from src.robot.core.hardware import HW_REAL, HW_STUB, InitializationError
from src.robot.core.mode_drive import resolve_database_url
from src.robot.core.term import banner, say
from src.robot.interactive import InteractiveSession
from src.robot.interrupt import Interrupted, StopController

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_VEHICLES_DIR = REPO_ROOT / "vehicles"
DEFAULT_COMMON = REPO_ROOT / "config" / "robot_common.yaml"

#: ランナーが動かせる手順（RPD A-1: 1・2・12〜14・20〜25。0・4〜11 は持ってこない）。
#: 手順 3（ボタン操作なしのモード走行）は src では使わない（モード走行は常に手順 24。ユーザー決定 2026-11-15）
SUPPORTED_STEPS = frozenset({1, 2, 12, 13, 14, 20, 21, 22, 23, 24, 25})
#: 手順 2・12〜14（学習）。wltp_mode_name を使うので --mode で変えられない
LEARNING_STEPS = frozenset({2, 12, 13, 14})
#: --mode で走るモードを選ぶ手順
MODE_STEPS = frozenset({24})

INTERACTIVE_STEP_NUMBER = 99
#: 非常停止の流れ（減速して停車 → N → IGN-OFF → 原点復帰）を使う手順（走行するもの。RPD_Phase_20261119_EmergencyStop）
EMERGENCY_STEP_NUMBERS = frozenset({2, 12, 13, 14, 24})


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="python -m src.robot.run",
        description="ロボットのジョブ（手順の並び）を 1 本走らせる。Web サーバーとは別プロセス。",
    )
    p.add_argument("--vehicle", required=True, metavar="車ID", help="vehicles/<車ID>/ の車ID")
    target = p.add_mutually_exclusive_group(required=False)  # --plan-only のときは無し（main で検査）
    target.add_argument(
        "--steps", metavar="LIST", help="実行する手順（カンマ区切り。例 1,24 / 1,2 / 1,13）"
    )
    target.add_argument(
        "--interactive",
        action="store_true",
        help="手順 1（初期化）だけ行い、標準入力の JSON 1 行コマンドを受け付ける（手動運転・min 測定）",
    )
    target.add_argument(
        "--serve",
        action="store_true",
        help="常駐ランナー: 手順 1（初期化）＋ PCA9685 の確認をしたら、標準入力の JSON 1 行コマンド"
        "（mode / schedule / steps / interactive / init / stop / quit）を受けて動かし続ける（src/robot/serve.py）",
    )
    target.add_argument(
        "--schedule",
        metavar="ID|名前",
        help="タイムスケジュール（DB の time_schedules の ID か名前。環境変数 DATABASE_URL があればその DB）を"
        "開ループで再生する（接続・サーボ ON → 走行前チェック → 再生 → 終了処理。手順 1 の初期化は行わない）",
    )
    target.add_argument(
        "--schedule-file",
        type=Path,
        metavar="JSON",
        help="タイムスケジュールの JSON（schedule_spec.schedule_to_dict の形）を再生する（Web 側が使う）",
    )
    p.add_argument(
        "--hw",
        choices=(HW_STUB, HW_REAL),
        default=HW_STUB,
        help=f"{HW_REAL} は実アクチュエータが動く（既定 {HW_STUB}）",
    )
    p.add_argument(
        "--mode",
        metavar="ID|名前",
        help="手順 24 で走る走行モード（DB の driving_modes の ID か名前。既定は共通設定の WLTP）",
    )
    p.add_argument("--limit-s", type=float, metavar="S", help="モード走行を先頭 S 秒で打ち切る")
    p.add_argument(
        "--plan-only",
        action="store_true",
        help="ハードに触れず、--mode の計画だけを作って --plan-out へ書く（Web の開始ポップアップの間に使う）",
    )
    p.add_argument("--plan-out", type=Path, metavar="PATH", help="--plan-only の書き出し先（pickle）")
    p.add_argument(
        "--plan-in",
        type=Path,
        metavar="PATH",
        help="手順 24 の計画をこのファイルから読む（--plan-only で作ったもの。車ID・モード名が違えば終了コード 2）",
    )
    p.add_argument(
        "--no-pre-check",
        action="store_true",
        help="走行前チェックを飛ばす（シーケンスの 2 段目以降）",
    )
    p.add_argument("--vehicles-dir", type=Path, default=DEFAULT_VEHICLES_DIR)
    p.add_argument("--common", type=Path, default=DEFAULT_COMMON, help="共通設定 yaml")
    p.add_argument(
        "--events-fd",
        type=int,
        metavar="FD",
        help="イベント（JSON 1 行）を出す fd。無ければ標準出力に @@EV 前置きで出す",
    )
    return p


# ── 引数の検査（純粋） ────────────────────────────────────────────


def parse_steps(text: str) -> list[int]:
    """``--steps`` の文字列 → 手順番号の並び（core の resolve_steps と同じ解釈。重複は許す）。"""
    ns = argparse.Namespace(only=None, steps=text, upto=None)
    return [step.number for step in core_main.resolve_steps(ns)]


def check_job(
    numbers: Sequence[int], *, mode: str | None, limit_s: float | None
) -> str | None:
    """ジョブの組み合わせの誤りを日本語で返す（問題なければ None）。HW に触る前に止める。"""
    unsupported = sorted(set(numbers) - SUPPORTED_STEPS)
    if unsupported:
        return (
            f"手順 {', '.join(map(str, unsupported))} はランナーでは動かせません"
            f"（動かせるのは {', '.join(map(str, sorted(SUPPORTED_STEPS)))}）"
        )
    if limit_s is not None and limit_s <= 0.0:
        return "--limit-s は正の秒数で指定してください"
    if mode is not None:
        if not set(numbers) & MODE_STEPS:
            return "--mode は手順 24 を含むときだけ指定できます"
        learning = sorted(set(numbers) & LEARNING_STEPS)
        if learning:
            return (
                f"--mode は学習の手順（{', '.join(map(str, learning))}）と同じ実行では指定できません"
                "（学習はモードの基準車速・共通設定の WLTP を使うため）"
            )
    if set(numbers) & LEARNING_STEPS and set(numbers) & {23, 24}:
        return (
            "学習の手順（2・12〜14）は自分で Ready・D・N を押すため、手順 23・24 と同じ実行にできません"
            "（Ready でスタートを 2 回押すと IGN-OFF になる）"
        )
    return None


# ── 走行モードの選択（DB 参照） ──────────────────────────────────


def pick_mode_name(
    text: str, rows: Sequence[tuple[str, str]], composed_names: Sequence[str]
) -> str:
    """``--mode`` の文字列（名前か ID）から、core が引く走行モードの名前を決める（純粋）。

    優先順: 合成モード名（modes.composed_modes。DB に無くても core が作る）→ DB の名前 → DB の ID。
    rows は DB の (id, name)。見つからなければ ConfigError。
    """
    if text in composed_names:
        return text
    for _id, name in rows:
        if name == text:
            return name
    for mode_id, name in rows:
        if mode_id == text:
            return name
    raise ConfigError(f"走行モード {text!r} が見つかりません（driving_modes の名前か ID）")


async def _fetch_mode_rows(database_url: str) -> list[tuple[str, str]]:
    import asyncpg  # noqa: PLC0415 - DB を使わない手順では import を要求しない

    try:
        conn = await asyncpg.connect(database_url)
    except Exception as exc:
        raise ConfigError(
            f"走行モードの DB に接続できません（{database_url}、{type(exc).__name__}: {exc}）"
        ) from exc
    try:
        records = await conn.fetch("SELECT id::text AS id, name FROM driving_modes")
    finally:
        await conn.close()
    return [(str(r["id"]), str(r["name"])) for r in records]


async def select_mode(
    cfg: ResearchConfig,
    text: str,
    *,
    fetch: Callable[[str], Awaitable[list[tuple[str, str]]]] = _fetch_mode_rows,
) -> str:
    """走行モードを選び、合成設定（メモリ）と合成 yaml の modes.wltp_mode_name に反映する。

    core は手順 24 のモードを ``modes.wltp_mode_name`` から引く（modes 節・DB の driving_modes）。
    合成 yaml にも書くのは、走行時の設定のスナップショット（merged.yaml のコピー）に選んだモードを
    残すため。この書き換えは車両キーではないので、学習の手順（cfg.save）とは同じ実行にさせない
    （check_job）。
    """
    composed = [str(c.get("name")) for c in cfg.modes.composed_modes]
    rows = [] if text in composed else await fetch(resolve_database_url(cfg))
    name = pick_mode_name(text, rows, composed)
    cfg.modes.wltp_mode_name = name
    update_yaml_values(cfg.source_path, {"modes.wltp_mode_name": name})
    return name


# ── タイムスケジュールの読み込み ────────────────────────────────


async def _fetch_schedule_row(database_url: str, text: str) -> dict[str, object] | None:
    import asyncpg  # noqa: PLC0415 - DB を使わない手順では import を要求しない

    try:
        conn = await asyncpg.connect(database_url)
    except Exception as exc:
        raise ConfigError(
            f"タイムスケジュールの DB に接続できません（{database_url}、{type(exc).__name__}: {exc}）"
        ) from exc
    try:
        rows = await conn.fetch(
            "SELECT id::text AS id, name, pedal_points::text AS pedal_points, "
            "button_events::text AS button_events FROM time_schedules WHERE id::text = $1 OR name = $1",
            text,
        )
    finally:
        await conn.close()
    if not rows:
        return None
    r = rows[0]
    return {
        "id": r["id"],
        "name": r["name"],
        "pedal_points": json.loads(r["pedal_points"]),
        "button_events": json.loads(r["button_events"]),
    }


async def load_schedule(
    args: argparse.Namespace,
    cfg: ResearchConfig,
    *,
    fetch: Callable[[str, str], Awaitable[dict[str, object] | None]] = _fetch_schedule_row,
) -> schedule_spec.ScheduleData:
    """``--schedule-file`` か ``--schedule`` からスケジュールを読む。読めなければ ConfigError。"""
    try:
        if args.schedule_file is not None:
            return schedule_spec.load_schedule_file(args.schedule_file)
        url = resolve_database_url(cfg)
        row = await fetch(url, args.schedule)
        if row is None:
            raise ConfigError(f"タイムスケジュール {args.schedule!r} が見つかりません（time_schedules の ID か名前）")
        return schedule_spec.schedule_from_dict(row)
    except schedule_spec.ScheduleError as exc:
        raise ConfigError(str(exc)) from exc


# ── ジョブの実行 ────────────────────────────────────────────────


class RunState:
    """イベント用の、実行中の記録（最後に result へ書く error など）。"""

    def __init__(self) -> None:
        self.error = ""


def _wrap_handler(
    step: core_main.Step, stop: StopController, state: RunState, *, phase: str
) -> Callable[[core_main.RunContext], Awaitable[None]]:
    async def handler(ctx: core_main.RunContext) -> None:
        events.emit("state", phase=phase, step=step.number, title=step.title)
        try:
            await stop.run(lambda: step.handler(ctx))
        except Interrupted:
            events.emit("state", phase="step_end", step=step.number, ok=False, interrupted=True)
            raise
        except BaseException as exc:
            state.error = f"{type(exc).__name__}: {exc}"
            events.emit(
                "state", phase="step_end", step=step.number, ok=False, error=state.error
            )
            raise
        events.emit("state", phase="step_end", step=step.number, ok=True)

    return handler


def _wrap_prepare(
    step: core_main.Step, stop: StopController, state: RunState
) -> Callable[[core_main.RunContext], Awaitable[None]]:
    assert step.prepare is not None  # noqa: S101
    prepare = step.prepare

    async def run_prepare(ctx: core_main.RunContext) -> None:
        events.emit("state", phase="prepare", step=step.number)
        try:
            await stop.run(lambda: stop.in_thread(lambda: prepare(ctx)))
        except Interrupted:
            raise
        except BaseException as exc:
            state.error = f"{type(exc).__name__}: {exc}"
            raise

    return run_prepare


def wrap_step(step: core_main.Step, stop: StopController, state: RunState) -> core_main.Step:
    """手順に state イベントと中断（StopController）を付ける。番号・drives・has_ref はそのまま。"""
    phase = "init" if step.number == 1 else "step"
    return dataclasses.replace(
        step,
        handler=_wrap_handler(step, stop, state, phase=phase),
        prepare=None if step.prepare is None else _wrap_prepare(step, stop, state),
    )


def install_pause_signal_guard(loop: asyncio.AbstractEventLoop) -> None:
    """一時停止（SIGUSR1）・再開（SIGUSR2）の受け口を整える。

    モード走行の間は core（mode_drive）が受け口を差し替えて一時停止する。それ以外の間（準備・N・IGN-OFF・
    学習など）は「無視」と言うだけにして、既定動作（プロセス終了）を防ぐ。
    core はモード走行の終わりに remove_signal_handler で外す（外すと既定動作に戻る）ので、外した直後に無視へ戻す。
    """

    def _ignore_pause_signal(sig: signal.Signals) -> None:
        loop.add_signal_handler(sig, say, f"{sig.name} を受けました（一時停止・再開はモード走行の間だけ。今は無視します）")

    for sig in (signal.SIGUSR1, signal.SIGUSR2):
        _ignore_pause_signal(sig)
    _orig_remove = loop.remove_signal_handler

    def _remove_signal_handler(sig: int) -> bool:
        removed = _orig_remove(sig)
        if sig in (signal.SIGUSR1, signal.SIGUSR2) and not loop.is_closed():  # 終わりの close では戻さない
            _ignore_pause_signal(signal.Signals(sig))
        return removed

    loop.remove_signal_handler = _remove_signal_handler  # type: ignore[method-assign]


def build_steps_job(
    steps_text: str,
    *,
    vehicle: str,
    plan_in: Path | None,
    cfg: ResearchConfig,
    stop: StopController,
    state: RunState,
) -> tuple[list[core_main.Step], str | None]:
    """``--steps`` の手順の並び → 走らせる手順（state・中断を包み、学習のボタン操作を足したもの）。

    手順 24 は準備を「ファイルから計画を読む」に差し替え（``plan_in``）、handler の前にブレーキ ON を足す。
    戻り値の 2 つ目は、ボタン手順の組み合わせの誤り（なければ None）。``--steps`` のランナーと常駐ランナーで共用。
    """
    raw_steps = list(core_main.resolve_steps(
        argparse.Namespace(only=None, steps=steps_text, upto=None)
    ))
    if plan_in is not None:
        # 手順 24 の準備を「ファイルから計画を読む」に差し替える（計画を作る約 30 秒を飛ばす）
        plan_prepare = plan_job.make_plan_in_prepare(
            plan_in, vehicle=vehicle, mode=cfg.modes.wltp_mode_name
        )
        raw_steps = [
            dataclasses.replace(s, prepare=plan_prepare) if s.number in MODE_STEPS else s
            for s in raw_steps
        ]
    # 手順 24 は handler の前にブレーキ ON（Ready をモードの 0 秒に押すため。wrap_step の内側＝中断できる）
    raw_steps = [patches.brake_on_before_step24(s) if s.number in MODE_STEPS else s for s in raw_steps]
    steps = [wrap_step(s, stop, state) for s in raw_steps]
    conflict = core_main.button_step_conflict(steps)
    if conflict is not None:
        return steps, conflict
    # 学習の手順（2・12〜14）: 最初の前に Ready・D、最後の後に N（D-14b。手順 24 と同じ押し方）
    return learning_buttons.add_button_ops(steps, stop, state), None


def _interactive_step(session_factory: Callable[[core_main.RunContext], InteractiveSession]) -> core_main.Step:
    async def handler(ctx: core_main.RunContext) -> None:
        await session_factory(ctx).run()

    return core_main.Step(INTERACTIVE_STEP_NUMBER, "手動運転（対話ジョブ）", handler)


def needs_quiet_connect(args: argparse.Namespace, numbers: Sequence[int]) -> bool:
    """手順 1 を含まないジョブか（対話だけ手順 1 を自分で持つ。スケジュールも手順 1 は入れない＝初期化は初期化ページだけ）。"""
    return not args.interactive and 1 not in numbers


async def _quiet_prepare(ctx: core_main.RunContext, stop: StopController, state: RunState) -> int | None:
    """最小の接続（quiet_connect）。失敗・中断したら片付けて終了コードを返す。成功なら None。"""
    try:
        core_main.validate_config_once(ctx)
    except ConfigError as exc:
        say(f"設定エラー: {exc}")
        state.error = str(exc)
        return 2
    code: int | None = None
    try:
        await stop.run(lambda: quiet_connect.prepare_hardware(ctx))
    except Interrupted:
        code = stop.exit_code
        say(f"中断しました（{stop.signal_name}）。終了処理は済んでいます")
    except InitializationError as exc:
        state.error = f"InitializationError: {exc}"
        say(f"初期化エラー: {exc}")
        code = 4
    except Exception as exc:  # noqa: BLE001 - 予期しない例外も片付けてから終える
        state.error = f"{type(exc).__name__}: {exc}"
        say(f"初期化エラー: {state.error}")
        code = 4
    if code is not None and ctx.hardware is not None:
        say()
        await core_main.shutdown(ctx.hardware)  # 開いた接続を閉じる（原点復帰・サーボ OFF）
    return code


def _finish(code: int, state: RunState) -> int:
    events.emit("state", phase="done", exit_code=code)
    events.emit(
        "result",
        exit_code=code,
        meaning=events.exit_meaning(code),
        error=state.error or None,
        elapsed_s=round(time.monotonic() - _T0, 3),
    )
    return code


_T0 = time.monotonic()
_STOP: StopController | None = None  # main が、中断で見捨てた準備スレッドの有無を見る


async def amain(args: argparse.Namespace) -> int:
    global _T0, _STOP  # noqa: PLW0603
    _T0 = time.monotonic()
    state = RunState()
    stop = StopController()
    _STOP = stop
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.request, sig.name)
    install_pause_signal_guard(loop)
    patches.install_all(stop)

    vehicles_dir = args.vehicles_dir
    if args.plan_only:
        return await plan_job.run_plan_only(
            args, pick_mode_name_fn=pick_mode_name, fetch_rows_fn=_fetch_mode_rows
        )
    schedule_mode = args.schedule is not None or args.schedule_file is not None
    kind_text = (
        "対話" if args.interactive
        else "タイムスケジュール" if schedule_mode
        else "手順 " + args.steps
    )
    banner(f"ランナー（車 {args.vehicle} / {kind_text}）")
    say(f"ハードウェアモード: {args.hw}")
    if args.hw == HW_REAL:
        say("*** 実機モード: アクセル/ブレーキアクチュエータが物理的に動きます ***")

    # 手順の解釈と組み合わせの検査（HW に触る前）
    if args.interactive:
        numbers = [1]
    elif schedule_mode:
        numbers = []  # 手順 1 は入れない（接続・サーボ ON だけ needs_quiet_connect で行う）
    else:
        try:
            numbers = parse_steps(args.steps)
        except SystemExit as exc:
            state.error = str(exc.code)
            say(f"設定エラー: {exc.code}")
            return _finish(2, state)
    problem = check_job(numbers, mode=args.mode, limit_s=args.limit_s)
    if problem is None and schedule_mode and (args.mode is not None or args.limit_s is not None):
        problem = "--mode・--limit-s はタイムスケジュールの再生では使えません"
    if problem is None and args.plan_in is not None and (24 not in numbers or args.mode is None):
        problem = "--plan-in は手順 24 と --mode を指定したときだけ使えます"
    if problem is not None:
        state.error = problem
        say(f"設定エラー: {problem}")
        return _finish(2, state)

    schedule: schedule_spec.ScheduleData | None = None
    try:
        cfg, merged = config_store.load_vehicle_config(
            args.vehicle, vehicles_dir=vehicles_dir, common_path=args.common
        )
        # cfg.save（手順 2・12〜14）の直後に、書いた車両キーを車両 yaml へ写す
        config_store.install_save_sync(
            cfg, args.vehicle, vehicles_dir=vehicles_dir, common_path=args.common
        )
        if args.mode is not None:
            chosen = await select_mode(cfg, args.mode)
            say(f"走行モード: {chosen!r}")
        if schedule_mode:
            schedule = await load_schedule(args, cfg)
            problem = schedule_spec.check_schedule(schedule)
            if problem is not None:
                raise ConfigError(problem)
            say(f"タイムスケジュール: 「{schedule.name}」（{schedule.total_duration:.1f}s）")
    except ConfigError as exc:
        state.error = str(exc)
        say(f"設定の読み込みに失敗しました: {exc}")
        return _finish(2, state)
    say(f"設定: 共通 {args.common.name} + 車両 {args.vehicle}（合成 yaml {merged}）")
    say(f"CSV の置き場所: {cfg.results_path}")

    ctx = core_main.RunContext(
        config=cfg, hw_mode=args.hw, started_at=time.monotonic(), limit_s=args.limit_s
    )
    if args.no_pre_check:
        ctx.pre_drive_checked = True  # run_steps は走行する手順の前のチェックを飛ばす
        say("走行前チェック: シーケンスの 2 段目以降のため行いません")
    if args.interactive:
        step1 = core_main.STEPS[1]
        steps = [
            wrap_step(step1, stop, state),
            wrap_step(
                _interactive_step(
                    lambda c: InteractiveSession(
                        c.require_hardware(), c.config,
                        vehicle_id=args.vehicle, vehicles_dir=vehicles_dir,
                    )
                ),
                stop,
                state,
            ),
        ]
    elif schedule is not None:
        # 手順 1（初期化）は入れない。モード走行と同じく、接続・サーボ ON だけ needs_quiet_connect が行う
        steps = [wrap_step(schedule_job.make_schedule_step(schedule), stop, state)]
    else:
        steps, conflict = build_steps_job(
            args.steps, vehicle=args.vehicle, plan_in=args.plan_in, cfg=cfg, stop=stop, state=state
        )
        if conflict is not None:
            state.error = conflict
            say(f"設定エラー: {conflict}")
            return _finish(2, state)

    # 非常停止の流れ（減速して停車 → N → IGN-OFF → 原点復帰）は、走行するジョブ（手順 2・12〜14・24・スケジュール）だけ
    drives = schedule is not None or any(s.number in EMERGENCY_STEP_NUMBERS for s in steps)
    emergency_stop.configure(ctx, enabled=drives and not args.interactive)
    patches.configure(cfg)  # ボタンなし（buttons.installed: false）の設定を差し替えに伝える
    try:
        if needs_quiet_connect(args, numbers):
            # 手順 1 を含まないジョブ: 初期化の表示・確認なしに、動かすのに要る接続・サーボ ON だけ行う（D-15c）
            code = await _quiet_prepare(ctx, stop, state)
            if code is not None:
                return _finish(code, state)
        code = await core_main.run_steps(ctx, steps)
    except Interrupted:
        code = stop.exit_code
        say(f"中断しました（{stop.signal_name}）。終了処理は済んでいます")
    return _finish(code, state)


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.serve:
        if args.plan_only:
            parser.error("--serve は --plan-only と同時に使えません")
        if args.mode or args.limit_s is not None or args.plan_in or args.no_pre_check or args.plan_out:
            parser.error("--serve では --mode・--limit-s・--plan-in・--no-pre-check・--plan-out は使えません（コマンドで指定する）")
    elif args.plan_only:
        if args.mode is None or args.plan_out is None:
            parser.error("--plan-only には --mode と --plan-out が要ります")
        if args.steps or args.interactive or args.schedule or args.schedule_file or args.plan_in:
            parser.error("--plan-only は --steps・--interactive・--schedule・--plan-in と同時に使えません")
        args.plan_out = args.plan_out.resolve()
    elif not (args.steps or args.interactive or args.schedule or args.schedule_file):
        parser.error("--steps・--interactive・--schedule・--schedule-file・--serve・--plan-only のどれかが要ります")
    elif args.plan_out is not None:
        parser.error("--plan-out は --plan-only のときだけ使えます")
    if args.plan_in is not None:
        args.plan_in = args.plan_in.resolve()
    args.vehicles_dir = args.vehicles_dir.resolve()
    args.common = args.common.resolve()
    if args.events_fd is not None:
        events.set_sink(events.EventSink(fd=args.events_fd))
    else:
        events.set_sink(events.EventSink())
    # core は設定の相対パス（config/settings.toml など）をカレントから引く
    os.chdir(REPO_ROOT)
    if args.serve:
        from src.robot import (
            serve,  # noqa: PLC0415 - serve は run を import する（循環を避けて遅延）
        )

        code = asyncio.run(serve.amain(args))
        if serve.orphan_alive():
            sys.stdout.flush()
            os._exit(code)
        return code
    code = asyncio.run(amain(args))
    if _STOP is not None and _STOP.orphan_alive:
        # 中断で見捨てた準備スレッド（重い計算の途中）を待たずに終える。終了処理は済んでいる
        sys.stdout.flush()
        os._exit(code)
    return code


if __name__ == "__main__":
    sys.exit(main())
