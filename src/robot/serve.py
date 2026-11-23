# ruff: noqa: E501
"""常駐ランナー（``python -m src.robot.run --vehicle X --serve``）。

ジョブごとにランナー（別プロセス）を起動すると、core の ``run_steps`` が終わりに必ず shutdown（原点復帰 →
サーボ OFF → 切断）するため、シーケンスの段の間に接続し直し（約 5 s）が入る。そこで、初期化を 1 回行ったら
ランナーを動かし続け、Web サーバー（JobManager）が標準入力の JSON 1 行コマンドでジョブを送る（ユーザー決定
2026-11-20、案 A。計画 snug-crunching-avalanche 段 2）。

流れ:
  起動 → 設定を読む → 手順 1（初期化）＋ PCA9685 の確認 → ``result``（job_id="init"）→ ``state phase=idle`` →
  コマンドを待つ（1 s ごとに両軸のアラーム・電流と CAN 車速を見張る）→ ジョブを走らせる → ``result`` → 待機へ …

- ジョブ ＝ 新しい ``RunContext`` ＋ core の ``run_steps``（同じ HW を渡す）。終わりの shutdown は
  ``patches.set_job_end`` で「原点復帰だけ（home）」「何もしない（keep）」に切り替える。サーボ ON・接続のまま。
- ジョブの終わり後の判定（A-D1）: ハードの異常（通信・アラーム・過電流・電流ゼロ・CAN）なら本物の shutdown して終了、
  設定・モードの誤りなどハードに関係しない失敗なら待機へ戻る（``decide_after_job``）。
- 待機中の見張り（A-D3）: 異常なら ``watch`` イベント → ブレーキを停車保持の開度で保つ（ボタンは押さない。``emergency_stop.hold_and_keep``）→ サーボ ON のまま切断 → 終了コード 5。
- SIGINT / SIGTERM: 走行中のジョブは今どおり中断（非常停止の流れ）し、その後は本物の shutdown → 終了（130 / 143）。

コマンドとイベントの形式は src/robot/README.md。core は直さない。
"""

from __future__ import annotations

import argparse
import asyncio
import functools
import json
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from src.robot import (
    config_store,
    emergency_stop,
    events,
    patches,
    run,
    schedule_job,
    schedule_spec,
)
from src.robot.button_hold import ButtonHold
from src.robot.core import button_steps as core_button_steps
from src.robot.core import main as core_main
from src.robot.core.config import ConfigError, ResearchConfig
from src.robot.core.hardware import HW_REAL, InitializationError
from src.robot.core.pattern_drive import _overcurrent_limit_ma
from src.robot.core.term import banner, say
from src.robot.interactive import COMMANDS as INTERACTIVE_OPS
from src.robot.interactive import CommandError, InteractiveSession, parse_command
from src.robot.interrupt import Interrupted, StopController

#: モード走行のジョブの手順（JobManager の MODE_DRIVE_STEPS と同じ）
MODE_JOB_STEPS = "24,25"
#: 起動時の初期化ジョブの job_id（コマンドで来たものではないので固定）
STARTUP_JOB_ID = "init"

JOB_COMMANDS = ("mode", "schedule", "steps", "interactive", "init")
CONTROL_COMMANDS = ("stop", "quit", "quit_interactive")
#: 手動運転のスイッチ（スタート・P・N・D）。キューに入れず、その場で ButtonHold へ渡す（button_hold.py）
BUTTON_COMMAND = "button"
BUTTON_NAMES = ("start", "P", "N", "D")
BUTTON_ACTIONS = ("down", "up")
#: 対話ジョブの間だけ受ける操作（interactive.py の ``quit`` は対話の終わりではなくランナーの quit なので除く）
INTERACTIVE_ONLY_OPS = tuple(c for c in INTERACTIVE_OPS if c != "quit")
SERVE_COMMANDS = (*JOB_COMMANDS, *CONTROL_COMMANDS, BUTTON_COMMAND, *INTERACTIVE_ONLY_OPS)

#: 待機中の見張りの間隔 [s]（A-D3）
WATCH_INTERVAL_S = 1.0
#: 1 回の読み取りの待ち時間の上限 [s]
WATCH_READ_TIMEOUT_S = 3.0
#: 通信失敗・電流ゼロがこの回数（連続）続いたら異常
WATCH_FAIL_LIMIT = 3
#: 見張りで異常を見つけたときの終了コード（走行エラーと同じ 5）
WATCH_EXIT_CODE = 5

#: 待機へ戻してよい終了コード（ハードに関係しない失敗・成功）
IDLE_CODES = frozenset({0, 2, 3, 6, 7, 8})
#: シグナルでの中断（_on_signal）。ランナーを閉じる。文は記録に出す理由
SIGNAL_CODES = {130: "非常停止（SIGINT）で中断しました", 143: "SIGTERM で中断しました"}

#: 終了コード 4・5 のうち、メッセージにこの語があればハードの異常とみなす（通信・アラーム・電流・CAN・ボタン基板）
HARDWARE_ERROR_WORDS = (
    "通信",
    "アラーム",
    "過電流",
    "電流",
    "Modbus",
    "modbus",
    "CAN",
    "シリアル",
    "接続",
    "応答",
    "タイムアウト",
    "読めません",
    "TimeoutError",
    "OSError",
    "ConnectionError",
    "SerialException",
    "ModbusException",
    "PCA9685",
    "I2C",
    "ボタンサーボ",
    "ボタンを押せません",
    "アクチュエータ",
)


# ── コマンドの解釈（純粋） ───────────────────────────────────────


@dataclass(frozen=True)
class ServeCommand:
    """標準入力の JSON 1 行を解釈したもの。ジョブのコマンドは ``job_id`` を持つ。"""

    cmd: str
    job_id: str | None = None
    req_id: Any = None
    raw: str = ""  # 対話の操作（jog など）は元の行をそのまま対話セッションへ渡す
    mode: str | None = None
    plan_in: Path | None = None
    limit_s: float | None = None
    no_pre_check: bool = False
    home_at_end: bool = True
    schedule: dict[str, Any] | None = None
    steps: str | None = None
    purpose: str | None = None
    button: str | None = None  # button コマンド: start / P / N / D
    action: str | None = None  # button コマンド: down / up
    vehicle: str | None = None  # このジョブで使う車（JobManager が今選んでいる車）。無ければ起動時の車（args.vehicle）


def _optional_bool(obj: dict[str, Any], key: str, default: bool) -> bool:
    value = obj.get(key, default)
    if value is None:
        return default
    if not isinstance(value, bool):
        raise CommandError(f"{key} は true / false で指定してください: {value!r}")
    return value


def parse_serve_command(line: str) -> ServeCommand:
    """JSON 1 行 → ServeCommand。誤りは CommandError（日本語の案内つき。ack の message になる）。"""
    text = line.strip()
    if not text:
        raise CommandError("空の行です")
    try:
        obj = json.loads(text)
    except json.JSONDecodeError as exc:
        raise CommandError(f"JSON として読めません: {exc.msg}") from exc
    if not isinstance(obj, dict):
        raise CommandError("コマンドは JSON のオブジェクトで指定してください")
    cmd = obj.get("cmd")
    if cmd not in SERVE_COMMANDS:
        raise CommandError(f"未知のコマンド {cmd!r}（{', '.join(SERVE_COMMANDS)}）")
    req_id = obj.get("id")
    job_id = obj.get("job_id")
    if job_id is not None and (not isinstance(job_id, str) or not job_id.strip()):
        raise CommandError(f"job_id は空でない文字列で指定してください: {job_id!r}")
    if cmd == BUTTON_COMMAND:
        button = obj.get("button")
        action = obj.get("action")
        if button not in BUTTON_NAMES:
            raise CommandError(f"button は {' / '.join(BUTTON_NAMES)} で指定してください: {button!r}")
        if action not in BUTTON_ACTIONS:
            raise CommandError(f"action は {' / '.join(BUTTON_ACTIONS)} で指定してください: {action!r}")
        return ServeCommand(cmd, job_id=job_id, req_id=req_id, raw=text, button=button, action=action)
    if cmd in INTERACTIVE_ONLY_OPS or cmd in ("stop", "quit", "quit_interactive"):
        return ServeCommand(cmd, job_id=job_id, req_id=req_id, raw=text)
    # ここから先はジョブのコマンド
    if job_id is None:
        raise CommandError("job_id がありません（ジョブのコマンドには job_id が要ります）")
    home_at_end = _optional_bool(obj, "home_at_end", True)
    vehicle = obj.get("vehicle")
    if vehicle is not None and (not isinstance(vehicle, str) or not vehicle.strip()):
        raise CommandError(f"vehicle は車 ID（空でない文字列）か null で指定してください: {vehicle!r}")
    vehicle = vehicle.strip() if vehicle else None
    if cmd == "init":
        return ServeCommand(cmd, job_id=job_id, req_id=req_id, vehicle=vehicle)
    if cmd == "interactive":
        purpose = obj.get("purpose", "manual")
        if purpose not in ("manual", "calib"):
            raise CommandError("purpose は manual / calib で指定してください")
        return ServeCommand(cmd, job_id=job_id, req_id=req_id, purpose=purpose, home_at_end=True, vehicle=vehicle)
    no_pre_check = _optional_bool(obj, "no_pre_check", False)
    if cmd == "mode":
        mode = obj.get("mode")
        if not isinstance(mode, str) or not mode.strip():
            raise CommandError("mode（走行モードの名前か ID）がありません")
        plan_in = obj.get("plan_in")
        if plan_in is not None and (not isinstance(plan_in, str) or not plan_in.strip()):
            raise CommandError("plan_in はファイルのパス（文字列）か null で指定してください")
        limit_s = obj.get("limit_s")
        if limit_s is not None:
            if isinstance(limit_s, bool) or not isinstance(limit_s, int | float) or limit_s <= 0:
                raise CommandError("limit_s は正の秒数か null で指定してください")
            limit_s = float(limit_s)
        return ServeCommand(
            cmd, job_id=job_id, req_id=req_id, mode=mode, plan_in=Path(plan_in) if plan_in else None,
            limit_s=limit_s, no_pre_check=no_pre_check, home_at_end=home_at_end, vehicle=vehicle,
        )
    if cmd == "schedule":
        schedule = obj.get("schedule")
        if not isinstance(schedule, dict):
            raise CommandError("schedule（schedule_to_dict の形の JSON オブジェクト）がありません")
        return ServeCommand(
            cmd, job_id=job_id, req_id=req_id, schedule=schedule,
            no_pre_check=no_pre_check, home_at_end=home_at_end, vehicle=vehicle,
        )
    # steps
    steps_text = obj.get("steps")
    if not isinstance(steps_text, str) or not steps_text.strip():
        raise CommandError("steps（手順の並び。例 2,25）がありません")
    try:
        numbers = run.parse_steps(steps_text)
    except SystemExit as exc:
        raise CommandError(str(exc.code)) from exc
    if not numbers:
        raise CommandError("steps が空です")
    if 1 in numbers:
        raise CommandError("手順 1（初期化）は steps では動かせません（init コマンドを使ってください）")
    problem = run.check_job(numbers, mode=None, limit_s=None)
    if problem is not None:
        raise CommandError(problem)
    return ServeCommand(
        cmd, job_id=job_id, req_id=req_id, steps=steps_text.strip(),
        no_pre_check=no_pre_check, home_at_end=home_at_end, vehicle=vehicle,
    )


# ── ジョブの終わりの判定（A-D1。純粋） ──────────────────────────────


@dataclass(frozen=True)
class JobDecision:
    close: bool  # True ならランナーを閉じる（本物の shutdown → 終了）
    needs_init: bool  # 待機へ戻るとき、次は init が要るか
    reason: str


def _has_hardware_word(error: str) -> bool:
    return any(w in (error or "") for w in HARDWARE_ERROR_WORDS)


def decide_after_job(
    code: int,
    *,
    error: str = "",
    diag: emergency_stop.Diagnosis | None = None,
    kept: bool = False,
    hw_connected: bool = True,
    stopped: bool = False,
) -> JobDecision:
    """ジョブの終了コード・例外の文・診断・ハードの状態から、待機へ戻すか、ランナーを閉じるか。

    | 条件（上から順に）                                                         | 結果 |
    |---|---|
    | 診断にハード異常（ブレーキ軸・アクセル軸の異常・CAN を読めない）            | 閉じる |
    | ブレーキを保ったまま終えた（brake_kept / buttons_unavailable）              | 待機（needs_init） |
    | 停止コマンドで中断した                                                       | 待機 |
    | 終了コード 0・2・3・6・7・8                                                  | 待機 |
    | 終了コード 4・5 で、接続が閉じている                                         | 閉じる |
    | 終了コード 4・5 で、文にハード語（通信・アラーム・電流・CAN・ボタン基板…）  | 閉じる |
    | 終了コード 4（上に当たらないもの）                                           | 閉じる（初期化の失敗はハード） |
    | 終了コード 5（上に当たらない: 車速が出ない・ボタン関係以外の DriveError）    | 待機 |
    | 終了コード 130・143（非常停止・アプリ終了の SIGINT / SIGTERM で中断）         | 閉じる |
    | それ以外（未知の終了コード）                                                 | 閉じる（安全側） |
    """
    if diag is not None and (diag.brake_faulty or diag.accel_faulty or not diag.can_ok):
        return JobDecision(True, False, "ハードの異常（" + "、".join(diag.notes or ["軸・CAN の診断"]) + "）")
    if kept:
        return JobDecision(False, True, "ブレーキを保ったまま終えたため、次は初期化が必要です")
    if stopped:
        return JobDecision(False, False, "停止コマンドで中断しました")
    if code in IDLE_CODES:
        return JobDecision(False, False, "")
    if code in (4, 5):
        if not hw_connected:
            return JobDecision(True, False, "接続が閉じています")
        if _has_hardware_word(error):
            return JobDecision(True, False, f"ハードの異常の疑い（{error}）")
        if code == 4:
            return JobDecision(True, False, f"初期化の失敗（{error or '詳細なし'}）")
        return JobDecision(False, False, "ハードに関係しない走行の失敗")
    if code in SIGNAL_CODES:
        return JobDecision(True, False, SIGNAL_CODES[code])
    return JobDecision(True, False, f"未知の終了コード {code}")


# ── 待機中の見張り（A-D3） ─────────────────────────────────────────


@dataclass(frozen=True)
class AxisReading:
    ok: bool  # 読めたか
    error: str = ""
    alarm: bool = False
    current_ma: float = 0.0
    position: int = 0


@dataclass(frozen=True)
class CanReading:
    ok: bool
    error: str = ""
    speed_kmh: float | None = None


@dataclass(frozen=True)
class WatchFinding:
    level: str  # "warn"（まだ様子見）/ "error"（異常。ブレーキを保って終了へ）
    detail: str


class IdleWatcher:
    """待機中の見張りの判定（純粋。読み取りは ``read_idle``）。

    error: アラーム・過電流・通信失敗が ``fail_limit`` 回連続・CAN が ``fail_limit`` 回連続で読めない・
    指令位置 > 0 なのに電流 0 mA が ``fail_limit`` 回連続（その軸が一度でも電流 > 0 を返した後だけ。スタブは常に 0 mA）。
    warn: 通信失敗・CAN の読めないのが 1 回目〜``fail_limit`` 回未満。
    """

    def __init__(self, *, overcurrent_ma: float, fail_limit: int = WATCH_FAIL_LIMIT) -> None:
        self.overcurrent_ma = overcurrent_ma
        self.fail_limit = fail_limit
        self._fail = {"アクセル": 0, "ブレーキ": 0}
        self._zero = {"アクセル": 0, "ブレーキ": 0}
        self._ever_current = {"アクセル": False, "ブレーキ": False}
        self._can_fail = 0

    def evaluate(self, accel: AxisReading, brake: AxisReading, can: CanReading) -> list[WatchFinding]:
        findings: list[WatchFinding] = []
        for label, r in (("アクセル", accel), ("ブレーキ", brake)):
            if not r.ok:
                self._fail[label] += 1
                self._zero[label] = 0
                n = self._fail[label]
                if n >= self.fail_limit:
                    findings.append(WatchFinding("error", f"{label}軸の通信が {n} 回続けて失敗しました（{r.error}）"))
                else:
                    findings.append(WatchFinding("warn", f"{label}軸の通信に失敗しました（{n}/{self.fail_limit} 回目。{r.error}）"))
                continue
            self._fail[label] = 0
            if r.alarm:
                findings.append(WatchFinding("error", f"{label}軸にアラームが出ています"))
            if r.current_ma > self.overcurrent_ma:
                findings.append(
                    WatchFinding("error", f"{label}軸が過電流です（{r.current_ma:.0f} mA > {self.overcurrent_ma:.0f} mA）")
                )
            if r.current_ma > 0.0:
                self._ever_current[label] = True
            if r.position > 0 and r.current_ma == 0.0 and self._ever_current[label]:
                self._zero[label] += 1
            else:
                self._zero[label] = 0
            if self._zero[label] >= self.fail_limit:
                findings.append(
                    WatchFinding("error", f"{label}軸の電流が 0 mA のままです（位置 {r.position}、{self._zero[label]} 回続けて）")
                )
        if can.ok:
            self._can_fail = 0
        else:
            self._can_fail += 1
            if self._can_fail >= self.fail_limit:
                findings.append(WatchFinding("error", f"CAN 車速を {self._can_fail} 回続けて読めません（{can.error}）"))
            else:
                findings.append(WatchFinding("warn", f"CAN 車速を読めません（{self._can_fail}/{self.fail_limit} 回目。{can.error}）"))
        return findings


async def _read_axis(axis: Any) -> AxisReading:
    try:
        alarm = bool(await asyncio.wait_for(axis.is_alarm_active(), WATCH_READ_TIMEOUT_S))
        monitor = await asyncio.wait_for(axis.read_monitor(), WATCH_READ_TIMEOUT_S)
        return AxisReading(True, "", alarm, float(monitor.current_ma), int(monitor.position_pulse))
    except Exception as exc:  # noqa: BLE001
        return AxisReading(False, f"{type(exc).__name__}: {exc}")


async def read_idle(hw: Any) -> tuple[AxisReading, AxisReading, CanReading]:
    """両軸のアラーム・電流・位置と CAN 車速を 1 回読む（ジョブの実行中は呼ばない。Modbus を取り合わない）。"""

    async def read_can() -> CanReading:
        try:
            speed = await asyncio.wait_for(hw.can.read_speed(), WATCH_READ_TIMEOUT_S)
            return CanReading(True, "", float(speed))
        except Exception as exc:  # noqa: BLE001
            return CanReading(False, f"{type(exc).__name__}: {exc}")

    accel, brake, can = await asyncio.gather(_read_axis(hw.accel), _read_axis(hw.brake), read_can())
    return accel, brake, can


# ── 診断の記録（A-D1 の判定材料） ──────────────────────────────────

_last_diagnosis: emergency_stop.Diagnosis | None = None
_classify_wrapped = False


def _install_diagnosis_recorder() -> None:
    """``emergency_stop.classify`` を包み、非常停止の流れの診断（brake_faulty など）を覚える（1 回だけ）。"""
    global _classify_wrapped  # noqa: PLW0603
    if _classify_wrapped:
        return
    orig = emergency_stop.classify

    @functools.wraps(orig)
    async def classify(hw: Any, reason: str) -> emergency_stop.Diagnosis:
        global _last_diagnosis  # noqa: PLW0603
        diag = await orig(hw, reason)
        _last_diagnosis = diag
        return diag

    emergency_stop.classify = classify  # type: ignore[assignment]
    _classify_wrapped = True


#: 終了で見捨てた準備スレッドを見るための、このプロセスの StopController の一覧
STOPS: list[StopController] = []


def orphan_alive() -> bool:
    return any(s.orphan_alive for s in STOPS)


# ── 常駐ランナー本体 ──────────────────────────────────────────────


class _ServeInteractive:
    """対話ジョブ: 標準入力は常駐ランナーが読むので、そこから回ってきた行（キュー）を実行する。"""

    def __init__(
        self, runner: ServeRunner, ctx: core_main.RunContext, queue: asyncio.Queue[str | None], vehicle: str | None = None
    ) -> None:
        self._runner = runner
        self._vehicle = vehicle or runner.args.vehicle
        self._ctx = ctx
        self._queue = queue

    async def run(self) -> None:
        args = self._runner.args
        session = InteractiveSession(
            self._ctx.require_hardware(), self._ctx.config,
            vehicle_id=self._vehicle, vehicles_dir=args.vehicles_dir,
        )
        hold = ButtonHold(self._ctx.require_hardware(), self._ctx.config)
        self._runner.button_hold = hold
        try:
            events.emit("state", phase="interactive", purpose=self._runner.purpose)
            say("対話ジョブ: コマンド待ち（jog / move / home / status / save_min / save_max / button。quit_interactive で待機へ戻る）")
            events.emit("status", id=None, **(await session.status()))
            while True:
                line = await self._queue.get()
                if line is None:
                    say("対話ジョブ: 終了します（待機へ戻ります）")
                    return
                try:
                    command = parse_command(line)
                except CommandError as exc:
                    say(f"対話: コマンドが不正です（{exc}）")
                    events.emit("ack", id=None, cmd=None, ok=False, message=str(exc))
                    continue
                await session.execute(command)
        finally:
            # 運転終了・中断・非常停止のどれでも、押しているスイッチを離してから先へ進む（D6）
            self._runner.button_hold = None
            await asyncio.shield(hold.release_all())


class ServeRunner:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.queue: asyncio.Queue[tuple[str, Any]] = asyncio.Queue()
        self.hw: Any = None
        self.init_ctx: core_main.RunContext | None = None
        self.stop: StopController = StopController()
        self.job_task: asyncio.Task[None] | None = None
        self.current: ServeCommand | None = None
        self.exit_after: int | None = None  # 実行中のジョブが終わったらこの終了コードでランナーも終える（SIGINT・quit）
        self.needs_init = False
        self.closing = False
        self.interactive_queue: asyncio.Queue[str | None] | None = None
        self.button_hold: ButtonHold | None = None  # 手動運転の間だけ（_ServeInteractive.run が作る）
        self.purpose: str | None = None
        self.final_job_id: str | None = None
        self._final_result_emitted = False
        self._t_start = time.monotonic()
        self.watcher: IdleWatcher | None = None
        self._final_error = ""

    # ── 出力 ──

    def _ack(self, cmd: str | None, ok: bool, message: str, *, job_id: str | None = None, req_id: Any = None) -> None:
        say(f"serve: {cmd} → {'OK' if ok else 'NG'} {message}")
        fields: dict[str, Any] = {"id": req_id, "cmd": cmd, "ok": ok, "message": message}
        if job_id is not None:
            fields["job_id"] = job_id
        events.emit("ack", **fields)

    def _emit_idle(self) -> None:
        say("待機中（次のコマンドを待ちます）" + ("。次は init が要ります" if self.needs_init else ""))
        events.emit("state", phase="idle", needs_init=self.needs_init)

    # ── 起動 ──

    async def run(self) -> int:
        args = self.args
        loop = asyncio.get_running_loop()
        run.install_pause_signal_guard(loop)
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self._on_signal, sig.name)
        STOPS.append(self.stop)
        patches.install_all(self.stop)
        _install_diagnosis_recorder()
        self._start_reader(loop)
        banner(f"常駐ランナー（車 {args.vehicle}）")
        say(f"ハードウェアモード: {args.hw}")
        if args.hw == HW_REAL:
            say("*** 実機モード: アクセル/ブレーキアクチュエータが物理的に動きます ***")
        self._start_job(ServeCommand("init", job_id=STARTUP_JOB_ID))
        try:
            code = await self._main_loop()
        finally:
            self.closing = True
        await self._final_shutdown()
        if not self._final_result_emitted:
            self._emit_runner_result(code)
        return code

    def _start_reader(self, loop: asyncio.AbstractEventLoop) -> None:
        def reader() -> None:
            # デーモンスレッド: 終了時に readline で止まっていてもプロセスの終了を妨げない
            for line in sys.stdin:
                loop.call_soon_threadsafe(self.queue.put_nowait, ("line", line))
            loop.call_soon_threadsafe(self.queue.put_nowait, ("eof", None))

        threading.Thread(target=reader, daemon=True, name="serve-stdin").start()

    def _on_signal(self, name: str) -> None:
        """SIGINT / SIGTERM。実行中のジョブは中断（非常停止の流れ）し、その後ランナーも終える。待機中はそのまま終える。"""
        code = 143 if name == "SIGTERM" else 130
        if self.closing:
            say(f"{name}: 終了処理中のため無視します")
            return
        if self.job_task is not None:
            self.exit_after = code
            self.stop.request(name)
        else:
            self.queue.put_nowait(("exit", code))

    # ── 主ループ ──

    def _can_watch(self) -> bool:
        hw = self.hw
        return (
            self.job_task is None
            and not self.needs_init
            and hw is not None
            and bool(getattr(hw, "connected", False))
            and bool(getattr(hw, "initialized", False))
        )

    async def _main_loop(self) -> int:
        next_watch = time.monotonic() + WATCH_INTERVAL_S
        while True:
            timeout = max(0.0, next_watch - time.monotonic()) if self._can_watch() else None
            try:
                kind, payload = await asyncio.wait_for(self.queue.get(), timeout)
            except TimeoutError:
                next_watch = time.monotonic() + WATCH_INTERVAL_S
                code = await self._watch_once()
                if code is not None:
                    return code
                continue
            if kind == "exit":
                return int(payload)
            if kind == "tick":
                next_watch = time.monotonic() + WATCH_INTERVAL_S
                continue
            if kind == "eof":
                say("標準入力が閉じられたので終了します")
                if self.job_task is not None:
                    self.exit_after = 0
                    self.stop.request("EOF")
                    continue
                return 0
            code = await self._handle_line(str(payload))
            if code is not None:
                return code
            if self._can_watch():
                next_watch = min(next_watch, time.monotonic() + WATCH_INTERVAL_S)

    async def _handle_line(self, line: str) -> int | None:
        """1 行を処理する。ランナーを終えるなら終了コード、続けるなら None。"""
        if not line.strip():
            return None
        try:
            command = parse_serve_command(line)
        except CommandError as exc:
            say(f"serve: コマンドが不正です（{exc}）")
            events.emit("ack", id=None, cmd=None, ok=False, message=str(exc))
            return None
        cmd = command.cmd
        busy = self.job_task is not None
        if cmd == "stop":
            if busy:
                self.stop.request("stop")
                self._ack(cmd, True, "実行中のジョブに中断を要求しました", req_id=command.req_id)
            else:
                self._ack(cmd, True, "実行中のジョブはありません", req_id=command.req_id)
            return None
        if cmd == "quit":
            self.final_job_id = command.job_id
            if busy:
                self.exit_after = 0
                self.stop.request("quit")
                self._ack(cmd, True, "実行中のジョブを止めてから終了します", job_id=command.job_id, req_id=command.req_id)
                return None
            self._ack(cmd, True, "原点復帰 → サーボOFF → 切断して終了します", job_id=command.job_id, req_id=command.req_id)
            return 0
        if cmd == "quit_interactive":
            if self.interactive_queue is not None:
                self.interactive_queue.put_nowait(None)
                self._ack(cmd, True, "対話を終えて待機へ戻ります", req_id=command.req_id)
            else:
                self._ack(cmd, False, "対話ジョブではありません", req_id=command.req_id)
            return None
        if cmd == BUTTON_COMMAND:
            hold = self.button_hold
            if hold is None or self.purpose != "manual":
                self._ack(cmd, False, "手動運転の間だけ使えます", req_id=command.req_id)
                return None
            assert command.button is not None
            if command.action == "down":
                ok, message = await hold.down(command.button)
            else:
                ok, message = await hold.up(command.button)
            self._ack(cmd, ok, message, req_id=command.req_id)
            return None
        if cmd in INTERACTIVE_ONLY_OPS:
            if self.interactive_queue is not None:
                self.interactive_queue.put_nowait(command.raw)  # ack・status は対話セッションが出す
            else:
                self._ack(cmd, False, "対話ジョブの間だけ使えます", req_id=command.req_id)
            return None
        # ジョブのコマンド
        if busy:
            running = self.current.job_id if self.current else "?"
            self._ack(cmd, False, f"実行中です（job_id={running}）", job_id=command.job_id, req_id=command.req_id)
            return None
        if cmd != "init" and (self.needs_init or not self._initialized()):
            self._ack(cmd, False, "初期化が必要です（init コマンド）", job_id=command.job_id, req_id=command.req_id)
            return None
        self._ack(cmd, True, "ジョブを開始します", job_id=command.job_id, req_id=command.req_id)
        self._start_job(command)
        return None

    def _initialized(self) -> bool:
        hw = self.hw
        return hw is not None and bool(getattr(hw, "connected", False)) and bool(getattr(hw, "initialized", False))

    # ── 待機中の見張り ──

    async def _watch_once(self) -> int | None:
        hw = self.hw
        if self.watcher is None:
            self.watcher = IdleWatcher(overcurrent_ma=_overcurrent_limit_ma(hw))
        accel, brake, can = await read_idle(hw)
        if self.job_task is not None or self.closing:  # 読んでいる間にコマンドは処理されないが念のため
            return None
        findings = self.watcher.evaluate(accel, brake, can)
        for f in findings:
            say(f"*** 見張り（{f.level}）: {f.detail} ***")
            events.emit("watch", level=f.level, detail=f.detail)
        errors = [f for f in findings if f.level == "error"]
        if not errors:
            return None
        reason = "待機中の見張りで異常: " + " / ".join(f.detail for f in errors)
        say(f"{reason}。ブレーキを停車保持の開度で保ち、サーボ ON のまま終了します（ボタンは押しません）")
        ctx = self.init_ctx
        if ctx is not None:
            emergency_stop.configure(ctx, enabled=False)
            try:
                await emergency_stop.hold_and_keep(hw, reason)
            except Exception as exc:  # noqa: BLE001
                say(f"  警告: ブレーキの保持に失敗しました（{type(exc).__name__}: {exc}）")
                emergency_stop.brake_kept = True
        self.final_job_id = None
        self._final_error = reason
        return WATCH_EXIT_CODE

    # ── ジョブ ──

    def _start_job(self, command: ServeCommand) -> None:
        self.stop = StopController()
        STOPS.append(self.stop)
        self.current = command
        self.purpose = command.purpose
        self.interactive_queue = asyncio.Queue() if command.cmd == "interactive" else None
        self.job_task = asyncio.create_task(self._job_main(command))

    async def _job_main(self, command: ServeCommand) -> None:
        global _last_diagnosis  # noqa: PLW0603
        stop = self.stop
        state = run.RunState()
        t0 = time.monotonic()
        events.set_job_id(command.job_id)
        _last_diagnosis = None
        code = 5
        try:
            code = await self._dispatch(command, stop, state)
        except Interrupted:
            code = stop.exit_code
            say(f"中断しました（{stop.signal_name}）。終了処理は済んでいます")
        except Exception as exc:  # noqa: BLE001 - ジョブの例外は result に載せる
            state.error = f"{type(exc).__name__}: {exc}"
            say(f"ジョブが例外で終わりました: {state.error}")
            code = 5
        exit_code: int | None
        try:
            exit_code = await self._finish(command, code, state, stop, t0)
        except Exception as exc:  # noqa: BLE001
            say(f"ジョブの終わりの処理で例外: {type(exc).__name__}: {exc}。安全側でランナーを閉じます")
            exit_code = code if code != 0 else 5
        # 後始末（以降は待機。コマンドを受け付ける）
        self.job_task = None
        self.current = None
        self.interactive_queue = None
        self.purpose = None
        events.set_job_id(None)
        if exit_code is None:
            self._emit_idle()
            self.queue.put_nowait(("tick", None))  # 主ループを起こす（待機に入ったので見張りの間隔を測り直す）
        else:
            self.queue.put_nowait(("exit", exit_code))

    async def _finish(
        self, command: ServeCommand, code: int, state: run.RunState, stop: StopController, t0: float
    ) -> int | None:
        """result を出し、待機へ戻すか閉じるかを決める。閉じるなら終了コード、待機なら None。"""
        hw = self.hw
        kept = emergency_stop.brake_kept or emergency_stop.buttons_unavailable
        emergency_stop.reset()  # 待機中に非常停止の状態が残らないように（本物の shutdown が流れを再開しない）
        diag = _last_diagnosis
        if (
            diag is None and code in (4, 5) and hw is not None
            and getattr(hw, "connected", False) and not stop.requested
        ):
            try:
                diag = await asyncio.wait_for(emergency_stop.classify(hw, state.error), 15.0)
            except Exception as exc:  # noqa: BLE001 - 診断できなければ文の判定へ
                say(f"  警告: ハードの状態を確かめられませんでした（{type(exc).__name__}: {exc}）")
        connected = hw is not None and bool(getattr(hw, "connected", False))
        decision = decide_after_job(
            code, error=state.error, diag=diag, kept=kept, hw_connected=connected,
            stopped=stop.requested and self.exit_after is None,
        )
        close = decision.close
        exit_code: int | None = None
        if self.exit_after is not None:
            exit_code = self.exit_after
        elif close:
            exit_code = code if code != 0 else 5
        self.needs_init = decision.needs_init and exit_code is None
        if decision.reason:
            say(f"ジョブの終わり: {decision.reason}" + ("（ランナーを閉じます）" if exit_code is not None else "（待機へ戻ります）"))
        events.emit("state", phase="done", exit_code=code)
        meaning = events.exit_meaning(code)
        if stop.requested and stop.signal_name in ("stop", "quit", "EOF"):
            meaning = f"中断（{stop.signal_name} コマンド）。終了処理は済ませた"
        events.emit(
            "result",
            exit_code=code,
            meaning=meaning,
            error=state.error or None,
            elapsed_s=round(time.monotonic() - t0, 3),
            needs_init=self.needs_init,
            runner_exit=exit_code is not None,
        )
        if exit_code is not None:
            self._final_result_emitted = True
        return exit_code

    async def _dispatch(self, command: ServeCommand, stop: StopController, state: run.RunState) -> int:
        if command.cmd == "init":
            return await self._run_init(command, stop, state)
        return await self._run_job(command, stop, state)

    def _vehicle_of(self, command: ServeCommand | None) -> str:
        """このジョブで使う車。コマンドに車 ID があればそれ、無ければ起動時の車。"""
        return (command.vehicle if command is not None and command.vehicle else None) or self.args.vehicle

    def _load_config(self, vehicle: str | None = None) -> tuple[ResearchConfig, Path]:
        """``vehicle``（無ければ起動時の車）の設定を読む。キャリブの保存先（vehicle.yaml）もその車にする。"""
        args = self.args
        vehicle = vehicle or args.vehicle
        cfg, merged = config_store.load_vehicle_config(
            vehicle, vehicles_dir=args.vehicles_dir, common_path=args.common
        )
        config_store.install_save_sync(cfg, vehicle, vehicles_dir=args.vehicles_dir, common_path=args.common)
        return cfg, merged

    async def _run_init(self, command: ServeCommand, stop: StopController, state: run.RunState) -> int:
        """手順 1（初期化。原点復帰を含む）＋ PCA9685 の確認。ペダルは原点復帰以外では動かさない。"""
        args = self.args
        if self.hw is not None and getattr(self.hw, "connected", False):
            say("初期化のやり直し: いったん本物の終了処理（原点復帰 → サーボOFF → 切断）をします")
            patches.set_job_end("full")
            emergency_stop.reset()
            await core_main.shutdown(self.hw)
        self.hw = None
        self.init_ctx = None
        self.watcher = None
        vehicle = self._vehicle_of(command)
        try:
            cfg, merged = self._load_config(vehicle)
        except ConfigError as exc:
            state.error = str(exc)
            say(f"設定の読み込みに失敗しました: {exc}")
            return 2
        say(f"設定: 共通 {args.common.name} + 車両 {vehicle}（合成 yaml {merged}）")
        ctx = core_main.RunContext(config=cfg, hw_mode=args.hw, started_at=time.monotonic())
        self.init_ctx = ctx
        patches.install_all(stop)
        patches.configure(cfg)
        patches.set_job_end("full")
        emergency_stop.configure(ctx, enabled=False)
        step1 = run.wrap_step(core_main.STEPS[1], stop, state)
        code = 0
        try:
            core_main.validate_config_once(ctx)
            await step1.handler(ctx)
            self.hw = ctx.hardware
            if ctx.hardware is not None:
                await stop.run(lambda: self._check_buttons(ctx.hardware))
        except ConfigError as exc:
            state.error = str(exc)
            say(f"設定エラー: {exc}")
            code = 2
        except InitializationError as exc:
            state.error = state.error or f"InitializationError: {exc}"
            say(f"初期化エラー: {exc}")
            code = 4
        except Exception as exc:  # noqa: BLE001
            state.error = state.error or f"{type(exc).__name__}: {exc}"
            say(f"初期化エラー: {state.error}")
            code = 4
        self.hw = ctx.hardware
        if code != 0 or self.hw is None or not getattr(self.hw, "initialized", False):
            return code or 4
        say("初期化 OK（サーボ ON・接続のまま待機します）")
        return 0

    async def _check_buttons(self, hw: Any) -> None:
        """PCA9685（ボタンサーボの基板）に接続して確かめる。``buttons.installed: false`` なら patches の差し替えが何もしない。

        接続すると全 ch が待機角へ動く。ブレーキ・アクセルは踏まない。
        """
        await core_button_steps.ensure_buttons_connected(hw)

    async def _run_job(self, command: ServeCommand, stop: StopController, state: run.RunState) -> int:
        args = self.args
        vehicle = self._vehicle_of(command)
        try:
            cfg, merged = self._load_config(vehicle)
            ctx_limit = command.limit_s
            schedule: schedule_spec.ScheduleData | None = None
            steps: list[core_main.Step]
            if command.cmd == "mode":
                assert command.mode is not None  # noqa: S101
                numbers = run.parse_steps(MODE_JOB_STEPS)
                problem = run.check_job(numbers, mode=command.mode, limit_s=command.limit_s)
                if problem is not None:
                    raise ConfigError(problem)
                chosen = await run.select_mode(cfg, command.mode)
                say(f"走行モード: {chosen!r}")
                plan_in = command.plan_in.resolve() if command.plan_in else None
                steps, conflict = run.build_steps_job(
                    MODE_JOB_STEPS, vehicle=vehicle, plan_in=plan_in, cfg=cfg, stop=stop, state=state
                )
                if conflict is not None:
                    raise ConfigError(conflict)
            elif command.cmd == "schedule":
                assert command.schedule is not None  # noqa: S101
                try:
                    schedule = schedule_spec.schedule_from_dict(command.schedule)
                except schedule_spec.ScheduleError as exc:
                    raise ConfigError(str(exc)) from exc
                problem = schedule_spec.check_schedule(schedule)
                if problem is not None:
                    raise ConfigError(problem)
                say(f"タイムスケジュール: 「{schedule.name}」（{schedule.total_duration:.1f}s）")
                steps = [run.wrap_step(schedule_job.make_schedule_step(schedule), stop, state)]
            elif command.cmd == "steps":
                assert command.steps is not None  # noqa: S101
                steps, conflict = run.build_steps_job(
                    command.steps, vehicle=vehicle, plan_in=None, cfg=cfg, stop=stop, state=state
                )
                if conflict is not None:
                    raise ConfigError(conflict)
            else:  # interactive
                assert self.interactive_queue is not None  # noqa: S101
                queue = self.interactive_queue
                steps = [
                    run.wrap_step(
                        run._interactive_step(lambda c: _ServeInteractive(self, c, queue, vehicle)),  # noqa: SLF001
                        stop,
                        state,
                    )
                ]
        except ConfigError as exc:
            state.error = str(exc)
            say(f"設定エラー: {exc}")
            return 2
        except SystemExit as exc:  # parse_steps
            state.error = str(exc.code)
            say(f"設定エラー: {exc.code}")
            return 2
        say(f"設定: 共通 {args.common.name} + 車両 {vehicle}（合成 yaml {merged}）")
        say(f"CSV の置き場所: {cfg.results_path}")

        ctx = core_main.RunContext(config=cfg, hw_mode=args.hw, started_at=time.monotonic(), limit_s=ctx_limit)
        ctx.hardware = self.hw  # 初期化済みの HW をそのまま使う（接続し直さない）
        if command.no_pre_check:
            ctx.pre_drive_checked = True  # run_steps は走行する手順の前のチェックを飛ばす
            say("走行前チェック: シーケンスの 2 段目以降のため行いません")
        drives = schedule is not None or any(s.number in run.EMERGENCY_STEP_NUMBERS for s in steps)
        patches.install_all(stop)  # 差し替えは済んでいる。中断の状態 stop だけ更新する
        patches.configure(cfg)
        emergency_stop.configure(ctx, enabled=drives)
        if command.cmd == "interactive":
            patches.set_job_end("home")
        else:
            patches.set_job_end("home" if command.home_at_end else "keep")
        try:
            return await core_main.run_steps(ctx, steps)
        finally:
            patches.set_job_end("full")

    # ── 終了 ──

    def _emit_runner_result(self, code: int) -> None:
        events.emit("state", phase="done", exit_code=code)
        fields: dict[str, Any] = {}
        if self.final_job_id is not None:
            fields["job_id"] = self.final_job_id
        events.emit(
            "result",
            exit_code=code,
            meaning=events.exit_meaning(code),
            error=self._final_error or None,
            elapsed_s=round(time.monotonic() - self._t_start, 3),
            needs_init=self.needs_init,
            runner_exit=True,
            **fields,
        )
        self._final_result_emitted = True

    async def _final_shutdown(self) -> None:
        """本物の shutdown（原点復帰 → サーボ OFF → 切断）。ブレーキを保った終わりは従来どおりサーボ ON のまま切る。"""
        hw = self.hw
        if hw is None and self.init_ctx is not None:
            hw = self.init_ctx.hardware
        if hw is None:
            return
        patches.set_job_end("full")
        say()
        try:
            await core_main.shutdown(hw)
        except Exception as exc:  # noqa: BLE001
            say(f"  警告: 終了処理で例外（{type(exc).__name__}: {exc}）")


async def amain(args: argparse.Namespace) -> int:
    runner = ServeRunner(args)
    return await runner.run()

