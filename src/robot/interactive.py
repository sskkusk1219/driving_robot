# ruff: noqa: E501
"""対話ジョブ（``--interactive``。手動運転・キャリブレーション用）。

手順 1（初期化）だけ行い、標準入力の JSON 1 行コマンドを受け付ける。終わりは必ず core の shutdown
（run_steps の finally）。コマンドと結果（ack・status イベント）の形式は src/robot/README.md。

    {"cmd":"jog","axis":"accel","amount_pct":2.0}     今の位置から ±量 [%] 動かす
    {"cmd":"move","axis":"brake","target_pct":20}     目標の開度 [%] へ動かす
    {"cmd":"home"}                                    両軸を原点へ
    {"cmd":"status"}                                  位置・開度・車速を status イベントで返す
    {"cmd":"save_min","axis":"accel"}                 今の位置を vehicle.min_*_opening_pct として車両 yaml へ
    {"cmd":"save_max","axis":"accel"}                 今の位置を vehicle.max_*_opening_pct として車両 yaml へ（段11）
    {"cmd":"quit"}                                    終了（終了処理へ）

どのコマンドにも ``"id"``（任意）を付けると ack に同じ値が返る。

開度の定義は core と同じ（原点 0% ・ 9500 pulse = 100%）。守ること:
- 開度は 0 〜 vehicle.max_*_opening_pct に収める（超える指令は max にして ack に知らせる）。
  ただし jog/move に ``"calib": true`` を付けた（キャリブレーション画面の）ときだけ、上限を
  ``CALIB_STROKE_LIMIT_PCT``（99%。ストロークの端 100% の手前）まで広げる（max を測るため。段11）。
- 片足ルール: 片方が min を超えているとき他方は min。踏む側を動かす前に、もう一方を先に min へ戻す
  （戻すのはいつも安全側なので、確認を挟まず自動で戻して ack に書く）。
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from src.robot import config_store, events
from src.robot.core.config import ResearchConfig
from src.robot.core.hardware import ResearchHardware
from src.robot.core.pre_drive_check import DIRECT_PRESS_SPEED_MM_S
from src.robot.core.term import say
from src.robot.core.vehicle import opening_to_pulse, pulse_to_opening

AXES = ("accel", "brake")
COMMANDS = ("jog", "move", "home", "status", "save_min", "save_max", "quit")

#: キャリブレーションで max を測るときだけ許す上限 [%]。ストロークの端（100% = 9500 pulse）に当てると
#: 機構を痛める・サーボのアラームになるので、1%（95 pulse ≒ 0.95 mm）手前で止める。手動運転は今までどおり max まで
CALIB_STROKE_LIMIT_PCT = 99.0
#: save_max で max が min からこれ以上離れていないと保存しない [%]
MAX_MIN_GAP_PCT = 1.0

#: 「min を超えている」の判定の余裕 [%]（読み戻しの丸め誤差）
ONE_PEDAL_EPS_PCT = 0.05
#: 移動の後に待つ余裕 [s]
SETTLE_S = 0.3
#: jog の 1 回の最大量 [%]
MAX_JOG_PCT = 100.0

_PULSE_PER_MM = 100.0


class CommandError(ValueError):
    """コマンドの形式・値の誤り（日本語の案内つき。ack の message になる）。"""


@dataclass(frozen=True)
class Command:
    cmd: str
    axis: str | None = None
    value: float | None = None  # jog: amount_pct / move: target_pct
    req_id: Any = None
    calib: bool = False  # jog/move: 上限を max でなくストロークの端の手前（CALIB_STROKE_LIMIT_PCT）にする


@dataclass(frozen=True)
class Limits:
    """ペダルごとの min（触れる位置）と max（上限）[%]。"""

    min_accel: float
    max_accel: float
    min_brake: float
    max_brake: float

    @classmethod
    def from_config(cls, cfg: ResearchConfig) -> Limits:
        v = cfg.vehicle
        return cls(
            v.min_accel_opening_pct,
            v.max_accel_opening_pct,
            v.min_brake_opening_pct,
            v.max_brake_opening_pct,
        )

    def for_calib(self) -> Limits:
        """キャリブレーション用: max を ``CALIB_STROKE_LIMIT_PCT`` に広げた制限（min はそのまま）。"""
        return Limits(self.min_accel, CALIB_STROKE_LIMIT_PCT, self.min_brake, CALIB_STROKE_LIMIT_PCT)

    def min_of(self, axis: str) -> float:
        return self.min_accel if axis == "accel" else self.min_brake

    def max_of(self, axis: str) -> float:
        return self.max_accel if axis == "accel" else self.max_brake


@dataclass(frozen=True)
class MovePlan:
    """動かす順の (軸, 目標開度 [%])、制限した後の目標、max で頭打ちにしたか、先にもう一方を戻したか。"""

    moves: tuple[tuple[str, float], ...]
    target_pct: float
    clamped: bool
    released_other: bool


def _number(obj: dict[str, Any], key: str) -> float:
    if key not in obj:
        raise CommandError(f"{key} がありません")
    value = obj[key]
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise CommandError(f"{key} は数値で指定してください: {value!r}")
    if value != value or value in (float("inf"), float("-inf")):
        raise CommandError(f"{key} が有限の数ではありません")
    return float(value)


def parse_command(line: str) -> Command:
    """JSON 1 行 → Command。誤りは CommandError。"""
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
    if cmd not in COMMANDS:
        raise CommandError(f"未知のコマンド {cmd!r}（{', '.join(COMMANDS)}）")
    req_id = obj.get("id")
    if cmd in ("home", "status", "quit"):
        return Command(cmd, req_id=req_id)
    axis = obj.get("axis")
    if cmd == "save_min":
        if axis not in (*AXES, "both"):
            raise CommandError("axis は accel / brake / both で指定してください")
        return Command(cmd, axis=axis, req_id=req_id)
    if cmd == "save_max":
        if axis not in AXES:
            raise CommandError("axis は accel / brake で指定してください")
        return Command(cmd, axis=axis, req_id=req_id)
    if axis not in AXES:
        raise CommandError("axis は accel / brake で指定してください")
    if cmd == "jog":
        amount = _number(obj, "amount_pct")
        if amount == 0.0 or abs(amount) > MAX_JOG_PCT:
            raise CommandError(f"amount_pct は 0 でなく絶対値 {MAX_JOG_PCT:g} 以下にしてください")
        return Command(cmd, axis=axis, value=amount, req_id=req_id, calib=obj.get("calib") is True)
    target = _number(obj, "target_pct")
    if target < 0.0:
        raise CommandError("target_pct は 0 以上にしてください")
    return Command(cmd, axis=axis, value=target, req_id=req_id, calib=obj.get("calib") is True)


def plan_move(
    axis: str, target_pct: float, accel_pct: float, brake_pct: float, limits: Limits
) -> MovePlan:
    """目標の開度を max で頭打ちにし、片足ルールのために先に戻す軸があれば手順に足す。

    ``accel_pct`` / ``brake_pct`` は今の実開度。踏む側の目標が自分の min を超え、かつ
    もう一方が今 min を超えているときだけ、先にもう一方を min へ戻す。
    """
    if axis not in AXES:
        raise CommandError(f"axis が不正です: {axis!r}")
    target = max(0.0, target_pct)
    top = limits.max_of(axis)
    clamped = target > top
    if clamped:
        target = top
    other = "brake" if axis == "accel" else "accel"
    other_now = brake_pct if axis == "accel" else accel_pct
    release = target > limits.min_of(axis) + ONE_PEDAL_EPS_PCT and (
        other_now > limits.min_of(other) + ONE_PEDAL_EPS_PCT
    )
    moves: list[tuple[str, float]] = []
    if release:
        moves.append((other, limits.min_of(other)))
    moves.append((axis, target))
    return MovePlan(tuple(moves), target, clamped, release)


#: jog の目標開度をそろえる目盛り [%]（0.01% の目盛り。1 pulse ≒ 0.0105%）。画面のジョグの量は pulse なので、
#: そのまま足すと 4.42% のように半端な値になり、min を決めた値（例 4.2%）にちょうど合わせられない（2026-11-16 段9）。
#: 2026-11-21 に ±1 pulse のジョグを足すため、0.1% から細かくした（0.1% だと 1 pulse が丸められて動かない）
JOG_GRID_PCT = 0.01


def on_jog_grid(opening_pct: float) -> float:
    """保存する min・max をジョグの目盛り（``JOG_GRID_PCT``）に丸める。

    ジョグの行き先は 0.01% の目盛り（1 pulse ≒ 0.0105%。2026-11-21 に 0.1% から細かくした）だが、pulse（9500 = 100%）にすると半端になる（47.3% → 4493.5 → 4494
    pulse ＝ 47.305%）。実位置をそのまま保存すると 47.31 になり、狙った値とずれる（2026-11-16 段11 実機）。
    """
    return round(round(opening_pct / JOG_GRID_PCT) * JOG_GRID_PCT, 4)


def jog_target(axis: str, amount_pct: float, accel_pct: float, brake_pct: float) -> float:
    """jog の目標開度 = 今の実開度 + 量 を ``JOG_GRID_PCT``（0.01%）の目盛りに丸めた値（0 未満は 0。max は plan_move が収める）。

    1 pulse（≒0.0105%）を足しても丸めで消えない（目盛りの半分 0.005% ＝ 0.475 pulse 未満の誤差）。
    """
    now = accel_pct if axis == "accel" else brake_pct
    target = round((now + amount_pct) / JOG_GRID_PCT) * JOG_GRID_PCT
    return max(0.0, round(target, 4))


class InteractiveSession:
    """初期化済みの HW に対してコマンドを実行する。"""

    def __init__(
        self,
        hw: ResearchHardware,
        cfg: ResearchConfig,
        *,
        vehicle_id: str,
        vehicles_dir: Path,
    ) -> None:
        self._hw = hw
        self._cfg = cfg
        self._vehicle_id = vehicle_id
        self._vehicles_dir = vehicles_dir

    @property
    def limits(self) -> Limits:
        return Limits.from_config(self._cfg)

    def _axis(self, name: str) -> Any:
        return self._hw.accel if name == "accel" else self._hw.brake

    async def read_openings(self) -> tuple[float, float]:
        a = await self._hw.accel.read_position()
        b = await self._hw.brake.read_position()
        return pulse_to_opening(a), pulse_to_opening(b)

    async def _move_axis(self, name: str, target_pct: float) -> None:
        axis = self._axis(name)
        target_pulse = opening_to_pulse(target_pct)
        current = await axis.read_position()
        duration_s = abs(target_pulse - current) / _PULSE_PER_MM / DIRECT_PRESS_SPEED_MM_S
        if duration_s <= 0.0:
            return
        await axis.move_to_position_timed(target_pulse, current, duration_s)
        await asyncio.sleep(duration_s + SETTLE_S)

    async def status(self) -> dict[str, Any]:
        a_pulse = await self._hw.accel.read_position()
        b_pulse = await self._hw.brake.read_position()
        try:
            speed: float | None = round(await self._hw.can.read_speed(), 3)
        except Exception:  # noqa: BLE001 - 車速が読めなくても位置は返す
            speed = None
        lim = self.limits
        return {
            "accel_pct": round(pulse_to_opening(a_pulse), 3),
            "brake_pct": round(pulse_to_opening(b_pulse), 3),
            "accel_pulse": a_pulse,
            "brake_pulse": b_pulse,
            "speed_kmh": speed,
            "min_accel_pct": lim.min_accel,
            "min_brake_pct": lim.min_brake,
            "max_accel_pct": lim.max_accel,
            "max_brake_pct": lim.max_brake,
        }

    async def execute(self, command: Command) -> bool:
        """1 コマンドを実行して ack を出す。続けてよければ True、quit なら False。"""
        try:
            keep, message = await self._execute(command)
        except CommandError as exc:
            self._ack(command, False, str(exc))
            return True
        except Exception as exc:  # noqa: BLE001 - 通信エラーなどでも対話ジョブは続ける
            self._ack(command, False, f"{type(exc).__name__}: {exc}")
            return True
        self._ack(command, True, message)
        if command.cmd != "quit":
            events.emit("status", id=command.req_id, **(await self.status()))
        return keep

    def _ack(self, command: Command, ok: bool, message: str) -> None:
        say(f"対話: {command.cmd} → {'OK' if ok else 'NG'} {message}")
        events.emit("ack", id=command.req_id, cmd=command.cmd, ok=ok, message=message)

    async def _execute(self, command: Command) -> tuple[bool, str]:
        cmd = command.cmd
        if cmd == "quit":
            return False, "終了処理へ進みます"
        if cmd == "status":
            return True, "状態を返しました"
        if cmd == "home":
            await asyncio.gather(self._hw.accel.home_return(), self._hw.brake.home_return())
            return True, "両軸を原点へ戻しました"
        if cmd in ("jog", "move"):
            assert command.axis is not None and command.value is not None  # noqa: S101
            accel_pct, brake_pct = await self.read_openings()
            target = (
                jog_target(command.axis, command.value, accel_pct, brake_pct)
                if cmd == "jog"
                else command.value
            )
            limits = self.limits.for_calib() if command.calib else self.limits
            plan = plan_move(command.axis, target, accel_pct, brake_pct, limits)
            for name, pct in plan.moves:
                await self._move_axis(name, pct)
            notes = []
            if plan.released_other:
                notes.append(f"片足ルールのため先に {plan.moves[0][0]} を min へ戻した")
            if plan.clamped:
                notes.append(f"max {plan.target_pct:g}% に制限した")
            note = f"（{'・'.join(notes)}）" if notes else ""
            return True, f"{command.axis} を {plan.target_pct:.2f}% へ{note}"
        if cmd == "save_min":
            return True, await self._save_min(command.axis or "")
        if cmd == "save_max":
            return True, await self._save_max(command.axis or "")
        raise CommandError(f"未対応のコマンド: {cmd}")

    async def _save_min(self, axis: str) -> str:
        """今の実位置を min として車両 yaml に保存する（0 以上・max 未満。写しは次回の合成で効く）。"""
        names = AXES if axis == "both" else (axis,)
        accel_pct, brake_pct = await self.read_openings()
        updates: dict[str, float] = {}
        for name in names:
            value = on_jog_grid(accel_pct if name == "accel" else brake_pct)
            top = self.limits.max_of(name)
            if not 0.0 <= value < top:
                raise CommandError(f"{name} の今の位置 {value:g}% は min にできません（0 以上 {top:g}% 未満）")
            updates[f"vehicle.min_{name}_opening_pct"] = value
        config_store.save_vehicle_values(
            self._vehicle_id, updates, vehicles_dir=self._vehicles_dir
        )
        for name in names:
            setattr(
                self._cfg.vehicle,
                f"min_{name}_opening_pct",
                updates[f"vehicle.min_{name}_opening_pct"],
            )
        return "車両 yaml に保存: " + ", ".join(f"{k}={v:g}" for k, v in updates.items())

    async def _save_max(self, axis: str) -> str:
        """今の実位置を max（開度上限）として車両 yaml に保存する（min より ``MAX_MIN_GAP_PCT`` 以上大きく、
        ``CALIB_STROKE_LIMIT_PCT`` 以下）。"""
        accel_pct, brake_pct = await self.read_openings()
        value = on_jog_grid(accel_pct if axis == "accel" else brake_pct)
        low = self.limits.min_of(axis)
        if not low + MAX_MIN_GAP_PCT <= value <= CALIB_STROKE_LIMIT_PCT:
            raise CommandError(
                f"{axis} の今の位置 {value:g}% は max にできません（min {low:g}% + {MAX_MIN_GAP_PCT:g}% 以上・{CALIB_STROKE_LIMIT_PCT:g}% 以下）"
            )
        key = f"vehicle.max_{axis}_opening_pct"
        config_store.save_vehicle_values(self._vehicle_id, {key: value}, vehicles_dir=self._vehicles_dir)
        setattr(self._cfg.vehicle, f"max_{axis}_opening_pct", value)
        return f"車両 yaml に保存: {key}={value:g}"

    async def run(self) -> None:
        """標準入力から 1 行ずつ読んで実行する（EOF・quit で戻る）。"""
        loop = asyncio.get_running_loop()
        queue: asyncio.Queue[str | None] = asyncio.Queue()

        def reader() -> None:
            # デーモンスレッド: 終了時に readline で止まっていてもプロセスの終了を妨げない
            for line in sys.stdin:
                loop.call_soon_threadsafe(queue.put_nowait, line)
            loop.call_soon_threadsafe(queue.put_nowait, None)

        threading.Thread(target=reader, daemon=True, name="interactive-stdin").start()
        events.emit("state", phase="interactive")
        say("対話ジョブ: コマンド待ち（JSON 1 行。quit か標準入力の EOF で終了）")
        events.emit("status", id=None, **(await self.status()))
        while True:
            line = await queue.get()
            if line is None:
                say("対話ジョブ: 標準入力が閉じられたので終了します")
                return
            if not line.strip():
                continue
            try:
                command = parse_command(line)
            except CommandError as exc:
                say(f"対話: コマンドが不正です（{exc}）")
                events.emit("ack", id=None, cmd=None, ok=False, message=str(exc))
                continue
            if not await self.execute(command):
                return
