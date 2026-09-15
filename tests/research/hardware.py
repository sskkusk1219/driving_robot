"""研究開発用ハーネスのハードウェア層（手順 1: 初期化）。

ProblemReport_20260910 の手順 1「本番環境から初期化（サーボ通信チェック、エラー消去、
CAN通信チェック、UPS通信チェック、アクチュエータ初期位置）コードを引用」に対応する。

引用元と対応:
    src/app/factory.py            … 実 HW の組み立て（ポート・ボーレート・DBC の渡し方）
    src/app/robot_controller.py   … start() の接続順、initialize() の初期化順
        通信確認(ブレーキ→アクセル→CAN) → アラームリセット → サーボON → 原点復帰
    src/domain/pre_check.py       … 合否判定のしきい値（UPS残量・原点許容パルス）

本番との違い（研究用にシンプルにしたところ）:
    - 状態機械（RobotState）・WebSocket 配信・DB は持たない。判定結果は戻り値と print だけ。
    - 原点復帰スキップ（last_normal_shutdown）は持たない。毎回原点復帰する。
    - キャリブレーション/プロファイル確認は行わない（開度→パルス変換は手順 2 で扱う）。

ハードウェアモード:
    stub … 実機なしで流れを検証する。ペダルは内部変数だけ動き、車速は最小の車両モデル
            （StubVehicle）がペダル位置から計算する（手順 2 以降の走行を一巡させるため）。
    real … 実アクチュエータ・実 CAN・実 UPS。**アクチュエータが物理的に動く**ため
            ユーザーが `--hw real` を明示したときだけ構築する。
"""

from __future__ import annotations

import asyncio
import logging
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

from src.domain.pre_check import HOME_POSITION_TOLERANCE_PULSE, UPS_MIN_BATTERY_PCT
from src.infra.settings import AppSettings, load_settings
from src.models.profile import FeedforwardParams, coast_decel_at
from tests.research.axis_monitor import AxisMonitor
from tests.research.config import ChecksSection, ResearchConfig
from tests.research.term import display_width, say
from tests.research.vehicle import feedforward_params, pulse_to_opening

HW_STUB = "stub"
HW_REAL = "real"


class InitializationError(Exception):
    """初期化シーケンスの失敗。1 項目でも NG ならこれを投げて走行に進ませない。"""


class PreDriveCheckError(Exception):
    """走行前チェックの失敗（NG 項目あり・ブレーキを踏んでも停車しない）。走行に進ませない。"""


class DriveError(Exception):
    """走行を完了できなかった（ペダル探索の失敗・非常停止・タイムアウト）。"""


# ─────────────────────────────────────────────────────────────────────
# ハードウェアの最小インターフェース（本番クラスがそのまま満たす）
# ─────────────────────────────────────────────────────────────────────


class ActuatorProtocol(Protocol):
    # 最後に送った目標位置 [pulse]（未指令なら None）。ログの「指令」列に使う（A7）
    last_command_pos: int | None

    async def connect(self) -> None: ...

    async def close(self) -> None: ...

    async def enable_modbus_control(self) -> None: ...

    async def reset_alarm(self) -> None: ...

    async def servo_on(self) -> None: ...

    async def servo_off(self) -> None: ...

    async def home_return(self) -> None: ...

    async def read_position(self) -> int: ...

    async def is_alarm_active(self) -> bool: ...

    async def move_to_position(self, pos: int, *, smooth_over_s: float | None = None) -> None: ...

    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None: ...

    async def read_current(self) -> float: ...

    async def read_monitor(self) -> AxisMonitor: ...


class CANProtocol(Protocol):
    async def connect(self) -> None: ...

    async def close(self) -> None: ...

    async def read_speed(self) -> float: ...


class UPSProtocol(Protocol):
    async def start_polling(self) -> None: ...

    async def stop_polling(self) -> None: ...

    async def get_battery_level_pct(self) -> float: ...


# ─────────────────────────────────────────────────────────────────────
# スタブ（実機なしでシーケンスを流すため。src/app/stubs.py を研究用に拡張）
# ─────────────────────────────────────────────────────────────────────


@dataclass
class StubActuator:
    """スタブ軸。位置指令を内部変数に反映するだけで、物理的には何も動かない。

    本番 `_StubActuator` と違い現在位置を保持する。手順 1 のアクチュエータ位置確認
    （原点復帰後に |位置| <= 許容パルス）が本物と同じ経路で検証できるようにするため。
    """

    axis_name: str
    position: int = 0
    servo_on_state: bool = False
    alarm: bool = False
    connected: bool = False
    homed: bool = False
    last_command_pos: int | None = None

    async def connect(self) -> None:
        self.connected = True

    async def close(self) -> None:
        self.connected = False

    async def enable_modbus_control(self) -> None:
        self._require_connected()

    async def reset_alarm(self) -> None:
        self._require_connected()
        self.alarm = False

    async def servo_on(self) -> None:
        self._require_connected()
        self.servo_on_state = True

    async def servo_off(self) -> None:
        self._require_connected()
        self.servo_on_state = False

    async def home_return(self) -> None:
        self._require_connected()
        self.position = 0
        self.last_command_pos = 0
        self.homed = True

    async def read_position(self) -> int:
        self._require_connected()
        return self.position

    async def is_alarm_active(self) -> bool:
        self._require_connected()
        return self.alarm

    async def move_to_position(self, pos: int, *, smooth_over_s: float | None = None) -> None:
        """手順 2 以降のペダル指令用。遅れなしで即座に到達する理想アクチュエータ。"""
        self._require_connected()
        self.position = pos
        self.last_command_pos = pos

    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None:
        """時間指定移動（PatternLoop が使う）。スタブは遅れなしで即座に到達する。"""
        self._require_connected()
        self.position = target_pos
        self.last_command_pos = target_pos

    async def read_current(self) -> float:
        self._require_connected()
        return 0.0

    async def read_monitor(self) -> AxisMonitor:
        """まとめ読み（A7）。遅れの無い理想アクチュエータなので実位置 = 指令・常に位置決め完了。"""
        self._require_connected()
        return AxisMonitor(
            position_pulse=self.position,
            current_ma=await self.read_current(),  # テストが read_current を差し替えて電流を作る
            alarm_code=1 if self.alarm else 0,
            servo_on=self.servo_on_state,
            moving=False,
            pos_done=True,
        )

    def _require_connected(self) -> None:
        if not self.connected:
            raise RuntimeError(f"{self.axis_name}: connect() を先に呼んでください")


# スタブ車両の定数。実車の同定値ではなく、手順 2 の流れを一巡させるための目安。
# 遊び（原点→ペダル接触＋ペダルの遊び）は設定の不感帯とは独立に持ち、手順 2-0 のペダル探索が
# これを当てられるかをテストで確かめる。
STUB_ACCEL_GAIN_KMHS_PER_PCT = 0.25  # アクセル 40% で約 +8.5 km/h/s
STUB_BRAKE_GAIN_KMHS_PER_PCT = 0.5
STUB_ACCEL_PLAY_PCT = 6.0
STUB_BRAKE_PLAY_PCT = 8.0
STUB_CREEP_STIFFNESS_PER_S = 2.0  # クリープ車速へ引き戻す強さ [1/s]
_STUB_MAX_STEP_S = 0.5  # 読み取り間隔が空いても 1 回に積分する時間の上限


@dataclass
class StubVehicle:
    """スタブ HW 用の最小車両モデル。CAN 車速を読むたびに前回からの経過時間ぶん積分する。

    加速度は scripts/simulate_control.py の `_accel_of` と同じ形:
        a(v) = a_base(v) + (アクセル開度 − 遊び) × アクセルゲイン
                         − (ブレーキ開度 − 遊び) × ブレーキゲイン
    a_base はクリープ車速へ引き戻す力 k×(v_creep − v) を、上は +クリープ加速率、下は −惰行減速量
    で挟んだもの。本番 pedal_plan.coast_accel はクリープ車速で段差があり、微小な踏み込みに車速が
    反応しないため、ペダル探索が成り立つよう連続にしている。
    """

    accel: StubActuator
    brake: StubActuator
    params: FeedforwardParams
    accel_gain_kmhs_per_pct: float = STUB_ACCEL_GAIN_KMHS_PER_PCT
    brake_gain_kmhs_per_pct: float = STUB_BRAKE_GAIN_KMHS_PER_PCT
    accel_play_pct: float = STUB_ACCEL_PLAY_PCT
    brake_play_pct: float = STUB_BRAKE_PLAY_PCT
    creep_stiffness_per_s: float = STUB_CREEP_STIFFNESS_PER_S
    last_t: float | None = None

    def advance(self, speed_kmh: float, now: float | None = None) -> float:
        now = time.monotonic() if now is None else now
        dt = 0.0 if self.last_t is None else max(0.0, min(now - self.last_t, _STUB_MAX_STEP_S))
        self.last_t = now
        p = self.params
        accel_pct = pulse_to_opening(self.accel.position)
        brake_pct = pulse_to_opening(self.brake.position)
        pull = self.creep_stiffness_per_s * (p.creep_speed_kmh - speed_kmh)
        a = min(p.creep_rate_kmhs, max(-coast_decel_at(p, speed_kmh), pull))
        a += max(0.0, accel_pct - self.accel_play_pct) * self.accel_gain_kmhs_per_pct
        a -= max(0.0, brake_pct - self.brake_play_pct) * self.brake_gain_kmhs_per_pct
        return max(0.0, speed_kmh + a * dt)


@dataclass
class StubCANReader:
    """スタブ CAN。`vehicle` があれば読むたびに車両モデルで車速を更新して返す。

    `vehicle` が無ければ `speed_kmh` に入れた値をそのまま返す。
    """

    speed_kmh: float = 0.0
    connected: bool = False
    vehicle: StubVehicle | None = None

    async def connect(self) -> None:
        self.connected = True

    async def close(self) -> None:
        self.connected = False

    async def read_speed(self) -> float:
        if not self.connected:
            raise RuntimeError("CAN: connect() を先に呼んでください")
        if self.vehicle is not None:
            self.speed_kmh = self.vehicle.advance(self.speed_kmh)
        return self.speed_kmh


@dataclass
class StubUPSMonitor:
    """スタブ UPS。常に満充電・AC 通電中を返す（本番 `_StubUPSMonitor` と同じ）。"""

    battery_pct: float = 100.0
    available: bool = True

    async def start_polling(self) -> None:
        pass

    async def stop_polling(self) -> None:
        pass

    async def get_battery_level_pct(self) -> float:
        if not self.available:
            raise RuntimeError("NUT サーバーに接続できません（スタブ設定）")
        return self.battery_pct


# ─────────────────────────────────────────────────────────────────────
# ハードウェア束
# ─────────────────────────────────────────────────────────────────────


class _BenignActuatorNoiseFilter(logging.Filter):
    """`ActuatorDriver` の正常系ログ（速度クランプ・再送成功）はエラーではないため黙らせる。

    再送上限到達（直後に例外を送出する経路）は実際の失敗なので通す。
    """

    def filter(self, record: logging.LogRecord) -> bool:
        msg = record.getMessage()
        return "Modbus再送検知:" not in msg and "のためクランプ" not in msg


def _configure_actuator_logging() -> None:
    logging.getLogger("src.infra.actuator_driver").addFilter(_BenignActuatorNoiseFilter())


@dataclass
class ResearchHardware:
    """研究用ハーネスが触るハードウェア一式。"""

    accel: ActuatorProtocol
    brake: ActuatorProtocol
    can: CANProtocol
    ups: UPSProtocol
    hw_mode: str
    settings: AppSettings | None = None
    connected: bool = False
    initialized: bool = False

    @property
    def is_real(self) -> bool:
        return self.hw_mode == HW_REAL


def build_hardware(cfg: ResearchConfig, hw_mode: str) -> ResearchHardware:
    """設定とハードウェアモードから HW 一式を組み立てる（まだ接続はしない）。

    実機の構築引数は本番 `src/app/factory.py::build_real_controller` と同一にし、
    ポート・ボーレート・DBC・スレーブ ID の扱いを本番から乖離させない。
    """
    if hw_mode == HW_STUB:
        say("ハードウェア: スタブ（実機には触りません）")
        accel = StubActuator(axis_name="accel")
        brake = StubActuator(axis_name="brake")
        params = feedforward_params(cfg)
        vehicle = StubVehicle(accel=accel, brake=brake, params=params)
        return ResearchHardware(
            accel=accel,
            brake=brake,
            # シャシダイナモ上で D レンジ・クリープ中から始める（実機と同じく、走行前チェックで
            # ブレーキを刻んで止める流れを通すため）
            can=StubCANReader(speed_kmh=params.creep_speed_kmh, vehicle=vehicle),
            ups=StubUPSMonitor(),
            hw_mode=HW_STUB,
        )
    if hw_mode != HW_REAL:
        raise InitializationError(f"未知のハードウェアモード: {hw_mode!r}")

    # 実機構築は import を遅延させる。スタブ実行に pymodbus / python-can を要求しないため。
    from src.infra.can_reader import CANReader  # noqa: PLC0415
    from src.infra.ups_monitor import NutUPSMonitor  # noqa: PLC0415
    from tests.research.research_actuator import ResearchActuatorDriver  # noqa: PLC0415

    _configure_actuator_logging()

    settings_path = Path(cfg.hardware.settings_path)
    if not settings_path.exists():
        raise InitializationError(
            f"本番ハードウェア設定が見つかりません: {settings_path}"
            "（config/settings.toml.example からコピーしてください）"
        )
    settings = load_settings(settings_path)
    say(f"ハードウェア: 実機（設定 {settings_path}）")
    say(f"  アクセル軸 : {settings.serial.accel_port} @ {settings.serial.baud_rate}bps")
    say(f"  ブレーキ軸 : {settings.serial.brake_port} @ {settings.serial.baud_rate}bps")
    say(f"  CAN        : {settings.can.interface} ch{settings.can.channel} "
        f"{settings.can.bitrate}bps  DBC={settings.can.dbc_path}")
    if cfg.checks.init_ups:
        say(f"  UPS (NUT)  : {settings.ups.nut_host}:{settings.ups.nut_port} "
            f"ups={settings.ups.ups_name}")
    else:
        say("  UPS (NUT)  : 監視しない（checks.init_ups: false）")

    # 本番 ActuatorDriver にまとめ読み（read_monitor）と指令位置の記憶を足した研究用（A7）
    accel = ResearchActuatorDriver(
        port=settings.serial.accel_port,
        slave_id=1,
        baud_rate=settings.serial.baud_rate,
        timeout=settings.serial.timeout_s,
        retries=settings.serial.retries,
        axis_name="accel",
        lead_mm=settings.actuator.accel.lead_mm,
    )
    brake = ResearchActuatorDriver(
        port=settings.serial.brake_port,
        slave_id=1,  # 各軸が独立した RS-485 バスを持つため両軸とも slave_id=1
        baud_rate=settings.serial.baud_rate,
        timeout=settings.serial.timeout_s,
        retries=settings.serial.retries,
        axis_name="brake",
        lead_mm=settings.actuator.brake.lead_mm,
    )
    can = CANReader(
        interface=settings.can.interface,
        channel=settings.can.channel,
        bitrate=settings.can.bitrate,
        dbc_path=settings.can.dbc_path,
        # 鮮度しきい値は研究用設定を優先する（本番 settings.can.max_speed_age_s と同義）
        max_speed_age_s=cfg.control.can_max_speed_age_s,
    )
    ups = NutUPSMonitor(
        nut_host=settings.ups.nut_host,
        nut_port=settings.ups.nut_port,
        ups_name=settings.ups.ups_name,
        poll_interval_s=settings.ups.poll_interval_s,
    )
    return ResearchHardware(
        accel=accel, brake=brake, can=can, ups=ups, hw_mode=HW_REAL, settings=settings
    )


# ─────────────────────────────────────────────────────────────────────
# 初期化シーケンス
# ─────────────────────────────────────────────────────────────────────


@dataclass
class CheckResult:
    """初期化 1 項目の結果。本番 `PreCheckItemResult` の研究用縮小版。"""

    name: str
    passed: bool
    detail: str = ""
    skipped: bool = False  # checks.xxx: false で実行しなかった項目（判定はしない＝passed=True扱い）


@dataclass
class InitReport:
    """初期化シーケンス全体の結果。"""

    items: list[CheckResult] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(item.passed for item in self.items)

    @property
    def ok_count(self) -> int:
        return sum(1 for item in self.items if item.passed and not item.skipped)

    @property
    def skip_count(self) -> int:
        return sum(1 for item in self.items if item.skipped)

    def add(self, name: str, *, passed: bool, detail: str = "") -> CheckResult:
        result = CheckResult(name=name, passed=passed, detail=detail)
        self.items.append(result)
        mark = "OK  " if passed else "NG  "
        say(f"  [{mark}] {name}{f'  … {detail}' if detail else ''}")
        return result

    def skip(self, name: str, key: str) -> CheckResult:
        """`checks.{key}: false` で実行しなかった項目を記録する（判定なし・NG 扱いにはしない）。"""
        detail = f"checks.{key}: false"
        result = CheckResult(name=name, passed=True, detail=detail, skipped=True)
        self.items.append(result)
        say(f"  [SKIP] {name}  … {detail}")
        return result

    def render(self) -> None:
        """項目一覧を表にして表示する。"""
        width = max(display_width(item.name) for item in self.items)
        say("── 初期化結果 " + "─" * 52)
        for item in self.items:
            pad = " " * (width - display_width(item.name))
            mark = "SKIP" if item.skipped else ("OK" if item.passed else "NG")
            say(f"  {item.name}{pad} : {mark}  {item.detail}")
        say("─" * 68)


async def run_initialize(hw: ResearchHardware, checks: ChecksSection | None = None) -> InitReport:
    """接続 → サーボ通信 → エラー消去 → サーボON → CAN → UPS → 原点復帰。

    1 項目でも NG なら `InitializationError` を投げる（走行に進ませない）。
    順序は本番 `RobotController.start()` + `initialize()` に合わせている。

    Args:
        checks: `checks.init_*` が false の項目は実行・判定せず [SKIP] にする
            （`config_testVehicle.yaml` の `checks:` セクション）。
            None なら全項目実行（既定・安全側）。
    """
    c = checks if checks is not None else ChecksSection()
    report = InitReport()
    try:
        await _connect_all(hw, report)  # 接続は切替対象外（常に実行）
        if c.init_servo_comm:
            await _check_servo_comm(hw, report)
        else:
            report.skip("サーボ通信チェック", "init_servo_comm")
        if c.init_clear_errors:
            await _clear_errors(hw, report)
        else:
            report.skip("エラー消去", "init_clear_errors")
        if c.init_servo_on:
            await _servo_on(hw, report)
        else:
            report.skip("サーボON", "init_servo_on")
        if c.init_can:
            await _check_can(hw, report)
        else:
            report.skip("CAN 通信チェック", "init_can")
        if c.init_ups:
            await _check_ups(hw, report)
        else:
            report.skip("UPS 通信チェック", "init_ups")
        if c.init_home_return:
            await _home_return(hw, report)
        else:
            report.skip("アクチュエータ初期位置", "init_home_return")
    except InitializationError:
        say()
        report.render()
        raise
    except Exception as exc:  # 予期しない例外も表を出してから包み直す
        report.add(f"予期しないエラー ({type(exc).__name__})", passed=False, detail=str(exc))
        say()
        report.render()
        raise InitializationError(f"初期化中に例外が発生しました: {exc}") from exc

    hw.initialized = True
    say()
    report.render()
    return report


def _fail(report: InitReport, name: str, detail: str) -> None:
    report.add(name, passed=False, detail=detail)
    raise InitializationError(f"{name}: {detail}")


async def _connect_all(hw: ResearchHardware, report: InitReport) -> None:
    """接続（本番 RobotController.start() 相当）。アクセル/ブレーキ/CAN を同時に開く。"""
    say("接続中: アクセル軸 / ブレーキ軸 / CAN …")
    # 一部だけ開いて残りが失敗した場合も shutdown で確実に閉じられるよう、試行前に立てる
    # （片方のシリアルポートを掴んだまま終了すると次回の実行が接続できない）
    hw.connected = True
    try:
        await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    except Exception as exc:
        _fail(report, "接続", f"{type(exc).__name__}: {exc}")
    report.add("接続 (アクセル/ブレーキ/CAN)", passed=True)


async def _check_servo_comm(hw: ResearchHardware, report: InitReport) -> None:
    """サーボ通信チェック。

    本番 initialize() と同じく Modbus 操作権（PMSL）を**ブレーキ→アクセルの順**で有効化し、
    続けて現在位置を読んで往復通信を確認する（本番 pre_check の「通信確認」相当）。
    ブレーキを先にするのは、万一片側しか通らないときに安全側の軸を先に握るため。
    """
    say("サーボ通信チェック: Modbus 操作権 (PMSL) → 現在位置読み出し …")
    for label, axis in (("ブレーキ", hw.brake), ("アクセル", hw.accel)):
        try:
            await axis.enable_modbus_control()
        except Exception as exc:
            _fail(report, f"Modbus 操作権 ({label})", f"{type(exc).__name__}: {exc}")
    report.add("Modbus 操作権 (ブレーキ→アクセル)", passed=True)

    try:
        accel_pos = await hw.accel.read_position()
        brake_pos = await hw.brake.read_position()
    except Exception as exc:
        _fail(report, "サーボ通信確認", f"{type(exc).__name__}: {exc}")
        return
    report.add(
        "サーボ通信確認 (現在位置読み出し)",
        passed=True,
        detail=f"アクセル={accel_pos}pulse ブレーキ={brake_pos}pulse",
    )


async def _clear_errors(hw: ResearchHardware, report: InitReport) -> None:
    """エラー消去。両軸のアラームをリセットし、消えたことを ALMC 読み出しで確認する。"""
    say("エラー消去: アラームリセット (両軸同時) → 残留確認 …")
    try:
        await asyncio.gather(hw.accel.reset_alarm(), hw.brake.reset_alarm())
        accel_alarm = await hw.accel.is_alarm_active()
        brake_alarm = await hw.brake.is_alarm_active()
    except Exception as exc:
        _fail(report, "エラー消去", f"{type(exc).__name__}: {exc}")
        return
    remaining = [
        name
        for name, active in (("アクセル軸", accel_alarm), ("ブレーキ軸", brake_alarm))
        if active
    ]
    if remaining:
        # リセットで消えないアラームは電源再投入や配線が必要。ここで止めるのが正しい。
        _fail(report, "エラー消去", f"アラームが残っています: {', '.join(remaining)}")
    report.add("エラー消去 (アラームリセット)", passed=True, detail="残留アラームなし")


async def _servo_on(hw: ResearchHardware, report: InitReport) -> None:
    """サーボ ON（両軸同時。本番 initialize() と同じ）。"""
    say("サーボON (両軸同時) …")
    try:
        await asyncio.gather(hw.accel.servo_on(), hw.brake.servo_on())
    except Exception as exc:
        _fail(report, "サーボON", f"{type(exc).__name__}: {exc}")
    report.add("サーボON (両軸)", passed=True)


async def _check_can(hw: ResearchHardware, report: InitReport) -> None:
    """CAN 通信チェック。車速が読めることだけを確認する（本番 initialize() と同じ）。

    停車判定はしない。本番も initialize() は read_speed() で疎通だけを見ており、
    車速確認は走行前チェック（pre_check の「車速確認」）の担当。
    """
    say("CAN 通信チェック: 車速読み出し …")
    try:
        speed = await hw.can.read_speed()
    except Exception as exc:
        _fail(report, "CAN 通信チェック", f"{type(exc).__name__}: {exc}")
        return
    report.add("CAN 通信チェック (車速読み出し)", passed=True, detail=f"車速={speed:.2f}km/h")


async def _check_ups(hw: ResearchHardware, report: InitReport) -> None:
    """UPS 通信チェック。NUT からバッテリー残量を取り、本番と同じ下限で判定する。"""
    say("UPS 通信チェック: NUT ポーリング開始 → 残量取得 …")
    try:
        await hw.ups.start_polling()
        level = await hw.ups.get_battery_level_pct()
    except Exception as exc:
        _fail(report, "UPS 通信チェック", f"{type(exc).__name__}: {exc}")
        return
    if level < UPS_MIN_BATTERY_PCT:
        _fail(
            report,
            "UPS 通信チェック",
            f"UPS 残量不足: {level:.1f}%（{UPS_MIN_BATTERY_PCT:.0f}% 以上必要）",
        )
    report.add("UPS 通信チェック (残量)", passed=True, detail=f"残量={level:.1f}%")


async def _home_return(hw: ResearchHardware, report: InitReport) -> None:
    """アクチュエータ初期位置。原点復帰 → 位置が許容範囲内にあることを確認する。

    本番は前回正常終了時にスキップするが、研究用は毎回実施して基準位置を揃える。
    """
    if hw.is_real:
        say("*** アクチュエータ初期位置: これから両軸が物理的に動きます（最大 30s/軸） ***")
    say("原点復帰 (両軸同時) → 位置確認 …")
    try:
        await asyncio.gather(hw.accel.home_return(), hw.brake.home_return())
        accel_pos = await hw.accel.read_position()
        brake_pos = await hw.brake.read_position()
    except Exception as exc:
        _fail(report, "アクチュエータ初期位置", f"{type(exc).__name__}: {exc}")
        return
    off_home = [
        f"{name}={pos}pulse"
        for name, pos in (("アクセル軸", accel_pos), ("ブレーキ軸", brake_pos))
        if abs(pos) > HOME_POSITION_TOLERANCE_PULSE
    ]
    if off_home:
        _fail(
            report,
            "アクチュエータ初期位置",
            f"原点から離れています（許容 ±{HOME_POSITION_TOLERANCE_PULSE}pulse）: "
            f"{', '.join(off_home)}",
        )
    report.add(
        "アクチュエータ初期位置 (原点復帰)",
        passed=True,
        detail=f"アクセル={accel_pos}pulse ブレーキ={brake_pos}pulse",
    )


async def shutdown(hw: ResearchHardware) -> None:
    """終了処理: 原点復帰 → サーボOFF → 切断（本番 stop() と同じ順）。

    ここは「失敗しても残りを必ず実行する」方針にする。1 つの失敗でペダルを踏んだまま
    プロセスが終わるのが最悪のため、各手順を個別に try で囲む。
    """
    if not hw.connected:
        return
    say("終了処理: 原点復帰 → サーボOFF → 切断 …")
    # コルーチンは呼び出し時に生成する（タプル内で先に生成すると gather が即座に走り出し、
    # 原点復帰とサーボOFF が同時に動いてしまう）
    steps: tuple[tuple[str, Callable[[], Awaitable[object]]], ...] = (
        ("原点復帰", lambda: asyncio.gather(hw.accel.home_return(), hw.brake.home_return())),
        ("サーボOFF", lambda: asyncio.gather(hw.accel.servo_off(), hw.brake.servo_off())),
        ("UPS ポーリング停止", hw.ups.stop_polling),
        ("切断", lambda: asyncio.gather(hw.accel.close(), hw.brake.close(), hw.can.close())),
    )
    for label, make_coro in steps:
        try:
            await make_coro()
        except Exception as exc:
            say(f"  警告: {label}に失敗しました（{type(exc).__name__}: {exc}）")
    hw.connected = False
    hw.initialized = False
    say("終了処理: 完了")
