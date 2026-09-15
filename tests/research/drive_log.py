"""走行ログ: 走行前チェック → ペダル探索 → パターン走行 → 緩減速〜停車保持 を 1 本の CSV/PNG に。

2026-09-13 A7（KAIZEN 表5-5 順6）から、手順 1 の走行前チェック以降の**すべての区間で同じ列・
同じ意味**にした（列は `CSV_COLUMNS` のコメント）。
    - 指示開度 FF [%]  … FF の出力をペダル別に分けた値（モード走行だけ）
    - 指示開度 FF・PID … アクチュエータへ送った最終指令 [%]・[mm]（全区間。パターン走行・
                         モード走行はその周期の指令、他の区間は各軸へ最後に送った目標位置）
    - 実開度           … 両軸を 0x9000 から 14 レジスタまとめ読みした実位置 PNOW [%]・[mm]
    - ステータス       … 区間・段・ガバナー・アラーム ＋ 軸ごとのサーボON・移動中・
                         位置決め完了・アラームコード
    - cycle_ms         … その行の読み取り（パターン・モード走行は 1 周期の処理）の実時間
旧形式（〜A6 の `accel_opening`・`ff_effort_pct` などの列）の CSV も `cmd_opening`・
`ff_effort` などで読める。

`section` 列で区間を分ける:
    PRE_DRIVE_CHECK … 走行前チェック（踏込前チェック / ブレーキを刻んで停止確認 / 踏込後チェック）
    PEDAL_SEARCH    … 2-0 ペダル探索
    PATTERN_DRIVE   … 2-1 パターン走行（FF モデルの学習に使うのはこの区間だけ）
    MODE_DRIVE      … 手順 3/5/7/9 のモード走行（KPI・レポートはこの区間だけ。
                      mode_time_s 列 = モード経過秒）
    DECEL_TO_STOP   … 走行後の緩減速 → 停車保持

記録の仕方:
    - パターン走行: PatternLoop（本番 LearningLoop 相当）の on_sample コールバックから
      1 行ごとに記録する（指令開度・指令位置）。
    - モード走行: 50ms 制御ループが log_every_n_cycles（100ms）ごとに記録する（指令開度・指令位置・
      effort 内訳）。
    - それ以外: バックグラウンドのサンプラーが output.csv_interval_s ごとに CAN 車速と両軸の
      まとめ読みを記録する（指令は各軸へ最後に送った目標位置）。パターン走行中は止める
      （100ms の制御ループと Modbus を取り合わないため）。
"""

from __future__ import annotations

import asyncio
import contextlib
import csv
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta
from pathlib import Path

from src.models.drive_log import DriveLog, DriveLogData
from src.utils.time import to_jst_naive
from tests.research.axis_monitor import AxisMonitor
from tests.research.config import ResearchConfig
from tests.research.hardware import ResearchHardware
from tests.research.live_plot import LivePlot, PlotSample, save_drive_figure
from tests.research.term import say
from tests.research.vehicle import pulse_to_opening

SECTION_PRE_DRIVE_CHECK = "PRE_DRIVE_CHECK"
SECTION_PEDAL_SEARCH = "PEDAL_SEARCH"
SECTION_PATTERN_DRIVE = "PATTERN_DRIVE"
SECTION_MODE_DRIVE = "MODE_DRIVE"
SECTION_DECEL_TO_STOP = "DECEL_TO_STOP"

LIVE_PNG_NAME = "live_drive.png"

PULSE_PER_MM = 100  # 位置 1 pulse = 0.01mm

# 2026-09-13 A7: 手順 1 からすべての区間で同じ列（KAIZEN 4.6 節末尾のログの提案）
CSV_COLUMNS: tuple[str, ...] = (
    # 時間
    "timestamp",
    "elapsed_s",
    "mode_time_s",  # モード経過秒（モード走行の行だけ）
    "cycle_ms",  # この行の読み取り（パターン・モード走行は 1 周期の処理）にかかった実時間 [ms]
    # 車速
    "ref_speed_kmh",
    "actual_speed_kmh",
    "deviation_kmh",  # 実車速 − 基準車速（基準のある行だけ）
    # 指示開度 FF（FF の出力をペダル別に分けた値。モード走行の行だけ）
    "accel_ff_pct",
    "brake_ff_pct",
    # 指示開度 FF・PID（アクチュエータへ送った最終指令。ガバナーで削った後・待機位置を含む）
    "accel_cmd_pct",
    "brake_cmd_pct",
    "accel_cmd_mm",
    "brake_cmd_mm",
    # 実開度（まとめ読みの PNOW。読めなかった行は空欄）
    "accel_actual_pct",
    "brake_actual_pct",
    "accel_actual_mm",
    "brake_actual_mm",
    # 電流 [mA]
    "accel_current",
    "brake_current",
    # ステータス
    "section",
    "candidate",  # V1: FF 候補名（C1〜C6）。モード走行の行だけ入る（他区間・旧 CSV は空欄）
    "pattern",
    "phase",
    "governor_active",  # 減速G ガバナーが頭打ち中か（モード・パターン走行。他の区間は False）
    "alarm_accel",  # アクセル軸のアラーム（未確認なら空欄。1.0s ごと・モード/パターン走行）
    "alarm_brake",  # ブレーキ軸のアラーム（未確認なら空欄。1.0s ごと・モード/パターン走行）
    "accel_servo_on",  # DSS1 SV（まとめ読み。読めなかった行は空欄。以下同じ）
    "accel_moving",  # DSSE MOVE
    "accel_pos_done",  # DSS1 PEND
    "accel_alarm_code",  # ALMC（0 = アラームなし）
    "brake_servo_on",
    "brake_moving",
    "brake_pos_done",
    "brake_alarm_code",
)


@dataclass(frozen=True)
class DriveSample:
    """ログ 1 行。"""

    elapsed_s: float
    timestamp: datetime  # UTC aware
    data: DriveLogData
    section: str
    phase: str
    # パターン走行は「番号:種類」（例 18:ACCEL_SWEEP）、モード走行は区間名（例 Mid）
    pattern: str = ""
    mode_time_s: float | None = None  # モード走行のみ、モード開始からの経過秒
    governor_active: bool = False  # 減速G ガバナーが頭打ち中か（モード走行・パターン走行）
    alarm_accel: bool | None = None  # アクセル軸のアラーム（未確認なら None）
    alarm_brake: bool | None = None  # ブレーキ軸のアラーム（未確認なら None）
    # A7: FF の出力のペダル別（モード走行だけ）・まとめ読み・この行の読み取りの実時間
    accel_ff_pct: float | None = None
    brake_ff_pct: float | None = None
    monitor_accel: AxisMonitor | None = None
    monitor_brake: AxisMonitor | None = None
    cycle_ms: float | None = None
    candidate: str = ""  # V1: FF 候補名（C1〜C5）。モード走行だけ入れる


class SessionLog:
    """1 回の走行（走行前チェック〜停車保持）のログ。

    `start` で時計・グラフ・サンプラーを始め、`close` で CSV/PNG に保存する。
    """

    def __init__(self, cfg: ResearchConfig, hw: ResearchHardware, *, has_ref: bool = False) -> None:
        self._cfg = cfg
        self._hw = hw
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_dir = cfg.results_path
        results_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = results_dir / f"drive_log_{hw.hw_mode}_{stamp}.csv"
        self.title = f"走行ログ {stamp}（{cfg.vehicle.name} / {hw.hw_mode}）"
        self.figure_path: Path | None = None
        self.samples: list[DriveSample] = []
        self._plot = LivePlot(
            results_dir / LIVE_PNG_NAME,
            title=self.title,
            max_speed_kmh=cfg.vehicle.max_speed_kmh,
            has_ref=has_ref,
            enabled=cfg.output.plot,
        )
        self._t0 = time.monotonic()
        self._wall0 = datetime.now(tz=UTC)
        self._section = ""
        self._phase = ""
        self._sampler: asyncio.Task[None] | None = None
        self._sampler_stop = asyncio.Event()
        self._closed = False

    def start(self, section: str, phase: str) -> None:
        """経過秒 0 をここにし、グラフとサンプラーを始める。"""
        self._t0 = time.monotonic()
        self._wall0 = datetime.now(tz=UTC)
        self.mark(section, phase)
        say(f"走行ログ: {self.csv_path}（{self._cfg.output.csv_interval_s:g}s 刻み。"
            "走行前チェック〜停車保持を 1 本に記録）")
        self._plot.start()
        self.start_sampler()

    @property
    def closed(self) -> bool:
        return self._closed

    def elapsed_s(self) -> float:
        return time.monotonic() - self._t0

    def mark(self, section: str, phase: str) -> None:
        """サンプラーが記録する行の区間・段を切り替える。"""
        self._section, self._phase = section, phase

    def record(
        self,
        data: DriveLogData,
        *,
        section: str | None = None,
        phase: str | None = None,
        pattern: str = "",
        mode_time_s: float | None = None,
        governor_active: bool = False,
        alarm_accel: bool | None = None,
        alarm_brake: bool | None = None,
        accel_ff_pct: float | None = None,
        brake_ff_pct: float | None = None,
        monitor_accel: AxisMonitor | None = None,
        monitor_brake: AxisMonitor | None = None,
        cycle_ms: float | None = None,
        candidate: str = "",
    ) -> DriveSample:
        elapsed = self.elapsed_s()
        sample = DriveSample(
            elapsed_s=elapsed,
            timestamp=self._wall0 + timedelta(seconds=elapsed),
            data=data,
            section=self._section if section is None else section,
            phase=self._phase if phase is None else phase,
            pattern=pattern,
            mode_time_s=mode_time_s,
            governor_active=governor_active,
            alarm_accel=alarm_accel,
            alarm_brake=alarm_brake,
            accel_ff_pct=accel_ff_pct,
            brake_ff_pct=brake_ff_pct,
            monitor_accel=monitor_accel,
            monitor_brake=monitor_brake,
            cycle_ms=cycle_ms,
            candidate=candidate,
        )
        self.samples.append(sample)
        self._plot.push(_plot_sample(sample))
        return sample

    def start_sampler(self) -> None:
        if self._sampler is None and not self._closed:
            self._sampler_stop = asyncio.Event()
            self._sampler = asyncio.get_running_loop().create_task(
                self._sample_loop(self._sampler_stop)
            )

    async def stop_sampler(self) -> None:
        """サンプラーを止める。読み取り中ならその 1 回が終わるのを待つ（cancel しない）。

        pymodbus の execute() は応答待ち中に CancelledError を受けると ModbusIOException に
        変えて投げ直すため、読み取り中に task.cancel() するとサンプラーが止まらず、この await が
        戻らなかった（実機で手順 2 の停車保持の後に停止）。途中で打ち切った要求の応答が遅れて
        届くのも避けたいので、停止フラグで読み取りの切れ目に抜けさせる。
        """
        task, self._sampler = self._sampler, None
        if task is None:
            return
        self._sampler_stop.set()
        await task

    async def close(self) -> Path | None:
        """サンプラーとグラフを止めて CSV/PNG を保存する（2 回目以降は何もしない）。"""
        if self._closed:
            return self.csv_path if self.samples else None
        self._closed = True
        if self._sampler is not None:
            await self.stop_sampler()
            # 最後の状態（停車保持など）を必ず 1 行残す。直前の段がすぐ終わるとサンプラーが
            # まだ 1 回も読んでいないことがあるため
            await self._read_and_record()
        await self._plot.aclose()
        if not self.samples:
            say("走行ログ: 記録が 0 行のため CSV・図は保存しません")
            return None
        write_csv(self.samples, self.csv_path)
        say(f"走行ログ CSV: {self.csv_path}（{len(self.samples)} 行）")
        if self._cfg.output.plot:
            self.figure_path = save_drive_figure(
                [_plot_sample(s) for s in self.samples],
                self.csv_path.with_suffix(".png"),
                title=self.title,
                max_speed_kmh=self._cfg.vehicle.max_speed_kmh,
                has_ref=any(s.data.ref_speed_kmh is not None for s in self.samples),
            )
            say(f"走行グラフ: {self.figure_path}")
        return self.csv_path

    async def _sample_loop(self, stop: asyncio.Event) -> None:
        loop = asyncio.get_running_loop()
        interval = self._cfg.output.csv_interval_s
        while not stop.is_set():
            started = loop.time()
            await self._read_and_record()
            wait_s = max(0.0, interval - (loop.time() - started))
            with contextlib.suppress(TimeoutError):
                await asyncio.wait_for(stop.wait(), timeout=wait_s)

    async def _read_and_record(self) -> DriveSample | None:
        """車速と両軸のまとめ読みを 1 行記録する。読めなければ記録を飛ばすだけ（判定は各段）。

        指令は各軸へ最後に送った目標位置（未指令なら原点 0）。
        """
        hw = self._hw
        started = time.perf_counter()
        try:
            speed = await hw.can.read_speed()
            monitor_accel, monitor_brake = await asyncio.gather(
                hw.accel.read_monitor(), hw.brake.read_monitor()
            )
        except Exception:
            return None
        cycle_ms = 1000.0 * (time.perf_counter() - started)
        accel_pos = hw.accel.last_command_pos or 0
        brake_pos = hw.brake.last_command_pos or 0
        data = DriveLogData(
            ref_speed_kmh=None,
            actual_speed_kmh=speed,
            accel_opening=pulse_to_opening(accel_pos),
            brake_opening=pulse_to_opening(brake_pos),
            accel_pos=accel_pos,
            brake_pos=brake_pos,
            accel_current=monitor_accel.current_ma,
            brake_current=monitor_brake.current_ma,
        )
        return self.record(
            data, monitor_accel=monitor_accel, monitor_brake=monitor_brake, cycle_ms=cycle_ms
        )


def mark(log: SessionLog | None, section: str, phase: str) -> None:
    """ログがあれば区間・段を切り替える（テストなどでログ無しに呼べるように）。"""
    if log is not None:
        log.mark(section, phase)


def _plot_sample(s: DriveSample) -> PlotSample:
    d = s.data
    return PlotSample(s.elapsed_s, d.ref_speed_kmh, d.actual_speed_kmh,
                      d.accel_opening, d.brake_opening)


def _fmt(value: float | None, digits: int = 3) -> str:
    return "" if value is None else f"{value:.{digits}f}"


def _fmt_bool(value: bool | None) -> str:
    return "" if value is None else ("1" if value else "0")


def _mm(pulse: int) -> str:
    return f"{pulse / PULSE_PER_MM:.2f}"


def _actual_cells(m: AxisMonitor | None) -> tuple[str, str]:
    if m is None:
        return "", ""
    return f"{pulse_to_opening(m.position_pulse):.3f}", _mm(m.position_pulse)


def _status_cells(m: AxisMonitor | None) -> list[str]:
    if m is None:
        return ["", "", "", ""]
    return [_fmt_bool(m.servo_on), _fmt_bool(m.moving), _fmt_bool(m.pos_done), str(m.alarm_code)]


def write_csv(samples: Sequence[DriveSample], path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        for s in samples:
            d = s.data
            deviation = None if d.ref_speed_kmh is None else d.actual_speed_kmh - d.ref_speed_kmh
            accel_actual_pct, accel_actual_mm = _actual_cells(s.monitor_accel)
            brake_actual_pct, brake_actual_mm = _actual_cells(s.monitor_brake)
            writer.writerow([
                to_jst_naive(s.timestamp).isoformat(sep=" "),
                f"{s.elapsed_s:.3f}",
                _fmt(s.mode_time_s),
                _fmt(s.cycle_ms, 1),
                _fmt(d.ref_speed_kmh),
                f"{d.actual_speed_kmh:.3f}",
                _fmt(deviation),
                _fmt(s.accel_ff_pct),
                _fmt(s.brake_ff_pct),
                f"{d.accel_opening:.3f}",
                f"{d.brake_opening:.3f}",
                _mm(d.accel_pos),
                _mm(d.brake_pos),
                accel_actual_pct,
                brake_actual_pct,
                accel_actual_mm,
                brake_actual_mm,
                f"{d.accel_current:.1f}",
                f"{d.brake_current:.1f}",
                s.section,
                s.candidate,
                s.pattern,
                s.phase,
                _fmt_bool(s.governor_active),
                _fmt_bool(s.alarm_accel),
                _fmt_bool(s.alarm_brake),
                *_status_cells(s.monitor_accel),
                *_status_cells(s.monitor_brake),
            ])


# ─────────────────────────────────────────────────────────────────────
# CSV の読み取り（新形式 A7〜 と旧形式 〜A6 の両方）
# ─────────────────────────────────────────────────────────────────────


def _num(text: str | None) -> float | None:
    return float(text) if text not in (None, "") else None


def cmd_opening(row: Mapping[str, str], axis: str) -> float:
    """最終指令の開度 [%]。新形式は `{axis}_cmd_pct`、旧形式は `{axis}_opening`。"""
    key = f"{axis}_cmd_pct"
    return float(row[key] if key in row else row[f"{axis}_opening"])


def cmd_pulse(row: Mapping[str, str], axis: str) -> int:
    """最終指令の位置 [pulse]。新形式は `{axis}_cmd_mm` × 100、旧形式は `{axis}_pos`。"""
    key = f"{axis}_cmd_mm"
    if key in row:
        return round(float(row[key]) * PULSE_PER_MM)
    return int(row[f"{axis}_pos"])


def actual_opening(row: Mapping[str, str], axis: str) -> float | None:
    """実開度 [%]（新形式だけ。旧形式・読めなかった行は None）。"""
    return _num(row.get(f"{axis}_actual_pct"))


def ff_effort(row: Mapping[str, str]) -> float | None:
    """FF の出力（+: アクセル / −: ブレーキ）。新形式はペダル別の 2 列から組み立てる。"""
    if "accel_ff_pct" in row:
        accel, brake = _num(row["accel_ff_pct"]), _num(row.get("brake_ff_pct"))
        if accel is None and brake is None:
            return None
        return (accel or 0.0) - (brake or 0.0)
    return _num(row.get("ff_effort_pct"))


def pid_effort(row: Mapping[str, str]) -> float | None:
    """PID の出力。旧形式の `pid_effort_pct` だけ（新形式は列を持たないので None）。"""
    return _num(row.get("pid_effort_pct"))


def total_effort(row: Mapping[str, str]) -> float | None:
    """合成の出力。旧形式の `effort_pct` だけ（新形式は列を持たないので None）。"""
    return _num(row.get("effort_pct"))


def read_drive_logs(path: Path, *, label: str = "cmd") -> list[DriveLog]:
    """CSV → 本番 model_training が受け取る DriveLog の列（パターン走行の行だけ）。

    `section` 列があれば PATTERN_DRIVE の行だけを読む（走行前チェック・探索・緩減速を学習に
    混ぜない）。
    `section` 列の無い旧形式の CSV は全行を読む。
    時刻は elapsed_s から組み立てる（ミリ秒丸めの timestamp 列より周期推定が正確なため）。

    `label`:
        "cmd"    （既定）学習ラベルは最終指令の開度（`cmd_opening`）。
        "actual" 学習ラベルは PNOW から換算した実開度（`actual_opening`、A7 の新形式だけ）。
                 実開度が読めなかった行（旧形式の CSV・欠測）は除外する。
    """
    if label not in ("cmd", "actual"):
        raise ValueError(f"label は 'cmd' か 'actual' のみです: {label!r}")
    origin = datetime(2000, 1, 1, tzinfo=UTC)
    logs: list[DriveLog] = []
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if "section" in row and row["section"] != SECTION_PATTERN_DRIVE:
                continue
            if label == "actual":
                accel_label = actual_opening(row, "accel")
                brake_label = actual_opening(row, "brake")
                if accel_label is None or brake_label is None:
                    continue
            else:
                accel_label = cmd_opening(row, "accel")
                brake_label = cmd_opening(row, "brake")
            logs.append(
                DriveLog(
                    id=len(logs),
                    session_id=path.stem,
                    timestamp=origin + timedelta(seconds=float(row["elapsed_s"])),
                    ref_speed_kmh=float(row["ref_speed_kmh"]) if row["ref_speed_kmh"] else None,
                    actual_speed_kmh=float(row["actual_speed_kmh"]),
                    accel_opening=accel_label,
                    brake_opening=brake_label,
                    accel_pos=cmd_pulse(row, "accel"),
                    brake_pos=cmd_pulse(row, "brake"),
                    accel_current=float(row["accel_current"]),
                    brake_current=float(row["brake_current"]),
                )
            )
    return logs
