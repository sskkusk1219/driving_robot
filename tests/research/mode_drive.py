"""手順 3: 走行モード管理の WLTP を FF だけで走り、車速追従を記録する（手順 5/7/9 で PID を足す）。

    基準車速 → FF（手順 2 のモデル）──┐
                                      ＋ → effort → 調停（符号で振り分け）→ アクセル/ブレーキ → 車両
      PID（手順 5 以降。手順 3 は 0）──┘                                                      │
       ↑                                                                                      │
       └──────────────────────────── 車速（CAN） ───────────────────────────────────────────────┘

引用元（アルゴリズムを移植。ループ自体は本番クラスを実行しない — 遵守事項「/src を実行しない」）:
    src/domain/control/drive_loop.py   … 1 サイクルの順序（基準車速の線形補間 → 先読み/過去の
                                          基準車速 → CAN 車速 → FF → 調停 → 変化した軸だけ位置指令
                                          → 電流 → 過電流・逸脱判定 → 100ms ごとのログ）、
                                          軸指令の smooth_over_s
    src/domain/control/feedforward.py  … FeedforwardController.predict_effort（手順 2 のモデルの
                                          推論。model_training と同じくドメインの関数として呼ぶ）。
                                          レジーム合成だけ tests/research/ff_candidate.py の
                                          CandidateFeedforward（C1）で差し替えている
    tests/research/pattern_loop.py     … 減速G ガバナー（0.4s の傾きで判定・頭打ち → 2% ずつ下げる）
    src/infra/mode_repository.py       … driving_modes.reference_speed の読み方

本番の自動走行との違い（第1段階として極端に単純にしたところ）:
    - プラン・トリム・フェーズ権限・ゲインスケジューリング・最小実効ブレーキ・FB 用ローパスは
      持たない。
    - 調停は effort の符号でアクセル/ブレーキに振り分け、開度上限でクランプするだけ。不感帯補償・
      レートリミット・ヒステリシス・再踏込ディレイ・解放レートは持たない（arbiter.enable_* が
      true なら手順 3 は開始しない）。
    - 一時停止・WebSocket・DB ログは持たない。ログは SessionLog（CSV/PNG）に 0.1s 刻みで残す。
    - 周期は固定の asyncio ループ。1 周期以上遅れたら次の周期から数え直し、回数をレポートに出す。
    - 減速G ガバナー（安全網）を持つ。本番の自動走行には無いので、作動した時間をレポートに出す。
"""

from __future__ import annotations

import asyncio
import bisect
import json
import math
import time
from collections import deque
from dataclasses import dataclass, field

from src.domain.control.conversions import G_TO_KMHS, VEHICLE_STOP_SPEED_KMH
from src.models.drive_log import DriveLogData
from src.models.driving_mode import DrivingMode, SpeedPoint
from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.config import ConfigError, ResearchConfig
from tests.research.drive_log import (
    SECTION_DECEL_TO_STOP,
    SECTION_MODE_DRIVE,
    DriveSample,
    SessionLog,
)
from tests.research.ff_candidate import CandidateFeedforward, make_candidate
from tests.research.ff_params import research_ff_params
from tests.research.hardware import ActuatorProtocol, DriveError, ResearchHardware
from tests.research.pattern_drive import _overcurrent_limit_ma, _release_pedals
from tests.research.stop_decel import PHASE_APPROACH, PHASE_STOP_HOLD, decelerate_to_stop
from tests.research.term import drive_status_line, say
from tests.research.vehicle import build_vehicle_profile, feedforward_params, opening_to_pulse

# 本番 drive_loop と同じ: 1 周期のうち軸の移動にかける割合と、書き込み直後の読み取りまでの待ち
AXIS_SMOOTH_DUTY = 0.8
AXIS_PRE_READ_DELAY_S = 0.002
# 減速G ガバナー（pattern_loop.PatternLoopConfig と同じ値）
GOVERNOR_WINDOW_S = 0.4  # 減速度を出す車速の傾きの窓 [s]
GOVERNOR_LIMIT_FRAC = 0.98  # vehicle.max_decel_g のこの割合で作動


# CSV の phase 列（その周期に動かしたペダル）
PHASE_ACCEL = "ACCEL"
PHASE_BRAKE = "BRAKE"
PHASE_BRAKE_GOVERNED = "BRAKE_GOV"  # 減速G ガバナーでブレーキを頭打ちにした周期
PHASE_COAST = "COAST"


# ─────────────────────────────────────────────────────────────────────
# 準備（走行前チェックより前に行う。失敗しても HW には触らない）
# ─────────────────────────────────────────────────────────────────────


@dataclass
class ModeDriveSetup:
    mode: DrivingMode
    ff: CandidateFeedforward


async def load_mode(cfg: ResearchConfig, name: str) -> DrivingMode:
    """走行モード管理（PostgreSQL の driving_modes）からモードを名前で読む。"""
    import asyncpg  # noqa: PLC0415 - DB を使わないテスト・手順で import を要求しない

    try:
        conn = await asyncpg.connect(cfg.hardware.database_url)
    except Exception as exc:
        raise ConfigError(
            f"走行モードの DB に接続できません（{cfg.hardware.database_url}、"
            f"{type(exc).__name__}: {exc}）"
        ) from exc
    try:
        row = await conn.fetchrow(
            "SELECT id, name, description, reference_speed, total_duration, max_speed, "
            "created_at, is_system FROM driving_modes WHERE name = $1",
            name,
        )
    finally:
        await conn.close()
    if row is None:
        raise ConfigError(f"走行モード {name!r} が driving_modes にありません（modes セクション）")
    points = [
        SpeedPoint(time_s=float(p["time_s"]), speed_kmh=float(p["speed_kmh"]))
        for p in json.loads(row["reference_speed"])
    ]
    if len(points) < 2:
        raise ConfigError(f"走行モード {name!r} の基準車速が 2 点未満です")
    return DrivingMode(
        id=str(row["id"]),
        name=row["name"],
        description=row["description"],
        reference_speed=points,
        total_duration=float(row["total_duration"]),
        max_speed=float(row["max_speed"]),
        created_at=row["created_at"],
        is_system=bool(row["is_system"]),
    )


def load_feedforward(cfg: ResearchConfig) -> CandidateFeedforward:
    """手順 2 で作った FF モデルと物理定数（config_testVehicle.yaml）を読み込む。"""
    ff_cfg = cfg.feedforward
    if not ff_cfg.is_model_trained:
        raise ConfigError(
            f"FF モデルがありません: {ff_cfg.model_path}（手順 2 で作成してください）"
        )
    ff = make_candidate(ff_cfg.candidate)  # V1 案スイッチ（C1〜C6。既定 C1）
    ff.set_params(feedforward_params(cfg))
    ff.set_research_params(research_ff_params(cfg))
    try:
        ff.load_model(ff_cfg.model_path)
    except (OSError, ValueError) as exc:
        raise ConfigError(f"FF モデルを読み込めません: {ff_cfg.model_path}（{exc}）") from exc
    # 段3: reach_horizons_s はモデルの先読みホライズンの部分集合でなければならない
    # （predict_effort が future_speeds の添字を引くため）。走行前に落とす
    missing = [h for h in ff_cfg.reach_horizons_s if h not in ff.horizons]
    if missing:
        raise ConfigError(
            f"feedforward.reach_horizons_s の {missing} はモデルの先読みホライズン "
            f"{list(ff.horizons)} に含まれていません"
        )
    return ff


def require_simple_arbiter(cfg: ResearchConfig) -> None:
    """第1段階（調停オプションすべて無効）でしか走らせない。"""
    enabled = [
        name
        for name, on in (
            ("enable_deadband_compensation", cfg.arbiter.enable_deadband_compensation),
            ("enable_rate_limit", cfg.arbiter.enable_rate_limit),
            ("enable_hysteresis", cfg.arbiter.enable_hysteresis),
        )
        if on
    ]
    if enabled:
        raise ConfigError(
            "モード走行の調停は第1段階（符号で振り分けるだけ）のみ実装しています。"
            f"arbiter.{' / arbiter.'.join(enabled)} を false にしてください"
        )


async def prepare_mode_drive(cfg: ResearchConfig, mode_name: str) -> ModeDriveSetup:
    require_simple_arbiter(cfg)
    say(f"走行モードを読み込みます: {mode_name!r}（{cfg.hardware.database_url}）")
    mode = await load_mode(cfg, mode_name)
    say(f"  {len(mode.reference_speed)} 点・{mode.total_duration:.0f}s・"
        f"最高 {mode.max_speed:.1f} km/h")
    if mode.max_speed > cfg.vehicle.max_speed_kmh:
        raise ConfigError(
            f"モードの最高速 {mode.max_speed:.1f} km/h が vehicle.max_speed_kmh "
            f"{cfg.vehicle.max_speed_kmh:.1f} を超えています"
        )
    ff = load_feedforward(cfg)
    ff_cfg = cfg.feedforward
    say(f"FF 候補: {ff.candidate}　モデル: {ff_cfg.model_path}"
        f"（先読み {list(ff.horizons)}s・過去 {list(ff.past_horizons)}s）")
    say(f"  不感帯 アクセル {ff_cfg.accel_deadband_pct:.2f}% / "
        f"ブレーキ {ff_cfg.brake_deadband_pct:.2f}%・停車保持 {ff_cfg.stop_brake_opening_pct:.2f}%")
    return ModeDriveSetup(mode=mode, ff=ff)


# ─────────────────────────────────────────────────────────────────────
# 部品
# ─────────────────────────────────────────────────────────────────────


class ReferenceSpeed:
    """基準車速の線形補間（本番 DriveLoop._ref_speed_at と同じ。範囲外は端点値）。"""

    def __init__(self, mode: DrivingMode) -> None:
        self._times = [p.time_s for p in mode.reference_speed]
        self._speeds = [p.speed_kmh for p in mode.reference_speed]

    def at(self, t_s: float) -> float:
        times, speeds = self._times, self._speeds
        if not times:
            return 0.0
        if t_s <= times[0]:
            return speeds[0]
        if t_s >= times[-1]:
            return speeds[-1]
        i = bisect.bisect_right(times, t_s) - 1
        dt = times[i + 1] - times[i]
        if dt <= 0.0:
            return speeds[i + 1]
        return speeds[i] + (t_s - times[i]) / dt * (speeds[i + 1] - speeds[i])


class ActualSpeedHistory:
    """実測車速の履歴（V2: C5 用）。`record` で追記し、`at` で線形補間する。

    過去ホライズンぶんだけ保持すればよいので、それより古い点は捨てる。履歴が浅い
    （走行開始直後）ときは直近値でフォールバックする（実測 ≈ 基準として実害は小さい）。
    """

    def __init__(self, max_span_s: float) -> None:
        self._max_span_s = max_span_s
        self._points: deque[tuple[float, float]] = deque()

    def record(self, t: float, speed: float) -> None:
        self._points.append((t, speed))
        while len(self._points) >= 2 and t - self._points[0][0] > self._max_span_s:
            self._points.popleft()

    def at(self, t: float) -> float:
        pts = self._points
        if not pts:
            return 0.0
        if t <= pts[0][0]:
            return pts[0][1]
        if t >= pts[-1][0]:
            return pts[-1][1]
        prev = pts[0]
        for point in pts:
            t1, v1 = point
            t0, v0 = prev
            if t0 <= t <= t1:
                return v1 if t1 <= t0 else v0 + (t - t0) / (t1 - t0) * (v1 - v0)
            prev = point
        return pts[-1][1]


def standby_openings(cfg: ResearchConfig) -> tuple[float, float]:
    """使っていないペダルの待機開度 (アクセル, ブレーキ) [%]。無効なら (0, 0)。"""
    md = cfg.mode_drive
    if not md.pedal_standby:
        return 0.0, 0.0
    ff = cfg.feedforward
    return (
        max(0.0, ff.accel_deadband_pct - md.standby_margin_pct),
        max(0.0, ff.brake_deadband_pct - md.standby_margin_pct),
    )


def standby_label(cfg: ResearchConfig) -> str:
    """表示・レポート用の待機位置の説明。"""
    if not cfg.mode_drive.pedal_standby:
        return "なし（0%）"
    accel, brake = standby_openings(cfg)
    return f"{accel:.2f}% / {brake:.2f}%（不感帯 − {cfg.mode_drive.standby_margin_pct:.2f}%）"


def split_effort(effort: float, max_accel_pct: float, max_brake_pct: float) -> tuple[float, float]:
    """調停（第1段階）: effort の符号でアクセル/ブレーキに振り分け、開度上限でクランプする。"""
    if effort > 0.0:
        return min(effort, max_accel_pct), 0.0
    if effort < 0.0:
        return 0.0, min(-effort, max_brake_pct)
    return 0.0, 0.0


@dataclass
class DecelGovernor:
    """減速G ガバナー（安全網）。pattern_loop.PatternLoop._update_governor と同じ下げ方の規則。

    ただし「減速が上限を十分下回ったら頭打ちを戻す」解除は pattern_loop だけに入れた
    （2026-09-13 A6）。こちらの「下限に張り付いて戻らない」穴 B は別課題 2 として残している。

    直近 GOVERNOR_WINDOW_S の車速の傾きで減速度を出し、max_decel_g × GOVERNOR_LIMIT_FRAC 以上なら
    ブレーキ開度を前周期の値で頭打ちにする。超えている間は周期ごとに reduce_step_pct ずつ下げる。
    指令がブレーキでなくなるか、頭打ちより浅くなったら解除する。
    """

    max_decel_g: float
    reduce_step_pct: float
    enabled: bool = True
    decel_kmhs: float = 0.0  # 直近の減速度（正が減速）
    _hist: deque[tuple[float, float]] = field(default_factory=deque)
    _cap: float | None = None
    _last_brake: float = 0.0

    @property
    def limit_kmhs(self) -> float:
        return self.max_decel_g * G_TO_KMHS * GOVERNOR_LIMIT_FRAC

    def apply(self, now: float, speed_kmh: float, brake_pct: float) -> tuple[float, bool]:
        """ブレーキ指令 [%] に頭打ちをかけ、(かけた後の開度, 頭打ち中か) を返す。"""
        hist = self._hist
        hist.append((now, speed_kmh))
        while len(hist) >= 2 and now - hist[0][0] > GOVERNOR_WINDOW_S:
            hist.popleft()
        (t0, v0), (t1, v1) = hist[0], hist[-1]
        self.decel_kmhs = (v0 - v1) / (t1 - t0) if t1 > t0 else 0.0

        if not self.enabled or brake_pct <= 0.0:
            self._cap = None
            self._last_brake = max(0.0, brake_pct)
            return brake_pct, False
        if self._cap is not None and brake_pct < self._cap:
            self._cap = None  # FF 自身が頭打ちより浅くした
        if self.decel_kmhs >= self.limit_kmhs:
            if self._cap is None:
                self._cap = self._last_brake
            else:
                self._cap = max(0.0, self._cap - self.reduce_step_pct)
        applied = brake_pct if self._cap is None else min(brake_pct, self._cap)
        self._last_brake = applied
        return applied, self._cap is not None


# ─────────────────────────────────────────────────────────────────────
# 走行
# ─────────────────────────────────────────────────────────────────────


@dataclass
class ModeDriveResult:
    mode_name: str
    mode_duration_s: float  # 走らせる予定だった長さ（--limit-s で縮めたらその値）
    run_duration_s: float  # 実際にモードを走った長さ
    completed: bool
    abort_reason: str = ""
    cycles: int = 0
    overruns: int = 0  # 1 周期以上遅れた回数
    governor_cycles: int = 0
    samples: list[DriveSample] = field(default_factory=list)  # MODE_DRIVE の行（0.1s 刻み）


class _ModeRun:
    """1 本のモード走行の状態。run() が 50ms 周期でモード終わりまで回す。"""

    def __init__(
        self,
        hw: ResearchHardware,
        cfg: ResearchConfig,
        setup: ModeDriveSetup,
        log: SessionLog,
        duration_s: float,
    ) -> None:
        self.hw = hw
        self.cfg = cfg
        self.ff = setup.ff
        self.mode = setup.mode
        self.ref = ReferenceSpeed(setup.mode)
        self.log = log
        self.duration_s = duration_s
        self.interval_s = cfg.control.loop_interval_s
        self.overcurrent_ma = _overcurrent_limit_ma(hw)
        self.governor = DecelGovernor(
            max_decel_g=cfg.vehicle.max_decel_g,
            reduce_step_pct=cfg.mode_drive.governor_reduce_step_pct,
            enabled=cfg.mode_drive.decel_governor,
        )
        self.accel_standby, self.brake_standby = standby_openings(cfg)
        # V2: C5 だけが使う実測車速の履歴（過去ホライズンの最大値ぶんだけ保持すればよい）
        self.actual_history = ActualSpeedHistory(max(self.ff.past_horizons, default=1.0) + 0.5)
        self.cycles = 0
        self.overruns = 0
        self.governor_cycles = 0
        self.cycle_ms: list[float] = []  # 全周期の処理時間（A7 の関門: 周期を超えないか）
        self.samples: list[DriveSample] = []
        self._accel_cmd: int | None = None
        self._brake_cmd: int | None = None
        self._deviation_since: float | None = None
        self.max_abs_dev = 0.0
        self._next_print = 0.0
        self._segment = ""
        # アクチュエータ脱落の安全網（アラーム確認・電流ゼロ継続。axis_safety.py）
        self.safety = AxisSafetyNet(self.interval_s)

    async def run(self) -> float:
        """モード終わり（または duration_s）まで回し、走った秒数を返す。異常は DriveError。"""
        loop = asyncio.get_running_loop()
        started = loop.time()
        next_tick = started
        while True:
            t = loop.time() - started
            if t >= self.duration_s:
                return t
            await self._cycle(t, loop.time())
            self.cycles += 1
            next_tick += self.interval_s
            now = loop.time()
            if now - next_tick > self.interval_s:
                self.overruns += 1  # 1 周期以上遅れた: まとめて取り返さず次の周期から数え直す
                next_tick = now
            await asyncio.sleep(max(0.0, next_tick - now))

    def _ff_inputs(
        self, t: float, ref: float, speed: float
    ) -> tuple[float, list[float], list[float]]:
        """FF へ渡す v0/future/past（V2）。

        C1〜C3 は今まで通り基準車速だけ。C4 は動作点だけ実車速にし、先読み・過去の変化量は
        基準車速のまま（偏差そのものは入れない）。C5 は過去を実測履歴、先読みを基準の絶対値にする
        （`dv = ref(t+h) − v_実測(t)` が自然に出る。KAIZEN 報告書 3 章 結論 8〜10）。
        ただし基準が停車レジームのときは C4・C5 も基準車速だけを返す（下記参照）。
        """
        ff = self.ff
        future = [self.ref.at(t + h) for h in ff.horizons]
        # 停車判定は候補によらず基準車速で行う（C1 に倣う）。v0 を実測にする C4・C5 は
        # 実車速が VEHICLE_STOP_SPEED_KMH を下回らない限り停車保持に入らないため
        # （A8 報告 5 章 #2）。判定の形は predict_effort の停車レジーム（ff_candidate.py）と
        # 同じく最短ホライズンのみ（3 秒先まで見ると発進の 3 秒前に保持ブレーキが解除され、
        # クリープで動き出す。src/domain/control/feedforward.py:211-223 のレビュー指摘 #5）。
        ref_is_stopped = (
            ref <= VEHICLE_STOP_SPEED_KMH and future and future[0] <= VEHICLE_STOP_SPEED_KMH
        )
        if not ff.uses_actual_speed or ref_is_stopped:
            past = [self.ref.at(t - h) for h in ff.past_horizons]
            return ref, future, past
        if ff.candidate == "C5":
            past = [self.actual_history.at(t - h) for h in ff.past_horizons]
        else:  # C4
            future = [speed + (self.ref.at(t + h) - ref) for h in ff.horizons]
            past = [speed + (self.ref.at(t - h) - ref) for h in ff.past_horizons]
        return speed, future, past

    async def _cycle(self, t: float, now: float) -> None:
        cfg, hw = self.cfg, self.hw
        cycle_started = time.perf_counter()
        try:
            speed = await hw.can.read_speed()
        except Exception as exc:
            raise DriveError(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）") from exc
        self.actual_history.record(t, speed)

        ref = self.ref.at(t)
        v0, future, past = self._ff_inputs(t, ref, speed)
        ff_effort = self.ff.predict_effort(v0, future, past)
        pid_effort = 0.0  # 手順 5 以降でここに PID の出力を足す
        effort = ff_effort + pid_effort
        # ログの「指示開度 FF」列（FF の出力をペダル別に分けた値。ガバナー・待機位置の前）
        accel_ff, brake_ff = split_effort(
            ff_effort, cfg.vehicle.max_accel_opening_pct, cfg.vehicle.max_brake_opening_pct
        )
        accel, brake = split_effort(
            effort, cfg.vehicle.max_accel_opening_pct, cfg.vehicle.max_brake_opening_pct
        )
        brake, governed = self.governor.apply(now, speed, brake)
        if governed:
            self.governor_cycles += 1
        phase = (
            PHASE_ACCEL if accel > 0.0
            else PHASE_BRAKE_GOVERNED if governed
            else PHASE_BRAKE if brake > 0.0
            else PHASE_COAST
        )
        # 使っていないペダルも不感帯の手前で待たせる（phase は待機位置を掛ける前の選択で決める）
        accel, brake = max(accel, self.accel_standby), max(brake, self.brake_standby)
        accel_pos, brake_pos = opening_to_pulse(accel), opening_to_pulse(brake)

        try:
            monitor_accel, monitor_brake = await asyncio.gather(
                self.drive_axis(hw.accel, accel_pos, is_accel=True),
                self.drive_axis(hw.brake, brake_pos, is_accel=False),
            )
        except Exception as exc:
            raise DriveError(
                f"アクチュエータ通信に失敗しました（{type(exc).__name__}: {exc}）"
            ) from exc
        cycle_ms = 1000.0 * (time.perf_counter() - cycle_started)
        self.cycle_ms.append(cycle_ms)
        accel_current, brake_current = monitor_accel.current_ma, monitor_brake.current_ma
        alarm_accel, alarm_brake = await self.safety.poll_alarms(self.cycles, hw.accel, hw.brake)
        self._check_safety(
            t, now, ref, speed, accel_current, brake_current,
            accel_pos, brake_pos, alarm_accel, alarm_brake,
        )

        deviation = speed - ref
        self.max_abs_dev = max(self.max_abs_dev, abs(deviation))
        segment = cfg.modes.segment_at(t)
        if self.cycles % cfg.control.log_every_n_cycles == 0:
            data = DriveLogData(
                ref_speed_kmh=ref,
                actual_speed_kmh=speed,
                accel_opening=accel,
                brake_opening=brake,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
                plan_effort_pct=ff_effort,
                trim_effort_pct=pid_effort,
                applied_effort_pct=effort,
                phase=phase,
            )
            self.samples.append(
                self.log.record(
                    data, section=SECTION_MODE_DRIVE, phase=phase, pattern=segment,
                    mode_time_s=t, governor_active=governed,
                    alarm_accel=alarm_accel, alarm_brake=alarm_brake,
                    accel_ff_pct=accel_ff, brake_ff_pct=brake_ff,
                    monitor_accel=monitor_accel, monitor_brake=monitor_brake, cycle_ms=cycle_ms,
                    candidate=self.ff.candidate,
                )
            )
        if segment != self._segment:
            self._segment = segment
            say(f"── 区間 {segment} ──")
        if t >= self._next_print:
            interval = cfg.output.print_interval_s  # 周期の端数で表示間隔が伸びないよう格子で数える
            self._next_print = (math.floor(t / interval) + 1) * interval
            gov = "  減速Gガバナー作動" if governed else ""
            say(drive_status_line(
                t_s=t, ref_kmh=ref, actual_kmh=speed, kp=0.0, ki=0.0, kd=0.0,
                accel_pct=accel, brake_pct=brake,
                extra=(f"FF={ff_effort:+6.1f}% {segment} "
                       f"max|偏差|={self.max_abs_dev:.2f} "
                       f"{100.0 * t / self.duration_s:4.1f}%{gov}"),
            ))

    async def drive_axis(
        self, axis: ActuatorProtocol, pos: int, *, is_accel: bool
    ) -> AxisMonitor:
        """位置が変わった軸だけ指令し（本番 _drive_accel_axis と同じ）、まとめ読みする。

        A7: 以前は電流（CNOW）だけ読んでいた。0x9000 から 14 レジスタのまとめ読みで
        実位置・ステータスも同じ 1 回で取る。
        """
        last = self._accel_cmd if is_accel else self._brake_cmd
        if pos != last:
            await axis.move_to_position(pos, smooth_over_s=self.interval_s * AXIS_SMOOTH_DUTY)
            if is_accel:
                self._accel_cmd = pos
            else:
                self._brake_cmd = pos
            await asyncio.sleep(AXIS_PRE_READ_DELAY_S)
        return await axis.read_monitor()

    def _check_safety(
        self,
        t: float,
        now: float,
        ref: float,
        speed: float,
        accel_current: float,
        brake_current: float,
        accel_pos: int,
        brake_pos: int,
        alarm_accel: bool | None,
        alarm_brake: bool | None,
    ) -> None:
        v = self.cfg.vehicle
        for label, current in (("アクセル", accel_current), ("ブレーキ", brake_current)):
            if current > self.overcurrent_ma:
                raise DriveError(
                    f"{label}軸が過電流です"
                    f"（{current:.0f} mA > {self.overcurrent_ma:.0f} mA、t={t:.1f}s）"
                )
        # アラーム・電流ゼロの継続（アラームが出ないサーボ脱落を拾う安全網。axis_safety.py）
        reason = self.safety.check(
            t=t, accel_pos=accel_pos, brake_pos=brake_pos,
            accel_current=accel_current, brake_current=brake_current,
            alarm_accel=alarm_accel, alarm_brake=alarm_brake,
        )
        if reason is not None:
            raise DriveError(reason)
        if speed > v.max_speed_kmh:
            raise DriveError(
                f"車速 {speed:.1f} km/h が最高速 {v.max_speed_kmh:.1f} km/h を超えました"
            )
        if abs(speed - ref) > v.stop_deviation_threshold_kmh:
            if self._deviation_since is None:
                self._deviation_since = now
            if now - self._deviation_since >= v.stop_deviation_duration_s:
                raise DriveError(
                    f"偏差 {speed - ref:+.2f} km/h が {v.stop_deviation_threshold_kmh:g} km/h を"
                    f" {v.stop_deviation_duration_s:g}s 超え続けました（t={t:.1f}s）"
                )
        else:
            self._deviation_since = None


def cycle_time_text(cycle_ms: list[float], interval_s: float) -> str:
    """周期の処理時間 [ms] の平均 / p95 / 最大と、周期を超えた回数（A7 の関門）。"""
    ordered = sorted(cycle_ms)
    p95 = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
    period_ms = 1000.0 * interval_s
    over = sum(1 for ms in cycle_ms if ms > period_ms)
    return (f"平均 {sum(cycle_ms) / len(cycle_ms):.1f} / p95 {p95:.1f} / 最大 {ordered[-1]:.1f} ms"
            f"（周期 {period_ms:.0f}ms を超えた {over} 周期）")


async def run_mode_drive(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    setup: ModeDriveSetup,
    *,
    log: SessionLog,
    limit_s: float | None = None,
) -> ModeDriveResult:
    """停車保持の状態からモードを走り、停車保持で終える。

    異常（CAN・通信・過電流・最高速超え・逸脱）は例外にせず、ペダルを離してから
    abort_reason 付きの結果を返す（途中までのレポートを作れるように）。
    `limit_s` はモードの先頭だけ走る動作確認用。
    """
    mode = setup.mode
    duration = mode.total_duration if limit_s is None else min(limit_s, mode.total_duration)
    limited = duration < mode.total_duration
    say(f"モード走行: {mode.name}（{duration:.0f}s"
        f"{f' ／ 動作確認のため先頭 {duration:.0f}s で打ち切り' if limited else ''}）")
    say("制御構成: FF のみ（Kp=Ki=Kd=0）→ 調停は effort の符号で振り分け（不感帯補償・"
        "レートリミット・ヒステリシスなし）")
    gov = cfg.mode_drive
    say(f"減速G ガバナー（安全網）: {'有効' if gov.decel_governor else '無効'}"
        f"（{cfg.vehicle.max_decel_g:g}G × {GOVERNOR_LIMIT_FRAC:g} 以上で頭打ち）")
    say(f"ペダルの待機位置: {standby_label(cfg)}")
    if hw.is_real:
        say(f"*** 実機モード: 車両が最高 {mode.max_speed:.0f} km/h まで走ります。"
            "シャシダイナモ上で実施してください ***")

    run = _ModeRun(hw, cfg, setup, log, duration)
    await log.stop_sampler()  # 50ms の制御ループと Modbus を取り合わない（記録はループから）
    log.mark(SECTION_MODE_DRIVE, "")
    loop = asyncio.get_running_loop()
    started = loop.time()
    abort_reason = ""
    try:
        run_s = await run.run()
    except DriveError as exc:
        run_s = loop.time() - started
        abort_reason = str(exc)
        say(f"モード走行を中断しました: {exc}")
        await _release_pedals(hw)
    result = ModeDriveResult(
        mode_name=mode.name,
        mode_duration_s=duration,
        run_duration_s=run_s,
        completed=not abort_reason,
        abort_reason=abort_reason,
        cycles=run.cycles,
        overruns=run.overruns,
        governor_cycles=run.governor_cycles,
        samples=run.samples,
    )
    say(f"モード走行 {'完了' if result.completed else '中断'}: {run_s:.1f}s・{run.cycles} 周期"
        f"（1 周期以上の遅れ {run.overruns} 回・減速G ガバナー作動 {run.governor_cycles} 周期）"
        f"・最大 |偏差| {run.max_abs_dev:.2f} km/h")
    if run.cycle_ms:
        say(f"  1 周期の処理時間: {cycle_time_text(run.cycle_ms, run.interval_s)}")
    if result.completed:
        await _finish_stopped(hw, cfg, log, run)
    return result


async def _finish_stopped(
    hw: ResearchHardware, cfg: ResearchConfig, log: SessionLog, run: _ModeRun
) -> None:
    """モード終わりで停車保持にする。

    止まっていなければ緩減速で止める（--limit-s で打ち切ったとき）。
    """
    try:
        speed = await hw.can.read_speed()
    except Exception as exc:
        await _release_pedals(hw)
        raise DriveError(
            f"モード終了時に CAN 車速を読めません（{type(exc).__name__}: {exc}）"
        ) from exc
    profile = build_vehicle_profile(cfg)
    if speed >= VEHICLE_STOP_SPEED_KMH:
        say(f"モード終了時の車速 {speed:.2f} km/h: 緩減速で停車させます …")
        log.mark(SECTION_DECEL_TO_STOP, PHASE_APPROACH)
        log.start_sampler()
        stop = await decelerate_to_stop(hw, cfg, profile, log=log)
        say(f"停車保持: ブレーキ {stop.hold_pct:.2f}%")
        return
    hold_pct = min(cfg.feedforward.stop_brake_opening_pct, cfg.vehicle.max_brake_opening_pct)
    log.mark(SECTION_DECEL_TO_STOP, PHASE_STOP_HOLD)
    await run.drive_axis(hw.accel, 0, is_accel=True)
    await run.drive_axis(hw.brake, opening_to_pulse(hold_pct), is_accel=False)
    log.start_sampler()
    say(f"停車しています（{speed:.2f} km/h）。停車保持: ブレーキ {hold_pct:.2f}%")
