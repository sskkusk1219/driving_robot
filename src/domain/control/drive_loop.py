"""50ms 制御ループ。FF+PID 制御・ペダル調停・安全チェック・KPI 計測・ログ記録を担う。"""

from __future__ import annotations

import asyncio
import bisect
import logging
from collections.abc import Awaitable, Callable
from typing import Protocol

from src.domain.control.feedforward import FeedforwardController
from src.domain.control.kpi_monitor import KPIMonitor
from src.domain.control.pedal_arbiter import PedalArbiter
from src.domain.control.pid import PIDController
from src.models.drive_log import DriveLogData
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile
from src.models.system_state import RealtimeSnapshot

_logger = logging.getLogger(__name__)

CONTROL_LOOP_INTERVAL_S: float = 0.05
LOG_EVERY_N_CYCLES: int = 2
# サイクルウォッチドッグ: 前サイクル未完了によるスキップがこの時間続いたら非常停止する。
# pymodbus のタイムアウト×リトライ（最大十数秒）を待つ間、ペダルが最終指令位置で
# 凍結したまま安全チェックが走らない時間を 1 秒に短縮する。
WEDGED_CYCLE_TIMEOUT_S: float = 1.0
# ログ書き込み保留タスクの上限。DB ストール時に 10 件/s で無制限に積み上がるのを防ぎ、
# 10 時間連続走行のメモリとイベントループ負荷を一定に保つ（走行継続をログより優先）。
MAX_PENDING_LOG_TASKS: int = 100


class ActuatorDriverProtocol(Protocol):
    async def move_to_position(self, pos: int) -> None: ...

    async def read_current(self) -> float: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


class SafetyCheckProtocol(Protocol):
    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...

    def check_deviation(self, ref: float, actual: float, duration: float) -> bool: ...


class LogWriterProtocol(Protocol):
    async def write_log(self, session_id: str, data: DriveLogData) -> None: ...


class DriveLoop:
    """50ms 制御ループを管理するドメインコンポーネント。

    start() でループを開始し、stop() または on_complete/on_emergency コールバックで停止する。
    asyncio.sleep を使わず call_later でスケジューリングすることでジッタを ±5ms 以内に抑制する。

    制御構成: FF（純粋フィードフォワード）と PID（誤差補正）を符号付き努力量として合成し、
    PedalArbiter がペダルへ写像する。アクセル・ブレーキの排他はこの写像の構造的性質であり、
    調停層の飽和フラグを PID の条件付き積分へ返してワインドアップを防ぐ。
    """

    _running: bool
    _started_at: float
    _cycle_count: int
    _deviation_start: float | None

    def __init__(
        self,
        ff_controller: FeedforwardController,
        pid: PIDController,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        profile: VehicleProfile,
        mode: DrivingMode,
        safety_check: SafetyCheckProtocol,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        log_writer: LogWriterProtocol | None = None,
        session_id: str | None = None,
        interval_s: float = CONTROL_LOOP_INTERVAL_S,
        log_every_n_cycles: int = LOG_EVERY_N_CYCLES,
    ) -> None:
        self._ff = ff_controller
        self._pid = pid
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._mode = mode
        self._safety_check = safety_check
        self._on_complete = on_complete
        self._on_emergency = on_emergency
        self._log_writer = log_writer
        self._session_id = session_id
        self._interval_s = interval_s
        self._log_every_n_cycles = log_every_n_cycles

        self._running = False
        self._started_at = 0.0
        self._cycle_count = 0
        self._deviation_start = None
        self._current_accel_opening: float = 0.0
        self._current_brake_opening: float = 0.0
        # FF+PID 合成後のペダル写像と振動抑制を担う調停器。プロファイル定数で構成する。
        self._arbiter = PedalArbiter(
            profile.feedforward_params,
            max_accel_opening=profile.max_accel_opening,
            max_brake_opening=profile.max_brake_opening,
        )
        self._kpi = KPIMonitor()
        # 調停の飽和フラグ。次サイクルの PID 条件付き積分（アンチワインドアップ）に渡す。
        self._saturated_high = False
        self._saturated_low = False
        # PID/調停の計測 dt 用。サイクルスキップ時に微分スパイクを起こさない。
        self._last_cycle_time: float | None = None
        # ウォッチドッグ: 前サイクル未完了による連続スキップ数
        self._consecutive_skips = 0
        self._log_backlog_active = False
        # bisect 用に基準速度の時刻列を前計算（_ref_speed_at はサイクル毎に複数回呼ばれる）
        self._ref_times: list[float] = [p.time_s for p in mode.reference_speed]
        # イベントループはタスクを弱参照でしか保持しないため、GC でサイクルタスクが
        # 実行途中に破棄されないよう強参照を保持する
        self._cycle_task: asyncio.Task[None] | None = None
        self._emergency_task: asyncio.Task[None] | None = None
        self._pending_log_tasks: set[asyncio.Task[None]] = set()
        # WS 配信用のサイクル計測値キャッシュ（走行中の追加 Modbus/CAN 読み取りを回避）
        self._last_snapshot: RealtimeSnapshot | None = None
        self._last_ref_speed: float | None = None

    def start(self) -> None:
        """制御ループを開始する。既に実行中の場合は何もしない。"""
        if self._running:
            return
        self._running = True
        self._pid.reset()
        self._arbiter.reset()
        self._kpi = KPIMonitor()
        self._saturated_high = False
        self._saturated_low = False
        self._last_cycle_time = None
        self._deviation_start = None
        self._cycle_count = 0
        self._consecutive_skips = 0
        loop = asyncio.get_running_loop()
        self._started_at = loop.time()
        loop.call_later(self._interval_s, self._schedule_next_cycle)

    def stop(self) -> None:
        """制御ループを停止する。進行中のサイクルはアクチュエータ書き込み前に中断される。"""
        self._running = False

    async def stop_and_join(self, timeout_s: float = 2.0) -> None:
        """停止し、進行中のサイクルタスクの完了を待つ。

        非常停止・正常停止で home_return を開始する前に必ず await すること。
        飛行中の位置指令（move_to_position の await 中）と原点復帰シーケンスが
        同一軸のバスでインターリーブし、非常停止中にペダル指令が再適用されるのを防ぐ
        （コードレビュー 2026-06-11 指摘 #6）。

        タイムアウト時はタスクをキャンセルする（バスが完全に固まっている場合、
        非常停止をこれ以上遅延させない）。
        """
        self.stop()
        task = self._cycle_task
        if task is None or task.done():
            return
        try:
            # shield: タイムアウトしてもサイクルタスク自体は cancel せず明示的に扱う
            await asyncio.wait_for(asyncio.shield(task), timeout_s)
        except TimeoutError:
            _logger.warning(
                "進行中の制御サイクルが %.1fs 以内に完了せずキャンセルします", timeout_s
            )
            task.cancel()
        except Exception:
            # サイクル内の例外は _on_cycle_done が回収・処理済み（ここでは無視してよい）
            pass

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def current_accel_opening(self) -> float:
        return self._current_accel_opening

    @property
    def current_brake_opening(self) -> float:
        return self._current_brake_opening

    @property
    def last_snapshot(self) -> RealtimeSnapshot | None:
        """直近サイクルの計測値。WS 配信が走行中の追加バス読み取りを避けるために使う。"""
        return self._last_snapshot

    @property
    def current_ref_speed(self) -> float | None:
        """直近サイクルの基準車速 [km/h]。未実行時は None。"""
        return self._last_ref_speed

    @property
    def kpi_summary(self) -> dict[str, float]:
        """走行中〜終了時点の KPI 集計（P95・最大偏差・符号反転率・ハード違反数）。"""
        return self._kpi.summary()

    def _schedule_next_cycle(self) -> None:
        if not self._running:
            return
        # 重複実行ガード: 前サイクルが未完了（バス遅延等）の場合は新サイクルを起動しない。
        # 並行サイクルは古い位置指令が新しい指令を上書きする危険があるためスキップする。
        if self._cycle_task is not None and not self._cycle_task.done():
            self._consecutive_skips += 1
            wedged_s = self._consecutive_skips * self._interval_s
            _logger.warning(
                "前回の制御サイクルが %.0fms 以内に完了せずスキップ", self._interval_s * 1000
            )
            # ウォッチドッグ: スキップが続く間は安全チェック（逸脱・過電流）が一切走らず
            # ペダルが最終指令位置で保持され続けるため、上限で非常停止に倒す。
            if wedged_s >= WEDGED_CYCLE_TIMEOUT_S:
                _logger.error(
                    "制御サイクルが %.1fs 以上完了していません: ウォッチドッグ非常停止します",
                    wedged_s,
                )
                self.stop()
                self._emergency_task = asyncio.ensure_future(self._on_emergency())
                self._emergency_task.add_done_callback(_log_emergency_error_callback)
                return
        else:
            self._consecutive_skips = 0
            self._cycle_task = asyncio.ensure_future(self._execute_one_cycle())
            self._cycle_task.add_done_callback(self._on_cycle_done)
        asyncio.get_running_loop().call_later(self._interval_s, self._schedule_next_cycle)

    def _on_cycle_done(self, task: asyncio.Task[None]) -> None:
        """サイクルタスクの未捕捉例外を回収する。黙殺すると安全チェック抜けが見えなくなる。"""
        if task.cancelled():
            return
        exc = task.exception()
        if exc is None:
            return
        _logger.error("制御サイクルで未捕捉例外: 緊急停止します", exc_info=exc)
        self.stop()
        self._emergency_task = asyncio.ensure_future(self._on_emergency())
        self._emergency_task.add_done_callback(_log_emergency_error_callback)

    async def _abort_emergency(self) -> None:
        """ループを停止して非常停止コールバックを呼ぶ。サイクル内の異常検知から使う。"""
        self.stop()
        await self._on_emergency()

    async def _execute_one_cycle(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        elapsed_s = loop.time() - self._started_at

        if elapsed_s >= self._mode.total_duration:
            self.stop()
            await self._on_complete()
            return

        ref_speed = self._ref_speed_at(elapsed_s)
        self._last_ref_speed = ref_speed
        future_speeds = [self._ref_speed_at(elapsed_s + h) for h in self._ff.horizons]

        try:
            actual_speed = await self._can_reader.read_speed()
        except Exception:
            _logger.exception("CAN 車速取得失敗: 緊急停止")
            await self._abort_emergency()
            return

        # 運転モデル未ロード（初回学習走行）では FF を 0 とし PID のみで基準を追従する。
        # 収集した連続ログから初回モデルを学習し、以降は FF+PID で精度を上げるブートストラップ。
        ff_effort = self._ff.predict_effort(ref_speed, future_speeds) if self._ff.has_model else 0.0

        # 計測 dt: サイクルスキップ（バス遅延）時に固定 dt のままだと微分が
        # スパイクし積分が過小評価されるため、実経過時間を PID と調停器に渡す。
        now = loop.time()
        dt = self._interval_s if self._last_cycle_time is None else now - self._last_cycle_time
        self._last_cycle_time = now

        pid_u = self._pid.update(
            ref_speed,
            actual_speed,
            dt=dt,
            saturated_high=self._saturated_high,
            saturated_low=self._saturated_low,
        )

        # FF と PID を符号付き努力量として合成し、ペダルへの写像は調停器に一任する。
        # 旧実装（PID の符号分割 + アクセル優先排他）は FF アクセル中の減速権限喪失と
        # FF ブレーキのバンバン制御を生んでいた（コードレビュー 2026-06-11 指摘 #1）。
        arb_out = self._arbiter.arbitrate(ff_effort + pid_u, dt)
        self._saturated_high = arb_out.saturated_high
        self._saturated_low = arb_out.saturated_low
        accel_opening = arb_out.accel_opening
        brake_opening = arb_out.brake_opening
        self._current_accel_opening = accel_opening
        self._current_brake_opening = brake_opening

        calib = self._profile.calibration
        if calib is None:
            _logger.error("キャリブレーションデータがない: 緊急停止")
            await self._abort_emergency()
            return

        accel_pos = self._opening_to_position(
            accel_opening, calib.accel_zero_pos, calib.accel_full_pos
        )
        brake_pos = self._opening_to_position(
            brake_opening, calib.brake_zero_pos, calib.brake_full_pos
        )

        # CAN 読み取りの await 中に stop()/非常停止が入った場合、
        # ここで中断しないと EMERGENCY 後の home_return と競合する位置指令を送ってしまう。
        if not self._running:
            return

        try:
            accel_current, brake_current = await asyncio.gather(
                self._drive_accel_axis(accel_pos),
                self._drive_brake_axis(brake_pos),
            )
        except Exception:
            _logger.exception("アクチュエータ通信失敗: 緊急停止")
            await self._abort_emergency()
            return

        self._last_snapshot = RealtimeSnapshot(
            actual_speed_kmh=actual_speed,
            accel_pos=accel_pos,
            brake_pos=brake_pos,
            accel_current_ma=accel_current,
            brake_current_ma=brake_current,
            captured_at=loop.time(),
        )

        if self._safety_check.check_overcurrent(accel_current, "accel"):
            _logger.warning("アクセル過電流: %.1f mA", accel_current)
            await self._abort_emergency()
            return

        if self._safety_check.check_overcurrent(brake_current, "brake"):
            _logger.warning("ブレーキ過電流: %.1f mA", brake_current)
            await self._abort_emergency()
            return

        # KPI 実行時計測: 逸脱自動停止（運用者設定、例 2.0km/h）はプライマリー KPI
        # （例外なく 1.0km/h 以内）より緩く、ログも間引かれるため、ここで全サンプルを
        # 集計しないと KPI 違反が観測できない（指摘 #7）。
        self._kpi.update(ref_speed, actual_speed, loop.time())

        deviation = abs(ref_speed - actual_speed)
        threshold = self._profile.stop_config.deviation_threshold_kmh
        if deviation > threshold:
            if self._deviation_start is None:
                self._deviation_start = loop.time()
            deviation_duration = loop.time() - self._deviation_start
        else:
            self._deviation_start = None
            deviation_duration = 0.0

        if self._safety_check.check_deviation(ref_speed, actual_speed, deviation_duration):
            _logger.warning(
                "走行逸脱: ref=%.1f actual=%.1f duration=%.1fs",
                ref_speed,
                actual_speed,
                deviation_duration,
            )
            await self._abort_emergency()
            return

        self._cycle_count += 1
        if (
            self._cycle_count % self._log_every_n_cycles == 0
            and self._log_writer
            and self._session_id
        ):
            self._enqueue_log_write(
                DriveLogData(
                    ref_speed_kmh=ref_speed,
                    actual_speed_kmh=actual_speed,
                    accel_opening=accel_opening,
                    brake_opening=brake_opening,
                    accel_pos=accel_pos,
                    brake_pos=brake_pos,
                    accel_current=accel_current,
                    brake_current=brake_current,
                )
            )

    def _enqueue_log_write(self, data: DriveLogData) -> None:
        """ログ書き込みタスクを上限付きで起動する。滞留時はログを捨てて走行を優先する。"""
        assert self._log_writer is not None and self._session_id is not None
        if len(self._pending_log_tasks) >= MAX_PENDING_LOG_TASKS:
            if not self._log_backlog_active:
                self._log_backlog_active = True
                _logger.warning(
                    "ログ書き込みが %d 件滞留: 解消まで走行ログをスキップします (DB 遅延の疑い)",
                    MAX_PENDING_LOG_TASKS,
                )
            return
        if self._log_backlog_active:
            self._log_backlog_active = False
            _logger.info("ログ書き込みの滞留が解消しました")
        task = asyncio.ensure_future(self._log_writer.write_log(self._session_id, data))
        # GC によるタスク消失防止のため完了まで強参照を保持する
        self._pending_log_tasks.add(task)
        task.add_done_callback(self._on_log_write_done)

    def _on_log_write_done(self, task: asyncio.Task[None]) -> None:
        """ログ書き込みタスクの参照を解放し、例外をログに記録する。走行は継続する。"""
        self._pending_log_tasks.discard(task)
        if task.cancelled():
            return
        exc = task.exception()
        if exc is not None:
            _logger.error("ログ書き込みエラー (走行継続)", exc_info=exc)

    def _ref_speed_at(self, t_s: float) -> float:
        """経過時間 [s] における基準車速 [km/h] を線形補間で返す。範囲外は端点値でクランプ。

        先読み（t_s = elapsed + horizon）でも使うため、軌跡末尾を超える場合は終端値を返す。
        """
        points = self._mode.reference_speed

        if not points:
            return 0.0

        if t_s <= points[0].time_s:
            return points[0].speed_kmh

        if t_s >= points[-1].time_s:
            return points[-1].speed_kmh

        # 50ms 毎に 5 回（現在値 + 先読み4点）呼ばれるため、線形走査ではなく bisect で
        # O(log n) で区間を特定する（WLTC 級の数千点モードでもサイクル予算を消費しない）
        i = bisect.bisect_right(self._ref_times, t_s) - 1
        p0 = points[i]
        p1 = points[i + 1]
        dt = p1.time_s - p0.time_s
        if dt == 0.0:
            return p1.speed_kmh
        t_frac = (t_s - p0.time_s) / dt
        return p0.speed_kmh + t_frac * (p1.speed_kmh - p0.speed_kmh)

    def _opening_to_position(self, opening_pct: float, zero_pos: int, full_pos: int) -> int:
        """開度 [%] をアクチュエータ位置 [pulse] に変換する。"""
        return zero_pos + round((full_pos - zero_pos) * opening_pct / 100.0)

    async def _drive_accel_axis(self, pos: int) -> float:
        """アクセル軸に位置指令を送り電流値を返す（同一バス上で逐次実行）。"""
        await self._accel_driver.move_to_position(pos)
        return await self._accel_driver.read_current()

    async def _drive_brake_axis(self, pos: int) -> float:
        """ブレーキ軸に位置指令を送り電流値を返す（同一バス上で逐次実行）。"""
        await self._brake_driver.move_to_position(pos)
        return await self._brake_driver.read_current()


def _log_emergency_error_callback(task: asyncio.Task[None]) -> None:
    """非常停止コールバックタスクの例外をログに記録する（黙殺防止）。"""
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        _logger.error("非常停止コールバックが失敗しました", exc_info=exc)
