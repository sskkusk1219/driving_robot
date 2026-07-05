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
from src.domain.control.pedal_safety import enforce_pedal_exclusion
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
# ブレーキ軸診断（.steering/20260620-modbus-retry-cycle-stall フェーズ3 試行2）:
# move_to_position（FC16書込）直後の read_current（FC03読取）がほぼ毎サイクル初回
# タイムアウトする現象を仮説検証するための待機。accel 軸では再現しないため timeout
# 調整ではなく、書込→読取間の RS-485 半二重切替 / スレーブ内部処理タイミングを疑う。
# 効果がなければ 0.0 に戻す（試行1の timeout 引き上げは無効と判明済み）。
BRAKE_PRE_READ_DELAY_S: float = 0.01
# サイクル内訳診断（フェーズ3 試行2後の残存軽微遅延切り分け用）: CAN 読取・軸駆動の
# どちらが 50ms 予算超過の原因かをサイクル毎に計測し、閾値超過時のみログする。
CYCLE_DIAG_THRESHOLD_S: float = 0.08
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
    _paused: bool
    _paused_elapsed: float
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
        disable_deviation_check: bool = False,
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
        # 逸脱（基準車速からの乖離）による自動非常停止を無効化するか。PID 自動適合では
        # 未適合ゲインの追従誤差が逸脱しきい値を超えて非常停止し、適合自体が成立しなく
        # なるため True にする。過電流・CAN 断・ウォッチドッグ等の安全網は維持される。
        self._disable_deviation_check = disable_deviation_check

        self._running = False
        self._paused = False
        self._paused_elapsed = 0.0
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
        # ストール計測（.steering/20260620-modbus-retry-cycle-stall）: 連続スキップが
        # 解消するたびに 1 件のストールとして回数・累積時間・最大継続時間を集計する。
        self._stall_count = 0
        self._stall_total_s = 0.0
        self._stall_max_s = 0.0
        self._log_backlog_active = False
        # bisect 用に基準速度の時刻列を前計算（_ref_speed_at はサイクル毎に複数回呼ばれる）
        self._ref_times: list[float] = [p.time_s for p in mode.reference_speed]
        # 先読み補償（アクチュエータ〜車両系のむだ時間補償）: 制御用の基準速度サンプリングを
        # この秒数だけ前倒しする。KPI・逸脱判定・ログは now-frame（前倒し前）のまま評価する。
        self._preview_s: float = max(0.0, profile.dynamics_params.preview_time_s)
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
        self._paused = False
        self._paused_elapsed = 0.0
        self._pid.reset()
        self._arbiter.reset()
        self._kpi = KPIMonitor()
        self._saturated_high = False
        self._saturated_low = False
        self._last_cycle_time = None
        self._deviation_start = None
        self._cycle_count = 0
        self._consecutive_skips = 0
        self._stall_count = 0
        self._stall_total_s = 0.0
        self._stall_max_s = 0.0
        loop = asyncio.get_running_loop()
        self._started_at = loop.time()
        loop.call_later(self._interval_s, self._schedule_next_cycle)

    def stop(self) -> None:
        """制御ループを停止する。進行中のサイクルはアクチュエータ書き込み前に中断される。"""
        self._running = False

    def pause(self) -> None:
        """走行を一時停止する。基準速度タイムライン（経過時間の進行）を凍結する。

        制御サイクル自体は止めず、一時停止した瞬間の経過時間を保持して以降のサイクルで
        その時刻の基準速度を参照し続ける。これにより目標車速が一定値に固定され、PID が
        その速度を保持して走り続ける（安全チェック・ウォッチドッグも通常どおり継続）。
        実行中でない、または既に一時停止中の場合は何もしない。
        """
        if not self._running or self._paused:
            return
        loop = asyncio.get_running_loop()
        self._paused_elapsed = loop.time() - self._started_at
        self._paused = True

    def resume(self) -> None:
        """一時停止した走行を再開する。タイムラインを凍結時点の続きから進める。

        _started_at を現在時刻から凍結経過時間を引いた値へシフトし、elapsed_s が
        凍結時点から連続して進むようにする。一時停止中でない場合は何もしない。
        """
        if not self._paused:
            return
        loop = asyncio.get_running_loop()
        self._started_at = loop.time() - self._paused_elapsed
        # サイクルスキップ扱いの dt スパイクを避けるため、次サイクルは固定 dt から再開する
        self._last_cycle_time = None
        self._paused = False

    @property
    def is_paused(self) -> bool:
        return self._paused

    async def stop_and_join(self, timeout_s: float = 2.0) -> None:
        """停止し、進行中のサイクルタスクの完了を待つ。

        非常停止・正常停止で home_return を開始する前に必ず await すること。
        飛行中の位置指令（move_to_position の await 中）と原点復帰シーケンスが
        同一軸のバスでインターリーブし、非常停止中にペダル指令が再適用されるのを防ぐ
        （コードレビュー 2026-06-11 指摘 #6）。

        タイムアウト時はタスクをキャンセルする（バスが完全に固まっている場合、
        非常停止をこれ以上遅延させない）。

        on_complete（stop_auto_drive）はサイクルタスク内から await されるため、その経路から
        本メソッドが呼ばれると自タスクを join しようとして wait_for がタイムアウト→自タスクを
        cancel し、呼び出し元の後続 await（home_return）が CancelledError で中断される。
        `current_task() is サイクルタスク` のときは既にサイクルが終了処理中のため join 不要として
        即 return する（正常完了で約2秒ハング＋原点未復帰になるのを防ぐ）。
        """
        self.stop()
        task = self._cycle_task
        if task is None or task.done() or task is asyncio.current_task():
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

    @property
    def stall_summary(self) -> dict[str, float]:
        """走行中〜終了時点のサイクルストール集計（回数・累積時間・最大継続時間）。

        ストール＝前サイクル未完了（バス再送等）により後続サイクルが連続スキップされた
        1 エピソード。ウォッチドッグ（非常停止）に至らず解消したものも含む。
        """
        return {
            "stall_count": float(self._stall_count),
            "stall_total_s": self._stall_total_s,
            "stall_max_s": self._stall_max_s,
        }

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
            if self._consecutive_skips > 0:
                stall_duration_s = self._consecutive_skips * self._interval_s
                self._stall_count += 1
                self._stall_total_s += stall_duration_s
                self._stall_max_s = max(self._stall_max_s, stall_duration_s)
                _logger.warning(
                    "制御サイクルストール解消: 継続時間=%.2fs (累計 %d 回 / %.2fs)",
                    stall_duration_s,
                    self._stall_count,
                    self._stall_total_s,
                )
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
        _cycle_t0 = loop.time()
        # 一時停止中は経過時間を凍結時点に固定し、基準速度を一定に保つ。
        # 自然完了判定もスキップして、保持区間の途中で走行が終了しないようにする。
        if self._paused:
            elapsed_s = self._paused_elapsed
        else:
            elapsed_s = loop.time() - self._started_at
            if elapsed_s >= self._mode.total_duration:
                self.stop()
                await self._on_complete()
                return

        # now-frame: KPI・逸脱判定・ログ・WS 表示はこの基準速度で評価する（前倒ししない）。
        ref_speed = self._ref_speed_at(elapsed_s)
        self._last_ref_speed = ref_speed
        # 制御フレーム: FF・PID はむだ時間補償のため preview_s だけ前倒しした基準速度で動く。
        t_ctrl = elapsed_s + self._preview_s
        ref_speed_ctrl = self._ref_speed_at(t_ctrl)
        future_speeds = [self._ref_speed_at(t_ctrl + h) for h in self._ff.horizons]
        # 過去方向の基準速度（過去Δv＝ランプ過渡/定常の識別）。走行開始直後（t<horizon）は
        # _ref_speed_at が先頭点にクランプするため安全。
        past_speeds = [self._ref_speed_at(t_ctrl - h) for h in self._ff.past_horizons]

        try:
            actual_speed = await self._can_reader.read_speed()
        except Exception:
            _logger.exception("CAN 車速取得失敗: 緊急停止")
            await self._abort_emergency()
            return
        _cycle_t_can = loop.time()

        # 運転モデル未ロード（初回学習走行）では FF を 0 とし PID のみで基準を追従する。
        # 収集した連続ログから初回モデルを学習し、以降は FF+PID で精度を上げるブートストラップ。
        ff_effort = (
            self._ff.predict_effort(ref_speed_ctrl, future_speeds, past_speeds)
            if self._ff.has_model
            else 0.0
        )

        # 計測 dt: サイクルスキップ（バス遅延）時に固定 dt のままだと微分が
        # スパイクし積分が過小評価されるため、実経過時間を PID と調停器に渡す。
        now = loop.time()
        dt = self._interval_s if self._last_cycle_time is None else now - self._last_cycle_time
        self._last_cycle_time = now

        pid_u = self._pid.update(
            ref_speed_ctrl,
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
        # 調停器は構造的に排他（高々一方のみ非ゼロ）だが、最終段でも同時踏み禁止を強制する保険
        accel_opening, brake_opening = enforce_pedal_exclusion(
            arb_out.accel_opening, arb_out.brake_opening
        )
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
        _cycle_t_axes = loop.time()
        _cycle_total = _cycle_t_axes - _cycle_t0
        if _cycle_total > CYCLE_DIAG_THRESHOLD_S:
            # dt（前サイクルとの実間隔）が cycle_total よりかなり大きい場合はこのサイクル
            # 開始前の遅延（イベントループのスケジューリング待ち）が主因、cycle_total 自体が
            # 大きく CAN/軸駆動のどちらかが突出していればそちらが主因と切り分けられる。
            _logger.warning(
                "サイクル内訳(閾値超): 前サイクルとの間隔dt=%.3fs "
                "本サイクル計測合計=%.3fs（CAN読取=%.3fs 軸駆動gather=%.3fs）",
                dt,
                _cycle_total,
                _cycle_t_can - _cycle_t0,
                _cycle_t_axes - _cycle_t_can,
            )

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
        # 一時停止中は保持区間のサンプルで KPI を汚さないため集計しない。
        if not self._paused:
            self._kpi.update(ref_speed, actual_speed, loop.time())

        # PID 自動適合中は逸脱による非常停止を行わない（未適合ゲインの追従誤差で
        # 適合が成立しなくなるため）。他の安全網（過電流・CAN 断・ウォッチドッグ）は維持。
        if not self._disable_deviation_check:
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

        # 一時停止中は保持区間のログを残さない（再開後にタイムラインが連続するため）
        if self._paused:
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
        # 診断用待機（BRAKE_PRE_READ_DELAY_S 参照）。
        await asyncio.sleep(BRAKE_PRE_READ_DELAY_S)
        return await self._brake_driver.read_current()


def _log_emergency_error_callback(task: asyncio.Task[None]) -> None:
    """非常停止コールバックタスクの例外をログに記録する（黙殺防止）。"""
    if task.cancelled():
        return
    exc = task.exception()
    if exc is not None:
        _logger.error("非常停止コールバックが失敗しました", exc_info=exc)
