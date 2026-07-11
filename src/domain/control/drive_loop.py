"""50ms 制御ループ。FF+PID 制御・ペダル調停・安全チェック・KPI 計測・ログ記録を担う。"""

from __future__ import annotations

import asyncio
import bisect
import logging
from collections.abc import Awaitable, Callable
from typing import Protocol

from src.domain.control.base_loop import (
    MAX_PENDING_LOG_TASKS,
    WEDGED_CYCLE_TIMEOUT_S,
    CycleLoopBase,
    LogWriterProtocol,
)
from src.domain.control.conversions import opening_to_position
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.ilc import ILCController
from src.domain.control.kpi_monitor import KPIMonitor
from src.domain.control.pedal_arbiter import PedalArbiter
from src.domain.control.pedal_plan import PedalPlan, PlanPhase
from src.domain.control.pedal_safety import enforce_pedal_exclusion
from src.domain.control.trim import TrimController
from src.models.drive_log import DriveLogData
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile
from src.models.system_state import RealtimeSnapshot

_logger = logging.getLogger(__name__)

CONTROL_LOOP_INTERVAL_S: float = 0.05
LOG_EVERY_N_CYCLES: int = 2
# ゲインスケジューリング: 速い補正層の実効ゲイン正規化係数 scale = clamp(g(v)/g_nominal, MIN, MAX)。
# 極端なプラントゲイン比でも不安定化しないよう比を有界にする。ブースト上限は 1.5 に抑える
# （B-8-3 移管）: むだ時間 θ 起因の安定限界はプラントゲイン正規化では消えないため、上限 3.0
# だと高速域で実効 kp が過大になる（実機 2026-07-10: scale3.0×kp3.91=11.7 が SIMC 適正 1.6 の
# 約7倍で 1Hz リミットサイクル＝ペダルシーソーを生んだ）。減衰側 0.5 は不確かな帯で弱める安全側。
_GAIN_SCALE_MIN: float = 0.5
_GAIN_SCALE_MAX: float = 1.5
# 基準速度が減速トレンドかを判定するしきい値 [km/h]（最短ホライズン先の基準 − 現基準）。
# これ未満なら減速フェーズとみなし制動側プラントゲインを使う。
_GAIN_DECEL_TREND_KMH: float = -0.1
# 軸診断（.steering/20260620-modbus-retry-cycle-stall フェーズ3）:
# move_to_position（FC16書込）直後の read_current（FC03読取）が初回タイムアウトする
# 現象への対策。timeout 調整では解決せず（試行1で無効と判明済み）、書込→読取間の
# RS-485 半二重切替 / スレーブ内部処理タイミングを疑い待機を挿入したところ有効だった
# （試行2、brake 軸）。試行2後の残存軽微遅延（52件）をサイクル内訳診断で切り分けた
# ところ、頻度は低いが accel 軸でも同じ機構（書込直後の読取で初回応答欠落）が起きて
# いたと判明したため、accel 軸にも同じ待機を適用する（試行3）。
AXIS_PRE_READ_DELAY_S: float = 0.01
# サイクル内訳診断（フェーズ3 試行2後の残存軽微遅延切り分け用）: CAN 読取・軸駆動の
# どちらが 50ms 予算超過の原因かをサイクル毎に計測し、閾値超過時のみログする。
CYCLE_DIAG_THRESHOLD_S: float = 0.08


class ActuatorDriverProtocol(Protocol):
    async def move_to_position(self, pos: int) -> None: ...

    async def read_current(self) -> float: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


class SafetyCheckProtocol(Protocol):
    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...

    def check_deviation(self, ref: float, actual: float, duration: float) -> bool: ...


class DriveLoop(CycleLoopBase):
    """50ms 制御ループを管理するドメインコンポーネント。

    start() でループを開始し、stop() または on_complete/on_emergency コールバックで停止する。
    asyncio.sleep を使わず、開始時刻基準の絶対時刻グリッド（call_at）でスケジューリングすることで
    ジッタを ±5ms 以内に抑えつつ、相対 call_later のドリフト（終端でのログ末尾欠落）を防ぐ。

    制御構成: FF（純粋フィードフォワード）と PID（誤差補正）を符号付き努力量として合成し、
    PedalArbiter がペダルへ写像する。アクセル・ブレーキの排他はこの写像の構造的性質であり、
    調停層の飽和フラグを PID の条件付き積分へ返してワインドアップを防ぐ。

    サイクル実行のライフサイクル・ウォッチドッグ・ログ書込バックログ管理は
    CycleLoopBase（LearningLoop/ScheduleLoop と共通）に委譲する。
    """

    _cycle_label = "制御サイクル"

    _paused: bool
    _paused_elapsed: float
    _started_at: float
    _cycle_count: int
    _deviation_start: float | None

    def __init__(
        self,
        ff_controller: FeedforwardController,
        trim: TrimController,
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
        ilc: ILCController | None = None,
        plan: PedalPlan | None = None,
    ) -> None:
        super().__init__(
            interval_s=interval_s,
            on_complete=on_complete,
            on_emergency=on_emergency,
            log_writer=log_writer,
            session_id=session_id,
        )
        self._ff = ff_controller
        # 閉ループ補正はトリム（凍結帯／低速トリム／速い補正層）が担う。速い補正層が
        # プロファイルの PID を内包する。
        self._trim = trim
        # ペダル操作計画（走行開始時にオフライン生成）。None なら従来経路（FF を毎サイクル
        # 評価し速い補正層を直結）へフォールバックする＝モデル未ロードのブートストラップ互換。
        self._plan = plan
        # 反復学習制御（ILC）の時刻別補正 effort。None なら補正なし（Stage C 性能向上手段で
        # あり安全の前提にしない）。FF+PID に加算する第3の effort 項として合成する。
        self._ilc = ilc
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._mode = mode
        self._safety_check = safety_check
        self._log_every_n_cycles = log_every_n_cycles
        # 逸脱（基準車速からの乖離）による自動非常停止を無効化するか。PID 自動適合では
        # 未適合ゲインの追従誤差が逸脱しきい値を超えて非常停止し、適合自体が成立しなく
        # なるため True にする。過電流・CAN 断・ウォッチドッグ等の安全網は維持される。
        self._disable_deviation_check = disable_deviation_check

        self._paused = False
        self._paused_elapsed = 0.0
        self._started_at = 0.0
        self._cycle_count = 0
        self._deviation_start = None
        # FF+PID 合成後のペダル写像と振動抑制を担う調停器。プロファイル定数で構成する。
        self._arbiter = PedalArbiter(
            profile.feedforward_params,
            max_accel_opening=profile.max_accel_opening,
            max_brake_opening=profile.max_brake_opening,
            nominal_dt_s=interval_s,
        )
        self._kpi = KPIMonitor()
        # 調停の飽和フラグ。次サイクルの PID 条件付き積分（アンチワインドアップ）に渡す。
        self._saturated_high = False
        self._saturated_low = False
        # PID/調停の計測 dt 用。サイクルスキップ時に微分スパイクを起こさない。
        self._last_cycle_time: float | None = None
        # bisect 用に基準速度の時刻列を前計算（_ref_speed_at はサイクル毎に複数回呼ばれる）
        self._ref_times: list[float] = [p.time_s for p in mode.reference_speed]
        # PID 先読み補償: PID フィードバックが参照する基準速度サンプリングのみを
        # この秒数だけ前倒しし、FB ループのむだ時間を補償する。FF は now-frame で動き、
        # 先読みはモデルの horizons 特徴量が担う（FF への前倒しは二重補償となり、
        # 実車速が基準を先行する系統偏差を生むため行わない）。KPI・逸脱判定・ログも
        # now-frame（前倒し前）で評価する。
        self._pid_preview_s: float = max(0.0, profile.dynamics_params.pid_preview_s)
        self._last_ref_speed: float | None = None
        # ゲインスケジューリングの公称逆ゲイン g_nominal = fopdt_tau/fopdt_k [%/(km/h/s)]。
        # FOPDT の初期加速応答は開度ステップ Δu に対し k/τ·Δu [km/h/s] なので、その逆数
        # τ/k が「単位加速度あたりの開度」= GainSchedule の g(v) と同じ次元になる（B-6 是正:
        # 旧 1/fopdt_k は定常ゲイン逆数 [%/(km/h)] で probe の過渡勾配と次元不整合だった）。
        # この点で scale=1 になる。fopdt 未同定なら無効（scale=1 固定）。
        fopdt_k = profile.dynamics_params.fopdt_k
        fopdt_tau = profile.dynamics_params.fopdt_tau
        self._g_nominal: float | None = (
            fopdt_tau / fopdt_k
            if fopdt_k is not None and fopdt_k > 0.0 and fopdt_tau is not None and fopdt_tau > 0.0
            else None
        )

    def _reset_for_start(self) -> None:
        self._paused = False
        self._paused_elapsed = 0.0
        self._trim.reset()
        self._arbiter.reset()
        self._kpi = KPIMonitor()
        self._saturated_high = False
        self._saturated_low = False
        self._last_cycle_time = None
        self._deviation_start = None
        self._cycle_count = 0
        loop = asyncio.get_running_loop()
        self._started_at = loop.time()

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

    @property
    def current_ref_speed(self) -> float | None:
        """直近サイクルの基準車速 [km/h]。未実行時は None。"""
        return self._last_ref_speed

    @property
    def kpi_summary(self) -> dict[str, float]:
        """走行中〜終了時点の KPI 集計（P95・最大偏差・符号反転率・ハード違反数）。"""
        return self._kpi.summary()

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
                # 完了分岐はログを書かずに return するため、t=total_duration 相当の
                # サンプルが構造的に欠落し最終ログが t≈total_duration−0.1s になっていた。
                # 停止前に末端状態を 1 行だけ記録してから終了する（新規バス通信は行わない）。
                self._log_final_sample()
                self.stop()
                await self._on_complete()
                return

        # now-frame: KPI・逸脱判定・ログ・WS 表示・名目 effort はこの基準速度で評価する。
        ref_speed = self._ref_speed_at(elapsed_s)
        self._last_ref_speed = ref_speed
        # 先読みは now-frame からのモデル horizons が担う（前倒しは二重補償になるため行わない）。
        future_speeds = [self._ref_speed_at(elapsed_s + h) for h in self._ff.horizons]
        past_speeds = [self._ref_speed_at(elapsed_s - h) for h in self._ff.past_horizons]
        # 制御フレーム: 速い補正層のみ FB ループのむだ時間補償として pid_preview_s だけ
        # 前倒しした基準速度を追う（pid_preview_s=0 なら now-frame と一致）。
        ref_speed_pid = self._ref_speed_at(elapsed_s + self._pid_preview_s)

        try:
            actual_speed = await self._can_reader.read_speed()
        except Exception:
            _logger.exception("CAN 車速取得失敗: 緊急停止")
            await self._abort_emergency()
            return
        _cycle_t_can = loop.time()

        # 計測 dt: サイクルスキップ（バス遅延）時に固定 dt のままだと微分が
        # スパイクし積分が過小評価されるため、実経過時間を制御層と調停器に渡す。
        now = loop.time()
        dt = self._interval_s if self._last_cycle_time is None else now - self._last_cycle_time
        self._last_cycle_time = now

        # ゲインスケジューリング: 実車速でのプラントゲイン g(v) を公称値との比にして速い補正層を
        # 正規化する。減速トレンド（基準が下降）では制動側、それ以外は駆動側の g を使う。
        gain_scale = self._gain_scale(actual_speed, ref_speed, future_speeds)

        # ILC 補正 effort（反復学習）: 同一モード反復走行で蓄積した時刻別残差補正を now-frame で
        # 加える。テーブル未ロード時は 0。名目 effort と同じ符号付き努力量として合成する。
        ilc_effort = self._ilc.effort_at(elapsed_s) if self._ilc is not None else 0.0

        if self._plan is not None:
            # プラン＋トリム経路: 名目 effort は走行開始時に焼き込んだ plan.effort_at（FF を
            # 毎サイクル評価しない）。トリムが偏差の大きさで凍結／低速／速い補正層を切り替え、
            # フェーズ権限で操作の向きを制限する（人間的なペダルワーク）。
            base_effort = self._plan.effort_at(elapsed_s)
            phase = self._plan.phase_at(elapsed_s)
            trim_u = self._trim.update(
                ref_speed_pid,
                actual_speed,
                dt,
                phase=phase,
                saturated_high=self._saturated_high,
                saturated_low=self._saturated_low,
                gain_scale=gain_scale,
            )
            # フェーズ権限: 速い補正層が非アクティブなら向きをクランプ（DRIVE はブレーキに
            # 落とさない・COAST は踏まない・BRAKE はアクセルに跳ねない・STOP_HOLD はプランの
            # 停車保持に委ねる）。速い補正層アクティブ（偏差 0.5km/h 超）は無権限＝max≤1.0 の
            # 安全網。削った向きは飽和フラグへ反映し次サイクルのトリム積分を止める。
            effort, clamp_high, clamp_low = self._apply_phase_authority(
                base_effort, trim_u, ilc_effort, phase
            )
            if _logger.isEnabledFor(logging.DEBUG):
                _logger.debug(
                    "plan t=%.2fs: base=%+.2f trim=%+.2f ilc=%+.2f phase=%s",
                    elapsed_s,
                    base_effort,
                    trim_u,
                    ilc_effort,
                    phase,
                )
        else:
            # プランなし（モデル未ロードのブートストラップ）: 従来経路。FF を毎サイクル評価し、
            # 速い補正層 PID を直結（トリム 3 層・フェーズ権限を通さない）＝完全回帰。
            base_effort = (
                self._ff.predict_effort(ref_speed, future_speeds, past_speeds)
                if self._ff.has_model
                else 0.0
            )
            pid_u = self._trim.fast_pid.update(
                ref_speed_pid,
                actual_speed,
                dt=dt,
                saturated_high=self._saturated_high,
                saturated_low=self._saturated_low,
                gain_scale=gain_scale,
            )
            effort = base_effort + pid_u + ilc_effort
            clamp_high = clamp_low = False

        # 名目 effort とトリム・ILC を符号付き努力量として合成し、ペダルへの写像は調停器に一任。
        arb_out = self._arbiter.arbitrate(effort, dt)
        # フェーズ権限で削った向きも飽和として次サイクルのトリムへ返す（積分ワインドアップ防止）。
        self._saturated_high = arb_out.saturated_high or clamp_high
        self._saturated_low = arb_out.saturated_low or clamp_low
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

        accel_pos = opening_to_position(accel_opening, calib.accel_zero_pos, calib.accel_full_pos)
        brake_pos = opening_to_position(brake_opening, calib.brake_zero_pos, calib.brake_full_pos)

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
            self._kpi.update(
                ref_speed,
                actual_speed,
                loop.time(),
                accel_opening=self._current_accel_opening,
                brake_opening=self._current_brake_opening,
            )

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

    def _log_final_sample(self) -> None:
        """走行完了時、停止前に末端状態を 1 行だけログする（新規バス通信は行わない）。

        基準速度は total_duration 時点の末端値（WLTP_ExHi なら 0.0）、実測系（実車速・
        位置・電流）は直近サイクルの `_last_snapshot` を、開度は直近の指令値を再利用する。
        これにより、従来欠落していた t=total_duration 相当のサンプルを補い、ログ時間軸を
        モード全長までカバーさせる。snapshot 未取得（初回サイクル即完了など）ならスキップする。
        書き込みは既存の `_enqueue_log_write`（上限・例外処理を再利用）に委ね、失敗しても
        `_on_complete` を妨げない。
        """
        if self._log_writer is None or self._session_id is None or self._last_snapshot is None:
            return
        snap = self._last_snapshot
        self._enqueue_log_write(
            DriveLogData(
                ref_speed_kmh=self._ref_speed_at(self._mode.total_duration),
                actual_speed_kmh=snap.actual_speed_kmh,
                accel_opening=self._current_accel_opening,
                brake_opening=self._current_brake_opening,
                accel_pos=snap.accel_pos,
                brake_pos=snap.brake_pos,
                accel_current=snap.accel_current_ma,
                brake_current=snap.brake_current_ma,
            )
        )

    def _apply_phase_authority(
        self,
        base_effort: float,
        trim_u: float,
        ilc_effort: float,
        phase: PlanPhase | None,
    ) -> tuple[float, bool, bool]:
        """フェーズ権限を適用した合成 effort と、クランプで削った向きの飽和フラグを返す。

        プランなし（phase=None）または速い補正層アクティブ時は権限を課さず素の合成を返す
        （偏差 0.5km/h 超では max≤1.0 を守るため向きを制限しない安全網）。プランありで速い層が
        非アクティブなら、フェーズが許す向きへ合成 effort をクランプする:
          - DRIVE     : effort<0（ブレーキ）を 0 に（アクセル調整のみ）→ 制動側クランプ
          - COAST     : effort を 0 に（踏まない）→ 両側クランプ
          - BRAKE     : effort>0（アクセル）を 0 に（制動のみ）→ 加速側クランプ
          - STOP_HOLD : プランの停車保持（base_effort）に委ね、トリム・ILC を無効化
        """
        total = base_effort + trim_u + ilc_effort
        if phase is None or self._trim.is_fast_active:
            return total, False, False
        if phase == PlanPhase.STOP_HOLD:
            # 停車保持はプランが支配。トリム/ILC を切る（両側とも積分を止める）。
            return base_effort, True, True
        if phase == PlanPhase.DRIVE:
            if total < 0.0:
                return 0.0, False, True  # ブレーキ方向を削った＝制動側飽和
            return total, False, False
        if phase == PlanPhase.BRAKE:
            if total > 0.0:
                return 0.0, True, False  # アクセル方向を削った＝加速側飽和
            return total, False, False
        # COAST: 踏まない
        return 0.0, True, True

    def _gain_scale(
        self, actual_speed: float, ref_speed: float, future_speeds: list[float]
    ) -> float:
        """速度依存プラントゲイン正規化係数 scale = clamp(g(v)/g_nominal, MIN, MAX) を返す。

        ゲインスケジュール未構築（モデル未ロード）または fopdt_k 未同定なら 1.0（従来動作）。
        基準速度が減速トレンド（最短ホライズン先が現基準より低い）なら制動側 g を、それ以外は
        駆動側 g を使う。ref ベースの判定なので計測ノイズでチャタリングしない。
        """
        schedule = self._ff.gain_schedule
        if schedule is None or self._g_nominal is None:
            return 1.0
        ref_trend = (future_speeds[0] - ref_speed) if future_speeds else 0.0
        if ref_trend < _GAIN_DECEL_TREND_KMH:
            g = schedule.brake_gain_at(actual_speed)
        else:
            g = schedule.accel_gain_at(actual_speed)
        scale = g / self._g_nominal
        return max(_GAIN_SCALE_MIN, min(_GAIN_SCALE_MAX, scale))

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

    async def _drive_accel_axis(self, pos: int) -> float:
        """アクセル軸に位置指令を送り電流値を返す（同一バス上で逐次実行）。"""
        await self._accel_driver.move_to_position(pos)
        # 書込直後の読取での初回応答欠落対策（AXIS_PRE_READ_DELAY_S 参照）。
        await asyncio.sleep(AXIS_PRE_READ_DELAY_S)
        return await self._accel_driver.read_current()

    async def _drive_brake_axis(self, pos: int) -> float:
        """ブレーキ軸に位置指令を送り電流値を返す（同一バス上で逐次実行）。"""
        await self._brake_driver.move_to_position(pos)
        # 書込直後の読取での初回応答欠落対策（AXIS_PRE_READ_DELAY_S 参照）。
        await asyncio.sleep(AXIS_PRE_READ_DELAY_S)
        return await self._brake_driver.read_current()


__all__ = [
    "CONTROL_LOOP_INTERVAL_S",
    "LOG_EVERY_N_CYCLES",
    "MAX_PENDING_LOG_TASKS",
    "WEDGED_CYCLE_TIMEOUT_S",
    "ActuatorDriverProtocol",
    "CANReaderProtocol",
    "DriveLoop",
    "LogWriterProtocol",
    "SafetyCheckProtocol",
]
