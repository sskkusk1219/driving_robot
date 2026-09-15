"""50ms 制御ループ。プラン+トリム制御・ペダル調停・安全チェック・KPI 計測・ログ記録を担う。"""

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
from src.domain.control.kpi_monitor import KPIMonitor
from src.domain.control.pedal_arbiter import PedalArbiter
from src.domain.control.pedal_plan import PedalPlan, PlanPhase, fold_times
from src.domain.control.pedal_safety import enforce_pedal_exclusion
from src.domain.control.trim import TrimController, fast_gain_scale_cap
from src.models.drive_log import DriveLogData
from src.models.driving_mode import DrivingMode
from src.models.profile import (
    SIMC_NOMINAL_SPEED_KMH,
    FeedforwardParams,
    VehicleProfile,
    pedal_gain_at,
)
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
# ゲインの向き（駆動/制動）を切り替える合成 effort の不感帯 [%]。この内側では前回の向きを保つ。
# アクセル不感帯 0.5% より小さくすると、どちらのペダルも動いていない領域で向きがチャタる。
_GAIN_SIDE_HYSTERESIS_PCT: float = 0.5
# BRAKE フェーズ中に許すアクセル介入の上限 [%]。
# 上限は解析 effort の低開度ブレンド域 pedal_plan.ANALYTIC_BLEND_LO_PCT と同じ 5.0% に置く。
# そこは「惰行より緩い減速」を作るためにアクセルを当てる領域として逆FFモデルではなく
# 惰行カーブ基準の解析値で設計されており、人間のペダルワークとしても妥当な範囲。
# これを超える介入は速い補正層（|偏差|≥0.5km/h）の担当で、そちらは元々権限を課さない。
BRAKE_PHASE_ACCEL_ASSIST_PCT: float = 5.0
# フィードバック（トリム／速い補正層 PID）入力専用の 1 次ローパス時定数 [s]
# （2026-09-08 追加: ProblemReport_20260908 対応）。CAN 車速はフィルタなしの生値で、
# 10Hz 隣接差の実測 std≈0.2km/h がそのまま P/I 項に乗って微小ハンチングの一因になって
# いた（アクセル向き反転 実測 31回/min）。KPI・ログ・ゲインスケジューリング・最小実効
# ブレーキ判定は生車速のまま（KPI 定義は変えない）で、フィードバック入力にのみ適用する。
#
# **必ず基準速度と実車速の両方へ同一のフィルタを掛ける**（＝偏差にフィルタを掛けるのと
# 等価）。2026-09-08 の初版は実車速だけを遅らせており、ランプ追従中に「勾配 × TAU」の
# 定常偏差が恒久的に残った（実機 3ca20d43: +2.0km/h/s 区間で制御が見る偏差 +0.07 に対し
# 実偏差 -0.43、-5.0km/h/s 区間で -0.06 に対し +1.06）。しかも ILC は前走行の trim を
# プランへ吸収する構造（plan_update）なので、このバイアスがプランへ焼き込まれて反復して
# も消えなかった。ランプ追従中は偏差が一定なので、両者を同じだけ遅らせればバイアスは
# 出ず、ノイズ除去効果だけが残る。
SPEED_FEEDBACK_LPF_TAU_S: float = 0.25
# 軸診断（.steering/20260620-modbus-retry-cycle-stall フェーズ3）:
# move_to_position（FC16書込）直後の read_current（FC03読取）が初回タイムアウトする
# 現象への対策。timeout 調整では解決せず（試行1で無効と判明済み）、書込→読取間の
# RS-485 半二重切替 / スレーブ内部処理タイミングを疑い待機を挿入したところ有効だった
# （試行2、brake 軸）。試行2後の残存軽微遅延（52件）をサイクル内訳診断で切り分けた
# ところ、頻度は低いが accel 軸でも同じ機構（書込直後の読取で初回応答欠落）が起きて
# いたと判明したため、accel 軸にも同じ待機を適用する（試行3）。
# 2026-09-09: 0.01 → 0.002。MODBUS 仕様 4-1 は「レスポンスメッセージの送信完了後、1ms
# 経過後に次のクエリー受信に備えます」であり、必要な軸間ギャップは 1ms。10ms は
# 50ms サイクル予算の 18% を仕様上不要な待機に使っていた（通信だけで予算の 82% を消費
# している状況で最も安く取り戻せる枠）。仕様の 2 倍のマージンを取って 2ms とする。
AXIS_PRE_READ_DELAY_S: float = 0.002
# 位置指令の ACMD をこの秒数で終わるよう落とす（actuator_driver.acmd_for_move）。
# 制御周期の 0.8 倍。1.0 倍だとサイクルジッタで取りこぼすため余裕を持たせる。
AXIS_SMOOTH_DUTY: float = 0.8
# ── プラン（フィードフォワード）のむだ時間前倒し ─────────────────────────────
# プラン参照時刻を θ×この係数だけ前倒しする（2026-09-09 追加）。
#
# プラント全体のむだ時間 θ（実機 9eee549b で 0.5s）に対し、旧実装のプラン参照は
# effort_at(elapsed_s) / phase_at(elapsed_s) と**時間シフト 0**だった。基準軌跡の
# 要求加速度がステップ変化するコーナー（t=146s で +2.0 → −5.0 km/h/s ＝ 7km/h/s の段差）
# では、ペダルが θ 秒遅れて効くぶん 7 × 0.5 = 3.5km/h の誤差が原理的に発生する
# （実測ピーク −4.21km/h と整合）。フィードバックは誤差が出てからしか動けないので、
# ゲインをいくら上げてもこの成分は消えない。プラン側を前倒しして初めて消える。
#
# **pid_preview_s（FB 側の前倒し）とは別物。** あちらは PID が見る基準速度を前倒しする
# もので、ランプ追従中に定常偏差を作るため 0.0 のまま（むだ時間入り閉ループ模擬で
# preview 0.0→0.8s のとき p95 が 1.13 → 3.67 と悪化）。こちらは FF の位相補償で、
# 同じ模擬で p95 1.66（前倒しなし）→ 1.57（0.2s）。
#
# 係数 0.4（θ=0.5s なら 0.2s）は閉ループ模擬の掃引で決めた。フィードバック権限を是正する
# 前は 0.4s 前後が最良だったが、権限が本来の大きさになるとフィードバックが過渡を吸収する
# ぶん最適な前倒しは小さくなる（0.2s: p95 1.57 / 0.4s: 1.80 / 0.5s: 1.96）。
# 前倒しを増やすとペダル開度変化率は下がる（0.2s で 11.1 → 0.3s で 8.6 %/s）ので、
# 滑らかさを優先するなら 0.6 まで上げる余地がある。p95 が主 KPI なので 0.4 を採る。
PLAN_LEAD_THETA_FACTOR: float = 0.4
PLAN_LEAD_MAX_S: float = 0.5  # 前倒しの上限 [s]（過補償で逆にコーナー手前が荒れるため）
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
        # ペダル操作計画（走行開始時にオフライン生成または保存プランをロード）。None なら
        # 従来経路（FF を毎サイクル評価し速い補正層を直結）へフォールバックする＝モデル未ロードの
        # ブートストラップ互換。実行時 effort は plan + trim の 2 層で合成する（ILC 独立層は
        # 廃止し、反復学習はプラン更新則へ吸収した）。
        self._plan = plan
        # effort 内訳の直近値（ログ・KPI・トリム寄与率の可観測化用）。プラン学習の実効 effort
        # ソースにもなる（applied はフェーズ権限クランプ後・調停器前の合成値）。
        self._last_plan_effort: float = 0.0
        self._last_trim_effort: float = 0.0
        self._last_applied_effort: float = 0.0
        self._last_phase: str | None = None
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
        # プランのフェーズ切替検知用（切替でトリムの補正持ち越しをクリアする）。
        self._prev_plan_phase: PlanPhase | None = None
        # ゲインスケジュールの向き（駆動/制動）。不感帯内では前回値を保つ（_is_accel_side）。
        self._gain_side_accel: bool | None = None
        # 最小実効ブレーキ（2026-07-16）: ブレーキ物理不感帯の幅 [%]。合成 effort が
        # (−db, 0) の不感帯デッドゾーンに落ちると調停器が開度 0 に丸め「制動を指令したのに
        # 物理的に効かない」ため、速い補正層アクティブ時はエッジ −db へ引き上げる。
        self._brake_deadband_pct = max(0.0, profile.feedforward_params.brake_deadband_pct)
        # bisect 用に基準速度の時刻列を前計算（_ref_speed_at はサイクル毎に複数回呼ばれる）
        self._ref_times: list[float] = [p.time_s for p in mode.reference_speed]
        # PID 先読み補償: PID フィードバックが参照する基準速度サンプリングのみを
        # この秒数だけ前倒しし、FB ループのむだ時間を補償する。FF は now-frame で動き、
        # 先読みはモデルの horizons 特徴量が担う（FF への前倒しは二重補償となり、
        # 実車速が基準を先行する系統偏差を生むため行わない）。KPI・逸脱判定・ログも
        # now-frame（前倒し前）で評価する。
        self._pid_preview_s: float = max(0.0, profile.dynamics_params.pid_preview_s)
        self._last_ref_speed: float | None = None
        # ゲインスケジューリングの公称逆ゲイン g_nominal [%/(km/h/s)]。GainSchedule の g(v)
        # 「単位加速度あたりの開度」がこの値のとき scale=1 になる。
        # 同定済みペダルゲイン k'(v)［km/h/s per %］の逆数を SIMC_NOMINAL_SPEED_KMH で評価する。
        # **compute_pid_gains_simc と同じ点・同じ量**を公称に取らないと、ロバスト上限が常に
        # 張り付くか常に緩むかのどちらかになる。
        # （2026-09-09 変更: 旧実装は fopdt_tau/fopdt_k = 0.80 を使っていたが、この k は学習
        #  運転がプラトーに達しないと保持区間長を測るだけの値で、実ペダルゲインの逆数
        #  1/0.22〜1/1.06 = 0.94〜4.5 と整合しなかった。robust_kp_at の docstring 参照）
        self._g_nominal: float | None = None
        nominal_gain = pedal_gain_at(
            profile.feedforward_params, SIMC_NOMINAL_SPEED_KMH, is_accel=True
        )
        if nominal_gain is not None:
            self._g_nominal = 1.0 / nominal_gain
        else:
            # 後方互換: ペダルゲイン未同定なら旧 FOPDT 由来の公称値へフォールバック
            fopdt_k = profile.dynamics_params.fopdt_k
            fopdt_tau = profile.dynamics_params.fopdt_tau
            if (
                fopdt_k is not None
                and fopdt_k > 0.0
                and fopdt_tau is not None
                and fopdt_tau > 0.0
            ):
                self._g_nominal = fopdt_tau / fopdt_k
        # 速い補正層のロバストゲイン上限に使う値。上限は速度と駆動/制動の向きで変わるため
        # サイクル毎に trim.fast_gain_scale_cap で評価する（旧実装は起動時 1 回の定数だった）。
        self._fopdt_theta: float | None = profile.dynamics_params.fopdt_theta
        # プラン参照の前倒し秒数（FF のむだ時間補償）。θ 未同定なら 0＝従来動作。
        theta = profile.dynamics_params.fopdt_theta
        self._plan_lead_s: float = (
            min(max(0.0, theta) * PLAN_LEAD_THETA_FACTOR, PLAN_LEAD_MAX_S)
            if theta is not None
            else 0.0
        )
        # 折返し点（駆動⇄制動の入れ替わり）の時刻。前倒しをここへ近づくほど絞る
        # （_plan_lead_at 参照）。プランは走行中に差し替わらないので 1 度だけ計算する。
        self._plan_fold_times: list[float] = (
            fold_times(plan.efforts, plan.phases, plan.dt_s) if plan is not None else []
        )
        self._profile_kp: float = profile.pid_gains.kp
        self._ffp: FeedforwardParams = profile.feedforward_params
        # フィードバック入力専用のフィルタ状態（KPI・ログ用の生値とは別）。基準速度側も
        # 同じ時定数で遅らせ、ランプ追従中に定常偏差を作らないようにする。
        self._ref_speed_filt: float | None = None
        self._actual_speed_filt: float | None = None
        # 直近に送った軸指令位置（変化のない軸の書込を省くため。None=未送信）
        self._accel_pos_cmd: int | None = None
        self._brake_pos_cmd: int | None = None

    def _filtered_feedback(
        self, ref_speed: float, actual_speed: float, dt: float
    ) -> tuple[float, float]:
        """トリム／速い補正層 PID 入力専用に基準速度・実車速をローパスした対を返す。

        両方に同一の 1 次ローパスを掛ける（＝偏差にフィルタを掛けるのと等価）。片方だけを
        遅らせるとランプ追従中に「勾配 × TAU」の定常偏差が残るため、必ず対で掛ける
        （SPEED_FEEDBACK_LPF_TAU_S のコメント参照）。KPI・ログ・ゲインスケジューリング・
        最小実効ブレーキ判定には使わない（生値のまま）。初回・dt<=0 は生値をそのまま初期
        状態にする（フィルタ立ち上がり遅延を作らない）。
        """
        if self._actual_speed_filt is None or self._ref_speed_filt is None or dt <= 0.0:
            self._ref_speed_filt = ref_speed
            self._actual_speed_filt = actual_speed
        else:
            alpha = dt / (SPEED_FEEDBACK_LPF_TAU_S + dt)
            self._ref_speed_filt += (ref_speed - self._ref_speed_filt) * alpha
            self._actual_speed_filt += (actual_speed - self._actual_speed_filt) * alpha
        return self._ref_speed_filt, self._actual_speed_filt

    def _reset_for_start(self) -> None:
        self._paused = False
        self._paused_elapsed = 0.0
        self._trim.reset()
        self._arbiter.reset()
        self._kpi = KPIMonitor()
        self._saturated_high = False
        self._saturated_low = False
        self._last_cycle_time = None
        self._prev_plan_phase = None
        self._gain_side_accel = None
        self._deviation_start = None
        self._cycle_count = 0
        self._last_plan_effort = 0.0
        self._last_trim_effort = 0.0
        self._last_applied_effort = 0.0
        self._last_phase = None
        self._ref_speed_filt = None
        self._actual_speed_filt = None
        # 走行開始時は必ず 1 度指令を送る（前走行の指令位置を引き継がない）
        self._accel_pos_cmd = None
        self._brake_pos_cmd = None
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
        # トリム／速い補正層 PID にのみ渡す基準速度・実車速（同一時定数でローパス済み）。
        # KPI・ログ・ゲインスケジューリング・最小実効ブレーキ判定は生値のまま使う。
        ref_speed_fb, actual_speed_fb = self._filtered_feedback(ref_speed_pid, actual_speed, dt)

        # ゲインスケジューリング: 実車速でのプラントゲイン g(v) を公称値との比にして速い補正層を
        # 正規化する。減速トレンド（基準が下降）では制動側、それ以外は駆動側の g を使う。
        # さらにむだ時間安定上限でクランプし、実効比例ゲインが SIMC ロバスト値を超えて
        # 限界サイクル（高速巡航のペダル踏み替え）を起こさないようにする。
        is_accel_side = self._is_accel_side(ref_speed, actual_speed, future_speeds)
        gain_scale = min(
            self._gain_scale(actual_speed, is_accel_side),
            fast_gain_scale_cap(
                self._ffp,
                actual_speed,
                self._fopdt_theta,
                self._profile_kp,
                is_accel=is_accel_side,
            ),
        )
        # 低速トリム層の比例ゲインを速い層の実効値へ追随させ、層をまたいだ権限逆転
        # （偏差 0.5km/h 超でゲインが下がる）を構造的に防ぐ（trim.TRIM_SLOW_KP_RATIO）。
        fast_kp_effective = self._profile_kp * gain_scale

        if self._plan is not None:
            # プラン＋トリム経路（2 層合成）: 名目 effort は走行開始時に焼き込んだ
            # plan.effort_at（FF を毎サイクル評価しない）。トリムが偏差の大きさで凍結／低速／
            # 速い補正層を切り替え、フェーズ権限で操作の向きを制限する（人間的なペダルワーク）。
            # むだ時間前倒しは折返し点に近づくほど絞る（_plan_lead_at 参照）。
            # effort と phase は**同じ時刻**から取る（片方だけずらすとフェーズ権限クランプが
            # effort とずれて向きを誤って削る）。
            base_effort, phase = self._plan_at(elapsed_s)
            # フェーズ切替でトリムの補正持ち越し（積分・出力）をクリアする。前フェーズの
            # プラン誤差補償を新フェーズへ持ち越すと切替頭で踏み抜く（ワインドアップ事故）。
            if self._prev_plan_phase is not None and phase != self._prev_plan_phase:
                self._trim.notify_phase_change()
            self._prev_plan_phase = phase
            trim_u = self._trim.update(
                ref_speed_fb,
                actual_speed_fb,
                dt,
                phase=phase,
                saturated_high=self._saturated_high,
                saturated_low=self._saturated_low,
                gain_scale=gain_scale,
                fast_kp_effective=fast_kp_effective,
            )
            # フェーズ権限: 速い補正層が非アクティブなら向きをクランプ（DRIVE はブレーキに
            # 落とさない・COAST は踏まない・BRAKE はアクセルに跳ねない・STOP_HOLD はプランの
            # 停車保持に委ねる）。速い補正層アクティブ（偏差 0.5km/h 超）は無権限＝max≤1.0 の
            # 安全網。削った向きは飽和フラグへ反映し次サイクルのトリム積分を止める。
            effort, clamp_high, clamp_low = self._apply_phase_authority(
                base_effort, trim_u, phase, ref_speed_pid - actual_speed
            )
            # 最小実効ブレーキ: 減速コーナーで必要制動が不感帯デッドゾーン (−db, 0) に落ちる
            # と調停器が開度 0 に丸め、物理制動ゼロのまま偏差が成長する（7/15 実走 t=151-153s:
            # applied −0.9〜−1.8% → brake_opening=0.0 で偏差 +1.7km/h）。速い補正層アクティブ
            # かつ超過速度のときのみ不感帯エッジ −db へ引き上げる（偏差形 FOPDT シムで
            # ピーク 1.04→0.52km/h・制動パルス数不変を確認、2026-07-16）。
            effort = self._apply_min_effective_brake(
                effort, actual_speed, ref_speed_pid, phase
            )
            phase_str: str | None = phase.value
        else:
            # プランなし（モデル未ロードのブートストラップ）: 従来経路。FF を毎サイクル評価し、
            # 速い補正層 PID を直結（トリム 3 層・フェーズ権限を通さない）＝完全回帰。
            base_effort = (
                self._ff.predict_effort(ref_speed, future_speeds, past_speeds)
                if self._ff.has_model
                else 0.0
            )
            pid_u = self._trim.fast_pid.update(
                ref_speed_fb,
                actual_speed_fb,
                dt=dt,
                saturated_high=self._saturated_high,
                saturated_low=self._saturated_low,
                gain_scale=gain_scale,
            )
            trim_u = pid_u
            effort = base_effort + pid_u
            clamp_high = clamp_low = False
            phase_str = None

        # effort 内訳を記録（プラン学習の実効 effort・トリム寄与率・フェーズ逸脱の可観測化）。
        # applied はフェーズ権限クランプ後・調停器前の合成値。
        self._last_plan_effort = base_effort
        self._last_trim_effort = trim_u
        self._last_applied_effort = effort
        self._last_phase = phase_str

        # 名目 effort とトリムを符号付き努力量として合成し、ペダルへの写像は調停器に一任。
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
                plan_effort_pct=self._last_plan_effort,
                trim_effort_pct=self._last_trim_effort,
                applied_effort_pct=self._last_applied_effort,
                phase=self._last_phase,
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
                    plan_effort_pct=self._last_plan_effort,
                    trim_effort_pct=self._last_trim_effort,
                    applied_effort_pct=self._last_applied_effort,
                    phase=self._last_phase,
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
                plan_effort_pct=self._last_plan_effort,
                trim_effort_pct=self._last_trim_effort,
                applied_effort_pct=self._last_applied_effort,
                phase=self._last_phase,
            )
        )

    def _plan_lead_at(self, elapsed_s: float) -> float:
        """その時刻に許す前倒し量 [s]。折返し点に近いほど 0 へ絞る。

        むだ時間補償の前倒しは「遅れて効く操作を早く出す」ためのもので、要求が単調に
        変化するランプ区間では正しい。ところが**折返し点**（加速→減速の頂点、減速率が
        緩む点）では「基準がまだ加速を要求しているのにアクセルを抜く」という真逆の操作に
        なる。実機 5ac4f31d の t=143-146 では基準がまだ +2.0km/h/s で上昇中なのにプランが
        15.3% → 0% へ落ち、単独で最大偏差 −5.14km/h（max の記録）を作っていた。しかも
        ILC が学習するのは effort の**値**だけで時間軸は学習対象外なので、ILC では直せない
        （docs/Problem/引き継ぎ20260909.md 3-3, 4-③）。

        折返し点までの距離で前倒しを線形に絞る（`lead × min(1, 距離/lead)`）。
        **切り替えではなく連続な絞り込みにするのが要点**で、「符号が変わったら前倒しを
        やめる」という二値の切り替えにすると、プラン effort が 0 を跨ぐ瞬間に指令が
        数 % 跳ぶ（閉ループ模擬で開度変化率 RMS が 7.9 → 12.5 %/s に悪化した）。
        ペダルが「パチン」と動く挙動そのものなので、連続性は必須。

        Args:
            elapsed_s: 走行開始からの経過秒（now-frame）。

        Returns:
            前倒し量 [s]（0 〜 self._plan_lead_s）。
        """
        lead = self._plan_lead_s
        if lead <= 0.0 or not self._plan_fold_times:
            return lead
        # 最も近い折返し点までの距離（前後どちらでもよい。通過直後に前倒しが跳ね戻ると
        # そこで段差が出るため、両側で対称に絞る）。
        idx = bisect.bisect_left(self._plan_fold_times, elapsed_s)
        distance = float("inf")
        if idx < len(self._plan_fold_times):
            distance = self._plan_fold_times[idx] - elapsed_s
        if idx > 0:
            distance = min(distance, elapsed_s - self._plan_fold_times[idx - 1])
        return lead * min(1.0, max(0.0, distance) / lead)

    def _plan_at(self, elapsed_s: float) -> tuple[float, PlanPhase]:
        """名目 effort とフェーズを取り出す（向き別のむだ時間前倒し込み）。

        前倒しは 2 段で絞る。どちらも**連続**であることが要点で、二値の切り替えにすると
        プラン effort が 0 を跨ぐ瞬間に指令が数 % 跳ぶ（閉ループ模擬で開度変化率 RMS が
        7.9 → 12.5 %/s に悪化した＝ペダルが「パチン」と動く挙動そのもの）。

        1. **折返し点への距離で前倒し量を絞る**（`_plan_lead_at`）。駆動⇄制動が入れ替わる点
           では前倒しが「基準がまだ加速を要求しているのにアクセルを抜く」真逆の操作になる。
        2. **同じ向きの中では「より踏んでいる側」を採る**。折返しを伴わない緩め
           （ブレーキ −5% → −1% など）でも、前倒しすると制動を早く抜いて超過を作る
           （実機 5ac4f31d t=81-84 の +2.48km/h）。前倒しは踏み増しにだけ効かせたいので、
           解放方向には now-frame を保つ。折返し点近傍では 1. により前倒しが 0 に絞られて
           いるので、この max/min は符号が確定した区間でしか働かず段差を作らない。

        フェーズは effort と**同じ時刻**から取る（片方だけずらすとフェーズ権限クランプが
        effort とずれて向きを誤って削る）。踏み増し方向で now-frame を採ったときも、
        フェーズは前倒し側のままにすると権限が先に切り替わってしまうため揃える。

        Args:
            elapsed_s: 走行開始からの経過秒（now-frame）。

        Returns:
            (名目 effort [%], フェーズ)。
        """
        plan = self._plan
        assert plan is not None  # 呼び出し側でプランありを確認済み
        lead = self._plan_lead_at(elapsed_s)
        if lead <= 0.0:
            return plan.effort_at(elapsed_s), plan.phase_at(elapsed_s)

        effort_now = plan.effort_at(elapsed_s)
        plan_t = elapsed_s + lead
        effort_lead = plan.effort_at(plan_t)
        # 向きは now-frame で決める。now が 0（惰行帯）なら前倒し側の符号に従う。
        side = effort_now if effort_now != 0.0 else effort_lead
        if side > 0.0:
            use_lead = effort_lead > effort_now  # アクセルを踏み増す方向のみ
        elif side < 0.0:
            use_lead = effort_lead < effort_now  # ブレーキを踏み増す方向のみ
        else:
            use_lead = False
        if not use_lead:
            return effort_now, plan.phase_at(elapsed_s)
        return effort_lead, plan.phase_at(plan_t)

    def _apply_phase_authority(
        self,
        base_effort: float,
        trim_u: float,
        phase: PlanPhase | None,
        deficit_kmh: float = 0.0,
    ) -> tuple[float, bool, bool]:
        """フェーズ権限を適用した合成 effort（plan+trim）と、クランプで削った向きの飽和フラグ。

        プランなし（phase=None）または速い補正層アクティブ時は権限を課さず素の合成を返す
        （偏差 0.5km/h 超では max≤1.0 を守るため向きを制限しない安全網）。プランありで速い層が
        非アクティブなら、フェーズが許す向きへ合成 effort をクランプする:
          - DRIVE     : effort<0（ブレーキ）を 0 に（アクセル調整のみ）→ 制動側クランプ
          - COAST     : effort を 0 に（踏まない）→ 両側クランプ
          - BRAKE     : effort>0（アクセル）を 0 に（制動のみ）→ 加速側クランプ
          - STOP_HOLD : プランの停車保持（base_effort）に委ね、トリムを無効化
        """
        total = base_effort + trim_u
        if phase is None or self._trim.is_fast_active:
            return total, False, False
        if phase == PlanPhase.STOP_HOLD:
            # 停車保持はプランが支配。トリムを切る（両側とも積分を止める）。
            return base_effort, True, True
        if phase == PlanPhase.DRIVE:
            if total < 0.0:
                return 0.0, False, True  # ブレーキ方向を削った＝制動側飽和
            return total, False, False
        if phase == PlanPhase.BRAKE:
            if total > 0.0:
                # 減速中に「減速しすぎた」ぶんを戻す微調整は、ブレーキではなくアクセルで行う。
                # ブレーキ不感帯 5% の 1 段は 45km/h で 2.4km/h/s・75km/h で 5.3km/h/s あり、
                # KPI 許容 0.4km/h に対して桁が違う（0 と 5% の間の制動力が物理的に出せない）。
                # 一方アクセル不感帯は 0.5% と細かく、微調整に使える。
                # 実測でもこの経路は機能している（5ac4f31d t=145.9-147.0 で brake フェーズ中に
                # トリムが +3.8〜4.0% のアクセルを出している）。
                # 速度が足りない（deficit>0）ときだけ、低開度に限って通す。
                if deficit_kmh > 0.0:
                    allowed = min(total, BRAKE_PHASE_ACCEL_ASSIST_PCT)
                    return allowed, total > allowed, False
                return 0.0, True, False  # アクセル方向を削った＝加速側飽和
            return total, False, False
        # COAST: 踏まない
        return 0.0, True, True

    def _apply_min_effective_brake(
        self,
        effort: float,
        actual_speed: float,
        ref_speed_pid: float,
        phase: PlanPhase | None,
    ) -> float:
        """不感帯デッドゾーンに落ちた制動指令を物理的に効く最小値（−db）へ引き上げる。

        条件（すべて満たすときのみ）:
          - 速い補正層アクティブ（|偏差| が FAST_ENGAGE 超で介入中＝ヒステリシス付き）
          - 超過速度（actual > ref、制動が正しい向き）
          - 合成 effort が (−db, 0) ＝調停器の丸めで開度 0 になるデッドゾーン
          - STOP_HOLD 以外（停車保持はプランの保持 effort が支配）

        エスカレーションの解除は速い補正層の離脱ヒステリシス（FAST_RELEASE）に委ねるため、
        不感帯を挟んだ ON/OFF は緩いパルス制動（デューティ制御）になり高速チャタしない。
        db=6% の物理不感帯では 0〜6% の中間制動力が構造的に出せないため、これが
        「0 か 6%」の中で偏差最小を狙う唯一の手段（構造的制約への対処であり、ゲイン
        チューニングでは解決しない）。
        """
        db = self._brake_deadband_pct
        if db <= 0.0 or phase == PlanPhase.STOP_HOLD:
            return effort
        if not self._trim.is_fast_active:
            return effort
        overspeed = actual_speed - ref_speed_pid
        if overspeed <= 0.0:
            return effort
        if not (-db < effort < 0.0):
            return effort
        if not self._escalation_pays_off(overspeed, actual_speed):
            return effort
        return -db

    def _escalation_pays_off(self, overspeed_kmh: float, actual_speed: float) -> bool:
        """不感帯エッジへの引き上げが「効きすぎ」にならないかを予測して判定する。

        ブレーキ不感帯 1 段（db%）が生む追加減速度は 45km/h で 2.4、75km/h で 5.3 km/h/s ある。
        旧実装は「速い補正層アクティブ＋超過」だけで引き上げていたため、偏差 0.5km/h
        （FAST_ENGAGE）を超えた時点で 1 段入り、解除は偏差 0.3km/h を切るまで待つ。その間に
        1 段が消せる速度は超過量をはるかに上回るので、必ず反対側（不足側）へ振れる。
        実機 5ac4f31d では +2.40km/h の超過を消しに 5% を 2 秒入れて −1.35km/h まで行き過ぎ、
        これが brake フェーズ p95 2.30 と符号反転 KPI の主因になっていた
        （docs/Problem/引き継ぎ20260909.md 4-④）。

        そこで「指令してから効き始めるまで（むだ時間 θ）に 1 段が消す速度」を見積もり、
        **超過量がそれを下回るなら引き上げない**。下回る場合は 1 段を入れた時点で
        行き過ぎが確定しており、不感帯以下の指令のまま惰行させたほうが偏差が小さい。

        ペダルゲイン曲線や θ が未同定なら判定できないので True（＝従来動作）を返す。

        Args:
            overspeed_kmh: 超過速度（実車速 − 基準速度、正）。
            actual_speed: 実車速 [km/h]（ペダルゲインの速度依存に使う）。

        Returns:
            引き上げてよければ True。
        """
        theta = self._fopdt_theta
        if theta is None or theta <= 0.0:
            return True
        gain = pedal_gain_at(self._ffp, actual_speed, is_accel=False)
        if gain is None or gain <= 0.0:
            return True
        # 1 段の追加減速度 [km/h/s] × むだ時間 [s] = 反応できるようになる前に消える速度
        removable_kmh = gain * self._brake_deadband_pct * theta
        return overspeed_kmh >= removable_kmh

    def _is_accel_side(
        self, ref_speed: float, actual_speed: float, future_speeds: list[float]
    ) -> bool:
        """プラントゲインを駆動側で見るか制動側で見るかを**今動いているペダル**で決める。

        2026-09-10 変更: 判定基準を「基準速度のトレンド」から「直前サイクルの合成 effort の
        符号」へ変えた。

        旧実装は `ref(t+0.5s) − ref(t) ≥ −0.1` で、**減速区間では常に制動側**が選ばれた。
        ところが減速区間でも、プランが惰行（effort≈0）で「減速しすぎたのでアクセルを当てる」
        局面では実際に動くのはアクセルである。ロバスト上限 Kc = 1/(k'(v)·(θ+τc)) の k' は
        「これから動かすペダルのゲイン」でなければ意味がなく、基準が上り坂か下り坂かは
        関係ない。制動側の Kc は高速域で 0.27〜0.36 %/(km/h) しかないため、この取り違えが
        起きるとフィードバックがほぼ効かなくなる（実機 5ac4f31d で brake フェーズの p95 2.30 が
        drive の 1.24 と倍悪いことの説明。docs/Problem/制御フロー.md 8-② 帰結2）。

        **偏差の符号で決めてはいけない**（2026-09-10 の閉ループ模擬で確認）。
        「速度が足りない＝駆動側」とすると、プランが強く制動している最中でも駆動側の
        小さい k' を選んでしまい、実際に動くブレーキペダルに対して安定余裕が過大評価される。
        模擬では max が 5.91 → 6.71 と悪化した。フィードバックの増分が乗るのは
        合成 effort が指すペダルなので、そこを見る。

        直前サイクルの applied effort を使う（同一サイクル内では trim を求めるのに
        gain_scale が要るので循環する）。1 サイクル 50ms の遅れは、この判定が持つ
        不感帯 `_GAIN_SIDE_HYSTERESIS_KMH` より十分速い。

        Args:
            ref_speed: 現時刻の基準速度 [km/h]（初回のシードに使う）。
            actual_speed: 実車速 [km/h]（未使用。呼び出し互換のため受ける）。
            future_speeds: FF の先読みホライズン先の基準速度列（初回のシードに使う）。

        Returns:
            True なら駆動側の k'(v) を使う。
        """
        del actual_speed  # 判定はペダル側で行うため使わない（シグネチャ互換のため受ける）
        effort = self._last_applied_effort
        if effort > _GAIN_SIDE_HYSTERESIS_PCT:
            self._gain_side_accel = True
        elif effort < -_GAIN_SIDE_HYSTERESIS_PCT:
            self._gain_side_accel = False
        elif self._gain_side_accel is None:
            ref_trend = (future_speeds[0] - ref_speed) if future_speeds else 0.0
            self._gain_side_accel = ref_trend >= _GAIN_DECEL_TREND_KMH
        return bool(self._gain_side_accel)

    def _gain_scale(self, actual_speed: float, is_accel_side: bool) -> float:
        """速度依存プラントゲイン正規化係数 scale = clamp(g(v)/g_nominal, MIN, MAX) を返す。

        ゲインスケジュール未構築（モデル未ロード）または公称値未確定なら 1.0（従来動作）。
        """
        schedule = self._ff.gain_schedule
        if schedule is None or self._g_nominal is None:
            return 1.0
        if is_accel_side:
            g = schedule.accel_gain_at(actual_speed)
        else:
            g = schedule.brake_gain_at(actual_speed)
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
        """アクセル軸に位置指令を送り電流値を返す（同一バス上で逐次実行）。

        指令位置が前サイクルと同じ軸は書込を省く。ペダル排他制御により**常にどちらか一方の
        軸は開度 0 で固定**なので、毎サイクル片軸ぶんの FC10（≒18ms）が丸ごと不要だった
        （schedule_loop._drive_or_sense / learning_loop と同方式）。
        """
        if pos != self._accel_pos_cmd:
            await self._accel_driver.move_to_position(
                pos, smooth_over_s=self._interval_s * AXIS_SMOOTH_DUTY
            )
            self._accel_pos_cmd = pos
            # 書込直後の読取での初回応答欠落対策（AXIS_PRE_READ_DELAY_S 参照）。
            await asyncio.sleep(AXIS_PRE_READ_DELAY_S)
        return await self._accel_driver.read_current()

    async def _drive_brake_axis(self, pos: int) -> float:
        """ブレーキ軸に位置指令を送り電流値を返す（_drive_accel_axis と同方式）。"""
        if pos != self._brake_pos_cmd:
            await self._brake_driver.move_to_position(
                pos, smooth_over_s=self._interval_s * AXIS_SMOOTH_DUTY
            )
            self._brake_pos_cmd = pos
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
