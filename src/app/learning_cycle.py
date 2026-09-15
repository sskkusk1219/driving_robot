"""学習サイクル(学習運転→訓練→PID粗適合→再学習→検証→プラン学習→PID仕上げ)を1操作で
自動進行させるオーケストレータ。

WebUI の「学習サイクル開始」ボタンから、自動運転と同じ arm→確認ポップアップ→start の
フローで以下を順に自動実行する(arm() で ARMING、確認後の start() で LEARNING 以降へ進む):
    1. ARMING/LEARNING : 学習運転(開ループパターン走行)
    2. TRAINING_1      : 学習セッションのログで運転モデル訓練 + SIMC初期ゲイン算出
    3. REFINE_1         : 規定パターンで PID 粗適合(最大 refine_runs_stage1 回)
    4. TRAINING_2       : サイクル全ログ(学習+適合)で再学習(ゲイン上書きなし)＝モデル確定
    5. VERIFY           : 網羅検証パターン(システムモード)で verify_runs 本走行(毎走行後に
                          仕上げ再学習)。KPI は記録するのみでゲートにしない(完走型)
    6. PLAN_LEARN       : 網羅パターンをプラン学習有効・ゲイン固定で反復(KPI合格で即打ち切り、
                          または改善なし2連続で収束打ち切り、無条件実行)
    7. REFINE_FINAL     : 収束プランを凍結して PID 仕上げ座標降下(＝PID を最後に置く、無条件実行)
    8. COMPLETED        : サイクル終了・原点復帰で解放(KPI 未達なら警告付きで完了)

旧 REFINE_2(規定パターンでの2段目適合)は REFINE_FINAL(本番同等の plan+trim 条件での仕上げ)へ
統合・廃止した。

2026-07-14 完走型再編: 旧 VERIFY は KPI 合格まで再学習・再走行を繰り返し、上限到達で不合格なら
PLAN_LEARN/REFINE_FINAL を丸ごとスキップしていた。しかし VERIFY は FF 由来プランのみで走るため
p95 が構造的な床（実機 3.1 前後）に達して飽和し、KPI p95≤0.4 に届かない限り PLAN_LEARN（この床を
下げる唯一の機構）に永遠に入れないデッドロックだった（実機 2026-07-14 で確認）。そのため
VERIFY は「モデル確定フェーズ」（verify_runs 本走行＋毎回仕上げ再学習）に縮小し、
PLAN_LEARN/REFINE_FINAL は KPI 合否によらず必ず実行する。サイクル完了時の KPI 合否は
PLAN_LEARN の最終走行（＝最新の学習済みプランでの実測）を基準に detail/完了メッセージへ記録する。

車両安全の不変条件: 学習運転終了(停車保持)〜PID仕上げ完了までの全期間、車両は停車保持
ブレーキで静止し続ける。フェーズ境界で保持が切れると転動状態から走行が始まるため、
`release_on_finish=False`（REFINE_1/PLAN_LEARN/REFINE_FINAL）と各フェーズの解放処理を厳密に守ること。
"""

from __future__ import annotations

import asyncio
import logging
from collections.abc import Callable
from dataclasses import asdict, dataclass, replace
from datetime import UTC, datetime
from enum import StrEnum
from typing import Protocol

from src.app.robot_controller import (
    InvalidStateTransition,
    LogWriterProtocol,
    RobotController,
)
from src.app.training_service import train_and_apply
from src.domain.control.kpi_monitor import (
    kpi_passed,
)
from src.domain.control.reward import reward_score
from src.domain.model_training import DEFAULT_FEATURE_SPEC, FeatureSpec
from src.domain.pid_tuning import (
    TuningParams,
    build_verification_trajectory,
)
from src.models.drive_log import DriveLog
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile
from src.models.system_state import RobotState

_logger = logging.getLogger(__name__)

# 学習運転（開ループパターン走行）完了待ちの既定タイムアウト [s]。
# LearningSettings.learning_timeout_s で上書き可能。
DEFAULT_LEARNING_TIMEOUT_S: float = 600.0

# PLAN_LEARN の収束判定: p95 改善なし（改善幅 < PLAN_LEARN_P95_EPSILON_KMH）がこの回数
# 連続したら打ち切る。1 発判定は走行間ばらつきで誤発動する: 7/15 実機サイクルでは
# p95 が 3.10→1.69 と単調改善中でも 1 本の悪化だけで「収束」になり得た。
PLAN_LEARN_PATIENCE: int = 2

# best 走行の選択・打ち切り判定は **主 KPI の p95 を主軸**にする（2026-09-09 変更）。
# 旧実装は reward（p95・max 偏差・滑らかさ・切替回数の重み付き和）だけで判定していたが、
# 各項が逆を向くと p95 の改善を「悪化」と誤判定する。実機 3ca20d43 では
# p95 が 2.92→2.22→2.09 と単調改善しているのに reward は run3 で -6.8 悪化し、degrade
# しきい値（epsilon×DEGRADE_FACTOR = 3.0）を超えて 4 本目を打ち切ったうえ、p95 の悪い
# run2（2.22）を最終結果として採用していた。
# best は (KPI合否, -p95, reward) の辞書順で選ぶ＝まず KPI 合否、同じなら p95、それも
# 同じなら reward（滑らかさ等）で決める。
PLAN_LEARN_P95_EPSILON_KMH: float = 0.05  # これ未満の改善は「改善なし」
PLAN_LEARN_P95_DEGRADE_KMH: float = 0.5  # best からこれ以上悪化したら即打ち切り

# reward の明確な悪化判定に使う倍数（p95 が同値のときの補助判定にのみ残す）。
PLAN_LEARN_DEGRADE_FACTOR: float = 3.0


class CyclePhase(StrEnum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    LEARNING = "LEARNING"
    TRAINING_1 = "TRAINING_1"
    REFINE_1 = "REFINE_1"
    TRAINING_2 = "TRAINING_2"
    VERIFY = "VERIFY"
    PLAN_LEARN = "PLAN_LEARN"
    REFINE_FINAL = "REFINE_FINAL"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    ABORTED = "ABORTED"


@dataclass
class CycleProgress:
    """学習サイクルの進捗。WebSocket 経由で UI へ配信する（RealtimeData.cycle_progress）。"""

    cycle_id: str | None
    phase: CyclePhase
    run_index: int = 0
    run_total: int = 0
    best_cost: float | None = None
    best_pid_preview_s: float | None = None
    message: str = ""
    started_at: datetime | None = None


class CycleBusyError(Exception):
    """既にサイクル実行中に別のサイクル開始を試みた場合に送出。"""


class CycleAborted(Exception):
    """abort() 呼び出しによりサイクルを中断した場合に内部で送出する制御フロー例外。"""


def _trajectory_points(mode: DrivingMode) -> list[tuple[float, float]]:
    """基準軌跡を比較可能な (time_s, speed_kmh) タプル列にする（軌跡変化検出用）。"""
    return [(p.time_s, p.speed_kmh) for p in mode.reference_speed]


class ProfileRepoProtocol(Protocol):
    async def get_by_id(self, profile_id: str) -> VehicleProfile | None: ...
    async def update(self, profile: VehicleProfile) -> VehicleProfile | None: ...


class SessionRepoProtocol(Protocol):
    async def list_session_ids_for_cycle(self, cycle_id: str) -> list[str]: ...
    async def list_logs_for_training(
        self,
        profile_id: str,
        session_ids: list[str] | None = None,
        limit: int = 100_000,
    ) -> list[DriveLog]: ...


class ModeRepoProtocol(Protocol):
    async def get_by_id(self, mode_id: str) -> DrivingMode | None: ...
    async def list_all(self) -> list[DrivingMode]: ...
    async def get_system_mode(self) -> DrivingMode | None: ...
    async def upsert_system_mode(self, mode: DrivingMode) -> DrivingMode: ...


class LearningCycleOrchestrator:
    """学習サイクルのフェーズ進行・進捗・中断/エラー処理を担うアプリケーションサービス。"""

    def __init__(
        self,
        controller: RobotController,
        profile_repo: ProfileRepoProtocol,
        session_repo: SessionRepoProtocol,
        log_writer: LogWriterProtocol | None,
        *,
        learning_timeout_s: float = DEFAULT_LEARNING_TIMEOUT_S,
        mode_repo: ModeRepoProtocol | None = None,
        verify_runs: int = 1,
        verify_pattern_budget_s: float = 180.0,
        plan_learn_runs_max: int = 5,
        plan_learn_reward_epsilon: float = 1.0,
        refine_final_runs: int = 3,
    ) -> None:
        self._controller = controller
        self._profile_repo = profile_repo
        self._session_repo = session_repo
        self._log_writer = log_writer
        self._learning_timeout_s = learning_timeout_s
        # VERIFY フェーズ（検証専用パターンでの走行、モデル確定）の走行本数。mode_repo が
        # 無い（登録モードを列挙できない）場合は VERIFY 以降（VERIFY/PLAN_LEARN/REFINE_F）を
        # スキップし REFINE で解放する。各走行後（最終走行後も）仕上げ再学習し、検証走行の
        # 閉ループデータを最終モデルへ吸収する。KPI はここではゲートにしない（完走型、
        # 2026-07-14）。
        self._verify_runs = max(0, verify_runs)
        # 網羅検証パターン（システムモード）の目標長 [s]。VERIFY/PLAN_LEARN/REFINE_F で共用する。
        self._verify_pattern_budget_s = verify_pattern_budget_s
        # PLAN_LEARN フェーズ: 網羅パターンをプラン学習有効・ゲイン固定で最大 N 本走行し、
        # KPI 合格で即打ち切り、または改善なし（改善幅 < epsilon）が PLAN_LEARN_PATIENCE 回
        # 連続で収束打ち切り。0 でフェーズスキップ。
        self._plan_learn_runs_max = max(0, plan_learn_runs_max)
        self._plan_learn_reward_epsilon = plan_learn_reward_epsilon
        # REFINE_F フェーズ: 収束プラン凍結・網羅パターンで少数走行の PID 仕上げ座標降下。
        self._refine_final_runs = max(0, refine_final_runs)
        self._mode_repo = mode_repo
        self._progress = CycleProgress(cycle_id=None, phase=CyclePhase.IDLE)
        self._task: asyncio.Task[None] | None = None
        self._abort_requested = False
        self._learning_session_id: str | None = None
        self._pending_profile_id: str | None = None
        # VERIFY 開始時に生成・永続化する網羅検証パターン（システムモード）。
        # VERIFY/PLAN_LEARN/REFINE_F の全走行で同一軌跡・同一 mode_id を共有する。
        self._system_mode: DrivingMode | None = None

    @property
    def progress(self) -> CycleProgress:
        return self._progress

    def _set_progress(self, **kwargs: object) -> None:
        self._progress = replace(self._progress, **kwargs)  # type: ignore[arg-type]

    def _check_abort(self) -> None:
        if self._abort_requested:
            raise CycleAborted

    async def arm(self, profile_id: str) -> None:
        """学習サイクル開始の準備(arm)。自動運転と同じ arm 手順を実行する。

        停車保持ブレーキ踏込・車速0収束待ち・走行前チェックを行い（`_VEHICLE_STOP_TIMEOUT_S`
        秒かかることがある）、合格したら PRE_CHECK で確認待ちにする。フロントは確認ポップアップ
        を表示し、「はい」で start()、「いいえ」で cancel() を呼ぶ。

        Raises:
            CycleBusyError: 既に学習サイクルが実行中の場合
            ValueError: プロファイルが見つからない場合
            InvalidStateTransition: READY 状態でない等、arm の前提を満たさない場合
            PreCheckFailed: 走行前チェック不合格の場合
        """
        if self._task is not None and not self._task.done():
            raise CycleBusyError("学習サイクルは既に実行中です")
        profile = await self._profile_repo.get_by_id(profile_id)
        if profile is None:
            raise ValueError(f"プロファイル {profile_id!r} が見つかりません")

        self._abort_requested = False
        self._progress = CycleProgress(
            cycle_id=None,
            phase=CyclePhase.ARMING,
            message="学習運転を準備しています",
            started_at=datetime.now(tz=UTC),
        )
        try:
            await self._controller.arm_learning_drive()
        except Exception:
            self._progress = CycleProgress(cycle_id=None, phase=CyclePhase.IDLE)
            raise
        self._pending_profile_id = profile_id

    async def cancel(self) -> None:
        """arm 済み(確認ポップアップ「いいえ」)の学習サイクルを中止する。PRE_CHECK → READY。"""
        self._pending_profile_id = None
        await self._controller.cancel_learning_drive()
        self._progress = CycleProgress(cycle_id=None, phase=CyclePhase.IDLE)

    async def start(
        self,
        refine_runs_stage1: int,
        feature_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    ) -> str:
        """arm 済みの学習サイクルを開始する(確認ポップアップ「はい」)。PRE_CHECK → RUNNING。

        学習走行自体は LearningLoop が非同期に進めるため start_learning_drive() 自体は速やかに
        返る。以降のフェーズ（学習完了待ち〜訓練〜粗適合〜再学習〜検証〜プラン学習〜仕上げ）は
        バックグラウンドタスクで進行する。

        Returns:
            開設した学習サイクルの UUID 文字列。

        Raises:
            InvalidStateTransition: arm() が未実行(PRE_CHECK でない)の場合
        """
        profile_id = self._pending_profile_id
        if profile_id is None:
            raise InvalidStateTransition("学習サイクルの開始には先に arm() が必要です")
        self._pending_profile_id = None

        self._set_progress(phase=CyclePhase.LEARNING, message="学習運転を実行しています")
        try:
            session = await self._controller.start_learning_drive(log_writer=self._log_writer)

            cycle_id = session.cycle_id or self._controller.active_cycle_id
            if cycle_id is None:
                raise RuntimeError("学習サイクルIDを採番できませんでした")
        except Exception:
            # start_learning_drive 失敗時、ロボット状態は READY へロールバック済みだが
            # progress を LEARNING のままにすると WS 配信で「実行中」が永久に見え続ける
            # （W5 レビュー指摘）。ロボット状態のロールバックに合わせて progress も戻す。
            self._progress = CycleProgress(cycle_id=None, phase=CyclePhase.IDLE)
            raise
        self._learning_session_id = session.id
        self._set_progress(cycle_id=cycle_id)

        self._task = asyncio.create_task(
            self._run(profile_id, refine_runs_stage1, feature_spec)
        )
        return cycle_id

    async def abort(self) -> None:
        """学習サイクルを中断する。

        チェックポイントはフェーズ境界（各フェーズ処理の直後）と PID 適合の `on_run`
        コールバック（次走行開始前）。走行中（RUNNING）なら `controller.stop()` で
        即座に停止させ、チェックポイント検知を待たずに中断を早める。

        アーミング中（arm() 済みで start() 未実行、_task 未生成）の abort() は
        cancel() と同じ「アーム中断」として扱う（409 にせず、保持ブレーキ解放と
        進捗リセットを行う）。arm() 完了後の確認待ち中に abort() が呼ばれても
        中断できるようにする（W5 レビュー指摘）。
        """
        if self._task is None or self._task.done():
            if self._progress.phase == CyclePhase.ARMING:
                await self.cancel()
                return
            raise InvalidStateTransition("学習サイクルは実行中ではありません")
        self._abort_requested = True
        if self._controller.get_system_state().robot_state == RobotState.RUNNING:
            try:
                await self._controller.stop()
            except InvalidStateTransition:
                pass  # 停止処理と競合した場合は次のチェックポイントに委ねる

    def _make_on_run(self, run_total: int) -> Callable[[int, TuningParams, float], None]:
        best_holder: dict[str, float | None] = {"cost": None, "pid_preview_s": None}

        def _on_run(run_index: int, params: TuningParams, cost: float) -> None:
            current_best = best_holder["cost"]
            if current_best is None or cost < current_best:
                best_holder["cost"] = cost
                best_holder["pid_preview_s"] = params.pid_preview_s
            self._set_progress(
                run_index=run_index,
                run_total=run_total,
                best_cost=best_holder["cost"],
                best_pid_preview_s=best_holder["pid_preview_s"],
                message=f"PID適合を実行しています（{run_index}/{run_total}回）",
            )
            self._check_abort()

        return _on_run

    @staticmethod
    def _kpi_passed(kpi: dict[str, float]) -> bool:
        """プライマリー KPI 3 項目の合否。判定はドメイン層（kpi_monitor.kpi_passed）に委譲する。

        ILC の採否（PedalPlanService._decide_outcome）と**同じ判定**を使うため、
        しきい値の突き合わせをここに二重に書かない。
        """
        return kpi_passed(kpi)

    async def _prepare_system_mode(self, profile_id: str) -> DrivingMode | None:
        """網羅検証パターン（システムモード）を生成・永続化して返す。

        登録全モードの包絡を約 verify_pattern_budget_s に合成し、予約名 `__verify_pattern__` で
        upsert する。id は世代をまたいで安定するため、pedal_plans（プラン学習の保存先）の FK が
        有効に保たれる。VERIFY/PLAN_LEARN/REFINE_F の全走行がこの単一の永続モードを共有し、
        軌跡・mode_id の一貫性を保証する。mode_repo 未配線・登録モードなしなら None（呼び出し元が
        VERIFY 以降をスキップ）。

        モデル更新はプランを引き継ぐ（PedalPlanService、2026-07-16 プラン引き継ぎ）が、
        **軌跡の変更は引き継げない**: 登録モードの追加・編集で包絡が前世代から変わった場合、
        旧軌跡で獲得した best_reward にロールバック機構が固着するため（新軌跡の走行では
        旧 best を超えられない）、システムモードの保存プランを全プロファイルぶんリセット
        してから PLAN_LEARN に入る。
        """
        if self._mode_repo is None:
            return None
        modes = await self._mode_repo.list_all()
        if not modes:
            return None
        profile = await self._profile_repo.get_by_id(profile_id)
        if profile is None:
            raise RuntimeError(f"プロファイル {profile_id!r} が見つかりません")
        pattern = build_verification_trajectory(
            modes, profile, budget_s=self._verify_pattern_budget_s
        )
        prev = await self._mode_repo.get_system_mode()
        system_mode = await self._mode_repo.upsert_system_mode(pattern)
        if prev is not None and _trajectory_points(prev) != _trajectory_points(system_mode):
            _logger.info(
                "網羅パターンの軌跡が前世代から変化: モード %s の保存プランをリセット",
                system_mode.id,
            )
            await self._controller.reset_saved_plans_for_mode(system_mode.id)
        return system_mode

    async def _run_verify_phase(
        self, profile_id: str, cycle_id: str, feature_spec: FeatureSpec
    ) -> dict[str, object]:
        """検証専用パターン（システムモード）でモデルを確定する（完走型・KPI はゲートにしない）。

        VERIFY 開始時に生成した網羅パターン（self._system_mode）を保存プランなし（FF 由来
        プラン）＋トリムで self._verify_runs 本走行し、**毎走行後（最終走行後も）**サイクル
        全ログで仕上げ再学習する（検証走行＝本番同分布の閉ループデータを最終モデルへ吸収し、
        初回自動運転のトリム寄与を下げる）。KPI は記録するのみで合否によるリトライ・スキップは
        行わない。旧実装は KPI 合格まで再学習・再走行を繰り返し上限到達で不合格なら以降の
        PLAN_LEARN/REFINE_FINAL を丸ごとスキップしていたが、VERIFY は FF 由来プランのみで
        走るため p95 が構造的な床（実機 3.1 前後）に達して飽和し、PLAN_LEARN（この床を下げる
        唯一の機構）に永遠に入れないデッドロックだった（2026-07-14 実機で確認）。

        システムモード未生成（mode_repo 未配線・登録モードなし）のときは VERIFY をスキップする。
        """
        if self._system_mode is None or self._verify_runs <= 0:
            return {"skipped": True, "reason": "システムモード未生成または verify_runs=0"}
        system_mode = self._system_mode

        runs: list[dict[str, float]] = []
        final_kpi: dict[str, float] = {}
        for run_index in range(1, self._verify_runs + 1):
            self._check_abort()
            profile = await self._profile_repo.get_by_id(profile_id)
            if profile is None:
                raise RuntimeError(f"プロファイル {profile_id!r} が見つかりません")
            self._set_progress(
                phase=CyclePhase.VERIFY,
                run_index=run_index,
                run_total=self._verify_runs,
                message=f"検証走行 {run_index}/{self._verify_runs} を実行しています",
            )
            kpi = await self._controller.run_verification_drive(
                profile, system_mode, self._log_writer
            )
            final_kpi = kpi
            reward = reward_score(kpi)
            runs.append(
                {
                    "run": float(run_index),
                    "p95_kmh": kpi.get("p95_kmh", 0.0),
                    "max_abs_deviation_kmh": kpi.get("max_abs_deviation_kmh", 0.0),
                    "reversal_max_per_5s": kpi.get("reversal_max_per_5s", 0.0),
                    "pedal_switch_per_min": kpi.get("pedal_switch_per_min", 0.0),
                    "reward": reward,
                    "trim_share": kpi.get("trim_share", 0.0),
                }
            )
            kpi_msg = (
                f"検証 {run_index}/{self._verify_runs}: "
                f"p95={kpi.get('p95_kmh', 0.0):.2f} "
                f"max={kpi.get('max_abs_deviation_kmh', 0.0):.2f} "
                f"反転={kpi.get('reversal_max_per_5s', 0.0):.0f} "
                f"不要切替={kpi.get('pedal_switch_per_min', 0.0):.1f}/min "
                f"reward={reward:+.3f}"
            )
            self._set_progress(
                phase=CyclePhase.VERIFY,
                run_index=run_index,
                run_total=self._verify_runs,
                message=kpi_msg,
            )
            # 仕上げ再学習: 検証走行ログを含む全サイクルログでモデルを再学習し、プラン/
            # ゲインスケジュールを更新する。最終走行後も行い、その閉ループデータを吸収する。
            self._set_progress(
                phase=CyclePhase.VERIFY,
                run_index=run_index,
                run_total=self._verify_runs,
                message=f"検証 {run_index} 完了。仕上げ再学習しています",
            )
            cycle_session_ids = await self._session_repo.list_session_ids_for_cycle(cycle_id)
            await train_and_apply(
                profile_repo=self._profile_repo,
                session_repo=self._session_repo,
                controller=self._controller,
                profile_id=profile_id,
                session_ids=cycle_session_ids,
                update_pid_gains=False,
                feature_spec=feature_spec,
            )
        kpi_passed = self._kpi_passed(final_kpi)
        if not kpi_passed:
            _logger.warning(
                "VERIFY: KPI 未達のまま完走型フローを継続します（p95=%.3f, max=%.3f）",
                final_kpi.get("p95_kmh", float("nan")),
                final_kpi.get("max_abs_deviation_kmh", float("nan")),
            )
        return {"runs": runs, "final_kpi": final_kpi, "kpi_passed": kpi_passed}

    def _plan_learn_run_key(
        self, kpi: dict[str, float], reward: float
    ) -> tuple[bool, float, float]:
        """PLAN_LEARN の走行比較キー（大きいほど良い）: (KPI合否, -p95, reward) の辞書順。

        まず主 KPI の合否、同じなら p95（小さいほど良いので符号反転）、それも同じなら
        reward（滑らかさ・切替回数を含む総合指標）で決める。reward 単独で比較すると、
        各項が逆を向いたときに p95 の改善を「悪化」と誤判定する（実機 3ca20d43）。
        """
        p95 = float(kpi.get("p95_kmh", float("inf")))
        return (self._kpi_passed(kpi), -p95, reward)

    def _plan_learn_improved(
        self,
        p95: float,
        reward: float,
        best_p95: float | None,
        best_reward: float | None,
    ) -> bool:
        """この走行が最良を意味のある幅で更新したか（収束打ち切りのカウント用）。

        主軸は p95（PLAN_LEARN_P95_EPSILON_KMH 以上の改善）。p95 が横ばい
        （±epsilon 以内）でも reward が reward_epsilon 以上改善していれば「学習は進んで
        いる」とみなす——p95 は主 KPI だが、滑らかさ・最大偏差・切替回数が改善している
        局面で打ち切るのは早すぎるため。
        """
        if best_p95 is None or best_reward is None:
            return True
        if best_p95 - p95 >= PLAN_LEARN_P95_EPSILON_KMH:
            return True
        if (
            p95 <= best_p95 + PLAN_LEARN_P95_EPSILON_KMH
            and reward - best_reward >= self._plan_learn_reward_epsilon
        ):
            return True
        return False

    async def _run_plan_learn_phase(self, profile_id: str) -> dict[str, object]:
        """網羅パターン（システムモード）をプラン学習有効・ゲイン固定で反復走行する。

        VERIFY（モデル確定）後に KPI 合否によらず無条件で実行する（2026-07-14: 完走型）。
        各走行で保存プランを prepare→走行→update_from_session を await 完了し、次走行が
        更新後プランを拾う。KPI 合格で即打ち切り、明確な悪化（p95 が best から
        PLAN_LEARN_P95_DEGRADE_KMH 以上悪化）で即打ち切り、または改善なし（p95 改善幅 <
        PLAN_LEARN_P95_EPSILON_KMH）が PLAN_LEARN_PATIENCE 回連続で収束打ち切り、最大
        plan_learn_runs_max 本で終了。

        **最終結果は最良走行を基準にする**（2026-09-08 修正）。旧実装は最終走行（last）の
        KPI をそのまま採用しており、ACCEPT/EXPLORE 後の候補プラン（次回探索用）は
        best_efforts と一致しないため、反復が悪化方向に振れると「最良を見つけたのに最終的に
        それより悪い状態でサイクルが完了する」ことがあった（実機 2026-09-07: p95
        1.74→2.35→3.34 と悪化して完了）。ループ終了後に保存プランを最良へ明示的に確定する
        （freeze_saved_plan_to_best）。

        **最良の選択・打ち切りは (KPI合否, -p95, reward) の辞書順**（2026-09-09 修正）。
        reward 単独では各項が逆を向いたときに p95 の改善を「悪化」と誤判定する
        （PLAN_LEARN_P95_EPSILON_KMH のコメント参照）。

        プランは軌跡固有のためここで学ぶのは網羅パターン専用プランだが、これにより
        (a) 続く REFINE_F を本番同等の plan+trim 条件で行える (b) プラン学習機構の収束を
        本番前にダイナモで確認できる。システムモード未生成・runs_max=0 ならスキップ。
        """
        if self._system_mode is None or self._plan_learn_runs_max <= 0:
            return {"skipped": True, "reason": "システムモード未生成または plan_learn_runs_max=0"}
        system_mode = self._system_mode

        runs: list[dict[str, float]] = []
        best_key: tuple[bool, float, float] | None = None
        best_reward: float | None = None
        best_p95: float | None = None
        best_kpi: dict[str, float] = {}
        best_run_index: int | None = None
        converged = False
        degraded = False
        no_improve_streak = 0
        kpi: dict[str, float] = {}
        for run_index in range(1, self._plan_learn_runs_max + 1):
            self._check_abort()
            profile = await self._profile_repo.get_by_id(profile_id)
            if profile is None:
                raise RuntimeError(f"プロファイル {profile_id!r} が見つかりません")
            self._set_progress(
                phase=CyclePhase.PLAN_LEARN,
                run_index=run_index,
                run_total=self._plan_learn_runs_max,
                message=f"プラン学習 {run_index}/{self._plan_learn_runs_max} を実行しています",
            )
            kpi = await self._controller.run_plan_learning_drive(
                profile, system_mode, self._log_writer
            )
            reward = reward_score(kpi)
            runs.append(
                {
                    "run": float(run_index),
                    "p95_kmh": kpi.get("p95_kmh", 0.0),
                    "max_abs_deviation_kmh": kpi.get("max_abs_deviation_kmh", 0.0),
                    "pedal_switch_per_min": kpi.get("pedal_switch_per_min", 0.0),
                    "reward": reward,
                    "trim_share": kpi.get("trim_share", 0.0),
                }
            )
            self._set_progress(
                phase=CyclePhase.PLAN_LEARN,
                run_index=run_index,
                run_total=self._plan_learn_runs_max,
                message=(
                    f"プラン学習 {run_index}/{self._plan_learn_runs_max}: "
                    f"p95={kpi.get('p95_kmh', 0.0):.2f} "
                    f"不要切替={kpi.get('pedal_switch_per_min', 0.0):.1f}/min "
                    f"reward={reward:+.3f} trim寄与={kpi.get('trim_share', 0.0):.3f}"
                ),
            )
            # 早期打ち切り: KPI 合格で即終了。明確な悪化（best から p95 が大幅悪化）でも
            # PATIENCE を待たず即終了。収束（p95 改善幅 < epsilon）は PLAN_LEARN_PATIENCE
            # 回連続したときのみ（小さな悪化はばらつきとみなし継続する）。
            p95 = float(kpi.get("p95_kmh", float("inf")))
            key = self._plan_learn_run_key(kpi, reward)
            improved = self._plan_learn_improved(p95, reward, best_p95, best_reward)
            is_degrade = best_p95 is not None and p95 > best_p95 + PLAN_LEARN_P95_DEGRADE_KMH
            if best_key is None or key > best_key:
                best_key = key
                best_reward = reward
                best_p95 = p95
                best_kpi = kpi
                best_run_index = run_index
            if self._kpi_passed(kpi):
                converged = True
                break
            if is_degrade:
                degraded = True
                break
            if not improved:
                no_improve_streak += 1
                if no_improve_streak >= PLAN_LEARN_PATIENCE:
                    converged = True
                    break
            else:
                no_improve_streak = 0

        # ループ終了後、保存プランを最良（best_efforts）へ明示的に確定する。ACCEPT/EXPLORE
        # 走行後の候補プラン（次回探索用）が best と一致しない状態のまま抜けても、続く
        # REFINE_FINAL・本番走行は必ず最良プランを使う。
        profile = await self._profile_repo.get_by_id(profile_id)
        if profile is not None:
            await self._controller.freeze_saved_plan_to_best(profile, system_mode)

        final_kpi = best_kpi if best_kpi else kpi
        return {
            "runs": runs,
            "converged": converged,
            "degraded": degraded,
            "best_reward": best_reward,
            "best_p95": best_p95,
            "best_run": best_run_index,
            "kpi_passed": self._kpi_passed(final_kpi),
            "final_kpi": final_kpi,
        }

    async def _run_refine_final_phase(self, profile_id: str) -> dict[str, object]:
        """REFINE_F（PID 仕上げ）: PLAN_LEARN の収束プランを凍結し、網羅パターンで少数走行の
        座標降下を行って最良ゲインをプロファイルへ永続化する（＝PID 適合を最後に置く）。

        全候補走行を同一の凍結プランで走るため座標降下のゲイン候補間比較（同一条件比較）が
        成立する。ゲイン変更後もモデルは不変なので保存プランは有効のまま。停車保持は
        release_on_finish=False で COMPLETED まで引き継ぐ。システムモード未生成・
        refine_final_runs=0 ならスキップ。
        """
        if self._system_mode is None or self._refine_final_runs <= 0:
            return {"skipped": True, "reason": "システムモード未生成または refine_final_runs=0"}
        system_mode = self._system_mode
        profile = await self._profile_repo.get_by_id(profile_id)
        if profile is None:
            raise RuntimeError(f"プロファイル {profile_id!r} が見つかりません")
        # PLAN_LEARN の収束プランを凍結（未保存なら None で FF 由来へフォールバック）。
        frozen_plan = await self._controller.get_saved_plan(profile, system_mode)
        self._set_progress(
            phase=CyclePhase.REFINE_FINAL,
            run_index=0,
            run_total=self._refine_final_runs,
            message="PID仕上げ（プラン凍結）を実行しています",
        )
        best, history = await self._controller.run_pid_tuning_session(
            profile,
            self._log_writer,
            max_runs=self._refine_final_runs,
            release_on_finish=False,
            on_run=self._make_on_run(self._refine_final_runs),
            mode=system_mode,
            plan=frozen_plan,
        )
        await self._persist_best_params(profile, best)
        best_cost = min((h["cost"] for h in history), default=None)
        return {
            "gains": asdict(best.gains),
            "pid_preview_s": best.pid_preview_s,
            "best_cost": best_cost,
            "frozen_plan": frozen_plan is not None,
        }

    async def _finalize_release(self) -> None:
        """VERIFY 後に停車保持ブレーキを解放する（READY で停車保持中の冪等操作）。"""
        try:
            await self._controller.release_stop_hold()
        except Exception:
            _logger.exception("学習サイクル完了処理: 停車保持ブレーキの解放に失敗しました")

    async def _persist_best_params(self, profile: VehicleProfile, best: TuningParams) -> None:
        """座標降下の最良ゲイン・PID先読み補償をプロファイルへ永続化し制御スタックへ反映する。"""
        profile.pid_gains = best.gains
        profile.dynamics_params = replace(
            profile.dynamics_params, pid_preview_s=best.pid_preview_s
        )
        updated = await self._profile_repo.update(profile)
        self._controller.refresh_active_profile(updated if updated is not None else profile)

    async def _release_and_stop_if_needed(self) -> None:
        """安全網: 走行中なら停止、そうでなければ停車保持ブレーキを解放する（冪等）。"""
        state = self._controller.get_system_state().robot_state
        if state == RobotState.RUNNING:
            try:
                await self._controller.stop()
                return
            except InvalidStateTransition:
                pass
        try:
            await self._controller.release_stop_hold()
        except Exception:
            _logger.exception("学習サイクル終了処理: 停車保持ブレーキの解放に失敗しました")

    async def _run(
        self,
        profile_id: str,
        refine_runs_stage1: int,
        feature_spec: FeatureSpec,
    ) -> None:
        cycle_id = self._progress.cycle_id
        assert cycle_id is not None
        detail: dict[str, object] = {}
        try:
            # 1. 学習運転の完了待ち
            try:
                await asyncio.wait_for(
                    self._controller._learning_complete.wait(),  # noqa: SLF001
                    timeout=self._learning_timeout_s,
                )
            except TimeoutError as e:
                if self._controller.get_system_state().robot_state == RobotState.RUNNING:
                    try:
                        await self._controller.stop()
                    except InvalidStateTransition:
                        pass
                raise RuntimeError("学習運転がタイムアウトしました") from e
            self._check_abort()
            if self._learning_session_id is None:
                raise RuntimeError("学習セッションIDを取得できませんでした")

            # 2. TRAINING_1: 学習セッションのログで訓練 + SIMC初期ゲイン
            self._set_progress(phase=CyclePhase.TRAINING_1, message="運転モデルを学習しています")
            result1 = await train_and_apply(
                profile_repo=self._profile_repo,
                session_repo=self._session_repo,
                controller=self._controller,
                profile_id=profile_id,
                session_ids=[self._learning_session_id],
                update_pid_gains=True,
                feature_spec=feature_spec,
            )
            detail["stage1_model_path"] = result1.model_path
            detail["stage1_metrics"] = result1.metrics
            detail["stage1_initial_gains"] = asdict(result1.pid_gains)
            detail["stage1_initial_pid_preview_s"] = result1.dynamics_params.pid_preview_s
            detail["fopdt"] = asdict(result1.dynamics_params)
            self._check_abort()

            # 3. REFINE_1: 規定パターンで PID 粗適合（保持ブレーキは解放しない）
            self._set_progress(
                phase=CyclePhase.REFINE_1,
                run_index=0,
                run_total=refine_runs_stage1,
                message="PID粗適合を実行しています",
            )
            profile1 = await self._profile_repo.get_by_id(profile_id)
            if profile1 is None:
                raise RuntimeError(f"プロファイル {profile_id!r} が見つかりません")
            best1, history1 = await self._controller.run_pid_tuning_session(
                profile1,
                self._log_writer,
                max_runs=refine_runs_stage1,
                release_on_finish=False,
                on_run=self._make_on_run(refine_runs_stage1),
            )
            await self._persist_best_params(profile1, best1)
            detail["stage1_gains"] = asdict(best1.gains)
            detail["stage1_pid_preview_s"] = best1.pid_preview_s
            detail["stage1_best_cost"] = min((h["cost"] for h in history1), default=None)
            self._check_abort()

            # 4. TRAINING_2: サイクル全ログ（学習+粗適合）で再学習（ゲイン上書きなし）。
            #    以降 VERIFY 内の仕上げ再学習が最後の再学習で、PLAN_LEARN/REFINE_F は
            #    再学習しないため model_path はそこで最終値に固定される。
            self._set_progress(
                phase=CyclePhase.TRAINING_2, message="サイクル全ログで再学習しています"
            )
            cycle_session_ids = await self._session_repo.list_session_ids_for_cycle(cycle_id)
            result2 = await train_and_apply(
                profile_repo=self._profile_repo,
                session_repo=self._session_repo,
                controller=self._controller,
                profile_id=profile_id,
                session_ids=cycle_session_ids,
                update_pid_gains=False,
                feature_spec=feature_spec,
            )
            detail["stage2_model_path"] = result2.model_path
            detail["stage2_metrics"] = result2.metrics
            self._check_abort()
            # 完了メッセージ用の最良コスト。REFINE_F 実行時にその best_cost で上書きする。
            final_best_cost: float | None = detail.get("stage1_best_cost")  # type: ignore[assignment]

            # 6. VERIFY: 網羅検証パターン（システムモード）で verify_runs 本走行し、モデルを
            #    確定する（毎走行後に仕上げ再学習）。KPI は記録のみでゲートにしない（完走型）。
            #    システムモードは VERIFY/PLAN_LEARN/REFINE_F で共有するため一度だけ生成する。
            self._system_mode = await self._prepare_system_mode(profile_id)
            verify_result = await self._run_verify_phase(profile_id, cycle_id, feature_spec)
            detail["verify"] = verify_result
            self._check_abort()

            # 7. PLAN_LEARN: KPI 合否によらず無条件で実行し、網羅パターンをプラン学習有効・
            #    ゲイン固定で反復して REFINE_F を本番同等の plan+trim 条件へ整える。
            plan_learn_result = await self._run_plan_learn_phase(profile_id)
            detail["plan_learn"] = plan_learn_result
            self._check_abort()

            # 8. REFINE_F: 収束プランを凍結して PID 仕上げ座標降下（PID を最後に置く）。
            #    これも KPI 合否によらず無条件で実行する。
            refine_final_result = await self._run_refine_final_phase(profile_id)
            detail["refine_final"] = refine_final_result
            # REFINE_F の最良コストを完了メッセージに反映する。
            rf_cost = refine_final_result.get("best_cost")
            if isinstance(rf_cost, int | float):
                final_best_cost = float(rf_cost)
            self._check_abort()

            # サイクル全体の KPI 合否は PLAN_LEARN 最終走行（＝最新の学習済みプランでの実測）を
            # 基準にする。PLAN_LEARN がスキップされた場合（システムモード未生成等）は VERIFY の
            # 結果にフォールバックする。
            final_kpi: dict[str, float]
            if "kpi_passed" in plan_learn_result:
                kpi_passed = bool(plan_learn_result["kpi_passed"])
                final_kpi = plan_learn_result.get("final_kpi") or {}  # type: ignore[assignment]
            else:
                kpi_passed = bool(verify_result.get("kpi_passed", True))
                final_kpi = verify_result.get("final_kpi") or {}  # type: ignore[assignment]
            detail["kpi_passed"] = kpi_passed
            detail["final_kpi"] = final_kpi

            # 9. COMPLETED: 停車保持ブレーキを解放して完了
            await self._finalize_release()
            if self._log_writer is not None:
                await self._log_writer.end_cycle(cycle_id, "completed", detail=detail)
            if kpi_passed:
                completion_message = "学習サイクルが完了しました（KPI 合格）"
            else:
                p95 = final_kpi.get("p95_kmh", float("nan"))
                max_dev = final_kpi.get("max_abs_deviation_kmh", float("nan"))
                completion_message = (
                    f"学習サイクルは完了しました"
                    f"（警告: KPI 未達 p95={p95:.2f} / max={max_dev:.2f}）"
                )
            self._set_progress(
                phase=CyclePhase.COMPLETED,
                message=completion_message,
                best_cost=final_best_cost,
            )
        except CycleAborted:
            _logger.info("学習サイクル %s を中断しました", cycle_id)
            await self._release_and_stop_if_needed()
            if self._log_writer is not None:
                await self._log_writer.end_cycle(cycle_id, "aborted", detail=detail)
            self._set_progress(phase=CyclePhase.ABORTED, message="学習サイクルを中断しました")
        except Exception as e:
            _logger.exception("学習サイクル %s でエラーが発生しました", cycle_id)
            await self._release_and_stop_if_needed()
            detail = {**detail, "error": str(e), "phase": self._progress.phase.value}
            if self._log_writer is not None:
                await self._log_writer.end_cycle(cycle_id, "error", detail=detail)
            self._set_progress(phase=CyclePhase.ERROR, message=f"エラーが発生しました: {e}")
        finally:
            # 完了・中断・エラーのいずれで終わっても controller の参加ポインタをクリアし、
            # 以降の通常走行（auto/manual）が完了済みサイクルの cycle_id を継承して
            # ログ画面でサイクル配下に紛れ込むのを防ぐ。
            self._controller.clear_active_cycle()
