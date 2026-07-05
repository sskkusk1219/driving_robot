"""2段階学習フロー(学習運転→訓練→PID適合→再学習→PID適合)を1操作で自動進行させるオーケストレータ。

WebUI の「学習サイクル開始」ボタン1操作から、以下を順に自動実行する:
    1. ARMING/LEARNING : 学習運転(開ループパターン走行)
    2. TRAINING_1      : 学習セッションのログで運転モデル訓練 + SIMC初期ゲイン算出
    3. REFINE_1         : 規定パターンで PID 座標降下適合(最大 refine_runs_stage1 回)
    4. TRAINING_2       : サイクル全ログ(学習+適合)で再学習(ゲイン上書きなし)
    5. REFINE_2         : PID 座標降下適合(最大 refine_runs_stage2 回、1段目ゲインから継続)
    6. COMPLETED        : サイクル終了・原点復帰で解放

車両安全の不変条件: 学習運転終了(停車保持)〜2段目適合完了までの全期間、車両は停車保持
ブレーキで静止し続ける。フェーズ境界で保持が切れると転動状態から走行が始まるため、
`release_on_finish=False`（REFINE_1）と各フェーズの解放処理を厳密に守ること。
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
from src.domain.model_training import DEFAULT_FEATURE_SPEC, FeatureSpec
from src.domain.pid_tuning import TuningParams
from src.models.drive_log import DriveLog
from src.models.profile import VehicleProfile
from src.models.system_state import RobotState

_logger = logging.getLogger(__name__)

# 学習運転（開ループパターン走行）完了待ちの既定タイムアウト [s]。
# LearningSettings.learning_timeout_s で上書き可能。
DEFAULT_LEARNING_TIMEOUT_S: float = 600.0


class CyclePhase(StrEnum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    LEARNING = "LEARNING"
    TRAINING_1 = "TRAINING_1"
    REFINE_1 = "REFINE_1"
    TRAINING_2 = "TRAINING_2"
    REFINE_2 = "REFINE_2"
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
    best_preview_time_s: float | None = None
    message: str = ""
    started_at: datetime | None = None


class CycleBusyError(Exception):
    """既にサイクル実行中に別のサイクル開始を試みた場合に送出。"""


class CycleAborted(Exception):
    """abort() 呼び出しによりサイクルを中断した場合に内部で送出する制御フロー例外。"""


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
    ) -> None:
        self._controller = controller
        self._profile_repo = profile_repo
        self._session_repo = session_repo
        self._log_writer = log_writer
        self._learning_timeout_s = learning_timeout_s
        self._progress = CycleProgress(cycle_id=None, phase=CyclePhase.IDLE)
        self._task: asyncio.Task[None] | None = None
        self._abort_requested = False
        self._learning_session_id: str | None = None

    @property
    def progress(self) -> CycleProgress:
        return self._progress

    def _set_progress(self, **kwargs: object) -> None:
        self._progress = replace(self._progress, **kwargs)  # type: ignore[arg-type]

    def _check_abort(self) -> None:
        if self._abort_requested:
            raise CycleAborted

    async def start(
        self,
        profile_id: str,
        refine_runs_stage1: int,
        refine_runs_stage2: int,
        feature_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    ) -> str:
        """学習サイクルを開始する。

        学習運転の準備(arm)と開始は、cycle_id を呼び出し元へ返すためここで同期的に実行する
        （arm の車速収束待ちで数秒〜最大 `_VEHICLE_STOP_TIMEOUT_S` 秒かかる。学習走行自体は
        LearningLoop が非同期に進めるため start_learning_drive() 自体は速やかに返る）。
        以降のフェーズ（学習完了待ち〜訓練〜適合〜再学習〜適合）はバックグラウンドタスクで進行する。

        Returns:
            開設した学習サイクルの UUID 文字列。

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
        await self._controller.arm_learning_drive()
        self._set_progress(phase=CyclePhase.LEARNING, message="学習運転を実行しています")
        session = await self._controller.start_learning_drive(log_writer=self._log_writer)

        cycle_id = session.cycle_id or self._controller.active_cycle_id
        if cycle_id is None:
            raise RuntimeError("学習サイクルIDを採番できませんでした")
        self._learning_session_id = session.id
        self._set_progress(cycle_id=cycle_id)

        self._task = asyncio.create_task(
            self._run(profile_id, refine_runs_stage1, refine_runs_stage2, feature_spec)
        )
        return cycle_id

    async def abort(self) -> None:
        """学習サイクルを中断する。

        チェックポイントはフェーズ境界（各フェーズ処理の直後）と PID 適合の `on_run`
        コールバック（次走行開始前）。走行中（RUNNING）なら `controller.stop()` で
        即座に停止させ、チェックポイント検知を待たずに中断を早める。
        """
        if self._task is None or self._task.done():
            raise InvalidStateTransition("学習サイクルは実行中ではありません")
        self._abort_requested = True
        if self._controller.get_system_state().robot_state == RobotState.RUNNING:
            try:
                await self._controller.stop()
            except InvalidStateTransition:
                pass  # 停止処理と競合した場合は次のチェックポイントに委ねる

    def _make_on_run(self, run_total: int) -> Callable[[int, TuningParams, float], None]:
        best_holder: dict[str, float | None] = {"cost": None, "preview_time_s": None}

        def _on_run(run_index: int, params: TuningParams, cost: float) -> None:
            current_best = best_holder["cost"]
            if current_best is None or cost < current_best:
                best_holder["cost"] = cost
                best_holder["preview_time_s"] = params.preview_time_s
            self._set_progress(
                run_index=run_index,
                run_total=run_total,
                best_cost=best_holder["cost"],
                best_preview_time_s=best_holder["preview_time_s"],
                message=f"PID適合を実行しています（{run_index}/{run_total}回）",
            )
            self._check_abort()

        return _on_run

    async def _persist_best_params(self, profile: VehicleProfile, best: TuningParams) -> None:
        """座標降下の最良ゲイン・先読み補償秒数をプロファイルへ永続化し、制御スタックへ反映する。"""
        profile.pid_gains = best.gains
        profile.dynamics_params = replace(
            profile.dynamics_params, preview_time_s=best.preview_time_s
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
        refine_runs_stage2: int,
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
            detail["stage1_initial_preview_time_s"] = result1.dynamics_params.preview_time_s
            detail["fopdt"] = asdict(result1.dynamics_params)
            self._check_abort()

            # 3. REFINE_1: 規定パターンで PID 座標降下（保持ブレーキは解放しない）
            self._set_progress(
                phase=CyclePhase.REFINE_1,
                run_index=0,
                run_total=refine_runs_stage1,
                message="PID適合(1段目)を実行しています",
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
            detail["stage1_preview_time_s"] = best1.preview_time_s
            detail["stage1_best_cost"] = min((h["cost"] for h in history1), default=None)
            self._check_abort()

            # 4. TRAINING_2: サイクル全ログ（学習+適合）で再学習（ゲイン上書きなし）
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

            # 5. REFINE_2: PID 座標降下（1段目ゲインから継続、完了時に保持ブレーキ解放）
            self._set_progress(
                phase=CyclePhase.REFINE_2,
                run_index=0,
                run_total=refine_runs_stage2,
                message="PID適合(2段目)を実行しています",
            )
            profile2 = await self._profile_repo.get_by_id(profile_id)
            if profile2 is None:
                raise RuntimeError(f"プロファイル {profile_id!r} が見つかりません")
            best2, history2 = await self._controller.run_pid_tuning_session(
                profile2,
                self._log_writer,
                max_runs=refine_runs_stage2,
                release_on_finish=True,
                on_run=self._make_on_run(refine_runs_stage2),
            )
            await self._persist_best_params(profile2, best2)
            detail["stage2_gains"] = asdict(best2.gains)
            detail["stage2_preview_time_s"] = best2.preview_time_s
            stage2_best_cost = min((h["cost"] for h in history2), default=None)
            detail["stage2_best_cost"] = stage2_best_cost

            # 6. COMPLETED
            if self._log_writer is not None:
                await self._log_writer.end_cycle(cycle_id, "completed", detail=detail)
            self._set_progress(
                phase=CyclePhase.COMPLETED,
                message="学習サイクルが完了しました",
                best_cost=stage2_best_cost,
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
