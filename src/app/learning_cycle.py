"""2段階学習フロー(学習運転→訓練→PID適合→再学習→PID適合)を1操作で自動進行させるオーケストレータ。

WebUI の「学習サイクル開始」ボタンから、自動運転と同じ arm→確認ポップアップ→start の
フローで以下を順に自動実行する(arm() で ARMING、確認後の start() で LEARNING 以降へ進む):
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
from src.domain.control.kpi_monitor import (
    KPI_HARD_LIMIT_KMH,
    KPI_P95_LIMIT_KMH,
    KPI_REVERSAL_LIMIT_PER_WINDOW,
)
from src.domain.model_training import DEFAULT_FEATURE_SPEC, FeatureSpec
from src.domain.pid_tuning import (
    TuningParams,
    build_tuning_trajectory_from_mode,
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


class CyclePhase(StrEnum):
    IDLE = "IDLE"
    ARMING = "ARMING"
    LEARNING = "LEARNING"
    TRAINING_1 = "TRAINING_1"
    REFINE_1 = "REFINE_1"
    TRAINING_2 = "TRAINING_2"
    REFINE_2 = "REFINE_2"
    VERIFY = "VERIFY"
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
        tuning_on_target_mode: bool = False,
        verify_runs_max: int = 5,
    ) -> None:
        self._controller = controller
        self._profile_repo = profile_repo
        self._session_repo = session_repo
        self._log_writer = log_writer
        self._learning_timeout_s = learning_timeout_s
        # VERIFY フェーズ（検証専用パターンでの KPI 合格確認）の最大走行本数。mode_repo が
        # 無い（登録モードを列挙できない）場合は VERIFY をスキップし従来どおり REFINE_2 で解放する。
        self._verify_runs_max = max(0, verify_runs_max)
        # REFINE_2 の評価走行を本番モード代表区間で行うための任意依存。mode_repo と
        # tuning_on_target_mode が揃い、arm 時に対象モードが指定された場合のみ有効化する。
        self._mode_repo = mode_repo
        self._tuning_on_target_mode = tuning_on_target_mode
        self._progress = CycleProgress(cycle_id=None, phase=CyclePhase.IDLE)
        self._task: asyncio.Task[None] | None = None
        self._abort_requested = False
        self._learning_session_id: str | None = None
        self._pending_profile_id: str | None = None
        self._target_mode_id: str | None = None

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
        refine_runs_stage2: int,
        feature_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
        target_mode_id: str | None = None,
    ) -> str:
        """arm 済みの学習サイクルを開始する(確認ポップアップ「はい」)。PRE_CHECK → RUNNING。

        学習走行自体は LearningLoop が非同期に進めるため start_learning_drive() 自体は速やかに
        返る。以降のフェーズ（学習完了待ち〜訓練〜適合〜再学習〜適合）はバックグラウンドタスクで
        進行する。

        Returns:
            開設した学習サイクルの UUID 文字列。

        Raises:
            InvalidStateTransition: arm() が未実行(PRE_CHECK でない)の場合
        """
        profile_id = self._pending_profile_id
        if profile_id is None:
            raise InvalidStateTransition("学習サイクルの開始には先に arm() が必要です")
        self._pending_profile_id = None
        self._target_mode_id = target_mode_id

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
            self._run(profile_id, refine_runs_stage1, refine_runs_stage2, feature_spec)
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

    async def _resolve_tuning_mode(self) -> DrivingMode | None:
        """REFINE_2 の評価走行に使うモードを解決する。

        tuning_on_target_mode が有効で mode_repo と対象モードIDが揃うときのみ、本番モードの
        代表区間（build_tuning_trajectory_from_mode）を返す。それ以外は None（呼び出し先が
        従来の規定パターンを使う）。モードが見つからない場合は警告して None にフォールバックし、
        適合自体は止めない。
        """
        if (
            not self._tuning_on_target_mode
            or self._mode_repo is None
            or self._target_mode_id is None
        ):
            return None
        mode = await self._mode_repo.get_by_id(self._target_mode_id)
        if mode is None:
            _logger.warning(
                "対象モード %r が見つからないため規定パターンで PID 適合します",
                self._target_mode_id,
            )
            return None
        return build_tuning_trajectory_from_mode(mode)

    @staticmethod
    def _kpi_passed(kpi: dict[str, float]) -> bool:
        """プライマリー KPI 3 項目（p95≤0.2 / max≤1.0 / 反転≤1）を満たすか。"""
        if kpi.get("n_samples", 0.0) <= 0.0:
            return False
        return (
            kpi.get("p95_kmh", 1e9) <= KPI_P95_LIMIT_KMH
            and kpi.get("max_abs_deviation_kmh", 1e9) <= KPI_HARD_LIMIT_KMH
            and kpi.get("reversal_max_per_5s", 1e9) <= KPI_REVERSAL_LIMIT_PER_WINDOW
        )

    async def _run_verify_phase(
        self, profile_id: str, cycle_id: str, feature_spec: FeatureSpec
    ) -> dict[str, object]:
        """検証専用パターンで KPI 合格を確認する（合格でサイクル完了の前提）。

        登録全モードから検証パターンを生成し、ILC 無効・プラン＋トリムで走行して KPI 判定する。
        合格で早期終了。不合格なら検証走行ログを含む全ログでモデルを再学習してプラン・ゲイン
        スケジュールを再構築し、再走行する（最大 self._verify_runs_max 本）。上限到達で未達なら
        passed=False で返す（走行自体は可能。呼び出し元が WARNING 付きで完了する）。

        mode_repo が無い（登録モードを列挙できない）ときは VERIFY をスキップする。
        """
        if self._mode_repo is None or self._verify_runs_max <= 0:
            return {"skipped": True, "reason": "mode_repo 未配線または verify_runs_max=0"}
        modes = await self._mode_repo.list_all()
        if not modes:
            return {"skipped": True, "reason": "登録モードがありません"}

        runs: list[dict[str, float]] = []
        best_kpi: dict[str, float] | None = None
        passed = False
        for run_index in range(1, self._verify_runs_max + 1):
            self._check_abort()
            profile = await self._profile_repo.get_by_id(profile_id)
            if profile is None:
                raise RuntimeError(f"プロファイル {profile_id!r} が見つかりません")
            verify_mode = build_verification_trajectory(modes, profile)
            self._set_progress(
                phase=CyclePhase.VERIFY,
                run_index=run_index,
                run_total=self._verify_runs_max,
                message=f"検証走行 {run_index}/{self._verify_runs_max} を実行しています",
            )
            kpi = await self._controller.run_verification_drive(
                profile, verify_mode, self._log_writer
            )
            runs.append(
                {
                    "run": float(run_index),
                    "p95_kmh": kpi.get("p95_kmh", 0.0),
                    "max_abs_deviation_kmh": kpi.get("max_abs_deviation_kmh", 0.0),
                    "reversal_max_per_5s": kpi.get("reversal_max_per_5s", 0.0),
                    "pedal_switch_per_min": kpi.get("pedal_switch_per_min", 0.0),
                }
            )
            if best_kpi is None or kpi.get("p95_kmh", 1e9) < best_kpi.get("p95_kmh", 1e9):
                best_kpi = kpi
            kpi_msg = (
                f"検証 {run_index}/{self._verify_runs_max}: "
                f"p95={kpi.get('p95_kmh', 0.0):.2f} "
                f"max={kpi.get('max_abs_deviation_kmh', 0.0):.2f} "
                f"反転={kpi.get('reversal_max_per_5s', 0.0):.0f} "
                f"不要切替={kpi.get('pedal_switch_per_min', 0.0):.1f}/min"
            )
            self._set_progress(
                phase=CyclePhase.VERIFY,
                run_index=run_index,
                run_total=self._verify_runs_max,
                message=kpi_msg,
            )
            if self._kpi_passed(kpi):
                passed = True
                break
            # 不合格: 検証走行ログを含む全サイクルログで再学習し、プラン/スケジュールを更新する。
            if run_index < self._verify_runs_max:
                self._set_progress(
                    phase=CyclePhase.VERIFY,
                    run_index=run_index,
                    run_total=self._verify_runs_max,
                    message=f"検証 {run_index} が KPI 未達。再学習しています",
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
        if not passed:
            _logger.warning(
                "VERIFY: %d 本走行しても KPI 未達（最良 p95=%.3f）",
                len(runs),
                (best_kpi or {}).get("p95_kmh", float("nan")),
            )
        return {"passed": passed, "runs": runs, "best_kpi": best_kpi or {}}

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
            detail["stage1_initial_pid_preview_s"] = result1.dynamics_params.pid_preview_s
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
            detail["stage1_pid_preview_s"] = best1.pid_preview_s
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
            # 本番モードの代表区間で評価すると適合結果の転移性が上がる。対象モードが指定され
            # 有効なときのみ使い、解決不能なら規定パターンへフォールバックする。
            tuning_mode = await self._resolve_tuning_mode()
            # 停車保持を VERIFY へ引き継ぐため release_on_finish=False（VERIFY 後の COMPLETED で
            # 解放する）。VERIFY を実行しない構成でも下の _finalize_release で必ず解放する。
            best2, history2 = await self._controller.run_pid_tuning_session(
                profile2,
                self._log_writer,
                max_runs=refine_runs_stage2,
                release_on_finish=False,
                on_run=self._make_on_run(refine_runs_stage2),
                mode=tuning_mode,
            )
            await self._persist_best_params(profile2, best2)
            detail["stage2_gains"] = asdict(best2.gains)
            detail["stage2_pid_preview_s"] = best2.pid_preview_s
            stage2_best_cost = min((h["cost"] for h in history2), default=None)
            detail["stage2_best_cost"] = stage2_best_cost
            self._check_abort()

            # 6. VERIFY: 検証専用パターンで KPI 合格を確認（合格でサイクル完了＝以降の自動運転は
            #    初回から KPI 満足）。不合格なら再学習＋再走行（上限 verify_runs_max）。
            verify_result = await self._run_verify_phase(profile_id, cycle_id, feature_spec)
            detail["verify"] = verify_result

            # 7. COMPLETED: 停車保持ブレーキを解放して完了
            await self._finalize_release()
            if self._log_writer is not None:
                await self._log_writer.end_cycle(cycle_id, "completed", detail=detail)
            passed = bool(verify_result.get("passed", True))
            self._set_progress(
                phase=CyclePhase.COMPLETED,
                message=(
                    "学習サイクルが完了しました"
                    if passed
                    else "学習サイクルは完了しましたが検証で KPI 未達です（要再サイクル）"
                ),
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
        finally:
            # 完了・中断・エラーのいずれで終わっても controller の参加ポインタをクリアし、
            # 以降の通常走行（auto/manual）が完了済みサイクルの cycle_id を継承して
            # ログ画面でサイクル配下に紛れ込むのを防ぐ。
            self._controller.clear_active_cycle()
