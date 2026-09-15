"""LearningCycleOrchestrator のユニットテスト。"""

import asyncio
from dataclasses import replace
from datetime import UTC, datetime

import pytest

from src.app.learning_cycle import (
    CycleAborted,
    CycleBusyError,
    CyclePhase,
    LearningCycleOrchestrator,
)
from src.app.robot_controller import InvalidStateTransition
from src.app.training_service import TrainResult
from src.domain.pid_tuning import TuningParams
from src.models.drive_log import DriveSession
from src.models.profile import (
    DynamicsParams,
    FeedforwardParams,
    PIDGains,
    StopConfig,
    VehicleProfile,
)
from src.models.system_state import RobotState, SystemState


def make_profile(pid: str = "p1") -> VehicleProfile:
    return VehicleProfile(
        id=pid,
        name="t",
        max_accel_opening=80.0,
        max_brake_opening=80.0,
        max_speed=120.0,
        max_decel_g=0.4,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None,
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )


class FakeProfileRepo:
    def __init__(self, profile: VehicleProfile | None) -> None:
        self._profile = profile
        self.updates: list[VehicleProfile] = []

    async def get_by_id(self, profile_id: str) -> VehicleProfile | None:  # noqa: ARG002
        return self._profile

    async def update(self, profile: VehicleProfile) -> VehicleProfile | None:
        self.updates.append(profile)
        self._profile = profile
        return profile


class FakeSessionRepo:
    def __init__(self, cycle_session_ids: list[str] | None = None) -> None:
        self._cycle_session_ids = cycle_session_ids or ["learn-sess-1", "tune-sess-1"]

    async def list_session_ids_for_cycle(self, cycle_id: str) -> list[str]:  # noqa: ARG002
        return self._cycle_session_ids

    async def list_logs_for_training(
        self,
        profile_id: str,  # noqa: ARG002
        session_ids: list[str] | None = None,  # noqa: ARG002
        limit: int = 100_000,  # noqa: ARG002
    ) -> list:
        return []


class FakeLogWriter:
    def __init__(self) -> None:
        self.ended_cycles: list[tuple[str, str, dict]] = []

    async def start_cycle(self, profile_id: str) -> str:  # noqa: ARG002
        return "cycle-1"

    async def end_cycle(self, cycle_id: str, status: str, detail: dict | None = None) -> None:
        self.ended_cycles.append((cycle_id, status, detail or {}))


class FakeController:
    """LearningCycleOrchestrator が依存する RobotController のごく一部を模擬する。"""

    def __init__(self) -> None:
        self._learning_complete = asyncio.Event()
        self.state = RobotState.READY
        self.stop_called = False
        self.release_called = False
        self.refreshed_profiles: list[VehicleProfile] = []
        self.tuning_calls: list[dict[str, object]] = []
        self._session_counter = 0
        self.fail_start_learning_drive = False
        # VERIFY 用: run_verification_drive が返す KPI の列（1本ずつ pop）。既定は 1 本合格。
        self.verify_kpis: list[dict[str, float]] = [
            {"n_samples": 100.0, "p95_kmh": 0.1, "max_abs_deviation_kmh": 0.5,
             "reversal_max_per_5s": 0.0}
        ]
        self.verify_calls = 0
        self.clear_active_cycle_called = False
        # PLAN_LEARN 用: run_plan_learning_drive が返す KPI の列（1本ずつ pop、最後は据え置き）。
        # 既定は KPI 合格・p95 一定（reward 改善が飽和 → 2本目で早期打ち切りになる系列）。
        self.plan_learn_kpis: list[dict[str, float]] = [
            {"n_samples": 100.0, "p95_kmh": 0.1, "max_abs_deviation_kmh": 0.5,
             "reversal_max_per_5s": 0.0}
        ]
        self.plan_learn_calls = 0
        # REFINE_F 用: get_saved_plan が返す凍結プラン（sentinel）。既定 None。
        self.saved_plan: object | None = None
        # 網羅パターンの軌跡変化時に呼ばれる reset_saved_plans_for_mode の記録。
        self.reset_plan_mode_ids: list[str] = []
        # PLAN_LEARN フェーズ終了時に呼ばれる freeze_saved_plan_to_best の呼び出し回数。
        self.freeze_to_best_calls = 0

    async def arm_learning_drive(self) -> None:
        pass

    async def cancel_learning_drive(self) -> None:
        self.release_called = True

    async def start_learning_drive(self, log_writer: object = None) -> DriveSession:  # noqa: ARG002
        if self.fail_start_learning_drive:
            # 実際の RobotController.start_learning_drive は失敗時に READY へロール
            # バックしてから例外を送出する。
            self.state = RobotState.READY
            raise InvalidStateTransition("学習運転に必要な構成が不足しています")
        self._session_counter += 1
        self.state = RobotState.RUNNING
        return DriveSession(
            id=f"learn-sess-{self._session_counter}",
            profile_id="p1",
            mode_id=None,
            run_type="learning",
            started_at=datetime.now(tz=UTC),
            ended_at=None,
            status="running",
            cycle_id="cycle-1",
        )

    @property
    def active_cycle_id(self) -> str | None:
        return "cycle-1"

    def clear_active_cycle(self) -> None:
        self.clear_active_cycle_called = True

    def get_system_state(self) -> SystemState:
        return SystemState(
            robot_state=self.state,
            active_profile_id="p1",
            active_session_id=None,
            last_normal_shutdown=True,
            updated_at=datetime.now(tz=UTC),
        )

    async def stop(self) -> None:
        if self.state != RobotState.RUNNING:
            raise InvalidStateTransition("not running")
        self.stop_called = True
        self.state = RobotState.READY
        self._learning_complete.set()

    async def release_stop_hold(self) -> None:
        self.release_called = True

    def refresh_active_profile(self, profile: VehicleProfile) -> bool:
        self.refreshed_profiles.append(profile)
        return True

    async def run_pid_tuning_session(
        self,
        profile: VehicleProfile,
        log_writer: object,  # noqa: ARG002
        max_runs: int,
        *,
        release_on_finish: bool = True,
        on_run: object = None,
        mode: object = None,
        plan: object = None,
    ) -> tuple[TuningParams, list[dict]]:
        self.tuning_calls.append(
            {
                "max_runs": max_runs,
                "release_on_finish": release_on_finish,
                "mode": mode,
                "plan": plan,
            }
        )
        best = TuningParams.from_profile(profile)
        history = []
        for i in range(1, max_runs + 1):
            await asyncio.sleep(0)
            cost = 1.0 / i
            best = replace(best, kp=best.kp + 0.1, pid_preview_s=best.pid_preview_s + 0.2)
            if on_run is not None:
                on_run(i, best, cost)  # type: ignore[operator]
            history.append(
                {
                    "kp": best.kp,
                    "ki": best.ki,
                    "kd": best.kd,
                    "pid_preview_s": best.pid_preview_s,
                    "cost": cost,
                }
            )
        return best, history

    async def run_verification_drive(
        self,
        profile: VehicleProfile,  # noqa: ARG002
        mode: object,  # noqa: ARG002
        log_writer: object = None,  # noqa: ARG002
    ) -> dict[str, float]:
        self.verify_calls += 1
        if len(self.verify_kpis) > 1:
            return self.verify_kpis.pop(0)
        return self.verify_kpis[0]

    async def run_plan_learning_drive(
        self,
        profile: VehicleProfile,  # noqa: ARG002
        mode: object,  # noqa: ARG002
        log_writer: object = None,  # noqa: ARG002
    ) -> dict[str, float]:
        self.plan_learn_calls += 1
        if len(self.plan_learn_kpis) > 1:
            return self.plan_learn_kpis.pop(0)
        return self.plan_learn_kpis[0]

    async def get_saved_plan(
        self, profile: VehicleProfile, mode: object  # noqa: ARG002
    ) -> object | None:
        return self.saved_plan

    async def reset_saved_plans_for_mode(self, mode_id: str) -> None:
        self.reset_plan_mode_ids.append(mode_id)

    async def freeze_saved_plan_to_best(
        self, profile: VehicleProfile, mode: object  # noqa: ARG002
    ) -> None:
        self.freeze_to_best_calls += 1


def make_train_result(model_path: str = "data/models/fake.pkl") -> TrainResult:
    return TrainResult(
        model_path=model_path,
        metrics={"accel": {"n": 10.0}, "brake": {"n": 10.0}},
        feedforward_params=FeedforwardParams(),
        pid_gains=PIDGains(kp=3.0, ki=0.3, kd=0.0),
        pid_auto_tuned=True,
        dynamics_params=DynamicsParams(
            pid_preview_s=0.6, fopdt_k=0.5, fopdt_tau=2.0, fopdt_theta=0.6
        ),
    )


def patch_train_and_apply(monkeypatch: pytest.MonkeyPatch, calls: list[dict]) -> None:
    async def _fake_train_and_apply(**kwargs: object) -> TrainResult:
        calls.append(kwargs)
        return make_train_result()

    monkeypatch.setattr("src.app.learning_cycle.train_and_apply", _fake_train_and_apply)


async def arm_and_start(
    orch: LearningCycleOrchestrator,
    profile_id: str,
    refine_runs_stage1: int,
    refine_runs_stage2: int = 0,  # 後方互換: REFINE_2 廃止により無視される
) -> str:
    """テスト用ヘルパー: 自動運転と同じ arm → start の2ステップをまとめて呼ぶ。"""
    await orch.arm(profile_id)
    return await orch.start(refine_runs_stage1)


class TestHappyPath:
    @pytest.mark.asyncio
    async def test_phase_sequence_and_completion(self, monkeypatch: pytest.MonkeyPatch) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        train_calls: list[dict] = []
        patch_train_and_apply(monkeypatch, train_calls)

        cycle_id = await arm_and_start(orch, "p1", refine_runs_stage1=3, refine_runs_stage2=2)
        assert cycle_id == "cycle-1"
        assert orch.progress.phase == CyclePhase.LEARNING

        # LEARNING完了を模擬（実機では stop_learning_drive が set する）
        ctrl.state = RobotState.READY
        ctrl._learning_complete.set()
        assert orch._task is not None
        await orch._task

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert len(train_calls) == 2
        assert train_calls[0]["update_pid_gains"] is True
        assert train_calls[1]["update_pid_gains"] is False
        # mode_repo 未配線のため VERIFY/PLAN_LEARN/REFINE_F はスキップ。REFINE_1 のみ座標降下。
        assert [c["max_runs"] for c in ctrl.tuning_calls] == [3]
        # REFINE_1 は保持を維持（COMPLETED の _finalize_release で解放）
        assert [c["release_on_finish"] for c in ctrl.tuning_calls] == [False]
        assert ctrl.release_called is True  # _finalize_release で解放
        assert log_writer.ended_cycles[-1][1] == "completed"
        # サイクル終了後は controller の参加ポインタをクリアし、後続の通常走行が
        # 完了済み cycle_id を継承しないようにする。
        assert ctrl.clear_active_cycle_called is True

    @pytest.mark.asyncio
    async def test_max_runs_propagated(self, monkeypatch: pytest.MonkeyPatch) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=7, refine_runs_stage2=4)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        # mode_repo 未配線のため REFINE_1 のみ（REFINE_2 は廃止・VERIFY 以降はスキップ）。
        assert [c["max_runs"] for c in ctrl.tuning_calls] == [7]

    @pytest.mark.asyncio
    async def test_persists_best_pid_preview_s_to_profile(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """座標降下の最良 pid_preview_s がプロファイルへ永続化され制御スタックへ反映される。"""
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=2, refine_runs_stage2=2)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        # FakeController.run_pid_tuning_session は各走行で pid_preview_s を +0.2 する。
        # mode_repo 未配線のため REFINE_1（2走行=+0.4）のみで、REFINE_F は走らない。
        assert profile_repo.updates[-1].dynamics_params.pid_preview_s == pytest.approx(0.4)
        assert ctrl.refreshed_profiles[-1].dynamics_params.pid_preview_s == pytest.approx(0.4)

    @pytest.mark.asyncio
    async def test_second_stage_does_not_overwrite_gains_via_training(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """TRAINING_2 は update_pid_gains=False で呼ばれ、適合結果を訓練で上書きしない。"""
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        train_calls: list[dict] = []
        patch_train_and_apply(monkeypatch, train_calls)

        await arm_and_start(orch, "p1", refine_runs_stage1=2, refine_runs_stage2=2)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        # REFINE_1 で適合したゲインが永続化され、TRAINING_2 呼び出し後も維持されていること
        assert len(profile_repo.updates) >= 1
        # train_and_apply(update_pid_gains=False)がゲインを変更しないことは
        # test_training_service.py で個別検証済み。ここでは呼び出しフラグのみ確認する。
        assert train_calls[1]["update_pid_gains"] is False


class TestBusyAndValidation:
    @pytest.mark.asyncio
    async def test_double_start_raises_busy_error(self, monkeypatch: pytest.MonkeyPatch) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", 2, 2)
        with pytest.raises(CycleBusyError):
            await arm_and_start(orch, "p1", 2, 2)

        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

    @pytest.mark.asyncio
    async def test_start_without_prior_arm_raises_invalid_state_transition(self) -> None:
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter()
        )
        with pytest.raises(InvalidStateTransition):
            await orch.start(2)

    @pytest.mark.asyncio
    async def test_cancel_releases_hold_and_resets_progress(self) -> None:
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter()
        )
        await orch.arm("p1")
        assert orch.progress.phase == CyclePhase.ARMING

        await orch.cancel()
        assert ctrl.release_called is True
        assert orch.progress.phase == CyclePhase.IDLE

        # cancel 後は再度 arm できる（保留中のプロファイルIDが残っていない）
        with pytest.raises(InvalidStateTransition):
            await orch.start(1)

    @pytest.mark.asyncio
    async def test_restart_after_completion_succeeds(self, monkeypatch: pytest.MonkeyPatch) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", 1, 1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]
        assert orch.progress.phase == CyclePhase.COMPLETED

        # 新しいサイクルを再度開始できる
        ctrl._learning_complete.clear()
        await arm_and_start(orch, "p1", 1, 1)
        assert orch.progress.phase == CyclePhase.LEARNING
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

    @pytest.mark.asyncio
    async def test_start_failure_resets_progress_to_idle(self) -> None:
        """W5 回帰テスト: start_learning_drive が失敗した場合、ロボット状態は READY へ
        ロールバックされるが、progress を LEARNING のままにすると WS 配信で「実行中」が
        永久に見え続ける。ロールバックに合わせて progress も IDLE へ戻すこと。"""
        ctrl = FakeController()
        ctrl.fail_start_learning_drive = True
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter()
        )
        await orch.arm("p1")

        with pytest.raises(InvalidStateTransition):
            await orch.start(2)

        assert orch.progress.phase == CyclePhase.IDLE

    @pytest.mark.asyncio
    async def test_arm_with_unknown_profile_raises_value_error(self) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(None)
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)

        with pytest.raises(ValueError):
            await orch.arm("missing")


class TestAbort:
    @pytest.mark.asyncio
    async def test_abort_without_running_cycle_raises(self) -> None:
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(None), FakeSessionRepo(), FakeLogWriter()
        )
        with pytest.raises(InvalidStateTransition):
            await orch.abort()

    @pytest.mark.asyncio
    async def test_abort_during_arming_cancels_instead_of_raising(self) -> None:
        """W5 回帰テスト: arm() 済み・start() 未実行（_task 未生成）の間の abort() は
        409 にせず、cancel() と同じアーム中断（保持ブレーキ解放＋進捗リセット）を行う。"""
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter()
        )
        await orch.arm("p1")
        assert orch.progress.phase == CyclePhase.ARMING
        assert orch._task is None

        await orch.abort()  # 例外を送出しない

        assert ctrl.release_called is True
        assert orch.progress.phase == CyclePhase.IDLE

    @pytest.mark.asyncio
    async def test_abort_during_learning_stops_drive_and_finalizes_aborted(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", 2, 2)
        assert ctrl.state == RobotState.RUNNING  # 学習運転中

        await orch.abort()
        assert ctrl.stop_called is True  # 走行中なら能動的に停止させる

        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.ABORTED
        assert log_writer.ended_cycles[-1][1] == "aborted"
        assert ctrl.clear_active_cycle_called is True

    def test_make_on_run_raises_cycle_aborted_when_flagged(self) -> None:
        """PID適合の on_run コールバックが中断フラグを検知して例外送出すること。"""
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(None), FakeSessionRepo(), FakeLogWriter()
        )
        on_run = orch._make_on_run(run_total=5)

        on_run(1, TuningParams(kp=1.0, ki=0.0, kd=0.0), 0.5)  # 通常は例外なし

        orch._abort_requested = True
        with pytest.raises(CycleAborted):
            on_run(2, TuningParams(kp=1.0, ki=0.0, kd=0.0), 0.4)


class TestErrorHandling:
    @pytest.mark.asyncio
    async def test_training_error_transitions_to_error_and_closes_cycle(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)

        async def _fail_train_and_apply(**kwargs: object) -> TrainResult:  # noqa: ARG001
            raise RuntimeError("学習に失敗しました")

        monkeypatch.setattr("src.app.learning_cycle.train_and_apply", _fail_train_and_apply)

        await arm_and_start(orch, "p1", 2, 2)
        ctrl.state = RobotState.READY
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.ERROR
        assert log_writer.ended_cycles[-1][1] == "error"
        assert "error" in log_writer.ended_cycles[-1][2]
        assert ctrl.clear_active_cycle_called is True

    @pytest.mark.asyncio
    async def test_learning_timeout_transitions_to_error(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(
            ctrl, profile_repo, session_repo, log_writer, learning_timeout_s=0.01
        )
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", 1, 1)
        # _learning_complete を set しないままタイムアウトさせる
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.ERROR
        assert log_writer.ended_cycles[-1][1] == "error"


class _VerifyModeRepo:
    """VERIFY 用に list_all を備えた最小モードリポジトリ。

    system_mode に前世代のシステムモードを渡すと get_system_mode がそれを返す
    （網羅パターンの軌跡変化検出のテスト用。既定 None＝初回サイクル）。
    """

    def __init__(self, system_mode=None) -> None:  # noqa: ANN001
        from src.models.driving_mode import DrivingMode, SpeedPoint

        self._modes = [
            DrivingMode(
                id="m1", name="M1", description="",
                reference_speed=[SpeedPoint(0.0, 0.0), SpeedPoint(30.0, 90.0),
                                 SpeedPoint(60.0, 90.0), SpeedPoint(90.0, 0.0)],
                total_duration=90.0, max_speed=90.0, created_at=datetime.now(tz=UTC),
            )
        ]
        self._system_mode = system_mode

    async def get_by_id(self, mode_id: str):  # noqa: ANN201, ARG002
        return None

    async def list_all(self):  # noqa: ANN201
        return self._modes

    async def get_system_mode(self):  # noqa: ANN201
        return self._system_mode

    async def upsert_system_mode(self, mode):  # noqa: ANN001, ANN201
        from dataclasses import replace

        return replace(mode, id="sys-verify", is_system=True)


class TestVerifyPhase:
    @pytest.mark.asyncio
    async def test_verify_default_one_run_with_finishing_retrain(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """既定 verify_runs=1（完走型）: 検証 1 本走行し、最終走行後も仕上げ再学習する。"""
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter(),
            mode_repo=_VerifyModeRepo(),
        )
        train_calls: list[dict] = []
        patch_train_and_apply(monkeypatch, train_calls)

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert ctrl.verify_calls == 1
        # TRAINING_1 + TRAINING_2 + VERIFY 内の仕上げ再学習（最終走行後も行う）= 3
        assert len(train_calls) == 3
        assert ctrl.release_called is True

    @pytest.mark.asyncio
    async def test_verify_multiple_runs_retrains_after_each(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """verify_runs=2: 2 本走行し、毎回（最終走行後も）仕上げ再学習する。"""
        ctrl = FakeController()  # 既定 KPI は常に合格
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter(),
            mode_repo=_VerifyModeRepo(), verify_runs=2,
        )
        train_calls: list[dict] = []
        patch_train_and_apply(monkeypatch, train_calls)

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.verify_calls == 2
        # 2(stage) + 2(VERIFY 内、毎走行後の仕上げ再学習) = 4
        assert len(train_calls) == 4

    @pytest.mark.asyncio
    async def test_system_mode_trajectory_change_resets_saved_plans(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """プラン引き継ぎ（2026-07-16）の安全弁: 登録モードの追加・編集で網羅パターンの
        軌跡が前世代から変わったら、システムモードの保存プランをリセットする。

        旧仕様はモデル再学習の model_path 不一致で暗黙に無効化されていたが、引き継ぎ導入で
        その経路が消えたため、明示リセットがないと旧軌跡の best_reward にロールバック機構が
        固着する。"""
        from src.models.driving_mode import DrivingMode as DM
        from src.models.driving_mode import SpeedPoint as SP

        prev_system_mode = DM(
            id="sys-verify", name="__verify_pattern__", description="",
            reference_speed=[SP(0.0, 0.0), SP(30.0, 50.0), SP(60.0, 0.0)],  # 旧包絡（別物）
            total_duration=60.0, max_speed=50.0, created_at=datetime.now(tz=UTC),
            is_system=True,
        )
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter(),
            mode_repo=_VerifyModeRepo(system_mode=prev_system_mode),
        )
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert ctrl.reset_plan_mode_ids == ["sys-verify"]

    @pytest.mark.asyncio
    async def test_system_mode_same_trajectory_keeps_saved_plans(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """網羅パターンの軌跡が前世代と同一なら保存プランを維持する（プラン引き継ぎ本則）。"""
        from dataclasses import replace as dc_replace

        from src.domain.pid_tuning import build_verification_trajectory

        mode_repo = _VerifyModeRepo()
        profile = make_profile()
        # 実装と同じ生成器・同じ入力で前世代の網羅パターンを作る（軌跡は一致するはず）。
        pattern = build_verification_trajectory(mode_repo._modes, profile, budget_s=180.0)
        mode_repo._system_mode = dc_replace(pattern, id="sys-verify", is_system=True)

        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(profile), FakeSessionRepo(), FakeLogWriter(),
            mode_repo=mode_repo,
        )
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert ctrl.reset_plan_mode_ids == []

    @pytest.mark.asyncio
    async def test_verify_kpi_failure_does_not_block_plan_learn_or_refine_final(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """デッドロック回帰テスト（2026-07-14）: VERIFY が KPI 未達でも PLAN_LEARN/REFINE_F は
        必ず実行される。旧実装は VERIFY 不合格で以降を丸ごとスキップしていたが、VERIFY は
        FF 由来プランのみで走るため KPI 0.2 に構造的に届かず、プラン学習に永遠に入れなかった
        （実機で確認）。"""
        ctrl = FakeController()
        failing_kpi = {"n_samples": 100.0, "p95_kmh": 3.0, "max_abs_deviation_kmh": 4.0,
                       "reversal_max_per_5s": 5.0}
        ctrl.verify_kpis = [failing_kpi]
        # PLAN_LEARN も改善せず不合格が続く想定（真因が未解決のまま最後まで完走するケース）。
        ctrl.plan_learn_kpis = [failing_kpi, failing_kpi]
        lw = FakeLogWriter()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), lw,
            mode_repo=_VerifyModeRepo(),
        )
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        detail = lw.ended_cycles[-1][2]
        assert detail["verify"]["kpi_passed"] is False
        assert ctrl.plan_learn_calls >= 1  # スキップされていない
        assert len(ctrl.tuning_calls) >= 2  # REFINE_1 に加え REFINE_F も実行されている
        assert "警告" in orch.progress.message

    @pytest.mark.asyncio
    async def test_verify_skipped_without_mode_repo(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """mode_repo 未配線なら VERIFY をスキップして COMPLETED（従来どおり解放）。"""
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter(),
        )
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert ctrl.verify_calls == 0
        assert ctrl.release_called is True


def _passing_kpi(over_limit: float = 0.0) -> dict[str, float]:
    return {
        "n_samples": 100.0, "p95_kmh": 0.1, "max_abs_deviation_kmh": 0.5,
        "reversal_max_per_5s": 0.0, "over_limit_integral_kmhs": over_limit,
    }


def _failing_kpi(over_limit: float = 0.0) -> dict[str, float]:
    return {
        "n_samples": 100.0, "p95_kmh": 0.6, "max_abs_deviation_kmh": 1.5,
        "reversal_max_per_5s": 0.0, "over_limit_integral_kmhs": over_limit,
    }


class TestPlanLearnPhase:
    """PLAN_LEARN フェーズ（VERIFY 合格後のプラン学習収束）。"""

    def _orch(
        self, ctrl: FakeController, lw: "FakeLogWriter | None" = None, **kw: object
    ) -> LearningCycleOrchestrator:
        return LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), lw or FakeLogWriter(),
            mode_repo=_VerifyModeRepo(), verify_runs=1, **kw,  # type: ignore[arg-type]
        )

    @pytest.mark.asyncio
    async def test_plan_learn_converges_after_patience_when_kpi_keeps_failing(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """KPI が合格しなくても改善なし（同一 KPI）が patience=2 回連続したら早期打ち切り。

        2026-07-16: 旧 1 発判定（改善なし 1 本で収束扱い）は reward の走行間ばらつきで
        単調改善中でも誤発動したため、2 連続に変更。
        """
        ctrl = FakeController()
        ctrl.plan_learn_kpis = [_failing_kpi()] * 3  # 同一 → 改善 0 が続く
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        # 1本目(基準) + 2本目(改善なし1) + 3本目(改善なし2=patience到達)で収束
        assert ctrl.plan_learn_calls == 3
        detail = lw.ended_cycles[-1][2]["plan_learn"]
        assert detail["converged"] is True
        assert detail["kpi_passed"] is False
        assert len(detail["runs"]) == 3

    @pytest.mark.asyncio
    async def test_plan_learn_single_regression_does_not_stop(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """改善→悪化→改善のシーケンスでは打ち切られない（1 本の悪化はばらつきとみなす）。

        7/15 実機の回帰ケース: p95 が単調改善中でも reward が 1 本ぶれただけで旧実装は
        「収束」と誤判定していた。
        """
        ctrl = FakeController()
        # reward: 基準 → 悪化(-2) → 最良を+2超えて改善(>ε) → 改善なしカウントがリセットされ
        # runs_max=3 まで走り切る（収束扱いにならない）
        ctrl.plan_learn_kpis = [
            _failing_kpi(over_limit=0.4),
            _failing_kpi(over_limit=0.6),
            _failing_kpi(over_limit=0.2),
        ]
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, plan_learn_runs_max=3)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.plan_learn_calls == 3
        detail = lw.ended_cycles[-1][2]["plan_learn"]
        assert detail["converged"] is False  # 打ち切りでなく本数上限で終了

    @pytest.mark.asyncio
    async def test_plan_learn_two_consecutive_regressions_stop(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """改善なしが 2 連続（悪化→悪化）したら runs_max 前でも収束打ち切りする。"""
        ctrl = FakeController()
        ctrl.plan_learn_kpis = [
            _failing_kpi(over_limit=0.4),  # 基準（最良）
            _failing_kpi(over_limit=0.6),  # 悪化1
            _failing_kpi(over_limit=0.5),  # 悪化2（最良未満）→ patience 到達
            _failing_kpi(over_limit=0.1),  # ここまで到達しない
        ]
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, plan_learn_runs_max=4)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.plan_learn_calls == 3
        detail = lw.ended_cycles[-1][2]["plan_learn"]
        assert detail["converged"] is True

    @pytest.mark.asyncio
    async def test_plan_learn_runs_to_max_when_kpi_fails_and_reward_keeps_improving(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """KPI 不合格が続き reward が毎回 ε 超で改善し続けるなら上限本数まで走る（収束せず）。"""
        ctrl = FakeController()
        # over_limit を 0.2 ずつ減らす → reward が毎回 +2.0（>ε=1.0）改善
        ctrl.plan_learn_kpis = [_failing_kpi(over_limit=o) for o in (0.8, 0.6, 0.4)]
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, plan_learn_runs_max=3)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.plan_learn_calls == 3
        detail = lw.ended_cycles[-1][2]["plan_learn"]
        assert detail["converged"] is False
        assert detail["kpi_passed"] is False

    @pytest.mark.asyncio
    async def test_plan_learn_stops_immediately_when_kpi_passes(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """KPI 合格は reward 収束を待たず即座に打ち切る（2026-07-14: 旧「合格かつ収束」から
        「合格または収束」に変更。旧実装は未合格時に必ず runs_max 本走り切っていた）。"""
        ctrl = FakeController()
        ctrl.plan_learn_kpis = [_passing_kpi()]
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, plan_learn_runs_max=5)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.plan_learn_calls == 1
        detail = lw.ended_cycles[-1][2]["plan_learn"]
        assert detail["converged"] is True
        assert detail["kpi_passed"] is True
        assert len(detail["runs"]) == 1

    @pytest.mark.asyncio
    async def test_plan_learn_runs_even_when_verify_failed(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """VERIFY 未達でも PLAN_LEARN は無条件で実行される
        （2026-07-14 完走型・デッドロック解消）。"""
        ctrl = FakeController()
        ctrl.verify_kpis = [
            {"n_samples": 100.0, "p95_kmh": 0.6, "max_abs_deviation_kmh": 1.5,
             "reversal_max_per_5s": 5.0}
        ]
        orch = self._orch(ctrl)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert ctrl.plan_learn_calls >= 1

    @pytest.mark.asyncio
    async def test_plan_learn_skipped_when_runs_max_zero(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """plan_learn_runs_max=0 なら PLAN_LEARN フェーズをスキップ。"""
        ctrl = FakeController()
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, plan_learn_runs_max=0)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1, refine_runs_stage2=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.plan_learn_calls == 0
        assert lw.ended_cycles[-1][2]["plan_learn"].get("skipped") is True

    @pytest.mark.asyncio
    async def test_plan_learn_reports_best_run_not_last(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """最終結果は最良走行（best reward）を基準にする（2026-09-08 修正）。

        旧実装は最終走行（last）の KPI をそのまま採用しており、反復が悪化方向に振れると
        「最良を見つけたのに最終的にそれより悪い状態でサイクルが完了する」ことがあった
        （実機 2026-09-07: p95 1.74→2.35→3.34 と悪化して完了）。1本目が最良で以降悪化し
        続けても、detail["plan_learn"]["final_kpi"] は 1本目の KPI を報告すること。
        """
        ctrl = FakeController()
        # 打ち切りしきい値（PLAN_LEARN_P95_DEGRADE_KMH=0.5）に触れない緩やかな悪化
        ctrl.plan_learn_kpis = [
            {"n_samples": 100.0, "p95_kmh": 1.74, "max_abs_deviation_kmh": 4.0,
             "reversal_max_per_5s": 7.0},
            {"n_samples": 100.0, "p95_kmh": 1.90, "max_abs_deviation_kmh": 4.4,
             "reversal_max_per_5s": 9.0},
            {"n_samples": 100.0, "p95_kmh": 2.05, "max_abs_deviation_kmh": 4.9,
             "reversal_max_per_5s": 10.0},
        ]
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, plan_learn_runs_max=3)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.plan_learn_calls == 3
        detail = lw.ended_cycles[-1][2]["plan_learn"]
        assert detail["best_run"] == 1
        assert detail["final_kpi"]["p95_kmh"] == pytest.approx(1.74)
        # サイクル全体の detail もこの best 基準の KPI を引き継ぐ
        assert lw.ended_cycles[-1][2]["final_kpi"]["p95_kmh"] == pytest.approx(1.74)
        # 保存プランを最良へ明示的に確定している
        assert ctrl.freeze_to_best_calls == 1

    @pytest.mark.asyncio
    async def test_plan_learn_best_follows_p95_not_reward(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """best・打ち切りは (KPI合否, -p95, reward) の辞書順で決める（2026-09-09 修正）。

        実機 3ca20d43 の回帰ケース: p95 は 2.92→2.22→2.09 と単調改善しているのに reward は
        run3 で悪化し、旧実装は (a) reward の degrade 判定で 4 本目を打ち切り (b) p95 の悪い
        run2 を最終結果として採用していた。
        """
        ctrl = FakeController()
        ctrl.plan_learn_kpis = [
            {"n_samples": 100.0, "p95_kmh": 2.92, "max_abs_deviation_kmh": 4.59,
             "reversal_max_per_5s": 8.0},
            {"n_samples": 100.0, "p95_kmh": 2.22, "max_abs_deviation_kmh": 3.82,
             "reversal_max_per_5s": 8.0},
            # p95 は改善だが max 偏差が悪化 → reward は run2 より悪くなる。
            # max の値は実機 13 反復の実測（iter 7 の 5.19 / iter 13 の 5.01）。
            {"n_samples": 100.0, "p95_kmh": 2.09, "max_abs_deviation_kmh": 5.19,
             "reversal_max_per_5s": 8.0},
            {"n_samples": 100.0, "p95_kmh": 2.00, "max_abs_deviation_kmh": 5.01,
             "reversal_max_per_5s": 8.0},
        ]
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, plan_learn_runs_max=4)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        # p95 が改善し続ける限り打ち切らない
        assert ctrl.plan_learn_calls == 4
        detail = lw.ended_cycles[-1][2]["plan_learn"]
        assert detail["degraded"] is False
        assert detail["best_run"] == 4
        assert detail["best_p95"] == pytest.approx(2.00)
        assert detail["final_kpi"]["p95_kmh"] == pytest.approx(2.00)
        # reward 最良は run2 だが、p95 が優先されるので採用されない。
        # この前提が崩れると「辞書順であること」を検証できないので明示的に確かめる。
        rewards = [r["reward"] for r in detail["runs"]]
        assert rewards[1] == max(rewards), "前提: reward 最良は run2（p95 最良の run4 ではない）"
        assert rewards[1] > rewards[3], "前提: reward は run2 が run4 より良い"

    @pytest.mark.asyncio
    async def test_plan_learn_degrade_stops_early(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """best から大幅に悪化（PLAN_LEARN_DEGRADE_FACTOR×epsilon 超）したら PATIENCE を
        待たず即座に打ち切る。"""
        ctrl = FakeController()
        ctrl.plan_learn_kpis = [
            {"n_samples": 100.0, "p95_kmh": 1.74, "max_abs_deviation_kmh": 4.0,
             "reversal_max_per_5s": 7.0},
            {"n_samples": 100.0, "p95_kmh": 6.0, "max_abs_deviation_kmh": 10.0,
             "reversal_max_per_5s": 20.0},
            {"n_samples": 100.0, "p95_kmh": 1.5, "max_abs_deviation_kmh": 3.5,
             "reversal_max_per_5s": 5.0},
        ]
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, plan_learn_runs_max=5)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.plan_learn_calls == 2  # 3本目は走らない
        detail = lw.ended_cycles[-1][2]["plan_learn"]
        assert detail["degraded"] is True
        assert detail["converged"] is False
        assert detail["final_kpi"]["p95_kmh"] == pytest.approx(1.74)
        assert ctrl.freeze_to_best_calls == 1


class TestRefineFinalPhase:
    """REFINE_F フェーズ（収束プラン凍結・PID 仕上げ座標降下）。"""

    def _orch(
        self, ctrl: FakeController, lw: "FakeLogWriter | None" = None, **kw: object
    ) -> LearningCycleOrchestrator:
        return LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), lw or FakeLogWriter(),
            mode_repo=_VerifyModeRepo(), verify_runs=1, **kw,  # type: ignore[arg-type]
        )

    @pytest.mark.asyncio
    async def test_refine_final_freezes_plan_and_persists_gains(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """VERIFY 合格 → PLAN_LEARN → REFINE_F: 凍結プランで座標降下しゲインを永続化する。"""
        ctrl = FakeController()
        sentinel_plan = object()
        ctrl.saved_plan = sentinel_plan  # get_saved_plan が返す凍結プラン
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, refine_final_runs=2)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1, refine_runs_stage2=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        # 最後の run_pid_tuning_session（REFINE_F）が凍結プラン＋システムモードで走る
        refine_f_call = ctrl.tuning_calls[-1]
        assert refine_f_call["plan"] is sentinel_plan
        assert refine_f_call["max_runs"] == 2
        assert refine_f_call["release_on_finish"] is False
        mode = refine_f_call["mode"]
        assert mode is not None and mode.is_system is True  # type: ignore[union-attr]
        # 最良ゲインが永続化・反映される（refresh_active_profile が呼ばれる）
        assert len(ctrl.refreshed_profiles) >= 1
        detail = lw.ended_cycles[-1][2]["refine_final"]
        assert detail["frozen_plan"] is True

    @pytest.mark.asyncio
    async def test_refine_final_skipped_when_runs_zero(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """refine_final_runs=0 なら REFINE_F をスキップ（tuning は REFINE_1/2 の 2 回のみ）。"""
        ctrl = FakeController()
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, refine_final_runs=0)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1, refine_runs_stage2=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert lw.ended_cycles[-1][2]["refine_final"].get("skipped") is True

    @pytest.mark.asyncio
    async def test_refine_final_falls_back_to_ff_plan_when_none(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """凍結プラン未保存（get_saved_plan None）なら plan=None で FF 由来へフォールバック。"""
        ctrl = FakeController()
        ctrl.saved_plan = None
        lw = FakeLogWriter()
        orch = self._orch(ctrl, lw, refine_final_runs=1)
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1, refine_runs_stage2=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert ctrl.tuning_calls[-1]["plan"] is None
        assert lw.ended_cycles[-1][2]["refine_final"]["frozen_plan"] is False
