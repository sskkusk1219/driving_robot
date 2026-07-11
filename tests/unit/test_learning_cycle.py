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
    ) -> tuple[TuningParams, list[dict]]:
        self.tuning_calls.append(
            {"max_runs": max_runs, "release_on_finish": release_on_finish, "mode": mode}
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
    refine_runs_stage2: int,
) -> str:
    """テスト用ヘルパー: 自動運転と同じ arm → start の2ステップをまとめて呼ぶ。"""
    await orch.arm(profile_id)
    return await orch.start(refine_runs_stage1, refine_runs_stage2)


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
        assert [c["max_runs"] for c in ctrl.tuning_calls] == [3, 2]
        # REFINE_1/REFINE_2 とも保持を維持（VERIFY 後の _finalize_release で解放）
        assert [c["release_on_finish"] for c in ctrl.tuning_calls] == [False, False]
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

        assert [c["max_runs"] for c in ctrl.tuning_calls] == [7, 4]

    @pytest.mark.asyncio
    async def test_refine2_uses_target_mode_representative_trajectory(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """Stage B: target_mode_id 指定 + tuning_on_target_mode 有効時、REFINE_2 の評価走行に
        本番モード代表区間（DrivingMode, id=pid-tune）が渡され、REFINE_1 は従来どおり None。"""
        from src.domain.pid_tuning import TUNING_MODE_ID
        from src.models.driving_mode import DrivingMode, SpeedPoint

        long_mode = DrivingMode(
            id="mode-exhi",
            name="ExHi",
            description="",
            reference_speed=[
                SpeedPoint(0.0, 0.0), SpeedPoint(40.0, 130.0), SpeedPoint(90.0, 130.0),
                SpeedPoint(200.0, 131.0), SpeedPoint(300.0, 0.0),
            ],
            total_duration=300.0,
            max_speed=131.0,
            created_at=datetime.now(tz=UTC),
        )

        class FakeModeRepo:
            async def get_by_id(self, mode_id: str) -> DrivingMode | None:
                return long_mode if mode_id == "mode-exhi" else None

            async def list_all(self) -> list[DrivingMode]:
                return [long_mode]

        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(
            ctrl, profile_repo, session_repo, log_writer,
            mode_repo=FakeModeRepo(), tuning_on_target_mode=True,
        )
        patch_train_and_apply(monkeypatch, [])

        await orch.arm("p1")
        await orch.start(2, 2, target_mode_id="mode-exhi")
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        # REFINE_1 は規定パターン(None)、REFINE_2 は代表区間(DrivingMode)
        modes = [c["mode"] for c in ctrl.tuning_calls]
        assert modes[0] is None
        refine2_mode = modes[1]
        assert isinstance(refine2_mode, DrivingMode)
        assert refine2_mode.id == TUNING_MODE_ID
        assert refine2_mode.total_duration <= 120.0 + 1e-6
        assert max(p.speed_kmh for p in refine2_mode.reference_speed) >= 0.8 * long_mode.max_speed

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
        # FakeController.run_pid_tuning_session は各走行で pid_preview_s を +0.2 する
        # ため、2段(各2走行=+0.4ずつ)適合後は 0.8 になっているはず
        assert profile_repo.updates[-1].dynamics_params.pid_preview_s == pytest.approx(0.8)
        assert ctrl.refreshed_profiles[-1].dynamics_params.pid_preview_s == pytest.approx(0.8)

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
        assert len(profile_repo.updates) >= 2
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
            await orch.start(2, 2)

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
            await orch.start(1, 1)

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
            await orch.start(2, 2)

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
    """VERIFY 用に list_all を備えた最小モードリポジトリ。"""

    def __init__(self) -> None:
        from src.models.driving_mode import DrivingMode, SpeedPoint

        self._modes = [
            DrivingMode(
                id="m1", name="M1", description="",
                reference_speed=[SpeedPoint(0.0, 0.0), SpeedPoint(30.0, 90.0),
                                 SpeedPoint(60.0, 90.0), SpeedPoint(90.0, 0.0)],
                total_duration=90.0, max_speed=90.0, created_at=datetime.now(tz=UTC),
            )
        ]

    async def get_by_id(self, mode_id: str):  # noqa: ANN201, ARG002
        return None

    async def list_all(self):  # noqa: ANN201
        return self._modes


class TestVerifyPhase:
    @pytest.mark.asyncio
    async def test_verify_passes_first_run(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """検証 1 本で KPI 合格 → COMPLETED（passed=True）、再学習は走らない。"""
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter(),
            mode_repo=_VerifyModeRepo(),
        )
        train_calls: list[dict] = []
        patch_train_and_apply(monkeypatch, train_calls)

        await arm_and_start(orch, "p1", refine_runs_stage1=1, refine_runs_stage2=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert ctrl.verify_calls == 1
        # TRAINING_1 + TRAINING_2 の 2 回のみ（VERIFY 内の再学習は起きない）
        assert len(train_calls) == 2
        assert ctrl.release_called is True

    @pytest.mark.asyncio
    async def test_verify_retries_then_warns(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """全走行 KPI 未達 → 上限まで再学習して走行し、passed=False で完了する。"""
        ctrl = FakeController()
        # 常に不合格の KPI（max 超過）
        ctrl.verify_kpis = [
            {"n_samples": 100.0, "p95_kmh": 0.6, "max_abs_deviation_kmh": 1.5,
             "reversal_max_per_5s": 5.0}
        ]
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter(),
            mode_repo=_VerifyModeRepo(), verify_runs_max=3,
        )
        train_calls: list[dict] = []
        patch_train_and_apply(monkeypatch, train_calls)

        await arm_and_start(orch, "p1", refine_runs_stage1=1, refine_runs_stage2=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert ctrl.verify_calls == 3  # 上限まで走行
        # 2(stage) + 2(VERIFY 内の再学習は不合格時 run<max の 2 回) = 4
        assert len(train_calls) == 2 + 2
        assert ctrl.release_called is True

    @pytest.mark.asyncio
    async def test_verify_skipped_without_mode_repo(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """mode_repo 未配線なら VERIFY をスキップして COMPLETED（従来どおり解放）。"""
        ctrl = FakeController()
        orch = LearningCycleOrchestrator(
            ctrl, FakeProfileRepo(make_profile()), FakeSessionRepo(), FakeLogWriter(),
        )
        patch_train_and_apply(monkeypatch, [])

        await arm_and_start(orch, "p1", refine_runs_stage1=1, refine_runs_stage2=1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        assert ctrl.verify_calls == 0
        assert ctrl.release_called is True
