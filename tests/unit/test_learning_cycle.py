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

    async def arm_learning_drive(self) -> None:
        pass

    async def start_learning_drive(self, log_writer: object = None) -> DriveSession:  # noqa: ARG002
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
    ) -> tuple[TuningParams, list[dict]]:
        self.tuning_calls.append(
            {"max_runs": max_runs, "release_on_finish": release_on_finish}
        )
        best = TuningParams.from_profile(profile)
        history = []
        for i in range(1, max_runs + 1):
            await asyncio.sleep(0)
            cost = 1.0 / i
            best = replace(best, kp=best.kp + 0.1, preview_time_s=best.preview_time_s + 0.2)
            if on_run is not None:
                on_run(i, best, cost)  # type: ignore[operator]
            history.append(
                {
                    "kp": best.kp,
                    "ki": best.ki,
                    "kd": best.kd,
                    "preview_time_s": best.preview_time_s,
                    "cost": cost,
                }
            )
        return best, history


def make_train_result(model_path: str = "data/models/fake.pkl") -> TrainResult:
    return TrainResult(
        model_path=model_path,
        metrics={"accel": {"n": 10.0}, "brake": {"n": 10.0}},
        feedforward_params=FeedforwardParams(),
        pid_gains=PIDGains(kp=3.0, ki=0.3, kd=0.0),
        pid_auto_tuned=True,
        dynamics_params=DynamicsParams(
            preview_time_s=0.6, fopdt_k=0.5, fopdt_tau=2.0, fopdt_theta=0.6
        ),
    )


def patch_train_and_apply(monkeypatch: pytest.MonkeyPatch, calls: list[dict]) -> None:
    async def _fake_train_and_apply(**kwargs: object) -> TrainResult:
        calls.append(kwargs)
        return make_train_result()

    monkeypatch.setattr("src.app.learning_cycle.train_and_apply", _fake_train_and_apply)


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

        cycle_id = await orch.start("p1", refine_runs_stage1=3, refine_runs_stage2=2)
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
        assert [c["release_on_finish"] for c in ctrl.tuning_calls] == [False, True]
        assert log_writer.ended_cycles[-1][1] == "completed"

    @pytest.mark.asyncio
    async def test_max_runs_propagated(self, monkeypatch: pytest.MonkeyPatch) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await orch.start("p1", refine_runs_stage1=7, refine_runs_stage2=4)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert [c["max_runs"] for c in ctrl.tuning_calls] == [7, 4]

    @pytest.mark.asyncio
    async def test_persists_best_preview_time_s_to_profile(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """座標降下の最良 preview_time_s がプロファイルへ永続化され制御スタックへ反映される。"""
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await orch.start("p1", refine_runs_stage1=2, refine_runs_stage2=2)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.COMPLETED
        # FakeController.run_pid_tuning_session は各走行で preview_time_s を +0.2 する
        # ため、2段(各2走行=+0.4ずつ)適合後は 0.8 になっているはず
        assert profile_repo.updates[-1].dynamics_params.preview_time_s == pytest.approx(0.8)
        assert ctrl.refreshed_profiles[-1].dynamics_params.preview_time_s == pytest.approx(0.8)

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

        await orch.start("p1", refine_runs_stage1=2, refine_runs_stage2=2)
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

        await orch.start("p1", 2, 2)
        with pytest.raises(CycleBusyError):
            await orch.start("p1", 2, 2)

        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

    @pytest.mark.asyncio
    async def test_restart_after_completion_succeeds(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await orch.start("p1", 1, 1)
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]
        assert orch.progress.phase == CyclePhase.COMPLETED

        # 新しいサイクルを再度開始できる
        ctrl._learning_complete.clear()
        await orch.start("p1", 1, 1)
        assert orch.progress.phase == CyclePhase.LEARNING
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

    @pytest.mark.asyncio
    async def test_start_with_unknown_profile_raises_value_error(self) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(None)
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)

        with pytest.raises(ValueError):
            await orch.start("missing", 1, 1)


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
    async def test_abort_during_learning_stops_drive_and_finalizes_aborted(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        ctrl = FakeController()
        profile_repo = FakeProfileRepo(make_profile())
        session_repo = FakeSessionRepo()
        log_writer = FakeLogWriter()
        orch = LearningCycleOrchestrator(ctrl, profile_repo, session_repo, log_writer)
        patch_train_and_apply(monkeypatch, [])

        await orch.start("p1", 2, 2)
        assert ctrl.state == RobotState.RUNNING  # 学習運転中

        await orch.abort()
        assert ctrl.stop_called is True  # 走行中なら能動的に停止させる

        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.ABORTED
        assert log_writer.ended_cycles[-1][1] == "aborted"

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

        await orch.start("p1", 2, 2)
        ctrl.state = RobotState.READY
        ctrl._learning_complete.set()
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.ERROR
        assert log_writer.ended_cycles[-1][1] == "error"
        assert "error" in log_writer.ended_cycles[-1][2]

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

        await orch.start("p1", 1, 1)
        # _learning_complete を set しないままタイムアウトさせる
        await orch._task  # type: ignore[union-attr]

        assert orch.progress.phase == CyclePhase.ERROR
        assert log_writer.ended_cycles[-1][1] == "error"
