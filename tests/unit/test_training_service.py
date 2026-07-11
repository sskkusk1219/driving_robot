"""training_service.train_and_apply のユニットテスト。"""

from datetime import UTC, datetime

import pytest

from src.app.training_service import TrainResult, feature_spec_from_settings, train_and_apply
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import FeatureSpec
from src.domain.pid_tuning import FOPDT
from src.infra.settings import ModelSettings
from src.models.drive_log import DriveLog
from src.models.profile import DynamicsParams, PIDGains, StopConfig, VehicleProfile


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


def make_log() -> DriveLog:
    return DriveLog(
        id=1,
        session_id="sess-1",
        timestamp=datetime.now(tz=UTC),
        ref_speed_kmh=None,
        actual_speed_kmh=10.0,
        accel_opening=30.0,
        brake_opening=0.0,
        accel_pos=1000,
        brake_pos=0,
        accel_current=500.0,
        brake_current=0.0,
    )


class FakeProfileRepo:
    def __init__(self, profile: VehicleProfile | None) -> None:
        self._profile = profile
        self.updated: VehicleProfile | None = None

    async def get_by_id(self, profile_id: str) -> VehicleProfile | None:  # noqa: ARG002
        return self._profile

    async def update(self, profile: VehicleProfile) -> VehicleProfile | None:
        self.updated = profile
        return profile


class FakeSessionRepo:
    def __init__(self, logs: list[DriveLog]) -> None:
        self._logs = logs

    async def list_logs_for_training(
        self,
        profile_id: str,  # noqa: ARG002
        session_ids: list[str] | None = None,  # noqa: ARG002
        limit: int = 100_000,  # noqa: ARG002
    ) -> list[DriveLog]:
        return self._logs


class FakeController:
    def __init__(self, refresh_result: bool = True) -> None:
        self._refresh_result = refresh_result
        self.refreshed_with: VehicleProfile | None = None

    def refresh_active_profile(self, profile: VehicleProfile) -> bool:
        self.refreshed_with = profile
        return self._refresh_result


@pytest.fixture(autouse=True)
def _patch_domain_functions(monkeypatch: pytest.MonkeyPatch) -> None:
    """train_inverse_model 等の重い処理をモックし、ユニットテストを高速・決定的にする。"""
    monkeypatch.setattr(
        "src.app.training_service.train_inverse_model",
        lambda logs, profile, output_dir="data/models", feature_spec=None: (  # noqa: ARG005
            "data/models/fake.pkl",
            {"accel": {"n": 1.0}, "brake": {"n": 1.0}},
        ),
    )
    monkeypatch.setattr(
        "src.app.training_service.estimate_dynamics_params",
        lambda logs, current: current,  # noqa: ARG005
    )


class TestTrainAndApplyUpdatePidGainsFlag:
    @pytest.mark.asyncio
    async def test_update_pid_gains_true_applies_simc_gains(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        profile = make_profile()
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()

        monkeypatch.setattr(
            "src.app.training_service.identify_fopdt",
            lambda logs, profile: FOPDT(k=0.5, tau=2.0, theta=0.3),  # noqa: ARG005
        )
        monkeypatch.setattr(
            "src.app.training_service.compute_pid_gains_simc",
            lambda fopdt, profile, tau_c_factor=0.5: PIDGains(kp=9.0, ki=1.0, kd=0.0),  # noqa: ARG005
        )

        result = await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=profile.id,
            session_ids=["sess-1"],
            update_pid_gains=True,
        )

        assert isinstance(result, TrainResult)
        assert result.pid_auto_tuned is True
        assert result.pid_gains == PIDGains(kp=9.0, ki=1.0, kd=0.0)
        assert profile_repo.updated is not None
        assert profile_repo.updated.pid_gains == PIDGains(kp=9.0, ki=1.0, kd=0.0)

    @pytest.mark.asyncio
    async def test_update_pid_gains_false_skips_simc_and_keeps_existing_gains(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        profile = make_profile()
        original_gains = profile.pid_gains
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()

        identify_called = False

        def _fail_if_called(logs: object, profile: object) -> FOPDT | None:
            nonlocal identify_called
            identify_called = True
            return FOPDT(k=0.5, tau=2.0, theta=0.3)

        monkeypatch.setattr("src.app.training_service.identify_fopdt", _fail_if_called)

        result = await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=profile.id,
            session_ids=["sess-1"],
            update_pid_gains=False,
        )

        assert identify_called is False  # FOPDT同定・SIMC計算は呼ばれない
        assert result.pid_auto_tuned is False
        assert result.pid_gains == original_gains
        assert profile_repo.updated is not None
        assert profile_repo.updated.pid_gains == original_gains

    @pytest.mark.asyncio
    async def test_update_pid_gains_true_but_fopdt_none_keeps_existing_gains(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """区間不足で FOPDT 同定できない場合は既存ゲインを保持する（既定挙動）。"""
        profile = make_profile()
        original_gains = profile.pid_gains
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()

        monkeypatch.setattr(
            "src.app.training_service.identify_fopdt",
            lambda logs, profile: None,  # noqa: ARG005
        )

        result = await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=profile.id,
            session_ids=["sess-1"],
            update_pid_gains=True,
        )

        assert result.pid_auto_tuned is False
        assert result.pid_gains == original_gains


class TestTrainAndApplyDynamicsParams:
    @pytest.mark.asyncio
    async def test_fopdt_success_sets_preview_and_fopdt_values(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """FOPDT同定成功時、pid_preview_s=0.0（θ 前倒し廃止）・fopdt_k/tau/theta が設定される。"""
        profile = make_profile()
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()

        monkeypatch.setattr(
            "src.app.training_service.identify_fopdt",
            lambda logs, profile: FOPDT(k=0.5, tau=2.0, theta=0.8),  # noqa: ARG005
        )
        monkeypatch.setattr(
            "src.app.training_service.compute_pid_gains_simc",
            lambda fopdt, profile, tau_c_factor=0.5: PIDGains(kp=9.0, ki=1.0, kd=0.0),  # noqa: ARG005
        )

        result = await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=profile.id,
            session_ids=["sess-1"],
            update_pid_gains=True,
        )

        # Stage A: pid_preview_s は θ に依らず 0.0（二重補償回避）。θ は fopdt_theta へ。
        assert result.dynamics_params.pid_preview_s == pytest.approx(0.0)
        assert result.dynamics_params.fopdt_k == pytest.approx(0.5)
        assert result.dynamics_params.fopdt_tau == pytest.approx(2.0)
        assert result.dynamics_params.fopdt_theta == pytest.approx(0.8)
        assert profile_repo.updated is not None
        assert profile_repo.updated.dynamics_params.pid_preview_s == pytest.approx(0.0)

    @pytest.mark.asyncio
    async def test_update_pid_gains_false_keeps_existing_dynamics_params(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """update_pid_gains=False（2段目の再学習）では dynamics_params を保持する。"""
        existing_dyn = DynamicsParams(
            pid_preview_s=1.1, fopdt_k=0.6, fopdt_tau=1.5, fopdt_theta=0.7
        )
        profile = make_profile()
        profile.dynamics_params = existing_dyn
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()

        def _fail_if_called(logs: object, profile: object) -> FOPDT | None:
            raise AssertionError("update_pid_gains=False では identify_fopdt を呼ばない")

        monkeypatch.setattr("src.app.training_service.identify_fopdt", _fail_if_called)

        result = await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=profile.id,
            session_ids=["sess-1"],
            update_pid_gains=False,
        )

        assert result.dynamics_params == existing_dyn

    @pytest.mark.asyncio
    async def test_fopdt_none_keeps_existing_dynamics_params(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """区間不足で FOPDT 同定できない場合は既存の dynamics_params を保持する。"""
        existing_dyn = DynamicsParams(pid_preview_s=0.9, fopdt_theta=0.9)
        profile = make_profile()
        profile.dynamics_params = existing_dyn
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()

        monkeypatch.setattr(
            "src.app.training_service.identify_fopdt",
            lambda logs, profile: None,  # noqa: ARG005
        )

        result = await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=profile.id,
            session_ids=["sess-1"],
            update_pid_gains=True,
        )

        assert result.dynamics_params == existing_dyn


class TestTrainAndApplyRefreshFailure:
    @pytest.mark.asyncio
    async def test_raises_runtime_error_when_refresh_fails(self) -> None:
        profile = make_profile()
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController(refresh_result=False)

        with pytest.raises(RuntimeError):
            await train_and_apply(
                profile_repo=profile_repo,
                session_repo=session_repo,
                controller=controller,
                profile_id=profile.id,
                session_ids=["sess-1"],
                update_pid_gains=False,
            )

    @pytest.mark.asyncio
    async def test_calls_refresh_with_updated_profile(self) -> None:
        profile = make_profile()
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController(refresh_result=True)

        await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=profile.id,
            session_ids=["sess-1"],
            update_pid_gains=False,
        )

        assert controller.refreshed_with is not None
        assert controller.refreshed_with.id == profile.id
        assert controller.refreshed_with.model_path == "data/models/fake.pkl"


class TestTrainAndApplyErrors:
    @pytest.mark.asyncio
    async def test_profile_not_found_raises_value_error(self) -> None:
        profile_repo = FakeProfileRepo(None)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()

        with pytest.raises(ValueError):
            await train_and_apply(
                profile_repo=profile_repo,
                session_repo=session_repo,
                controller=controller,
                profile_id="missing-id",
                session_ids=["sess-1"],
            )

    @pytest.mark.asyncio
    async def test_learning_data_error_propagates(self, monkeypatch: pytest.MonkeyPatch) -> None:
        profile = make_profile()
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()

        def _raise(
            logs: object,
            profile: object,
            output_dir: str = "data/models",
            feature_spec: object = None,
        ) -> object:
            raise LearningDataError("サンプル不足")

        monkeypatch.setattr("src.app.training_service.train_inverse_model", _raise)

        with pytest.raises(LearningDataError):
            await train_and_apply(
                profile_repo=profile_repo,
                session_repo=session_repo,
                controller=controller,
                profile_id=profile.id,
                session_ids=["sess-1"],
            )


class TestFeatureSpecFromSettings:
    def test_default_settings_produce_default_spec(self) -> None:
        spec = feature_spec_from_settings(ModelSettings())
        assert spec == FeatureSpec()

    def test_custom_settings_are_threaded_through(self) -> None:
        settings = ModelSettings(
            lookahead_horizons_s=(0.2, 0.5, 1.0),
            past_horizons_s=(0.2,),
            regime_horizon_s=0.5,
            include_v0_sq=False,
            include_dv_regime_x_v0=False,
            accel_horizons_s=(0.2,),
        )
        spec = feature_spec_from_settings(settings)
        assert spec.lookahead_horizons_s == (0.2, 0.5, 1.0)
        assert spec.past_horizons_s == (0.2,)
        assert spec.regime_horizon_s == 0.5
        assert spec.include_v0_sq is False
        assert spec.include_dv_regime_x_v0 is False
        assert spec.accel_horizons_s == (0.2,)

    @pytest.mark.asyncio
    async def test_train_and_apply_passes_feature_spec_through(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """train_and_apply の feature_spec が train_inverse_model に渡ること。"""
        profile = make_profile()
        profile_repo = FakeProfileRepo(profile)
        session_repo = FakeSessionRepo([make_log()])
        controller = FakeController()
        custom_spec = FeatureSpec(lookahead_horizons_s=(0.2, 0.5, 1.0), regime_horizon_s=0.5)
        captured: dict[str, object] = {}

        def _capture(
            logs: object,
            profile: object,
            output_dir: str = "data/models",
            feature_spec: object = None,
        ) -> tuple[str, dict]:
            captured["feature_spec"] = feature_spec
            return "data/models/fake.pkl", {"accel": {"n": 1.0}, "brake": {"n": 1.0}}

        monkeypatch.setattr("src.app.training_service.train_inverse_model", _capture)

        await train_and_apply(
            profile_repo=profile_repo,
            session_repo=session_repo,
            controller=controller,
            profile_id=profile.id,
            session_ids=["sess-1"],
            update_pid_gains=False,
            feature_spec=custom_spec,
        )

        assert captured["feature_spec"] is custom_spec
