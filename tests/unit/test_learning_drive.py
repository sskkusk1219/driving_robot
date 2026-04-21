"""LearningDriveManager のユニットテスト。"""

import pickle
import tempfile
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import pytest

from src.domain.control.feedforward import FeedforwardController
from src.domain.learning_drive import (
    LearningDataError,
    LearningDriveConfig,
    LearningDriveManager,
)
from src.models.calibration import CalibrationData
from src.models.learning_drive import LearningLog, LearningPattern
from src.models.profile import PIDGains, StopConfig, VehicleProfile

# ---------------------------------------------------------------------------
# テスト用ヘルパー
# ---------------------------------------------------------------------------


def make_profile(
    max_speed: float = 100.0,
    max_accel_opening: float = 80.0,
    max_brake_opening: float = 80.0,
    max_decel_g: float = 0.5,
) -> VehicleProfile:
    return VehicleProfile(
        id="test-profile",
        name="TestProfile",
        max_accel_opening=max_accel_opening,
        max_brake_opening=max_brake_opening,
        max_speed=max_speed,
        max_decel_g=max_decel_g,
        pid_gains=PIDGains(kp=1.0, ki=0.1, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None,
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )


def make_calibration(
    accel_zero: int = 0,
    accel_full: int = 5000,
    brake_zero: int = 0,
    brake_full: int = 5000,
) -> CalibrationData:
    return CalibrationData(
        accel_zero_pos=accel_zero,
        accel_full_pos=accel_full,
        accel_stroke=accel_full - accel_zero,
        brake_zero_pos=brake_zero,
        brake_full_pos=brake_full,
        brake_stroke=brake_full - brake_zero,
        calibrated_at=datetime.now(tz=UTC),
        is_valid=True,
    )


def make_manager(
    speed_step: float = 20.0,
    accel_step: float = 2.0,
    accel_max: float = 4.0,
    hold_duration: float = 0.0,
    speed_sample_interval: float = 0.01,
) -> LearningDriveManager:
    cfg = LearningDriveConfig(
        speed_step_kmh=speed_step,
        accel_step_kmhs=accel_step,
        accel_max_kmhs=accel_max,
        hold_duration_s=hold_duration,
        speed_sample_interval_s=speed_sample_interval,
    )
    return LearningDriveManager(config=cfg)


class MockActuator:
    def __init__(self) -> None:
        self.positions_commanded: list[int] = []
        self._current_pos = 0

    async def move_to_position(self, pos: int) -> None:
        self.positions_commanded.append(pos)
        self._current_pos = pos

    async def read_position(self) -> int:
        return self._current_pos


class MockCAN:
    def __init__(self, speed: float = 50.0) -> None:
        self._speed = speed

    async def read_speed(self) -> float:
        return self._speed


def make_logs(n: int, speeds: list[float] | None = None) -> list[LearningLog]:
    """n 個の LearningLog を生成する。グリッドは 2×2 以上になるよう速度・加速度を分散させる。"""
    logs = []
    speed_list = [10.0, 20.0, 30.0, 40.0]
    accel_list = [0.0, 2.0, -1.0, 1.0]
    for i in range(n):
        s = speed_list[i % len(speed_list)]
        a = accel_list[i % len(accel_list)]
        actual = speeds[i] if speeds else s
        pattern = LearningPattern(
            speed_kmh=s,
            accel_kmhs=a,
            accel_opening=min(40.0 + i * 5, 80.0),
            brake_opening=0.0 if a >= 0 else 20.0,
            hold_duration_s=2.0,
        )
        logs.append(
            LearningLog(
                pattern=pattern,
                actual_speed_kmh=actual,
                accel_opening_applied=pattern.accel_opening,
                brake_opening_applied=pattern.brake_opening,
                recorded_at=datetime.now(tz=UTC),
            )
        )
    return logs


# ---------------------------------------------------------------------------
# generate_patterns テスト
# ---------------------------------------------------------------------------


class TestGeneratePatterns:
    def test_returns_non_empty_list(self) -> None:
        manager = make_manager()
        profile = make_profile()
        patterns = manager.generate_patterns(profile)
        assert len(patterns) > 0

    def test_accel_opening_within_max(self) -> None:
        manager = make_manager()
        profile = make_profile(max_accel_opening=60.0)
        for p in manager.generate_patterns(profile):
            assert p.accel_opening <= profile.max_accel_opening

    def test_brake_opening_within_max(self) -> None:
        manager = make_manager()
        profile = make_profile(max_brake_opening=50.0)
        for p in manager.generate_patterns(profile):
            assert p.brake_opening <= profile.max_brake_opening

    def test_decel_patterns_within_max_decel_g(self) -> None:
        manager = make_manager()
        profile = make_profile(max_decel_g=0.3)
        max_decel_kmhs = 0.3 * 9.81 * 3.6
        for p in manager.generate_patterns(profile):
            if p.accel_kmhs < 0:
                assert abs(p.accel_kmhs) <= max_decel_kmhs + 1e-6

    def test_speed_within_max_speed(self) -> None:
        manager = make_manager()
        profile = make_profile(max_speed=60.0)
        for p in manager.generate_patterns(profile):
            assert p.speed_kmh <= profile.max_speed + 1e-6

    def test_all_openings_non_negative(self) -> None:
        manager = make_manager()
        profile = make_profile()
        for p in manager.generate_patterns(profile):
            assert p.accel_opening >= 0.0
            assert p.brake_opening >= 0.0

    def test_hold_duration_uses_config(self) -> None:
        manager = make_manager(hold_duration=3.0)
        profile = make_profile()
        patterns = manager.generate_patterns(profile)
        assert all(p.hold_duration_s == pytest.approx(3.0) for p in patterns)


# ---------------------------------------------------------------------------
# run_pattern テスト
# ---------------------------------------------------------------------------


class TestRunPattern:
    @pytest.mark.asyncio
    async def test_actuators_receive_position_commands(self) -> None:
        manager = make_manager(hold_duration=0.0, speed_sample_interval=0.01)
        accel = MockActuator()
        brake = MockActuator()
        can = MockCAN(speed=30.0)
        calibration = make_calibration()

        pattern = LearningPattern(
            speed_kmh=30.0,
            accel_kmhs=0.0,
            accel_opening=40.0,
            brake_opening=0.0,
            hold_duration_s=0.0,
        )
        await manager.run_pattern(pattern, accel, brake, can, calibration)

        assert len(accel.positions_commanded) >= 1
        assert len(brake.positions_commanded) >= 1

    @pytest.mark.asyncio
    async def test_returns_learning_log(self) -> None:
        manager = make_manager(hold_duration=0.0, speed_sample_interval=0.01)
        accel = MockActuator()
        brake = MockActuator()
        can = MockCAN(speed=45.0)
        calibration = make_calibration()

        pattern = LearningPattern(
            speed_kmh=40.0,
            accel_kmhs=1.0,
            accel_opening=30.0,
            brake_opening=0.0,
            hold_duration_s=0.0,
        )
        log = await manager.run_pattern(pattern, accel, brake, can, calibration)

        assert isinstance(log, LearningLog)

    @pytest.mark.asyncio
    async def test_actual_speed_recorded_in_log(self) -> None:
        manager = make_manager(hold_duration=0.15, speed_sample_interval=0.05)
        accel = MockActuator()
        brake = MockActuator()
        can = MockCAN(speed=55.0)
        calibration = make_calibration()

        pattern = LearningPattern(
            speed_kmh=50.0,
            accel_kmhs=0.0,
            accel_opening=50.0,
            brake_opening=0.0,
            hold_duration_s=0.15,
        )
        log = await manager.run_pattern(pattern, accel, brake, can, calibration)

        assert log.actual_speed_kmh == pytest.approx(55.0)

    @pytest.mark.asyncio
    async def test_accel_pulse_computed_from_calibration(self) -> None:
        manager = make_manager(hold_duration=0.0, speed_sample_interval=0.01)
        accel = MockActuator()
        brake = MockActuator()
        can = MockCAN()
        calibration = make_calibration(accel_zero=100, accel_full=5100)

        pattern = LearningPattern(
            speed_kmh=20.0,
            accel_kmhs=0.0,
            accel_opening=50.0,
            brake_opening=0.0,
            hold_duration_s=0.0,
        )
        await manager.run_pattern(pattern, accel, brake, can, calibration)

        # 50% of stroke=5000 → 2500 + zero=100 → 2600
        assert accel.positions_commanded[0] == 2600


# ---------------------------------------------------------------------------
# train_model テスト
# ---------------------------------------------------------------------------


class TestTrainModel:
    def test_pkl_file_is_created(self) -> None:
        manager = make_manager()
        logs = make_logs(8)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = manager.train_model(logs, "test-profile", output_dir=tmpdir)
            assert Path(path).exists()

    def test_pkl_contains_required_keys(self) -> None:
        manager = make_manager()
        logs = make_logs(8)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = manager.train_model(logs, "test-profile", output_dir=tmpdir)
            with open(path, "rb") as f:
                data = pickle.load(f)  # noqa: S301
            assert "speed_grid" in data
            assert "accel_grid" in data
            assert "accel_map" in data
            assert "brake_map" in data

    def test_feedforward_controller_can_load_model(self) -> None:
        manager = make_manager()
        logs = make_logs(8)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = manager.train_model(logs, "test-profile", output_dir=tmpdir)
            ff = FeedforwardController()
            ff.load_model(path)
            accel_opening, brake_opening = ff.predict(15.0, 0.0)
            assert 0.0 <= accel_opening <= 100.0
            assert 0.0 <= brake_opening <= 100.0

    def test_raises_learning_data_error_when_logs_insufficient(self) -> None:
        manager = LearningDriveManager(LearningDriveConfig(min_logs_for_training=4))
        logs = make_logs(3)
        with tempfile.TemporaryDirectory() as tmpdir:
            with pytest.raises(LearningDataError):
                manager.train_model(logs, "test-profile", output_dir=tmpdir)

    def test_raises_learning_data_error_when_grid_insufficient(self) -> None:
        manager = make_manager()
        # 全ログが同一速度・同一加速度 → グリッドが 1×1
        pattern = LearningPattern(
            speed_kmh=30.0,
            accel_kmhs=0.0,
            accel_opening=40.0,
            brake_opening=0.0,
            hold_duration_s=2.0,
        )
        logs = [
            LearningLog(
                pattern=pattern,
                actual_speed_kmh=30.0,
                accel_opening_applied=40.0,
                brake_opening_applied=0.0,
                recorded_at=datetime.now(tz=UTC),
            )
            for _ in range(4)
        ]
        with tempfile.TemporaryDirectory() as tmpdir:
            with pytest.raises(LearningDataError):
                manager.train_model(logs, "test-profile", output_dir=tmpdir)

    def test_profile_id_with_path_separator_is_sanitized(self) -> None:
        manager = make_manager()
        logs = make_logs(8)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = manager.train_model(logs, "../evil/profile", output_dir=tmpdir)
            assert Path(path).parent == Path(tmpdir)

    def test_accel_map_shape_matches_grid(self) -> None:
        manager = make_manager()
        logs = make_logs(8)
        with tempfile.TemporaryDirectory() as tmpdir:
            path = manager.train_model(logs, "test-profile", output_dir=tmpdir)
            with open(path, "rb") as f:
                data = pickle.load(f)  # noqa: S301
            n_speed = len(np.unique([data["speed_grid"]]))
            n_accel = len(np.unique([data["accel_grid"]]))
            assert data["accel_map"].shape == (n_speed, n_accel)
            assert data["brake_map"].shape == (n_speed, n_accel)

    def test_nan_regions_filled_with_nearest_neighbor_not_zero(self) -> None:
        """griddata でグリッド端にNaNが生じても 0 ではなく最近傍値で補完されること。

        疎なログ（3速度×2加速度の6点）を使って3×3グリッドを構築すると
        凸包外に NaN が発生する。その NaN が 0 ではなく隣接値で埋まることを確認。
        """
        # 3速度 × 2加速度 = 6点のみのログ（3×3グリッドの凸包を外れる点が生まれる）
        speeds = [10.0, 20.0, 30.0]
        accels = [0.0, 2.0]
        logs = []
        for s in speeds:
            for a in accels:
                pattern = LearningPattern(
                    speed_kmh=s,
                    accel_kmhs=a,
                    accel_opening=s * 0.5,  # 速度比例の開度
                    brake_opening=0.0,
                    hold_duration_s=2.0,
                )
                logs.append(
                    LearningLog(
                        pattern=pattern,
                        actual_speed_kmh=s,
                        accel_opening_applied=pattern.accel_opening,
                        brake_opening_applied=pattern.brake_opening,
                        recorded_at=datetime.now(tz=UTC),
                    )
                )

        manager = make_manager()
        with tempfile.TemporaryDirectory() as tmpdir:
            path = manager.train_model(logs, "test-profile", output_dir=tmpdir)
            with open(path, "rb") as f:
                data = pickle.load(f)  # noqa: S301

        # マップに NaN が残っていないこと
        assert not np.any(np.isnan(data["accel_map"]))
        assert not np.any(np.isnan(data["brake_map"]))

        # 速度比例パターンなので最大速度点の accel_opening は正（0 で埋まっていない）
        max_speed_idx = len(data["speed_grid"]) - 1
        assert data["accel_map"][max_speed_idx, :].max() > 0.0
