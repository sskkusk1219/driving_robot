"""LearningDriveManager のユニットテスト。"""

from datetime import UTC, datetime

import pytest

from src.domain.learning_drive import (
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


# ---------------------------------------------------------------------------
# build_learning_reference テスト
# ---------------------------------------------------------------------------


class TestBuildLearningReference:
    def test_returns_driving_mode(self) -> None:
        from src.models.driving_mode import DrivingMode

        manager = make_manager()
        mode = manager.build_learning_reference(make_profile(max_speed=100.0))
        assert isinstance(mode, DrivingMode)
        assert mode.max_speed == 100.0

    def test_starts_and_ends_at_zero_speed(self) -> None:
        manager = make_manager()
        mode = manager.build_learning_reference(make_profile())
        assert mode.reference_speed[0].speed_kmh == 0.0
        assert mode.reference_speed[-1].speed_kmh == 0.0

    def test_times_are_monotonically_increasing(self) -> None:
        manager = make_manager()
        mode = manager.build_learning_reference(make_profile())
        times = [p.time_s for p in mode.reference_speed]
        assert times == sorted(times)
        assert mode.total_duration == times[-1]

    def test_covers_up_to_max_speed(self) -> None:
        manager = make_manager(speed_step=20.0)
        mode = manager.build_learning_reference(make_profile(max_speed=100.0))
        peak = max(p.speed_kmh for p in mode.reference_speed)
        assert peak == 100.0

    def test_never_exceeds_max_speed(self) -> None:
        manager = make_manager(speed_step=30.0)
        profile = make_profile(max_speed=100.0)
        mode = manager.build_learning_reference(profile)
        for p in mode.reference_speed:
            assert p.speed_kmh <= profile.max_speed + 1e-9

    def test_mode_id_is_not_persisted_uuid(self) -> None:
        """学習用は一時生成のため learning- プレフィックス付き ID とする。"""
        manager = make_manager()
        mode = manager.build_learning_reference(make_profile())
        assert mode.id.startswith("learning-")


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
