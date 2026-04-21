from datetime import UTC, datetime

from src.models.calibration import CalibrationData
from src.models.drive_log import DriveLog, DriveLogData, DriveSession
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import PIDGains, StopConfig, VehicleProfile
from src.models.system_state import RobotState, SystemState

NOW = datetime(2026, 4, 20, 0, 0, 0, tzinfo=UTC)


def make_calibration() -> CalibrationData:
    return CalibrationData(
        accel_zero_pos=100,
        accel_full_pos=500,
        accel_stroke=400,
        brake_zero_pos=200,
        brake_full_pos=600,
        brake_stroke=400,
        calibrated_at=NOW,
        is_valid=True,
    )


def make_profile(calibration: CalibrationData | None = None) -> VehicleProfile:
    return VehicleProfile(
        id="profile-uuid-1",
        name="Prius_2024",
        max_accel_opening=80.0,
        max_brake_opening=90.0,
        max_speed=180.0,
        max_decel_g=0.5,
        pid_gains=PIDGains(kp=1.0, ki=0.1, kd=0.01),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=calibration,
        model_path=None,
        created_at=NOW,
        updated_at=NOW,
    )


class TestPIDGains:
    def test_fields(self) -> None:
        g = PIDGains(kp=1.0, ki=0.5, kd=0.1)
        assert g.kp == 1.0
        assert g.ki == 0.5
        assert g.kd == 0.1


class TestStopConfig:
    def test_fields(self) -> None:
        cfg = StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0)
        assert cfg.deviation_threshold_kmh == 2.0
        assert cfg.deviation_duration_s == 4.0


class TestVehicleProfile:
    def test_fields_without_calibration(self) -> None:
        p = make_profile()
        assert p.name == "Prius_2024"
        assert p.max_accel_opening == 80.0
        assert p.calibration is None
        assert p.model_path is None

    def test_fields_with_calibration(self) -> None:
        cal = make_calibration()
        p = make_profile(calibration=cal)
        assert p.calibration is not None
        assert p.calibration.is_valid is True


class TestCalibrationData:
    def test_stroke_fields(self) -> None:
        cal = make_calibration()
        assert cal.accel_full_pos > cal.accel_zero_pos
        assert cal.brake_full_pos > cal.brake_zero_pos
        assert cal.accel_stroke == cal.accel_full_pos - cal.accel_zero_pos
        assert cal.brake_stroke == cal.brake_full_pos - cal.brake_zero_pos

    def test_is_valid_flag(self) -> None:
        cal = make_calibration()
        assert cal.is_valid is True


class TestDrivingMode:
    def test_fields(self) -> None:
        points = [SpeedPoint(time_s=0.0, speed_kmh=0.0), SpeedPoint(time_s=10.0, speed_kmh=50.0)]
        mode = DrivingMode(
            id="mode-uuid-1",
            name="WLTP_Class3",
            description="WLTPクラス3テストサイクル",
            reference_speed=points,
            total_duration=1800.0,
            max_speed=131.3,
            created_at=NOW,
        )
        assert mode.name == "WLTP_Class3"
        assert len(mode.reference_speed) == 2
        assert mode.reference_speed[0].speed_kmh == 0.0

    def test_speed_point(self) -> None:
        p = SpeedPoint(time_s=5.0, speed_kmh=30.0)
        assert p.time_s == 5.0
        assert p.speed_kmh == 30.0


class TestDriveSession:
    def test_fields(self) -> None:
        session = DriveSession(
            id="session-uuid-1",
            profile_id="profile-uuid-1",
            mode_id="mode-uuid-1",
            run_type="auto",
            started_at=NOW,
            ended_at=None,
            status="running",
        )
        assert session.run_type == "auto"
        assert session.ended_at is None
        assert session.status == "running"

    def test_manual_run_type(self) -> None:
        session = DriveSession(
            id="session-uuid-2",
            profile_id="profile-uuid-1",
            mode_id=None,
            run_type="manual",
            started_at=NOW,
            ended_at=NOW,
            status="completed",
        )
        assert session.mode_id is None
        assert session.run_type == "manual"


class TestDriveLog:
    def test_fields(self) -> None:
        log = DriveLog(
            id=1,
            session_id="session-uuid-1",
            timestamp=NOW,
            ref_speed_kmh=60.0,
            actual_speed_kmh=59.5,
            accel_opening=42.0,
            brake_opening=0.0,
            accel_pos=420,
            brake_pos=0,
            accel_current=850.0,
            brake_current=120.0,
        )
        assert log.ref_speed_kmh == 60.0
        assert log.actual_speed_kmh == 59.5

    def test_ref_speed_none_for_manual(self) -> None:
        log = DriveLog(
            id=2,
            session_id="session-uuid-2",
            timestamp=NOW,
            ref_speed_kmh=None,
            actual_speed_kmh=40.0,
            accel_opening=30.0,
            brake_opening=0.0,
            accel_pos=300,
            brake_pos=0,
            accel_current=600.0,
            brake_current=100.0,
        )
        assert log.ref_speed_kmh is None


class TestDriveLogData:
    def test_fields(self) -> None:
        data = DriveLogData(
            ref_speed_kmh=60.0,
            actual_speed_kmh=59.5,
            accel_opening=42.0,
            brake_opening=0.0,
            accel_pos=420,
            brake_pos=0,
            accel_current=850.0,
            brake_current=120.0,
        )
        assert data.actual_speed_kmh == 59.5


class TestSystemState:
    def test_ready_state(self) -> None:
        state = SystemState(
            robot_state=RobotState.READY,
            active_profile_id="profile-uuid-1",
            active_session_id=None,
            last_normal_shutdown=True,
            updated_at=NOW,
        )
        assert state.robot_state == RobotState.READY
        assert state.robot_state.value == "READY"

    def test_emergency_state(self) -> None:
        state = SystemState(
            robot_state=RobotState.EMERGENCY,
            active_profile_id=None,
            active_session_id=None,
            last_normal_shutdown=False,
            updated_at=NOW,
        )
        assert state.robot_state == RobotState.EMERGENCY

    def test_all_states_defined(self) -> None:
        expected = {
            "BOOTING",
            "STANDBY",
            "INITIALIZING",
            "READY",
            "CALIBRATING",
            "PRE_CHECK",
            "RUNNING",
            "MANUAL",
            "EMERGENCY",
            "ERROR",
        }
        actual = {s.value for s in RobotState}
        assert actual == expected
