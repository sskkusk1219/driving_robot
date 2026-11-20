from datetime import UTC, datetime

from src.models.drive_log import DriveLog, DriveLogData, DriveSession, LearningCycle
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.system_state import RobotState, SystemState

NOW = datetime(2026, 4, 20, 0, 0, 0, tzinfo=UTC)


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

    def test_cycle_id_defaults_to_none(self) -> None:
        session = DriveSession(
            id="session-uuid-3",
            profile_id="profile-uuid-1",
            mode_id=None,
            run_type="learning",
            started_at=NOW,
            ended_at=None,
            status="running",
        )
        assert session.cycle_id is None

    def test_cycle_id_set(self) -> None:
        session = DriveSession(
            id="session-uuid-4",
            profile_id="profile-uuid-1",
            mode_id=None,
            run_type="tuning",
            started_at=NOW,
            ended_at=None,
            status="running",
            cycle_id="cycle-uuid-1",
        )
        assert session.cycle_id == "cycle-uuid-1"


class TestLearningCycle:
    def test_fields(self) -> None:
        cycle = LearningCycle(
            id="cycle-uuid-1",
            profile_id="profile-uuid-1",
            status="running",
            started_at=NOW,
            ended_at=None,
            detail={},
        )
        assert cycle.status == "running"
        assert cycle.ended_at is None
        assert cycle.detail == {}
        assert cycle.session_count == 0

    def test_session_count(self) -> None:
        cycle = LearningCycle(
            id="cycle-uuid-2",
            profile_id="profile-uuid-1",
            status="completed",
            started_at=NOW,
            ended_at=NOW,
            detail={"stage1_gains": {"kp": 1.0}},
            session_count=11,
        )
        assert cycle.session_count == 11
        assert cycle.detail["stage1_gains"]["kp"] == 1.0


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
            "PAUSED",
            "MANUAL",
            "EMERGENCY",
            "ERROR",
        }
        actual = {s.value for s in RobotState}
        assert actual == expected
