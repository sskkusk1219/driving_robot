import uuid
from unittest.mock import AsyncMock

import pytest

from src.infra.log_writer import LogWriter
from src.models.drive_log import DriveLogData


def make_conn() -> AsyncMock:
    conn = AsyncMock()
    conn.execute = AsyncMock()
    return conn


def sample_log_data() -> DriveLogData:
    return DriveLogData(
        ref_speed_kmh=60.0,
        actual_speed_kmh=59.5,
        accel_opening=42.0,
        brake_opening=0.0,
        accel_pos=1500,
        brake_pos=0,
        accel_current=850.0,
        brake_current=120.0,
    )


class TestLogWriterStartSession:
    @pytest.mark.asyncio
    async def test_start_session_returns_uuid_string(self) -> None:
        """start_session が UUID 形式の文字列を返すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        session_id = await writer.start_session(
            profile_id="prof-uuid",
            mode_id="mode-uuid",
            run_type="auto",
        )

        uuid.UUID(session_id)  # UUID として解析できれば形式が正しい

    @pytest.mark.asyncio
    async def test_start_session_calls_execute_once(self) -> None:
        """start_session が conn.execute を1回呼ぶこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.start_session(
            profile_id="prof-uuid",
            mode_id=None,
            run_type="manual",
        )

        conn.execute.assert_called_once()

    @pytest.mark.asyncio
    async def test_start_session_passes_profile_id(self) -> None:
        """start_session が profile_id を SQL の $2 に渡すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.start_session(
            profile_id="expected-profile-id",
            mode_id=None,
            run_type="auto",
        )

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[2] == "expected-profile-id"

    @pytest.mark.asyncio
    async def test_start_session_passes_none_mode_id(self) -> None:
        """mode_id が None の場合、そのまま SQL に渡すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.start_session(
            profile_id="prof-uuid",
            mode_id=None,
            run_type="learning",
        )

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[3] is None  # mode_id = $3

    @pytest.mark.asyncio
    async def test_start_session_passes_run_type(self) -> None:
        """run_type が SQL に正しく渡されること。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.start_session(
            profile_id="prof-uuid",
            mode_id=None,
            run_type="learning",
        )

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[4] == "learning"  # run_type = $4


class TestLogWriterWriteLog:
    @pytest.mark.asyncio
    async def test_write_log_calls_execute_once(self) -> None:
        """write_log が conn.execute を1回呼ぶこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.write_log("session-uuid", sample_log_data())

        conn.execute.assert_called_once()

    @pytest.mark.asyncio
    async def test_write_log_passes_session_id(self) -> None:
        """write_log が session_id を $1 に渡すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.write_log("my-session-id", sample_log_data())

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[1] == "my-session-id"

    @pytest.mark.asyncio
    async def test_write_log_passes_actual_speed(self) -> None:
        """write_log が actual_speed_kmh を SQL に渡すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)
        data = sample_log_data()

        await writer.write_log("session-uuid", data)

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[3] == data.actual_speed_kmh  # actual_speed_kmh = $3

    @pytest.mark.asyncio
    async def test_write_log_passes_none_ref_speed(self) -> None:
        """ref_speed_kmh が None の場合、そのまま渡すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)
        data = sample_log_data()
        data.ref_speed_kmh = None

        await writer.write_log("session-uuid", data)

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[2] is None  # ref_speed_kmh = $2


class TestLogWriterEndSession:
    @pytest.mark.asyncio
    async def test_end_session_calls_execute_once(self) -> None:
        """end_session が conn.execute を1回呼ぶこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.end_session("session-uuid", "completed")

        conn.execute.assert_called_once()

    @pytest.mark.asyncio
    async def test_end_session_passes_session_id(self) -> None:
        """end_session が session_id を $1 に渡すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.end_session("target-session", "completed")

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[1] == "target-session"

    @pytest.mark.asyncio
    async def test_end_session_passes_status(self) -> None:
        """end_session が status を $2 に渡すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.end_session("session-uuid", "emergency")

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[2] == "emergency"

    @pytest.mark.asyncio
    async def test_end_session_all_valid_statuses(self) -> None:
        """すべての有効な status 値で end_session が呼べること。"""
        valid_statuses = ["completed", "error", "emergency"]
        for status in valid_statuses:
            conn = make_conn()
            writer = LogWriter(conn)
            await writer.end_session("session-uuid", status)
            conn.execute.assert_called_once()
