import json
import uuid
from unittest.mock import AsyncMock

import pytest

from src.infra.log_writer import LogWriter
from src.models.drive_log import DriveLogData


def make_conn() -> AsyncMock:
    conn = AsyncMock()
    conn.execute = AsyncMock()
    conn.executemany = AsyncMock()
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

    @pytest.mark.asyncio
    async def test_start_session_cycle_id_defaults_to_none(self) -> None:
        """cycle_id 省略時は None が SQL に渡ること。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.start_session(profile_id="prof-uuid", mode_id=None, run_type="learning")

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[5] is None  # cycle_id = $5

    @pytest.mark.asyncio
    async def test_start_session_passes_cycle_id(self) -> None:
        """cycle_id 指定時にそのまま SQL へ渡ること。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.start_session(
            profile_id="prof-uuid", mode_id=None, run_type="tuning", cycle_id="cycle-uuid-1"
        )

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[5] == "cycle-uuid-1"


class TestLogWriterWriteLog:
    """E5: write_log はバッファに追加するのみで、フラッシュ（executemany）は
    LOG_FLUSH_INTERVAL_S 経過後または明示的な _flush_log_buffer 呼び出しで発生する。"""

    @pytest.mark.asyncio
    async def test_write_log_does_not_call_db_immediately(self) -> None:
        """write_log 単体では conn.execute/executemany を呼ばない（バッファリングのみ）。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.write_log("session-uuid", sample_log_data())

        conn.execute.assert_not_called()
        conn.executemany.assert_not_called()
        assert len(writer._log_buffer) == 1

    @pytest.mark.asyncio
    async def test_flush_passes_session_id(self) -> None:
        """フラッシュ時に session_id が行の $1 として executemany に渡ること。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.write_log("my-session-id", sample_log_data())
        await writer._flush_log_buffer()

        rows = conn.executemany.call_args[0][1]
        assert rows[0][0] == "my-session-id"

    @pytest.mark.asyncio
    async def test_flush_passes_actual_speed(self) -> None:
        """フラッシュ時に actual_speed_kmh が行に含まれること。"""
        conn = make_conn()
        writer = LogWriter(conn)
        data = sample_log_data()

        await writer.write_log("session-uuid", data)
        await writer._flush_log_buffer()

        rows = conn.executemany.call_args[0][1]
        assert rows[0][3] == data.actual_speed_kmh  # actual_speed_kmh = $4（$2 に timestamp）

    @pytest.mark.asyncio
    async def test_flush_passes_none_ref_speed(self) -> None:
        """ref_speed_kmh が None の場合、そのまま渡すこと。"""
        conn = make_conn()
        writer = LogWriter(conn)
        data = sample_log_data()
        data.ref_speed_kmh = None

        await writer.write_log("session-uuid", data)
        await writer._flush_log_buffer()

        rows = conn.executemany.call_args[0][1]
        assert rows[0][2] is None  # ref_speed_kmh = $3

    @pytest.mark.asyncio
    async def test_flush_with_empty_buffer_does_not_call_db(self) -> None:
        """バッファが空の場合、フラッシュしても executemany は呼ばれない。"""
        conn = make_conn()
        writer = LogWriter(conn)

        await writer._flush_log_buffer()

        conn.executemany.assert_not_called()

    @pytest.mark.asyncio
    async def test_multiple_writes_batch_into_single_flush(self) -> None:
        """複数回の write_log が1回の executemany にまとめられること（往復削減、E5 指摘）。"""
        conn = make_conn()
        writer = LogWriter(conn)

        for _ in range(5):
            await writer.write_log("session-uuid", sample_log_data())
        await writer._flush_log_buffer()

        conn.executemany.assert_called_once()
        rows = conn.executemany.call_args[0][1]
        assert len(rows) == 5

    @pytest.mark.asyncio
    async def test_write_log_auto_flushes_after_interval_elapsed(self) -> None:
        """LOG_FLUSH_INTERVAL_S 経過後の write_log 呼び出しで自動フラッシュされること。"""
        from src.infra.log_writer import LOG_FLUSH_INTERVAL_S

        conn = make_conn()
        writer = LogWriter(conn)

        await writer.write_log("session-uuid", sample_log_data())
        conn.executemany.assert_not_called()
        # 直前のフラッシュ時刻を過去へずらし、間隔経過を模擬する
        assert writer._last_flush_at is not None
        writer._last_flush_at -= LOG_FLUSH_INTERVAL_S + 0.1

        await writer.write_log("session-uuid", sample_log_data())

        conn.executemany.assert_called_once()
        rows = conn.executemany.call_args[0][1]
        assert len(rows) == 2


class TestLogWriterEndSession:
    @pytest.mark.asyncio
    async def test_end_session_flushes_pending_log_buffer_first(self) -> None:
        """E5 回帰テスト: フラッシュ間隔未達でもセッション終了時にバッファが失われないこと。"""
        conn = make_conn()
        writer = LogWriter(conn)
        await writer.write_log("session-uuid", sample_log_data())

        await writer.end_session("session-uuid", "completed")

        conn.executemany.assert_called_once()
        assert writer._log_buffer == []

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


class TestLogWriterStartCycle:
    @pytest.mark.asyncio
    async def test_start_cycle_returns_uuid_string(self) -> None:
        conn = make_conn()
        writer = LogWriter(conn)

        cycle_id = await writer.start_cycle(profile_id="prof-uuid")

        uuid.UUID(cycle_id)

    @pytest.mark.asyncio
    async def test_start_cycle_calls_execute_once(self) -> None:
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.start_cycle(profile_id="prof-uuid")

        conn.execute.assert_called_once()

    @pytest.mark.asyncio
    async def test_start_cycle_passes_profile_id(self) -> None:
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.start_cycle(profile_id="expected-profile-id")

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[2] == "expected-profile-id"


class TestLogWriterEndCycle:
    @pytest.mark.asyncio
    async def test_end_cycle_calls_execute_once(self) -> None:
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.end_cycle("cycle-uuid", "completed")

        conn.execute.assert_called_once()

    @pytest.mark.asyncio
    async def test_end_cycle_passes_cycle_id_and_status(self) -> None:
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.end_cycle("target-cycle", "aborted")

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert positional_args[1] == "target-cycle"
        assert positional_args[2] == "aborted"

    @pytest.mark.asyncio
    async def test_end_cycle_serializes_detail_as_json(self) -> None:
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.end_cycle("cycle-uuid", "completed", detail={"stage1": {"kp": 1.0}})

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert json.loads(positional_args[3]) == {"stage1": {"kp": 1.0}}

    @pytest.mark.asyncio
    async def test_end_cycle_defaults_detail_to_empty_dict(self) -> None:
        conn = make_conn()
        writer = LogWriter(conn)

        await writer.end_cycle("cycle-uuid", "error")

        call_args = conn.execute.call_args
        positional_args = call_args[0]
        assert json.loads(positional_args[3]) == {}


class TestLogWriterReapInterruptedSessions:
    @pytest.mark.asyncio
    async def test_reap_returns_count(self) -> None:
        """asyncpg の 'UPDATE <n>' から是正件数を返すこと。"""
        conn = make_conn()
        conn.execute = AsyncMock(return_value="UPDATE 2")
        writer = LogWriter(conn)

        reaped = await writer.reap_interrupted_sessions()

        assert reaped == 2

    @pytest.mark.asyncio
    async def test_reap_zero_when_none(self) -> None:
        """是正対象が無ければ 0 を返すこと。"""
        conn = make_conn()
        conn.execute = AsyncMock(return_value="UPDATE 0")
        writer = LogWriter(conn)

        assert await writer.reap_interrupted_sessions() == 0

    @pytest.mark.asyncio
    async def test_reap_handles_empty_result(self) -> None:
        """execute が None/空でも例外を投げず 0 を返すこと。"""
        conn = make_conn()
        conn.execute = AsyncMock(return_value=None)
        writer = LogWriter(conn)

        assert await writer.reap_interrupted_sessions() == 0

    @pytest.mark.asyncio
    async def test_reap_also_closes_orphan_cycles(self) -> None:
        """孤児 learning_cycles(status='running') も 'error' で回収すること（2回 execute）。"""
        conn = make_conn()
        conn.execute = AsyncMock(return_value="UPDATE 1")
        writer = LogWriter(conn)

        await writer.reap_interrupted_sessions()

        assert conn.execute.await_count == 2
        cycle_sql = conn.execute.await_args_list[1][0][0]
        assert "learning_cycles" in cycle_sql
        assert "'error'" in cycle_sql
