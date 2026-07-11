"""LogWriter ↔ PostgreSQL 統合テスト。

実行には TEST_DATABASE_URL 環境変数が必要。CI 対象外 (pytest.mark.integration)。

  export TEST_DATABASE_URL=postgresql://localhost/driving_robot_test
  pytest tests/integration/ -v -s -m integration
"""

import json
import os
from collections.abc import AsyncGenerator
from datetime import UTC, datetime

import asyncpg
import pytest

from src.infra.log_writer import LogWriter
from src.models.drive_log import DriveLogData

TEST_DATABASE_URL = os.environ.get("TEST_DATABASE_URL", "postgresql://localhost/driving_robot_test")

DUMMY_PROFILE_ID = "00000000-0000-0000-0000-000000000001"

_DUMMY_PROFILE_INSERT = """
    INSERT INTO vehicle_profiles (
        id, name, max_accel_opening, max_brake_opening, max_speed,
        max_decel_g, pid_gains, stop_config, created_at, updated_at
    ) VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
    ON CONFLICT (id) DO NOTHING
"""


@pytest.fixture
async def db_conn() -> AsyncGenerator[asyncpg.Connection]:  # type: ignore[type-arg]
    """テスト用 DB 接続。各テスト後にセッション・ログをクリーンアップする。"""
    conn = await asyncpg.connect(TEST_DATABASE_URL)
    now = datetime.now(UTC)
    await conn.execute(
        _DUMMY_PROFILE_INSERT,
        DUMMY_PROFILE_ID,
        "test-profile",
        100.0,
        100.0,
        200.0,
        0.5,
        '{"kp": 1.0, "ki": 0.0, "kd": 0.0}',
        '{"decel_g": 0.3}',
        now,
        now,
    )
    yield conn
    await conn.execute(
        "DELETE FROM drive_logs WHERE session_id IN "
        "(SELECT id FROM drive_sessions WHERE profile_id = $1)",
        DUMMY_PROFILE_ID,
    )
    await conn.execute("DELETE FROM drive_sessions WHERE profile_id = $1", DUMMY_PROFILE_ID)
    await conn.execute("DELETE FROM learning_cycles WHERE profile_id = $1", DUMMY_PROFILE_ID)
    await conn.execute("DELETE FROM vehicle_profiles WHERE id = $1", DUMMY_PROFILE_ID)
    await conn.close()


@pytest.mark.integration
@pytest.mark.asyncio
async def test_start_session_inserts_row(db_conn: asyncpg.Connection) -> None:
    """start_session が drive_sessions にレコードを INSERT すること。"""
    writer = LogWriter(db_conn)

    session_id = await writer.start_session(
        profile_id=DUMMY_PROFILE_ID,
        mode_id=None,
        run_type="manual",
    )

    row = await db_conn.fetchrow("SELECT * FROM drive_sessions WHERE id = $1", session_id)
    assert row is not None
    assert str(row["profile_id"]) == DUMMY_PROFILE_ID
    assert row["run_type"] == "manual"
    assert row["status"] == "running"
    assert row["ended_at"] is None


@pytest.mark.integration
@pytest.mark.asyncio
async def test_write_log_inserts_row_after_flush(db_conn: asyncpg.Connection) -> None:
    """write_log はバッファリングのみ行い、フラッシュ後に drive_logs へ INSERT されること
    （E5: 1サンプル=1INSERTのDBラウンドトリップ削減のためバッチ書き込みに変更）。"""
    writer = LogWriter(db_conn)
    session_id = await writer.start_session(
        profile_id=DUMMY_PROFILE_ID,
        mode_id=None,
        run_type="auto",
    )
    data = DriveLogData(
        ref_speed_kmh=60.0,
        actual_speed_kmh=59.5,
        accel_opening=42.0,
        brake_opening=0.0,
        accel_pos=1500,
        brake_pos=0,
        accel_current=850.0,
        brake_current=120.0,
    )

    await writer.write_log(session_id, data)
    await writer._flush_log_buffer()

    row = await db_conn.fetchrow("SELECT * FROM drive_logs WHERE session_id = $1", session_id)
    assert row is not None
    assert row["actual_speed_kmh"] == pytest.approx(59.5)
    assert row["accel_opening"] == pytest.approx(42.0)
    assert row["ref_speed_kmh"] == pytest.approx(60.0)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_write_log_row_appears_after_end_session_flush(
    db_conn: asyncpg.Connection,
) -> None:
    """E5 回帰テスト: フラッシュ間隔未達のまま end_session してもサンプルが失われないこと。"""
    writer = LogWriter(db_conn)
    session_id = await writer.start_session(
        profile_id=DUMMY_PROFILE_ID,
        mode_id=None,
        run_type="auto",
    )
    data = DriveLogData(
        ref_speed_kmh=60.0,
        actual_speed_kmh=59.5,
        accel_opening=42.0,
        brake_opening=0.0,
        accel_pos=1500,
        brake_pos=0,
        accel_current=850.0,
        brake_current=120.0,
    )

    await writer.write_log(session_id, data)
    await writer.end_session(session_id, "completed")

    row = await db_conn.fetchrow("SELECT * FROM drive_logs WHERE session_id = $1", session_id)
    assert row is not None
    assert row["actual_speed_kmh"] == pytest.approx(59.5)


@pytest.mark.integration
@pytest.mark.asyncio
async def test_end_session_updates_status(db_conn: asyncpg.Connection) -> None:
    """end_session が drive_sessions.ended_at と status を UPDATE すること。"""
    writer = LogWriter(db_conn)
    session_id = await writer.start_session(
        profile_id=DUMMY_PROFILE_ID,
        mode_id=None,
        run_type="auto",
    )

    await writer.end_session(session_id, "completed")

    row = await db_conn.fetchrow("SELECT * FROM drive_sessions WHERE id = $1", session_id)
    assert row is not None
    assert row["status"] == "completed"
    assert row["ended_at"] is not None


@pytest.mark.integration
@pytest.mark.asyncio
async def test_emergency_stop_sets_emergency_status(db_conn: asyncpg.Connection) -> None:
    """非常停止時に status='emergency' で end_session できること。"""
    writer = LogWriter(db_conn)
    session_id = await writer.start_session(
        profile_id=DUMMY_PROFILE_ID,
        mode_id=None,
        run_type="auto",
    )

    await writer.end_session(session_id, "emergency")

    row = await db_conn.fetchrow("SELECT status FROM drive_sessions WHERE id = $1", session_id)
    assert row is not None
    assert row["status"] == "emergency"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_start_session_persists_cycle_id(db_conn: asyncpg.Connection) -> None:
    """start_session に cycle_id を渡すと drive_sessions.cycle_id に永続化されること。"""
    writer = LogWriter(db_conn)
    cycle_id = await writer.start_cycle(profile_id=DUMMY_PROFILE_ID)

    session_id = await writer.start_session(
        profile_id=DUMMY_PROFILE_ID,
        mode_id=None,
        run_type="tuning",
        cycle_id=cycle_id,
    )

    row = await db_conn.fetchrow("SELECT * FROM drive_sessions WHERE id = $1", session_id)
    assert row is not None
    assert str(row["cycle_id"]) == cycle_id
    assert row["run_type"] == "tuning"


@pytest.mark.integration
@pytest.mark.asyncio
async def test_start_cycle_inserts_running_row(db_conn: asyncpg.Connection) -> None:
    """start_cycle が learning_cycles に status='running' で INSERT すること。"""
    writer = LogWriter(db_conn)

    cycle_id = await writer.start_cycle(profile_id=DUMMY_PROFILE_ID)

    row = await db_conn.fetchrow("SELECT * FROM learning_cycles WHERE id = $1", cycle_id)
    assert row is not None
    assert row["status"] == "running"
    assert row["ended_at"] is None


@pytest.mark.integration
@pytest.mark.asyncio
async def test_end_cycle_updates_status_and_detail(db_conn: asyncpg.Connection) -> None:
    """end_cycle が status/ended_at/detail を UPDATE すること。"""
    writer = LogWriter(db_conn)
    cycle_id = await writer.start_cycle(profile_id=DUMMY_PROFILE_ID)

    await writer.end_cycle(cycle_id, "completed", detail={"stage1_gains": {"kp": 1.0}})

    row = await db_conn.fetchrow("SELECT * FROM learning_cycles WHERE id = $1", cycle_id)
    assert row is not None
    assert row["status"] == "completed"
    assert row["ended_at"] is not None
    assert json.loads(row["detail"]) == {"stage1_gains": {"kp": 1.0}}


@pytest.mark.integration
@pytest.mark.asyncio
async def test_reap_closes_orphan_running_cycle(db_conn: asyncpg.Connection) -> None:
    """reap_interrupted_sessions が status='running' の孤児サイクルを 'error' で閉じること。"""
    writer = LogWriter(db_conn)
    cycle_id = await writer.start_cycle(profile_id=DUMMY_PROFILE_ID)

    await writer.reap_interrupted_sessions()

    row = await db_conn.fetchrow("SELECT * FROM learning_cycles WHERE id = $1", cycle_id)
    assert row is not None
    assert row["status"] == "error"
    assert row["ended_at"] is not None
