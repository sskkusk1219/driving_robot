"""PostgreSQL テーブル・インデックスを作成する初期化スクリプト。冪等実行可能（IF NOT EXISTS）。"""

import asyncio
import os

import asyncpg


DDL_STATEMENTS = [
    """
    CREATE TABLE IF NOT EXISTS vehicle_profiles (
        id          UUID PRIMARY KEY,
        name        TEXT NOT NULL UNIQUE,
        max_accel_opening DOUBLE PRECISION NOT NULL,
        max_brake_opening DOUBLE PRECISION NOT NULL,
        max_speed   DOUBLE PRECISION NOT NULL,
        max_decel_g DOUBLE PRECISION NOT NULL,
        pid_gains   JSONB NOT NULL,
        stop_config JSONB NOT NULL,
        model_path  TEXT,
        feedforward_params JSONB,
        created_at  TIMESTAMPTZ NOT NULL,
        updated_at  TIMESTAMPTZ NOT NULL
    )
    """,
    # 既存DB向けマイグレーション（CREATE TABLE IF NOT EXISTS は列追加しないため）
    """
    ALTER TABLE vehicle_profiles
        ADD COLUMN IF NOT EXISTS feedforward_params JSONB
    """,
    """
    CREATE TABLE IF NOT EXISTS calibration_data (
        id              UUID PRIMARY KEY,
        profile_id      UUID NOT NULL UNIQUE REFERENCES vehicle_profiles(id),
        accel_zero_pos  INTEGER NOT NULL,
        accel_full_pos  INTEGER NOT NULL,
        accel_stroke    INTEGER NOT NULL,
        brake_zero_pos  INTEGER NOT NULL,
        brake_full_pos  INTEGER NOT NULL,
        brake_stroke    INTEGER NOT NULL,
        calibrated_at   TIMESTAMPTZ NOT NULL,
        is_valid        BOOLEAN NOT NULL
    )
    """,
    # 既存DB向けマイグレーション: UNIQUE(profile_id) 制約を冪等に追加する。
    # 旧 setup_db で UNIQUE 無しに作成されたテーブルは CREATE TABLE IF NOT EXISTS では
    # 変更されず、save_calibration の ON CONFLICT(profile_id) が失敗するため。
    # Postgres は ADD CONSTRAINT IF NOT EXISTS 非対応のため DO ブロックで存在チェックする。
    """
    DO $$
    BEGIN
        IF NOT EXISTS (
            SELECT 1 FROM pg_constraint WHERE conname = 'calibration_data_profile_id_key'
        ) THEN
            ALTER TABLE calibration_data
                ADD CONSTRAINT calibration_data_profile_id_key UNIQUE (profile_id);
        END IF;
    END $$;
    """,
    """
    CREATE TABLE IF NOT EXISTS driving_modes (
        id              UUID PRIMARY KEY,
        name            TEXT NOT NULL UNIQUE,
        description     TEXT NOT NULL DEFAULT '',
        reference_speed JSONB NOT NULL,
        total_duration  DOUBLE PRECISION NOT NULL,
        max_speed       DOUBLE PRECISION NOT NULL,
        created_at      TIMESTAMPTZ NOT NULL
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS drive_sessions (
        id          UUID PRIMARY KEY,
        profile_id  UUID NOT NULL REFERENCES vehicle_profiles(id),
        mode_id     UUID REFERENCES driving_modes(id),
        run_type    TEXT NOT NULL CHECK (run_type IN ('auto', 'manual', 'learning')),
        started_at  TIMESTAMPTZ NOT NULL,
        ended_at    TIMESTAMPTZ,
        status      TEXT NOT NULL CHECK (status IN ('running', 'completed', 'error', 'emergency'))
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS drive_logs (
        id                BIGSERIAL PRIMARY KEY,
        session_id        UUID NOT NULL REFERENCES drive_sessions(id),
        timestamp         TIMESTAMPTZ NOT NULL,
        ref_speed_kmh     DOUBLE PRECISION,
        actual_speed_kmh  DOUBLE PRECISION NOT NULL,
        accel_opening     DOUBLE PRECISION NOT NULL,
        brake_opening     DOUBLE PRECISION NOT NULL,
        accel_pos         INTEGER NOT NULL,
        brake_pos         INTEGER NOT NULL,
        accel_current     DOUBLE PRECISION NOT NULL,
        brake_current     DOUBLE PRECISION NOT NULL
    )
    """,
    # architecture.md 定義の3インデックス
    """
    CREATE INDEX IF NOT EXISTS idx_drive_logs_session_timestamp
        ON drive_logs (session_id, timestamp DESC)
    """,
    """
    CREATE INDEX IF NOT EXISTS idx_drive_sessions_started_at
        ON drive_sessions (started_at DESC)
    """,
    """
    CREATE INDEX IF NOT EXISTS idx_drive_sessions_ended_at
        ON drive_sessions (ended_at ASC)
    """,
]


async def setup(dsn: str) -> None:
    conn = await asyncpg.connect(dsn)
    try:
        for stmt in DDL_STATEMENTS:
            await conn.execute(stmt)
        print("DB setup completed.")
    finally:
        await conn.close()


if __name__ == "__main__":
    dsn = os.environ.get("DATABASE_URL", "postgresql://localhost/driving_robot")
    asyncio.run(setup(dsn))
