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
    # PID先読み補償(pid_preview_s)・FOPDT同定値(k, tau, theta)を保持する動特性パラメータ
    """
    ALTER TABLE vehicle_profiles
        ADD COLUMN IF NOT EXISTS dynamics_params JSONB
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
    # 学習サイクル（学習運転〜PID適合の全セッションを1サイクルに紐付ける）
    """
    CREATE TABLE IF NOT EXISTS learning_cycles (
        id          UUID PRIMARY KEY,
        profile_id  UUID NOT NULL REFERENCES vehicle_profiles(id),
        status      TEXT NOT NULL CHECK (status IN ('running','completed','error','aborted')),
        started_at  TIMESTAMPTZ NOT NULL,
        ended_at    TIMESTAMPTZ,
        detail      JSONB NOT NULL DEFAULT '{}'::jsonb
    )
    """,
    """
    ALTER TABLE drive_sessions
        ADD COLUMN IF NOT EXISTS cycle_id UUID REFERENCES learning_cycles(id)
    """,
    """
    CREATE INDEX IF NOT EXISTS idx_drive_sessions_cycle_id
        ON drive_sessions (cycle_id)
    """,
    # run_type CHECK制約に 'tuning'(PID適合走行) を追加する。Postgres は既存 CHECK 制約を
    # ADD CONSTRAINT IF NOT EXISTS で条件緩和できないため、旧制約を drop してから作り直す
    # （calibration_data_profile_id_key の DO ブロックと同方針）。
    """
    DO $$
    BEGIN
        ALTER TABLE drive_sessions DROP CONSTRAINT IF EXISTS drive_sessions_run_type_check;
        ALTER TABLE drive_sessions
            ADD CONSTRAINT drive_sessions_run_type_check
            CHECK (run_type IN ('auto', 'manual', 'learning', 'tuning'));
    END $$;
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
    # タイムスケジュール（統合タイムライン）: ペダル開度とボタンイベントを1エンティティで管理
    """
    CREATE TABLE IF NOT EXISTS time_schedules (
        id             UUID PRIMARY KEY,
        name           TEXT NOT NULL UNIQUE,
        description    TEXT NOT NULL DEFAULT '',
        pedal_points   JSONB NOT NULL,
        button_events  JSONB NOT NULL,
        total_duration DOUBLE PRECISION NOT NULL,
        created_at     TIMESTAMPTZ NOT NULL
    )
    """,
    # ループ再生機能を廃止したため既存DBから loop 列を削除
    """
    ALTER TABLE time_schedules
        DROP COLUMN IF EXISTS loop
    """,
    # 反復学習制御（ILC）テーブル: profile×mode 単位で時刻別補正 effort を永続化する。
    # 同一モードの反復走行で残差を学習し、次回走行に補正として適用する（Stage C）。
    """
    CREATE TABLE IF NOT EXISTS ilc_tables (
        profile_id  UUID NOT NULL REFERENCES vehicle_profiles(id) ON DELETE CASCADE,
        mode_id     UUID NOT NULL REFERENCES driving_modes(id) ON DELETE CASCADE,
        enabled     BOOLEAN NOT NULL DEFAULT TRUE,
        iteration   INTEGER NOT NULL,
        dt_s        DOUBLE PRECISION NOT NULL,
        efforts     JSONB NOT NULL,
        best_p95_kmh DOUBLE PRECISION,
        kpi_history JSONB NOT NULL DEFAULT '[]'::jsonb,
        updated_at  TIMESTAMPTZ NOT NULL,
        PRIMARY KEY (profile_id, mode_id)
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
