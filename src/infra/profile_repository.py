"""VehicleProfile・CalibrationData を PostgreSQL へ永続化する。"""

from __future__ import annotations

import json
import logging
import uuid
from dataclasses import fields
from datetime import UTC, datetime
from typing import Any

import asyncpg

from src.infra.db import DuplicateNameError
from src.models.calibration import CalibrationData
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile

logger = logging.getLogger(__name__)


def _ffp_from_value(value: object) -> FeedforwardParams:
    """JSONB 値（str/dict/None）を FeedforwardParams に変換する。欠損キーはデフォルト。"""
    data: dict[str, Any]
    if isinstance(value, str):
        data = json.loads(value)
    elif isinstance(value, dict):
        data = value
    else:
        return FeedforwardParams()
    defaults = FeedforwardParams()
    # 旧レコードに存在しないキー（後から追加した調停定数等）はデフォルトで補完する
    kwargs = {
        f.name: data.get(f.name, getattr(defaults, f.name)) for f in fields(FeedforwardParams)
    }
    return FeedforwardParams(**kwargs)


def _ffp_to_json(ffp: FeedforwardParams) -> str:
    return json.dumps({f.name: getattr(ffp, f.name) for f in fields(FeedforwardParams)})


def _row_to_profile(row: asyncpg.Record) -> VehicleProfile:
    pid = json.loads(row["pid_gains"])
    stop = json.loads(row["stop_config"])
    return VehicleProfile(
        id=str(row["id"]),
        name=row["name"],
        max_accel_opening=row["max_accel_opening"],
        max_brake_opening=row["max_brake_opening"],
        max_speed=row["max_speed"],
        max_decel_g=row["max_decel_g"],
        pid_gains=PIDGains(kp=pid["kp"], ki=pid["ki"], kd=pid["kd"]),
        stop_config=StopConfig(
            deviation_threshold_kmh=stop["deviation_threshold_kmh"],
            deviation_duration_s=stop["deviation_duration_s"],
        ),
        calibration=None,
        model_path=row["model_path"],
        created_at=row["created_at"],
        updated_at=row["updated_at"],
        feedforward_params=_ffp_from_value(row["feedforward_params"]),
    )


class ProfileRepository:
    """vehicle_profiles + calibration_data の CRUD を担うリポジトリ。

    asyncpg.Pool を外部から受け取り、操作ごとに接続を acquire/release する。
    """

    def __init__(self, pool: asyncpg.Pool) -> None:
        self._pool = pool

    async def list_all(self) -> list[VehicleProfile]:
        """全プロファイルをキャリブレーションデータ付きで返す。"""
        async with self._pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT p.*, c.accel_zero_pos, c.accel_full_pos, c.accel_stroke,
                       c.brake_zero_pos, c.brake_full_pos, c.brake_stroke,
                       c.calibrated_at AS calib_at, c.is_valid
                FROM vehicle_profiles p
                LEFT JOIN calibration_data c ON c.profile_id = p.id
                ORDER BY p.created_at DESC
                """
            )
        return [_row_to_profile_with_calib(row) for row in rows]

    async def get_by_id(self, profile_id: str) -> VehicleProfile | None:
        """ID でプロファイルを取得する。存在しない場合は None。"""
        async with self._pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT p.*, c.accel_zero_pos, c.accel_full_pos, c.accel_stroke,
                       c.brake_zero_pos, c.brake_full_pos, c.brake_stroke,
                       c.calibrated_at AS calib_at, c.is_valid
                FROM vehicle_profiles p
                LEFT JOIN calibration_data c ON c.profile_id = p.id
                WHERE p.id = $1
                """,
                uuid.UUID(profile_id),
            )
        if row is None:
            return None
        return _row_to_profile_with_calib(row)

    async def create(self, profile: VehicleProfile) -> VehicleProfile:
        """プロファイルを新規作成する。"""
        now = datetime.now(tz=UTC)
        profile_id = profile.id if profile.id else str(uuid.uuid4())
        pid_json = json.dumps(
            {"kp": profile.pid_gains.kp, "ki": profile.pid_gains.ki, "kd": profile.pid_gains.kd}
        )
        stop_json = json.dumps(
            {
                "deviation_threshold_kmh": profile.stop_config.deviation_threshold_kmh,
                "deviation_duration_s": profile.stop_config.deviation_duration_s,
            }
        )
        ffp_json = _ffp_to_json(profile.feedforward_params)
        async with self._pool.acquire() as conn:
            try:
                await conn.execute(
                    """
                    INSERT INTO vehicle_profiles
                        (id, name, max_accel_opening, max_brake_opening, max_speed,
                         max_decel_g, pid_gains, stop_config, model_path,
                         feedforward_params, created_at, updated_at)
                    VALUES ($1, $2, $3, $4, $5, $6, $7::jsonb, $8::jsonb, $9, $10::jsonb, $11, $12)
                    """,
                    uuid.UUID(profile_id),
                    profile.name,
                    profile.max_accel_opening,
                    profile.max_brake_opening,
                    profile.max_speed,
                    profile.max_decel_g,
                    pid_json,
                    stop_json,
                    profile.model_path,
                    ffp_json,
                    now,
                    now,
                )
            except asyncpg.UniqueViolationError as e:
                raise DuplicateNameError(
                    f"プロファイル名 {profile.name!r} は既に使用されています"
                ) from e
        created = VehicleProfile(
            id=profile_id,
            name=profile.name,
            max_accel_opening=profile.max_accel_opening,
            max_brake_opening=profile.max_brake_opening,
            max_speed=profile.max_speed,
            max_decel_g=profile.max_decel_g,
            pid_gains=profile.pid_gains,
            stop_config=profile.stop_config,
            calibration=None,
            model_path=profile.model_path,
            created_at=now,
            updated_at=now,
            feedforward_params=profile.feedforward_params,
        )
        return created

    async def update(self, profile: VehicleProfile) -> VehicleProfile | None:
        """プロファイルを更新する。存在しない場合は None。"""
        now = datetime.now(tz=UTC)
        pid_json = json.dumps(
            {"kp": profile.pid_gains.kp, "ki": profile.pid_gains.ki, "kd": profile.pid_gains.kd}
        )
        stop_json = json.dumps(
            {
                "deviation_threshold_kmh": profile.stop_config.deviation_threshold_kmh,
                "deviation_duration_s": profile.stop_config.deviation_duration_s,
            }
        )
        ffp_json = _ffp_to_json(profile.feedforward_params)
        async with self._pool.acquire() as conn:
            result = await conn.execute(
                """
                UPDATE vehicle_profiles SET
                    name = $2,
                    max_accel_opening = $3,
                    max_brake_opening = $4,
                    max_speed = $5,
                    max_decel_g = $6,
                    pid_gains = $7::jsonb,
                    stop_config = $8::jsonb,
                    model_path = $9,
                    feedforward_params = $10::jsonb,
                    updated_at = $11
                WHERE id = $1
                """,
                uuid.UUID(profile.id),
                profile.name,
                profile.max_accel_opening,
                profile.max_brake_opening,
                profile.max_speed,
                profile.max_decel_g,
                pid_json,
                stop_json,
                profile.model_path,
                ffp_json,
                now,
            )
        if str(result) == "UPDATE 0":
            return None
        return VehicleProfile(
            id=profile.id,
            name=profile.name,
            max_accel_opening=profile.max_accel_opening,
            max_brake_opening=profile.max_brake_opening,
            max_speed=profile.max_speed,
            max_decel_g=profile.max_decel_g,
            pid_gains=profile.pid_gains,
            stop_config=profile.stop_config,
            calibration=profile.calibration,
            model_path=profile.model_path,
            created_at=profile.created_at,
            updated_at=now,
            feedforward_params=profile.feedforward_params,
        )

    async def delete(self, profile_id: str) -> bool:
        """プロファイルを削除する。削除できた場合 True。

        calibration_data / drive_sessions / drive_logs は profile_id を参照する
        外部キー制約(いずれも ON DELETE CASCADE 無し)を持つため、プロファイル本体
        より先に下位テーブルから順に削除する。drive_logs は drive_sessions を経由
        して紐づくので最初に消す。全削除を同一トランザクションで実行し、途中失敗時に
        一部だけ消える状態を防ぐ。

        注意: プロファイル削除は、そのプロファイルに紐づく走行セッションとログ履歴
        もすべて削除する。
        """
        pid = uuid.UUID(profile_id)
        async with self._pool.acquire() as conn:
            async with conn.transaction():
                await conn.execute(
                    """
                    DELETE FROM drive_logs
                    WHERE session_id IN (
                        SELECT id FROM drive_sessions WHERE profile_id = $1
                    )
                    """,
                    pid,
                )
                await conn.execute(
                    "DELETE FROM drive_sessions WHERE profile_id = $1",
                    pid,
                )
                await conn.execute(
                    "DELETE FROM calibration_data WHERE profile_id = $1",
                    pid,
                )
                result = await conn.execute(
                    "DELETE FROM vehicle_profiles WHERE id = $1",
                    pid,
                )
        return str(result) != "DELETE 0"

    async def save_calibration(self, profile_id: str, data: CalibrationData) -> None:
        """profile_id に対応する calibration_data を UPSERT する。

        同一 profile_id のレコードが既に存在する場合は全フィールドを上書きする。
        """
        record_id = str(uuid.uuid4())
        try:
            async with self._pool.acquire() as conn:
                await conn.execute(
                    """
                    INSERT INTO calibration_data
                        (id, profile_id, accel_zero_pos, accel_full_pos, accel_stroke,
                         brake_zero_pos, brake_full_pos, brake_stroke, calibrated_at, is_valid)
                    VALUES
                        ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
                    ON CONFLICT (profile_id) DO UPDATE SET
                        accel_zero_pos = EXCLUDED.accel_zero_pos,
                        accel_full_pos = EXCLUDED.accel_full_pos,
                        accel_stroke   = EXCLUDED.accel_stroke,
                        brake_zero_pos = EXCLUDED.brake_zero_pos,
                        brake_full_pos = EXCLUDED.brake_full_pos,
                        brake_stroke   = EXCLUDED.brake_stroke,
                        calibrated_at  = EXCLUDED.calibrated_at,
                        is_valid       = EXCLUDED.is_valid
                    """,
                    record_id,
                    uuid.UUID(profile_id),
                    data.accel_zero_pos,
                    data.accel_full_pos,
                    data.accel_stroke,
                    data.brake_zero_pos,
                    data.brake_full_pos,
                    data.brake_stroke,
                    data.calibrated_at,
                    data.is_valid,
                )
        except Exception:
            logger.exception(
                "キャリブレーションデータの保存に失敗しました (profile_id=%s)", profile_id
            )
            raise


def _row_to_profile_with_calib(row: asyncpg.Record) -> VehicleProfile:
    """JOIN 結果行を VehicleProfile (CalibrationData 付き) に変換する。"""
    profile = _row_to_profile(row)
    if row["accel_zero_pos"] is not None:
        profile.calibration = CalibrationData(
            accel_zero_pos=row["accel_zero_pos"],
            accel_full_pos=row["accel_full_pos"],
            accel_stroke=row["accel_stroke"],
            brake_zero_pos=row["brake_zero_pos"],
            brake_full_pos=row["brake_full_pos"],
            brake_stroke=row["brake_stroke"],
            calibrated_at=row["calib_at"],
            is_valid=row["is_valid"],
        )
    return profile
