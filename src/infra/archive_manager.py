"""ログアーカイブ管理。

内蔵SSD容量が閾値を超えた際、古いセッションを CSV+gzip 圧縮して USB SSD へ移行する。
"""

from __future__ import annotations

import csv
import gzip
import logging
import shutil
from datetime import UTC, datetime, timedelta
from pathlib import Path

import asyncpg

from src.infra.settings import ArchiveSettings

logger = logging.getLogger(__name__)


class ArchiveManager:
    """走行ログの圧縮アーカイブと容量管理を担当するインフラコンポーネント。

    - 内蔵SSD使用率 > storage_limit_pct% のとき active_log_days 日超のセッションを移行
    - USB SSD 使用率 > storage_limit_pct% のとき最古アーカイブから削除
    - asyncpg.Connection は外部から注入（接続プール管理は呼び出し元が担当）
    """

    def __init__(self, conn: asyncpg.Connection, settings: ArchiveSettings) -> None:
        self._conn = conn
        self._settings = settings
        self._usb_path = Path(settings.usb_ssd_path)

    async def check_and_archive(self) -> None:
        """内蔵SSD使用率をチェックし、閾値超えならアーカイブを実行する。

        起動時・走行終了時に呼び出す（定期実行は行わない）。
        """
        db_path = await self._get_pg_data_path()
        usage = self._check_storage_usage(db_path)
        if usage >= self._settings.storage_limit_pct:
            logger.info(
                "Storage usage %.1f%% >= %.1f%%, starting archive.",
                usage,
                self._settings.storage_limit_pct,
            )
            await self._archive_old_sessions()
        else:
            logger.debug("Storage usage %.1f%%, no archive needed.", usage)

    async def _archive_old_sessions(self) -> None:
        """保持日数を超えたセッションをCSV+gzip圧縮してUSB SSDへ移行し、DBから削除する。"""
        self._usb_path.mkdir(parents=True, exist_ok=True)

        cutoff = datetime.now(tz=UTC) - timedelta(days=self._settings.active_log_days)
        sessions = await self._conn.fetch(
            """
            SELECT id, started_at
            FROM drive_sessions
            WHERE ended_at IS NOT NULL AND ended_at < $1
            ORDER BY ended_at ASC
            """,
            cutoff,
        )

        if not sessions:
            logger.info("No sessions to archive.")
            return

        for session in sessions:
            session_id: str = str(session["id"])
            started_at: datetime = session["started_at"]
            await self._archive_session(session_id, started_at)

        self._cleanup_usb_ssd_if_needed()

    async def _archive_session(self, session_id: str, started_at: datetime) -> None:
        """単一セッションをCSV+gzip圧縮して移行し、DBから削除する。"""
        rows = await self._conn.fetch(
            """
            SELECT timestamp, ref_speed_kmh, actual_speed_kmh,
                   accel_opening, brake_opening, accel_pos, brake_pos,
                   accel_current, brake_current
            FROM drive_logs
            WHERE session_id = $1
            ORDER BY timestamp ASC
            """,
            session_id,
        )

        filename = f"{session_id}_{started_at.strftime('%Y%m%d_%H%M%S')}.csv"
        csv_path = self._usb_path / filename
        self._export_to_csv(rows, csv_path)
        gz_path = self._compress(csv_path)
        logger.info("Archived session %s → %s", session_id, gz_path)

        await self._delete_session_from_db(session_id)

    def _export_to_csv(self, rows: list[asyncpg.Record], path: Path) -> None:
        """走行ログレコードを CSV ファイルに書き出す。"""
        fieldnames = [
            "timestamp", "ref_speed_kmh", "actual_speed_kmh",
            "accel_opening", "brake_opening", "accel_pos", "brake_pos",
            "accel_current", "brake_current",
        ]
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(dict(row))

    def _compress(self, csv_path: Path) -> Path:
        """CSV を gzip 圧縮し、元ファイルを削除して .gz パスを返す。"""
        gz_path = csv_path.with_suffix(".csv.gz")
        with csv_path.open("rb") as f_in, gzip.open(gz_path, "wb") as f_out:
            shutil.copyfileobj(f_in, f_out)
        csv_path.unlink()
        return gz_path

    def _check_storage_usage(self, path: Path) -> float:
        """指定パスのストレージ使用率 [%] を返す。"""
        usage = shutil.disk_usage(path)
        return usage.used / usage.total * 100.0

    def _cleanup_usb_ssd_if_needed(self) -> None:
        """USB SSD 使用率が閾値超えなら最古のアーカイブファイルから削除する。"""
        if not self._usb_path.exists():
            return
        while True:
            usage = self._check_storage_usage(self._usb_path)
            if usage < self._settings.storage_limit_pct:
                break
            gz_files = sorted(self._usb_path.glob("*.csv.gz"), key=lambda p: p.stat().st_mtime)
            if not gz_files:
                logger.warning("USB SSD usage %.1f%% but no archive files to delete.", usage)
                break
            oldest = gz_files[0]
            oldest.unlink()
            logger.warning("USB SSD usage %.1f%%, deleted oldest archive: %s", usage, oldest.name)

    async def _delete_session_from_db(self, session_id: str) -> None:
        """drive_logs → drive_sessions の順にレコードを削除する（FK制約順）。"""
        await self._conn.execute(
            "DELETE FROM drive_logs WHERE session_id = $1",
            session_id,
        )
        await self._conn.execute(
            "DELETE FROM drive_sessions WHERE id = $1",
            session_id,
        )

    async def _get_pg_data_path(self) -> Path:
        """PostgreSQL データディレクトリのパスを取得する。"""
        row = await self._conn.fetchrow("SHOW data_directory")
        if row is None:
            return Path("/var/lib/postgresql")
        return Path(str(row["data_directory"]))
