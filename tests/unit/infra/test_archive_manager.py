"""ArchiveManager のユニットテスト（DB・ファイルシステムはモック化）。"""

from __future__ import annotations

import gzip
from datetime import UTC, datetime
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.infra.archive_manager import ArchiveManager
from src.infra.settings import ArchiveSettings


def make_settings(tmp_path: Path, storage_limit_pct: float = 80.0) -> ArchiveSettings:
    return ArchiveSettings(
        usb_ssd_path=str(tmp_path),
        active_log_days=90,
        storage_limit_pct=storage_limit_pct,
    )


def make_conn() -> AsyncMock:
    conn = AsyncMock()
    conn.execute = AsyncMock()
    conn.fetch = AsyncMock(return_value=[])
    conn.fetchrow = AsyncMock(return_value={"data_directory": "/var/lib/postgresql/15/main"})
    return conn


def make_fake_record(**kwargs: object) -> MagicMock:
    record = MagicMock()
    record.__getitem__ = lambda self, key: kwargs[key]

    def _iter() -> object:
        yield from kwargs.items()

    record.__iter__ = _iter
    return record


class TestCheckAndArchive:
    @pytest.mark.asyncio
    async def test_does_not_archive_when_usage_below_threshold(self, tmp_path: Path) -> None:
        """使用率が閾値未満のときアーカイブを実行しないこと。"""
        conn = make_conn()
        settings = make_settings(tmp_path, storage_limit_pct=80.0)
        manager = ArchiveManager(conn, settings)

        with patch("src.infra.archive_manager.shutil.disk_usage") as mock_usage:
            mock_usage.return_value = MagicMock(used=700, total=1000)
            await manager.check_and_archive()

        conn.fetch.assert_not_called()

    @pytest.mark.asyncio
    async def test_archives_when_usage_equals_threshold(self, tmp_path: Path) -> None:
        """使用率が閾値ちょうどのときアーカイブを実行すること。"""
        conn = make_conn()
        settings = make_settings(tmp_path, storage_limit_pct=80.0)
        manager = ArchiveManager(conn, settings)

        with patch("src.infra.archive_manager.shutil.disk_usage") as mock_usage:
            mock_usage.return_value = MagicMock(used=800, total=1000)
            await manager.check_and_archive()

        conn.fetch.assert_called_once()

    @pytest.mark.asyncio
    async def test_archives_when_usage_above_threshold(self, tmp_path: Path) -> None:
        """使用率が閾値超のときアーカイブを実行すること。"""
        conn = make_conn()
        settings = make_settings(tmp_path, storage_limit_pct=80.0)
        manager = ArchiveManager(conn, settings)

        with patch("src.infra.archive_manager.shutil.disk_usage") as mock_usage:
            mock_usage.return_value = MagicMock(used=900, total=1000)
            await manager.check_and_archive()

        conn.fetch.assert_called_once()


class TestCheckStorageUsage:
    def test_returns_correct_percentage(self, tmp_path: Path) -> None:
        """disk_usage の結果から正しい使用率を返すこと。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        with patch("src.infra.archive_manager.shutil.disk_usage") as mock_usage:
            mock_usage.return_value = MagicMock(used=300, total=1000)
            result = manager._check_storage_usage(tmp_path)

        assert result == pytest.approx(30.0)

    def test_returns_100_when_full(self, tmp_path: Path) -> None:
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        with patch("src.infra.archive_manager.shutil.disk_usage") as mock_usage:
            mock_usage.return_value = MagicMock(used=1000, total=1000)
            result = manager._check_storage_usage(tmp_path)

        assert result == pytest.approx(100.0)


class TestExportToCsv:
    def test_creates_csv_with_header(self, tmp_path: Path) -> None:
        """CSV ファイルが正しいヘッダーで作成されること。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        row = make_fake_record(
            timestamp=datetime(2026, 1, 1, tzinfo=UTC),
            ref_speed_kmh=60.0,
            actual_speed_kmh=59.5,
            accel_opening=42.0,
            brake_opening=0.0,
            accel_pos=1500,
            brake_pos=0,
            accel_current=850.0,
            brake_current=120.0,
        )
        csv_path = tmp_path / "test.csv"
        manager._export_to_csv([row], csv_path)

        assert csv_path.exists()
        content = csv_path.read_text(encoding="utf-8")
        assert "timestamp" in content
        assert "actual_speed_kmh" in content

    def test_creates_empty_csv_with_header_for_no_rows(self, tmp_path: Path) -> None:
        """ログなしセッションでも CSV ヘッダーが書き出されること。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        csv_path = tmp_path / "empty.csv"
        manager._export_to_csv([], csv_path)

        assert csv_path.exists()
        lines = csv_path.read_text(encoding="utf-8").strip().splitlines()
        assert len(lines) == 1  # ヘッダー行のみ


class TestCompress:
    def test_creates_gz_file(self, tmp_path: Path) -> None:
        """_compress が .csv.gz ファイルを作成すること。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        csv_path = tmp_path / "test.csv"
        csv_path.write_text("col1,col2\nval1,val2\n", encoding="utf-8")

        gz_path = manager._compress(csv_path)

        assert gz_path.exists()
        assert gz_path.suffix == ".gz"

    def test_removes_original_csv(self, tmp_path: Path) -> None:
        """_compress が元の CSV ファイルを削除すること。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        csv_path = tmp_path / "test.csv"
        csv_path.write_text("col1,col2\nval1,val2\n", encoding="utf-8")

        manager._compress(csv_path)

        assert not csv_path.exists()

    def test_gz_content_is_readable(self, tmp_path: Path) -> None:
        """圧縮ファイルが gzip で正しく読み戻せること。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        original_content = "col1,col2\nval1,val2\n"
        csv_path = tmp_path / "test.csv"
        csv_path.write_text(original_content, encoding="utf-8")

        gz_path = manager._compress(csv_path)

        with gzip.open(gz_path, "rt", encoding="utf-8") as f:
            content = f.read()
        assert content == original_content


class TestCleanupUsbSsd:
    def test_does_nothing_when_below_threshold(self, tmp_path: Path) -> None:
        """使用率が閾値未満のとき削除しないこと。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path, storage_limit_pct=80.0))

        gz_file = tmp_path / "old.csv.gz"
        gz_file.write_bytes(b"data")

        with patch("src.infra.archive_manager.shutil.disk_usage") as mock_usage:
            mock_usage.return_value = MagicMock(used=700, total=1000)
            manager._cleanup_usb_ssd_if_needed()

        assert gz_file.exists()

    def test_deletes_oldest_when_above_threshold(self, tmp_path: Path) -> None:
        """使用率が閾値超のとき最古ファイルを削除すること。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path, storage_limit_pct=80.0))

        old_file = tmp_path / "old.csv.gz"
        new_file = tmp_path / "new.csv.gz"
        old_file.write_bytes(b"old")
        new_file.write_bytes(b"new")
        # 確実に古い方を古くする
        import os
        os.utime(old_file, (0, 0))

        call_count = 0

        def mock_disk_usage(path: Path) -> MagicMock:
            nonlocal call_count
            call_count += 1
            if call_count == 1:
                return MagicMock(used=900, total=1000)
            return MagicMock(used=700, total=1000)

        with patch("src.infra.archive_manager.shutil.disk_usage", side_effect=mock_disk_usage):
            manager._cleanup_usb_ssd_if_needed()

        assert not old_file.exists()
        assert new_file.exists()


class TestDeleteSessionFromDb:
    @pytest.mark.asyncio
    async def test_deletes_logs_before_session(self, tmp_path: Path) -> None:
        """drive_logs → drive_sessions の順に DELETE が呼ばれること。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        await manager._delete_session_from_db("test-session-id")

        assert conn.execute.call_count == 2
        first_call_sql: str = conn.execute.call_args_list[0][0][0]
        second_call_sql: str = conn.execute.call_args_list[1][0][0]
        assert "drive_logs" in first_call_sql
        assert "drive_sessions" in second_call_sql

    @pytest.mark.asyncio
    async def test_passes_session_id_to_both_deletes(self, tmp_path: Path) -> None:
        """session_id が両方の DELETE に正しく渡されること。"""
        conn = make_conn()
        manager = ArchiveManager(conn, make_settings(tmp_path))

        await manager._delete_session_from_db("my-session")

        for call in conn.execute.call_args_list:
            assert call[0][1] == "my-session"
