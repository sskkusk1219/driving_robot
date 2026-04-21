"""CANReader のユニットテスト（python-can / cantools はモック化）。"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.infra.can_reader import CANReader


def _make_can_message(arbitration_id: int = 0x100, data: bytes = b"\x00" * 8) -> MagicMock:
    msg = MagicMock()
    msg.arbitration_id = arbitration_id
    msg.data = data
    return msg


class TestConnect:
    @pytest.mark.asyncio
    async def test_connect_without_dbc(self) -> None:
        mock_bus = MagicMock()
        with patch.dict("sys.modules", {"can": MagicMock()}):
            import can

            can.Bus.return_value = mock_bus  # type: ignore[attr-defined]

            reader = CANReader(interface="kvaser", channel=0, dbc_path=None)
            reader._bus = None

            with patch("asyncio.get_event_loop") as mock_loop_fn:
                mock_loop = MagicMock()
                mock_loop_fn.return_value = mock_loop
                mock_loop.run_in_executor = AsyncMock(return_value=mock_bus)

                await reader.connect()

            assert reader._bus is not None
            assert reader._db is None  # DBC 未指定

    @pytest.mark.asyncio
    async def test_connect_with_missing_dbc_raises(self, tmp_path: Path) -> None:
        dbc_path = tmp_path / "nonexistent.dbc"
        reader = CANReader(dbc_path=str(dbc_path))

        mock_bus = MagicMock()

        with patch.dict("sys.modules", {"can": MagicMock()}):
            with patch("asyncio.get_event_loop") as mock_loop_fn:
                mock_loop = MagicMock()
                mock_loop_fn.return_value = mock_loop
                mock_loop.run_in_executor = AsyncMock(return_value=mock_bus)

                with pytest.raises(FileNotFoundError):
                    await reader.connect()


class TestReadSpeed:
    @pytest.mark.asyncio
    async def test_read_speed_without_db_raises(self) -> None:
        reader = CANReader()
        reader._bus = MagicMock()
        reader._db = None  # DB 未ロード

        with pytest.raises(NotImplementedError):
            await reader.read_speed()

    @pytest.mark.asyncio
    async def test_read_speed_without_connect_raises(self) -> None:
        reader = CANReader()
        reader._bus = None
        reader._db = MagicMock()

        with pytest.raises(RuntimeError):
            await reader.read_speed()

    @pytest.mark.asyncio
    async def test_read_speed_success(self) -> None:
        reader = CANReader()
        mock_bus = MagicMock()
        mock_db = MagicMock()
        reader._bus = mock_bus
        reader._db = mock_db

        can_msg = _make_can_message(arbitration_id=0x100)
        mock_db.decode_message.return_value = {"VehicleSpeed": 72.5}

        with patch("asyncio.get_event_loop") as mock_loop_fn:
            mock_loop = MagicMock()
            mock_loop_fn.return_value = mock_loop
            mock_loop.run_in_executor = AsyncMock(return_value=can_msg)

            speed = await reader.read_speed()

        assert speed == 72.5

    @pytest.mark.asyncio
    async def test_read_speed_timeout_raises(self) -> None:
        reader = CANReader()
        reader._bus = MagicMock()
        reader._db = MagicMock()

        with patch("asyncio.get_event_loop") as mock_loop_fn:
            mock_loop = MagicMock()
            mock_loop_fn.return_value = mock_loop
            mock_loop.run_in_executor = AsyncMock(return_value=None)

            with pytest.raises(TimeoutError):
                await reader.read_speed()

    @pytest.mark.asyncio
    async def test_read_speed_missing_signal_raises(self) -> None:
        reader = CANReader()
        mock_bus = MagicMock()
        mock_db = MagicMock()
        reader._bus = mock_bus
        reader._db = mock_db

        can_msg = _make_can_message()
        mock_db.decode_message.return_value = {"OtherSignal": 10.0}

        with patch("asyncio.get_event_loop") as mock_loop_fn:
            mock_loop = MagicMock()
            mock_loop_fn.return_value = mock_loop
            mock_loop.run_in_executor = AsyncMock(return_value=can_msg)

            with pytest.raises(ValueError, match="VehicleSpeed"):
                await reader.read_speed()


class TestClose:
    @pytest.mark.asyncio
    async def test_close_shuts_down_bus(self) -> None:
        reader = CANReader()
        mock_bus = MagicMock()
        mock_bus.shutdown = MagicMock()
        reader._bus = mock_bus

        with patch("asyncio.get_event_loop") as mock_loop_fn:
            mock_loop = MagicMock()
            mock_loop_fn.return_value = mock_loop
            mock_loop.run_in_executor = AsyncMock()

            with patch.dict("sys.modules", {"can": MagicMock()}):
                import can

                can.BusABC = object  # type: ignore[attr-defined]
                await reader.close()

        assert reader._bus is None
