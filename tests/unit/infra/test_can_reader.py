"""CANReader のユニットテスト（python-can / cantools はモック化）。"""

from __future__ import annotations

import asyncio
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.infra.can_reader import DEFAULT_MAX_SPEED_AGE_S, CANReader

_DBC_PATH = Path("config/can/MEIDEN_MEIDACS.dbc")


def _make_can_message(arbitration_id: int = 0x100, data: bytes = b"\x00" * 8) -> MagicMock:
    msg = MagicMock()
    msg.arbitration_id = arbitration_id
    msg.data = data
    msg.is_error_frame = False
    return msg


def _setup_consumer(reader: CANReader, frames: list[MagicMock]) -> asyncio.Task[None]:
    """常駐コンシューマを手動起動するヘルパー。

    与えたフレームを順に返し、尽きたら永久待機する get_message を構成し、
    _consume_frames タスクを起動して返す（テスト終了時に cancel すること）。
    """
    pending = list(frames)

    async def fake_get_message() -> MagicMock:
        if pending:
            return pending.pop(0)
        await asyncio.Event().wait()
        raise AssertionError("unreachable")

    reader._async_reader = MagicMock()
    reader._async_reader.get_message = fake_get_message
    reader._first_frame = asyncio.Event()
    return asyncio.ensure_future(reader._consume_frames())


class TestConnect:
    @pytest.mark.asyncio
    async def test_connect_without_dbc(self) -> None:
        mock_bus = MagicMock()
        mock_can = MagicMock()
        mock_can.Bus.return_value = mock_bus
        mock_can.AsyncBufferedReader.return_value = MagicMock()
        mock_can.Notifier.return_value = MagicMock()

        with patch.dict("sys.modules", {"can": mock_can}):
            reader = CANReader(interface="kvaser", channel=0, dbc_path=None)

            with patch("asyncio.get_event_loop") as mock_loop_fn:
                mock_loop = MagicMock()
                mock_loop_fn.return_value = mock_loop
                mock_loop.run_in_executor = AsyncMock(return_value=mock_bus)

                await reader.connect()

        assert reader._bus is not None
        assert reader._db is None  # DBC 未指定
        assert reader._async_reader is not None
        assert reader._notifier is not None

    @pytest.mark.asyncio
    async def test_connect_with_missing_dbc_raises(self, tmp_path: Path) -> None:
        dbc_path = tmp_path / "nonexistent.dbc"
        reader = CANReader(dbc_path=str(dbc_path))

        mock_bus = MagicMock()
        mock_can = MagicMock()
        mock_can.Bus.return_value = mock_bus

        with patch.dict("sys.modules", {"can": mock_can}):
            with patch("asyncio.get_event_loop") as mock_loop_fn:
                mock_loop = MagicMock()
                mock_loop_fn.return_value = mock_loop
                mock_loop.run_in_executor = AsyncMock(return_value=mock_bus)

                with pytest.raises(FileNotFoundError):
                    await reader.connect()

        # I6 回帰テスト: DBC 読み込みは Bus オープンより先に行うため、DBC が無い場合は
        # can.Bus が一切呼ばれず、ハンドルがリークしない（reader._bus も None のまま）。
        mock_loop.run_in_executor.assert_not_called()
        assert reader._bus is None


class TestReadSpeed:
    @pytest.mark.asyncio
    async def test_read_speed_without_db_raises(self) -> None:
        reader = CANReader()
        reader._async_reader = MagicMock()
        reader._db = None

        with pytest.raises(NotImplementedError):
            await reader.read_speed()

    @pytest.mark.asyncio
    async def test_read_speed_without_connect_raises(self) -> None:
        reader = CANReader()
        reader._async_reader = None
        reader._db = MagicMock()

        with pytest.raises(RuntimeError):
            await reader.read_speed()

    @pytest.mark.asyncio
    async def test_read_speed_success(self) -> None:
        reader = CANReader()
        mock_db = MagicMock()
        reader._db = mock_db

        can_msg = _make_can_message(arbitration_id=0x120)
        mock_db.decode_message.return_value = {"Speed": 72.5}
        task = _setup_consumer(reader, [can_msg])
        try:
            speed = await reader.read_speed()
        finally:
            task.cancel()

        assert speed == 72.5

    @pytest.mark.asyncio
    async def test_read_speed_skips_unknown_frame_id(self) -> None:
        """不明な ID のフレームはスキップして次の Speed フレームを待つ。"""
        reader = CANReader()
        mock_db = MagicMock()
        reader._db = mock_db

        unknown_msg = _make_can_message(arbitration_id=0x999)
        speed_msg = _make_can_message(arbitration_id=0x120)

        mock_db.decode_message.side_effect = [KeyError("unknown"), {"Speed": 50.0}]
        task = _setup_consumer(reader, [unknown_msg, speed_msg])
        try:
            speed = await reader.read_speed()
        finally:
            task.cancel()

        assert speed == 50.0

    @pytest.mark.asyncio
    async def test_read_speed_skips_frame_without_speed_signal(self) -> None:
        """Speed シグナルを含まないフレームはスキップして次を待つ。"""
        reader = CANReader()
        mock_db = MagicMock()
        reader._db = mock_db

        other_msg = _make_can_message(arbitration_id=0x100)
        speed_msg = _make_can_message(arbitration_id=0x120)

        mock_db.decode_message.side_effect = [{"OtherSignal": 10.0}, {"Speed": 30.0}]
        task = _setup_consumer(reader, [other_msg, speed_msg])
        try:
            speed = await reader.read_speed()
        finally:
            task.cancel()

        assert speed == 30.0

    @pytest.mark.asyncio
    async def test_read_speed_returns_latest_not_oldest(self) -> None:
        """複数フレーム受信後は最古ではなく最新の車速を返す（陳腐化防止）。"""
        reader = CANReader()
        mock_db = MagicMock()
        reader._db = mock_db

        frames = [_make_can_message(arbitration_id=0x120) for _ in range(3)]
        mock_db.decode_message.side_effect = [
            {"Speed": 10.0},
            {"Speed": 20.0},
            {"Speed": 30.0},
        ]
        task = _setup_consumer(reader, frames)
        try:
            # コンシューマが全フレームを処理するまで待つ
            await asyncio.sleep(0.05)
            speed = await reader.read_speed()
        finally:
            task.cancel()

        assert speed == 30.0

    @pytest.mark.asyncio
    async def test_read_speed_stale_cache_raises_timeout(self) -> None:
        """キャッシュが鮮度上限を超えて古い場合は TimeoutError（バス無音→非常停止）。"""
        reader = CANReader()
        mock_db = MagicMock()
        reader._db = mock_db

        can_msg = _make_can_message(arbitration_id=0x120)
        mock_db.decode_message.return_value = {"Speed": 40.0}
        task = _setup_consumer(reader, [can_msg])
        try:
            speed = await reader.read_speed()
            assert speed == 40.0
            # 受信時刻を鮮度上限より過去へずらしてバス無音を模擬する
            reader._latest_at -= 10.0
            with pytest.raises(TimeoutError):
                await reader.read_speed()
        finally:
            task.cancel()

    @pytest.mark.asyncio
    async def test_default_freshness_bound_is_tight(self) -> None:
        """既定の鮮度上限は 0.2s（制御 4 周期）。0.3s 古いキャッシュは拒否される。

        1.0s では凍結車速のまま最大 20 周期制御が走り、減速ランプ中に KPI ハード上限
        （1.0km/h）超の偏差が逸脱チェックに見えなくなる（レビュー指摘 #2）。
        """
        assert DEFAULT_MAX_SPEED_AGE_S == pytest.approx(0.2)
        reader = CANReader()
        mock_db = MagicMock()
        reader._db = mock_db
        mock_db.decode_message.return_value = {"Speed": 40.0}
        task = _setup_consumer(reader, [_make_can_message(arbitration_id=0x120)])
        try:
            await reader.read_speed()
            reader._latest_at -= 0.3
            with pytest.raises(TimeoutError):
                await reader.read_speed()
        finally:
            task.cancel()

    @pytest.mark.asyncio
    async def test_freshness_bound_configurable(self) -> None:
        """max_speed_age_s を緩めた場合は同じ古さでも許容される（設定可能）。"""
        reader = CANReader(max_speed_age_s=1.0)
        mock_db = MagicMock()
        reader._db = mock_db
        mock_db.decode_message.return_value = {"Speed": 40.0}
        task = _setup_consumer(reader, [_make_can_message(arbitration_id=0x120)])
        try:
            await reader.read_speed()
            reader._latest_at -= 0.3
            assert await reader.read_speed() == 40.0
        finally:
            task.cancel()


class TestClose:
    @pytest.mark.asyncio
    async def test_close_shuts_down_bus_and_notifier(self) -> None:
        reader = CANReader()
        mock_bus = MagicMock()
        mock_notifier = MagicMock()
        reader._bus = mock_bus
        reader._notifier = mock_notifier

        with patch("asyncio.get_event_loop") as mock_loop_fn:
            mock_loop = MagicMock()
            mock_loop_fn.return_value = mock_loop
            mock_loop.run_in_executor = AsyncMock()

            await reader.close()

        mock_notifier.stop.assert_called_once()
        assert reader._bus is None
        assert reader._notifier is None


class TestDbcIntegration:
    """実際の DBC ファイルを使った統合テスト。"""

    def test_dbc_loads_and_has_speed_signal(self) -> None:
        import cantools

        db = cantools.database.load_file(str(_DBC_PATH))
        msg = db.get_message_by_name("MEIDACS_Frame0")
        signal_names = [s.name for s in msg.signals]
        assert "Speed" in signal_names

    def test_dbc_speed_signal_unit_kmh(self) -> None:
        import cantools

        db = cantools.database.load_file(str(_DBC_PATH))
        msg = db.get_message_by_name("MEIDACS_Frame0")
        speed_signal = next(s for s in msg.signals if s.name == "Speed")
        assert speed_signal.unit == "km/h"

    def test_dbc_frame_id_is_288(self) -> None:
        import cantools

        db = cantools.database.load_file(str(_DBC_PATH))
        msg = db.get_message_by_name("MEIDACS_Frame0")
        assert msg.frame_id == 288

    @pytest.mark.asyncio
    async def test_read_speed_with_real_dbc(self) -> None:
        """実際の DBC ファイルをロードし、Speed=100.0 km/h のフレームをデコードする。"""
        import cantools

        db = cantools.database.load_file(str(_DBC_PATH))
        msg_def = db.get_message_by_name("MEIDACS_Frame0")

        # Speed=100.0 km/h (factor=0.01 なので raw=10000), dummy=0.0 でエンコード
        raw_data = msg_def.encode({"Speed": 100.0, "dummy": 0.0})

        reader = CANReader(dbc_path=str(_DBC_PATH))
        reader._db = db

        can_msg = _make_can_message(arbitration_id=msg_def.frame_id, data=bytes(raw_data))
        task = _setup_consumer(reader, [can_msg])
        try:
            speed = await reader.read_speed()
        finally:
            task.cancel()

        assert abs(speed - 100.0) < 0.01
