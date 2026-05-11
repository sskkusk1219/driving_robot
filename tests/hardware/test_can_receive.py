"""CAN 受信ハードウェアテスト。

Kvaser USB-CAN を実機接続した状態でのみ実行できる。

実行方法:
    .venv/bin/pytest tests/hardware/test_can_receive.py -v -m hardware
"""

from __future__ import annotations

import sys
from collections.abc import AsyncGenerator
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.infra.can_reader import CANReader
from src.infra.settings import CanSettings, load_settings

pytestmark = pytest.mark.hardware

_PROJECT_ROOT = Path(__file__).parent.parent.parent


def _can_settings() -> CanSettings:
    try:
        return load_settings(_PROJECT_ROOT / "config" / "settings.toml").can
    except FileNotFoundError:
        return CanSettings()


@pytest.fixture
async def reader() -> AsyncGenerator[CANReader]:
    can = _can_settings()
    r = CANReader(
        interface=can.interface,
        channel=can.channel,
        dbc_path=str(_PROJECT_ROOT / can.dbc_path),
    )
    await r.connect()
    yield r
    await r.close()


@pytest.mark.asyncio
async def test_connect(reader: CANReader) -> None:
    """Kvaser に接続できること。"""
    assert reader is not None


@pytest.mark.asyncio
async def test_read_speed_returns_float(reader: CANReader) -> None:
    """read_speed() が float [km/h] を返すこと。"""
    speed = await reader.read_speed()
    assert isinstance(speed, float)
    assert speed >= 0.0
