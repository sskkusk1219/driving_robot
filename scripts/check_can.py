"""CAN 受信確認スクリプト。

Kvaser USB-CAN に接続し、シャシダイナモの車速 [km/h] をターミナルに表示する。
Ctrl+C で終了。

使い方:
    .venv/bin/python scripts/check_can.py
"""

from __future__ import annotations

import asyncio
import sys
from datetime import datetime
from pathlib import Path

# プロジェクトルートを Python パスに追加（src パッケージを解決するため）
_PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_PROJECT_ROOT))

from src.infra.can_reader import CANReader  # noqa: E402
from src.infra.settings import CanSettings, load_settings  # noqa: E402


def _load_can_settings() -> CanSettings:
    settings_path = _PROJECT_ROOT / "config" / "settings.toml"
    try:
        return load_settings(settings_path).can
    except FileNotFoundError:
        print(f"[警告] {settings_path} が見つかりません。デフォルト設定を使用します。")
        return CanSettings()


def _now() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


async def main() -> None:
    can = _load_can_settings()
    reader = CANReader(
        interface=can.interface,
        channel=can.channel,
        bitrate=can.bitrate,
        dbc_path=str(_PROJECT_ROOT / can.dbc_path),
    )

    print(f"CAN 接続中... interface={can.interface} channel={can.channel} bitrate={can.bitrate}")
    print(f"DBC: {can.dbc_path}")
    print("Ctrl+C で終了\n")

    try:
        await reader.connect()
        print("接続完了。車速を受信します。\n")
    except Exception as e:
        print(f"[エラー] CAN 接続失敗: {e}")
        return

    try:
        while True:
            await asyncio.sleep(0.001)
            speed = await reader.read_speed()
            print(f"[{_now()}] Speed: {speed:6.2f} km/h")
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        await reader.close()
        print("\n接続を閉じました。")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
