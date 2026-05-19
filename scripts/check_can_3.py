"""
CAN 診断スクリプト。

MCP2515 + socketcan 用。

バス統計・エラーフレーム・受信フレームを表示し、
Bus-Off / Error Passive / Overrun の調査に使う。

使い方:
    sudo .venv/bin/python scripts/check_can_3.py
"""

from __future__ import annotations

import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

_PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_PROJECT_ROOT))

import can  # noqa: E402


CHANNEL = "can0"
BITRATE = 500000
STATS_INTERVAL_S = 5.0


def _now() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def _get_can_state() -> str:
    """
    Linux socketcan の状態取得
    """

    try:
        result = subprocess.run(
            ["ip", "-details", "link", "show", CHANNEL],
            capture_output=True,
            text=True,
            check=False,
        )

        text = result.stdout

        if "BUS-OFF" in text:
            return "BUS-OFF"

        if "ERROR-PASSIVE" in text:
            return "ERROR-PASSIVE"

        if "ERROR-WARNING" in text:
            return "ERROR-WARNING"

        if "STOPPED" in text:
            return "STOPPED"

        return "ERROR-ACTIVE"

    except Exception as e:
        return f"UNKNOWN({e})"


def _print_stats() -> None:
    """
    socketcan 統計情報表示
    """

    try:
        result = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", CHANNEL],
            capture_output=True,
            text=True,
            check=False,
        )

        print("\n========== CAN STATISTICS ==========")
        print(result.stdout.strip())
        print("====================================\n")

    except Exception as e:
        print(f"[STATS] 取得失敗: {e}")


def main() -> None:

    print(f"CAN 診断開始 channel={CHANNEL} bitrate={BITRATE}")
    print("interface=socketcan (MCP2515)")
    print("Ctrl+C で終了\n")

    try:
        bus = can.Bus(
            interface="socketcan",
            channel=CHANNEL,
            bitrate=BITRATE,
        )

    except Exception as e:
        print(f"CAN 初期化失敗: {e}")
        sys.exit(1)

    print(f"接続完了 state={_get_can_state()}\n")

    last_stats_time = time.monotonic()

    frame_count = 0
    error_count = 0

    try:

        while True:

            msg = bus.recv(timeout=1.0)

            now = time.monotonic()
            state = _get_can_state()

            if msg is None:

                print(f"[{_now()}] TIMEOUT state={state}")

            elif msg.is_error_frame:

                error_count += 1

                print(
                    f"[{_now()}] ERROR FRAME #{error_count} "
                    f"ID=0x{msg.arbitration_id:03X} "
                    f"state={state}"
                )

            else:

                frame_count += 1

                data_hex = " ".join(f"{b:02X}" for b in msg.data)

                print(
                    f"[{_now()}] FRAME #{frame_count} "
                    f"ID=0x{msg.arbitration_id:03X} "
                    f"DLC={msg.dlc} "
                    f"DATA={data_hex} "
                    f"state={state}"
                )

            if now - last_stats_time >= STATS_INTERVAL_S:

                _print_stats()

                last_stats_time = now

    except KeyboardInterrupt:

        print("\n終了")

        _print_stats()

    finally:

        bus.shutdown()


if __name__ == "__main__":
    main()