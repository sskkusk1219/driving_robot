"""CAN 診断スクリプト。

バス統計・エラーフレーム・受信フレームをすべて表示し、
Pico の Bus-Off 問題の原因調査に使う。
インターフェース:kvaser

使い方:
    sudo .venv/bin/python scripts/check_can_2.py
"""

import sys
import time
from datetime import datetime
from pathlib import Path

_PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_PROJECT_ROOT))

import can  # noqa: E402
from can.interfaces.kvaser.canlib import DRIVER_MODE_NORMAL  # noqa: E402

CHANNEL = 0
BITRATE = 500000
STATS_INTERVAL_S = 5.0


def _now() -> str:
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def _print_stats(bus: can.Bus) -> None:
    try:
        stats = bus.get_stats()
        print(
            f"  [STATS] std={stats.m_stdData} ext={stats.m_extData} "
            f"err={stats.m_errFrame} overrun={stats.m_overruns} "
            f"busLoad={stats.m_busLoad / 100:.1f}%"
        )
    except Exception as e:
        print(f"  [STATS] 取得失敗: {e}")


def main() -> None:
    print(f"CAN 診断開始 channel={CHANNEL} bitrate={BITRATE}")
    print(f"driver_mode=NORMAL ({DRIVER_MODE_NORMAL}) → Kvaser が ACK を送出します")
    print("Ctrl+C で終了\n")

    bus = can.Bus(
        interface="kvaser",
        channel=CHANNEL,
        bitrate=BITRATE,
        # True = NORMAL モード: Kvaser が ACK を送出する（必須）
        # False = SILENT モード: ACK を送出しない → Pico が Bus-Off になる
        driver_mode=DRIVER_MODE_NORMAL,
        single_handle=True,
    )
    print(f"接続完了。bus.state={bus.state}\n")

    last_stats_time = time.monotonic()
    frame_count = 0
    error_count = 0

    try:
        while True:
            msg = bus.recv(timeout=1.0)
            now = time.monotonic()

            if msg is None:
                print(f"[{_now()}] TIMEOUT (state={bus.state})")
            elif msg.is_error_frame:
                error_count += 1
                print(
                    f"[{_now()}] ERROR FRAME #{error_count} "
                    f"ID=0x{msg.arbitration_id:03X} state={bus.state}"
                )
            else:
                frame_count += 1
                data_hex = " ".join(f"{b:02X}" for b in msg.data)
                print(
                    f"[{_now()}] FRAME #{frame_count} "
                    f"ID=0x{msg.arbitration_id:03X} "
                    f"DLC={msg.dlc} DATA={data_hex} "
                    f"state={bus.state}"
                )

            if now - last_stats_time >= STATS_INTERVAL_S:
                _print_stats(bus)
                last_stats_time = now

    except KeyboardInterrupt:
        print("\n終了")
        _print_stats(bus)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
