"""研究開発用ハーネスのターミナル出力。

「進捗はターミナルに都度 print し、現在何をやっているかユーザーが分かるようにすること」
（ProblemReport_20260910「コードの作成と実行手順」）を満たすための最小の共通出力。
main / hardware / 以降の走行モジュールが共有するため、循環 import を避けて別モジュールに置く。
"""

from __future__ import annotations

import time

_RUN_START = time.monotonic()

LINE_WIDTH = 68


def elapsed_s() -> float:
    """ハーネス起動からの経過秒。"""
    return time.monotonic() - _RUN_START


def say(message: str = "") -> None:
    """経過秒つきで 1 行出力し、即座にフラッシュする。"""
    if message == "":
        print(flush=True)
        return
    print(f"[{elapsed_s():7.1f}s] {message}", flush=True)


def banner(text: str) -> None:
    say("=" * LINE_WIDTH)
    say(text)
    say("=" * LINE_WIDTH)


def display_width(text: str) -> int:
    """全角文字を 2 桁として数え、ラベル幅を揃える。"""
    return sum(2 if ord(ch) > 0x2E80 else 1 for ch in text)


def drive_status_line(
    *,
    t_s: float,
    ref_kmh: float | None,
    actual_kmh: float,
    kp: float,
    ki: float,
    kd: float,
    accel_pct: float,
    brake_pct: float,
    extra: str = "",
) -> str:
    """走行中の 1 行表示（基準車速・実車速・偏差・Kp/Ki/Kd・開度・その他）。

    基準車速を持たない走行（手順 2 のパターン走行）は基準・偏差を「---」で表す。
    """
    ref = "   ---" if ref_kmh is None else f"{ref_kmh:6.2f}"
    dev = "   ---" if ref_kmh is None else f"{actual_kmh - ref_kmh:+6.2f}"
    return (
        f"t={t_s:6.1f}s 基準={ref} 実={actual_kmh:6.2f} 偏差={dev} km/h "
        f"Kp={kp:g} Ki={ki:g} Kd={kd:g} | アクセル={accel_pct:5.1f}% ブレーキ={brake_pct:5.1f}%"
        f"{f' | {extra}' if extra else ''}"
    )
