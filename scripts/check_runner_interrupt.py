#!/usr/bin/env python3
# ruff: noqa: E501
"""ランナーの中断の確認（スタブ）: 走行中に SIGINT（任意で 2 回目も）を送り、終了処理が走って終わるまでの秒数を測る。

    .venv/bin/python scripts/check_runner_interrupt.py [--signal INT|TERM] [--second] [--after-s 5] [--steps 1,24]

--when の印（既定: モード走行の行が流れ始めた = sample イベントの section=MODE_DRIVE）から --after-s 秒後にシグナルを送る。
終了までの秒数・終了コード・「終了処理: 完了」の行・result イベントの有無を表示する（実機は動かさない。常に --hw stub）。
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
MARKERS = {
    "mode": '"section":"MODE_DRIVE"',
    "prepare": '"phase":"prepare"',
    "pre_check": '"phase":"pre_check"',
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--signal", choices=("INT", "TERM"), default="INT")
    ap.add_argument("--second", action="store_true", help="0.05 秒後に 2 回目を送る（無視されること）")
    ap.add_argument(
        "--when",
        choices=tuple(MARKERS),
        default="mode",
        help="どこで送るか: mode=モード走行の行が流れ始めてから / prepare=手順の準備（モード・FF 計画の読み込み）が始まってから / pre_check=走行前チェックが始まってから",
    )
    ap.add_argument("--after-s", type=float, default=5.0)
    ap.add_argument("--steps", default="1,24")
    ap.add_argument("--vehicle", default="test_vehicle")
    ap.add_argument("--vehicles-dir", default=None, help="車両フォルダの親（既定 vehicles/）")
    ap.add_argument("--timeout-s", type=float, default=600.0)
    args = ap.parse_args()
    sig = signal.SIGINT if args.signal == "INT" else signal.SIGTERM

    cmd = [sys.executable, "-m", "src.robot.run", "--vehicle", args.vehicle, "--steps", args.steps, "--hw", "stub"]
    if args.vehicles_dir:
        cmd += ["--vehicles-dir", args.vehicles_dir]
    proc = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
    assert proc.stdout is not None
    t_marker: float | None = None
    sent: list[float] = []
    lines: list[str] = []

    def fire() -> None:
        sent.append(time.monotonic())
        proc.send_signal(sig)
        if args.second:
            time.sleep(0.05)
            proc.send_signal(sig)

    deadline = time.monotonic() + args.timeout_s
    for line in proc.stdout:
        lines.append(line.rstrip("\n"))
        now = time.monotonic()
        if t_marker is None and line.startswith("@@EV ") and MARKERS[args.when] in line:
            t_marker = now
            print(f"{args.when} の印が出た（{args.after_s:g} 秒後に SIG{args.signal} を送る）")
            # 出力が止まっている間（重い準備など）にも送れるよう、タイマーのスレッドから送る
            threading.Timer(args.after_s, fire).start()
        if now > deadline:
            proc.kill()
            print("タイムアウト")
            return 2
    rc = proc.wait()
    t_end = time.monotonic()
    t_sent = sent[0] if sent else None
    if t_sent is None:
        print("シグナルを送る前に終わった（印が出なかった）")
        return 2
    done_line = any("終了処理: 完了" in ln for ln in lines)
    result = None
    for ln in lines:
        if ln.startswith('@@EV {"type":"result"'):
            result = json.loads(ln[5:])
    print(f"終了コード: {rc}  /  シグナルから終了まで: {t_end - t_sent:.2f} 秒")
    print(f"「終了処理: 完了」の行: {'あり' if done_line else 'なし'}")
    print(f"result イベント: {result}")
    for ln in lines:
        if any(k in ln for k in ("を受けました", "無視", "中断", "終了処理", "走行ログ", "警告")) and not ln.startswith("@@EV"):
            print("  " + ln[:200])
    expected = 130 if args.signal == "INT" else 143
    return 0 if (rc == expected and done_line and result is not None) else 1


if __name__ == "__main__":
    sys.exit(main())
