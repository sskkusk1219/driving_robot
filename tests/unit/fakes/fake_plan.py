# ruff: noqa: E501
"""準備（prepare）のテスト用の偽の計画プロセス。``--plan-only`` と同じ形のイベントを出し、計画ファイルを書く。

    python fake_plan.py OUT MODE BEHAVIOR [LOG]

BEHAVIOR: ok（すぐ書く）/ slow（0.6 秒後に書く）/ hang（SIGINT まで居座り、書かず 130）/ fail（終了コード 2・理由つき）。
LOG があれば ``MODE`` を 1 行追記する（何本・どの順に起動されたか）。
"""

import json
import signal
import sys
import time
from pathlib import Path


def emit(type_, **fields):
    print("@@EV " + json.dumps({"type": type_, "ts": time.time(), **fields}, ensure_ascii=False), flush=True)


def main():
    out, mode, behavior = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
    if len(sys.argv) > 4:
        with Path(sys.argv[4]).open("a") as f:
            f.write(mode + "\n")
    emit("state", phase="planning", mode=mode)
    if behavior == "hang":
        signal.signal(signal.SIGINT, lambda *_: sys.exit(130))
        time.sleep(60)
    if behavior == "slow":
        time.sleep(0.6)
    if behavior == "fail":
        print("設定エラー: 走行モードが見つかりません", flush=True)
        emit("result", exit_code=2, meaning="設定エラー", error=f"モード {mode} がありません", elapsed_s=0.1)
        sys.exit(2)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(b"plan:" + mode.encode())
    emit("state", phase="planned", mode=mode, path=str(out))
    emit("result", exit_code=0, meaning="成功", error=None, elapsed_s=0.1)


main()
