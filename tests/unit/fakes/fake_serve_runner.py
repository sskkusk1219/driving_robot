# ruff: noqa: E501
"""JobManager のテスト用の偽・常駐ランナー（``--serve`` と同じ標準入力のコマンド・標準出力のイベントの形）。

    python fake_serve_runner.py LOGFILE CODES

- 起動するたびに LOGFILE へ ``START`` を 1 行、受け取ったコマンド（stop・quit_interactive・jog など制御以外のジョブのコマンド）を
  JSON 1 行で追記する（プロセスの数・コマンドの順と home_at_end などをテストが見る）
- 起動時に初期化（``result`` ``job_id="init"``）→ ``state idle``。環境変数 FAKE_INIT_EXIT が 0 以外なら
  ``runner_exit: true`` の result で終わる。FAKE_WATCH_AFTER=秒 で、待機の間にその秒数後に watch error を出して終了コード 5 で終わる
- CODES は「ジョブごと（init・interactive を除く）の動き」のカンマ区切り（足りない回は 0）:
    数字 … 0.05 s 後にその終了コードで終わる / ``h`` … stop・SIGINT が来るまで sample を出し続ける
    ``n`` … 終了コード 5 で終わり ``needs_init`` を付ける / ``e`` … 自動の非常停止（phase=emergency stage=start）の後 5 で終わる
    ``p`` … ``h`` と同じだが SIGUSR1/2 を受けて行を出す / ``k`` … stop も SIGINT も無視して居座る（SIGKILL のテスト）
    ``w`` … stop を無視して 0.5 s 走り 0 で終わる / ``c`` … 0.05 s 後に 0 で終わる（FAKE_CSV の CSV を csv イベントで知らせる）
- ジョブが ``h``/``p`` のとき stop が来たら、本物と同じく ``state phase=emergency stage=start`` を出してから
  result（130・meaning「中断（stop コマンド）…」）。SIGINT のときは result（130・runner_exit）を出して終了する
"""

import json
import os
import select
import signal
import sys
import time
from pathlib import Path


def emit(type_, **fields):
    print("@@EV " + json.dumps({"type": type_, "ts": time.time(), **fields}, ensure_ascii=False), flush=True)


LOG = Path(sys.argv[1])
CODES = sys.argv[2].split(",") if len(sys.argv) > 2 and sys.argv[2] else []
sigint = []
job = None  # 実行中のジョブ: {"cmd","job_id","code","t0","kind"}
job_count = 0
needs_init = False
idle_since = time.time()
watch_after = float(os.environ.get("FAKE_WATCH_AFTER", "0") or 0)


def log(text):
    with LOG.open("a") as f:
        f.write(text + "\n")


def idle():
    emit("state", phase="idle", needs_init=needs_init)


def result(job_id, code, *, meaning="x", needs=False, runner_exit=False, error=None):
    emit("state", phase="done", exit_code=code, job_id=job_id)
    emit("result", job_id=job_id, exit_code=code, meaning=meaning, error=error, elapsed_s=0.1, needs_init=needs, runner_exit=runner_exit)


def do_init(job_id):
    emit("state", phase="init", step=1, title="初期化", job_id=job_id)
    for name in ("通信確認", "原点復帰"):
        print(f"  [OK  ] {name}", flush=True)
        emit("init_step", name=name, status="ok", detail="", job_id=job_id)
    code = int(os.environ.get("FAKE_INIT_EXIT", "0"))
    if code != 0:
        result(job_id, code, runner_exit=True, error="初期化エラー")
        sys.exit(code)
    result(job_id, 0)
    global needs_init
    needs_init = False
    idle()


def start_job(cmd):
    global job, job_count
    log(json.dumps(cmd, ensure_ascii=False, sort_keys=True))
    kind = cmd["cmd"]
    jid = cmd["job_id"]
    if needs_init:
        emit("ack", id=None, cmd=kind, ok=False, message="初期化が必要です（init コマンド）", job_id=jid)
        return
    emit("ack", id=None, cmd=kind, ok=True, message="ジョブを開始します", job_id=jid)
    if kind == "interactive":
        job = {"cmd": cmd, "job_id": jid, "kind": "interactive", "t0": time.time()}
        emit("state", phase="interactive", purpose=cmd.get("purpose"), job_id=jid)
        emit("status", id=None, accel_pct=5.0, brake_pct=5.0, accel_pulse=475, brake_pulse=475, speed_kmh=0.0,
             min_accel_pct=5.0, min_brake_pct=5.0, max_accel_pct=80.0, max_brake_pct=80.0, job_id=jid)
        return
    code = CODES[job_count] if job_count < len(CODES) else "0"
    job_count += 1
    job = {"cmd": cmd, "job_id": jid, "kind": kind, "code": code, "t0": time.time()}
    emit("state", phase="step", step=24 if kind == "mode" else 3, title="走行", job_id=jid)
    if cmd.get("home_at_end", True):
        print("[   1.0s] ジョブの終わり: 原点復帰だけ行います（サーボON・接続のまま待機します）", flush=True)
    else:
        print("[   1.0s] ジョブの終わり: 停車保持のまま待機します（原点復帰・サーボOFF・切断はしません）", flush=True)
    if code == "p":  # ボタン押下のイベント（スタートを 1 秒）
        emit("button", button="start", phase="start", seconds=1.0, job_id=jid)
        emit("sample", t_s=0.5, mode_time_s=0.5, ref_kmh=0.0, speed_kmh=0.0, section="MODE_DRIVE", pattern="Low", phase="", job_id=jid)
        emit("button", button="start", phase="end", seconds=1.0, job_id=jid)
    if code == "e":
        time.sleep(0.2)
        emit("state", phase="emergency", stage="start", detail="DriveError: CAN 車速を読めません", job_id=jid)


def end_job(code, **kw):
    global job
    jid = job["job_id"]
    emit("state", phase="job_end", mode="home" if job["cmd"].get("home_at_end", True) else "keep", job_id=jid)
    result(jid, code, **kw)
    job = None
    idle()


def tick():
    global job, needs_init
    if job is None:
        return
    kind, code = job["kind"], job.get("code", "0")
    elapsed = time.time() - job["t0"]
    if kind == "interactive":
        return
    if code in ("h", "p", "k"):
        emit("sample", t_s=elapsed, mode_time_s=elapsed, ref_kmh=10.0, speed_kmh=10.5, accel_cmd_pct=1.0, brake_cmd_pct=0.0,
             accel_actual_pct=1.0, brake_actual_pct=0.0, section="MODE_DRIVE", pattern="Low", phase="", job_id=job["job_id"])
        return
    if code == "w":  # stop を無視して 0.5 s 走り、正常（0）で終わる
        if elapsed >= 0.5:
            end_job(0)
        return
    if elapsed < 0.05:
        return
    if code == "e":
        if elapsed > 0.5:
            end_job(5, meaning="走行エラー", error="DriveError")
        return
    if code == "n":
        needs_init = True
        end_job(5, needs=True, error="boom")
        return
    if kind == "mode" and code == "c" and os.environ.get("FAKE_CSV"):
        emit("csv", path=os.environ["FAKE_CSV"].replace("{n}", str(job_count)), kind="drive", rows=3, job_id=job["job_id"])
    c = int(code) if code.isdigit() else 0
    end_job(c, error="boom" if c else None)


def handle_line(line):
    global job, needs_init
    if not line.strip():
        return
    cmd = json.loads(line)
    kind = cmd["cmd"]
    if kind == "init":
        log(json.dumps(cmd, ensure_ascii=False, sort_keys=True))
        emit("ack", id=None, cmd="init", ok=True, message="ジョブを開始します", job_id=cmd["job_id"])
        do_init(cmd["job_id"])
        return
    if kind in ("mode", "schedule", "steps", "interactive"):
        if job is not None:
            emit("ack", id=None, cmd=kind, ok=False, message=f"実行中です（job_id={job['job_id']}）", job_id=cmd["job_id"])
            return
        start_job(cmd)
        return
    if kind == "stop":
        log('{"cmd": "stop"}')
        emit("ack", id=None, cmd="stop", ok=True, message="x")
        if job is not None and job.get("code") not in ("k", "w") and job["kind"] != "interactive":
            emit("state", phase="emergency", stage="start", detail="SIGINT による中断", job_id=job["job_id"])
            end_job(130, meaning="中断（stop コマンド）。終了処理は済ませた")
        return
    if kind == "quit_interactive":
        log('{"cmd": "quit_interactive"}')
        if job is not None and job["kind"] == "interactive":
            emit("ack", id=cmd.get("id"), cmd=kind, ok=True, message="x")
            end_job(0, meaning="成功")
        else:
            emit("ack", id=cmd.get("id"), cmd=kind, ok=False, message="対話ジョブではありません")
        return
    if kind == "quit":
        log('{"cmd": "quit"}')
        jid = cmd.get("job_id") or (job["job_id"] if job else None)
        emit("ack", id=None, cmd="quit", ok=True, message="x", **({"job_id": jid} if jid else {}))
        if job is not None and job["kind"] != "interactive":
            result(job["job_id"], 130, runner_exit=True, meaning="中断（quit コマンド）")  # 走っているジョブは止めてから終える
        elif jid:
            result(jid, 0, runner_exit=True)
        sys.exit(0)
    # 対話の操作
    if job is not None and job["kind"] == "interactive":
        log(json.dumps(cmd, ensure_ascii=False, sort_keys=True))
        ok = kind != "bad"
        emit("ack", id=cmd.get("id"), cmd=kind, ok=ok, message="OK" if ok else "不正です", job_id=job["job_id"])
        if ok:
            emit("status", id=cmd.get("id"), accel_pct=7.0, brake_pct=5.0, accel_pulse=665, brake_pulse=475, speed_kmh=0.0,
                 min_accel_pct=5.0, min_brake_pct=5.0, max_accel_pct=80.0, max_brake_pct=80.0, job_id=job["job_id"])
    else:
        emit("ack", id=cmd.get("id"), cmd=kind, ok=False, message="対話ジョブの間だけ使えます")


def on_sigint(*_):
    sigint.append(1)


def main():
    log("START")
    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGUSR1, lambda *_: print("GOT-USR1", flush=True))
    signal.signal(signal.SIGUSR2, lambda *_: print("GOT-USR2", flush=True))
    do_init("init")
    global idle_since
    idle_since = time.time()
    while True:
        if sigint:
            if job is not None and job.get("code") == "k":
                sigint.clear()  # 居座る
            else:
                if job is not None:
                    result(job["job_id"], 130, runner_exit=True, meaning="中断（SIGINT）")
                else:
                    emit("result", job_id=None, exit_code=130, meaning="中断（SIGINT）", error=None, elapsed_s=0.1, needs_init=False, runner_exit=True)
                print("終了処理: 原点復帰 → サーボOFF → 切断", flush=True)
                sys.exit(130)
        r, _, _ = select.select([sys.stdin], [], [], 0.05)
        if r:
            line = sys.stdin.readline()
            if not line:
                sys.exit(0)
            handle_line(line)
        tick()
        if watch_after and job is None and time.time() - idle_since > watch_after:
            emit("watch", level="error", detail="ブレーキ軸にアラーム")
            emit("result", job_id=None, exit_code=5, meaning="走行エラー", error="待機中の見張りで異常", elapsed_s=0.1, needs_init=False, runner_exit=True)
            sys.exit(5)


main()
