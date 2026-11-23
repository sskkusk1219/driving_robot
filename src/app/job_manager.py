# ruff: noqa: E501
"""ジョブ管理: 常駐ランナー（``python -m src.robot.run --serve``）を 1 本起動し、ジョブはコマンドで送る。

RPD_Phase_20261115_PortResearch.md §3 段3、ProblemReport_20261120_本番自動運転 段3（案 A）。制御（50 ms ループ・
NN 推論）は Web サーバーとは別プロセスのランナーで動く。ジョブごとにランナーを起動すると、終わりの shutdown で
原点復帰・サーボ OFF・切断が走り、シーケンスの段の間に接続し直しが入っていた。そこで**初期化で常駐ランナーを 1 本
起動し、サーボ ON・接続のまま動かし続ける**。Web 側（ここ）は次だけを受け持つ。

- 初期化（``start_init_only``）: 常駐ランナーが無ければ ``--serve`` で起動（起動時の初期化 ``result``（``job_id: "init"``）が
  初期化ジョブの終わり）。生きていれば ``{"cmd":"init"}`` を送る。``initialized`` は「常駐ランナーが生きていて、初期化が済み、
  最後の idle/result に ``needs_init`` が無い」こと
- ジョブ（モード・スケジュール・学習・IGN-OFF・手動・キャリブ）: 常駐ランナーへ標準入力の JSON 1 行コマンドで送る。
  ジョブの始まり・終わりはイベントの ``job_id`` で区切り、``result`` が終わり（プロセスの終了ではない）
- 停止（``stop``）はコマンド ``stop``（ランナーは残る。非常停止の状態には入らない）。非常停止（``emergency``）とアプリ終了は
  SIGINT / ``quit``（ランナーは終了処理 → 終了。終わらなければ SIGKILL）
- 待機中の見張りの ``watch`` イベント（warn / error）をメッセージにし、error でランナーが終わったら非常停止の状態（きっかけ auto）にする
- ランナーの標準出力を 1 行ずつ読み、``@@EV `` のイベント（src/robot/events.py）は状態に反映、
  それ以外の行は「進行のメッセージ」（画面の「現在ステータス」用のログバッファ＋ジョブごとに DB）
- 状態: 待機（STANDBY）/ 実行中（RUNNING）/ 非常停止中（EMERGENCY。解除するまで次のジョブを受け付けない）
- セッション（drive_sessions）: 学習（run_type learning）・モード走行とスケジュール（auto）・手動運転（manual）で作る。
  **キャリブレーション（min 測定のジョグ）と初期化だけのジョブ（init_only）は作らない**（走行ではなく、
  ログに残す値がない。進行のメッセージは画面のバッファにだけ残る）
- 終了時の status: 終了コード 0 → completed、非常停止で止めた → emergency、利用者の走行終了（stop。130/143）→ stopped、
  それ以外 → error。
  対話ジョブ（手動・キャリブレーション）は利用者が閉じるのが通常の終わりなので 130/143 も completed
- CSV: ランナーが ``csv`` イベントで知らせた CSV を、ジョブの終わり（異常終了も）に drive_logs へ取り込む。
  種類 drive（モード走行）は取り込み後に消す。種類 learning（vehicles/<車ID>/data/。手順 13・14 が後で読み直す学習データ）は
  残す。DB が無い環境では取り込まず、CSV は消さない

- エラーのできごと（エラーページ。RPD_Phase_20261120_ErrorLog）: ジョブの終わり（_settle）・非常停止（_enter_emergency）・
  待機中の警告/エラーの行（_say_idle）・計画の準備の失敗・シーケンスの停止を、``ErrorRecorder``（src/app/error_recorder.py）へ
  渡して ``error_events`` テーブルへ残す。渡すだけで待たない・失敗しても制御に影響しない。ジョブの警告/エラーの行は
  ジョブの件の本文にまとめるので、ここのログ行には ``extra=RECORDED`` を付け、サーバーログのハンドラが二重に記録しない

ここは subprocess と asyncio だけを使い、DB は ``DriveRecorder``（src/infra/drive_recorder.py）越し。
"""

from __future__ import annotations

import asyncio
import bisect
import contextlib
import json
import logging
import math
import os
import shutil
import signal
import sys
import time
import traceback
import uuid
from collections import deque
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, replace
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

from src.app.error_recorder import ErrorRecorder, NullErrorRecorder
from src.infra.button_events import LEVEL_BUTTON, button_message
from src.infra.drive_recorder import DriveRecorder, MessageRow, NullDriveRecorder
from src.infra.error_log_handler import RECORDED
from src.models.error_event import (
    CONTEXT_MESSAGES_MAX,
    LEVEL_ERROR,
    LEVEL_WARNING,
    SOURCE_EMERGENCY,
    SOURCE_IDLE,
    SOURCE_JOB,
    SOURCE_PREPARE,
    SOURCE_SEQUENCE,
    NewErrorEvent,
)
from src.robot import events

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parents[2]

# ── 定数 ───────────────────────────────────────────────────────────────

#: SIGINT（通常の停止・非常停止）を送ってから SIGKILL するまでの猶予 [s]。
#: ランナーは SIGINT で手順を打ち切り、終了処理（両軸の原点復帰 → サーボOFF → 切断 → ログ保存）まで
#: 済ませてから終わる。スタブで 0.4 s（RPD 段2）。実機の原点復帰は core のアクチュエータの
#: タイムアウトが 1 軸 30 s（actuator_driver._HOME_RETURN_TIMEOUT_S）なので、それより短い猶予で
#: SIGKILL すると「原点復帰の途中でペダルを踏んだまま」残る。30 s ＋ サーボOFF・切断・CSV 保存の余裕 10 s。
#: 実機の本当の所要時間は段8 で測り、この値を見直す。
KILL_TIMEOUT_S = 120.0  # 走行するジョブは非常停止の流れ（減速して停車 → N → スタート 10 秒 → 原点復帰）に時間がかかる

#: 進行のメッセージを DB へ書くまとめの間隔 [s]（1 行ずつ INSERT しない）
MESSAGE_FLUSH_INTERVAL_S = 1.0

#: ランナーの標準出力 1 行の最大長（core の say は短いが、例外のトレースバックなどに備える）
STDOUT_LINE_LIMIT = 1 << 20

#: 標準出力が閉じてから残りを読み切るまでの待ち [s]
DRAIN_TIMEOUT_S = 5.0

#: 対話コマンドの応答待ち [s]（ジョグ 1 回は最大数秒。余裕を見る）
COMMAND_TIMEOUT_S = 120.0
#: ack の後に来る status の待ち [s]
STATUS_TIMEOUT_S = 10.0
#: 対話ジョブが初期化を終えてコマンド待ちになるまでの待ち [s]（原点復帰を含む）
INTERACTIVE_READY_TIMEOUT_S = 180.0
#: 常駐ランナーが起動時の初期化に使う job_id（src/robot/serve.py の STARTUP_JOB_ID と同じ）
STARTUP_JOB_ID = "init"

# 種類（kind）
KIND_LEARNING = "learning"
KIND_MODE_DRIVE = "mode_drive"
KIND_INTERACTIVE = "interactive"
KIND_INIT_ONLY = "init_only"
KIND_SCHEDULE = "schedule"  # タイムスケジュールの開ループ再生（段6）
KIND_IGN_OFF = "ign_off"  # IGN-OFF だけを押すジョブ（手順 1,25。D-15b）
KINDS = (KIND_LEARNING, KIND_MODE_DRIVE, KIND_INTERACTIVE, KIND_INIT_ONLY, KIND_SCHEDULE, KIND_IGN_OFF)

# 対話ジョブの目的（purpose）
PURPOSE_MANUAL = "manual"
PURPOSE_CALIB = "calib"

# マネージャの状態
#: 画面の読み直し用に覚えておく sample の数（sample は 5 Hz なので 120 s 分）
SAMPLE_HISTORY_MAX = 600

STATE_STANDBY = "standby"
STATE_RUNNING = "running"
STATE_EMERGENCY = "emergency"

#: 学習の種類 → 手順（RPD A-1: 手順 2 と部分学習 12〜14）。走行前チェック → 学習（Ready・D・学習・N はランナーが
#: 学習の手順に足す）→ IGN-OFF（25）の流れ。手順 1（初期化）は初期化ページだけで行う（D-15c、ユーザー決定 2026-11-16）。
#: 21（メンテナンス）・22（N ホールド）は外した（ユーザー決定 2026-11-15）
LEARNING_STEPS: dict[str, str] = {
    "full": "2,25",
    "fel": "12,25",
    "hold": "13,25",
    "stop": "14,25",
}

#: IGN-OFF だけを押すジョブ（走行・学習がエラー・中断で終わって車が Ready のまま残ったときの後始末）。
#: 手順 1 は含まない（ランナーが接続・サーボ ON だけ裏で行う）。初期化済みでなくても始められる
#: （エラー終了の後は初期化済みの印が消えるが、車を IGN-OFF に戻す手段が要るため）
IGN_OFF_STEPS = "25"

#: 初期化済みでないと始められない種類（モード走行・学習・スケジュール・IGN-OFF。シーケンスの段はモード走行・スケジュールのジョブ）。
#: 初期化は初期化ページだけ（ユーザー決定 2026-11-20）。手動運転・キャリブレーションだけは未初期化なら内部で初期化してから始める
_INIT_REQUIRED_KINDS = (KIND_LEARNING, KIND_MODE_DRIVE, KIND_SCHEDULE, KIND_IGN_OFF)

#: 初期化が要るのに済んでいないときの案内（画面にも同じ文を出す）
NOT_INITIALIZED_MESSAGE = (
    "初期化が済んでいません。初期化ページで「初期化」を実行してから始めてください"
    "（サーバーの起動後・非常停止の後・エラー終了の後は、もう一度初期化が要ります）"
)

#: この手順が始まったら、ボタン操作で車の状態を変えた（IGN-ON 以降。終わるまで IGN-OFF が要る）
# 21・22 は通しの流れから外れたが、ランナーで単独に走らせたときのため残す
_BUTTON_STATE_STEPS = (21, 22, 24)

#: csv イベントの kind のうち、取り込み後に消す方（モード走行。もう一方の learning は残す）
CSV_KIND_DRIVE = "drive"

#: sample の section のうち、運転指標・偏差の KPI の対象（core の SECTION_MODE_DRIVE と同じ文字列）
SECTION_MODE_DRIVE = "MODE_DRIVE"

#: 画面へ渡すボタン押下イベントの最大数（古いものから捨てる）
BUTTON_EVENTS_MAX = 100

#: エラーページのジョブの呼び名
_KIND_LABEL = {
    KIND_LEARNING: "学習運転",
    KIND_MODE_DRIVE: "自動運転（モード走行）",
    KIND_INTERACTIVE: "対話ジョブ",
    KIND_INIT_ONLY: "初期化",
    KIND_SCHEDULE: "自動運転（スケジュール）",
    KIND_IGN_OFF: "IGN-OFF",
}
_PURPOSE_LABEL = {PURPOSE_MANUAL: "手動運転", PURPOSE_CALIB: "キャリブレーション"}

#: 非常停止のきっかけの呼び名
_CAUSE_LABEL = {
    "screen": "画面の非常停止ボタン",
    "switch": "非常停止スイッチ",
    "ac_loss": "AC 電源断",
    "auto": "自動（ランナーが検出）",
}

#: ジョブの件の本文に並べる警告・エラーの行の最大数
_ISSUE_LINES_MAX = 30

#: 対話ジョブが正常に閉じられたときの終了コード（0 または SIGINT/SIGTERM での中断）
_INTERACTIVE_OK_CODES = (0, 130, 143)


class JobError(Exception):
    """ジョブを始められない・操作できない（日本語の案内つき）。``status_code`` は API の応答用。"""

    status_code = 400


class JobConflict(JobError):  # noqa: N818
    status_code = 409


class EmergencyStillActive(JobConflict):  # noqa: N818
    """物理の非常停止スイッチがまだ押されている。"""

    status_code = 423


class JobUnavailable(JobError):
    status_code = 503


# ── 純粋な部分 ─────────────────────────────────────────────────────────


@dataclass(frozen=True)
class JobSpec:
    kind: str
    vehicle_id: str
    steps: str | None = None  # interactive 以外のとき手順（"24,25" など）
    mode_id: str | None = None
    mode_name: str | None = None
    limit_s: float | None = None
    purpose: str | None = None  # interactive: manual / calib
    schedule_id: str | None = None  # schedule: タイムスケジュールの ID・名前・ランナーへ渡す内容（schedule_to_dict の形）
    schedule_name: str | None = None
    schedule: dict[str, Any] | None = None
    plan_file: str | None = None  # 手順 24 の計画ファイル（コマンドの plan_in。準備 prepare で作ったもの）
    no_pre_check: bool = False  # 走行前チェックを飛ばす（コマンドの no_pre_check。シーケンスの 2 段目以降）
    home_at_end: bool = True  # ジョブの終わりに原点復帰するか（シーケンスの最後の段だけ true。段の間は停車保持のまま）

    @property
    def run_type(self) -> str | None:
        """drive_sessions.run_type。セッションを作らないジョブは None。"""
        if self.kind == KIND_LEARNING:
            return "learning"
        if self.kind in (KIND_MODE_DRIVE, KIND_SCHEDULE):
            return "auto"
        if self.kind == KIND_INTERACTIVE and self.purpose == PURPOSE_MANUAL:
            return "manual"
        return None


#: モード走行の手順。src では常に 24,25（走行前チェック → Ready・D・モード・N → IGN-OFF）。
#: 手順 1（初期化）は初期化ページだけで行う（D-15c、ユーザー決定 2026-11-16）。
#: 21（メンテナンス）・22（N ホールド）は外した（ユーザー決定 2026-11-15）。手順 3 は使わない（D-15 / D-15b）
MODE_DRIVE_STEPS = "24,25"


def hw_mode_from_env(env: dict[str, str] | None = None) -> str:
    """ランナーの --hw。DRIVING_ROBOT_USE_REAL_HW=1 なら real、それ以外は stub。

    ベンチ（DRIVING_ROBOT_BENCH_GPIO_ONLY=1）は非常停止スイッチ（GPIO）だけ実機でアクチュエータは
    スタブなので、real にしない。
    """
    e = os.environ if env is None else env
    if e.get("DRIVING_ROBOT_USE_REAL_HW") == "1" and e.get("DRIVING_ROBOT_BENCH_GPIO_ONLY") != "1":
        return "real"
    return "stub"


def build_serve_command(
    vehicle_id: str,
    *,
    vehicles_dir: Path,
    common_path: Path,
    hw: str,
    python: str | None = None,
) -> list[str]:
    """常駐ランナー（--serve）の起動コマンド。ジョブはここへ標準入力のコマンドで送る（job_command）。"""
    return [
        python or sys.executable,
        "-m",
        "src.robot.run",
        "--vehicle",
        vehicle_id,
        "--serve",
        "--hw",
        hw,
        "--vehicles-dir",
        str(vehicles_dir),
        "--common",
        str(common_path),
    ]


def job_command(spec: JobSpec, job_id: str) -> dict[str, Any]:
    """ジョブを常駐ランナーへ送る JSON コマンド（src/robot/README.md の「コマンド」）。"""
    if spec.kind == KIND_INIT_ONLY:
        return {"cmd": "init", "job_id": job_id, "vehicle": spec.vehicle_id}
    if spec.kind == KIND_INTERACTIVE:
        return {"cmd": "interactive", "job_id": job_id, "vehicle": spec.vehicle_id, "purpose": spec.purpose or PURPOSE_MANUAL}
    if spec.kind == KIND_MODE_DRIVE:
        return {
            "cmd": "mode", "job_id": job_id, "vehicle": spec.vehicle_id, "mode": spec.mode_name, "plan_in": spec.plan_file,
            "limit_s": spec.limit_s, "no_pre_check": spec.no_pre_check, "home_at_end": spec.home_at_end,
        }
    if spec.kind == KIND_SCHEDULE:
        return {
            "cmd": "schedule", "job_id": job_id, "vehicle": spec.vehicle_id, "schedule": spec.schedule,
            "no_pre_check": spec.no_pre_check, "home_at_end": spec.home_at_end,
        }
    return {"cmd": "steps", "job_id": job_id, "vehicle": spec.vehicle_id, "steps": spec.steps, "home_at_end": spec.home_at_end}


def build_plan_command(
    vehicle_id: str,
    mode_name: str,
    out: Path,
    *,
    vehicles_dir: Path,
    common_path: Path,
    python: str | None = None,
) -> list[str]:
    """計画だけを作るプロセス（--plan-only。ハードに触れない）の起動コマンド。"""
    return [
        python or sys.executable,
        "-m",
        "src.robot.run",
        "--vehicle",
        vehicle_id,
        "--plan-only",
        "--mode",
        mode_name,
        "--plan-out",
        str(out),
        "--vehicles-dir",
        str(vehicles_dir),
        "--common",
        str(common_path),
    ]


@dataclass
class JobStatus:
    """走っている（または直前に終わった）ジョブの状態。イベントを受けて更新する。"""

    job_id: str
    spec: JobSpec
    started_at: datetime
    t0: float  # monotonic
    session_id: str | None = None
    pid: int | None = None
    phase: str = "starting"  # runner の state.phase（init / prepare / pre_check / step / shutdown / done / interactive）
    step: int | None = None
    step_title: str | None = None
    init_steps: list[dict[str, str]] = field(default_factory=list)
    sample: dict[str, Any] | None = None
    sample_at: float | None = None  # monotonic
    # 押下の時刻の物差し。時刻つきの最後の sample の（時刻, 到着 monotonic）。mode_time_s を一度でも受けたら
    # モードの時刻、無ければ t_s。時刻の欄が無い sample（STOPPING など）の間は、ここからの経過を足す
    time_ref: tuple[float, float] | None = None
    mode_time_seen: bool = False
    interactive_status: dict[str, Any] | None = None
    csv_files: list[dict[str, Any]] = field(default_factory=list)
    result: dict[str, Any] | None = None
    exit_code: int | None = None
    ended_at: datetime | None = None
    final_status: str | None = None  # completed / stopped（利用者の走行終了）/ error / emergency
    stop_requested: bool = False
    emergency: bool = False
    sequence: SequenceRun | None = None  # シーケンスの 1 段として走っているとき、その全体の進み具合
    button_state_changed: bool = False  # 手順 21・22・24 を始めた（車を IGN-ON 以降にした）
    ign_off_done: bool = False  # 手順 25（IGN-OFF）が正常に終わった（非常停止の流れでスタートを押し終えたときも）
    emergency_stage: str | None = None  # 非常停止の流れの今の段（state phase=emergency の stage）
    emergency_detail: str | None = None
    brake_kept: bool = False  # 止まらず、ブレーキを最大開度で踏んだまま終えた（サーボ ON のまま）
    buttons_unavailable: bool = False  # 停車しているがボタンを押せず、ブレーキを停車保持の開度で止めたまま終えた（サーボ ON のまま）
    paused: bool = False  # 一時停止中（SIGUSR1 を送ってから SIGUSR2 を送るまで。段11）
    # ボタン（スタート・P・N・D）の押下。{button, seconds, start_s, end_s|None}。start_s/end_s は
    # 押した時点の最新 sample の mode_time_s（無ければ t_s・sample 前は 0）。画面が同じ時間軸に矩形で描く
    button_events: list[dict[str, Any]] = field(default_factory=list)
    # モード走行の |偏差|（実車速 − 基準車速）の昇順の並び（一時停止中の行は含めない）。最大・p95 の元
    dev_abs: list[float] = field(default_factory=list)
    # 画面を読み直したときに過去の軌跡を描き直すための、最近の sample（monotonic 秒, sample）。
    # 自動運転の図は過去 15 s を描くので 120 s 分あれば足りる（2026-11-16 ユーザー指示: 読み直しで軌跡が消える）
    sample_history: deque[tuple[float, dict[str, Any]]] = field(
        default_factory=lambda: deque(maxlen=SAMPLE_HISTORY_MAX)
    )

    @property
    def needs_ign_off(self) -> bool:
        """終わったジョブが、車を IGN-ON 以降（Ready・N）のまま残したか（手順 25 まで行かなかった）。

        中断・非常停止・エラーでは core の run_steps が後の手順（25）へ進まないため、車は Ready のまま残る。
        画面はこのとき「IGN-OFF にしてください」と出し、``1,25`` のジョブを実行できるボタンを付ける。
        """
        if self.spec.kind not in (KIND_LEARNING, KIND_MODE_DRIVE):
            return False
        return self.ended_at is not None and self.button_state_changed and not self.ign_off_done

    @property
    def interactive_ready(self) -> bool:
        return self.phase == "interactive"

    def elapsed_s(self) -> float:
        return round(time.monotonic() - self.t0, 1)

    def sample_time_s(self) -> float:
        """最新 sample の時刻（mode_time_s があればそれ、無ければ t_s。sample 前は 0）。"""
        sm = self.sample
        if sm is None:
            return 0.0
        t = sm.get("mode_time_s")
        if t is None:
            t = sm.get("t_s")
        return float(t) if t is not None else 0.0

    def time_at(self, now: float) -> float:
        """画面の横軸と同じ物差しの「今」の時刻（最後の時刻つき sample の時刻 ＋ 到着からの経過）。

        一時停止中は時刻が止まるので経過を足さない。時刻つきの sample が一つも無ければ 0。
        """
        if self.time_ref is None:
            return 0.0
        t, at = self.time_ref
        if self.paused:
            return t
        return t + max(0.0, now - at)

    def dev_stats(self) -> dict[str, Any] | None:
        """|偏差| の最大・p95（モード走行の行だけ。まだ無ければ None）。"""
        v = self.dev_abs
        if not v:
            return None
        idx = min(len(v) - 1, max(0, math.ceil(0.95 * len(v)) - 1))
        return {"max": round(v[-1], 3), "p95": round(v[idx], 3), "n": len(v)}

    def to_dict(self) -> dict[str, Any]:
        s = self.spec
        out: dict[str, Any] = {
            "job_id": self.job_id,
            "kind": s.kind,
            "purpose": s.purpose,
            "vehicle_id": s.vehicle_id,
            "steps": s.steps,
            "mode_id": s.mode_id,
            "mode_name": s.mode_name,
            "schedule_id": s.schedule_id,
            "schedule_name": s.schedule_name,
            "limit_s": s.limit_s,
            "run_type": s.run_type,
            "session_id": self.session_id,
            "pid": self.pid,
            "phase": self.phase,
            "step": self.step,
            "step_title": self.step_title,
            "init_steps": list(self.init_steps),
            "sample": self.sample,
            "interactive_status": self.interactive_status,
            "started_at": self.started_at.isoformat(),
            "elapsed_s": self.elapsed_s(),
            "csv_files": list(self.csv_files),
            "exit_code": self.exit_code,
            "final_status": self.final_status,
            "stop_requested": self.stop_requested,
            "emergency": self.emergency,
            "emergency_stage": self.emergency_stage,
            "emergency_detail": self.emergency_detail,
            "brake_kept": self.brake_kept,
            "buttons_unavailable": self.buttons_unavailable,
            "sequence": self.sequence.to_dict() if self.sequence is not None else None,
            "needs_ign_off": self.needs_ign_off,
            "paused": self.paused,
            "button_events": [dict(e) for e in self.button_events],
            "dev_stats": self.dev_stats(),
        }
        if self.result is not None:
            out["result"] = {
                k: self.result.get(k) for k in ("exit_code", "meaning", "error", "elapsed_s")
            }
        if self.ended_at is not None:
            out["ended_at"] = self.ended_at.isoformat()
        return out


#: init_step の status（runner）→ 画面の InitStepStatus の値
_INIT_STATUS = {"ok": "done", "ng": "error", "skip": "skipped"}


def apply_event(status: JobStatus, event: dict[str, Any], *, now: float | None = None) -> None:
    """ランナーのイベント（events.py の形式）を状態に反映する（純粋。副作用は status だけ）。

    ack / status の「待っている呼び出し元への返事」は JobManager 側（_dispatch）が行う。
    """
    kind = event.get("type")
    if kind == "state":
        phase = event.get("phase")
        if isinstance(phase, str):
            status.phase = phase
        if "step" in event and event["step"] is not None:
            status.step = event["step"]
        if event.get("title"):
            status.step_title = event["title"]
        step = event.get("step")
        if phase == "step" and step in _BUTTON_STATE_STEPS:
            status.button_state_changed = True
        if phase in ("ready", "d"):
            status.button_state_changed = True
        if phase == "step_end" and step == 25 and event.get("ok"):
            status.ign_off_done = True
        if phase == "emergency":
            stage = event.get("stage")
            status.emergency_stage = str(stage) if stage is not None else None
            status.emergency_detail = str(event.get("detail") or "") or None
            if stage == "ign_off":
                status.ign_off_done = True  # スタートを押し終えた: IGN-OFF を促す案内は出さない
            elif stage == "brake_kept":
                status.brake_kept = True
            elif stage == "buttons_unavailable":
                status.buttons_unavailable = True
        if phase == "done" and event.get("exit_code") is not None:
            status.exit_code = int(event["exit_code"])
        if "paused" in event:  # ランナーが一時停止・再開を確認したとき（後で実装）
            status.paused = bool(event["paused"])
    elif kind == "init_step":
        name = str(event.get("name", ""))
        item = {
            "key": name,
            "label": name,
            "status": _INIT_STATUS.get(str(event.get("status")), "pending"),
            "detail": str(event.get("detail") or ""),
        }
        for i, old in enumerate(status.init_steps):
            if old["key"] == name:
                status.init_steps[i] = item
                break
        else:
            status.init_steps.append(item)
    elif kind == "sample":
        status.sample = {k: v for k, v in event.items() if k not in ("type", "ts")}
        status.sample_at = time.monotonic() if now is None else now
        status.sample_history.append((status.sample_at, status.sample))
        _update_time_ref(status, status.sample, status.sample_at)
        _add_deviation(status, status.sample)
    elif kind == "button":
        _apply_button_event(status, event, time.monotonic() if now is None else now)
    elif kind == "csv":
        status.csv_files.append(
            {"path": event.get("path"), "kind": event.get("kind"), "rows": event.get("rows")}
        )
    elif kind == "result":
        status.result = {k: v for k, v in event.items() if k not in ("type", "ts")}
        if event.get("exit_code") is not None:
            status.exit_code = int(event["exit_code"])
    elif kind == "status":
        status.interactive_status = {k: v for k, v in event.items() if k not in ("type", "ts", "id")}


def _add_deviation(status: JobStatus, sample: dict[str, Any]) -> None:
    """モード走行の行なら |偏差| を昇順の並びへ足す（一時停止中・paused 印の行は除く）。"""
    if sample.get("section") != SECTION_MODE_DRIVE or status.paused or sample.get("paused"):
        return
    ref, speed = sample.get("ref_kmh"), sample.get("speed_kmh")
    if ref is None or speed is None:
        return
    bisect.insort(status.dev_abs, abs(float(speed) - float(ref)))


def _update_time_ref(status: JobStatus, sample: dict[str, Any], at: float) -> None:
    """押下の時刻の物差しを更新する。mode_time_s を受けたジョブはそれだけ、無いジョブは t_s。"""
    m = sample.get("mode_time_s")
    if m is not None:
        status.mode_time_seen = True
        status.time_ref = (float(m), at)
    elif not status.mode_time_seen and sample.get("t_s") is not None:
        status.time_ref = (float(sample["t_s"]), at)


def _apply_button_event(status: JobStatus, event: dict[str, Any], now: float) -> None:
    """button イベント（phase=start/end）を押下の記録にする。時刻は画面の横軸と同じ物差しの今。"""
    name = str(event.get("button", ""))
    now_s = status.time_at(now)
    if event.get("phase") == "start":
        status.button_events.append(
            {
                "button": name,
                "seconds": float(event.get("seconds") or 0.0),
                "start_s": now_s,
                "end_s": None,
            }
        )
        del status.button_events[:-BUTTON_EVENTS_MAX]
    elif event.get("phase") == "end":
        for item in reversed(status.button_events):
            if item["button"] == name and item["end_s"] is None:
                item["end_s"] = max(now_s, item["start_s"])
                if event.get("hold") and event.get("seconds") is not None:
                    item["seconds"] = float(event["seconds"])  # 押し続け: 実際に押した秒数
                break


def exit_to_status(
    code: int | None, *, emergency: bool, interactive: bool, stopped_by_user: bool = False
) -> str:
    """終了コードと非常停止の有無から drive_sessions.status を決める。

    利用者の「走行終了」（stop コマンド）で止めたジョブ（stopped_by_user・130/143）は ``stopped``（異常ではない）。
    """
    if emergency:
        return "emergency"
    if code == 0:
        return "completed"
    if interactive and code in _INTERACTIVE_OK_CODES:
        return "completed"
    if stopped_by_user and code in _INTERACTIVE_OK_CODES:
        return "stopped"
    return "error"


# 「エラー」「失敗」の語を含むが異常ではない決まった文の部分（判定の前に取り除く）。
# 「エラー消去」（アラームリセットの手順名）・「失敗 0 回」（再生の集計）・「中止条件:」（中止する条件の一覧）
_NOT_ERROR_PHRASES = ("エラー消去", "失敗 0 回")


def message_level(text: str) -> str:
    if "中止条件:" in text:
        text = text.split("中止条件:", 1)[0]
    for phrase in _NOT_ERROR_PHRASES:
        text = text.replace(phrase, "")
    if any(w in text for w in ("エラー", "失敗", "Traceback", "Exception")):
        return "ERROR"
    if any(w in text for w in ("警告", "WARNING", "[NG")):
        return "WARNING"
    return "INFO"


def derive_robot_state(manager_state: str, job: JobStatus | None) -> str:
    """画面（RealtimeData.robot_state）の文字列。

    待機中は READY（どのジョブも自分で初期化してから動くので、いつでも始められる）。
    """
    if manager_state == STATE_EMERGENCY:
        return "EMERGENCY"
    if job is None:
        return "READY"
    if job.phase in ("starting", "init"):
        return "INITIALIZING"
    if job.spec.kind == KIND_INTERACTIVE:
        if job.phase == "interactive":
            return "MANUAL" if job.spec.purpose == PURPOSE_MANUAL else "CALIBRATING"
        return "INITIALIZING" if job.phase not in ("shutdown", "job_end", "done") else "RUNNING"
    return "RUNNING"


@dataclass
class _Pending:
    ack: asyncio.Future[dict[str, Any]]
    status: asyncio.Future[dict[str, Any]]


@dataclass
class InteractiveReply:
    ack: dict[str, Any]
    status: dict[str, Any] | None


@dataclass
class _Runner:
    """常駐ランナー（``--serve``）の 1 プロセス。ジョブはここへコマンドで送る。"""

    proc: asyncio.subprocess.Process
    vehicle_id: str
    task: asyncio.Task[None] | None = None
    exited: asyncio.Event = field(default_factory=asyncio.Event)
    exit_code: int | None = None
    needs_init: bool = False  # 最後の idle / result に needs_init が付いていた（次は init が要る）
    exiting: bool = False  # 終わらせる途中（SIGINT・quit を送った／runner_exit の result が来た／見張りの異常）
    watch_error: str | None = None  # 待機中の見張りの error の detail
    kill_task: asyncio.Task[None] | None = None

    @property
    def alive(self) -> bool:
        return self.proc.returncode is None and not self.exited.is_set()


@dataclass
class _Job:
    status: JobStatus
    runner: _Runner
    wire_id: str  # ランナーのイベントの job_id（起動時の初期化は "init"、それ以外は status.job_id）
    task: asyncio.Task[None] | None = None
    kill_task: asyncio.Task[None] | None = None
    seq: int = 0
    pending_messages: list[MessageRow] = field(default_factory=list)
    pending: dict[int, _Pending] = field(default_factory=dict)
    next_cmd_id: int = 1
    cmd_lock: asyncio.Lock = field(default_factory=asyncio.Lock)
    write_lock: asyncio.Lock = field(default_factory=asyncio.Lock)  # 標準入力への書き込みだけを守る（send_button は cmd_lock を通らない）
    ready: asyncio.Event = field(default_factory=asyncio.Event)
    flusher: asyncio.Task[None] | None = None
    # エラーページ用: 直前の進行メッセージ（最大 50 行）と、警告・エラーの行（水準, 文）
    tail: deque[dict[str, str]] = field(default_factory=lambda: deque(maxlen=CONTEXT_MESSAGES_MAX))
    issues: list[tuple[str, str]] = field(default_factory=list)
    result_event: asyncio.Event = field(default_factory=asyncio.Event)  # result が来た（ランナーが落ちたときも）
    settled: asyncio.Event = field(default_factory=asyncio.Event)  # result を受けて最終状態が決まった（次のジョブを始めてよい）
    done: asyncio.Event = field(default_factory=asyncio.Event)  # 後始末（DB への記録）まで済んだ

    @property
    def proc(self) -> asyncio.subprocess.Process:
        return self.runner.proc


# ── シーケンス（段 = ジョブを順に・繰り返し回数どおりに走らせる） ──────────

SEQ_RUNNING = "running"
SEQ_COMPLETED = "completed"
SEQ_STOPPED = "stopped"  # 利用者が止めた
SEQ_ERROR = "error"  # どれかの段が 0 以外で終わった（残りは走らせない）
SEQ_EMERGENCY = "emergency"


@dataclass(frozen=True)
class PlannedStep:
    """シーケンスの 1 段（参照先を解決済み）。kind=schedule は ``schedule``（schedule_to_dict の形）を持つ。"""

    kind: str  # mode / schedule
    ref_id: str
    name: str
    repeat: int = 1
    mode_id: str | None = None
    schedule: dict[str, Any] | None = None


@dataclass(frozen=True)
class RunItem:
    """展開した 1 回分（= 1 ランナー起動 = 1 セッション）。番号は 1 始まり。"""

    cycle: int  # 全体の何周目
    step_no: int  # 何段目
    step_repeat: int  # その段の何回目
    step: PlannedStep


def expand_runs(steps: Sequence[PlannedStep], repeat: int) -> list[RunItem]:
    """全体の繰り返し × 段 × 段の繰り返し を、走らせる順の 1 回分ずつに展開する（純粋）。"""
    runs: list[RunItem] = []
    for cycle in range(1, repeat + 1):
        for step_no, step in enumerate(steps, start=1):
            for rep in range(1, step.repeat + 1):
                runs.append(RunItem(cycle=cycle, step_no=step_no, step_repeat=rep, step=step))
    return runs


@dataclass
class SequenceRun:
    """走っている（または直前に終わった）シーケンスの進み具合。"""

    sequence_id: str
    name: str
    vehicle_id: str
    steps: tuple[PlannedStep, ...]
    cycles: int
    runs: list[RunItem]
    limit_s: float | None = None
    index: int = 0  # 今の（または最後に走らせた）回の 0 始まりの番号
    state: str = SEQ_RUNNING
    message: str | None = None
    started_at: datetime = field(default_factory=lambda: datetime.now(tz=UTC))
    ended_at: datetime | None = None
    stop_requested: bool = False
    results: list[dict[str, Any]] = field(default_factory=list)
    session_ids: list[str] = field(default_factory=list)
    task: asyncio.Task[None] | None = None
    current_job: _Job | None = None
    plan_files: dict[str, str] | None = None  # 準備済みのとき: モード名 → 計画ファイル（2 段目以降は走行前チェックなし）
    plan_dir: Path | None = None  # シーケンスが終わったら消す

    @property
    def current(self) -> RunItem:
        return self.runs[min(self.index, len(self.runs) - 1)]

    def to_dict(self) -> dict[str, Any]:
        item = self.current
        return {
            "sequence_id": self.sequence_id,
            "name": self.name,
            "state": self.state,
            "message": self.message,
            "cycle": item.cycle,
            "cycles": self.cycles,
            "step_no": item.step_no,
            "steps_total": len(self.steps),
            "step_repeat": item.step_repeat,
            "step_repeats": item.step.repeat,
            "run_no": min(self.index + 1, len(self.runs)),
            "runs_total": len(self.runs),
            "kind": item.step.kind,
            "ref_name": item.step.name,
            "started_at": self.started_at.isoformat(),
            "ended_at": self.ended_at.isoformat() if self.ended_at is not None else None,
            "results": list(self.results),
            "session_ids": list(self.session_ids),
        }


# ── 準備（prepare。「開始」のポップアップの間に計画を作る） ─────────────────

PREP_PLANNING = "planning"
PREP_READY = "ready"
PREP_WAITING = "waiting"  # 「はい」が押された後、計画の完成を待っている（できたら始める）
PREP_ERROR = "error"

#: 計画プロセスへ SIGINT を送ってから SIGKILL するまでの猶予 [s]
PLAN_KILL_TIMEOUT_S = 5.0


@dataclass
class PrepareRun:
    """走行の前の「準備」。モードの段ごとに 1 つ（同じモードは 1 つ）の計画プロセスを順に走らせる。"""

    prepare_id: str
    kind: str  # mode / schedule / sequence
    name: str
    vehicle_id: str
    limit_s: float | None
    plan_dir: Path
    mode_names: list[str]  # 計画を作るモード名（重複なし・走る順）
    mode_id: str | None = None  # kind=mode
    schedule: dict[str, Any] | None = None  # kind=schedule（schedule_to_dict の形）
    sequence_id: str | None = None  # kind=sequence
    steps: tuple[PlannedStep, ...] = ()
    cycles: int = 1
    state: str = PREP_PLANNING
    done: int = 0
    message: str | None = None
    plan_files: dict[str, str] = field(default_factory=dict)  # モード名 → 計画ファイル
    task: asyncio.Task[None] | None = None
    proc: asyncio.subprocess.Process | None = None
    cancelled: bool = False
    launching: bool = False  # ジョブを始めている最中（二重の「はい」を弾く）

    @property
    def total(self) -> int:
        return len(self.mode_names)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.prepare_id,
            "kind": self.kind,
            "name": self.name,
            "state": self.state,
            "done": self.done,
            "total": self.total,
            "message": self.message,
        }


# ── JobManager ─────────────────────────────────────────────────────────


class JobManager:
    def __init__(
        self,
        *,
        vehicles_dir: Path,
        common_path: Path,
        hw_mode: str = "stub",
        recorder: DriveRecorder | None = None,
        command_factory: Callable[[str], Sequence[str]] | None = None,
        emergency_probe: Callable[[], bool] | None = None,
        on_min_saved: Callable[[str, str], None] | None = None,
        kill_timeout_s: float = KILL_TIMEOUT_S,
        plan_command_factory: Callable[[str, str, Path], Sequence[str]] | None = None,
        idle_log_path: Path | None = None,
        error_recorder: ErrorRecorder | NullErrorRecorder | None = None,
    ) -> None:
        self.vehicles_dir = Path(vehicles_dir)
        self.common_path = Path(common_path)
        self.hw_mode = hw_mode
        self._recorder: DriveRecorder = recorder or NullDriveRecorder()
        self._command_factory = command_factory
        self._emergency_probe = emergency_probe
        self._on_min_saved = on_min_saved
        self.kill_timeout_s = kill_timeout_s
        self._plan_command_factory = plan_command_factory
        # ジョブの外でランナーが出す行（待機中の見張り・非常停止の後とアプリ終了の終了処理）を残すファイル。
        # ジョブの中の行はセッションのメッセージ（DB）に残る。None なら書かない（テスト）
        self.idle_log_path = Path(idle_log_path) if idle_log_path is not None else None
        self._prepare: PrepareRun | None = None
        self._state = STATE_STANDBY
        # 非常停止に入ったきっかけ: screen（画面のボタン・API）/ switch（非常停止スイッチ）/ ac_loss（AC 電源断）/ auto（走行中の異常でランナーが自動で）
        self._emergency_cause: str | None = None
        self._emergency_cause_detail: str | None = None  # auto のときのランナーの理由（例: SIGINT による中断・CAN 断）
        self._job: _Job | None = None
        self._persist_lock = asyncio.Lock()
        self._persist_tasks: set[asyncio.Task[None]] = set()
        self._runner: _Runner | None = None  # 常駐ランナー（初期化で起動し、ジョブはコマンドで送る）
        self._last: JobStatus | None = None
        self._start_lock = asyncio.Lock()
        self._sequence: SequenceRun | None = None
        self._last_sequence: SequenceRun | None = None
        self._job_counter = 0
        self._recent: deque[str] = deque(maxlen=200)
        # エラーページ（error_events）への記録。省略時は何もしない
        self._errors: ErrorRecorder | NullErrorRecorder = error_recorder or NullErrorRecorder()
        self._tail: deque[dict[str, str]] = deque(maxlen=CONTEXT_MESSAGES_MAX)  # 直前の進行メッセージ（ジョブの内外とも）
        self._emergency_open = False  # 非常停止の件を記録して、解除するまで書き足す間
        # 常駐ランナーが生きていて、初期化（init_only）が成功し、その後 needs_init・ランナーの終了・非常停止が
        # 無い間 True（D-15c）。サーバー起動時は False
        self._initialized = False
        self._last_vehicle_id: str | None = None  # 最後にジョブを始めた車（待機中のエラーの記録用）
        self._shutting_down = False  # アプリ終了（shutdown）で止めたジョブは利用者の走行終了ではない（stopped にしない）

    # ── 状態 ───────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        return self._state

    @property
    def initialized(self) -> bool:
        """初期化ページで初期化済みか（モード走行・学習を始められる条件）。"""
        return self._initialized

    @property
    def current(self) -> JobStatus | None:
        return self._job.status if self._job is not None else None

    @property
    def last_job(self) -> JobStatus | None:
        return self._last

    @property
    def is_busy(self) -> bool:
        return self._job is not None or self._sequence is not None

    @property
    def sequence(self) -> SequenceRun | None:
        return self._sequence

    @property
    def prepare(self) -> PrepareRun | None:
        return self._prepare

    @property
    def recent_messages(self) -> list[str]:
        return list(self._recent)

    def current_interactive_status(self) -> dict[str, Any] | None:
        """対話ジョブの最新の status イベント（位置・車速・min/max）。"""
        cur = self.current
        return cur.interactive_status if cur is not None else None

    def robot_state(self) -> str:
        if self._job is None and self._sequence is not None and self._state != STATE_EMERGENCY:
            return "RUNNING"  # シーケンスの段の切れ目（次のランナーを起動する間）
        return derive_robot_state(self._state, self.current)

    def snapshot(self) -> dict[str, Any]:
        cur = self.current
        return {
            "state": self._state,
            "initialized": self._initialized,
            "robot_state": self.robot_state(),
            "emergency_cause": self._emergency_cause if self._state == STATE_EMERGENCY else None,
            "emergency_cause_detail": (
                self._emergency_cause_detail if self._state == STATE_EMERGENCY else None
            ),
            "job": cur.to_dict() if cur is not None else None,
            "last_job": self._last.to_dict() if self._last is not None else None,
            "sequence": self._sequence.to_dict() if self._sequence is not None else None,
            "prepare": self._prepare.to_dict() if self._prepare is not None else None,
            "last_sequence": (
                self._last_sequence.to_dict() if self._last_sequence is not None else None
            ),
        }

    def sample_history(self) -> dict[str, Any]:
        """実行中のジョブの最近の sample（画面の読み直しで軌跡を描き直す用）。

        時刻は「今から何 ms 前か」（age_ms）で返す（ブラウザと Pi の時計がずれていても位置が合う）。
        """
        cur = self.current
        if cur is None:
            return {"job_id": None, "samples": []}
        now = time.monotonic()
        out = []
        for at, sm in cur.sample_history:
            accel = sm.get("accel_actual_pct")
            brake = sm.get("brake_actual_pct")
            out.append({
                "age_ms": round((now - at) * 1000.0),
                "actual_speed_kmh": float(sm.get("speed_kmh") or 0.0),
                "ref_speed_kmh": sm.get("ref_kmh"),
                "accel_opening": float(accel if accel is not None else sm.get("accel_cmd_pct") or 0.0),
                "brake_opening": float(brake if brake is not None else sm.get("brake_cmd_pct") or 0.0),
            })
        return {"job_id": cur.job_id, "samples": out}

    def live_values(self) -> dict[str, Any]:
        """WebSocket 用のライブ値（車速・基準・開度）。ジョブが無い・まだ値が無ければ 0 / None。"""
        cur = self.current
        speed = 0.0
        ref: float | None = None
        accel = 0.0
        brake = 0.0
        if cur is not None:
            if cur.interactive_status is not None:
                st = cur.interactive_status
                speed = float(st.get("speed_kmh") or 0.0)
                accel = float(st.get("accel_pct") or 0.0)
                brake = float(st.get("brake_pct") or 0.0)
            if cur.sample is not None:
                sm = cur.sample
                speed = float(sm.get("speed_kmh") or 0.0)
                ref = sm.get("ref_kmh")
                accel = float(
                    sm["accel_actual_pct"] if sm.get("accel_actual_pct") is not None
                    else sm.get("accel_cmd_pct") or 0.0
                )
                brake = float(
                    sm["brake_actual_pct"] if sm.get("brake_actual_pct") is not None
                    else sm.get("brake_cmd_pct") or 0.0
                )
        return {"speed_kmh": speed, "ref_kmh": ref, "accel_pct": accel, "brake_pct": brake}

    # ── 起動 ───────────────────────────────────────────────────────

    def _runner_command(self, vehicle_id: str) -> list[str]:
        if self._command_factory is not None:
            return list(self._command_factory(vehicle_id))
        return build_serve_command(
            vehicle_id, vehicles_dir=self.vehicles_dir, common_path=self.common_path, hw=self.hw_mode
        )

    def _live_runner(self) -> _Runner | None:
        r = self._runner
        return r if r is not None and r.alive else None

    def mode_drive_spec(
        self, vehicle_id: str, mode_id: str, mode_name: str, limit_s: float | None = None
    ) -> JobSpec:
        return JobSpec(
            kind=KIND_MODE_DRIVE,
            vehicle_id=vehicle_id,
            steps=MODE_DRIVE_STEPS,
            mode_id=mode_id,
            mode_name=mode_name,
            limit_s=limit_s,
        )

    @staticmethod
    def learning_spec(vehicle_id: str, kind: str) -> JobSpec:
        if kind not in LEARNING_STEPS:
            raise JobError(f"学習の種類は {', '.join(LEARNING_STEPS)} のどれかです: {kind!r}")
        return JobSpec(kind=KIND_LEARNING, vehicle_id=vehicle_id, steps=LEARNING_STEPS[kind])

    @staticmethod
    def ign_off_spec(vehicle_id: str) -> JobSpec:
        return JobSpec(kind=KIND_IGN_OFF, vehicle_id=vehicle_id, steps=IGN_OFF_STEPS)

    @staticmethod
    def init_only_spec(vehicle_id: str) -> JobSpec:
        return JobSpec(kind=KIND_INIT_ONLY, vehicle_id=vehicle_id, steps="1")  # 手順 1 を行うのはこのジョブだけ

    @staticmethod
    def interactive_spec(vehicle_id: str, purpose: str) -> JobSpec:
        return JobSpec(kind=KIND_INTERACTIVE, vehicle_id=vehicle_id, purpose=purpose)

    def _check_can_start(self, sequence: SequenceRun | None = None, *, prepared: bool = False) -> None:
        if self._state == STATE_EMERGENCY:
            raise JobConflict("非常停止中です。非常停止を解除してから始めてください")
        if self._prepare is not None and not prepared:
            raise JobConflict("走行の準備（計画の作成）中です。取り消してから始めてください")
        if self._sequence is not None and self._sequence is not sequence:
            raise JobConflict(
                f"シーケンス「{self._sequence.name}」の実行中です。終わるか停止してから始めてください"
            )
        if self._job is not None:
            kind = self._job.status.spec.kind
            raise JobConflict(f"別のジョブ（{kind}）が実行中です。終わるか停止してから始めてください")

    async def start(self, spec: JobSpec, *, prepared: bool = False) -> JobStatus:
        """ジョブを始める。同時に 1 本だけ（実行中・非常停止中・シーケンス実行中は JobConflict）。

        ``prepared``: 準備（prepare）を消費して始めるとき（準備中の 409 を通す）。"""
        async with self._start_lock:
            return await self._start_locked(spec, prepared=prepared)

    async def _start_locked(
        self,
        spec: JobSpec,
        sequence: SequenceRun | None = None,
        *,
        prepared: bool = False,
        skip_check: bool = False,
    ) -> JobStatus:
        """``start`` の本体（_start_lock を持っている前提）。``sequence`` はシーケンスの段として始めるとき。

        初期化（init_only）は、常駐ランナーが無ければ ``--serve`` で起動し、生きていれば ``init`` コマンドを送る。
        それ以外のジョブは、初期化済みの常駐ランナーへコマンドで送る（ランナーは起動しない）。"""
        if not skip_check:
            self._check_can_start(sequence, prepared=prepared)
        runner = self._live_runner()
        # 初期化は機械の初期化（車と結びつけない）。ジョブは今選んでいる車（spec.vehicle_id）でコマンドを送る
        if spec.kind != KIND_INIT_ONLY:
            ready = self._initialized and runner is not None
            if not ready:
                if spec.kind in _INIT_REQUIRED_KINDS:
                    raise JobConflict(NOT_INITIALIZED_MESSAGE)
                await self._auto_init(spec.vehicle_id)  # スケジュール・手動・キャリブ・IGN-OFF: 初期化から続ける
                runner = self._live_runner()
                if runner is None:
                    raise JobConflict("常駐ランナーが終了しました。初期化してからやり直してください")
        session_id: str | None = None
        if spec.run_type is not None:
            try:
                session_id = await self._recorder.start_session(
                    vehicle_id=spec.vehicle_id,
                    mode_id=spec.mode_id,
                    run_type=spec.run_type,
                    schedule_id=spec.schedule_id if spec.kind == KIND_SCHEDULE else None,
                    schedule_name=spec.schedule_name if spec.kind == KIND_SCHEDULE else None,
                )
            except Exception as exc:
                raise JobUnavailable(
                    f"DB にセッションを作れないため始めません（{type(exc).__name__}: {exc}）。"
                    "DB の接続を確認してください"
                ) from exc
        spawn = runner is None
        if spawn:
            try:
                runner = await self._spawn_runner(spec.vehicle_id)
            except OSError as exc:
                if session_id is not None:
                    await self._safe_end_session(session_id, "error")
                raise JobUnavailable(f"ランナーを起動できません: {exc}") from exc
        assert runner is not None  # noqa: S101
        # ここから _job を置くまで await しない（起動直後のランナーの出力を、このジョブへ振り分けるため）
        self._job_counter += 1
        status = JobStatus(
            job_id=f"job{self._job_counter}-{int(time.time())}",
            spec=spec,
            started_at=datetime.now(tz=UTC),
            t0=time.monotonic(),
            session_id=session_id,
            pid=runner.proc.pid,
        )
        status.sequence = sequence
        job = _Job(status=status, runner=runner, wire_id=STARTUP_JOB_ID if spawn else status.job_id)
        if sequence is not None:
            sequence.current_job = job
            if session_id is not None:
                sequence.session_ids.append(session_id)
        self._job = job
        self._last_vehicle_id = spec.vehicle_id
        job.flusher = asyncio.create_task(self._flush_loop(job))
        self._state = STATE_RUNNING
        if spec.kind == KIND_INIT_ONLY:
            self._initialized = False  # 初期化し直している間は未初期化
            runner.needs_init = False
        self._say(job, f"ジョブ開始: {spec.kind}（車 {spec.vehicle_id}"
                  f"{'、手順 ' + spec.steps if spec.steps else ''}"
                  f"{'、モード ' + spec.mode_name if spec.mode_name else ''}"
                  f"{'、スケジュール ' + spec.schedule_name if spec.schedule_name else ''}"
                  f"、HW {self.hw_mode}、pid {runner.proc.pid}、"
                  f"{'常駐ランナーを起動' if spawn else '常駐ランナーへコマンド送信'}）")
        if not spawn:
            try:
                await self._write(runner, job_command(spec, status.job_id), drain=True)
            except (OSError, ConnectionError) as exc:
                self._job = None
                self._state = STATE_STANDBY if self._state != STATE_EMERGENCY else self._state
                if job.flusher is not None:
                    job.flusher.cancel()
                job.result_event.set()
                job.settled.set()
                job.done.set()
                if session_id is not None:
                    await self._safe_end_session(session_id, "error")
                raise JobUnavailable(f"ランナーへコマンドを送れません: {exc}") from exc
        return status

    async def _auto_init(self, vehicle_id: str) -> None:
        """初期化済みでない常駐ランナーに、手動運転・キャリブレーションの前の初期化をさせる（終わるまで待つ）。"""
        await self._start_locked(self.init_only_spec(vehicle_id), skip_check=True)
        job = self._job
        if job is not None:
            await job.settled.wait()
        if not self._initialized:
            code = self._last.exit_code if self._last is not None else None
            raise JobConflict(f"初期化に失敗しました（終了コード {code}）。進行のメッセージを確認してください")

    async def start_mode_drive(
        self, vehicle_id: str, mode_id: str, mode_name: str, limit_s: float | None = None
    ) -> JobStatus:
        return await self.start(self.mode_drive_spec(vehicle_id, mode_id, mode_name, limit_s))

    def schedule_spec(self, vehicle_id: str, schedule: dict[str, Any]) -> JobSpec:
        return JobSpec(
            kind=KIND_SCHEDULE,
            vehicle_id=vehicle_id,
            schedule_id=str(schedule.get("id") or ""),
            schedule_name=str(schedule.get("name") or ""),
            schedule=schedule,
        )

    async def start_schedule(
        self, vehicle_id: str, schedule: dict[str, Any], *, prepared: bool = False
    ) -> JobStatus:
        """タイムスケジュール（schedule_to_dict の形）を開ループで再生するジョブを始める。"""
        return await self.start(self.schedule_spec(vehicle_id, schedule), prepared=prepared)

    async def start_learning(self, vehicle_id: str, kind: str) -> JobStatus:
        return await self.start(self.learning_spec(vehicle_id, kind))

    async def start_ign_off(self, vehicle_id: str) -> JobStatus:
        return await self.start(self.ign_off_spec(vehicle_id))

    async def start_init_only(self, vehicle_id: str) -> JobStatus:
        return await self.start(self.init_only_spec(vehicle_id))

    # ── 準備（prepare。開始ポップアップの間に計画を作る） ─────────────────

    @staticmethod
    def _remove_plan_dir(path: Path | None) -> None:
        if path is not None:
            shutil.rmtree(path, ignore_errors=True)

    def _plan_command(self, vehicle_id: str, mode_name: str, out: Path) -> list[str]:
        if self._plan_command_factory is not None:
            return list(self._plan_command_factory(vehicle_id, mode_name, out))
        return build_plan_command(
            vehicle_id, mode_name, out, vehicles_dir=self.vehicles_dir, common_path=self.common_path
        )

    async def _start_prepare(self, prep: PrepareRun, *, needs_init: bool) -> PrepareRun:
        async with self._start_lock:
            self._check_can_start()
            if needs_init and not self._initialized:
                raise JobConflict(NOT_INITIALIZED_MESSAGE)
            # 前の準備の取り残し（サーバーの落ち方による）を消してから始める（準備は同時に 1 つなので安全）
            self._remove_plan_dir(prep.plan_dir.parent)
            prep.plan_dir.mkdir(parents=True, exist_ok=True)
            self._prepare = prep
            if prep.mode_names:
                prep.task = asyncio.create_task(self._prepare_loop(prep), name=f"prepare-{prep.prepare_id}")
            else:
                prep.state = PREP_READY  # 作るものなし（スケジュールだけ）
        return prep

    def _new_prepare(self, kind: str, name: str, vehicle_id: str, limit_s: float | None,
                     mode_names: Sequence[str], **kw: Any) -> PrepareRun:
        pid = uuid.uuid4().hex
        distinct = list(dict.fromkeys(mode_names))
        return PrepareRun(
            prepare_id=pid, kind=kind, name=name, vehicle_id=vehicle_id, limit_s=limit_s,
            plan_dir=self.vehicles_dir / vehicle_id / ".work" / "plans" / pid, mode_names=distinct, **kw,
        )

    async def start_prepare_mode(
        self, vehicle_id: str, mode_id: str, mode_name: str, limit_s: float | None = None
    ) -> PrepareRun:
        prep = self._new_prepare("mode", mode_name, vehicle_id, limit_s, [mode_name], mode_id=mode_id)
        return await self._start_prepare(prep, needs_init=True)

    async def start_prepare_schedule(self, vehicle_id: str, schedule: dict[str, Any]) -> PrepareRun:
        prep = self._new_prepare(
            "schedule", str(schedule.get("name") or ""), vehicle_id, None, [], schedule=schedule
        )
        return await self._start_prepare(prep, needs_init=True)

    async def start_prepare_sequence(
        self,
        vehicle_id: str,
        sequence_id: str,
        name: str,
        steps: Sequence[PlannedStep],
        cycles: int,
        limit_s: float | None = None,
    ) -> PrepareRun:
        if not expand_runs(steps, cycles):
            raise JobError("シーケンスに走らせる段がありません")
        mode_names = [st.name for st in steps if st.kind == "mode"]
        prep = self._new_prepare(
            "sequence", name, vehicle_id, limit_s, mode_names,
            sequence_id=sequence_id, steps=tuple(steps), cycles=cycles,
        )
        return await self._start_prepare(prep, needs_init=True)

    async def _prepare_loop(self, prep: PrepareRun) -> None:
        """違うモードごとに計画プロセスを 1 本ずつ順に走らせる。全部できたら ready（待っていれば始める）。"""
        try:
            for n, mode_name in enumerate(prep.mode_names, start=1):
                out = prep.plan_dir / f"{n}.pkl"
                error = await self._run_plan_process(prep, mode_name, out)
                if prep.cancelled:
                    return
                if error is not None:
                    prep.state = PREP_ERROR
                    prep.message = f"モード「{mode_name}」の計画を作れませんでした: {error}"
                    self._record_prepare_error(prep, prep.message)
                    self._remove_plan_dir(prep.plan_dir)
                    return
                prep.plan_files[mode_name] = str(out)
                prep.done += 1
            waiting = prep.state == PREP_WAITING
            prep.state = PREP_READY
            if waiting:
                await self._launch_prepared(prep, from_waiting=True)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001
            logger.exception("準備の進行で例外: %s", exc, extra=RECORDED)
            if not prep.cancelled and self._prepare is prep:
                prep.state = PREP_ERROR
                prep.message = f"準備の進行で予期しない例外: {type(exc).__name__}: {exc}"
                self._record_prepare_error(prep, prep.message, traceback.format_exc())

    async def _run_plan_process(self, prep: PrepareRun, mode_name: str, out: Path) -> str | None:
        """計画プロセスを 1 本走らせて待つ。成功なら None、失敗なら理由（日本語）。"""
        try:
            proc = await asyncio.create_subprocess_exec(
                *self._plan_command(prep.vehicle_id, mode_name, out),
                stdin=asyncio.subprocess.DEVNULL,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                cwd=str(REPO_ROOT),
                limit=STDOUT_LINE_LIMIT,
                start_new_session=True,
            )
        except OSError as exc:
            return f"計画プロセスを起動できません: {exc}"
        prep.proc = proc
        tail: deque[str] = deque(maxlen=5)
        result_error: str | None = None
        assert proc.stdout is not None  # noqa: S101
        try:
            while True:
                raw = await proc.stdout.readline()
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").rstrip()
                ev = events.decode_event_line(line)
                if ev is None:
                    if line.strip():
                        tail.append(line.strip())
                elif ev.get("type") == "result" and ev.get("error"):
                    result_error = str(ev["error"])
            code = await proc.wait()
        except asyncio.CancelledError:
            if proc.returncode is None:
                with contextlib.suppress(ProcessLookupError):
                    proc.kill()
            raise
        finally:
            prep.proc = None
        if code == 0 and out.exists():
            return None
        if result_error:
            return f"{result_error}（終了コード {code}）"
        return f"終了コード {code}" + (f"（{tail[-1]}）" if tail else "")

    async def _launch_prepared(self, prep: PrepareRun, *, from_waiting: bool = False) -> str | None:
        """できた準備でジョブを始める。準備は消費される（status の prepare は None になる）。始められなければ
        例外を投げる（待っていたときは状態を error にする）。戻り値は job_id。"""
        prep.launching = True
        try:
            job_id = await self._launch_prepared_inner(prep)
        except JobError as exc:
            prep.launching = False
            self._remove_plan_dir(prep.plan_dir)
            if from_waiting:
                prep.state = PREP_ERROR
                prep.message = f"始められませんでした: {exc}"
                self._record_prepare_error(prep, prep.message)
                return None
            if self._prepare is prep:
                self._prepare = None
            raise
        if self._prepare is prep:
            self._prepare = None
        return job_id

    async def _launch_prepared_inner(self, prep: PrepareRun) -> str:
        if prep.kind == "mode":
            spec = self.mode_drive_spec(prep.vehicle_id, prep.mode_id or "", prep.name, prep.limit_s)
            spec = replace(spec, plan_file=prep.plan_files.get(prep.name))
            return (await self.start(spec, prepared=True)).job_id
        if prep.kind == "schedule":
            assert prep.schedule is not None  # noqa: S101
            return (await self.start_schedule(prep.vehicle_id, prep.schedule, prepared=True)).job_id
        _, first = await self.start_sequence(
            prep.vehicle_id, prep.sequence_id or "", prep.name, prep.steps, prep.cycles, prep.limit_s,
            plan_files=dict(prep.plan_files), plan_dir=prep.plan_dir, prepared=True,
        )
        return first.job_id

    async def prepare_go(self) -> tuple[str, str | None]:
        """「はい」: できていればすぐ始める（("started", job_id)）、まだなら待たせる（("waiting", None)）。"""
        prep = self._prepare
        if prep is None:
            raise JobConflict("準備がありません")
        if prep.state == PREP_ERROR:
            raise JobConflict(prep.message or "計画を作れなかったため始められません")
        if prep.launching:
            raise JobConflict("すでに始めています")
        if prep.state == PREP_WAITING:
            return "waiting", None
        if prep.state == PREP_PLANNING:
            prep.state = PREP_WAITING
            return "waiting", None
        job_id = await self._launch_prepared(prep)
        return "started", job_id

    async def cancel_prepare(self) -> bool:
        """準備を取り消す（計画プロセスへ SIGINT → 猶予後 SIGKILL。ファイルを消す）。無ければ False。"""
        prep = self._prepare
        if prep is None or prep.launching:
            return False  # 無い／もうジョブを始めている最中（取り消せない）
        prep.cancelled = True
        self._prepare = None
        proc = prep.proc
        if proc is not None and proc.returncode is None:
            with contextlib.suppress(ProcessLookupError):
                proc.send_signal(signal.SIGINT)
            try:
                await asyncio.wait_for(proc.wait(), PLAN_KILL_TIMEOUT_S)
            except TimeoutError:
                with contextlib.suppress(ProcessLookupError):
                    proc.kill()
        task = prep.task
        if task is not None and not task.done() and task is not asyncio.current_task():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await task
        self._remove_plan_dir(prep.plan_dir)
        return True

    # ── シーケンス ─────────────────────────────────────────────────

    def _run_spec(self, seq: SequenceRun, item: RunItem) -> JobSpec:
        step = item.step
        # 準備済みのシーケンスは、2 段目以降の走行前チェックを行わない（段と段の間を待たせない）
        no_pre_check = seq.plan_files is not None and seq.index > 0
        # 原点復帰は全体のジョブの終わり（最後の run）だけ。段の間は停車保持のまま次へ（ユーザー決定 2026-11-20）
        home_at_end = seq.index >= len(seq.runs) - 1
        if step.kind == "mode":
            spec = self.mode_drive_spec(seq.vehicle_id, step.mode_id or step.ref_id, step.name, seq.limit_s)
            plan_file = (seq.plan_files or {}).get(step.name)
            return replace(spec, plan_file=plan_file, no_pre_check=no_pre_check, home_at_end=home_at_end)
        assert step.schedule is not None  # noqa: S101 - 計画の時点で保証（start_sequence）
        spec = self.schedule_spec(seq.vehicle_id, step.schedule)
        return replace(spec, no_pre_check=no_pre_check, home_at_end=home_at_end)

    async def _start_run_locked(self, seq: SequenceRun, *, prepared: bool = False) -> JobStatus:
        spec = self._run_spec(seq, seq.runs[seq.index])
        return await self._start_locked(spec, seq, prepared=prepared)

    async def start_sequence(
        self,
        vehicle_id: str,
        sequence_id: str,
        name: str,
        steps: Sequence[PlannedStep],
        cycles: int,
        limit_s: float | None = None,
        *,
        plan_files: dict[str, str] | None = None,
        plan_dir: Path | None = None,
        prepared: bool = False,
    ) -> tuple[SequenceRun, JobStatus]:
        """シーケンスを始める。段を順に、繰り返し回数どおり 1 回ずつジョブ（常駐ランナーへ 1 コマンド・1 セッション）として走らせる。

        最初の 1 回はここで起動する（始められない理由は呼び出し元に返る）。残りはバックグラウンドで続ける。
        どれかが 0 以外で終わったら残りは走らせず止める。停止・非常停止で全体を止める。
        """
        runs = expand_runs(steps, cycles)
        if not runs:
            raise JobError("シーケンスに走らせる段がありません")
        async with self._start_lock:
            self._check_can_start(prepared=prepared)
            seq = SequenceRun(
                sequence_id=sequence_id, name=name, vehicle_id=vehicle_id, steps=tuple(steps),
                cycles=cycles, runs=runs, limit_s=limit_s, plan_files=plan_files, plan_dir=plan_dir,
            )
            self._sequence = seq
            try:
                first = await self._start_run_locked(seq, prepared=prepared)
            except BaseException:
                self._sequence = None
                raise
            seq.task = asyncio.create_task(self._sequence_loop(seq), name=f"sequence-{sequence_id}")
        return seq, first

    async def _sequence_loop(self, seq: SequenceRun) -> None:
        try:
            while True:
                job = seq.current_job
                assert job is not None  # noqa: S101
                if seq.stop_requested and not job.status.stop_requested:
                    # 起動の最中に止められた（stop は走っているジョブを見つけられなかった）
                    job.status.stop_requested = True
                    self._send_stop(job)
                await job.settled.wait()  # result を受けた時点（DB への取り込みは裏で続く）
                st = job.status
                item = seq.runs[seq.index]
                seq.results.append({
                    "run_no": seq.index + 1, "kind": item.step.kind, "name": item.step.name,
                    "final_status": st.final_status, "exit_code": st.exit_code, "session_id": st.session_id,
                })
                label = self._run_label(seq, item)
                if st.final_status != "completed":
                    if st.emergency or self._state == STATE_EMERGENCY:
                        seq.state = SEQ_EMERGENCY
                        seq.message = f"{label}の途中で非常停止しました。残りは走らせません"
                    elif seq.stop_requested:
                        seq.state = SEQ_STOPPED
                        seq.message = f"{label}の途中で停止しました。残りは走らせません"
                    else:
                        code = st.exit_code
                        why = (st.result or {}).get("error") or events.exit_meaning(code) if code is not None else "不明"
                        seq.state = SEQ_ERROR
                        seq.message = f"{label}が終了コード {code}（{why}）で終わったため、残りは走らせません"
                    return
                if seq.index + 1 >= len(seq.runs):
                    seq.state = SEQ_COMPLETED
                    return
                if seq.stop_requested:
                    seq.state = SEQ_STOPPED
                    seq.message = f"{label}の後で停止しました。残りは走らせません"
                    return
                seq.index += 1
                try:
                    async with self._start_lock:
                        if seq.stop_requested:
                            seq.state = SEQ_STOPPED
                            seq.message = f"{label}の後で停止しました。残りは走らせません"
                            return
                        await self._start_run_locked(seq)
                except JobError as exc:
                    seq.state = SEQ_EMERGENCY if self._state == STATE_EMERGENCY else SEQ_ERROR
                    seq.message = f"{self._run_label(seq, seq.runs[seq.index])}を始められません: {exc}"
                    self._record_sequence_error(seq, seq.message)
                    return
        except Exception as exc:  # noqa: BLE001
            logger.exception("シーケンスの進行で例外: %s", exc, extra=RECORDED)
            seq.state = SEQ_ERROR
            seq.message = f"シーケンスの進行で予期しない例外: {type(exc).__name__}: {exc}"
            self._record_sequence_error(seq, seq.message, traceback.format_exc())
        finally:
            seq.ended_at = datetime.now(tz=UTC)
            if seq.state == SEQ_RUNNING:
                seq.state = SEQ_ERROR
            logger.info("シーケンス終了: %s（%s）%s", seq.name, seq.state, seq.message or "")
            self._last_sequence = seq
            self._sequence = None
            self._remove_plan_dir(seq.plan_dir)

    @staticmethod
    def _run_label(seq: SequenceRun, item: RunItem) -> str:
        return (
            f"{item.step_no} 段目「{item.step.name}」（{item.step_repeat}/{item.step.repeat} 回目、"
            f"全体 {item.cycle}/{seq.cycles} 周目）"
        )

    # ── 常駐ランナー（起動・読み取り・イベントの振り分け） ─────────────────

    async def _spawn_runner(self, vehicle_id: str) -> _Runner:
        """常駐ランナーを起動する（起動時の初期化は ``job_id: "init"`` のイベントで知らせてくる）。"""
        proc = await asyncio.create_subprocess_exec(
            *self._runner_command(vehicle_id),
            stdin=asyncio.subprocess.PIPE,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
            cwd=str(REPO_ROOT),
            limit=STDOUT_LINE_LIMIT,
            # 端末の Ctrl+C（uvicorn 宛）がランナーへ直接届かないように。停止は必ずここから送る
            start_new_session=True,
        )
        runner = _Runner(proc=proc, vehicle_id=vehicle_id)
        self._runner = runner
        runner.task = asyncio.create_task(self._runner_main(runner), name=f"runner-{proc.pid}")
        return runner

    @staticmethod
    async def _write(runner: _Runner, command: dict[str, Any], *, drain: bool = False) -> None:
        stdin = runner.proc.stdin
        if stdin is None or not runner.alive:
            raise ConnectionError("常駐ランナーは終了しています")
        stdin.write((json.dumps(command, ensure_ascii=False) + "\n").encode())
        if drain:
            await stdin.drain()

    def _write_nowait(self, runner: _Runner, command: dict[str, Any]) -> bool:
        """短い制御コマンド（stop・quit など）を書く。書けなければ False（ランナーはもう終わっている）。"""
        stdin = runner.proc.stdin
        if stdin is None or not runner.alive:
            return False
        try:
            stdin.write((json.dumps(command, ensure_ascii=False) + "\n").encode())
        except (OSError, RuntimeError):
            return False
        return True

    async def _runner_main(self, runner: _Runner) -> None:
        """常駐ランナーの標準出力を終わりまで読み、イベントをジョブへ振り分ける。終わったら後始末。"""
        stdout = runner.proc.stdout
        assert stdout is not None  # noqa: S101
        code: int | None = None
        try:
            while True:
                try:
                    raw = await stdout.readline()
                except (asyncio.LimitOverrunError, ValueError):
                    self._say_runner(runner, "(出力の 1 行が長すぎるため読み飛ばしました)", "WARNING")
                    continue
                if not raw:
                    break
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                event = events.decode_event_line(line)
                if event is None:
                    self._say_runner(runner, line)
                else:
                    self._on_event(runner, event)
            code = await runner.proc.wait()
        except BaseException:  # noqa: BLE001 - 取り消し（アプリ終了）でも後始末へ進む
            if runner.proc.returncode is None:
                with contextlib.suppress(ProcessLookupError):
                    runner.proc.kill()
            raise
        finally:
            self._on_runner_exit(runner, code if code is not None else runner.proc.returncode)

    def _current_job_of(self, runner: _Runner) -> _Job | None:
        job = self._job
        if job is not None and job.runner is runner and not job.result_event.is_set():
            return job
        return None

    def _say_idle(self, text: str, level: str | None = None, *, record: bool = True) -> None:
        """ジョブが無い間のランナーの行（待機中の表示・見張り）。画面のサーバーログと直近の行に出し、
        警告・エラーの行はエラーページへも残す（``record=False`` は別の件がもう伝えているとき）。"""
        text = text.rstrip("\r\n")
        if not text:
            return
        level = level or message_level(text)
        self._recent.append(text)
        self._tail.append(self._tail_row(level, text))
        logger.log(
            {"ERROR": logging.ERROR, "WARNING": logging.WARNING}.get(level, logging.INFO), "%s", text,
            extra=RECORDED,
        )
        self._append_idle_log(level, text)
        if record and level in ("ERROR", "WARNING"):
            self._record_idle(level, text)
        elif self._emergency_open and "終了処理" in text:
            # 非常停止の後の終了処理（原点復帰 → サーボOFF → 切断・完了）の行は、非常停止の件へ書き足す
            try:
                self._errors.append_emergency(text)
            except Exception as exc:  # noqa: BLE001 - 記録の失敗で制御を止めない
                logger.debug("エラーの記録に失敗: %s", exc)

    # ── エラーページへの記録（待たない・失敗しても制御に影響しない） ───────────

    @staticmethod
    def _tail_row(level: str, text: str) -> dict[str, str]:
        return {"time": datetime.now(tz=UTC).isoformat(timespec="milliseconds"), "level": level, "text": text}

    @staticmethod
    def _title_of(text: str, limit: int = 120) -> str:
        first = text.strip().splitlines()[0] if text.strip() else ""
        return first[:limit]

    def _record(self, ev: NewErrorEvent) -> None:
        try:
            self._errors.record(ev)
        except Exception as exc:  # noqa: BLE001 - 記録の失敗で制御を止めない
            logger.debug("エラーの記録に失敗: %s", exc)

    def _record_idle(self, level: str, text: str) -> None:
        """待機中（ジョブの外）の警告・エラーの行。非常停止の件が開いている間はその本文へ書き足す。"""
        try:
            if self._emergency_open:
                self._errors.append_emergency(f"[{level}] {text}")
                return
            runner = self._runner
            self._record(NewErrorEvent(
                level=LEVEL_ERROR if level == "ERROR" else LEVEL_WARNING,
                source=SOURCE_IDLE,
                title=self._title_of(text),
                detail=text,
                context={"messages": list(self._tail)},
                vehicle_id=self._last_vehicle_id or (runner.vehicle_id if runner is not None else None),
                dedupe=True,
            ))
        except Exception as exc:  # noqa: BLE001
            logger.debug("エラーの記録に失敗: %s", exc)

    @staticmethod
    def _job_label(spec: JobSpec) -> str:
        if spec.kind == KIND_INTERACTIVE:
            return _PURPOSE_LABEL.get(spec.purpose or "", "対話ジョブ")
        return _KIND_LABEL.get(spec.kind, spec.kind)

    @staticmethod
    def _end_meaning(status: Any) -> str:
        """終了の意味の文。ランナーの result の meaning（例: 中断（stop コマンド））があればそれ、無ければコード別の一般の文。"""
        code = status.exit_code
        result = status.result
        if isinstance(result, dict):
            text = result.get("meaning")
            if isinstance(text, str) and text:
                return text
        return events.exit_meaning(code) if code is not None else "不明"

    def _record_job_end(self, job: _Job, issues: list[tuple[str, str]]) -> None:
        """ジョブの終わりの 1 件（_settle から）。非常停止のジョブは非常停止の件へ終了処理の結果を書き足す。"""
        try:
            st = job.status
            code = st.exit_code
            meaning = self._end_meaning(st)
            label = self._job_label(st.spec)
            lines = [f"[{lv}] {tx}" for lv, tx in issues[:_ISSUE_LINES_MAX]]
            if len(issues) > _ISSUE_LINES_MAX:
                lines.append(f"…ほか {len(issues) - _ISSUE_LINES_MAX} 件")
            if st.emergency and self._emergency_open:
                text = "\n".join([f"終了処理の結果: {label}は終了コード {code}（{meaning}）で終わりました", *lines])
                self._errors.append_emergency(
                    text, context_patch={"job_end": {"exit_code": code, "meaning": meaning, "job_id": st.job_id}}
                )
                return
            stopped_by_user = st.final_status == "stopped" or (
                st.stop_requested and not st.emergency and code in (130, 143)
            )
            if st.final_status == "completed" or stopped_by_user:
                if not issues:
                    return
                level = LEVEL_WARNING
                title = (
                    f"{label}: 利用者が停止（警告・エラーの行 {len(issues)} 件）" if stopped_by_user
                    else f"{label}: 警告ありで完了（{len(issues)} 件）"
                )
                head = f"終了コード {code}（{meaning}）"
            else:
                level = LEVEL_ERROR
                title = f"{label}が異常終了（終了コード {code}: {meaning}）"
                head = f"終了コード {code}（{meaning}）"
            parts = [head]
            result_error = (st.result or {}).get("error")
            if result_error:
                parts.append(f"理由: {result_error}")
            seq = st.sequence
            seq_name: str | None = None
            if seq is not None:
                seq_name = seq.name
                parts.append(f"シーケンス: 「{seq.name}」{self._run_label(seq, seq.runs[seq.index])}")
            if lines:
                parts.append(f"警告・エラーの行（{len(issues)} 件）:")
                parts.extend(lines)
            ctx: dict[str, Any] = {"messages": list(job.tail), "exit_code": code}
            if seq is not None:
                ctx["sequence_step"] = {
                    "run_no": seq.index + 1, "runs_total": len(seq.runs), "step_no": seq.runs[seq.index].step_no,
                }
            self._record(NewErrorEvent(
                level=level, source=SOURCE_JOB, title=title, detail="\n".join(parts), context=ctx,
                job_id=st.job_id, job_kind=st.spec.kind, vehicle_id=st.spec.vehicle_id,
                session_id=st.session_id, sequence_name=seq_name,
            ))
        except Exception as exc:  # noqa: BLE001
            logger.debug("エラーの記録に失敗: %s", exc)

    def _record_emergency(self, cause: str, detail: str | None) -> None:
        """非常停止に入った（_enter_emergency）。1 件にして、ジョブの終わりに終了処理の結果を書き足す。"""
        try:
            label = _CAUSE_LABEL.get(cause, cause)
            parts = [f"きっかけ: {label}"]
            if detail:
                parts.append(f"理由: {detail}")
            job = self._job
            job_id = job_kind = session_id = seq_name = None
            vehicle_id = None
            if job is not None:
                st = job.status
                job_id, job_kind, session_id, vehicle_id = st.job_id, st.spec.kind, st.session_id, st.spec.vehicle_id
                parts.append(f"そのとき: {self._job_label(st.spec)}を実行中")
            elif self._runner is not None:
                vehicle_id = self._last_vehicle_id or self._runner.vehicle_id
                parts.append("そのとき: 待機中")
            seq = self._sequence
            if seq is not None:
                seq_name = seq.name
                parts.append(f"シーケンス: 「{seq.name}」{self._run_label(seq, seq.runs[seq.index])}")
            self._emergency_open = True
            self._record(NewErrorEvent(
                level=LEVEL_ERROR, source=SOURCE_EMERGENCY, title=f"非常停止（{label}）",
                detail="\n".join(parts), context={"messages": list(self._tail), "cause": cause},
                job_id=job_id, job_kind=job_kind, vehicle_id=vehicle_id, session_id=session_id,
                sequence_name=seq_name,
            ))
        except Exception as exc:  # noqa: BLE001
            logger.debug("エラーの記録に失敗: %s", exc)

    def _record_prepare_error(self, prep: PrepareRun, message: str, detail: str | None = None) -> None:
        try:
            seq_name = prep.name if prep.kind == "sequence" else None
            kind = {"mode": "モード", "schedule": "スケジュール", "sequence": "シーケンス"}.get(prep.kind, prep.kind)
            self._record(NewErrorEvent(
                level=LEVEL_ERROR, source=SOURCE_PREPARE,
                title=f"計画の準備に失敗（{kind}「{prep.name}」）",
                detail=message if not detail else f"{message}\n{detail}",
                context={"messages": list(self._tail), "prepare_id": prep.prepare_id},
                vehicle_id=prep.vehicle_id, sequence_name=seq_name,
            ))
        except Exception as exc:  # noqa: BLE001
            logger.debug("エラーの記録に失敗: %s", exc)

    def _record_sequence_error(self, seq: SequenceRun, message: str, detail: str | None = None) -> None:
        """ジョブ以外の理由でシーケンスが止まった（段を始められない等）。段のジョブの失敗はジョブの件に付く。"""
        try:
            self._record(NewErrorEvent(
                level=LEVEL_ERROR, source=SOURCE_SEQUENCE, title=f"シーケンス「{seq.name}」が止まりました",
                detail=message if not detail else f"{message}\n{detail}",
                context={"messages": list(self._tail)}, vehicle_id=seq.vehicle_id, sequence_name=seq.name,
            ))
        except Exception as exc:  # noqa: BLE001
            logger.debug("エラーの記録に失敗: %s", exc)

    def _append_idle_log(self, level: str, text: str) -> None:
        """ジョブの外の行を ``idle_log_path`` へ 1 行ずつ追記する（時刻は JST のローカル時刻）。書けなくても止めない。"""
        path = self.idle_log_path
        if path is None:
            return
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as f:
                f.write(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]} {level:<7} {text}\n")  # noqa: DTZ005
        except OSError as exc:
            logger.warning("ジョブの外の記録を書けません（%s）: %s", path, exc)
            self.idle_log_path = None  # 1 回だけ知らせる

    def _say_runner(self, runner: _Runner, text: str, level: str | None = None) -> None:
        job = self._current_job_of(runner)
        if job is not None:
            self._say(job, text, level)
        else:
            self._say_idle(text, level)

    def _on_event(self, runner: _Runner, event: dict[str, Any]) -> None:
        """ランナーのイベントを振り分ける。ジョブの区切りは ``job_id``（``result`` が終わり）。"""
        kind = event.get("type")
        if kind == "state" and event.get("phase") == "idle":
            runner.needs_init = bool(event.get("needs_init"))
            if runner.needs_init:
                self._initialized = False
            return
        if kind == "watch":
            self._on_watch(runner, event)
            return
        job = self._current_job_of(runner)
        if job is None:
            if kind == "state" and event.get("phase") == "emergency":
                # 非常停止の件が開いていれば、その本文へ書き足される
                self._say_idle(f"停止の流れ: {event.get('stage')} {event.get('detail') or ''}".rstrip(), "WARNING")
            return
        job_id = event.get("job_id")
        if job_id is not None and job_id != job.wire_id:
            return  # 前のジョブの遅れたイベント
        if kind == "ack" and job_id == job.wire_id and event.get("id") is None and event.get("ok") is False:
            self._on_job_rejected(job, event)
            return
        self._dispatch(job, event)
        if kind == "result":
            self._on_result(job, event)

    def _on_watch(self, runner: _Runner, event: dict[str, Any]) -> None:
        """待機中の見張り（A-D3）。warn は警告のメッセージ、error は ERROR のメッセージ＋非常停止の状態（きっかけ auto）。"""
        level = event.get("level")
        detail = str(event.get("detail") or "")
        if level == "error":
            runner.watch_error = detail or "待機中の見張りの異常"
            runner.exiting = True
            # 非常停止の件（きっかけ auto＋理由）が伝えるので、この行は別の件にしない
            self._say_idle(f"待機中の見張りが異常を検出しました: {detail}", "ERROR", record=False)
            self._enter_emergency("auto", detail=detail)
        else:
            self._say_idle(f"待機中の見張り（警告）: {detail}", "WARNING")

    def _on_job_rejected(self, job: _Job, event: dict[str, Any]) -> None:
        """ランナーがジョブのコマンドを断った（実行中・初期化が必要など）。ジョブを失敗で終える。"""
        message = str(event.get("message") or "ランナーがコマンドを受け付けませんでした")
        self._say(job, f"ランナーがコマンドを断りました: {message}", "ERROR")
        if "初期化" in message:
            self._initialized = False
        job.status.result = {"exit_code": 5, "meaning": events.exit_meaning(5), "error": message, "elapsed_s": job.status.elapsed_s()}
        job.status.exit_code = 5
        job.result_event.set()
        job.ready.set()
        self._start_finish(job)

    def _on_result(self, job: _Job, event: dict[str, Any]) -> None:
        """ジョブの終わり（result）。初期化の印を更新し、後始末（DB への記録）を始める。"""
        runner = job.runner
        job.result_event.set()
        code = event.get("exit_code")
        if event.get("needs_init"):
            runner.needs_init = True
        if event.get("runner_exit"):
            runner.exiting = True
        if event.get("needs_init") or event.get("runner_exit"):
            if self._initialized:
                self._say_global("初期化済みの印を消しました（ランナーが初期化のやり直しか終了を求めています）。次は初期化してください")
            self._initialized = False
        elif job.status.spec.kind == KIND_INIT_ONLY:
            self._initialized = code == 0
        self._start_finish(job)

    def _on_runner_exit(self, runner: _Runner, code: int | None) -> None:
        """常駐ランナーのプロセスが終わった。実行中のジョブはここで終える。"""
        runner.exit_code = code
        runner.exited.set()
        if runner.kill_task is not None and not runner.kill_task.done():
            runner.kill_task.cancel()
        if self._runner is runner:
            self._runner = None
            self._initialized = False
        job = self._job
        if job is not None and job.runner is runner and not job.result_event.is_set():
            job.status.exit_code = code
            job.result_event.set()
            job.ready.set()
            self._start_finish(job)
        if runner.watch_error is not None and self._state != STATE_EMERGENCY:
            self._enter_emergency("auto", detail=runner.watch_error)
        elif job is None and not runner.exiting:
            self._say_idle(f"常駐ランナーが終了しました（終了コード {code}）。初期化し直してください", "WARNING")
        if self._emergency_open and runner.exiting:
            # 非常停止の流れの終わり（ランナーの終了）。件の本文へ書き足す
            self._errors.append_emergency(f"常駐ランナーが終了しました（終了コード {code}）")

    def _say(self, job: _Job, text: str, level: str | None = None) -> None:
        """進行のメッセージを 1 行足す（ログバッファ・DB 用の溜め・直近の行）。"""
        text = text.rstrip("\r\n")
        if not text:
            return
        level = level or message_level(text)
        job.seq += 1
        self._recent.append(text)
        row = self._tail_row(level, text)
        job.tail.append(row)
        self._tail.append(row)
        if level in ("ERROR", "WARNING"):
            job.issues.append((level, text))  # ジョブの終わりの 1 件の本文にまとめる（行ごとには記録しない）
        logger.log(
            {"ERROR": logging.ERROR, "WARNING": logging.WARNING}.get(level, logging.INFO),
            "%s", text, extra=RECORDED,
        )
        job.pending_messages.append(
            (job.seq, round(time.monotonic() - job.status.t0, 3), datetime.now(tz=UTC), level, text)
        )

    async def _flush_messages(self, job: _Job) -> None:
        sid = job.status.session_id
        if not job.pending_messages:
            return
        rows, job.pending_messages = job.pending_messages, []
        if sid is None:
            return
        try:
            await self._recorder.add_messages(sid, rows)
        except Exception as exc:
            logger.warning("進行のメッセージを DB に書けません: %s", exc)

    async def _flush_loop(self, job: _Job) -> None:
        while True:
            await asyncio.sleep(MESSAGE_FLUSH_INTERVAL_S)
            await self._flush_messages(job)

    def _dispatch(self, job: _Job, event: dict[str, Any]) -> None:
        apply_event(job.status, event)
        kind = event.get("type")
        if kind == "state" and event.get("phase") == "emergency" and event.get("stage") == "brake_kept":
            self._say(
                job,
                "停車を確認できず、ブレーキを最大開度で踏んだまま終えます（サーボ ON のまま）。"
                "車の状態を確認し、ブレーキを手で保持してから対処してください",
                "WARNING",
            )
        if kind == "state" and event.get("phase") == "emergency" and event.get("stage") == "buttons_unavailable":
            self._say(
                job,
                "ボタンを押せません（停車保持のまま）。ブレーキは停車保持の開度で止めたまま終えます（サーボ ON のまま）",
                "WARNING",
            )
        if (
            kind == "state"
            and event.get("phase") == "emergency"
            and event.get("stage") == "start"
            and not job.status.emergency
            and not job.status.stop_requested
        ):
            # ランナーが自分で始めた自動の非常停止（走行中の DriveError。CAN 断など）。流れはもう走っているので
            # SIGINT は送らず、画面・スイッチの非常停止と同じ状態（EMERGENCY・解除待ち）にだけ揃える。
            # 「走行終了」（stop コマンド）で出る停止の流れ（stop_requested）は非常停止の状態に入れない
            # （段階の表示 phase=emergency は apply_event が残す）
            detail = str(event.get("detail") or "")
            self._enter_emergency("auto", detail=detail or None)
            job.status.emergency = True
            job.status.stop_requested = True
            self._emergency_cause_detail = detail or None
            self._say(
                job,
                f"走行中の異常でランナーが自動の非常停止に入りました（{detail}）。"
                "減速して停車 → N → IGN-OFF → 原点復帰の後、非常停止を解除してください",
                "WARNING",
            )
        if kind == "button" and (
            (event.get("phase") == "start" and not event.get("hold"))
            or (event.get("phase") == "end" and event.get("hold"))
        ):
            # ボタン押下の機械可読な記録（level BUTTON・time は押し始めの壁時計）。ログ画面の走行グラフが読む。
            # 押し続け（hold）は押し始めの時点で秒数が決まらないので、離したとき（end）に
            # 「押し始めの時刻 ＝ 今 − 秒数」・実際の秒数で 1 行書く
            seconds = float(event.get("seconds") or 0.0)
            back = seconds if event.get("hold") else 0.0
            job.seq += 1
            job.pending_messages.append(
                (
                    job.seq,
                    round(max(0.0, time.monotonic() - job.status.t0 - back), 3),
                    datetime.now(tz=UTC) - timedelta(seconds=back),
                    LEVEL_BUTTON,
                    button_message(str(event.get("button", "")), seconds),
                )
            )
        if kind == "state" and event.get("phase") == "interactive":
            job.ready.set()
        elif kind in ("ack", "status"):
            pending = job.pending.get(event.get("id")) if event.get("id") is not None else None
            if pending is not None:
                fut = pending.ack if kind == "ack" else pending.status
                if not fut.done():
                    fut.set_result({k: v for k, v in event.items() if k not in ("type", "ts")})

    def _start_finish(self, job: _Job) -> None:
        """result を受けた（ランナーが落ちた）とき。最終状態をすぐ決めて次のジョブを始められるようにし、
        DB への記録（メッセージ・CSV 取り込み・セッション終了）は裏で続ける（段の間を延ばさない）。"""
        if job.settled.is_set():
            return
        self._settle(job)
        task = asyncio.create_task(self._persist(job), name=f"persist-{job.status.job_id}")
        job.task = task
        self._persist_tasks.add(task)
        task.add_done_callback(self._persist_tasks.discard)

    def _settle(self, job: _Job) -> None:
        status = job.status
        code = status.exit_code
        meaning = self._end_meaning(status)
        interactive = status.spec.kind == KIND_INTERACTIVE
        status.final_status = exit_to_status(
            code,
            emergency=status.emergency,
            interactive=interactive,
            stopped_by_user=status.stop_requested and not status.emergency and not self._shutting_down,
        )
        issues = list(job.issues)  # 終了の行（下の _say）は含めない
        self._say(
            job,
            # 利用者の走行終了は使う人の言葉で（コードは出さない。ユーザー決定 2026-11-21 O-10）
            "終了しました（停止ボタンで中断）"
            if status.final_status == "stopped"
            else f"ジョブ終了: 終了コード {code}（{meaning}）"
            + ("、非常停止で止めました" if status.emergency else ""),
            "INFO" if status.final_status in ("completed", "stopped") else "WARNING",
        )
        self._record_job_end(job, issues)
        if job.flusher is not None:
            job.flusher.cancel()
        for pending in job.pending.values():
            for fut in (pending.ack, pending.status):
                if not fut.done():
                    fut.cancel()
        status.ended_at = datetime.now(tz=UTC)
        status.sample = None
        self._last = status
        if self._job is job:
            self._job = None
        if self._state != STATE_EMERGENCY:
            self._state = STATE_STANDBY
        if job.kill_task is not None and not job.kill_task.done():
            job.kill_task.cancel()
        job.ready.set()  # 待っている人を起こす（もう対話できない）
        job.settled.set()

    async def _persist(self, job: _Job) -> None:
        """ジョブの記録を DB へ（ジョブの終わり順に 1 本ずつ。次のジョブの記録と混ざらない）。"""
        status = job.status
        sid = status.session_id
        async with self._persist_lock:
            try:
                await self._flush_messages(job)
                if sid is not None:
                    await self._import_csvs(job)
                    await self._flush_messages(job)
                    await self._recorder.end_session(sid, status.final_status or "error")
            except Exception as exc:  # noqa: BLE001
                logger.exception("ジョブ終了後の記録に失敗しました: %s", exc)
                # 失敗の記録（status は DB 側で running のままにしない）
                if sid is not None:
                    await self._safe_end_session(sid, "error")
            finally:
                if status.sequence is None and status.spec.plan_file:
                    self._remove_plan_dir(Path(status.spec.plan_file).parent)
                job.done.set()

    async def wait_persisted(self) -> None:
        """終わったジョブの DB への記録（裏で続けているもの）が全部済むまで待つ。"""
        while True:
            pending = [t for t in self._persist_tasks if not t.done()]
            if not pending:
                return
            await asyncio.gather(*pending, return_exceptions=True)

    @staticmethod
    def _say_global(text: str) -> None:
        logger.warning("%s", text, extra=RECORDED)  # 案内なので、エラーページの件にはしない

    async def _safe_end_session(self, session_id: str, status: str) -> None:
        try:
            await self._recorder.end_session(session_id, status)
        except Exception as exc:  # noqa: BLE001
            logger.warning("セッションを終了済みにできません: %s", exc)

    async def _import_csvs(self, job: _Job) -> None:
        """csv イベントで知らされた CSV を drive_logs へ取り込む。drive の CSV は取り込み後に消す。"""
        sid = job.status.session_id
        assert sid is not None  # noqa: S101
        for item in job.status.csv_files:
            path = Path(str(item.get("path")))
            kind = item.get("kind")
            if not path.is_file():
                self._say(job, f"CSV が見つかりません（取り込めません）: {path}", "WARNING")
                continue
            try:
                n = await self._recorder.import_csv(sid, path)
            except Exception as exc:  # noqa: BLE001
                self._say(
                    job,
                    f"走行ログの DB への取り込みに失敗しました（CSV は残します）: {path.name}"
                    f"（{type(exc).__name__}: {exc}）",
                    "ERROR",
                )
                continue
            self._say(job, f"走行ログ {n} 行を DB へ取り込みました（{path.name}、種類 {kind}）")
            if kind == CSV_KIND_DRIVE:
                for victim in (path, path.with_suffix(".yaml")):
                    with contextlib.suppress(OSError):
                        victim.unlink()
                self._say(job, f"取り込み済みの走行 CSV を消しました: {path.name}")

    # ── 停止・非常停止 ─────────────────────────────────────────────

    def _send_sigint(self, job: _Job) -> None:
        """非常停止・アプリ終了: ランナーへ SIGINT（停止の流れ → 本物の終了処理 → 終了）。猶予を超えたら SIGKILL。"""
        runner = job.runner
        runner.exiting = True
        if runner.proc.returncode is None:
            with contextlib.suppress(ProcessLookupError):
                runner.proc.send_signal(signal.SIGINT)
        if job.kill_task is None or job.kill_task.done():
            job.kill_task = asyncio.create_task(self._kill_after(job, runner_exit=True))

    def _send_stop(self, job: _Job) -> None:
        """「走行終了」: コマンド stop（対話ジョブは quit_interactive）。ランナーは残り、待機へ戻る。"""
        command = {"cmd": "quit_interactive" if job.status.spec.kind == KIND_INTERACTIVE else "stop"}
        if not self._write_nowait(job.runner, command):
            self._say(job, "ランナーへ停止のコマンドを送れませんでした（もう終了しています）", "WARNING")
            return
        if job.kill_task is None or job.kill_task.done():
            job.kill_task = asyncio.create_task(self._kill_after(job, runner_exit=False))

    async def _kill_after(self, job: _Job, *, runner_exit: bool) -> None:
        """猶予（kill_timeout_s）で終わらなければ SIGKILL してメッセージに残す。

        SIGINT（非常停止）はランナーのプロセスの終了、stop コマンドはジョブの result を待つ。"""
        ev = job.runner.exited if runner_exit else job.result_event
        try:
            await asyncio.wait_for(ev.wait(), self.kill_timeout_s)
        except TimeoutError:
            self._say(
                job,
                f"ランナーが {self.kill_timeout_s:g} 秒で終わらないため強制終了（SIGKILL）しました。"
                "停止の流れ・終了処理（原点復帰）が済んでいない可能性があります。ペダルと車の状態（IGN・レンジ）を確認してください",
                "ERROR",
            )
            with contextlib.suppress(ProcessLookupError):
                job.runner.proc.kill()

    async def _kill_runner_after(self, runner: _Runner) -> None:
        """ジョブが無いときの SIGINT・quit の後、猶予で終わらなければ SIGKILL。"""
        try:
            await asyncio.wait_for(runner.exited.wait(), self.kill_timeout_s)
        except TimeoutError:
            self._say_idle(f"ランナーが {self.kill_timeout_s:g} 秒で終わらないため強制終了（SIGKILL）しました", "ERROR")
            with contextlib.suppress(ProcessLookupError):
                runner.proc.kill()

    async def stop(self) -> JobStatus | None:
        """「走行終了」（コマンド stop。ランナーは終了処理 = 停止の流れまで済ませて待機へ戻る）。
        実行中のジョブが無ければ JobConflict。非常停止の状態には入らない。

        シーケンスの実行中は、走っている段を止め、残りの段は走らせない（段の切れ目なら残りを走らせない）。
        """
        job = self._job
        seq = self._sequence
        if seq is not None:
            seq.stop_requested = True
        if job is None:
            if seq is None:
                raise JobConflict("実行中のジョブがありません")
            return None  # 段の切れ目: 次の段は始めない
        job.status.stop_requested = True
        if job.status.emergency:
            # 非常停止の流れが走行中。stop を重ねても意味がないので送らない
            self._say(job, "非常停止の流れが走っているため停止のコマンドは送りません。終了処理（原点復帰）を待ちます")
            return job.status
        self._say(job, "停止を要求しました（stop コマンド）。停止の流れ（走行するジョブは減速して停車 → N → IGN-OFF）と終了処理（原点復帰）を待ちます")
        self._send_stop(job)
        return job.status

    def _pausable_job(self) -> _Job:
        job = self._job
        if job is None or job.status.spec.kind != KIND_MODE_DRIVE or job.proc.returncode is not None:
            raise JobConflict("一時停止・再開できるのはモード走行の実行中だけです")
        return job

    async def pause(self) -> JobStatus:
        """一時停止（ランナーへ SIGUSR1）。モード走行以外・既に一時停止中は JobConflict（409）。"""
        job = self._pausable_job()
        if job.status.paused:
            raise JobConflict("すでに一時停止中です")
        with contextlib.suppress(ProcessLookupError):
            job.proc.send_signal(signal.SIGUSR1)
        job.status.paused = True
        self._say(job, "一時停止を要求しました（SIGUSR1）")
        return job.status

    async def resume(self) -> JobStatus:
        """再開（ランナーへ SIGUSR2）。モード走行以外・一時停止中でなければ JobConflict（409）。"""
        job = self._pausable_job()
        if not job.status.paused:
            raise JobConflict("一時停止中ではありません")
        with contextlib.suppress(ProcessLookupError):
            job.proc.send_signal(signal.SIGUSR2)
        job.status.paused = False
        self._say(job, "再開を要求しました（SIGUSR2）")
        return job.status

    def _enter_emergency(self, cause: str = "screen", *, detail: str | None = None) -> None:
        """非常停止の状態へ入る共通部分（画面・スイッチの非常停止と、ランナーの自動の非常停止で共有）。
        ジョブの emergency / stop_requested は呼び出し側が立てる（ここは manager 側の状態だけ）。"""
        if self._sequence is not None:
            self._sequence.stop_requested = True  # 残りの段は走らせない
        self._initialized = False  # 非常停止の後は初期化し直す（D-15c）
        if self._state != STATE_EMERGENCY:
            self._emergency_cause = cause  # 最初のきっかけを残す（二重に来ても上書きしない）
            self._emergency_cause_detail = detail or None
            self._record_emergency(cause, detail)  # エラーページの 1 件（ジョブの終わりに終了処理の結果を書き足す）
        self._state = STATE_EMERGENCY

    async def emergency(self, cause: str = "screen") -> None:
        """非常停止。ランナーへ SIGINT（停止の流れ → 本物の終了処理 → 終了）。解除まで次のジョブを受け付けない。
        ジョブが無くても常駐ランナーが生きていれば SIGINT を送る（終了後は初期化し直し）。
        cause: きっかけ（screen / switch / ac_loss。画面の表示用）。"""
        job = self._job
        runner = self._live_runner()
        self._enter_emergency(cause)
        await self.cancel_prepare()  # 準備中なら計画を捨てる
        if job is None:
            if runner is None:
                logger.warning("非常停止（ジョブなし）。解除するまで次のジョブを受け付けません", extra=RECORDED)
                return
            logger.warning("非常停止（待機中）。常駐ランナーへ SIGINT を送ります（終了処理 → 終了）", extra=RECORDED)
            runner.exiting = True
            with contextlib.suppress(ProcessLookupError):
                runner.proc.send_signal(signal.SIGINT)
            if runner.kill_task is None or runner.kill_task.done():
                runner.kill_task = asyncio.create_task(self._kill_runner_after(runner))
            return
        if job.status.emergency:
            return
        job.status.emergency = True
        job.status.stop_requested = True
        self._say(job, "非常停止: ランナーへ SIGINT を送りました（走行するジョブは減速して停車 → N → IGN-OFF → 原点復帰 → 終了処理）", "WARNING")
        self._send_sigint(job)

    async def reset_emergency(self) -> None:
        """非常停止を解除して待機へ。ランナーの終了処理中・物理スイッチが押されたままなら解除しない。"""
        if self._state != STATE_EMERGENCY:
            return
        runner = self._live_runner()
        if self._job is not None or self._sequence is not None or (runner is not None and runner.exiting):
            raise JobConflict("ランナーが終了処理中です。終わるまで解除できません")
        if self._emergency_probe is not None and self._emergency_probe():
            raise EmergencyStillActive(
                "非常停止スイッチがまだ押されています。スイッチを戻してから解除してください"
            )
        self._state = STATE_STANDBY
        self._emergency_cause = None
        self._emergency_cause_detail = None
        if self._emergency_open:
            self._emergency_open = False
            self._errors.append_emergency("非常停止を解除しました", close=True)
        logger.info("非常停止を解除しました")

    async def wait_idle(self, timeout: float | None = None) -> None:
        job = self._job
        if job is not None:
            await asyncio.wait_for(job.done.wait(), timeout)
        await asyncio.wait_for(self.wait_persisted(), timeout)

    async def shutdown(self) -> None:
        """アプリ終了時（A-D2）: 常駐ランナーへ quit（ジョブ実行中ならランナーがそのジョブを止めてから）→
        終わるまで待つ（猶予を超えたら SIGKILL）。"""
        self._shutting_down = True
        await self.cancel_prepare()
        job = self._job
        seq = self._sequence
        runner = self._live_runner()
        if seq is not None:
            seq.stop_requested = True
        if job is not None:
            job.status.stop_requested = True
            self._say(job, "アプリ終了のため常駐ランナーを終了します（quit）")
        if runner is not None:
            runner.exiting = True
            self._write_nowait(runner, {"cmd": "quit"})
            if runner.kill_task is None or runner.kill_task.done():
                runner.kill_task = asyncio.create_task(self._kill_runner_after(runner))
            with contextlib.suppress(Exception):
                await asyncio.wait_for(runner.exited.wait(), self.kill_timeout_s + 15.0)
            if runner.task is not None:
                with contextlib.suppress(Exception, asyncio.CancelledError):
                    await asyncio.wait_for(asyncio.shield(runner.task), 5.0)
        if job is not None:
            with contextlib.suppress(Exception):
                await asyncio.wait_for(job.done.wait(), 15.0)
        with contextlib.suppress(Exception):
            await asyncio.wait_for(self.wait_persisted(), 30.0)
        if seq is not None and seq.task is not None:
            with contextlib.suppress(Exception):
                await asyncio.wait_for(asyncio.shield(seq.task), 10.0)

    # ── 対話ジョブ ─────────────────────────────────────────────────

    async def ensure_interactive(self, vehicle_id: str, purpose: str) -> JobStatus:
        """対話ジョブがコマンド待ちになっていることを保証する。無ければ始めて待つ。

        同じ車の対話ジョブが既に走っていれば使い回す（手動運転とキャリブレーションは同じ入口）。
        別の種類のジョブ・別の車の対話ジョブが走っていれば JobConflict。
        """
        async with self._start_lock:
            cur = self._job
            if cur is not None:
                s = cur.status
                if s.spec.kind != KIND_INTERACTIVE or s.spec.vehicle_id != vehicle_id:
                    self._check_can_start()  # 409 の理由を出す
                if s.spec.purpose != purpose:
                    names = {PURPOSE_MANUAL: "手動運転", PURPOSE_CALIB: "キャリブレーション"}
                    raise JobConflict(
                        f"{names.get(s.spec.purpose, '対話')}の実行中です。終えてから始めてください"
                    )
                if s.stop_requested:
                    raise JobConflict("対話ジョブを終了処理中です。終わってからやり直してください")
        if self._job is None:
            await self.start(self.interactive_spec(vehicle_id, purpose))
        job = self._job
        if job is None:
            raise JobConflict("対話ジョブが終了しました（初期化に失敗した可能性があります）")
        if not job.status.interactive_ready:
            try:
                await asyncio.wait_for(job.ready.wait(), INTERACTIVE_READY_TIMEOUT_S)
            except TimeoutError as exc:
                raise JobUnavailable("初期化が終わりません（対話ジョブがコマンド待ちになりません）") from exc
        if not job.status.interactive_ready:
            code = job.status.exit_code
            raise JobConflict(f"初期化に失敗しました（終了コード {code}）。進行のメッセージを確認してください")
        return job.status

    async def send_button(
        self, button: str, action: str, *, timeout: float = COMMAND_TIMEOUT_S
    ) -> dict[str, Any]:
        """手動運転のスイッチ（スタート・P・N・D）の down / up をランナーへ送り、ack を返す。

        ペダルのジョグ（``send_command``）の順番待ち（cmd_lock）を通らない。ランナーは button を
        キューに入れずその場で処理するので、ジョグの最中でも即座に返る。"""
        job = self._job
        if (
            job is None
            or job.status.spec.kind != KIND_INTERACTIVE
            or job.status.spec.purpose != PURPOSE_MANUAL
            or not job.status.interactive_ready
        ):
            raise JobConflict("スイッチを操作できる手動運転がありません")
        stdin = job.proc.stdin
        if stdin is None or job.proc.returncode is not None:
            raise JobConflict("手動運転は終了しています")
        cmd_id = job.next_cmd_id
        job.next_cmd_id += 1
        loop = asyncio.get_running_loop()
        pending = _Pending(ack=loop.create_future(), status=loop.create_future())
        job.pending[cmd_id] = pending
        try:
            async with job.write_lock:
                line = {"cmd": "button", "button": button, "action": action, "id": cmd_id}
                stdin.write((json.dumps(line, ensure_ascii=False) + "\n").encode())
                await stdin.drain()
            try:
                return await asyncio.wait_for(pending.ack, timeout)
            except TimeoutError as exc:
                raise JobUnavailable("スイッチ操作の応答がありません") from exc
            except asyncio.CancelledError:
                raise JobConflict("手動運転が終了しました") from None
        finally:
            job.pending.pop(cmd_id, None)

    async def send_command(
        self, command: dict[str, Any], *, timeout: float = COMMAND_TIMEOUT_S
    ) -> InteractiveReply:
        """対話ジョブへコマンド（README の JSON）を送り、ack（と status）を返す。

        ``quit`` は対話ジョブの終わり（``quit_interactive``。ランナーは終わらず待機へ戻る）。"""
        if command.get("cmd") == "quit":
            command = {**command, "cmd": "quit_interactive"}
        job = self._job
        if job is None or job.status.spec.kind != KIND_INTERACTIVE or not job.status.interactive_ready:
            raise JobConflict("コマンドを受け付ける対話ジョブがありません")
        stdin = job.proc.stdin
        if stdin is None or job.proc.returncode is not None:
            raise JobConflict("対話ジョブは終了しています")
        async with job.cmd_lock:
            cmd_id = job.next_cmd_id
            job.next_cmd_id += 1
            loop = asyncio.get_running_loop()
            pending = _Pending(ack=loop.create_future(), status=loop.create_future())
            job.pending[cmd_id] = pending
            try:
                async with job.write_lock:
                    stdin.write((json.dumps({**command, "id": cmd_id}, ensure_ascii=False) + "\n").encode())
                    await stdin.drain()
                try:
                    ack = await asyncio.wait_for(pending.ack, timeout)
                except TimeoutError as exc:
                    raise JobUnavailable("対話コマンドの応答がありません") from exc
                except asyncio.CancelledError:
                    raise JobConflict("対話ジョブが終了しました") from None
                st: dict[str, Any] | None = None
                if ack.get("ok") and command.get("cmd") != "quit_interactive":
                    with contextlib.suppress(TimeoutError, asyncio.CancelledError):
                        st = await asyncio.wait_for(pending.status, STATUS_TIMEOUT_S)
                if ack.get("ok") and command.get("cmd") == "save_min" and self._on_min_saved:
                    self._on_min_saved(job.status.spec.vehicle_id, str(command.get("axis")))
                return InteractiveReply(ack=ack, status=st)
            finally:
                job.pending.pop(cmd_id, None)
