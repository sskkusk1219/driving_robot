# ruff: noqa: E501
"""ランナーが Web 側へ知らせる「機械が読むイベント」（JSON 1 行）の出し口と形式。

RPD_Phase_20261115_PortResearch.md §3 段2。標準出力には core の say() の行がそのまま流れる
（Web 側は「現在ステータス」にそのまま出す）。それとは別に、状態・初期化の項目・ライブ値・CSV・結果を
JSON 1 行で出す。既定は標準出力に ``@@EV `` を前置きした行、``--events-fd N`` があればその fd
（行の前置きなし）。種類と項目は src/robot/README.md の表。

ここは純粋な部分（行の組み立て・解析）と、書き出しの口（EventSink）だけで、core を import しない。
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
from typing import Any, TextIO

#: 標準出力に出すときの行頭の印
EVENT_PREFIX = "@@EV "

#: イベントの種類（README の表と同じ）
EVENT_TYPES: tuple[str, ...] = (
    "state",
    "init_step",
    "sample",
    "csv",
    "result",
    "ack",
    "status",
    "button",  # ボタン（スタート・P・N・D）の押し始め・離し（phase=start/end・button・seconds。段11）
    "watch",  # 常駐ランナー（--serve）の待機中の見張りの結果（level=warn/error・detail。A-D3）
)

#: state の phase の値
STATE_PHASES: tuple[str, ...] = (
    "init",  # 手順 1（初期化）の開始
    "planning",  # 計画だけを作るジョブ（--plan-only）の開始
    "planned",  # 計画を書き終えた
    "prepare",  # 手順の準備（走行モード・FF モデルの読み込み）
    "pre_check",  # 走行前チェック
    "step",  # 手順の開始（step に番号）
    "step_end",  # 手順の終わり（ok=true/false）
    "ready",  # 学習の手順の前: ブレーキを踏んでスタート短押し（Ready。D-14b）
    "d",  # 学習の手順の前: D を押す（Ready から d_press_at_s 後）
    "n",  # 学習の手順の後: ブレーキ ON で N を押す
    "shutdown",  # 終了処理（原点復帰・サーボOFF・切断）
    "done",  # 完了（この後に result）
    "interactive",  # 対話ジョブがコマンド待ちになった
    "idle",  # 常駐ランナー（--serve）が次のコマンドを待つ（needs_init=true なら次は init が要る）
    "job_end",  # 常駐ランナー: ジョブの終わり（mode=home: 原点復帰だけ / keep: 何もしない。サーボ ON・接続のまま）
    "emergency",  # 非常停止の流れ（stage=start/decel/n_press/release/ign_off/home/done/brake_kept/buttons_unavailable/can_lost/brake_fault/accel_fault/error・detail）
)

#: 終了コードの意味（core の run_steps と同じ。130/143 だけランナーが足す）
EXIT_MEANINGS: dict[int, str] = {
    0: "成功",
    2: "設定エラー",
    3: "未実装の手順に到達",
    4: "初期化エラー",
    5: "走行エラー",
    6: "モデル作成エラー",
    7: "走行前チェックエラー",
    8: "手順2 の学習結果のチェックが NG（yaml は書き換えていない）",
    130: "中断（SIGINT）。終了処理は済ませた",
    143: "中断（SIGTERM）。終了処理は済ませた",
}


def exit_meaning(code: int) -> str:
    return EXIT_MEANINGS.get(code, f"不明な終了コード {code}")


def build_event(type_: str, **fields: Any) -> dict[str, Any]:
    """イベントの辞書を作る（``type`` と、壁時計の ``ts`` [epoch 秒] を付ける）。"""
    if type_ not in EVENT_TYPES:
        raise ValueError(f"未知のイベント種類: {type_!r}")
    return {"type": type_, "ts": round(time.time(), 3), **fields}


def encode_event(event: dict[str, Any], *, prefix: str = EVENT_PREFIX) -> str:
    """イベントを 1 行（改行なし）にする。"""
    return prefix + json.dumps(event, ensure_ascii=False, separators=(",", ":"))


def decode_event_line(line: str, *, prefix: str = EVENT_PREFIX) -> dict[str, Any] | None:
    """標準出力の 1 行がイベントなら辞書を、そうでなければ None を返す（Web 側・テスト用）。

    fd 経由（前置きなし）の行は ``prefix=""`` で読む。
    """
    text = line.rstrip("\r\n")
    if prefix:
        if not text.startswith(prefix):
            return None
        text = text[len(prefix) :]
    try:
        data = json.loads(text)
    except json.JSONDecodeError:
        return None
    if not isinstance(data, dict) or data.get("type") not in EVENT_TYPES:
        return None
    return data


class EventSink:
    """イベントを書き出す口。スレッドから呼ばれても 1 行が混ざらないよう、1 回の write で出す。"""

    def __init__(self, *, fd: int | None = None, stream: TextIO | None = None) -> None:
        self._lock = threading.Lock()
        if fd is not None:
            self._stream: TextIO = os.fdopen(fd, "w", buffering=1, encoding="utf-8", closefd=False)
            self._prefix = ""
        else:
            self._stream = stream if stream is not None else sys.stdout
            self._prefix = EVENT_PREFIX

    def emit(self, type_: str, **fields: Any) -> None:
        line = encode_event(build_event(type_, **fields), prefix=self._prefix) + "\n"
        with self._lock:
            try:
                self._stream.write(line)
                self._stream.flush()
            except (OSError, ValueError):
                pass  # 読む側が閉じた。走行は止めない


_sink: EventSink | None = None


def set_sink(sink: EventSink | None) -> None:
    global _sink  # noqa: PLW0603
    _sink = sink


_job_id: str | None = None


def set_job_id(job_id: str | None) -> None:
    """常駐ランナー（--serve）が、今のジョブの ID を持たせる。以降の ``emit`` は ``job_id`` を自動で付ける。"""
    global _job_id  # noqa: PLW0603
    _job_id = job_id


def current_job_id() -> str | None:
    return _job_id


def emit(type_: str, **fields: Any) -> None:
    """設定済みの口へイベントを出す。口が無ければ（テストなど）何もしない。

    今のジョブ ID（``set_job_id``）があれば ``job_id`` を付ける（呼び出し側が ``job_id`` を渡していればそちらを優先）。
    """
    if _sink is not None:
        if _job_id is not None and "job_id" not in fields:
            fields = {"job_id": _job_id, **fields}
        _sink.emit(type_, **fields)
