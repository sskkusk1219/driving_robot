# ruff: noqa: E501
"""走行中のボタン押下（スタート・P・N・D）を、進行のメッセージ（drive_session_messages）から読む（段11）。

保存の形（今後のセッション）: ボタンを押し始めるたびに JobManager が level ``BUTTON`` の行を 1 本足す。
message は ``{"button":"N","seconds":1.2}``（JSON）、time は押し始めの壁時計。スキーマは変えない。
古いセッションには BUTTON の行が無いので、ランナーが出す次の 2 種類の文から読み直す（time は押し始め）。

- ボタンなし: ``  ボタンなし: 「N」を押したことにします（1.2s 待ちます）``
- ボタンあり: ``  ボタン「D」を ch1 175° で 1.2s 押します …``

BUTTON の行が 1 本でもあれば、それだけを使う（同じ押下の通常の文との二重数えを避ける）。
"""

from __future__ import annotations

import json
import re
from collections.abc import Iterable
from datetime import datetime
from typing import Any

LEVEL_BUTTON = "BUTTON"

#: 表示名（core の BUTTON_NAMES）→ ボタンのキー（start/P/N/D）
_NAME_TO_KEY = {"スタート": "start", "P": "P", "N": "N", "D": "D"}

_RE_OFF = re.compile(r"ボタンなし: 「(?P<name>[^」]+)」を押したことにします（(?P<sec>[0-9.]+)s 待ちます）")
_RE_ON = re.compile(r"ボタン「(?P<name>[^」]+)」を ch\d+ [0-9.]+° で (?P<sec>[0-9.]+)s 押します")


def button_message(button: str, seconds: float) -> str:
    """BUTTON の行の message（機械可読の JSON）。"""
    return json.dumps({"button": button, "seconds": round(float(seconds), 3)}, ensure_ascii=False)


def parse_button_text(text: str) -> tuple[str, float] | None:
    """旧形式の 1 行（ボタンなし／ボタンあり）から (ボタンのキー, 押す秒数)。該当しなければ None。"""
    m = _RE_OFF.search(text) or _RE_ON.search(text)
    if m is None:
        return None
    name = m.group("name")
    return _NAME_TO_KEY.get(name, name), float(m.group("sec"))


def _parse_json_row(text: str) -> tuple[str, float] | None:
    try:
        d = json.loads(text)
        return str(d["button"]), float(d["seconds"])
    except (ValueError, KeyError, TypeError):
        return None


def parse_button_presses(messages: Iterable[Any]) -> list[dict[str, Any]]:
    """メッセージ行（level・message・time を持つ）から ``[{button, seconds, time}]``（time 昇順）。"""
    msgs = list(messages)
    structured = [m for m in msgs if m.level == LEVEL_BUTTON]
    out: list[dict[str, Any]] = []
    if structured:
        for m in structured:
            parsed = _parse_json_row(m.message)
            if parsed is not None:
                out.append({"button": parsed[0], "seconds": parsed[1], "time": m.time})
    else:
        for m in msgs:
            parsed = parse_button_text(m.message)
            if parsed is not None:
                out.append({"button": parsed[0], "seconds": parsed[1], "time": m.time})
    out.sort(key=lambda e: e["time"])
    return out


def presses_to_intervals(
    presses: Iterable[dict[str, Any]], times: list[datetime], xs: list[float]
) -> list[dict[str, Any]]:
    """押下の壁時計を、ログの時刻 ``times``（昇順）と図の x ``xs`` の対応で x の区間にする。

    ログの範囲の外は傾き 1（壁時計 1 秒 = x 1 秒）で延ばす。返す ``start_s``・``end_s`` は x の値。
    """
    if len(times) < 2 or len(times) != len(xs):
        return []
    import numpy as np  # noqa: PLC0415

    ts = np.array([t.timestamp() for t in times])
    xa = np.array(xs)

    def to_x(wall: float) -> float:
        if wall <= ts[0]:
            return float(xa[0] - (ts[0] - wall))
        if wall >= ts[-1]:
            return float(xa[-1] + (wall - ts[-1]))
        return float(np.interp(wall, ts, xa))

    out = []
    for p in presses:
        w = p["time"].timestamp()
        start = to_x(w)
        out.append(
            {
                "button": p["button"],
                "seconds": p["seconds"],
                "start_s": round(start, 2),
                "end_s": round(start + float(p["seconds"]), 2),
            }
        )
    return out
