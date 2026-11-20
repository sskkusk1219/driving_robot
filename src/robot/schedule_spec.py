# ruff: noqa: E501
"""タイムスケジュール（開ループ再生）の純粋な部分: データ・補間・ペダルの決め方・ボタンの検査。

RPD_Phase_20261115_PortResearch.md §3 段6。core を import しない（Web 側からも使えるように軽くしてある）。
再生そのもの（HW を動かす所）は ``src/robot/schedule_job.py``。考え方は旧 ``src/domain/control/schedule_loop.py``
（時刻でタイムラインを引き、ペダル開度を線形補間して指令、ボタンを指定時刻に押す）から移した。

ボタンは ch ではなくボタン名（``start`` / ``P`` / ``N`` / ``D``）で指す。名前のないイベント（旧データ。
ch だけを持つ）は推測で押さず、実行前にエラーにする。
"""

from __future__ import annotations

import bisect
import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

#: ボタンの名前（core の button_servo.BUTTON_KEYS と同じ。ここは core を import しないので写してある）
BUTTON_KEYS: tuple[str, ...] = ("start", "P", "N", "D")
BUTTON_LABELS: dict[str, str] = {"start": "スタート", "P": "P", "N": "N", "D": "D"}

#: 押す時間が無いイベントの総時間の見積もりに使う秒数（車両 yaml の既定秒数は実行時に使う）
ESTIMATE_PRESS_S = 1.2


class ScheduleError(Exception):
    """スケジュールを走らせられない（日本語の理由つき）。"""


@dataclass(frozen=True)
class PedalPoint:
    time_s: float
    accel_pct: float
    brake_pct: float


@dataclass(frozen=True)
class ButtonPress:
    time_s: float
    button: str | None  # None ＝ 名前なし（旧データ。ch だけ持つ）
    press_duration_s: float | None  # None ＝ 車両 yaml の既定秒数
    legacy_channel: int | None = None


@dataclass(frozen=True)
class ScheduleData:
    id: str
    name: str
    points: tuple[PedalPoint, ...]
    buttons: tuple[ButtonPress, ...]  # time_s 昇順

    @property
    def total_duration(self) -> float:
        pedal_end = self.points[-1].time_s if self.points else 0.0
        button_end = max(
            (b.time_s + (b.press_duration_s or ESTIMATE_PRESS_S) for b in self.buttons), default=0.0
        )
        return max(pedal_end, button_end)


# ── 読み書き ───────────────────────────────────────────────────────


def schedule_from_dict(data: Mapping[str, Any]) -> ScheduleData:
    """JSON の辞書（``schedule_to_dict`` の形）から作る。形の誤りは ScheduleError。"""
    try:
        points = tuple(
            PedalPoint(
                time_s=float(p["time_s"]),
                accel_pct=float(p["accel_opening"]),
                brake_pct=float(p["brake_opening"]),
            )
            for p in data.get("pedal_points", [])
        )
        buttons = tuple(
            sorted(
                (
                    ButtonPress(
                        time_s=float(e["time_s"]),
                        button=e.get("button") or None,
                        press_duration_s=(
                            None if e.get("press_duration_s") is None else float(e["press_duration_s"])
                        ),
                        legacy_channel=None if e.get("channel") is None else int(e["channel"]),
                    )
                    for e in data.get("button_events", [])
                ),
                key=lambda b: b.time_s,
            )
        )
        return ScheduleData(
            id=str(data.get("id", "")), name=str(data.get("name", "")), points=points, buttons=buttons
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ScheduleError(f"スケジュールの形が正しくありません（{type(exc).__name__}: {exc}）") from exc


def schedule_to_dict(schedule: Any) -> dict[str, Any]:
    """``TimeSchedule``（src/models/time_schedule.py）→ ランナーへ渡す JSON の辞書。"""
    return {
        "id": schedule.id,
        "name": schedule.name,
        "pedal_points": [
            {
                "time_s": p.time_s,
                "accel_opening": p.accel_opening,
                "brake_opening": p.brake_opening,
            }
            for p in schedule.pedal_points
        ],
        "button_events": [
            {
                "time_s": e.time_s,
                "button": getattr(e, "button", None),
                "channel": getattr(e, "channel", None),
                "press_duration_s": getattr(e, "press_duration_s", None),
            }
            for e in schedule.button_events
        ],
    }


def load_schedule_file(path: Path) -> ScheduleData:
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ScheduleError(f"スケジュールのファイルを読めません: {path}（{exc}）") from exc
    return schedule_from_dict(raw)


# ── 検査 ───────────────────────────────────────────────────────────


def check_schedule(schedule: ScheduleData) -> str | None:
    """走らせられない理由を日本語で返す（問題なければ None）。HW に触る前に止めるための検査。"""
    if not schedule.points and not schedule.buttons:
        return f"スケジュール「{schedule.name}」にペダル開度の点もボタンイベントもありません"
    for a, b in zip(schedule.points, schedule.points[1:], strict=False):
        if b.time_s <= a.time_s:
            return f"スケジュール「{schedule.name}」のペダル開度の時刻が増えていません（{a.time_s:g}s → {b.time_s:g}s）"
    unnamed = [b for b in schedule.buttons if b.button is None]
    if unnamed:
        times = "・".join(f"{b.time_s:g}s" for b in unnamed[:5])
        more = "" if len(unnamed) <= 5 else f" ほか{len(unnamed) - 5}件"
        return (
            f"スケジュール「{schedule.name}」にボタン名のないイベントが {len(unnamed)} 件あります"
            f"（{times}{more}。旧データはチャンネルだけを持ち、どのボタンか分かりません）。"
            "スケジュール画面で各イベントのボタン（スタート・P・N・D）を選んで保存し直してください"
        )
    bad = [b for b in schedule.buttons if b.button not in BUTTON_KEYS]
    if bad:
        return (
            f"スケジュール「{schedule.name}」に知らないボタン名があります"
            f"（{', '.join(repr(b.button) for b in bad)}。使えるのは {', '.join(BUTTON_KEYS)}）"
        )
    return None


# ── 補間・ペダルの決め方 ──────────────────────────────────────────


class PedalTimeline:
    """ペダル開度の点を時刻で線形補間する。範囲外は端点で保持。点が無ければ (0, 0)。"""

    def __init__(self, points: Sequence[PedalPoint]) -> None:
        self._points = tuple(points)
        self._times = [p.time_s for p in self._points]

    def at(self, t: float) -> tuple[float, float]:
        pts = self._points
        if not pts:
            return 0.0, 0.0
        if t <= pts[0].time_s:
            return pts[0].accel_pct, pts[0].brake_pct
        if t >= pts[-1].time_s:
            return pts[-1].accel_pct, pts[-1].brake_pct
        i = bisect.bisect_right(self._times, t) - 1
        p0, p1 = pts[i], pts[i + 1]
        span = p1.time_s - p0.time_s
        if span <= 0.0:
            return p1.accel_pct, p1.brake_pct
        frac = (t - p0.time_s) / span
        return (
            p0.accel_pct + (p1.accel_pct - p0.accel_pct) * frac,
            p0.brake_pct + (p1.brake_pct - p0.brake_pct) * frac,
        )


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def pedal_command(
    accel: float,
    brake: float,
    *,
    min_accel: float,
    min_brake: float,
    max_accel: float,
    max_brake: float,
) -> tuple[float, float]:
    """スケジュールの開度 → 実際に指令する開度 [%]。

    1. 各ペダルを 0〜max（車両の ``max_*_opening_pct``）に頭打ち。
    2. 使っていない（min 以下の）ペダルは待機位置の min（core のモード走行と同じ。min 未満は効かない）。

    タイムスケジュールはアクセルとブレーキの同時踏みを許す（2026-11-19 ユーザー決定）ので、
    片足ルール・実開度の安全網は掛けず、両ペダルを独立に扱う。
    """
    a = _clamp(accel, 0.0, max_accel)
    b = _clamp(brake, 0.0, max_brake)
    return max(a, min_accel), max(b, min_brake)


def button_seconds(
    press_duration_s: float | None, button: str, *, start_short_press_s: float, shift_press_s: float
) -> float:
    """ボタンを押す秒数。イベントに無ければ車両 yaml の既定（スタートは短押し・P/N/D は shift の秒数）。"""
    if press_duration_s is not None and press_duration_s > 0.0:
        return press_duration_s
    return start_short_press_s if button == "start" else shift_press_s
