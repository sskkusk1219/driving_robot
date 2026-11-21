# ruff: noqa: E501
"""研究ハーネス形式の走行ログ CSV（約 100 列）と DB の drive_logs の行の相互変換（純粋関数）。

RPD_Phase_20261115_PortResearch.md §3 段3。ランナーが保存する CSV（src/robot/core/drive_log.py の
``CSV_COLUMNS``）を DB へ入れるとき、今の drive_logs の 9 列に入る列は専用の列へ、残りの全列は
``detail JSONB``（キー = CSV の列名・値 = CSV のセルの文字列そのまま。空欄は入れない）へ入れる。
ログ画面の CSV ダウンロード（``/sessions/{id}/logs.csv``）は逆向きに、``CSV_COLUMNS`` の順で全列を書き出す。
tests/research の CSV と同じ列なので、解析の道具（research-run-analysis）がそのまま読める。

9 列と CSV の列の対応（この対応は 1 か所。ここを変えたら ``CSV_TO_DB`` と ``db_to_csv_cells`` を直す）:

| drive_logs の列 | CSV の列 | 備考 |
|---|---|---|
| timestamp | timestamp | CSV は JST の naive 文字列。DB は UTC aware（JST − 9 h） |
| ref_speed_kmh | ref_speed_kmh | 空欄は NULL |
| actual_speed_kmh | actual_speed_kmh | |
| accel_opening / brake_opening | accel_cmd_pct / brake_cmd_pct | **指令**開度 [%]（今の DB の意味と同じ。実開度は detail の accel_actual_pct） |
| accel_pos / brake_pos | accel_cmd_mm / brake_cmd_mm | 指令位置 [pulse] ＝ mm × 100（CSV は mm、DB は pulse） |
| accel_current / brake_current | accel_current / brake_current | [mA] |

NOT NULL の列が CSV で空欄だった行（実車速が無いなど）は 0 にする（行は捨てない）。
"""

from __future__ import annotations

import csv
import json
from collections.abc import Iterable, Iterator, Mapping, Sequence
from datetime import UTC, datetime, timedelta, timezone
from pathlib import Path
from typing import Any

_JST = timezone(timedelta(hours=9))
_PULSE_PER_MM = 100

#: ボタンの列（ボタンのキー → 列名）。押している間 1・他 0
BUTTON_COLUMNS: dict[str, str] = {
    "start": "btn_start",
    "P": "btn_P",
    "N": "btn_N",
    "D": "btn_D",
}
#: エラーの列（level は ERROR 優先・title は " / " でつなぐ）
ERROR_COLUMNS: tuple[str, ...] = ("error_level", "error_title")
#: ``csv_header`` の最後に付く列（ボタン 4 列＋エラー 2 列）
EXTRA_COLUMNS: tuple[str, ...] = (*BUTTON_COLUMNS.values(), *ERROR_COLUMNS)

#: CSV の列名 → drive_logs の専用列に入る（この列は detail に入れない）
CSV_TO_DB: dict[str, str] = {
    "timestamp": "timestamp",
    "ref_speed_kmh": "ref_speed_kmh",
    "actual_speed_kmh": "actual_speed_kmh",
    "accel_cmd_pct": "accel_opening",
    "brake_cmd_pct": "brake_opening",
    "accel_cmd_mm": "accel_pos",
    "brake_cmd_mm": "brake_pos",
    "accel_current": "accel_current",
    "brake_current": "brake_current",
}

#: drive_logs へ入れる列の順（copy_records_to_table の columns と行の並び）
DB_COLUMNS: tuple[str, ...] = (
    "session_id",
    "timestamp",
    "ref_speed_kmh",
    "actual_speed_kmh",
    "accel_opening",
    "brake_opening",
    "accel_pos",
    "brake_pos",
    "accel_current",
    "brake_current",
    "detail",
)


def csv_columns() -> tuple[str, ...]:
    """core の CSV_COLUMNS（列順の唯一の定義）。重い import なので初回の呼び出しで読み込む。"""
    from src.robot.core.drive_log import CSV_COLUMNS  # noqa: PLC0415

    return tuple(CSV_COLUMNS)


def parse_csv_timestamp(text: str) -> datetime:
    """CSV の timestamp（JST の naive な ISO 文字列）→ UTC aware の datetime。"""
    return datetime.fromisoformat(text.strip()).replace(tzinfo=_JST).astimezone(UTC)


def _float_or_none(text: str | None) -> float | None:
    if text is None or text.strip() == "":
        return None
    try:
        return float(text)
    except ValueError:
        return None


def csv_row_to_db(
    session_id: str, row: Mapping[str, str], *, detail_as_json: bool = True
) -> tuple[Any, ...]:
    """CSV の 1 行（列名 → 文字列）→ ``DB_COLUMNS`` の並びのタプル。

    detail は空欄でない残りの全列（文字列のまま）。``detail_as_json`` が真なら JSON 文字列にする
    （asyncpg の jsonb に渡す形）。
    """
    detail = {
        key: value
        for key, value in row.items()
        if key is not None and key not in CSV_TO_DB and value is not None and value != ""
    }

    def num(column: str) -> float:
        value = _float_or_none(row.get(column))
        return 0.0 if value is None else value

    def pulse(column: str) -> int:
        value = _float_or_none(row.get(column))
        return 0 if value is None else round(value * _PULSE_PER_MM)

    return (
        session_id,
        parse_csv_timestamp(row["timestamp"]),
        _float_or_none(row.get("ref_speed_kmh")),
        num("actual_speed_kmh"),
        num("accel_cmd_pct"),
        num("brake_cmd_pct"),
        pulse("accel_cmd_mm"),
        pulse("brake_cmd_mm"),
        num("accel_current"),
        num("brake_current"),
        json.dumps(detail, ensure_ascii=False, separators=(",", ":")) if detail_as_json else detail,
    )


def read_csv_rows(path: Path) -> Iterator[dict[str, str]]:
    """CSV を 1 行ずつ辞書で返す（数万行でもメモリに全部は持たない）。"""
    with path.open(newline="", encoding="utf-8") as f:
        yield from csv.DictReader(f)


def csv_to_db_rows(
    session_id: str, rows: Iterable[Mapping[str, str]]
) -> Iterator[tuple[Any, ...]]:
    """CSV の行の並び → DB の行の並び。timestamp が読めない行は捨てる（壊れた末尾行など）。"""
    for row in rows:
        try:
            yield csv_row_to_db(session_id, row)
        except (KeyError, ValueError):
            continue


def _fmt(value: float | None) -> str:
    return "" if value is None else repr(value)


def db_to_csv_cells(
    *,
    timestamp: datetime,
    ref_speed_kmh: float | None,
    actual_speed_kmh: float,
    accel_opening: float,
    brake_opening: float,
    accel_pos: int,
    brake_pos: int,
    accel_current: float,
    brake_current: float,
    detail: Mapping[str, Any] | None,
    columns: Sequence[str],
) -> list[str]:
    """DB の 1 行 → CSV の 1 行（``columns`` の順）。detail に無い列は空欄。

    timestamp は tests の CSV と同じく JST の naive 文字列（``isoformat(sep=" ")``）。
    ``columns`` に無い detail のキー（core が列を足す前の CSV など）は無視せず、列の後ろへ回すのは
    ``csv_header`` の役目（ここでは columns の列だけ書く）。
    """
    jst_naive = timestamp.astimezone(_JST).replace(tzinfo=None)
    mapped: dict[str, str] = {
        "timestamp": jst_naive.isoformat(sep=" "),
        "ref_speed_kmh": _fmt(ref_speed_kmh),
        "actual_speed_kmh": _fmt(actual_speed_kmh),
        "accel_cmd_pct": _fmt(accel_opening),
        "brake_cmd_pct": _fmt(brake_opening),
        "accel_cmd_mm": f"{accel_pos / _PULSE_PER_MM:.2f}",
        "brake_cmd_mm": f"{brake_pos / _PULSE_PER_MM:.2f}",
        "accel_current": _fmt(accel_current),
        "brake_current": _fmt(brake_current),
    }
    detail = detail or {}
    return [mapped[c] if c in mapped else str(detail.get(c, "")) for c in columns]


def csv_header(detail_keys: Iterable[str]) -> list[str]:
    """CSV の見出し。core の列順に、detail にあって core に無い列（古い CSV の旧列など）を後ろへ足し、
    最後にボタン・エラーの列（``EXTRA_COLUMNS``）を付ける。"""
    base = list(csv_columns())
    known = set(base) | set(EXTRA_COLUMNS)
    extra = sorted(k for k in set(detail_keys) if k not in known)
    return base + extra + list(EXTRA_COLUMNS)


def decode_detail(value: Any) -> dict[str, Any]:
    """asyncpg が返す jsonb（str か dict か None）→ dict。"""
    if value is None:
        return {}
    if isinstance(value, str):
        try:
            loaded = json.loads(value)
        except json.JSONDecodeError:
            return {}
        return loaded if isinstance(loaded, dict) else {}
    return dict(value)


def format_csv_timestamp(timestamp: datetime) -> str:
    """UTC aware（naive は UTC とみなす）→ CSV の timestamp（JST の naive 文字列）。"""
    if timestamp.tzinfo is None:
        timestamp = timestamp.replace(tzinfo=UTC)
    return timestamp.astimezone(_JST).replace(tzinfo=None).isoformat(sep=" ")


def _aware(timestamp: datetime) -> datetime:
    return timestamp if timestamp.tzinfo is not None else timestamp.replace(tzinfo=UTC)


def merge_error_cells(events: Sequence[Any]) -> tuple[str, str]:
    """同じ行に入るエラー（level・title を持つ）→ (error_level, error_title)。ERROR 優先・title は " / " でつなぐ。"""
    if not events:
        return "", ""
    level = "ERROR" if any(e.level == "ERROR" for e in events) else events[0].level
    return level, " / ".join(e.title for e in events)


class CsvOverlay:
    """ログの行の流れに、ボタン・エラーの列を重ねる（ストリームのまま使える。ボタン・エラーは数件なので先に渡す）。

    - ボタン: 行の時刻が ``[押し始め, 押し始め + 秒数)`` に入れば 1。
    - エラー: ``first_at`` 以上で最初の行に入れる。
    - ログの範囲の外（最初の行より前・最後の行より後）の押し始め・``first_at`` は、その時刻に行を足す
      （``rows_before``・``rows_after``・``rows_only``）。足した行は ``(timestamp, 6 列のセル)`` で返す。
    使い方: 最初の行で ``rows_before(t0)`` → 各行で ``cells_for(t)`` → 最後に ``rows_after(t_last)``。
    ログが 0 行なら ``rows_only()`` だけ。
    """

    def __init__(self, presses: Iterable[Mapping[str, Any]], errors: Iterable[Any]) -> None:
        self._intervals: list[tuple[str, datetime, datetime]] = []
        for p in presses:
            start = _aware(p["time"])
            self._intervals.append((str(p["button"]), start, start + timedelta(seconds=float(p["seconds"]))))
        self._errors = sorted(errors, key=lambda e: _aware(e.first_at))
        self._next_error = 0

    def flags(self, t: datetime) -> list[str]:
        """時刻 t にボタンを押している列（BUTTON_COLUMNS の順）の "1"/"0"。"""
        t = _aware(t)
        pressed = {b for b, s, e in self._intervals if s <= t < e}
        return ["1" if key in pressed else "0" for key in BUTTON_COLUMNS]

    def _take_errors(self, *, upto: datetime, inclusive: bool) -> list[Any]:
        taken: list[Any] = []
        while self._next_error < len(self._errors):
            at = _aware(self._errors[self._next_error].first_at)
            if at < upto or (inclusive and at == upto):
                taken.append(self._errors[self._next_error])
                self._next_error += 1
            else:
                break
        return taken

    def cells_for(self, t: datetime) -> list[str]:
        """ログの 1 行（時刻 t）の 6 列。first_at が t 以下で未割り当てのエラーをここへ入れる。"""
        level, title = merge_error_cells(self._take_errors(upto=_aware(t), inclusive=True))
        return [*self.flags(t), level, title]

    def _synthetic(self, times: list[datetime], errors: list[Any]) -> list[tuple[datetime, list[str]]]:
        by_time: dict[datetime, list[Any]] = {}
        for t in times:
            by_time.setdefault(t, [])
        for e in errors:
            by_time.setdefault(_aware(e.first_at), []).append(e)
        rows = []
        for t in sorted(by_time):
            level, title = merge_error_cells(by_time[t])
            rows.append((t, [*self.flags(t), level, title]))
        return rows

    def rows_before(self, first_ts: datetime) -> list[tuple[datetime, list[str]]]:
        """最初の行より前の押し始め・first_at に足す行（時刻順）。cells_for より先に 1 度だけ呼ぶ。"""
        first_ts = _aware(first_ts)
        times = [s for _, s, _ in self._intervals if s < first_ts]
        return self._synthetic(times, self._take_errors(upto=first_ts, inclusive=False))

    def rows_after(self, last_ts: datetime) -> list[tuple[datetime, list[str]]]:
        """最後の行より後の押し始め・first_at に足す行（時刻順）。全部の行を流したあとに呼ぶ。"""
        last_ts = _aware(last_ts)
        times = [s for _, s, _ in self._intervals if s > last_ts]
        rest = self._errors[self._next_error :]
        self._next_error = len(self._errors)
        return self._synthetic(times, rest)

    def rows_only(self) -> list[tuple[datetime, list[str]]]:
        """ログが 0 行のとき、ボタン・エラーだけの行（時刻順）。"""
        times = [s for _, s, _ in self._intervals]
        rest = self._errors[self._next_error :]
        self._next_error = len(self._errors)
        return self._synthetic(times, rest)


def extra_row(columns: Sequence[str], timestamp: datetime, extra_cells: Sequence[str]) -> list[str]:
    """足す行 1 本（timestamp と末尾 6 列だけ埋め、他は空欄）。"""
    row = [""] * len(columns)
    row[list(columns).index("timestamp")] = format_csv_timestamp(timestamp)
    row[len(row) - len(EXTRA_COLUMNS) :] = list(extra_cells)
    return row


def with_extra(cells: list[str], extra_cells: Sequence[str]) -> list[str]:
    """``db_to_csv_cells`` の行の末尾 6 列をボタン・エラーに置き換える。"""
    return [*cells[: len(cells) - len(EXTRA_COLUMNS)], *extra_cells]
