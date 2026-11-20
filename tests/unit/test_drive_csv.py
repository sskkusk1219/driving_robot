# ruff: noqa: E501
"""研究ハーネス形式の CSV（約 100 列）と drive_logs の行の相互変換（src/infra/drive_csv.py）。"""

from __future__ import annotations

import csv
from datetime import UTC, datetime
from pathlib import Path

from src.infra import drive_csv


def _row(**over: str) -> dict[str, str]:
    cols = drive_csv.csv_columns()
    row = dict.fromkeys(cols, "")
    row.update(
        timestamp="2026-11-15 09:30:15.250000",
        elapsed_s="12.250",
        mode_time_s="2.250",
        ref_speed_kmh="30.500",
        actual_speed_kmh="29.875",
        deviation_kmh="-0.625",
        accel_cmd_pct="12.345",
        brake_cmd_pct="0.000",
        accel_cmd_mm="11.73",
        brake_cmd_mm="0.00",
        accel_actual_pct="12.100",
        accel_current="310.5",
        brake_current="0.0",
        section="MODE_DRIVE",
        candidate="C15",
        phase="Low",
        governor_active="0",
        c10_effort_pct="0.250",
    )
    row.update(over)
    return row


def test_csv_columns_are_core_columns() -> None:
    cols = drive_csv.csv_columns()
    assert cols[0] == "timestamp" and "accel_cmd_pct" in cols and len(cols) > 70
    # 9 列の対応の CSV 側の列名はすべて core の列にある
    assert set(drive_csv.CSV_TO_DB) <= set(cols)


def test_timestamp_jst_to_utc() -> None:
    ts = drive_csv.parse_csv_timestamp("2026-11-15 09:30:15.250000")
    assert ts == datetime(2026, 11, 15, 0, 30, 15, 250000, tzinfo=UTC)
    assert drive_csv.parse_csv_timestamp("2026-11-15 09:30:15").second == 15  # 小数なしも読める


def test_row_to_db_maps_nine_columns_and_detail() -> None:
    sid = "00000000-0000-0000-0000-000000000001"
    out = drive_csv.csv_row_to_db(sid, _row(), detail_as_json=False)
    session, ts, ref, actual, acc, brk, acc_pos, brk_pos, acc_i, brk_i, detail = out
    assert session == sid
    assert ts.tzinfo is UTC
    assert (ref, actual, acc, brk) == (30.5, 29.875, 12.345, 0.0)
    assert (acc_pos, brk_pos) == (1173, 0)  # mm × 100 = pulse
    assert (acc_i, brk_i) == (310.5, 0.0)
    # 9 列に入った列は detail に無い。空欄は入れない。残りは文字列のまま
    for col in drive_csv.CSV_TO_DB:
        assert col not in detail
    assert detail["section"] == "MODE_DRIVE" and detail["c10_effort_pct"] == "0.250"
    assert detail["accel_actual_pct"] == "12.100"
    assert "brake_actual_pct" not in detail


def test_blank_cells_become_null_or_zero() -> None:
    out = drive_csv.csv_row_to_db("s", _row(ref_speed_kmh="", actual_speed_kmh="", accel_cmd_mm=""), detail_as_json=False)
    assert out[2] is None  # 基準なし → NULL
    assert out[3] == 0.0 and out[6] == 0  # NOT NULL の列は 0


def test_bad_rows_are_dropped() -> None:
    rows = [_row(), _row(timestamp=""), _row(timestamp="not a time"), _row()]
    out = list(drive_csv.csv_to_db_rows("s", rows))
    assert len(out) == 2


def test_roundtrip_csv_to_db_to_csv(tmp_path: Path) -> None:
    """CSV → DB の行 → CSV の 1 行で、全列が数値として同じになる（書式 12.300 と 12.3 の違いは許す）。"""
    path = tmp_path / "x.csv"
    cols = drive_csv.csv_columns()
    original = [_row(), _row(timestamp="2026-11-15 09:30:15.350000", ref_speed_kmh="", elapsed_s="12.350")]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for r in original:
            w.writerow([r[c] for c in cols])
    rows = list(drive_csv.csv_to_db_rows("s", drive_csv.read_csv_rows(path)))
    assert len(rows) == 2
    dbs = [drive_csv.csv_row_to_db("s", r, detail_as_json=False) for r in original]
    header = drive_csv.csv_header({k for db in dbs for k in db[10]})
    assert header == list(cols)
    for src_row, db in zip(original, dbs, strict=True):
        cells = drive_csv.db_to_csv_cells(
            timestamp=db[1], ref_speed_kmh=db[2], actual_speed_kmh=db[3], accel_opening=db[4],
            brake_opening=db[5], accel_pos=db[6], brake_pos=db[7], accel_current=db[8],
            brake_current=db[9], detail=db[10], columns=header,
        )
        back = dict(zip(header, cells, strict=True))
        assert back["timestamp"] == src_row["timestamp"]  # JST の文字列に戻る
        for col in cols:
            a, b = src_row[col], back[col]
            if a == b:
                continue
            assert a != "" and b != "", f"{col}: {a!r} vs {b!r}"
            assert float(a) == float(b), f"{col}: {a!r} vs {b!r}"


def test_header_appends_unknown_detail_keys() -> None:
    h = drive_csv.csv_header({"accel_kp", "old_column_x"})
    assert h[: len(drive_csv.csv_columns())] == list(drive_csv.csv_columns())
    assert h[-1] == "old_column_x"


def test_decode_detail_variants() -> None:
    assert drive_csv.decode_detail(None) == {}
    assert drive_csv.decode_detail('{"a": "1"}') == {"a": "1"}
    assert drive_csv.decode_detail({"a": "1"}) == {"a": "1"}
    assert drive_csv.decode_detail("broken") == {}
