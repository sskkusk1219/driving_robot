"""drop_patterns の単体テスト（2026-10-17 段2o-8）。"""

from __future__ import annotations

import csv
from pathlib import Path

from tests.research import drop_patterns as dp


def _write(path: Path) -> None:
    rows = [
        ("0.0", "", "1"), ("1.0", "1:A", "10"), ("2.0", "1:A", "20"),
        ("3.0", "2:B", "30"), ("4.0", "2:B", "5"), ("5.0", "3:C", "7"),
    ]
    with path.open("w", newline="") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["elapsed_s", "pattern", "actual_speed_kmh"])
        w.writerows(rows)


def test_drop_removes_only_given_patterns(tmp_path: Path) -> None:
    src = tmp_path / "log.csv"
    _write(src)
    assert dp.main([str(src), "--drop", "2"]) == 0
    out = tmp_path / "log_lap1.csv"
    with out.open(newline="") as f:
        got = list(csv.DictReader(f))
    assert [r["pattern"] for r in got] == ["", "1:A", "1:A", "3:C"]
    assert list(got[0].keys()) == ["elapsed_s", "pattern", "actual_speed_kmh"]


def test_summary_and_missing_number(tmp_path: Path) -> None:
    src = tmp_path / "log.csv"
    _write(src)
    with src.open(newline="") as f:
        s = dp.summarize(list(csv.DictReader(f)))
    got = [(x["pattern"], x["start"], x["end"], x["vmax"]) for x in s]
    assert got[1] == ("2:B", 3.0, 4.0, 30.0)
    assert dp.main([str(src), "--drop", "9"]) == 2
