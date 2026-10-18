"""mode_report の「FB が直した量」と FEL の 1 行のテスト（2026-10-19 段2s-2b）。"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import mode_report as mr
from tests.research.drive_log import CSV_COLUMNS


def _row(t, ref, act, a, b, phase, *, aff=None, bff=None, c10=None, dob=None, coast=None):
    return mr.ModeRow(
        t_s=t, ref_kmh=ref, actual_kmh=act, accel_pct=a, brake_pct=b, ff_effort_pct=0.0,
        pid_effort_pct=0.0, effort_pct=0.0, segment="Low", phase=phase,
        accel_ff_pct=aff, brake_ff_pct=bff, c10_effort_pct=c10, dob_effort_pct=dob,
        coast_effort_pct=coast,
    )


def test_fb_effect_splits_p_dob_coast_and_final_minus_ff() -> None:
    rows = [
        # アクセル: FF 20、P 1.0 + DOB 0.5 → 最終 21.5（待機位置 6 より上）
        _row(0.0, 30, 30, 21.5, 6.0, "ACCEL", aff=20.0, bff=0.0, c10=1.5, dob=0.5, coast=0.0),
        _row(0.1, 30, 30, 22.5, 6.0, "ACCEL", aff=20.0, bff=0.0, c10=2.5, dob=0.5, coast=0.0),
        # ブレーキ: FF −10、P −2 → 最終ブレーキ 12
        _row(0.2, 20, 20, 0.0, 12.0, "BRAKE", aff=0.0, bff=10.0, c10=-2.0, dob=0.0, coast=0.0),
        # 惰行の補助: C10 の列に補助の分が入る（P は 0）
        _row(0.3, 20, 20, 0.0, 6.0, "COAST", aff=0.0, bff=0.0, c10=3.0, dob=0.0, coast=3.0),
        # 停車（数えない）
        _row(0.4, 0, 0, 0.0, 6.0, "COAST", aff=0.0, bff=0.0, c10=0.0, dob=0.0, coast=0.0),
    ]
    p, dob, coast, diff = mr.fb_effect(rows, 6.0, 6.0)
    assert p.stats["アクセル"][0] == 2 and p.stats["アクセル"][1] == pytest.approx(1.5)
    assert p.stats["ブレーキ"][1] == pytest.approx(2.0)
    assert p.stats["全体"][0] == 4  # 停車の行は入らない
    assert p.stats["全体"][1] == pytest.approx((1.0 + 2.0 + 2.0 + 0.0) / 4)
    assert dob.stats["アクセル"][1] == pytest.approx(0.5)
    assert coast.stats["全体"][1] == pytest.approx(3.0 / 4)
    # 最終 − FF: アクセル 1.5・2.5、ブレーキ −12 − (−10) = −2、惰行 0
    assert diff.stats["アクセル"][1] == pytest.approx(2.0)
    assert diff.stats["ブレーキ"][1] == pytest.approx(2.0)
    assert "アクセル" in p.text(0.1) and "p95" in p.text(0.1)


def test_fb_effect_without_columns_is_empty() -> None:
    rows = [_row(0.0, 30, 30, 20.0, 6.0, "ACCEL")]
    for e in mr.fb_effect(rows, 6.0, 6.0):
        assert e.stats == {} and "なし" in e.text(0.1)


def test_rows_from_csv_reads_the_ff_and_fb_columns(tmp_path: Path) -> None:
    path = tmp_path / "log.csv"
    cols = list(CSV_COLUMNS)
    line = dict.fromkeys(cols, "")
    line.update({
        "mode_time_s": "1.0", "ref_speed_kmh": "30", "actual_speed_kmh": "30.1",
        "accel_ff_pct": "20.0", "brake_ff_pct": "0.0", "accel_cmd_pct": "21.5",
        "brake_cmd_pct": "6.0", "section": "MODE_DRIVE", "pattern": "Low", "phase": "ACCEL",
        "c10_effort_pct": "1.5", "dob_effort_pct": "0.5", "coast_effort_pct": "0.0",
    })
    path.write_text(",".join(cols) + "\n" + ",".join(line[c] for c in cols) + "\n",
                    encoding="utf-8")
    (r,) = mr.rows_from_csv(path)
    assert (r.accel_ff_pct, r.brake_ff_pct, r.c10_effort_pct) == (20.0, 0.0, 1.5)
    assert (r.dob_effort_pct, r.coast_effort_pct) == (0.5, 0.0)


def test_report_has_fb_effect_and_fel_lines(tmp_path: Path) -> None:
    cfg = cfgmod.load_config()
    rows = [
        _row(0.1 * i, 30.0, 30.0 + 0.2 * (i % 3), 21.5, 6.0, "ACCEL",
             aff=20.0, bff=0.0, c10=1.5, dob=0.5, coast=0.0)
        for i in range(300)
    ]
    info = mr.RunInfo(
        label="T", title="t", controller="c", csv_path=tmp_path / "x.csv", hw_mode="stub",
        mode_name="m", started_at=datetime(2026, 10, 19, 12, 0),
        fel_text="  FEL: 有効・方法 cell・補正のあるマス 123・学習 CSV a.csv",
    )
    path = mr.write_mode_report(rows, cfg, info, tmp_path)
    text = path.read_text(encoding="utf-8")
    assert "FB が直した量: P（C10 の偏差の上乗せ）" in text
    assert "FB が直した量: 最終の指令 − FF の開度" in text
    assert "FEL: 有効・方法 cell・補正のあるマス 123・学習 CSV a.csv" in text
    info2 = mr.RunInfo(
        label="U", title="t", controller="c", csv_path=tmp_path / "x.csv", hw_mode="stub",
        mode_name="m", started_at=datetime(2026, 10, 19, 12, 1),
    )
    assert "不明" in mr.write_mode_report(rows, cfg, info2, tmp_path).read_text(encoding="utf-8")
