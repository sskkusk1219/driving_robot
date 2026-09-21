"""compare_runs.py のユニットテスト（KAIZEN 5.8.1 の順位基準）。

車両には触らない。合成 CSV で、走破 > 最大逸脱 > p95 > 符号反転 > ペダル切替・指令 p95 の
優先順位どおりに順位が付くことを確かめる。
"""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import pytest

from src.models.drive_log import DriveLogData
from tests.research import compare_runs as cr
from tests.research import config as cfgmod
from tests.research import drive_log as dlmod


def _mode_sample(
    t: float, ref: float, actual: float, accel: float, brake: float, *, candidate: str = ""
) -> dlmod.DriveSample:
    data = DriveLogData(
        ref_speed_kmh=ref, actual_speed_kmh=actual, accel_opening=accel, brake_opening=brake,
        accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
    )
    phase = "ACCEL" if accel > 0.0 else "BRAKE" if brake > 0.0 else "COAST"
    return dlmod.DriveSample(
        elapsed_s=t, timestamp=datetime.now(tz=UTC), data=data,
        section=dlmod.SECTION_MODE_DRIVE, phase=phase, pattern="Low", mode_time_s=t,
        candidate=candidate,
    )


def _write_run(
    path: Path, *, n: int, dt: float, deviation_kmh: float, accel: float,
    ref: float = 50.0, candidate: str = "",
) -> None:
    """0 〜 (n-1)*dt 秒、一定の偏差・一定のペダルで走る合成ラン。"""
    samples = [
        _mode_sample(i * dt, ref=ref, actual=ref + deviation_kmh, accel=accel, brake=0.0,
                     candidate=candidate)
        for i in range(n)
    ]
    dlmod.write_csv(samples, path)


def _cfg(tmp_path: Path) -> Path:
    path = tmp_path / "cfg.yaml"
    cfgmod.load_config(path)  # 既定値で作る（コピー）
    return path


def test_completed_run_ranks_above_aborted_run(tmp_path: Path) -> None:
    completed = tmp_path / "completed.csv"
    aborted = tmp_path / "aborted.csv"
    _write_run(completed, n=20, dt=0.1, deviation_kmh=5.0, accel=15.0)  # 最後まで走る（1.9s）
    # 偏差は小さいが早く止まる（0.4s）
    _write_run(aborted, n=5, dt=0.1, deviation_kmh=0.1, accel=15.0)

    results = [
        cr.evaluate_run(completed, "完走", cfgmod.load_config(_cfg(tmp_path)), duration_s=1.9),
        cr.evaluate_run(aborted, "中断", cfgmod.load_config(_cfg(tmp_path)), duration_s=1.9),
    ]
    ranked = cr.rank(results)
    assert [r.label for r in ranked] == ["完走", "中断"]  # 偏差が小さくても中断は下


def test_later_abort_ranks_above_earlier_abort(tmp_path: Path) -> None:
    early = tmp_path / "early.csv"
    late = tmp_path / "late.csv"
    _write_run(early, n=5, dt=0.1, deviation_kmh=0.0, accel=15.0)  # 0.4s で中断
    _write_run(late, n=10, dt=0.1, deviation_kmh=0.0, accel=15.0)  # 0.9s で中断
    cfg = cfgmod.load_config(_cfg(tmp_path))
    results = [
        cr.evaluate_run(early, "早い中断", cfg, duration_s=100.0),
        cr.evaluate_run(late, "遅い中断", cfg, duration_s=100.0),
    ]
    ranked = cr.rank(results)
    assert [r.label for r in ranked] == ["遅い中断", "早い中断"]


def test_smaller_max_deviation_ranks_higher_when_both_completed(tmp_path: Path) -> None:
    good = tmp_path / "good.csv"
    bad = tmp_path / "bad.csv"
    _write_run(good, n=20, dt=0.1, deviation_kmh=1.0, accel=15.0)
    _write_run(bad, n=20, dt=0.1, deviation_kmh=10.0, accel=15.0)
    cfg = cfgmod.load_config(_cfg(tmp_path))
    results = [
        cr.evaluate_run(bad, "偏差大", cfg, duration_s=1.9),
        cr.evaluate_run(good, "偏差小", cfg, duration_s=1.9),
    ]
    ranked = cr.rank(results)
    assert [r.label for r in ranked] == ["偏差小", "偏差大"]


def test_completion_allows_one_cycle_short_of_duration(tmp_path: Path) -> None:
    """1 周期弱だけ足りない実走は走破とみなす（実機 1799.898s の縮小再現）。

    走行ループは duration_s 到達時点で抜けるため、最後の記録行は 1 周期手前になる。
    dt=0.1s・到達 1.9s に対し duration_s=2.002 を渡すと、旧式のしきい値は
    2.002 - 0.1 = 1.902 で 1.9 < 1.902 のため未走破と誤判定されていた。
    2 周期の許容幅なら 2.002 - 0.2 = 1.802 で 1.9 >= 1.802 となり走破と判定される。
    """
    path = tmp_path / "near_complete.csv"
    _write_run(path, n=20, dt=0.1, deviation_kmh=0.0, accel=15.0)  # 到達 1.9s
    cfg = cfgmod.load_config(_cfg(tmp_path))
    result = cr.evaluate_run(path, "近接完走", cfg, duration_s=2.002)
    assert result.completed


def test_completion_rejects_run_short_by_more_than_two_cycles(tmp_path: Path) -> None:
    """2 周期より大きく足りない走行は未走破のまま（緩めすぎていないことの確認）。"""
    path = tmp_path / "near_complete.csv"
    _write_run(path, n=20, dt=0.1, deviation_kmh=0.0, accel=15.0)  # 到達 1.9s
    cfg = cfgmod.load_config(_cfg(tmp_path))
    result = cr.evaluate_run(path, "未走破", cfg, duration_s=2.3)
    assert not result.completed


def test_command_rate_p95_of_constant_signal_is_zero() -> None:
    rows = [
        cr.ModeRow(
            t_s=i * 0.1, ref_kmh=50.0, actual_kmh=50.0, accel_pct=15.0, brake_pct=0.0,
            ff_effort_pct=15.0, pid_effort_pct=0.0, effort_pct=15.0, segment="Low", phase="ACCEL",
        )
        for i in range(10)
    ]
    assert cr.command_rate_p95(rows) == 0.0


# ─────────────────────────────────────────────────────────────────────
# 2026-09-15: 候補ごとの集計・帯別偏差・既定ラベル（各候補 3 本まとめて走る比較のため）
# ─────────────────────────────────────────────────────────────────────


def test_three_runs_same_label_aggregate_median_and_range(tmp_path: Path) -> None:
    """同じラベル 3 本の集計は中央値・最小〜最大が正しい。"""
    cfg = cfgmod.load_config(_cfg(tmp_path))
    paths = [tmp_path / f"c1_{i}.csv" for i in range(3)]
    # 最大逸脱（= 偏差の絶対値）を 1.0 / 2.0 / 3.0 km/h にする
    for path, dev in zip(paths, (1.0, 2.0, 3.0), strict=True):
        _write_run(path, n=20, dt=0.1, deviation_kmh=dev, accel=15.0)
    results = [cr.evaluate_run(p, "C1", cfg, duration_s=1.9) for p in paths]
    groups = cr.group_by_label(results)
    assert [g.label for g in groups] == ["C1"]
    table = cr.aggregate_table(groups)
    assert "| C1 | 3 | 3 | 2.00（1.00〜3.00） |" in table


def test_group_with_aborted_run_ranks_below_completed_group(tmp_path: Path) -> None:
    """未走破本を含む候補は、全本走破した候補より下に並ぶ（中央値の良し悪しに関わらず）。"""
    cfg = cfgmod.load_config(_cfg(tmp_path))
    good_paths = [tmp_path / f"good_{i}.csv" for i in range(2)]
    for path in good_paths:
        _write_run(path, n=20, dt=0.1, deviation_kmh=10.0, accel=15.0)  # 偏差は大きいが完走
    bad_paths = [tmp_path / f"bad_{i}.csv" for i in range(2)]
    for path in bad_paths:
        _write_run(path, n=5, dt=0.1, deviation_kmh=0.1, accel=15.0)  # 偏差は小さいが中断
    results = (
        [cr.evaluate_run(p, "完走", cfg, duration_s=1.9) for p in good_paths]
        + [cr.evaluate_run(p, "中断あり", cfg, duration_s=1.9) for p in bad_paths]
    )
    groups = cr.group_by_label(results)
    ranked = cr.rank_groups(groups)
    assert [g.label for g in ranked] == ["完走", "中断あり"]


def test_band_stats_mean_p95_and_empty_band() -> None:
    """帯ごとの平均偏差・|偏差|p95 と、行が無い帯（—扱い）を確認する。"""
    rows = [
        cr.ModeRow(
            t_s=float(i), ref_kmh=20.0, actual_kmh=20.0 + dev, accel_pct=15.0, brake_pct=0.0,
            ff_effort_pct=15.0, pid_effort_pct=0.0, effort_pct=15.0, segment="Low", phase="ACCEL",
        )
        for i, dev in enumerate((1.0, -1.0, 2.0, -2.0, 3.0))
    ]
    stats = cr.compute_band_stats(rows)
    band_0_40 = stats["0〜40"]
    assert band_0_40.n == 5
    assert band_0_40.mean_kmh == pytest.approx(0.6)  # (1-1+2-2+3)/5
    assert band_0_40.p95_abs_kmh == pytest.approx(float(np.percentile([1, 1, 2, 2, 3], 95)))
    # 40〜80 / 80〜120 / 120〜 の帯には行が無い
    assert stats["40〜80"].mean_kmh is None
    assert stats["80〜120"].mean_kmh is None
    assert stats["120〜"].mean_kmh is None


def test_default_label_from_candidate_column(tmp_path: Path) -> None:
    """--labels 省略時は CSV の candidate 列を使う。"""
    path = tmp_path / "c5.csv"
    _write_run(path, n=10, dt=0.1, deviation_kmh=0.0, accel=15.0, candidate="C5")
    assert cr.default_label(path) == "C5"


def test_default_label_falls_back_to_stem_when_candidate_missing_or_mixed(
    tmp_path: Path,
) -> None:
    """candidate 列が空、または混在しているときはファイル名にフォールバックする。"""
    empty_path = tmp_path / "no_candidate.csv"
    _write_run(empty_path, n=10, dt=0.1, deviation_kmh=0.0, accel=15.0)  # candidate=""
    assert cr.default_label(empty_path) == "no_candidate"

    mixed_path = tmp_path / "mixed_candidate.csv"
    samples = [
        _mode_sample(0.0, ref=50.0, actual=50.0, accel=15.0, brake=0.0, candidate="C1"),
        _mode_sample(0.1, ref=50.0, actual=50.0, accel=15.0, brake=0.0, candidate="C4"),
    ]
    dlmod.write_csv(samples, mixed_path)
    assert cr.default_label(mixed_path) == "mixed_candidate"


def test_cli_main_with_repeated_labels_returns_0_and_prints_aggregate_header(
    tmp_path: Path, capsys: pytest.CaptureFixture[str],
) -> None:
    cfg_path = _cfg(tmp_path)
    paths = [tmp_path / f"run_{i}.csv" for i in range(3)]
    for i, path in enumerate(paths):
        _write_run(path, n=20, dt=0.1, deviation_kmh=1.0 + i, accel=15.0, candidate="C1")
    rc = cr.main([str(p) for p in paths] + ["--config", str(cfg_path), "--duration", "1.9"])
    out = capsys.readouterr().out
    assert rc == 0
    assert "## 候補ごとの集計" in out
    assert "1 位（中央値）: C1" in out
    assert "## 基準車速帯別の偏差" in out
