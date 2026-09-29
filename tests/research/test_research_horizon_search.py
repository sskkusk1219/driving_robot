"""ホライズン自動選択（`tests/research/horizon_search.py`）のユニットテスト。

ProblemReport_20260921 手順6（2026-09-28）。`search_pedal`/`search_ff_horizons` は
パターン単位の交差検証 MAE で貪欲法（前向き選択）を行うだけの純粋関数なので、
車速の合成系列（既知の関数形）で「正しい方向に選ぶ・止まる」ことを検算する。
"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from tests.research.horizon_search import (
    HorizonSearchSettings,
    format_result_table,
    read_pattern_groups,
    search_ff_horizons,
    search_pedal,
)
from tests.research.learning_patterns import LearningDataError
from tests.research.research_types import DriveLog

T0 = datetime(2026, 9, 28, tzinfo=UTC)


def _timestamps(n: int, dt_s: float = 0.1) -> list[datetime]:
    return [T0 + timedelta(seconds=dt_s * i) for i in range(n)]


def _sine_speed(n: int, dt_s: float = 0.1, period_s: float = 20.0) -> np.ndarray:
    """周期 `period_s` の正弦波状の車速（0 以上）。先読みが長いほど当てやすくなる形。"""
    t = np.arange(n) * dt_s
    return 60.0 + 20.0 * np.sin(2.0 * np.pi * t / period_s)


def _groups(n: int, pattern_len: int = 50) -> np.ndarray:
    """`pattern_len` 行ごとに別パターン名にする（GroupKFold 用）。"""
    return np.array([f"p{i // pattern_len}" for i in range(n)])


# ── search_pedal ──────────────────────────────────────────────────────


def test_search_pedal_starts_from_regime_horizon_only() -> None:
    """改善する候補が無ければ、開始点（regime_horizon_s のみ）のまま止まる。"""
    n = 400
    speed = _sine_speed(n)
    timestamps = _timestamps(n)
    # ラベルは車速と無関係な乱数 → どのホライズンを足しても MAE は改善しない
    rng = np.random.default_rng(0)
    label = np.clip(rng.normal(20.0, 0.01, n), 0.0, None)
    groups = _groups(n)

    result = search_pedal(
        pedal="accel", speed=speed, timestamps=timestamps, label=label, deadband_pct=5.0,
        groups=groups, regime_horizon_s=1.0, past_horizons_s=(0.5,), past_as_delta=True,
        include_v0_sq=True, include_dv_regime_x_v0=True, grid=(0.5, 1.0, 2.0, 3.0),
        max_horizons=4, min_improvement=0.5, cv_splits=3,
    )

    assert result.spec.lookahead_horizons_s == (1.0,)
    assert len(result.steps) == 1
    assert result.steps[0].added_horizon_s is None


def test_search_pedal_adds_horizons_that_improve_mae() -> None:
    """開度がその先読み先の車速に強く相関する合成データでは、追加のホライズンを選ぶ。"""
    n = 1200
    speed = _sine_speed(n, period_s=15.0)
    timestamps = _timestamps(n)
    dt = 0.1
    off_2s = round(2.0 / dt)
    # 開度 = 2.0s 先の車速（+ わずかなノイズ）。regime(1.0s) だけでは当てにくく、
    # 2.0s を足すと大きく改善するはず
    rng = np.random.default_rng(1)
    label = np.clip(
        np.concatenate([speed[off_2s:], np.full(off_2s, speed[-1])])
        + rng.normal(0.0, 0.05, n),
        0.0,
        None,
    )
    groups = _groups(n)

    result = search_pedal(
        pedal="accel", speed=speed, timestamps=timestamps, label=label, deadband_pct=0.0,
        groups=groups, regime_horizon_s=1.0, past_horizons_s=(0.5,), past_as_delta=True,
        include_v0_sq=True, include_dv_regime_x_v0=True, grid=(0.5, 1.0, 1.5, 2.0, 2.5, 3.0),
        max_horizons=4, min_improvement=0.01, cv_splits=3,
    )

    assert 2.0 in result.spec.lookahead_horizons_s
    assert len(result.steps) >= 2
    # MAE は開始点より改善している
    assert result.cv_mae < result.steps[0].cv_mae


def test_search_pedal_respects_max_horizons() -> None:
    n = 800
    speed = _sine_speed(n)
    timestamps = _timestamps(n)
    rng = np.random.default_rng(2)
    label = np.clip(speed + rng.normal(0.0, 0.2, n), 0.0, None)
    groups = _groups(n)

    result = search_pedal(
        pedal="accel", speed=speed, timestamps=timestamps, label=label, deadband_pct=0.0,
        groups=groups, regime_horizon_s=1.0, past_horizons_s=(0.5,), past_as_delta=True,
        include_v0_sq=True, include_dv_regime_x_v0=True,
        grid=(0.5, 1.0, 1.5, 2.0, 2.5, 3.0), max_horizons=2, min_improvement=0.0, cv_splits=3,
    )

    assert len(result.spec.lookahead_horizons_s) <= 2


def test_search_pedal_only_picks_horizons_from_grid() -> None:
    n = 600
    speed = _sine_speed(n)
    timestamps = _timestamps(n)
    rng = np.random.default_rng(3)
    label = np.clip(speed + rng.normal(0.0, 0.2, n), 0.0, None)
    groups = _groups(n)

    result = search_pedal(
        pedal="accel", speed=speed, timestamps=timestamps, label=label, deadband_pct=0.0,
        groups=groups, regime_horizon_s=1.0, past_horizons_s=(0.5,), past_as_delta=True,
        include_v0_sq=True, include_dv_regime_x_v0=True, grid=(0.7, 1.3),
        max_horizons=4, min_improvement=0.0, cv_splits=3,
    )

    assert set(result.spec.lookahead_horizons_s) <= {0.7, 1.0, 1.3}


def test_search_pedal_raises_when_too_few_effective_rows() -> None:
    n = 20
    speed = _sine_speed(n)
    timestamps = _timestamps(n)
    label = np.full(n, 20.0)  # 全行が不感帯未満（0行が有効）
    groups = _groups(n, pattern_len=5)

    with pytest.raises(LearningDataError, match="brake"):
        search_pedal(
            pedal="brake", speed=speed, timestamps=timestamps, label=label, deadband_pct=50.0,
            groups=groups, regime_horizon_s=1.0, past_horizons_s=(0.5,), past_as_delta=True,
            include_v0_sq=True, include_dv_regime_x_v0=True, grid=(0.5, 1.0, 2.0),
            max_horizons=4, min_improvement=0.01, cv_splits=3,
        )


def test_format_result_table_contains_steps() -> None:
    n = 400
    speed = _sine_speed(n)
    timestamps = _timestamps(n)
    rng = np.random.default_rng(4)
    label = np.clip(speed + rng.normal(0.0, 0.2, n), 0.0, None)
    groups = _groups(n)

    result = search_pedal(
        pedal="accel", speed=speed, timestamps=timestamps, label=label, deadband_pct=0.0,
        groups=groups, regime_horizon_s=1.0, past_horizons_s=(0.5,), past_as_delta=True,
        include_v0_sq=True, include_dv_regime_x_v0=True, grid=(0.5, 1.0, 2.0),
        max_horizons=4, min_improvement=0.01, cv_splits=3,
    )

    table = format_result_table(result)
    assert "CV-MAE" in table
    assert "(開始)" in table


# ── search_ff_horizons / _single_session_inputs ─────────────────────────


def _logs(rows: list[tuple[float, float, float]], session_id: str = "s1") -> list[DriveLog]:
    t0 = T0
    return [
        DriveLog(
            id=i, session_id=session_id, timestamp=t0 + timedelta(seconds=0.1 * i),
            ref_speed_kmh=None, actual_speed_kmh=v,
            accel_opening=a, brake_opening=b,
            accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
        )
        for i, (v, a, b) in enumerate(rows)
    ]


def _mixed_rows(n: int = 600) -> list[tuple[float, float, float]]:
    rows: list[tuple[float, float, float]] = []
    v = 5.0
    for i in range(n):
        if i % 3 == 0:
            v += 0.5
            rows.append((v, 20.0 + 0.01 * v, 0.0))
        elif i % 3 == 1:
            v = max(1.0, v - 0.2)
            rows.append((v, 0.0, 0.0))
        else:
            v = max(1.0, v - 0.5)
            rows.append((v, 0.0, 15.0 + 0.01 * v))
    return rows


def _settings(grid: tuple[float, ...] = (0.5, 1.0, 2.0)) -> HorizonSearchSettings:
    return HorizonSearchSettings(
        grid=grid, max_horizons=3, min_improvement=0.01, cv_splits=3,
        regime_horizon_s=1.0, past_horizons_s=(0.5,), past_as_delta=True,
        include_v0_sq=True, include_dv_regime_x_v0=True,
    )


def test_search_ff_horizons_returns_accel_and_brake_results() -> None:
    logs = _logs(_mixed_rows())
    patterns = np.array([f"p{i // 40}" for i in range(len(logs))])

    accel_result, brake_result = search_ff_horizons(logs, patterns, 10.0, 10.0, _settings())

    assert accel_result.pedal == "accel"
    assert brake_result.pedal == "brake"
    assert 1.0 in accel_result.spec.lookahead_horizons_s
    assert 1.0 in brake_result.spec.lookahead_horizons_s


def test_search_ff_horizons_rejects_multi_session_logs() -> None:
    logs = _logs(_mixed_rows(60), session_id="s1") + _logs(_mixed_rows(60), session_id="s2")
    patterns = np.array([f"p{i // 20}" for i in range(len(logs))])

    with pytest.raises(ValueError, match="単一セッション"):
        search_ff_horizons(logs, patterns, 10.0, 10.0, _settings())


# ── read_pattern_groups ──────────────────────────────────────────────


def test_read_pattern_groups_filters_to_pattern_drive_section(tmp_path: Path) -> None:
    csv_path = tmp_path / "log.csv"
    csv_path.write_text(
        "section,pattern\n"
        "PRE_DRIVE_CHECK,\n"
        "PATTERN_DRIVE,grid:0\n"
        "PATTERN_DRIVE,grid:1\n"
        "MODE_DRIVE,\n",
        encoding="utf-8",
    )

    groups = read_pattern_groups(csv_path)

    assert list(groups) == ["grid:0", "grid:1"]
