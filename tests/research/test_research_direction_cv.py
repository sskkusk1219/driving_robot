"""direction_cv（向き別の交差検証）のテスト。合成データだけを使い、実 CSV には依存しない。"""

from __future__ import annotations

from dataclasses import replace
from datetime import UTC, datetime, timedelta
from types import SimpleNamespace

import numpy as np
import pytest

from tests.research.direction_cv import (
    DIR_PRESS,
    DIR_RELEASE,
    TRAIN_ALL,
    TRAIN_SAME,
    directed_regime_data,
    direction_cv_table,
    format_table,
)
from tests.research.ff_model import DEFAULT_FEATURE_SPEC
from tests.research.train_candidate_model import effective_regime_data
from tests.research.wltp_grid import pedal_release

N_PATTERNS = 8
ROWS_PER_PATTERN = 80


def _synthetic(offset_release: float, seed: int = 0):
    """向きで切片が違う（戻しは +offset_release）合成データ。各パターンに両方の向きを含む。"""
    rng = np.random.default_rng(seed)
    n = N_PATTERNS * ROWS_PER_PATTERN
    a = rng.uniform(0.0, 30.0, n)
    v0 = rng.uniform(0.0, 150.0, n)
    x = np.column_stack([v0, a])
    release = np.tile(np.arange(ROWS_PER_PATTERN) % 2 == 1, N_PATTERNS)
    y = 2.0 * a + np.where(release, offset_release, 0.0)
    pattern = np.repeat([f"{i}:P" for i in range(N_PATTERNS)], ROWS_PER_PATTERN)
    data = SimpleNamespace(x=x, y=y, pattern=pattern)
    return data, release


def _find(rows, direction, training, band=None):
    (hit,) = [r for r in rows if r.direction == direction and r.training == training
              and r.band == band]
    return hit


def test_intercept_differs_by_direction() -> None:
    data, release = _synthetic(5.0)
    rows = direction_cv_table(data, release)
    # 全部で学習すると、踏み込みは多め（正）・戻しは少なめ（負）に出る
    assert _find(rows, DIR_PRESS, TRAIN_ALL).mean_error > 1.0
    assert _find(rows, DIR_RELEASE, TRAIN_ALL).mean_error < -1.0
    # 向きごとに学習すればどちらもほぼ 0
    assert _find(rows, DIR_PRESS, TRAIN_SAME).mean_error == pytest.approx(0.0, abs=0.1)
    assert _find(rows, DIR_RELEASE, TRAIN_SAME).mean_error == pytest.approx(0.0, abs=0.1)
    assert _find(rows, DIR_PRESS, TRAIN_ALL).n == release.size - int(release.sum())


def test_no_direction_difference_is_near_zero() -> None:
    data, release = _synthetic(0.0)
    rows = direction_cv_table(data, release)
    for direction in (DIR_PRESS, DIR_RELEASE):
        for training in (TRAIN_ALL, TRAIN_SAME):
            assert _find(rows, direction, training).mean_error == pytest.approx(0.0, abs=0.1)


def test_band_rows_and_seconds() -> None:
    data, release = _synthetic(5.0)
    rows = direction_cv_table(data, release, period_s=0.5)
    whole = _find(rows, DIR_RELEASE, TRAIN_ALL)
    assert whole.seconds == pytest.approx(whole.n * 0.5)
    band_n = sum(r.n for r in rows if r.band is not None
                 and r.direction == DIR_RELEASE and r.training == TRAIN_ALL)
    assert band_n == whole.n
    text = format_table(rows, with_band=True)
    assert "[120,∞)" in text


def test_too_few_training_rows_gives_nan() -> None:
    data, release = _synthetic(5.0)
    release[:] = False  # 戻しの行が無い → 戻しだけで学習は作れない
    rows = direction_cv_table(data, release)
    assert _find(rows, DIR_RELEASE, TRAIN_SAME).n == 0
    assert np.isnan(_find(rows, DIR_RELEASE, TRAIN_SAME).mae)


def _logs(accel: np.ndarray, brake: np.ndarray):
    t0 = datetime(2026, 1, 1, tzinfo=UTC)
    rng = np.random.default_rng(1)
    speed = 50.0 + 10.0 * np.sin(np.arange(len(accel)) / 20.0) + rng.normal(0, 0.1, len(accel))
    return [
        SimpleNamespace(
            timestamp=t0 + timedelta(seconds=i), actual_speed_kmh=float(speed[i]),
            accel_opening=float(accel[i]), brake_opening=float(brake[i]),
        )
        for i in range(len(accel))
    ]


def _accel_series() -> np.ndarray:
    accel = np.zeros(400)
    accel[100:105] = [10, 20, 30, 40, 50]
    accel[105:110] = [40, 30, 20, 15, 10]  # 戻し
    accel[110:120] = 0.0  # 不感帯未満（行は落ちる）
    accel[120:123] = [8.0, 9.0, 11.0]  # 新しい episode の踏み込み
    accel[123:130] = 0.0
    return accel


def test_direction_is_judged_before_filtering() -> None:
    accel = _accel_series()
    logs = _logs(accel, np.zeros(400))
    kwargs = dict(accel_deadband_pct=5.0, brake_deadband_pct=5.0,
                  accel_threshold_pct=5.0, brake_threshold_pct=5.0)
    (a_data, a_rel), _ = directed_regime_data(
        logs, np.array(["0:P"] * 400), np.array([""] * 400), **kwargs)
    by_y = dict(zip(a_data.y.tolist(), a_rel.tolist(), strict=True))
    # 全行で判定: 不感帯をはさんだ後の 8, 9, 11 は新しい episode なので踏み込み
    assert by_y[8.0] is False
    assert by_y[9.0] is False
    assert by_y[11.0] is False
    assert by_y[15.0] is True  # 戻し中
    # 絞ってから判定すると 8, 9 は直前の戻しの続きになってしまう（この違いを見ている）
    wrong = pedal_release(a_data.y, 5.0, 5.0)
    assert wrong[a_data.y.tolist().index(8.0)]


def test_matches_effective_regime_data() -> None:
    accel = _accel_series()
    brake = np.zeros(400)
    brake[200:210] = [10, 20, 30, 20, 10, 6, 7, 8, 9, 10]
    logs = _logs(accel, brake)
    patterns = np.array([f"{i // 100}:P" for i in range(400)])
    phases = np.array(["ph"] * 400)
    (a, _), (b, _) = directed_regime_data(
        logs, patterns, phases, accel_deadband_pct=5.0, brake_deadband_pct=5.0,
        accel_threshold_pct=5.0, brake_threshold_pct=3.0,
        accel_spec=DEFAULT_FEATURE_SPEC, brake_spec=DEFAULT_FEATURE_SPEC)
    ea, eb = effective_regime_data(
        logs, patterns, phases, accel_deadband_pct=5.0, brake_deadband_pct=5.0,
        spec=DEFAULT_FEATURE_SPEC)
    for mine, ref in ((a, ea), (b, eb)):
        np.testing.assert_array_equal(mine.x, ref.x)
        np.testing.assert_array_equal(mine.y, ref.y)
        np.testing.assert_array_equal(mine.pattern, ref.pattern)


def test_per_pedal_spec_changes_feature_columns() -> None:
    accel = _accel_series()
    brake = np.zeros(400)
    brake[200:230] = 20.0
    logs = _logs(accel, brake)
    brake_spec = replace(DEFAULT_FEATURE_SPEC, lookahead_horizons_s=(0.5, 1.0),
                         regime_horizon_s=1.0)
    (a, _), (b, _) = directed_regime_data(
        logs, np.array(["0:P"] * 400), np.array([""] * 400),
        accel_deadband_pct=5.0, brake_deadband_pct=5.0,
        accel_threshold_pct=5.0, brake_threshold_pct=3.0,
        accel_spec=DEFAULT_FEATURE_SPEC, brake_spec=brake_spec)
    assert a.x.shape[1] == len(DEFAULT_FEATURE_SPEC.feature_names())
    assert b.x.shape[1] == len(brake_spec.feature_names())
    assert a.x.shape[1] != b.x.shape[1]
