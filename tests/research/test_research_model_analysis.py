"""逆モデル分析（model_analysis.py）のユニットテスト。

合成した走行ログから、学習セットが本番 train_inverse_model と同じ規則（dv_1.0 の符号で分割・
ブレーキ不感帯未満のラベルは 0）で作られること、pattern 列が行とずれないこと、指標の計算が
本番 _metrics と一致すること、パターン単位の分割検証が全行を予測することを確かめる。
"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta

import numpy as np
import pytest

from src.domain.model_training import DEFAULT_FEATURE_SPEC, _make_estimator, _metrics
from src.models.drive_log import DriveLog
from tests.research import model_analysis as ma
from tests.research.mode_drive import split_effort

ACCEL_PCT = 15.0
BRAKE_DB = 13.68


def _logs() -> tuple[list[DriveLog], np.ndarray, np.ndarray]:
    """0.1s 刻みの合成ログ。

    前半 10s は +2 km/h/s で加速（アクセル 15%）、後半 10s は減速（ブレーキ 10% → 20%）。
    """
    origin = datetime(2026, 9, 12, tzinfo=UTC)
    logs, patterns, phases = [], [], []
    for i in range(200):
        if i < 100:
            speed, accel, brake, pattern = 10.0 + 0.2 * i, ACCEL_PCT, 0.0, "1:ACCEL_SWEEP"
        else:
            speed, accel, pattern = 29.8 - 0.2 * (i - 99), 0.0, "2:BRAKE_HOLD"
            brake = 10.0 if i < 150 else 20.0
        logs.append(DriveLog(
            id=i, session_id="s", timestamp=origin + timedelta(seconds=0.1 * i),
            ref_speed_kmh=None, actual_speed_kmh=speed, accel_opening=accel,
            brake_opening=brake, accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
        ))
        patterns.append(pattern)
        phases.append("DRIVE_ACCEL" if i < 100 else "BRAKE_HOLD")
    return logs, np.array(patterns), np.array(phases)


def _regimes() -> tuple[ma.RegimeData, ma.RegimeData]:
    logs, patterns, phases = _logs()
    return ma.build_training_set(
        logs, patterns, phases, accel_deadband_pct=10.0, brake_deadband_pct=BRAKE_DB
    )


def test_split_by_dv_sign() -> None:
    accel, brake = _regimes()
    col = DEFAULT_FEATURE_SPEC.regime_col()
    assert (accel.x[:, col] >= 0.0).all()
    assert (brake.x[:, col] < 0.0).all()
    assert len(accel.y) + len(brake.y) == 200 - 30 - 10  # 先読み 3s・過去 1s の端を除く


def test_brake_labels_below_deadband_become_zero() -> None:
    _, brake = _regimes()
    assert set(np.unique(brake.y)) <= {0.0, 20.0}
    assert 20.0 in brake.y
    assert (brake.y[brake.pattern == "2:BRAKE_HOLD"][:10] == 0.0).all()  # 10% 区間は 0


def test_pattern_column_stays_aligned_with_rows() -> None:
    accel, brake = _regimes()
    assert (accel.y[accel.pattern == "1:ACCEL_SWEEP"] == ACCEL_PCT).all()
    # 加速区間の終わりは先読みが減速に入るのでブレーキ側に来る（ブレーキ開度 0）
    assert (brake.y[brake.pattern == "1:ACCEL_SWEEP"] == 0.0).all()
    assert list(dict.fromkeys(accel.kind)) == ["ACCEL_SWEEP"]


def test_score_matches_production_metrics() -> None:
    rng = np.random.default_rng(0)
    x = rng.normal(size=(80, 9))
    y = x[:, 0] * 3.0 + rng.normal(size=80)
    model = _make_estimator().fit(x, y)
    assert ma.metrics_match(_metrics(model, x, y), ma.score(y, model.predict(x)))
    assert not ma.metrics_match(_metrics(model, x, y), ma.score(y, model.predict(x) + 1.0))


def test_out_of_pattern_prediction_covers_all_rows() -> None:
    _, brake = _regimes()
    assert len(set(brake.pattern)) == 2
    oof = ma.predict_out_of_pattern(brake)
    assert oof.shape == brake.y.shape
    assert np.isfinite(oof).all()


def test_out_of_pattern_needs_two_patterns() -> None:
    accel, _ = _regimes()
    assert len(set(accel.pattern)) == 1
    assert np.isnan(ma.predict_out_of_pattern(accel)).all()


@pytest.mark.parametrize(
    ("value", "expected"), [(0.0, False), (0.5, False), (0.6, True), (13.6, True), (13.68, False)]
)
def test_in_deadband(value: float, expected: bool) -> None:
    assert bool(ma.in_deadband(np.array([value]), BRAKE_DB)[0]) is expected


# ── 2-2 ──────────────────────────────────────────────────────────────


def _outputs(
    a_req: list[float], accel_raw: list[float], brake_raw: list[float]
) -> ma.ModelOutputs:
    n = len(a_req)
    speed = np.full(n, 50.0)
    return ma.ModelOutputs(
        t=np.arange(n) * 0.1, v0_raw=speed, near=speed, v0=speed,
        a_req=np.array(a_req, dtype=float),
        accel_raw=np.array(accel_raw, dtype=float),
        brake_raw=np.array(brake_raw, dtype=float),
    )


def test_raw_openings_follow_training_split() -> None:
    out = _outputs([1.0, 0.0, -0.5, -2.0], [12.0, -1.0, 9.0, 9.0], [3.0, 3.0, -1.0, 20.0])
    accel, brake = ma.raw_openings(out)
    assert list(accel) == [12.0, 0.0, 0.0, 0.0]  # a_req = 0 はアクセル側（負の出力は 0）
    assert list(brake) == [0.0, 0.0, 0.0, 20.0]


@pytest.mark.parametrize("effort", [90.0, 5.0, 0.0, -0.1, -30.0, -95.0])
def test_ff_openings_match_mode_drive_split(effort: float) -> None:
    accel, brake = ma.ff_openings(np.array([effort]), 80.0, 80.0)
    assert (float(accel[0]), float(brake[0])) == split_effort(effort, 80.0, 80.0)


def test_switch_count_ignores_coast_between_pedals() -> None:
    accel = np.array([5.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0])
    brake = np.array([0.0, 0.0, 4.0, 0.0, 0.0, 2.0, 2.0])
    assert ma.switch_count(accel, brake) == 3


def test_series_csv_columns(tmp_path) -> None:
    path = tmp_path / "series.csv"
    ma.write_series_csv(path, np.array([0.0, 0.1]), np.array([0.0, 1.5]),
                        np.array([0.0, 12.0]), np.array([30.5, 0.0]))
    lines = path.read_text(encoding="utf-8").splitlines()
    assert lines[0] == "time_s,ref_speed_kmh,accel_opening,brake_opening"
    assert lines[2] == "0.1,1.500,12.000,0.000"
