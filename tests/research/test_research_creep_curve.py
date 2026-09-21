"""クリープ加速カーブ推定（creep_curve.py）のユニットテスト。

ProblemReport_20260916 課題#2: 「両ペダル不感帯以下・STOP_SPEED_KMH<v<creep_speed_kmh・dv>0」の
条件でのビン化と、有効ビン2未満（未同定）のフォールバックを合成ログで確かめる。
"""

from __future__ import annotations

from dataclasses import replace
from datetime import UTC, datetime, timedelta

import pytest

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research.creep_curve import estimate_creep_accel_curve

PARAMS = FeedforwardParams(
    creep_speed_kmh=5.0, accel_deadband_pct=10.0, brake_deadband_pct=12.0,
)


def _ramp(v0: float, v1: float, dv_kmhs: float, dt: float) -> list[float]:
    """v0 から v1 まで、一定の加速度 dv_kmhs [km/h/s] で dt 刻みに上がる速度列（v1 を含む）。"""
    speeds = [v0]
    v = v0
    step = dv_kmhs * dt
    while v < v1:
        v = min(v1, v + step)
        speeds.append(v)
    return speeds


def _logs(
    speeds: list[float],
    *,
    dt: float = 0.02,
    accel_pct: float = 0.0,
    brake_pct: float = 0.0,
    session_id: str = "test",
) -> list[DriveLog]:
    origin = datetime(2026, 9, 17, tzinfo=UTC)
    return [
        DriveLog(
            id=i,
            session_id=session_id,
            timestamp=origin + timedelta(seconds=i * dt),
            ref_speed_kmh=None,
            actual_speed_kmh=v,
            accel_opening=accel_pct,
            brake_opening=brake_pct,
            accel_pos=0,
            brake_pos=0,
            accel_current=0.0,
            brake_current=0.0,
        )
        for i, v in enumerate(speeds)
    ]


def test_estimate_creep_accel_curve_identifies_speed_dependent_curve() -> None:
    """低速側 3.4 km/h/s・高速側 2.0 km/h/s の2段階クリープ加速が別ビンで区別できること。"""
    dt = 0.02
    speeds = _ramp(0.0, 2.5, 3.4, dt) + _ramp(2.5, 4.9, 2.0, dt)[1:]
    curve = estimate_creep_accel_curve(
        _logs(speeds, dt=dt), PARAMS, bin_kmh=1.0, min_bin_samples=5
    )
    assert curve.identified
    assert len(curve.speeds_kmh) >= 2
    # 末尾はクリープ平衡点（段2.5。ProblemReport_20260916）。本体（実測ビン）はそれより前
    assert curve.speeds_kmh[-1] == pytest.approx(PARAMS.creep_speed_kmh)
    assert curve.accel_kmhs[-1] == pytest.approx(0.0)
    body_speeds, body_accels = curve.speeds_kmh[:-1], curve.accel_kmhs[:-1]
    low = [g for v, g in zip(body_speeds, body_accels, strict=True) if v < 2.0]
    high = [g for v, g in zip(body_speeds, body_accels, strict=True) if v >= 3.0]
    assert low and high
    assert low == pytest.approx([3.4] * len(low), rel=0.05)
    assert high == pytest.approx([2.0] * len(high), rel=0.05)


def test_estimate_creep_accel_curve_appends_equilibrium_point_at_tail() -> None:
    """同定できたときだけ、結果の末尾に (creep_speed_kmh, 0.0) が付く（惰行カーブ側の先頭点と
    共有し、free_accel_at を creep_speed_kmh の前後で構造として連続にするため）。"""
    dt = 0.02
    speeds = _ramp(0.0, 4.9, 3.4, dt)
    curve = estimate_creep_accel_curve(
        _logs(speeds, dt=dt), PARAMS, bin_kmh=1.0, min_bin_samples=5
    )
    assert curve.identified
    assert curve.speeds_kmh[-1] == pytest.approx(PARAMS.creep_speed_kmh)
    assert curve.accel_kmhs[-1] == pytest.approx(0.0)
    assert curve.speeds_kmh == tuple(sorted(curve.speeds_kmh))  # 末尾を足しても昇順を保つ


def test_estimate_creep_accel_curve_excludes_samples_with_pedal_applied() -> None:
    dt = 0.02
    speeds = _ramp(0.0, 4.9, 3.4, dt)
    curve = estimate_creep_accel_curve(
        _logs(speeds, dt=dt, accel_pct=50.0), PARAMS, bin_kmh=1.0, min_bin_samples=5
    )
    assert not curve.identified
    assert curve.samples == 0


def test_estimate_creep_accel_curve_excludes_speeds_at_or_above_creep_speed() -> None:
    """creep_speed_kmh 以上（惰行域）は対象外。境界の STOP_SPEED_KMH 以下も対象外。"""
    dt = 0.02
    # 停車直後から高速（惰行域）まで一気に上げる。クリープ域（STOP<v<creep_speed）はごく僅か
    speeds = _ramp(0.0, 60.0, 20.0, dt)
    curve = estimate_creep_accel_curve(
        _logs(speeds, dt=dt), PARAMS, bin_kmh=1.0, min_bin_samples=50
    )
    assert not curve.identified  # クリープ域のサンプルが薄く、min_bin_samples に届かない


def test_estimate_creep_accel_curve_returns_unidentified_below_two_bins() -> None:
    dt = 0.02
    speeds = _ramp(0.0, 0.9, 3.4, dt)  # 1 ビン分（0〜1km/h）しか埋まらない
    curve = estimate_creep_accel_curve(
        _logs(speeds, dt=dt), PARAMS, bin_kmh=1.0, min_bin_samples=5
    )
    assert not curve.identified
    assert curve.speeds_kmh == () and curve.accel_kmhs == ()


def test_estimate_creep_accel_curve_ignores_stopped_samples() -> None:
    """速度がほぼ 0（STOP_SPEED_KMH 以下）で足踏みしているだけの区間は対象外。"""
    dt = 0.02
    stopped = [0.0] * 20  # dv=0 かつ STOP_SPEED_KMH 以下
    speeds = stopped + _ramp(0.0, 4.9, 3.4, dt)[1:]
    curve = estimate_creep_accel_curve(
        _logs(speeds, dt=dt), PARAMS, bin_kmh=1.0, min_bin_samples=5
    )
    assert curve.identified
    assert curve.samples < len(speeds) - 1  # 停車区間の分だけ対象サンプルが減っている


def test_estimate_creep_accel_curve_no_samples_is_unidentified() -> None:
    curve = estimate_creep_accel_curve([], PARAMS, bin_kmh=1.0, min_bin_samples=5)
    assert not curve.identified and curve.samples == 0


def test_estimate_creep_accel_curve_respects_current_deadbands() -> None:
    """不感帯は params（呼び出し時点の値）を使う。不感帯を上げると同じログでも除外されうる。"""
    dt = 0.02
    speeds = _ramp(0.0, 4.9, 3.4, dt)
    logs = _logs(speeds, dt=dt, accel_pct=0.3)
    lenient = estimate_creep_accel_curve(logs, PARAMS, bin_kmh=1.0, min_bin_samples=5)
    assert lenient.identified  # 0.3% は不感帯 10.0% 以下

    strict = replace(PARAMS, accel_deadband_pct=0.1)
    curve = estimate_creep_accel_curve(logs, strict, bin_kmh=1.0, min_bin_samples=5)
    assert not curve.identified  # 0.3% > 不感帯 0.1% なので全サンプル除外
    assert VEHICLE_STOP_SPEED_KMH < PARAMS.creep_speed_kmh  # 前提の確認
