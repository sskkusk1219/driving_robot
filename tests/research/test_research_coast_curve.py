"""惰行減速カーブの低速端推定（coast_curve.py）のユニットテスト。

ProblemReport_20260916 段2.5: 「両ペダル不感帯以下・sp>creep_speed_kmh・dv<0」の条件でのビン化
（低速は細ビン・高速は本番と同じ 10km/h ビン）、先頭のクリープ平衡点、サンプル不足・有効点不足の
フォールバックを合成ログで確かめる。
"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta

import pytest

from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research.coast_curve import estimate_coast_decel_curve

PARAMS = FeedforwardParams(
    creep_speed_kmh=5.0, accel_deadband_pct=10.0, brake_deadband_pct=12.0,
)


def _decel_ramp(v0: float, v1: float, decel_kmhs: float, dt: float) -> list[float]:
    """v0 から v1（v1 < v0）まで、一定の減速度 decel_kmhs [km/h/s] で dt 刻みに下がる速度列。"""
    speeds = [v0]
    v = v0
    step = decel_kmhs * dt
    while v > v1:
        v = max(v1, v - step)
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
    origin = datetime(2026, 9, 18, tzinfo=UTC)
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


def test_estimate_coast_decel_curve_uses_fine_bins_below_low_max_and_coarse_bins_above() -> None:
    """低速側（5〜15km/h）は 1km/h 幅・高速側（15km/h 以上）は 10km/h 幅で採られる。"""
    dt = 0.02
    # 5→15km/h をゆっくり（低速側の各1km/hビンに十分点数）、15→65km/h を速く（高速側5ビン分）
    speeds = (
        _decel_ramp(15.0, 5.0, 2.0, dt) + _decel_ramp(65.0, 15.0, 10.0, dt)[1:]
    )
    curve = estimate_coast_decel_curve(
        _logs(speeds, dt=dt), PARAMS,
        low_bin_kmh=1.0, low_max_kmh=15.0, low_min_bin_samples=5,
    )
    assert curve.identified
    # 先頭はクリープ平衡点
    assert curve.speeds_kmh[0] == pytest.approx(PARAMS.creep_speed_kmh)
    assert curve.decels_kmh[0] == pytest.approx(0.0)

    body = list(zip(curve.speeds_kmh[1:], curve.decels_kmh[1:], strict=True))
    low = [(v, d) for v, d in body if v < 15.0]
    high = [(v, d) for v, d in body if v >= 15.0]
    assert low  # 低速側が採れている
    assert high  # 高速側も採れている
    # 低速側は 1km/h 刻みのビン中心（5.5, 6.5, ...）
    low_speeds = sorted(v for v, _ in low)
    assert all(abs((v - 0.5) - round(v - 0.5)) < 1e-6 for v in low_speeds)
    # 高速側は 10km/h 刻みのビン中心（0起点。25.0, 35.0, ...）
    high_speeds = sorted(v for v, _ in high)
    assert all(abs((v - 5.0) % 10.0) < 1e-6 for v in high_speeds)
    # 低速側の減速度はほぼ一定（2.0）
    assert [d for _, d in low] == pytest.approx([2.0] * len(low), rel=0.1)
    # 高速側は 0 起点の 10km/h ビン境界なので、low_max_kmh をまたぐ最初のビン（中心 15.0。
    # ビン境界 [10,20) が低速レジームの一部を含む）だけは低速データと混ざる。それより上の
    # ビン（境界が完全に高速レジームの内側＝中心 25.0 以上）は純粋に 10.0 になるはず
    pure_high = [(v, d) for v, d in high if v >= 25.0]
    assert pure_high
    assert [d for _, d in pure_high] == pytest.approx([10.0] * len(pure_high), rel=0.1)


def test_estimate_coast_decel_curve_does_not_duplicate_speed_range() -> None:
    """低速側と高速側で速度帯が重複しない（高速側はビン中心が low_max_kmh 以上のみ）。"""
    dt = 0.02
    speeds = _decel_ramp(65.0, 5.0, 5.0, dt)
    curve = estimate_coast_decel_curve(
        _logs(speeds, dt=dt), PARAMS,
        low_bin_kmh=1.0, low_max_kmh=15.0, low_min_bin_samples=5,
    )
    assert curve.identified
    body_speeds = curve.speeds_kmh[1:]  # 先頭のクリープ平衡点を除く
    assert any(v < 15.0 for v in body_speeds)  # 低速側が採れている
    assert any(v >= 15.0 for v in body_speeds)  # 高速側も採れている
    # 重複無し: 各ビン中心はちょうど一度だけ現れる
    assert len(curve.speeds_kmh) == len(set(curve.speeds_kmh))


def test_estimate_coast_decel_curve_drops_bins_with_too_few_samples() -> None:
    """低速ビンのサンプル数が min_bin_samples 未満なら捨てる（高速側は十分な点数で残る）。"""
    dt = 0.02
    high = _decel_ramp(65.0, 15.0, 5.0, dt)  # 十分な点数（高速側の各10km/hビンに約100点）
    low = _decel_ramp(15.0, 5.0, 50.0, dt)[1:]  # 各1km/hビンに約1点しかない速さで駆け抜ける
    speeds = high + low
    curve = estimate_coast_decel_curve(
        _logs(speeds, dt=dt), PARAMS,
        low_bin_kmh=1.0, low_max_kmh=15.0, low_min_bin_samples=50,
    )
    assert curve.identified
    body_speeds = curve.speeds_kmh[1:]  # 先頭のクリープ平衡点を除く
    assert all(v >= 15.0 for v in body_speeds)  # 低速ビンは全部サンプル不足で捨てられている


def test_estimate_coast_decel_curve_unidentified_below_two_valid_bins() -> None:
    """有効ビン（低速+高速の合計）が2個未満なら未同定（空タプル）。"""
    dt = 0.02
    # 5〜7km/h の2km/h幅しか通らない → 1km/hビンが2個(5-6,6-7)埋まるかどうかギリギリなので
    # サンプル数を極端に少なくして「有効ビン0〜1個」を作る
    speeds = _decel_ramp(7.0, 5.0, 1.0, dt)
    curve = estimate_coast_decel_curve(
        _logs(speeds, dt=dt), PARAMS,
        low_bin_kmh=1.0, low_max_kmh=15.0, low_min_bin_samples=100,
    )
    assert not curve.identified
    assert curve.speeds_kmh == () and curve.decels_kmh == ()


def test_estimate_coast_decel_curve_excludes_samples_with_pedal_applied() -> None:
    dt = 0.02
    speeds = _decel_ramp(65.0, 5.0, 5.0, dt)
    curve = estimate_coast_decel_curve(
        _logs(speeds, dt=dt, brake_pct=50.0), PARAMS,
        low_bin_kmh=1.0, low_max_kmh=15.0, low_min_bin_samples=5,
    )
    assert not curve.identified
    assert curve.samples == 0


def test_estimate_coast_decel_curve_no_samples_is_unidentified() -> None:
    curve = estimate_coast_decel_curve(
        [], PARAMS, low_bin_kmh=1.0, low_max_kmh=15.0, low_min_bin_samples=5
    )
    assert not curve.identified and curve.samples == 0


def test_estimate_coast_decel_curve_excludes_speeds_at_or_below_creep_speed() -> None:
    """creep_speed_kmh 以下（クリープ域・停車）は m_eng の条件（sp > creep_speed_kmh）で対象外。"""
    dt = 0.02
    speeds = _decel_ramp(5.0, 0.0, 1.0, dt)  # creep_speed_kmh(5.0) ちょうど以下しか通らない
    curve = estimate_coast_decel_curve(
        _logs(speeds, dt=dt), PARAMS,
        low_bin_kmh=1.0, low_max_kmh=15.0, low_min_bin_samples=5,
    )
    assert not curve.identified
    assert curve.samples == 0
