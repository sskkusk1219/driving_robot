"""研究開発用ハーネス ペダルゲイン推定（pedal_gain.py）のユニットテスト。

スタブ車両モデル（加速度 = 惰行 + (開度 − 遊び) × ゲイン）を 0.1s 刻みで回したログから、
不感帯 = 遊びとしたときにスタブのゲインが推定されること、しきい値未満・開度が動いている間の
サンプルを使わないこと、同定できなかった側は本番推定の値を保つことを確かめる。
"""

from __future__ import annotations

from dataclasses import replace
from datetime import UTC, datetime, timedelta

import pytest

from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import pedal_gain as pgmod
from tests.research.vehicle import feedforward_params, opening_to_pulse


def _params() -> FeedforwardParams:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    return replace(
        feedforward_params(cfg),
        accel_deadband_pct=hwmod.STUB_ACCEL_PLAY_PCT,
        brake_deadband_pct=hwmod.STUB_BRAKE_PLAY_PCT,
    )


def _logs(
    params: FeedforwardParams, segments: list[tuple[float, float, float]]
) -> list[DriveLog]:
    """(アクセル%, ブレーキ%, 秒) の区間を順に踏んだスタブ車両の 0.1s 刻みログ。"""
    accel = hwmod.StubActuator("accel", connected=True)
    brake = hwmod.StubActuator("brake", connected=True)
    vehicle = hwmod.StubVehicle(accel=accel, brake=brake, params=params)
    origin = datetime(2026, 9, 11, tzinfo=UTC)
    speed = vehicle.advance(0.0, now=0.0)
    t = 0.0
    logs: list[DriveLog] = []
    for accel_pct, brake_pct, seconds in segments:
        accel.position = opening_to_pulse(accel_pct)
        brake.position = opening_to_pulse(brake_pct)
        for _ in range(round(seconds / 0.1)):
            t += 0.1
            speed = vehicle.advance(speed, now=t)
            logs.append(
                DriveLog(
                    id=len(logs),
                    session_id="test",
                    timestamp=origin + timedelta(seconds=t),
                    ref_speed_kmh=None,
                    actual_speed_kmh=speed,
                    accel_opening=accel_pct,
                    brake_opening=brake_pct,
                    accel_pos=accel.position,
                    brake_pos=brake.position,
                    accel_current=0.0,
                    brake_current=0.0,
                )
            )
    return logs


# 40% で 120 km/h 付近まで加速してから、不感帯 +0.5/+2/+4% のブレーキを 6s ずつ保持
BRAKE_HOLDS = [(40.0, 0.0, 25.0), (0.0, 8.5, 6.0), (0.0, 10.0, 6.0), (0.0, 12.0, 6.0)]


def test_brake_gain_matches_stub_vehicle() -> None:
    params = _params()
    curve = pgmod.estimate_gain_curve(
        _logs(params, BRAKE_HOLDS), params, is_accel=False, min_offset_pct=0.5
    )
    assert curve.identified
    assert len(curve.speeds_kmh) >= 2
    assert curve.gains == pytest.approx(
        [hwmod.STUB_BRAKE_GAIN_KMHS_PER_PCT] * len(curve.gains), rel=0.05
    )


def test_accel_gain_matches_stub_vehicle() -> None:
    params = _params()
    curve = pgmod.estimate_gain_curve(
        _logs(params, BRAKE_HOLDS), params, is_accel=True, min_offset_pct=0.5
    )
    assert curve.identified
    # 低速はスタブのクリープ引き戻しが効くので、惰行カーブどおりに動く 20 km/h 以上で比べる
    fast = [g for v, g in zip(curve.speeds_kmh, curve.gains, strict=True) if v >= 20.0]
    assert len(fast) >= 2
    assert fast == pytest.approx([hwmod.STUB_ACCEL_GAIN_KMHS_PER_PCT] * len(fast), rel=0.05)


def test_samples_below_min_offset_are_not_used() -> None:
    params = _params()
    logs = _logs(params, [(40.0, 0.0, 25.0), (0.0, 8.3, 10.0)])  # 不感帯 +0.3%
    curve = pgmod.estimate_gain_curve(logs, params, is_accel=False, min_offset_pct=0.5)
    assert curve.samples == 0
    assert not curve.identified


def test_samples_while_opening_moves_are_not_used() -> None:
    params = _params()
    ramp = [(0.0, 9.0 + 0.3 * i, 0.1) for i in range(40)]  # 1s で 3% ずつ踏み増し続ける
    curve = pgmod.estimate_gain_curve(
        _logs(params, [(40.0, 0.0, 25.0), *ramp]), params, is_accel=False, min_offset_pct=0.5
    )
    assert curve.samples == 0


def test_apply_replaces_only_identified_side() -> None:
    params = replace(
        _params(),
        pedal_gain_speeds_kmh=(15.0, 25.0, 35.0),
        accel_gain_kmhs_per_pct=(1.0, 2.0, 3.0),
        brake_gain_kmhs_per_pct=(),
    )
    brake = pgmod.GainCurve(speeds_kmh=(20.0, 30.0), gains=(0.5, 0.6), samples=40)
    accel = pgmod.GainCurve(speeds_kmh=(), gains=(), samples=3)

    applied = pgmod.apply_pedal_gains(params, accel=accel, brake=brake)

    assert applied.pedal_gain_speeds_kmh == (15.0, 20.0, 25.0, 30.0, 35.0)
    # アクセルは本番推定の値を共通グリッドへ補間、ブレーキは研究側の推定（範囲外は端点）
    assert applied.accel_gain_kmhs_per_pct == pytest.approx((1.0, 1.5, 2.0, 2.5, 3.0))
    assert applied.brake_gain_kmhs_per_pct == pytest.approx((0.5, 0.5, 0.55, 0.6, 0.6))


def test_apply_without_identified_curves_keeps_params() -> None:
    params = _params()
    empty = pgmod.GainCurve(speeds_kmh=(), gains=(), samples=0)
    assert pgmod.apply_pedal_gains(params, accel=empty, brake=empty) == params
