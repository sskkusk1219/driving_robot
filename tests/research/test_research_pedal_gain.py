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
from tests.research.ff_params import ResearchFFParams
from tests.research.vehicle import feedforward_params, opening_to_pulse

# 既定の config_testVehicle.yaml と同じクリープ域ビン化パラメータ
CREEP_BIN_KMH = 1.0
CREEP_MIN_BIN_SAMPLES = 5


def _params() -> FeedforwardParams:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    return replace(
        feedforward_params(cfg),
        accel_deadband_pct=hwmod.STUB_ACCEL_PLAY_PCT,
        brake_deadband_pct=hwmod.STUB_BRAKE_PLAY_PCT,
    )


def _research() -> ResearchFFParams:
    """空の ResearchFFParams を返す（config のクリープ加速カーブを混入させない）。

    estimate_gain_curve は基準に free_accel_at(params, research, v) を使う。
    research にクリープ加速カーブ（creep_accel_speeds_kmh/creep_accel_kmhs）が
    入っていると、手順2の実機走行で自動保存された実測値が基準に混ざり、
    StubVehicle の物理（惰行 = creep_rate_kmhs の定数）とずれて偽のゲインが出る
    （実測では期待 0.5 に対し 1.86/1.57/1.18/0.79 が出ており、差 1.36/1.07/0.68/0.29 は
    creep_accel_kmhs − creep_rate_kmhs に一致）。
    空にすると free_accel_at は creep_rate_kmhs にフォールバックし、
    スタブの物理と一致するため、実機走行が config を書き換えてもテストが壊れない。
    """
    return ResearchFFParams()


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
        _logs(params, BRAKE_HOLDS), params, _research(), is_accel=False, min_offset_pct=0.5,
        creep_bin_kmh=CREEP_BIN_KMH, creep_min_bin_samples=CREEP_MIN_BIN_SAMPLES,
    )
    assert curve.identified
    assert len(curve.speeds_kmh) >= 2
    assert curve.gains == pytest.approx(
        [hwmod.STUB_BRAKE_GAIN_KMHS_PER_PCT] * len(curve.gains), rel=0.05
    )


def test_accel_gain_matches_stub_vehicle() -> None:
    params = _params()
    curve = pgmod.estimate_gain_curve(
        _logs(params, BRAKE_HOLDS), params, _research(), is_accel=True, min_offset_pct=0.5,
        creep_bin_kmh=CREEP_BIN_KMH, creep_min_bin_samples=CREEP_MIN_BIN_SAMPLES,
    )
    assert curve.identified
    # 低速はスタブのクリープ引き戻しが効くので、惰行カーブどおりに動く 20 km/h 以上で比べる
    fast = [g for v, g in zip(curve.speeds_kmh, curve.gains, strict=True) if v >= 20.0]
    assert len(fast) >= 2
    assert fast == pytest.approx([hwmod.STUB_ACCEL_GAIN_KMHS_PER_PCT] * len(fast), rel=0.05)


def test_samples_below_min_offset_are_not_used() -> None:
    params = _params()
    logs = _logs(params, [(40.0, 0.0, 25.0), (0.0, 8.3, 10.0)])  # 不感帯 +0.3%
    curve = pgmod.estimate_gain_curve(
        logs, params, _research(), is_accel=False, min_offset_pct=0.5,
        creep_bin_kmh=CREEP_BIN_KMH, creep_min_bin_samples=CREEP_MIN_BIN_SAMPLES,
    )
    assert curve.samples == 0
    assert not curve.identified


def test_samples_while_opening_moves_are_not_used() -> None:
    params = _params()
    ramp = [(0.0, 9.0 + 0.3 * i, 0.1) for i in range(40)]  # 1s で 3% ずつ踏み増し続ける
    curve = pgmod.estimate_gain_curve(
        _logs(params, [(40.0, 0.0, 25.0), *ramp]), params, _research(), is_accel=False,
        min_offset_pct=0.5, creep_bin_kmh=CREEP_BIN_KMH,
        creep_min_bin_samples=CREEP_MIN_BIN_SAMPLES,
    )
    assert curve.samples == 0


# ── クリープ域（ProblemReport_20260916 課題#2） ──────────────────────


def test_brake_gain_identified_from_creep_domain_samples() -> None:
    """クリープ域（速度 < creep_speed_kmh）のブレーキ保持サンプルから低速ブレーキゲインが
    同定できること（従来は sp > creep_speed_kmh で構造的に除外されていた）。"""
    params = _params()
    # クリープ域まで自走させてから、不感帯 +1% のブレーキで停車まで保持する
    logs = _logs(params, [(0.0, 0.0, 20.0), (0.0, 9.0, 15.0)])
    curve = pgmod.estimate_gain_curve(
        logs, params, _research(), is_accel=False, min_offset_pct=0.5,
        creep_bin_kmh=CREEP_BIN_KMH, creep_min_bin_samples=CREEP_MIN_BIN_SAMPLES,
    )
    assert curve.identified
    low = [(v, g) for v, g in zip(curve.speeds_kmh, curve.gains, strict=True)
           if v < params.creep_speed_kmh]
    assert low  # クリープ域のビンが少なくとも1つ同定できている
    assert [g for _, g in low] == pytest.approx(
        [hwmod.STUB_BRAKE_GAIN_KMHS_PER_PCT] * len(low), rel=0.1
    )


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
