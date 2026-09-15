"""簡易車両モデル（vehicle_sim.py）のユニットテスト。

不感帯の中の開度は惰行と同じ、表の補間とゲイン比例、ログからの同定が既知の応答を当てること、
むだ時間で効き始めが遅れること、同じモデルで作ったログの再生誤差が 0 になることを確かめる。
"""

from __future__ import annotations

import numpy as np
import pytest

from src.models.profile import FeedforwardParams, coast_decel_at
from tests.research import vehicle_sim as vs

SPEEDS = (5.0, 15.0, 25.0, 35.0, 45.0, 55.0, 65.0, 75.0, 85.0, 95.0, 105.0, 115.0, 125.0, 135.0)
COAST = (1.6, 1.73, 2.58, 3.195, 3.575, 3.57, 3.11, 2.54, 1.83, 1.6, 1.6, 1.6, 1.6, 1.6)
PARAMS = FeedforwardParams(
    creep_speed_kmh=4.944,
    creep_rate_kmhs=0.155,
    coast_decel_speeds_kmh=SPEEDS,
    coast_decel_kmhs=COAST,
    accel_deadband_pct=10.0,
    brake_deadband_pct=13.68,
)
GAIN = 0.5  # 合成車両のペダルゲイン [km/h/s per %]


def _model(delay_s: float = 0.0, lag_s: float = 0.0) -> vs.VehicleModel:
    resp = vs.proportional_response("比例", (0.0, 140.0), (GAIN, GAIN))
    return vs.VehicleModel("合成", PARAMS, resp, resp, delay_s=delay_s, lag_s=lag_s)


@pytest.mark.parametrize("v", [3.0, 5.0, 37.0, 90.0, 150.0])
def test_coast_decel_matches_production(v: float) -> None:
    assert float(vs.coast_decel(PARAMS, v)) == pytest.approx(coast_decel_at(PARAMS, v))


def test_opening_inside_deadband_is_coast() -> None:
    m = _model()
    assert float(m.pedal_accel(60.0, 9.9, 13.6)) == 0.0
    assert float(m.pedal_accel(60.0, 14.0, 0.0)) == pytest.approx(GAIN * 4.0)
    assert float(m.pedal_accel(60.0, 0.0, 15.68)) == pytest.approx(-GAIN * 2.0)


def test_response_interpolates_speed_and_opening() -> None:
    resp = vs.proportional_response("比例", (20.0, 60.0), (1.0, 3.0))
    assert float(resp.at(40.0, 2.25)) == pytest.approx(2.0 * 2.25)
    assert float(resp.at(0.0, 1.0)) == pytest.approx(1.0)  # 速度は端でクランプ
    assert float(resp.at(40.0, -3.0)) == 0.0


def test_scale_response_multiplies_every_point() -> None:
    resp = vs.proportional_response("比例", (20.0, 60.0), (1.0, 3.0))
    scaled = vs.scale_response(resp, 0.8)
    assert float(scaled.at(60.0, 4.0)) == pytest.approx(float(resp.at(60.0, 4.0)) * 0.8)
    assert float(scaled.at(20.0, 0.0)) == 0.0


def test_creep_region_is_continuous() -> None:
    m = _model()
    assert float(m.coast_accel(0.0)) == pytest.approx(PARAMS.creep_rate_kmhs)
    assert float(m.coast_accel(80.0)) == pytest.approx(-coast_decel_at(PARAMS, 80.0))


def _synthetic_log(model: vs.VehicleModel, accel: np.ndarray, brake: np.ndarray) -> vs.LogSeries:
    n = len(accel)
    speed = np.empty(n)
    v = 100.0  # ブレーキの終わりまでクリープ域（5 km/h 未満）に入らない速度から始める
    for i in range(n):
        speed[i] = v
        j = max(0, i - round(model.delay_s / vs.LOG_DT_S))
        a = float(model.coast_accel(v)) + float(model.pedal_accel(v, accel[j], brake[j]))
        v = max(0.0, v + a * vs.LOG_DT_S)
    return vs.LogSeries("合成", np.arange(n) * vs.LOG_DT_S, speed, accel, brake)


def _step_openings() -> tuple[np.ndarray, np.ndarray]:
    """アクセル 13%（不感帯 + 3%）を 8s 保持 → 離して 4s → ブレーキ 16%（+2.32%）を 8s 保持。"""
    accel = np.concatenate([np.full(80, 13.0), np.zeros(120)])
    brake = np.concatenate([np.zeros(120), np.full(80, 16.0)])
    return accel, brake


def test_identify_recovers_known_response() -> None:
    model = _model(delay_s=0.5)
    log = _synthetic_log(model, *_step_openings())
    accel_resp, counts = vs.identify_response("アクセル", [log], PARAMS, is_accel=True)
    brake_resp, _ = vs.identify_response("ブレーキ", [log], PARAMS, is_accel=False)
    assert counts.sum() > 0
    v = float(np.median(log.speed))
    assert float(accel_resp.at(v, 3.0)) == pytest.approx(GAIN * 3.0, rel=0.05)
    assert float(brake_resp.at(v, 2.32)) == pytest.approx(GAIN * 2.32, rel=0.05)


def test_identify_without_rows_raises() -> None:
    log = vs.LogSeries("空", np.arange(50) * 0.1, np.full(50, 40.0), np.zeros(50), np.zeros(50))
    with pytest.raises(ValueError):
        vs.identify_response("アクセル", [log], PARAMS, is_accel=True)


def test_replay_of_same_model_has_no_error() -> None:
    model = _model(delay_s=0.5)
    log = _synthetic_log(model, *_step_openings())
    result = vs.replay(model, log, horizon_s=3.0, every_s=1.0)
    assert result.error_kmh.size > 0
    assert np.max(np.abs(result.error_kmh)) < 1e-9
    assert set(result.pedal) <= {vs.PEDAL_ACCEL, vs.PEDAL_BRAKE, vs.PEDAL_COAST}


def test_delay_postpones_pedal_effect() -> None:
    def late(i: int, v: float) -> tuple[float, float]:
        """5 周期目（i = 4）からアクセル 20% を踏む。"""
        return (20.0, 0.0) if i >= 4 else (0.0, 0.0)

    fast = vs.simulate(_model(), 20, late, v_start=60.0)
    slow = vs.simulate(_model(delay_s=0.5), 20, late, v_start=60.0)
    assert fast.speed[6] > slow.speed[6]  # 遅れなしは 5 周期目から加速
    assert np.all(np.diff(slow.speed[:14]) < 0)  # 0.5s 遅れは 14 周期目まで惰行のまま


def test_simulate_stops_above_speed_limit() -> None:
    run = vs.simulate(_model(), 1000, lambda i, v: (80.0, 0.0), v_start=130.0, stop_above_kmh=140.0)
    assert run.stopped_at_s is not None
    assert run.speed[-1] > 140.0
    assert run.end_s == pytest.approx(run.stopped_at_s)
