"""FF パラメータ解析（ff_explain.py）のユニットテスト。

FF の分岐（predict_effort の写し）が条件どおりに選ばれること、モード走行の FF が参照しない
パラメータ（engine_brake_decel_kmhs・ペダルゲイン・不感帯）を変えても effort が変わらないこと、
特徴の組み立て（要求加速度・学習域クリップ）が predict_effort と同じことを、定数を返す
模擬モデルで確かめる（手順 2 の pkl には依存しない）。
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from src.domain.model_training import FeatureSpec
from src.models.profile import FeedforwardParams
from tests.research import ff_explain as ffx

ACCEL_PRED = 12.0
BRAKE_PRED = 20.0


class _ConstModel:
    def __init__(self, value: float) -> None:
        self.value = value

    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.full(len(x), self.value)


def _model(clip: float | None = None) -> ffx.FFModel:
    return ffx.FFModel(
        path="",
        accel_model=_ConstModel(ACCEL_PRED),
        brake_model=_ConstModel(BRAKE_PRED),
        spec=FeatureSpec(),
        speed_clip_max=clip,
    )


def _params() -> FeedforwardParams:
    return FeedforwardParams(
        creep_speed_kmh=5.0,
        creep_rate_kmhs=0.2,
        stop_brake_opening_pct=30.0,
        engine_brake_decel_kmhs=1.8,
        coast_decel_speeds_kmh=(10.0, 50.0),
        coast_decel_kmhs=(2.0, 4.0),
        accel_deadband_pct=10.0,
        brake_deadband_pct=13.0,
    )


def _decide(p: FeedforwardParams, v0: float, a_req: float, near: float | None = None):
    return ffx.decide(p, v0, v0 if near is None else near, v0, a_req, ACCEL_PRED, BRAKE_PRED)


@pytest.mark.parametrize(
    ("v0", "a_req", "near", "branch", "effort"),
    [
        (0.0, 0.0, 0.0, ffx.BR_STOP, -30.0),
        (0.0, 1.0, 0.5, ffx.BR_ACCEL, ACCEL_PRED),  # 0.5s 先が動くなら停車保持を解く
        (3.0, 0.1, None, ffx.BR_CREEP, 0.0),
        (3.0, 0.5, None, ffx.BR_ACCEL, ACCEL_PRED),
        (30.0, 0.0, None, ffx.BR_ACCEL, ACCEL_PRED),
        (30.0, -1.5, None, ffx.BR_TAPER, ACCEL_PRED * (1.0 - 1.5 / 3.0)),  # 惰行 3.0 km/h/s
        (30.0, -3.0, None, ffx.BR_TAPER, 0.0),  # 境目ちょうどはテーパ側（effort 0）
        (30.0, -3.01, None, ffx.BR_BRAKE, -BRAKE_PRED),
        (3.0, -0.5, None, ffx.BR_LOW_BRAKE, -BRAKE_PRED),
    ],
)
def test_decide_branches(v0, a_req, near, branch, effort) -> None:
    got_branch, got_effort = _decide(_params(), v0, a_req, near)
    assert got_branch == branch
    assert got_effort == pytest.approx(effort)


def test_unused_params_do_not_change_effort() -> None:
    p = _params()
    q = replace(
        p,
        engine_brake_decel_kmhs=p.engine_brake_decel_kmhs * 3.0,
        pedal_gain_speeds_kmh=(10.0, 50.0),
        accel_gain_kmhs_per_pct=(1.0, 0.5),
        brake_gain_kmhs_per_pct=(0.5, 2.0),
        accel_deadband_pct=25.0,
        brake_deadband_pct=25.0,
    )
    for v0 in (0.0, 3.0, 20.0, 60.0, 120.0):
        for a_req in (-5.0, -2.5, -0.5, 0.0, 0.1, 2.0):
            assert _decide(p, v0, a_req) == _decide(q, v0, a_req)


def test_coast_curve_moves_taper_brake_boundary() -> None:
    p = _params()
    assert _decide(p, 30.0, -3.5)[0] == ffx.BR_BRAKE
    stronger = replace(p, coast_decel_kmhs=(3.0, 5.0))  # 30 km/h で 4.0
    assert _decide(stronger, 30.0, -3.5)[0] == ffx.BR_TAPER


def test_outputs_a_req_on_constant_accel_manifold() -> None:
    tr = ffx.constant_accel_trace(_model(), _params(), 60.0, np.array([-2.0, 0.0, 1.5]))
    assert tr.out.a_req == pytest.approx([-2.0, 0.0, 1.5])
    assert list(tr.branch) == [ffx.BR_TAPER, ffx.BR_ACCEL, ffx.BR_ACCEL]


def test_outputs_clip_shifts_trajectory_above_learning_range() -> None:
    out = ffx.outputs_from_points(
        _model(clip=100.0), [0.0], [110.0], [[110.5, 111.0, 112.0, 113.0]], [[109.5, 109.0]]
    )
    assert out.v0 == pytest.approx([100.0])
    assert out.v0_raw == pytest.approx([110.0])
    assert out.a_req == pytest.approx([0.0])  # 平行移動後 1.0s 先は 101 → 学習端 100 に飽和


def test_pedal_class_uses_deadbands() -> None:
    p = _params()
    effort = np.array([15.0, 5.0, 0.0, -10.0, -20.0])
    assert list(ffx.pedal_class(*ffx.effort_to_pedals(effort), p)) == ["A", "-", "-", "-", "B"]
