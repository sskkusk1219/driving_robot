"""研究段階のみの FF パラメータ（ff_params.py）のユニットテスト。

ProblemReport_20260916 課題#2: `free_accel_at` の単一ソース性（クリープ域/惰行域の切替・
線形補間・端点クランプ・未同定フォールバック）を確かめる。
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from src.models.profile import FeedforwardParams
from tests.research import config as cfgmod
from tests.research.ff_params import (
    ResearchFFParams,
    creep_accel_at,
    free_accel_at,
    research_ff_params,
)


def _params(**kw: float) -> FeedforwardParams:
    return replace(
        FeedforwardParams(),
        creep_speed_kmh=5.0,
        creep_rate_kmhs=0.19,
        coast_decel_speeds_kmh=(5.0, 15.0, 25.0),
        coast_decel_kmhs=(2.0, 4.0, 6.0),
        **kw,
    )


def test_creep_accel_at_returns_none_when_unidentified() -> None:
    research = ResearchFFParams()
    assert creep_accel_at(research, 2.0) is None


_SAMPLE_CURVE: dict[str, tuple[float, ...]] = {
    "creep_accel_speeds_kmh": (0.0, 2.0, 4.0),
    "creep_accel_kmhs": (3.4, 2.0, 0.5),
}


def test_creep_accel_at_interpolates_linearly() -> None:
    research = ResearchFFParams(**_SAMPLE_CURVE)
    assert creep_accel_at(research, 1.0) == pytest.approx(2.7)
    assert creep_accel_at(research, 3.0) == pytest.approx(1.25)


def test_creep_accel_at_clamps_at_endpoints() -> None:
    research = ResearchFFParams(**_SAMPLE_CURVE)
    assert creep_accel_at(research, -1.0) == pytest.approx(3.4)
    assert creep_accel_at(research, 10.0) == pytest.approx(0.5)


def test_free_accel_at_below_creep_speed_falls_back_to_creep_rate_when_unidentified() -> None:
    params = _params()
    research = ResearchFFParams()
    assert free_accel_at(params, research, 2.0) == pytest.approx(params.creep_rate_kmhs)


def test_free_accel_at_below_creep_speed_uses_identified_curve() -> None:
    params = _params()
    research = ResearchFFParams(**_SAMPLE_CURVE)
    assert free_accel_at(params, research, 1.0) == pytest.approx(2.7)


def test_free_accel_at_above_creep_speed_uses_coast_decel_as_negative() -> None:
    params = _params()
    research = ResearchFFParams()
    # coast_decel_at(60.0) は端点クランプで速度グリッド上限（25km/h）の値 6.0 になる
    assert free_accel_at(params, research, 60.0) == pytest.approx(-6.0)
    assert free_accel_at(params, research, 10.0) == pytest.approx(-3.0)  # 5〜15km/h の線形補間


def test_free_accel_at_boundary_is_continuous_with_coast_side() -> None:
    """creep_speed_kmh ちょうどは惰行側（coast_decel_at）を使う（v < creep_speed が境界）。"""
    params = _params()
    research = ResearchFFParams()
    assert free_accel_at(params, research, params.creep_speed_kmh) == pytest.approx(-2.0)


def test_free_accel_at_is_continuous_across_creep_speed_boundary() -> None:
    """段2.5（ProblemReport_20260916）: 惰行カーブの先頭とクリープカーブの末尾が同じ
    (creep_speed_kmh, 0.0) を共有するとき、free_accel_at は creep_speed_kmh の前後どちらから
    近づいても 0 に収束する（coast_curve.estimate_coast_decel_curve /
    creep_curve.estimate_creep_accel_curve が実際に置く点と同じ形）。
    """
    creep_speed = 4.793
    params = replace(
        FeedforwardParams(),
        creep_speed_kmh=creep_speed,
        coast_decel_speeds_kmh=(creep_speed, 10.0, 20.0),
        coast_decel_kmhs=(0.0, 3.0, 5.0),
    )
    research = ResearchFFParams(
        creep_accel_speeds_kmh=(1.0, 3.0, creep_speed),
        creep_accel_kmhs=(1.7, 0.5, 0.0),
    )
    just_below = free_accel_at(params, research, creep_speed - 1e-6)
    at_boundary = free_accel_at(params, research, creep_speed)
    just_above = free_accel_at(params, research, creep_speed + 1e-6)
    assert just_below == pytest.approx(0.0, abs=1e-3)
    assert at_boundary == pytest.approx(0.0, abs=1e-9)
    assert just_above == pytest.approx(0.0, abs=1e-3)


def test_free_accel_at_unidentified_curves_keeps_previous_behavior() -> None:
    """カーブが未同定（空タプル）のときの従来動作は変わらない（フォールバック/端点クランプ）。"""
    params = _params()
    research = ResearchFFParams()
    assert free_accel_at(params, research, 2.0) == pytest.approx(params.creep_rate_kmhs)
    assert free_accel_at(params, research, params.creep_speed_kmh) == pytest.approx(-2.0)
    assert free_accel_at(params, research, 60.0) == pytest.approx(-6.0)


def test_research_ff_params_maps_yaml() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.feedforward.creep_accel_speeds_kmh = [1.0, 3.0]
    cfg.feedforward.creep_accel_kmhs = [3.0, 1.0]
    cfg.feedforward.coast_band_kmhs = 0.2
    cfg.feedforward.stop_brake_floor_offset_pct = 8.0
    cfg.feedforward.brake_trim_max_kmh = 3.0
    cfg.feedforward.brake_trim_ref_kmh = 0.4
    research = research_ff_params(cfg)
    assert research.creep_accel_speeds_kmh == (1.0, 3.0)
    assert research.creep_accel_kmhs == (3.0, 1.0)
    assert research.coast_band_kmhs == pytest.approx(0.2)
    assert research.stop_brake_floor_offset_pct == pytest.approx(8.0)
    assert research.brake_trim_max_kmh == pytest.approx(3.0)
    assert research.brake_trim_ref_kmh == pytest.approx(0.4)
