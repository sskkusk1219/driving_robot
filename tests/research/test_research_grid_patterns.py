"""grid_patterns（WLTP 集計 → 格子ステップ走行のパターン列）と関連 config のテスト。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research.grid_patterns import build_grid_patterns, grid_settings_from_config
from tests.research.pattern_loop import GridLaunchPattern, GridStationPattern
from tests.research.research_types import PatternKind

CFG_PATH = Path("tests/research/config_testVehicle.yaml")


class _Stats:
    """`wltp_cell_stats` の結果と同じ形（速度 14 行 × 加速度 9 列。列の境界は config の既定）。"""

    def __init__(self) -> None:
        n_speed, n_accel = 14, 9
        self.seconds = np.full((n_speed, n_accel), 9.0)
        self.seconds[13] = 0.0  # 130〜140 は WLTP が無い
        self.mean_accel = np.tile(
            np.array([-9.0, -4.0, -2.0, -1.0, 0.0, 1.0, 2.0, 4.0, 9.0]), (n_speed, 1)
        )
        self.max_accel_by_speed = np.full(n_speed, 5.0)
        self.min_accel_by_speed = np.full(n_speed, -5.0)


def _cfg() -> cfgmod.ResearchConfig:
    return cfgmod.load_config(CFG_PATH)


def test_builds_stations_in_ascending_order_then_one_launch() -> None:
    patterns = build_grid_patterns(_cfg(), _Stats())
    stations = [p for p in patterns if isinstance(p, GridStationPattern)]
    assert [p.plan.speed_kmh for p in stations if p.plan] == [
        15.0, 25.0, 35.0, 45.0, 55.0, 65.0, 75.0, 85.0, 95.0, 105.0, 115.0, 125.0
    ]  # 0〜10 は station_min 未満、130〜140 は WLTP が無い
    assert all(p.kind is PatternKind.GRID_STEP for p in stations)
    assert isinstance(patterns[-1], GridLaunchPattern)
    assert patterns[-1].kind is PatternKind.GRID_LAUNCH
    assert len(patterns) == len(stations) + 1


def test_stations_and_launch_can_be_narrowed() -> None:
    patterns = build_grid_patterns(_cfg(), _Stats(), stations_kmh=[15.0, 55.0], with_launch=False)
    assert [p.plan.speed_kmh for p in patterns if isinstance(p, GridStationPattern) and p.plan] == [
        15.0, 55.0
    ]
    assert not any(isinstance(p, GridLaunchPattern) for p in patterns)


def test_no_matching_station_gives_no_pattern() -> None:
    assert build_grid_patterns(_cfg(), _Stats(), stations_kmh=[999.0], with_launch=False) == []


def test_settings_come_from_config_and_targets_from_stats() -> None:
    cfg = _cfg()
    settings = grid_settings_from_config(cfg)
    assert settings.settle_s == cfg.learning.grid_settle_s
    assert settings.hold_max_rate_pct_s == cfg.learning.grid_hold_max_rate_pct_per_s
    assert settings.wltp_min_s == pytest.approx(2.5)  # 窓 3.0s − 頭の除外 0.5s
    assert settings.wltp_min_s == cfg.learning.grid_target_min_s
    assert settings.step_min_fit_s == cfg.learning.grid_step_min_fit_s
    assert settings.grid_return_accel_kmhs == cfg.learning.grid_return_accel_kmhs
    assert settings.grid_return_switch_kmh == cfg.learning.grid_return_switch_kmh
    first = build_grid_patterns(cfg, _Stats(), with_launch=False)[0]
    assert isinstance(first, GridStationPattern) and first.plan is not None
    assert [t.a_kmhs for t in first.plan.decel] == [-1.0, -2.0, -4.0, -9.0]  # 緩い順
    assert [t.a_kmhs for t in first.plan.accel] == [1.0, 2.0, 4.0, 9.0]  # 小さい順


def test_launch_is_skipped_when_wltp_has_no_low_speed_targets() -> None:
    stats = _Stats()
    stats.seconds[:2] = 0.0  # 0〜20 km/h に WLTP が無い
    patterns = build_grid_patterns(_cfg(), stats)
    assert not any(isinstance(p, GridLaunchPattern) for p in patterns)


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("grid_settle_s", 0.0),
        ("grid_step_lag_s", 5.0),  # 窓（3.0s）以上
        ("grid_overshoot_frac", 0.9),
        ("grid_max_tries", 0),
        ("grid_sweep_max_passes", -1),
        ("grid_gain_init_kmhs_per_pct", 100.0),  # 上限（20）を超える
        ("grid_hold_max_rate_pct_per_s", -1.0),
        ("grid_step_min_fit_s", 0.0),
        ("grid_step_min_fit_s", 2.5),  # 窓 − 頭の除外（2.5s）以上
        ("grid_return_accel_kmhs", 0.0),
        ("grid_return_switch_kmh", 0.0),
    ],
)
def test_config_rejects_out_of_range_grid_values(key: str, value: float) -> None:
    cfg = _cfg()
    assert not any("learning.grid" in p for p in cfgmod.validate_config(cfg))
    setattr(cfg.learning, key, value)
    assert any("learning.grid" in p for p in cfgmod.validate_config(cfg))


def test_config_rejects_return_switch_at_or_below_settle_tol() -> None:
    """段7b: grid_return_switch_kmh は grid_settle_tol_kmh より大きくないと GRID_RETURN が
    抜けられなくなる（許容幅の中に入っても切り替え条件を満たさない）。"""
    cfg = _cfg()
    assert not any("grid_return_switch_kmh" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.grid_return_switch_kmh = cfg.learning.grid_settle_tol_kmh
    assert any("grid_return_switch_kmh" in p for p in cfgmod.validate_config(cfg))


@pytest.mark.parametrize("value", [0.0, 0.4, 0.5])
def test_config_rejects_g_cap_at_or_above_the_vehicle_limit(value: float) -> None:
    cfg = _cfg()
    assert not any("g_cap_g" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.g_cap_g = value  # 0 以下、または vehicle.max_decel_g（0.4G）以上
    assert any("g_cap_g" in p for p in cfgmod.validate_config(cfg))


def test_target_min_seconds_follows_step_window_and_lag() -> None:
    cfg = _cfg()
    cfg.learning.grid_step_window_s = 4.0
    cfg.learning.grid_step_lag_s = 1.0
    assert grid_settings_from_config(cfg).wltp_min_s == pytest.approx(3.0)


def test_cells_between_step_length_and_five_seconds_are_now_targeted() -> None:
    """以前のしきい値 5s だと落ちていた（2.5〜5s の）マスも狙う。"""
    stats = _Stats()
    stats.seconds[3, :] = 3.0  # 30〜40 km/h の各マスは 3s（5s 未満）
    patterns = build_grid_patterns(_cfg(), stats, with_launch=False)
    p30 = next(
        p for p in patterns if isinstance(p, GridStationPattern) and p.plan
        and p.plan.speed_kmh == 35.0
    )
    assert p30.plan is not None and len(p30.plan.accel) == 4 and len(p30.plan.decel) == 4
