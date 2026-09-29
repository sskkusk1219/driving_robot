"""格子ステップ走行のパターン列を組み立てる（ProblemReport_20260925 段3）。

WLTP の集計（`wltp_grid.wltp_cell_stats` の結果）から、車速ステーションごとの
`GridStationPattern` と、最後に 1 本の `GridLaunchPattern`（発進・停車セル）を作る。
`wltp_grid` は import しない（wltp_grid → mode_drive → pattern_drive の循環になるため）。集計結果は
呼び出し側が渡す（`seconds`・`mean_accel`・`max_accel_by_speed`・`min_accel_by_speed` を持つもの）。
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol

import numpy as np

from tests.research.config import ResearchConfig
from tests.research.grid_planner import GridSettings, plan_launch, plan_stations
from tests.research.pattern_loop import GridLaunchPattern, GridStationPattern
from tests.research.research_types import LearningPattern, PatternKind


class WltpStatsLike(Protocol):
    seconds: np.ndarray
    mean_accel: np.ndarray
    max_accel_by_speed: np.ndarray
    min_accel_by_speed: np.ndarray


def grid_settings_from_config(cfg: ResearchConfig) -> GridSettings:
    lr = cfg.learning
    return GridSettings(
        station_min_kmh=lr.grid_station_min_kmh,
        settle_tol_kmh=lr.grid_settle_tol_kmh,
        settle_s=lr.grid_settle_s,
        settle_timeout_s=lr.grid_settle_timeout_s,
        step_window_s=lr.grid_step_window_s,
        step_lag_s=lr.grid_step_lag_s,
        step_band_max_kmh=lr.grid_step_band_max_kmh,
        step_min_fit_s=lr.grid_step_min_fit_s,
        overshoot_frac=lr.grid_overshoot_frac,
        max_tries=lr.grid_max_tries,
        gain_init=lr.grid_gain_init_kmhs_per_pct,
        brake_gain_init=lr.grid_brake_gain_init_kmhs_per_pct,
        gain_min=lr.grid_gain_min_kmhs_per_pct,
        gain_max=lr.grid_gain_max_kmhs_per_pct,
        hold_kp_norm=lr.grid_hold_kp_norm,
        hold_ki_norm=lr.grid_hold_ki_norm,
        hold_max_rate_pct_s=lr.grid_hold_max_rate_pct_per_s,
        launch_end_kmh=lr.grid_launch_end_kmh,
        wltp_min_s=lr.grid_target_min_s,
        grid_return_accel_kmhs=lr.grid_return_accel_kmhs,
        grid_return_switch_kmh=lr.grid_return_switch_kmh,
    )


def build_grid_patterns(
    cfg: ResearchConfig,
    stats: WltpStatsLike,
    *,
    stations_kmh: Sequence[float] | None = None,
    with_launch: bool = True,
) -> list[LearningPattern]:
    """ステーション（車速の昇順）→ 発進・停車セル、の順のパターン列。

    `stations_kmh` は、動作確認で車速ステーションを絞るときにその中心車速を渡す（None で全部）。
    ステーションが 1 つも無ければ空。発進・停車の狙いが無ければ発進パターンは足さない。
    """
    lr = cfg.learning
    settings = grid_settings_from_config(cfg)
    speed_edges, accel_edges = lr.grid_speed_edges_kmh, lr.grid_accel_edges_kmhs
    args = (
        stats.seconds, stats.mean_accel, stats.max_accel_by_speed, stats.min_accel_by_speed,
        speed_edges, accel_edges, settings,
    )
    plans = plan_stations(*args)
    if stations_kmh is not None:
        wanted = {float(v) for v in stations_kmh}
        plans = [p for p in plans if p.speed_kmh in wanted]
    patterns: list[LearningPattern] = [
        GridStationPattern(
            PatternKind.GRID_STEP, accel_opening=0.0, brake_opening=0.0,
            hold_duration_s=settings.settle_timeout_s, plan=plan, settings=settings,
        )
        for plan in plans
    ]
    if with_launch:
        launch = plan_launch(*args)
        if launch.accel or launch.decel:
            patterns.append(
                GridLaunchPattern(
                    PatternKind.GRID_LAUNCH, accel_opening=0.0, brake_opening=0.0,
                    hold_duration_s=settings.settle_timeout_s, plan=launch, settings=settings,
                )
            )
    return patterns


__all__ = ["build_grid_patterns", "grid_settings_from_config"]
