"""wltp_grid（WLTP の車速×加速度格子と学習データの網羅マップ）のテスト。"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from tests.research import wltp_grid
from tests.research.research_types import DriveLog, DrivingMode, SpeedPoint

EDGES = wltp_grid.GridEdges(
    speed_kmh=(0.0, 10.0, 20.0, 30.0), accel_kmhs=(-3.0, -0.5, 0.5, 3.0)
)


def _mode(points: list[tuple[float, float]]) -> DrivingMode:
    return DrivingMode(
        id="x", name="t", description="", total_duration=points[-1][0], max_speed=100.0,
        created_at=datetime.now(tz=UTC), is_system=False,
        reference_speed=[SpeedPoint(time_s=t, speed_kmh=v) for t, v in points],
    )


def _logs(
    speeds: list[float], accel: list[float], brake: list[float], dt: float = 0.1
) -> list[DriveLog]:
    origin = datetime(2000, 1, 1, tzinfo=UTC)
    return [
        DriveLog(
            id=i, session_id="s", timestamp=origin + timedelta(seconds=i * dt),
            ref_speed_kmh=None, actual_speed_kmh=v, accel_opening=a, brake_opening=b,
            accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
        )
        for i, (v, a, b) in enumerate(zip(speeds, accel, brake, strict=True))
    ]


def test_wltp_cells_constant_speed_lands_in_zero_accel_column() -> None:
    # 15 km/h 一定 60s → 車速ビン 10〜20、加速度ビン −0.5〜0.5。窓の端（先読み 3s + 過去 1s）を除く
    cells = wltp_grid.wltp_cells(_mode([(0.0, 15.0), (60.0, 15.0)]), EDGES)
    assert cells.shape == EDGES.shape
    assert cells[1, 1] == pytest.approx(56.0, abs=0.5)
    assert cells.sum() == pytest.approx(cells[1, 1])


def test_wltp_cells_ramp_lands_in_accel_column() -> None:
    # 1 km/h/s のランプ 0→30。加速度 +1 は 0.5〜3 の列
    cells = wltp_grid.wltp_cells(_mode([(0.0, 0.0), (30.0, 30.0)]), EDGES)
    assert cells[:, 2].sum() > 0.0
    assert cells[:, 0].sum() == 0.0
    assert cells[:, 1].sum() == 0.0


def test_wltp_cells_excludes_standstill() -> None:
    cells = wltp_grid.wltp_cells(_mode([(0.0, 0.0), (60.0, 0.0)]), EDGES)
    assert cells.sum() == 0.0


def test_data_cells_split_by_pedal_effectiveness() -> None:
    n = 100
    speeds = [15.0] * n
    # 前半: アクセルが効いている / 後半: どちらも不感帯未満（惰行）
    accel = [10.0] * 50 + [0.0] * 50
    brake = [0.0] * n
    cells = wltp_grid.data_cells(_logs(speeds, accel, brake), 4.0, 7.0, EDGES)
    assert cells.brake.sum() == 0.0
    assert cells.accel[1, 1] == pytest.approx(cells.accel.sum())
    assert cells.coast[1, 1] == pytest.approx(cells.coast.sum())
    # 先頭 1s（過去窓）と末尾 3s（先読み窓）は特徴量が作れず数えない
    assert cells.accel.sum() == pytest.approx(4.0, abs=0.1)
    assert cells.coast.sum() == pytest.approx(2.0, abs=0.1)
    assert cells.total.sum() == pytest.approx(cells.accel.sum() + cells.coast.sum())


def test_data_cells_brake_rows_counted_separately() -> None:
    n = 100
    cells = wltp_grid.data_cells(
        _logs([15.0] * n, [0.0] * n, [12.0] * n), 4.0, 7.0, EDGES
    )
    assert cells.brake.sum() > 0.0
    assert cells.accel.sum() == 0.0


def test_find_holes_only_cells_with_wltp_demand_and_no_data() -> None:
    wltp = np.zeros(EDGES.shape)
    wltp[1, 1] = 20.0  # 需要あり・データなし → 穴
    wltp[2, 2] = 20.0  # 需要あり・データあり → 穴でない
    wltp[0, 0] = 1.0  # 需要が小さい → 対象外
    zeros = np.zeros(EDGES.shape)
    data_accel = zeros.copy()
    data_accel[2, 2] = 10.0
    data = wltp_grid.DataCells(accel=data_accel, brake=zeros, coast=zeros)
    holes = wltp_grid.find_holes(wltp, data, EDGES, wltp_min_s=5.0, data_max_s=2.0)
    assert len(holes) == 1
    assert (holes[0].speed_lo_kmh, holes[0].accel_lo_kmhs) == (10.0, -0.5)
    assert holes[0].wltp_s == 20.0


def test_find_holes_sorted_by_wltp_demand_descending() -> None:
    wltp = np.zeros(EDGES.shape)
    wltp[0, 1], wltp[1, 1] = 10.0, 30.0
    zeros = np.zeros(EDGES.shape)
    data = wltp_grid.DataCells(accel=zeros, brake=zeros, coast=zeros)
    holes = wltp_grid.find_holes(wltp, data, EDGES, 5.0, 2.0)
    assert [h.wltp_s for h in holes] == [30.0, 10.0]


def test_coverage_table_marks_holes() -> None:
    wltp = np.zeros(EDGES.shape)
    wltp[1, 1] = 20.0
    zeros = np.zeros(EDGES.shape)
    data = wltp_grid.DataCells(accel=zeros, brake=zeros, coast=zeros)
    text = wltp_grid.coverage_table(wltp, data, EDGES, 5.0, 2.0)
    assert "20/0.0*" in text
    assert text.count("*") == 1


def test_summary_text_reports_hole_share() -> None:
    wltp = np.zeros(EDGES.shape)
    wltp[1, 1] = 20.0
    wltp[2, 2] = 20.0
    zeros = np.zeros(EDGES.shape)
    data_accel = zeros.copy()
    data_accel[2, 2] = 10.0
    data = wltp_grid.DataCells(accel=data_accel, brake=zeros, coast=zeros)
    holes = wltp_grid.find_holes(wltp, data, EDGES, 5.0, 2.0)
    text = wltp_grid.summary_text(wltp, data, holes, 5.0, 2.0)
    assert "1 セル" in text
    assert "50%" in text


def test_holes_text_when_none() -> None:
    assert wltp_grid.holes_text([]) == "穴はありません"


def test_mode_from_csv_reads_mode_time_and_ref_speed(tmp_path: Path) -> None:
    path = tmp_path / "ref.csv"
    path.write_text(
        "mode_time_s,ref_speed_kmh,other\n0.0,0.0,x\n1.0,5.0,x\n2.0,10.0,x\n", encoding="utf-8"
    )
    mode = wltp_grid.mode_from_csv(path)
    assert [(p.time_s, p.speed_kmh) for p in mode.reference_speed] == [
        (0.0, 0.0), (1.0, 5.0), (2.0, 10.0),
    ]
    assert mode.max_speed == 10.0


def test_mode_from_csv_rejects_missing_columns(tmp_path: Path) -> None:
    from tests.research.config import ConfigError

    path = tmp_path / "bad.csv"
    path.write_text("a,b\n1,2\n", encoding="utf-8")
    with pytest.raises(ConfigError):
        wltp_grid.mode_from_csv(path)


def test_coverage_table_shows_data_seconds_with_one_decimal() -> None:
    wltp = np.zeros(EDGES.shape)
    wltp[1, 1] = 20.0
    zeros = np.zeros(EDGES.shape)
    accel = zeros.copy()
    accel[1, 1] = 1.5  # 丸めて「2」と出ながら穴（< 2s）と判定されないように 1 桁で出す
    data = wltp_grid.DataCells(accel=accel, brake=zeros, coast=zeros)
    text = wltp_grid.coverage_table(wltp, data, EDGES, 5.0, 2.0)
    assert "20/1.5*" in text


def test_default_grid_treats_plus_minus_half_as_one_cruise_column() -> None:
    from tests.research.wltp_grid import GRID_ACCEL_EDGES_KMHS

    edges = GRID_ACCEL_EDGES_KMHS
    assert -0.5 in edges
    assert 0.5 in edges
    assert not any(-0.5 < e < 0.5 for e in edges)


def test_wltp_cell_stats_mean_and_extremes_per_speed_band() -> None:
    # 1 km/h/s のランプ 0→30 のあと 30 km/h 一定 → 加速度 +1 の行と 0 の行が混ざる
    stats = wltp_grid.wltp_cell_stats(_mode([(0.0, 0.0), (30.0, 30.0), (90.0, 30.0)]), EDGES)
    assert stats.seconds.shape == EDGES.shape
    assert stats.mean_accel[1, 2] == pytest.approx(1.0, abs=0.05)  # 10〜20 km/h × 0.5〜3
    assert np.isnan(stats.mean_accel[0, 0])  # 滞在の無いセルは nan
    assert stats.max_accel_by_speed[1] == pytest.approx(1.0, abs=0.05)
    assert stats.min_accel_by_speed[1] == pytest.approx(1.0, abs=0.05)
    assert stats.seconds.sum() == pytest.approx(wltp_grid.wltp_cells(
        _mode([(0.0, 0.0), (30.0, 30.0), (90.0, 30.0)]), EDGES).sum())


# ── 複数モードの合成（段4b） ─────────────────────────────────────────────


def _stats(seconds: list[list[float]], means: list[list[float]], amax: list[float],
           amin: list[float]) -> wltp_grid.WltpCellStats:
    return wltp_grid.WltpCellStats(
        np.array(seconds, dtype=float), np.array(means, dtype=float),
        np.array(amax, dtype=float), np.array(amin, dtype=float),
    )


def test_combined_stats_takes_max_seconds_weighted_mean_and_extremes() -> None:
    nan = float("nan")
    a = _stats([[3.0, 0.0]], [[1.0, nan]], [2.0], [-1.0])
    b = _stats([[1.5, 4.0]], [[3.0, -2.0]], [5.0], [nan])
    out = wltp_grid.combined_cell_stats([a, b])
    assert out.seconds.tolist() == [[3.0, 4.0]]  # 合計にしない（3+1.5 で 4.5 にならない）
    assert out.mean_accel[0, 0] == pytest.approx((3.0 * 1.0 + 1.5 * 3.0) / 4.5)  # 秒数で重み付け
    assert out.mean_accel[0, 1] == pytest.approx(-2.0)
    assert out.max_accel_by_speed.tolist() == [5.0]
    assert out.min_accel_by_speed.tolist() == [-1.0]  # nan は無視して他方を採る


def test_combined_stats_keeps_nan_for_cells_no_mode_uses() -> None:
    nan = float("nan")
    a = _stats([[0.0]], [[nan]], [nan], [nan])
    out = wltp_grid.combined_cell_stats([a, a])
    assert np.isnan(out.mean_accel[0, 0]) and np.isnan(out.max_accel_by_speed[0])
    with pytest.raises(ValueError):
        wltp_grid.combined_cell_stats([])


def test_outside_seconds_counts_running_beyond_the_accel_edges() -> None:
    # 5 km/h/s のランプ 0→50 は列の端（±3）の外。停車は除く
    inside = wltp_grid.outside_seconds(_mode([(0.0, 0.0), (30.0, 30.0)]), EDGES)  # +1
    outside = wltp_grid.outside_seconds(_mode([(0.0, 0.0), (10.0, 50.0)]), EDGES)  # +5
    assert inside == 0.0
    assert outside > 3.0


def test_coverage_stats_from_modes_merges_and_reports_each_mode() -> None:
    slow = _mode([(0.0, 15.0), (60.0, 15.0)])
    ramp = _mode([(0.0, 0.0), (10.0, 50.0)])
    ramp = ramp.model_copy(update={"name": "ramp"}) if hasattr(ramp, "model_copy") else ramp
    stats, outside = wltp_grid.coverage_stats_from_modes([slow, ramp], EDGES)
    assert [n for n, _ in outside] == [slow.name, ramp.name]
    assert outside[0][1] == 0.0 and outside[1][1] > 3.0
    assert stats.seconds[1, 1] == pytest.approx(56.0, abs=0.5)  # 一定走行のマスは残る
    assert "格子外" in wltp_grid.outside_text(outside)


def test_coverage_stats_for_config_builds_release_seconds() -> None:
    """実機の手順2 が pedal_rule を渡し忘れて戻しの狙いが None になっていた（20261001）。"""
    from tests.research import config as cfgmod

    cfg = cfgmod.ResearchConfig()
    cfg.feedforward.coast_decel_speeds_kmh = [0.0, 50.0, 100.0]
    cfg.feedforward.coast_decel_kmhs = [0.0, -0.3, -0.6]
    assert wltp_grid.pedal_rule_from_config(cfg) is not None
    stats, outside = wltp_grid.coverage_stats_for_config(cfg, [_mode([(0.0, 20.0), (60.0, 60.0)])])
    assert stats.seconds_release_brake is not None
    assert stats.seconds_release_accel is not None
    assert len(outside) == 1


def test_coverage_stats_for_config_without_coast_curve_has_no_release_seconds() -> None:
    from tests.research import config as cfgmod

    cfg = cfgmod.ResearchConfig()
    cfg.feedforward.coast_decel_speeds_kmh = []
    cfg.feedforward.coast_decel_kmhs = []
    stats, _ = wltp_grid.coverage_stats_for_config(cfg, [_mode([(0.0, 20.0), (60.0, 60.0)])])
    assert stats.seconds_release_brake is None


# ── 向きの軸と重さ（段2c-1） ────────────────────────────────────────────


def test_mode_direction_decel_strengthen_then_weaken() -> None:
    # 減速が 0 → −1 → −3 と強まり、−3 → −1 と弱まる（ブレーキを戻す側）
    a = np.array([-1.0, -2.0, -3.0, -3.0, -2.5, -2.0, -1.0])
    out = wltp_grid.mode_direction(a, 0.3)
    assert out.tolist() == [False, False, False, False, True, True, True]


def test_mode_direction_accel_waveform_and_threshold() -> None:
    a = np.array([1.0, 2.0, 3.0, 2.9, 2.0])
    assert wltp_grid.mode_direction(a, 0.3).tolist() == [False, False, False, False, True]
    # しきい値未満の落ち（2.9 は 0.1 だけ）は踏み込みのまま。しきい値が小さければ戻し
    assert wltp_grid.mode_direction(a, 0.05).tolist() == [False, False, False, True, True]


def test_mode_direction_resets_at_sign_change_and_zero() -> None:
    a = np.array([3.0, 1.0, -1.0, -1.2, -1.0, 0.0, 2.0, 1.0])
    out = wltp_grid.mode_direction(a, 0.3)
    # 1.0 は戻し。符号が変わった −1.0 は新しい episode の頭（踏み込み）。0 で区切り、2.0 から再開
    assert out.tolist() == [False, True, False, False, False, False, False, True]


def test_wltp_cell_stats_press_plus_release_equals_seconds() -> None:
    mode = _mode([(0.0, 40.0), (10.0, 40.0), (20.0, 20.0), (30.0, 20.0), (40.0, 30.0)])
    stats = wltp_grid.wltp_cell_stats(mode, EDGES_WIDE)
    assert stats.seconds_press is not None and stats.seconds_release is not None
    assert (stats.seconds_press + stats.seconds_release) == pytest.approx(stats.seconds)
    assert stats.seconds_release.sum() > 0.0
    assert stats.seconds_press.sum() > 0.0


def test_stats_without_direction_arrays_are_all_press() -> None:
    z = np.zeros((1, 1))
    s = wltp_grid.WltpCellStats(z + 3.0, z, np.zeros(1), np.zeros(1))
    assert s.seconds_press is not None and s.seconds_release is not None
    assert s.seconds_press[0, 0] == 3.0 and s.seconds_release[0, 0] == 0.0


def test_combined_cell_stats_takes_max_per_direction() -> None:
    z = np.zeros((1, 1))
    a = wltp_grid.WltpCellStats(z + 3.0, z, np.zeros(1), np.zeros(1), z + 2.0, z + 1.0)
    b = wltp_grid.WltpCellStats(z + 4.0, z, np.zeros(1), np.zeros(1), z + 1.0, z + 3.0)
    out = wltp_grid.combined_cell_stats([a, b])
    assert out.seconds_press is not None and out.seconds_release is not None
    assert out.seconds_press[0, 0] == 2.0 and out.seconds_release[0, 0] == 3.0


def test_pedal_release_rise_then_fall() -> None:
    opening = np.array([0.0, 2.0, 5.0, 5.0, 4.8, 4.5, 2.0, 0.0, 3.0, 2.9])
    out = wltp_grid.pedal_release(opening, deadband_pct=1.0, threshold_pct=0.3)
    assert out.tolist() == [False, False, False, False, False, True, True, False, False, False]


def test_pedal_release_restarts_press_after_release() -> None:
    # 10 は episode の最初、50 が山、20 で戻し。戻し中に 25（最小 20 から +5）は踏み込みに切り替え
    opening = np.array([10.0, 50.0, 50.0, 20.0, 20.0, 25.0, 25.0])
    out = wltp_grid.pedal_release(opening, deadband_pct=5.0, threshold_pct=1.5)
    assert out.tolist() == [False, False, False, True, True, False, False]
    # その後 24（25−24 < 1.5）はまだ踏み込み、23（25−23 ≥ 1.5）で戻し
    opening2 = np.append(opening, [24.0, 23.0])
    out2 = wltp_grid.pedal_release(opening2, deadband_pct=5.0, threshold_pct=1.5)
    assert out2.tolist() == [False, False, False, True, True, False, False, False, True]


def test_pedal_release_episode_ends_below_deadband() -> None:
    # 不感帯（5）を下回ると episode が切れ、次の episode の最初（30）は踏み込み
    opening = np.array([10.0, 50.0, 20.0, 0.0, 30.0, 29.0, 20.0])
    out = wltp_grid.pedal_release(opening, deadband_pct=5.0, threshold_pct=1.5)
    assert out.tolist() == [False, False, True, False, False, False, True]


def _coast_rule(free: float = -2.0, band: float = 0.3) -> wltp_grid.PedalRule:
    return (lambda v: free, band)


def test_mode_direction_with_pedal_rule_throttle_lift_is_release() -> None:
    # free = −2.0・帯 0.3: a_req > −1.7 はアクセル。アクセルを絞って減速を強めていく
    a = np.array([0.0, -0.5, -1.0, -1.4])
    v0 = np.full(4, 50.0)
    out = wltp_grid.mode_direction(a, 0.3, v0, _coast_rule())
    # 踏む量 = a − free = 2.0, 1.5, 1.0, 0.6 → 2 行目以降は戻し
    assert out.tolist() == [False, True, True, True]
    # 規則なし（従来の符号ベース）は a_req=0 で区切られ、−0.5 から減速が強まる → 全部踏み込み
    assert wltp_grid.mode_direction(a, 0.3).tolist() == [False] * 4


def test_mode_direction_with_pedal_rule_brake_rows() -> None:
    # a_req ≤ −2.3 はブレーキ。減速が強まる → 踏み込み、弱まる → 戻し
    a = np.array([-2.5, -3.0, -4.0, -3.5, -3.0])
    v0 = np.full(5, 50.0)
    out = wltp_grid.mode_direction(a, 0.3, v0, _coast_rule())
    assert out.tolist() == [False, False, False, True, True]


def test_data_cells_release_rows_counted_separately() -> None:
    n = 120
    speeds = [29.0 - 0.1 * i for i in range(n)]  # -1 km/h/s（定速列は向きを分けない）
    brake = [0.0] * 20 + [5.0] * 40 + [3.0] * 40 + [0.0] * 20
    cells = wltp_grid.data_cells(
        _logs(speeds, [0.0] * n, brake), 4.0, 1.0, EDGES, direction_threshold_pct=0.3
    )
    assert cells.brake.sum() == pytest.approx(4.0, abs=0.5)  # 山の 40 行
    # 戻した 40 行（末尾 3s は先読みの窓で特徴量が作れず落ちる → 3s）
    assert cells.brake_release.sum() == pytest.approx(3.0, abs=0.5)
    assert cells.accel_release.sum() == 0.0
    assert cells.total.sum() == pytest.approx(
        cells.press_total.sum() + cells.release_total.sum()
    )


def test_data_cells_accel_uses_its_own_larger_threshold() -> None:
    n = 120
    speeds = [15.0] * n
    accel = [0.0] * 20 + [8.0] * 40 + [7.0] * 40 + [0.0] * 20  # 1% 戻し
    logs = _logs(speeds, accel, [0.0] * n)
    # EDGES の加速度 0 列（-0.5〜0.5）は定速の列なので、向きを分けず全部踏み込み側
    d = wltp_grid.data_cells(logs, 4.0, 1.0, EDGES, direction_threshold_accel_pct=1.5)
    assert d.accel_release.sum() == 0.0
    d = wltp_grid.data_cells(logs, 4.0, 1.0, EDGES_WIDE, direction_threshold_accel_pct=1.5)
    assert d.accel_release.sum() == 0.0 and d.accel.sum() > 0.0  # 1% < 1.5%
    d = wltp_grid.data_cells(logs, 4.0, 1.0, EDGES_WIDE, direction_threshold_accel_pct=0.5)
    assert d.accel_release.sum() == 0.0  # 定速の列は 0.5 でも向きを分けない


def test_cruise_column_is_never_release_on_either_side() -> None:
    mode = _mode([(0.0, 40.0), (60.0, 40.0)])  # 定速
    stats = wltp_grid.wltp_cell_stats(mode, EDGES_WIDE, direction_threshold_kmhs=0.0)
    assert stats.seconds_release.sum() == 0.0
    assert stats.seconds_press.sum() == pytest.approx(stats.seconds.sum())


def test_required_seconds_formula() -> None:
    mode_s = np.array([[4.0, 20.0, 100.0]])
    req = wltp_grid.required_seconds(mode_s, 2.0, 0.25)
    assert req.tolist() == [[2.0, 5.0, 25.0]]


def test_find_holes_by_direction_uses_weighted_requirement() -> None:
    z = np.zeros(EDGES.shape)
    press = z.copy()
    press[1, 1] = 40.0
    release = z.copy()
    release[1, 1] = 20.0
    release[2, 0] = 6.0
    stats = wltp_grid.WltpCellStats(
        press + release, z, np.zeros(3), np.zeros(3), press, release
    )
    data_press = z.copy()
    data_press[1, 1] = 5.0  # 要る 10s（0.25×40）に足りない
    data_rel = z.copy()
    data_rel[1, 1] = 5.0  # 要る 5s（0.25×20）ちょうど → 穴でない
    data = wltp_grid.DataCells(data_press, z, z, z, data_rel)
    holes = wltp_grid.find_holes_by_direction(stats, data, EDGES, 2.5, 2.0, 0.25)
    got = {(h.direction, h.speed_lo_kmh, h.accel_lo_kmhs): h for h in holes}
    assert ("press", 10.0, -0.5) in got and got[("press", 10.0, -0.5)].required_s == 10.0
    assert ("release", 10.0, -0.5) not in got
    assert ("release", 20.0, -3.0) in got  # データ無し・要る 2s
    assert len(holes) == 2


def test_direction_report_shows_both_tables() -> None:
    z = np.zeros(EDGES.shape)
    rel = z.copy()
    rel[1, 1] = 10.0
    stats = wltp_grid.WltpCellStats(rel, z, np.zeros(3), np.zeros(3), z, rel)
    text = wltp_grid.direction_report(stats, wltp_grid.DataCells(z, z, z), EDGES, 2.5, 2.0, 0.25)
    assert "踏み込み" in text and "戻し" in text and "10/0.0*" in text


EDGES_WIDE = wltp_grid.GridEdges(
    speed_kmh=(0.0, 10.0, 20.0, 30.0, 40.0, 50.0), accel_kmhs=(-14.0, -3.0, -0.5, 0.5, 3.0, 14.0)
)


# ── 行ごとのペダル割り当てと、ペダル別の戻し穴 ──────────────────────────


def test_row_pedals_around_band_edges() -> None:
    rule = (lambda v: -1.0, 0.5)
    v0 = np.full(5, 40.0)
    a = np.array([-0.5, -0.51, -1.0, -1.49, -1.5])
    assert wltp_grid.row_pedals(v0, a, rule).tolist() == ["A", "-", "-", "-", "B"]


def test_row_pedals_use_each_rows_own_speed() -> None:
    rule = (lambda v: -v / 20.0, 0.5)  # 40 km/h で −2、20 km/h で −1
    out = wltp_grid.row_pedals(np.array([40.0, 20.0]), np.array([-2.6, -2.6]), rule)
    assert out.tolist() == ["B", "B"]
    out = wltp_grid.row_pedals(np.array([40.0, 20.0]), np.array([-2.2, -1.2]), rule)
    assert out.tolist() == ["-", "-"]


def test_coast_mean_cell_with_brake_rows_yields_brake_release_seconds() -> None:
    # 減速が −3（ブレーキ行）から −2（惰行帯）へ弱まる。折れ点のまわりの −2.5〜−3 の行は
    # ブレーキの戻し、−2.5 より弱い戻しの行は惰行帯で、行ごとに別のペダルになる
    pts = [(0.0, 100.0), (15.0, 55.0), (40.0, 5.0)]  # −3 km/h/s → −2 km/h/s（折れ点 15s）
    mode = _mode(pts)
    edges = wltp_grid.GridEdges(
        speed_kmh=(0.0, 100.0), accel_kmhs=(-14.0, -3.0, -1.5, -0.5, 0.5, 3.0)
    )
    rule = (lambda v: -2.0, 0.5)
    st = wltp_grid.wltp_cell_stats(mode, edges, direction_threshold_kmhs=0.3, pedal_rule=rule)
    assert st.seconds_release_brake is not None and st.seconds_release_accel is not None
    # 強い減速の戻し側の行はブレーキ戻し。惰行帯の戻しはどちらにも数えない
    assert st.seconds_release_brake.sum() > 0.0
    assert st.seconds_release_accel.sum() == 0.0
    assert (st.seconds_release_brake + st.seconds_release_accel).sum() <= st.seconds_release.sum()
    assert (st.seconds_press + st.seconds_release).sum() == pytest.approx(st.seconds.sum())


def test_combined_cell_stats_takes_max_for_pedal_split_and_drops_when_missing() -> None:
    z = np.zeros((1, 1))
    a = wltp_grid.WltpCellStats(
        z + 3.0, z, np.zeros(1), np.zeros(1), z + 2.0, z + 1.0, z + 0.5, z + 0.5)
    b = wltp_grid.WltpCellStats(
        z + 4.0, z, np.zeros(1), np.zeros(1), z + 1.0, z + 3.0, z + 0.2, z + 2.5)
    out = wltp_grid.combined_cell_stats([a, b])
    assert out.seconds_release_accel[0, 0] == 0.5 and out.seconds_release_brake[0, 0] == 2.5
    c = wltp_grid.WltpCellStats(z + 4.0, z, np.zeros(1), np.zeros(1), z + 1.0, z + 3.0)
    assert wltp_grid.combined_cell_stats([a, c]).seconds_release_brake is None


def test_release_holes_are_judged_per_pedal() -> None:
    edges = wltp_grid.GridEdges(speed_kmh=(20.0, 30.0), accel_kmhs=(-3.0, -1.5))
    z = np.zeros((1, 1))
    # モードの戻し 16s のうち ブレーキ 8s・アクセル 6s・惰行 2s
    stats = wltp_grid.WltpCellStats(
        z + 16.0, z - 2.0, np.zeros(1), np.zeros(1), z, z + 16.0, z + 6.0, z + 8.0)
    data = wltp_grid.DataCells(z, z, z, z + 11.5, z)  # アクセルの戻しだけ 11.5s
    holes = wltp_grid.find_holes_by_direction(stats, data, edges, 2.5, 2.0, 0.25)
    rel = [h for h in holes if h.direction == "release"]
    assert [(h.pedal, h.wltp_s, h.data_s, h.required_s) for h in rel] == [("B", 8.0, 0.0, 2.0)]
    # ブレーキの戻しが十分あり、アクセルが無ければアクセル側だけ穴
    data = wltp_grid.DataCells(z, z, z, z, z + 5.0)
    rel = [h for h in wltp_grid.find_holes_by_direction(stats, data, edges, 2.5, 2.0, 0.25)
           if h.direction == "release"]
    assert [h.pedal for h in rel] == ["A"]
    # 両方足りなければ 1 マスに 2 つ
    data = wltp_grid.DataCells(z, z, z, z, z)
    rel = [h for h in wltp_grid.find_holes_by_direction(stats, data, edges, 2.5, 2.0, 0.25)
           if h.direction == "release"]
    assert sorted(h.pedal for h in rel) == ["A", "B"]


def test_release_holes_fall_back_to_combined_without_pedal_split() -> None:
    edges = wltp_grid.GridEdges(speed_kmh=(20.0, 30.0), accel_kmhs=(-3.0, -1.5))
    z = np.zeros((1, 1))
    stats = wltp_grid.WltpCellStats(z + 16.0, z - 2.0, np.zeros(1), np.zeros(1), z, z + 16.0)
    data = wltp_grid.DataCells(z, z, z, z + 11.5, z)
    assert wltp_grid.find_holes_by_direction(stats, data, edges, 2.5, 2.0, 0.25) == []


def test_direction_report_shows_per_pedal_release_tables() -> None:
    edges = wltp_grid.GridEdges(speed_kmh=(20.0, 30.0), accel_kmhs=(-3.0, -1.5))
    z = np.zeros((1, 1))
    stats = wltp_grid.WltpCellStats(
        z + 16.0, z - 2.0, np.zeros(1), np.zeros(1), z, z + 16.0, z + 6.0, z + 8.0)
    text = wltp_grid.direction_report(stats, wltp_grid.DataCells(z, z, z), edges, 2.5, 2.0, 0.25)
    assert "戻し（ブレーキ）" in text and "戻し（アクセル）" in text and "8/0.0*" in text
