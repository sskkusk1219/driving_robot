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
    from tests.research.config import LearningSection

    edges = LearningSection().grid_accel_edges_kmhs
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
