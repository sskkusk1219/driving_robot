"""fwd_data（順モデルの学習用の行）のテスト。合成の小さな CSV だけを使う。"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import pytest

from tests.research.fwd_data import (
    DROP_BOTH_PEDALS,
    DROP_GOVERNOR,
    DROP_NO_OPENING,
    DROP_NO_WINDOW,
    DROP_STOPPED,
    FwdRows,
    cell_counts,
    concat_rows,
    effective_opening,
    read_fwd_rows,
    thin_cells,
    u_cell,
)

AD, BD = 4.0, 7.0
COLUMNS = [
    "elapsed_s",
    "actual_speed_kmh",
    "accel_actual_pct",
    "brake_actual_pct",
    "accel_cmd_pct",
    "brake_cmd_pct",
    "section",
    "governor_active",
]


def write_csv(path: Path, rows: list[dict[str, object]]) -> Path:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in COLUMNS})
    return path


def make_rows(
    n: int,
    speed,
    accel: float = 0.0,
    brake: float = 0.0,
    section: str = "PATTERN_DRIVE",
    t0: float = 0.0,
    dt: float = 0.1,
    **extra: object,
) -> list[dict[str, object]]:
    return [
        {
            "elapsed_s": round(t0 + i * dt, 6),
            "actual_speed_kmh": speed(t0 + i * dt),
            "accel_actual_pct": accel,
            "brake_actual_pct": brake,
            "section": section,
            "governor_active": 0,
            **extra,
        }
        for i in range(n)
    ]


def read(path: Path, **kw) -> FwdRows:
    kw.setdefault("opening_source", "actual")  # 既存テストは実開度の列で書かれている
    return read_fwd_rows(path, accel_deadband_pct=AD, brake_deadband_pct=BD, **kw)


def test_effective_opening_sign_and_deadband() -> None:
    assert effective_opening(14.0, 0.0, AD, BD) == pytest.approx(10.0)
    assert effective_opening(0.0, 17.0, AD, BD) == pytest.approx(-10.0)
    assert effective_opening(3.0, 6.0, AD, BD) == 0.0
    assert effective_opening(4.0, 7.0, AD, BD) == 0.0  # ちょうど不感帯は惰行
    assert effective_opening(10.0, 10.0, AD, BD) is None


def test_constant_acceleration_slope_and_u(tmp_path: Path) -> None:
    rows = make_rows(40, lambda t: 20.0 + 2.0 * t, accel=14.0)
    r = read(write_csv(tmp_path / "a.csv", rows))
    assert len(r) > 0
    assert np.allclose(r.accel_kmhs, 2.0)
    assert np.allclose(r.u_pct, 10.0)
    assert r.dt_s == pytest.approx(0.1)


def test_lag_accel_rows_use_later_time_brake_rows_use_now(tmp_path: Path) -> None:
    # t < 5 は加速度 +1、t >= 5 は 加速度 +3（速度は連続）
    def speed(t: float) -> float:
        return 20.0 + t if t < 5.0 else 25.0 + 3.0 * (t - 5.0)

    # アクセル行: 1 行だけ t=4.6 の行を見る。+0.30 秒後 = 4.9 中心の窓 [4.4,5.4] は両方にまたがる
    # 見分けやすいよう、窓が全部 +3 側に入る t=5.2 の行（中心 5.5 → [5.0,6.0]）と、
    # ブレーキ行（中心 t のまま）の t=4.4 の行（窓 [3.9,4.9]、全部 +1 側）で確かめる
    acc_rows = make_rows(100, speed, accel=14.0)
    r = read(write_csv(tmp_path / "a.csv", acc_rows), accel_lag_s=0.30, brake_lag_s=0.0)
    k = int(np.argmin(np.abs(r.t_s - 4.6)))
    # 中心 4.9 の窓 [4.4, 5.4]: 傾きは +1 と +3 の混合で 1 より大きく 3 より小さい
    assert 1.0 < r.accel_kmhs[k] < 3.0
    k = int(np.argmin(np.abs(r.t_s - 4.6)))
    brk_rows = make_rows(100, speed, brake=17.0)
    rb = read(write_csv(tmp_path / "b.csv", brk_rows), accel_lag_s=0.30, brake_lag_s=0.0)
    kb = int(np.argmin(np.abs(rb.t_s - 4.4)))
    assert rb.accel_kmhs[kb] == pytest.approx(1.0)  # 遅れ 0 なら窓 [3.9,4.9] は全部 +1
    ka = int(np.argmin(np.abs(r.t_s - 4.4)))
    assert r.accel_kmhs[ka] > 1.0 + 1e-6  # アクセル行は 0.30 秒後の窓 [4.2,5.2] に +3 が入る
    # 5.0 より前から +3 が全窓に入るのは アクセル行の方が早い（遅れの向き）
    ka2 = int(np.argmin(np.abs(r.t_s - 4.5)))
    kb2 = int(np.argmin(np.abs(rb.t_s - 4.5)))
    assert r.accel_kmhs[ka2] > rb.accel_kmhs[kb2]


def test_drops_stopped_governor_and_missing_opening(tmp_path: Path) -> None:
    rows = make_rows(30, lambda t: 20.0 + t, accel=14.0)
    rows[5]["actual_speed_kmh"] = 0.2  # 停車
    rows[10]["governor_active"] = 1
    rows[11]["governor_active"] = 1
    rows[15]["accel_actual_pct"] = ""  # 実開度が空
    rows[20]["accel_actual_pct"] = 10.0
    rows[20]["brake_actual_pct"] = 10.0  # 両踏み
    r = read(write_csv(tmp_path / "a.csv", rows))
    assert r.dropped[DROP_STOPPED] == 1
    assert r.dropped[DROP_GOVERNOR] == 2
    assert r.dropped[DROP_NO_OPENING] == 1
    assert r.dropped[DROP_BOTH_PEDALS] == 1
    assert r.dropped[DROP_NO_WINDOW] > 0  # 先頭・末尾
    assert len(r) + sum(r.dropped.values()) == 30
    assert r.dropped_seconds()[DROP_GOVERNOR] == pytest.approx(0.2)


def test_other_sections_are_ignored(tmp_path: Path) -> None:
    rows = make_rows(20, lambda t: 20.0 + t, accel=14.0, section="PEDAL_SEARCH")
    assert len(read(write_csv(tmp_path / "a.csv", rows))) == 0
    rows = make_rows(30, lambda t: 20.0 + t, accel=14.0, section="MODE_DRIVE")
    p = write_csv(tmp_path / "b.csv", rows)
    assert len(read(p)) > 0
    assert len(read(p, sections={"PATTERN_DRIVE"})) == 0


def test_slope_does_not_cross_segment_boundary(tmp_path: Path) -> None:
    # 区間 1: +1 km/h/s、区間 2（時刻が 10 秒飛ぶ）: -1 km/h/s。傾きは ±1 のどちらかだけ
    a = make_rows(30, lambda t: 20.0 + t, accel=14.0)
    b = make_rows(30, lambda t: 40.0 - (t - 10.0), accel=14.0, t0=10.0)
    r = read(write_csv(tmp_path / "a.csv", a + b))
    assert set(np.round(r.accel_kmhs, 6)) == {1.0, -1.0}
    # 区間の端の行は窓が取れず捨てられる（遅れ 0.3 秒 + 半窓 0.5 秒 = 末尾 0.8 秒、先頭 0.5 秒）
    assert r.dropped[DROP_NO_WINDOW] > 0


def test_section_change_cuts_segment_even_without_time_gap(tmp_path: Path) -> None:
    a = make_rows(30, lambda t: 20.0 + t, accel=14.0, section="PATTERN_DRIVE")
    b = make_rows(30, lambda t: 40.0 - (t - 3.0), accel=14.0, section="MODE_DRIVE", t0=3.0)
    r = read(write_csv(tmp_path / "a.csv", a + b))
    assert set(np.round(r.accel_kmhs, 6)) == {1.0, -1.0}


def test_u_cell_edges_and_coast_is_own_cell() -> None:
    u = np.array([0.0, 0.1, 2.0, 2.1, -0.1, -2.0, -2.1, 4.0])
    assert u_cell(u, 2.0).tolist() == [0, 1, 1, 2, -1, -1, -2, 2]


def test_cell_counts_seconds_and_speed_edges() -> None:
    n = 4
    rows = FwdRows(
        speed_kmh=np.array([4.9, 5.0, 5.0, 9.99]),
        u_pct=np.array([0.0, 0.0, 2.0, 2.1]),
        accel_kmhs=np.zeros(n),
        t_s=np.arange(n, dtype=float),
        dt_s=0.05,
        source="x",
        dt_row_s=np.full(n, 0.05),
    )
    c = cell_counts(rows, 5.0, 2.0)
    assert c == {(0, 0): pytest.approx(0.05), (1, 0): pytest.approx(0.05),
                 (1, 1): pytest.approx(0.05), (1, 2): pytest.approx(0.05)}


def test_concat_rows_sums_dropped(tmp_path: Path) -> None:
    rows = make_rows(30, lambda t: 20.0 + t, accel=14.0)
    p = write_csv(tmp_path / "a.csv", rows)
    a, b = read(p), read(p)
    c = concat_rows([a, b])
    assert len(c) == 2 * len(a)
    assert c.dropped[DROP_NO_WINDOW] == 2 * a.dropped[DROP_NO_WINDOW]
    assert c.dt_s == pytest.approx(0.1)


def test_thin_cells_lists_check_cells_with_little_training() -> None:
    train = {(1, 0): 5.0, (2, 1): 0.4, (3, 1): 1.0}
    check = {(1, 0): 2.0, (2, 1): 1.5, (3, 1): 0.7, (4, -1): 0.3}
    th = thin_cells(train, check, 1.0)
    assert [(c.speed_idx, c.u_idx) for c in th] == [(2, 1), (4, -1)]
    assert th[0].check_s == 1.5 and th[0].train_s == 0.4
    assert th[1].train_s == 0.0


def test_make_speed_edges_mixed_steps() -> None:
    from tests.research.fwd_data import make_speed_edges

    e = make_speed_edges(32.0)
    assert e.tolist() == [0, 2.5, 5, 7.5, 10, 12.5, 15, 20, 25, 30, 35]
    assert make_speed_edges(32.0, 5.0, 15.0, 5.0).tolist() == [0, 5, 10, 15, 20, 25, 30, 35]


def test_speed_cell_edges_and_cell_counts_with_edges() -> None:
    from tests.research.fwd_data import make_speed_edges, speed_cell_edges

    e = make_speed_edges(40.0)
    v = np.array([0.0, 2.4, 2.5, 14.9, 15.0, 19.9, 20.0, 39.9, 99.0])
    assert speed_cell_edges(v, e).tolist() == [0, 0, 1, 5, 6, 6, 7, 10, len(e) - 2]
    n = 3
    rows = FwdRows(
        speed_kmh=np.array([2.6, 16.0, 19.0]),
        u_pct=np.array([0.0, 0.0, 2.0]),
        accel_kmhs=np.zeros(n),
        t_s=np.arange(n, dtype=float),
        dt_s=0.05,
        source="x",
        dt_row_s=np.full(n, 0.05),
    )
    c = cell_counts(rows, speed_edges=e)
    assert c == {(1, 0): pytest.approx(0.05), (6, 0): pytest.approx(0.05),
                 (6, 1): pytest.approx(0.05)}


def test_opening_source_cmd_vs_actual(tmp_path: Path) -> None:
    # 実開度は 14（u=10）、最終指令は 24（u=20）。ブレーキ側は両方 0
    rows = make_rows(
        40, lambda t: 20.0 + 2.0 * t, accel=14.0, accel_cmd_pct=24.0, brake_cmd_pct=0.0
    )
    p = write_csv(tmp_path / "a.csv", rows)
    assert np.allclose(read(p, opening_source="actual").u_pct, 10.0)
    assert np.allclose(read(p, opening_source="cmd").u_pct, 20.0)
    with pytest.raises(ValueError):
        read(p, opening_source="x")


def test_default_opening_source_is_cmd_and_missing_cmd_is_dropped(tmp_path: Path) -> None:
    rows = make_rows(30, lambda t: 20.0 + t, accel=14.0, accel_cmd_pct=24.0, brake_cmd_pct=0.0)
    rows[10]["accel_cmd_pct"] = ""
    p = write_csv(tmp_path / "a.csv", rows)
    r = read_fwd_rows(p, accel_deadband_pct=AD, brake_deadband_pct=BD)
    assert np.allclose(r.u_pct, 20.0)
    assert r.dropped[DROP_NO_OPENING] == 1


def test_coast_rows_use_coast_lag(tmp_path: Path) -> None:
    def speed(t: float) -> float:
        return 20.0 + t if t < 5.0 else 25.0 + 3.0 * (t - 5.0)

    p = write_csv(tmp_path / "a.csv", make_rows(100, speed, accel_cmd_pct=0.0, brake_cmd_pct=0.0))
    k_default = read(p, opening_source="cmd", accel_lag_s=0.0)
    k_coast = read(p, opening_source="cmd", accel_lag_s=0.0, coast_lag_s=0.4)
    i = int(np.argmin(np.abs(k_default.t_s - 4.4)))
    j = int(np.argmin(np.abs(k_coast.t_s - 4.4)))
    assert k_coast.accel_kmhs[j] > k_default.accel_kmhs[i]  # 0.4 秒先の窓には +3 側が入る
