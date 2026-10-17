"""fwd_data（順モデルの学習用の行）のテスト。合成の小さな CSV だけを使う。"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import pytest

from tests.research.fwd_data import (
    DROP_ACTUATOR_LAG,
    DROP_BOTH_PEDALS,
    DROP_GOVERNOR,
    DROP_NO_OPENING,
    DROP_NO_WINDOW,
    DROP_REASONS,
    DROP_STOPPED,
    FwdRows,
    cell_counts,
    concat_rows,
    effective_opening,
    launch_mask,
    read_fwd_rows,
    release_amount,
    take_rows,
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


# ── 2026-10-10 段2l-10 ウ: 指令と実開度がずれた行を外す ─────────────────────────


def _lag_rows(tmp_path: Path, *, actual: object = 14.0, cmd: float = 20.0) -> Path:
    rows = make_rows(30, lambda t: 20.0 + t, accel=actual, accel_cmd_pct=cmd, brake_cmd_pct=0.0)
    return write_csv(tmp_path / "lag.csv", rows)


def test_rows_with_cmd_actual_gap_over_tolerance_are_dropped(tmp_path: Path) -> None:
    r = read(_lag_rows(tmp_path, actual=14.0, cmd=20.0), opening_source="cmd")  # ずれ 6%
    assert r.dropped[DROP_ACTUATOR_LAG] == 30
    assert len(r) == 0
    assert r.dropped_seconds()[DROP_ACTUATOR_LAG] == pytest.approx(30 * r.dt_s)
    assert DROP_ACTUATOR_LAG in DROP_REASONS


def test_rows_within_tolerance_are_kept(tmp_path: Path) -> None:
    r = read(_lag_rows(tmp_path, actual=19.0, cmd=20.0), opening_source="cmd")  # ずれ 1%
    assert r.dropped[DROP_ACTUATOR_LAG] == 0
    assert len(r) > 0


def test_rows_without_actual_opening_are_not_dropped_for_lag(tmp_path: Path) -> None:
    r = read(_lag_rows(tmp_path, actual="", cmd=20.0), opening_source="cmd")
    assert r.dropped[DROP_ACTUATOR_LAG] == 0
    assert len(r) > 0


def test_lag_tolerance_none_disables_the_drop(tmp_path: Path) -> None:
    r = read(
        _lag_rows(tmp_path, actual=14.0, cmd=20.0), opening_source="cmd", actuator_lag_tol_pct=None
    )
    assert r.dropped[DROP_ACTUATOR_LAG] == 0
    assert len(r) > 0


def test_lag_tolerance_is_adjustable(tmp_path: Path) -> None:
    path = _lag_rows(tmp_path, actual=14.0, cmd=20.0)
    r = read(path, opening_source="cmd", actuator_lag_tol_pct=10.0)
    assert r.dropped[DROP_ACTUATOR_LAG] == 0


def test_keep_stopped_keeps_stopped_rows_only_when_asked(tmp_path: Path) -> None:
    rows = make_rows(30, lambda t: 0.2 if t < 1.0 else 0.2 + 3.0 * (t - 1.0), brake=9.0)
    path = write_csv(tmp_path / "s.csv", rows)
    default = read(path)
    kept = read(path, keep_stopped=True)
    assert default.dropped[DROP_STOPPED] > 0
    assert kept.dropped[DROP_STOPPED] == 0
    assert len(kept) > len(default)
    assert float(kept.speed_kmh.min()) < 0.5 <= float(default.speed_kmh.min())


def synth_launch_rows(speed, u, accel, t=None, dt: float = 0.1) -> FwdRows:
    n = len(speed)
    return FwdRows(
        speed_kmh=np.asarray(speed, dtype=float),
        u_pct=np.asarray(u, dtype=float),
        accel_kmhs=np.asarray(accel, dtype=float),
        t_s=np.arange(n) * dt if t is None else np.asarray(t, dtype=float),
        dt_s=dt,
        source="synthetic",
        dt_row_s=np.full(n, dt),
    )


def test_launch_mask_stop_then_rising_is_true_until_accel_pedal() -> None:
    speed = [0.0, 0.0, 0.3, 1.0, 2.0, 3.0, 4.0, 5.0]
    u = [-3.0, -3.0, -3.0, -3.0, -3.0, 2.0, 2.0, 2.0]  # 5 行目からアクセル
    accel = [0.0, 0.0, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]
    m = launch_mask(synth_launch_rows(speed, u, accel))
    assert m.tolist() == [True] * 5 + [False] * 3


def test_launch_mask_ends_when_slowing_to_stop() -> None:
    # 走行 → 減速して停車 → ブレーキを緩めて発進
    speed = [10.0, 5.0, 2.0, 0.8, 0.2, 0.0, 0.6, 1.5]
    u = [-5.0, -5.0, -5.0, -5.0, -5.0, -3.0, -3.0, -3.0]
    accel = [-5.0, -4.0, -3.0, -2.0, -0.1, 0.0, 1.5, 1.5]
    m = launch_mask(synth_launch_rows(speed, u, accel))
    # 減速して止まる行（0〜3）は False。停車の行 4 から区間が始まる
    assert m.tolist() == [False, False, False, False, True, True, True, True]


def test_launch_mask_cuts_on_time_gap_and_needs_a_stop_row() -> None:
    speed = [0.0, 0.5, 1.0, 1.5, 2.0]
    u = [-3.0] * 5
    accel = [0.0, 1.5, 1.5, 1.5, 1.5]
    t = [0.0, 0.1, 0.2, 5.0, 5.1]  # 3 行目で時刻が飛ぶ
    m = launch_mask(synth_launch_rows(speed, u, accel, t=t))
    assert m.tolist() == [True, True, True, False, False]
    # 停車の行が無ければ全部 False
    none = launch_mask(synth_launch_rows([1.0, 2.0, 3.0], [-3.0] * 3, [1.5] * 3))
    assert not none.any()


def test_launch_mask_stopped_row_with_accel_pedal_does_not_start() -> None:
    speed = [0.0, 0.0, 1.0, 2.0]
    u = [2.0, 2.0, -3.0, -3.0]  # 停車中にアクセルを踏んでいる
    accel = [0.0, 0.0, 1.5, 1.5]
    m = launch_mask(synth_launch_rows(speed, u, accel))
    assert not m.any()


def test_take_rows_selects_by_mask() -> None:
    r = synth_launch_rows([0.0, 1.0, 2.0], [-1.0, -2.0, -3.0], [0.0, 1.0, 2.0])
    sub = take_rows(r, np.array([True, False, True]))
    assert sub.speed_kmh.tolist() == [0.0, 2.0]
    assert sub.dt_row_s.tolist() == [0.1, 0.1]
    assert len(sub) == 2


# ───────── 2026-10-14 段2o-5: 戻し量 ─────────


def test_release_amount_is_distance_from_recent_min_for_negative_u() -> None:
    t = np.arange(8) * 1.0
    u = np.array([0.0, -2.0, -6.0, -6.0, -4.0, -3.0, -3.0, -3.0])
    # 窓 3 秒（その点を含む）: 点 4 の窓は t=1..4 → 最小 −6 → 戻し 2、点 5 は t=2..5 → 3
    # 点 7 は t=4..7 → 最小 −4 → 戻し 1（山が窓から出ると戻し量が縮む）
    rel = release_amount(t, u, 3.0)
    assert rel.tolist() == [0.0, 0.0, 0.0, 0.0, 2.0, 3.0, 3.0, 1.0]


def test_release_amount_zero_for_nonnegative_u_and_ignores_nan() -> None:
    t = np.arange(5) * 1.0
    u = np.array([-5.0, 0.0, 3.0, np.nan, -1.0])
    rel = release_amount(t, u, 10.0)
    # u>=0 の点・NaN の点は 0。最後の点は窓の最小 −5 から 4 戻している
    assert rel.tolist() == [0.0, 0.0, 0.0, 0.0, 4.0]


def test_read_fwd_rows_release_uses_full_series_before_dropping(tmp_path: Path) -> None:
    # 停車（捨てる行）で深く踏んでいて、動き出してから浅くなる: 捨てた停車の行の u も山に入る
    def speed(t: float) -> float:
        return 0.0 if t < 1.0 else 3.0 * (t - 1.0)

    rows = make_rows(40, speed, brake=BD + 6.0)  # u = −6
    for r in rows:
        if r["elapsed_s"] >= 2.0:
            r["brake_actual_pct"] = BD + 3.0  # u = −3（戻した）
    r = read(write_csv(tmp_path / "r.csv", rows))
    assert len(r.release_pct) == len(r) > 0
    after = r.t_s >= 2.0
    # 山 −6 は t < 1.0 の停車の行（学習では捨てる）にもあり、窓 3 秒の内なら戻し 3
    assert np.allclose(r.release_pct[after & (r.t_s <= 3.0)], 3.0)
    assert np.all(r.release_pct[~after] == 0.0)
    # 窓から山が出れば 0
    assert np.all(r.release_pct[r.t_s > 4.5] == 0.0)


def test_take_and_concat_rows_carry_release() -> None:
    a = synth_launch_rows([1.0, 2.0, 3.0], [-1.0, -2.0, -3.0], [0.0, 1.0, 2.0])
    a = FwdRows(**{**a.__dict__, "release_pct": np.array([0.0, 0.5, 1.5])})
    b = FwdRows(**{**a.__dict__, "release_pct": np.array([2.0, 0.0, 0.1])})
    sub = take_rows(a, np.array([False, True, True]))
    assert sub.release_pct.tolist() == [0.5, 1.5]
    both = concat_rows([a, b])
    assert both.release_pct.tolist() == [0.0, 0.5, 1.5, 2.0, 0.0, 0.1]


def test_fwdrows_without_release_defaults_to_zeros() -> None:
    r = synth_launch_rows([1.0, 2.0], [-1.0, -2.0], [0.0, 1.0])
    assert r.release_pct.tolist() == [0.0, 0.0]
    assert concat_rows([r, r]).release_pct.tolist() == [0.0] * 4


def test_read_fwd_rows_marks_launch_rows_and_carries_them(tmp_path: Path) -> None:
    """2026-10-14 段2o-5: 停車 → ブレーキを踏んだまま動き出す行は launch_row。走行中は False。"""

    def speed(t: float) -> float:
        return 0.0 if t < 1.0 else min(3.0 * (t - 1.0), 12.0)

    stop = make_rows(30, speed, brake=BD + 6.0)  # 約 0〜3 秒: 発進（アクセルなし・加速中）
    cruise = make_rows(30, lambda t: 40.0 + 2.0 * (t - 10.0), brake=BD + 3.0, t0=10.0)
    path = write_csv(tmp_path / "l.csv", stop + cruise)
    r = read(path)
    assert len(r.launch_row) == len(r)
    assert r.launch_row[r.t_s < 3.0].all() and r.launch_row.any()
    assert not r.launch_row[r.t_s >= 10.0].any()
    assert take_rows(r, r.launch_row).launch_row.all()
    assert concat_rows([r, r]).launch_row.sum() == 2 * r.launch_row.sum()
    off = read(path, mark_launch=False)
    assert not off.launch_row.any()
