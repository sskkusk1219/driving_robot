"""local_gain（手順2 の dither から局所の効きの倍率・遅れを測る）のテスト。2026-10-28 段3。

合成の時系列（段ごとに加速度を決めて車速を積分）で、実測が表の k 倍の傾き → 倍率 k、clip、
捨てる条件、マスの割り当て、引く関数の補間、遅れの 50% を確かめる。pkl・CSV の入出口は
fwd_tool のサブコマンドと pattern_drive の取り込みで確かめる。
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import pytest

from tests.research import fwd_tool, local_gain
from tests.research import play_onset as po
from tests.research.fwd_map import POINT_MEASURED, ForwardTable, load_forward_map, save_table

BMIN, AMIN = 6.0, 10.0
DT = 0.05
HOLD = 1.5


def _flat_table(slope_brake: float = 0.6, slope_accel: float = 0.3, **meta) -> ForwardTable:
    """車速によらない表: a = −2 + (u≥0: slope_accel·u / u<0: slope_brake·u)。u の格子は −30..30。"""
    rep = np.array([0.0, 50.0, 100.0, 150.0])
    grid = np.arange(-30.0, 31.0, 1.0)
    base = np.full((4, len(grid)), -2.0)
    accel = base + np.where(grid >= 0, slope_accel * grid, slope_brake * grid)[None, :]
    m = {"accel_deadband_pct": AMIN, "brake_deadband_pct": BMIN}
    m.update(meta)
    return ForwardTable(
        speed_edges=np.array([-25.0, 25.0, 75.0, 125.0, 175.0]),
        speed_rep=rep, u_grid=grid, accel_grid=accel,
        seconds=np.full(accel.shape, 10.0),
        point_kind=np.full(accel.shape, POINT_MEASURED, dtype=int),
        meas_u_min=np.full(4, -30.0), meas_u_max=np.full(4, 30.0), meta=m,
    )


def _synth(
    pedal: str, v0: float, openings: list[float], *, scale: float, table_slope: float,
    delay_s: float = 0.0, t0: float = 0.0, pre_s: float = 0.0, pre_open: float | None = None,
) -> local_gain.DitherRun:
    """段ごとの開度 openings を HOLD 秒ずつ。加速度 = −2 + scale·table_slope·u（u はブレーキが負）を
    delay_s 遅らせて車速を積分する。pre_s 秒の前置き（開度 pre_open）を付けると、最初の段が
    切り替えになる。"""
    t_list, v_list, c_list = [], [], []
    v, t = v0, t0
    mn = BMIN if pedal == "brake" else AMIN
    seq = ([pre_open] * int(pre_s / DT) if pre_s and pre_open is not None else []) + [
        o for o in openings for _ in range(int(HOLD / DT))
    ]
    hist: list[float] = []
    for c in seq:
        hist.append(c)
        eff = hist[max(0, len(hist) - 1 - int(round(delay_s / DT)))]
        d = eff - mn
        u = -d if pedal == "brake" else d
        a = -2.0 + scale * table_slope * u
        v += a * DT
        t += DT
        t_list.append(t)
        v_list.append(v)
        c_list.append(c)
    cmd = np.array(c_list)
    vv = np.array(v_list)
    vv += v0 - float(vv.mean())  # v0 は区間の平均車速（積分の傾きは変えず、平行に動かす）
    return local_gain.DitherRun(pedal, np.array(t_list), vv, cmd, cmd.copy())


def _brake_run(v0: float, scale: float, open0: float = 10.0, delta: float = 0.5, **kw):
    return _synth(
        "brake", v0, [open0 + delta, open0, open0 - delta, open0],
        scale=scale, table_slope=0.6, **kw,
    )


def _fit(runs, table=None, onset=2.0, reps=(90.0, 60.0, 30.0), pedal="brake"):
    table = table or _flat_table()
    return local_gain.fit_pedal(
        runs, pedal, table, pedal_min_pct=BMIN if pedal == "brake" else AMIN,
        onset_u_pct=onset, speed_reps_kmh=list(reps) if pedal == "brake" else None,
    )


def test_entry_stub_row_at_u0_is_not_counted_as_a_step() -> None:
    """入った直後の u0 の短い行（入る前の u から跳んだ直後）は段に数えず、4 段だけで当てる。
    倍率は前置きなしの区間と同じ。"""
    plain = _brake_run(60.0, 2.0)
    stub = _brake_run(60.0, 2.0, pre_s=DT, pre_open=10.0)
    assert len(local_gain.split_steps(stub.cmd_pct)) == 5
    assert len(local_gain.dither_steps(stub)) == 4
    assert len(local_gain.dither_steps(plain)) == 4
    n_stub = len(local_gain.step_stats(stub, local_gain.dither_steps(stub)))
    assert n_stub == 4
    f_plain, f_stub = _fit([plain]), _fit([stub])
    assert f_stub.entry is not None and f_plain.entry is not None
    assert f_stub.entry["ratio"][0][0] == pytest.approx(f_plain.entry["ratio"][0][0])


def test_split_steps_cuts_at_command_jumps() -> None:
    cmd = np.array([5.0, 5.0, 5.0, 6.0, 6.0, 5.5, 5.5, 5.5])
    assert local_gain.split_steps(cmd) == [(0, 3), (3, 5), (5, 8)]
    assert local_gain.split_steps(np.array([])) == []


def test_ratio_is_measured_slope_over_table_slope() -> None:
    fit = _fit([_brake_run(60.0, 2.0)])
    assert fit.entry is not None
    assert fit.entry["speed_rep_kmh"] == [60.0]
    # u0 の深さ 4.0% − 効き始め 2.0% = 2.0%（0.5% 刻み）
    assert fit.entry["u_from_onset_pct"] == [2.0]
    assert abs(fit.entry["ratio"][0][0] - 2.0) < 0.1
    assert fit.entry["n"] == [[1]]
    assert (fit.n_runs, fit.n_used, fit.n_cells) == (1, 1, 1)


def test_ratio_is_clipped_to_one_through_four() -> None:
    hi = _fit([_brake_run(60.0, 6.0)])
    lo = _fit([_brake_run(60.0, 0.5)])
    assert hi.entry["ratio"][0][0] == local_gain.GAIN_MAX
    assert lo.entry["ratio"][0][0] == local_gain.GAIN_MIN  # 1 未満は 1（FB を強くしない）


def test_run_is_dropped_when_table_slope_or_opening_span_is_too_small() -> None:
    flat = _fit([_brake_run(60.0, 2.0)], table=_flat_table(slope_brake=0.02))
    assert flat.entry is None and flat.n_used == 0  # 表の傾き 0.02 < 0.05
    narrow = _fit([_brake_run(60.0, 2.0, delta=0.05)])  # 実開度の幅 0.1% < 0.15%
    assert narrow.entry is None and narrow.n_used == 0
    # 実開度の幅の境目（0.3% > 0.15%）は使う
    assert _fit([_brake_run(60.0, 2.0, delta=0.15)]).n_used == 1


def test_cells_use_nearest_speed_rep_and_median_over_runs() -> None:
    runs = [
        _brake_run(85.0, 2.0), _brake_run(88.0, 3.0), _brake_run(95.0, 2.5),  # → 90 の行
        _brake_run(33.0, 1.5, open0=11.0),  # → 30 の行・列は深さ 5.0 − 2.0 = 3.0
    ]
    fit = _fit(runs)
    e = fit.entry
    assert e["speed_rep_kmh"] == [30.0, 90.0]
    assert e["u_from_onset_pct"] == [2.0, 2.5, 3.0]
    assert e["n"] == [[0, 0, 1], [3, 0, 0]]
    assert abs(e["ratio"][1][0] - 2.5) < 0.15  # 中央値
    assert e["ratio"][0][0] == 1.0  # 測っていないマスは 1・回数 0


def test_slow_runs_below_7_5_kmh_are_not_tabled() -> None:
    assert _fit([_brake_run(8.0, 2.0)]).entry is None  # 平均車速は 7.5 未満になる


def test_accel_uses_20kmh_speed_rep_and_positive_u() -> None:
    run = _synth("accel", 52.0, [14.5, 14.0, 13.5, 14.0], scale=2.5, table_slope=0.3)
    fit = _fit([run], pedal="accel", onset=2.0)
    e = fit.entry
    assert e["speed_rep_kmh"] == [60.0]  # 車速は 52 付近から −2 km/h/s で増える・平均 ≈ 50〜60 → 60
    assert e["u_from_onset_pct"] == [2.0]  # 14.0 − 10.0 − 2.0
    assert abs(e["ratio"][0][0] - 2.5) < 0.2
    assert local_gain.accel_speed_rep(5.0) == 20.0  # 最小は 1 刻み
    assert local_gain.accel_speed_rep(31.0) == 40.0


def test_no_onset_gives_no_table_but_still_measures_lag() -> None:
    fit = _fit([_brake_run(60.0, 2.0, delay_s=0.25, pre_s=1.0, pre_open=10.0)], onset=None)
    assert fit.entry is None and "効き始め" in fit.reason
    assert fit.lag_s is not None


def test_local_gain_at_interpolates_and_returns_one_outside() -> None:
    entry = {
        "speed_rep_kmh": [30.0, 90.0],
        "u_from_onset_pct": [0.0, 0.5, 1.0],
        "ratio": [[2.0, 3.0, 1.0], [1.0, 4.0, 1.0]],
        "n": [[2, 1, 0], [0, 3, 0]],
    }
    at = local_gain.local_gain_at
    assert at(entry, 30.0, 0.0) == 2.0
    assert at(entry, 30.0, 0.25) == 2.5  # 列の間
    assert at(entry, 60.0, 0.5) == 3.5  # 行の間（3.0 と 4.0）
    assert at(entry, 60.0, 0.0) == 2.0  # 90 の行は 0.0 を測っていないので 30 の行だけ
    assert at(entry, 30.0, 1.0) == 1.0  # どの行も測った範囲の外は 1
    # 測っていないマスは補間に入れない（0.5 と 1.5 の間は 2 つの測った値の間）
    gap = {"speed_rep_kmh": [60.0], "u_from_onset_pct": [0.5, 1.0, 1.5],
           "ratio": [[1.0, 1.0, 3.0]], "n": [[2, 0, 2]]}
    assert at(gap, 60.0, 1.0) == 2.0
    # 車速の範囲外は端の行のまま
    assert at(entry, 100.0, 0.5) == 4.0
    assert at(entry, 20.0, 0.5) == 3.0
    # 1 行だけの表もその行の値
    one = {"speed_rep_kmh": [90.0], "u_from_onset_pct": [0.0, 0.5],
           "ratio": [[2.0, 2.0]], "n": [[1, 1]]}
    assert at(one, 30.0, 0.25) == 2.0
    # 開度の範囲外・低速・表なしは 1
    assert at(entry, 60.0, -0.5) == 1.0
    assert at(entry, 60.0, 1.5) == 1.0
    assert at(entry, 6.0, 0.5) == 1.0
    assert at(entry, 7.0, 0.5, min_speed_kmh=7.5) == 1.0
    assert at(None, 60.0, 0.5) == 1.0
    assert at({}, 60.0, 0.5) == 1.0


def test_boundary_lag_is_command_to_half_of_accel_change() -> None:
    # 指令の切り替え（前置き 10.0 → 10.5）から 0.25 s 遅れて効く
    run = _brake_run(60.0, 2.0, delay_s=0.25, pre_s=1.5, pre_open=10.0)
    steps = local_gain.split_steps(run.cmd_pct)
    stats = local_gain.step_stats(run, steps)
    lag = local_gain.boundary_lag_s(run, stats[0], stats[1])
    assert lag is not None and 0.1 <= lag <= 0.4
    fit = _fit([run])
    assert fit.lag_s is not None and 0.1 <= fit.lag_s <= 0.4
    assert fit.lag_n >= 1


def test_boundary_lag_is_none_without_command_or_accel_change() -> None:
    run = _brake_run(60.0, 2.0)
    stats = local_gain.step_stats(run, local_gain.split_steps(run.cmd_pct))
    flat = local_gain.StepStat(stats[1].i0, stats[1].i1, stats[0].accel_kmhs, 10.0, 60.0)
    assert local_gain.boundary_lag_s(run, stats[0], flat) is None  # 加速度が変わらない


def _write_dither_csv(path: Path, runs: list[local_gain.DitherRun], *, gap: bool = True) -> None:
    cols = ["section", "phase", "elapsed_s", "actual_speed_kmh", "accel_cmd_pct", "brake_cmd_pct",
            "accel_actual_pct", "brake_actual_pct"]
    t_off = 0.0
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for run in runs:
            phase = "RAMP_BRAKE_DITHER" if run.pedal == "brake" else "RAMP_ACCEL_DITHER"
            for t, v, c, a in zip(run.t_s, run.speed_kmh, run.cmd_pct, run.actual_pct, strict=True):
                ac, bc = (AMIN, c) if run.pedal == "brake" else (c, BMIN)
                aa, ba = (AMIN, a) if run.pedal == "brake" else (a, BMIN)
                w.writerow(["PATTERN_DRIVE", phase, f"{t + t_off:.3f}", f"{v:.4f}",
                            ac, bc, aa, ba])
            t_off += run.t_s[-1] + 5.0
            w.writerow(
                ["PATTERN_DRIVE", "RAMP_DOWN", f"{t_off - 2.0:.3f}", "50", AMIN, BMIN, AMIN, BMIN]
            )


def test_read_dither_runs_splits_regions_and_pedals(tmp_path: Path) -> None:
    p = tmp_path / "d.csv"
    accel = _synth("accel", 52.0, [14.5, 14.0, 13.5, 14.0], scale=2.0, table_slope=0.3)
    _write_dither_csv(p, [_brake_run(60.0, 2.0), _brake_run(30.0, 2.0), accel])
    runs = local_gain.read_dither_runs(p)
    assert [r.pedal for r in runs] == ["brake", "brake", "accel"]
    assert all(len(r.t_s) == 120 for r in runs)


def test_attach_to_meta_puts_json_plain_values() -> None:
    import json

    fit_b = _fit([_brake_run(60.0, 2.0, delay_s=0.25, pre_s=1.5, pre_open=10.0)])
    fit_a = local_gain.LocalGainFit(entry=None)
    meta: dict = {}
    assert local_gain.attach_to_meta(meta, fit_b, fit_a, "x.csv")
    assert set(meta) >= {"local_gain_brake", "dither_brake_lag_s", "dither_brake_lag_n",
                         "local_gain_source"}
    assert "local_gain_accel" not in meta and "dither_accel_lag_s" not in meta
    json.dumps(meta)  # 素の list・float・int だけ
    empty: dict = {}
    assert not local_gain.attach_to_meta(empty, fit_a, fit_a, "x.csv") and empty == {}


def test_play_onset_read_csv_excludes_dither_rows(tmp_path: Path) -> None:
    p = tmp_path / "o.csv"
    with p.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pattern", "phase", "elapsed_s", "actual_speed_kmh", "accel_actual_pct",
                    "brake_actual_pct"])
        for i, ph in enumerate(["RAMP_DOWN", "RAMP_BRAKE_DITHER", "RAMP_ACCEL_DITHER", "HOLD_STEP",
                                "RAMP_UP"]):
            w.writerow(["G_RAMP", ph, i, 50, 10, 8])
    data = po._read_csv(p, "G_RAMP")
    assert list(data["t"]) == [0.0, 4.0]  # RAMP_DOWN と RAMP_UP だけ


def _attach_fixture(tmp_path: Path) -> tuple[Path, Path, Path]:
    src = tmp_path / "in.pkl"
    save_table(_flat_table(brake_onset_u_pct=2.0, accel_onset_u_pct=2.0), src)
    csvp = tmp_path / "d.csv"
    _write_dither_csv(csvp, [
        _brake_run(60.0, 2.0, delay_s=0.25, pre_s=1.5, pre_open=10.0),
        _synth("accel", 52.0, [14.5, 14.0, 13.5, 14.0], scale=2.0, table_slope=0.3),
    ])
    return src, csvp, tmp_path / "out.pkl"


def test_fwd_tool_attach_local_gain_writes_new_pkl(tmp_path: Path) -> None:
    src, csvp, out = _attach_fixture(tmp_path)
    before = load_forward_map(src).meta
    rc = fwd_tool.main(
        ["attach-local-gain", "--pkl", str(src), "--csv", str(csvp), "--out", str(out)]
    )
    assert rc == 0
    meta = load_forward_map(out).meta
    assert meta["local_gain_source"] == "d.csv"
    assert abs(meta["local_gain_brake"]["ratio"][0][0] - 2.0) < 0.2
    assert abs(meta["local_gain_accel"]["ratio"][0][0] - 2.0) < 0.2
    assert 0.1 <= meta["dither_brake_lag_s"] <= 0.4
    assert "local_gain_brake" not in before  # 元の pkl は書き換えない
    assert "local_gain_brake" not in load_forward_map(src).meta


def test_fwd_tool_attach_local_gain_refuses_overwrite_and_empty(tmp_path: Path) -> None:
    src, csvp, out = _attach_fixture(tmp_path)
    out.write_bytes(b"x")
    assert fwd_tool.main(["attach-local-gain", "--pkl", str(src), "--csv", str(csvp),
                          "--out", str(out)]) == 2
    assert fwd_tool.main(["attach-local-gain", "--pkl", str(src), "--csv", str(csvp),
                          "--out", str(src)]) == 2
    nod = tmp_path / "nod.csv"
    nod.write_text("section,phase,elapsed_s,actual_speed_kmh,accel_cmd_pct,brake_cmd_pct\n"
                   "PATTERN_DRIVE,RAMP_DOWN,0,50,10,6\n")
    assert fwd_tool.main(["attach-local-gain", "--pkl", str(src), "--csv", str(nod),
                          "--out", str(tmp_path / "o2.pkl")]) == 1
    assert not (tmp_path / "o2.pkl").exists()
