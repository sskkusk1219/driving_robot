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


# ── 段7（2026-11-10 ProblemReport_20261109）: 定速の保持からの局所の効き ──────


def _cruise_rows(
    scale: float, *, v0: float = 60.0, base_u: float | None = None, amp: float = 0.4,
    lag_s: float = 0.14,
    t0: float = 0.0, dur_s: float = 8.0, phase: str = "RAMP_CRUISE", table_slope: float = 0.3,
    freq_hz: float = 0.4,
) -> list[dict[str, float | str]]:
    """加速度 = −2 + scale·table_slope·(遅れ lag_s 前の u)。開度 = AMIN + base_u + 正弦のゆらぎ
    （保持の PI が動かす開度の代わり）。車速は加速度を積分。"""
    rows = []
    v = v0
    n = int(dur_s / DT)
    if base_u is None:
        base_u = 2.0 / (scale * table_slope)  # 加速度 0（定速）になる開度
    op = [AMIN + base_u + amp * np.sin(2 * np.pi * freq_hz * k * DT) for k in range(n)]
    for k in range(n):
        eff = op[max(0, k - int(round(lag_s / DT)))]
        a = -2.0 + scale * table_slope * (eff - AMIN)
        rows.append({
            "elapsed_s": t0 + k * DT, "actual_speed_kmh": v, "accel_actual_pct": op[k],
            "brake_actual_pct": BMIN, "phase": phase,
        })
        v += a * DT
    return rows


def _write_rows(path: Path, rows: list[dict[str, float | str]]) -> None:
    cols = ["elapsed_s", "actual_speed_kmh", "accel_actual_pct", "brake_actual_pct", "phase"]
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)


def _cruise_fit(rows, tmp_path: Path, table=None) -> local_gain.CruiseFit:
    table = table or _flat_table(accel_onset_u_pct=2.0, accel_lag_s=0.14)
    p = tmp_path / "c.csv"
    _write_rows(p, rows)
    segs = local_gain.read_cruise_segments(p, accel_min_pct=AMIN, brake_min_pct=BMIN)
    return local_gain.fit_cruise(
        segs, table, accel_min_pct=AMIN, onset_u_pct=2.0, lag_s=0.14
    )


@pytest.mark.parametrize("k", [1.5, 3.0, 3.7])
def test_cruise_ratio_is_k_times_table_slope(tmp_path: Path, k: float) -> None:
    fit = _cruise_fit(_cruise_rows(k), tmp_path)
    assert fit.entry is not None and fit.n_used == 1
    assert fit.ratios[0] == pytest.approx(k, rel=0.05)
    # 行 = 60 km/h、列 = (u − 効き始め) = 約 0〜2.4 → 5% 刻みで 0 列目
    assert fit.entry["speed_rep_kmh"] == [60.0]
    assert fit.entry["u_from_onset_pct"] == [0.0]
    assert fit.entry["ratio"][0][0] == pytest.approx(min(4.0, k), rel=0.05)
    assert fit.entry["n"] == [[1]]


def test_cruise_rejects_small_span_low_r2_and_few_rows(tmp_path: Path) -> None:
    assert _cruise_fit(_cruise_rows(3.0, amp=0.02), tmp_path).reject == {"span": 1}
    assert _cruise_fit(_cruise_rows(3.0, dur_s=0.8), tmp_path).reject == {"rows": 1}
    noisy = _cruise_rows(3.0)
    rng = np.random.default_rng(0)
    v = np.array([r["actual_speed_kmh"] for r in noisy]) + rng.normal(0, 0.3, len(noisy))
    for r, vv in zip(noisy, v, strict=True):
        r["actual_speed_kmh"] = vv
    assert _cruise_fit(noisy, tmp_path).reject == {"r2": 1}


def test_cruise_ignores_brake_slow_and_other_phases(tmp_path: Path) -> None:
    rows = _cruise_rows(3.0, v0=2.0, dur_s=2.0)  # 7.5 km/h 未満
    assert _cruise_fit(rows, tmp_path).n_segments == 0
    rows = _cruise_rows(3.0)
    for r in rows:
        r["phase"] = "RAMP_UP"
    assert _cruise_fit(rows, tmp_path).n_segments == 0
    rows = _cruise_rows(3.0)
    for r in rows:
        r["brake_actual_pct"] = BMIN + 2.0
    assert _cruise_fit(rows, tmp_path).n_segments == 0
    rows = _cruise_rows(3.0)
    for r in rows:
        r["accel_actual_pct"] = AMIN  # アクセルが min
    assert _cruise_fit(rows, tmp_path).n_segments == 0


def test_cruise_splits_segments_on_gap_and_phase_and_both_phases_count(tmp_path: Path) -> None:
    a = _cruise_rows(3.0, phase="RAMP_CRUISE")
    b = _cruise_rows(3.0, phase="RAMP_HOLD", t0=100.0, v0=100.0)
    fit = _cruise_fit(a + b, tmp_path)
    assert fit.n_segments == 2 and fit.n_used == 2
    assert fit.entry["speed_rep_kmh"] == [60.0, 100.0]


def test_cruise_attach_keeps_dither_key_and_cli_writes_new_pkl(tmp_path: Path) -> None:
    src = tmp_path / "in.pkl"
    save_table(
        _flat_table(accel_onset_u_pct=2.0, accel_lag_s=0.14, local_gain_accel={"sentinel": 1}), src
    )
    csvp = tmp_path / "c.csv"
    _write_rows(csvp, _cruise_rows(3.0))
    out = tmp_path / "out.pkl"
    assert fwd_tool.main(
        ["attach-local-gain", "--pkl", str(src), "--csv", str(csvp), "--out", str(out)]
    ) == 0
    meta = load_forward_map(out).meta
    assert meta["local_gain_accel"] == {"sentinel": 1}  # dither のキーは触らない
    assert meta["local_gain_accel_cruise"]["ratio"][0][0] == pytest.approx(3.0, rel=0.05)
    assert meta["local_gain_accel_cruise_source"] == "c.csv"
    assert meta["local_gain_accel_cruise_n_used"] == 1
    assert "local_gain_accel_cruise" not in load_forward_map(src).meta


def test_local_gain_at_edge_hold_extends_measured_columns() -> None:
    entry = {"speed_rep_kmh": [60.0], "u_from_onset_pct": [5.0], "ratio": [[3.0]], "n": [[1]]}
    assert local_gain.local_gain_at(entry, 60.0, 6.5) == 1.0  # 既定は範囲の外 → 1
    assert local_gain.local_gain_at(entry, 60.0, 6.5, edge_hold_pct=2.5) == pytest.approx(3.0)
    assert local_gain.local_gain_at(entry, 60.0, 8.0, edge_hold_pct=2.5) == 1.0


def test_hold_step_ratio_open_loop_check(tmp_path: Path) -> None:
    # 開度 AMIN+6 → AMIN+9 の階段。加速度 = −2 + 3·0.3·u（遅れ 0.14 s）→ 倍率 3
    rows = []
    v, t = 60.0, 0.0
    for k in range(int(8.0 / DT)):
        op = AMIN + (6.0 if t < 3.0 else 9.0)
        a = -2.0 + 0.9 * ((AMIN + (6.0 if t - 0.14 < 3.0 else 9.0)) - AMIN)
        rows.append({
            "elapsed_s": t, "actual_speed_kmh": v, "accel_actual_pct": op,
            "brake_actual_pct": BMIN, "phase": "HOLD_STEP",
        })
        v += a * DT
        t += DT
    p = tmp_path / "h.csv"
    _write_rows(p, rows)
    got = local_gain.hold_step_ratios(p, _flat_table(), accel_min_pct=AMIN, brake_min_pct=BMIN)
    assert len(got) == 1
    assert got[0][1] == pytest.approx(3.0, rel=0.1)


# ── 段10c（2026-11-10 ProblemReport_20261109）: 保持の正弦波（RAMP_CRUISE_SINE）からの 1 Hz
# 付近の効き ──

SINE_HZ = 1.2


def _sine_rows(
    scale: float, *, v0: float = 60.0, amp: float = 0.6, hz: float = SINE_HZ, periods: int = 6,
    lag_s: float = 0.14, t0: float = 0.0, table_slope: float = 0.3, first_period_junk: float = 0.0,
    noise: float = 0.0, seed: int = 0,
) -> list[dict[str, float | str]]:
    """開度 = AMIN + 定速の開度 + A·sin(2πft)。加速度 = −2 + scale·table_slope·(遅れ lag_s 前の u)。
    車速は積分。理論: |P| ＝ scale·table_slope ÷ (2πf) → 倍率 r ＝ scale。"""
    rng = np.random.default_rng(seed)
    n = int(round(periods / hz / DT))
    base_u = 2.0 / (scale * table_slope)
    op = [AMIN + base_u + amp * np.sin(2 * np.pi * hz * k * DT) for k in range(n)]
    rows = []
    v = v0
    for k in range(n):
        eff = op[max(0, k - int(round(lag_s / DT)))]
        a = -2.0 + scale * table_slope * (eff - AMIN)
        if first_period_junk and k * DT < 1.0 / hz:
            a += first_period_junk
        rows.append({
            "elapsed_s": t0 + k * DT, "actual_speed_kmh": v + float(rng.normal(0, noise)),
            "accel_actual_pct": op[k], "brake_actual_pct": BMIN, "phase": "RAMP_CRUISE_SINE",
        })
        v += a * DT
    return rows


def _sine_fit(rows, tmp_path: Path, table=None, **kw) -> local_gain.SineFit:
    table = table or _flat_table(accel_onset_u_pct=2.0, accel_lag_s=0.14)
    p = tmp_path / "s.csv"
    _write_rows(p, rows)
    segs = local_gain.read_sine_segments(p)
    return local_gain.fit_cruise_sine(segs, table, accel_min_pct=AMIN, hz=SINE_HZ, **kw)


@pytest.mark.parametrize("k", [1.0, 2.7, 3.65])
def test_sine_ratio_recovers_known_gain_and_phase(tmp_path: Path, k: float) -> None:
    fit = _sine_fit(_sine_rows(k), tmp_path)
    assert fit.entry is not None and fit.n_used == 1
    seg = fit.per_segment[0]
    assert seg["ratio"] == pytest.approx(k, rel=0.05)
    # |P| ＝ k·0.3 ÷ (2π·1.2)・位相 ＝ −90° − 360·f·(実効の遅れ 0.15 s（0.14 を DT 刻みに丸めた）
    # ＋ 離散の積分の DT/2)
    assert seg["abs_p"] == pytest.approx(k * 0.3 / (2 * np.pi * SINE_HZ), rel=0.05)
    assert seg["phase_deg"] == pytest.approx(-90.0 - 360.0 * SINE_HZ * (0.15 + DT / 2), abs=6.0)
    assert seg["snr"] > 10.0
    assert fit.entry["speed_rep_kmh"] == [60.0]
    assert fit.entry["ratio"][0] == pytest.approx(k, rel=0.05) and fit.entry["n"] == [1]
    assert local_gain.hf_ratio_at(fit.entry, 60.0) == pytest.approx(k, rel=0.05)
    assert local_gain.hf_ratio_at(fit.entry, 120.0) == pytest.approx(k, rel=0.05)  # 端の値
    assert local_gain.hf_ratio_at(None, 60.0) is None


def test_sine_fit_drops_first_period_and_rejects_low_snr(tmp_path: Path) -> None:
    # 最初の 1 周期に大きな過渡があっても、捨てるので倍率は変わらない
    fit = _sine_fit(_sine_rows(3.0, first_period_junk=4.0), tmp_path)
    assert fit.n_used == 1 and fit.per_segment[0]["ratio"] == pytest.approx(3.0, rel=0.08)
    # 車速のノイズが大きい → SNR 3 未満で捨てる
    noisy = _sine_fit(_sine_rows(3.0, noise=0.6), tmp_path)
    assert noisy.entry is None and noisy.reject == {"snr": 1}
    # 6 周期より短い（1 周期だけ）は行が足りない
    short = _sine_fit(_sine_rows(3.0, periods=1), tmp_path)
    assert short.entry is None and short.reject == {"rows": 1}


def test_sine_fit_rejects_invalid_table_sensitivity_and_splits_segments(tmp_path: Path) -> None:
    flat = _flat_table(slope_accel=0.0, accel_onset_u_pct=2.0)
    assert _sine_fit(_sine_rows(3.0), tmp_path, table=flat).reject == {"sens": 1}
    a = _sine_rows(3.0)
    b = _sine_rows(2.0, v0=100.0, t0=100.0)
    fit = _sine_fit(a + b, tmp_path)
    assert fit.n_segments == 2 and fit.n_used == 2
    assert fit.entry["speed_rep_kmh"] == [60.0, 100.0]
    assert fit.entry["ratio"] == pytest.approx([3.0, 2.0], rel=0.08)
    # RAMP_CRUISE の行は正弦波の区間に数えない
    for r in a:
        r["phase"] = "RAMP_CRUISE"
    p = tmp_path / "c.csv"
    _write_rows(p, a)
    assert local_gain.read_sine_segments(p) == []


def test_sine_attach_cli_and_meta_keys(tmp_path: Path) -> None:
    from tests.research.pattern_drive import _attach_cruise_sine_gain  # noqa: PLC0415

    table = _flat_table(accel_onset_u_pct=2.0, accel_lag_s=0.14)
    p = tmp_path / "s.csv"
    _write_rows(p, _sine_rows(3.0))
    assert _attach_cruise_sine_gain(table, p, SINE_HZ)
    m = table.meta
    assert m["local_gain_accel_hf_hz"] == SINE_HZ and m["local_gain_accel_hf_source"] == "s.csv"
    assert m["local_gain_accel_hf_n_segments"] == 1 and m["local_gain_accel_hf_n_used"] == 1
    assert m["local_gain_accel_hf"]["ratio"][0] == pytest.approx(3.0, rel=0.05)
    # 正弦波の区間が無い CSV（正弦波 off の手順2）は何も入れない
    q = tmp_path / "n.csv"
    _write_rows(q, _cruise_rows(3.0))
    t2 = _flat_table(accel_onset_u_pct=2.0)
    assert not _attach_cruise_sine_gain(t2, q, SINE_HZ) and "local_gain_accel_hf" not in t2.meta
    # attach-local-gain（CLI）は dither が無くても正弦波があれば pkl を作る
    src = tmp_path / "in.pkl"
    save_table(_flat_table(accel_onset_u_pct=2.0, accel_lag_s=0.14), src)
    out = tmp_path / "out.pkl"
    rc = fwd_tool.main(
        ["attach-local-gain", "--pkl", str(src), "--csv", str(p), "--out", str(out),
         "--sine-hz", str(SINE_HZ)]
    )
    assert rc == 0
    assert load_forward_map(out).meta["local_gain_accel_hf"]["ratio"][0] == pytest.approx(
        3.0, rel=0.05
    )


def test_cruise_sine_check_on_excite_blocks(tmp_path: Path, capsys: pytest.CaptureFixture) -> None:
    """加振 CSV の `<車速>kmh_<周波数>Hz` のブロック（EXCITE）に同じ解析を当てる。"""
    src = tmp_path / "in.pkl"
    save_table(_flat_table(accel_onset_u_pct=2.0, accel_lag_s=0.14), src)
    rows = []
    for r in _sine_rows(3.0, periods=10):
        rows.append({
            "section": "EXCITE", "pattern": "60kmh_1.20Hz", "mode_time_s": r["elapsed_s"],
            "actual_speed_kmh": r["actual_speed_kmh"], "accel_cmd_pct": r["accel_actual_pct"],
            "accel_actual_pct": r["accel_actual_pct"],
        })
    for r in _sine_rows(3.0, periods=10):  # 別の周波数のブロックは読まない
        rows.append({
            "section": "EXCITE", "pattern": "60kmh_1.00Hz", "mode_time_s": r["elapsed_s"],
            "actual_speed_kmh": r["actual_speed_kmh"], "accel_cmd_pct": r["accel_actual_pct"],
            "accel_actual_pct": r["accel_actual_pct"],
        })
    csvp = tmp_path / "ex.csv"
    with csvp.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    assert len(local_gain.read_excite_blocks(csvp, 1.2)) == 1
    rc = fwd_tool.main(["cruise-sine-check", "--csv", str(csvp), "--pkl", str(src), "--hz", "1.2"])
    out = capsys.readouterr().out
    assert rc == 0 and "60kmh_1.20Hz" in out and "1.00Hz" not in out
