"""release_gain（減速中のブレーキの戻しから、戻す側の効きを測る）のテスト。2026-11-05 段14。

合成の時系列（開度を決めて、加速度 = a0 + 効き × (開度の下がり) を遅れ L 後に車速へ積分）で、
効きが出る・加速中や止まった後の山は数えない・山 3 未満で測れず・meta に入る・
fwd_tool の付け足しを確かめる。
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from tests.research import fwd_tool, release_gain
from tests.research.fwd_map import POINT_MEASURED, ForwardTable, load_forward_map, save_table

DT = 0.05
LAG = 0.15
BMIN = 4.0
ONSET = 2.0
GAIN = 4.0  # km/h/s per %


def _table(**meta) -> ForwardTable:
    rep = np.array([0.0, 50.0])
    grid = np.arange(-10.0, 11.0, 1.0)
    acc = np.zeros((2, len(grid)))
    m = {"brake_deadband_pct": BMIN, "brake_onset_u_pct": ONSET, "brake_lag_s": LAG}
    m.update(meta)
    return ForwardTable(
        speed_edges=np.array([-25.0, 25.0, 75.0]), speed_rep=rep, u_grid=grid, accel_grid=acc,
        seconds=np.full(acc.shape, 10.0), point_kind=np.full(acc.shape, POINT_MEASURED, dtype=int),
        meas_u_min=np.full(2, -10.0), meas_u_max=np.full(2, 10.0), meta=m,
    )


def _peak_rows(
    t0: float, v0: float, a0: float, open0: float, *, drop: float = 0.4, hold_s: float = 1.0
) -> list[tuple[float, float, float]]:
    """開度 open0 で hold_s 秒（加速度 a0）→ 0.05 s ごとに 0.1% ずつ drop まで下がる → hold_s 秒。
    効きは「開度の下がり × GAIN」を遅れ LAG 後に足す。(時刻, 車速, 実開度)。"""
    n_hold = int(hold_s / DT)
    n_drop = int(round(drop / 0.1))
    opens = [open0] * n_hold + [open0 - 0.1 * (k + 1) for k in range(n_drop)]
    opens += [open0 - drop] * n_hold
    rows = []
    v, t = v0, t0
    lag_n = int(round(LAG / DT))
    for i, o in enumerate(opens):
        eff = opens[max(0, i - lag_n)]
        a = a0 + GAIN * (open0 - eff)
        rows.append((t, v, o))
        v += a * DT  # a は km/h/s、v は km/h
        t += DT
    return rows


def _write(path: Path, segments: list[list[tuple[float, float, float]]],
           section: str = "PATTERN_DRIVE") -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["section", "elapsed_s", "actual_speed_kmh", "brake_actual_pct"])
        for seg in segments:
            for t, v, b in seg:
                w.writerow([section, f"{t:.3f}", f"{v:.4f}", f"{b:.3f}"])


def _kw() -> dict:
    return {"brake_min_pct": BMIN, "onset_u_pct": ONSET, "lag_s": LAG}


def _decel_segments(n: int, *, v0: float = 5.0, a0: float = -3.0) -> list:
    # 山ごとに別の区間（行が飛ぶ）
    return [_peak_rows(100.0 * i, v0, a0, 9.0) for i in range(n)]


def test_gain_is_measured_from_peaks(tmp_path: Path) -> None:
    p = tmp_path / "a.csv"
    _write(p, _decel_segments(4))
    res = release_gain.measure_stop_release_gain([p], **_kw())
    assert res.n_peaks == 4
    assert res.gain_kmhs_pct is not None
    assert 3.0 < res.gain_kmhs_pct < 4.5  # 3 行の平滑化で少し小さく出る
    assert res.per_csv["a.csv"] and len(res.per_csv["a.csv"]) == 4


def test_accelerating_or_stopped_or_shallow_peaks_not_counted(tmp_path: Path) -> None:
    p = tmp_path / "a.csv"
    segs = _decel_segments(2)
    segs.append(_peak_rows(500.0, 5.0, +2.0, 9.0))  # 加速中
    segs.append(_peak_rows(600.0, 0.0, -3.0, 9.0))  # 停まった後（車速 0.5 未満）
    segs.append(_peak_rows(700.0, 5.0, -3.0, BMIN + ONSET - 0.5))  # 効き始めより浅い
    segs.append(_peak_rows(800.0, 30.0, -3.0, 9.0))  # 速すぎ（7.5 超）
    _write(p, segs)
    res = release_gain.measure_stop_release_gain([p], **_kw())
    assert res.n_peaks == 2
    assert res.gain_kmhs_pct is None  # 3 未満
    assert "山が 2" in res.reason


def test_stopped_after_lag_not_counted(tmp_path: Path) -> None:
    p = tmp_path / "a.csv"
    # 減速が強く、戻す途中で止まる（車速 0 以下に張り付く）
    _write(p, [_peak_rows(100.0 * i, 0.6, -6.0, 9.0) for i in range(4)])
    # 車速は負になる合成。止まった（<= 0.05）山は数えない
    res = release_gain.measure_stop_release_gain([p], **_kw())
    assert res.n_peaks == 0


def test_not_continuously_falling_not_counted(tmp_path: Path) -> None:
    p = tmp_path / "a.csv"
    # 戻している途中で開度が一度上がる（下がり続けていない）
    seg = [(t, v, b + (0.2 if k >= 22 else 0.0)) for k, (t, v, b) in
           enumerate(_peak_rows(0.0, 5.0, -3.0, 9.0))]
    _write(p, [seg])
    res = release_gain.measure_stop_release_gain([p], **_kw())
    assert res.n_peaks == 0


def test_other_sections_ignored(tmp_path: Path) -> None:
    p = tmp_path / "a.csv"
    _write(p, _decel_segments(4), section="PEDAL_SEARCH")
    res = release_gain.measure_stop_release_gain([p], **_kw())
    assert res.n_peaks == 0 and res.gain_kmhs_pct is None


def test_attach_to_meta_and_not_measured_has_no_keys(tmp_path: Path) -> None:
    p = tmp_path / "a.csv"
    _write(p, _decel_segments(4))
    res = release_gain.measure_stop_release_gain([p], **_kw())
    meta: dict = {}
    assert release_gain.attach_to_meta(meta, res)
    assert meta["stop_release_gain_n"] == 4
    assert len(meta["stop_release_gain_list"]) == 4
    assert meta["stop_release_gain_csv"] == ["a.csv"]
    assert meta["stop_release_gain_amount_pct"] == 0.3
    bad = release_gain.measure_stop_release_gain([p], amount_pct=0.3, accel_max_kmhs=-99.0, **_kw())
    meta2: dict = {}
    assert not release_gain.attach_to_meta(meta2, bad)
    assert meta2 == {}


def test_fwd_tool_attach_release_gain_writes_new_pkl(tmp_path: Path) -> None:
    src = tmp_path / "in.pkl"
    save_table(_table(), src)
    csvp = tmp_path / "d.csv"
    _write(csvp, _decel_segments(4))
    out = tmp_path / "out.pkl"
    rc = fwd_tool.main(
        ["attach-release-gain", "--pkl", str(src), "--csv", str(csvp), "--out", str(out)]
    )
    assert rc == 0
    meta = load_forward_map(out).meta
    assert 3.0 < meta["stop_release_gain_kmhs_pct"] < 4.5
    assert "stop_release_gain_kmhs_pct" not in load_forward_map(src).meta
    # 上書き拒否・測れないとき
    assert fwd_tool.main(["attach-release-gain", "--pkl", str(src), "--csv", str(csvp),
                          "--out", str(out)]) == 2
    nod = tmp_path / "nod.csv"
    _write(nod, _decel_segments(1))
    o2 = tmp_path / "o2.pkl"
    assert fwd_tool.main(["attach-release-gain", "--pkl", str(src), "--csv", str(nod),
                          "--out", str(o2)]) == 1
    assert not o2.exists()
