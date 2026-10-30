"""hold_check の純関数のテスト（合成の配列）。"""

from __future__ import annotations

import numpy as np

from tests.research import hold_check as hc

DT = 0.05
SLOPE = hc.slope_threshold_kmhs(0.005)


def _t(n: int) -> np.ndarray:
    return np.arange(n) * DT


def _seg(dev: np.ndarray, cmd: np.ndarray | None = None) -> hc.HoldSegment:
    n = len(dev)
    c = np.full(n, 5.0) if cmd is None else cmd
    return hc.HoldSegment(_t(n), dev, c, c)


def test_slope_threshold() -> None:
    assert abs(SLOPE - 0.005 * 9.80665 * 3.6) < 1e-12


def test_reach_found_after_settling() -> None:
    n = 400  # 20 s
    dev = np.where(_t(n) < 6.0, 1.0, 0.1)
    j = hc.judge_segment(_seg(dev), tol_kmh=0.3, hold_s=5.0)
    assert j.reach_t_s is not None
    # 6 s から帯の中 → 合計 5 s に達するのは 11 s
    assert abs(j.reach_t_s - 11.0) < 0.06
    assert abs(j.reach_after_s - 11.0) < 0.06
    assert j.abs_dev_max == 1.0
    assert 0.6 < j.within_tol_ratio < 0.75


def test_reach_not_found_when_short() -> None:
    n = 400  # 20 s のうち帯の中は 4 s だけ
    dev = np.where(_t(n) < 16.0, 1.0, 0.1)
    j = hc.judge_segment(_seg(dev), tol_kmh=0.3, hold_s=5.0)
    assert j.reach_t_s is None and j.reach_after_s is None


def test_reach_counts_total_not_continuous() -> None:
    # 3 s ごとに外れても、帯の中の時間の合計で数える（G ランプの定速の保持と同じ）
    n = 400
    dev = np.full(n, 0.1)
    dev[::60] = 1.0
    j = hc.judge_segment(_seg(dev), tol_kmh=0.3, hold_s=5.0)
    assert j.reach_t_s is not None and 5.0 < j.reach_t_s < 5.2


def test_reversals_counted_with_hysteresis() -> None:
    n = 200
    cmd = np.full(n, 5.0)
    cmd[40:80] = 5.3  # 上げて
    cmd[80:120] = 5.0  # 戻す → 1 回
    cmd[120:160] = 5.3  # 上げる → 2 回、160 以降で戻す → 3 回
    cmd[::2] += 0.05  # hyst 未満の細かい上下は数えない
    j = hc.judge_segment(_seg(np.zeros(n), cmd), tol_kmh=0.3, hold_s=5.0)
    assert j.reversals == 3
    assert abs(j.reversals_per_s - 3 / j.length_s) < 1e-9


def test_find_segments_excludes_slope_and_stop() -> None:
    n = 600
    t = _t(n)
    ref = np.full(n, 1.6)
    ref[:100] = 0.0  # 停車
    ref[200:300] = np.linspace(1.6, 4.0, 100)  # 傾きあり（約 0.48 km/h/s）
    ref[400:] = 8.0  # クリープ車速以上
    z = np.zeros(n)
    segs = hc.find_hold_segments(t, ref, z, z, z, slope_max_kmhs=SLOPE, creep_kmh=5.0)
    # 保持: 100〜200 付近と 300〜400 付近の 2 つ（端の勾配の行は除かれる）
    assert len(segs) == 2
    assert segs[0].t_s[0] >= 100 * DT and segs[0].t_s[-1] < 200 * DT
    assert segs[1].t_s[0] > 300 * DT and segs[1].t_s[-1] < 400 * DT
    for s in segs:
        assert np.all(s.t_s[1:] - s.t_s[:-1] < 0.5)


def test_find_segments_splits_on_time_gap() -> None:
    t = np.concatenate([_t(100), _t(100) + 10.0])
    ref = np.full(200, 1.6)
    z = np.zeros(200)
    segs = hc.find_hold_segments(t, ref, z, z, z, slope_max_kmhs=SLOPE, creep_kmh=5.0)
    assert len(segs) == 2
