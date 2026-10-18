"""fel_pattern（FEL の学習用パターン）のテスト。合成モードだけを使う（2026-10-19 段2s-2b）。"""

from __future__ import annotations

from datetime import datetime

import numpy as np
import pytest

from tests.research import fel_pattern as fp
from tests.research.config import ResearchConfig
from tests.research.research_types import G_TO_KMHS, DrivingMode, SpeedPoint


def _mode(points: list[tuple[float, float]]) -> DrivingMode:
    return DrivingMode(
        id="m", name="m", description="", reference_speed=[SpeedPoint(t, v) for t, v in points],
        total_duration=points[-1][0], max_speed=max(v for _, v in points),
        created_at=datetime(2026, 1, 1),
    )


def _trips(tops: list[float], g: float = 0.08) -> DrivingMode:
    """停車 → 一定の G で上の車速まで → 同じ G で停車、を tops ぶん並べた合成モード（1 秒刻み）。"""
    pts: list[tuple[float, float]] = [(0.0, 0.0)]
    t = 3.0
    pts.append((t, 0.0))
    for top in tops:
        dur = top / (g * G_TO_KMHS)
        n = max(2, round(dur))
        for i in range(1, n + 1):
            pts.append((t + i, top * i / n))
        t += n
        pts.append((t + 2.0, top))  # 定速 2 秒
        t += 2.0
        for i in range(1, n + 1):
            pts.append((t + i, top * (1 - i / n)))
        t += n
        pts.append((t + 3.0, 0.0))
        t += 3.0
    return _mode(pts)


@pytest.fixture
def cfg() -> ResearchConfig:
    return ResearchConfig()


@pytest.fixture
def modes() -> list[DrivingMode]:
    return [_trips([20, 30, 40, 50, 60, 80, 100, 120, 30, 45])]


def _pattern(cfg, modes, coast=None):
    stats = fp.compute_stats(modes, cfg.fel, cfg.learning.ramp_g_levels)
    return fp.build_pattern(stats, cfg.fel, cfg.learning, top_cap_kmh=130.0,
                            coast_accel_kmhs=coast)


def _arrays(p: fp.FelPattern) -> tuple[np.ndarray, np.ndarray]:
    t = np.array([x.time_s for x in p.mode.reference_speed])
    v = np.array([x.speed_kmh for x in p.mode.reference_speed])
    return t, v


def test_stats_from_synthetic_trips(cfg, modes) -> None:
    s = fp.compute_stats(modes, cfg.fel, cfg.learning.ramp_g_levels)
    assert s.g_acc_med == pytest.approx(0.08, abs=0.02)
    assert s.g_dec_med == pytest.approx(0.08, abs=0.02)
    assert s.g_end == pytest.approx(0.08, abs=0.02)
    assert len(s.trip_tops_kmh) == 10 and max(s.trip_tops_kmh) == pytest.approx(120.0)


def test_stats_are_capped_by_g_levels(cfg) -> None:
    m = _trips([30, 40, 50], g=0.6)  # 0.6G は ramp_g_levels の最大を超える
    s = fp.compute_stats([m], cfg.fel, cfg.learning.ramp_g_levels)
    cap = max(cfg.learning.ramp_g_levels)
    assert s.g_acc_med <= cap and s.g_dec_hi <= cap and s.g_end <= cap


def test_stats_without_stops_raises(cfg) -> None:
    flat = _mode([(0.0, 50.0), (60.0, 50.0)])
    with pytest.raises(fp.PatternError):
        fp.compute_stats([flat], cfg.fel, cfg.learning.ramp_g_levels)


def test_pattern_is_deterministic_and_well_formed(cfg, modes) -> None:
    a, b = _pattern(cfg, modes), _pattern(cfg, modes)
    assert a.mode.reference_speed == b.mode.reference_speed
    t, v = _arrays(a)
    assert v[-1] == 0.0 and v.min() >= 0.0
    # 頭に停車 stop_hold_s
    assert np.all(v[t <= cfg.fel.stop_hold_s - 0.2] == 0.0)
    # 停車の回数（停車へ着く回数）＝ 低速 8 + 中速 1 + 高速 1
    assert a.stop_count == cfg.fel.low_trip_count + 2
    arrivals = np.flatnonzero((v[1:] == 0.0) & (v[:-1] > 0.0))
    assert len(arrivals) == a.stop_count
    # 最大 G は上限以下
    g = np.abs(np.diff(v) / (t[1] - t[0]) / G_TO_KMHS)
    assert g.max() <= max(cfg.learning.ramp_g_levels) + 1e-6
    assert v.max() <= 130.0 + 1e-6
    assert a.mode.total_duration == pytest.approx(t[-1])
    # 部分の秒数が連続している
    assert a.parts[0].start_s == 0.0
    for x, y in zip(a.parts, a.parts[1:], strict=False):
        assert x.end_s == pytest.approx(y.start_s)


def test_low_trip_tops_follow_quantiles_and_shapes_cycle(cfg, modes) -> None:
    p = _pattern(cfg, modes)
    low = [x for x in p.parts if x.name.startswith("① 低速")]
    assert len(low) == 8
    assert [x.note.split("形 ")[1] for x in low] == list("ABCDABCD")
    tops = [p.tops_kmh[x.name] for x in low]
    assert tops == sorted(tops)
    qs = np.quantile([x for x in p.stats.trip_tops_kmh if x <= cfg.fel.low_trip_max_kmh],
                     [(k + 0.5) / 8 for k in range(8)])
    assert tops == pytest.approx(list(qs))
    assert p.tops_kmh["③ 高速"] <= 130.0
    assert "② 中速" in p.tops_kmh


def test_total_is_about_eleven_minutes_for_wltp_like_stats(cfg) -> None:
    # 実際の WLTP に近い G（0.06〜0.1）で、約 11 分前後になる（規則で決まる値。合わせ込まない）
    m = _trips([22, 25, 30, 36, 42, 46, 49, 54, 65, 80, 113, 131], g=0.07)
    p = _pattern(cfg, [m])
    assert 8.0 * 60 < p.total_s < 14.0 * 60


def _builder(cfg, coast=None) -> fp._Builder:
    return fp._Builder(cfg.fel, cfg.learning, coast)


def _g_series(b: fp._Builder) -> tuple[np.ndarray, np.ndarray]:
    v = np.array(b.vs)
    return v, np.diff(v) / b.dt / G_TO_KMHS


def test_shape_a_decelerates_at_constant_g_then_eases_to_edge(cfg) -> None:
    b = _builder(cfg)
    b.launch_to(60.0, 0.1)
    b.hold(2.0)
    n0 = b.n
    b.descend_stop(0.1, "A", 0.05)
    v, g = _g_series(b)
    seg = g[n0:]
    mid = seg[int(3 / b.dt):int(10 / b.dt)]
    assert np.allclose(mid, -0.1, atol=0.01)  # 途中は -G
    assert abs(seg[-3:]).max() <= 0.1 and v[-1] == 0.0


def test_shape_b_eases_g_proportionally_below_ease_start(cfg) -> None:
    b = _builder(cfg)
    b.launch_to(60.0, 0.1)
    b.hold(2.0)
    n0 = b.n
    b.descend_stop(0.1, "B", 0.03)
    v, g = _g_series(b)
    v, g = v[n0:-1], g[n0:]
    ease = cfg.fel.ease_start_kmh
    hi = g[(v[:len(g)] > ease + 1) & (v[:len(g)] < 40)]
    assert np.allclose(hi, -0.1, atol=0.01)
    for vq in (12.0, 8.0, 4.0):
        i = np.argmin(np.abs(v[:len(g)] - vq))
        expect = 0.03 + (0.1 - 0.03) * vq / ease
        assert -g[i] == pytest.approx(expect, abs=0.015)
    assert b.v == 0.0


def test_shape_c_halves_g_in_the_middle_for_half_g_s(cfg) -> None:
    b = _builder(cfg)
    b.launch_to(60.0, 0.1)
    b.hold(2.0)
    n0 = b.n
    fp._stop_with_shape(b, 60.0, 0.1, "C", 0.05)
    v, g = _g_series(b)
    seg = g[n0:]
    half = np.abs(seg + 0.05) < 0.003
    assert half.sum() * b.dt == pytest.approx(cfg.fel.half_g_s, abs=0.6)
    # 半分にしている間の車速は上の車速の真ん中より少し下（切り替えと保持の間に下がる）
    idx = np.flatnonzero(half)
    i = n0 + int(idx[len(idx) // 2])
    assert 18.0 < v[i] < 30.0
    assert b.v == 0.0


def test_shape_d_coasts_with_table_then_brakes(cfg) -> None:
    b = _builder(cfg, coast=lambda v: -0.5)  # 惰行で −0.5 km/h/s の表
    b.launch_to(60.0, 0.1)
    b.hold(2.0)
    n0, v0 = b.n, b.v
    fp._stop_with_shape(b, 60.0, 0.1, "D", 0.05)
    v, g = _g_series(b)
    k = round((cfg.fel.coast_s - 0.5) / b.dt)
    # 惰行の間は表の減速（−0.5/35.3G）
    assert g[n0 + k] == pytest.approx(-0.5 / G_TO_KMHS, abs=1e-3)
    assert v[n0 + int(cfg.fel.coast_s / b.dt)] == pytest.approx(
        v0 - 0.5 * cfg.fel.coast_s, abs=1.0
    )
    assert b.v == 0.0
    # 表なしは 0G の定速
    b2 = _builder(cfg)
    b2.launch_to(60.0, 0.1)
    b2.hold(2.0)
    n1, v1 = b2.n, b2.v
    fp._stop_with_shape(b2, 60.0, 0.1, "D", 0.05)
    assert b2.vs[n1 + int(cfg.fel.coast_s / b2.dt)] == pytest.approx(v1, abs=0.2)


def test_pattern_uses_coast_table_note(cfg, modes) -> None:
    assert "0G" in _pattern(cfg, modes).coast_note
    assert "表の惰行" in _pattern(cfg, modes, coast=lambda v: -0.3).coast_note


def test_launch_rises_from_edge_g_at_launch_rate(cfg) -> None:
    b = _builder(cfg)
    b.launch_to(80.0, 0.25)
    _, g = _g_series(b)
    assert g[0] == pytest.approx(cfg.learning.ramp_edge_g, abs=0.02)
    # 立ち上がりの時間 ＝ (G − edge) ÷ rate
    blend = (0.25 - cfg.learning.ramp_edge_g) / cfg.learning.ramp_launch_g_rate_gps
    i = int(blend / b.dt) + 1
    assert g[i] == pytest.approx(0.25, abs=0.01)
    assert g[: int(blend / b.dt) - 1].max() < 0.25


def test_format_pattern_lists_parts_total_and_stops(cfg, modes) -> None:
    text = fp.format_pattern(_pattern(cfg, modes))
    assert "合計" in text and "停車 10 回" in text and "③ 高速" in text and "② 中速" in text


def test_config_validation_of_pattern_keys(cfg) -> None:
    from tests.research.config import validate_config  # noqa: PLC0415

    assert validate_config(cfg) == [] or all("fel." not in p for p in validate_config(cfg))
    cfg.fel.cruise_s = 0.0
    cfg.fel.high_speed_quantile = 1.5
    msgs = validate_config(cfg)
    assert any("fel.cruise_s" in m for m in msgs)
    assert any("fel.high_speed_quantile" in m for m in msgs)
