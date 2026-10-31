"""手順13: 4 点の保持の開度（2026-10-30 段6。pedal_search の純関数と main の手順の並び）。"""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import main as mainmod
from tests.research import pedal_search as ps
from tests.research.hold_scene import hold_opening_points, interp_hold_opening


def test_hold_point_speeds_uses_fractions_extra_and_drops_outside() -> None:
    v = ps.hold_point_speeds(5.0, [0.25, 0.5, 0.75], [1.6])
    assert v == pytest.approx([1.25, 1.6, 2.5, 3.75])
    # クリープ以上・停車速度以下は除く、重複は 1 つ
    assert ps.hold_point_speeds(5.0, [0.5], [5.0, 6.0, 0.01, 2.5]) == pytest.approx([2.5])
    assert ps.hold_point_speeds(0.05, [0.25, 0.5], [1.6]) == pytest.approx([0.025])  # 0.0125 は停車
    assert ps.hold_point_speeds(5.0, [], []) == []


def test_hold_initial_opening_interpolates_by_speed_ascending() -> None:
    # 開度が深いほど車速が下がる（min ＝ クリープ 5.0）
    pairs = [(3.0, 5.0), (4.0, 4.0), (5.0, 2.0), (6.0, 0.0)]
    assert ps.hold_initial_opening(pairs, 3.0, 99.0) == pytest.approx(4.5)
    assert ps.hold_initial_opening(pairs, 1.0, 99.0) == pytest.approx(5.5)
    # 端の外は端の値
    assert ps.hold_initial_opening(pairs, 9.0, 99.0) == pytest.approx(3.0)
    # 並びが車速の昇順でなくても同じ
    assert ps.hold_initial_opening(pairs[::-1], 3.0, 99.0) == pytest.approx(4.5)
    # 組が足りなければ動き出す開度
    assert ps.hold_initial_opening([(3.0, 5.0)], 3.0, 99.0) == 99.0
    assert ps.hold_initial_opening([], 3.0, 7.5) == 7.5


def test_hold_band_ok_uses_g_band() -> None:
    band = 0.005  # G → 0.005 × 9.81 × 3.6 ≒ 0.1766 km/h/s
    assert ps.hold_band_ok(0.17, band)
    assert ps.hold_band_ok(-0.17, band)
    assert not ps.hold_band_ok(0.2, band)
    assert not ps.hold_band_ok(None, band)


def test_hold_verdict_accumulates_and_times_out() -> None:
    # 帯の合計が 5 s に達したら保てた（途切れても合計は戻さない）
    in_band = 0.0
    result = None
    for k in range(200):  # 0.05 s 刻み。2 s 入り・1 s 外れ・また入る
        t = k * 0.05
        if t < 2.0 or t >= 3.0:
            in_band += 0.05
        result = ps.hold_verdict(in_band, t, 5.0, 17.0)
        if result is not None:
            break
    assert result is True
    assert 5.9 < t < 6.2  # 2 s ＋ 3 s で合計 5 s（間の 1 s の外れは数えない）
    # 打ち切り: 17 s 経って合計が足りなければ保てず
    assert ps.hold_verdict(4.9, 16.9, 5.0, 17.0) is None
    assert ps.hold_verdict(4.9, 17.0, 5.0, 17.0) is False
    # 同じ周期で両方満たすなら保てた
    assert ps.hold_verdict(5.0, 17.0, 5.0, 17.0) is True


def test_hold_opening_meta_is_readable_by_hold_scene() -> None:
    pts = [
        ps.HoldPointResult(1.25, True, 1.24, 9.0, 5.1, 7.0, 9.2),
        ps.HoldPointResult(1.6, False, None, None, 0.0, 17.0, 8.8, note="x"),
        ps.HoldPointResult(2.5, True, 2.49, 8.0, 5.0, 8.0, 8.3),
    ]
    meta = ps.hold_opening_meta(
        pts, creep_kmh=5.0, breakaway_pct=10.0, min_pct=3.0, source_csv="a.csv"
    )
    points = hold_opening_points({"hold_opening": meta})
    assert points == [(0.0, 10.0), (1.24, 9.0), (2.49, 8.0), (5.0, 3.0)]  # 保てなかった点は除く
    assert interp_hold_opening(points, 1.865) == pytest.approx(8.5)
    assert meta["source_csv"] == "a.csv"


def test_config_validates_hold_points_lists() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    assert cfg.pedal_search.hold_points_enabled is True  # 2026-10-30 手順2 で自動
    assert cfg.pedal_search.hold_points_fractions == [0.25, 0.5, 0.75]
    assert cfg.pedal_search.hold_points_extra_kmh == [1.6]
    assert not [p for p in cfgmod.validate_config(cfg) if "hold_points" in p]
    for fractions in ([0.0], [1.0], [1.5], [-0.1]):
        cfg.pedal_search.hold_points_fractions = fractions
        assert any("hold_points_fractions" in p for p in cfgmod.validate_config(cfg))
    cfg.pedal_search.hold_points_fractions = [0.5]
    cfg.pedal_search.hold_points_extra_kmh = [0.0]
    assert any("hold_points_extra_kmh" in p for p in cfgmod.validate_config(cfg))


def test_step13_exists_and_pkl_name() -> None:
    step = next(s for s in mainmod.STEPS if s.number == 13)
    assert step.drives and step.prepare is not None
    assert mainmod.MAX_STEP == 14
    assert mainmod.hold_pkl_path("a/b/fwd_model_x_hpi.pkl") == Path("a/b/fwd_model_x_hpi_hold.pkl")
