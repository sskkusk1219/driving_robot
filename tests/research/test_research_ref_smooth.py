"""段3（基準のぼかし）と段3b（合成モード）のテスト。合成の基準・表だけを使う。"""

from __future__ import annotations

import asyncio
from datetime import datetime
from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import mode_drive as mdmod
from tests.research.fwd_plan import FLAG_STOP, build_plan
from tests.research.research_types import DrivingMode, SpeedPoint
from tests.research.test_research_fwd_plan import AD, BD, STOP_PCT, linear_table

PERIOD = 0.05


def _mode(points: list[tuple[float, float]], name: str = "m") -> DrivingMode:
    return DrivingMode(
        id="t", name=name, description="", total_duration=points[-1][0],
        reference_speed=[SpeedPoint(time_s=t, speed_kmh=v) for t, v in points],
        max_speed=max(v for _, v in points), created_at=datetime(2026, 10, 19),
    )


# 停車 0〜5s → 加速 10s で 40 km/h → 5s 保持 → 減速 10s で停車 → 停車 5s
TRAPEZOID = [(0, 0), (5, 0), (15, 40), (20, 40), (30, 0), (35, 0)]


def _plan(ref_at, orig_at=None):
    return build_plan(
        ref_at, 35.0, linear_table(), accel_deadband_pct=AD, brake_deadband_pct=BD,
        stop_brake_opening_pct=STOP_PCT, stop_horizon_s=0.5, period_s=PERIOD,
        return_gap_enabled=False, stop_ref_at=orig_at,
    )


def test_sigma_zero_is_the_original_reference_and_the_same_plan() -> None:
    mode = _mode(TRAPEZOID)
    orig = mdmod.ReferenceSpeed(mode)
    target = mdmod.make_target_reference(mode, 0.0)
    assert target.at.__self__.__class__ is mdmod.ReferenceSpeed
    a, b = _plan(orig.at), _plan(target.at, orig.at)
    assert np.array_equal(a.effort_pct, b.effort_pct)
    assert np.array_equal(a.flag, b.flag)
    assert np.array_equal(a.stops_ahead, b.stops_ahead)


def test_smoothing_rounds_corners_without_delay_and_keeps_ramps() -> None:
    mode = _mode(TRAPEZOID)
    orig = mdmod.ReferenceSpeed(mode)
    sm = mdmod.SmoothedReference(mode, 0.5)
    # 傾きの途中は変わらない（線形はガウスを通しても線形）
    assert sm.at(10.0) == pytest.approx(orig.at(10.0), abs=1e-3)
    assert sm.at(25.0) == pytest.approx(orig.at(25.0), abs=1e-3)
    # 折れ目（40 km/h の肩）は丸まって低くなる。左右対称なので遅れは足さない
    assert sm.at(15.0) < orig.at(15.0) and sm.at(20.0) < orig.at(20.0)
    assert sm.at(15.0) == pytest.approx(orig.at(15.0) - 4.0 * 0.5 * 0.3989, abs=0.05)
    # 折れ目の前後の傾きの変化がなだらかになる（最大の 0.1s 傾きの差）
    ts = np.arange(10.0, 25.0, 0.1)
    d_orig = np.abs(np.diff([orig.at(t) for t in ts], 2)).max()
    d_sm = np.abs(np.diff([sm.at(t) for t in ts], 2)).max()
    assert d_sm < d_orig * 0.5
    # 範囲外は端点値
    assert sm.at(-3.0) == 0.0 and sm.at(99.0) == 0.0


def test_stop_intervals_return_to_the_original_zero() -> None:
    mode = _mode(TRAPEZOID)
    sm = mdmod.SmoothedReference(mode, 0.5)
    for t in (0.0, 2.5, 5.0, 30.0, 32.0, 35.0):
        assert sm.at(t) == 0.0
    # 停車の手前・発進の直後は 0 より大きい（元の基準より少し高い）
    assert 0.0 < sm.at(5.2) and 0.0 < sm.at(29.8)


def test_plan_stop_flags_use_the_original_reference() -> None:
    mode = _mode(TRAPEZOID)
    orig = mdmod.ReferenceSpeed(mode)
    sm = mdmod.SmoothedReference(mode, 0.5)
    base = _plan(orig.at)
    blurred = _plan(sm.at, orig.at)
    # 停車の判定（停車保持の行）は元の基準のときと完全に同じ
    assert np.array_equal(base.flag == FLAG_STOP, blurred.flag == FLAG_STOP)
    assert np.array_equal(base.stops_ahead, blurred.stops_ahead)
    # 停車の中の指令は同じ停車保持
    stop = base.flag == FLAG_STOP
    assert np.all(blurred.effort_pct[stop] == -STOP_PCT)
    # 走っている所の指令の折れ目は、ぼかすとなだらかになる
    run = ~stop
    jump_orig = np.abs(np.diff(base.effort_pct))[run[1:] & run[:-1]].max()
    jump_sm = np.abs(np.diff(blurred.effort_pct))[run[1:] & run[:-1]].max()
    assert jump_sm < jump_orig


def test_max_offset_caps_the_gap_and_keeps_small_offsets() -> None:
    mode = _mode(TRAPEZOID)
    orig = mdmod.ReferenceSpeed(mode)
    plain = mdmod.SmoothedReference(mode, 0.5)
    capped = mdmod.make_target_reference(mode, 0.5, 0.1)
    ts = np.arange(0.0, 35.0, 0.01)
    gap = np.array([capped.at(t) - orig.at(t) for t in ts])
    assert np.abs(gap).max() <= 0.1 + 1e-9
    assert np.abs(gap).max() > 0.09  # 肩では実際に上限で切られている
    for t in ts:
        d = plain.at(t) - orig.at(t)
        if abs(d) < 0.1 and orig.at(t) > 1e-6:
            assert capped.at(t) == pytest.approx(plain.at(t), abs=1e-9)
    # 0 は上限なし（今と同じ）
    zero = mdmod.make_target_reference(mode, 0.5, 0.0)
    assert all(zero.at(t) == plain.at(t) for t in ts[::50])
    assert "±0.1" in mdmod.ref_smooth_text(0.5, 0.1)
    assert "±" not in mdmod.ref_smooth_text(0.5)


def test_brake_weight_keeps_original_where_decel_is_steep() -> None:
    # 緩い減速（1 km/h/s）の後に急な減速（-3 km/h/s）の角がある
    mode = _mode([(0, 0), (5, 0), (15, 30), (20, 30), (30, 20), (35, 20),
                  (45, 20), (55, 0), (60, 0)])
    orig = mdmod.ReferenceSpeed(mode)
    capped = mdmod.make_target_reference(mode, 0.5, 0.1)
    both = mdmod.make_target_reference(mode, 0.5, 0.1, 0.3, 0.6)
    ts = np.arange(5.5, 59.0, 0.01)
    # 同じ σ でぼかした元の基準の加速度（テスト側で独立に計算）
    grid = np.arange(0.0, 60.0, 0.01)
    base = np.array([orig.at(t) for t in grid])
    x = np.arange(-200, 201) * 0.01
    k = np.exp(-0.5 * (x / 0.5) ** 2)
    k /= k.sum()
    acc = np.convolve(np.pad(np.gradient(base, 0.01), 200, mode="edge"), k, mode="valid")
    n_steep = n_flat = n_mid = 0
    for t in ts:
        a = acc[int(round(t / 0.01))]
        if orig.at(t) <= 1e-6:
            continue
        if a <= -0.6 - 0.02:
            n_steep += 1
            assert both.at(t) == pytest.approx(orig.at(t), abs=1e-6)
        elif a >= -0.3 + 0.02:
            n_flat += 1
            assert both.at(t) == pytest.approx(capped.at(t), abs=1e-6)
        else:
            n_mid += 1
            lo, hi = sorted((orig.at(t), capped.at(t)))
            assert lo - 1e-9 <= both.at(t) <= hi + 1e-9
    assert n_steep > 100 and n_flat > 100 and n_mid > 0
    # from 0 は今まで通り（ビット同一）
    off = mdmod.make_target_reference(mode, 0.5, 0.1, 0.0, 0.0)
    assert all(off.at(t) == capped.at(t) for t in ts[::37])
    text = mdmod.ref_smooth_text(0.5, 0.1, 0.3, 0.6)
    assert "減速 0.3〜0.6 km/h/s より強い所はぼかさない" in text
    assert "減速" not in mdmod.ref_smooth_text(0.5, 0.1)


def test_brake_side_upper_stops_only_the_positive_offset() -> None:
    mode = _mode([(0, 0), (5, 0), (15, 30), (20, 30), (30, 20), (35, 20),
                  (45, 20), (55, 0), (60, 0)])
    orig = mdmod.ReferenceSpeed(mode)
    capped = mdmod.make_target_reference(mode, 0.5, 0.1)
    both = mdmod.make_target_reference(mode, 0.5, 0.1, 0.3, 0.6)
    both2 = mdmod.make_target_reference(mode, 0.5, 0.1, 0.3, 0.6, "both")
    upper = mdmod.make_target_reference(mode, 0.5, 0.1, 0.3, 0.6, "upper")
    ts = np.arange(5.5, 59.0, 0.01)
    assert all(both.at(t) == both2.at(t) for t in ts[::13])
    grid = np.arange(0.0, 60.0, 0.01)
    base = np.array([orig.at(t) for t in grid])
    x = np.arange(-200, 201) * 0.01
    k = np.exp(-0.5 * (x / 0.5) ** 2)
    k /= k.sum()
    acc = np.convolve(np.pad(np.gradient(base, 0.01), 200, mode="edge"), k, mode="valid")
    n_pos = n_neg = 0
    for t in ts:
        if orig.at(t) <= 1e-6:
            continue
        a = acc[int(round(t / 0.01))]
        off = capped.at(t) - orig.at(t)
        if a <= -0.6 - 0.02:
            if off > 1e-9:
                n_pos += 1
                assert upper.at(t) == pytest.approx(orig.at(t), abs=1e-6)
            elif off < -1e-9:
                n_neg += 1
                assert upper.at(t) == pytest.approx(capped.at(t), abs=1e-6)
        elif a >= -0.3 + 0.02:
            assert upper.at(t) == pytest.approx(capped.at(t), abs=1e-6)
        else:
            if off < -1e-9:
                assert upper.at(t) == pytest.approx(capped.at(t), abs=1e-6)
    assert n_pos > 20 and n_neg > 20
    text = mdmod.ref_smooth_text(0.5, 0.1, 0.3, 0.6, "upper")
    assert "より強い所は基準より上に出る分だけぼかさない" in text
    assert "基準より上に出る分だけ" not in mdmod.ref_smooth_text(0.5, 0.1, 0.3, 0.6)


def test_brake_side_is_validated(tmp_path: Path) -> None:
    cfg = _cfg(tmp_path)
    assert cfg.feedforward.ref_smooth_brake_side == "both"
    cfg.feedforward.ref_smooth_brake_side = "lower"
    assert any("ref_smooth_brake_side" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.ref_smooth_brake_side = "upper"
    assert not any("ref_smooth_brake_side" in p for p in cfgmod.validate_config(cfg))


def test_csv_has_ref_smooth_column_at_the_end() -> None:
    # 後から列が末尾に足されるので、並び（pred_dev_kmh の直後）だけを見る（2026-11-11）
    cols = dlmod.CSV_COLUMNS
    assert cols.index("ref_smooth_kmh") == cols.index("pred_dev_kmh") + 1


# ── 段3b: 合成モード ─────────────────────────────────────────────

# 停車 0〜4・走行・停車 8〜12・走行・停車 20〜24
SOURCE = [(0, 0), (4, 0), (6, 20), (8, 0), (12, 0), (14, 30), (18, 30), (20, 0), (24, 0)]


def test_compose_mode_joins_windows_at_stops() -> None:
    src = _mode(SOURCE, "src")
    orig = mdmod.ReferenceSpeed(src)
    out = mdmod.compose_mode(src, [[0, 10], [10, 22]], "short")
    assert out.name == "short" and out.total_duration == 22.0
    ts = [p.time_s for p in out.reference_speed]
    assert ts == sorted(set(ts))  # 単調増加・重なりなし
    ref = mdmod.ReferenceSpeed(out)
    for t in np.arange(0.0, 22.0, 0.25):
        assert ref.at(float(t)) == pytest.approx(orig.at(float(t)))
    # 飛び飛びの窓: 時刻は 0 からつながり、長さは幅の合計。後ろの窓は前の窓の終わりに続く
    out2 = mdmod.compose_mode(src, [[0, 10], [22, 24]], "short2")
    assert out2.total_duration == 12.0
    assert out2.reference_speed[0].time_s == 0.0
    assert out2.reference_speed[-1].speed_kmh == 0.0
    assert out2.max_speed == 20.0
    ref2 = mdmod.ReferenceSpeed(out2)
    assert ref2.at(5.0) == pytest.approx(orig.at(5.0))
    assert ref2.at(11.0) == 0.0
    # 停車を挟んでつなぐ（走行の途中で切らない）: [2,10] の次が [10,..] ならそのまま続く
    out3 = mdmod.compose_mode(src, [[2, 10], [10, 22]], "short3")
    assert out3.total_duration == 20.0
    assert mdmod.ReferenceSpeed(out3).at(3.0) == pytest.approx(orig.at(5.0))


def test_compose_mode_rejects_edges_not_at_stop_and_bad_windows() -> None:
    src = _mode(SOURCE, "src")
    with pytest.raises(cfgmod.ConfigError, match="停車の途中"):
        mdmod.compose_mode(src, [[0, 7]], "x")
    with pytest.raises(cfgmod.ConfigError, match="不正"):
        mdmod.compose_mode(src, [[0, 30]], "x")
    with pytest.raises(cfgmod.ConfigError, match="不正"):
        mdmod.compose_mode(src, [[10, 24], [0, 5]], "x")
    with pytest.raises(cfgmod.ConfigError, match="windows_s"):
        mdmod.compose_mode(src, [], "x")


def _cfg(tmp_path: Path, composed: list[dict] | None = None) -> cfgmod.ResearchConfig:
    cfg = cfgmod.load_config(tmp_path / "cfg.yaml")
    if composed is not None:
        cfg.modes.composed_modes = composed
    return cfg


def test_load_mode_sees_composed_modes_before_the_db(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    calls: list[str] = []

    async def fake_db(_cfg: object, name: str) -> DrivingMode:
        calls.append(name)
        if name != "src":
            raise cfgmod.ConfigError(f"走行モード {name!r} が driving_modes にありません")
        return _mode(SOURCE, "src")

    monkeypatch.setattr(mdmod, "_load_mode_from_db", fake_db)
    cfg = _cfg(tmp_path, [{"name": "short", "source": "src", "windows_s": [[0, 10], [22, 24]]}])
    mode = asyncio.run(mdmod.load_mode(cfg, "short"))
    assert mode.name == "short" and mode.total_duration == 12.0 and calls == ["src"]
    # DB にもある普通のモード名は今まで通り DB から
    assert asyncio.run(mdmod.load_mode(cfg, "src")).name == "src"
    # DB にも合成モードにも無い名前は今まで通りのエラー
    with pytest.raises(cfgmod.ConfigError, match="driving_modes にありません"):
        asyncio.run(mdmod.load_mode(cfg, "nothing"))


def test_composed_modes_and_sigma_are_validated(tmp_path: Path) -> None:
    cfg = _cfg(tmp_path)
    assert cfg.feedforward.ref_smooth_sigma_s == 0.15  # 2026-11-11 PDCA-4 で本体 on
    assert not any("composed_modes" in p or "ref_smooth" in p for p in cfgmod.validate_config(cfg))
    good = {"name": "a", "source": "b", "windows_s": [[0, 5], [8, 9]]}
    for bad in (
        {"name": "", "source": "b", "windows_s": [[0, 5]]},
        {"name": "a", "source": "a", "windows_s": [[0, 5]]},
        {"name": "a", "source": "b", "windows_s": []},
        {"name": "a", "source": "b", "windows_s": [[5, 0]]},
        {"name": "a", "source": "b", "windows_s": [[0, 5], [4, 9]]},
        {"name": "a", "source": "b", "windows_s": [[0, 5, 6]]},
        {"name": "a", "source": "b", "windows_s": [[0, 5]], "extra": 1},
    ):
        cfg.modes.composed_modes = [bad]
        assert any("composed_modes" in p for p in cfgmod.validate_config(cfg)), bad
    cfg.modes.composed_modes = [good]
    assert not any("composed_modes" in p for p in cfgmod.validate_config(cfg))
    assert cfg.feedforward.ref_smooth_max_kmh == 0.1
    for bad_max in (-1.0, 5.5):
        cfg.feedforward.ref_smooth_max_kmh = bad_max
        assert any("ref_smooth_max_kmh" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.ref_smooth_max_kmh = 0.1
    assert not any("ref_smooth_max_kmh" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.ref_smooth_max_kmh = 0.0
    ff = cfg.feedforward
    assert ff.ref_smooth_brake_from_kmhs == 0.3 and ff.ref_smooth_brake_to_kmhs == 0.6  # 本体 yaml
    for fr, to in ((0.6, 0.3), (0.3, 0.3), (-0.1, 0.5), (0.3, 11.0), (0.0, 0.5)):
        ff.ref_smooth_brake_from_kmhs, ff.ref_smooth_brake_to_kmhs = fr, to
        assert any("ref_smooth_brake" in p for p in cfgmod.validate_config(cfg)), (fr, to)
    ff.ref_smooth_brake_from_kmhs, ff.ref_smooth_brake_to_kmhs = 0.3, 0.6
    assert not any("ref_smooth_brake" in p for p in cfgmod.validate_config(cfg))
    ff.ref_smooth_brake_from_kmhs = ff.ref_smooth_brake_to_kmhs = 0.0
    for sigma in (-0.1, 2.5):
        cfg.feedforward.ref_smooth_sigma_s = sigma
        assert any("ref_smooth_sigma_s" in p for p in cfgmod.validate_config(cfg))


def test_default_yaml_has_ama_short() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    item = next(c for c in cfg.modes.composed_modes if c["name"] == "12_AMA_short")
    assert item["source"] == "12_AMA" and len(item["windows_s"]) == 5
