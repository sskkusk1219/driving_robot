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


def test_csv_has_ref_smooth_column_at_the_end() -> None:
    assert dlmod.CSV_COLUMNS[-1] == "ref_smooth_kmh"
    assert dlmod.CSV_COLUMNS.index("pred_dev_kmh") == len(dlmod.CSV_COLUMNS) - 2


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
    assert cfg.feedforward.ref_smooth_sigma_s == 0.0
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
    for sigma in (-0.1, 2.5):
        cfg.feedforward.ref_smooth_sigma_s = sigma
        assert any("ref_smooth_sigma_s" in p for p in cfgmod.validate_config(cfg))


def test_default_yaml_has_ama_short() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    item = next(c for c in cfg.modes.composed_modes if c["name"] == "12_AMA_short")
    assert item["source"] == "12_AMA" and len(item["windows_s"]) == 5
