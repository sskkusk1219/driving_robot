"""C14（停車の最後のブレーキの下限。creep_floor.py）のテスト（2026-10-19 段3a）。"""

from __future__ import annotations

import pytest

from tests.research import config as cfgmod
from tests.research.creep_floor import CreepFloor, StopArrivals, creep_floor_opening_pct
from tests.research.ff_candidate import CoastAssistFeedforward, make_candidate

# 基準: 0 → 10 km/h に上がり、10 → 3 → 0 と停車へ向かい、30 s から再発進
_TIMES = [0.0, 5.0, 10.0, 18.0, 20.0, 30.0, 35.0]
_SPEEDS = [0.0, 10.0, 10.0, 3.0, 0.0, 0.0, 10.0]


def _floor(floor_pct: float | None = 8.5) -> CreepFloor:
    return CreepFloor(
        floor_pct=floor_pct, lead_s=5.0, speed_limit_kmh=7.5,
        arrivals=StopArrivals(_TIMES, _SPEEDS), accel_min_pct=4.2,
    )


def test_floor_is_min_plus_abs_breakaway_capped_by_stop_hold() -> None:
    meta = {"launch_breakaway_u_pct": -4.33}
    assert creep_floor_opening_pct(meta, 4.2, 17.88) == pytest.approx(8.53)
    assert creep_floor_opening_pct(meta, 4.2, 6.0) == pytest.approx(6.0)  # 停車保持より深くしない
    assert creep_floor_opening_pct({}, 4.2, 17.88) is None  # メタが無ければ下限なし


def test_lifts_shallow_brake_only_when_all_conditions_hold() -> None:
    f = _floor()
    # 条件がそろう: 次の停車(20 s)まで 2 s・基準 3 km/h・実車速 2.5 km/h・アクセルは min
    assert f.apply(18.0, 3.0, 2.5, -7.9, 4.2) == pytest.approx(-8.5)
    assert f.stats.cycles == 1 and f.stats.raise_max_pct == pytest.approx(0.6)
    # 深い指令は変えない
    assert f.apply(18.0, 3.0, 2.5, -12.0, 4.2) == -12.0
    # 停車まで遠い (t=10: 10 s 先) / 基準が速い / 実車速が速い / 停車中（基準 0）
    assert f.apply(12.0, 8.0, 2.5, -7.9, 4.2) == -7.9
    assert f.apply(10.0, 10.0, 2.5, -7.9, 4.2) == -7.9
    assert f.apply(18.0, 3.0, 9.0, -7.9, 4.2) == -7.9
    assert f.apply(25.0, 0.0, 0.0, -7.9, 4.2) == -7.9


def test_not_applied_on_accel_side_or_when_accel_is_pressed() -> None:
    f = _floor()
    assert f.apply(18.0, 3.0, 2.5, 3.0, 4.2) == 3.0  # アクセル側の指令
    assert f.apply(18.0, 3.0, 2.5, 0.0, 4.2) == 0.0  # 惰行
    assert f.apply(18.0, 3.0, 2.5, -7.9, 9.0) == -7.9  # アクセルを踏んでいる
    assert f.stats.cycles == 0


def test_no_meta_behaves_like_c12() -> None:
    f = _floor(None)
    assert f.apply(18.0, 3.0, 2.5, -7.9, 4.2) == -7.9
    assert "下限なし" in f.settings_text()


def test_stats_count_entries_and_mean() -> None:
    f = _floor()
    for e in (-7.9, -8.1, -12.0, -8.0):
        f.apply(18.0, 3.0, 2.5, e, 4.2)
    assert f.stats.entries == 2 and f.stats.cycles == 3
    assert "掛けた 2 回" in f.stats.summary()


def test_record_false_does_not_count() -> None:
    f = _floor()
    f.apply(18.0, 3.0, 2.5, -7.9, 4.2, record=False)
    assert f.stats.cycles == 0


def test_c14_is_a_c12_and_config_knows_it() -> None:
    ff = make_candidate("C14")
    assert isinstance(ff, CoastAssistFeedforward) and ff.candidate == "C14"
    assert ff.expected_kind == "model" and ff.creep_floor_lead_s == 5.0
    ff.set_creep_floor(lead_s=3.0)
    assert ff.creep_floor_lead_s == 3.0
    assert "C14" in cfgmod.CANDIDATE_NAMES
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.feedforward.candidate = "C14"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.creep_floor_lead_s = 0.0
    assert any("creep_floor_lead_s" in p for p in cfgmod.validate_config(cfg))
