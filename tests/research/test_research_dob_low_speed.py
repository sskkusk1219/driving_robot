"""段6-2: 低速の DOB（薄める・基準車速でそろえる）と停C の τ の上限のテスト。2026-10-23。"""

from __future__ import annotations

import types
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research.dob import PedalDisturbanceObserver
from tests.research.ff_model import STOP_SPEED_KMH
from tests.research.fwd_plan import FLAG_STOP
from tests.research.mode_drive import _ModeRun, dob_fade_weight
from tests.research.stop_end import cap_stop_end_tau

TOP = 7.5


def _fake(fade: bool, align: bool) -> types.SimpleNamespace:
    fm = types.SimpleNamespace(dob_low_speed_fade=fade, dob_low_speed_align=align)
    meta = {"stop_max_speed_kmh": TOP}
    obj = types.SimpleNamespace(
        cfg=types.SimpleNamespace(forward_map=fm),
        ff=types.SimpleNamespace(table=types.SimpleNamespace(meta=meta, speed_edges=[])),
    )
    for name in ("_dob_low_top", "_dob_boundary_speed", "_dob_low_speed", "_dob_weight",
                 "_dob_launch_top", "_dob_follow_plan_tables"):
        setattr(obj, name, types.MethodType(getattr(_ModeRun, name), obj))
    return obj


def test_defaults_are_off_and_yaml_matches() -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.dob_low_speed_fade is False
    assert fm.dob_low_speed_align is False
    assert fm.stop_end_tau_cap_normal is False
    cfg = cfgmod.load_config(Path("tests/research/config_testVehicle.yaml"))
    f = cfg.forward_map
    assert (f.dob_low_speed_fade, f.dob_low_speed_align, f.stop_end_tau_cap_normal) == (
        False, False, False)
    assert not any("dob_low_speed" in p or "stop_end_tau" in p for p in cfgmod.validate_config(cfg))


def test_default_weight_is_binary_like_before() -> None:
    o = _fake(False, False)
    assert o._dob_weight(7.4, 0, 20.0) == 0.0  # 実車速で判定（基準は見ない）
    assert o._dob_weight(7.5, 0, 0.0) == 1.0
    assert o._dob_weight(30.0, FLAG_STOP, 30.0) == 0.0
    assert o._dob_weight(STOP_SPEED_KMH, 0, 30.0) == 0.0


def test_fade_weight_is_one_above_top_zero_at_stop_and_linear() -> None:
    assert dob_fade_weight(7.5, 0.5, 7.5) == 1.0
    assert dob_fade_weight(20.0, 0.5, 7.5) == 1.0
    assert dob_fade_weight(0.5, 0.5, 7.5) == 0.0
    assert dob_fade_weight(0.0, 0.5, 7.5) == 0.0
    assert dob_fade_weight(4.0, 0.5, 7.5) == pytest.approx(0.5)
    o = _fake(True, False)
    assert o._dob_weight(4.0, 0, 99.0) == pytest.approx(0.5)  # align なし: 実車速
    assert o._dob_weight(STOP_SPEED_KMH, 0, 30.0) == 0.0
    assert o._dob_weight(0.5, 0, 30.0) == 0.0
    assert o._dob_weight(5.0, FLAG_STOP, 5.0) == 0.0


def test_align_uses_reference_speed_for_boundary() -> None:
    o = _fake(False, True)
    assert o._dob_weight(30.0, 0, 3.0) == 0.0  # 基準が境目未満 → 止める
    assert o._dob_weight(3.0, 0, 30.0) == 1.0
    assert o._dob_weight(STOP_SPEED_KMH, 0, 30.0) == 0.0  # 停車速度以下は実車速で止める
    f = _fake(True, True)
    assert f._dob_weight(30.0, 0, 4.0) == pytest.approx(0.5)
    assert f._dob_follow_plan_tables() is True
    assert _fake(True, False)._dob_follow_plan_tables() is False


def test_stop_end_tau_cap() -> None:
    assert cap_stop_end_tau(3.0, 2.0, False) == 3.0  # 既定: 今と同じ
    assert cap_stop_end_tau(3.0, 2.0, True) == 2.0  # 残り時間が長い → 普段まで
    assert cap_stop_end_tau(1.0, 2.0, True) == 1.0  # 短いときはそのまま
    assert cap_stop_end_tau(None, 2.0, True) is None
    assert cap_stop_end_tau(3.0, None, True) == 3.0


class _SideTable:
    def accel(self, v: float, u: float, *, launch: bool = False, stop: bool = False,
              **_: object) -> float:
        return -1.0 + (5.0 if launch else 0.0) + (-3.0 if stop else 0.0) + 0.3 * u


def _drive(obs: PedalDisturbanceObserver, *, side_tables: bool, **side: bool) -> None:
    t, v = 0.0, 5.0
    for _ in range(60):
        obs.update(t=t, speed_kmh=v, u_now=10.0, governed=False, at_limit_now=False,
                   launch=side.get("launch", False), stop=side.get("stop", False),
                   side_tables=side_tables)
        t += 0.1
        v += (_SideTable().accel(v, 10.0, **side) - 0.2) * 0.1


def test_dob_side_tables_updates_with_plan_table_only_when_asked() -> None:
    for side in ({"launch": True}, {"stop": True}):
        off = PedalDisturbanceObserver(_SideTable(), tq_accel_s=1.0, tq_brake_s=1.0,
                                       lag_accel_s=0.0, lag_brake_s=0.0)
        _drive(off, side_tables=False, **side)
        assert off.n_accel == 0 and off.d_accel == 0.0  # 今までどおり更新しない
        on = PedalDisturbanceObserver(_SideTable(), tq_accel_s=1.0, tq_brake_s=1.0,
                                      lag_accel_s=0.0, lag_brake_s=0.0)
        _drive(on, side_tables=True, **side)
        assert on.n_accel > 0 and on.d_accel == pytest.approx(-0.2, abs=0.02)  # 側の表で残差


def _fake_split(lo: float = 5.0, hi: float = 10.0) -> types.SimpleNamespace:
    o = _fake(True, True)
    fm = o.cfg.forward_map
    fm.dob_split_low_speed, fm.dob_split_blend_lo_kmh, fm.dob_split_blend_hi_kmh = True, lo, hi
    o.dob = PedalDisturbanceObserver(
        _SideTable(), tq_accel_s=1.0, tq_brake_s=1.0, lag_accel_s=0.0, lag_brake_s=0.0,
        split_low=True,
    )
    o.dob.d_accel, o.dob.d_accel_low = -0.4, 2.0
    o._dob_estimate = types.MethodType(_ModeRun._dob_estimate, o)
    return o


def test_dob_estimate_split_off_is_estimate_times_weight() -> None:
    o = _fake(True, True)
    o.cfg.forward_map.dob_split_low_speed = False
    o.dob = PedalDisturbanceObserver(
        _SideTable(), tq_accel_s=1.0, tq_brake_s=1.0, lag_accel_s=0.0, lag_brake_s=0.0)
    o.dob.d_accel = -0.4
    o._dob_estimate = types.MethodType(_ModeRun._dob_estimate, o)
    for v in (1.0, 4.0, 7.5, 20.0):
        assert o._dob_estimate(True, v, 0, v) == pytest.approx(-0.4 * o._dob_weight(v, 0, v))


def test_dob_estimate_split_on_blends_low_and_main() -> None:
    o = _fake_split()
    w = o._dob_weight  # 低速側は w（7.5 → 0.5 km/h で 1 → 0）を掛ける
    assert o._dob_estimate(True, 4.0, 0, 4.0) == pytest.approx(w(4.0, 0, 4.0) * 2.0)
    assert o._dob_estimate(True, 5.0, 0, 5.0) == pytest.approx(w(5.0, 0, 5.0) * 2.0)
    assert o._dob_estimate(True, 10.0, 0, 10.0) == pytest.approx(-0.4)
    assert o._dob_estimate(True, 30.0, 0, 30.0) == pytest.approx(-0.4)
    # 間は線形（7.5 で s = 0.5, w = 1）
    assert o._dob_estimate(True, 7.5, 0, 7.5) == pytest.approx(0.5 * -0.4 + 0.5 * 2.0)
    # 境目 7.5 の前後で連続
    below = o._dob_estimate(True, 7.5 - 1e-6, 0, 7.5 - 1e-6)
    above = o._dob_estimate(True, 7.5 + 1e-6, 0, 7.5 + 1e-6)
    assert below == pytest.approx(above, abs=1e-4)
    # 境目は基準車速で見る（align）。停車速度以下は w = 0 で低速の寄与なし
    assert o._dob_estimate(True, 30.0, 0, 4.0) == pytest.approx(w(30.0, 0, 4.0) * 2.0)
    assert o._dob_estimate(True, STOP_SPEED_KMH, 0, 0.0) == 0.0


def test_split_low_config_validation() -> None:
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.dob_split_low_speed is False
    assert (fm0.dob_split_blend_lo_kmh, fm0.dob_split_blend_hi_kmh) == (5.0, 10.0)

    def problems(**kw: object) -> list[str]:
        cfg = cfgmod.load_config(Path("tests/research/config_testVehicle.yaml"))
        for k, val in kw.items():
            setattr(cfg.forward_map, k, val)
        return [p for p in cfgmod.validate_config(cfg) if "dob_split" in p]

    assert problems() == []
    base = {"dob_low_speed_fade": True, "dob_low_speed_align": True}
    assert problems(dob_split_low_speed=True, **base) == []
    assert problems(dob_split_low_speed=True, dob_low_speed_fade=False, dob_low_speed_align=True)
    assert problems(dob_split_low_speed=True, dob_low_speed_fade=True, dob_low_speed_align=False)
    assert problems(dob_split_blend_lo_kmh=0.0)
    assert problems(dob_split_blend_lo_kmh=10.0, dob_split_blend_hi_kmh=10.0)
