"""pedal_arbiter（調停の移植）のテスト。全スイッチ off は split_effort と同一であること。"""

from __future__ import annotations

import pytest

from tests.research import config as cfgmod
from tests.research.mode_drive import split_effort
from tests.research.pedal_arbiter import PedalArbiter, enabled_arbiter_features, plan_accel_kmhs

DT = 0.05


def _arb(section: cfgmod.ArbiterSection | None = None, **flags: bool) -> PedalArbiter:
    sec = section or cfgmod.ArbiterSection()
    for k, v in flags.items():
        setattr(sec, k, v)
    return PedalArbiter(sec, accel_deadband_pct=2.0, brake_deadband_pct=10.0,
                        max_accel_opening=80.0, max_brake_opening=60.0, nominal_dt_s=DT)


def test_all_off_equals_split_effort() -> None:
    arb = _arb()
    for effort in (-100.0, -60.0, -59.9, -10.0, -0.01, 0.0, 0.01, 0.3, 2.0, 79.9, 80.0, 150.0):
        out = arb.arbitrate(effort, DT)
        assert (out.accel_opening, out.brake_opening) == split_effort(effort, 80.0, 60.0)


def test_all_off_sequence_has_no_memory() -> None:
    arb = _arb()
    for effort in (30.0, -20.0, 0.0, 5.0, -1.0, 40.0):
        out = arb.arbitrate(effort, DT)
        assert (out.accel_opening, out.brake_opening) == split_effort(effort, 80.0, 60.0)


def test_min_step_holds_small_changes() -> None:
    arb = _arb(cfgmod.ArbiterSection(accel_min_step_pct=0.5), enable_min_step=True)
    arb.arbitrate(10.0, DT)
    assert arb.arbitrate(10.3, DT).accel_opening == 10.0  # 0.3 < 0.5 は保持
    assert arb.arbitrate(10.4, DT).accel_opening == 10.0
    assert arb.arbitrate(10.6, DT).accel_opening == pytest.approx(10.6)  # 0.6 >= 0.5 は追従


def test_min_step_threshold_value() -> None:
    sec = cfgmod.ArbiterSection(accel_min_step_pct=0.5)
    arb = _arb(sec, enable_min_step=True)
    arb.arbitrate(10.0, DT)
    assert arb.arbitrate(10.49, DT).accel_opening == 10.0
    assert arb.arbitrate(10.5, DT).accel_opening == pytest.approx(10.5)


def test_brake_selection_zeroes_accel_immediately() -> None:
    arb = _arb(enable_release_rate=True, enable_rate_limit=True, enable_min_step=True)
    for _ in range(5):
        arb.arbitrate(30.0, DT)
    out = arb.arbitrate(-20.0, DT)
    assert out.accel_opening == 0.0
    assert out.brake_opening > 0.0


def test_hysteresis_band_coasts() -> None:
    arb = _arb(enable_hysteresis=True)  # 帯 ±0.5
    out = arb.arbitrate(0.4, DT)
    assert (out.accel_opening, out.brake_opening) == (0.0, 0.0)
    assert arb.arbitrate(-0.4, DT).brake_opening == 0.0
    assert arb.arbitrate(0.6, DT).accel_opening == pytest.approx(0.6)
    # off なら同じ 0.4 でも踏む
    assert _arb().arbitrate(0.4, DT).accel_opening == pytest.approx(0.4)


def test_deadband_compensation() -> None:
    arb = _arb(enable_deadband_compensation=True)  # アクセル 2.0 / ブレーキ 10.0
    assert arb.arbitrate(0.9, DT).accel_opening == 0.0  # < db/2
    assert arb.arbitrate(1.5, DT).accel_opening == 2.0  # db に持ち上げ
    assert arb.arbitrate(5.0, DT).accel_opening == 5.0
    assert arb.arbitrate(-4.0, DT).brake_opening == 0.0
    assert arb.arbitrate(-7.0, DT).brake_opening == 10.0


def test_rate_limit_increase_only() -> None:
    arb = _arb(enable_rate_limit=True)  # アクセル 200%/s × 0.05 = 10%/周期
    out = arb.arbitrate(50.0, DT)
    assert out.accel_opening == pytest.approx(10.0)
    assert out.saturated_high
    assert arb.arbitrate(50.0, DT).accel_opening == pytest.approx(20.0)
    assert arb.arbitrate(5.0, DT).accel_opening == pytest.approx(5.0)  # 減少は無制限
    assert arb.arbitrate(-50.0, DT).brake_opening == pytest.approx(15.0)  # 300%/s


def test_reengage_dwell_after_brake() -> None:
    arb = _arb(enable_reengage_dwell=True)  # 0.3s
    arb.arbitrate(-20.0, DT)
    out = arb.arbitrate(20.0, DT)
    assert out.accel_opening == 0.0 and out.saturated_high
    for _ in range(4):
        arb.arbitrate(20.0, DT)
    assert arb.arbitrate(20.0, DT).accel_opening == 20.0  # 0.3s 経過
    # off なら即踏む
    off = _arb()
    off.arbitrate(-20.0, DT)
    assert off.arbitrate(20.0, DT).accel_opening == 20.0


def test_release_rate_ramps_down_on_coast() -> None:
    arb = _arb(enable_release_rate=True)  # 10%/s × 0.05 = 0.5%/周期
    arb.arbitrate(10.0, DT)
    assert arb.arbitrate(0.0, DT).accel_opening == pytest.approx(9.5)
    assert arb.arbitrate(0.0, DT).accel_opening == pytest.approx(9.0)
    assert _arb().arbitrate(0.0, DT).accel_opening == 0.0


def test_reset_clears_state() -> None:
    arb = _arb(enable_release_rate=True, enable_reengage_dwell=True)
    arb.arbitrate(-20.0, DT)
    arb.arbitrate(10.0, DT)
    arb.reset()
    assert arb.arbitrate(10.0, DT).accel_opening == 10.0


def test_enabled_features_labels() -> None:
    sec = cfgmod.ArbiterSection()
    assert enabled_arbiter_features(sec) == []
    sec.enable_min_step = True
    sec.enable_reengage_dwell = True
    sec.enable_release_rate = True
    sec.enable_hysteresis = True
    labels = enabled_arbiter_features(sec)
    assert "微小変化の保持 0.2%" in labels
    assert "再踏込ディレイ 0.3s" in labels
    assert "アクセル解放レート 10 %/s" in labels
    assert "切替ヒステリシス ±0.5%" in labels


# ── 段3c: 加速度帯の保持 ────────────────────────────────────────────


def _band_arb(**flags: bool) -> PedalArbiter:
    return _arb(enable_accel_band=True, **flags)


def test_plan_accel_kmhs_linear_and_constant() -> None:
    assert plan_accel_kmhs(lambda t: 2.0 * t + 10.0, 5.0, 3.0) == pytest.approx(2.0)
    assert plan_accel_kmhs(lambda t: 30.0, 5.0, 3.0) == pytest.approx(0.0)


def test_accel_band_holds_inside_band() -> None:
    arb = _band_arb()
    assert arb.arbitrate(10.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0).accel_opening == 10.0
    out = arb.arbitrate(11.0, DT, plan_accel_kmhs=1.2, deviation_kmh=0.1)
    assert out.accel_opening == 10.0  # 帯（±0.25）の中は保持


def test_accel_band_open_escape_follows() -> None:
    """帯の中（計画・偏差は不変）でも、要求が保持開度から 2% 以上離れたら追従する。"""
    arb = _band_arb()
    arb.arbitrate(10.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0)
    assert arb.arbitrate(11.9, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0).accel_opening == 10.0
    assert arb.arbitrate(8.1, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0).accel_opening == 10.0
    assert arb.arbitrate(8.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0).accel_opening == 8.0
    # 追従したので保持開度は 8.0: そこから 2% 未満は再び保持
    assert arb.arbitrate(9.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0).accel_opening == 8.0


def test_accel_band_exit_follows_and_reanchors() -> None:
    arb = _band_arb()
    arb.arbitrate(10.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0)
    assert arb.arbitrate(13.0, DT, plan_accel_kmhs=1.3, deviation_kmh=0.0).accel_opening == 13.0
    # アンカーが 1.3 に移った: 1.5 は帯の中で保持、1.6 は外で追従
    assert arb.arbitrate(14.0, DT, plan_accel_kmhs=1.5, deviation_kmh=0.0).accel_opening == 13.0
    assert arb.arbitrate(15.0, DT, plan_accel_kmhs=1.6, deviation_kmh=0.0).accel_opening == 15.0


def test_accel_band_deviation_escape_follows() -> None:
    arb = _band_arb()
    arb.arbitrate(10.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0)
    assert arb.arbitrate(11.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.3).accel_opening == 10.0
    assert arb.arbitrate(11.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.4).accel_opening == 11.0


def test_accel_band_anchor_cleared_by_brake_and_coast() -> None:
    for between in (-30.0, 0.0):
        arb = _band_arb()
        arb.arbitrate(10.0, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0)
        arb.arbitrate(between, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0)
        out = arb.arbitrate(10.5, DT, plan_accel_kmhs=1.0, deviation_kmh=0.0)
        assert out.accel_opening == 10.5  # アンカーが消えているので追従


def test_accel_band_without_inputs_behaves_as_before() -> None:
    arb = _band_arb()
    for effort in (10.0, 10.4, 12.0):
        assert arb.arbitrate(effort, DT).accel_opening == split_effort(effort, 80.0, 60.0)[0]


# ── 段3d: 向きのヒステリシス ────────────────────────────────────────


def _dir_arb(w: float = 0.5) -> PedalArbiter:
    return _arb(cfgmod.ArbiterSection(accel_direction_hysteresis_pct=w),
                enable_direction_hysteresis=True)


def test_direction_hysteresis_same_direction_passes() -> None:
    arb = _dir_arb()
    arb.arbitrate(10.0, DT)
    assert arb.arbitrate(11.0, DT).accel_opening == 11.0  # 向きが上に決まる
    assert arb.arbitrate(11.1, DT).accel_opening == pytest.approx(11.1)  # 同じ向きは小さくても通る
    assert arb.arbitrate(11.2, DT).accel_opening == pytest.approx(11.2)


def test_direction_hysteresis_opposite_holds_then_reverses() -> None:
    arb = _dir_arb()
    arb.arbitrate(10.0, DT)
    arb.arbitrate(11.0, DT)  # 上向き
    assert arb.arbitrate(10.8, DT).accel_opening == 11.0  # 逆向き 0.2 < 0.5 は保持
    assert arb.arbitrate(10.5, DT).accel_opening == pytest.approx(10.5)  # 0.5 以上で反転
    assert arb.arbitrate(10.4, DT).accel_opening == pytest.approx(10.4)  # 下向きは即追従
    assert arb.arbitrate(10.6, DT).accel_opening == pytest.approx(10.4)  # 上向き 0.2 は保持


def test_direction_hysteresis_reset_and_brake_clear_state() -> None:
    arb = _dir_arb()
    arb.arbitrate(10.0, DT)
    arb.arbitrate(11.0, DT)
    arb.reset()
    assert arb._accel_dir == 0
    arb.arbitrate(10.0, DT)
    arb.arbitrate(11.0, DT)
    arb.arbitrate(-30.0, DT)
    assert arb._accel_dir == 0


def test_band_and_direction_labels() -> None:
    sec = cfgmod.ArbiterSection(enable_accel_band=True, enable_direction_hysteresis=True)
    labels = enabled_arbiter_features(sec)
    assert "加速度帯の保持 ±0.25km/h/s（3s先の傾き）・偏差変化 0.3km/h・開度差 2%" in labels
    assert "向きのヒステリシス 0.5%" in labels
