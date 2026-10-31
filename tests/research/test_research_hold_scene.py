"""保持の場面の PI・保持の開度の FF（2026-10-30 段4・7・8。hold_scene.py と mode_drive）。"""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import mode_drive as mdmod
from tests.research.hold_check import slope_threshold_kmhs as check_slope
from tests.research.hold_scene import (
    HoldInterval,
    HoldScenePI,
    banded_accel,
    hold_lead_in_intervals,
    hold_opening_points,
    hold_scene_intervals,
    in_hold_interval,
    interp_hold_opening,
    is_hold_scene,
    lead_in_ff_pct,
    min_scene_length_s,
    predicted_deviation,
    predicted_deviation_current_g,
    predicted_deviation_floor0,
    slope_threshold_kmhs,
)
from tests.research.test_research_mode_drive import _c7_cfg, _held_stub, _mode

SLOPE = slope_threshold_kmhs(0.005)


def _scene(now: float, ahead: float, lag: float = 0.155) -> bool:
    return is_hold_scene(now, ahead, lag, slope_max_kmhs=SLOPE, stop_kmh=0.5, creep_kmh=5.0)


def test_slope_threshold_matches_hold_check() -> None:
    assert SLOPE == pytest.approx(check_slope(0.005))
    assert SLOPE == pytest.approx(0.005 * 9.80665 * 3.6)


def test_is_hold_scene_conditions() -> None:
    assert _scene(1.6, 1.6)  # 平ら
    assert not _scene(1.6, 1.6 + 0.5 * 0.155)  # 傾き 0.5 km/h/s > しきい値
    assert not _scene(1.6, 1.6 - 0.5 * 0.155)  # 下りも同じ（大きさで見る）
    assert not _scene(0.0, 0.0)  # 停車
    assert not _scene(0.5, 0.5)  # 停車速度ちょうどは含まない
    assert not _scene(5.0, 5.0)  # クリープ車速以上
    assert not _scene(7.0, 7.0)
    assert not _scene(1.6, 1.6, lag=0.0)  # L が決まらない


def test_pi_sign_slow_means_shallower_brake() -> None:
    pi = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    effort, add = pi.step(0.5, -8.84)  # 遅い（偏差 +）
    assert add > 0.0 and effort > -8.84  # ブレーキが浅くなる
    pi.reset()
    effort, add = pi.step(-0.5, -8.84)  # 速い（偏差 −）
    assert add < 0.0 and effort < -8.84  # ブレーキが深くなる
    assert add == pytest.approx(0.04 * -0.5 + 0.14 * -0.5 * 0.05)


def test_pi_clamps_and_does_not_wind_up() -> None:
    pi = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    # 遅れが続いて浅い端（−7）を超える側へ: 出力は −7 で止まり、積分は進まない
    for _ in range(200):
        effort, _ = pi.step(5.0, -7.0)
    assert effort == pytest.approx(-7.0)
    assert pi.integral == 0.0
    # 逆向きの偏差ならすぐ内側へ戻れる（巻き上がっていない）
    effort, _ = pi.step(-1.0, -7.0)
    assert effort < -7.0
    # 深い端（−22）側
    pi.reset()
    for _ in range(200):
        effort, _ = pi.step(-50.0, -21.0)
    assert effort == pytest.approx(-22.0)
    integral = pi.integral
    pi.step(-50.0, -21.0)
    assert pi.integral == integral


def test_pi_integral_starts_at_zero_on_each_entry() -> None:
    pi = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    for _ in range(20):
        pi.step(0.3, -9.0)
    assert pi.integral > 0.0
    pi.reset()  # 場面を出た
    assert pi.integral == 0.0
    _, add = pi.step(0.3, -9.0)
    assert add == pytest.approx(0.04 * 0.3 + 0.14 * 0.3 * 0.05)  # I は 1 周期ぶんだけ


def test_pi_deadband_stops_integral_inside_only() -> None:
    pi = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0, deadband_kmh=0.15)
    for dev in (0.11, -0.15, 0.0):  # 不感帯の中（境目を含む）: 積分は進まず P だけ
        _, add = pi.step(dev, -9.0)
        assert pi.integral == 0.0
        assert add == pytest.approx(0.04 * dev)
    _, add = pi.step(0.16, -9.0)  # 外: 進む
    assert pi.integral == pytest.approx(0.14 * 0.16 * 0.05)
    assert add == pytest.approx(0.04 * 0.16 + 0.14 * 0.16 * 0.05)


def test_pi_deadband_default_zero_is_unchanged() -> None:
    a = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    b = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0, deadband_kmh=0.0)
    assert a.deadband_kmh == 0.0
    for dev in (0.01, -0.02, 0.3, 0.0):
        assert a.step(dev, -9.0) == b.step(dev, -9.0)
    assert a.integral == pytest.approx(0.14 * 0.05 * (0.01 - 0.02 + 0.3))


def _meta() -> dict:
    return {"hold_opening": {
        "points": [
            {"speed_kmh": 1.0, "opening_pct": 8.0, "held": True},
            {"speed_kmh": 2.0, "opening_pct": 30.0, "held": False},  # 飛ばす
            {"speed_kmh": 3.0, "opening_pct": 7.6, "held": True},
        ],
        "creep_kmh": 5.0, "breakaway_pct": 8.6, "min_pct": 7.0, "source_csv": "x.csv",
    }}


def test_hold_opening_interpolation_uses_held_points_and_ends() -> None:
    pts = hold_opening_points(_meta())
    assert pts == [(0.0, 8.6), (1.0, 8.0), (3.0, 7.6), (5.0, 7.0)]
    assert interp_hold_opening(pts, 0.0) == pytest.approx(8.6)  # 両端
    assert interp_hold_opening(pts, 5.0) == pytest.approx(7.0)
    assert interp_hold_opening(pts, 2.0) == pytest.approx(7.8)  # held=False の 2.0 は飛ばす
    assert hold_opening_points({}) is None
    assert hold_opening_points({"hold_opening": {"points": []}}) is None  # 形が違う


def _hold_cfg(tmp_path: Path, *, with_hold_pi: bool, with_opening: bool = False):
    from tests.research.fwd_map import load_forward_map, save_table

    cfg = _c7_cfg(tmp_path)
    path = Path(cfg.feedforward.model_path)
    table = load_forward_map(path)
    if with_hold_pi:
        table.meta["hold_pi"] = {"kp_pct_per_kmh": 0.0417, "ki_pct_per_kmh_s": 0.137,
            "lambda_s": 0.933, "L_s": 0.1555}
    if with_opening:
        table.meta.update(_meta())
    save_table(table, path)
    cfg.forward_map.hold_pi_enabled = True
    cfg.forward_map.hold_lead_in_enabled = False  # 段11 は別のテストで（本体 yaml は on）
    cfg.mode_drive.pedal_standby = False
    return cfg


def test_defaults_are_off_and_identical() -> None:
    fm = cfgmod.ForwardMapSection()
    assert (fm.hold_pi_enabled, fm.hold_ff_enabled, fm.hold_ff_offset_pct) == (False, False, 0.0)
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)  # noqa: SLF001
    assert run.hold is None
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.forward_map.hold_pi_enabled = False  # 本体 yaml は on なので既定（off）に戻して確かめる
    assert mdmod.make_hold_scene(cfg, object()) is None


def test_hold_pi_without_meta_is_config_error_before_run(tmp_path: Path) -> None:
    cfg = _hold_cfg(tmp_path, with_hold_pi=False)
    mode = _mode([(0.0, 0.0), (1.0, 1.6), (5.0, 1.6)])
    with pytest.raises(cfgmod.ConfigError, match="hold_pi"):
        mdmod.prepare_mode_drive_from(cfg, mode)


async def test_run_logs_hold_scene_and_pi(tmp_path: Path) -> None:
    cfg = _hold_cfg(tmp_path, with_hold_pi=True, with_opening=True)
    cfg.forward_map.hold_ff_enabled = True
    cfg.forward_map.hold_ff_offset_pct = 0.3
    mode = _mode([(0.0, 0.0), (1.0, 1.6), (6.0, 1.6)])
    setup = mdmod.prepare_mode_drive_from(cfg, mode)
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    result = await mdmod.run_mode_drive(hw, cfg, setup, log=log)
    await log.close()
    rows = [s for s in result.samples if s.hold_scene == 1]
    assert rows
    for s in rows:
        assert s.hold_pi_effort_pct is not None
        want = -(interp_hold_opening(hold_opening_points(_meta()), s.data.ref_speed_kmh) + 0.3)
        assert s.hold_ff_pct == pytest.approx(-want)
        assert s.data.plan_effort_pct != pytest.approx(want)  # 計画の列はそのまま
    assert any(s.hold_scene == 0 for s in result.samples)  # 停車の行は外
    out = [s for s in result.samples if s.hold_scene == 0]
    assert all(s.hold_pi_effort_pct is None and s.hold_ff_pct is None for s in out)


def _intervals(points: list[tuple[float, float]], duration: float, min_len: float):
    def ref_at(t: float) -> float:
        if t <= points[0][0]:
            return points[0][1]
        for (t0, v0), (t1, v1) in zip(points, points[1:]):
            if t <= t1:
                return v0 + (t - t0) / (t1 - t0) * (v1 - v0)
        return points[-1][1]

    return hold_scene_intervals(
        ref_at, duration, 0.05, 0.1, slope_max_kmhs=SLOPE, stop_kmh=0.5, creep_kmh=5.0,
        min_length_s=min_len,
    )


def test_min_scene_length_is_twice_lambda_plus_l() -> None:
    assert min_scene_length_s(0.933, 0.1555) == pytest.approx(2.1770)


def test_long_flat_interval_is_kept_whole_and_short_is_dropped() -> None:
    # 0〜1 s で 0→2 km/h、2 km/h 一定 3 s、4 s 後に 0 へ。さらに 1 s だけ 1 km/h の平らな区間
    pts = [(0, 0), (1, 2.0), (4, 2.0), (5, 0.0), (6, 0.0), (6.5, 1.0), (7.5, 1.0), (8, 0.0)]
    kept, dropped = _intervals(pts, 10.0, 2.18)
    assert dropped >= 1
    assert len(kept) == 1
    iv = kept[0]
    assert iv.length_s >= 2.18
    assert iv.mean_kmh == pytest.approx(2.0, abs=0.3)
    step = 0.05
    assert in_hold_interval(kept, iv.start_s, step)  # 最初の周期から
    assert in_hold_interval(kept, iv.end_s - step, step)  # 最後の周期まで
    assert not in_hold_interval(kept, iv.end_s + step, step)
    assert not in_hold_interval(kept, iv.start_s - step, step)
    assert not in_hold_interval(kept, 7.0, step)  # 短い方は入らない


def test_interval_exactly_at_the_limit_is_kept() -> None:
    pts = [(0, 0), (0.5, 2.0), (3.5, 2.0), (4, 0.0), (6, 0.0)]
    kept, _ = _intervals(pts, 6.0, 1.0)
    length = kept[0].length_s
    kept_eq, dropped_eq = _intervals(pts, 6.0, length)
    assert len(kept_eq) == 1 and dropped_eq == 0
    kept_gt, dropped_gt = _intervals(pts, 6.0, length + 0.01)
    assert kept_gt == [] and dropped_gt == 1


# ---- 段11: 保持の直前の上り ----


def _ramp_ref(t: float) -> float:
    """5 s まで 0・1.6 km/h/s で 6 s に 1.6・以後 20 s まで一定。"""
    return min(max(1.6 * (t - 5.0), 0.0), 1.6)


def _lead(ref, start_s: float, length_s: float, mean: float):
    return hold_lead_in_intervals(
        ref, [HoldInterval(start_s, length_s, mean)], 0.05, stop_kmh=0.5, creep_kmh=5.0
    )


def test_lead_in_rising_from_stop_stops_at_stop_speed() -> None:
    # 0.5 km/h を超える最初の周期は 5.35 s（1.6·0.35 ＝ 0.56）。区間は 6.0 s から
    (iv,) = _lead(_ramp_ref, 6.0, 14.0, 1.6)
    assert iv.start_s == pytest.approx(5.35)
    assert iv.end_s == pytest.approx(6.0)
    assert iv.mean_kmh == pytest.approx(1.6)


def test_lead_in_none_when_preceding_samples_are_above_creep() -> None:
    def ref(t: float) -> float:
        return 8.0 if t < 6.0 else 1.6

    assert _lead(ref, 6.0, 5.0, 1.6) == []


def test_lead_in_falling_approach_is_detected() -> None:
    def ref(t: float) -> float:
        return min(max(4.0 - 2.4 * (t - 5.0), 1.6), 4.0)  # 4.0 → 1.6（5〜6 s）

    (iv,) = _lead(ref, 6.0, 5.0, 1.6)
    assert iv.start_s <= 5.0 + 1e-9  # 4.0 の平らな所も向かう先に近づく側（遠ざからない）
    assert iv.end_s == pytest.approx(6.0)
    assert iv.start_s == pytest.approx(0.0)


def test_lead_in_stops_where_ref_moves_away_from_target() -> None:
    def ref(t: float) -> float:
        if t < 3.0:
            return 1.0  # 目標 1.6 に近い側から
        if t < 4.0:
            return 1.0 + 0.5 * (t - 3.0)  # 3〜4 s は 1.0 → 1.5（出来物）
        if t < 5.0:
            return 1.5 - 0.9 * (t - 4.0)  # 4〜5 s は 1.5 → 0.6（遠ざかる）
        return min(0.6 + 1.0 * (t - 5.0), 1.6)

    (iv,) = _lead(ref, 6.0, 5.0, 1.6)
    assert iv.start_s == pytest.approx(5.0)  # 5 s より前（遠ざかる側）には入らない


def test_lead_in_does_not_enter_previous_interval() -> None:
    ivs = [HoldInterval(0.0, 3.0, 1.6), HoldInterval(5.0, 3.0, 1.6)]
    out = hold_lead_in_intervals(lambda t: 1.6, ivs, 0.05, stop_kmh=0.5, creep_kmh=5.0)
    assert [o.start_s for o in out] == [pytest.approx(3.0)]
    assert out[0].end_s == pytest.approx(5.0)


def test_lead_in_ff_pct_formula_clip_and_bad_c() -> None:
    pts = [(0.0, 9.0), (5.0, 4.0)]  # 1.6 km/h → 9 − 0.8·1.6 ＝ 7.72
    base = interp_hold_opening(pts, 1.6)
    assert lead_in_ff_pct(pts, 1.6, 1.6, 2.0, 4.0, 12.0) == pytest.approx(base - 0.8)
    assert lead_in_ff_pct(pts, 1.6, 100.0, 2.0, 4.0, 12.0) == 4.0  # 下端
    assert lead_in_ff_pct(pts, 1.6, -100.0, 2.0, 4.0, 12.0) == 12.0  # 上端
    assert lead_in_ff_pct(pts, 1.6, 1.6, 0.0, 4.0, 12.0) == pytest.approx(base)
    assert lead_in_ff_pct(pts, 1.6, 1.6, float("nan"), 4.0, 12.0) == pytest.approx(base)


def test_hold_lead_in_enabled_default_false() -> None:
    assert cfgmod.ForwardMapSection().hold_lead_in_enabled is False


def test_pi_p_dev_none_is_identical_to_old_behaviour() -> None:
    a = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    b = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    for dev in (0.5, -0.2, 0.1):
        assert a.step(dev, -8.84) == b.step(dev, -8.84, None)
        assert a.integral == b.integral
    c = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    d = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    assert c.step(0.5, -8.84) == d.step(0.5, -8.84, 0.5)  # p_dev ＝ dev なら同じ


def test_pi_p_dev_drives_p_but_integral_uses_dev() -> None:
    pi = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0)
    _, add = pi.step(0.5, -8.84, p_dev_kmh=-1.0)
    assert add == pytest.approx(0.04 * -1.0 + 0.14 * 0.5 * 0.05)  # P は p_dev・I は dev
    assert pi.integral == pytest.approx(0.14 * 0.5 * 0.05)


def test_pi_deadband_still_uses_dev_with_p_dev() -> None:
    pi = HoldScenePI(kp=0.04, ki=0.14, dt_s=0.05, lo_pct=7.0, hi_pct=22.0, deadband_kmh=0.15)
    _, add = pi.step(0.1, -8.84, p_dev_kmh=-1.0)  # dev は不感帯の中 → 積分は進まない
    assert pi.integral == 0.0
    assert add == pytest.approx(0.04 * -1.0)
    pi.step(0.5, -8.84, p_dev_kmh=0.0)  # dev は外 → 積分は進む
    assert pi.integral == pytest.approx(0.14 * 0.5 * 0.05)


def test_predicted_deviation_floor0_stops_predicted_speed_at_zero() -> None:
    # 2026-10-31 段3b: 1.98 + (-4.5)*0.655 < 0 → 予測車速は 0。偏差 ＝ 基準 − 0
    assert predicted_deviation_floor0(0.0, 1.98, -4.5, 0.655) == pytest.approx(0.0)
    assert predicted_deviation_floor0(0.3, 1.98, -4.5, 0.655) == pytest.approx(0.3)
    # 負にならない入力は predicted_deviation と同じ
    assert predicted_deviation_floor0(1.6, 1.0, 1.5, 0.4) == pytest.approx(
        predicted_deviation(1.6, 1.0, 1.5, 0.4)
    )


def test_predicted_deviation_formula() -> None:
    assert predicted_deviation(1.6, 1.0, 1.5, 0.4) == pytest.approx(1.6 - (1.0 + 0.6))
    assert predicted_deviation(1.6, 1.6, 0.0, 0.4) == pytest.approx(0.0)
    assert predicted_deviation(1.6, 1.0, -1.0, 0.5) == pytest.approx(1.1)


def test_banded_accel_none_inside_and_outside_band() -> None:
    assert banded_accel(None, 0.18) == 0.0
    assert banded_accel(0.18, 0.18) == 0.0
    assert banded_accel(-0.1, 0.18) == 0.0
    assert banded_accel(1.5, 0.18) == 1.5
    assert banded_accel(-1.5, 0.18) == -1.5


def test_hold_accel_pred_default_false_and_needs_hold_pi() -> None:
    assert cfgmod.ForwardMapSection().hold_accel_pred_enabled is False
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.forward_map.hold_pi_enabled = False
    cfg.forward_map.hold_lead_in_enabled = False
    cfg.forward_map.hold_accel_pred_enabled = True
    assert any("hold_accel_pred_enabled" in e for e in cfgmod.validate_config(cfg))
    cfg.forward_map.hold_pi_enabled = True
    assert not any("hold_accel_pred_enabled" in e for e in cfgmod.validate_config(cfg))


def test_predicted_deviation_current_g() -> None:
    # 基準の G と実測の G が同じ: 今の偏差（基準 − 車速）
    assert predicted_deviation_current_g(10.0, -3.0, 9.0, -3.0, 0.5) == pytest.approx(1.0)
    # 実測が基準より強い減速: 遅すぎ側（正）
    assert predicted_deviation_current_g(10.0, -3.0, 9.0, -5.0, 0.5) == pytest.approx(2.0)
    # 2026-10-31 段3d: 基準か車速が Tp 後に 0 以下と予測されたら今の偏差（基準 − 車速）
    assert predicted_deviation_current_g(1.0, -4.0, 2.0, -4.0, 0.5) == pytest.approx(-1.0)
    assert predicted_deviation_current_g(0.3, 0.0, 1.0, -4.0, 0.5) == pytest.approx(-0.7)
