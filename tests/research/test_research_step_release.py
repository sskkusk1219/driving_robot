"""遅すぎのとき少しずつ戻す（2026-11-05 段15 stop_approach_step_release）のテスト。
mode_drive.step_release_brake（走行中の判定の純関数）・step_release_setup（効きの読み出し・
meta が無ければ ConfigError）・config の既定値と検証。"""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from tests.research import config as cfgmod
from tests.research import mode_drive as mdmod
from tests.research.config import ConfigError

GAIN = 4.0  # km/h/s per %
LAG = 0.15
MIN_PCT = 4.0
WAIT = 5
KW = dict(
    in_zone=True, first=False, speed_kmh=0.8, dev_kmh=-0.1, slope_kmhs=-2.0, lag_s=LAG,
    release_dev_kmh=0.3, gain_kmhs_pct=GAIN, min_pct=MIN_PCT, wait_left=0, wait_cycles=WAIT,
)


def _step(prev: float | None, new: float, **over):
    kw = {**KW, **over}
    return mdmod.step_release_brake(prev, new, **kw)


def test_releases_once_by_slope_over_gain_when_forecast_too_slow() -> None:
    # 見込み = −0.1 + (−2.0) × 0.15 = −0.4 < −0.3、傾き負 → 1 回戻す。量 = 2.0 ÷ 4.0 = 0.5%
    pct, code, wait = _step(9.0, 9.0)
    assert code == mdmod.STEP_CODE_STEPPED
    assert pct == pytest.approx(8.5)
    assert wait == WAIT


def test_wait_cycles_keep_previous_command_then_count_down() -> None:
    pct, code, wait = _step(8.5, 9.5, wait_left=WAIT)  # 新しい指令が深くても前のまま
    assert (pct, code, wait) == (8.5, mdmod.STEP_CODE_WAIT, WAIT - 1)
    pct, code, wait = _step(8.5, 7.0, wait_left=1)  # 新しい指令が浅くても前のまま
    assert (pct, code, wait) == (8.5, mdmod.STEP_CODE_WAIT, 0)


def test_after_wait_slope_nonnegative_holds_like_now() -> None:
    pct, code, wait = _step(8.5, 7.0, slope_kmhs=0.0)  # 浅くしない
    assert (pct, code, wait) == (8.5, mdmod.STEP_CODE_HELD, 0)
    pct, code, wait = _step(8.5, 9.0, slope_kmhs=0.5)  # 深くするのは良い
    assert (pct, code, wait) == (9.0, mdmod.STEP_CODE_NONE, 0)


def test_second_release_when_still_too_slow_after_wait() -> None:
    pct1, code1, wait1 = _step(9.0, 9.0)
    assert code1 == mdmod.STEP_CODE_STEPPED
    for _ in range(WAIT):
        pct1, code, wait1 = _step(pct1, 9.0, wait_left=wait1)
        assert code == mdmod.STEP_CODE_WAIT
    assert wait1 == 0
    pct2, code2, _ = _step(pct1, 9.0, wait_left=wait1)
    assert code2 == mdmod.STEP_CODE_STEPPED and pct2 < pct1


def test_no_release_when_forecast_not_late_enough_or_slope_not_negative() -> None:
    assert _step(9.0, 8.0, dev_kmh=0.2)[1] == mdmod.STEP_CODE_HELD  # 見込み −0.1 > −0.3
    assert _step(9.0, 8.0, slope_kmhs=None)[1] == mdmod.STEP_CODE_HELD  # 傾きがまだ無い
    pct, code, _ = _step(9.0, 8.0, dev_kmh=-1.0, slope_kmhs=0.0)  # 傾きが 0 以上
    assert (pct, code) == (9.0, mdmod.STEP_CODE_HELD)


def test_stopped_car_is_not_released() -> None:
    pct, code, wait = _step(9.0, 8.0, speed_kmh=0.0)
    assert (pct, code, wait) == (9.0, mdmod.STEP_CODE_HELD, 0)


def test_never_shallower_than_min() -> None:
    pct, code, _ = _step(4.2, 4.2, slope_kmhs=-8.0, dev_kmh=-1.0)
    assert code == mdmod.STEP_CODE_STEPPED and pct == pytest.approx(MIN_PCT)
    # すでに min なら戻せない（今の保持）
    pct, code, wait = _step(MIN_PCT, 3.0, slope_kmhs=-8.0, dev_kmh=-1.0)
    assert (pct, code, wait) == (MIN_PCT, mdmod.STEP_CODE_HELD, 0)


def test_outside_zone_first_or_no_prev_pass_through_and_drop_wait() -> None:
    assert _step(9.0, 5.0, in_zone=False, wait_left=3) == (5.0, mdmod.STEP_CODE_NONE, 0)
    assert _step(9.0, 5.0, first=True, wait_left=3) == (5.0, mdmod.STEP_CODE_NONE, 0)
    assert _step(None, 5.0) == (5.0, mdmod.STEP_CODE_NONE, 0)


def _cfg(tmp_path: Path):
    return cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))


def test_setup_off_is_none_and_on_reads_meta(tmp_path: Path) -> None:
    cfg = _cfg(tmp_path)
    table = SimpleNamespace(meta={"stop_release_gain_kmhs_pct": 4.4, "brake_lag_s": 0.155})
    cfg.forward_map.stop_approach_step_release = False
    assert mdmod.step_release_setup(cfg, table) is None
    cfg.forward_map.stop_approach_step_release = True
    cfg.forward_map.brake_lag_s = None
    gain, lag, wait = mdmod.step_release_setup(cfg, table)
    assert (gain, lag) == (4.4, 0.155)
    assert wait == round((0.155 + 0.1) / cfg.control.loop_interval_s)


def test_setup_without_gain_meta_raises(tmp_path: Path) -> None:
    cfg = _cfg(tmp_path)
    cfg.forward_map.stop_approach_step_release = True
    with pytest.raises(ConfigError, match="attach-release-gain"):
        mdmod.step_release_setup(cfg, SimpleNamespace(meta={"brake_lag_s": 0.155}))


def test_config_default_and_validate() -> None:
    assert cfgmod.ForwardMapSection().stop_approach_step_release is False
    cfg = cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))
    cfg.forward_map.stop_tail_ramp_enabled = True
    cfg.forward_map.launch_release_ramp = True
    cfg.forward_map.stop_approach_no_release = True
    cfg.forward_map.stop_approach_step_release = True
    assert not any("step_release" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.stop_approach_no_release = False
    assert any("stop_approach_step_release" in p for p in cfgmod.validate_config(cfg))


# ---- _ModeRun._step_release_cycle（周期ごとの通し。偏差の履歴・待ち・集計・区間の出入り） ----

def _run_step():
    from collections import deque

    run = mdmod._ModeRun.__new__(mdmod._ModeRun)  # noqa: SLF001
    run.cfg = SimpleNamespace(
        forward_map=SimpleNamespace(stop_approach_release_dev_kmh=0.3),
        vehicle=SimpleNamespace(min_brake_opening_pct=MIN_PCT),
    )
    run.interval_s = 0.05
    run._step_setup = (GAIN, LAG, WAIT)  # noqa: SLF001
    run._step_dev_hist = deque(maxlen=5)  # noqa: SLF001
    run._step_wait_left = 0  # noqa: SLF001
    run._step_episode_count = 0  # noqa: SLF001
    run.step_release_stepped_cycles = 0
    run.step_release_wait_cycles = 0
    run.step_release_again = 0
    run.step_release_amount_sum = 0.0
    run.step_release_amount_max = 0.0
    run.stop_keep_prev_pct = None
    return run


def _tick(run, dev: float, effort_in: float, *, zone: bool):
    """_cycle の stop_keep ブロックと同じ順: first を決め → メソッド → prev を更新。"""
    first = run.stop_keep_prev_pct is None
    effort, held, released, code = run._step_release_cycle(  # noqa: SLF001
        effort_in, zone=zone, first=first, speed=1.0, ref=1.0 - dev
    )
    assert released is False
    if zone and held and code == mdmod.STEP_CODE_HELD:
        effort = -run.stop_keep_prev_pct  # 保つ周期は _cycle の共通部分が前の指令にする
    run.stop_keep_prev_pct = -effort if zone else None
    return effort, held, code


def test_step_release_cycle_full_sequence() -> None:
    run = _run_step()
    dev = 0.3
    # 区間の外 3 周期（偏差は 0.1/周期で下がる ＝ 傾き −2.0）。何もしない・prev は None のまま
    for _ in range(3):
        eff, held, code = _tick(run, dev, -9.0, zone=False)
        assert (eff, held, code) == (-9.0, False, 0)
        assert run.stop_keep_prev_pct is None
        dev -= 0.1
    # 区間に入った最初の周期（first）: そのまま・prev ＝ 9.0
    eff, held, code = _tick(run, dev, -9.0, zone=True)
    assert (eff, code) == (-9.0, 0) and run.stop_keep_prev_pct == pytest.approx(9.0)
    dev -= 0.1
    # (a) 見込み ＝ dev ＋ (−2.0) × 0.15 < −0.3 になった周期で 3。量 ＝ 2.0 ÷ 4.0 ＝ 0.5%
    eff, held, code = _tick(run, dev, -9.0, zone=True)
    assert code == mdmod.STEP_CODE_STEPPED and not held
    assert eff == pytest.approx(-8.5)  # ブレーキは effort 負
    assert run.stop_keep_prev_pct == pytest.approx(8.5)
    assert run.step_release_stepped_cycles == 1
    assert run.step_release_amount_sum == pytest.approx(0.5)
    assert run.step_release_amount_max == pytest.approx(0.5)
    # (b) 続く 5 周期は 4。新しい指令が何であっても指令は変わらない
    for k in range(WAIT):
        dev -= 0.1
        eff, held, code = _tick(run, dev, -9.0 - 0.2 * k, zone=True)
        assert (code, held) == (mdmod.STEP_CODE_WAIT, True)
        assert eff == pytest.approx(-8.5) and run.stop_keep_prev_pct == pytest.approx(8.5)
    assert run.step_release_wait_cycles == WAIT
    # (c) 待ちの後も傾き < 0 で遅すぎなら再び 3。step_release_again が 1
    dev -= 0.1
    eff, held, code = _tick(run, dev, -9.0, zone=True)
    assert code == mdmod.STEP_CODE_STEPPED
    assert eff == pytest.approx(-8.0) and run.stop_keep_prev_pct == pytest.approx(8.0)
    assert run.step_release_again == 1
    assert run.step_release_stepped_cycles == 2
    assert run.step_release_amount_sum == pytest.approx(1.0)
    # (d) 偏差が平らになる（傾き ≥ 0）: 待ちの後は 1 ＝ 浅くしない（0 の周期は深くするとき）
    for _ in range(WAIT):
        eff, _, code = _tick(run, dev, -9.0, zone=True)
        assert code == mdmod.STEP_CODE_WAIT
    eff, held, code = _tick(run, dev, -7.0, zone=True)  # 新しい指令が浅い → 前のまま
    assert (code, held) == (mdmod.STEP_CODE_HELD, True)
    assert eff == pytest.approx(-8.0) and run.stop_keep_prev_pct == pytest.approx(8.0)
    eff, held, code = _tick(run, dev, -9.5, zone=True)  # 深くするのは良い
    assert (eff, held, code) == (-9.5, False, 0) and run.stop_keep_prev_pct == pytest.approx(9.5)
    # (e) 区間を出ると _step_episode_count が 0・prev は None
    assert run._step_episode_count == 2  # noqa: SLF001
    _tick(run, dev, -9.0, zone=False)
    assert run._step_episode_count == 0  # noqa: SLF001
    assert run.stop_keep_prev_pct is None
