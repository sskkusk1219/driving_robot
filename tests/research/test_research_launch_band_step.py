"""発進のブレーキの帯の FB（2026-11-06 段1 launch_band_step_fb）のテスト。
launch_band_step（純関数）・launch_band_step_setup（meta 欠落は ConfigError）・config の検証・
_ModeRun の帯の周期・ログ列。段8: 上乗せの形・延長なし。"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import launch_band_step as lbs
from tests.research import mode_drive as mdmod
from tests.research.config import ConfigError

GAIN = 4.0
LAG = 0.15
MIN_PCT = 4.2
CAP = 8.905
WAIT = 5
KW = dict(
    dev_kmh=0.0, slope_kmhs=0.0, lag_s=LAG, dev_band_kmh=0.3, gain_kmhs_pct=GAIN,
    cap_pct=CAP, min_pct=MIN_PCT, wait_left=0, wait_cycles=WAIT,
)


def _step(prev_off: float, ff: float, **over):
    return lbs.launch_band_step(prev_off, ff, **{**KW, **over})


def test_raise_when_forecast_fast_and_slope_positive() -> None:
    # 見込み 0.3 + 2.0 × 0.15 = 0.6 > 0.3、傾き正 → 上乗せ 1.0 → 1.5（2.0 ÷ 4.0 = 0.5）
    off, out, code, wait = _step(1.0, 6.0, dev_kmh=0.3, slope_kmhs=2.0)
    assert (off, out) == (pytest.approx(1.5), pytest.approx(7.5))
    assert (code, wait) == (lbs.LBS_CODE_RAISED, WAIT)


def test_raise_capped_rounds_offset_and_holds_at_cap() -> None:
    off, out, code, _ = _step(2.7, 6.0, dev_kmh=0.5, slope_kmhs=2.0)
    assert code == lbs.LBS_CODE_RAISED and out == pytest.approx(CAP)
    assert off == pytest.approx(CAP - 6.0)  # 上乗せも丸める
    # 頭打ちで増えない → 待たずに保つ（印 1）
    off, out, code, wait = _step(CAP - 6.0, 6.0, dev_kmh=0.5, slope_kmhs=2.0)
    assert (out, code, wait) == (pytest.approx(CAP), lbs.LBS_CODE_HELD, 0)
    assert off == pytest.approx(CAP - 6.0)


def test_hold_when_forecast_slightly_fast() -> None:
    off, out, code, wait = _step(1.0, 6.0, dev_kmh=0.1, slope_kmhs=0.4)  # 見込み 0.16
    assert (off, out, code, wait) == (1.0, 7.0, lbs.LBS_CODE_HELD, 0)
    # 見込みが大きくても傾きが 0 以下なら増やさない（保つ）
    assert _step(1.0, 6.0, dev_kmh=1.0, slope_kmhs=-0.1)[:3] == (1.0, 7.0, lbs.LBS_CODE_HELD)


def test_hold_with_zero_offset_is_code_none() -> None:
    assert _step(0.0, 6.0, dev_kmh=0.1, slope_kmhs=0.4) == (0.0, 6.0, lbs.LBS_CODE_NONE, 0)
    assert _step(0.0, 6.0, slope_kmhs=None) == (0.0, 6.0, lbs.LBS_CODE_NONE, 0)


def test_release_toward_zero_when_forecast_late_or_even() -> None:
    # 見込み −0.1 + (−1.0) × 0.15 = −0.25（> −0.3）→ 上乗せを 0 へ。量 1.0 ÷ 4.0 = 0.25
    off, out, code, wait = _step(1.0, 6.0, dev_kmh=-0.1, slope_kmhs=-1.0)
    assert (off, out) == (pytest.approx(0.75), pytest.approx(6.75))
    assert (code, wait) == (lbs.LBS_CODE_RELEASED_TO_FF, WAIT)
    # 0 で止まる（0 より下へは行かない）
    off, out, code, _ = _step(0.1, 6.0, dev_kmh=-0.1, slope_kmhs=-1.0)
    assert code == lbs.LBS_CODE_RELEASED_TO_FF and off == pytest.approx(0.0)
    assert out == pytest.approx(6.0)
    # 上乗せ 0 以下なら動かさない
    assert _step(0.0, 6.0, dev_kmh=-0.1, slope_kmhs=-1.0)[:3] == (0.0, 6.0, lbs.LBS_CODE_NONE)
    assert _step(-0.5, 6.0, dev_kmh=-0.1, slope_kmhs=-1.0)[:3] == (-0.5, 5.5, lbs.LBS_CODE_HELD)
    # 見込み 0 以下でも傾き 0 以上なら保つ
    assert _step(1.0, 6.0, dev_kmh=-0.1, slope_kmhs=0.0)[:3] == (1.0, 7.0, lbs.LBS_CODE_HELD)


def test_release_below_zero_when_forecast_much_slower_than_ref() -> None:
    # 見込み −0.2 + (−2.0) × 0.15 = −0.5 < −0.3 → 0 より下へ。量 0.5
    off, out, code, wait = _step(0.2, 6.0, dev_kmh=-0.2, slope_kmhs=-2.0)
    assert (off, out) == (pytest.approx(-0.3), pytest.approx(5.7))
    assert (code, wait) == (lbs.LBS_CODE_RELEASED_BELOW_FF, WAIT)
    # さらに 0 より下へ行ける
    off, _, code, _ = _step(-0.3, 6.0, dev_kmh=-0.2, slope_kmhs=-2.0)
    assert code == lbs.LBS_CODE_RELEASED_BELOW_FF and off == pytest.approx(-0.8)


def test_release_floor_min_rounds_offset() -> None:
    off, out, code, _ = _step(-1.5, 6.0, dev_kmh=-1.0, slope_kmhs=-8.0)
    assert code == lbs.LBS_CODE_RELEASED_BELOW_FF and out == pytest.approx(MIN_PCT)
    assert off == pytest.approx(MIN_PCT - 6.0)
    # すでに min なら保つ（印 1）
    off, out, code, wait = _step(MIN_PCT - 6.0, 6.0, dev_kmh=-1.0, slope_kmhs=-8.0)
    assert (out, code, wait) == (pytest.approx(MIN_PCT), lbs.LBS_CODE_HELD, 0)


def test_wait_keeps_offset_and_counts_down() -> None:
    off, out, code, wait = _step(1.0, 6.0, wait_left=WAIT, dev_kmh=1.0, slope_kmhs=2.0)
    assert (off, out, code, wait) == (1.0, 7.0, lbs.LBS_CODE_WAIT, WAIT - 1)
    assert _step(1.0, 5.0, wait_left=1)[2:] == (lbs.LBS_CODE_WAIT, 0)


def test_slope_none_holds() -> None:
    assert _step(1.0, 6.0, slope_kmhs=None, dev_kmh=2.0) == (1.0, 7.0, lbs.LBS_CODE_HELD, 0)


def test_ff_moves_output_moves_by_same_amount() -> None:
    # 保つ間、FF が動けば出す開度も同じだけ動く（FF の先読みが通る）
    _, out1, _, _ = _step(1.0, 5.0, dev_kmh=0.1, slope_kmhs=0.4)
    _, out2, _, _ = _step(1.0, 4.2, dev_kmh=0.1, slope_kmhs=0.4)
    assert out1 == pytest.approx(6.0) and out2 == pytest.approx(5.2)
    # 待ちの間も同じ
    assert _step(1.0, 4.2, wait_left=2)[1] == pytest.approx(5.2)


def test_ff_deeper_than_cap_is_output_as_is() -> None:
    off, out, code, wait = _step(0.0, 9.5)
    assert (off, out, code, wait) == (0.0, 9.5, lbs.LBS_CODE_NONE, 0)
    # 増やしたくても FF は削らず、上乗せも増やさない（保つ）
    off, out, code, wait = _step(0.0, 9.5, dev_kmh=0.5, slope_kmhs=2.0)
    assert (off, out, code, wait) == (0.0, 9.5, lbs.LBS_CODE_NONE, 0)
    # FF が下がって cap 以下に戻れば、丸めた分だけ効き始める
    assert _step(1.0, 8.0)[1] == pytest.approx(CAP)


def test_forecast_and_raise_wanted() -> None:
    assert lbs.launch_band_forecast(0.2, 2.0, 0.15) == pytest.approx(0.5)
    assert lbs.launch_band_forecast(0.2, None, 0.15) == 0.2
    assert lbs.launch_band_raise_wanted(dev_kmh=0.3, slope_kmhs=2.0, lag_s=LAG)
    assert not lbs.launch_band_raise_wanted(dev_kmh=0.3, slope_kmhs=-2.0, lag_s=LAG)
    assert not lbs.launch_band_raise_wanted(dev_kmh=0.3, slope_kmhs=None, lag_s=LAG)


def _cfg():
    return cfgmod.load_config(Path(cfgmod.__file__).with_name("config_testVehicle.yaml"))


def test_setup_off_none_on_reads_meta() -> None:
    cfg = _cfg()
    table = SimpleNamespace(meta={
        "stop_release_gain_kmhs_pct": 4.4, "brake_lag_s": 0.155, "launch_breakaway_u_pct": -4.705,
    })
    cfg.forward_map.launch_band_step_fb = False
    assert lbs.launch_band_step_setup(cfg, table) is None
    cfg.forward_map.launch_band_step_fb = True
    cfg.forward_map.brake_lag_s = None
    cfg.vehicle.min_brake_opening_pct = 4.2
    cfg.feedforward.stop_brake_offset_pct = 20.0
    gain, lag, wait, cap, mn = lbs.launch_band_step_setup(cfg, table)
    assert (gain, lag, mn) == (4.4, 0.155, 4.2)
    assert wait == round((0.155 + 0.1) / cfg.control.loop_interval_s)
    assert cap == pytest.approx(8.905)


@pytest.mark.parametrize("missing", ["stop_release_gain_kmhs_pct", "brake_lag_s",
                                     "launch_breakaway_u_pct"])
def test_setup_missing_meta_raises(missing: str) -> None:
    cfg = _cfg()
    cfg.forward_map.launch_band_step_fb = True
    cfg.forward_map.brake_lag_s = None
    meta = {"stop_release_gain_kmhs_pct": 4.4, "brake_lag_s": 0.155,
            "launch_breakaway_u_pct": -4.705}
    del meta[missing]
    with pytest.raises(ConfigError):
        lbs.launch_band_step_setup(cfg, SimpleNamespace(meta=meta))


def test_config_default_and_validate() -> None:
    assert cfgmod.ForwardMapSection().launch_band_step_fb is False
    cfg = _cfg()
    cfg.forward_map.launch_band_step_fb = True
    cfg.forward_map.launch_brake_band = False
    assert any("launch_band_step_fb" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.launch_brake_band = True
    cfg.forward_map.hold_pi_enabled = False
    assert any("launch_band_step_fb" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.hold_pi_enabled = True
    assert not any("launch_band_step_fb" in p for p in cfgmod.validate_config(cfg))


# ---- _ModeRun の帯の周期 ----

def _run():
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)  # noqa: SLF001
    run.interval_s = 0.05
    run._lbs_setup = (GAIN, LAG, WAIT, CAP, MIN_PCT)  # noqa: SLF001
    run._lbs_dev_hist = deque(maxlen=5)  # noqa: SLF001
    run._lbs_prev_offset = None  # noqa: SLF001
    run._lbs_wait_left = 0  # noqa: SLF001
    run.lbs_code_cycles = {}
    run.lbs_raise_count = 0
    run.lbs_raise_max = 0.0
    run.lbs_cap_hits = 0
    run.lbs_offset_max = 0.0
    run.lbs_offset_min = 0.0
    return run


def test_band_first_cycle_offset_zero_code_zero_then_raises() -> None:
    run = _run()
    for k in range(5):  # 偏差が 0.1/周期で増える ＝ 傾き 2.0
        dev, slope = run._lbs_observe(0.3 + 0.1 * k, 0.0)  # noqa: SLF001
    assert slope == pytest.approx(2.0)
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert (pct, code) == (6.0, 0)  # 帯の最初の周期（上乗せ 0 ＝ FF）
    assert run._lbs_prev_offset == 0.0  # noqa: SLF001
    dev, slope = run._lbs_observe(0.8, 0.0)  # noqa: SLF001
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert code == lbs.LBS_CODE_RAISED and pct > 6.0
    assert run.lbs_raise_count == 1 and run.lbs_raise_max == pytest.approx(pct - 6.0)
    assert run.lbs_offset_max == pytest.approx(pct - 6.0)
    # 待ち中は FF が動けば出す開度も動く
    dev, slope = run._lbs_observe(0.9, 0.0)  # noqa: SLF001
    pct2, code2 = run._lbs_band_cycle(5.0, dev, slope)  # noqa: SLF001
    assert code2 == lbs.LBS_CODE_WAIT and pct2 == pytest.approx(pct - 1.0)


def test_cap_hits_counted_when_wanted_but_at_cap() -> None:
    run = _run()
    run._lbs_prev_offset = CAP - 6.0  # noqa: SLF001
    for k in range(5):
        dev, slope = run._lbs_observe(0.3 + 0.1 * k, 0.0)  # noqa: SLF001
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert code == lbs.LBS_CODE_HELD and pct == pytest.approx(CAP)
    assert run.lbs_cap_hits == 1 and run.lbs_raise_count == 0


def test_offset_min_recorded_below_zero() -> None:
    run = _run()
    run._lbs_prev_offset = 0.0  # noqa: SLF001
    for k in range(5):  # 偏差が −0.1/周期 ＝ 傾き −2.0、偏差 −0.4 → 見込み −0.7
        dev, slope = run._lbs_observe(-0.1 * k, 0.0)  # noqa: SLF001
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert code == lbs.LBS_CODE_RELEASED_BELOW_FF and pct < 6.0
    assert run.lbs_offset_min == pytest.approx(pct - 6.0) and run.lbs_offset_min < 0.0


def test_no_extension_code_or_hooks_remain() -> None:
    # 帯の終わりの延長は無い（印 6・延長の関数・状態を持たない）
    assert not hasattr(lbs, "LBS_CODE_END_WAIT")
    assert not hasattr(mdmod._ModeRun, "_lbs_extension")  # noqa: SLF001
    assert "_lbs_ext_armed" not in mdmod._ModeRun.__init__.__code__.co_names  # noqa: SLF001


def _cycle_src() -> str:
    import inspect

    return inspect.getsource(mdmod._ModeRun._cycle)  # noqa: SLF001


def test_cycle_drops_offset_outside_band_and_has_no_extension() -> None:
    src = _cycle_src()
    # 帯でない周期（hold_scene != 3）では上乗せを捨て、延長の分岐は無い
    assert "self._lbs_prev_offset = None" in src
    assert "lbs_ext" not in src and "_lbs_extension" not in src
    # 帯の外での捨て方そのもの（待ちも 0 に戻る）
    run = _run()
    run._lbs_prev_offset = 1.0  # noqa: SLF001
    run._lbs_wait_left = 3  # noqa: SLF001
    # 帯の最初の周期と同じ入口: None なら 0 から始まり印 0
    run._lbs_prev_offset = None  # noqa: SLF001
    dev, slope = run._lbs_observe(0.0, 0.0)  # noqa: SLF001
    assert run._lbs_band_cycle(6.0, dev, slope) == (6.0, 0)  # noqa: SLF001
    assert run._lbs_wait_left == 0  # noqa: SLF001


def test_off_has_no_state_effect() -> None:
    # off（setup None）なら _lbs_observe は呼ばれず、印は None（ログ空欄）
    src = _cycle_src()
    assert "None if self._lbs_setup is None else LBS_CODE_NONE" in src


def test_launch_step_code_column() -> None:
    assert "launch_step_code" in dlmod.CSV_COLUMNS
    assert dlmod.DriveSample.__dataclass_fields__["launch_step_code"].default is None
    # off は空欄・on は印の数字（行の組み立ては record の引数がそのまま列になる）
    import inspect

    assert "launch_step_code" in inspect.signature(dlmod.SessionLog.record).parameters
