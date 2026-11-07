"""発進のブレーキの帯の FB（2026-11-06 段1 launch_band_step_fb）のテスト。
launch_band_step（純関数）・launch_band_step_setup（meta 欠落は ConfigError）・config の検証・
_ModeRun の帯の周期・ログ列。段8: 上乗せの形・延長なし。"""

from __future__ import annotations

import inspect
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


def _step(prev_off: float, ff: float, *, prev_ff: float | None = None, **over):
    """上乗せ prev_off（前の周期の FF は prev_ff。既定は今の FF）から呼ぶ。"""
    pf = ff if prev_ff is None else prev_ff
    return lbs.launch_band_step(pf + prev_off, pf, ff, **{**KW, **over})


def _open(prev_out: float, prev_ff: float, ff: float, **over):
    """前の周期の出す開度・FF を直接渡す。"""
    return lbs.launch_band_step(prev_out, prev_ff, ff, **{**KW, **over})


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
    # p ＝ dev。p > band なら開度を保つ・それ以外は上乗せを保つ（増減しない）
    assert _step(1.0, 6.0, slope_kmhs=None, dev_kmh=2.0) == (1.0, 7.0, lbs.LBS_CODE_HELD, 0)
    assert _step(1.0, 6.0, slope_kmhs=None, dev_kmh=0.1) == (1.0, 7.0, lbs.LBS_CODE_HELD, 0)
    assert _open(7.0, 6.0, 5.0, slope_kmhs=None, dev_kmh=2.0)[:2] == (2.0, 7.0)


def test_ff_moves_output_moves_by_same_amount() -> None:
    # p ≤ +band で保つ間、FF が動けば出す開度も同じだけ動く（FF の先読みが通る）
    _, out1, _, _ = _open(6.0, 5.0, 5.0, dev_kmh=0.1, slope_kmhs=0.4)
    _, out2, _, _ = _open(6.0, 5.0, 4.2, dev_kmh=0.1, slope_kmhs=0.4)
    assert out1 == pytest.approx(6.0) and out2 == pytest.approx(5.2)
    # 待ちの間も同じ
    assert _open(6.0, 5.0, 4.2, wait_left=2)[1] == pytest.approx(5.2)


def test_over_band_keeps_opening_when_ff_drops() -> None:
    # p = 0.5 + 0 > 0.3 → FF が下がっても出す開度は下がらない（上乗せが増える）
    off, out, code, wait = _open(7.0, 6.0, 5.0, dev_kmh=0.5, slope_kmhs=0.0)
    assert (off, out, code, wait) == (pytest.approx(2.0), pytest.approx(7.0),
                                      lbs.LBS_CODE_HELD, 0)
    # 傾きが負でも p > band なら同じ
    assert _open(7.0, 6.0, 5.0, dev_kmh=1.0, slope_kmhs=-0.5)[1] == pytest.approx(7.0)


def test_over_band_ff_deeper_is_ff() -> None:
    off, out, code, _ = _open(7.0, 6.0, 7.5, dev_kmh=0.5, slope_kmhs=0.0)
    assert (off, out, code) == (0.0, pytest.approx(7.5), lbs.LBS_CODE_HELD)


def test_return_to_band_no_step() -> None:
    # p が +0.3 以下に戻った周期は FF ＋（前の出す開度 − 前の FF）＝ 段差なし
    off, out, code, _ = _open(7.0, 6.0, 5.0, dev_kmh=0.1, slope_kmhs=0.0)
    assert out == pytest.approx(5.0 + (7.0 - 6.0))
    assert (off, code) == (pytest.approx(1.0), lbs.LBS_CODE_HELD)


def test_wait_over_band_keeps_opening_else_offset() -> None:
    # 待ちの間も p > +0.3 なら開度を保つ
    off, out, code, wait = _open(7.0, 6.0, 5.0, wait_left=3, dev_kmh=0.5, slope_kmhs=0.0)
    assert (out, code, wait) == (pytest.approx(7.0), lbs.LBS_CODE_WAIT, 2)
    # p ≤ +0.3 なら上乗せを保つ
    off, out, code, wait = _open(7.0, 6.0, 5.0, wait_left=3, dev_kmh=0.1, slope_kmhs=0.0)
    assert (off, out, code, wait) == (pytest.approx(1.0), pytest.approx(6.0),
                                      lbs.LBS_CODE_WAIT, 2)


def test_over_band_raise_from_open_and_cap() -> None:
    # 開度 7.0 から slope 2.0 ÷ 4.0 = 0.5 増やす
    off, out, code, wait = _open(7.0, 6.0, 5.0, dev_kmh=0.5, slope_kmhs=2.0)
    assert (out, code, wait) == (pytest.approx(7.5), lbs.LBS_CODE_RAISED, WAIT)
    assert off == pytest.approx(2.5)
    # 上限で頭打ち、すでに上限なら保つ（印 1・待ちなし）
    assert _open(8.8, 6.0, 6.0, dev_kmh=0.5, slope_kmhs=2.0)[1] == pytest.approx(CAP)
    off, out, code, wait = _open(CAP, 6.0, 6.0, dev_kmh=0.5, slope_kmhs=2.0)
    assert (out, code, wait) == (pytest.approx(CAP), lbs.LBS_CODE_HELD, 0)


def test_over_band_ff_over_cap_is_ff_code_none() -> None:
    assert _open(8.0, 7.0, 9.5, dev_kmh=0.5, slope_kmhs=2.0) == (0.0, 9.5, lbs.LBS_CODE_NONE, 0)


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
    run._lbs_prev_out = None  # noqa: SLF001
    run._lbs_prev_ff = None  # noqa: SLF001
    run._lbs_wait_left = 0  # noqa: SLF001
    run.lbs_code_cycles = {}
    run.lbs_raise_count = 0
    run.lbs_raise_max = 0.0
    run.lbs_cap_hits = 0
    run.lbs_hold_open_cycles = 0
    run.lbs_offset_max = 0.0
    run.lbs_offset_min = 0.0
    run._obs = None  # noqa: SLF001  # 2026-11-07 段16 型 A（off）
    run._ret_obs = None  # noqa: SLF001  # 2026-11-08 段20 B-1（off）
    run._b1_rows = []  # noqa: SLF001
    run._b1_pending_end = None  # noqa: SLF001
    run.b1_launches = 0
    run.b1_extended = []
    return run


def test_band_first_cycle_offset_zero_code_zero_then_raises() -> None:
    run = _run()
    for k in range(5):  # 偏差が 0.1/周期で増える ＝ 傾き 2.0
        dev, slope = run._lbs_observe(0.3 + 0.1 * k, 0.0)  # noqa: SLF001
    assert slope == pytest.approx(2.0)
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert (pct, code) == (6.0, 0)  # 帯の最初の周期（上乗せ 0 ＝ FF）
    assert (run._lbs_prev_out, run._lbs_prev_ff) == (6.0, 6.0)  # noqa: SLF001
    dev, slope = run._lbs_observe(0.8, 0.0)  # noqa: SLF001
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert code == lbs.LBS_CODE_RAISED and pct > 6.0
    assert run.lbs_raise_count == 1 and run.lbs_raise_max == pytest.approx(pct - 6.0)
    assert run.lbs_offset_max == pytest.approx(pct - 6.0)
    # 待ち中も見込み > +0.3 なら FF が下がっても出す開度は下がらない
    dev, slope = run._lbs_observe(0.9, 0.0)  # noqa: SLF001
    pct2, code2 = run._lbs_band_cycle(5.0, dev, slope)  # noqa: SLF001
    assert code2 == lbs.LBS_CODE_WAIT and pct2 == pytest.approx(pct)


def test_cap_hits_counted_when_wanted_but_at_cap() -> None:
    run = _run()
    run._lbs_prev_out, run._lbs_prev_ff = CAP, 6.0  # noqa: SLF001
    for k in range(5):
        dev, slope = run._lbs_observe(0.3 + 0.1 * k, 0.0)  # noqa: SLF001
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert code == lbs.LBS_CODE_HELD and pct == pytest.approx(CAP)
    assert run.lbs_cap_hits == 1 and run.lbs_raise_count == 0


def test_offset_min_recorded_below_zero() -> None:
    run = _run()
    run._lbs_prev_out, run._lbs_prev_ff = 6.0, 6.0  # noqa: SLF001
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
    assert "self._lbs_prev_out = None" in src and "self._lbs_prev_ff = None" in src
    assert "lbs_ext" not in src and "_lbs_extension" not in src
    # 帯の外での捨て方そのもの（待ちも 0 に戻る）
    run = _run()
    run._lbs_prev_out, run._lbs_prev_ff = 7.0, 6.0  # noqa: SLF001
    run._lbs_wait_left = 3  # noqa: SLF001
    # 帯の最初の周期と同じ入口: None なら 0 から始まり印 0
    run._lbs_prev_out = run._lbs_prev_ff = None  # noqa: SLF001
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


def test_hold_open_cycles_counted() -> None:
    run = _run()
    run._lbs_prev_out, run._lbs_prev_ff = 7.0, 6.0  # noqa: SLF001
    for _ in range(5):  # 偏差 0.5 一定 ＝ 傾き 0、見込み 0.5 > 0.3
        dev, slope = run._lbs_observe(0.5, 0.0)  # noqa: SLF001
    pct, code = run._lbs_band_cycle(5.0, dev, slope)  # noqa: SLF001
    assert code == lbs.LBS_CODE_HELD and pct == pytest.approx(7.0)
    assert run.lbs_hold_open_cycles == 1
    dev, slope = run._lbs_observe(0.0, 0.0)  # noqa: SLF001
    run._lbs_band_cycle(5.0, dev, slope)  # noqa: SLF001
    assert run.lbs_hold_open_cycles == 1  # p ≤ band の周期は数えない


# ───────── 2026-11-07 段16 型 A: その場で測る動き出す開度 ─────────

STOP = 0.02


def _obs_run(rows, obs=None):
    """rows ＝ (active, ref, speed, 前の周期の実ブレーキ開度) の列。各周期の u_obs を返す。"""
    obs = obs or lbs.LaunchBreakawayObserver()
    out = []
    for active, ref, speed, prev in rows:
        out.append(obs.update(
            active=active, ref_kmh=ref, speed_kmh=speed, prev_brake_actual_pct=prev,
            stop_speed_kmh=STOP,
        ))
    return obs, out


def test_observer_measures_only_first_cycle_over_move_threshold() -> None:
    rows = [
        (True, 0.0, 0.000, 9.5),
        (True, 0.0, 0.010, 9.4),  # ちょうど 0.01 は動いていない（> のみ）
        (True, 0.0, 0.011, 9.1),  # 最初に超えた周期: 前の周期の実開度 9.1
        (True, 0.0, 0.200, 8.0),  # 以降は測り直さない
    ]
    obs, out = _obs_run(rows)
    assert out == [None, None, 9.1, 9.1]
    assert obs.measured == [9.1]


def test_observer_does_not_measure_after_reference_started_moving() -> None:
    rows = [
        (True, 0.0, 0.0, 9.0),
        (True, 0.5, 0.0, 9.0),  # 基準が停車速度を超えた: もう測らない
        (True, 0.6, 0.3, 8.9),  # 車が後から動いても None のまま
    ]
    obs, out = _obs_run(rows)
    assert out == [None, None, None] and obs.measured == []
    assert "測れた発進 0 / 1" in obs.summary()


def test_observer_remeasures_for_each_new_launch() -> None:
    rows = [
        (True, 0.0, 0.05, 9.2),  # 1 本目: 9.2（同じ周期で動いている）
        (False, 0.0, 0.0, 9.2),  # 並びを出ると捨てる
        (True, 0.0, 0.0, 9.0),
        (True, 0.0, 0.02, 8.8),  # 2 本目: 8.8
    ]
    obs, out = _obs_run(rows)
    assert out == [9.2, None, None, 8.8]
    assert obs.measured == [9.2, 8.8] and obs.n_sequences == 2
    assert "8.80〜9.20%" in obs.summary()


def test_observer_none_when_prev_actual_missing() -> None:
    obs, out = _obs_run([(True, 0.0, 0.5, None), (True, 0.0, 0.5, 9.0)])
    assert out == [None, 9.0]  # 前の周期の実開度が無い周期は測れず、次の周期で測る


def test_launch_band_seq_rows_only_runs_with_band_rows() -> None:
    import numpy as np  # noqa: PLC0415

    prep = np.array([0, 1, 1, 1, 0, 1, 1, 0, 0], dtype=bool)
    band = np.array([0, 0, 0, 1, 0, 0, 0, 0, 0], dtype=bool)
    got = lbs.launch_band_seq_rows(prep, band)
    assert got.tolist() == [False, True, True, True, False, False, False, False, False]
    assert lbs.launch_band_seq_rows(None, band) is None
    assert lbs.launch_band_seq_rows(prep, None) is None


def test_obs_inputs_without_u_obs_is_unchanged() -> None:
    assert lbs.launch_band_obs_inputs(None, cap_pct=CAP) == CAP


def test_obs_inputs_with_u_obs_only_caps() -> None:
    # 2026-11-08 段20: u_obs があれば上限 ＝ u_obs だけ（pkl の値との深い方にはしない）
    assert lbs.launch_band_obs_inputs(9.1, cap_pct=CAP) == 9.1
    assert lbs.launch_band_obs_inputs(8.5, cap_pct=CAP) == 8.5
    import inspect  # noqa: PLC0415

    # 前の出す開度・前の FF は入力に無い（触れない）
    assert set(inspect.signature(lbs.launch_band_obs_inputs).parameters) == {"u_obs_pct", "cap_pct"}


def test_obs_does_not_lift_prev_out_or_override_ff_in_step() -> None:
    # 帯の最初の周期でも持ち上げない: 前の出す開度が無ければ出す開度 ＝ FF（深くしない）
    run = _run()
    run._obs = lbs.LaunchBreakawayObserver()  # noqa: SLF001
    run._obs.update(  # noqa: SLF001
        active=True, ref_kmh=0.0, speed_kmh=0.5, prev_brake_actual_pct=9.1, stop_speed_kmh=STOP
    )
    pct, code = run._lbs_band_cycle(7.5, 0.5, 0.0)  # noqa: SLF001
    assert pct == pytest.approx(7.5) and code == lbs.LBS_CODE_NONE


def test_stretch_launch_bands_extends_matching_interval_by_delta() -> None:
    from tests.research.hold_scene import HoldInterval  # noqa: PLC0415

    dt = 0.05
    bands = [HoldInterval(3.0, 0.60, 1.0), HoldInterval(20.0, 0.50, 1.0)]
    new, ext = lbs.stretch_launch_bands(bands, [(72, 3), (400, 0)], dt)  # a ＝ 72 → 3.6 s
    assert new[0].start_s == 3.0 and new[0].length_s == pytest.approx(0.75)
    assert new[1] == bands[1]  # δ ＝ 0 は触らない
    assert ext == [(pytest.approx(3.55), pytest.approx(3.70))]
    # 合う区間が無い（遠い）ものは何もしない
    assert lbs.stretch_launch_bands(bands, [(200, 4)], dt) == (bands, [])


def test_band_ff_time_replaces_only_extended_rows() -> None:
    ext = [(3.55, 3.70)]
    dt = 0.05
    assert lbs.band_ff_time(3.50, ext, dt) == 3.50
    assert lbs.band_ff_time(3.55, ext, dt) == 3.55  # 元の最後の周期
    assert lbs.band_ff_time(3.60, ext, dt) == 3.55
    assert lbs.band_ff_time(3.70, ext, dt) == 3.55
    assert lbs.band_ff_time(3.75, ext, dt) == 3.75
    assert lbs.band_ff_time(5.0, [], dt) == 5.0


def test_mode_run_band_cycle_with_u_obs_caps_raise_at_u_obs() -> None:
    run = _run()
    run._obs = lbs.LaunchBreakawayObserver()  # noqa: SLF001
    run._obs.update(  # noqa: SLF001
        active=True, ref_kmh=0.0, speed_kmh=0.5, prev_brake_actual_pct=8.0, stop_speed_kmh=STOP
    )
    assert run._obs.u_obs == 8.0  # noqa: SLF001
    run._lbs_band_cycle(7.5, 0.5, 0.0)  # noqa: SLF001
    # 次の周期の上限は u_obs（pkl の 8.905 ではない）。上げたい状態でも 8.0 を超えない
    pct2, _ = run._lbs_band_cycle(7.5, 0.5, 2.0)  # noqa: SLF001
    assert pct2 <= 8.0 + 1e-9


def test_mode_run_band_cycle_without_obs_keeps_first_cycle_behavior() -> None:
    run = _run()
    pct, code = run._lbs_band_cycle(7.5, 0.5, 0.0)  # noqa: SLF001
    assert pct == 7.5 and code == lbs.LBS_CODE_NONE  # 上乗せ 0・出す開度 ＝ FF（今と同じ）


# ───────── 2026-11-08 段20 B-1: 帯の終わりの離しを空きの分だけ遅くする ─────────

TOL = 0.05


def _ret(rows, obs=None):
    """rows ＝ (in_band, その周期の指令直後のブレーキ実開度) の列。"""
    obs = obs or lbs.BandReturnObserver(TOL, MIN_PCT)
    for in_band, act in rows:
        obs.update(in_band=in_band, brake_actual_pct=act)
    return obs


def test_return_observer_decides_r_at_first_cycle_within_min_tol() -> None:
    # 11-07 の実機: 帯の終わり 0 番の指令 → 3 番の直後に ≦ 4.25 で R ＝ 3
    obs = _ret([(True, 8.9), (True, 8.9), (False, 7.0), (False, 5.5), (False, 4.6),
                (False, 4.25), (False, 4.2)])
    assert obs.measured == [3] and obs.r_max == 3
    # 0 番ですでに着いていれば R ＝ 0
    assert _ret([(True, 8.9), (False, 4.2)]).measured == [0]


def test_return_observer_measures_per_launch_and_keeps_max() -> None:
    rows = [(True, 8.9), (False, 6.0), (False, 4.2),  # R ＝ 1
            (False, 4.2), (True, 8.9), (True, 8.9), (False, 7.0), (False, 5.0),
            (False, 4.5), (False, 4.2)]  # R ＝ 3
    obs = _ret(rows)
    assert obs.measured == [1, 3] and obs.r_max == 3
    assert lbs.BandReturnObserver(TOL, MIN_PCT).r_max is None


def test_return_observer_rejoining_band_discards_and_gives_up_after_max_wait() -> None:
    obs = _ret([(True, 8.9), (False, 7.0), (True, 8.9), (False, 4.2)])
    assert obs.measured == [0]  # 途中で帯に入り直したら測りかけを捨て、新しい帯の終わりから
    obs = lbs.BandReturnObserver(TOL, MIN_PCT, max_wait=3)
    _ret([(True, 8.9)] + [(False, 9.0)] * 6 + [(False, 4.2)], obs)
    assert obs.measured == []  # 戻らないまま max_wait を超えたら測れなかった扱い
    # 実開度が読めない周期は戻ったとみなさない
    assert _ret([(True, 8.9), (False, None), (False, 4.2)]).measured == [1]


def test_late_release_rows() -> None:
    assert lbs.late_release_rows(5, 3) == 1
    assert lbs.late_release_rows(4, 3) == 0
    assert lbs.late_release_rows(5, None) == 0
    assert lbs.late_release_rows(0, 3) == 0
    assert lbs.late_release_rows(2, 5) == 0  # 負にならない
    assert lbs.late_release_rows(8, 3) == 4


def test_plan_gap_rows_counts_rows_from_band_end_to_first_accel() -> None:
    import numpy as np  # noqa: PLC0415

    # 帯の最後の行 2。0 番 ＝ 行 3。アクセルの最初の行 8 → G ＝ 5
    eff = np.array([0, -8, -8, 0, 0, 0, 0, 0, 3.0, 4.0])
    assert lbs.plan_gap_rows(eff, 2) == 5
    # すぐアクセル → 0・間にブレーキの行がある → 0・アクセルが無い → 0
    assert lbs.plan_gap_rows(np.array([-8, 2.0]), 0) == 0
    assert lbs.plan_gap_rows(np.array([-8, 0, -3.0, 2.0]), 0) == 0
    assert lbs.plan_gap_rows(np.array([-8, 0, 0, 0.0]), 0) == 0


def test_extend_launch_band_and_ff_time_for_b1() -> None:
    from tests.research.hold_scene import HoldInterval  # noqa: PLC0415

    dt = 0.05
    bands = [HoldInterval(3.0, 0.60, 1.0)]  # 最後の周期 3.55
    new, k = lbs.extend_launch_band(bands, 3.55, 1, dt)
    assert k == 0 and new[0].length_s == pytest.approx(0.65) and bands[0].length_s == 0.60
    assert lbs.extend_launch_band(bands, 3.55, 0, dt) == (bands, None)
    assert lbs.extend_launch_band(bands, 9.0, 1, dt) == (bands, None)
    ext = lbs.extend_band_ext([], 3.55, 1, dt)
    assert ext == [(3.55, pytest.approx(3.60))]
    assert lbs.band_ff_time(3.60, ext, dt) == 3.55  # 伸ばした周期の FF は元の最後の周期の値
    assert lbs.band_ff_time(3.65, ext, dt) == 3.65
    # B-2 で伸ばした区間の最後の周期から更に伸ばすと、その項がつながる
    ext2 = lbs.extend_band_ext([(3.55, 3.70)], 3.70, 2, dt)
    assert ext2 == [(3.55, pytest.approx(3.80))]
    assert lbs.band_ff_time(3.80, ext2, dt) == 3.55


def _b1_run(r_max_rows, gap):
    """B-1 の判定用の _ModeRun 風。帯 3.0〜3.55・計画は帯の後 gap 行が 0・その次がアクセル。"""
    import numpy as np  # noqa: PLC0415

    from tests.research.hold_scene import HoldInterval  # noqa: PLC0415

    run = _run()
    dt = 0.05
    n = 100
    eff = np.zeros(n)
    eff[60:72] = -8.0  # 行 60〜71 ＝ 帯（t 3.0〜3.55）
    eff[72 + gap] = 3.0
    run.interval_s = dt
    run.hold = SimpleNamespace(launch_bands=[HoldInterval(3.0, 0.60, 1.0)], step_s=dt)
    run.ff = SimpleNamespace(plan=SimpleNamespace(
        effort_pct=eff, index_at=lambda t: int(round(t / dt)),
        launch_band=eff < 0.0,
    ))
    run._band_ext = []  # noqa: SLF001
    run._ret_obs = lbs.BandReturnObserver(TOL, MIN_PCT)  # noqa: SLF001
    run._ret_obs.measured = list(r_max_rows)  # noqa: SLF001
    run._plan_band_row = lambda t: True  # noqa: SLF001
    return run


def test_b1_extends_running_band_by_gap_minus_r_plus_one() -> None:
    run = _b1_run([3], gap=5)
    run._b1_extend_if_last(3.50)  # noqa: SLF001  最後の周期ではない
    assert run.b1_launches == 0 and run.b1_extended == []
    run._b1_extend_if_last(3.55)  # noqa: SLF001
    assert run.b1_launches == 1 and run.b1_extended == [1]
    assert run.hold.launch_bands[0].length_s == pytest.approx(0.65)
    assert run._band_ext == [(3.55, pytest.approx(3.60))]  # noqa: SLF001
    assert run._b1_rows_at(3.60) == 1 and run._b1_rows_at(3.55) is None  # noqa: SLF001
    # 伸ばした後の最後の周期では再判定しない（数えない・もう伸ばさない）
    run._b1_extend_if_last(3.60)  # noqa: SLF001
    assert run.b1_launches == 1 and run.b1_extended == [1]


def test_b1_no_extension_without_r_or_without_slack() -> None:
    for r, gap in (([], 5), ([3], 4), ([3], 3)):
        run = _b1_run(r, gap)
        run._b1_extend_if_last(3.55)  # noqa: SLF001
        assert run.b1_launches == 1 and run.b1_extended == []
        assert run.hold.launch_bands[0].length_s == pytest.approx(0.60)
        assert run._band_ext == []  # noqa: SLF001


def test_plan_band_row_counts_b1_extended_rows() -> None:
    run = _b1_run([3], gap=5)
    del run._plan_band_row  # noqa: SLF001  インスタンスの差し替えを外してクラスのものを使う
    run.ff = mdmod.ForwardMapFeedforward.__new__(mdmod.ForwardMapFeedforward)
    run.ff._plan = SimpleNamespace(  # noqa: SLF001
        launch_band=[False] * 100, index_at=lambda t: int(round(t / 0.05))
    )
    assert run._plan_band_row(3.60) is False  # noqa: SLF001
    run._b1_rows = [(3.60, 3.60, 1)]  # noqa: SLF001
    assert run._plan_band_row(3.60) is True  # noqa: SLF001
    assert run._plan_band_row(3.65) is False  # noqa: SLF001


def test_b1_log_column_exists_and_defaults_none() -> None:
    assert "band_late_rows" in dlmod.CSV_COLUMNS
    assert dlmod.DriveSample.__dataclass_fields__["band_late_rows"].default is None
    assert "band_late_rows" in inspect.signature(dlmod.SessionLog.record).parameters
