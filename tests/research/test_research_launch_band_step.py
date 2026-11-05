"""発進のブレーキの帯の FB（2026-11-06 段1 launch_band_step_fb）のテスト。
launch_band_step（純関数）・launch_band_step_setup（meta 欠落は ConfigError）・config の検証・
_ModeRun の帯の周期と帯の終わりの延長・ログ列。"""

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


def _step(prev: float, ff: float, **over):
    return lbs.launch_band_step(prev, ff, **{**KW, **over})


def test_raise_when_forecast_fast_and_slope_positive() -> None:
    # 見込み 0.3 + 2.0 × 0.15 = 0.6 > 0.3、傾き正 → 2.0 ÷ 4.0 = 0.5% 踏み増し
    pct, code, wait = _step(7.0, 6.0, dev_kmh=0.3, slope_kmhs=2.0)
    assert (pct, code, wait) == (pytest.approx(7.5), lbs.LBS_CODE_RAISED, WAIT)


def test_raise_capped_and_holds_when_at_cap() -> None:
    pct, code, _ = _step(8.8, 6.0, dev_kmh=0.5, slope_kmhs=2.0)
    assert code == lbs.LBS_CODE_RAISED and pct == pytest.approx(CAP)
    pct, code, wait = _step(CAP, 6.0, dev_kmh=0.5, slope_kmhs=2.0)
    assert (pct, code, wait) == (CAP, lbs.LBS_CODE_HELD, 0)


def test_hold_when_forecast_slightly_fast() -> None:
    pct, code, wait = _step(7.0, 6.0, dev_kmh=0.1, slope_kmhs=0.4)  # 見込み 0.16
    assert (pct, code, wait) == (7.0, lbs.LBS_CODE_HELD, 0)
    # 見込みが大きくても傾きが 0 以下なら踏み増さない（保つ）
    assert _step(7.0, 6.0, dev_kmh=1.0, slope_kmhs=-0.1)[:2] == (7.0, lbs.LBS_CODE_HELD)


def test_release_toward_ff_when_forecast_late_or_even() -> None:
    # 見込み −0.1 + (−1.0) × 0.15 = −0.25（> −0.3）→ FF へ。量 1.0 ÷ 4.0 = 0.25
    pct, code, wait = _step(7.0, 6.0, dev_kmh=-0.1, slope_kmhs=-1.0)
    assert (pct, code, wait) == (pytest.approx(6.75), lbs.LBS_CODE_RELEASED_TO_FF, WAIT)
    # FF より浅くしない
    pct, code, _ = _step(6.1, 6.0, dev_kmh=-0.1, slope_kmhs=-1.0)
    assert code == lbs.LBS_CODE_RELEASED_TO_FF and pct == pytest.approx(6.0)
    # 見込み 0 以下でも傾き 0 以上なら保つ
    assert _step(7.0, 6.0, dev_kmh=-0.1, slope_kmhs=0.0)[:2] == (7.0, lbs.LBS_CODE_HELD)


def test_release_below_ff_when_forecast_much_slower_than_ref() -> None:
    # 見込み −0.2 + (−2.0) × 0.15 = −0.5 < −0.3 → FF より浅くてよい。量 0.5
    pct, code, wait = _step(6.2, 6.0, dev_kmh=-0.2, slope_kmhs=-2.0)
    assert (pct, code, wait) == (pytest.approx(5.7), lbs.LBS_CODE_RELEASED_BELOW_FF, WAIT)
    # min より浅くしない
    pct, code, _ = _step(4.4, 6.0, dev_kmh=-1.0, slope_kmhs=-8.0)
    assert code == lbs.LBS_CODE_RELEASED_BELOW_FF and pct == pytest.approx(MIN_PCT)
    # すでに min なら保つ
    assert _step(MIN_PCT, 3.0, dev_kmh=-1.0, slope_kmhs=-8.0)[:2] == (
        MIN_PCT, lbs.LBS_CODE_HELD,
    )


def test_wait_keeps_prev_and_counts_down() -> None:
    assert _step(7.0, 9.0, wait_left=WAIT, dev_kmh=1.0, slope_kmhs=2.0) == (
        7.0, lbs.LBS_CODE_WAIT, WAIT - 1,
    )
    assert _step(7.0, 5.0, wait_left=1) == (7.0, lbs.LBS_CODE_WAIT, 0)


def test_slope_none_holds() -> None:
    assert _step(7.0, 6.0, slope_kmhs=None, dev_kmh=2.0) == (7.0, lbs.LBS_CODE_HELD, 0)


def test_ff_deeper_than_prev_follows_ff() -> None:
    assert _step(6.0, 7.5) == (7.5, lbs.LBS_CODE_NONE, 0)
    assert _step(6.0, 7.5, slope_kmhs=None) == (7.5, lbs.LBS_CODE_NONE, 0)
    # 踏み増しでも FF の方が深ければ FF
    pct, code, _ = _step(6.0, 7.5, dev_kmh=0.5, slope_kmhs=0.4)
    assert code == lbs.LBS_CODE_RAISED and pct == pytest.approx(7.5)


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


# ---- _ModeRun の帯の周期・帯の終わりの延長 ----

class _Ref:
    def __init__(self, v: float) -> None:
        self.v = v

    def at(self, _t: float) -> float:
        return self.v


def _run(ref_ahead: float = 1.0):
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)  # noqa: SLF001
    run.cfg = SimpleNamespace(feedforward=SimpleNamespace(creep_speed_kmh=5.0))
    run.interval_s = 0.05
    run.ref = _Ref(ref_ahead)
    run._lbs_setup = (GAIN, LAG, WAIT, CAP, MIN_PCT)  # noqa: SLF001
    run._lbs_dev_hist = deque(maxlen=5)  # noqa: SLF001
    run._lbs_prev_pct = None  # noqa: SLF001
    run._lbs_wait_left = 0  # noqa: SLF001
    run._lbs_ext_armed = False  # noqa: SLF001
    run._lbs_ext_run = 0  # noqa: SLF001
    run.lbs_code_cycles = {}
    run.lbs_raise_count = 0
    run.lbs_raise_max = 0.0
    run.lbs_cap_hits = 0
    run.lbs_ext_max_cycles = 0
    return run


def test_band_first_cycle_uses_ff_with_code_zero_then_raises() -> None:
    run = _run()
    for k in range(5):  # 偏差が 0.1/周期で増える ＝ 傾き 2.0
        dev, slope = run._lbs_observe(0.3 + 0.1 * k, 0.0)  # noqa: SLF001
    assert slope == pytest.approx(2.0)
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert (pct, code) == (6.0, 0)  # 帯の最初の周期
    dev, slope = run._lbs_observe(0.8, 0.0)  # noqa: SLF001
    pct, code = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert code == lbs.LBS_CODE_RAISED and pct > 6.0
    assert run.lbs_raise_count == 1 and run.lbs_raise_max == pytest.approx(pct - 6.0)
    dev, slope = run._lbs_observe(0.9, 0.0)  # noqa: SLF001
    pct2, code2 = run._lbs_band_cycle(6.0, dev, slope)  # noqa: SLF001
    assert code2 == lbs.LBS_CODE_WAIT and pct2 == pytest.approx(pct)


def _band_then(run, dev_seq):
    for d in dev_seq:
        dev, slope = run._lbs_observe(d, 0.0)  # noqa: SLF001
    run._lbs_band_cycle(7.0, dev, slope)  # noqa: SLF001
    return run


def test_extension_holds_while_forecast_positive_then_ends() -> None:
    run = _band_then(_run(), [0.4] * 5)  # 偏差 0.4 で平ら
    dev, slope = run._lbs_observe(0.4, 0.0)  # noqa: SLF001
    assert run._lbs_extension(1.0, dev, slope) == 7.0  # noqa: SLF001
    assert run.lbs_code_cycles[lbs.LBS_CODE_END_WAIT] == 1
    assert run.lbs_ext_max_cycles == 1
    # 偏差が負へ（見込み ≤ 0）→ 延長を終え、再び延長しない
    for d in (0.2, 0.0, -0.1, -0.2, -0.3):
        dev, slope = run._lbs_observe(d, 0.0)  # noqa: SLF001
    assert run._lbs_extension(1.1, dev, slope) is None  # noqa: SLF001
    assert run._lbs_prev_pct is None and not run._lbs_ext_armed  # noqa: SLF001
    dev, slope = run._lbs_observe(0.5, 0.0)  # noqa: SLF001
    assert run._lbs_extension(1.2, dev, slope) is None  # noqa: SLF001


def test_extension_cut_when_ref_ahead_reaches_creep_speed() -> None:
    run = _band_then(_run(ref_ahead=5.0), [0.4] * 5)
    dev, slope = run._lbs_observe(0.4, 0.0)  # noqa: SLF001
    assert run._lbs_extension(1.0, dev, slope) is None  # noqa: SLF001


def test_extension_not_armed_without_band() -> None:
    run = _run()
    dev, slope = run._lbs_observe(0.4, 0.0)  # noqa: SLF001
    assert run._lbs_extension(1.0, dev, slope) is None  # noqa: SLF001


def test_launch_step_code_column() -> None:
    assert "launch_step_code" in dlmod.CSV_COLUMNS
    assert dlmod.DriveSample.__dataclass_fields__["launch_step_code"].default is None
    # off は空欄・on は印の数字（行の組み立ては record の引数がそのまま列になる）
    import inspect

    assert "launch_step_code" in inspect.signature(dlmod.SessionLog.record).parameters
