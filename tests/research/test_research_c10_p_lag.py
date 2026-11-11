"""2026-11-11 PDCA-8-3（c10_p_lag_s）: P の偏差の 1 次の遅れの単体テスト。"""

from __future__ import annotations

import math

from tests.research import config as cfgmod
from tests.research.mode_drive import c10_p_lag_text, p_lag_step


def test_first_step_returns_dev() -> None:
    assert p_lag_step(None, 0.7, 0.02, 0.5) == 0.7


def test_converges_to_constant_input() -> None:
    x: float | None = None
    x = p_lag_step(x, 0.0, 0.02, 0.5)
    for _ in range(500):
        x = p_lag_step(x, 1.0, 0.02, 0.5)
    assert math.isclose(x, 1.0, abs_tol=1e-6)


def test_single_step_value_and_lag() -> None:
    # dt ÷ (tau ＋ dt) ＝ 0.1 ÷ 0.5 ＝ 0.2
    assert math.isclose(p_lag_step(0.0, 1.0, 0.1, 0.4), 0.2)


def test_nonpositive_dt_holds() -> None:
    assert p_lag_step(0.3, 5.0, 0.0, 0.5) == 0.3
    assert p_lag_step(0.3, 5.0, -0.1, 0.5) == 0.3


def test_config_defaults_and_validation() -> None:
    fm0 = cfgmod.ForwardMapSection()
    assert fm0.c10_p_lag_s == 0.0 and fm0.c10_p_lag_min_kmh == 0.0
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    for key in ("c10_p_lag_s", "c10_p_lag_min_kmh"):
        old = getattr(cfg.forward_map, key)
        setattr(cfg.forward_map, key, -0.1)
        assert any(key in p for p in cfgmod.validate_config(cfg)), key
        setattr(cfg.forward_map, key, old)


def test_text() -> None:
    fm = cfgmod.ForwardMapSection()
    assert c10_p_lag_text(fm) == "off"
    fm.c10_p_lag_s, fm.c10_p_lag_min_kmh = 0.5, 70.0
    assert "0.5" in c10_p_lag_text(fm) and "70" in c10_p_lag_text(fm)
