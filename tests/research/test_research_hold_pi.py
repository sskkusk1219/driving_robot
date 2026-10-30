"""hold_pi のテスト（合成の段）。"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research import hold_pi as hp

DT = 0.05


def _series(k: float, tau: float, lag: float, n_steps: int = 6, dur: float = 4.0,
            noise: float = 0.003, seed: int = 0) -> hp.Series:
    """速度 5 km/h から、指令を 0.1% ずつ深くした段。各段で v は v_end へ L 後に 1 次遅れ。"""
    rng = np.random.default_rng(seed)
    t_all, v_all, u_all = [], [], []
    v0, u, t0 = 5.0, 4.0, 0.0
    for _ in range(n_steps):
        u += 0.1
        v_end = v0 - k * 0.1
        t = t0 + np.arange(0, dur, DT)
        x = np.clip(t - t0 - lag, 0.0, None)
        v = v_end + (v0 - v_end) * np.exp(-x / tau)
        t_all.append(t)
        v_all.append(v + rng.normal(0, noise, len(t)))
        u_all.append(np.full(len(t), u))
        v0, t0 = v_end, t0 + dur
    # 先頭に Δu の基準になる直前の段（指令 4.0）
    pre_t = -1.0 + np.arange(0, 1.0, DT)
    return hp.Series(
        np.concatenate([pre_t, *t_all]),
        np.concatenate([np.full(len(pre_t), 5.0), *v_all]),
        np.concatenate([np.full(len(pre_t), 4.0), *u_all]),
    )


def test_recovers_k_and_t() -> None:
    s = _series(k=2.0, tau=1.5, lag=0.155)
    r = hp.compute_hold_pi(s, lag_s=0.155, creep_kmh=5.0, deviation_c=6.0)
    assert r["steps_used"] >= 3
    assert r["K_kmh_per_pct"] == pytest.approx(2.0, rel=0.1)
    assert r["T_s"] == pytest.approx(1.5, rel=0.1)


def test_pi_formula() -> None:
    s = _series(k=2.0, tau=1.5, lag=0.2)
    r = hp.compute_hold_pi(s, lag_s=0.2, creep_kmh=5.0, deviation_c=6.0)
    lam = 6.0 * 0.2
    assert r["lambda_s"] == pytest.approx(lam)
    kp = r["T_s"] / (r["K_kmh_per_pct"] * (lam + 0.2))
    assert r["kp_pct_per_kmh"] == pytest.approx(kp)
    assert r["ki_pct_per_kmh_s"] == pytest.approx(kp / r["T_s"])


def test_too_few_steps_raises() -> None:
    s = _series(k=2.0, tau=1.5, lag=0.155, n_steps=2)
    with pytest.raises(hp.HoldPiError):
        hp.compute_hold_pi(s, lag_s=0.155, creep_kmh=5.0, deviation_c=6.0)


def test_steps_above_creep_not_used() -> None:
    s = _series(k=2.0, tau=1.5, lag=0.155)
    with pytest.raises(hp.HoldPiError):
        hp.compute_hold_pi(s, lag_s=0.155, creep_kmh=4.0, deviation_c=6.0)


def test_no_change_steps_not_used() -> None:
    s = _series(k=0.0, tau=1.5, lag=0.155, noise=0.0)
    with pytest.raises(hp.HoldPiError):
        hp.compute_hold_pi(s, lag_s=0.155, creep_kmh=5.0, deviation_c=6.0)


def test_diverged_fit_not_used() -> None:
    # 段の中で落ち着かず直線的に下がり続ける段（062805 の 83.83 s）は、T が境界・v_end が負に
    # なるので使わない
    s = _series(k=40.0, tau=200.0, lag=0.155)
    with pytest.raises(hp.HoldPiError):
        hp.compute_hold_pi(s, lag_s=0.155, creep_kmh=5.0, deviation_c=6.0)


def _low_hold_point(k: float, tau: float, lag: float, kp: float, ki: float,
                    dur: float = 40.0, seed: int = 1, t_start: float = 0.0) -> hp.Series:
    """1 次遅れ（K・T・L）を PI で閉ループにした点。基準を 0.05 Hz 級でゆっくり動かして励起する。"""
    rng = np.random.default_rng(seed)
    n = int(dur / DT)
    t = t_start + np.arange(n) * DT
    nd = int(round(lag / DT))
    u = np.zeros(n)
    v = np.zeros(n)
    v[0] = 1.6
    integ = 0.0
    u0 = 5.0
    for i in range(1, n):
        ref = 1.6 + 0.3 * np.sin(2 * np.pi * 0.2 * t[i]) + 0.2 * np.sin(2 * np.pi * 0.07 * t[i])
        err = v[i - 1] - ref  # 速すぎ → ブレーキを深く
        integ += err * DT
        u[i] = u0 + kp * err + ki * integ
        ud = u[i - nd] if i >= nd else u0
        dv = (-v[i - 1] - k * (ud - u0)) / tau + 1.6 / tau  # 平衡 v=1.6 at u0
        v[i] = v[i - 1] + dv * DT
    v = v + rng.normal(0, 0.01, n)
    return hp.Series(t, v, u)


def _two_points(**kw) -> hp.Series:
    a = _low_hold_point(t_start=0.0, seed=1, **kw)
    b = _low_hold_point(t_start=100.0, seed=2, **kw)
    return hp.Series(
        np.concatenate([a.t_s, b.t_s]),
        np.concatenate([a.speed_kmh, b.speed_kmh]),
        np.concatenate([a.cmd_pct, b.cmd_pct]),
    )


def test_low_hold_recovers_k_and_t() -> None:
    s = _two_points(k=50.0, tau=7.0, lag=0.1555, kp=0.13, ki=0.03)
    r = hp.compute_hold_pi_low_hold(s, lag_s=0.1555, deviation_c=6.0)
    assert r["source"] == "LOW_HOLD"
    assert r["steps_total"] == 2
    assert r["K_kmh_per_pct"] == pytest.approx(50.0, rel=0.3)
    assert r["T_s"] == pytest.approx(7.0, rel=0.3)


def test_low_hold_too_few_points_raises() -> None:
    s = _low_hold_point(k=50.0, tau=7.0, lag=0.1555, kp=0.13, ki=0.03)
    with pytest.raises(hp.HoldPiError):
        hp.compute_hold_pi_low_hold(s, lag_s=0.1555, deviation_c=6.0)


def test_simc_ti_is_min() -> None:
    long_t = hp.pi_values(50.0, 7.0, 0.1555, 6.0)
    cap = 4.0 * (6.0 * 0.1555 + 0.1555)
    assert long_t["Ti_s"] == pytest.approx(cap)
    assert long_t["ki_pct_per_kmh_s"] == pytest.approx(long_t["kp_pct_per_kmh"] / cap)
    short_t = hp.pi_values(2.0, 0.3, 0.1555, 6.0)
    assert short_t["Ti_s"] == pytest.approx(0.3)


def test_brake_search_result_has_source_and_ti() -> None:
    s = _series(k=2.0, tau=1.5, lag=0.155)
    r = hp.compute_hold_pi(s, lag_s=0.155, creep_kmh=5.0, deviation_c=6.0)
    assert r["source"] == "BRAKE_SEARCH"
    assert r["Ti_s"] == pytest.approx(min(r["T_s"], 4.0 * (0.93 + 0.155)))
