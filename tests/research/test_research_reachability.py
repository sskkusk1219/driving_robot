"""段3 到達可能性判定（reachability.py）のユニットテスト。

ProblemReport_20260916 段3: `free_accel_at` の数値積分（`free_speeds_at`）がクリープ平衡へ
収束すること・追い越さないこと・刻みを細かくしても大きくは変わらないこと（オイラー法として
妥当なこと）、`reach_needs`/`decide_regime` が定義どおりであることを合成パラメータで確かめる。
"""

from __future__ import annotations

from dataclasses import replace

import pytest

from src.models.profile import FeedforwardParams
from tests.research.ff_params import ResearchFFParams
from tests.research.reachability import decide_regime, free_speeds_at, reach_needs

# クリープ域（0〜5.0 km/h）は速度依存カーブ、惰行域（5.0 km/h 以上）は速度依存カーブ。
# どちらも creep_speed_kmh=5.0 でちょうど 0 になる（段2.5 の接続点の作り方と同じ）。
PARAMS = FeedforwardParams(
    creep_speed_kmh=5.0,
    creep_rate_kmhs=0.5,
    coast_decel_speeds_kmh=(5.0, 10.0, 20.0, 30.0),
    coast_decel_kmhs=(0.0, 1.0, 2.0, 3.0),
)
RESEARCH = ResearchFFParams(
    creep_accel_speeds_kmh=(0.0, 2.0, 4.0, 5.0),
    creep_accel_kmhs=(2.0, 1.2, 0.4, 0.0),
)


# ── free_speeds_at ──────────────────────────────────────────────────


def test_free_speeds_at_converges_to_creep_equilibrium_from_below() -> None:
    """クリープ域から始めると、平衡速度（creep_speed_kmh）へ下から収束し追い越さない。"""
    (v_free,) = free_speeds_at(PARAMS, RESEARCH, 1.0, (60.0,), step_s=0.05)
    assert v_free == pytest.approx(PARAMS.creep_speed_kmh, abs=1e-2)
    assert v_free <= PARAMS.creep_speed_kmh + 1e-6  # 追い越さない


def test_free_speeds_at_converges_to_creep_equilibrium_from_above() -> None:
    """惰行域（平衡より少し上）から始めると、平衡速度へ上から収束し下回らない。"""
    (v_free,) = free_speeds_at(PARAMS, RESEARCH, 6.0, (60.0,), step_s=0.05)
    assert v_free == pytest.approx(PARAMS.creep_speed_kmh, abs=1e-2)
    assert v_free >= PARAMS.creep_speed_kmh - 1e-6  # 下回らない


def test_free_speeds_at_step_size_barely_matters() -> None:
    """惰行域（20 km/h）で 1.0s 先の値は、刻み 0.05 → 0.005 で 0.01 km/h 未満しか変わらない。"""
    (coarse,) = free_speeds_at(PARAMS, RESEARCH, 20.0, (1.0,), step_s=0.05)
    (fine,) = free_speeds_at(PARAMS, RESEARCH, 20.0, (1.0,), step_s=0.005)
    assert abs(coarse - fine) < 0.01


def test_free_speeds_at_clamps_to_zero() -> None:
    """強い減速カーブで長いホライズンなら 0 未満にクランプされる（逆走しない）。"""
    strong = replace(
        PARAMS,
        creep_speed_kmh=0.0,  # 常に惰行カーブ側（クリープ域を無効化）
        coast_decel_speeds_kmh=(0.0, 5.0, 10.0),
        coast_decel_kmhs=(5.0, 5.0, 5.0),  # 一定 5 km/h/s の強い減速
    )
    (v_free,) = free_speeds_at(strong, RESEARCH, 6.0, (2.0,), step_s=0.05)
    assert v_free == pytest.approx(0.0)


def test_free_speeds_at_records_horizon_not_a_multiple_of_step() -> None:
    """刻みの倍数でないホライズンでも正確な位置で記録される（一定減速なら厳密に一致）。

    一定減速（線形 ODE）なら前進オイラー法は刻みに関わらず厳密に一致するため、
    `v0 - decel * h` と厳密に比較できる。
    """
    linear = replace(
        PARAMS,
        creep_speed_kmh=0.0,
        coast_decel_speeds_kmh=(0.0, 100.0),
        coast_decel_kmhs=(3.0, 3.0),  # 常に一定 3 km/h/s
    )
    horizons = (0.3, 0.7, 1.5)  # いずれも step_s=0.2 の倍数ではない
    v0 = 20.0
    free = free_speeds_at(linear, RESEARCH, v0, horizons, step_s=0.2)
    expected = tuple(v0 - 3.0 * h for h in horizons)
    for got, want in zip(free, expected, strict=True):
        assert got == pytest.approx(want)


def test_free_speeds_at_rejects_non_positive_step() -> None:
    with pytest.raises(ValueError, match="step_s"):
        free_speeds_at(PARAMS, RESEARCH, 20.0, (1.0,), step_s=0.0)
    with pytest.raises(ValueError, match="step_s"):
        free_speeds_at(PARAMS, RESEARCH, 20.0, (1.0,), step_s=-0.1)


def test_free_speeds_at_empty_horizons_returns_empty_tuple() -> None:
    assert free_speeds_at(PARAMS, RESEARCH, 20.0, (), step_s=0.05) == ()


# ── reach_needs ─────────────────────────────────────────────────────


def test_reach_needs_matches_definition() -> None:
    future = (10.0, 20.0, 30.0)
    free = (9.0, 19.0, 27.0)
    horizons = (0.5, 1.0, 2.0)
    needs = reach_needs(future, free, horizons)
    assert needs == pytest.approx((2.0, 1.0, 1.5))


def test_reach_needs_rejects_length_mismatch() -> None:
    with pytest.raises(ValueError, match="長さ"):
        reach_needs((10.0, 20.0), (9.0,), (0.5, 1.0))


# ── decide_regime ───────────────────────────────────────────────────


def test_decide_regime_all_within_band_is_none() -> None:
    assert decide_regime((0.1, -0.2, 0.3), band_kmhs=0.5) is None


def test_decide_regime_mixed_signs_picks_shortest_horizon() -> None:
    """符号が違うホライズンが複数あるとき、最短ホライズンの添字を返す（遠い側より優先）。"""
    needs = (0.8, -0.9, 0.7)  # 全部帯の外。先頭（最短）が正
    assert decide_regime(needs, band_kmhs=0.5) == 0


def test_decide_regime_single_outside_band() -> None:
    assert decide_regime((0.1, 0.6, -0.9), band_kmhs=0.5) == 1


def test_decide_regime_zero_band_always_picks_first_index() -> None:
    """band_kmhs=0.0（帯無効）なら、0.0 ちょうどでも帯の外扱いになり必ず添字 0 を返す。"""
    assert decide_regime((0.0, 5.0), band_kmhs=0.0) == 0
    assert decide_regime((-0.001, 5.0), band_kmhs=0.0) == 0
