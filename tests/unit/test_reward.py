"""reward_score（エピソード型プラン学習の報酬）のユニットテスト。"""

import statistics

import pytest

from src.domain.control.kpi_monitor import KPI_HARD_LIMIT_KMH, KPI_P95_LIMIT_KMH
from src.domain.control.reward import (
    REWARD_SIGMA_WINDOW,
    reward_noise_sigma,
    reward_scale_key,
    reward_score,
)


def _kpi(**overrides: float) -> dict[str, float]:
    """全項 0 のベース KPI に上書きを適用する。"""
    base = {
        "p95_kmh": 0.0,
        "max_abs_deviation_kmh": 0.0,
        "over_limit_integral_kmhs": 0.0,
        "effort_rate_rms_pct_s": 0.0,
        "pedal_switch_per_min": 0.0,
        "phase_violation_pct": 0.0,
    }
    base.update(overrides)
    return base


class TestMonotonicity:
    """各項の悪化でスコアが単調に下がる（大きいほど良い）。"""

    def test_perfect_run_is_zero(self) -> None:
        assert reward_score(_kpi()) == 0.0

    def test_worse_p95_lowers_score(self) -> None:
        assert reward_score(_kpi(p95_kmh=0.3)) < reward_score(_kpi(p95_kmh=0.1))

    def test_worse_max_lowers_score(self) -> None:
        assert reward_score(_kpi(max_abs_deviation_kmh=1.5)) < reward_score(
            _kpi(max_abs_deviation_kmh=0.5)
        )

    def test_worse_over_limit_lowers_score(self) -> None:
        assert reward_score(_kpi(over_limit_integral_kmhs=0.2)) < reward_score(
            _kpi(over_limit_integral_kmhs=0.0)
        )

    def test_rougher_effort_lowers_score(self) -> None:
        assert reward_score(_kpi(effort_rate_rms_pct_s=5.0)) < reward_score(
            _kpi(effort_rate_rms_pct_s=1.0)
        )

    def test_more_switches_lowers_score(self) -> None:
        assert reward_score(_kpi(pedal_switch_per_min=10.0)) < reward_score(
            _kpi(pedal_switch_per_min=1.0)
        )

    def test_more_phase_violation_lowers_score(self) -> None:
        assert reward_score(_kpi(phase_violation_pct=5.0)) < reward_score(
            _kpi(phase_violation_pct=0.0)
        )


class TestTrackingDominates:
    def test_tracking_term_dominates_at_kpi_limit(self) -> None:
        """KPI 限界（p95/max とも上限値）の追従悪化は、他項が中程度に悪い走行より低評価。"""
        at_limit = reward_score(
            _kpi(p95_kmh=KPI_P95_LIMIT_KMH, max_abs_deviation_kmh=KPI_HARD_LIMIT_KMH)
        )
        rough_but_accurate = reward_score(
            _kpi(effort_rate_rms_pct_s=3.0, pedal_switch_per_min=3.0, phase_violation_pct=2.0)
        )
        assert at_limit < rough_but_accurate


class TestKeyRobustness:
    def test_missing_keys_treated_as_zero(self) -> None:
        """ペダル・effort 情報の無い summary（学習運転）でも追従項だけで評価できる。"""
        assert reward_score({"p95_kmh": 0.1}) < 0.0
        assert reward_score({}) == 0.0


class TestRewardNoiseSigma:
    """走行間ばらつき σ の推定（採否判定の許容幅に使う）。"""

    def test_none_when_too_few_samples(self) -> None:
        assert reward_noise_sigma([]) is None
        assert reward_noise_sigma([-50.0]) is None
        assert reward_noise_sigma([-50.0, -60.0]) is None  # 既定 min_samples=3 未満

    def test_matches_sample_stdev(self) -> None:
        """実機 2026-07-23 の本番モード同一プラン 4 本（σ≈19.6）。"""
        rewards = [-53.5, -99.3, -76.1, -88.1]
        assert reward_noise_sigma(rewards) == pytest.approx(statistics.stdev(rewards))

    def test_only_recent_window_is_used(self) -> None:
        """窓長を超えた古い外れ値は σ に影響しない。"""
        recent = [-53.5, -99.3, -76.1, -88.1, -70.0]
        assert len(recent) == REWARD_SIGMA_WINDOW
        with_old_outlier = [-9999.0, *recent]
        assert reward_noise_sigma(with_old_outlier) == pytest.approx(
            reward_noise_sigma(recent)
        )

    def test_min_samples_override(self) -> None:
        assert reward_noise_sigma([-50.0, -60.0], min_samples=2) == pytest.approx(
            statistics.stdev([-50.0, -60.0])
        )

    def test_identical_rewards_give_zero(self) -> None:
        """完全に同じ報酬が続けば σ=0（許容幅ゼロ＝厳密判定に戻る）。"""
        assert reward_noise_sigma([-50.0, -50.0, -50.0]) == 0.0


class TestRewardScaleKey:
    """報酬の比較可能性キー（KPI しきい値が変われば変わる）。"""

    def test_contains_current_limits(self) -> None:
        key = reward_scale_key()
        assert f"{KPI_P95_LIMIT_KMH:g}" in key
        assert f"{KPI_HARD_LIMIT_KMH:g}" in key

    def test_changes_with_p95_limit(self, monkeypatch: pytest.MonkeyPatch) -> None:
        before = reward_scale_key()
        monkeypatch.setattr("src.domain.control.reward.KPI_P95_LIMIT_KMH", 0.2)
        assert reward_scale_key() != before

    def test_changes_with_hard_limit(self, monkeypatch: pytest.MonkeyPatch) -> None:
        before = reward_scale_key()
        monkeypatch.setattr("src.domain.control.reward.KPI_HARD_LIMIT_KMH", 2.0)
        assert reward_scale_key() != before
