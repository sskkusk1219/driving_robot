"""KPIMonitor のユニットテスト。"""

import logging

import pytest

from src.domain.control.kpi_monitor import KPIMonitor

DT = 0.05


class TestP95:
    def test_empty_returns_zero(self) -> None:
        assert KPIMonitor().p95_kmh() == 0.0

    def test_p95_of_known_series(self) -> None:
        """100 点中 95 点が 0.1、5 点が 0.5 → P95 はおおよそ 0.1。"""
        mon = KPIMonitor()
        t = 0.0
        for _ in range(95):
            mon.update(50.0, 50.1, t)
            t += DT
        for _ in range(5):
            mon.update(50.0, 50.5, t)
            t += DT
        assert mon.p95_kmh() == pytest.approx(0.11, abs=0.011)  # ビン上端の保守値

    def test_p95_counts_sign_agnostic(self) -> None:
        """偏差の符号に関係なく絶対値で集計する。"""
        mon = KPIMonitor()
        for i in range(100):
            mon.update(50.0, 50.0 - 0.3, i * DT)
        assert mon.p95_kmh() == pytest.approx(0.31, abs=0.011)


class TestMaxDeviation:
    def test_max_tracks_peak(self) -> None:
        mon = KPIMonitor()
        mon.update(50.0, 50.2, 0.0)
        mon.update(50.0, 49.3, 0.05)
        mon.update(50.0, 50.1, 0.10)
        assert mon.summary()["max_abs_deviation_kmh"] == pytest.approx(0.7)


class TestSignReversal:
    def test_oscillation_counted_in_window(self) -> None:
        """0.5 秒ごとに偏差符号が反転する系列 → 5 秒窓で多数の反転を検出。"""
        mon = KPIMonitor()
        t = 0.0
        for i in range(20):  # 10 秒間、0.5 秒ごとに ±0.3 を交互
            dev = 0.3 if i % 2 == 0 else -0.3
            for _ in range(10):  # 0.5 秒 = 10 サイクル
                mon.update(50.0, 50.0 + dev, t)
                t += DT
        assert mon.summary()["reversal_max_per_5s"] >= 8

    def test_noise_floor_suppresses_tiny_chatter(self) -> None:
        """±0.03 km/h（ノイズフロア内）のチャタは反転としてカウントしない。"""
        mon = KPIMonitor()
        t = 0.0
        for i in range(200):
            dev = 0.03 if i % 2 == 0 else -0.03
            mon.update(50.0, 50.0 + dev, t)
            t += DT
        assert mon.summary()["reversal_max_per_5s"] == 0.0

    def test_single_crossing_counts_once(self) -> None:
        mon = KPIMonitor()
        for i in range(10):
            mon.update(50.0, 50.2, i * DT)
        for i in range(10, 20):
            mon.update(50.0, 49.8, i * DT)
        assert mon.summary()["reversal_max_per_5s"] == 1.0

    def test_old_reversals_fall_out_of_window(self) -> None:
        """5 秒より古い反転は窓から外れる（10 時間走行でも deque が成長しない）。"""
        mon = KPIMonitor()
        mon.update(50.0, 50.2, 0.0)
        mon.update(50.0, 49.8, 0.1)  # 反転1
        mon.update(50.0, 50.2, 10.0)  # 反転2（窓外の反転1 は落ちる）
        assert len(mon._reversal_times) == 1


class TestHardLimit:
    def test_violation_logged_once_per_excursion(self, caplog: pytest.LogCaptureFixture) -> None:
        mon = KPIMonitor()
        with caplog.at_level(logging.WARNING):
            for i in range(10):  # 違反継続中
                mon.update(50.0, 51.5, i * DT)
        warnings = [r for r in caplog.records if "ハード上限違反" in r.message]
        assert len(warnings) == 1
        assert mon.summary()["hard_limit_violations"] == 1.0

    def test_new_excursion_counts_again(self) -> None:
        mon = KPIMonitor()
        mon.update(50.0, 51.5, 0.0)  # 違反1
        mon.update(50.0, 50.0, 0.05)  # 解除（< 0.9）
        mon.update(50.0, 51.2, 0.10)  # 違反2
        assert mon.summary()["hard_limit_violations"] == 2.0

    def test_no_violation_below_limit(self) -> None:
        mon = KPIMonitor()
        for i in range(100):
            mon.update(50.0, 50.9, i * DT)
        assert mon.summary()["hard_limit_violations"] == 0.0


class TestSummary:
    def test_summary_keys_and_counts(self) -> None:
        mon = KPIMonitor()
        for i in range(42):
            mon.update(30.0, 30.05, i * DT)
        s = mon.summary()
        assert s["n_samples"] == 42.0
        assert set(s) == {
            "n_samples",
            "max_abs_deviation_kmh",
            "p95_kmh",
            "reversal_max_per_5s",
            "hard_limit_violations",
        }
