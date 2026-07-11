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


class TestOverLimitIntegral:
    """Stage B: 超過量×時間の積分（tuning_cost に連続勾配を与える）。"""

    def test_zero_when_no_violation(self) -> None:
        mon = KPIMonitor()
        for i in range(100):
            mon.update(50.0, 50.5, i * DT)  # |dev|=0.5 < 1.0
        s = mon.summary()
        assert s["over_limit_integral_kmhs"] == 0.0
        assert s["time_over_limit_s"] == 0.0

    def test_matches_analytic_rectangular_integral(self) -> None:
        """定常超過 |dev|=1.5（超過0.5）を 1.0s（0.1s×10サンプル）継続 → 積分≒0.5×1.0=0.5。
        最初のサンプルは dt 基準がないため積分対象外（10サンプルなら 9 区間ぶん = 0.9s×0.5）。"""
        mon = KPIMonitor()
        dt = 0.1
        n = 11  # 11 サンプル = 10 区間 = 1.0s
        for i in range(n):
            mon.update(50.0, 51.5, i * dt)  # |dev|=1.5, over=0.5
        s = mon.summary()
        assert s["over_limit_integral_kmhs"] == pytest.approx(0.5 * 1.0, abs=1e-9)
        assert s["time_over_limit_s"] == pytest.approx(1.0, abs=1e-9)

    def test_large_dt_gap_is_clamped(self) -> None:
        """pause 復帰等の大ギャップ（dt=100s）でも積分は _DT_CLAMP_S(=0.5s) で頭打ち。"""
        mon = KPIMonitor()
        mon.update(50.0, 52.0, 0.0)  # over=1.0（1サンプル目、積分対象外）
        mon.update(50.0, 52.0, 100.0)  # dt=100s → 0.5s にクランプ
        s = mon.summary()
        assert s["over_limit_integral_kmhs"] == pytest.approx(1.0 * 0.5, abs=1e-9)
        assert s["time_over_limit_s"] == pytest.approx(0.5, abs=1e-9)


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
            "over_limit_integral_kmhs",
            "time_over_limit_s",
            "accel_on_count",
            "accel_on_per_min",
            "pedal_travel_pct",
            "pedal_switch_count",
            "pedal_switch_per_min",
        }


class TestPedalActivity:
    """B-7-4: ペダルハンチング指標（ON-OFF 立ち上がり・総移動量）。"""

    def test_no_pedal_data_yields_zero(self) -> None:
        """accel_opening を渡さなければペダル指標は 0（後方互換）。"""
        mon = KPIMonitor()
        for i in range(20):
            mon.update(50.0, 50.0, i * DT)
        s = mon.summary()
        assert s["accel_on_count"] == 0.0
        assert s["accel_on_per_min"] == 0.0
        assert s["pedal_travel_pct"] == 0.0

    def test_counts_on_transitions(self) -> None:
        """0→非0 の立ち上がりのみをカウントする（ON 継続や OFF はカウントしない）。"""
        mon = KPIMonitor()
        # openings: 0, 2, 2, 0, 0, 3, 0 → 立ち上がりは 0→2 と 0→3 の2回
        seq = [0.0, 2.0, 2.0, 0.0, 0.0, 3.0, 0.0]
        for i, op in enumerate(seq):
            mon.update(50.0, 50.0, i * DT, accel_opening=op)
        assert mon.summary()["accel_on_count"] == 2.0

    def test_threshold_ignores_sub_half_percent(self) -> None:
        """0.5% 以下の微小開度は ON とみなさない（ノイズ・不感帯ゆらぎ除外）。"""
        mon = KPIMonitor()
        seq = [0.0, 0.4, 0.0, 0.6, 0.0]  # 0.4 は ON でない、0.6 のみ立ち上がり
        for i, op in enumerate(seq):
            mon.update(50.0, 50.0, i * DT, accel_opening=op)
        assert mon.summary()["accel_on_count"] == 1.0

    def test_pedal_travel_sums_abs_changes(self) -> None:
        """総移動量は開度差の絶対値の和。"""
        mon = KPIMonitor()
        seq = [0.0, 2.0, 1.0, 3.0]  # |Δ| = 2 + 1 + 2 = 5
        for i, op in enumerate(seq):
            mon.update(50.0, 50.0, i * DT, accel_opening=op)
        assert mon.summary()["pedal_travel_pct"] == pytest.approx(5.0)

    def test_accel_on_per_min_normalizes_by_duration(self) -> None:
        """立ち上がり回数を走行時間で正規化する（回/min）。"""
        mon = KPIMonitor()
        # 6秒で3回立ち上げる → 3 / (6/60) = 30回/min
        dt = 0.1
        pattern = [2.0, 0.0]
        # 60サンプル=6s。ON-OFF を繰り返すと立ち上がりは 30 回になるが、
        # ここでは明示的に3回だけ立ち上げるシーケンスを作る。
        seq = [0.0] * 60
        for idx in (10, 30, 50):
            seq[idx] = 2.0  # 各点で 0→2 の立ち上がり（前後は 0）
        _ = pattern
        for i, op in enumerate(seq):
            mon.update(50.0, 50.0, i * dt, accel_opening=op)
        s = mon.summary()
        assert s["accel_on_count"] == 3.0
        # duration = (59-0)*0.1 = 5.9s → 3/(5.9/60) ≈ 30.5
        assert s["accel_on_per_min"] == pytest.approx(3.0 / (5.9 / 60.0), rel=1e-6)


class TestPedalSwitch:
    """B-8-4: アクセル⇔ブレーキ交互踏み（不要切替）の集計。"""

    def test_no_brake_data_yields_zero(self) -> None:
        """brake_opening を渡さなければ交互踏みは 0（後方互換）。"""
        mon = KPIMonitor()
        for i in range(20):
            mon.update(50.0, 50.0, i * DT, accel_opening=2.0)
        assert mon.summary()["pedal_switch_count"] == 0.0

    def test_counts_accel_brake_alternation_within_window(self) -> None:
        """2秒以内のアクセル⇔ブレーキ切替を数える。"""
        mon = KPIMonitor()
        dt = 0.1
        # accel → (0.5s後) brake → (0.5s後) accel = 2回の交互踏み
        seq = [(5.0, 0.0)] * 3 + [(0.0, 5.0)] * 3 + [(5.0, 0.0)] * 3
        for i, (a, b) in enumerate(seq):
            mon.update(50.0, 50.0, i * dt, accel_opening=a, brake_opening=b)
        assert mon.summary()["pedal_switch_count"] == 2.0

    def test_slow_alternation_not_counted(self) -> None:
        """切替間隔が 2 秒を超えたら交互踏みに数えない。"""
        mon = KPIMonitor()
        dt = 0.1
        # accel を踏み、25サンプル(2.5s)後にブレーキ → 窓外
        seq = [(5.0, 0.0)] + [(0.0, 0.0)] * 25 + [(0.0, 5.0)]
        for i, (a, b) in enumerate(seq):
            mon.update(50.0, 50.0, i * dt, accel_opening=a, brake_opening=b)
        assert mon.summary()["pedal_switch_count"] == 0.0

    def test_switch_per_min_normalized(self) -> None:
        mon = KPIMonitor()
        dt = 0.1
        seq = [(5.0, 0.0)] * 2 + [(0.0, 5.0)] * 2  # 1回の交互踏み
        for i, (a, b) in enumerate(seq):
            mon.update(50.0, 50.0, i * dt, accel_opening=a, brake_opening=b)
        s = mon.summary()
        assert s["pedal_switch_count"] == 1.0
        assert s["pedal_switch_per_min"] == pytest.approx(1.0 / (0.3 / 60.0), rel=1e-6)
