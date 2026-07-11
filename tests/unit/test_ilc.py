"""反復学習制御（ILC）ドメインのユニットテスト。

線形静的プラント上での多反復収束・発散検知・振幅クランプ・ゼロ位相性・Δシフト端点処理・
コントローラの補間/クランプを検証する。
"""

import numpy as np
import pytest

from src.domain.control.ilc import (
    ILC_AMP_LIMIT_PCT,
    ILCController,
    ILCLearner,
    ILCTable,
    is_diverged,
    l_gain_from_fopdt,
    zero_phase_lowpass,
)


class TestILCController:
    def test_empty_table_returns_zero(self) -> None:
        ctrl = ILCController(ILCTable())
        assert ctrl.effort_at(0.0) == 0.0
        assert ctrl.effort_at(5.0) == 0.0

    def test_linear_interpolation(self) -> None:
        ctrl = ILCController(ILCTable(efforts=[0.0, 2.0, 4.0], dt_s=1.0))
        assert ctrl.effort_at(0.0) == pytest.approx(0.0)
        assert ctrl.effort_at(0.5) == pytest.approx(1.0)  # 0 と 2 の中間
        assert ctrl.effort_at(1.5) == pytest.approx(3.0)

    def test_out_of_range_clamps_to_endpoints(self) -> None:
        ctrl = ILCController(ILCTable(efforts=[1.0, 2.0, 3.0], dt_s=0.1))
        assert ctrl.effort_at(-5.0) == pytest.approx(1.0)  # 先頭へクランプ
        assert ctrl.effort_at(100.0) == pytest.approx(3.0)  # 末尾へクランプ

    def test_amplitude_clamp(self) -> None:
        ctrl = ILCController(ILCTable(efforts=[-50.0, 50.0], dt_s=1.0), amp_limit_pct=10.0)
        assert ctrl.effort_at(0.0) == pytest.approx(-10.0)
        assert ctrl.effort_at(1.0) == pytest.approx(10.0)


class TestZeroPhase:
    def test_symmetric_bump_keeps_peak_position(self) -> None:
        """ゼロ位相ローパスは対称な山のピーク位置を動かさない（位相遅れ 0）。"""
        n = 101
        x = np.exp(-((np.arange(n) - 50.0) ** 2) / (2 * 8.0**2))  # 中心 50 のガウス山
        y = zero_phase_lowpass(x, cutoff_hz=0.5, dt_s=0.1)
        assert int(np.argmax(y)) == 50

    def test_short_series_returned_asis(self) -> None:
        x = np.array([3.0])
        assert np.array_equal(zero_phase_lowpass(x, 0.3, 0.1), x)

    def test_zero_cutoff_is_passthrough(self) -> None:
        x = np.array([1.0, 5.0, 2.0])
        assert np.array_equal(zero_phase_lowpass(x, 0.0, 0.1), x)


class TestILCLearner:
    def _plant_errors(
        self, u: list[float], target_error: np.ndarray, plant_gain: float
    ) -> list[float]:
        """線形静的プラント: e(t) = 系統誤差(t) − plant_gain × 補正 effort(t)。"""
        u_arr = np.asarray(u, dtype=float) if u else np.zeros(len(target_error))
        return [float(te - plant_gain * uu) for te, uu in zip(target_error, u_arr, strict=True)]

    def test_converges_over_iterations(self) -> None:
        """反復で最大残差が単調減少し、系統誤差をほぼ打ち消す（KL=0.5 で収束）。"""
        n = 120
        dt = 0.1
        times = (np.arange(n) * dt).tolist()
        # 低周波の系統誤差（中央のガウス山。端でほぼ 0・微分 0 でカットオフ 0.3Hz 内に収まる）
        target_error = 2.0 * np.exp(-((np.arange(n) - 60.0) ** 2) / (2 * 15.0**2))
        plant_gain = 1.0
        learner = ILCLearner()
        table = ILCTable(efforts=[], dt_s=dt)

        max_errs = []
        for _ in range(8):
            errors = self._plant_errors(table.efforts, target_error, plant_gain)
            max_errs.append(max(abs(e) for e in errors))
            table = learner.update(
                table, times, errors, l_gain=0.5, delta_s=0.0, cutoff_hz=0.3, dt_s=dt
            )
        # 単調減少
        assert all(a >= b - 1e-9 for a, b in zip(max_errs[:-1], max_errs[1:], strict=True))
        # 初期の 3 割未満まで縮む
        assert max_errs[-1] < 0.3 * max_errs[0]

    def test_iteration_increments_and_p95_tracked(self) -> None:
        learner = ILCLearner()
        table = ILCTable(efforts=[0.0, 0.0, 0.0], dt_s=0.1)
        out = learner.update(
            table, [0.0, 0.1, 0.2], [0.0, 0.0, 0.0], l_gain=0.4, delta_s=0.0, new_p95_kmh=0.5
        )
        assert out.iteration == 1
        assert out.best_p95_kmh == 0.5
        # より良い p95 で更新、悪化では据え置き
        out2 = learner.update(out, [0.0, 0.1, 0.2], [0.0, 0.0, 0.0], l_gain=0.4, delta_s=0.0,
                              new_p95_kmh=0.9)
        assert out2.best_p95_kmh == 0.5

    def test_amplitude_clamped_in_table(self) -> None:
        """過大な誤差×ゲインでも補正はテーブル生成時に ±amp へクランプされる。"""
        learner = ILCLearner()
        table = ILCTable(efforts=[0.0] * 20, dt_s=0.1)
        errors = [100.0] * 20  # 巨大な誤差
        out = learner.update(
            table, (np.arange(20) * 0.1).tolist(), errors,
            l_gain=1.0, delta_s=0.0, amp_limit_pct=10.0, cutoff_hz=0.3, dt_s=0.1
        )
        assert all(abs(e) <= 10.0 + 1e-9 for e in out.efforts)

    def test_empty_times_returns_input(self) -> None:
        learner = ILCLearner()
        table = ILCTable(efforts=[1.0, 2.0], dt_s=0.1)
        assert learner.update(table, [], [], l_gain=0.4, delta_s=0.0) is table

    def test_delta_shift_uses_future_error_and_clamps_end(self) -> None:
        """Δ>0 は e_j(t+Δ) を参照し、末尾を越える分は最後の誤差へクランプ（例外なし）。"""
        learner = ILCLearner()
        n = 10
        dt = 0.1
        times = (np.arange(n) * dt).tolist()
        errors = [float(v) for v in range(n)]  # 0..9 のランプ
        out = learner.update(
            table=ILCTable(efforts=[0.0] * n, dt_s=dt),
            times=times,
            errors=errors,
            l_gain=1.0,
            delta_s=0.2,  # 2サンプル先を参照
            amp_limit_pct=1e9,  # クランプ無効化して素の値を見る
            cutoff_hz=0.0,  # フィルタ無効化
            dt_s=dt,
        )
        # grid[0]=0 → e(0.2)=2、grid[1]=0.1 → e(0.3)=3、… 末尾は 9 でクランプ
        assert out.efforts[0] == pytest.approx(2.0)
        assert out.efforts[1] == pytest.approx(3.0)
        assert out.efforts[-1] == pytest.approx(9.0)  # 端点クランプ


class TestDivergence:
    def test_diverged_when_p95_exceeds_best_times_factor(self) -> None:
        assert is_diverged(0.2, 0.3, factor=1.2) is True  # 0.3 > 0.24
        assert is_diverged(0.2, 0.22, factor=1.2) is False  # 0.22 < 0.24

    def test_no_best_or_no_new_is_not_diverged(self) -> None:
        assert is_diverged(None, 0.5) is False
        assert is_diverged(0.2, None) is False


class TestLGainFromFopdt:
    def test_normalizes_by_k(self) -> None:
        assert l_gain_from_fopdt(2.0, factor=0.4) == pytest.approx(0.2)

    def test_missing_k_returns_factor(self) -> None:
        assert l_gain_from_fopdt(None, factor=0.4) == pytest.approx(0.4)
        assert l_gain_from_fopdt(0.0, factor=0.4) == pytest.approx(0.4)


def test_amp_limit_default_constant() -> None:
    assert ILC_AMP_LIMIT_PCT == 10.0
