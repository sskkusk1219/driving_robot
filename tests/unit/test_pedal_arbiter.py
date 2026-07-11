"""PedalArbiter のユニットテスト。

排他性（同時踏みなし）・ヒステリシス・再踏込ディレイ・不感帯逆補償・
レートリミット・飽和フラグを検証する。
"""

import pytest

from src.domain.control.pedal_arbiter import PedalArbiter
from src.models.profile import FeedforwardParams

DT = 0.05


def make_arbiter(
    *,
    hysteresis: float = 0.5,
    dwell: float = 0.3,
    accel_rate: float = 200.0,
    brake_rate: float = 300.0,
    accel_db: float = 1.0,
    brake_db: float = 1.0,
    max_accel: float = 80.0,
    max_brake: float = 60.0,
    accel_release_rate: float = 10.0,
    accel_min_step: float = 0.2,
) -> PedalArbiter:
    params = FeedforwardParams(
        switch_hysteresis_pct=hysteresis,
        accel_reengage_dwell_s=dwell,
        accel_rate_limit_pct_s=accel_rate,
        brake_rate_limit_pct_s=brake_rate,
        accel_deadband_pct=accel_db,
        brake_deadband_pct=brake_db,
        accel_release_rate_pct_s=accel_release_rate,
        accel_min_step_pct=accel_min_step,
    )
    return PedalArbiter(params, max_accel_opening=max_accel, max_brake_opening=max_brake)


class TestExclusivity:
    def test_no_simultaneous_pedals_across_effort_sweep(self) -> None:
        """努力量の全域掃引でアクセル・ブレーキが同時に非ゼロにならない。"""
        arb = make_arbiter(dwell=0.0)
        effort = -120.0
        while effort <= 120.0:
            out = arb.arbitrate(effort, DT)
            assert not (out.accel_opening > 0.0 and out.brake_opening > 0.0), f"effort={effort}"
            effort += 0.7

    def test_positive_effort_drives_accel(self) -> None:
        arb = make_arbiter()
        out = arb.arbitrate(20.0, DT)
        assert out.accel_opening > 0.0
        assert out.brake_opening == 0.0

    def test_negative_effort_drives_brake(self) -> None:
        arb = make_arbiter()
        out = arb.arbitrate(-20.0, DT)
        assert out.brake_opening > 0.0
        assert out.accel_opening == 0.0

    def test_pedal_switch_releases_other_immediately(self) -> None:
        """ペダル切替時、旧ペダルはレート制限なしで即座に 0 になる。"""
        arb = make_arbiter(dwell=0.0)
        for _ in range(10):
            arb.arbitrate(-30.0, DT)  # ブレーキ確立
        out = arb.arbitrate(30.0, DT)  # アクセルへ切替
        assert out.brake_opening == 0.0


class TestHysteresis:
    def test_small_effort_within_band_coasts(self) -> None:
        """±ヒステリシス帯内は惰行（両方 0）。微小な符号チャタでペダル反転しない。"""
        arb = make_arbiter(hysteresis=0.5)
        for effort in (0.3, -0.3, 0.4, -0.4, 0.0):
            out = arb.arbitrate(effort, DT)
            assert out.accel_opening == 0.0
            assert out.brake_opening == 0.0

    def test_ff_brake_not_cancelled_by_small_positive_pid(self) -> None:
        """レビュー #1(b) 再現ケース: FF ブレーキ 15% 中に PID +0.4 が乗っても
        合成努力量は −14.6 のままブレーキが維持される（旧実装は全解除していた）。"""
        arb = make_arbiter()
        arb.arbitrate(-15.0, DT)
        out = arb.arbitrate(-15.0 + 0.4, DT)
        assert out.brake_opening > 0.0
        assert out.accel_opening == 0.0

    def test_coast_in_band_is_not_saturation(self) -> None:
        arb = make_arbiter(hysteresis=0.5)
        out = arb.arbitrate(0.2, DT)
        assert not out.saturated_high
        assert not out.saturated_low


class TestReengageDwell:
    def test_accel_blocked_right_after_braking(self) -> None:
        """制動直後のアクセルはディレイ中 COAST になり飽和フラグが立つ。"""
        arb = make_arbiter(dwell=0.3)
        arb.arbitrate(-20.0, DT)  # ブレーキ
        out = arb.arbitrate(10.0, DT)  # 直後のアクセル要求
        assert out.accel_opening == 0.0
        assert out.saturated_high

    def test_accel_allowed_after_dwell_elapsed(self) -> None:
        arb = make_arbiter(dwell=0.3)
        arb.arbitrate(-20.0, DT)
        for _ in range(7):  # 0.35s 経過
            out = arb.arbitrate(10.0, DT)
        assert out.accel_opening > 0.0

    def test_brake_never_delayed_after_accel(self) -> None:
        """アクセル直後の制動要求は遅延なく適用される（減速権限の即時性）。"""
        arb = make_arbiter(dwell=10.0)
        arb.arbitrate(30.0, DT)
        out = arb.arbitrate(-30.0, DT)
        assert out.brake_opening > 0.0


class TestDeadbandCompensation:
    def test_tiny_command_snaps_to_zero(self) -> None:
        arb = make_arbiter(accel_db=2.0, dwell=0.0)
        out = arb.arbitrate(0.9, DT)  # < db/2=1.0（かつヒステリシス 0.5 超）
        assert out.accel_opening == 0.0

    def test_small_command_snaps_up_to_deadband(self) -> None:
        """死帯内 (0, db) の指令は db に引き上げられ、力の出ない開度を指令しない。"""
        arb = make_arbiter(accel_db=2.0, dwell=0.0)
        out = arb.arbitrate(1.5, DT)  # db/2=1.0 ≤ 1.5 < db=2.0
        assert out.accel_opening == pytest.approx(2.0)

    def test_command_above_deadband_passes_through(self) -> None:
        arb = make_arbiter(accel_db=2.0, dwell=0.0)
        out = arb.arbitrate(10.0, DT)
        assert out.accel_opening == pytest.approx(10.0)

    def test_brake_deadband_applied_to_magnitude(self) -> None:
        arb = make_arbiter(brake_db=2.0)
        out = arb.arbitrate(-1.5, DT)
        assert out.brake_opening == pytest.approx(2.0)


class TestRateLimit:
    def test_accel_ramp_limited(self) -> None:
        """ステップ要求はレートリミットで漸増する（200%/s × 0.05s = 10%/サイクル）。"""
        arb = make_arbiter(accel_rate=200.0, dwell=0.0)
        out1 = arb.arbitrate(50.0, DT)
        out2 = arb.arbitrate(50.0, DT)
        assert out1.accel_opening == pytest.approx(10.0)
        assert out2.accel_opening == pytest.approx(20.0)
        assert out1.saturated_high  # レートで削られた間は飽和扱い

    def test_release_not_rate_limited(self) -> None:
        arb = make_arbiter(accel_rate=200.0, dwell=0.0)
        for _ in range(10):
            arb.arbitrate(50.0, DT)
        out = arb.arbitrate(5.0, DT)  # 大幅に戻す
        assert out.accel_opening == pytest.approx(5.0)

    def test_brake_ramp_uses_brake_rate(self) -> None:
        arb = make_arbiter(brake_rate=300.0)
        out = arb.arbitrate(-50.0, DT)
        assert out.brake_opening == pytest.approx(15.0)  # 300 × 0.05
        assert out.saturated_low

    def test_large_dt_after_stall_is_clamped_to_4x_nominal(self) -> None:
        """D6 回帰テスト: バスストール復帰直後の大きな dt（例: 0.95s）は
        公称周期(0.05s)の4倍にクランプされ、1サイクルで最大速度分しか動かない。"""
        arb = make_arbiter(accel_rate=200.0, dwell=0.0)
        out = arb.arbitrate(90.0, dt=0.95)  # クランプ無しなら 200%/s×0.95s=190%（要求を満たす）
        # 4×0.05=0.2s にクランプされるため、実際のステップ上限は 200%/s×0.2s=40%
        assert out.accel_opening == pytest.approx(40.0)
        assert out.saturated_high

    def test_zero_or_negative_dt_falls_back_to_nominal(self) -> None:
        """dt<=0（未計測）は公称周期そのものを使う（PIDController と同方式）。"""
        arb = make_arbiter(accel_rate=200.0, dwell=0.0)
        out = arb.arbitrate(90.0, dt=0.0)
        assert out.accel_opening == pytest.approx(10.0)  # 200%/s × 0.05s(公称)


class TestClampAndSaturation:
    def test_accel_clamped_to_profile_max(self) -> None:
        arb = make_arbiter(max_accel=30.0, accel_rate=10000.0, dwell=0.0)
        out = arb.arbitrate(90.0, DT)
        assert out.accel_opening == pytest.approx(30.0)
        assert out.saturated_high
        assert not out.saturated_low

    def test_brake_clamped_to_profile_max(self) -> None:
        arb = make_arbiter(max_brake=25.0, brake_rate=10000.0)
        out = arb.arbitrate(-90.0, DT)
        assert out.brake_opening == pytest.approx(25.0)
        assert out.saturated_low
        assert not out.saturated_high

    def test_unsaturated_request_has_no_flags(self) -> None:
        arb = make_arbiter(accel_rate=10000.0, dwell=0.0)
        out = arb.arbitrate(20.0, DT)
        assert not out.saturated_high
        assert not out.saturated_low


class TestReset:
    def test_reset_clears_dwell_and_ramp_state(self) -> None:
        arb = make_arbiter(dwell=10.0, accel_rate=200.0)
        arb.arbitrate(-20.0, DT)  # ブレーキ実績を作る
        arb.reset()
        out = arb.arbitrate(10.0, DT)  # リセット後はディレイ起点なし
        assert out.accel_opening == pytest.approx(10.0)


class TestAccelReleaseSmoothing:
    """B-7-3: 惰行遷移時のアクセル解放を漸減し ON-OFF ハンチングを抑える。"""

    def _establish_accel(self, arb: PedalArbiter, effort: float = 20.0, cycles: int = 30) -> float:
        """定常アクセルを確立して開度を返す。"""
        out = None
        for _ in range(cycles):
            out = arb.arbitrate(effort, DT)
        assert out is not None
        return out.accel_opening

    def test_coast_dip_does_not_fully_release_accel(self) -> None:
        """定常アクセル中に1サイクルだけ惰行帯へ落ちても即0にならず漸減に留まる。"""
        arb = make_arbiter(dwell=0.0, accel_release_rate=10.0)
        steady = self._establish_accel(arb)
        assert steady > 0.0
        out = arb.arbitrate(0.2, DT)  # ヒステリシス帯内（惰行）に1サイクル落ちる
        # 10%/s × 0.05s = 0.5% しか下がらない（即0解放にならない）
        assert out.accel_opening == pytest.approx(steady - 0.5, abs=1e-6)
        assert out.brake_opening == 0.0

    def test_coast_pulse_recovers_to_steady(self) -> None:
        """1サイクルの惰行→再加速で開度がほぼ維持される（ON-OFFパルスにならない）。"""
        arb = make_arbiter(dwell=0.0, accel_release_rate=10.0)
        steady = self._establish_accel(arb)
        arb.arbitrate(0.2, DT)  # 惰行1サイクル
        out = arb.arbitrate(20.0, DT)  # 再びアクセル
        assert out.accel_opening == pytest.approx(steady)  # 0.5%下降→レートで即復帰

    def test_sustained_coast_fully_releases(self) -> None:
        """惰行が続けば最終的に0まで解放される（漸減は遅延であって固着ではない）。"""
        arb = make_arbiter(dwell=0.0, accel_release_rate=10.0)
        self._establish_accel(arb)
        out = None
        for _ in range(200):  # 十分な時間、惰行を継続
            out = arb.arbitrate(0.0, DT)
        assert out is not None
        assert out.accel_opening == 0.0

    def test_brake_request_releases_accel_immediately(self) -> None:
        """惰行と異なり、制動要求時はアクセルを即0解放する（減速権限の即時性・安全）。"""
        arb = make_arbiter(dwell=0.0, accel_release_rate=10.0)
        self._establish_accel(arb)
        out = arb.arbitrate(-20.0, DT)  # ブレーキ要求
        assert out.accel_opening == 0.0
        assert out.brake_opening > 0.0

    def test_release_rate_zero_is_instant_release(self) -> None:
        """解放レート0は従来どおり即0解放（後方互換）。"""
        arb = make_arbiter(dwell=0.0, accel_release_rate=0.0)
        self._establish_accel(arb)
        out = arb.arbitrate(0.2, DT)
        assert out.accel_opening == 0.0

    def test_no_simultaneous_pedals_with_release_smoothing(self) -> None:
        """アクセル確立後に惰行→制動と遷移しても同時踏みが起きない。"""
        arb = make_arbiter(dwell=0.0, accel_release_rate=10.0)
        self._establish_accel(arb)
        for effort in (0.2, 0.1, 0.0, -0.1, -2.0, -20.0):
            out = arb.arbitrate(effort, DT)
            assert not (out.accel_opening > 0.0 and out.brake_opening > 0.0), f"effort={effort}"


class TestAccelQuantization:
    """B-7-3: アクセルの微小変化を量子化しサーボ震え・チャタを抑える。"""

    def test_tiny_change_holds_previous_opening(self) -> None:
        """前回開度との差が min_step 未満なら前回値を保持する。"""
        arb = make_arbiter(dwell=0.0, accel_rate=10000.0, accel_min_step=0.2)
        first = arb.arbitrate(20.0, DT)  # 開度 20 を確立（レート大で即到達）
        out = arb.arbitrate(20.1, DT)  # 差 0.1 < 0.2 → 保持
        assert out.accel_opening == pytest.approx(first.accel_opening)

    def test_change_above_step_passes_through(self) -> None:
        """min_step 以上の変化は反映される。"""
        arb = make_arbiter(dwell=0.0, accel_rate=10000.0, accel_min_step=0.2)
        arb.arbitrate(20.0, DT)
        out = arb.arbitrate(20.5, DT)  # 差 0.5 ≥ 0.2 → 反映
        assert out.accel_opening == pytest.approx(20.5)
