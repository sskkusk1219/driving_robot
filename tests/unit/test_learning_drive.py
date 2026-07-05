"""LearningDriveManager（開度パターン生成）のユニットテスト。"""

from datetime import UTC, datetime

from src.domain.learning_drive import LearningDriveConfig, LearningDriveManager
from src.models.learning_drive import PatternKind
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile

# ---------------------------------------------------------------------------
# テスト用ヘルパー
# ---------------------------------------------------------------------------


def make_profile(
    max_speed: float = 100.0,
    max_accel_opening: float = 80.0,
    max_brake_opening: float = 80.0,
    max_decel_g: float = 0.5,
    stop_brake_opening_pct: float = 20.0,
) -> VehicleProfile:
    return VehicleProfile(
        id="test-profile",
        name="TestProfile",
        max_accel_opening=max_accel_opening,
        max_brake_opening=max_brake_opening,
        max_speed=max_speed,
        max_decel_g=max_decel_g,
        pid_gains=PIDGains(kp=1.0, ki=0.1, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None,
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
        feedforward_params=FeedforwardParams(stop_brake_opening_pct=stop_brake_opening_pct),
    )


def make_manager(
    creep_steps: int = 5,
    hold_duration: float = 3.0,
    *,
    accel_deadband_probe_pcts: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0, 5.0),
    accel_deadband_probe_hold_s: float = 3.0,
    accel_sweep_fracs: tuple[float, ...] = (0.3, 0.5, 0.7, 1.0),
    accel_sweep_reset_brake_pct: float = 30.0,
    brake_hold_openings_pct: tuple[float, ...] = (10.0, 20.0, 30.0, 40.0),
    brake_hold_accel_pct: float = 70.0,
    coast_down_count: int = 0,
    coast_down_accel_pct: float = 70.0,
) -> LearningDriveManager:
    cfg = LearningDriveConfig(
        creep_release_steps=creep_steps,
        hold_duration_s=hold_duration,
        accel_deadband_probe_pcts=accel_deadband_probe_pcts,
        accel_deadband_probe_hold_s=accel_deadband_probe_hold_s,
        accel_sweep_fracs=accel_sweep_fracs,
        accel_sweep_reset_brake_pct=accel_sweep_reset_brake_pct,
        brake_hold_openings_pct=brake_hold_openings_pct,
        brake_hold_accel_pct=brake_hold_accel_pct,
        coast_down_count=coast_down_count,
        coast_down_accel_pct=coast_down_accel_pct,
    )
    return LearningDriveManager(config=cfg)


# ---------------------------------------------------------------------------
# generate_patterns テスト
# ---------------------------------------------------------------------------


class TestGeneratePatterns:
    def test_returns_non_empty_list(self) -> None:
        patterns = make_manager().generate_patterns(make_profile())
        assert len(patterns) > 0

    def test_contains_expected_kinds(self) -> None:
        kinds = {
            p.kind for p in make_manager(coast_down_count=3).generate_patterns(make_profile())
        }
        assert kinds == {
            PatternKind.CREEP,
            PatternKind.CREEP_SETTLE,
            PatternKind.ACCEL_DEADBAND_PROBE,
            PatternKind.ACCEL_SWEEP,
            PatternKind.BRAKE_HOLD,
            PatternKind.COAST_DOWN,
        }

    def test_exactly_one_creep_settle_with_zero_openings(self) -> None:
        settles = [
            p
            for p in make_manager().generate_patterns(make_profile())
            if p.kind is PatternKind.CREEP_SETTLE
        ]
        assert len(settles) == 1
        assert settles[0].accel_opening == 0.0
        assert settles[0].brake_opening == 0.0

    def test_accel_opening_within_max(self) -> None:
        profile = make_profile(max_accel_opening=60.0)
        for p in make_manager().generate_patterns(profile):
            assert p.accel_opening <= profile.max_accel_opening + 1e-9

    def test_brake_opening_within_max(self) -> None:
        profile = make_profile(max_brake_opening=50.0)
        for p in make_manager().generate_patterns(profile):
            assert p.brake_opening <= profile.max_brake_opening + 1e-9

    def test_all_openings_non_negative(self) -> None:
        for p in make_manager().generate_patterns(make_profile()):
            assert p.accel_opening >= 0.0
            assert p.brake_opening >= 0.0

    def test_hold_duration_uses_config(self) -> None:
        # ACCEL_DEADBAND_PROBE は専用の hold_duration_s（accel_deadband_probe_hold_s）を持つため
        # 汎用の hold_duration_s とは別に検証する。
        patterns = make_manager(hold_duration=2.5).generate_patterns(make_profile())
        non_probe = [p for p in patterns if p.kind is not PatternKind.ACCEL_DEADBAND_PROBE]
        assert all(p.hold_duration_s == 2.5 for p in non_probe)

    def test_creep_release_starts_at_stop_brake_and_decreases_above_zero(self) -> None:
        profile = make_profile(stop_brake_opening_pct=20.0)
        creep = [p.brake_opening for p in make_manager(creep_steps=5).generate_patterns(profile)
                 if p.kind is PatternKind.CREEP]
        assert creep[0] == 20.0  # 停車保持ブレーキから開始
        assert creep[-1] > 0.0  # 0 は CREEP_SETTLE が担うため解放ステップは > 0 で終わる
        assert creep == sorted(creep, reverse=True)  # 単調減少

    def test_creep_brake_clamped_to_max_brake_opening(self) -> None:
        # 停車保持ブレーキが max_brake_opening を超える設定でもクランプされる
        profile = make_profile(max_brake_opening=15.0, stop_brake_opening_pct=20.0)
        for p in make_manager().generate_patterns(profile):
            assert p.brake_opening <= profile.max_brake_opening + 1e-9


class TestAccelSweep:
    def test_accel_sweep_openings_are_fractions_of_max(self) -> None:
        # 30/50/70/100% of max_accel(80) → 24/40/56/80
        profile = make_profile(max_accel_opening=80.0)
        accel = sorted(
            p.accel_opening
            for p in make_manager(accel_sweep_fracs=(0.3, 0.5, 0.7, 1.0)).generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_SWEEP
        )
        assert accel == [24.0, 40.0, 56.0, 80.0]

    def test_accel_sweep_reaches_max_accel_opening(self) -> None:
        # 全域到達する加速段（最大開度＝cap まで届く段）が含まれる
        profile = make_profile(max_accel_opening=80.0)
        accel = [
            p.accel_opening
            for p in make_manager().generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_SWEEP
        ]
        assert max(accel) == 80.0

    def test_accel_sweep_clamped_to_max(self) -> None:
        # frac×max でも max を超えない（frac>1 のような設定でもクランプ）
        profile = make_profile(max_accel_opening=50.0)
        for p in make_manager(accel_sweep_fracs=(0.5, 1.0, 1.5)).generate_patterns(profile):
            if p.kind is PatternKind.ACCEL_SWEEP:
                assert p.accel_opening <= 50.0 + 1e-9

    def test_accel_sweep_has_reset_brake(self) -> None:
        # ACCEL_SWEEP は停車復帰用のリセットブレーキ（>0）を持つ
        profile = make_profile(max_brake_opening=80.0)
        sweeps = [
            p for p in make_manager(accel_sweep_reset_brake_pct=30.0).generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_SWEEP
        ]
        assert sweeps
        for p in sweeps:
            assert p.brake_opening == 30.0

    def test_accel_sweep_reset_brake_clamped_to_max(self) -> None:
        profile = make_profile(max_brake_opening=20.0)
        for p in make_manager(accel_sweep_reset_brake_pct=30.0).generate_patterns(profile):
            if p.kind is PatternKind.ACCEL_SWEEP:
                assert p.brake_opening == 20.0  # max_brake で頭打ち

    def test_zero_max_accel_yields_no_accel_sweep(self) -> None:
        profile = make_profile(max_accel_opening=0.0)
        sweeps = [
            p for p in make_manager().generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_SWEEP
        ]
        assert sweeps == []


class TestAccelDeadbandProbe:
    def test_probe_openings_ascending_and_match_config(self) -> None:
        profile = make_profile(max_accel_opening=80.0)
        probes = [
            p.accel_opening
            for p in make_manager(
                accel_deadband_probe_pcts=(0.5, 1.0, 2.0, 3.0, 5.0)
            ).generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_DEADBAND_PROBE
        ]
        assert probes == [0.5, 1.0, 2.0, 3.0, 5.0]

    def test_probe_brake_is_zero(self) -> None:
        profile = make_profile()
        probes = [
            p
            for p in make_manager().generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_DEADBAND_PROBE
        ]
        assert probes
        assert all(p.brake_opening == 0.0 for p in probes)

    def test_probe_uses_dedicated_hold_duration(self) -> None:
        profile = make_profile()
        probes = [
            p
            for p in make_manager(
                hold_duration=3.0, accel_deadband_probe_hold_s=1.5
            ).generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_DEADBAND_PROBE
        ]
        assert probes
        assert all(p.hold_duration_s == 1.5 for p in probes)

    def test_probe_clamped_to_max_accel_opening(self) -> None:
        profile = make_profile(max_accel_opening=2.0)
        probes = [
            p.accel_opening
            for p in make_manager(
                accel_deadband_probe_pcts=(0.5, 1.0, 2.0, 3.0, 5.0)
            ).generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_DEADBAND_PROBE
        ]
        assert all(o <= 2.0 + 1e-9 for o in probes)

    def test_zero_max_accel_yields_no_probe(self) -> None:
        profile = make_profile(max_accel_opening=0.0)
        probes = [
            p
            for p in make_manager().generate_patterns(profile)
            if p.kind is PatternKind.ACCEL_DEADBAND_PROBE
        ]
        assert probes == []


class TestBrakeHold:
    def test_default_brake_hold_openings_include_low_values(self) -> None:
        """既定の BRAKE_HOLD_OPENINGS_PCT に不感帯推定用の低開度が含まれること。"""
        from src.domain.learning_drive import BRAKE_HOLD_OPENINGS_PCT

        assert min(BRAKE_HOLD_OPENINGS_PCT) <= 5.0


    def test_brake_hold_sweeps_several_openings(self) -> None:
        profile = make_profile(max_brake_opening=80.0)
        brakes = sorted(
            p.brake_opening
            for p in make_manager(
                brake_hold_openings_pct=(10.0, 20.0, 30.0, 40.0)
            ).generate_patterns(profile)
            if p.kind is PatternKind.BRAKE_HOLD
        )
        assert brakes == [10.0, 20.0, 30.0, 40.0]

    def test_brake_hold_clamped_to_max_brake(self) -> None:
        # max_brake を超えるブレーキ段はクランプされる
        profile = make_profile(max_brake_opening=25.0)
        for p in make_manager(brake_hold_openings_pct=(10.0, 20.0, 30.0, 40.0)).generate_patterns(
            profile
        ):
            if p.kind is PatternKind.BRAKE_HOLD:
                assert p.brake_opening <= 25.0 + 1e-9

    def test_brake_hold_accel_reaches_cap_and_clamped(self) -> None:
        # cap まで上げる加速開度を持ち、max_accel でクランプされる
        profile = make_profile(max_accel_opening=60.0)
        holds = [
            p for p in make_manager(brake_hold_accel_pct=70.0).generate_patterns(profile)
            if p.kind is PatternKind.BRAKE_HOLD
        ]
        assert holds
        for p in holds:
            assert p.accel_opening == 60.0  # 70% 指定だが max_accel 60 で頭打ち

    def test_zero_max_accel_yields_no_brake_hold(self) -> None:
        # cap まで上げられないなら BRAKE_HOLD は生成しない
        profile = make_profile(max_accel_opening=0.0)
        holds = [
            p for p in make_manager().generate_patterns(profile)
            if p.kind is PatternKind.BRAKE_HOLD
        ]
        assert holds == []


class TestCoastDown:
    def test_coast_down_patterns_generated(self) -> None:
        profile = make_profile(max_accel_opening=80.0)
        patterns = make_manager(coast_down_count=3, coast_down_accel_pct=70.0).generate_patterns(
            profile
        )
        coast = [p for p in patterns if p.kind is PatternKind.COAST_DOWN]
        assert len(coast) == 3
        for p in coast:
            assert p.accel_opening == 70.0
            assert p.brake_opening == 0.0  # ブレーキ無しで惰行

    def test_coast_down_accel_clamped_to_max(self) -> None:
        profile = make_profile(max_accel_opening=30.0)
        coast = [p for p in make_manager(coast_down_count=2, coast_down_accel_pct=70.0)
                 .generate_patterns(profile) if p.kind is PatternKind.COAST_DOWN]
        assert all(p.accel_opening == 30.0 for p in coast)

    def test_coast_down_count_zero_yields_none(self) -> None:
        coast = [p for p in make_manager(coast_down_count=0).generate_patterns(make_profile())
                 if p.kind is PatternKind.COAST_DOWN]
        assert coast == []


class TestOrdering:
    def test_phases_in_expected_order(self) -> None:
        # CREEP → CREEP_SETTLE → ACCEL_DEADBAND_PROBE → ACCEL_SWEEP → BRAKE_HOLD → COAST_DOWN の順
        patterns = make_manager(coast_down_count=2).generate_patterns(make_profile())
        order = [p.kind for p in patterns]
        first_idx = {
            kind: order.index(kind)
            for kind in (
                PatternKind.CREEP,
                PatternKind.CREEP_SETTLE,
                PatternKind.ACCEL_DEADBAND_PROBE,
                PatternKind.ACCEL_SWEEP,
                PatternKind.BRAKE_HOLD,
                PatternKind.COAST_DOWN,
            )
        }
        assert (
            first_idx[PatternKind.CREEP]
            < first_idx[PatternKind.CREEP_SETTLE]
            < first_idx[PatternKind.ACCEL_DEADBAND_PROBE]
            < first_idx[PatternKind.ACCEL_SWEEP]
            < first_idx[PatternKind.BRAKE_HOLD]
            < first_idx[PatternKind.COAST_DOWN]
        )
