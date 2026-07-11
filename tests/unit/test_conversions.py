"""src.domain.control.conversions のユニットテスト。

A1/A2 レビュー指摘で複数モジュールに重複していた開度→パルス変換・クランプ・
共通定数を一本化したモジュール。ここでの検証が全消費側の正しさを保証する。
"""

from src.domain.control.conversions import (
    G_TO_KMHS,
    VEHICLE_STOP_SPEED_KMH,
    clamp_opening,
    opening_to_position,
)


class TestOpeningToPosition:
    def test_zero_opening_returns_zero_pos(self) -> None:
        assert opening_to_position(0.0, zero_pos=100, full_pos=600) == 100

    def test_full_opening_returns_full_pos(self) -> None:
        assert opening_to_position(100.0, zero_pos=100, full_pos=600) == 600

    def test_half_opening_returns_midpoint(self) -> None:
        assert opening_to_position(50.0, zero_pos=100, full_pos=600) == 350

    def test_rounds_to_nearest_pulse(self) -> None:
        # 1/3 of stroke 300 = 100; zero=0 → 100
        assert opening_to_position(100.0 / 3.0, zero_pos=0, full_pos=300) == 100


class TestClampOpening:
    def test_clamps_negative_to_zero(self) -> None:
        assert clamp_opening(-10.0, 80.0) == 0.0

    def test_clamps_above_max_to_max(self) -> None:
        assert clamp_opening(90.0, 80.0) == 80.0

    def test_passes_through_within_range(self) -> None:
        assert clamp_opening(40.0, 80.0) == 40.0


class TestSharedConstants:
    def test_vehicle_stop_speed_kmh_value(self) -> None:
        assert VEHICLE_STOP_SPEED_KMH == 0.5

    def test_g_to_kmhs_value(self) -> None:
        assert G_TO_KMHS == 9.81 * 3.6
