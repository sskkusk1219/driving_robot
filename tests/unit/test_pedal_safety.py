"""enforce_pedal_exclusion（同時踏み禁止ガード）のユニットテスト。"""

from __future__ import annotations

from src.domain.control.pedal_safety import enforce_pedal_exclusion


class TestEnforcePedalExclusion:
    def test_accel_only_passthrough(self) -> None:
        assert enforce_pedal_exclusion(40.0, 0.0) == (40.0, 0.0)

    def test_brake_only_passthrough(self) -> None:
        assert enforce_pedal_exclusion(0.0, 30.0) == (0.0, 30.0)

    def test_both_zero_passthrough(self) -> None:
        assert enforce_pedal_exclusion(0.0, 0.0) == (0.0, 0.0)

    def test_both_nonzero_zeros_smaller_accel(self) -> None:
        # accel < brake → accel を 0 に
        assert enforce_pedal_exclusion(10.0, 30.0) == (0.0, 30.0)

    def test_both_nonzero_zeros_smaller_brake(self) -> None:
        # brake < accel → brake を 0 に
        assert enforce_pedal_exclusion(50.0, 20.0) == (50.0, 0.0)

    def test_tie_keeps_brake(self) -> None:
        # 同値はブレーキ優先（安全側）
        assert enforce_pedal_exclusion(25.0, 25.0) == (0.0, 25.0)
