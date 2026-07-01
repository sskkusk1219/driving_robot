"""アクセル・ブレーキの同時踏みを禁止する共有ガード。

学習運転・自動運転とも、ペダルへの指令は「常にどちらか一方のみ非ゼロ」であるべき
（同時踏みは機構保護・安全の観点で禁止）。自動運転は `PedalArbiter` が、学習運転は
`LearningLoop` のフェーズ機が構造的に排他を保証しているが、将来の不具合に対する保険として
両ループがアクチュエータへ送出する直前にこのガードを通す。
"""

from __future__ import annotations


def enforce_pedal_exclusion(accel_opening: float, brake_opening: float) -> tuple[float, float]:
    """アクセル・ブレーキが同時に非ゼロなら小さい方を 0 にして排他を強制する。

    同値のときはブレーキを優先（安全側）に残す。通常運用では片方が常に 0 のため no-op。
    """
    if accel_opening > 0.0 and brake_opening > 0.0:
        if brake_opening >= accel_opening:
            return 0.0, brake_opening
        return accel_opening, 0.0
    return accel_opening, brake_opening
