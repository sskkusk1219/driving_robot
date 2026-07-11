"""制御ループ共通の変換関数・定数。

開度[%]⇔アクチュエータ位置[pulse]の変換とクランプ、および複数モジュールで
独立定義されていた「停車とみなす車速しきい値」「1G→km/h/s 換算係数」を
単一のソースに集約する（A1/A2 レビュー指摘）。
"""

from __future__ import annotations

# これ未満/以下を「停車」とみなす車速しきい値 [km/h]。
# 従来 robot_controller.py・learning_loop.py・pre_check.py・model_training.py で
# それぞれ独立に 0.5 と定義され、コメントで「同一であること」を頼りに揃えていた。
VEHICLE_STOP_SPEED_KMH: float = 0.5

# 1G を km/h/s へ換算する係数（9.81 m/s^2 * 3.6 km/h/(m/s)）。
# 従来 learning_loop.py・robot_controller.py・pid_tuning.py でそれぞれ独立定義されていた。
G_TO_KMHS: float = 9.81 * 3.6


def opening_to_position(opening_pct: float, zero_pos: int, full_pos: int) -> int:
    """開度 [%] をアクチュエータ位置 [pulse] に変換する。"""
    return zero_pos + round((full_pos - zero_pos) * opening_pct / 100.0)


def clamp_opening(opening_pct: float, max_opening_pct: float) -> float:
    """開度 [%] を [0, max_opening_pct] にクランプする。"""
    return max(0.0, min(opening_pct, max_opening_pct))
