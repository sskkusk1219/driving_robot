"""クリープ域ブレーキの下限の推定（stop_brake_floor.py）のユニットテスト。

ProblemReport_20260916 段4改訂: クリープ平衡から候補開度を振った保持ログから「止まった／浮いた」
候補開度を集め、`offset_pct = 止まった最小開度 − 不感帯` を同定することを合成ログで確かめる。
"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta

import pytest

from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research.stop_brake_floor import estimate_stop_brake_floor

PARAMS = FeedforwardParams(
    accel_deadband_pct=5.0, brake_deadband_pct=10.0, stop_brake_opening_pct=28.42,
)


def _hold(
    speeds: list[float],
    *,
    brake_pct: float,
    accel_pct: float = 0.0,
    dt: float = 1.0,
    session_id: str,
) -> list[DriveLog]:
    """1 本の連続ブレーキ保持（accel/brake は一定）を、そのまま 1 セッションとして作る。"""
    origin = datetime(2026, 9, 19, tzinfo=UTC)
    return [
        DriveLog(
            id=i,
            session_id=session_id,
            timestamp=origin + timedelta(seconds=i * dt),
            ref_speed_kmh=None,
            actual_speed_kmh=v,
            accel_opening=accel_pct,
            brake_opening=brake_pct,
            accel_pos=0,
            brake_pos=0,
            accel_current=0.0,
            brake_current=0.0,
        )
        for i, v in enumerate(speeds)
    ]


def test_identifies_offset_as_min_stopped_opening_minus_deadband() -> None:
    """クリープ平衡（5.0 km/h）から開度を振った保持ログから、止まった最小開度 − 不感帯 が返る。

    12.0% は浮いたまま（1.3 km/h 付近に張り付き）、14.0%・16.0% は停止する。最小の 14.0% から
    オフセットを採る（浮いた最大 12.0% はそれより低いので矛盾しない）。
    """
    floated_run = _hold(
        [5.0, 4.0, 3.0, 2.0, 1.5, 1.4, 1.35, 1.3], brake_pct=12.0, session_id="float"
    )
    stopped_run_a = _hold([5.0, 4.0, 3.0, 2.0, 1.0, 0.5, 0.0], brake_pct=14.0, session_id="stop_a")
    stopped_run_b = _hold([5.0, 3.0, 1.0, 0.0], brake_pct=16.0, session_id="stop_b")

    floor = estimate_stop_brake_floor(
        floated_run + stopped_run_a + stopped_run_b, PARAMS,
        candidate_openings_pct=(12.0, 14.0, 16.0),
        start_speed_kmh=5.0, start_tol_kmh=1.0, min_float_s=5.0, opening_tol_pct=0.3,
    )

    assert floor.identified
    assert floor.stopped_pct == (14.0, 16.0)
    assert floor.floated_pct == (12.0,)
    assert floor.offset_pct == pytest.approx(14.0 - PARAMS.brake_deadband_pct)


def test_high_speed_holds_are_excluded_by_start_speed_window() -> None:
    """A5（120 km/h からの高ブレーキ停車）・A4（19 km/h のトリム階段）を模した保持は、
    実際には停止まで至っていても先頭車速の窓（クリープ平衡付近）に入らないため混入しない。
    """
    a5_high_speed_stop = _hold(
        [120.0, 90.0, 45.0, 5.0, 1.0, 0.0], brake_pct=20.0, session_id="a5"
    )
    a4_trim_stair_stop = _hold(
        [19.0, 15.0, 10.0, 5.0, 1.0, 0.0], brake_pct=20.0, session_id="a4"
    )
    valid_creep_stop = _hold([5.0, 3.0, 1.0, 0.0], brake_pct=14.0, session_id="valid")

    floor = estimate_stop_brake_floor(
        a5_high_speed_stop + a4_trim_stair_stop + valid_creep_stop, PARAMS,
        candidate_openings_pct=(14.0, 20.0),
        start_speed_kmh=5.0, start_tol_kmh=1.0, min_float_s=5.0, opening_tol_pct=0.3,
    )

    # 20.0% は A5・A4 側でしか登場しない（クリープ平衡からの保持ではないので混入しない）
    assert floor.stopped_pct == (14.0,)
    assert floor.offset_pct == pytest.approx(14.0 - PARAMS.brake_deadband_pct)


def test_stepwise_confirm_ratchet_is_too_short_to_count_as_floated() -> None:
    """停止確認の刻み送り（1.0s ごとに開度を上げる）は、各刻みが 1 行しか続かないので
    `min_float_s` 未満となり「浮いた」に数えない。最後に複数行続く本当の保持だけが
    「停止した」に入る。
    """
    ratchet = _hold(
        [5.0, 4.8, 4.6, 4.4, 4.2], brake_pct=0.0, session_id="ratchet",
    )
    # 刻みごとに開度が違う（1 行ずつ）行を作り直す（_hold は brake 一定なので個別に組む）
    ratchet_rows = []
    for row, brake in zip(ratchet, (12.0, 12.5, 13.0, 13.5, 14.0), strict=True):
        row.brake_opening = brake
        ratchet_rows.append(row)
    final_hold = _hold([4.0, 3.0, 2.0, 1.0, 0.0], brake_pct=16.0, session_id="ratchet")
    # 同一セッションとして時刻を連続させる（ratchet の続きに final_hold を繋げる）
    offset = ratchet_rows[-1].timestamp - final_hold[0].timestamp + timedelta(seconds=1.0)
    for row in final_hold:
        row.timestamp = row.timestamp + offset

    floor = estimate_stop_brake_floor(
        ratchet_rows + final_hold, PARAMS,
        candidate_openings_pct=(12.0, 13.0, 14.0, 16.0),
        start_speed_kmh=5.0, start_tol_kmh=1.0, min_float_s=5.0, opening_tol_pct=0.3,
    )

    assert floor.floated_pct == ()  # 刻みの単発一致は floated に入らない
    assert floor.stopped_pct == (16.0,)
    assert floor.offset_pct == pytest.approx(16.0 - PARAMS.brake_deadband_pct)


def test_contradiction_when_floated_max_reaches_stopped_min_is_unidentified() -> None:
    """同じ開度（12.0%）が一方のセッションでは浮き、別のセッションでは止まる（矛盾）場合、
    `max(floated) >= min(stopped)` となり未同定（`offset_pct == 0.0`）になる。
    `stopped_pct`/`floated_pct` は観測どおり返す（点検できるように）。
    """
    floated_run = _hold(
        [5.0, 4.5, 4.0, 3.8, 3.7, 3.65, 3.6, 3.55], brake_pct=12.0, session_id="float_x"
    )
    stopped_run = _hold([5.0, 3.0, 1.0, 0.0], brake_pct=12.0, session_id="stop_x")

    floor = estimate_stop_brake_floor(
        floated_run + stopped_run, PARAMS,
        candidate_openings_pct=(12.0,),
        start_speed_kmh=5.0, start_tol_kmh=1.0, min_float_s=5.0, opening_tol_pct=0.3,
    )

    assert floor.offset_pct == 0.0
    assert not floor.identified
    assert floor.stopped_pct == (12.0,)
    assert floor.floated_pct == (12.0,)


def test_unidentified_when_no_stopped_points() -> None:
    """止まった点が一つも無ければ未同定（`floated_pct` があっても offset は出さない）。"""
    floated_run = _hold(
        [5.0, 4.0, 3.0, 2.0, 1.5, 1.4, 1.35, 1.3], brake_pct=12.0, session_id="float_only"
    )

    floor = estimate_stop_brake_floor(
        floated_run, PARAMS,
        candidate_openings_pct=(12.0,),
        start_speed_kmh=5.0, start_tol_kmh=1.0, min_float_s=5.0, opening_tol_pct=0.3,
    )

    assert floor.offset_pct == 0.0
    assert not floor.identified
    assert floor.stopped_pct == ()
    assert floor.floated_pct == (12.0,)
