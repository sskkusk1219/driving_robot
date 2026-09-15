"""手順 2・3 デバッグのまとめ（debug_process23.py）のユニットテスト。

実走行ログを使わず、合成した行で切り替え・立ち上がり・ガバナー張り付き・脱落・暴走の数え方を確かめる。
"""

from __future__ import annotations

from collections.abc import Sequence

import pytest

from tests.research import debug_process23 as dp


def _row(t: float, v: float, *, phase: str = "ACCEL", accel: float = 0.0, brake: float = 0.0,
         ff: float = 0.0, ref: float = 30.0, current: float = 400.0, gov: bool = False) -> dp.Row:
    return dp.Row(t=t, section=dp.MODE_DRIVE, phase=phase, ref=ref, v=v, accel=accel, brake=brake,
                  ff=ff, brake_current=current, governor=gov)


def _series(speeds: Sequence[float], **kw: object) -> list[dp.Row]:
    return [_row(0.1 * i, v, **kw) for i, v in enumerate(speeds)]  # type: ignore[arg-type]


def test_switch_to_brake_counts_full_stop_and_over_limit() -> None:
    accel = [_row(0.1 * i, 30.0, accel=12.0) for i in range(10)]
    # 1.0s 目でブレーキへ。0.4s で 30→10 km/h（50 km/h/s）、その後停止
    speeds = [30.0, 25.0, 20.0, 15.0, 10.0, 5.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    brake = [_row(1.0 + 0.1 * i, v, phase="BRAKE", accel=8.0, brake=18.0)
             for i, v in enumerate(speeds)]
    events = dp.switch_events([*accel, *brake], to_brake=True)
    assert len(events) == 1
    e = events[0]
    assert e.before_pct == 0.0 and e.after_pct == 18.0
    assert e.peak_kmhs == pytest.approx(50.0)
    assert e.v_min == 0.0
    s = dp.summarize_switches(events)
    assert (s.n, s.full_stops, s.over_limit) == (1, 1, 1)


def test_switch_ignores_low_reference_and_respects_t_max() -> None:
    rows = [_row(0.0, 3.0, ref=3.0, accel=12.0), _row(0.1, 3.0, ref=3.0, phase="BRAKE", brake=15.0),
            _row(0.2, 30.0, accel=12.0), _row(0.3, 30.0, phase="BRAKE_GOV", brake=15.0)]
    assert [e.t for e in dp.switch_events(rows, to_brake=True)] == [pytest.approx(0.3)]
    assert dp.switch_events(rows, to_brake=True, t_max=0.25) == []
    back = dp.switch_events(rows, to_brake=False)  # ブレーキ→アクセル（基準 3 km/h の行は除外済み）
    assert len(back) == 1 and back[0].after_pct == 12.0


def test_onset_ramp_first_opening_and_t90() -> None:
    zeros = [_row(0.1 * i, 40.0) for i in range(3)]
    ramp = [_row(0.3 + 0.1 * i, 40.0 - 0.5 * i, phase="BRAKE", brake=2.0 * (i + 1))
            for i in range(10)]  # 2, 4, …, 20%
    tail = [_row(1.3, 35.0)]
    onsets = dp.onset_ramp([*zeros, *ramp, *tail])
    assert len(onsets) == 1
    o = onsets[0]
    assert o.first_pct == 2.0 and o.peak_pct == 20.0
    assert o.t90_s == pytest.approx(0.8)  # 18% に届くのは 9 行目
    # 待機位置からの立ち上がりは floor で見る
    standby = [_row(0.0, 40.0, brake=11.16), _row(0.1, 40.0, phase="BRAKE", brake=18.0),
               _row(0.2, 30.0, brake=11.16)]
    assert dp.onset_ramp(standby) == []
    assert [o.first_pct for o in dp.onset_ramp(standby, floor_pct=11.16)] == [18.0]


def test_governor_latch_counts_only_floor_while_ff_wants_more() -> None:
    rows = [
        _row(0.0, 30.0, phase="BRAKE", brake=18.0, ff=-18.0),
        _row(0.1, 20.0, phase="BRAKE_GOV", brake=14.0, ff=-18.0, gov=True),
        _row(0.2, 10.0, phase="BRAKE_GOV", brake=11.16, ff=-18.0, gov=True),
        _row(0.3, 5.0, phase="BRAKE_GOV", brake=11.16, ff=-30.0, gov=True),
        _row(0.4, 5.0, phase="ACCEL", accel=10.0, brake=11.16, ff=10.0),
    ]
    eps = dp.governor_episodes(rows, floor_pct=11.16)
    assert len(eps) == 1
    e = eps[0]
    assert e.duration_s == pytest.approx(0.3) and e.latched_s == pytest.approx(0.2)
    assert (e.brake_start, e.brake_end) == (14.0, 11.16)


def test_axis_dropout_needs_one_second_of_zero_current() -> None:
    rows = [_row(0.1 * i, 30.0, phase="BRAKE", brake=16.0, gov=i >= 5) for i in range(10)]
    rows += [_row(1.0 + 0.1 * i, 30.0, phase="BRAKE", brake=0.0, current=0.0) for i in range(3)]
    rows += [_row(1.3, 30.0, brake=0.0, current=400.0)]
    assert dp.axis_dropouts(rows) == []  # 0.2s だけ
    rows += [_row(1.4 + 0.1 * i, 30.0, brake=0.0, current=0.0) for i in range(12)]
    drops = dp.axis_dropouts(rows)
    assert len(drops) == 1 and drops[0].t == pytest.approx(1.4)
    assert drops[0].governor_before  # 直前 1s（0.4〜1.3s）のうち 0.5〜0.9s でガバナーが作動していた
    assert drops[0].brake_max_before == 16.0


def test_runaway_window_first_and_last_acceleration() -> None:
    speeds = [80.0 + 0.1 * i for i in range(20)] + [82.0 + 0.5 * i for i in range(21)]
    rows = _series(speeds, accel=16.0, ref=85.0)
    w = dp.runaway_window(rows, 0.0, 4.0)
    assert w.first_accel_kmhs == pytest.approx(1.0)
    assert w.last_accel_kmhs == pytest.approx(5.0)
    assert (w.accel_min, w.accel_max) == (16.0, 16.0)


def test_search_onsets_first_opening_that_moves_speed() -> None:
    accel = [dp.Row(t=0.1 * i, section="PEDAL_SEARCH", phase="ACCEL_SEARCH", ref=0.0,
                    v=5.0 if i < 5 else 5.4, accel=0.5 * i, brake=0.0, ff=0.0,
                    brake_current=0.0, governor=False) for i in range(8)]
    brake = [dp.Row(t=1.0 + 0.1 * i, section="PEDAL_SEARCH", phase="BRAKE_SEARCH", ref=0.0,
                    v=5.0 if i < 3 else 4.5, accel=0.0, brake=4.0 * i, ff=0.0,
                    brake_current=0.0, governor=False) for i in range(5)]
    assert dp.search_onsets([*accel, *brake]) == {"accel": 2.5, "brake": 12.0}


def test_b3_clamped_rows_counts_rows_exactly_at_deadband() -> None:
    rows = [_row(0.0, 30.0, ff=-13.16), _row(0.1, 30.0, ff=-17.5),
            _row(0.2, 0.0, ref=0.0, ff=-13.16),
            _row(0.3, 30.0, ff=12.0)]
    assert dp.b3_clamped_rows(rows, 13.16) == (2, 1)
