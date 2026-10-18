"""C12（惰行の補助）の入る・離すの判断（coast_assist.CoastAssist）と候補クラスのテスト。
2026-10-18 段2r。合成の小さな plan の配列だけを使う。"""

from __future__ import annotations

import numpy as np
import pytest

from tests.research.coast_assist import CoastAssist, CoastStats, coast_settings_text
from tests.research.dob import PEDAL_ACCEL, PEDAL_BRAKE
from tests.research.ff_candidate import (
    CoastAssistFeedforward,
    DisturbanceObserverFeedforward,
    make_candidate,
)
from tests.research.fwd_plan import (
    FLAG_COAST,
    FLAG_MEASURED,
    FLAG_STOP,
    ReturnTime,
)

DT = 0.1
ON, OFF = 0.5, 0.2
HOLD = 1.0
# 戻り時間 T ＝ 0.1 ＋ 深さ ÷ 100 ＋ 0.1（アクセル min 4・ブレーキ min 7）
RET = ReturnTime(
    accel_min_pct=4.0, brake_min_pct=7.0,
    accel_delay_s=0.1, accel_rate_pct_s=100.0, brake_delay_s=0.1, brake_rate_pct_s=100.0,
)
ACCEL_ONSET, BRAKE_ONSET = 6.0, -9.0  # 効き始めの絶対開度（T は 0.22 s・0.22 s）


def _assist(effort: list[float], flag: list[int]) -> CoastAssist:
    return CoastAssist(
        np.array(effort), np.array(flag), np.arange(len(effort)) * DT,
        on_kmh=ON, off_kmh=OFF, hold_accel_s=HOLD, hold_brake_s=HOLD, ret=RET, period_s=DT,
        accel_onset_effort=ACCEL_ONSET, brake_onset_effort=BRAKE_ONSET,
    )


def _all_coast(n: int = 100) -> CoastAssist:
    return _assist([0.0] * n, [FLAG_COAST] * n)


def _step(ca: CoastAssist, i: int, dev: float, speed: float = 30.0) -> str | None:
    return ca.step(t=i * DT, i=i, dev_kmh=dev, speed_kmh=speed, ref_kmh=speed + dev)


def test_enters_on_threshold_with_pedal_by_sign() -> None:
    ca = _all_coast()
    assert _step(ca, 0, 0.49) is None
    assert _step(ca, 1, 0.5) == PEDAL_ACCEL  # 遅れている → アクセル
    assert ca.stats.accel_entries == 1
    cb = _all_coast()
    assert _step(cb, 0, -0.6) == PEDAL_BRAKE  # 速すぎる → ブレーキ
    assert cb.stats.brake_entries == 1


def test_holds_until_hold_time_then_releases_at_off() -> None:
    ca = _all_coast()
    assert _step(ca, 0, 0.6) == PEDAL_ACCEL
    # 偏差が off 以下に戻っても、保持時間の前には離さない
    for i in range(1, 10):
        assert _step(ca, i, 0.1) == PEDAL_ACCEL
    # 保持が済んだ（t=1.0）周期で off 以下 → 離す
    assert _step(ca, 10, 0.1) is None
    assert ca.stats.accel_s == pytest.approx(10 * DT)


def test_keeps_pushing_after_hold_while_deviation_above_off() -> None:
    ca = _all_coast()
    _step(ca, 0, 0.6)
    for i in range(1, 30):
        assert _step(ca, i, 0.4) == PEDAL_ACCEL  # off(0.2) より大きいうちは保持し続ける


def test_releases_when_sign_flips_after_hold() -> None:
    ca = _all_coast()
    _step(ca, 0, 0.6)
    for i in range(1, 10):
        _step(ca, i, 0.6)
    assert _step(ca, 10, -0.3) is None  # 符号が逆（行き過ぎ）→ 離す


def test_one_pedal_per_coast_section_and_cleared_on_non_coast_row() -> None:
    flag = [FLAG_COAST] * 40 + [FLAG_MEASURED] * 3 + [FLAG_COAST] * 40
    effort = [0.0] * 40 + [10.0] * 3 + [0.0] * 40
    ca = _assist(effort, flag)
    _step(ca, 0, 0.6)
    for i in range(1, 11):
        _step(ca, i, 0.1)  # 10 で離す
    assert ca.pedal is None
    # 保持が過ぎても、同じ区間では反対のペダルを使わない
    assert _step(ca, 25, -0.8) is None
    assert ca.stats.blocked_section == 1
    # 反対でなければ入り直せる
    assert _step(ca, 26, 0.8) == PEDAL_ACCEL
    for i in range(27, 40):
        _step(ca, i, 0.8)
    # 踏む行（惰行でない行）に来ると区間の記憶が消え、次の惰行の区間ではブレーキも使える
    _step(ca, 40, 0.0)
    assert ca.section_pedal is None
    assert _step(ca, 55, -0.8) == PEDAL_BRAKE  # 前のアクセルが戻り、保持も済んだ後


def test_no_reentry_before_hold_after_release() -> None:
    ca = _all_coast()
    _step(ca, 0, 0.6)
    for i in range(1, 11):
        _step(ca, i, 0.1)  # t=1.0 で離す
    assert _step(ca, 11, 0.8) is None  # 離した直後（0.1 s）には入り直さない
    assert ca.stats.blocked_reentry == 1
    assert _step(ca, 20, 0.8) == PEDAL_ACCEL  # hold たった


def test_waits_for_previous_opposite_pedal_to_return() -> None:
    # 行 0〜4 がアクセル 24%（終わりは 0.5 s。T ＝ 0.1 ＋ 20/100 ＋ 0.1 ＝ 0.4 s）、そのあと惰行
    ca = _assist([24.0] * 5 + [0.0] * 30, [FLAG_MEASURED] * 5 + [FLAG_COAST] * 30)
    assert _step(ca, 5, -0.8) is None
    assert _step(ca, 8, -0.8) is None  # 0.8 − 0.5 ＝ 0.3 s ＜ 0.4 s
    assert ca.stats.blocked_prev_return == 2
    assert _step(ca, 10, -0.8) == PEDAL_BRAKE  # 1.0 − 0.5 ＝ 0.5 s ≥ 0.4 s
    # 前が同じペダルなら待たない
    cb = _assist([24.0] * 5 + [0.0] * 30, [FLAG_MEASURED] * 5 + [FLAG_COAST] * 30)
    assert _step(cb, 5, 0.8) == PEDAL_ACCEL


def test_stop_hold_counts_as_previous_brake_row() -> None:
    ca = _assist([-18.0] * 5 + [0.0] * 30, [FLAG_STOP] * 5 + [FLAG_COAST] * 30)
    assert _step(ca, 5, 0.8) is None  # 停車保持のブレーキが戻るまで待つ
    assert ca.stats.blocked_prev_return == 1


def test_does_not_enter_when_next_opposite_pedal_is_too_close() -> None:
    # 惰行 0〜29、行 30 からブレーキ（始まり 3.0 s）。入るには 3.0 − t ≥ 1.0 ＋ 0.22
    ca = _assist([0.0] * 30 + [-20.0] * 10, [FLAG_COAST] * 30 + [FLAG_MEASURED] * 10)
    assert _step(ca, 19, 0.8) is None  # 残り 1.1 s
    assert ca.stats.blocked_next_return == 1
    assert _step(ca, 17, 0.8) == PEDAL_ACCEL  # 残り 1.3 s
    # 次が同じペダルなら条件なし
    cs = _assist([0.0] * 30 + [20.0] * 10, [FLAG_COAST] * 30 + [FLAG_MEASURED] * 10)
    assert _step(cs, 29, 0.8) == PEDAL_ACCEL


def test_releases_before_next_opposite_pedal_even_during_hold() -> None:
    # 惰行 0〜29、行 30（3.0 s）からブレーキ。t=1.2 で入る → 保持（〜2.2 s）の途中でも、
    # アクセル 24% が戻る時間 T ＝ 0.4 s を残して離す
    ca = _assist([0.0] * 30 + [-20.0] * 10, [FLAG_COAST] * 30 + [FLAG_MEASURED] * 10)
    assert _step(ca, 12, 0.8) == PEDAL_ACCEL
    ca.note_effort(24.0)
    for i in range(13, 25):
        assert _step(ca, i, 0.8) == PEDAL_ACCEL  # 2.4 s: 残り 0.6 s
        ca.note_effort(24.0)
    assert _step(ca, 25, 0.8) == PEDAL_ACCEL  # 2.5 s: 残り 0.5 s ＞ 0.4 s
    assert _step(ca, 27, 0.8) is None  # 2.7 s: 残り 0.3 s ≤ 0.4 s
    assert ca.stats.released_next == 1


def test_none_on_non_coast_rows_and_when_stopped_or_slow() -> None:
    ca = _assist([10.0] * 10 + [0.0] * 10, [FLAG_MEASURED] * 10 + [FLAG_COAST] * 10)
    assert _step(ca, 3, 2.0) is None  # 惰行の行でない
    assert _step(ca, 12, 2.0, speed=0.0) is None  # 実車速が停車速度以下
    assert ca.step(t=1.2, i=12, dev_kmh=-1.0, speed_kmh=30.0, ref_kmh=0.0) is None  # 基準が停車
    assert _step(ca, 13, 2.0) == PEDAL_ACCEL
    # 補助中に惰行の行でなくなる／止まる → 離して None
    assert _step(ca, 14, 2.0, speed=0.0) is None and ca.pedal is None


def test_release_on_leaving_coast_sets_t_off() -> None:
    ca = _assist([0.0] * 5 + [10.0] * 5, [FLAG_COAST] * 5 + [FLAG_MEASURED] * 5)
    _step(ca, 0, 0.8)
    assert _step(ca, 5, 0.8) is None
    assert ca.t_off == pytest.approx(0.5)


def test_stats_summary_and_settings_text() -> None:
    st = CoastStats(period_s=0.05, accel_entries=2, accel_s=3.0, blocked_section=20)
    assert "アクセル 2 回（3.0s）" in st.summary() and "1.0s" in st.summary()
    text = coast_settings_text(0.5, 0.2, 4.0, 1.5, 2.0)
    assert "入る 0.5 km/h" in text and "保持 アクセル 1.50s / ブレーキ 2.00s" in text


def test_make_candidate_c12_and_set_coast() -> None:
    ff = make_candidate("C12")
    assert isinstance(ff, CoastAssistFeedforward)
    assert isinstance(ff, DisturbanceObserverFeedforward)  # C11 の DOB をそのまま使う
    assert ff.candidate == "C12" and ff.expected_kind == "model"
    ff.set_coast(on_kmh=0.5, off_kmh=0.2, hold_coef=4.0)
    assert (ff.coast_on_kmh, ff.coast_off_kmh, ff.coast_hold_coef) == (0.5, 0.2, 4.0)
