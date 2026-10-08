"""ff_bias（作ったずれの集計）のテスト。手で作った行だけを使う。"""

from __future__ import annotations

from datetime import UTC, datetime, timedelta

import numpy as np
import pytest

from tests.research.ff_bias import (
    CHOICE_BRAKE,
    CHOICE_COAST,
    CHOICE_OK,
    MAX_SHIFT_S,
    PEDAL_ACCEL,
    PEDAL_BRAKE,
    PEDAL_COAST,
    BrakeRows,
    BrakeSegment,
    OpeningStat,
    Recomputed,
    RecomputeDrive,
    RecomputeSetting,
    accel_row_diff,
    backoff_bin_index,
    bin_index,
    brake_effect,
    classify_pedal_choice,
    compute_created,
    decel_bin_label,
    effect_cell,
    find_brake_segments,
    main,
    motion_index,
    opening_error_pct,
    opening_errors,
    pair_timing,
    peak_crossings,
    pedal_choice,
    predict_on_trajectory,
    recompute_commands,
    recompute_settings,
    ref_slopes,
    release_timings,
    segment_labels,
    shift_forward,
    switch_timings,
    top_contributors,
    training_sets,
    window_slope,
    xcorr_shift,
)
from tests.research.ff_candidate import CandidateFeedforward
from tests.research.ff_model import FeatureSpec, build_feature_row
from tests.research.ff_params import ResearchFFParams
from tests.research.mode_report import STATE_CRUISE, STATE_DECEL, ModeRow
from tests.research.research_types import DriveLog, FeedforwardParams


def _row(t: float, ref: float, dev: float, phase: str) -> ModeRow:
    return ModeRow(
        t_s=t, ref_kmh=ref, actual_kmh=ref + dev, accel_pct=0.0, brake_pct=0.0,
        ff_effort_pct=0.0, pid_effort_pct=0.0, effort_pct=0.0, segment="", phase=phase,
    )


def _decel_rows(dt: float, devs: list[float], phases: list[str]) -> list[ModeRow]:
    """基準が 60 から 2 km/h/s で下がり続ける行（全行が 減速）。"""
    return [
        _row(i * dt, 60.0 - 2.0 * i * dt, d, ph)
        for i, (d, ph) in enumerate(zip(devs, phases, strict=True))
    ]


def test_created_sums_per_category_and_ignores_entrance_deviation():
    # 入口の偏差 +5 は数えない。増分: 0→+1(ACCEL), +1→+3(ACCEL), +3→+2(BRAKE), 最終行は数えない
    phases = [PEDAL_ACCEL, PEDAL_ACCEL, PEDAL_BRAKE, PEDAL_BRAKE, PEDAL_COAST]
    rows = _decel_rows(0.1, [5.0, 6.0, 8.0, 7.0, 7.0], phases)
    rep = compute_created(rows)
    acc = rep.state_pedal[(STATE_DECEL, PEDAL_ACCEL)]
    brk = rep.state_pedal[(STATE_DECEL, PEDAL_BRAKE)]
    assert acc.created_kmh == pytest.approx(3.0)
    assert brk.created_kmh == pytest.approx(-1.0)
    assert (STATE_DECEL, PEDAL_COAST) not in rep.state_pedal  # 最終行は増分なし


def test_rate_uses_dt():
    rows = _decel_rows(0.5, [0.0, 1.0, 2.0, 3.0], [PEDAL_ACCEL] * 4)
    c = compute_created(rows).state_pedal[(STATE_DECEL, PEDAL_ACCEL)]
    assert c.time_s == pytest.approx(1.5)  # 3 行 × 0.5s
    assert c.created_kmh == pytest.approx(3.0)
    assert c.per_s == pytest.approx(2.0)


def test_decel_bin_assignment():
    assert decel_bin_label(0.2) is None
    assert decel_bin_label(0.3) == "0.3〜1.5"
    assert decel_bin_label(1.5) == "1.5〜3"
    assert decel_bin_label(2.9) == "1.5〜3"
    assert decel_bin_label(4.0) == "3〜5"
    assert decel_bin_label(5.0) == "5以上"
    assert decel_bin_label(9.0) == "5以上"


def test_brake_bin_and_accel_band_tables():
    # 両端は傾きが端で打ち切られるので、中ほど（行 2〜5）だけを見る。dt=0.5s で傾き -2 km/h/s
    phases = [PEDAL_COAST, PEDAL_COAST, PEDAL_BRAKE, PEDAL_BRAKE, PEDAL_ACCEL, PEDAL_ACCEL,
              PEDAL_COAST, PEDAL_COAST]
    rows = _decel_rows(0.5, [0.0, 0.0, 0.0, 0.2, 0.4, 0.7, 0.7, 0.7], phases)
    rep = compute_created(rows)
    assert ref_slopes(rows)[3] == pytest.approx(-2.0)
    assert rep.decel_brake_bins["1.5〜3"].created_kmh == pytest.approx(0.4)
    assert rep.decel_accel_bands["40〜60"].created_kmh == pytest.approx(0.3)


def test_positive_share_of_over_limit_rows():
    devs = [2.0, 3.0, -2.0, 0.5, 1.0, -0.2]  # |e|>1: 2.0, 3.0, -2.0（1.0 ちょうどは含めない）
    rows = [_row(i * 0.1, 50.0, d, PEDAL_COAST) for i, d in enumerate(devs)]
    rep = compute_created(rows)
    assert rep.over_count == 3
    assert rep.over_positive_count == 2
    assert rep.positive_share == pytest.approx(2 / 3)
    assert (STATE_CRUISE, PEDAL_COAST) in rep.state_pedal


# ── --part cause ──────────────────────────────────────────────────────


def test_window_slope_linear_and_edges():
    t = np.arange(0.0, 2.0, 0.1)
    y = 3.0 - 2.0 * t
    out = window_slope(t, y, np.array([1.0, 0.05, 1.98]), 0.4)
    assert out[0] == pytest.approx(-2.0)
    assert np.isnan(out[1])  # 窓が先頭からはみ出す
    assert np.isnan(out[2])  # 窓が末尾からはみ出す


def test_opening_error_conversion_with_known_gain():
    assert opening_error_pct(1.0, 0.5) == pytest.approx(2.0)
    assert opening_error_pct(-0.3, 0.25) == pytest.approx(-1.2)


def _kink_rows(pedal_at: dict[int, str]) -> list[ModeRow]:
    """基準は −2 km/h/s（59→41）。実車速は t=5.0 s 以降だけ傾き −1（減速が足りない）。"""
    rows = []
    for i in range(91):
        t = i * 0.1
        ref = 59.0 - 2.0 * t
        actual = ref + (t - 5.0 if t > 5.0 else 0.0)
        rows.append(_row(t, ref, actual - ref, pedal_at.get(i, PEDAL_COAST)))
    return rows


_GAINS = FeedforwardParams(
    pedal_gain_speeds_kmh=(0.0, 200.0),
    accel_gain_kmhs_per_pct=(0.5, 0.5),
    brake_gain_kmhs_per_pct=(0.25, 0.25),
)


def test_opening_error_uses_pedal_lag_shift():
    # t=4.8 の ACCEL: 窓の中心は t+0.5=5.3（傾き −1 の側）→ 誤差 +1 → 1/0.5 = +2%。
    # 遅れなしなら窓 [4.6,5.0] は傾き −2 の側で誤差 0 になる
    (st,) = opening_errors(_kink_rows({48: PEDAL_ACCEL}), _GAINS).values()
    assert st.median_pct == pytest.approx(2.0, abs=1e-6)
    assert st.mean_err_kmhs == pytest.approx(1.0, abs=1e-6)
    # t=5.0 の BRAKE: 窓の中心は t+0.3=5.3 → 誤差 +1 → 1/0.25 = +4%
    (st,) = opening_errors(_kink_rows({50: PEDAL_BRAKE}), _GAINS).values()
    assert st.median_pct == pytest.approx(4.0, abs=1e-6)


def _train_logs(accel: list[float], brake: list[float]) -> list[DriveLog]:
    origin = datetime(2000, 1, 1, tzinfo=UTC)
    return [
        DriveLog(
            id=i, session_id="s", timestamp=origin + timedelta(seconds=0.1 * i),
            ref_speed_kmh=None, actual_speed_kmh=30.0 + 0.1 * i, accel_opening=a,
            brake_opening=b, accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
        )
        for i, (a, b) in enumerate(zip(accel, brake, strict=True))
    ]


_SMALL_SPEC = FeatureSpec(
    lookahead_horizons_s=(1.0,), past_horizons_s=(), regime_horizon_s=1.0,
    include_v0_sq=False, include_dv_regime_x_v0=False,
)


def test_training_rows_exclude_below_deadband():
    # 60 行。先頭 5 行は 0%、次の 10 行は 10%、1 行は不感帯の直下 2.9%、残りは 0%。
    accel = [0.0] * 5 + [10.0] * 10 + [2.9] + [0.0] * 44
    brake = [0.0] * 60
    sets = training_sets(_train_logs(accel, brake), _SMALL_SPEC, _SMALL_SPEC, 3.0, 3.0)
    assert len(sets[PEDAL_BRAKE].label_pct) == 0  # ブレーキは効いている行が無い
    ts = sets[PEDAL_ACCEL]
    assert len(ts.label_pct) == 10
    assert np.all(ts.label_pct >= 3.0)
    assert ts.x.shape == (10, 2)  # v0 と dv_1.0


def test_pedal_choice_classification():
    # 惰行 −1.0、帯 0.5 の例
    assert classify_pedal_choice(-2.0, -1.0, 0.5, PEDAL_ACCEL) == CHOICE_BRAKE
    assert classify_pedal_choice(-1.2, -1.0, 0.5, PEDAL_ACCEL) == CHOICE_COAST
    assert classify_pedal_choice(0.5, -1.0, 0.5, PEDAL_ACCEL) == CHOICE_OK
    assert classify_pedal_choice(-2.0, -1.0, 0.5, PEDAL_BRAKE) == CHOICE_OK
    assert classify_pedal_choice(-1.2, -1.0, 0.5, PEDAL_BRAKE) == CHOICE_COAST
    assert classify_pedal_choice(0.0, -1.0, 0.5, PEDAL_BRAKE) == CHOICE_COAST


def test_pedal_choice_counts_only_strong_decel_accel_rows():
    # 既定の惰行減速は −1.0 km/h/s。基準が −2 km/h/s（惰行より強い）の ACCEL 行は全部
    # 「ブレーキが妥当」、基準が −1.05 km/h/s（帯 0.2 の内）の ACCEL 行は全部「惰行が妥当」
    research = ResearchFFParams(coast_band_kmhs=0.2)
    params = FeedforwardParams()
    strong = [_row(i * 0.1, 80.0 - 2.0 * i * 0.1, 0.0, PEDAL_ACCEL) for i in range(100)]
    st = pedal_choice(strong, params, research)[(PEDAL_ACCEL, "60〜80")]
    assert st.brake_s == pytest.approx(st.time_s) and st.coast_s == 0.0
    mild = [_row(i * 0.1, 80.0 - 1.05 * i * 0.1, 0.0, PEDAL_ACCEL) for i in range(100)]
    st = pedal_choice(mild, params, research)[(PEDAL_ACCEL, "60〜80")]
    assert st.coast_s == pytest.approx(st.time_s) and st.brake_s == 0.0


def test_predict_on_trajectory_uses_shared_feature_builder():
    speed = np.linspace(20.0, 50.0, 50)
    spec = FeatureSpec(
        lookahead_horizons_s=(0.5, 1.0), past_horizons_s=(0.5,), regime_horizon_s=1.0
    )
    seen: list[np.ndarray] = []

    class _Model:
        def predict(self, x: np.ndarray) -> np.ndarray:
            seen.append(x)
            return x[:, 0]  # v0 をそのまま返す

    out = predict_on_trajectory(speed, 0.1, spec, _Model())
    # 有効行: 先頭 5（過去 0.5s）・末尾 10（先読み 1.0s）は NaN
    assert np.isnan(out[:5]).all() and np.isnan(out[-10:]).all()
    i = 20
    expected = build_feature_row(
        speed[i], [speed[i + 5], speed[i + 10]], [speed[i - 5]], spec
    )
    np.testing.assert_allclose(seen[0][i - 5], expected[0])
    assert out[i] == pytest.approx(speed[i])


def test_top_contributors_ranks_by_time_times_mean_err_both_signs():
    def st(t: float, e: float) -> OpeningStat:
        return OpeningStat(t, 0.0, 0.0, e)

    stats = {
        ("BRAKE", "20〜40", "3〜5"): st(39.0, 0.64),   # +25.0
        ("ACCEL", "60〜80", "0.3〜1.5"): st(59.0, -0.11),  # -6.5（負も数える）
        ("ACCEL", "0〜20", "0.3〜1.5"): st(43.0, 0.10),  # +4.3
        ("ACCEL", "80〜100", "1.5〜3"): st(0.5, 9.0),  # 1s 未満は除く
        ("BRAKE", "40〜60", "5以上"): st(10.0, 0.01),  # +0.1
    }
    assert top_contributors(stats, cover=0.6, max_n=10) == [("BRAKE", "20〜40", "3〜5")]
    assert top_contributors(stats, cover=0.95, max_n=10) == [
        ("BRAKE", "20〜40", "3〜5"), ("ACCEL", "60〜80", "0.3〜1.5"), ("ACCEL", "0〜20", "0.3〜1.5")
    ]
    assert len(top_contributors(stats, cover=1.0, max_n=2)) == 2


def _phases(spec: list[tuple[str, int]]) -> list[str]:
    return [p for p, n in spec for _ in range(n)]


def test_find_brake_segments_mid_vs_stop_and_short_dropped():
    # 0.1s 刻み。[COAST 10][BRAKE 30 (3.0s) 途中まで][COAST 10][BRAKE 10 (1.0s) 短い]
    # [COAST 5][BRAKE 30 (3.0s) 基準が 1 km/h 未満へ][COAST 5]
    phase = _phases([
        (PEDAL_COAST, 10), (PEDAL_BRAKE, 30), (PEDAL_COAST, 10), (PEDAL_BRAKE, 10),
        (PEDAL_COAST, 5), (PEDAL_BRAKE, 30), (PEDAL_COAST, 5),
    ])
    n = len(phase)
    t = [0.1 * i for i in range(n)]
    ref = [50.0] * n
    # 停車まで: 最後の BRAKE 区間の途中から基準を 0.5 km/h 未満にする
    stop_start = 10 + 30 + 10 + 10 + 5
    for i in range(stop_start + 25, n):
        ref[i] = 0.5
    segs = find_brake_segments(phase, ref, t)
    assert [(s.i0, s.i1, s.to_stop) for s in segs] == [
        (10, 39, False),
        (stop_start, stop_start + 24, True),
    ]


def test_find_brake_segments_ignores_low_speed_start():
    phase = _phases([(PEDAL_COAST, 5), (PEDAL_BRAKE, 40)])
    t = [0.1 * i for i in range(len(phase))]
    assert find_brake_segments(phase, [4.0] * len(phase), t) == []


def _trapezoid(t: np.ndarray, rise: float, fall: float, width: float = 0.4) -> np.ndarray:
    """高さ 10 の台形。rise で 50% を越え、fall で 50% を下回る。"""
    up = np.clip((t - (rise - width / 2)) / width, 0.0, 1.0)
    down = np.clip(((fall + width / 2) - t) / width, 0.0, 1.0)
    return 10.0 * np.minimum(up, down)


def test_peak_crossings_gives_known_onset_and_release():
    t = np.arange(0.0, 10.0, 0.05)
    got = peak_crossings(t, _trapezoid(t, 3.0, 6.5))
    assert got is not None
    assert got == (pytest.approx(3.0, abs=1e-6), pytest.approx(6.5, abs=1e-6))


def test_peak_crossings_none_when_never_below_half():
    t = np.arange(0.0, 5.0, 0.05)
    assert peak_crossings(t, np.full_like(t, 5.0)) is None
    assert peak_crossings(t, np.zeros_like(t)) is None


def test_pair_timing_sign_later_minus_earlier():
    t = np.arange(0.0, 10.0, 0.05)
    a = _trapezoid(t, 3.0, 6.0)
    b = _trapezoid(t, 3.6, 6.3)
    got = pair_timing(t, a, b)
    assert got.onset_s == pytest.approx(0.6, abs=1e-6)
    assert got.release_s == pytest.approx(0.3, abs=1e-6)
    assert got.xcorr_s is not None and got.xcorr_s > 0


def test_xcorr_shift_recovers_known_shift():
    t = np.arange(0.0, 10.0, 0.05)
    a = _trapezoid(t, 4.0, 6.0)
    b = _trapezoid(t, 4.3, 6.3)  # 0.3 s 遅い
    got = xcorr_shift(t, a, b)
    assert got is not None
    shift, edge = got
    assert shift == pytest.approx(0.3, abs=0.051)
    assert not edge
    back = xcorr_shift(t, b, a)
    assert back is not None and back[0] == pytest.approx(-0.3, abs=0.051)


def test_xcorr_shift_flags_edge_when_shift_exceeds_range():
    t = np.arange(0.0, 12.0, 0.05)
    a = _trapezoid(t, 4.0, 6.0)
    b = _trapezoid(t, 6.0, 8.0)  # 2.0 s 遅い（範囲 1.5 s を超える）
    got = xcorr_shift(t, a, b)
    assert got is not None
    shift, edge = got
    assert edge
    assert shift == pytest.approx(MAX_SHIFT_S, abs=0.051)


# ── 指令の再計算（--part recompute） ───────────────────────────────────


class _NegDvBrake:
    """ブレーキ予測 = −10 × 最短ホライズンの dv（入力の時刻がずれると値もずれる）。"""

    def predict(self, x: np.ndarray) -> np.ndarray:
        return -10.0 * x[:, 1]


class _ConstAccel:
    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.full(len(x), 20.0)


def _recompute_ff() -> CandidateFeedforward:
    ff = CandidateFeedforward()
    ff.set_params(FeedforwardParams(
        creep_speed_kmh=4.944, creep_rate_kmhs=0.155, stop_brake_opening_pct=30.53,
        coast_decel_speeds_kmh=(5.0, 135.0), coast_decel_kmhs=(1.6, 1.6),
        accel_deadband_pct=10.0, brake_deadband_pct=13.68,
    ))
    ff.set_research_params(ResearchFFParams(coast_band_kmhs=0.2))
    ff._accel_model = _ConstAccel()  # noqa: SLF001 - テスト用の差し込み
    ff._brake_model = _NegDvBrake()  # noqa: SLF001
    ff._speed_clip_max = None  # noqa: SLF001
    return ff


_DRIVE = RecomputeDrive("window", 1.0, 80.0, 80.0, 0.0, 0.0)


def _ramp_ref() -> tuple[np.ndarray, np.ndarray]:
    """60 km/h 一定 → t=5 から 6 km/h/s で減速 → t=10 で 30 km/h 一定（0.1s 刻み）。"""
    t = np.arange(0.0, 15.0, 0.1)
    return t, np.interp(t, [0.0, 5.0, 10.0, 15.0], [60.0, 60.0, 30.0, 30.0])


def test_recompute_delay_shifts_brake_command_later_by_d():
    ff = _recompute_ff()
    t, ref = _ramp_ref()
    base = RecomputeSetting("b", 0.0, 0.3, 1.0)
    r0 = recompute_commands(ff, t, ref, _DRIVE, base)
    r3 = recompute_commands(ff, t, ref, _DRIVE, RecomputeSetting("d", 0.3, 0.3, 1.0))
    # ペダル選択は遅らせの影響を受けない
    assert r0.phase == r3.phase
    assert PEDAL_BRAKE in r0.phase
    # ブレーキ指令は d=0.3s（3 行）後ろへずれる: r3[i] == r0[i-3]（両方ブレーキの行）
    both = [
        i for i in range(3, len(t))
        if r3.phase[i] == PEDAL_BRAKE and r0.phase[i - 3] == PEDAL_BRAKE
    ]
    assert both
    np.testing.assert_allclose(r3.brake_ff_pct[both], r0.brake_ff_pct[[i - 3 for i in both]])
    assert not np.allclose(r3.brake_ff_pct, r0.brake_ff_pct)  # 自明でないこと


def test_recompute_baseline_delay_zero_is_plain_predict_effort():
    ff = _recompute_ff()
    t, ref = _ramp_ref()
    r = recompute_commands(ff, t, ref, _DRIVE, RecomputeSetting("b", 0.0, 0.3, 1.0))
    i = 70  # 減速中
    hs, phs = ff.horizons, ff.past_horizons
    v0 = ref[i]
    fut = [float(np.interp(t[i] + h, t, ref)) for h in hs]
    pst = [float(np.interp(t[i] - h, t, ref)) for h in phs]
    eff = ff.predict_effort(
        v0, fut, pst, select_accel_kmhs=-6.0, select_speed_kmh=float(np.interp(t[i] + 0.3, t, ref))
    )
    assert r.brake_ff_pct[i] == pytest.approx(-eff)


def test_recompute_standby_floor_keeps_ff_unchanged():
    ff = _recompute_ff()
    t, ref = _ramp_ref()
    drive = RecomputeDrive("window", 1.0, 80.0, 80.0, 8.0, 11.0)
    r = recompute_commands(ff, t, ref, drive, RecomputeSetting("b", 0.0, 0.3, 1.0))
    assert (r.brake_cmd_pct >= 11.0).all() and (r.accel_cmd_pct >= 8.0).all()
    np.testing.assert_array_equal(r.brake_cmd_pct, np.maximum(r.brake_ff_pct, 11.0))


def test_recompute_settings_change_one_at_a_time():
    base = RecomputeSetting("x", 0.0, 0.3, 3.0)
    ss = recompute_settings(base)
    assert ss[0] == RecomputeSetting("ベースライン", 0.0, 0.3, 3.0)
    for s in ss[1:]:
        changed = sum(
            a != b for a, b in zip(
                (s.delay_s, s.center_s, s.width_s, s.brake_center_s),
                (0.0, 0.3, 3.0, None), strict=True,
            )
        )
        assert changed == 1
    assert len(ss) == 1 + 5 + 3 + 2 + 3
    assert [s.brake_center_s for s in ss[-3:]] == [0.2, 0.1, 0.0]


def test_recompute_brake_center_keeps_accel_rows_and_shifts_brake_later():
    ff = _recompute_ff()
    t, ref = _ramp_ref()
    base = recompute_commands(ff, t, ref, _DRIVE, RecomputeSetting("b", 0.0, 0.3, 1.0))
    same = recompute_commands(ff, t, ref, _DRIVE, RecomputeSetting("l", 0.0, 0.3, 1.0, 0.3))
    assert same.phase == base.phase  # L_b = L は従来と同じ
    np.testing.assert_array_equal(same.brake_ff_pct, base.brake_ff_pct)
    lb = recompute_commands(ff, t, ref, _DRIVE, RecomputeSetting("l", 0.0, 0.3, 1.0, 0.0))
    assert accel_row_diff(lb, base) == 0  # アクセルの行は 1 行も変わらない
    b0 = [i for i, p in enumerate(base.phase) if p == PEDAL_BRAKE]
    b1 = [i for i, p in enumerate(lb.phase) if p == PEDAL_BRAKE]
    assert b0 and b1 and b1[0] > b0[0]  # ブレーキの入りは後ろへ
    # point 方式では L_b は無視される（何も足さない）
    drive_pt = RecomputeDrive("point", 1.0, 80.0, 80.0, 0.0, 0.0)
    p0 = recompute_commands(ff, t, ref, drive_pt, RecomputeSetting("b", 0.0, 0.3, 1.0))
    p1 = recompute_commands(ff, t, ref, drive_pt, RecomputeSetting("b", 0.0, 0.3, 1.0, 0.0))
    assert p0.phase == p1.phase


def test_accel_row_diff_counts_phase_or_value_changes():
    n = 4
    mk = lambda ph, a: Recomputed(  # noqa: E731
        np.array(a, float), np.zeros(n), np.array(a, float), np.zeros(n), ph
    )
    base = mk([PEDAL_COAST, PEDAL_ACCEL, PEDAL_ACCEL, PEDAL_BRAKE], [0, 10, 10, 0])
    assert accel_row_diff(base, base) == 0
    other = mk([PEDAL_ACCEL, PEDAL_ACCEL, PEDAL_ACCEL, PEDAL_BRAKE], [5, 10, 10.1, 0])
    assert accel_row_diff(other, base) == 2  # 行 0（phase と値）・行 2（値）


def test_release_timings_sign_and_switch_timings_unchanged():
    """選択が BRAKE を抜けた時刻 − 基準の減速の離し 50%。+ は選択のほうが先に離す。"""
    t = np.arange(0.0, 20.0, 0.1)
    ref = np.interp(t, [0.0, 5.0, 10.0, 20.0], [60.0, 60.0, 40.0, 40.0])  # 5〜10s が減速
    log_phase = [PEDAL_BRAKE if 5.0 <= x < 10.0 else PEDAL_COAST for x in t]
    rows = BrakeRows(
        t_s=t, ref_kmh=ref, actual_kmh=ref.copy(), phase=log_phase,
        brake_cmd_pct=np.zeros(len(t)), brake_actual_pct=np.zeros(len(t)),
    )
    # 再計算したペダル: 4.0〜9.0s が BRAKE（基準の減速より 1s 早く入り、1s 早く離す）
    early = [PEDAL_BRAKE if 4.0 <= x < 9.0 else PEDAL_COAST for x in t]
    late = [PEDAL_BRAKE if 5.5 <= x < 10.5 else PEDAL_COAST for x in t]
    (r_early,) = release_timings(rows, early)
    (r_late,) = release_timings(rows, late)
    (s_early,) = switch_timings(rows, early)
    assert r_early is not None and r_late is not None and s_early is not None
    assert r_early == pytest.approx(1.0, abs=0.15)  # 選択のほうが 1s 先に離す（+）
    assert r_late == pytest.approx(-0.5, abs=0.15)  # 選択のほうが 0.5s 遅く離す（−）
    assert s_early == pytest.approx(1.0, abs=0.15)  # 入りも同じ向き（+ が選択のほうが先）
    # 切替後に離さない（窓の終わりまで BRAKE）なら None
    assert release_timings(rows, [PEDAL_BRAKE] * len(t)) == [None]


def test_segment_labels_backoff_thirds_and_post_release():
    """山の後に下がる波形: 戻した量 = それまでの最大 − 今。3 分割と離した後が付く。"""
    t = np.arange(0.0, 12.0, 0.5)  # 24 行
    pos = np.zeros(len(t))
    pos[2:8] = [2.0, 4.0, 6.0, 6.0, 5.0, 3.0]  # 区間 [2, 10] の中で山 → 下がる
    pos[8:11] = [3.0, 2.5, 2.0]
    lab = segment_labels(t, pos, [BrakeSegment(2, 10, False)], post_s=1.0)
    assert lab.backoff_pct[2:11] == pytest.approx([0, 0, 0, 0, 1, 3, 3, 3.5, 4.0])
    assert np.isnan(lab.backoff_pct[0]) and lab.elapsed_s[2] == 0.0
    assert lab.elapsed_s[10] == pytest.approx(4.0)
    # 9 行 → 3 行ずつ。離した後は 1.0s 以内の 2 行（i = 11, 12）で、山は区間全体の最大 6
    assert list(lab.third[2:11]) == [0, 0, 0, 1, 1, 1, 2, 2, 2]
    assert list(lab.third[11:14]) == [3, 3, -1]
    assert lab.backoff_pct[11] == pytest.approx(6.0 - pos[11])


def test_backoff_bin_index_edges():
    b = np.array([0.0, 0.05, 0.0501, 0.99, 1.0, 1.99, 2.0, 5.0, np.nan])
    assert list(backoff_bin_index(b)) == [0, 0, 1, 1, 2, 2, 3, 3, -1]


def test_motion_index_boundaries():
    """傾き +1%/s ちょうどは保持、それより大きければ踏み増し。−1%/s も同様。"""
    t = np.arange(0.0, 5.0, 1.0)
    assert list(motion_index(t, np.array([0.0, 1.0, 2.0, 3.0, 4.0]))) == [1] * 5
    assert list(motion_index(t, np.array([0.0, 1.5, 3.0, 4.5, 6.0]))) == [0] * 5
    assert list(motion_index(t, np.array([0.0, -1.0, -2.0, -3.0, -4.0]))) == [1] * 5
    assert list(motion_index(t, np.array([0.0, -1.5, -3.0, -4.5, -6.0]))) == [2] * 5


def test_brake_effect_recovers_brake_part_and_lag_shift():
    """一定のブレーキ減速 + 惰行 → ブレーキ分だけ取り出せる。遅れの分だけ前にずれる。"""
    t = np.arange(0.0, 20.0, 0.1)
    coast = 0.5  # 惰行の減速 [km/h/s]
    brake = np.where(t >= 10.0, 2.0, 0.0)  # 10s からブレーキが 2 km/h/s 効く
    v = 80.0 - np.cumsum((coast + brake) * 0.1)
    free = np.full(len(t), -coast)  # 惰行では負
    eff = brake_effect(t, v, free, 0.2)
    assert eff[50] == pytest.approx(0.0, abs=0.02)
    assert eff[150] == pytest.approx(2.0, abs=0.02)
    assert np.isnan(eff[-1])
    # 遅れ 0.2s = 2 行。立ち上がり（行 100）の中央は 2 行手前に出る
    eff0 = brake_effect(t, v, free, 0.0)
    mid0 = int(np.nanargmin(np.abs(eff0 - 1.0)))
    mid2 = int(np.nanargmin(np.abs(eff - 1.0)))
    assert mid0 - mid2 == 2
    assert list(shift_forward(np.array([1.0, 2.0, 3.0]), 1)[:2]) == [2.0, 3.0]
    assert np.isnan(shift_forward(np.array([1.0, 2.0, 3.0]), 1)[2])


def test_bin_index_and_effect_cell_minimum_rows():
    assert list(bin_index(np.array([0.4, 0.5, 1.4, 1.5, 9.0, np.nan]), (0.5, 1.5, 2.5))) == [
        -1, 0, 0, 1, 2, -1,
    ]
    eff = np.full(20, 1.5)
    all_rows = np.ones(20, dtype=bool)
    assert effect_cell(eff, all_rows, 0.1) == "1.50(2s)"
    few = np.zeros(20, dtype=bool)
    few[:9] = True  # 9 行 < 10 行
    assert effect_cell(eff, few, 0.1) == "−"


def test_main_effect_requires_train_csv(tmp_path):
    with pytest.raises(SystemExit):
        main(["--part", "effect", str(tmp_path / "x.csv")])
