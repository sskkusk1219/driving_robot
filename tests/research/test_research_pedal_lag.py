"""pedal_lag（ペダル指令→加速度の遅れ L の自動測定）のテスト（合成データのみ）。"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from tests.research import pedal_lag
from tests.research.pedal_lag import (
    PHASE_HOLD_STEP,
    StepRows,
    estimate_pedal_lag,
    read_step_rows,
)

DT = 0.05


def _synth(
    n_steps: int,
    lag_s: float,
    ramp_s: float = 0.0,
    tau_s: float = 0.1,
    brake_every: int = 0,
    step_s: float = 3.0,
) -> StepRows:
    """一次遅れの加速度応答を持つ合成走行。

    指令 e を 0 ↔ 20% で交互に切り替え（ramp_s でランプ）、加速度 = 0.5*e を lag_s 遅らせて
    tau_s の一次遅れで通す。ステップ間は 3s の保持（phase は HOLD_STEP 以外）。
    """
    hold_n, gap_n = round(step_s / DT), round(3.0 / DT)
    t_list, e_list, ph = [], [], []
    e_prev, k = 0.0, 0
    t = 0.0
    for i in range(n_steps):
        e_to = 20.0 if i % 2 == 0 else 0.0
        for j in range(hold_n):
            frac = 1.0 if ramp_s <= 0 else min(1.0, (j * DT) / ramp_s)
            t_list.append(t)
            e_list.append(e_prev + (e_to - e_prev) * frac)
            ph.append(PHASE_HOLD_STEP)
            t += DT
        e_prev = e_to
        for _ in range(gap_n):
            t_list.append(t)
            e_list.append(e_prev)
            ph.append("SETTLE")
            t += DT
        k += 1
    t_arr, e_arr = np.array(t_list), np.array(e_list)
    # e を lag_s 遅らせて一次遅れを通した加速度 → 車速を積分
    acc = np.zeros_like(t_arr)
    shift = round(lag_s / DT)
    for i in range(1, len(t_arr)):
        target = 0.5 * e_arr[max(0, i - shift)]
        acc[i] = acc[i - 1] + (target - acc[i - 1]) * DT / tau_s
    speed = 50.0 + np.cumsum(acc) * DT
    accel_cmd = np.clip(e_arr, 0, None)
    brake_cmd = np.clip(-e_arr, 0, None)
    if brake_every:
        # 符号を反転してブレーキ扱い（種類判定のテスト用）
        accel_cmd, brake_cmd = brake_cmd, accel_cmd
    return StepRows(t_arr, speed, accel_cmd, brake_cmd, ph)


def test_known_lag_is_recovered_without_ramp() -> None:
    rows = _synth(16, lag_s=0.30)
    res = estimate_pedal_lag(rows)
    assert res.lag_s is not None
    assert res.n_used == 15  # 先頭ステップは直前行が無いので除外
    assert abs(res.lag_s - 0.30) < 0.06


def test_ramp_is_cancelled_by_fifty_percent_definition() -> None:
    flat = estimate_pedal_lag(_synth(16, lag_s=0.30))
    ramped = estimate_pedal_lag(_synth(16, lag_s=0.30, ramp_s=0.5))
    assert flat.lag_s is not None and ramped.lag_s is not None
    # ランプ 0.5s を含めると +0.25s ずれるはずが、50% 点定義ではほぼ同じ値になる
    assert abs(ramped.lag_s - flat.lag_s) < 0.08


def test_too_few_steps_returns_none() -> None:
    res = estimate_pedal_lag(_synth(6, lag_s=0.30))
    assert res.lag_s is None
    assert "件" in res.reason


def test_out_of_range_returns_none() -> None:
    res = estimate_pedal_lag(_synth(16, lag_s=2.5))
    assert res.lag_s is None
    assert "範囲" in res.reason


def test_no_hold_step_returns_none() -> None:
    rows = _synth(16, lag_s=0.30)
    rows = StepRows(rows.t_s, rows.speed_kmh, rows.accel_cmd_pct, rows.brake_cmd_pct,
                    ["" for _ in rows.phase])
    res = estimate_pedal_lag(rows)
    assert res.lag_s is None
    assert res.n_steps_total == 0


def test_brake_kind_is_classified() -> None:
    res = estimate_pedal_lag(_synth(16, lag_s=0.30, brake_every=1))
    assert res.by_kind.get("brake", (0, 0.0))[0] > 0


def test_read_step_rows_skips_other_sections_and_old_format(tmp_path: Path) -> None:
    path = tmp_path / "log.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["section", "elapsed_s", "actual_speed_kmh", "accel_cmd_pct",
                    "brake_cmd_pct", "phase"])
        w.writerow(["PATTERN_DRIVE", "0.0", "10.0", "5.0", "0.0", "HOLD_STEP"])
        w.writerow(["DECEL_TO_STOP", "0.05", "9.0", "0.0", "0.0", ""])
        w.writerow(["PATTERN_DRIVE", "0.1", "", "5.0", "0.0", "HOLD_STEP"])
    rows = read_step_rows(path)
    assert len(rows.t_s) == 2
    assert np.isnan(rows.speed_kmh[1])
    assert rows.phase == ["HOLD_STEP", "HOLD_STEP"]
    assert pedal_lag.estimate_pedal_lag(rows).lag_s is None


# ── 手順2 の遅れの測定（5 つのステップが続いた HOLD_STEP。2026-10-13 段2o-2）──────

ACCEL_MIN, BRAKE_MIN = 4.2, 2.9  # 待機位置 min（反対のペダルの指令は 0 でなく min になる）


def _synth_lag_steps(
    speeds: tuple[float, ...], lag_s: float, tau_s: float = 0.1, hold_s: float = 2.5,
    u_hold: float = 5.0, sens_a: float = 0.4, sens_b: float = 0.5,
) -> StepRows:
    """車速ごとに、保持（RAMP_HOLD）→ 5 つの HOLD_STEP（u: 保持 +6 → 保持 → 0 → −5 → 0）を
    続けて並べた合成走行。加速度 ＝ −0.5 ＋ 感度 × u を lag_s 遅らせて tau_s の一次遅れで通す。
    アクセル指令 ＝ min ＋ u（u ≥ 0）、ブレーキ指令 ＝ min ＋ |u|（u ＜ 0）、反対側は min。"""
    t_list: list[float] = []
    u_list: list[float] = []
    ph: list[str] = []
    t = 0.0
    start_idx: list[int] = []
    for _ in speeds:
        start_idx.append(len(t_list))
        for _ in range(round(4.0 / DT)):  # 保持（a_pre の窓用）
            t_list.append(t)
            u_list.append(u_hold)
            ph.append("RAMP_HOLD")
            t += DT
        for u in (u_hold + 6.0, u_hold, 0.0, -5.0, 0.0):
            for _ in range(round(hold_s / DT)):
                t_list.append(t)
                u_list.append(u)
                ph.append(PHASE_HOLD_STEP)
                t += DT
        for _ in range(round(4.0 / DT)):  # 次の車速への間
            t_list.append(t)
            u_list.append(0.0)
            ph.append("RAMP_DOWN")
            t += DT
    t_arr, u_arr = np.array(t_list), np.array(u_list)
    acc = np.zeros_like(t_arr)
    shift = round(lag_s / DT)
    for i in range(1, len(t_arr)):
        u = u_arr[max(0, i - shift)]
        target = -0.5 + (sens_a * u if u >= 0 else sens_b * u)
        acc[i] = acc[i - 1] + (target - acc[i - 1]) * DT / tau_s
    speed = np.zeros_like(t_arr)
    for k, v0 in zip(start_idx, speeds, strict=True):
        speed[k:] = v0  # 各車速の区間は、その車速を基準に積分する
        end = min(len(t_arr), k + round((4.0 + 5 * hold_s + 4.0) / DT))
        speed[k:end] = v0 + np.cumsum(acc[k:end]) * DT
    accel_cmd = ACCEL_MIN + np.clip(u_arr, 0, None)
    brake_cmd = BRAKE_MIN + np.clip(-u_arr, 0, None)
    return StepRows(t_arr, speed, accel_cmd, brake_cmd, ph)


def test_hold_step_five_steps_are_split_and_lag_recovered_per_pedal() -> None:
    rows = _synth_lag_steps((30.0, 60.0), lag_s=0.20)
    res = pedal_lag.estimate_hold_step_lag(rows)
    assert res.n_steps_total == 10  # 2 車速 × 5 ステップ（指令の飛びで区間を分けた）
    assert res.n_used == 10
    assert res.lag_s is not None and abs(res.lag_s - 0.20) < 0.05
    # ペダル別: アクセル 3 つ（踏み増し・戻し・離す）× 2、ブレーキ 2 つ（踏み増し・戻し）× 2
    assert res.by_pedal["accel"][0] == 6 and res.by_pedal["brake"][0] == 4
    for key in ("accel", "brake"):
        assert abs(res.by_pedal[key][1] - 0.20) < 0.05
    assert res.by_pedal_dir["accel_press"][0] == 2
    assert res.by_pedal_dir["accel_release"][0] == 4  # 戻し ＋ 離す
    assert res.by_pedal_dir["brake_press"][0] == 2 and res.by_pedal_dir["brake_release"][0] == 2
    # 車速別に読める（各ステップの平均車速。合成の車は保持中も少し加速するので境目は 58 km/h）
    low = [s for s in res.steps if s.speed_kmh < 58.0]
    high = [s for s in res.steps if s.speed_kmh >= 58.0]
    assert len(low) == 5 and len(high) == 5


def test_hold_step_default_estimate_does_not_split_and_needs_ten_steps() -> None:
    """引数の既定は今まで通り: 区間を分けず（1 車速 ＝ 1 区間）、10 件未満は None。"""
    rows = _synth_lag_steps((30.0, 60.0), lag_s=0.20)
    res = estimate_pedal_lag(rows)
    assert res.n_steps_total == 2
    assert res.lag_s is None


def test_hold_step_lag_differs_per_pedal() -> None:
    """ペダルごとに遅れが違っても読み分けられる（ここではアクセルのみ遅れを変えた合成は
    作れないので、感度の違いで混ざらないこと = 全ステップが同じ 0.3 秒で返ることを見る）。"""
    res = pedal_lag.estimate_hold_step_lag(_synth_lag_steps((30.0, 60.0), lag_s=0.30))
    assert res.lag_s is not None and abs(res.lag_s - 0.30) < 0.06
    assert abs(res.by_pedal["brake"][1] - 0.30) < 0.06


def test_hold_step_steps_starting_or_ending_at_min_are_excluded() -> None:
    """2026-10-16 段2o-6: min を渡すと、動いたペダルの指令の始点か終点が min のステップは使わない
    （遊びを通る時間が混ざるため）。使うのは アクセルの踏み増し・戻し（保持の開度どうし）だけ。"""
    rows = _synth_lag_steps((30.0, 60.0), lag_s=0.20)
    res = pedal_lag.estimate_hold_step_lag(rows, accel_min_pct=ACCEL_MIN, brake_min_pct=BRAKE_MIN)
    assert res.n_steps_total == 10 and res.n_used == 4
    assert len(res.excluded_steps) == 6  # アクセルの離す 2 ＋ ブレーキの踏み出し・戻し 4
    assert all(s.at_min for s in res.excluded_steps) and not any(s.at_min for s in res.steps)
    assert {s.pedal for s in res.steps} == {"accel"}
    assert res.by_pedal_dir["accel_press"][0] == 2 and res.by_pedal_dir["accel_release"][0] == 2
    assert "brake" not in res.by_pedal
    assert res.lag_s is not None and abs(res.lag_s - 0.20) < 0.05


def test_hold_step_without_min_keeps_the_old_behaviour() -> None:
    res = pedal_lag.estimate_hold_step_lag(_synth_lag_steps((30.0, 60.0), lag_s=0.20))
    assert res.excluded_steps == () and res.n_used == 10
    assert not any(s.at_min for s in res.steps)


# ─── 戻る時間（2026-10-17 段2o-9） ───────────────────────────────────────

_RET_COLS = [
    "elapsed_s", "accel_cmd_pct", "brake_cmd_pct", "accel_actual_pct", "brake_actual_pct",
]


def _write_return_csv(
    path: Path, amounts: list[float], delay_s: float, rate: float, pedal: str = "accel",
    cmd_min: float = 4.2,
) -> Path:
    """指令が 1 周期で下がり、実開度は delay_s 後から rate [%/s] で min へ戻る合成走行。

    場面ごとに 1 秒保持 → 指令が cmd_min へ → 1 秒保持 → 次の場面の開度へ（上げは即座）。
    """
    rows = []
    t = 0.0
    for amount in amounts:
        level = cmd_min + amount
        for _ in range(20):  # 保持
            rows.append((t, level, level))
            t += DT
        t_drop = t
        for _ in range(20):  # 戻し: 指令は cmd_min、実開度は遅れてから直線で下がる
            act = max(cmd_min, level - rate * max(t - t_drop - delay_s, 0.0))
            rows.append((t, cmd_min, act))
            t += DT
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(_RET_COLS)
        for tt, cmd, act in rows:
            other = (cmd_min, cmd_min)
            if pedal == "accel":
                w.writerow([f"{tt:.3f}", cmd, other[0], act, other[1]])
            else:
                w.writerow([f"{tt:.3f}", other[0], cmd, other[1], act])
    return path


def test_estimate_pedal_return_recovers_delay_and_rate(tmp_path: Path) -> None:
    csv_path = _write_return_csv(tmp_path / "r.csv", [5, 10, 15, 25, 40, 20], 0.10, 100.0)
    res = pedal_lag.estimate_pedal_return(csv_path)
    a = res.accel
    assert a.measured and a.n == 6 and a.reason == ""
    assert abs(a.delay_s - 0.10) < 0.03 and abs(a.rate_pct_s - 100.0) < 15.0
    assert a.max_err_s < 0.06
    # ブレーキ側は動いていない（min のまま）ので測れず
    assert not res.brake.measured and res.brake.n == 0 and "件" in res.brake.reason


def test_estimate_pedal_return_brake_pedal_and_other_values(tmp_path: Path) -> None:
    csv_path = _write_return_csv(
        tmp_path / "b.csv", [10, 20, 30, 45], 0.20, 50.0, pedal="brake"
    )
    b = pedal_lag.estimate_pedal_return(csv_path).brake
    assert b.measured and abs(b.delay_s - 0.20) < 0.03 and abs(b.rate_pct_s - 50.0) < 8.0


def test_estimate_pedal_return_not_measurable_with_few_scenes_or_same_amount(
    tmp_path: Path,
) -> None:
    few = pedal_lag.estimate_pedal_return(
        _write_return_csv(tmp_path / "f.csv", [10, 20], 0.1, 100.0)
    )
    assert not few.accel.measured and few.accel.n == 2 and "3" in few.accel.reason
    same = pedal_lag.estimate_pedal_return(
        _write_return_csv(tmp_path / "s.csv", [10, 10, 10, 10], 0.1, 100.0)
    )
    assert not same.accel.measured and same.accel.reason != ""


def test_estimate_pedal_return_missing_file_or_columns(tmp_path: Path) -> None:
    res = pedal_lag.estimate_pedal_return(tmp_path / "nothing.csv")
    assert not res.accel.measured and not res.brake.measured and "読めない" in res.accel.reason
    bad = tmp_path / "bad.csv"
    bad.write_text("elapsed_s,speed\n0,1\n", encoding="utf-8")
    assert not pedal_lag.estimate_pedal_return(bad).accel.measured


# --- 2026-10-26 段11-2: estimate_pedal_track（手順2 からペダルの動き方を測る） ---------


class _StubTable:
    """加速度 = 0.1 × u（アクセル側。ブレーキは u が負）、惰行 = −0.2 の表（車速に依らない）。"""

    def accel(self, v, u):
        return 0.1 * np.asarray(u, dtype=float)

    def coast_accel(self, v):
        return np.full_like(np.asarray(v, dtype=float), -0.2)


def _write_track_csv(path: Path, *, delay: float, vmax: float, amax: float, lag: float) -> None:
    """アクセル指令（min 4.2 ↔ 30 を不規則な長さで切り替え）→ 既知の動きで実位置 →
    既知の遅れで加速度 → 車速。ブレーキは動かさない（min のまま）。"""
    from tests.research.fwd_plan import PedalTrack, _follow_pedal, predicted_accel

    rng = np.random.default_rng(0)
    mn = 4.2
    cmd = []
    level = mn
    while len(cmd) < 1600:
        level = 30.0 if level == mn else mn
        cmd += [level] * int(rng.integers(10, 40))
    cmd_a = np.array(cmd[:1600])
    act = _follow_pedal(cmd_a, mn, delay, vmax, amax, mn, DT)
    brk = np.full(len(cmd_a), mn)
    trk = PedalTrack(accel_effect_lag_s=lag, brake_effect_lag_s=lag)
    a = predicted_accel(
        _StubTable(), np.full(len(cmd_a), 50.0), act, brk, track=trk,
        accel_min_pct=mn, brake_min_pct=mn, period_s=DT,
    )
    speed = 50.0 + np.cumsum(a) * DT
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["actual_speed_kmh", "accel_cmd_pct", "accel_actual_pct", "brake_actual_pct"])
        for row in zip(speed, cmd_a, act, brk, strict=True):
            w.writerow([f"{x:.6f}" for x in row])


def test_estimate_pedal_track_recovers_motion_and_lag(tmp_path: Path) -> None:
    p = tmp_path / "s2.csv"
    _write_track_csv(p, delay=0.02, vmax=90.0, amax=1700.0, lag=0.15)
    fit = pedal_lag.estimate_pedal_track(
        p, _StubTable(), accel_min_pct=4.2, brake_min_pct=4.2, period_s=DT
    )
    assert fit.measured
    # delay は 1 周期未満だと指令が階段のとき位置が同じになり分けられない（同点は小さい方）
    assert (fit.accel_max_rate_pct_s, fit.accel_max_accel_pct_s2) == (90.0, 1700.0)
    assert fit.accel_delay_s < DT
    # ブレーキの動きはアクセルの値を流用（ユーザー決定）
    assert (fit.brake_delay_s, fit.brake_max_rate_pct_s, fit.brake_max_accel_pct_s2) == (
        fit.accel_delay_s, fit.accel_max_rate_pct_s, fit.accel_max_accel_pct_s2)
    assert fit.rmse_pct < 1e-6
    assert abs(fit.accel_effect_lag_s - 0.15) <= 0.025
    assert fit.n_moved_accel > 100 and fit.n_moved_brake == 0
    # ブレーキは動かないので遅れはアクセルの値を流用
    assert fit.brake_effect_lag_s == fit.accel_effect_lag_s


def test_estimate_pedal_track_not_measured_when_pedal_does_not_move(tmp_path: Path) -> None:
    p = tmp_path / "flat.csv"
    with p.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["actual_speed_kmh", "accel_cmd_pct", "accel_actual_pct", "brake_actual_pct"])
        for _ in range(200):
            w.writerow([50.0, 4.2, 4.2, 4.2])
    fit = pedal_lag.estimate_pedal_track(
        p, _StubTable(), accel_min_pct=4.2, brake_min_pct=4.2, period_s=DT
    )
    assert not fit.measured and "動いた行" in fit.reason


def test_estimate_pedal_track_unreadable_csv(tmp_path: Path) -> None:
    fit = pedal_lag.estimate_pedal_track(
        tmp_path / "none.csv", _StubTable(), accel_min_pct=4.2, brake_min_pct=4.2
    )
    assert not fit.measured and "読めない" in fit.reason
