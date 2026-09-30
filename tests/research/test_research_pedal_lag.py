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
