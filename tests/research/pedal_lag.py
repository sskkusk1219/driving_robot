"""ペダル指令が変わってから車の加速度が変わるまでの遅れ L を、格子ステップ走行の CSV から測る。

`feedforward.pedal_select_center_s`（L。ProblemReport_20260929 段2）の自動測定。手順2 と
relearn（どちらも `pattern_drive.build_ff_model` を通る）が、この結果を config へ書き戻す。

測り方（155259 のログで試算して決めた定義）:
  格子ステップは踏むペダルを `grid_step_lag_s`（0.5s）かけてランプさせている
  （`pattern_loop._grid_step_openings`）が、本番のモード走行にはランプが無い。ランプの影響を
  打ち消すため「指令の 50% 点 → 加速度の 50% 点」の時間差を遅れとする。
    t_cmd: ステップ内で符号付き指令 e = アクセル指令 − ブレーキ指令 [%] が
           (e_from+e_to)/2 を変化の向きに初めて越えた時刻
    t_acc: ステップ内で局所加速度（±LOCAL_HALF_WINDOW_S の最小二乗の傾き。中心窓なので遅れを
           足さない）が (a_pre+a_post)/2 を変化の向きに初めて越えた時刻
  採用ステップの遅れの中央値を L とする。アクセル踏み増し・戻し・ブレーキは区別せず全部使う
  （種類別の中央値・件数は表示用）。

純関数のみ（config・ハードウェアには触らない）。src/ は import しない。
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from tests.research.drive_log import SECTION_PATTERN_DRIVE, cmd_opening

# HOLD_STEP: 格子ステップの「ステップ指令を保持している間」の phase 名
# （pattern_loop が CSV の phase 列へ書く値）。連続区間を 1 ステップとして扱う。
PHASE_HOLD_STEP = "HOLD_STEP"

# ステップ長の下限 [s]。155259 の HOLD_STEP は約 2.5〜3.0s。a_post の窓（開始+1.2s〜終了）に
# 十分な点数を残すための下限で、短いステップ（途中で打ち切られたもの）を捨てる。
MIN_STEP_S = 2.4

# 指令の変化量の下限 [%]（|e_to − e_from|）。これ未満は 50% 点の時刻がノイズで決まる。
MIN_CMD_CHANGE_PCT = 3.0

# 加速度の変化量の下限 [km/h/s]（|a_post − a_pre|）。155259 で変化が明瞭なステップだけを
# 残す値。これ未満は加速度の 50% 点が見つからない・ふらつく。
MIN_ACCEL_CHANGE_KMHS = 1.0

# a_pre（ステップ前の加速度）を測る窓の長さ [s]（ステップ開始の直前）。
PRE_WINDOW_S = 1.0

# a_post（ステップ後の加速度）を測る窓の開始（ステップ開始からの秒数）。ランプ 0.5s ＋
# 遅れ（約 0.3s）＋加速度が立ち上がる山が収まる時間として 1.2s 待つ。窓の終わりはステップ終了。
POST_WINDOW_START_S = 1.2

# 局所加速度を測る窓の半幅 [s]（±0.2s の最小二乗の傾き）。中心窓なので遅れを足さない。
# 車速は 0.05s 周期なので、この窓には 9 点前後が入る。
LOCAL_HALF_WINDOW_S = 0.2

# 採用ステップ数の下限。155259 では 60 件採用。これ未満は中央値が偏りやすいので書き戻さない。
MIN_STEPS = 10

# L の有効範囲 (0, MAX_LAG_S]。config.validate_config の pedal_select_center_s と同じ範囲。
MAX_LAG_S = 2.0

# 直線当てはめに必要な最小点数
_MIN_FIT_POINTS = 3

KIND_UP = "up"  # アクセル踏み増し（e が増え、ステップ終了時にブレーキ無し）
KIND_DOWN = "down"  # アクセル戻し（e が減り、ステップ終了時にブレーキ無し）
KIND_BRAKE = "brake"  # ステップ終了時に brake_cmd > 0
KINDS = (KIND_UP, KIND_DOWN, KIND_BRAKE)


@dataclass(frozen=True)
class StepRows:
    """PATTERN_DRIVE 行の時系列（列ごとの配列）。"""

    t_s: np.ndarray
    speed_kmh: np.ndarray
    accel_cmd_pct: np.ndarray
    brake_cmd_pct: np.ndarray
    phase: list[str]


@dataclass(frozen=True)
class PedalLagResult:
    lag_s: float | None  # 採用できたときの遅れの中央値 [s]（書き戻す値の元。丸めは呼び出し側）
    n_steps_total: int  # HOLD_STEP の連続区間の数
    n_used: int  # 採用ステップ数
    iqr_s: tuple[float, float] | None  # 四分位（25%, 75%）
    by_kind: dict[str, tuple[int, float]] = field(default_factory=dict)  # 種類 → (件数, 中央値)
    lags_s: tuple[float, ...] = ()  # 採用ステップごとの遅れ
    reason: str = ""  # lag_s が None のときの不採用理由


def read_step_rows(csv_path: Path) -> StepRows:
    """走行ログ CSV から PATTERN_DRIVE 行だけを読む（`read_pattern_groups` と同じ行選び）。

    phase 列が無い旧形式の CSV は phase がすべて空になる（HOLD_STEP が 0 件 → 推定は None）。
    """
    t: list[float] = []
    v: list[float] = []
    ac: list[float] = []
    bc: list[float] = []
    ph: list[str] = []
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if "section" in row and row["section"] != SECTION_PATTERN_DRIVE:
                continue
            try:
                elapsed = float(row["elapsed_s"])
                speed = float(row["actual_speed_kmh"]) if row["actual_speed_kmh"] else math.nan
                accel = cmd_opening(row, "accel")
                brake = cmd_opening(row, "brake")
            except (KeyError, ValueError):
                continue
            t.append(elapsed)
            v.append(speed)
            ac.append(accel)
            bc.append(brake)
            ph.append(row.get("phase", "") or "")
    return StepRows(
        t_s=np.array(t, dtype=float),
        speed_kmh=np.array(v, dtype=float),
        accel_cmd_pct=np.array(ac, dtype=float),
        brake_cmd_pct=np.array(bc, dtype=float),
        phase=ph,
    )


def _slope(t_s: np.ndarray, v_kmh: np.ndarray) -> float:
    """最小二乗の傾き [km/h/s]。NaN を除いて _MIN_FIT_POINTS 点未満なら NaN。"""
    ok = np.isfinite(t_s) & np.isfinite(v_kmh)
    if ok.sum() < _MIN_FIT_POINTS:
        return math.nan
    return float(np.polyfit(t_s[ok], v_kmh[ok], 1)[0])


def _hold_step_segments(phase: list[str]) -> list[tuple[int, int]]:
    """phase == HOLD_STEP の連続区間 [i0, i1)。"""
    segs: list[tuple[int, int]] = []
    i, n = 0, len(phase)
    while i < n:
        if phase[i] == PHASE_HOLD_STEP:
            j = i
            while j < n and phase[j] == PHASE_HOLD_STEP:
                j += 1
            segs.append((i, j))
            i = j
        else:
            i += 1
    return segs


def _first_crossing(values: np.ndarray, level: float, sign: float) -> int | None:
    """values が level を sign の向きに初めて越えた（sign*(v-level) >= 0）位置。無ければ None。"""
    hit = np.nonzero(sign * (values - level) >= 0)[0]
    return int(hit[0]) if len(hit) else None


def _step_lag(rows: StepRows, e: np.ndarray, i0: int, i1: int) -> tuple[str, float] | None:
    """1 ステップの (種類, 遅れ [s])。採用条件を満たさなければ None。"""
    t, v = rows.t_s, rows.speed_kmh
    if i0 < 1 or t[i1 - 1] - t[i0] < MIN_STEP_S:
        return None
    e_from, e_to = e[i0 - 1], e[i1 - 1]
    if abs(e_to - e_from) < MIN_CMD_CHANGE_PCT:
        return None
    pre = (t >= t[i0] - PRE_WINDOW_S) & (t < t[i0])
    a_pre = _slope(t[pre], v[pre])
    post = (t >= t[i0] + POST_WINDOW_START_S) & (t <= t[i1 - 1])
    a_post = _slope(t[post], v[post])
    if not (math.isfinite(a_pre) and math.isfinite(a_post)):
        return None
    if abs(a_post - a_pre) < MIN_ACCEL_CHANGE_KMHS:
        return None
    sign_cmd = 1.0 if e_to > e_from else -1.0
    k_cmd = _first_crossing(e[i0:i1], (e_from + e_to) / 2.0, sign_cmd)
    if k_cmd is None:
        return None
    t_cmd = t[i0 + k_cmd]
    local = np.array([
        _slope(
            t[(t >= t[k] - LOCAL_HALF_WINDOW_S) & (t <= t[k] + LOCAL_HALF_WINDOW_S)],
            v[(t >= t[k] - LOCAL_HALF_WINDOW_S) & (t <= t[k] + LOCAL_HALF_WINDOW_S)],
        )
        for k in range(i0, i1)
    ])
    local = np.where(np.isfinite(local), local, a_pre)  # 測れない点は変化前扱い（越えない）
    sign_acc = 1.0 if a_post > a_pre else -1.0
    k_acc = _first_crossing(local, (a_pre + a_post) / 2.0, sign_acc)
    if k_acc is None:
        return None
    t_acc = t[i0 + k_acc]
    if rows.brake_cmd_pct[i1 - 1] > 0:
        kind = KIND_BRAKE
    else:
        kind = KIND_UP if sign_cmd > 0 else KIND_DOWN
    return kind, float(t_acc - t_cmd)


def estimate_pedal_lag(rows: StepRows) -> PedalLagResult:
    """HOLD_STEP ごとの「指令 50% 点 → 加速度 50% 点」の遅れを集め、中央値を L として返す。

    採用ステップが MIN_STEPS 未満、または中央値が 0 < L ≤ MAX_LAG_S を外れるときは
    lag_s=None（reason に理由）。
    """
    e = rows.accel_cmd_pct - rows.brake_cmd_pct
    segs = _hold_step_segments(rows.phase)
    used: list[tuple[str, float]] = []
    for i0, i1 in segs:
        got = _step_lag(rows, e, i0, i1)
        if got is not None:
            used.append(got)
    lags = np.array([lag for _, lag in used], dtype=float)
    by_kind: dict[str, tuple[int, float]] = {}
    for kind in KINDS:
        sel = [lag for k, lag in used if k == kind]
        if sel:
            by_kind[kind] = (len(sel), float(np.median(sel)))
    iqr = (float(np.percentile(lags, 25)), float(np.percentile(lags, 75))) if len(lags) else None
    median = float(np.median(lags)) if len(lags) else None

    def result(lag: float | None, reason: str = "") -> PedalLagResult:
        return PedalLagResult(
            lag_s=lag, n_steps_total=len(segs), n_used=len(used), iqr_s=iqr,
            by_kind=by_kind, lags_s=tuple(float(x) for x in lags), reason=reason,
        )

    if not segs:
        return result(None, "HOLD_STEP の行がありません（旧形式・スタブの CSV）")
    if len(used) < MIN_STEPS:
        return result(
            None,
            f"採用ステップが {len(used)} 件（{MIN_STEPS} 件未満。HOLD_STEP {len(segs)} 件中）",
        )
    assert median is not None
    if not 0.0 < median <= MAX_LAG_S:
        return result(None, f"中央値 {median:.3f}s が有効範囲 0 < L ≤ {MAX_LAG_S:g}s の外")
    return result(median)
