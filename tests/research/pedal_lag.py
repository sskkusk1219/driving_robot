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

2026-10-13 段2o-2: 手順2 の G ランプの後ろに足した「遅れの測定」（`pattern_loop` の
`HOLD_STEP`。30・60 km/h の定速からアクセル踏み増し → 戻し → 離す → ブレーキ踏み増し → 戻しの
5 つを開度を固定で持つ）も同じ定義で測る。違いは 3 つ（既定は今のまま）:
  - 5 つのステップは同じ HOLD_STEP の続いた区間に入るので、指令が 1 周期で `split_jump_pct` 以上
    飛んだ所で区間を分ける（`split_jump_pct` を渡したときだけ。既定 None は分けない）。
  - 件数の下限 `min_steps`・指令の変化の下限 `min_cmd_change_pct`・ステップ長の下限 `min_step_s`
    を引数にした（今回はペダル別に最大 4 件・変化の小さいステップがあるため）。
  - ペダル別（アクセル／ブレーキ）× 踏み増し／戻しの中央値と、ステップごとの車速も返す
    （`by_pedal`・`by_pedal_dir`・`steps`）。ペダルは指令が大きく動いたほう、向きはそのペダルの
    指令の増減で決める（待機位置が min のとき、反対のペダルの指令は 0 でなく min のため
    `brake_cmd > 0` では分けられない）。

純関数のみ（config・ハードウェアには触らない）。src/ は import しない。
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

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

# 2026-10-16 段2o-6: 指令が min（待機位置）にいるとみなす幅 [%]（アクチュエータの位置の揺れ）。
# play_onset.OTHER_MIN_TOL_PCT と同じ値（tests/research の各モジュールは独立に持つ）
AT_MIN_TOL_PCT = 0.2

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


PEDAL_ACCEL = "accel"
PEDAL_BRAKE = "brake"


@dataclass(frozen=True)
class StepLag:
    """採用した 1 ステップ（2026-10-13 段2o-2）。pedal は指令が大きく動いたほう、press は
    そのペダルの指令が増えたか（踏み増し）。"""

    kind: str
    pedal: str
    press: bool
    lag_s: float
    speed_kmh: float  # ステップ中の車速の平均
    # 2026-10-16 段2o-6: そのペダルの指令の始点か終点が min（待機位置）。遊びを通る時間が混ざる。
    # min が渡されたときだけ判定する（渡さなければ常に False）
    at_min: bool = False


@dataclass(frozen=True)
class PedalLagResult:
    lag_s: float | None  # 採用できたときの遅れの中央値 [s]（書き戻す値の元。丸めは呼び出し側）
    n_steps_total: int  # HOLD_STEP の連続区間の数
    n_used: int  # 採用ステップ数
    iqr_s: tuple[float, float] | None  # 四分位（25%, 75%）
    by_kind: dict[str, tuple[int, float]] = field(default_factory=dict)  # 種類 → (件数, 中央値)
    lags_s: tuple[float, ...] = ()  # 採用ステップごとの遅れ
    reason: str = ""  # lag_s が None のときの不採用理由
    # 2026-10-13 段2o-2: ペダル別（"accel" / "brake"）と、ペダル × 向き（"accel_press" /
    # "accel_release" / "brake_press" / "brake_release"）の (件数, 中央値)。採用ステップの一覧
    by_pedal: dict[str, tuple[int, float]] = field(default_factory=dict)
    by_pedal_dir: dict[str, tuple[int, float]] = field(default_factory=dict)
    steps: tuple[StepLag, ...] = ()
    # 2026-10-16 段2o-6: min を渡したときに、始点か終点が min のため使わなかったステップ
    excluded_steps: tuple[StepLag, ...] = ()


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


def _hold_step_segments(
    phase: list[str], e: np.ndarray | None = None, split_jump_pct: float | None = None
) -> list[tuple[int, int]]:
    """phase == HOLD_STEP の連続区間 [i0, i1)。

    `split_jump_pct` を渡すと、符号付き指令 e が 1 周期で |Δe| ≥ split_jump_pct 飛んだ所で区間を
    分ける（2026-10-13 段2o-2。開度を固定で持つ 5 つのステップが続いた区間に入るため）。
    """
    segs: list[tuple[int, int]] = []
    i, n = 0, len(phase)
    while i < n:
        if phase[i] == PHASE_HOLD_STEP:
            j = i
            while j < n and phase[j] == PHASE_HOLD_STEP:
                j += 1
            if e is None or split_jump_pct is None:
                segs.append((i, j))
            else:
                start = i
                for k in range(i + 1, j):
                    if abs(e[k] - e[k - 1]) >= split_jump_pct:
                        segs.append((start, k))
                        start = k
                segs.append((start, j))
            i = j
        else:
            i += 1
    return segs


def _first_crossing(values: np.ndarray, level: float, sign: float) -> int | None:
    """values が level を sign の向きに初めて越えた（sign*(v-level) >= 0）位置。無ければ None。"""
    hit = np.nonzero(sign * (values - level) >= 0)[0]
    return int(hit[0]) if len(hit) else None


def _step_lag(
    rows: StepRows, e: np.ndarray, i0: int, i1: int, *,
    min_cmd_change_pct: float = MIN_CMD_CHANGE_PCT, min_step_s: float = MIN_STEP_S,
    accel_min_pct: float | None = None, brake_min_pct: float | None = None,
) -> StepLag | None:
    """1 ステップの遅れ。採用条件を満たさなければ None。min を渡すと at_min も判定する。"""
    t, v = rows.t_s, rows.speed_kmh
    if i0 < 1 or t[i1 - 1] - t[i0] < min_step_s:
        return None
    e_from, e_to = e[i0 - 1], e[i1 - 1]
    if abs(e_to - e_from) < min_cmd_change_pct:
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
    d_accel = rows.accel_cmd_pct[i1 - 1] - rows.accel_cmd_pct[i0 - 1]
    d_brake = rows.brake_cmd_pct[i1 - 1] - rows.brake_cmd_pct[i0 - 1]
    if abs(d_brake) > abs(d_accel):
        pedal, press = PEDAL_BRAKE, bool(d_brake > 0)
    else:
        pedal, press = PEDAL_ACCEL, bool(d_accel > 0)
    seg_v = v[i0:i1]
    speed = float(np.nanmean(seg_v)) if np.isfinite(seg_v).any() else math.nan
    pedal_min = accel_min_pct if pedal == PEDAL_ACCEL else brake_min_pct
    cmd = rows.accel_cmd_pct if pedal == PEDAL_ACCEL else rows.brake_cmd_pct
    at_min = pedal_min is not None and bool(
        abs(cmd[i0 - 1] - pedal_min) <= AT_MIN_TOL_PCT
        or abs(cmd[i1 - 1] - pedal_min) <= AT_MIN_TOL_PCT
    )
    return StepLag(kind, pedal, press, float(t_acc - t_cmd), speed, at_min)


def estimate_pedal_lag(
    rows: StepRows, *, min_steps: int = MIN_STEPS,
    min_cmd_change_pct: float = MIN_CMD_CHANGE_PCT, min_step_s: float = MIN_STEP_S,
    split_jump_pct: float | None = None,
    accel_min_pct: float | None = None, brake_min_pct: float | None = None,
) -> PedalLagResult:
    """HOLD_STEP ごとの「指令 50% 点 → 加速度 50% 点」の遅れを集め、中央値を L として返す。

    採用ステップが `min_steps` 未満、または中央値が 0 < L ≤ MAX_LAG_S を外れるときは
    lag_s=None（reason に理由）。引数の既定は今までと同じ（2026-10-13 段2o-2 で引数にした。
    手順2 の遅れの測定は `min_steps`=2・`min_cmd_change_pct`=1.0・`split_jump_pct` つきで呼ぶ）。

    2026-10-16 段2o-6: `accel_min_pct`・`brake_min_pct`（待機位置 min）を渡すと、動いたペダルの
    指令の始点か終点が min（± AT_MIN_TOL_PCT）のステップは使わない（遊びを通る時間が遅れに
    混ざるため）。使わなかったステップは `excluded_steps`。渡さなければ今までどおり全部使う。
    """
    e = rows.accel_cmd_pct - rows.brake_cmd_pct
    segs = _hold_step_segments(rows.phase, e, split_jump_pct)
    steps: list[StepLag] = []
    excluded: list[StepLag] = []
    for i0, i1 in segs:
        got = _step_lag(
            rows, e, i0, i1, min_cmd_change_pct=min_cmd_change_pct, min_step_s=min_step_s,
            accel_min_pct=accel_min_pct, brake_min_pct=brake_min_pct,
        )
        if got is not None:
            (excluded if got.at_min else steps).append(got)
    used = [(st.kind, st.lag_s) for st in steps]
    lags = np.array([lag for _, lag in used], dtype=float)
    by_kind: dict[str, tuple[int, float]] = {}
    for kind in KINDS:
        sel = [lag for k, lag in used if k == kind]
        if sel:
            by_kind[kind] = (len(sel), float(np.median(sel)))
    by_pedal: dict[str, tuple[int, float]] = {}
    by_pedal_dir: dict[str, tuple[int, float]] = {}
    for pedal in (PEDAL_ACCEL, PEDAL_BRAKE):
        sel = [st.lag_s for st in steps if st.pedal == pedal]
        if sel:
            by_pedal[pedal] = (len(sel), float(np.median(sel)))
        for press in (True, False):
            sel = [st.lag_s for st in steps if st.pedal == pedal and st.press == press]
            if sel:
                key = f"{pedal}_{'press' if press else 'release'}"
                by_pedal_dir[key] = (len(sel), float(np.median(sel)))
    iqr = (float(np.percentile(lags, 25)), float(np.percentile(lags, 75))) if len(lags) else None
    median = float(np.median(lags)) if len(lags) else None

    def result(lag: float | None, reason: str = "") -> PedalLagResult:
        return PedalLagResult(
            lag_s=lag, n_steps_total=len(segs), n_used=len(used), iqr_s=iqr,
            by_kind=by_kind, lags_s=tuple(float(x) for x in lags), reason=reason,
            by_pedal=by_pedal, by_pedal_dir=by_pedal_dir, steps=tuple(steps),
            excluded_steps=tuple(excluded),
        )

    if not segs:
        return result(None, "HOLD_STEP の行がありません（旧形式・スタブの CSV）")
    if len(used) < min_steps:
        return result(
            None,
            f"採用ステップが {len(used)} 件（{min_steps} 件未満。HOLD_STEP {len(segs)} 件中）",
        )
    assert median is not None
    if not 0.0 < median <= MAX_LAG_S:
        return result(None, f"中央値 {median:.3f}s が有効範囲 0 < L ≤ {MAX_LAG_S:g}s の外")
    return result(median)


# ── 手順2 の遅れの測定（G ランプの後ろの HOLD_STEP。2026-10-13 段2o-2）──────
# 5 つのステップ（アクセル踏み増し → 戻し → 離す → ブレーキ踏み増し → 戻し）を 2 つの車速で
# 測る。ペダル別に最大 4 件しか取れないので、件数の下限・指令の変化の下限を小さくする。
# 値は仮: 件数 2（中央値が取れる最小）・指令の変化 1.0%（ステップの大きさは 0.05G の差で
# 数 %）・ステップ長 2.2 秒（保持は ramp_lag_step_hold_s 2.5 秒。周期のばらつきで最初と最後の
# 行の間が 2.4 秒に届かなくても採るため MIN_STEP_S より短く）・区間を分ける指令の飛び 1.0%。
LAG_STEP_MIN_STEPS = 2
LAG_STEP_MIN_CMD_CHANGE_PCT = 1.0
LAG_STEP_MIN_STEP_S = 2.2
LAG_STEP_SPLIT_JUMP_PCT = 1.0


def estimate_hold_step_lag(
    rows: StepRows, *, accel_min_pct: float | None = None, brake_min_pct: float | None = None,
) -> PedalLagResult:
    """手順2 の遅れの測定（`HOLD_STEP`）用の `estimate_pedal_lag`。上の LAG_STEP_* で呼ぶ。
    ペダル別の遅れは戻り値の `by_pedal`・`by_pedal_dir`・`steps`（車速別）で読む。
    2026-10-16 段2o-6: min を渡すと、min から出る・min へ戻るステップは使わない
    （`excluded_steps`）。
    渡さなければ今までどおり。"""
    return estimate_pedal_lag(
        rows,
        min_steps=LAG_STEP_MIN_STEPS,
        min_cmd_change_pct=LAG_STEP_MIN_CMD_CHANGE_PCT,
        min_step_s=LAG_STEP_MIN_STEP_S,
        split_jump_pct=LAG_STEP_SPLIT_JUMP_PCT,
        accel_min_pct=accel_min_pct, brake_min_pct=brake_min_pct,
    )


# ---------------------------------------------------------------------------------------------
# 2026-10-17 段2o-9: ペダルが min へ戻るまでの時間（遅れ ＋ 戻す量 ÷ 戻る速さ）
# ---------------------------------------------------------------------------------------------

# 指令が 1 周期でこれ以上下がった所を「戻しの場面」とする [%]
RETURN_MIN_DROP_PCT = 1.0

# 実開度が「新しい指令 ＋ この値」以下になった時刻を、戻った時刻とする [%]。止まっているときの
# 読みの揺れ（4.19〜4.24%）より少し大きい値
RETURN_SETTLE_TOL_PCT = 0.05

# 戻った時刻を探す先の行数（0.05s 周期で 2.0s）
RETURN_SEARCH_ROWS = 40

# 場面の間、指令が動かないとみなす幅 [%]
RETURN_CMD_STEADY_TOL_PCT = 0.3

# 場面の数の下限。これ未満は「測れず」
RETURN_MIN_SCENES = 3

# 2026-10-28 ProblemReport_20261028 段5b: 戻しの場面に使わない phase。小さな動き（dither）は入った
# 周期に中心 u0 へ段で移り、低速の階段は停車保持から一気に離すので、G ランプの戻しと別の動き方の
# 場面が混ざり、当てはめの傾きが負になった（`111335` のブレーキ: 16 件のうち dither 5・階段 1）
RETURN_EXCLUDED_PHASES = frozenset(
    {"RAMP_BRAKE_DITHER", "RAMP_ACCEL_DITHER", "LOW_RELEASE_STAIR", "LOW_PRESS_STAIR"}
)


@dataclass(frozen=True)
class PedalReturnFit:
    """1 ペダルの戻り時間の当てはめ。時間 = delay_s + 戻す量 / rate_pct_s。"""

    n: int  # 場面の数
    delay_s: float | None = None
    rate_pct_s: float | None = None
    max_err_s: float | None = None  # 当てはめからの外れの最大 [s]
    reason: str = ""  # 測れなかった理由

    @property
    def measured(self) -> bool:
        return self.delay_s is not None and self.rate_pct_s is not None


@dataclass(frozen=True)
class PedalReturn:
    accel: PedalReturnFit
    brake: PedalReturnFit


def _fit_return(
    cmd: np.ndarray, actual: np.ndarray, t: np.ndarray, excluded: np.ndarray | None = None
) -> PedalReturnFit:
    amounts: list[float] = []
    times: list[float] = []
    n = len(cmd)
    for i in range(1, n):
        if not cmd[i - 1] - cmd[i] >= RETURN_MIN_DROP_PCT:
            continue
        if excluded is not None and excluded[i]:
            continue
        target = cmd[i] + RETURN_SETTLE_TOL_PCT
        k_end = min(n, i + RETURN_SEARCH_ROWS)
        k = next((j for j in range(i, k_end) if actual[j] <= target), None)
        if k is None:
            continue
        if np.any(np.abs(np.diff(cmd[i:k + 1])) >= RETURN_CMD_STEADY_TOL_PCT):
            continue
        amount = actual[i - 1] - cmd[i]
        if not amount > 0.0:
            continue
        amounts.append(float(amount))
        times.append(float(t[k] - t[i]))
    if len(amounts) < RETURN_MIN_SCENES:
        return PedalReturnFit(
            n=len(amounts), reason=f"場面 {len(amounts)} 件（{RETURN_MIN_SCENES} 件未満）"
        )
    x = np.array(amounts)
    y = np.array(times)
    if float(np.ptp(x)) < 1e-9:
        return PedalReturnFit(n=len(x), reason="戻す量が全部同じで速さを分けられない")
    slope_inv, delay = np.polyfit(x, y, 1)  # time = delay + amount * (1/rate)
    if slope_inv <= 0.0:
        return PedalReturnFit(n=len(x), reason="戻る速さが 0 以下")
    err = float(np.max(np.abs(y - (delay + slope_inv * x))))
    return PedalReturnFit(
        n=len(x), delay_s=float(delay), rate_pct_s=float(1.0 / slope_inv), max_err_s=err,
    )


def estimate_pedal_return(csv_path: Path) -> PedalReturn:
    """走行 CSV から、ペダルごとの戻り時間（遅れ・戻る速さ）を測る。

    場面: 指令が 1 周期で `RETURN_MIN_DROP_PCT` 以上下がり、実開度が「新しい指令 +
    `RETURN_SETTLE_TOL_PCT`」以下になるまで指令が動かない所。戻した量 = 直前の実開度 − 新しい指令、
    時間 = 実開度が戻った時刻 − 指令が下がった時刻。最小二乗で 時間 = 遅れ + 量 / 速さ を当てる。
    場面が `RETURN_MIN_SCENES` 未満・速さ 0 以下は「測れず」（reason に理由）。
    全セクションの行を使う。ただし指令が下がった行の phase が `RETURN_EXCLUDED_PHASES` の場面は
    使わない（phase 列が無い CSV は今までどおり）。
    読めない CSV・列が無い CSV は両方「測れず」。
    """
    cols = ("elapsed_s", "accel_cmd_pct", "brake_cmd_pct", "accel_actual_pct", "brake_actual_pct")
    data: dict[str, list[float]] = {c: [] for c in cols}
    excluded: list[bool] = []
    try:
        with Path(csv_path).open(newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                try:
                    vals = [float(row[c]) for c in cols]
                except (KeyError, ValueError):
                    continue
                for c, v in zip(cols, vals, strict=True):
                    data[c].append(v)
                excluded.append(row.get("phase") in RETURN_EXCLUDED_PHASES)
    except OSError as exc:
        miss = PedalReturnFit(n=0, reason=f"CSV を読めない（{exc}）")
        return PedalReturn(accel=miss, brake=miss)
    t = np.array(data["elapsed_s"])
    ex = np.array(excluded, dtype=bool)
    return PedalReturn(
        accel=_fit_return(
            np.array(data["accel_cmd_pct"]), np.array(data["accel_actual_pct"]), t, ex
        ),
        brake=_fit_return(
            np.array(data["brake_cmd_pct"]), np.array(data["brake_actual_pct"]), t, ex
        ),
    )


# ---------------------------------------------------------------------------
# 2026-10-26 段11-2: ペダルの位置の動き方・位置 → 車の加速度の遅れ（PedalTrack）を手順2 から測る
# （ProblemReport_20261026_2 段2・段3）。値は以前 yaml の forward_map.pedal_track_* にモード走行から
# 当てはめた数字を置いていたが、本番ではモードを走れないので、手順2 の CSV と表から求める。

# 動きの当てはめの格子（delay [s]・速さの上限 [%/s]・加速度の上限 [%/s^2]）
TRACK_DELAY_GRID_S = (0.0, 0.01, 0.02, 0.03)
# 2026-10-27 ProblemReport_20261027 段1: ペダルの速度を 225/190 mm/s・0.6G に上げたので上側を広げた
# （225 mm/s ≒ 237 %/s・0.6G ≒ 6200 %/s^2。旧格子の上端 120・3000 に張り付いた `215040`）
TRACK_VMAX_GRID = (
    70.0, 80.0, 90.0, 100.0, 105.0, 110.0, 120.0, 140.0, 160.0, 180.0, 200.0, 220.0, 240.0,
)
TRACK_AMAX_GRID = (
    800.0, 1000.0, 1200.0, 1400.0, 1700.0, 2000.0, 2500.0, 3000.0, 4000.0, 5000.0, 6000.0, 7000.0,
)
# 実位置が 1 周期でこれ以上動いた行を「動いた」とする [%]
TRACK_MOVED_PCT = 0.05
# 動いた行の数の下限。これ未満は「測れず」
TRACK_MIN_MOVED_ROWS = 20
# 位置 → 加速度の遅れ λ の探索範囲 [s]
TRACK_LAG_GRID_S = tuple(round(0.025 * i, 3) for i in range(13))
# 遅れの残差: 車速の下限 [km/h]・動いた行の前後の行数・基準のゆっくりした成分を引く窓の行数
TRACK_LAG_MIN_SPEED_KMH = 3.0
TRACK_LAG_NEAR_ROWS = 4
TRACK_LAG_DETREND_ROWS = 81
TRACK_LAG_SPEED_SMOOTH_ROWS = 3


# pkl の meta に入れるキー（`pedal_track_` + 名前）。`fwd_plan.PedalTrack` のフィールド名と同じ
PEDAL_TRACK_META_KEYS = (
    "accel_delay_s", "accel_max_rate_pct_s", "accel_max_accel_pct_s2",
    "brake_delay_s", "brake_max_rate_pct_s", "brake_max_accel_pct_s2",
    "accel_effect_lag_s", "brake_effect_lag_s",
)


@dataclass(frozen=True)
class PedalTrackFit:
    """手順2 から測ったペダルの動き方（`fwd_plan.PedalTrack` の 8 値）。

    ブレーキの動き（delay・速さ・加速度の上限）は手順2 に速い動きが無く測れないため、アクセルの
    当てはめ値をそのまま使う（2026-10-26 ユーザー決定）。遅れ λ だけペダル別に測る。
    """

    measured: bool
    reason: str = ""
    accel_delay_s: float = math.nan
    accel_max_rate_pct_s: float = math.nan
    accel_max_accel_pct_s2: float = math.nan
    brake_delay_s: float = math.nan
    brake_max_rate_pct_s: float = math.nan
    brake_max_accel_pct_s2: float = math.nan
    accel_effect_lag_s: float = math.nan
    brake_effect_lag_s: float = math.nan
    rmse_pct: float = math.nan  # 動きの当てはめの位置の RMSE [%]（動いた行）
    n_moved_accel: int = 0  # アクセルの実位置が動いた行数
    n_moved_brake: int = 0  # ブレーキの同


def _read_track_columns(csv_path: Path) -> dict[str, np.ndarray]:
    cols = (
        "actual_speed_kmh", "accel_cmd_pct", "accel_actual_pct", "brake_actual_pct",
    )
    data: dict[str, list[float]] = {c: [] for c in cols}
    with Path(csv_path).open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            try:
                vals = [float(row[c]) for c in cols]
            except (KeyError, ValueError):
                continue
            for c, v in zip(cols, vals, strict=True):
                data[c].append(v)
    return {c: np.array(v) for c, v in data.items()}


def _rolling_median(x: np.ndarray, window: int) -> np.ndarray:
    """中心窓の移動中央値（端は窓を縮める＝min_periods=1）。"""
    half = window // 2
    n = len(x)
    out = np.empty(n)
    for i in range(n):
        out[i] = np.median(x[max(0, i - half):min(n, i + half + 1)])
    return out


def _rolling_mean3(x: np.ndarray, window: int) -> np.ndarray:
    half = window // 2
    n = len(x)
    cs = np.concatenate([[0.0], np.cumsum(x)])
    lo = np.maximum(0, np.arange(n) - half)
    hi = np.minimum(n, np.arange(n) + half + 1)
    return (cs[hi] - cs[lo]) / (hi - lo)


def estimate_pedal_track(
    csv_path: Path, table: Any, *, accel_min_pct: float, brake_min_pct: float,
    period_s: float = 0.05,
) -> PedalTrackFit:
    """手順2 の CSV と表から `PedalTrack` の 8 値を測る（2026-10-26 段11-2）。

    動き（delay・速さの上限・加速度の上限）: アクセルの指令（min で下限）→ 実位置の列を
    `fwd_plan._follow_pedal` で追い、実位置が動いた行（|差分| > 0.05%）の RMSE が最小の格子点を採る
    ブレーキは同じ値を使う（手順2 に速い動きが無く測れない。ユーザー決定）。
    遅れ λ: 実測の加速度（車速の 3 点平均の勾配）と、実位置を λ ずらして表（本体）から引いた加速度の
    差（80 行の中央値を引いて水準のずれを除く）の二乗和を、ペダルが動いた行の前後 ±4 行・車速 3 km/h
    超で最小にする λ（0〜0.3 s・0.025 刻み）。ペダル別。
    動いた行が少ない・読めない・列が無いときは measured=False（reason に理由）。
    """
    from tests.research.fwd_plan import PedalTrack, _follow_pedal, predicted_accel  # noqa: PLC0415

    try:
        d = _read_track_columns(csv_path)
    except OSError as exc:
        return PedalTrackFit(measured=False, reason=f"CSV を読めない（{exc}）")
    cmd, act = d["accel_cmd_pct"], d["accel_actual_pct"]
    brk, v = d["brake_actual_pct"], d["actual_speed_kmh"]
    n = len(cmd)
    if n < 2:
        return PedalTrackFit(measured=False, reason="CSV の行がない・列が足りない")
    moved_a = np.zeros(n, dtype=bool)
    moved_b = np.zeros(n, dtype=bool)
    moved_a[1:] = np.abs(np.diff(act)) > TRACK_MOVED_PCT
    moved_b[1:] = np.abs(np.diff(brk)) > TRACK_MOVED_PCT
    na, nb = int(moved_a.sum()), int(moved_b.sum())
    if na < TRACK_MIN_MOVED_ROWS:
        return PedalTrackFit(
            measured=False, n_moved_accel=na, n_moved_brake=nb,
            reason=f"アクセルが動いた行 {na} 件（{TRACK_MIN_MOVED_ROWS} 件未満）",
        )

    best: tuple[float, float, float, float] | None = None
    for dl in TRACK_DELAY_GRID_S:
        for vm in TRACK_VMAX_GRID:
            for am in TRACK_AMAX_GRID:
                pos = _follow_pedal(cmd, float(act[0]), dl, vm, am, accel_min_pct, period_s)
                rmse = float(np.sqrt(np.mean((pos[moved_a] - act[moved_a]) ** 2)))
                if best is None or rmse < best[0]:
                    best = (rmse, dl, vm, am)
    assert best is not None
    rmse, dl, vm, am = best

    aa = np.gradient(_rolling_mean3(v, TRACK_LAG_SPEED_SMOOTH_ROWS), period_s)
    base = (v > TRACK_LAG_MIN_SPEED_KMH)
    kernel = np.ones(2 * TRACK_LAG_NEAR_ROWS + 1)
    mask_a = base & (np.convolve(moved_a.astype(float), kernel, "same") > 0)
    mask_b = base & (np.convolve(moved_b.astype(float), kernel, "same") > 0)
    sse_a: dict[float, float] = {}
    sse_b: dict[float, float] = {}
    for lam in TRACK_LAG_GRID_S:
        trk = PedalTrack(accel_effect_lag_s=lam, brake_effect_lag_s=lam)
        a = predicted_accel(
            table, v, act, brk, track=trk, accel_min_pct=accel_min_pct,
            brake_min_pct=brake_min_pct, period_s=period_s,
        )
        x = aa - a
        x = x - _rolling_median(x, TRACK_LAG_DETREND_ROWS)
        sse_a[lam] = float(np.sum(x[mask_a] ** 2)) if mask_a.any() else math.inf
        sse_b[lam] = float(np.sum(x[mask_b] ** 2)) if mask_b.any() else math.inf
    lag_a = min(sse_a, key=lambda k: sse_a[k])
    lag_b = min(sse_b, key=lambda k: sse_b[k]) if nb >= TRACK_MIN_MOVED_ROWS else lag_a
    return PedalTrackFit(
        measured=True, accel_delay_s=dl, accel_max_rate_pct_s=vm, accel_max_accel_pct_s2=am,
        brake_delay_s=dl, brake_max_rate_pct_s=vm, brake_max_accel_pct_s2=am,
        accel_effect_lag_s=lag_a, brake_effect_lag_s=lag_b,
        rmse_pct=rmse, n_moved_accel=na, n_moved_brake=nb,
    )
