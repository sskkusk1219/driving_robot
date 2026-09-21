"""段1: アクセル不感帯直上の欠陥切り分け（読み取り専用 CLI）。

`docs/Problem/ProblemReport_20260919.md` 6 章が保留した 4 つの原因候補
（(a) 真の不感帯が 8.21% より高い／(b) 不感帯の上に開度方向の非線形がある／
(c) 手順2 の学習パターンが低開度を測れていない／(d) 応答遅れが模型に入っていない）を、
既存の走行ログ（手順3・MODE_DRIVE 区間）だけで切り分けるための道具。車両には一切触らない。
CSV と yaml を読むだけで、既定ではファイルも書かない。

流儀は `tests/research/cruise_curve.py` / `stop_brake_floor.py` を踏襲する
（純関数 + frozen dataclass の結果 + `md_table` で表を stdout + `argparse` の `main`）。

用語（承認済み計画書「測り方（訂正を織り込んだ確定仕様）」節と同じ定義）:
    dt          … CSV ごとに `kpi.sample_interval_s` で実測した MODE_DRIVE 行の周期 [s]
                  （MODE_DRIVE は 0.05s、手順2 の CSV は 0.1s と混在するため CSV ごとに測る）
    定常窓      … `--steady-window-s`（既定 0.45s）の窓で実開度（アクセル）の幅が
                  `--steady-tol-pct`（既定 0.3%）以内なら「定常」とみなす
    a_obs(i)    … 中心差分 `(v[i+h] − v[i−h]) / (t[i+h] − t[i−h])`。
                  h は `--accel-half-window-s`（既定 0.30s）を dt で割ったサンプル数。
                  実車速は CAN 更新が約 14Hz のため隣接差分（1 周期差分）は使えない
                  （隣接差分の 28.7% がゼロになる）
    a_eff(i)    … `a_obs(i) − free_accel_at(params, research, v[i])`
                  （`free_accel_at` は「今ペダルを離したときの加速度」の単一ソース）
    x(i)        … `実開度(アクセル) − --deadband-pct`（既定は yaml の accel_deadband_pct）
    踏み方向    … `実開度[i] − 実開度[i − --direction-lookback-s]` を ±0.3% で
                  「下り」「一定」「上り」に 3 値化する（低開度サンプルは「開度を下げてきた」
                  側に偏っており、層別しないと (b) と (d) が混ざる）
    サンプル条件 … アクセル実開度 > 不感帯 ∧ ブレーキ実開度 <= ブレーキ不感帯 ∧
                  実車速 >= `--min-speed-kmh`（既定 5.0。`free_accel_at` は creep_speed_kmh=4.77
                  で符号が反転するため 3 ではなく 5 で切る）

出す表 6 つ＋表3b（詳細は各 `build_table*` の docstring 参照）:
    1. サンプル母数（CSV ごとの行数・dt・条件を満たす行数・惰行残差の検算）
    2. 速度帯 × 開度帯（a_eff 中央値・n・割線・接線の 3 つの十字表）
    3. 踏み方向の層別（下り/一定/上り の a_eff 中央値と、上り−下り の差）
    3b. 低開度階段の上り／下り層別（`LowOpenStairPattern` の往復を段の山型から検出し、
       表3 の瞬時踏み方向では区別できない「一定保持中の上り／下り」を対で比較する）
    4. 当てはめ（判定）… 速度帯ごとに折れ線 `a = k·max(0, x−x0)` をビン中央値へ等重みで当てはめ、
       x0 の 95% 区間をブートストラップで出す
    5. 応答遅れ（指令→実開度、実開度→a_eff のピーク lag・相関）
    6. 低開度エピソード（定常窓を通らない区間も拾う。偏差エピソードとの対応も見る）。
       `ref_speed_kmh`/`deviation_kmh` が全行で欠測（PATTERN_DRIVE 等、基準車速の無い区間）
       のときは出さず、省略した旨を 1 行出す。

2026-09-19（段3-1。ProblemReport_20260919 低開度階段の検証用）: `--section` を足した
（既定 MODE_DRIVE。**既定の挙動は変えない**）。PATTERN_DRIVE を指定すると手順2 のパターン走行
区間を同じロジックで解析できる（低開度階段が狙いどおり測れたかの確認に使う）。時刻列は
`mode_time_s` があればそれ、無ければ `elapsed_s` にフォールバックする。

CLI:
    .venv/bin/python -m tests.research.accel_onset \
        tests/research/results/drive_log_real_<日時>.csv [...複数可]
    .venv/bin/python -m tests.research.accel_onset --section PATTERN_DRIVE \
        tests/research/results/drive_log_real_<日時>.csv
"""

from __future__ import annotations

import argparse
import csv
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.models.profile import FeedforwardParams, pedal_gain_at
from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.debug_process23 import md_table
from tests.research.drive_log import (
    SECTION_MODE_DRIVE,
    SECTION_PATTERN_DRIVE,
    actual_opening,
    cmd_opening,
)
from tests.research.ff_params import ResearchFFParams, free_accel_at, research_ff_params
from tests.research.kpi import find_episodes, sample_interval_s
from tests.research.vehicle import feedforward_params

Row = Mapping[str, str]

# 踏み方向の 3 値化しきい値 [%]（計画書「測り方」節に明記された固定値。CLI 引数にはしない）
DIRECTION_TOL_PCT = 0.3

# 表6「低開度」の定義 [%]。表1「うち x<1.5%」と揃えた（計画書には明示が無く、実装時の判断。
# 詳細は本ファイルの呼び出し元への報告を参照）
LOW_X_THRESHOLD_PCT = 1.5

# 表5 の lag 探索範囲 [s]（計画書には CLI 引数として明記が無いため定数で持つ。実装時の判断）
TABLE5_MAX_LAG_S = 2.0
TABLE5_FIXED_LAG_S = 0.5

# 表4 のビン数がこれ未満なら当てはめない（当てはめには最低 3 点必要）
MIN_FIT_BINS = 3
# 3次当てはめは過学習を避けるため、この点数以上のときだけ行う
MIN_CUBIC_BINS = 5
# 判定(b) を疑う 3次 R² − 折れ線 R² の差のしきい値（実装時の判断。計画書に閾値の明記は無い）
CUBIC_GAIN_THRESHOLD = 0.02


# ─────────────────────────────────────────────────────────────────────
# CSV → 解析用の列（純関数。`rows` は csv.DictReader が返す dict 列を渡せる）
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True, eq=False)
class CsvSeries:
    """1 本の CSV（MODE_DRIVE 区間だけ）から作った、解析に使う列一式。"""

    source: str  # 表示用の名前（CLI ではファイル名）
    dt_s: float  # 実測した行間隔の中央値 [s]
    n_mode_rows: int
    t_s: np.ndarray  # mode_time_s
    v_kmh: np.ndarray  # actual_speed_kmh
    accel_pct: np.ndarray  # 実開度アクセル（NaN=欠測。actual_opening が None を返した行）
    brake_pct: np.ndarray  # 実開度ブレーキ（NaN=欠測）
    accel_cmd_pct: np.ndarray  # 指令開度アクセル（表5 の「指令→実開度」用）
    deviation_kmh: np.ndarray  # 偏差＝実車速−基準車速（NaN=基準なし）
    pattern: np.ndarray  # pattern 列（表3b の階段判定用。dtype=str。欠測は空文字）
    phase: np.ndarray  # phase 列（表3b の階段判定用。dtype=str。欠測は空文字）


def _opt_opening(row: Row, axis: str) -> float:
    v = actual_opening(row, axis)
    return float("nan") if v is None else v


def _opt_float(text: str | None) -> float:
    return float("nan") if not text else float(text)


def series_from_rows(
    rows: Iterable[Row], *, source: str, section: str = SECTION_MODE_DRIVE,
) -> CsvSeries:
    """CSV の全行（dict 列）→ 指定区間（既定 MODE_DRIVE）だけの `CsvSeries`。

    `mode_report.rows_from_csv` は実開度の列を持たない（`cmd_opening` しか読まない）ため使えず、
    ここで `csv.DictReader` 相当の dict 列を自前で回す（計画書の指示どおり）。

    時刻列は `mode_time_s`（モード経過秒。モード走行の行だけ持つ）があればそれを使い、
    無ければ `elapsed_s` にフォールバックする（PATTERN_DRIVE は mode_time_s を持たないため）。
    2026-09-19: `section=SECTION_PATTERN_DRIVE` を指定すると手順2 のパターン走行区間を解析できる
    （accel_onset.py 本体の解析ロジックはそのまま使える。ProblemReport_20260919 段3-1 の検証用）。
    **既定（SECTION_MODE_DRIVE）の挙動は変えない**: 行の絞り込みは従来どおり
    `section == MODE_DRIVE` かつ `mode_time_s` が空でないことを両方要求する。
    """
    if section == SECTION_MODE_DRIVE:
        sec_rows = [r for r in rows if r.get("section") == section and r.get("mode_time_s")]
    else:
        sec_rows = [r for r in rows if r.get("section") == section]
    if not sec_rows:
        raise ValueError(f"{section} の行が見つかりません: {source}")

    def _time(r: Row) -> float:
        mt = r.get("mode_time_s")
        return float(mt) if mt else float(r["elapsed_s"])

    t = np.array([_time(r) for r in sec_rows], dtype=float)
    v = np.array([float(r["actual_speed_kmh"]) for r in sec_rows], dtype=float)
    accel = np.array([_opt_opening(r, "accel") for r in sec_rows], dtype=float)
    brake = np.array([_opt_opening(r, "brake") for r in sec_rows], dtype=float)
    accel_cmd = np.array([cmd_opening(r, "accel") for r in sec_rows], dtype=float)
    deviation = np.array([_opt_float(r.get("deviation_kmh")) for r in sec_rows], dtype=float)
    pattern = np.array([r.get("pattern", "") or "" for r in sec_rows], dtype=str)
    phase = np.array([r.get("phase", "") or "" for r in sec_rows], dtype=str)

    dt = sample_interval_s(t.tolist())
    return CsvSeries(
        source, dt, len(sec_rows), t, v, accel, brake, accel_cmd, deviation, pattern, phase,
    )


def load_csv_series(path: Path, *, section: str = SECTION_MODE_DRIVE) -> CsvSeries:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return series_from_rows(rows, source=path.name, section=section)


# ─────────────────────────────────────────────────────────────────────
# 解析列の計算（中心差分・a_eff・定常判定・踏み方向）
# ─────────────────────────────────────────────────────────────────────


def _center_diff(t: np.ndarray, v: np.ndarray, half_window_s: float, dt_s: float) -> np.ndarray:
    """中心差分 `(v[i+h] − v[i−h]) / (t[i+h] − t[i−h])`。端（h 個）は NaN。"""
    n = len(t)
    h = max(1, round(half_window_s / dt_s))
    a_obs = np.full(n, np.nan)
    if n > 2 * h:
        num = v[2 * h :] - v[: n - 2 * h]
        den = t[2 * h :] - t[: n - 2 * h]
        with np.errstate(invalid="ignore", divide="ignore"):
            a_obs[h : n - h] = np.where(den > 0, num / den, np.nan)
    return a_obs


def _steady_mask(t: np.ndarray, accel: np.ndarray, window_s: float, tol_pct: float) -> np.ndarray:
    """`window_s` の窓（i を中心に前後 window_s/2）で実開度の幅が `tol_pct` 以内なら True。

    `t` が昇順であることを前提にした 2 ポインタのスライディングウィンドウ（O(n)）。
    """
    n = len(t)
    half = window_s / 2.0
    mask = np.zeros(n, dtype=bool)
    lo = 0
    hi = 0
    for i in range(n):
        lo_bound = t[i] - half
        hi_bound = t[i] + half
        while lo < n and t[lo] < lo_bound:
            lo += 1
        while hi < n and t[hi] <= hi_bound:
            hi += 1
        window = accel[lo:hi]
        valid = window[~np.isnan(window)]
        if valid.size == 0:
            continue
        mask[i] = (float(valid.max()) - float(valid.min())) <= tol_pct
    return mask


def _direction_labels(
    accel: np.ndarray, dt_s: float, lookback_s: float, tol_pct: float
) -> np.ndarray:
    """`accel[i] − accel[i − lookback]` を ±tol_pct で「上り」「一定」「下り」に 3 値化する。

    lookback 分の履歴が無い先頭 `lookback` サンプルは ""（不明。表3 では使わない）。
    """
    n = len(accel)
    lb = max(1, round(lookback_s / dt_s))
    labels = np.full(n, "", dtype="<U4")
    for i in range(lb, n):
        a0, a1 = accel[i - lb], accel[i]
        if np.isnan(a0) or np.isnan(a1):
            continue
        diff = a1 - a0
        if diff > tol_pct:
            labels[i] = "上り"
        elif diff < -tol_pct:
            labels[i] = "下り"
        else:
            labels[i] = "一定"
    return labels


# 低開度階段（`pattern_loop.LowOpenStairPattern`）の一定保持がこの phase に入る。
# 加速掃引（ACCEL_SWEEP）やブレーキ保持（BRAKE_HOLD）も「踏む→離す」で指令開度が山型に
# なるため、段数・山型だけでは階段と区別できない。phase で絞り込むことで、この2つを弾く
# （2026-09-20 追加。当初 phase 絞り込み無しで実装したところ、階段以外まで誤って
# 階段扱いしていたための訂正）。
STAIR_PHASE = "CRUISE_TRIM"


def _stair_leg_labels(pattern: np.ndarray, phase: np.ndarray, accel_cmd: np.ndarray) -> np.ndarray:
    """低開度階段（往復）の上り／下りレッグを判定する（表3b 用）。

    `phase == STAIR_PHASE` の行だけを対象にする（それ以外は問答無用で ""）。そのうえで
    `pattern` が同じ連続行を 1 つの「パターン区間」とみなし、その中で `accel_cmd` の値が
    変わるたびに段番号を振る。段が 5 段以上あり、かつ段の代表値が「単調増→単調減」の
    山型（折り返しが実在＝先頭・末尾以外に頂点がある）になっている区間だけを階段とみなし、
    頂点までの段を「上り」、頂点より後の段を「下り」にする。それ以外の行は ""。

    パターン名の文字列には依存しない（phase と指令開度の形だけで判定する）。
    """
    n = len(accel_cmd)
    labels = np.full(n, "", dtype="<U4")
    if n == 0:
        return labels

    stair_rows = phase == STAIR_PHASE
    seg_start = 0
    for i in range(1, n + 1):
        if i == n or pattern[i] != pattern[seg_start]:
            _label_stair_segment(labels, accel_cmd, stair_rows, seg_start, i)
            seg_start = i
    return labels


def _label_stair_segment(
    labels: np.ndarray, accel_cmd: np.ndarray, stair_rows: np.ndarray, start: int, end: int,
) -> None:
    """`[start, end)` の 1 パターン区間を山型階段として判定し、`labels` を書き換える（副作用）。"""
    idx = [i for i in range(start, end) if stair_rows[i] and not np.isnan(accel_cmd[i])]
    if not idx:
        return

    steps: list[list[int]] = [[idx[0]]]
    values: list[float] = [float(accel_cmd[idx[0]])]
    for i in idx[1:]:
        v = float(accel_cmd[i])
        if v != values[-1]:
            steps.append([i])
            values.append(v)
        else:
            steps[-1].append(i)

    m = len(steps)
    if m < 5:
        return

    s = np.array(values)
    peak = int(np.argmax(s))
    if peak < 1 or peak > m - 2:
        return
    if not np.all(np.diff(s[: peak + 1]) >= 0):
        return
    if not np.all(np.diff(s[peak:]) <= 0):
        return

    for step_i, rows in enumerate(steps):
        label = "上り" if step_i <= peak else "下り"
        for i in rows:
            labels[i] = label


def _free_accel_array(
    v: np.ndarray, params: FeedforwardParams, research: ResearchFFParams
) -> np.ndarray:
    return np.array([free_accel_at(params, research, float(vi)) for vi in v], dtype=float)


@dataclass(frozen=True, eq=False)
class CsvAnalysis:
    """`CsvSeries` に a_obs・a_eff・x・定常・踏み方向・各種サンプルマスクを足したもの。"""

    series: CsvSeries
    a_obs: np.ndarray
    free_accel: np.ndarray
    a_eff: np.ndarray
    x_pct: np.ndarray
    steady: np.ndarray
    direction: np.ndarray
    sample_mask: np.ndarray  # 表2/3/4 の母集団（定常窓あり）
    episode_mask: np.ndarray  # 表6 の母集団（定常窓なし）
    coast_mask: np.ndarray  # 表1 の惰行残差検算用（アクセル・ブレーキとも不感帯以下）
    stair_leg: np.ndarray  # 表3b 用（低開度階段の上り／下りレッグ。"上り"/"下り"/""）


def analyze_rows(
    rows: Iterable[Row],
    params: FeedforwardParams,
    research: ResearchFFParams,
    *,
    source: str,
    deadband_pct: float,
    steady_window_s: float,
    steady_tol_pct: float,
    accel_half_window_s: float,
    min_speed_kmh: float,
    direction_lookback_s: float,
    direction_tol_pct: float = DIRECTION_TOL_PCT,
    section: str = SECTION_MODE_DRIVE,
) -> CsvAnalysis:
    """CSV の全行（dict 列）→ `CsvAnalysis`。計画書「測り方」節の計算をすべてここで行う。"""
    series = series_from_rows(rows, source=source, section=section)
    t, v = series.t_s, series.v_kmh

    a_obs = _center_diff(t, v, accel_half_window_s, series.dt_s)
    free_accel = _free_accel_array(v, params, research)
    a_eff = a_obs - free_accel
    x = series.accel_pct - deadband_pct
    steady = _steady_mask(t, series.accel_pct, steady_window_s, steady_tol_pct)
    direction = _direction_labels(
        series.accel_pct, series.dt_s, direction_lookback_s, direction_tol_pct
    )
    stair_leg = _stair_leg_labels(series.pattern, series.phase, series.accel_cmd_pct)

    valid = ~np.isnan(series.accel_pct) & ~np.isnan(series.brake_pct) & ~np.isnan(a_eff)
    brake_ok = series.brake_pct <= params.brake_deadband_pct
    speed_ok = v >= min_speed_kmh
    pedal_on = x > 0.0
    pedal_off = series.accel_pct <= deadband_pct

    episode_mask = valid & brake_ok & speed_ok & pedal_on
    sample_mask = episode_mask & steady
    coast_mask = valid & brake_ok & speed_ok & pedal_off & steady

    return CsvAnalysis(
        series, a_obs, free_accel, a_eff, x, steady, direction,
        sample_mask, episode_mask, coast_mask, stair_leg,
    )


def analyze_csv_file(
    path: Path,
    params: FeedforwardParams,
    research: ResearchFFParams,
    **kwargs: float | str,
) -> CsvAnalysis:
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return analyze_rows(rows, params, research, source=path.name, **kwargs)  # type: ignore[arg-type]


def _pool_samples(
    analyses: Sequence[CsvAnalysis],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """`sample_mask` 行の v・x・a_eff・踏み方向を全 CSV 分プールする。

    既存の呼び出し元（本体の `run()` と `test_research_accel_onset.py` の回帰テストが直接
    この関数を呼んでいる）が 4 要素タプルの分解に依存しているため、戻り値の個数は変えない。
    表3b 用の階段レッグは `_pool_stair_legs`（直後）を別途呼ぶ。
    """
    v_all, x_all, a_all, dir_all = [], [], [], []
    for an in analyses:
        m = an.sample_mask
        v_all.append(an.series.v_kmh[m])
        x_all.append(an.x_pct[m])
        a_all.append(an.a_eff[m])
        dir_all.append(an.direction[m])
    return (
        np.concatenate(v_all) if v_all else np.array([]),
        np.concatenate(x_all) if x_all else np.array([]),
        np.concatenate(a_all) if a_all else np.array([]),
        np.concatenate(dir_all) if dir_all else np.array([], dtype="<U4"),
    )


def _pool_stair_legs(analyses: Sequence[CsvAnalysis]) -> np.ndarray:
    """`sample_mask` 行の階段レッグ（表3b 用）を全 CSV 分プールする（`_pool_samples` と対で使う）。

    表3b だけが使うため、`_pool_samples` の戻り値の個数は変えずに別関数として切り出した。
    """
    leg_all = [an.stair_leg[an.sample_mask] for an in analyses]
    return np.concatenate(leg_all) if leg_all else np.array([], dtype="<U4")


# ─────────────────────────────────────────────────────────────────────
# 表1: サンプル母数
# ─────────────────────────────────────────────────────────────────────


def build_table1(analyses: Sequence[CsvAnalysis]) -> tuple[list[str], list[list[str]]]:
    """CSV ごとの行数・dt・条件を満たす行・惰行残差（free_accel_at のバイアス検算）。"""
    header = [
        "CSV", "MODE行数", "dt[s]", "条件を満たす行", f"うち x<{LOW_X_THRESHOLD_PCT:g}%",
        "惰行行数", "惰行残差 中央値", "惰行残差 std",
    ]
    rows = []
    for an in analyses:
        s = an.series
        n_sample = int(np.count_nonzero(an.sample_mask))
        n_low = int(np.count_nonzero(an.sample_mask & (an.x_pct < LOW_X_THRESHOLD_PCT)))
        coast_resid = an.a_eff[an.coast_mask]
        n_coast = len(coast_resid)
        med = float(np.median(coast_resid)) if n_coast else None
        std = float(np.std(coast_resid)) if n_coast else None
        rows.append([
            s.source, str(s.n_mode_rows), f"{s.dt_s:.3f}", str(n_sample), str(n_low),
            str(n_coast),
            "—" if med is None else f"{med:+.3f}",
            "—" if std is None else f"{std:.3f}",
        ])
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# 表2: 速度帯 × 開度帯（a_eff 中央値・n / 割線 / 接線 の 3 つの十字表）
# ─────────────────────────────────────────────────────────────────────

BinStats = dict[tuple[int, int], tuple[int, float, float]]  # (si, oi) -> (n, median_a, median_x)


def _bin_stats(
    v: np.ndarray, x: np.ndarray, a: np.ndarray, speed_bins: Sequence[float],
    offset_bins: Sequence[float],
) -> BinStats:
    stats: BinStats = {}
    for si in range(len(speed_bins) - 1):
        smask = (v >= speed_bins[si]) & (v < speed_bins[si + 1])
        for oi in range(len(offset_bins) - 1):
            omask = smask & (x >= offset_bins[oi]) & (x < offset_bins[oi + 1])
            n = int(np.count_nonzero(omask))
            if n == 0:
                continue
            stats[(si, oi)] = (n, float(np.median(a[omask])), float(np.median(x[omask])))
    return stats


def _grid_header(speed_bins: Sequence[float]) -> list[str]:
    return ["オフセット [%]"] + [
        f"{speed_bins[i]:g}〜{speed_bins[i + 1]:g}" for i in range(len(speed_bins) - 1)
    ]


def _grid_table(
    stats: BinStats, speed_bins: Sequence[float], offset_bins: Sequence[float], cell_fn,
) -> tuple[list[str], list[list[str]]]:
    header = _grid_header(speed_bins)
    rows = []
    for oi in range(len(offset_bins) - 1):
        row = [f"{offset_bins[oi]:g}〜{offset_bins[oi + 1]:g}"]
        for si in range(len(speed_bins) - 1):
            row.append(cell_fn(stats.get((si, oi))))
        rows.append(row)
    return header, rows


def _cell_median_n(cell: tuple[int, float, float] | None) -> str:
    return "—" if cell is None else f"{cell[1]:+.2f} (n={cell[0]})"


def _cell_secant(cell: tuple[int, float, float] | None) -> str:
    if cell is None or cell[2] == 0:
        return "—"
    return f"{cell[1] / cell[2]:+.2f}"


def _tangent_grid(
    stats: BinStats, speed_bins: Sequence[float], offset_bins: Sequence[float],
) -> tuple[list[str], list[list[str]]]:
    """隣接する（存在する）開度ビン間の傾き `Δa_eff/Δx`（ビン中央値の x を代表点にする）。"""
    header = _grid_header(speed_bins)
    rows = []
    for oi in range(len(offset_bins) - 1):
        row = [f"{offset_bins[oi]:g}〜{offset_bins[oi + 1]:g}"]
        for si in range(len(speed_bins) - 1):
            cur = stats.get((si, oi))
            if cur is None:
                row.append("—")
                continue
            prev = next(
                (stats[(si, pj)] for pj in range(oi - 1, -1, -1) if (si, pj) in stats), None
            )
            if prev is None or cur[2] == prev[2]:
                row.append("—")
            else:
                row.append(f"{(cur[1] - prev[1]) / (cur[2] - prev[2]):+.2f}")
        rows.append(row)
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# 表3: 踏み方向の層別
# ─────────────────────────────────────────────────────────────────────


def build_table3(
    v: np.ndarray, x: np.ndarray, a: np.ndarray, direction: np.ndarray,
    speed_bins: Sequence[float], offset_bins: Sequence[float],
) -> tuple[list[str], list[list[str]]]:
    """(a)/(b) の判定は「一定」セルだけを使う。残り2つ（上り・下り）は (d) の大きさの推定用。"""
    header = [
        "速度帯 [km/h]", "開度帯 [%]", "下り n", "下り a_eff", "一定 n", "一定 a_eff",
        "上り n", "上り a_eff", "差(上り-下り)",
    ]
    rows = []
    for si in range(len(speed_bins) - 1):
        smask = (v >= speed_bins[si]) & (v < speed_bins[si + 1])
        for oi in range(len(offset_bins) - 1):
            omask = smask & (x >= offset_bins[oi]) & (x < offset_bins[oi + 1])
            if not np.any(omask):
                continue
            cells = [
                f"{speed_bins[si]:g}〜{speed_bins[si + 1]:g}",
                f"{offset_bins[oi]:g}〜{offset_bins[oi + 1]:g}",
            ]
            medians: dict[str, float | None] = {}
            for label in ("下り", "一定", "上り"):
                dmask = omask & (direction == label)
                n = int(np.count_nonzero(dmask))
                if n == 0:
                    cells += ["0", "—"]
                    medians[label] = None
                else:
                    med = float(np.median(a[dmask]))
                    cells += [str(n), f"{med:+.2f}"]
                    medians[label] = med
            up, down = medians["上り"], medians["下り"]
            cells.append("—" if up is None or down is None else f"{up - down:+.2f}")
            rows.append(cells)
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# 表3b: 低開度階段の上り／下り
# ─────────────────────────────────────────────────────────────────────


def build_table3b(
    v: np.ndarray, x: np.ndarray, a: np.ndarray, leg: np.ndarray,
    speed_bins: Sequence[float], offset_bins: Sequence[float],
) -> tuple[list[str], list[list[str]]]:
    """`LowOpenStairPattern`（低開度階段）の往復を上り／下りで層別する（表3 の一定保持版）。

    一定保持中は表3 の瞬時踏み方向では「一定」に潰れて区別できないため、`_stair_leg_labels`
    が判定した往復レッグ（段の山型から折り返しを検出）で層別する。上り・下りの両方に
    1 サンプル以上あるセルだけを出す（片方しか無いセルは差が計算できず読みにくいため省く）。
    """
    header = [
        "速度帯 [km/h]", "開度帯 [%]", "上り n", "上り a_eff", "下り n", "下り a_eff",
        "差(上り-下り)",
    ]
    rows = []
    for si in range(len(speed_bins) - 1):
        smask = (v >= speed_bins[si]) & (v < speed_bins[si + 1])
        for oi in range(len(offset_bins) - 1):
            omask = smask & (x >= offset_bins[oi]) & (x < offset_bins[oi + 1])
            if not np.any(omask):
                continue
            medians: dict[str, float | None] = {}
            counts: dict[str, int] = {}
            for label in ("上り", "下り"):
                dmask = omask & (leg == label)
                n = int(np.count_nonzero(dmask))
                counts[label] = n
                medians[label] = float(np.median(a[dmask])) if n else None
            up, down = medians["上り"], medians["下り"]
            if up is None or down is None:
                continue
            rows.append([
                f"{speed_bins[si]:g}〜{speed_bins[si + 1]:g}",
                f"{offset_bins[oi]:g}〜{offset_bins[oi + 1]:g}",
                str(counts["上り"]), f"{up:+.2f}",
                str(counts["下り"]), f"{down:+.2f}",
                f"{up - down:+.2f}",
            ])
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# 表4: 当てはめ（判定）
# ─────────────────────────────────────────────────────────────────────


def _r2(y: np.ndarray, pred: np.ndarray) -> float:
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    if ss_tot <= 0.0:
        return 1.0 if ss_res < 1e-9 else 0.0
    return 1.0 - ss_res / ss_tot


def _hinge_fit_no_intercept(
    x_pts: np.ndarray, y_pts: np.ndarray, x0_candidates: np.ndarray,
) -> tuple[float, float, float] | None:
    """`a = k·max(0, x − x0)` を、候補 x0 をグリッド探索して残差二乗和最小の組を返す。

    Returns:
        (rss, x0, k)、候補が 1 つも有効でなければ None。
    """
    best: tuple[float, float, float] | None = None
    for x0 in x0_candidates:
        u = np.maximum(0.0, x_pts - x0)
        denom = float(np.dot(u, u))
        if denom <= 0.0:
            continue
        k = float(np.dot(u, y_pts)) / denom
        resid = y_pts - k * u
        rss = float(np.dot(resid, resid))
        if best is None or rss < best[0]:
            best = (rss, float(x0), k)
    return best


def _hinge_fit_with_intercept(
    x_pts: np.ndarray, y_pts: np.ndarray, x0: float,
) -> tuple[float, float]:
    """検算用: 同じ x0 で `a = k·max(0, x − x0) + c` を最小二乗であてはめ、(k, c) を返す。"""
    u = np.maximum(0.0, x_pts - x0)
    design = np.vstack([u, np.ones_like(u)]).T
    coef, *_ = np.linalg.lstsq(design, y_pts, rcond=None)
    return float(coef[0]), float(coef[1])


def _cubic_r2(x_pts: np.ndarray, y_pts: np.ndarray) -> float | None:
    if len(x_pts) < MIN_CUBIC_BINS or len(set(x_pts.tolist())) < MIN_CUBIC_BINS:
        return None
    coeffs = np.polyfit(x_pts, y_pts, 3)
    pred = np.polyval(coeffs, x_pts)
    return _r2(y_pts, pred)


@dataclass(frozen=True, eq=False)
class BandFit:
    """1 速度帯の当てはめ結果（表4 の 1 行）。"""

    speed_lo: float
    speed_hi: float
    n_bins: int
    x0_pct: float | None
    x0_ci: tuple[float, float] | None
    k: float | None
    model_gain: float | None
    r2_line: float | None
    r2_cubic: float | None
    low_x_residual_sign: str
    c_check: float | None
    judgement: str

    @property
    def identified(self) -> bool:
        """当てはめできたか（ビン数不足・fit 失敗なら False）。"""
        return self.x0_pct is not None


def _band_fit_points(
    x_band: np.ndarray, a_band: np.ndarray, offset_bins: Sequence[float], min_bin_samples: int,
) -> tuple[np.ndarray, np.ndarray]:
    """開度帯ごとのビン中央値（x・a_eff）。`--min-bin-samples` 未満のビンは除外する。"""
    pts_x: list[float] = []
    pts_y: list[float] = []
    for oi in range(len(offset_bins) - 1):
        omask = (x_band >= offset_bins[oi]) & (x_band < offset_bins[oi + 1])
        n = int(np.count_nonzero(omask))
        if n < min_bin_samples:
            continue
        pts_x.append(float(np.median(x_band[omask])))
        pts_y.append(float(np.median(a_band[omask])))
    return np.array(pts_x), np.array(pts_y)


def _bootstrap_x0_ci(
    x_band: np.ndarray, a_band: np.ndarray, offset_bins: Sequence[float], min_bin_samples: int,
    n_boot: int, rng: np.random.Generator, x0_candidates: np.ndarray,
) -> tuple[float, float] | None:
    """行を復元抽出でリサンプルし、ビン中央値の再集計 → 折れ線再フィットを `n_boot` 回繰り返して
    x0 の分布を作り、95% 区間（パーセンタイル法）を返す。有効な反復が少なすぎれば None。
    """
    n = len(x_band)
    if n == 0 or n_boot <= 0:
        return None
    reps: list[float] = []
    for _ in range(n_boot):
        idx = rng.integers(0, n, size=n)
        pts_x, pts_y = _band_fit_points(x_band[idx], a_band[idx], offset_bins, min_bin_samples)
        if len(pts_x) < MIN_FIT_BINS:
            continue
        fit = _hinge_fit_no_intercept(pts_x, pts_y, x0_candidates)
        if fit is not None:
            reps.append(fit[1])
    if len(reps) < max(10, n_boot // 4):
        return None
    lo, hi = np.percentile(reps, [2.5, 97.5])
    return float(lo), float(hi)


def _judge(
    r2_line: float | None, r2_cubic: float | None, x0_ci: tuple[float, float] | None,
) -> str:
    """(a)/(b)/両方/不定 の簡易ヒューリスティック判定。

    計画書は「何を測れば決まるか」までしか定義しておらず、厳密な数値しきい値は無い
    （判定そのものは人が x0・CI・R² の数字を読んで行う想定）。ここでの機械的な判定は
    その一次スクリーニング用の実装判断であり、閾値は `CUBIC_GAIN_THRESHOLD` と
    「CI 下限が 0 より十分大きいか」の 2 条件のみ:
        - CI が求まらない → 不定
        - CI 下限 > 0.05% （真の不感帯が deadband_pct より高い）→ (a) を疑う
        - 3次 R² が 折れ線 R² を `CUBIC_GAIN_THRESHOLD` 以上上回る → (b) を疑う
        - 両方成立なら「両方」、どちらも成立しなければ「不定」
    """
    if x0_ci is None:
        return "不定(CI不明)"
    a_holds = x0_ci[0] > 0.05
    b_holds = r2_line is not None and r2_cubic is not None and (r2_cubic - r2_line) > (
        CUBIC_GAIN_THRESHOLD
    )
    if a_holds and b_holds:
        return "両方"
    if a_holds:
        return "a"
    if b_holds:
        return "b"
    return "不定"


def fit_band(
    x_band: np.ndarray, a_band: np.ndarray, *, speed_lo: float, speed_hi: float,
    offset_bins: Sequence[float], min_bin_samples: int, bootstrap_n: int,
    model_gain: float | None, rng: np.random.Generator,
) -> BandFit:
    """1 速度帯（`x_band`/`a_band` は呼び出し側で既にこの帯にフィルタ済み）の当てはめ。"""
    px, py = _band_fit_points(x_band, a_band, offset_bins, min_bin_samples)
    n_bins = len(px)
    if n_bins < MIN_FIT_BINS:
        return BandFit(speed_lo, speed_hi, n_bins, None, None, None, model_gain, None, None,
                        "", None, "不定(ビン不足)")

    x0_candidates = np.arange(max(0.0, float(px.min()) - 1.0), float(px.max()), 0.01)
    fit = _hinge_fit_no_intercept(px, py, x0_candidates)
    if fit is None:
        return BandFit(speed_lo, speed_hi, n_bins, None, None, None, model_gain, None, None,
                        "", None, "不定(fit失敗)")
    _, x0, k = fit
    pred_line = k * np.maximum(0.0, px - x0)
    r2_line = _r2(py, pred_line)
    r2_cubic = _cubic_r2(px, py)
    _, c_check = _hinge_fit_with_intercept(px, py, x0)

    i_min = int(np.argmin(px))
    resid_low = float(py[i_min] - pred_line[i_min])
    sign = "+" if resid_low > 1e-6 else ("−" if resid_low < -1e-6 else "0")

    ci = _bootstrap_x0_ci(x_band, a_band, offset_bins, min_bin_samples, bootstrap_n, rng,
                           x0_candidates)
    judgement = _judge(r2_line, r2_cubic, ci)

    return BandFit(speed_lo, speed_hi, n_bins, x0, ci, k, model_gain, r2_line, r2_cubic, sign,
                    c_check, judgement)


def _fit_table(
    v: np.ndarray, x: np.ndarray, a: np.ndarray, speed_bins: Sequence[float],
    offset_bins: Sequence[float], min_bin_samples: int, bootstrap_n: int,
    params: FeedforwardParams,
) -> tuple[list[str], list[list[str]]]:
    """速度帯ごとに `fit_band` を呼ぶ（母集団は呼び出し側が絞り込み済み）。表4 の1枚分を作る。"""
    header = [
        "速度帯 [km/h]", "ビン数", "x0 [%]", "95% CI", "k [(km/h/s)/%]", "模型ゲイン", "k/模型",
        "R²(折れ線)", "R²(3次)", "低x残差の符号", "c(検算)", "判定",
    ]
    rows = []
    rng = np.random.default_rng(0)
    for si in range(len(speed_bins) - 1):
        lo_v, hi_v = speed_bins[si], speed_bins[si + 1]
        smask = (v >= lo_v) & (v < hi_v)
        x_band, a_band = x[smask], a[smask]
        model_gain = pedal_gain_at(params, (lo_v + hi_v) / 2.0, is_accel=True)
        fit = fit_band(
            x_band, a_band, speed_lo=lo_v, speed_hi=hi_v, offset_bins=offset_bins,
            min_bin_samples=min_bin_samples, bootstrap_n=bootstrap_n, model_gain=model_gain,
            rng=rng,
        )
        ratio = None
        if fit.k is not None and fit.model_gain not in (None, 0.0):
            ratio = fit.k / fit.model_gain
        rows.append([
            f"{lo_v:g}〜{hi_v:g}",
            str(fit.n_bins),
            "—" if fit.x0_pct is None else f"{fit.x0_pct:.2f}",
            "—" if fit.x0_ci is None else f"{fit.x0_ci[0]:.2f}〜{fit.x0_ci[1]:.2f}",
            "—" if fit.k is None else f"{fit.k:.2f}",
            "—" if fit.model_gain is None else f"{fit.model_gain:.2f}",
            "—" if ratio is None else f"{ratio:.2f}",
            "—" if fit.r2_line is None else f"{fit.r2_line:.3f}",
            "—" if fit.r2_cubic is None else f"{fit.r2_cubic:.3f}",
            fit.low_x_residual_sign or "—",
            "—" if fit.c_check is None else f"{fit.c_check:+.2f}",
            fit.judgement,
        ])
    return header, rows


def build_table4(
    analyses: Sequence[CsvAnalysis], speed_bins: Sequence[float], offset_bins: Sequence[float],
    min_bin_samples: int, bootstrap_n: int, params: FeedforwardParams,
) -> tuple[tuple[list[str], list[list[str]]], tuple[list[str], list[list[str]]]]:
    """表4 を「一定のみ（計画書の主判定。訂正3）」と「参考（全方向）」の 2 枚で返す。

    表3 が示すとおり低開度は「下り」、高開度は「上り」に偏る（踏み方向と開度が交絡する）ため、
    全方向のまま当てはめると x0 が歪む。「一定」サンプルだけを使うのが主判定で、全方向は
    「一定」だとビンが痩せる帯（表3 参照）の比較用の参考値として併記する。
    """
    v, x, a, direction = _pool_samples(analyses)
    steady = direction == "一定"
    table_steady = _fit_table(
        v[steady], x[steady], a[steady], speed_bins, offset_bins, min_bin_samples, bootstrap_n,
        params,
    )
    table_all = _fit_table(v, x, a, speed_bins, offset_bins, min_bin_samples, bootstrap_n, params)
    return table_steady, table_all


# ─────────────────────────────────────────────────────────────────────
# 表5: 応答遅れ
# ─────────────────────────────────────────────────────────────────────


def _lagged_corr(x: np.ndarray, y: np.ndarray, lag: int) -> float | None:
    """`y` を `lag` サンプル遅らせて `x` と相関を取る（lag>0: y が x より遅れている）。"""
    n = len(x)
    if lag > 0:
        xs, ys = x[: n - lag], y[lag:]
    elif lag < 0:
        xs, ys = x[-lag:], y[: n + lag]
    else:
        xs, ys = x, y
    mask = ~np.isnan(xs) & ~np.isnan(ys)
    xs, ys = xs[mask], ys[mask]
    if len(xs) < 10 or float(np.std(xs)) == 0.0 or float(np.std(ys)) == 0.0:
        return None
    return float(np.corrcoef(xs, ys)[0, 1])


def _peak_lag(
    x: np.ndarray, y: np.ndarray, dt_s: float, max_lag_s: float,
) -> tuple[float, float] | None:
    """`x` に対して `y` が最も強く相関する非負ラグ（応答遅れ）[s] とその相関係数。"""
    max_lag_n = max(1, round(max_lag_s / dt_s))
    best: tuple[float, float] | None = None
    for lag in range(0, max_lag_n + 1):
        c = _lagged_corr(x, y, lag)
        if c is None:
            continue
        if best is None or c > best[1]:
            best = (lag * dt_s, c)
    return best


#: 表5 の脚注（相互相関は生信号ではなく 1 階差分どうしで取る理由。表の直前に印字する）
TABLE5_NOTE = (
    "※ 相互相関は 1 階差分どうしで取る（開度は 589s かけてゆっくり動く強い自己相関を持ち、"
    "生信号のままではどのラグでも相関が高止まりして遅れを判別できないため）。"
)


def build_table5(analyses: Sequence[CsvAnalysis]) -> tuple[list[str], list[list[str]]]:
    """応答遅れ（指令→実開度・実開度→a_eff のピーク lag・相関）。`TABLE5_NOTE` 参照。"""
    header = [
        "CSV", "指令→実開度 lag[s]", "指令→実開度 corr", "実開度→a_eff lag[s]",
        "実開度→a_eff corr", f"lag={TABLE5_FIXED_LAG_S:g}s corr",
    ]
    rows = []
    for an in analyses:
        s = an.series
        d_cmd = np.diff(s.accel_cmd_pct)
        d_actual = np.diff(s.accel_pct)
        d_aeff = np.diff(an.a_eff)
        cmd_to_actual = _peak_lag(d_cmd, d_actual, s.dt_s, TABLE5_MAX_LAG_S)
        actual_to_aeff = _peak_lag(d_actual, d_aeff, s.dt_s, TABLE5_MAX_LAG_S)
        fixed_lag_n = max(1, round(TABLE5_FIXED_LAG_S / s.dt_s))
        lag05 = _lagged_corr(d_actual, d_aeff, fixed_lag_n)
        rows.append([
            s.source,
            "—" if cmd_to_actual is None else f"{cmd_to_actual[0]:.2f}",
            "—" if cmd_to_actual is None else f"{cmd_to_actual[1]:.3f}",
            "—" if actual_to_aeff is None else f"{actual_to_aeff[0]:.2f}",
            "—" if actual_to_aeff is None else f"{actual_to_aeff[1]:.3f}",
            "—" if lag05 is None else f"{lag05:.3f}",
        ])
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# 表6: 低開度エピソード
# ─────────────────────────────────────────────────────────────────────


def _local_regression(x: np.ndarray, y: np.ndarray) -> tuple[float | None, float | None]:
    """`a_eff = α + β·x` を最小二乗で当て、(β, x0=−α/β) を返す（点が足りなければ None）。"""
    if len(x) < 3 or len(set(x.tolist())) < 2:
        return None, None
    beta, alpha = np.polyfit(x, y, 1)
    if beta == 0.0:
        return float(beta), None
    return float(beta), float(-alpha / beta)


def _find_overlap(start_s: float, end_s: float, dev_episodes, margin_s: float = 2.0) -> str:
    for d in dev_episodes:
        if d.start_s - margin_s <= end_s and d.end_s + margin_s >= start_s:
            return f"t={d.start_s:.1f}〜{d.end_s:.1f} (peak {d.peak_kmh:+.2f})"
    return "—"


def build_table6(
    analyses: Sequence[CsvAnalysis], episode_min_s: float,
) -> tuple[list[str], list[list[str]]]:
    """低開度が続いたエピソード（定常窓を通らない区間も拾う）と、偏差エピソードとの対応。

    `kpi.find_episodes` を再利用する: `episode_mask` を満たし `x < LOW_X_THRESHOLD_PCT` の行で
    不足量 `LOW_X_THRESHOLD_PCT − x` を作り、それが連続して正の区間を「低開度エピソード」とする
    （`find_episodes` の `limit_kmh=0.0` で abs(不足量) > 0 の連続区間を拾う）。
    エピソード内の `a_eff = α + β·x` の局所回帰から `x0 = −α/β`（低開度域の主推定器）。
    """
    header = [
        "CSV", "開始t[s]", "長さ[s]", "xの範囲[%]", "速度の範囲[km/h]", "β", "x0[%]",
        "対応する偏差エピソード",
    ]
    rows = []
    for an in analyses:
        s = an.series
        deficit = np.where(
            an.episode_mask & (an.x_pct < LOW_X_THRESHOLD_PCT),
            LOW_X_THRESHOLD_PCT - an.x_pct, 0.0,
        )
        episodes = [
            e for e in find_episodes(s.t_s.tolist(), deficit.tolist(), limit_kmh=0.0)
            if e.duration_s >= episode_min_s
        ]
        deviation_filled = np.nan_to_num(s.deviation_kmh, nan=0.0)
        dev_episodes = find_episodes(s.t_s.tolist(), deviation_filled.tolist(), limit_kmh=1.0)

        for ep in episodes:
            idx = (s.t_s >= ep.start_s) & (s.t_s <= ep.end_s) & an.episode_mask
            xs, as_ = an.x_pct[idx], an.a_eff[idx]
            vs = s.v_kmh[idx]
            beta, x0_local = _local_regression(xs, as_)
            rows.append([
                s.source, f"{ep.start_s:.1f}", f"{ep.duration_s:.1f}",
                f"{xs.min():.2f}〜{xs.max():.2f}" if len(xs) else "—",
                f"{vs.min():.1f}〜{vs.max():.1f}" if len(vs) else "—",
                "—" if beta is None else f"{beta:.2f}",
                "—" if x0_local is None else f"{x0_local:.2f}",
                _find_overlap(ep.start_s, ep.end_s, dev_episodes),
            ])
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


def _parse_float_list(text: str) -> tuple[float, ...]:
    return tuple(float(v) for v in text.split(","))


def run(
    csv_paths: Sequence[Path],
    config_path: Path,
    *,
    deadband_pct: float | None,
    speed_bins: Sequence[float],
    offset_bins: Sequence[float],
    steady_window_s: float,
    steady_tol_pct: float,
    accel_half_window_s: float,
    min_speed_kmh: float,
    direction_lookback_s: float,
    episode_min_s: float,
    min_bin_samples: int,
    bootstrap_n: int,
    exclude: set[str],
    section: str = SECTION_MODE_DRIVE,
) -> int:
    cfg = load_config(config_path)
    params = feedforward_params(cfg)
    research = research_ff_params(cfg)
    db = params.accel_deadband_pct if deadband_pct is None else deadband_pct

    paths = [p for p in csv_paths if p.name not in exclude]
    if not paths:
        print("解析対象の CSV がありません（--exclude で全部除外されました）")
        return 1

    analyses = [
        analyze_csv_file(
            p, params, research, deadband_pct=db, steady_window_s=steady_window_s,
            steady_tol_pct=steady_tol_pct, accel_half_window_s=accel_half_window_s,
            min_speed_kmh=min_speed_kmh, direction_lookback_s=direction_lookback_s,
            section=section,
        )
        for p in paths
    ]

    db_src = "CLI 指定" if deadband_pct is not None else "yaml既定"
    print(f"不感帯（--deadband-pct）: {db:.2f}%（{db_src}）")
    print(f"対象 CSV: {len(analyses)} 本")

    print("\n## 表1: サンプル母数\n")
    print(md_table(*build_table1(analyses)))

    v, x, a, direction = _pool_samples(analyses)
    leg = _pool_stair_legs(analyses)
    stats = _bin_stats(v, x, a, speed_bins, offset_bins)

    print("\n## 表2: 速度帯 × 開度帯\n")
    print("### a_eff 中央値 (n)\n")
    print(md_table(*_grid_table(stats, speed_bins, offset_bins, _cell_median_n)))
    print("\n### 割線 a_eff/x\n")
    print(md_table(*_grid_table(stats, speed_bins, offset_bins, _cell_secant)))
    print("\n### 接線 Δa_eff/Δx\n")
    print(md_table(*_tangent_grid(stats, speed_bins, offset_bins)))

    print("\n## 表3: 踏み方向の層別\n")
    print(md_table(*build_table3(v, x, a, direction, speed_bins, offset_bins)))

    print("\n## 表3b: 低開度階段の上り／下り\n")
    print(
        "※ 各段は「一定保持に入った直後の過渡」と「落ち着いた後」の両方を含む。"
        "上りと下りが同じ速度帯を通るのは主に過渡なので、この差は過渡の差として読むこと。"
    )
    if not np.any(leg != ""):
        print("低開度階段（指令開度が山型の一定保持パターン）が見つからないため表3b は省略します")
    else:
        header3b, rows3b = build_table3b(v, x, a, leg, speed_bins, offset_bins)
        print(md_table(header3b, rows3b) if rows3b else "（該当セル無し）")

    print("\n## 表4: 当てはめ（判定）\n")
    table_steady, table_all = build_table4(
        analyses, speed_bins, offset_bins, min_bin_samples, bootstrap_n, params
    )
    print("### 判定（一定のみ。計画書の主判定。訂正3）\n")
    print(md_table(*table_steady))
    print("\n### 参考（全方向。「一定」でビンが痩せる帯の比較用）\n")
    print(md_table(*table_all))

    print("\n## 表5: 応答遅れ\n")
    print(TABLE5_NOTE)
    print(md_table(*build_table5(analyses)))

    print("\n## 表6: 低開度エピソード\n")
    has_reference = any(
        not np.all(np.isnan(an.series.deviation_kmh)) for an in analyses
    )
    if not has_reference:
        print(f"{section} には基準車速が無いため表6 は省略します")
    else:
        header6, rows6 = build_table6(analyses, episode_min_s)
        print(md_table(header6, rows6) if rows6 else "（該当エピソード無し）")

    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("csv", nargs="+", type=Path, help="解析する走行ログ CSV（手順3。複数可）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument(
        "--deadband-pct", type=float, default=None,
        help="アクセル不感帯 [%%]（既定: yaml の feedforward.accel_deadband_pct）",
    )
    ap.add_argument(
        "--speed-bins", type=_parse_float_list, default=(5.0, 8.0, 12.0, 16.0, 24.0, 35.0, 50.0),
        help="速度帯の境界（カンマ区切り、昇順）",
    )
    ap.add_argument(
        "--offset-bins", type=_parse_float_list,
        default=(0.0, 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 30.0),
        help="開度オフセット（x）帯の境界（カンマ区切り、昇順）",
    )
    ap.add_argument("--steady-window-s", type=float, default=0.45)
    ap.add_argument("--steady-tol-pct", type=float, default=0.3)
    ap.add_argument("--accel-half-window-s", type=float, default=0.30)
    ap.add_argument("--min-speed-kmh", type=float, default=5.0)
    ap.add_argument("--direction-lookback-s", type=float, default=1.45)
    ap.add_argument("--episode-min-s", type=float, default=1.0)
    ap.add_argument("--min-bin-samples", type=int, default=15)
    ap.add_argument("--bootstrap", type=int, default=200)
    ap.add_argument(
        "--section", choices=(SECTION_MODE_DRIVE, SECTION_PATTERN_DRIVE),
        default=SECTION_MODE_DRIVE,
        help="解析する区間（既定 MODE_DRIVE。手順2 のログを見るときは PATTERN_DRIVE）",
    )
    ap.add_argument(
        "--exclude", nargs="*", default=[], metavar="FILENAME",
        help="除外する CSV のファイル名（複数可。例: その日の1本目を外す）",
    )
    args = ap.parse_args(argv)

    return run(
        args.csv, args.config, deadband_pct=args.deadband_pct, speed_bins=args.speed_bins,
        offset_bins=args.offset_bins, steady_window_s=args.steady_window_s,
        steady_tol_pct=args.steady_tol_pct, accel_half_window_s=args.accel_half_window_s,
        min_speed_kmh=args.min_speed_kmh, direction_lookback_s=args.direction_lookback_s,
        episode_min_s=args.episode_min_s, min_bin_samples=args.min_bin_samples,
        bootstrap_n=args.bootstrap, exclude=set(args.exclude), section=args.section,
    )


if __name__ == "__main__":
    raise SystemExit(main())
