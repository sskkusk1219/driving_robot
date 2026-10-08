"""FF の「作ったずれ」を状態・ペダル・減速の強さ・速度帯ごとに集計する（段1）。

偏差 e = 実車速 − 基準（km/h）。隣り合う行の増分 de_i = e[i+1] − e[i] を、
行 i の区分（走行状態 × ペダル）ごとに足したものを「作ったずれ」と呼ぶ。
区間の入口で持ち込んだ偏差は増分に入らないので、その区間の FF が
偏差を増やしたのか減らしたのかだけが残る。

    .venv/bin/python -m tests.research.ff_bias --part created \
        tests/research/results/drive_log_real_XXXX.csv [...]

    .venv/bin/python -m tests.research.ff_bias --part cause \
        --train-csv tests/research/results/drive_log_real_20260928_155259.csv \
        tests/research/results/drive_log_real_XXXX.csv [...]

`--part cause` は減速中の「開度の過不足」を場面（速度帯 × 減速の強さ × ペダル）ごとに出し、
原因 A（ペダル選択）・B（学習データの網羅）・C（学習データ上の当てはまり）・D（走行データ上の
整合）のどれを指すかを数字で並べる。特徴量の作り方は ff_model.build_feature_matrix、
モデルの読み込みは ff_candidate.make_candidate → load_model
（mode_drive.load_feedforward と同じ手順）をそのまま呼ぶ。

    .venv/bin/python -m tests.research.ff_bias --part timing \
        [--train-csv tests/research/results/drive_log_real_20260928_155259.csv] \
        tests/research/results/drive_log_real_XXXX.csv [...]

`--part timing` はブレーキ区間ごとに、指令・実位置・基準の減速度・実車の減速度の時間差を
秒で測る（山の 50% 越えの時刻差と、相互相関のずらし量）。学習データのブレーキのステップも
同じ測り方で並べる。

    .venv/bin/python -m tests.research.ff_bias --part recompute \
        tests/research/results/drive_log_real_XXXX.csv [...]

`--part recompute` は走行 CSV の基準車速（線形補間）から FF の指令を計算し直し、ブレーキモデル入力の
遅らせ d・ペダル選択の窓（幅 H・中心 L）を 1 つずつ変えたときの「指令 → 基準の減速」の時間差・
ペダル切替のタイミング・アクセルの往復・アクセル⇔ブレーキの切替回数を並べる（車の応答は含まない）。

    .venv/bin/python -m tests.research.ff_bias --part effect \
        --train-csv tests/research/results/drive_log_real_20260928_155259.csv \
        tests/research/results/drive_log_real_XXXX.csv [...]

`--part effect` はブレーキ区間の「ブレーキが作った減速」（実車の減速 + 惰行の加速度。遅れ
`--effect-lag` 既定 0.2s）を、開度の超過（実位置 − 不感帯）・速度帯・区間の前/中/後 1/3・
ペダルの動き・山から戻した量・経過時間ごとに並べ、後半で効きが落ちる原因（H1 ヒステリシス・
H2 時間・H3 速度・H4 測り方）の候補ごとの予測の形に合うかを事実として出す。
"""

from __future__ import annotations

import argparse
import csv
import math
import pickle
import re
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.drive_log import (
    SECTION_MODE_DRIVE,
    SECTION_PATTERN_DRIVE,
    actual_opening,
    cmd_opening,
    read_drive_logs,
)
from tests.research.ff_candidate import CandidateFeedforward, make_candidate
from tests.research.ff_model import (
    FeatureSpec,
    build_feature_matrix,
    estimate_offsets,
    group_by_session,
)
from tests.research.ff_params import ResearchFFParams, free_accel_at, research_ff_params
from tests.research.kpi import (
    pedal_reversal_rates,
    sample_interval_s,
    speed_band_label,
    speed_band_order,
)
from tests.research.mode_report import (
    BRAKE_CONTRIB_WINDOW_S,
    REF_SLOPE_HALF_WINDOW_S,
    STATE_ACCEL,
    STATE_CRUISE,
    STATE_DECEL,
    ModeRow,
    driving_states,
    rows_from_csv,
)
from tests.research.pedal_lag import (
    KIND_BRAKE,
    LOCAL_HALF_WINDOW_S,
    MIN_ACCEL_CHANGE_KMHS,
    MIN_CMD_CHANGE_PCT,
    MIN_STEP_S,
    _hold_step_segments,
    estimate_pedal_lag,
    read_step_rows,
)
from tests.research.research_types import FeedforwardParams, pedal_gain_at
from tests.research.vehicle import feedforward_params

PEDAL_ACCEL = "ACCEL"
PEDAL_COAST = "COAST"
PEDAL_BRAKE = "BRAKE"
PEDAL_OTHER = "その他"
PEDALS = (PEDAL_ACCEL, PEDAL_COAST, PEDAL_BRAKE, PEDAL_OTHER)
# 停車は表に出さない（他と区別が付きにくいため）
STATES_SHOWN = (STATE_ACCEL, STATE_CRUISE, STATE_DECEL)
MIN_ROW_TIME_S = 1.0  # 表でこれ未満の時間の行は隠す（ノイズ行）

# 仮値: 議論時の集計の区切り [km/h/s]（基準の減速の強さ）。3 km/h/s を境に向きが反転したので残す。
# 表の粒度だけに使い、判定には使わない。
DECEL_BIN_EDGES_KMHS: tuple[float, ...] = (0.3, 1.5, 3.0, 5.0)
DEV_LIMIT_KMH = 1.0  # 「+側の割合」を数える偏差の大きさ


@dataclass(frozen=True)
class Cell:
    """1 区分の集計。"""

    time_s: float = 0.0
    created_kmh: float = 0.0  # 作ったずれの合計 [km/h]

    @property
    def per_s(self) -> float:
        return self.created_kmh / self.time_s if self.time_s > 0 else 0.0


@dataclass(frozen=True)
class CreatedReport:
    """1 本の CSV の集計結果。"""

    dt_s: float
    state_pedal: dict[tuple[str, str], Cell]
    decel_brake_bins: dict[str, Cell]
    decel_accel_bands: dict[str, Cell]
    over_count: int  # |e| > DEV_LIMIT_KMH の行数
    over_positive_count: int  # そのうち e > 0

    @property
    def positive_share(self) -> float:
        return self.over_positive_count / self.over_count if self.over_count else 0.0


def ref_slopes(rows: Sequence[ModeRow]) -> np.ndarray:
    """基準の傾き [km/h/s]（前後 REF_SLOPE_HALF_WINDOW_S）。`driving_states` と同じ作り方。"""
    if len(rows) == 0:
        return np.zeros(0)
    t = np.array([r.t_s for r in rows])
    ref = np.array([r.ref_kmh for r in rows])
    ahead = np.interp(t + REF_SLOPE_HALF_WINDOW_S, t, ref)
    behind = np.interp(t - REF_SLOPE_HALF_WINDOW_S, t, ref)
    return (ahead - behind) / (2.0 * REF_SLOPE_HALF_WINDOW_S)


def decel_bin_labels() -> list[str]:
    e = DECEL_BIN_EDGES_KMHS
    labels = [f"{lo:g}〜{hi:g}" for lo, hi in zip(e, e[1:], strict=False)]
    return [*labels, f"{e[-1]:g}以上"]


def decel_bin_label(decel_kmhs: float) -> str | None:
    """減速の強さ（正の値）が属する区切りのラベル。最小の区切り未満は None。"""
    e = DECEL_BIN_EDGES_KMHS
    if decel_kmhs < e[0]:
        return None
    labels = decel_bin_labels()
    for i, hi in enumerate(e[1:]):
        if decel_kmhs < hi:
            return labels[i]
    return labels[-1]


def pedal_of(phase: str) -> str:
    return phase if phase in (PEDAL_ACCEL, PEDAL_COAST, PEDAL_BRAKE) else PEDAL_OTHER


def _add(cells: dict, key: object, dt: float, inc: float) -> None:
    c = cells.get(key, Cell())
    cells[key] = Cell(c.time_s + dt, c.created_kmh + inc)


def compute_created(rows: Sequence[ModeRow]) -> CreatedReport:
    """行ごとの増分 e[i+1] − e[i] を、行 i の区分に足す。最終行は増分がないので数えない。"""
    dt = sample_interval_s([r.t_s for r in rows])
    states = driving_states(rows)
    slopes = ref_slopes(rows)
    state_pedal: dict[tuple[str, str], Cell] = {}
    brake_bins: dict[str, Cell] = {}
    accel_bands: dict[str, Cell] = {}
    over = over_pos = 0
    for i in range(len(rows) - 1):
        r = rows[i]
        inc = rows[i + 1].deviation_kmh - r.deviation_kmh
        pedal = pedal_of(r.phase)
        _add(state_pedal, (states[i], pedal), dt, inc)
        if states[i] == STATE_DECEL and pedal == PEDAL_BRAKE:
            label = decel_bin_label(-slopes[i])
            if label is not None:
                _add(brake_bins, label, dt, inc)
        if states[i] == STATE_DECEL and pedal == PEDAL_ACCEL:
            _add(accel_bands, speed_band_label(r.ref_kmh), dt, inc)
        if abs(r.deviation_kmh) > DEV_LIMIT_KMH:
            over += 1
            over_pos += r.deviation_kmh > 0
    return CreatedReport(dt, state_pedal, brake_bins, accel_bands, over, over_pos)


# ─────────────────────────────────────────────────────────────────────
# 表示
# ─────────────────────────────────────────────────────────────────────


def _label_of(path: Path) -> str:
    m = re.search(r"(\d{6})$", path.stem)
    return m.group(1) if m else path.stem


def _table(header: Sequence[str], body: Sequence[Sequence[object]]) -> list[str]:
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(str(c) for c in row) + " |" for row in body]
    return lines


def _cells_text(c: Cell | None) -> list[str]:
    c = c or Cell()
    return [f"{c.time_s:.1f}", f"{c.created_kmh:+.1f}", f"{c.per_s:+.3f}"]


def _shown(cells: Sequence[Cell | None]) -> bool:
    """どの CSV でも MIN_ROW_TIME_S 未満の行は隠す。"""
    return max((c.time_s for c in cells if c), default=0.0) >= MIN_ROW_TIME_S


def _sub_header(labels: Sequence[str]) -> list[str]:
    return [h for lab in labels for h in (f"{lab} 時間[s]", f"{lab} 合計[km/h]", f"{lab} [km/h/s]")]


def format_created(labels: Sequence[str], reports: Sequence[CreatedReport]) -> str:
    out: list[str] = ["## 1. 状態 × ペダル（作ったずれ。停車は除く・1.0s 未満の行は隠す）", ""]
    body = []
    for st in STATES_SHOWN:
        for pedal in PEDALS:
            cells = [rep.state_pedal.get((st, pedal)) for rep in reports]
            if not _shown(cells):
                continue
            body.append([st, pedal] + [t for c in cells for t in _cells_text(c)])
    out += _table(["状態", "ペダル", *_sub_header(labels)], body)

    out += ["", "## 2. 減速 × ブレーキ（基準の減速の強さ別 [km/h/s]）", ""]
    body = [
        [b] + [t for rep in reports for t in _cells_text(rep.decel_brake_bins.get(b))]
        for b in decel_bin_labels()
        if _shown([rep.decel_brake_bins.get(b) for rep in reports])
    ]
    out += _table(["減速の強さ", *_sub_header(labels)], body)

    out += ["", "## 3. 減速 × アクセル（基準の速度帯別 [km/h]）", ""]
    body = [
        [b] + [t for rep in reports for t in _cells_text(rep.decel_accel_bands.get(b))]
        for b in speed_band_order()
        if _shown([rep.decel_accel_bands.get(b) for rep in reports])
    ]
    out += _table(["速度帯", *_sub_header(labels)], body)

    out += ["", f"## 4. |偏差| > {DEV_LIMIT_KMH:g} km/h の行のうち + 側の割合", ""]
    body = [
        [lab, rep.over_count, rep.over_positive_count, f"{rep.positive_share:.2f}"]
        for lab, rep in zip(labels, reports, strict=True)
    ]
    out += _table(["CSV", "行数", "うち e>0", "割合"], body)
    return "\n".join(out)


# ─────────────────────────────────────────────────────────────────────
# 原因の切り分け（--part cause）
#   場面 = 速度帯 × 減速の強さ × ペダル。純関数（配列・行 → dataclass）と表示を分ける。
# ─────────────────────────────────────────────────────────────────────

# 指令からペダルが効くまでの遅れ [s]。データ由来: 155259 の格子ステップ
# （ProblemReport_20260929 段2。アクセル 0.5s・ブレーキ 0.3s）。固定。
LAG_ACCEL_S = 0.5
LAG_BRAKE_S = 0.3
PEDAL_LAG_S = {PEDAL_ACCEL: LAG_ACCEL_S, PEDAL_BRAKE: LAG_BRAKE_S}
# 実測加速度を出す窓 [s]。仮値: 減速 G ガバナーの判定窓（BRAKE_CONTRIB_WINDOW_S）に合わせる。
# ばらつきが大きければ広げる。
SLOPE_WINDOW_S = BRAKE_CONTRIB_WINDOW_S
SLOPE_MIN_POINTS = 3
# e のペダル選択の判定に使う先読み [s]（基準の傾き = (ref(t+1) − ref(t)) / 1）。仮値。
SELECT_LOOKAHEAD_S = 1.0
LOW_DECEL_LABEL = "0.3未満"  # 最小の区切り未満（惰行〜加速）
# 解釈の見出しだけに使う目印（仮値。判定には使わない）
MARK_FIT_PCT = 1.0  # 仮値: c の |中央値| がこれ以上なら C の目印 [%]
MARK_CONSIST_PCT = 0.5  # 仮値: d の |actual−指令| がこれ以上なら D の目印 [%]
TOP_COVER_SHARE = 0.8  # 仮値: 寄与の |合計| のこの割合に届くまで上位の場面を並べる
TOP_MAX_SCENES = 10  # 仮値: 1 CSV あたりの最大行数
MARK_CHOICE_SHARE = 0.2  # ペダル選択が妥当でない行の割合

CHOICE_BRAKE = "ブレーキが妥当"
CHOICE_COAST = "惰行が妥当"
CHOICE_OK = "妥当"

SceneKey = tuple[str, str, str]  # (ペダル, 速度帯, 減速の強さ)


def window_slope(t: np.ndarray, y: np.ndarray, centers: np.ndarray, window_s: float) -> np.ndarray:
    """中心 c の ±window_s/2 に入る点への最小二乗の傾き [y の単位/s]。

    窓がデータの範囲からはみ出す点・窓内の点が SLOPE_MIN_POINTS 未満の点は NaN。
    """
    out = np.full(len(centers), np.nan)
    half = window_s / 2.0
    eps = 1e-9
    lo_idx = np.searchsorted(t, centers - half - eps, side="left")
    hi_idx = np.searchsorted(t, centers + half + eps, side="right")
    for k, (c, lo, hi) in enumerate(zip(centers, lo_idx, hi_idx, strict=True)):
        if c - half < t[0] - eps or c + half > t[-1] + eps or hi - lo < SLOPE_MIN_POINTS:
            continue
        tt = t[lo:hi] - c
        yy = y[lo:hi]
        tc = tt - tt.mean()
        denom = float(np.dot(tc, tc))
        if denom > 0.0:
            out[k] = float(np.dot(tc, yy - yy.mean())) / denom
    return out


def opening_error_pct(err_kmhs: float, gain_kmhs_per_pct: float) -> float:
    """加速度の過不足 [km/h/s] → 開度の過不足 [%]（ゲイン [km/h/s per %] で割る）。"""
    return err_kmhs / gain_kmhs_per_pct


def scene_label(speed_kmh: float, accel_kmhs: float) -> tuple[str, str]:
    """(速度帯, 減速の強さ)。減速の強さは −加速度。最小の区切り未満は LOW_DECEL_LABEL。"""
    return speed_band_label(speed_kmh), decel_bin_label(-accel_kmhs) or LOW_DECEL_LABEL


def _median(v: Sequence[float]) -> float:
    return float(np.median(v)) if len(v) else float("nan")


def _mean(v: Sequence[float]) -> float:
    return float(np.mean(v)) if len(v) else float("nan")


@dataclass(frozen=True)
class OpeningStat:
    """a: 1 場面の開度の過不足（+ = アクセルは開きすぎ／ブレーキは弱すぎ）。"""

    time_s: float
    median_pct: float
    mean_pct: float
    mean_err_kmhs: float  # 実測加速度 − 基準の加速度（+ = 減速が足りない）

    @property
    def contribution_kmh(self) -> float:
        """寄与 = 時間 × 平均誤差 [km/h]（この場面が偏差に足した量の目安。+ = 遅れ方向）。"""
        return self.time_s * self.mean_err_kmhs


def top_contributors(
    stats: dict[SceneKey, OpeningStat],
    cover: float = TOP_COVER_SHARE,
    max_n: int = TOP_MAX_SCENES,
) -> list[SceneKey]:
    """|寄与| の大きい順に、|寄与| の合計の cover に届くまで（最大 max_n 件）の場面。"""
    ranked = sorted(
        (
            k
            for k, v in stats.items()
            if v.time_s >= MIN_ROW_TIME_S and not np.isnan(v.mean_err_kmhs)
        ),
        key=lambda k: -abs(stats[k].contribution_kmh),
    )
    total = sum(abs(stats[k].contribution_kmh) for k in ranked)
    out: list[SceneKey] = []
    acc = 0.0
    for k in ranked:
        if len(out) >= max_n or (total > 0 and acc >= cover * total):
            break
        out.append(k)
        acc += abs(stats[k].contribution_kmh)
    return out


def opening_errors(
    rows: Sequence[ModeRow], params: FeedforwardParams
) -> dict[SceneKey, OpeningStat]:
    """a: 減速 × (ACCEL|BRAKE) の各行の開度の過不足を場面ごとに集計する。"""
    if len(rows) < 2:
        return {}
    dt = sample_interval_s([r.t_s for r in rows])
    states = driving_states(rows)
    ref_a = ref_slopes(rows)
    t = np.array([r.t_s for r in rows])
    actual = np.array([r.actual_kmh for r in rows])
    ref = np.array([r.ref_kmh for r in rows])
    errs: dict[SceneKey, list[float]] = {}
    pcts: dict[SceneKey, list[float]] = {}
    for pedal, lag in PEDAL_LAG_S.items():
        ii = np.array(
            [i for i, r in enumerate(rows) if states[i] == STATE_DECEL and r.phase == pedal],
            dtype=int,
        )
        if len(ii) == 0:
            continue
        centers = t[ii] + lag
        a_meas = window_slope(t, actual, centers, SLOPE_WINDOW_S)
        a_ref = window_slope(t, ref, centers, SLOPE_WINDOW_S)
        for k, i in enumerate(ii):
            if np.isnan(a_meas[k]) or np.isnan(a_ref[k]):
                continue
            gain = pedal_gain_at(params, float(actual[i]), is_accel=pedal == PEDAL_ACCEL)
            if gain is None:
                continue
            err = float(a_meas[k] - a_ref[k])
            band, dbin = scene_label(float(ref[i]), float(ref_a[i]))
            key = (pedal, band, dbin)
            errs.setdefault(key, []).append(err)
            pcts.setdefault(key, []).append(opening_error_pct(err, gain))
    return {
        k: OpeningStat(len(v) * dt, _median(pcts[k]), _mean(pcts[k]), _mean(v))
        for k, v in errs.items()
    }


@dataclass
class TrainSet:
    """b・c: モデルが実際に学習する行（train_inverse_model_effective と同じ選び方）。"""

    pedal: str
    x: np.ndarray
    label_pct: np.ndarray
    speed_kmh: np.ndarray
    accel_kmhs: np.ndarray  # 実測加速度（遅れ込み。取れない行は NaN）
    dt_s: float


def training_sets(
    logs: Sequence[Any],
    accel_spec: FeatureSpec,
    brake_spec: FeatureSpec,
    accel_db: float,
    brake_db: float,
) -> dict[str, TrainSet]:
    """学習に使われる行を取り出す。

    `train_inverse_model_effective` の行の選び方（セッション別に `build_feature_matrix` で
    特徴量を作り、ラベル開度 ≥ 不感帯の行だけ残す）と同じ手順。あちらは関数の中に書き下されて
    いて呼べないので、同じ公開部品（group_by_session・estimate_offsets・build_feature_matrix）で
    組み直した。
    """
    parts: dict[str, list[tuple[np.ndarray, ...]]] = {PEDAL_ACCEL: [], PEDAL_BRAKE: []}
    dts: list[float] = []
    setups = (
        (PEDAL_ACCEL, accel_spec, accel_db, "accel_opening"),
        (PEDAL_BRAKE, brake_spec, brake_db, "brake_opening"),
    )
    for session in group_by_session(list(logs)):
        if len(session) < 2:
            continue
        speed = np.clip(np.array([lg.actual_speed_kmh for lg in session], dtype=float), 0.0, None)
        timestamps = [lg.timestamp for lg in session]
        t = np.array([ts.timestamp() for ts in timestamps])
        dts.append(sample_interval_s(list(t)))
        for pedal, spec, deadband, attr in setups:
            opening = np.array([getattr(lg, attr) for lg in session], dtype=float)
            x, idx = build_feature_matrix(
                speed,
                estimate_offsets(timestamps, spec.lookahead_horizons_s),
                estimate_offsets(timestamps, spec.past_horizons_s),
                spec,
            )
            if len(idx) == 0:
                continue
            label = opening[idx]
            mask = label >= deadband
            sel = idx[mask]
            slope = window_slope(t, speed, t[sel] + PEDAL_LAG_S[pedal], SLOPE_WINDOW_S)
            parts[pedal].append((x[mask], label[mask], speed[sel], slope))
    dt = float(np.median(dts)) if dts else 0.1
    out: dict[str, TrainSet] = {}
    for pedal, plist in parts.items():
        if not plist:
            continue
        out[pedal] = TrainSet(
            pedal,
            np.vstack([p[0] for p in plist]),
            np.concatenate([p[1] for p in plist]),
            np.concatenate([p[2] for p in plist]),
            np.concatenate([p[3] for p in plist]),
            dt,
        )
    return out


def training_coverage(ts: TrainSet) -> dict[tuple[str, str], float]:
    """b: 場面（速度帯, 減速の強さ）ごとの学習行の秒数。"""
    out: dict[tuple[str, str], float] = {}
    for v, a in zip(ts.speed_kmh, ts.accel_kmhs, strict=True):
        if np.isnan(a):
            continue
        key = scene_label(float(v), float(a))
        out[key] = out.get(key, 0.0) + ts.dt_s
    return out


def drive_scene_seconds(rows: Sequence[ModeRow]) -> dict[SceneKey, float]:
    """b: 走行 CSV の減速行を、基準ベースの場面 × 使ったペダル（phase）ごとに数えた秒数。"""
    if len(rows) < 2:
        return {}
    dt = sample_interval_s([r.t_s for r in rows])
    states = driving_states(rows)
    ref_a = ref_slopes(rows)
    out: dict[SceneKey, float] = {}
    for i, r in enumerate(rows):
        if states[i] != STATE_DECEL:
            continue
        band, dbin = scene_label(r.ref_kmh, float(ref_a[i]))
        key = (pedal_of(r.phase), band, dbin)
        out[key] = out.get(key, 0.0) + dt
    return out


@dataclass(frozen=True)
class FitStat:
    """c: 学習行での 予測 − 実開度 [%]。"""

    n: int
    time_s: float
    mean_pct: float
    median_pct: float


def training_fit(ts: TrainSet, model: Any) -> dict[tuple[str, str], FitStat]:  # noqa: ANN401
    """c: 学習行に対する pkl の素の予測（不感帯の切り上げ・クランプ前）− 実開度を場面ごとに。"""
    pred = np.asarray(model.predict(ts.x), dtype=float)
    resid: dict[tuple[str, str], list[float]] = {}
    for v, a, r in zip(ts.speed_kmh, ts.accel_kmhs, pred - ts.label_pct, strict=True):
        if np.isnan(a):
            continue
        resid.setdefault(scene_label(float(v), float(a)), []).append(float(r))
    return {
        k: FitStat(len(v), len(v) * ts.dt_s, _mean(v), _median(v)) for k, v in resid.items()
    }


def predict_on_trajectory(
    speed_kmh: np.ndarray, dt_s: float, spec: FeatureSpec, model: Any  # noqa: ANN401
) -> np.ndarray:
    """速度の軌跡から学習と同じ作り方（build_feature_matrix）で特徴量を作り、素の予測を返す。

    返り値は行ごとの長さ len(speed_kmh) の配列。未来／過去が足りない端は NaN。
    """
    speed = np.clip(np.asarray(speed_kmh, dtype=float), 0.0, None)
    offsets = [max(1, round(h / dt_s)) for h in spec.lookahead_horizons_s]
    past_offsets = [max(1, round(h / dt_s)) for h in spec.past_horizons_s]
    x, idx = build_feature_matrix(speed, offsets, past_offsets, spec)
    out = np.full(len(speed), np.nan)
    if len(idx):
        out[idx] = np.asarray(model.predict(x), dtype=float)
    return out


@dataclass(frozen=True)
class ConsistencyStat:
    """d: 1 場面の中央値。pred_actual/pred_ref は素の予測、cmd は実際に出した指令。"""

    time_s: float
    actual_minus_cmd_pct: float
    ref_minus_cmd_pct: float


@dataclass(frozen=True)
class SanityStat:
    """d の検算: 素の予測（基準軌跡）と記録された FF 出力の差。切り上げ行は除く。"""

    n: int
    max_abs_pct: float
    median_abs_pct: float


@dataclass
class ConsistencyReport:
    scenes: dict[SceneKey, ConsistencyStat] = field(default_factory=dict)
    sanity_decel: dict[str, SanityStat] = field(default_factory=dict)
    sanity_all: dict[str, SanityStat] = field(default_factory=dict)


def _sanity(diff: Sequence[float]) -> SanityStat:
    a = np.abs(np.asarray(diff, dtype=float))
    return SanityStat(len(a), float(a.max()) if len(a) else float("nan"), _median(list(a)))


def consistency(
    rows: Sequence[ModeRow],
    specs: dict[str, FeatureSpec],
    models: dict[str, Any],
    deadbands: dict[str, float],
) -> ConsistencyReport:
    """d: 減速 × (ACCEL|BRAKE) の行で、実車速の軌跡・基準の軌跡から作った予測と指令を比べる。

    指令は記録された最終指令（accel_cmd_pct / brake_cmd_pct）。FF 出力（検算用）は
    `ModeRow.ff_effort_pct`（+ アクセル / − ブレーキ）。FF は不感帯まで切り上げるので、
    検算は FF 出力が不感帯を超えた行（切り上げでない行）だけで取る。
    """
    rep = ConsistencyReport()
    if len(rows) < 2:
        return rep
    dt = sample_interval_s([r.t_s for r in rows])
    states = driving_states(rows)
    ref_a = ref_slopes(rows)
    actual = np.array([r.actual_kmh for r in rows])
    ref = np.array([r.ref_kmh for r in rows])
    cols: dict[SceneKey, list[tuple[float, float]]] = {}
    san_d: dict[str, list[float]] = {}
    san_a: dict[str, list[float]] = {}
    for pedal in (PEDAL_ACCEL, PEDAL_BRAKE):
        pa = predict_on_trajectory(actual, dt, specs[pedal], models[pedal])
        pr = predict_on_trajectory(ref, dt, specs[pedal], models[pedal])
        sign = 1.0 if pedal == PEDAL_ACCEL else -1.0
        for i, r in enumerate(rows):
            if r.phase != pedal or np.isnan(pa[i]) or np.isnan(pr[i]):
                continue
            cmd = r.accel_pct if pedal == PEDAL_ACCEL else r.brake_pct
            ff = sign * r.ff_effort_pct
            is_decel = states[i] == STATE_DECEL
            if ff > deadbands[pedal] + 1e-6:
                san_a.setdefault(pedal, []).append(float(pr[i]) - ff)
                if is_decel:
                    san_d.setdefault(pedal, []).append(float(pr[i]) - ff)
            if is_decel:
                band, dbin = scene_label(float(ref[i]), float(ref_a[i]))
                cols.setdefault((pedal, band, dbin), []).append(
                    (float(pa[i]) - cmd, float(pr[i]) - cmd)
                )
    for k, v in cols.items():
        rep.scenes[k] = ConsistencyStat(
            len(v) * dt, _median([a for a, _ in v]), _median([b for _, b in v])
        )
    rep.sanity_decel = {p: _sanity(v) for p, v in san_d.items()}
    rep.sanity_all = {p: _sanity(v) for p, v in san_a.items()}
    return rep


def classify_pedal_choice(
    demand_kmhs: float, free_kmhs: float, band_kmhs: float, pedal: str
) -> str:
    """e: 基準の要求加速度と惰行の加速度から、そのペダルの選択が妥当か。

    ACCEL の行: 要求 < 惰行 − 帯 なら「ブレーキが妥当」、|要求 − 惰行| < 帯 なら「惰行が妥当」。
    BRAKE の行: 要求 > 惰行 − 帯 なら「惰行が妥当」（惰行かアクセルでよかった）。
    """
    if pedal == PEDAL_ACCEL:
        if demand_kmhs < free_kmhs - band_kmhs:
            return CHOICE_BRAKE
        if abs(demand_kmhs - free_kmhs) < band_kmhs:
            return CHOICE_COAST
        return CHOICE_OK
    if pedal == PEDAL_BRAKE and demand_kmhs > free_kmhs - band_kmhs:
        return CHOICE_COAST
    return CHOICE_OK


@dataclass(frozen=True)
class ChoiceStat:
    """e: (ペダル, 速度帯) の集計。inc は part created と同じ増分（e[i+1] − e[i]）の合計。"""

    time_s: float
    brake_s: float  # 「ブレーキが妥当」の時間
    coast_s: float  # 「惰行が妥当」の時間
    brake_inc_kmh: float
    coast_inc_kmh: float
    ok_inc_kmh: float

    @property
    def wrong_share(self) -> float:
        return (self.brake_s + self.coast_s) / self.time_s if self.time_s > 0 else 0.0


def pedal_choice(
    rows: Sequence[ModeRow], params: FeedforwardParams, research: ResearchFFParams
) -> dict[tuple[str, str], ChoiceStat]:
    """e: 減速 × (ACCEL|BRAKE) の行のペダル選択が妥当だったかを速度帯ごとに数える。"""
    if len(rows) < 3:
        return {}
    dt = sample_interval_s([r.t_s for r in rows])
    states = driving_states(rows)
    n_ahead = max(1, round(SELECT_LOOKAHEAD_S / dt))
    acc: dict[tuple[str, str], list[float]] = {}
    for i in range(len(rows) - max(1, n_ahead)):
        r = rows[i]
        if states[i] != STATE_DECEL or r.phase not in (PEDAL_ACCEL, PEDAL_BRAKE):
            continue
        demand = (rows[i + n_ahead].ref_kmh - r.ref_kmh) / (n_ahead * dt)
        free = free_accel_at(params, research, r.ref_kmh)
        choice = classify_pedal_choice(demand, free, research.coast_band_kmhs, r.phase)
        inc = rows[i + 1].deviation_kmh - r.deviation_kmh
        a = acc.setdefault((r.phase, speed_band_label(r.ref_kmh)), [0.0] * 6)
        a[0] += dt
        if choice == CHOICE_BRAKE:
            a[1] += dt
            a[3] += inc
        elif choice == CHOICE_COAST:
            a[2] += dt
            a[4] += inc
        else:
            a[5] += inc
    return {k: ChoiceStat(*v) for k, v in acc.items()}


@dataclass
class ModelBundle:
    """pkl と設定を読み込んだ結果（mode_drive.load_feedforward と同じ手順）。"""

    model_path: str
    params: FeedforwardParams
    research: ResearchFFParams
    specs: dict[str, FeatureSpec]
    models: dict[str, Any]
    deadbands: dict[str, float]
    ff: CandidateFeedforward | None = None  # 読み込み済みの候補（--part recompute が使う）


def load_bundle(config_path: Path, model_path: str | None) -> ModelBundle:
    cfg = load_config(config_path)
    path = model_path or cfg.feedforward.model_path
    ff = make_candidate(cfg.feedforward.candidate)
    params = feedforward_params(cfg)
    research = research_ff_params(cfg)
    ff.set_params(params)
    ff.set_research_params(research)
    ff.load_model(path)
    with Path(path).open("rb") as f:  # 開発者がローカルで作った信頼済みファイル
        data = pickle.load(f)  # noqa: S301
    db = data.get("deadbands_pct") or {}
    return ModelBundle(
        model_path=path,
        params=params,
        research=research,
        specs={PEDAL_ACCEL: ff.accel_spec, PEDAL_BRAKE: ff.brake_spec},
        models={PEDAL_ACCEL: data["accel_model"], PEDAL_BRAKE: data["brake_model"]},
        deadbands={
            PEDAL_ACCEL: float(db.get("accel", params.accel_deadband_pct)),
            PEDAL_BRAKE: float(db.get("brake", params.brake_deadband_pct)),
        },
        ff=ff,
    )


@dataclass
class CauseResult:
    labels: list[str]
    opening: list[dict[SceneKey, OpeningStat]]
    train_seconds: dict[str, dict[tuple[str, str], float]]
    drive_seconds: list[dict[SceneKey, float]]
    fit: dict[str, dict[tuple[str, str], FitStat]]
    consistency: list[ConsistencyReport]
    choice: list[dict[tuple[str, str], ChoiceStat]]
    coast_band_kmhs: float
    model_path: str


def compute_cause(
    labels: Sequence[str],
    drive_rows: Sequence[Sequence[ModeRow]],
    train_logs: Sequence[Any],
    bundle: ModelBundle,
) -> CauseResult:
    sets = training_sets(
        train_logs,
        bundle.specs[PEDAL_ACCEL],
        bundle.specs[PEDAL_BRAKE],
        bundle.deadbands[PEDAL_ACCEL],
        bundle.deadbands[PEDAL_BRAKE],
    )
    return CauseResult(
        labels=list(labels),
        opening=[opening_errors(r, bundle.params) for r in drive_rows],
        train_seconds={p: training_coverage(ts) for p, ts in sets.items()},
        drive_seconds=[drive_scene_seconds(r) for r in drive_rows],
        fit={p: training_fit(ts, bundle.models[p]) for p, ts in sets.items()},
        consistency=[
            consistency(r, bundle.specs, bundle.models, bundle.deadbands) for r in drive_rows
        ],
        choice=[pedal_choice(r, bundle.params, bundle.research) for r in drive_rows],
        coast_band_kmhs=bundle.research.coast_band_kmhs,
        model_path=bundle.model_path,
    )


# ── 表示 ──


def _scene_order() -> list[tuple[str, str]]:
    return [(b, d) for b in speed_band_order() for d in [*decel_bin_labels(), LOW_DECEL_LABEL]]


def _f(v: float, spec: str = "+.1f") -> str:
    return "—" if v is None or np.isnan(v) else format(v, spec)


def format_cause(res: CauseResult) -> str:
    labels = res.labels
    out: list[str] = [f"モデル: {res.model_path}", ""]

    out += ["## a. 開度の過不足（減速 × ACCEL|BRAKE。+ = アクセル開きすぎ／ブレーキ弱すぎ）", ""]
    out += [
        f"実測加速度は実車速の {SLOPE_WINDOW_S:g}s 窓の傾き（窓の中心は t+遅れ。アクセル "
        f"{LAG_ACCEL_S:g}s・ブレーキ {LAG_BRAKE_S:g}s）。誤差 = 実測 − 基準の同じ窓の傾き "
        f"[km/h/s] を実速でのペダルゲインで割った値。{MIN_ROW_TIME_S:g}s 未満の行は隠す。",
        "",
    ]
    hdr = [h for lab in labels for h in (f"{lab} 時間[s]", f"{lab} 中央値[%]", f"{lab} 平均[%]",
                                          f"{lab} 平均誤差[km/h/s]", f"{lab} 寄与[km/h]")]
    body = []
    for pedal in (PEDAL_ACCEL, PEDAL_BRAKE):
        for band, dbin in _scene_order():
            stats = [o.get((pedal, band, dbin)) for o in res.opening]
            if max((s.time_s for s in stats if s), default=0.0) < MIN_ROW_TIME_S:
                continue
            cells: list[str] = []
            for s in stats:
                cells += ([f"{s.time_s:.1f}", _f(s.median_pct), _f(s.mean_pct),
                           _f(s.mean_err_kmhs, "+.2f"), _f(s.contribution_kmh)]
                          if s else ["0.0", "—", "—", "—", "—"])
            body.append([pedal, band, dbin, *cells])
    out += _table(["ペダル", "速度帯", "減速の強さ[km/h/s]", *hdr], body)

    out += ["", "## b. 学習データの網羅（155259 の効いている行）と WLTP の時間", ""]
    out += [
        "学習行の場面は「実速の速度帯 × 実測加速度（遅れ込み）の減速の強さ」。WLTP は走行 CSV の"
        "減速行を「基準の速度帯 × 基準の傾き」で数え、使ったペダル（phase）別に並べる。",
        "",
    ]
    hdr = ["学習 ACCEL[s]", "学習 BRAKE[s]"] + [
        h for lab in labels for h in (f"{lab} WLTP ACCEL[s]", f"{lab} WLTP BRAKE[s]")
    ]
    body = []
    for band, dbin in _scene_order():
        ta = res.train_seconds.get(PEDAL_ACCEL, {}).get((band, dbin), 0.0)
        tb = res.train_seconds.get(PEDAL_BRAKE, {}).get((band, dbin), 0.0)
        wl = [(d.get((PEDAL_ACCEL, band, dbin), 0.0), d.get((PEDAL_BRAKE, band, dbin), 0.0))
              for d in res.drive_seconds]
        if max([ta, tb, *[x for w in wl for x in w]]) < MIN_ROW_TIME_S:
            continue
        body.append([band, dbin, f"{ta:.1f}", f"{tb:.1f}",
                     *[f"{x:.1f}" for w in wl for x in w]])
    out += _table(["速度帯", "減速の強さ[km/h/s]", *hdr], body)

    out += ["", "## c. 学習データ上の当てはまり（pkl の素の予測 − 学習行の実開度 [%]）", ""]
    body = []
    for pedal in (PEDAL_ACCEL, PEDAL_BRAKE):
        for band, dbin in _scene_order():
            s = res.fit.get(pedal, {}).get((band, dbin))
            if s is None or s.time_s < MIN_ROW_TIME_S:
                continue
            body.append([pedal, band, dbin, s.n, f"{s.time_s:.1f}", _f(s.mean_pct),
                         _f(s.median_pct)])
    out += _table(["ペダル", "速度帯", "減速の強さ[km/h/s]", "行数", "時間[s]", "平均[%]",
                   "中央値[%]"], body)

    out += ["", "## d. 走行データ上の整合（中央値 [%]。予測 − 実際に出した指令）", ""]
    out += [
        "actual = 実車速の軌跡から学習と同じ作り方で特徴量を作った予測、ref = 基準の軌跡から"
        "作った予測（FF が見ていた入力）。指令は accel_cmd_pct / brake_cmd_pct（不感帯の切り上げ・"
        "ガバナー後の最終値）。",
        "",
    ]
    hdr = [h for lab in labels for h in (f"{lab} 時間[s]", f"{lab} actual−指令", f"{lab} ref−指令")]
    body = []
    for pedal in (PEDAL_ACCEL, PEDAL_BRAKE):
        for band, dbin in _scene_order():
            stats = [c.scenes.get((pedal, band, dbin)) for c in res.consistency]
            if max((s.time_s for s in stats if s), default=0.0) < MIN_ROW_TIME_S:
                continue
            cells = []
            for s in stats:
                cells += ([f"{s.time_s:.1f}", _f(s.actual_minus_cmd_pct),
                           _f(s.ref_minus_cmd_pct)] if s else ["0.0", "—", "—"])
            body.append([pedal, band, dbin, *cells])
    out += _table(["ペダル", "速度帯", "減速の強さ[km/h/s]", *hdr], body)
    out += ["", "検算: |ref の素の予測 − 記録された FF 出力|（FF 出力が不感帯を超えた行だけ）", ""]
    body = []
    for lab, c in zip(labels, res.consistency, strict=True):
        for name, d in (("減速行", c.sanity_decel), ("全状態", c.sanity_all)):
            for pedal in (PEDAL_ACCEL, PEDAL_BRAKE):
                s = d.get(pedal)
                if s:
                    body.append([lab, name, pedal, s.n, _f(s.median_abs_pct, ".3f"),
                                 _f(s.max_abs_pct, ".3f")])
    out += _table(["CSV", "対象", "ペダル", "行数", "中央値[%]", "最大[%]"], body)

    out += ["", "## e. ペダル選択（減速行のうち、基準の要求が惰行と食い違う行の割合）", ""]
    out += [
        f"要求 = (ref(t+{SELECT_LOOKAHEAD_S:g}) − ref(t)) / {SELECT_LOOKAHEAD_S:g} [km/h/s]、"
        "惰行 = "
        f"free_accel_at(基準の速度)、帯 coast_band_kmhs = {res.coast_band_kmhs:g}。"
        "ACCEL の行は「ブレーキが妥当」「惰行が妥当」、BRAKE の行は「惰行が妥当（惰行かアクセルで"
        "よかった）」を数える。増分は part created と同じ e[i+1] − e[i] の合計 [km/h]。",
        "",
    ]
    hdr = [h for lab in labels for h in (f"{lab} 時間[s]", f"{lab} ブレーキ妥当[%]",
                                          f"{lab} 惰行妥当[%]", f"{lab} 増分 ブレーキ妥当",
                                          f"{lab} 増分 惰行妥当", f"{lab} 増分 妥当")]
    body = []
    for pedal in (PEDAL_ACCEL, PEDAL_BRAKE):
        for band in speed_band_order():
            stats = [c.get((pedal, band)) for c in res.choice]
            if max((s.time_s for s in stats if s), default=0.0) < MIN_ROW_TIME_S:
                continue
            cells = []
            for s in stats:
                if s:
                    cells += [f"{s.time_s:.1f}", f"{100 * s.brake_s / s.time_s:.0f}",
                              f"{100 * s.coast_s / s.time_s:.0f}", _f(s.brake_inc_kmh),
                              _f(s.coast_inc_kmh), _f(s.ok_inc_kmh)]
                else:
                    cells += ["0.0"] + ["—"] * 5
            body.append([pedal, band, *cells])
    out += _table(["ペダル", "速度帯", *hdr], body)
    return "\n".join(out)


def format_interpretation(res: CauseResult) -> str:
    """寄与（時間 × 平均誤差）の大きい場面ごとに、A〜D のどれを数字が指すかを並べる。"""
    out = [
        "",
        "## 解釈の見出し（寄与の大きい場面。数字の並べ替えで、目印は仮値・判定ではない）",
        "",
        f"場面は a の 寄与[km/h] = 時間 × 平均誤差（+ = 遅れ方向）の |値| が大きい順に、"
        f"|寄与| の合計の {TOP_COVER_SHARE:.0%} に届くまで（最大 {TOP_MAX_SCENES} 件/CSV）。"
        f"目印: A = e の妥当でない割合 ≥ {MARK_CHOICE_SHARE:.0%}、B = 学習行の時間 < その CSV の"
        f"同ペダルの WLTP 時間、C = c の |中央値| ≥ {MARK_FIT_PCT:g}%、"
        f"D = c が小さく d の |actual−指令| ≥ {MARK_CONSIST_PCT:g}%。",
        "",
    ]
    for ci, lab in enumerate(res.labels):
        stats = res.opening[ci]
        top = top_contributors(stats)
        total = sum(abs(v.contribution_kmh) for v in stats.values() if v.time_s >= MIN_ROW_TIME_S)
        shown = sum(abs(stats[k].contribution_kmh) for k in top)
        out.append(f"### {lab}（|寄与| 合計 {total:.1f} km/h のうち表示分 {shown:.1f}）")
        out.append("")
        for pedal, band, dbin in top:
            a = stats[(pedal, band, dbin)]
            ch = res.choice[ci].get((pedal, band))
            tr = res.train_seconds.get(pedal, {}).get((band, dbin), 0.0)
            wl = res.drive_seconds[ci].get((pedal, band, dbin), 0.0)
            fit = res.fit.get(pedal, {}).get((band, dbin))
            cons = res.consistency[ci].scenes.get((pedal, band, dbin))
            tags = []
            if ch and ch.wrong_share >= MARK_CHOICE_SHARE:
                tags.append("A")
            if tr < wl:
                tags.append("B")
            c_big = fit is not None and abs(fit.median_pct) >= MARK_FIT_PCT
            if c_big:
                tags.append("C")
            d_big = cons is not None and abs(cons.actual_minus_cmd_pct) >= MARK_CONSIST_PCT
            if not c_big and d_big:
                tags.append("D")
            parts = [
                f"寄与 {_f(a.contribution_kmh)} km/h"
                f"（{a.time_s:.1f}s × {_f(a.mean_err_kmhs, '+.2f')}）",
                f"a {_f(a.median_pct)}%",
                f"e 妥当でない {100 * ch.wrong_share:.0f}%" if ch else "e —",
                f"b 学習 {tr:.1f}s / WLTP {wl:.1f}s",
                f"c {_f(fit.median_pct)}%（n={fit.n}）" if fit else "c —",
                f"d actual−指令 {_f(cons.actual_minus_cmd_pct)}%" if cons else "d —",
            ]
            out.append(
                f"- {pedal} {band} km/h × 減速 {dbin}: {' ／ '.join(parts)} "
                f"→ 指す先: {'・'.join(tags) if tags else '目印なし'}"
            )
        out.append("")
    return "\n".join(out)


# ─────────────────────────────────────────────────────────────────────
# ブレーキのタイミングのずれ（--part timing）
#   ブレーキ区間ごとに、指令・実位置・基準の減速度・実車の減速度の時間差を秒で測る。
#   純関数（配列 → 数値）と CSV の読み込み・表示を分ける。
# ─────────────────────────────────────────────────────────────────────

BRAKE_START_MIN_REF_KMH = 5.0  # 区間の開始: BRAKE に入った行の基準がこれより大きい
STOP_REF_KMH = 1.0  # 基準がこれ未満になったら「停車まで」。区間の終わりもここで切る
MIN_SEGMENT_S = 2.0  # これ未満の区間は捨てる
# 仮値: 区間の前後に足す時間 [s]。段1 の 6 の「離した後 3s」に合わせた。
# 相互相関の範囲。結果が区間の端に張り付いたら広げる。
PRE_S = 2.0
POST_S = 3.0
# 仮値: 相互相関のずらし幅 [s]。スクラッチで踏み始めの差が 0.7s 程度だったので余裕を見た。
# 端に張り付いたら広げる。
MAX_SHIFT_S = 1.5
PEAK_LEVEL = 0.5  # 山の 50% で踏み始め・離しを決める
MIN_XCORR_POINTS = 10  # 相互相関に使う重なりの最小点数
TRAIN_PAD_S = 1.0  # 学習ステップの窓の前後に足す時間 [s]

MID_LABEL = "途中の速度まで"
STOP_LABEL = "停車まで"

# (表示名, 先の波形, 後の波形)。時間差 = t(後) − t(先)。+ は後の波形が遅い。
TIMING_PAIRS: tuple[tuple[str, str, str], ...] = (
    ("基準の減速 → 実車の減速", "ref_dec", "act_dec"),
    ("指令 → 基準の減速", "cmd", "ref_dec"),
    ("指令 → 実車の減速", "cmd", "act_dec"),
    ("指令 → 実位置", "cmd", "pos"),
)
TIMING_HEADER = ["組み合わせ", "測り方", "件数", "中央値[s]", "四分位 25〜75%[s]", "範囲の端"]
MEASURE_ONSET = "踏み始め 50%"
MEASURE_RELEASE = "離し 50%"
MEASURE_XCORR = "相互相関"


@dataclass(frozen=True)
class BrakeRows:
    """MODE_DRIVE 行の時系列（列ごとの配列。取れない値は NaN）。"""

    t_s: np.ndarray
    ref_kmh: np.ndarray
    actual_kmh: np.ndarray
    brake_cmd_pct: np.ndarray
    brake_actual_pct: np.ndarray
    phase: list[str]


@dataclass(frozen=True)
class BrakeSegment:
    """ブレーキ区間 [i0, i1]（両端を含む行番号）。"""

    i0: int
    i1: int
    to_stop: bool  # True = 区間の後に基準が STOP_REF_KMH 未満になった


@dataclass(frozen=True)
class PairTiming:
    """1 区間・1 組の時間差 [s]。測れなかったものは None。"""

    onset_s: float | None
    release_s: float | None
    xcorr_s: float | None
    xcorr_edge: bool


def read_brake_rows(path: Path) -> BrakeRows:
    """モード走行 CSV から MODE_DRIVE 行を読む（`rows_from_csv` と同じ行選び）。"""
    cols: dict[str, list[float]] = {k: [] for k in ("t", "ref", "act", "cmd", "pos")}
    phase: list[str] = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("section") != SECTION_MODE_DRIVE or not r.get("mode_time_s"):
                continue
            pos = actual_opening(r, "brake")
            cols["t"].append(float(r["mode_time_s"]))
            cols["ref"].append(float(r["ref_speed_kmh"]))
            cols["act"].append(float(r["actual_speed_kmh"]))
            cols["cmd"].append(cmd_opening(r, "brake"))
            cols["pos"].append(math.nan if pos is None else pos)
            phase.append(r["phase"])
    a = {k: np.array(v, dtype=float) for k, v in cols.items()}
    return BrakeRows(a["t"], a["ref"], a["act"], a["cmd"], a["pos"], phase)


def find_brake_segments(
    phase: Sequence[str], ref_kmh: Sequence[float], t_s: Sequence[float]
) -> list[BrakeSegment]:
    """ブレーキ区間を探す。

    開始 = 基準が BRAKE_START_MIN_REF_KMH より大きい行で phase が BRAKE に入った行。
    終わり = phase が BRAKE を離れる直前、または基準が STOP_REF_KMH 未満になる直前の BRAKE 行。
    MIN_SEGMENT_S 未満は捨てる。終わりの次の行の基準が STOP_REF_KMH 未満なら「停車まで」。
    """
    n = len(phase)
    out: list[BrakeSegment] = []
    i = 0
    while i < n:
        if phase[i] != PEDAL_BRAKE:
            i += 1
            continue
        j = i  # BRAKE の連続区間 [i, run_end]
        while j + 1 < n and phase[j + 1] == PEDAL_BRAKE:
            j += 1
        run_end = j
        if ref_kmh[i] > BRAKE_START_MIN_REF_KMH:
            k = i
            while k + 1 <= run_end and ref_kmh[k + 1] >= STOP_REF_KMH:
                k += 1
            nxt = min(k + 1, n - 1)
            to_stop = bool(ref_kmh[nxt] < STOP_REF_KMH)
            if t_s[k] - t_s[i] >= MIN_SEGMENT_S:
                out.append(BrakeSegment(i, k, to_stop))
        i = run_end + 1
    return out


def decel_series(t_s: np.ndarray, speed_kmh: np.ndarray) -> np.ndarray:
    """減速度 [km/h/s] = −(±LOCAL_HALF_WINDOW_S の最小二乗の傾き)。端・欠測は NaN。"""
    return -window_slope(t_s, speed_kmh, t_s, 2.0 * LOCAL_HALF_WINDOW_S)


def peak_crossings(
    t: np.ndarray, y: np.ndarray, level: float = PEAK_LEVEL
) -> tuple[float, float] | None:
    """山の level 倍を越えた時刻 (踏み始め, 離し)。線形補間。どちらかが無ければ None。

    踏み始め = 山から過去へ戻って、最後に山の level 倍未満だった点と次の点の間。
    離し = 山から未来へ進んで、最初に山の level 倍未満になった点と 1 つ前の点の間。
    山が 0 以下・窓の端まで level 倍以上のままのときは None。
    """
    ok = np.isfinite(y)
    if not ok.any():
        return None
    ip = int(np.nanargmax(np.where(ok, y, -np.inf)))
    peak = float(y[ip])
    if peak <= 0.0:
        return None
    thr = level * peak
    a = ip
    while a > 0 and ok[a - 1] and y[a - 1] >= thr:
        a -= 1
    if a == 0 or not ok[a - 1]:
        return None
    b = ip
    while b < len(y) - 1 and ok[b + 1] and y[b + 1] >= thr:
        b += 1
    if b == len(y) - 1 or not ok[b + 1]:
        return None

    def cross(i_lo: int, i_hi: int) -> float:
        y0, y1 = float(y[i_lo]), float(y[i_hi])
        return float(t[i_lo] + (thr - y0) / (y1 - y0) * (t[i_hi] - t[i_lo]))

    return cross(a - 1, a), cross(b + 1, b)


def xcorr_shift(
    t: np.ndarray, first: np.ndarray, second: np.ndarray, max_shift_s: float = MAX_SHIFT_S
) -> tuple[float, bool] | None:
    """相関が最大になるずらし量 [s] と、範囲の端かどうか。+ は second が first より遅い。

    t の中央値の刻みの等間隔格子に np.interp で載せ、second(t + s) と first(t) の
    ピアソン相関が最大の s（格子の刻みの整数倍で ±max_shift_s）を選ぶ。欠測は除いて補間する。
    重なりが MIN_XCORR_POINTS 未満・どちらかが一定の s は候補から外す。
    """
    fa, fb = np.isfinite(first), np.isfinite(second)
    if fa.sum() < 2 or fb.sum() < 2 or len(t) < 2:
        return None
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        return None
    grid = np.arange(t[0], t[-1] + dt / 2, dt)
    a = np.interp(grid, t[fa], first[fa], left=np.nan, right=np.nan)
    b = np.interp(grid, t[fb], second[fb], left=np.nan, right=np.nan)
    n_max = int(round(max_shift_s / dt))
    best: tuple[float, int] | None = None
    for k in range(-n_max, n_max + 1):
        if k >= 0:
            x, y = a[: len(a) - k], b[k:]
        else:
            x, y = a[-k:], b[: len(b) + k]
        m = np.isfinite(x) & np.isfinite(y)
        if m.sum() < MIN_XCORR_POINTS:
            continue
        xs, ys = x[m] - x[m].mean(), y[m] - y[m].mean()
        den = math.sqrt(float(np.dot(xs, xs) * np.dot(ys, ys)))
        if den <= 1e-12:
            continue
        r = float(np.dot(xs, ys)) / den
        if best is None or r > best[0]:
            best = (r, k)
    if best is None:
        return None
    return best[1] * dt, abs(best[1]) >= n_max


def pair_timing(t: np.ndarray, first: np.ndarray, second: np.ndarray) -> PairTiming:
    """2 本の波形の時間差（後 − 先）。"""
    c1, c2 = peak_crossings(t, first), peak_crossings(t, second)
    onset = c2[0] - c1[0] if c1 and c2 else None
    release = c2[1] - c1[1] if c1 and c2 else None
    xc = xcorr_shift(t, first, second)
    return PairTiming(onset, release, xc[0] if xc else None, bool(xc and xc[1]))


def segment_timings(rows: BrakeRows) -> list[tuple[BrakeSegment, dict[str, PairTiming]]]:
    """各ブレーキ区間の [start − PRE_S, end + POST_S] で 4 組の時間差を測る。

    波形は区間の前（窓の頭〜開始）の中央値を引いてから山を探す。
    """
    t = rows.t_s
    if len(t) < 3:
        return []
    waves = {
        "cmd": rows.brake_cmd_pct,
        "pos": rows.brake_actual_pct,
        "ref_dec": decel_series(t, rows.ref_kmh),
        "act_dec": decel_series(t, rows.actual_kmh),
    }
    out = []
    for seg in find_brake_segments(rows.phase, rows.ref_kmh, t):
        m = (t >= t[seg.i0] - PRE_S) & (t <= t[seg.i1] + POST_S)
        tw = t[m]
        pre = tw < t[seg.i0]
        # ブレーキ指令は区間の前も 5.8% 前後の下駄（惰行の保持）を持つので、区間の前の
        # 中央値を引いてから山の 50% を測る（相互相関は定数を足しても変わらない）
        w = {}
        for name, y in waves.items():
            yw = y[m]
            base = float(np.nanmedian(yw[pre])) if pre.any() and np.isfinite(yw[pre]).any() else 0.0
            w[name] = yw - base
        out.append((
            seg,
            {name: pair_timing(tw, w[a], w[b]) for name, a, b in TIMING_PAIRS},
        ))
    return out


def train_brake_timings(csv_path: Path) -> list[PairTiming]:
    """学習データ（格子ステップ走行）のブレーキのステップごとの「指令 → 実車の減速」。

    ブレーキのステップ = HOLD_STEP の連続区間のうち、MIN_STEP_S 以上で、ブレーキ指令が
    MIN_CMD_CHANGE_PCT 以上増えて 0 より大きいまま終わるもの。窓は [開始 − TRAIN_PAD_S、
    終了 + TRAIN_PAD_S]。ステップの前の値（窓の頭の平均）を引いてから測り、減速度の山が
    MIN_ACCEL_CHANGE_KMHS 未満のステップは捨てる（pedal_lag と同じ足切り）。離しは測らない。
    """
    rows = read_step_rows(csv_path)
    t, cmd = rows.t_s, rows.brake_cmd_pct
    if len(t) < 3:
        return []
    decel = decel_series(t, rows.speed_kmh)
    out: list[PairTiming] = []
    for i0, i1 in _hold_step_segments(rows.phase):
        if i0 < 1 or t[i1 - 1] - t[i0] < MIN_STEP_S:
            continue
        if cmd[i1 - 1] <= 0 or cmd[i1 - 1] - cmd[i0 - 1] < MIN_CMD_CHANGE_PCT:
            continue
        m = (t >= t[i0] - TRAIN_PAD_S) & (t <= t[i1 - 1] + TRAIN_PAD_S)
        pre = m & (t < t[i0])
        tw, c, d = t[m], cmd[m].copy(), decel[m].copy()
        if pre.sum() < 2 or not np.isfinite(decel[pre]).any():
            continue
        c -= float(np.mean(cmd[pre]))
        d -= float(np.nanmean(decel[pre]))
        if not np.isfinite(d).any() or np.nanmax(d) < MIN_ACCEL_CHANGE_KMHS:
            continue
        # 離しは測らないので、離し側が窓の端まで続くステップも踏み始めは採る
        onset = _onset_only(tw, c), _onset_only(tw, d)
        xc = xcorr_shift(tw, c, d)
        out.append(PairTiming(
            onset[1] - onset[0] if None not in onset else None,
            None,
            xc[0] if xc else None,
            bool(xc and xc[1]),
        ))
    return out


def _onset_only(t: np.ndarray, y: np.ndarray, level: float = PEAK_LEVEL) -> float | None:
    """山の level 倍を越えた時刻（踏み始めだけ）。`peak_crossings` の離し側を見ない版。"""
    ok = np.isfinite(y)
    if not ok.any():
        return None
    ip = int(np.nanargmax(np.where(ok, y, -np.inf)))
    peak = float(y[ip])
    if peak <= 0.0:
        return None
    thr = level * peak
    a = ip
    while a > 0 and ok[a - 1] and y[a - 1] >= thr:
        a -= 1
    if a == 0 or not ok[a - 1]:
        return None
    y0, y1 = float(y[a - 1]), float(y[a])
    return float(t[a - 1] + (thr - y0) / (y1 - y0) * (t[a] - t[a - 1]))


def _quartiles(v: Sequence[float]) -> tuple[float, float]:
    if not len(v):
        return float("nan"), float("nan")
    return float(np.percentile(v, 25)), float(np.percentile(v, 75))


def _timing_rows(
    timings: Sequence[PairTiming],
) -> tuple[list[float], list[float], list[float], int]:
    onset = [x.onset_s for x in timings if x.onset_s is not None]
    release = [x.release_s for x in timings if x.release_s is not None]
    xc = [x.xcorr_s for x in timings if x.xcorr_s is not None]
    return onset, release, xc, sum(x.xcorr_edge for x in timings)


def _stat_cells(v: Sequence[float]) -> list[str]:
    q1, q3 = _quartiles(v)
    return [str(len(v)), _f(_median(v), "+.2f"), f"{_f(q1, '+.2f')} 〜 {_f(q3, '+.2f')}"]


def _timing_table(per_pair: dict[str, list[PairTiming]]) -> list[str]:
    body = []
    for name, _, _ in TIMING_PAIRS:
        onset, release, xc, edge = _timing_rows(per_pair.get(name, []))
        body.append([name, MEASURE_ONSET, *_stat_cells(onset), ""])
        body.append([name, MEASURE_RELEASE, *_stat_cells(release), ""])
        body.append([name, MEASURE_XCORR, *_stat_cells(xc), f"{edge}/{len(xc)}" if xc else ""])
    return _table(TIMING_HEADER, body)


def format_timing(
    labels: Sequence[str],
    results: Sequence[list[tuple[BrakeSegment, dict[str, PairTiming]]]],
    train: list[PairTiming] | None = None,
    train_label: str = "",
    train_pedal_lag: tuple[int, float] | None = None,
) -> str:
    out: list[str] = [
        "## ブレーキのタイミングのずれ（秒。時間差 = 後の波形 − 先の波形）",
        "",
        f"区間の窓 = [開始 − {PRE_S:g}s, 終了 + {POST_S:g}s]、相互相関のずらし幅 ±{MAX_SHIFT_S:g}s"
        f"、減速度は ±{LOCAL_HALF_WINDOW_S:g}s の傾き。",
        "",
    ]
    for lab, res in zip(labels, results, strict=True):
        for to_stop, title in ((False, MID_LABEL), (True, STOP_LABEL)):
            segs = [tm for seg, tm in res if seg.to_stop == to_stop]
            note = "" if not to_stop else "（参考）"
            out += [f"### {lab} {title}{note}（区間 {len(segs)} 件）", ""]
            per_pair = {
                name: [tm[name] for tm in segs] for name, _, _ in TIMING_PAIRS
            }
            out += _timing_table(per_pair)
            out.append("")
    if train is not None:
        out += [f"### 学習データ {train_label}（ブレーキのステップ {len(train)} 件）", ""]
        onset = [x.onset_s for x in train if x.onset_s is not None]
        xc = [x.xcorr_s for x in train if x.xcorr_s is not None]
        edge = sum(x.xcorr_edge for x in train)
        out += _table(
            TIMING_HEADER,
            [
                ["指令 → 実車の減速", MEASURE_ONSET, *_stat_cells(onset), ""],
                ["指令 → 実車の減速", MEASURE_XCORR, *_stat_cells(xc), f"{edge}/{len(xc)}"],
            ],
        )
        if train_pedal_lag is not None:
            out += [
                "",
                f"参考: pedal_lag の既存の値（ブレーキ、指令 50% 点 → 加速度 50% 点）"
                f" 件数 {train_pedal_lag[0]}・中央値 {train_pedal_lag[1]:+.2f}s"
                "（測り方が違うので、上の表と同じ物差しでは並ばない）",
            ]
        out.append("")
    out += [
        "読み方: 「A → B: +x」は B が A より x 秒遅い（A が先）。"
        "例えば「指令 → 基準の減速: +0.65」は指令が基準の減速より 0.65 秒早い。"
        "「基準の減速 → 実車の減速」が − なら実車の減速が基準より早い。"
        "踏み始め・離しは各波形の山の 50% を越えた時刻の差、相互相関は区間全体の形が最も"
        "合うずらし量。範囲の端 = ずらし幅の端に張り付いた件数（増えたら幅を広げる）。",
    ]
    return "\n".join(out)


# ─────────────────────────────────────────────────────────────────────
# 指令の再計算（--part recompute）
#   C1 の FF の入力は基準車速だけなので、ログの基準から指令を作り直せる。ブレーキモデル入力の
#   遅らせ d・ペダル選択の窓（幅 H・中心 L）を変えたとき、指令が基準の減速に対してどの時刻に
#   立ち上がるか・往復が悪化しないかを見る。車の応答は含まない（採否は実機で決める）。
#   mode_drive の `_ff_inputs`（C1 経路）・`_select_kwargs`・`split_effort` と同じ組み立てを
#   純関数で持つ（mode_drive は HW 依存が重いので import しない）。
# ─────────────────────────────────────────────────────────────────────

DELAY_CANDIDATES_S = (0.1, 0.2, 0.3, 0.4, 0.5)  # 仮値: 段1b で先読みが 0.4〜0.5s 早すぎたため
WIDTH_CANDIDATES_S = (2.0, 1.5, 1.0)  # 仮値: 今の H=3.0 は手で決めた値
CENTER_CANDIDATES_S = (0.1, 0.0)  # 仮値: 今の L=0.3（格子ステップの指令→加速度 50% の遅れ）
BRAKE_CENTER_CANDIDATES_S = (0.2, 0.1, 0.0)  # 仮値: ブレーキの判定だけの窓の中心 L_b（段2-2）
ACCEL_DIFF_EPS_PCT = 1e-9  # アクセル FF が「違う」とみなす差 [%]
RECOMPUTE_HEADER = [
    "設定", "区間", "指令→基準の減速 踏み始め[s]", "同 相互相関[s]",
    "選択の切替→基準の減速 50%[s]", "ブレーキ離し→基準の減速 離し50%[s]",
    "アクセル往復 全体[回/s]", "同 60s最大[回/s]",
    "アクセル⇔ブレーキ切替[回]", "アクセルの行の違い[行]",
]


@dataclass(frozen=True)
class RecomputeSetting:
    """再計算の設定。delay_s はブレーキモデル入力の遅らせ、center/width はペダル選択の窓。
    brake_center_s はブレーキの判定だけの窓の中心 L_b（None なら L と同じ。window のみ）。"""

    label: str
    delay_s: float
    center_s: float
    width_s: float
    brake_center_s: float | None = None


@dataclass(frozen=True)
class RecomputeDrive:
    """再計算に要る走行設定（config から取り出したもの）。"""

    select_mode: str  # "window" / "point"
    point_s: float
    max_accel_pct: float
    max_brake_pct: float
    accel_standby_pct: float
    brake_standby_pct: float


@dataclass(frozen=True)
class LogFF:
    """ログの FF 列（待機位置の前・ペダル別）。MODE_DRIVE 行だけ（read_brake_rows と同じ）。"""

    accel_ff_pct: np.ndarray
    brake_ff_pct: np.ndarray
    accel_cmd_pct: np.ndarray


@dataclass(frozen=True)
class Recomputed:
    """再計算した FF（待機位置の前）と、待機位置で下限を掛けた指令。"""

    accel_ff_pct: np.ndarray
    brake_ff_pct: np.ndarray
    accel_cmd_pct: np.ndarray
    brake_cmd_pct: np.ndarray
    phase: list[str]  # ACCEL / BRAKE / COAST（再計算した FF の符号。待機位置の前）


def read_log_ff(path: Path) -> LogFF:
    """モード走行 CSV の FF 列とアクセル指令（`read_brake_rows` と同じ行選び）。"""
    a: list[float] = []
    b: list[float] = []
    c: list[float] = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("section") != SECTION_MODE_DRIVE or not r.get("mode_time_s"):
                continue
            a.append(float(r["accel_ff_pct"]) if r.get("accel_ff_pct") else math.nan)
            b.append(float(r["brake_ff_pct"]) if r.get("brake_ff_pct") else math.nan)
            c.append(cmd_opening(r, "accel"))
    return LogFF(np.array(a), np.array(b), np.array(c))


def recompute_drive_from_config(cfg: Any) -> RecomputeDrive:  # noqa: ANN401
    """ResearchConfig から再計算の走行設定を取り出す（待機位置は standby_openings と同じ）。"""
    ff = cfg.feedforward
    # 2026-10-08 段2l-6: 待機位置は vehicle.min_*_opening_pct（pedal_standby=false なら 0）
    accel_sb = cfg.vehicle.min_accel_opening_pct if cfg.mode_drive.pedal_standby else 0.0
    brake_sb = cfg.vehicle.min_brake_opening_pct if cfg.mode_drive.pedal_standby else 0.0
    return RecomputeDrive(
        ff.pedal_select_mode,
        ff.pedal_select_point_s,
        cfg.vehicle.max_accel_opening_pct,
        cfg.vehicle.max_brake_opening_pct,
        accel_sb,
        brake_sb,
    )


def _window_slope_of(ref_at: Any, t_s: float, center_s: float, width_s: float) -> float:  # noqa: ANN401
    """窓 [t+center±width/2] の基準の最小二乗の傾き（window_slope_kmhs と同じ計算）。"""
    step = 0.1
    n = max(1, round(width_s / step))
    start = t_s + center_s - width_s / 2.0
    ts = [k * step for k in range(n + 1)]
    vs = [ref_at(start + x) for x in ts]
    t_mean, v_mean = sum(ts) / len(ts), sum(vs) / len(vs)
    denom = sum((x - t_mean) ** 2 for x in ts)
    return sum((x - t_mean) * (v - v_mean) for x, v in zip(ts, vs, strict=True)) / denom


def split_effort_pct(
    effort: float, max_accel_pct: float, max_brake_pct: float
) -> tuple[float, float]:
    """符号付きの努力量 → (アクセル, ブレーキ)。mode_drive.split_effort と同じ振り分け。"""
    if effort > 0.0:
        return min(effort, max_accel_pct), 0.0
    if effort < 0.0:
        return 0.0, min(-effort, max_brake_pct)
    return 0.0, 0.0


def recompute_commands(
    ff: CandidateFeedforward,
    t_s: np.ndarray,
    ref_kmh: np.ndarray,
    drive: RecomputeDrive,
    setting: RecomputeSetting,
) -> Recomputed:
    """ログの基準（線形補間・両端は一定）から各行の FF を計算し直す（C1〜C3 の経路）。

    入力は mode_drive._ff_inputs と同じ: v0 = ref(t)、future = ref(t+h)、past = ref(t−h)。
    ペダル選択は mode_drive._select_kwargs と同じ（window は窓の傾きと ref(t+L)＋L_b 指定時は
    L_b の同じ 2 つ、point は (ref(t+hp) − v0)/hp）。setting.delay_s > 0 のときはブレーキモデル
    だけ ref を delay_s 遅らせた入力（brake_inputs）で評価する。
    """
    if ff.uses_actual_speed:
        raise ValueError("再計算は基準だけで入力が決まる C1〜C3 に対応（C4・C5 は実測が要る）")

    def ref_at(x: float) -> float:
        return float(np.interp(x, t_s, ref_kmh))

    hs, phs = ff.horizons, ff.past_horizons
    n = len(t_s)
    accel_ff, brake_ff = np.zeros(n), np.zeros(n)
    phase: list[str] = []
    d = setting.delay_s
    for i in range(n):
        t = float(t_s[i])
        v0 = ref_at(t)
        future = [ref_at(t + h) for h in hs]
        past = [ref_at(t - h) for h in phs]
        if drive.select_mode == "window":
            sel = {
                "select_accel_kmhs": _window_slope_of(ref_at, t, setting.center_s, setting.width_s),
                "select_speed_kmh": ref_at(t + setting.center_s),
            }
            if setting.brake_center_s is not None:  # mode_drive._select_kwargs と同じ
                lb = setting.brake_center_s
                sel["select_brake_accel_kmhs"] = _window_slope_of(ref_at, t, lb, setting.width_s)
                sel["select_brake_speed_kmh"] = ref_at(t + lb)
        else:
            sel = {"select_accel_kmhs": (ref_at(t + drive.point_s) - v0) / drive.point_s}
        brake_inputs = None
        if d > 0.0:
            brake_inputs = (
                ref_at(t - d),
                [ref_at(t - d + h) for h in hs],
                [ref_at(t - d - h) for h in phs],
            )
        effort = ff.predict_effort(v0, future, past, brake_inputs=brake_inputs, **sel)
        accel_ff[i], brake_ff[i] = split_effort_pct(
            effort, drive.max_accel_pct, drive.max_brake_pct
        )
        phase.append(
            PEDAL_ACCEL if accel_ff[i] > 0.0 else PEDAL_BRAKE if brake_ff[i] > 0.0 else PEDAL_COAST
        )
    return Recomputed(
        accel_ff,
        brake_ff,
        np.maximum(accel_ff, drive.accel_standby_pct),
        np.maximum(brake_ff, drive.brake_standby_pct),
        phase,
    )


def count_pedal_switches(phase: Sequence[str]) -> int:
    """アクセル⇔ブレーキの切替回数。COAST（どちらも踏まない）は飛ばして、直前に踏んでいた
    ペダルと違うペダルに変わった回数を数える（ACCEL → COAST → BRAKE も 1 回）。"""
    count = 0
    last: str | None = None
    for p in phase:
        if p == PEDAL_COAST or p not in (PEDAL_ACCEL, PEDAL_BRAKE):
            continue
        if last is not None and p != last:
            count += 1
        last = p
    return count


def switch_timings(
    rows: BrakeRows, phase: Sequence[str]
) -> list[float | None]:
    """途中の速度まで の各区間で「選択の切替 → 基準の減速 50%」の時間差 [s]（= 基準の減速の踏み始め
    − 再計算したペダルが最初にブレーキへ切り替わった時刻。+ は切替のほうが先）。

    区間の窓は segment_timings と同じ（[開始 − PRE_S, 終了 + POST_S]）。切替 = 窓の中で前の行が
    BRAKE でなく、その行が BRAKE になった最初の行（窓の頭から BRAKE の場合は切替なし = None）。
    基準の減速は区間の前の中央値を引いてから山の 50% を測る。測れなければ None。
    """
    t = rows.t_s
    if len(t) < 3:
        return []
    ref_dec = decel_series(t, rows.ref_kmh)
    out: list[float | None] = []
    for seg in find_brake_segments(rows.phase, rows.ref_kmh, t):
        if seg.to_stop:
            continue
        idx = np.flatnonzero((t >= t[seg.i0] - PRE_S) & (t <= t[seg.i1] + POST_S))
        tw, yw = t[idx], ref_dec[idx]
        pre = tw < t[seg.i0]
        base = float(np.nanmedian(yw[pre])) if pre.any() and np.isfinite(yw[pre]).any() else 0.0
        cross = peak_crossings(tw, yw - base)
        sw = next(
            (
                float(t[j])
                for j in idx[1:]
                if phase[j] == PEDAL_BRAKE and phase[j - 1] != PEDAL_BRAKE
            ),
            None,
        )
        out.append(cross[0] - sw if cross is not None and sw is not None else None)
    return out


def release_timings(rows: BrakeRows, phase: Sequence[str]) -> list[float | None]:
    """途中の速度まで の各区間で「選択のブレーキ離し → 基準の減速の離し 50%」の時間差 [s]
    （= 基準の減速の離し − 再計算したペダルが BRAKE から抜けた時刻。+ は選択のほうが先に離す。
    switch_timings と同じ符号の向き: 基準の側の時刻 − 選択の側の時刻）。

    区間の窓・基準の減速の波形は switch_timings と同じ。離し = 窓の中で、最初の切替（前の行が
    BRAKE でなく、その行が BRAKE）以降に、前の行が BRAKE でその行が BRAKE でなくなった最初の行。
    測れなければ None。
    """
    t = rows.t_s
    if len(t) < 3:
        return []
    ref_dec = decel_series(t, rows.ref_kmh)
    out: list[float | None] = []
    for seg in find_brake_segments(rows.phase, rows.ref_kmh, t):
        if seg.to_stop:
            continue
        idx = np.flatnonzero((t >= t[seg.i0] - PRE_S) & (t <= t[seg.i1] + POST_S))
        tw, yw = t[idx], ref_dec[idx]
        pre = tw < t[seg.i0]
        base = float(np.nanmedian(yw[pre])) if pre.any() and np.isfinite(yw[pre]).any() else 0.0
        cross = peak_crossings(tw, yw - base)
        entered = False
        rel = None
        for j in idx[1:]:
            if phase[j] == PEDAL_BRAKE and phase[j - 1] != PEDAL_BRAKE:
                entered = True
            elif entered and phase[j] != PEDAL_BRAKE and phase[j - 1] == PEDAL_BRAKE:
                rel = float(t[j])
                break
        out.append(cross[1] - rel if cross is not None and rel is not None else None)
    return out


def accel_row_diff(rec: Recomputed, baseline: Recomputed) -> int:
    """アクセルの行がベースラインと違う行数（ペダルの ACCEL 判定が違う、または accel_ff が
    1e-9% を超えて違う行）。L_b の行は 0 でなければならない（アクセルは L の窓で判定するため）。"""
    is_a = np.array([p == PEDAL_ACCEL for p in rec.phase])
    is_a0 = np.array([p == PEDAL_ACCEL for p in baseline.phase])
    value_differs = np.abs(rec.accel_ff_pct - baseline.accel_ff_pct) > ACCEL_DIFF_EPS_PCT
    differ = (is_a != is_a0) | value_differs
    return int(np.sum(differ))


@dataclass(frozen=True)
class RecomputeResult:
    """1 設定分の結果。"""

    setting: RecomputeSetting
    onset_s: list[float]
    xcorr_s: list[float]
    n_segments: int
    switch_s: list[float]
    reversal_overall: float
    reversal_window_max: float
    switches: int
    release_s: list[float] = field(default_factory=list)
    accel_diff_rows: int = 0


def evaluate_recompute(
    rows: BrakeRows,
    rec: Recomputed,
    setting: RecomputeSetting,
    kpi: Any,  # noqa: ANN401
    baseline: Recomputed | None = None,
) -> RecomputeResult:
    """再計算した指令を、`--part timing` と同じ測り方・KPI で評価する。

    ブレーキ指令は rows.brake_cmd_pct を再計算値（待機位置の下限つき）に置き換え、ログの phase で
    「途中の速度まで」の区間を取る。アクセルの往復は再計算したペダル（phase）・アクセル指令で数える。
    baseline（ベースラインの再計算結果）を渡すと、アクセルの行の違い（accel_row_diff）も数える。
    """
    timed = segment_timings(replace(rows, brake_cmd_pct=rec.brake_cmd_pct))
    mid = [tm["指令 → 基準の減速"] for seg, tm in timed if not seg.to_stop]
    onset, _, xc, _ = _timing_rows(mid)
    overall, win_max = pedal_reversal_rates(
        rows.t_s, rec.accel_cmd_pct, rows.ref_kmh, rec.phase,
        hyst_pct=kpi.pedal_reversal_hyst_pct,
        window_s=kpi.pedal_reversal_window_s,
        min_window_active_s=kpi.pedal_reversal_min_window_s,
    )
    sw = [x for x in switch_timings(rows, rec.phase) if x is not None]
    rel = [x for x in release_timings(rows, rec.phase) if x is not None]
    n_diff = accel_row_diff(rec, baseline) if baseline is not None else 0
    return RecomputeResult(
        setting, onset, xc, len(mid), sw, overall, win_max, count_pedal_switches(rec.phase),
        rel, n_diff,
    )


def recompute_settings(base: RecomputeSetting) -> list[RecomputeSetting]:
    """ベースライン + 1 つずつ変えた設定（d → H → L → L_b の順。他はベースライン）。"""
    out = [replace(base, label="ベースライン")]
    out += [replace(base, label=f"d={d:g}s", delay_s=d) for d in DELAY_CANDIDATES_S]
    out += [replace(base, label=f"H={h:g}s", width_s=h) for h in WIDTH_CANDIDATES_S]
    out += [replace(base, label=f"L={c:g}s", center_s=c) for c in CENTER_CANDIDATES_S]
    out += [
        replace(base, label=f"L_b={c:g}s", brake_center_s=c) for c in BRAKE_CENTER_CANDIDATES_S
    ]
    return out


def baseline_diff(rec: Recomputed, log: LogFF) -> tuple[float, float, int, int]:
    """再計算とログの FF（待機位置の前）の差。(ブレーキの最大 |差|, アクセルの最大 |差|,
    どちらかが 0.1% を超えた行数, 行数) [%]。欠測行は除く。"""
    db = np.abs(rec.brake_ff_pct - log.brake_ff_pct)
    da = np.abs(rec.accel_ff_pct - log.accel_ff_pct)

    def mx(d: np.ndarray) -> float:
        return float(np.nanmax(d)) if np.isfinite(d).any() else math.nan

    over = int(np.sum((np.nan_to_num(db) > 0.1) | (np.nan_to_num(da) > 0.1)))
    return mx(db), mx(da), over, len(db)


def _med_q(v: Sequence[float]) -> str:
    if not len(v):
        return "—"
    q1, q3 = _quartiles(v)
    return f"{_f(_median(v), '+.2f')} ({_f(q1, '+.2f')}〜{_f(q3, '+.2f')})"


def format_recompute(
    label: str,
    base_setting: RecomputeSetting,
    diff: tuple[float, float, int, int],
    results: Sequence[RecomputeResult],
) -> str:
    lb = base_setting.brake_center_s
    lb_text = "なし(=L)" if lb is None else f"{lb:g}s"
    out = [
        f"## 指令の再計算 {label}（車の応答は含まない。基準は CSV の基準車速の線形補間）",
        "",
        f"ベースライン（d={base_setting.delay_s:g}s・H={base_setting.width_s:g}s・"
        f"L={base_setting.center_s:g}s・L_b="
        f"{lb_text}）"
        f"とログの FF（待機位置の前）の最大 |差|: "
        f"ブレーキ {diff[0]:.4f}%・アクセル {diff[1]:.4f}%"
        f"（0.1% を超えた行 {diff[2]} / {diff[3]}）",
        "",
        f"区間 = ログの phase で取った「{MID_LABEL}」のブレーキ区間。中央値（四分位 25〜75%）。"
        "指令→基準の減速は + が「指令のほうが先」（段1b と同じ測り方）。"
        "切替→基準の減速 50% は、再計算したペダルが最初にブレーキへ切り替わった時刻と基準の減速の"
        "踏み始めの差で、+ が「切替のほうが先」。"
        "ブレーキ離し→基準の減速 離し50% は、再計算したペダルが最初の切替の後に BRAKE から抜けた"
        "時刻と、基準の減速が山の 50% を下回る（離し）時刻の差で、同じ向き（+ が「選択のほうが先に"
        "離す」＝基準の側の時刻 − 選択の側の時刻）。アクセルの行の違い は、アクセルの判定"
        "（ACCEL か）またはアクセル FF が 1e-9% を超えてベースラインと違う行数（L_b の行は 0 になる"
        "はず。ならなければアクセルの判定が L_b に引きずられている）。",
        "",
    ]
    body = [
        [
            r.setting.label,
            str(r.n_segments),
            f"{_med_q(r.onset_s)} n={len(r.onset_s)}",
            f"{_med_q(r.xcorr_s)} n={len(r.xcorr_s)}",
            f"{_med_q(r.switch_s)} n={len(r.switch_s)}",
            f"{_med_q(r.release_s)} n={len(r.release_s)}",
            f"{r.reversal_overall:.3f}",
            f"{r.reversal_window_max:.3f}",
            str(r.switches),
            str(r.accel_diff_rows),
        ]
        for r in results
    ]
    out += _table(RECOMPUTE_HEADER, body)
    out += [
        "",
        "アクセル往復は再計算したアクセル指令（待機位置の下限つき）を再計算したペダルで評価"
        "（KPI と同じ kwargs）。アクセル⇔ブレーキ切替は COAST を飛ばして数える。",
    ]
    return "\n".join(out)


# ─────────────────────────────────────────────────────────────────────
# ブレーキが作った減速の「効き」の落ち方（--part effect）
#   ブレーキ区間の後半で、同じ実位置でも減速が出ない原因（H1 ヒステリシス・H2 時間・
#   H3 速度・H4 測り方）を、行ごとのラベル付きの表で見分ける。純関数（配列 → 配列）と
#   CSV の読み込み・表示を分ける。
# ─────────────────────────────────────────────────────────────────────

# 実位置 → 減速の遅れ [s]。半分データ由来: 指令→実車の減速（学習の相互相関 0.20s）−
# 指令→実位置（0.05〜0.10s）。表 4 で 0.1〜0.4s を振って確かめる。--effect-lag で変えられる。
EFFECT_LAG_S = 0.2
EFFECT_LAG_SWEEP_S: tuple[float, ...] = (0.1, 0.2, 0.3, 0.4)  # 表 4 で振る遅れ
# 仮値: 開度の超過がこれ以下の行は数えない（不感帯すぐ上は実位置の測り誤差が支配的）
MIN_EXCESS_PCT = 0.5
# 仮値: 踏み増し／戻しの境 [%/s]（スクラッチで使った値）。結果が境の値で大きく変わるなら見直す。
MOTION_EDGE_PCTS = 1.0
POST_RELEASE_S = 3.0  # 「離した後」の長さ [s]（段1 の 6 の離した後 3s に合わせた）
MIN_CELL_ROWS = 10  # 仮値: これ未満の行数のセルは「−」。表の見やすさだけで判定には使わない
# 以下の帯は仮値。走行の山の中央値（約 10.5% ＝ 不感帯 +2.7%）に合わせた表の粒度で、固定（表示用）。
EFFECT_SPEED_EDGES_KMH: tuple[float, ...] = (0.0, 20.0, 40.0, 60.0)
EFFECT_EXCESS_EDGES_PCT: tuple[float, ...] = (0.5, 1.5, 2.5, 3.5, 5.0)
EFFECT_ELAPSED_EDGES_S: tuple[float, ...] = (0.0, 1.0, 2.0, 3.0)
BACKOFF_ZERO_PCT = 0.05  # 山から戻した量がこれ以下なら「0（まだ戻していない）」
BACKOFF_EDGES_PCT: tuple[float, ...] = (1.0, 2.0)  # 0〜1／1〜2／2 以上の境
FOCUS_SPEED_BANDS: tuple[int, ...] = (1, 0)  # 表に出す速度帯の番号（20〜40, 0〜20）
FOCUS_EXCESS_RANGE: tuple[int, int] = (1, 3)  # 表 4・見出しの開度の超過 1.5〜3.5%（帯番号 1〜2）
PHASE_TRAIN_HOLD = "HOLD_STEP"

THIRD_NAMES = ("前 1/3", "中 1/3", "後 1/3", "離した後")
MOTION_NAMES = ("踏み増し", "保持", "戻し")
BACKOFF_NAMES = ("0", "0〜1%", "1〜2%", "2%以上")
ELAPSED_NAMES = ("0〜1s", "1〜2s", "2〜3s", "3s以上")


def _edge_names(edges: Sequence[float], unit: str = "") -> list[str]:
    names = [f"{lo:g}〜{hi:g}{unit}" for lo, hi in zip(edges[:-1], edges[1:], strict=True)]
    return [*names, f"{edges[-1]:g}{unit}以上"]


SPEED_NAMES = _edge_names(EFFECT_SPEED_EDGES_KMH)
EXCESS_NAMES = _edge_names(EFFECT_EXCESS_EDGES_PCT, "%")


@dataclass(frozen=True)
class EffectRows:
    """学習 CSV の PATTERN_DRIVE 行の時系列（取れない値は NaN）。"""

    t_s: np.ndarray
    speed_kmh: np.ndarray
    brake_actual_pct: np.ndarray
    phase: list[str]


def read_effect_train_rows(path: Path) -> EffectRows:
    """学習 CSV（手順2）から PATTERN_DRIVE 行を読む（時刻は `elapsed_s`）。"""
    t: list[float] = []
    v: list[float] = []
    pos: list[float] = []
    phase: list[str] = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("section") != SECTION_PATTERN_DRIVE or not r.get("elapsed_s"):
                continue
            p = actual_opening(r, "brake")
            t.append(float(r["elapsed_s"]))
            v.append(float(r["actual_speed_kmh"]) if r.get("actual_speed_kmh") else math.nan)
            pos.append(math.nan if p is None else p)
            phase.append(r.get("phase", "") or "")
    return EffectRows(
        np.array(t, dtype=float), np.array(v, dtype=float), np.array(pos, dtype=float), phase
    )


def free_accel_array(
    params: FeedforwardParams, research: ResearchFFParams, speed_kmh: np.ndarray
) -> np.ndarray:
    """各行の車速での惰行の加速度 [km/h/s]（惰行では負）。車速が NaN の行は NaN。"""
    return np.array(
        [
            free_accel_at(params, research, float(x)) if np.isfinite(x) else math.nan
            for x in speed_kmh
        ],
        dtype=float,
    )


def shift_forward(a: np.ndarray, k: int) -> np.ndarray:
    """out[i] = a[i + k]。末尾 k 個は NaN。"""
    out = np.full(len(a), np.nan)
    if k <= 0:
        return a.astype(float).copy()
    if k < len(a):
        out[:-k] = a[k:]
    return out


def brake_effect(
    t_s: np.ndarray, speed_kmh: np.ndarray, free_accel: np.ndarray, lag_s: float
) -> np.ndarray:
    """ブレーキが作った減速 [km/h/s]（正 = 減速）。

    effect[i] = decel(t_i + lag) + free_accel(v_i)。decel は減速が正、free_accel は惰行で負なので、
    和は「惰行の減速を引いた、ブレーキ分の減速」。遅れは行の刻みの中央値の整数倍に丸める。
    """
    dt = float(np.median(np.diff(t_s)))
    k = int(round(lag_s / dt))
    return shift_forward(decel_series(t_s, speed_kmh), k) + free_accel


def bin_index(x: np.ndarray, lower_edges: Sequence[float]) -> np.ndarray:
    """下側の境で帯番号を返す（最後の帯は上が開いている）。最初の境未満・NaN は −1。"""
    idx = np.searchsorted(np.asarray(lower_edges, dtype=float), x, side="right") - 1
    return np.where(np.isfinite(x), idx, -1)


def backoff_bin_index(backoff_pct: np.ndarray) -> np.ndarray:
    """山から戻した量の帯番号: 0（≤BACKOFF_ZERO_PCT）／0〜1／1〜2／2 以上。NaN は −1。"""
    idx = (
        (backoff_pct > BACKOFF_ZERO_PCT).astype(int)
        + (backoff_pct >= BACKOFF_EDGES_PCT[0])
        + (backoff_pct >= BACKOFF_EDGES_PCT[1])
    )
    return np.where(np.isfinite(backoff_pct), idx, -1)


def motion_index(t_s: np.ndarray, pos_pct: np.ndarray) -> np.ndarray:
    """ペダルの動き: 0 踏み増し（傾き > +境）／1 保持／2 戻し（傾き < −境）。NaN は −1。"""
    slope = np.gradient(pos_pct, t_s)
    idx = np.where(slope > MOTION_EDGE_PCTS, 0, np.where(slope < -MOTION_EDGE_PCTS, 2, 1))
    return np.where(np.isfinite(slope), idx, -1)


@dataclass(frozen=True)
class SegmentLabels:
    """区間に対する行ごとのラベル。区間外の行は third = −1、elapsed・backoff は NaN。"""

    third: np.ndarray  # 0 前／1 中／2 後 1/3、3 離した後
    elapsed_s: np.ndarray  # 区間の開始行からの時間 [s]
    backoff_pct: np.ndarray  # 区間内のそれまでの実位置の最大 − 今の実位置（山から戻した量）


def segment_labels(
    t_s: np.ndarray,
    pos_pct: np.ndarray,
    segments: Sequence[BrakeSegment],
    post_s: float = POST_RELEASE_S,
) -> SegmentLabels:
    """区間 [i0, i1] の 3 分割（i0 + j*n//3 〜 i0 + (j+1)*n//3）と離した後 post_s 秒のラベル。

    離した後の行の山から戻した量は、区間全体の最大 − 今の実位置。
    """
    n = len(t_s)
    third = np.full(n, -1, dtype=int)
    elapsed = np.full(n, np.nan)
    backoff = np.full(n, np.nan)
    for seg in segments:
        i0, i1 = seg.i0, seg.i1
        length = i1 - i0 + 1
        for j in range(3):
            third[i0 + j * length // 3 : i0 + (j + 1) * length // 3] = j
        run_max = np.fmax.accumulate(pos_pct[i0 : i1 + 1])
        backoff[i0 : i1 + 1] = run_max - pos_pct[i0 : i1 + 1]
        elapsed[i0 : i1 + 1] = t_s[i0 : i1 + 1] - t_s[i0]
        peak = run_max[-1]
        k = i1 + 1
        while k < n and t_s[k] - t_s[i1] <= post_s and third[k] == -1:
            third[k] = 3
            elapsed[k] = t_s[k] - t_s[i0]
            backoff[k] = peak - pos_pct[k]
            k += 1
    return SegmentLabels(third, elapsed, backoff)


@dataclass(frozen=True)
class EffectTable:
    """1 本のログの行ごとの値とラベル（表の集計元）。"""

    dt_s: float
    speed_kmh: np.ndarray
    excess_pct: np.ndarray
    effect_kmhs: np.ndarray
    motion: np.ndarray
    third: np.ndarray
    elapsed_s: np.ndarray
    backoff_pct: np.ndarray
    is_train_hold: np.ndarray  # 学習: HOLD_STEP の行
    to_stop: np.ndarray  # 停車までの区間の行（参考表用）


def build_effect_table(
    t_s: np.ndarray,
    speed_kmh: np.ndarray,
    pos_pct: np.ndarray,
    free_accel: np.ndarray,
    deadband_pct: float,
    lag_s: float,
    segments: Sequence[BrakeSegment] = (),
    phase: Sequence[str] = (),
) -> EffectTable:
    """行ごとの効き・開度の超過・ラベルをまとめる。segments が空なら区間ラベルは付かない。"""
    n = len(t_s)
    lab_all = segment_labels(t_s, pos_pct, segments)
    dt = float(np.median(np.diff(t_s)))
    stop = np.zeros(n, dtype=bool)
    for seg in segments:
        if seg.to_stop:
            stop[seg.i0 : seg.i1 + 1 + int(POST_RELEASE_S / dt)] = True
    return EffectTable(
        dt_s=dt,
        speed_kmh=speed_kmh,
        excess_pct=pos_pct - deadband_pct,
        effect_kmhs=brake_effect(t_s, speed_kmh, free_accel, lag_s),
        motion=motion_index(t_s, pos_pct),
        third=lab_all.third,
        elapsed_s=lab_all.elapsed_s,
        backoff_pct=lab_all.backoff_pct,
        is_train_hold=np.array([p == PHASE_TRAIN_HOLD for p in phase], dtype=bool)
        if len(phase) == n
        else np.zeros(n, dtype=bool),
        to_stop=stop,
    )


def effect_cell(effect: np.ndarray, mask: np.ndarray, dt_s: float) -> str:
    """セルの文字列「中央値(秒数)」。有限な値が MIN_CELL_ROWS 行未満なら「−」。"""
    m = mask & np.isfinite(effect)
    n = int(m.sum())
    if n < MIN_CELL_ROWS:
        return "−"
    return f"{np.median(effect[m]):.2f}({n * dt_s:.0f}s)"


def effect_median(effect: np.ndarray, mask: np.ndarray) -> float:
    """マスクした行の効きの中央値。MIN_CELL_ROWS 行未満は NaN。"""
    m = mask & np.isfinite(effect)
    return float(np.median(effect[m])) if m.sum() >= MIN_CELL_ROWS else math.nan


def base_mask(tb: EffectTable, speed_bin: int | None = None) -> np.ndarray:
    """開度の超過が MIN_EXCESS_PCT より大きい行（速度帯を指定すればその帯だけ）。"""
    m = tb.excess_pct > MIN_EXCESS_PCT
    if speed_bin is not None:
        m = m & (bin_index(tb.speed_kmh, EFFECT_SPEED_EDGES_KMH) == speed_bin)
    return m


def _excess_row(tb: EffectTable, mask: np.ndarray) -> list[str]:
    eb = bin_index(tb.excess_pct, EFFECT_EXCESS_EDGES_PCT)
    return [
        effect_cell(tb.effect_kmhs, mask & (eb == b), tb.dt_s) for b in range(len(EXCESS_NAMES))
    ]


@dataclass(frozen=True)
class EffectResult:
    labels: list[str]
    train_label: str
    train: EffectTable
    train_phase_seconds: dict[str, float]
    drives: list[EffectTable]  # 走行ログごと（lag は --effect-lag）
    sweeps: dict[float, list[EffectTable]]  # 遅れ → 走行ログごと（表 4）
    lag_s: float
    deadband_pct: float


def compute_effect(
    labels: Sequence[str],
    drive_rows: Sequence[BrakeRows],
    train_label: str,
    train_rows: EffectRows,
    bundle: ModelBundle,
    lag_s: float,
) -> EffectResult:
    db = bundle.deadbands[PEDAL_BRAKE]
    train_free = free_accel_array(bundle.params, bundle.research, train_rows.speed_kmh)
    train = build_effect_table(
        train_rows.t_s, train_rows.speed_kmh, train_rows.brake_actual_pct, train_free,
        db, lag_s, phase=train_rows.phase,
    )
    seconds: dict[str, float] = {}
    over = train.excess_pct > MIN_EXCESS_PCT
    for ph, o in zip(train_rows.phase, over, strict=True):
        if o:
            seconds[ph] = seconds.get(ph, 0.0) + train.dt_s
    segs = [find_brake_segments(r.phase, r.ref_kmh, r.t_s) for r in drive_rows]
    frees = [free_accel_array(bundle.params, bundle.research, r.actual_kmh) for r in drive_rows]

    def tables(lag: float) -> list[EffectTable]:
        return [
            build_effect_table(r.t_s, r.actual_kmh, r.brake_actual_pct, f, db, lag, s)
            for r, f, s in zip(drive_rows, frees, segs, strict=True)
        ]

    lags = sorted({*EFFECT_LAG_SWEEP_S})
    sweeps = {lag: tables(lag) for lag in lags}
    return EffectResult(
        list(labels), train_label, train, seconds, tables(lag_s), sweeps, lag_s, db
    )


def _drive_mask(tb: EffectTable, to_stop: bool) -> np.ndarray:
    """to_stop=False は「途中の速度まで」の区間（と離した後）、True は停車までの区間の行。"""
    return tb.to_stop if to_stop else ~tb.to_stop


def format_effect(res: EffectResult) -> str:
    L = res.labels
    out: list[str] = [
        f"設定: 実位置→減速の遅れ {res.lag_s:g}s・ブレーキ不感帯 {res.deadband_pct:.2f}%・"
        f"開度の超過 = 実位置 − 不感帯が {MIN_EXCESS_PCT:g}% を超えた行だけ。",
        "",
        "ブレーキが作った減速 = 実車の減速（実位置から遅れ後）+ その速度の惰行の加速度"
        "（正 = 減速）。セルは「中央値 [km/h/s]（秒数）」で、"
        f"{MIN_CELL_ROWS} 行未満は「−」。区間は「途中の速度まで」のブレーキ区間"
        "（前・中・後 1/3 と、離した後 "
        f"{POST_RELEASE_S:g}s）。",
        "",
        "## 学習データの行（開度の超過 > "
        f"{MIN_EXCESS_PCT:g}% の phase 別の秒数。{res.train_label}）",
        "",
    ]
    secs = sorted(res.train_phase_seconds.items(), key=lambda kv: -kv[1])
    out += _table(["phase", "秒数"], [[p or "（空）", f"{s:.0f}"] for p, s in secs])
    out.append("")

    def drive_rows_of(tb_i: int, to_stop: bool, mask: np.ndarray) -> np.ndarray:
        return mask & _drive_mask(res.drives[tb_i], to_stop)

    for sb in FOCUS_SPEED_BANDS:
        for to_stop in (False, True):
            kind = "停車までの区間（参考）" if to_stop else "途中の速度まで"
            out += [
                f"## 1. 速度 {SPEED_NAMES[sb]} km/h × 開度の超過（{kind}）",
                "",
            ]
            body: list[list[str]] = []
            if not to_stop:
                tr = res.train
                tm = base_mask(tr, sb) & tr.is_train_hold
                body.append([f"学習 {PHASE_TRAIN_HOLD}", *_excess_row(tr, tm)])
            for i, tb in enumerate(res.drives):
                bm = base_mask(tb, sb)
                for j, nm in enumerate(THIRD_NAMES):
                    if to_stop and j == 3:
                        continue
                    m = drive_rows_of(i, to_stop, bm & (tb.third == j))
                    body.append([f"{L[i]} {nm}", *_excess_row(tb, m)])
            out += _table(["行", *EXCESS_NAMES], body)
            out.append("")

    for sb in FOCUS_SPEED_BANDS:
        out += [
            f"## 1b. 速度 {SPEED_NAMES[sb]} km/h × 開度の超過 × ペダルの動き"
            f"（途中の速度まで・区間の中。境 ±{MOTION_EDGE_PCTS:g}%/s）",
            "",
        ]
        body = []
        tr = res.train
        for k, nm in enumerate(MOTION_NAMES):
            body.append(
                [f"学習 {PHASE_TRAIN_HOLD} {nm}",
                 *_excess_row(tr, base_mask(tr, sb) & tr.is_train_hold & (tr.motion == k))]
            )
        for i, tb in enumerate(res.drives):
            inseg = (tb.third >= 0) & (tb.third <= 2)
            for k, nm in enumerate(MOTION_NAMES):
                m = drive_rows_of(i, False, base_mask(tb, sb) & inseg & (tb.motion == k))
                body.append([f"{L[i]} {nm}", *_excess_row(tb, m)])
        out += _table(["行", *EXCESS_NAMES], body)
        out.append("")

    for sb in FOCUS_SPEED_BANDS:
        out += [
            f"## 2. H1 速度 {SPEED_NAMES[sb]} km/h: 開度の超過 × 山から戻した量"
            "（途中の速度まで・区間の中）",
            "",
        ]
        body = []
        for i, tb in enumerate(res.drives):
            bb = backoff_bin_index(tb.backoff_pct)
            inseg = (tb.third >= 0) & (tb.third <= 2)
            for b, bn in enumerate(BACKOFF_NAMES):
                m = drive_rows_of(i, False, base_mask(tb, sb) & inseg & (bb == b))
                body.append([f"{L[i]} 戻した量 {bn}", *_excess_row(tb, m)])
        out += _table(["行", *EXCESS_NAMES], body)
        out.append("")

    for sb in FOCUS_SPEED_BANDS:
        out += [
            f"## 3. H2 速度 {SPEED_NAMES[sb]} km/h: まだ戻していない行（戻した量 0）の"
            "開度の超過 × 区間に入ってからの時間",
            "",
        ]
        body = []
        for i, tb in enumerate(res.drives):
            eb = bin_index(tb.elapsed_s, EFFECT_ELAPSED_EDGES_S)
            bb = backoff_bin_index(tb.backoff_pct)
            inseg = (tb.third >= 0) & (tb.third <= 2)
            for e, en in enumerate(ELAPSED_NAMES):
                m = drive_rows_of(i, False, base_mask(tb, sb) & inseg & (bb == 0) & (eb == e))
                body.append([f"{L[i]} 経過 {en}", *_excess_row(tb, m)])
        out += _table(["行", *EXCESS_NAMES], body)
        out.append("")

    lo, hi = FOCUS_EXCESS_RANGE
    out += [
        f"## 4. H4 遅れを振ったときの 前 1/3 と 後 1/3 の比（速度 {SPEED_NAMES[1]} km/h・"
        f"開度の超過 {EFFECT_EXCESS_EDGES_PCT[lo]:g}〜{EFFECT_EXCESS_EDGES_PCT[hi]:g}%・"
        "途中の速度まで）",
        "",
    ]
    head = ["遅れ[s]"]
    for lb in L:
        head += [f"{lb} 前", f"{lb} 後", f"{lb} 後/前"]
    body = []
    ratios: dict[float, list[float]] = {}
    for lag, tbs in sorted(res.sweeps.items()):
        row = [f"{lag:g}"]
        ratios[lag] = []
        for tb in tbs:
            front, back, ratio = lag_ratio(tb)
            row += [_f(front, ".2f"), _f(back, ".2f"), _f(ratio, ".2f")]
            ratios[lag].append(ratio)
        body.append(row)
    out += _table(head, body)
    out.append("")
    out += format_effect_headline(res, ratios)
    return "\n".join(out)


def focus_mask(tb: EffectTable) -> np.ndarray:
    """20〜40 km/h・開度の超過 1.5〜3.5%・途中の速度まで・区間の中（見出し・表 4 の対象）。"""
    eb = bin_index(tb.excess_pct, EFFECT_EXCESS_EDGES_PCT)
    in_range = (eb >= FOCUS_EXCESS_RANGE[0]) & (eb <= FOCUS_EXCESS_RANGE[1])
    return base_mask(tb, 1) & in_range & ~tb.to_stop


def lag_ratio(tb: EffectTable) -> tuple[float, float, float]:
    """(前 1/3 の中央値, 後 1/3 の中央値, 後/前)。足りなければ NaN。"""
    fm = focus_mask(tb)
    front = effect_median(tb.effect_kmhs, fm & (tb.third == 0))
    back = effect_median(tb.effect_kmhs, fm & (tb.third == 2))
    ratio = back / front if np.isfinite(front) and front > 0 and np.isfinite(back) else math.nan
    return front, back, ratio


def _monotone_down(vals: Sequence[float]) -> bool | None:
    """有限な値が 2 つ以上あり、並びの順に減り続ければ True。増える箇所があれば False。"""
    v = [x for x in vals if np.isfinite(x)]
    if len(v) < 2:
        return None
    return all(b < a for a, b in zip(v[:-1], v[1:], strict=True))


def format_effect_headline(res: EffectResult, ratios: dict[float, list[float]]) -> list[str]:
    """見出し: どの候補の予測の形に数字が合うかを事実として並べる（判定はユーザーが決める）。"""
    out = [
        "## 見出し（20〜40 km/h・開度の超過 1.5〜3.5%・途中の速度まで。数字の並びの事実のみ）",
        "",
    ]
    for i, lb in enumerate(res.labels):
        tb = res.drives[i]
        fm = focus_mask(tb)
        inseg = (tb.third >= 0) & (tb.third <= 2)
        bb = backoff_bin_index(tb.backoff_pct)
        h1 = [effect_median(tb.effect_kmhs, fm & inseg & (bb == b)) for b in range(4)]
        eb = bin_index(tb.elapsed_s, EFFECT_ELAPSED_EDGES_S)
        h2 = [
            effect_median(tb.effect_kmhs, fm & inseg & (bb == 0) & (eb == e)) for e in range(4)
        ]
        d1, d2 = _monotone_down(h1), _monotone_down(h2)
        out.append(
            f"- {lb} H1（戻した量 {'／'.join(BACKOFF_NAMES)} の順）: "
            f"{' → '.join(_f(x, '.2f') for x in h1)} km/h/s。"
            + _trend_text(d1, "戻した量が増えるほど下がる（H1 の予測の形）", "下がり続けてはいない")
        )
        out.append(
            f"- {lb} H2（戻していない行の経過 {'／'.join(ELAPSED_NAMES)} の順）: "
            f"{' → '.join(_f(x, '.2f') for x in h2)} km/h/s。"
            + _trend_text(d2, "経過とともに下がる（H2 の予測の形）", "下がり続けてはいない")
        )
        rs = " ・ ".join(f"{lag:g}s={_f(r[i], '.2f')}" for lag, r in sorted(ratios.items()))
        vals = [r[i] for r in ratios.values() if np.isfinite(r[i])]
        near1 = [lag for lag, r in sorted(ratios.items()) if np.isfinite(r[i]) and r[i] >= 0.9]
        out.append(
            f"- {lb} H4（後/前の比、遅れ別）: {rs}。"
            + _h4_text(near1, vals)
        )
    out += [
        "",
        "H3（速度の違い）は表 1 の速度帯ごとの数字で見る（同じ速度帯の中でも前 → 後で下がるか）。",
        "ここは数字の並びだけで、原因の判定はユーザーとの議論で決める。",
    ]
    return out


def _h4_text(near1: Sequence[float], vals: Sequence[float]) -> str:
    if near1:
        return f"遅れ {', '.join(f'{x:g}' for x in near1)}s で比が 0.9 以上（H4 の予測の形）"
    if vals:
        return f"どの遅れでも比は 0.9 未満（最大 {max(vals):.2f}）"
    return "比が出せない"


def _trend_text(down: bool | None, yes: str, no: str) -> str:
    if down is None:
        return "比べられる帯が 2 つ未満"
    return yes if down else no


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="FF の作ったずれの集計")
    p.add_argument(
        "--part", choices=["created", "cause", "timing", "recompute", "effect"], required=True
    )
    p.add_argument("csv", nargs="+", type=Path, help="モード走行 CSV（複数可）")
    p.add_argument(
        "--train-csv", type=Path, help="cause / timing / effect: 手順2（格子ステップ走行）の CSV"
    )
    p.add_argument("--model", help="cause: pkl のパス（省略時は設定の feedforward.model_path）")
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="cause: 設定 YAML")
    p.add_argument(
        "--effect-lag", type=float, default=EFFECT_LAG_S,
        help=f"effect: 実位置から減速までの遅れ [s]（既定 {EFFECT_LAG_S:g}）",
    )
    args = p.parse_args(argv)
    if args.part == "effect":
        if args.train_csv is None:
            p.error("--part effect には --train-csv が要ります")
        bundle = load_bundle(args.config, args.model)
        print(format_effect(compute_effect(
            [_label_of(c) for c in args.csv],
            [read_brake_rows(c) for c in args.csv],
            _label_of(args.train_csv),
            read_effect_train_rows(args.train_csv),
            bundle,
            args.effect_lag,
        )))
        return 0
    if args.part == "cause":
        if args.train_csv is None:
            p.error("--part cause には --train-csv が要ります")
        bundle = load_bundle(args.config, args.model)
        res = compute_cause(
            [_label_of(c) for c in args.csv],
            [rows_from_csv(c) for c in args.csv],
            read_drive_logs(args.train_csv),
            bundle,
        )
        print(format_cause(res))
        print(format_interpretation(res))
        return 0
    if args.part == "recompute":
        bundle = load_bundle(args.config, args.model)
        cfg = load_config(args.config)
        drive = recompute_drive_from_config(cfg)
        base = RecomputeSetting(
            "ベースライン", 0.0,
            cfg.feedforward.pedal_select_center_s, cfg.feedforward.pedal_select_width_s,
            cfg.feedforward.pedal_select_brake_center_s,
        )
        assert bundle.ff is not None  # noqa: S101
        for c in args.csv:
            rows, log = read_brake_rows(c), read_log_ff(c)
            results, diff = [], (math.nan, math.nan, 0, 0)
            baseline_rec: Recomputed | None = None
            for st in recompute_settings(base):
                rec = recompute_commands(bundle.ff, rows.t_s, rows.ref_kmh, drive, st)
                if baseline_rec is None:  # 先頭がベースライン（以降の行はこれと比べる）
                    baseline_rec = rec
                    diff = baseline_diff(rec, log)
                results.append(evaluate_recompute(rows, rec, st, cfg.kpi, baseline_rec))
            print(format_recompute(_label_of(c), base, diff, results))
        return 0
    if args.part == "timing":
        train = lag = None
        if args.train_csv is not None:
            train = train_brake_timings(args.train_csv)
            lag = estimate_pedal_lag(read_step_rows(args.train_csv)).by_kind.get(KIND_BRAKE)
        print(format_timing(
            [_label_of(c) for c in args.csv],
            [segment_timings(read_brake_rows(c)) for c in args.csv],
            train,
            _label_of(args.train_csv) if args.train_csv else "",
            lag,
        ))
        return 0
    labels = [_label_of(c) for c in args.csv]
    reports = [compute_created(rows_from_csv(c)) for c in args.csv]
    print(format_created(labels, reports))
    return 0


if __name__ == "__main__":
    sys.exit(main())
