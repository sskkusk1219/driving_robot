"""WLTP の「車速 × 加速度」格子と、手順2 の学習データの網羅マップ（ProblemReport_20260925 段1）。

FF 逆モデルの入力（車速・この先の車速変化）は WLTP が決めるので車両に依らない。開度で網羅を
測ると車両が変わるたびに意味が変わるため、網羅性は「車速 × 加速度」の格子で測る。

加速度は FF の regime ホライズン先（`FeatureSpec.regime_horizon_s`）との車速差 / ホライズン
[km/h/s]。学習側（`ff_model.build_feature_matrix` の dv 列）と同じ定義にそろえてある。

    WLTP 側  : 基準車速（ReferenceSpeed）を 0.1s で刻み、セルごとに滞在時間 [s] を数える
    データ側  : 手順2 の CSV の行を、そのペダルが効いているか（`ff_candidate` の A1 と同じ判定）で
               アクセル／ブレーキ／惰行（どちらも不感帯未満）に分けて、セルごとの秒数を数える

停車（v0 < STOP_SPEED_KMH）は両側から除外する（クリープ・停車保持の領分で、逆モデルの対象外）。

使い方:
    .venv/bin/python -m tests.research.wltp_grid <手順2のCSV>
    .venv/bin/python -m tests.research.wltp_grid <手順2のCSV> --ref-csv <基準車速を持つCSV>
        # --ref-csv: DB を使わず、mode_time_s（または time_s）と ref_speed_kmh を持つ CSV から読む
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import sys
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, ConfigError, ResearchConfig, load_config
from tests.research.drive_log import read_drive_logs
from tests.research.ff_model import (
    DEFAULT_FEATURE_SPEC,
    STOP_SPEED_KMH,
    FeatureSpec,
    build_feature_matrix,
    estimate_offsets,
    group_by_session,
)
from tests.research.ff_params import free_accel_at, research_ff_params
from tests.research.mode_drive import ReferenceSpeed, load_mode
from tests.research.research_types import DriveLog, DrivingMode, SpeedPoint
from tests.research.vehicle import build_vehicle_profile, feedforward_params

WLTP_DT_S: float = 0.1  # WLTP を刻む周期（走行ログと同じ 100ms）
_ORIGIN = datetime(2000, 1, 1, tzinfo=UTC)


@dataclass(frozen=True)
class GridEdges:
    """格子の境界。行 = 車速のビン、列 = 加速度のビン（境界は昇順。範囲外の行は数えない）。"""

    speed_kmh: tuple[float, ...]
    accel_kmhs: tuple[float, ...]

    @property
    def shape(self) -> tuple[int, int]:
        return len(self.speed_kmh) - 1, len(self.accel_kmhs) - 1


@dataclass(frozen=True)
class DataCells:
    """学習データのセルごとの秒数。`accel`/`brake` はそのペダルが効いている行、`coast` はどちらも
    不感帯未満の行（惰行カーブが担当）。"""

    accel: np.ndarray
    brake: np.ndarray
    coast: np.ndarray
    # 戻し側（2c-1）。そのペダルが効いている連続区間の中で、直近の折り返し点から
    # `grid_direction_threshold_pct` 以上戻った行。`accel`/`brake` は踏み込み側の行だけ。
    # 省略時は 0（向きを分けない従来の作り方との互換）
    accel_release: np.ndarray | None = None
    brake_release: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.accel_release is None:
            object.__setattr__(self, "accel_release", np.zeros_like(self.accel))
        if self.brake_release is None:
            object.__setattr__(self, "brake_release", np.zeros_like(self.brake))

    @property
    def total(self) -> np.ndarray:
        """全行（踏み込み・戻し・惰行）の合計。向きを分けない従来の意味のまま。"""
        return self.press_total + self.release_total

    @property
    def press_total(self) -> np.ndarray:
        """踏み込み側の秒数。惰行（どちらのペダルも不感帯未満）は踏み込み側に数える。"""
        return self.accel + self.brake + self.coast

    @property
    def release_total(self) -> np.ndarray:
        """戻し側の秒数（アクセル戻し + ブレーキ戻し）。"""
        assert self.accel_release is not None and self.brake_release is not None
        return self.accel_release + self.brake_release


def edges_from_config(speed_kmh: list[float], accel_kmhs: list[float]) -> GridEdges:
    return GridEdges(tuple(speed_kmh), tuple(accel_kmhs))


def _histogram(v0: np.ndarray, a_req: np.ndarray, edges: GridEdges, dt_s: float) -> np.ndarray:
    """(車速, 加速度) の 2 次元ヒストグラム × dt。停車は除く。"""
    moving = v0 >= STOP_SPEED_KMH
    hist, _, _ = np.histogram2d(
        v0[moving], a_req[moving], bins=[edges.speed_kmh, edges.accel_kmhs]
    )
    return hist * dt_s


def _v0_and_a_req(
    speed: np.ndarray, timestamps: list[datetime], spec: FeatureSpec
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """学習側と同じ特徴量行列から (v0, a_req, 有効行の index) を取り出す。"""
    x, idx = build_feature_matrix(
        speed,
        estimate_offsets(timestamps, spec.lookahead_horizons_s),
        estimate_offsets(timestamps, spec.past_horizons_s),
        spec,
    )
    if len(idx) == 0:
        return np.empty(0), np.empty(0), idx
    return x[:, 0], x[:, spec.regime_col()] / spec.regime_horizon_s, idx


def _wltp_v0_and_a_req(mode: DrivingMode, spec: FeatureSpec) -> tuple[np.ndarray, np.ndarray]:
    """WLTP（基準車速）を 0.1s で刻み、学習側と同じ定義の (v0, a_req) を返す。"""
    points = mode.reference_speed
    t = np.arange(points[0].time_s, points[-1].time_s, WLTP_DT_S)
    ref = ReferenceSpeed(mode)
    speed = np.clip(np.array([ref.at(float(s)) for s in t]), 0.0, None)
    timestamps = [_ORIGIN + timedelta(seconds=float(s)) for s in t]
    v0, a_req, _ = _v0_and_a_req(speed, timestamps, spec)
    return v0, a_req


def wltp_cells(
    mode: DrivingMode, edges: GridEdges, spec: FeatureSpec = DEFAULT_FEATURE_SPEC
) -> np.ndarray:
    """WLTP（基準車速）が各セルに滞在する秒数。形は `edges.shape`。"""
    v0, a_req = _wltp_v0_and_a_req(mode, spec)
    return _histogram(v0, a_req, edges, WLTP_DT_S)


# ペダル選択の規則: (車速 → 惰行加速度 free(v) [km/h/s], 惰行帯の半幅 [km/h/s])
PedalRule = tuple[Callable[[float], float], float]
PEDAL_ACCEL = "A"
PEDAL_BRAKE = "B"
PEDAL_COAST = "-"

DIRECTION_THRESHOLD_KMHS: float = 0.3  # yaml の learning.grid_direction_threshold_kmhs の既定
DIRECTION_THRESHOLD_PCT: float = 0.3  # 同 grid_direction_threshold_pct（ブレーキ）
DIRECTION_THRESHOLD_ACCEL_PCT: float = 1.5  # 同 grid_direction_threshold_accel_pct（アクセル）
WEIGHT_FRAC: float = 0.25  # 同 grid_weight_frac
_A_REQ_EPS_KMHS: float = 1e-6  # |a_req| がこれ未満は「ゼロ」として episode を区切る


def _cruise_mask(a_req: np.ndarray, edges: GridEdges) -> np.ndarray:
    """加速度ビンが 0 をまたぐ列（定速の列）の行。ここは向きを分けず、全部踏み込み側に数える
    （定速保持の PI のゆらぎで、意味のある向きが無いため）。"""
    mask = np.zeros(len(a_req), dtype=bool)
    for lo, hi in zip(edges.accel_kmhs[:-1], edges.accel_kmhs[1:], strict=True):
        if lo < 0.0 < hi:
            mask |= (a_req >= lo) & (a_req < hi)
    return mask


def _reversal_release(values: np.ndarray, active: np.ndarray, threshold: float) -> np.ndarray:
    """行ごとの「戻し」（True）。`active` が続く区間を 1 つの episode とし、直近の折り返し点から
    数える: 踏み込み中は走査中の最大から `threshold` 以上下がったら戻しに切り替え、戻し中は
    走査中の最小から `threshold` 以上上がったら踏み込みに切り替える。episode の最初の行と
    `active` でない行は False。"""
    out = np.zeros(len(values), dtype=bool)
    inside = False
    released = False
    extreme = 0.0
    for k in range(len(values)):
        if not active[k]:
            inside = False
            continue
        cur = float(values[k])
        if not inside:
            inside, released, extreme = True, False, cur
            continue
        if not released:
            extreme = max(extreme, cur)
            if extreme - cur >= threshold:
                released, extreme = True, cur
        else:
            extreme = min(extreme, cur)
            if cur - extreme >= threshold:
                released, extreme = False, cur
        out[k] = released
    return out


def row_pedals(v0: np.ndarray, a_req: np.ndarray, rule: PedalRule) -> np.ndarray:
    """行ごとのペダル（'A'/'B'/'-'）。FF のペダル選択と同じ規則を、行自身の (v0, a_req) に掛ける:
    a ≥ free(v0) + 帯 → アクセル、a ≤ free(v0) − 帯 → ブレーキ、それ以外は惰行。"""
    free_accel, band = rule
    free = np.array([free_accel(float(v)) for v in v0], dtype=float)
    out = np.full(len(v0), PEDAL_COAST, dtype="<U1")
    out[a_req >= free + band] = PEDAL_ACCEL
    out[a_req <= free - band] = PEDAL_BRAKE
    return out


def mode_direction(
    a_req: np.ndarray,
    threshold_kmhs: float,
    v0: np.ndarray | None = None,
    pedal_rule: PedalRule | None = None,
) -> np.ndarray:
    """モード側の向きのラベル。True = 戻し、False = 踏み込み。

    `v0` と `pedal_rule` があるとき（ペダルの向きで数える）: 行ごとのペダル（`row_pedals`）を
    求め、「踏む量」= アクセルの行は a_req − free(v0)、ブレーキの行は free(v0) − a_req
    として、同じペダルが続く区間を 1 つの episode に `_reversal_release` で判定する
    （踏む量が直近の最大から `threshold_kmhs` 以上下がったら戻し）。惰行の行は False。
    アクセルを絞って作る弱い減速（a_req<0 でもペダルはアクセル）は、a_req の絶対値が増えても
    踏む量は減るので「戻し」になり、ペダルの動きと一致する。

    無いとき（従来の符号ベース）: 同じ符号の a_req が続く区間を episode とし、加速は a_req、
    減速は −a_req を踏む量として同じ判定をする。
    """
    if v0 is not None and pedal_rule is not None:
        free_accel, _ = pedal_rule
        free = np.array([free_accel(float(v)) for v in v0], dtype=float)
        pedal = row_pedals(v0, a_req, pedal_rule)
        press = np.where(pedal == PEDAL_ACCEL, a_req - free, free - a_req)
        return _reversal_release(
            press, pedal == PEDAL_ACCEL, threshold_kmhs
        ) | _reversal_release(press, pedal == PEDAL_BRAKE, threshold_kmhs)
    return _reversal_release(
        a_req, a_req > _A_REQ_EPS_KMHS, threshold_kmhs
    ) | _reversal_release(-a_req, a_req < -_A_REQ_EPS_KMHS, threshold_kmhs)


@dataclass(frozen=True)
class WltpCellStats:
    """WLTP の「車速 × 加速度」の集計（網羅表の「要る秒数」の元）。

    seconds / mean_accel: 形は `edges.shape`。mean_accel はセルの中の WLTP の平均加速度
    （滞在 0 のセルは nan）。max/min_accel_by_speed: 車速ビンごとの WLTP の加速度の最大／最小
    （そのビンに滞在が無ければ nan）。

    seconds_press / seconds_release（2c-1）: `seconds` を `mode_direction` で踏み込み／戻しに
    分けたもの（形は同じ。和は `seconds`）。省略時は全部踏み込み側。

    seconds_release_accel / seconds_release_brake: 戻しの行を、その行自身の (a_req, v0) で
    FF のペダル選択と同じ規則（`PedalRule`）にかけてペダルごとに数えたもの。惰行帯の戻しの行は
    どちらにも数えない（なので和は `seconds_release` 以下）。規則を渡さなければ None。
    """

    seconds: np.ndarray
    mean_accel: np.ndarray
    max_accel_by_speed: np.ndarray
    min_accel_by_speed: np.ndarray
    seconds_press: np.ndarray | None = None
    seconds_release: np.ndarray | None = None
    seconds_release_accel: np.ndarray | None = None
    seconds_release_brake: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.seconds_press is None and self.seconds_release is None:
            object.__setattr__(self, "seconds_press", self.seconds)
            object.__setattr__(self, "seconds_release", np.zeros_like(self.seconds))
        elif self.seconds_press is None or self.seconds_release is None:
            raise ValueError("seconds_press と seconds_release は両方渡してください")


def wltp_cell_stats(
    mode: DrivingMode,
    edges: GridEdges,
    spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    direction_threshold_kmhs: float = DIRECTION_THRESHOLD_KMHS,
    pedal_rule: PedalRule | None = None,
) -> WltpCellStats:
    """WLTP のセルごとの秒数・平均加速度と、車速ビンごとの最大／最小加速度。停車は除く。

    向き（踏み込み／戻し）は停車を除く前の時系列全体（全行の v0・a_req とペダル規則）で
    判定してから、停車の行を除く。
    加速度ビンが 0 をまたぐ列（定速）は向きを分けず、全部踏み込み側に数える。
    `pedal_rule` があれば、戻しの行をペダルごとにも数える（`seconds_release_accel/brake`）。
    """
    v0, a_req = _wltp_v0_and_a_req(mode, spec)
    release = mode_direction(a_req, direction_threshold_kmhs, v0, pedal_rule)
    release &= ~_cruise_mask(a_req, edges)
    moving = v0 >= STOP_SPEED_KMH
    v0, a_req, release = v0[moving], a_req[moving], release[moving]
    n_speed, n_accel = edges.shape
    seconds = _histogram(v0, a_req, edges, WLTP_DT_S)
    seconds_release = _histogram(v0[release], a_req[release], edges, WLTP_DT_S)
    seconds_press = seconds - seconds_release
    rel_accel: np.ndarray | None = None
    rel_brake: np.ndarray | None = None
    if pedal_rule is not None:
        pedal = row_pedals(v0, a_req, pedal_rule)
        rel_accel = _histogram(
            v0[release & (pedal == PEDAL_ACCEL)], a_req[release & (pedal == PEDAL_ACCEL)],
            edges, WLTP_DT_S)
        rel_brake = _histogram(
            v0[release & (pedal == PEDAL_BRAKE)], a_req[release & (pedal == PEDAL_BRAKE)],
            edges, WLTP_DT_S)
    a_sum, _, _ = np.histogram2d(
        v0, a_req, bins=[edges.speed_kmh, edges.accel_kmhs], weights=a_req
    )
    with np.errstate(invalid="ignore", divide="ignore"):
        mean_accel = np.where(seconds > 0.0, a_sum * WLTP_DT_S / seconds, np.nan)
    speed_bin = np.digitize(v0, edges.speed_kmh) - 1
    a_max = np.full(n_speed, np.nan)
    a_min = np.full(n_speed, np.nan)
    for i in range(n_speed):
        in_bin = speed_bin == i
        if in_bin.any():
            a_max[i] = float(a_req[in_bin].max())
            a_min[i] = float(a_req[in_bin].min())
    return WltpCellStats(
        seconds, mean_accel, a_max, a_min, seconds_press, seconds_release, rel_accel, rel_brake
    )


def outside_seconds(
    mode: DrivingMode, edges: GridEdges, spec: FeatureSpec = DEFAULT_FEATURE_SPEC
) -> float:
    """格子の外（加速度・車速の端の外）に出て、数えられていない走行の秒数 [s]。停車は除く。"""
    v0, a_req = _wltp_v0_and_a_req(mode, spec)
    moving = v0 >= STOP_SPEED_KMH
    v0, a_req = v0[moving], a_req[moving]
    inside = (
        (v0 >= edges.speed_kmh[0]) & (v0 < edges.speed_kmh[-1])
        & (a_req >= edges.accel_kmhs[0]) & (a_req < edges.accel_kmhs[-1])
    )
    return float((~inside).sum() * WLTP_DT_S)


def combined_cell_stats(stats: list[WltpCellStats]) -> WltpCellStats:
    """複数モードの集計を 1 つにまとめる（段4b。狙いのマスは「どれかのモードで要るマス」の和集合）。

    - seconds・seconds_press・seconds_release: モードごとの **最大**
      （合計にしない。2 つのモードで 1.5s ずつ使うマスが合計 3s で
      `grid_target_min_s` を超えてしまうのを避け、「どれかのモードで必要秒数以上使う」を保つ）
    - mean_accel: 秒数で重み付けした平均
    - 車速ビンごとの最大／最小加速度: 全モードの最大／最小
    """
    if not stats:
        raise ValueError("集計が空です")
    seconds = np.maximum.reduce([s.seconds for s in stats])
    total = sum(s.seconds for s in stats)
    weighted = sum(np.nan_to_num(s.mean_accel) * s.seconds for s in stats)
    with np.errstate(invalid="ignore", divide="ignore"):
        mean_accel = np.where(total > 0.0, weighted / np.where(total > 0.0, total, 1.0), np.nan)
    has_pedal = all(
        s.seconds_release_accel is not None and s.seconds_release_brake is not None
        for s in stats
    )
    return WltpCellStats(
        seconds,
        mean_accel,
        np.fmax.reduce([s.max_accel_by_speed for s in stats]),
        np.fmin.reduce([s.min_accel_by_speed for s in stats]),
        np.maximum.reduce([s.seconds_press for s in stats]),  # type: ignore[misc]
        np.maximum.reduce([s.seconds_release for s in stats]),  # type: ignore[misc]
        np.maximum.reduce([s.seconds_release_accel for s in stats]) if has_pedal else None,  # type: ignore[misc]
        np.maximum.reduce([s.seconds_release_brake for s in stats]) if has_pedal else None,  # type: ignore[misc]
    )


def coverage_stats_from_modes(
    modes: list[DrivingMode],
    edges: GridEdges,
    direction_threshold_kmhs: float = DIRECTION_THRESHOLD_KMHS,
    pedal_rule: PedalRule | None = None,
) -> tuple[WltpCellStats, list[tuple[str, float]]]:
    """読み込んだ全モードを合成した集計と、モードごとの格子外の秒数（名前, 秒）。"""
    stats = [
        wltp_cell_stats(
            m, edges, direction_threshold_kmhs=direction_threshold_kmhs, pedal_rule=pedal_rule
        )
        for m in modes
    ]
    outside = [(m.name, outside_seconds(m, edges)) for m in modes]
    return combined_cell_stats(stats), outside


def coverage_stats_for_config(
    cfg: ResearchConfig, modes: list[DrivingMode]
) -> tuple[WltpCellStats, list[tuple[str, float]]]:
    """config と読み込み済みのモードから、網羅の集計を作る（実機の手順2 もスタブも必ずここを通る）。

    格子・向きのしきい値・ペダル別の戻しの判定（`pedal_rule`）をすべて config から決める。
    pedal_rule を渡し忘れると戻しの秒数（ペダル別）が作られず、戻しの穴が数えられない。
    """
    lr = cfg.learning
    return coverage_stats_from_modes(
        modes,
        edges_from_config(lr.grid_speed_edges_kmh, lr.grid_accel_edges_kmhs),
        lr.grid_direction_threshold_kmhs,
        pedal_rule_from_config(cfg),
    )


async def load_coverage_stats(
    cfg: ResearchConfig, edges: GridEdges | None = None, pedal_rule: PedalRule | None = None
) -> tuple[WltpCellStats, list[tuple[str, float]]]:
    """`modes.coverage_mode_names` の全モードを DB から読み、合成した集計と格子外の秒数を返す。

    `edges` / `pedal_rule` は無視する（互換用の引数。作り方は `coverage_stats_for_config`）。
    """
    modes = [await load_mode(cfg, name) for name in cfg.modes.coverage_mode_names]
    return coverage_stats_for_config(cfg, modes)


def outside_text(outside: list[tuple[str, float]]) -> str:
    """モードごとの格子外の秒数（狙えない走り）の表示。"""
    return "\n".join(f"  {name}: 格子外 {sec:.0f}s" for name, sec in outside)


def pedal_release(
    opening: np.ndarray, deadband_pct: float, threshold_pct: float
) -> np.ndarray:
    """1 本のセッションの開度の時系列から、行ごとの「戻し」（True）を返す。

    episode = 開度が不感帯以上で連続する区間（不感帯未満の行は episode の外で False）。
    episode の中は直近の折り返し点から数える（`_reversal_release`）: 踏み込み中は走査中の最大から
    `threshold_pct` 以上下がったら戻し、戻し中は走査中の最小から `threshold_pct` 以上上がったら
    踏み込みに戻る。高速でアクセルが長く不感帯を下回らなくても、ずっと前の山を引きずらない。
    """
    return _reversal_release(opening, opening >= deadband_pct, threshold_pct)


def data_cells(
    logs: list[DriveLog],
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    edges: GridEdges,
    spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    keep: Callable[[DriveLog], bool] | None = None,
    direction_threshold_pct: float = DIRECTION_THRESHOLD_PCT,
    direction_threshold_accel_pct: float = DIRECTION_THRESHOLD_ACCEL_PCT,
) -> DataCells:
    """学習データ（手順2 の行）が各セルに何秒あるか。ペダル判定は A1 と同じ（開度 ≥ 不感帯）。

    向き（2c-1）: ペダルごとに `pedal_release`（直近の折り返し点から しきい値 以上戻したら戻し、
    それ以上踏み直したら踏み込み）で判定する。
    しきい値はブレーキが `direction_threshold_pct`、アクセルは PI のゆらぎより大きい
    `direction_threshold_accel_pct`。`accel`/`brake` は踏み込み側、
    `accel_release`/`brake_release` は戻し側に数える。
    加速度ビンが 0 をまたぐ列（定速）は向きを分けず全部踏み込み側。
    開度は `DriveLog.accel_opening`/`brake_opening` を使う（`read_drive_logs` の `label` で
    指令開度（既定）か PNOW 換算の実開度かが決まる。呼び出し側の読み方に従う）。
    向きの判定は `keep` で絞る前の、セッション全行の時系列で行う。

    `keep` があれば、その行だけを数える（加速度は全行の車速から作ってから絞る。格子ステップ走行の
    「開度固定のステップの行だけ」の網羅を見るため）。
    """
    accel = np.zeros(edges.shape)
    brake = np.zeros(edges.shape)
    coast = np.zeros(edges.shape)
    accel_rel = np.zeros(edges.shape)
    brake_rel = np.zeros(edges.shape)
    for session_logs in group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([log.actual_speed_kmh for log in session_logs], dtype=float), 0.0, None
        )
        timestamps = [log.timestamp for log in session_logs]
        v0, a_req, idx = _v0_and_a_req(speed, timestamps, spec)
        if len(idx) == 0:
            continue
        diffs = np.diff([ts.timestamp() for ts in timestamps])
        diffs = diffs[diffs > 0.0]
        dt_s = float(np.median(diffs)) if len(diffs) else WLTP_DT_S
        if keep is not None:
            selected = np.array([keep(session_logs[i]) for i in idx], dtype=bool)
            v0, a_req, idx = v0[selected], a_req[selected], idx[selected]
        full_a = np.array([log.accel_opening for log in session_logs], dtype=float)
        full_b = np.array([log.brake_opening for log in session_logs], dtype=float)
        rel_a = pedal_release(full_a, accel_deadband_pct, direction_threshold_accel_pct)
        rel_b = pedal_release(full_b, brake_deadband_pct, direction_threshold_pct)
        a_open = full_a[idx]
        b_open = full_b[idx]
        rel_a, rel_b = rel_a[idx], rel_b[idx]  # idx は keep で絞り済み（二重に絞らない）
        cruise = _cruise_mask(a_req, edges)
        rel_a, rel_b = rel_a & ~cruise, rel_b & ~cruise
        is_accel = a_open >= accel_deadband_pct
        is_brake = b_open >= brake_deadband_pct
        for is_on, rel, press_acc, rel_acc in (
            (is_accel, rel_a, accel, accel_rel),
            (is_brake, rel_b, brake, brake_rel),
        ):
            press_acc += _histogram(v0[is_on & ~rel], a_req[is_on & ~rel], edges, dt_s)
            rel_acc += _histogram(v0[is_on & rel], a_req[is_on & rel], edges, dt_s)
        off = ~is_accel & ~is_brake
        coast += _histogram(v0[off], a_req[off], edges, dt_s)
    return DataCells(accel, brake, coast, accel_rel, brake_rel)


@dataclass(frozen=True)
class Hole:
    speed_lo_kmh: float
    speed_hi_kmh: float
    accel_lo_kmhs: float
    accel_hi_kmhs: float
    wltp_s: float
    data_s: float
    direction: str = "press"  # 'press'（踏み込み）／'release'（戻し）
    required_s: float = 0.0  # 向き別の判定で「埋まった」とみなす秒数（find_holes は 0）
    pedal: str = ""  # 戻しの判定に使ったペダル 'A'/'B'（ペダル割り当てなしなら ''）


def required_seconds(
    mode_s: np.ndarray, data_max_s: float, weight_frac: float = WEIGHT_FRAC
) -> np.ndarray:
    """そのマスで学習データが要る秒数 = max(`data_max_s`, `weight_frac` × モードの秒数)。

    モードで長く使うマスほど多くのデータを要求する（向きごとに、その向きのモード秒数で出す）。
    """
    return np.maximum(data_max_s, weight_frac * np.asarray(mode_s, dtype=float))


def _direction_arrays(
    stats: WltpCellStats, data: DataCells, direction: str
) -> tuple[np.ndarray, np.ndarray]:
    """向きごとの (モード秒, データ秒)。direction は 'press' / 'release'（両ペダル合算）/
    'release_brake' / 'release_accel'（ペダル別。`stats` にペダル別の秒数があるときだけ）。"""
    assert stats.seconds_press is not None and stats.seconds_release is not None
    assert data.accel_release is not None and data.brake_release is not None
    if direction == "press":
        return stats.seconds_press, data.press_total
    if direction == "release_brake":
        assert stats.seconds_release_brake is not None
        return stats.seconds_release_brake, data.brake_release
    if direction == "release_accel":
        assert stats.seconds_release_accel is not None
        return stats.seconds_release_accel, data.accel_release
    return stats.seconds_release, data.release_total


def _has_pedal_split(stats: WltpCellStats) -> bool:
    return stats.seconds_release_accel is not None and stats.seconds_release_brake is not None


def find_holes_by_direction(
    stats: WltpCellStats,
    data: DataCells,
    edges: GridEdges,
    wltp_min_s: float,
    data_max_s: float,
    weight_frac: float = WEIGHT_FRAC,
) -> list[Hole]:
    """向きごとの穴。モードのその向きの秒数 ≥ `wltp_min_s` かつ 学習のその向きの秒数 <
    `required_seconds`。学習の秒数は 踏み込み = アクセル + ブレーキ + 惰行。

    戻しは、`stats` にペダル別のモード秒数があれば (マス, ペダル) ごとに判定する
    （ブレーキの戻しの穴 = モードのブレーキ戻し秒数 vs データの brake_release。
    アクセルも同様。1 マスに 2 つ出うる。
    惰行帯の戻しの行は要求しない）。無ければ両ペダル合算で判定する。
    モードの需要が大きい順（踏み込み・戻しを混ぜて並べる）。"""
    kinds: list[tuple[str, str, str]] = [("press", "press", "")]
    if _has_pedal_split(stats):
        kinds += [("release_brake", "release", PEDAL_BRAKE),
                  ("release_accel", "release", PEDAL_ACCEL)]
    else:
        kinds.append(("release", "release", ""))
    holes: list[Hole] = []
    for key, direction, pedal in kinds:
        mode_s, data_s = _direction_arrays(stats, data, key)
        req = required_seconds(mode_s, data_max_s, weight_frac)
        for i in range(mode_s.shape[0]):
            for j in range(mode_s.shape[1]):
                if mode_s[i, j] >= wltp_min_s and data_s[i, j] < req[i, j]:
                    holes.append(
                        Hole(
                            edges.speed_kmh[i], edges.speed_kmh[i + 1],
                            edges.accel_kmhs[j], edges.accel_kmhs[j + 1],
                            float(mode_s[i, j]), float(data_s[i, j]),
                            direction, float(req[i, j]), pedal,
                        )
                    )
    return sorted(holes, key=lambda h: -h.wltp_s)


def find_holes(
    wltp: np.ndarray, data: DataCells, edges: GridEdges, wltp_min_s: float, data_max_s: float
) -> list[Hole]:
    """WLTP が `wltp_min_s` 以上要るのに、学習データ（3 種の合計）が `data_max_s` 未満のセル。
    WLTP の需要が大きい順。"""
    holes: list[Hole] = []
    total = data.total
    for i in range(wltp.shape[0]):
        for j in range(wltp.shape[1]):
            if wltp[i, j] >= wltp_min_s and total[i, j] < data_max_s:
                holes.append(
                    Hole(
                        edges.speed_kmh[i], edges.speed_kmh[i + 1],
                        edges.accel_kmhs[j], edges.accel_kmhs[j + 1],
                        float(wltp[i, j]), float(total[i, j]),
                    )
                )
    return sorted(holes, key=lambda h: -h.wltp_s)


def coverage_table(
    wltp: np.ndarray,
    data: DataCells,
    edges: GridEdges,
    wltp_min_s: float,
    data_max_s: float,
) -> str:
    """セルは「WLTP が要る秒 / 学習データの秒（3 種の合計）」。穴のセルは末尾に * を付ける。"""
    total = data.total
    header = ["車速\\加速度"] + [
        f"{edges.accel_kmhs[j]:g}〜{edges.accel_kmhs[j + 1]:g}" for j in range(wltp.shape[1])
    ]
    rows: list[list[str]] = [header]
    for i in range(wltp.shape[0]):
        row = [f"{edges.speed_kmh[i]:g}〜{edges.speed_kmh[i + 1]:g}"]
        for j in range(wltp.shape[1]):
            hole = wltp[i, j] >= wltp_min_s and total[i, j] < data_max_s
            row.append(f"{wltp[i, j]:.0f}/{total[i, j]:.1f}{'*' if hole else ''}")
        rows.append(row)
    widths = [max(len(r[c]) for r in rows) for c in range(len(header))]
    return "\n".join(
        "  ".join(cell.rjust(widths[c]) for c, cell in enumerate(r)) for r in rows
    )


def summary_text(
    wltp: np.ndarray, data: DataCells, holes: list[Hole], wltp_min_s: float, data_max_s: float
) -> str:
    """穴の件数・WLTP に占める割合・ペダル別の学習データ量。"""
    wltp_total = float(wltp.sum())
    hole_s = sum(h.wltp_s for h in holes)
    covered = float(wltp[data.total >= data_max_s].sum())
    lines = [
        f"WLTP（停車を除く）: {wltp_total:.0f}s",
        f"学習データ: アクセル {data.accel.sum():.0f}s / ブレーキ {data.brake.sum():.0f}s / "
        f"惰行 {data.coast.sum():.0f}s",
        f"学習データが {data_max_s:g}s 以上あるセルが占める WLTP: "
        f"{covered:.0f}s ({100 * covered / max(wltp_total, 1e-9):.0f}%)",
        f"穴（WLTP ≥ {wltp_min_s:g}s かつ 学習 < {data_max_s:g}s）: {len(holes)} セル、"
        f"WLTP {hole_s:.0f}s ({100 * hole_s / max(wltp_total, 1e-9):.0f}%)",
    ]
    return "\n".join(lines)


_DIRECTION_LABEL = {"press": "踏み込み", "release": "戻し"}
_KEY_LABEL = {
    "press": "踏み込み", "release": "戻し（両ペダル合算）",
    "release_brake": "戻し（ブレーキ）", "release_accel": "戻し（アクセル）",
}


def coverage_table_by_direction(
    stats: WltpCellStats,
    data: DataCells,
    edges: GridEdges,
    direction: str,
    wltp_min_s: float,
    data_max_s: float,
    weight_frac: float = WEIGHT_FRAC,
) -> str:
    """向き別の網羅表。セルは「モードの秒 / 学習の秒」（その向き）、穴は末尾に *。
    direction は `_direction_arrays` と同じ（ペダル別は 'release_brake' / 'release_accel'）。"""
    mode_s, data_s = _direction_arrays(stats, data, direction)
    req = required_seconds(mode_s, data_max_s, weight_frac)
    header = ["車速\\加速度"] + [
        f"{edges.accel_kmhs[j]:g}〜{edges.accel_kmhs[j + 1]:g}" for j in range(mode_s.shape[1])
    ]
    rows: list[list[str]] = [header]
    for i in range(mode_s.shape[0]):
        row = [f"{edges.speed_kmh[i]:g}〜{edges.speed_kmh[i + 1]:g}"]
        for j in range(mode_s.shape[1]):
            hole = mode_s[i, j] >= wltp_min_s and data_s[i, j] < req[i, j]
            row.append(f"{mode_s[i, j]:.0f}/{data_s[i, j]:.1f}{'*' if hole else ''}")
        rows.append(row)
    widths = [max(len(r[c]) for r in rows) for c in range(len(header))]
    return "\n".join(
        "  ".join(cell.rjust(widths[c]) for c, cell in enumerate(r)) for r in rows
    )


def summary_text_by_direction(
    stats: WltpCellStats,
    data: DataCells,
    holes: list[Hole],
    direction: str,
    wltp_min_s: float,
    data_max_s: float,
    weight_frac: float = WEIGHT_FRAC,
) -> str:
    """向き別の要約: モードの秒数・学習の秒数・穴の件数と、穴が占めるモードの秒数。"""
    mode_s, data_s = _direction_arrays(stats, data, direction)
    base = "release" if direction.startswith("release") else "press"
    want_pedal = {"release_brake": PEDAL_BRAKE, "release_accel": PEDAL_ACCEL}.get(direction, "")
    mine = [h for h in holes if h.direction == base and h.pedal == want_pedal]
    total = float(mode_s.sum())
    hole_s = sum(h.wltp_s for h in mine)
    return (
        f"[{_KEY_LABEL[direction]}] モード {total:.0f}s / 学習 {float(data_s.sum()):.0f}s、"
        f"穴（モード ≥ {wltp_min_s:g}s かつ 学習 < max({data_max_s:g}s, "
        f"{weight_frac:g}×モード））: {len(mine)} セル、モード {hole_s:.0f}s "
        f"({100 * hole_s / max(total, 1e-9):.0f}%)"
    )


def direction_report(
    stats: WltpCellStats,
    data: DataCells,
    edges: GridEdges,
    wltp_min_s: float,
    data_max_s: float,
    weight_frac: float = WEIGHT_FRAC,
) -> str:
    """踏み込み・戻しの網羅表と要約をまとめた表示（表示専用。走行は変えない）。

    `stats` にペダル別のモード秒数があれば、戻しは「ブレーキ」「アクセル」の 2 枚（各ペダルの
    データだけで判定）。無ければ両ペダル合算の 1 枚（従来）。
    """
    holes = find_holes_by_direction(stats, data, edges, wltp_min_s, data_max_s, weight_frac)
    parts = [
        "向き別: セル = 「モードが要る秒 / 学習データの秒（その向き）」、* = 穴。"
        "学習の踏み込みは 惰行を含む"
    ]
    keys = ["press"]
    keys += ["release_brake", "release_accel"] if _has_pedal_split(stats) else ["release"]
    for key in keys:
        parts.append(f"\n── {_KEY_LABEL[key]} ──")
        parts.append(coverage_table_by_direction(
            stats, data, edges, key, wltp_min_s, data_max_s, weight_frac))
        parts.append(summary_text_by_direction(
            stats, data, holes, key, wltp_min_s, data_max_s, weight_frac))
    return "\n".join(parts)


def holes_text(holes: list[Hole], limit: int = 15) -> str:
    if not holes:
        return "穴はありません"
    lines = ["穴（WLTP が要る順）:"]
    for h in holes[:limit]:
        lines.append(
            f"  {h.speed_lo_kmh:g}〜{h.speed_hi_kmh:g} km/h × "
            f"{h.accel_lo_kmhs:g}〜{h.accel_hi_kmhs:g} km/h/s: "
            f"WLTP {h.wltp_s:.0f}s / 学習 {h.data_s:.1f}s"
            + (f"（{_DIRECTION_LABEL[h.direction]}"
               + (f"・{h.pedal}" if h.pedal else "")
               + f"、要る {h.required_s:.1f}s）"
               if h.required_s > 0.0 else "")
        )
    if len(holes) > limit:
        lines.append(f"  … 他 {len(holes) - limit} セル")
    return "\n".join(lines)


def pedal_rule_from_config(cfg: ResearchConfig) -> PedalRule | None:
    """config の惰行カーブ・クリープ・惰行帯から `PedalRule` を作る。作れなければ None。"""
    try:
        params = feedforward_params(cfg)
        research = research_ff_params(cfg)
        if not params.coast_decel_speeds_kmh or not params.coast_decel_kmhs:
            return None
        return (lambda v: free_accel_at(params, research, v)), research.coast_band_kmhs
    except (ValueError, KeyError, AttributeError):
        return None


def mode_from_csv(path: Path) -> DrivingMode:
    """基準車速を持つ CSV（`mode_time_s`/`time_s` と `ref_speed_kmh`）から DrivingMode を作る。

    手順3 のモード走行 CSV や `wltp_ff.csv` のように、DB を使わずに WLTP を渡したいとき用。
    """
    points: dict[float, float] = {}
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        time_col = next(
            (c for c in ("mode_time_s", "time_s") if reader.fieldnames and c in reader.fieldnames),
            None,
        )
        if time_col is None or "ref_speed_kmh" not in (reader.fieldnames or []):
            raise ConfigError(
                f"{path} に mode_time_s（または time_s）と ref_speed_kmh がありません"
            )
        for row in reader:
            if row[time_col] and row["ref_speed_kmh"]:
                points[float(row[time_col])] = float(row["ref_speed_kmh"])
    if len(points) < 2:
        raise ConfigError(f"{path} の基準車速が 2 点未満です")
    times = sorted(points)
    return DrivingMode(
        id="csv", name=path.stem, description="", total_duration=times[-1] - times[0],
        max_speed=max(points.values()), created_at=datetime.now(tz=UTC), is_system=False,
        reference_speed=[SpeedPoint(time_s=t, speed_kmh=points[t]) for t in times],
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="WLTP の車速×加速度格子に対する、手順2 の学習データの網羅マップを出す"
    )
    parser.add_argument("csv", type=Path, help="手順2 の走行ログ CSV")
    parser.add_argument(
        "--ref-csv", type=Path, default=None,
        help="基準車速を持つ CSV（省略時は DB の modes.coverage_mode_names を読んで合成）",
    )
    args = parser.parse_args(argv)
    try:
        cfg = load_config(DEFAULT_CONFIG_PATH)
        lr = cfg.learning
        rule = pedal_rule_from_config(cfg)
        edges = edges_from_config(lr.grid_speed_edges_kmh, lr.grid_accel_edges_kmhs)
        if args.ref_csv is not None:
            mode = mode_from_csv(args.ref_csv)
            stats = wltp_cell_stats(
                mode, edges, direction_threshold_kmhs=lr.grid_direction_threshold_kmhs,
                pedal_rule=rule,
            )
            outside = [(mode.name, outside_seconds(mode, edges))]
        else:
            stats, outside = asyncio.run(load_coverage_stats(cfg, edges, rule))
    except ConfigError as exc:
        print(f"設定エラー: {exc}")
        return 2
    ff = build_vehicle_profile(cfg).feedforward_params
    logs = read_drive_logs(args.csv)
    if not logs:
        print(f"{args.csv} に手順2（PATTERN_DRIVE）の行がありません")
        return 2
    wltp = stats.seconds
    min_s = lr.grid_target_min_s
    data = data_cells(
        logs, ff.accel_deadband_pct, ff.brake_deadband_pct, edges,
        direction_threshold_pct=lr.grid_direction_threshold_pct,
        direction_threshold_accel_pct=lr.grid_direction_threshold_accel_pct,
    )
    holes = find_holes(wltp, data, edges, min_s, lr.grid_hole_data_max_s)
    print(f"CSV: {args.csv.name}（不感帯 アクセル {ff.accel_deadband_pct:g}% / "
          f"ブレーキ {ff.brake_deadband_pct:g}%。config の値で判定）")
    print("セル = 「モードが要る秒（複数モードは最大） / 学習データの秒」、* = 穴")
    print(
        f"注: 穴の判定は「モードが {min_s:g}s（grid_step_window_s − grid_step_lag_s）以上"
        "要る」マス。"
        "「埋まっている割合」はモデル精度の保証ではない。"
        "データのペダルは合算（アクセル/ブレーキ/惰行）"
    )
    print("格子の外（数えられていない走り）:")
    print(outside_text(outside) + "\n")
    print(coverage_table(wltp, data, edges, min_s, lr.grid_hole_data_max_s))
    print()
    print(summary_text(wltp, data, holes, min_s, lr.grid_hole_data_max_s))
    print(holes_text(holes))
    print(
        f"\n===== 向き別（戻し判定: データ ブレーキ {lr.grid_direction_threshold_pct:g}% / "
        f"アクセル {lr.grid_direction_threshold_accel_pct:g}% / "
        f"モード {lr.grid_direction_threshold_kmhs:g} km/h/s、"
        f"要る秒 = max({lr.grid_hole_data_max_s:g}s, {lr.grid_weight_frac:g}×モード) ====="
    )
    if rule is None:
        print("警告: 惰行カーブからペダル選択の規則を作れないため、戻しは両ペダル合算で判定します")
    print(direction_report(
        stats, data, edges, min_s, lr.grid_hole_data_max_s, lr.grid_weight_frac))
    print(holes_text(find_holes_by_direction(
        stats, data, edges, min_s, lr.grid_hole_data_max_s, lr.grid_weight_frac)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
