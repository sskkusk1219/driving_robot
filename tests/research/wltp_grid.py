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
from tests.research.mode_drive import ReferenceSpeed, load_mode
from tests.research.research_types import DriveLog, DrivingMode, SpeedPoint
from tests.research.vehicle import build_vehicle_profile

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

    @property
    def total(self) -> np.ndarray:
        return self.accel + self.brake + self.coast


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


@dataclass(frozen=True)
class WltpCellStats:
    """格子ステップ走行（ProblemReport_20260925 段3）が狙いを決めるための WLTP の集計。

    seconds / mean_accel: 形は `edges.shape`。mean_accel はセルの中の WLTP の平均加速度
    （滞在 0 のセルは nan）。max/min_accel_by_speed: 車速ビンごとの WLTP の加速度の最大／最小
    （そのビンに滞在が無ければ nan）。
    """

    seconds: np.ndarray
    mean_accel: np.ndarray
    max_accel_by_speed: np.ndarray
    min_accel_by_speed: np.ndarray


def wltp_cell_stats(
    mode: DrivingMode, edges: GridEdges, spec: FeatureSpec = DEFAULT_FEATURE_SPEC
) -> WltpCellStats:
    """WLTP のセルごとの秒数・平均加速度と、車速ビンごとの最大／最小加速度。停車は除く。"""
    v0, a_req = _wltp_v0_and_a_req(mode, spec)
    moving = v0 >= STOP_SPEED_KMH
    v0, a_req = v0[moving], a_req[moving]
    n_speed, n_accel = edges.shape
    seconds = _histogram(v0, a_req, edges, WLTP_DT_S)
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
    return WltpCellStats(seconds, mean_accel, a_max, a_min)


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

    - seconds: モードごとの **最大**（合計にしない。2 つのモードで 1.5s ずつ使うマスが合計 3s で
      1 ステップ分の長さを超えてしまうのを避け、「どれかのモードで 1 ステップ分以上使う」を保つ）
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
    return WltpCellStats(
        seconds,
        mean_accel,
        np.fmax.reduce([s.max_accel_by_speed for s in stats]),
        np.fmin.reduce([s.min_accel_by_speed for s in stats]),
    )


def coverage_stats_from_modes(
    modes: list[DrivingMode], edges: GridEdges
) -> tuple[WltpCellStats, list[tuple[str, float]]]:
    """読み込んだ全モードを合成した集計と、モードごとの格子外の秒数（名前, 秒）。"""
    stats = [wltp_cell_stats(m, edges) for m in modes]
    outside = [(m.name, outside_seconds(m, edges)) for m in modes]
    return combined_cell_stats(stats), outside


async def load_coverage_stats(
    cfg: ResearchConfig, edges: GridEdges
) -> tuple[WltpCellStats, list[tuple[str, float]]]:
    """`modes.coverage_mode_names` の全モードを DB から読み、合成した集計と格子外の秒数を返す。"""
    modes = [await load_mode(cfg, name) for name in cfg.modes.coverage_mode_names]
    return coverage_stats_from_modes(modes, edges)


def outside_text(outside: list[tuple[str, float]]) -> str:
    """モードごとの格子外の秒数（狙えない走り）の表示。"""
    return "\n".join(f"  {name}: 格子外 {sec:.0f}s" for name, sec in outside)


def data_cells(
    logs: list[DriveLog],
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    edges: GridEdges,
    spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    keep: Callable[[DriveLog], bool] | None = None,
) -> DataCells:
    """学習データ（手順2 の行）が各セルに何秒あるか。ペダル判定は A1 と同じ（開度 ≥ 不感帯）。

    `keep` があれば、その行だけを数える（加速度は全行の車速から作ってから絞る。格子ステップ走行の
    「開度固定のステップの行だけ」の網羅を見るため）。
    """
    accel = np.zeros(edges.shape)
    brake = np.zeros(edges.shape)
    coast = np.zeros(edges.shape)
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
        a_open = np.array([session_logs[i].accel_opening for i in idx], dtype=float)
        b_open = np.array([session_logs[i].brake_opening for i in idx], dtype=float)
        is_accel = a_open >= accel_deadband_pct
        is_brake = b_open >= brake_deadband_pct
        accel += _histogram(v0[is_accel], a_req[is_accel], edges, dt_s)
        brake += _histogram(v0[is_brake], a_req[is_brake], edges, dt_s)
        off = ~is_accel & ~is_brake
        coast += _histogram(v0[off], a_req[off], edges, dt_s)
    return DataCells(accel=accel, brake=brake, coast=coast)


@dataclass(frozen=True)
class Hole:
    speed_lo_kmh: float
    speed_hi_kmh: float
    accel_lo_kmhs: float
    accel_hi_kmhs: float
    wltp_s: float
    data_s: float


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


def holes_text(holes: list[Hole], limit: int = 15) -> str:
    if not holes:
        return "穴はありません"
    lines = ["穴（WLTP が要る順）:"]
    for h in holes[:limit]:
        lines.append(
            f"  {h.speed_lo_kmh:g}〜{h.speed_hi_kmh:g} km/h × "
            f"{h.accel_lo_kmhs:g}〜{h.accel_hi_kmhs:g} km/h/s: "
            f"WLTP {h.wltp_s:.0f}s / 学習 {h.data_s:.1f}s"
        )
    if len(holes) > limit:
        lines.append(f"  … 他 {len(holes) - limit} セル")
    return "\n".join(lines)


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
        edges = edges_from_config(lr.grid_speed_edges_kmh, lr.grid_accel_edges_kmhs)
        if args.ref_csv is not None:
            mode = mode_from_csv(args.ref_csv)
            stats = wltp_cell_stats(mode, edges)
            outside = [(mode.name, outside_seconds(mode, edges))]
        else:
            stats, outside = asyncio.run(load_coverage_stats(cfg, edges))
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
    data = data_cells(logs, ff.accel_deadband_pct, ff.brake_deadband_pct, edges)
    holes = find_holes(wltp, data, edges, min_s, lr.grid_hole_data_max_s)
    print(f"CSV: {args.csv.name}（不感帯 アクセル {ff.accel_deadband_pct:g}% / "
          f"ブレーキ {ff.brake_deadband_pct:g}%。config の値で判定）")
    print("セル = 「モードが要る秒（複数モードは最大） / 学習データの秒」、* = 穴")
    print(
        f"注: 穴の判定は「モードが {min_s:g}s（1 ステップで測れる長さ）以上要る」マス。"
        "「埋まっている割合」はモデル精度の保証ではない。"
        "データのペダルは合算（アクセル/ブレーキ/惰行）"
    )
    print("格子の外（数えられていない走り）:")
    print(outside_text(outside) + "\n")
    print(coverage_table(wltp, data, edges, min_s, lr.grid_hole_data_max_s))
    print()
    print(summary_text(wltp, data, holes, min_s, lr.grid_hole_data_max_s))
    print(holes_text(holes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
