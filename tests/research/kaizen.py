"""手順 2・手順 3 の改善案を簡易車両モデルで模擬して比べる（ProblemReport_20260912 3.）。

車両・アクチュエータには触らない。表をターミナルに出し、図を --out に保存する
（レポート reportYYYYMMDD_KAIZEN_process2,3.md に使う）。

    .venv/bin/python -m tests.research.kaizen --part sim-check

    --part sim-check  段階 1: 簡易車両モデル（tests/research/vehicle_sim.py）が実車と合うか
        K-1 ペダル応答の表（手順 2・手順 3 の実走行ログから同定）と config のペダルゲインの比例
        K-2 開ループ再生: 実走行の指令開度だけで 5s 走らせたときの車速の誤差
            （1s ごとに実車速から再開）。車両モデル × むだ時間 × 一次遅れ で比べ、
            手順 3 のログで誤差が最小の組を選ぶ
        K-3 閉ループ再現: 手順 3 と同じ FF 指令で WLTP を模擬し、手順 3 の実走行
            （926s で中断）と区間別・走行状態別の平均偏差・中断時刻を比べる

    --part compare    段階 2: FF の改善案を車両モデル（段階 1 で選んだ表・むだ時間 0s）で比べる
        C0 現行（基準）
        C1 惰行カーブでペダルを選び、そのペダルが効いている行だけで学習し、出力を不感帯以上にする
        C2 C1 のブレーキ側を物理式（不感帯 + Δa/ゲイン）にする
        C3 C1 の先読み窓を実測の遅れ（0.5s）だけずらして学習し直す
        C4 C1 の動作点 v0 を実車速にする（要求加速度・先読みは基準車速のまま）
        C5 C1 の t 以前を実測・t 以降を基準の絶対値にする（FF の中にフィードバックが入る）
        判定は WLTP 1800s の閉ループ模擬（KPI・走行状態別の偏差・効くペダルの割合・
        不感帯内の指令・開度の跳び）と、ペダル応答 ±20% での頑健性。

    （--part coverage＝手順 2 パターン走行の網羅性は、2026-09-25 段4 で旧パターンとともに削除した。
    WLTP の網羅マップは `python -m tests.research.wltp_grid`。）
"""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, ResearchConfig, load_config
from tests.research.drive_log import SECTION_MODE_DRIVE, SECTION_PATTERN_DRIVE
from tests.research.ff_explain import (
    FFModel,
    _plt,
    decide_all,
    load_ff_model,
    md_table,
    outputs_for_mode,
    outputs_from_points,
    pedal_class,
)
from tests.research.ff_model import (
    DEFAULT_FEATURE_SPEC,
    STOP_SPEED_KMH,
    FeatureSpec,
    build_feature_matrix,
    estimate_offsets,
    make_estimator,
)
from tests.research.kpi import compute_kpi
from tests.research.live_plot import COLOR_ACTUAL, COLOR_REF
from tests.research.mode_drive import ReferenceSpeed, load_mode
from tests.research.mode_report import STATES, ModeRow, driving_states, rows_from_csv
from tests.research.model_analysis import (
    DEFAULT_TRAIN_CSV,
    ff_openings,
    in_deadband,
    load_training_rows,
    switch_count,
)
from tests.research.research_types import (
    VEHICLE_STOP_SPEED_KMH,
    FeedforwardParams,
    coast_decel_at,
    pedal_gain_at,
)
from tests.research.vehicle import feedforward_params
from tests.research.vehicle_sim import (
    OVER_EDGES_PCT,
    PEDAL_ACCEL,
    PEDAL_BRAKE,
    PEDAL_COAST,
    SIM_DT_S,
    SPEED_EDGES_KMH,
    LogSeries,
    ReplayResult,
    SimRun,
    VehicleModel,
    coast_decel,
    identify_response,
    proportional_response,
    read_log,
    replay,
    scale_response,
    simulate,
)

# 手順 3（FF のみでモード走行）の実走行ログ
DEFAULT_RUN_CSV = Path("tests/research/results/drive_log_real_20260911_171637.csv")
LOG_RUN = "手順 3"
LOG_TRAIN = "手順 2"
MODEL_PROP = "config ゲイン比例"
MODEL_TABLE = "表（手順 2＋3）"
MODEL_TABLE_TRAIN = "表（手順 2 のみ）"
DELAYS_S = (0.0, 0.2, 0.3, 0.5, 0.8)
LAGS_S = (0.0, 0.5, 1.0)
REPLAY_HORIZON_S = 5.0
REPLAY_EVERY_S = 1.0
TABLE_OVER_PCT = (1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0)
ZOOMS_S = ((20.0, 120.0), (860.0, 930.0))
SIM_COLORS = ("#3a7bd5", "#2e7d32", "#8e44ad")
PEDAL_NAMES = {PEDAL_ACCEL: "アクセル", PEDAL_BRAKE: "ブレーキ", PEDAL_COAST: "惰行"}


# ─────────────────────────────────────────────────────────────────────
# K-1 車両モデル
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class Models:
    proportional: VehicleModel
    table: VehicleModel  # 手順 2＋3 のログから同定
    table_train: VehicleModel  # 手順 2 のログだけから同定（手順 3 のログは同定に使わない）
    counts: dict[str, np.ndarray]  # "accel" / "brake" → 同定に使った行数 [速度帯 × 開度帯]

    @property
    def all(self) -> tuple[VehicleModel, ...]:
        return (self.proportional, self.table, self.table_train)


def build_models(p: FeedforwardParams, run: LogSeries, train: LogSeries) -> Models:
    prop = VehicleModel(
        MODEL_PROP,
        p,
        proportional_response("アクセル", p.pedal_gain_speeds_kmh, p.accel_gain_kmhs_per_pct),
        proportional_response("ブレーキ", p.pedal_gain_speeds_kmh, p.brake_gain_kmhs_per_pct),
    )
    accel, accel_n = identify_response("アクセル", [run, train], p, is_accel=True)
    brake, brake_n = identify_response("ブレーキ", [run, train], p, is_accel=False)
    accel_t, _ = identify_response("アクセル", [train], p, is_accel=True)
    brake_t, _ = identify_response("ブレーキ", [train], p, is_accel=False)
    return Models(
        proportional=prop,
        table=VehicleModel(MODEL_TABLE, p, accel, brake),
        table_train=VehicleModel(MODEL_TABLE_TRAIN, p, accel_t, brake_t),
        counts={"accel": accel_n, "brake": brake_n},
    )


def response_table(models: Models, *, is_accel: bool) -> str:
    """表（手順 2＋3）の値と、同じ点の config ゲイン比例（括弧内）。"""
    table = models.table.accel_response if is_accel else models.table.brake_response
    prop = models.proportional.accel_response if is_accel else models.proportional.brake_response
    speeds = table.speeds_kmh
    rows = [
        [f"+{u:g}%"]
        + [f"{float(table.at(v, u)):.2f}（{float(prop.at(v, u)):.2f}）" for v in speeds]
        for u in TABLE_OVER_PCT
    ]
    return md_table(["不感帯超の開度", *[f"{v:.0f} km/h" for v in speeds]], rows)


def counts_table(counts: np.ndarray) -> str:
    speed_bins = [f"{lo:.0f}〜{hi:.0f}" for lo, hi in zip(SPEED_EDGES_KMH, SPEED_EDGES_KMH[1:],
                                                       strict=False)]
    over_bins = [f"+{lo:g}〜{hi:g}" for lo, hi in zip(OVER_EDGES_PCT, OVER_EDGES_PCT[1:],
                                                     strict=False)]
    rows = [[ob, *[str(counts[j, k]) for j in range(len(speed_bins))]]
            for k, ob in enumerate(over_bins)]
    return md_table(["開度帯 ＼ 車速[km/h]", *speed_bins], rows)


# ─────────────────────────────────────────────────────────────────────
# K-2 開ループ再生
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ReplayRow:
    model: str
    delay_s: float
    lag_s: float
    log: str
    result: ReplayResult

    @property
    def rmse(self) -> float:
        return self.result.summary()[0]


def replay_grid(
    models: Sequence[VehicleModel],
    logs: Sequence[LogSeries],
    delays: Sequence[float] = DELAYS_S,
    lags: Sequence[float] = LAGS_S,
) -> list[ReplayRow]:
    rows = []
    for m in models:
        for d in delays:
            for lag in lags:
                vm = m.with_delay(d, lag)
                for lg in logs:
                    result = replay(vm, lg, horizon_s=REPLAY_HORIZON_S, every_s=REPLAY_EVERY_S)
                    rows.append(ReplayRow(m.name, d, lag, lg.name, result))
    return rows


def best_row(rows: Sequence[ReplayRow], model: str, log: str) -> ReplayRow:
    """model・log の組み合わせのうち、再生の RMSE が最小の行。"""
    cands = [r for r in rows if r.model == model and r.log == log]
    if not cands:
        raise ValueError(f"{model} / {log} の再生結果がありません")
    return min(cands, key=lambda r: r.rmse)


def _cell(result: ReplayResult) -> str:
    rmse, _, mean, _ = result.summary()
    parts = [f"{result.summary(k)[0]:.2f}" for k in (PEDAL_ACCEL, PEDAL_BRAKE, PEDAL_COAST)]
    return f"{rmse:.2f}（{' / '.join(parts)}）平均 {mean:+.2f}"


def replay_table(rows: Sequence[ReplayRow], logs: Sequence[str]) -> str:
    keys = list(dict.fromkeys((r.model, r.delay_s, r.lag_s) for r in rows))
    body = []
    for model, d, lag in keys:
        cells = []
        for lg in logs:
            match = [r for r in rows if (r.model, r.delay_s, r.lag_s, r.log) == (model, d, lag, lg)]
            cells.append(_cell(match[0].result) if match else "—")
        body.append([model, f"{d:g}", f"{lag:g}", *cells])
    headers = ["車両モデル", "むだ時間[s]", "一次遅れ[s]",
               *[f"{lg}: RMSE（アクセル / ブレーキ / 惰行）[km/h]" for lg in logs]]
    return md_table(headers, body)


def replay_windows_table(row: ReplayRow) -> str:
    body = []
    for key in (PEDAL_ACCEL, PEDAL_BRAKE, PEDAL_COAST, None):
        rmse, p95, mean, n = row.result.summary(key)
        body.append([PEDAL_NAMES.get(key, "全体") if key else "全体", n,
                     f"{rmse:.2f}", f"{p95:.2f}", f"{mean:+.2f}"])
    return md_table(["窓の中で効いたペダル", "窓の数", "RMSE[km/h]", "|誤差| p95[km/h]",
                     "平均[km/h]（+ は模擬が速い）"], body)


# ─────────────────────────────────────────────────────────────────────
# K-3 閉ループ再現
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class FFCommands:
    t: np.ndarray  # SIM_DT_S 刻み
    ref: np.ndarray
    accel: np.ndarray
    brake: np.ndarray


def step3_commands(cfg: ResearchConfig, p: FeedforwardParams) -> FFCommands:
    """手順 3 と同じ FF 指令（predict_effort を分岐ごとに写した decide_all → 符号で振り分け）。"""
    model = load_ff_model(cfg.feedforward.model_path)
    mode = asyncio.run(load_mode(cfg, cfg.modes.wltp_mode_name))
    ref = ReferenceSpeed(mode)
    t = np.round(np.arange(0.0, mode.total_duration + SIM_DT_S / 2, SIM_DT_S), 4)
    out = outputs_for_mode(model, ref, t)
    accel, brake = ff_openings(
        decide_all(p, out).effort, cfg.vehicle.max_accel_opening_pct,
        cfg.vehicle.max_brake_opening_pct,
    )
    return FFCommands(t=t, ref=out.v0_raw, accel=accel, brake=brake)


def run_closed_loop(model: VehicleModel, cmd: FFCommands, stop_above_kmh: float) -> SimRun:
    return simulate(
        model, len(cmd.t), lambda i, _v: (float(cmd.accel[i]), float(cmd.brake[i])),
        dt=SIM_DT_S, stop_above_kmh=stop_above_kmh,
    )


@dataclass(frozen=True)
class GroupDeviation:
    name: str
    duration_s: float
    real_mean: float
    sim_means: tuple[float, ...]
    real_p95: float
    sim_p95s: tuple[float, ...]


def deviation_by_group(
    keys: np.ndarray,
    order: Sequence[str],
    real_dev: np.ndarray,
    sim_devs: Sequence[np.ndarray],
    dt: float,
) -> list[GroupDeviation]:
    """グループごとの平均偏差と |偏差| p95。模擬が先に打ち切られた時刻（NaN）は数えない。"""
    out = []
    for name in order:
        m = keys == name
        if not np.any(m):
            continue

        def stats(dev: np.ndarray, m: np.ndarray = m) -> tuple[float, float]:
            d = dev[m & np.isfinite(dev)]
            if d.size == 0:
                return float("nan"), float("nan")
            return float(np.mean(d)), float(np.percentile(np.abs(d), 95))

        real = stats(real_dev)
        sims = [stats(d) for d in sim_devs]
        out.append(GroupDeviation(
            name=name, duration_s=float(np.count_nonzero(m)) * dt,
            real_mean=real[0], sim_means=tuple(s[0] for s in sims),
            real_p95=real[1], sim_p95s=tuple(s[1] for s in sims),
        ))
    return out


def group_table(first: str, groups: Sequence[GroupDeviation], labels: Sequence[str]) -> str:
    headers = [first, "時間[s]", "実走行 平均偏差",
               *[f"{lb} 平均偏差" for lb in labels], "実走行 |偏差| p95",
               *[f"{lb} |偏差| p95" for lb in labels]]
    body = [
        [g.name, f"{g.duration_s:.0f}", f"{g.real_mean:+.2f}",
         *[f"{x:+.2f}" for x in g.sim_means], f"{g.real_p95:.2f}",
         *[f"{x:.2f}" for x in g.sim_p95s]]
        for g in groups
    ]
    return md_table(headers, body)


def fig_overlay(
    real_t: np.ndarray, real_ref: np.ndarray, real_speed: np.ndarray,
    sims: Sequence[tuple[str, SimRun]], cmd: FFCommands, t0: float, t1: float, path: Path,
) -> None:
    """1 段目 = 基準・実走行・模擬の車速、2 段目 = 偏差（実車速 − 基準車速）。"""
    plt = _plt()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    m = (real_t >= t0) & (real_t <= t1)
    mc = (cmd.t >= t0) & (cmd.t <= t1)
    ax1.plot(cmd.t[mc], cmd.ref[mc], color=COLOR_REF, linestyle="--", linewidth=1.2,
             label="基準車速")
    ax1.plot(real_t[m], real_speed[m], color=COLOR_ACTUAL, linewidth=1.4, label="実走行（手順 3）")
    ax2.plot(real_t[m], real_speed[m] - real_ref[m], color=COLOR_ACTUAL, linewidth=1.4,
             label="実走行")
    for (label, run), color in zip(sims, SIM_COLORS, strict=False):
        ms = (run.t >= t0) & (run.t <= t1)
        ax1.plot(run.t[ms], run.speed[ms], color=color, linewidth=1.0, label=f"模擬: {label}")
        ref_at = np.interp(run.t[ms], cmd.t, cmd.ref)
        ax2.plot(run.t[ms], run.speed[ms] - ref_at, color=color, linewidth=1.0,
                 label=f"模擬: {label}")
    ax1.set_ylabel("車速 [km/h]")
    ax2.set_ylabel("偏差 [km/h]")
    ax2.set_xlabel("モード経過時間 [s]")
    ax2.axhline(0.0, color="#555555", linewidth=0.8)
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
    fig.suptitle(f"手順 3 の FF 指令: 実走行と簡易車両モデルの模擬（{t0:.0f}〜{t1:.0f}s）")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────
# 段階 2: FF の改善案
# ─────────────────────────────────────────────────────────────────────

C0, C1, C2, C3, C4, C5 = "C0", "C1", "C2", "C3", "C4", "C5"
CANDIDATE_NAMES = {
    C0: "C0 現行",
    C1: "C1 惰行カーブで選ぶ＋効く行だけ学習＋不感帯以上",
    C2: "C2 C1＋ブレーキは物理式",
    C3: "C3 C1＋先読みを遅れ分ずらす",
    C4: "C4 C1＋動作点は実車速",
    C5: "C5 C1＋t 以前は実測・t 以降は基準の絶対値",
}
CANDIDATE_COLORS = {
    C0: "#c05050", C1: "#2e7d32", C2: "#3a7bd5", C3: "#8e44ad", C4: "#c8922a",
    C5: "#0f7d7d",
}
SHIFT_S = 0.5  # C3 の先読みのずらし（2 章で実測したペダルの遅れ）
ROBUST_SCALES = (0.8, 1.0, 1.2)  # ペダル応答の倍率（車両モデルが外れていた場合）
COMPARE_ZOOMS_S = ((20.0, 120.0), (860.0, 930.0), (1550.0, 1650.0))


# 移植元: src/domain/control/pedal_plan.py の coast_accel / analytic_efforts
# （`tests/` だけで完結させるため持ち込んだ。ロジックは同じ。ProblemReport_20260924）

def coast_accel(v: float, params: FeedforwardParams) -> float:
    """速度 v [km/h] での惰行加速度 a_coast [km/h/s]（ペダル未操作時）。

    クリープ速度未満はクリープが車を押す（+creep_rate）、以上は惰行減速カーブ
    （coast_decel_at: 同定済みなら速度依存の補間、未同定は engine_brake_decel 定数）で
    減速する。フェーズ分類の基準線。速度依存を無視すると、実惰行が基準より強い速度域で
    緩減速を BRAKE と誤分類し、プランが必要な正 effort を出せなくなる（sample_004 実機）。
    """
    if v < params.creep_speed_kmh:
        return params.creep_rate_kmhs
    return -coast_decel_at(params, v)


def analytic_efforts(
    speeds: np.ndarray, accels: np.ndarray, params: FeedforwardParams
) -> np.ndarray:
    """惰行カーブとペダルゲインから必要 effort [%] を解析的に求める（同定なしは NaN）。

    effort = (a_req − a_coast(v)) / ペダルゲイン(v) + 不感帯。Δa=0（＝惰行そのまま）で
    effort=0 が構造的に保証されるため、学習データが無い低開度域が「モデルの外挿」ではなく
    「原点との内挿」になる。不感帯を足すのは PedalArbiter が開度を max(deadband, |effort|)
    に丸めるため（ゲインは不感帯超の開度で同定している。model_training.
    _estimate_pedal_gain_curve 参照）。

    ペダルゲイン未同定の向き・停車域は np.nan を返し、呼び出し元がモデル出力へ
    フォールバックできるようにする。
    """
    out = np.full(len(speeds), np.nan, dtype=float)
    accel_db = max(0.0, params.accel_deadband_pct)
    brake_db = max(0.0, params.brake_deadband_pct)
    for i, (v, a) in enumerate(zip(speeds, accels)):
        v_f = float(v)
        if v_f < VEHICLE_STOP_SPEED_KMH:
            continue  # 停車保持は clamp_effort_by_phase の保持 effort が支配する
        delta_a = float(a) - coast_accel(v_f, params)
        if delta_a > 0.0:
            gain = pedal_gain_at(params, v_f, is_accel=True)
            if gain is not None:
                out[i] = delta_a / gain + accel_db
        elif delta_a < 0.0:
            gain = pedal_gain_at(params, v_f, is_accel=False)
            if gain is not None:
                out[i] = -(-delta_a / gain + brake_db)
        else:
            out[i] = 0.0
    return out


def coast_accel_array(p: FeedforwardParams, speed_kmh: np.ndarray) -> np.ndarray:
    """両ペダルを離したときの加速度（本番 pedal_plan.coast_accel の配列版）。"""
    return np.where(speed_kmh < p.creep_speed_kmh, p.creep_rate_kmhs, -coast_decel(p, speed_kmh))


@dataclass(frozen=True)
class TrainedModels:
    """改善案の逆モデル（そのペダルが効いている行だけで学習したもの）。"""

    label: str
    ff: FFModel
    shift_s: float
    rows: tuple[int, int]  # (アクセル, ブレーキ) の学習行数
    mae: tuple[float, float]  # 学習行での MAE [%]
    below_db: tuple[float, float]  # 予測が不感帯未満だった割合（切り上げ前）


def _shift_samples(timestamps: Sequence[datetime], shift_s: float) -> int:
    if shift_s <= 0.0:
        return 0
    diffs = np.diff([ts.timestamp() for ts in timestamps])
    dt = float(np.median(diffs[diffs > 0.0])) if np.any(diffs > 0.0) else 0.1
    return int(round(shift_s / dt)) if dt > 0.0 else 0


def _feature_matrix(
    train_csv: Path, spec: FeatureSpec
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, list[datetime], float, np.ndarray]:
    """学習 CSV から特徴行列と、行に対応するアクセル・ブレーキ開度・パターン種別を返す。

    最後の要素は CSV の全行ぶんのパターン種別（`12:ACCEL_SWEEP` の `":"` より後ろ）で、
    行番号（`idx` / ラベル行）で引ける。段階 3 の leave-pattern-out が使う。
    """
    logs, patterns, _ = load_training_rows(train_csv)
    speed = np.clip(np.array([lg.actual_speed_kmh for lg in logs], dtype=float), 0.0, None)
    accel_open = np.array([lg.accel_opening for lg in logs], dtype=float)
    brake_open = np.array([lg.brake_opening for lg in logs], dtype=float)
    timestamps = [lg.timestamp for lg in logs]
    x, idx = build_feature_matrix(
        speed,
        estimate_offsets(timestamps, spec.lookahead_horizons_s),
        estimate_offsets(timestamps, spec.past_horizons_s),
        spec,
    )
    kinds = np.array([s.split(":", 1)[-1] for s in patterns])
    return x, idx, accel_open, brake_open, timestamps, float(speed.max()), kinds


def train_models(
    train_csv: Path,
    p: FeedforwardParams,
    *,
    shift_s: float = 0.0,
    label: str = "",
    exclude_kinds: Sequence[str] = (),
) -> TrainedModels:
    """そのペダルが効いている行だけで学習する（惰行の行はどちらのモデルにも入れない）。

    shift_s > 0 なら「今の開度は shift_s 先から始まる動きを作る」として、特徴量を
    shift_s ぶん先の行から作る（C3）。
    exclude_kinds を渡すと、その系統（ACCEL_SWEEP 等）のラベル行を学習から外す
    （段階 3 の leave-pattern-out。パターン構成が指令にどれだけ効くかを測る）。
    """
    spec = DEFAULT_FEATURE_SPEC
    x, idx, accel_open, brake_open, timestamps, clip_max, kinds = _feature_matrix(train_csv, spec)
    shift = _shift_samples(timestamps, shift_s)
    label_idx = idx - shift
    keep = label_idx >= 0
    x, label_idx = x[keep], label_idx[keep]
    if len(exclude_kinds) > 0:
        stay = ~np.isin(kinds[label_idx], list(exclude_kinds))
        x, label_idx = x[stay], label_idx[stay]
    a_lab, b_lab = accel_open[label_idx], brake_open[label_idx]
    a_mask = a_lab >= p.accel_deadband_pct
    b_mask = b_lab >= p.brake_deadband_pct
    accel_model = make_estimator().fit(x[a_mask], a_lab[a_mask])
    brake_model = make_estimator().fit(x[b_mask], b_lab[b_mask])
    a_pred = np.maximum(0.0, accel_model.predict(x[a_mask]))
    b_pred = np.maximum(0.0, brake_model.predict(x[b_mask]))
    return TrainedModels(
        label=label or f"ずらし {shift_s:g}s",
        ff=FFModel(
            path=str(train_csv), accel_model=accel_model, brake_model=brake_model,
            spec=spec, speed_clip_max=clip_max,
        ),
        shift_s=shift_s,
        rows=(int(a_mask.sum()), int(b_mask.sum())),
        mae=(
            float(np.mean(np.abs(a_pred - a_lab[a_mask]))),
            float(np.mean(np.abs(b_pred - b_lab[b_mask]))),
        ),
        below_db=(
            float(np.mean(a_pred < p.accel_deadband_pct)),
            float(np.mean(b_pred < p.brake_deadband_pct)),
        ),
    )


def current_model_stats(
    train_csv: Path, p: FeedforwardParams, ff: FFModel
) -> tuple[tuple[int, int], tuple[float, float], tuple[float, float]]:
    """C0（現行）の学習セットでの行数・MAE・不感帯未満の予測の割合。"""
    spec = ff.spec
    x, idx, accel_open, brake_open, _, _, _ = _feature_matrix(train_csv, spec)
    accel_mask = x[:, spec.regime_col()] >= 0.0
    a_lab = accel_open[idx][accel_mask]
    b_lab = np.where(brake_open >= p.brake_deadband_pct, brake_open, 0.0)[idx][~accel_mask]
    a_pred = np.maximum(0.0, ff.accel_model.predict(x[accel_mask]))
    b_pred = np.maximum(0.0, ff.brake_model.predict(x[~accel_mask]))
    return (
        (len(a_lab), len(b_lab)),
        (float(np.mean(np.abs(a_pred - a_lab))), float(np.mean(np.abs(b_pred - b_lab)))),
        (
            float(np.mean(a_pred < p.accel_deadband_pct)),
            float(np.mean(b_pred < p.brake_deadband_pct)),
        ),
    )


@dataclass(frozen=True)
class RefFrames:
    """基準車速から作った、各周期のモデル入力（実車速に依らない部分）。"""

    t: np.ndarray
    v0_raw: np.ndarray  # ref(t)（停車判定に使う）
    near: np.ndarray  # ref(t + 最短ホライズン)（停車判定に使う）
    v0_model: np.ndarray  # モデルに渡す v0（C3 は ref(t + ずらし)）
    dv_future: np.ndarray  # 先読みの変化量（n, ホライズン数）
    dv_past: np.ndarray  # 過去の変化量（n, 過去ホライズン数）
    a_req: np.ndarray  # 要求加速度 [km/h/s]


def ref_frames(
    ref: ReferenceSpeed, t: np.ndarray, spec: FeatureSpec, shift_s: float = 0.0
) -> RefFrames:
    base = np.array([ref.at(float(x) + shift_s) for x in t])
    dv_future = np.column_stack(
        [[ref.at(float(x) + shift_s + h) for x in t] for h in spec.lookahead_horizons_s]
    ) - base[:, None]
    dv_past = base[:, None] - np.column_stack(
        [[ref.at(float(x) + shift_s - h) for x in t] for h in spec.past_horizons_s]
    )
    col = spec.lookahead_horizons_s.index(spec.regime_horizon_s)
    return RefFrames(
        t=t,
        v0_raw=np.array([ref.at(float(x)) for x in t]),
        near=np.array([ref.at(float(x) + spec.lookahead_horizons_s[0]) for x in t]),
        v0_model=base,
        dv_future=dv_future,
        dv_past=dv_past,
        a_req=dv_future[:, col] / spec.regime_horizon_s,
    )


def decide_openings(
    p: FeedforwardParams,
    cfg: ResearchConfig,
    v0_raw: np.ndarray,
    near: np.ndarray,
    v0: np.ndarray,
    a_req: np.ndarray,
    accel_pred: np.ndarray,
    brake_pred: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """改善案（C1〜C4）の指令の決め方。

    1. 停車保持（現行と同じ）
    2. クリープ任せ（現行と同じ）
    3. 要求加速度が惰行以上ならアクセル、下ならブレーキ（現行は dv_1.0 の符号。惰行テーパは無い）
    4. 選んだペダルの開度は不感帯以上に切り上げる（不感帯の中の指令は出さない）
    """
    stop = (v0_raw <= STOP_SPEED_KMH) & (near <= STOP_SPEED_KMH)
    creep = (~stop) & (a_req >= 0.0) & (v0 < p.creep_speed_kmh) & (a_req <= p.creep_rate_kmhs)
    accel_side = (~stop) & (~creep) & (a_req >= coast_accel_array(p, v0))
    brake_side = (~stop) & (~creep) & (~accel_side)
    accel = np.where(accel_side, np.maximum(p.accel_deadband_pct, accel_pred), 0.0)
    brake = np.where(brake_side, np.maximum(p.brake_deadband_pct, brake_pred), 0.0)
    hold = min(p.stop_brake_opening_pct, cfg.vehicle.max_brake_opening_pct)
    brake = np.where(stop, hold, brake)
    return (
        np.minimum(accel, cfg.vehicle.max_accel_opening_pct),
        np.minimum(brake, cfg.vehicle.max_brake_opening_pct),
    )


def candidate_openings(
    key: str, models: TrainedModels, p: FeedforwardParams, cfg: ResearchConfig, frames: RefFrames
) -> tuple[np.ndarray, np.ndarray]:
    """C1〜C3 の指令（実車速に依らないので先に全周期ぶん計算できる）。"""
    futures = [list(frames.v0_model[i] + frames.dv_future[i]) for i in range(len(frames.t))]
    pasts = [list(frames.v0_model[i] - frames.dv_past[i]) for i in range(len(frames.t))]
    out = outputs_from_points(models.ff, frames.t, frames.v0_model, futures, pasts)
    accel_pred = np.maximum(0.0, out.accel_raw)
    brake_pred = np.maximum(0.0, out.brake_raw)
    if key == C2:
        effort = analytic_efforts(out.v0, out.a_req, p)
        physical = np.where(np.isfinite(effort) & (effort < 0.0), -effort, np.nan)
        brake_pred = np.where(np.isnan(physical), brake_pred, physical)
    return decide_openings(
        p, cfg, frames.v0_raw, frames.near, out.v0, out.a_req, accel_pred, brake_pred
    )


@dataclass(frozen=True)
class ActualSpeedCommand:
    """C4: 動作点 v0 だけ実車速にする（先読みの変化量は基準車速のまま）。

    偏差そのものは入れない。入れると PID（手順 4）と同じ働きになるため、ここでは分けて扱う。
    """

    models: TrainedModels
    p: FeedforwardParams
    cfg: ResearchConfig
    frames: RefFrames

    def __call__(self, i: int, v_actual: float) -> tuple[float, float]:
        f = self.frames
        clip = self.models.ff.speed_clip_max
        v0 = min(float(v_actual), clip) if clip is not None else float(v_actual)
        out = outputs_from_points(
            self.models.ff, [f.t[i]], [v0],
            [[v0 + dv for dv in f.dv_future[i]]], [[v0 - dv for dv in f.dv_past[i]]],
        )
        accel, brake = decide_openings(
            self.p, self.cfg, f.v0_raw[i : i + 1], f.near[i : i + 1], out.v0, out.a_req,
            np.maximum(0.0, out.accel_raw), np.maximum(0.0, out.brake_raw),
        )
        return float(accel[0]), float(brake[0])


@dataclass
class ReplanCommand:
    """C5: t 以前は実測（動作点 v0・過去の変化量）、t 以降は基準の絶対値 ref(t+h)。

    C4 は基準の「増分」を実車速に足すので偏差が消え、遅れても基準へ戻ろうとしない。こちらは
    未来を絶対値で渡すため dv = ref(t+h) − v_実測(t) になり、遅れるほど要求 Δv が増える。
    つまり **FF の中にフィードバックが入る**（手順 4 の PID は誤差の積分、こちらは逆モデルの
    再計画なので役割が違う）。過去 2 列は自分が受け取った実車速の履歴から作り、履歴が足りない
    開始直後だけ基準の変化量で代用する。
    """

    models: TrainedModels
    p: FeedforwardParams
    cfg: ResearchConfig
    frames: RefFrames
    history: list[float]

    def __call__(self, i: int, v_actual: float) -> tuple[float, float]:
        f = self.frames
        self.history.append(float(v_actual))
        clip = self.models.ff.speed_clip_max
        v0 = min(float(v_actual), clip) if clip is not None else float(v_actual)
        # 未来は ref(t+h) の絶対値（= v0_model + dv_future）。実車速には足さない。
        future = [float(f.v0_model[i] + dv) for dv in f.dv_future[i]]
        dt = float(f.t[1] - f.t[0])
        past: list[float] = []
        for h, dv in zip(self.models.ff.spec.past_horizons_s, f.dv_past[i], strict=True):
            k = i - round(h / dt)
            past.append(self.history[k] if k >= 0 else v0 - float(dv))
        out = outputs_from_points(self.models.ff, [f.t[i]], [v0], [future], [past])
        accel, brake = decide_openings(
            self.p, self.cfg, f.v0_raw[i : i + 1], f.near[i : i + 1], out.v0, out.a_req,
            np.maximum(0.0, out.accel_raw), np.maximum(0.0, out.brake_raw),
        )
        return float(accel[0]), float(brake[0])


def array_command(accel: np.ndarray, brake: np.ndarray):  # noqa: ANN201 - 内部の小さな閉包
    """あらかじめ計算した指令を simulate に渡すための関数。"""
    def command(i: int, _v: float) -> tuple[float, float]:
        return float(accel[i]), float(brake[i])
    return command


# ─────────────────────────────────────────────────────────────────────
# 段階 2: 評価
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class RunMetrics:
    key: str
    scale: float
    end_s: float
    aborted: bool
    max_abs_kmh: float
    p95_kmh: float
    reversals: int
    effective: dict[str, float]
    in_deadband_share: float
    at_deadband_share: float  # 指令が不感帯ちょうど（切り上げの結果。効き目は不感帯内と同じ）
    switches: int
    accel_max: float
    brake_max: float


def run_metrics(
    key: str, scale: float, run: SimRun, frames: RefFrames, p: FeedforwardParams,
    cfg: ResearchConfig,
) -> RunMetrics:
    n = len(run.t)
    dev = run.speed - frames.v0_raw[:n]
    step = max(1, round(0.1 / SIM_DT_S))
    kpi = compute_kpi(list(run.t[::step]), list(dev[::step]), cfg.kpi)
    cls = pedal_class(run.accel, run.brake, p)
    inside = in_deadband(run.accel, p.accel_deadband_pct) | in_deadband(
        run.brake, p.brake_deadband_pct
    )
    # 不感帯ちょうどの指令（C1 以降の切り上げの結果）。ペダルは動くが効かない。
    at_db = np.isclose(run.accel, p.accel_deadband_pct) | np.isclose(
        run.brake, p.brake_deadband_pct
    )
    return RunMetrics(
        key=key, scale=scale, end_s=run.end_s, aborted=run.stopped_at_s is not None,
        max_abs_kmh=kpi.max_abs_kmh, p95_kmh=kpi.p95_kmh,
        reversals=kpi.reversal_max_per_window,
        effective={k: float(np.mean(cls == k)) for k in (PEDAL_ACCEL, PEDAL_BRAKE, PEDAL_COAST)},
        in_deadband_share=float(np.mean(inside)),
        at_deadband_share=float(np.mean(at_db)),
        switches=switch_count(run.accel, run.brake),
        accel_max=float(run.accel.max()), brake_max=float(run.brake.max()),
    )


def summary_table(metrics: Sequence[RunMetrics]) -> str:
    body = [
        [CANDIDATE_NAMES[m.key],
         f"{m.end_s:.0f}s で中断" if m.aborted else "完走",
         f"{m.max_abs_kmh:.2f}", f"{m.p95_kmh:.2f}", str(m.reversals),
         f"{100 * m.effective[PEDAL_ACCEL]:.0f}%", f"{100 * m.effective[PEDAL_BRAKE]:.0f}%",
         f"{100 * m.effective[PEDAL_COAST]:.0f}%", f"{100 * m.in_deadband_share:.0f}%",
         f"{100 * m.at_deadband_share:.0f}%",
         str(m.switches), f"{m.accel_max:.1f}", f"{m.brake_max:.1f}"]
        for m in metrics
    ]
    return md_table(
        ["改善案", "走破", "最大逸脱[km/h]", "|偏差| p95[km/h]", "符号反転[回/5s]",
         "実効アクセル", "実効ブレーキ", "実質惰行", "不感帯内の指令", "不感帯ちょうどの指令",
         "切替[回]", "アクセル最大[%]", "ブレーキ最大[%]"],
        body,
    )


def _dev_cell(run: SimRun, frames: RefFrames, mask: np.ndarray) -> str:
    """そのグループでの 平均偏差 / |偏差| p95。中断後の時間は数えない。"""
    n = len(run.t)
    dev = np.full(len(frames.t), np.nan)
    dev[:n] = run.speed - frames.v0_raw[:n]
    d = dev[mask & np.isfinite(dev)]
    if d.size == 0:
        return "—"
    return f"{np.mean(d):+.2f} / {np.percentile(np.abs(d), 95):.2f}"


def group_dev_table(
    first: str, order: Sequence[str], group_keys: np.ndarray, runs: dict[str, SimRun],
    frames: RefFrames, keys: Sequence[str],
) -> str:
    body = []
    for name in order:
        m = group_keys == name
        if not np.any(m):
            continue
        body.append([name, f"{np.count_nonzero(m) * SIM_DT_S:.0f}",
                     *[_dev_cell(runs[k], frames, m) for k in keys]])
    return md_table([first, "時間[s]", *[f"{k} 平均偏差 / p95[km/h]" for k in keys]], body)


def state_table(
    runs: dict[str, SimRun], frames: RefFrames, states: np.ndarray, keys: Sequence[str]
) -> str:
    return group_dev_table("走行状態", STATES, states, runs, frames, keys)


def segment_table(
    runs: dict[str, SimRun], frames: RefFrames, segments: np.ndarray, cfg: ResearchConfig,
    keys: Sequence[str],
) -> str:
    return group_dev_table("区間", cfg.modes.segment_names, segments, runs, frames, keys)


def common_window_table(
    runs: dict[str, SimRun], frames: RefFrames, cfg: ResearchConfig, keys: Sequence[str]
) -> tuple[str, float]:
    """全案が走れた時間だけで KPI を比べる（中断の早い案が有利に見えるのを防ぐ）。"""
    end = min(runs[k].end_s for k in keys)
    n = int(round(end / SIM_DT_S)) + 1
    step = max(1, round(0.1 / SIM_DT_S))
    body = []
    for key in keys:
        run = runs[key]
        dev = run.speed[:n] - frames.v0_raw[:n]
        kpi = compute_kpi(list(run.t[:n:step]), list(dev[:n:step]), cfg.kpi)
        over = float(np.mean(np.abs(dev) > cfg.kpi.max_abs_deviation_kmh))
        body.append([CANDIDATE_NAMES[key], f"{kpi.max_abs_kmh:.2f}", f"{kpi.p95_kmh:.2f}",
                     str(kpi.reversal_max_per_window), f"{100 * over:.0f}%"])
    return md_table(
        ["改善案", "最大逸脱[km/h]", "|偏差| p95[km/h]", "符号反転[回/5s]",
         "|偏差| が 1 km/h を超えた割合"],
        body,
    ), end


def robust_table(metrics: Sequence[RunMetrics], keys: Sequence[str]) -> str:
    body = []
    for key in keys:
        row = [CANDIDATE_NAMES[key]]
        for scale in ROBUST_SCALES:
            m = next((x for x in metrics if x.key == key and x.scale == scale), None)
            row.append(
                f"{m.max_abs_kmh:.1f} / {m.p95_kmh:.1f}"
                + ("（中断）" if m.aborted else "") if m else "—"
            )
        body.append(row)
    return md_table(
        ["改善案", *[f"応答 ×{s:g}: 最大逸脱 / p95[km/h]" for s in ROBUST_SCALES]], body
    )


def training_table(
    current: tuple[tuple[int, int], tuple[float, float], tuple[float, float]],
    models: Sequence[TrainedModels],
) -> str:
    rows, mae, below = current
    body = [["C0 現行（dv_1.0 の符号で分割・ブレーキは不感帯未満を 0）",
             str(rows[0]), str(rows[1]), f"{mae[0]:.2f}", f"{mae[1]:.2f}",
             f"{100 * below[0]:.0f}%", f"{100 * below[1]:.0f}%"]]
    for m in models:
        body.append([f"{m.label}（効いている行だけ）", str(m.rows[0]), str(m.rows[1]),
                     f"{m.mae[0]:.2f}", f"{m.mae[1]:.2f}",
                     f"{100 * m.below_db[0]:.0f}%", f"{100 * m.below_db[1]:.0f}%"])
    return md_table(
        ["学習の作り方", "アクセル行数", "ブレーキ行数", "アクセル MAE[%]", "ブレーキ MAE[%]",
         "アクセル予測が不感帯未満", "ブレーキ予測が不感帯未満"],
        body,
    )


def fig_candidates(
    frames: RefFrames, runs: dict[str, SimRun], keys: Sequence[str], t0: float, t1: float,
    path: Path,
) -> None:
    plt = _plt()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
    m = (frames.t >= t0) & (frames.t <= t1)
    ax1.plot(frames.t[m], frames.v0_raw[m], color=COLOR_REF, linestyle="--", linewidth=1.4,
             label="基準車速")
    for key in keys:
        run = runs[key]
        ms = (run.t >= t0) & (run.t <= t1)
        ref_at = frames.v0_raw[: len(run.t)][ms]
        ax1.plot(run.t[ms], run.speed[ms], color=CANDIDATE_COLORS[key], linewidth=1.1, label=key)
        ax2.plot(run.t[ms], run.speed[ms] - ref_at, color=CANDIDATE_COLORS[key], linewidth=1.1,
                 label=key)
    ax1.set_ylabel("車速 [km/h]")
    ax2.set_ylabel("偏差 [km/h]")
    ax2.set_xlabel("モード経過時間 [s]")
    ax2.axhline(0.0, color="#555555", linewidth=0.8)
    for ax in (ax1, ax2):
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8, ncol=3)
    fig.suptitle(f"FF 改善案の閉ループ模擬（{t0:.0f}〜{t1:.0f}s）")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def past_direction_table(
    models: TrainedModels, p: FeedforwardParams, cfg: ResearchConfig, run_csv: Path
) -> str:
    """過去 2 列だけ実測に差し替えると指令がどちらへ動くか（手順 3 の実走行ログ上で測る）。

    逆モデルは「実現した軌跡を作った開度」を返すので、実測を入れれば追従が良くなるとは
    限らない。偏差の符号ごとに符号付きの変化を出して向きを確かめる。
    """
    rows = rows_from_csv(run_csv)
    ref = np.array([x.ref_kmh for x in rows])
    act = np.array([x.actual_kmh for x in rows])
    dt = float(np.median(np.diff([x.t_s for x in rows])))
    spec = models.ff.spec
    fo = [max(1, round(h / dt)) for h in spec.lookahead_horizons_s]
    po = [max(1, round(h / dt)) for h in spec.past_horizons_s]
    idx = np.arange(max(po), len(ref) - max(fo))
    v0 = ref[idx]
    future = [[float(ref[i + o]) for o in fo] for i in idx]
    near = np.array([f[0] for f in future])
    outs = [
        outputs_from_points(models.ff, [0.0] * len(idx), list(v0), future,
                            [[float(src[i - o]) for o in po] for i in idx])
        for src in (ref, act)
    ]
    cmds = [
        decide_openings(p, cfg, v0, near, o.v0, o.a_req,
                        np.maximum(0.0, o.accel_raw), np.maximum(0.0, o.brake_raw))
        for o in outs
    ]
    side = outs[0].a_req >= coast_accel_array(p, outs[0].v0)
    d_accel, d_brake = cmds[1][0] - cmds[0][0], cmds[1][1] - cmds[0][1]
    dev = act[idx] - ref[idx]
    body = []
    for label, sel in (
        ("遅れ（偏差 < −1 km/h）", dev < -1.0),
        ("ほぼ一致（|偏差| ≤ 1）", np.abs(dev) <= 1.0),
        ("出過ぎ（偏差 > +1）", dev > 1.0),
    ):
        ma, mb = sel & side, sel & ~side
        body.append([
            label, str(int(ma.sum())),
            f"{d_accel[ma].mean():+.2f}" if ma.any() else "—",
            str(int(mb.sum())), f"{d_brake[mb].mean():+.2f}" if mb.any() else "—",
        ])
    body.append([
        "全体（|Δ| 平均 / p95）", str(int(side.sum())),
        f"{np.abs(d_accel[side]).mean():.2f} / {np.percentile(np.abs(d_accel[side]), 95):.2f}",
        str(int((~side).sum())),
        f"{np.abs(d_brake[~side]).mean():.2f} / {np.percentile(np.abs(d_brake[~side]), 95):.2f}",
    ])
    return md_table(
        ["偏差の区分", "アクセル側[行]", "Δアクセル開度[%]", "ブレーキ側[行]",
         "Δブレーキ開度[%]"],
        body,
    )


def opening_vs_speed_table(models: TrainedModels) -> str:
    """アクセル予測開度が v0 のどこで最大になるか（C4 の効く向きが反転する理由）。"""
    spec = models.ff.spec
    speeds = np.arange(10.0, 135.0, 5.0)
    shown = (25.0, 55.0, 85.0, 115.0, 130.0)
    body = []
    for a_req in (1.0, 0.0, -1.0):
        future = [[float(v + a_req * h) for h in spec.lookahead_horizons_s] for v in speeds]
        past = [[float(v - a_req * h) for h in spec.past_horizons_s] for v in speeds]
        out = outputs_from_points(models.ff, [0.0] * len(speeds), list(speeds), future, past)
        a = np.maximum(0.0, out.accel_raw)
        body.append([
            f"{a_req:+.1f}", f"{speeds[int(np.argmax(a))]:.0f}", f"{a.max():.2f}",
            *[f"{a[int(np.argmin(np.abs(speeds - v)))]:.1f}" for v in shown],
        ])
    return md_table(
        ["a_req[km/h/s]", "開度が最大になる v0[km/h]", "そのときの開度[%]",
         *[f"v0 {v:.0f}[%]" for v in shown]],
        body,
    )


@dataclass
class StressCommand:
    """C5 の指令に、実機で効いてくる粗さを被せる（車速の量子化・指令のレート制限）。"""

    inner: ReplanCommand
    quant: float
    rate_pct_s: float
    state: list[float]

    def __call__(self, i: int, v: float) -> tuple[float, float]:
        v_seen = round(v / self.quant) * self.quant if self.quant > 0.0 else v
        want = self.inner(i, v_seen)
        if self.rate_pct_s <= 0.0:
            return want
        step = self.rate_pct_s * SIM_DT_S
        for k in (0, 1):
            self.state[k] += float(np.clip(want[k] - self.state[k], -step, step))
        return self.state[0], self.state[1]


# C5 の頑健性を見る条件: (表示名, むだ時間[s], 車速の量子化[km/h], レート制限[%/s])
C5_STRESS = (
    ("そのまま（S-2 と同じ）", 0.0, 0.0, 0.0),
    ("むだ時間 0.2s", 0.2, 0.0, 0.0),
    ("車速を 0.5 km/h に量子化", 0.0, 0.5, 0.0),
    ("指令のレート制限 20%/s", 0.0, 0.0, 20.0),
    ("むだ時間 0.2s＋レート制限 20%/s", 0.2, 0.0, 20.0),
)


def _command_motion(run: SimRun) -> np.ndarray:
    """1 周期あたりの指令の動き [%/s]（アクセルとブレーキの変化の合計）。"""
    return (np.abs(np.diff(run.accel)) + np.abs(np.diff(run.brake))) / SIM_DT_S


def chatter_table(runs: dict[str, SimRun], keys: Sequence[str]) -> str:
    """指令がどれだけ細かく動くか（実機での振動の目安）。"""
    body = []
    for key in keys:
        run = runs[key]
        d = _command_motion(run)
        switches = switch_count(run.accel, run.brake)
        body.append([
            CANDIDATE_NAMES[key], f"{run.end_s:.0f}", str(switches),
            f"{switches / (run.end_s / 60.0):.1f}",
            f"{100 * np.mean(d > 1e-9):.0f}%",
            f"{np.percentile(d, 50):.1f} / {np.percentile(d, 95):.1f}",
        ])
    return md_table(
        ["改善案", "走った時間[s]", "ペダル切替[回]", "切替[回/分]", "指令が動いた周期",
         "指令の動き p50 / p95[%/s]"],
        body,
    )


def c5_stress_table(
    models: TrainedModels, p: FeedforwardParams, cfg: ResearchConfig, vehicle: VehicleModel,
    frames: RefFrames, n_steps: int,
) -> str:
    """C5 の模擬が理想的すぎないか（遅れ・車速の粗さ・レート制限を入れて確かめる）。"""
    body = []
    for label, delay_s, quant, rate in C5_STRESS:
        cmd = StressCommand(ReplanCommand(models, p, cfg, frames, []), quant, rate, [0.0, 0.0])
        vm = vehicle.with_delay(delay_s, 0.0) if delay_s > 0.0 else vehicle
        run = simulate(vm, n_steps, cmd, dt=SIM_DT_S, stop_above_kmh=cfg.vehicle.max_speed_kmh)
        m = run_metrics(C5, 1.0, run, frames, p, cfg)
        body.append([
            label, f"{m.end_s:.0f}s で中断" if m.aborted else "完走",
            f"{m.max_abs_kmh:.2f}", f"{m.p95_kmh:.2f}", str(m.switches),
            f"{np.percentile(_command_motion(run), 95):.1f}",
        ])
    return md_table(
        ["条件", "走破", "最大逸脱[km/h]", "|偏差| p95[km/h]", "ペダル切替[回]",
         "指令の動き p95[%/s]"],
        body,
    )


def run_compare(cfg: ResearchConfig, run_csv: Path, train_csv: Path, out_dir: Path) -> int:
    p = feedforward_params(cfg)
    run_log = read_log(run_csv, SECTION_MODE_DRIVE, LOG_RUN)
    train_log = read_log(train_csv, SECTION_PATTERN_DRIVE, LOG_TRAIN)
    accel_resp, _ = identify_response("アクセル", [run_log, train_log], p, is_accel=True)
    brake_resp, _ = identify_response("ブレーキ", [run_log, train_log], p, is_accel=False)

    mode = asyncio.run(load_mode(cfg, cfg.modes.wltp_mode_name))
    ref = ReferenceSpeed(mode)
    t = np.round(np.arange(0.0, mode.total_duration + SIM_DT_S / 2, SIM_DT_S), 4)
    frames = ref_frames(ref, t, DEFAULT_FEATURE_SPEC)
    frames_shift = ref_frames(ref, t, DEFAULT_FEATURE_SPEC, SHIFT_S)
    print(f"# 段階 2 FF 改善案の比較（{mode.name} {mode.total_duration:.0f}s・"
          f"{SIM_DT_S:g}s 刻み {len(t)} 点、車両モデル: 表（手順 2＋3）・むだ時間 0s）")

    current = load_ff_model(cfg.feedforward.model_path)
    base = train_models(train_csv, p, label="C1・C2・C4")
    shifted = train_models(train_csv, p, shift_s=SHIFT_S, label=f"C3（{SHIFT_S:g}s ずらし）")
    print("\n### S-1 学習の中身（学習データは手順 2 の同じ CSV）")
    print(training_table(current_model_stats(train_csv, p, current), [base, shifted]))

    out0 = outputs_for_mode(current, ref, t)
    series = {
        C0: ff_openings(decide_all(p, out0).effort, cfg.vehicle.max_accel_opening_pct,
                        cfg.vehicle.max_brake_opening_pct),
        C1: candidate_openings(C1, base, p, cfg, frames),
        C2: candidate_openings(C2, base, p, cfg, frames),
        C3: candidate_openings(C3, shifted, p, cfg, frames_shift),
    }
    keys = (C0, C1, C2, C3, C4, C5)

    metrics: list[RunMetrics] = []
    runs_at_1: dict[str, SimRun] = {}
    for scale in ROBUST_SCALES:
        vehicle = VehicleModel(
            f"表×{scale:g}", p, scale_response(accel_resp, scale), scale_response(brake_resp, scale)
        )
        for key in keys:
            if key == C4:  # 実車速を見る案は毎回作り直す（C5 は履歴を持つため）
                command = ActualSpeedCommand(base, p, cfg, frames)
            elif key == C5:
                command = ReplanCommand(base, p, cfg, frames, [])
            else:
                command = array_command(*series[key])
            run = simulate(vehicle, len(t), command, dt=SIM_DT_S,
                           stop_above_kmh=cfg.vehicle.max_speed_kmh)
            metrics.append(run_metrics(key, scale, run, frames, p, cfg))
            if scale == 1.0:
                runs_at_1[key] = run

    at_one = [m for m in metrics if m.scale == 1.0]
    print("\n### S-2 WLTP 1800s の閉ループ模擬（ペダル応答は実測の表）")
    print(summary_table(at_one))
    common, end = common_window_table(runs_at_1, frames, cfg, keys)
    print(f"\n### S-3 全案が走れた 0〜{end:.0f}s だけで比べた KPI")
    print(common)
    rows = [ModeRow(t_s=float(x), ref_kmh=float(r), actual_kmh=0.0, accel_pct=0.0, brake_pct=0.0,
                    ff_effort_pct=0.0, pid_effort_pct=0.0, effort_pct=0.0, segment="", phase="")
            for x, r in zip(t, frames.v0_raw, strict=True)]
    states = np.array(driving_states(rows))
    segments = np.array([cfg.modes.segment_at(float(x)) for x in t])
    print("\n### S-4 WLTP 区間ごとの平均偏差 / |偏差| p95（中断後の時間は数えない）")
    print(segment_table(runs_at_1, frames, segments, cfg, keys))
    print("\n### S-5 走行状態ごとの平均偏差 / |偏差| p95（同）")
    print(state_table(runs_at_1, frames, states, keys))
    print("\n### S-6 ペダル応答が外れていた場合（表を ×0.8 / ×1.2 した車両モデル）")
    print(robust_table(metrics, keys))
    print(f"\n### S-7 過去 2 列だけ実測にしたときの指令の変化（{LOG_RUN} の実走行ログ上）")
    print(past_direction_table(base, p, cfg, run_csv))
    print("\n### S-8 アクセル予測開度は v0 のどこで最大か（C4 の向きが反転する理由）")
    print(opening_vs_speed_table(base))
    print("\n### S-9 指令の細かさ（実機での振動の目安）")
    print(chatter_table(runs_at_1, keys))
    print("\n### S-10 C5 に遅れ・車速の粗さ・指令のレート制限を入れたとき")
    print(c5_stress_table(base, p, cfg, VehicleModel("表", p, accel_resp, brake_resp),
                          frames, len(t)))

    out_dir.mkdir(parents=True, exist_ok=True)
    fig_candidates(frames, runs_at_1, keys, 0.0, float(t[-1]),
                   out_dir / "compare_overview.png")
    for t0, t1 in COMPARE_ZOOMS_S:
        fig_candidates(frames, runs_at_1, keys, t0, t1,
                       out_dir / f"compare_{t0:.0f}_{t1:.0f}.png")
    print(f"\n- 図: {out_dir}")
    return 0


# ─────────────────────────────────────────────────────────────────────
# --part sim-check
# ─────────────────────────────────────────────────────────────────────


def run_sim_check(cfg: ResearchConfig, run_csv: Path, train_csv: Path, out_dir: Path) -> int:
    p = feedforward_params(cfg)
    run = read_log(run_csv, SECTION_MODE_DRIVE, LOG_RUN)
    train = read_log(train_csv, SECTION_PATTERN_DRIVE, LOG_TRAIN)
    models = build_models(p, run, train)
    print(f"# 段階 1 簡易車両モデルの確認（{LOG_RUN}: `{run_csv}` {len(run.t)} 行、"
          f"{LOG_TRAIN}: `{train_csv}` {len(train.t)} 行）")

    print("\n### K-1 ペダル応答: 不感帯を超えた開度 → 惰行からの加速度の変化 [km/h/s]"
          "（表の値。括弧内は config ゲイン比例）")
    for is_accel, key, label in ((True, "accel", "アクセル"), (False, "brake", "ブレーキ")):
        print(f"\n#### {label}（不感帯 "
              f"{p.accel_deadband_pct if is_accel else p.brake_deadband_pct:g}%）")
        print(response_table(models, is_accel=is_accel))
        print(f"\n{label}: 同定に使った行数（手順 2＋3、帯ごと。{8} 行未満の帯は使わない）")
        print(counts_table(models.counts[key]))

    logs = (run, train)
    grid = replay_grid(models.all, logs)
    print(f"\n### K-2 開ループ再生: 指令開度だけで {REPLAY_HORIZON_S:g}s 走らせた車速の誤差"
          f"（{REPLAY_EVERY_S:g}s ごとに実車速から再開、停車中の窓は除く）")
    print(replay_table(grid, [lg.name for lg in logs]))
    chosen = [best_row(grid, m.name, LOG_RUN) for m in models.all]
    print(f"\n{LOG_RUN} のログで RMSE 最小の組:")
    for r in chosen:
        print(f"- {r.model}: むだ時間 {r.delay_s:g}s・一次遅れ {r.lag_s:g}s"
              f" → RMSE {r.rmse:.2f} km/h")
        print(replay_windows_table(r))
        other = [x for x in grid if (x.model, x.delay_s, x.lag_s, x.log)
                 == (r.model, r.delay_s, r.lag_s, LOG_TRAIN)][0]
        print(f"  同じ組の {LOG_TRAIN} のログ: RMSE {other.rmse:.2f} km/h")

    print(f"\n### K-3 閉ループ再現: 手順 3 の FF 指令で WLTP を模擬（{SIM_DT_S:g}s 刻み、"
          f"{cfg.vehicle.max_speed_kmh:g} km/h 超で打ち切り）と実走行の比較")
    cmd = step3_commands(cfg, p)
    sims: list[tuple[str, SimRun]] = []
    for r in chosen:
        vm = next(m for m in models.all if m.name == r.model).with_delay(r.delay_s, r.lag_s)
        sims.append((f"{r.model}・{r.delay_s:g}s/{r.lag_s:g}s",
                     run_closed_loop(vm, cmd, cfg.vehicle.max_speed_kmh)))

    rows = rows_from_csv(run_csv)
    real_t = np.array([x.t_s for x in rows])
    real_ref = np.array([x.ref_kmh for x in rows])
    real_speed = np.array([x.actual_kmh for x in rows])
    real_dev = real_speed - real_ref
    sim_devs = []
    for _, s in sims:
        dev = np.interp(real_t, s.t, s.speed) - real_ref
        dev[real_t > s.end_s] = np.nan
        sim_devs.append(dev)
    labels = [lb for lb, _ in sims]
    dt_real = float(np.median(np.diff(real_t)))

    body = [["実走行（手順 3）", "—" if rows[-1].t_s >= cmd.t[-1] else f"{real_t[-1]:.1f}",
             f"{np.nanmax(np.abs(real_dev)):.2f}", f"{np.percentile(np.abs(real_dev), 95):.2f}",
             "—", "—"]]
    for (label, s), dev in zip(sims, sim_devs, strict=True):
        step = max(1, round(0.1 / SIM_DT_S))
        t10, v10 = s.t[::step], s.speed[::step]
        kpi = compute_kpi(list(t10), list(v10 - np.interp(t10, cmd.t, cmd.ref)), cfg.kpi)
        both = np.isfinite(dev)
        body.append([
            f"模擬: {label}",
            f"{s.stopped_at_s:.1f}" if s.stopped_at_s is not None else "完走",
            f"{kpi.max_abs_kmh:.2f}", f"{kpi.p95_kmh:.2f}",
            f"{np.sqrt(np.mean((dev[both] - real_dev[both]) ** 2)):.2f}",
            f"{np.corrcoef(dev[both], real_dev[both])[0, 1]:.2f}",
        ])
    print(md_table(["", "中断時刻[s]", "最大逸脱[km/h]", "|偏差| p95[km/h]",
                    "実走行との車速差 RMS[km/h]（実走行の区間）", "偏差の相関（同）"], body))

    segments = np.array([cfg.modes.segment_at(float(x)) for x in real_t])
    print("\n#### 区間別（実走行が走った 0〜926s）")
    print(group_table("区間", deviation_by_group(
        segments, cfg.modes.segment_names, real_dev, sim_devs, dt_real), labels))
    states = np.array(driving_states(rows))
    print("\n#### 走行状態別（同）")
    print(group_table("走行状態", deviation_by_group(
        states, STATES, real_dev, sim_devs, dt_real), labels))

    out_dir.mkdir(parents=True, exist_ok=True)
    end = float(real_t[-1])
    fig_overlay(real_t, real_ref, real_speed, sims, cmd, 0.0, end,
                out_dir / "simcheck_overview.png")
    for t0, t1 in ZOOMS_S:
        fig_overlay(real_t, real_ref, real_speed, sims, cmd, t0, t1,
                    out_dir / f"simcheck_{t0:.0f}_{t1:.0f}.png")
    print(f"\n- 図: {out_dir}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--part", choices=("sim-check", "compare"), required=True)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--run-csv", type=Path, default=DEFAULT_RUN_CSV,
                    help="手順 3 の実走行 CSV")
    ap.add_argument("--train-csv", type=Path, default=DEFAULT_TRAIN_CSV,
                    help="手順 2 のパターン走行 CSV")
    ap.add_argument("--out", type=Path, default=None,
                    help="図の保存先（既定: results/report<今日>_KAIZEN_process2,3/）")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    out_dir = args.out or cfg.results_path / f"report{datetime.now():%Y%m%d}_KAIZEN_process2,3"
    if args.part == "compare":
        return run_compare(cfg, args.run_csv, args.train_csv, out_dir)
    return run_sim_check(cfg, args.run_csv, args.train_csv, out_dir)


if __name__ == "__main__":
    raise SystemExit(main())
