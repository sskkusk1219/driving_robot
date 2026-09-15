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

    --part coverage   段階 3: 手順 2 のパターン走行が WLTP に要る領域を覆えているか
        V-1 速度帯 × 開度で「WLTP が要る時間」と「学習の効く行数」を並べる
        V-2 開度ごとの分布（速度をまとめたもの）
        V-3 パターンごとの実績（所要時間・開度・開始車速・効く行数）
        V-4 フェーズごとの所要時間（打ち切りに張り付いていないか）
        V-5 現行パターンの欠陥（V-3・V-4 の実測を根拠にする）
        V-6 パターン案と所要時間（実測の単位時間から積算。車両モデルでは模擬しない）
        V-7 系統を 1 つ抜いて学習し直したときの WLTP 指令開度の変化
        V-8 実開度 PNOW（0x9000）を毎周期の読み取りに相乗りさせる案のフレーム長
        要る開度は基準車速から物理式（惰行カーブ＋ペダルゲイン）で逆算する。学習した
        モデルの予測は使わない（学習データの中身に結論を依存させないため）。
"""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from src.domain.control.pedal_plan import analytic_efforts
from src.domain.learning_drive import (
    ACCEL_DEADBAND_PROBE_HOLD_S,
    ACCEL_SWEEP_FRACS,
    CRUISE_TRIM_HOLD_S,
    HOLD_DURATION_S,
)
from src.domain.model_training import (
    DEFAULT_FEATURE_SPEC,
    STOP_SPEED_KMH,
    FeatureSpec,
    _build_feature_matrix,
    _estimate_offsets,
    _make_estimator,
)
from src.models.profile import FeedforwardParams
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
from tests.research.pattern_loop import PatternLoopConfig
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
    x, idx = _build_feature_matrix(
        speed,
        _estimate_offsets(timestamps, spec.lookahead_horizons_s),
        _estimate_offsets(timestamps, spec.past_horizons_s),
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
    accel_model = _make_estimator().fit(x[a_mask], a_lab[a_mask])
    brake_model = _make_estimator().fit(x[b_mask], b_lab[b_mask])
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
# --part coverage（段階 3: 手順 2 パターン走行の網羅性と所要時間）
# ─────────────────────────────────────────────────────────────────────

COVER_SPEED_EDGES_KMH = (0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0)
COVER_OPEN_EDGES_PCT = (10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0)
SPEED_BAND_LABELS = [
    f"{COVER_SPEED_EDGES_KMH[i]:g}〜{COVER_SPEED_EDGES_KMH[i + 1]:g}"
    for i in range(len(COVER_SPEED_EDGES_KMH) - 1)
]
OPEN_BIN_LABELS = [
    *(f"{COVER_OPEN_EDGES_PCT[i]:g}〜{COVER_OPEN_EDGES_PCT[i + 1]:g}"
      for i in range(len(COVER_OPEN_EDGES_PCT) - 1)),
    f"{COVER_OPEN_EDGES_PCT[-1]:g}〜",
]
# 学習の系統（CSV の pattern 列 "12:ACCEL_SWEEP" の ":" より後ろ）
PATTERN_KINDS = (
    "ACCEL_SWEEP", "BRAKE_HOLD", "COAST_DOWN", "CRUISE_TRIM", "ACCEL_DEADBAND_PROBE",
    "CREEP", "CREEP_SETTLE",
)
# フェーズごとの上限（打ち切り時間、または設計上の保持時間）。
# 打ち切りは tests/research/pattern_loop.PatternLoopConfig、
# 設計上の保持は src/domain/learning_drive の既定値。
PHASE_LIMIT_S = {
    ("", "DRIVE_ACCEL"): PatternLoopConfig().accel_full_range_timeout_s,
    ("", "DRIVE_BRAKE"): PatternLoopConfig().brake_stop_timeout_s,
    ("", "BRAKE_HOLD"): PatternLoopConfig().brake_hold_timeout_s,
    ("", "COAST"): PatternLoopConfig().coast_timeout_s,
    ("", "CRUISE_TRIM"): CRUISE_TRIM_HOLD_S,
    ("CREEP", "MEASURE"): HOLD_DURATION_S,
    ("CREEP_SETTLE", "MEASURE"): PatternLoopConfig().creep_settle_timeout_s,
    ("ACCEL_DEADBAND_PROBE", "MEASURE"): ACCEL_DEADBAND_PROBE_HOLD_S,
}
AT_LIMIT_TOL_S = 0.3  # これだけ下回っていなければ「上限に張り付いた」と数える
HOLE_NEED_S = 10.0  # 「空白」と呼ぶ下限: WLTP がこれ以上の時間を要求している
HOLE_ROWS = 50  # かつ学習の効く行がこれ未満（0.1s 刻みなので 50 行 = 5s ぶん）
MODBUS_BYTE_S = 10.0 / 38400.0  # 8N1・38400bps で 1 バイトあたりの時間 [s]
CONTROL_PERIOD_S = 0.05  # 制御周期（src/domain/control/drive_loop.CONTROL_LOOP_INTERVAL_S）


def _band_index(values: np.ndarray, edges: Sequence[float]) -> np.ndarray:
    """速度帯の番号（最後の辺以上は最後の帯に入れる）。"""
    e = np.asarray(edges, dtype=float)
    return np.clip(np.searchsorted(e, values, side="right") - 1, 0, len(e) - 2)


def _open_index(values: np.ndarray, edges: Sequence[float]) -> np.ndarray:
    """開度ビンの番号（edges[0] 未満と NaN は -1、最後の辺以上は最後のビン）。"""
    e = np.asarray(edges, dtype=float)
    idx = np.searchsorted(e, values, side="right") - 1
    return np.where(np.isfinite(values) & (values >= e[0]), idx, -1)


@dataclass(frozen=True)
class Demand:
    """WLTP が要る「ペダル」と「開度」。"""

    pedal: np.ndarray
    opening: np.ndarray  # 要る開度 [%]（惰行・停車・クリープの周期は NaN）


def wltp_demand(p: FeedforwardParams, frames: RefFrames) -> Demand:
    """基準車速だけから「どのペダルが何 % 要るか」を物理式で逆算する。

    Ridge の予測は使わない（学習データの中身に依存させないため）。`analytic_efforts` は
    要求加速度が惰行より上ならアクセル・下ならブレーキと分けるので、`decide_openings`
    （C1〜C5 のペダル選択）と同じ基準になる。停車保持とクリープ任せの周期は数えない。
    """
    eff = analytic_efforts(frames.v0_raw, frames.a_req, p)
    stop = (frames.v0_raw <= STOP_SPEED_KMH) & (frames.near <= STOP_SPEED_KMH)
    creep = (
        (~stop) & (frames.a_req >= 0.0) & (frames.v0_raw < p.creep_speed_kmh)
        & (frames.a_req <= p.creep_rate_kmhs)
    )
    idle = stop | creep | ~np.isfinite(eff) | (eff == 0.0)
    pedal = np.where(idle, PEDAL_COAST, np.where(eff > 0.0, PEDAL_ACCEL, PEDAL_BRAKE))
    return Demand(pedal=pedal, opening=np.where(idle, np.nan, np.abs(eff)))


@dataclass(frozen=True)
class EffectiveRows:
    """C1〜C5 が学習に使う行（そのペダルの開度が不感帯以上）。"""

    speed: np.ndarray
    opening: np.ndarray
    kind: np.ndarray


def effective_rows(train_csv: Path, p: FeedforwardParams) -> tuple[EffectiveRows, EffectiveRows]:
    """手順 2 のログから、アクセル / ブレーキが効いている行を取り出す。"""
    logs, patterns, _ = load_training_rows(train_csv)
    kinds = np.array([s.split(":", 1)[-1] for s in patterns])
    speed = np.clip(np.array([lg.actual_speed_kmh for lg in logs], dtype=float), 0.0, None)
    accel = np.array([lg.accel_opening for lg in logs], dtype=float)
    brake = np.array([lg.brake_opening for lg in logs], dtype=float)
    ma, mb = accel >= p.accel_deadband_pct, brake >= p.brake_deadband_pct
    return (
        EffectiveRows(speed[ma], accel[ma], kinds[ma]),
        EffectiveRows(speed[mb], brake[mb], kinds[mb]),
    )


def coverage_table(demand: Demand, frames: RefFrames, rows: EffectiveRows, pedal: str) -> str:
    """速度帯 × 開度ビンで「WLTP が要る時間[s]」と「学習の効く行数」を並べる。

    太字は空白のセル: 要る時間が 1s 以上あって学習の行が 0、または要る時間が
    HOLE_NEED_S 以上あって学習の行が HOLE_ROWS 未満。
    """
    grid = coverage_grid(demand, frames, rows, pedal)
    body = []
    for i in range(len(SPEED_BAND_LABELS)):
        cells = []
        for j in range(len(OPEN_BIN_LABELS)):
            sec, cnt = grid[i][j]
            if sec < 1.0:
                cells.append("—" if cnt == 0 else f"要らない / {cnt}")
            elif _is_hole(sec, cnt):
                cells.append(f"**{sec:.0f}s / {cnt}**")
            else:
                cells.append(f"{sec:.0f}s / {cnt}")
        body.append([SPEED_BAND_LABELS[i], *cells])
    return md_table(["速度帯[km/h] \\ 開度[%]", *OPEN_BIN_LABELS], body)


def _is_hole(sec: float, rows: int) -> bool:
    """そのセルが「要るのに学習データが無い（少ない）」か。"""
    return (sec >= 1.0 and rows == 0) or (sec >= HOLE_NEED_S and rows < HOLE_ROWS)


def coverage_grid(
    demand: Demand, frames: RefFrames, rows: EffectiveRows, pedal: str
) -> list[list[tuple[float, int]]]:
    """速度帯 × 開度ビンの (WLTP が要る時間[s], 学習の効く行数)。"""
    m = demand.pedal == pedal
    sb = _band_index(frames.v0_raw[m], COVER_SPEED_EDGES_KMH)
    ob = _open_index(demand.opening[m], COVER_OPEN_EDGES_PCT)
    rb = _band_index(rows.speed, COVER_SPEED_EDGES_KMH)
    ro = _open_index(rows.opening, COVER_OPEN_EDGES_PCT)
    return [
        [(SIM_DT_S * int(np.sum((sb == i) & (ob == j))), int(np.sum((rb == i) & (ro == j))))
         for j in range(len(OPEN_BIN_LABELS))]
        for i in range(len(SPEED_BAND_LABELS))
    ]


def worst_holes(
    demand: Demand, frames: RefFrames, rows: EffectiveRows, pedal: str, top: int = 3
) -> str:
    """要る時間が長いのに学習データが少ないセルを、要る時間の多い順に並べた文字列。"""
    grid = coverage_grid(demand, frames, rows, pedal)
    found = [
        (sec, cnt, i, j)
        for i, row in enumerate(grid) for j, (sec, cnt) in enumerate(row) if _is_hole(sec, cnt)
    ]
    found.sort(reverse=True)
    text = " / ".join(
        f"{SPEED_BAND_LABELS[i]} km/h × {OPEN_BIN_LABELS[j]}% は {sec:.0f}s 要るのに {cnt} 行"
        for sec, cnt, i, j in found[:top]
    )
    return f"{text}（空白のセルは全部で {len(found)} 個）" if found else "なし"


def opening_hist_table(demand: Demand, rows_a: EffectiveRows, rows_b: EffectiveRows) -> str:
    """開度ビンごとの「WLTP が要る時間」と「学習の効く行数」（速度をまとめた分布）。"""
    pairs = ((PEDAL_ACCEL, rows_a), (PEDAL_BRAKE, rows_b))
    body = []
    for j, label in enumerate(OPEN_BIN_LABELS):
        cells: list[str] = [label]
        for pedal, rows in pairs:
            m = demand.pedal == pedal
            sec = SIM_DT_S * int(
                np.sum(_open_index(demand.opening[m], COVER_OPEN_EDGES_PCT) == j)
            )
            cnt = int(np.sum(_open_index(rows.opening, COVER_OPEN_EDGES_PCT) == j))
            cells += [f"{sec:.0f}" if sec >= 0.5 else "—", str(cnt) if cnt > 0 else "—"]
        body.append(cells)
    totals: list[str] = ["**合計**"]
    for pedal, rows in pairs:
        m = demand.pedal == pedal
        totals += [f"**{SIM_DT_S * int(np.sum(m)):.0f}**", f"**{len(rows.opening)}**"]
    body.append(totals)
    return md_table(
        ["開度[%]", "アクセル 要る時間[s]", "アクセル 学習行",
         "ブレーキ 要る時間[s]", "ブレーキ 学習行"],
        body,
    )


@dataclass(frozen=True)
class PatternStat:
    """手順 2 のログで、1 パターンが実際にどう走ったか。"""

    name: str
    kind: str
    rows: int
    duration_s: float
    max_accel: float
    max_brake: float
    v_start: float
    v_max: float
    eff_accel: int
    eff_brake: int
    phases: str


def pattern_stats(train_csv: Path, p: FeedforwardParams) -> tuple[list[PatternStat], float]:
    """CSV の pattern 列ごとに、所要時間・開度・車速・効く行数をまとめる。

    2 番目の返り値はパターン走行の全体時間（最初の行から最後の行まで）。パターンごとの
    所要時間の合計より数秒長い（パターンの切れ目に指令の送り直しが入るため）。
    """
    logs, patterns, phases = load_training_rows(train_csv)
    out: list[PatternStat] = []
    start = 0
    for i in range(1, len(patterns) + 1):
        if i < len(patterns) and patterns[i] == patterns[start]:
            continue
        block = logs[start:i]
        ph = phases[start:i]
        speed = np.clip(np.array([lg.actual_speed_kmh for lg in block], dtype=float), 0.0, None)
        accel = np.array([lg.accel_opening for lg in block], dtype=float)
        brake = np.array([lg.brake_opening for lg in block], dtype=float)
        seen: list[str] = []
        for name in ph:
            if not seen or seen[-1] != name:
                seen.append(str(name))
        out.append(PatternStat(
            name=str(patterns[start]),
            kind=str(patterns[start]).split(":", 1)[-1],
            rows=len(block),
            duration_s=(block[-1].timestamp - block[0].timestamp).total_seconds(),
            max_accel=float(accel.max()),
            max_brake=float(brake.max()),
            v_start=float(speed[0]),
            v_max=float(speed.max()),
            eff_accel=int(np.sum(accel >= p.accel_deadband_pct)),
            eff_brake=int(np.sum(brake >= p.brake_deadband_pct)),
            phases="→".join(seen),
        ))
        start = i
    span = (logs[-1].timestamp - logs[0].timestamp).total_seconds()
    return out, span


def pattern_table(stats: Sequence[PatternStat], span_s: float) -> str:
    body = [[
        s.name, str(s.rows), f"{s.duration_s:.1f}", f"{s.max_accel:.2f}", f"{s.max_brake:.2f}",
        f"{s.v_start:.1f}", f"{s.v_max:.1f}", str(s.eff_accel), str(s.eff_brake), s.phases,
    ] for s in stats]
    body.append(["**合計**", f"**{sum(s.rows for s in stats)}**",
                 f"**{span_s:.1f}**（パターンごとの和は {sum(s.duration_s for s in stats):.1f}）",
                 "—", "—", "—", "—", f"**{sum(s.eff_accel for s in stats)}**",
                 f"**{sum(s.eff_brake for s in stats)}**", "—"])
    return md_table(
        ["パターン", "行数", "所要[s]", "最大アクセル[%]", "最大ブレーキ[%]", "開始車速[km/h]",
         "到達最高速[km/h]", "効くアクセル行", "効くブレーキ行", "フェーズ"],
        body,
    )


@dataclass(frozen=True)
class PhaseStat:
    """系統 × フェーズの連続ブロックの所要時間（打ち切りに張り付いていないか）。"""

    kind: str
    phase: str
    count: int
    median_s: float
    min_s: float
    max_s: float
    limit_s: float
    at_limit: int


def phase_stats(train_csv: Path) -> list[PhaseStat]:
    logs, patterns, phases = load_training_rows(train_csv)
    blocks: dict[tuple[str, str], list[float]] = {}
    start = 0
    for i in range(1, len(patterns) + 1):
        same = i < len(patterns) and patterns[i] == patterns[start] and phases[i] == phases[start]
        if same:
            continue
        key = (str(patterns[start]).split(":", 1)[-1], str(phases[start]))
        dur = (logs[i - 1].timestamp - logs[start].timestamp).total_seconds()
        blocks.setdefault(key, []).append(dur)
        start = i
    out: list[PhaseStat] = []
    for (kind, phase), durs in sorted(blocks.items()):
        limit = PHASE_LIMIT_S.get((kind, phase), PHASE_LIMIT_S.get(("", phase), float("nan")))
        at_limit = int(sum(1 for d in durs if np.isfinite(limit) and d >= limit - AT_LIMIT_TOL_S))
        out.append(PhaseStat(
            kind=kind, phase=phase, count=len(durs), median_s=float(np.median(durs)),
            min_s=min(durs), max_s=max(durs), limit_s=limit, at_limit=at_limit,
        ))
    return out


def phase_table(stats: Sequence[PhaseStat]) -> str:
    body = [[
        s.kind, s.phase, str(s.count), f"{s.median_s:.1f}", f"{s.min_s:.1f}", f"{s.max_s:.1f}",
        f"{s.limit_s:.0f}" if np.isfinite(s.limit_s) else "—",
        f"**{s.at_limit} / {s.count}**" if s.at_limit == s.count else f"{s.at_limit} / {s.count}",
    ] for s in stats]
    return md_table(
        ["系統", "フェーズ", "本数", "所要 中央値[s]", "最短[s]", "最長[s]",
         "上限[s]（打ち切り / 設計の保持）", "上限に張り付いた本数"],
        body,
    )


def defect_table(
    stats: Sequence[PatternStat], phases: Sequence[PhaseStat], demand: Demand, frames: RefFrames,
    rows_a: EffectiveRows, rows_b: EffectiveRows, cfg: ResearchConfig,
) -> str:
    """現行パターンの欠陥（症状・根拠の数値・直し方）。数値はすべて上の表から取る。"""
    sweep = [s for s in stats if s.kind == "ACCEL_SWEEP"]
    hold = [s for s in stats if s.kind == "BRAKE_HOLD"]
    coast = [s for s in stats if s.kind == "COAST_DOWN"]
    trim = [s for s in stats if s.kind == "CRUISE_TRIM"]
    probe = [s for s in stats if s.kind == "ACCEL_DEADBAND_PROBE"]
    cap = cfg.vehicle.max_speed_kmh * PatternLoopConfig().accel_speed_cap_frac
    accel_need_max = float(np.nanmax(demand.opening[demand.pedal == PEDAL_ACCEL]))
    over = int(np.sum(rows_a.opening > accel_need_max))
    starts = " / ".join(f"{s.v_start:.0f}" for s in sweep)
    hold_starts = " / ".join(f"{s.v_start:.0f}" for s in hold)
    ret = next((s for s in phases if s.phase == "DRIVE_BRAKE"), None)
    acc = next((s for s in phases if s.kind == "BRAKE_HOLD" and s.phase == "DRIVE_ACCEL"), None)
    acc_coast = next(
        (s for s in phases if s.kind == "COAST_DOWN" and s.phase == "DRIVE_ACCEL"), None
    )
    bh = next((s for s in phases if s.phase == "BRAKE_HOLD"), None)
    probe_rows = sum(s.eff_accel for s in probe)
    body = [
        ["1. 開始速度が前パターン次第",
         f"ACCEL_SWEEP 4 本の開始車速が {starts} km/h、BRAKE_HOLD 8 本が {hold_starts} km/h",
         f"停車へ戻す DRIVE_BRAKE が {ret.count if ret else 0} 本すべて打ち切り "
         f"{ret.limit_s if ret else 0:.0f}s に張り付き、停車しないまま次へ進む"
         if ret else "—",
         "各段の前に「停車まで戻す」か「指定速度まで整える」を置き、打ち切りを停車できる長さにする"],
        ["2. WLTP で使わない開度を掃いている",
         f"ACCEL_SWEEP の保持開度は {ACCEL_SWEEP_FRACS} × 上限 "
         f"{cfg.vehicle.max_accel_opening_pct:.0f}% = "
         + " / ".join(f"{f * cfg.vehicle.max_accel_opening_pct:.0f}" for f in ACCEL_SWEEP_FRACS)
         + "%",
         f"WLTP のアクセル需要は最大 {accel_need_max:.1f}%。効くアクセル行 {len(rows_a.opening)} の"
         f"うち {over} 行（{100 * over / len(rows_a.opening):.0f}%）がそれを超える開度",
         "高開度の段を落とし、不感帯 +2〜+12%（＝12〜22%）の保持に付け替える"],
        ["3. 逆に、WLTP がいちばん長く要求する域を掃いていない",
         f"アクセル: {worst_holes(demand, frames, rows_a, PEDAL_ACCEL)}",
         f"ブレーキ: {worst_holes(demand, frames, rows_b, PEDAL_BRAKE)}",
         "固定開度で cap まで上げる代わりに、目標速度まで上げてから"
         "「不感帯すぐ上の開度」を保持して速度帯ごとに採る"],
        ["4. 加速が cap に届かない",
         f"BRAKE_HOLD・COAST_DOWN の到達最高速は {min(s.v_max for s in hold + coast):.1f}〜"
         f"{max(s.v_max for s in hold + coast):.1f} km/h（cap {cap:.1f} km/h 未達）。"
         f"COAST_DOWN の加速は {acc_coast.at_limit if acc_coast else 0} / "
         f"{acc_coast.count if acc_coast else 0} 本が打ち切り "
         f"{acc_coast.limit_s if acc_coast else 0:.0f}s に張り付く",
         "保持や惰行を始める速度が毎回変わるので、高速側のブレーキ・惰行データが揃わない",
         f"加速フェーズの打ち切りを延ばす（{acc.limit_s if acc else 0:.0f} → 30s）"],
        ["5. ブレーキ保持が停車まで届かない",
         f"BRAKE_HOLD の保持は {bh.at_limit if bh else 0} / {bh.count if bh else 0} 本が"
         f"打ち切り {bh.limit_s if bh else 0:.0f}s に張り付く",
         "低開度（不感帯 +0.5〜+4%）ばかりで低速域まで減速できず、"
         "低速 × 高ブレーキ開度が空白のまま（V-1）",
         "低開度の段を減らし、20 km/h から 20〜50% を停車まで踏む段を足す"],
        ["6. 惰行がどちらも cap から始まっていない",
         " / ".join(f"COAST_DOWN {s.name.split(':')[0]} は {s.v_max:.1f} km/h から" for s in coast),
         f"2 本とも cap {cap:.1f} km/h 未達。とくに 1 本は 86 km/h 止まりで、"
         "高速側の惰行カーブを裏取りできるのが 1 本だけになる",
         "加速の打ち切りを延ばして 2 本とも cap から惰行させる"],
        ["7. 保持が設計より短い / 空振り",
         " / ".join(f"CRUISE_TRIM {s.name.split(':')[0]} は {s.duration_s:.1f}s" for s in trim),
         f"設計の保持は {CRUISE_TRIM_HOLD_S:.0f}s。最短の 1 本は実質データ無し",
         "本数を減らして 1 本ずつ設計どおり保持する"],
        ["8. 不感帯プローブは学習セルを埋めない",
         f"ACCEL_DEADBAND_PROBE {len(probe)} 本・"
         f"{sum(s.duration_s for s in probe):.1f}s で効くアクセル行 {probe_rows} 行",
         "すべて低速 × 不感帯すぐ上に入るので V-1 の空白は埋めない",
         "不感帯・クリープの同定用として残す（学習データとしては数えない）"],
    ]
    return md_table(["欠陥", "実測", "何が起きるか", "直し方"], body)


@dataclass(frozen=True)
class ProposalStep:
    """パターン案の 1 項目（所要時間の増減は実測の単位時間から積算する）。"""

    action: str
    name: str
    delta_s: float
    basis: str
    effect: str


PROPOSAL: tuple[ProposalStep, ...] = (
    ProposalStep(
        "削除", "ACCEL_SWEEP の 40 / 56 / 80%（3 段）", -45.9,
        "実測 19.5 + 13.4 + 13.0s",
        "WLTP のアクセル需要は最大 25.2%。この 3 段は外挿にしか効かない（V-1・V-2）",
    ),
    ProposalStep(
        "変更", "BRAKE_HOLD 8 段 → 5 段。うち 2 段は cap から（+1.0 / +3.0%）、"
        "3 段は 60 km/h から（+0.5 / +2.0 / +4.0%）＝20s の打ち切り内で停車まで届く", -98.3,
        "2 本 ×（加速 20.0s ＋ 保持 20.0s）＋ 3 本 ×（60 km/h まで 10.0s ＋ 保持 20.0s）"
        "= 170.0s（現行は実測 268.3s）",
        "15〜20% の 1172 行（V-2）を減らし、代わりに 20〜60 km/h × 13.7〜15% "
        "（27 + 22s 要るのに 0〜16 行）を埋める",
    ),
    ProposalStep(
        "置換", "CRUISE_TRIM（高速 3 本）→ アクセルトリム階段（目標速度まで上げてから"
        "不感帯 +8 / +5 / +2% を各 8s 保持して踏みながら減速 → 停車）を cap / 90 / 50 km/h 始まりで"
        " 3 本", +169.1,
        "削除 −34.9s（実測 28.0 + 6.8 + 0.1s）、追加 +204.0s"
        "（3 本 ×（加速 20.0s ＋ 保持 3 × 8.0s ＋ 停車まで 24.0s = 68.0s））",
        "20〜60 km/h × 10〜20%（WLTP が 625s 要求するのに 37 行しかない最大の空白）と"
        "「踏みながら減速」が埋まる",
    ),
    ProposalStep(
        "追加", "低速 × 高ブレーキ保持（20 km/h から 20 / 30 / 40 / 50% を停車まで）", +36.0,
        "4 本 ×（20 km/h まで 5.0s ＋ 停車まで 4.0s）",
        "0〜20 km/h × 25% 以上のうち、学習行が 0 のセル（25〜30 / 40〜50 / 50% 以上で"
        "合わせて 21s 要る）が埋まる（V-1）",
    ),
    ProposalStep(
        "変更", "加速フェーズの打ち切りを 20 → 30s（cap まで上げる BRAKE_HOLD 2 本・"
        "COAST_DOWN 2 本・トリム階段の cap 始まり 1 本）", +40.0,
        "4 本 × +10.0s（トリム階段の 1 本は上の行に入っている）",
        "cap（137.2 km/h）まで届かせて、高速側のブレーキ・惰行データを揃える",
    ),
    ProposalStep(
        "残す", "CREEP 5 本・CREEP_SETTLE・ACCEL_DEADBAND_PROBE 5 本・COAST_DOWN 2 本", 0.0,
        "実測 14.7 + 12.9 + 14.8 + 136.2s",
        "不感帯・停車保持開度・クリープ・惰行カーブの同定に要る（学習セルは埋めない）",
    ),
)


def proposal_table(current_s: float, timeout_s: float) -> str:
    body = [[s.action, s.name, f"{s.delta_s:+.1f}", s.basis, s.effect] for s in PROPOSAL]
    total = current_s + sum(s.delta_s for s in PROPOSAL)
    body.append([
        "—", "**合計**", f"**{total - current_s:+.1f}**",
        f"**現状 {current_s:.1f}s → 提案 {total:.1f}s**",
        f"**打ち切り {timeout_s:.0f}s に対し余裕 {timeout_s - total:.1f}s**",
    ])
    return md_table(["", "内容", "所要時間[s]", "時間の根拠（手順 2 の実測）", "埋まる / 空くもの"],
                    body)


def _delta_cell(full: np.ndarray, other: np.ndarray) -> str:
    """そのペダルを踏んでいる周期での指令開度の差（平均 / p95 / 最大 [%]）。"""
    m = (full > 0.0) | (other > 0.0)
    if not np.any(m):
        return "—"
    d = np.abs(full[m] - other[m])
    return f"{d.mean():.2f} / {np.percentile(d, 95):.2f} / {d.max():.2f}"


def leave_out_table(
    train_csv: Path, p: FeedforwardParams, cfg: ResearchConfig, frames: RefFrames,
    base: TrainedModels, rows_a: EffectiveRows, rows_b: EffectiveRows,
) -> str:
    """系統を 1 つ抜いて学習し直し、WLTP の指令開度がどれだけ動くかを測る。

    実測データだけで完結する（合成した走行で採点しない）。動き幅が不感帯より大きければ
    「パターン構成が指令を決めている」＝パターン見直しの効き先が分かる。
    「学習行の減り」が「効く行」より小さいのは、パターンの切れ目の行は先読み・過去の窓が
    揃わず、抜く前から学習に入っていないため。
    """
    full_a, full_b = candidate_openings(C1, base, p, cfg, frames)
    body = []
    for kind in PATTERN_KINDS:
        share_a = int(np.sum(rows_a.kind == kind))
        share_b = int(np.sum(rows_b.kind == kind))
        if share_a == 0 and share_b == 0:
            body.append([kind, "0", "0", "—", "—", "変わらない", "変わらない"])
            continue
        model = train_models(train_csv, p, exclude_kinds=(kind,))
        accel, brake = candidate_openings(C1, model, p, cfg, frames)
        body.append([
            kind,
            f"{share_a} ({100 * share_a / len(rows_a.kind):.0f}%)",
            f"{share_b} ({100 * share_b / len(rows_b.kind):.0f}%)",
            f"{model.rows[0] - base.rows[0]:+d}", f"{model.rows[1] - base.rows[1]:+d}",
            _delta_cell(full_a, accel), _delta_cell(full_b, brake),
        ])
    return md_table(
        ["抜いた系統", "効くアクセル行（全体比）", "効くブレーキ行（全体比）",
         "学習行の減り アクセル", "同 ブレーキ",
         "WLTP アクセル指令の変化 平均/p95/最大[%]", "同 ブレーキ[%]"],
        body,
    )


def pnow_table() -> str:
    """PNOW（実位置）を毎周期の読み取りに相乗りさせたときのフレーム長と計算上の増分。"""
    def frame(count: int) -> int:
        return 8 + (5 + 2 * count)  # クエリ 8 バイト＋レスポンス（3 + データ + CRC2）

    cases = (
        ("現行: CNOW だけ（0x900C count=2）を両軸", frame(2) * 2, "電流"),
        ("PNOW を別クエリで足す（0x9000 count=2 を追加）を両軸",
         (frame(2) + frame(2)) * 2, "電流＋実位置"),
        ("まとめ読み 0x9000 count=14 を両軸", frame(14) * 2, "実位置〜電流"),
        ("まとめ読み 0x9000 count=14 を動いている軸だけ（他軸は現行）",
         frame(14) + frame(2), "実位置〜電流"),
        ("まとめ読み 0x9000 count=16 を動いている軸だけ（偏差モニターまで）",
         frame(16) + frame(2), "実位置〜電流＋偏差"),
    )
    base = cases[0][1]
    body = []
    for name, total_b, gets in cases:
        ms = 1000.0 * MODBUS_BYTE_S * total_b
        body.append([
            name, str(total_b), f"{ms:.1f}",
            f"{1000.0 * MODBUS_BYTE_S * (total_b - base):+.1f}",
            f"{100.0 * MODBUS_BYTE_S * (total_b - base) / CONTROL_PERIOD_S:+.1f}%", gets,
        ])
    return md_table(
        ["読み方", "1 周期のバイト数", "伝送時間[ms]", "現行との差[ms]",
         f"周期 {1000 * CONTROL_PERIOD_S:.0f}ms に対する増分", "取れるもの"],
        body,
    )


def fig_coverage_map(
    demand: Demand, frames: RefFrames, rows_a: EffectiveRows, rows_b: EffectiveRows, path: Path
) -> None:
    plt = _plt()
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    panels = (
        (axes[0], PEDAL_ACCEL, rows_a, "アクセル", 40.0),
        (axes[1], PEDAL_BRAKE, rows_b, "ブレーキ", 100.0),
    )
    for ax, pedal, rows, name, top in panels:
        m = demand.pedal == pedal
        ax.scatter(frames.v0_raw[m], demand.opening[m], s=4, color=COLOR_REF, alpha=0.25,
                   label="WLTP が要る開度（物理式で逆算）")
        ax.scatter(rows.speed, rows.opening, s=4, color=COLOR_ACTUAL, alpha=0.35,
                   label="手順 2 の効いている行")
        ax.set_title(f"{name}（{top:.0f}% より上は表示を切っている）")
        ax.set_xlabel("車速 [km/h]")
        ax.set_ylabel("開度 [%]")
        ax.set_xlim(0.0, 145.0)
        ax.set_ylim(0.0, top)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle("WLTP が要る開度と、手順 2 の学習データにある開度")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def fig_coverage_openings(
    demand: Demand, rows_a: EffectiveRows, rows_b: EffectiveRows, path: Path
) -> None:
    plt = _plt()
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    x = np.arange(len(OPEN_BIN_LABELS))
    for ax, pedal, rows, name in (
        (axes[0], PEDAL_ACCEL, rows_a, "アクセル"),
        (axes[1], PEDAL_BRAKE, rows_b, "ブレーキ"),
    ):
        m = demand.pedal == pedal
        ob = _open_index(demand.opening[m], COVER_OPEN_EDGES_PCT)
        ro = _open_index(rows.opening, COVER_OPEN_EDGES_PCT)
        sec = np.array([SIM_DT_S * int(np.sum(ob == j)) for j in x])
        cnt = np.array([int(np.sum(ro == j)) for j in x])
        ax.bar(x - 0.2, sec, width=0.4, color=COLOR_REF, label="WLTP が要る時間 [s]")
        ax.set_ylabel("WLTP が要る時間 [s]")
        ax2 = ax.twinx()
        ax2.bar(x + 0.2, cnt, width=0.4, color=COLOR_ACTUAL, label="手順 2 の効く行数")
        ax2.set_ylabel("学習の効く行数")
        ax.set_xticks(x)
        ax.set_xticklabels(OPEN_BIN_LABELS)
        ax.set_xlabel("開度 [%]")
        ax.set_title(name)
        ax.grid(alpha=0.3)
        handles = ax.get_legend_handles_labels()[0] + ax2.get_legend_handles_labels()[0]
        labels = ax.get_legend_handles_labels()[1] + ax2.get_legend_handles_labels()[1]
        ax.legend(handles, labels, loc="upper right", fontsize=8)
    fig.suptitle("開度ごとの「WLTP が要る時間」と「学習データの行数」")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def run_coverage(cfg: ResearchConfig, train_csv: Path, out_dir: Path) -> int:
    p = feedforward_params(cfg)
    mode = asyncio.run(load_mode(cfg, cfg.modes.wltp_mode_name))
    ref = ReferenceSpeed(mode)
    t = np.round(np.arange(0.0, mode.total_duration + SIM_DT_S / 2, SIM_DT_S), 4)
    frames = ref_frames(ref, t, DEFAULT_FEATURE_SPEC)
    demand = wltp_demand(p, frames)
    rows_a, rows_b = effective_rows(train_csv, p)
    stats, span_s = pattern_stats(train_csv, p)
    phases = phase_stats(train_csv)
    print(f"# 段階 3 手順 2 パターン走行の網羅性と所要時間"
          f"（要る開度は {mode.name} {mode.total_duration:.0f}s から物理式で逆算、"
          f"学習データは {train_csv.name}）")
    print(f"\n- 不感帯: アクセル {p.accel_deadband_pct:g}% / ブレーキ {p.brake_deadband_pct:g}%"
          f"（開度ビンの下端はアクセル不感帯に合わせている）")

    print("\n### V-1 網羅性（速度帯 × 開度。セルは「WLTP が要る時間 / 学習の効く行数」）")
    print("\n#### アクセル")
    print(coverage_table(demand, frames, rows_a, PEDAL_ACCEL))
    print("\n#### ブレーキ")
    print(coverage_table(demand, frames, rows_b, PEDAL_BRAKE))
    print("\n### V-2 開度ごとの分布（速度をまとめたもの）")
    print(opening_hist_table(demand, rows_a, rows_b))
    print("\n### V-3 パターンごとの実績（手順 2 のログ）")
    print(pattern_table(stats, span_s))
    print("\n### V-4 フェーズごとの所要時間（打ち切りに張り付いていないか）")
    print(phase_table(phases))
    print("\n### V-5 現行パターンの欠陥")
    print(defect_table(stats, phases, demand, frames, rows_a, rows_b, cfg))
    print("\n### V-6 パターン案と所要時間（車両モデルでは模擬しない）")
    print(proposal_table(span_s, cfg.learning.timeout_s))
    print("\n### V-7 系統を 1 つ抜いて学習し直したときの指令の変化")
    print(leave_out_table(train_csv, p, cfg, frames,
                          train_models(train_csv, p, label="全体"), rows_a, rows_b))
    print("\n### V-8 実開度（PNOW 0x9000）を毎周期の読み取りに相乗りさせる案")
    print(pnow_table())

    out_dir.mkdir(parents=True, exist_ok=True)
    fig_coverage_map(demand, frames, rows_a, rows_b, out_dir / "coverage_map.png")
    fig_coverage_openings(demand, rows_a, rows_b, out_dir / "coverage_openings.png")
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
    ap.add_argument("--part", choices=("sim-check", "compare", "coverage"), required=True)
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
    if args.part == "coverage":
        return run_coverage(cfg, args.train_csv, out_dir)
    return run_sim_check(cfg, args.run_csv, args.train_csv, out_dir)


if __name__ == "__main__":
    raise SystemExit(main())
