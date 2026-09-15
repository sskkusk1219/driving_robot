"""手順 2 の 2次多項式 Ridge 逆モデルを分析する（ProblemReport_20260912「今回実施する内容」2.）。

車両・アクチュエータには触らない。表をターミナルに出し、図・CSV を --out に保存する
（レポート reportYYYYMMDD_analysisFF_model.md に使う）。

    .venv/bin/python -m tests.research.model_analysis --part metrics
    .venv/bin/python -m tests.research.model_analysis --part simulate

    --part metrics   2-1 手順 2 時点の当てはまり（R²・RMSE・MAE）。学習セットを手順 2 と同じ手順で
                     作り直し、保存済みの指標が再現できることを確かめてから、パターン単位の分割検証・
                     ラベル別・パターン種別・フェーズごとの誤差を出す。
    --part simulate  2-2 走行モード管理の WLTP 1800s の基準車速だけをモデルに通し（実車速なし）、
                     アクセル・ブレーキ開度を 2 通りで出す（CSV・PNG・集計表）:
                       wltp_model_raw … モデル出力そのもの。学習時と同じく dv_1.0 ≥ 0 なら
                                        アクセルモデル、< 0 ならブレーキモデルの出力（負は 0）。
                                        FF の定数は使わない
                       wltp_ff        … 手順 3 と同じ FF の effort を符号で振り分け、
                                        開度上限でクランプ

学習セットの作り方は本番 train_inverse_model（src/domain/model_training.py）と同じ:
    ラベル   … アクセル = 原点からの開度、ブレーキ = 開度（brake_deadband_pct 未満は 0）
    分け方   … 1.0s 先までの車速変化 dv_1.0 ≥ 0 の行 → アクセルモデル、< 0 の行 → ブレーキモデル
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import pickle
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.model_selection import GroupKFold

from src.domain.model_training import (
    DEFAULT_FEATURE_SPEC,
    FeatureSpec,
    _build_feature_matrix,
    _estimate_offsets,
    _make_estimator,
    _metrics,
)
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research.config import DEFAULT_CONFIG_PATH, ResearchConfig, load_config
from tests.research.drive_log import SECTION_PATTERN_DRIVE, actual_opening, read_drive_logs
from tests.research.ff_explain import (
    BRANCHES,
    DT_S,
    ModelOutputs,
    _branch_patches,
    _plt,
    _shade,
    decide_all,
    load_ff_model,
    md_table,
    outputs_for_mode,
    pedal_class,
    verify_against_production,
)
from tests.research.live_plot import COLOR_ACCEL, COLOR_BRAKE, COLOR_REF, FIGSIZE
from tests.research.mode_drive import ReferenceSpeed, load_mode
from tests.research.mode_report import STATES, ModeRow, driving_states
from tests.research.vehicle import feedforward_params

# 手順 2 で test_vehicle_20260911_072504.pkl を作った走行（pkl には学習 CSV のパスが残らない。
# 取り違えは保存済み指標の再現チェックで検出する）
DEFAULT_TRAIN_CSV = Path("tests/research/results/drive_log_real_20260911_161247.csv")
IN_DEADBAND_MIN_PCT = 0.5  # これより大きく不感帯未満の予測を「不感帯内」と数える [%]
CV_SPLITS = 5  # パターン単位の分割検証の分割数
METRIC_REL_TOL = 1e-6  # 保存値との一致判定（相対）
EXIT_METRIC_MISMATCH = 2
SERIES_CSV_COLUMNS = ("time_s", "ref_speed_kmh", "accel_opening", "brake_opening")
PEDAL_CLASS_NAMES = {"A": "実効アクセル", "B": "実効ブレーキ", "-": "実質惰行"}


# ─────────────────────────────────────────────────────────────────────
# 2-1 学習セット
# ─────────────────────────────────────────────────────────────────────


@dataclass
class RegimeData:
    """アクセル側またはブレーキ側の学習セット（train_inverse_model と同じ行・同じ特徴）。"""

    name: str  # "accel" / "brake"（pkl の metrics のキー）
    label: str  # 表示名
    x: np.ndarray
    y: np.ndarray  # ラベル（開度 [%]）
    pattern: np.ndarray  # CSV の pattern 列（"12:ACCEL_SWEEP"）
    phase: np.ndarray  # CSV の phase 列
    deadband_pct: float

    @property
    def kind(self) -> np.ndarray:
        """パターン種別（pattern 列の ":" より後ろ）。"""
        return np.array([p.split(":", 1)[-1] for p in self.pattern])


def load_training_rows(
    csv_path: Path, *, label: str = "cmd"
) -> tuple[list[DriveLog], np.ndarray, np.ndarray]:
    """学習に使うパターン走行の行と、それぞれの pattern / phase 列。

    `label`（`read_drive_logs` と同じ）: "actual" のときは実開度が読めなかった行を
    `read_drive_logs` 側で除外するので、pattern/phase もここで同じ行だけに絞る。
    """
    logs = read_drive_logs(csv_path, label=label)
    patterns: list[str] = []
    phases: list[str] = []
    with csv_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if "section" in row and row["section"] != SECTION_PATTERN_DRIVE:
                continue
            if label == "actual" and (
                actual_opening(row, "accel") is None or actual_opening(row, "brake") is None
            ):
                continue
            patterns.append(row.get("pattern", ""))
            phases.append(row.get("phase", ""))
    if len(patterns) != len(logs):  # pragma: no cover - read_drive_logs と同じ行を読んでいる
        raise ValueError(f"pattern 列の行数 {len(patterns)} が学習行 {len(logs)} と一致しません")
    return logs, np.array(patterns), np.array(phases)


def build_training_set(
    logs: Sequence[DriveLog],
    patterns: np.ndarray,
    phases: np.ndarray,
    *,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
) -> tuple[RegimeData, RegimeData]:
    """train_inverse_model（1 セッション分）と同じ手順で学習セットを作る。

    (アクセル側, ブレーキ側) を返す。ブレーキのラベルは不感帯未満を 0 にし、
    dv_1.0 の符号で行を分ける。
    """
    speed = np.clip(np.array([lg.actual_speed_kmh for lg in logs], dtype=float), 0.0, None)
    accel = np.array([lg.accel_opening for lg in logs], dtype=float)
    brake_raw = np.array([lg.brake_opening for lg in logs], dtype=float)
    brake = np.where(brake_raw >= brake_deadband_pct, brake_raw, 0.0)
    timestamps = [lg.timestamp for lg in logs]
    x, idx = _build_feature_matrix(
        speed,
        _estimate_offsets(timestamps, spec.lookahead_horizons_s),
        _estimate_offsets(timestamps, spec.past_horizons_s),
        spec,
    )
    accel_mask = x[:, spec.regime_col()] >= 0.0

    def part(name: str, label: str, y: np.ndarray, mask: np.ndarray, db: float) -> RegimeData:
        return RegimeData(
            name=name, label=label, x=x[mask], y=y[idx][mask],
            pattern=patterns[idx][mask], phase=phases[idx][mask], deadband_pct=db,
        )

    return (
        part("accel", "アクセル", accel, accel_mask, accel_deadband_pct),
        part("brake", "ブレーキ", brake, ~accel_mask, brake_deadband_pct),
    )


# ─────────────────────────────────────────────────────────────────────
# 2-1 指標
# ─────────────────────────────────────────────────────────────────────


def score(y: np.ndarray, pred: np.ndarray) -> dict[str, float]:
    """_metrics と同じ指標（R² は分散があるときだけ）を予測値から出す。"""
    out = {
        "mae": float(mean_absolute_error(y, pred)),
        "rmse": float(mean_squared_error(y, pred) ** 0.5),
        "n": float(len(y)),
    }
    if len(y) >= 2 and float(np.var(y)) > 1e-9:
        out["r2"] = float(r2_score(y, pred))
    return out


def metrics_match(saved: dict[str, float], recomputed: dict[str, float]) -> bool:
    keys = set(saved) | set(recomputed)
    return all(
        k in saved and k in recomputed
        and abs(saved[k] - recomputed[k]) <= METRIC_REL_TOL * max(1.0, abs(saved[k]))
        for k in keys
    )


def predict_out_of_pattern(data: RegimeData, n_splits: int = CV_SPLITS) -> np.ndarray:
    """パターン単位で分けて学習し直し、各行を「その行のパターンを学習に使っていないモデル」で予測する。"""
    groups = data.pattern
    splits = min(n_splits, len(np.unique(groups)))
    oof = np.full(len(data.y), np.nan)
    if splits < 2:
        return oof  # パターンが 1 本だけでは「使っていないパターン」が作れない
    for train, test in GroupKFold(n_splits=splits).split(data.x, data.y, groups=groups):
        model = _make_estimator().fit(data.x[train], data.y[train])
        oof[test] = model.predict(data.x[test])
    return oof


def in_deadband(values: np.ndarray, deadband_pct: float) -> np.ndarray:
    """0.5% より大きく不感帯未満（ペダルは動くが効かない開度）。"""
    return (values > IN_DEADBAND_MIN_PCT) & (values < deadband_pct)


def _fmt(metrics: dict[str, float], key: str, digits: int = 3) -> str:
    return f"{metrics[key]:.{digits}f}" if key in metrics else "—"


# ─────────────────────────────────────────────────────────────────────
# 2-1 の表と図
# ─────────────────────────────────────────────────────────────────────


def saved_models_table(model_path: Path) -> str:
    rows = []
    for path in sorted(model_path.parent.glob("*.pkl")):
        with path.open("rb") as f:
            data = pickle.load(f)  # noqa: S301 - 手順 2 で作った信頼済みファイル
        m = data.get("metrics", {})
        a, b = m.get("accel", {}), m.get("brake", {})
        rows.append([
            f"{'★ ' if path.name == model_path.name else ''}`{path.name}`",
            str(data.get("trained_at", ""))[:19].replace("T", " "),
            _fmt(a, "r2"), _fmt(a, "rmse", 2), _fmt(a, "mae", 2), f"{a.get('n', 0):.0f}",
            _fmt(b, "r2"), _fmt(b, "rmse", 2), _fmt(b, "mae", 2), f"{b.get('n', 0):.0f}",
            f"{data.get('speed_clip_max', float('nan')):.1f}",
        ])
    return "\n\n".join([
        "### M-1 保存済みモデルの指標（学習データ上。★ = config の model_path）",
        md_table(
            ["モデル", "学習日時(UTC)", "アクセル R²", "アクセル RMSE[%]", "アクセル MAE[%]",
             "アクセル n", "ブレーキ R²", "ブレーキ RMSE[%]", "ブレーキ MAE[%]", "ブレーキ n",
             "学習最高車速[km/h]"],
            rows,
        ),
    ])


def fit_table(
    regimes: Sequence[RegimeData],
    saved: dict[str, dict[str, float]],
    recomputed: dict[str, dict[str, float]],
    oof: dict[str, np.ndarray],
) -> str:
    rows = []
    for d in regimes:
        for label, m in (
            ("学習データ上（保存値）", saved[d.name]),
            ("学習データ上（再計算）", recomputed[d.name]),
            (f"パターン外（{CV_SPLITS} 分割）", score(d.y, oof[d.name])),
        ):
            rows.append([d.label, label, _fmt(m, "r2"), _fmt(m, "rmse", 3), _fmt(m, "mae", 3),
                         f"{m['n']:.0f}"])
    return "\n\n".join([
        "### M-2 当てはまり（保存値の再現と、学習に使っていないパターンでの値）",
        md_table(["モデル", "評価", "R²", "RMSE[%]", "MAE[%]", "n"], rows),
        "パターン外 = CSV の pattern 列（28 本）を GroupKFold で分け、そのパターンを学習に使わずに"
        "作り直したモデルで予測した値。",
    ])


def label_class_table(
    regimes: Sequence[RegimeData], pred: dict[str, np.ndarray], oof: dict[str, np.ndarray]
) -> str:
    rows = []
    for d in regimes:
        p, q, y, db = pred[d.name], oof[d.name], d.y, d.deadband_pct
        classes = (
            ("0（踏んでいない）", y == 0.0),
            (f"0 < ラベル < {db:g}（不感帯内）", (y > 0.0) & (y < db)),
            (f"{db:g} 以上（効いている）", y >= db),
            ("全体", np.ones(len(y), dtype=bool)),
        )
        for name, m in classes:
            if not m.any():
                rows.append([d.label, name, 0, "0%", "—", "—", "—", "—", "—", "—"])
                continue
            rows.append([
                d.label, name, int(m.sum()), f"{100 * np.mean(m):.0f}%",
                f"{y[m].mean():.1f}", f"{np.median(p[m]):.1f}",
                f"{mean_absolute_error(y[m], p[m]):.2f}", f"{mean_absolute_error(y[m], q[m]):.2f}",
                f"{100 * np.mean(in_deadband(p[m], db)):.0f}%",
                f"{100 * np.mean(in_deadband(q[m], db)):.0f}%",
            ])
    return "\n\n".join([
        "### M-3 ラベルの種類別の誤差と、不感帯内の予測",
        md_table(
            ["モデル", "ラベル", "行数", "割合", "ラベル平均[%]", "予測 中央値[%]",
             "MAE 学習データ上[%]", "MAE パターン外[%]", "予測が不感帯内（学習データ上）",
             "予測が不感帯内（パターン外）"],
            rows,
        ),
        f"不感帯内 = {IN_DEADBAND_MIN_PCT:g}% より大きく不感帯未満（ペダルは動くが効かない開度）。",
    ])


GROUPINGS = {
    "kind": ("### M-4 パターン種別ごとの誤差", "パターン種別"),
    "phase": ("### M-5 フェーズ（その行でパターン走行ループが何をしていたか）ごとの誤差",
              "フェーズ"),
}


def group_table(
    regimes: Sequence[RegimeData],
    pred: dict[str, np.ndarray],
    oof: dict[str, np.ndarray],
    *,
    by: str,
) -> str:
    """by = "kind"（パターン種別）/ "phase"（行ごとのフェーズ）で分けた誤差。"""
    title, header = GROUPINGS[by]
    rows = []
    for d in regimes:
        keys = d.kind if by == "kind" else d.phase
        for k in dict.fromkeys(keys.tolist()):
            m = keys == k
            y, p, q = d.y[m], pred[d.name][m], oof[d.name][m]
            rows.append([
                d.label, k, int(m.sum()), f"{y.mean():.1f}", f"{p.mean():.1f}",
                f"{mean_absolute_error(y, p):.2f}", f"{mean_absolute_error(y, q):.2f}",
                f"{100 * np.mean(in_deadband(p, d.deadband_pct)):.0f}%",
            ])
    return "\n\n".join([
        title,
        md_table(
            ["モデル", header, "行数", "ラベル平均[%]", "予測平均[%]",
             "MAE 学習データ上[%]", "MAE パターン外[%]", "予測が不感帯内（学習データ上）"],
            rows,
        ),
    ])


def fig_fit(regimes: Sequence[RegimeData], pred: dict[str, np.ndarray], path: Path) -> None:
    plt = _plt()
    fig, axes = plt.subplots(2, len(regimes), figsize=(14, 10))
    for col, d in enumerate(regimes):
        color = COLOR_ACCEL if d.name == "accel" else COLOR_BRAKE
        p, y, db = pred[d.name], d.y, d.deadband_pct
        top = max(float(y.max()), float(p.max()), db) * 1.05
        ax = axes[0, col]
        ax.axhspan(IN_DEADBAND_MIN_PCT, db, color="#f5d76e", alpha=0.25, label="予測が不感帯内")
        ax.scatter(y, p, s=4, color=color, alpha=0.35)
        ax.plot([0, top], [0, top], color="#555555", linewidth=1.0, label="予測 = ラベル")
        ax.axvline(db, color="#999999", linestyle=":", linewidth=1.0)
        ax.set_xlim(-1, top)
        ax.set_ylim(min(-1.0, float(p.min()) - 1.0), top)
        ax.set_xlabel("ラベル（実開度）[%]")
        ax.set_ylabel("モデルの予測 [%]")
        ax.set_title(f"{d.label}モデル: 予測 vs ラベル（学習データ上・{len(y)} 行）")
        ax.legend(loc="upper left", fontsize=9)
        ax.grid(alpha=0.3)

        ax = axes[1, col]
        bins = np.arange(-2.0, top + 0.5, 0.5)
        ax.axvspan(IN_DEADBAND_MIN_PCT, db, color="#f5d76e", alpha=0.25, label="不感帯内")
        ax.hist(y, bins=bins, color="#888888", alpha=0.6, label="ラベル")
        ax.hist(p, bins=bins, color=color, alpha=0.6, label="予測")
        ax.set_yscale("log")
        ax.set_xlabel("開度 [%]")
        ax.set_ylabel("行数（対数）")
        ax.set_title(f"{d.label}: ラベルと予測の分布（0.5% 刻み）")
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def run_metrics(model_path: Path, train_csv: Path, out_dir: Path, *, accel_db: float,
                brake_db: float) -> int:
    with model_path.open("rb") as f:
        model: dict[str, Any] = pickle.load(f)  # noqa: S301 - 手順 2 で作った信頼済みファイル
    spec = FeatureSpec(**model["feature_spec"])
    logs, patterns, phases = load_training_rows(train_csv)
    regimes = build_training_set(
        logs, patterns, phases, accel_deadband_pct=accel_db, brake_deadband_pct=brake_db, spec=spec
    )
    estimators = {"accel": model["accel_model"], "brake": model["brake_model"]}
    print(f"# 2-1 モデルの当てはまり: `{model_path}`")
    print(f"- 学習 CSV: `{train_csv}`（パターン走行 {len(logs)} 行）・"
          f"不感帯 アクセル {accel_db:g}% / ブレーキ {brake_db:g}%")

    saved = model["metrics"]
    recomputed = {d.name: _metrics(estimators[d.name], d.x, d.y) for d in regimes}
    for d in regimes:
        if not metrics_match(saved[d.name], recomputed[d.name]):
            print(f"\n**保存値を再現できません（{d.label}）**: 保存 {saved[d.name]} / "
                  f"再計算 {recomputed[d.name]}。学習 CSV（--train-csv）か不感帯が違います。")
            return EXIT_METRIC_MISMATCH
    print("- 保存済みの指標を再現できた（学習セットは手順 2 と同じ）")

    pred = {d.name: np.asarray(estimators[d.name].predict(d.x), dtype=float) for d in regimes}
    oof = {d.name: predict_out_of_pattern(d) for d in regimes}
    print()
    print(saved_models_table(model_path))
    print()
    print(fit_table(regimes, saved, recomputed, oof))
    print()
    print(label_class_table(regimes, pred, oof))
    print()
    print(group_table(regimes, pred, oof, by="kind"))
    print()
    print(group_table(regimes, pred, oof, by="phase"))
    out_dir.mkdir(parents=True, exist_ok=True)
    fig_fit(regimes, pred, out_dir / "fit_scatter.png")
    print()
    print(f"- 図: {out_dir}")
    return 0


# ─────────────────────────────────────────────────────────────────────
# 2-2 モデルだけで WLTP を通す
# ─────────────────────────────────────────────────────────────────────


@dataclass
class OpeningSeries:
    """1 通りの開度の定義で出した WLTP 全体の開度 [%]。"""

    key: str  # ファイル名（wltp_model_raw / wltp_ff）
    title: str
    accel: np.ndarray
    brake: np.ndarray


def raw_openings(out: ModelOutputs) -> tuple[np.ndarray, np.ndarray]:
    """モデル出力そのもの。

    学習時と同じく a_req（dv_1.0）≥ 0 ならアクセル、< 0 ならブレーキのモデル出力（負は 0）。
    """
    accel_side = out.a_req >= 0.0
    accel = np.where(accel_side, np.clip(out.accel_raw, 0.0, None), 0.0)
    brake = np.where(accel_side, 0.0, np.clip(out.brake_raw, 0.0, None))
    return accel, brake


def ff_openings(
    effort: np.ndarray, max_accel_pct: float, max_brake_pct: float
) -> tuple[np.ndarray, np.ndarray]:
    """手順 3 の調停（mode_drive.split_effort）: effort の符号で振り分け、開度上限でクランプ。"""
    return np.clip(effort, 0.0, max_accel_pct), np.clip(-effort, 0.0, max_brake_pct)


def switch_count(accel: np.ndarray, brake: np.ndarray) -> int:
    """アクセル ⇔ ブレーキの切り替え回数（両方 0 の区間は挟んでも数えない）。"""
    seq = np.where(accel > 0.0, 1, np.where(brake > 0.0, -1, 0))
    active = seq[seq != 0]
    return int(np.count_nonzero(np.diff(active)))


def write_series_csv(
    path: Path, t: np.ndarray, ref: np.ndarray, accel: np.ndarray, brake: np.ndarray
) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(SERIES_CSV_COLUMNS)
        for row in zip(t, ref, accel, brake, strict=True):
            writer.writerow([f"{row[0]:.1f}", *(f"{v:.3f}" for v in row[1:])])


def _pct(mask: np.ndarray) -> str:
    return f"{100 * np.mean(mask):.0f}%" if mask.size else "—"


def _seconds(mask: np.ndarray) -> str:
    return f"{np.sum(mask) * DT_S:.0f}"


def _max(values: np.ndarray) -> str:
    return f"{values.max():.1f}" if values.size else "—"


def summary_table(series: Sequence[OpeningSeries], p: FeedforwardParams) -> str:
    rows = []
    for s in series:
        cls = pedal_class(s.accel, s.brake, p)
        rows.append([
            s.title,
            _seconds(s.accel > 0.0), _seconds(in_deadband(s.accel, p.accel_deadband_pct)),
            _max(s.accel),
            _seconds(s.brake > 0.0), _seconds(in_deadband(s.brake, p.brake_deadband_pct)),
            _max(s.brake),
            _pct(cls == "A"), _pct(cls == "B"), _pct(cls == "-"),
            switch_count(s.accel, s.brake),
        ])
    return "\n\n".join([
        "### S-1 WLTP 1800s 全体",
        md_table(
            ["開度の定義", "アクセル指令あり[s]", "うち不感帯内[s]", "アクセル最大[%]",
             "ブレーキ指令あり[s]", "うち不感帯内[s]", "ブレーキ最大[%]",
             "実効アクセル", "実効ブレーキ", "実質惰行", "アクセル⇔ブレーキ切替[回]"],
            rows,
        ),
        f"指令あり = 開度 > 0。不感帯内 = {IN_DEADBAND_MIN_PCT:g}% より大きく不感帯"
        f"（アクセル {p.accel_deadband_pct:g}% / ブレーキ {p.brake_deadband_pct:g}%）未満。"
        "実効アクセル / 実効ブレーキ / 実質惰行 = 開度が不感帯を超えてアクセルが効く / "
        "ブレーキが効く / どちらも効かない時間の割合。",
    ])


def grouped_pedal_table(
    title: str,
    header: str,
    keys: np.ndarray,
    order: Sequence[str],
    series: Sequence[OpeningSeries],
    p: FeedforwardParams,
) -> str:
    rows = []
    for k in order:
        m = keys == k
        if not m.any():
            continue
        for s in series:
            a, b = s.accel[m], s.brake[m]
            cls = pedal_class(a, b, p)
            shallow = in_deadband(a, p.accel_deadband_pct) | in_deadband(b, p.brake_deadband_pct)
            rows.append([
                k, s.title, _seconds(m), _pct(cls == "A"), _pct(cls == "B"), _pct(cls == "-"),
                _pct(shallow), _max(a), _max(b),
            ])
    return "\n\n".join([
        title,
        md_table(
            [header, "開度の定義", "時間[s]", "実効アクセル", "実効ブレーキ", "実質惰行",
             "不感帯内の指令", "アクセル最大[%]", "ブレーキ最大[%]"],
            rows,
        ),
    ])


def branch_diff_table(
    branch: np.ndarray, raw: OpeningSeries, ff: OpeningSeries, p: FeedforwardParams
) -> str:
    """FF の分岐ごとに、モデル出力そのものと FF 指令で効くペダルがどう変わったか。"""
    raw_cls = pedal_class(raw.accel, raw.brake, p)
    ff_cls = pedal_class(ff.accel, ff.brake, p)
    rows = []
    for name in BRANCHES:
        m = branch == name
        if not m.any():
            continue
        rows.append([
            name, _seconds(m),
            *(_pct(raw_cls[m] == c) for c in PEDAL_CLASS_NAMES),
            *(_pct(ff_cls[m] == c) for c in PEDAL_CLASS_NAMES),
            _seconds(m & (raw_cls != ff_cls)),
            f"{np.mean(np.abs(ff.accel[m] - raw.accel[m])):.1f}",
            f"{np.mean(np.abs(ff.brake[m] - raw.brake[m])):.1f}",
        ])
    names = list(PEDAL_CLASS_NAMES.values())
    return "\n\n".join([
        "### S-4 FF の分岐ごとの差（モデル出力そのもの → 手順 3 の FF 指令）",
        md_table(
            ["FF の分岐", "時間[s]", *(f"モデル: {n}" for n in names), *(f"FF: {n}" for n in names),
             "効くペダルが変わった時間[s]", "|Δアクセル| 平均[%]", "|Δブレーキ| 平均[%]"],
            rows,
        ),
        "分岐は 1. のレポート（report20260912_explanationFF.md）と同じ。"
        "モデル出力そのものは分岐を持たず、a_req の符号だけでアクセル/ブレーキモデルを選ぶ。",
    ])


def fig_series(
    t: np.ndarray, ref: np.ndarray, s: OpeningSeries, p: FeedforwardParams, cfg: ResearchConfig,
    path: Path,
) -> None:
    """手順 3 の走行図と同じ 2 段（1 段目 = 車速、2 段目 = 開度）。実車速は無いので基準車速だけ。"""
    plt = _plt()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=FIGSIZE, sharex=True)
    ax1.plot(t, ref, color=COLOR_REF, linestyle="--", linewidth=1.2, label="基準車速")
    ax1.set_ylabel("車速 [km/h]")
    ax1.set_ylim(0.0, cfg.vehicle.max_speed_kmh * 1.05)
    ax1.legend(loc="upper right")
    ax2.plot(t, s.accel, color=COLOR_ACCEL, linewidth=0.9, label="アクセル")
    ax2.plot(t, s.brake, color=COLOR_BRAKE, linewidth=0.9, label="ブレーキ")
    ax2.axhline(p.accel_deadband_pct, color=COLOR_ACCEL, linestyle=":", linewidth=1.2,
                label=f"アクセル不感帯 {p.accel_deadband_pct:g}%")
    ax2.axhline(p.brake_deadband_pct, color=COLOR_BRAKE, linestyle=":", linewidth=1.2,
                label=f"ブレーキ不感帯 {p.brake_deadband_pct:g}%")
    top = max(float(s.accel.max()), float(s.brake.max()), p.brake_deadband_pct)
    ax2.set_ylim(0.0, max(40.0, top * 1.1))
    ax2.set_ylabel("開度 [%]")
    ax2.set_xlabel("モード経過時間 [s]")
    ax2.legend(loc="upper right", fontsize=8, ncol=2)
    for ax in (ax1, ax2):
        for b in cfg.modes.segment_bounds_s:
            ax.axvline(b, color="#555555", linestyle=":", linewidth=0.8)
        ax.grid(alpha=0.3)
    fig.suptitle(f"WLTP 1800s: {s.title}")
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)


def fig_compare(
    t: np.ndarray, ref: np.ndarray, branch: np.ndarray, series: Sequence[OpeningSeries],
    p: FeedforwardParams, t0: float, t1: float, path: Path,
) -> None:
    """拡大図: 1 段目 = 基準車速（背景色 = FF の分岐）、2 段目以降 = 定義ごとの開度。"""
    plt = _plt()
    m = (t >= t0) & (t <= t1)
    fig, axes = plt.subplots(1 + len(series), 1, figsize=(14, 3.2 * (1 + len(series))),
                             sharex=True)
    for ax in axes:
        _shade(ax, t[m], branch[m])
        ax.grid(alpha=0.3)
    axes[0].plot(t[m], ref[m], color="#333333", linestyle="--", linewidth=1.4, label="基準車速")
    axes[0].set_ylabel("車速 [km/h]")
    lines, _ = axes[0].get_legend_handles_labels()
    axes[0].legend(handles=lines + _branch_patches(), loc="upper left", ncol=4, fontsize=8)
    top = max(max(float(s.accel[m].max()), float(s.brake[m].max())) for s in series)
    for ax, s in zip(axes[1:], series, strict=True):
        ax.plot(t[m], s.accel[m], color=COLOR_ACCEL, linewidth=1.5, label="アクセル")
        ax.plot(t[m], s.brake[m], color=COLOR_BRAKE, linewidth=1.5, label="ブレーキ")
        ax.axhline(p.accel_deadband_pct, color=COLOR_ACCEL, linestyle=":", linewidth=1.2,
                   label="アクセル不感帯")
        ax.axhline(p.brake_deadband_pct, color=COLOR_BRAKE, linestyle=":", linewidth=1.2,
                   label="ブレーキ不感帯")
        ax.set_ylim(0.0, max(top, p.brake_deadband_pct) * 1.1)
        ax.set_ylabel("開度 [%]")
        ax.set_title(s.title, fontsize=10)
        ax.legend(loc="upper right", fontsize=8, ncol=2)
    axes[-1].set_xlabel("モード経過時間 [s]")
    fig.suptitle(f"モデル出力そのもの と FF 指令の比較（{t0:.0f}〜{t1:.0f}s、背景色 = FF の分岐）")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def run_simulate(cfg: ResearchConfig, out_dir: Path, zooms: Sequence[tuple[float, float]]) -> int:
    p = feedforward_params(cfg)
    model = load_ff_model(cfg.feedforward.model_path)
    mode = asyncio.run(load_mode(cfg, cfg.modes.wltp_mode_name))
    ref_speed = ReferenceSpeed(mode)
    t = np.round(np.arange(0.0, mode.total_duration + DT_S / 2, DT_S), 3)
    out = outputs_for_mode(model, ref_speed, t)
    trace = decide_all(p, out)
    worst = verify_against_production(p, model, ref_speed, trace)
    ref = out.v0_raw

    raw = OpeningSeries("wltp_model_raw", "モデル出力そのもの", *raw_openings(out))
    ff = OpeningSeries(
        "wltp_ff", "手順 3 の FF 指令",
        *ff_openings(trace.effort, cfg.vehicle.max_accel_opening_pct,
                     cfg.vehicle.max_brake_opening_pct),
    )
    series = (raw, ff)
    print(f"# 2-2 モデルだけで WLTP: `{cfg.feedforward.model_path}` / {mode.name}"
          f"（{mode.total_duration:.0f}s・最高 {mode.max_speed:.1f} km/h・"
          f"{DT_S:g}s 刻み {len(t)} 点）")
    print(f"- FF 指令は本番 predict_effort と一致（最大差 {worst:.1e} %）")

    out_dir.mkdir(parents=True, exist_ok=True)
    for s in series:
        write_series_csv(out_dir / f"{s.key}.csv", t, ref, s.accel, s.brake)
        fig_series(t, ref, s, p, cfg, out_dir / f"{s.key}.png")

    rows = [
        ModeRow(t_s=float(ti), ref_kmh=float(r), actual_kmh=0.0, accel_pct=0.0, brake_pct=0.0,
                ff_effort_pct=0.0, pid_effort_pct=0.0, effort_pct=0.0, segment="", phase="")
        for ti, r in zip(t, ref, strict=True)
    ]
    states = np.array(driving_states(rows))
    segments = np.array([cfg.modes.segment_at(float(x)) for x in t])
    print()
    print(summary_table(series, p))
    print()
    print(grouped_pedal_table("### S-2 WLTP 区間ごと", "区間", segments,
                              cfg.modes.segment_names, series, p))
    print()
    print(grouped_pedal_table(
        "### S-3 走行状態ごと（基準車速の前後 0.5s の傾き ±0.3 km/h/s で分類）",
        "走行状態", states, STATES, series, p,
    ))
    print()
    print(branch_diff_table(trace.branch, raw, ff, p))
    for t0, t1 in zooms:
        fig_compare(t, ref, trace.branch, series, p, t0, t1,
                    out_dir / f"compare_{t0:.0f}_{t1:.0f}.png")
    print()
    print(f"- CSV・図: {out_dir}")
    return 0


# ─────────────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────────────


def parse_zooms(text: str) -> list[tuple[float, float]]:
    spans = []
    for span in text.split(","):
        t0, t1 = (float(x) for x in span.split(":"))
        spans.append((t0, t1))
    return spans


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--part", choices=("metrics", "simulate"), required=True)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--train-csv", type=Path, default=DEFAULT_TRAIN_CSV,
                    help="手順 2 でモデルを作った走行 CSV（--part metrics）")
    ap.add_argument("--zoom", default="20:120,860:930",
                    help="比較の拡大図の区間 [s]（開始:終了 をカンマ区切り。--part simulate）")
    ap.add_argument("--out", type=Path, default=None,
                    help="図・CSV の保存先（既定: results/report<今日>_analysisFF_model/）")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    out_dir = args.out or cfg.results_path / f"report{datetime.now():%Y%m%d}_analysisFF_model"
    ff = cfg.feedforward
    if args.part == "simulate":
        return run_simulate(cfg, out_dir, parse_zooms(args.zoom))
    return run_metrics(
        Path(ff.model_path), args.train_csv, out_dir,
        accel_db=ff.accel_deadband_pct, brake_db=ff.brake_deadband_pct,
    )


if __name__ == "__main__":
    raise SystemExit(main())
