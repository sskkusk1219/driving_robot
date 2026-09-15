"""feedforward パラメータがモード走行でどう作用するかを解析する。

ProblemReport_20260912「今回実施する内容」1. 用。車両・アクチュエータには触らない。
DB から走行モード、config_testVehicle.yaml から FF を読み、
    A. 走行モードの基準車速だけを FF に通す（オフライン。車両なし）
    B. 手順 3 の走行 CSV の各行を、そのとき FF がどの分岐で effort を出したかで分類する
    C. feedforward の各パラメータを 1 つずつ変え、effort がどれだけ変わるかを見る（感度）
を出す。図は --out に保存し、レポート（reportYYYYMMDD_explanationFF.md）に使う表をターミナルに出す。

    .venv/bin/python -m tests.research.ff_explain \
        --csv tests/research/results/drive_log_real_20260911_171637.csv

FF の分岐は本番 FeedforwardController.predict_effort（src/domain/control/feedforward.py。手順 3 の
mode_drive.py が呼んでいるもの）を 1 行ずつ写したもの。写し間違いが無いことを、全サンプルで
predict_effort と一致するか確かめてから集計する。
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import pickle
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from src.domain.control.feedforward import FeedforwardController
from src.domain.model_training import STOP_SPEED_KMH, FeatureSpec, build_feature_row
from src.models.profile import FeedforwardParams, coast_decel_at, pedal_gain_at
from tests.research.config import DEFAULT_CONFIG_PATH, ResearchConfig, load_config
from tests.research.drive_log import SECTION_MODE_DRIVE, cmd_opening, ff_effort
from tests.research.live_plot import (
    COLOR_ACCEL,
    COLOR_ACTUAL,
    COLOR_BRAKE,
    COLOR_REF,
    FONT_FAMILY,
)
from tests.research.mode_drive import ReferenceSpeed, load_mode
from tests.research.vehicle import feedforward_params

Floats = Sequence[float] | np.ndarray

# FF の分岐（predict_effort の return の数だけある）
BR_STOP = "停車保持"
BR_CREEP = "クリープ任せ"
BR_ACCEL = "駆動"
BR_TAPER = "惰行テーパ"
BR_BRAKE = "制動"
BR_LOW_BRAKE = "低速制動"
BRANCHES = (BR_STOP, BR_CREEP, BR_ACCEL, BR_TAPER, BR_BRAKE, BR_LOW_BRAKE)
BRANCH_COLORS = {
    BR_STOP: "#9e9e9e",
    BR_CREEP: "#b39ddb",
    BR_ACCEL: "#4fa3e0",
    BR_TAPER: "#6cc06c",
    BR_BRAKE: "#f07070",
    BR_LOW_BRAKE: "#a33a3a",
}
COLOR_COAST = "#2e7d32"
DT_S = 0.1  # 解析の刻み（CSV と同じ）
SLOPE_HALF_WINDOW_S = 0.5  # 実車速・基準車速の傾きを出す窓の半幅 [s]
EFFORT_CHANGED_PCT = 0.05  # 感度解析で「effort が変わった」とみなす差 [%]


# ─────────────────────────────────────────────────────────────────────
# FF の再現（predict_effort を分岐ごとに分解）
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class FFModel:
    """手順 2 の pkl（train_inverse_model の出力）。"""

    path: str
    accel_model: Any
    brake_model: Any
    spec: FeatureSpec
    speed_clip_max: float | None


def load_ff_model(path: str) -> FFModel:
    with open(path, "rb") as f:
        data = pickle.load(f)  # noqa: S301 - 手順 2 で作った信頼済みファイル
    clip = data.get("speed_clip_max")
    return FFModel(
        path=path,
        accel_model=data["accel_model"],
        brake_model=data["brake_model"],
        spec=FeatureSpec(**data["feature_spec"]),
        speed_clip_max=float(clip) if clip is not None else None,
    )


@dataclass
class ModelOutputs:
    """各点のモデル入力と出力（feedforward のパラメータに依らない部分）。"""

    t: np.ndarray
    v0_raw: np.ndarray  # クリップ前の基準車速（停車判定に使う）
    near: np.ndarray  # 最短ホライズン（0.5s 先）の基準車速（停車判定に使う）
    v0: np.ndarray  # 学習域クリップ後の v0（分岐判定に使う）
    a_req: np.ndarray  # レジーム判定の要求加速度 = (1.0s 先 − 現在) / 1.0 [km/h/s]
    accel_raw: np.ndarray  # アクセルモデルの出力（0 クランプ前）[%]
    brake_raw: np.ndarray  # ブレーキモデルの出力（0 クランプ前）[%]


def outputs_from_points(
    model: FFModel,
    t: Floats,
    v0s: Floats,
    futures: Sequence[Sequence[float]],
    pasts: Sequence[Sequence[float]],
) -> ModelOutputs:
    """predict_effort と同じ手順（学習域クリップ → build_feature_row）で特徴を作り、推論する。"""
    spec, cm = model.spec, model.speed_clip_max
    rows, clipped = [], []
    for v_ref, future, past in zip(v0s, futures, pasts, strict=True):
        v0, fut, pst = float(v_ref), list(future), list(past)
        if cm is not None:
            if v0 > cm:
                shift = v0 - cm
                v0 = cm
                fut = [f - shift for f in fut]
                pst = [p - shift for p in pst]
            fut = [min(f, cm) for f in fut]
            pst = [min(p, cm) for p in pst]
        rows.append(build_feature_row(v0, fut, pst, spec)[0])
        clipped.append(v0)
    x = np.array(rows)
    return ModelOutputs(
        t=np.asarray(t, dtype=float),
        v0_raw=np.asarray(v0s, dtype=float),
        near=np.array([f[0] for f in futures], dtype=float),
        v0=np.array(clipped),
        a_req=x[:, spec.regime_col()] / spec.regime_horizon_s,
        accel_raw=np.asarray(model.accel_model.predict(x), dtype=float),
        brake_raw=np.asarray(model.brake_model.predict(x), dtype=float),
    )


def outputs_for_mode(model: FFModel, ref: ReferenceSpeed, times: Floats) -> ModelOutputs:
    spec = model.spec
    return outputs_from_points(
        model,
        times,
        [ref.at(t) for t in times],
        [[ref.at(t + h) for h in spec.lookahead_horizons_s] for t in times],
        [[ref.at(t - h) for h in spec.past_horizons_s] for t in times],
    )


def decide(
    p: FeedforwardParams,
    v0_raw: float,
    near: float,
    v0: float,
    a_req: float,
    accel_pred: float,
    brake_pred: float,
) -> tuple[str, float]:
    """predict_effort の分岐をそのまま写す。(分岐名, effort [%]) を返す。"""
    if v0_raw <= STOP_SPEED_KMH and near <= STOP_SPEED_KMH:
        return BR_STOP, -max(0.0, min(100.0, p.stop_brake_opening_pct))
    if a_req >= 0.0:
        if v0 < p.creep_speed_kmh and a_req <= p.creep_rate_kmhs:
            branch, effort = BR_CREEP, 0.0
        else:
            branch, effort = BR_ACCEL, accel_pred
    elif v0 >= p.creep_speed_kmh:
        eng = coast_decel_at(p, v0)
        if eng > 0.0 and -a_req <= eng:
            branch, effort = BR_TAPER, accel_pred * (1.0 - (-a_req) / eng)
        else:
            branch, effort = BR_BRAKE, -brake_pred
    else:
        branch, effort = BR_LOW_BRAKE, -brake_pred
    return branch, max(-100.0, min(100.0, effort))


@dataclass
class Trace:
    out: ModelOutputs
    coast: np.ndarray  # coast_decel_at(v0) [km/h/s]（正値）
    branch: np.ndarray
    effort: np.ndarray


def decide_all(p: FeedforwardParams, out: ModelOutputs) -> Trace:
    n = len(out.t)
    branches: list[str] = []
    effort = np.empty(n)
    for i in range(n):
        b, e = decide(
            p,
            float(out.v0_raw[i]),
            float(out.near[i]),
            float(out.v0[i]),
            float(out.a_req[i]),
            max(0.0, float(out.accel_raw[i])),
            max(0.0, float(out.brake_raw[i])),
        )
        branches.append(b)
        effort[i] = e
    coast = np.array([coast_decel_at(p, float(v)) for v in out.v0])
    return Trace(out=out, coast=coast, branch=np.array(branches), effort=effort)


def verify_against_production(
    p: FeedforwardParams, model: FFModel, ref: ReferenceSpeed, trace: Trace
) -> float:
    """写した分岐が本番 predict_effort と一致するか。最大の差 [%] を返す。"""
    ff = FeedforwardController()
    ff.set_params(p)
    ff.load_model(model.path)
    worst = 0.0
    for t, e in zip(trace.out.t, trace.effort, strict=True):
        future = [ref.at(t + h) for h in ff.horizons]
        past = [ref.at(t - h) for h in ff.past_horizons]
        worst = max(worst, abs(ff.predict_effort(ref.at(t), future, past) - e))
    return worst


# ─────────────────────────────────────────────────────────────────────
# 集計
# ─────────────────────────────────────────────────────────────────────


def md_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> str:
    lines = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    lines += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(lines)


def pedal_class(accel: np.ndarray, brake: np.ndarray, p: FeedforwardParams) -> np.ndarray:
    """不感帯を超えているかで A（実効アクセル）/ B（実効ブレーキ）/ -（実質惰行）に分ける。"""
    return np.where(
        accel >= p.accel_deadband_pct, "A", np.where(brake >= p.brake_deadband_pct, "B", "-")
    )


def effort_to_pedals(effort: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    return np.clip(effort, 0.0, None), np.clip(-effort, 0.0, None)


def slope(t: np.ndarray, y: np.ndarray) -> np.ndarray:
    h = SLOPE_HALF_WINDOW_S
    return (np.interp(t + h, t, y) - np.interp(t - h, t, y)) / (2.0 * h)


def _share(mask: np.ndarray) -> str:
    return f"{100 * np.mean(mask):.0f}%"


def offline_tables(trace: Trace, p: FeedforwardParams, cfg: ResearchConfig) -> str:
    out, br, eff = trace.out, trace.branch, trace.effort
    cls = pedal_class(*effort_to_pedals(eff), p)
    total = len(br)
    effective = {BR_ACCEL: "A", BR_TAPER: "A", BR_BRAKE: "B", BR_LOW_BRAKE: "B"}
    raw = {BR_ACCEL: out.accel_raw, BR_TAPER: out.accel_raw,
           BR_BRAKE: out.brake_raw, BR_LOW_BRAKE: out.brake_raw}
    rows = []
    for name in BRANCHES:
        m = br == name
        n = int(m.sum())
        if n == 0:
            continue
        e = eff[m]
        rows.append([
            name,
            f"{n * DT_S:.0f}",
            f"{100 * n / total:.1f}%",
            f"{np.percentile(e, 50):+.1f}",
            f"{np.percentile(np.abs(e), 95):.1f}",
            f"{np.max(np.abs(e)):.1f}",
            _share(cls[m] != effective[name]) if name in effective else "—",
            _share(raw[name][m] < 0.0) if name in raw else "—",
        ])
    text = [
        "### A-1 分岐ごとの時間と指令（WLTP 1800s・基準車速だけ）",
        md_table(
            ["分岐", "時間[s]", "割合", "effort p50[%]", "|effort| p95[%]",
             "|effort| 最大[%]", "不感帯より浅い", "モデル出力が負→0"],
            rows,
        ),
    ]

    seg = np.array([cfg.modes.segment_at(t) for t in out.t])
    names = cfg.modes.segment_names
    seg_rows = [[name, *(_share(br[seg == s] == name) for s in names)] for name in BRANCHES]
    text += ["### A-2 WLTP 区間ごとの分岐の割合", md_table(["分岐", *names], seg_rows)]

    trans: Counter[tuple[str, str]] = Counter()
    jumps: dict[tuple[str, str], list[float]] = {}
    for i in range(1, total):
        if br[i] != br[i - 1]:
            key = (str(br[i - 1]), str(br[i]))
            trans[key] += 1
            jumps.setdefault(key, []).append(abs(eff[i] - eff[i - 1]))
    trans_rows = [
        [f"{a} → {b}", c, f"{np.mean(jumps[(a, b)]):.1f}", f"{np.max(jumps[(a, b)]):.1f}"]
        for (a, b), c in trans.most_common(10)
    ]
    text += [
        "### A-3 分岐の切り替わり（多い順）と、その 1 刻み（0.1s）での effort の跳び",
        md_table(["切り替わり", "回数", "|Δeffort| 平均[%]", "|Δeffort| 最大[%]"], trans_rows),
    ]
    return "\n\n".join(text)


@dataclass
class RunRows:
    t: np.ndarray
    ref: np.ndarray
    actual: np.ndarray
    accel: np.ndarray
    brake: np.ndarray
    ff_effort: np.ndarray




def read_run(path: Path) -> RunRows:
    """モード走行の行を読む（新形式 A7〜・旧形式 〜A6 のどちらの CSV も）。"""
    cols: list[list[float]] = [[], [], [], [], [], []]
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("section") != SECTION_MODE_DRIVE or not r.get("mode_time_s"):
                continue
            values = (
                float(r["mode_time_s"]),
                float(r["ref_speed_kmh"]) if r["ref_speed_kmh"] else 0.0,
                float(r["actual_speed_kmh"]) if r["actual_speed_kmh"] else 0.0,
                cmd_opening(r, "accel"),
                cmd_opening(r, "brake"),
                ff_effort(r) or 0.0,
            )
            for col, value in zip(cols, values, strict=True):
                col.append(value)
    return RunRows(*(np.array(c) for c in cols))


def run_tables(run: RunRows, trace: Trace, p: FeedforwardParams) -> str:
    br = trace.branch
    total = len(br)
    cls = pedal_class(run.accel, run.brake, p)
    dev = run.actual - run.ref
    acc_err = slope(run.t, run.actual) - slope(run.t, run.ref)
    rows = []
    for name in BRANCHES:
        m = br == name
        n = int(m.sum())
        if n == 0:
            continue
        rows.append([
            name,
            f"{n * DT_S:.0f}",
            f"{100 * n / total:.1f}%",
            _share(cls[m] == "A"),
            _share(cls[m] == "B"),
            _share(cls[m] == "-"),
            f"{np.mean(dev[m]):+.2f}",
            f"{np.percentile(np.abs(dev[m]), 95):.2f}",
            f"{np.mean(acc_err[m]):+.2f}",
        ])
    text = [
        "### B-1 実走行（手順 3）の分岐ごとの結果",
        md_table(
            ["分岐", "時間[s]", "割合", "実効アクセル", "実効ブレーキ", "実質惰行",
             "平均偏差[km/h]", "|偏差| p95[km/h]", "加速度誤差 平均[km/h/s]"],
            rows,
        ),
        "加速度誤差 = 実車速の傾き − 基準車速の傾き（前後 0.5s）。偏差は積分量で前の分岐の"
        "遅れを持ち越すので、分岐そのものの良し悪しは加速度誤差で見る。",
    ]

    # 惰行カーブの妥当性: 両ペダルが 1s 以上続けて不感帯の中にあった点で、実測減速を曲線と比べる
    off = cls == "-"
    back, ahead = round(1.0 / DT_S), round(SLOPE_HALF_WINDOW_S / DT_S)
    steady = np.array([off[max(0, i - back): i + ahead + 1].all() for i in range(total)])
    mask = steady & (run.actual > p.creep_speed_kmh)
    decel = -slope(run.t, run.actual)
    coast_rows = []
    for lo in range(0, 140, 10):
        mb = mask & (run.actual >= lo) & (run.actual < lo + 10)
        if mb.sum() < 10:
            continue
        coast_rows.append([
            f"{lo}〜{lo + 10}",
            f"{mb.sum() * DT_S:.1f}",
            f"{np.median(decel[mb]):.2f}",
            f"{coast_decel_at(p, lo + 5.0):.2f}",
        ])
    text += [
        "### B-2 惰行カーブの確認（実走行で両ペダルが 1s 以上不感帯の中にあった点）",
        md_table(
            ["実車速[km/h]", "時間[s]", "実測の減速 中央値[km/h/s]", "coast_decel カーブ[km/h/s]"],
            coast_rows,
        ),
    ]
    return "\n\n".join(text)


def perturbations(p: FeedforwardParams) -> list[tuple[str, str, FeedforwardParams]]:
    def scaled(values: tuple[float, ...], k: float) -> tuple[float, ...]:
        return tuple(v * k for v in values)

    stop, creep, rate = p.stop_brake_opening_pct, p.creep_speed_kmh, p.creep_rate_kmhs
    eng = p.engine_brake_decel_kmhs
    return [
        ("stop_brake_opening_pct", "×0.8", replace(p, stop_brake_opening_pct=stop * 0.8)),
        ("stop_brake_opening_pct", "×1.2", replace(p, stop_brake_opening_pct=stop * 1.2)),
        ("creep_speed_kmh", "×0.5", replace(p, creep_speed_kmh=creep * 0.5)),
        ("creep_speed_kmh", "×1.5", replace(p, creep_speed_kmh=creep * 1.5)),
        ("creep_rate_kmhs", "×0.5", replace(p, creep_rate_kmhs=rate * 0.5)),
        ("creep_rate_kmhs", "×2", replace(p, creep_rate_kmhs=rate * 2.0)),
        ("coast_decel_kmhs", "×0.8", replace(p, coast_decel_kmhs=scaled(p.coast_decel_kmhs, 0.8))),
        ("coast_decel_kmhs", "×1.2", replace(p, coast_decel_kmhs=scaled(p.coast_decel_kmhs, 1.2))),
        ("coast_decel_*（曲線を空にする）", "定数 engine_brake_decel_kmhs を使う",
         replace(p, coast_decel_speeds_kmh=(), coast_decel_kmhs=())),
        ("engine_brake_decel_kmhs", "×0.5", replace(p, engine_brake_decel_kmhs=eng * 0.5)),
        ("engine_brake_decel_kmhs", "×2", replace(p, engine_brake_decel_kmhs=eng * 2.0)),
        ("accel/brake_gain_kmhs_per_pct", "×2",
         replace(p, accel_gain_kmhs_per_pct=scaled(p.accel_gain_kmhs_per_pct, 2.0),
                 brake_gain_kmhs_per_pct=scaled(p.brake_gain_kmhs_per_pct, 2.0))),
        ("accel_deadband_pct", "+5%", replace(p, accel_deadband_pct=p.accel_deadband_pct + 5.0)),
        ("brake_deadband_pct", "+5%", replace(p, brake_deadband_pct=p.brake_deadband_pct + 5.0)),
    ]


def sensitivity_table(base: Trace, p: FeedforwardParams) -> str:
    base_cls = pedal_class(*effort_to_pedals(base.effort), p)
    rows = []
    for key, change, q in perturbations(p):
        tr = decide_all(q, base.out)
        diff = np.abs(tr.effort - base.effort)
        cls = pedal_class(*effort_to_pedals(tr.effort), p)  # 不感帯は実車の値（base）で判定する
        rows.append([
            f"`{key}`",
            change,
            f"{np.sum(diff > EFFORT_CHANGED_PCT) * DT_S:.0f}",
            f"{np.sum(tr.branch != base.branch) * DT_S:.0f}",
            f"{diff.max():.1f}",
            f"{np.sum(cls != base_cls) * DT_S:.0f}",
        ])
    return "\n\n".join([
        "### C 感度（WLTP 1800s の基準車速だけ。モデル出力は同じで、分岐と合成だけが変わる）",
        md_table(
            ["パラメータ", "変更", "effort が変わった時間[s]", "分岐が変わった時間[s]",
             "|Δeffort| 最大[%]", "効くペダルが変わった時間[s]"],
            rows,
        ),
        "効くペダル = 指令が実車の不感帯（アクセル "
        f"{p.accel_deadband_pct:g}% / ブレーキ {p.brake_deadband_pct:g}%）を超えるかで決まる "
        "実効アクセル / 実効ブレーキ / 実質惰行。",
    ])


# ─────────────────────────────────────────────────────────────────────
# 図
# ─────────────────────────────────────────────────────────────────────


def _plt() -> Any:
    import matplotlib  # noqa: PLC0415 - 図を出すときだけ読み込む

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    plt.rcParams["font.family"] = FONT_FAMILY
    return plt


def _shade(ax: Any, x: np.ndarray, branch: np.ndarray, step: float = DT_S) -> None:
    start = 0
    for i in range(1, len(x) + 1):
        if i == len(x) or branch[i] != branch[start]:
            ax.axvspan(x[start], x[i - 1] + step, color=BRANCH_COLORS[str(branch[start])],
                       alpha=0.22, linewidth=0)
            start = i


def _branch_patches() -> list[Any]:
    from matplotlib.patches import Patch  # noqa: PLC0415

    return [Patch(color=BRANCH_COLORS[b], alpha=0.5, label=b) for b in BRANCHES]


def _deadband_lines(ax: Any, p: FeedforwardParams, *, label: bool) -> None:
    ax.axhline(p.accel_deadband_pct, color=COLOR_ACCEL, linestyle=":", linewidth=1.2,
               label="アクセル不感帯" if label else None)
    ax.axhline(-p.brake_deadband_pct, color=COLOR_BRAKE, linestyle=":", linewidth=1.2,
               label="ブレーキ不感帯（−）" if label else None)
    ax.axhline(0.0, color="#999999", linewidth=0.6)


def fig_timeline(trace: Trace, p: FeedforwardParams, cfg: ResearchConfig, path: Path) -> None:
    plt = _plt()
    t = trace.out.t
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 7), sharex=True,
                                   gridspec_kw={"height_ratios": [1.2, 1.0]})
    _shade(ax1, t, trace.branch)
    ax1.plot(t, trace.out.v0_raw, color="#333333", linewidth=1.0)
    ax1.set_ylabel("基準車速 [km/h]")
    ax1.legend(handles=_branch_patches(), loc="upper left", ncol=6, fontsize=9)
    ax2.plot(t, trace.effort, color="#333333", linewidth=0.7,
             label="FF effort（+アクセル / −ブレーキ）")
    _deadband_lines(ax2, p, label=True)
    ax2.set_ylabel("effort [%]")
    ax2.set_xlabel("モード経過時間 [s]")
    ax2.legend(loc="lower left", fontsize=9)
    for ax in (ax1, ax2):
        for b in cfg.modes.segment_bounds_s:
            ax.axvline(b, color="#555555", linestyle=":", linewidth=0.8)
        ax.grid(alpha=0.3)
    ax1.set_title("WLTP の基準車速だけを FF に通したときの分岐（背景色）と effort")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def fig_zoom(
    trace: Trace, run: RunRows | None, p: FeedforwardParams, t0: float, t1: float, path: Path
) -> None:
    plt = _plt()
    o = trace.out
    m = (o.t >= t0) & (o.t <= t1)
    t, br = o.t[m], trace.branch[m]
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    for ax in axes:
        _shade(ax, t, br)
        ax.grid(alpha=0.3)
    ax1, ax2, ax3 = axes
    ax1.plot(t, o.v0_raw[m], color=COLOR_REF, linestyle="--", linewidth=1.6, label="基準車速")
    ax2.plot(t, o.a_req[m], color="#333333", linewidth=1.4,
             label="要求加速度 a_req（1.0s 先 − 現在）")
    ax2.plot(t, -trace.coast[m], color=COLOR_COAST, linewidth=1.4,
             label="−coast_decel(v)（これより上の減速は惰行テーパ）")
    ax2.axhline(0.0, color="#999999", linewidth=0.6)
    if run is not None:
        mr = (run.t >= t0) & (run.t <= t1)
        ax1.plot(run.t[mr], run.actual[mr], color=COLOR_ACTUAL, linewidth=1.8,
                 label="実車速（手順 3）")
        ax2.plot(run.t[mr], slope(run.t, run.actual)[mr], color=COLOR_ACTUAL, linewidth=0.8,
                 alpha=0.8, label="実車速の傾き（前後 0.5s）")
    ax1.set_ylabel("車速 [km/h]")
    lines, _ = ax1.get_legend_handles_labels()
    ax1.legend(handles=lines + _branch_patches(), loc="upper left", ncol=2, fontsize=8)
    ax2.set_ylabel("加速度 [km/h/s]")
    ax2.legend(loc="lower left", fontsize=8)

    ax3.plot(t, np.clip(o.accel_raw[m], 0, None), color=COLOR_ACCEL, linestyle="--",
             linewidth=1.0, label="アクセルモデル出力")
    ax3.plot(t, -np.clip(o.brake_raw[m], 0, None), color=COLOR_BRAKE, linestyle="--",
             linewidth=1.0, label="−ブレーキモデル出力")
    ax3.plot(t, trace.effort[m], color="#222222", linewidth=1.6, label="FF effort（最終）")
    _deadband_lines(ax3, p, label=True)
    ax3.set_ylabel("effort [%]")
    ax3.set_xlabel("モード経過時間 [s]")
    ax3.legend(loc="lower left", fontsize=8, ncol=3)
    ax1.set_title(f"FF の分岐の拡大（{t0:.0f}〜{t1:.0f}s）")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def fig_decision_map(trace: Trace, p: FeedforwardParams, path: Path) -> None:
    plt = _plt()
    o = trace.out
    fig, axes = plt.subplots(1, 2, figsize=(15, 6), gridspec_kw={"width_ratios": [2.2, 1.0]})
    v = np.linspace(0.0, 140.0, 281)
    for ax, (vmax, amin, amax) in zip(axes, ((135.0, -4.5, 4.5), (20.0, -4.5, 3.0)), strict=True):
        for name in BRANCHES[1:]:
            mm = trace.branch == name
            ax.scatter(o.v0[mm], o.a_req[mm], s=3, color=BRANCH_COLORS[name], label=name,
                       alpha=0.6)
        ax.plot(v, [-coast_decel_at(p, x) for x in v], color=COLOR_COAST, linewidth=2.0,
                label="−coast_decel(v)（惰行テーパ／制動の境目）")
        ax.axhline(-p.engine_brake_decel_kmhs, color="#777777", linestyle="--", linewidth=1.0,
                   label="−engine_brake_decel_kmhs（曲線があるので未使用）")
        ax.axvline(p.creep_speed_kmh, color=BRANCH_COLORS[BR_CREEP], linestyle="--",
                   linewidth=1.2, label="creep_speed_kmh")
        ax.plot([0.0, p.creep_speed_kmh, p.creep_speed_kmh], [p.creep_rate_kmhs] * 2 + [0.0],
                color="#5e35b1", linewidth=1.5, label="creep_rate_kmhs（クリープ任せの上限）")
        ax.axhline(0.0, color="#999999", linewidth=0.6)
        ax.set_xlim(0.0, vmax)
        ax.set_ylim(amin, amax)
        ax.set_xlabel("基準車速 v0 [km/h]")
        ax.set_ylabel("要求加速度 a_req [km/h/s]")
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8, markerscale=4)
    axes[0].set_title(
        "FF の分岐は (v0, a_req) の平面で決まる（点 = WLTP の 0.1s ごと、停車保持を除く）"
    )
    axes[1].set_title("低速域の拡大")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def fig_curves(p: FeedforwardParams, path: Path) -> None:
    plt = _plt()
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5))
    ax = axes[0]
    ax.plot(p.coast_decel_speeds_kmh, p.coast_decel_kmhs, "o-", color=COLOR_COAST,
            label="coast_decel_kmhs（使用）")
    ax.axhline(p.engine_brake_decel_kmhs, color="#777777", linestyle="--",
               label=f"engine_brake_decel_kmhs={p.engine_brake_decel_kmhs:g}（未使用）")
    ax.set_title("惰行減速カーブ（分岐の境目になる）")
    ax.set_ylabel("惰行減速 [km/h/s]")
    for ax, values, color, label in (
        (axes[1], p.accel_gain_kmhs_per_pct, COLOR_ACCEL, "accel_gain_kmhs_per_pct"),
        (axes[2], p.brake_gain_kmhs_per_pct, COLOR_BRAKE, "brake_gain_kmhs_per_pct"),
    ):
        ax.plot(p.pedal_gain_speeds_kmh, values, "o-", color=color, label=label)
        ax.set_title(f"{label}\n（手順 3 のモード走行では未使用）")
        ax.set_ylabel("ゲイン [km/h/s per %]")
    for ax in axes:
        ax.set_xlabel("車速 [km/h]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


SLICE_SPEEDS_KMH = (15.0, 35.0, 60.0, 90.0, 120.0)
SLICE_STEP_KMHS = 0.02


def accel_grid(lo: float, hi: float) -> np.ndarray:
    return np.round(np.arange(lo, hi + SLICE_STEP_KMHS / 2, SLICE_STEP_KMHS), 3)


def constant_accel_trace(
    model: FFModel, p: FeedforwardParams, v: float, accels: np.ndarray
) -> Trace:
    """速度 v を一定加速度 a で通過中の軌跡（先 = v + a·h、過去 = v − a·h）を FF に通す。"""
    spec = model.spec
    out = outputs_from_points(
        model,
        accels,
        [v] * len(accels),
        [[max(0.0, v + x * h) for h in spec.lookahead_horizons_s] for x in accels],
        [[max(0.0, v - x * h) for h in spec.past_horizons_s] for x in accels],
    )
    return decide_all(p, out)


def flat_zone_table(model: FFModel, p: FeedforwardParams) -> str:
    """指令が両ペダルの不感帯の中に落ちる（＝何を要求しても惰行になる）a_req の範囲。"""
    a = accel_grid(-6.0, 2.0)
    rows = []
    for v in SLICE_SPEEDS_KMH:
        cls = pedal_class(*effort_to_pedals(constant_accel_trace(model, p, v, a).effort), p)
        coast = cls == "-"
        brake_on, accel_on = a[cls == "B"], a[cls == "A"]
        if coast.any():
            first, last = int(np.argmax(coast)), len(coast) - int(np.argmax(coast[::-1]))
            gap = "" if coast[first:last].all() else "（途切れあり）"
            zone = f"{a[coast].min():+.2f} 〜 {a[coast].max():+.2f}{gap}"
            width = f"{a[coast].max() - a[coast].min():.2f}"
        else:
            zone, width = "なし", "0"
        rows.append([
            f"{v:g}",
            f"{-coast_decel_at(p, v):+.2f}",
            zone,
            width,
            f"{brake_on.max():+.2f}" if brake_on.size else "−6 まで効かない",
            f"{accel_on.min():+.2f}" if accel_on.size else "—",
        ])
    return "\n\n".join([
        "### D 不感帯で実質惰行になる要求加速度（一定加速度で走る軌跡を FF に通した場合）",
        md_table(
            ["v0[km/h]", "惰行の加速度 −coast_decel(v)[km/h/s]", "実質惰行になる a_req[km/h/s]",
             "幅[km/h/s]", "ブレーキが効く a_req（これ以下）", "アクセルが効く a_req（これ以上）"],
            rows,
        ),
        "この範囲の要求はどれも両ペダルが効かず、車両は −coast_decel(v) の加速度で惰行する。",
    ])


def hold_opening_table(model: FFModel, p: FeedforwardParams) -> str:
    """同定した惰行カーブとアクセルゲインから、定速に要る開度を出して FF の定速出力と比べる。"""
    db = p.accel_deadband_pct
    zero = np.array([0.0])
    rows = []
    for v in p.pedal_gain_speeds_kmh:
        gain = pedal_gain_at(p, v, is_accel=True)
        if gain is None:
            continue
        u_hold = db + coast_decel_at(p, v) / gain
        gain_up = pedal_gain_at(p, v + 10.0, is_accel=True) or gain
        a_up = gain_up * (u_hold - db) - coast_decel_at(p, v + 10.0)
        rows.append([
            f"{v:g}",
            f"{coast_decel_at(p, v):.2f}",
            f"{gain:.3f}",
            f"{u_hold:.1f}",
            f"{constant_accel_trace(model, p, v, zero).effort[0]:.1f}",
            f"{a_up:+.2f}",
        ])
    return "\n\n".join([
        "### E 定速に要るアクセル開度（同定値からの計算）と FF の定速出力",
        md_table(
            ["車速[km/h]", "coast_decel[km/h/s]", "accel_gain[km/h/s per %]",
             "定速に要る開度[%]", "FF の定速出力 a_req=0 [%]",
             "同じ開度のまま +10 km/h での加速度[km/h/s]"],
            rows,
        ),
        "定速に要る開度 = accel_deadband_pct + coast_decel(v) ÷ accel_gain(v)。"
        "右端が + なら、速度が上がるほど加速が強まる（開度一定では速度のずれが広がる向き）。",
    ])


def fig_slices(model: FFModel, p: FeedforwardParams, path: Path) -> None:
    """一定加速度の軌跡での effort を a_req の関数として描く。"""
    plt = _plt()
    a = accel_grid(-4.0, 2.0)
    fig, axes = plt.subplots(1, len(SLICE_SPEEDS_KMH), figsize=(20, 4.8), sharey=True)
    for ax, v in zip(axes, SLICE_SPEEDS_KMH, strict=True):
        tr = constant_accel_trace(model, p, v, a)
        out = tr.out
        _shade(ax, a, tr.branch, step=SLICE_STEP_KMHS)
        ax.plot(a, np.clip(out.accel_raw, 0, None), color=COLOR_ACCEL, linestyle="--",
                label="アクセルモデル出力")
        ax.plot(a, -np.clip(out.brake_raw, 0, None), color=COLOR_BRAKE, linestyle="--",
                label="−ブレーキモデル出力")
        ax.plot(a, tr.effort, color="#222222", linewidth=2.0, label="FF effort")
        ax.axvline(-coast_decel_at(p, v), color=COLOR_COAST, linewidth=1.2,
                   label="−coast_decel(v)")
        _deadband_lines(ax, p, label=False)
        ax.set_title(f"v0 = {v:g} km/h")
        ax.set_xlabel("a_req [km/h/s]")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("effort [%]（点線 = 不感帯）")
    axes[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("一定加速度 a_req で走る軌跡を入れたときの FF 出力（背景色 = 分岐）")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────
# main
# ─────────────────────────────────────────────────────────────────────


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--csv", type=Path, default=None,
                    help="手順 3 の走行 CSV（省略すると B を出さない）")
    ap.add_argument("--out", type=Path, default=None,
                    help="図の保存先（既定: results/report<今日>_explanationFF/）")
    ap.add_argument("--zoom", default="20:120,860:927",
                    help="拡大図の区間 [s]（開始:終了 をカンマ区切り）")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    p = feedforward_params(cfg)
    out_dir = args.out or cfg.results_path / f"report{datetime.now():%Y%m%d}_explanationFF"
    out_dir.mkdir(parents=True, exist_ok=True)
    model = load_ff_model(cfg.feedforward.model_path)
    mode = asyncio.run(load_mode(cfg, cfg.modes.wltp_mode_name))
    ref = ReferenceSpeed(mode)
    print(f"# FF 解析: {cfg.feedforward.model_path} / {mode.name}"
          f"（{mode.total_duration:.0f}s・最高 {mode.max_speed:.1f} km/h）")
    print(f"- 特徴量: {model.spec.feature_names()}・学習域クリップ {model.speed_clip_max}")

    times = np.round(np.arange(0.0, mode.total_duration + DT_S / 2, DT_S), 3)
    trace = decide_all(p, outputs_for_mode(model, ref, times))
    worst = verify_against_production(p, model, ref, trace)
    print(f"- 本番 predict_effort との最大差（{len(times)} 点）: {worst:.2e} %")
    print()
    print(offline_tables(trace, p, cfg))
    print()
    print(sensitivity_table(trace, p))
    print()
    print(flat_zone_table(model, p))
    print()
    print(hold_opening_table(model, p))
    fig_timeline(trace, p, cfg, out_dir / "branch_timeline.png")
    fig_decision_map(trace, p, out_dir / "decision_map.png")
    fig_curves(p, out_dir / "param_curves.png")
    fig_slices(model, p, out_dir / "effort_slices.png")

    run = None
    if args.csv is not None:
        run = read_run(args.csv)
        run_trace = decide_all(p, outputs_for_mode(model, ref, run.t))
        diff = np.abs(run_trace.effort - run.ff_effort)
        print()
        print(f"- 走行 CSV: {args.csv}（{len(run.t)} 行・{run.t[-1]:.1f}s）。"
              f"CSV の ff_effort_pct と再計算の差: 最大 {diff.max():.3f} %（CSV の丸め）")
        print()
        print(run_tables(run, run_trace, p))
    for span in args.zoom.split(","):
        t0, t1 = (float(x) for x in span.split(":"))
        fig_zoom(trace, run, p, t0, t1, out_dir / f"zoom_{t0:.0f}_{t1:.0f}.png")
    print()
    print(f"- 図: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
