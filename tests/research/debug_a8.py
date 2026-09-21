"""A8: 手順 3 の C1・C5・C4 各 3 本（9/15 実機）の比較レポート用の表・図を作る。

車両には触らない。9 本の走行ログ CSV（MODE_DRIVE の行）と、モデル pkl を読むだけ。
表（Markdown）をターミナルに出し、図を results/report20260916_..._A8/ に保存する。

    .venv/bin/python -m tests.research.debug_a8

出すもの（`report20260916_debag_process2,3_A8.md` の章と対応）:
    summary/aggregate/band  compare_runs と同じ関数で 1 本ごと・候補ごと・帯別の表
    reached                 到達時間と走破判定（151421 が「未走破」になった理由）
    idle_creep              基準車速 0 の区間で実車速が止まっていない率（停車保持の効き）
    switches                ACCEL→BRAKE 切り替え（基準 > 5 km/h）の一段開度・完全停止・最大減速
    reversal                C5 の符号反転が集中する 5s ブロック
    gain                    C5 の実質ゲイン（速度で変わる）と、Kp 注入を要求加速度に足した場合の比較
    cruise                  モデルの定速開度 vs 車速（C4 に戻す力が無い理由）
    excerpts                C4（164819）の暴走区間のログ抜粋
    fig1/2/3                図（速度重ね描き・帯別偏差・定速開度）
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib
import numpy as np

from tests.research.compare_runs import (
    RunResult,
    aggregate_table,
    band_table,
    completion_threshold_s,
    compute_band_stats,
    evaluate_run,
    group_by_label,
    median_band_stats,
    rank_groups,
    summary_table,
)
from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.debug_process23 import md_table
from tests.research.kpi import sample_interval_s
from tests.research.live_plot import COLOR_ACCEL, COLOR_ACTUAL, COLOR_BRAKE, COLOR_REF, FONT_FAMILY
from tests.research.mode_drive import load_feedforward
from tests.research.mode_report import ModeRow

RESULTS = Path("tests/research/results")
OUT_DIR = RESULTS / "report20260916_debag_process2,3_A8"
DURATION_S = 1800.0  # WLTP の総時間（compare_runs の既定と同じ）

#: 9 本の CSV（9/15。候補ごと 3 本、走った順）
RUNS: tuple[tuple[str, str], ...] = (
    ("C1", "drive_log_real_20260915_132814.csv"),
    ("C1", "drive_log_real_20260915_135952.csv"),
    ("C1", "drive_log_real_20260915_143757.csv"),
    ("C5", "drive_log_real_20260915_151421.csv"),
    ("C5", "drive_log_real_20260915_154610.csv"),
    ("C5", "drive_log_real_20260915_161718.csv"),
    ("C4", "drive_log_real_20260915_164819.csv"),
    ("C4", "drive_log_real_20260915_171938.csv"),
    ("C4", "drive_log_real_20260915_181645.csv"),
)

# ─────────────────────────────────────────────────────────────────────
# 到達時間・走破判定
# ─────────────────────────────────────────────────────────────────────


def reached_table(results: Sequence[RunResult]) -> str:
    body = []
    for r in results:
        dt = sample_interval_s([row.t_s for row in r.rows])
        threshold = completion_threshold_s(DURATION_S, [row.t_s for row in r.rows])
        body.append([
            r.label, r.csv_path.name, f"{r.reached_s:.3f}", f"{dt:.3f}", f"{threshold:.3f}",
            "○" if r.completed else "×",
        ])
    return md_table(
        ["ラベル", "CSV", "到達[s]", "行間隔 中央値[s]", "走破のしきい値[s]", "走破"], body
    )


# ─────────────────────────────────────────────────────────────────────
# 停車保持（基準車速 0 の区間）
# ─────────────────────────────────────────────────────────────────────


def idle_creep_table(groups: Sequence[Sequence[RunResult]], labels: Sequence[str]) -> str:
    body = []
    for label, runs in zip(labels, groups, strict=True):
        rows = [row for r in runs for row in r.rows if row.ref_kmh == 0.0]
        n = len(rows)
        frac = sum(row.actual_kmh > 0.5 for row in rows) / n if n else 0.0
        max_actual = max((row.actual_kmh for row in rows), default=0.0)
        mean_brake = float(np.mean([row.brake_pct for row in rows])) if n else 0.0
        body.append([
            label, str(n), f"{100 * frac:.0f}%", f"{max_actual:.2f}", f"{mean_brake:.2f}",
        ])
    return md_table(
        ["候補", "基準0の行数", "実車速>0.5km/hの割合", "実車速の最大[km/h]",
         "ブレーキ指令の平均[%]"],
        body,
    )


# ─────────────────────────────────────────────────────────────────────
# ACCEL→BRAKE 切り替え（基準 > 5 km/h）
# ─────────────────────────────────────────────────────────────────────

SWITCH_REF_MIN_KMH = 5.0  # これ以下の基準車速での切り替えは数えない（停車前後の細かい往復を除く）
SWITCH_WINDOW_S = 1.5  # 切り替え後、この秒数の窓で完全停止・最大減速を見る
SWITCH_STOP_KMH = 1.0  # 実車速がこれ未満なら「完全停止」


@dataclass(frozen=True)
class SwitchEvent:
    t_s: float
    ref_kmh: float
    brake_step_pct: float  # 切り替え直後 1 周期目のブレーキ開度 − 直前のブレーキ開度
    complete_stop: bool
    max_decel_kmhs: float  # 窓内で隣り合う 2 行の差から出した減速度の最大（1 周期の跳びを見る）


def accel_to_brake_switches(rows: Sequence[ModeRow]) -> list[SwitchEvent]:
    out = []
    for i in range(1, len(rows)):
        prev, cur = rows[i - 1], rows[i]
        if prev.phase != "ACCEL" or cur.phase not in ("BRAKE", "BRAKE_GOV"):
            continue
        if cur.ref_kmh <= SWITCH_REF_MIN_KMH:
            continue
        window = [row for row in rows[i:] if row.t_s - cur.t_s <= SWITCH_WINDOW_S]
        decels = [
            (a.actual_kmh - b.actual_kmh) / (b.t_s - a.t_s)
            for a, b in zip(window, window[1:], strict=False)
            if b.t_s > a.t_s
        ]
        out.append(SwitchEvent(
            t_s=cur.t_s, ref_kmh=cur.ref_kmh, brake_step_pct=cur.brake_pct - prev.brake_pct,
            complete_stop=any(row.actual_kmh < SWITCH_STOP_KMH for row in window),
            max_decel_kmhs=max(decels, default=0.0),
        ))
    return out


def switches_table(results: Sequence[RunResult]) -> str:
    body = []
    for r in results:
        evs = accel_to_brake_switches(r.rows)
        steps = [e.brake_step_pct for e in evs]
        decels = [e.max_decel_kmhs for e in evs]
        stops = sum(e.complete_stop for e in evs)
        body.append([
            r.label, r.csv_path.name, str(len(evs)), str(stops),
            f"{float(np.median(steps)):.2f}" if steps else "—",
            f"{max(steps, default=0.0):.2f}",
            f"{float(np.median(decels)):.2f}" if decels else "—",
            f"{max(decels, default=0.0):.2f}",
        ])
    return md_table(
        ["候補", "CSV", "切替回数(基準>5km/h)", "完全停止(<1km/h)", "一段開度 中央値[%]",
         "一段開度 最大[%]", "最大減速 中央値[km/h/s]", "最大減速 最大[km/h/s]"],
        body,
    )


# ─────────────────────────────────────────────────────────────────────
# C5 の符号反転の集中（5s ブロック）
# ─────────────────────────────────────────────────────────────────────

REVERSAL_BAND_KMH = 0.3  # kpi.reversal_band_kmh と同じ
REVERSAL_BLOCK_S = 5.0


def _reversal_events(rows: Sequence[ModeRow]) -> list[float]:
    """偏差が ±REVERSAL_BAND_KMH を両側で超えて符号が入れ替わった時刻。

    kpi.reversal_max と同じ数え方（帯の中は直前の符号を保持）。
    """
    events = []
    last_sign = 0
    for row in rows:
        dev = row.deviation_kmh
        sign = 1 if dev > REVERSAL_BAND_KMH else -1 if dev < -REVERSAL_BAND_KMH else 0
        if sign == 0:
            continue
        if last_sign != 0 and sign != last_sign:
            events.append(row.t_s)
        last_sign = sign
    return events


def reversal_hotspot_table(results: Sequence[RunResult]) -> str:
    body = []
    for r in results:
        events = _reversal_events(r.rows)
        n_blocks = int(r.reached_s // REVERSAL_BLOCK_S)
        counts: dict[int, int] = {}
        for t in events:
            b = int(t // REVERSAL_BLOCK_S)
            counts[b] = counts.get(b, 0) + 1
        hot = sum(1 for c in counts.values() if c > 1)
        ref_at_max = next(
            (row.ref_kmh for row in r.rows if row.t_s == r.kpi.reversal_max_t_s), None
        )
        body.append([
            r.label, r.csv_path.name, f"{hot} / {n_blocks}", f"{100 * hot / n_blocks:.1f}%",
            f"{r.kpi.reversal_max_per_window}",
            "—" if r.kpi.reversal_max_t_s is None else f"{r.kpi.reversal_max_t_s:.1f}",
            "—" if ref_at_max is None else f"{ref_at_max:.1f}",
        ])
    return md_table(
        ["候補", "CSV", "反転>1のブロック", "割合", "符号反転(窓内最大)", "最大に達した時刻[s]",
         "そのときの基準車速[km/h]"],
        body,
    )


# ─────────────────────────────────────────────────────────────────────
# C5 の実質ゲイン・Kp 注入との比較・モデルの定速開度
# ─────────────────────────────────────────────────────────────────────

GAIN_SPEEDS_KMH: tuple[float, ...] = (20.0, 40.0, 60.0, 90.0, 120.0)
CRUISE_SPEEDS_KMH: tuple[float, ...] = (20.0, 40.0, 60.0, 75.0, 90.0, 110.0, 120.0, 131.0, 140.0,
                                        160.0)
FIG3_SPEEDS_KMH = np.arange(20.0, 160.01, 2.0)


def gain_table(ff: Any) -> str:
    """C5 の実質ゲイン（1 km/h 遅れたときの開度の増分）と、Kp 注入（要求加速度に足す）との比較。"""
    n_future, n_past = len(ff.horizons), len(ff.past_horizons)
    body = []
    for v in GAIN_SPEEDS_KMH:
        base = ff.predict_effort(v, [v] * n_future, [v] * n_past)
        c5_gain = ff.predict_effort(v - 1.0, [v] * n_future, [v - 1.0] * n_past) - base
        kp_inject = ff.predict_effort(v, [v + 1.0] * n_future, [v] * n_past) - base
        body.append([f"{v:.0f}", f"{c5_gain:+.3f}", f"{kp_inject:+.3f}"])
    return md_table(
        ["車速 V[km/h]", "C5 実質ゲイン（1km/h遅れ→開度増分）[%]",
         "a_req 注入（要求加速度+1km/h/s→開度増分）[%]"],
        body,
    )


def cruise_opening_table(ff: Any) -> str:
    n_future, n_past = len(ff.horizons), len(ff.past_horizons)
    body = [
        [f"{v:.0f}", f"{ff.predict_effort(v, [v] * n_future, [v] * n_past):.2f}"]
        for v in CRUISE_SPEEDS_KMH
    ]
    return md_table(["車速 V[km/h]", "定速開度（FF, v0=future=past=V）[%]"], body)


def coast_decel_table(cfg: Any) -> str:
    ff = cfg.feedforward
    body = [
        [f"{v:.0f}", f"{d:.2f}"]
        for v, d in zip(ff.coast_decel_speeds_kmh, ff.coast_decel_kmhs, strict=True)
    ]
    return md_table(["車速[km/h]", "惰行減速度[km/h/s]"], body)


# ─────────────────────────────────────────────────────────────────────
# C4 ログ抜粋
# ─────────────────────────────────────────────────────────────────────


def _nearest(rows: Sequence[ModeRow], target: float) -> ModeRow:
    return min(rows, key=lambda row: abs(row.t_s - target))


def excerpt_table(rows: Sequence[ModeRow], lo: float, hi: float, step: float) -> str:
    n = round((hi - lo) / step)
    targets = [lo + i * step for i in range(n + 1)]
    body = []
    for t in targets:
        row = _nearest(rows, t)
        accel_ff = max(row.ff_effort_pct, 0.0)
        body.append([
            f"{row.t_s:.1f}", f"{row.ref_kmh:.1f}", f"{row.actual_kmh:.2f}", f"{accel_ff:.2f}",
        ])
    return md_table(["モード経過時間[s]", "基準車速[km/h]", "実車速[km/h]", "アクセルFF[%]"], body)


# ─────────────────────────────────────────────────────────────────────
# 図
# ─────────────────────────────────────────────────────────────────────


def _figure(nrows: int, ncols: int, size: tuple[float, float], **kwargs: Any) -> tuple[Any, Any]:
    from matplotlib.figure import Figure  # noqa: PLC0415

    fig = Figure(figsize=size, dpi=100, layout="constrained")
    return fig, fig.subplots(nrows, ncols, **kwargs)


def fig_speed_overlay(runs: dict[str, RunResult], out: Path) -> Path:
    """候補ごと 1 本目の速度（基準・実車速）とアクセル FF を、序盤と終盤で重ねる。"""
    matplotlib.rcParams["font.family"] = FONT_FAMILY
    fig, axs = _figure(2, 2, (13.0, 7.5), sharey="row")
    windows = ((0.0, 160.0), (1760.0, 1800.0))
    colors = {"C1": "#5a9bd6", "C5": COLOR_ACTUAL, "C4": COLOR_BRAKE}
    ref_plotted = [False, False]
    for col, (lo, hi) in enumerate(windows):
        ax_v, ax_a = axs[0, col], axs[1, col]
        for label, r in runs.items():
            part = [row for row in r.rows if lo <= row.t_s <= hi]
            if not part:
                continue
            t = [row.t_s for row in part]
            if not ref_plotted[col]:
                ax_v.plot(t, [row.ref_kmh for row in part], color=COLOR_REF, linestyle="--",
                          linewidth=1.3, label="基準車速")
                ref_plotted[col] = True
            ax_v.plot(t, [row.actual_kmh for row in part], color=colors[label], linewidth=1.6,
                      label=f"実車速 {label}")
            ax_a.plot(t, [max(row.ff_effort_pct, 0.0) for row in part], color=colors[label],
                      linewidth=1.4, label=f"アクセルFF {label}")
        ax_v.set_title(f"{lo:.0f}〜{hi:.0f} s")
        ax_v.set_ylabel("車速 [km/h]")
        ax_a.set_ylabel("アクセルFF [%]")
        ax_a.set_xlabel("モード経過時間 [s]")
        for ax in (ax_v, ax_a):
            ax.grid(True, alpha=0.3)
            ax.legend(loc="best", fontsize=8)
    out.mkdir(parents=True, exist_ok=True)
    path = out / "speed_c1_c4_c5.png"
    fig.savefig(path)
    return path


def fig_band_deviation(groups: Sequence[Any], out: Path) -> Path:
    from tests.research.compare_runs import SPEED_BANDS  # noqa: PLC0415

    matplotlib.rcParams["font.family"] = FONT_FAMILY
    fig, (ax_mean, ax_p95) = _figure(1, 2, (12.0, 5.0))
    names = [name for _, _, name in SPEED_BANDS]
    x = np.arange(len(names))
    width = 0.25
    colors = {"C1": "#5a9bd6", "C5": COLOR_ACTUAL, "C4": COLOR_BRAKE}
    for i, g in enumerate(groups):
        per_run = [compute_band_stats(r.rows) for r in g.runs]
        med = median_band_stats(per_run)
        means = [med[n].mean_kmh or 0.0 for n in names]
        p95s = [med[n].p95_abs_kmh or 0.0 for n in names]
        lo_mean = [min((s[n].mean_kmh for s in per_run if s[n].mean_kmh is not None), default=0.0)
                   for n in names]
        hi_mean = [max((s[n].mean_kmh for s in per_run if s[n].mean_kmh is not None), default=0.0)
                   for n in names]
        lo_p95 = [
            min((s[n].p95_abs_kmh for s in per_run if s[n].p95_abs_kmh is not None), default=0.0)
            for n in names
        ]
        hi_p95 = [
            max((s[n].p95_abs_kmh for s in per_run if s[n].p95_abs_kmh is not None), default=0.0)
            for n in names
        ]
        off = (i - 1) * width
        ax_mean.bar(x + off, means, width=width, color=colors[g.label], label=g.label,
                    yerr=[np.array(means) - np.array(lo_mean), np.array(hi_mean) - np.array(means)],
                    capsize=3)
        ax_p95.bar(x + off, p95s, width=width, color=colors[g.label], label=g.label,
                   yerr=[np.array(p95s) - np.array(lo_p95), np.array(hi_p95) - np.array(p95s)],
                   capsize=3)
    for ax, title in ((ax_mean, "平均偏差（中央値、縦線=3本の範囲）"),
                      (ax_p95, "|偏差| p95（中央値、縦線=3本の範囲）")):
        ax.set_xticks(x, [f"{n} km/h" for n in names])
        ax.set_ylabel("偏差 [km/h]")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")
    out.mkdir(parents=True, exist_ok=True)
    path = out / "band_deviation.png"
    fig.savefig(path)
    return path


def fig_cruise_opening(ff: Any, out: Path) -> Path:
    matplotlib.rcParams["font.family"] = FONT_FAMILY
    fig, ax = _figure(1, 1, (8.0, 5.0))
    n_future, n_past = len(ff.horizons), len(ff.past_horizons)
    ys = [ff.predict_effort(v, [v] * n_future, [v] * n_past) for v in FIG3_SPEEDS_KMH]
    ax.plot(FIG3_SPEEDS_KMH, ys, color=COLOR_ACCEL, linewidth=2.0, label="定速開度（FF）")
    if ff._speed_clip_max is not None:  # noqa: SLF001 - 研究スクリプトで学習域上限を可視化する
        ax.axvline(ff._speed_clip_max, color="#d64545", linestyle="--", linewidth=1.2,  # noqa: SLF001
                   label=f"学習域上限 {ff._speed_clip_max:.1f} km/h")  # noqa: SLF001
    ax.set_xlabel("車速 V [km/h]（v0 = 先読み = 過去 = V の定速）")
    ax.set_ylabel("FF の開度 [%]")
    ax.set_title("モデルの定速開度（単調増加 → C4 に戻す力が無い）")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    out.mkdir(parents=True, exist_ok=True)
    path = out / "model_cruise_opening.png"
    fig.savefig(path)
    return path


# ─────────────────────────────────────────────────────────────────────


def run(config_path: Path = DEFAULT_CONFIG_PATH, out: Path = OUT_DIR) -> int:
    cfg = load_config(config_path)
    ff = load_feedforward(cfg)  # candidate=C4 のまま（predict_effort は C1/C4/C5 で同一）
    print(f"設定: {config_path}（candidate={cfg.feedforward.candidate}・モデル "
          f"{cfg.feedforward.model_path}）")
    print(f"最高速 {cfg.vehicle.max_speed_kmh:g} km/h・偏差停止 "
          f"{cfg.vehicle.stop_deviation_duration_s:g} s（安全網の設定値）")

    results = [
        evaluate_run(RESULTS / fname, label, cfg, DURATION_S) for label, fname in RUNS
    ]
    by_file = dict(zip((f for _, f in RUNS), results, strict=True))

    print("\n### 1 本ごと\n")
    print(summary_table(results))
    groups = group_by_label(results)
    print("\n### 候補ごとの集計\n")
    print(aggregate_table(groups))
    print("\n### 基準車速帯別の偏差\n")
    print(band_table(results))

    print("\n### 到達時間・走破判定\n")
    print(reached_table(results))

    print("\n### 停車保持（基準車速 0 の区間）\n")
    label_groups = {g.label: g.runs for g in groups}
    print(idle_creep_table(list(label_groups.values()), list(label_groups.keys())))

    c1_c5 = [r for r in results if r.label in ("C1", "C5")]
    print("\n### ACCEL→BRAKE 切り替え（基準 > 5 km/h）\n")
    print(switches_table(c1_c5))

    c5_results = [r for r in results if r.label == "C5"]
    print("\n### C5: 符号反転が集中する 5s ブロック\n")
    print(reversal_hotspot_table(c5_results))

    print("\n### C5 の実質ゲイン と Kp 注入との比較\n")
    print(gain_table(ff))

    print("\n### モデルの定速開度（v0 = 先読み = 過去 = V）\n")
    print(cruise_opening_table(ff))

    print("\n### 惰行減速カーブ（config_testVehicle.yaml）\n")
    print(coast_decel_table(cfg))

    c4_first = by_file["drive_log_real_20260915_164819.csv"]
    print("\n### C4（164819）ログ抜粋 100〜136s（4s 刻み）\n")
    print(excerpt_table(c4_first.rows, 100.0, 136.0, 4.0))
    print("\n### C4（164819）ログ抜粋 1797〜1800s（0.5s 刻み）\n")
    print(excerpt_table(c4_first.rows, 1797.0, 1799.9, 0.5))

    first_runs = {
        label: by_file[next(fname for lbl, fname in RUNS if lbl == label)]
        for label in ("C1", "C5", "C4")
    }
    p1 = fig_speed_overlay(first_runs, out)
    p2 = fig_band_deviation(rank_groups(groups), out)
    p3 = fig_cruise_opening(ff, out)
    print(f"\n図1: {p1}\n図2: {p2}\n図3: {p3}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    import argparse  # noqa: PLC0415

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    args = ap.parse_args(argv)
    return run(args.config, args.out)


if __name__ == "__main__":
    raise SystemExit(main())
