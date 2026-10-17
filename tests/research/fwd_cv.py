"""順モデル（C8）の交差検証の採点の枠（計画 cryptic-scribbling-alpaca の段0）。

何をするか: 設定（層・活性化・損失・u = 0 の取り方・検証の抜き方）を 1 つ受け、
    分割ごとに「学習 → 乱数違いの表を平均 → 外した 1 本のログで採点」を行い、
    結果を追記して表で並べる。
    C7 の表も同じ分割で作り直して同じ物差しで並べる（基準）。
分割の決め方（ユーザー決定）: WLTP のログ 3 本（142410・040027・155834）を 1 本ずつ外す 3 分割。
    格子ステップ 034517 は常に学習側に入れる。
表を平均する理由: 乱数だけを変えても積み上げたずれの最大が 5.5〜7.2 km/h とぶれる。乱数ごとの
    単調化前の配列（raw_grid）を平均してから 1 回だけ単調化して 1 枚の表にしてから採点する
    （実機にも同じ平均の表を出すため）。
採点: 外した 1 本に対して、積み上げたずれ（主）・外れの 3 秒平均 p95・惰行／ブレーキの外れの平均・
    単調化で動いた量。穴の定義は C7 の表（DEFAULT_C7_MAP）で固定する。

使い方:
    .venv/bin/python -m tests.research.fwd_cv --stage 0 [--only 名前] [--workers 4]
        [--force] [--smoke]
"""

from __future__ import annotations

import argparse
import json
import math
import multiprocessing
import time
from collections.abc import Callable, Sequence
from concurrent.futures import ProcessPoolExecutor
from dataclasses import asdict, dataclass
from datetime import datetime
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np

from tests.research import fwd_torch
from tests.research.fwd_data import STOP_SPEED_KMH, FwdRows
from tests.research.fwd_map import RELEASE_GRID_PCT, ForwardTable, load_forward_map, refine_u_grid
from tests.research.fwd_model import FittedModel, grid_from_raw, raw_grid, raw_release_slices
from tests.research.fwd_plan import lagged_input
from tests.research.fwd_tool import (
    DEFAULT_C7_MAP,
    DEFAULT_TRAIN,
    REGIONS,
    RESULTS,
    DeadbandLag,
    Trace,
    _settings,
    eval_model,
    fmt,
    md_table,
    read_logs,
    read_traces,
    residuals,
    settings_of_table,
    table_from,
)
from tests.research.fwd_torch import ALPHA_L2

GRID_LOG = DEFAULT_TRAIN[0]  # 格子ステップ（034517）。常に学習側
WLTP_LOGS = DEFAULT_TRAIN[1:]  # 1 本ずつ外す 3 本
RESULT_DIR = RESULTS / "fwd_cv"
SMOKE_MAX_ITER = 3
SMOKE_ROLLOUT_EPOCHS = 2  # smoke のときの追加学習のエポック数
BASELINE_KEY = "C7基準"

# 採点の帯（車速 [km/h]・u [%]）。惰行は u == 0、ブレーキは車速 40〜60
COAST_BANDS = ((15.0, 40.0), (60.0, 80.0))
BRAKE_SPEED_BAND = (40.0, 60.0)
BRAKE_U_BANDS = ((-1.5, 0.0), (-3.0, -1.5), (-6.0, -3.0))  # (下端, 上端)。上端は含まない

# 停車まぎわの強いブレーキの行を学習から外すしきい値（CvConfig.drop_stop_approach）。
# 根拠: WLTP の学習ログでは止まる直前に停車保持の位置（u ≈ −11%）へ踏み込むが、すぐ車速が 0 に
#   なり減速が −3.4〜−4.6 km/h/s しか測れない。この行が「深く踏むほど減速が弱い」逆転を作る。
#   車速は半分データ由来（該当行は 3.3 km/h 以下。余裕を足した仮値）、u はデータ由来
#   （WLTP に −7.5〜−9% の行が無い）。docs/Problem/ProblemReport_20261001.md 段2i
STOP_APPROACH_SPEED_KMH = 5.0
STOP_APPROACH_U_PCT = -8.0

SCORE_KEYS = (
    "drift_p95",
    "drift_max",
    "ma_all",
    "ma_brake",
    "ma_slow",
    "ma_hole",
    "mono_max",
    "coast_15_40",
    "coast_60_80",
    "brk_0_1.5",
    "brk_1.5_3",
    "brk_3_6",
)


@dataclass(frozen=True)
class CvConfig:
    """設定 1 つ分。coast_from="accel" はブレーキ側を u < 0 の行だけで学習し、
    u = 0 をアクセル側から取る。"""

    name: str
    hidden: tuple[int, ...]
    activation: str = "logistic"
    loss: str = "squared"
    coast_from: str = "mean"
    val_split: str = "random"  # "random"（行の無作為抜き）／"block"（かたまり抜き）
    seeds: tuple[int, ...] = (0, 1, 2)
    alpha: float = ALPHA_L2
    time_weight: bool = False  # True: 行の重みをログの刻み dt_s にする（秒数あたりで公平に）
    # True: 停車まぎわの強いブレーキの行（stop_approach_mask）を学習から取り除く（両側）
    drop_stop_approach: bool = False
    # 追加学習（段4）で車速を積み上げる窓の長さ [s]。0 は追加学習なし
    rollout_s: float = 0.0
    # 2026-10-14 段2o-4: C8 の表の u の刻み [%]。None は C7 の表（like）と同じ刻み。
    # 学習（行の作り方・NN）は変えず、表の格子だけ細かくする（fwd_map.refine_u_grid）
    table_u_step_pct: float | None = None
    # 2026-10-14 段2o-5: True（既定）なら乱数ごとに戻し量ごとの配列（raw_release_slices）も作って
    # 返す（C8 の表に戻しの表を付けるため。fwd_final.build_final_table）。C7 には関係しない
    release_slices: bool = True
    # 2026-10-14 段2o-5: True（既定）ならブレーキ側の学習から「発進の区間」の行
    # （FwdRows.launch_row。
    # 停止保持のブレーキを離しながら動き出す行）を外す。発進は attach_launch の別の表が受け持ち、
    # ここに混ぜると「戻し ⇒ ブレーキが弱い」を過大に教えるため。アクセル側は変えない
    brake_exclude_launch: bool = True

    def key(self) -> str:
        """全フィールドから作る文字列（結果ファイルで同じ設定を見分ける）。
        time_weight・drop_stop_approach・rollout_s・table_u_step_pct は既定値（False／0／None）の
        とき含めない
        （足す前の結果を読み直せるようにするため）。"""
        d = asdict(self)
        for k in ("time_weight", "drop_stop_approach", "rollout_s", "table_u_step_pct"):
            if not d[k]:
                del d[k]
        if d["release_slices"]:  # 既定（True）は含めない（足す前の結果を読み直せるように）
            del d["release_slices"]
        if d["brake_exclude_launch"]:
            del d["brake_exclude_launch"]
        return "|".join(f"{k}={v}" for k, v in d.items())


def folds() -> list[tuple[list[Path], Path]]:
    """(学習ログの一覧, 外す 1 本) の 3 分割。格子ステップは常に学習側。"""
    return [
        ([GRID_LOG, *[p for p in WLTP_LOGS if p != held]], held) for held in WLTP_LOGS
    ]


def fold_name(held: Path) -> str:
    return "_".join(held.stem.split("_")[-2:])


def side_rows(
    rows_list: Sequence[FwdRows], side: str, *, coast_in_brake: bool, exclude_launch: bool = False
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """片側の行の x・加速度 y・ログ番号・時刻 t_s。選び方は fwd_model._xy と同じ
    （accel: u >= 0、brake: u <= 0。coast_in_brake=False なら brake は u < 0）。
    exclude_launch=True はブレーキ側から発進の区間の行（launch_row）を除く（アクセル側は除かない）。
    x は accel が (N, 2)（車速・u）、brake が (N, 3)（車速・u・戻し量。2026-10-14 段2o-5）。"""
    xs, ys, gs, ts = [], [], [], []
    for g, r in enumerate(rows_list):
        if side == "accel":
            m = r.u_pct >= 0
        else:
            m = r.u_pct <= 0 if coast_in_brake else r.u_pct < 0
            if exclude_launch:
                m = m & ~r.launch_row
        cols = [r.speed_kmh[m], r.u_pct[m]]
        if side != "accel":
            cols.append(r.release_pct[m])
        xs.append(np.column_stack(cols))
        ys.append(r.accel_kmhs[m])
        gs.append(np.full(int(m.sum()), g))
        ts.append(r.t_s[m])
    return np.concatenate(xs), np.concatenate(ys), np.concatenate(gs), np.concatenate(ts)


def stop_approach_mask(speed_kmh: np.ndarray, u_pct: np.ndarray) -> np.ndarray:
    """停車まぎわの強いブレーキの行（車速 < しきい値 かつ u <= しきい値）。"""
    return (speed_kmh < STOP_APPROACH_SPEED_KMH) & (u_pct <= STOP_APPROACH_U_PCT)


def side_weights(g: np.ndarray, rows_list: Sequence[FwdRows]) -> np.ndarray:
    """行の重み = その行のログの dt_s を、片側の全行で平均 1 に正規化したもの。
    g は side_rows が返すログ番号。"""
    w = np.array([r.dt_s for r in rows_list])[g]
    return w / w.mean()


def make_windows(
    traces: Sequence[Trace],
    st: DeadbandLag,
    window_s: float,
    *,
    drop_mask_fn: Callable[[np.ndarray, np.ndarray], np.ndarray] | None = None,
) -> tuple[list[fwd_torch.WindowGroup], float, float]:
    """1 本のログの trace から、車速を積み上げる窓を切り出す。
    窓を切ってよい区間 = 実車速が有限で STOP_SPEED_KMH 以上が続く区間
    （accumulate_mountains と同じ）。drop_mask_fn(v, u) が True の行も区間の切れ目にする
    （学習から外した行を積み上げで学習し直さない）。
    区間を n = round(window_s / dt) + 1 点（dt はその trace の刻みの中央値）の
    重ならない窓に切り、端数は「区間の末尾にそろえた窓」を 1 つ足す（前の窓と重なってよい）。
    n 点に満たない区間は捨てる。窓ごとに、遅らせた u・刻み・遅らせた車速の窓内の列番号を
    simulate_speed と同じ規則で前計算する（添字は trace 全体で数え、窓の頭より前を指すときは
    窓の頭＝列 0）。
    返り値: n ごとのグループ、使えた秒数（窓にした区間の長さ）、
    捨てた秒数（n 点に満たない区間の長さ）。"""
    by_n: dict[int, dict[str, list[np.ndarray]]] = {}
    used_s = dropped_s = 0.0
    for tr in traces:
        t, v, u = tr.t, tr.v, tr.u
        if len(t) < 2:
            continue
        n = int(round(window_s / float(np.median(np.diff(t))))) + 1
        ok = np.isfinite(v) & (v >= STOP_SPEED_KMH)
        if drop_mask_fn is not None:
            ok &= ~drop_mask_fn(v, u)
        # trace 全体で 1 度だけ: 行 i（i >= 1）の遅らせた u・遅れ・遅らせた車速の添字（trace 全体）
        u_lag = np.zeros(len(t))
        j_glob = np.zeros(len(t), dtype=np.int64)
        for i in range(1, len(t)):
            tau = float(t[i - 1])
            uu, lag = lagged_input(t, u, tau, st.accel_lag_s, st.brake_lag_s)
            u_lag[i] = uu
            j_glob[i] = int(np.searchsorted(t, tau - lag + 1e-9, side="right")) - 1
        edge = np.diff(np.concatenate([[0], ok.astype(int), [0]]))
        for i0, i1 in zip(np.flatnonzero(edge == 1), np.flatnonzero(edge == -1), strict=True):
            length = int(i1 - i0)
            dur = float(t[i1 - 1] - t[i0])
            if length < n:
                dropped_s += dur
                continue
            used_s += dur
            starts = list(range(int(i0), int(i1) - n + 1, n))
            if starts[-1] + n < i1:  # 端数: 区間の末尾にそろえた窓を足す
                starts.append(int(i1) - n)
            g = by_n.setdefault(n, {"v": [], "u": [], "dt": [], "idx": []})
            for a in starts:
                ks = np.arange(n)
                dt = np.zeros(n)
                dt[1:] = t[a + 1 : a + n] - t[a : a + n - 1]
                j = np.maximum(j_glob[a : a + n] - a, 0)
                idx = np.minimum(j, np.maximum(ks - 1, 0))
                uw = u_lag[a : a + n].copy()
                uw[0] = 0.0
                g["v"].append(v[a : a + n])
                g["u"].append(uw)
                g["dt"].append(dt)
                g["idx"].append(idx)
    groups = [
        fwd_torch.WindowGroup(
            np.array(g["v"]),
            np.array(g["u"]),
            np.array(g["dt"]),
            np.array(g["idx"], dtype=np.int64),
        )
        for _, g in sorted(by_n.items())
    ]
    return groups, used_s, dropped_s


def fit_one_info(
    cfg: CvConfig,
    train_rows: Sequence[FwdRows],
    seed: int,
    *,
    max_iter: int = fwd_torch.MAX_ITER,
    train_traces: Sequence[Sequence[Trace]] | None = None,
    st: DeadbandLag | None = None,
    rollout_epochs: int = fwd_torch.ROLLOUT_MAX_EPOCHS,
) -> tuple[FittedModel, dict | None]:
    """アクセル側・ブレーキ側を学習して FittedModel にする（乱数 1 つ分）。
    cfg.rollout_s > 0 のときは、学習後に train_traces（学習ログごとの trace）から窓を作って
    finetune_rollout で 2 本を追加学習する。返り値の 2 つ目は追加学習の info（なしなら None）。"""
    t0 = time.perf_counter()
    sides = {}
    point_data = {}
    for side in ("accel", "brake"):
        x, y, g, t = side_rows(
            train_rows,
            side,
            coast_in_brake=cfg.coast_from != "accel",
            exclude_launch=cfg.brake_exclude_launch,
        )
        if cfg.drop_stop_approach:  # 重み 0 ではなく行そのものを除く（検証の分け方も変わる）
            keep = ~stop_approach_mask(x[:, 0], x[:, 1])
            x, y, g, t = x[keep], y[keep], g[keep], t[keep]
        blocks = fwd_torch.block_ids(t, g) if cfg.val_split == "block" else None
        w = side_weights(g, train_rows) if cfg.time_weight else None
        extra = {"weights": w} if cfg.time_weight else {}
        point_data[side] = (x, y, w)
        sides[side] = fwd_torch.fit_side(
            x,
            y,
            hidden=cfg.hidden,
            activation=cfg.activation,
            loss=cfg.loss,
            alpha=cfg.alpha,
            seed=seed,
            max_iter=max_iter,
            val_blocks=blocks,
            **extra,
        )
    info = None
    if cfg.rollout_s > 0:
        if train_traces is None or st is None:
            raise ValueError("rollout_s > 0 には train_traces と st が要る")
        drop = stop_approach_mask if cfg.drop_stop_approach else None
        groups: list[fwd_torch.WindowGroup] = []
        used = dropped = 0.0
        for traces in train_traces:
            gs, us, ds = make_windows(traces, st, cfg.rollout_s, drop_mask_fn=drop)
            groups += gs
            used, dropped = used + us, dropped + ds
        sides["accel"], sides["brake"], info = fwd_torch.finetune_rollout(
            sides["accel"],
            sides["brake"],
            point_data,
            groups,
            loss=cfg.loss,
            alpha=cfg.alpha,
            seed=seed,
            coast_from=cfg.coast_from,
            max_epochs=rollout_epochs,
        )
        info["used_s"], info["dropped_s"] = used, dropped
    fm = FittedModel(
        "torch",
        sides["accel"],
        sides["brake"],
        hidden=cfg.hidden,
        train_time_s=time.perf_counter() - t0,
        sources=[r.source for r in train_rows],
    )
    return fm, info


def fit_one(cfg: CvConfig, train_rows: Sequence[FwdRows], seed: int, **kwargs) -> FittedModel:
    """fit_one_info のモデルだけ返す版。"""
    return fit_one_info(cfg, train_rows, seed, **kwargs)[0]


def format_rollout_info(info: dict) -> str:
    """追加学習の info の 1 行表示。"""
    return (
        f"追加学習 lam={info['lam']:.3g}"
        f" 1点 {info['train_point_start']:.4f}→{info['train_point_end']:.4f}"
        f" 積み上げ {info['train_roll_start']:.3f}→{info['train_roll_end']:.3f} km/h"
        f"（検証 1点 {info['val_point_start']:.4f}→{info['val_point_end']:.4f}"
        f" 積み上げ {info['val_roll_start']:.3f}→{info['val_roll_end']:.3f}）"
        f" エポック {info['epochs']}（採用 {info['best_epoch']}）{info['seconds']:.1f} 秒"
        f" 窓 {info['n_windows_train']}+{info['n_windows_val']}"
        f"（使えた {info['used_s']:.0f} 秒・捨てた {info['dropped_s']:.0f} 秒）"
    )


# ─────────────────────────── 採点 ───────────────────────────


@lru_cache(maxsize=8)
def _held_data(path: Path, st: DeadbandLag) -> tuple[list[FwdRows], list]:
    return read_logs([path], st), read_traces(path, st)


def _band_mean(err: np.ndarray, mask: np.ndarray) -> float:
    return float(err[mask].mean()) if mask.any() else math.nan


def score_table(tb: ForwardTable, like: ForwardTable, held_path: Path, st: DeadbandLag) -> dict:
    """外した 1 本に対する採点。like は C7 の表（穴の定義をそろえる）。外れ = 実測 − 見込み。"""
    logs, traces = _held_data(Path(held_path), st)
    ev = eval_model("cv", tb, like, logs, traces, st, is_ref=tb.accel_before_mono is None)
    res = residuals(tb, logs)
    err, v, u = res["err"], res["v"], res["u"]
    out: dict[str, float] = {
        "drift_p95": ev.drift_p95,
        "drift_max": ev.drift_max,
        "ma_all": ev.stats["全体"].ma_p95,
        "ma_brake": ev.stats[REGIONS[0]].ma_p95,
        "ma_slow": ev.stats[REGIONS[1]].ma_p95,
        "ma_hole": ev.stats[REGIONS[2]].ma_p95,
        "mono_max": ev.mono_max,
    }
    for (lo, hi), key in zip(COAST_BANDS, ("coast_15_40", "coast_60_80"), strict=True):
        out[key] = _band_mean(err, (u == 0) & (v >= lo) & (v < hi))
    vb = (v >= BRAKE_SPEED_BAND[0]) & (v < BRAKE_SPEED_BAND[1])
    for (lo, hi), key in zip(BRAKE_U_BANDS, ("brk_0_1.5", "brk_1.5_3", "brk_3_6"), strict=True):
        out[key] = _band_mean(err, vb & (u >= lo) & (u < hi))
    return out


def _agg(per_fold: dict[str, dict], how: str) -> dict[str, float]:
    out = {}
    for k in SCORE_KEYS:
        vals = np.array([d[k] for d in per_fold.values()], dtype=float)
        vals = vals[np.isfinite(vals)]
        if len(vals) == 0:
            out[k] = math.nan
        elif how == "mean":
            out[k] = float(vals.mean())
        else:  # 最悪の 1 本。外れの平均（符号つき）は絶対値が最大のもの
            signed = k.startswith(("coast", "brk"))
            out[k] = float(vals[np.argmax(np.abs(vals))]) if signed else float(vals.max())
    return out


# ─────────────────────────── 学習・集計 ───────────────────────────


def _worker(args: tuple) -> tuple[int, int, np.ndarray, float, np.ndarray | None]:
    """(分割番号, 乱数) 1 タスク。単調化前の配列・学習秒数・戻し量ごとの単調化前の配列
    （cfg.release_slices でなければ None。2026-10-14 段2o-5）を返す。pickle できるようトップレベル。
    末尾の (学習ログごとの trace, DeadbandLag) は rollout_s > 0 のときだけ使う（省略可）。
    追加学習の info はここで表示する。"""
    cfg, fold_idx, seed, train_rows, like_path, max_iter, n_threads, *rest = args
    train_traces, st = rest if rest else (None, None)
    if n_threads:
        fwd_torch.set_threads(n_threads)
    like = load_forward_map(Path(like_path))
    if cfg.table_u_step_pct is not None:
        like = refine_u_grid(like, cfg.table_u_step_pct)
    t0 = time.perf_counter()
    epochs = SMOKE_ROLLOUT_EPOCHS if max_iter == SMOKE_MAX_ITER else fwd_torch.ROLLOUT_MAX_EPOCHS
    fm, info = fit_one_info(
        cfg,
        train_rows,
        seed,
        max_iter=max_iter,
        train_traces=train_traces,
        st=st,
        rollout_epochs=epochs,
    )
    if info is not None:
        print(f"    分割{fold_idx} 乱数{seed}: {format_rollout_info(info)}", flush=True)
    raw = raw_grid(fm, like, coast_from=cfg.coast_from)
    rel = (
        raw_release_slices(fm, like, RELEASE_GRID_PCT, coast_from=cfg.coast_from)
        if cfg.release_slices
        else None
    )
    return fold_idx, seed, raw, time.perf_counter() - t0, rel


@dataclass
class Context:
    like: ForwardTable
    like_path: Path
    st: DeadbandLag
    fold_rows: list[tuple[list[FwdRows], Path]]  # (学習の行, 外す 1 本)
    # 分割ごとの、学習ログごとの trace（追加学習 rollout_s > 0 の設定があるときだけ読む）
    fold_traces: list[list[list[Trace]]] | None = None


def make_context(like_path: Path = DEFAULT_C7_MAP, *, with_traces: bool = False) -> Context:
    like = load_forward_map(Path(like_path))
    st = settings_of_table(like, _settings())
    all_rows = dict(zip(DEFAULT_TRAIN, read_logs(DEFAULT_TRAIN, st), strict=True))
    fold_rows = [([all_rows[p] for p in tr], held) for tr, held in folds()]
    fold_traces = None
    if with_traces:
        all_traces = {p: read_traces(p, st) for p in DEFAULT_TRAIN}
        fold_traces = [[all_traces[p] for p in tr] for tr, _ in folds()]
    return Context(like, Path(like_path), st, fold_rows, fold_traces)


def run_baseline(ctx: Context) -> dict:
    """C7: 各分割の学習側のログから表を作り直して同じ物差しで採点する（単調の量は NaN）。"""
    per_fold: dict[str, dict] = {}
    for train_rows, held in ctx.fold_rows:
        tb = table_from(
            train_rows,
            ctx.st,
            edges=ctx.like.speed_edges,
            min_s=float(ctx.like.meta["min_s"]),
            u_step=float(ctx.like.meta["u_step_pct"]),
        )
        tb.accel_before_mono = None  # 基準は単調化の量を出さない（eval_model の is_ref 相当）
        per_fold[fold_name(held)] = score_table(tb, ctx.like, held, ctx.st)
    return {"folds": per_fold, "train_s": 0.0}


def run_model_config(cfg: CvConfig, ctx: Context, workers: int, max_iter: int) -> dict:
    if cfg.rollout_s > 0 and ctx.fold_traces is None:
        raise ValueError("rollout_s > 0 には make_context(with_traces=True) が要る")
    tail = (lambda i: (ctx.fold_traces[i], ctx.st)) if cfg.rollout_s > 0 else (lambda i: ())
    tasks = [
        (cfg, i, seed, tr, str(ctx.like_path), max_iter, 1 if workers > 1 else 0, *tail(i))
        for i, (tr, _) in enumerate(ctx.fold_rows)
        for seed in cfg.seeds
    ]
    if workers > 1:
        mp = multiprocessing.get_context("spawn")
        with ProcessPoolExecutor(max_workers=workers, mp_context=mp) as ex:
            outs = list(ex.map(_worker, tasks))
    else:
        outs = [_worker(t) for t in tasks]
    for fold_idx, seed, _, sec, _rel in outs:
        print(f"    タスク 分割{fold_idx} 乱数{seed}: {sec:.1f} 秒", flush=True)
    per_fold: dict[str, dict] = {}
    for i, (_, held) in enumerate(ctx.fold_rows):
        raws = [o[2] for o in outs if o[0] == i]
        # raw は worker が細かい格子（table_u_step_pct）で評価した値。採点の like は C7 のまま
        like_raw = ctx.like
        if cfg.table_u_step_pct is not None:
            like_raw = refine_u_grid(ctx.like, cfg.table_u_step_pct)
        tb = grid_from_raw(
            np.mean(raws, axis=0),
            like_raw,
            {"coast_from": cfg.coast_from, "n_models": len(raws), "hidden": cfg.hidden},
        )
        per_fold[fold_name(held)] = score_table(tb, ctx.like, held, ctx.st)
    return {"folds": per_fold, "train_s": float(sum(o[3] for o in outs))}


def load_results(path: Path) -> dict[str, dict]:
    out: dict[str, dict] = {}
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                rec = json.loads(line)
                out[rec["key"]] = rec
    return out


def append_result(path: Path, rec: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")


def run_one(
    key: str,
    name: str,
    compute: Any,
    results_path: Path,
    done: dict[str, dict],
    force: bool,
) -> dict:
    if key in done and not force:
        print(f"- {name}: 結果あり（読み直し）", flush=True)
        return done[key]
    print(f"- {name}: 実行中", flush=True)
    t0 = time.perf_counter()
    body = compute()
    rec = {
        "key": key,
        "name": name,
        **body,
        "mean": _agg(body["folds"], "mean"),
        "worst": _agg(body["folds"], "worst"),
        "wall_s": time.perf_counter() - t0,
        "at": datetime.now().isoformat(timespec="seconds"),
    }
    append_result(results_path, rec)
    done[key] = rec
    return rec


# ─────────────────────────── 表示 ───────────────────────────

def _layers(hidden: tuple[int, ...]) -> str:
    """(30, 30) → "2層30個"（設定名用。層ごとの個数は同じ前提）。"""
    return f"{len(hidden)}層{hidden[0]}個"


STAGES: dict[int, list[CvConfig]] = {
    0: [
        CvConfig("V0 1層10個", (10,)),
        CvConfig("V5 2層30個", (30, 30)),
        CvConfig("V5 乱数3-5", (30, 30), seeds=(3, 4, 5)),
        CvConfig("V5 かたまり抜き", (30, 30), val_split="block"),
    ],
    # 段1: 惰行とブレーキの段差（層は V5 に固定）。1 行目は段0 の V5 と同じ設定（結果を読み直す）
    1: [
        CvConfig("V5 2層30個", (30, 30)),
        CvConfig("1-1 絶対誤差", (30, 30), loss="absolute"),
        CvConfig("1-2 u=0をアクセル側", (30, 30), coast_from="accel"),
        CvConfig("1-3 絶対誤差+u=0をアクセル側", (30, 30), loss="absolute", coast_from="accel"),
    ],
    # 段3 の 1 巡目: 層 5 通り × 活性化 3 通り（損失は段1 で選んだ絶対誤差、u = 0 は 2 式の平均、
    # 単調の形の縛りなし＝段2 は未実施）。
    # 2 層 30 個・logistic は段1 の 1-1 と同じ設定（結果を読み直す）
    3: [
        CvConfig(
            "1-1 絶対誤差" if (h, a) == ((30, 30), "logistic") else f"3 {_layers(h)} {a}",
            h,
            activation=a,
            loss="absolute",
        )
        for a in ("logistic", "tanh", "relu")
        for h in ((10,), (30,), (30, 30), (50, 50), (30, 30, 30))
    ],
    # 段3 の 2 巡目（段番号 32）: 1 巡目の上位 2 版（2 層 50 個 × relu・tanh。2026-10-05 決定）
    # × L2 3 通り × 秒数重み 2 通り。L2=1e-4・秒数重みなしは 1 巡目と同じ設定（結果を読み直す）
    32: [
        CvConfig(
            f"3 {_layers((50, 50))} {a}"
            if (al, tw) == (1e-4, False)
            else f"32 {_layers((50, 50))} {a} L2={al:g}" + (" 秒数重み" if tw else ""),
            (50, 50),
            activation=a,
            loss="absolute",
            alpha=al,
            time_weight=tw,
        )
        for a in ("relu", "tanh")
        for al in (1e-4, 1e-3, 1e-2)
        for tw in (False, True)
    ],
    # 段4: 積み上げたずれでの追加学習（2 層 50 個・絶対誤差・停車まぎわ除外）。
    # rollout_s=0 は追加学習なし
    4: [
        CvConfig(
            f"4 {_layers((50, 50))} {a} 外す" + (f" 積み上げ{k:g}秒" if k else ""),
            (50, 50),
            activation=a,
            loss="absolute",
            drop_stop_approach=True,
            rollout_s=k,
        )
        for a in ("relu", "tanh")
        for k in (0.0, 10.0, 30.0)
    ],
}


def print_tables(recs: Sequence[dict]) -> None:
    f = fmt
    print("\n### (1) 積み上げたずれと外れの 3 秒平均 p95（外した 1 本ごとの 3 本平均／最悪）\n")
    head = [
        "設定",
        "積み上げp95 平均",
        "最悪",
        "最大 平均",
        "最悪",
        "外れp95 全体",
        "ブレーキ行",
        "0-15",
        "穴",
        "単調で動いた量(最大)",
        "学習秒",
    ]
    body = [
        [
            r["name"],
            f(r["mean"]["drift_p95"]),
            f(r["worst"]["drift_p95"]),
            f(r["mean"]["drift_max"]),
            f(r["worst"]["drift_max"]),
            f(r["mean"]["ma_all"]),
            f(r["mean"]["ma_brake"]),
            f(r["mean"]["ma_slow"]),
            f(r["mean"]["ma_hole"]),
            f(r["mean"]["mono_max"]),
            f(r["train_s"], 1),
        ]
        for r in recs
    ]
    print(md_table(head, body))
    print("\n### (2) 外れ（実測 − 見込み）の平均（3 本平均）\n")
    head = [
        "設定",
        "惰行 15-40",
        "惰行 60-80",
        "ブレーキ 0〜-1.5%",
        "-1.5〜-3%",
        "-3〜-6%",
    ]
    body = [
        [r["name"], *[f(r["mean"][k]) for k in SCORE_KEYS[7:]]]
        for r in recs
    ]
    print(md_table(head, body))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="fwd_cv", description="C8 の交差検証（段0）")
    p.add_argument("--stage", type=int, default=0)
    p.add_argument("--only", default=None, help="設定名の部分一致")
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--force", action="store_true", help="同じ設定の結果があってもやり直す")
    p.add_argument("--smoke", action="store_true", help="動作確認（max_iter 3）")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.stage not in STAGES:
        print(f"段 {args.stage} は未定義（定義済み: {sorted(STAGES)}）")
        return 2
    cfgs = [c for c in STAGES[args.stage] if args.only is None or args.only in c.name]
    results_path = RESULT_DIR / ("results_smoke.jsonl" if args.smoke else "results.jsonl")
    max_iter = SMOKE_MAX_ITER if args.smoke else fwd_torch.MAX_ITER
    ctx = make_context(with_traces=any(c.rollout_s > 0 for c in cfgs))
    done = load_results(results_path)
    print(f"# 順モデル C8 の交差検証 段{args.stage}\n")
    print(f"- 結果ファイル: `{results_path}`")
    held_names = ', '.join(fold_name(h) for _, h in folds())
    print(f"- 分割: 外す 1 本 = {held_names}（034517 は常に学習側）")
    recs = [
        run_one(
            BASELINE_KEY,
            "C7(同じ分割で再作成)",
            lambda: run_baseline(ctx),
            results_path,
            done,
            args.force,
        )
    ]
    for cfg in cfgs:
        recs.append(
            run_one(
                cfg.key() + f"|max_iter={max_iter}",
                cfg.name,
                lambda cfg=cfg: run_model_config(cfg, ctx, args.workers, max_iter),
                results_path,
                done,
                args.force,
            )
        )
    print_tables(recs)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
