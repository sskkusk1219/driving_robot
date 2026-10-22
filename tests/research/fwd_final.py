"""順モデル C8 の最終版を作る（交差検証で選んだ設定で、学習ログ全部を学習して pkl にする）。

何をするか: 設定（既定は `1-1 絶対誤差` = 2 層 30 個・logistic・損失=絶対誤差・u = 0 は 2 式の平均・
    検証は無作為抜き）で DEFAULT_TRAIN の 4 本全部を学習し、乱数ごとの単調化前の配列（raw_grid）を
    平均 → grid_from_raw で 1 回だけ単調化して 1 枚の表にする（fwd_cv.run_model_config と同じ）。
    選定確認ログ（DEFAULT_CHECK）で C7・現行 C8（V0）・新しい表を並べて 1 回採点し、
    続けて参考ログでも同じ表を出す（参考のみ。選定・学習には使わない）。
    採否は実機で決める。オフラインの点数では決めない。

使い方:
    .venv/bin/python -m tests.research.fwd_final [--workers 2] [--seeds 0 1 2 3 4]
        [--train CSV...] [--map PKL] [--check CSV...] [--ref-check CSV...] [--out PKL]
        [--save] [--smoke]
    （--smoke は max_iter 3 の動作確認。名前で区別できないので smoke 時は保存しない）
"""

from __future__ import annotations

import argparse
import math
import multiprocessing
import time
from collections.abc import Sequence
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from tests.research import fwd_torch
from tests.research.fwd_cv import (
    SMOKE_MAX_ITER,
    STOP_APPROACH_SPEED_KMH,
    STOP_APPROACH_U_PCT,
    CvConfig,
    _layers,
    _worker,
    stop_approach_mask,
)
from tests.research.fwd_data import FwdRows
from tests.research.fwd_map import (
    RELEASE_GRID_PCT,
    ForwardTable,
    attach_accel_push,
    attach_accel_release,
    attach_release,
    load_forward_map,
    refine_u_grid,
    save_table,
)
from tests.research.fwd_model import grid_from_raw, monotone_rows
from tests.research.fwd_tool import (
    DEFAULT_C7_MAP,
    DEFAULT_CHECK,
    DEFAULT_TRAIN,
    REF_NAME,
    RESULTS,
    Trace,
    _settings,
    err_stats,
    model_default_path,
    print_eval_block,
    read_logs,
    read_traces,
    residuals,
    settings_of_table,
    skip_missing,
)

DEFAULT_SEEDS = (0, 1, 2, 3, 4)
V0_NAME = "V0（現行 C8）"
V0_PATH = RESULTS / "models" / "fwd_model_20261003_010536.pkl"
REF_CHECK = [
    RESULTS / "drive_log_real_20261003_063303.csv",
    RESULTS / "drive_log_real_20261003_173302.csv",
]


def final_config(
    seeds: Sequence[int] = DEFAULT_SEEDS,
    *,
    hidden: Sequence[int] = (30, 30),
    activation: str = "logistic",
    drop_stop_approach: bool = False,
    rollout_s: float = 0.0,
    table_u_step_pct: float | None = None,
    release_slices: bool = True,
    brake_exclude_launch: bool = True,
    accel_release_input: bool = True,
    accel_push_input: bool = False,
) -> CvConfig:
    """既定は交差検証で良かった `1-1 絶対誤差`（乱数の本数だけ最終用に増やす）。
    層・活性化・停車まぎわ除外を変えたときは設定名を引数から作る。
    rollout_s が 0 以外（積み上げたずれでの追加学習）のときは名前の末尾に積み上げ秒数を付ける。"""
    hidden = tuple(hidden)
    if (hidden, activation, drop_stop_approach) == ((30, 30), "logistic", False):
        name = "1-1 絶対誤差"
    else:
        name = f"{_layers(hidden)} {activation} 絶対誤差" + (
            " 停車まぎわ除外" if drop_stop_approach else ""
        )
    if rollout_s:
        name += f" 積み上げ{rollout_s:g}秒"
    return CvConfig(
        name,
        hidden,
        activation=activation,
        loss="absolute",
        seeds=tuple(seeds),
        drop_stop_approach=drop_stop_approach,
        rollout_s=rollout_s,
        table_u_step_pct=table_u_step_pct,
        release_slices=release_slices,
        brake_exclude_launch=brake_exclude_launch,
        accel_release_input=accel_release_input,
        accel_push_input=accel_push_input,
    )


def build_final_table(
    cfg: CvConfig,
    train_rows: Sequence[FwdRows],
    like_path: Path,
    *,
    workers: int,
    max_iter: int = fwd_torch.MAX_ITER,
    train_traces: Sequence[Sequence[Trace]] | None = None,
    train_paths: Sequence[str | Path] | None = None,
    seed_mae: bool = False,
) -> tuple[ForwardTable, list[tuple[int, float]]]:
    """全学習ログで乱数ごとに学習 → 単調化前の配列を平均 → 1 回だけ単調化した表と、
    (乱数, 学習秒数) の一覧を返す。cfg.rollout_s > 0 のときは学習ログごとの trace が要る
    （train_traces を渡さなければ train_paths、それも無ければ DEFAULT_TRAIN から読む）。
    追加学習の info は乱数ごとに表示する。

    seed_mae=True のとき、乱数ごとの単調化前の配列（outs[2]）から 1 本ずつ表を作り、学習ログ
    （train_rows）での MAE を求めて表の meta["seed_mae"]（{乱数: MAE}）に入れる（手順2 の
    チェック `step2_check` の乱数の外れ用）。既定 False は従来どおり。"""
    like = load_forward_map(Path(like_path))
    st_like = like  # 設定（遅れ・不感帯）と穴の定義は C7 の表のまま
    if cfg.table_u_step_pct is not None:
        like = refine_u_grid(like, cfg.table_u_step_pct)  # worker の raw と同じ格子にする
    threads = 1 if workers > 1 else 0
    tail: tuple = ()
    if cfg.rollout_s > 0:
        st = settings_of_table(st_like, _settings())
        if train_traces is None:
            train_traces = [read_traces(Path(p), st) for p in (train_paths or DEFAULT_TRAIN)]
        tail = (train_traces, st)
    tasks = [
        (cfg, 0, s, list(train_rows), str(like_path), max_iter, threads, *tail) for s in cfg.seeds
    ]
    if workers > 1:
        mp = multiprocessing.get_context("spawn")
        with ProcessPoolExecutor(max_workers=workers, mp_context=mp) as ex:
            outs = list(ex.map(_worker, tasks))
    else:
        outs = [_worker(t) for t in tasks]
    extra = (
        {
            "drop_stop_approach": True,
            "stop_approach_speed_kmh": STOP_APPROACH_SPEED_KMH,
            "stop_approach_u_pct": STOP_APPROACH_U_PCT,
        }
        if cfg.drop_stop_approach
        else {"drop_stop_approach": False}
    )
    seed_mae_meta: dict[str, object] = {}
    if seed_mae:
        mae_by_seed: dict[int, float] = {}
        for o in outs:
            res = residuals(grid_from_raw(o[2], like, {}), list(train_rows))
            st_s = err_stats(res["err"], res["ma"], np.ones(len(res["err"]), dtype=bool))
            mae_by_seed[int(o[1])] = float(st_s.mae)
        seed_mae_meta["seed_mae"] = mae_by_seed
    tb = grid_from_raw(
        np.mean([o[2] for o in outs], axis=0),
        like,
        {
            **extra,
            **seed_mae_meta,
            "coast_from": cfg.coast_from,
            "n_models": len(outs),
            "hidden": cfg.hidden,
            "loss": cfg.loss,
            "activation": cfg.activation,
            "val_split": cfg.val_split,
            "seeds": cfg.seeds,
            "rollout_s": cfg.rollout_s,
            "accel_release_input_enabled": cfg.accel_release_input,
            "accel_push_input_enabled": cfg.accel_push_input,
            "sources": "+".join(r.source for r in train_rows),
        },
    )
    if cfg.release_slices and all(o[4] is not None for o in outs):
        # 2026-10-14 段2o-5: 戻しの表。本体と同じ流れ（乱数の平均 → 行ごとに単調化）。ブレーキの行
        # （u < 0。停車まぎわ除外のときは学習から外した行も除く）の戻し量で release_max_pct と
        # 秒数を決め、足りなければ付けない（attach_release）
        rel_p, rel_dt = [], []
        for r in train_rows:
            m = r.u_pct < 0.0
            if cfg.brake_exclude_launch:
                m &= ~r.launch_row  # 学習のブレーキ側と同じ行で数える
            if cfg.drop_stop_approach:
                m &= ~stop_approach_mask(r.speed_kmh, r.u_pct)
            rel_p.append(r.release_pct[m])
            rel_dt.append(r.dt_row_s[m])
        slices = monotone_rows(np.mean([o[4] for o in outs], axis=0), like.u_grid)
        tb = attach_release(
            tb,
            slices,
            release_pct=np.concatenate(rel_p) if rel_p else np.zeros(0),
            dt_row_s=np.concatenate(rel_dt) if rel_dt else np.zeros(0),
            release_grid=RELEASE_GRID_PCT,
        )
    if cfg.release_slices and cfg.accel_release_input and all(o[5] is not None for o in outs):
        # 2026-10-20 段4-3: アクセルの戻しの表。ブレーキの戻しの表と同じ流れ。アクセルの行
        # （u > 0。停車まぎわ除外は u <= 閾値の行だけを外すのでアクセルの行は変わらない）の
        # アクセルの戻し量で accel_release_max_pct と秒数を決め、足りなければ付けない
        arel_p, arel_dt = [], []
        for r in train_rows:
            m = r.u_pct > 0.0
            arel_p.append(r.accel_release_pct[m])
            arel_dt.append(r.dt_row_s[m])
        aslices = monotone_rows(np.mean([o[5] for o in outs], axis=0), like.u_grid)
        tb = attach_accel_release(
            tb,
            aslices,
            release_pct=np.concatenate(arel_p) if arel_p else np.zeros(0),
            dt_row_s=np.concatenate(arel_dt) if arel_dt else np.zeros(0),
            release_grid=RELEASE_GRID_PCT,
        )
    if cfg.release_slices and cfg.accel_push_input and all(o[6] is not None for o in outs):
        # 2026-10-23 段5-7b: アクセルの踏み増しの表。アクセルの戻しの表と同じ流れ。アクセルの行
        # （u > 0）の踏み増し量で accel_push_max_pct と秒数を決め、足りなければ付けない
        push_p, push_dt = [], []
        for r in train_rows:
            m = r.u_pct > 0.0
            push_p.append(r.accel_push_pct[m])
            push_dt.append(r.dt_row_s[m])
        pslices = monotone_rows(np.mean([o[6] for o in outs], axis=0), like.u_grid)
        tb = attach_accel_push(
            tb,
            pslices,
            push_pct=np.concatenate(push_p) if push_p else np.zeros(0),
            dt_row_s=np.concatenate(push_dt) if push_dt else np.zeros(0),
            push_grid=RELEASE_GRID_PCT,
        )
    return tb, [(o[1], o[3]) for o in outs]


def print_blocks(tables: dict, like: ForwardTable, st, check: Sequence[str], ref: Sequence[str]):
    """比較ログが空なら、その採点をとばす（固定ログが無い環境用）。"""
    print("## 検証ログ（選定確認。1 回だけ採点）\n")
    if check:
        print_eval_block(like, tables, check, st)
    else:
        print("- 検証ログが無いので採点をとばします")
    print("\n## 参考ログ（参考のみ。選定にも学習にも使わない）\n")
    if ref:
        print_eval_block(like, tables, ref, st)
    else:
        print("- 参考ログが無いので採点をとばします")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="fwd_final", description="C8 最終版（全ログ学習）")
    p.add_argument("--workers", type=int, default=2)
    p.add_argument("--seeds", type=int, nargs="+", default=list(DEFAULT_SEEDS))
    p.add_argument("--activation", choices=("logistic", "tanh", "relu"), default="logistic")
    p.add_argument("--hidden", type=int, nargs="+", default=[30, 30], help="各層の個数")
    p.add_argument(
        "--drop-stop-approach", action="store_true", help="停車まぎわの強いブレーキの行を除く"
    )
    p.add_argument(
        "--rollout-s",
        type=float,
        default=0.0,
        help="積み上げたずれで追加学習するときの窓の長さ [s]（0 は追加学習なし）",
    )
    p.add_argument("--train", nargs="+", default=[str(x) for x in DEFAULT_TRAIN], metavar="CSV")
    p.add_argument("--map", default=str(DEFAULT_C7_MAP), help="C7 の表の pkl（格子・設定の元）")
    p.add_argument("--check", nargs="+", default=[str(x) for x in DEFAULT_CHECK], metavar="CSV")
    p.add_argument("--ref-check", nargs="+", default=[str(x) for x in REF_CHECK], metavar="CSV")
    p.add_argument("--out", default=None, help="保存先 pkl（省略: results/models/fwd_model_*.pkl）")
    p.add_argument("--save", action="store_true", help="fwd_model_<UTC>.pkl に保存")
    p.add_argument("--smoke", action="store_true", help="max_iter 3 の動作確認（保存しない）")
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    cfg = final_config(
        args.seeds,
        hidden=args.hidden,
        activation=args.activation,
        drop_stop_approach=args.drop_stop_approach,
        rollout_s=args.rollout_s,
    )
    max_iter = SMOKE_MAX_ITER if args.smoke else fwd_torch.MAX_ITER
    like = load_forward_map(Path(args.map))
    st = settings_of_table(like, _settings())
    train_rows = read_logs(args.train, st)
    print("# 順モデル C8 最終版（全学習ログで学習）\n")
    print(f"- 設定: {cfg.name}（隠れ層 {cfg.hidden}・{cfg.activation}・損失 {cfg.loss}・"
          f"u=0 {cfg.coast_from}・検証 {cfg.val_split}）、乱数 {list(cfg.seeds)}、"
          f"max_iter {max_iter}{'（smoke）' if args.smoke else ''}"
          + (f"、追加学習: 積み上げ {cfg.rollout_s:g} 秒の窓" if cfg.rollout_s else ""))
    print(f"- 学習ログ: {', '.join(r.source for r in train_rows)}")
    if cfg.drop_stop_approach:
        print(f"- 停車まぎわ除外（車速 < {STOP_APPROACH_SPEED_KMH:g} km/h かつ "
              f"u <= {STOP_APPROACH_U_PCT:g}%）で学習から外した行:")
        for r in train_rows:
            n = int(stop_approach_mask(r.speed_kmh, r.u_pct).sum())
            print(f"    {r.source}: {n} 行（{n * r.dt_s:.2f} 秒）")
    t0 = time.perf_counter()
    tb, secs = build_final_table(
        cfg, train_rows, Path(args.map), workers=args.workers, max_iter=max_iter,
        train_paths=args.train,
    )
    wall = time.perf_counter() - t0
    for seed, sec in secs:
        print(f"    乱数{seed}: 学習 {sec:.1f} 秒")
    print(f"- 学習の合計 {sum(s for _, s in secs):.1f} 秒・壁時計 {wall:.1f} 秒")
    print(
        f"- 戻しの表: {'あり' if tb.release_grid is not None else 'なし'}"
        f"（戻し量 0.3% 以上のブレーキの行 {tb.meta.get('release_rows_s', 0.0):.1f} 秒、"
        f"release_max_pct {tb.meta.get('release_max_pct', 0.0):.2f}%）"
    )
    print(
        f"- アクセルの戻しの表: {'あり' if tb.accel_release_grid is not None else 'なし'}"
        f"（アクセル側の入力 {'3（戻し量あり）' if cfg.accel_release_input else '2（車速・u）'}、"
        f"戻し量 0.3% 以上のアクセルの行 {tb.meta.get('accel_release_rows_s', 0.0):.1f} 秒、"
        f"accel_release_max_pct {tb.meta.get('accel_release_max_pct', 0.0):.2f}%）"
    )
    print(
        f"- アクセルの踏み増しの表: {'あり' if tb.accel_push_grid is not None else 'なし'}"
        f"（踏み増し量 0.3% 以上のアクセルの行 {tb.meta.get('accel_push_rows_s', 0.0):.1f} 秒、"
        f"accel_push_max_pct {tb.meta.get('accel_push_max_pct', 0.0):.2f}%）"
    )
    moved = float(np.abs(tb.accel_grid - tb.accel_before_mono).max())
    print(
        f"- 単調化で動いた量: 最大 {moved:.4f} km/h/s"
        f"（monotone_violation 最大 {tb.meta['mono_violation_max']:.4f}・"
        f"平均 {tb.meta['mono_violation_mean']:.4f}）\n"
    )
    tables = {REF_NAME: (like, math.nan)}
    if V0_PATH.exists():
        tables[V0_NAME] = (load_forward_map(V0_PATH), math.nan)
    else:
        print(f"- {V0_NAME}: `{V0_PATH}` が無いのでその行をとばします")
    tables[f"新（{cfg.name} 全ログ）"] = (tb, sum(s for _, s in secs))
    check = skip_missing(args.check, "検証ログ")
    ref = skip_missing(args.ref_check, "参考ログ")
    print_blocks(tables, like, st, check, ref)
    if (args.save or args.out) and not args.smoke:
        out = Path(args.out) if args.out else model_default_path()
        print(f"\n- 保存: `{save_table(tb, out)}`")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
