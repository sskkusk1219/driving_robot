"""順モデル（C7〜C9）のオフライン入口。

    .venv/bin/python -m tests.research.fwd_tool cells [--train CSV ...] [--check CSV ...]
        [--speed-step 5] [--u-step 2] [--min-s 1.0]
        （--speed-step 省略 = 15 以下 2.5・上は 5）
    .venv/bin/python -m tests.research.fwd_tool build [--train CSV ...] [--out PATH]
    .venv/bin/python -m tests.research.fwd_tool score [--map PKL] [--check CSV ...]
    .venv/bin/python -m tests.research.fwd_tool plan [--map PKL] [--mode NAME ...]
        [--ref-csv CSV] [--out-dir DIR]
    .venv/bin/python -m tests.research.fwd_tool model [--map PKL] [--train CSV ...]
        [--check CSV ...] [--ref-check CSV ...] [--save]
    .venv/bin/python -m tests.research.fwd_tool attach-pedal-track --pkl IN --csv CSV --out OUT
    .venv/bin/python -m tests.research.fwd_tool attach-local-gain --pkl IN --csv CSV --out OUT
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import re
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.drive_log import SECTION_PATTERN_DRIVE
from tests.research.ff_params import free_accel_at, research_ff_params
from tests.research.fwd_data import (
    DEFAULT_ACCEL_LAG_S,
    DEFAULT_ACCEL_WINDOW_S,
    DEFAULT_ACTUATOR_LAG_TOL_PCT,
    DEFAULT_BRAKE_LAG_S,
    DEFAULT_SECTIONS,
    DROP_REASONS,
    OPENING_ACTUAL,
    OPENING_CMD,
    SEGMENT_GAP_S,
    STOP_SPEED_KMH,
    Cell,
    FwdRows,
    cell_counts,
    concat_rows,
    effective_opening,
    make_speed_edges,
    read_fwd_rows,
    thin_cells,
)
from tests.research.fwd_map import (
    MODELS_DIR,
    POINT_FILLED,
    POINT_LABELS,
    POINT_MEASURED,
    POINT_OUT,
    ForwardTable,
    _row_weights,
    build_table,
    load_forward_map,
    save_table,
)
from tests.research.fwd_model import (
    fit_models,
    model_to_grid,
    monotone_violation,
)
from tests.research.fwd_plan import (
    DEFAULT_COAST_BAND_KMHS,
    FLAG_LABELS,
    FLAG_OUT,
    FLAG_STOP,
    build_plan,
    effective_u_series,
    simulate_speed,
    switch_times_of,
)
from tests.research.vehicle import feedforward_params

RESULTS = Path("tests/research/results")
DEFAULT_TRAIN = [
    RESULTS / "drive_log_real_20261002_034517.csv",  # 手順2
    RESULTS / "drive_log_real_20261001_142410.csv",  # WLTP
    RESULTS / "drive_log_real_20261001_040027.csv",  # WLTP
    RESULTS / "drive_log_real_20260930_155834.csv",  # WLTP
]
DEFAULT_CHECK = [RESULTS / "drive_log_real_20261002_052422.csv"]
DEFAULT_C7_MAP = RESULTS / "models" / "fwd_table_20261003_051808.pkl"
COMPARE_SPEED_STEPS = (5.0, 10.0)
COMPARE_U_STEPS = (1.0, 2.0, 4.0)
TABLE_MAX_COLS = 16


def _g(x: float) -> str:
    return f"{x:g}"


def u_label(k: int, step: float) -> str:
    """u のマスの見出し。惰行は「0」、正は上端「+k*step」（(k-1)*step 超〜k*step）、負は下端。"""
    if k == 0:
        return "0"
    return f"+{_g(k * step)}" if k > 0 else f"{_g(k * step)}"


def speed_label(k: int, step: float, edges: np.ndarray | None = None) -> str:
    if edges is not None:
        return f"{_g(float(edges[k]))}-{_g(float(edges[k + 1]))}"
    return f"{_g(k * step)}-{_g((k + 1) * step)}"


def md_table(header: Sequence[str], body: Sequence[Sequence[str]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    lines += ["| " + " | ".join(r) + " |" for r in body]
    return "\n".join(lines)


def grid_tables(
    counts: dict[Cell, float],
    speed_step: float,
    u_step: float,
    min_s: float,
    edges: np.ndarray | None = None,
) -> str:
    """車速 × u の秒数の表。ブレーキ側（u<0）と、惰行＋アクセル側（u>=0）に分ける。"""
    if not counts:
        return "（行なし）"
    s_lo = min(s for s, _ in counts)
    s_hi = max(s for s, _ in counts)
    u_all = sorted({u for _, u in counts})
    groups = [
        ("ブレーキ側（u < 0）", [u for u in u_all if u < 0]),
        ("惰行（u = 0）とアクセル側（u > 0）", [u for u in u_all if u >= 0]),
    ]
    out: list[str] = []
    for title, us in groups:
        # 0 秒の列は表から外す。列が多いときは TABLE_MAX_COLS ごとに折り返す
        for i in range(0, max(len(us), 1), TABLE_MAX_COLS):
            chunk = us[i : i + TABLE_MAX_COLS]
            if not chunk:
                continue
            body = []
            for s in range(s_lo, s_hi + 1):
                cells = []
                for u in chunk:
                    sec = counts.get((s, u), 0.0)
                    cells.append("" if sec <= 0 else f"{sec:.1f}" + ("*" if sec < min_s else ""))
                body.append([speed_label(s, speed_step, edges), *cells])
            out.append(f"**{title}**\n")
            out.append(md_table(["車速\\u", *[u_label(u, u_step) for u in chunk]], body))
            out.append("")
    return "\n".join(out)


def filled(counts: dict[Cell, float], min_s: float) -> int:
    return sum(1 for v in counts.values() if v >= min_s)


def run_cells(args: argparse.Namespace) -> int:
    veh = load_config(DEFAULT_CONFIG_PATH).vehicle
    kw = {
        "accel_deadband_pct": veh.min_accel_opening_pct,
        "brake_deadband_pct": veh.min_brake_opening_pct,
        "actuator_lag_tol_pct": args.u_step,  # 表のマス幅に合わせる（ずれ超えの行は外す）
    }
    train_logs = [read_fwd_rows(Path(p), **kw) for p in args.train]
    check_logs = [read_fwd_rows(Path(p), **kw) for p in skip_missing(args.check, "検証ログ")]
    train = concat_rows(train_logs, "学習ログ全体")
    check = concat_rows(check_logs, "検証ログ")
    us, mn = args.u_step, args.min_s
    if args.speed_step is None:
        top = max(float(train.speed_kmh.max()), float(check.speed_kmh.max()))
        edges: np.ndarray | None = make_speed_edges(top)
        ss = 5.0
        step_label = "15 km/h 以下 2.5・それより上 5"
    else:
        edges, ss = None, args.speed_step
        step_label = _g(ss)

    print("# 順モデルの学習データ: マスごとの秒数\n")
    print(
        f"u の原点（min）: アクセル {veh.min_accel_opening_pct} %・"
        f"ブレーキ {veh.min_brake_opening_pct} %。"
        f"刻み: 車速 {step_label} km/h × u {_g(us)} %。閾値: {_g(mn)} 秒。\n"
    )

    print("## (a) ログごとの使えた秒数と捨てた秒数\n")
    header = ["ログ", "区分", "使えた秒", *DROP_REASONS]
    body = []
    for kind, logs in (("学習", train_logs), ("検証", check_logs)):
        for r in logs:
            ds = r.dropped_seconds()
            dropped = [f"{ds.get(k, 0.0):.1f}" for k in DROP_REASONS]
            body.append([r.source, kind, f"{r.seconds:.1f}", *dropped])
    print(md_table(header, body))
    print("\n（捨てた秒数は上の列の順に最初に当てはまった理由に数える）\n")

    counts = cell_counts(train, ss, us, edges)
    print(f"## (b) 学習ログ全体の「車速 × u」の秒数（{step_label} km/h × {_g(us)} %）\n")
    print(
        "見出しの u は、正は上端（+2 は 0 超〜2 以下）、負は下端（−2 は −2 以上〜0 未満）、"
        "0 は惰行。"
        f"空欄は 0 秒、`*` は {_g(mn)} 秒未満。車速のマスは下端を含み上端を含まない。\n"
    )
    print(grid_tables(counts, ss, us, mn, edges))

    print("## (c) 手順2 のログと WLTP のログの秒数・埋まったマス数\n")
    p2 = [r for r in train_logs if SECTION_PATTERN_DRIVE in r.sections]
    wl = [r for r in train_logs if SECTION_PATTERN_DRIVE not in r.sections]
    body = []
    for name, logs in (("手順2", p2), ("WLTP", wl), ("合計", train_logs)):
        if not logs:
            body.append([name, "0.0", "0", "0"])
            continue
        c = cell_counts(concat_rows(logs), ss, us, edges)
        body.append(
            [
                name,
                f"{sum(r.seconds for r in logs):.1f}",
                str(len(c)),
                str(filled(c, mn)),
            ]
        )
    head_c = ["ログ", "使えた秒", "行のあるマス数", f"埋まったマス数（{_g(mn)} 秒以上）"]
    print(md_table(head_c, body))

    print(f"\n## (d) 検証ログが通ったマスのうち学習が {_g(mn)} 秒未満のマス\n")
    ccounts = cell_counts(check, ss, us, edges)
    thin = thin_cells(counts, ccounts, mn)
    if thin:
        body = [
            [
                speed_label(c.speed_idx, ss, edges),
                u_label(c.u_idx, us),
                f"{c.check_s:.2f}",
                f"{c.train_s:.2f}",
            ]
            for c in thin
        ]
        print(md_table(["車速", "u", "検証側の秒数", "学習側の秒数"], body))
    else:
        print("（なし）")
    tot = sum(c.check_s for c in thin)
    share = tot / check.seconds * 100 if check.seconds > 0 else 0.0
    print(
        f"\n合計 {tot:.1f} 秒 / 検証ログの使えた秒 {check.seconds:.1f} 秒 = {share:.1f} %"
        f"（マス数 {len(thin)}）\n"
    )

    print("## (e) 刻みを変えたときの比較\n")
    print(
        f"「穴の数」は検証ログが通ったマスのうち学習が {_g(mn)} 秒未満のマスの数、"
        "「穴の秒数」はそこに検証ログが居た秒数。\n"
    )
    body = []
    top_all = max(float(train.speed_kmh.max()), float(check.speed_kmh.max()))
    variants: list[tuple[str, float, np.ndarray | None]] = [
        ("2.5/5 混在", 5.0, make_speed_edges(top_all))
    ]
    variants += [(_g(x), x, None) for x in COMPARE_SPEED_STEPS]
    for sname, sstep, sedges in variants:
        for ustep in COMPARE_U_STEPS:
            tc = cell_counts(train, sstep, ustep, sedges)
            cc = cell_counts(check, sstep, ustep, sedges)
            th = thin_cells(tc, cc, mn)
            sec = sum(c.check_s for c in th)
            body.append(
                [
                    sname,
                    _g(ustep),
                    str(filled(tc, mn)),
                    str(len(th)),
                    f"{sec:.1f}",
                    f"{sec / check.seconds * 100 if check.seconds > 0 else 0.0:.1f} %",
                ]
            )
    head_e = ["車速刻み", "u 刻み", "埋まったマス数", "穴の数", "穴の秒数", "検証の秒に対する割合"]
    print(md_table(head_e, body))
    return 0


# ─────────────────────────────────────────────────────────────────────
# build / score / plan 共通
# ─────────────────────────────────────────────────────────────────────

MA_WINDOW_S = 3.0  # 外れの移動平均の窓 [s]（ユーザー指定: 3 秒）
SPEED_BANDS = ((0.0, 15.0), (15.0, 60.0), (60.0, 100.0), (100.0, 1e9))
REP_ROW_SPEEDS = (0.0, 10.0, 30.0, 60.0, 100.0, 125.0)  # build で u → 加速度を並べる行の下端
REP_U_POINTS = (-20.0, -10.0, -6.0, -4.0, -2.0, 0.0, 2.0, 4.0, 6.0, 10.0, 16.0, 24.0)
MOVE_REPORT_KMHS = 0.1  # 単調化で動いた点を一覧にするしきい値 [km/h/s]
MOVE_REPORT_MAX = 40  # 一覧の最大行数
DEFAULT_PLAN_MODES_EXTRA = "09_US06"
C1_COMPARE_LOG = RESULTS / "drive_log_real_20261002_052422.csv"
ZOOM_WINDOWS = (
    (None, None, "全体"),
    (0.0, 150.0, "先頭 0〜150 秒"),
    (1500.0, 1800.0, "高速域 1500〜1800 秒"),
)


@dataclass(frozen=True)
class DeadbandLag:
    """不感帯と、表の定義の遅れ L（指令ベース。時刻 t の車速・指令 → 時刻 t + L の加速度）。"""

    accel_deadband_pct: float
    brake_deadband_pct: float
    accel_lag_s: float = DEFAULT_ACCEL_LAG_S
    brake_lag_s: float = DEFAULT_BRAKE_LAG_S
    coast_lag_s: float | None = None  # None = accel_lag_s と同じ（惰行行は u = 0 で、アクセル側）
    opening_source: str = OPENING_CMD
    # 指令と実開度のずれでその行を学習から外す許容 [%]。表のマス幅に合わせる。None で外さない
    actuator_lag_tol_pct: float | None = DEFAULT_ACTUATOR_LAG_TOL_PCT


def _settings(cfg_path: Path = DEFAULT_CONFIG_PATH) -> DeadbandLag:
    veh = load_config(cfg_path).vehicle
    return DeadbandLag(veh.min_accel_opening_pct, veh.min_brake_opening_pct)


def settings_of_table(table: ForwardTable, base: DeadbandLag) -> DeadbandLag:
    """表を作ったときの遅れ L・開度の元（pkl の meta）に合わせた設定。無ければ base のまま。"""
    m = table.meta
    la, lb = m.get("accel_lag_s"), m.get("brake_lag_s")
    lc = m.get("coast_lag_s")
    return replace(
        base,
        accel_lag_s=base.accel_lag_s if la is None or not math.isfinite(la) else float(la),
        brake_lag_s=base.brake_lag_s if lb is None or not math.isfinite(lb) else float(lb),
        coast_lag_s=lc if lc is not None and math.isfinite(lc) else None,
        opening_source=m.get("opening_source", base.opening_source),
    )


def read_logs(paths: Sequence[str | Path], st: DeadbandLag, **over: object) -> list[FwdRows]:
    """CSV から学習用の行を作る。遅れ・開度の元は over で差し替えられる
    （accel_lag_s・brake_lag_s・coast_lag_s・opening_source）。"""
    kw: dict[str, object] = {
        "accel_deadband_pct": st.accel_deadband_pct,
        "brake_deadband_pct": st.brake_deadband_pct,
        "accel_lag_s": st.accel_lag_s,
        "brake_lag_s": st.brake_lag_s,
        "coast_lag_s": st.coast_lag_s,
        "opening_source": st.opening_source,
        "actuator_lag_tol_pct": st.actuator_lag_tol_pct,
    }
    kw.update(over)
    return [read_fwd_rows(Path(p), **kw) for p in paths]  # type: ignore[arg-type]


def table_from(
    rows_list: Sequence[FwdRows],
    st: DeadbandLag,
    *,
    edges: np.ndarray | None = None,
    min_s: float = 1.0,
    u_step: float = 2.0,
    **over: object,
) -> ForwardTable:
    rows = concat_rows(list(rows_list), "+".join(r.source for r in rows_list))
    top = float(rows.speed_kmh.max())
    la = float(over.get("accel_lag_s", st.accel_lag_s))  # type: ignore[arg-type]
    lb = float(over.get("brake_lag_s", st.brake_lag_s))  # type: ignore[arg-type]
    lc = over.get("coast_lag_s", st.coast_lag_s)
    return build_table(
        rows,
        speed_edges=make_speed_edges(top) if edges is None else edges,
        u_step_pct=u_step,
        min_s=min_s,
        accel_deadband_pct=st.accel_deadband_pct,
        brake_deadband_pct=st.brake_deadband_pct,
        accel_lag_s=la,
        brake_lag_s=lb,
        accel_window_s=DEFAULT_ACCEL_WINDOW_S,
        extra_meta={
            "coast_lag_s": la if lc is None else float(lc),  # type: ignore[arg-type]
            "opening_source": over.get("opening_source", st.opening_source),
        },
    )


def skip_missing(paths: Sequence[str | Path] | None, label: str) -> list[str]:
    """ファイルとして無い CSV をとばす（とばしたことを 1 行表示）。固定の比較ログは実行環境に
    無いことがあるため、採点・その行だけをとばして続ける。"""
    out: list[str] = []
    for p in paths or []:
        if Path(p).exists():
            out.append(str(p))
        else:
            print(f"- {label}: `{p}` が無いのでとばします")
    return out


def fmt(x: float, nd: int = 3) -> str:
    return "-" if not math.isfinite(x) else f"{x:.{nd}f}"


# ─────────────────────────────────────────────────────────────────────
# build
# ─────────────────────────────────────────────────────────────────────


def run_build(args: argparse.Namespace) -> int:
    cfg = load_config(DEFAULT_CONFIG_PATH)
    st = _settings()
    logs = read_logs(args.train, st)
    t = table_from(logs, st, min_s=args.min_s, u_step=args.u_step)
    edges = t.speed_edges
    out = save_table(t, Path(args.out) if args.out else None)
    meta = t.meta
    print("# 順モデルの表（C7）の作成\n")
    print(f"- 保存先: `{out}`")
    names = ', '.join(r.source for r in logs)
    print(f"- 学習ログ: {names}（使えた秒 {sum(r.seconds for r in logs):.1f}）")
    print(
        f"- 不感帯: アクセル {st.accel_deadband_pct}・ブレーキ {st.brake_deadband_pct} %、"
        f"遅れ: アクセル {st.accel_lag_s}・ブレーキ {st.brake_lag_s} s、"
        f"窓 {DEFAULT_ACCEL_WINDOW_S} s"
    )
    print(
        f"- 車速の行 {len(t.speed_rep)} 本（境界 {', '.join(_g(float(e)) for e in edges)}）、"
        f"u の格子 {len(t.u_grid)} 点（{_g(float(t.u_grid[0]))}〜{_g(float(t.u_grid[-1]))} %）、"
        f"穴の基準 {_g(args.min_s)} 秒未満、重み: 穴埋め点 {meta['filled_weight_s']} 秒相当\n"
    )

    print("## (a) 行ごとの実測の u の範囲と点の数\n")
    body = []
    totals = {POINT_MEASURED: 0, POINT_FILLED: 0, POINT_OUT: 0}
    for r in range(len(t.speed_rep)):
        k = t.point_kind[r]
        cnt = {x: int(np.count_nonzero(k == x)) for x in totals}
        for x, n in cnt.items():
            totals[x] += n
        copied = r in meta["copied_rows"]
        body.append(
            [
                speed_label(r, 0.0, edges) + ("（写し）" if copied else ""),
                f"{_g(float(t.meas_u_min[r]))}〜{_g(float(t.meas_u_max[r]))}",
                str(cnt[POINT_MEASURED]),
                str(cnt[POINT_FILLED]),
                str(cnt[POINT_OUT]),
                f"{float(t.seconds[r].sum()):.1f}",
            ]
        )
    body.append(["合計", "", str(totals[POINT_MEASURED]), str(totals[POINT_FILLED]),
                 str(totals[POINT_OUT]), f"{float(t.seconds.sum()):.1f}"])
    head = ["車速の行", "実測の u の範囲", "実測", "穴埋め", "範囲外", "マスの秒数合計"]
    print(md_table(head, body))

    print("\n## (b) 単調化で動かした量\n")
    assert t.accel_before_mono is not None
    diff = np.abs(t.accel_grid - t.accel_before_mono)
    inside = t.point_kind != POINT_OUT
    print(
        f"- 点ごとの |前 − 後| の最大: {diff.max():.3f} km/h/s"
        f"（範囲内の点のみ: {diff[inside].max():.3f}）"
    )
    moved = np.argwhere((diff >= MOVE_REPORT_KMHS) & inside)
    print(
        f"- {MOVE_REPORT_KMHS} km/h/s 以上動いた範囲内の点: "
        f"{len(moved)} 点 / {int(inside.sum())} 点\n"
    )
    if len(moved):
        order = np.argsort(-diff[moved[:, 0], moved[:, 1]])[:MOVE_REPORT_MAX]
        body = []
        for r, j in moved[order]:
            body.append(
                [
                    speed_label(int(r), 0.0, edges),
                    _g(float(t.u_grid[j])),
                    POINT_LABELS[int(t.point_kind[r, j])],
                    f"{float(t.seconds[r, j]):.1f}",
                    fmt(float(t.accel_before_mono[r, j])),
                    fmt(float(t.accel_grid[r, j])),
                    fmt(float(diff[r, j])),
                ]
            )
        print(md_table(["車速の行", "u", "種類", "秒数", "前", "後", "動いた量"], body))
        if len(moved) > MOVE_REPORT_MAX:
            print(f"\n（大きい順に上位 {MOVE_REPORT_MAX} 件）")

    print("\n## (c) 代表の行の u → 加速度 [km/h/s]\n")
    print("`*` 穴埋め、`^` 範囲外。\n")
    for lo in REP_ROW_SPEEDS:
        r = int(np.clip(np.searchsorted(edges, lo + 1e-9, side="right") - 1, 0, len(edges) - 2))
        cols = sorted({int(np.argmin(np.abs(t.u_grid - u))) for u in REP_U_POINTS})
        head = ["u", *[_g(float(t.u_grid[j])) for j in cols]]
        row = ["加速度"]
        for j in cols:
            mark = {POINT_MEASURED: "", POINT_FILLED: "*", POINT_OUT: "^"}[int(t.point_kind[r, j])]
            row.append(f"{float(t.accel_grid[r, j]):.2f}{mark}")
        print(f"**{speed_label(r, 0.0, edges)} km/h**（代表車速 {float(t.speed_rep[r]):g}）\n")
        print(md_table(head, [row]))
        print()

    print("## (d) 惰行（u = 0）の加速度と config の惰行カーブ\n")
    print(
        "config 側は `free_accel_at`（クリープ域は +creep、それ以上は −惰行減速）。"
        "差 = 表 − config。\n"
    )
    fp = feedforward_params(cfg)
    rp = research_ff_params(cfg)
    body = []
    j0 = int(np.argmin(np.abs(t.u_grid)))
    for r, v in enumerate(t.speed_rep.tolist()):
        tab = float(t.coast_accel(v))
        conf = float(free_accel_at(fp, rp, v))
        body.append(
            [
                speed_label(r, 0.0, edges),
                f"{v:g}",
                f"{float(t.seconds[r, j0]):.1f}",
                POINT_LABELS[int(t.point_kind[r, j0])],
                fmt(tab, 2),
                fmt(conf, 2),
                fmt(tab - conf, 2),
            ]
        )
    head = ["車速の行", "代表車速", "惰行マスの秒数", "種類", "表 u=0", "config", "差"]
    print(md_table(head, body))
    return 0


# ─────────────────────────────────────────────────────────────────────
# score
# ─────────────────────────────────────────────────────────────────────


def moving_mean_abs(rows: FwdRows, err: np.ndarray, window_s: float = MA_WINDOW_S) -> np.ndarray:
    """外れの移動平均（符号つき。窓は時刻で ±window_s/2）。連続区間（行間 SEGMENT_GAP_S 未満）
    の中で取り、窓が区間からはみ出す行は NaN。"""
    out = np.full(len(err), np.nan)
    t = rows.t_s
    if len(t) == 0:
        return out
    brk = np.flatnonzero(~((np.diff(t) > 0.0) & (np.diff(t) < SEGMENT_GAP_S))) + 1
    bounds = [0, *brk.tolist(), len(t)]
    half = window_s / 2.0
    for a, b in zip(bounds[:-1], bounds[1:], strict=True):
        ts = t[a:b]
        e = err[a:b]
        cs = np.concatenate([[0.0], np.cumsum(e)])
        lo = np.searchsorted(ts, ts - half - 1e-9, side="left")
        hi = np.searchsorted(ts, ts + half + 1e-9, side="right")
        ok = (ts - half >= ts[0] - 0.06) & (ts + half <= ts[-1] + 0.06)
        mean = (cs[hi] - cs[lo]) / np.maximum(hi - lo, 1)
        out[a:b] = np.where(ok, mean, np.nan)
    return out


@dataclass
class ErrStats:
    n: int
    mean: float
    mae: float
    ma_p95: float
    ma_max: float


def err_stats(err: np.ndarray, ma: np.ndarray, mask: np.ndarray) -> ErrStats:
    e = err[mask]
    m = np.abs(ma[mask])
    m = m[np.isfinite(m)]
    if len(e) == 0:
        return ErrStats(0, math.nan, math.nan, math.nan, math.nan)
    return ErrStats(
        len(e),
        float(e.mean()),
        float(np.abs(e).mean()),
        float(np.percentile(m, 95)) if len(m) else math.nan,
        float(m.max()) if len(m) else math.nan,
    )


def residuals(table: ForwardTable, rows_list: Sequence[FwdRows]) -> dict[str, np.ndarray]:
    """行ごとの外れ = 実測の加速度 − accel(実車速, 実 u)、その 3 秒移動平均、車速・u。"""
    errs, mas, vs, us = [], [], [], []
    for r in rows_list:
        if len(r) == 0:
            continue
        e = r.accel_kmhs - table.accel(r.speed_kmh, r.u_pct)
        errs.append(e)
        mas.append(moving_mean_abs(r, e))
        vs.append(r.speed_kmh)
        us.append(r.u_pct)
    return {
        "err": np.concatenate(errs),
        "ma": np.concatenate(mas),
        "v": np.concatenate(vs),
        "u": np.concatenate(us),
    }


@dataclass
class Trace:
    """検証ログの時系列（積み上げ用）。u は両ペダル・空の行を直前の値で持ち越した列。"""

    t: np.ndarray
    v: np.ndarray
    u: np.ndarray


def read_traces(path: Path, st: DeadbandLag) -> list[Trace]:
    """CSV を連続区間（section が同じで行間 SEGMENT_GAP_S 未満）に分け、時刻・実車速・u を持つ。
    u は停車中の行も含めて計算する（両ペダル超え・開度が空の行だけ直前の値を持ち越す）。"""
    from tests.research.drive_log import actual_opening  # noqa: PLC0415
    from tests.research.fwd_data import _cmd_or_none  # noqa: PLC0415

    pick = _cmd_or_none if st.opening_source == OPENING_CMD else actual_opening
    segs: list[list[tuple[float, float, float]]] = []
    prev_t, prev_sec = math.nan, ""
    last_u = 0.0
    with Path(path).open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            sec = r.get("section") or ""
            try:
                t = float(r.get("elapsed_s") or "nan")
                v = float(r.get("actual_speed_kmh") or "nan")
            except ValueError:
                continue
            if sec not in DEFAULT_SECTIONS or not math.isfinite(t):
                continue
            if not segs or sec != prev_sec or not (0.0 < t - prev_t < SEGMENT_GAP_S):
                segs.append([])
            prev_t, prev_sec = t, sec
            ap, bp = pick(r, "accel"), pick(r, "brake")
            u = (
                None
                if ap is None or bp is None
                else effective_opening(ap, bp, st.accel_deadband_pct, st.brake_deadband_pct)
            )
            if u is not None:
                last_u = u
            segs[-1].append((t, v, last_u))
    return [
        Trace(np.array([x[0] for x in s]), np.array([x[1] for x in s]), np.array([x[2] for x in s]))
        for s in segs
        if len(s) >= 2
    ]


@dataclass
class MountainResult:
    t_start: float
    seconds: float
    v_start: float
    v_peak: float
    max_dev: float


def accumulate_mountains(
    table: ForwardTable, traces: Sequence[Trace], st: DeadbandLag
) -> tuple[list[MountainResult], np.ndarray]:
    """発進〜停車の山（実車速 ≥ STOP_SPEED_KMH が続く区間）ごとに、山の頭の実車速から
    車速を積み上げる。

    指令ベース（表の定義: 時刻 t の車速・指令 u → 時刻 t + L の加速度）の向き:
    時刻 τ の加速度 = accel(v_sim(τ − L), u_cmd(τ − L))。v_sim(τ + dt) = v_sim(τ) + 加速度 × dt。
    L は u(τ − accel_lag) ≥ 0 ならアクセルの遅れ、そうでなく u(τ − brake_lag) < 0 ならブレーキの
    遅れ、どちらでもなければ（惰行）アクセルの遅れ。山の頭より前を指すときは頭の実車速を使う。
    停車中は積み上げない（山ごとに実車速に戻す）。両ペダル・開度が空の行は u を直前の値で持ち越す。
    返り値: 山ごとの結果、全行の |v_sim − 実車速|。"""
    results: list[MountainResult] = []
    devs: list[np.ndarray] = []
    for tr in traces:
        up = np.isfinite(tr.v) & (tr.v >= STOP_SPEED_KMH)
        edge = np.diff(np.concatenate([[0], up.astype(int), [0]]))
        for i0, i1 in zip(np.flatnonzero(edge == 1), np.flatnonzero(edge == -1), strict=True):
            if i1 - i0 < 2:
                continue
            vs = simulate_speed(
                table, tr.t, tr.u, int(i0), int(i1), float(tr.v[i0]), st.accel_lag_s, st.brake_lag_s
            )
            d = np.abs(vs - tr.v[i0:i1])
            devs.append(d)
            results.append(
                MountainResult(
                    float(tr.t[i0]),
                    float(tr.t[i1 - 1] - tr.t[i0]),
                    float(tr.v[i0]),
                    float(np.nanmax(tr.v[i0:i1])),
                    float(d.max()),
                )
            )
    return results, (np.concatenate(devs) if devs else np.zeros(0))


def pstats(x: np.ndarray) -> tuple[float, float]:
    return (float(np.percentile(x, 95)), float(x.max())) if len(x) else (math.nan, math.nan)


LAG_STEP_S = 0.05
LAG_RANGE_ACCEL_S = 0.6  # 振る範囲の最初の上端。最小が上端なら 0.2 ずつ広げる
LAG_RANGE_BRAKE_S = 0.5
LAG_RANGE_MAX_S = 1.5


def eval_lags(
    train_paths: Sequence[str | Path],
    check_paths: Sequence[str | Path],
    st: DeadbandLag,
    **over: object,
) -> dict[str, ErrStats]:
    """遅れ・開度の元を over で差し替えて学習・検証の行を作り直し、表を作って (i) を出す。"""
    tl = read_logs(train_paths, st, **over)
    cl = read_logs(check_paths, st, **over)
    tb = table_from(tl, st, **over)
    rr = residuals(tb, cl)
    err, ma, u = rr["err"], rr["ma"], rr["u"]
    return {
        "全体": err_stats(err, ma, np.ones(len(err), dtype=bool)),
        "アクセル行": err_stats(err, ma, u > 0),
        "ブレーキ行": err_stats(err, ma, u < 0),
        "惰行行": err_stats(err, ma, u == 0),
    }


def sweep_axis(
    fn: Callable[[float], dict[str, ErrStats]], group: str, hi: float
) -> tuple[dict[float, dict[str, ErrStats]], float]:
    """L を 0 から hi まで 0.05 刻みで振り、group の 3 秒平均 p95 が最小の L を返す。
    最小が上端なら 0.2 ずつ広げて、端でなくなるまで（上限 LAG_RANGE_MAX_S）確認する。"""
    res: dict[float, dict[str, ErrStats]] = {}
    cur = hi
    while True:
        for k in range(int(round(cur / LAG_STEP_S)) + 1):
            lag = round(k * LAG_STEP_S, 2)
            if lag not in res:
                res[lag] = fn(lag)
        best = min(res, key=lambda x: res[x][group].ma_p95)
        if best >= cur - 1e-9 and cur < LAG_RANGE_MAX_S:
            cur = round(cur + 0.2, 2)
            continue
        return res, best


def print_sweep(res: dict[float, dict[str, ErrStats]], group: str, best: float) -> None:
    body = []
    for lag in sorted(res):
        g, a = res[lag][group], res[lag]["全体"]
        mark = "★" if lag == best else ""
        body.append(
            [f"{lag:g}{mark}", str(g.n), fmt(g.mean), fmt(g.mae), fmt(g.ma_p95), fmt(g.ma_max),
             fmt(a.mae), fmt(a.ma_p95)]
        )
    head = ["L [s]", f"{group} 行数", "平均", "MAE", "3 秒平均 p95", "同 最大"]
    head += ["全体 MAE", "全体 p95"]
    print(md_table(head, body))
    print(
        f"\n{group}の 3 秒平均 p95 が最小の L: {best:g} s"
        f"（p95 {res[best][group].ma_p95:.3f}、MAE {res[best][group].mae:.3f}）。"
        f"{'0 が端（負の L は意味が無いので広げない）。' if best == 0.0 else ''}\n"
    )


def run_lag_sweep(args: argparse.Namespace, st: DeadbandLag) -> None:
    """(iii) 指令ベースの遅れ L の振り。アクセル行の外れはアクセルの L だけ、ブレーキ行の外れは
    ブレーキの L だけで決まるので、片方ずつ振る（他方は直前に決めた値）。"""
    print("\n## (iii) 遅れ L の確認（指令ベース）\n")
    print("学習・検証とも同じ L で行を作り直し、表も作り直す。\n")
    print(
        "L は「時刻 t の指令 → 時刻 t + L の加速度」の遅れ。0.05 刻み。"
        "惰行行（u = 0）はアクセルの L を使う（`coast_lag_s` を省略したとき）。"
        "最小が上端なら範囲を広げて確認する。\n"
    )
    ev = lambda **o: eval_lags(args.train, args.check, st, **o)  # noqa: E731
    print("### アクセル側（ブレーキの L = 0、惰行の L = アクセルと同じ）\n")
    res_a, best_a = sweep_axis(
        lambda x: ev(accel_lag_s=x, brake_lag_s=0.0, coast_lag_s=None),
        "アクセル行",
        LAG_RANGE_ACCEL_S,
    )
    print_sweep(res_a, "アクセル行", best_a)
    print(f"### ブレーキ側（アクセルの L = {best_a:g}、惰行の L = アクセルと同じ）\n")
    res_b, best_b = sweep_axis(
        lambda x: ev(accel_lag_s=best_a, brake_lag_s=x, coast_lag_s=None),
        "ブレーキ行",
        LAG_RANGE_BRAKE_S,
    )
    print_sweep(res_b, "ブレーキ行", best_b)
    print(f"### 惰行行だけの L（アクセル {best_a:g}・ブレーキ {best_b:g} に固定）\n")
    res_c, best_c = sweep_axis(
        lambda x: ev(accel_lag_s=best_a, brake_lag_s=best_b, coast_lag_s=x),
        "惰行行",
        LAG_RANGE_ACCEL_S,
    )
    print_sweep(res_c, "惰行行", best_c)
    print("### 選んだ L の表全体と、参考（実開度ベース・遅れ 0）\n")
    chosen = ev(accel_lag_s=best_a, brake_lag_s=best_b, coast_lag_s=None)
    actual = ev(accel_lag_s=0.0, brake_lag_s=0.0, coast_lag_s=None, opening_source=OPENING_ACTUAL)
    body = []
    for name, d in (
        (f"指令ベース L = アクセル {best_a:g}・ブレーキ {best_b:g}", chosen),
        ("実開度ベース L = 0・0（参考）", actual),
    ):
        for g in ("全体", "アクセル行", "ブレーキ行", "惰行行"):
            x = d[g]
            body.append([name, g, str(x.n), fmt(x.mean), fmt(x.mae), fmt(x.ma_p95), fmt(x.ma_max)])
    print(md_table(["ベース", "区分", "行数", "平均", "MAE", "3 秒平均 p95", "同 最大"], body))


def run_score(args: argparse.Namespace) -> int:
    base = _settings()
    args.check = skip_missing(args.check, "検証ログ")
    if not args.check:
        print("- 検証ログが無いので採点をとばします")
        return 0
    if args.sweep_only:
        print("# 順モデルの採点: 遅れ L の振りだけ\n")
        run_lag_sweep(args, base)
        return 0
    if args.map:
        table = load_forward_map(Path(args.map))
        st = settings_of_table(table, base)
        src = f"`{args.map}`"
        train_logs = read_logs(args.train, st)
        check_logs = read_logs(args.check, st)
    else:
        st = base
        train_logs = read_logs(args.train, st)
        check_logs = read_logs(args.check, st)
        table = table_from(train_logs, st)
        src = "学習ログからその場で作った表"
    print("# 順モデルの採点（検証ログ）\n")
    print(f"- 表: {src}")
    src_name = "最終指令" if st.opening_source == OPENING_CMD else "実"
    print(
        f"- 表の定義: 時刻 t の車速 × 時刻 t の{src_name}開度 → 時刻 t + L の加速度。"
        f"L: アクセル {st.accel_lag_s:g} s・ブレーキ {st.brake_lag_s:g} s・"
        f"惰行（u = 0）は {'アクセルと同じ' if st.coast_lag_s is None else f'{st.coast_lag_s:g} s'}"
    )
    cnames = ', '.join(r.source for r in check_logs)
    csec = sum(r.seconds for r in check_logs)
    print(f"- 検証ログ: {cnames}（使えた秒 {csec:.1f}）")
    print(f"- 学習ログ: {', '.join(r.source for r in train_logs)}\n")

    print("## (i) 加速度の外れ（実測 − 順モデル）[km/h/s]\n")
    print(
        "外れ = 実測の加速度 − accel(実車速, 実 u)（u は表と同じ開度の元・同じ L で作った行）。"
        "「3 秒平均」は外れを時刻順の連続区間の中で"
        f"±{MA_WINDOW_S / 2:g} 秒の窓で平均した値の |絶対値|（窓が区間からはみ出す行は除く）。\n"
    )
    res = residuals(table, check_logs)
    err, ma, v, u = res["err"], res["ma"], res["v"], res["u"]
    groups: list[tuple[str, np.ndarray]] = [
        ("全体", np.ones(len(err), dtype=bool)),
        ("アクセル行（u > 0）", u > 0),
        ("ブレーキ行（u < 0）", u < 0),
        ("惰行行（u = 0）", u == 0),
    ]
    for lo, hi in SPEED_BANDS:
        groups.append(
            (f"車速 {_g(lo)}–{'' if hi > 1e8 else _g(hi)} km/h", (v >= lo) & (v < hi))
        )
    body = []
    for name, m in groups:
        s = err_stats(err, ma, m)
        body.append([name, str(s.n), fmt(s.mean), fmt(s.mae), fmt(s.ma_p95), fmt(s.ma_max)])
    print(md_table(["区分", "行数", "平均", "MAE", "3 秒平均 外れ p95", "同 最大"], body))
    tres = residuals(table, train_logs)
    s = err_stats(tres["err"], tres["ma"], np.ones(len(tres["err"]), dtype=bool))
    print(
        f"\n参考（当てはまりの上限）: 学習ログ自身 — 行数 {s.n}、平均 {fmt(s.mean)}、"
        f"MAE {fmt(s.mae)}、"
        f"3 秒平均 p95 {fmt(s.ma_p95)}、最大 {fmt(s.ma_max)}\n"
    )

    print("## (ii) 積み上げた車速のずれ（山ごと）\n")
    print(
        "検証ログの「発進〜停車」の山（実車速 ≥ 0.5 km/h の連続区間）ごとに、山の頭の実車速から"
        "車速を積み上げ、実車速との差 絶対差 |v_sim − 実車速| を見る。"
        "指令ベース（表の定義）の向き: 時刻 τ の加速度 = accel(v_sim(τ − L), u_cmd(τ − L))、"
        "v_sim(τ + dt) = v_sim(τ) + 加速度 × dt。"
        f"L は u(τ − {st.accel_lag_s:g}) ≥ 0 ならアクセルの {st.accel_lag_s:g} s、"
        f"そうでなく u(τ − {st.brake_lag_s:g}) が負ならブレーキの {st.brake_lag_s:g} s、"
        "どちらでもなければ（惰行）アクセルの L。"
        "両ペダル・開度が空の行は u を直前の値で持ち越す。\n"
    )
    traces = [tr for p in args.check for tr in read_traces(Path(p), st)]
    mount, devs = accumulate_mountains(table, traces, st)
    print_mountains(mount, devs)

    if not args.no_sweep:
        run_lag_sweep(args, st)
    else:
        print("\n## (iii) 遅れの確認\n\n（--no-sweep のため省略）")

    print("\n## (iv) 車速の刻みの比較\n")
    top = max(float(r.speed_kmh.max()) for r in [*train_logs, *check_logs])
    variants = [
        ("15 以下 2.5 / 上 5", make_speed_edges(top)),
        ("全部 5", make_speed_edges(top, 5.0, 15.0, 5.0)),
    ]
    body = []
    for name, ed in variants:
        tb = table_from(train_logs, st, edges=ed)
        rr = residuals(tb, check_logs)
        ss = err_stats(rr["err"], rr["ma"], np.ones(len(rr["err"]), dtype=bool))
        mt, dv = accumulate_mountains(tb, traces, st)
        p95, mx = pstats(dv)
        med_max = float(np.median([m.max_dev for m in mt])) if mt else math.nan
        body.append(
            [name, str(len(tb.speed_rep)), fmt(ss.mean), fmt(ss.mae), fmt(ss.ma_p95),
             fmt(ss.ma_max),
             fmt(p95, 2), fmt(mx, 2), fmt(med_max, 2)]
        )
    print(md_table(
        ["刻み", "行数", "(i) 平均", "(i) MAE", "(i) 3 秒平均 p95", "(i) 同 最大",
         "(ii) 全行 ずれ p95", "(ii) 同 最大", "(ii) 山ごとの最大の中央値"], body))
    return 0


def print_mountains(mount: Sequence[MountainResult], devs: np.ndarray) -> None:
    p95, mx = pstats(devs)
    mean_dev = float(devs.mean()) if len(devs) else math.nan
    maxes = np.array([m.max_dev for m in mount])
    print(
        f"- 山 {len(mount)} 個（行数 {len(devs)}）。全行の |v_sim − 実車速|: "
        f"平均 {fmt(mean_dev, 2)}、p95 {fmt(p95, 2)}、最大 {fmt(mx, 2)} km/h"
    )
    if len(maxes):
        print(
            f"- 山ごとの最大: 中央値 {fmt(float(np.median(maxes)), 2)}、"
            f"p95 {fmt(float(np.percentile(maxes, 95)), 2)}、"
            f"最大 {fmt(float(maxes.max()), 2)} km/h\n"
        )
        top = sorted(mount, key=lambda m: -m.max_dev)[:10]
        body = [
            [f"{m.t_start:.0f}", f"{m.seconds:.0f}", f"{m.v_start:.1f}", f"{m.v_peak:.1f}",
             f"{m.max_dev:.2f}"]
            for m in top
        ]
        print("山ごとの最大 |ずれ| の上位 10 件:\n")
        head = ["山の始まり [s]", "長さ [s]", "頭の実車速", "山の最高車速", "最大 |ずれ| [km/h]"]
        print(md_table(head, body))


# ─────────────────────────────────────────────────────────────────────
# plan
# ─────────────────────────────────────────────────────────────────────


def safe_name(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9_-]+", "_", name).strip("_")


async def _load_modes(names: Sequence[str]) -> list:
    from tests.research.mode_drive import load_mode  # noqa: PLC0415

    cfg = load_config(DEFAULT_CONFIG_PATH)
    return [await load_mode(cfg, n) for n in names]


def load_modes(args: argparse.Namespace) -> list:
    """基準車速のモード。--ref-csv があれば CSV から、無ければ DB から読む。"""
    if args.ref_csv:
        from tests.research.wltp_grid import mode_from_csv  # noqa: PLC0415

        return [mode_from_csv(Path(args.ref_csv))]
    wltp = load_config(DEFAULT_CONFIG_PATH).modes.wltp_mode_name
    names = args.mode or [wltp, DEFAULT_PLAN_MODES_EXTRA]
    try:
        return asyncio.run(_load_modes(names))
    except Exception as exc:
        print(f"エラー: 走行モードを DB から読めませんでした（{type(exc).__name__}: {exc}）。")
        print("手順3 の CSV から基準車速を渡すには `--ref-csv CSV` を使ってください。")
        raise SystemExit(1) from exc


def pedal_of(effort: np.ndarray) -> np.ndarray:
    return np.where(effort > 0, 1, np.where(effort < 0, -1, 0))


SIM_MOVING_KMH = 0.5  # 検算で「走行中」とみなす基準車速 [km/h]


@dataclass
class PlanContext:
    """plan の計算に共通の入力。"""

    table: ForwardTable
    st: DeadbandLag
    stop_pct: float
    stop_horizon_s: float
    band: float

    def build(self, ref, dur: float, **over):
        kw = {
            "accel_deadband_pct": self.st.accel_deadband_pct,
            "brake_deadband_pct": self.st.brake_deadband_pct,
            "stop_brake_opening_pct": self.stop_pct,
            "stop_horizon_s": self.stop_horizon_s,
            "coast_band_kmhs": self.band,
        }
        kw.update(over)
        return build_plan(ref, dur, self.table, **kw)


def run_plan(args: argparse.Namespace) -> int:
    from tests.research.mode_drive import ReferenceSpeed  # noqa: PLC0415

    cfg = load_config(DEFAULT_CONFIG_PATH)
    base = _settings()
    if args.map:
        table = load_forward_map(Path(args.map))
        st = settings_of_table(table, base)
    else:
        st = base
        table = table_from(read_logs(args.train, st), st)
    ctx = PlanContext(
        table, st, cfg.stop_brake_opening_pct, cfg.features.h0_s, args.coast_band
    )
    modes = load_modes(args)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    print("# 開度の列（基準車速 → 順モデルの逆引き）\n")
    print(
        f"- 表: {args.map or '学習ログからその場で作った表'}\n"
        f"- 不感帯 アクセル {st.accel_deadband_pct}・ブレーキ {st.brake_deadband_pct} %、"
        f"停車保持 {ctx.stop_pct} %、停車判定の先読み {ctx.stop_horizon_s} s\n"
        f"- 遅れ L（表の meta）: アクセル {st.accel_lag_s:g}・ブレーキ {st.brake_lag_s:g} s、"
        f"要る加速度の窓 1.0 s、惰行の帯 ±{ctx.band:g} km/h/s\n"
        "- 時刻 t の指令 = invert(v = 基準(t), a = 基準の窓の傾き(t + L))\n"
    )
    for mode in modes:
        ref = ReferenceSpeed(mode).at
        dur = float(mode.reference_speed[-1].time_s)
        plan = ctx.build(ref, dur)
        report_plan(mode.name, plan, ref, ctx, out_dir, stamp)
        report_check_sim(mode.name, plan, ref, dur, ctx)
        if mode.name == cfg.modes.wltp_mode_name or args.ref_csv:
            if skip_missing([args.c1_log], "C1 比較ログ"):
                compare_c1(mode.name, plan, ref, out_dir, stamp, Path(args.c1_log))
    return 0


def switch_summary(t: np.ndarray, effort: np.ndarray, dt: float, stop: np.ndarray | None = None):
    """切替回数・切替の間隔（連続する切替の時刻の差）が 1 秒未満・3 秒未満の回数・惰行の秒数。
    フェーズは effort の符号（正 = アクセル、負 = ブレーキ。停車保持はブレーキに含める）。"""
    from tests.research.ff_bias import PEDAL_ACCEL, PEDAL_BRAKE, PEDAL_COAST  # noqa: PLC0415

    phases = [PEDAL_ACCEL if e > 0 else PEDAL_BRAKE if e < 0 else PEDAL_COAST for e in effort]
    times = np.array(switch_times_of(t, phases))
    gaps = np.diff(times) if len(times) > 1 else np.zeros(0)
    return {
        "切替回数": len(times),
        "間隔 1 秒未満": int(np.count_nonzero(gaps < 1.0)),
        "間隔 3 秒未満": int(np.count_nonzero(gaps < 3.0)),
        "惰行の秒数": float(np.count_nonzero(effort == 0)) * dt,
    }


def print_switch_rows(rows: list[tuple[str, dict]]) -> None:
    keys = list(rows[0][1])
    def cell(x: object) -> str:
        return f"{x:.1f}" if isinstance(x, float) else str(x)

    body = [[name, *[cell(d[k]) for k in keys]] for name, d in rows]
    print(md_table(["", *keys], body))
    print()


def oor_breakdown(plan, rv: np.ndarray, table: ForwardTable) -> dict[str, float]:
    """範囲外の秒数を、要る加速度が表の最大より大きい／最小より小さい／実測の範囲外の点にかかった、
    に分ける（車速は基準(t)、要る加速度は plan に残した値）。"""
    m = plan.flag == FLAG_OUT
    cmax = np.asarray(table.accel(rv, float(table.u_grid[-1])))
    cmin = np.asarray(table.accel(rv, float(table.u_grid[0])))
    need = plan.need_accel_kmhs
    hi = m & (need > cmax + 1e-9)
    lo = m & ~hi & (need < cmin - 1e-9)
    mid = m & ~hi & ~lo
    d = plan.period_s
    return {
        "アクセル側（要る加速度 > 表の最大）": float(hi.sum()) * d,
        "ブレーキ側（要る加速度 < 表の最小）": float(lo.sum()) * d,
        "実測の範囲外の点にかかった": float(mid.sum()) * d,
    }


def report_plan(name, plan, ref, ctx: PlanContext, out_dir: Path, stamp: str) -> None:
    print(f"## モード {name}（{plan.t_s[-1]:.0f} 秒、{len(plan.t_s)} 点）\n")
    ps = plan.pedal_seconds()
    print(
        f"- 範囲外 {plan.out_of_range_s():.2f} 秒・穴埋め {plan.filled_s():.2f} 秒"
        f"（実測 {plan.seconds_of(0):.1f}・惰行 {plan.seconds_of(3):.1f}・"
        f"停車保持 {plan.seconds_of(FLAG_STOP):.1f} 秒）"
    )
    print("- ペダル別: " + "、".join(f"{k} {v:.1f} 秒" for k, v in ps.items()))
    e = plan.effort_pct
    stop = plan.flag == FLAG_STOP
    print(
        f"- 開度の最大: アクセル {float(e.max()):.1f} %、ブレーキ {float(-e[~stop].min()):.1f} %"
        f"（停車保持を除く）\n"
    )
    print("切替（アクセル⇔ブレーキ。停車保持はブレーキに含める）:\n")
    print_switch_rows([("C7 の列", switch_summary(plan.t_s, e, plan.period_s))])
    rv = np.array([ref(float(x)) for x in plan.t_s])
    if plan.out_of_range_s() > 0:
        print("範囲外の秒数の内訳:\n")
        body = [[k, f"{v:.2f}"] for k, v in oor_breakdown(plan, rv, ctx.table).items()]
        print(md_table(["内訳", "秒数"], body))
        print()
    # 範囲外・穴埋めが起きた車速（基準(t)）× 要る加速度
    bad = (plan.flag == POINT_OUT) | (plan.flag == POINT_FILLED)
    if bad.any():
        key = list(zip(
            (np.floor(rv[bad] / 5.0) * 5.0).tolist(),
            (np.floor(plan.need_accel_kmhs[bad] / 0.5) * 0.5).tolist(),
            plan.flag[bad].tolist(),
            strict=True,
        ))
        agg: dict[tuple[float, float, int], int] = {}
        for k in key:
            agg[k] = agg.get(k, 0) + 1
        top = sorted(agg.items(), key=lambda kv: -kv[1])[:10]
        body = [
            [f"{k[0]:g}–{k[0] + 5:g}", f"{k[1]:g}–{k[1] + 0.5:g}", FLAG_LABELS[k[2]],
             f"{n * plan.period_s:.2f}"]
            for k, n in top
        ]
        print("範囲外・穴埋めが起きた車速 × 要る加速度（秒数の上位 10 件）:\n")
        print(md_table(["車速 [km/h]", "要る加速度 [km/h/s]", "種類", "秒数"], body))
        print()
    else:
        print("範囲外・穴埋めは 0 秒。\n")
    path = out_dir / f"fwd_plan_{safe_name(name)}_{stamp}.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            ["t_s", "ref_kmh", "need_accel_kmhs", "effort_pct", "accel_pct", "brake_pct", "flag"]
        )
        cols = zip(plan.t_s, rv, e, plan.need_accel_kmhs, plan.flag, strict=True)
        for t, r, ev, need, fl in cols:
            w.writerow(
                [f"{t:.3f}", f"{r:.3f}", f"{need:.4f}", f"{ev:.3f}",
                 f"{max(ev, 0.0):.3f}", f"{max(-ev, 0.0):.3f}", FLAG_LABELS[int(fl)]]
            )
    print(f"開度の列 CSV: `{path}`\n")


def check_sim(plan, ref, ctx: PlanContext) -> np.ndarray:
    """開度の列を表に通して車速を積み上げ、v_sim − 基準 を返す（表が正しいと仮定した計算）。
    v_sim(τ + dt) = v_sim(τ) + accel(v_sim(τ − L), u(τ − L)) × dt。u は effort から不感帯を引いた
    効く開度。停車保持の区間は v_sim = 0 に保ち、保持が外れたら積み上げを始める。"""
    st = ctx.st
    u = effective_u_series(plan.effort_pct, plan.flag, st.accel_deadband_pct, st.brake_deadband_pct)
    vs = simulate_speed(
        ctx.table, plan.t_s, u, 0, len(plan.t_s), 0.0, st.accel_lag_s, st.brake_lag_s,
        hold=plan.flag == FLAG_STOP,
    )
    return vs - np.array([ref(float(x)) for x in plan.t_s])


def report_check_sim(name, plan_a, ref, dur: float, ctx: PlanContext) -> None:
    print(f"### 検算: 開度の列を表に通して積み上げた車速のずれ（{name}）\n")
    print(
        "**表が正しいと仮定した計算で、実車の結果ではない。** "
        "v_sim(τ + dt) = v_sim(τ) + accel(v_sim(τ − L), u(τ − L)) × dt。"
        "ずれ = v_sim − 基準。「走行中」は基準 ≥ 0.5 km/h の点。\n"
    )
    rv = np.array([ref(float(x)) for x in plan_a.t_s])
    run = rv >= SIM_MOVING_KMH
    variants = [
        ("(a) 先読みあり（標準。帯 " + f"{ctx.band:g}）", plan_a),
        (
            "(b) 先読みなし（need(t)・v = ref(t)。帯は標準）",
            ctx.build(ref, dur, accel_lag_s=0.0, brake_lag_s=0.0),
        ),
        ("(c) 先読みあり・惰行の帯 0", ctx.build(ref, dur, coast_band_kmhs=0.0)),
    ]
    body = []
    diffs = {}
    for label, pl in variants:
        d = check_sim(pl, ref, ctx)
        diffs[label] = d
        a = np.abs(d[run])
        body.append(
            [label, fmt(float(d[run].mean()), 2), fmt(float(a.mean()), 2),
             fmt(float(np.percentile(a, 95)), 2), fmt(float(a.max()), 2),
             fmt(float(np.percentile(np.abs(d), 95)), 2)]
        )
    head = ["案", "走行中 平均（符号つき）", "走行中 平均 |差|", "走行中 |差| p95"]
    head += ["走行中 |差| 最大", "全点 |差| p95"]
    print(md_table(head, body))
    print()
    oor = plan_a.flag == FLAG_OUT
    if oor.any():
        a = np.abs(diffs[variants[0][0]])
        o = oor & run
        n = run & ~oor
        print("(a) の |差| を範囲外フラグの点とそれ以外（どちらも走行中）で分けた値:\n")
        p95o = float(np.percentile(a[o], 95)) if o.any() else math.nan
        maxo = float(a[o].max()) if o.any() else math.nan
        body = [
            ["範囲外の点", str(int(o.sum())), fmt(p95o, 2), fmt(maxo, 2)],
            ["それ以外", str(int(n.sum())), fmt(float(np.percentile(a[n], 95)), 2),
             fmt(float(a[n].max()), 2)],
        ]
        print(md_table(["区分", "点数", "|差| p95", "|差| 最大"], body))
        print()


def compare_c1(name, plan, ref, out_dir: Path, stamp: str, log: Path) -> None:
    """C1 の実走（FF 指令と実開度）と開度の列を mode_time_s で対応させて比べる。"""
    rows = []
    with log.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("section") != "MODE_DRIVE" or not r.get("mode_time_s"):
                continue
            try:
                rows.append(
                    [float(r["mode_time_s"]), float(r["accel_ff_pct"]), float(r["brake_ff_pct"]),
                     float(r["accel_actual_pct"]), float(r["brake_actual_pct"])]
                )
            except (ValueError, KeyError):
                continue
    a = np.array(rows)
    t, ffa, ffb, acta, actb = a.T
    eff = np.array([plan.at(float(x)) for x in t])
    c7a, c7b = np.maximum(eff, 0.0), np.maximum(-eff, 0.0)
    dt = float(np.median(np.diff(t)))
    pc7, pc1 = pedal_of(eff), np.where(ffa > 0, 1, np.where(ffb > 0, -1, 0))
    print(f"### C1 の実走との比較（{log.name}、行 {len(t)}、刻み {dt:.3f} s）\n")
    print(
        "差 = C7 の列 − C1 の FF 指令。どちらかがそのペダルを使っている行だけで集計。"
        "ペダル選択違い = アクセル／ブレーキ／惰行の判定が C7 と C1 で違う秒数。\n"
    )
    print("切替の比較（C1 は FF 指令 accel_ff − brake_ff の符号。C7 の列は同じ時刻で引いた値）:\n")
    print_switch_rows(
        [
            ("C1 の FF 指令", switch_summary(t, ffa - ffb, dt)),
            ("C7 の列", switch_summary(t, eff, dt)),
        ]
    )
    body = []
    for lo, hi, label in ZOOM_WINDOWS:
        m = np.ones(len(t), dtype=bool) if lo is None else (t >= lo) & (t < hi)
        for pname, c7, c1 in (("アクセル", c7a, ffa), ("ブレーキ", c7b, ffb)):
            mm = m & ((c7 > 0) | (c1 > 0))
            d = (c7 - c1)[mm]
            body.append(
                [label, pname, str(int(mm.sum())),
                 fmt(float(d.mean()) if len(d) else math.nan, 2),
                 fmt(float(np.percentile(d, 5)) if len(d) else math.nan, 2),
                 fmt(float(np.percentile(d, 95)) if len(d) else math.nan, 2)]
            )
        diff_s = float(np.count_nonzero(m & (pc7 != pc1))) * dt
        body.append([label, "選択違いの秒数", f"{diff_s:.1f}", "", "", ""])
    print(md_table(["範囲", "ペダル", "行数（秒）", "差の平均 [%]", "p5", "p95"], body))
    print()
    png = plot_compare(name, plan, ref, t, ffa, ffb, acta, actb, out_dir, stamp)
    print(f"比較図 PNG: `{png}`\n")


def plot_compare(name, plan, ref, t, ffa, ffb, acta, actb, out_dir: Path, stamp: str) -> Path:
    import matplotlib  # noqa: PLC0415

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: PLC0415

    from tests.research.live_plot import (  # noqa: PLC0415
        COLOR_ACCEL,
        COLOR_ACTUAL,
        COLOR_BRAKE,
        FONT_FAMILY,
    )

    e = plan.effort_pct
    rv = np.array([ref(float(x)) for x in plan.t_s])
    with matplotlib.rc_context({"font.family": FONT_FAMILY}):
        fig, axes = plt.subplots(3, 3, figsize=(18, 9), sharex="col")
        for c, (lo, hi, label) in enumerate(ZOOM_WINDOWS):
            tp = np.ones(len(plan.t_s), dtype=bool)
            if lo is not None:
                tp = (plan.t_s >= lo) & (plan.t_s <= hi)
            tc = np.ones(len(t), dtype=bool) if lo is None else (t >= lo) & (t <= hi)
            axes[0, c].plot(plan.t_s[tp], rv[tp], color="#444", lw=0.8)
            axes[0, c].set_title(label)
            for r, (c7, c1, act, color, pn) in enumerate(
                ((np.maximum(e, 0.0), ffa, acta, COLOR_ACCEL, "アクセル"),
                 (np.maximum(-e, 0.0), ffb, actb, COLOR_BRAKE, "ブレーキ")), start=1
            ):
                ax = axes[r, c]
                ax.plot(t[tc], act[tc], color=COLOR_ACTUAL, lw=0.6, label="実開度（C1 実走）")
                ax.plot(t[tc], c1[tc], color="#888", lw=0.8, label="C1 の FF")
                ax.plot(plan.t_s[tp], c7[tp], color=color, lw=0.9, label="C7 の列")
                ax.set_ylabel(f"{pn} [%]")
            axes[2, c].set_xlabel("モード時間 [s]")
        axes[0, 0].set_ylabel("基準車速 [km/h]")
        axes[1, 0].legend(loc="upper right", fontsize=8)
        fig.suptitle(f"開度の列（C7）と C1 の実走: {name}")
        fig.tight_layout()
        path = out_dir / f"fwd_plan_{safe_name(name)}_{stamp}.png"
        fig.savefig(path, dpi=110)
        plt.close(fig)
    return path


# ─────────────────────────────────────────────────────────────────────
# model（C8 の 3 種をオフラインで比べる）
# ─────────────────────────────────────────────────────────────────────

REF_NAME = "C7(参考)"
REGIONS = ("ブレーキ行(u<0)", "車速0-15", "C7の穴埋め・範囲外")


def hole_mask(table: ForwardTable, v: np.ndarray, u: np.ndarray) -> np.ndarray:
    """検証行の (車速, u) を挟む格子点（grid_accel と同じ 2 行 × 2 点）に、
    実測でない点（穴埋め・範囲外）を含む行。"""
    i0, i1, _ = _row_weights(table.speed_rep, v)
    ug = table.u_grid
    j0 = np.clip(np.searchsorted(ug, u, side="right") - 1, 0, len(ug) - 2)
    j1 = j0 + 1
    pk = table.point_kind
    worst = np.maximum.reduce([pk[i0, j0], pk[i0, j1], pk[i1, j0], pk[i1, j1]])
    return worst != POINT_MEASURED


@dataclass
class ModelEval:
    name: str
    stats: dict[str, ErrStats]  # 区分 → 外れ
    mono_max: float
    mono_mean: float
    drift_p95: float
    drift_max: float
    train_time_s: float


def eval_model(
    name: str,
    tb: ForwardTable,
    like: ForwardTable,
    check_logs: Sequence[FwdRows],
    traces: Sequence[Trace],
    st: DeadbandLag,
    *,
    is_ref: bool,
    train_time_s: float = math.nan,
) -> ModelEval:
    res = residuals(tb, check_logs)
    err, ma, v, u = res["err"], res["ma"], res["v"], res["u"]
    masks = {
        "全体": np.ones(len(err), dtype=bool),
        REGIONS[0]: u < 0,
        REGIONS[1]: (v >= SPEED_BANDS[0][0]) & (v < SPEED_BANDS[0][1]),
        REGIONS[2]: hole_mask(like, v, u),
    }
    stats = {k: err_stats(err, ma, m) for k, m in masks.items()}
    _, devs = accumulate_mountains(tb, traces, st)
    p95, mx = pstats(devs)
    if is_ref:
        vmax = vmean = math.nan
    else:
        vmax, vmean = monotone_violation(tb)
    return ModelEval(name, stats, vmax, vmean, p95, mx, train_time_s)


def print_model_table(evals: Sequence[ModelEval]) -> None:
    keys = ["全体", *REGIONS]
    header = ["モデル"]
    for k in keys:
        header += [f"{k} 3秒平均p95", f"{k} MAE"]
    header += ["単調化 最大", "単調化 平均", "備考", "積み上げ p95", "積み上げ 最大", "学習時間 s"]
    body = []
    for e in evals:
        row = [e.name]
        for k in keys:
            row += [fmt(e.stats[k].ma_p95), fmt(e.stats[k].mae)]
        judge = "-" if not math.isfinite(e.mono_max) else "単調化後を使う"
        row += [
            fmt(e.mono_max),
            fmt(e.mono_mean),
            judge,
            fmt(e.drift_p95, 2),
            fmt(e.drift_max, 2),
            fmt(e.train_time_s, 1),
        ]
        body.append(row)
    print(md_table(header, body))


def select_model(evals: Sequence[ModelEval]) -> tuple[str | None, str]:
    """C8 の 3 種（C7 は除く）から選ぶ。3 か所（ブレーキ行・0-15・穴埋め/範囲外）の 3 秒平均 p95 が
    2 か所以上で単独最小のモデル。同値（同じ予測のモデル）の場所は勝者なしとして数えない。
    決まらなければ全体の 3 秒平均 p95 が小さいほう。単調化で動いた量では外さない。"""
    cand = [e for e in evals if e.name != REF_NAME]
    if not cand:
        return None, "候補がない"
    wins = dict.fromkeys((e.name for e in cand), 0)
    detail = []
    for reg in REGIONS:
        vals = {e.name: e.stats[reg].ma_p95 for e in cand if math.isfinite(e.stats[reg].ma_p95)}
        if not vals:
            continue
        best = min(vals.values())
        w = [n for n, x in vals.items() if x <= best + 1e-6]
        if len(w) == 1:
            wins[w[0]] += 1
            detail.append(f"{reg}: {w[0]}")
        else:
            detail.append(f"{reg}: 同値（{'・'.join(w)}）で勝者なし")
    top = max(wins.values())
    leaders = [n for n, c in wins.items() if c == top]
    by_overall = {e.name: e.stats["全体"].ma_p95 for e in cand}
    pick = min(leaders, key=lambda n: by_overall[n])
    if top >= 2 and len(leaders) == 1:
        why = f"{top} か所で最小"
    elif top >= 2:
        why = f"{top} か所で最小が {len(leaders)} 種あり、全体の 3 秒平均 p95 が小さいほう"
    else:
        why = "2 か所以上で最小のモデルがなく、全体の 3 秒平均 p95 が最小"
    return pick, f"{why}（{'、'.join(detail)}）"


def model_default_path(now: datetime | None = None) -> Path:
    """results/models/fwd_model_<UTC>.pkl（pkl 名は UTC）。"""
    stamp = (now or datetime.now(UTC)).strftime("%Y%m%d_%H%M%S")
    return MODELS_DIR / f"fwd_model_{stamp}.pkl"


def _hole_seconds(like: ForwardTable, logs: Sequence[FwdRows]) -> tuple[int, float]:
    n, sec = 0, 0.0
    for r in logs:
        if len(r) == 0:
            continue
        m = hole_mask(like, r.speed_kmh, r.u_pct)
        n += int(m.sum())
        sec += float(r.dt_row_s[m].sum())
    return n, sec


def print_eval_block(
    like: ForwardTable,
    tables: dict[str, tuple[ForwardTable, float]],
    check_paths: Sequence[str],
    st: DeadbandLag,
) -> list[ModelEval]:
    check_logs = read_logs(check_paths, st)
    traces = [tr for p in check_paths for tr in read_traces(Path(p), st)]
    n_hole, s_hole = _hole_seconds(like, check_logs)
    print(
        f"- 検証ログ: {', '.join(r.source for r in check_logs)}"
        f"（使えた秒 {sum(r.seconds for r in check_logs):.1f}）"
    )
    print(
        f"- 「C7の穴埋め・範囲外」の行: {n_hole} 行・{s_hole:.1f} 秒"
        "（検証行の (車速, u) を挟む 2 行 × 2 点に実測でない点を含む行）\n"
    )
    evals = [
        eval_model(
            name, tb, like, check_logs, traces, st, is_ref=name == REF_NAME, train_time_s=tt
        )
        for name, (tb, tt) in tables.items()
    ]
    print_model_table(evals)
    return evals


def run_model(args: argparse.Namespace) -> int:
    base = _settings()
    args.check = skip_missing(args.check, "検証ログ")
    args.ref_check = skip_missing(args.ref_check, "参考ログ")
    like = load_forward_map(Path(args.map))
    st = settings_of_table(like, base)
    train_logs = read_logs(args.train, st)
    print("# 順モデルのモデル版（C8）3 種のオフライン比較\n")
    print(f"- C7 の表: `{args.map}`")
    src_name = "最終指令" if st.opening_source == OPENING_CMD else "実"
    print(
        f"- 表の定義: 時刻 t の車速 × 時刻 t の{src_name}開度 → 時刻 t + L の加速度。"
        f"L: アクセル {st.accel_lag_s:g} s・ブレーキ {st.brake_lag_s:g} s"
    )
    print(f"- 学習ログ（固定）: {', '.join(r.source for r in train_logs)}")
    fitted = fit_models(train_logs)
    for f in fitted.values():
        extra = (
            f"alpha={f.alpha:g}" if f.alpha is not None
            else f"隠れ層={f.hidden}" if f.hidden is not None else "-"
        )
        print(f"- {f.name}: {extra}、学習 {f.train_time_s:.1f} 秒")
    print("- 単調化: 行ごとの IsotonicRegression をかけた後の配列を使う（C7 と同じ。外さない）\n")
    grids: dict[str, ForwardTable] = {n: model_to_grid(f, like) for n, f in fitted.items()}
    tables: dict[str, tuple[ForwardTable, float]] = {REF_NAME: (like, math.nan)}
    tables.update({n: (grids[n], fitted[n].train_time_s) for n in grids})

    print("## 検証ログ（選定に使う）\n")
    evals = print_eval_block(like, tables, args.check, st)
    pick, why = select_model(evals)
    print("\n## 選び方\n")
    if pick is None:
        print(f"- 選べない: {why}")
    else:
        print(f"- 選んだモデル: **{pick}**（{why}）")
    print("- 採否（C7 と比べて C8 を採るか）は実機で決める。オフラインでは決めない。")

    if args.ref_check:
        print("\n## 参考ログ（参考のみ。選定にも学習にも使わない）\n")
        print_eval_block(like, tables, args.ref_check, st)

    if args.save and pick is not None:
        out = save_table(grids[pick], model_default_path())
        print(f"\n- 保存: `{out}`")
    elif args.save:
        print("\n- 選べるモデルがないため保存しない")
    return 0


def run_attach_pedal_track(args: argparse.Namespace) -> int:
    """既存の pkl に、手順2 の CSV から測ったペダルの動き方（PedalTrack の 8 値）を meta として
    付けて、新しい pkl に保存する（2026-10-26 段11-2。元の pkl は書き換えない）。"""
    from tests.research.fwd_map import load_forward_map  # noqa: PLC0415
    from tests.research.pattern_drive import _attach_pedal_track  # noqa: PLC0415

    src, out = Path(args.pkl), Path(args.out)
    if out.exists():
        print(f"保存先 {out} が既にある。上書きしない")
        return 2
    if out.resolve() == src.resolve():
        print("保存先が元の pkl と同じ。別の名前にする")
        return 2
    table = load_forward_map(src)
    _attach_pedal_track(table, Path(args.csv))
    if "pedal_track_accel_delay_s" not in table.meta:
        print("測れなかったので保存しない")
        return 1
    print(f"保存: {save_table(table, out)}")
    return 0


def run_attach_local_gain(args: argparse.Namespace) -> int:
    """既存の pkl に、手順2 の CSV の dither から測った局所の効きの倍率の表と効くまでの遅れ
    （local_gain_brake・local_gain_accel・dither_*_lag_s）を meta として付けて、新しい pkl に
    保存する
    （2026-10-28 段3。倍率はその pkl の表の傾きで割る。元の pkl は書き換えない）。"""
    from tests.research.fwd_map import load_forward_map  # noqa: PLC0415
    from tests.research.pattern_drive import _attach_local_gain  # noqa: PLC0415

    src, out = Path(args.pkl), Path(args.out)
    if out.exists():
        print(f"保存先 {out} が既にある。上書きしない")
        return 2
    if out.resolve() == src.resolve():
        print("保存先が元の pkl と同じ。別の名前にする")
        return 2
    table = load_forward_map(src)
    if not _attach_local_gain(table, Path(args.csv), args.brake_speeds):
        print("測れなかったので保存しない")
        return 1
    print(f"保存: {save_table(table, out)}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="fwd_tool", description="順モデル（C7〜C9）のオフライン道具")
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("cells", help="マスごとの秒数（刻みを決める材料）")
    c.add_argument("--train", nargs="+", default=[str(x) for x in DEFAULT_TRAIN], metavar="CSV")
    c.add_argument("--check", nargs="+", default=[str(x) for x in DEFAULT_CHECK], metavar="CSV")
    c.add_argument(
        "--speed-step", type=float, default=None,
        help="全部この刻み。省略すると 15 km/h 以下 2.5・それより上 5",
    )
    c.add_argument("--u-step", type=float, default=2.0)
    c.add_argument("--min-s", type=float, default=1.0)
    c.set_defaults(func=run_cells)

    def add_train(q: argparse.ArgumentParser) -> None:
        q.add_argument("--train", nargs="+", default=[str(x) for x in DEFAULT_TRAIN], metavar="CSV")

    b = sub.add_parser("build", help="表を作って pkl に保存する")
    add_train(b)
    b.add_argument("--out", default=None, help="保存先 pkl（省略: results/models/fwd_table_*.pkl）")
    b.add_argument("--u-step", type=float, default=2.0)
    b.add_argument("--min-s", type=float, default=1.0)
    b.set_defaults(func=run_build)

    ap = sub.add_parser(
        "attach-pedal-track", help="pkl にペダルの動き方（手順2 の CSV から測る）を付けて別名で保存"
    )
    ap.add_argument("--pkl", required=True, help="元の表の pkl（書き換えない）")
    ap.add_argument("--csv", required=True, help="手順2 の走行 CSV")
    ap.add_argument("--out", required=True, help="保存先 pkl（既にあれば失敗）")
    ap.set_defaults(func=run_attach_pedal_track)

    lg = sub.add_parser(
        "attach-local-gain",
        help="pkl に局所の効きの倍率の表と遅れ（手順2 の dither から測る）を付けて別名で保存",
    )
    lg.add_argument("--pkl", required=True, help="元の表の pkl（書き換えない）")
    lg.add_argument("--csv", required=True, help="手順2 の走行 CSV")
    lg.add_argument("--out", required=True, help="保存先 pkl（既にあれば失敗）")
    lg.add_argument(
        "--brake-speeds", nargs="+", type=float, default=None, metavar="KMH",
        help="ブレーキの車速の行の代表（省略: learning.ramp_dither_brake_speeds_kmh）",
    )
    lg.set_defaults(func=run_attach_local_gain)

    sc = sub.add_parser("score", help="検証ログで順モデルを採点する")
    add_train(sc)
    sc.add_argument("--map", default=None, help="表の pkl（省略: 学習ログからその場で作る）")
    sc.add_argument("--check", nargs="+", default=[str(x) for x in DEFAULT_CHECK], metavar="CSV")
    sc.add_argument("--no-sweep", action="store_true", help="(iii) の L の振りを省く")
    sc.add_argument("--sweep-only", action="store_true", help="(iii) の L の振りだけ走らせる")
    sc.set_defaults(func=run_score)

    pl = sub.add_parser("plan", help="モードの開度の列を作る")
    add_train(pl)
    pl.add_argument("--map", default=None, help="表の pkl（省略: 学習ログからその場で作る）")
    pl.add_argument(
        "--mode", nargs="+", default=None, help="モード名（省略: wltp_mode_name と 09_US06）"
    )
    pl.add_argument("--ref-csv", default=None, help="DB が読めないとき: 手順3 の CSV を渡す")
    pl.add_argument("--out-dir", default=str(RESULTS))
    pl.add_argument("--c1-log", default=str(C1_COMPARE_LOG))
    pl.add_argument(
        "--coast-band", type=float, default=DEFAULT_COAST_BAND_KMHS,
        help="惰行の帯 [km/h/s]（既定 0.25。ユーザー決定）",
    )
    pl.set_defaults(func=run_plan)

    mo = sub.add_parser("model", help="C8: 線形回帰・Ridge・ニューラルネットをオフラインで比べる")
    add_train(mo)
    mo.add_argument("--map", default=str(DEFAULT_C7_MAP), help="C7 の表の pkl（格子・設定の元）")
    mo.add_argument("--check", nargs="+", default=[str(x) for x in DEFAULT_CHECK], metavar="CSV")
    mo.add_argument(
        "--ref-check", nargs="+", default=None, metavar="CSV",
        help="参考の検証ログ（表を出すだけ。選定・学習に使わない）",
    )
    mo.add_argument("--save", action="store_true", help="選んだモデルを fwd_model_<UTC>.pkl に保存")
    mo.set_defaults(func=run_model)
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
