"""段3-3: 低開度階段からペダルゲイン k(v) と真の立ち上がりオフセット x0 を求める
（読み取り専用 CLI）。

`tests/research/pattern_loop.py` の `LowOpenStairPattern`（低開度階段）は、アクセル開度を
不感帯の直上で 1 段ずつ一定保持し、上って折り返して下りてくる往復の階段パターンである。
この走行ログ（手順2・PATTERN_DRIVE 区間）から、ペダルゲイン `k(v)` と真の立ち上がり
オフセット `x0` を求める道具がこのファイルである。車両には一切触らない。CSV と yaml を
読むだけで、既定ではファイルも書かない。

用語:
    x       … 実アクセル開度 − 不感帯 [%]（不感帯からのオフセット）
    a_eff   … 実測加速度 − 惰行加速度 [km/h/s]（＝ペダルが生んだぶんの加速度）
    モデル  … `a_eff = (k0 + k1·v)·(x − x0)`（`v` は車速 [km/h]）

なぜ `tests/research/accel_onset.py` の速度帯で切る当てはめ（`fit_band` の折れ線
`a = k·max(0, x−x0)`）ではないのか:
    低開度階段は「一定開度を保持したまま速度が変化する」往復データであり、一定保持の
    平衡状態では `a_eff` が惰行減速側に張り付く（ペダルが生む加速度と惰行減速がほぼ
    釣り合う）。accel_onset の当てはめは速度帯を固定してから x→a_eff の傾きを取るが、
    速度帯を固定すると階段データでは `a_eff` が開度にほぼ反応しなくなり、実測で
    R² が −5〜−117 になることを確認済み。つまりこのデータには「速度帯で切る」当てはめは
    構造的に効かない。そこで速度 `v` と開度オフセット `x` を同時に使う 3 パラメータ
    （k0, k1, x0）の当てはめに変える。`x0` を固定すれば式は (k0, k1) について線形になる
    ため、`x0` を一定刻みで走査し、各 `x0` について `numpy.linalg.lstsq` で (k0, k1) を
    解き、残差二乗和が最小の `x0` を選ぶ（scipy は使わない）。

なぜ上り（踏み増していく側）と下り（戻していく側）を必ず分けて当てはめるのか:
    上りと下り（応答遅れ・ヒステリシス・機構のガタ等）を混ぜて当てはめると解が退化する
    ことを実測で確認済み（混ぜると k→0・x0→−10510 という無意味な解に落ちる）。そのため
    このツールは上り・下りを常に別々に当てはめ、「全体」は参考値としてのみ出す。

CLI:
    .venv/bin/python -m tests.research.stair_gain \
        tests/research/results/drive_log_real_<日時>.csv [...複数可]
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.models.profile import FeedforwardParams, pedal_gain_at
from tests.research.accel_onset import CsvAnalysis, analyze_csv_file
from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.debug_process23 import md_table
from tests.research.drive_log import SECTION_MODE_DRIVE, SECTION_PATTERN_DRIVE
from tests.research.ff_params import research_ff_params
from tests.research.vehicle import feedforward_params

# 表3 の既定車速グリッド [km/h]
DEFAULT_SPEED_GRID: tuple[float, ...] = (8.0, 10.0, 12.0, 14.0, 16.0, 20.0, 24.0, 30.0)
# 表4 の既定速度帯境界 [km/h]（昇順）
DEFAULT_SPEED_BINS: tuple[float, ...] = (3.0, 8.0, 12.0, 16.0, 24.0, 30.0)

# 当てはめに使う最低サンプル数
MIN_FIT_SAMPLES = 10
# 当てはめに必要な x のユニーク値の最低数
MIN_UNIQUE_X = 2


# ─────────────────────────────────────────────────────────────────────
# 当てはめの中核（scipy 不使用。x0 を走査し、各 x0 で lstsq）
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class StairGainFit:
    """`a_eff = (k0 + k1·v)·(x − x0)` の当てはめ結果。"""

    k0: float  # k(v) = k0 + k1*v の切片 [(km/h/s)/%]
    k1: float  # 同 傾き [(km/h/s)/%/(km/h)]
    x0_pct: float  # 真の立ち上がりオフセット [%]
    r2: float  # 決定係数
    n: int  # 使ったサンプル数

    def gain_at(self, speed_kmh: float) -> float:
        """速度 `speed_kmh` でのペダルゲイン k(v) = k0 + k1*v。"""
        return self.k0 + self.k1 * speed_kmh


def fit_stair_gain(
    v: np.ndarray,
    x: np.ndarray,
    a: np.ndarray,
    *,
    x0_lo: float = -1.0,
    x0_hi: float = 1.0,
    x0_step: float = 0.005,
) -> StairGainFit | None:
    """`v・x・a` の 3 列（同じ長さ）から `a = (k0 + k1·v)·(x − x0)` を当てはめる。

    `x0` を `x0_lo`〜`x0_hi` を `x0_step` 刻みで走査し、各 `x0` について `(k0, k1)` を
    `numpy.linalg.lstsq` で解き、残差二乗和（SSE）が最小の `x0` を採用する（scipy 不使用）。
    NaN を含む行は先に落とす。有効サンプルが `MIN_FIT_SAMPLES` 未満、または `x` の
    ユニーク値が `MIN_UNIQUE_X` 未満なら同定できないため `None` を返す。
    """
    v = np.asarray(v, dtype=float)
    x = np.asarray(x, dtype=float)
    a = np.asarray(a, dtype=float)

    valid = ~np.isnan(v) & ~np.isnan(x) & ~np.isnan(a)
    v, x, a = v[valid], x[valid], a[valid]
    n = len(v)
    if n < MIN_FIT_SAMPLES:
        return None
    if len(np.unique(x)) < MIN_UNIQUE_X:
        return None

    mean_a = float(np.mean(a))
    denom = float(np.sum((a - mean_a) ** 2))
    if denom <= 0.0:
        return None

    best_sse: float | None = None
    best_k0 = 0.0
    best_k1 = 0.0
    best_x0 = 0.0
    for x0 in np.arange(x0_lo, x0_hi + 1e-9, x0_step):
        dx = x - x0
        design = np.column_stack([dx, v * dx])
        coef, *_ = np.linalg.lstsq(design, a, rcond=None)
        pred = design @ coef
        sse = float(np.sum((a - pred) ** 2))
        if best_sse is None or sse < best_sse:
            best_sse = sse
            best_k0 = float(coef[0])
            best_k1 = float(coef[1])
            best_x0 = float(x0)

    assert best_sse is not None
    r2 = 1.0 - best_sse / denom
    return StairGainFit(k0=best_k0, k1=best_k1, x0_pct=best_x0, r2=r2, n=n)


# ─────────────────────────────────────────────────────────────────────
# 階段サンプルの取り出し（accel_onset.analyze_csv_file を使う。accel_onset は変更しない）
# ─────────────────────────────────────────────────────────────────────


def _pool_stair_rows(
    analyses: Sequence[CsvAnalysis],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """全 CSV の階段サンプル（`stair_leg != ""`）の v・x・a_eff・レッグをプールする。

    `sample_mask`（accel_onset の定常窓・速度・ブレーキ条件）とは交わりを取らない。階段の
    一定保持は「一定保持に入った直後の過渡」と「落ち着いた後」の両方を含めて 1 レッグの
    サンプルとして扱う（NaN 行は `fit_stair_gain` 側で落とす）。
    """
    v_all, x_all, a_all, leg_all = [], [], [], []
    for an in analyses:
        mask = an.stair_leg != ""
        v_all.append(an.series.v_kmh[mask])
        x_all.append(an.x_pct[mask])
        a_all.append(an.a_eff[mask])
        leg_all.append(an.stair_leg[mask])
    if not v_all:
        return np.array([]), np.array([]), np.array([]), np.array([], dtype="<U4")
    return (
        np.concatenate(v_all), np.concatenate(x_all), np.concatenate(a_all),
        np.concatenate(leg_all),
    )


# ─────────────────────────────────────────────────────────────────────
# 表1: 階段サンプルの母数
# ─────────────────────────────────────────────────────────────────────


def build_table1(analyses: Sequence[CsvAnalysis]) -> tuple[list[str], list[list[str]]]:
    """CSV ごとの階段行数（過渡込み・生の判定）・上り／下りの内訳・車速と開度offsetの範囲。"""
    header = ["CSV", "階段行数", "上り", "下り", "車速 [km/h]", "開度offset [%]"]
    rows = []
    for an in analyses:
        mask = an.stair_leg != ""
        n = int(np.count_nonzero(mask))
        n_up = int(np.count_nonzero(mask & (an.stair_leg == "上り")))
        n_down = int(np.count_nonzero(mask & (an.stair_leg == "下り")))
        if n:
            v = an.series.v_kmh[mask]
            x = an.x_pct[mask]
            v_range = f"{float(np.min(v)):.1f}〜{float(np.max(v)):.1f}"
            x_range = f"{float(np.min(x)):.2f}〜{float(np.max(x)):.2f}"
        else:
            v_range = "—"
            x_range = "—"
        rows.append([an.series.source, str(n), str(n_up), str(n_down), v_range, x_range])
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# 表2: 当てはめ（上り／下り／全体（参考））
# ─────────────────────────────────────────────────────────────────────


def build_table2(fits: Mapping[str, StairGainFit | None]) -> tuple[list[str], list[list[str]]]:
    header = ["レッグ", "n", "k0", "k1", "x0 [%]", "R²"]
    rows = []
    for label in ("上り", "下り", "全体"):
        fit = fits.get(label)
        if fit is None:
            rows.append([label, "0", "—", "—", "—", "—"])
        else:
            rows.append([
                label, str(fit.n), f"{fit.k0:.4f}", f"{fit.k1:.4f}",
                f"{fit.x0_pct:+.3f}", f"{fit.r2:.4f}",
            ])
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# 表3: k(v) と yaml の模型ゲインの比較
# ─────────────────────────────────────────────────────────────────────


def build_table3(
    fits: Mapping[str, StairGainFit | None],
    speed_grid: Sequence[float],
    params: FeedforwardParams,
) -> tuple[list[str], list[list[str]]]:
    header = ["車速 [km/h]", "上り k(v)", "下り k(v)", "yaml 模型ゲイン", "上り/模型"]
    up_fit, down_fit = fits.get("上り"), fits.get("下り")
    rows = []
    for speed in speed_grid:
        up = up_fit.gain_at(speed) if up_fit is not None else None
        down = down_fit.gain_at(speed) if down_fit is not None else None
        model = pedal_gain_at(params, float(speed), is_accel=True)
        ratio = up / model if (up is not None and model is not None and model != 0.0) else None
        rows.append([
            f"{speed:g}",
            "—" if up is None else f"{up:.3f}",
            "—" if down is None else f"{down:.3f}",
            "—" if model is None else f"{model:.3f}",
            "—" if ratio is None else f"{ratio:.3f}",
        ])
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# 表4: 残差（上り）
# ─────────────────────────────────────────────────────────────────────


def build_table4(
    v: np.ndarray, x: np.ndarray, a: np.ndarray, fit: StairGainFit, speed_bins: Sequence[float],
) -> tuple[list[str], list[list[str]]]:
    """上りレッグの残差 `a − 当てはめ予測` を速度帯ごとに集計する。"""
    header = ["速度帯 [km/h]", "n", "残差 中央値", "残差 std"]
    gains = np.array([fit.gain_at(float(vi)) for vi in v])
    pred = gains * (x - fit.x0_pct)
    resid = a - pred
    rows = []
    for i in range(len(speed_bins) - 1):
        mask = (v >= speed_bins[i]) & (v < speed_bins[i + 1])
        n = int(np.count_nonzero(mask))
        label = f"{speed_bins[i]:g}〜{speed_bins[i + 1]:g}"
        if n == 0:
            rows.append([label, "0", "—", "—"])
            continue
        rows.append([
            label, str(n),
            f"{float(np.median(resid[mask])):+.4f}", f"{float(np.std(resid[mask])):.4f}",
        ])
    return header, rows


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


def _parse_float_list(text: str) -> tuple[float, ...]:
    return tuple(float(v) for v in text.split(","))


def run(
    csv_paths: Sequence[Path],
    config_path: Path,
    *,
    deadband_pct: float | None,
    section: str,
    speed_grid: Sequence[float],
    speed_bins: Sequence[float],
    x0_lo: float,
    x0_hi: float,
    x0_step: float,
    steady_window_s: float,
    steady_tol_pct: float,
    accel_half_window_s: float,
    min_speed_kmh: float,
    direction_lookback_s: float,
) -> int:
    cfg = load_config(config_path)
    params = feedforward_params(cfg)
    research = research_ff_params(cfg)
    db = params.accel_deadband_pct if deadband_pct is None else deadband_pct
    db_src = "CLI 指定" if deadband_pct is not None else "yaml既定"

    analyses = [
        analyze_csv_file(
            p, params, research, deadband_pct=db, steady_window_s=steady_window_s,
            steady_tol_pct=steady_tol_pct, accel_half_window_s=accel_half_window_s,
            min_speed_kmh=min_speed_kmh, direction_lookback_s=direction_lookback_s,
            section=section,
        )
        for p in csv_paths
    ]

    print(f"不感帯: {db:.2f}%（{db_src}）")
    print(f"対象 CSV: {len(analyses)} 本")

    v, x, a, leg = _pool_stair_rows(analyses)
    if v.size == 0:
        print("\n低開度階段が見つかりません（--section と CSV を確認してください）")
        return 1

    print("\n## 表1: 階段サンプルの母数\n")
    print(md_table(*build_table1(analyses)))

    fits: dict[str, StairGainFit | None] = {}
    for label in ("上り", "下り", "全体"):
        mask = np.ones(len(v), dtype=bool) if label == "全体" else (leg == label)
        fits[label] = fit_stair_gain(
            v[mask], x[mask], a[mask], x0_lo=x0_lo, x0_hi=x0_hi, x0_step=x0_step,
        )

    print("\n## 表2: 当てはめ（a_eff = (k0 + k1·v)·(x − x0)）\n")
    print("※「全体」は上り下りを混ぜると解が退化するため参考値（判定には使わない）")
    print(md_table(*build_table2(fits)))

    print("\n## 表3: k(v) と yaml の模型ゲインの比較\n")
    print(md_table(*build_table3(fits, speed_grid, params)))

    print("\n## 表4: 残差（上り）\n")
    up_fit = fits.get("上り")
    if up_fit is None:
        print("上りの当てはめが同定できないため表4 は省略します")
    else:
        up_mask = leg == "上り"
        print(md_table(*build_table4(v[up_mask], x[up_mask], a[up_mask], up_fit, speed_bins)))

    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("csv", nargs="+", type=Path, help="解析する走行ログ CSV（手順2。複数可）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument(
        "--deadband-pct", type=float, default=None,
        help="アクセル不感帯 [%%]（既定: yaml の feedforward.accel_deadband_pct）",
    )
    ap.add_argument(
        "--section", choices=(SECTION_MODE_DRIVE, SECTION_PATTERN_DRIVE),
        default=SECTION_PATTERN_DRIVE,
        help="解析する区間（既定 PATTERN_DRIVE。手順2 の低開度階段ログを見る道具のため）",
    )
    ap.add_argument(
        "--speed-grid", type=_parse_float_list, default=DEFAULT_SPEED_GRID,
        help="表3 の車速グリッド（カンマ区切り）",
    )
    ap.add_argument(
        "--speed-bins", type=_parse_float_list, default=DEFAULT_SPEED_BINS,
        help="表4 の速度帯境界（カンマ区切り、昇順）",
    )
    ap.add_argument("--x0-lo", type=float, default=-1.0, help="x0 走査の下限 [%%]")
    ap.add_argument("--x0-hi", type=float, default=1.0, help="x0 走査の上限 [%%]")
    ap.add_argument("--x0-step", type=float, default=0.005, help="x0 走査の刻み [%%]")
    ap.add_argument("--steady-window-s", type=float, default=0.45)
    ap.add_argument("--steady-tol-pct", type=float, default=0.3)
    ap.add_argument("--accel-half-window-s", type=float, default=0.30)
    ap.add_argument(
        "--min-speed-kmh", type=float, default=2.0,
        help="階段は 3.7 km/h まで下がるため accel_onset の既定 5.0 より低くする",
    )
    ap.add_argument("--direction-lookback-s", type=float, default=1.45)
    args = ap.parse_args(argv)

    return run(
        args.csv, args.config, deadband_pct=args.deadband_pct, section=args.section,
        speed_grid=args.speed_grid, speed_bins=args.speed_bins,
        x0_lo=args.x0_lo, x0_hi=args.x0_hi, x0_step=args.x0_step,
        steady_window_s=args.steady_window_s, steady_tol_pct=args.steady_tol_pct,
        accel_half_window_s=args.accel_half_window_s, min_speed_kmh=args.min_speed_kmh,
        direction_lookback_s=args.direction_lookback_s,
    )


if __name__ == "__main__":
    raise SystemExit(main())
