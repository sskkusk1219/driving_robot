"""順モデル（車速 × 効く開度 u → 加速度）の表（候補 C7）と、逆に引く部分（段2g）。

表: 車速の行（境界の列で区切る）× u の格子点（2% マスの中心。u=0 は惰行の 1 点）。
    加速度 accel[行, 格子点] [km/h/s]。u が増えたら加速度は減らない（単調）にそろえる。

作り方（build_table）:
    1. マス（車速の行 × u のマス）ごとに加速度の中央値と秒数。
    2. 秒数が min_s 未満のマスは穴。
    3. 穴埋め: 同じ行の中で両隣の実測点から直線で埋める（種類＝穴埋め）。実測の範囲の外側は
       端の値で止める（種類＝範囲外）。実測点が 2 つ未満の行は、いちばん近い行（実測点 2 つ以上）
       の値を写し、全点を範囲外とする。
    4. 単調化: 行ごとに IsotonicRegression(increasing=True)。重みは秒数（穴埋め点は小さい重み）。
       実測の範囲内だけに当て、範囲外は当てた後の端の値で止める。

`accel`・`invert`・`coast_accel` の計算は「行 × 格子点の加速度の配列」だけに依る関数
（grid_accel・grid_invert）で、配列の出どころ（表・モデル・モデル＋差の表）に依らない。
後の段の C8・C9 はこの 2 関数に自分の配列を渡して使う。
"""

from __future__ import annotations

import math
import pickle
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from sklearn.isotonic import IsotonicRegression

from tests.research.fwd_data import (
    STOP_SPEED_KMH,
    FwdRows,
    make_speed_edges,
    speed_cell_edges,
    take_rows,
    u_cell,
)

MODEL_TYPE = "forward_map_v1"
KIND_TABLE = "table"
KIND_MODEL = "model"
MODELS_DIR = Path("tests/research/results/models")

# 点の種類
POINT_MEASURED = 0  # 実測（秒数が min_s 以上のマス）
POINT_FILLED = 1  # 穴埋め（両隣の実測点から直線）
POINT_OUT = 2  # 範囲外（実測の範囲の外側。端の値で止めた点）
POINT_LABELS = {POINT_MEASURED: "実測", POINT_FILLED: "穴埋め", POINT_OUT: "範囲外"}

DEFAULT_U_STEP_PCT = 2.0  # 仮値（段2f の集計で決めた刻み）
DEFAULT_MIN_S = 1.0  # 仮値: 穴とみなす秒数（少ない行の中央値を信じない）
# 仮値: 単調化での穴埋め点の重み [秒相当]。実測点（重み＝秒数、≥ min_s）より十分小さくする
FILLED_WEIGHT_S = 0.1
# 仮値（発進側の表 attach_launch）: 1 マスの最小秒数と、発進側を作るのに要る「動いている行」の
# 合計秒数。手順2 `193155` の発進側は min＋3〜5% に約 12 秒（0.5% のマスで 1〜4 秒）。
# 段2o-4 の確認でマスごとの秒数を見て決める
LAUNCH_MIN_S = 0.5
LAUNCH_MIN_TOTAL_S = 5.0
# 仮値（戻しの表 attach_release。2026-10-14 段2o-5）:
#   RELEASE_GRID_PCT … 戻し量 [%] の格子。モードの戻し量は 0.3〜4% 超、山は min＋3〜9%
#     （段2o-4 の数え）
#   RELEASE_MIN_TOTAL_S … 戻し量 RELEASE_COUNT_MIN_PCT 以上のブレーキの行の合計秒数がこれ未満なら
#     戻しの表を作らない。`083450` の戻しの行（0.02G の下りの保持）が約 33 秒
#   RELEASE_COUNT_MIN_PCT … 「戻している」とみなす戻し量の下限（雑音ぶん）
RELEASE_GRID_PCT: tuple[float, ...] = (0.5, 1.0, 2.0, 4.0, 8.0)
RELEASE_MIN_TOTAL_S = 10.0
RELEASE_COUNT_MIN_PCT = 0.3
RELEASE_CLAMP_QUANTILE = 0.95  # release_max_pct ＝ 学習のブレーキの行の戻し量のこの分位
_EPS = 1e-12
_CHUNK = 20000


@dataclass
class ForwardTable:
    """順モデルの表。配列の行は車速の行、列は u の格子点。"""

    speed_edges: np.ndarray  # 車速の行の境界（行数 + 1）
    speed_rep: np.ndarray  # 行ごとの代表車速（マスの中心）
    u_grid: np.ndarray  # u の格子点 [%]（昇順。0 を含む）
    accel_grid: np.ndarray  # (行, 格子点) 加速度 [km/h/s]（単調化後）。メソッド accel と区別
    seconds: np.ndarray  # (行, 格子点) マスの秒数
    point_kind: np.ndarray  # (行, 格子点) POINT_*
    meas_u_min: np.ndarray  # 行ごとの実測の u の最小の点
    meas_u_max: np.ndarray  # 行ごとの実測の u の最大の点
    accel_before_mono: np.ndarray | None = None  # 単調化の前（穴埋め後）の加速度
    meta: dict[str, Any] = field(default_factory=dict)
    kind: str = KIND_TABLE
    # 発進側（基準車速が上がっているときに引く）の加速度。accel_grid と同じ形で、発進側の実測がある
    # 車速の行だけ置き換えた後の配列。None ＝ 発進側なし（2026-10-14 段2o-4。attach_launch）
    launch_accel: np.ndarray | None = None
    # 戻しの表（2026-10-14 段2o-5。attach_release）。release_grid ＝ 戻し量 [%] の格子（昇順・正）、
    # release_accel ＝ (戻し量の数, 行, 格子点) 加速度。戻し量 0 は本体 accel_grid。
    # None ＝ 戻しの表なし（今までと完全に同じ）。戻し量は meta["release_max_pct"] で頭打ちにする
    release_grid: np.ndarray | None = None
    release_accel: np.ndarray | None = None

    def _release_nodes(
        self, release: Any, launch: bool, shape: tuple[int, ...]
    ) -> tuple[list[np.ndarray], np.ndarray, np.ndarray] | None:
        """戻し量を使うときの (節の配列のリスト, 節の戻し量, 頭打ちした戻し量)。使わないなら None。

        使わない: release が None・戻しの表なし・発進側の表を引くとき（launch=True で
        launch_accel があるとき。発進は戻しではないので戻し量は無視する）。
        節は 0（本体）と release_grid。
        """
        if release is None or self.release_grid is None or self.release_accel is None:
            return None
        if launch and self.launch_accel is not None:
            return None
        vals = np.concatenate([[0.0], np.asarray(self.release_grid, dtype=float)])
        top = float(vals[-1])
        cap = self.meta.get("release_max_pct")
        if cap is not None and np.isfinite(cap):
            top = min(top, float(cap))
        r = np.clip(np.broadcast_to(np.asarray(release, dtype=float), shape), 0.0, top)
        return [self.accel_grid, *self.release_accel], vals, r

    def _grid(self, launch: bool) -> np.ndarray:
        return self.launch_accel if (launch and self.launch_accel is not None) else self.accel_grid

    def accel(
        self,
        v_kmh: np.ndarray | float,
        u_pct: np.ndarray | float,
        *,
        launch: bool = False,
        release: np.ndarray | float | None = None,
    ) -> Any:
        """車速 v・効く開度 u での加速度 [km/h/s]（ベクトル入力可）。

        launch=True で発進側の表があればそちらを引く（無ければ本体）。
        release（戻し量 [%]。スカラーか配列）を渡すと、戻しの表があるとき戻し量の節の間
        （0＝本体、release_grid）を直線補間して引く（release_max_pct で頭打ち）。release が
        None・0、戻しの表なし、発進側の表を引くときは今までと同じ。
        """
        v, u = np.broadcast_arrays(np.asarray(v_kmh, dtype=float), np.asarray(u_pct, dtype=float))
        nodes = self._release_nodes(release, launch, v.shape)
        if nodes is None:
            return grid_accel(self.speed_rep, self.u_grid, self._grid(launch), v_kmh, u_pct)
        arrs, vals, r = nodes
        k, w = _release_weights(vals, r)
        out = np.zeros(v.shape)
        for j, arr in enumerate(arrs):
            h = np.where(k == j, 1.0 - w, 0.0) + np.where(k + 1 == j, w, 0.0)
            if h.any():
                out = out + h * np.asarray(grid_accel(self.speed_rep, self.u_grid, arr, v, u))
        return _scalar_or_array(out)

    def invert(
        self,
        v_kmh: np.ndarray | float,
        a_kmhs: np.ndarray | float,
        *,
        launch: bool = False,
        release: np.ndarray | float | None = None,
    ) -> tuple[Any, Any]:
        """車速 v で加速度 a になる u と点の種類 POINT_*（ベクトル入力可）。launch・release は
        accel と同じ（release は 1 サンプルごとに違う値を渡せる。サンプルごとに補間した列で解く）。
        """
        v, a = np.broadcast_arrays(np.asarray(v_kmh, dtype=float), np.asarray(a_kmhs, dtype=float))
        nodes = self._release_nodes(release, launch, v.shape)
        if nodes is None:
            return grid_invert(
                self.speed_rep, self.u_grid, self._grid(launch), self.point_kind, v_kmh, a_kmhs
            )
        arrs, vals, r = nodes
        return grid_invert(
            self.speed_rep, self.u_grid, self.accel_grid, self.point_kind, v, a,
            release_nodes=arrs, release_node_pct=vals, release=r,
        )

    def coast_accel(self, v_kmh: np.ndarray | float, *, launch: bool = False) -> Any:
        """u=0（惰行）の加速度 [km/h/s]。launch は accel と同じ。"""
        return grid_accel(self.speed_rep, self.u_grid, self._grid(launch), v_kmh, 0.0)

    def save(self, path: Path | None = None) -> Path:
        return save_table(self, path)


def _row_weights(speed_rep: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """車速 v を挟む 2 行の番号と、上の行の重み（端は止める）。"""
    r = len(speed_rep)
    if r == 1:
        z = np.zeros(v.shape, dtype=int)
        return z, z, np.zeros(v.shape)
    i0 = np.clip(np.searchsorted(speed_rep, v, side="right") - 1, 0, r - 2)
    span = speed_rep[i0 + 1] - speed_rep[i0]
    w = np.clip((v - speed_rep[i0]) / span, 0.0, 1.0)
    return i0, i0 + 1, w


def _release_weights(vals: np.ndarray, r: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """戻し量 r を挟む 2 つの節の番号 k（と k+1）と、上の節の重み w（節の戻し量 vals は昇順）。"""
    n = len(vals)
    if n == 1:
        return np.zeros(r.shape, dtype=int), np.zeros(r.shape)
    k = np.clip(np.searchsorted(vals, r, side="right") - 1, 0, n - 2)
    w = np.clip((r - vals[k]) / (vals[k + 1] - vals[k]), 0.0, 1.0)
    return k, w


def _scalar_or_array(x: np.ndarray) -> Any:
    return float(x) if x.ndim == 0 else x


def grid_accel(
    speed_rep: np.ndarray,
    u_grid: np.ndarray,
    accel: np.ndarray,
    v_kmh: np.ndarray | float,
    u_pct: np.ndarray | float,
) -> Any:
    """加速度 [km/h/s]。車速方向は行の代表車速の間を、u 方向は格子点の間を直線補間（端は止める）。
    """
    v, u = np.broadcast_arrays(
        np.asarray(v_kmh, dtype=float), np.asarray(u_pct, dtype=float)
    )
    i0, i1, w = _row_weights(speed_rep, v)
    k = len(u_grid)
    if k == 1:
        j0 = np.zeros(u.shape, dtype=int)
        j1, wu = j0, np.zeros(u.shape)
    else:
        j0 = np.clip(np.searchsorted(u_grid, u, side="right") - 1, 0, k - 2)
        j1 = j0 + 1
        wu = np.clip((u - u_grid[j0]) / (u_grid[j1] - u_grid[j0]), 0.0, 1.0)
    a_lo = accel[i0, j0] * (1.0 - wu) + accel[i0, j1] * wu
    a_hi = accel[i1, j0] * (1.0 - wu) + accel[i1, j1] * wu
    return _scalar_or_array(a_lo * (1.0 - w) + a_hi * w)


def grid_invert(
    speed_rep: np.ndarray,
    u_grid: np.ndarray,
    accel: np.ndarray,
    point_kind: np.ndarray,
    v_kmh: np.ndarray | float,
    a_kmhs: np.ndarray | float,
    *,
    release_nodes: Sequence[np.ndarray] | None = None,
    release_node_pct: np.ndarray | None = None,
    release: np.ndarray | None = None,
) -> tuple[Any, Any]:
    """車速 v で加速度 a になる u と点の種類を返す（ベクトル入力可）。

    release_nodes（戻し量の節ごとの (行, 格子点) 配列。先頭 ＝ 戻し量 0）・release_node_pct（節の
    戻し量。昇順）・release（サンプルごとの戻し量。v と同じ形）を渡すと、accel の代わりに
    サンプルごとに節の間を直線補間した列を使う（2026-10-14 段2o-5）。渡さなければ今までと同じ。

    車速 v の列は、上下 2 行を車速で直線補間した加速度の列（単調）。
    - 平らな区間（同じ加速度が続く）では |u| が小さい側を返す。
    - a が列の最大より大きい／最小より小さいときは、端の u（最大に最初に届く点／最小の最後の点）
      を返し、種類は範囲外。
    - 種類 = 挟んだ格子点（と上下の行）の種類のうち最悪のもの（範囲外 > 穴埋め > 実測）。
    """
    v, a = np.broadcast_arrays(np.asarray(v_kmh, dtype=float), np.asarray(a_kmhs, dtype=float))
    shape = v.shape
    vf, af = v.reshape(-1), a.reshape(-1)
    rf = None
    if release_nodes is not None:
        assert release_node_pct is not None and release is not None  # noqa: S101
        rf = np.broadcast_to(np.asarray(release, dtype=float), shape).reshape(-1)
    u_out = np.empty(len(vf))
    kind_out = np.empty(len(vf), dtype=int)
    kg = len(u_grid)
    zero_idx = int(np.argmin(np.abs(u_grid)))
    for s in range(0, len(vf), _CHUNK):
        sl = slice(s, s + _CHUNK)
        vv, aa = vf[sl], af[sl]
        i0, i1, w = _row_weights(speed_rep, vv)
        if rf is None:
            c = (1.0 - w)[:, None] * accel[i0] + w[:, None] * accel[i1]
        else:
            kr, wr = _release_weights(release_node_pct, rf[sl])
            c = np.zeros((len(vv), kg))
            for j, arr in enumerate(release_nodes):
                h = np.where(kr == j, 1.0 - wr, 0.0) + np.where(kr + 1 == j, wr, 0.0)
                if h.any():
                    cj = (1.0 - w)[:, None] * arr[i0] + w[:, None] * arr[i1]
                    c = c + h[:, None] * cj
        c = np.maximum.accumulate(c, axis=1)
        kd = np.maximum(
            np.where(w[:, None] < 1.0, point_kind[i0], 0),
            np.where(w[:, None] > 0.0, point_kind[i1], 0),
        )
        ac = aa[:, None]
        khi = (c < ac - _EPS).sum(axis=1)  # a 以上になる最初の点
        klo = (c <= ac + _EPS).sum(axis=1) - 1  # a 以下の最後の点
        n = len(vv)
        rng = np.arange(n)
        u = np.empty(n)
        kind = np.empty(n, dtype=int)

        hi_out = khi >= kg
        lo_out = klo < 0
        flat = (klo >= khi) & ~hi_out & ~lo_out
        cross = ~(hi_out | lo_out | flat)

        if hi_out.any():
            top = c[:, -1:]
            kk = (c < top - _EPS).sum(axis=1)
            u[hi_out] = u_grid[kk[hi_out]]
            kind[hi_out] = POINT_OUT
        if lo_out.any():
            bot = c[:, :1]
            kk = (c <= bot + _EPS).sum(axis=1) - 1
            u[lo_out] = u_grid[kk[lo_out]]
            kind[lo_out] = POINT_OUT
        if flat.any():
            h = np.clip(khi, 0, kg - 1)
            lo = np.clip(klo, 0, kg - 1)
            kc = np.where(u_grid[h] > 0.0, h, np.where(u_grid[lo] < 0.0, lo, zero_idx))
            u[flat] = u_grid[kc[flat]]
            kind[flat] = kd[rng, kc][flat]
        if cross.any():
            lo = np.clip(klo, 0, kg - 2)
            hi = lo + 1
            c0, c1 = c[rng, lo], c[rng, hi]
            frac = (aa - c0) / np.where(c1 - c0 == 0.0, 1.0, c1 - c0)
            uu = u_grid[lo] + np.clip(frac, 0.0, 1.0) * (u_grid[hi] - u_grid[lo])
            u[cross] = uu[cross]
            kind[cross] = np.maximum(kd[rng, lo], kd[rng, hi])[cross]
        u_out[sl], kind_out[sl] = u, kind
    return _scalar_or_array(u_out.reshape(shape)), _scalar_or_array(kind_out.reshape(shape))


def u_grid_points(k_min: int, k_max: int, u_step_pct: float) -> np.ndarray:
    """u のマス番号 k_min..k_max の中心。k>0: (k−0.5)*step、k<0: (k+0.5)*step、0: 0（惰行の点）。"""
    ks = np.arange(k_min, k_max + 1)
    return np.where(ks > 0, (ks - 0.5) * u_step_pct, np.where(ks < 0, (ks + 0.5) * u_step_pct, 0.0))


def refine_u_grid(table: ForwardTable, u_step_pct: float) -> ForwardTable:
    """表の u の格子を細かい刻みに作り直す（2026-10-14 段2o-4）。

    範囲は元と同じ（元のマス k_min..k_max の外側の端まで）。0（惰行）は 1 点のまま。
    加速度は元の表を u 方向に直線補間、秒数・点の種類は新しい点が入る元のマスの値を引き継ぐ。
    元の刻みは新しい刻みの整数倍であること（そうでなければ ValueError）。
    """
    old = float(table.meta.get("u_step_pct", DEFAULT_U_STEP_PCT))
    if u_step_pct >= old - 1e-9:
        return table
    ratio = old / u_step_pct
    n = int(round(ratio))
    if abs(ratio - n) > 1e-6:
        raise ValueError(f"元の刻み {old} が新しい刻み {u_step_pct} の整数倍ではありません")
    ks_old = u_cell(table.u_grid, old)
    k_min, k_max = int(ks_old.min()), int(ks_old.max())
    grid = u_grid_points(k_min * n, k_max * n, u_step_pct)
    cell = u_cell(grid, old)
    col = (cell - k_min).astype(int)  # 元の表の列（元の格子は k_min..k_max の連続）
    accel = np.array([np.interp(grid, table.u_grid, r) for r in table.accel_grid])
    before = (
        None
        if table.accel_before_mono is None
        else np.array([np.interp(grid, table.u_grid, r) for r in table.accel_before_mono])
    )
    meta = dict(table.meta)
    meta["like_u_step_pct"] = old
    meta["u_step_pct"] = u_step_pct
    return ForwardTable(
        speed_edges=table.speed_edges,
        speed_rep=table.speed_rep,
        u_grid=grid,
        accel_grid=accel,
        seconds=table.seconds[:, col],
        point_kind=table.point_kind[:, col],
        meas_u_min=table.meas_u_min,  # u の値（格子の番号ではない）
        meas_u_max=table.meas_u_max,
        accel_before_mono=before,
        meta=meta,
        kind=table.kind,
        launch_accel=None,  # 発進側は refine の後に attach_launch で付ける
    )


def attach_launch(
    table: ForwardTable,
    launch_rows: FwdRows,
    *,
    min_s: float = LAUNCH_MIN_S,
    min_total_s: float = LAUNCH_MIN_TOTAL_S,
    accel_window_s: float = math.nan,
) -> ForwardTable:
    """発進側の加速度（launch_accel）を付けた新しい表を返す（2026-10-14 段2o-4）。

    launch_rows: 発進の区間の行（`fwd_data.launch_mask` で絞ったもの。停車の行を含んでよい）。
    動いている行（実車速 >= STOP_SPEED_KMH）の合計秒数が min_total_s 未満なら launch_accel は None
    （meta に launch_rows_s だけ残す）。足りれば発進側の行だけで表を作り、実測のマスが 2 つ以上ある
    車速の行だけ次のように置き換える（それ以外の行は本体のまま）:
        u <= 0 の列 … 発進側の表をその u で引いた値（範囲外は端の値）
        u >  0 の列 … max(本体の値, 発進側の u = 0 の値)
    置き換えた行は単調化（IsotonicRegression）し直す。元の表は書き換えない。
    accel_window_s は発進側の行を作ったときの加速度の窓で、meta に残すだけ。
    """
    moving = launch_rows.speed_kmh >= STOP_SPEED_KMH
    moving_s = float(np.sum(launch_rows.dt_row_s[moving])) if len(launch_rows) else 0.0
    meta = dict(table.meta)
    meta.update(
        launch_rows_s=moving_s,
        launch_speed_rows=[],
        launch_min_s=min_s,
        launch_accel_window_s=accel_window_s,
    )
    if len(launch_rows) == 0 or moving_s < min_total_s:
        return replace(table, launch_accel=None, meta=meta)
    try:
        lt = build_table(
            take_rows(launch_rows, moving),
            speed_edges=table.speed_edges,
            u_step_pct=float(table.meta.get("u_step_pct", DEFAULT_U_STEP_PCT)),
            min_s=min_s,
        )
    except ValueError:  # 実測のマスが 2 つ以上ある行が 1 つも無い
        return replace(table, launch_accel=None, meta=meta)
    used = np.flatnonzero((lt.point_kind != POINT_OUT).any(axis=1))
    if len(used) == 0:
        return replace(table, launch_accel=None, meta=meta)
    grid = table.u_grid
    neg = grid <= 0.0
    out = table.accel_grid.copy()
    for r in used:
        lrow = lt.accel_grid[r]
        row = out[r].copy()
        row[neg] = np.interp(grid[neg], lt.u_grid, lrow)
        a0 = float(np.interp(0.0, lt.u_grid, lrow))
        row[~neg] = np.maximum(row[~neg], a0)
        out[r] = IsotonicRegression(increasing=True).fit_transform(grid, row)
    meta["launch_speed_rows"] = [float(table.speed_rep[r]) for r in used]
    return replace(table, launch_accel=out, meta=meta)


def release_stats(
    release_pct: np.ndarray,
    dt_row_s: np.ndarray,
    *,
    count_min_pct: float = RELEASE_COUNT_MIN_PCT,
    quantile: float = RELEASE_CLAMP_QUANTILE,
) -> tuple[float, float]:
    """学習のブレーキの行（u < 0 の行。呼ぶ側で絞って渡す）の戻し量から
    (戻し量 count_min_pct 以上の行の合計秒数, 戻し量の quantile 分位) を返す。
    行が無ければ (0, 0)。"""
    rel = np.asarray(release_pct, dtype=float)
    if len(rel) == 0:
        return 0.0, 0.0
    total = float(np.sum(np.asarray(dt_row_s, dtype=float)[rel >= count_min_pct - 1e-12]))
    return total, float(np.quantile(rel, quantile))


def attach_release(
    table: ForwardTable,
    release_accel: np.ndarray,
    *,
    release_pct: np.ndarray,
    dt_row_s: np.ndarray,
    release_grid: Sequence[float] = RELEASE_GRID_PCT,
    min_total_s: float = RELEASE_MIN_TOTAL_S,
) -> ForwardTable:
    """戻しの表（release_grid・release_accel）を付けた新しい表を返す（2026-10-14 段2o-5）。

    release_accel: (戻し量の数, 行, 格子点)。単調化済み（fwd_model.monotone_rows）で、
    table と同じ格子。
    release_pct・dt_row_s: 学習に使ったブレーキの行（u < 0）の戻し量と行ごとの秒数。
    戻し量 RELEASE_COUNT_MIN_PCT 以上の行の合計秒数が min_total_s 未満なら付けない
    （release_grid は None ＝ 今までと完全に同じ。meta に秒数だけ残す）。付けるときは meta の
    release_max_pct ＝ 戻し量の p95 を残す（accel・invert はこれで頭打ちにして外挿しない）。
    """
    total_s, p95 = release_stats(release_pct, dt_row_s)
    meta = dict(table.meta)
    meta.update(
        release_rows_s=total_s,
        release_max_pct=p95,
        release_min_total_s=min_total_s,
        release_count_min_pct=RELEASE_COUNT_MIN_PCT,
    )
    grid = np.asarray(release_grid, dtype=float)
    acc = np.asarray(release_accel, dtype=float)
    if acc.shape != (len(grid), *table.accel_grid.shape):
        raise ValueError(
            f"release_accel の形 {acc.shape} が"
            f" (戻し量 {len(grid)}, {table.accel_grid.shape}) と違う"
        )
    if total_s < min_total_s:
        return replace(table, release_grid=None, release_accel=None, meta=meta)
    meta["release_grid_pct"] = [float(x) for x in grid]
    return replace(table, release_grid=grid, release_accel=acc, meta=meta)


def build_table(
    rows: FwdRows,
    *,
    speed_edges: np.ndarray | None = None,
    u_step_pct: float = DEFAULT_U_STEP_PCT,
    min_s: float = DEFAULT_MIN_S,
    accel_deadband_pct: float = math.nan,
    brake_deadband_pct: float = math.nan,
    accel_lag_s: float = math.nan,
    brake_lag_s: float = math.nan,
    accel_window_s: float = math.nan,
    extra_meta: dict[str, Any] | None = None,
) -> ForwardTable:
    """学習用の行から表を作る（手順はモジュールの docstring）。

    不感帯・遅れ・窓は meta に書くだけ（計算には使わない）。
    """
    if len(rows) == 0:
        raise ValueError("学習用の行が空です")
    edges = (
        make_speed_edges(float(rows.speed_kmh.max()))
        if speed_edges is None
        else np.asarray(speed_edges, dtype=float)
    )
    n_rows = len(edges) - 1
    rep = (edges[:-1] + edges[1:]) / 2.0
    sc = speed_cell_edges(rows.speed_kmh, edges)
    uc = u_cell(rows.u_pct, u_step_pct)
    k_min, k_max = min(int(uc.min()), 0), max(int(uc.max()), 0)
    grid = u_grid_points(k_min, k_max, u_step_pct)
    n_k = len(grid)

    # 1. マスごとの中央値と秒数
    key = sc * n_k + (uc - k_min)
    order = np.argsort(key, kind="stable")
    key_s = key[order]
    acc_s = rows.accel_kmhs[order]
    dt_s = rows.dt_row_s[order]
    uniq, start = np.unique(key_s, return_index=True)
    med = np.full((n_rows, n_k), np.nan)
    sec = np.zeros((n_rows, n_k))
    for q, (kq, s0) in enumerate(zip(uniq.tolist(), start.tolist(), strict=True)):
        s1 = int(start[q + 1]) if q + 1 < len(start) else len(key_s)
        r, j = divmod(int(kq), n_k)
        med[r, j] = float(np.median(acc_s[s0:s1]))
        sec[r, j] = float(dt_s[s0:s1].sum())

    accel = np.zeros((n_rows, n_k))
    before = np.zeros((n_rows, n_k))
    kind = np.full((n_rows, n_k), POINT_OUT, dtype=int)
    lo_idx = np.zeros(n_rows, dtype=int)
    hi_idx = np.zeros(n_rows, dtype=int)
    good = np.zeros(n_rows, dtype=bool)
    for r in range(n_rows):
        meas = np.flatnonzero((sec[r] >= min_s) & np.isfinite(med[r]))
        if len(meas) < 2:
            continue
        good[r] = True
        lo, hi = int(meas[0]), int(meas[-1])
        lo_idx[r], hi_idx[r] = lo, hi
        vals = med[r, meas]
        row = np.empty(n_k)
        row[lo : hi + 1] = np.interp(grid[lo : hi + 1], grid[meas], vals)  # 3. 穴は直線で埋める
        row[:lo] = vals[0]  # 範囲外は端の値で止める
        row[hi + 1 :] = vals[-1]
        before[r] = row
        kind[r, lo : hi + 1] = POINT_FILLED
        kind[r, meas] = POINT_MEASURED
        # 4. 単調化（実測の範囲内だけ。重み = 秒数、穴埋め点は FILLED_WEIGHT_S）
        w = np.where(kind[r, lo : hi + 1] == POINT_MEASURED, sec[r, lo : hi + 1], FILLED_WEIGHT_S)
        mono = IsotonicRegression(increasing=True).fit_transform(
            grid[lo : hi + 1], row[lo : hi + 1], sample_weight=w
        )
        row[lo : hi + 1] = mono
        row[:lo] = mono[0]
        row[hi + 1 :] = mono[-1]
        accel[r] = row
    if not good.any():
        raise ValueError(f"実測点が 2 つ以上ある行がありません（min_s={min_s}）")
    # 実測点が 2 つ未満の行はいちばん近い行を写す（全点を範囲外）
    good_rows = np.flatnonzero(good)
    for r in np.flatnonzero(~good):
        src = int(good_rows[np.argmin(np.abs(rep[good_rows] - rep[r]))])
        accel[r] = accel[src]
        before[r] = before[src]
        kind[r] = POINT_OUT
        lo_idx[r], hi_idx[r] = lo_idx[src], hi_idx[src]

    meta: dict[str, Any] = {
        "accel_deadband_pct": accel_deadband_pct,
        "brake_deadband_pct": brake_deadband_pct,
        "accel_lag_s": accel_lag_s,
        "brake_lag_s": brake_lag_s,
        "accel_window_s": accel_window_s,
        "u_step_pct": u_step_pct,
        "min_s": min_s,
        "filled_weight_s": FILLED_WEIGHT_S,
        "sources": rows.source,
        "copied_rows": [int(r) for r in np.flatnonzero(~good)],
        "created_at": datetime.now().isoformat(timespec="seconds"),
    }
    meta.update(extra_meta or {})
    return ForwardTable(
        speed_edges=edges,
        speed_rep=rep,
        u_grid=grid,
        accel_grid=accel,
        seconds=sec,
        point_kind=kind,
        meas_u_min=grid[lo_idx],
        meas_u_max=grid[hi_idx],
        accel_before_mono=before,
        meta=meta,
    )


def default_table_path(now: datetime | None = None) -> Path:
    stamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
    return MODELS_DIR / f"fwd_table_{stamp}.pkl"


def save_table(table: ForwardTable, path: Path | None = None) -> Path:
    """pkl に保存する（dict。model_type は forward_map_v1）。"""
    out = Path(path) if path is not None else default_table_path()
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "model_type": MODEL_TYPE,
        "kind": table.kind,
        "speed_edges": table.speed_edges,
        "speed_rep": table.speed_rep,
        "u_grid": table.u_grid,
        "accel": table.accel_grid,
        "seconds": table.seconds,
        "point_kind": table.point_kind,
        "meas_u_min": table.meas_u_min,
        "meas_u_max": table.meas_u_max,
        "accel_before_mono": table.accel_before_mono,
        "launch_accel": table.launch_accel,
        "release_grid": table.release_grid,
        "release_accel": table.release_accel,
        "meta": table.meta,
    }
    with out.open("wb") as f:
        pickle.dump(payload, f)
    return out


def load_forward_map(path: Path) -> ForwardTable:
    """pkl を読む。model_type が forward_map_v1 でなければ ValueError。"""
    with Path(path).open("rb") as f:
        d = pickle.load(f)  # noqa: S301 - 自分で作った pkl だけを読む
    if not isinstance(d, dict) or d.get("model_type") != MODEL_TYPE:
        got = d.get("model_type") if isinstance(d, dict) else type(d).__name__
        raise ValueError(f"{path} は順モデルの pkl ではありません（model_type={got!r}）")
    return ForwardTable(
        speed_edges=d["speed_edges"],
        speed_rep=d["speed_rep"],
        u_grid=d["u_grid"],
        accel_grid=d["accel"],
        seconds=d["seconds"],
        point_kind=d["point_kind"],
        meas_u_min=d["meas_u_min"],
        meas_u_max=d["meas_u_max"],
        accel_before_mono=d.get("accel_before_mono"),
        meta=d.get("meta", {}),
        kind=d.get("kind", KIND_TABLE),
        launch_accel=d.get("launch_accel"),  # 古い pkl（キー無し）は None
        release_grid=d.get("release_grid"),  # 2026-10-14 段2o-5: 古い pkl（キー無し）は None
        release_accel=d.get("release_accel"),
    )
