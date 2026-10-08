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
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from sklearn.isotonic import IsotonicRegression

from tests.research.fwd_data import FwdRows, make_speed_edges, speed_cell_edges, u_cell

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

    def accel(self, v_kmh: np.ndarray | float, u_pct: np.ndarray | float) -> Any:
        """車速 v・効く開度 u での加速度 [km/h/s]（ベクトル入力可）。"""
        return grid_accel(self.speed_rep, self.u_grid, self.accel_grid, v_kmh, u_pct)

    def invert(self, v_kmh: np.ndarray | float, a_kmhs: np.ndarray | float) -> tuple[Any, Any]:
        """車速 v で加速度 a になる u と点の種類 POINT_*（ベクトル入力可）。"""
        return grid_invert(
            self.speed_rep, self.u_grid, self.accel_grid, self.point_kind, v_kmh, a_kmhs
        )

    def coast_accel(self, v_kmh: np.ndarray | float) -> Any:
        """u=0（惰行）の加速度 [km/h/s]。"""
        return grid_accel(self.speed_rep, self.u_grid, self.accel_grid, v_kmh, 0.0)

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
) -> tuple[Any, Any]:
    """車速 v で加速度 a になる u と点の種類を返す（ベクトル入力可）。

    車速 v の列は、上下 2 行を車速で直線補間した加速度の列（単調）。
    - 平らな区間（同じ加速度が続く）では |u| が小さい側を返す。
    - a が列の最大より大きい／最小より小さいときは、端の u（最大に最初に届く点／最小の最後の点）
      を返し、種類は範囲外。
    - 種類 = 挟んだ格子点（と上下の行）の種類のうち最悪のもの（範囲外 > 穴埋め > 実測）。
    """
    v, a = np.broadcast_arrays(np.asarray(v_kmh, dtype=float), np.asarray(a_kmhs, dtype=float))
    shape = v.shape
    vf, af = v.reshape(-1), a.reshape(-1)
    u_out = np.empty(len(vf))
    kind_out = np.empty(len(vf), dtype=int)
    kg = len(u_grid)
    zero_idx = int(np.argmin(np.abs(u_grid)))
    for s in range(0, len(vf), _CHUNK):
        sl = slice(s, s + _CHUNK)
        vv, aa = vf[sl], af[sl]
        i0, i1, w = _row_weights(speed_rep, vv)
        c = (1.0 - w)[:, None] * accel[i0] + w[:, None] * accel[i1]
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
    )
