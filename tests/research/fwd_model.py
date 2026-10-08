"""順モデルのモデル版（候補 C8）: 線形回帰・Ridge・ニューラルネットを学習し、C7 の表と同じ格子で
評価して ForwardTable(kind="model") にする（段2i）。

形: アクセル側（u >= 0 の行）とブレーキ側（u <= 0 の行）で別の式。u = 0（惰行）の行は両方に入れる。
    線形回帰・Ridge: 入力は 車速・u・車速²・u²・車速×u（標準化）。2 次の曲面。
    ニューラルネット: Apollo の形。入力は 車速・u（標準化）、隠れ層 1 つ・シグモイド・10 個。
格子化（model_to_grid）: 格子は C7 の表（like）の speed_rep・u_grid をそのまま使い、u > 0 の点は
    アクセルの式、u < 0 はブレーキの式、u = 0 は 2 式の平均。point_kind・seconds・meas_u_min/max は
    like から写す（範囲外・穴埋めの秒数を C7 と同じ物差しで数えるため）。
    値自体はモデルが全点を埋める。
    単調化は行ごとの IsotonicRegression（重みは一様）。かける前の配列を accel_before_mono に残す。
    単調化で動いた量は meta に残すが、モデルを外す理由にはしない（C7 の表と同じく、単調化した後の
    配列を使う）。
ニューラルネットの目的変数（加速度）は標準化して学習し、予測で元に戻す（収束を安定させるため。
    計画書にない追加。meta の target_standardized）。
"""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np
from sklearn.isotonic import IsotonicRegression
from sklearn.linear_model import LinearRegression, Ridge
from sklearn.model_selection import GroupKFold
from sklearn.neural_network import MLPRegressor
from sklearn.pipeline import Pipeline, make_pipeline
from sklearn.preprocessing import FunctionTransformer, StandardScaler

from tests.research.fwd_data import FwdRows
from tests.research.fwd_map import (
    KIND_MODEL,
    POINT_FILLED,
    POINT_MEASURED,
    ForwardTable,
)

NAME_LINEAR = "線形回帰"
NAME_RIDGE = "Ridge"
NAME_MLP = "ニューラルネット"
MODEL_NAMES = (NAME_LINEAR, NAME_RIDGE, NAME_MLP)

RIDGE_ALPHAS = (0.01, 0.1, 1.0, 10.0, 100.0)  # 仮値: ログ単位の交差検証で選ぶ
MLP_HIDDEN = (10,)  # 仮値（Apollo の形。個数は論文に記載なし）
MLP_MAX_ITER = 500
RANDOM_STATE = 0  # 固定（再現のため）


def poly_features(x: np.ndarray) -> np.ndarray:
    """(車速, u) → 車速・u・車速²・u²・車速×u。"""
    v, u = x[:, 0], x[:, 1]
    return np.column_stack([v, u, v * v, u * u, v * u])


@dataclass
class FittedModel:
    """学習済みの 1 種類。accel_side・brake_side は (車速, u) の (N, 2) 配列 → 加速度。"""

    name: str
    accel_side: Any
    brake_side: Any
    alpha: float | None = None
    hidden: tuple[int, ...] | None = None
    train_time_s: float = 0.0
    sources: list[str] = field(default_factory=list)


def _xy(
    rows_list: Sequence[FwdRows], side: str, *, coast_in_brake: bool = True
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """片側の行（accel: u >= 0、brake: u <= 0）の入力 (N, 2)・加速度・ログ番号（グループ）。
    coast_in_brake=False のときブレーキ側は u < 0 の行だけ（惰行の行を入れない）。"""
    xs, ys, gs = [], [], []
    for g, r in enumerate(rows_list):
        if side == "accel":
            m = r.u_pct >= 0
        else:
            m = r.u_pct <= 0 if coast_in_brake else r.u_pct < 0
        xs.append(np.column_stack([r.speed_kmh[m], r.u_pct[m]]))
        ys.append(r.accel_kmhs[m])
        gs.append(np.full(int(m.sum()), g))
    return np.concatenate(xs), np.concatenate(ys), np.concatenate(gs)


def _poly_pipe(est: Any) -> Pipeline:
    return make_pipeline(FunctionTransformer(poly_features), StandardScaler(), est)


def _mlp_pipe(hidden: tuple[int, ...], max_iter: int) -> Pipeline:
    return make_pipeline(
        StandardScaler(),
        MLPRegressor(
            hidden_layer_sizes=hidden,
            activation="logistic",
            random_state=RANDOM_STATE,
            early_stopping=True,
            max_iter=max_iter,
        ),
    )


def _fit_scaled_target(make: Callable[[], Pipeline], x: np.ndarray, y: np.ndarray) -> Any:
    """加速度を標準化して学習し、予測で戻す（MLP の収束を安定させる）。"""
    mu, sd = float(y.mean()), float(y.std()) or 1.0
    pipe = make().fit(x, (y - mu) / sd)

    class _Scaled:
        def predict(self, xx: np.ndarray) -> np.ndarray:
            return pipe.predict(xx) * sd + mu

    return _Scaled()


def select_ridge_alpha(
    rows_list: Sequence[FwdRows], alphas: Sequence[float] = RIDGE_ALPHAS
) -> tuple[float, dict[float, float]]:
    """ログ単位の交差検証（GroupKFold。1 本を除いて学習・その 1 本で採点）で alpha を選ぶ。
    採点はアクセル側・ブレーキ側の二乗誤差の合計。返り値: 選んだ alpha、alpha ごとの RMSE。"""
    n_logs = len(rows_list)
    sse = dict.fromkeys(alphas, 0.0)
    cnt = 0
    for side in ("accel", "brake"):
        x, y, g = _xy(rows_list, side)
        cnt += len(y)
        for tr, te in GroupKFold(n_splits=n_logs).split(x, y, g):
            for a in alphas:
                est = _poly_pipe(Ridge(alpha=a)).fit(x[tr], y[tr])
                sse[a] += float(np.sum((est.predict(x[te]) - y[te]) ** 2))
    rmse = {a: float(np.sqrt(s / cnt)) for a, s in sse.items()}
    return min(rmse, key=lambda a: rmse[a]), rmse


def fit_models(
    rows_list: Sequence[FwdRows],
    *,
    hidden: tuple[int, ...] = MLP_HIDDEN,
    mlp_max_iter: int = MLP_MAX_ITER,
    alphas: Sequence[float] = RIDGE_ALPHAS,
) -> dict[str, FittedModel]:
    """学習用の行（ログごとのリスト。GroupKFold のグループにするため連結しない）から
    3 種を学習する。学習時間はモデルごとに計る（Ridge は alpha の交差検証を含む）。"""
    parts = [r for r in rows_list if len(r) > 0]
    if not parts:
        raise ValueError("学習用の行が空です")
    sources = [r.source for r in parts]
    xa, ya, _ = _xy(parts, "accel")
    xb, yb, _ = _xy(parts, "brake")
    out: dict[str, FittedModel] = {}

    t0 = time.perf_counter()
    out[NAME_LINEAR] = FittedModel(
        NAME_LINEAR,
        _poly_pipe(LinearRegression()).fit(xa, ya),
        _poly_pipe(LinearRegression()).fit(xb, yb),
        train_time_s=time.perf_counter() - t0,
        sources=sources,
    )

    t0 = time.perf_counter()
    alpha, _ = select_ridge_alpha(parts, alphas)
    out[NAME_RIDGE] = FittedModel(
        NAME_RIDGE,
        _poly_pipe(Ridge(alpha=alpha)).fit(xa, ya),
        _poly_pipe(Ridge(alpha=alpha)).fit(xb, yb),
        alpha=alpha,
        train_time_s=time.perf_counter() - t0,
        sources=sources,
    )

    t0 = time.perf_counter()
    out[NAME_MLP] = FittedModel(
        NAME_MLP,
        _fit_scaled_target(lambda: _mlp_pipe(hidden, mlp_max_iter), xa, ya),
        _fit_scaled_target(lambda: _mlp_pipe(hidden, mlp_max_iter), xb, yb),
        hidden=hidden,
        train_time_s=time.perf_counter() - t0,
        sources=sources,
    )
    return out


def monotone_violation(tb: ForwardTable) -> tuple[float, float]:
    """単調化で動いた量 |accel_grid − accel_before_mono| の (最大, 平均) [km/h/s]。
    数えるのは C7 で実測・穴埋めだった点だけ（データの外は入れない）。点の種類は tb.point_kind。"""
    if tb.accel_before_mono is None:
        raise ValueError("accel_before_mono がありません")
    m = (tb.point_kind == POINT_MEASURED) | (tb.point_kind == POINT_FILLED)
    d = np.abs(tb.accel_grid - tb.accel_before_mono)[m]
    if len(d) == 0:
        return 0.0, 0.0
    return float(d.max()), float(d.mean())


_COPY_META = (
    "accel_deadband_pct",
    "brake_deadband_pct",
    "accel_lag_s",
    "brake_lag_s",
    "accel_window_s",
    "coast_lag_s",
    "opening_source",
    "u_step_pct",
    "min_s",
)


def raw_grid(fitted: FittedModel, like: ForwardTable, *, coast_from: str = "mean") -> np.ndarray:
    """学習済みモデルを like（C7 の表）と同じ格子で評価した、単調化する前の配列。
    行 = 車速、列 = u。
    coast_from: u = 0 の取り方。"mean" は 2 式の平均、"accel" はアクセル側の式。"""
    if coast_from not in ("mean", "accel"):
        raise ValueError(f"coast_from が不明: {coast_from}")
    rep, ug = like.speed_rep, like.u_grid
    vv = np.repeat(rep, len(ug))
    uu = np.tile(ug, len(rep))
    x = np.column_stack([vv, uu])
    acc = np.asarray(fitted.accel_side.predict(x)).reshape(len(rep), len(ug))
    brk = np.asarray(fitted.brake_side.predict(x)).reshape(len(rep), len(ug))
    zero = acc if coast_from == "accel" else (acc + brk) / 2.0
    return np.where(ug[None, :] > 0, acc, np.where(ug[None, :] < 0, brk, zero))


def grid_from_raw(raw: np.ndarray, like: ForwardTable, meta_extra: dict[str, Any]) -> ForwardTable:
    """単調化する前の配列を、行ごとの IsotonicRegression（重みは一様）で単調化して
    ForwardTable にする。"""
    rep, ug = like.speed_rep, like.u_grid
    mono = np.empty_like(raw)
    for r in range(len(rep)):
        mono[r] = IsotonicRegression(increasing=True).fit_transform(ug, raw[r])
    meta: dict[str, Any] = {k: like.meta[k] for k in _COPY_META if k in like.meta}
    meta.update(meta_extra)
    tb = ForwardTable(
        speed_edges=like.speed_edges.copy(),
        speed_rep=rep.copy(),
        u_grid=ug.copy(),
        accel_grid=mono,
        seconds=like.seconds.copy(),
        point_kind=like.point_kind.copy(),
        meas_u_min=like.meas_u_min.copy(),
        meas_u_max=like.meas_u_max.copy(),
        accel_before_mono=raw,
        meta=meta,
        kind=KIND_MODEL,
    )
    vmax, vmean = monotone_violation(tb)
    tb.meta["mono_violation_max"] = vmax
    tb.meta["mono_violation_mean"] = vmean
    return tb


def model_to_grid(
    fitted: FittedModel, like: ForwardTable, *, coast_from: str = "mean"
) -> ForwardTable:
    """学習済みモデルを like（C7 の表）と同じ格子で評価し、単調化して ForwardTable にする。"""
    raw = raw_grid(fitted, like, coast_from=coast_from)
    return grid_from_raw(
        raw,
        like,
        {
            "model_name": fitted.name,
            "alpha": fitted.alpha,
            "hidden": fitted.hidden,
            "train_time_s": fitted.train_time_s,
            "target_standardized": fitted.name == NAME_MLP,
            "sources": "+".join(fitted.sources),
            "coast_from": coast_from,
        },
    )


def mean_grid(
    fitted_list: Sequence[FittedModel], like: ForwardTable, *, coast_from: str = "mean"
) -> ForwardTable:
    """乱数違いの複数モデルの raw_grid を平均してから、単調化を 1 回だけかけて表にする
    （ぶれを減らすため。各モデルを単調化してから平均しない）。"""
    if not fitted_list:
        raise ValueError("モデルが空です")
    raw = np.mean([raw_grid(f, like, coast_from=coast_from) for f in fitted_list], axis=0)
    first = fitted_list[0]
    return grid_from_raw(
        raw,
        like,
        {
            "model_name": first.name,
            "alpha": first.alpha,
            "hidden": first.hidden,
            "train_time_s": float(sum(f.train_time_s for f in fitted_list)),
            "target_standardized": first.name == NAME_MLP,
            "sources": "+".join(first.sources),
            "coast_from": coast_from,
            "n_models": len(fitted_list),
        },
    )
