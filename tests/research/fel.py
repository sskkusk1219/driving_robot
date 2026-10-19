"""フィードバック誤差学習 FEL: 表の予想の外れを「状態ごと」に測り、補正表 c として pkl に付ける。

2026-10-19 段2s-2（机上の参照: results/analysis/fel_check.py。pandas は使わず numpy だけ）。

考え方:
    r ＝ 実加速度 − f(v, u(t−L))。f は順モデルの表（遅れ L 分前の指令から作った u を引く。
    ブレーキの戻し中は戻しの表、停止側・発進側は専用の表）。r を状態（マス）ごとにならして
    貯めると、表の外れの癖が分かる。次の走行では f + c を引く（`ForwardTable.with_fel`）。

    マス ＝ 車速の帯 × u のビン × ペダル × 戻し中か。アクセルは `accel_u_bin_pct`、ブレーキは
    `brake_u_bin_pct`（floor）。惰行（pedal 0）は u ビン 0 の 1 マス／車速帯。
    その日のずれ: ペダル別の r の平均（offset）を引いてから貯める。

補正の作り方（両方作って pkl に保存し、走るときに yaml の `fel.method` で選ぶ）:
    cell … マスの平均。マスの秒数 < min_cell_s なら 0。上限 ±c_max でクリップ。
    nn   … MLPRegressor(20, 20)・relu を seed 0,1,2 の 3 本平均。特徴 [v, u, pedal, rel] を
           標準化、目標は r − offset。±c_max でクリップ。学習データが近くに無い所（そのマスと、
           同じペダル・rel で車速ビン ±1・u ビン ±1 の隣のマスすべてに行が 1 行も無い）では 0。

更新を止める行（r を貯めない）: v ≤ 速さ境界の最小・基準 ≤ 0.01（停車）・ガバナー作動・
ペダル切替の ±0.5 秒。

CLI（机上）:
    python -m tests.research.fel attach <pkl> <csv> [--out <pkl>]
        走行ログ CSV から補正表を作って pkl に付ける（--out 無しは上書き。f は不変）。
    python -m tests.research.fel score <pkl> <eval_csv>
        pkl の補正表を eval_csv の行に当てて、|e| の「なし→あり」を群別に出す。2 通り出す:
        ① 行に直接 … pkl に残した学習器（マスの値・ニューラルネット）をそのまま行に当てる
        ② 表で引き直し … 格子に写した後の表（with_fel）で予測し直して r を作る（走行と同じ引き方）
        eval 側の offset は eval 側の走りの平均を引く（fel_check と同じ）。
"""

from __future__ import annotations

import argparse
import sys
import warnings
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler

from tests.research.fwd_data import RELEASE_WINDOW_S
from tests.research.fwd_map import ForwardTable, load_forward_map, save_table

DT_S = 0.05  # 走行ログ CSV の刻み（output.csv_interval_s）
REL_ON_PCT = 0.3  # 戻し中とみなす戻し量 [%]
STOP_LOOKAHEAD_S = 5.0  # 基準が停車するまでこの秒数以内なら停止側
LAUNCH_WINDOW_S = 8.0  # 基準が動き出してからこの秒数以内なら発進側
SWITCH_GUARD_S = 0.5  # ペダル切替の前後でこの秒数は捨てる
NN_HIDDEN = (20, 20)
NN_ALPHA = 1e-3
NN_MAX_ITER = 300
NN_SEEDS = (0, 1, 2)
NN_MIN_ROWS = 50  # これ未満の行数では nn を作らない
KIND_MAIN, KIND_STOP, KIND_LAUNCH, KIND_RELEASE = 0, 1, 2, 3

# マスの鍵の符号化: ((車速ビン*3 + ペダル+1)*2 + rel)*1024 + (u ビン+512)
_U_SPAN = 1024
_U_OFF = 512
_V_STRIDE = 3 * 2 * _U_SPAN

_CSV_COLS = (
    "mode_time_s", "ref_speed_kmh", "actual_speed_kmh", "accel_cmd_pct", "brake_cmd_pct",
    "governor_active",
)


@dataclass(frozen=True)
class FelParams:
    """マスの切り方と補正の上限（config.FelSection と同じ名前）。"""

    speed_edges_kmh: tuple[float, ...] = (
        0.3, 2.5, 5.0, 7.5, 10.0, 15.0, 20.0, 30.0, 40.0, 50.0, 60.0, 80.0, 100.0, 140.0,
    )
    accel_u_bin_pct: float = 1.0
    brake_u_bin_pct: float = 0.5
    min_cell_s: float = 2.0
    c_max_kmhs: float = 0.5

    @classmethod
    def from_config(cls, fel: Any) -> FelParams:
        """config.FelSection から作る。"""
        return cls(
            speed_edges_kmh=tuple(float(x) for x in fel.speed_edges_kmh),
            accel_u_bin_pct=float(fel.accel_u_bin_pct),
            brake_u_bin_pct=float(fel.brake_u_bin_pct),
            min_cell_s=float(fel.min_cell_s),
            c_max_kmhs=float(fel.c_max_kmhs),
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "speed_edges_kmh": list(self.speed_edges_kmh),
            "accel_u_bin_pct": self.accel_u_bin_pct,
            "brake_u_bin_pct": self.brake_u_bin_pct,
            "min_cell_s": self.min_cell_s,
            "c_max_kmhs": self.c_max_kmhs,
        }


@dataclass
class FelRows:
    """走行ログから作った、補正の学習・採点に使う行（捨てる行を除いた後）。"""

    r: np.ndarray  # 実加速度 − 表の予測 [km/h/s]
    pedal: np.ndarray  # -1 ブレーキ / 0 惰行 / 1 アクセル
    v: np.ndarray  # 車速 [km/h]
    u: np.ndarray  # 遅れ分前の指令 − min（正がアクセル、負がブレーキ）[%]
    rel: np.ndarray  # 戻し量 > REL_ON_PCT なら 1
    stop20: np.ndarray  # 停車前の 20 km/h 未満のブレーキか（採点の群）
    acc: np.ndarray  # 実加速度
    kind: np.ndarray  # 予測に引いた表: KIND_*
    release: np.ndarray  # 戻し量 [%]（ブレーキの行だけ。ほかは 0）
    dt_s: float = DT_S
    # アクセルの戻し量 [%]（アクセルの行だけ。ほかは 0。2026-10-20 段4-3）。None ＝ 使わない
    accel_release: np.ndarray | None = None

    def __len__(self) -> int:
        return len(self.r)


# ─────────────────────────────────────────────────────────────────────
# 行の作り方
# ─────────────────────────────────────────────────────────────────────


def _read_csv(path: Path) -> dict[str, np.ndarray]:
    """走行ログ CSV から必要な列を float 配列で読む（空欄は nan）。mode_time_s が空の行は捨てる。"""
    with Path(path).open(encoding="utf-8") as f:
        header = f.readline().rstrip("\n").split(",")
    missing = [c for c in _CSV_COLS if c not in header]
    if missing:
        raise ValueError(f"{path} に列がありません: {missing}")
    idx = [header.index(c) for c in _CSV_COLS]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        raw = np.genfromtxt(
            path, delimiter=",", skip_header=1, usecols=idx, dtype=float, filling_values=np.nan,
            encoding="utf-8",
        )
    raw = np.atleast_2d(raw)
    raw = raw[np.isfinite(raw[:, 0])]  # mode_time_s のある行（手順3 の走行中）
    return {c: raw[:, i] for i, c in enumerate(_CSV_COLS)}


def _delay(x: np.ndarray, steps: int, fill: float) -> np.ndarray:
    """x を steps 周期遅らせる（先頭は fill）。"""
    if steps <= 0:
        return x.copy()
    return np.concatenate([np.full(steps, fill), x[:-steps]])


def _centered_mean(x: np.ndarray, w: int) -> np.ndarray:
    """中心移動平均（窓 w。端は縮める。pandas rolling(center=True, min_periods=1) と同じ位置）。"""
    n = len(x)
    cs = np.concatenate([[0.0], np.cumsum(x)])
    i = np.arange(n)
    lo = np.maximum(i - w // 2, 0)
    hi = np.minimum(i + (w - 1) // 2, n - 1)
    return (cs[hi + 1] - cs[lo]) / (hi - lo + 1)


def _trailing_min(x: np.ndarray, k: int) -> np.ndarray:
    """直近 k+1 点（その点を含む。端は縮める）の最小。"""
    pad = np.concatenate([np.full(k, np.inf), x])
    win = np.lib.stride_tricks.sliding_window_view(pad, k + 1)
    return win.min(axis=1)


def _predict(
    table: ForwardTable,
    v: np.ndarray,
    u: np.ndarray,
    kind: np.ndarray,
    release: np.ndarray,
    accel_release: np.ndarray | None = None,
) -> np.ndarray:
    """行ごとの表の予測（kind で停止側・発進側・戻し・本体を引き分ける）。
    停止側は、表が停止側の戻しの層を持つときだけ戻し量を渡す（2026-10-20 段4-3。持たなければ
    今までと同じ）。本体は accel_release（アクセルの戻し量）を渡す（表がアクセルの戻しの表を
    持たなければ無視される）。"""
    pred = np.zeros(len(v))
    m = kind == KIND_STOP
    if m.any():
        stop_rel = release[m] if table.stop_release_accel is not None else None
        pred[m] = table.accel(v[m], u[m], stop=True, release=stop_rel)
    m = kind == KIND_LAUNCH
    if m.any():
        pred[m] = table.accel(v[m], u[m], launch=True)
    m = kind == KIND_RELEASE
    if m.any():
        pred[m] = table.accel(v[m], u[m], release=release[m])
    m = kind == KIND_MAIN
    if m.any():
        arel = None if accel_release is None else accel_release[m]
        pred[m] = table.accel(v[m], u[m], accel_release=arel)
    return pred


def predict_rows(table: ForwardTable, rows: FelRows) -> np.ndarray:
    """rows の各行を table で予測し直した加速度（走行と同じ引き方）。"""
    return _predict(table, rows.v, rows.u, rows.kind, rows.release, rows.accel_release)


def fel_rows(
    csv_path: Path | str,
    table: ForwardTable,
    params: FelParams | None = None,
    *,
    dt_s: float = DT_S,
) -> FelRows:
    """走行ログ CSV から、行ごとの r（実加速度 − 表の予測）とマスの鍵になる量を作る。

    - 実加速度 ＝ 車速の 1 秒中心移動平均の勾配。
    - u ＝ 遅れ（meta の accel_lag_s・brake_lag_s）分前の指令 − min（meta の
      accel_deadband_pct・brake_deadband_pct。値は min）。正がアクセル、負がブレーキ。
    - 戻し量 ＝ u − 直近 RELEASE_WINDOW_S の u の最小（ブレーキの行だけ）。
    - 停止側: ブレーキ・v < stop_max_speed_kmh・基準が 5 秒以内に停車。発進側: アクセル・
      v < stop_max_speed_kmh・発進 8 秒以内。
    """
    p = params or FelParams()
    d = _read_csv(Path(csv_path))
    v, ref = d["actual_speed_kmh"], d["ref_speed_kmh"]
    ac = np.nan_to_num(d["accel_cmd_pct"], nan=0.0)
    bc = np.nan_to_num(d["brake_cmd_pct"], nan=0.0)
    gov = np.nan_to_num(d["governor_active"], nan=0.0) > 0.5
    n = len(v)
    meta = table.meta
    amin, bmin = float(meta["accel_deadband_pct"]), float(meta["brake_deadband_pct"])
    sa = int(round(float(meta["accel_lag_s"]) / dt_s))
    sb = int(round(float(meta["brake_lag_s"]) / dt_s))
    top = float(meta.get("stop_max_speed_kmh", 0.0))

    vs = _centered_mean(np.nan_to_num(v, nan=0.0), int(round(1.0 / dt_s)))
    acc = np.gradient(vs, dt_s)
    ua = np.maximum(_delay(ac, sa, amin) - amin, 0.0)
    ub = np.maximum(_delay(bc, sb, bmin) - bmin, 0.0)
    pedal = np.where(ua > 0, 1, np.where(ub > 0, -1, 0))
    u = np.where(pedal > 0, ua, np.where(pedal < 0, -ub, 0.0))
    k = int(round(RELEASE_WINDOW_S / dt_s))
    release = np.where(pedal < 0, u - _trailing_min(u, k), 0.0)
    accel_release = np.where(pedal > 0, -_trailing_min(-u, k) - u, 0.0)  # 段4-3

    refv = np.nan_to_num(ref, nan=0.0)
    stop_soon = np.zeros(n, bool)
    launch_win = np.zeros(n, bool)
    h = int(round(STOP_LOOKAHEAD_S / dt_s))
    for i in np.flatnonzero((refv[1:] <= 0.01) & (refv[:-1] > 0.01)) + 1:
        stop_soon[max(i - h, 0) : i + 1] = True
    for i in np.flatnonzero((refv[1:] > 0.01) & (refv[:-1] <= 0.01)) + 1:
        launch_win[i : i + int(round(LAUNCH_WINDOW_S / dt_s))] = True
    low = v < top
    is_stop = (pedal < 0) & low & stop_soon
    is_launch = (pedal > 0) & low & launch_win
    is_rel = ~is_stop & ~is_launch & (pedal < 0) & (release > 0)
    kind = np.where(
        is_stop, KIND_STOP,
        np.where(is_launch, KIND_LAUNCH, np.where(is_rel, KIND_RELEASE, KIND_MAIN)),
    )

    vv = np.nan_to_num(v, nan=0.0)
    pred = _predict(table, vv, u, kind, release, accel_release)
    r = acc - pred

    sw = np.concatenate([[False], pedal[1:] != pedal[:-1]])
    g = int(round(SWITCH_GUARD_S / dt_s))
    cs = np.concatenate([[0], np.cumsum(sw)])
    i = np.arange(n)
    near_sw = (cs[np.minimum(i + g, n - 1) + 1] - cs[np.maximum(i - g, 0)]) > 0
    ok = (
        np.isfinite(v) & np.isfinite(ref) & np.isfinite(r)
        & (v > p.speed_edges_kmh[0]) & (refv > 0.01) & ~gov & ~near_sw
    )
    stop20 = is_stop | ((pedal < 0) & (v < 20) & stop_soon)
    rel = (release > REL_ON_PCT).astype(int)
    return FelRows(
        r=r[ok], pedal=pedal[ok].astype(int), v=v[ok], u=u[ok], rel=rel[ok], stop20=stop20[ok],
        acc=acc[ok], kind=kind[ok].astype(int), release=release[ok], dt_s=dt_s,
        accel_release=accel_release[ok],
    )


# ─────────────────────────────────────────────────────────────────────
# マスと補正の学習
# ─────────────────────────────────────────────────────────────────────


def cell_keys(
    v: np.ndarray, u: np.ndarray, pedal: np.ndarray, rel: np.ndarray, p: FelParams
) -> np.ndarray:
    """行ごとのマスの鍵（int）。惰行は u ビン 0。"""
    vb = np.digitize(v, np.asarray(p.speed_edges_kmh))
    ub = np.where(
        pedal > 0,
        np.floor(u / p.accel_u_bin_pct),
        np.where(pedal < 0, np.floor(u / p.brake_u_bin_pct), 0.0),
    ).astype(int)
    ub = np.clip(ub, -_U_OFF, _U_OFF - 1)
    return (((vb * 3 + (pedal + 1)) * 2 + rel) * _U_SPAN + (ub + _U_OFF)).astype(np.int64)


def _neighbor_covered(keys: np.ndarray, occupied: np.ndarray) -> np.ndarray:
    """そのマスと、車速ビン ±1・u ビン ±1 の隣（同じペダル・rel）のどれかに行があるか。"""
    hit = np.zeros(len(keys), bool)
    for dv in (-1, 0, 1):
        for du in (-1, 0, 1):
            hit |= np.isin(keys + dv * _V_STRIDE + du, occupied)
    return hit


@dataclass
class FelFit:
    """学習した補正（マスの平均と nn）。pkl に残し、机上の採点で行に直接当てる（走行は使わない）"""

    params: FelParams
    offset: dict[int, float]  # ペダル別（-1/0/1）の r の平均
    occupied: np.ndarray  # 行が 1 行でもあるマスの鍵（昇順）
    cell_keys: np.ndarray  # 秒数 ≥ min_cell_s のマスの鍵（昇順）
    cell_vals: np.ndarray  # その補正値（クリップ後）
    cell_seconds: np.ndarray  # その秒数
    nn: list[tuple[StandardScaler, MLPRegressor]] | None
    n_rows: int
    # 行が 1 行でもあるマス（occupied）ごとの秒数（埋まり方の表示用。2026-10-19 段2s-2b。
    # 古い pkl は None）
    occupied_seconds: np.ndarray | None = None

    def cell_value(
        self, v: np.ndarray, u: np.ndarray, pedal: np.ndarray, rel: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """cell の補正値と「補正のあるマスか」。無いマスは 0。"""
        keys = cell_keys(v, u, pedal, rel, self.params)
        pos = np.searchsorted(self.cell_keys, keys)
        pos = np.minimum(pos, max(len(self.cell_keys) - 1, 0))
        if len(self.cell_keys) == 0:
            return np.zeros(len(keys)), np.zeros(len(keys), bool)
        found = self.cell_keys[pos] == keys
        return np.where(found, self.cell_vals[pos], 0.0), found

    def nn_value(
        self, v: np.ndarray, u: np.ndarray, pedal: np.ndarray, rel: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """nn の補正値と「近くに学習データがあるか」。近くに無い所は 0。"""
        n = len(v)
        if self.nn is None or n == 0:
            return np.zeros(n), np.zeros(n, bool)
        keys = cell_keys(v, u, pedal, rel, self.params)
        cov = _neighbor_covered(keys, self.occupied)
        out = np.zeros(n)
        if cov.any():
            x = np.column_stack([v, u, pedal, rel])[cov].astype(float)
            pred = np.mean([m.predict(sc.transform(x)) for sc, m in self.nn], axis=0)
            out[cov] = np.clip(pred, -self.params.c_max_kmhs, self.params.c_max_kmhs)
        return out, cov


def fit(rows: FelRows, params: FelParams | None = None) -> FelFit:
    """行から補正（cell と nn）を学習する。乱数は seed 固定で毎回同じ結果。"""
    p = params or FelParams()
    off = _offsets(rows)
    x = rows.r - np.array([off.get(int(pd), 0.0) for pd in rows.pedal])
    keys = cell_keys(rows.v, rows.u, rows.pedal, rows.rel, p)
    uniq, inv = np.unique(keys, return_inverse=True)
    cnt = np.bincount(inv)
    mean = np.bincount(inv, weights=x) / np.maximum(cnt, 1)
    sec = cnt * rows.dt_s
    keep = sec >= p.min_cell_s - 1e-9
    vals = np.clip(mean[keep], -p.c_max_kmhs, p.c_max_kmhs)

    models: list[tuple[StandardScaler, MLPRegressor]] | None = None
    if len(rows) >= NN_MIN_ROWS:
        feat = np.column_stack([rows.v, rows.u, rows.pedal, rows.rel]).astype(float)
        sc = StandardScaler().fit(feat)
        z = sc.transform(feat)
        models = []
        for seed in NN_SEEDS:
            m = MLPRegressor(
                hidden_layer_sizes=NN_HIDDEN, activation="relu", alpha=NN_ALPHA,
                max_iter=NN_MAX_ITER, early_stopping=True, random_state=seed,
            )
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")
                m.fit(z, x)
            models.append((sc, m))
    return FelFit(
        params=p, offset=off, occupied=uniq, cell_keys=uniq[keep], cell_vals=vals,
        cell_seconds=sec[keep], nn=models, n_rows=len(rows), occupied_seconds=sec,
    )


def grid_corrections(model: FelFit, table: ForwardTable) -> dict[str, np.ndarray | None]:
    """表の格子点（speed_rep[i], u_grid[j]）ごとの補正を 4 枚の配列にする。

    c_main（rel=0。本体・停止側・発進側に足す）と c_rel（rel=1。戻しの表の全層に足す）を
    cell と nn それぞれで。u_grid の負はブレーキ、0 は惰行、正はアクセル。nn が作れなかった
    ときは nn の 2 枚は None。
    """
    shape = table.accel_grid.shape
    vv = np.repeat(table.speed_rep[:, None], shape[1], axis=1).ravel()
    uu = np.tile(table.u_grid[None, :], (shape[0], 1)).ravel()
    pedal = np.sign(uu).astype(int)
    out: dict[str, np.ndarray | None] = {}
    for rel in (0, 1):
        rl = np.full(len(vv), rel, dtype=int)
        c, _ = model.cell_value(vv, uu, pedal, rl)
        out[f"cell_{'rel' if rel else 'main'}"] = c.reshape(shape)
        if model.nn is None:
            out[f"nn_{'rel' if rel else 'main'}"] = None
        else:
            c, _ = model.nn_value(vv, uu, pedal, rl)
            out[f"nn_{'rel' if rel else 'main'}"] = c.reshape(shape)
    return out


def attach_fel(
    table: ForwardTable,
    rows: FelRows,
    params: FelParams | None = None,
    *,
    source: str = "",
) -> ForwardTable:
    """学習して補正表を付けた新しい表を返す（f は不変。meta["fel"] に出どころ・offset・マス数）。"""
    model = fit(rows, params)
    g = grid_corrections(model, table)
    nz = {k: (None if v is None else int(np.count_nonzero(v))) for k, v in g.items()}
    meta = dict(table.meta)
    meta["fel"] = {
        "source_csv": source,
        "offset_kmhs": {
            "accel": model.offset.get(1),
            "brake": model.offset.get(-1),
            "coast": model.offset.get(0),
        },
        "rows": len(rows),
        "occupied_cells": int(len(model.occupied)),
        "cells_with_correction": {
            "cell": int(len(model.cell_keys)),
            "nn": int(len(model.occupied)) if model.nn is not None else 0,
        },
        "filled_seconds": float(model.cell_seconds.sum()),
        "total_seconds": len(rows) * rows.dt_s,
        "grid_nonzero": nz,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "params": model.params.as_dict(),
        "nn": {"hidden": list(NN_HIDDEN), "seeds": list(NN_SEEDS), "alpha": NN_ALPHA},
    }
    return replace(
        table,
        fel_cell_main=g["cell_main"], fel_cell_rel=g["cell_rel"],
        fel_nn_main=g["nn_main"], fel_nn_rel=g["nn_rel"],
        fel_fit=model, meta=meta,
    )


# ─────────────────────────────────────────────────────────────────────
# 埋まり方（WLTP が通るマスに補正があるか。2026-10-19 段2s-2b）
# ─────────────────────────────────────────────────────────────────────


@dataclass
class MissingCell:
    """補正の無いマス 1 つ（WLTP が通るのに学習が足りなかった所）。"""

    text: str  # 「車速 a〜b km/h・ブレーキ・開度 x〜y%・戻し中」などの表示
    wltp_s: float  # WLTP がこのマスにいる秒数
    learn_s: float  # 学習の走りがこのマスにいた秒数（min_cell_s 未満）


@dataclass
class FelCoverage:
    """WLTP が走っている時間のうち、補正（cell で min_cell_s 以上）のあるマスにいる時間の割合。"""

    groups: list[tuple[str, float, float]]  # (群の名前, WLTP の秒数, 補正のあるマスの秒数)
    missing: list[MissingCell]  # WLTP の秒数の多い順
    min_cell_s: float

    def ratio(self, name: str) -> float:
        for nm, tot, cov in self.groups:
            if nm == name:
                return cov / tot if tot > 0.0 else float("nan")
        return float("nan")


def _describe_key(key: int, p: FelParams) -> str:
    ub = int(key % _U_SPAN) - _U_OFF
    rest = int(key // _U_SPAN)
    rel = rest % 2
    rest //= 2
    ped = rest % 3 - 1
    vb = rest // 3
    e = p.speed_edges_kmh
    lo = "0" if vb == 0 else f"{e[vb - 1]:g}"
    v_text = f"{lo}〜{e[vb]:g}" if vb < len(e) else f"{e[-1]:g}〜"
    if ped > 0:
        u_text = f"アクセル u {ub * p.accel_u_bin_pct:g}〜{(ub + 1) * p.accel_u_bin_pct:g}%"
    elif ped < 0:
        # ブレーキの u は負。深さ（絶対値）で書く
        d0, d1 = -(ub + 1) * p.brake_u_bin_pct, -ub * p.brake_u_bin_pct
        u_text = f"ブレーキ 深さ {d0:g}〜{d1:g}%"
    else:
        u_text = "惰行"
    return f"{v_text} km/h・{u_text}{'・戻し中' if rel else ''}"


def wltp_coverage(
    table: ForwardTable,
    mode: Any,
    params: FelParams | None = None,
    *,
    dt_s: float = 0.1,
    top_n: int = 10,
) -> FelCoverage | None:
    """WLTP の基準車速から、補正（cell）のあるマスにいる時間の割合を出す。

    要る加速度 ＝ 基準の 1 秒移動平均の勾配 → 表 f の invert で u・ペダル → 戻し中か（u の列から
    fel_rows と同じ定義: 遅れ分前の u、戻し量 ＝ u − 直近 RELEASE_WINDOW_S の u の最小）→ マス。
    補正表（pkl の fel_fit）が無ければ None。停車（基準 ≤ 0.01）と最小の車速帯より遅い所は数えない。
    群: 全体・アクセル・ブレーキ・ブレーキ<20 km/h・停車前<20 km/h（基準が 5 秒以内に停車）。
    """
    fit = table.fel_fit
    if not isinstance(fit, FelFit):
        return None
    p = params or fit.params
    t_ref = np.array([x.time_s for x in mode.reference_speed], dtype=float)
    v_ref = np.array([x.speed_kmh for x in mode.reference_speed], dtype=float)
    grid = np.arange(t_ref[0], t_ref[-1] + 1e-9, dt_s)
    ref = np.interp(grid, t_ref, v_ref)
    n = len(ref)
    vs = _centered_mean(ref, max(1, int(round(1.0 / dt_s))))
    acc = np.gradient(vs, dt_s)
    u_raw, _ = table.invert(ref, acc)
    u_raw = np.asarray(u_raw, dtype=float)
    meta = table.meta
    sa = int(round(float(meta["accel_lag_s"]) / dt_s))
    sb = int(round(float(meta["brake_lag_s"]) / dt_s))
    ua = _delay(np.maximum(u_raw, 0.0), sa, 0.0)
    ub = _delay(np.maximum(-u_raw, 0.0), sb, 0.0)
    pedal = np.where(ua > 0, 1, np.where(ub > 0, -1, 0))
    u = np.where(pedal > 0, ua, np.where(pedal < 0, -ub, 0.0))
    k = int(round(RELEASE_WINDOW_S / dt_s))
    release = np.where(pedal < 0, u - _trailing_min(u, k), 0.0)
    rel = (release > REL_ON_PCT).astype(int)
    stop_soon = np.zeros(n, bool)
    h = int(round(STOP_LOOKAHEAD_S / dt_s))
    for i in np.flatnonzero((ref[1:] <= 0.01) & (ref[:-1] > 0.01)) + 1:
        stop_soon[max(i - h, 0) : i + 1] = True
    ok = (ref > 0.01) & (ref > p.speed_edges_kmh[0])
    _, found = fit.cell_value(ref, u, pedal, rel)
    low = ref < 20.0
    sel = {
        "全体": ok,
        "アクセル": ok & (pedal == 1),
        "ブレーキ": ok & (pedal == -1),
        "ブレーキ<20 km/h": ok & (pedal == -1) & low,
        "停車前<20 km/h": ok & (pedal == -1) & low & stop_soon,
    }
    groups = [(nm, float(m.sum() * dt_s), float((m & found).sum() * dt_s)) for nm, m in sel.items()]
    keys = cell_keys(ref, u, pedal, rel, p)
    miss_keys, cnt = np.unique(keys[ok & ~found], return_counts=True)
    order = np.argsort(-cnt, kind="stable")[:top_n]
    learn = {}
    if fit.occupied_seconds is not None:
        learn = dict(zip(fit.occupied.tolist(), fit.occupied_seconds.tolist(), strict=True))
    missing = [
        MissingCell(
            _describe_key(int(miss_keys[i]), p), float(cnt[i] * dt_s),
            float(learn.get(int(miss_keys[i]), 0.0)),
        )
        for i in order
    ]
    return FelCoverage(groups=groups, missing=missing, min_cell_s=p.min_cell_s)


def format_coverage(cov: FelCoverage) -> str:
    """埋まり方の表示（群別の割合と、足りないマスの上位）。"""
    lines = [f"FEL の埋まり方（WLTP が走っている時間のうち、補正のあるマス＝学習で "
             f"{cov.min_cell_s:g} 秒以上いたマスにいる時間の割合）:"]
    for nm, tot, c in cov.groups:
        r = f"{c / tot * 100:5.1f}%" if tot > 0.0 else "   -  "
        lines.append(f"  {nm:12s} {r}（{c:6.1f} / {tot:6.1f} s）")
    if cov.missing:
        lines.append("  足りないマス（WLTP の秒数の多い順。学習の秒数は min_cell_s 未満）:")
        for i, m in enumerate(cov.missing, 1):
            lines.append(f"    {i:2d}. {m.text}　WLTP {m.wltp_s:5.1f}s・学習 {m.learn_s:4.2f}s")
    else:
        lines.append("  足りないマスはありません")
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────
# 採点（机上）
# ─────────────────────────────────────────────────────────────────────


def _groups(rows: FelRows) -> dict[str, np.ndarray]:
    low = rows.v < 20
    return {
        "全体": np.ones(len(rows), bool),
        "アクセル": rows.pedal == 1,
        "ブレーキ": rows.pedal == -1,
        "惰行": rows.pedal == 0,
        "ブレーキ<20": (rows.pedal == -1) & low,
        "ブレーキ戻し": (rows.pedal == -1) & (rows.rel == 1),
        "停車前<20": rows.stop20,
    }


def report(rows: FelRows, e0: np.ndarray, e1: np.ndarray, cov: np.ndarray, title: str) -> None:
    """fel_check.report と同じ群で |e| 平均と p95 の「なし→あり」、補正のある行の割合を出す。"""
    print(f"\n== {title}")
    print(f"{'群':12s} {'秒':>6s} {'補正行':>6s} {'|e|平均 なし→あり':>26s} "
          f"{'p95 なし→あり':>26s} {'平均':>14s}")
    for nm, m in _groups(rows).items():
        if m.sum() == 0:
            continue
        a, b = np.abs(e0[m]), np.abs(e1[m])
        pa, pb = np.quantile(a, 0.95), np.quantile(b, 0.95)
        print(
            f"{nm:12s} {m.sum() * rows.dt_s:6.0f} {cov[m].mean() * 100:5.0f}% "
            f"{a.mean():7.3f} → {b.mean():6.3f} ({(1 - b.mean() / a.mean()) * 100:+4.0f}%) "
            f"{pa:7.3f} → {pb:6.3f} ({(1 - pb / pa) * 100:+4.0f}%) "
            f"{e0[m].mean():+.3f}→{e1[m].mean():+.3f}"
        )


def score(table: ForwardTable, rows: FelRows) -> None:
    """table の補正表を rows に当てて、直接（学習器）と表で引き直し（with_fel）の両方を出す。"""
    off = _offsets(rows)
    e0 = rows.r - np.array([off.get(int(pd), 0.0) for pd in rows.pedal])
    print(
        f"eval の走り全体の平均（offset）: アクセル {off.get(1, np.nan):+.3f} "
        f"ブレーキ {off.get(-1, np.nan):+.3f} 惰行 {off.get(0, np.nan):+.3f}  行 {len(rows)}"
    )
    model = table.fel_fit
    if isinstance(model, FelFit):
        rounded = {k: round(v, 3) for k, v in model.offset.items()}
        print(f"学習側の offset: {rounded}  行 {model.n_rows}")
        for method in ("cell", "nn"):
            fn = model.cell_value if method == "cell" else model.nn_value
            c, cov = fn(rows.v, rows.u, rows.pedal, rows.rel)
            report(rows, e0, e0 - c, cov, f"① 行に直接 {method}")
    else:
        print("（pkl に学習器 fel_fit が無いので ① 行に直接 は省略）")
    for method in ("cell", "nn"):
        if not table.has_fel(method):
            continue
        t2 = table.with_fel(method)
        pred_new = predict_rows(t2, rows)
        pred_old = predict_rows(table, rows)
        e1 = rows.acc - pred_new - off_vec(rows, off)
        cov = np.abs(pred_new - pred_old) > 1e-9
        report(rows, e0, e1, cov, f"② 表で引き直し {method}（with_fel）")


def _offsets(rows: FelRows) -> dict[int, float]:
    """ペダル別（-1/0/1）の r の平均（その走りの全体のずれ）。"""
    return {
        int(pd): float(rows.r[rows.pedal == pd].mean())
        for pd in (-1, 0, 1)
        if (rows.pedal == pd).any()
    }


def off_vec(rows: FelRows, off: dict[int, float]) -> np.ndarray:
    return np.array([off.get(int(pd), 0.0) for pd in rows.pedal])


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m tests.research.fel", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("attach", help="走行ログ CSV から補正表を作って pkl に付ける")
    a.add_argument("pkl")
    a.add_argument("csv")
    a.add_argument("--out", default=None, help="出力 pkl（無しは上書き）")
    s = sub.add_parser("score", help="pkl の補正表を eval の走行ログ CSV に当てて採点する")
    s.add_argument("pkl")
    s.add_argument("csv")
    args = ap.parse_args(argv)
    table = load_forward_map(Path(args.pkl))
    params = FelParams()
    if args.cmd == "attach":
        rows = fel_rows(args.csv, table, params)
        out = attach_fel(table, rows, params, source=Path(args.csv).name)
        path = save_table(out, Path(args.out) if args.out else Path(args.pkl))
        info = out.meta["fel"]
        print(f"保存: {path}")
        print(
            f"学習 {info['source_csv']}  行 {info['rows']}（{info['total_seconds']:.0f} 秒）"
            f"  マス {info['occupied_cells']}"
            f"  補正あり(cell) {info['cells_with_correction']['cell']}"
            f"  埋まった秒数 {info['filled_seconds']:.0f}  offset {info['offset_kmhs']}"
        )
    else:
        score(table, fel_rows(args.csv, table, params))
    return 0


if __name__ == "__main__":
    # -m で動かすと FelFit のモジュール名が __main__ になり、pkl の学習器が他から読めなくなる。
    # 本来のモジュール名で読み直した main を呼ぶ
    from tests.research.fel import main as _main

    sys.exit(_main())
