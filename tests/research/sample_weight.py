"""学習サンプルの WLTP 重み付け（ProblemReport_20260925 段2）。

`wltp_grid.py` は `mode_drive`（→ `pattern_drive` → …）を import しており、`ff_candidate.py`
から import すると循環しやすい（`ff_candidate` は `pattern_drive` から import される側）。
重みの計算自体は WLTP の「車速 × 加速度」滞在秒（`wltp_grid.wltp_cells` の結果）と学習行の
(v0, a_req) だけあれば済むので、依存を numpy と `ff_model.STOP_SPEED_KMH` だけに絞った
この独立モジュールに切り出す。

重みの考え方: 学習行が「WLTP でよく使われるセル」に集中していれば下げ、「あまり使われない
セルにしか無い」なら上げる。比べるのは**占有率**（WLTP 全体に対するそのセルの割合 と、
学習行の格子内総数に対するそのセルの行数の割合）であって、秒数・行数そのものではない。
そうしないと、WLTP の総秒数と学習データの総秒数の大小関係がそのまま重みの大小に乗ってしまい、
`w_min`/`w_max` の意味（「このセルを最大何倍/最小何倍まで持ち上げ/下げるか」）が測定のたびに
変わってしまう。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from tests.research.ff_model import STOP_SPEED_KMH

__all__ = ["WltpWeighting", "compute_weights", "summarize"]


@dataclass(frozen=True)
class WltpWeighting:
    """WLTP 重み付けの設定一式。`wltp_grid.wltp_cells` の結果と格子の境界、重みの範囲を持つ。

    Attributes:
        wltp_s: WLTP がセルごとに滞在する秒数（`wltp_grid.wltp_cells` の戻り値）。
            形は (車速ビン数, 加速度ビン数)。
        speed_edges_kmh: 車速ビンの境界（昇順）。`len` はビン数 + 1。
        accel_edges_kmhs: 加速度ビンの境界（昇順）。`len` はビン数 + 1。
        w_min: 重みの下限。
        w_max: 重みの上限。
        enabled: True のときだけ `train_inverse_model_effective` が実際に学習へ重みを渡す。
            False でも `compute_weights` 自体は計算する（`mae_wltp` で比較できるように）。
    """

    wltp_s: np.ndarray
    speed_edges_kmh: tuple[float, ...]
    accel_edges_kmhs: tuple[float, ...]
    w_min: float
    w_max: float
    enabled: bool


def _cell_index(values: np.ndarray, edges: tuple[float, ...]) -> np.ndarray:
    """`np.histogram2d` と同じ境界規則で 1 次元のビン index を返す。

    ビン `j` は `[edges[j], edges[j+1])`。ただし最後のビンだけ右端を含む
    （`np.histogram2d` の規約と同じ）。範囲外（`values < edges[0]` または
    `values > edges[-1]`）は -1。
    """
    edges_arr = np.asarray(edges, dtype=float)
    # edges[1:-1] を境界にした digitize は、そのまま「両端を除く内側の境界」でビンを
    # 割り当てる。x == edges[-1]（最後の境界ちょうど）も、内側境界より大きいので最後の
    # ビンに入る＝histogram2d の「最後のビンは右端を含む」と一致する。
    idx = np.digitize(values, edges_arr[1:-1], right=False)
    in_range = (values >= edges_arr[0]) & (values <= edges_arr[-1])
    return np.where(in_range, idx, -1)


def compute_weights(
    v0: np.ndarray, a_req: np.ndarray, weighting: WltpWeighting
) -> np.ndarray:
    """1 ペダル分の学習行（v0・a_req は同じ長さの配列）の重みを返す（平均 1 に正規化済み）。

    手順:
        1. 各行を格子のセルに割り当てる（`_cell_index`。範囲外・`v0 < STOP_SPEED_KMH` の
           行は「格子外」）。
        2. 格子内の行について、セルごとの占有率比
           `r = (WLTP のそのセルの秒 / WLTP の格子全体の秒) ÷
                (学習行のそのセルの行数 / 学習行の格子内の総行数)`
           を求める。WLTP がそのセルで 0 秒なら（そこは WLTP が全く使わない領域なので）
           `r = 0`。
        3. `r` を `[w_min, w_max]` に clip。格子外の行は 1.0（中立。WLTP の格子で測れない
           領域なので上げも下げもしない）。
        4. 全行の平均が 1 になるよう正規化する（`train_inverse_model_effective` はアクセル・
           ブレーキそれぞれで呼ぶので、ペダルごとに平均 1 になる）。

    Returns:
        空入力なら空配列。格子内の行が 1 つも無ければ全行 1.0（正規化後も 1.0 のまま）。
    """
    v0 = np.asarray(v0, dtype=float)
    a_req = np.asarray(a_req, dtype=float)
    n = len(v0)
    if n == 0:
        return np.empty(0, dtype=float)

    weights = np.ones(n, dtype=float)
    i = _cell_index(v0, weighting.speed_edges_kmh)
    j = _cell_index(a_req, weighting.accel_edges_kmhs)
    in_grid = (i >= 0) & (j >= 0) & (v0 >= STOP_SPEED_KMH)

    if np.any(in_grid):
        wltp_s = weighting.wltp_s
        wltp_total = float(wltp_s.sum())
        ii, jj = i[in_grid], j[in_grid]
        data_total = float(len(ii))  # 学習行の格子内総行数

        counts = np.zeros(wltp_s.shape, dtype=float)
        np.add.at(counts, (ii, jj), 1.0)

        wltp_frac = wltp_s / wltp_total if wltp_total > 0.0 else np.zeros_like(wltp_s)
        data_frac = counts / data_total  # data_total > 0（in_grid が非空なので）

        ratio = np.zeros_like(wltp_frac)
        nonzero = data_frac > 0.0
        ratio[nonzero] = wltp_frac[nonzero] / data_frac[nonzero]
        ratio = np.where(wltp_s > 0.0, ratio, 0.0)  # WLTP が 0 のセルは r = 0（→ clip で w_min）

        row_ratio = ratio[ii, jj]
        weights[in_grid] = np.clip(row_ratio, weighting.w_min, weighting.w_max)

    mean = float(np.mean(weights))
    if mean <= 0.0:
        return weights
    return weights / mean


def summarize(weights: np.ndarray) -> dict[str, float]:
    """重み配列の表示用要約（件数・最小/最大/平均・下限/上限に張り付いた行の割合）。"""
    w = np.asarray(weights, dtype=float)
    if len(w) == 0:
        return {
            "n": 0.0, "min": 0.0, "max": 0.0, "mean": 0.0,
            "at_min_ratio": 0.0, "at_max_ratio": 0.0,
        }
    lo, hi = float(w.min()), float(w.max())
    return {
        "n": float(len(w)),
        "min": lo,
        "max": hi,
        "mean": float(w.mean()),
        "at_min_ratio": float(np.mean(np.isclose(w, lo))),
        "at_max_ratio": float(np.mean(np.isclose(w, hi))),
    }
