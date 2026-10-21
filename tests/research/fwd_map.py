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

発進側・停止側（段2o-10。attach_launch・attach_stop）: 低速の車速の行だけ、別の行（発進の区間・
停止へ向かう区間）から同じ考え方（マスの中央値・穴は直線・単調化）で 0.25% マスで作り直した配列
（launch_accel・stop_accel）と点の種類（launch_kind・stop_kind）を持つ。実測の外側は端の値で
止めず、本体の傾きでつなぐ（発進側の車速 0 付近の行は動き出す開度より深い側を 0 で埋める）。

戻しの表（段2o-5）・アクセルの戻しの表・停止側の戻しの層（段4-3。attach_accel_release・
attach_stop_release）: 戻し量の節（0＝本体、release_grid）の間を直線補間して引く。アクセルの戻しの
表は u > 0 の列に効き（accel_release）、停止側の戻しの層は stop=True の引きに効く（release）。
どれも None の表は今までと完全に同じ。

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
    FINE_SPEED_STEP_KMH,
    STOP_SPEED_KMH,
    Breakaway,
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
# 発進側・停止側の表（attach_launch・attach_stop。2026-10-18 段2o-10）:
#   SIDE_U_STEP_PCT … マス幅 [%]。手順2 の低速の崖は約 1.5% 幅で、0.5% マスでは 1〜3 点しか入らず
#     2 段の階段になった（段2o-10 の確認）。0.25% にして崖を 6 点で分ける（本体の格子は今のまま）
#   LOW_SPEED_MARGIN_KMH … 置き換える車速の行は「代表車速 ≦ クリープ車速（手順2 の実測）＋余裕」。
#     余裕は車速の行 1 マス分（FINE_SPEED_STEP_KMH）。根拠: 表は行の代表車速の間を直線補間するので、
#     クリープ車速の行の上の行まで置き換えないと、クリープ車速付近で本体と継ぎ目ができる。
#     車の値は直接置かない（クリープ車速は config から渡る）。仮値。実機の比較で決める
SIDE_U_STEP_PCT = 0.25
LOW_SPEED_MARGIN_KMH = FINE_SPEED_STEP_KMH
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
    # 停止側（基準車速が下がって停車へ向かうときに引く。2026-10-18 段2o-10。attach_stop）。
    # launch_accel と同じ形・同じ置き方。None ＝ 停止側なし
    stop_accel: np.ndarray | None = None
    # 発進側・停止側の配列の点の種類（POINT_*。accel_grid と同じ形）。None ＝ 本体の point_kind
    launch_kind: np.ndarray | None = None
    stop_kind: np.ndarray | None = None
    # 発進側・停止側の表を車速方向に補間するときの行の代表車速（2026-10-22 段5-4 e。
    # apply_side_row_speed_rep）。speed_rep と同じ長さ。置き換えた行だけ学習データの車速の中央値、
    # それ以外は speed_rep と同じ。None ＝ 本体と同じ speed_rep（今までと完全に同じ）
    launch_speed_rep: np.ndarray | None = None
    stop_speed_rep: np.ndarray | None = None
    # 戻しの表（2026-10-14 段2o-5。attach_release）。release_grid ＝ 戻し量 [%] の格子（昇順・正）、
    # release_accel ＝ (戻し量の数, 行, 格子点) 加速度。戻し量 0 は本体 accel_grid。
    # None ＝ 戻しの表なし（今までと完全に同じ）。戻し量は meta["release_max_pct"] で頭打ちにする
    release_grid: np.ndarray | None = None
    release_accel: np.ndarray | None = None
    # アクセルの戻しの表（2026-10-20 段4-3。attach_accel_release）。ブレーキの戻しの表のアクセル版。
    # accel_release_grid ＝ アクセルの戻し量 [%] の格子（昇順・正）、accel_release_accel ＝
    # (戻し量の数, 行, 格子点) 加速度（u > 0 の列だけ意味がある。u <= 0 の列は本体と同じ）。
    # 戻し量 0 は本体 accel_grid。戻し量は meta["accel_release_max_pct"] で頭打ちにする。
    # None ＝ アクセルの戻しの表なし（今までと完全に同じ）
    accel_release_grid: np.ndarray | None = None
    accel_release_accel: np.ndarray | None = None
    # 停止側の戻しの表（2026-10-20 段4-3。attach_stop_release）。
    # stop_release_grid ＝ ブレーキの戻し量 [%] の格子、stop_release_accel ＝
    # (戻し量の数, 行, 格子点) 加速度。戻し量 0 は stop_accel。
    # 層に行が足りない車速の行は一つ下の層（最後は stop_accel）の値のまま。頭打ちは
    # meta["stop_release_max_pct"]。None ＝ 停止側は戻し量を見ない（今までと完全に同じ）
    stop_release_grid: np.ndarray | None = None
    stop_release_accel: np.ndarray | None = None
    # FEL の補正表（2026-10-19 段2s-2。fel.py の attach が付ける）。格子点ごとの補正 [km/h/s]。
    # *_main は本体・停止側・発進側に足す（戻し量 0）、*_rel は戻しの表の全層に足す（戻し中）。
    # cell ＝ マスの平均、nn ＝ 小さなニューラルネット。None ＝ 補正なし（今までと完全に同じ）。
    # 元の accel_grid 等は書き換えない。足すのは with_fel
    fel_cell_main: np.ndarray | None = None
    fel_cell_rel: np.ndarray | None = None
    fel_nn_main: np.ndarray | None = None
    fel_nn_rel: np.ndarray | None = None
    # FEL を作ったときの学習器（マスの平均・ニューラルネット。fel.py の FelFit）。机上の score が
    # 行に直接当てるのに使う。走行では使わない。None ＝ なし
    fel_fit: Any = None

    def has_fel(self, method: str | None = None) -> bool:
        """補正表があるか（method を渡すとその方法の補正があるか）。"""
        names = (method,) if method is not None else ("cell", "nn")
        return any(
            getattr(self, f"fel_{m}_main") is not None or getattr(self, f"fel_{m}_rel") is not None
            for m in names
        )

    def with_fel(self, method: str) -> ForwardTable:
        """補正 c を足した新しい表を返す（2026-10-19 段2s-2）。補正表が無ければ self のまま。

        accel_grid・stop_accel・launch_accel（あるものだけ）に c_main、release_accel の全層に
        c_rel を足し、各車速の行で u に対して単調（加速度が u とともに増える）になるよう
        build_table と同じ IsotonicRegression（重み一様）でそろえる。point_kind 等はそのまま。
        """
        if method not in ("cell", "nn"):
            raise ValueError(f"method は cell か nn: {method!r}")
        c_main = getattr(self, f"fel_{method}_main")
        c_rel = getattr(self, f"fel_{method}_rel")
        if c_main is None and c_rel is None:
            return self
        meta = dict(self.meta)
        meta["fel_applied"] = method
        out = replace(self, meta=meta)
        if c_main is not None:
            ug = self.u_grid
            out.accel_grid = _monotone_rows(self.accel_grid + c_main, ug)
            if self.stop_accel is not None:
                out.stop_accel = _monotone_rows(self.stop_accel + c_main, ug)
            if self.launch_accel is not None:
                out.launch_accel = _monotone_rows(self.launch_accel + c_main, ug)
            if self.stop_release_accel is not None:
                out.stop_release_accel = np.stack(
                    [_monotone_rows(layer + c_main, ug) for layer in self.stop_release_accel]
                )
        if c_rel is not None and self.release_accel is not None:
            out.release_accel = np.stack(
                [_monotone_rows(layer + c_rel, self.u_grid) for layer in self.release_accel]
            )
        return out

    def _release_nodes(
        self, release: Any, launch: bool, shape: tuple[int, ...], stop: bool = False
    ) -> tuple[list[np.ndarray], np.ndarray, np.ndarray, np.ndarray] | None:
        """戻し量を使うときの (節の配列のリスト, 節の戻し量, 頭打ちした戻し量, 点の種類の配列)。
        使わないなら None。

        使わない: release が None・戻しの表なし・発進側の表を引くとき（launch=True で
        launch_accel があるとき。発進は戻しではないので戻し量は無視する）。
        停止側（stop=True で stop_accel があるとき）は、停止側の戻しの表（stop_release_accel。
        2026-10-20 段4-3）があるときだけ戻し量を使い、無ければ戻し量は無視する（今までと同じ）。
        節は 0（本体。停止側なら stop_accel）と release_grid（停止側なら stop_release_grid）。
        頭打ちは meta の release_max_pct（停止側は stop_release_max_pct。無ければ同じ）。
        """
        if launch and stop:
            raise ValueError("launch と stop は同時に True にできません")
        if release is None:
            return None
        if stop and self.stop_accel is not None:
            if self.stop_release_grid is None or self.stop_release_accel is None:
                return None
            base, layers, grid = self.stop_accel, self.stop_release_accel, self.stop_release_grid
            kind = self._kind(False, True)
            cap = self.meta.get("stop_release_max_pct", self.meta.get("release_max_pct"))
        else:
            if self.release_grid is None or self.release_accel is None:
                return None
            if launch and self.launch_accel is not None:
                return None
            base, layers, grid = self.accel_grid, self.release_accel, self.release_grid
            kind = self.point_kind
            cap = self.meta.get("release_max_pct")
        vals = np.concatenate([[0.0], np.asarray(grid, dtype=float)])
        top = float(vals[-1])
        if cap is not None and np.isfinite(cap):
            top = min(top, float(cap))
        r = np.clip(np.broadcast_to(np.asarray(release, dtype=float), shape), 0.0, top)
        return [base, *layers], vals, r, kind

    def _accel_release_nodes(
        self, accel_release: Any, launch: bool, shape: tuple[int, ...], stop: bool = False
    ) -> tuple[list[np.ndarray], np.ndarray, np.ndarray] | None:
        """アクセルの戻し量を使うときの (節の配列のリスト, 節の戻し量, 頭打ちした戻し量)。
        使わないなら None（2026-10-20 段4-3）。

        使わない: accel_release が None・アクセルの戻しの表なし・発進側／停止側の表を引くとき
        （launch=True で launch_accel があるとき、stop=True で stop_accel があるとき）。
        節は 0（本体）と accel_release_grid。頭打ちは meta の accel_release_max_pct。
        """
        if launch and stop:
            raise ValueError("launch と stop は同時に True にできません")
        if accel_release is None or self.accel_release_grid is None:
            return None
        if self.accel_release_accel is None:
            return None
        if (launch and self.launch_accel is not None) or (stop and self.stop_accel is not None):
            return None
        vals = np.concatenate([[0.0], np.asarray(self.accel_release_grid, dtype=float)])
        top = float(vals[-1])
        cap = self.meta.get("accel_release_max_pct")
        if cap is not None and np.isfinite(cap):
            top = min(top, float(cap))
        r = np.clip(np.broadcast_to(np.asarray(accel_release, dtype=float), shape), 0.0, top)
        return [self.accel_grid, *self.accel_release_accel], vals, r

    def _blend(
        self,
        arrs: list[np.ndarray],
        vals: np.ndarray,
        r: np.ndarray,
        v: np.ndarray,
        u: np.ndarray,
        speed_rep: np.ndarray | None = None,
    ) -> np.ndarray:
        """戻し量の節の間を直線補間して引いた加速度（accel の戻し量ありの計算）。"""
        k, w = _release_weights(vals, r)
        out = np.zeros(v.shape)
        for j, arr in enumerate(arrs):
            h = np.where(k == j, 1.0 - w, 0.0) + np.where(k + 1 == j, w, 0.0)
            if h.any():
                out = out + h * np.asarray(
                    grid_accel(self.speed_rep if speed_rep is None else speed_rep,
                               self.u_grid, arr, v, u)
                )
        return out

    def _rep(self, launch: bool, stop: bool = False) -> np.ndarray:
        """車速方向の補間に使う行の代表車速（_grid と同じ選び方。発進側・停止側の専用の代表車速が
        あり、その表を引くときだけそれを使う。2026-10-22 段5-4）。"""
        if launch and self.launch_accel is not None and self.launch_speed_rep is not None:
            return self.launch_speed_rep
        if stop and self.stop_accel is not None and self.stop_speed_rep is not None:
            return self.stop_speed_rep
        return self.speed_rep

    def _grid(self, launch: bool, stop: bool = False) -> np.ndarray:
        if launch and stop:
            raise ValueError("launch と stop は同時に True にできません")
        if launch and self.launch_accel is not None:
            return self.launch_accel
        if stop and self.stop_accel is not None:
            return self.stop_accel
        return self.accel_grid

    def _kind(self, launch: bool, stop: bool = False) -> np.ndarray:
        if launch and self.launch_accel is not None and self.launch_kind is not None:
            return self.launch_kind
        if stop and self.stop_accel is not None and self.stop_kind is not None:
            return self.stop_kind
        return self.point_kind

    def accel(
        self,
        v_kmh: np.ndarray | float,
        u_pct: np.ndarray | float,
        *,
        launch: bool = False,
        release: np.ndarray | float | None = None,
        stop: bool = False,
        accel_release: np.ndarray | float | None = None,
    ) -> Any:
        """車速 v・効く開度 u での加速度 [km/h/s]（ベクトル入力可）。

        launch=True で発進側の表があればそちらを引く（無ければ本体）。stop=True は停止側
        （launch と同時に True はエラー）。
        release（ブレーキの戻し量 [%]。スカラーか配列）を渡すと、戻しの表があるとき戻し量の節の間
        （0＝本体、release_grid）を直線補間して引く（release_max_pct で頭打ち）。release が
        None・0、戻しの表なし、発進側の表を引くときは今までと同じ。停止側（stop=True）は
        停止側の戻しの表（stop_release_accel）があるときだけ戻し量を使う（2026-10-20 段4-3）。
        accel_release（アクセルの戻し量 [%]。2026-10-20 段4-3）は、アクセルの戻しの表が
        あるとき u > 0 の点を同じ方法で引く（accel_release_max_pct で頭打ち）。None・0、表なし、
        発進側・停止側の表を引くときは今までと同じ。
        """
        v, u = np.broadcast_arrays(np.asarray(v_kmh, dtype=float), np.asarray(u_pct, dtype=float))
        nodes = self._release_nodes(release, launch, v.shape, stop)
        anodes = self._accel_release_nodes(accel_release, launch, v.shape, stop)
        if nodes is None and anodes is None:
            return grid_accel(
                self._rep(launch, stop), self.u_grid, self._grid(launch, stop), v_kmh, u_pct
            )
        if nodes is None:
            out_b = np.asarray(
                grid_accel(self._rep(launch, stop), self.u_grid, self._grid(launch, stop), v, u)
            )
        else:
            arrs, vals, r, _ = nodes
            out_b = self._blend(arrs, vals, r, v, u, self._rep(launch, stop))
        if anodes is None:
            return _scalar_or_array(out_b)
        arrs_a, vals_a, r_a = anodes
        out_a = self._blend(arrs_a, vals_a, r_a, v, u, self._rep(launch, stop))
        return _scalar_or_array(np.where(u > 0.0, out_a, out_b))

    def invert(
        self,
        v_kmh: np.ndarray | float,
        a_kmhs: np.ndarray | float,
        *,
        launch: bool = False,
        release: np.ndarray | float | None = None,
        stop: bool = False,
        accel_release: np.ndarray | float | None = None,
    ) -> tuple[Any, Any]:
        """車速 v で加速度 a になる u と点の種類 POINT_*（ベクトル入力可）。launch・release は
        accel と同じ（release は 1 サンプルごとに違う値を渡せる。サンプルごとに補間した列で解く）。
        accel_release（2026-10-20 段4-3）は u > 0 の列だけ、アクセルの戻しの表で補間した列を使う。
        """
        v, a = np.broadcast_arrays(np.asarray(v_kmh, dtype=float), np.asarray(a_kmhs, dtype=float))
        nodes = self._release_nodes(release, launch, v.shape, stop)
        anodes = self._accel_release_nodes(accel_release, launch, v.shape, stop)
        if nodes is None and anodes is None:
            return grid_invert(
                self._rep(launch, stop), self.u_grid, self._grid(launch, stop),
                self._kind(launch, stop), v_kmh, a_kmhs,
            )
        extra: dict[str, Any] = {}
        if anodes is not None:
            extra = {
                "accel_release_nodes": anodes[0],
                "accel_release_node_pct": anodes[1],
                "accel_release": anodes[2],
            }
        if nodes is None:
            return grid_invert(
                self._rep(launch, stop), self.u_grid, self._grid(launch, stop),
                self._kind(launch, stop), v, a, **extra,
            )
        arrs, vals, r, kind = nodes
        return grid_invert(
            self._rep(launch, stop), self.u_grid, arrs[0], kind, v, a,
            release_nodes=arrs, release_node_pct=vals, release=r, **extra,
        )

    def coast_accel(
        self, v_kmh: np.ndarray | float, *, launch: bool = False, stop: bool = False
    ) -> Any:
        """u=0（惰行）の加速度 [km/h/s]。launch・stop は accel と同じ。"""
        return grid_accel(
            self._rep(launch, stop), self.u_grid, self._grid(launch, stop), v_kmh, 0.0
        )

    def save(self, path: Path | None = None) -> Path:
        return save_table(self, path)


def _monotone_rows(arr: np.ndarray, u_grid: np.ndarray) -> np.ndarray:
    """行ごとに u に対して単調（増加）にそろえる（IsotonicRegression・重み一様）。"""
    out = np.empty_like(arr, dtype=float)
    for r in range(arr.shape[0]):
        out[r] = IsotonicRegression(increasing=True).fit_transform(u_grid, arr[r])
    return out


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


def _blend_columns(
    nodes: Sequence[np.ndarray],
    node_pct: np.ndarray,
    r: np.ndarray,
    i0: np.ndarray,
    i1: np.ndarray,
    w: np.ndarray,
    kg: int,
) -> np.ndarray:
    """サンプルごとに、戻し量の節の間を直線補間した列（車速は上下 2 行を直線補間）。"""
    kr, wr = _release_weights(node_pct, r)
    c = np.zeros((len(r), kg))
    for j, arr in enumerate(nodes):
        h = np.where(kr == j, 1.0 - wr, 0.0) + np.where(kr + 1 == j, wr, 0.0)
        if h.any():
            cj = (1.0 - w)[:, None] * arr[i0] + w[:, None] * arr[i1]
            c = c + h[:, None] * cj
    return c


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
    accel_release_nodes: Sequence[np.ndarray] | None = None,
    accel_release_node_pct: np.ndarray | None = None,
    accel_release: np.ndarray | None = None,
) -> tuple[Any, Any]:
    """車速 v で加速度 a になる u と点の種類を返す（ベクトル入力可）。

    release_nodes（戻し量の節ごとの (行, 格子点) 配列。先頭 ＝ 戻し量 0）・release_node_pct（節の
    戻し量。昇順）・release（サンプルごとの戻し量。v と同じ形）を渡すと、accel の代わりに
    サンプルごとに節の間を直線補間した列を使う（2026-10-14 段2o-5）。渡さなければ今までと同じ。
    accel_release_nodes・accel_release_node_pct・accel_release は、アクセルの戻し量について同じ
    （2026-10-20 段4-3）。u > 0 の列だけをその補間した列にし、u <= 0 の列はブレーキ側の列
    （release を渡していれば戻し量で補間した列、なければ accel）のまま。渡さなければ今までと同じ。

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
    arf = None
    if accel_release_nodes is not None:
        assert accel_release_node_pct is not None and accel_release is not None  # noqa: S101
        arf = np.broadcast_to(np.asarray(accel_release, dtype=float), shape).reshape(-1)
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
            c = _blend_columns(release_nodes, release_node_pct, rf[sl], i0, i1, w, kg)
        if arf is not None:
            c_a = _blend_columns(
                accel_release_nodes, accel_release_node_pct, arf[sl], i0, i1, w, kg
            )
            c = np.where(u_grid[None, :] > 0.0, c_a, c)
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
        launch_accel=None,  # 発進側・停止側は refine の後に attach_launch・attach_stop で付ける
    )


def _side_arrays(
    table: ForwardTable,
    rows: FwdRows,
    *,
    creep_speed_kmh: float,
    margin_kmh: float,
    min_s: float,
    breakaway_u_pct: float | None,
    keep_stationary: bool,
    base_accel: np.ndarray | None = None,
    base_kind: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, list[int], float] | None:
    """発進側・停止側の配列を作る（2026-10-18 段2o-10）。作れなければ None。

    返り値: (加速度の配列, 点の種類の配列, 置き換えた行の番号, 動いている行の合計秒数)。
    手順（置き換えた車速の行ごと。マス幅は SIDE_U_STEP_PCT。C7 の build_table と同じ考え方）:
      1. 行の u を SIDE_U_STEP_PCT のマスに分け、マスごとの加速度の中央値と秒数。
      2. 秒数が min_s 以上のマスが実測点。実測点が 2 つ未満の行は置き換えない（本体のまま）。
         keep_stationary（発進側）で動き出す開度 breakaway_u_pct が分かっているときは、停車中の
         行のうち u が動き出す開度以下（深い）ものも実測に入れ（加速度 0）、動き出す開度の
         点 (breakaway, 0) を足す。
      3. 実測点を単調化（IsotonicRegression。重み＝秒数）し、実測点の間は直線（穴は直線）。
      4. 範囲外を作らない。実測の外側は、端の値に本体の傾き（本体の値の差）を足してつなぐ
         （端の値で止めない）。車速 0 付近の行（下端の境界 ＜ 停車の車速）で動き出す開度が分かって
         いるときは、深い側（実測の外側）を 0 で埋め、行全体を 0 未満にしない。
      5. 表の u の格子点（本体の 0.5% 刻み）に直線補間してから、行ごとに単調化し直す。
    使うのは、行の代表車速が creep_speed_kmh + margin_kmh 以下の車速の行。
    base_accel・base_kind（2026-10-20 段4-3）: 置き換える前の配列・点の種類。省略は本体
    （table.accel_grid・point_kind）。行が足りずに置き換えない車速の行はこの配列のまま残り、
    実測の外側をつなぐ傾きもこの配列から取る（停止側の戻しの層は一つ下の層を渡す）。
    """
    edges, grid = table.speed_edges, table.u_grid
    base_accel = table.accel_grid if base_accel is None else base_accel
    base_kind = table.point_kind if base_kind is None else base_kind
    stationary = rows.speed_kmh < STOP_SPEED_KMH
    moving = ~stationary
    moving_s = float(np.sum(rows.dt_row_s[moving])) if len(rows) else 0.0
    if len(rows) == 0 or moving_s < LAUNCH_MIN_TOTAL_S:
        return None
    use_zero = keep_stationary and breakaway_u_pct is not None
    keep = moving.copy()
    if use_zero:
        assert breakaway_u_pct is not None  # noqa: S101
        keep |= stationary & (rows.u_pct <= breakaway_u_pct + _EPS)
    r_all = take_rows(rows, keep)
    sc = speed_cell_edges(r_all.speed_kmh, edges)
    uc = u_cell(r_all.u_pct, SIDE_U_STEP_PCT)
    out = base_accel.copy()
    kind_out = base_kind.copy()
    used: list[int] = []
    for r in np.flatnonzero(table.speed_rep <= creep_speed_kmh + margin_kmh + _EPS):
        sel = sc == r
        zero_deep = use_zero and edges[r] < STOP_SPEED_KMH
        pu: list[float] = []
        pa: list[float] = []
        pw: list[float] = []
        if sel.any():
            ks = uc[sel]
            for k in np.unique(ks):
                m = ks == k
                sec = float(r_all.dt_row_s[sel][m].sum())
                if sec >= min_s:
                    pu.append(float(u_grid_points(int(k), int(k), SIDE_U_STEP_PCT)[0]))
                    pa.append(float(np.median(r_all.accel_kmhs[sel][m])))
                    pw.append(sec)
        meas_u = np.array(pu)
        if zero_deep:
            assert breakaway_u_pct is not None  # noqa: S101
            pu.append(breakaway_u_pct)
            pa.append(0.0)
            pw.append(1.0)
        if len(pu) < 2:
            continue
        order = np.argsort(pu, kind="stable")
        xu, xa, xw = np.array(pu)[order], np.array(pa)[order], np.array(pw)[order]
        iso = IsotonicRegression(increasing=True).fit_transform(xu, xa, sample_weight=xw)
        lo, hi = float(xu[0]), float(xu[-1])
        body = np.interp(grid, grid, base_accel[r])
        b_lo, b_hi = float(np.interp(lo, grid, body)), float(np.interp(hi, grid, body))
        val = np.interp(grid, xu, iso)
        below, above = grid < lo - _EPS, grid > hi + _EPS
        if zero_deep:
            val[below] = max(float(iso[0]), 0.0)
        else:
            val[below] = iso[0] + np.minimum(body[below] - b_lo, 0.0)
        val[above] = iso[-1] + np.maximum(body[above] - b_hi, 0.0)
        if zero_deep:
            val = np.maximum(val, 0.0)
        out[r] = IsotonicRegression(increasing=True).fit_transform(grid, val)
        near = (
            np.min(np.abs(grid[:, None] - (meas_u if len(meas_u) else np.array([np.inf]))[None]),
                   axis=1)
            <= SIDE_U_STEP_PCT + _EPS
        )
        if zero_deep:
            near |= np.abs(grid - float(breakaway_u_pct or 0.0)) <= SIDE_U_STEP_PCT + _EPS
            near |= grid <= float(breakaway_u_pct or 0.0)
        kind = np.full(len(grid), POINT_OUT, dtype=int)
        kind[~below & ~above] = POINT_FILLED
        if zero_deep:
            kind[below] = POINT_FILLED
        kind[near & (~below | zero_deep)] = POINT_MEASURED
        kind_out[r] = kind
        used.append(int(r))
    if not used:
        return None
    return out, kind_out, used, moving_s


def attach_launch(
    table: ForwardTable,
    launch_rows: FwdRows,
    *,
    creep_speed_kmh: float,
    breakaway: Breakaway | None = None,
    margin_kmh: float = LOW_SPEED_MARGIN_KMH,
    min_s: float = LAUNCH_MIN_S,
    min_total_s: float = LAUNCH_MIN_TOTAL_S,
    accel_window_s: float = math.nan,
) -> ForwardTable:
    """発進側の加速度（launch_accel）を付けた新しい表を返す（2026-10-14 段2o-4、10-18 段2o-10 で
    作り直し）。

    launch_rows: 発進側の行（発進の区間 `fwd_data.launch_mask` の行・CREEP_WAIT・
    RAMP_LAUNCH_RELEASE の停車中の行を連結したもの。停車の行を含んでよい）。
    creep_speed_kmh（手順2 で測ったクリープ車速）が 0 以下なら作らない（クリープの無い車。None）。
    作る範囲・作り方は `_side_arrays`（車速の行は代表車速が creep_speed_kmh + margin_kmh 以下）。
    動いている行の合計秒数が min_total_s 未満、または置き換えられる行が無いときも None。
    breakaway（`fwd_data.launch_breakaway`）があれば車速 0 付近の行の深い側を 0 で埋める。
    元の表は書き換えない。accel_window_s は発進側の行を作ったときの加速度の窓で、meta に残すだけ。
    """
    moving_s = (
        float(np.sum(launch_rows.dt_row_s[launch_rows.speed_kmh >= STOP_SPEED_KMH]))
        if len(launch_rows) else 0.0
    )
    meta = dict(table.meta)
    meta.update(
        launch_rows_s=moving_s,
        launch_speed_rows=[],
        launch_min_s=min_s,
        launch_accel_window_s=accel_window_s,
        launch_u_step_pct=SIDE_U_STEP_PCT,
        low_speed_margin_kmh=margin_kmh,
        launch_breakaway_u_pct=None if breakaway is None else breakaway.u_pct,
        launch_breakaway_source=None if breakaway is None else breakaway.source,
        launch_breakaway_u_list=[] if breakaway is None else list(breakaway.per_segment_u_pct),
    )
    none = replace(table, launch_accel=None, launch_kind=None, meta=meta)
    if creep_speed_kmh <= 0.0 or moving_s < min_total_s:
        return none
    res = _side_arrays(
        table, launch_rows, creep_speed_kmh=creep_speed_kmh, margin_kmh=margin_kmh, min_s=min_s,
        breakaway_u_pct=None if breakaway is None else breakaway.u_pct, keep_stationary=True,
    )
    if res is None:
        return none
    out, kind, used, _ = res
    meta["launch_speed_rows"] = [float(table.speed_rep[r]) for r in used]
    return replace(table, launch_accel=out, launch_kind=kind, meta=meta)


# 2026-10-22 段5-3: 発進側のアクセル側（u > 0）を実測の点で作る（apply_launch_accel_measured）。
#   LAUNCH_ACCEL_CELL_PCT … アクセルの開度のマス幅 [%]（ユーザー決定 2026-10-22 の 2%）
#   LAUNCH_ACCEL_MIN_S … 1 マスの最小秒数（発進側の LAUNCH_MIN_S と同じ 0.5 秒。仮値）
#   LAUNCH_ACCEL_SPLIT_KMH … 行の帯の下端がこの車速以上なら実測の点、上端がこの車速以下なら
#     max(クリープ, 本体)（ユーザー決定 2026-10-22 の 2.5 km/h。0.5〜2.5 km/h は手順2 にアクセルの
#     行が無い）
LAUNCH_ACCEL_CELL_PCT = 2.0
LAUNCH_ACCEL_MIN_S = 0.5
LAUNCH_ACCEL_SPLIT_KMH = 2.5


def apply_launch_accel_measured(
    table: ForwardTable,
    rows: FwdRows,
    *,
    cell_pct: float = LAUNCH_ACCEL_CELL_PCT,
    min_s: float = LAUNCH_ACCEL_MIN_S,
    split_kmh: float = LAUNCH_ACCEL_SPLIT_KMH,
) -> ForwardTable:
    """発進側の表のアクセル側（u > 0 の格子）を作り直した新しい表を返す（2026-10-22 段5-3）。

    対象は attach_launch が置き換えた車速の行（meta launch_speed_rows）。launch_accel が無い、
    対象の行が無いときは何も変えない（meta に launch_accel_side_source だけ残す）。
    rows: 手順2 の PATTERN_DRIVE の行（車速・u・加速度・秒数を持つ。u は min からの量）。
    - 帯の下端 ≧ split_kmh の行: 車速が帯の中・u > 0 の行を cell_pct 幅のマスに分け、秒数が min_s
      以上のマスの (u の中央値, 加速度の中央値) を点にする。u = 0 の点はその行の今の値（クリープ）。
      点の間は直線。いちばん深い点より深い所は「いちばん深い点の値 ＋ 本体の表のその行の
      (u) − (u_last) の差」。点が 1 つも無い行は今のまま。
    - 帯の上端 ≦ split_kmh の行: u > 0 の各格子で max(その行の u = 0 の値, 本体の同じ u の値)。
    - 最後にアクセル側だけ u について単調（非減少）にそろえ、ブレーキ側（u ≦ 0）の最大値（u = 0 に
      いちばん近い格子の値）を下回らないようにする。ブレーキ側は変えない。
    meta: launch_accel_side_source（"measured"）・launch_accel_measured（行の代表車速 →
    [[u, 加速度, 秒], …]）・launch_accel_cell_pct・launch_accel_min_s・launch_accel_split_kmh。
    """
    meta = dict(table.meta)
    meta["launch_accel_side_source"] = "measured"
    meta.update(
        launch_accel_measured={},
        launch_accel_cell_pct=cell_pct,
        launch_accel_min_s=min_s,
        launch_accel_split_kmh=split_kmh,
    )
    targets = [float(x) for x in meta.get("launch_speed_rows", [])]
    if table.launch_accel is None or not targets:
        return replace(table, meta=meta)
    edges, grid = table.speed_edges, table.u_grid
    out = table.launch_accel.copy()
    kind = (table.launch_kind if table.launch_kind is not None else table.point_kind).copy()
    pos = grid > _EPS
    neg_idx = np.flatnonzero(grid <= _EPS)
    measured: dict[str, list[list[float]]] = {}
    for r in range(len(table.speed_rep)):
        if not any(abs(float(table.speed_rep[r]) - x) < 1e-6 for x in targets):
            continue
        row = out[r]
        creep = float(np.interp(0.0, grid, row))
        body = table.accel_grid[r]
        lo_edge, hi_edge = float(edges[r]), float(edges[r + 1])
        if hi_edge <= split_kmh + _EPS:
            new = np.maximum(creep, body)
            val = np.where(pos, new, row)
        elif lo_edge >= split_kmh - _EPS:
            sel = (rows.speed_kmh >= lo_edge) & (rows.speed_kmh < hi_edge) & (rows.u_pct > 0.0)
            pts: list[tuple[float, float, float]] = []
            if sel.any():
                u_s, a_s, dt_s = rows.u_pct[sel], rows.accel_kmhs[sel], rows.dt_row_s[sel]
                cells = u_cell(u_s, cell_pct)
                for k in np.unique(cells):
                    m = cells == k
                    sec = float(dt_s[m].sum())
                    if sec >= min_s:
                        pts.append((float(np.median(u_s[m])), float(np.median(a_s[m])), sec))
            measured[f"{table.speed_rep[r]:g}"] = [[u, a, sec] for u, a, sec in pts]
            if not pts:
                continue
            xu = np.array([0.0] + [p[0] for p in pts])
            xa = np.array([creep] + [p[1] for p in pts])
            u_last, a_last = xu[-1], xa[-1]
            b_last = float(np.interp(u_last, grid, body))
            inside = np.interp(grid, xu, xa)
            beyond = a_last + (body - b_last)
            val = np.where(pos, np.where(grid > u_last + _EPS, beyond, inside), row)
        else:
            continue  # 帯が split_kmh をまたぐ行は今のまま
        acc_part = IsotonicRegression(increasing=True).fit_transform(grid[pos], val[pos])
        floor = float(row[neg_idx].max()) if len(neg_idx) else -np.inf
        res = row.copy()
        res[pos] = np.maximum(acc_part, floor)
        out[r] = res
        k_row = kind[r].copy()
        k_row[pos] = POINT_FILLED
        kind[r] = k_row
    meta["launch_accel_measured"] = measured
    return replace(table, launch_accel=out, launch_kind=kind, meta=meta)


def attach_stop(
    table: ForwardTable,
    stop_rows: FwdRows,
    *,
    creep_speed_kmh: float,
    margin_kmh: float = LOW_SPEED_MARGIN_KMH,
    min_s: float = LAUNCH_MIN_S,
    min_total_s: float = LAUNCH_MIN_TOTAL_S,
    accel_window_s: float = math.nan,
) -> ForwardTable:
    """停止側の加速度（stop_accel）を付けた新しい表を返す（2026-10-18 段2o-10）。

    stop_rows: 停止側の行（BRAKE_STEP・BRAKE_SEARCH・低速の停車の手前の行・CREEP_WAIT の u = 0 の
    行を連結したもの）。停車中の行は使わない（停止側は減速している行だけ）。
    作り方・作らない条件は attach_launch と同じ。違いは、深い側の外側を 0 で埋めず、いちばん深い
    実測の値に本体の傾きを足してつなぐこと。meta の stop_max_speed_kmh ＝ 置き換えた車速の行の
    いちばん上の境界（計画が停止側を引く車速の上限）。
    """
    moving_s = (
        float(np.sum(stop_rows.dt_row_s[stop_rows.speed_kmh >= STOP_SPEED_KMH]))
        if len(stop_rows) else 0.0
    )
    meta = dict(table.meta)
    meta.update(
        stop_rows_s=moving_s,
        stop_speed_rows=[],
        stop_min_s=min_s,
        stop_accel_window_s=accel_window_s,
        stop_u_step_pct=SIDE_U_STEP_PCT,
        stop_max_speed_kmh=0.0,
        low_speed_margin_kmh=margin_kmh,
    )
    none = replace(table, stop_accel=None, stop_kind=None, meta=meta)
    if creep_speed_kmh <= 0.0 or moving_s < min_total_s:
        return none
    res = _side_arrays(
        table, stop_rows, creep_speed_kmh=creep_speed_kmh, margin_kmh=margin_kmh, min_s=min_s,
        breakaway_u_pct=None, keep_stationary=False,
    )
    if res is None:
        return none
    out, kind, used, _ = res
    meta["stop_speed_rows"] = [float(table.speed_rep[r]) for r in used]
    meta["stop_max_speed_kmh"] = float(table.speed_edges[max(used) + 1])
    return replace(table, stop_accel=out, stop_kind=kind, meta=meta)


# 2026-10-22 段5-4 b: 発進側の表を停止側と同じ車速の行まで置き換える
# （extend_launch_rows_to_stop_max）。
# 発進側 5 km/h・停止側 7.5 km/h で置き換える範囲が違い、5〜7.5 km/h の開度 0 が発進側（本体）
# −0.44 ↔ 停止側 0.00 と食い違っていた。置き換えていない行は、停止側の u ≦ 0（開度 0 と
# ブレーキ側）をそのまま写し、アクセル側（u > 0）は「停止側の開度 0 の値 ＋ 本体の開度による
# 増え」で作る
# （measured のときはこの後 apply_launch_accel_measured がアクセル側を実測の点で作り直す）。
def extend_launch_rows_to_stop_max(table: ForwardTable) -> ForwardTable:
    """発進側の表（launch_accel）を、停止側が置き換えていて発進側が置き換えていない車速の行まで
    広げた新しい表を返す（2026-10-22 段5-4 b）。

    - u ≦ 0（開度 0 とブレーキ側）: 停止側の表（stop_accel）のその行の値。
    - u > 0（アクセル側）: 停止側の開度 0 の値 ＋ max(本体の同じ u の値 − 本体の開度 0 の値, 0)。
      そのあと u について単調（非減少）にそろえ、ブレーキ側の最大値を下回らないようにする。
    meta: launch_rows_to_stop_max（True）・launch_rows_to_stop_max_rows（広げた行の代表車速）。
    launch_speed_rows に広げた行を足す（昇順）。発進側か停止側の表が無い、広げる行が無いときは
    表はそのまま（meta のフラグだけ残す）。元の表は書き換えない。
    """
    meta = dict(table.meta)
    meta["launch_rows_to_stop_max"] = True
    meta["launch_rows_to_stop_max_rows"] = []
    if table.launch_accel is None or table.stop_accel is None:
        return replace(table, meta=meta)
    have = [float(x) for x in meta.get("launch_speed_rows", [])]
    want = [float(x) for x in meta.get("stop_speed_rows", [])]
    targets = [x for x in want if not any(abs(x - h) < 1e-6 for h in have)]
    if not targets:
        return replace(table, meta=meta)
    grid = table.u_grid
    pos = grid > _EPS
    neg_idx = np.flatnonzero(grid <= _EPS)
    out = table.launch_accel.copy()
    kind = (table.launch_kind if table.launch_kind is not None else table.point_kind).copy()
    stop_kind = table.stop_kind if table.stop_kind is not None else table.point_kind
    done: list[float] = []
    for r in range(len(table.speed_rep)):
        if not any(abs(float(table.speed_rep[r]) - x) < 1e-6 for x in targets):
            continue
        stop_row = table.stop_accel[r]
        body = table.accel_grid[r]
        row = out[r].copy()
        row[neg_idx] = stop_row[neg_idx]
        u0_stop = float(np.interp(0.0, grid, stop_row))
        u0_body = float(np.interp(0.0, grid, body))
        val = u0_stop + np.maximum(body - u0_body, 0.0)
        acc_part = IsotonicRegression(increasing=True).fit_transform(grid[pos], val[pos])
        floor = float(row[neg_idx].max()) if len(neg_idx) else -np.inf
        row[pos] = np.maximum(acc_part, floor)
        out[r] = row
        k_row = kind[r].copy()
        k_row[neg_idx] = stop_kind[r][neg_idx]
        k_row[pos] = POINT_FILLED
        kind[r] = k_row
        done.append(float(table.speed_rep[r]))
    meta["launch_rows_to_stop_max_rows"] = done
    meta["launch_speed_rows"] = sorted(have + done)
    return replace(table, launch_accel=out, launch_kind=kind, meta=meta)


# 2026-10-22 段5-4 e: 発進側・停止側の置き換えた行の代表車速を、学習データの車速の中央値にする
# （apply_side_row_speed_rep）。停止側の 5〜7.5 km/h の行のデータはほとんどちょうど 5.00 km/h、
# 2.5〜5 の行も半分が 4.95〜5.0 km/h なのに、表は行の中心（1.25・3.75・6.25）に置いていた。
#   SIDE_REP_MIN_GAP_KMH … 隣の行の代表車速との間隔の下限 [km/h]。これより近いと車速方向の
#     補間が急になりすぎる（4.95 と 5.00 の間で 1 行ぶん入れ替わる）ので、その行は元の中心に戻す。
#     仮値（データ由来ではない。置き換えた行の幅 2.5 km/h の 1/5）
SIDE_REP_MIN_GAP_KMH = 0.5


def apply_side_row_speed_rep(
    table: ForwardTable,
    launch_rows: FwdRows,
    stop_rows: FwdRows,
    *,
    min_gap_kmh: float = SIDE_REP_MIN_GAP_KMH,
) -> ForwardTable:
    """発進側・停止側の表を補間するときの行の代表車速を、置き換えた行ごとに学習データ（動いている
    行）の車速の中央値にした新しい表を返す（2026-10-22 段5-4 e）。

    - 対象: meta launch_speed_rows（発進側）・stop_speed_rows（停止側）の行。置き換えていない行は
      本体の speed_rep のまま。
    - 中央値は、その車速の行の帯（speed_edges）に入る動いている行（車速 ≧ STOP_SPEED_KMH）の車速。
      発進側で段5-4 b が広げた行（meta launch_rows_to_stop_max_rows）は、値の元が停止側の表なので
      停止側の中央値を使う。動いている行が無い行は元の代表車速のまま。
    - 代表車速は単調増加を保つ: 隣の行（置き換えた後）との間隔が min_gap_kmh 未満になる行は、元の
      代表車速に戻す（下の行から順に確かめる）。
    meta: launch_speed_rep_map / stop_speed_rep_map（行の元の代表車速 → 新しい代表車速）・
    side_row_speed_rep（"data"）・side_row_speed_rep_min_gap_kmh。launch_speed_rep・stop_speed_rep
    を付ける。launch_accel / stop_accel が無い側は付けない。
    """
    meta = dict(table.meta)
    meta["side_row_speed_rep"] = "data"
    meta["side_row_speed_rep_min_gap_kmh"] = min_gap_kmh
    edges = table.speed_edges
    grid_rep = np.asarray(table.speed_rep, dtype=float)

    def medians(rows: FwdRows) -> dict[int, float]:
        mv = rows.speed_kmh >= STOP_SPEED_KMH
        sc = speed_cell_edges(rows.speed_kmh[mv], edges)
        v = rows.speed_kmh[mv]
        return {int(r): float(np.median(v[sc == r])) for r in np.unique(sc)}

    def row_ids(key: str) -> list[int]:
        xs = [float(x) for x in meta.get(key, [])]
        return [r for r in range(len(grid_rep)) if any(abs(grid_rep[r] - x) < 1e-6 for x in xs)]

    def build(ids: list[int], med: dict[int, float]) -> tuple[np.ndarray, dict[str, float]]:
        rep = grid_rep.copy()
        for r in ids:
            if r in med:
                rep[r] = med[r]
        changed = [r for r in ids if r in med]
        for r in changed:
            for q in (r - 1, r + 1):
                if 0 <= q < len(rep) and abs(rep[r] - rep[q]) < min_gap_kmh - _EPS:
                    rep[r] = grid_rep[r]
                    break
        # 戻したあとの確認（単調増加と間隔）。崩れていたら全部元に戻す（安全側）
        if np.any(np.diff(rep) <= 0.0):
            rep = grid_rep.copy()
        return rep, {f"{grid_rep[r]:g}": float(rep[r]) for r in ids}

    stop_ids = row_ids("stop_speed_rows")
    med_stop = medians(stop_rows) if len(stop_rows) else {}
    launch_ids = row_ids("launch_speed_rows")
    ext_ids = set(row_ids("launch_rows_to_stop_max_rows"))
    med_launch = medians(launch_rows) if len(launch_rows) else {}
    for r in ext_ids:
        if r in med_stop:
            med_launch[r] = med_stop[r]
        else:
            med_launch.pop(r, None)
    out = table
    if table.stop_accel is not None and stop_ids:
        rep, mp = build(stop_ids, med_stop)
        meta["stop_speed_rep_map"] = mp
        out = replace(out, stop_speed_rep=rep)
    if table.launch_accel is not None and launch_ids:
        rep, mp = build(launch_ids, med_launch)
        meta["launch_speed_rep_map"] = mp
        out = replace(out, launch_speed_rep=rep)
    return replace(out, meta=meta)


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


# 戻しの表の 7.5〜20 km/h の帯（2026-10-20 段4-3）。下端は停止側の表の車速の上限
# （meta の stop_max_speed_kmh）、上端はこの値。この帯のブレーキの戻しの行が手順2 に少ない
# （段3b）ことを step2 のチェックで秒数として出すだけ。表の作り方には使わない。
# 20 km/h は段3b の分析の区切り（fel.py の「停車前 20 km/h 未満」と同じ）
LOW_RELEASE_BAND_MAX_KMH = 20.0


def release_seconds_in_band(
    rows: FwdRows,
    lo_kmh: float,
    hi_kmh: float = LOW_RELEASE_BAND_MAX_KMH,
    *,
    count_min_pct: float = RELEASE_COUNT_MIN_PCT,
) -> float:
    """車速が lo_kmh 以上 hi_kmh 未満で、ブレーキの戻し量が count_min_pct 以上の行の合計秒数
    （2026-10-20 段4-3。step2 のチェックの表示用）。"""
    m = (
        (rows.u_pct < 0.0)
        & (rows.speed_kmh >= lo_kmh)
        & (rows.speed_kmh < hi_kmh)
        & (rows.release_pct >= count_min_pct - 1e-12)
    )
    return float(np.sum(rows.dt_row_s[m]))


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


def release_bin_edges(release_grid: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    """戻し量の格子の各節について、その節に入れる戻し量の範囲 [下端, 上端)（2026-10-20 段4-3）。
    境は隣の節との中点（先頭の下端は 0 との中点、末尾の上端は無限大）。戻し量 0 の節（本体）は
    先頭の下端より小さい戻し量。"""
    g = np.asarray(release_grid, dtype=float)
    mids = (g[:-1] + g[1:]) / 2.0
    return np.concatenate([[g[0] / 2.0], mids]), np.concatenate([mids, [np.inf]])


def attach_accel_release(
    table: ForwardTable,
    accel_release_accel: np.ndarray,
    *,
    release_pct: np.ndarray,
    dt_row_s: np.ndarray,
    release_grid: Sequence[float] = RELEASE_GRID_PCT,
    min_total_s: float = RELEASE_MIN_TOTAL_S,
) -> ForwardTable:
    """アクセルの戻しの表（accel_release_grid・accel_release_accel）を付けた新しい表を返す
    （2026-10-20 段4-3。attach_release のアクセル版）。

    accel_release_accel: (戻し量の数, 行, 格子点)。単調化済みで、table と同じ格子。
    release_pct・dt_row_s: 学習に使ったアクセルの行（u > 0）のアクセルの戻し量と行ごとの秒数。
    戻し量 RELEASE_COUNT_MIN_PCT 以上の行の合計秒数が min_total_s 未満なら付けない
    （accel_release_grid は None ＝ 今までと完全に同じ。meta に秒数だけ残す）。付けるときは
    meta の accel_release_max_pct ＝ 戻し量の p95 を残す（accel・invert はこれで頭打ちにする）。
    """
    total_s, p95 = release_stats(release_pct, dt_row_s)
    meta = dict(table.meta)
    meta.update(
        accel_release_rows_s=total_s,
        accel_release_max_pct=p95,
        accel_release_min_total_s=min_total_s,
        accel_release_count_min_pct=RELEASE_COUNT_MIN_PCT,
    )
    grid = np.asarray(release_grid, dtype=float)
    acc = np.asarray(accel_release_accel, dtype=float)
    if acc.shape != (len(grid), *table.accel_grid.shape):
        raise ValueError(
            f"accel_release_accel の形 {acc.shape} が"
            f" (戻し量 {len(grid)}, {table.accel_grid.shape}) と違う"
        )
    if total_s < min_total_s:
        return replace(table, accel_release_grid=None, accel_release_accel=None, meta=meta)
    meta["accel_release_grid_pct"] = [float(x) for x in grid]
    return replace(table, accel_release_grid=grid, accel_release_accel=acc, meta=meta)


def attach_stop_release(
    table: ForwardTable,
    brake_rows: FwdRows,
    *,
    creep_speed_kmh: float,
    release_grid: Sequence[float] = RELEASE_GRID_PCT,
    margin_kmh: float = LOW_SPEED_MARGIN_KMH,
    min_s: float = LAUNCH_MIN_S,
) -> ForwardTable:
    """停止側の戻しの表（stop_release_grid・stop_release_accel）を付けた新しい表を返す
    （2026-10-20 段4-3）。stop_accel が無いときは付けない。

    brake_rows: 停止側のブレーキの行（呼ぶ側が集める。手順2 の BRAKE_STEP・BRAKE_SEARCH・
    PATTERN_DRIVE の停車の手前の行と、戻し量を持つ低速のブレーキの行（発進の区間でブレーキを
    離している行を含む））。この中から、動いている（停車でない）・u < 0・車速が
    meta の stop_max_speed_kmh 未満の行だけを使う。
    作り方: 戻し量 r を release_grid の各節の範囲（release_bin_edges）に分け、節ごとに
    停止側の表と同じ `_side_arrays`（マスの中央値・単調化・直線補間）で層を作る。行が足りない
    車速の行は一つ下の層（先頭の層は stop_accel）の値のまま。どの層も作れなければ付けない。
    meta: stop_release_layer_s（節ごとの秒数）・stop_release_layer_built・stop_release_max_pct
    （使った行の戻し量の p95。accel・invert はこれで頭打ちにする）・stop_release_rows_s・
    stop_release_grid_pct。
    """
    grid = np.asarray(release_grid, dtype=float)
    meta = dict(table.meta)
    meta.update(
        stop_release_layer_s=[0.0] * len(grid), stop_release_layer_built=[False] * len(grid)
    )
    none = replace(table, stop_release_grid=None, stop_release_accel=None, meta=meta)
    if table.stop_accel is None or creep_speed_kmh <= 0.0 or len(brake_rows) == 0:
        return none
    top = float(table.meta.get("stop_max_speed_kmh", 0.0))
    sel = (brake_rows.speed_kmh >= STOP_SPEED_KMH) & (brake_rows.u_pct < 0.0)
    sel &= brake_rows.speed_kmh < top
    used = take_rows(brake_rows, sel)
    total_s, p95 = release_stats(used.release_pct, used.dt_row_s)
    meta.update(stop_release_rows_s=total_s, stop_release_max_pct=p95)
    lo, hi = release_bin_edges(grid)
    prev = table.stop_accel
    prev_kind = table.stop_kind if table.stop_kind is not None else table.point_kind
    layers: list[np.ndarray] = []
    seconds: list[float] = []
    built: list[bool] = []
    for k in range(len(grid)):
        m = (used.release_pct >= lo[k]) & (used.release_pct < hi[k])
        part = take_rows(used, m)
        seconds.append(float(np.sum(part.dt_row_s)))
        res = (
            _side_arrays(
                table, part, creep_speed_kmh=creep_speed_kmh, margin_kmh=margin_kmh,
                min_s=min_s, breakaway_u_pct=None, keep_stationary=False,
                base_accel=prev, base_kind=prev_kind,
            )
            if len(part)
            else None
        )
        if res is not None:
            prev, prev_kind = res[0], res[1]
        built.append(res is not None)
        layers.append(prev.copy())
    meta.update(
        stop_release_layer_s=seconds,
        stop_release_layer_built=built,
        stop_release_grid_pct=[float(x) for x in grid],
    )
    if not any(built):
        return replace(table, stop_release_grid=None, stop_release_accel=None, meta=meta)
    return replace(
        table, stop_release_grid=grid, stop_release_accel=np.stack(layers), meta=meta
    )


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
        "stop_accel": table.stop_accel,
        "launch_kind": table.launch_kind,
        "stop_kind": table.stop_kind,
        "launch_speed_rep": table.launch_speed_rep,  # 2026-10-22 段5-4 e
        "stop_speed_rep": table.stop_speed_rep,
        "release_grid": table.release_grid,
        "release_accel": table.release_accel,
        "accel_release_grid": table.accel_release_grid,  # 2026-10-20 段4-3
        "accel_release_accel": table.accel_release_accel,
        "stop_release_grid": table.stop_release_grid,
        "stop_release_accel": table.stop_release_accel,
        "fel_cell_main": table.fel_cell_main,  # 2026-10-19 段2s-2
        "fel_cell_rel": table.fel_cell_rel,
        "fel_nn_main": table.fel_nn_main,
        "fel_nn_rel": table.fel_nn_rel,
        "fel_fit": table.fel_fit,
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
        stop_accel=d.get("stop_accel"),  # 2026-10-18 段2o-10: 古い pkl（キー無し）は None
        launch_kind=d.get("launch_kind"),
        stop_kind=d.get("stop_kind"),
        launch_speed_rep=d.get("launch_speed_rep"),  # 2026-10-22 段5-4 e: 古い pkl は None
        stop_speed_rep=d.get("stop_speed_rep"),
        release_grid=d.get("release_grid"),  # 2026-10-14 段2o-5: 古い pkl（キー無し）は None
        release_accel=d.get("release_accel"),
        accel_release_grid=d.get("accel_release_grid"),  # 2026-10-20 段4-3: 古い pkl は None
        accel_release_accel=d.get("accel_release_accel"),
        stop_release_grid=d.get("stop_release_grid"),
        stop_release_accel=d.get("stop_release_accel"),
        fel_cell_main=d.get("fel_cell_main"),  # 2026-10-19 段2s-2: 古い pkl（キー無し）は None
        fel_cell_rel=d.get("fel_cell_rel"),
        fel_nn_main=d.get("fel_nn_main"),
        fel_nn_rel=d.get("fel_nn_rel"),
        fel_fit=d.get("fel_fit"),
    )
