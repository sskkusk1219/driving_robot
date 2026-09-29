"""研究環境の FF 逆モデル（特徴量・学習の部品・pkl の読み込み）。

`src/domain/model_training.py` と `src/domain/control/feedforward.py` のうち、研究側が使う部分を
`tests/research` に持ち込んだもの（ProblemReport_20260921 手順5-1: tests は tests だけで完結する。
2026-09-24 ユーザー決定）。**`src/` は変更していない**。

本番との違い（新規2項目。既定値は本番の9特徴と完全一致）:
    dv_excluded_horizons_s  先読みホライズンのうち dv 列を特徴量から外すもの。ホライズン自体は
                            `lookahead_horizons_s` に残す（future_speeds[0] を使う停車保持の判定と
                            ブレーキ下限が、これまでと同じ 0.5s 先を見続けるため）
    past_as_delta           False なら過去列を `v0 − past` ではなく `past` そのものにする
                            （手順4 4-3。列名は `past_{h}`）

pkl 形式は本番と同じ（`model_type` も同じ文字列）。本番で作った pkl も、ここで作った pkl も
`FeedforwardModel.load_model` で読める（旧 pkl の feature_spec に新規2項目が無くても既定値で補う）。

特徴量の並び:
    [v0, dv_h(除外されていないもの), v0_sq, dv1_x_v0, dv_past_h または past_h, accel_h]

ペダル別ホライズン（ProblemReport_20260921 手順6。2026-09-28）:
    `FeedforwardModel` はアクセル・ブレーキで**別々の** `FeatureSpec`（`accel_spec` / `brake_spec`）
    を持てる。`spec`（pkl の `feature_spec`）は両者の**和集合**（`horizons`/`past_horizons` が
    返す並び。mode_drive はこの並びで future/past を組み立てる）。`predict_effort` は和集合から
    ペダルごとの行を組み立てて、それぞれの `accel_model`/`brake_model` に渡す。
    `stop_horizon_s`（既定は spec の先頭＝旧来の `future_speeds[0]`）は停車保持の判定・
    ブレーキ下限の `ref_next` が見るホライズンで、ペダル別の spec がそのホライズンを
    dv 列に含むかどうかとは独立（`horizon_search.py` が選ばない固定値 h0）。
    旧 pkl（`accel_feature_spec`/`brake_feature_spec` が無い）は両ペダルとも `feature_spec` を使う
    （完全後方互換）。
"""

from __future__ import annotations

import pickle
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Literal

import numpy as np
import yaml
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.pipeline import Pipeline, make_pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler

from tests.research.research_types import DriveLog, FeedforwardParams, coast_decel_at

__all__ = [
    "DEFAULT_FEATURE_SPEC",
    "MODEL_TYPE",
    "FeatureSpec",
    "FeedforwardModel",
    "build_feature_matrix",
    "build_feature_row",
    "deviation_gain",
    "estimate_offsets",
    "export_model_coefficients",
    "group_by_session",
    "make_estimator",
    "raw_unit_coefficients",
    "metrics",
    "pkl_is_pedal_separated",
    "require_single_spec_pkl",
]

# 本番と同じ識別子（本番の pkl も読めるようにするため変えない）
MODEL_TYPE: str = "poly_spec_inverse_lookahead"
POLY_DEGREE: int = 2
RIDGE_ALPHA: float = 1.0
DEFAULT_DT_S: float = 0.1  # ログ周期が推定できない場合のフォールバック (100ms)
MIN_SAMPLES_FOR_TRAINING: int = 20  # 全レジーム合計の最小サンプル数
MIN_REGIME_SAMPLES: int = 8  # 各モデル（アクセル/ブレーキ）の最小サンプル数
STOP_SPEED_KMH: float = 0.02  # これ以下を「停車」とみなす（src.domain.control.conversions と同値）


@dataclass(frozen=True)
class FeatureSpec:
    """先読み逆モデルの特徴量構成。既定値は本番の9特徴と完全一致する。

    Attributes:
        lookahead_horizons_s: 先読みホライズン [s]。昇順（狭義単調増加）。停車保持の判定が
            `future_speeds[0]` を最短ホライズンとして参照する。
        past_horizons_s: 過去方向ホライズン [s]。
        regime_horizon_s: アクセル/ブレーキのレジーム判定に使うホライズン（先読みに含まれること）。
        include_v0_sq: 二次項 v0² を含めるか。
        include_dv_regime_x_v0: 交互作用項 dv_{regime}·v0 を含めるか。
        accel_horizons_s: 加速度項を追加するホライズン集合（先読みと過去の両方に含まれること）。
        dv_excluded_horizons_s: dv 列を特徴量から外す先読みホライズン（先読みに含まれること。
            `regime_horizon_s` と `accel_horizons_s` は外せない）。
        past_as_delta: True で `v0 − past`（`dv_past_h`）、False で `past` そのもの（`past_h`）。
    """

    lookahead_horizons_s: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0)
    past_horizons_s: tuple[float, ...] = (0.5, 1.0)
    regime_horizon_s: float = 1.0
    include_v0_sq: bool = True
    include_dv_regime_x_v0: bool = True
    accel_horizons_s: tuple[float, ...] = ()
    dv_excluded_horizons_s: tuple[float, ...] = ()
    past_as_delta: bool = True

    def __post_init__(self) -> None:
        horizons = self.lookahead_horizons_s
        if len(horizons) == 0:
            raise ValueError("lookahead_horizons_s は少なくとも1つ指定してください")
        if any(a >= b for a, b in zip(horizons, horizons[1:], strict=False)):
            raise ValueError("lookahead_horizons_s は昇順（狭義単調増加）である必要があります")
        if self.regime_horizon_s not in horizons:
            raise ValueError(
                f"regime_horizon_s={self.regime_horizon_s} は "
                f"lookahead_horizons_s に含まれる必要があります"
            )
        for h in self.accel_horizons_s:
            if h not in horizons or h not in self.past_horizons_s:
                raise ValueError(
                    f"accel_horizons_s の {h} は lookahead_horizons_s と "
                    f"past_horizons_s の両方に含まれる必要があります"
                )
        for h in self.dv_excluded_horizons_s:
            if h not in horizons:
                raise ValueError(
                    f"dv_excluded_horizons_s の {h} は "
                    f"lookahead_horizons_s に含まれる必要があります"
                )
            if h == self.regime_horizon_s:
                raise ValueError(
                    f"dv_excluded_horizons_s に regime_horizon_s={h} は含められません"
                    f"（レジーム判定に使う）"
                )
            if h in self.accel_horizons_s:
                raise ValueError(
                    f"dv_excluded_horizons_s に accel_horizons_s の {h} は含められません"
                )

    def dv_horizons_s(self) -> tuple[float, ...]:
        """dv 列として特徴量に入れる先読みホライズン（除外を除いたもの）。"""
        return tuple(h for h in self.lookahead_horizons_s if h not in self.dv_excluded_horizons_s)

    def feature_names(self) -> list[str]:
        """特徴量ベクトルの列順に対応する名前一覧を返す。"""
        names = ["v0", *[f"dv_{h}" for h in self.dv_horizons_s()]]
        if self.include_v0_sq:
            names.append("v0_sq")
        if self.include_dv_regime_x_v0:
            names.append("dv1_x_v0")
        prefix = "dv_past_" if self.past_as_delta else "past_"
        names.extend(f"{prefix}{h}" for h in self.past_horizons_s)
        names.extend(f"accel_{h}" for h in self.accel_horizons_s)
        return names

    def regime_col(self) -> int:
        """特徴行列内の dv_{regime_horizon_s} の列番号（0始まり）を返す。"""
        return 1 + self.dv_horizons_s().index(self.regime_horizon_s)


DEFAULT_FEATURE_SPEC = FeatureSpec()


def make_estimator() -> Pipeline:
    """逆モデルの推定器: 完全2次多項式展開 → 標準化 → Ridge。"""
    return make_pipeline(
        PolynomialFeatures(POLY_DEGREE, include_bias=False),
        StandardScaler(),
        Ridge(alpha=RIDGE_ALPHA),
    )


def raw_unit_coefficients(
    model: Pipeline, input_names: Sequence[str]
) -> tuple[list[str], list[float], float]:
    """2次多項式→標準化→Ridge の係数を、特徴量の元の単位の係数に戻す（参照用）。

    標準化は `z_j = (x_j - mean_j) / scale_j` なので、`u = Σ coef_j·z_j + b` は
    `u = Σ (coef_j/scale_j)·x_j + (b - Σ coef_j·mean_j/scale_j)` と書き直せる。
    ここで x_j は多項式展開後の項（`v0^2`・`v0 dv_1.0` など）。予測は変わらない
    （`model.predict` と一致する）。項の値の大きさが違うので、係数の大小だけでは効きの大きさを
    比べられない。

    Args:
        model: `make_estimator` が作ったパイプライン（学習済み）
        input_names: 展開前の特徴量名（`FeatureSpec.feature_names()`。列順どおり）

    Returns:
        (項名のリスト, 各項の係数, 切片)。項の並びは多項式展開の列順
    """
    poly, scaler, ridge = (step for _, step in model.steps)
    if len(input_names) != poly.n_features_in_:
        raise ValueError(
            f"特徴量名の数({len(input_names)})とモデルの入力数({poly.n_features_in_})が違います"
        )
    terms = [str(t) for t in poly.get_feature_names_out(list(input_names))]
    coef = np.asarray(ridge.coef_, dtype=float).ravel() / scaler.scale_
    intercept = float(np.ravel(ridge.intercept_)[0] - np.sum(coef * scaler.mean_))
    return terms, [float(c) for c in coef], intercept


def export_model_coefficients(model_path: str | Path) -> Path:
    """pkl の隣に、ペダル別のホライズン・特徴量・元の単位の係数を `<pkl名>.yaml` で書く。

    参照用（走行は今までどおり pkl を読む。この yaml は読まない）。手順2・`relearn` が pkl を
    保存した直後に呼ぶ。

    Returns:
        書いた yaml のパス
    """
    path = Path(model_path)
    with path.open("rb") as f:
        data: dict[str, Any] = pickle.load(f)  # noqa: S301 - 直前に自分で保存した pkl
    out: dict[str, Any] = {
        "model_pkl": path.name,
        "trained_at": data.get("trained_at"),
        "note": (
            "参照用。走行時は pkl を読み、この yaml は読まない。"
            "係数は特徴量の元の単位（km/h など）に戻した値: "
            "開度[%] = intercept + Σ coefficients[項] × 項の値。"
            "項名の空白は積（v0 dv_1.0 = v0×dv_1.0）"
        ),
    }
    for side in ("accel", "brake"):
        spec_dict = data.get(f"{side}_feature_spec", data.get("feature_spec"))
        spec = FeatureSpec(**spec_dict) if spec_dict else DEFAULT_FEATURE_SPEC
        names = spec.feature_names()
        terms, coef, intercept = raw_unit_coefficients(data[f"{side}_model"], names)
        out[side] = {
            "horizons_s": [float(h) for h in spec.lookahead_horizons_s],
            "past_horizons_s": [float(h) for h in spec.past_horizons_s],
            "features": names,
            "intercept": intercept,
            "coefficients": dict(zip(terms, coef, strict=True)),
        }
    yaml_path = path.with_suffix(".yaml")
    yaml_path.write_text(
        yaml.safe_dump(out, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    return yaml_path


def build_feature_row(
    v0: float,
    future_speeds: Sequence[float],
    past_speeds: Sequence[float],
    spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
) -> np.ndarray:
    """1 サンプル分の特徴量ベクトル (1, n_features) を返す（推論用）。

    Args:
        v0: 現在の速度 [km/h]
        future_speeds: 各ホライズン（`spec.lookahead_horizons_s` 順、除外分も含む）の速度 [km/h]
        past_speeds: 各過去ホライズン（`spec.past_horizons_s` 順）の速度 [km/h]
        spec: 特徴量構成。

    Raises:
        ValueError: future_speeds / past_speeds の長さがホライズン数と一致しない場合
    """
    if len(future_speeds) != len(spec.lookahead_horizons_s):
        raise ValueError(
            f"future_speeds の長さ {len(future_speeds)} は "
            f"ホライズン数 {len(spec.lookahead_horizons_s)} と一致する必要があります"
        )
    if len(past_speeds) != len(spec.past_horizons_s):
        raise ValueError(
            f"past_speeds の長さ {len(past_speeds)} は "
            f"過去ホライズン数 {len(spec.past_horizons_s)} と一致する必要があります"
        )
    dv_all = dict(zip(spec.lookahead_horizons_s, (fs - v0 for fs in future_speeds), strict=True))
    dv_regime = dv_all[spec.regime_horizon_s]
    past_cols = [v0 - ps for ps in past_speeds] if spec.past_as_delta else list(past_speeds)

    row: list[float] = [v0, *[dv_all[h] for h in spec.dv_horizons_s()]]
    if spec.include_v0_sq:
        row.append(v0 * v0)
    if spec.include_dv_regime_x_v0:
        row.append(dv_regime * v0)
    row.extend(past_cols)
    for h in spec.accel_horizons_s:
        f_val = future_speeds[spec.lookahead_horizons_s.index(h)]
        p_val = past_speeds[spec.past_horizons_s.index(h)]
        row.append((f_val - 2.0 * v0 + p_val) / (h * h))
    return np.array([row], dtype=float)


def build_feature_matrix(
    speed: np.ndarray,
    offsets: list[int],
    past_offsets: list[int],
    spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
) -> tuple[np.ndarray, np.ndarray]:
    """速度系列から先読み特徴量行列と有効サンプル indices を返す（学習用）。

    末尾 max(offsets) サンプルは未来データが、先頭 max(past_offsets) サンプルは
    過去データが不足するため除外する。
    """
    max_off = max(offsets, default=0)
    lead = max(past_offsets, default=0)
    n_valid = len(speed) - max_off - lead
    if n_valid <= 0:
        return np.empty((0, len(spec.feature_names()))), np.empty(0, dtype=int)

    idx = np.arange(lead, lead + n_valid)
    v0 = speed[idx]
    dv_all = {
        h: speed[idx + off] - v0
        for h, off in zip(spec.lookahead_horizons_s, offsets, strict=True)
    }
    dv_regime = dv_all[spec.regime_horizon_s]
    if spec.past_as_delta:
        past_cols = [v0 - speed[idx - off] for off in past_offsets]
    else:
        past_cols = [speed[idx - off] for off in past_offsets]

    cols: list[np.ndarray] = [v0, *[dv_all[h] for h in spec.dv_horizons_s()]]
    if spec.include_v0_sq:
        cols.append(v0 * v0)
    if spec.include_dv_regime_x_v0:
        cols.append(dv_regime * v0)
    cols.extend(past_cols)
    for h in spec.accel_horizons_s:
        off = offsets[spec.lookahead_horizons_s.index(h)]
        past_off = past_offsets[spec.past_horizons_s.index(h)]
        cols.append((speed[idx + off] - 2.0 * v0 + speed[idx - past_off]) / (h * h))

    return np.column_stack(cols), idx


def deviation_gain(
    model: Any,  # noqa: ANN401
    spec: FeatureSpec,
    v0: float,
    *,
    pedal: Literal["accel", "brake"],
    eps: float = 0.25,
) -> float:
    """実車速が基準からずれたとき、開度が正しい向きへ動くかを符号つきで返す [%/(km/h)]。

    手順6 段2（ProblemReport_20260921。2026-09-28）: 実車速が基準より遅れているのにアクセル
    FF が下がる（逆向き）pkl が実機で採用され、遅れが自分で広がって最大逸脱 126km/h に至った。
    学習データ（偏差 0 の自分の軌跡）だけでは、C5 の中に隠れた比例フィードバックの符号は
    決まらない（`model_gain` モジュールの docstring 参照）ため、この関数で符号つきに測って
    合否判定できるようにする。

    `model_gain.level_gain` と同じ中心差分（基準車速は v0 に等しい定常点、過去も同じ値に置き
    dv_past = 0 とする）だが、絶対値にせず符号を残し、ペダルごとに「正しい向きなら正」になる
    よう揃える:
        raw = ∂u/∂v0（実車速 v0 が上がったとき開度 u がどう動くか。中心差分）
        accel: 実車速が基準より遅れる（v0 が下がる）と開度が増えるのが正しい → 戻り値は `-raw`
        brake: 実車速が基準より速い（v0 が上がる）と開度が増えるのが正しい → 戻り値は `raw`

    Args:
        model: `.predict(X)` を持つ学習済みモデル（sklearn Pipeline など）。
        spec: そのモデルの特徴量構成（`pedal` 側の spec を渡すこと）。
        v0: 測る速度 [km/h]。
        pedal: "accel" または "brake"（符号の揃え方が変わる）。
        eps: 中心差分の刻み幅 [km/h]。

    Returns:
        実質Kp [%/(km/h)]。正なら正しい向き、負なら逆向き（ずれが自分で広がる）。
    """
    n_future = len(spec.lookahead_horizons_s)
    n_past = len(spec.past_horizons_s)
    future = [v0] * n_future
    row_plus = build_feature_row(v0 + eps, future, [v0 + eps] * n_past, spec)
    row_minus = build_feature_row(v0 - eps, future, [v0 - eps] * n_past, spec)
    preds = np.asarray(model.predict(np.vstack([row_plus, row_minus])), dtype=float).reshape(-1)
    raw = float((preds[0] - preds[1]) / (2.0 * eps))
    return -raw if pedal == "accel" else raw


def estimate_offsets(timestamps: list[datetime], horizons: Sequence[float]) -> list[int]:
    """timestamp 列から周期 dt を推定し、各ホライズンのサンプルオフセットを返す。"""
    if len(timestamps) >= 2:
        epochs = np.array([ts.timestamp() for ts in timestamps])
        diffs = np.diff(epochs)
        diffs = diffs[diffs > 0.0]
        dt = float(np.median(diffs)) if len(diffs) > 0 else DEFAULT_DT_S
    else:
        dt = DEFAULT_DT_S
    if dt <= 0.0:
        dt = DEFAULT_DT_S
    return [max(1, round(h / dt)) for h in horizons]


def group_by_session(logs: list[DriveLog]) -> list[list[DriveLog]]:
    """ログを session_id でグループ化し、各グループを timestamp 昇順で返す。

    先読み特徴量がセッション境界をまたがないようにするため必須。
    """
    grouped: dict[str, list[DriveLog]] = {}
    for log in logs:
        grouped.setdefault(log.session_id, []).append(log)
    return [sorted(g, key=lambda x: x.timestamp) for g in grouped.values()]


def metrics(model: Any, x: np.ndarray, y: np.ndarray) -> dict[str, float]:  # noqa: ANN401
    """学習データに対する評価指標（in-sample）を返す。"""
    pred = model.predict(x)
    result: dict[str, float] = {
        "mae": float(mean_absolute_error(y, pred)),
        "rmse": float(mean_squared_error(y, pred) ** 0.5),
        "n": float(len(y)),
    }
    if len(y) >= 2 and float(np.var(y)) > 1e-9:
        result["r2"] = float(r2_score(y, pred))
    return result


def pkl_is_pedal_separated(data: dict[str, Any]) -> bool:
    """pkl の生 dict が、アクセル・ブレーキで異なる特徴量構成を持つか（手順6）。

    `FeedforwardModel.load_model` を経由しない、pkl を直接読む解析ツール
    （`model_gain.py`・`model_analysis.py`・`ff_explain.py` 等）が使う。
    """
    accel = data.get("accel_feature_spec", data.get("feature_spec"))
    brake = data.get("brake_feature_spec", data.get("feature_spec"))
    return accel != brake


def require_single_spec_pkl(data: dict[str, Any], tool_name: str, path: str = "") -> None:
    """ペダル別ホライズンの pkl（手順6）なら、未対応のツールから分かりやすく拒否する。

    Raises:
        ValueError: アクセル・ブレーキで特徴量構成が異なる pkl だった場合
    """
    if pkl_is_pedal_separated(data):
        where = f"（{path}）" if path else ""
        raise ValueError(
            f"{tool_name} はペダル別ホライズンの pkl{where}に未対応です"
            "（アクセル/ブレーキで feature_spec が異なります。"
            "ProblemReport_20260921 手順6 の対応範囲外）"
        )


class FeedforwardModel:
    """pkl から逆モデルをロードして保持する（研究側の FF 候補の基底クラス）。

    本番 `FeedforwardController` のうち、モデルの保持・ロード・ホライズンの参照・物理定数の設定
    だけを持つ。推論（`predict_effort`）は `ff_candidate.CandidateFeedforward` が実装する。
    """

    def __init__(self) -> None:
        self._accel_model: Any | None = None
        self._brake_model: Any | None = None
        # 推論時に速度をこの上限へクリップして学習域外の外挿を防ぐ。None は無制限。
        self._speed_clip_max: float | None = None
        self._params: FeedforwardParams = FeedforwardParams()
        # load_model でロード済み pkl の feature_spec に置き換わる（和集合＝制御用）
        self._spec: FeatureSpec = DEFAULT_FEATURE_SPEC
        # ペダル別 spec（手順6）。旧 pkl・未ロード時は _spec と同じ（完全後方互換）
        self._accel_spec: FeatureSpec = DEFAULT_FEATURE_SPEC
        self._brake_spec: FeatureSpec = DEFAULT_FEATURE_SPEC
        # 停車保持判定・ブレーキ下限の ref_next が見るホライズン（既定は spec の先頭）
        self._stop_horizon_s: float = DEFAULT_FEATURE_SPEC.lookahead_horizons_s[0]

    @property
    def spec(self) -> FeatureSpec:
        """ロード済みモデルの特徴量構成（未ロード時は既定）。アクセル・ブレーキの和集合（制御用）。"""
        return self._spec

    @property
    def accel_spec(self) -> FeatureSpec:
        """アクセル側の特徴量構成（手順6。旧 pkl・未ロード時は `spec` と同じ）。"""
        return self._accel_spec

    @property
    def brake_spec(self) -> FeatureSpec:
        """ブレーキ側の特徴量構成（手順6。旧 pkl・未ロード時は `spec` と同じ）。"""
        return self._brake_spec

    @property
    def is_pedal_separated(self) -> bool:
        """アクセル・ブレーキが異なる特徴量構成を持つか（手順6 のペダル別ホライズン pkl）。"""
        return self._accel_spec != self._brake_spec

    @property
    def stop_horizon_s(self) -> float:
        """停車保持判定・ブレーキ下限 `ref_next` が見るホライズン（手順6。既定は spec の先頭）。"""
        return self._stop_horizon_s

    @property
    def horizons(self) -> tuple[float, ...]:
        """先読みホライズン [s]（呼び出し元が future_speeds を組むために参照する）。"""
        return self._spec.lookahead_horizons_s

    @property
    def past_horizons(self) -> tuple[float, ...]:
        """過去方向ホライズン [s]（呼び出し元が past_speeds を組むために参照する）。"""
        return self._spec.past_horizons_s

    @property
    def has_model(self) -> bool:
        return self._accel_model is not None and self._brake_model is not None

    def set_params(self, params: FeedforwardParams) -> None:
        """車両プロファイルのフィードフォワード物理定数を設定する。"""
        self._params = params

    def load_model(self, model_path: str) -> None:
        """pkl ファイルから逆モデルをロードする（開発者がローカルで作った信頼済みファイルのみ）。

        必須キー: model_type / accel_model / brake_model / feature_spec。
        feature_spec に新規2項目（dv_excluded_horizons_s・past_as_delta）が無い旧 pkl は
        既定値で補う。

        ペダル別ホライズン（手順6）: `accel_feature_spec`/`brake_feature_spec` があれば
        それぞれ読む。無い旧 pkl は両ペダルとも `feature_spec`（完全後方互換）。
        `stop_horizon_s` が無ければ `feature_spec` の先頭ホライズン（旧来の `future_speeds[0]`）。
        """
        path = Path(model_path)
        if not path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")

        with path.open("rb") as f:
            data: dict[str, Any] = pickle.load(f)  # noqa: S301

        if not isinstance(data, dict) or data.get("model_type") != MODEL_TYPE:
            raise ValueError(
                f"Unsupported model file (expected model_type={MODEL_TYPE!r}): {model_path}"
            )
        missing = {"accel_model", "brake_model", "feature_spec"} - data.keys()
        if missing:
            raise ValueError(f"Model file is missing required keys: {missing}")

        try:
            spec = FeatureSpec(**data["feature_spec"])
            accel_spec = (
                FeatureSpec(**data["accel_feature_spec"])
                if "accel_feature_spec" in data
                else spec
            )
            brake_spec = (
                FeatureSpec(**data["brake_feature_spec"])
                if "brake_feature_spec" in data
                else spec
            )
        except TypeError as e:
            raise ValueError(f"Model file has an invalid feature_spec: {model_path}") from e
        if accel_spec.regime_horizon_s != brake_spec.regime_horizon_s:
            raise ValueError(
                f"Model file has mismatched regime_horizon_s between accel/brake specs: "
                f"{model_path}"
            )
        stop_horizon_s = float(data.get("stop_horizon_s", spec.lookahead_horizons_s[0]))
        if stop_horizon_s not in spec.lookahead_horizons_s:
            raise ValueError(
                f"Model file's stop_horizon_s={stop_horizon_s} is not in "
                f"feature_spec.lookahead_horizons_s: {model_path}"
            )

        self._accel_model = data["accel_model"]
        self._brake_model = data["brake_model"]
        self._spec = spec
        self._accel_spec = accel_spec
        self._brake_spec = brake_spec
        self._stop_horizon_s = stop_horizon_s
        clip = data.get("speed_clip_max")
        self._speed_clip_max = float(clip) if clip is not None else None

    def predict_effort(
        self, v0: float, future_speeds: Sequence[float], past_speeds: Sequence[float]
    ) -> float:
        """現在・先読み・過去の基準速度から符号付き努力量 [%] を返す。

        本番 `FeedforwardController.predict_effort`（src/domain/control/feedforward.py）の移植。
        研究側の候補（`ff_candidate.CandidateFeedforward`）は合成を差し替えて上書きするが、
        `ff_explain` が「写した分岐が本番と一致するか」を確かめる照合元はこちら。

        Args:
            v0: 現在の基準速度 [km/h]
            future_speeds: 各ホライズン（horizons 順）の基準速度 [km/h]
            past_speeds: 各過去ホライズン（past_horizons 順）の基準速度 [km/h]。
                過去Δv（ランプ過渡か定常保持か）の識別に使う。

        Returns:
            努力量 [%]。正は名目アクセル開度、負は名目ブレーキ開度。-100.0〜100.0。

        Raises:
            RuntimeError: load_model() が呼ばれていない場合
        """
        if self._accel_model is None or self._brake_model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        p = self._params
        spec = self._spec  # 制御用（アクセル・ブレーキの和集合）

        # 1. 停車レジーム: 停車保持ブレーキ（学習モデルは原点で 0 を保証しないため定数で補う。
        #    負予測は後段で 0 にクランプされる）。判定は `stop_horizon_s`（既定 0.5 秒先）のみ:
        #    全先読み点（3 秒先まで）で判定すると発進の 3 秒前に保持ブレーキが解除され、
        #    クリープで基準 0 に逆らって動き出す（レビュー指摘 #5）。手順6: アクセル・ブレーキが
        #    別々のホライズンを選んでも、この判定は spec（和集合）の中の固定ホライズンを見続ける
        #    （future_speeds は spec.lookahead_horizons_s と同じ並びで渡される）。
        future_map_raw = dict(zip(spec.lookahead_horizons_s, future_speeds, strict=True))
        stop_future = future_map_raw.get(self._stop_horizon_s)
        if v0 <= STOP_SPEED_KMH and stop_future is not None and stop_future <= STOP_SPEED_KMH:
            return -max(0.0, min(100.0, p.stop_brake_opening_pct))

        # 学習域外の外挿を防ぐため速度を観測最高車速 cm にクリップ（木の域外飽和も含めて
        # 学習端で有界にする。残差は PID と包絡線ガバナが吸収する）。v0>cm のときは軌跡全体を平行
        # 移動して v0 を cm に置き、減速/加速の相対トレンド（dv＝レジーム判定の基）を保つ。単純に
        # 各点を独立クリップすると near-horizon の dv が 0 に潰れてレジームを誤判定するため。
        cm = self._speed_clip_max
        if cm is not None:
            if v0 > cm:
                shift = v0 - cm
                v0 = cm
                future_speeds = [f - shift for f in future_speeds]
                past_speeds = [p - shift for p in past_speeds]
            # 学習域を超える分（域外の速度点）は学習端に飽和させる
            future_speeds = [min(f, cm) for f in future_speeds]
            past_speeds = [min(p, cm) for p in past_speeds]

        # 手順6: future_speeds/past_speeds は spec（和集合）の並びで渡される。ホライズン値で
        # 引ける辞書にしてから、ペダルごとの spec に必要な列だけを取り出す。
        future_map = dict(zip(spec.lookahead_horizons_s, future_speeds, strict=True))
        past_map = dict(zip(spec.past_horizons_s, past_speeds, strict=True))
        # レジーム判定: 先読みトレンド dv_{regime_horizon_s} の符号（両ペダルの spec で共通）
        dv_regime = future_map[spec.regime_horizon_s] - v0
        desired_accel = dv_regime / spec.regime_horizon_s  # km/h/s（正:加速, 負:減速）

        # 2. 両モデルを評価（ペダルごとに自分の spec で特徴量を組む）。負の予測はノイズとして 0。
        accel_row = build_feature_row(
            v0,
            [future_map[h] for h in self._accel_spec.lookahead_horizons_s],
            [past_map[h] for h in self._accel_spec.past_horizons_s],
            self._accel_spec,
        )
        brake_row = build_feature_row(
            v0,
            [future_map[h] for h in self._brake_spec.lookahead_horizons_s],
            [past_map[h] for h in self._brake_spec.past_horizons_s],
            self._brake_spec,
        )
        accel_pred = max(0.0, float(self._accel_model.predict(accel_row)[0]))
        brake_pred = max(0.0, float(self._brake_model.predict(brake_row)[0]))

        # 3. レジーム合成。旧実装の「dv>=0 → アクセルモデル / dv<0 → ブレーキ 0 or
        #    ブレーキモデル」は dv=0 境界で巡航開度 → 0 の不連続ジャンプを生み、巡航
        #    うねりで FF がドロップアウトして PID が穴埋めを繰り返していた（指摘 #9）。
        #    エンジンブレーキで届く緩減速は「スロットルを巡航開度から漸減して作る」のが
        #    物理実体のため、テーパで連続化する。
        if desired_accel >= 0.0:
            # 加速・定常: 駆動側。低速でクリープ能力内の加速要求はペダル不要。
            # 旧実装の abs() 判定は低速の「緩減速」までペダルオフにし、クリープが車を
            # 押す方向と逆に誤差が成長していたため、[0, creep_rate] に限定する（指摘 #8）。
            if v0 < p.creep_speed_kmh and desired_accel <= p.creep_rate_kmhs:
                effort = 0.0
            else:
                effort = accel_pred
        elif v0 >= p.creep_speed_kmh:
            # 惰行減速の基準は速度依存カーブ（同定済みなら補間、未同定は定数）。単一定数だと
            # 実惰行が強い速度域の緩減速がブレーキモデルへ誤送され、フェーズ分類（同じ基準線）
            # と併せて減速区間の effort 符号が真逆になる（sample_004 実機 p95=4.05 の主因）。
            eng = coast_decel_at(p, v0)
            if eng > 0.0 and (-desired_accel) <= eng:
                # 惰行で届く緩減速: スロットルテーパ（dv=0 で accel_pred、-eng で 0）
                effort = accel_pred * (1.0 - (-desired_accel) / eng)
            else:
                effort = -brake_pred
        else:
            # クリープ速度未満の減速要求: ペダルオフでは加速する領域のため
            # エンジンブレーキ則を適用せずブレーキを残す（指摘 #8）。
            effort = -brake_pred

        # 不感帯処理は FF 単体では行わない: FF+PID 合成後の最終指令に対して
        # PedalArbiter が逆補償する（FF 側で切り捨てると合成値が死帯に落ちる。指摘 #11）。
        return max(-100.0, min(100.0, effort))
