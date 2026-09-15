"""改善案 C1: 手順 2 の学習セット選別（A1）と、手順 3 の FF レジーム合成（B1〜B3）。

`tests/research/results/report20260912_KAIZEN_process2,3.md` 5 章の提案のうち、走行が要らない
◎ の 4 項目（A1・B1・B2・B3）を研究環境だけで実装したもの。**`src/` は変更していない**
（ProblemReport_20260910 遵守事項「実行環境はすべて tests/ に記載する」）。本番の自動走行・
学習運転は今まで通り `src/` の実装で動く。

引用元（アルゴリズムの出どころ）:
    src/domain/model_training.py        … train_inverse_model の特徴量・推定器・pkl 形式
    src/domain/control/feedforward.py   … predict_effort の停車保持・学習域クリップ・推論
    tests/research/kaizen.py            … C1 のオフライン版（train_models・decide_openings）

変更点（現行 C0 との差）:
    A1 学習行の選別   そのペダルが効いている行（開度 ≥ 不感帯）だけで各モデルを学習する。
                      惰行の行はどちらのモデルにも入れない（ラベル 0 を作らない）。
    B1 ペダルの選択   要求加速度 ≥ 惰行の加速度 ならアクセル、下ならブレーキ
                      （現行は dv_1.0 の符号）。
    B2 惰行テーパ廃止 `accel_pred × (1 − (−a)/eng)` を削除する。
    B3 不感帯切り上げ 選んだペダルの開度を max(不感帯, 予測) にする（足すのは FF の 1 か所だけ。
                      `arbiter.enable_deadband_compensation` は false のまま）。

C6（2026-09-15 追加。docs/memo.md「段4-2 関門確認 → 骨格を実測テーブルに変更」）:
    アクセル側だけ、骨格（定速階段の実測テーブル `cruise_curve.CruiseCurve` + 要求加速度 ÷
    ペダルゲイン）と残差 ML モデルの和にする。C1 との差はアクセルモデルの目的変数だけ（ラベル
    − 骨格）で、特徴量・推定器・B1〜B3・停車保持・クリープ・学習域クリップは C1 と同じ
    （1 変数比較を保つ、というユーザー決定）。ブレーキ側は C1 と同一。
"""

from __future__ import annotations

import pickle
from collections.abc import Sequence
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pedal_plan import pedal_gain_at
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import (
    DEFAULT_FEATURE_SPEC,
    MIN_REGIME_SAMPLES,
    MIN_SAMPLES_FOR_TRAINING,
    MODEL_TYPE,
    STOP_SPEED_KMH,
    FeatureSpec,
    _build_feature_matrix,  # noqa: PLC2701 - 学習の特徴量は本番と同一にする
    _estimate_offsets,  # noqa: PLC2701
    _group_by_session,  # noqa: PLC2701
    _make_estimator,  # noqa: PLC2701
    _metrics,  # noqa: PLC2701
    build_feature_row,
)
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams, VehicleProfile, coast_decel_at
from tests.research.cruise_curve import CruiseCurve

__all__ = [
    "CandidateC2",
    "CandidateC3",
    "CandidateC4",
    "CandidateC5",
    "CandidateC6",
    "CandidateFeedforward",
    "CANDIDATE_CLASSES",
    "cruise_skeleton",
    "make_candidate",
    "train_inverse_model_effective",
]

# pkl に残す学習セットの作り方（後から取り違えないため。load_model は無視する追加キー）
TRAINING_ROWS_EFFECTIVE: str = "effective_only"


# ─────────────────────────────────────────────────────────────────────
# C6: 骨格（定速階段の実測テーブル + 要求加速度 ÷ ペダルゲイン）
# ─────────────────────────────────────────────────────────────────────


def cruise_skeleton(
    curve: CruiseCurve, params: FeedforwardParams, v0: float, a_req: float
) -> float:
    """C6 のアクセル骨格開度 [%]（学習・推論の両方から呼び、定義の食い違いを防ぐ）。

    骨格 = curve.opening_at(v0, floor=アクセル不感帯) + a_req ÷ k(v0)。
    k はアクセル側ペダルゲイン（`pedal_gain_at`、[km/h/s per %]）。未同定（None）・0 以下なら
    a_req 項は 0（惰行の骨格だけを返す）。
    """
    base = float(curve.opening_at(v0, params.accel_deadband_pct))
    k = pedal_gain_at(params, v0, is_accel=True)
    term = 0.0 if k is None or k <= 0.0 else a_req / k
    return base + term


class _PrecomputedPredictor:
    """既に計算済みの予測配列をそのまま返す（`_metrics`/`_below_ratio` を再利用するための器）。

    C6 の学習時、_metrics は「骨格＋残差予測（reconstructed）」と元のラベルを比べたい。
    `_metrics`/`_below_ratio` は model.predict(x) を呼ぶだけなので、reconstructed を
    そのまま返す predict() を持たせれば実装を重複させずに済む。
    """

    def __init__(self, predictions: np.ndarray) -> None:
        self._predictions = predictions

    def predict(self, x: np.ndarray) -> np.ndarray:
        del x  # 予測は呼び出し側で計算済み
        return self._predictions


# ─────────────────────────────────────────────────────────────────────
# A1: そのペダルが効いている行だけで学習する
# ─────────────────────────────────────────────────────────────────────


def train_inverse_model_effective(
    logs: list[DriveLog],
    profile: VehicleProfile,
    output_dir: str = "data/models",
    feature_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    cruise_curve: CruiseCurve | None = None,
) -> tuple[str, dict[str, dict[str, float]]]:
    """A1: 効いている行だけで 2 次多項式 Ridge 逆モデルを学習し pkl 保存する。

    本番 `train_inverse_model` との違いは**学習行の選び方だけ**で、特徴量・推定器・pkl 形式は
    同じ（`FeedforwardController.load_model` がそのまま読める）。

    - アクセルモデル: `accel_opening >= accel_deadband_pct` の行だけ
    - ブレーキモデル: `brake_opening >= brake_deadband_pct` の行だけ
    - どちらでもない行（惰行）はどちらにも入れない

    現行は dv_1.0 の符号で全行を 2 分し、ブレーキ側は不感帯未満を 0 に潰していたため、
    「0% の行」と「14〜18% の行」が同じような入力で混ざり、回帰がその平均（＝不感帯の中）を
    返していた（レポート 表 3-2 の 66%）。

    Args:
        cruise_curve: 指定すると C6（骨格 + 残差 ML）としてアクセルモデルを学習する。骨格は
            行ごとに `cruise_skeleton(cruise_curve, params, v0, a_req)`（v0・a_req は特徴量
            から取り出す）で求め、アクセルモデルの目的変数を「ラベル − 骨格」にする。ブレーキ
            モデルは変えない。None（既定）なら今まで通り（C1）。

    Returns:
        (保存パス, {"accel": metrics, "brake": metrics})。metrics には本番の mae/rmse/r2/n に
        加えて `below_deadband`（学習行での予測が不感帯未満だった割合）が入る。`cruise_curve`
        指定時、accel の指標は骨格 + 残差予測（reconstructed）と元のラベルの比較で、C1 の
        指標とそのまま比べられる。

    Raises:
        LearningDataError: サンプルが不足しモデル構築できない場合
    """
    spec = feature_spec
    n_features = len(spec.feature_names())
    params = profile.feedforward_params
    accel_db = params.accel_deadband_pct
    brake_db = params.brake_deadband_pct

    x_accel_parts: list[np.ndarray] = []
    y_accel_parts: list[np.ndarray] = []
    x_brake_parts: list[np.ndarray] = []
    y_brake_parts: list[np.ndarray] = []
    speed_clip_max = 0.0  # 全ログの観測最高車速（推論時の入力クリップ＝外挿の飽和に使う）

    for session_logs in _group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([log.actual_speed_kmh for log in session_logs], dtype=float), 0.0, None
        )
        speed_clip_max = max(speed_clip_max, float(speed.max()))
        accel_open = np.array([log.accel_opening for log in session_logs], dtype=float)
        brake_open = np.array([log.brake_opening for log in session_logs], dtype=float)

        timestamps = [log.timestamp for log in session_logs]
        x, idx = _build_feature_matrix(
            speed,
            _estimate_offsets(timestamps, spec.lookahead_horizons_s),
            _estimate_offsets(timestamps, spec.past_horizons_s),
            spec,
        )
        if len(idx) == 0:
            continue

        # A1: そのペダルが効いている行だけを、そのモデルに入れる（惰行の行はどちらにも入れない）
        a_label, b_label = accel_open[idx], brake_open[idx]
        a_mask = a_label >= accel_db
        b_mask = b_label >= brake_db
        x_accel_parts.append(x[a_mask])
        y_accel_parts.append(a_label[a_mask])
        x_brake_parts.append(x[b_mask])
        y_brake_parts.append(b_label[b_mask])

    x_accel = np.vstack(x_accel_parts) if x_accel_parts else np.empty((0, n_features))
    y_accel = np.concatenate(y_accel_parts) if y_accel_parts else np.empty(0)
    x_brake = np.vstack(x_brake_parts) if x_brake_parts else np.empty((0, n_features))
    y_brake = np.concatenate(y_brake_parts) if y_brake_parts else np.empty(0)

    total = len(y_accel) + len(y_brake)
    if total < MIN_SAMPLES_FOR_TRAINING:
        raise LearningDataError(
            f"学習サンプルが不足しています ({total} 点)。"
            f"最低 {MIN_SAMPLES_FOR_TRAINING} 点必要です。"
        )
    if len(y_accel) < MIN_REGIME_SAMPLES:
        raise LearningDataError(
            f"アクセルが効いている行が不足しています ({len(y_accel)} 点、"
            f"開度 ≥ 不感帯 {accel_db:.2f}%)。最低 {MIN_REGIME_SAMPLES} 点必要です。"
        )
    if len(y_brake) < MIN_REGIME_SAMPLES:
        raise LearningDataError(
            f"ブレーキが効いている行が不足しています ({len(y_brake)} 点、"
            f"開度 ≥ 不感帯 {brake_db:.2f}%)。最低 {MIN_REGIME_SAMPLES} 点必要です。"
        )

    # C6: アクセルの目的変数を「ラベル − 骨格」にする（骨格は v0・a_req から一意に決まるので、
    # x_accel の列だけから再現できる。skeleton_accel を別途持ち回らなくてよい）
    skeleton_accel: np.ndarray | None = None
    if cruise_curve is not None:
        regime_col = spec.regime_col()
        v0_accel = x_accel[:, 0]
        a_req_accel = x_accel[:, regime_col] / spec.regime_horizon_s
        skeleton_accel = np.array(
            [
                cruise_skeleton(cruise_curve, params, float(v), float(a))
                for v, a in zip(v0_accel, a_req_accel, strict=True)
            ]
        )
        y_accel_fit = y_accel - skeleton_accel
    else:
        y_accel_fit = y_accel

    accel_model = _make_estimator()
    accel_model.fit(x_accel, y_accel_fit)
    brake_model = _make_estimator()
    brake_model.fit(x_brake, y_brake)

    # C6 は「骨格 + 残差予測（reconstructed）」を元のラベルと比べる。_metrics/_below_ratio は
    # model.predict(x) を呼ぶだけなので、reconstructed を返す _PrecomputedPredictor を挟めば
    # C1 の指標計算をそのまま再利用できる（C1 との比較可能性を保つ）
    if skeleton_accel is not None:
        reconstructed_accel = skeleton_accel + accel_model.predict(x_accel)
        accel_metrics_model: Any = _PrecomputedPredictor(reconstructed_accel)
    else:
        accel_metrics_model = accel_model

    metrics = {
        "accel": _metrics(accel_metrics_model, x_accel, y_accel),
        "brake": _metrics(brake_model, x_brake, y_brake),
    }
    # B3 の切り上げ前に、予測がどれだけ不感帯の中へ落ちているか（レポート 表 3-2 の指標）
    metrics["accel"]["below_deadband"] = _below_ratio(accel_metrics_model, x_accel, accel_db)
    metrics["brake"]["below_deadband"] = _below_ratio(brake_model, x_brake, brake_db)

    pkl_path = Path(output_dir) / (
        f"{Path(profile.id).name}_{datetime.now(tz=UTC).strftime('%Y%m%d_%H%M%S')}.pkl"
    )
    pkl_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "model_type": MODEL_TYPE,
        "accel_model": accel_model,
        "brake_model": brake_model,
        "feature_names": spec.feature_names(),
        "horizons": list(spec.lookahead_horizons_s),
        "past_horizons": list(spec.past_horizons_s),
        "regime_horizon": spec.regime_horizon_s,
        "feature_spec": asdict(spec),
        "speed_clip_max": speed_clip_max,
        "profile_id": profile.id,
        "trained_at": datetime.now(tz=UTC).isoformat(),
        "metrics": metrics,
        # 研究用の追加キー（load_model は読み飛ばす）。学習セットの作り方を pkl に残す
        "training_rows": TRAINING_ROWS_EFFECTIVE,
        "deadbands_pct": {"accel": accel_db, "brake": brake_db},
    }
    if cruise_curve is not None:
        # C6 専用キー。CandidateC6.load_model はこれが無い pkl（= C1）を拒否する
        payload["cruise_curve"] = cruise_curve.to_dict()
    with pkl_path.open("wb") as f:
        pickle.dump(payload, f)

    return str(pkl_path), metrics


def _below_ratio(model: object, x: np.ndarray, deadband_pct: float) -> float:
    """学習行での予測（0 クランプ後）が不感帯未満だった割合。"""
    if len(x) == 0:
        return 0.0
    pred = np.maximum(0.0, np.asarray(model.predict(x), dtype=float))  # type: ignore[attr-defined]
    return float(np.mean(pred < deadband_pct))


# ─────────────────────────────────────────────────────────────────────
# B1〜B3: レジーム合成
# ─────────────────────────────────────────────────────────────────────


class CandidateFeedforward(FeedforwardController):
    """C1 の FF。停車保持・クリープ任せ・学習域クリップは現行のまま、合成だけ差し替える。

    `FeedforwardController` を継承しているので、モデルのロード（`load_model`）・物理定数の設定
    （`set_params`）・ホライズンの参照は本番と同じ実装を使う。差し替えるのは `predict_effort` の
    「手順 3 レジーム合成」だけ。
    """

    #: 走行ログ・レポートに残す候補名（後から取り違えないため）
    candidate: str = "C1"
    #: V2: True の候補（C4・C5）は呼び出し側（mode_drive.py）が v0/future/past を
    #: 実測車速から組み立てる。C1〜C3 は基準車速のまま
    uses_actual_speed: bool = False

    def predict_effort(
        self, v0: float, future_speeds: Sequence[float], past_speeds: Sequence[float]
    ) -> float:
        """C1 の符号付き努力量 [%]（正: 名目アクセル開度、負: 名目ブレーキ開度）。

        1. 停車保持（現行と同じ）
        2. クリープ任せ（現行と同じ）
        3. B1 要求加速度が惰行の加速度以上ならアクセル、下ならブレーキ（B2 惰行テーパは無い）
        4. B3 選んだペダルの開度を不感帯以上に切り上げる
        """
        if self._accel_model is None or self._brake_model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        p = self._params
        spec = self._spec

        # 1. 停車レジーム（現行と同じ。判定は最短ホライズンのみ）
        if v0 <= STOP_SPEED_KMH and future_speeds and future_speeds[0] <= STOP_SPEED_KMH:
            return -max(0.0, min(100.0, p.stop_brake_opening_pct))

        # C6: 骨格（実測テーブル）はクリップ前の v0 を参照する（外挿は opening_at の直線延長が
        # 担う）。残差モデルの入力は今まで通りクリップ後の特徴量。
        v0_raw = v0

        # 学習域クリップ（現行と同じ。v0 > cm のときは軌跡全体を平行移動して dv を保つ）
        cm = self._speed_clip_max
        if cm is not None:
            if v0 > cm:
                shift = v0 - cm
                v0 = cm
                future_speeds = [f - shift for f in future_speeds]
                past_speeds = [s - shift for s in past_speeds]
            future_speeds = [min(f, cm) for f in future_speeds]
            past_speeds = [min(s, cm) for s in past_speeds]

        features = build_feature_row(v0, future_speeds, past_speeds, spec)
        dv_regime = float(features[0, spec.regime_col()])
        desired_accel = dv_regime / spec.regime_horizon_s  # km/h/s（正: 加速, 負: 減速）

        # 2. クリープ任せ（現行と同じ。クリープ能力内の加速要求はペダル不要）
        if v0 < p.creep_speed_kmh and 0.0 <= desired_accel <= p.creep_rate_kmhs:
            return 0.0

        accel_pred = self._accel_opening(v0_raw, desired_accel, features)
        brake_pred = max(0.0, float(self._brake_model.predict(features)[0]))

        # 3. B1: 両ペダルを離したときの加速度（クリープ域はクリープ加速率）を境目にする。
        #    現行の「dv_1.0 の符号」だと、惰行より緩い減速＝アクセルが要る場面がブレーキ側に落ちる。
        coast = p.creep_rate_kmhs if v0 < p.creep_speed_kmh else -coast_decel_at(p, v0)
        # 4. B3: 選んだペダルは不感帯以上（効かない指令を出さない）。B2: 惰行テーパは無い。
        effort = (
            max(p.accel_deadband_pct, accel_pred)
            if desired_accel >= coast
            else self._brake_effort(v0, desired_accel, coast, brake_pred)
        )
        return max(-100.0, min(100.0, effort))

    def _brake_effort(
        self, v0: float, desired_accel: float, coast: float, brake_pred: float
    ) -> float:
        """ブレーキ側の effort（符号付き、負）。C2 だけ物理式に差し替える。"""
        return -max(self._params.brake_deadband_pct, brake_pred)

    def _accel_opening(self, v0_raw: float, desired_accel: float, features: np.ndarray) -> float:
        """アクセル開度の予測（0 クランプ済み）。C6 だけ骨格 + 残差に差し替える。

        Args:
            v0_raw: 学習域クリップ前の v0（C6 の骨格参照用。C1〜C5 では未使用）。
            desired_accel: 要求加速度 [km/h/s]（C6 の骨格計算用）。
            features: クリップ後の特徴行（残差/本体モデルへの入力）。
        """
        model: Any = self._accel_model  # predict_effort で None でないことを確認済み
        return max(0.0, float(model.predict(features)[0]))


# ─────────────────────────────────────────────────────────────────────
# C2〜C5（KAIZEN 報告書 3 章 表 3-1 の定義。report20260912_KAIZEN_process2,3.md:249-253）
# ─────────────────────────────────────────────────────────────────────


class CandidateC2(CandidateFeedforward):
    """C2: ブレーキ側を物理式（不感帯 + Δa ÷ ペダルゲイン）に置き換える。

    本番 `src.domain.control.pedal_plan.analytic_efforts` のブレーキ分岐と同じ式。
    ペダルゲインが未同定（`pedal_gain_at` が None）の速度域は C1 のモデル予測にフォールバックする。
    """

    candidate = "C2"

    def _brake_effort(
        self, v0: float, desired_accel: float, coast: float, brake_pred: float
    ) -> float:
        p = self._params
        gain = pedal_gain_at(p, v0, is_accel=False)
        if gain is None:
            return super()._brake_effort(v0, desired_accel, coast, brake_pred)
        delta_a = desired_accel - coast  # 負（ブレーキ側）
        return -(-delta_a / gain + p.brake_deadband_pct)


class CandidateC3(CandidateFeedforward):
    """C3: C1 と同じロジック。先読み窓を 0.5s ずらして学習したモデルを読ませるだけ。

    `FeedforwardController.load_model` が pkl の horizons を読み込み、呼び出し側
    （mode_drive.py）はその horizons で future/past を組み立てるため、コードは C1 と同一でよい。
    候補名だけ別にして走行ログで取り違えないようにする。
    """

    candidate = "C3"


class CandidateC4(CandidateFeedforward):
    """C4: 動作点 v0 を実車速にする（先読みの変化量は基準車速のまま。偏差そのものは入れない）。

    `predict_effort` 自体は C1 と同じ。v0/future/past の組み立てを呼び出し側（mode_drive.py）が
    実車速ベースに変える（V2）。
    """

    candidate = "C4"
    uses_actual_speed = True


class CandidateC5(CandidateFeedforward):
    """C5: t 以前は実測・t 以降は基準の絶対値（FF の中に比例フィードバックが入る）。

    `predict_effort` 自体は C1 と同じ。past を実測履歴、future を基準の絶対値にする組み立ては
    呼び出し側（mode_drive.py）が行う（V2）。
    """

    candidate = "C5"
    uses_actual_speed = True


class CandidateC6(CandidateFeedforward):
    """C6: アクセル側の骨格を定速階段の実測テーブルにし、残差だけ ML で学習する。

    骨格 = `cruise_skeleton(cruise_curve, params, v0, a_req)`（実測テーブル + 要求加速度 ÷
    ペダルゲイン）。`_accel_model` は学習時に骨格を引いた残差を学習しているので、推論では
    骨格 + 残差予測を返す。ブレーキ側・停車保持・クリープ・B1・B3・学習域クリップは C1 と同じ。
    骨格の v0 だけクリップ前の実測値を使う（学習域を超えても `opening_at` の直線延長で外挿
    する。残差モデルの入力は今まで通りクリップ後の特徴量）。
    """

    candidate = "C6"

    def __init__(self) -> None:
        super().__init__()
        self._cruise_curve: CruiseCurve | None = None

    def load_model(self, model_path: str) -> None:
        """本体の load_model に加えて、pkl の `cruise_curve` キーを読む（無ければ ValueError）。"""
        super().load_model(model_path)
        with Path(model_path).open("rb") as f:
            payload: dict[str, Any] = pickle.load(f)  # noqa: S301 - 開発者が生成した信頼済み pkl
        if "cruise_curve" not in payload:
            raise ValueError(
                f"C6 用の pkl ではありません（cruise_curve キーがありません）: {model_path}。"
                "train_candidate_model.py --cruise-curve-from で作った pkl を指定してください。"
            )
        self._cruise_curve = CruiseCurve.from_dict(payload["cruise_curve"])

    def _accel_opening(self, v0_raw: float, desired_accel: float, features: np.ndarray) -> float:
        if self._cruise_curve is None:
            raise RuntimeError("C6 は cruise_curve が未ロードです。load_model() を呼んでください。")
        model: Any = self._accel_model  # predict_effort で None でないことを確認済み
        residual = float(model.predict(features)[0])
        skeleton = cruise_skeleton(self._cruise_curve, self._params, v0_raw, desired_accel)
        return max(0.0, skeleton + residual)


#: V1 案スイッチ: config の feedforward.candidate 名 → クラス
CANDIDATE_CLASSES: dict[str, type[CandidateFeedforward]] = {
    "C1": CandidateFeedforward,
    "C2": CandidateC2,
    "C3": CandidateC3,
    "C4": CandidateC4,
    "C5": CandidateC5,
    "C6": CandidateC6,
}


def make_candidate(name: str) -> CandidateFeedforward:
    """候補名（C1〜C6）から FF インスタンスを作る。未知の名前は ValueError。"""
    try:
        cls = CANDIDATE_CLASSES[name]
    except KeyError:
        raise ValueError(
            f"未知の候補です: {name!r}（{sorted(CANDIDATE_CLASSES)} のいずれか）"
        ) from None
    return cls()
