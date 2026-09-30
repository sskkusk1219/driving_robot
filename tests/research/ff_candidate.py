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

C6（骨格を定速階段の実測テーブルにする案）は 2026-09-25 段4 で、定速階段とともに削除した。

段2（2026-09-25 学習サンプルの WLTP 重み付け。ProblemReport_20260925）:
    `train_inverse_model_effective` に `weighting: WltpWeighting | None` を追加した。
    `weighting.enabled` のときだけ fit に `sample_weight` を渡す（既定 None は今までと完全に
    同じ経路）。`weighting` を渡した場合は enabled に関係なく metrics に `mae_wltp`
    （重み付き MAE。重みなし／ありのモデルを同じ物差しで比べるため）を追加する。

段4（2026-09-19 改訂: クリープ域ブレーキの下限。ProblemReport_20260916）:
    レジーム判定（1〜3）・B1〜B3・停車保持は変えない。B3 の後、ブレーキ側の開度だけ
    `_apply_brake_trim` で実測の停止境界（不感帯 + `stop_brake_floor_offset_pct`）を
    下回らないようにする（`brake_trim_max_kmh` 既定 0.0 なら何もしない＝後方互換）。
    カーブだった旧仕様（`ref_far` で停止／発進を場合分けし上限・下限の 2 式を使う）は
    実測で「境界に速度依存がほとんど無く、停止側・発進側が同じ規則で書ける」ことが
    分かったため定数 1 個の下限 1 本に単純化した。詳細は `_apply_brake_trim` の docstring。

手順6（2026-09-28 ホライズン自動選択。ProblemReport_20260921）:
    `train_inverse_model_effective` の `feature_spec` 引数を `accel_spec`/`brake_spec` に
    分割した（省略時は両方とも既定の `DEFAULT_FEATURE_SPEC`＝今までと完全に同じ）。
    `predict_effort`（`CandidateFeedforward` とその派生）はアクセル・ブレーキそれぞれの
    spec で特徴量行を組み、それぞれの `accel_model`/`brake_model` に渡す
    （`FeedforwardModel`（ff_model.py）の同じ仕組みを継承）。
"""

from __future__ import annotations

import pickle
from collections.abc import Sequence
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
from sklearn.metrics import mean_absolute_error

from tests.research.ff_model import (
    DEFAULT_FEATURE_SPEC,
    MIN_REGIME_SAMPLES,
    MIN_SAMPLES_FOR_TRAINING,
    MODEL_TYPE,
    STOP_SPEED_KMH,
    FeatureSpec,
    FeedforwardModel,
    build_feature_matrix,
    build_feature_row,
    estimate_offsets,
    group_by_session,
    make_estimator,
    metrics,
)
from tests.research.ff_params import ResearchFFParams, free_accel_at
from tests.research.learning_patterns import LearningDataError
from tests.research.reachability import decide_regime, free_speeds_at, reach_needs
from tests.research.research_types import DriveLog, VehicleProfile, pedal_gain_at
from tests.research.sample_weight import WltpWeighting, compute_weights, summarize

__all__ = [
    "CandidateC2",
    "CandidateC3",
    "CandidateC4",
    "CandidateC5",
    "CandidateFeedforward",
    "CANDIDATE_CLASSES",
    "make_candidate",
    "train_inverse_model_effective",
]

# pkl に残す学習セットの作り方（後から取り違えないため。load_model は無視する追加キー）
TRAINING_ROWS_EFFECTIVE: str = "effective_only"


# ─────────────────────────────────────────────────────────────────────
# A1: そのペダルが効いている行だけで学習する
# ─────────────────────────────────────────────────────────────────────


def train_inverse_model_effective(
    logs: list[DriveLog],
    profile: VehicleProfile,
    output_dir: str = "data/models",
    accel_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    brake_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    stop_horizon_s: float | None = None,
    weighting: WltpWeighting | None = None,
) -> tuple[str, dict[str, dict[str, float]]]:
    """A1: 効いている行だけで 2 次多項式 Ridge 逆モデルを学習し pkl 保存する。

    本番 `train_inverse_model` との違いは**学習行の選び方だけ**で、特徴量・推定器・pkl 形式は
    同じ（`FeedforwardModel.load_model` がそのまま読める）。

    - アクセルモデル: `accel_opening >= accel_deadband_pct` の行だけ
    - ブレーキモデル: `brake_opening >= brake_deadband_pct` の行だけ
    - どちらでもない行（惰行）はどちらにも入れない

    現行は dv_1.0 の符号で全行を 2 分し、ブレーキ側は不感帯未満を 0 に潰していたため、
    「0% の行」と「14〜18% の行」が同じような入力で混ざり、回帰がその平均（＝不感帯の中）を
    返していた（レポート 表 3-2 の 66%）。

    手順6（ProblemReport_20260921。2026-09-28）: `accel_spec`/`brake_spec` を別々に渡すと、
    アクセル・ブレーキが別のホライズンで学習される（`horizon_search.py` が交差検証で選ぶ）。
    省略時（両方とも既定の `DEFAULT_FEATURE_SPEC`）は今までと完全に同じ挙動。

    Args:
        accel_spec: アクセルモデルの特徴量構成。
        brake_spec: ブレーキモデルの特徴量構成。`accel_spec` と `regime_horizon_s` が違うと
            レジーム判定が二重になるため ValueError。
        stop_horizon_s: 停車保持の判定・ブレーキ下限の `ref_next` が見るホライズン。省略時は
            `accel_spec.lookahead_horizons_s[0]`（旧来の「先読みの先頭」と同じ）。
        weighting: 指定すると WLTP の車速×加速度分布に基づく学習サンプル重み（段2。
            `sample_weight.compute_weights`）をアクセル・ブレーキそれぞれに求める。
            `weighting.enabled` が True のときだけ実際に `fit` へ渡す（False は今まで通り
            重みなしで学習する）。None（既定）なら計算自体行わず、今までと完全に同じ挙動。

    Returns:
        (保存パス, {"accel": metrics, "brake": metrics})。metrics には本番の mae/rmse/r2/n に
        加えて `below_deadband`（学習行での予測が不感帯未満だった割合）が入る。
        `weighting` 指定時（enabled に関係なく）は `mae_wltp`（重み付き MAE）と
        `weight_min`/`weight_max`/`weight_mean`/`weight_at_min_ratio`/
        `weight_at_max_ratio`（学習行の重み分布の要約）が入る。

    Raises:
        ValueError: `accel_spec`/`brake_spec` の `regime_horizon_s` が食い違う場合、または
            `stop_horizon_s` がどちらの spec の `lookahead_horizons_s` にも含まれない場合
        LearningDataError: サンプルが不足しモデル構築できない場合
    """
    if accel_spec.regime_horizon_s != brake_spec.regime_horizon_s:
        raise ValueError(
            f"accel_spec.regime_horizon_s({accel_spec.regime_horizon_s}) と "
            f"brake_spec.regime_horizon_s({brake_spec.regime_horizon_s}) が違います"
            "（レジーム判定は両ペダル共通のホライズンを使います）"
        )
    if stop_horizon_s is None:
        stop_horizon_s = accel_spec.lookahead_horizons_s[0]

    params = profile.feedforward_params
    accel_db = params.accel_deadband_pct
    brake_db = params.brake_deadband_pct

    x_accel_parts: list[np.ndarray] = []
    y_accel_parts: list[np.ndarray] = []
    x_brake_parts: list[np.ndarray] = []
    y_brake_parts: list[np.ndarray] = []
    speed_clip_max = 0.0  # 全ログの観測最高車速（推論時の入力クリップ＝外挿の飽和に使う）
    n_accel_features = len(accel_spec.feature_names())
    n_brake_features = len(brake_spec.feature_names())

    for session_logs in group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([log.actual_speed_kmh for log in session_logs], dtype=float), 0.0, None
        )
        speed_clip_max = max(speed_clip_max, float(speed.max()))
        accel_open = np.array([log.accel_opening for log in session_logs], dtype=float)
        brake_open = np.array([log.brake_opening for log in session_logs], dtype=float)
        timestamps = [log.timestamp for log in session_logs]

        # A1: そのペダルが効いている行だけを、そのモデルに入れる（惰行の行はどちらにも入れない）。
        # アクセル・ブレーキで spec（ホライズン）が違うと有効行の範囲も違うため、別々に作る。
        xa, idx_a = build_feature_matrix(
            speed,
            estimate_offsets(timestamps, accel_spec.lookahead_horizons_s),
            estimate_offsets(timestamps, accel_spec.past_horizons_s),
            accel_spec,
        )
        if len(idx_a) > 0:
            a_label = accel_open[idx_a]
            a_mask = a_label >= accel_db
            x_accel_parts.append(xa[a_mask])
            y_accel_parts.append(a_label[a_mask])

        xb, idx_b = build_feature_matrix(
            speed,
            estimate_offsets(timestamps, brake_spec.lookahead_horizons_s),
            estimate_offsets(timestamps, brake_spec.past_horizons_s),
            brake_spec,
        )
        if len(idx_b) > 0:
            b_label = brake_open[idx_b]
            b_mask = b_label >= brake_db
            x_brake_parts.append(xb[b_mask])
            y_brake_parts.append(b_label[b_mask])

    x_accel = np.vstack(x_accel_parts) if x_accel_parts else np.empty((0, n_accel_features))
    y_accel = np.concatenate(y_accel_parts) if y_accel_parts else np.empty(0)
    x_brake = np.vstack(x_brake_parts) if x_brake_parts else np.empty((0, n_brake_features))
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

    # v0・要求加速度（regime ホライズン先）は段2 の WLTP 重み付けが使う取り出し方
    # （wltp_grid._v0_and_a_req と同じ定義）。regime_horizon_s は両 spec で共通なので、
    # 値は spec が違っても同じ意味になる（列位置だけがそれぞれの spec 内で違う）
    v0_accel = x_accel[:, 0]
    a_req_accel = x_accel[:, accel_spec.regime_col()] / accel_spec.regime_horizon_s
    v0_brake = x_brake[:, 0]
    a_req_brake = x_brake[:, brake_spec.regime_col()] / brake_spec.regime_horizon_s

    # 段2: WLTP 重み付け。weighting が None なら計算自体行わず、今までと完全に同じ経路
    # （fit に sample_weight を渡さない）。weighting.enabled が False でも重みは計算する
    # （mae_wltp で重みなし/ありを同じ物差しで比較できるようにするため）
    w_accel: np.ndarray | None = None
    w_brake: np.ndarray | None = None
    if weighting is not None:
        w_accel = compute_weights(v0_accel, a_req_accel, weighting)
        w_brake = compute_weights(v0_brake, a_req_brake, weighting)

    accel_model = make_estimator()
    brake_model = make_estimator()
    if weighting is not None and weighting.enabled:
        accel_model.fit(x_accel, y_accel, ridge__sample_weight=w_accel)
        brake_model.fit(x_brake, y_brake, ridge__sample_weight=w_brake)
    else:
        accel_model.fit(x_accel, y_accel)
        brake_model.fit(x_brake, y_brake)

    fit_metrics = {
        "accel": metrics(accel_model, x_accel, y_accel),
        "brake": metrics(brake_model, x_brake, y_brake),
    }
    # B3 の切り上げ前に、予測がどれだけ不感帯の中へ落ちているか（レポート 表 3-2 の指標）
    fit_metrics["accel"]["below_deadband"] = _below_ratio(accel_model, x_accel, accel_db)
    fit_metrics["brake"]["below_deadband"] = _below_ratio(brake_model, x_brake, brake_db)
    # 段2: weighting 指定時は enabled に関係なく mae_wltp と重み分布の要約を残す（重みなし／
    # ありのモデルを同じ物差しで比べるため）
    if weighting is not None:
        assert w_accel is not None  # weighting is not None のとき必ず計算済み
        assert w_brake is not None
        fit_metrics["accel"]["mae_wltp"] = float(
            mean_absolute_error(y_accel, accel_model.predict(x_accel), sample_weight=w_accel)
        )
        fit_metrics["brake"]["mae_wltp"] = float(
            mean_absolute_error(y_brake, brake_model.predict(x_brake), sample_weight=w_brake)
        )
        for side, w in (("accel", w_accel), ("brake", w_brake)):
            for key, value in summarize(w).items():
                if key != "n":
                    fit_metrics[side][f"weight_{key}"] = value

    # 制御用 spec（pkl の feature_spec）: アクセル・ブレーキのホライズンの和集合＋stop_horizon_s。
    # mode_drive はこの並びで future/past を組む（ff.horizons / ff.past_horizons）
    control_lookahead = tuple(
        sorted({*accel_spec.lookahead_horizons_s, *brake_spec.lookahead_horizons_s, stop_horizon_s})
    )
    control_past = tuple(sorted({*accel_spec.past_horizons_s, *brake_spec.past_horizons_s}))
    control_spec = FeatureSpec(
        lookahead_horizons_s=control_lookahead,
        past_horizons_s=control_past,
        regime_horizon_s=accel_spec.regime_horizon_s,
        include_v0_sq=accel_spec.include_v0_sq,
        include_dv_regime_x_v0=accel_spec.include_dv_regime_x_v0,
        past_as_delta=accel_spec.past_as_delta,
    )

    # ファイル名はローカル時刻（走行ログ CSV = drive_log.py:142 の datetime.now() と揃える）で
    # 付ける。同じ走行の CSV と pkl をファイル名で対応づけるため。
    # 注意: 2026-09-20 より前に作られた pkl は UTC 命名（JST − 9h）なので混在する。
    pkl_path = Path(output_dir) / (
        f"{Path(profile.id).name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pkl"
    )
    pkl_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "model_type": MODEL_TYPE,
        "accel_model": accel_model,
        "brake_model": brake_model,
        "feature_names": accel_spec.feature_names(),
        "horizons": list(control_spec.lookahead_horizons_s),
        "past_horizons": list(control_spec.past_horizons_s),
        "regime_horizon": control_spec.regime_horizon_s,
        "feature_spec": asdict(control_spec),
        "accel_feature_spec": asdict(accel_spec),
        "brake_feature_spec": asdict(brake_spec),
        "stop_horizon_s": stop_horizon_s,
        "speed_clip_max": speed_clip_max,
        "profile_id": profile.id,
        "trained_at": datetime.now(tz=UTC).isoformat(),
        "metrics": fit_metrics,
        # 研究用の追加キー（load_model は読み飛ばす）。学習セットの作り方を pkl に残す
        "training_rows": TRAINING_ROWS_EFFECTIVE,
        "deadbands_pct": {"accel": accel_db, "brake": brake_db},
        # 段2: どの重み設定で学習したか（研究設定は走行ログに残らないため、pkl で追えるように）
        "sample_weight": (
            {"enabled": weighting.enabled, "w_min": weighting.w_min, "w_max": weighting.w_max}
            if weighting is not None
            else {"enabled": False}
        ),
    }
    with pkl_path.open("wb") as f:
        pickle.dump(payload, f)

    return str(pkl_path), fit_metrics


def _below_ratio(model: object, x: np.ndarray, deadband_pct: float) -> float:
    """学習行での予測（0 クランプ後）が不感帯未満だった割合。"""
    if len(x) == 0:
        return 0.0
    pred = np.maximum(0.0, np.asarray(model.predict(x), dtype=float))  # type: ignore[attr-defined]
    return float(np.mean(pred < deadband_pct))


# ─────────────────────────────────────────────────────────────────────
# B1〜B3: レジーム合成
# ─────────────────────────────────────────────────────────────────────


class CandidateFeedforward(FeedforwardModel):
    """C1 の FF。停車保持・学習域クリップは現行のまま、合成だけ差し替える。

    `FeedforwardModel`（ff_model.py）を継承しているので、モデルのロード（`load_model`）・物理定数の設定
    （`set_params`）・ホライズンの参照は本番と同じ実装を使う。差し替えるのは `predict_effort` の
    「手順 3 レジーム合成」だけ。

    ProblemReport_20260916 段2: 旧「クリープ任せ」分岐（幅 0.23 km/h/s の固定窓）を廃止し、
    惰行（＝両ペダルを離したときの加速度 `free_accel_at`）の ±`coast_band_kmhs` を惰行レジームと
    する帯判定に一本化した。`set_research_params` を呼ばなければ `coast_band_kmhs` は既定 0.0
    （帯は無効）で、B1 の閾値計算（`free_accel_at` のフォールバック）は段1前の C1 と同じになる。
    """

    #: 走行ログ・レポートに残す候補名（後から取り違えないため）
    candidate: str = "C1"
    #: V2: True の候補（C4・C5）は呼び出し側（mode_drive.py）が v0/future/past を
    #: 実測車速から組み立てる。C1〜C3 は基準車速のまま
    uses_actual_speed: bool = False

    def __init__(self) -> None:
        super().__init__()
        self._research = ResearchFFParams()  # 既定は空＝帯も無効（後方互換）

    def set_research_params(self, research: ResearchFFParams) -> None:
        """研究側のみのパラメータ（クリープ加速カーブ・惰行帯）。`set_params` と対で呼ぶ。"""
        self._research = research

    def predict_effort(
        self,
        v0: float,
        future_speeds: Sequence[float],
        past_speeds: Sequence[float],
        *,
        select_accel_kmhs: float | None = None,
        select_speed_kmh: float | None = None,
    ) -> float:
        """C1 の符号付き努力量 [%]（正: 名目アクセル開度、負: 名目ブレーキ開度）。

        1. 停車保持（現行と同じ）
        2. レジーム判定（惰行／アクセル／ブレーキ）
           - 段3 無効時（`reach_horizons_s` 空。既定）: 要求加速度が惰行の加速度
             （`free_accel_at`。1.0s 一定とみなす）の ±帯 内なら惰行（旧「クリープ任せ」の
             固定窓はこの帯に吸収した）。それ以外は B1（要求 ≥ 惰行ならアクセル、下なら
             ブレーキ。B2 惰行テーパは無い）
           - 段3 有効時: 惰行のまま進んだ先の速度 `v_free(t+h)` を数値積分で作り、各
             ホライズンの `need(h)` が全部帯の中なら惰行。外に出た最短ホライズンの符号で
             アクセル／ブレーキを選ぶ（`reachability.decide_regime` 参照）
        3. 開度の計算式は段3 の有無で変えない（`desired_accel` を使う 1.0s 1 点の値のまま
           `_accel_opening`/`_brake_effort` へ渡す）
        4. B3 選んだペダルの開度を不感帯以上に切り上げる

        ペダル選択の差し替え（ProblemReport_20260929 段3）: `select_accel_kmhs`（窓の傾き G）と
        `select_speed_kmh`（惰行加速度を評価する速度 = ref(t+L)）が渡されたときだけ、2. の
        「惰行／アクセル／ブレーキ」の判定に使う要求加速度と惰行加速度を差し替える。開度の計算
        （モデル予測・`_brake_effort` の desired_accel/coast）は一切変えない。None（既定）なら
        従来と完全に同じ。`reach_horizons_s`（段3 到達可能性）と併用すると ValueError。
        """
        if self._accel_model is None or self._brake_model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        p = self._params
        spec = self._spec  # 制御用（アクセル・ブレーキの和集合）

        # 1. 停車レジーム（現行と同じ。判定は stop_horizon_s のみ。手順6: アクセル・ブレーキが
        #    別のホライズンを選んでも、この判定は固定のホライズンを見続ける）
        future_map_raw = dict(zip(spec.lookahead_horizons_s, future_speeds, strict=True))
        stop_future = future_map_raw.get(self._stop_horizon_s)
        if v0 <= STOP_SPEED_KMH and stop_future is not None and stop_future <= STOP_SPEED_KMH:
            return -max(0.0, min(100.0, p.stop_brake_opening_pct))

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

        # 手順6: future_speeds/past_speeds は spec（和集合）の並びで渡される。ホライズン値で
        # 引ける辞書にしてから、ペダルごとの spec に必要な列だけを取り出す。
        future_map = dict(zip(spec.lookahead_horizons_s, future_speeds, strict=True))
        past_map = dict(zip(spec.past_horizons_s, past_speeds, strict=True))
        dv_regime = future_map[spec.regime_horizon_s] - v0
        desired_accel = dv_regime / spec.regime_horizon_s  # km/h/s（正: 加速, 負: 減速）

        # 2. B1: 両ペダルを離したときの加速度。クリープ域は +creep_accel_at(v0)、それ以上は
        #    -coast_decel_at(v0)（段1で作った単一ソース）。旧「クリープ任せ」分岐はこの帯に
        #    吸収した（`coast_band_kmhs` 既定 0.0 なら帯は無効）
        coast = free_accel_at(p, self._research, v0)
        use_select = select_accel_kmhs is not None or select_speed_kmh is not None
        if use_select and self._research.reach_horizons_s:
            raise ValueError(
                "select_accel_kmhs/select_speed_kmh は reach_horizons_s と併用できません"
            )
        if self._research.reach_horizons_s:
            # 段3: 惰行のまま進んだ先の速度 v_free(t+h) と基準を各ホライズンで比べる
            # （coast を 1 秒一定とみなす近似をやめ、ホライズンも 1 点から複数へ）。
            # future_speeds・v0 はここまでで学習域クリップ済みの値（特徴量と基準をそろえる）
            hs = self._research.reach_horizons_s
            v_free = free_speeds_at(p, self._research, v0, hs, step_s=self._research.reach_step_s)
            needs = reach_needs([future_map[h] for h in hs], v_free, hs)
            decisive = decide_regime(needs, self._research.coast_band_kmhs)
            if decisive is None:
                return 0.0
            want_accel = needs[decisive] >= 0.0
        else:
            # 3. 惰行レジーム: 要求が惰行の ±帯 内なら、どちらのペダルも使わない（待機位置）
            # ペダル選択だけを差し替える（窓の傾き G と ref(t+L) での惰行加速度）。開度側の
            # desired_accel・coast は変えない。速度は v0 と同じ学習域クリップを掛ける
            sel_accel = desired_accel if select_accel_kmhs is None else select_accel_kmhs
            sel_coast = coast
            if select_speed_kmh is not None:
                sel_v = select_speed_kmh if cm is None else min(select_speed_kmh, cm)
                sel_coast = free_accel_at(p, self._research, sel_v)
            if abs(sel_accel - sel_coast) < self._research.coast_band_kmhs:
                return 0.0
            want_accel = sel_accel >= sel_coast

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
        # 4. B3: 選んだペダルは不感帯以上（効かない指令を出さない）。B2: 惰行テーパは無い。
        # 段4: ブレーキ側だけ、学習域クリップ後の stop_horizon_s 先（= ref_next）を渡して
        # クリープ域ブレーキの下限を効かせる（_apply_brake_trim 参照）
        effort = (
            max(p.accel_deadband_pct, accel_pred)
            if want_accel
            else self._brake_effort(
                v0, desired_accel, coast, brake_pred, ref_next=future_map[self._stop_horizon_s]
            )
        )
        return max(-100.0, min(100.0, effort))

    def _brake_effort(
        self, v0: float, desired_accel: float, coast: float, brake_pred: float,
        *, ref_next: float,
    ) -> float:
        """ブレーキ側の effort（符号付き、負）。C2 だけ物理式に差し替える。

        B3（不感帯切り上げ）の後に段4 クリープ域ブレーキの下限（`_apply_brake_trim`）を適用し、
        トリム後の値を改めて不感帯以上へ切り上げる（下限側は理屈上 brake_deadband_pct +
        stop_brake_floor_offset_pct 以上になるので不感帯を割ることは無いはずだが、境界を保つため
        `_apply_brake_trim` と同じ規約で二重にくくる）。
        """
        opening = max(self._params.brake_deadband_pct, brake_pred)
        trimmed = self._apply_brake_trim(opening, v0, ref_next)
        return -max(self._params.brake_deadband_pct, trimmed)

    def _apply_brake_trim(self, opening_pct: float, v0: float, ref_next: float) -> float:
        """段4 クリープ域ブレーキの下限（正値・不感帯適用前の開度に対して働く）。

        用語: **停止境界** = 転がっている車を、そのブレーキ開度のまま停車まで持っていける
        最小の開度。

        実測（2026-09-19 04:06 の手順2 `drive_log_real_20260919_040613.csv`。
        ProblemReport_20260916 段4）: クリープ平衡 4.77 km/h からブレーキを保持した実測で、
        不感帯超 +6.0%（開度 17.79%）は 0.33 km/h に浮き、**+8.0%（19.79%）は 1.78s で停止**
        （停止確認開度 18.32% とも整合）。境界は +6.0〜+8.0% の間で、0.33〜4.95 km/h の
        どの開始速度でもほぼ同じ帯に収まり、**速度依存がほとんど見えない**。

        `RunFF_6` の停止接近 5 回は、ピーク開度 18.45/18.71/19.50/19.17/21.07%（全部境界の
        すぐ内側）まで踏み増して減速できていたのに、要求減速の縮小につれて最後の約0.6秒だけ
        17〜18.2% へ緩めて境界を割り、車が 1.0〜1.7 km/h で浮いて偏差
        +0.97/+1.17/+1.44/+1.05/+1.62 km/h を出した（KPI 最大 1.0 km/h 超）。発進 5 回は
        同じ原因の裏返しで、停車保持 28.42% → 境界の下 17.05% へ抜くので車が基準より先に出て
        +0.25〜+0.59 km/h 先行した。どちらも「基準がまだ 0 付近なのに境界を割っている」状態
        なので、**下限 1 本で両方が直る**（旧仕様の `ref_far` による停止／発進の場合分けは
        不要——境界に方向性が無いため）。

        レートリミット（開度変化率の制限）は実装しない（ユーザー方針）。この下限は速度と
        基準の関数であって、時間の関数ではない。

        `RunFF_6` の実測行への試算（`brake_trim_ref_kmh=0.3`・`brake_trim_max_kmh=5.0`）:
        下限が効くのは 10 区間・8.35s / 589s だけで、その全部が停止接近の最後や発進直後の
        場面と一致した。引き上げ量は平均 +2.63%・最大 +4.27%。

        規則（`brake_trim_max_kmh<=0.0`・`v0` がその上限超・`stop_brake_floor_offset_pct`
        未同定（0.0）・`ref_next` が `brake_trim_ref_kmh` 超のいずれかなら何もしない）:
            `floor = brake_deadband_pct + stop_brake_floor_offset_pct`
            `opening = max(opening, floor)`
        `ref_next`（future_speeds[0]。最短ホライズン先の基準）で判定するのは、基準が
        まだ動き出していない（≈0）間だけ下限を掛けたいため。発進直後は `v0` がまだ 0 の
        ままなので `v0` では判定できない。基準が `brake_trim_ref_kmh` を超えて動き出したら
        （＝発進が進んだら）下限を解放する。
        """
        research = self._research
        if research.brake_trim_max_kmh <= 0.0 or research.stop_brake_floor_offset_pct <= 0.0:
            return opening_pct  # 既定は現行と完全に同じ経路
        if v0 > research.brake_trim_max_kmh:
            return opening_pct
        if ref_next > research.brake_trim_ref_kmh:
            return opening_pct
        floor = self._params.brake_deadband_pct + research.stop_brake_floor_offset_pct
        return max(opening_pct, floor)  # 上側は既存のクランプに任せる


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
        self, v0: float, desired_accel: float, coast: float, brake_pred: float,
        *, ref_next: float,
    ) -> float:
        p = self._params
        gain = pedal_gain_at(p, v0, is_accel=False)
        if gain is None:
            return super()._brake_effort(
                v0, desired_accel, coast, brake_pred, ref_next=ref_next
            )
        delta_a = desired_accel - coast  # 負（ブレーキ側）
        opening = -delta_a / gain + p.brake_deadband_pct
        trimmed = self._apply_brake_trim(opening, v0, ref_next)
        return -max(p.brake_deadband_pct, trimmed)


class CandidateC3(CandidateFeedforward):
    """C3: C1 と同じロジック。先読み窓を 0.5s ずらして学習したモデルを読ませるだけ。

    `FeedforwardModel.load_model` が pkl の horizons を読み込み、呼び出し側
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


#: V1 案スイッチ: config の feedforward.candidate 名 → クラス
CANDIDATE_CLASSES: dict[str, type[CandidateFeedforward]] = {
    "C1": CandidateFeedforward,
    "C2": CandidateC2,
    "C3": CandidateC3,
    "C4": CandidateC4,
    "C5": CandidateC5,
}


def make_candidate(name: str) -> CandidateFeedforward:
    """候補名（C1〜C5）から FF インスタンスを作る。未知の名前は ValueError。"""
    try:
        cls = CANDIDATE_CLASSES[name]
    except KeyError:
        raise ValueError(
            f"未知の候補です: {name!r}（{sorted(CANDIDATE_CLASSES)} のいずれか）"
        ) from None
    return cls()
