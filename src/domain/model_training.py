"""連続走行ログから先読み型 多項式Ridge 逆モデルを学習するドメインモジュール。

人間の運転を模した「先読み（look-ahead / preview）型」の純粋フィードフォワード逆モデル。
現在の基準速度 v0 と数秒先の基準速度トレンド（Δv）から、名目アクセル/ブレーキ開度を予測する。
追従誤差の補正は PID に委ねるため、FF は基準軌跡のみの関数とする。

推定器は完全2次多項式展開＋標準化＋Ridge の Pipeline（`_make_estimator`）。特徴量には過去方向Δv
（0.5s/1.0s 前からの速度変化）を含め、ランプ過渡か定常保持かを識別する。
学習は最新の学習走行セッションのみを用いる（Web 経路で session_id を限定）。

不採用の経緯（フェーズ6-2→6-3）: 一時、単調制約付き HistGradientBoosting（木）を試したが、
in-sample の MAE/RMSE/R² は大幅改善した一方、実走行モード（WLTP-ExHi）で閉ループが破綻した。
学習パターン（ACCEL_SWEEP: cap まで加速→即ブレーキ）には定常巡航サンプルがほぼ無く、巡航域は
学習データの外（off-manifold）になる。木モデルはこの領域で速度によらず一定値に飽和し、走行中の
巡航で開度不足→失速、ランプでは開度過大→過加速という往復を起こした。poly Ridge は同じ外挿域でも
滑らかに補間し実用的な開度を返すため、閉ループ追従を優先し poly に復帰した（詳細は
`.steering/20260630-learning-wide-speed-range/design.md` フェーズ6-3）。in-sample/CV の指標だけでは
この off-manifold 性能を判定できない点に注意。

学習データ構築:
    手動/学習走行の連続ログには目標速度系列が記録されないため、実車速の軌跡そのものを
    「その走行で意図された目標軌跡」とみなす。
        v0          = actual_speed[i]
        target(t+k) = actual_speed[i + offset_k]
        ラベル       = accel_opening[i] / brake_opening[i]
    推論時は v0 = 基準車速(t)、target(t+k) = 基準車速(t+k)（CSV を先読み）で一貫する。

特徴量構成は `FeatureSpec` で設定可能（.steering/20260703-learning-process-revamp）。
デフォルト `DEFAULT_FEATURE_SPEC` は現行9特徴と完全一致する:
    [v0, dv_0.5, dv_1.0, dv_2.0, dv_3.0, v0², dv_1.0·v0, dv_past_0.5, dv_past_1.0]
レジーム分割: dv_{regime_horizon_s} >= 0 → アクセルモデル、< 0 → ブレーキモデル。
"""

import pickle
from collections.abc import Sequence
from dataclasses import asdict, dataclass, replace
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
from sklearn.pipeline import Pipeline, make_pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler

from src.domain.learning_drive import LearningDataError
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams, VehicleProfile

# 多項式(2次)＋標準化＋Ridge の逆モデル。ブレーキの「不感帯→急制動」非線形を、同じ次元数の
# 入力の完全2次多項式展開で表現する（純線形 Ridge では捉えられず減速 R² が頭打ちだった）。
# 旧識別子 "poly_inverse_lookahead"（7特徴）・"gbm_inverse_lookahead"（GBM・閉ループ破綻で不採用）・
# "poly_past_inverse_lookahead"（FeatureSpec 導入前の9特徴固定）のいずれとも異なる新識別子にし、
# feature_spec を持たない旧 pkl を確実に拒否して再学習を強制する。
MODEL_TYPE: str = "poly_spec_inverse_lookahead"
POLY_DEGREE: int = 2  # 多項式特徴量の次数
RIDGE_ALPHA: float = 1.0

BRAKE_DEADBAND_PCT: float = 1.0  # これ未満のブレーキ開度はノイズとして 0 扱い
DEFAULT_DT_S: float = 0.1  # ログ周期が推定できない場合のフォールバック (100ms)

MIN_SAMPLES_FOR_TRAINING: int = 20  # 全レジーム合計の最小サンプル数
MIN_REGIME_SAMPLES: int = 8  # 各モデル（アクセル/ブレーキ）の最小サンプル数

# 物理定数推定用
STOP_SPEED_KMH: float = 0.5  # これ以下を「停車」とみなす（FF predict でも使用）
CREEP_STEADY_TOL_KMHS: float = 0.3  # クリープ定常判定の |加速度| しきい値
MIN_OBS_SAMPLES: int = 5  # 各定数を上書きするのに必要な最小観測サンプル数

# 不感帯（accel/brake）推定用: 開度→応答(加速度)曲線をビン分割し、開度ゼロ近傍の
# ベースライン応答をマージン以上上回る最初のビンの下端を不感帯境界とみなす
# （学習運転の ACCEL_DEADBAND_PROBE / BRAKE_HOLD 低開度段が主なサンプル源）。
DEADBAND_BIN_WIDTH_PCT: float = 0.5  # 開度ビン幅
DEADBAND_SCAN_MAX_PCT: float = 10.0  # この開度まで探索（探索上限＝クランプ上限を兼ねる）
DEADBAND_ONSET_MARGIN_KMHS: float = 0.3  # ベースラインを上回る「応答あり」判定マージン
DEADBAND_MIN_BIN_SAMPLES: int = 5  # ビンを採用するのに必要な最小サンプル数


@dataclass(frozen=True)
class FeatureSpec:
    """先読み逆モデルの特徴量構成。

    先読みホライズン・過去ホライズン・加速度項・二次項の有無を設定可能にし、候補セットの
    オフライン評価（`scripts/evaluate_feature_sets.py`）を可能にする。デフォルト値
    （`DEFAULT_FEATURE_SPEC`）は現行9特徴と完全一致する。

    Attributes:
        lookahead_horizons_s: 先読みホライズン [s]。昇順（狭義単調増加）である必要がある
            （停車ショートサーキットが `future_speeds[0]` を最短ホライズンとして参照するため）。
        past_horizons_s: 過去方向ホライズン [s]。過去Δv（ランプ過渡か定常保持か）の識別に使う。
        regime_horizon_s: アクセル/ブレーキのレジーム判定に使うホライズン。
            `lookahead_horizons_s` に含まれること。
        include_v0_sq: 二次項 v0² を含めるか。
        include_dv_regime_x_v0: 交互作用項 dv_{regime}·v0 を含めるか。
        accel_horizons_s: 加速度項を追加するホライズン集合。各 h は中央差分
            `a_h = (v(t+h) - 2·v0 + v(t-h)) / h²` として構築するため、
            `lookahead_horizons_s` と `past_horizons_s` の両方に含まれること。
    """

    lookahead_horizons_s: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0)
    past_horizons_s: tuple[float, ...] = (0.5, 1.0)
    regime_horizon_s: float = 1.0
    include_v0_sq: bool = True
    include_dv_regime_x_v0: bool = True
    accel_horizons_s: tuple[float, ...] = ()

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

    def feature_names(self) -> list[str]:
        """特徴量ベクトルの列順に対応する名前一覧を返す。"""
        names = ["v0", *[f"dv_{h}" for h in self.lookahead_horizons_s]]
        if self.include_v0_sq:
            names.append("v0_sq")
        if self.include_dv_regime_x_v0:
            names.append("dv1_x_v0")
        names.extend(f"dv_past_{h}" for h in self.past_horizons_s)
        names.extend(f"accel_{h}" for h in self.accel_horizons_s)
        return names

    def regime_col(self) -> int:
        """特徴行列内の dv_{regime_horizon_s} の列番号（0始まり）を返す。"""
        return 1 + self.lookahead_horizons_s.index(self.regime_horizon_s)


DEFAULT_FEATURE_SPEC = FeatureSpec()  # 現行9特徴と完全一致（回帰テストで固定）


def _make_estimator() -> Pipeline:
    """逆モデルの推定器を返す: N次元入力 → 完全2次多項式展開 → 標準化 → Ridge。

    ブレーキの「不感帯→急制動」非線形を、多項式展開（dv²・dv·v0 等の相互作用を含む）で表現する。
    StandardScaler はスケール差の大きい多項式項（v0² 等）の正則化を均す。`build_feature_row` の
    入力をそのまま受け、Pipeline 内で展開するため呼び出し側の入力形状は不変。

    HistGradientBoosting（木）は学習データに無い巡航域（off-manifold）で予測が定数に飽和し
    実走行の閉ループを破綻させたため不採用（フェーズ6-3）。poly Ridge は同じ外挿域でも滑らかに
    補間する。
    """
    return make_pipeline(
        PolynomialFeatures(POLY_DEGREE, include_bias=False),
        StandardScaler(),
        Ridge(alpha=RIDGE_ALPHA),
    )


def build_feature_row(
    v0: float,
    future_speeds: Sequence[float],
    past_speeds: Sequence[float],
    spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
) -> np.ndarray:
    """1 サンプル分の特徴量ベクトル (1, n_features) を返す（推論用）。

    Args:
        v0: 現在の基準速度 [km/h]
        future_speeds: 各ホライズン（`spec.lookahead_horizons_s` 順）の基準速度 [km/h]
        past_speeds: 各過去ホライズン（`spec.past_horizons_s` 順）の基準速度 [km/h]
        spec: 特徴量構成。既定は現行9特徴。

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
    dv = [fs - v0 for fs in future_speeds]
    regime_pos = spec.lookahead_horizons_s.index(spec.regime_horizon_s)
    dv_regime = dv[regime_pos]
    dv_past = [v0 - ps for ps in past_speeds]

    row: list[float] = [v0, *dv]
    if spec.include_v0_sq:
        row.append(v0 * v0)
    if spec.include_dv_regime_x_v0:
        row.append(dv_regime * v0)
    row.extend(dv_past)
    for h in spec.accel_horizons_s:
        f_val = future_speeds[spec.lookahead_horizons_s.index(h)]
        p_val = past_speeds[spec.past_horizons_s.index(h)]
        row.append((f_val - 2.0 * v0 + p_val) / (h * h))
    return np.array([row], dtype=float)


def _build_feature_matrix(
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
    dv_cols = [speed[idx + off] - v0 for off in offsets]
    regime_pos = spec.lookahead_horizons_s.index(spec.regime_horizon_s)
    dv_regime = dv_cols[regime_pos]
    past_cols = [v0 - speed[idx - off] for off in past_offsets]

    cols: list[np.ndarray] = [v0, *dv_cols]
    if spec.include_v0_sq:
        cols.append(v0 * v0)
    if spec.include_dv_regime_x_v0:
        cols.append(dv_regime * v0)
    cols.extend(past_cols)
    for h in spec.accel_horizons_s:
        off = offsets[spec.lookahead_horizons_s.index(h)]
        past_off = past_offsets[spec.past_horizons_s.index(h)]
        a_h = (speed[idx + off] - 2.0 * v0 + speed[idx - past_off]) / (h * h)
        cols.append(a_h)

    X = np.column_stack(cols)
    return X, idx


def _estimate_offsets(timestamps: list[datetime], horizons: Sequence[float]) -> list[int]:
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


def _group_by_session(logs: list[DriveLog]) -> list[list[DriveLog]]:
    """ログを session_id でグループ化し、各グループを timestamp 昇順で返す。

    先読み特徴量がセッション境界をまたがないようにするため必須。
    """
    grouped: dict[str, list[DriveLog]] = {}
    for log in logs:
        grouped.setdefault(log.session_id, []).append(log)
    return [sorted(g, key=lambda x: x.timestamp) for g in grouped.values()]


def _metrics(model: Pipeline, x: np.ndarray, y: np.ndarray) -> dict[str, float]:
    """学習データに対する評価指標（in-sample）を返す。"""
    pred = model.predict(x)
    result: dict[str, float] = {
        "mae": float(mean_absolute_error(y, pred)),
        "rmse": float(mean_squared_error(y, pred) ** 0.5),
        "n": float(len(y)),
    }
    # R² は分散がある程度ないと不安定なため 2 サンプル以上かつ分散 > 0 の場合のみ
    if len(y) >= 2 and float(np.var(y)) > 1e-9:
        result["r2"] = float(r2_score(y, pred))
    return result


def train_inverse_model(
    logs: list[DriveLog],
    profile: VehicleProfile,
    output_dir: str = "data/models",
    feature_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
) -> tuple[str, dict[str, dict[str, float]]]:
    """連続走行ログから先読み 多項式Ridge 逆モデルを学習し pkl 保存してパスとメトリクスを返す。

    Args:
        logs: 学習に使う連続走行ログ（複数セッション可）
        profile: 紐づける車両プロファイル
        output_dir: pkl 出力ディレクトリ
        feature_spec: 特徴量構成。既定は現行9特徴（`DEFAULT_FEATURE_SPEC`）。

    Returns:
        (保存パス, {"accel": metrics, "brake": metrics})

    Raises:
        LearningDataError: サンプルが不足しモデル構築できない場合
    """
    spec = feature_spec
    n_features = len(spec.feature_names())
    x_accel_parts: list[np.ndarray] = []
    y_accel_parts: list[np.ndarray] = []
    x_brake_parts: list[np.ndarray] = []
    y_brake_parts: list[np.ndarray] = []
    speed_clip_max = 0.0  # 全ログの観測最高車速（推論時の入力クリップ＝外挿の飽和に使う）

    # ブレーキ整形のデッドバンドは車両プロファイルの不感帯を使う（既定 1.0 で従来と等価）
    brake_db = profile.feedforward_params.brake_deadband_pct
    regime_col = spec.regime_col()

    for session_logs in _group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([log.actual_speed_kmh for log in session_logs], dtype=float), 0.0, None
        )
        speed_clip_max = max(speed_clip_max, float(speed.max()))
        accel_open = np.array([log.accel_opening for log in session_logs], dtype=float)
        brake_raw = np.array([log.brake_opening for log in session_logs], dtype=float)
        brake_open = np.where(brake_raw >= brake_db, brake_raw, 0.0)

        timestamps = [log.timestamp for log in session_logs]
        offsets = _estimate_offsets(timestamps, spec.lookahead_horizons_s)
        past_offsets = _estimate_offsets(timestamps, spec.past_horizons_s)
        x, idx = _build_feature_matrix(speed, offsets, past_offsets, spec)
        if len(idx) == 0:
            continue

        # レジーム分割: dv_{regime} >= 0 → アクセル、< 0 → ブレーキ（coast 含む全減速標本）。
        # coast（brake < 不感帯）はラベル 0 として残し、多項式モデルが「緩減速ではブレーキ 0」も
        # 学習する（推論時の coast 緩減速は FF のエンジンブレーキ テーパが担当）。
        accel_mask = x[:, regime_col] >= 0.0
        x_accel_parts.append(x[accel_mask])
        y_accel_parts.append(accel_open[idx][accel_mask])
        x_brake_parts.append(x[~accel_mask])
        y_brake_parts.append(brake_open[idx][~accel_mask])

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
            f"加速側のサンプルが不足しています ({len(y_accel)} 点)。"
            f"最低 {MIN_REGIME_SAMPLES} 点必要です。"
        )
    if len(y_brake) < MIN_REGIME_SAMPLES:
        raise LearningDataError(
            f"減速側のサンプルが不足しています ({len(y_brake)} 点)。"
            f"最低 {MIN_REGIME_SAMPLES} 点必要です。"
        )

    # 多項式(2次)＋標準化＋Ridge。停止時 (v0=0,全dv=0) → 開度0 の物理制約は fit_intercept ではなく
    # FF 側（predict_effort の停車短絡・負予測クランプ）が担保する。
    accel_model = _make_estimator()
    accel_model.fit(x_accel, y_accel)
    brake_model = _make_estimator()
    brake_model.fit(x_brake, y_brake)

    metrics = {
        "accel": _metrics(accel_model, x_accel, y_accel),
        "brake": _metrics(brake_model, x_brake, y_brake),
    }

    safe_profile_id = Path(profile.id).name
    timestamp = datetime.now(tz=UTC).strftime("%Y%m%d_%H%M%S")
    filename = f"{safe_profile_id}_{timestamp}.pkl"

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    pkl_path = out_dir / filename

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
    }
    with pkl_path.open("wb") as f:
        pickle.dump(payload, f)

    return str(pkl_path), metrics


def _median_or_none(values: list[float]) -> float | None:
    """十分なサンプルがあれば中央値、なければ None を返す。"""
    return float(np.median(values)) if len(values) >= MIN_OBS_SAMPLES else None


def _estimate_onset_deadband_pct(openings: np.ndarray, response: np.ndarray) -> float | None:
    """開度→応答曲線から不感帯（無反応域）の境界開度を推定する。

    開度を `DEADBAND_BIN_WIDTH_PCT` 幅でビン分割し、開度ゼロ近傍ビン（bin 0）の中央値応答を
    「無反応時ベースライン」とする。それを `DEADBAND_ONSET_MARGIN_KMHS` 以上上回る最初のビンの
    下端開度を不感帯境界として返す。各ビンは `DEADBAND_MIN_BIN_SAMPLES` 以上のサンプルを要求し
    （統計的信頼性）、満たさないビンはスキップする。ベースラインが求まらない、または境界が
    見つからない場合は None を返す（呼び出し元は既存値を保持する）。

    Args:
        openings: 開度サンプル列 [%]（0 以上、`DEADBAND_SCAN_MAX_PCT` 程度までを想定）
        response: 対応する応答（アクセルは dv、ブレーキは -dv）[km/h/s]。大きいほど反応あり。
    """
    if len(openings) == 0:
        return None

    n_bins = int(DEADBAND_SCAN_MAX_PCT / DEADBAND_BIN_WIDTH_PCT) + 1
    bin_medians: list[float | None] = []
    for i in range(n_bins):
        lo = i * DEADBAND_BIN_WIDTH_PCT
        hi = (i + 1) * DEADBAND_BIN_WIDTH_PCT
        mask = (openings >= lo) & (openings < hi)
        if int(np.count_nonzero(mask)) < DEADBAND_MIN_BIN_SAMPLES:
            bin_medians.append(None)
            continue
        bin_medians.append(float(np.median(response[mask])))

    baseline = bin_medians[0]
    if baseline is None:
        return None

    for i in range(1, n_bins):
        m = bin_medians[i]
        if m is None:
            continue
        if m >= baseline + DEADBAND_ONSET_MARGIN_KMHS:
            return i * DEADBAND_BIN_WIDTH_PCT
    return None


def estimate_dynamics_params(logs: list[DriveLog], current: FeedforwardParams) -> FeedforwardParams:
    """連続走行ログから物理定数を推定し、十分なサンプルがある項目のみ上書きする。

    観測が不足する項目は current の値を保持する。不感帯（accel/brake）は、学習運転の
    ACCEL_DEADBAND_PROBE（低アクセル開度の意図的保持）・BRAKE_HOLD の低開度段から得られる
    開度→応答曲線のオンセット検出で推定する（`_estimate_onset_deadband_pct`）。これらの
    専用プローブが無い旧ログのみの場合は十分なビンが埋まらず None となり、既存値を保持する。
    """
    accel_db = current.accel_deadband_pct
    brake_db = current.brake_deadband_pct

    stop_brakes: list[float] = []
    creep_speeds: list[float] = []
    eng_decels: list[float] = []
    creep_rates: list[float] = []
    accel_scan_openings: list[float] = []
    accel_scan_dv: list[float] = []
    brake_scan_openings: list[float] = []
    brake_scan_decel: list[float] = []

    for session_logs in _group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([lg.actual_speed_kmh for lg in session_logs], dtype=float), 0.0, None
        )
        accel = np.array([lg.accel_opening for lg in session_logs], dtype=float)
        brake = np.array([lg.brake_opening for lg in session_logs], dtype=float)

        epochs = np.array([lg.timestamp.timestamp() for lg in session_logs])
        d = np.diff(epochs)
        d = d[d > 0.0]
        dt = float(np.median(d)) if len(d) > 0 else DEFAULT_DT_S
        if dt <= 0.0:
            dt = DEFAULT_DT_S

        dv = np.diff(speed) / dt  # i→i+1 の加速度 [km/h/s]（長さ n-1）
        sp = speed[:-1]  # dv に揃えた始点速度
        pedal_off = (accel[:-1] < accel_db) & (brake[:-1] < brake_db)

        # 停車保持ブレーキ: 停車中にかけているブレーキ開度
        m_stop = (speed < STOP_SPEED_KMH) & (brake >= brake_db)
        stop_brakes.extend(brake[m_stop].tolist())

        # クリープ車速: ペダルオフで定常（|dv| 小）かつ動いている
        m_creep = pedal_off & (sp > STOP_SPEED_KMH) & (np.abs(dv) < CREEP_STEADY_TOL_KMHS)
        creep_speeds.extend(sp[m_creep].tolist())

        # エンジンブレーキ減速量: ペダルオフ・クリープ超・減速中
        m_eng = pedal_off & (sp > current.creep_speed_kmh) & (dv < 0.0)
        eng_decels.extend((-dv[m_eng]).tolist())

        # クリープ加速率: ペダルオフ・低速・加速中
        m_rate = pedal_off & (sp >= STOP_SPEED_KMH) & (sp < current.creep_speed_kmh) & (dv > 0.0)
        creep_rates.extend(dv[m_rate].tolist())

        # 不感帯推定用スキャンサンプル: 他ペダルオフ・探索上限以下の開度域で開度→応答を収集
        m_accel_scan = (brake[:-1] < brake_db) & (accel[:-1] <= DEADBAND_SCAN_MAX_PCT)
        accel_scan_openings.extend(accel[:-1][m_accel_scan].tolist())
        accel_scan_dv.extend(dv[m_accel_scan].tolist())

        m_brake_scan = (accel[:-1] < accel_db) & (brake[:-1] <= DEADBAND_SCAN_MAX_PCT)
        brake_scan_openings.extend(brake[:-1][m_brake_scan].tolist())
        brake_scan_decel.extend((-dv[m_brake_scan]).tolist())

    new_stop = _median_or_none(stop_brakes)
    new_creep_speed = _median_or_none(creep_speeds)
    new_eng = _median_or_none(eng_decels)
    new_rate = _median_or_none(creep_rates)
    new_accel_db = _estimate_onset_deadband_pct(
        np.array(accel_scan_openings), np.array(accel_scan_dv)
    )
    new_brake_db = _estimate_onset_deadband_pct(
        np.array(brake_scan_openings), np.array(brake_scan_decel)
    )

    return replace(
        current,
        stop_brake_opening_pct=(
            new_stop if new_stop is not None else current.stop_brake_opening_pct
        ),
        creep_speed_kmh=(
            new_creep_speed if new_creep_speed is not None else current.creep_speed_kmh
        ),
        engine_brake_decel_kmhs=(
            new_eng if new_eng is not None else current.engine_brake_decel_kmhs
        ),
        creep_rate_kmhs=(new_rate if new_rate is not None else current.creep_rate_kmhs),
        accel_deadband_pct=(
            new_accel_db if new_accel_db is not None else current.accel_deadband_pct
        ),
        brake_deadband_pct=(
            new_brake_db if new_brake_db is not None else current.brake_deadband_pct
        ),
    )
