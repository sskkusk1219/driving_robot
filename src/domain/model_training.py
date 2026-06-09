"""連続走行ログから先読み型 Ridge 逆モデルを学習するドメインモジュール。

人間の運転を模した「先読み（look-ahead / preview）型」の純粋フィードフォワード逆モデル。
現在の基準速度 v0 と数秒先の基準速度トレンド（Δv）から、名目アクセル/ブレーキ開度を予測する。
追従誤差の補正は PID に委ねるため、FF は基準軌跡のみの関数とする。

学習データ構築:
    手動/学習走行の連続ログには目標速度系列が記録されないため、実車速の軌跡そのものを
    「その走行で意図された目標軌跡」とみなす。
        v0          = actual_speed[i]
        target(t+k) = actual_speed[i + offset_k]
        ラベル       = accel_opening[i] / brake_opening[i]
    推論時は v0 = 基準車速(t)、target(t+k) = 基準車速(t+k)（CSV を先読み）で一貫する。

先読み特徴量 (7 次元): [v0, dv_0.5, dv_1.0, dv_2.0, dv_3.0, v0², dv_1.0·v0]
レジーム分割: dv_1.0 >= 0 → アクセルモデル、< 0 → ブレーキモデル。
"""

import pickle
from collections.abc import Sequence
from dataclasses import replace
from datetime import UTC, datetime
from pathlib import Path

import numpy as np
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

from src.domain.learning_drive import LearningDataError
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams, VehicleProfile

MODEL_TYPE: str = "ridge_inverse_lookahead"

# 先読みホライズン [s]。学習・推論で共有する。
LOOKAHEAD_HORIZONS_S: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0)
# アクセル/ブレーキのレジーム判定に使うホライズン。LOOKAHEAD_HORIZONS_S に含まれること。
REGIME_HORIZON_S: float = 1.0

BRAKE_DEADBAND_PCT: float = 1.0  # これ未満のブレーキ開度はノイズとして 0 扱い
RIDGE_ALPHA: float = 1.0
DEFAULT_DT_S: float = 0.1  # ログ周期が推定できない場合のフォールバック (100ms)

MIN_SAMPLES_FOR_TRAINING: int = 20  # 全レジーム合計の最小サンプル数
MIN_REGIME_SAMPLES: int = 8  # 各モデル（アクセル/ブレーキ）の最小サンプル数

# 物理定数推定用
STOP_SPEED_KMH: float = 0.5  # これ以下を「停車」とみなす（FF predict でも使用）
CREEP_STEADY_TOL_KMHS: float = 0.3  # クリープ定常判定の |加速度| しきい値
MIN_OBS_SAMPLES: int = 5  # 各定数を上書きするのに必要な最小観測サンプル数

# 特徴量名（v0, 各ホライズンの Δv, v0², dv_regime·v0）
FEATURE_NAMES: list[str] = [
    "v0",
    *[f"dv_{h}" for h in LOOKAHEAD_HORIZONS_S],
    "v0_sq",
    "dv1_x_v0",
]

_REGIME_POS: int = LOOKAHEAD_HORIZONS_S.index(REGIME_HORIZON_S)
# X の列順: [v0, dv_0.5, dv_1.0, ..., v0², dv1·v0] なのでレジーム列は 1 + _REGIME_POS
_REGIME_COL: int = 1 + _REGIME_POS


def build_feature_row(v0: float, future_speeds: Sequence[float]) -> np.ndarray:
    """1 サンプル分の特徴量ベクトル (1, n_features) を返す（推論用）。

    Args:
        v0: 現在の基準速度 [km/h]
        future_speeds: 各ホライズン（LOOKAHEAD_HORIZONS_S 順）の基準速度 [km/h]

    Raises:
        ValueError: future_speeds の長さがホライズン数と一致しない場合
    """
    if len(future_speeds) != len(LOOKAHEAD_HORIZONS_S):
        raise ValueError(
            f"future_speeds の長さ {len(future_speeds)} は "
            f"ホライズン数 {len(LOOKAHEAD_HORIZONS_S)} と一致する必要があります"
        )
    dv = [fs - v0 for fs in future_speeds]
    dv1 = dv[_REGIME_POS]
    row = [v0, *dv, v0 * v0, dv1 * v0]
    return np.array([row], dtype=float)


def _build_feature_matrix(speed: np.ndarray, offsets: list[int]) -> tuple[np.ndarray, np.ndarray]:
    """速度系列から先読み特徴量行列と有効サンプル indices を返す（学習用）。

    末尾 max(offsets) サンプルは未来データが不足するため除外する。
    """
    max_off = max(offsets)
    n_valid = len(speed) - max_off
    if n_valid <= 0:
        return np.empty((0, len(FEATURE_NAMES))), np.empty(0, dtype=int)

    idx = np.arange(n_valid)
    v0 = speed[idx]
    dv_cols = [speed[idx + off] - v0 for off in offsets]
    dv1 = dv_cols[_REGIME_POS]
    X = np.column_stack([v0, *dv_cols, v0 * v0, dv1 * v0])
    return X, idx


def _estimate_offsets(timestamps: list[datetime]) -> list[int]:
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
    return [max(1, round(h / dt)) for h in LOOKAHEAD_HORIZONS_S]


def _group_by_session(logs: list[DriveLog]) -> list[list[DriveLog]]:
    """ログを session_id でグループ化し、各グループを timestamp 昇順で返す。

    先読み特徴量がセッション境界をまたがないようにするため必須。
    """
    grouped: dict[str, list[DriveLog]] = {}
    for log in logs:
        grouped.setdefault(log.session_id, []).append(log)
    return [sorted(g, key=lambda x: x.timestamp) for g in grouped.values()]


def _metrics(model: Ridge, x: np.ndarray, y: np.ndarray) -> dict[str, float]:
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
) -> tuple[str, dict[str, dict[str, float]]]:
    """連続走行ログから先読み Ridge 逆モデルを学習し pkl 保存してパスとメトリクスを返す。

    Args:
        logs: 学習に使う連続走行ログ（複数セッション可）
        profile: 紐づける車両プロファイル
        output_dir: pkl 出力ディレクトリ

    Returns:
        (保存パス, {"accel": metrics, "brake": metrics})

    Raises:
        LearningDataError: サンプルが不足しモデル構築できない場合
    """
    x_accel_parts: list[np.ndarray] = []
    y_accel_parts: list[np.ndarray] = []
    x_brake_parts: list[np.ndarray] = []
    y_brake_parts: list[np.ndarray] = []

    # ブレーキ整形のデッドバンドは車両プロファイルの不感帯を使う（既定 1.0 で従来と等価）
    brake_db = profile.feedforward_params.brake_deadband_pct

    for session_logs in _group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([log.actual_speed_kmh for log in session_logs], dtype=float), 0.0, None
        )
        accel_open = np.array([log.accel_opening for log in session_logs], dtype=float)
        brake_raw = np.array([log.brake_opening for log in session_logs], dtype=float)
        brake_open = np.where(brake_raw >= brake_db, brake_raw, 0.0)

        offsets = _estimate_offsets([log.timestamp for log in session_logs])
        x, idx = _build_feature_matrix(speed, offsets)
        if len(idx) == 0:
            continue

        # レジーム分割: dv_1.0 >= 0 → アクセル、< 0 → ブレーキ
        accel_mask = x[:, _REGIME_COL] >= 0.0
        x_accel_parts.append(x[accel_mask])
        y_accel_parts.append(accel_open[idx][accel_mask])
        x_brake_parts.append(x[~accel_mask])
        y_brake_parts.append(brake_open[idx][~accel_mask])

    x_accel = np.vstack(x_accel_parts) if x_accel_parts else np.empty((0, len(FEATURE_NAMES)))
    y_accel = np.concatenate(y_accel_parts) if y_accel_parts else np.empty(0)
    x_brake = np.vstack(x_brake_parts) if x_brake_parts else np.empty((0, len(FEATURE_NAMES)))
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

    # fit_intercept=False: 停止時 (v0=0, 全 dv=0) → 開度 0 の物理制約を保証
    accel_model = Ridge(alpha=RIDGE_ALPHA, fit_intercept=False)
    accel_model.fit(x_accel, y_accel)
    brake_model = Ridge(alpha=RIDGE_ALPHA, fit_intercept=False)
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
        "feature_names": FEATURE_NAMES,
        "horizons": list(LOOKAHEAD_HORIZONS_S),
        "regime_horizon": REGIME_HORIZON_S,
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


def estimate_dynamics_params(logs: list[DriveLog], current: FeedforwardParams) -> FeedforwardParams:
    """連続走行ログから物理定数を推定し、十分なサンプルがある項目のみ上書きする。

    観測が不足する項目は current の値を保持する。不感帯（accel/brake）は
    ログからの信頼推定が困難なため推定対象外（current を保持）。
    """
    accel_db = current.accel_deadband_pct
    brake_db = current.brake_deadband_pct

    stop_brakes: list[float] = []
    creep_speeds: list[float] = []
    eng_decels: list[float] = []
    creep_rates: list[float] = []

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

    new_stop = _median_or_none(stop_brakes)
    new_creep_speed = _median_or_none(creep_speeds)
    new_eng = _median_or_none(eng_decels)
    new_rate = _median_or_none(creep_rates)

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
    )
