import pickle
from collections.abc import Sequence
from pathlib import Path
from typing import Any

from sklearn.linear_model import Ridge

from src.domain.model_training import (
    _REGIME_POS,
    LOOKAHEAD_HORIZONS_S,
    MODEL_TYPE,
    REGIME_HORIZON_S,
    STOP_SPEED_KMH,
    build_feature_row,
)
from src.models.profile import FeedforwardParams


class FeedforwardController:
    """先読み型 Ridge 逆モデルによる FF 制御。load_model() 後に predict() を呼ぶ。

    現在の基準速度 v0 と数秒先の基準速度（先読み）から、名目アクセル/ブレーキ開度を予測する。
    追従誤差の補正は PID に委ねる純粋フィードフォワード。
    """

    def __init__(self) -> None:
        self._accel_model: Ridge | None = None
        self._brake_model: Ridge | None = None
        # 車両物理定数。select_profile 時に set_params で更新する。未設定時は AT 標準デフォルト。
        self._params: FeedforwardParams = FeedforwardParams()

    @property
    def horizons(self) -> tuple[float, ...]:
        """先読みホライズン [s]（呼び出し元が future_speeds を組むために参照する）。"""
        return LOOKAHEAD_HORIZONS_S

    @property
    def has_model(self) -> bool:
        """運転モデルがロード済みか。未学習（初回学習走行）では False。"""
        return self._accel_model is not None and self._brake_model is not None

    def set_params(self, params: FeedforwardParams) -> None:
        """車両プロファイルのフィードフォワード物理定数を設定する。"""
        self._params = params

    def load_model(self, model_path: str) -> None:
        """pkl ファイルから先読み Ridge 逆モデルをロードする。

        pkl は train_inverse_model が出力する dict 形式:
            model_type:   "ridge_inverse_lookahead"
            accel_model:  sklearn Ridge
            brake_model:  sklearn Ridge
            feature_names, horizons, ...

        運転モデルは開発者がローカルで生成した信頼済みファイルのみを想定。
        外部入力のパスをそのまま渡さないこと（呼び出し元の責務）。
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

        required_keys = {"accel_model", "brake_model"}
        missing = required_keys - data.keys()
        if missing:
            raise ValueError(f"Model file is missing required keys: {missing}")

        self._accel_model = data["accel_model"]
        self._brake_model = data["brake_model"]

    def predict(self, v0: float, future_speeds: Sequence[float]) -> tuple[float, float]:
        """現在の基準速度と先読み基準速度からアクセル・ブレーキ開度[%]を返す。

        Args:
            v0: 現在の基準速度 [km/h]
            future_speeds: 各ホライズン（horizons 順）の基準速度 [km/h]

        Returns:
            (accel_opening, brake_opening) — 各 0.0〜100.0 [%]

        Raises:
            RuntimeError: load_model() が呼ばれていない場合
        """
        if self._accel_model is None or self._brake_model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        p = self._params

        # 1. 停車レジーム: 基準が停車（現在も先読みも停車しきい値以下）→ 停車保持ブレーキ
        #    fit_intercept=False の Ridge は原点で 0 を返すため、定数で補う必要がある。
        if v0 <= STOP_SPEED_KMH and all(fs <= STOP_SPEED_KMH for fs in future_speeds):
            return 0.0, max(0.0, min(100.0, p.stop_brake_opening_pct))

        features = build_feature_row(v0, future_speeds)
        # レジーム判定: 先読みトレンド dv_1.0 の符号（X 列順より features[0, 1+_REGIME_POS]）
        dv_regime = float(features[0, 1 + _REGIME_POS])
        desired_accel = dv_regime / REGIME_HORIZON_S  # km/h/s（正:加速, 負:減速）

        # 2. Ridge 予測（レジーム別）
        if dv_regime >= 0.0:
            accel_raw = float(self._accel_model.predict(features)[0])
            brake_raw = 0.0
        else:
            accel_raw = 0.0
            brake_raw = float(self._brake_model.predict(features)[0])

        # 3. クリープ域: 低速で要求加速がクリープ能力内 → 放っておけば進むのでペダル 0
        if v0 < p.creep_speed_kmh and abs(desired_accel) <= p.creep_rate_kmhs:
            accel_raw = 0.0
            brake_raw = 0.0

        # 4. 惰行: 緩減速がエンジンブレーキの範囲内 → ブレーキ不要（惰行で足りる）
        if dv_regime < 0.0 and (-desired_accel) <= p.engine_brake_decel_kmhs:
            brake_raw = 0.0

        # 5. 不感帯スナップ: 遊び未満の微小開度は 0（チャタリング抑制）
        if accel_raw < p.accel_deadband_pct:
            accel_raw = 0.0
        if brake_raw < p.brake_deadband_pct:
            brake_raw = 0.0

        accel_opening = max(0.0, min(100.0, accel_raw))
        brake_opening = max(0.0, min(100.0, brake_raw))
        return accel_opening, brake_opening
