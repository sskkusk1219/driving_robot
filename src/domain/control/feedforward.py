import pickle
from collections.abc import Sequence
from pathlib import Path
from typing import Any, Protocol

from src.domain.model_training import (
    _REGIME_POS,
    LOOKAHEAD_HORIZONS_S,
    MODEL_TYPE,
    REGIME_HORIZON_S,
    STOP_SPEED_KMH,
    build_feature_row,
)
from src.models.profile import FeedforwardParams


class _Regressor(Protocol):
    """逆モデル推定器の構造的型（sklearn Ridge / Pipeline 等が満たす）。"""

    def predict(self, x: Any) -> Any: ...


class FeedforwardController:
    """先読み型 多項式Ridge 逆モデルによる FF 制御。load_model() 後に predict_effort() を呼ぶ。

    現在の基準速度 v0 と数秒先の基準速度（先読み）から、符号付き努力量
    （+: 名目アクセル開度 [%]、−: 名目ブレーキ開度 [%]）を予測する。
    追従誤差の補正は PID に、ペダルへの写像は PedalArbiter に委ねる純粋フィードフォワード。
    """

    def __init__(self) -> None:
        self._accel_model: _Regressor | None = None
        self._brake_model: _Regressor | None = None
        # 推論時に v0・先読み速度をこの上限へクリップして学習域外の外挿（多項式の暴れ）を防ぐ。
        # 学習データの観測最高車速。None は無制限（クリップしない）。
        self._speed_clip_max: float | None = None
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
            model_type:     "poly_inverse_lookahead"
            accel_model:    sklearn Pipeline（多項式＋標準化＋Ridge）
            brake_model:    sklearn Pipeline
            speed_clip_max: 学習観測最高車速（推論時の入力クリップ上限）
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
        clip = data.get("speed_clip_max")
        self._speed_clip_max = float(clip) if clip is not None else None

    def unload_model(self) -> None:
        """運転モデルを破棄する。プロファイル切替時に必ず呼び、前車両のモデル残留を防ぐ。

        モデルなしプロファイルへ切替えた際にアンロードしないと has_model が True のまま
        前車両のペダルマップが適用される（コードレビュー 2026-06-11 指摘 #4）。
        """
        self._accel_model = None
        self._brake_model = None
        self._speed_clip_max = None

    def predict_effort(self, v0: float, future_speeds: Sequence[float]) -> float:
        """現在の基準速度と先読み基準速度から符号付き努力量 [%] を返す。

        Args:
            v0: 現在の基準速度 [km/h]
            future_speeds: 各ホライズン（horizons 順）の基準速度 [km/h]

        Returns:
            努力量 [%]。正は名目アクセル開度、負は名目ブレーキ開度。-100.0〜100.0。

        Raises:
            RuntimeError: load_model() が呼ばれていない場合
        """
        if self._accel_model is None or self._brake_model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        p = self._params

        # 1. 停車レジーム: 停車保持ブレーキ（多項式モデルは原点で 0 を保証しないため定数で補う。
        #    負予測は後段で 0 にクランプされる）。判定は最短ホライズン（0.5 秒先）のみ:
        #    全先読み点（3 秒先まで）で判定すると発進の 3 秒前に保持ブレーキが解除され、
        #    クリープで基準 0 に逆らって動き出す（レビュー指摘 #5）。
        if v0 <= STOP_SPEED_KMH and future_speeds and future_speeds[0] <= STOP_SPEED_KMH:
            return -max(0.0, min(100.0, p.stop_brake_opening_pct))

        # 学習域外の外挿を防ぐため速度を観測最高車速 cm にクリップ（多項式が域外で暴れるのを避け
        # 学習端で飽和させる。残差は PID と包絡線ガバナが吸収する）。v0>cm のときは軌跡全体を平行
        # 移動して v0 を cm に置き、減速/加速の相対トレンド（dv＝レジーム判定の基）を保つ。単純に
        # 各点を独立クリップすると near-horizon の dv が 0 に潰れてレジームを誤判定するため。
        cm = self._speed_clip_max
        if cm is not None:
            if v0 > cm:
                shift = v0 - cm
                v0 = cm
                future_speeds = [f - shift for f in future_speeds]
            # 先読みが学習域を超える分（域外への加速要求）は学習端に飽和させる
            future_speeds = [min(f, cm) for f in future_speeds]

        features = build_feature_row(v0, future_speeds)
        # レジーム判定: 先読みトレンド dv_1.0 の符号（X 列順より features[0, 1+_REGIME_POS]）
        dv_regime = float(features[0, 1 + _REGIME_POS])
        desired_accel = dv_regime / REGIME_HORIZON_S  # km/h/s（正:加速, 負:減速）

        # 2. 両モデルを評価。負の予測はノイズとして 0。
        accel_pred = max(0.0, float(self._accel_model.predict(features)[0]))
        brake_pred = max(0.0, float(self._brake_model.predict(features)[0]))

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
            eng = p.engine_brake_decel_kmhs
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
