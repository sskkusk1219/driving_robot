import bisect
import pickle
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

from src.domain.model_training import (
    DEFAULT_FEATURE_SPEC,
    MODEL_TYPE,
    STOP_SPEED_KMH,
    FeatureSpec,
    build_feature_row,
)
from src.models.profile import FeedforwardParams

# ── 速度依存プラントゲイン（ゲインスケジューリング）─────────────────────────
# g(v) = ∂(ペダル開度[%]) / ∂(目標加速度[km/h/s]) を FF モデルの局所勾配から求めた値のクランプ。
# 単位は %/(km/h/s)。公称値 g_nominal = fopdt_tau/fopdt_k（同オーダー）を基準に比を取るため、
# 生の局所勾配レンジ（実測 0.5〜3 程度、モデルにより外れも）を有界にする下限・上限。
_GAIN_MIN: float = 0.05
_GAIN_MAX: float = 5.0
# 局所勾配を測る目標加速度の摂動 [km/h/s]。定加速度マニフォールド（future_h=v+a·h）上で
# a=±この値の差分から ∂開度/∂a を算出する。学習運転の過渡（数 km/h/s）内の中庸値。
_GAIN_ACCEL_PROBE_KMHS: float = 0.5


def _moving_average_3(ys: Sequence[float]) -> list[float]:
    """3点移動平均（端点は自分＋隣接1点）。局所勾配ノイズを平滑化する。"""
    n = len(ys)
    if n <= 2:
        return list(ys)
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - 1)
        hi = min(n, i + 2)
        window = ys[lo:hi]
        out[i] = sum(window) / len(window)
    return out


def _interp_sorted(x: float, xs: Sequence[float], ys: Sequence[float]) -> float:
    """昇順 xs 上の (xs, ys) を x で線形補間する。範囲外は端点クランプ。"""
    if not xs:
        return 1.0
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_right(xs, x)
    x0, x1 = xs[i - 1], xs[i]
    y0, y1 = ys[i - 1], ys[i]
    if x1 == x0:
        return y0
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


@dataclass(frozen=True)
class GainSchedule:
    """速度依存プラントゲイン g(v)（加速側・制動側）のテーブル。

    PID 出力の正規化に使う。速度が上がるとプラントゲイン（開度あたりの加速）が変わるため、
    固定ゲイン PID は高速域で振動・低速域で応答不足になる。各速度で「目標加速を 1 単位
    増やすのに必要な開度」= 逆ゲイン g(v) を FF モデルから抽出し、公称点との比で PID の
    実効ゲインを速度域で一定に保つ。加速フェーズは accel、減速フェーズは brake 側を使う。
    """

    speeds: tuple[float, ...]
    accel_gains: tuple[float, ...]
    brake_gains: tuple[float, ...]

    def accel_gain_at(self, v: float) -> float:
        return _interp_sorted(v, self.speeds, self.accel_gains)

    def brake_gain_at(self, v: float) -> float:
        return _interp_sorted(v, self.speeds, self.brake_gains)


class _Regressor(Protocol):
    """逆モデル推定器の構造的型（sklearn Ridge / Pipeline 等が満たす）。"""

    def predict(self, x: Any) -> Any: ...


class FeedforwardController:
    """先読み型 多項式Ridge 逆モデルによる FF 制御。load_model() 後に predict_effort() を呼ぶ。

    現在の基準速度 v0・数秒先の基準速度（先読み）・数秒前の基準速度（過去Δv＝過渡状態の
    識別）から、符号付き努力量（+: 名目アクセル開度 [%]、−: 名目ブレーキ開度 [%]）を予測する。
    追従誤差の補正は PID に、ペダルへの写像は PedalArbiter に委ねる純粋フィードフォワード。

    特徴量構成（ホライズン・レジーム判定等）は pkl に保存された `FeatureSpec` から復元する
    （モジュール定数のハードコード参照はしない）。未ロード時は `DEFAULT_FEATURE_SPEC` を使う。
    """

    def __init__(self) -> None:
        self._accel_model: _Regressor | None = None
        self._brake_model: _Regressor | None = None
        # 推論時に v0・先読み速度をこの上限へクリップして学習域外の外挿（多項式の暴れ）を防ぐ。
        # 学習データの観測最高車速。None は無制限（クリップしない）。
        self._speed_clip_max: float | None = None
        # 車両物理定数。select_profile 時に set_params で更新する。未設定時は AT 標準デフォルト。
        self._params: FeedforwardParams = FeedforwardParams()
        # 特徴量構成。load_model でロード済み pkl の feature_spec に置き換わる。
        self._spec: FeatureSpec = DEFAULT_FEATURE_SPEC
        # 速度依存プラントゲイン表（ゲインスケジューリング）。プロファイル選択時に
        # rebuild_gain_schedule で構築し、走行中は補間参照する。未構築/未ロード時 None。
        self._gain_schedule: GainSchedule | None = None

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
        """運転モデルがロード済みか。未学習（初回学習走行）では False。"""
        return self._accel_model is not None and self._brake_model is not None

    def set_params(self, params: FeedforwardParams) -> None:
        """車両プロファイルのフィードフォワード物理定数を設定する。"""
        self._params = params

    def load_model(self, model_path: str) -> None:
        """pkl ファイルから先読み 多項式Ridge 逆モデルをロードする。

        pkl は train_inverse_model が出力する dict 形式:
            model_type:     "poly_spec_inverse_lookahead"
            accel_model:    sklearn Pipeline（多項式＋標準化＋Ridge）
            brake_model:    sklearn Pipeline
            speed_clip_max: 学習観測最高車速（推論時の入力クリップ上限）
            feature_spec:   FeatureSpec を asdict() した plain dict（特徴量構成の正）
            feature_names, horizons, past_horizons, ...（参考情報。読み手は spec を正として扱う）

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

        required_keys = {"accel_model", "brake_model", "feature_spec"}
        missing = required_keys - data.keys()
        if missing:
            raise ValueError(f"Model file is missing required keys: {missing}")

        spec_dict = data["feature_spec"]
        try:
            spec = FeatureSpec(**spec_dict)
        except TypeError as e:
            raise ValueError(f"Model file has an invalid feature_spec: {model_path}") from e

        self._accel_model = data["accel_model"]
        self._brake_model = data["brake_model"]
        self._spec = spec
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
        self._spec = DEFAULT_FEATURE_SPEC
        self._gain_schedule = None

    @property
    def gain_schedule(self) -> GainSchedule | None:
        """速度依存プラントゲイン表（未構築/モデル未ロード時 None）。"""
        return self._gain_schedule

    def rebuild_gain_schedule(self, v_max: float, step: float = 5.0) -> None:
        """速度依存プラントゲイン表を再構築する。プロファイル/モデル適用時に呼ぶ。

        モデル未ロード時は None にリセットする（ゲインスケジューリング無効＝scale 1.0）。
        """
        self._gain_schedule = self.build_gain_schedule(v_max, step)

    def predict_effort(
        self, v0: float, future_speeds: Sequence[float], past_speeds: Sequence[float]
    ) -> float:
        """現在・先読み・過去の基準速度から符号付き努力量 [%] を返す。

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
        spec = self._spec

        # 1. 停車レジーム: 停車保持ブレーキ（学習モデルは原点で 0 を保証しないため定数で補う。
        #    負予測は後段で 0 にクランプされる）。判定は最短ホライズン（0.5 秒先）のみ:
        #    全先読み点（3 秒先まで）で判定すると発進の 3 秒前に保持ブレーキが解除され、
        #    クリープで基準 0 に逆らって動き出す（レビュー指摘 #5）。lookahead_horizons_s は
        #    昇順制約があるため future_speeds[0] が最短ホライズンになる。
        if v0 <= STOP_SPEED_KMH and future_speeds and future_speeds[0] <= STOP_SPEED_KMH:
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

        features = build_feature_row(v0, future_speeds, past_speeds, spec)
        # レジーム判定: 先読みトレンド dv_{regime_horizon_s} の符号
        dv_regime = float(features[0, spec.regime_col()])
        desired_accel = dv_regime / spec.regime_horizon_s  # km/h/s（正:加速, 負:減速）

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

    def _accel_features(self, v: float, a: float) -> Any:  # noqa: ANN401
        """速度 v・一定加速度 a [km/h/s] の定加速度マニフォールド上の特徴行を組む。

        future_h = v + a·h、past_h = v − a·h とする（＝「一定加速度 a で走行中」という、
        学習運転に実在する軌跡）。これにより dv_h = a·h・過去Δv が互いに整合し、共線でない
        現実的な勾配 ∂開度/∂a が得られる。旧実装の「全ホライズン一律 dv=±const」は
        「一瞬で +const して以後停滞」という学習データに無い off-manifold 点を突き、
        共線な dv 特徴群の回帰勾配が信頼できなかった（B-6 是正）。

        速度は学習域（speed_clip_max）にクリップし、負値は 0 に丸めて域外外挿を避ける。
        """
        cm = self._speed_clip_max
        v_eff = v if cm is None else min(v, cm)
        future = [v_eff + a * h for h in self._spec.lookahead_horizons_s]
        past = [v_eff - a * h for h in self._spec.past_horizons_s]
        future = [max(0.0, f if cm is None else min(f, cm)) for f in future]
        past = [max(0.0, p if cm is None else min(p, cm)) for p in past]
        return build_feature_row(v_eff, future, past, self._spec)

    def build_gain_schedule(self, v_max: float, step: float = 5.0) -> GainSchedule | None:
        """速度依存プラントゲイン g(v)（加速側・制動側）を FF モデルから抽出して返す。

        各速度 v で定加速度 a=±probe を与えた差分から
        g_accel(v) = ∂(accel開度)/∂a、g_brake(v) = ∂(brake開度)/∂(−a) を求め（単位 %/(km/h/s)）、
        [_GAIN_MIN, _GAIN_MAX] にクランプ・3点移動平均で平滑化する。モデル未ロード時は None。

        グリッド上限は `speed_clip_max − max_horizon·probe` 手前に制限する。これを超えると
        +probe の先読み速度が学習域上限にクリップされて勾配が人工的に潰れるため（B-6 是正）。
        表を超える速度は補間の端点クランプで最終ゲインを流用する。

        プロファイル選択時に一度だけ計算し、走行中は補間参照するだけにする（20Hz 制御の
        ホットパスでモデル推論を避ける）。
        """
        if self._accel_model is None or self._brake_model is None:
            return None

        a_probe = _GAIN_ACCEL_PROBE_KMHS
        cm = self._speed_clip_max
        max_h = max(self._spec.lookahead_horizons_s)
        grid_max = v_max if cm is None else min(v_max, cm - max_h * a_probe)
        grid_max = max(grid_max, step)  # 最低でも 0 と step の 2 点は作る
        speeds = [
            float(s)
            for s in range(0, int(grid_max) + int(step), int(step))
            if s <= grid_max
        ]
        accel_raw: list[float] = []
        brake_raw: list[float] = []
        for v in speeds:
            base_a = max(0.0, float(self._accel_model.predict(self._accel_features(v, 0.0))[0]))
            up_a = max(0.0, float(self._accel_model.predict(self._accel_features(v, a_probe))[0]))
            g_a = (up_a - base_a) / a_probe
            accel_raw.append(min(_GAIN_MAX, max(_GAIN_MIN, g_a)))

            base_b = max(0.0, float(self._brake_model.predict(self._accel_features(v, 0.0))[0]))
            dn_b = max(0.0, float(self._brake_model.predict(self._accel_features(v, -a_probe))[0]))
            g_b = (dn_b - base_b) / a_probe
            brake_raw.append(min(_GAIN_MAX, max(_GAIN_MIN, g_b)))

        accel_sm = _moving_average_3(accel_raw)
        brake_sm = _moving_average_3(brake_raw)
        return GainSchedule(
            speeds=tuple(speeds),
            accel_gains=tuple(accel_sm),
            brake_gains=tuple(brake_sm),
        )
