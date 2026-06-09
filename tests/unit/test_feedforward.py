import pickle
from pathlib import Path

import numpy as np
import pytest
from sklearn.linear_model import Ridge

from src.domain.control.feedforward import FeedforwardController
from src.domain.model_training import (
    FEATURE_NAMES,
    LOOKAHEAD_HORIZONS_S,
    MODEL_TYPE,
    REGIME_HORIZON_S,
)
from src.models.profile import FeedforwardParams

N_FEATURES = len(FEATURE_NAMES)
# horizons 内で REGIME_HORIZON_S が何番目か（future_speeds のレジーム判定インデックス）
REGIME_IDX = LOOKAHEAD_HORIZONS_S.index(REGIME_HORIZON_S)


def _ridge_with_coef(coef: list[float]) -> Ridge:
    """指定係数を持つ fit_intercept=False の Ridge を作る（predict を決定論的にするため）。"""
    model = Ridge(alpha=1.0, fit_intercept=False)
    model.fit(np.eye(len(coef)), np.array(coef, dtype=float))
    model.coef_ = np.array(coef, dtype=float)
    model.intercept_ = 0.0
    return model


def make_model_file(
    tmp_path: Path,
    accel_coef: list[float] | None = None,
    brake_coef: list[float] | None = None,
) -> Path:
    """新フォーマット（ridge_inverse_lookahead）のモデル pkl を作る。"""
    # v0 係数だけ持つ単純モデル（accel: 0.5·v0, brake: 0.3·v0）
    accel_coef = accel_coef or [0.5] + [0.0] * (N_FEATURES - 1)
    brake_coef = brake_coef or [0.3] + [0.0] * (N_FEATURES - 1)

    payload = {
        "model_type": MODEL_TYPE,
        "accel_model": _ridge_with_coef(accel_coef),
        "brake_model": _ridge_with_coef(brake_coef),
        "feature_names": FEATURE_NAMES,
        "horizons": list(LOOKAHEAD_HORIZONS_S),
        "regime_horizon": REGIME_HORIZON_S,
    }
    path = tmp_path / "test_model.pkl"
    with path.open("wb") as f:
        pickle.dump(payload, f)
    return path


def _rising(v0: float) -> list[float]:
    """加速レジーム（先読み速度が上昇）の future_speeds。"""
    return [v0 + (i + 1) * 5.0 for i in range(len(LOOKAHEAD_HORIZONS_S))]


def _falling(v0: float) -> list[float]:
    """減速レジーム（先読み速度が下降）の future_speeds。"""
    return [v0 - (i + 1) * 5.0 for i in range(len(LOOKAHEAD_HORIZONS_S))]


class TestFeedforwardControllerHasModel:
    def test_has_model_false_before_load(self) -> None:
        ff = FeedforwardController()
        assert ff.has_model is False

    def test_has_model_true_after_load(self, tmp_path: Path) -> None:
        ff = FeedforwardController()
        ff.load_model(str(make_model_file(tmp_path)))
        assert ff.has_model is True


class TestFeedforwardControllerLoadModel:
    def test_load_valid_model(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        assert ff._accel_model is not None
        assert ff._brake_model is not None

    def test_load_nonexistent_file(self) -> None:
        ff = FeedforwardController()
        with pytest.raises(FileNotFoundError):
            ff.load_model("/nonexistent/path/model.pkl")

    def test_load_wrong_model_type_raises(self, tmp_path: Path) -> None:
        bad_path = tmp_path / "bad.pkl"
        with bad_path.open("wb") as f:
            pickle.dump({"model_type": "grid", "speed_grid": np.array([0.0])}, f)
        ff = FeedforwardController()
        with pytest.raises(ValueError, match="Unsupported model file"):
            ff.load_model(str(bad_path))

    def test_load_missing_keys_raises(self, tmp_path: Path) -> None:
        bad_path = tmp_path / "bad2.pkl"
        with bad_path.open("wb") as f:
            pickle.dump({"model_type": MODEL_TYPE}, f)
        ff = FeedforwardController()
        with pytest.raises(ValueError, match="missing required keys"):
            ff.load_model(str(bad_path))


class TestFeedforwardControllerPredict:
    def test_predict_without_model_raises(self) -> None:
        ff = FeedforwardController()
        with pytest.raises(RuntimeError, match="Model not loaded"):
            ff.predict(50.0, _rising(50.0))

    def test_predict_wrong_lookahead_length_raises(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        with pytest.raises(ValueError, match="future_speeds"):
            ff.predict(50.0, [55.0, 60.0])  # ホライズン数と不一致

    def test_accel_regime_uses_accel_model(self, tmp_path: Path) -> None:
        """先読みが上昇 → アクセル側を予測し、ブレーキは 0。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        accel, brake = ff.predict(40.0, _rising(40.0))
        assert accel == pytest.approx(0.5 * 40.0)  # 0.5·v0
        assert brake == 0.0

    def test_brake_regime_uses_brake_model(self, tmp_path: Path) -> None:
        """先読みが下降 → ブレーキ側を予測し、アクセルは 0。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        accel, brake = ff.predict(60.0, _falling(60.0))
        assert accel == 0.0
        assert brake == pytest.approx(0.3 * 60.0)  # 0.3·v0

    def test_flat_lookahead_is_accel_regime(self, tmp_path: Path) -> None:
        """先読みが平坦（dv_1.0 == 0）はアクセルレジーム（>=0）。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        accel, brake = ff.predict(50.0, [50.0] * len(LOOKAHEAD_HORIZONS_S))
        assert brake == 0.0
        assert accel == pytest.approx(0.5 * 50.0)

    def test_predict_clamps_to_100(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path, accel_coef=[1000.0] + [0.0] * (N_FEATURES - 1))
        ff = FeedforwardController()
        ff.load_model(str(path))
        accel, _ = ff.predict(50.0, _rising(50.0))
        assert accel == 100.0

    def test_predict_clamps_to_zero(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path, accel_coef=[-1000.0] + [0.0] * (N_FEATURES - 1))
        ff = FeedforwardController()
        ff.load_model(str(path))
        accel, brake = ff.predict(50.0, _rising(50.0))
        assert accel == 0.0
        assert brake >= 0.0

    def test_predict_returns_tuple(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        result = ff.predict(50.0, _rising(50.0))
        assert isinstance(result, tuple)
        assert len(result) == 2

    def test_horizons_property(self, tmp_path: Path) -> None:
        ff = FeedforwardController()
        assert ff.horizons == LOOKAHEAD_HORIZONS_S


class TestFeedforwardControllerParams:
    def _loaded(self, tmp_path: Path, params: FeedforwardParams | None = None):
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        if params is not None:
            ff.set_params(params)
        return ff

    def test_default_params_are_at_standard(self) -> None:
        ff = FeedforwardController()
        # 既定値が反映されること（set_params 前でも停車レジームが機能する）
        assert ff._params == FeedforwardParams()

    def test_stop_regime_applies_stop_brake(self, tmp_path: Path) -> None:
        """基準が停車（v0・先読みが停車しきい値以下）→ 停車保持ブレーキ、アクセル0。"""
        ff = self._loaded(tmp_path, FeedforwardParams(stop_brake_opening_pct=22.0))
        accel, brake = ff.predict(0.0, [0.0, 0.0, 0.0, 0.0])
        assert accel == 0.0
        assert brake == pytest.approx(22.0)

    def test_creep_zone_outputs_no_pedal(self, tmp_path: Path) -> None:
        """クリープ車速以下＆要求加速がクリープ能力内 → ペダル 0。"""
        ff = self._loaded(tmp_path, FeedforwardParams(creep_speed_kmh=10.0, creep_rate_kmhs=1.0))
        # v0=5(<10), 先読みで +0.5km/h/s 程度（dv@1.0=0.5 → desired_accel=0.5 <= 1.0）
        accel, brake = ff.predict(5.0, [5.25, 5.5, 6.0, 6.5])
        assert accel == 0.0
        assert brake == 0.0

    def test_gentle_decel_within_engine_brake_no_brake(self, tmp_path: Path) -> None:
        """緩い減速がエンジンブレーキ範囲内 → ブレーキ 0。"""
        ff = self._loaded(
            tmp_path, FeedforwardParams(engine_brake_decel_kmhs=2.0, brake_deadband_pct=0.0)
        )
        # v0=50, dv@1.0 = -0.5 → desired_decel=0.5 <= 2.0 → ブレーキ不要
        accel, brake = ff.predict(50.0, [49.75, 49.5, 49.0, 48.5])
        assert accel == 0.0
        assert brake == 0.0

    def test_brake_deadband_snaps_small_brake_to_zero(self, tmp_path: Path) -> None:
        """ブレーキ不感帯未満の微小ブレーキは 0 にスナップされる。"""
        # 小さなブレーキ係数のモデル + 大きめ不感帯
        ff = FeedforwardController()
        path = make_model_file(tmp_path, brake_coef=[0.05] + [0.0] * (N_FEATURES - 1))
        ff.load_model(str(path))
        ff.set_params(FeedforwardParams(brake_deadband_pct=10.0, engine_brake_decel_kmhs=0.0))
        # 強い減速予見でブレーキレジーム、ただし予測は 0.05*60=3% < 不感帯10% → 0
        accel, brake = ff.predict(60.0, [50.0, 40.0, 20.0, 0.0])
        assert brake == 0.0
