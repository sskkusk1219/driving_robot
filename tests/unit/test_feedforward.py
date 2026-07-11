import pickle
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pytest
from sklearn.linear_model import Ridge

from src.domain.control.feedforward import (
    _GAIN_ACCEL_PROBE_KMHS,
    _GAIN_MAX,
    _GAIN_MIN,
    FeedforwardController,
)
from src.domain.model_training import DEFAULT_FEATURE_SPEC, MODEL_TYPE, FeatureSpec
from src.models.profile import FeedforwardParams

FEATURE_NAMES = DEFAULT_FEATURE_SPEC.feature_names()
LOOKAHEAD_HORIZONS_S = DEFAULT_FEATURE_SPEC.lookahead_horizons_s
PAST_HORIZONS_S = DEFAULT_FEATURE_SPEC.past_horizons_s
REGIME_HORIZON_S = DEFAULT_FEATURE_SPEC.regime_horizon_s

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
    speed_clip_max: float | None = 1000.0,
    spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    include_feature_spec: bool = True,
) -> Path:
    """新フォーマット（poly_spec_inverse_lookahead）のモデル pkl を作る。

    予測ロジックの検証では推定器に決定論的な線形 Ridge（係数指定）を注入する。FF 側は
    `.predict()` を呼ぶだけのため Ridge / Pipeline いずれも受けられる。`speed_clip_max` は既定で
    十分大きく取り（=クリップ無効）、既存の予測アサーションに影響しないようにする。
    """
    n_features = len(spec.feature_names())
    # v0 係数だけ持つ単純モデル（accel: 0.5·v0, brake: 0.3·v0）
    accel_coef = accel_coef or [0.5] + [0.0] * (n_features - 1)
    brake_coef = brake_coef or [0.3] + [0.0] * (n_features - 1)

    payload = {
        "model_type": MODEL_TYPE,
        "accel_model": _ridge_with_coef(accel_coef),
        "brake_model": _ridge_with_coef(brake_coef),
        "feature_names": spec.feature_names(),
        "horizons": list(spec.lookahead_horizons_s),
        "past_horizons": list(spec.past_horizons_s),
        "regime_horizon": spec.regime_horizon_s,
        "speed_clip_max": speed_clip_max,
    }
    if include_feature_spec:
        payload["feature_spec"] = asdict(spec)
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


def _past(v0: float) -> list[float]:
    """過去方向の基準速度（平坦＝過去Δv 0。係数注入モデルは v0 のみ見るため値は影響しない）。"""
    return [v0] * len(PAST_HORIZONS_S)


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

    def test_load_missing_feature_spec_rejected(self, tmp_path: Path) -> None:
        """feature_spec の無い旧pkl（9特徴決め打ち世代）はロード時に拒否される。"""
        path = make_model_file(tmp_path, include_feature_spec=False)
        ff = FeedforwardController()
        with pytest.raises(ValueError, match="missing required keys"):
            ff.load_model(str(path))

    def test_load_invalid_feature_spec_rejected(self, tmp_path: Path) -> None:
        bad_path = tmp_path / "bad3.pkl"
        with bad_path.open("wb") as f:
            pickle.dump(
                {
                    "model_type": MODEL_TYPE,
                    "accel_model": _ridge_with_coef([0.5] + [0.0] * (N_FEATURES - 1)),
                    "brake_model": _ridge_with_coef([0.3] + [0.0] * (N_FEATURES - 1)),
                    "feature_spec": {"not_a_valid_field": 1},
                },
                f,
            )
        ff = FeedforwardController()
        with pytest.raises(ValueError, match="invalid feature_spec"):
            ff.load_model(str(bad_path))


class TestFeedforwardControllerFeatureSpecFromPkl:
    """load_model が pkl の feature_spec を復元し、horizons 等がそれに従うこと。"""

    def test_horizons_reflect_loaded_spec(self, tmp_path: Path) -> None:
        custom_spec = FeatureSpec(
            lookahead_horizons_s=(0.2, 0.5, 1.0),
            past_horizons_s=(0.2,),
            regime_horizon_s=0.5,
        )
        path = make_model_file(tmp_path, spec=custom_spec)
        ff = FeedforwardController()
        ff.load_model(str(path))
        assert ff.horizons == custom_spec.lookahead_horizons_s
        assert ff.past_horizons == custom_spec.past_horizons_s

    def test_unload_resets_to_default_spec(self, tmp_path: Path) -> None:
        custom_spec = FeatureSpec(
            lookahead_horizons_s=(0.2, 0.5, 1.0),
            past_horizons_s=(0.2,),
            regime_horizon_s=0.5,
        )
        path = make_model_file(tmp_path, spec=custom_spec)
        ff = FeedforwardController()
        ff.load_model(str(path))
        ff.unload_model()
        assert ff.horizons == DEFAULT_FEATURE_SPEC.lookahead_horizons_s
        assert ff.past_horizons == DEFAULT_FEATURE_SPEC.past_horizons_s

    def test_default_spec_before_any_load(self) -> None:
        ff = FeedforwardController()
        assert ff.horizons == DEFAULT_FEATURE_SPEC.lookahead_horizons_s

    def test_predict_with_non_default_spec_accel_regime(self, tmp_path: Path) -> None:
        """非デフォルトspec（短期3ホライズン）でも特徴構築・レジーム判定が正しく動作すること。"""
        custom_spec = FeatureSpec(
            lookahead_horizons_s=(0.2, 0.5, 1.0),
            past_horizons_s=(0.2,),
            regime_horizon_s=0.5,
        )
        n_features = len(custom_spec.feature_names())
        accel_coef = [0.5] + [0.0] * (n_features - 1)  # v0 係数のみ
        path = make_model_file(tmp_path, accel_coef=accel_coef, spec=custom_spec)
        ff = FeedforwardController()
        ff.load_model(str(path))

        v0 = 40.0
        future = [v0 + 1.0, v0 + 3.0, v0 + 6.0]  # 上昇トレンド → アクセルレジーム
        past = [v0 - 1.0]
        effort = ff.predict_effort(v0, future, past)
        assert effort == pytest.approx(0.5 * v0)

    def test_predict_with_non_default_spec_brake_regime(self, tmp_path: Path) -> None:
        custom_spec = FeatureSpec(
            lookahead_horizons_s=(0.2, 0.5, 1.0),
            past_horizons_s=(0.2,),
            regime_horizon_s=0.5,
        )
        n_features = len(custom_spec.feature_names())
        brake_coef = [0.3] + [0.0] * (n_features - 1)
        path = make_model_file(tmp_path, brake_coef=brake_coef, spec=custom_spec)
        ff = FeedforwardController()
        ff.load_model(str(path))

        v0 = 60.0
        future = [v0 - 2.0, v0 - 10.0, v0 - 20.0]  # 強い下降トレンド → ブレーキレジーム
        past = [v0 + 1.0]
        effort = ff.predict_effort(v0, future, past)
        assert effort == pytest.approx(-0.3 * v0)


class TestFeedforwardControllerPredictEffort:
    def test_predict_without_model_raises(self) -> None:
        ff = FeedforwardController()
        with pytest.raises(RuntimeError, match="Model not loaded"):
            ff.predict_effort(50.0, _rising(50.0), _past(50.0))

    def test_predict_wrong_lookahead_length_raises(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        with pytest.raises(ValueError, match="future_speeds"):
            ff.predict_effort(50.0, [55.0, 60.0], _past(50.0))  # ホライズン数と不一致

    def test_accel_regime_uses_accel_model(self, tmp_path: Path) -> None:
        """先読みが上昇 → アクセルモデルの正の努力量。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        effort = ff.predict_effort(40.0, _rising(40.0), _past(40.0))
        assert effort == pytest.approx(0.5 * 40.0)  # 0.5·v0

    def test_brake_regime_uses_brake_model(self, tmp_path: Path) -> None:
        """エンジンブレーキを超える減速 → ブレーキモデルの負の努力量。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        # dv@1.0=-10 ≫ エンジンブレーキ
        effort = ff.predict_effort(60.0, _falling(60.0), _past(60.0))
        assert effort == pytest.approx(-0.3 * 60.0)  # −0.3·v0

    def test_flat_lookahead_outputs_full_cruise_throttle(self, tmp_path: Path) -> None:
        """先読みが平坦（dv_1.0 == 0）は巡航スロットルをフルに出す（半減しない）。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        effort = ff.predict_effort(50.0, [50.0] * len(LOOKAHEAD_HORIZONS_S), _past(50.0))
        assert effort == pytest.approx(0.5 * 50.0)

    def test_regime_boundary_is_continuous(self, tmp_path: Path) -> None:
        """レビュー #9: dv=0 境界で巡航開度 → 0 へ不連続ジャンプしない。

        dv=0− の緩減速はスロットルテーパで巡航開度から漸減する。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        v0 = 50.0
        flat = ff.predict_effort(v0, [v0] * len(LOOKAHEAD_HORIZONS_S), _past(v0))
        # dv@1.0 = -0.01 km/h のごく緩い減速
        eps_down = [v0 - 0.01 * (i + 1) for i in range(len(LOOKAHEAD_HORIZONS_S))]
        near_flat = ff.predict_effort(v0, eps_down, _past(v0))
        assert abs(flat - near_flat) < 1.0  # 旧実装はここで約 25% のジャンプ

    def test_taper_reduces_throttle_with_decel_demand(self, tmp_path: Path) -> None:
        """減速要求が深まるほどスロットルが漸減し、エンジンブレーキ上限で 0 になる。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        ff.set_params(FeedforwardParams(engine_brake_decel_kmhs=2.0))
        v0 = 50.0
        # dv@1.0 = -1.0 → desired_decel=1.0 は eng=2.0 の半分 → 巡航開度の半分
        half = ff.predict_effort(v0, [v0 - 0.5, v0 - 1.0, v0 - 2.0, v0 - 3.0], _past(v0))
        assert half == pytest.approx(0.5 * 50.0 * 0.5)
        # dv@1.0 = -2.0 → テーパ終端 → 0
        end = ff.predict_effort(v0, [v0 - 1.0, v0 - 2.0, v0 - 4.0, v0 - 6.0], _past(v0))
        assert end == pytest.approx(0.0)

    def test_predict_clamps_to_100(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path, accel_coef=[1000.0] + [0.0] * (N_FEATURES - 1))
        ff = FeedforwardController()
        ff.load_model(str(path))
        assert ff.predict_effort(50.0, _rising(50.0), _past(50.0)) == 100.0

    def test_negative_accel_prediction_treated_as_zero(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path, accel_coef=[-1000.0] + [0.0] * (N_FEATURES - 1))
        ff = FeedforwardController()
        ff.load_model(str(path))
        assert ff.predict_effort(50.0, _rising(50.0), _past(50.0)) == 0.0

    def test_input_clipped_to_speed_clip_max(self, tmp_path: Path) -> None:
        """v0・先読み速度が学習観測レンジ(speed_clip_max)を超えたら端へクリップされる。"""
        # accel: 0.5·v0。clip=50 で v0=80 を入れても 50 にクリップ → 0.5·50=25。
        path = make_model_file(tmp_path, speed_clip_max=50.0)
        ff = FeedforwardController()
        ff.load_model(str(path))
        clipped = ff.predict_effort(80.0, _rising(80.0), _past(80.0))
        assert clipped == pytest.approx(0.5 * 50.0)  # 0.5·80=40 ではなく 0.5·50=25

    def test_no_clip_when_speed_clip_max_absent(self, tmp_path: Path) -> None:
        """speed_clip_max が無い payload はクリップ無効（v0 をそのまま使う）。"""
        path = make_model_file(tmp_path, speed_clip_max=None)
        ff = FeedforwardController()
        ff.load_model(str(path))
        assert ff.predict_effort(80.0, _rising(80.0), _past(80.0)) == pytest.approx(0.5 * 80.0)

    def test_horizons_property(self, tmp_path: Path) -> None:
        ff = FeedforwardController()
        assert ff.horizons == LOOKAHEAD_HORIZONS_S

    def test_unload_model_clears_state(self, tmp_path: Path) -> None:
        """レビュー #4: アンロードで has_model が False に戻り predict が拒否される。"""
        ff = FeedforwardController()
        ff.load_model(str(make_model_file(tmp_path)))
        ff.unload_model()
        assert ff.has_model is False
        with pytest.raises(RuntimeError, match="Model not loaded"):
            ff.predict_effort(50.0, _rising(50.0), _past(50.0))


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
        """基準が停車（v0・0.5 秒先が停車しきい値以下）→ 停車保持ブレーキ。"""
        ff = self._loaded(tmp_path, FeedforwardParams(stop_brake_opening_pct=22.0))
        effort = ff.predict_effort(0.0, [0.0, 0.0, 0.0, 0.0], _past(0.0))
        assert effort == pytest.approx(-22.0)

    def test_stop_brake_held_until_just_before_launch(self, tmp_path: Path) -> None:
        """レビュー #5: 発進が 2 秒先に見えても 0.5 秒先が停車なら保持ブレーキを維持する。

        旧実装は全先読み点（3 秒先まで）で判定し、発進 3 秒前に保持を解除して
        クリープで動き出していた。"""
        ff = self._loaded(tmp_path, FeedforwardParams(stop_brake_opening_pct=20.0))
        effort = ff.predict_effort(0.0, [0.0, 0.0, 2.0, 8.0], _past(0.0))
        assert effort == pytest.approx(-20.0)

    def test_stop_brake_released_when_launch_imminent(self, tmp_path: Path) -> None:
        """0.5 秒先が動き出していれば停車レジームを抜けて発進準備に入る。"""
        ff = self._loaded(tmp_path, FeedforwardParams(stop_brake_opening_pct=20.0))
        effort = ff.predict_effort(0.0, [2.0, 5.0, 10.0, 15.0], _past(0.0))
        assert effort > -20.0  # 保持ブレーキではない（発進側の出力）

    def test_creep_zone_outputs_no_pedal(self, tmp_path: Path) -> None:
        """クリープ車速以下＆要求加速がクリープ能力内 → ペダル 0。"""
        ff = self._loaded(tmp_path, FeedforwardParams(creep_speed_kmh=10.0, creep_rate_kmhs=1.0))
        # v0=5(<10), 先読みで +0.5km/h/s 程度（dv@1.0=0.5 → desired_accel=0.5 <= 1.0）
        effort = ff.predict_effort(5.0, [5.25, 5.5, 6.0, 6.5], _past(5.0))
        assert effort == 0.0

    def test_low_speed_gentle_decel_keeps_brake(self, tmp_path: Path) -> None:
        """レビュー #8: クリープ速度未満の緩減速はブレーキを残す。

        旧実装は abs() 判定でペダルオフにし、クリープ（+0.5km/h/s）が基準の減速と
        逆方向に車を押して誤差が成長していた。"""
        ff = self._loaded(tmp_path, FeedforwardParams(creep_speed_kmh=7.0, creep_rate_kmhs=0.5))
        # v0=5(<7), dv@1.0=-0.4 → 緩い減速要求
        effort = ff.predict_effort(5.0, [4.8, 4.6, 4.2, 3.8], _past(5.0))
        assert effort == pytest.approx(-0.3 * 5.0)  # ブレーキモデル出力（負）

    def test_gentle_decel_within_engine_brake_tapers_throttle(self, tmp_path: Path) -> None:
        """巡航速度の緩減速（エンジンブレーキ内）はブレーキではなくスロットルテーパ。"""
        ff = self._loaded(tmp_path, FeedforwardParams(engine_brake_decel_kmhs=2.0))
        # v0=50, dv@1.0=-0.5 → desired_decel=0.5、テーパ率 1-0.5/2=0.75
        effort = ff.predict_effort(50.0, [49.75, 49.5, 49.0, 48.5], _past(50.0))
        assert effort == pytest.approx(0.5 * 50.0 * 0.75)
        assert effort > 0.0  # ブレーキは出さない

    def test_no_deadband_snap_in_ff(self, tmp_path: Path) -> None:
        """レビュー #11: 不感帯処理は FF では行わない（PedalArbiter が合成後に逆補償）。"""
        ff = FeedforwardController()
        path = make_model_file(tmp_path, brake_coef=[0.05] + [0.0] * (N_FEATURES - 1))
        ff.load_model(str(path))
        ff.set_params(FeedforwardParams(brake_deadband_pct=10.0, engine_brake_decel_kmhs=0.0))
        # 強い減速予見 → ブレーキ予測 0.05*60=3% は不感帯 10% 未満でもそのまま返す
        effort = ff.predict_effort(60.0, [50.0, 40.0, 20.0, 0.0], _past(60.0))
        assert effort == pytest.approx(-3.0)


class TestBuildGainSchedule:
    """Stage B: 速度依存プラントゲイン表を FF モデルの局所勾配から抽出する。"""

    @staticmethod
    def _coef(**name_to_val: float) -> list[float]:
        """feature名→係数の指定から係数ベクトルを作る（キーの _ は . に読み替える）。"""
        coef = [0.0] * N_FEATURES
        for key, val in name_to_val.items():
            name = key.replace("dv_0_5", "dv_0.5").replace("dv_1_0", "dv_1.0")
            coef[FEATURE_NAMES.index(name)] = val
        return coef

    def test_no_model_returns_none(self) -> None:
        ff = FeedforwardController()
        assert ff.build_gain_schedule(v_max=100.0) is None
        assert ff.gain_schedule is None

    def test_constant_accel_gradient(self, tmp_path: Path) -> None:
        """dv_1.0 係数のみ 0.2 のモデル → g_accel(v)=0.2（速度非依存, 平滑化後も一定）。"""
        path = make_model_file(
            tmp_path, accel_coef=self._coef(dv_1_0=0.2), speed_clip_max=None
        )
        ff = FeedforwardController()
        ff.load_model(str(path))
        sched = ff.build_gain_schedule(v_max=100.0, step=5.0)
        assert sched is not None
        assert all(g == pytest.approx(0.2, abs=1e-6) for g in sched.accel_gains)

    def test_zero_gradient_clamped_to_min(self, tmp_path: Path) -> None:
        """既定モデル（v0 係数のみ）は dv 勾配 0 → _GAIN_MIN にクランプされる。"""
        path = make_model_file(tmp_path, speed_clip_max=None)
        ff = FeedforwardController()
        ff.load_model(str(path))
        sched = ff.build_gain_schedule(v_max=60.0)
        assert sched is not None
        assert all(g == pytest.approx(_GAIN_MIN) for g in sched.accel_gains)

    def test_large_gradient_clamped_to_max(self, tmp_path: Path) -> None:
        accel_coef = [0.0] * N_FEATURES
        accel_coef[FEATURE_NAMES.index("dv_1.0")] = 10.0  # g=10 → clamp 1.0
        path = make_model_file(tmp_path, accel_coef=accel_coef, speed_clip_max=None)
        ff = FeedforwardController()
        ff.load_model(str(path))
        sched = ff.build_gain_schedule(v_max=40.0)
        assert sched is not None
        assert all(g == pytest.approx(_GAIN_MAX) for g in sched.accel_gains)

    def test_brake_gradient_from_negative_dv(self, tmp_path: Path) -> None:
        """dv_1.0 係数 −0.3 のブレーキモデル → g_brake(v)=0.3（定減速度 −a 方向の勾配）。"""
        brake_coef = [0.0] * N_FEATURES
        brake_coef[FEATURE_NAMES.index("v0")] = 0.3  # 予測を非負に保つ土台
        brake_coef[FEATURE_NAMES.index("dv_1.0")] = -0.3
        path = make_model_file(tmp_path, brake_coef=brake_coef, speed_clip_max=None)
        ff = FeedforwardController()
        ff.load_model(str(path))
        sched = ff.build_gain_schedule(v_max=80.0)
        assert sched is not None
        # 低速端は future の 0 丸めで勾配が変わり、3点移動平均で1格子点分ズレが波及する。
        # 標準的な速度域（v>=10）では純粋な線形勾配 0.3 になる。
        for v, g in zip(sched.speeds, sched.brake_gains, strict=False):
            if v >= 10.0:
                assert g == pytest.approx(0.3, abs=1e-6)

    def test_speed_dependent_gradient(self, tmp_path: Path) -> None:
        """dv1_x_v0 係数で g(v) が速度依存になる: g(v)=0.1+0.002·v（端点以外は厳密）。"""
        accel_coef = [0.0] * N_FEATURES
        accel_coef[FEATURE_NAMES.index("dv_1.0")] = 0.1
        accel_coef[FEATURE_NAMES.index("dv1_x_v0")] = 0.002
        path = make_model_file(tmp_path, accel_coef=accel_coef, speed_clip_max=None)
        ff = FeedforwardController()
        ff.load_model(str(path))
        sched = ff.build_gain_schedule(v_max=100.0, step=5.0)
        assert sched is not None
        # 内部点は3点移動平均でも線形関数なら不変（等間隔サンプル）
        for v, g in zip(sched.speeds[1:-1], sched.accel_gains[1:-1], strict=False):
            assert g == pytest.approx(0.1 + 0.002 * v, abs=1e-6)

    def test_rebuild_and_unload(self, tmp_path: Path) -> None:
        accel_coef = [0.0] * N_FEATURES
        accel_coef[FEATURE_NAMES.index("dv_1.0")] = 0.2
        path = make_model_file(tmp_path, accel_coef=accel_coef, speed_clip_max=None)
        ff = FeedforwardController()
        ff.load_model(str(path))
        ff.rebuild_gain_schedule(v_max=50.0)
        assert ff.gain_schedule is not None
        ff.unload_model()
        assert ff.gain_schedule is None

    def test_probe_constant_is_positive(self) -> None:
        assert _GAIN_ACCEL_PROBE_KMHS > 0.0

    def test_grid_excludes_clip_boundary(self, tmp_path: Path) -> None:
        """speed_clip_max 近傍は +probe の先読みがクリップされ勾配が潰れるため、グリッド上限は
        speed_clip_max − max_horizon·a_probe 手前に制限される（超過速度は補間端点クランプ）。"""
        accel_coef = [0.0] * N_FEATURES
        accel_coef[FEATURE_NAMES.index("dv_1.0")] = 0.2
        path = make_model_file(tmp_path, accel_coef=accel_coef, speed_clip_max=100.0)
        ff = FeedforwardController()
        ff.load_model(str(path))
        sched = ff.build_gain_schedule(v_max=140.0, step=5.0)
        assert sched is not None
        # 100 − 3.0·0.5 = 98.5 → 最大格子点は 95。クリップ域(100超)を含まず v_max(140)にも達しない。
        assert max(sched.speeds) <= 100.0 - 3.0 * _GAIN_ACCEL_PROBE_KMHS
        assert max(sched.speeds) < 140.0
