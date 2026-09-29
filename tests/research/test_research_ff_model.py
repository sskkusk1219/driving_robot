"""ff_model（特徴量の構成・pkl の読み込み）のテスト。

既定 spec が本番の9特徴と一致すること（回帰）と、新規2項目
（dv_excluded_horizons_s・past_as_delta）の挙動を固定する。
"""

from __future__ import annotations

import pickle
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pytest

from tests.research.config import FeaturesSection
from tests.research.ff_model import (
    DEFAULT_FEATURE_SPEC,
    MODEL_TYPE,
    FeatureSpec,
    FeedforwardModel,
    build_feature_matrix,
    build_feature_row,
    deviation_gain,
    export_model_coefficients,
    make_estimator,
    raw_unit_coefficients,
)
from tests.research.research_types import FeedforwardParams

FUTURE = [33.0, 36.0, 42.0, 48.0]  # v0=30 → dv = 3, 6, 12, 18
PAST = [29.0, 28.0]  # v0=30 → dv_past = 1, 2


def test_default_spec_is_production_nine_features() -> None:
    assert DEFAULT_FEATURE_SPEC.feature_names() == [
        "v0", "dv_0.5", "dv_1.0", "dv_2.0", "dv_3.0",
        "v0_sq", "dv1_x_v0", "dv_past_0.5", "dv_past_1.0",
    ]  # fmt: skip
    row = build_feature_row(30.0, FUTURE, PAST)
    assert row.shape == (1, 9)
    assert row[0].tolist() == [30.0, 3.0, 6.0, 12.0, 18.0, 900.0, 180.0, 1.0, 2.0]
    assert DEFAULT_FEATURE_SPEC.regime_col() == 2


def test_excluding_h0_drops_only_its_dv_column() -> None:
    spec = FeatureSpec(dv_excluded_horizons_s=(0.5,))
    assert spec.feature_names() == [
        "v0", "dv_1.0", "dv_2.0", "dv_3.0", "v0_sq", "dv1_x_v0", "dv_past_0.5", "dv_past_1.0",
    ]  # fmt: skip
    assert spec.regime_col() == 1
    row = build_feature_row(30.0, FUTURE, PAST, spec)
    assert row[0].tolist() == [30.0, 6.0, 12.0, 18.0, 900.0, 180.0, 1.0, 2.0]
    # future_speeds の長さは除外しても4のまま（停車保持の判定が future[0] を使うため）
    assert spec.lookahead_horizons_s == (0.5, 1.0, 2.0, 3.0)


def test_past_as_delta_false_uses_raw_past_speeds() -> None:
    spec = FeatureSpec(dv_excluded_horizons_s=(0.5,), past_as_delta=False)
    assert spec.feature_names()[-2:] == ["past_0.5", "past_1.0"]
    row = build_feature_row(30.0, FUTURE, PAST, spec)
    assert row[0, -2:].tolist() == [29.0, 28.0]


def test_matrix_matches_row_for_every_spec() -> None:
    """学習用の行列と推論用の1行が、同じ spec で同じ値になる（列の食い違い防止）。"""
    rng = np.random.default_rng(0)
    speed = np.cumsum(rng.normal(0.0, 0.3, 200)) + 60.0
    offsets, past_offsets = [5, 10, 20, 30], [5, 10]
    for spec in (
        DEFAULT_FEATURE_SPEC,
        FeatureSpec(dv_excluded_horizons_s=(0.5,)),
        FeatureSpec(dv_excluded_horizons_s=(0.5,), past_as_delta=False),
        FeatureSpec(dv_excluded_horizons_s=(0.5, 3.0), past_horizons_s=(0.5, 1.0)),
    ):
        x, idx = build_feature_matrix(speed, offsets, past_offsets, spec)
        assert x.shape[1] == len(spec.feature_names())
        i = int(idx[10])
        row = build_feature_row(
            float(speed[i]),
            [float(speed[i + o]) for o in offsets],
            [float(speed[i - o]) for o in past_offsets],
            spec,
        )
        assert x[10] == pytest.approx(row[0])


@pytest.mark.parametrize(
    "kwargs",
    [
        {"dv_excluded_horizons_s": (1.0,)},  # レジーム判定のホライズンは外せない
        {"dv_excluded_horizons_s": (4.0,)},  # 先読みに無いホライズン
        {
            "dv_excluded_horizons_s": (2.0,),
            "accel_horizons_s": (2.0,),
            "past_horizons_s": (0.5, 2.0),
        },  # 加速度項に使うホライズンは外せない
    ],
)
def test_invalid_exclusion_is_rejected(kwargs: dict) -> None:
    with pytest.raises(ValueError, match="dv_excluded_horizons_s"):
        FeatureSpec(**kwargs)


def _save_pkl(path: Path, spec_dict: dict) -> None:
    accel, brake = make_estimator(), make_estimator()
    n = len(FeatureSpec(**spec_dict).feature_names())
    x = np.random.default_rng(1).normal(size=(30, n))
    accel.fit(x, x[:, 0])
    brake.fit(x, x[:, 0])
    payload = {
        "model_type": MODEL_TYPE,
        "accel_model": accel,
        "brake_model": brake,
        "feature_spec": spec_dict,
        "speed_clip_max": 100.0,
    }
    with path.open("wb") as f:
        pickle.dump(payload, f)


def test_load_model_reads_old_pkl_without_new_spec_fields(tmp_path: Path) -> None:
    """本番で作った旧 pkl（feature_spec に新規2項目が無い）は既定値で補って読める。"""
    old = {k: v for k, v in asdict(DEFAULT_FEATURE_SPEC).items()
           if k not in ("dv_excluded_horizons_s", "past_as_delta")}  # fmt: skip
    path = tmp_path / "old.pkl"
    _save_pkl(path, old)
    model = FeedforwardModel()
    model.load_model(str(path))
    assert model.has_model
    assert model.spec == DEFAULT_FEATURE_SPEC
    assert model.horizons == (0.5, 1.0, 2.0, 3.0)


def test_load_model_round_trips_new_spec(tmp_path: Path) -> None:
    spec = FeatureSpec(dv_excluded_horizons_s=(0.5,), past_as_delta=False)
    path = tmp_path / "new.pkl"
    _save_pkl(path, asdict(spec))
    model = FeedforwardModel()
    model.load_model(str(path))
    assert model.spec == spec


def test_load_model_rejects_other_model_type(tmp_path: Path) -> None:
    path = tmp_path / "bad.pkl"
    with path.open("wb") as f:
        pickle.dump({"model_type": "other"}, f)
    with pytest.raises(ValueError, match="Unsupported model file"):
        FeedforwardModel().load_model(str(path))


# ── ペダル別ホライズン（手順6。ProblemReport_20260921）──────────────────────


def _save_pedal_separated_pkl(
    path: Path,
    accel_spec: FeatureSpec,
    brake_spec: FeatureSpec,
    control_spec: FeatureSpec,
    stop_horizon_s: float,
) -> None:
    accel = make_estimator()
    brake = make_estimator()
    rng = np.random.default_rng(2)
    xa = rng.normal(size=(30, len(accel_spec.feature_names())))
    xb = rng.normal(size=(30, len(brake_spec.feature_names())))
    accel.fit(xa, xa[:, 0])
    brake.fit(xb, xb[:, 0])
    payload = {
        "model_type": MODEL_TYPE,
        "accel_model": accel,
        "brake_model": brake,
        "feature_spec": asdict(control_spec),
        "accel_feature_spec": asdict(accel_spec),
        "brake_feature_spec": asdict(brake_spec),
        "stop_horizon_s": stop_horizon_s,
        "speed_clip_max": 100.0,
    }
    with path.open("wb") as f:
        pickle.dump(payload, f)


def test_load_model_reads_pedal_separated_specs(tmp_path: Path) -> None:
    accel_spec = FeatureSpec(lookahead_horizons_s=(0.5, 1.0, 2.0, 3.0))
    brake_spec = FeatureSpec(lookahead_horizons_s=(0.1, 1.0))
    control_spec = FeatureSpec(lookahead_horizons_s=(0.1, 0.5, 1.0, 2.0, 3.0))
    path = tmp_path / "separated.pkl"
    _save_pedal_separated_pkl(path, accel_spec, brake_spec, control_spec, stop_horizon_s=0.5)

    model = FeedforwardModel()
    model.load_model(str(path))

    assert model.accel_spec == accel_spec
    assert model.brake_spec == brake_spec
    assert model.is_pedal_separated
    assert model.stop_horizon_s == 0.5
    assert model.horizons == control_spec.lookahead_horizons_s


def test_load_model_old_pkl_is_not_pedal_separated(tmp_path: Path) -> None:
    """`accel_feature_spec`/`brake_feature_spec` が無い旧 pkl は両ペダルとも `feature_spec`。"""
    path = tmp_path / "old.pkl"
    _save_pkl(path, asdict(DEFAULT_FEATURE_SPEC))

    model = FeedforwardModel()
    model.load_model(str(path))

    assert not model.is_pedal_separated
    assert model.accel_spec == model.brake_spec == DEFAULT_FEATURE_SPEC
    assert model.stop_horizon_s == DEFAULT_FEATURE_SPEC.lookahead_horizons_s[0]


def test_load_model_rejects_mismatched_regime_horizon(tmp_path: Path) -> None:
    accel_spec = FeatureSpec(lookahead_horizons_s=(1.0, 2.0), regime_horizon_s=1.0)
    brake_spec = FeatureSpec(lookahead_horizons_s=(1.5,), regime_horizon_s=1.5)
    control_spec = FeatureSpec(lookahead_horizons_s=(1.0, 1.5, 2.0), regime_horizon_s=1.0)
    path = tmp_path / "mismatched_regime.pkl"
    _save_pedal_separated_pkl(path, accel_spec, brake_spec, control_spec, stop_horizon_s=1.0)

    with pytest.raises(ValueError, match="regime_horizon_s"):
        FeedforwardModel().load_model(str(path))


def test_load_model_rejects_stop_horizon_not_in_control_spec(tmp_path: Path) -> None:
    accel_spec = FeatureSpec(lookahead_horizons_s=(1.0, 2.0))
    brake_spec = FeatureSpec(lookahead_horizons_s=(1.0,))
    control_spec = FeatureSpec(lookahead_horizons_s=(1.0, 2.0))
    path = tmp_path / "bad_stop_horizon.pkl"
    # stop_horizon_s=0.5 は control_spec.lookahead_horizons_s に含まれない
    _save_pedal_separated_pkl(path, accel_spec, brake_spec, control_spec, stop_horizon_s=0.5)

    with pytest.raises(ValueError, match="stop_horizon_s"):
        FeedforwardModel().load_model(str(path))


def test_pkl_is_pedal_separated_and_require_single_spec_pkl() -> None:
    from tests.research.ff_model import pkl_is_pedal_separated, require_single_spec_pkl

    same = {"accel_feature_spec": {"a": 1}, "brake_feature_spec": {"a": 1}}
    different = {"accel_feature_spec": {"a": 1}, "brake_feature_spec": {"a": 2}}
    legacy = {"feature_spec": {"a": 1}}

    assert not pkl_is_pedal_separated(same)
    assert pkl_is_pedal_separated(different)
    assert not pkl_is_pedal_separated(legacy)

    require_single_spec_pkl(same, "test_tool")  # 例外にならない
    require_single_spec_pkl(legacy, "test_tool")  # 例外にならない
    with pytest.raises(ValueError, match="test_tool"):
        require_single_spec_pkl(different, "test_tool")


def test_features_section_default_matches_production_spec() -> None:
    assert FeaturesSection().to_feature_spec() == DEFAULT_FEATURE_SPEC


def test_features_section_case_a() -> None:
    """案A: dv_0.5 を外し、過去は素の past_speeds。"""
    spec = FeaturesSection(use_h0=False, past_as_delta=False).to_feature_spec()
    assert spec == FeatureSpec(dv_excluded_horizons_s=(0.5,), past_as_delta=False)
    assert spec.lookahead_horizons_s[0] == 0.5  # 停車保持の判定は 0.5s 先のまま


def test_features_section_drops_unused_past_horizon() -> None:
    spec = FeaturesSection(use_p1=False).to_feature_spec()
    assert spec.past_horizons_s == (1.0,)
    assert spec.feature_names()[-1] == "dv_past_1.0"


# ── predict_effort（本番 FeedforwardController の移植）────────────────────────


class _Linear:
    """特徴量の線形和を返す決定的な偽の回帰器（sklearn を使わず期待値を固定するため）。"""

    def __init__(self, weights: np.ndarray, bias: float) -> None:
        self._w = np.asarray(weights, dtype=float)
        self._b = bias

    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.array([float(np.asarray(x)[0] @ self._w) + self._b])


# (v0, 加速度 a [km/h/s], 期待 effort [%])。2026-09-25 に本番 FeedforwardController へ
# 同じ偽回帰器・同じ入力を通して採取した固定値（停車保持・クリープ・惰行テーパ・制動・
# 学習域クリップの各分岐を通る）。tests/ だけで完結させるため src は呼ばない
# （ProblemReport_20260924）。
PROD_PREDICT_EFFORT = [
    (0.0, -4.0, -20.0), (0.0, -1.5, -20.0), (0.0, -0.5, -20.0), (0.0, 0.0, -20.0),
    (0.0, 0.3, 0.0), (0.0, 1.0, 3.0135), (0.0, 3.0, 3.0405), (2.0, -4.0, -1.9886),
    (2.0, -1.5, -1.99505), (2.0, -0.5, -2.0033), (2.0, 0.0, 0.0), (2.0, 0.3, 0.0),
    (2.0, 1.0, 3.04), (2.0, 3.0, 3.0895), (5.0, -4.0, -1.9934), (5.0, -1.5, -2.01875),
    (5.0, -0.5, -2.03525), (5.0, 0.0, 0.0), (5.0, 0.3, 0.0), (5.0, 1.0, 3.115),
    (5.0, 3.0, 3.19), (30.0, -4.0, -3.005), (30.0, -1.5, -3.1025), (30.0, -0.5, 3.297292),
    (30.0, 0.0, 5.715), (30.0, 0.3, 5.7525), (30.0, 1.0, 5.84), (30.0, 3.0, 6.09),
    (60.0, -4.0, -6.218), (60.0, -1.5, 0.0), (60.0, -0.5, 9.143333), (60.0, 0.0, 13.83),
    (60.0, 0.3, 13.899), (60.0, 1.0, 14.06), (60.0, 3.0, 14.52), (140.0, -4.0, -19.1264),
    (140.0, -1.5, 10.1355), (140.0, -0.5, 34.106111), (140.0, 0.0, 46.26), (140.0, 0.3, 46.26195),
    (140.0, 1.0, 46.2665), (140.0, 3.0, 46.2795),
]


def test_predict_effort_matches_production_values() -> None:
    ff = FeedforwardModel()
    n = len(ff.spec.feature_names())
    ff._accel_model = _Linear(np.arange(1, n + 1) * 0.0005, 3.0)
    ff._brake_model = _Linear(np.arange(n, 0, -1) * 0.0003, 2.0)
    ff._speed_clip_max = 120.0
    ff.set_params(
        FeedforwardParams(
            coast_decel_speeds_kmh=(10.0, 60.0, 130.0), coast_decel_kmhs=(1.0, 1.5, 2.0)
        )
    )
    for v0, a, expected in PROD_PREDICT_EFFORT:
        future = [max(0.0, v0 + a * h) for h in ff.horizons]
        past = [max(0.0, v0 - a * h) for h in ff.past_horizons]
        assert ff.predict_effort(v0, future, past) == pytest.approx(expected, abs=1e-5), (v0, a)


def test_predict_effort_uses_pedal_specific_feature_specs() -> None:
    """アクセル・ブレーキで異なる spec（手順6）でも、それぞれ自分の spec の行が渡ること。

    accel_spec は past_0.5・brake_spec は past_1.0 しか持たないので、モデルの列数が
    それぞれの spec の feature_names() と一致していないと predict 自体が形状エラーになる
    （＝この test が通ること自体が「正しい spec で行を組んでいる」ことの検算）。
    """
    accel_spec = FeatureSpec(lookahead_horizons_s=(1.0, 2.0), past_horizons_s=(0.5,))
    brake_spec = FeatureSpec(lookahead_horizons_s=(0.5, 1.0), past_horizons_s=(1.0,))
    control_spec = FeatureSpec(lookahead_horizons_s=(0.5, 1.0, 2.0), past_horizons_s=(0.5, 1.0))

    ff = FeedforwardModel()
    ff._accel_model = _Linear(np.full(len(accel_spec.feature_names()), 0.01), 1.0)  # noqa: SLF001
    ff._brake_model = _Linear(np.full(len(brake_spec.feature_names()), 0.01), 1.0)  # noqa: SLF001
    ff._spec = control_spec  # noqa: SLF001
    ff._accel_spec = accel_spec  # noqa: SLF001
    ff._brake_spec = brake_spec  # noqa: SLF001
    ff._stop_horizon_s = 0.5  # noqa: SLF001
    ff.set_params(FeedforwardParams())  # 既定値（coast_decel 未同定 → engine_brake_decel_kmhs=1.0）

    v0 = 40.0
    future = {0.5: 41.0, 1.0: 45.0, 2.0: 50.0}  # dv_1.0 = +5（アクセル・レジーム）
    past = {0.5: 38.0, 1.0: 36.0}
    future_list = [future[h] for h in control_spec.lookahead_horizons_s]
    past_list = [past[h] for h in control_spec.past_horizons_s]

    expected_accel_row = build_feature_row(
        v0,
        [future[h] for h in accel_spec.lookahead_horizons_s],
        [past[h] for h in accel_spec.past_horizons_s],
        accel_spec,
    )
    expected_accel_pred = float(ff._accel_model.predict(expected_accel_row)[0])  # noqa: SLF001
    assert ff.predict_effort(v0, future_list, past_list) == pytest.approx(expected_accel_pred)

    future[1.0] = 35.0  # dv_1.0 = -5（ブレーキ・レジーム。engine_brake 1.0 より強い減速要求）
    future_list = [future[h] for h in control_spec.lookahead_horizons_s]
    expected_brake_row = build_feature_row(
        v0,
        [future[h] for h in brake_spec.lookahead_horizons_s],
        [past[h] for h in brake_spec.past_horizons_s],
        brake_spec,
    )
    expected_brake_pred = float(ff._brake_model.predict(expected_brake_row)[0])  # noqa: SLF001
    assert ff.predict_effort(v0, future_list, past_list) == pytest.approx(-expected_brake_pred)


# ─────────────────────────────────────────────────────────────────────
# deviation_gain（手順6 段2。ProblemReport_20260921 6-7/6-8。2026-09-28）
# ─────────────────────────────────────────────────────────────────────


class _BatchLinear:
    """`_Linear` と違い複数行を一度に受けられる偽の回帰器（`deviation_gain` は中心差分で
    2 行まとめて `.predict` するため）。"""

    def __init__(self, weights: np.ndarray, bias: float = 0.0) -> None:
        self._w = np.asarray(weights, dtype=float)
        self._b = bias

    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.asarray(x, dtype=float) @ self._w + self._b


def test_deviation_gain_accel_is_negative_when_opening_rises_with_actual_speed() -> None:
    """opening = v0（実車速が上がるほど開度が増える）は、アクセルとしては逆向き
    （実車速が基準より遅れているのに踏み増さない）。`deviation_gain` は `-raw_slope` なので
    負になる。
    """
    model = _BatchLinear(np.array([1.0, *([0.0] * 8)]))

    got = deviation_gain(model, DEFAULT_FEATURE_SPEC, v0=60.0, pedal="accel")

    assert got == pytest.approx(-1.0, abs=1e-6)
    assert got < 0.0


def test_deviation_gain_accel_is_positive_when_opening_falls_with_actual_speed() -> None:
    """opening = -v0（実車速が上がるほど開度が減る）は正しいアクセルの向き
    （実車速が基準より遅れる＝v0 が下がる → 開度が増える）。`deviation_gain` は正になる。
    """
    model = _BatchLinear(np.array([-1.0, *([0.0] * 8)]))

    got = deviation_gain(model, DEFAULT_FEATURE_SPEC, v0=60.0, pedal="accel")

    assert got == pytest.approx(1.0, abs=1e-6)
    assert got > 0.0


def test_deviation_gain_brake_sign_is_opposite_of_accel_for_the_same_model() -> None:
    """同じモデル（opening = v0）でも、ブレーキは向きの意味が逆（実車速が基準より速い＝v0 が
    上がる → 開度が増えるのが正しい）ので、`deviation_gain` は accel と符号が反転する。
    """
    model = _BatchLinear(np.array([1.0, *([0.0] * 8)]))

    accel_gain = deviation_gain(model, DEFAULT_FEATURE_SPEC, v0=60.0, pedal="accel")
    brake_gain = deviation_gain(model, DEFAULT_FEATURE_SPEC, v0=60.0, pedal="brake")

    assert accel_gain == pytest.approx(-brake_gain, abs=1e-6)
    assert brake_gain > 0.0


# ── 参照用: 元の単位の係数（raw_unit_coefficients / export_model_coefficients） ──


def _fitted_pipeline(n_features: int, seed: int = 0) -> tuple[object, np.ndarray]:
    rng = np.random.default_rng(seed)
    scale = 3.0 * np.arange(1, n_features + 1)
    x = rng.normal(loc=10.0 * np.arange(1, n_features + 1), scale=scale, size=(200, n_features))
    y = 0.3 * x[:, 0] - 1.2 * x[:, 1] + 0.01 * x[:, 0] * x[:, 1] + rng.normal(scale=0.1, size=200)
    model = make_estimator()
    model.fit(x, y)
    return model, x


def test_raw_unit_coefficients_reproduce_pipeline_predictions() -> None:
    model, x = _fitted_pipeline(3)
    names = ["v0", "dv_1.0", "dv_2.0"]

    terms, coef, intercept = raw_unit_coefficients(model, names)

    poly_x = model.steps[0][1].transform(x)  # 展開後の項（標準化前）
    got = poly_x @ np.asarray(coef) + intercept
    assert got == pytest.approx(model.predict(x), abs=1e-6)
    assert len(terms) == len(coef) == 9  # 1次3 + 2次6
    assert terms[:3] == names
    assert "v0 dv_1.0" in terms


def test_raw_unit_coefficients_rejects_wrong_name_count() -> None:
    model, _ = _fitted_pipeline(3)
    with pytest.raises(ValueError, match="特徴量名の数"):
        raw_unit_coefficients(model, ["v0", "dv_1.0"])


def test_export_model_coefficients_writes_yaml_next_to_pkl(tmp_path: Path) -> None:
    import yaml

    accel_spec = FeatureSpec(
        lookahead_horizons_s=(0.5, 1.0), past_horizons_s=(0.5,), include_v0_sq=False,
        include_dv_regime_x_v0=False,
    )
    brake_spec = FeatureSpec(
        lookahead_horizons_s=(0.2, 1.0, 2.0), past_horizons_s=(0.5,), include_v0_sq=False,
        include_dv_regime_x_v0=False,
    )
    accel_model, _ = _fitted_pipeline(len(accel_spec.feature_names()))
    brake_model, _ = _fitted_pipeline(len(brake_spec.feature_names()), seed=1)
    pkl = tmp_path / "m.pkl"
    with pkl.open("wb") as f:
        pickle.dump(
            {
                "model_type": MODEL_TYPE, "accel_model": accel_model, "brake_model": brake_model,
                "feature_spec": asdict(accel_spec), "accel_feature_spec": asdict(accel_spec),
                "brake_feature_spec": asdict(brake_spec), "trained_at": "2026-09-29T00:00:00",
            },
            f,
        )

    out = export_model_coefficients(pkl)

    assert out == tmp_path / "m.yaml"
    doc = yaml.safe_load(out.read_text(encoding="utf-8"))
    assert doc["accel"]["horizons_s"] == [0.5, 1.0]
    assert doc["brake"]["horizons_s"] == [0.2, 1.0, 2.0]
    assert doc["accel"]["features"] == accel_spec.feature_names()
    assert len(doc["brake"]["coefficients"]) > len(doc["accel"]["coefficients"])
    assert isinstance(doc["accel"]["intercept"], float)
