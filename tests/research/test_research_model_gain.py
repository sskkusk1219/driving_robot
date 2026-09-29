"""model_gain.py（C5 の実質 Kp を測るツール）のユニットテスト。

`tests/research/test_research_kpi.py` と同じ流儀で、実 pkl は使わず `.predict(X)` を持つだけの
合成した線形モデルで検算する。特徴量の並びは `DEFAULT_FEATURE_SPEC.feature_names()` に従い
    [v0, dv_0.5, dv_1.0, dv_2.0, dv_3.0, v0_sq, dv1_x_v0, dv_past_0.5, dv_past_1.0]
なので、係数を 1 つずつ立てれば `level_gain`/`sinusoid_gain` が「どの経路をどう測っているか」を
手計算した値と突き合わせられる。
"""

from __future__ import annotations

import pickle
from dataclasses import dataclass

import numpy as np
import pytest

import tests.research.model_gain as model_gain
from tests.research.ff_model import DEFAULT_FEATURE_SPEC, FeatureSpec


@dataclass
class _LinearModel:
    """`.predict(X)` だけを持つ合成モデル（opening = X @ coef + intercept）。"""

    coef: np.ndarray
    intercept: float = 0.0

    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.asarray(x, dtype=float) @ self.coef + self.intercept


def _linear_model(spec: FeatureSpec, weights: dict[str, float]) -> _LinearModel:
    """特徴量名で指定した係数だけを立てた線形モデルを作る（他は 0）。"""
    names = spec.feature_names()
    coef = np.zeros(len(names))
    for name, w in weights.items():
        coef[names.index(name)] = w
    return _LinearModel(coef=coef)


# ─────────────────────────────────────────────────────────────────────
# level_gain
# ─────────────────────────────────────────────────────────────────────


def test_level_gain_matches_analytic_slope() -> None:
    """opening = a*v0 + b*dv_1.0 + c*dv_past_0.5 のとき、level_gain は |a - b| に一致する。

    level_gain は past を v0 と同じ値に置いて dv_past を常に 0 に固定するため、c は結果に
    効かない（手計算: v0'=v0±eps・future=[v0]*4・past=[v0±eps]*2 → dv_1.0=∓eps・dv_past=0）。
    """
    a, b, c = 0.7, 0.3, 0.05
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"v0": a, "dv_1.0": b, "dv_past_0.5": c})

    got = model_gain.level_gain(model, DEFAULT_FEATURE_SPEC, v0=60.0)

    assert got == pytest.approx(abs(a - b), abs=1e-6)


def test_level_gain_is_nonnegative_even_when_slope_is_positive() -> None:
    """開度が実車速とともに増える(b>a)符号のモデルでも、戻り値は絶対値（正）。"""
    a, b = 0.2, 0.9
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"v0": a, "dv_1.0": b})

    got = model_gain.level_gain(model, DEFAULT_FEATURE_SPEC, v0=90.0)

    assert got == pytest.approx(abs(a - b), abs=1e-6)
    assert got > 0.0


# ─────────────────────────────────────────────────────────────────────
# sinusoid_gain
# ─────────────────────────────────────────────────────────────────────


def test_sinusoid_gain_dv_past_only_path_is_zero_without_dv_past_coefficient() -> None:
    """dv_past の係数が無いモデルでは「dv_past 経路のみ」の振幅比が 0 になる。"""
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"v0": 1.0, "dv_1.0": 0.5})

    total, dv, dv_past = model_gain.sinusoid_gain(
        model, DEFAULT_FEATURE_SPEC, v_mean=60.0, freq_hz=1.4
    )

    assert dv_past == pytest.approx(0.0, abs=1e-9)
    # v0 の係数だけを持つ成分は合計・dv経路のどちらにも乗るので 0 より大きい
    assert total > 0.0
    assert dv > 0.0


def test_sinusoid_gain_v0_only_model_matches_analytic_amplitude() -> None:
    """opening = a*v0 のみのモデルでは、合計・dv経路とも振幅比が a に一致し、
    dv_past経路は 0 になる（v0・future を v_mean に固定するため過去は効かない）。"""
    a = 1.3
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"v0": a})

    total, dv, dv_past = model_gain.sinusoid_gain(
        model, DEFAULT_FEATURE_SPEC, v_mean=60.0, freq_hz=1.4
    )

    assert total == pytest.approx(a, abs=1e-6)
    assert dv == pytest.approx(a, abs=1e-6)
    assert dv_past == pytest.approx(0.0, abs=1e-9)


def test_sinusoid_gain_total_path_reacts_to_dv_past_coefficient() -> None:
    """dv_past の係数を持つモデルでは、「dv_past 経路のみ」の振幅比が 0 より大きくなる。"""
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"dv_past_0.5": 0.4, "dv_past_1.0": 0.2})

    _total, _dv, dv_past = model_gain.sinusoid_gain(
        model, DEFAULT_FEATURE_SPEC, v_mean=60.0, freq_hz=1.4
    )

    assert dv_past > 0.0


# ─────────────────────────────────────────────────────────────────────
# gain_table / _load_pkl（tmp_path に書いた合成 pkl で確認する）
# ─────────────────────────────────────────────────────────────────────


def _write_pkl(path, model: _LinearModel, spec: FeatureSpec) -> None:
    from dataclasses import asdict

    payload = {
        "accel_model": model,
        "brake_model": model,
        "feature_spec": asdict(spec),
        "speed_clip_max": 140.0,
        "deadbands_pct": {"accel": 5.0, "brake": 8.0},
    }
    with path.open("wb") as f:
        pickle.dump(payload, f)


def test_gain_table_contains_pkl_path_and_speeds(tmp_path) -> None:
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"v0": 0.8, "dv_1.0": 0.3})
    pkl_path = tmp_path / "synthetic_20260101_000000.pkl"
    _write_pkl(pkl_path, model, DEFAULT_FEATURE_SPEC)

    table = model_gain.gain_table([pkl_path], speeds_kmh=[30.0, 60.0])

    assert pkl_path.name in table
    assert "30km/h" in table
    assert "60km/h" in table
    # level_gain(v0=30) == level_gain(v0=60) == |0.8-0.3| = 0.5（v0 に依存しない線形モデルのため）
    assert "0.500" in table


def test_load_pkl_uses_default_feature_spec_when_key_missing(tmp_path) -> None:
    """`feature_spec` キーが無い pkl は DEFAULT_FEATURE_SPEC を使う。"""
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"v0": 1.0})
    pkl_path = tmp_path / "no_spec.pkl"
    with pkl_path.open("wb") as f:
        pickle.dump({"accel_model": model}, f)

    loaded_model, spec, meta = model_gain._load_pkl(pkl_path)

    assert spec == DEFAULT_FEATURE_SPEC
    # pickle 経由なので同一オブジェクトではなく別インスタンスになる。中身（係数）が一致すればよい
    assert np.array_equal(loaded_model.coef, model.coef)
    assert meta["speed_clip_max"] is None
    assert meta["deadbands_pct"] is None


def test_load_pkl_raises_value_error_when_accel_model_missing(tmp_path) -> None:
    pkl_path = tmp_path / "broken.pkl"
    with pkl_path.open("wb") as f:
        pickle.dump({"brake_model": object()}, f)

    with pytest.raises(ValueError, match="accel_model"):
        model_gain._load_pkl(pkl_path)


def test_load_pkl_raises_value_error_when_file_unreadable(tmp_path) -> None:
    missing = tmp_path / "does_not_exist.pkl"

    with pytest.raises(ValueError, match="読み込めません"):
        model_gain._load_pkl(missing)


def test_main_exits_with_code_2_on_unreadable_pkl(tmp_path, capsys) -> None:
    missing = tmp_path / "missing.pkl"

    code = model_gain.main([str(missing)])

    assert code == 2
    err = capsys.readouterr().err
    assert "エラー" in err


def test_main_prints_markdown_for_valid_pkl(tmp_path, capsys) -> None:
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"v0": 1.0, "dv_1.0": 0.4})
    pkl_path = tmp_path / "ok.pkl"
    _write_pkl(pkl_path, model, DEFAULT_FEATURE_SPEC)

    code = model_gain.main([str(pkl_path), "--speeds", "30,60", "--freq", "1.4"])

    out = capsys.readouterr().out
    assert code == 0
    assert "level_gain" in out
    assert "sinusoid_gain" in out
    assert pkl_path.name in out


# ── 手順6: ペダル別ホライズン（ProblemReport_20260921） ─────────────


def _write_pedal_separated_pkl(
    path, accel_model: _LinearModel, brake_model: _LinearModel,
    accel_spec: FeatureSpec, brake_spec: FeatureSpec,
) -> None:
    from dataclasses import asdict

    lookahead = tuple(sorted({*accel_spec.lookahead_horizons_s, *brake_spec.lookahead_horizons_s}))
    control_spec = FeatureSpec(lookahead_horizons_s=lookahead)
    payload = {
        "accel_model": accel_model,
        "brake_model": brake_model,
        "feature_spec": asdict(control_spec),
        "accel_feature_spec": asdict(accel_spec),
        "brake_feature_spec": asdict(brake_spec),
        "stop_horizon_s": lookahead[0],
        "speed_clip_max": 140.0,
        "deadbands_pct": {"accel": 5.0, "brake": 8.0},
    }
    with path.open("wb") as f:
        pickle.dump(payload, f)


def test_load_pkl_side_accel_vs_brake_use_different_specs(tmp_path) -> None:
    accel_spec = FeatureSpec(lookahead_horizons_s=(0.5, 1.0, 2.0, 3.0))
    brake_spec = FeatureSpec(lookahead_horizons_s=(0.1, 1.0))
    accel_model = _linear_model(accel_spec, {"v0": 1.0})
    brake_model = _linear_model(brake_spec, {"v0": 2.0})
    pkl_path = tmp_path / "separated.pkl"
    _write_pedal_separated_pkl(pkl_path, accel_model, brake_model, accel_spec, brake_spec)

    loaded_accel, spec_a, meta_a = model_gain._load_pkl(pkl_path, "accel")
    loaded_brake, spec_b, meta_b = model_gain._load_pkl(pkl_path, "brake")

    assert spec_a == accel_spec
    assert spec_b == brake_spec
    assert meta_a["pedal_separated"] is True
    assert meta_b["pedal_separated"] is True
    assert np.array_equal(loaded_accel.coef, accel_model.coef)
    assert np.array_equal(loaded_brake.coef, brake_model.coef)


def test_load_pkl_pedal_separated_false_for_legacy_pkl(tmp_path) -> None:
    model = _linear_model(DEFAULT_FEATURE_SPEC, {"v0": 1.0})
    pkl_path = tmp_path / "legacy.pkl"
    _write_pkl(pkl_path, model, DEFAULT_FEATURE_SPEC)

    _loaded, _spec, meta = model_gain._load_pkl(pkl_path)

    assert meta["pedal_separated"] is False


def test_main_side_brake_uses_brake_model(tmp_path, capsys) -> None:
    accel_spec = FeatureSpec(lookahead_horizons_s=(0.5, 1.0, 2.0, 3.0))
    brake_spec = FeatureSpec(lookahead_horizons_s=(0.1, 1.0))
    accel_model = _linear_model(accel_spec, {"v0": 1.0})
    brake_model = _linear_model(brake_spec, {"v0": 2.5})
    pkl_path = tmp_path / "separated_main.pkl"
    _write_pedal_separated_pkl(pkl_path, accel_model, brake_model, accel_spec, brake_spec)

    code = model_gain.main([str(pkl_path), "--side", "brake", "--speeds", "60"])

    out = capsys.readouterr().out
    assert code == 0
    assert "--side brake" in out
    assert "2.500" in out  # level_gain(v0=60) == |2.5 - 0| = 2.5（v0 のみの線形モデル）
