"""先読み Ridge 逆モデル学習（model_training）のユニットテスト。"""

import pickle
import tempfile
from dataclasses import asdict
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from src.domain.control.feedforward import FeedforwardController
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import (
    DEADBAND_BIN_WIDTH_PCT,
    DEADBAND_MIN_BIN_SAMPLES,
    DEADBAND_SCAN_MAX_PCT,
    DEFAULT_FEATURE_SPEC,
    MODEL_TYPE,
    FeatureSpec,
    _build_feature_matrix,
    _estimate_onset_deadband_pct,
    build_feature_row,
    estimate_dynamics_params,
    train_inverse_model,
)
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile

DT_S = 0.1

# 現行9特徴のデフォルト仕様に対する参照（既存テストの回帰ピンをそのまま使うためのエイリアス）
FEATURE_NAMES = DEFAULT_FEATURE_SPEC.feature_names()
LOOKAHEAD_HORIZONS_S = DEFAULT_FEATURE_SPEC.lookahead_horizons_s
PAST_HORIZONS_S = DEFAULT_FEATURE_SPEC.past_horizons_s


def make_profile(pid: str = "p1") -> VehicleProfile:
    return VehicleProfile(
        id=pid,
        name="t",
        max_accel_opening=80.0,
        max_brake_opening=80.0,
        max_speed=120.0,
        max_decel_g=0.4,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=None,
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )


def make_session_logs(
    session_id: str,
    n_rise: int = 50,
    n_fall: int = 50,
    brake_open: float = 30.0,
) -> list[DriveLog]:
    """加速フェーズ → 減速フェーズの連続ログを 1 セッション分生成する。"""
    t0 = datetime.now(tz=UTC)
    logs: list[DriveLog] = []
    speed = 0.0
    i = 0
    for _ in range(n_rise):
        speed = min(100.0, speed + 1.0)
        logs.append(_log(session_id, i, t0, speed, accel_open=40.0, brake_open=0.0))
        i += 1
    for _ in range(n_fall):
        speed = max(0.0, speed - 1.0)
        logs.append(_log(session_id, i, t0, speed, accel_open=0.0, brake_open=brake_open))
        i += 1
    return logs


def _log(
    session_id: str,
    i: int,
    t0: datetime,
    speed: float,
    accel_open: float,
    brake_open: float,
) -> DriveLog:
    return DriveLog(
        id=i,
        session_id=session_id,
        timestamp=t0 + timedelta(seconds=DT_S * i),
        ref_speed_kmh=None,
        actual_speed_kmh=speed,
        accel_opening=accel_open,
        brake_opening=brake_open,
        accel_pos=0,
        brake_pos=0,
        accel_current=0.0,
        brake_current=0.0,
    )


class TestBuildFeatureRow:
    def test_length_matches_feature_names(self) -> None:
        row = build_feature_row(30.0, [33.0, 36.0, 42.0, 48.0], [29.0, 28.0])
        assert row.shape == (1, len(FEATURE_NAMES))

    def test_values(self) -> None:
        v0 = 30.0
        future = [33.0, 36.0, 42.0, 48.0]
        past = [29.0, 27.0]  # 0.5s/1.0s 前の速度（加速ランプ中を模す）
        row = build_feature_row(v0, future, past)[0]
        # [v0, dv0.5, dv1.0, dv2.0, dv3.0, v0², dv1.0·v0, dv_past0.5, dv_past1.0]
        assert row[0] == pytest.approx(30.0)
        assert row[1] == pytest.approx(3.0)
        assert row[2] == pytest.approx(6.0)
        assert row[3] == pytest.approx(12.0)
        assert row[4] == pytest.approx(18.0)
        assert row[5] == pytest.approx(900.0)
        assert row[6] == pytest.approx(6.0 * 30.0)
        assert row[7] == pytest.approx(1.0)  # v0 - past[0]
        assert row[8] == pytest.approx(3.0)  # v0 - past[1]

    def test_wrong_future_length_raises(self) -> None:
        with pytest.raises(ValueError, match="future_speeds"):
            build_feature_row(30.0, [33.0, 36.0], [29.0, 28.0])

    def test_wrong_past_length_raises(self) -> None:
        with pytest.raises(ValueError, match="past_speeds"):
            build_feature_row(30.0, [33.0, 36.0, 42.0, 48.0], [29.0])

    def test_custom_spec_short_horizons_with_accel_term(self) -> None:
        """短期ホライズン(0.1/0.2/0.3秒)+加速度項のカスタムspecで names・値が正しいこと。"""
        spec = FeatureSpec(
            lookahead_horizons_s=(0.1, 0.2, 0.3),
            past_horizons_s=(0.1, 0.2),
            regime_horizon_s=0.2,
            accel_horizons_s=(0.2,),
        )
        assert spec.feature_names() == [
            "v0",
            "dv_0.1",
            "dv_0.2",
            "dv_0.3",
            "v0_sq",
            "dv1_x_v0",
            "dv_past_0.1",
            "dv_past_0.2",
            "accel_0.2",
        ]

        v0 = 20.0
        future = [20.5, 21.2, 21.5]
        past = [19.5, 18.5]
        row = build_feature_row(v0, future, past, spec)[0]

        assert row.shape == (len(spec.feature_names()),)
        assert row[0] == pytest.approx(20.0)  # v0
        assert row[1] == pytest.approx(0.5)  # dv_0.1
        assert row[2] == pytest.approx(1.2)  # dv_0.2 (regime)
        assert row[3] == pytest.approx(1.5)  # dv_0.3
        assert row[4] == pytest.approx(400.0)  # v0_sq
        assert row[5] == pytest.approx(1.2 * 20.0)  # dv1_x_v0（regime=dv_0.2）
        assert row[6] == pytest.approx(0.5)  # dv_past_0.1 = v0 - past[0]
        assert row[7] == pytest.approx(1.5)  # dv_past_0.2 = v0 - past[1]
        # accel_0.2 = (future[0.2] - 2*v0 + past[0.2]) / 0.2² = (21.2 - 40 + 18.5) / 0.04
        assert row[8] == pytest.approx((21.2 - 40.0 + 18.5) / 0.04)

    def test_custom_spec_disabling_terms_shrinks_row(self) -> None:
        spec = FeatureSpec(include_v0_sq=False, include_dv_regime_x_v0=False)
        row = build_feature_row(30.0, [33.0, 36.0, 42.0, 48.0], [29.0, 27.0], spec)[0]
        assert row.shape == (7,)  # v0 + 4 dv + 2 past（v0_sq・交互作用項なし）
        assert list(row) == pytest.approx([30.0, 3.0, 6.0, 12.0, 18.0, 1.0, 3.0])


class TestFeatureSpecValidation:
    def test_default_matches_current_nine_features(self) -> None:
        """DEFAULT_FEATURE_SPEC の特徴名が現行9特徴と完全一致する（回帰ピン）。"""
        assert DEFAULT_FEATURE_SPEC.feature_names() == [
            "v0",
            "dv_0.5",
            "dv_1.0",
            "dv_2.0",
            "dv_3.0",
            "v0_sq",
            "dv1_x_v0",
            "dv_past_0.5",
            "dv_past_1.0",
        ]

    def test_default_regime_col(self) -> None:
        # 列順: v0(0), dv_0.5(1), dv_1.0(2, =regime) ...
        assert DEFAULT_FEATURE_SPEC.regime_col() == 2

    def test_non_ascending_horizons_raises(self) -> None:
        with pytest.raises(ValueError, match="昇順"):
            FeatureSpec(lookahead_horizons_s=(1.0, 0.5, 2.0))

    def test_duplicate_horizons_raises(self) -> None:
        with pytest.raises(ValueError, match="昇順"):
            FeatureSpec(lookahead_horizons_s=(1.0, 1.0, 2.0))

    def test_empty_lookahead_raises(self) -> None:
        with pytest.raises(ValueError):
            FeatureSpec(lookahead_horizons_s=())

    def test_regime_horizon_not_in_lookahead_raises(self) -> None:
        with pytest.raises(ValueError, match="regime_horizon_s"):
            FeatureSpec(lookahead_horizons_s=(0.5, 1.0), regime_horizon_s=2.0)

    def test_accel_horizon_not_in_lookahead_raises(self) -> None:
        with pytest.raises(ValueError, match="accel_horizons_s"):
            FeatureSpec(
                lookahead_horizons_s=(0.5, 1.0),
                past_horizons_s=(0.5, 1.0, 2.0),
                accel_horizons_s=(2.0,),
            )

    def test_accel_horizon_not_in_past_raises(self) -> None:
        with pytest.raises(ValueError, match="accel_horizons_s"):
            FeatureSpec(
                lookahead_horizons_s=(0.5, 1.0),
                past_horizons_s=(0.5,),
                accel_horizons_s=(1.0,),
            )

    def test_feature_flags_can_disable_terms(self) -> None:
        spec = FeatureSpec(include_v0_sq=False, include_dv_regime_x_v0=False)
        assert spec.feature_names() == [
            "v0",
            "dv_0.5",
            "dv_1.0",
            "dv_2.0",
            "dv_3.0",
            "dv_past_0.5",
            "dv_past_1.0",
        ]


class TestBuildFeatureMatrixTrimming:
    def test_matrix_trims_by_max_offset_and_past_offset(self) -> None:
        speed = np.arange(0.0, 20.0, 1.0)
        spec = FeatureSpec(
            lookahead_horizons_s=(1.0, 2.0),
            past_horizons_s=(1.0,),
            regime_horizon_s=1.0,
        )
        offsets = [1, 2]
        past_offsets = [1]

        x, idx = _build_feature_matrix(speed, offsets, past_offsets, spec)

        # 先頭 max(past_offsets)=1 個・末尾 max(offsets)=2 個がトリムされる
        assert len(idx) == len(speed) - 2 - 1
        assert idx[0] == 1
        assert idx[-1] == len(speed) - 1 - 2
        assert x.shape[1] == len(spec.feature_names())

    def test_matrix_with_accel_term_shape(self) -> None:
        speed = np.arange(0.0, 20.0, 1.0)
        spec = FeatureSpec(
            lookahead_horizons_s=(1.0, 2.0),
            past_horizons_s=(1.0,),
            regime_horizon_s=1.0,
            accel_horizons_s=(1.0,),
        )
        offsets = [1, 2]
        past_offsets = [1]

        x, idx = _build_feature_matrix(speed, offsets, past_offsets, spec)

        # v0, dv_1.0, dv_2.0, v0_sq, dv1_x_v0, dv_past_1.0, accel_1.0 = 7列
        assert x.shape[1] == len(spec.feature_names()) == 7
        assert len(idx) == len(speed) - 2 - 1


class TestTrainInverseModel:
    def test_produces_loadable_model(self) -> None:
        logs = make_session_logs("s1")
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            path, _ = train_inverse_model(logs, profile, output_dir=tmpdir)
            assert Path(path).exists()
            ff = FeedforwardController()
            ff.load_model(path)
            effort = ff.predict_effort(30.0, [33.0, 36.0, 42.0, 48.0], [29.0, 28.0])
            assert -100.0 <= effort <= 100.0
            assert effort > 0.0  # 加速予見なので駆動側の努力量

    def test_pkl_contains_metadata(self) -> None:
        logs = make_session_logs("s1")
        profile = make_profile("p-meta")
        with tempfile.TemporaryDirectory() as tmpdir:
            path, _ = train_inverse_model(logs, profile, output_dir=tmpdir)
            with open(path, "rb") as f:
                data = pickle.load(f)  # noqa: S301
        assert data["model_type"] == MODEL_TYPE
        assert "accel_model" in data
        assert "brake_model" in data
        assert data["feature_names"] == FEATURE_NAMES
        assert data["horizons"] == list(LOOKAHEAD_HORIZONS_S)
        assert data["past_horizons"] == list(PAST_HORIZONS_S)
        assert data["feature_spec"] == asdict(DEFAULT_FEATURE_SPEC)
        assert data["profile_id"] == "p-meta"
        # 入力クリップ上限＝学習ログ観測最高車速（make_session_logs は 50 ステップ昇速で 50km/h）
        assert data["speed_clip_max"] == pytest.approx(50.0)

    def test_estimator_is_polynomial_pipeline(self) -> None:
        """推定器が多項式展開＋標準化＋Ridge の Pipeline であること。"""
        from sklearn.pipeline import Pipeline  # noqa: PLC0415
        from sklearn.preprocessing import PolynomialFeatures  # noqa: PLC0415

        logs = make_session_logs("s1")
        with tempfile.TemporaryDirectory() as tmpdir:
            path, _ = train_inverse_model(logs, make_profile(), output_dir=tmpdir)
            with open(path, "rb") as f:
                data = pickle.load(f)  # noqa: S301
        for key in ("accel_model", "brake_model"):
            model = data[key]
            assert isinstance(model, Pipeline)
            assert isinstance(model.steps[0][1], PolynomialFeatures)

    def test_metrics_present(self) -> None:
        logs = make_session_logs("s1")
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            _, metrics = train_inverse_model(logs, profile, output_dir=tmpdir)
        assert "mae" in metrics["accel"]
        assert "rmse" in metrics["accel"]
        assert metrics["accel"]["n"] > 0
        assert metrics["brake"]["n"] > 0

    def test_insufficient_total_samples_raises(self) -> None:
        logs = make_session_logs("s1", n_rise=3, n_fall=2)
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            with pytest.raises(LearningDataError):
                train_inverse_model(logs, profile, output_dir=tmpdir)

    def test_insufficient_brake_regime_raises(self) -> None:
        # 加速のみ（減速フェーズなし）→ ブレーキ側サンプル不足
        logs = make_session_logs("s1", n_rise=80, n_fall=0)
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            with pytest.raises(LearningDataError, match="減速"):
                train_inverse_model(logs, profile, output_dir=tmpdir)

    def test_profile_id_with_path_separator_is_sanitized(self) -> None:
        logs = make_session_logs("s1")
        profile = make_profile("../evil/profile")
        with tempfile.TemporaryDirectory() as tmpdir:
            path, _ = train_inverse_model(logs, profile, output_dir=tmpdir)
            assert Path(path).parent == Path(tmpdir)

    def test_sessions_are_not_crossed(self) -> None:
        """2 セッションを渡しても各セッション内のみで先読みされる（境界をまたがない）。

        非クロスなら「まとめて学習した標本数」＝「個別に学習した標本数の和」。誤って 1 系列として
        扱うと境界をまたぐ先読みで標本数が変わる。coast 除外で標本数がデータ依存になるため、
        固定の期待値ではなく個別学習との一致で検証する。
        """
        s1 = make_session_logs("s1")
        s2 = make_session_logs("s2")
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            _, m_both = train_inverse_model(s1 + s2, profile, output_dir=tmpdir)
            _, m1 = train_inverse_model(s1, profile, output_dir=tmpdir)
            _, m2 = train_inverse_model(s2, profile, output_dir=tmpdir)
        assert m_both["accel"]["n"] == m1["accel"]["n"] + m2["accel"]["n"]
        assert m_both["brake"]["n"] == m1["brake"]["n"] + m2["brake"]["n"]

    def test_brake_deadband_zeroes_small_openings(self) -> None:
        """ブレーキ開度 < 1% は 0 扱いになり、ブレーキモデルがほぼ 0 を出力する。"""
        logs = make_session_logs("s1", brake_open=0.5)  # デッドバンド未満
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            path, _ = train_inverse_model(logs, profile, output_dir=tmpdir)
            ff = FeedforwardController()
            ff.load_model(path)
            # 強い減速予見（エンジンブレーキ超）→ ブレーキモデル出力（負の努力量）
            effort = ff.predict_effort(60.0, [57.0, 54.0, 48.0, 42.0], [61.0, 62.0])
        assert effort == pytest.approx(0.0, abs=1e-6)


def _flat_session(
    session_id: str,
    speeds: list[float],
    accel_open: float,
    brake_open: float,
) -> list[DriveLog]:
    t0 = datetime.now(tz=UTC)
    return [
        _log(session_id, i, t0, s, accel_open=accel_open, brake_open=brake_open)
        for i, s in enumerate(speeds)
    ]


class TestEstimateDynamicsParams:
    def test_estimates_observed_constants(self) -> None:
        """停車保持/クリープ車速/エンジンブレーキ/クリープ加速率を観測値で上書きする。"""
        n = 12
        stop = _flat_session("s_stop", [0.0] * n, 0.0, 25.0)  # 停車保持ブレーキ 25%
        creep = _flat_session("s_creep", [6.0] * n, 0.0, 0.0)  # クリープ定常 6km/h
        coast = _flat_session("s_coast", [60.0 - 0.1 * i for i in range(n)], 0.0, 0.0)  # -1km/h/s
        accel = _flat_session("s_accel", [1.0 + 0.05 * i for i in range(n)], 0.0, 0.0)  # +0.5km/h/s
        logs = stop + creep + coast + accel

        new = estimate_dynamics_params(logs, FeedforwardParams())

        assert new.stop_brake_opening_pct == pytest.approx(25.0)
        assert new.creep_speed_kmh == pytest.approx(6.0, abs=0.5)
        assert new.engine_brake_decel_kmhs == pytest.approx(1.0, abs=0.2)
        assert new.creep_rate_kmhs == pytest.approx(0.5, abs=0.2)

    def test_deadband_zero_still_estimates_pedal_off_constants(self) -> None:
        """D5 回帰テスト: accel/brake_deadband_pct=0.0（合法値）でも strict < の
        falsy-zero でクリープ/エンジンブレーキ推定が全滅しない。"""
        n = 12
        creep = _flat_session("s_creep", [6.0] * n, 0.0, 0.0)  # クリープ定常 6km/h
        coast = _flat_session("s_coast", [60.0 - 0.1 * i for i in range(n)], 0.0, 0.0)  # -1km/h/s

        new = estimate_dynamics_params(
            creep + coast, FeedforwardParams(accel_deadband_pct=0.0, brake_deadband_pct=0.0)
        )

        assert new.creep_speed_kmh == pytest.approx(6.0, abs=0.5)
        assert new.engine_brake_decel_kmhs == pytest.approx(1.0, abs=0.2)

    def test_deadbands_keep_current_when_no_probe_data(self) -> None:
        """不感帯プローブ域(低開度の意図的保持)のサンプルが無いログでは既存値を保持する。

        セッションの開度が停車保持ブレーキ25%（探索上限10%超）のみのため、不感帯推定用の
        スキャンサンプルが collected されず None → 既存値保持となる（後方互換）。
        """
        n = 12
        logs = _flat_session("s_stop", [0.0] * n, 0.0, 25.0)
        current = FeedforwardParams(brake_deadband_pct=3.0, accel_deadband_pct=2.0)
        new = estimate_dynamics_params(logs, current)
        assert new.brake_deadband_pct == 3.0
        assert new.accel_deadband_pct == 2.0

    def test_insufficient_observations_keep_current(self) -> None:
        logs = _flat_session("s", [0.0, 0.0], 0.0, 0.0)  # 観測不足
        current = FeedforwardParams(creep_speed_kmh=9.0, stop_brake_opening_pct=33.0)
        new = estimate_dynamics_params(logs, current)
        assert new == current

    def test_estimates_accel_and_brake_deadband_with_sufficient_probe_data(self) -> None:
        """低開度保持プローブ相当のデータから不感帯境界(ビン下端)を推定する。"""
        n = 12
        # アクセル: 0/1.0/2.5% は無反応（速度一定）、4.0% で明確な加速応答
        accel_logs = (
            _flat_session("a0", [5.0] * n, 0.0, 0.0)
            + _flat_session("a1", [5.0] * n, 1.0, 0.0)
            + _flat_session("a2", [5.0] * n, 2.5, 0.0)
            + _flat_session("a3", [5.0 + 0.5 * i for i in range(n)], 4.0, 0.0)
        )
        # ブレーキ: 0/1.0/2.5% は無反応、4.0% で明確な減速応答
        brake_logs = (
            _flat_session("b0", [10.0] * n, 0.0, 0.0)
            + _flat_session("b1", [10.0] * n, 0.0, 1.0)
            + _flat_session("b2", [10.0] * n, 0.0, 2.5)
            + _flat_session("b3", [10.0 - 0.5 * i for i in range(n)], 0.0, 4.0)
        )
        new = estimate_dynamics_params(accel_logs + brake_logs, FeedforwardParams())
        assert new.accel_deadband_pct == pytest.approx(4.0)
        assert new.brake_deadband_pct == pytest.approx(4.0)


class TestEstimateOnsetDeadbandPct:
    def test_returns_none_for_empty_input(self) -> None:
        assert _estimate_onset_deadband_pct(np.array([]), np.array([])) is None

    def test_returns_none_when_baseline_bin_insufficient(self) -> None:
        # bin0（開度0近傍）のサンプルが最小数未満 → ベースライン不明で None
        openings = np.array([4.0] * 10)
        response = np.array([5.0] * 10)
        assert _estimate_onset_deadband_pct(openings, response) is None

    def test_detects_onset_bin_lower_edge(self) -> None:
        n = DEADBAND_MIN_BIN_SAMPLES
        openings = np.concatenate([np.full(n, 0.0), np.full(n, 4.0)])
        response = np.concatenate([np.zeros(n), np.full(n, 5.0)])
        assert _estimate_onset_deadband_pct(openings, response) == pytest.approx(4.0)

    def test_returns_none_when_no_onset_within_scan_range(self) -> None:
        n = DEADBAND_MIN_BIN_SAMPLES
        openings = np.concatenate([np.full(n, 0.0), np.full(n, 4.0)])
        response = np.concatenate([np.zeros(n), np.zeros(n)])  # どの開度でも無反応
        assert _estimate_onset_deadband_pct(openings, response) is None

    def test_onset_bin_below_min_samples_is_skipped(self) -> None:
        """応答ありのビンでもサンプル数不足ならスキップされ、次の十分なビンで検出される。"""
        few = DEADBAND_MIN_BIN_SAMPLES - 1
        enough = DEADBAND_MIN_BIN_SAMPLES
        openings = np.concatenate([np.full(enough, 0.0), np.full(few, 3.0), np.full(enough, 5.0)])
        response = np.concatenate([np.zeros(enough), np.full(few, 9.0), np.full(enough, 9.0)])
        # bin(3.0) はサンプル不足でスキップされ、bin(5.0) で検出される
        assert _estimate_onset_deadband_pct(openings, response) == pytest.approx(5.0)

    def test_result_never_exceeds_scan_max(self) -> None:
        n = DEADBAND_MIN_BIN_SAMPLES
        top_bin_opening = DEADBAND_SCAN_MAX_PCT - DEADBAND_BIN_WIDTH_PCT / 2
        openings = np.concatenate([np.full(n, 0.0), np.full(n, top_bin_opening)])
        response = np.concatenate([np.zeros(n), np.full(n, 9.0)])
        result = _estimate_onset_deadband_pct(openings, response)
        assert result is not None
        assert result <= DEADBAND_SCAN_MAX_PCT
