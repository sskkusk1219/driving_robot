"""先読み Ridge 逆モデル学習（model_training）のユニットテスト。"""

import pickle
import tempfile
from datetime import UTC, datetime, timedelta
from pathlib import Path

import pytest

from src.domain.control.feedforward import FeedforwardController
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import (
    FEATURE_NAMES,
    LOOKAHEAD_HORIZONS_S,
    MODEL_TYPE,
    build_feature_row,
    estimate_dynamics_params,
    train_inverse_model,
)
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile

DT_S = 0.1
MAX_OFFSET = round(max(LOOKAHEAD_HORIZONS_S) / DT_S)  # = 30


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
        row = build_feature_row(30.0, [33.0, 36.0, 42.0, 48.0])
        assert row.shape == (1, len(FEATURE_NAMES))

    def test_values(self) -> None:
        v0 = 30.0
        future = [33.0, 36.0, 42.0, 48.0]
        row = build_feature_row(v0, future)[0]
        # [v0, dv0.5, dv1.0, dv2.0, dv3.0, v0², dv1.0·v0]
        assert row[0] == pytest.approx(30.0)
        assert row[1] == pytest.approx(3.0)
        assert row[2] == pytest.approx(6.0)
        assert row[3] == pytest.approx(12.0)
        assert row[4] == pytest.approx(18.0)
        assert row[5] == pytest.approx(900.0)
        assert row[6] == pytest.approx(6.0 * 30.0)

    def test_wrong_length_raises(self) -> None:
        with pytest.raises(ValueError, match="future_speeds"):
            build_feature_row(30.0, [33.0, 36.0])


class TestTrainInverseModel:
    def test_produces_loadable_model(self) -> None:
        logs = make_session_logs("s1")
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            path, _ = train_inverse_model(logs, profile, output_dir=tmpdir)
            assert Path(path).exists()
            ff = FeedforwardController()
            ff.load_model(path)
            effort = ff.predict_effort(30.0, [33.0, 36.0, 42.0, 48.0])
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
        assert data["profile_id"] == "p-meta"

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
        """2 セッションのログを渡すと、各セッション内のみで先読みされる。

        セッション境界をまたがないため、有効サンプル数 = Σ(len_i - max_offset)。
        誤って 1 系列として扱うと (合計len - max_offset) になり値が変わる。
        """
        s1 = make_session_logs("s1")  # 100 件
        s2 = make_session_logs("s2")  # 100 件
        # わざと混在順で渡す（リポジトリ順序に依存しないことの確認も兼ねる）
        logs = s1 + s2
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            _, metrics = train_inverse_model(logs, profile, output_dir=tmpdir)
        total = metrics["accel"]["n"] + metrics["brake"]["n"]
        expected = (len(s1) - MAX_OFFSET) + (len(s2) - MAX_OFFSET)
        assert total == expected

    def test_brake_deadband_zeroes_small_openings(self) -> None:
        """ブレーキ開度 < 1% は 0 扱いになり、ブレーキモデルがほぼ 0 を出力する。"""
        logs = make_session_logs("s1", brake_open=0.5)  # デッドバンド未満
        profile = make_profile()
        with tempfile.TemporaryDirectory() as tmpdir:
            path, _ = train_inverse_model(logs, profile, output_dir=tmpdir)
            ff = FeedforwardController()
            ff.load_model(path)
            # 強い減速予見（エンジンブレーキ超）→ ブレーキモデル出力（負の努力量）
            effort = ff.predict_effort(60.0, [57.0, 54.0, 48.0, 42.0])
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

    def test_deadbands_are_never_auto_estimated(self) -> None:
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
