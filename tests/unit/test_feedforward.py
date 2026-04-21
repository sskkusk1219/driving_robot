import pickle
from pathlib import Path

import numpy as np
import pytest

from src.domain.control.feedforward import FeedforwardController


def make_model_file(tmp_path: Path) -> Path:
    """テスト用の簡単な運転モデルを pkl ファイルとして作成する。

    速度グリッド: [0, 50, 100] km/h
    加速度グリッド: [-5, 0, 5] km/h/s
    アクセルマップ: 速度が上がるほど開度が上がる単純マップ
    ブレーキマップ: 加速度が負になるほど開度が上がる単純マップ
    """
    speed_grid = np.array([0.0, 50.0, 100.0])
    accel_grid = np.array([-5.0, 0.0, 5.0])

    # accel_map[i, j] = speed_grid[i] * 0.5 + max(0, accel_grid[j]) * 5
    accel_map = np.array(
        [
            [0.0, 0.0, 25.0],
            [25.0, 25.0, 50.0],
            [50.0, 50.0, 75.0],
        ]
    )

    # brake_map[i, j] = max(0, -accel_grid[j]) * 10
    brake_map = np.array(
        [
            [50.0, 0.0, 0.0],
            [50.0, 0.0, 0.0],
            [50.0, 0.0, 0.0],
        ]
    )

    model_path = tmp_path / "test_model.pkl"
    data = {
        "speed_grid": speed_grid,
        "accel_grid": accel_grid,
        "accel_map": accel_map,
        "brake_map": brake_map,
    }
    with model_path.open("wb") as f:
        pickle.dump(data, f)
    return model_path


class TestFeedforwardControllerLoadModel:
    def test_load_valid_model(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        assert ff._accel_interp is not None
        assert ff._brake_interp is not None

    def test_load_nonexistent_file(self) -> None:
        ff = FeedforwardController()
        with pytest.raises(FileNotFoundError):
            ff.load_model("/nonexistent/path/model.pkl")

    def test_load_model_missing_keys_raises(self, tmp_path: Path) -> None:
        """必須キーが欠けた pkl は ValueError になること。"""
        bad_path = tmp_path / "bad_model.pkl"
        with bad_path.open("wb") as f:
            pickle.dump({"speed_grid": np.array([0.0, 50.0])}, f)
        ff = FeedforwardController()
        with pytest.raises(ValueError, match="missing required keys"):
            ff.load_model(str(bad_path))


class TestFeedforwardControllerPredict:
    def test_predict_without_model_raises(self) -> None:
        ff = FeedforwardController()
        with pytest.raises(RuntimeError, match="Model not loaded"):
            ff.predict(ref_speed=50.0, ref_accel=0.0)

    def test_predict_grid_point_accel(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        # speed=50, accel=0 → accel_map[1,1]=25, brake_map[1,1]=0
        accel, brake = ff.predict(ref_speed=50.0, ref_accel=0.0)
        assert accel == pytest.approx(25.0)
        assert brake == pytest.approx(0.0)

    def test_predict_grid_point_braking(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        # speed=50, accel=-5 → accel_map[1,0]=25, brake_map[1,0]=50
        accel, brake = ff.predict(ref_speed=50.0, ref_accel=-5.0)
        assert accel == pytest.approx(25.0)
        assert brake == pytest.approx(50.0)

    def test_predict_interpolated(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        # speed=25 (0と50の中間) accel=0 → accel_map補間で 12.5 が期待値
        accel, brake = ff.predict(ref_speed=25.0, ref_accel=0.0)
        assert accel == pytest.approx(12.5)
        assert brake == pytest.approx(0.0)

    def test_predict_clamps_to_100(self, tmp_path: Path) -> None:
        """グリッド外側でも開度が 0〜100 にクランプされること。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        # グリッド範囲外（速度が極端に大きい）でも例外なくクランプ
        accel, brake = ff.predict(ref_speed=200.0, ref_accel=10.0)
        assert 0.0 <= accel <= 100.0
        assert 0.0 <= brake <= 100.0

    def test_predict_clamps_to_zero(self, tmp_path: Path) -> None:
        """負の補間値が 0.0 にクランプされること。"""
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        accel, brake = ff.predict(ref_speed=0.0, ref_accel=0.0)
        assert accel >= 0.0
        assert brake >= 0.0

    def test_predict_returns_tuple(self, tmp_path: Path) -> None:
        path = make_model_file(tmp_path)
        ff = FeedforwardController()
        ff.load_model(str(path))
        result = ff.predict(ref_speed=50.0, ref_accel=0.0)
        assert isinstance(result, tuple)
        assert len(result) == 2
