import pickle
from pathlib import Path

import numpy as np
from scipy.interpolate import RegularGridInterpolator


class FeedforwardController:
    """2次元グリッド補間による FF 制御。load_model() 後に predict() を呼ぶ。"""

    def __init__(self) -> None:
        self._accel_interp: RegularGridInterpolator | None = None
        self._brake_interp: RegularGridInterpolator | None = None

    def load_model(self, model_path: str) -> None:
        """pkl ファイルから運転モデルをロードして補間器を構築する。

        pkl 構造:
            speed_grid: np.ndarray  # [N] km/h
            accel_grid: np.ndarray  # [M] km/h/s
            accel_map:  np.ndarray  # [N, M] %
            brake_map:  np.ndarray  # [N, M] %
        """
        path = Path(model_path)
        if not path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")

        # 運転モデルは開発者がローカルで生成した信頼済みファイルのみを想定。
        # 外部入力のパスをそのまま渡さないこと（呼び出し元の責務）。
        with path.open("rb") as f:
            data: dict[str, np.ndarray] = pickle.load(f)  # noqa: S301

        required_keys = {"speed_grid", "accel_grid", "accel_map", "brake_map"}
        missing = required_keys - data.keys()
        if missing:
            raise ValueError(f"Model file is missing required keys: {missing}")

        speed_grid: np.ndarray = data["speed_grid"]
        accel_grid: np.ndarray = data["accel_grid"]

        # bounds_error=False + fill_value=None で端点外側も端点値を返す（クランプ相当）
        self._accel_interp = RegularGridInterpolator(
            (speed_grid, accel_grid),
            data["accel_map"],
            method="linear",
            bounds_error=False,
            fill_value=None,
        )
        self._brake_interp = RegularGridInterpolator(
            (speed_grid, accel_grid),
            data["brake_map"],
            method="linear",
            bounds_error=False,
            fill_value=None,
        )

    def predict(self, ref_speed: float, ref_accel: float) -> tuple[float, float]:
        """基準車速・基準加速度からアクセル・ブレーキ開度[%]を返す。

        Returns:
            (accel_opening, brake_opening) — 各 0.0〜100.0 [%]

        Raises:
            RuntimeError: load_model() が呼ばれていない場合
        """
        if self._accel_interp is None or self._brake_interp is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        point = np.array([[ref_speed, ref_accel]])
        accel_raw = float(self._accel_interp(point)[0])
        brake_raw = float(self._brake_interp(point)[0])

        accel_opening = max(0.0, min(100.0, accel_raw))
        brake_opening = max(0.0, min(100.0, brake_raw))
        return accel_opening, brake_opening
