# 設計: 制御アルゴリズム実装

## PIDController 設計

離散時間PID（後退差分）。dt は制御ループ周期 = 0.05s (50ms)。

```python
class PIDController:
    _kp, _ki, _kd, _dt: float
    _integral: float = 0.0
    _prev_error: float = 0.0

    def update(setpoint, measurement) -> float:
        error = setpoint - measurement
        _integral += error * _dt
        derivative = (error - _prev_error) / _dt
        _prev_error = error
        return _kp * error + _ki * _integral + _kd * derivative

    def reset():
        _integral = 0.0
        _prev_error = 0.0
```

**アンチワインドアップ**: 今回は未実装（初期実装）。実機チューニング後に追加予定。

## FeedforwardController 設計

`.pkl` ファイルには以下の構造を保存:
```python
{
    "speed_grid": np.ndarray,   # [N] km/h
    "accel_grid": np.ndarray,   # [M] km/h/s
    "accel_map": np.ndarray,    # [N, M] %
    "brake_map": np.ndarray,    # [N, M] %
}
```

`scipy.interpolate.RegularGridInterpolator` を2本（アクセル・ブレーキ）構築。
グリッド範囲外は `bounds_error=False, fill_value=None`（端点外挿なし、クランプ）。

```python
class FeedforwardController:
    _accel_interp: RegularGridInterpolator | None = None
    _brake_interp: RegularGridInterpolator | None = None

    def load_model(model_path: str) -> None:
        # pickle.load → RegularGridInterpolator 構築

    def predict(ref_speed, ref_accel) -> tuple[float, float]:
        # モデル未ロード → RuntimeError
        # 補間 → clamp(0, 100)
```

## ファイル構成

```
src/domain/
├── __init__.py
└── control/
    ├── __init__.py
    ├── pid.py
    └── feedforward.py

tests/unit/
├── test_pid.py
└── test_feedforward.py
```
