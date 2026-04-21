# 要求仕様: 制御アルゴリズム実装

## 対象機能

`src/domain/control/` 配下の制御アルゴリズムクラス。ハードウェア不要で実装・テスト可能。

## 実装対象

| ファイル | クラス | 概要 |
|---------|-------|------|
| `src/domain/control/pid.py` | PIDController | PIDフィードバック制御 |
| `src/domain/control/feedforward.py` | FeedforwardController | 2Dグリッド補間モデル |
| `src/domain/__init__.py` | — | パッケージ初期化 |
| `src/domain/control/__init__.py` | — | パッケージ初期化 |
| `tests/unit/test_pid.py` | — | PID ユニットテスト |
| `tests/unit/test_feedforward.py` | — | FF ユニットテスト |

## 参照仕様 (docs/functional-design.md より)

### PIDController

```python
class PIDController:
    def __init__(kp: float, ki: float, kd: float, dt: float = 0.05)
    def update(setpoint: float, measurement: float) -> float
    def reset() -> None
```

制御則: `error = setpoint - measurement`
`output = Kp * error + Ki * ∫error * dt + Kd * d(error)/dt`

### FeedforwardController

```python
class FeedforwardController:
    def load_model(model_path: str) -> None
    def predict(ref_speed: float, ref_accel: float) -> tuple[float, float]
```

- 入力: (ref_speed [km/h], ref_accel [km/h/s]) の2次元グリッド
- 出力: (accel_opening [%], brake_opening [%])
- 補間: scipy.interpolate.RegularGridInterpolator（線形）
- ファイル形式: `.pkl`（numpy配列+グリッド座標）

## 制約

- `docs/development-guidelines.md` のコーディング規約に従う
- 型注釈必須、ruff/mypy クリーン
- モデルファイル未ロード時は適切な例外を送出
