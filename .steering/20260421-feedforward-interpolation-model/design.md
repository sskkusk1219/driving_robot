---
# 設計: パターン生成・フィルタリング、モデル学習ロジック / FeedforwardController 補間モデル

## アーキテクチャ上の位置づけ

```
ドメインレイヤー
├── src/domain/learning_drive.py   ← LearningDriveManager (パターン生成・学習)
└── src/domain/control/
    └── feedforward.py             ← FeedforwardController (補間モデル)
```

## 変更内容

### 1. `train_model` NaN 埋め改善

**現状**: `np.nan_to_num(arr, nan=0.0)` で全NaNを0埋め

**問題**: フィルタアウトされたパターン（例: 高速×高減速）のグリッド点が `0.0` になる。
`FeedforwardController.predict()` でその領域が参照されると誤った0%開度が返る可能性がある。

**改善**: `NearestNDInterpolator` で最近傍値を使って補完する。

```python
from scipy.interpolate import NearestNDInterpolator

def _fill_nan_nearest(map_: np.ndarray, grid_speed: np.ndarray, grid_accel: np.ndarray) -> np.ndarray:
    """NaN 領域を最近傍既知値で埋める。"""
    known = ~np.isnan(map_)
    if not np.any(known):
        return np.zeros_like(map_)
    nn = NearestNDInterpolator(
        np.column_stack([grid_speed[known], grid_accel[known]]),
        map_[known],
    )
    result = map_.copy()
    nan_mask = np.isnan(result)
    result[nan_mask] = nn(grid_speed[nan_mask], grid_accel[nan_mask])
    return result
```

`train_model` 内で `nan_to_num` の代わりに `_fill_nan_nearest` を使用。

### 2. 冗長フィルタ除去

`generate_patterns` 内の以下のフィルタは `accel_points` の範囲定義と重複：

```python
# 削除対象（arange が -max_decel_kmhs から始まるため常に偽）
if accel < 0 and abs(accel) > max_decel_kmhs:
    continue
```

ただし `np.arange` の浮動小数点精度で稀に境界値が滲む可能性があるため、
`abs(accel) > max_decel_kmhs + 1e-9` に変更して安全マージンを持ったフィルタとして残す。

### 3. テスト追加

`tests/unit/test_learning_drive.py` に `TestTrainModel` クラスへ追加：

```python
def test_nan_regions_filled_with_nearest_neighbor(self) -> None:
    """グリッドの一部が欠損しても 0.0 ではなく最近傍値で埋まること。"""
    ...
```

### 4. pytest integration mark 登録

`pyproject.toml` の `[tool.pytest.ini_options]` に `markers` を追加。

## 変更ファイル一覧

| ファイル | 変更種別 |
|---------|---------|
| `src/domain/learning_drive.py` | 改善 (NaN埋め・冗長フィルタ) |
| `tests/unit/test_learning_drive.py` | テスト追加 |
| `pyproject.toml` | mark 登録 |
