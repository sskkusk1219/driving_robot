---
# 要求定義: パターン生成・フィルタリング、モデル学習ロジック / FeedforwardController 補間モデル

## 機能概要

`LearningDriveManager` の以下3機能と `FeedforwardController` の補間モデルを実装する。

1. **パターン生成・フィルタリング** (`generate_patterns`)
   - 速度×加速度の2次元グリッドパターンを生成
   - `max_accel_opening / max_brake_opening / max_decel_g` を超えるパターンを除外

2. **モデル学習ロジック** (`train_model`)
   - 収集ログから2次元グリッドモデルを構築
   - `GridData` でスキャッタデータ → 正規グリッドに補間
   - `.pkl` 形式で保存

3. **FeedforwardController 補間モデル** (`load_model` / `predict`)
   - `.pkl` から `RegularGridInterpolator` を構築
   - `(ref_speed, ref_accel)` → `(accel_opening%, brake_opening%)` を線形補間

## 既存実装の状態（調査結果）

調査の結果、コア実装はすでに存在している：

| ファイル | クラス/関数 | 状態 |
|---------|------------|------|
| `src/domain/control/feedforward.py` | `FeedforwardController` | 実装済み・テスト通過 |
| `src/domain/learning_drive.py` | `LearningDriveManager.generate_patterns` | 実装済み・テスト通過 |
| `src/domain/learning_drive.py` | `LearningDriveManager.train_model` | 実装済み・テスト通過 |

ユニットテスト 28件すべて通過。

## 改善点（本実装で対処する）

1. **NaN 埋め戦略の改善**: `train_model` でフィルタアウトされたグリッド点が `0.0` で埋まる問題を、最近傍補間で埋める方式に改善する（`scipy.interpolate.NearestNDInterpolator` を使用）
2. **冗長なフィルタの除去**: `generate_patterns` 内の重複した `accel < 0 and abs(accel) > max_decel_kmhs` チェックを削除
3. **テスト追加**: NaN 埋め動作を検証するテストを追加
4. **pytest mark 登録**: `integration` マークが未登録によるワーニングを解消

## 非スコープ

- `RobotController.run_learning_drive()` の実装（別機能として別ステアリングで対応）
- FeedforwardController の RobotController への配線
