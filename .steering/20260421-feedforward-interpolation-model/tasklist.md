---
# タスクリスト: パターン生成・フィルタリング、モデル学習ロジック / FeedforwardController 補間モデル

## フェーズ1: 実装改善

- [x] `train_model` の NaN 埋め戦略を `_fill_nan_nearest` で改善
- [x] `generate_patterns` の冗長フィルタを安全マージン付きに変更
- [x] pyproject.toml に pytest `integration` mark を登録

## フェーズ2: テスト追加

- [x] `TestTrainModel` に NaN 埋め動作のテストを追加
- [x] 全ユニットテストが通過することを確認

## フェーズ3: 品質確認

- [x] ruff lint / format チェック
- [x] mypy 型チェック

---

## 実装後の振り返り

**実装完了日**: 2026-04-21

**計画と実績の差分**:
- コア実装（`FeedforwardController`, `generate_patterns`, `train_model`）は既に存在していた。実装すべき具体的改善点を洗い出して対処した。
- `_fill_nan_nearest` ヘルパーの実装は想定通り。
- `generate_patterns` の冗長フィルタ修正は浮動小数点安全マージン付きに変更（`1e-9`）。

**実施内容**:
1. `train_model` NaN埋め: `nan_to_num(nan=0.0)` → `_fill_nan_nearest` (最近傍補間)
2. `generate_patterns` フィルタ: 浮動小数点安全マージン `1e-9` を付与
3. `pyproject.toml`: `integration` pytest mark 登録
4. テスト追加: `test_nan_regions_filled_with_nearest_neighbor_not_zero`

**学んだこと**:
- `griddata(method="linear")` は凸包外を NaN にする。`FeedforwardController` 側は `fill_value=None` で最近傍を使うが、モデルファイル側でも NaN を除去しておく方が安全。
- 最近傍補間は `NearestNDInterpolator` で簡潔に実装できる。

**次回への改善提案**:
- `RobotController.run_learning_drive()` の実装が残っている（別ステアリングで対応）
- 学習済みモデルを `VehicleProfile.model_path` へ自動更新するロジックが未実装
