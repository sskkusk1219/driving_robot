# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: データモデル実装

- [x] `src/models/learning_drive.py` を作成する
  - [x] `LearningPattern` dataclass を実装
  - [x] `LearningLog` dataclass を実装

## フェーズ2: ドメインロジック実装

- [x] `src/domain/learning_drive.py` を作成する
  - [x] `LearningActuatorProtocol` Protocol を定義
  - [x] `LearningCANProtocol` Protocol を定義
  - [x] `LearningDataError` カスタム例外を定義
  - [x] `LearningDriveConfig` dataclass を定義（グリッドパラメータ定数）
  - [x] `LearningDriveManager.__init__` を実装
  - [x] `generate_patterns(profile)` を実装
    - [x] 速度グリッド・加速度グリッドの生成
    - [x] 初期開度の線形マッピング計算
    - [x] max_opening / max_decel_g によるフィルタリング
  - [x] `run_pattern(pattern, accel_driver, brake_driver, can_reader, calibration)` を実装
    - [x] 開度 % → pulse 換算
    - [x] move_to_position 送信 + hold_duration_s 待機
    - [x] 実車速サンプリングと LearningLog 生成
  - [x] `train_model(logs, profile_id, output_dir)` を実装
    - [x] ログ不足バリデーション（LearningDataError）
    - [x] speed_grid / accel_grid 構築
    - [x] scipy.griddata で accel_map / brake_map を補間
    - [x] pkl 保存（パストラバーサル対策済み）
    - [x] pkl パスを返す

## フェーズ3: ユニットテスト実装

- [x] `tests/unit/test_learning_drive.py` を作成する
  - [x] モックアクチュエータ / モック CAN リーダーを定義
  - [x] `generate_patterns` のテスト
    - [x] 生成パターンが空でないこと
    - [x] max_accel_opening を超えるパターンが含まれないこと
    - [x] max_brake_opening を超えるパターンが含まれないこと
    - [x] max_decel_g 超過の減速パターンが除外されること
  - [x] `run_pattern` のテスト
    - [x] アクチュエータへ位置指令が送信されること
    - [x] LearningLog が正しい型で返ること
    - [x] 実車速が LearningLog に記録されること
  - [x] `train_model` のテスト
    - [x] pkl ファイルが保存されること
    - [x] pkl に必要キーが含まれること
    - [x] FeedforwardController.load_model() で読み込めること
    - [x] ログ不足で LearningDataError が送出されること

## フェーズ4: 品質チェックと修正

- [x] 全テストが通ることを確認
  - [x] `python -m pytest tests/unit/test_learning_drive.py -v` (18/18 passed)
- [x] 既存テストが壊れていないこと
  - [x] `python -m pytest tests/unit/ -v` (166/166 passed)
- [x] ruff チェック
  - [x] `python -m ruff check src/domain/learning_drive.py src/models/learning_drive.py` (All checks passed)
- [x] mypy 型チェック
  - [x] `python -m mypy src/domain/learning_drive.py src/models/learning_drive.py` (no issues found)

## フェーズ5: 振り返り

- [x] tasklist.md の振り返りセクションを記録

---

## 実装後の振り返り

### 実装完了日
2026-04-21

### 計画と実績の差分

**計画と異なった点**:
- `run_pattern` の引数に `CalibrationData` を追加（設計書では省略していたが、開度→pulse換算に必須と判明）
- `import asyncio` をメソッド内ローカルインポートにした（トップレベルに置いても問題ないが既存パターンに合わせた）

**新たに必要になったタスク**:
- なし（計画どおり）

### 学んだこと

**技術的な学び**:
- `scipy.griddata` は不規則点群を正規グリッドに補間できるため、疎な学習ログでも `RegularGridInterpolator` 向けの正規グリッドを生成できる
- `np.nan_to_num` で外挿域の NaN を 0 に置き換えることで、`FeedforwardController` がそのまま読み込める pkl 構造を維持できた
- パストラバーサル対策は `Path(profile_id).name` で profile_id のベース名のみを取り出すことで簡潔に実現できた

**プロセス上の改善点**:
- 設計書のデータフロー記述に `run_pattern` の引数リストを明記すると実装時の迷いがなくなる

### 次回への改善提案
- `RobotController.run_learning_drive()` に `LearningDriveManager` を組み込む際、`CalibrationData` の取得経路（ProfileRepository）を先に設計しておくとスムーズ
- パターン数が多い場合（max_speed=200 など）はグリッド刻みを粗くするオプションが必要になる可能性がある
