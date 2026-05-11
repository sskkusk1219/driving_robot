# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

---

## フェーズ1: RobotController 更新

- [x] `CalibrationManagerProtocol` を `robot_controller.py` に追加
- [x] `RobotController.__init__()` に `calibration_manager` パラメータ追加
- [x] `RobotController.run_calibration()` を CalibrationManager に委譲するよう更新

## フェーズ2: Web API 追加

- [x] `CalibrationDataResponse` スキーマを `schemas.py` に追加
- [x] `CalibrationResultResponse` スキーマを `schemas.py` に追加
- [x] `POST /api/v1/drive/calibrate` エンドポイントを `drive.py` に追加

## フェーズ3: ファクトリ更新

- [x] `factory.py` で `CalibrationManager` を生成して `RobotController` に渡す

## フェーズ4: テスト追加

- [x] `test_robot_controller.py` に CalibrationManager 委譲テストを追加
  - [x] `calibration_manager` が呼ばれることを確認するテスト
  - [x] `calibration_manager=None` でも状態遷移が正常なテスト
- [x] `test_web_drive.py` に `/calibrate` エンドポイントテストを追加
  - [x] 成功時 200 + CalibrationResultResponse のテスト
  - [x] 不正状態での 409 のテスト

## フェーズ5: 品質チェック

- [x] すべてのテストが通ることを確認
  - [x] `python -m pytest tests/unit/ -q` → 314 passed
- [x] リントエラーがないことを確認
  - [x] `python -m ruff check src/ tests/` → All checks passed
- [x] 型エラーがないことを確認
  - [x] `python -m mypy src/` → Success: no issues found in 40 source files

## フェーズ6: ドキュメント更新

- [x] 実装後の振り返り（このファイルの下部に記録）

---

## フェーズ7: ProfileRepository Protocol 注入（キャリブレーション永続化）

### 7-1: ドメイン層更新 (`src/domain/calibration.py`)

- [x] `ProfileRepositoryProtocol` を追加 (`save_calibration(profile_id, data) -> None`)
- [x] `CalibrationManager.__init__` に `profile_repo: ProfileRepositoryProtocol | None = None` を追加
- [x] `run_calibration` でバリデーション成功時に `await self._profile_repo.save_calibration(profile_id, data)` を呼び出す
- [x] `# noqa: ARG002` を削除（profile_id が実際に使用されるため）
- [x] TODO コメントを docstring から削除

### 7-2: インフラ層新規実装 (`src/infra/profile_repository.py`)

- [x] `ProfileRepository` クラスを `asyncpg.Connection` を受け取る形で実装
- [x] `save_calibration`: `calibration_data` テーブルへ UPSERT

### 7-3: テスト追加 (`tests/unit/test_calibration.py`)

- [x] バリデーション成功時に `profile_repo.save_calibration` が呼ばれることのテスト
- [x] バリデーション失敗時には `profile_repo.save_calibration` が呼ばれないことのテスト
- [x] `profile_repo=None` でもエラーなく動作するテスト
- [x] `save_calibration` が例外を投げた場合に呼び出し元まで伝播するテスト

### 7-4: 品質チェック

- [x] `python -m pytest tests/unit/ -q` → 319 passed
- [x] `python -m ruff check src/ tests/` → All checks passed
- [x] `python -m mypy src/` → Success: no issues found in 41 source files

### 7-5: 振り返り記録

- [x] 実装後の振り返りを記録

---

## フェーズ7 振り返り

### 実装完了日
2026-04-22

### 計画と実績の差分

**計画と異なった点**:
- `profile_repository.py` の docstring が 100 文字を超えて ruff E501 エラーが発生。短縮して解決。
- `test_save_calibration_exception_propagates` を最初に try/except で書いたが、
  例外が起きない場合に無条件 pass になる弱いテストになるため `pytest.raises` に修正した。

**新たに必要になったタスク**:
- なし

### 学んだこと

- `ProfileRepositoryProtocol` をドメイン層 (`calibration.py`) に定義することで、
  インフラ層への直接依存を避けながらテスト可能な設計を維持できた。
- `ON CONFLICT (profile_id) DO UPDATE` の UPSERT パターンは、
  同一プロファイルへの再キャリブレーションを安全に上書きできる。

### 次回への改善提案
- `factory.py` でDB接続確立後に `ProfileRepository` を生成して
  `CalibrationManager` に注入することで実機の永続化が完成する。
  現状 `profile_repo=None` のため走行前チェック #3 はまだ通らない。
- 接続プールからの Connection 取得タイミング（キャリブレーション開始時）を
  アプリケーション層で設計する必要がある。

---

## 実装後の振り返り

### 実装完了日
2026-04-22

### 計画と実績の差分

**計画と異なった点**:
- `schemas.py` に新しいスキーマを追加した際にインポート順が乱れ（E402）、`ruff` エラーが 1 件発生した。ファイル先頭への移動で即解決。
- `test_robot_controller.py` のテストで `__import__` を誤って使用したため、validator の指摘で修正。テスト上部の既存 import を活用すれば良かった。

**新たに必要になったタスク**:
- なし（計画通り完了）

### 学んだこと

**技術的な学び**:
- `schemas.py` への新しいモデル追加は既存 import の前に挿入しないよう注意が必要。ファイル全体のトップに import を集約するのが安全。
- Protocol クラスをアプリケーション層（`robot_controller.py`）に定義することで、ドメイン層（`CalibrationManager`）への直接依存を避けられる。既存の `ActuatorDriverProtocol` 等と同じパターン。

### 次回への改善提案
- キャリブレーション結果の VehicleProfile 永続化（`CalibrationManager.run_calibration()` の TODO）を次フェーズで実装することで、走行前チェック #3 が通る完全なフローが実現できる。
- `/calibrate` エンドポイントの一般例外ハンドリング（Modbus 通信断での 500 変換）は、他エンドポイントの方針確定後に統一実装を検討する。
