# タスクリスト: WebSocket・LogWriter・ArchiveManager・テスト強化・フロントエンド・DB統合テスト・学習走行の実装

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

---

## フェーズ1: DriveLoop 開度プロパティ追加

- [x] `src/domain/control/drive_loop.py` に開度プロパティを追加
  - [x] `_current_accel_opening: float = 0.0` / `_current_brake_opening: float = 0.0` インスタンス変数追加
  - [x] `_execute_one_cycle()` で開度算出後に変数を更新
  - [x] `@property current_accel_opening` / `current_brake_opening` を追加

- [x] `tests/unit/test_drive_loop.py` に開度プロパティのテストを追加
  - [x] 初期値が 0.0 であることのテスト
  - [x] 制御サイクル後に更新されることのテスト

## フェーズ2: RobotController - start_learning_drive() 追加

- [x] `src/app/robot_controller.py` に学習走行を追加
  - [x] `LearningDriveManagerProtocol` Protocol クラスを定義
  - [x] コンストラクタに `learning_manager: LearningDriveManagerProtocol | None = None` を追加
  - [x] `_active_learning_task: asyncio.Task | None = None` フィールドを追加
  - [x] `start_learning_drive()` メソッドを実装（READY → PRE_CHECK → RUNNING 遷移）
  - [x] `shutdown()` で `_active_learning_task` のキャンセル対応

- [x] `tests/unit/test_robot_controller.py` に学習走行テストを追加
  - [x] READY → RUNNING 遷移のテスト
  - [x] PreCheckFailed で READY に戻るテスト
  - [x] DriveSession(run_type='learning') を返すテスト

## フェーズ3: drive.py - /learning/start エンドポイント追加

- [x] `src/web/routers/drive.py` に `/learning/start` エンドポイントを追加
  - [x] `POST /api/v1/drive/learning/start`
  - [x] 成功時: DriveSessionResponse を返す
  - [x] InvalidStateTransition → 409
  - [x] PreCheckFailed → 422

- [x] `tests/unit/test_web_drive.py` に学習走行エンドポイントテストを追加
  - [x] 200 正常ケース
  - [x] 409 InvalidStateTransition ケース
  - [x] 422 PreCheckFailed ケース

## フェーズ4: WebSocket 開度取得修正

- [x] `src/web/ws.py` の `broadcast_loop` を修正
  - [x] `controller._drive_loop` が None でない場合、`current_accel_opening` / `current_brake_opening` を取得
  - [x] None の場合は 0.0 を維持（既存動作と同じ）

## フェーズ5: フロントエンド - 学習走行ボタン追加

- [x] `src/web/static/index.html` にボタンを追加
  - [x] `<button id="btn-learning-start" class="btn-warning">学習運転 開始</button>` を追加

- [x] `src/web/static/js/app.js` にハンドラを追加
  - [x] `btn-learning-start` の click ハンドラ（`/api/v1/drive/learning/start` POST）

## フェーズ6: 品質チェックと修正

- [x] すべてのユニットテストが通ることを確認
  - [x] `python -m pytest tests/unit/ -q` → 370 passed, 4 warnings
- [x] リントエラーがないことを確認
  - [x] `python -m ruff check src/ tests/` → All checks passed!
- [x] 型エラーがないことを確認
  - [x] `python -m mypy src/` → Success: no issues found in 43 source files

## フェーズ7: 振り返り

- [x] tasklist.md の振り返りセクションを更新

---

## 実装後の振り返り

### 実装完了日
2026-04-22

### 計画と実績の差分

**計画と異なった点**:
- `LearningDriveManagerProtocol` は Protocol クラスとして定義したが、現時点では空ボディ（`...`のみ）。将来のメソッド追加は `run_pattern()` / `train_model()` 等を追加する予定。
- `stop()` での `_active_learning_task` キャンセルは `shutdown()` でのみ対応（`stop()` は RUNNING/MANUAL からの正常停止なので学習タスクとは排他）
- 実装検証の結果、`ws.py` の `getattr` によるプライベート属性アクセスを指摘され、`RobotController.current_openings` プロパティを追加してレイヤー境界を修正した

**新たに必要になったタスク**:
- `RobotController.current_openings` プロパティ追加（実装検証で `getattr` アクセスのアーキテクチャ問題が指摘されたため）
- `TestShutdown` に `_active_learning_task` キャンセルのテストを追加（実装検証でカバレッジの穴を指摘されたため）
- `test_properties_updated_after_cycle` のアサーションを具体的な値で検証するよう修正

### 学んだこと

**技術的な学び**:
- `getattr(controller, "_drive_loop", None)` を使えばプライベート属性の安全な参照が可能
- DriveLoop の開度プロパティを `float = 0.0` で初期化しておけば、WebSocket 側でのデフォルト値処理が不要になる
- 学習走行は自動走行と同じ PRE_CHECK フローを経るため、既存パターンをほぼそのまま流用できた

**プロセス上の改善点**:
- 未実装ギャップの発見は `grep` で全ファイルを検索するよりも、機能設計書と実装ファイルを対照することで効率的に特定できた

### 次回への改善提案
- 学習走行の `_active_learning_task` を実際に `asyncio.create_task()` で起動し、LearningDriveManager.run_pattern() ループを実装する
- WebSocket の開度配信が DriveLoop の `_drive_loop` に依存しているため、将来的には学習走行中の開度もリアルタイム配信できるよう拡張を検討
- `test_drive_loop.py` の警告（coroutine never awaited）は既存テストの問題で今回の変更とは無関係だが、修正を検討する価値あり
