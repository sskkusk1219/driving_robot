# 要求内容

## 概要

WebSocket・LogWriter・ArchiveManager・LearningDriveManager の実装を完成させ、学習走行APIとフロントエンドを統合する。既存のコアMVP実装に続き、残存ギャップを埋める。

## 背景

コアMVP実装ステアリングで以下が完成した:
- ProfileRepository / ModeRepository / SessionRepository (DB + InMemory)
- WebSocket配信ループ (`src/web/ws.py`)
- LogWriter / ArchiveManager インフラ
- LearningDriveManager ドメイン実装
- 基本フロントエンド (HTML/CSS/JS)

しかし以下のギャップが残っている:
1. `RobotController.start_learning_drive()` が未実装（機能設計書に定義あり）
2. `/api/v1/drive/learning/start` エンドポイントが存在しない
3. WebSocket `broadcast_loop` の `accel_opening` / `brake_opening` がハードコード `0.0`（DriveLoop の実際の値が使われていない）
4. フロントエンドに学習走行ボタンが存在しない
5. 学習走行 API のユニットテストがない

## 実装対象の機能

### 1. 学習走行 API 実装
- `RobotController.start_learning_drive()` メソッドを追加
  - READY → PRE_CHECK → RUNNING 遷移
  - DriveSession(run_type='learning') を返す
- `/api/v1/drive/learning/start` POST エンドポイントを追加

### 2. WebSocket リアルタイム開度データ
- DriveLoop に `current_accel_opening` / `current_brake_opening` プロパティを追加
- `broadcast_loop` からこれらの値を取得して配信（ハードコード 0.0 を解消）

### 3. フロントエンド学習走行ボタン
- `index.html` に「学習運転 開始」ボタンを追加
- `app.js` に `/api/v1/drive/learning/start` 呼び出しを追加

### 4. テスト強化
- `test_robot_controller.py` に学習走行テストを追加
- `test_web_drive.py` に学習走行エンドポイントテストを追加
- `test_drive_loop.py` に開度プロパティのテストを追加

## 受け入れ条件

### 学習走行 API
- [ ] `RobotController.start_learning_drive()` が READY→PRE_CHECK→RUNNING の遷移を行う
- [ ] PreCheckFailed を正しく処理し READY に戻る
- [ ] DriveSession(run_type='learning') を返す
- [ ] `POST /api/v1/drive/learning/start` が 200 で DriveSession を返す
- [ ] 409 / 422 を正しく返す

### WebSocket 開度データ
- [ ] DriveLoop.current_accel_opening が最新の制御値を返す
- [ ] broadcast_loop が DriveLoop から開度を取得して配信する

### フロントエンド
- [ ] 「学習運転 開始」ボタンがダッシュボードに表示される
- [ ] ボタン押下で `/api/v1/drive/learning/start` を呼び出す

### テスト
- [ ] 全ユニットテストがパスする
- [ ] `ruff check` エラーゼロ
- [ ] `mypy src/` エラーゼロ

## スコープ外

- LearningDriveManager の実際の非同期ループ実行（DriveSession 生成のみ実装）
- DB統合テスト拡張（ArchiveManager の DB 統合テスト）

## 参照ドキュメント

- `docs/functional-design.md` - UC6: 学習運転、WebSocket仕様
- `docs/architecture.md` - 非同期実行アーキテクチャ
