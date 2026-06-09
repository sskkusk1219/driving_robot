# 設計書（記録）

> 本ステアリングは**バックログ記録**であり、未着手。実装着手時に design を確定・更新する。

## アーキテクチャ概要

走行ログの収集・永続化を Web/app 経路に正しく接続する。学習運転は案A（連続走行ログ）を推奨方針として設計するが、着手時に最終決定する。

```
[学習] /learning/start
   → controller.start_learning_drive(profile, log_writer)
   → LogWriter.start_session(profile_id, run_type='learning') で drive_sessions 作成
   → _active_learning_task 起動:
        学習用基準プロファイル or generate_patterns を実行
        100ms 周期で DriveLog を write_log
   → 完了/stop/emergency で停止・session 終了・READY
[学習終了] UI(screens/learning.js) が RUNNING→READY を検知 → /learning/train（実装済み）
   → list_logs_for_training(profile_id) が収集ログを取得 → 学習成功

[自動走行] /start
   → mode/profile/log_writer を解決して start_auto_drive へ
   → DriveLoop 起動 → 100ms 周期で write_log
```

## コンポーネント設計（案）

### 1. `RobotController.start_learning_drive`
- 引数に `log_writer` を受け取る（または保持済みを使う）
- `LogWriter.start_session(...run_type='learning')` で session_id を採番
- 非同期タスクを起動し、運転パターンを実行しながら `DriveLog` を記録
- `stop_learning` / emergency でタスクを cancel し session を `completed`/`emergency` で終了
- **要決定**: 案A=連続走行ログ（DriveLoop 流用 or 専用ループ） / 案B=run_pattern の LearningLog を DriveLog へ変換保存

### 2. Web `/start`（`routers/drive.py`）
- `profile_repo`/`mode_repo`/`db_pool` から mode・profile・LogWriter を解決
- `start_auto_drive(mode_id, mode=..., profile=..., log_writer=...)` に渡す
- セッション終了時に `end_session` を呼ぶ経路を整理

### 3. `LogWriter` 配線
- `db_pool` から接続を取得して `LogWriter` を生成（auto/learning 共通）
- `app.state.db_pool` 経由（in-memory 時はログ無効でスキップ）

### 4. factory / stubs
- `LearningDriveManager` を生成し RobotController に注入（案A採用時は不要になる可能性あり）
- 実機は `LogWriter`、stub はログ無効 or in-memory

## データモデル整合（重要）
- 学習データは最終的に `train_inverse_model` が使う `DriveLog`（連続時系列）であること
- `run_pattern` の `LearningLog` を使う場合は変換層が必要（案B）

## テスト戦略（案）
- `test_robot_controller`: start_learning_drive がセッション作成・ログ書込・状態遷移する
- `test_web_drive`: /start が DriveLoop を起動しログ記録する（モック LogWriter で検証）
- integration: 実 DB でセッション＋ログが保存される

## 依存ライブラリ
追加なし。

## 実装順序（案）
1. データモデル方針（案A/B）決定
2. LogWriter の app/factory 配線
3. 学習運転オーケストレーション実装
4. 自動走行ログ記録配線
5. テスト・品質チェック
6. 実機での収集→学習→紐付け確認
