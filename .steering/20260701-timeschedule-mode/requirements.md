# 要求内容: タイムスケジュールモードの実装

## 背景

`docs/functional-design.md` / `docs/product-requirements.md` に Post-MVP 仕様として定義済みの
「タイムスケジュール（統合タイムライン）」を実装する。基準車速追従（自動運転）ではなく、
時系列でアクセル・ブレーキ開度（ペダル）と物理ボタン押下（ボタンサーボ）を開ループで
再生する機能。Phase 0（PCA9685+SG90 単体疎通確認スクリプト）は
`.steering/20260701-servo-pca9685-smoke-test/` で完了済み。本作業は Phase 1。

## スコープ（今回やること）

### データモデル・永続化
- `TimeSchedule` / `PedalPoint` / `ButtonEvent` エンティティ（`docs/functional-design.md` 準拠）
- `time_schedules` テーブル（DDL・冪等マイグレーション）
- `ScheduleRepository`（CRUD、`ModeRepository` と同一パターン）＋ in-memory 版

### ハードウェア抽象
- `ButtonServoDriver`（PCA9685+SG90、Protocol ベース、`connect/press/release_all`）
  - 実装は Phase 0 の `_Pca9685`（smbus2 直叩き）を発展させる
  - 非ハード環境向けスタブ（`_StubButtonServo`）

### 実行制御
- `ScheduleLoop`（統合タイムライン開ループ実行）
  - ペダル開度を時系列補間して位置指令、ボタンイベントを time_s で発火
  - loop 再生、on_complete/on_emergency、過電流安全、連続ログ記録
- `RobotController` 統合: `start_schedule_drive`/`stop_schedule_drive`、
  非常停止/停止時の `release_all`、リアルタイム I/F への組み込み

### 走行前チェック
- 項目8「ボタンサーボ確認」（タイムスケジュール実行時のみ）

### Web API
- `/api/v1/schedules/` CRUD（JSON ボディ）
- `/api/v1/drive/schedule/start`・`/stop`（コマンド）
- スキーマ・DI（`get_schedule_repo`）・ルーター登録・factory/stub 配線

### フロントエンド
- `ScheduleScreen`（Post-MVP プレースホルダを実機能へ差し替え）
  一覧・JSON 作成・開始/停止/削除

### テスト
- モデル・ButtonServoDriver・ScheduleLoop・ScheduleRepository（in-memory）の単体テスト
- schedules CRUD / schedule start-stop の統合テスト

## スコープ外（今回やらないこと）

- 実機での PCA9685 動作検証（別途ハードウェア結合テストで実施）
- `SequenceScreen`（シーケンスモード）は Post-MVP プレースホルダのまま
- タイムラインのグラフィカルエディタ（ペダル/ボタンを可視化する高度な UI）

## 制約・前提

- Python 3.13 / FastAPI / asyncpg / smbus2（新規依存追加なし）
- PCA9685 の I2C バスはアクセル・ブレーキの Modbus RS-485 と独立（50ms 制御と非競合）
- 用語: SG90 は「ボタンサーボ」。IAI アクチュエータの「サーボON/サーボ状態」とは区別する
- 安全: 非常停止・エラー時は全ボタンサーボを待機位置へ戻す（`release_all`）
