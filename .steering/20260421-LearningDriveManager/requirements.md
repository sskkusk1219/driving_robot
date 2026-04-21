# 要求内容

## 概要

`LearningDriveManager` を実装する。速度×加速度グリッドのパターンを生成・実行し、収集したログから FF 運転モデル（pkl）を学習・保存するドメインクラス。

## 背景

`FeedforwardController` は `(ref_speed, ref_accel) → (accel_opening%, brake_opening%)` を補間する pkl モデルを必要とする。このモデルを実機走行ログから生成するのが `LearningDriveManager` の役割。現在 `src/domain/learning_drive.py` が存在しないため新規実装が必要。

## 実装対象の機能

### 1. パターン生成 (generate_patterns)
- `VehicleProfile` の `max_speed`, `max_accel_opening`, `max_brake_opening`, `max_decel_g` から安全な速度×加速度グリッドを生成
- 最大開度・最大減速 G を超えるパターンを自動スキップ

### 2. パターン走行 (run_pattern)
- 指定された `LearningPattern`（開度指令・保持時間）を実機に送信
- 実車速を読み取り、`LearningLog` を返す

### 3. モデル学習 (train_model)
- 収集した `LearningLog` リストから 2D グリッド補間モデルを構築
- `data/models/` 配下に pkl 保存し、ファイルパスを返す

## 受け入れ条件

### パターン生成
- [ ] 生成パターンが空でないこと
- [ ] 全パターンの `accel_opening <= profile.max_accel_opening`
- [ ] 全パターンの `brake_opening <= profile.max_brake_opening`
- [ ] max_decel_g を G 換算（9.81 * 3.6 km/h/s）超える減速パターンは含まれないこと

### パターン走行
- [ ] アクチュエータへ opening% に応じた位置指令を送信すること
- [ ] `hold_duration_s` 秒間保持後に `LearningLog` を返すこと
- [ ] Protocol 注入でハードウェアを差し替え可能なこと

### モデル学習
- [ ] pkl ファイルが `data/models/` に保存されること
- [ ] pkl に `speed_grid`, `accel_grid`, `accel_map`, `brake_map` キーが含まれること
- [ ] FeedforwardController がそのまま `load_model()` できること
- [ ] ログが不足してモデル構築できない場合に例外を送出すること

## 成功指標

- ユニットテストがハードウェアモックで全パス
- `ruff check` / `mypy` がエラーなし
- `FeedforwardController.load_model()` で読み込めることをテストで検証

## スコープ外

以下はこのフェーズでは実装しません:

- RobotController への組み込み（run_learning_drive はスタブのまま）
- Web UI / REST API
- 実機 CAN・Modbus 接続テスト
- 走行前チェック 6 項目の評価

## 参照ドキュメント

- `docs/functional-design.md` — LearningDriveManager コンポーネント設計・UC6
- `docs/architecture.md` — レイヤードアーキテクチャ・pkl モデル構造
