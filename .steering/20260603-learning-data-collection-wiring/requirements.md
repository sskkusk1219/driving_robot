# 要求内容

## 概要

学習運転・自動走行で**走行ログ（`drive_logs`）が実際に DB へ保存される**ように配線し、学習運転終了後の自動モデル学習（先読み Ridge 逆モデル）が機能するようにする。現状はパターン生成・1パターン実行・FF制御・ログ書込の「部品」は存在するが、それらを実行・永続化する「組み立て（オーケストレーション）」が未実装。

## 背景（現状のギャップ）

調査により、現状どの経路でも `drive_logs` への書き込みが行われていないことが判明した。

| 箇所 | 現状 |
|---|---|
| `LearningDriveManager.generate_patterns` / `run_pattern` | ロジックは実装済み・テスト済み（`src/domain/learning_drive.py`） |
| `factory.py` / `stubs.py` | `learning_manager` を RobotController に**注入していない**（`self._learning_manager` は常に None） |
| `RobotController.start_learning_drive` | 状態を RUNNING にしてセッションを返すだけの**スタブ**。`generate_patterns`/`run_pattern`/ログ保存を呼ばない。`_active_learning_task` も実際には起動されない |
| Web `/start` → `start_auto_drive` | `mode_id` のみ渡し、mode/profile/log_writer 未指定 → `DriveLoop` が起動せず**自動走行でもログ未記録** |
| `LogWriter` | `DriveLoop` 内でしか使われず、Web 経由では `start_session`/`write_log` が呼ばれない |

このため UI 側（学習運転終了→自動学習トリガー、`screens/learning.js` 実装済み）は動くが、学習データが無いため学習APIは 422（サンプル不足）を返す。

## 重要な設計論点（要決定）

現行の運転モデルは **連続走行ログ（`DriveLog`、100ms周期の時系列）から先読み Ridge 逆モデルを学習する**設計（`.steering/20260531-ridge-inverse-model`）。
一方、既存の `run_pattern` は離散的な `LearningLog`（速度×加速度グリッドの1点ずつ）を返す**旧グリッドモデル時代の設計**。

→ データ収集をどちらに合わせるかを決める必要がある:
- **案A（推奨）**: 学習運転は「学習用の基準速度プロファイル（加減速を網羅した運転パターン）を連続走行」し、`DriveLoop` 同様に 100ms 周期で `DriveLog` を記録する。現行の連続ログ前提の学習と整合し、`run_pattern`/`generate_patterns` への依存を断てる。
- **案B**: `run_pattern` の離散実行を維持し、`LearningLog` を `DriveLog` 相当に変換して保存する。ただし「人間の運転に近い連続データ」という現行モデルの狙いと乖離する。

## 実装対象の機能

### 1. 学習運転のデータ収集オーケストレーション
- 学習運転中に運転パターンを実行し、走行ログ（`DriveLog`）を 100ms 周期で `drive_logs` に保存する
- 走行セッションを `drive_sessions` に `run_type='learning'` で作成する
- 完了・停止・非常停止で適切に終了し RUNNING→READY 遷移する
- 非同期タスク（`_active_learning_task`）として管理し、stop/emergency でキャンセルできる

### 2. 自動走行のログ記録配線
- Web `/start` から mode/profile/log_writer を解決して `start_auto_drive` に渡し、`DriveLoop` を起動して走行ログを記録する

### 3. 依存注入（factory / stubs）
- `learning_manager` と学習用 `LogWriter` を RobotController（または起動経路）に注入する

## 受け入れ条件

### 学習運転データ収集
- [ ] 学習運転を開始すると `drive_sessions`(run_type='learning') が1件作成される
- [ ] 学習運転中、走行ログが `drive_logs` に 100ms 周期で保存される
- [ ] 学習運転を停止/完了すると RUNNING→READY に遷移し、UI の自動学習が成功する（サンプル十分時）
- [ ] 非常停止時はログ保存を中断し原点復帰する

### 自動走行ログ記録
- [ ] Web `/start` 経由の自動走行で `DriveLoop` が起動し、走行ログが `drive_logs` に保存される

### 依存注入
- [ ] factory / stubs で必要な依存（learning_manager・LogWriter）が注入される

## 成功指標
- 学習運転 → 終了 → 自動学習 が実データ（収集ログ）で成功し、`model_path`・`feedforward_params` がプロファイルに保存される
- `pytest tests/unit/` 全通過、ruff/mypy が本変更分でクリーン

## スコープ外
- 実機（アクチュエータ・CAN）での精度検証・PID/FF チューニング
- 学習用基準速度プロファイルの内容設計（案A採用時は別途定義）

## 参照ドキュメント
- `docs/product-requirements.md`（3. 学習運転 / 4. 運転モデル学習 / 6. 自動運転）
- `.steering/20260531-ridge-inverse-model/`（連続ログからの Ridge 逆モデル）
- `.steering/20260603-feedforward-dynamics-params/`（FF 物理定数）
- `src/domain/control/drive_loop.py`（連続ログ記録の既存実装パターン）
