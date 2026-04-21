# 要求内容

## 概要

自動走行時の 50ms 制御ループ（`DriveLoop`）を実装し、`RobotController` に統合する。
FF 制御 + PID フィードバック制御・安全チェック・100ms 周期ログ記録をループ内で実行する。

## 背景

`RobotController.start_auto_drive()` は現時点ではセッション生成のみのスタブであり、
実際の 50ms 制御ループ（CAN 車速取得 → 開度算出 → アクチュエータ指令 → 安全チェック）が未実装。
`docs/architecture.md` で `src/domain/control/drive_loop.py` が設計上想定されているが存在しない。

## 実装対象の機能

### 1. DriveLoop（50ms 制御ループコア）

- `asyncio.get_event_loop().call_later` で 50ms 周期を維持（asyncio.sleep は使わない）
- 各サイクルで以下を実行:
  1. 経過時間を確認し、走行モード終了なら正常停止コールバックを呼ぶ
  2. DrivingMode から基準車速・基準加速度を線形補間で取得
  3. CAN から実車速を非同期取得
  4. FeedforwardController で FF 開度を算出
  5. PIDController で偏差補正量を算出
  6. FF + PID 合成 → アクセル優先排他制御 → プロファイル最大値クランプ
  7. 開度[%] → アクチュエータ位置[pulse] に変換（CalibrationData 使用）
  8. 両軸に位置指令と電流読み取りをドライバごとに逐次・両軸間で並列実行
  9. 過電流・逸脱超過の安全チェック → 異常時に緊急停止コールバックを呼ぶ
  10. 2 サイクルごと（100ms）に LogWriter へ非同期書き込み

### 2. RobotController への統合

- `start_auto_drive()` に `mode: DrivingMode | None` / `profile: VehicleProfile | None` オプション引数を追加
- DrivingMode・VehicleProfile・CalibrationData・FeedforwardController が揃っている場合に DriveLoop を起動
- `stop_auto_drive()` / `stop()` / `emergency_stop()` で DriveLoop を停止

## 受け入れ条件

### DriveLoop

- [ ] 50ms 周期に `call_later` を使用し `asyncio.sleep` を使わない
- [ ] 走行モード終了（elapsed >= total_duration）で `on_complete` コールバックが呼ばれる
- [ ] 基準車速が SpeedPoint 間で線形補間される
- [ ] 基準加速度が隣接 SpeedPoint の速度差/時間差で計算される
- [ ] FF + PID 合成式の通りに開度が計算される（functional-design.md 記載式）
- [ ] アクセル・ブレーキ両方に開度がある場合にアクセル優先でブレーキがゼロになる
- [ ] 最終開度が [0, max_accel_opening] / [0, max_brake_opening] にクランプされる
- [ ] 開度[%] が `zero_pos + round((full_pos - zero_pos) * opening / 100)` で変換される
- [ ] 過電流検知時に `on_emergency` が呼ばれる
- [ ] 逸脱継続時間超過時に `on_emergency` が呼ばれる
- [ ] CAN 読み取り失敗時に `on_emergency` が呼ばれる
- [ ] 2 サイクルごとに `log_writer.write_log()` が呼ばれる

### RobotController 統合

- [ ] `start_auto_drive()` に `mode` / `profile` 引数追加後も既存テストがパスする
- [ ] `stop_auto_drive()` 呼び出しで DriveLoop が停止する
- [ ] `emergency_stop()` 呼び出しで DriveLoop が停止する

## 成功指標

- ユニットテストカバレッジ: `src/domain/control/drive_loop.py` で 80% 以上
- `pytest tests/unit/`, `ruff check src/ tests/`, `mypy src/` がすべてパス

## スコープ外

以下はこのフェーズでは実装しません:

- 実機 Modbus RTU / CAN 通信の動作確認（HW 結合テストは別タスク）
- LogWriter の DB 書き込み統合テスト
- DrivingMode の DB 取得（プロファイル・モードの DB 連携は別フェーズ）
- 学習運転ループ（LearningDriveManager）

## 参照ドキュメント

- `docs/functional-design.md` - FF+PID 合成式・制御ループ仕様
- `docs/architecture.md` - call_later ループ実装パターン・タイムバジェット
- `docs/development-guidelines.md` - コーディング規約・安全コードの原則
