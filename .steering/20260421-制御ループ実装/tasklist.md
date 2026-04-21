# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: DriveLoop コア実装

- [x] `src/domain/control/drive_loop.py` を新規作成
  - [x] `SafetyCheckProtocol` 定義（check_overcurrent, check_deviation）
  - [x] `LogWriterProtocol` 定義（write_log）
  - [x] `ActuatorDriverProtocol` 定義（move_to_position, read_current）
  - [x] `CANReaderProtocol` 定義（read_speed）
  - [x] `DriveLoop.__init__` 実装
  - [x] `DriveLoop.start()` 実装（call_later ベース）
  - [x] `DriveLoop.stop()` 実装
  - [x] `DriveLoop._schedule_next_cycle()` 実装
  - [x] `DriveLoop._execute_one_cycle()` 実装
  - [x] `DriveLoop._get_ref_speed_and_accel()` 実装（線形補間）
  - [x] `DriveLoop._opening_to_position()` 実装
  - [x] `DriveLoop._drive_accel_axis()` / `_drive_brake_axis()` 実装

## フェーズ2: RobotController 統合

- [x] `src/app/robot_controller.py` を拡張
  - [x] `FeedforwardController` インポート追加
  - [x] `DriveLoop` / `LogWriterProtocol` インポート追加
  - [x] `SafetyCheckProtocol` Protocol を robot_controller.py に追加
  - [x] `__init__` に `ff_controller` / `safety_check` オプション引数追加
  - [x] `_drive_loop: DriveLoop | None = None` フィールド追加
  - [x] `start_auto_drive()` に `mode` / `profile` / `log_writer` オプション引数追加
  - [x] DriveLoop 起動ロジック実装（条件チェック + セッション ID 連携）
  - [x] `stop_auto_drive()` に `_drive_loop.stop()` 追加
  - [x] `stop()` に `_drive_loop.stop()` 追加
  - [x] `emergency_stop()` に `_drive_loop.stop()` 追加

## フェーズ3: `_StubActuator` に `move_to_position` 追加

- [x] `src/app/stubs.py` の `_StubActuator` に `move_to_position(pos: int) -> None` を追加

## フェーズ4: ユニットテスト

- [x] `tests/unit/test_drive_loop.py` を新規作成
  - [x] `_get_ref_speed_and_accel` テスト（補間・境界値・単点モード）
  - [x] `_opening_to_position` テスト（0%, 50%, 100%）
  - [x] FF+PID 合成・排他制御・クランプのテスト
  - [x] 正常完了（on_complete）テスト
  - [x] 過電流検知（on_emergency）テスト
  - [x] 逸脱超過（on_emergency）テスト
  - [x] CAN エラー（on_emergency）テスト
  - [x] ログ書き込み 2 サイクル 1 回テスト

## フェーズ5: 品質チェックと修正

- [x] すべてのテストが通ることを確認
  - [x] `python -m pytest tests/unit/ tests/integration/ -v` → 287 passed
- [x] リントエラーがないことを確認
  - [x] `python -m ruff check src/ tests/` → All checks passed
- [x] 型エラーがないことを確認
  - [x] `python -m mypy src/` → Success: no issues found in 38 source files
- [x] フォーマット適用
  - [x] `python -m ruff format src/ tests/`

---

## 実装後の振り返り

### 実装完了日
2026-04-21

### 計画と実績の差分

**計画と異なった点**:
- `ActuatorDriverProtocol` に `move_to_position` が不足していたため `robot_controller.py` 側の Protocol にも追加が必要だった（mypy で検出）
- `test_accel_priority_exclusive_control` など 3 件のテストで `dl._running = True` の設定漏れがあり修正が必要だった

**新たに必要になったタスク**（バリデーション指摘対応）:
- `emergency_stop()` の冪等性対応（EMERGENCY→EMERGENCY 遷移ガード追加）
- `stop()` / `stop_auto_drive()` / `stop_manual()` の `home_return` を `asyncio.gather` に統一
- `asyncio.get_event_loop()` → `asyncio.get_running_loop()` に変更（3箇所）

### 学んだこと

**技術的な学び**:
- `call_later` ベースのスケジューリングでは `_schedule_next_cycle` を同期関数にし、`ensure_future` でサイクル実行を非ブロッキングにする設計が `asyncio.sleep` より安定している
- `asyncio.get_running_loop()` は `get_event_loop()` より明示的で Python 3.10+ では推奨 API
- ドメイン Protocol は名前が同じでも `robot_controller.py` 側と `drive_loop.py` 側で別定義になるため、mypy が両方の満足を要求する。`robot_controller.py` の `ActuatorDriverProtocol` に `move_to_position` を追加することで解決

### 次回への改善提案
- `start_auto_drive()` に DriveLoop 統合パスのテストを追加すると回帰防止になる
- ログ書き込みテストの `RuntimeWarning` は ensure_future モック方式を改善すれば解消できる
- 実機 Modbus RTU / CAN 結合テストは HW 結合テストフェーズで実施予定
