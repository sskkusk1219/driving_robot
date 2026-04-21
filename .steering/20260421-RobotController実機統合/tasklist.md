# タスクリスト: RobotController 実機統合

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

---

## フェーズ1: Protocol 拡張 + RealtimeSnapshot 定義

- [x] `src/app/robot_controller.py` の `ActuatorDriverProtocol` に `reset_alarm()` / `read_position()` / `read_current()` / `connect()` を追加
- [x] `CANReaderProtocol` に `connect()` を追加
- [x] `SafetyMonitorProtocol` に `stop_monitoring()` を追加
- [x] `RealtimeSnapshot` dataclass を `src/models/system_state.py` に追加
- [x] `src/app/stubs.py` のスタブクラスに追加メソッドを実装（Protocol 準拠）

## フェーズ2: RobotController メソッド実装

- [x] `RobotController.start()` を実装（実 HW connect + start_monitoring）
- [x] `RobotController.initialize()` を実装（reset_alarm + servo_on + home_return）
- [x] `RobotController.get_realtime_data()` を実装（asyncio.gather で並列取得）

## フェーズ3: WebSocket 実データ化

- [x] `src/web/ws.py` の `broadcast_loop()` を `get_realtime_data()` 使用に更新
  - [x] 例外時はフォールバック値（0.0）を使用してブロードキャストを継続

## フェーズ4: 本番用ファクトリ

- [x] `src/app/factory.py` を作成（`build_real_controller(settings)` 実装）
- [x] `src/web/app.py` に `DRIVING_ROBOT_USE_REAL_HW` 環境変数フラグを追加
  - [x] 環境変数 `1` の場合: `build_real_controller()` を使用
  - [x] それ以外: 従来の `build_stub_controller()` を使用

## フェーズ5: テスト

- [x] `tests/unit/test_robot_controller.py` を更新
  - [x] `start()` テスト（connect + start_monitoring 呼び出し確認）
  - [x] `initialize()` テスト（reset_alarm + servo_on + home_return 呼び出し確認）
  - [x] `get_realtime_data()` 正常系テスト（gather 結果の確認）
  - [x] `get_realtime_data()` エラー系テスト（CAN エラー時）
- [x] `tests/unit/test_factory.py` を作成
  - [x] `build_real_controller()` が正しい設定で各ドライバーを生成するかテスト
- [x] 既存テスト（193件）がパスすること（最終: 249件全通過）

## フェーズ6: 品質チェック

- [x] `python -m ruff check src/app/ src/web/ src/models/ tests/unit/` がパス（3件自動修正）
- [x] `python -m mypy src/app/ src/web/ src/models/` がパス（`_build_controller()` 戻り値型を `object` → `RobotController` に修正）
- [x] 実装後の振り返り記録

---

## 実装後の振り返り

**完了日**: 2026-04-21

**計画と実績の差分**:
- `_build_controller()` の戻り値型が `object` のままで mypy エラー → `RobotController` に変更（設計上正しい）
- テストヘルパー関数（`make_accel_driver` 等）に新メソッドの `AsyncMock` 追加が必要で、既存テストが全て影響を受けた（修正により249件全通過）
- `ruff --fix` で3件の `timezone.utc` → `datetime.UTC` 変換が既存テストファイルで発生（UP017）

**学んだこと**:
- Protocol 拡張時はテストヘルパーも同時に更新が必要。今回はフェーズ5のタスクチェックが先走りで `[x]` になっており、実装が後追いになった
- `build_real_controller()` のテストは `patch` で各ドライバーのコンストラクタをモックする方式が最も堅牢（実 HW 依存なし）
- `_GpioSafetyAdapter` のコールバック登録（`trigger_emergency`, `handle_ac_power_loss`）のテストを `test_factory.py` に含めることで、アプリ層の配線を検証できる

**次回への改善提案**:
- Protocol 拡張とテストヘルパー更新は同一フェーズ・同一コミットでまとめること
- `make_*_driver()` ヘルパーを各ドライバーの Protocol 定義から自動生成する仕組みを検討
