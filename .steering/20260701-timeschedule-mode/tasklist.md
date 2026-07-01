# タスクリスト: タイムスケジュールモード

## データモデル・永続化
- [x] T1: `src/models/time_schedule.py`（TimeSchedule/PedalPoint/ButtonEvent）
- [x] T2: `scripts/setup_db.py` に time_schedules DDL 追記
- [x] T3: `src/infra/schedule_repository.py`（ScheduleRepository CRUD）
- [x] T4: `src/app/stubs.py` に InMemoryScheduleRepository

## ハードウェア抽象
- [x] T5: `src/infra/button_servo_driver.py`（ButtonServoDriver + 角度変換純関数 + ch定数）
- [x] T6: `src/app/stubs.py` に _StubButtonServo

## 実行制御
- [x] T7: `src/domain/control/schedule_loop.py`（ScheduleLoop）
- [x] T8: `src/app/robot_controller.py`（ButtonServoProtocol・start/stop_schedule_drive・
        emergency/shutdown への release_all・_realtime_loop 組み込み）
- [x] T9: `src/domain/pre_check.py` 項目8（ボタンサーボ確認）

## Web API
- [x] T10: `src/web/schemas.py` にスケジュール系スキーマ
- [x] T11: `src/web/routers/schedules.py`（CRUD）
- [x] T12: `src/web/routers/drive.py` に schedule/start・schedule/stop
- [x] T13: `src/web/deps.py`（ScheduleRepoProtocol + get_schedule_repo）
- [x] T14: `src/web/app.py`（ルーター登録 + schedule_repo 配線）
- [x] T15: `src/app/factory.py`（実 ButtonServoDriver 注入 + settings.ServoSettings）

## フロントエンド
- [x] T16: `src/web/static/js/screens/schedule-sequence.js`（ScheduleScreen 実機能化）

## テスト
- [x] T17: `tests/unit/test_time_schedule.py`（モデル）
- [x] T18: `tests/unit/infra/test_button_servo_driver.py`
- [x] T19: `tests/unit/test_schedule_loop.py`
- [x] T20: `tests/unit/infra/test_schedule_repository.py`（in-memory + 行変換）
- [x] T21: `tests/integration/test_web_api.py` に schedules CRUD / start-stop 追加

## 検証
- [x] T22: pytest（762 passed / hardware 除く）・ruff check（All checks passed）通過
- [x] T23: 振り返り（下記）
- [x] T24: implementation-validator 検証（4.6/5）→ 指摘対応（下記）

---

## 振り返り（申し送り事項）

**実装完了日**: 2026-07-01

**計画と実績の差分**:
- ButtonServoDriver は当初 smbus2 の `i2c_rdwr(i2c_msg.write(...))` を使う設計だったが、
  テスト容易性のため標準の `write_i2c_block_data` に変更し、`bus_factory` 注入点を追加した
  （フェイク I2C バスでユニットテスト可能に）。角度→カウント変換は純関数 `angle_to_count` に分離。
- ScheduleLoop の周期は RobotController から `control_interval_s * 2`（=100ms）で渡し、
  drive_logs の記録間隔に一致させた。
- 走行前チェック項目8は `PreCheckRunner.run(include_button_servo=True)` のオプトインとし、
  スケジュールにボタンイベントがある場合のみ有効化する（ペダルのみのスケジュールでは不要）。

**学んだこと / 設計判断**:
- スケジュール走行は run_type='auto'・mode_id=None で drive_sessions に記録する
  （drive_sessions.run_type の CHECK 制約 auto/manual/learning を変更しないため）。
- 非常停止・停止・シャットダウンの3経路すべてに `_release_button_servo()` を織り込み、
  どの停止経路でも全ボタンサーボが待機位置へ戻ることを保証した。
- ボタンサーボの I2C 失敗は press タスク内でログのみ（ペダル制御を優先し走行継続）。
  ペダル/CAN/過電流は従来どおり非常停止。

**implementation-validator 指摘への対応（2026-07-01, スコア 4.6/5）**:
- 問題1（最優先: 安全機構のテスト欠如）→ `tests/unit/test_robot_controller.py::TestScheduleDrive`
  を追加（start_schedule_drive のループ構築、stop/emergency/shutdown/stop_schedule_drive での
  `release_all` 呼び出し保証、pre-check の include_button_servo フラグ）。
- 問題2（ScheduleLoop 安全系パス）→ `test_schedule_loop.py` に アクチュエータ失敗・CAN await 中停止・
  wedged cycle・未捕捉例外・lifecycle・ボタン発火順のテストを追加。
- 問題3（項目8 未検証）→ `test_pre_check.py::TestCheckButtonServo`（4分岐 + 8項目化）を追加。
- 問題4（interval_s 結合）→ `robot_controller._build_and_start_schedule_loop` の
  `interval_s=control_interval_s*2` を削除し `SCHEDULE_LOOP_INTERVAL_S`(0.1s) を単一ソースに。
- 提案1（docstring）→ pre_check モジュール docstring を項目8 対応に更新。
- 提案2（button_events 昇順検証）→ schemas に `_button_events_sorted` バリデータを追加。
- 結果: 762 passed（+18）/ ruff check クリーン。

**次回への改善提案（スコープ外・フォローアップ）**:
- 実機（PCA9685+SG90×16）での動作検証（別途ハードウェア結合テスト）。
- フロントの JSON 入力を、タイムライン・グラフィカルエディタ（ペダル/ボタン可視化）へ拡張。
- ScheduleRepository の実 DB 結合テスト（現状は in-memory + 行変換関数の単体テスト）。
- ボタンイベント発火の実時刻ドリフト（loop 巻き戻し時の端数）の実機評価。
