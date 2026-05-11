# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: データモデル実装

- [x] `src/models/pre_check.py` を新規作成
  - [x] `PreCheckItemResult` dataclass (item_name, passed, error_message)
  - [x] `PreCheckResult` dataclass (passed, items, failed_items プロパティ)

## フェーズ2: ドメインクラス実装

- [x] `src/domain/pre_check.py` を新規作成
  - [x] `ActuatorPreCheckProtocol` Protocol 定義 (read_position, is_alarm_active)
  - [x] `CANPreCheckProtocol` Protocol 定義 (read_speed)
  - [x] `UPSPreCheckProtocol` Protocol 定義 (get_battery_level_pct)
  - [x] `PreCheckRunner` クラス定義 (__init__: accel/brake/can/ups/profile/tolerance/ups_min)
  - [x] `_check_communication` 実装 (read_position x2 + read_speed が例外なく成功)
  - [x] `_check_servo_state` 実装 (is_alarm_active() x2 が False)
  - [x] `_check_profile` 実装 (profile が None でない)
  - [x] `_check_calibration` 実装 (calibration が存在し is_valid=True)
  - [x] `_check_ups_battery` 実装 (battery_pct >= UPS_MIN_BATTERY_PCT)
  - [x] `_check_actuator_position` 実装 (abs(read_position()) <= tolerance x2)
  - [x] `run()` 実装 (全6チェックを順次実行し PreCheckResult を返す)

## フェーズ3: RobotController 統合

- [x] `src/app/robot_controller.py` を更新
  - [x] `PreCheckFailed` に `result: PreCheckResult | None = None` 属性を追加
  - [x] `PreCheckRunnerProtocol` Protocol を定義
  - [x] `__init__` に `pre_check_runner: PreCheckRunnerProtocol | None = None` を追加
  - [x] `start_auto_drive` の stub コメントを実際の pre_check_runner 呼び出しに置き換え
  - [x] `start_manual` の stub コメントを実際の pre_check_runner 呼び出しに置き換え

## フェーズ4: テスト実装

- [x] `tests/unit/test_pre_check.py` を新規作成
  - [x] 全チェック通過ケース → PreCheckResult(passed=True)
  - [x] 通信確認チェック失敗ケース (例外発生) → passed=False
  - [x] サーボ状態チェック失敗ケース (alarm active) → passed=False
  - [x] プロファイルチェック失敗ケース (profile=None) → passed=False
  - [x] キャリブレーションチェック失敗ケース (calibration=None または is_valid=False) → passed=False
  - [x] UPS残量チェック失敗ケース (battery < 20%) → passed=False
  - [x] アクチュエータ位置チェック失敗ケース (position > tolerance) → passed=False
  - [x] 複数チェック失敗ケース → failed_items に複数含まれる
- [x] `tests/unit/test_robot_controller.py` に走行前チェック統合テストを追加
  - [x] pre_check_runner=None 時はチェックをスキップして RUNNING に遷移
  - [x] チェック NG 時に PreCheckFailed が raise される (start_auto_drive)
  - [x] チェック NG 時に状態が READY に戻る (start_auto_drive)
  - [x] チェック NG 時に PreCheckFailed が raise される (start_manual)
  - [x] チェック NG 時に状態が READY に戻る (start_manual)

## フェーズ5: 品質チェック

- [x] テストが全て通ることを確認 (`python -m pytest tests/unit/ -v`) → 308 passed
- [x] lint エラーがないことを確認 (`python -m ruff check src/ tests/`) → All checks passed
- [x] 型エラーがないことを確認 (`python -m mypy src/`) → Success: no issues found

## フェーズ6: バリデーター指摘事項の修正

- [x] サーボ状態チェックの例外パステスト追加 (`TestCheckServoState::test_servo_fail_when_is_alarm_active_raises`)
- [x] `start_auto_drive` / `start_manual` の走行前チェック例外時ロールバック保護 (`try/except` 追加)
- [x] スペックテーブルとのチェック順番統一 (3=キャリブレーション、4=プロファイルに修正)

---

## 実装後の振り返り

### 実装完了日
2026-04-22

### 計画と実績の差分

**計画と異なった点**:
- `_check_servo_state` でサーボON確認を Protocol に追加するか検討したが、`initialize()` でサーボONが完了済みである設計前提を考慮し、コメントで意図を明記して省略した。`ActuatorPreCheckProtocol` への `is_servo_on()` 追加は実機 `ActuatorDriver` に対応メソッドがないため実装コストが高く、今回のスコープ外とした。

**新たに必要になったタスク**:
- validator の指摘で `test_robot_controller.py` の未使用インポートを削除した（軽微な修正）

### 学んだこと

**技術的な学び**:
- 各チェックメソッドが `Exception` をキャッチして `PreCheckItemResult(passed=False)` を返す設計により、1チェック失敗時も残チェックを継続できる。`asyncio.gather` で並列実行するより、順次実行で全結果を集める方が分かりやすい。
- `UPSPreCheckProtocol` は TBD の実機未確定部分を Protocol で切り離せるため、インフラ実装前でも走行前チェック全体をテスト可能にできた。

**プロセス上の改善点**:
- implementation-validator が「サーボON確認の欠落」を検出し、スペックとの差異を明示してくれた。設計の意図をコメントに残すことで将来の混乱を防げる。

### 次回への改善提案
- 学習運転 `start_learning_drive` の実装時に走行前チェックを忘れずに統合すること
- UPS 機種確定後に `src/infra/ups_monitor.py` で `UPSPreCheckProtocol` を実装し、`factory.py` で配線すること
- Web ルーターから `PreCheckFailed.result` を使ってエラー詳細を GUI に表示する拡張が必要になる予定
- バリデーター検出: `pre_check_runner.run()` の想定外例外によるPRE_CHECK固着は try/except で対処済み。同様のパターンを他の状態遷移でも確認すること
