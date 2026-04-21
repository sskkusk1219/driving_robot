# タスクリスト: ハードウェア抽象層

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

---

## フェーズ1: ActuatorDriver

- [x] `src/infra/actuator_driver.py` を作成
  - [x] `ActuatorDriver.__init__(port, slave_id, baud_rate)` 実装
  - [x] `connect()` / `close()` 実装（pymodbus AsyncModbusSerialClient）
  - [x] `servo_on()` / `servo_off()` 実装（FC05 コイル書き込み）
  - [x] `reset_alarm()` 実装（FC05 ALRS エッジ入力）
  - [x] `home_return()` 実装（FC05 HOME + DSS1 HEND=1 ポーリング）
  - [x] `move_to_position(pos)` 実装（FC10 PCMD/VCMD/ACMD/CTLF）
  - [x] `read_position()` 実装（FC03 PNOW 32bit）
  - [x] `read_current()` 実装（FC03 CNOW 32bit）
  - [x] `is_alarm_active()` 実装（FC03 ALMC ≠ 0）

## フェーズ2: CANReader

- [x] `src/infra/can_reader.py` を作成
  - [x] `CANReader.__init__(interface, channel, dbc_path)` 実装
  - [x] `connect()` / `close()` 実装（python-can Bus）
  - [x] `read_speed()` 実装（recv + cantools デコード、executor 経由）
  - [x] DBC ファイルが存在しない場合の NotImplementedError

## フェーズ3: GPIOMonitor

- [x] `src/infra/gpio_monitor.py` を作成
  - [x] `GPIOMonitor.__init__(emergency_pin, ac_detect_pin, loop)` 実装
  - [x] `start_monitoring()` 実装（GPIO セットアップ・割り込み登録）
  - [x] `stop_monitoring()` 実装（GPIO クリーンアップ）
  - [x] `register_emergency_callback(cb)` 実装
  - [x] `register_ac_loss_callback(cb)` 実装
  - [x] FALLING エッジ割り込みで asyncio コールバック呼び出し

## フェーズ4: テスト

- [x] `tests/unit/infra/test_actuator_driver.py` を作成
  - [x] `connect()` 正常系テスト
  - [x] `servo_on()` / `servo_off()` テスト（FC05 呼び出し確認）
  - [x] `reset_alarm()` テスト
  - [x] `home_return()` テスト（HEND ポーリング）
  - [x] `move_to_position()` テスト（FC10 レジスタ値確認）
  - [x] `read_position()` テスト（32bit 結合・符号変換）
  - [x] `read_current()` テスト
  - [x] `is_alarm_active()` テスト
- [x] `tests/unit/infra/test_can_reader.py` を作成
  - [x] `read_speed()` 正常系テスト
  - [x] DBC ファイル未存在時の NotImplementedError テスト
- [x] `tests/unit/infra/test_gpio_monitor.py` を作成
  - [x] `start_monitoring()` でGPIOセットアップ確認
  - [x] FALLING エッジでコールバックが呼ばれることのテスト
  - [x] `stop_monitoring()` でGPIOクリーンアップ確認

## フェーズ5: 統合・品質チェック

- [x] `src/infra/__init__.py` を更新（新モジュールをエクスポート）
- [x] 全テストがパスすること（`python -m pytest tests/unit/infra/ -v`）36 passed
- [x] ruff チェックパス（`python -m ruff check src/infra/ tests/unit/infra/`）All checks passed
- [x] mypy チェックパス（`python -m mypy src/infra/`）Success: no issues
- [x] 実装後の振り返り記録

---

## 実装後の振り返り

### 実装完了日
2026-04-21

### 計画と実績の差分

**当初計画から変更した点**:
- `ActuatorDriver.__init__` で `AsyncModbusSerialClient` を即時生成する設計から、`connect()` での遅延初期化に変更。理由: pyserial 未インストール環境（テスト・CI）でのインスタンス化エラー回避
- pymodbus 3.x API は `slave=` ではなく `device_id=` キーワードを使用。設計ドキュメントの想定と異なったため修正
- `framer="rtu"` ではなく `framer=FramerType.RTU`（Enum）が正しい API であることを実機確認
- `can_reader.py` で `self._bus / self._db` を `object` 型から `Any` 型に変更（`ignore_missing_imports=true` 環境では `type: ignore[assignment]` が "unused" エラーになる）
- GPIO テスト: `patch.dict("sys.modules", {"RPi.GPIO": mock_gpio})` だけでは不十分で、`mock_rpi.GPIO = mock_gpio` を明示的にセットする必要があった（Python の `IMPORT_FROM` 命令は親モジュール属性アクセスを優先するため）

### 学んだこと

**技術的な学び**:
- pymodbus 3.x の `AsyncModbusSerialClient` は `pyserial` を `__init__` 内でチェックするため、テスト環境では遅延初期化パターンが必須
- Python の `import A.B as C` は内部的に `IMPORT_NAME` + `IMPORT_FROM` 命令で実行され、`IMPORT_FROM` は `sys.modules["A.B"]` ではなく `sys.modules["A"].B` (属性アクセス) を優先する
- mypy の `ignore_missing_imports = true` 設定下では、未インストールのモジュール型アノテーションに `type: ignore` をつけると逆に "unused-ignore" エラーになる

### 次回への改善提案
- `RobotController` の `start()` メソッドに `ActuatorDriver.connect()` / `CANReader.connect()` の実呼び出しを組み込む
- `GPIOMonitor` を `SafetyMonitor` から `start_monitoring()` で起動するよう統合する
- `ActuatorDriver.move_to_position()` の VCMD/ACMD 値は実機チューニングが必要（デフォルト 50mm/s は仮値）
- `CANReader` の `_SPEED_SIGNAL_NAME = "VehicleSpeed"` はシャシダイナモの DBC 仕様確定後に更新する
- pyserial を `.venv` に追加インストールすれば実機での結合テストが可能（`pip install pyserial`）
