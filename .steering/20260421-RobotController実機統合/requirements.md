# 要求内容: RobotController 実機統合

## 背景

ハードウェア抽象層（ActuatorDriver / CANReader / GPIOMonitor）の実装が完了した。
現在の `RobotController` には `# 実機実装` コメントが残っており、スタブ動作のみ。
WebSocket も `actual_speed_kmh=0.0` などのハードコード値を返している。

## 実装対象

### 1. `RobotController` の実機 HW 接続

- `start()`: ActuatorDriver × 2 / CANReader / GPIOMonitor の `connect()` を呼ぶ
- `initialize()`: `reset_alarm()` + `servo_on()` を実行してから `home_return()`

### 2. `RobotController.get_realtime_data()` メソッド追加

- `read_position()` / `read_current()` / `can_reader.read_speed()` を呼んで実値を返す
- `RealtimeData` 相当のデータ構造を返す（ドメイン依存しない dataclass）

### 3. WebSocket broadcast_loop の実データ化

- `ws.py` の broadcast_loop でスタブ値 → `controller.get_realtime_data()` を使用

### 4. 本番用コントローラーファクトリ `build_real_controller()`

- `src/app/factory.py` に `build_real_controller(settings: AppSettings) -> RobotController` を実装
- `src/web/app.py` でスタブ → 実コントローラーに切り替える環境変数フラグ

## 完了条件

- 全テストがパスすること
- ruff / mypy チェックがパスすること
- 既存テスト（193件）のリグレッションなし
