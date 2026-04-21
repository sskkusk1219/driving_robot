# 要求内容: ハードウェア抽象層実装

## 背景

インフラ層（`src/infra/`）に以下の3ファイルが未実装のまま残っている。
Web層・ドメイン層・DB層はすべて完成しており、ハードウェア抽象層の実装が残っている。

## 実装対象

### 1. `src/infra/actuator_driver.py`
- IAI P-CON-CB（アクセル・ブレーキ）の Modbus RTU ドライバ
- pymodbus 3.x async API を使用
- 38400 bps / 8N1

### 2. `src/infra/can_reader.py`
- Kvaser USB-CAN 経由でシャシダイナモ車速を受信
- python-can 4.x + cantools（DBC デコード）を使用

### 3. `src/infra/gpio_monitor.py`
- RPi.GPIO で GPIO17（非常停止）・GPIO27（AC断）を割り込み監視
- コールバック登録パターン

## 各ファイルのテスト

- `tests/unit/infra/test_actuator_driver.py`
- `tests/unit/infra/test_can_reader.py`
- `tests/unit/infra/test_gpio_monitor.py`

## 完了条件

- 全テストがパスすること
- ruff / mypy チェックがパスすること
- `src/infra/__init__.py` から新モジュールがエクスポートされること
