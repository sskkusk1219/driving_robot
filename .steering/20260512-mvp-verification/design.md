# 設計: MVP機能動作確認

## 調査結果サマリー

### ユニットテスト (380件)
全件パス済み。

### Lint エラー (ruff E501: 行長超過)
| ファイル | 行 | 内容 |
|---------|---|------|
| src/infra/gpio_monitor.py | 6 | docstring の GPIO27 説明行 (104文字) |
| tests/hardware/test_emergency_stop.py | 39 | state_str 代入行 (112文字) |
| tests/hardware/test_emergency_stop_home_return.py | 29 | lgpio エラーメッセージ行 (112文字) |

### 型エラー (mypy)
| ファイル | 内容 |
|---------|------|
| src/app/stubs.py | _StubActuator が ActuatorDriverProtocol の enable_modbus_control を未実装 |

## 修正方針
1. `stubs.py`: `_StubActuator` に `enable_modbus_control` を追加
2. 各ファイルの行を100文字以内に収める（文字列の折り返しまたは変数分割）
