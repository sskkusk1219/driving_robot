# 設計書

## アーキテクチャ概要

既存の過電流検知フロー（DriveLoop → SafetyMonitor → RobotController → ActuatorDriver）は実装済みだが、factory.py での配線が欠落している。本作業はその配線修正と設定ファイル追加、ハードウェアテストスクリプト作成が主な変更。

```
【修正後の production フロー】

DriveLoop._execute_one_cycle()
    └── self._safety_check.check_overcurrent(current_ma, axis)
            └── SafetyMonitor.check_overcurrent()  ← factory で正しく渡される
                    └── current_ma > overcurrent_limit_ma  ← settings.toml から読み込み
                            True なら on_emergency() → RobotController.emergency_stop()
                                    └── asyncio.gather(
                                            accel_driver.home_return(),
                                            brake_driver.home_return(),
                                        )

【ハードウェアテストスクリプト（スタンドアロン）】

test_overcurrent_home_return.py
    ├── ActuatorDriver × 2 (ttyUSB0/1)
    │       reset_alarm → servo_on → home_return（起動時）
    └── 制御ループ模擬（非同期）
            ├── move_to_position(目標位置)
            ├── read_current() 両軸
            └── current > LOW_THRESHOLD → home_return() 両軸並列
```

## コンポーネント設計

### 1. SafetySettings（新規: src/infra/settings.py）

**責務**:
- `config/settings.toml` の `[safety]` セクションを保持する

**実装の要点**:
```python
@dataclass
class SafetySettings:
    overcurrent_limit_ma: float = 3000.0

@dataclass
class AppSettings:
    ...
    safety: SafetySettings = field(default_factory=SafetySettings)
```

- `load_settings()` で `raw.get("safety", {})` を追加

### 2. factory.py の修正

**責務**:
- `SafetyMonitor` に `overcurrent_limit_ma` を settings から渡す
- `RobotController` に `safety_check=safety_monitor` を渡す

**実装の要点**:
```python
# 修正前
safety_monitor = SafetyMonitor(stop_config=stop_config)

# 修正後
safety_monitor = SafetyMonitor(
    stop_config=stop_config,
    overcurrent_limit_ma=settings.safety.overcurrent_limit_ma,
)

# RobotController の引数に追加
return RobotController(
    ...
    safety_monitor=safety_adapter,
    safety_check=safety_monitor,   # ← 追加
    ...
)
```

**注意**: `safety_adapter` (`_GpioSafetyAdapter`) は `SafetyMonitorProtocol` を満たすが `SafetyCheckProtocol` は満たさない。`safety_monitor`（ラップ前の `SafetyMonitor`）が両方を満たす。

### 3. settings.toml.example の追加

```toml
[safety]
# 走行中の過電流検知閾値 [mA]（P-CON-CB 仕様から設定）
overcurrent_limit_ma = 3000
```

### 4. ハードウェアテストスクリプト

**責務**:
- functional-design.md のエラーハンドリング「過電流検知: 制御ループ停止 → 緊急停止」を実機で再現する
- アプリケーション全体（FastAPI / RobotController）を起動せず単体で動作する

**実装の要点**:

```python
# 低い閾値（通常走行電流 200〜500mA 程度を想定）
# テスト時は試験者が閾値を調整して過電流を意図的に発生させる
TEST_OVERCURRENT_LIMIT_MA = 200.0  # 実機の通常電流を確認して調整

# 起動時フロー
async def main():
    # 1. Modbus 接続・初期化
    # 2. reset_alarm → servo_on → home_return（両軸）
    # 3. 制御ループ模擬開始（Ctrl+C まで継続）
    #    - move_to_position(目標位置)
    #    - read_current() 両軸
    #    - 閾値超過 → "過電流検知: [軸名] [値]mA → 原点復帰開始" を表示
    #    - home_return() 両軸並列
    #    - 原点復帰後はスクリプト終了 or ループ継続（選択可）
```

**テスト手順**:
1. 閾値 `TEST_OVERCURRENT_LIMIT_MA` を実機の通常電流より少し低く設定する
   （例: `read_current()` で 300mA を確認 → 閾値を 200mA に設定）
2. スクリプトを起動し、電流が閾値を超えたときに原点復帰が発生することを目視確認する

## データフロー

### スクリプト起動時
```
1. Modbus 接続（両軸）
2. reset_alarm → servo_on（両軸並列）
3. home_return（両軸並列）← 起動時の安全確認
4. 「起動完了: 制御ループ開始」を表示
5. 制御ループ開始
```

### 制御ループ（1サイクル）
```
1. move_to_position(100) → アクセル軸
   move_to_position(100) → ブレーキ軸 （asyncio.gather 並列）
2. read_current() → アクセル電流
   read_current() → ブレーキ電流 （asyncio.gather 並列）
3. check_overcurrent(accel_current) → True なら:
   「過電流検知: accel XXXX mA → 原点復帰開始」を表示
   home_return() 両軸並列 → 「原点復帰完了」を表示
   ループ終了
4. check_overcurrent(brake_current) → True なら:
   「過電流検知: brake XXXX mA → 原点復帰開始」を表示
   home_return() 両軸並列 → 「原点復帰完了」を表示
   ループ終了
5. 0.5秒待機（50ms制御ループよりゆっくりで十分）
```

### Ctrl+C 終了時
```
1. KeyboardInterrupt キャッチ
2. home_return()（安全位置に戻す）
3. Modbus 切断（両軸）
4. 「クリーンアップ完了」を表示
```

## エラーハンドリング戦略

| エラー | 対処 |
|---|---|
| `home_return` タイムアウト（30秒） | エラーログを表示してループ継続 |
| Modbus 接続失敗 | エラー表示して即終了 |
| `read_current` 失敗 | エラーログを表示してループ継続 |

## テスト戦略

### ユニットテスト

**test_factory.py に追加するテスト**:
```python
async def test_robot_controller_has_safety_check():
    """build_real_controller が safety_check を RobotController に渡していること"""
    ...
    ctrl = await build_real_controller(settings)
    assert ctrl._safety_check is not None

async def test_safety_monitor_uses_overcurrent_from_settings():
    """SafetyMonitor が settings の overcurrent_limit_ma を使うこと"""
    ...
```

**test_settings.py（または test_factory.py 内）に追加するテスト**:
- `SafetySettings` のデフォルト値が 3000.0 であること
- `load_settings()` が `[safety]` セクションを読み込むこと

## 依存ライブラリ

追加不要。既存の依存関係のみ使用:
- `pymodbus` (`.venv` パッケージ)
- `src.infra.actuator_driver.ActuatorDriver` (プロジェクト内モジュール)

**実行コマンド**: `.venv/bin/python tests/hardware/test_overcurrent_home_return.py`

## ディレクトリ構造

```
変更ファイル:
├── config/settings.toml.example      # [safety] セクション追加
├── src/infra/settings.py             # SafetySettings dataclass 追加
├── src/app/factory.py                # safety_check 引数追加・overcurrent_limit_ma 設定から読み込み
└── tests/unit/test_factory.py        # safety_check テスト追加

新規ファイル:
└── tests/hardware/test_overcurrent_home_return.py  # 過電流→原点復帰確認スクリプト
```

## 実装の順序

1. `settings.toml.example` に `[safety]` セクション追加
2. `settings.py` に `SafetySettings` dataclass 追加・`load_settings()` 修正
3. `factory.py` を修正（`overcurrent_limit_ma` + `safety_check`）
4. `tests/unit/test_factory.py` にテスト追加
5. `tests/hardware/test_overcurrent_home_return.py` 作成
6. ユニットテスト全件実行・確認
7. ハードウェア実機確認（手動）
8. PRD チェックボックス更新
