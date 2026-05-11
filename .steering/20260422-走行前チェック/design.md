# 設計書

## アーキテクチャ概要

レイヤードアーキテクチャに従い、`PreCheckRunner` はドメインレイヤーに配置する。
Protocol でハードウェア依存を抽象化し、インフラレイヤーの実装に依存しない。

```
Web レイヤー
    └── (将来: 走行前チェック結果のエラー表示)
アプリケーションレイヤー (RobotController)
    └── PreCheckRunner.run() を呼ぶ
ドメインレイヤー (src/domain/pre_check.py)
    └── PreCheckRunner: 6項目チェック実行
        ├── ActuatorPreCheckProtocol (infra: ActuatorDriver が満たす)
        ├── CANPreCheckProtocol      (infra: CANReader が満たす)
        └── UPSPreCheckProtocol      (infra: 機種確定後に実装、現在 stub)
モデルレイヤー (src/models/pre_check.py)
    └── PreCheckItemResult, PreCheckResult
```

## コンポーネント設計

### 1. PreCheckItemResult / PreCheckResult (src/models/pre_check.py)

**責務**:
- チェック結果の値オブジェクト

```python
@dataclass
class PreCheckItemResult:
    item_name: str
    passed: bool
    error_message: str | None = None

@dataclass
class PreCheckResult:
    passed: bool
    items: list[PreCheckItemResult]
    
    @property
    def failed_items(self) -> list[PreCheckItemResult]:
        return [i for i in self.items if not i.passed]
```

### 2. PreCheckRunner (src/domain/pre_check.py)

**責務**:
- 6項目の走行前チェックを順次実行し `PreCheckResult` を返す
- ハードウェア実装に依存しない（Protocol で抽象化）

**Protocol 定義**:

```python
class ActuatorPreCheckProtocol(Protocol):
    async def read_position(self) -> int: ...
    async def is_alarm_active(self) -> bool: ...

class CANPreCheckProtocol(Protocol):
    async def read_speed(self) -> float: ...

class UPSPreCheckProtocol(Protocol):
    async def get_battery_level_pct(self) -> float: ...
```

**主要定数**:
- `HOME_POSITION_TOLERANCE_PULSE: int = 10` (原点付近の許容誤差 ±10 pulse = ±0.1mm)
- `UPS_MIN_BATTERY_PCT: float = 20.0`

**run() の実装方針**:
- 6項目を順次実行（`asyncio.gather` は使わない: エラー発生時に全結果を集めるため）
- 各チェックメソッドは例外を内部でキャッチし `PreCheckItemResult(passed=False)` を返す
- 全チェック完了後、いずれかが False なら `passed=False` の `PreCheckResult` を返す

**6チェックの実装方針**:

| # | メソッド | ロジック |
|---|---------|---------|
| 1 | `_check_communication` | `read_position()`×2 + `read_speed()` が例外なく成功するか |
| 2 | `_check_servo_state` | `is_alarm_active()`×2 が False であるか |
| 3 | `_check_calibration` | `profile` が None でなく `calibration.is_valid` が True であるか |
| 4 | `_check_profile` | `profile` が None でないか |
| 5 | `_check_ups_battery` | `ups.get_battery_level_pct() >= UPS_MIN_BATTERY_PCT` であるか |
| 6 | `_check_actuator_position` | `abs(read_position()) <= HOME_POSITION_TOLERANCE_PULSE` ×2 |

### 3. RobotController 統合 (src/app/robot_controller.py)

**追加する Protocol**:

```python
class PreCheckRunnerProtocol(Protocol):
    async def run(self) -> PreCheckResult: ...
```

**変更点**:
- `__init__` に `pre_check_runner: PreCheckRunnerProtocol | None = None` を追加
- `start_auto_drive` / `start_manual` の stub コメントを実際の呼び出しに置き換え
- チェック NG 時: `_transition(RobotState.READY)` → `raise PreCheckFailed(result)`

**PreCheckFailed の変更**:
- `PreCheckResult` を保持できるよう `result: PreCheckResult | None = None` 属性を追加

## データフロー

### 走行開始時の走行前チェック

```
start_auto_drive(mode_id)
    → _transition(PRE_CHECK)
    → pre_check_runner.run()
        → _check_communication()  → read_position() x2 + read_speed()
        → _check_servo_state()    → is_alarm_active() x2
        → _check_calibration()    → profile.calibration.is_valid
        → _check_profile()        → profile is not None
        → _check_ups_battery()    → ups.get_battery_level_pct() >= 20.0
        → _check_actuator_position() → abs(read_position()) <= 10 x2
    → result.passed == False
        → _transition(READY)
        → raise PreCheckFailed(result)
    → result.passed == True
        → _transition(RUNNING)
        → セッション作成・DriveLoop 起動
```

## エラーハンドリング戦略

### PreCheckFailed

```python
class PreCheckFailed(Exception):
    def __init__(self, result: PreCheckResult | None = None) -> None:
        self.result = result
        super().__init__(str(result))
```

### チェック内部の例外処理

各 `_check_*` メソッドは `Exception` をキャッチし、例外メッセージを `error_message` に含めた
`PreCheckItemResult(passed=False)` を返す。チェック途中の例外が走行前チェック全体を崩さない。

## テスト戦略

### ユニットテスト (tests/unit/test_pre_check.py)

- 全6チェック通過 → `PreCheckResult(passed=True)`
- 各チェック単独失敗（6ケース）→ `PreCheckResult(passed=False)` かつ該当 item が False
- 複数チェック同時失敗 → `passed=False`、`failed_items` に複数含まれる
- 通信チェックで例外発生 → `PreCheckItemResult(passed=False, error_message=...)` になる

### 統合テスト (test_robot_controller.py に追加)

- `pre_check_runner=None`（デフォルト）時はチェックをスキップして遷移する
- `pre_check_runner` が NG を返す場合 `PreCheckFailed` が raise され状態が READY に戻る
- `pre_check_runner` が OK を返す場合 RUNNING 状態に遷移する

## ディレクトリ構造

```
src/
├── models/
│   └── pre_check.py       ← NEW: PreCheckItemResult, PreCheckResult
├── domain/
│   └── pre_check.py       ← NEW: PreCheckRunner + Protocols
└── app/
    └── robot_controller.py ← MODIFY: PreCheckRunnerProtocol, PreCheckFailed 変更, 統合
tests/
└── unit/
    └── test_pre_check.py  ← NEW: PreCheckRunner ユニットテスト
```

## 実装の順序

1. `src/models/pre_check.py` - データモデル
2. `src/domain/pre_check.py` - PreCheckRunner 実装
3. `src/app/robot_controller.py` - RobotController への統合
4. `tests/unit/test_pre_check.py` - ユニットテスト
5. `tests/unit/test_robot_controller.py` に統合テストを追加
6. テスト・lint・型チェック実行

## セキュリティ考慮事項

- 特になし（ローカルネットワーク限定システム）

## パフォーマンス考慮事項

- チェックは走行開始時のみ実行（制御ループ外）
- 通信チェックで `read_position()` / `read_speed()` を呼ぶが、各1回のみ
- 合計実行時間は Modbus RTU 2軸分 ≈ 28ms + CAN 数ms = 50ms 未満を想定

## 将来の拡張性

- `UPSPreCheckProtocol` の実機実装は UPS 機種確定後に `src/infra/ups_monitor.py` で実装
- 学習運転への統合は `start_learning_drive` 実装時に追加
- Web API エラーレスポンスで `PreCheckResult` を返す拡張は Web ルーター実装時に追加
