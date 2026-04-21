# 設計書

## アーキテクチャ概要

レイヤードアーキテクチャの「アプリケーションレイヤー」に配置。

```
src/app/robot_controller.py   ← 今回実装（アプリケーションレイヤー）
src/models/calibration.py     ← CalibrationResult を追加
```

## コンポーネント設計

### 1. InvalidStateTransition（例外クラス）

**責務**: 不正な状態遷移を表すドメイン例外

```python
class InvalidStateTransition(Exception):
    pass
```

### 2. VALID_TRANSITIONS（許可遷移マップ）

```python
VALID_TRANSITIONS: dict[RobotState, frozenset[RobotState]] = {
    RobotState.BOOTING:      frozenset({RobotState.STANDBY, RobotState.ERROR}),
    RobotState.STANDBY:      frozenset({RobotState.INITIALIZING}),
    RobotState.INITIALIZING: frozenset({RobotState.READY}),
    RobotState.READY:        frozenset({RobotState.CALIBRATING, RobotState.PRE_CHECK, RobotState.EMERGENCY}),
    RobotState.CALIBRATING:  frozenset({RobotState.READY}),
    RobotState.PRE_CHECK:    frozenset({RobotState.RUNNING, RobotState.MANUAL, RobotState.READY}),
    RobotState.RUNNING:      frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.MANUAL:       frozenset({RobotState.READY, RobotState.EMERGENCY}),
    RobotState.EMERGENCY:    frozenset({RobotState.READY}),
    RobotState.ERROR:        frozenset({RobotState.STANDBY}),
}
```

### 3. Protocol インターフェース

ハードウェア依存を抽象化し、テスト時にモックを注入できるようにする。

```python
class ActuatorDriverProtocol(Protocol):
    async def home_return(self) -> None: ...
    async def servo_off(self) -> None: ...
    async def servo_on(self) -> None: ...
    async def is_alarm_active(self) -> bool: ...

class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...

class SafetyMonitorProtocol(Protocol):
    async def start_monitoring(self) -> None: ...
    def register_emergency_callback(self, cb: Callable[[], Awaitable[None]]) -> None: ...
    async def trigger_emergency(self) -> None: ...
```

### 4. CalibrationResult モデル

`src/models/calibration.py` に追加:

```python
@dataclass
class CalibrationResult:
    success: bool
    data: CalibrationData | None
    error_message: str | None
```

### 5. RobotController

```python
class RobotController:
    _state: RobotState
    _active_profile_id: str | None
    _active_session_id: str | None
    _last_normal_shutdown: bool
    _accel_driver: ActuatorDriverProtocol
    _brake_driver: ActuatorDriverProtocol
    _can_reader: CANReaderProtocol
    _safety_monitor: SafetyMonitorProtocol
    _pid: PIDController

    def __init__(
        self,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        safety_monitor: SafetyMonitorProtocol,
        pid: PIDController,
        last_normal_shutdown: bool = False,
    ) -> None: ...

    def _transition(self, new_state: RobotState) -> None:
        """有効遷移のみ許可。無効なら InvalidStateTransition を送出。"""

    def get_system_state(self) -> SystemState: ...

    async def start(self) -> None:           # BOOTING → STANDBY or ERROR
    async def initialize(self) -> None:      # STANDBY → INITIALIZING → READY
    async def stop(self) -> None:            # RUNNING/MANUAL → READY
    async def emergency_stop(self) -> None:  # any → EMERGENCY
    async def reset_emergency(self) -> None: # EMERGENCY → READY
    async def clear_error(self) -> None:     # ERROR → STANDBY
    async def run_calibration(self) -> CalibrationResult:
    async def start_auto_drive(self, mode_id: str) -> DriveSession:
    async def stop_auto_drive(self) -> None:
    async def start_manual(self) -> DriveSession:
    async def stop_manual(self) -> None:
```

## データフロー

### 自動走行開始

```
start_auto_drive(mode_id)
  → _transition(PRE_CHECK)
  → 走行前チェック（6項目）
    → NG: _transition(READY), raise PreCheckFailed
    → OK: _transition(RUNNING)
  → DriveSession を返す
```

### 非常停止

```
emergency_stop()
  → _transition(EMERGENCY) を呼ぶ（RUNNING/MANUAL/READY のみ許可）
  → accel_driver.home_return(), brake_driver.home_return() (並列)
  → safety_monitor.trigger_emergency()
```

## エラーハンドリング戦略

### カスタムエラークラス

```python
class InvalidStateTransition(Exception):
    """不正な状態遷移を試みた場合に送出。"""

class PreCheckFailed(Exception):
    """走行前チェックが失敗した場合に送出。"""
```

### エラーハンドリングパターン

- 不正遷移: `InvalidStateTransition` を送出し、状態は変更しない
- 走行前チェック失敗: `PreCheckFailed` を送出し、状態を READY に戻す
- ハードウェア通信エラー: 上位（FastAPI ルーター）に例外を伝播させる

## テスト戦略

### ユニットテスト（`tests/unit/test_robot_controller.py`）

- 全ての有効な状態遷移テスト
- 無効な遷移が `InvalidStateTransition` を送出することを確認
- `get_system_state()` の返り値確認
- `emergency_stop()` が RUNNING / MANUAL / READY から呼べることを確認
- ハードウェア依存はすべてモック（`AsyncMock` / `MagicMock`）

## 依存ライブラリ

新規追加なし（既存の依存関係のみ使用）。

## ディレクトリ構造

```
src/
  app/
    __init__.py          ← 新規作成（空）
    robot_controller.py  ← 新規作成（メイン実装）
  models/
    calibration.py       ← CalibrationResult を追加
tests/
  unit/
    test_robot_controller.py  ← 新規作成
```

## 実装の順序

1. `src/models/calibration.py` に `CalibrationResult` 追加
2. `src/app/__init__.py` 作成（空ファイル）
3. `src/app/robot_controller.py` 実装
   - 例外クラス
   - VALID_TRANSITIONS
   - Protocol インターフェース
   - RobotController クラス
4. `tests/unit/test_robot_controller.py` 実装

## セキュリティ考慮事項

- ハードウェアドライバは外部から注入されるため、型チェックのみで対応

## パフォーマンス考慮事項

- 50ms 制御ループ実装はスコープ外（今回はスタブで実装）
- 状態遷移は辞書ルックアップのみ（O(1)）

## 将来の拡張性

- `ActuatorDriverProtocol` に従えば実際の pymodbus 実装を後から注入可能
- 50ms 制御ループは `_run_control_loop()` メソッドのスタブとして骨格のみ実装
