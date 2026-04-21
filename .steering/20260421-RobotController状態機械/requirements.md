# 要求内容

## 概要

`RobotController` クラスを実装する。システム全体の状態機械を管理し、各コンポーネント（PID制御、フィードフォワード制御、安全監視、アクチュエータ、CAN）の協調制御を担う。

## 背景

機能設計書に RobotController の状態機械・API・協調制御シーケンスが定義されているが、`src/app/robot_controller.py` が未実装。
`src/models/system_state.py` には `RobotState` と `SystemState` が定義済みで、状態機械の骨格はある。

## 実装対象の機能

### 1. 状態機械

- `VALID_TRANSITIONS` 辞書で許可遷移のみを管理
- `_transition(new_state)` ヘルパーで不正遷移を `InvalidStateTransition` 例外として弾く
- 状態: BOOTING → STANDBY → INITIALIZING → READY ↔ CALIBRATING / PRE_CHECK / RUNNING / MANUAL / EMERGENCY、ERROR ↔ STANDBY

### 2. RobotController 公開 API

機能設計書に定義されたシグネチャを実装:

- `async def start() -> None` — BOOTING → STANDBY（通信確認OK）または ERROR
- `async def initialize() -> None` — STANDBY → INITIALIZING → READY
- `async def stop() -> None` — RUNNING/MANUAL → READY（原点復帰 → サーボOFF）
- `async def emergency_stop() -> None` — RUNNING/MANUAL/READY → EMERGENCY
- `async def reset_emergency() -> None` — EMERGENCY → READY
- `async def clear_error() -> None` — ERROR → STANDBY
- `async def run_calibration() -> CalibrationResult` — READY → CALIBRATING → READY
- `async def start_auto_drive(mode_id: str) -> DriveSession` — READY → PRE_CHECK → RUNNING
- `async def stop_auto_drive() -> None` — RUNNING → READY
- `async def start_manual() -> DriveSession` — READY → PRE_CHECK → MANUAL
- `async def stop_manual() -> None` — MANUAL → READY
- `def get_system_state() -> SystemState`

### 3. 依存性注入（Protocol）

ハードウェア依存（ActuatorDriver, CANReader）は未実装のため、`typing.Protocol` でインターフェースを定義し、RobotController はインターフェースに依存する。テスト時はモックを注入できる。

### 4. CalibrationResult モデル

`src/models/calibration.py` に `CalibrationResult` dataclass を追加。

### 5. ユニットテスト

状態遷移・境界条件を網羅するユニットテストを `tests/unit/test_robot_controller.py` に実装。

## 受け入れ条件

### 状態機械
- [ ] 全ての有効な遷移が成功する
- [ ] 無効な遷移は `InvalidStateTransition` を送出する
- [ ] 非常停止は RUNNING/MANUAL/READY どの状態からでも EMERGENCY へ遷移できる

### API
- [ ] `get_system_state()` が現在の SystemState を返す
- [ ] 各メソッドが正しい状態遷移を行う

### テスト
- [ ] ユニットテストが全件パスする
- [ ] ruff lint/format エラーなし
- [ ] mypy strict エラーなし

## スコープ外

以下はこのフェーズでは実装しません:

- 50ms 制御ループの実際のハードウェア通信（ActuatorDriver, CANReader の実装）
- FastAPI エンドポイント（Webレイヤー）
- LogWriter との統合
- CalibrationManager の実装

## 参照ドキュメント

- `docs/functional-design.md` — 状態機械図・コンポーネント設計
- `docs/architecture.md` — レイヤードアーキテクチャ、asyncio 設計
- `docs/development-guidelines.md` — コーディング規約
