# 設計書

## アーキテクチャ概要

ドメインレイヤーに `DriveLoop` クラスを新規追加し、アプリケーションレイヤーの `RobotController` に統合する。

```
RobotController (app)
  └─ DriveLoop (domain/control)
       ├─ FeedforwardController (domain/control)
       ├─ PIDController (domain/control)
       ├─ ActuatorDriverProtocol (infra)
       ├─ CANReaderProtocol (infra)
       ├─ SafetyCheckProtocol (domain/control, Protocol)
       └─ LogWriterProtocol (infra, Protocol, optional)
```

## コンポーネント設計

### 1. DriveLoop (`src/domain/control/drive_loop.py`)

**責務**:
- 50ms 制御ループのスケジューリング（`call_later`）
- 1 サイクルごとの制御演算・アクチュエータ指令・安全チェック
- 100ms 周期でのログ書き込み依頼

**インターフェース**:

```python
class SafetyCheckProtocol(Protocol):
    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...
    def check_deviation(self, ref: float, actual: float, duration: float) -> bool: ...

class LogWriterProtocol(Protocol):
    async def write_log(self, session_id: str, data: DriveLogData) -> None: ...

class DriveLoop:
    def __init__(
        self,
        ff_controller: FeedforwardController,
        pid: PIDController,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        profile: VehicleProfile,
        mode: DrivingMode,
        safety_check: SafetyCheckProtocol,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        log_writer: LogWriterProtocol | None = None,
        session_id: str | None = None,
    ) -> None: ...

    def start(self) -> None: ...
    def stop(self) -> None: ...
```

**実装の要点**:
- `_running` フラグで停止制御。`stop()` 後は `_schedule_next_cycle` が何もしない
- `_started_at`: `loop.time()` で記録（monotonic clock、UTC 依存なし）
- `_cycle_count`: ログ間引きカウンタ（LOG_EVERY_N_CYCLES = 2）
- `_deviation_start`: 逸脱開始時刻（`loop.time()`）。逸脱解消時は `None` にリセット
- `_drive_accel_axis` / `_drive_brake_axis` でドライバごとに「指令→電流読み取り」を逐次実行し
  `asyncio.gather` で両軸を並列化（RS-485 バスが独立した 2 ポート構成のため並列安全）
- ログ書き込みは `asyncio.ensure_future` で fire-and-forget（50ms ループをブロックしない）
- 安全チェック失敗・CAN エラー時は `self.stop()` → `await self._on_emergency()` の順
- ループの重複実行を防ぐため `_execute_one_cycle` は `_running` フラグを先頭で確認

**定数**:
```python
CONTROL_LOOP_INTERVAL_S: float = 0.05   # 50ms
LOG_EVERY_N_CYCLES: int = 2             # 100ms 周期
```

### 2. RobotController 拡張 (`src/app/robot_controller.py`)

**追加フィールド**:
- `_ff_controller: FeedforwardController | None` — コンストラクタのオプション引数として追加
- `_drive_loop: DriveLoop | None = None` — 走行中のみ保持

**変更メソッド**:

`__init__` — オプション引数追加（既存テストと後方互換）:
```python
ff_controller: FeedforwardController | None = None,
safety_check: SafetyCheckProtocol | None = None,
```

`start_auto_drive` — オプション引数追加（後方互換）:
```python
async def start_auto_drive(
    self,
    mode_id: str,
    mode: DrivingMode | None = None,
    profile: VehicleProfile | None = None,
    log_writer: LogWriterProtocol | None = None,
) -> DriveSession:
```

DriveLoop 起動条件: `mode`, `profile`, `profile.calibration`, `self._ff_controller`, `self._safety_check` がすべて `not None` の場合

`stop_auto_drive` / `stop` / `emergency_stop` — `_drive_loop.stop()` を呼ぶ

## データフロー

### 50ms 制御サイクル

```
call_later(50ms) → _schedule_next_cycle() → ensure_future(_execute_one_cycle())
                                                      │
                        elapsed = loop.time() - started_at
                        ref_speed, ref_accel = _get_ref_speed_and_accel(elapsed)
                                                      │
                              actual_speed ← CAN read
                                                      │
                        ff_accel, ff_brake = ff.predict(ref_speed, ref_accel)
                        pid_correction = pid.update(ref_speed, actual_speed)
                                                      │
                        raw_accel = ff_accel + max(0, pid_correction)
                        raw_brake = ff_brake + max(0, -pid_correction)
                        if raw_accel>0 and raw_brake>0: raw_brake=0  (排他)
                        accel_opening = clamp(raw_accel, 0, max_accel)
                        brake_opening = clamp(raw_brake, 0, max_brake)
                                                      │
                        accel_pos = opening_to_pos(accel_opening, calib)
                        brake_pos = opening_to_pos(brake_opening, calib)
                                                      │
                        asyncio.gather(
                          _drive_accel_axis(accel_pos),  (move + read_current 逐次)
                          _drive_brake_axis(brake_pos),  (move + read_current 逐次)
                        )
                                                      │
                        safety_check (overcurrent, deviation)
                                                      │
                        cycle_count % 2 == 0 → ensure_future(log_writer.write_log())
```

## エラーハンドリング戦略

- CAN 読み取り失敗: `try/except Exception` → `stop()` → `on_emergency()`
- アクチュエータ通信失敗: `try/except Exception` → `stop()` → `on_emergency()`
- ログ書き込み失敗: ensure_future の done_callback で logging.exception（走行継続）

## テスト戦略

### ユニットテスト (`tests/unit/test_drive_loop.py`)

- `_get_ref_speed_and_accel`: 補間精度・境界値（0秒・終端・範囲外）
- `_opening_to_position`: 0%, 50%, 100% の変換精度
- FF+PID 合成・排他制御・クランプのロジック
- 正常完了: `on_complete` が呼ばれること
- 過電流: `on_emergency` が呼ばれること
- 逸脱超過: `on_emergency` が呼ばれること（持続時間カウント確認）
- CAN エラー: `on_emergency` が呼ばれること
- ログ書き込み: 2 サイクルに 1 回呼ばれること

## 依存ライブラリ

新規追加なし（既存 asyncio, typing, dataclasses を使用）

## ディレクトリ構造

```
src/domain/control/
  ├── __init__.py        (変更なし)
  ├── pid.py             (変更なし)
  ├── feedforward.py     (変更なし)
  └── drive_loop.py      (新規追加)

src/app/
  └── robot_controller.py  (DriveLoop 統合のため拡張)

tests/unit/
  └── test_drive_loop.py   (新規追加)
```

## 実装の順序

1. `src/domain/control/drive_loop.py` — DriveLoop・Protocol 定義
2. `src/app/robot_controller.py` — ff_controller / safety_check / drive_loop 統合
3. `tests/unit/test_drive_loop.py` — ユニットテスト

## パフォーマンス考慮事項

- `_execute_one_cycle` 内でブロッキング処理禁止（すべて await）
- ログ書き込みは ensure_future で fire-and-forget（ループ本体をブロックしない）
- 位置指令 + 電流読み取りのドライバごと逐次・両軸間並列: 約 25ms（50ms 以内に収まる）
