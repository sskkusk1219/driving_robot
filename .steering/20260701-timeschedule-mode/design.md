# 設計書: タイムスケジュールモード

## アーキテクチャ概要

既存の3層（web / app / domain・infra）を踏襲する。自動運転（DriveLoop・閉ループ）・
学習運転（LearningLoop・開ループ）と並ぶ第3の実行ループとして `ScheduleLoop`（統合タイム
ライン開ループ）を追加する。基準車速を持たず、時刻でペダル開度とボタン押下を再生する。

```
Web(schedules CRUD / drive schedule コマンド)
   → RobotController.start_schedule_drive
      → ScheduleLoop（100ms周期）
          ├─ ペダル: pedal_points を線形補間 → 開度 → 位置 → ActuatorDriver（両軸）
          └─ ボタン: button_events を time_s で発火 → ButtonServoDriver.press（I2C）
```

## データモデル（`src/models/time_schedule.py`）

`docs/functional-design.md` の定義に一致させる。

```python
@dataclass
class PedalPoint:
    time_s: float
    accel_opening: float   # [%]
    brake_opening: float   # [%]

@dataclass
class ButtonEvent:
    time_s: float
    channel: int           # 0-15
    press_duration_s: float

@dataclass
class TimeSchedule:
    id: str
    name: str
    description: str
    pedal_points: list[PedalPoint]
    button_events: list[ButtonEvent]
    total_duration: float
    loop: bool
    created_at: datetime
```

## 永続化

### DDL（`scripts/setup_db.py` に追記）
```sql
CREATE TABLE IF NOT EXISTS time_schedules (
    id             UUID PRIMARY KEY,
    name           TEXT NOT NULL UNIQUE,
    description    TEXT NOT NULL DEFAULT '',
    pedal_points   JSONB NOT NULL,
    button_events  JSONB NOT NULL,
    total_duration DOUBLE PRECISION NOT NULL,
    loop           BOOLEAN NOT NULL DEFAULT FALSE,
    created_at     TIMESTAMPTZ NOT NULL
);
```
drive_sessions.run_type は現状 CHECK(auto/manual/learning)。スケジュール走行は run_type
='auto' で記録する（mode_id=None）。CHECK 制約変更は避け、既存カラムに寄せる。

### ScheduleRepository（`src/infra/schedule_repository.py`）
`ModeRepository` と同一構造。`_row_to_schedule` で JSONB を dataclass へ復元。
`create` は `asyncpg.UniqueViolationError` を `DuplicateNameError` に変換。

## ButtonServoDriver（`src/infra/button_servo_driver.py`）

Phase 0 の `_Pca9685`（smbus2 直叩き・角度/パルス変換）を移植し、16ch・待機/押下2値・
`release_all` を持つドライバへ発展させる。同期 I2C 呼び出しは `asyncio.to_thread` で包む
（イベントループを塞がない）。

```python
# チャンネル定数（docs のマッピング）
CH_ENGINE_START = 0
CH_SHIFT_P, CH_SHIFT_N, CH_SHIFT_D = 1, 2, 3
# ch4-15 = オプション1-12

class ButtonServoDriver:  # Protocol は robot_controller / schedule_loop 側で定義
    def __init__(i2c_bus, address=0x40, pwm_freq_hz=50, rest_angle, press_angle): ...
    async def connect() -> None            # PCA9685 初期化 + 全ch待機角
    async def press(channel, duration_s)   # 押下角へ → duration 保持 → 待機角へ
    async def release_all() -> None         # 全chを待機角へ（非常停止・エラー時）
```
角度→カウント変換は純関数 `_angle_to_count` として切り出しユニットテスト可能にする。
実 I2C は `smbus2.SMBus` を DI 可能にし、テストではフェイクバスを注入する。

## ScheduleLoop（`src/domain/control/schedule_loop.py`）

`LearningLoop` のスケジューリング機構（`call_later` + サイクルタスク + wedged 監視 +
`stop_and_join`）を踏襲。状態機械ではなく「経過時刻でタイムラインを引く」方式。

- 経過時刻 `t = now - start`。`pedal_points` を time_s で線形補間して (accel%, brake%) を得る。
  `enforce_pedal_exclusion` を通し同時踏みを構造的に排除する。開度→位置は calibration で換算。
- `button_events` は time_s 昇順。`t` がイベント時刻を超えたら未発火のものを
  `button_servo.press(ch, dur)` で fire-and-forget（I2C は独立バス）。press タスクは
  set で管理し例外はログ。
- `t >= total_duration`: `loop=True` なら start をリセットして継続、False なら on_complete。
- 安全: 過電流・CAN 断・アクチュエータ失敗・サイクル例外 → on_emergency。ボタンサーボの
  I2C 失敗は press タスク内でログのみ（走行は継続、非常停止しない＝ペダル制御を優先）。
- 連続ログ記録（ref_speed_kmh=None、run_type='auto'）。`current_ref_speed=None`。
- リアルタイム I/F（`current_accel_opening`/`current_brake_opening`/`current_ref_speed`/
  `last_snapshot`）を LearningLoop と同一シグネチャで公開し、RobotController の
  `_realtime_loop` に組み込む。

## RobotController 統合

- `__init__` に `button_servo: ButtonServoProtocol | None = None` を追加。
- `ButtonServoProtocol`（connect/press/release_all）を定義。
- `_schedule_loop` フィールドと `_realtime_loop` プロパティへの組み込み。
- `start_schedule_drive(schedule, ...)`: PRE_CHECK 経由（arm フローは使わず、
  自動運転の READY 直接開始と同じく走行前チェック→RUNNING）。項目8を含めて pre-check 実行。
- `stop_schedule_drive`/emergency: ループ停止 → `button_servo.release_all()` → 両軸 home_return。
- emergency_stop / shutdown / stop に schedule_loop の停止と release_all を織り込む。

## 走行前チェック 項目8

`PreCheckRunner` に `button_servo: ButtonServoPreCheckProtocol | None` を追加し、
`run(..., include_button_servo: bool=False)` で項目8（I2C 疎通 = connect 相当の確認）を
チェックリストへ追加する。スケジュール走行経路のみ True で呼ぶ。

## Web API

- `src/web/schemas.py`: `PedalPointSchema`/`ButtonEventSchema`/`ScheduleResponse`/
  `ScheduleDetailResponse`/`ScheduleCreateRequest`/`ScheduleUpdateRequest`/`StartScheduleRequest`。
- `src/web/routers/schedules.py`: GET一覧 / POST作成(JSON, 409重複) / GET詳細 / PUT / DELETE。
  time_s 単調増加・開度 0-100・channel 0-15・press_duration>0 をバリデーション。
- `src/web/routers/drive.py`: `POST /schedule/start`（body: schedule_id）・`POST /schedule/stop`。
- `src/web/deps.py`: `ScheduleRepoProtocol` + `get_schedule_repo`。
- `src/web/app.py`: ルーター登録 + `schedule_repo`（DB/in-memory）配線。
- `src/app/factory.py`: 実 `ButtonServoDriver` を controller に注入（bench/HW 無し時はスタブ）。
- `src/app/stubs.py`: `InMemoryScheduleRepository` + `_StubButtonServo` + build_stub_controller 配線。

## フロントエンド（`schedule-sequence.js`）

`ScheduleScreen` を実機能へ。既存共有コンポーネント（`window.Box/H2/Note/Btn` 等）と
`apiFetch` を使い、一覧表示・JSON 入力での作成・開始/停止/削除を提供する。フルの
タイムラインエディタは対象外（テキスト/JSON 入力で最小限）。`SequenceScreen` は据え置き。

## テスト戦略
- 単体: モデル、`_angle_to_count`/release（フェイクバス）、ScheduleLoop（stub 注入・補間/
  ボタン発火/loop/完了/過電流）、InMemoryScheduleRepository。
- 統合: schedules CRUD（作成・重複409・取得・削除）、schedule start/stop（モック Controller）。

## エラーハンドリング
- 名称重複 → `DuplicateNameError` → 409。存在しない → 404。バリデーション → 422。
- 不正状態遷移 → 409（`InvalidStateTransition`）。走行前チェック NG → 422（`PreCheckFailed`）。
