# 設計: データモデル実装

## 方針

- `src/models/` 配下を純粋なデータ定義層とする（I/O・DB・ビジネスロジックなし）
- エンティティは `@dataclass` で定義（シリアライズ不要な内部モデル）
- API入出力は別途 Pydantic モデルとする（今回は対象外）
- PostgreSQL スキーマは `scripts/setup_db.py` で管理

## ファイル構成

```
src/
└── models/
    ├── __init__.py
    ├── profile.py       # VehicleProfile, PIDGains, StopConfig
    ├── calibration.py   # CalibrationData
    ├── drive_log.py     # DriveSession, DriveLog, DriveLogData
    ├── driving_mode.py  # DrivingMode, SpeedPoint
    └── system_state.py  # SystemState, RobotState (Enum)

scripts/
└── setup_db.py         # CREATE TABLE / INDEX DDL

tests/
└── unit/
    └── test_models.py  # dataclass制約・フィールド型チェック

pyproject.toml          # プロジェクト設定
```

## データモデル設計詳細

### profile.py

```python
@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float

@dataclass
class StopConfig:
    deviation_threshold_kmh: float
    deviation_duration_s: float

@dataclass
class VehicleProfile:
    id: str
    name: str
    max_accel_opening: float   # 0.0-100.0
    max_brake_opening: float   # 0.0-100.0
    max_speed: float           # > 0
    max_decel_g: float         # > 0
    pid_gains: PIDGains
    stop_config: StopConfig
    calibration: CalibrationData | None
    model_path: str | None
    created_at: datetime
    updated_at: datetime
```

### system_state.py

```python
class RobotState(str, Enum):
    BOOTING = "BOOTING"
    STANDBY = "STANDBY"
    INITIALIZING = "INITIALIZING"
    READY = "READY"
    CALIBRATING = "CALIBRATING"
    PRE_CHECK = "PRE_CHECK"
    RUNNING = "RUNNING"
    MANUAL = "MANUAL"
    EMERGENCY = "EMERGENCY"
    ERROR = "ERROR"

@dataclass
class SystemState:
    robot_state: RobotState
    active_profile_id: str | None
    active_session_id: str | None
    last_normal_shutdown: bool
    updated_at: datetime
```

### drive_log.py

DriveLogData は LogWriter が 100ms 周期で書き込む際の転送オブジェクト（ id・timestamp なし）。

## DB スキーマ方針

- 全テーブルに `id UUID PRIMARY KEY`（drive_logs のみ BIGSERIAL）
- インデックスは architecture.md 定義の3本のみ作成
- IF NOT EXISTS で冪等実行可能にする
