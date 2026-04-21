# タスクリスト: データモデル実装

## フェーズ1: プロジェクト基盤

- [x] pyproject.toml を作成（依存関係・ruff・mypy・pytest設定）
- [x] src/__init__.py と src/models/__init__.py を作成

## フェーズ2: データモデル実装

- [x] src/models/profile.py を実装（PIDGains, StopConfig, VehicleProfile）
- [x] src/models/calibration.py を実装（CalibrationData）
- [x] src/models/driving_mode.py を実装（SpeedPoint, DrivingMode）
- [x] src/models/drive_log.py を実装（DriveSession, DriveLog, DriveLogData）
- [x] src/models/system_state.py を実装（RobotState Enum, SystemState）

## フェーズ3: DB初期化スクリプト

- [x] scripts/setup_db.py を実装（CREATE TABLE / INDEX DDL）

## フェーズ4: テスト

- [x] tests/__init__.py と tests/unit/__init__.py を作成
- [x] tests/unit/test_models.py を実装

## フェーズ5: 静的解析確認

- [x] ruff check で lint エラーなし
- [x] mypy で型エラーなし
- [x] pytest tests/unit/ でテスト全パス（16/16）

## 申し送り事項

**実装完了日**: 2026-04-20

**実装ファイル**:
- `pyproject.toml` (新規)
- `src/__init__.py`, `src/models/__init__.py` (新規)
- `src/models/profile.py` — VehicleProfile, PIDGains, StopConfig
- `src/models/calibration.py` — CalibrationData
- `src/models/driving_mode.py` — DrivingMode, SpeedPoint
- `src/models/drive_log.py` — DriveSession, DriveLog, DriveLogData
- `src/models/system_state.py` — RobotState (StrEnum), SystemState
- `scripts/setup_db.py` — PostgreSQL DDL（IF NOT EXISTS、冪等実行可）
- `tests/unit/test_models.py` — 16テスト全パス

**計画と実績の差分**:
- 計画通り。追加変更なし。

**次フェーズへの引き継ぎ事項**:
- `src/models/` は確定済み。上位レイヤー実装時にインポートして使用すること
- `setup_db.py` 実行には `DATABASE_URL` 環境変数または PostgreSQL ローカル接続が必要
- `asyncio_mode = "auto"` は `pytest-asyncio` インストール後に有効になる（現在は警告のみ、非同期テスト不使用のため問題なし）
- 次の実装候補: 制御アルゴリズム（PIDController, FeedforwardController）
