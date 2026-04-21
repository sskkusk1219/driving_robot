# 要求仕様: データモデル実装

## 対象機能

`src/models/` ディレクトリ配下のデータモデル全クラスと、DB初期化スクリプト、プロジェクト基盤ファイルの実装。

## 背景

ハードウェア不要で実装・テスト可能な最初のフェーズ。
全ての上位コンポーネント（RobotController, LogWriter など）がこのデータモデルに依存するため、最初に確定させる。

## 実装対象

### データモデル (`src/models/`)

| ファイル | クラス | 概要 |
|---------|-------|------|
| `profile.py` | VehicleProfile, PIDGains, StopConfig | 車両プロファイル |
| `calibration.py` | CalibrationData | キャリブレーションデータ |
| `drive_log.py` | DriveSession, DriveLog, DriveLogData | 走行セッション・ログ |
| `driving_mode.py` | DrivingMode, SpeedPoint | 走行モード（速度時系列） |
| `system_state.py` | SystemState | システム状態（シャットダウン時保存用） |

### プロジェクト基盤

| ファイル | 概要 |
|---------|------|
| `pyproject.toml` | 依存関係・ツール設定（ruff, mypy, pytest） |
| `scripts/setup_db.py` | PostgreSQL テーブル作成スクリプト |
| `tests/unit/test_models.py` | データモデルのユニットテスト |

## 制約・参照ドキュメント

- `docs/functional-design.md` のデータモデル定義セクションに完全準拠
- `docs/development-guidelines.md` のコーディング規約に従う
- Python 3.13 + dataclass + pydantic v2 を使用
- 型注釈必須
