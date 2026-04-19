# リポジトリ構造定義書 (Repository Structure Document)

## プロジェクト構造

```
driving_robot/
├── src/                        # ソースコード
│   ├── web/                    # Webレイヤー (FastAPI)
│   ├── app/                    # アプリケーションレイヤー
│   ├── domain/                 # ドメインレイヤー（制御・安全ロジック）
│   ├── infra/                  # インフラレイヤー（HW・DB・ファイルI/O）
│   └── models/                 # データモデル (dataclass / pydantic)
├── tests/                      # テストコード
│   ├── unit/                   # ユニットテスト（モックHW）
│   ├── integration/            # 統合テスト（ローカルDB）
│   └── hardware/               # ハードウェア結合テスト（手動実行）
├── config/                     # 設定ファイル
│   ├── can/                    # DBCファイル（自作CAN定義）
│   ├── profiles/               # 車両プロファイルJSONバックアップ
│   └── settings.toml           # システム設定
├── data/                       # 実行時データ
│   ├── models/                 # 運転モデルファイル (.pkl)
│   └── system_state.json       # シャットダウン時状態保存
├── sample/                     # 実装サンプル・動作確認用スクリプト（MVP完了後に削除予定）
├── scripts/                    # セットアップ・起動スクリプト
├── docs/                       # プロジェクトドキュメント
├── .steering/                  # 作業単位のステアリングファイル
├── .claude/                    # Claude Code設定
├── .venv/                      # Python仮想環境（gitignore対象）
├── pyproject.toml              # プロジェクト設定・依存関係
├── requirements.lock           # 再現性のための完全バージョン固定
└── README.md                   # プロジェクト概要
```

---

## ディレクトリ詳細

### src/web/ （Webレイヤー）

**役割**: HTTP・WebSocketの受付、フロントエンド配信、入力バリデーション

**依存可能**: `src/app/`、`src/models/`
**依存禁止**: `src/domain/`、`src/infra/`（アプリレイヤー経由のみ）

**配置ファイル**:
- `app.py`: FastAPIアプリ定義・起動設定
- `routers/`: APIルーター（機能別に分割）
- `static/`: フロントエンド（HTML/JS/CSS）
- `websocket.py`: WebSocketハンドラ

**例**:
```
src/web/
├── app.py
├── routers/
│   ├── profile.py          # /api/profiles
│   ├── calibration.py      # /api/calibration
│   ├── drive.py            # /api/drive
│   ├── log.py              # /api/logs
│   └── system.py           # /api/system
├── websocket.py
└── static/
    ├── index.html
    ├── js/
    └── css/
```

---

### src/app/ （アプリケーションレイヤー）

**役割**: ユースケースの調整、システム状態機械の管理、各コンポーネントの協調

**依存可能**: `src/domain/`、`src/infra/`、`src/models/`
**依存禁止**: `src/web/`（HTTP/WebSocketへの依存禁止）

**配置ファイル**:
- `robot_controller.py`: メインコントローラ・状態機械
- `session_manager.py`: 走行セッション管理

**例**:
```
src/app/
├── robot_controller.py
└── session_manager.py
```

---

### src/domain/ （ドメインレイヤー）

**役割**: 制御アルゴリズム、安全監視、キャリブレーション・学習運転ロジック

**依存可能**: `src/infra/`（HW抽象クラス経由）、`src/models/`
**依存禁止**: `src/web/`、`src/app/`（上位レイヤーへの依存禁止）、DBへの直接アクセス

**配置ファイル**:
- `control/`: 制御アルゴリズム群
- `calibration.py`: キャリブレーション管理（`CalibrationManager`）
- `learning_drive.py`: 学習運転管理（`LearningDriveManager`）
- `safety_monitor.py`: 安全監視（`SafetyMonitor`）

**例**:
```
src/domain/
├── control/
│   ├── feedforward.py      # フィードフォワード制御 (FeedforwardController)
│   ├── pid.py              # PIDコントローラ (PIDController)
│   └── drive_loop.py       # 50ms制御ループ (DriveLoop)
├── calibration.py          # CalibrationManager
├── learning_drive.py       # LearningDriveManager
└── safety_monitor.py       # SafetyMonitor
```

---

### src/infra/ （インフラレイヤー）

**役割**: ハードウェア通信、DB書き込み、ファイルI/Oの実装

**依存可能**: `src/models/`、外部ライブラリ（pymodbus・python-can・asyncpg等）
**依存禁止**: `src/web/`、`src/app/`、`src/domain/`（ビジネスロジック禁止）

**配置ファイル**:
- `actuator_driver.py`: P-CON-CB Modbus RTU通信
- `can_reader.py`: Kvaser USB-CAN車速受信
- `gpio_monitor.py`: GPIO（非常停止・UPS）監視
- `log_writer.py`: PostgreSQL非同期ログ書き込み
- `archive_manager.py`: ログアーカイブ（CSV+gzip → USB SSD）
- `db.py`: DBコネクション管理

**例**:
```
src/infra/
├── actuator_driver.py
├── can_reader.py
├── gpio_monitor.py
├── log_writer.py
├── archive_manager.py
└── db.py
```

---

### src/models/ （データモデル）

**役割**: システム全体で共有するデータ構造の定義（dataclass・pydantic）

**依存可能**: 標準ライブラリ・pydantic のみ
**依存禁止**: 他の `src/` 配下のモジュール（循環依存防止）

**配置ファイル**:
- `profile.py`: VehicleProfile・PIDGains・StopConfig
- `calibration.py`: CalibrationData
- `drive_log.py`: DriveSession・DriveLog・DriveLogData
- `driving_mode.py`: DrivingMode・SpeedPoint
- `system_state.py`: SystemState（シャットダウン時保存）

**例**:
```
src/models/
├── profile.py
├── calibration.py
├── drive_log.py
├── driving_mode.py
└── system_state.py
```

---

### tests/ （テストディレクトリ）

#### tests/unit/

**役割**: 単一クラス・関数のテスト（ハードウェア不要、モックHW使用）

**構造**: `src/` のディレクトリ構造と対応

```
tests/unit/
├── domain/
│   ├── control/
│   │   ├── test_feedforward.py
│   │   └── test_pid.py
│   ├── test_calibration.py
│   └── test_safety_monitor.py
└── infra/
    └── test_archive_manager.py
```

**命名規則**: `test_[対象モジュール名].py`

#### tests/integration/

**役割**: 複数コンポーネント間の結合テスト（ローカルPostgreSQL使用）

```
tests/integration/
├── test_robot_controller_states.py   # 状態遷移全パターン
├── test_log_writer_db.py             # ログDB書き込み
└── test_api_endpoints.py             # FastAPI疎通
```

#### tests/hardware/

**役割**: 実機を使ったハードウェア結合テスト（手動実行・要実機環境）

```
tests/hardware/
├── test_actuator_modbus.py     # Modbus RTU通信
├── test_can_reader.py          # CAN受信
├── test_gpio_emergency.py      # 非常停止GPIO
└── test_ac_ups_detect.py       # AC UPS接点出力によるAC断検知確認
```

---

### config/ （設定ファイル）

```
config/
├── can/
│   └── MEIDEN_MEIDACS.dbc      # シャシダイナモ車速CAN定義（明電舎 MEIDACS）
├── profiles/
│   └── [profile_name].json     # 車両プロファイルJSONバックアップ
├── settings.toml               # システム設定（gitignore対象、機器固有値）
└── settings.toml.example       # 設定テンプレート（バージョン管理対象）
```

**settings.toml の構造**:
```toml
[serial]
accel_port = "/dev/ttyUSB0"
brake_port = "/dev/ttyUSB1"
baud_rate = 38400

[can]
interface = "kvaser"
channel = 0

[database]
dsn = "postgresql://localhost/driving_robot"

[gpio]
ac_detect_pin = 27        # AC UPS接点出力（物理ピン13）[要確認: AC UPS機種確定後に更新]
emergency_stop_pin = 17   # 非常停止スイッチ（物理ピン11）

[archive]
usb_ssd_path = "/mnt/usb_ssd/archive"
active_log_days = 90
storage_limit_pct = 80

[control]
loop_interval_ms = 50
log_interval_ms = 100
```

---

### data/ （実行時データ）

```
data/
├── models/
│   └── [profile_id]/
│       └── driving_model.pkl   # 学習済み運転モデル
└── system_state.json           # 前回終了時の状態
```

**system_state.json の構造**:
```json
{
  "last_shutdown": "2026-04-17T09:00:00+09:00",
  "shutdown_type": "normal",
  "active_profile_id": "uuid",
  "accel_homed": true,
  "brake_homed": true
}
```

---

### scripts/ （スクリプト）

```
scripts/
├── setup_db.py     # PostgreSQL DB初期化・テーブル作成
├── setup_env.sh    # 仮想環境セットアップ・依存関係インストール
└── start.sh        # システム起動（uvicorn + 制御ループ）
```

---

### docs/ （ドキュメント）

```
docs/
├── product-requirements.md     # プロダクト要求定義書
├── functional-design.md        # 機能設計書
├── architecture.md             # アーキテクチャ設計書
├── repository-structure.md     # 本ドキュメント
├── development-guidelines.md   # 開発ガイドライン
├── glossary.md                 # 用語集
├── ideas/                      # 壁打ち・アイデアメモ
└── manuals/                    # 機器マニュアル（PDF等）
```

---

### .steering/ （ステアリングファイル）

**役割**: 特定の開発作業における「今回何をするか」を定義する作業単位ドキュメント

```
.steering/
└── [YYYYMMDD]-[task-name]/
    ├── requirements.md     # 今回の作業の要求内容
    ├── design.md           # 実装アプローチ
    └── tasklist.md         # タスクリスト（進捗管理）
```

**命名規則**: `20260501-implement-calibration` 形式

---

## ファイル配置規則

### ソースファイル

| ファイル種別 | 配置先 | 命名規則 | 例 |
|------------|--------|---------|-----|
| FastAPIルーター | `src/web/routers/` | `snake_case.py` | `calibration.py` |
| アプリレイヤー | `src/app/` | `snake_case.py` | `robot_controller.py` |
| ドメインロジック | `src/domain/` | `snake_case.py` | `safety_monitor.py` |
| インフラ実装 | `src/infra/` | `snake_case.py` | `actuator_driver.py` |
| データモデル | `src/models/` | `snake_case.py` | `drive_log.py` |
| 設定 | `config/` | `snake_case.toml/json` | `settings.toml` |
| 運転モデル | `data/models/[profile_id]/` | `driving_model.pkl` | — |

### テストファイル

| テスト種別 | 配置先 | 命名規則 | 例 |
|-----------|--------|---------|-----|
| ユニットテスト | `tests/unit/` | `test_[対象].py` | `test_pid.py` |
| 統合テスト | `tests/integration/` | `test_[機能].py` | `test_log_writer_db.py` |
| HW結合テスト | `tests/hardware/` | `test_[HW名].py` | `test_actuator_modbus.py` |

---

## 命名規則

### ディレクトリ名

- **レイヤーディレクトリ**: 小文字・単数形 (`web/`, `app/`, `domain/`, `infra/`)
- **機能サブディレクトリ**: 小文字・snake_case (`control/`, `can/`, `profiles/`)

### ファイル名

- **Pythonソースファイル**: snake_case（`robot_controller.py`、`actuator_driver.py`）
- **テストファイル**: `test_` プレフィックス + snake_case（`test_pid.py`）
- **設定ファイル**: snake_case（`settings.toml`）
- **ドキュメント**: kebab-case（`product-requirements.md`）

### クラス名・関数名

- **クラス**: PascalCase（`RobotController`、`ActuatorDriver`）
- **関数・メソッド**: snake_case（`run_calibration()`、`read_current()`）
- **定数**: UPPER_SNAKE_CASE（`CONTROL_LOOP_INTERVAL_MS = 50`）
- **async関数**: 通常と同じsnake_case（`async def start_auto_drive()`）

---

## 依存関係のルール

### レイヤー間の依存

```
src/web/
    ↓ (OK)
src/app/
    ↓ (OK)
src/domain/ ←→ src/infra/   (双方向可: domain→infra の呼び出し)
    ↓           ↓
src/models/ (全レイヤーから参照可)
```

**禁止される依存**:
- `src/infra/` → `src/domain/` （インフラがビジネスロジックに依存禁止）
- `src/domain/` → `src/app/` （ドメインが上位レイヤーに依存禁止）
- `src/app/` → `src/web/` （アプリがWebプロトコルに依存禁止）
- `src/models/` → 他の `src/` 配下（モデルは孤立したまま）

### 循環依存の防止

各モジュールのインポートは必ず一方向を維持します。循環が発生する場合は `src/models/` に共通型を抽出して解決します。

---

## 除外設定（.gitignore）

```gitignore
# 仮想環境
.venv/

# Python
__pycache__/
*.pyc
*.pyo
*.pkl          # 運転モデルは除外（容量大、再学習可能）

# 実行時データ
data/system_state.json
data/models/

# 機密・環境設定
config/settings.toml    # テンプレートのみ管理
.env

# ログ・一時ファイル
*.log
.DS_Store

# テストカバレッジ
.coverage
htmlcov/

# ステアリングファイル（作業用一時ファイル）
# ※ 履歴として残す場合はコメントアウト
# .steering/
```

**gitignore対象外（バージョン管理する）**:
- `config/can/*.dbc`: CANシグナル定義
- `config/profiles/`: 車両プロファイルのJSONバックアップ
- `config/settings.toml.example`: 設定テンプレート
- `docs/`: すべてのドキュメント
- `scripts/`: セットアップスクリプト

---

## スケーリング戦略

### 機能追加時のファイル配置

| 追加する機能 | 配置先 |
|------------|--------|
| 新しいAPIエンドポイント | `src/web/routers/[機能名].py` |
| 新しい制御アルゴリズム | `src/domain/control/[名前].py` |
| 新しいハードウェア対応 | `src/infra/[デバイス名]_driver.py` |
| 新しいデータモデル | `src/models/[エンティティ名].py` |
| 外部API連携（Post-MVP） | `src/web/routers/api_v1.py` |

### ファイルサイズの目安

- 1ファイル: 300行以下を推奨
- 300-500行: リファクタリングを検討（サブモジュールへの分割）
- 500行以上: 分割を強く推奨
