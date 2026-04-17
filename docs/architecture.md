# 技術仕様書 (Architecture Design Document)

## テクノロジースタック

### 言語・ランタイム

| 技術 | バージョン | 選定理由 |
|------|-----------|----------|
| Python | 3.13 | asyncioによる10ms制御ループ、豊富な科学計算・通信ライブラリ |
| PostgreSQL | 15 | 時系列ログの高速書き込み・検索、JSON型でプロファイル保存可能 |

### フレームワーク・ライブラリ

| 技術 | バージョン | 用途 | 選定理由 |
|------|-----------|------|----------|
| FastAPI | 最新安定版 | Web APIサーバー | asyncio対応、WebSocket、自動APIドキュメント生成 |
| uvicorn | 最新安定版 | ASGIサーバー | FastAPIと組み合わせで高パフォーマンス |
| pymodbus | 3.x | Modbus RTU通信 | asyncio対応、Python製Modbus実装のデファクトスタンダード |
| python-can | 4.x | CAN bus通信 | Kvaser backenに対応 |
| asyncpg | 最新安定版 | PostgreSQL非同期ドライバ | asyncio対応で高速書き込み |
| RPi.GPIO | 最新安定版 | GPIO制御 | Raspberry Pi GPIO、UPS検知・非常停止割り込み |
| smbus2 | 最新安定版 | I2C通信 | X1201 UPS残量取得（I2C 0x36） |
| numpy | 最新安定版 | 運転モデル補間 | 高速な2次元グリッド補間 |
| scipy | 最新安定版 | 運転モデル補間 | RegularGridInterpolator |
| gzip / shutil | 標準ライブラリ | ログアーカイブ圧縮 | 追加インストール不要 |
| pydantic | v2 | データバリデーション | FastAPIと統合、型安全なAPI |

### 開発ツール

| 技術 | バージョン | 用途 | 選定理由 |
|------|-----------|------|----------|
| pytest | 最新安定版 | ユニット・統合テスト | Python標準テストフレームワーク |
| pytest-asyncio | 最新安定版 | 非同期テスト | asyncioコルーチンのテスト対応 |
| ruff | 最新安定版 | Linter / Formatter | 高速、設定簡単 |
| mypy | 最新安定版 | 型チェック | 実行前のバグ検出 |

---

## アーキテクチャパターン

### レイヤードアーキテクチャ

```
┌─────────────────────────────────────────┐
│   Webレイヤー (FastAPI)                   │  ← HTTP / WebSocket
├─────────────────────────────────────────┤
│   アプリケーションレイヤー                  │  ← ユースケース・状態管理
│   RobotController / SessionManager       │
├─────────────────────────────────────────┤
│   ドメインレイヤー                          │  ← 制御ロジック・安全監視
│   FeedforwardController / PIDController  │
│   CalibrationManager / SafetyMonitor     │
├─────────────────────────────────────────┤
│   インフラレイヤー                          │  ← HW抽象・DB・ファイルI/O
│   ActuatorDriver / CANReader             │
│   LogWriter / ArchiveManager             │
└─────────────────────────────────────────┘
```

#### Webレイヤー
- **責務**: HTTPリクエスト受付、WebSocket配信、入力バリデーション
- **許可**: アプリケーションレイヤーの呼び出しのみ
- **禁止**: ドメインロジック・DBへの直接アクセス

#### アプリケーションレイヤー
- **責務**: ユースケースの調整、システム状態機械の管理
- **許可**: ドメインレイヤー・インフラレイヤーの呼び出し
- **禁止**: HTTP/WebSocketプロトコルへの依存

#### ドメインレイヤー
- **責務**: 制御アルゴリズム、安全監視、キャリブレーションロジック
- **許可**: インフラレイヤー（HW抽象クラス）の呼び出し
- **禁止**: DBへの直接アクセス、HTTP依存

#### インフラレイヤー
- **責務**: ハードウェア通信、DB書き込み、ファイルI/O
- **許可**: 外部リソース（Modbus/CAN/PostgreSQL/GPIO）への直接アクセス
- **禁止**: ビジネスロジックの実装

---

### 非同期実行アーキテクチャ

Python asyncioのイベントループで全コンポーネントを並列実行します。

```
asyncio イベントループ
│
├── 制御タスク（10ms周期）
│   ├── CAN車速受信
│   ├── アクセル位置指令送信（ttyUSB0）
│   └── ブレーキ位置指令送信（ttyUSB1）
│
├── ログタスク（100ms周期）
│   └── PostgreSQL非同期書き込み
│
├── WebSocketタスク（100ms周期）
│   └── リアルタイムデータ配信
│
├── 安全監視タスク（常時）
│   ├── GPIO割り込み（非常停止）
│   ├── GPIO割り込み（AC電源断）
│   └── 過電流監視
│
└── FastAPI（HTTPリクエスト処理）
```

**10ms制御ループの実装方針**:
- `asyncio.create_task()` + `asyncio.sleep(0.01)` ではなく
- `asyncio.get_event_loop().run_forever()` + `loop.call_later(0.01, ...)` を使用
- アクチュエータへの2軸同時送信は `asyncio.gather()` で並列実行

---

## データ永続化戦略

### ストレージ方式

| データ種別 | ストレージ | フォーマット | 保持期間 |
|-----------|----------|-------------|---------|
| 車両プロファイル | PostgreSQL | テーブル (JSON列含む) | 無期限 |
| キャリブレーションデータ | PostgreSQL | テーブル | 無期限（最新版） |
| 走行モード定義 | PostgreSQL + CSV | テーブル + 元CSVファイル | 無期限 |
| アクティブ走行ログ | PostgreSQL | drive_logs テーブル | 3ヶ月 |
| アーカイブログ | 外付けUSB SSD | CSV + gzip圧縮 | 容量80%まで |
| 運転モデル | ファイル (`.pkl`) | numpy/pickle形式 | プロファイルに紐づく |
| システム状態（シャットダウン時） | ファイル | JSON | 起動時に参照 |

### バックアップ戦略

- **アクティブログ**: PostgreSQLのWALにより耐障害性確保
- **プロファイル**: 変更時に `config/profiles/` へJSONエクスポート（バージョン管理可能）
- **アーカイブ**: USB SSD（外付け）への定期移行で内蔵SSDを保護
- **システム状態**: シャットダウン時に `data/system_state.json` へ保存

### PostgreSQLインデックス設計

```sql
-- ログ検索の高速化（セッション・時刻でのレンジ検索）
CREATE INDEX idx_drive_logs_session_timestamp
    ON drive_logs (session_id, timestamp DESC);

-- セッション一覧の高速取得
CREATE INDEX idx_drive_sessions_started_at
    ON drive_sessions (started_at DESC);

-- 3ヶ月アーカイブ対象の特定
CREATE INDEX idx_drive_sessions_ended_at
    ON drive_sessions (ended_at ASC);
```

---

## ディレクトリ・プロセス構成

```
driving_robot/
├── src/
│   ├── web/                  # Webレイヤー
│   │   ├── app.py            # FastAPIアプリ定義
│   │   ├── routers/          # APIルーター
│   │   └── static/           # フロントエンド (HTML/JS/CSS)
│   ├── app/                  # アプリケーションレイヤー
│   │   ├── robot_controller.py
│   │   └── session_manager.py
│   ├── domain/               # ドメインレイヤー
│   │   ├── control/
│   │   │   ├── feedforward.py
│   │   │   └── pid.py
│   │   ├── calibration.py
│   │   ├── learning_drive.py
│   │   └── safety_monitor.py
│   ├── infra/                # インフラレイヤー
│   │   ├── actuator_driver.py
│   │   ├── can_reader.py
│   │   ├── gpio_monitor.py
│   │   ├── log_writer.py
│   │   └── archive_manager.py
│   └── models/               # データモデル (dataclass / pydantic)
│       ├── profile.py
│       ├── calibration.py
│       ├── drive_log.py
│       └── system_state.py
├── config/
│   ├── can/                  # DBCファイル（自作）
│   ├── profiles/             # 車両プロファイルJSONバックアップ
│   └── settings.toml         # システム設定（ポート・閾値など）
├── data/
│   ├── models/               # 運転モデル (.pkl)
│   └── system_state.json     # シャットダウン時状態保存
├── tests/
│   ├── unit/
│   ├── integration/
│   └── hardware/             # 実機テスト（手動実行）
├── scripts/
│   ├── setup_db.py           # DB初期化
│   └── start.sh              # システム起動スクリプト
└── docs/
```

---

## パフォーマンス要件

### レスポンスタイム

| 操作 | 目標時間 | 測定方法 |
|------|---------|---------|
| 制御ループ1周期 | 10ms以内 | asyncio ループ計測 |
| ログ書き込み（100ms周期） | 5ms以内 | asyncpg INSERT計測 |
| WebSocket配信遅延 | 100ms以内 | クライアント受信時刻との差分 |
| GUI起動（電源ON後） | 60秒以内 | 起動スクリプト計測 |
| キャリブレーション完了 | 60秒以内 | 両軸合計 |

### リソース使用量（Raspberry Pi 5 16GB）

| リソース | 上限 | 理由 |
|---------|------|------|
| CPU（制御ループ） | 20%以内 | 他プロセスへの影響を最小化 |
| 制御ループメモリ | 512MB以内 | ログバッファ含む |
| 内蔵SSD（PostgreSQL） | 3ヶ月分：推定10GB以内 | 100ms×8h×5回/日×90日 |
| 制御ループジッタ | ±2ms以内 | 10ms周期の安定性確保 |

### 推定ログ容量計算

```
1レコードサイズ: 約100バイト（8フィールド × 8バイト + オーバーヘッド）
100ms周期 → 10レコード/秒
1走行8時間: 10 × 60 × 60 × 8 = 288,000レコード ≈ 28.8MB
1日5走行: 144MB
3ヶ月(90日): 144 × 90 = 12.96GB（PostgreSQL）

→ アーカイブは外付けUSB SSDで管理
```

---

## ハードウェア構成

### Raspberry Pi 5 接続構成

```
Raspberry Pi 5 (16GB)
│
├── USB (ttyUSB0)  →  USB-RS485変換ケーブル  →  P-CON-CB #1 (アクセル, SLAVE_ID=1)
│                                                 └→ IAI RCP6-ROD (アクセル)
│
├── USB (ttyUSB1)  →  USB-RS485変換ケーブル  →  P-CON-CB #2 (ブレーキ, SLAVE_ID=2)
│                                                 └→ IAI RCP6-ROD (ブレーキ)
│
├── USB            →  Kvaser USB-CAN         →  シャシダイナモ CAN bus
│
├── GPIO 6         →  X1201 UPS (AC断検知)
├── I2C 0x36       →  X1201 UPS (バッテリー残量)
├── GPIO (IN)      →  非常停止スイッチ #1 (シャシダイナモ室)
│                   →  非常停止スイッチ #2 (操作エリア)  ← 並列接続
│
├── Geekworm X1201 UPS (5V)  ← Raspberry Pi本体 + 内蔵SSD
│
└── 24V UPS (TBD)  →  P-CON-CB #1 / #2 電源バックアップ
                       └→ AC断時に home_return() を実行する時間を確保
```

### Modbus RTU 通信設定

| 項目 | 値 |
|------|---|
| ボーレート | 57600 bps（P-CON-CB仕様に準拠） |
| データビット | 8 |
| パリティ | なし |
| ストップビット | 1 |
| アクセル SLAVE_ID | 1（ttyUSB0） |
| ブレーキ SLAVE_ID | 2（ttyUSB1） |

---

## セキュリティアーキテクチャ

### ネットワークアクセス制御

- **ローカルLAN限定**: FastAPIをバインドするネットワークインターフェースをLAN側のみに制限
- **ポート**: HTTP 8080（または設定可能）
- **認証なし**: 同一LAN内からの接続を信頼する（シャシダイナモ室の閉じたネットワーク）

### データ保護

- **機密情報**: 環境変数または `config/settings.toml`（gitignore対象）で管理
- **PostgreSQL接続**: ローカルホスト接続のみ（ソケット認証）
- **DBCファイル**: `config/can/` に配置、gitignore対象外（バージョン管理）

### 入力検証

- **APIリクエスト**: Pydanticモデルで自動バリデーション
- **CSVアップロード**: ヘッダー・数値範囲・時刻単調増加を検証
- **プロファイル設定値**: 開度0-100%・ゲイン正数・閾値正数をバリデーション

---

## スケーラビリティ設計

### データ増加への対応

- **3ヶ月超ログ**: `ArchiveManager` が自動的にCSV+gzip圧縮してUSB SSDへ移行
- **USB SSD 80%超**: 最古のアーカイブから自動削除
- **車両プロファイル数**: 上限なし（PostgreSQLの行数制限のみ）
- **走行モード数**: 上限なし（CSVファイルサイズに依存）

### 機能拡張性

- **外部API（Post-MVP）**: FastAPIルーターを追加するだけで拡張可能
- **新しいアクチュエータ軸**: `ActuatorDriver` を継承してプラグイン的に追加可能
- **新しい車速ソース**: `CANReader` を抽象化（CAN以外にもOBD2等を将来追加可能）

---

## テスト戦略

### ユニットテスト（`tests/unit/`）

- **フレームワーク**: pytest + pytest-asyncio
- **対象**:
  - PIDController（ステップ応答・積分リセット・ゲイン計算）
  - FeedforwardController（グリッド補間精度）
  - CalibrationManager（バリデーションロジック）
  - SafetyMonitor（閾値判定・タイマー）
  - ArchiveManager（アーカイブ判定・削除ロジック）
- **カバレッジ目標**: ドメインレイヤー 80%以上
- **モック**: ハードウェアドライバはすべてモック化

### 統合テスト（`tests/integration/`）

- **対象**:
  - RobotController 状態遷移（モックHWで全遷移を検証）
  - LogWriter ↔ PostgreSQL（ローカルDBへの実書き込み）
  - FastAPI ↔ RobotController（エンドポイント疎通）
- **環境**: テスト用PostgreSQLデータベース（本番と分離）

### ハードウェア結合テスト（`tests/hardware/`、手動実行）

- アクチュエータ単体Modbus通信（位置指令・読み取り）
- CAN受信（シャシダイナモまたは模擬信号発生器）
- 非常停止GPIO割り込み動作確認
- X1201 UPS残量取得
- AC電源断シーケンス（実際にコンセントを抜いて確認）

---

## 技術的制約

### 環境要件

- **OS**: Raspberry Pi OS 64-bit (Bookworm以降)
- **ハードウェア**: Raspberry Pi 5 (4GB以上推奨、16GB使用)
- **Python**: 3.13（仮想環境 `.venv` で管理、グローバルインストール禁止）
- **PostgreSQL**: 15（systemdサービスとして起動）
- **USB接続**: ttyUSB0・ttyUSB1 が安定してデバイスに割り当てられること
  - udev rules で固定割り当てを推奨（シリアル番号でデバイスを固定）
- **CAN**: Kvaser Linux ドライバインストール済み

### パフォーマンス制約

- 10ms制御ループのジッタ: Raspberry Pi OSはリアルタイムOSではないため±2ms程度を許容
- Modbus RTU応答時間: P-CON-CB仕様の応答時間（TBD、典型的に数ms）を考慮したループ設計が必要

### ネットワーク制約

- ローカルLAN（有線LANケーブル推奨、Wi-Fi は不安定なため非推奨）
- クラウド・インターネット接続不要

---

## 依存関係管理

| ライブラリ | 用途 | バージョン管理方針 |
|-----------|------|-------------------|
| fastapi | WebAPI | `>=0.100.0` 下位互換性あり |
| uvicorn | ASGIサーバー | `>=0.20.0` |
| pymodbus | Modbus RTU | `>=3.0.0` 3.x APIに依存 |
| python-can | CAN通信 | `>=4.0.0` Kvaser backend |
| asyncpg | PostgreSQL | `>=0.28.0` |
| RPi.GPIO | GPIO | `>=0.7.0` |
| smbus2 | I2C | `>=0.4.0` |
| numpy | 数値計算 | `>=1.25.0` |
| scipy | 補間 | `>=1.11.0` |
| pydantic | バリデーション | `>=2.0.0` v2 API |
| pytest | テスト | `>=7.0.0` |
| pytest-asyncio | 非同期テスト | `>=0.21.0` |
| ruff | Lint/Format | `>=0.1.0` |

**管理方針**:
- `requirements.txt` または `pyproject.toml` で最小バージョンを指定
- 再現性のため `pip freeze > requirements.lock` を使用
- ライブラリは `.venv` 内にのみインストール（グローバル禁止）
