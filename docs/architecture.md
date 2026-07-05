# 技術仕様書 (Architecture Design Document)

## テクノロジースタック

### 言語・ランタイム

| 技術 | バージョン | 選定理由 |
|------|-----------|----------|
| Python | 3.13 | asyncioによる50ms制御ループ、豊富な科学計算・通信ライブラリ |
| PostgreSQL | 15 | 時系列ログの高速書き込み・検索、JSON型でプロファイル保存可能 |

### フレームワーク・ライブラリ

| 技術 | バージョン | 用途 | 選定理由 |
|------|-----------|------|----------|
| FastAPI | 最新安定版 | Web APIサーバー | asyncio対応、WebSocket、自動APIドキュメント生成 |
| uvicorn | 最新安定版 | ASGIサーバー | FastAPIと組み合わせで高パフォーマンス |
| pymodbus | 3.x | Modbus RTU通信 | asyncio対応、Python製Modbus実装のデファクトスタンダード |
| python-can | 4.x | CAN bus通信 | Kvaser backendに対応 |
| asyncpg | 最新安定版 | PostgreSQL非同期ドライバ | asyncio対応で高速書き込み |
| lgpio | 最新安定版 | GPIO制御 | AC UPS接点出力によるAC断検知・非常停止割り込み。RPi.GPIO の後継（Raspberry Pi OS Bookworm以降の推奨ライブラリ） |
| adafruit-circuitpython-pca9685<br/>（または smbus2） | 最新安定版 | PCA9685 I2C PWM制御 | ボタンサーボ（SG90 ×16）駆動。Post-MVP のタイムスケジュール機能で使用 |
| numpy | 最新安定版 | 運転モデル補間 | 高速な2次元グリッド補間 |
| scipy | 最新安定版 | 運転モデル補間 | RegularGridInterpolator |
| gzip / shutil | 標準ライブラリ | ログアーカイブ圧縮 | 追加インストール不要 |
| pydantic | v2 | データバリデーション | FastAPIと統合、型安全なAPI |
| React | 18.x（CDN） | フロントエンドUI | ビルド工程不要。`@babel/standalone` でブラウザ内 JSX トランスパイル。リアルタイムグラフは SVG で描画（中央固定プレイヘッド方式） |

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
├── 制御タスク（50ms周期）
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

**50ms制御ループの実装方針**:
- `asyncio.create_task()` + `asyncio.sleep(0.05)` ではなく
- `asyncio.get_event_loop().run_forever()` + `loop.call_later(0.05, ...)` を使用
- アクチュエータへの2軸同時送信は `asyncio.gather()` で並列実行

**2系統の制御ループ**:
- **DriveLoop（閉ループ・50ms）**: 自動走行・手動操作・基準速度追従。FF（先読み 多項式Ridge 逆モデル）と
  PID 補正を符号付き努力量として合成し PedalArbiter でペダルへ写像する。
- **LearningLoop（開ループ・100ms）**: 学習運転。固定の開度パターンを開ループで指令し、走行全体を
  連続した (実車速, 開度) 軌跡として `drive_logs` に記録する。コントローラ（FF/PID）に依存しないため
  運転モデル未学習の初期状態でも開度軸を端から端まで励起でき、逆モデルのブートストラップに適する。
  call_later スケジューリング・サイクルスキップ ウォッチドッグ・ログ滞留制御は DriveLoop と共通方針。
  安全は2段階（過速度・過G はパターンスキップ／過電流・通信断は非常停止）。

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

-- 学習サイクル配下セッションの取得（学習サイクル・オーケストレータ、20260703-learning-process-revamp）
CREATE INDEX idx_drive_sessions_cycle_id
    ON drive_sessions (cycle_id);
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
│   │   ├── training_service.py    # 運転モデル学習+PID自動適合（ルーターから抽出）
│   │   ├── learning_cycle.py      # 学習サイクル・オーケストレータ（2段階学習フロー）
│   │   └── session_manager.py
│   ├── domain/               # ドメインレイヤー
│   │   ├── control/
│   │   │   ├── feedforward.py
│   │   │   ├── pid.py
│   │   │   ├── drive_loop.py      # 閉ループ FF+PID（自動/手動）
│   │   │   └── learning_loop.py   # 開ループ実行ループ（学習運転）
│   │   ├── calibration.py
│   │   ├── learning_drive.py      # 学習運転の開度パターン生成
│   │   ├── model_training.py      # 運転モデル学習・物理定数推定
│   │   ├── pid_tuning.py          # PID自動適合（FOPDT同定・SIMC・規定パターン・座標降下）
│   │   └── safety_monitor.py
│   ├── infra/                # インフラレイヤー
│   │   ├── actuator_driver.py
│   │   ├── can_reader.py
│   │   ├── gpio_monitor.py
│   │   ├── log_writer.py
│   │   ├── archive_manager.py
│   │   └── db.py
│   └── models/               # データモデル (dataclass / pydantic)
│       ├── profile.py
│       ├── calibration.py
│       ├── drive_log.py
│       ├── driving_mode.py
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
│   ├── evaluate_feature_sets.py  # FeatureSpec候補セットのオフライン評価（読み取り専用）
│   ├── setup_env.sh          # 仮想環境・依存関係セットアップ
│   └── start.sh              # システム起動スクリプト
└── docs/
```

---

## パフォーマンス要件

### レスポンスタイム

| 操作 | 目標時間 | 測定方法 |
|------|---------|---------|
| 制御ループ1周期 | 50ms以内 | asyncio ループ計測 |
| ログ書き込み（100ms周期） | 5ms以内 | asyncpg INSERT計測 |
| WebSocket配信遅延 | 100ms以内 | クライアント受信時刻との差分 |
| GUI起動（電源ON後） | 60秒以内 | 起動スクリプト計測 |
| キャリブレーション完了 | 60秒以内 | 両軸合計 |

### リソース使用量（Raspberry Pi 5 16GB）

| リソース | 上限 | 理由 |
|---------|------|------|
| CPU（制御ループ） | 20%以内 | 他プロセスへの影響を最小化 |
| 制御ループメモリ | 512MB以内 | ログバッファ含む |
| 内蔵SSD（PostgreSQL） | 3ヶ月分：推定20GB以内（マージン込み） | 最悪ケース: 100ms×10h×5回/日×90日=16.2GB |
| 制御ループジッタ | ±5ms以内 | 50ms周期の安定性確保 |

### 推定ログ容量計算

```
1レコードサイズ: 約100バイト（8フィールド × 8バイト + オーバーヘッド）
100ms周期 → 10レコード/秒

【典型ケース: 8h × 3走行/日】
1走行8時間: 10 × 3600 × 8 = 288,000レコード ≈ 28.8MB
1日3走行: 86.4MB
3ヶ月(90日): 86.4 × 90 ≈ 7.8GB

【最悪ケース: 10h × 5走行/日（PRD「連続稼働10時間以上」準拠）】
1走行10時間: 10 × 3600 × 10 = 360,000レコード ≈ 36MB
1日5走行: 180MB
3ヶ月(90日): 180 × 90 = 16.2GB（PostgreSQL）

→ 内蔵SSD容量要件: 最悪ケース16.2GBに対し20GBマージンを確保
→ 3ヶ月を超えたログはアーカイブして外付けUSB SSDで管理
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
├── USB (ttyUSB1)  →  USB-RS485変換ケーブル  →  P-CON-CB #2 (ブレーキ, SLAVE_ID=1)
│                                                 └→ IAI RCP6-ROD (ブレーキ)
│
├── USB            →  Kvaser USB-CAN         →  シャシダイナモ CAN bus
│
├── I2C1 (GPIO2 SDA / GPIO3 SCL)  →  PCA9685 (I2C 0x40, 16ch PWM)  →  SG90 ボタンサーボ ×16
│                      物理ピン3/5。PCA9685 の V+ は外部5V電源から給電（本体ロジック電源と分離）
│                      ch0:エンジンスタート, ch1-3:シフトP/N/D, ch4-15:オプション1-12
│
├── GPIO22 (OUT)   →  PCA9685 OE端子  ← 物理ピン15、負論理（LOW=出力有効／HIGH=出力無効）
│                      非常停止時にソフトウェア経由（release_all）と並行してハード的に
│                      全16chのPWM出力を即遮断する冗長系。通常時はLOWを維持
│
├── GPIO17 (IN)    →  非常停止スイッチ #1 (シャシダイナモ室)  ← 物理ピン11、NC接点、プルアップ、HIGH=停止（RISING検知）
│                   →  非常停止スイッチ #2 (操作エリア)  ← 並列接続
│                      LOW=通常（NC接点閉→GND）、HIGH=停止（NC接点開→プルアップ有効、断線時も停止）
├── GPIO27 (IN)    →  未使用（APC Smart-UPS 750 は接点出力なし。NUT 経由で AC断検知）
│
└── APC Smart-UPS 750 (SUA750JB)
                    →  5V PSU  →  Raspberry Pi 本体 + 内蔵SSD
                    →  24V PSU →  P-CON-CB #1 / #2
                    └→ シリアル(DB-9)→USB変換 → /dev/ttyUSBx → NUT (upsd) → NutUPSMonitor
```

> **ボタンサーボの電源分離**: SG90 ×16 の突入・保持電流は Raspberry Pi 本体ロジック電源とは分離した外部5V電源から PCA9685 の V+ 端子に供給する。信号線（I2C）と電源のGNDは共通に接続する。

> **ボタンサーボのハードウェア非常停止（OE）**: 非常停止はGPIO17割り込み→ソフトウェア経由（`ButtonServoDriver.release_all()`）が主系統だが、ソフトウェアが正常動作していることが前提となる。PCA9685 の `OE` 端子を GPIO22 に接続し、非常停止トリガ時にソフト処理と並行して GPIO22 を HIGH にすることで、I2C通信やソフト処理を待たずにハードウェアレベルで全16chのPWM出力を即座に遮断できる（ソフトフリーズ時の最終防衛線）。PCA9685 の `VCC`（ロジック電源）は Pi の3.3Vを使用するためOEのしきい値も3.3V系と一致し、レベルシフタなしでGPIO22に直結できる。基板のOE内蔵プルダウン仕様によっては起動直後の既定状態（有効/無効）が変わるため、使用する基板のデータシートで確認すること。

### Modbus RTU 通信設定

| 項目 | 値 |
|------|---|
| ボーレート | 38400 bps（P-CON-CB実機確認値） |
| データビット | 8 |
| パリティ | なし |
| ストップビット | 1 |
| アクセル SLAVE_ID | 1（`/dev/actuator_accel`） |
| ブレーキ SLAVE_ID | 1（`/dev/actuator_brake`）※各軸が独立した RS-485 バスのため両軸とも 1 |

> **ポートの固定**: `/dev/actuator_accel` / `/dev/actuator_brake` は `scripts/setup_udev.sh`
> が FTDI シリアル番号で固定した安定シンボリックリンク（`/dev/ttyUSBn` は USB 再列挙の
> たびに番号が変わりうるため直接参照しない）。**NUT（UPS監視）など他プロセスの設定
> （`/etc/nut/ups.conf` 等）も `/dev/ttyUSBn` ではなく `/dev/serial/by-id/` の安定パスを
> 使うこと**。2026-07-03、NUT が `/dev/ttyUSB2` を固定指定していたため USB 再列挙後に
> ブレーキ用アダプタと衝突し、走行前チェック（UPS残量）が失敗する障害が発生した
> （`.steering/20260620-modbus-retry-cycle-stall`）。

> **制御ループ中のブレーキ軸タイミング**: ブレーキ軸は `move_to_position`（書込）直後に
> `read_current`（読取）を送ると、初回応答が高頻度で欠落し pymodbus の再送
> （`retries=3`、実測 ~0.3s/回）が毎サイクル発生してストールする問題があった
> （accel 軸では再現しない、brake 軸固有の RS-485/スレーブ側タイミング要因と推定）。
> `timeout` を伸ばしても実測 elapsed が timeout 値に比例してスライドするだけで
> 無効だったため、`DriveLoop._drive_brake_axis` で書込→読取間に
> `BRAKE_PRE_READ_DELAY_S`（10ms）の待機を挿入して解消した。詳細・実測値は
> `.steering/20260620-modbus-retry-cycle-stall/tasklist.md` フェーズ3参照。

### I2C / ボタンサーボ設定（Post-MVP）

| 項目 | 値 |
|------|---|
| バス | I2C1（GPIO2 SDA / GPIO3 SCL） |
| PCA9685 I2Cアドレス | 0x40（既定） |
| PWM周波数 | 50Hz（SG90標準） |
| チャンネル数 | 16（ch0-15） |
| サーボ角度 | 待機／押下の2ポジション。全チャンネル共通のグローバル設定（`config/settings.toml` `[servo]`） |
| 押下時間 | ボタンごとにタイムスケジュールで可変（例: エンジンスタート1.0s、シフト0.5s） |
| 電源 | 外部5V（PCA9685 V+）。本体ロジック電源と分離、GNDは共通 |
| OE（出力イネーブル） | GPIO22（物理ピン15）に接続。負論理（LOW=有効／HIGH=無効）。非常停止時のハードウェア冗長遮断に使用 |

### CAN 通信・配線設定

#### ノード構成

| 機器 | CAN ドライバ | インターフェース | ビットレート |
|------|------------|----------------|------------|
| RPi3B (driving_simulator) | SocketCAN (MCP2515 HAT) | `can0` | 500kbps |
| RPi5 (driving_robot) | Kvaser canlib (usbcanII) | `/dev/usbcanII0` | 500kbps |

#### Kvaser USB-CAN の注意点

- Kvaser Memorator は **SocketCAN ではなく Kvaser canlib 経由**でアクセスする
- そのため `ip link show` の出力に `can0` などのインターフェースは**表示されない**（正常動作）
- デバイス認識確認は `listChannels` コマンドを使用し、`/dev/usbcanII0`, `/dev/usbcanII1` が表示されることを確認する
- 実行には `sudo` が必要（udev rules 未設定の場合）

#### 異電源間の CAN 配線注意点（RPi3B ↔ RPi5）

- **GND 接続必須**: 別電源のノード間は CANH/CANL だけでなく GND も接続しなければ通信できない
  - Kvaser DB9 ピン3(GND) ↔ MCP2515 HAT GND を配線すること
- **終端抵抗**: 2ノード構成では両端末ノードそれぞれに 120Ω の終端抵抗が必要
  - MCP2515 HAT 側: ジャンパで内蔵終端を有効化
  - Kvaser Memorator 側: 終端内蔵なし → DB9 ピン7(CANH) と ピン2(CANL) の間に 120Ω を外付け
  - **確認方法**: CANH-CANL 間をテスターで測定 → **約60Ω** が正常値（120Ω × 2個並列 = 60Ω）

---

## セキュリティアーキテクチャ

### ネットワークアクセス制御

- **ローカルLAN限定**: FastAPIをバインドするネットワークインターフェースをLAN側のみに制限
- **ポート**: HTTP **8080**（デフォルト固定、`config/settings.toml` の `server.port` で変更可能）
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

- **3ヶ月超ログ**: 内蔵SSD使用率が閾値（80%）を超えた場合に `ArchiveManager` が古いレコードからCSV+gzip圧縮してUSB SSDへ移行（常時起動ではないため定期実行ではなく容量トリガー）
- **USB SSD 80%超**: 最古のアーカイブから自動削除
- **車両プロファイル数**: 上限なし（PostgreSQLの行数制限のみ）
- **走行モード数**: 上限なし（CSVファイルサイズに依存）

### 機能拡張性

- **外部API（Post-MVP）**: FastAPIルーターを追加するだけで拡張可能
- **新しいアクチュエータ軸**: `ActuatorDriver` を継承してプラグイン的に追加可能
- **ボタンサーボ（Post-MVP）**: `ButtonServoDriver`（I2C/PCA9685）をインフラレイヤーに追加。RS-485（Modbus RTU）とは独立したI2Cバスで駆動するため、50ms制御ループと通信バスを共有せず競合しない。押下は「待機／押下」2ポジションの開ループ制御で、タイムスケジュールの時系列から呼び出す
- **新しい車速ソース**: `CANReader` を抽象化（CAN以外にもOBD2等を将来追加可能）
- **PID自動適合の同定/同調則**: `pid_tuning.py` は純粋ロジック（FOPDT同定・SIMC・コスト・座標降下）を
  ハードから分離。同定則（例: ブレーキ側独立同定）や同調則（例: Lambda/ZN）の差し替え・拡張が容易

#### PID自動適合の制御フロー（手動パス・個別API）

```
学習運転(開ループ) → /learning/train
   ├─ train_inverse_model        … 多項式Ridge 逆FFモデル（model_path、feature_spec を pkl に保存）
   ├─ estimate_dynamics_params   … クリープ等の物理定数
   └─ identify_fopdt → compute_pid_gains_simc … PID初期値（profile.pid_gains）
        ↓ refresh_active_profile（制御スタックへ反映）
/pid-tune/refine（規定パターン走行・max_runs 指定）
   build_tuning_trajectory（上限G/最高車速厳守）→ start_auto_drive（既存安全経路）
   → KPIMonitor.summary → tuning_cost → CoordinateDescentTuner（座標降下）
   → 最良ゲイン保存・反映
```
> 新規の制御/プラントコードは追加せず、既存の自動走行経路（走行前チェック・非常停止・KPI集計）を再利用。
> `train_and_apply`（training_service.py）が `/learning/train` の実処理を担い、`LearningCycleOrchestrator`
> と共有する（下記）。

#### 学習サイクル・オーケストレーション（LearningCycleOrchestrator、20260703-learning-process-revamp）

上記の手動パスを2段階に拡張し、WebUI 1操作（`POST /learning-cycle/start`）で自動進行させる
（`src/app/learning_cycle.py`）。学習運転→訓練→PID適合(10回)→**サイクル全ログで再学習**
（ゲイン上書きなし）→PID適合(5回、1段目ゲインから継続)。フェーズ間は停車保持ブレーキを
維持し続け（`run_pid_tuning_session(release_on_finish=False)`）、2段目完了または
中断・エラー時にのみ原点復帰で解放する。進捗（フェーズ・走行回数・最良コスト）は
既存の100ms WebSocket `broadcast_loop` が `orchestrator.progress` を pull 配信する
（新規イベントバスは作らない）。学習運転〜適合走行は `learning_cycles` テーブル・
`drive_sessions.cycle_id` で1サイクルとして紐付けられ、ログ画面で1項目に集約表示される。

> **2段階再学習の裏付け（2026-07-03 オフライン評価）**: 実ログ評価で、逆モデルの holdout 精度は
> 特徴量構成の選択（±5%程度）より訓練データ量（学習セッション1→4本で-16%）の影響が支配的で、
> 精度の主なボトルネックは開ループ学習データと閉ループ配備データの分布ギャップ（holdout R²が
> 負〜0.2）であることを確認した。サイクル全ログ（学習+閉ループ適合走行）での再学習はこのギャップを
> 直接埋める設計であり、特徴量チューニングより優先する。

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
  - pid_tuning（FOPDT同定・SIMCゲイン算出・規定パターンの上限G厳守・コスト・座標降下の収束）
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
- AC UPS接点出力によるAC断検知確認
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
- **AC UPS**: APC Smart-UPS 750 (SUA750JB) を使用。フル充電時に数分以上のバックアップが可能
  - 根拠: home_return() + PostgreSQL正常終了 + シャットダウンを30秒以内に完了する設計
  - AC断検知: NUT (Network UPS Tools) + apcsmart ドライバ経由（シリアル接続）

### パフォーマンス制約

- 50ms制御ループのジッタ: Raspberry Pi OSはリアルタイムOSではないため±5ms程度を許容
- Modbus RTU応答時間（MJ0162-12A 表4.1-1より）:
  - 内部処理時間（位置・ステータスレジスタ）: 最大 1ms
  - 従局トランスミッター活性化最小遅延（Param No.17）: 初期値 5ms（要チューニング、縮小可能）
  - レスポンス後インターメッセージギャップ: 1ms
  - 合計レスポンス遅延: 最大 7ms（Param No.17 デフォルト時）

**制御ループのタイムバジェット（38,400bps、2軸、FC03で6レジスタ読み取り）**:

| 処理 | 時間 |
|------|------|
| FC03 クエリー送信（8バイト × 0.286ms/byte） | 2.3ms |
| P-CON-CB 内部処理 + Param No.17 遅延 | 最大 6ms |
| FC03 レスポンス受信（17バイト × 0.286ms/byte） | 4.9ms |
| インターメッセージギャップ | 1ms |
| **1軸あたり小計** | **約 14ms** |
| **2軸合計（RS-485 逐次）** | **約 28ms** |

> ⚠️ 38,400bps では 2 軸逐次読み取りだけで約 28ms かかる。
> 10ms・20ms ループは不可能。**50ms ループ（20Hz）を推奨**。
> 位置指令（FC10）は値が変化した場合のみ送信することで実質的なレイテンシを低減する。
> Param No.17 を 1ms に縮小すると約 4ms 短縮可能（実機チューニングで検証）。

### python-can aarch64 既知バグ（要パッチ）

python-can 4.x を aarch64 (Raspberry Pi 5) で使用する場合、以下の2つのバグをパッチする必要がある。
パッチは `.venv` 内のファイルを直接修正する。`pip install --upgrade python-can` で上書きされたら再パッチが必要。

**バグ1: `ctypes.c_long` のサイズ不一致**

- 場所: `.venv/lib/.../can/interfaces/kvaser/canlib.py` line 509 付近
- 原因: aarch64 では `ctypes.c_long` が 8バイト。canlib ioctl に `c_long` + `size=4` を渡すと `canERR_PARAM`
- 修正: `ctypes.c_long(TIMESTAMP_RESOLUTION)` → `ctypes.c_uint(TIMESTAMP_RESOLUTION)`、`4` → `ctypes.sizeof(ctypes.c_uint)`

**バグ2: `canIOCTL_SET_LOCAL_TXACK` 未サポートエラー**

- 場所: 同ファイルの `canIoCtlInit` 呼び出し部
- 原因: Kvaser Memorator は `canIOCTL_SET_LOCAL_TXACK` 未サポートで `canERR_PARAM` を返す
- 注意: `canIoCtlInit` と `canIoCtl` は ctypes のキャッシュにより同一関数オブジェクトを返すため、
  `errcheck` が `__check_status_operation` に上書きされる。
  `except CANLIBInitializationError` では捕捉できず `except CANLIBError` が必要
- 修正: `canIoCtlInit(SET_LOCAL_TXACK, ...)` を `try/except CANLIBError` で囲む

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
| adafruit-circuitpython-pca9685 | PCA9685 I2C PWM（ボタンサーボ, Post-MVP） | 最新安定版（smbus2 での代替可） |
| numpy | 数値計算 | `>=1.25.0` |
| scipy | 補間 | `>=1.11.0` |
| pydantic | バリデーション | `>=2.0.0` v2 API |
| pytest | テスト | `>=7.0.0` |
| pytest-asyncio | 非同期テスト | `>=0.21.0` |
| ruff | Lint/Format | `>=0.1.0` |

**管理方針**:
- `pyproject.toml` で最小バージョンを指定（Python 3.13 対応のモダン構成）
- 再現性のため `pip freeze > requirements.lock` を使用
- ライブラリは `.venv` 内にのみインストール（グローバル禁止）
