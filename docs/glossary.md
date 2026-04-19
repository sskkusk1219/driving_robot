# プロジェクト用語集 (Glossary)

## 概要

このドキュメントは、driving_robot プロジェクト内で使用される用語の定義を管理します。

**更新日**: 2026-04-17

---

## ドメイン用語

### 車両プロファイル

**定義**: 1台の試験車両に紐づく設定・データの集合体

**説明**: アクセル・ブレーキの最大開度、PIDゲイン、停止判定設定、キャリブレーションデータ、運転モデルをまとめた単位。複数の車両を扱う場合は車両ごとにプロファイルを作成する。

**関連用語**: キャリブレーションデータ、運転モデル、最大開度

**使用例**:
- 「Prius_2024のプロファイルを選択してキャリブレーションを実施する」
- 「プロファイルに紐づく運転モデルがない場合は学習運転が必要」

**英語表記**: Vehicle Profile

---

### キャリブレーション

**定義**: アクセル・ブレーキアクチュエータのゼロ位置（接触点）とフル位置（最大踏み込み位置）を自動検出するプロセス

**説明**: 電流急増検出アルゴリズムを使い、アクチュエータを低速で移動させながらペダルへの接触点とメカニカルエンド近傍のフル位置を検出する。キャリブレーション中は車両プロファイルの最大開度設定を無視し、全ストロークを検出する。

**関連用語**: 接触点（ゼロ位置）、フル位置、ストローク、電流急増検出

**使用例**:
- 「車両を乗せ換えた後は必ずキャリブレーションを実施する」
- 「キャリブレーションはアクセル・ブレーキ独立して実施する」

**英語表記**: Calibration

---

### 接触点 / ゼロ位置

**定義**: アクチュエータがペダルに最初に触れる位置（開度0%に相当）

**説明**: キャリブレーション時に電流急増で検出する。アクチュエータのpulse単位で記録される。自動運転・学習運転では開度0%がこの位置に対応する。

**関連用語**: フル位置、ストローク、キャリブレーション

**英語表記**: Zero Position / Contact Point

---

### フル位置

**定義**: アクチュエータが最大踏み込み位置まで達した時の位置（開度100%に相当）

**説明**: キャリブレーション時に2回目の電流急増（機械的終端への到達）で検出する。実際の走行では車両プロファイルの最大開度設定によりフル位置まで踏み込まれない場合がある。

**関連用語**: 接触点、ストローク、最大開度

**英語表記**: Full Position

---

### ストローク

**定義**: 接触点からフル位置までのアクチュエータ移動量

**計算式**: `stroke = full_pos - zero_pos` [pulse]

**関連用語**: 接触点、フル位置

---

### 最大開度

**定義**: 車両プロファイルに設定されたアクセル・ブレーキの最大踏み込み割合

**説明**: アクチュエータが全ストロークの何%まで踏み込むかを制限する。キャリブレーション時は無視される。自動運転・学習運転・手動操作時は必ず守られる。単位: %（0〜100）

**関連用語**: 車両プロファイル、ストローク

**英語表記**: Max Opening

---

### 学習運転

**定義**: 運転モデルを構築するためのデータ収集専用の走行モード

**説明**: 速度・加速度グリッドで定義された各パターンを自動走行し、そのときのアクセル・ブレーキ開度と実車速を記録する。車両プロファイルの最大開度・G上限を超えるパターンは自動スキップされる。

**関連用語**: 運転モデル、走行パターン、学習ログ

**英語表記**: Learning Drive

---

### 走行パターン

**定義**: 学習運転で自動走行する1組の目標値（基準速度・基準加速度のペア）

**説明**: 速度グリッド × 加速度グリッドの全組み合わせが走行パターンとして定義される。各パターンを走行した際のアクセル・ブレーキ開度が学習ログとして記録され、運転モデルのグリッドデータになる。

**コード識別子**: `LearningPattern`（`src/domain/learning_drive.py`）

**英語表記**: Learning Pattern

---

### 学習ログ

**定義**: 学習運転の1走行パターンで収集した時系列データ

**説明**: 走行パターンごとに記録され、実車速・アクセル開度・ブレーキ開度の時系列が含まれる。運転モデル生成の入力データとなる。

**コード識別子**: `LearningLog`（`src/domain/learning_drive.py`）

**英語表記**: Learning Log

---

### 運転モデル

**定義**: 目標車速・加速度とアクセル・ブレーキ開度の関係を表したマップデータ

**説明**: 学習運転のログから生成される。フィードフォワード制御の入力として使用される。速度・加速度グリッドに対してアクセル・ブレーキ開度を保持し、グリッド間は線形補間する。`.pkl` 形式でファイル保存され、車両プロファイルに紐づく。

**関連用語**: フィードフォワード制御、学習運転、車両プロファイル

**英語表記**: Driving Model

---

### 走行モード

**定義**: 時系列の基準車速（CSVファイル）に名前を付けて管理する単位

**説明**: WLTPやNEDCなどの運転サイクルをCSVとしてアップロードして作成する。自動運転実行時にどの走行モードで走行するかを選択する。

**関連用語**: 基準車速、運転サイクル、自動運転

**使用例**:
- 「WLTP_Class3走行モードで自動走行を開始する」
- 「走行モード管理画面から新しいサイクルをアップロードする」

**英語表記**: Driving Mode

---

### 基準車速

**定義**: 走行モードに含まれる目標とする車速の時系列データ

**説明**: 時刻[s]と車速[km/h]のペアで構成されるCSVファイル。自動運転中にアクチュエータはこの基準車速を追従するよう制御される。

**関連用語**: 走行モード、フィードフォワード制御、追従誤差

**英語表記**: Reference Speed

---

### 追従誤差

**定義**: 基準車速と実車速の差

**計算式**: `error = actual_speed - ref_speed` [km/h]

**関連用語**: 基準車速、PID制御

**英語表記**: Tracking Error

---

### 走行前チェック

**定義**: 自動運転・学習運転・手動操作を開始する前に実施する6項目の安全確認

**説明**: 通信確認（ttyUSB0/1・CAN）、サーボ状態、キャリブレーション有無、プロファイル選択、UPS残量、アクチュエータ位置を確認する。1項目でもNGの場合は走行開始できない。

**関連用語**: 安全システム、UPS

**英語表記**: Pre-Check

---

### AC電源断

**定義**: AC100V供給が途絶えた状態

**説明**: AC UPSの接点出力がGPIO27（物理ピン13）をLOWにすることで検知する。検知後は安全停止シーケンスを即座に起動し、UPSのバックアップ給電中（30秒以上）に全処理を完了させる。

**関連用語**: AC UPS、安全停止シーケンス

**英語表記**: AC Power Loss

---

### 安全停止シーケンス

**定義**: AC電源断・非常停止・重大エラー発生時に実行される定型の停止手順

**手順**:
1. 制御ループ停止・走行セッション終了
2. 両軸アクチュエータ原点復帰（`asyncio.gather` で並列実行）
3. 走行ログをPostgreSQLにフラッシュ
4. PostgreSQL正常終了（`systemctl stop postgresql`）
5. Raspberry Piシャットダウン

**制約**: 上記5ステップをAC断後30秒以内に完了する設計

**実装箇所**: `src/domain/safety_monitor.py`（`SafetyMonitor`）

**英語表記**: Safety Stop Sequence

---

### 原点復帰

**定義**: アクチュエータをホームポジション（RCP6-RODのロッドが最短位置まで収縮した状態。電源断時はペダルスプリングの反力で自然にこの位置に戻る）に戻す動作

**説明**: 非常停止・通常停止・AC電源断時に実行される。両軸を asyncio.gather で並列に実行する。P-CON-CB に原点復帰コマンドを送信する。

**関連用語**: 非常停止、ホームポジション

**英語表記**: Home Return

---

### セッション

**定義**: 1回の走行（自動運転・学習運転・手動操作）の単位

**説明**: 走行開始から停止までを1セッションとして管理する。PostgreSQL の `drive_sessions` テーブルに記録され、走行ログは `drive_logs` テーブルに紐づく。

**関連用語**: 走行ログ、DriveSession

**英語表記**: Drive Session

---

## 技術用語

### Modbus RTU

**定義**: シリアル通信（RS-485）上で動作するマスタ・スレーブ型産業用通信プロトコル

**本プロジェクトでの用途**: Raspberry Pi（マスタ）からP-CON-CB（スレーブ）へのアクチュエータ位置指令・状態読み取り

**接続**: USB-RS485変換ケーブル経由（ttyUSB0: アクセル、ttyUSB1: ブレーキ）

**ライブラリ**: pymodbus 3.x

**関連ドキュメント**: `docs/architecture.md` → Modbus RTU 通信設定

---

### P-CON-CB

**定義**: IAI製アクチュエータコントローラ。Modbus RTUで位置指令を受け付ける

**本プロジェクトでの用途**: RCP6-RODアクチュエータを駆動する。アクセル用（SLAVE_ID=1, 24V電源）とブレーキ用（SLAVE_ID=2, 24V電源）の2台を使用

**電源**: 24V（AC UPS経由でバックアップあり）

---

### RCP6-ROD

**定義**: IAI製ロッドタイプ電動アクチュエータ

**本プロジェクトでの用途**: アクセルペダルとブレーキペダルを機械的に押し込むアクチュエータ。電磁ブレーキなし（型式Bなし）のため、電源断時はペダルの反力（スプリング）で収縮する。

**関連用語**: P-CON-CB、原点復帰

---

### CAN bus（Controller Area Network）

**定義**: 自動車向け車内通信規格

**本プロジェクトでの用途**: シャシダイナモからリアルタイム車速データを受信する。DBCファイル（自作、`config/can/`に配置）でシグナルをデコードする。

**インターフェース**: Kvaser USB-CAN（Linuxドライバ使用）

**ライブラリ**: python-can 4.x（Kvaser backend）

---

### DBC ファイル

**定義**: CANバスのメッセージ・シグナル定義ファイル（Database CAN）

**本プロジェクトでの用途**: シャシダイナモから送信される車速CANフレームのID・ビット位置・スケール係数を定義する自作ファイル。`config/can/` に配置。

---

### asyncio

**定義**: Pythonの標準ライブラリに含まれる非同期I/Oフレームワーク

**本プロジェクトでの用途**: 50ms制御ループ・100msログ書き込み・WebSocket配信・GPIO監視を単一プロセスで並列実行する。`asyncio.gather()` でアクセル・ブレーキの両軸に同時送信する。

---

### FastAPI

**定義**: Pythonの非同期対応WebフレームワークでOpenAPI仕様の自動生成機能を持つ

**本プロジェクトでの用途**: Web GUIのAPIサーバー、WebSocketによるリアルタイムデータ配信

**バージョン**: 最新安定版

---

### AC UPS

**定義**: AC100Vを入力とし、停電時に5V・24V両系統へのバックアップ給電と接点出力によるAC断通知を行う無停電電源装置（機種TBD）

**本プロジェクトでの用途**: Raspberry Pi 5（5V系）とP-CON-CB（24V系）を1台でバックアップ。接点出力を GPIO27（物理ピン13）に接続し、AC断を検知して安全停止シーケンスを起動する。[要確認: 機種確定後にGPIOピン番号を更新]

**バックアップ時間要件**: AC断後30秒以上（home_return + PostgreSQL正常終了 + シャットダウンを30秒以内に完了するため）

**関連用語**: AC電源断、安全停止シーケンス、原点復帰

---

## 略語・頭字語

### FF（フィードフォワード）

**正式名称**: Feedforward Control

**意味**: 運転モデルを使って基準車速から事前にアクチュエータ位置を予測する制御方式。偏差が発生する前に制御出力を出すため応答が速い。

**本プロジェクトでの使用**: `src/domain/control/feedforward.py`（`FeedforwardController`）

---

### PID

**正式名称**: Proportional-Integral-Derivative Control

**意味**: 実車速と基準車速の偏差に対して比例・積分・微分の3要素で補正量を算出するフィードバック制御。FFコントロールの残差を補正するために使用する。

**本プロジェクトでの使用**: `src/domain/control/pid.py`（`PIDController`）

---

### UPS

**正式名称**: Uninterruptible Power Supply

**意味**: 停電時にも電力を継続供給する無停電電源装置

**本プロジェクトでの使用**: AC UPS 1台で5V系（Raspberry Pi用）と24V系（P-CON-CB用）の両系統をバックアップ

---

### WLTP

**正式名称**: Worldwide Harmonized Light-Duty Vehicles Test Procedure

**意味**: 乗用車・小型商用車の燃費・排ガス試験に使用される国際標準運転サイクル

**本プロジェクトでの使用**: 走行モードとして最も一般的に使用される運転サイクル

---

### NEDC

**正式名称**: New European Driving Cycle

**意味**: 欧州の旧燃費・排ガス試験サイクル（現在はWLTPに移行）

**本プロジェクトでの使用**: 走行モードの一つとして対応

---


## アーキテクチャ用語

### レイヤードアーキテクチャ

**定義**: システムを役割ごとにWebレイヤー・アプリケーションレイヤー・ドメインレイヤー・インフラレイヤーに分割する設計パターン

**本プロジェクトでの適用**:
```
src/web/    → HTTP・WebSocket受付
src/app/    → ユースケース・状態管理
src/domain/ → 制御ロジック・安全監視
src/infra/  → HW通信・DB・ファイルI/O
```

**関連コンポーネント**: RobotController、ActuatorDriver、LogWriter

---

### コンポーネント一覧

| クラス名 | 実装ファイル | 責務 |
|---------|------------|------|
| `RobotController` | `src/app/robot_controller.py` | システム状態機械・ユースケース調整 |
| `SessionManager` | `src/app/session_manager.py` | 走行セッションのライフサイクル管理 |
| `CalibrationManager` | `src/domain/calibration.py` | キャリブレーション実行・電流急増検出 |
| `LearningDriveManager` | `src/domain/learning_drive.py` | 学習運転の実行・走行パターン管理 |
| `SafetyMonitor` | `src/domain/safety_monitor.py` | 非常停止・AC断検知・過電流監視・逸脱監視 |
| `FeedforwardController` | `src/domain/control/feedforward.py` | 運転モデルからの開度算出 |
| `PIDController` | `src/domain/control/pid.py` | 追従誤差からのフィードバック補正 |
| `ActuatorDriver` | `src/infra/actuator_driver.py` | P-CON-CB Modbus RTU通信 |
| `CANReader` | `src/infra/can_reader.py` | Kvaser USB-CANからの車速受信 |
| `GPIOMonitor` | `src/infra/gpio_monitor.py` | GPIO割り込み監視（非常停止・AC断） |
| `LogWriter` | `src/infra/log_writer.py` | PostgreSQLへの100ms周期ログ書き込み |
| `ArchiveManager` | `src/infra/archive_manager.py` | 容量トリガーによるログのCSV+gzip圧縮・USB SSD移行 |

---

### 制御ループ

**定義**: 一定周期でセンサ読み取り → 制御量計算 → アクチュエータ指令を繰り返すループ

**本プロジェクトでの適用**: 50ms周期（20Hz）で CAN車速読み取り → FF+PID計算 → 両軸位置指令送信を asyncio で実行

**実装箇所**: `src/domain/control/drive_loop.py`

---

### フェイルセーフ

**定義**: システムに障害が発生したとき、安全な状態（故障側）に移行する設計原則

**本プロジェクトでの適用**: 例外発生時・通信断時・非常停止時はすべて原点復帰（ペダルを完全に離す）方向に動作する。Python の `finally` ブロックで保証する。

---

## ステータス・状態

### システム状態（RobotController）

| 状態 | 意味 | 遷移条件 | 次の状態 |
|------|------|---------|---------|
| BOOTING | 起動中・通信確認中 | 起動 | STANDBY / ERROR |
| STANDBY | 通信確認済み・初期化待ち | 通信OK | INITIALIZING |
| ERROR | 通信エラー等の異常 | 通信NG | STANDBY（エラー解除後） |
| INITIALIZING | アラームリセット・サーボON中 | 「初期化」ボタン | READY |
| READY | 走行可能な待機状態 | 初期化完了 | CALIBRATING / PRE_CHECK |
| CALIBRATING | キャリブレーション実行中 | キャリブレーション開始 | READY（成功/失敗） |
| PRE_CHECK | 走行前チェック実行中 | 走行開始要求 | RUNNING / READY |
| RUNNING | 走行中（自動/学習/手動） | チェックOK | READY / EMERGENCY |
| EMERGENCY | 非常停止・原点復帰中 | 非常停止スイッチ / 安全監視 | READY（リセット後） |

### セッション状態（DriveSession.status）

| ステータス | 意味 |
|-----------|------|
| `running` | 走行中 |
| `completed` | 正常完了 |
| `error` | エラー停止 |
| `emergency` | 非常停止 |

---

## データモデル用語

### VehicleProfile

**定義**: 車両プロファイルのデータモデル

**主要フィールド**:
- `id`: UUID（プライマリーキー）
- `name`: プロファイル名（ユニーク）
- `max_accel_opening`: アクセル最大開度 [%]
- `max_brake_opening`: ブレーキ最大開度 [%]
- `max_speed`: 最高車速 [km/h]
- `max_decel_g`: 最大減速G [G]
- `pid_gains`: PIDゲイン（JSON）
- `calibration`: キャリブレーションデータ（外部キー）
- `model_path`: 運転モデルファイルパス

**関連エンティティ**: CalibrationData、DriveSession

---

### DrivingMode

**定義**: 走行モードのデータモデル

**主要フィールド**:
- `id`: UUID
- `name`: 走行モード名（例: `WLTP_Class3`）
- `reference_speed`: 基準車速の時系列リスト（`SpeedPoint` のリスト）

**コード識別子**: `DrivingMode`（`src/models/driving_mode.py`）

---

### SpeedPoint

**定義**: 基準車速の1時刻分のデータ点

**主要フィールド**:
- `time_s`: 時刻 [s]
- `speed_kmh`: 基準車速 [km/h]

**コード識別子**: `SpeedPoint`（`src/models/driving_mode.py`）

---

### PIDGains

**定義**: PID制御のゲイン設定

**主要フィールド**:
- `kp`: 比例ゲイン
- `ki`: 積分ゲイン
- `kd`: 微分ゲイン

**コード識別子**: `PIDGains`（`src/models/profile.py`、`VehicleProfile.pid_gains` に内包）

---

### StopConfig

**定義**: 自動停止判定の設定

**主要フィールド**:
- `deviation_threshold_kmh`: 逸脱閾値 [km/h]（この値を超えると逸脱とみなす）
- `deviation_duration_s`: 逸脱継続時間 [s]（この時間以上逸脱が続いたら自動停止）

**使用例**: PRD既定値は `deviation_threshold_kmh=2.0`、`deviation_duration_s=4.0`（±2.0km/h・4.0秒以上で自動停止）

**コード識別子**: `StopConfig`（`src/models/profile.py`、`VehicleProfile.stop_config` に内包）

---

### DriveLog

**定義**: 100ms周期で記録される走行データの1レコード

**主要フィールド**:
- `session_id`: セッションID（外部キー）
- `timestamp`: 記録時刻
- `ref_speed_kmh`: 基準車速 [km/h]（自動運転時）
- `actual_speed_kmh`: 実車速 [km/h]
- `accel_opening`: アクセル開度 [%]
- `brake_opening`: ブレーキ開度 [%]
- `accel_current`: アクセル電流値 [mA]
- `brake_current`: ブレーキ電流値 [mA]

**制約**: `session_id + timestamp` でユニーク

---

## エラー・例外

### ActuatorCommunicationError

**クラス名**: `ActuatorCommunicationError`

**発生条件**: P-CON-CBとのModbus RTU通信が失敗したとき（タイムアウト・CRCエラー等）

**対処方法**: 制御ループを停止し、緊急停止シーケンス（原点復帰）を実行する

**例**:
```python
raise ActuatorCommunicationError(axis="accel", message="タイムアウト: 5ms超過")
```

---

### CalibrationError

**クラス名**: `CalibrationError`

**発生条件**: ゼロフル検出に失敗またはバリデーション（ストローク妥当性等）でNGのとき

**対処方法**: GUIにエラー内容を表示し、キャリブレーション結果は保存しない。再キャリブレーションが必要。

---

### PreCheckError

**クラス名**: `PreCheckError`

**発生条件**: 走行前チェックで1つ以上のNGが発生したとき

**対処方法**: GUIに失敗した項目を表示し、走行を開始しない。問題を解消してから再試行。

---

### SafetyError

**クラス名**: `SafetyError`

**発生条件**: 過電流検知・逸脱超過・CAN受信タイムアウト等の安全監視による停止

**対処方法**: 緊急停止（原点復帰）を実行し、GUIにEMERGENCY状態を表示する。リセット操作でREADYに戻れる。

---

## 計算・アルゴリズム

### 電流急増検出

**定義**: キャリブレーション時に機械的終端（接触点・フル位置）を検出するアルゴリズム

**計算式**:
```
moving_avg = 直近5サンプル（50ms分）の電流移動平均
threshold  = baseline_current × 1.5
接触点/フル位置 = current_ma > moving_avg + threshold
```

**実装箇所**: `src/domain/calibration.py`（`CalibrationManager`）

> ⚠️ この電流急増検出はキャリブレーション時の接触点・フル位置検出専用。走行中の過電流保護は別ロジックであり `src/domain/safety_monitor.py`（`SafetyMonitor`）が担当する。

---

### フィードフォワード制御（開度算出）

**定義**: 運転モデルから基準車速・加速度を入力としてアクチュエータ開度を算出する

**計算式**:
```
(ff_accel%, ff_brake%) = model.predict(ref_speed, ref_accel)
final_accel = clamp(ff_accel + pid_correction, 0, max_accel_opening)
final_brake = clamp(ff_brake - pid_correction, 0, max_brake_opening)
```

**実装箇所**: `src/domain/control/feedforward.py`

---

### PIDフィードバック補正

**定義**: 追従誤差（基準車速 - 実車速）から補正量を計算する制御則

**計算式**:
```
error      = ref_speed - actual_speed
correction = Kp × error + Ki × ∫error dt + Kd × d(error)/dt
```

**実装箇所**: `src/domain/control/pid.py`
