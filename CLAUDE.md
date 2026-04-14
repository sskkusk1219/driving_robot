# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 言語
- すべての回答・コメント・説明は**日本語**で行うこと

## プロジェクトの目的
- シャシダイナモの上で車速トレースをするロボットを作製する

## 作りたいもの
- 運転モデルによって導いた基準車速に対するアクセル開度とブレーキ開度をフィードフォワード制御によってアクチュエータを動かし、
  基準車速と実車速のずれをフィードバック制御で調整するロボット

## アーキテクチャ全体像
```
基準車速CSV
    ↓ DriveDataLoader（10ms補間）
driving_model (rf_accel.pkl / rf_brake.pkl)
    ↓ アクセル開度[%] / ブレーキ開度[%]
CalibrationMap（開度%→位置mm 変換テーブル）
    ↓ アクチュエータ目標位置[mm]
FeedforwardController → PconClient (RS485 x2台)

シャシダイナモ CAN → CanReader（実車速[km/h]）
    ↓ 偏差
FeedbackController → 補正量をアクチュエータ目標位置に加算
```

## 開発コマンド

### 仮想環境のセットアップ
```bash
python3 -m venv venv
source venv/bin/activate
pip install pymodbus python-can cantools pandas numpy matplotlib pyyaml RPi.GPIO
```

### サンプルの実行
```bash
source venv/bin/activate
python sample/sample_1.py   # 同期Modbusデモ
python sample/sample_2.py   # 非同期Modbusデモ
```

### シリアルポートの確認
```bash
ls /dev/ttyUSB*   # シリアルポートは /dev/ttyUSB0 を使用（sample_1.py 設定済み）
```

## 作り方
- いきなりすべてのコードを書くのではなく、1つずつ段階を踏んで作ること
- いきなりコードを書くのではなく、まずはプロジェクトの構成や必要な機能を整理し、設計を行うこと
- 設計が固まったら、コードを書く前に必要なライブラリやツールを調査し、選定すること
- コーディングは設計に基づいて行い、必要に応じて設計を見直すこと
- コードを書いたら、必ずテストを行い、動作確認をすること

### Phase 1: 1つのアクチュエータを動かす
**実装ファイル**: `src/actuator/client.py`
- `PconClient` クラスを実装（`AsyncModbusSerialClient` ベース）
  - `connect()` / `close()`
  - `enable_modbus()` — 0x0427 コイルON
  - `reset_alarm()` — 0x0407 ON→OFF
  - `servo_on()` / `servo_off()` — 0x0403
  - `home_return()` — 0x040B エッジ検出・HEND（0x9005 bit4）待機
  - `move_absolute(pos_mm, speed_mms)` — 0x9900-0x9908、CTLF=0x0000
  - `move_relative(delta_mm, speed_mms)` — CTLF=0x0008
- 単位変換: mm × 100 → レジスタ値（0.01mm単位）
- **テスト**: 実機接続後、原点復帰 → 絶対位置移動 → 相対位置移動を手動実行

### Phase 2: 1つのコントローラのエラー・電流値・位置を50msで監視
**実装ファイル**: `src/actuator/monitor.py`
- `PconMonitor` クラスを実装
  - `read_status()` — 0x9000-0x9001（位置）・0x9005（ステータス）・0x9002（アラーム）・0x900C-0x900D（電流）を一括読取り
  - 50ms周期でポーリング
  - タイムスタンプ付きでコンソール出力
- **テスト**: 手動でアクチュエータを動かしながら監視値が更新されることを確認

### Phase 3: asyncio非同期でアクチュエータ動作と監視を並行実行
**実装ファイル**: `src/actuator/client.py`（非同期版）、`src/actuator/monitor.py`
- `PconMonitor` を `asyncio.Task` として実行
- 制御コルーチンと監視タスクを `asyncio.gather()` で並行実行
- エラー発生時のタスクキャンセル処理
- **テスト**: 移動コマンド実行中に監視が50ms周期で途切れないことを確認

### Phase 4: 基準車速CSVを10msデータに変換 + driving_modelでアクセル/ブレーキ開度を予測
**実装ファイル**: `src/data/csv_loader.py`、`src/model/predictor.py`

`DriveDataLoader`:
- 基準車速CSVを読み込み（列: `Time[s]`, `Speed[km/h]`）
- pandas で10ms間隔にリサンプリング（線形補間）

`DrivingModelPredictor`:
- `models/rf_accel.pkl`・`models/rf_brake.pkl`（driving_model リポジトリ）をロード
- `data/processed/scaler.pkl` で正規化/逆正規化
- ラグ特徴量（1, 5, 10, 20ステップ分）・ローリング統計量を計算
- 入力: 車速時系列 → 出力: アクセル開度[0-100%]・ブレーキ開度[0-100%]
- **テスト**: サンプルCSVで予測値が物理的に妥当な範囲内にあることを確認

### Phase 5: キャリブレーション（開度%→アクチュエータ位置mm）
**実装ファイル**: `src/data/calibration.py`、`data/calibration/accel_map.csv`、`data/calibration/brake_map.csv`
- アクチュエータを手動操作して開度[%]と位置[mm]の対応テーブルを計測・保存
- `CalibrationMap` クラスで CSV読み込み + 線形補間（`numpy.interp`）
- `accel_pct_to_mm(pct)` / `brake_pct_to_mm(pct)` を提供
- **テスト**: 0%/50%/100%の3点でアクチュエータが正しい位置に移動することを確認

### Phase 6: 1台をアクセル開度に従って動作させる（フィードフォワード制御）
**実装ファイル**: `src/control/feedforward.py`
- `FeedforwardController` クラスを実装
  - `DriveDataLoader` + `DrivingModelPredictor` + `CalibrationMap` を統合
  - 10ms周期で目標位置を更新する制御ループ（`asyncio` ベース）
  - アクセル用 `PconClient`（SLAVE_ID=1）に位置指令を送信
- **テスト**: 基準車速CSVを再生してアクチュエータが追従することを確認

### Phase 7: 2台をアクセル開度・ブレーキ開度に従って動作させる（フィードフォワード制御）
**実装ファイル**: `src/control/feedforward.py`（拡張）
- ブレーキ用 `PconClient`（SLAVE_ID=2）を追加
- 2台分の位置指令を同一制御ループ内で送信
- **テスト**: アクセル・ブレーキ両アクチュエータが同期して動作することを確認

### Phase 8: シャシダイナモCAN経由で実車速を読み取る
**実装ファイル**: `src/can/reader.py`、`src/can/dbc/`（DBCファイル格納先）
- `CanReader` クラスを実装（`python-can` + Kvaser USB-CAN）
  - `cantools` でDBCファイルを読み込み、CAN IDからシグナルをデコード
  - `asyncio` 非同期受信ループ
  - 実車速[km/h]を非同期キューで提供
- **テスト**: シャシダイナモ接続時に実車速が正しくデコードされることを確認

### Phase 9: シャシダイナモで走行・実車速と基準車速の偏差確認
**実装ファイル**: `src/data/logger.py`、`src/main.py`（統合）
- `DataLogger` — CSV保存（時刻・基準車速・実車速・アクセル/ブレーキ開度・目標位置）
- 走行後に偏差をグラフ化（matplotlib）
- フィードフォワードのみで走行し、偏差の傾向を記録
- **テスト**: ログCSVで基準車速と実車速の偏差を確認、フィードバック制御の必要性を判断

### Phase 10: フィードバック制御で実車速を基準車速に追従させる
**実装ファイル**: `src/control/feedback.py`
- `FeedbackController` クラスを実装（PID制御）
  - 入力: 実車速偏差[km/h] → 出力: 開度補正量[%]
  - フィードフォワード出力 + フィードバック補正量を合算して `PconClient` に送信
  - PIDゲイン（Kp, Ki, Kd）を `config/pid_gains.yaml` で管理
- **テスト**: シャシダイナモで実車速が基準車速に収束することを確認

### Phase 11: 非常停止スイッチの実装
**実装ファイル**: `src/safety/emergency_stop.py`
- `EmergencyStop` クラスを実装
  - Raspberry Pi GPIO入力監視（`asyncio` タスク）
  - 押下時: 全アクチュエータへ即座に `home_return()` を呼び出し
  - 状態を `asyncio.Event` で他タスクに通知
- **テスト**: 走行中に非常停止スイッチを押してアクチュエータが安全に停止することを確認

### Phase 12: シグナルタワーの実装
**実装ファイル**: `src/safety/signal_tower.py`
- `SignalTower` クラスを実装（Raspberry Pi GPIO出力）
  - 待機中（緑）・走行中（青）・アラーム（赤）・非常停止（赤点滅）
  - 状態遷移を `asyncio.Task` で管理
- **テスト**: 各状態遷移でシグナルタワーの表示が正しく切り替わることを確認

### Phase 13: UIの作成
- TBD（要件確認後に詳細化）

## ハード構成
他に必要なものがあれば適宜追加すること
- PC linux-raspberry pi
- P-CON-CB（アクチュエータコントローラ）2つ
- IAI製アクチュエータ 2つ
- 車両のアクセルペダルとブレーキペダルに設置
- USB-RS485変換ケーブル 2つ（P-CON-CBとPCを接続）
- CAN読み取り装置: KvaserのUSB-CANインターフェース
  - シャシダイナモの実車速
- 非常停止スイッチ: 押された瞬間にアクチュエータを原点復帰する
- シグナルタワー

## ソフト構成
- git, GitHubでコード管理を行うこと（リポジトリ名: "driving_robot"）
- `/sample/sample_1.py`、`/sample/sample_2.py`、`docs/MODBUS(MJ0162-12A).pdf` を参考にして
  pymodbusを使用してP-CON-CBと通信するコードを書くこと
- 運転モデルは driving_model リポジトリのモデルを使用すること
  `https://github.com/sskkusk1219/driving_model/tree/main`

### ディレクトリ構成（本番コードは src/ に格納）
```
driving_robot/
├── src/
│   ├── actuator/              # P-CON-CB 通信・アクチュエータ制御
│   │   ├── client.py          # PconClient（AsyncModbusSerialClient ベース）
│   │   └── monitor.py         # PconMonitor（50ms 周期で位置・電流・ステータス読取り）
│   ├── can/                   # CAN 通信（シャシダイナモ実車速取得）
│   │   ├── reader.py          # CanReader（Kvaser USB-CAN + cantools DBCデコード）
│   │   └── dbc/               # DBCファイル格納先（後で格納）
│   ├── control/               # 制御ロジック
│   │   ├── feedforward.py     # FeedforwardController（CSV→モデル→開度→位置→アクチュエータ）
│   │   └── feedback.py        # FeedbackController（PID、実車速偏差補正）
│   ├── data/                  # データ処理
│   │   ├── csv_loader.py      # DriveDataLoader（基準車速CSV→10ms補間）
│   │   ├── calibration.py     # CalibrationMap（開度%→位置mm 変換テーブル）
│   │   └── logger.py          # DataLogger（走行ログCSV保存）
│   ├── model/                 # driving_model 推論
│   │   └── predictor.py       # DrivingModelPredictor（rf_accel/rf_brake ロード・推論）
│   ├── safety/                # 安全機能
│   │   ├── emergency_stop.py  # EmergencyStop（GPIO監視・即時原点復帰）
│   │   └── signal_tower.py    # SignalTower（GPIO出力・状態表示）
│   └── main.py                # エントリーポイント
├── data/
│   └── calibration/           # キャリブレーションデータ（accel_map.csv, brake_map.csv）
├── config/
│   └── pid_gains.yaml         # PIDゲイン設定
├── sample/                    # 参考サンプルコード（変更不要）
└── docs/                      # データシート類（PDF）

driving_model/（別リポジトリ: /home/raspi5_16gb/projects/driving_model）
├── models/
│   ├── rf_accel.pkl           # アクセル開度予測 Random Forest
│   └── rf_brake.pkl           # ブレーキ開度予測 Random Forest
└── data/processed/
    └── scaler.pkl             # MinMaxScaler（正規化・逆正規化）
```

## P-CON-CB 主要Modbusレジスタ早見表

| アドレス | 内容 | R/W |
|---------|------|-----|
| 0x0403 | サーボON（コイル） | W |
| 0x0407 | アラームリセット（コイル） | W |
| 0x040B | 原点復帰 HOME（コイル、エッジ検出） | W |
| 0x0427 | Modbus操作権有効化（コイル） | W |
| 0x9000-0x9001 | 現在位置（0.01mm単位、32bit符号付き） | R |
| 0x9002 | アラームコード | R |
| 0x9005 | デバイスステータス1（bit4=HEND、bit10=重故障） | R |
| 0x900C-0x900D | 電流値（mA、32bit） | R |
| 0x9900-0x9908 | 位置指令レジスタ群（PCMD/INP/VCMD/ACMD等） | W |

- CTLF: `0x0000`=絶対位置移動、`0x0008`=相対位置移動
- 単位: 位置値 = mm × 100（0.01mm単位）、速度値 = mm/s × 100
- スレーブID: アクセル用=1、ブレーキ用=2

## documents
- `docs/DS_USB_RS485_CABLES.pdf` — RS485ケーブルのデータシート
- `docs/MODBUS(MJ0162-12A).pdf` — pymodbusを使用する際の通信プロトコル（1章から5章を参照）
- `docs/PCON-CB(MJ0342-4G).pdf` — PCON-CBのデータシート

## コーディングスタイル
- 基本はpython
- コメントは必要に応じて適切に追加すること
- ドキュメントストリングを使用して関数やクラスの説明を行うこと
- コードは読みやすく、理解しやすいように書くこと
- コードの再利用性を考慮して、関数やクラスを適切に分割すること
- 適切にファイルを分割し、機能ごとに整理すること
  1つファイルにつき、500行程度を目安にすること
  可読性が高い構成にすること
- ライブラリは仮想環境にインストールすること
- asyncio, pymodbus, python-can, cantools, pandas, numpy を使用し、必要なライブラリは適宜インストールの提案をすること

## 禁止事項
- IDやパスワードなどの機密情報にアクセスしないこと
- グローバル環境にライブラリをインストールしないこと
