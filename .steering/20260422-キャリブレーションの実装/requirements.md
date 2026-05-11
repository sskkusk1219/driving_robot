# 要求内容

## フェーズ2（追加実装）

`src/domain/calibration.py` の64行目付近にある TODO を実装する:
バリデーション成功後に `profile_id` のプロファイルへ `CalibrationData` を永続化する。

## 概要（フェーズ1: 完了済み）

`RobotController.run_calibration()` のスタブ実装を実際の `CalibrationManager` に接続し、Web API エンドポイントを追加することで、キャリブレーション機能を完全に動作可能にする。

## 背景

`CalibrationManager`（`src/domain/calibration.py`）はアクセル・ブレーキのゼロフルキャリブレーションロジックを持つが、`RobotController.run_calibration()` はスタブのまま（`success=False, error_message="未実装"`）。  
走行前チェック #3 がキャリブレーションデータを必要とするため、実機でキャリブレーションを実行できる状態にすることが急務。

## 実装対象の機能

### 1. RobotController と CalibrationManager の接続

- `RobotController.__init__()` に `calibration_manager` パラメータを追加（Optional）
- `run_calibration()` を `CalibrationManager.run_calibration()` に委譲
- `CalibrationManagerProtocol` を定義して疎結合を保つ

### 2. Web API エンドポイント追加

- `POST /api/v1/drive/calibrate` エンドポイント
- `CalibrationResultResponse` スキーマ（success, error_message, data）

### 3. ファクトリ更新

- `build_real_controller()` で `CalibrationManager` を生成して `RobotController` に渡す

## 受け入れ条件

### RobotController 統合
- [ ] `CalibrationManager` を注入した `RobotController` が `run_calibration()` でマネージャを呼ぶ
- [ ] `CalibrationManager` が None の場合でも状態遷移（CALIBRATING→READY）は正常に行われる
- [ ] 状態遷移テストが引き続きパスする

### Web API
- [ ] `POST /api/v1/drive/calibrate` が 200 で `CalibrationResultResponse` を返す
- [ ] キャリブレーション中に `InvalidStateTransition` が発生すると 409 を返す

### ファクトリ
- [ ] `build_real_controller()` が `CalibrationManager` を生成して渡す

## スコープ外

以下はこのフェーズでは実装しません:

- キャリブレーション結果の PostgreSQL 永続化
- プロファイルへの CalibrationData 保存
- UI 画面の実装

## 参照ドキュメント

- `docs/functional-design.md` - CalibrationManager・UC2 シーケンス図
- `docs/architecture.md` - レイヤードアーキテクチャ
