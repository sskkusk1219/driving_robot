# 設計書

## アーキテクチャ概要

既存のレイヤードアーキテクチャを踏襲しつつ、4領域に変更を加える。

```
Web Layer
  ├── drive.py  ← /learning/start エンドポイント追加
  └── ws.py     ← broadcast_loop で DriveLoop 開度を取得

App Layer
  └── robot_controller.py  ← start_learning_drive() 追加

Domain Layer
  └── control/drive_loop.py  ← current_accel_opening / current_brake_opening プロパティ追加

Frontend
  ├── index.html  ← 学習運転ボタン追加
  └── app.js      ← ボタンハンドラ追加
```

## コンポーネント設計

### 1. DriveLoop - 開度プロパティ追加

- `_current_accel_opening: float = 0.0` / `_current_brake_opening: float = 0.0` をインスタンス変数に追加
- `_execute_one_cycle()` で開度算出直後に更新
- `@property current_accel_opening` / `current_brake_opening` を公開
- 停止中は 0.0 を返す（デフォルト値）

### 2. RobotController - start_learning_drive()

- `start_auto_drive()` の実装パターンを踏襲
- READY → PRE_CHECK → RUNNING 遷移
- DriveSession(run_type='learning') を返す
- コンストラクタに `learning_manager: LearningDriveManagerProtocol | None = None` を追加（将来の拡張用）

### 3. drive.py - /learning/start

- Body なし
- 成功時: DriveSessionResponse を返す
- InvalidStateTransition → 409
- PreCheckFailed → 422

### 4. broadcast_loop - 開度取得修正

- `controller._drive_loop` が None でない場合のみ開度を取得
- `None` の場合は 0.0（現在と同じ）

### 5. フロントエンド

- index.html: ボタングリッドに「学習運転 開始」ボタンを追加
- app.js: `/api/v1/drive/learning/start` を呼び出すハンドラを追加

## データフロー

### 学習走行開始
```
ブラウザ → POST /api/v1/drive/learning/start
→ controller.start_learning_drive()
→ READY → PRE_CHECK → RUNNING
→ DriveSession(run_type='learning')
→ 200 DriveSessionResponse
```

### WebSocket 開度配信
```
broadcast_loop (100ms)
→ controller._drive_loop.current_accel_opening
→ RealtimeData.accel_opening に設定
→ WebSocket 配信
```

## テスト戦略

### ユニットテスト追加

1. `test_drive_loop.py` - 開度プロパティ初期値・更新後の値
2. `test_robot_controller.py` - start_learning_drive() 状態遷移・PreCheckFailed・返り値
3. `test_web_drive.py` - POST /learning/start 200/409/422

## 実装の順序

1. DriveLoop 開度プロパティ追加（+ テスト）
2. RobotController start_learning_drive() 追加（+ テスト）
3. drive.py /learning/start エンドポイント追加（+ テスト）
4. broadcast_loop 開度取得修正
5. フロントエンド 学習走行ボタン追加
6. 品質チェック（pytest / ruff / mypy）
