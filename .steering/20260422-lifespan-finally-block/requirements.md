# 要求内容

## 概要

`src/web/app.py` の `lifespan` コンテキストマネージャの `finally` ブロックに、`RobotController` のグレースフルシャットダウン処理を追加する。

## 背景

現在の `finally` ブロックはブロードキャストタスクのキャンセルのみを行っており、`controller.start()` で起動したコンポーネント（安全監視、ハードウェア接続）が正常にクリーンアップされない。サーバー停止時にリソースリークや安全監視の継続が発生する可能性がある。

## 実装対象の機能

### 1. `RobotController.shutdown()` メソッド

- どの状態からでも呼び出せるグレースフルシャットダウンメソッド
- DriveLoop が実行中なら停止する
- SafetyMonitor の監視を停止する
- 状態遷移エラーを発生させない

### 2. `lifespan` finally ブロックの更新

- `await controller.shutdown()` を呼び出す
- 既存のタスクキャンセル処理の後に追加

## 受け入れ条件

### `RobotController.shutdown()`
- [ ] BOOTING/STANDBY/READY など任意の状態から呼び出せる
- [ ] DriveLoop が実行中の場合は停止する
- [ ] `safety_monitor.stop_monitoring()` を必ず呼ぶ
- [ ] 例外が発生しても安全に終了する

### `lifespan` finally ブロック
- [ ] タスクキャンセル後に `await controller.shutdown()` を呼ぶ
- [ ] shutdown 中の例外がサーバー終了を妨げない

## スコープ外

- ハードウェアドライバの disconnect（プロトコルに未定義）
- 状態を SHUTDOWN などの新しい状態に遷移させること

## 参照ドキュメント

- `docs/functional-design.md` - 機能設計書
- `src/app/robot_controller.py` - RobotController 実装
- `src/web/app.py` - FastAPI lifespan
