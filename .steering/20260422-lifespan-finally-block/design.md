# 設計書

## アーキテクチャ概要

`RobotController` にシャットダウン専用メソッドを追加し、`lifespan` の `finally` から呼び出す。

```
lifespan finally ブロック
  ├── task.cancel() + await task  （既存）
  └── await controller.shutdown()  （追加）
        ├── _drive_loop.stop() if _drive_loop is not None
        └── await _safety_monitor.stop_monitoring()
```

## コンポーネント設計

### 1. `RobotController.shutdown()`

**責務**:
- どの状態からでも安全にリソースを解放する
- 状態遷移は行わない（状態機械に影響を与えない）

**実装の要点**:
- `stop()` と異なり、状態チェックなし
- DriveLoop を止める（`_drive_loop.stop()` + `_drive_loop = None`）
- `await self._safety_monitor.stop_monitoring()` を呼ぶ
- 例外を suppress しない（呼び出し元の `lifespan` で処理）

### 2. `lifespan` finally ブロック

**実装の要点**:
- タスクキャンセル後に `await controller.shutdown()` を追加
- shutdown の例外がサーバー終了を妨げないよう try/except で保護

## データフロー

### サーバーシャットダウン時
```
1. SIGTERM/Ctrl+C → FastAPI lifespan finally 実行
2. task.cancel() + await task（broadcast_loop を停止）
3. controller.shutdown() 呼び出し
4. DriveLoop を停止（実行中の場合）
5. safety_monitor.stop_monitoring() を呼び出し
```

## エラーハンドリング戦略

- `controller.shutdown()` 内では例外を伝播させる（呼び出し元に委ねる）
- `lifespan` の `finally` では shutdown の例外を `except Exception` で捕捉してログ出力し、サーバー終了をブロックしない

## テスト戦略

### ユニットテスト (`tests/unit/test_robot_controller.py`)
- BOOTING 状態で `shutdown()` を呼んでも安全に完了する
- STANDBY 状態で `shutdown()` を呼ぶと `stop_monitoring` が呼ばれる
- RUNNING 状態（DriveLoop あり）で `shutdown()` を呼ぶと DriveLoop が停止する
- READY 状態で `shutdown()` を呼ぶと `stop_monitoring` が呼ばれる

## ディレクトリ構造

```
変更ファイル:
  src/app/robot_controller.py  - shutdown() メソッドを追加
  src/web/app.py               - finally ブロックを更新
  tests/unit/test_robot_controller.py  - テストを追加
```

## 実装の順序

1. `RobotController.shutdown()` を実装
2. `lifespan` finally ブロックを更新
3. テストを追加
4. linter / type check を実行
