# 設計書

## アーキテクチャ概要

FastAPI のレイヤードアーキテクチャに従い、Webレイヤーはアプリケーションレイヤー（RobotController）のみを呼び出す。ドメインロジック・DBへの直接アクセスは禁止。

```
ブラウザ
  │ HTTP / WebSocket
  ▼
src/web/app.py          ← FastAPI アプリ本体・lifespan
  ├── routers/drive.py  ← 走行制御エンドポイント
  ├── routers/profiles.py
  ├── routers/modes.py
  ├── routers/sessions.py
  ├── ws.py             ← WebSocket + ConnectionManager
  ├── deps.py           ← DI: RobotController をシングルトンで提供
  ├── schemas.py        ← Pydantic v2 リクエスト/レスポンスモデル
  └── static/           ← HTML/JS/CSS
        ├── index.html
        ├── js/app.js
        └── css/style.css
  │
  ▼
src/app/robot_controller.py  ← アプリケーションレイヤー（既存）
```

## コンポーネント設計

### 1. `src/web/app.py`

**責務**:
- FastAPI インスタンスの生成とルーター登録
- lifespan で RobotController を初期化しシングルトンとして保持
- static ファイルのマウント
- WebSocket ブロードキャストタスクの起動

**実装の要点**:
- `asynccontextmanager` でライフサイクル管理
- `StaticFiles` を `/static` にマウント
- `app.state.controller` に RobotController を格納
- WebSocket ブロードキャストは `asyncio.create_task()` で起動

### 2. `src/web/deps.py`

**責務**:
- `get_controller()` で `app.state.controller` を返す FastAPI Depends 関数

### 3. `src/web/schemas.py`

**主要モデル**:
- `SystemStateResponse`: robot_state, active_profile_id, active_session_id, updated_at
- `StartDriveRequest`: mode_id
- `DriveSessionResponse`: id, profile_id, mode_id, run_type, started_at, status
- `RealtimeData`: timestamp, robot_state, actual_speed_kmh, ref_speed_kmh, accel_opening, brake_opening, accel_current_ma, brake_current_ma

### 4. `src/web/routers/drive.py`

**エンドポイント**:
- `POST /api/v1/drive/initialize` → `controller.initialize()`
- `POST /api/v1/drive/start` → `controller.start_auto_drive(mode_id)`
- `POST /api/v1/drive/stop` → `controller.stop()`
- `GET /api/v1/drive/status` → `controller.get_system_state()`
- `POST /api/v1/drive/emergency` → `controller.emergency_stop()`
- `POST /api/v1/drive/reset-emergency` → `controller.reset_emergency()`
- `POST /api/v1/drive/manual/start` → `controller.start_manual()`
- `POST /api/v1/drive/manual/stop` → `controller.stop_manual()`

**エラーハンドリング**:
- `InvalidStateTransition` → HTTP 409 Conflict
- `PreCheckFailed` → HTTP 422 Unprocessable Entity

### 5. `src/web/routers/profiles.py`, `modes.py`, `sessions.py`

スタブ実装。インメモリの空リスト/404 を返す。DB 連携は別フェーズ。

### 6. `src/web/ws.py`

**ConnectionManager**:
- `connect(ws)`: accept して内部リストに追加
- `disconnect(ws)`: リストから除去
- `broadcast(data)`: 全クライアントへ送信、送信失敗クライアントは除去

**broadcast_loop(app)**:
- `asyncio.sleep(0.1)` で 100ms 周期
- `controller.get_system_state()` → `RealtimeData` → `manager.broadcast(json)`

### 7. `src/web/static/`

- `index.html`: メインダッシュボード（CDN なし、vanilla JS）
- `js/app.js`: WebSocket 接続・DOM 更新・REST API 呼び出し
- `css/style.css`: 最小限のスタイル

## データフロー

### WebSocket ブロードキャスト
```
1. lifespan で broadcast_loop タスク起動
2. 100ms ごとに controller.get_system_state() 呼び出し
3. RealtimeData JSON を全 WS クライアントへ broadcast
4. クライアント JS が受信 → DOM 更新
```

### REST API 呼び出し
```
1. ボタンクリック → fetch(POST /api/v1/drive/...)
2. ルーター関数が controller メソッドを await
3. JSON レスポンス返却 → UI 更新
```

## エラーハンドリング戦略

既存の `InvalidStateTransition`, `PreCheckFailed` を HTTP エラーにマッピング:
- `InvalidStateTransition` → 409
- `PreCheckFailed` → 422
- その他例外 → 500

## テスト戦略

### ユニットテスト (`tests/unit/test_web_drive.py`)
- `httpx.AsyncClient` + `ASGITransport` で各エンドポイントをテスト
- RobotController をモック化（Protocol に合わせた MagicMock）
- 正常系: 状態遷移成功 → 200/201
- 異常系: InvalidStateTransition → 409

### 統合テスト (`tests/integration/test_web_api.py`)
- FastAPI アプリ全体（モック Controller）でエンドポイント疎通確認
- WebSocket 接続・メッセージ受信テスト

## 依存ライブラリ追加

```toml
[project.optional-dependencies]
dev = [
    "httpx>=0.25.0",  # FastAPI テスト用
]
```

## ディレクトリ構造

```
src/web/
├── __init__.py
├── app.py
├── deps.py
├── schemas.py
├── ws.py
├── routers/
│   ├── __init__.py
│   ├── drive.py
│   ├── profiles.py
│   ├── modes.py
│   └── sessions.py
└── static/
    ├── index.html
    ├── js/app.js
    └── css/style.css

tests/unit/test_web_drive.py
tests/integration/test_web_api.py
```

## 実装の順序

1. 基盤: `__init__.py`, `deps.py`, `schemas.py`
2. ルーター: `routers/drive.py`（最重要）
3. WebSocket: `ws.py`
4. アプリ統合: `app.py`
5. スタブルーター: `profiles.py`, `modes.py`, `sessions.py`
6. UI: `static/`
7. テスト: `test_web_drive.py`, `test_web_api.py`
8. 依存追加: `pyproject.toml`

## セキュリティ考慮事項

- ローカルLAN限定（architecture.md 方針通り）
- Pydantic v2 による自動バリデーション
- WebSocket 接続数の上限は設けない（ローカル運用のため）

## パフォーマンス考慮事項

- WebSocket ブロードキャストは `asyncio.sleep(0.1)` で 100ms 周期
- 制御ループ（50ms）と干渉しないよう完全非同期
- broadcast 中の送信エラーは catch して接続除去（安定性優先）
