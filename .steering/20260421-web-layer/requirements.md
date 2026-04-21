# 要求内容

## 概要

FastAPI を用いた Webレイヤーを実装する。全 REST API エンドポイント、WebSocket リアルタイム配信、ダッシュボード UI を提供し、オペレーターがブラウザから走行ロボットを操作・監視できるようにする。

## 背景

既存の `RobotController`（アプリケーションレイヤー）は実装済みだが、Webレイヤー（`src/web/`）が未実装のため、ブラウザからの操作ができない。本作業でWebレイヤーを完成させ、システムを操作可能な状態にする。

## 実装対象の機能

### 1. FastAPI アプリケーション基盤
- `src/web/app.py`: FastAPI アプリ定義、lifespan、ルーター登録
- `src/web/deps.py`: RobotController の依存注入
- `src/web/schemas.py`: Pydantic v2 リクエスト/レスポンスモデル

### 2. REST API エンドポイント（全ルーター）
- **走行制御** (`/api/v1/drive/`):
  - `POST /initialize` - 初期化（アラームリセット・サーボON・原点復帰）
  - `POST /start` - 自動走行開始（mode_id 指定）
  - `POST /stop` - 正常停止
  - `GET /status` - システム状態取得
  - `POST /emergency` - 緊急停止
  - `POST /reset-emergency` - 非常停止リセット
  - `POST /manual/start` - 手動操作開始
  - `POST /manual/stop` - 手動操作終了
- **プロファイル管理** (`/api/v1/profiles/`):
  - `GET /` - 一覧取得（スタブ）
  - `GET /{profile_id}` - 個別取得（スタブ）
  - `POST /` - 新規作成（スタブ）
  - `PUT /{profile_id}` - 更新（スタブ）
  - `DELETE /{profile_id}` - 削除（スタブ）
- **走行モード管理** (`/api/v1/modes/`):
  - `GET /` - 一覧取得（スタブ）
  - `GET /{mode_id}` - 個別取得（スタブ）
  - `POST /` - 新規作成（スタブ）
- **セッション・ログ** (`/api/v1/sessions/`):
  - `GET /` - セッション一覧（スタブ）
  - `GET /{session_id}` - セッション詳細（スタブ）
  - `GET /{session_id}/logs` - ログ一覧（スタブ）

### 3. WebSocket リアルタイム配信
- エンドポイント: `WS /ws/realtime`
- 100ms 周期でシステム状態・実車速・開度・電流値を全接続クライアントへブロードキャスト
- 複数クライアント接続対応（ConnectionManager）

### 4. ダッシュボード UI（HTML/JS/CSS）
- `src/web/static/index.html`: メインダッシュボード
  - システム状態バッジ
  - 実車速・基準車速の数値表示
  - アクセル・ブレーキ開度ゲージ
  - リアルタイムライングラフ
  - 操作ボタン群（初期化 / 自動走行 / 手動操作 / 停止 / 緊急停止）
- WebSocket で受信したデータをリアルタイム更新

## 受け入れ条件

### FastAPI 基盤
- [ ] `uvicorn src.web.app:app` で起動できる
- [ ] `/docs` で Swagger UI が表示される

### REST API
- [ ] `GET /api/v1/drive/status` が SystemState を JSON で返す
- [ ] `POST /api/v1/drive/initialize` が RobotController.initialize() を呼ぶ
- [ ] `POST /api/v1/drive/start` が RobotController.start_auto_drive() を呼ぶ
- [ ] `POST /api/v1/drive/emergency` が RobotController.emergency_stop() を呼ぶ
- [ ] プロファイル・モード・セッション API がスタブ応答を返す

### WebSocket
- [ ] `WS /ws/realtime` に接続できる
- [ ] 100ms 周期でデータを受信できる
- [ ] 複数クライアントが同時接続できる

### UI
- [ ] `http://localhost:8080/` でダッシュボードが表示される
- [ ] システム状態バッジが WebSocket データで更新される
- [ ] 各操作ボタンが対応 API を呼び出す

## 成功指標

- WebSocket 配信遅延 100ms 以内（architecture.md パフォーマンス要件）
- 全エンドポイントのユニットテストがパス
- `ruff check` と `mypy --strict` エラーなし

## スコープ外

以下はこのフェーズでは実装しません:

- PostgreSQL への実データ保存（プロファイル・モード・セッション API はスタブ）
- キャリブレーション API（CalibrationManager との連携）
- 学習運転 API（LearningDriveManager との連携）
- 走行モード CSV アップロード
- 手動操作スライダーの詳細 UI
- ログ詳細画面

## 参照ドキュメント

- `docs/product-requirements.md` - プロダクト要求定義書
- `docs/functional-design.md` - 機能設計書
- `docs/architecture.md` - アーキテクチャ設計書
