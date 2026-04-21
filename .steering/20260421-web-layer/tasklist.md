# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: 基盤実装

- [x] `src/web/__init__.py` を作成（空ファイル）
- [x] `src/web/deps.py` を作成（get_controller 依存注入関数）
- [x] `src/web/schemas.py` を作成（Pydantic v2 モデル定義）
  - [x] `SystemStateResponse`
  - [x] `StartDriveRequest`
  - [x] `DriveSessionResponse`
  - [x] `RealtimeData`
  - [x] `ErrorResponse`
- [x] `src/web/routers/__init__.py` を作成（空ファイル）

## フェーズ2: 走行制御ルーター

- [x] `src/web/routers/drive.py` を作成
  - [x] `POST /api/v1/drive/initialize`
  - [x] `POST /api/v1/drive/start`
  - [x] `POST /api/v1/drive/stop`
  - [x] `GET /api/v1/drive/status`
  - [x] `POST /api/v1/drive/emergency`
  - [x] `POST /api/v1/drive/reset-emergency`
  - [x] `POST /api/v1/drive/manual/start`
  - [x] `POST /api/v1/drive/manual/stop`
  - [x] `InvalidStateTransition` → HTTP 409 のエラーハンドリング
  - [x] `PreCheckFailed` → HTTP 422 のエラーハンドリング

## フェーズ3: WebSocket リアルタイム配信

- [x] `src/web/ws.py` を作成
  - [x] `ConnectionManager` クラス（connect/disconnect/broadcast）
  - [x] `WS /ws/realtime` エンドポイント
  - [x] `broadcast_loop()` コルーチン（100ms 周期）

## フェーズ4: FastAPI アプリ統合

- [x] `src/web/app.py` を作成
  - [x] lifespan で RobotController 初期化・broadcast_loop 起動
  - [x] 全ルーターの登録
  - [x] `/static` の StaticFiles マウント
  - [x] `/` → index.html リダイレクト

## フェーズ5: スタブルーター

- [x] `src/web/routers/profiles.py` を作成
  - [x] `GET /api/v1/profiles/` → 空リスト
  - [x] `GET /api/v1/profiles/{profile_id}` → 404
  - [x] `POST /api/v1/profiles/` → 501 Not Implemented
  - [x] `PUT /api/v1/profiles/{profile_id}` → 501
  - [x] `DELETE /api/v1/profiles/{profile_id}` → 501
- [x] `src/web/routers/modes.py` を作成
  - [x] `GET /api/v1/modes/` → 空リスト
  - [x] `GET /api/v1/modes/{mode_id}` → 404
  - [x] `POST /api/v1/modes/` → 501
- [x] `src/web/routers/sessions.py` を作成
  - [x] `GET /api/v1/sessions/` → 空リスト
  - [x] `GET /api/v1/sessions/{session_id}` → 404
  - [x] `GET /api/v1/sessions/{session_id}/logs` → 空リスト

## フェーズ6: ダッシュボード UI

- [x] `src/web/static/css/style.css` を作成（最小限スタイル）
- [x] `src/web/static/js/app.js` を作成
  - [x] WebSocket 接続 (`/ws/realtime`)
  - [x] 受信データで DOM 更新（状態バッジ・速度・開度ゲージ）
  - [x] 簡易ラインチャート（Canvas API）
  - [x] 操作ボタンの fetch API 呼び出し
- [x] `src/web/static/index.html` を作成
  - [x] システム状態バッジ
  - [x] 実車速・基準車速表示
  - [x] アクセル・ブレーキ開度ゲージ
  - [x] リアルタイムグラフ領域
  - [x] 操作ボタン群

## フェーズ7: テスト

- [x] `tests/unit/test_web_drive.py` を作成
  - [x] `GET /api/v1/drive/status` 正常系テスト
  - [x] `POST /api/v1/drive/initialize` 正常系テスト
  - [x] `POST /api/v1/drive/start` 正常系テスト
  - [x] `POST /api/v1/drive/emergency` 正常系テスト
  - [x] `POST /api/v1/drive/stop` - 不正状態で 409 返却テスト
  - [x] `POST /api/v1/drive/manual/start` 正常系テスト
  - [x] `POST /api/v1/drive/manual/stop` 正常系テスト
- [x] `tests/integration/test_web_api.py` を作成
  - [x] FastAPI アプリ全体の疎通テスト
  - [x] WebSocket 接続・受信テスト

## フェーズ8: 品質チェックと修正

- [x] `pyproject.toml` の dev 依存に `httpx>=0.25.0` を追加
- [x] `.venv` に httpx をインストール
- [x] すべてのテストが通ることを確認
  - [x] `python -m pytest tests/unit/test_web_drive.py -v` (10 passed)
  - [x] `python -m pytest tests/integration/test_web_api.py -v` (8 passed)
- [x] リントエラーがないことを確認
  - [x] `python -m ruff check src/web/ ...` (All checks passed)
- [x] 型エラーがないことを確認
  - [x] `python -m mypy src/web/` (Success: no issues found in 10 source files)

## フェーズ9: ドキュメント更新

- [x] `docs/functional-design.md` の REST API 設計セクションを実装済みに更新
- [x] 実装後の振り返り（このファイルの下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-04-21

### 計画と実績の差分

**計画と異なった点**:
- implementation-validator の指摘を受けて `src/app/stubs.py` を追加した（当初は `app.py` に埋め込む予定だったがアーキテクチャ違反のため分離）
- `schemas.py` の `robot_state` 型を `str` → `RobotState`（StrEnum）に変更し OpenAPI 仕様を強化
- `ConnectionManager` に `has_connections` プロパティを追加（内部リストの直接アクセスをカプセル化）
- テストを 18件→20件に追加（PreCheckFailed 422 レスポンスのカバレッジ追加）

**新たに必要になったタスク**:
- `src/app/stubs.py` の作成（validator 指摘による分離）
- `ws.py` への logging 追加（`except Exception` の握りつぶし解消）
- PreCheckFailed 422 テスト2件の追加

**技術的理由でスキップしたタスク**（該当する場合のみ）:
- なし（全タスク完了）

### 学んだこと

**技術的な学び**:
- FastAPI の `ASGITransport` + `httpx.AsyncClient` による非同期テストパターンが既存テストパターン（MagicMock + AsyncMock）とよく合う
- `StrEnum` を Pydantic v2 モデルのフィールド型に使うと OpenAPI の enum 値が自動生成される
- `app.state` を使った依存注入パターンはテスト時に `app.state.controller = stub` で簡単に差し替えられる

**プロセス上の改善点**:
- implementation-validator を ステップ6 で実行したことで、アーキテクチャ違反（ドメイン層直接依存）を早期に発見・修正できた
- steering スキルによるタスクリアルタイム更新がフェーズ全体の見通しを良くした

### 次回への改善提案
- 走行ログ・プロファイル・モードの DB 連携実装時は `src/web/routers/profiles.py` 等のスタブを置き換える
- RobotController に `get_realtime_data()` メソッドを追加して WebSocket 配信の実データ化を行う
- WebSocket の `ConnectionManager.broadcast` ロジックのユニットテスト追加を推奨（切断処理のバグが混入しやすい）
- `pytest-cov` を dev 依存に追加してカバレッジ計測を CI に組み込む
