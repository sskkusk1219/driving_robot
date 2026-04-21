# requirements.md — LogWriter 実装

## 概要

`LogWriter` は走行ログを 100ms 周期で PostgreSQL に書き込むインフラレイヤーのコンポーネント。
`functional-design.md` のコンポーネント設計に基づき、`src/infra/log_writer.py` として実装する。

## 要求仕様

### 機能要求

| # | 要求 | 出典 |
|---|------|------|
| F1 | `start_session(profile_id, mode_id, run_type)` → `session_id: str` (UUID) を `drive_sessions` に INSERT | functional-design.md |
| F2 | `write_log(session_id, data: DriveLogData)` → `drive_logs` に INSERT（timestamp は DB側 NOW()） | functional-design.md |
| F3 | `end_session(session_id, status)` → `drive_sessions.ended_at = NOW(), status = status` に UPDATE | functional-design.md |
| F4 | asyncpg の非同期接続を使用（`asyncpg.Connection` または `asyncpg.Pool`） | architecture.md |

### 非機能要求

| # | 要求 | 出典 |
|---|------|------|
| NF1 | write_log の DB INSERT は 5ms 以内に完了すること（architecture.md のパフォーマンス要件） | architecture.md |
| NF2 | 型注釈を全メソッドに付与（mypy strict 対応） | development-guidelines.md |
| NF3 | ユニットテスト（asyncpg をモック化）と統合テスト（テスト用 DB への実書き込み）を作成 | development-guidelines.md |

## スコープ外

- `src/infra/db.py`: 接続プール管理。LogWriter は `asyncpg.Connection` を受け取る設計とし、接続管理は呼び出し元が行う。今回は db.py のスタブのみ作成。
- ArchiveManager: 別機能として今回のスコープ外。

## 受け入れ条件

1. `LogWriter` が `asyncpg.Connection` を受け取るコンストラクタで初期化できる
2. `start_session` が UUID 文字列を返し、`drive_sessions` テーブルに行を INSERT する
3. `write_log` が `DriveLogData` を `drive_logs` テーブルに INSERT する（timestamp は `NOW()`）
4. `end_session` が `drive_sessions.ended_at` と `status` を UPDATE する
5. `pytest tests/unit/` がパスする（asyncpg モック使用）
6. `ruff check src/ tests/` がエラーなし
7. `mypy src/` がエラーなし
