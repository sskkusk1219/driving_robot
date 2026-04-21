# tasklist.md — LogWriter 実装

## フェーズ1: インフラディレクトリのセットアップ

- [x] `src/infra/__init__.py` を作成する
- [x] `src/infra/db.py` スタブを作成する（`create_pool` / `get_connection` の型スタブのみ）

## フェーズ2: LogWriter 本体の実装

- [x] `src/infra/log_writer.py` に `LogWriter` クラスを実装する
  - [x] `__init__(self, conn: asyncpg.Connection) -> None`
  - [x] `start_session(self, profile_id, mode_id, run_type) -> str`
  - [x] `write_log(self, session_id, data: DriveLogData) -> None`
  - [x] `end_session(self, session_id, status) -> None`

## フェーズ3: ユニットテストの実装

- [x] `tests/unit/test_log_writer.py` を作成する
  - [x] `start_session` が UUID を返し、正しい SQL を呼ぶことをテスト
  - [x] `write_log` が正しい引数で `conn.execute` を呼ぶことをテスト
  - [x] `end_session` が正しい引数で `conn.execute` を呼ぶことをテスト

## フェーズ4: 統合テストの実装

- [x] `tests/integration/__init__.py` を作成する
- [x] `tests/integration/test_log_writer_db.py` を作成する（`pytest.mark.integration` で CI 分離）

## フェーズ5: 品質チェック

- [x] `ruff check src/ tests/` がエラーなし
- [x] `mypy src/` がエラーなし
- [x] `pytest tests/unit/` がパス

---

## 実装後の振り返り

**実装完了日**: 2026-04-21

### 計画と実績の差分

- 計画通りに完了。スコープ外としていた `db.py` も最小スタブとして作成し、`create_pool` のみを提供した。
- asyncpg が venv 未インストールだったため、`pip install -e ".[dev]" asyncpg` を実施した（pyproject.toml に asyncpg は依存として記載済みだが未インストールだった）。

### 学んだこと

1. **mypy strict + asyncpg**: asyncpg が未インストールの場合、`asyncpg.Connection` は mypy に `Any` として認識されるため `type: ignore` コメントが不要（むしろ `unused-ignore` エラーになる）。
2. **ruff UP043**: Python 3.13 では `AsyncGenerator[X, None]` の `None` は省略可能なデフォルト引数扱いとなり、ruff が警告する。`AsyncGenerator[X]` に変更する必要がある。
3. **ローカルインポートは原則 NG**: テストメソッド内の `import uuid` は ruff が検出しないが、コーディング規約上ファイル先頭に移動すべき。

### 次回への改善提案

1. `RunType` / `EndStatus` を `Literal` 型として `src/models/drive_log.py` に定義し、`LogWriter` シグネチャに適用すると仕様外の値を静的に検出できる。
2. `write_log` の DB エラー時の振る舞い（伝播のみ）をユニットテストで明示すると仕様変更時のリグレッションを防止できる。
