# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: ProfileRepository の Pool 対応

- [x] `src/infra/profile_repository.py` を `asyncpg.Pool` を受け取るよう変更
  - [x] `__init__` の引数を `conn: asyncpg.Connection` → `pool: asyncpg.Pool` に変更
  - [x] `save_calibration` 内を `async with self._pool.acquire() as conn:` パターンに変更

## フェーズ2: factory.py の async 化と注入

- [x] `src/app/factory.py` の `build_real_controller` を `async def` に変更
  - [x] `from src.infra.db import create_pool` インポート追加
  - [x] `from src.infra.profile_repository import ProfileRepository` インポート追加
  - [x] `pool = await create_pool(settings.database.dsn)` を追加
  - [x] `CalibrationManager` に `profile_repo=ProfileRepository(pool)` を渡す

## フェーズ3: app.py の非同期対応

- [x] `src/web/app.py` の `_build_controller` を `async def` に変更
  - [x] `controller = await _build_controller()` に変更

## フェーズ4: テスト更新と追加

- [x] `tests/unit/test_factory.py` の既存テストを `@pytest.mark.asyncio` 付きの `async def` に変換
- [x] `test_build_real_controller_creates_pool_with_dsn` テストを追加
- [x] `test_profile_repository_injected_into_calibration_manager` テストを追加

## フェーズ5: 品質チェック

- [x] テストが全てパスすることを確認
  - [x] `python -m pytest tests/unit/test_factory.py -v` (11/11 passed)
  - [x] `python -m pytest tests/ -v` (337/337 passed)
- [x] lint エラーがないことを確認
  - [x] `python -m ruff check src/ tests/` (All checks passed)
- [x] 型エラーがないことを確認
  - [x] `python -m mypy src/` (Success: no issues found in 41 source files)

---

## 実装後の振り返り

### 実装完了日
2026-04-22

### 計画と実績の差分

**計画と異なった点**:
- バリデーション後に `save_calibration` のエラーハンドリング（ログ追加）を実装した（設計書では未記載だったが validator の指摘で追加）
- `test_factory.py` の `_default_patches()` ヘルパー関数をデッドコードとして削除した

**新たに必要になったタスク**:
- `tests/unit/infra/test_profile_repository.py` の新設（validator の指摘でカバレッジ改善のため追加）
- `save_calibration` への `logger.exception` 追加（validator の指摘）

### 学んだこと

**技術的な学び**:
- `asyncpg.Pool` を `async with pool.acquire() as conn:` パターンで使うと接続リークがコンテキストマネージャで防止される
- `build_real_controller` を `async def` にする場合、呼び出し側（`app.py` の `_build_controller`）も同様に非同期化が必要

**プロセス上の改善点**:
- implementation-validator を実行することで設計時に見落としたプールクローズ問題やテスト不足が発見できた

### 次回への改善提案
- **プールクローズ処理**: `app.py` の `lifespan` の `finally` ブロックで `await pool.close()` を呼ぶ実装が必要（本タスクのスコープ外として残存）。pool を `app.state` に保持するか、factory の戻り値を `(controller, pool)` にする方針の検討が必要
- **infra 層の接続管理方針統一**: `LogWriter`・`ArchiveManager` は `asyncpg.Connection` 注入型、`ProfileRepository` は Pool 内包型と不統一。将来的にいずれかに統一する
