# 設計書

## アーキテクチャ概要

レイヤードアーキテクチャのアプリケーション層（factory.py）で DB 接続プールを確立し、インフラ層の `ProfileRepository` を生成してドメイン層の `CalibrationManager` に注入する。

```
app.py (lifespan)
  └─ await _build_controller(settings)     # async に変更
        └─ await build_real_controller(settings)  # async に変更
              ├─ pool = await create_pool(dsn)
              ├─ profile_repo = ProfileRepository(pool)
              └─ CalibrationManager(..., profile_repo=profile_repo)
```

## コンポーネント設計

### 1. ProfileRepository（変更）

**責務**:
- `asyncpg.Pool` を保持し、`save_calibration` ごとに接続を acquire/release する

**変更内容**:
- `__init__` の引数を `conn: asyncpg.Connection` → `pool: asyncpg.Pool` に変更
- `save_calibration` 内で `async with self._pool.acquire() as conn:` パターンを使用

**理由**: 単一接続をアプリ起動中ずっと保持するのは接続タイムアウトのリスクがある。プールから都度 acquire することで信頼性が高まる。

### 2. build_real_controller（変更）

**責務**:
- 実ハードウェアコンポーネントと DB 接続を組み合わせて `RobotController` を生成する

**変更内容**:
- `def` → `async def` に変更
- `from src.infra.db import create_pool` のインポートを追加
- `from src.infra.profile_repository import ProfileRepository` のインポートを追加
- `pool = await create_pool(settings.database.dsn)` を追加
- `CalibrationManager` に `profile_repo=ProfileRepository(pool)` を渡す

### 3. app.py の _build_controller（変更）

**変更内容**:
- `def _build_controller()` → `async def _build_controller()` に変更
- `lifespan` 内で `controller = await _build_controller()` に変更

## データフロー

### キャリブレーション実行時
```
1. Web API → controller.run_calibration()
2. CalibrationManager.run_calibration(profile_id)
3. ゼロ/フル検出 → CalibrationData 生成
4. バリデーション成功時 → profile_repo.save_calibration(profile_id, data)
5. ProfileRepository.save_calibration が pool.acquire() → INSERT/UPSERT
6. 接続を pool に返却
```

## エラーハンドリング戦略

- `create_pool` の失敗: RuntimeError が伝播し、アプリ起動失敗として扱われる（既存の `db.py` の動作を踏襲）
- `save_calibration` の DB エラー: `CalibrationManager.run_calibration` から呼び出し元に伝播（既存テストで確認済み）

## テスト戦略

### 既存テストへの影響

`test_factory.py` の既存テストはすべて同期テスト。`build_real_controller` が `async def` になるため、以下の対応が必要:
- 既存テストを `@pytest.mark.asyncio` 付きの `async def` に変更
- または `asyncio.run()` でラップ

→ **`@pytest.mark.asyncio` に変換する**（プロジェクトの既存パターン）

### 追加テスト

- `test_profile_repository_injected_into_calibration_manager`: pool モックを使い、factory が CalibrationManager に ProfileRepository を渡すことを確認
- `test_build_real_controller_creates_pool_with_dsn`: `create_pool` が `settings.database.dsn` で呼ばれることを確認

## 依存ライブラリ

追加なし（asyncpg は既存依存）

## ディレクトリ構造

変更対象ファイル:
```
src/infra/profile_repository.py   # Pool 対応
src/app/factory.py                # async 化 + pool + ProfileRepository 注入
src/web/app.py                    # _build_controller async 化
tests/unit/test_factory.py        # async テスト対応 + 追加テスト
```

## 実装の順序

1. `ProfileRepository` を `asyncpg.Pool` 対応に変更
2. `factory.py` を async 化し Pool + ProfileRepository を注入
3. `app.py` を async 対応に変更
4. `test_factory.py` を async テスト対応 + 新テスト追加
5. テスト・lint・型チェック実行
