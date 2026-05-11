# 要求内容

## 概要

`factory.py` の `build_real_controller` で DB 接続プールを確立し、`ProfileRepository` を生成して `CalibrationManager` に注入する。

## 背景

`CalibrationManager` は `ProfileRepositoryProtocol` を受け取ることでキャリブレーション結果を PostgreSQL に永続化できる設計になっているが、実際のコントローラー生成時（`build_real_controller`）に `ProfileRepository` が注入されていないため、DB 永続化が機能していない。

## 実装対象の機能

### 1. ProfileRepository の DB 接続切替（接続プール対応）

- `ProfileRepository` が `asyncpg.Connection` ではなく `asyncpg.Pool` を受け取るよう変更
- `save_calibration` の呼び出しごとにプールから接続を acquire/release する

### 2. factory.py での DB 接続確立と注入

- `build_real_controller` を `async def` に変更
- `create_pool(settings.database.dsn)` で接続プールを生成
- `ProfileRepository(pool)` を生成して `CalibrationManager` の `profile_repo` に注入

### 3. app.py の非同期対応

- `_build_controller()` を `async def` に変更し、`lifespan` から `await` で呼び出す

### 4. テスト追加・更新

- `test_factory.py` に DB 注入のテストを追加（`pytest.mark.asyncio` + pool モック）

## 受け入れ条件

### ProfileRepository
- [ ] `asyncpg.Pool` を受け取り、`save_calibration` 内で `acquire()` する
- [ ] `ProfileRepositoryProtocol` を満たし続ける

### build_real_controller
- [ ] `async def` として定義されている
- [ ] `settings.database.dsn` から接続プールを生成する
- [ ] `ProfileRepository(pool)` を `CalibrationManager` に渡す

### app.py
- [ ] `_build_controller` が `async def` に変更されている
- [ ] `lifespan` 内で `await _build_controller()` が呼ばれる

### テスト
- [ ] 既存テストが全てパスする
- [ ] DB 注入のテストが追加されている

## スコープ外

- 接続プールのライフサイクル管理（クローズ処理）
- リトライ・フェイルオーバー設計
- DB マイグレーション

## 参照ドキュメント

- `docs/functional-design.md` - 機能設計書
- `docs/architecture.md` - アーキテクチャ設計書
