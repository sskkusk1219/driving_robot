# タスクリスト

## インフラ層 DBスキーマ作成 ArchiveManager 設定管理

---

## 申し送り事項

**実装完了日**: 2026-04-21

### 計画と実績の差分
- 計画どおり 6 タスクをすべて完了
- `tests/unit/infra/` ディレクトリを新設（repository-structure.md に記載のパスと一致）
- ruff の E501 （日本語 docstring の行長超過）で修正が必要だった。日本語文字は幅2でカウントされるため、日本語 docstring は短めに書くこと

### 学んだこと
- ruff の line-length は文字コードポイント数ではなく文字幅ベースで判定するため、日本語は実質100文字制限よりも短い
- `asyncpg.Record` を mock する際、`__iter__` と `__getitem__` を両方設定する必要がある

### 次回への改善提案
- `ArchiveManager` の `_get_pg_data_path()` は PostgreSQL の `data_directory` を使用しているが、実際の内蔵SSD使用率は PostgreSQL データディレクトリの親パーティションで測定すべきかもしれない。実機確認が必要
- `check_and_archive()` の実行タイミング（起動時・走行終了時）は `RobotController` 側で組み込む必要がある
- `settings.toml` が存在しない場合の fallback（デフォルト値での起動）は現在非対応。要件に応じて検討

- [x] T1: `src/infra/settings.py` を実装する（tomllib + dataclass）
- [x] T2: `config/settings.toml.example` を作成する
- [x] T3: `src/infra/archive_manager.py` を実装する（ArchiveManager）
- [x] T4: `tests/unit/infra/test_archive_manager.py` を実装する（ユニットテスト）
- [x] T5: `src/infra/__init__.py` を更新し、新モジュールをエクスポートする
- [x] T6: `scripts/setup_db.py` を確認し、不足 DDL があれば補完する（5テーブル+3インデックス完成済み・追加不要）
