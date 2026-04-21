# 要求内容

## 機能名
インフラ層 DBスキーマ作成 ArchiveManager 設定管理

## 概要
インフラ層の未実装コンポーネントを実装する。

1. **DBスキーマ確認**: `scripts/setup_db.py` の DDL を確認し、不足があれば補完する
2. **ArchiveManager 実装**: `src/infra/archive_manager.py` を実装する
3. **設定管理実装**: `config/settings.toml.example` と `src/infra/settings.py` を実装する

## 現状

### 実装済み
- `scripts/setup_db.py`: DDL（5テーブル + 3インデックス）完成済み
- `src/infra/db.py`: 接続プール作成（最小実装）
- `src/infra/log_writer.py`: LogWriter 完成済み

### 未実装
- `src/infra/archive_manager.py`: ArchiveManager
- `config/settings.toml.example`: 設定テンプレート
- `src/infra/settings.py`: settings.toml 読み込みモジュール

## 要件

### ArchiveManager
- 内蔵SSD使用率が80%超の場合、3ヶ月超のセッションをCSV + gzip で USB SSD へ移行
- USB SSD が 80% 超の場合、最古アーカイブから削除
- 実行タイミング: 起動時・走行終了時（定期実行なし、容量トリガー）
- asyncpg.Connection を外部注入（LogWriter と同じパターン）

### settings.toml.example
- serial / can / database / gpio / archive / control セクション

### settings.py
- tomllib で config/settings.toml を読み込み
- dataclass で設定値を保持
- ファイル不在時は FileNotFoundError を raise
