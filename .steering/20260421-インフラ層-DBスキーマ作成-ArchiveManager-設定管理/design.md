# 実装アプローチ

## 方針

既存パターン（LogWriter）に倣い、以下を実装する。

---

## 1. settings.py (`src/infra/settings.py`)

- Python 3.11+ 標準の `tomllib` を使用（追加依存なし）
- dataclass でセクションごとにネスト
- `load_settings(path)` 関数でファイルを読み込み `AppSettings` を返す

```
@dataclass SerialSettings / CanSettings / DatabaseSettings /
GpioSettings / ArchiveSettings / ControlSettings / AppSettings

load_settings(path: Path) -> AppSettings
```

---

## 2. archive_manager.py (`src/infra/archive_manager.py`)

- asyncpg.Connection を外部注入（LogWriter と同じパターン）
- `check_and_archive()` が公開 API

```
ArchiveManager.__init__(conn, settings)
check_and_archive()          # 使用率チェック → 必要なら _archive_old_sessions()
_archive_old_sessions()      # 3ヶ月超セッションを CSV+gz → DB 削除
_export_to_csv(...)          # drive_logs を CSV 書き込み
_compress(csv_path)          # gzip 圧縮、元 CSV 削除
_check_storage_usage(path)   # shutil.disk_usage で使用率[%]
_cleanup_usb_ssd_if_needed() # USB SSD 80%超なら最古削除
_delete_session_from_db()    # drive_logs → drive_sessions の順に DELETE
```

ファイル命名: `{session_id}_{started_at:%Y%m%d_%H%M%S}.csv.gz`

---

## 3. settings.toml.example (`config/settings.toml.example`)

repository-structure.md の設定構造どおりに作成。
