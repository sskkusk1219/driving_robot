# design.md — LogWriter 実装設計

## アーキテクチャ上の位置づけ

- **レイヤー**: インフラレイヤー (`src/infra/`)
- **ファイル**: `src/infra/log_writer.py`
- **依存先**: `asyncpg.Connection`（DI）、`src/models/drive_log.py`（DriveLogData）

## クラス設計

```python
class LogWriter:
    def __init__(self, conn: asyncpg.Connection) -> None: ...

    async def start_session(
        self,
        profile_id: str,
        mode_id: str | None,
        run_type: str,
    ) -> str:
        """drive_sessions に INSERT し、生成した UUID を返す。"""

    async def write_log(self, session_id: str, data: DriveLogData) -> None:
        """drive_logs に INSERT する。timestamp は DB 側 NOW() を使用。"""

    async def end_session(self, session_id: str, status: str) -> None:
        """drive_sessions.ended_at = NOW(), status を UPDATE する。"""
```

## SQL設計

### start_session

```sql
INSERT INTO drive_sessions
    (id, profile_id, mode_id, run_type, started_at, status)
VALUES
    ($1, $2, $3, $4, NOW(), 'running')
```

- `id` はアプリ側で `uuid.uuid4()` を生成して渡す
- `started_at` は DB 側 `NOW()` を使用

### write_log

```sql
INSERT INTO drive_logs
    (session_id, timestamp, ref_speed_kmh, actual_speed_kmh,
     accel_opening, brake_opening, accel_pos, brake_pos,
     accel_current, brake_current)
VALUES
    ($1, NOW(), $2, $3, $4, $5, $6, $7, $8, $9)
```

- `timestamp` は DB 側 `NOW()` を使用（100ms 周期での呼び出しを前提）

### end_session

```sql
UPDATE drive_sessions
SET ended_at = NOW(), status = $2
WHERE id = $1
```

## モジュール構成

```
src/infra/
├── __init__.py     (新規作成)
├── db.py           (スタブ: create_pool / get_connection のみ)
└── log_writer.py   (主実装)

tests/unit/
└── test_log_writer.py    (asyncpg をモック化したユニットテスト)

tests/integration/
├── __init__.py     (新規作成)
└── test_log_writer_db.py (実 DB への統合テスト、手動実行)
```

## テスト方針

### ユニットテスト (tests/unit/test_log_writer.py)

- `asyncpg.Connection` を `AsyncMock` で差し替える
- `conn.fetchval` / `conn.execute` の呼び出し引数を検証
- `start_session` が UUID 文字列を返すことを確認
- `write_log` / `end_session` が正しい SQL を呼ぶことを確認

### 統合テスト (tests/integration/test_log_writer_db.py)

- `pytest.mark.integration` マーカーで CI と分離
- テスト用 DB (`TEST_DATABASE_URL` 環境変数) に接続
- 実際に INSERT/UPDATE し、SELECT で検証
- `TRUNCATE drive_sessions, drive_logs CASCADE` で後片付け

## 実装上の注意点

- `run_type` の値は DB CHECK 制約 (`'auto' | 'manual' | 'learning'`) に準拠する（バリデーションは呼び出し元責任）
- `status` の値は DB CHECK 制約 (`'running' | 'completed' | 'error' | 'emergency'`) に準拠する
- asyncpg は型変換を自動的に行うため、UUID は文字列で渡してよい（asyncpg が UUID → `uuid` 型に変換）
