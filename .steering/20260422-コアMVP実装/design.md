# 設計: コア機能（MVP）実装

## アーキテクチャ方針

### リポジトリ層の設計

Protocol ベースの二重実装：
- `ProfileRepository` (asyncpg.Pool で DB 永続化)
- `ModeRepository` (asyncpg.Pool で DB 永続化)
- `SessionRepository` (asyncpg.Pool で読み取り専用)
- In-memory 版はスタブ (`src/app/stubs.py` に追加)

### アプリケーション層との統合

`app.py` の `lifespan` でリポジトリを生成して `app.state` に注入する:
```python
# 環境変数 DATABASE_URL があれば DB、なければ in-memory
app.state.profile_repo = ...
app.state.mode_repo = ...
app.state.session_repo = ...
```

### RobotController の拡張

- `select_profile(profile: VehicleProfile) -> None` を追加
- `_active_profile: VehicleProfile | None` を保持
- `_active_profile_id` は `_active_profile.id` から導出
- `start_auto_drive` / `start_manual` で profile をDriveLoopに渡せるように

### Web API 設計

#### プロファイル管理
- `GET  /api/v1/profiles/`          一覧
- `POST /api/v1/profiles/`          新規作成
- `GET  /api/v1/profiles/{id}`      詳細
- `PUT  /api/v1/profiles/{id}`      更新
- `DELETE /api/v1/profiles/{id}`    削除

#### 走行モード管理
- `GET  /api/v1/modes/`             一覧
- `POST /api/v1/modes/upload`       CSV アップロード
- `GET  /api/v1/modes/{id}`         詳細
- `DELETE /api/v1/modes/{id}`       削除

#### セッション
- `GET  /api/v1/sessions/`          一覧
- `GET  /api/v1/sessions/{id}/logs` ログ一覧

#### プロファイル選択 (drive router)
- `POST /api/v1/drive/select-profile`  {profile_id} → RobotController に反映

## 注意事項

- calibration_data テーブルに `UNIQUE (profile_id)` 制約が漏れているため setup_db.py を修正
- CSV パースは RFC 4180 準拠 (Python 標準 `csv` モジュール使用)
- CSV 形式: `time_s,speed_kmh` のヘッダーあり
