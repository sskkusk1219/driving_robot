# タスクリスト: コア機能（MVP）実装

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

---

## フェーズ1: DB スキーマ修正

- [x] `scripts/setup_db.py` - `calibration_data` に `UNIQUE (profile_id)` 制約を追加

## フェーズ2: ProfileRepository CRUD 拡張

- [x] `src/infra/profile_repository.py` に以下を追加:
  - [x] `list_all()` → `list[VehicleProfile]`
  - [x] `get_by_id(profile_id: str)` → `VehicleProfile | None`
  - [x] `create(profile: VehicleProfile)` → `VehicleProfile`
  - [x] `update(profile: VehicleProfile)` → `VehicleProfile | None`
  - [x] `delete(profile_id: str)` → `bool`

## フェーズ3: ModeRepository 新規作成

- [x] `src/infra/mode_repository.py` を新規作成:
  - [x] `list_all()` → `list[DrivingMode]`
  - [x] `get_by_id(mode_id: str)` → `DrivingMode | None`
  - [x] `create(mode: DrivingMode)` → `DrivingMode`
  - [x] `delete(mode_id: str)` → `bool`

## フェーズ4: SessionRepository 新規作成

- [x] `src/infra/session_repository.py` を新規作成:
  - [x] `list_all(limit: int = 100)` → `list[DriveSession]`
  - [x] `get_by_id(session_id: str)` → `DriveSession | None`
  - [x] `list_logs(session_id: str, limit: int = 1000)` → `list[DriveLog]`

## フェーズ5: In-memory リポジトリ（スタブ用）

- [x] `src/app/stubs.py` に追加:
  - [x] `InMemoryProfileRepository` (list_all, get_by_id, create, update, delete, save_calibration)
  - [x] `InMemoryModeRepository` (list_all, get_by_id, create, delete)
  - [x] `InMemorySessionRepository` (list_all, get_by_id, list_logs) - 常に空リスト

## フェーズ6: schemas.py 拡張

- [x] `src/web/schemas.py` に以下を追加:
  - [x] `PIDGainsSchema`, `StopConfigSchema`
  - [x] `ProfileCreateRequest`, `ProfileUpdateRequest`, `ProfileResponse`
  - [x] `ModeResponse`, `SpeedPointSchema`
  - [x] `SessionResponse`

## フェーズ7: deps.py 拡張

- [x] `src/web/deps.py` に以下を追加:
  - [x] `get_profile_repo(request)` → ProfileRepository
  - [x] `get_mode_repo(request)` → ModeRepository
  - [x] `get_session_repo(request)` → SessionRepository

## フェーズ8: profiles ルーター実装

- [x] `src/web/routers/profiles.py` を実装:
  - [x] `GET /` - list_all()
  - [x] `POST /` - create()
  - [x] `GET /{id}` - get_by_id()
  - [x] `PUT /{id}` - update()
  - [x] `DELETE /{id}` - delete()

## フェーズ9: modes ルーター実装

- [x] `src/web/routers/modes.py` を実装:
  - [x] `GET /` - list_all()
  - [x] `POST /upload` - CSV アップロード → DrivingMode 作成
  - [x] `GET /{id}` - get_by_id()
  - [x] `DELETE /{id}` - delete()

## フェーズ10: sessions ルーター実装

- [x] `src/web/routers/sessions.py` を実装:
  - [x] `GET /` - list_all()
  - [x] `GET /{id}/logs` - list_logs()

## フェーズ11: RobotController 拡張

- [x] `src/app/robot_controller.py` に `select_profile(profile: VehicleProfile) -> None` 追加
- [x] `_active_profile: VehicleProfile | None` フィールドを追加
- [x] `get_active_profile()` → `VehicleProfile | None` を追加

## フェーズ12: drive ルーター拡張

- [x] `src/web/routers/drive.py` に以下を追加:
  - [x] `POST /select-profile` - {profile_id} → RobotController.select_profile()

## フェーズ13: app.py 更新

- [x] `src/web/app.py` の `lifespan` を更新:
  - [x] `DATABASE_URL` 環境変数があれば DB pool を作成し DB バックエンドリポジトリを使用
  - [x] 未設定なら In-memory リポジトリを使用
  - [x] `app.state.profile_repo`, `app.state.mode_repo`, `app.state.session_repo` に設定

## フェーズ14: テスト追加

- [x] `tests/unit/infra/test_profile_repository.py` に CRUD テストを追加
  - [x] list_all, get_by_id, create, update, delete のテスト
- [x] `tests/unit/infra/test_mode_repository.py` を新規作成
  - [x] list_all, get_by_id, create, delete のテスト
- [x] `tests/unit/test_robot_controller.py` に select_profile テストを追加
- [x] `tests/unit/test_web_drive.py` に select-profile エンドポイントテストを追加

## フェーズ15: 品質チェック

- [x] `python -m pytest tests/unit/ -q` → 347 passed, 3 warnings
- [x] `python -m ruff check src/ tests/` → All checks passed!
- [x] `python -m mypy src/` → Success: no issues found in 43 source files

---

## 実装後の振り返り

**実装完了日**: 2026-04-22

### 計画と実績の差分

- **計画通り実装できた部分**: フェーズ1〜14は設計通りに完了。スタブ/DBバックエンドの切り替えも `DATABASE_URL` 環境変数で問題なく動作。
- **追加で必要だった修正**:
  - `deps.py` に Protocol クラス（`ProfileRepoProtocol`, `ModeRepoProtocol`, `SessionRepoProtocol`）を追加してルーターの型安全性を確保（当初設計では想定外）
  - `infra/` の `delete()` 返り値が `Any` になり mypy エラー → `str(result) != "DELETE 0"` にキャスト
  - `python-multipart` が未インストールで CSV アップロードが失敗 → pip install で解決
  - テストのプロファイル ID が非 UUID 文字列だったため、UUID バリデーション時にエラー → `str(uuid4())` に全面変更

### 学んだこと

- asyncpg の `conn.execute()` 戻り値は `str` ではなく `Any` 型扱いになるため、`str()` でキャストしてから比較する必要がある
- FastAPI の `UploadFile` を使う際は `python-multipart` が必要（依存関係として明示すべき）
- Protocol クラスはルーター側の型注釈を `object` のままにしておくと mypy エラーが発生するため、`Annotated[ProtocolType, Depends(...)]` まで揃える必要がある

### 実装バリデーション後に追加修正した事項

- `ProfileUpdateRequest` にバリデーター追加（`opening_range`, `speed_positive`）
- `update()` の result 比較を `str()` で統一（`str(result) == "UPDATE 0"`）
- `select_profile()` に状態チェック追加（STANDBY/READY のみ許可、それ以外は `InvalidStateTransition`）
- `profiles.py` ルーターの遅延インポートを削除（循環インポートなしと確認済み）
- CSV アップロードに 10MB 上限を追加
- 各ルーターの get/delete で `ValueError`（不正 UUID）→ 400 のハンドリングを追加
- `SessionRepository` のユニットテストを新規作成（`tests/unit/infra/test_session_repository.py`）
- `docs/functional-design.md` の API 定義を実装済み内容に更新

### 次回への改善提案

- `pyproject.toml` に `python-multipart` を明示的に依存関係として追加すること
- DB バックエンドのインテグレーションテストを追加する（現状はユニットテストのみ）
- セッションリポジトリの `list_all` に日付範囲フィルタを追加するとログ取得が便利になる
- `profiles`/`modes`/`sessions` ルーターのルーターレベルのユニットテスト（`test_web_profiles.py` 等）を追加してカバレッジを向上させる
