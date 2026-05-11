# タスクリスト: Kvaser CANReader 実装・修正・テスト

## フェーズ1: 依存関係・設定の整備

- [x] `pyproject.toml` に `cantools>=0.18.0` を追加する
- [x] `cantools` を仮想環境にインストールする (`pip install cantools`)
- [x] `config/settings.toml` を `settings.toml.example` からコピー作成する

## フェーズ2: ソースコード修正

- [x] `src/infra/can_reader.py`: `_SPEED_SIGNAL_NAME = "VehicleSpeed"` → `"Speed"` に修正
- [x] `src/infra/settings.py`: `CanSettings` に `dbc_path: str` フィールドを追加する
- [x] `config/settings.toml.example`: `[can]` セクションに `dbc_path` を追記する
- [x] `config/settings.toml`: `[can]` セクションに `dbc_path` を追記する
- [x] `src/app/factory.py`: `CANReader` に `dbc_path=settings.can.dbc_path` を渡す

## フェーズ3: テスト修正・追加

- [x] `tests/unit/infra/test_can_reader.py`: 既存テストのシグナル名を `"Speed"` に更新する
  - `test_read_speed_success`: `{"VehicleSpeed": 72.5}` → `{"Speed": 72.5}`
  - `test_read_speed_missing_signal_raises`: match=`"Speed"` に変更
- [x] `tests/unit/infra/test_can_reader.py`: DBC ファイルを使った統合テストを追加する
  - `test_dbc_loads_and_has_speed_signal`: 実際の DBC ファイルのロードと Speed シグナルの存在確認
  - `test_read_speed_with_real_dbc`: 実際の DBC + モック Bus でデコードが成功することを確認

## フェーズ4: 検証

- [x] `pytest tests/unit/infra/test_can_reader.py -v` で全件パスを確認（13件、不明フレームIDテスト追加）
- [x] `pytest tests/ -v` で全テストパスを確認（388件）
- [x] `ruff check src/infra/can_reader.py src/infra/settings.py src/app/factory.py` でエラーなし確認

---

## 実装後の振り返り

**実装完了日**: 2026-04-22

### 計画と実績の差分

- 計画通り: 全6ファイルの修正と1ファイルの新規作成を完了
- 追加対応: `tests/integration/test_web_api.py` の修正（計画外）
  - `app.state.profile_repo/mode_repo/session_repo` が未設定だったため統合テストが失敗していた
  - これは既存の `src/web/app.py` の修正（`_build_repos` 追加）に対応するテスト側の修正漏れ
  - テストフィクスチャに in-memory リポジトリを注入して解決
- 追加対応: `tests/unit/test_factory.py` の修正（計画外）
  - `CanSettings` に `dbc_path` が追加されたため、factory テストの `make_settings` と assertion を更新

### 学んだこと

1. **DBC シグナル名の確認が必要**: `can_reader.py` の `_SPEED_SIGNAL_NAME = "VehicleSpeed"` が実際の DBC ファイルのシグナル名 `Speed` と異なっていた。実機で使用する前に DBC ファイルとコードの整合性を確認することが重要。
2. **cantools のエンコード/デコード**: `msg_def.encode({"Speed": 100.0, "dummy": 0.0})` で生バイトを生成し `decode_message` でデコードする統合テストパターンが有効。
3. **test フィクスチャと lifespan の分離**: FastAPI の `AsyncClient` はデフォルトで lifespan を実行しないため、テストフィクスチャで必要な状態を明示的に注入する必要がある。

### 次回への改善提案

- CAN バスのビットレート設定（`config/settings.toml` の `[can]` セクションに `bitrate` フィールド追加を検討）
- MEIDACS の CAN バスビットレートを実機確認後に DBC ファイルのコメントに記載する
- 実機テスト手順を `tests/hardware/` に追加する（現在は未実装）
