# 設計書

## アーキテクチャ概要

既存のレイヤ構成(web → app → domain / infra)を維持し、以下を追加・変更する:

- **infra**: `learning_cycles` テーブル + `drive_sessions.cycle_id`(冪等DDLを `scripts/setup_db.py` に追加)
- **app**: 訓練処理をルーターから `training_service.py` へ抽出、新規 `learning_cycle.py`(サイクルオーケストレータ)
- **domain**: `FeatureSpec` による特徴量構築の設定可能化(`model_training.py` / `feedforward.py`)
- **web**: サイクル実行API 3本 + `RealtimeData.cycle_progress` + UI改修(learning.js / logs.js)

```
WebUI「学習サイクル開始」
  └→ POST /drive/learning-cycle/start
       └→ LearningCycleOrchestrator (src/app/learning_cycle.py) ← バックグラウンドタスク
            ├ 1. ARMING/LEARNING : controller.arm/start_learning_drive → _learning_complete 待ち
            ├ 2. TRAINING_1      : training_service.train_and_apply(学習セッション, update_pid_gains=True)
            ├ 3. REFINE_1        : controller.run_pid_tuning_session(max_runs=10, release_on_finish=False)
            ├ 4. TRAINING_2      : train_and_apply(サイクル全セッション, update_pid_gains=False)
            ├ 5. REFINE_2        : run_pid_tuning_session(max_runs=5, release_on_finish=True)
            └ 6. COMPLETED       : log_writer.end_cycle(detail={gains, costs, model paths})
       進捗は ws.py の100ms broadcast_loop が orchestrator.progress をpullして配信(イベントバスは作らない)
```

**車両安全の不変条件**: 学習運転終了(停車保持)→ 訓練 → 1段目適合(走行間・終了後も保持)→ 再学習 → 2段目適合、の全期間で車両は停車保持ブレーキで静止し続け、2段目完了(または中断・エラー)時のみ原点復帰で解放する。PID適合走行は走行前チェックをスキップして停止保持状態から直接開始するため、フェーズ境界で保持が切れると転動状態から走行が始まる。

## コンポーネント設計

### 1. DBスキーマ(scripts/setup_db.py の DDL_STATEMENTS に追加)

```sql
CREATE TABLE IF NOT EXISTS learning_cycles (
    id          UUID PRIMARY KEY,
    profile_id  UUID NOT NULL REFERENCES vehicle_profiles(id),
    status      TEXT NOT NULL CHECK (status IN ('running','completed','error','aborted')),
    started_at  TIMESTAMPTZ NOT NULL,
    ended_at    TIMESTAMPTZ,
    detail      JSONB NOT NULL DEFAULT '{}'::jsonb
);
ALTER TABLE drive_sessions ADD COLUMN IF NOT EXISTS cycle_id UUID REFERENCES learning_cycles(id);
CREATE INDEX IF NOT EXISTS idx_drive_sessions_cycle_id ON drive_sessions (cycle_id);
-- run_type CHECK 制約に 'tuning' を追加: DO $$ ブロックで既存制約 drop → add
-- (既存の calibration_data_profile_id_key の DO ブロックが前例)
```

**セマンティクス**:
- 学習運転開始で新サイクル開設(`status='running'`)。以降の適合走行セッションは `_active_cycle_id` を継承
- `select_profile` / サーバー再起動でサイクル参加は終了。manual・通常autoは `cycle_id NULL`
- `detail` JSONB に到達フェーズ・各段のゲイン/コスト/モデルパス/メトリクスを記録(スキーマレスで将来拡張)
- 既存の `run_type='auto'` 適合走行の過去行はそのまま(遡及変換しない・要求のスコープ外)

### 2. LogWriter / SessionRepository(src/infra/)

**log_writer.py**:
- `start_session(profile_id, mode_id, run_type, cycle_id: str | None = None) -> str` — INSERT に cycle_id 追加
- `async def start_cycle(self, profile_id: str) -> str` / `async def end_cycle(self, cycle_id: str, status: str, detail: dict | None = None) -> None`
- `reap_interrupted_sessions`: 孤児 `learning_cycles`(`status='running'`)も `error` で回収

**session_repository.py**:
- `async def list_session_ids_for_cycle(self, cycle_id: str) -> list[str]`
- `async def list_cycles(self, profile_id: str | None = None, limit: int = 100) -> list[LearningCycle]`(セッション数・学習セッションIDをJOINで付与)

**src/models/drive_log.py**: `DriveSession.cycle_id: str | None` 追加、`LearningCycle` dataclass 新設。
**src/app/stubs.py**: `InMemorySessionRepository` 等に同メソッドを追加(DBなしモード・テスト用)。
**src/web/deps.py / robot_controller.py の LogWriterProtocol**: 新シグネチャを反映。

### 3. RobotController 配管(src/app/robot_controller.py)

- `_active_cycle_id: str | None` フィールド追加。`start_learning_drive`(現 :1267)で `log_writer.start_cycle()`(log_writer=None時はローカルuuid4)を呼び設定。`select_profile` でクリア
- `_open_session`(現 :780)が `cycle_id=self._active_cycle_id` を `start_session` へ渡す
- `_run_tuning_drive`(現 :865)の `_begin_session` を `run_type="tuning"` に変更
- **`run_pid_tuning_session(..., *, release_on_finish: bool = True)`**: 現在 finally で無条件に `_release_after_tuning()`(両軸原点復帰=ブレーキ解放)している(:971-972)。変更後: 例外時は必ず解放、正常完了時のみ `release_on_finish=False` でスキップ可能。オーケストレータの1段目はこれで保持を維持する
- `run_pid_tuning_session` にオプション `on_run: Callable[[int, PIDGains, float], None] | None = None` を追加(走行ごとに (走行番号, ゲイン, コスト) を通知。オーケストレータの進捗更新・中断チェックに使用)
- **`_learning_complete: asyncio.Event`** 新設: `stop_learning_drive`(現 :1342)の末尾と `emergency_stop`/`stop` で set。`start_learning_drive` 冒頭で clear(既存 `_drive_complete` の clear 位置のパターンに合わせる)

### 4. 訓練サービス抽出(src/app/training_service.py 新規)

`src/web/routers/drive.py:374-468` のロジックを移設(FastAPI 非依存):

```python
@dataclass
class TrainResult:
    model_path: str
    metrics: dict[str, dict[str, float]]
    feedforward_params: FeedforwardParams
    pid_gains: PIDGains
    pid_auto_tuned: bool

async def train_and_apply(
    *, profile_repo, session_repo, controller, profile_id: str,
    session_ids: list[str], update_pid_gains: bool = True,
    feature_spec: FeatureSpec | None = None,
) -> TrainResult
```

- 現行どおり `asyncio.to_thread` で `train_inverse_model` / `estimate_dynamics_params` / `identify_fopdt`+`compute_pid_gains_simc` を実行
- **`update_pid_gains=False` 時は FOPDT同定・SIMC計算・`profile.pid_gains` 更新をスキップ**(2段目再学習で1段目の座標降下結果を消さないための必須フラグ)
- `controller.refresh_active_profile()` の戻り値を検査し、False(RUNNING等)なら例外(オーケストレータはフェーズ間READYで呼ぶため通常成功する。失敗はシーケンスバグの検出)
- `/learning/train` ルーターは本サービスの薄いラッパーに(`LearningDataError` → 422 は現行踏襲)。`TrainModelRequest` に `cycle_id: str | None = None` を追加し、指定時は `list_session_ids_for_cycle` で session_ids を解決

### 5. FeatureSpec(src/domain/model_training.py / feedforward.py)

```python
@dataclass(frozen=True)
class FeatureSpec:
    lookahead_horizons_s: tuple[float, ...] = (0.5, 1.0, 2.0, 3.0)  # 昇順必須
    past_horizons_s: tuple[float, ...] = (0.5, 1.0)
    regime_horizon_s: float = 1.0        # lookahead_horizons_s に含まれること(__post_init__で検証)
    include_v0_sq: bool = True
    include_dv_regime_x_v0: bool = True
    accel_horizons_s: tuple[float, ...] = ()  # a_h = (v(t+h) - 2*v0 + v(t-h)) / h²(中央差分)
                                              # h は lookahead/past 両方に含まれること
    def feature_names(self) -> list[str]: ...
    def regime_col(self) -> int: ...      # 特徴行列内の dv_{regime_horizon} の列番号
DEFAULT_FEATURE_SPEC = FeatureSpec()      # 現行9特徴と完全一致(回帰テストで固定)
```

- `build_feature_row(v0, future_speeds, past_speeds, spec=DEFAULT_FEATURE_SPEC)` / `_build_feature_matrix(..., spec)` / `train_inverse_model(..., feature_spec=DEFAULT_FEATURE_SPEC)` に spec を貫通させる
- **pkl**: `MODEL_TYPE` を `"poly_spec_inverse_lookahead"` へバンプ(旧pkl拒否は既存規約)。`feature_spec` を plain dict で保存(`horizons`/`past_horizons` キーは後方互換のため残してもよいが読み手はfeature_specを正とする)
- **feedforward.py**: 現状 `LOOKAHEAD_HORIZONS_S` 等のモジュール定数を直接参照し、pkl内のホライズンを読んでいない。`load_model` で `feature_spec` を復元(欠落時は拒否)し、`horizons`/`past_horizons` プロパティを spec 由来に変更。**DriveLoop は変更不要**(:352-357 で `self._ff.horizons` を毎サイクル参照しているため可変ホライズンが自然に流れる)。`predict_effort` の regime 判定は `spec.regime_col()`/`spec.regime_horizon_s` を使用。停車ショートサーキットは `future_speeds[0]` を使うため昇順制約が前提
- `scripts/analyze_decel_fit.py` の `_REGIME_COL`/`FEATURE_NAMES` 参照を spec 由来に更新
- 本番デフォルトは現行9特徴のまま。特徴セットの切替は `config/settings.toml` の `[model]` セクション(`ModelSettings` dataclass を settings.py に追加)経由で可能にするが、値の変更はオフライン評価後に別途判断

### 6. オフライン特徴量評価スクリプト(scripts/evaluate_feature_sets.py 新規)

`scripts/analyze_decel_fit.py` の規約(モジュールdocstringに usage、argparse、`asyncio.run`、`create_pool`/`load_settings`/`SessionRepository`、読み取り専用)に従う。

- **入力**: `--profile-id`(必須)。データ選択は `--cycle-id`(train=サイクルの学習セッション、holdout=サイクルの適合走行セッション)または `--train-session-id` + `--eval-session-id ...`。`--specs` で組み込み候補名(`baseline` / `short_lookahead`(+0.1/0.2/0.3秒) / `short_plus_accel` 等の辞書)、`--spec-json` で任意定義。`--json-out` で結果保存(省略時はstdout表のみ)
- **候補セットごと×レジーム(accel/brake)ごとに算出**:
  - in-sample MAE/RMSE/R²(`train_inverse_model` と同一手順で訓練)
  - **holdout-A**: holdoutログの `actual_speed_kmh` 軌跡から特徴構築 → 予測開度 vs 実開度
  - **holdout-B**: holdoutログの `ref_speed_kmh`(規定パターン=滑らかな基準。DriveLoopが記録済み)から特徴構築 → 予測 vs 実開度
  - A−Bギャップと特徴ごとのstd比(actual vs ref)— 「学習はノイジーな実速度・推論は滑らかな基準速度」という訓練/推論の不一致を可視化する。短期ホライズン(0.1秒≒ログ1サンプル差分)はこの不一致を増幅するため、**採否判断は配備条件に近い holdout-B MAE で行う**
- ガード: `ref_speed_kmh` が無いholdoutセッション(学習運転ログ等)はholdout-Bからスキップして警告

### 7. サイクルオーケストレータ(src/app/learning_cycle.py 新規)

```python
class CyclePhase(StrEnum):
    IDLE, ARMING, LEARNING, TRAINING_1, REFINE_1, TRAINING_2, REFINE_2, COMPLETED, ERROR, ABORTED

@dataclass
class CycleProgress:
    cycle_id: str | None
    phase: CyclePhase
    run_index: int          # 現フェーズの走行番号(REFINE中のみ有効)
    run_total: int
    best_cost: float | None
    message: str
    started_at: datetime | None

class LearningCycleOrchestrator:
    def __init__(self, controller, profile_repo, session_repo, log_writer): ...
    @property
    def progress(self) -> CycleProgress: ...
    async def start(self, profile_id: str, refine_runs_stage1: int, refine_runs_stage2: int) -> str
        # cycle_id を返しバックグラウンドタスク(asyncio.create_task)を起動。二重起動は例外
    async def abort(self) -> None
```

**フェーズシーケンス**(バックグラウンドタスク内、各ステップで `self._progress` を更新):
1. ARMING/LEARNING: `arm_learning_drive()` → `start_learning_drive(log_writer)`(ここでサイクル開設)→ `_learning_complete` を待つ(タイムアウト=固定の余裕値。学習パターン総時間はマネージャから取得困難なため定数 `LEARNING_TIMEOUT_S` で運用し、超過はERROR)
2. TRAINING_1: `train_and_apply(session_ids=[学習セッション], update_pid_gains=True)`(車両は停車保持のままREADY)
3. REFINE_1: `run_pid_tuning_session(profile, log_writer, max_runs=stage1, release_on_finish=False, on_run=進捗/中断チェック)` → 最良ゲインを `profile_repo.update` + `refresh_active_profile` で永続化
4. TRAINING_2: `session_ids = list_session_ids_for_cycle(cycle_id)`(学習1+適合N)で `train_and_apply(update_pid_gains=False)`。先読み特徴のセッション境界分離は既存 `_group_by_session` がそのまま担保
5. REFINE_2: `run_pid_tuning_session(max_runs=stage2, release_on_finish=True)`(1段目ゲインは既に `self._pid` とprofileに反映済みなので初期値として自然に継続)→ 最良ゲイン永続化
6. COMPLETED: `end_cycle(cycle_id, 'completed', detail={stage別 gains/costs/model_path/metrics})`。2段目メトリクスは1段目と併記し退行を可視化

**エラー/中断処理**:
- 全体を try/except で包み、例外時: RUNNING なら `controller.stop()`(優雅停止・セッションは 'completed' で閉じる)→ 必ずブレーキ解放(`release_on_finish` 相当の原点復帰)→ `end_cycle('error', detail={phase, error})` → progress=ERROR
- `abort()`: フラグを立てる。チェックポイントはフェーズ境界と `on_run` コールバック(次走行開始前)。走行中なら `controller.stop()`。処理後 `end_cycle('aborted')`。**中断済みサイクルのセッション群は有効な学習データとして残る**(後で `cycle_id` 指定の手動再学習に使える)
- `_drive_complete`/`_learning_complete` の clear は各走行/学習の開始直前(既存 `_run_tuning_drive` のパターン踏襲)。`emergency_stop` は両方を set(冪等)

### 8. 設定・API・WS(src/infra/settings.py, src/web/)

**settings.py + config/settings.toml**: `LearningSettings` dataclass、`[learning]` セクション: `refine_runs_stage1 = 10`, `refine_runs_stage2 = 5`, `learning_timeout_s`。

**API(src/web/routers/drive.py, schemas.py)**:
- `POST /api/v1/drive/learning-cycle/start` — body `{"refine_runs_stage1"?: int, "refine_runs_stage2"?: int}`(ge=1 le=50、省略時settings)。202 `{"cycle_id", "status": "started"}`。READY以外・サイクル実行中は409、プロファイル未選択404
- `POST /api/v1/drive/learning-cycle/abort` — 200 `{"status": "aborting"}`、非実行中409
- `GET /api/v1/drive/learning-cycle/status` — `CycleProgressSchema`
- `GET /api/v1/sessions/cycles` — `CycleSummaryResponse` 一覧(sessions.py)。`SessionResponse` に `cycle_id` 追加
- オーケストレータは `src/web/app.py` の lifespan で生成し `app.state.cycle_orchestrator` に保持、`deps.py` に `get_cycle_orchestrator`

**WS(src/web/ws.py, schemas.py)**: `RealtimeData.cycle_progress: CycleProgressSchema | None = None` を追加し、既存100ms `broadcast_loop` が `orchestrator.progress` をpull(スタブモードNone安全)。新イベント機構は作らない。

### 9. WebUI(src/web/static/js/)

**learning.js**:
- **既存のクライアント側自動チェーンを削除**: `robotState` 監視(`wasRunningRef`)による `runLearningPipeline`(train→refine自動発火)と `busyRef`。残すとオーケストレーション中のREADY遷移(〜16回)ごとに旧チェーンが発火し二重訓練になる
- 「学習サイクル開始」ボタン(確認ポップアップ→ `POST /learning-cycle/start`)と「中断」ボタンを追加
- WSペイロードの `cycle_progress` からフェーズ名・走行 i/N・最良コストを既存 resultPanel 枠に表示
- 手動パス用の学習運転 arm/start 操作は維持(DriveMonitorScreen の props 変更なし)

**logs.js**: セッション一覧を `cycle_id` でグループ表示(1サイクル=1折りたたみ項目: 学習1+適合N。`cycle_id NULL` は従来どおりフラット)。`RUN_TYPE_LABEL` に `tuning` を追加。

## エラーハンドリング戦略

- 既存 `LearningDataError` は現行踏襲(422)。オーケストレータ内では ERROR フェーズ遷移+`end_cycle('error')` に変換
- 新規例外: `CycleAborted`(中断チェックポイントで送出、ABORTED処理へ)、`CycleBusyError`(二重起動→409)
- `refresh_active_profile` False は RuntimeError(シーケンスバグ検出用)
- どのエラーパスでも「ブレーキ解放→サイクル行クローズ→progress更新」の順で必ず実施(finally)

## テスト戦略

### ユニットテスト(tests/unit/、srcレイアウトをミラー)
- `test_model_training.py`: DEFAULT specの特徴名/値が現行9特徴と一致する回帰ピン(既存 :94-108 を更新)、カスタムspec(0.1/0.2/0.3秒+加速度項)の names/値/行列境界トリム、pklの `feature_spec`、新MODEL_TYPE
- `test_feedforward.py`: pklのspecが `horizons` に反映、旧model_type拒否、`feature_spec` 欠落拒否、非デフォルトspecでのpredict
- `test_log_writer.py`(+ `tests/integration/`): start_sessionのcycle_id、start_cycle/end_cycle、reapの孤児サイクル回収
- `test_robot_controller.py`: 学習→適合セッションへのcycle_id継承、`run_type='tuning'`、`release_on_finish=False` は正常時に原点復帰しない/例外時は必ず解放、`_learning_complete` がstop/emergencyでset
- 新 `test_learning_cycle.py`: スタブ依存でハッピーパスのフェーズ順序(TRAINING_2 が `update_pid_gains=False` でゲイン不変)、max_runs 10/5 の伝搬、各フェーズでのabort→ABORTED+解放、訓練エラー→ERROR+サイクル行クローズ
- 新 `test_training_service.py`: `update_pid_gains` フラグ両系、refresh失敗時例外
- `test_web_drive.py`: 新3エンドポイント(202/409二重起動/abort、status)

### 実機検証(ベンチ)
1. 既存ログで `evaluate_feature_sets.py` を実行し出力の妥当性確認(特徴量選定はこの結果で別途判断)
2. `refine_runs 2/1` に絞った短縮サイクルでシーケンス検証: cycle_id共有(GET /sessions/cycles)、フェーズ間の保持ブレーキ維持(WSのブレーキ開度監視)、最終 `profile.pid_gains` = 2段目出力(SIMC値でない)こと
3. フル 10/5 サイクル実行、KPI達成率を1段階学習と比較
4. 中断ドリル: 各フェーズでabort+物理非常停止、ブレーキ解放とサイクル状態記録を確認

## 依存ライブラリ

新規追加なし(sklearn / asyncpg / FastAPI 既存のまま)。

## ディレクトリ構造

```
scripts/setup_db.py                     # DDL追加
scripts/evaluate_feature_sets.py        # 新規
src/app/training_service.py             # 新規(drive.pyから抽出)
src/app/learning_cycle.py               # 新規(オーケストレータ)
src/app/robot_controller.py             # cycle_id配管・release_on_finish・_learning_complete
src/app/stubs.py                        # スタブにサイクルメソッド追加
src/domain/model_training.py            # FeatureSpec
src/domain/control/feedforward.py       # spec復元・spec由来プロパティ
src/infra/log_writer.py                 # cycle CRUD
src/infra/session_repository.py         # cycle読み取り
src/infra/settings.py                   # LearningSettings/ModelSettings
src/models/drive_log.py                 # cycle_id / LearningCycle
src/web/routers/drive.py                # learning-cycle API・train薄型化
src/web/routers/sessions.py             # GET /sessions/cycles
src/web/schemas.py                      # 各スキーマ
src/web/ws.py                           # cycle_progress
src/web/app.py / deps.py                # orchestrator DI
src/web/static/js/screens/learning.js   # 旧チェーン削除・サイクルUI
src/web/static/js/screens/logs.js       # サイクルグループ表示
tests/unit/...                          # 上記テスト戦略
```

## 実装の順序

1. DB/モデル層(WP1): setup_db.py → log_writer → session_repository → models → stubs
2. コントローラ配管(WP2): cycle_id・run_type='tuning'・release_on_finish・_learning_complete
3. 訓練サービス抽出(WP3): training_service.py + /learning/train 薄型化 + cycle_id指定訓練
4. FeatureSpec(WP4): model_training → feedforward → analyze_decel_fit更新
5. オフライン評価スクリプト(WP5)
6. オーケストレータ+設定+API+WS(WP6)
7. WebUI(WP7): 旧チェーン削除は新API追加と同一変更で実施
8. テスト・実機検証(WP8)

## セキュリティ考慮事項

- 既存API同様に認証なしLAN内運用の前提を踏襲。サイクル開始は409で排他制御し、二重起動による競合走行を防ぐ

## パフォーマンス考慮事項

- 訓練(`asyncio.to_thread`)中もWS broadcast_loop(100ms)とコントローラのブレーキ保持は継続すること(ブロッキング処理をイベントループに載せない)
- TRAINING_2 はセッション11件分のログ読み込みになるが、`list_logs_for_training` は既存の一括クエリで対応可能(数万行オーダー)

## 主要リスク

1. **ブレーキ保持のフェーズ跨ぎ**(最重要): `release_on_finish` の実装漏れ・例外パスの解放漏れがあると、2段目適合が転動状態から pre-check なしで開始される
2. **learning.js 旧チェーンの削除漏れ**: オーケストレーション中に旧チェーンが発火し二重訓練・二重refineになる。新APIと同一コミットで削除
3. **短期ホライズンのノイズ増幅**: dv_0.1 はログ1サンプル差分≒CAN量子化ノイズ。オフライン評価(holdout-B)で事前検証し、即採用しない
4. **閉ループログのラベルはFF+PID合成開度**: 2段目の逆モデルは合成制御器側に寄るバイアスを持つ。要求上許容だが、cycle detail に1段目/2段目メトリクスを併記して退行を可視化する
5. **run_type CHECK制約の変更**: DOブロックのdrop→addで既存データに影響しないこと(冪等性)をsetup_db再実行で確認
6. **イベントのclearタイミング**: `_drive_complete`/`_learning_complete` は各開始直前にclear(既存パターン踏襲)。emergency_stop の二重setは冪等で無害

## 将来の拡張性

- `learning_cycles.detail` JSONB により段数追加(3段以上の反復)やメトリクス項目追加はスキーマ変更なしで対応可能
- `FeatureSpec` は spec-json 入力に対応するため、オフライン評価→`[model]` 設定変更→再訓練のループが実装変更なしで回せる
- オーケストレータのフェーズ定義はリスト化すれば N 段反復に一般化できる(今回は2段固定で実装)
