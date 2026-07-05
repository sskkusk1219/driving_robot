# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

### 実装可能なタスクのみを計画
- 計画段階で「実装可能なタスク」のみをリストアップ
- 「将来やるかもしれないタスク」は含めない
- 「検討中のタスク」は含めない

### タスクスキップが許可される唯一のケース
以下の技術的理由に該当する場合のみスキップ可能:
- 実装方針の変更により、機能自体が不要になった
- アーキテクチャ変更により、別の実装方法に置き換わった
- 依存関係の変更により、タスクが実行不可能になった

スキップ時は必ず理由を明記:
```markdown
- [x] ~~タスク名~~（実装方針変更により不要: 具体的な技術的理由）
```

### タスクが大きすぎる場合
- タスクを小さなサブタスクに分割
- 分割したサブタスクをこのファイルに追加
- サブタスクを1つずつ完了させる

---

**注**: 実機検証タスク(フェーズ9)は車両・シャシダイナモが必要なため、実装セッションではなくユーザー立ち会いのベンチ作業で実施する(design.md「実機検証」参照)。実装セッションの完了範囲はフェーズ1〜8。

## フェーズ1: DB・モデル層(WP1)

- [x] `scripts/setup_db.py` の `DDL_STATEMENTS` に冪等DDLを追加
  - [x] `learning_cycles` テーブル(id/profile_id/status CHECK/started_at/ended_at/detail JSONB)
  - [x] `drive_sessions.cycle_id UUID REFERENCES learning_cycles(id)` + `idx_drive_sessions_cycle_id`
  - [x] `run_type` CHECK制約へ `'tuning'` 追加(DOブロックで drop→add、既存DOブロックの前例に倣う)
- [x] `src/models/drive_log.py`: `DriveSession.cycle_id: str | None` 追加、`LearningCycle` dataclass 新設、`_row_to_session` 更新
- [x] `src/infra/log_writer.py`
  - [x] `start_session(..., cycle_id: str | None = None)`(INSERTにcycle_id)
  - [x] `start_cycle(profile_id) -> str` / `end_cycle(cycle_id, status, detail=None)`
  - [x] `reap_interrupted_sessions` で孤児 `learning_cycles` を `error` 回収
- [x] `src/infra/session_repository.py`: `list_session_ids_for_cycle(cycle_id)`、`list_cycles(profile_id=None, limit=100)`(セッション数JOIN)
- [x] `src/app/stubs.py`: `InMemorySessionRepository`・スタブLogWriterに同メソッド追加(スタブLogWriterクラスは存在せず、DBなし時は`get_log_writer`がNoneを返すため対象なし)
- [x] `src/web/deps.py` の `SessionRepoProtocol`・`src/app/robot_controller.py` の `LogWriterProtocol` に新シグネチャ反映
- [x] テスト: `tests/unit/test_log_writer.py`(cycle_id列・start/end_cycle・reap回収)、`tests/unit/infra/test_session_repository.py`(list_session_ids_for_cycle・list_cycles)、`tests/unit/test_models.py`(LearningCycle・cycle_id)

## フェーズ2: コントローラ配管(WP2)

- [x] `_active_cycle_id` フィールド追加、`start_learning_drive` でサイクル開設(log_writer=None時はuuid4)、`select_profile` でクリア
- [x] `_open_session` が `cycle_id=self._active_cycle_id` を渡す
- [x] `_run_tuning_drive` のセッションを `run_type="tuning"` に変更
- [x] `run_pid_tuning_session` に `release_on_finish: bool = True` 追加(例外時は必ず解放、正常完了時のみスキップ可)
- [x] `run_pid_tuning_session` に `on_run` コールバック追加(走行ごとに走行番号・ゲイン・コストを通知)
- [x] `_learning_complete: asyncio.Event` 追加(`start_learning_drive` 冒頭clear、`stop_learning_drive` 末尾・`emergency_stop`/`stop` でset)
- [x] テスト: `tests/unit/test_robot_controller.py`(cycle_id継承、run_type='tuning'、release_on_finish両系、_learning_complete)

## フェーズ3: 訓練サービス抽出(WP3)

- [x] `src/app/training_service.py` 新規: `train_and_apply(..., update_pid_gains=True) -> TrainResult`(drive.py:374-468のロジック移設)
  - [x] `update_pid_gains=False` でFOPDT同定・SIMC・`profile.pid_gains` 更新をスキップ
  - [x] `refresh_active_profile()` の戻り値検査(FalseでRuntimeError)
  - 注: `feature_spec` 引数はFeatureSpec自体が未実装(WP4未着手)のため本フェーズでは追加せず、WP4で`train_inverse_model`がspecを受け付けるようになった時点でtraining_serviceにも追加する(未配線の引数を先行追加しない方針)
- [x] `/learning/train` ルーターを薄いラッパー化(`LearningDataError`→422踏襲、`RuntimeError`→409追加)
- [x] `TrainModelRequest.cycle_id` 追加(指定時 `list_session_ids_for_cycle` でsession_ids解決)
- [x] テスト: `tests/unit/test_training_service.py`(フラグ両系・refresh失敗・profile未検出・LearningDataError伝播)、`test_web_drive.py` のtrain既存テストが通ること(monkeypatch対象を`src.app.training_service.train_inverse_model`に更新)

## フェーズ4: FeatureSpec(WP4)

- [x] `src/domain/model_training.py`: `FeatureSpec` frozen dataclass(検証: lookahead昇順・regime_horizon含有・accel_horizonは両ホライズンに含有)、`DEFAULT_FEATURE_SPEC`
- [x] `build_feature_row` / `_build_feature_matrix` / `train_inverse_model` に spec を貫通(加速度項 `a_h=(v(t+h)-2v0+v(t-h))/h²`)
- [x] `MODEL_TYPE` を `"poly_spec_inverse_lookahead"` へバンプ、pklに `feature_spec` dict 保存
- [x] `src/domain/control/feedforward.py`: `load_model` で `feature_spec` 復元(欠落・旧model_typeは拒否)、`horizons`/`past_horizons`/regime判定をspec由来に変更
- [x] `scripts/analyze_decel_fit.py` の `_REGIME_COL`/`FEATURE_NAMES` 参照をspec由来に更新
- [x] `src/infra/settings.py` + `config/settings.toml.example`: `ModelSettings`(`[model]` セクション、デフォルトは現行9特徴)を追加し、`training_service.feature_spec_from_settings` + `app.py`(起動時読込→`app.state.feature_spec`) + `deps.get_feature_spec` + `/learning/train` 経由でtrainパスへ配線(本番デフォルト値は変更せず現行9特徴のまま)
- [x] テスト: `test_model_training.py`(デフォルトspec=現行9特徴の回帰ピン、カスタムspec、pklメタデータ、行列トリム)、`test_feedforward.py`(spec反映・旧pkl拒否・feature_spec欠落拒否・非デフォルトspecのpredict)、`test_drive_loop.py`(モジュール定数参照の更新)、`tests/unit/infra/test_settings.py`新規(`[model]`パース)、`test_training_service.py`(`feature_spec_from_settings`・`train_and_apply`へのfeature_spec貫通)。実データを使わない end-to-end smoke test(train→pkl保存→load_model→predict_effort、デフォルト/カスタムspec双方)で手動検証済み

## フェーズ5: オフライン特徴量評価スクリプト(WP5)

- [x] `scripts/evaluate_feature_sets.py` 新規(analyze_decel_fit.pyの規約: docstring usage・argparse・asyncio.run・読み取り専用)
  - [x] データ選択: `--cycle-id`(train=学習セッション/holdout=適合セッション)または明示セッションID(`--train-session-id`/`--eval-session-id`)
  - [x] 組み込み候補セット辞書(baseline / short_lookahead / short_plus_accel)+ `--spec-json`
  - [x] 指標: in-sample MAE/RMSE/R²、holdout-A(実速度特徴)、holdout-B(ref_speed_kmh特徴)、A−Bギャップ・特徴std比(actual/ref)
  - [x] holdout-B加重MAE順のランキング表出力、`--json-out` 対応、ref_speed欠落セッションのスキップ警告
  - 検証: 合成ログによるスモークテスト(3候補セットの評価・レポート出力・ref_speed欠落時の警告パス)で動作確認済み。既存の`analyze_decel_fit.py`同様、読み取り専用CLIのため専用pytestファイルは追加していない(プロジェクト内に前例なし)

## フェーズ6: オーケストレータ・設定・API・WS(WP6)

- [x] `src/infra/settings.py` + `config/settings.toml.example`: `LearningSettings`(`[learning]`: refine_runs_stage1=10, refine_runs_stage2=5, learning_timeout_s)
- [x] `src/app/learning_cycle.py` 新規: `CyclePhase` / `CycleProgress` / `LearningCycleOrchestrator`
  - [x] フェーズシーケンス実装(ARMING/LEARNING→TRAINING_1→REFINE_1(release_on_finish=False)→TRAINING_2(update_pid_gains=False)→REFINE_2→COMPLETED)
  - [x] 各REFINE後の最良ゲイン永続化(profile_repo.update + refresh_active_profile)
  - [x] エラー処理(停止→ブレーキ解放→end_cycle('error')→ERROR)と `abort()`(チェックポイント=フェーズ境界+on_run、`end_cycle('aborted')`)
  - [x] cycle detail に stage別 gains/costs/model_path/metrics を記録(1段目/2段目メトリクス併記)
  - 実装補足: `start()` はcycle_idを同期的に返す必要があるため、arm+学習運転開始(数秒〜十数秒)を同期実行し、以降(学習完了待ち〜適合〜再学習)をバックグラウンドタスク化。`RobotController._release_after_tuning`を`release_stop_hold`に公開名変更しオーケストレータの安全網解放から利用、`_begin_session`が返す`DriveSession.cycle_id`を設定、`active_cycle_id`プロパティを追加
- [x] API: `POST /drive/learning-cycle/start`(202/409/404/422)、`/abort`(200/409)、`GET /status`。`src/web/schemas.py` に各スキーマ(`CycleProgressSchema`/`LearningCycleStartRequest`/`Response`/`AbortResponse`)
- [x] `src/web/app.py` lifespanで生成し `app.state.cycle_orchestrator`、`deps.py` に `get_cycle_orchestrator`/`get_learning_settings`
- [x] `src/web/ws.py`: `RealtimeData.cycle_progress`(スタブモードNone安全)
- [x] `GET /api/v1/sessions/cycles`(sessions.py)+ `SessionResponse.cycle_id`
- [x] テスト: `tests/unit/test_learning_cycle.py`(フェーズ順序・TRAINING_2ゲイン不変・max_runs伝搬・LEARNING中abort・on_run abortチェックポイント・訓練エラー・タイムアウト、11件)、`test_web_drive.py`(新3エンドポイント+cycle_id付きtrain、9件追加)、`test_ws_broadcast.py`(cycle_progress配信、2件追加)。実App(lifespan込み)へのスモークテストでDI配線を確認済み

## フェーズ7: WebUI(WP7)

- [x] `src/web/static/js/screens/learning.js`
  - [x] 旧クライアント側自動チェーン削除(`runLearningPipeline`/`wasRunningRef`/`busyRef` のrobotState監視発火)
  - [x] 「学習サイクル開始」ボタン(確認ポップアップ→POST /learning-cycle/start)+「中断」ボタン(サイクル実行中のみ表示)
  - [x] WS `cycle_progress` の表示(フェーズ名・走行i/N・最良コスト、既存resultPanel枠)
  - [x] 手動パス(学習運転arm/start)の操作は維持されていることを確認(DriveMonitorScreenのprops変更なし)
- [x] `src/web/static/js/screens/logs.js`: cycle_idグループ表示(1サイクル=1折りたたみ、NULLはフラット)、`RUN_TYPE_LABEL` に tuning 追加(`SessionRow`/`CycleGroup`コンポーネントに分割)
- [x] Playwright MCPでUI動作確認: サーバー起動(stub/DBモード双方)→学習画面(プロファイル未選択/未キャリブレーション時のポップアップ、ボタン表示、コンソールエラー0件)→ログ画面をtest DBに学習サイクル(学習1+適合2)+フラットmanualセッションをシードして確認(サイクルグループ折りたたみ・展開・配下セッションのログ詳細表示まで実データで動作確認、コンソールエラー0件)。シードデータ・スクリーンショットは検証後にクリーンアップ済み

## フェーズ8: 品質チェックと修正

- [x] すべてのユニットテストが通ることを確認
  - [x] `.venv/bin/python -m pytest tests/unit/`(841件全通過)。加えて `tests/integration/test_web_api.py`(25件、DBなしTestClient)、`tests/integration/`(DB要、8件)も全通過
- [x] リントエラーがないことを確認
  - [x] `.venv/bin/python -m ruff check src/ tests/ scripts/`(All checks passed!)
- [x] 型エラーがないことを確認
  - [x] `.venv/bin/python -m mypy src/`(このステアリングでの変更ファイルは全てエラーなし。`src/domain/calibration.py:123`・`src/web/app.py`のUPS型不整合の2件は本ステアリング着手前から存在する既存の無関係なエラーで、変更前後で同一であることを `git stash` で確認済み・対象外)
- [x] `scripts/setup_db.py` を再実行して冪等性を確認(既存DBでエラーなく完走。`driving_robot_test` で新DDL適用後の再実行・スキーマ確認(`\d drive_sessions`/`\d learning_cycles`)も実施済み)

## フェーズ9: ドキュメント更新

- [x] `docs/functional-design.md` 更新: DriveSession/LearningCycleエンティティ・ER図、FeedforwardController(FeatureSpec)、新規`LearningCycleOrchestrator`コンポーネント節、PIDTuning節のrun_type='tuning'反映、UC6修正+UC7(学習サイクル)シーケンス図新設、REST API(`/learning-cycle/*`・`/learning/train`のcycle_id・`/sessions/cycles`)、WebSocketペイロード(`cycle_progress`)
- [x] `docs/architecture.md` 更新: `idx_drive_sessions_cycle_id`インデックス、ディレクトリ構成に`training_service.py`/`learning_cycle.py`/`evaluate_feature_sets.py`追加、PID自動適合フロー図を手動パス/学習サイクルの2節に分割
- [x] `docs/glossary.md` に用語追加: 学習サイクル・FeatureSpec・LearningCycle(データモデル)・セッション種別(run_type)表・学習サイクル状態表を新設、運転モデル/セッション/PID自動適合/座標降下/コンポーネント一覧の各既存項目を更新
- [x] 実装後の振り返り(このファイルの下部に記録)

---

## 実機検証メモ(実装セッション対象外・ベンチ作業)

design.md「実機検証」参照。①既存ログで評価スクリプト→特徴量選定判断 ②refine_runs 2/1 短縮サイクルでシーケンス検証(cycle_id共有・保持ブレーキ維持・最終ゲイン=2段目出力) ③フル10/5でKPI比較 ④各フェーズの中断・非常停止ドリル。

---

## 実装後の振り返り

### 実装完了日
2026-07-03

### 計画と実績の差分

**計画と異なった点**:
- **WP2の`_release_after_tuning`を公開名`release_stop_hold`へ改名**: design.mdでは既存の私有メソッド名のまま言及していたが、WP6でオーケストレータの安全網（例外・中断時のブレーキ解放）から呼ぶ必要が生じたため、公開APIとして命名し直した（呼び出し元2箇所+docstringを更新）。
- **`LearningCycleOrchestrator.start()`の同期/非同期境界**: design.mdは「cycle_idを返しバックグラウンドタスクを起動」とだけ記載していたが、cycle_idはcontroller.start_learning_drive()完了まで採番されない。実装では`start()`内でarm+学習運転開始（数秒〜十数秒、車速収束待ち含む）を同期実行してcycle_idを確定させ、以降（学習完了待ち〜適合〜再学習〜適合）のみを`asyncio.create_task`でバックグラウンド化した。結果としてPOST `/learning-cycle/start`の応答は数秒かかるが、既存の`/learning/arm`エンドポイントも同様の同期待ちをしており、UX的な逸脱はない。
- **`_begin_session`が返す`DriveSession.cycle_id`の未設定を発見・修正**: フェーズ1実装時点では`_open_session`内部でDBへcycle_idを渡すことのみ対応しており、呼び出し元へ返す`DriveSession`オブジェクト自体にはcycle_idが反映されていなかった（設計文書には明記なし）。オーケストレータがcycle_idを同期的に取得する経路として気づき、Phase 6着手時に修正・回帰テスト追加した。
- **`training_service.train_and_apply`への`feature_spec`引数**: WP3時点ではFeatureSpec未実装のため意図的に追加せず、WP4完了後に追加した（タスクリストに理由を明記して先行スキップ）。
- **`ModelSettings`/`LearningSettings`のtrainパス配線**: design.mdは「trainパスへ配線」とだけ記載していたが、具体的な配線経路（infra→app→web、layering順守）を実装時に設計: `training_service.feature_spec_from_settings()`（app層の変換関数）+ `app.py`起動時読込→`app.state.feature_spec`/`app.state.learning_settings` + `deps.get_feature_spec`/`get_learning_settings`。infraレイヤーがdomainのFeatureSpecへ直接依存しないよう変換をapp層に置いた。

**新たに必要になったタスク**:
- `RobotController.active_cycle_id`プロパティの追加（オーケストレータがサイクルIDを安全に参照するため）。
- `tests/unit/infra/test_settings.py`の新規作成（`[model]`/`[learning]`セクションのTOMLパース検証。既存に類似テストが無かったため新設）。
- Playwright MCPによる実ブラウザ検証（学習画面・ログ画面双方、stubモードとDBモード双方で実施。ログ画面はtest DBへ学習サイクル+セッションを直接シードして折りたたみ表示・展開・ログ詳細表示まで実データで確認）。

**技術的理由でスキップしたタスク**: なし（全タスク完了）。

### 学んだこと

**技術的な学び**:
- **`asyncio.Event`ベースの完了待ち + `on_run`コールバックによる中断チェックポイント**は、既存の`_drive_complete`パターン（PID適合1走行の完了待ち）を踏襲しつつ、複数フェーズにまたがる長時間非同期処理（学習サイクル全体）の中断・エラー処理を素直に実装できた。`except CycleAborted`と`except Exception`を分けることで、ユーザー起因の中断とシステムエラーを`learning_cycles.status`（aborted/error）で明確に区別できる。
- **`run_pid_tuning_session`の`except Exception: release; raise` → `else: if release_on_finish: release`** という構造は、「例外時は必ず解放・正常完了時のみ条件付き解放」という安全要件を1箇所で表現でき、オーケストレータ側の重複解放処理（`_release_and_stop_if_needed`の冪等呼び出し）と組み合わせても実害がない設計にできた。
- **FeatureSpecの列順**: `regime_col()`が`1 + lookahead_horizons_s.index(regime_horizon_s)`で機械的に求まる設計にしたことで、`train_inverse_model`・`_build_feature_matrix`・`build_feature_row`・`FeedforwardController.predict_effort`の4箇所すべてが同じロジックでレジーム列を特定でき、実装の食い違いバグを防げた。
- **レイヤ間の変換責務の置き場所**（`ModelSettings`→`FeatureSpec`）は、infraがdomainへ依存しないアーキテクチャ制約を守るため、app層（`training_service.py`）に変換関数を置くのが自然だった。

**プロセス上の改善点**:
- 各フェーズ完了ごとに`pytest`・`ruff`・`mypy`をそのフェーズの変更ファイルに対して即座に実行し、フェーズ間の手戻りを防いだ。
- 安全性に関わるコード（ブレーキ保持・中断処理）は、ユニットテストの`FakeController`で状態遷移を明示的にシミュレートし、さらにPlaywrightで実ブラウザ・実APIエンドツーエンド動作を確認する二段構えにしたことで、モックだけでは見落としがちなDI配線漏れ（`app.state.cycle_orchestrator`未設定等）を早期発見できた。

### 次回への改善提案
- 実機検証（ベンチ作業）は本ステアリングのスコープ外だが、フェーズ間のブレーキ保持継続（`release_on_finish=False`時に本当に転動しないか）は実車で最優先に確認すること。
- `scripts/evaluate_feature_sets.py`の出力を確認し、短期ホライズン特徴（0.1/0.2/0.3秒）採用の可否を別ステアリングで判断すること（本ステアリングはスコープ外と明記済み）。
