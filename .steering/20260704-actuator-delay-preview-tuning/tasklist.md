# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

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

**実装担当メモ**: 分析・設計はFable(本ステアリング)、実装はSonnetが行う。実装前に requirements.md / design.md を必ず読むこと。design.md に各変更の行番号目安と根拠がある。

## フェーズ1: 永続化基盤(models / DB / repository)

- [x] `src/models/profile.py`: `DynamicsParams` dataclass 追加(preview_time_s=0.0, fopdt_k/tau/theta=None)
  - [x] `VehicleProfile` 末尾に `dynamics_params: DynamicsParams = field(default_factory=DynamicsParams)` を追加
- [x] `scripts/setup_db.py`: `ALTER TABLE vehicle_profiles ADD COLUMN IF NOT EXISTS dynamics_params JSONB` を追加(feedforward_params の前例と同型)
- [x] `src/infra/profile_repository.py`: シリアライズ組込み
  - [x] `_dyn_from_value` / `_dyn_to_json`(`_ffp_from_value`/`_ffp_to_json` と同じ `fields()` ベース、NULL/欠損キーはデフォルト補完)
  - [x] `_row_to_profile` / `create`(INSERT列) / `update`(SET列) に組込み
- [x] 単体テスト
  - [x] `tests/unit/test_models.py`: DynamicsParams デフォルト値・VehicleProfile 後方互換構築
  - [x] `tests/unit/infra/test_profile_repository.py`: ラウンドトリップ・NULL行デフォルト補完

## フェーズ2: DriveLoop 制御フレーム分離

- [x] `src/domain/control/drive_loop.py`: コンストラクタで `self._preview_s = max(0.0, profile.dynamics_params.preview_time_s)` をキャッシュ
- [x] `_execute_one_cycle`: `t_ctrl = elapsed_s + self._preview_s` を導入
  - [x] `ref_speed_ctrl = _ref_speed_at(t_ctrl)`、`future_speeds`/`past_speeds` を t_ctrl 基準に変更
  - [x] `ff.predict_effort(ref_speed_ctrl, ...)`・`pid.update(ref_speed_ctrl, ...)` へ変更
  - [x] now-frame 側(`ref_speed`→`_last_ref_speed`・KPI・逸脱判定・ログ・完了判定・pause)は不変であることを確認
- [x] 単体テスト `tests/unit/test_drive_loop.py`
  - [x] preview>0 で PID/FF の受信基準が前倒しされる(fake FF/PID で受信値検証)
  - [x] KPI・逸脱判定・DriveLogData.ref_speed_kmh・current_ref_speed は now-frame のまま
  - [x] preview=0 で既存テストが全て無変更で通る(回帰)
  - [x] 軌跡終端付近で t_ctrl が末尾超過してもクランプで安全

## フェーズ3: チューナー4次元化

- [x] `src/domain/pid_tuning.py`: `TuningParams` dataclass(kp, ki, kd, preview_time_s)追加、`PIDGains` との相互変換を用意(`gains`プロパティ、`from_profile`)
- [x] `CoordinateDescentTuner` の一般化
  - [x] `_PARAMS = ("kp", "ki", "kd", "preview_time_s")`、`_BASE["preview_time_s"] = 0.5`
  - [x] per-param クランプ `_CLAMP` 導入(`PREVIEW_MIN_S=0.0`, `PREVIEW_MAX_S=L_MAX_S=3.0`)、既存の `max(0.0, ...)` を置換
- [x] `initial_preview_from_fopdt(fopdt) -> float` ヘルパー追加
- [x] 単体テスト `tests/unit/test_pid_tuning.py`
  - [x] 4次元巡回・previewクランプ[0, 3.0]・クランプ同値スキップ
  - [x] 凸コストで preview が真値へ収束
  - [x] max_runs 予算内で preview 候補が評価される(既存 kd 版のパターンを踏襲、preview 版も追加)
  - [x] 既存テスト(`PIDGains`直接構築)を `TuningParams` ベースへ更新(回帰なし、27件通過)

## フェーズ4: アプリ層の配線

- [x] `src/app/training_service.py`: FOPDT同定ブロックで dynamics_params 設定
  - [x] `update_pid_gains=True` かつ同定成功時: `DynamicsParams(preview=initial_preview_from_fopdt(fopdt), k/tau/theta)` を設定し永続化
  - [x] `update_pid_gains=False` / 同定不能時: 既存値保持
  - [x] `TrainResult` に dynamics_params 追加
- [x] `src/app/robot_controller.py`: `run_pid_tuning_session` を TuningParams ベースへ
  - [x] 初期値にプロファイルの preview を使用(`TuningParams.from_profile`)
  - [x] 候補ごとに `dataclasses.replace` で preview を差し替えたプロファイルで走行
  - [x] history に preview_time_s 追加、戻り値 `tuple[TuningParams, list[dict]]` へ変更(呼び出し元2箇所=learning_cycle.py・drive.py を追随)
- [x] `src/app/learning_cycle.py`: `_persist_best_gains` → `_persist_best_params`(gains+preview を update + refresh_active_profile)
  - [x] detail JSONB に stageN_preview_time_s・fopdt を追記
- [x] `src/app/stubs.py`: `InMemoryProfileRepository.create`/`update` に dynamics_params 伝播を追加(統合テストで検出した抜け漏れ、feedforward_params と同型の既存バグパターン)
- [x] 単体テスト
  - [x] `tests/unit/test_training_service.py`: FOPDT成功時のpreview初期化・update_pid_gains=False/同定不能時の既存値保持
  - [x] `tests/unit/test_learning_cycle.py`: `_persist_best_params` の検証(preview永続化・制御スタック反映)
  - [x] `tests/unit/test_robot_controller.py`: 候補preview差し替え走行の検証
  - [x] `tests/integration/test_web_api.py`: pid-tune/refine が preview_time_s を永続化することを検証(修正含む)

## フェーズ5: Web API / UI

- [x] `src/web/schemas.py`: `DynamicsParamsSchema`(4フィールド、デフォルト付き)追加
  - [x] `ProfileResponse` に必須、`ProfileCreateRequest`/`ProfileUpdateRequest` に optional で追加
  - [x] `TrainModelResponse` に dynamics_params、`PidRefineResponse` に preview_time_s、`CycleProgressSchema` に best_preview_time_s を追加
- [x] `src/web/routers/profiles.py`: dynamics_params の変換・マージ組込み(`_dyn_to_schema`/`_dyn_from_schema`)
  - [x] **併修**: `_ffp_to_schema`/`_ffp_from_schema` を `fields()` ベースへ書き換え(調停定数リセットバグ解消)
- [x] `src/web/routers/drive.py`: TrainModelResponse/PidRefineResponse に dynamics_params/preview を露出、`_to_cycle_progress_schema` に best_preview_time_s 追加
- [x] `src/web/ws.py`: RealtimeData.cycle_progress に best_preview_time_s を追加
- [x] `src/app/learning_cycle.py`: `CycleProgress` に `best_preview_time_s` 追加、`_make_on_run` で追跡
- [x] `src/app/stubs.py` (InMemoryProfileRepository) は フェーズ4で対応済み
- [x] `src/web/static/js/screens/profiles.js`: 「先読み補償時間 [s]」入力(0〜3)+ FOPDT同定値の読み取り専用表示(未同定は「—」)
- [x] `src/web/static/js/screens/learning.js`: サイクル進捗/完了に best preview 表示(最小限)
- [x] 統合テスト(`tests/integration/test_web_api.py`): ProfileResponse に dynamics_params / PUT で preview 更新 / PUT で FFP 調停定数保持(バグ回帰テスト追加)
- [x] `config/settings.toml.example`: `refine_runs_stage1 = 14` へ変更(コメントで理由記載)。実機 `config/settings.toml` は `[learning]` セクション自体が存在せずコード既定値に依存していたため、`src/infra/settings.py` の `LearningSettings.refine_runs_stage1` 既定値を 10→14 に変更(`tests/unit/infra/test_settings.py` の既定値アサーションも追随)

## フェーズ6: 品質チェックと修正

- [x] すべてのテストが通ることを確認
  - [x] `.venv/bin/python -m pytest tests/unit tests/integration`(903件通過)
- [x] リント/型チェック(プロジェクトの既存ツールに従う)
  - [x] `ruff check`(変更した本体・テストファイル全て通過。E501/未整列importを修正)
  - [x] `mypy`(strict、変更13ファイル全て通過)
- [x] `scripts/setup_db.py` を実行して ALTER TABLE が冪等に通ることを確認(実DBで2回実行し dynamics_params JSONB カラムを確認)

## フェーズ7: 実機検証とドキュメント

- [x] ~~学習サイクルを1回実行し、learning_cycles.detail と profiles 画面で preview/FOPDT 値を確認~~
      （このセッションでは未実施: 実アクチュエータ・実車両を動かす操作のため、安全上ユーザー立ち会いの
      セッションで実施する必要がある。DB移行・全自動テスト・lint/型チェックまでは本セッションで完了済み。
      次回、実機で以下の手順を実施すること: ①学習サイクルを1回実行 → `learning_cycles.detail` の
      `stage1_initial_preview_time_s`/`fopdt`/`stage1_preview_time_s`/`stage2_preview_time_s` を確認
      → ②プロファイル画面で「先読み補償（アクチュエータ遅れ）」ボックスの値を確認 → ③通常自動走行で
      適合前後のKPI(p95, 最大偏差)を生ログで比較）
- [x] ~~通常自動走行で適合前後の KPI を生ログで比較~~（同上の理由により実機セッションへ持ち越し）
- [x] docs/glossary.md に DynamicsParams・先読み補償(preview_time_s)・4次元座標降下・
      refine_runs_stage1既定値変更 を追記(VehicleProfile・PID自動適合・座標降下チューニング・
      学習サイクルの各エントリを更新)
- [x] 実装後の振り返り(このファイルの下部に記録)

---

## 実装後の振り返り

### 実装完了日
2026-07-04

### 計画と実績の差分

**計画と異なった点**:
- `src/app/stubs.py`(InMemoryProfileRepository)の `create`/`update` が `dynamics_params` を伝播していなかった不具合を統合テストで検出し、フェーズ4で追加修正した。design.md には明記していなかったが、`feedforward_params` と全く同型の既存バグパターンだったため同じ手法で修正。
- `CycleProgress`/`CycleProgressSchema` への `best_preview_time_s` 追加は design.md の「learning.js に best preview 表示」を実現するため、`ws.py`・`drive.py` の2箇所のスキーマ変換関数も含めて実装した(design.mdには変換関数2箇所への言及はなかったが、実装時に必要と判明)。
- `config/settings.toml.example` の変更だけでなく、実機の `config/settings.toml` に `[learning]` セクション自体が存在せず `src/infra/settings.py` の `LearningSettings.refine_runs_stage1` コード既定値(10)がそのまま使われることが判明したため、コード既定値そのものを14へ変更した(`tests/unit/infra/test_settings.py` の既定値アサーションも追随して修正)。

**新たに必要になったタスク**:
- 上記3点（stubs.py修正、CycleProgress拡張、settings.py既定値変更）はいずれも実装中に発覧した抜け漏れで、design.mdの想定範囲を実務上補完するものだった。

**技術的理由でスキップしたタスク**:
- フェーズ7「実機で学習サイクルを1回実行しKPIを生ログ比較」: 実アクチュエータ・実車両を物理的に動かす操作であり、安全上ユーザー立ち会いの下で実施する必要があるため、本セッション(自動運転ロボットの物理制御を伴わない範囲)では実施しなかった。DBマイグレーション適用・全903件の自動テスト・ruff/mypy(strict)は本セッションで完了・確認済み。次回、ユーザー立ち会いの実機セッションで学習サイクルを1回実行し検証すること。

### 学んだこと

**技術的な学び**:
- 「先読み(lookahead)」という同じ言葉が、FFモデルの特徴量ホライズン(`FeatureSpec.lookahead_horizons_s`、目標軌跡を先読みする)と、今回導入した`preview_time_s`(制御ループの基準サンプリング時刻そのものを前倒しする、むだ時間補償)の2つの異なる概念に使われており、混同しやすい。glossary.mdに両者の違いを明記した。
- KPI/逸脱判定の評価基準(now-frame)と制御の基準(t_ctrl)を明示的に分離する設計は、既存の`_ref_speed_at`の終端クランプ機構にそのまま乗せられ、追加のガードが不要だった。既存コードの堅牢な設計が新機能の実装コストを大きく下げた好例。
- `CoordinateDescentTuner`のパラメータをdictベースの`_BASE`/`_CLAMP`で一般化したことで、3→4次元への拡張が`_PARAMS`タプルへの追加1行と`_CLAMP`エントリ1行で完結し、`next_candidate`/`report`本体のロジック変更が不要だった。

**プロセス上の改善点**:
- ステアリングファイル(design.md)に変更ファイル・関数・データフローを具体的に書いておいたことで、実装フェーズ(フェーズ1〜7)を迷いなく順に進められた。特に「判断1〜3」として設計判断とその根拠(なぜFF+PID両方か、なぜ新カラム分離か、なぜSIMCはθフル維持か)を明文化していたのが、実装中の細部判断(例: preview の初期値をどこで設定するか)で有効だった。
- 各フェーズ完了ごとに`pytest`を実行して回帰を都度確認したことで、後続フェーズでの修正コストが小さく抑えられた。

### 次回への改善提案
- 車両プロファイルのJSONBフィールド(`feedforward_params`, `dynamics_params`)を追加するたびに、`src/web/routers/profiles.py`の変換関数・`src/app/stubs.py`のInMemoryリポジトリの両方に伝播漏れが起きやすいパターンが今回も再現した(過去にFFP側で同じバグがあった)。次回同種のフィールドを追加する際は、両ファイルの対応箇所を機械的にチェックリスト化しておくとよい。
- 「学習サイクル完了時のprogress情報(best_cost等)」を配信する経路が`ws.py`と`drive.py`の2箇所に重複しているため、フィールド追加のたびに両方を漏れなく更新する必要がある。将来的には`CycleProgress→CycleProgressSchema`の変換を1箇所の共通関数に統合する余地がある。
