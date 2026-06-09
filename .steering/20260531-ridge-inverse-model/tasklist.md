# タスクリスト: 連続走行ログからの Ridge 逆モデル学習

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

### タスクスキップが許可される唯一のケース
- 実装方針の変更により、機能自体が不要になった
- アーキテクチャ変更により、別の実装方法に置き換わった
- 依存関係の変更により、タスクが実行不可能になった

スキップ時は必ず理由を明記:
```markdown
- [x] ~~タスク名~~（実装方針変更により不要: 具体的な技術的理由）
```

---

## フェーズ1: 依存追加

- [x] `pyproject.toml` に `scikit-learn>=1.4` を追加
- [x] `.venv` に scikit-learn をインストール（1.9.0）

## フェーズ2: ドメイン層

- [x] `src/domain/model_training.py` を新規作成
  - [x] ホライズン定数（`LOOKAHEAD_HORIZONS_S`・`REGIME_HORIZON_S`）
  - [x] `build_feature_row`（推論用）/ `_build_feature_matrix`（学習用）先読み7特徴量構築
  - [x] `train_inverse_model(logs, profile, output_dir)` 学習・保存・メトリクス
  - [x] ブレーキデッドバンド・速度クリップ・dt推定/オフセット算出・末尾除外・セッション分割・レジーム分割（`Δv@1.0`符号）
  - [x] データ不足時 `LearningDataError`
- [x] `src/domain/control/feedforward.py` を先読み Ridge 逆モデルに置換
  - [x] `load_model()` を dict pkl 対応（`ridge_inverse_lookahead` 検証）
  - [x] `predict(v0, future_speeds)` を差分特徴量構築 + レジーム分岐 + クランプに変更
  - [x] `RegularGridInterpolator` 廃止
- [x] `src/domain/control/drive_loop.py` を先読みサンプリング対応
  - [x] `_ref_speed_at(t_s)` を追加（既存補間ロジック再利用・末尾クランプ）
  - [x] `_execute_one_cycle` で `ff.predict(v0, future_speeds)` を呼ぶ
- [x] `src/domain/learning_drive.py` の grid 学習を整理
  - [x] `train_model()`・`_fill_nan_nearest()`・`griddata`/`NearestNDInterpolator` import を削除
  - [x] `generate_patterns`/`run_pattern` は残す

## フェーズ3: インフラ層

- [x] `src/infra/session_repository.py` に `list_logs_for_training()` を追加
  - [x] profile_id でセッション絞り込み、session 単位 timestamp 昇順、`_row_to_log` 再利用

## フェーズ4: Web層

- [x] `src/web/schemas.py` に `TrainModelRequest` / `TrainModelResponse` を追加
- [x] `src/web/deps.py`・`stubs.py` の `SessionRepoProtocol`/stub に `list_logs_for_training` を追加
- [x] `src/web/routers/drive.py` に `POST /api/v1/drive/learning/train` を追加
  - [x] ログ取得→学習→pkl保存→`model_path`更新→メトリクス返却
  - [x] ログ不足 422・プロファイル無し 404

## フェーズ5: App層（制御反映の配線）

- [x] `src/app/factory.py` で `FeedforwardController` を生成し注入（stub も同様）
- [x] `src/app/robot_controller.py` `select_profile()` で `model_path` からロード（失敗は警告）

## フェーズ6: フロントエンド

- [x] `src/web/static/js/screens/learning.js` に「モデル学習」ボタンと結果表示を追加

## フェーズ7: テスト

- [x] `tests/unit/test_model_training.py` を新規作成
- [x] `tests/unit/test_feedforward.py` を先読み Ridge 対応に更新
- [x] `tests/unit/test_drive_loop.py` を先読み `predict` I/F に更新
- [x] `tests/unit/test_learning_drive.py` から grid `train_model` テストを除去
- [x] `tests/unit/test_web_drive.py` に `/learning/train` テストを追加
- [x] `tests/unit/test_robot_controller.py` に `select_profile` ロード検証を追加

## フェーズ8: 品質チェックと修正

- [x] すべてのユニットテストが通ることを確認
  - [x] `.venv/bin/pytest tests/unit/` → 401 passed
- [x] リントエラーがないことを確認（本変更分は clean）
  - [x] `.venv/bin/ruff check`（変更ファイルは clean。`deps.py` の未使用 import も整理）
  - [x] `.venv/bin/ruff format --check`（新規/変更ファイルは clean）
- [x] 型エラーがないことを確認
  - [x] `.venv/bin/mypy`（新規/変更モジュール 9 ファイル: issues なし）

> 注: `app.py:56`・`calibration.py:123`・`robot_controller.py:555` の mypy エラー、
> 一部 test の lint、プロジェクト全体の format 差分は**本作業着手前から存在する未コミット差分/既存事項**であり、
> 本機能のスコープ外として未修整。

## フェーズ9: ドキュメント更新

- [x] 実装後の振り返り（このファイルの下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-06-03

### 計画と実績の差分

**計画と異なった点**:
- 当初案の特徴量 `[a, speed, a², speed², a·speed]`（瞬時加速度ベース）から、ユーザー指摘により
  **先読み（preview）型特徴量 `[v0, Δv@{0.5,1,2,3}, v0², Δv@1·v0]`** へ設計変更。人間の運転（数秒先を見て操作）を模す。
- 学習データは「実車速の軌跡＝意図軌跡」とみなす純粋FF方式に決定。`speed` と `target(t)` を `v0` に統合。
- FF `predict` の I/F を `(ref_speed, ref_accel)` → `(v0, future_speeds)` に変更。これに伴い `DriveLoop` に
  先読みサンプリング `_ref_speed_at(t)` を追加（`_get_ref_speed_and_accel` を置換）。

**新たに必要になったタスク**:
- `DriveLoop` の先読み対応（当初の plan には無く、predict I/F 変更で必須化）。
- `select_profile`/factory/stub での `FeedforwardController` 生成・モデルロード配線が**そもそも未実装**だったため補完。
- `SessionRepoProtocol`/`InMemorySessionRepository` への `list_logs_for_training` 追加。

### 学んだこと

**技術的な学び**:
- FF は基準軌跡のみの関数（純粋FF）にすると、手動運転ログのみで学習可能・学習/推論の特徴量意味が一致する。
  追従誤差補正は PID に分離するのが理論的にもクリーン。
- 先読み特徴量はセッション境界をまたいではいけない。`session_id` でグループ化してから特徴量構築する必要がある
  （`list_logs_for_training` は session 単位で順序化、ドメインで再グループ化）。
- `MagicMock(spec=...)` は属性アクセスを許すが、反復対象（`ff.horizons`）は実値を設定しないと iterate で失敗する。

**プロセス上の改善点**:
- 特徴量設計をユーザーと AskUserQuestion で確定してから実装に入ったことで手戻りを防げた。
- 着手前の未コミット差分（calibration.py 等）が品質チェックのノイズになった。stash でベースライン比較し、
  本変更由来のエラーを切り分けて判断できた。

### 次回への改善提案
- DB を使った integration テスト（`list_logs_for_training` の JOIN クエリ）を追加すると、実 PostgreSQL での
  セッション横断取得を検証できる（今回はユニット + 手動確認に留めた）。
- 学習メトリクスは in-sample。将来は時系列 split によるホールドアウト評価を検討。
- 実機ログでの閉ループ精度検証（PRD KPI: 速度偏差 95%tile ≤ 0.2km/h）を別ステアリングで実施。
