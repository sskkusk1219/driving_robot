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

---

## フェーズ1: 減速 R² の原因調査（先行・機能1）

- [x] `scripts/analyze_decel_fit.py` を作成（読み取り専用）
  - [x] DB の drive_logs を profile_id / session_id / 期間で取得
  - [x] `model_training` の特徴量生成（`_group_by_session`/`_build_feature_matrix`/`_estimate_offsets`）を流用
  - [x] 減速サンプルを coast(brake<deadband) と brake≥deadband に分離し個別 Ridge で R² 比較
  - [x] brake_opening ↔ 実減速率(dv) の相関・むだ時間ラグを数値化して出力
- [x] 2026/6/30 のログで実行し、データ起因/モデル起因を切り分け
- [x] 判断（機能4 着手要否）を design.md に追記（結論: データ起因が支配的・機能4 は保留）

## フェーズ2: 全車速域 加速スイープ（機能2）

- [x] `LearningLoopConfig` に全域加速用タイムアウト（`accel_full_range_timeout_s`）を追加
- [x] `learning_loop.py` `_advance_drive_accel` の離脱を「6秒固定」から「cap 到達主導」に変更
  - [x] 全域スイープ種別/COAST_DOWN ではプラトー早期離脱を抑止（cap/overspeed/timeout を離脱条件に）
  - [x] max_speed 超過スキップ・ガバナ（max_decel_g）・非常停止の安全は維持
- [x] `learning_drive.py` の加速開度スイープを「全域到達する数段（30/50/70/100%）」に再構成
- [x] 「踏む→戻す→低速キープ」反復が解消される構成になっていることを確認（ACCEL_SWEEP は cap まで加速）

## フェーズ3: 定常ブレーキ計測（機能3）

- [x] `models/learning_drive.py` に `PatternKind.BRAKE_HOLD` を追加
- [x] `learning_loop.py` に BRAKE_HOLD フェーズを実装
  - [x] `_command_openings` に accel=0・brake=固定（ランプ後一定保持）の分岐を追加
  - [x] `_advance` に「低速まで定常減速→次パターン」遷移を追加（加速プラトー保持と対称）
  - [x] `_update_governor` の DRIVE_BRAKE/BRAKE_HOLD 経路で上限G厳守を確認
- [x] `generate_patterns` でブレーキ開度を数段スイープ（10/20/30/40%、`max_brake_opening` クランプ）
- [x] 学習走行 ≤30分の本数/保持時間見積りを design.md に記載

## フェーズ4: 品質チェックと修正

- [x] 単体テスト追加・更新
  - [x] `tests/unit/test_learning_drive.py`: ACCEL_SWEEP 段・BRAKE_HOLD 段の生成、開度上限クランプ
  - [x] 学習ループのフェーズ遷移（cap 到達離脱・BRAKE_HOLD 保持→遷移・G 抑制）
- [x] すべてのテストが通ることを確認（701 passed, 2 deselected）
  - [x] `.venv/bin/pytest -m "not hardware"`
- [x] リントエラーがないことを確認
  - [x] `.venv/bin/ruff check`
- [x] 型エラーがないことを確認
  - [x] `.venv/bin/mypy`（変更ファイル）

## フェーズ5: 実機検証（Playwright MCP）

- [x] ユーザーがサーバ再起動後、学習走行を Playwright で実行（session d0fdd36a、2026-06-30 21:18）
- [x] 学習走行が 0〜0.9×max_speed を掃き、30分以内に完走することを確認
      → 実車速 **0〜97.9km/h** をカバー（旧 ~30km/h 頭打ちを解消）、走行 **約6分**で完走
- [x] 上限G・最高車速の逸脱がないことを確認
      → 最高車速 97.9<100 OK。減速Gはブレーキ食い付き時の過渡で平滑 0.57G まで瞬間的に超過
        （ブレーキ標本の 4% が >0.30G、ガバナが持続減速は引き戻す）。**過渡オーバーシュートは既存ガバナの
        特性で本変更による回帰ではない**（ガバナロジックは BRAKE_HOLD 経路追加のみ）。要別途改善検討。
- [x] 再 train 後の結果で 加速・減速 R² を確認
      → **新セッション単体: 加速 R² 0.851（≥0.8 達成）/ 減速 R² 0.178（未達）**。
        全セッション集計（旧データ込み）は加速 0.581 / 減速 0.148。
- [x] R² 未達かつ機能1 がモデル要因と判断した場合 → フェーズ6
      → **減速のみ未達**。新データで coast 比率 33.5%→15.9%、brake↔減速率相関 0.330→0.503 と改善した
        が、brake 単独 fit は依然 R²=-0.434。**「定常データ追加だけでは減速 R²≥0.8 に届かない」が実証**
        され、フェーズ6（機能4 モデル改修）の着手条件が成立。**ユーザー方針確認待ち**。

## フェーズ6: ブレーキモデル改修（条件付き・機能4）

> フェーズ1 の調査が「データ追加だけでは不足」と示し、かつフェーズ5 で R² 未達の場合のみ。
> 着手前にユーザーへ方針確認すること。

- [x] 改修方針をユーザーと合意 → **多項式Ridge(2次)＋標準化＋入力クリップ。coast はブレーキ学習から除外**。
      実機検証（session d0fdd36a）で減速 R²0.178 と未達、GBM 上限 0.887・coast 除外実制動 MAE≈1.4 を
      読み取り専用アブレーションで確認し方針確定（計画: `~/.claude/plans/snug-juggling-squirrel.md`）。
- [x] `model_training.py` を改修（多項式2次＋標準化＋Ridge の Pipeline `_make_estimator`・ブレーキ
      coast 除外 `brake_raw>=db`・`speed_clip_max` を payload 保存・`MODEL_TYPE="poly_inverse_lookahead"`）
- [x] `feedforward.py` を改修（推論時の入力クリップ＝v0>cm は軌跡平行移動で dv 保持・型ヒントを
      `_Regressor` Protocol 化・新 MODEL_TYPE 受理・停車短絡/負クランプは維持）
- [x] テスト更新・ruff・mypy（705 passed / ruff OK / mypy OK）。外挿 v0=120 減速予見 −4.02（有界）。
- [x] 実機再学習で R²<0.8 を検知 → **真因は「全18セッション集計（旧データ・160km/h 異常値混在）」**と
      「coast 除外で減速 R²<0.8」の2点と判明。ユーザー方針で **①学習は最新の学習走行セッションのみ
      （`latest_learning_session_id` を train 既定対象に・session_ids 指定時はそれ優先）②coast込みに復帰**
      へ修正。`src/web/routers/drive.py`・`session_repository.py`・`deps.py`・`stubs.py` を改修。
      → 最新セッションのみ＋coast込みで **加速 R²0.923 / 減速 R²0.915**（両 ≥0.8 達成）、speed_clip_max=97.8。
- [x] 実機再学習で R²≥0.8 を確認（poly2 モデル: 加速 R²0.916/MAE1.68・減速 R²0.925/MAE1.43）

## フェーズ6-2: 精度目標 MAE≤0.5 / RMSE≤1.0 / R²≥0.8（機能4 続き・2026-07-02）

> ユーザー新目標。アブレーションで poly 微調整（MAE~1.3-1.5）では届かず、
> **GBM＋過去Δv特徴**のみ達成可能と実測 → 計画承認のうえ実装。

- [x] `model_training.py`: 推定器を**単調制約付き HistGradientBoosting** に変更
      （dv_1.0 に accel:+1 / brake:-1 の単調制約）。特徴量に**過去方向Δv**（0.5s/1.0s 前、
      `PAST_HORIZONS_S`）を追加し 9 特徴に（ランプ過渡/定常保持の識別）。
      `MODEL_TYPE="gbm_inverse_lookahead"`・payload に past_horizons 追加。
- [x] `feedforward.py`: `past_horizons` プロパティ・`predict_effort(v0, future, past)` に拡張。
      入力クリップは平行移動シフトを past にも適用。
- [x] `drive_loop.py`: past_speeds を `_ref_speed_at(elapsed−h)` で構築し FF へ（範囲外は端点クランプ）。
- [x] テスト更新（build_feature_row 9次元・単調制約・past_horizons・全 predict_effort 呼び出し）
      → **763 passed / ruff OK / mypy OK**
- [x] オフライン検証（実 train 経路・最新セッション）:
      **加速 MAE0.295 / RMSE0.755 / R²0.995・減速 MAE0.218 / RMSE0.464 / R²0.997 — 全目標達成**。
      推論 3.5ms/回（50ms 周期の 7%）、外挿 v0=120 有界。
- [x] ドキュメント更新（functional-design / architecture / glossary）
- [x] 実機: 学習走行→再学習で目標達成（in-sample）を確認。**しかし自動走行（06_WLTP_ExHi）で追従破綻**
      → フェーズ6-3 で対応（GBM は off-manifold の巡航域で定数飽和し閉ループ不成立と判明）

## フェーズ6-3: 推定器を poly2 Ridge に復帰（GBM 閉ループ破綻対応・2026-07-02）

> 実走行 06_WLTP_ExHi（session 4c88b2dd）で偏差 MAE 18.3km/h・最大36.8、ref74 に対し 105.8km/h まで
> 過加速。原因分析で「GBM は巡航域（dv=0）で速度によらず一定 7.3% を返す＝off-manifold で定数飽和」
> と確定（詳細・比較表は design.md フェーズ6-3 参照）。ユーザー決定: **Ridge 復帰のみ（最小変更）**。
> 過去Δv 9特徴・最新セッション学習・speed_clip_max クリップ・drive_loop の past_speeds は維持。
> **実装は design.md フェーズ6-3 の実装設計に従うこと。**

- [x] `model_training.py`: `_make_estimator()` を poly2 Pipeline に復帰
      （`PolynomialFeatures(2, include_bias=False)` → `StandardScaler` → `Ridge(alpha=1.0)`。
      `dv_monotonic` 引数と HistGradientBoostingRegressor import を削除、Pipeline 系 import 復活、
      `_metrics` 型注釈を `Pipeline` に）
- [x] `model_training.py`: `MODEL_TYPE = "poly_past_inverse_lookahead"` に変更
      （旧7特徴 poly と GBM の両 pkl を確実に弾く新識別子。9特徴・PAST_HORIZONS_S・payload 構成は不変）
- [x] `model_training.py`: docstring/コメントを poly 表現に戻し、GBM 不採用の理由
      （off-manifold の巡航域で定数飽和→閉ループ破綻）をコメントで残す
- [x] `feedforward.py`: docstring のみ更新（ロジック変更なし）
- [x] テスト更新: `test_estimator_is_monotonic_gbm` → `test_estimator_is_polynomial_pipeline` に戻す
      （`test_feedforward.py` の `make_model_file` docstring 内 model_type 表記も修正。他は変更不要）
- [x] 品質チェック: `.venv/bin/pytest -m "not hardware"`（763 passed）/ `.venv/bin/ruff check`（All
      checks passed）/ `.venv/bin/mypy`（Success: no issues found）
- [x] オフライン検証: 最新学習セッション（b23a2fd1、2515行）で実 `train_inverse_model` 経路の再学習
      → 加速 R²=0.895・MAE=2.895、減速 R²=0.904・MAE=1.397（poly 水準に復帰）。巡航予測（dv=0, past
      dv=0）は 60〜100km/h で **10.90%→12.70%→10.45% の滑らかな曲線**（GBM の一定7.3%飽和が解消）
- [x] ドキュメント更新: functional-design（FeedforwardController構造・predict_effortシグネチャに
      past_speeds追加も反映）/ glossary / architecture の推定器記述を poly に戻し、GBM 不採用の経緯を残す
- [ ] 実機検証: ユーザーがサーバ再起動 → 学習運転1回（新 MODEL_TYPE のため再学習必須）→
      06_WLTP_ExHi を実走し追従（偏差・過加速なし）を確認

### 残課題（次フェーズ候補・スコープ外）

- 巡航データの欠如（両モデル共通）: ACCEL_SWEEP に「中間開度の巡航保持」を追加し (v0, dv≈0) の
  定常標本を採る。120km/h 超の巡航予測（poly で ~1% と過小）の改善にも必要。
- MAE≤0.5 目標は GBM でしか出ないが閉ループ追従を優先して poly 採用。巡航データ追加後に GBM 再評価の余地。

## フェーズ7: ドキュメント更新

- [x] `docs/functional-design.md` の学習フロー記述を改修後の構成に更新（glossary.md も更新）
- [x] 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-06-30〜2026-07-02（フェーズ1〜7 のコード/ドキュメント実装、フェーズ6-2 の GBM 化とその閉ループ
破綻対応（フェーズ6-3・poly 復帰）まで完了。フェーズ5 の実機検証は実施済み＝学習走行が 0〜97.9km/h
を約6分で完走。フェーズ6-3 もコード実装・全品質チェック・オフライン検証まで完了。残るは新コード
反映後の自動走行 閉ループ確認のみ＝ユーザーのサーバ再起動待ち）

### 計画と実績の差分

**計画と異なった点**:
- 設計では「ACCEL_SWEEP（既存 COMBINED の改修）」としていたが、COMBINED を廃止し ACCEL_SWEEP と
  BRAKE_HOLD の2種別に分割した。COMBINED（加速→惰行→減速を1サイクル）は加速サンプルと減速サンプルが
  混在していたが、ACCEL_SWEEP（純加速→停車復帰）/ BRAKE_HOLD（cap→定常ブレーキ）に分けることで
  片軸ずつ清浄なサンプルになり、調査結果（減速は定常データ欠如が主因）に直接対応できた。
- DRIVE_ACCEL のプラトー早期離脱ロジック（accel_plateau_*・accel_hold_min_s）を完全に撤去し、
  cap 到達 / overspeed / `accel_full_range_timeout_s` の3条件に単純化した。これに伴い旧 config
  フィールド（accel_hold_timeout_s 等・coast_to_brake_* ・coast_settle_*）と `_sweep` 系の
  COMBINED 用パラメータ（max_combined_patterns 等）を削除。
- フェーズ1 の調査では「定常データ追加で届く」と見込んだが、フェーズ5 実機検証で BRAKE_HOLD 追加後も
  減速 R²0.178 と未達。読み取り専用アブレーションで **真因は「定常データ欠如」ではなく「線形モデルが
  非線形を捉えられないこと」**と判明（GBM 上限 0.887・多項式2次で 0.818）。→ フェーズ6（機能4）の着手
  条件が成立し、ユーザー方針確認のうえ **多項式Ridge(2次)＋入力クリップ＋coast 除外**で実装した。

**新たに必要になったタスク**:
- 既存テスト（test_learning_loop.py / test_learning_drive.py / test_robot_controller.py）の
  COMBINED 依存の全面書き換え。COMBINED 用の挙動テストを ACCEL_SWEEP/BRAKE_HOLD/COAST_DOWN の
  cap 到達・保持→遷移・ガバナ抑制テストへ再構成した。

**技術的理由でスキップしたタスク**:
- なし（フェーズ6 も条件成立により実装した）。
- フェーズ6-2（GBM＋過去Δv・MAE≤0.5目標）は in-sample では全目標達成したが、実走行で閉ループが
  破綻したため**フェーズ6-3 で推定器のみ poly2 Ridge に巻き戻し**（過去Δv 特徴量・最新セッション
  学習・入力クリップは維持）。「実装方針の変更により不要になった」に該当し、GBM 版は不採用として
  design.md に経緯を記録した（アーキテクチャ変更というより、実測で反証されたため撤回）。

### 学んだこと

**技術的な学び**:
- 減速 R²≈0.1 の主因は当初「定常ブレーキデータの欠如」と推定したが、実機で BRAKE_HOLD を追加しても
  未達。アブレーションで **真因は「線形モデルの表現力不足」**と判明: 同じ7次元入力でも完全2次多項式
  ＋標準化にすると 0.15→0.82、GBM では 0.89。「2次項あり＝非線形」でも v0²・dv1·v0 だけでは制動の
  不感帯非線形には不足で、dv の高次・交互作用項が要る。
- 推定器ごとに**外挿挙動が全く異なる**（線形=発散、多項式=暴れ、木=飽和）。外挿は学習できないため
  入力クリップで学習端に飽和させ、残差を FF+PID・包絡線ガバナで吸収する設計が要。v0>cm では各点
  独立クリップだと near-horizon の dv が潰れレジーム誤判定 → 軌跡平行移動で dv を保つのが正解。
- coast 除外は R²（分散説明率）を下げるが実制動 MAE は同等（1.39 vs 1.41）。R² は分母（標本集合）に
  依存するため、指標は実制動 MAE/RMSE を主に見るべき。
- 加速が ~30km/h で頭打ちだった主因は「低開度の早期プラトー離脱」と「6秒固定 timeout」。cap 到達
  主導に変え、開度を全域到達する数段（max の 30/50/70/100%）に再構成することで高速域を埋められる。

- **in-sample/CV の指標だけでは実走行の性能を保証できない**（フェーズ6-2→6-3 の最大の教訓）。GBM は
  MAE≤0.5/R²≥0.8 を大幅達成しつつ、学習パターン外の巡航域（off-manifold）で予測が定数に飽和し、
  実走行モード（WLTP-ExHi）で偏差 MAE 18.3km/h・最大105.8km/h の過加速という閉ループ破綻を招いた。
  poly Ridge は同じ外挿域でも滑らかに補間するため実用に耐えた。**指標最適化と閉ループ安定性は別軸**。
- 過去方向Δv 特徴量（ランプ過渡/定常保持の識別）は推定器を問わず有効な改善で、GBM→poly の巻き戻し後
  も維持した。

**プロセス上の改善点**:
- 読み取り専用の調査/アブレーションスクリプト（DB→特徴量→fit→R²/MAE）で、実装前に推定器・特徴量・
  外挿挙動を定量比較してから方針確定できた。重い改修を当て推量で始めず、データで裏付けて選べた。
- 一方で、そのアブレーションは学習データ内の CV に留まり、実走行軌跡（別分布）での検証を経ずに
  GBM を本番投入してしまった。**推定器変更は必ず実機の閉ループ走行で検証してから確定させる**べきだった。

### 次回への改善提案
- 自動走行 閉ループ検証（新コード反映後）で FF 制動の実挙動を確認し、必要なら `POLY_DEGREE`・
  `RIDGE_ALPHA`・`accel_sweep_fracs`・`brake_hold_openings_pct`・各 timeout を調整する。
- 巡航データの欠如（フェーズ6-3 の残課題）に対応する: ACCEL_SWEEP に「中間開度の巡航保持」段を
  追加し (v0, dv≈0) の定常標本を採る。これで poly の 120km/h 超巡航予測（現状 ~1% と過小）も改善
  でき、将来 GBM 系を再評価する際の off-manifold の穴も埋まる。
- 今後モデル/推定器を変更する際は、in-sample・CV に加えて**実走行モードでの閉ループ追従**を
  必須の検証ステップとして tasklist に明記する。
- 減速 R² の受け入れ基準は coast 除外方針と整合させ、実制動 MAE/RMSE を主指標に据え直すと良い。
