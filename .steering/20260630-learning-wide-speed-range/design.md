# 設計書

## アーキテクチャ概要

学習運転は既存の「開ループ パターン実行（`LearningLoop`）→ 連続ログ記録 → Ridge 逆モデル学習
（`train_inverse_model`）」構成を維持し、**パターン生成と加速/減速の状態機械のみ**を改修する。
新規の制御コードやプラントモデルは追加しない。

```
LearningDriveManager.generate_patterns(profile)
   ├─ CREEP（解放）            … 既存
   ├─ CREEP_SETTLE            … 既存
   ├─ ACCEL_SWEEP（全域）★改修  … 数段の開度で 0→0.9×max_speed まで加速（cap 到達主導）
   ├─ BRAKE_HOLD（新）★追加     … 高速まで加速→固定ブレーキ保持で定常減速
   └─ COAST_DOWN              … 既存（エンジンブレーキ/走行抵抗カーブ）
        │
        ▼
LearningLoop（状態機械: DRIVE_ACCEL / COAST / DRIVE_BRAKE / BRAKE_HOLD★ / MEASURE）
        │  100ms 周期で (実車速, 開度) を drive_logs に連続記録
        ▼
train_inverse_model → accel/brake Ridge（R² を _metrics で算出）
```

## コンポーネント設計

### 1. 減速 R² 調査スクリプト `scripts/analyze_decel_fit.py`（先行・機能1）

**責務**:
- DB の drive_logs（対象セッション/プロファイル）を読み、`model_training` の特徴量生成
  （`_group_by_session`, `_build_feature_matrix`, `_estimate_offsets`）を流用して減速サンプルを構築。
- 減速側を coast(brake<deadband) と brake≥deadband に分離し、それぞれ単独で Ridge を fit して R² を比較。
- brake_opening ↔ 実減速率（dv）の散布・相関、speed 応答のむだ時間ラグを数値化して出力。

**実装の要点**:
- 読み取り専用（DB/モデルを書き換えない）。CLI 引数で profile_id / session_id / 期間を指定。
- 出力は標準出力のサマリ＋必要なら CSV。判断材料（データ起因 vs モデル起因）を明示。
- 既存の `train_inverse_model` のレジーム分割ロジックと整合させる（同じ閾値・特徴量）。

**調査結果（2026/6/30 実行 — `scripts/analyze_decel_fit.py`）**:

対象: profile `1230e6fc`（sample_0001, max_speed=100, max_decel_g=0.3）。直近セッション
`c80f2aad`（6/30 07:39 JST）単体と、全 36 セッション集計の両方で確認。

| 集団 | n | R² | 備考 |
|------|---|----|----|
| 全減速サンプル（現行モデル同条件） | 22714 | **0.155** | 要求記載の 0.096 と同水準の低さを再現 |
| coast(brake<deadband) のみ | 7612 (33.5%) | N/A | ラベル全 0・分散ゼロ（自明） |
| brake≥deadband のみ単独 fit | 15102 | **-0.430** | 平均以下。**分離だけでは改善しない** |

- **むだ時間ラグ**: brake↔実減速率の相互相関ピークはセッション中央値 **0 サンプル（≈0s）**、
  瞬時相関は各セッション中央値 **~0.5〜0.58**。→「ブレーキ開度↔減速率」の物理関係は瞬時に存在する。
- **dv_1.0（1秒先読み差分）との相関は 0.330** と弱い。現データの brake サンプルは全て
  「踏む→戻す」サイクル中の**過渡（ランプ）**で、固定開度を保持した**定常区間が皆無**。

**判断（機能4 着手要否）**: **データ起因が支配的**。加速側は定常プラトー保持があるため
R²≈0.587 まで上がるのに対し、減速側は定常保持が無く過渡だらけ・ラベルの 1/3 が 0 張り付き。
むだ時間がほぼ無く瞬時相関 ~0.5 ある以上、固定ブレーキ開度を保持した清浄な定常サンプル
（機能3 BRAKE_HOLD）を採れば、現行の線形＋2次特徴（v0², dv1·v0）でも表現可能と見込む。
→ **機能4（モデル改修）は保留**。機能2+3 のデータ改善後に実機 train で R² を再評価し、
なお R²<0.8 の場合のみユーザー確認のうえ機能4 に着手する。

### 2. 全車速域 加速スイープ（機能2）

**責務**（`learning_drive.py` / `learning_loop.py`）:
- 加速開度スイープを「低開度多数（先頭20本が2–40%）」から「全域到達する数段（例 max_accel の
  30/50/70/100%）」へ再構成。各段が cap（0.9×max_speed）まで加速し、速度全域 × 加速率を採取。
- `DRIVE_ACCEL` の離脱条件を「6秒固定打ち切り」から「cap 到達主導」に変更。

**実装の要点**:
- `LearningLoopConfig` に全域加速用タイムアウト（例 `accel_full_range_timeout_s`）を追加。ガバナ
  加速率（`max_decel_g × g_limit_frac`）から cap 到達所要時間を見積もり、それ＋マージンで設定。
- 全域スイープ種別（および COAST_DOWN）では低開度プラトーの早期離脱を抑止し、`accel_speed_cap`
  到達 or overspeed を主離脱条件にする（プラトーは安全弁としては残す）。
- 安全はそのまま維持: max_speed 超過スキップ（デバウンス）、包絡線ガバナ（max_decel_g）、
  過電流/CAN/アクチュエータ非常停止。加速にも引き続き G 上限を適用。

### 3. 定常ブレーキ計測フェーズ `PatternKind.BRAKE_HOLD`（機能3）

**責務**:
- `models/learning_drive.py` に `PatternKind.BRAKE_HOLD` を追加。
- `learning_loop.py` に保持フェーズを実装: cap 付近まで加速 → 固定ブレーキ開度をランプ後**一定保持**し、
  低速（停止しきい近傍）まで定常減速を記録 → 次のブレーキ開度（次パターン）へ。
- `generate_patterns` でブレーキ開度を数段スイープ（例 10/20/30/40%, `max_brake_opening` クランプ）。

**実装の要点**:
- 既存 `_advance_drive_accel`（加速プラトー保持）と対称な離脱判定を作る。`max_decel_g` ガバナ
  （`_update_governor` の DRIVE_BRAKE 経路）で上限G厳守。
- 加速→保持の遷移は `_enter_phase` を流用。BRAKE_HOLD は accel=0・brake=固定（ランプ後一定）の
  `_command_openings` 分岐を追加。
- ログは既存の連続記録経路（`ref_speed_kmh=None`）に乗る。学習側のレジーム分割（dv<0→brake）で
  定常区間が brake>0 サンプルとして寄与する。

### 4. ブレーキモデル改修（機能4・条件付き）

**責務**: 機能1が「データ追加だけでは R²≥0.8 に届かない」と示した場合のみ着手。
**実装の要点**: 候補は (a) ブレーキ不感帯→急制動の非線形に対応する特徴量追加、(b) coast/brake の
サブレジーム分割、(c) 過渡（ランプ）を除き定常区間優先のラベリング。**着手前にユーザーへ方針確認**。

## データフロー

### 学習走行（改修後）
```
1. CREEP 解放 → CREEP_SETTLE（クリープ計測）         … 既存
2. ACCEL_SWEEP: 数段の開度で 0→0.9×max_speed まで加速 … 全車速域の加速サンプル
3. BRAKE_HOLD: 各ブレーキ開度で高速→低速まで定常減速  … 清浄な減速サンプル
4. COAST_DOWN: 高速→惰行で停止近くまで               … 既存（コースト/エンジンブレーキ）
5. train: accel/brake Ridge を fit、R² を算出          … R²≥0.8 を確認
```

## エラーハンドリング戦略

- 既存方針を踏襲: スキップ型安全（過速度・過G→当該区間打ち切り）と非常停止型（過電流・CAN/
  アクチュエータ失敗・サイクル wedge）。新フェーズ BRAKE_HOLD も同じ安全経路に乗せる。
- 学習データ不足は `LearningDataError`（既存）で 422 系として扱う（`train_inverse_model` のしきい）。

## テスト戦略

### ユニットテスト
- `tests/unit/test_learning_drive.py`: `generate_patterns` が ACCEL_SWEEP 段（全域到達開度）と
  BRAKE_HOLD 段を含み、開度が `max_accel_opening`/`max_brake_opening` を超えないこと。
- 学習ループのフェーズ遷移テスト（短縮 config）: DRIVE_ACCEL が cap 到達で離脱、BRAKE_HOLD が
  固定保持→次パターンへ遷移、`max_decel_g` 超で開度が抑制されること。
- （機能4 着手時）`model_training` のブレーキ R² 改善を合成データで確認。

### 統合テスト
- 既存 `tests/integration/test_web_api.py` の learning/train 経路が壊れないこと。

## 依存ライブラリ

新規追加なし（numpy / scikit-learn / 既存スタックを使用）。

## ディレクトリ構造

```
src/domain/learning_drive.py            # 加速スイープ再構成・BRAKE_HOLD 生成・本数/時間調整
src/domain/control/learning_loop.py     # 全域加速の離脱条件・BRAKE_HOLD フェーズ・Config 追加
src/models/learning_drive.py            # PatternKind.BRAKE_HOLD
src/domain/model_training.py            # 機能4 着手時のみ
scripts/analyze_decel_fit.py            # 新規・調査用（機能1）
tests/unit/test_learning_drive.py       # 生成パターン/フェーズ遷移
```

## 実装の順序

1. 機能1（調査スクリプト）→ データ起因/モデル起因を切り分け、design.md に判断を追記
2. 機能2（全車速域 加速スイープ）
3. 機能3（BRAKE_HOLD 定常計測）
4. 単体テスト・ruff・mypy
5. 実機検証（Playwright）→ R²≥0.8 確認。未達かつ機能1がモデル要因と判断 → 機能4（ユーザー確認後）

## セキュリティ考慮事項

- 実車を動かす走行のため、上限G・最高車速・過電流・非常停止の安全機構を一切弱めない。
- 実機走行はオペレーター（ユーザー）がサーバ再起動し、検証は Playwright MCP で当方が駆動する既存運用。

## パフォーマンス考慮事項

- 学習走行 ≤30分の予算管理: 全域加速は1本が長いので本数を絞り、BRAKE_HOLD は段数×保持時間で見積もる。
  `LearningDriveConfig` の段数（`accel_sweep_fracs`/`brake_hold_openings_pct`/`coast_down_count`）で
  本数を制御する。

**走行時間見積り（profile sample_0001: max_speed=100, max_decel_g=0.3, cap=90km/h, ガバナ加速
≈0.27G≈9.5km/h/s を仮定）**:

| フェーズ | 本数 | 1本あたり | 小計 |
|---------|------|----------|------|
| CREEP 解放＋安定待ち | 5+1 | ~3s + ~15s | ~30s |
| ACCEL_SWEEP（30/50/70/100%） | 4 | 加速 ~10s（≤timeout 20s）＋リセットブレーキ停車 ~10s | ~80–160s |
| BRAKE_HOLD（10/20/30/40%） | 4 | 加速 ~10s ＋ 保持減速 ~10–20s（≤timeout 20s） | ~80–160s |
| COAST_DOWN | 3 | 加速 ~10s ＋ 惰行 ~6s（coast_timeout） | ~48s |

合計概算 **約 4〜7 分**。30 分予算に対し十分な余裕。低開度 ACCEL_SWEEP / 低ブレーキ BRAKE_HOLD が
頭打ち/緩減速で timeout（各 20s）に達しても、最悪ケースで ~10 分程度に収まる。

## 将来の拡張性

- BRAKE_HOLD の段数・速度帯は config 化し、車両ごとに調整可能にする。
- 機能4 のモデル改修は学習データ品質が十分になってから（過剰適合回避）。

---

# フェーズ6-3: 推定器を poly2 Ridge に復帰（GBM の閉ループ破綻対応・2026-07-02）

## 背景（実走行での破綻）

フェーズ6-2 で推定器を単調制約付き HistGradientBoosting＋過去Δv 9特徴に変更し、in-sample で
MAE≤0.5 / RMSE≤1.0 / R²≥0.8 を達成した（学習モデル `…_20260701_224556.pkl`）。しかし実走行モード
**06_WLTP_ExHi**（auto session `4c88b2dd`, 2026-07-02 07:59 JST）で追従が破綻:

- 追従偏差 MAE **18.3km/h**・最大 **36.8km/h**。ref 最大 74km/h の区間で実車速 **105.8km/h** まで過加速。
- 直前の走行（22:58 UTC）は emergency 終了。

## 原因分析（読み取り専用・確定）

学習セッション `b23a2fd1`（このモデルの学習元）と WLTP-ExHi 基準軌跡でオフライン検証した結果:

**1. GBM は巡航域（dv=0）で速度によらず一定値を返す**

| v0 [km/h] | GBM 予測 | poly2 予測 | 妥当値の目安 |
|---|---|---|---|
| 40 | 7.3% | 7.4% | ~7% |
| 60 | **7.3%** | 10.9% | ~11-12% |
| 80 | **7.3%** | 12.7% | ~13% |
| 100 | **7.3%** | 10.5% | ~14% |

木モデルは学習データに無い特徴領域（off-manifold）で**定数に飽和**する。巡航で開度不足→失速、
ランプでは 18〜23% と過大→過加速。この交互で「まともに走らない」挙動になる。

**2. 根本: 学習パターンに定常巡航サンプルがほぼ無い**

ACCEL_SWEEP は「cap まで加速→即リセットブレーキ」のため、|dv|<0.6 かつアクセル踏みの標本が
各速度帯 n≤5。巡航は**両モデルとも外挿**だが、poly Ridge は滑らかに補間して実用値を返し、GBM は
破綻する。**in-sample/CV が良くても走行モード軌跡は学習パターンと別分布**であり、CV は同一軌跡
ファミリー内のシャッフルなので off-manifold 性能を測れない（重要な学び）。

**3. 補足**: プロファイルは max_speed=140 / max_decel_g=0.4 に変更されており、学習セッションの
133.9km/h は正当（cap=0.9×140=126 の範囲内挙動）。speed_clip_max=133.9 も正常。

## 決定（ユーザー確認済み）

**Ridge 復帰のみ（最小変更）**。以下は推定器と独立の改善なので**維持**する:
- 過去Δv 9特徴（`PAST_HORIZONS_S`・`build_feature_row`/`_build_feature_matrix`/`predict_effort` の
  past_speeds・drive_loop の past_speeds 構築）
- 最新学習セッションのみで学習（`latest_learning_session_id`）
- `speed_clip_max` 入力クリップ（v0>cm は軌跡平行移動で dv 保持）

巡航データ追加（学習パターンへの巡航保持追加）は今回**スコープ外**（将来課題として記録）。

## 実装設計（フェーズ6-3）

### 1. `src/domain/model_training.py`
- `_make_estimator()` を poly2 Pipeline に復帰:
  `make_pipeline(PolynomialFeatures(2, include_bias=False), StandardScaler(), Ridge(alpha=1.0))`。
  `dv_monotonic` 引数を削除（呼び出し側 2箇所も引数なしに）。
- import: `sklearn.ensemble.HistGradientBoostingRegressor` を削除し、
  `sklearn.pipeline.Pipeline, make_pipeline` / `sklearn.preprocessing.PolynomialFeatures,
  StandardScaler` / `sklearn.linear_model.Ridge` を復活。`_metrics` の型注釈は `Pipeline` に。
- `MODEL_TYPE = "poly_past_inverse_lookahead"` に変更。**"poly_inverse_lookahead"（旧7特徴）と
  "gbm_inverse_lookahead" の両方を弾いて再学習を強制する**ための新識別子（旧 pkl をロードすると
  9特徴入力で predict が壊れるため、識別子で確実に拒否する）。
- モジュール docstring・コメントを poly 表現に戻す。GBM は off-manifold（巡航域のデータ欠如）で
  閉ループ破綻したため不採用、の旨をコメントで残す（再発防止）。
- **9特徴・PAST_HORIZONS_S・speed_clip_max・payload 構成（past_horizons キー含む）は変更しない。**

### 2. `src/domain/control/feedforward.py`
- docstring のみ更新（HistGradientBoostingRegressor → Pipeline（多項式＋標準化＋Ridge））。
  ロジック変更なし（`_Regressor` Protocol は `.predict()` のみ要求するのでそのまま動く）。

### 3. テスト
- `tests/unit/test_model_training.py`:
  `test_estimator_is_monotonic_gbm` → `test_estimator_is_polynomial_pipeline` に戻す
  （`isinstance(model, Pipeline)` かつ `model.steps[0][1]` が `PolynomialFeatures`）。
  他のテスト（9特徴 build_feature_row・past_horizons payload・speed_clip_max）は変更不要。
- `tests/unit/test_feedforward.py` / `test_drive_loop.py`: 変更不要（payload 形は同一。
  `make_model_file` の docstring の "gbm_inverse_lookahead" 文言だけ気になるなら直す）。

### 4. ドキュメント
- `docs/functional-design.md`（運転モデル構造）: 推定器記述を「完全2次多項式展開＋標準化＋Ridge の
  Pipeline（9特徴・過去Δv 含む・単調制約なし）」に戻し、model_type を更新。GBM を試して閉ループ
  破綻した経緯を1〜2行残す。
- `docs/glossary.md`（運転モデル）・`docs/architecture.md`（GBM 逆モデル/逆FFモデルの記述 2箇所）:
  同様に poly 表現へ。

### 5. 検証手順
1. `.venv/bin/pytest -m "not hardware"` / `.venv/bin/ruff check` / `.venv/bin/mypy`（変更ファイル）
2. オフライン: 最新学習セッションで実 `train_inverse_model` 経路の再学習 → in-sample が poly 水準
   （加速 MAE≈1.3〜1.5 / R²≈0.93 前後）に戻り、巡航予測（dv=0, past dv=0）が 60〜100km/h で
   10〜13% の滑らかな曲線になること。
3. ユーザー: サーバ再起動 → 学習運転1回（新 MODEL_TYPE のため再学習必須。それまで FF 無効=PID のみ）
   → 06_WLTP_ExHi を実走し追従（偏差・過加速なし）を確認。

## 残課題（次フェーズ候補）

- **巡航データの欠如**（両モデル共通の弱点）: ACCEL_SWEEP に「中間開度の巡航保持」を追加し、
  (v0, dv≈0) × アクセル開度の定常標本を採る。これで 120km/h 超の巡航予測（現状 poly で ~1% と過小）
  も改善できる。
- MAE≤0.5 の精度目標は GBM でしか達成できなかったが、閉ループ追従を優先して poly を採用。
  巡航データ追加後に GBM を再評価する余地はある（off-manifold の穴が埋まれば飽和問題は緩和）。
