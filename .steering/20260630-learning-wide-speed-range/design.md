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
