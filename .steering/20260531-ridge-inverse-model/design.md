# 設計書

## アーキテクチャ概要

レイヤードアーキテクチャ（domain / infra / web / app）を踏襲する。Ridge 逆モデルの学習はドメイン層、ログ取得はインフラ層、学習トリガはWeb層、配線はapp層に配置する。

本モデルは**先読み（look-ahead / preview）型の純粋フィードフォワード**として設計する。人間の運転を模し、現在の基準速度と数秒先の基準速度トレンドから名目ペダル開度を予測する。追従誤差の補正は PID に委ねる（FF は基準軌跡のみの関数）。

```
[Web] POST /api/v1/drive/learning/train
        │  profile_id, session_ids?
        ▼
[Infra] SessionRepository.list_logs_for_training() ──► list[DriveLog]
        ▼
[Domain] model_training.train_inverse_model(logs, profile)
        │  - 前処理（デッドバンド・速度クリップ）
        │  - 先読み特徴量（実車速軌跡を「意図軌跡」とみなす）
        │  - レジーム分割（先読みトレンド Δv@1.0 の符号）
        │  - 2 Ridge 学習（fit_intercept=False）
        │  - dict pkl 保存
        ▼  (model_path, metrics)
[Infra] ProfileRepository.update(model_path)
        ▼
[App] select_profile() → FeedforwardController.load_model(model_path)
        ▼
[Domain] DriveLoop が基準軌跡を先読みサンプルし ff.predict(v0, future_speeds) で制御
```

## 特徴量設計（先読み・差分ハイブリッド）

**ホライズン**: `[0.5, 1.0, 2.0, 3.0]` 秒。ログは約 10Hz（0.1s 周期 = `LOG_EVERY_N_CYCLES(2)×0.05`）なのでサンプルオフセット = `round(horizon / dt)`（≈ 5/10/20/30）。

**特徴量（アクセル/ブレーキ各モデル共通の 7 次元）**:

| 特徴量 | 定義 | 意味 |
|---|---|---|
| `v0` | 現在の基準速度 | 動作点 |
| `dv_0.5` | `target(t+0.5) − v0` | 直近の先読みトレンド |
| `dv_1.0` | `target(t+1.0) − v0` | 短期先読み |
| `dv_2.0` | `target(t+2.0) − v0` | 中期先読み |
| `dv_3.0` | `target(t+3.0) − v0` | 長期先読み |
| `v0_sq` | `v0²` | 空気抵抗・高速域の非線形 |
| `dv1_x_v0` | `dv_1.0 · v0` | 速度依存の必要開度（先読み版交互作用） |

**学習データ構築（純粋FF）**: 手動/学習走行の連続ログでは目標速度系列が記録されないため、**実車速の軌跡そのものを「その走行で意図された目標軌跡」とみなす**。
- 学習時: `v0 = actual_speed[i]`、`target(t+k) = actual_speed[i + offset_k]`、ラベル = `accel_opening[i]` / `brake_opening[i]`
- 推論時: `v0 = 基準車速(t)`、`target(t+k) = 基準車速(t+k)`（CSV を先読み）

これにより学習・推論で特徴量の意味が一貫し、`speed` と `target(t)` は同一（= `v0`）に統合される。

## コンポーネント設計

### 1. `src/domain/model_training.py`（新規）

**責務**:
- 連続 `DriveLog` から先読み型 Ridge 逆モデルを学習・保存する

**実装の要点**:
- ホライズン定数 `LOOKAHEAD_HORIZONS_S = (0.5, 1.0, 2.0, 3.0)`、`REGIME_HORIZON_S = 1.0`（共有）
- `build_lookahead_features(speed_series, offsets) -> np.ndarray`: 上表の 7 特徴量を構築（`v0`, `dv_*`, `v0²`, `dv_1.0·v0`）
- `train_inverse_model(logs, profile, output_dir="data/models") -> tuple[str, dict]`:
  - ブレーキデッドバンド 1.0%、速度クリップ `>=0`
  - timestamp 差分から中央値 dt を算出し、各ホライズンのサンプルオフセット = `round(horizon/dt)`
  - 末尾 `max(offset)` サンプルは未来データ不足のため除外
  - レジーム分割: `dv_1.0 >= 0` → accel モデル（ラベル `accel_opening`）、`< 0` → brake モデル（ラベル `brake_opening`）
  - `Ridge(alpha, fit_intercept=False)` を 2 本学習
  - MAE/RMSE/R² を算出
  - dict pkl 保存（`model_type="ridge_inverse_lookahead"`、`horizons`、`offsets`、`feature_names`）
  - データ不足時 `LearningDataError`（`learning_drive` から import して共有）

### 2. `src/domain/control/feedforward.py`（置換）

**責務**:
- pkl をロードし、先読み基準速度から FF 開度を推論する

**実装の要点**:
- `RegularGridInterpolator` を廃止
- `load_model()`: dict をロードし `model_type=="ridge_inverse_lookahead"` を検証、accel/brake モデル・horizons・feature_names を保持
- `predict(v0, future_speeds)`: `future_speeds` はホライズン順の基準速度リスト。内部で差分特徴量を構築
  - レジーム判定: `dv_1.0 = future_speeds[REGIME index] − v0` の符号。`>=0` ならアクセルモデル（brake=0）、`<0` ならブレーキモデル（accel=0）
  - 0〜100% クランプ

### 3. `src/infra/session_repository.py`（追加）

**責務**:
- 学習用に複数セッションのログを集約取得する

**実装の要点**:
- `list_logs_for_training(profile_id, session_ids=None, limit=...) -> list[DriveLog]`
- 既存 `_row_to_log` を再利用。profile_id でセッションを絞り、timestamp 昇順で返す

### 4. `src/web/routers/drive.py` / `schemas.py`（追加）

**責務**:
- 学習をトリガし結果を返す

**実装の要点**:
- `TrainModelRequest{profile_id, session_ids?}` / `TrainModelResponse{model_path, metrics}`
- `POST /api/v1/drive/learning/train`: ログ取得→`train_inverse_model`→`ProfileRepository.update`→返却
- 既存 `get_profile_repo` deps と新規 session_repo deps を利用

### 5. `src/app/factory.py` / `robot_controller.py`（配線）

**責務**:
- モデルを制御へ反映する

**実装の要点**:
- factory で `FeedforwardController()` を生成し `RobotController(ff_controller=...)` に注入
- `select_profile()`: `profile.model_path` があれば `ff_controller.load_model()` を呼ぶ。失敗/未設定時は警告ログのみ（走行は別途 pre_check で担保）

### 6. `src/domain/control/drive_loop.py`（先読みサンプリング対応）

**責務**:
- 基準軌跡をホライズン分先読みし FF に渡す

**実装の要点**:
- `_get_ref_lookahead(elapsed_s) -> tuple[float, list[float]]`: 既存 `_get_ref_speed_and_accel` の補間ロジックを再利用し、`elapsed_s` と `elapsed_s + horizon`（各ホライズン）の基準速度を返す。軌跡末尾を超える場合は終端値でクランプ（既存挙動踏襲）
- `_execute_one_cycle`: `ff.predict(v0, future_speeds)` を呼ぶ。PID は従来どおり `ref_speed(now)` と `actual_speed` の偏差で更新。逸脱判定も `ref_speed(now)` を使用

### 7. `src/web/static/js/screens/learning.js`（追加）

**責務**:
- 学習導線 UI

**実装の要点**:
- 「モデル学習」ボタン → `POST /api/v1/drive/learning/train` → メトリクス/保存パス表示
- 既存 `apiFetch`・`ValidationPopup` パターンを流用

## データフロー

### モデル学習
```
1. ユーザーが学習画面で「モデル学習」を押下
2. POST /api/v1/drive/learning/train {profile_id}
3. SessionRepository が profile のログを集約取得
4. train_inverse_model が前処理→学習→pkl保存
5. ProfileRepository が model_path を更新
6. メトリクスをレスポンスで返し UI に表示
```

### 制御への反映
```
1. プロファイル選択 → controller.select_profile(profile)
2. model_path があれば ff.load_model(model_path)
3. 自動走行で DriveLoop が基準軌跡を先読みサンプル（v0, future_speeds）
4. ff.predict(v0, future_speeds) で名目開度、PID が actual との偏差を補正
```

## エラーハンドリング戦略

### カスタムエラークラス
- `LearningDataError`（既存・`src/domain/learning_drive.py`）をデータ不足に再利用

### エラーハンドリングパターン
- ログ不足 → `LearningDataError` → API 422
- プロファイル無し → API 404
- `load_model` 失敗 → 警告ログのみ（クラッシュさせない）

## テスト戦略

### ユニットテスト
- `test_model_training.py`（新規）: 特徴量構築・レジーム分割・データ不足例外・pkl 内容・メトリクス
- `test_feedforward.py`（更新）: ridge_inverse load/predict・クランプ・符号分岐
- `test_web_drive.py`（更新）: `/learning/train` 正常系・422・404
- `test_robot_controller.py`（更新）: `select_profile` でのモデルロード
- `test_learning_drive.py`（更新）: 削除した grid `train_model` のテストを除去

### 統合テスト
- 既存セッションログから学習 → pkl 生成・model_path 更新を確認（手動 + 必要に応じ integration）

## 依存ライブラリ

scikit-learn を追加（`.venv` にインストール）。

```
scikit-learn>=1.4
```

## ディレクトリ構造

```
src/domain/model_training.py          (新規)
src/domain/control/feedforward.py     (置換: 先読み predict)
src/domain/control/drive_loop.py      (先読みサンプリング対応)
src/domain/learning_drive.py          (grid train_model 削除)
src/infra/session_repository.py       (メソッド追加)
src/web/routers/drive.py              (エンドポイント追加)
src/web/schemas.py                    (スキーマ追加)
src/app/factory.py                    (FF 生成・注入)
src/app/robot_controller.py           (select_profile でロード)
src/web/static/js/screens/learning.js (学習ボタン追加)
tests/unit/test_model_training.py     (新規)
tests/unit/test_feedforward.py        (更新)
tests/unit/test_web_drive.py          (更新)
tests/unit/test_robot_controller.py   (更新)
tests/unit/test_learning_drive.py     (更新)
pyproject.toml                        (scikit-learn 追加)
```

## 実装の順序

1. 依存追加（pyproject + venv install）
2. ドメイン: model_training.py + feedforward.py 置換 + learning_drive.py 整理
3. インフラ: session_repository ログ集約
4. Web: schemas + drive ルーター
5. App: factory + robot_controller 配線
6. フロントエンド: 学習ボタン
7. テスト更新・追加
8. 品質チェック（ruff/mypy/pytest）

## セキュリティ考慮事項

- pkl は開発者がローカル生成した信頼済みファイルのみロードする（`feedforward.py` 既存方針を維持）
- `profile_id` はパス生成時に `Path(...).name` でサニタイズ（既存 `train_model` 方針を踏襲）

## パフォーマンス考慮事項

- 学習は同期処理だが Ridge 閉形式で軽量。大量ログ時は `limit` で件数制御
- FF `predict` は 50ms 制御ループ内で呼ばれるため、行列演算は最小特徴量数（5）で軽量に保つ

## 将来の拡張性

- `model_type` を pkl に持たせるため、将来別アルゴリズム（GPR 等）追加時も `load_model` dispatch で拡張可能
- オンライン学習（PRD #15）は本逆モデルの係数更新として拡張余地あり
