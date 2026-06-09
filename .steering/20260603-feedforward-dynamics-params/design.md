# 設計書

## アーキテクチャ概要

既存レイヤード構成を踏襲。`FeedforwardParams` を車両プロファイルのドメイン値として追加し、JSONB 列で永続化。FF は select_profile 時に定数を受け取り、predict でレジームオーバーライド＋不感帯スナップを行う。学習時はログから観測可能な定数のみ推定して更新する。

```
[学習] POST /learning/train
   ├ train_inverse_model(logs, profile)          → (model_path, metrics)
   ├ estimate_dynamics_params(logs, profile.ffp) → 観測可能な定数のみ上書きした FeedforwardParams
   └ profile.model_path / profile.feedforward_params を更新 → repo.update
[選択] select_profile(profile)
   ├ ff.load_model(model_path)
   └ ff.set_params(profile.feedforward_params)
[制御] DriveLoop → ff.predict(v0, future_speeds)
        定数で停車/クリープ/惰行をオーバーライド、不感帯で微小開度を 0 にスナップ
```

## コンポーネント設計

### 1. `src/models/profile.py`
- `@dataclass FeedforwardParams`（6定数、AT 標準のデフォルト付き）
  - `creep_speed_kmh=7.0, creep_rate_kmhs=0.5, engine_brake_decel_kmhs=1.0,`
    `stop_brake_opening_pct=20.0, brake_deadband_pct=1.0, accel_deadband_pct=1.0`
- `VehicleProfile` に `feedforward_params: FeedforwardParams = field(default_factory=FeedforwardParams)` を**末尾**に追加
  （デフォルト付き末尾フィールドにすることで既存の全コンストラクタ呼び出しを壊さない）

### 2. `scripts/setup_db.py`
- `vehicle_profiles` の CREATE TABLE に `feedforward_params JSONB`（nullable）追加
- 既存DB向けに `ALTER TABLE vehicle_profiles ADD COLUMN IF NOT EXISTS feedforward_params JSONB` を追加

### 3. `src/infra/profile_repository.py`
- `_row_to_profile`: `feedforward_params` を JSON→`FeedforwardParams`。NULL/欠損キーはデフォルト
- `create`/`update`: `feedforward_params` を JSON 化して INSERT/UPDATE に追加。戻り値の VehicleProfile にも反映
- ヘルパ `_ffp_to_json` / `_ffp_from_row` を用意

### 4. `src/domain/model_training.py`
- 定数: `STOP_SPEED_KMH=0.5`, `CREEP_STEADY_TOL_KMHS=0.3`, `MIN_OBS_SAMPLES=5`
- `estimate_dynamics_params(logs, current: FeedforwardParams) -> FeedforwardParams`
  - セッション単位で dt 推定し、ペダルオフ判定に current の不感帯を使用
  - stop_brake: speed<STOP_SPEED かつ brake>=deadband の brake 中央値
  - creep_speed: ペダルオフ かつ speed>STOP かつ |dv/dt|<TOL（定常）の speed 中央値
  - engine_brake_decel: ペダルオフ かつ speed>creep かつ減速中の (-dv/dt) 中央値
  - creep_rate: ペダルオフ かつ speed<creep かつ加速中の (dv/dt) 中央値
  - 各々 MIN_OBS_SAMPLES 以上で上書き、未満は current を保持
  - **不感帯（accel/brake）は推定せず current を保持**（スコープ外）
- `train_inverse_model`: brake ラベル整形のデッドバンドを `profile.feedforward_params.brake_deadband_pct` から取得（デフォルト 1.0 で従来と等価）

### 5. `src/domain/control/feedforward.py`
- `set_params(params: FeedforwardParams)`（未設定時はデフォルト）
- `predict(v0, future_speeds)` の順序:
  1. 停車レジーム: v0<=STOP_SPEED かつ全 future<=STOP_SPEED → (0, stop_brake)
  2. Ridge 予測（dv1 符号でアクセル/ブレーキ）
  3. クリープ域: v0<creep_speed かつ |desired_accel|<=creep_rate → (0, 0)
  4. 惰行: dv1<0 かつ desired_decel(=-dv1/REGIME_HORIZON)<=engine_brake_decel → brake=0
  5. 不感帯スナップ: accel<accel_deadband→0、brake<brake_deadband→0
  6. 0〜100 クランプ

### 6. `src/app/robot_controller.py`
- `select_profile`: `load_model` 後（または常に）`self._ff_controller.set_params(profile.feedforward_params)`

### 7. Web 層
- `schemas.py`: `FeedforwardParamsSchema`（6フィールド+デフォルト）。`ProfileCreateRequest`/`ProfileUpdateRequest`（任意）/`ProfileResponse` に追加。`TrainModelResponse` に `feedforward_params` 追加
- `profiles.py`: `_to_response`・create・update で FeedforwardParams をマッピング（update は req 未指定なら existing 保持）
- `drive.py` `/learning/train`: `estimate_dynamics_params` を呼び、`profile.feedforward_params` を更新して保存・返却

### 8. フロントエンド `profiles.js`
- 編集フォームに「フィードフォワード定数」Box を追加（6入力）。form 初期値・handleSave に組み込む

## エラーハンドリング
- JSONB 欠損/NULL → デフォルト値
- 推定でデータ不足 → 既存値保持（例外にしない）

## テスト戦略
- `test_model_training.py`: estimate_dynamics_params（観測あり上書き／不足で保持／不感帯は不変）
- `test_feedforward.py`: 停車・クリープ・惰行・不感帯の各レジーム、set_params 既定値
- `test_profile_repository.py`（integration）: feedforward_params ラウンドトリップ（DB 必要なら最小限）
- `test_web_profiles` / `test_web_drive.py`: スキーマ往復・train で params 更新
- `test_robot_controller.py`: select_profile で set_params 呼出

## 依存ライブラリ
追加なし。

## ディレクトリ構造
```
scripts/setup_db.py                   (列追加)
src/models/profile.py                 (FeedforwardParams + field)
src/infra/profile_repository.py       (JSONB 読み書き)
src/domain/model_training.py          (estimate_dynamics_params)
src/domain/control/feedforward.py     (set_params + predict 拡張)
src/app/robot_controller.py           (set_params 配線)
src/web/schemas.py                    (スキーマ追加)
src/web/routers/profiles.py           (マッピング)
src/web/routers/drive.py              (train で推定・保存)
src/web/static/js/screens/profiles.js (編集UI)
tests/unit/*                          (各テスト)
```

## 実装順序
1. models → 2. DB → 3. repository → 4. model_training(estimate) → 5. feedforward(predict) →
6. robot_controller → 7. schemas → 8. profiles router → 9. drive router → 10. UI → 11. tests → 12. 品質チェック
