# タスクリスト: フィードフォワード物理定数（クリープ/不感帯ほか）

## 🚨 タスク完全完了の原則
全タスクを `[x]` にするまで継続。未完了を残して終了しない。スキップは技術的理由のみ明記。

---

## フェーズ1: ドメインモデル
- [x] `src/models/profile.py` に `FeedforwardParams` を追加（6定数+デフォルト）
- [x] `VehicleProfile` に `feedforward_params` を末尾フィールド（default_factory）で追加

## フェーズ2: DB スキーマ
- [x] `scripts/setup_db.py` の CREATE TABLE に `feedforward_params JSONB` 追加
- [x] `ALTER TABLE ... ADD COLUMN IF NOT EXISTS feedforward_params JSONB` を追加

## フェーズ3: リポジトリ
- [x] `_row_to_profile` で `feedforward_params` を読込（NULL/欠損→デフォルト）
- [x] `create`/`update` で JSON 化して永続化、戻り値にも反映（stub も同様）
- [x] `_ffp_to_json` / `_ffp_from_value` ヘルパ

## フェーズ4: 学習時推定
- [x] `model_training.py` に定数（STOP_SPEED_KMH 等）と `estimate_dynamics_params` を追加
  - [x] stop_brake / creep_speed / engine_brake_decel / creep_rate を観測時のみ上書き
  - [x] 不感帯は current 保持
- [x] `train_inverse_model` の brake デッドバンドを `profile.feedforward_params.brake_deadband_pct` 由来に

## フェーズ5: FF 推論
- [x] `feedforward.py` に `set_params` を追加
- [x] `predict` を停車/クリープ/惰行レジーム + 不感帯スナップに拡張

## フェーズ6: 配線
- [x] `robot_controller.select_profile` で `ff.set_params(profile.feedforward_params)`

## フェーズ7: Web 層
- [x] `schemas.py` に `FeedforwardParamsSchema`、Profile Create/Update/Response・TrainModelResponse へ追加
- [x] `profiles.py` の `_to_response`/create/update でマッピング
- [x] `drive.py` `/learning/train` で `estimate_dynamics_params` を呼び params 更新・保存・返却

## フェーズ8: フロントエンド
- [x] `profiles.js` 編集フォームに「フィードフォワード定数」入力群を追加（初期値・保存組込み）

## フェーズ9: テスト
- [x] `test_model_training.py`: estimate_dynamics_params（観測上書き/不感帯不変/不足保持）
- [x] `test_feedforward.py`: 停車/クリープ/惰行/不感帯 + set_params
- [x] `test_robot_controller.py`: select_profile で set_params
- [x] `test_web_drive.py`: train で feedforward_params 更新
- [x] `tests/unit/infra/test_profile_repository.py`: feedforward_params 往復（モック範囲）

## フェーズ10: 品質チェック
- [x] `.venv/bin/pytest tests/unit/` → 416 passed
- [x] `.venv/bin/ruff check`（変更分 clean。既存の setup_db I001・robot_controller E501 は着手前から）
- [x] `.venv/bin/ruff format`（新規/変更分を整形済み）
- [x] `.venv/bin/mypy`（変更モジュール: issues なし）

## フェーズ11: 振り返り
- [x] 実装後の振り返りを記録

---

## 実装後の振り返り
### 実装完了日
2026-06-03

### 計画と実績の差分
- 当初「forward sim 用だから不要」と判断していたクリープ等の定数を、`fit_intercept=False` の逆FFが
  原点で必ず 0 を返す＝停車保持ブレーキを出せない、という気付きから採用に転換。
- 不感帯（accel/brake）は自動推定対象外とし、ユーザー手動調整＋デフォルト保持に限定（信頼推定が困難なため）。
  これは「観測可能な定数のみ上書き」ポリシーと整合。
- `VehicleProfile.feedforward_params` はデフォルト付き末尾フィールドにすることで、既存の全コンストラクタ
  呼び出し（テスト・stub・repo）を壊さずに追加できた。

### 学んだこと
- pydantic v2 + `from __future__ import annotations` 無しのモジュールでは、前方参照が効かず
  クラス定義順が重要。`FeedforwardParamsSchema` を参照する `TrainModelResponse` より前に定義する必要があった。
- `FF.predict` は model 未ロードだと停車レジーム（定数のみ）も含めて RuntimeError とする契約を維持。
  FF はモデル前提のコンポーネントであり、部分的に動かすと設定ミスを隠すため。
- 推定はセッション単位で dt を出し、ペダルオフ/速度域/加減速のマスクで観測を分類。境界をまたがないよう
  既存の `_group_by_session` を再利用。

### 次回への改善提案
- 不感帯の自動推定（速度が変化し始める開度の検出）は、専用の低速スイープ走行データがあれば実現可能。別途検討。
- DB を使った integration テストで feedforward_params 列の往復と ALTER 冪等性を検証するとより堅牢。
- 実機の閉ループで、停車保持ブレーキ・クリープ域の挙動を PRD KPI（停車安定・低速追従）で評価。
