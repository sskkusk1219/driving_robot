# 要求内容

## 概要

連続走行ログ（auto/manual/learning の任意セッション）から Ridge 回帰の**逆モデル**（目標速度・加速度 → ペダル開度）を学習し、車両プロファイルに紐づけて保存・制御へ反映する運転モデル学習機能を実装する。既存の 2D グリッド補間モデルは廃止し Ridge へ完全置換する。

## 背景

PRD「4. 運転モデル学習」(P0) はほぼ未完成だった:

- モデル学習ロジックは 2D グリッド補間（`griddata`）として存在するが、学習 API・`model_path` のプロファイル更新・`FeedforwardController.load_model()` の呼び出し配線がいずれも未実装で、**学習モデルが制御へ反映されない**。
- 2D グリッド補間は各速度点での整定待ちが必要で、全グリッドを実測で埋めると 60 分超となり PRD の 20 分制約を満たせない。さらに `run_pattern()` が整定を待たず即測定するためグリッド軸（目標値）と実測値が乖離する。

連続走行ログから Ridge 逆モデルを学習する方式に切り替えることで、構造化グリッド走行も 20 分制約も不要になり、Ridge の汎化により疎なデータでも全領域をカバーできる。

参考: `sskkusk1219/driving_simulator` の `train_ridge.py` / `speed_predictor.py`。ただし simulator は順モデル（ペダル→Δspeed）、本システムは逆モデル（目標速度・加速度→ペダル開度）なので方向を反転して移植する。

## 実装対象の機能

### 1. 先読み型 Ridge 逆モデル学習ロジック
- 連続 `DriveLog` の実車速軌跡を「意図軌跡」とみなし、現在速度と数秒先（0.5/1.0/2.0/3.0s）の基準速度トレンドから人間の運転を模した先読み特徴量を構築する
- 先読み差分ハイブリッド特徴量 `[v0, Δv@0.5, Δv@1.0, Δv@2.0, Δv@3.0, v0², Δv@1.0·v0]`（7次元）を使用
- 先読みトレンド `Δv@1.0` の符号でアクセル/ブレーキ 2 モデルに分割し Ridge 回帰（`fit_intercept=False`）で学習する
- メタデータ付き dict 形式 pkl として保存する

### 2. FeedforwardController の先読み対応
- pkl をロードし `predict(v0, future_speeds)` で開度を推論する（純粋FF: 基準軌跡のみの関数）
- 2D グリッド補間ロジックは廃止する

### 6. DriveLoop の先読みサンプリング対応
- 基準軌跡を各ホライズン分先読みサンプルして FF に渡す

### 3. 学習 API エンドポイント
- `POST /api/v1/drive/learning/train` でログから学習・保存・プロファイル更新・メトリクス返却を行う
- 再学習も同一エンドポイントで実行できる

### 4. モデルを制御へ反映する配線
- factory で `FeedforwardController` を生成・注入し、`select_profile` で `model_path` からモデルをロードする

### 5. フロントエンド学習導線
- 学習画面に「モデル学習」ボタンを追加し、学習結果（メトリクス・保存パス）を表示する

## 受け入れ条件

### 先読み型 Ridge 逆モデル学習ロジック
- [ ] 連続 `DriveLog` のリストから先読み特徴量を構築し Ridge 逆モデル（アクセル/ブレーキ）を学習できる
- [ ] 先読みトレンド `Δv@1.0` の符号でレジーム分割する
- [ ] ログ不足時に `LearningDataError` を送出する
- [ ] メタデータ付き dict 形式 pkl（`model_type="ridge_inverse_lookahead"`・horizons 等）を保存する
- [ ] 学習時に MAE/RMSE/R² を算出する

### FeedforwardController
- [ ] `ridge_inverse_lookahead` 形式の pkl をロードできる
- [ ] `predict(v0, future_speeds)` が先読みトレンド符号でアクセル/ブレーキを切り替え、0〜100% にクランプする

### DriveLoop
- [ ] 基準軌跡をホライズン分先読みサンプルし `ff.predict` に渡す

### 学習 API
- [ ] `POST /api/v1/drive/learning/train` でログ取得→学習→pkl 保存→`model_path` 更新ができる
- [ ] ログ不足は 422、プロファイル無しは 404 を返す

### モデル反映配線
- [ ] factory で `FeedforwardController` が生成され `RobotController` に注入される
- [ ] `select_profile` で `model_path` がある場合にモデルがロードされる

### フロントエンド
- [ ] 学習画面から学習を実行でき、結果が表示される

## 成功指標

- `pytest tests/unit/` が全通過する
- ruff / mypy がエラーなく通る
- 既存セッションログから学習を実行し、Δ開度 MAE が妥当な範囲に収まる

## スコープ外

以下はこのフェーズでは実装しません:

- クリープ/エンジンブレーキ定数の採用（順モデル向け挙動のため）
- オンライン学習（PRD #15, P2）
- 学習データのセッション選択 UI の高度化（最小限の導線のみ）

## 参照ドキュメント

- `docs/product-requirements.md` - プロダクト要求定義書（4. 運転モデル学習）
- `docs/functional-design.md` - 機能設計書
- `docs/architecture.md` - アーキテクチャ設計書
