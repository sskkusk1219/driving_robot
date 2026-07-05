# 要求内容

## 概要

アクチュエータ(ペダル機構)を含む系の応答遅れ(むだ時間)を補償するため、基準軌跡の時間シフト量 `preview_time_s` を車両プロファイルに導入し、PID自動適合で先読み秒数も自動適合し、FOPDT同定値とともにプロファイルへ永続化する。

## 背景

学習運転のPID適合で基準速度に追従できない事象があり、原因としてモデル/PIDゲインだけでなくアクチュエータの応答遅れが疑われる。現状の実装調査で以下が判明:

- FOPDT同定(`src/domain/pid_tuning.py:66-118`)で開度→車速のむだ時間θを推定しているが、SIMC則(`compute_pid_gains_simc`)で**ゲインを下げる方向にしか使われず**、指令を前倒しする明示補償は存在しない。遅れが大きい車両ほどゲインが下がり、追従がさらに緩くなる構造。
- FFの先読み(`FeatureSpec.lookahead_horizons_s`)は逆モデルの特徴量ホライズンであり遅れ補償ではない。PIDは現在時刻の基準速度のみ使用(`drive_loop.py:362-397`)。
- FOPDT同定値(k, τ, θ)はどこにも永続化されていない。PIDゲインのみプロファイルへ自動保存済み。

## 実装対象の機能

### 1. 基準軌跡の先読み補償(preview_time_s)

- 制御用の基準速度サンプリングを `t_ctrl = elapsed_s + preview_time_s` に前倒しする
- **FFとPIDの両方**に適用(ユーザー確定)。FFのみだとPIDが先行分を誤差とみなして打ち消すため
- KPI・逸脱判定・ログ・WS表示の基準は従来通り現在時刻(now-frame)で評価(ダイナモの追従仕様)
- `preview_time_s = 0.0` で従来動作に完全一致(後方互換)

### 2. 先読み秒数の自動適合

- TRAINING段: FOPDT同定のθを初期値として `preview_time_s = clamp(θ, 0..3.0)` を自動設定
- REFINE段: `CoordinateDescentTuner` を kp→ki→kd→preview_time_s の4次元巡回に拡張し、実走KPIコストで絞り込む(ユーザー確定)
- SIMCゲイン計算はθフルのまま変更しない(previewは基準位相補償でありフィードバックループのむだ時間は不変。θを割引くと安定余裕を失う)
- 走行予算: `refine_runs_stage1` を 10→14 に増やす

### 3. 車両プロファイルへの永続化

- 新 dataclass `DynamicsParams`(preview_time_s, fopdt_k, fopdt_tau, fopdt_theta)を `VehicleProfile` に追加
- DB: `vehicle_profiles.dynamics_params JSONB` カラム追加(冪等ALTER TABLE)
- 適合結果(best gains + best preview)は学習サイクル完了時に自動でプロファイルへ保存
- プロファイルAPI/UIに露出(previewは手動微調整可、FOPDT値は表示)

### 4. 既存バグの併修: プロファイルPUTでFF調停定数がリセットされる

- `src/web/routers/profiles.py` の `_ffp_to_schema`/`_ffp_from_schema` が `FeedforwardParams` 11フィールド中6つしかマッピングしておらず、PUTで調停定数5個(`switch_hysteresis_pct` 等)がデフォルトへリセットされる
- `fields()` ベースの汎用変換に修正(適合値がUI編集で消える事故の防止)

## 受け入れ条件

### 先読み補償
- [ ] preview>0 のとき、PID/FFに渡る基準速度が前倒しされる(単体テストで検証)
- [ ] KPI・逸脱判定・DriveLogData.ref_speed_kmh・WS表示は now-frame のまま
- [ ] preview=0 で既存テストが全て無変更で通る(回帰なし)
- [ ] 軌跡終端付近で t_ctrl が末尾を超えても終端クランプで安全

### 自動適合
- [ ] FOPDT同定成功時に dynamics_params が設定され preview=clamp(θ,0..3) で初期化される
- [ ] 座標降下が4次元(kp, ki, kd, preview)を巡回し、previewは [0.0, 3.0] にクランプされる
- [ ] update_pid_gains=False(TRAINING_2)では既存の dynamics_params を保持する

### 永続化
- [ ] dynamics_params がDBへラウンドトリップ保存される(NULL行はデフォルト補完)
- [ ] 学習サイクル完了時に best gains + best preview がプロファイルへ保存される
- [ ] ProfileResponse に dynamics_params が含まれ、PUTで preview を手動更新できる
- [ ] PUTで feedforward_params の調停定数が保持される(バグ修正の検証)

## 成功指標

- 学習サイクル後の通常自動走行で、適合前後のKPI(p95偏差, 最大偏差)が改善すること(**生ログで確認**。サマリだけで判断しない)
- 遅れの大きい車両でランプ追従の位相遅れが減少すること

## スコープ外

以下はこのフェーズでは実装しません:

- FFモデルの特徴量ホライズン(`lookahead_horizons_s`)自体のプロファイル化・自動適合(pklに焼き込まれるため再学習が必要。今回はグローバル設定のまま)
- アクチュエータ単体の遅れ計測(サーボステップ応答試験)。FOPDTのθは系全体の遅れとして扱う
- スミス予測器などのモデルベース遅れ補償
- KPI定義・コスト関数の変更
- 学習運転(開ループ)・タイムスケジュール走行への preview 適用

## 参照ドキュメント

- `docs/functional-design.md` - 機能設計書
- `docs/architecture.md` - アーキテクチャ設計書
- `.steering/20260620-pid-auto-tuning/` - PID自動チューニングの設計
- `.steering/20260703-learning-process-revamp/` - 学習サイクルの設計
