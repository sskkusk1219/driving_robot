# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

- 全てのタスクを`[x]`にすること
- 「時間の都合により別タスクとして実施予定」は禁止
- 未完了タスク(`[ ]`)を残したまま作業を終了しない
- スキップは技術的理由がある場合のみ(理由を明記)

---

## フェーズ1: モデル層・制御コア

- [x] FeedforwardParams に調停用定数を追加(レビュー #1/#11 対応の定数群)
  - [x] `src/models/profile.py`: switch_hysteresis_pct / accel_reengage_dwell_s / accel_rate_limit_pct_s / brake_rate_limit_pct_s / pid_output_limit_pct
  - [x] `src/infra/profile_repository.py`: _ffp_from_value / _ffp_to_json に新キー追加(dataclasses.fields で全キー自動対応に変更)
  - [x] `src/web/schemas.py`: FeedforwardParamsSchema 拡張
  - [x] `tests/unit/infra/test_profile_repository.py`: JSONB ラウンドトリップ・欠損キー補完テスト
- [x] RealtimeSnapshot に captured_at を追加(レビュー #16 テレメトリ鮮度の基盤)
  - [x] `src/models/system_state.py`: captured_at: float = 0.0
- [x] PIDController 強化(レビュー #3/#12)
  - [x] output_limit / set_output_limit / 出力クランプ
  - [x] 計測 dt 引数(公称比 0.5〜4 倍クランプ)
  - [x] 条件付き積分(saturated_high/low)+ 積分量クランプ
  - [x] `tests/unit/test_pid.py` 更新(39 passed)
- [x] PedalArbiter 新規実装(レビュー #1/#11、同時踏み構造禁止)
  - [x] `src/domain/control/pedal_arbiter.py`: ヒステリシス・再踏込ディレイ・不感帯写像・レートリミット・クランプ・飽和フラグ
  - [x] `tests/unit/test_pedal_arbiter.py` 新規(21 passed)
- [x] FeedforwardController 連続化(レビュー #4/#5/#8/#9/#11)
  - [x] predict_effort(符号付き)へ変更・境界連続化(実装方法変更: 対称ブレンドは dv=0 で巡航スロットルを半減させ定常追従を悪化させるため、物理実体に即した「エンジンブレーキテーパ」= 緩減速はスロットル漸減で作る方式に変更)
  - [x] 停車保持を 0.5 秒先読み判定に変更
  - [x] クリープ則 [0, creep_rate] 限定・惰行則 v0 ≥ creep_speed 限定
  - [x] 不感帯スナップ削除
  - [x] unload_model() 追加
  - [x] `tests/unit/test_feedforward.py` 更新(25 passed)
- [x] KPIMonitor 新規実装(レビュー #7)
  - [x] `src/domain/control/kpi_monitor.py`: P95 ヒストグラム・最大偏差・5 秒窓符号反転・ハード違反 warning
  - [x] `tests/unit/test_kpi_monitor.py` 新規(12 passed)

## フェーズ2: DriveLoop 統合

- [x] effort 経路への置き換え(レビュー #1)
  - [x] FF+PID 合成 → PedalArbiter 呼び出し。旧符号分割・排他・max クランプを削除
  - [x] dt 計測と飽和フラグの引き回し
- [x] 緊急停止ヘルパー集約(`_abort_emergency`)
- [x] ウォッチドッグ(レビュー #16): 連続スキップ 1 秒で非常停止
- [x] stop_and_join 実装(レビュー #6)
- [x] ログ保留タスク上限(レビュー #13): MAX_PENDING_LOG_TASKS=100
- [x] KPIMonitor 統合と kpi_summary 公開・スナップショット captured_at 設定
- [x] `tests/unit/test_drive_loop.py` 更新(effort 経路・watchdog・stop_and_join・ログ上限・KPI、46 passed)

## フェーズ3: アプリ・インフラ層

- [x] CANReader 鮮度上限 0.2 秒(レビュー #2)
  - [x] `src/infra/can_reader.py`: max_speed_age_s コンストラクタ引数化(注: セッション中に作業ツリーのキャッシュ実装が HEAD 版へ巻き戻っていたため、レビュー時点の実装を復元した上で適用)
  - [x] `src/infra/settings.py`: can.max_speed_age_s 追加(config/settings.toml にも追記)
  - [x] `src/app/factory.py`: 配線
  - [x] `tests/unit/infra/test_can_reader.py` 更新(16 passed)
- [x] RobotController プロファイル整合(レビュー #4/#10)
  - [x] `_apply_profile_to_control_stack` 抽出(unload → set_params → 条件付き load、pid_output_limit 反映)
  - [x] `refresh_active_profile` 追加
  - [x] `src/web/routers/drive.py`: /learning/train 成功時に refresh_active_profile 呼び出し
- [x] RobotController 緊急経路(レビュー #6/#15)
  - [x] `_dispatch_emergency`(trigger_emergency 経由 + フォールバック)を DriveLoop の on_emergency に配線
  - [x] emergency_stop / stop / stop_auto_drive / shutdown で stop_and_join を home_return より前に実施(stop_manual は DriveLoop を持たないため対象外)
- [x] RobotController テレメトリ鮮度(レビュー #16): get_realtime_data のキャッシュ age > 0.5s でハードウェア読み取りへフォールバック
- [x] 走行終了時の KPI サマリログ + last_kpi_summary 公開(レビュー #7)
- [x] WS broadcast 並列送信(レビュー #14): クライアント毎 wait_for(0.5s) + gather
- [x] テスト更新
  - [x] `tests/unit/test_robot_controller.py`(123 passed)
  - [x] `tests/unit/test_web_drive.py`
  - [x] `tests/unit/test_ws_broadcast.py`
  - [x] `tests/unit/test_factory.py`

## フェーズ4: 品質チェックと修正

- [x] ruff format / ruff check がパスする(既存の指摘 2 件含めて解消)
- [x] 全ユニットテストがパスする(`pytest tests/unit` → 551 passed)
- [x] 既存テストの回 regression をすべて修正(factory/model_training/robot_controller の 4 件)

## フェーズ5: ドキュメント更新

- [x] `docs/functional-design.md` の FF+PID 出力合成・排他制御セクションを PedalArbiter 設計に更新(FF/PID/PedalArbiter/KPIMonitor の 4 セクション + UC3 シーケンス図)
- [x] 実装後の振り返り(このファイル下部に記録)

---

## 実装後の振り返り

### 実装完了日
2026-06-12

### 計画と実績の差分

**計画と異なった点**:
- **FF 境界連続化の方式変更**: design.md では「±REGIME_BLEND_BAND_KMH の対称線形ブレンド」を計画したが、実装時に dv=0(平坦基準)で巡航スロットルが半減し定常追従を悪化させることが判明。物理実体に即した「エンジンブレーキテーパ」(惰行で届く緩減速はスロットルを巡航開度から線形漸減して作る)に変更した。dv=0 で巡航開度フル、−engine_brake で 0 という連続関数になり、巡航うねりでの FF ドロップアウト(レビュー #9)を同様に解消する。
- **profile_repository の JSONB 変換**: 新キーを 1 個ずつ追加する計画だったが、`dataclasses.fields` による全フィールド自動変換に変更。今後 FeedforwardParams にフィールドを足しても repository の修正が不要になる。
- **can_reader.py の復元**: セッション中に作業ツリーの未コミット変更(キャッシュ+鮮度チェック実装)が HEAD 版へ巻き戻る事象が発生(原因不明、テストファイルは変更後のまま残存)。レビュー時点の実装を復元した上で鮮度上限 0.2 秒のパラメータ化を適用した。**未コミットのまま大規模変更を放置しない**こと。

**新たに必要になったタスク**:
- ruff 既存指摘 2 件の解消(modes.py の E402、robot_controller.py の E501)— フェーズ4 のゲート条件を満たすため。

### 学んだこと

**技術的な学び**:
- ペダル排他は「事後の消去ルール」ではなく「符号付き努力量 → 単一写像」にすると、同時踏み禁止・減速権限・アンチワインドアップ(飽和フラグ)が 1 つの構造から同時に導ける。
- 線形 FF モデルの境界不連続は、対称ブレンドのような数学的平滑化より「その領域の物理(惰行・クリープ)」で埋める方が定常特性を壊さない。
- KPI(P95・最大値・符号反転率)は固定長ヒストグラム+時刻 deque で O(1) メモリのまま 10 時間計測できる。

**プロセス上の改善点**:
- レビュー指摘 16 件を requirements.md の表に番号付きで固定し、コード内コメント・テスト docstring から「指摘 #n」で参照したことで対応漏れの照合が容易だった。

### 次回への改善提案
- 実機チューニング時は FeedforwardParams の調停定数(ヒステリシス 0.5%・再踏込ディレイ 0.3s・レートリミット 200/300%/s・PID 出力制限 50%)を学習走行の KPI サマリを見ながら調整する。`last_kpi_summary` のログが判断材料になる。
- 停車保持の解除ホライズンは現状 horizons[0](0.5 秒)に固定。発進応答が遅い車両では定数化(プロファイル昇格)を検討。
- KPI サマリの DB 保存・GUI 表示(今回スコープ外)を次の作業単位として切る価値が高い。
