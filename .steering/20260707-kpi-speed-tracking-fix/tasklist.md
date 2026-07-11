# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

### 実装可能なタスクのみを計画
- 計画段階で「実装可能なタスク」のみをリストアップ
- 「将来やるかもしれないタスク」は含めない

### タスクスキップが許可される唯一のケース
以下の技術的理由に該当する場合のみスキップ可能:
- 実装方針の変更により、機能自体が不要になった
- アーキテクチャ変更により、別の実装方法に置き換わった
- 依存関係の変更により、タスクが実行不可能になった

スキップ時は必ず理由を明記:
```markdown
- [x] ~~タスク名~~（実装方針変更により不要: 具体的な技術的理由）
```

### 実機検証ゲートについて
- 各Stage末尾の「実機検証ゲート」はユーザーがシャシダイナモで実施する（学習サイクル再実行＋WLTP_ExHi自動走行）
- ゲート判定はユーザー実施のため、実装セッションでは「実装＋単体テストまで」を完了させ、ゲート結果を待ってから次Stageに着手する

---

## Stage A: preview二重補償の解消

### A-1: モデル層のフィールド改名
- [x] `src/models/profile.py`: `DynamicsParams.preview_time_s` → `pid_preview_s: float = 0.0` に改名、docstring更新（「PIDのみ前倒し。FFはnow-frame」）
- [x] `tests/unit/infra` に旧キー `preview_time_s` 入りJSONBから `pid_preview_s=0.0` で復元されるロードテストを追加（`test_legacy_preview_time_s_key_ignored_defaults_to_zero`）

### A-2: drive_loopのフレーム分離
- [x] `src/domain/control/drive_loop.py`: `self._preview_s` → `self._pid_preview_s`（プロファイルから取得、`max(0.0, ...)`）
- [x] FF入力をnow-frame化: `predict_effort(ref_speed, ...)`、`future_speeds`/`past_speeds` を `elapsed_s` 基準に変更、`t_ctrl` 廃止
- [x] PIDのみ `ref_speed_pid = self._ref_speed_at(elapsed_s + self._pid_preview_s)` を渡す

### A-3: チューニング側の追従
- [x] `src/domain/pid_tuning.py`: `PID_PREVIEW_MAX_S = 1.0` に変更（旧 `PREVIEW_MAX_S=3.0`）、`TuningParams` のキー改名（`_PARAMS`/`_BASE`(0.5→0.25)/`_CLAMP`）
- [x] `initial_preview_from_fopdt` を「常に0.0を返す」に変更（θ初期値は系統誤差を再導入するため）
- [x] `src/app/training_service.py`: dynamics_params構成時に `pid_preview_s=0.0`
- [x] `src/app/learning_cycle.py` / `src/app/robot_controller.py::run_pid_tuning_session`: フィールド名追従

### A-4: Web層の追従
- [x] `src/web/schemas.py`: `DynamicsParamsSchema` / `PidRefineResponse` / `CycleProgressSchema` のキー改名
- [x] `src/web/routers/drive.py`: 改名追従
- [x] `src/web/static/js/screens/profiles.js` / `learning.js`: フォーム・進捗表示のキー改名（ラベル「PID先読み補償 [s]」）

### A-5: テストと品質チェック
- [x] `tests/unit/test_drive_loop.py` のpreview系テストを新仕様に置換（①FFはnow-frame基準/PIDのみシフト ②FF先読み/過去もnow-frame ③pid_preview_s=0でFF・PID両方now-frame ④PID先読みの終端クランプ）
- [x] `tests/unit/test_pid_tuning.py`: 初期preview=0.0（θ非依存）、クランプ上限1.0(PID_PREVIEW_MAX_S)に更新
- [x] `tests/unit/test_models.py` / `test_training_service.py` / `test_learning_cycle.py` / `test_robot_controller.py` / `test_web_api.py`: 改名追従
- [x] `pytest tests/unit` 全緑（947 passed）＋ `tests/integration/test_web_api.py`（33 passed）
- [x] `ruff check` 通過 / `mypy` はStage A変更箇所にエラーなし（残5件は既存の未コミット変更由来でStage A無関係）

### A-6: 実機検証ゲートA（ユーザー実施） — 2026-07-08 実施 (session ccb3bf89)
- [x] WLTP_ExHi自動走行を実施し、ログのラグ走査で corr最大ラグ 0±0.1s を確認 → **合格**: 最良ラグ −0.4s→**0.0s**（系統ラグ解消）。同一モデル・同一ゲインのクリーンA/B
- [x] max偏差・p95が現状値（3.44 / 1.97 km/h）から改善していることを確認 → **部分的**: p50 0.475→0.239(半減)、p95 1.97→1.45、違反時間25.6%→14.2%(45%減)は改善。**但し max 3.44→3.40 はほぼ不変、符号反転 max/5s は 19→25 に悪化**（系統バイアス除去で潜在振動が露出）
- 診断: 残違反は停止移行(t=313-319, +3.40)・急減速(refdot<-2.5)・高速域ハンチング(周期1.5-7s、高速ほど悪化)に集中。いずれも第2・3層（FF残差＋速度依存ゲイン＋固定PID振動＋再現性スパイク）= Stage B/C の対象。現ゲインは旧・二重補償下で適合された値でありnow-frame構造に不整合 → Stage B再チューニングが直接是正

## Stage B: チューニングプロセス改善

### B-1: KPI超過積分とコスト連続化 ✅
- [x] `src/domain/control/kpi_monitor.py::update`: 前回now_sとの差分dt(クランプ[0,0.5s])で `over_limit_integral += max(0,|dev|-1.0)*dt` を逐次加算、`summary()` に `over_limit_integral_kmhs` / `time_over_limit_s` 追加（drive_loop呼び出し不変）
- [x] `src/domain/pid_tuning.py::tuning_cost`: `100·hard` → `10·over_integral + 2·max_dev/1.0 + 1·(hard>0)` に置換
- [x] `tests/unit/test_kpi_monitor.py`: 積分の解析値一致・違反ゼロ→0・大dtクランプ
- [x] `tests/unit/test_pid_tuning.py::TestTuningCost`: 超過積分の単調性・hard段差(1.0)非支配・違反ゼロ優位

### B-2: 本番モード代表区間での評価走行 ✅
- [x] `src/domain/pid_tuning.py::build_tuning_trajectory_from_mode(mode, max_duration_s=120)` 新設（最高速度窓＋加減速最大窓を切り出し接続、0→接続速度/窓間/末尾→0ランプ、時間重複畳み込み）。実機WLTP_ExHi(323s)→78.7s/peak131.3で検証
- [x] `src/app/robot_controller.py::run_pid_tuning_session` / `_run_tuning_drive` に `mode: DrivingMode | None = None` 引数追加
- [x] `src/infra/settings.py`: `refine_runs_stage2: 5→12`、`tuning_on_target_mode: bool = False` 追加（**ユーザー指定で既定OFF＝学習サイクルは規定パターンで適合**。ゲインスケジューリングにより規定パターン適合ゲインが高速域へ転移するため本番モード適合は任意。standalone refineのmode_idは常に利用可）
- [x] `src/app/learning_cycle.py`: REFINE_2で対象モードを渡す（`mode_repo`/`tuning_on_target_mode`/`target_mode_id` 配線、`_resolve_tuning_mode`）。app.py factory注入
- [x] `src/web/routers/drive.py::pid_tune_refine`(mode_id)/`start_learning_cycle`(target_mode_id) / `schemas.py`: 任意 mode_id 追加
- [x] テスト: 代表区間の安全包絡（max_speed以内・ランプ率≤元モード最大・0始端0終端・単調時間・予算内・高速域包含）、mode引数の配線（REFINE_1=None/REFINE_2=代表区間）

### B-3: FFモデル局所勾配によるゲインスケジューリング ✅（ブレーキ側含む＝ユーザー選択）
- [x] `src/domain/control/feedforward.py`: `GainSchedule`（accel/brake両側の速度-ゲイン表＋補間）と `build_gain_schedule(v_max, step=5.0)` 新設（定常特徴で±probeの前進差分、クランプ[0.02,1.0]＋3点移動平均）。設計の`build_gain_table→list[tuple]`から、ブレーキ側も扱うため`GainSchedule`データクラスに拡張。`rebuild_gain_schedule`/`gain_schedule`プロパティ、unload時クリア
- [x] `src/domain/control/pid.py::update` に `gain_scale: float = 1.0` 追加（出力乗算・積分クランプ `output_limit/(ki·gain_scale)` 整合・scale≤0で1.0フォールバック）
- [x] `src/domain/control/drive_loop.py::_gain_scale`: `scale = clamp(g(actual)/g_nominal, 0.5, 3.0)`、`g_nominal=1/fopdt_k`、減速トレンド(ref基準)ならbrake側/それ以外accel側、schedule/fopdt_k無しはscale=1.0
- [x] `src/app/robot_controller.py::_apply_profile_to_control_stack`: プロファイル選択時に `rebuild_gain_schedule(max_speed)` 構築
- [x] テスト: `test_pid.py`（gain_scale比例性・scale=1回帰・積分クランプ整合）、`test_feedforward.py`（線形ダミーで解析勾配一致・クランプ・brake側・速度依存・rebuild/unload）、`test_drive_loop.py`（schedule/fopdt_k無しでscale=1、加速/減速でaccel/brake切替・クランプ）

### B-4: 解析スクリプト整備と品質チェック ✅
- [x] `scripts/analyze_session.py` 新設: session_id/`--latest`でdrive_logsから |偏差|p50/p95/max・違反時間・ラグ走査・5秒窓符号反転をKPI合否付きで出力。実機2セッション(717e2609/ccb3bf89)で手動解析値と一致を確認
- [x] `pytest tests/unit`（975）＋ `tests/integration/test_web_api.py`（33）全緑、`ruff check` 通過、`mypy` Stage B新規エラーなし（残5件は既存）

### B-5: 実機検証ゲートB（2026-07-09 実施, cycle 6027f43f / session ff431d46）— **部分合格**
- [x] 学習サイクル再実行でコスト単調減少・4座標が動くことを確認 → **合格**: cost 67.5(stage1)→28.1(stage2)、kp 0.75→2.84 / ki 0.74→1.76 / kd 0→0.345 / pid_preview_s 0→0.075 全て移動。FOPDT再同定 k=1.489/τ=1.003/θ=0.400
- [x] WLTP_ExHiフル走行 → **p95=0.492 ✅(≤0.5) / max=1.589 ❌(≤1.0) / 符号反転23回/5s ❌(≤3)**。p50=0.149、違反時間1.9s(0.6%、前回45.9s)、ラグ0.00s維持
- [x] 速度帯別偏差分散: steady 80-110 std=0.191 / 110-132 std=0.200 → 帯域間の不均衡は解消
- **残NGの診断（机上検証済み）**:
  - max違反は**停止移行のみ**（t=314-318の3区間、減速-3.4〜-4.2km/h/s、ブレーキ15-32%でも+1.0〜+1.59） → 0.25Hzの再現性誤差＝**Stage C ILCの帯域内・対象のまま**
  - 符号反転は**高速域(123-130km/h)の±0.2〜0.4km/h・約2.3Hzチャタ** → ILC帯域外（cutoff 0.3Hz）のため**B-6で対処必須**
  - 原因1: **ゲインスケジュール退化** — probeの過渡勾配(生値2.3〜6.0 %/(km/h))にクランプ[0.02,1.0]（定常勾配想定の校正）が全速度域で上限飽和し「全域一律×1.49」に退化。g_nominal=1/k(定常)とprobe(過渡)の次元不整合。speed_clip境界(130-140)で勾配が人工的に減衰するアーティファクトも確認
  - 原因2: tuning_costのreversal重み0.2が弱く、チューナーが規定パターン上の振動(14回/5s)を受容（0.2×14=2.8 vs p95項）

### B-6: ゲインスケジュール是正＋高速域チャタ対策（ゲートB残NG対応・Opus実装）
- [x] `feedforward.py`: `_steady_features`→`_accel_features(v, a)`（定加速度マニフォールド future_h=v+a·h / past_h=v−a·h、負値0丸め）。`build_gain_schedule` を a=±0.5km/h/s 差分で g=∂opening/∂a に修正
- [x] `g_nominal` 次元整合: drive_loop `_g_nominal = fopdt_tau/fopdt_k`（tau/k 未同定時 None）
- [x] クランプ再校正: `_GAIN_MIN/_GAIN_MAX = 0.02/1.0` → **0.05/5.0**、probe定数 `_GAIN_PROBE_DV_KMH`→`_GAIN_ACCEL_PROBE_KMHS=0.5`
- [x] speed_clip境界対策: グリッド上限 `min(v_max, speed_clip_max − max_horizon·a_probe)`、超過速度は補間端点クランプ
- [x] `pid_tuning.py::tuning_cost` の reversal 重み **0.2→1.0**（`TestTuningCost` 期待値更新）
- [x] 修正後スケジュールの机上検証: 実機モデル(_20260709_090619)で g(v)/scale(v) を出力。旧「全域一律×1.49」→**速度で変化する表**に是正。g_nominal=τ/k=0.674。brake側は0.05→1.97で単調増加（scale 0.5→2.92）。accel側は0-110で0.95→4.08（scale 1.4→3.0クランプ）だが120-140でモデルデータ不足により0.05へ崩落→scale下限0.5。**高速域(123-130)チャタ帯でscaleが1.49→0.5に下がる=FF残差追従を弱める方向でチャタ抑制に therapeutic**。低速ブレーキ/高速アクセルのゲイン崩落は「不確かなら弱める（scale 0.5）」の安全側フェイル。停止移行の弱PID化はStage C対象で許容
- [x] `pytest tests/unit`（978）＋integration（33）全緑、`ruff check` 通過、`mypy` 新規エラーなし（残5件は既存）

### B-6': 実機検証ゲートB再判定（2026-07-09 実施, cycle 5ab93a2e / session 343e80c1）— **部分合格・ペダルハンチング問題が発覚**
- 経路: 学習サイクル（規定パターン）再実行 → WLTP_ExHi自動走行 → `scripts/analyze_session.py --latest auto`
- [x] max≤1.0/p95≤0.5/符号反転≤3回/5s → **p95=0.448 ✅ / max=1.372 ❌（高速ピークt=247.9の-1.37と停止移行）/ 符号反転20回/5s ❌**。p50=0.100、違反時間0.7s(0.2%、前回1.9s)、ラグ0.00s維持 — KPIは全項目で改善傾向だが反転が残存
- [x] 高速域(110-132)のsteady偏差std → 0.200→0.227 と微増（悪化ではないが改善もなし。チャタが支配）
- **B-6'で新たに確定した診断（生ログ解析）**:
  - **ペダルハンチング**: アクセルON 90回/322.9s のうち110-135km/h帯に36.2回/min集中。128-131km/h巡航で開度0.9〜2.3%が0.6秒周期振動（1.7Hzリミットサイクル）。coast遷移の「アクセル即0解放」がパルス化の機構
  - **モデルデータ欠損**: stage2学習データの125-140km/h×アクセル0-6%がほぼゼロ件（チャタ帯＝欠損帯＝規定パターン非カバー帯が一致）。規定パターン最高0.8比=112がWLTP巡航131に届かないのが根本
  - **学習運転の過速度**: max_speed=140に対し実測max 153.9〜156.1（>140が14区間）。cap離脱判定が反応的で応答遅れ~1.2s×13.8km/h/s=+16km/hオーバーシュート
  - 対応は B-7（design.md 4b節）。PID/previewのアクセル・ブレーキ分離は検討の結果不採用（B-7-6に記録）

### B-7: ペダルワーク平滑化と適合走行の拡張（ゲートB第3次対応・Opus実装）

実装順序: B-7-5 → B-7-1 → B-7-3 → B-7-4 → B-7-2 → 品質チェック（設計は design.md 4b節）

#### B-7-5: 学習運転の予測的cap離脱（安全・最優先）✅
- [x] `src/domain/control/learning_loop.py`: DRIVE_ACCEL離脱条件を `speed >= cap − max(0, accel_kmhs) × overspeed_lead_s` に変更（config新設 `overspeed_lead_s=1.2`、既存の平滑加速度 `_smoothed_accel` を使用）
- [x] 既存バックストップ（>max_speed→DRIVE_BRAKE、過速度スキップ判定）は不変で回帰テスト維持
- [x] `tests/unit/test_learning_loop.py::TestPredictiveCapExit`: 離脱しきい値の直接検証（加速中は cap 手前・lead=0は cap・プラトーは cap）＋一次遅れプラント（tau=1.0, 70%→14km/h/s）シミュレーションで **lead=1.2 でピーク136.1km/h ≤140、lead=0 でピーク151.5km/h >140（実機153-156の再現）** を対比検証。44 passed

#### B-7-1: 規定パターン拡張 ✅
- [x] `src/domain/pid_tuning.py::build_tuning_trajectory`: 速度点 `0→[fast]0.6v hold6→[slow]0.8v hold6→[slow]0.95v hold10→[fast]0.3v hold6→[stop]0`、区間別レート係数（fast=0.8/slow=0.35/stop=0.4 × max_decel_g、モジュール定数化、床 MIN_RATE_KMHS）。max_speed=140で 0→84→112→133(巡航10s)→42→0、レート3種(11.3/4.9/5.7km/h/s)、総60.8s（旧42s）
- [x] `tests/unit/test_pid_tuning.py`: 巡航保持の存在（peak=0.95v・保持≥10s）・レート多様性（≥2種）・全区間レート≤0.8×max_decel_g（床考慮）を検証。旧 test_decel_rate_within_max_g は加速側も含む test_ramp_rate_within_max_g に置換。40 passed

#### B-7-3: ペダルワーク平滑化 ✅
- [x] `src/models/profile.py::FeedforwardParams`: `accel_release_rate_pct_s`（既定10.0）・`accel_min_step_pct`（既定0.2）追加。profile_repository の汎用 `_dataclass_from_jsonb` が既存JSONBの欠損キーを既定補完（マイグレーション不要）
- [x] `src/domain/control/pedal_arbiter.py`: (a) coast遷移で `_release_accel` により直前アクセルを解放レートで漸減（rate≤0は即0解放で後方互換。effort<−h のブレーキ要求時は accel=0 即解放を維持） (b) アクセル選択時 |applied−前回| < accel_min_step_pct なら前回値保持
- [x] `src/web/schemas.py::FeedforwardParamsSchema`: 新2フィールド追加（`_ffp_from_schema` が全dataclassフィールドをschemaから読むため必須。ge=0バリデーション）
- [x] `tests/unit/test_pedal_arbiter.py`: TestAccelReleaseSmoothing（1サイクル惰行は0.5%のみ下降・再加速で復帰・継続惰行は0まで解放・ブレーキ要求は即解放・rate=0後方互換・同時踏み禁止）＋TestAccelQuantization（保持/追従境界）。90 passed（arbiter+profile_repo+web_api）

#### B-7-4: チューニングコストにペダル活動度項 ✅
- [x] `src/domain/control/kpi_monitor.py::update`: 任意引数 `accel_opening` を追加し `accel_on_count`（0→非0立ち上がり、しきい値0.5%）・`accel_on_per_min`（走行時間で正規化）・`pedal_travel_pct`（総移動量）を summary に追加。None なら偏差KPIのみ（後方互換）
- [x] `src/domain/control/drive_loop.py`: KPIMonitor.update に `accel_opening=self._current_accel_opening`（調停後）を渡す
- [x] `src/domain/pid_tuning.py::tuning_cost`: `+ PEDAL_ACTIVITY_WEIGHT × accel_on_per_min`。**重み机上校正=0.1**（実機 ff431d46=21.7回/min・343e80c1=16.7回/min。ハンチング寄与1.7〜2.2 が p95項2.25〜2.5 と同オーダー、支配項 reversal21〜24 は上書きしない。20→5回/min改善≒p95換算0.3km/h）
- [x] `tests/unit/test_kpi_monitor.py::TestPedalActivity`（立ち上がりカウント・しきい値・総移動量・時間正規化・None後方互換）／`test_pid_tuning.py::TestTuningCost`（単調性・重み一致・欠損0・reversal非支配）。131 passed（kpi+tuning+drive_loop）

#### B-7-2: 学習運転 CRUISE_TRIM パターン追加 ✅
- [x] `src/models/learning_drive.py`: `PatternKind.CRUISE_TRIM` 追加＋`LearningPattern.trim_opening: float = 0.0`（cap到達後に保持する微小開度。他種別は0.0でデフォルト、frozen dataclassで後方互換）
- [x] `src/domain/learning_drive.py`: cap まで70%加速→微小開度（既定 1.5/3.0%＝2本）を保持するパターン生成（`CRUISE_TRIM_ACCEL_PCT`/`CRUISE_TRIM_OPENINGS_PCT`/`CRUISE_TRIM_HOLD_S=8.0` 定数。generate_patterns 手順7）
- [x] `src/domain/control/learning_loop.py`: `_Phase.CRUISE_TRIM` 追加。DRIVE_ACCEL（予測的cap離脱）→CRUISE_TRIM保持のフェーズ遷移。`_advance_cruise_trim`（hold_duration/低速で次パターン・cap以上復帰で離脱・max_speed超で能動減速）。ガバナ・過速度バックストップは共通
- [x] `tests/unit/test_learning_drive.py::TestCruiseTrim`（2本生成・開度一致・cap加速クランプ・順序）／`test_learning_loop.py::TestCruiseTrim`（cap到達で遷移・trim保持・hold離脱・cap復帰で安全離脱・max_speed超で回復ブレーキ）。81 passed
- [x] `requirements.md` スコープ外条項を「B-7-2/5 の範囲で変更する」に改訂済み

#### B-7 品質チェック ✅
- [x] `scripts/analyze_session.py::_pedal_activity` 追加（accel ON回数/min・110-135km/h帯の立ち上がり密度・OFF滞在中央値、`accel ON 110-135 ≤10回/min` のゲート判定付き）。実機 343e80c1 で 35.8回/min・OFF中央値0.10s と手計算一致
- [x] `pytest tests/unit`（1049）＋ `tests/integration/test_web_api.py` 全緑、`ruff check` 通過、`mypy` はB-7変更8ファイルで「Success」＝新規エラーなし（全体の残6件は既存＝calibration/profile_repository/robot_controller/app.py/analyze_session の未変更箇所）

### B-7': 実機検証ゲートB最終判定（ユーザー実施）— **合格**（2026-07-10 ユーザー確認）
- 経路: 学習サイクル再実行 → WLTP_ExHi自動走行 → `scripts/analyze_session.py --latest auto`
- [x] 学習運転の実測最大車速 ≤ max_speed(140)
- [x] stage2学習データの 125-140km/h × アクセル0-6% セルにサンプルが入っていること（カバレッジ確認）
- [x] WLTP: p95≤0.5維持・max≤1.0（停止移行はStage C対象として残置可）・符号反転≤3回/5秒窓
- [x] ペダル活動度: 110-135km/h帯の accel ON ≤ 10回/min 目安（現状36.2回/min）
- → Stage B 完了。Stage C（ILC）着手。

### B-8: ペダルシーソー対策 → **新ステアリング `20260711-human-pedal-plan` へ移行**（2026-07-11 ユーザー決定）

C-6初回実機（2026-07-10 cycle abd4632a / auto e3773d9c 13:13）でアクセル⇔ブレーキ交互踏み
39.4回/min・p95=1.330/max=5.205 に悪化。直接原因は実効PIDゲイン過大（kp=3.91×scale上限3.0
張り付き=11.7 %/(km/h)、SIMC適正≈1.6の7倍）。ユーザー承認済み設計判断: ブレーキ抑制しきい値
0.5km/h / PID不感帯±0.1km/h / scale上限1.5。

**移行の経緯**: B-8計画レビューでユーザーから「人間のペダル操作スタイル（クリープ発進・開度
なるべく一定・エンジンブレーキ優先・停止中ブレーキ保持）をKPIと両立せよ。大幅修正可」の要求が
提示され、B-8の部分修正では実現不可能（20HzのFF+PID連続変調構造が残る）と判断。制御を
「ペダルプラン＋低速トリム」へ再構成する新ステアリングに一本化した。B-8-1/2は新アーキテクチャ
に吸収、B-8-3/4は移管（診断・実測値は design.md 4c節が正本として残る）。

#### B-8-1〜B-8-4 の処置（2026-07-11）
- [x] ~~B-8-1: PID誤差不感帯（pid.py error_deadband / DynamicsParams.pid_error_deadband_kmh / スキーマ追従）~~（実装方針変更により不要: 新アーキテクチャの TrimController 凍結帯 HOLD_BAND_KMH=0.1 が同じ役割をより上位で担う。`20260711-human-pedal-plan` design.md 2節）
- [x] ~~B-8-2: ブレーキ抑制ガード（drive_loop allow_brake / pedal_arbiter）~~（実装方針変更により不要: 新アーキテクチャのペダルプラン・フェーズ権限（DRIVE中はブレーキ不可、速い補正層のみ例外）がガードの一般形。同 design.md 3節）
- [x] ~~B-8-3: _GAIN_SCALE_MAX 3.0→1.5~~（新ステアリングへ移管: `20260711-human-pedal-plan` tasklist P-4 に同内容を収録）
- [x] ~~B-8-4: シーソー計測・コスト項・analyze_session ゲート~~（新ステアリングへ移管: 同 tasklist P-6。なおゲートは固定3回/minでなく**モード相対（超過切替≤2回/min）**に修正 — 全12モードの必要切替床の机上計算で 09_US06=3.7/min・03_WLTP_Low/10_SC03=3.2/min と固定3では走行不能なため）
- [x] ~~B-8 品質チェック / B-8' 実機検証~~（新ステアリング P-7/P-8 に統合。ILCリセット→学習サイクル再実行→初回 max≤1.0→ILC収束の手順は P-8 が引き継ぐ）

## Stage C: 反復学習制御（ILC）

### C-1: ドメイン層 ✅
- [x] `src/domain/control/ilc.py` 新規: `ILCTable`（efforts・dt_s=0.1・iteration・best_p95_kmh・duration_sプロパティ）
- [x] `ILCController.effort_at(elapsed_s)`: np.interp 線形補間＋端点クランプ＋±amp_limit(既定10%)クランプ、空テーブルは0
- [x] `ILCLearner.update`: `u_{j+1}(t)=clip(Q(u_j(t)+L·e_j(t+Δ)), ±amp)`、Qはゼロ位相ローパス（numpy forward-backward EWMA `_zero_phase_lowpass`）。`l_gain_from_fopdt`（L=0.4/k）・`is_diverged`（p95>best×1.2）も追加
- [x] `tests/unit/test_ilc.py`: 線形静的プラント8反復収束（2.0→0.36、単調減少）・発散検知・振幅クランプ・ゼロ位相性（対称山ピーク不動）・Δシフト端点クランプ・補間/クランプ。17 passed

### C-2: 永続化層 ✅
- [x] `scripts/setup_db.py`: `ilc_tables` 追加（profile_id×mode_id PK, enabled, iteration, dt_s, efforts JSONB, best_p95_kmh, kpi_history JSONB, updated_at、CASCADE削除）
- [x] `src/infra/ilc_repository.py` 新規: `ILCRecord` ＋ `get / upsert(enabled保持) / reset / reset_for_mode / set_enabled`
- [x] `tests/unit/infra/test_ilc_repository.py`: JSONB文字列/native両パース・get/upsert/reset/set_enabled のSQL検証。8 passed

### C-3: アプリ層の配線 ✅
- [x] `src/app/ilc_service.py` 新規: `prepare(profile, mode)→ILCController|None`（無効/空/失敗はNone）／`learn_from_session`（ログ不足・無効・発散でスキップ、fire-and-forget前提で例外握りつぶし）
- [x] `src/domain/control/drive_loop.py`: `ilc: ILCController|None=None` 注入、`arbitrate(ff+pid+ilc_effort, dt)` 合成、ilc_effortをDEBUGログ（DriveLogData不変）
- [x] `src/app/robot_controller.py`: `ILCServiceProtocol`、`start_auto_drive` で `prepare` 配線＋正常完了専用 `_finish_auto_drive_with_ilc`（stop_auto_driveでログflush後にlearn を ensure_future。手動/緊急停止はこの経路を通らず学習しない）
- [x] `src/app/factory.py`: `ILCRepository`/`SessionRepository`/`ILCService` 構築・注入
- [x] `src/web/routers/modes.py`: `replace_mode`（PUT＝基準軌跡変更）成功後に `ilc_repo.reset_for_mode`（PATCHはメタデータのみで軌跡不変のため据え置き。DELETEはDB CASCADE）
- [x] テスト: `test_ilc_service.py`（11）、drive_loop の ilc合成/None回帰（`TestILCEffortSynthesis` 3）、robot_controller の `TestILCWiring`（prepare→DriveLoop配線・正常完了で学習・手動停止で非学習 3）

### C-4: WebUI ✅
- [x] ILC状態API: `GET /api/v1/drive/ilc/{profile_id}/{mode_id}`、`POST .../enable|disable|reset`（`get_ilc_repo` dep＋`ILCStatusResponse`。未登録は iteration0/enabled true/has_table false）
- [x] `src/web/static/js/screens/auto-drive.js`: `ILCPanel`（反復学習 第N回・有効/無効・最良p95・kpi_history収束列・有効化/無効化/リセットボタン。走行中は操作不可・READY復帰で自動refresh）。mode ベース自動走行の session-info に表示（schedule除く）。Babel react preset で構文検証済み
- [x] `app.py`/`deps.py`/`stubs.py`（`InMemoryILCRepository`）配線。`tests/integration/test_web_api.py` にILC 3テスト追加（既定値・enable/disable往復・reset）

### C-5: 品質チェック ✅
- [x] `pytest tests/unit`（1061）＋`tests/integration/test_web_api.py`（36）全緑＝**1094 passed**、`ruff check` 通過、`mypy` Stage C 12ファイルで新規エラーなし（残2件 robot_controller:722/app.py:100 は既存）
- [x] setup_db.py を実機DBに適用（ilc_tables 作成確認済み）

### C-6: 実機検証ゲートC（ユーザー実施・最終合格判定）

**2026-07-10 初回試行はペダルシーソー＋KPI悪化（p95=1.330/max=5.205）で中断。対策は B-8 →
新ステアリング `20260711-human-pedal-plan`（ペダルプラン＋トリム構成）に移行した。
KPI達成の枠組みもユーザー指示で改訂（2026-07-11）: ILC収束をKPIの前提にせず、学習サイクルの
最終フェーズに検証走行（登録全モードを包絡する約5分の専用パターン・ILC無効）を追加して
サイクル内でKPI合格させ、**サイクル完了後の最初の自動運転からKPI満足**とする。
このゲートは新ステアリング tasklist P-9 の実機検証をもって完了扱い（ILCはモード別の
任意強化として存続）。**

- [ ] WLTP_ExHiをILC有効で連続5〜8回走行し、kpi_historyで収束カーブ確認
- [ ] 連続2走行で p95≤0.2 / max≤1.0（hard=0） / 符号反転≤1回/5秒窓 を全満足
- [ ] 停止移行区間（走行終盤）の違反吸収を区間別に確認
- [ ] ILCリセット後1走行目（補正なし）でもmax≤1.0（Stage Bゲート）を維持

## ドキュメント更新

- [x] `docs/architecture.md`: 2系統制御ループのeffort合成（FF+PID+ILC）・pid_preview_s/ゲインスケジューリング記述、ILC制御フロー節、ディレクトリツリー（ilc.py/ilc_service.py/ilc_repository.py等）、ストレージ表にilc_tables追加
- [x] `docs/functional-design.md`: 構成図にILCService、合成コード（FF+PID+ILC）、KPIMonitorのペダル活動度、ILCService節（prepare/learn/WebUI）、ILCTableエンティティ＋ER図
- [x] `docs/glossary.md`: 反復学習制御(ILC)・ゲインスケジューリング・PID先読み補償(pid_preview_s) の3用語追加
- [x] 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
- Stage A: 2026-07-08（ゲートA合格）
- Stage B（B-1〜B-6'）: 2026-07-09〜07-10（ゲートB最終=B-7'合格）
- Stage C（ILC）: 2026-07-10 実装完了（C-6 実機ゲートはユーザー実施待ち）

### 計画と実績の差分

**計画と異なった点**:
- **B-3 ゲインスケジューリング**: 設計の `build_gain_table→list[tuple]` から、ユーザー選択でブレーキ側も対象にしたため `GainSchedule` データクラスへ拡張。さらにB-6で probe を定常特徴→定加速度マニフォールドに、g_nominal を 1/k→τ/k に是正（過渡勾配との次元整合）。
- **B-2 tuning_on_target_mode**: 既定OFF（学習サイクルは規定パターンで適合）。ゲインスケジューリングで規定パターン適合ゲインが高速域へ転移するため本番モード適合は任意とした。
- **B-7（計画外の追加ステージ）**: ゲートB(B-5)は KPI 改善したがペダルハンチング（高速域 36回/min）が新たに顕在化し、B-6'実機で「規定パターンが巡航帯125-140をカバーせずモデルデータ欠損」「学習運転が最高車速156km/hオーバー」も判明。当初計画に無かった B-7（規定パターン拡張・CRUISE_TRIM・ペダル解放平滑化・コストのペダル活動度項・予測的cap離脱）を追加した。
- **ILCTable の best_p95_kmh をDB列に追加**: 設計SQLには無かったが発散検知の永続化に必要で列追加。
- **LearningPattern.trim_opening フィールド追加**: CRUISE_TRIM が cap加速用と保持用の2開度を要するため（frozen dataclass に既定0.0で後方互換）。

**新たに必要になったタスク**:
- B-7-3 のスキーマ追従（`_ffp_from_schema` が全dataclassフィールドをschemaから読むため `FeedforwardParamsSchema` に新2フィールド追加が必須だった）。
- `InMemoryILCRepository`（DBなし環境のWebUI用）と app.py/deps.py 配線。
- モード基準軌跡変更時の ILC リセット（`replace_mode` のみ。PATCHは据え置き）。

### 学んだこと

**技術的な学び**:
- ペダルハンチングの機構: 定常巡航で必要開度~2%に対しPIDが20Hz即応→effortがヒステリシス帯を跨ぐ→調停器がアクセルを即0解放、のパルス化。解放レート制限＋量子化で機構的に抑制。
- モデルデータ欠損＝チャタ帯＝規定パターン非カバー帯が完全一致。適合の規定パターンが本番の速度域を包含していないと、その帯でFFが外挿しゲインスケジュールも崩落する。
- 学習運転の過速度は「離脱判定の遅れ」＝アクセル解放後も応答遅れ(θ+τ)の間 惰性で+16km/h伸びる。予測的離脱 `cap − 加速度×lead` で是正（一次遅れシミュレーションで 151→136km/h）。
- ILC のゼロ位相ローパスは低周波の系統誤差なら単調収束するが、半サインのような端で急峻な誤差はフィルタ歪みで下げ止まる（テストは中央ガウス山で検証）。ILC が有効なのは反復再現性のある帯域内(<0.3Hz)の残差で、高速域チャタ(2.3Hz)は帯域外＝B-7-3で別途対処という役割分担。

**プロセス上の改善点**:
- 生ログを KPIMonitor/analyze_session で机上再現してから重み・しきい値を校正した（tuning_cost のペダル重み0.1は実機2セッションで校正）。「解決した」を自己選択した閾値のサマリで言い切らず生データで確認する方針が機能した。
- 各Stageを実機ゲートで区切り、ゲート結果の診断（どの層の問題か）を次Stageの設計入力にする直列運用が有効だった。

### 次回への改善提案
- 規定パターンは「本番モードの速度域・ランプ分布を包含する」ことを適合の前提条件として明示すべき（B-7-1で0.95比巡航を追加したが、モード依存の代表区間適合=B-2 を既定にする選択肢も再評価余地あり）。
- ILC の寄与を可視化するため、将来 DriveLogData に ilc_effort 列を追加すると実機での分解分析が容易になる（今回はスキーマ変更回避でDEBUGログに留めた）。
- C-6 実機ゲートで発散検知の係数(1.2)や L=0.4/k が過大/過小なら、fopdt再同定値で再校正する。
