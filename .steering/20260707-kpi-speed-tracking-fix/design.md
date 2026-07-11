# 設計書

## アーキテクチャ概要

既存のeffort経路（FF + PID → PedalArbiter → ペダル開度）を維持したまま、3段階で追従精度を改善する。

```
基準速度(now-frame) ──┬─→ FF (poly_spec_inverse_lookahead, horizons先読み内蔵) ─┐
                      ├─→ PID (pid_preview_s だけ前倒し可・gain_scale正規化) ──┼─→ Σ effort → PedalArbiter → ペダル
                      └─→ ILC (profile×mode の時刻別補正テーブル) ─────────────┘
                                        ↑
                     走行完了後: drive_logs の残差 e_j(t) から次回テーブルを学習
```

- **Stage A**: FFへの前倒し(preview)を廃止し now-frame 化。previewはPID専用ノブに分離
- **Stage B**: チューニングコストの連続化＋本番モード代表区間での評価＋FFモデル局所勾配によるPIDゲインスケジューリング
- **Stage C**: ILC（反復学習制御）を第3のeffort項として追加

## コンポーネント設計

### 1. Stage A: フレーム分離（drive_loop / profile / pid_tuning）

**責務**:
- FFは now-frame 基準（`elapsed_s`）で動く。先読みはモデルの horizons（0.5/1.0/2.0/3.0s）が担う
- PIDのみ `pid_preview_s` だけ前倒しした基準を追える（既定0.0）

**実装の要点**:
- `src/models/profile.py`: `DynamicsParams.preview_time_s` → **`pid_preview_s: float = 0.0`** に改名。docstringに「PIDの基準サンプリングのみ前倒し。FFはnow-frame（先読みはモデルのhorizonsが担う）」と明記
- **DBマイグレーション不要**: `profile_repository._dataclass_from_jsonb` はdataclass fieldsのみでkwargsを構成するため、既存JSONBの旧キー `preview_time_s: 0.5` は読込時に自然に無視され、新フィールドは既定0.0で補完される（暗黙リセット。θは `fopdt_theta` に保存済みのまま）。次回profile更新時にJSONBが新キーで書き直される
- `src/domain/control/drive_loop.py` (`_execute_one_cycle`, 現行:220-259):
  - `self._preview_s` → `self._pid_preview_s = max(0.0, profile.dynamics_params.pid_preview_s)`
  - FF入力を now-frame 化: `predict_effort(ref_speed, ...)`、`future_speeds = [self._ref_speed_at(elapsed_s + h) for h in self._ff.horizons]`、`past_speeds` 同様
  - PIDのみ `ref_speed_pid = self._ref_speed_at(elapsed_s + self._pid_preview_s)` を渡す。`t_ctrl` 変数は廃止
- `src/domain/pid_tuning.py`:
  - `PREVIEW_MIN_S/PREVIEW_MAX_S(=3.0)` → **`PID_PREVIEW_MAX_S = 1.0`**
  - `TuningParams.preview_time_s` → `pid_preview_s`（`_PARAMS`/`_BASE`(0.5→0.25)/`_CLAMP` のキーも改名）
  - `initial_preview_from_fopdt` は **0.0 を返す**（θを初期値にすると系統誤差を再導入するため。探索で必要なら上がる）
- 機械的改名の追従: `src/app/training_service.py`（dynamics_params構成時に `pid_preview_s=0.0`）、`src/app/learning_cycle.py`、`src/app/robot_controller.py::run_pid_tuning_session`、`src/web/schemas.py`（DynamicsParamsSchema/PidRefineResponse/CycleProgressSchema）、`src/web/routers/drive.py`、`src/web/static/js/screens/profiles.js`・`learning.js`（ラベル「PID先読み補償 [s]」）

### 2. Stage B-1: KPI超過積分とコスト連続化（kpi_monitor / pid_tuning）

**責務**:
- ハード違反の「回数」ではなく「超過量×時間」の連続量でチューナーに勾配を与える

**実装の要点**:
- `src/domain/control/kpi_monitor.py::update`: 前回 `now_s` との差分dtで `self._over_limit_integral += max(0.0, abs_dev - KPI_HARD_LIMIT_KMH) * dt` を逐次加算（drive_loop側の呼び出しシグネチャは変更不要）。`summary()` に `over_limit_integral_kmhs` と `time_over_limit_s` を追加
- `src/domain/pid_tuning.py::tuning_cost`: `100.0 * hard` → **`10.0 * over_limit_integral + 2.0 * max_dev / KPI_HARD_LIMIT_KMH + 1.0 * (hard > 0)`**（連続項が勾配を与え、小さな定数項が「違反ゼロ」への段差インセンティブを保つ）。既存の p95/reversal 項は維持

### 3. Stage B-2: 本番モード代表区間での評価走行（pid_tuning / robot_controller / learning_cycle）

**責務**:
- 座標降下の評価走行を本番モードの代表区間で行い、適合結果の転移性を確保

**実装の要点**:
- `src/domain/pid_tuning.py` に `build_tuning_trajectory_from_mode(mode: DrivingMode, max_duration_s: float = 120.0) -> DrivingMode` を新設: 最高速度を含む窓＋加減速密度の高い窓を切り出して接続し、先頭に0→接続速度のランプを付与（安全包絡 max_speed/max_decel_g は元モードで保証済み）。フル走行(323s)×19runsは非現実的なため代表区間で降下し、最終確認のみフルモード
- `src/app/robot_controller.py::run_pid_tuning_session` / `_run_tuning_drive`: `mode: DrivingMode | None = None` 引数追加（Noneなら従来の `build_tuning_trajectory`）
- `src/infra/settings.py::LearningSettings`: `refine_runs_stage2: 5 → 12`、`tuning_on_target_mode: bool = True` を追加。`src/app/learning_cycle.py` REFINE_2 で対象モードを渡す
- `src/web/routers/drive.py::pid_tune_refine` / `schemas.py::PidRefineRequest`: 任意 `mode_id` を追加し `mode_repository.get_by_id` で解決

### 4. Stage B-3: ゲインスケジューリング（feedforward / pid / drive_loop）

**責務**:
- 速度依存プラントゲイン g(v)=∂opening/∂dv をFFモデルから抽出し、PID出力を正規化する

**実装の要点**（速度帯別ゲインテーブル案は追加適合走行が必要なため不採用。本方式は追加走行ゼロ・速度に連続・モデル再学習で自動更新）:
- **実装時の設計変更**: ユーザー選択により**ブレーキ側も対象**にしたため、`build_gain_table→list[tuple]` ではなく `GainSchedule`（accel/brake 両側の (speed→gain) 表＋線形補間メソッド）データクラスと `FeedforwardController.build_gain_schedule(v_max, step=5.0)` を新設。加速側は accel モデルに +probe（目標加速）、制動側は brake モデルに −probe（目標減速）を与えた前進差分で g を算出。摂動は全ホライズンの future を一律 ±probe（＝一定加減速要求）とした。クランプ `[0.02, 1.0]`＋3点移動平均で平滑化
- 保持は `FeedforwardController` 内（`_gain_schedule`、`rebuild_gain_schedule`/`gain_schedule`プロパティ、`unload_model` でクリア）。`robot_controller._apply_profile_to_control_stack` がモデルロード後に `rebuild_gain_schedule(profile.max_speed)` を1回呼ぶ
- `src/domain/control/pid.py::update` に `gain_scale: float = 1.0` 追加: `output = gain_scale * (kp·e + ki·I + kd·d)`、積分クランプは `output_limit/(ki·gain_scale)` に整合。積分器は誤差単位のまま（スケール緩変化に対し安全）。`scale<=0` は 1.0 にフォールバック
- `src/domain/control/drive_loop.py::_gain_scale`: サイクル毎に `scale = clamp(g(actual_speed)/g_nominal, 0.5, 3.0)`、`g_nominal = 1/fopdt_k`。**減速/加速フェーズの選択は基準速度トレンド（最短ホライズン先の基準−現基準）の符号**で行い、減速なら brake 側 g、それ以外は accel 側 g を使う（ref ベースなので計測ノイズでチャタリングしない）。schedule 未構築 or fopdt_k 未同定なら scale=1.0
- **Kd/微分フィルタは現行維持**（D-on-error + τf=0.2s。正規化によりkdも全速度域で一貫した意味を持つ。kdは座標降下の探索対象のまま）

**B-6 是正設計（ゲートB実機結果 2026-07-09 による）**: 初版実装は次の欠陥が実機＋机上検証で判明した。
(1) probe「全ホライズン一律 dv=±2km/h」は学習データに存在しない off-manifold 点（0.5秒で+2、その後加速停止という軌跡）で、共線な dv 特徴群の回帰勾配が信頼できない。生勾配2.3〜6.0 %/(km/h) がクランプ[0.02,1.0]（定常勾配想定）で全速度域上限飽和し、**スケジュールが「全域一律×1.49」に退化**。
(2) g_nominal=1/fopdt_k は定常ゲインの逆数 [%/(km/h)] で、probe が測る過渡勾配と次元不一致。
(3) speed_clip 境界（130-140km/h）で +probe がクリップされ勾配が人工的に減衰。
是正: probe を**定加速度マニフォールド**（dv_h = a·h、past = v0−a·h_past、g=∂opening/∂a を a=±0.5km/h/s差分で）に変更し、g_nominal = **fopdt_tau/fopdt_k** [%/(km/h/s)]（FOPDT初期加速応答 k/τ の逆数）、クランプ 0.05/5.0 に再校正、グリッド上限を clip 境界手前に制限。加えて tuning_cost の reversal 重み 0.2→1.0（チューナーが高速域2.3Hzチャタ＝振動KPI違反を受容していたため）。詳細タスクは tasklist B-6。

**B-2 実装時の追加事項（学習サイクルへの対象モード配線）**: 学習サイクルはこれまでモードを持たなかったため、`LearningCycleOrchestrator` に `mode_repo`/`tuning_on_target_mode` を任意注入し、`start(..., target_mode_id)` で対象モードを受けて REFINE_2 の `_resolve_tuning_mode()` が代表区間を構築する（未指定・解決不能時は従来の規定パターンへフォールバック）。`app.py` factory で `mode_repo` と `settings.learning.tuning_on_target_mode` を注入。standalone `/pid-tune/refine` は `mode_id` で即座に本番モード適合が可能。

### 4b. Stage B-7: ペダルワーク平滑化と適合走行の拡張（ゲートB第3次対応・Opus実装）

**背景（B-6' 実機結果 2026-07-09、cycle 5ab93a2e / auto session 343e80c1 の生ログ解析）**:
KPI は改善傾向（p95 0.492→0.448、max 1.589→1.372、違反時間 0.6%→0.2%）だが、ペダルワークが人間の運転に比べ過剰にハンチングしている。

1. **高速域アクセル ON-OFF ハンチング**: アクセルON回数 90回/322.9s（16.7回/min）のうち **110-135km/h 帯に 36.2回/min** が集中（0-110km/h帯は1〜5回/min で正常）。128-131km/h 定常巡航で開度 0.9〜2.3% が約0.6秒周期で振動（偏差±0.2〜0.4km/h、約1.7Hz のリミットサイクル）、OFF滞在 median 0.1s＝1〜2サイクルだけ0に落ちて戻るパルス。機構: 必要定常開度 ~2% に対し PID(kp=3.26, ki=2.76) が20Hzで偏差に即応 → effort がヒステリシス帯(±0.5%)を跨ぐ → PedalArbiter が coast 遷移でアクセルを**即0解放**する現仕様が ON-OFF パルスに変換
2. **モデル学習データのカバレッジ欠損**: cycle 5ab93a2e の learning+tuning 全ログ（=stage2学習データ）で **125-140km/h × アクセル0-6% のサンプルがほぼゼロ**（1-3%帯 0件。110-125帯は規定パターンの112保持が763件を供給）。WLTP 巡航131km/hで必要な開度1.5〜2.5%の領域が完全な外挿 → B-6 で観測した「120-140でゲインスケジュール崩落→scale床0.5」の根本原因。**チャタ帯(123-131)＝データ欠損帯＝規定パターン非カバー帯が一致**
3. **規定パターンの限界**: 現行は比率固定 `0.6/0.8/0.3 × max_speed` = 84/112/42、レート単一 `0.8×max_decel_g`≈11.2km/h/s（理論的根拠のない「安全包絡内で一通り振る」設計）。最高0.8比=112 が WLTP 巡航131に届かず、レート1種は WLTP の多様なランプ(0.5〜3.8km/h/s)と乖離
4. **学習運転の最高車速オーバー**: max_speed=140 に対し実測最大 **153.9〜156.1km/h**、>140 区間が14回。DRIVE_ACCEL の離脱が反応的（speed≥cap=0.98×140=137.2 で判定）で、70%踏込のガバナ上限加速 ≈13.8km/h/s × 解放〜車両応答遅れ ~1.2s = **+16km/h** のオーバーシュート（実測と一致。ピーク時アクセルは常に0%＝解放自体は効いており離脱判定の遅れが原因）

**B-7-1: 規定パターン拡張**（`src/domain/pid_tuning.py::build_tuning_trajectory`）
- 速度点を `0 → 0.6v(84) hold6s → 0.8v(112) hold6s → 0.95v(133) hold10s → 0.3v(42) hold6s → 0` に拡張。0.95比の巡航保持を新設し、WLTP巡航帯131を包含（125-140×微小開度のデータをstage2へ供給＋チューナーがチャタ帯を評価対象化）
- 加減速レートを多様化: 単一 `0.8×max_decel_g` → 区間ごとに高率(0.8)/低率(0.35)を混在（例: 0→84 は0.8率、112→133 は0.35率、133→42 は0.8率、42→0 は0.4率）。比率・保持長はモジュール定数
- 所要時間 42s→約62s、19runs で学習サイクル +6.5分程度（許容。必要なら保持長で調整）
- 全点 ≤max_speed・レート ≤0.8×max_decel_g の安全包絡は現行同様に保証
- 代替案 `tuning_on_target_mode=True`（B-2実装済み）は「モード非依存の適合」というユーザー既定選択を維持するため不採用

**B-7-2: 学習運転に高速微小開度パターン CRUISE_TRIM 追加**（`src/domain/learning_drive.py`, `src/domain/control/learning_loop.py`, `src/models/learning_drive.py`）
- 新 `PatternKind.CRUISE_TRIM`: 70%で cap 付近まで加速 → アクセルを小開度（例 1.5/2.5/4%）へ落として各数秒保持（惰性減速しつつ高速×微小開度の応答を採取）。2本程度
- 目的: stage1 モデル（学習運転のみで学習）の時点で高速帯 FF/ゲインスケジュールを成立させる（B-7-1 は stage2 にしか効かない）
- learning_loop にフェーズ遷移（DRIVE_ACCEL→段階TRIM保持）を追加。requirements.md スコープ外条項「学習運転パターンの変更」は B-7-2/5 の範囲で改訂

**B-7-3: ペダルワーク平滑化（人間的操作の機構的保証）**（`src/domain/control/pedal_arbiter.py`, `src/models/profile.py::FeedforwardParams`）
- (a) **coast 遷移時のアクセル解放レート制限**: 現行「即0解放」→ 直前がアクセルなら新パラメータ `accel_release_rate_pct_s`（既定 ~10%/s）で漸減。**effort < −h（ブレーキ要求）時は現行どおり即解放**（減速権限は遅延させない・安全維持）
- (b) **微小変化の量子化**: アクセル側で |requested − 前回開度| < min_step（~0.2%）なら前回値を保持（サーボ微振動も削減）
- 効果見込み: OFF滞在 median 0.1s のパルスが消え、110-135帯の ON回数 36/min → 数回/min。閉ループ1.7Hzリミットサイクルもループゲイン低下で減衰
- 既存の同時踏み禁止・ヒステリシス・再踏込ディレイは不変

**B-7-4: チューニングコストにペダル活動度項**（`src/domain/control/kpi_monitor.py`, `src/domain/control/drive_loop.py`, `src/domain/pid_tuning.py::tuning_cost`）
- KPIMonitor.update にペダル開度を渡し、`accel_on_count`（0→非0立ち上がり）と `pedal_rate_integral`（|Δopening|積分）を summary に追加
- `tuning_cost += w_pedal × accel_on_per_min`。重みは実機2セッション（ff431d46 / 343e80c1）のログで机上校正し、既存項（p95/超過積分/reversal）を支配しない値に設定
- 狙い: チューナーが「滑らかさ」を最適化対象として認識する（現在は kp=3.26 のような即応ゲインが選好される）

**B-7-5: 学習運転の予測的 cap 離脱（最高車速オーバー是正）**（`src/domain/control/learning_loop.py`）
- DRIVE_ACCEL 離脱条件を `speed >= cap − max(0, accel_kmhs_filtered) × overspeed_lead_s` に変更（`overspeed_lead_s` 新config、既定1.2s ≈ 解放時間＋θ＋τ相当）。accel_kmhs は `_update_governor` が受け取っている計測加速度を平滑化して使用
- 既存バックストップ（>max_speed→DRIVE_BRAKE、過速度スキップ判定）は維持
- 受け入れ: 学習運転の実測最大車速 ≤ max_speed。惰性でピークが cap 付近まで伸びるため計測カバレッジはほぼ不変

**B-7-6: PID/preview のアクセル・ブレーキ分離 — 今回は不採用（検討記録）**
- 根拠: (1) ハンチングはアクセル単独の定常巡航で発生しており分離では直らない (2) ゲインスケジュールが accel/brake 別テーブルで既にプラント非対称を正規化している (3) 座標降下の次元が4→7になり適合走行が倍増する (4) cycle 5ab93a2e は stage2 で pid_preview_s=0.075→0.0 を自ら選択しており preview 分離の必要性を示すシグナルがない (5) 停止移行の残違反は Stage C ILC の対象
- Stage C 完了後に停止移行違反が残る場合のみ、brake 側 preview の分離を再評価

**実装順序**: B-7-5（安全）→ B-7-1 → B-7-3 → B-7-4（実機ログ机上校正含む）→ B-7-2（learning_loop 改修を含むため最後）→ 品質チェック → B-7' 実機ゲート

### 4c. Stage B-8: ペダルシーソー対策 — 滑らかなペダル操作とKPIの両立（C-6ゲート初回で発覚）

> **【2026-07-11 改訂】本節の対策実装は新ステアリング `.steering/20260711-human-pedal-plan/`
> （ペダルプラン＋低速トリム構成への再編）に一本化した。B-8-1/2 はトリム凍結帯・フェーズ権限に
> 吸収、B-8-3/4 は移管（ゲートはモード相対に修正）。本節の**診断・実測値は正本として有効**。

**背景（2026-07-10 C-6初回実機: cycle abd4632a / tuning 2af4c09b(12:55) / auto e3773d9c(13:13) の生ログ解析）**:
学習サイクル再実行後の WLTP_ExHi 自動走行でペダル操作がギザギザ化し KPI も大幅悪化
（p95 0.448→**1.330** / max 1.372→**5.205** / 反転 20→18回/5s）。ユーザー指摘は「踏んだり
離したりのギザギザ運転。偏差±0.1km/h以内ならペダル操作しない等、無理に動かさない方が
トレースできるのでは」。

1. **アクセル⇔ブレーキ交互踏み（シーソー）**: auto 13:13 で 212回/走行（**39.4回/min**）、
   110-135km/h帯に集中（accelON 58.2/min + brakeON 45.0/min）。緩ランプ(+0.8km/h/s)の
   116-122km/h で約1秒周期のリミットサイクル: 偏差−0.4 → アクセル0→8-15% → 行き過ぎて
   偏差+0.3〜0.5 → アクセル0＋ブレーキ1.5-5%パルス、の繰り返し。交互踏み時の|dev|は
   p50=0.35km/h（212回中81回は|dev|<0.3で切替）＝人間なら惰行で済ませる場面。
2. **直接原因＝実効PIDゲイン過大**: kp=3.91 × ゲインスケジュール scale=3.0（上限クランプ
   張り付き）＝実効 kp **11.7 %/(km/h)**。FOPDT(k=2.16, τ=2.08, θ=0.30) の SIMC 適正値
   ≈1.6 の**約7倍**。モデル _20260710_035646.pkl での机上再現: accel側 g=3.35〜3.66
   @100-120km/h、g_nominal=τ/k=0.962 → scale が 90-125km/h 全域で上限3.0に飽和。
   B-6'まではこの帯のモデルデータ欠損で g が崩落し scale 床0.5 が偶然マイルドにしていたが、
   B-7-2 CRUISE_TRIM でデータが埋まり上限3.0のブーストが本当に効くようになった
   （欠損修正が過大ゲインを顕在化）。むだ時間θ起因の安定限界はプラントゲイン正規化では
   消えないため、ブースト側クランプ3.0は理論的にも過大だった。
3. **ブレーキパルスの増幅ループ**: 小さな行き過ぎで effort が −ヒステリシス(0.5%) を割り
   ブレーキ選択 → アクセル即0解放＋再踏込ディレイ0.3s → 偏差拡大 → 大きなアクセルジャンプ。
   B-7-3 の解放レート制限は「ブレーキ要求時は即解放」の安全仕様のためシーソーには効かない。
4. **付随観測**: max=5.2km/h は停止移行（t=317.9, ref=0 で act=5.2）＝Stage C 対象の悪化形。
   FOPDT 同定が同日朝(k=1.48/τ=1.02)と午後(k=2.16/τ=2.08)で大きく食い違う（同定分散、
   要ウォッチ）。ILC テーブルは今回の悪走行から iteration=1 を学習済み（best_p95=1.34）
   → ゲイン変更後は無効なのでゲート再開前にリセット必須。

**ユーザー承認済みの設計判断（2026-07-10）**: ブレーキ抑制ガードしきい値 **0.5km/h**、
PID誤差不感帯 **±0.1km/h**、ゲインスケジュールのブースト上限 **3.0→1.5**（減衰側0.5維持）。

**B-8-1: PID誤差不感帯（ユーザー提案の実装形）**（`src/domain/control/pid.py`, `src/models/profile.py`, `src/domain/control/drive_loop.py`）
- `PIDController.update` に `error_deadband: float = 0.0` キーワード引数を追加。連続形
  デッドゾーン `e_eff = sign(e)·max(0, |e|−db)` を誤差に適用してから P/I/D すべてを e_eff で
  計算（帯内では P=0・積分停止・微分0 → PID出力0。境界で不連続にしない）。db=0 で完全回帰
- `DynamicsParams` に `pid_error_deadband_kmh: float = 0.1` を追加（pid_preview_s と同居。
  `_dataclass_from_jsonb` の既定補完で DB マイグレーション不要＝B-7-3 と同じパターン）
- drive_loop が `error_deadband=profile.dynamics_params.pid_error_deadband_kmh` を配線
- スキーマ追従: `DynamicsParamsSchema`（ge=0）、`profiles.js` フォーム（ラベル「PID不感帯 [km/h]」）
- **ペダル開度そのものの凍結（原案の字義どおり）は不採用**: ランプ中に開度固定→偏差蓄積→
  段付き補正の鋸歯になる。PID寄与のみ停止し FF/ILC の滑らかな踏み増しは通す形を採用

**B-8-2: ブレーキ抑制ガード（惰行優先）**（`src/domain/control/drive_loop.py`, `src/domain/control/pedal_arbiter.py`）
- drive_loop: gain_scale 用に計算済みの ref_trend（`future_speeds[0] − ref_speed`）を再利用し
  `allow_brake = (ref_trend < _GAIN_DECEL_TREND_KMH) or ((actual − ref) >= BRAKE_ENGAGE_DEV_KMH)`
  （新定数 `BRAKE_ENGAGE_DEV_KMH = 0.5`）。基準が減速トレンドのとき・偏差+0.5以上のときは
  従来どおり即ブレーキ＝減速権限を遅延させない
- `PedalArbiter.arbitrate(effort, dt, allow_brake=True)`: `effort < −h` でも `allow_brake=False`
  なら pedal="coast"（アクセルは既存の解放レートで漸減、エンジンブレーキ1.6km/h/sで自然減速）
  とし **saturated_low=True** を返す（制動要求が削られた→PID負方向積分停止でワインドアップ
  防止）。既定 True で完全後方互換
- 安全性: 逸脱緊急停止・過電流等の安全網は不変。停止保持ブレーキは別経路のため影響なし

**B-8-3: ゲインスケジュールのブースト上限引き下げ**（`src/domain/control/drive_loop.py`）
- `_GAIN_SCALE_MAX = 3.0 → 1.5`（`_GAIN_SCALE_MIN=0.5` は維持）。高速域の実効 kp 11.7→5.9
  以下。docstring に「ブースト側はむだ時間起因の安定限界が正規化では消えないため控えめに
  する（B-8 実測: scale3.0×kp3.91＝SIMC適正の7倍で1Hzリミットサイクル）」を記録

**B-8-4: チューニングコストにシーソーペナルティ（再発防止）**（`src/domain/control/kpi_monitor.py`, `src/domain/control/drive_loop.py`, `src/domain/pid_tuning.py`, `scripts/analyze_session.py`）
- `KPIMonitor.update` に任意引数 `brake_opening: float | None = None` を追加。アクセルON⇔
  ブレーキON の交互踏み（前回アクティブペダルと異なるペダルが**2秒以内**にON）を
  `pedal_switch_count` としてカウントし、summary に `pedal_switch_count` /
  `pedal_switch_per_min` を追加（accel_on_count と同パターン・None で後方互換）
- drive_loop が `brake_opening=self._current_brake_opening` を配線
- `tuning_cost += PEDAL_SEESAW_WEIGHT × pedal_switch_per_min`。重みは実機2セッション
  （e3773d9c=39.4/min・2af4c09b=18.8/min）で机上校正（初期候補 0.5。B-7-4 と同じ手順:
  既存支配項を上書きせず、シーソーがあるときは無視できない値）
- `analyze_session.py` に交互踏み回数/min を追加し、ゲート目安 **交互踏み ≤3回/min** の
  合否表示を追加

**B-8-5: 不採用の検討記録**
- accel_rate_limit_pct_s(200) の引き下げ・ヒステリシス拡大: ゲイン是正＋ガードで踏み込み
  ジャンプの原因側を消す。レート制限強化は位相遅れを足すだけなので保留
- FOPDT 同定分散の是正: CRUISE_TRIM 保持区間は rise<0 で棄却される構造は確認済み。
  B-7-5 の予測的 cap 離脱でプラトー到達前に区間が切れる影響が疑われるが今回はスコープ外
  （ゲート結果で再燃したら別途）

**実装順序**: B-8-1 → B-8-2 → B-8-3 → B-8-4 → 品質チェック → ILCリセット＋学習サイクル
再実行（新構造でのゲイン再適合。scale上限・不感帯・ガードは適合走行にも効くため必須）→
Stage B ゲート水準の回復確認 → C-6 再開

### 5. Stage C: ILC（新規 src/domain/control/ilc.py ほか）

**責務**:
- 同一 profile×mode の反復走行で残差を学習し、時刻別補正effortとして次回走行に適用

**実装の要点**:
- `src/domain/control/ilc.py`（純ドメイン、I/Oなし）:
  - `ILCTable`: `dt_s=0.1`（drive_logsと同周期グリッド）、`efforts: list[float]`（符号付き%、FF/PIDと同単位）、`iteration: int`、`best_p95_kmh: float | None`
  - `ILCController.effort_at(elapsed_s)`: 線形補間＋±amp_limit（既定10%）クランプ。now-frameで参照（Δシフトは学習時に焼き込み済み）
  - `ILCLearner.update(table, times, errors, *, l_gain, delta_s, amp_limit_pct, cutoff_hz)`: `u_{j+1}(t) = clip(Q(u_j(t) + L·e_j(t+Δ)), ±amp_limit)`。Qはゼロ位相ローパス（numpy自作のforward-backward 1次フィルタで十分）。初期値: **L = 0.4 / fopdt_k、Δ = fopdt_theta、cutoff ≈ 0.3Hz**（PID帯域より低くして役割分離）
- 安全策: (1) 振幅上限10%で暴走を機構的に制限、(2) 発散検知＝直近走行p95が `best_p95×1.2` 超なら学習スキップ＋WARNING（テーブルはbest時点を保持）、(3) リセットAPI、(4) `drive_sessions.status='completed'` のセッションのみ学習（emergency/手動停止/pause使用は除外）、(5) モード編集時に `ilc_repo.reset`
- 永続化 `scripts/setup_db.py` に新テーブル:
  ```sql
  CREATE TABLE IF NOT EXISTS ilc_tables (
      profile_id UUID NOT NULL REFERENCES vehicle_profiles(id) ON DELETE CASCADE,
      mode_id    UUID NOT NULL REFERENCES driving_modes(id) ON DELETE CASCADE,
      enabled    BOOLEAN NOT NULL DEFAULT TRUE,
      iteration  INTEGER NOT NULL,
      dt_s       DOUBLE PRECISION NOT NULL,
      efforts    JSONB NOT NULL,
      kpi_history JSONB NOT NULL DEFAULT '[]'::jsonb,
      updated_at TIMESTAMPTZ NOT NULL,
      PRIMARY KEY (profile_id, mode_id)
  )
  ```
- `src/infra/ilc_repository.py`（新規、profile_repositoryパターン踏襲）: `get / upsert / reset / set_enabled`
- `src/app/ilc_service.py`（新規）: `prepare(profile, mode) -> ILCController | None`（走行開始時ロード）、`learn_from_session(session_id, profile, mode, kpi_summary)`（完了後にdrive_logsから残差学習→upsert。発散検知は `controller.last_kpi_summary` を使用）
- `src/app/robot_controller.py`: `start_auto_drive` で `ilc_service.prepare` を配線、正常完了経路で `learn_from_session` を fire-and-forget タスク起動（走行停止をブロックしない）。`factory.py` で注入
- `src/domain/control/drive_loop.py`: コンストラクタに `ilc: ILCController | None = None`、合成を `arbitrate(ff_effort + pid_u + ilc_effort, dt)` に。ilc_effortはDEBUGログに留める（DriveLogDataスキーマ変更回避）
- WebUI: `src/web/routers/drive.py`（または新規ilcルータ）に `GET /api/v1/drive/ilc/{profile_id}/{mode_id}`（iteration/enabled/kpi_history）、`POST .../enable|disable|reset`。`src/web/static/js/screens/auto-drive.js` にモード選択時のILC状態表示（「反復学習: 第N回・前回p95」＋トグル＋リセット＋収束表示）

## データフロー

### 自動走行1サイクル（20Hz、Stage C適用後）
```
1. elapsed_s から now-frame 基準速度 ref_speed を取得（KPI/ログ/WS用）
2. FF: ref_speed と horizons/past の now-frame 先読み値から ff_effort を予測
3. PID: ref_speed_pid = ref(elapsed_s + pid_preview_s) と実車速から pid_u を計算（gain_scale 正規化）
4. ILC: ilc_effort = table.effort_at(elapsed_s)
5. arbitrate(ff_effort + pid_u + ilc_effort, dt) → ペダル開度 → サーボ
6. KPIMonitor.update(now-frame ref, actual)（超過積分含む）
```

### ILC学習（走行正常完了後）
```
1. robot_controller が completed を確認し ilc_service.learn_from_session を非同期起動
2. drive_logs から (timestamp, ref, actual) を取得し e_j(t) = ref - actual を10Hzグリッド化
3. 発散検知（p95 > best×1.2 → スキップ）
4. u_{j+1}(t) = clip(Q(u_j(t) + L·e_j(t+Δ)), ±10%) を計算し ilc_tables に upsert、kpi_history 追記
```

## エラーハンドリング戦略

- ILCテーブルロード失敗・モデル未ロード時は **ilc_effort=0 / scale=1.0 にフォールバック**し走行は継続（ILC/スケジューリングは性能向上手段であり安全の前提にしない）
- 逸脱緊急停止（stop_config.deviation_threshold_kmh）は現行のまま安全網として維持
- ILC学習タスクの例外は走行停止処理に伝播させない（fire-and-forget＋ログ）

## テスト戦略

### ユニットテスト
- Stage A: `test_drive_loop.py` のpreview系を新仕様に置換（FFはnow-frame / PIDのみシフト / pid_preview_s=0で完全回帰）。`test_pid_tuning.py` の初期preview=0.0・クランプ1.0。旧キー入りJSONBから `pid_preview_s=0.0` で復元されるロードテスト（tests/unit/infra）
- Stage B: `test_kpi_monitor.py`（超過積分の解析値一致・違反ゼロ→0）、`TestTuningCost`（超過積分の単調性・hard段差が非支配）、`test_pid.py`（gain_scale比例性・積分クランプ整合・scale=1回帰）、`test_feedforward.py`（線形ダミーモデルで解析勾配一致・クランプ）、`build_tuning_trajectory_from_mode` の安全包絡（max_speed以内・G上限・0始端0終端）
- Stage C: `test_ilc.py`（新規: FOPDTシミュレータで5反復収束、L過大発散→検知、振幅クランプ、ゼロ位相性、Δシフト端点処理）、`test_ilc_repository.py`、drive_loopのilc合成/None回帰、正常完了時のみ学習

### 統合テスト・実機検証
- 各Stage完了時に実機で学習サイクル再実行＋WLTP_ExHi自動走行し、requirements.mdの受け入れ条件（ゲートA/B/C）で合否判定
- 実走ログ解析（|偏差|p50/p95/max、違反セグメント、ラグ走査、5秒窓符号反転）は解析スクリプトとして `scripts/analyze_session.py` に整備し各ゲートで再実行

## 依存ライブラリ

新規追加なし（numpy/scikit-learnの既存範囲。ゼロ位相フィルタはnumpy自作）

## ディレクトリ構造

```
src/domain/control/ilc.py          (新規: ILCTable/ILCController/ILCLearner)
src/infra/ilc_repository.py        (新規)
src/app/ilc_service.py             (新規)
scripts/analyze_session.py         (新規: 実走ログKPI解析)
tests/unit/test_ilc.py             (新規)
tests/unit/infra/test_ilc_repository.py (新規)
既存変更: src/models/profile.py, src/domain/control/{drive_loop,pid,kpi_monitor,feedforward}.py,
          src/domain/pid_tuning.py, src/app/{robot_controller,learning_cycle,training_service,factory}.py,
          src/infra/settings.py, scripts/setup_db.py,
          src/web/{schemas.py,routers/drive.py,static/js/screens/{profiles,learning,auto-drive}.js}
```

## 実装の順序

1. **Stage A**（A完了→実機ゲートA判定→B着手）
2. **Stage B**（B-1→B-2→B-3、実機ゲートB判定→C着手）
3. **Stage C**（ドメイン→infra→app配線→WebUI、実機ゲートC＝最終合格判定）

依存関係: AはBのpreview探索の前提（二重補償があると誤った最適点に行く）。Bの安定化はCの前提（PIDが発振するとe_j(t)の再現性が崩れILCが収束しない）。

## セキュリティ考慮事項

- 新APIエンドポイント（ILC enable/disable/reset）は既存の drive ルータと同じ認可・バリデーション方針に従う

## パフォーマンス考慮事項

- 制御ループ20Hz内の追加コストは ILC線形補間＋gain_scale参照のみ（O(log n)＋O(1)、テーブルはプロファイル選択時に構築）
- ILC学習は走行完了後の非同期タスク（3230点程度、ミリ秒オーダー）

## 将来の拡張性

- ILCテーブルはprofile×mode複合キーのため、モード追加時は自動的に独立学習
- gain_scale機構はモデル再学習のたびに自動追従（テーブル再構築のみ）
- 将来的にDriveLogDataへ ilc_effort カラムを追加すれば寄与の可視化が可能（今回はスキーマ変更回避のためDEBUGログ）
