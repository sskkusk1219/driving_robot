# 引き継ぎ書（学習運転 開度パターン方式）

最終更新: 2026-06-18 / 状態: 実装完了・**未コミット**（作業ツリーのみ）

このタスクは長く、実機フィードバックで複数回イテレーションした。新しいセッションが続きを行うための
要点をここにまとめる。詳細な経緯は同フォルダの `requirements.md` / `design.md` / `tasklist.md`
（フェーズ1〜12 + 振り返り）を参照。

---

## 1. 何をしたか（全体像）

学習運転を、`docs/ideas/initial-requirements.md` Phase 8 の **「固定の開度パターンを開ループで
順に走らせ、実車速・開度を連続記録する」方式**へ本番配線した。収集した連続ログから
先読み Ridge 逆モデルを学習する既存の `/drive/learning/train` はそのまま使う。

旧方式（`build_learning_reference` による閉ループ基準速度追従・速度×加速度グリッド）は削除済み。

### 学習運転の現在のフロー
1. 開始ボタン → `POST /drive/learning/arm`
   - **踏込前チェック**（走行前チェックから「車速確認」を除外して実行＝両ペダル原点・通信・サーボ等）
   - 停車保持ブレーキ（`stop_brake_opening_pct`）まで踏む
   - 車速が 0 に収束するまで待機（タイムアウト 10s）
   - **踏込後チェック**（走行前チェックから「アクチュエータ位置」を除外＝保持ブレーキで原点を離れるため。車速0/通信/サーボ等を判定）
   - 合格 → PRE_CHECK のまま待機（フロントが確認ポップアップ表示）
2. ポップアップ「はい」→ `POST /drive/learning/start`（RUNNING、ログ開始、`LearningLoop` 起動）
   「いいえ」→ `POST /drive/learning/cancel`（ブレーキ原点復帰・READY）
3. `LearningLoop` がパターン列を開ループ実行（下記）。完了 → RUNNING→READY
4. フロントが RUNNING→READY を検知し `autoTrain`（`/drive/learning/train`）を自動発火
5. `train_inverse_model`＋`estimate_dynamics_params` が逆モデルとクリープ等の物理定数を
   プロファイルへ反映

### パターン構成（`LearningDriveManager.generate_patterns`）※フェーズ14で更新
1. **クリープ解放**（CREEP）: 停車保持ブレーキ → 0手前まで段階的に緩める（`creep_release_steps`=5 分割）
2. **クリープ安定待ち**（CREEP_SETTLE）: accel=brake=0 で車速が安定するまで保持。クリープ車速/加速率を計測
3. **コーストダウン**（COAST_DOWN）数本: 高開度（`coast_down_accel_pct`=50）で加速→ブレーキ無しで低速まで惰行。
   速度全域のエンジンブレーキ＋走行抵抗の減速率を計測
4. **連続走行スイープ**（COMBINED）: 1サイクルで「アクセルで加速→惰行→ブレーキで減速・停車」を連続実行。
   accel/brake スイープを **zip でペア化**（短い方は巡回）。20分予算のため `max_combined_patterns`=40 で本数上限、
   `brake_sweep_start_pct`=10（低ブレーキは惰行/ランプで採れる）、高開度域(>50%)は `high_opening_step_pct`=5 の粗刻み。

実行（`LearningLoop`）のサブフェーズ（フェーズ14で再構成）:
- COMBINED: `DRIVE_ACCEL`（加速し**車速プラトーまで保持** or timeout）→ `COAST`（accel=brake=0 で惰行・エンジン
  ブレーキ計測。所定速度 or 減速プラトー or timeout）→ `DRIVE_BRAKE`（停車）。COAST が加速と制動を物理分離。
- COAST_DOWN: `DRIVE_ACCEL` → `COAST`（低速 `coast_down_stop_speed_kmh` まで惰行）→ 次パターン（ブレーキ無し）。
- CREEP/CREEP_SETTLE は `MEASURE`。
- **時間指定移動**: フェーズ入場時に1回だけ `move_to_position_timed(target,current,duration)` を発行（servo が
  距離÷時間で速度算出し滑らかにランプ）。以降のサイクルは電流のみ読む。`accel_to_brake_speed_frac` は廃止。

### 安全（2段階）＋同時踏み禁止
- **スキップ型**（非常停止しない、デバウンス2サイクル）: 過速度（>`max_speed`）。DRIVE_ACCEL では**惰行へ移行**、
  DRIVE_BRAKE 過Gでは**ブレーキ踏み増しを頭打ち**にして停車まで継続（`_brake_ramp_cap`＋是正の再 timed move）。
  ※加速方向の過G（踏み始めの強い加速）は離脱要因にしない（高開度でも定常まで保持するため）。
- **非常停止型**: 過電流・CAN断・アクチュエータ失敗・サイクル例外 → `on_emergency`
- **同時踏み禁止**: `src/domain/control/pedal_safety.py` の `enforce_pedal_exclusion` を学習・自動の両ループで
  最終段に通す（自動は PedalArbiter が構造的に排他済み。学習は各フェーズが片ペダルのみ＋COAST で物理分離）。

---

## 2. 現在の状態

- **全テスト 601 passed**（`tests/hardware` 除く。`tests/hardware/test_can_receive.py` は実CAN要で環境依存・本件無関係）
- **変更 Python ファイルは ruff・mypy クリーン**。`mypy src` で残る2件（`src/domain/calibration.py:123`、`src/web/app.py:71`）は**本件と無関係の既存エラー**（触っていない）
- **未コミット**。コミット指示は出ていない。コミット前に `/code-review` 済み（指摘#1〜#4は修正済み、後述）

---

## 3. 主要ファイルと責務

| ファイル | 役割 | 新規/改修 |
|---|---|---|
| `src/models/learning_drive.py` | `LearningPattern` / `PatternKind`(CREEP/CREEP_SETTLE/ACCEL/BRAKE) | 改修 |
| `src/domain/learning_drive.py` | `LearningDriveManager.generate_patterns`（開度スイープ生成）/ `LearningDriveConfig` / `LearningDataError` | 改修 |
| `src/domain/control/learning_loop.py` | **`LearningLoop`**（開ループ実行・100ms連続ログ・状態機械・安全） / `LearningLoopConfig` | **新規** |
| `src/domain/control/drive_loop.py` | 自動/手動の閉ループ。`stop_and_join` に自タスクjoinガードを追加（後述#1） | 改修（1点） |
| `src/domain/pre_check.py` | `run(exclude=...)`＋`ITEM_*`定数、`_check_vehicle_stopped`（車速0、7項目目） | 改修 |
| `src/app/robot_controller.py` | `arm/start/cancel/stop_learning_drive`、`_wait_for_vehicle_stopped`、`_apply_brake_hold`、`_realtime_loop`、stop/emergency/shutdown で学習ループも停止 | 改修 |
| `src/web/routers/drive.py` | `/learning/arm` `/learning/cancel`（新規）、`/learning/start`（armed前提へ） | 改修 |
| `src/web/routers/profiles.py` | `PUT /profiles/{id}` で `refresh_active_profile` を呼ぶ（編集の即時反映） | 改修 |
| `src/web/static/js/screens/auto-drive.js` | `DriveMonitorScreen` に arm フロー用props（driveArmPath/driveCancelPath/confirmStartMessage）＋確認ポップアップ＋アンマウントcancel | 改修 |
| `src/web/static/js/screens/learning.js` | 上記 props を渡す | 改修 |
| テスト | `test_learning_loop.py`(新規)、`test_learning_drive.py`/`test_pre_check.py`/`test_robot_controller.py`/`test_web_drive.py`/`test_web_api.py`(改修) | |

`LearningLoop` は `DriveLoop` を範に call_later スケジューリング・サイクルスキップ ウォッチドッグ・
ログ滞留制御・`stop_and_join` を踏襲している。FF/PID は使わない（開ループ）。

---

## 4. 実機フィードバックの反映履歴（重要・時系列）

実機で回しながら直した。経緯を理解しておくと次の判断がしやすい。

1. **arm 順序**: 当初「走行前チェック（車速0含む）→ブレーキ踏込」で、走行中(5km/h)だと即422。
   → 「**ブレーキ踏込→車速0収束待ち→走行前チェック**」へ修正。
2. **位置チェック競合**: 踏込後の走行前チェックで「アクチュエータ位置」が保持ブレーキ(例1575pulse)を
   原点逸脱と判定し422。→ **走行前チェックを2段階化**（踏込前=車速除外、踏込後=位置除外）。
3. **プロファイル編集が反映されない**: `PUT /profiles` がコントローラ `_active_profile` を更新せず、
   `stop_brake_opening_pct` 等を変えても踏込量が変わらなかった。→ `refresh_active_profile` を呼ぶよう修正。
   （%↔pulse 換算 `zero_pos + (full_pos-zero_pos)*pct/100` は**正しい**。1575=ストローク20%）。
4. **1分足らずで終了（6:40ログ, 76s）**: `max_decel_g=0.3G(≈10.6km/h/s)` の過Gスキップが、固定開度を
   踏んだ瞬間（低速＝高加速）にほぼ全パターンを0.2〜0.5秒で打ち切っていた。データの42%が
   RETURN/SPINUP。creep_speed 未更新(7.0)・creep_rate のみ更新。
   - **ユーザー判断: 過Gスキップは維持**（安全優先）。
   - → **クリープ安定待ち CREEP_SETTLE 追加**（creep_speed を確実に計測）＋**開度刻み 10%→5%**。
5. **まだ2分（7:11ログ, 133s）+ 急停止が危険**: RETURN(0%/30%)が39.6s(約30%)で、減速時にいきなり30%へ
   踏んで急停止していた。→ **開度刻み 5%→2%**＋**RETURN のブレーキを 0→目標へレートリミット
   (15%/s) で緩やかに**（踏込過程の連続ログもブレーキ学習データになる）。
6. **アクセル/ブレーキを別々に振るのをやめたい＋緩やかに踏みたい（フェーズ13, 6-18）**:
   ユーザー要望で走行構成を再設計。→ **アクセル/ブレーキを統合し1サイクルで加速→減速**する `COMBINED`
   パターンへ（accel/brake スイープを zip ペア化）。SPINUP・固定30%RETURN を廃止し、直前のアクセル加速が
   助走、ブレーキ印加が停車を兼ねる（20分budget も短縮）。さらに **MEASURE の一気踏みをやめ、accel/brake とも
   `ramp_time_s` 秒かけて 0→目標へ線形に踏み込む**（人の踏み込みを模す。既定1.5s、実機で適合予定）。
   安全は維持しつつ、加速区間の過G/過速度は減速へ移行、減速区間の過Gはブレーキ頭打ちに変更。
7. **実機ログ(437s)で4問題（フェーズ14, 6-19）**: ①アクチュエータ動作が急（loop ジッタでランプが粗化）
   ②アクセル保持が短い（0.5×max_speed で即ブレーキ＝定常前に減速）③エンジンブレーキ未計測 ④ペダル切替が
   物理的に乱暴。→ **時間指定移動**（servo が duration かけて到達）でジッタ非依存に滑らか化、**プラトー保持**
   （定常までアクセル保持）、**COAST 区間＋専用コーストダウン**でエンジンブレーキ減速率を計測、**同時踏み禁止
   ガード**（`enforce_pedal_exclusion`、COAST で物理分離）。`accel_to_brake_speed_frac` 廃止、20分予算は
   `max_combined_patterns`/各フェーズ timeout で管理。
8. **制約違反（フェーズ15, 7:09ログ558s）**: 上限開度80%は守れていたが、**最高車速100→実160km/h(31%超過)**・
   **上限0.3G→加速1.2G/減速1.5G**。原因: プラトー保持に車速上限が無く終端速度100超の開度で突き抜け／overspeed
   時 COAST で惰行のみ・ブレーキせず居座り／ブレーキ過Gが「頭打ち」だけで減速Gを下げない。重要: 自動運転も
   0.3G/100km/h 包絡内でしか動かない＝包絡超の高開度は学習しても無駄。→ **包絡線ガバナ**（平滑Gが上限超で開度を
   止め/下げる）＋**車速キャップ**（max_speed手前で加速終了）＋**overspeed能動制動復帰**＋**ブレーキG モジュレート**
   （freeze→reduce）。ユーザー判断: 包絡内のみ計測でOK。`max_combined_patterns` 20 に削減（高開度は同挙動収束）。

### code-review（high）の指摘と対応（全て対応済み）
- **#1 完了経路の自タスク join バグ**（重大）: `on_complete=stop_*_drive` がサイクルタスク内から
  `stop_and_join` を呼び、自タスクを2秒待って cancel → 後続 `home_return` が中断され原点未復帰。
  → `stop_and_join` に `task is asyncio.current_task()` ガード（LearningLoop と DriveLoop 両方）。
- #2 クリープ反映が弱い → CREEP_SETTLE / CREEP は RETURN を挟まず連続リリースにして pedal-off 連続サンプルを確保。
- #3 確認ポップアップ未確定で離脱→保持ブレーキ取り残し → フロントのアンマウント cleanup で cancel。
- #4 過G/過速度の単発ノイズ誤スキップ → 連続2サイクル デバウンス。

---

## 5. 現在のチューニングパラメータ

`src/domain/learning_drive.py`:
- `ACCEL_OPENING_STEP_PCT=2.0`, `BRAKE_OPENING_STEP_PCT=2.0`, `CREEP_RELEASE_STEPS=5`, `HOLD_DURATION_S=3.0`

`src/domain/control/learning_loop.py` の `LearningLoopConfig`（フェーズ14で刷新）:
- **`accel_ramp_time_s=1.5`**, **`brake_ramp_time_s=1.5`**, **`pedal_release_time_s=0.4`**（時間指定移動の到達秒数。主役）
- プラトー保持: `accel_hold_min_s=1.0`, `accel_plateau_tol_kmhs=0.5`, `accel_plateau_duration_s=1.5`, `accel_hold_timeout_s=6.0`
- 惰行: `coast_to_brake_speed_frac=0.6`, `coast_down_stop_speed_kmh=5.0`, `coast_settle_tol_kmhs=0.3`, `coast_settle_duration_s=1.0`, `coast_timeout_s=6.0`
- `brake_stop_timeout_s=10.0`, `skip_consecutive_required=2`, `creep_settle_*`（フェーズ11のまま）
- ※ `accel_to_brake_speed_frac`（0.5×max_speed 切替）は**削除**。加速はプラトーまで保持する方式に変更

`src/domain/learning_drive.py` の `LearningDriveConfig`（フェーズ14で追加）:
- `brake_sweep_start_pct=10.0`, `high_opening_threshold_pct=50.0`, `high_opening_step_pct=5.0`,
  `max_combined_patterns=40`, `coast_down_count=3`, `coast_down_accel_pct=50.0`（いずれも20分予算の調整つまみ）

`src/infra/actuator_driver.py`:
- `move_to_position_timed(target, current, duration_s)`（速度=距離mm÷duration、`_MIN/_MAX_TIMED_SPEED_MM_S`=1/100 でクランプ）。
  予算超過時にまず下げる: `accel_hold_timeout_s`/`coast_timeout_s`/`brake_stop_timeout_s`/`max_combined_patterns`。

`LearningLoopConfig`（フェーズ15で追加した包絡線ガバナ）:
- `g_smoothing_window_s=0.4`（平滑加速度の窓。CANノイズ対策）, `g_limit_frac=0.9`（上限G に対する作動しきい）,
  `gov_reduce_step_pct=2.0`（超過継続時に開度を下げる量/サイクル）, `accel_speed_cap_frac=0.9`（加速を max_speed の
  この比で打ち切り）, `pedal_step_time_s=0.2`（指令変化時の時間指定移動の所要時間）
- ガバナの上限G は `profile.max_decel_g × g_limit_frac`（加速・減速同一）。位置指令は「指令開度が変化した軸のみ」発行。
- `LearningDriveConfig.max_combined_patterns` 40→20（ガバナで高開度が冗長化するため低中開度を密に）。

※ これらは `factory.py` / `stubs.py` で `LearningDriveManager()` / `LearningLoop(...)` を**デフォルト構成**で
注入している（config 引数は省略）。チューニングは上記デフォルト値を変えるか、生成箇所で config を渡す。

---

## 6. 既知の課題・未確認事項（次の人へ）

1. **20分budget（要実機確認）**: 2%刻みでブレーキスイープが約40本になり、各本の前の SPINUP
   （`max_speed×0.5` まで加速）が累積コストの主因。RETURN を緩やかにした分も増える。
   実機で 20 分を超えるなら圧縮策:
   - `spinup_target_speed_frac` を下げる（0.5→0.3 等）
   - ブレーキ低開度域(0〜30%)は RETURN の緩やか踏込で既にデータが採れるので、ブレーキスイープを
     中〜高開度に絞る（generate_patterns の brake sweep 開始開度を上げる）
2. **クリープ反映の最終確認**: CREEP_SETTLE 追加で `creep_speed` が更新されるはず。次回学習後に
   DB の `vehicle_profiles.feedforward_params->>'creep_speed_kmh'` が 7.0(既定) から変わるか確認。
3. **過Gスキップの是非**: ユーザーは「維持」を選択。結果として高開度のアクセル/ブレーキは
   データが薄いまま（安全側）。学習精度に効いてくるようなら再相談。
4. **MAE は in-sample**（学習データ自身への誤差＝楽観的）。汎化性能ではない。表示改善は未対応（スコープ外）。
5. **未コミット**。コミット/PR は未指示。

---

## 7. 検証手順

### テスト・静的解析
```
.venv/bin/python -m pytest -q --ignore=tests/hardware    # 601 passed
.venv/bin/ruff check <変更したPyファイル>                  # All checks passed
.venv/bin/mypy src                                        # 既存2件のみ（本件無関係）
```
注意: ruff/mypy に `.js` を渡さない（Python専用）。

### 実機起動（ユーザー実行。`memo.txt` 参照）
```
DRIVING_ROBOT_USE_REAL_HW=1 DATABASE_URL=postgresql://localhost/driving_robot \
  .venv/bin/uvicorn src.web.app:app --host 0.0.0.0 --port 8080
```

### 実機ログの確認（DB クエリ。所要時間・開度別保持時間の分析に有用）
```
psql postgresql://localhost/driving_robot -P pager=off -c "
SELECT id, started_at, EXTRACT(EPOCH FROM (ended_at-started_at)) AS dur_s, status
FROM drive_sessions WHERE run_type='learning' ORDER BY started_at DESC LIMIT 5;"

psql postgresql://localhost/driving_robot -P pager=off -c "
SELECT round(accel_opening::numeric,0) a, round(brake_opening::numeric,0) b,
       count(*) n, round((count(*)*0.1)::numeric,1) held_s
FROM drive_logs WHERE session_id='<SESSION_ID>' GROUP BY 1,2 ORDER BY n DESC LIMIT 20;"

# プロファイルのクリープ定数が更新されたか
psql postgresql://localhost/driving_robot -P pager=off -c "
SELECT name, feedforward_params->>'creep_speed_kmh' creep_speed,
       feedforward_params->>'creep_rate_kmhs' creep_rate FROM vehicle_profiles;"
```

---

## 8. 重要な設計判断・落とし穴

- **逆モデルの特徴量は全て「実車速」由来**。開度の決め方（開ループ指令）に依存せず、
  (実速度,実加速度,開度) が物理整合していれば有効な学習データ。連続軌跡で記録すれば
  既存のセッション単位 lookahead 特徴量・`estimate_dynamics_params` が無改造で成立する。
- **hold 状態は専用stateを足さず PRE_CHECK を流用**（状態機械・フロントへの波及最小化）。
  start: PRE_CHECK→RUNNING、cancel: PRE_CHECK→READY。既存の「RUNNING→READYでautoTrain」が無改造で成立。
- **`stop_and_join` を on_complete/on_emergency 経路（サイクルタスク内）から呼ぶと自タスクjoinになる**。
  ガード済みだが、LearningLoop/DriveLoop を触るときは要注意。
- **CREEP / CREEP_SETTLE は RETURN を挟まない**（連続リリースで pedal-off 連続サンプルを確保するため）。
- **走行前チェックは7項目**（6→7に車速確認を追加）。`run(exclude=...)` で項目を間引ける。
- **factory/stubs はデフォルト config で注入**。テストは config を明示して短縮している
  （例: `skip_consecutive_required=1`, creep_settle の各値）。
