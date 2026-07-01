# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

### タスクスキップが許可される唯一のケース
- 実装方針の変更により、機能自体が不要になった
- アーキテクチャ変更により、別の実装方法に置き換わった
- 依存関係の変更により、タスクが実行不可能になった

スキップ時は必ず理由を明記:
```markdown
- [x] ~~タスク名~~（実装方針変更により不要: 具体的な技術的理由）
```

---

## フェーズ1: パターン生成の作り替え

- [x] `src/models/learning_drive.py` の `LearningPattern` を開度パターン用に整理
  - [x] (kind, accel_opening, brake_opening, hold_duration_s) の構造へ（`PatternKind` 追加）
  - [x] 未使用化する `LearningLog` を削除
- [x] `src/domain/learning_drive.py` の `generate_patterns` を開度スイープへ再定義
  - [x] アクセルスイープ（brake=0、段階的に〜`max_accel_opening`）
  - [x] ブレーキスイープ（accel=0、段階的に〜`max_brake_opening`）
  - [x] クリープ緩め列（brake を `stop_brake_opening_pct` から段階的に下げる）
  - [x] 最大開度超過のクランプ
- [x] 不要関数を削除: `run_pattern` / `_compute_initial_opening` / `_opening_to_pulse` / `build_learning_reference`（`LearningDataError` は model_training/drive が参照のため保持）
- [x] `tests/unit/test_learning_drive.py` を新仕様へ更新（旧テスト削除・新テスト追加、13件 pass）

## フェーズ2: 走行前チェックへ車速0確認を追加

- [x] `src/domain/pre_check.py` に `_check_vehicle_stopped()` を追加（`can.read_speed()` < しきい値）
  - [x] 停車判定しきい値を定数化（`STOPPED_SPEED_THRESHOLD_KMH = 0.5`）
  - [x] `run()` のチェック列へ組み込み（6→7項目、docstring も更新）
- [x] `tests/unit/test_pre_check.py` に車速 ≠ 0 で NG になるテストを追加（30件 pass）

## フェーズ3: 開ループ実行ループの新設

- [x] `src/domain/control/learning_loop.py` を新規作成（`DriveLoop` を範に）
  - [x] 周期スケジューリング・サイクルスキップガード・例外回収・`stop_and_join`
  - [x] 開度→位置変換（`zero_pos`/`full_pos` 線形補間）
  - [x] フェーズ0 クリープ計測（ブレーキ保持から段階的に緩める＝CREEP パターン）
  - [x] フェーズ1 アクセルスイープ / フェーズ2 ブレーキスイープ（SPINUP→MEASURE→RETURN 状態機械）
  - [x] 各サイクルで固定開度指令・電流取得・実車速取得・連続 `write_log`（ref_speed=None）
  - [x] パターン前進条件（最大ホールド時間／停車）とパターン間原点復帰（RETURN フェーズ）
  - [x] スキップ型安全: 過速度(>profile.max_speed)・過G(abs(実測加速度)>profile.max_decel_g、加速/減速両方向) → 当該パターン打ち切り＋次へ（非常停止しない）
  - [x] 非常停止型安全: 過電流・CAN断・サイクル例外 → `on_emergency`
  - [x] `on_complete` を完了時に一度だけ
  - [x] 公開プロパティ: `current_accel_opening`/`current_brake_opening`/`current_ref_speed`(None)/`last_snapshot`
  - [x] 20分以内に収まるパターン数×ホールド上限の config 化（`LearningLoopConfig`）
- [x] `tests/unit/test_learning_loop.py` を新規作成（11件 pass）
  - [x] 各サイクルで `write_log` 呼出
  - [x] 過速度／過G で当該パターン打ち切り＋次へ（非常停止しない）
  - [x] 過電流／CAN例外で `on_emergency` 発火
  - [x] 完了で `on_complete` が一度だけ
  - [x] `current_*_opening` 公開

## フェーズ4: コントローラの arm / start / cancel

- [x] `src/app/robot_controller.py` に開始フローを実装
  - [x] `arm_learning_drive()`: READY→PRE_CHECK、走行前チェック→ブレーキを `stop_brake_opening_pct` まで踏み PRE_CHECK 待機
  - [x] `start_learning_drive()` 改修: PRE_CHECK(armed) 前提でセッション開始・RUNNING・`LearningLoop` 起動
  - [x] `cancel_learning_drive()`: PRE_CHECK 前提でブレーキリリース・PRE_CHECK→READY
  - [x] `stop_learning_drive()`（on_complete）: RUNNING→READY・原点復帰・セッション completed
  - [x] 依存欠落時の silent no-op を排除し明示例外で弾く
  - [x] `current_openings`/`current_ref_speed`/`get_realtime_data` が新ループも参照（`_realtime_loop`）。stop/emergency/shutdown も学習ループを停止
  - [x] LearningDriveManagerProtocol を generate_patterns へ更新、レガシー `_active_learning_task` 整理
- [x] `tests/unit/test_robot_controller.py` を新仕様へ更新（arm/start/cancel 経路、129件 pass）

## フェーズ5: ルーターとフロント

- [x] `src/web/routers/drive.py`
  - [x] `POST /drive/learning/arm`（新規）
  - [x] `POST /drive/learning/cancel`（新規）
  - [x] `POST /drive/learning/start`（armed 前提へ改修）
- [x] `src/web/static/js/screens/learning.js` ＋ `auto-drive.js`（DriveMonitorScreen に arm フロー用 props 追加）
  - [x] 開始ボタン → arm → 確認ポップアップ「学習運転を開始しますか?」
  - [x] 「はい」→ start / 「いいえ」→ cancel
- [x] Web E2E（全 ASGI スタック）で確認
  - [x] `tests/unit/test_web_drive.py`: arm/start/cancel エンドポイント＋エラーマッピング（409/422）
  - [x] `tests/integration/test_web_api.py`: arm→start シーケンス / arm→cancel シーケンス
  - [x] 連続 `drive_logs` 蓄積→train で逆モデル生成は単体で網羅（LearningLoop の write_log・model_training・/learning/train。スタブ harness は DB ログ永続化を持たないため DB バック E2E は対象外）

## フェーズ6: 品質チェックと修正

- [x] すべてのテストが通ることを確認
  - [x] `.venv/bin/pytest` → 593 passed（唯一の失敗 tests/hardware/test_can_receive.py は実 CAN ハード要・環境要因で本変更と無関係）
- [x] リントエラーがないことを確認
  - [x] `.venv/bin/ruff check`（変更ファイルは All checks passed。sample/scripts/actuator_driver の既存債務は対象外）
- [x] 型エラーがないことを確認
  - [x] `.venv/bin/mypy src`（変更ファイルは型エラーなし。calibration.py/app.py の2件は本変更外の既存エラー）

## フェーズ8: コードレビュー指摘の修正（/code-review high）

- [x] #1【重大】完了経路の自タスク join バグを修正
  - [x] `LearningLoop.stop_and_join` / `DriveLoop.stop_and_join` に `task is asyncio.current_task()` ガード追加（on_complete→stop_*_drive 経路で自タスクを 2秒 join→cancel し home_return が中断され原点復帰しない問題。実機相当スクリプトで 2.06s→0.04s・home_return 実行を確認）
  - [x] 回帰テスト `test_stop_and_join_from_within_cycle_returns_immediately`
- [x] #4 過速度・過G スキップに連続成立デバウンス（既定2サイクル）を追加し単発 CAN ノイズ誤検知を防止＋テスト
- [x] #2 クリープ計測の連続リリース化（CREEP は RETURN を挟まず次の弱ブレーキへ直行）で pedal-off 連続サンプルを確保＋テスト
- [x] #3 確認ポップアップ未確定での画面離脱時に保持ブレーキを best-effort で cancel（DriveMonitorScreen アンマウント cleanup）
- [x] 全テスト 595 passed / 変更ファイル ruff・mypy クリーン

## フェーズ9: 実機フィードバック対応（arm 順序の修正）

- [x] 走行開始ボタンで「ブレーキ踏込→車速0収束待ち→走行前チェック」の順に修正（従来は踏む前に車速0判定していたため走行中だと即 422）
  - [x] `arm_learning_drive`: deps → `_apply_brake_hold` → `_wait_for_vehicle_stopped`（10s タイムアウト）→ 走行前チェックの順へ
  - [x] arm 失敗時はブレーキを原点復帰してから READY へロールバック
  - [x] 順序検証テスト `test_arm_brakes_and_waits_stop_before_precheck`（brake→precheck）
  - [x] UC6 シーケンス図・PRD 受け入れ条件を新順序へ更新
- [x] 全テスト 596 passed / 変更ファイル ruff・mypy クリーン

## フェーズ10: 実機フィードバック対応2（位置チェック競合・プロファイル反映）

- [x] 走行前チェックに項目除外を追加（`PreCheckRunner.run(exclude)`＋`ITEM_*` 定数）
- [x] `arm_learning_drive` を2段階チェックに変更
  - [x] 踏込前: 車速確認を除外して実行（両ペダル原点・通信・サーボ等を確認）
  - [x] ブレーキ踏込→車速0収束待ち
  - [x] 踏込後: アクチュエータ位置を除外して実行（保持ブレーキで原点から離れるため）＝「ブレーキ軸=1575pulse で原点から離れています」422 を解消
- [x] プロファイル編集がコントローラに反映されない不具合を修正（`PUT /profiles/{id}` で `refresh_active_profile` を呼ぶ）＝stop_brake_opening_pct 等の編集が arm のブレーキ踏込量に即時反映
- [x] テスト追加（pre_check の exclude／arm の2段階順序・除外項目）。全テスト 598 passed・ruff/mypy クリーン
- [x] UC6 シーケンス図を2段階チェックへ更新

## フェーズ11: 実機ログ分析と改善（クリープ安定待ち・細かい開度）

- [x] 6:40 学習ログ(76s)を分析。`max_decel_g=0.3G(≈10.6km/h/s)` の過Gスキップで各計測パターンが0.2〜0.5秒で打ち切られ、データの42%が RETURN/SPINUP に費やされていた。creep_speed は未更新(7.0)・creep_rate のみ更新(0.94) を確認
- [x] ユーザー判断: 過Gスキップは維持
- [x] クリープ安定待ち `PatternKind.CREEP_SETTLE` を追加（accel=brake=0 で |加速度|<tol が継続するまで保持、min_s/timeout/連続安定で判定）。クリープ解放ステップは >0 で終え、0 は SETTLE が担う
  - [x] `LearningLoopConfig` に creep_settle_* を追加、`_advance_creep_settle` 実装、`_stable_count` 管理
  - [x] テスト追加（CREEP_SETTLE の安定待ち・generate_patterns の SETTLE 生成）
- [x] 開度スイープを 10%→5% 刻みへ（細かく振る。20分以内に収まる）
- [x] UC6・glossary・PRD を新仕様へ更新。全テスト 600 passed・ruff/mypy クリーン

## フェーズ12: 実機ログ分析と改善2（2%刻み・RETURN緩やか化）

- [x] 7:11 学習ログ(133s)を分析。RETURN(0%/30%)が39.6秒(約30%)を占め、減速時に一気に30%へ踏んで急停止していた
- [x] 開度スイープを 5%→2% 刻みへ（さらに細かく振る）
- [x] RETURN のブレーキを 0→目標へレートリミット（既定15%/s）で緩やかに踏み込むよう変更（急停止防止。踏込過程の連続ログもブレーキ学習データに）
  - [x] `LearningLoopConfig.return_brake_rate_pct_s` 追加、`_command_openings(RETURN)` をランプ化、テスト追加
- [x] UC6・glossary・PRD を更新。全テスト 601 passed・ruff/mypy クリーン

## フェーズ7: ドキュメント更新

- [x] `docs/functional-design.md` の UC6 学習運転シーケンスを開度パターン方式（arm/確認/クリープ計測含む）へ更新
  - [x] LearningDriveManager 設計・走行前チェック7項目（車速確認）も更新
  - [x] `docs/glossary.md` の学習運転/走行パターン/運転モデルを新方式へ、削除した学習ログ(LearningLog)用語を撤去
- [x] 実装後の振り返り（このファイルの下部に記録）

## フェーズ13: 連続走行への再構成（加速→減速）と緩やか踏み込み（2026-06-18）

- [x] `src/models/learning_drive.py` の `PatternKind` を更新
  - [x] `ACCEL` / `BRAKE` を削除し `COMBINED` を追加（CREEP / CREEP_SETTLE は維持）
- [x] `src/domain/learning_drive.py` の `generate_patterns` を COMBINED へ再構成
  - [x] CREEP 解放 → CREEP_SETTLE は維持
  - [x] accel/brake スイープを zip（長い方の長さ、短い方は `% len` 巡回）して COMBINED を生成
  - [x] 旧 ACCEL/BRAKE 別ループを削除（全 accel 値・全 brake 値が最低1回現れる）
- [x] `src/domain/control/learning_loop.py` を連続走行＋ランプへ改修
  - [x] `_Phase` から `SPINUP`/`RETURN` を削除し `DRIVE_ACCEL`/`DRIVE_BRAKE` を追加
  - [x] `_initial_phase`: COMBINED→DRIVE_ACCEL、CREEP/CREEP_SETTLE→MEASURE
  - [x] `_command_openings`: DRIVE_ACCEL/DRIVE_BRAKE をランプ化（`target × min(1, elapsed/ramp_time_s)`、`_ramp_fraction`）
  - [x] `_advance`: DRIVE_ACCEL→DRIVE_BRAKE（速度到達/hold/過速度・過G）、DRIVE_BRAKE→次（停車/timeout、過Gは `_brake_ramp_cap` で頭打ち）
  - [x] `LearningLoopConfig`: `accel_ramp_time_s`/`brake_ramp_time_s`/`accel_to_brake_speed_frac`/`brake_stop_timeout_s` 追加、SPINUP/RETURN 系を削除（accel hold は pattern.hold_duration_s を流用）
- [x] `tests/unit/test_learning_drive.py`: COMBINED zip 生成テストへ更新（CREEP/CREEP_SETTLE 維持・14件 pass）
- [x] `tests/unit/test_learning_loop.py`: DRIVE_ACCEL→DRIVE_BRAKE 遷移・ランプ・速度到達遷移・過速度過Gスキップ・過G頭打ち・on_emergency/on_complete を新仕様へ
- [x] 旧 ACCEL/BRAKE/SPINUP/RETURN 前提の既存テスト・参照を更新（robot_controller の `_make_patterns` を COMBINED へ）
- [x] `docs/functional-design.md` LearningDriveManager 節 / `docs/glossary.md` 学習運転・走行パターンを連続走行＋ランプへ更新
- [x] `handover.md` §1 パターン構成・安全・§4 履歴・§5 チューニングパラメータを更新
- [x] 品質チェック: `pytest --ignore=tests/hardware` 605 passed / 変更 Py の ruff・mypy クリーン（既存2件のみ）
- [x] 振り返りを追記

---

## 実装後の振り返り

### 実装完了日
2026-06-17

### 計画と実績の差分

**計画と異なった点**:
- 「`generate_patterns`/`run_pattern` を本番配線」が当初の見立てだったが、`run_pattern` は
  1パターン=1平均値しか返さず先読み逆モデル（連続サンプル間 `dv`）と不整合だったため、
  実行は新規 `LearningLoop`（開ループ・100ms連続ログ・状態機械）に置き換えた。`run_pattern` は削除。
- ユーザー追加要件で、開始フローを arm/start/cancel の3段（ブレーキ保持＋確認ポップアップ）に拡張。
  hold 状態は新状態を足さず PRE_CHECK を流用して状態機械への波及を最小化した。
- 安全方針を「過速度・過Gは非常停止」から「当該パターンをスキップして次の開度へ」に変更
  （過電流・CAN断のみ非常停止の2段階）。
- クリープ車速・加速率は新規推定器を作らず、既存 `estimate_dynamics_params`（train経路）に委ねた
  （ブレーキ緩めフェーズを連続ログに含めることで成立）。
- パターン生成は速度×加速度グリッドから「開度スイープ（CREEP/ACCEL/BRAKE）」へ再定義。

**新たに必要になったタスク**:
- 走行前チェックへ「車速0確認」追加（6→7項目）。整合のため docstring/設計書/UI 文言も更新。
- `RobotController` の stop/emergency_stop/shutdown を `_learning_loop` にも対応（`_realtime_loop`
  共通プロパティでライブ配信も統一）。
- DriveMonitorScreen に arm フロー用 props（driveArmPath/driveCancelPath/confirmStartMessage）を追加し、
  自動走行は無変更のまま学習運転のみ確認ポップアップ経由に。
- glossary の旧用語（学習ログ＝LearningLog、速度×加速度グリッド）の撤去・更新。

**技術的理由でスキップしたタスク**:
- DB バックの「連続 drive_logs 蓄積→train」E2E 単一テストは、スタブ harness が
  ログ永続化（write_log↔list_logs_for_training の結線）を持たないため作成せず。
  代替として LearningLoop の write_log（単体）・model_training（単体）・/learning/train（web）で
  鎖を分割網羅し、arm→start/arm→cancel は全 ASGI スタックの統合テストで担保した。

### 学んだこと

**技術的な学び**:
- 先読み逆モデルの特徴量はすべて「実車速」由来のため、開度の決め方（開ループ指令でも PID 出力でも）
  に依存せず、(実速度, 実加速度, 開度) が物理整合していれば有効な学習データになる。これが
  「連続軌跡として記録すればセッション単位 lookahead がそのまま成立する」設計の根拠になった。
- DriveLoop の call_later スケジューリング／サイクルスキップ・ウォッチドッグ／ログ滞留制御の
  パターンは LearningLoop でも有効で、範として踏襲することで安全機構の取りこぼしを防げた。

**プロセス上の改善点**:
- 着手前にコードを精読して「結線するだけでは済まない2点（ログ未永続化・平均値1点問題）」を
  洗い出し、ステアリングへ明記してから実装したため手戻りが少なかった。
- 仕様追加（arm/確認/クリープ/スキップ型安全）をまず requirements/design/tasklist に反映してから
  実装したことで、フェーズ単位で着実に進められた。

### 次回への改善提案
- スタブ harness にメモリ内 LogWriter↔SessionRepository の結線を用意すれば、DB レスでも
  「走行→ログ蓄積→学習」の真の E2E が書けるようになる（将来の学習系変更の回帰防止に有効）。
- LearningLoop の MEASURE 前進は最大ホールド時間のみで判定している。実機検証後、速度プラトー検知を
  足せば 20 分枠内でのデータ効率を上げられる（今回は単純さ優先で見送り）。

---

## フェーズ13 振り返り（2026-06-18: 連続走行への再構成＋緩やか踏み込み）

### 計画と実績の差分
- ユーザー要望（「アクセルとブレーキを一緒に」「いきなりでなく xx 秒かけて踏む」）を受け、走行構成を
  ACCEL/BRAKE 別スイープ → **1サイクルで加速→減速する `COMBINED`**（accel/brake を zip ペア化）に統合。
  SPINUP・固定30%RETURN を廃止し、MEASURE の一気踏みを **`ramp_time_s` 線形ランプ**へ変更した。
- 当初 config に `accel_hold_timeout_s` を足す案だったが、加速区間の最大保持は既存
  `LearningPattern.hold_duration_s` で十分（低開度パターンが長引かない）と判断し追加せず削減。
- 安全方針: 「過速度・過Gで当該パターン打ち切り（RETURN へ）」を、連続走行に合わせ
  **DRIVE_ACCEL=減速へ移行／DRIVE_BRAKE=ブレーキ頭打ち（`_brake_ramp_cap`）** に再設計。停車を必ず
  経由するため、減速中の過Gで次パターンへ飛ばさない（安全側）。

### 学んだこと
- 逆モデル特徴量が実車速由来である性質（フェーズ1の学び）が効き、加速→減速を1本の連続軌跡に
  まとめても加速区間=純アクセル・減速区間=純ブレーキとして片軸データが成立する。統合は学習面の
  デメリットなしに時間短縮（SPINUP/RETURN 廃止）と実運転らしさを得られた。
- ランプ（緩やか踏み込み）は副次的に過Gスキップの誤発火を減らす（踏み始めの加速度が小さい）。
  安全要件とユーザー体感要望が同じ方向に働いた。

### 次回への改善提案
- `ramp_time_s`（既定1.5s）と `accel_to_brake_speed_frac`（0.5）は実機適合の主役。6:40/7:11 のように
  DB ログで「区間別の所要時間・到達速度・採れた開度範囲」を確認し詰めるとよい（handover §7 のクエリ）。
- zip ペアリングは素直に (2,2)(4,4)… としている。低accel×高brake のような組合せデータが薄い場合は
  ペアリング戦略（オフセット zip 等）を検討。

---

## フェーズ14: 滑らか化・プラトー保持・コースト計測（2026-06-19 実機ログ分析）

実機ログ(437s)分析で判明した4問題（急峻なアクチュエータ動作／アクセル保持不足／エンジンブレーキ未計測／
乱暴なペダルハンドオフ）に対応。プラン: `.claude/plans/docs-result-session-7155f154-a6cc-4059-async-origami.md`

- [x] `src/infra/actuator_driver.py`: `move_to_position_timed(target, current, duration_s)` 追加
  - [x] 距離mm/duration→speed_mm_s 算出、`_MIN/_MAX_TIMED_SPEED_MM_S` クランプ、duration<=0/距離0 は最速フォールバック
  - [x] `tests/unit/infra/test_actuator_driver.py`: VCMD レジスタ・クランプ・フォールバック（27件 pass）
- [x] `src/domain/control/pedal_safety.py`（新規）: `enforce_pedal_exclusion(accel,brake)`（両>0で小さい方0・同値はブレーキ優先）
  - [x] `tests/unit/control/test_pedal_safety.py`（新規）
- [x] `src/models/learning_drive.py`: `PatternKind.COAST_DOWN` 追加
- [x] `src/domain/learning_drive.py`: COAST_DOWN 数本生成＋予算 config（`max_combined_patterns`/`brake_sweep_start_pct`/区分刻み/`coast_down_*`）
  - [x] `_sweep` に start_pct・区分刻みを追加、COMBINED 切詰め
  - [x] `tests/unit/test_learning_drive.py` 更新（20件 pass）
- [x] `src/domain/control/learning_loop.py`: フェーズ機・時間指定移動・プラトー・COAST・安全
  - [x] `_Phase` に COAST 追加（DRIVE_ACCEL→COAST→DRIVE_BRAKE / COAST_DOWN は COAST 後 次へ）
  - [x] フェーズ入場時に1回だけ timed move 発行（`_accel_pos_cmd`/`_brake_pos_cmd`/`_timed_move_pending`、`_drive_or_sense`）
  - [x] DRIVE_ACCEL プラトー保持（0.5×max_speed 通常切替を廃止、overspeed は安全早期離脱、加速側過Gは離脱要因にしない）
  - [x] COAST 計測（速度低下/減速プラトー(COMBINED)/timeout で前進。COAST_DOWN は低速まで惰行→次へ。両0なので過Gは benign）
  - [x] 安全フリーズの再 timed move 化（`_brake_ramp_cap`＋`_timed_move_pending`）、`enforce_pedal_exclusion` 通過
  - [x] `LearningLoopConfig` 追加/削除（`accel_to_brake_speed_frac` 削除）、プラトー/コースト判定は `_stable_since` タイムスタンプ基準
  - [x] `tests/unit/test_learning_loop.py` を新仕様へ全面更新（28件 pass）
  - [x] `src/app/stubs.py` の `_StubActuator` に `move_to_position_timed` 追加
  - [x] robot_controller / drive_loop の `ActuatorDriverProtocol` に `move_to_position_timed` を追加（型整合）
- [x] `src/app/robot_controller.py`: `_brake_pos_cmd` 初期化（保持ブレーキ位置）の確認＝ループが profile から自前算出（コントローラ変更不要）
- [x] `src/domain/control/drive_loop.py`: 最終段に冗長排他ガード（`enforce_pedal_exclusion`）
- [x] 品質チェック: `pytest --ignore=tests/hardware` 630 passed / 変更 Py の ruff・mypy クリーン（既存債務2件のみ）
- [x] ドキュメント（functional-design / glossary）・handover をフェーズ14へ更新
- [x] 振り返りを追記

### フェーズ14 振り返り（2026-06-19: 実機ログ分析駆動の改善）

**計画と実績の差分**:
- 実機ログ(437s)の定量分析で4問題を特定（急峻なアクチュエータ動作・アクセル保持不足・エンジンブレーキ
  未計測・乱暴なハンドオフ）。`accel_to_brake_speed_frac`（0.5×max_speed 即切替）が「定常前にブレーキ」の
  主因と判明し**廃止**、プラトー保持へ置換。
- 「同時踏み」はログ上は指令・物理とも重なり0（accel が1サイクルで74→0に戻る）で、真因は**乱暴な瞬間
  ハンドオフ**だった。COAST 区間挿入で物理分離＋`enforce_pedal_exclusion` を保険に。自動運転は PedalArbiter で
  既に排他済みのため最終段ガードのみ（ユーザー選択）。
- 「滑らかさ」はソフトランプを servo 側の**時間指定移動**へ移譲して解決（ループジッタ非依存）。ログ用開度は
  従来のソフトランプ値を維持し連続軌跡を担保（位置はフェーズ入場時のみ発行）。
- COAST_DOWN を新 `PatternKind` として追加（速度全域のエンジンブレーキカーブ）。COAST 自体はループ内部挿入で
  COMBINED の構造は不変。

**学んだこと**:
- 実機 CSV をフェーズ別に分解（区間ごとの保持時間・到達速度・サンプル間レート）すると、設定値（切替速度・
  ループ周期）が体感問題に直結していると定量的に示せる。`max_decel_g` の過Gスキップと同様、安全つまみが
  データ品質を左右する。
- 加速方向の過Gは「危険」ではなく「特性」。高開度で定常まで保持するには、過Gを離脱要因から外す判断が要る
  （overspeed のみ安全早期離脱に残した）。

**次回への改善提案**:
- `accel_ramp_time_s`/`pedal_release_time_s`/`accel_plateau_*`/`coast_*` は実機適合の主役。新CSVで
  「プラトー到達まで保持できているか」「COAST 区間で減速率が安定して採れているか」「総時間20分以内」を確認。
- 時間指定移動の mid-ramp 再目標（安全フリーズ）が実機でジャーキーなら `_MAX_TIMED_SPEED_MM_S` 低減 or
  `pedal_release_time_s` 増で調整。

---

## フェーズ15: プロファイル包絡線の厳守（G/車速ガバナ）（2026-06-19 実機ログ分析2）

7:09 ログ(558s)分析: 上限開度80%は守られていたが、**最高車速100→実160km/h(31%の時間で超過、最長12.5s)**、
**上限0.3G→加速は開度20%超・ブレーキは22%超で超過(加速1.2G/減速1.5G)**。原因: プラトー保持に車速上限が無く
終端速度100超の開度で突き抜ける／overspeed時COASTで惰行のみ・ブレーキしない／ブレーキ過Gが「頭打ち」のみで
減速Gを下げない。
ユーザー判断: **包絡線ガバナ**で守らせる／**包絡内のみ計測でOK**（自動運転も同じ包絡でしか動かないため高開度は不要）。

- [x] `LearningLoop`: 平滑化加速度（時間窓スロープ `_speed_hist`、履歴不足は差分にフォールバック）を G ガバナ判定に使う
- [x] DRIVE_ACCEL に **G ガバナ**: 平滑加速G ≥ 上限(×`g_limit_frac`)で開度の踏み増しを止め、超過継続で段階的に下げる（`_accel_gov_cap`）
- [x] DRIVE_ACCEL に **車速キャップ**: `speed ≥ max_speed×accel_speed_cap_frac` で COAST へ（max_speed 手前で加速終了）
- [x] DRIVE_BRAKE のブレーキを **G ガバナでモジュレート**（freeze ではなく減速Gが上限超で開度を下げる、`_brake_gov_cap`）
- [x] **overspeed 復帰**: `speed > max_speed`（デバウンス）→ DRIVE_BRAKE（G ガバナ付き）で能動的に減速。COAST 中の超過も同様
- [x] COAST_DOWN/COAST は加速が max_speed 手前で終わるため惰行中も包絡内
- [x] 位置指令を「指令開度が変化した軸のみ時間指定移動」に変更（`_drive_or_sense` 改、`pedal_step_time_s` 追加、`_timed_move_pending` 廃止）
- [x] `LearningLoopConfig`: `accel_speed_cap_frac`/`g_limit_frac`/`g_smoothing_window_s`/`gov_reduce_step_pct`/`pedal_step_time_s` 追加、旧 `_brake_ramp_cap`/`_is_skip_condition` 撤去
- [x] `max_combined_patterns` 40→20（ガバナで高開度は同挙動収束＝冗長。低中開度を密に）
- [x] `tests/unit/test_learning_loop.py`: G ガバナ（加速/減速で開度抑制・reduce）・車速キャップ・overspeed→ブレーキ復帰・on-change 移動を追加/更新（30件 pass）
- [x] 品質チェック: `pytest --ignore=tests/hardware` 632 passed / 変更 Py の ruff・mypy クリーン（既存債務2件のみ）
- [x] docs（functional-design/glossary）・handover をフェーズ15へ更新

### フェーズ15 振り返り（2026-06-19: プロファイル包絡線の厳守）

**計画と実績の差分**:
- 7:09 ログで上限開度は守れていたが最高車速（160/100）と上限G（加速1.2G・減速1.5G/0.3G）が大幅超過。
  フェーズ14のプラトー保持・overspeed→COAST・ブレーキ過G「頭打ち」が、いずれも能動的に包絡へ戻す力を
  持たなかったことが原因。
- 「上限Gを厳守すると高開度が踏めない」という根本対立は、**自動運転も同じ包絡内でしか動かない**という
  気付きで解消（包絡超の高開度は学習不要）。ユーザーも「包絡内のみでOK」を選択。
- 実装: 包絡線ガバナ（平滑Gで開度を止め/下げる）＋車速キャップ＋overspeed能動制動＋ブレーキGモジュレート。
  フェーズ14の「フェーズ入場時に1回だけ移動」は、ガバナが開度を動的に変えるため「指令変化時のみ移動」へ戻した
  （servo の時間指定移動で滑らかさは維持）。
- ガバナで高開度パターンが上限Gに収束＝同挙動で冗長になるため `max_combined_patterns` を 20 に削減。

**学んだこと**:
- G の単発スパイク（CAN ノイズで±170km/h/s）に誤反応しないよう、ガバナ判定は**時間窓スロープの平滑G**が必須。
  履歴不足時は差分にフォールバックさせ、既存テスト（prev_speed/prev_time 注入）との互換も保てた。
- 「学習は包絡を超えて探索すべき」と思い込みがちだが、制御対象（自動運転）の動作範囲＝学習に必要な範囲。
  上限は安全機構であると同時に、学習空間の自然な境界でもある。

**次回への改善提案（要実機検証）**:
- `g_limit_frac`/`accel_speed_cap_frac`/`gov_reduce_step_pct`/`g_smoothing_window_s` は実機適合の主役。新CSVで
  「車速が 100 を超えないか」「加速/減速G が 0.3G 以内か」「総時間 20 分以内か」を最優先で確認。
- ガバナが効くと 1 パターンが長い（0.3G で 90km/h まで≈8.5s）。20 分超過なら `max_combined_patterns`/
  `accel_speed_cap_frac`/各 timeout を下げる。
- COAST_DOWN は `coast_timeout_s=6s` で高速域しか採れない可能性。エンジンブレーキ全域が要るなら COAST_DOWN 専用に
  長めの timeout を分けることを検討。
