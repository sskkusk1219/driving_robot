# 設計書

## アーキテクチャ概要

既存のレイヤ構造(web → app → domain → infra)は維持し、以下の2つの構造的変更を導入する。それ以外は各バグの局所修正。

```
src/domain/control/
├── base_loop.py        ★新規: CycleLoopBase(3ループ共通基盤)
├── conversions.py      ★新規: 開度→パルス変換・クランプ・共通定数
├── drive_loop.py       CycleLoopBase を継承
├── learning_loop.py    CycleLoopBase を継承
└── schedule_loop.py    CycleLoopBase を継承
```

## コンポーネント設計

### 1. CycleLoopBase(フェーズ3 / S1 対応)

**責務**:
- `_schedule_next_cycle` ウォッチドッグ(WEDGED_CYCLE_TIMEOUT_S)・`_on_cycle_done`・`stop_and_join`(self-join デッドロック回避 + shield)・`_abort_emergency`
- ログ書込バックログ管理(`_enqueue_log_write` / `_on_log_write_done` / MAX_PENDING_LOG_TASKS=100)
- ストール計測(`_stall_count`/`_stall_total_s`: 現状 DriveLoop のみ → 全ループに提供)
- `snapshot`/`opening` プロパティ

**実装の要点**:
- 各ループはサイクル本体(1 tick の処理)のみをオーバーライドする
- drive_loop.py の stop_and_join に記録された過去のバグ修正(review #6 バス干渉・self-join ハング)を基底クラスの正とする
- 挙動変更なしのリファクタリングであること(既存テストで担保)

### 2. conversions.py(フェーズ3 / A1, A2 対応)

**責務**:
- `opening_to_position(zero_pos, full_pos, opening_pct)` — 現在3ループ+RobotController×2箇所に重複する変換の唯一の実装
- `clamp_opening(pct, max_opening)` — schedule_loop/learning_loop/learning_drive に重複するクランプの唯一の実装
- 共通定数: `VEHICLE_STOP_SPEED_KMH = 0.5`(現在4箇所で独立定義)、`G_TO_KMHS = 9.81 * 3.6`(現在3箇所で定義)

### 3. FastAPI 例外ハンドラ(フェーズ2・3 / W1, S4 対応)

**責務**:
- `web/app.py` に app レベルの exception handler を登録: `InvalidStateTransition`→409, `PreCheckFailed`→422, `PidTuningAborted`→409
- 各エンドポイントの個別 try/except(drive.py だけで27箇所)を削除。`(InvalidStateTransition, ValueError)`→409 としている箇所はローカルの ValueError catch のみ残す
- これにより W1(select-profile が 500 を返す)は構造的に解消される

## 主要バグの修正方針

### D1: COAST_DOWN オーバースピード回復が無効(learning_loop.py:515)
DRIVE_BRAKE 遷移時のブレーキ開度を `pattern.brake_opening`(COAST_DOWN では 0.0)ではなく、回復用の実効ブレーキ開度(例: `max(pattern.brake_opening, profile.stop_brake_opening_pct)` 相当の回復専用値)にする。回復フェーズの退出条件は現行維持。

### D3: ScheduleLoop のペダル同時踏み(schedule_loop.py:259)
「タイムスケジュールでは同時踏み許可」というコメントは仕様意図として残っているが、pedal_safety.py の機構保護不変条件と矛盾する。**保存時バリデーション**(schemas.py の TimeScheduleSchema validator)で accel/brake の補間値が同時に非ゼロになる区間を reject し、さらに実行時にも `enforce_pedal_exclusion` を通す(既存スケジュールデータへの防御)。※仕様として同時踏みを本当に許可したい場合はユーザーに確認 — Sonnet は上記安全側で実装してよい。

### D6: dt 無クランプでレート制限が破られる(drive_loop.py:409)
`arbitrate()` に渡す dt を PID と同じ方式(公称 tick の 0.5×〜4× にクランプ、pid.py:63 参照)でクランプする。クランプは drive_loop 側ではなく pedal_arbiter.py の `arbitrate()` 入口で行い、全呼び出し元を保護する。

### D2: キャリブレーション接触検出のベースライン汚染(calibration.py:165)
(1) ベースライン取得ウィンドウ中の電流が閾値超なら「既に接触」としてエラー中断、(2) 探索ループに絶対電流上限(SafetyMonitor の過電流閾値を流用)による中断を追加。

### W2/W3: stop() の完了通知漏れと READY 先行遷移(robot_controller.py:653, 668)
- stop() で `_drive_complete.set()` も行う(チューニング走行の待機を即時解放)
- stop()/stop_auto_drive() は home_return/servo_off の **完了後** に READY へ遷移する
- release_stop_hold を呼ぶ except パス(run_pid_tuning_session 等)は現在状態を確認してから home_return する

### W4: RUNNING 遷移前の前提検証(robot_controller.py:1088)
profile/calibration/ff_controller/safety_check の None チェックを `start_auto_drive`/`start_schedule_drive` の **遷移前** に移動し、欠落時は InvalidStateTransition(→409)を送出。ループビルダーの silent return は例外送出に変更。

### W5: LearningCycle の進捗の巻き戻し(learning_cycle.py:164)
start() を try/except で包み、arm/start 失敗時に `_progress` を IDLE 相当へ戻す。アーミング中(_task 生成前)の abort() は「アーム中断」として brake hold 解除+進捗リセットを行う(409 にしない)。

### W6: キャリブレーション中の EMERGENCY 競合(robot_controller.py:809)
finally の `_transition(READY)` を「現在状態が CALIBRATING の場合のみ READY へ」に変更。CalibrationManager に cancel フック(asyncio.Event またはタスク cancel)を追加し、emergency_stop から停止させてから home_return する。

### I1: ボタンサーボ押しっぱなし(button_servo_driver.py:191)
press() を try/finally 化し、finally で rest_angle への復帰を再試行(失敗時はログ+servo off)。CancelledError も finally で復帰してから再送出。

### I3: アーカイブ削除の非トランザクション化(archive_manager.py:153)
2つの DELETE を単一トランザクションに。さらに `_export_to_csv` は「0行ならアーカイブを書かない(既存ファイルを上書きしない)」ガードを追加。

### I4: UPS コールバックの例外握り潰し(ups_monitor.py:216)
イベントループスレッド内からの呼び出しなので `run_coroutine_threadsafe` をやめ `asyncio.create_task` + gpio_monitor.py:147 と同じ失敗ログ用 done-callback に変更。

### I5: update 系の UniqueViolationError 未変換(mode_repository.py:94, profile_repository.py:209)
create と同じ `asyncpg.UniqueViolationError → DuplicateNameError` 変換を update にも追加。

### I6: CANReader.connect の Bus リーク(can_reader.py:90)
DBC 読み込みを Bus オープンの **前** に移動(または except で bus.shutdown() してから re-raise)。

### D4: tuning_cost の逆転回数正規化(pid_tuning.py:320)
`0.2 * reversal / KPI_REVERSAL_WINDOW_S`(時間で割っている)を KPI 上限値(1回/5s)で正規化する意図どおり `0.2 * reversal / KPI_REVERSAL_MAX_PER_WINDOW`(=1)に修正。**注意: チューニングコストの重みが変わるため、修正後に既存プロファイルの再チューニング推奨をユーザーへ報告すること。**

### D5: deadband=0.0 の falsy-zero(pid_tuning.py:141, model_training.py:490-491)
strict `<` を `<=` に変更(brake==0.0 を「踏んでいない」と判定)。schemas.py の deadband フィールドに `ge=0` 制約も追加。

### I2: ArchiveManager のブロッキングI/O + 本番未配線(archive_manager.py:114)
CSV/gzip/disk_usage を `asyncio.to_thread` でオフロード。あわせて「check_and_archive が本番から呼ばれていない」事実を発見済み — 配線(起動時 or セッション終了時に呼ぶ)は仕様判断を要するため、本タスクではオフロード化のみ行い、未配線である旨を振り返りに記録してユーザーへ報告する。

## エラーハンドリング戦略

- 既存のカスタム例外(InvalidStateTransition / PreCheckFailed / PidTuningAborted / DuplicateNameError)を維持し、HTTP マッピングを app レベルハンドラに一元化
- 安全系の except は「ログ+安全側動作(ブレーキ保持/サーボoff)」を原則とし、握り潰し禁止

## テスト戦略

### ユニットテスト
- フェーズ1の各修正に対し、該当シナリオの回帰テストを追加(例: COAST_DOWN 中のオーバースピード→ブレーキ>0、dt=0.95s でも rate_limit ≤ 1tick分、deadband=0.0 で FOPDT セグメント検出)
- CycleLoopBase 抽出は既存テストが全てパスすることで担保

### 統合テスト
- stop→即arm のシーケンスで brake hold が維持されること(W3)
- pre-check 失敗後に learning-cycle progress が IDLE に戻ること(W5)

### 検証コマンド
- `pytest`
- `ruff check src/ tests/`
- `ruff format --check src/ tests/`
