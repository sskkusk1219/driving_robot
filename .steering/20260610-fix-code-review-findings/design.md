# 設計書

## アーキテクチャ概要

既存のレイヤー構造(web → app → domain → infra)は変更しない。非常停止のディスパッチ経路のみ「ソース別の直接配線」から「SafetyMonitor を単一ディスパッチャとする配線」へ変更する。

```
【現状】                          【変更後】
GPIO ──────────→ emergency_stop   GPIO ──→ SafetyMonitor.trigger_emergency ──→ emergency_stop
UPS(AC断) ──→ trigger_emergency   UPS(AC断) ──→ SafetyMonitor.trigger_emergency ─┘
              (コールバック空=無動作)
DriveLoop.on_emergency ──→ emergency_stop   DriveLoop.on_emergency ──→ emergency_stop (変更なし・直接動作)
```

## コンポーネント設計

### 1. 非常停止一元化(factory.py / robot_controller.py / safety_monitor.py / gpio_monitor.py)

**変更点**:
- `factory.build_real_controller`:
  - `safety_monitor.register_emergency_callback(controller.emergency_stop)` を追加
  - GPIO は `gpio_monitor.register_emergency_callback(safety_adapter.trigger_emergency)` に変更(SafetyMonitor 経由)
- `RobotController.emergency_stop`: 内部の `await self._safety_monitor.trigger_emergency()` 呼び出しを削除(自分自身が trigger_emergency のコールバックになるため再帰ループの原因になる。冪等ガードがあるため無限再帰はしないが、ディスパッチの向きを「monitor → controller」の一方向に固定する)
- スタブ環境(`deps.py`/`stubs.py` の組み立て)も同じ配線に揃える
- `VALID_TRANSITIONS`: `ERROR` に `EMERGENCY` を追加(コメントの「BOOTING を除く全状態から EMERGENCY 可」と実装を一致させる)。`INITIALIZING` に `ERROR` を追加(C9)
- `gpio_monitor._fire_callbacks`: `run_coroutine_threadsafe` の Future に `add_done_callback` を付け、例外を `logger.error` で記録

**実装の要点**:
- `emergency_stop` は既に冪等(EMERGENCY なら早期 return)なので、複数ソースからの多重発火は安全
- trigger_emergency の ExceptionGroup は GPIO 経由では done_callback でログされる

### 2. DriveLoop 堅牢化(drive_loop.py)

**変更点**:
- `_schedule_next_cycle`: 生成タスクを `self._cycle_task` に保持(GC対策=C8)。前サイクル未完了時は新サイクルを起動せずスキップし警告ログ(重複実行ガード=C3)
- `_cycle_task` に done_callback を付け、未捕捉例外発生時は stop() + on_emergency をスケジュール
- `_execute_one_cycle`: CAN 読み取り後・アクチュエータ書き込み(gather)直前に `if not self._running: return` を再チェック(C4)
- `_ref_speed_at`: コンストラクタで `self._ref_times = [p.time_s for p in points]` を前計算し `bisect.bisect_right` で区間特定(C16)
- write_log タスクも `self._pending_log_tasks: set` に保持し done で discard(C8)
- サイクル毎に計測値スナップショット(actual_speed / currents / positions / ref_speed)を属性に保持(C17 用)
- コンストラクタに `interval_s` / `log_every_n_cycles` パラメータを追加し、モジュール定数は既定値に降格(50ms 単一ソース化)

### 3. CANReader 最新値キャッシュ(can_reader.py)

**変更点**:
- `connect()` で常駐コンシューマタスクを起動: `AsyncBufferedReader` を常時ドレインし、Speed フレームをデコードして `_latest_speed` / `_latest_at(monotonic)` を更新
- `read_speed()`:
  - キャッシュ未取得なら初回フレームを最大 `_FIRST_FRAME_TIMEOUT_S`(2.0s) 待つ(asyncio.Event)
  - キャッシュ済みでも鮮度が `_MAX_SPEED_AGE_S`(1.0s) を超えていれば `TimeoutError` を送出(DriveLoop が捕捉し非常停止=C3)
  - 鮮度内なら即座にキャッシュ値を返す(ブロックしない=C17 にも寄与)
- `close()` でコンシューマタスクをキャンセル

**実装の要点**:
- キューは常時ドレインされるため無制限成長しない(C7)
- 複数コンシューマ(DriveLoop と WS)がキューを奪い合う問題も解消(両者ともキャッシュ読み)

### 4. 状態機械・シャットダウン(robot_controller.py)

**変更点**:
- `shutdown()`: DriveLoop 停止後、ベストエフォートで `home_return` ×2 → `servo_off` ×2 を try/except 付きで実行(失敗はログのみ)。アクティブセッションがあれば `_close_session("error")`(C2)
- `initialize()`: except 節で `_transition(RobotState.ERROR)` を追加(C9)。`/api/v1/drive/clear-error` ルートが無ければ追加
- `start_auto_drive` / `start_learning_drive` / `start_manual` を共通ヘルパー `_start_drive_session()` に統合:
  - PRE_CHECK プロローグ(遷移→チェック→失敗時 READY ロールバック)を一本化
  - `_open_session` の await 後に `self._state` を再確認し、RUNNING/MANUAL でなければセッションを閉じて `InvalidStateTransition` を送出(C5)
  - manual も `_open_session`/`_close_session` でセッションを永続化(乖離修正)。`/manual/start` ルートに log_writer 依存を追加、`stop_manual` で `_close_session("completed")`

### 5. プロファイル適用(pid.py / safety_monitor.py / robot_controller.py)

**変更点**:
- `PIDController.set_gains(kp, ki, kd)` を追加(reset も実施)
- `SafetyMonitor.set_stop_config(stop_config)` を追加
- `SafetyCheckProtocol` に `set_stop_config` を追加し、stubs の実装にも反映
- `select_profile()`: `pid.set_gains(profile.pid_gains...)` と `safety_check.set_stop_config(profile.stop_config)` を呼ぶ。これで逸脱閾値のソースはプロファイルに一本化(factory のハードコード値はプロファイル未選択時のデフォルトに格下げ)

### 6. Web 層(drive.py / modes.py / profiles.py / schemas.py / ws.py)

**変更点**:
- `JogRequest`: `axis: Literal["accel", "brake"]`、`step: int = Field(..., ge=-5000, le=5000)`(UI のジョグボタン最大値を確認して調整)。`jog_axis` は MANUAL 状態でキャリブレーション済みなら目標位置を [zero, full] 範囲にクランプ
- `/learning/train`: controller を依存に追加し RUNNING/MANUAL/CALIBRATING/PRE_CHECK 中は 409。`train_inverse_model` / `estimate_dynamics_params` を `asyncio.to_thread` で実行(C12)
- `/drive/start`: `mode is None` で 404(C13)
- modes/profiles の `create`: リポジトリ層で `asyncpg.UniqueViolationError` を捕捉し共通例外 `DuplicateNameError`(infra/db.py に定義)へ変換、ルーターで 409(C14)。InMemory リポジトリも同名チェックで同例外を送出し挙動を揃える
- `ws.broadcast_loop`: DriveLoop 動作中は `controller.get_realtime_data()` を呼ばず、controller 経由で DriveLoop のサイクルスナップショットを返す(`get_realtime_data` 内で分岐)。`ref_speed_kmh` も DriveLoop から取得して配信(C17)
- セッションレスポンス変換 `_to_session_response()`(drive.py 3箇所 + sessions.py 2箇所)、キャリブ変換 `_calib_to_response()`(drive.py 2箇所)をヘルパー化
- `auto-drive.js`: 一時停止ボタンと paused 状態を削除(C11)

### 7. 周期の単一ソース化(settings.py / factory.py)

- `factory`: `loop_s = settings.control.loop_interval_ms / 1000` を `PIDController(dt=loop_s)` と `RobotController(control_interval_s=loop_s, log_every_n_cycles=...)` に注入
- `RobotController` は DriveLoop 構築時にこれらを渡す

## エラーハンドリング戦略

- 新規例外: `DuplicateNameError`(infra/db.py)— UNIQUE 制約違反のドメイン表現
- shutdown 中のハード操作失敗は握りつぶさず `_logger.exception` で記録(プロセス終了を妨げない)
- DriveLoop サイクルタスクの未捕捉例外 → ログ + 非常停止(黙殺しない)

## テスト戦略

### ユニットテスト
- 既存テスト(tests/unit)を全て通す。挙動変更箇所(emergency_stop の trigger_emergency 削除、initialize 失敗時 ERROR 遷移、_active_learning_task 削除はしない※)は既存テストを修正
- 追加: AC断配線(SafetyMonitor 登録)、_running 再チェック、状態再チェック(C5)、CANReader 鮮度チェック、JogRequest バリデーション、set_gains/set_stop_config 反映

### 統合テスト
- 既存 tests/integration を全て通す

## 実装の順序

1. フェーズ1: 非常停止系統(C1, C4, C5, C10)
2. フェーズ2: CANReader + DriveLoop 堅牢化(C3, C7, C8, C16)
3. フェーズ3: 状態機械・シャットダウン(C2, C9, C13)+ start 系統合
4. フェーズ4: Web層(C6, C12, C14, C11)+ レスポンスヘルパー
5. フェーズ5: プロファイル適用(C15)+ 周期単一ソース化
6. フェーズ6: WS スナップショット配信(C17)
7. フェーズ7: テスト全実行・lint・検証

## パフォーマンス考慮事項

- read_speed のキャッシュ化により制御サイクルの CAN 待ちがゼロになる(従来は次フレームまで最大100msブロック)。PID の実測値が最大1フレーム周期分古くなるが、従来の「キュー先頭の任意に古いフレーム」より常に新しい
- WS 配信の Modbus トランザクション(4回/100ms)が走行中ゼロになり、制御ループのバス帯域が確保される
