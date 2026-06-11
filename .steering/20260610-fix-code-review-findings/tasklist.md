# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

- 全てのタスクを`[x]`にすること
- 「時間の都合により別タスクとして実施予定」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない
- タスクスキップは技術的理由がある場合のみ（理由を明記）

---

## フェーズ1: 非常停止系統の一元化（C1, C4, C5, C10）

- [x] VALID_TRANSITIONS 修正（ERROR→EMERGENCY 追加、INITIALIZING→ERROR 追加）
- [x] emergency_stop から trigger_emergency 呼び出しを削除（ディスパッチを monitor→controller の一方向に固定）
- [x] factory: SafetyMonitor に controller.emergency_stop を登録、GPIO を trigger_emergency 経由に変更
- [x] スタブ環境（deps.py/stubs.py）の配線も同様に統一（_StubSafetyMonitor を実ディスパッチ化）
- [x] gpio_monitor._fire_callbacks: Future に done_callback を付け例外をログ記録
- [x] drive_loop._execute_one_cycle: アクチュエータ書き込み直前に _running 再チェック

## フェーズ2: CANReader + DriveLoop 堅牢化（C3, C7, C8, C16）

- [x] CANReader: 常駐コンシューマタスク + 最新値キャッシュ + 鮮度チェック（TimeoutError）
- [x] drive_loop: サイクルタスク参照保持 + 重複実行ガード + done_callback で未捕捉例外→非常停止
- [x] drive_loop: write_log タスクの参照保持
- [x] drive_loop: _ref_speed_at の bisect 化

## フェーズ3: 状態機械・シャットダウン（C2, C5, C9, C13）

- [x] shutdown(): ベストエフォートで home_return + servo_off + セッションクローズ
- [x] initialize(): 失敗時 ERROR へ遷移（/clear-error ルート追加は不要に変更: ERROR 画面の再試行ボタンは /initialize を叩くため、initialize() が ERROR→STANDBY→INITIALIZING の再試行を直接受け付ける設計にした）
- [x] start_auto_drive / start_learning_drive / start_manual を共通ヘルパーに統合（_open_session 後の状態再チェック含む）
- [x] start_manual のセッション永続化 + stop_manual の _close_session（ルーターに log_writer 追加）
- [x] /drive/start: mode None で 404

## フェーズ4: Web層（C6, C12, C14, C11）

- [x] JogRequest: axis Literal + step 範囲バリデーション（±5000 = UI ドラッグ最大量）、MANUAL 時の位置クランプ + 全状態で負方向は 0 クランプ
- [x] /learning/train: 状態ガード（走行中 409）+ asyncio.to_thread 化
- [x] DuplicateNameError 導入、modes/profiles create で 409（InMemory リポジトリも同挙動）
- [x] auto-drive.js: 一時停止ボタン削除（バックエンド未実装のため）
- [x] セッション/キャリブのレスポンス変換ヘルパー化（drive.py / sessions.py）

## フェーズ5: プロファイル適用 + 周期単一ソース化（C15）

- [x] PIDController.set_gains / SafetyMonitor.set_stop_config 追加（プロトコル・スタブも）
- [x] select_profile で pid_gains / stop_config を制御スタックに反映
- [x] 50ms 周期を settings → factory → controller → DriveLoop/PID に注入（単一ソース化）

## フェーズ6: WS スナップショット配信（C17）

- [x] DriveLoop にサイクル計測値スナップショット保持を追加
- [x] get_realtime_data: DriveLoop 動作中はキャッシュ返却、ref_speed_kmh も配信

## フェーズ7: 品質チェックと検証

- [x] 挙動変更に伴う既存テストの修正（can_reader 5件 / factory 2件 / robot_controller 2件 / web_drive 3件）
- [x] 新規テスト追加（AC断配線 / _running 再チェック / 状態再チェック / CAN 鮮度・最新値 / set_gains・set_stop_config 反映 / 初期化失敗復旧 / shutdown ペダル解放 / 重複実行ガード / タスク参照保持 / 未捕捉例外→非常停止 / mode 404）
- [x] 全ユニットテスト・統合テストが通ることを確認（484 passed）
- [x] lint（ruff）・型チェック（mypy）が通ることを確認（残存エラーは全て変更前から存在する既存のもの: ruff 3件・mypy 2件、git stash で baseline 比較済み）
- [x] 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-06-10

### 計画と実績の差分

**計画と異なった点**:
- `/clear-error` ルートの新設は不要と判断。フロントの ERROR 画面の再試行ボタンは既に `/api/v1/drive/initialize` を叩く実装だったため、`initialize()` 側が ERROR→STANDBY→INITIALIZING の再試行を直接受け付ける設計にした（フロント変更ゼロで復旧フローが成立）
- レスポンス変換ヘルパー化（フェーズ4予定）は、フェーズ3の `/manual/start` 修正で `_to_session_response` を参照する必要が生じたため前倒しで実施
- DriveLoop のスナップショット保持（フェーズ6予定）は、フェーズ2の DriveLoop 改修と同一関数の編集だったため同時に実装

**新たに必要になったタスク**:
- `_StubSafetyMonitor` を実ディスパッチ化（コールバック保持と trigger_emergency での呼び出し）。開発モードでも本番と同じ非常停止経路をテストできるようにするため
- CAN テストヘルパー `_setup_consumer`: 常駐コンシューマ方式への変更で、旧テストの「get_message を直接モック」パターンが通用しなくなったため

**技術的理由でスキップしたタスク**: なし（全タスク完了）

### 学んだこと

**技術的な学び**:
- `asyncio.ensure_future` の戻り値を破棄するとイベントループは弱参照しか持たず、GC でタスクが実行途中に消える（CPython 公式ドキュメント記載の落とし穴）。制御ループ・ログ書き込み・非常停止コールバックの3箇所で参照保持が必要だった
- `run_coroutine_threadsafe` の Future も同様に、`add_done_callback` で例外を回収しないと非常停止の失敗が完全に黙殺される
- 状態機械の check-then-act は await を挟むと壊れる。`_transition(RUNNING)` 後の DB INSERT 中に GPIO 割り込みが状態を変えるため、await 後の再チェックが必須

**プロセス上の改善点**:
- レビュー検出値（C1〜C17）をタスクに紐づけたことで、修正漏れの確認が容易だった
- 根本原因が同じ複数の指摘（C3/C4/C5/C8 = 「非常停止後もアクチュエータに指令が届く」）をフェーズとしてまとめたことで、設計が一貫した

### 次回への改善提案
- 非常停止系統の変更は今回のように「単一ディスパッチャ + 一方向」の原則を維持すること（新しい停止トリガーは SafetyMonitor への callback 登録ではなく trigger_emergency の呼び出し側に追加する）
- DriveLoop に渡す依存を増やす場合は `_build_and_start_drive_loop` だけを変更すれば auto/learning 両方に効く
- 実機検証時の確認項目: CAN 無音時の非常停止発火（鮮度 1.0s 超過）、AC断での停止、systemd stop 時のペダル解放
