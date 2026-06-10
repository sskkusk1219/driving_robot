# 設計: 初期化ページのハードウェア連動

## アプローチ

バックエンドが初期化の各ステップ進捗を保持し、既存の WebSocket リアルタイム
配信（`broadcast_loop`、100ms 周期）に相乗りして配信する。フロントは疑似
タイマーを廃止し、配信された進捗をそのまま描画する。

## 変更箇所

### 1. `src/models/system_state.py`
- `InitStepStatus`（StrEnum）追加: `PENDING / RUNNING / DONE / SKIPPED / ERROR`
- `InitStep`（dataclass）追加: `key: str`, `label: str`, `status: InitStepStatus`

### 2. `src/app/robot_controller.py`
- `__init__` で `self._init_steps: list[InitStep]` を初期化（`_build_init_steps()`）。
- `_build_init_steps()`: 6 ステップを PENDING で生成。
- `_set_init_step(key, status)`: 指定キーのステップ状態を更新するヘルパ。
- `_fail_running_init_steps()`: RUNNING 中のステップを ERROR にする（例外時）。
- `init_progress` プロパティ: `list[InitStep]` を返す。
- `initialize()` を書き換え:
  - 開始時に `_build_init_steps()` で進捗をリセット。
  - 通信確認をブレーキ→アクセル→CAN の順に逐次実行し、各完了で DONE。
  - アラームリセット・サーボON は両軸 gather のまま、開始で RUNNING / 完了で DONE。
  - 原点復帰は条件付き。実行時は RUNNING→DONE、スキップ時は SKIPPED。
  - 例外発生時は `_fail_running_init_steps()` で ERROR にして再送出。
  - 既存の呼び出し回数（enable_modbus_control/reset_alarm/servo_on/home_return
    が各軸 1 回）は維持し、既存テストを壊さない。

### 3. `src/web/schemas.py`
- `InitStepSchema`（BaseModel）追加: `key`, `label`, `status`。
- `RealtimeData` に `init_steps: list[InitStepSchema] = []` を追加。

### 4. `src/web/ws.py`
- `broadcast_loop` で `controller.init_progress` を `InitStepSchema` に変換して
  `RealtimeData(init_steps=...)` に詰める。

### 5. `src/web/static/js/app.js`
- `INIT_REALTIME` に `init_steps: []` を追加。

### 6. `src/web/static/js/screens/init.js`
- 経過時間ベースの `stepStatus()` と関連 state（`forceUpdate`, `initStartRef`,
  interval）を廃止。
- `realtimeData.init_steps` を描画。配信前（空）は静的ラベルで PENDING 表示。
- ステータス→インジケータ: DONE=✓, RUNNING=⟳, SKIPPED=⤼(スキップ), ERROR=✗,
  PENDING=—。

## 影響範囲・リスク

- WebSocket ペイロードに `init_steps` 配列が増えるが軽量。
- `initialize()` の通信確認が並列→逐次になるが、各操作は短時間。呼び出し回数は不変。
- 状態遷移ルール（VALID_TRANSITIONS）は変更しない。エラー時は INITIALIZING の
  まま再送出し、ステップ ERROR で UI に通知（既存挙動と整合）。
