# 設計書

## アーキテクチャ概要

「一時停止」を `DriveLoop` の経過時間（`elapsed_s = loop.time() - _started_at`）の凍結として実装する。制御サイクル自体は止めず（PID・安全チェック・アクチュエータ制御・ウォッチドッグは継続）、基準速度を参照する時刻だけを固定する。これにより目標車速が一定値に固定され、PID がその速度を保持し続ける。

新たに `RobotState.PAUSED` を導入し、フロントへは既存の WebSocket `robot_state` 配信（`src/web/ws.py`）にそのまま乗るため WS 側の変更は不要。

```
[フロント auto-drive.js]
  ⏸ 一時停止 → POST /api/v1/drive/pause ┐
  ▶ 走行再開 → POST /api/v1/drive/resume ┘
        │
        ▼
[API drive.py] pause_drive / resume_drive（InvalidStateTransition→409）
        │
        ▼
[RobotController] pause_auto_drive: RUNNING→PAUSED / resume_auto_drive: PAUSED→RUNNING
        │  drive_loop.pause() / drive_loop.resume()
        ▼
[DriveLoop] 経過時間タイムラインを凍結／再開（_paused, _paused_elapsed）
        │
        ▼
[WS broadcast] robot_state=PAUSED をフロントへ配信（変更不要）
```

## コンポーネント設計

### 1. DriveLoop（`src/domain/control/drive_loop.py`）

**責務**:
- 経過時間タイムラインの凍結／再開
- 一時停止中も目標車速を保持しつつ安全チェックを継続

**実装の要点**:
- フィールド `_paused: bool` / `_paused_elapsed: float` を追加
- `pause()`: `_paused_elapsed = loop.time() - _started_at` を記録し `_paused=True`
- `resume()`: `_started_at = loop.time() - _paused_elapsed` でタイムラインを続きから再開、`_last_cycle_time=None`（dt スパイク回避）、`_paused=False`
- `is_paused` プロパティ
- `_execute_one_cycle()`: 一時停止中は `elapsed_s = _paused_elapsed`（凍結値）を使い、正常完了判定（`elapsed_s >= total_duration`）をスキップ。KPI 集計とログ書込もスキップ（逸脱安全チェックは継続）
- `start()` で `_paused` をリセット

### 2. RobotState / RobotController（`src/models/system_state.py`, `src/app/robot_controller.py`）

**責務**:
- PAUSED 状態と遷移検証、pause/resume のオーケストレーション

**実装の要点**:
- `RobotState.PAUSED` を追加
- 遷移テーブル: `RUNNING→{READY, PAUSED, EMERGENCY}`、`PAUSED→{RUNNING, READY, EMERGENCY}`
- `pause_auto_drive()`: RUNNING かつ drive_loop ありを要求 → `drive_loop.pause()` → PAUSED
- `resume_auto_drive()`: PAUSED かつ drive_loop ありを要求 → `drive_loop.resume()` → RUNNING
- `stop()` ガードを `(RUNNING, PAUSED, MANUAL)` に拡張（一時停止中の走行終了を許可）
- `shutdown()` のペダル後始末対象に PAUSED を追加
- モデル学習エンドポイントのブロック対象に PAUSED を追加（走行中扱い）

### 3. API（`src/web/routers/drive.py`）

**責務**: `/pause`・`/resume` エンドポイント

**実装の要点**: `/cancel` の例外マッピングを踏襲（`InvalidStateTransition`→409）

### 4. フロント（`src/web/static/js/screens/auto-drive.js`, `sketch.js`）

**責務**: 3分岐ボタン、プレイヘッド凍結、PAUSED バッジ配色

**実装の要点**:
- `showPause` プロップを受け取る（`AutoDriveScreen` は `showPause={true}`、学習運転は渡さず false）
- 経過時間タイマー: RUNNING/PAUSED の両方を「走行中」とみなす。`pausedAccumMsRef` で一時停止区間の実時間を累積し、`nowS = (Date.now() - driveStart - pausedAccumMs)/1000`。PAUSED 中は rAF を回さずプレイヘッドを凍結（バックエンドの `_started_at` シフトと整合）
- ボタン 3分岐: RUNNING=「⏸ 一時停止（白）＋■ 走行終了（赤）」、PAUSED=「▶ 走行再開（緑）＋■ 走行終了（赤）」、その他=「▶ 走行開始（緑）」。一時停止ボタンは `INK/INK_SOFT/PAPER_2`（白系）で配色
- `sketch.js` の `STATE_TINT` に PAUSED（アンバー）を追加

## データフロー

### 一時停止→再開
```
1. RUNNING 中にユーザーが「⏸ 一時停止」を押す
2. POST /api/v1/drive/pause → pause_auto_drive → drive_loop.pause()
3. DriveLoop が elapsed を凍結、目標車速一定で走行継続。状態 PAUSED を WS 配信
4. フロントがプレイヘッドを凍結、ボタンを「走行再開＋走行終了」に切替
5. ユーザーが「▶ 走行再開」を押す
6. POST /api/v1/drive/resume → resume_auto_drive → drive_loop.resume()
7. _started_at をシフトしてタイムライン継続、状態 RUNNING を WS 配信
```

## エラーハンドリング戦略

- RUNNING/PAUSED 以外での pause/resume は `InvalidStateTransition` → API で 409
- 既存の例外マッピングパターン（`/cancel` 等）を踏襲

## テスト戦略

### ユニットテスト
- `tests/unit/test_drive_loop.py`: pause で elapsed 凍結・ref 一定・自然完了しない・KPI/ログスキップ・アクチュエータ継続、resume でタイムライン継続、start で _paused リセット
- `tests/unit/test_robot_controller.py`: pause_auto_drive(RUNNING→PAUSED)・resume_auto_drive(PAUSED→RUNNING)・不正遷移・PAUSED からの stop()
- `tests/unit/test_models.py`: RobotState の列挙に PAUSED を追加

### 統合テスト
- `tests/integration/test_web_api.py`: /pause→/resume の正常系、状態不一致時の 409

## ディレクトリ構造

```
src/
  domain/control/drive_loop.py     # pause/resume, 経過時間凍結
  models/system_state.py           # RobotState.PAUSED
  app/robot_controller.py          # 遷移テーブル, pause/resume, stop/shutdown 拡張
  web/routers/drive.py             # /pause, /resume, 学習ブロック拡張
  web/static/js/screens/auto-drive.js  # 3分岐ボタン, プレイヘッド凍結
  web/static/js/sketch.js          # PAUSED バッジ配色
tests/
  unit/test_drive_loop.py
  unit/test_robot_controller.py
  unit/test_models.py
  integration/test_web_api.py
```

## 実装の順序

1. DriveLoop に pause/resume と経過時間凍結を実装
2. RobotState.PAUSED とコントローラの pause/resume・遷移テーブルを追加
3. API に /pause /resume を追加
4. フロントに 3分岐ボタン・プレイヘッド凍結・バッジ配色を実装
5. テスト追加と全体検証

## セキュリティ考慮事項

- 特になし（ローカル運用 WebUI）

## パフォーマンス考慮事項

- 一時停止判定は 50ms 制御サイクル内の単純分岐のみで、サイクル予算に影響しない

## 将来の拡張性

- 学習運転（開ループ）の一時停止が必要になれば、LearningLoop 側に同様の凍結機構を追加して `showPause` を渡す
