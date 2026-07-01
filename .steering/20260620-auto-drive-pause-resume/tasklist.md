# タスクリスト

## フェーズ1: 制御ループ（DriveLoop）

- [x] `_paused` / `_paused_elapsed` フィールドを追加（`__init__`・型注釈・`start()` でリセット）
- [x] `pause()` を実装（実行中かつ未一時停止のとき経過時間を凍結）
- [x] `resume()` を実装（`_started_at` シフトでタイムライン継続、`_last_cycle_time` リセット）
- [x] `is_paused` プロパティを追加
- [x] `_execute_one_cycle()` を一時停止対応に（凍結 elapsed 使用・正常完了スキップ・KPI/ログスキップ）

## フェーズ2: 状態・コントローラ

- [x] `RobotState.PAUSED` を追加（`src/models/system_state.py`）
- [x] 遷移テーブルに RUNNING→PAUSED, PAUSED→{RUNNING, READY, EMERGENCY} を追加
- [x] `pause_auto_drive()` / `resume_auto_drive()` を実装
- [x] `stop()` ガードを `(RUNNING, PAUSED, MANUAL)` に拡張
- [x] `shutdown()` のペダル後始末対象に PAUSED を追加
- [x] モデル学習ブロック対象（`drive.py`）に PAUSED を追加

## フェーズ3: API

- [x] `POST /api/v1/drive/pause` を追加（InvalidStateTransition→409）
- [x] `POST /api/v1/drive/resume` を追加（InvalidStateTransition→409）

## フェーズ4: フロントエンド

- [x] `DriveMonitorScreen` に `showPause` プロップを追加
- [x] 経過時間タイマーを RUNNING/PAUSED 対応に（`pausedAccumMsRef` でプレイヘッド凍結）
- [x] `handlePause` / `handleResume` を追加
- [x] ボタンを 3分岐に（一時停止=白、走行再開=緑、走行終了=赤）／並び順を「一時停止→走行終了」に
- [x] `sketch.js` の `STATE_TINT` に PAUSED（アンバー）を追加

## フェーズ5: テストと品質チェック

- [x] `test_drive_loop.py` に pause/resume テストを追加（凍結・継続・KPI/ログスキップ・アクチュエータ継続）
- [x] `test_robot_controller.py` に pause/resume・不正遷移・PAUSED からの stop テストを追加
- [x] `test_models.py` の RobotState 列挙に PAUSED を追加
- [x] `test_web_api.py` に /pause→/resume と 409 テストを追加
- [x] すべてのテストが通ることを確認（`.venv/bin/pytest tests/unit tests/integration` → 664 passed）
- [x] リントエラーがないことを確認（`.venv/bin/ruff check src/` → All checks passed）
- [x] 型エラーがないことを確認（`.venv/bin/mypy` → no issues）
- [x] WebUI が 0 console error でレンダリングされることを Playwright で確認

---

## 実装後の振り返り

### 実装完了日
2026-06-20

### 計画と実績の差分

**計画と異なった点**:
- 当初プランにはなかった UI 配色・並び順の要望（一時停止ボタンを白に、並びを「一時停止→走行終了」）を実装中にユーザーから受領し反映した。
- レビュー過程で、当初プランに含めていなかった以下の波及箇所にも PAUSED 対応を追加した:
  - `RobotController.shutdown()` のペダル後始末対象に PAUSED を追加（一時停止中はペダルが踏まれているため）
  - モデル学習エンドポイント（`drive.py`）のブロック対象に PAUSED を追加（一時停止も走行中扱い）
  - `tests/unit/test_models.py` の RobotState 網羅テストに PAUSED を追加（既存テストが失敗したため）

**新たに必要になったタスク**:
- 上記 shutdown / 学習ブロック / test_models の3点。RobotState を増やすと網羅的に状態を列挙している箇所の追従が必要、という気づきから追加した。

**技術的理由でスキップしたタスク**:
- なし（全タスク完了）。
- ただし「実機での RUNNING→PAUSED→RUNNING の動作確認」は、実際の走行（物理ペダル動作）が必要で安全上自律実行しないため未実施。コードロジックは単体・統合テストで網羅済み。サーバ再起動後にユーザー立ち会いで確認する。

### 学んだこと

**技術的な学び**:
- 一時停止は「制御ループを止める」のではなく「基準速度タイムラインの時刻を凍結する」設計が安全（PID・安全チェック・ウォッチドッグを動かし続けたまま目標車速を一定保持できる）。
- フロントのプレイヘッド（`nowS`）も、バックエンドの `_started_at` シフトと整合する形で「一時停止区間の実時間を累積に加える」方式にすると、再開後のタイムラインが連続する。
- 既存の WS 配信が `robot_state` をそのまま流すため、新状態の追加だけでフロントへ伝播でき、WS 層の変更が不要だった。

**プロセス上の改善点**:
- StrEnum の状態を増やす変更は、`_VALID_TRANSITIONS`・`stop`/`shutdown` のガード・学習ブロック・網羅テストなど「状態を列挙している箇所」を grep で洗い出してから着手すると漏れがない。

### 次回への改善提案
- 状態追加時のチェックリスト（遷移テーブル / 各ガード / 網羅テスト / フロントのバッジ配色 / 自動画面遷移）をテンプレ化すると追従漏れを防げる。
- UI 配色・文言は早い段階でユーザーに確認しておくと手戻りが減る。
