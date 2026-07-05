# タスクリスト: タイムスケジュールUI修正（ループ削除・P/N/D排他・ペダルプレビュー）

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: バックエンドからloop削除

- [x] T1: `src/models/time_schedule.py` の `TimeSchedule.loop` フィールドを削除
- [x] T2: `src/infra/schedule_repository.py` の SQL・引数から `loop` を削除
  - [x] `_row_to_schedule`
  - [x] `create`（INSERT文・戻り値の TimeSchedule 生成）
  - [x] `update`（UPDATE文）
- [x] T3: `src/domain/control/schedule_loop.py` の終端判定からloop巻き戻し分岐を削除
  - [x] 常に `self.stop(); await self._on_complete()` にする
- [x] T4: `src/web/schemas.py` から `loop` フィールドを削除
  - [x] `ScheduleResponse` / `ScheduleDetailResponse` / `ScheduleCreateRequest` / `ScheduleUpdateRequest`
- [x] T5: `src/web/routers/schedules.py` の `loop=` 参照を削除
  - [x] `_to_response` / `_to_detail_response` / `create_schedule` / `update_schedule`
- [x] T6: `scripts/setup_db.py` の `time_schedules` DDLからloop列を削除 + 既存DB向け `ALTER TABLE ... DROP COLUMN IF EXISTS loop` を追加

## フェーズ2: バックエンドのテスト更新

- [x] T7: `tests/unit/test_time_schedule.py` から `loop=` を削除
- [x] T8: `tests/unit/test_schedule_loop.py` を更新
  - [x] `_make_schedule` の `loop` 引数を削除
  - [x] `test_loop_resets_instead_of_completing` を削除
  - [x] `test_completion_calls_on_complete_when_not_loop` を「終端到達で常にon_complete」の
        テストとして整理（`test_completion_calls_on_complete_at_timeline_end` に改名）
- [x] T9: `tests/unit/infra/test_schedule_repository.py` から `loop` 参照を削除（`src/app/stubs.py` の
      `InMemoryScheduleRepository` も追従）
- [x] T10: `tests/integration/test_web_api.py` から `loop` フィールドを削除
- [x] T11: `pytest` 実行し全件パスを確認（915 passed。`tests/unit/test_robot_controller.py` の
      `_make_time_schedule` ヘルパーにも `loop=False` の残存があり追加修正）
- [x] T12: `ruff check` 実行しパスを確認（All checks passed!）

## フェーズ3: フロントエンド修正

- [x] T13: `schedule-sequence.js` からループ再生UIを削除
  - [x] `ScheduleEditForm` の `loop` state・チェックボックスを削除
  - [x] 保存payloadから `loop` を削除
  - [x] `ScheduleScreen` 一覧の「ループ」列ヘッダ・セル・`gridCols` を削除/調整
- [x] T14: P/N/D（ch1-3）の同一行相互排他を実装
  - [x] `EXCLUSIVE_CHANNELS = [1,2,3]` を定義
  - [x] `toggleButton` を変更し、ONにする際は同一行の他のP/N/D列を自動OFFにする
- [x] T15: ペダル開度プレビューを追加
  - [x] `computePedalSegments(rows)` 純関数を実装（隣接行ペアごとの開度区間）
  - [x] プレビューBox内に Acc/Brk の推移バーを描画（既存ボタン行の上に配置）
  - [x] 色は Acc=`#78c8f0` / Brk=`#f07070`（auto-drive.jsの配色と統一）

## フェーズ4: ドキュメント更新

- [x] T16: `docs/functional-design.md` の `TimeSchedule` 定義から `loop` フィールドを削除
- [x] T17: `docs/glossary.md` の「タイムスケジュール」説明文から「ループ再生」の記述を削除

## フェーズ5: 検証

- [x] T18: `pytest` / `ruff check` 最終回帰確認（915 passed / All checks passed!）
- [x] T19: Playwright MCP で実ブラウザ確認
  - [x] 編集画面にループチェックボックスが表示されないこと
  - [x] 一覧にループ列が表示されないこと（ヘッダ・データ行とも無し）
  - [x] P列ONの状態でN列をONにするとP列が自動OFFになること（実機で確認。D列も同一ロジックのため対称性から確認済みとみなす）
  - [x] Acc/Brkプレビューが表示され、開度に応じてバーの見た目が変化すること
        （Acc 0→80% のグラデーションバーをスクリーンショットで確認）
  - [x] 既存の保存・編集・削除フローに回帰がないこと（作成→一覧表示→APIレスポンスにloop無し確認→削除→クリーンアップ）
- [x] T20: 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-07-05

### 計画と実績の差分

**計画と異なった点**:
- design.md では `tests/unit/test_schedule_loop.py` 等4ファイルへの `loop` 参照を想定していたが、
  実際には `tests/unit/test_robot_controller.py` の `_make_time_schedule` ヘルパーにも
  `loop=False` が残っており、pytest実行で初めて検出・追加修正した。
  `grep -rn loop` は事前に実行していたが、`src/app/stubs.py`（`InMemoryScheduleRepository`）
  も同様に見落としており、こちらもpyright診断で検出して追従した。
- DBスキーマ変更（`ALTER TABLE ... DROP COLUMN`）は設計通りだが、実運用DBへの適用のため
  `scripts/setup_db.py` 実行 + uvicorn再起動が実際に必要だった（design.mdには明記していたが
  実行手順として重要だったため記録）。

**新たに必要になったタスク**:
- なし（design.md / tasklist.md の記載範囲内で完結。上記2件は既存タスクの追加修正で対応）

### 学んだこと

**技術的な学び**:
- `loop` のようなboolフィールドの全体削除は、モデル/リポジトリ/ドメイン/スキーマ/ルーター/DB の
  6層に加えて、テストヘルパー関数（`_make_schedule` 系）にも独立して同じフィールドが
  複製されていることがあるため、`grep -rn "loop"` だけでなく実際にpytestを実行して
  TypeErrorであぶり出す方が確実だった。
- P/N/D の同一行排他制御は、クリックハンドラ内で「ONにする操作の時だけ同グループの他chを
  強制OFF」という単純なロジックで「最後に押したONを優先」という要求を自然に満たせた
  (優先度管理や履歴保持は不要)。
- CSS `linear-gradient` の各端に8bit hex alpha（`#78c8f0cc`等）を付与する手法で、
  区間ごとの開度変化を追加DOM無しに1つのdivグラデーションとして表現できた。

**プロセス上の改善点**:
- バックエンドのフィールド削除時は、`grep` による網羅確認 → 実装 → `pytest` 実行という順で
  進めることで、grep漏れをテスト失敗から機械的に検出できた（今回の `test_robot_controller.py`
  ケース）。

### 次回への改善提案
- 同様の「フィールド全体削除」作業では、実装直後に一度 `pytest` をフル実行してから
  ドキュメント更新に進む方が、後戻りが減って効率的。
