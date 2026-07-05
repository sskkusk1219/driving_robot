# 設計書: タイムスケジュールUI修正（ループ削除・P/N/D排他・ペダルプレビュー）

## アーキテクチャ概要

前回実装（表エディタ）とは異なり、今回は**バックエンドを含む全層**を変更する。

```
[UI] schedule-sequence.js
  - loop チェックボックス・一覧「ループ」列を削除
  - toggleButton: P/N/D(ch1-3) 相互排他ロジックを追加
  - プレビューBoxに Acc/Brk 開度推移を追加描画
      ↕ payload から loop を削除
[API] schemas.py / routers/schedules.py
  - ScheduleResponse / ScheduleDetailResponse / CreateRequest / UpdateRequest から loop 削除
[Domain] models/time_schedule.py / domain/control/schedule_loop.py
  - TimeSchedule.loop フィールド削除
  - ScheduleLoop: 終端到達時は常に on_complete（巻き戻し分岐を削除）
[Infra] infra/schedule_repository.py
  - INSERT/UPDATE/SELECT から loop 列を削除
[DB] scripts/setup_db.py
  - CREATE TABLE から loop 列を削除 + 既存DB向け ALTER TABLE DROP COLUMN IF EXISTS
```

## コンポーネント設計

### 1. ループ再生機能の削除（バックエンド→フロント の順で変更）

**責務**: `loop` という概念をコードベース全体から除去する。

**実装の要点**:
- `src/models/time_schedule.py`: `TimeSchedule` dataclass から `loop: bool` フィールドを削除
- `src/infra/schedule_repository.py`: `_row_to_schedule` / `create` / `update` の SQL・引数から `loop` を削除
- `src/domain/control/schedule_loop.py`: `_execute_one_cycle` の終端判定
  ```python
  # 変更前: if self._schedule.loop: 巻き戻し else: on_complete
  # 変更後: 常に self.stop(); await self._on_complete()
  ```
- `src/web/schemas.py`: `ScheduleResponse` / `ScheduleDetailResponse` / `ScheduleCreateRequest` /
  `ScheduleUpdateRequest` から `loop` フィールドを削除
- `src/web/routers/schedules.py`: `_to_response` / `_to_detail_response` / `create_schedule` /
  `update_schedule` の `loop=` 引数を削除
- `scripts/setup_db.py`: `time_schedules` の `CREATE TABLE` から `loop` 列を削除し、
  既存DB向けに `ALTER TABLE time_schedules DROP COLUMN IF EXISTS loop` を追加
  （`vehicle_profiles` の既存マイグレーションパターンに合わせる）
- `src/web/static/js/screens/schedule-sequence.js`:
  - `ScheduleEditForm` の `loop` state・チェックボックス・保存payloadの `loop` を削除
  - `ScheduleScreen` 一覧の「ループ」列ヘッダ・セルを削除、`gridCols` を調整

**テストへの反映**:
- `tests/unit/test_time_schedule.py`: `TimeSchedule` 生成から `loop=` を削除
- `tests/unit/test_schedule_loop.py`: `_make_schedule` の `loop` 引数・
  `test_loop_resets_instead_of_completing` を削除、
  `test_completion_calls_on_complete_when_not_loop` は「終端到達で常にon_complete」テストとして残す（名称調整）
- `tests/unit/infra/test_schedule_repository.py`: フィクスチャ・アサーションから `loop` を削除
- `tests/integration/test_web_api.py`: リクエスト/レスポンスの `loop` フィールドを削除

### 2. P/N/D 相互排他（フロントのみ）

**責務**: 同一行内でチャンネル1(P)/2(N)/3(D) が同時にONにならないようにする。

**実装の要点**:
- `toggleButton(idx, ch)` を変更: `ch` が `1,2,3` のいずれかで、かつこれからONにする操作の場合、
  同じ行の他の `1,2,3` チャンネルを強制OFFにしてから対象chをONにする
  ```js
  const EXCLUSIVE_CHANNELS = [1, 2, 3];
  function toggleButton(idx, ch) {
    setRows(rs => rs.map((r, i) => {
      if (i !== idx) return r;
      const turningOn = !r.buttons[ch];
      const buttons = { ...r.buttons };
      if (turningOn && EXCLUSIVE_CHANNELS.includes(ch)) {
        EXCLUSIVE_CHANNELS.forEach(c => { if (c !== ch) delete buttons[c]; });
      }
      buttons[ch] = turningOn;
      return { ...r, buttons };
    }));
  }
  ```
- OFFにする操作（既にONの列を押下）の場合は排他ロジックを適用しない（単純トグルでOFF）
- 「最後に押したONを優先」という要求は、クリック＝最新操作なのでこのハンドラの実行順序で
  自然に満たされる（追加の優先度管理は不要）

### 3. ペダル開度プレビュー（フロントのみ）

**責務**: 「ボタン押下プレビュー」Box内にAcc/Brkの開度推移を追加表示する。

**実装の要点**:
- 既存 `computeIntervals` はON/OFF区間のみを扱うため、Acc/Brk用に別途
  `computePedalSegments(rows)` を追加: 隣接する行のペアごとに
  `{ startTime, endTime, accelStart, accelEnd, brakeStart, brakeEnd }` を返す
- 描画: 各chの行と同じ横幅レイアウト（ラベル + バー + テキスト）で "Acc" "Brk" の2行を追加
  - バーは区間ごとに `<div>` を敷き詰め、`background: linear-gradient(to right, rgba(color,startOpacity), rgba(color,endOpacity))`
    で開度を不透明度として表現（0%=透明、100%=不透明）
  - 色は既存の auto-drive.js のペダル配色に合わせる: Acc=`#78c8f0`、Brk=`#f07070`
  - ラベル横にテキストで現在の開度域（例: 最小-最大%）を表示するか検討 →
    シンプルさ優先でバーのみ＋ホバーtitleで区間の開度を表示（既存ボタンプレビューのtitleパターンを流用）
- 配置順序: Acc・Brk を上、既存のch別ボタン行を下（ペダルが主、ボタンが副という画面の主従関係に合わせる）

## データフロー

### ループ削除後のスケジュール保存
```
1. ScheduleEditForm で保存 → rowsToApi(rows, visibleChannels) → payload（loop無し）
2. POST/PUT /api/v1/schedules/ → ScheduleCreateRequest/UpdateRequest（loopフィールド無し）
3. TimeSchedule 生成（loop無し）→ repo.create/update（SQLにloop列無し）
```

### スケジュール実行（ScheduleLoop）
```
1. t >= total_duration に到達
2. 常に stop() → on_complete()（従来のloop分岐を削除）
```

## エラーハンドリング戦略

変更なし（既存の409/404/422ハンドリングを維持）。

## テスト戦略

### ユニットテスト
- `test_time_schedule.py`: loop フィールド削除後の TimeSchedule 生成確認
- `test_schedule_loop.py`: 終端到達時に常に on_complete が呼ばれることを確認（loop分岐削除の回帰確認）
- `test_schedule_repository.py`: loop列無しでのCRUD往復確認

### 統合テスト
- `test_web_api.py`: スケジュールCRUD APIレスポンスに loop が含まれないことを確認

### フロントエンド
- Playwright MCP で以下を確認:
  - 編集画面にループチェックボックスが無い
  - 一覧にループ列が無い
  - P列ONの状態でN列をONにするとP列が自動OFFになる
  - Acc/Brkプレビューが表示され、開度変化に応じてバーが変化する

## 依存ライブラリ

追加なし。

## ディレクトリ構造

```
scripts/setup_db.py                                # loop列削除 + DROP COLUMN マイグレーション追加
src/models/time_schedule.py                        # loop フィールド削除
src/infra/schedule_repository.py                   # loop 列参照削除
src/domain/control/schedule_loop.py                # 終端判定からloop分岐削除
src/web/schemas.py                                 # loop フィールド削除
src/web/routers/schedules.py                       # loop 参照削除
src/web/static/js/screens/schedule-sequence.js      # loop UI削除・P/N/D排他・ペダルプレビュー追加
tests/unit/test_time_schedule.py                    # loop 削除に追従
tests/unit/test_schedule_loop.py                    # loop 削除に追従
tests/unit/infra/test_schedule_repository.py        # loop 削除に追従
tests/integration/test_web_api.py                   # loop 削除に追従
docs/functional-design.md                           # TimeSchedule定義からloop削除
docs/glossary.md                                    # 「ループ再生」記述を削除
```

## 実装の順序

1. バックエンド: モデル→リポジトリ→ドメイン(ScheduleLoop)→スキーマ→ルーター→DB migration
2. バックエンドのテスト更新（pytest回帰確認）
3. フロント: loop UI削除
4. フロント: P/N/D排他制御
5. フロント: ペダル開度プレビュー追加
6. ドキュメント更新（functional-design.md / glossary.md）
7. Playwright MCP で実UI確認 → pytest / ruff 最終確認

## セキュリティ考慮事項

なし（入力バリデーションの変更なし）。

## パフォーマンス考慮事項

なし（既存同様、数十行規模を想定）。

## 将来の拡張性

- 排他グループ（EXCLUSIVE_CHANNELS）は配列定義のため、将来シフト以外の排他グループが
  必要になった場合も同じパターンで拡張できる。
