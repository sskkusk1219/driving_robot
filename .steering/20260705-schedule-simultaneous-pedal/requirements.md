# 要求内容

## 概要

タイムスケジュールモードでは、アクセル・ブレーキの同時踏み（同一時刻で両方>0）を許可する。
排他強制（`enforce_pedal_exclusion`）は「走行モード」（`DriveLoop` による基準車速追従の自動運転）
実行中のみ適用する。

## 背景

これまで `ScheduleLoop` は `LearningLoop`・`DriveLoop` と同様に `enforce_pedal_exclusion` を通して
アクセル・ブレーキの同時踏みを禁止していた。ユーザーからタイムスケジュールモードでは
意図的に同時踏み（例: 停車保持ブレーキを残したまま緩加速するテスト走行）を許可したいとの指示。
一方「走行モード」（基準車速追従の自動運転、`DriveLoop`）は従来通り排他を維持する
（`PedalArbiter` が担う既存の安全設計のため変更しない）。

## 実装対象の機能

### 1. ScheduleLoop の排他強制を撤廃
- `src/domain/control/schedule_loop.py` から `enforce_pedal_exclusion` 呼び出しを削除
- ペダル開度は `interpolate_pedal` → クランプのみを経てそのままアクチュエータへ指令される

### 2. スケジュール編集UIの警告を削除
- `schedule-sequence.js` の `validateSchedule` から「Acc/Brk同時入力は走行時に自動排除される」
  という警告（`warnings` 配列への push）を削除する（同時踏みが正式に許可されるため誤解を招く）

## 受け入れ条件

- [ ] `ScheduleLoop._execute_one_cycle` はアクセル・ブレーキが共に>0のペダル点をそのまま
      アクチュエータへ指令する（どちらかを0にクランプしない）
- [ ] スケジュール編集フォームで同一行のAcc/Brkを両方>0にしても警告が表示されない
- [ ] `DriveLoop`（走行モード）・`LearningLoop`（学習運転）の排他強制は変更しない

## スコープ外

- `DriveLoop` / `LearningLoop` の排他強制ロジック（変更しない）
- `pedal_safety.enforce_pedal_exclusion` 関数自体の削除（他ループが使い続けるため関数は残す）

## 参照ドキュメント

- `docs/functional-design.md` - ScheduleLoop・DriveLoop・LearningLoop の定義
- `.steering/20260705-timeschedule-ui-fixes/` - 直前のタイムスケジュールUI修正作業
