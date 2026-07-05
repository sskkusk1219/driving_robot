# 設計書: タイムスケジュールの同時踏み許可

## アーキテクチャ概要

`enforce_pedal_exclusion`（`src/domain/control/pedal_safety.py`）は3つの制御ループ
（`DriveLoop` / `LearningLoop` / `ScheduleLoop`）が個別に呼び出す共有ガード関数。
今回は `ScheduleLoop` からの呼び出しのみを削除し、関数自体・他2ループでの利用は変更しない。

```
DriveLoop（走行モード・自動運転）    → enforce_pedal_exclusion を維持（変更なし）
LearningLoop（学習運転）            → enforce_pedal_exclusion を維持（変更なし）
ScheduleLoop（タイムスケジュール）  → enforce_pedal_exclusion 呼び出しを削除
```

## コンポーネント設計

### 1. ScheduleLoop（`src/domain/control/schedule_loop.py`）

**変更点**:
- `from src.domain.control.pedal_safety import enforce_pedal_exclusion` の import を削除
- `_execute_one_cycle` 内:
  ```python
  # 変更前
  accel_opening = self._clamp_accel(accel_opening)
  brake_opening = self._clamp_brake(brake_opening)
  accel_opening, brake_opening = enforce_pedal_exclusion(accel_opening, brake_opening)
  # 変更後（enforce_pedal_exclusion 行を削除）
  accel_opening = self._clamp_accel(accel_opening)
  brake_opening = self._clamp_brake(brake_opening)
  ```
- クランプ（`max_accel_opening` / `max_brake_opening` の上限）は安全包絡として維持する
  （同時踏み許可とプロファイル上限クランプは別の関心事のため）

### 2. スケジュール編集UI（`schedule-sequence.js`）

**変更点**:
- `validateSchedule` 内の以下の警告push処理を削除:
  ```js
  if (r.accel > 0 && r.brake > 0) {
    warnings.push(`${i + 1}行目: Acc/Brk同時入力は走行時に自動排他されます（同時踏み不可）`);
  }
  ```

## テスト戦略

### ユニットテスト
- `tests/unit/test_schedule_loop.py` に新規テストを追加:
  `test_simultaneous_pedal_press_is_not_excluded` — accel・brake共に>0のPedalPointで
  `_execute_one_cycle` を実行し、`current_accel_opening` / `current_brake_opening` が
  共に非ゼロのまま（排他されない）ことを確認
- `DriveLoop` / `LearningLoop` 側の既存の排他テストは変更しない（回帰確認のみ）

### フロントエンド
- Playwright MCP不要（警告メッセージの削除のみ、既存の受け入れ確認手順を流用できるスコープ）。
  コードレビューで警告push箇所の削除を確認する。

## 実装の順序

1. `schedule_loop.py` から `enforce_pedal_exclusion` 呼び出しを削除
2. `test_schedule_loop.py` に同時踏み許可の回帰テストを追加
3. `schedule-sequence.js` の警告文言を削除
4. `pytest` / `ruff check` 回帰確認

## セキュリティ・安全性考慮事項

- `DriveLoop`（走行モード）は `PedalArbiter` 由来の排他保証と `enforce_pedal_exclusion` の
  二重ガードを維持するため、本変更による自動運転時の安全性への影響はない
- `ScheduleLoop` はユーザーが明示的に作成したタイムラインをそのまま再生するため、
  同時踏みは「ユーザーが意図した開度指令」として扱う（既存の `max_accel_opening` /
  `max_brake_opening` クランプによる安全包絡は維持）
