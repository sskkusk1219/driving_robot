# タスクリスト: タイムスケジュールの同時踏み許可

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

---

## フェーズ1: 実装

- [x] T1: `src/domain/control/schedule_loop.py` から `enforce_pedal_exclusion` の
      import と呼び出しを削除
- [x] T2: `tests/unit/test_schedule_loop.py` に同時踏み許可の回帰テストを追加
      （`test_simultaneous_pedal_press_is_not_excluded`）
- [x] T3: `src/web/static/js/screens/schedule-sequence.js` の `validateSchedule` から
      Acc/Brk同時入力の警告を削除（`warnings` 機構がこの1件のみだったため
      `handleSave` の `warnings.forEach` 呼び出しごと削除）

## フェーズ2: 検証

- [x] T4: `pytest` 実行し全件パスを確認（916 passed）
- [x] T5: `ruff check` 実行しパスを確認（All checks passed!）
- [x] T6: `DriveLoop` / `LearningLoop` の既存の排他テストに回帰がないことを確認
      （`test_drive_loop.py` / `test_learning_loop.py` / `test_pedal_safety.py` とも
      pytest全件パスに含まれ、既存の `enforce_pedal_exclusion` 呼び出しは無変更のまま通過）
- [x] T7: 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-07-05

### 計画と実績の差分

**計画と異なった点**:
- design.mdでは「警告push処理の削除」のみを想定していたが、実装時に `warnings` 機構が
  この1件の警告のためだけに存在していたことに気づき、`validateSchedule` の戻り値・
  `handleSave` 側の `warnings.forEach` 呼び出しごと削除した（未使用コードを残さない
  という開発ガイドラインの原則に沿った判断）。

### 学んだこと

**技術的な学び**:
- `enforce_pedal_exclusion` は `DriveLoop`・`LearningLoop`・`ScheduleLoop` の3ループが
  個別に呼び出す設計だったため、1ループだけ挙動を変える変更は該当ループのimport・呼び出し
  1箇所を消すだけで完結し、他ループへの影響がないことをpytest全件（916 passed）で確認できた。
- Playwright MCPで実際にAcc=30/Brk=40の同一行を保存し、`GET /api/v1/schedules/{id}` の
  生データで両方の値がそのまま(排他されず)保存されていることを確認した。

### 次回への改善提案
- 特になし。

