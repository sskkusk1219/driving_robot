# 設計書

## アーキテクチャ概要

変更は `src/domain/control/drive_loop.py` の `DriveLoop` に閉じる。制御アルゴリズム
（FF/PID/調停）・安全チェック・Modbus 通信（`AXIS_PRE_READ_DELAY_S` 含む）は変更しない。

```
[機能1] 完了パスの最終サンプル記録
    _execute_one_cycle 冒頭の完了分岐で、stop() の前に最終ログを 1 行書く
[機能2] 絶対時刻スケジューリング
    _schedule_next_cycle の call_later(interval) を「開始時刻 + n×interval」の
    call_at に置き換え、グリッドを絶対時刻に固定する
```

## コンポーネント設計

### 1. 完了時の最終サンプル記録（`_execute_one_cycle` 完了分岐）

**現状** (`drive_loop.py` 353〜360 行付近):
```python
elapsed_s = loop.time() - self._started_at
if elapsed_s >= self._mode.total_duration:
    self.stop()
    await self._on_complete()
    return
```

**変更方針**:
- 完了分岐に入ったら、`stop()` の前に最終ログを書く。
- 値の出所:
  - `ref_speed_kmh`: `self._ref_speed_at(self._mode.total_duration)`（末端値。
    WLTP_ExHi は 0.0）
  - 実測系（actual_speed / pos / current）: **直近サイクルの値を再利用**する。
    `self._last_snapshot`（`RealtimeSnapshot`）に actual_speed_kmh / accel_pos /
    brake_pos / accel_current_ma / brake_current_ma が揃っている。開度は
    `self._current_accel_opening` / `self._current_brake_opening`。
  - 新たな CAN 読取・Modbus トランザクションは**発行しない**（完了パスを軽量に保ち、
    home_return との競合窓を作らないため）。
- 書き込みは既存の `_enqueue_log_write(DriveLogData(...))` を使う（上限・例外処理を再利用）。
- ガード条件: `self._log_writer and self._session_id and self._last_snapshot is not None`
  のときのみ書く（初回サイクル即完了のような端ケースでは snapshot が無い）。
- `_paused` 中は完了分岐に入らない（既存挙動のまま）。

**二重実行の防止**: 完了分岐は `stop()` で `_running=False` になるため再入しない
（既存の構造で担保される）。最終ログが通常ログ（偶数サイクル）と同一 timestamp 帯に
重複する可能性は、完了分岐が「ログを書く前に return する」現行構造上ない
（完了サイクルは通常ログに到達しない）。

### 2. 絶対時刻スケジューリング（`_schedule_next_cycle` / `start`）

**現状**: `start()` と `_schedule_next_cycle` が `loop.call_later(self._interval_s, ...)`
（相対）で次回を予約 → コールバック遅延が累積（実測 +0.07ms/サイクル → 323s で
約 10 サイクル分早く完了到達）。

**変更方針**:
- サイクル番号 `n`（tick カウンタ、スキップされた tick も進む）を導入し、
  `loop.call_at(self._started_at + (n + 1) * self._interval_s, ...)` で予約する。
- **遅延発火時の丸め**: 発火が遅れて「次グリッド時刻が既に過去」の場合は、
  `n` を現在時刻より未来の最小グリッドまで進める（catch-up バースト発火をしない）。
  こうすることで「前サイクル未完了によるスキップ」の意味（1 tick = 50ms）と
  `stall_summary` の集計単位が現行と互換になる。
- `_started_at` は `start()` で設定済み。一時停止（`pause`/`resume`）は経過時間を
  凍結する既存実装（`_paused_elapsed`）に依存しており、グリッドは実時間のまま
  進み続けるため影響しない（要テスト確認）。
- ウォッチドッグ（`_consecutive_skips` / `WEDGED_CYCLE_TIMEOUT_S`）のロジックは
  変更しない。

**リスクと確認点**:
- `call_at` はイベントループの単調時計（`loop.time()`）基準。`_started_at` も同時計
  なので整合する。
- 万一 tick 処理が恒常的に 50ms を超える環境では、丸めにより実行サイクル数が減るが、
  それは現行の「スキップ」と同じ扱いであり安全側。

### 3. t=0 サンプル（任意）

実装するなら `start()` 内で ref=先頭値・実測は None 相当が無いため書けない
（DriveLogData は non-null）。**推奨: 今回は実装しない**（受け入れ条件にも含めない）。
必要になったら「初回サイクル（n=1、elapsed≈0.05）でもログする」等の別設計で対応。

## エラーハンドリング戦略

- 最終ログの書き込みは `_enqueue_log_write` の既存例外処理（`_on_log_write_done` で
  ログして走行継続＝ここでは終了継続）に委ねる。最終ログの失敗が `_on_complete` を
  妨げてはならない。

## テスト戦略

### ユニットテスト（`tests/unit/test_drive_loop.py` に追加）

1. 完了時に最終ログが 1 行書かれる（ref が total_duration 末端値、実測が直近
   snapshot 値であること）
2. `_last_snapshot` が無い状態で完了しても例外にならずログは書かれない
3. `_on_complete` が 1 回だけ呼ばれる（既存テストの非退行）
4. 絶対時刻スケジューリング: モック時計でサイクル発火時刻がグリッドに張り付くこと、
   遅延発火時に catch-up バーストしないこと
5. pause/resume を挟んでも完了・ログが正しく動くこと（既存テストの非退行）

### 実機検証

- WLTP_ExHi 1 本走行し、受け入れ条件（最終行 ≥322.95s、行数 ≥3229、gap>0.11s = 0 件）
  を DB（drive_logs）の SQL 集計で確認する。集計は複数閾値（>0.11s / >0.2s）で行い、
  生データを直接確認してから結論を出すこと（前ステアリングの教訓）。

## 影響範囲

- `src/domain/control/drive_loop.py` のみ（+ そのテスト）。
- `LearningLoop` は total_duration 終了判定を持たず、`ScheduleLoop` は「ログ→終了判定」の
  順で本問題がないことを確認済み（`schedule_loop.py:234`）。

## 実装の順序

1. 機能1（最終サンプル記録）＋ユニットテスト
2. 機能2（絶対時刻スケジューリング）＋ユニットテスト
3. 全テスト・ruff/mypy
4. 実機 WLTP_ExHi 1 本で受け入れ条件を確認
