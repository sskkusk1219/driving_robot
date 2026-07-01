# 設計書

## アーキテクチャ概要

開ループの学習運転を、既存の閉ループ `DriveLoop` と対をなす新コンポーネント `LearningLoop` として
実装する。ドメイン層（`LearningDriveManager`）はパターン生成のみを担い、ハードウェア結合のある
実行・ログ記録・安全監視は `LearningLoop` が担う。開始は「arm（ブレーキ保持）→ 確認 → start/cancel」
の3段で構成する。

```
[学習開始ボタン]
  → POST /drive/learning/arm
      → RobotController.arm_learning_drive
          ├─ 走行前チェック（6項目＋車速0確認）
          └─ 合格 → ブレーキを stop_brake_opening_pct まで踏み保持（PRE_CHECK で待機）
  → UI: 「学習運転を開始しますか?」ポップアップ
      ├─「はい」→ POST /drive/learning/start
      │     → RobotController.start_learning_drive（PRE_CHECK→RUNNING、セッション開始＝ログ開始）
      │         ├─ generate_patterns(profile) で開度スイープ列を生成
      │         └─ LearningLoop 起動
      │               ├─ フェーズ0: クリープ計測（ブレーキを少しずつ緩める）
      │               ├─ フェーズ1: アクセルスイープ
      │               ├─ フェーズ2: ブレーキスイープ（走行中から）
      │               ├─ 100ms 周期で連続 write_log（ref_speed=None）
      │               ├─ 過速度/過G → 当該パターンを打ち切り次の開度へ（非常停止しない）
      │               ├─ 過電流/CAN断/サイクル例外 → on_emergency
      │               └─ 全完了 → on_complete（原点復帰・セッション completed・RUNNING→READY）
      └─「いいえ」→ POST /drive/learning/cancel
            → RobotController.cancel_learning_drive（ブレーキリリース・PRE_CHECK→READY）
  → UI が RUNNING→READY を検知 → autoTrain（train）
  → POST /drive/learning/train（連続 drive_logs から Ridge 逆モデル学習＋
       estimate_dynamics_params がクリープ車速/加速率等を推定しプロファイルへ反映・既存のまま）
```

## コンポーネント設計

### 1. LearningDriveManager（`src/domain/learning_drive.py`・改修）

**責務**: 開度スイープのパターン列を生成する（最大開度でスケール、超過は除外）

**実装の要点**:
- `generate_patterns` を開度スイープへ再定義（速度×加速度グリッド＋線形開度推定をやめる）
  - アクセルスイープ（brake=0、段階的に〜`max_accel_opening`）
  - ブレーキスイープ（accel=0、段階的に〜`max_brake_opening`）
  - クリープ緩め用に、ブレーキを `stop_brake_opening_pct` から段階的に下げる列も生成
- 不要関数を削除: `run_pattern` / `_compute_initial_opening` / `_opening_to_pulse` /
  `build_learning_reference`（閉ループ学習用・本方式では未使用化）
- `LearningPattern`（`src/models/learning_drive.py`）は (accel_opening, brake_opening, hold 条件) を
  表す最小構造へ整理。`LearningLog`（インメモリ集約）は未使用化のため削除

### 2. LearningLoop（`src/domain/control/learning_loop.py`・新規）

**責務**: パターン列を開ループ実行し、走行全体を 100ms 周期で連続ログ記録。スキップ型安全＋非常停止連動

**実装の要点**:
- `DriveLoop` の周期スケジューリング・サイクルスキップガード・例外回収・`stop_and_join` を範に踏襲
- 開度→位置変換は `zero_pos`/`full_pos` 線形補間（`DriveLoop._opening_to_position` と同規約）
- フェーズ構成（連続軌跡として途切れさせない）:
  - フェーズ0 クリープ計測: ブレーキ保持（`stop_brake_opening_pct`）から段階的に緩め、
    車速が立ち上がる過程を記録（後段の `estimate_dynamics_params` がクリープ車速/加速率を抽出）
  - フェーズ1 アクセルスイープ: 各 accel 開度で加速→原点復帰
  - フェーズ2 ブレーキスイープ: 加速後に各 brake 開度を適用→減速
- 各サイクル: 固定開度を位置指令 → `read_current`（過電流判定）→ CAN 実車速取得 →
  過速度/過G判定 → `DriveLogData(ref_speed_kmh=None, ...)` を `write_log`
- **2段階安全**:
  - スキップ型（非常停止しない）: `actual_speed > profile.max_speed`、
    `abs(実測加速度) > profile.max_decel_g`（設定上限Gは加速・減速で同一のため両方向に適用）
    → 当該パターンを打ち切り、原点復帰して次の開度へ。
    `max_speed`/`max_decel_g` はコンストラクタで受け取る `profile` から参照
  - 非常停止型: 過電流（`safety_check.check_overcurrent`）・CAN 読取失敗・アクチュエータ失敗・
    サイクル例外 → `stop()` + `on_emergency`
- パターン前進条件: 速度プラトー到達／上限到達／最大ホールド時間経過。パターン間は原点（開度0）へ戻し
  減速過程も連続記録
- 公開プロパティ: `current_accel_opening`/`current_brake_opening`/`current_ref_speed`(=None)/`last_snapshot`
- 20 分以内: パターン数×ホールド上限を config 化、既定で 20 分以内

### 3. RobotController（`src/app/robot_controller.py`・改修）

**責務**: 学習運転の状態遷移・ブレーキ保持・セッション管理・ループ起動

**実装の要点**:
- `arm_learning_drive()`（新規）: READY→PRE_CHECK、走行前チェック（車速0含む）を実行。合格時は
  ブレーキを `stop_brake_opening_pct` 相当の位置へ移動して保持し、**PRE_CHECK のまま待機**。
  不合格は READY へロールバックして `PreCheckFailed`
- `start_learning_drive()`（改修）: 前提状態 PRE_CHECK（armed）。セッション開始（ログ開始）、
  PRE_CHECK→RUNNING、`generate_patterns` → `LearningLoop` 起動。
  `on_complete`（停止・原点復帰・セッション completed・RUNNING→READY）/ `on_emergency=_dispatch_emergency`
- `cancel_learning_drive()`（新規）: 前提状態 PRE_CHECK。ブレーキをリリース（home_return）し PRE_CHECK→READY
- 依存欠落（profile/calibration/drivers）時は silent no-op をやめ明示例外で弾く
- `current_openings`（`:342`）が新ループも参照するよう保持先を共通化
- ブレーキ保持の開度→位置換算はキャリブレーション（`brake_zero_pos`/`brake_full_pos`）を使用

**hold 状態のモデリング方針**:
ブレーキ保持＋確認待ちは専用状態を追加せず **PRE_CHECK を流用**する（状態機械・フロントの
状態ハンドリングへの波及を最小化）。start は PRE_CHECK→RUNNING、cancel は PRE_CHECK→READY。
これにより既存 UI の「RUNNING→READY で autoTrain」ロジックが無改造で成立する（cancel は
RUNNING を経ないため学習を誘発しない）。

### 4. PreCheckRunner（`src/domain/pre_check.py`・改修）

**責務**: 走行前チェックに「車速0確認」を追加

**実装の要点**:
- `run()` のチェック列に `_check_vehicle_stopped()` を追加。`can.read_speed()` が
  しきい値（例 0.5km/h）未満であることを確認。`StoppedSpeed` 判定しきい値は定数化

### 5. Web ルーター（`src/web/routers/drive.py`・改修）

- `POST /drive/learning/arm`（新規）: `arm_learning_drive` を呼ぶ。PreCheckFailed→422、状態不正→409
- `POST /drive/learning/start`（改修）: armed 前提で `start_learning_drive` を呼ぶ
- `POST /drive/learning/cancel`（新規）: `cancel_learning_drive` を呼ぶ

### 6. UI（`src/web/static/js/screens/learning.js`・改修）

- 開始ボタン押下 → `POST /drive/learning/arm`。成功で確認ポップアップ「学習運転を開始しますか?」
- 「はい」→ `POST /drive/learning/start`、「いいえ」→ `POST /drive/learning/cancel`
- 既存の「RUNNING→READY で autoTrain 自動発火」「基準速度軸非表示」はそのまま活用
- 開始ボタンの導線は `DriveMonitorScreen` の駆動ボタンを arm 起点へ差し替え

### 変更しないもの
- `train_inverse_model` / `estimate_dynamics_params` / `list_logs_for_training`（連続 `drive_logs` を
  そのまま消費。クリープ車速/加速率の推定・反映も既存のまま）
- `POST /drive/learning/train` と `refresh_active_profile` 即時反映

## データフロー

### 学習運転（開度パターン）
```
1. arm: READY→PRE_CHECK、走行前チェック（車速0含む）→ ブレーキを stop_brake_opening まで踏み保持
2. UI: 確認ポップアップ
3a. はい → start: セッション開始（ログ開始）、PRE_CHECK→RUNNING
    → LearningLoop: クリープ緩め → アクセルスイープ → ブレーキスイープ（100ms 連続記録）
    → 過速度/過G はスキップ、過電流/CAN断は非常停止
    → 全完了 → 原点復帰・セッション completed・RUNNING→READY
3b. いいえ → cancel: ブレーキリリース・PRE_CHECK→READY（ログ・学習なし）
4. UI が READY を検知 → train: Ridge 逆モデル学習＋クリープ等物理定数の推定・プロファイル反映
```

## エラーハンドリング戦略
- ループ内 致命異常（過電流/CAN断/サイクル例外）→ `stop()` + `on_emergency`（`_dispatch_emergency`）
- ループ内 運用上限超過（過速度/過G）→ 当該パターン打ち切り＋次パターン（非常停止しない）
- 依存欠落 → 起動前に明示例外で弾く（silent no-op を排除）
- セッションは正常時 `completed` / 非常時 `emergency` で必ず閉じる（既存 `_close_session` 流用）

## テスト戦略

### ユニットテスト（`tests/unit/`）
- パターン生成: 最大開度超過の除外、開度の段階性、アクセル/ブレーキ/クリープ緩め列
- `LearningLoop`: 各サイクルで `write_log`、過速度/過G で当該パターン打ち切り＋次へ（非常停止しない）、
  過電流/CAN例外で `on_emergency`、完了で `on_complete` が一度だけ、`current_*_opening` 公開
- `PreCheckRunner`: 車速 ≠ 0 で NG
- `RobotController`: arm（ブレーキ保持・PRE_CHECK 滞留）、start（RUNNING 遷移・ループ起動）、
  cancel（ブレーキリリース・READY 復帰）

### 統合テスト（`tests/integration/test_web_api.py`）
- スタブ構成で arm → start → 連続 `drive_logs` 蓄積 → train で逆モデル生成・プロファイル反映
- arm → cancel で READY に戻りログが残らないこと

## ディレクトリ構造
```
src/domain/control/learning_loop.py        # 新規: 開ループ実行ループ
src/domain/learning_drive.py               # 改修: generate_patterns 再定義、不要関数削除
src/models/learning_drive.py               # 改修: LearningPattern 整理、LearningLog 削除
src/domain/pre_check.py                     # 改修: 車速0確認を追加
src/app/robot_controller.py                # 改修: arm/start/cancel、current_openings
src/web/routers/drive.py                    # 改修: arm/cancel エンドポイント、start 改修
src/web/static/js/screens/learning.js      # 改修: 確認ポップアップ＋arm/start/cancel 導線
tests/unit/test_learning_drive.py          # 改修
tests/unit/test_learning_loop.py           # 新規
tests/unit/test_pre_check.py               # 改修（車速0）
tests/unit/test_robot_controller.py        # 改修（arm/start/cancel）
tests/integration/test_web_api.py          # 改修（arm→start→train、arm→cancel）
```

## 実装の順序
1. パターン生成の再定義＋不要関数削除＋ユニットテスト更新
2. 走行前チェックへ車速0確認を追加＋テスト
3. `LearningLoop` 新設（フェーズ構成・2段階安全・連続ログ）＋ユニットテスト
4. `RobotController` の arm/start/cancel ＋ `current_openings` 共通化＋テスト
5. ルーター（arm/cancel/start）とフロント（ポップアップ導線）＋統合テスト
6. 品質チェック（ruff/mypy/pytest）

## セキュリティ考慮事項
- 該当なし（外部公開 I/F は学習系エンドポイントの追加のみ）

## パフォーマンス考慮事項
- 100ms 周期の `write_log` は既存 DriveLoop と同等。サイクルスキップガードを踏襲

## 将来の拡張性
- 2 段階学習（開度→G）を見据え、パターン生成と実行ループを分離。将来 G パターン生成器を
  追加すれば同じ `LearningLoop` で実行できる構成にする
- hold 状態を将来きちんと分けたくなった場合に備え、arm/start/cancel の責務は明確に分離しておく

---

## 追加設計（2026-06-18）: 連続走行への再構成と緩やか踏み込み

### 走行構成（旧 ACCEL/BRAKE 別建て → COMBINED 連続走行）

```
1サイクル（COMBINED）:
  DRIVE_ACCEL: accel 0→target を ramp_time_s で踏み込み、brake=0 で加速
      → 速度が accel_to_brake_speed（max_speed×frac）到達 / hold 超過 / 過速度・過G(デバウンス) で次へ
  DRIVE_BRAKE: accel=0、brake 0→target を ramp_time_s で踏み込み、減速
      → 停車（speed≤STOP）/ brake_stop_timeout で次パターンへ
```

CREEP / CREEP_SETTLE は現状維持（MEASURE フェーズ、RETURN を挟まない連続リリース）。

### `LearningPattern` / `PatternKind`（`src/models/learning_drive.py`・改修）

- `PatternKind` から `ACCEL` / `BRAKE` を削除し `COMBINED` を追加（CREEP / CREEP_SETTLE は維持）
- `LearningPattern` は (kind, accel_opening, brake_opening, hold_duration_s) のまま。
  COMBINED は accel_opening・brake_opening の両方を持つ

### `LearningDriveManager.generate_patterns`（`src/domain/learning_drive.py`・改修）

- CREEP 解放列 → CREEP_SETTLE（現状維持）
- COMBINED 列: `_sweep(accel_step, max_accel)` と `_sweep(brake_step, max_brake)` を生成し、
  **長い方の長さだけ zip（短い方は `idx % len` で巡回）**して (accel, brake) ペアの COMBINED を並べる
- 旧 ACCEL/BRAKE 別ループは削除

### `LearningLoop`（`src/domain/control/learning_loop.py`・改修）

- `_Phase`: `SPINUP`/`RETURN` を廃止し `DRIVE_ACCEL` / `DRIVE_BRAKE` を追加（`MEASURE`=CREEP系, `DONE` は維持）
- `_initial_phase`: COMBINED → `DRIVE_ACCEL`、CREEP/CREEP_SETTLE → `MEASURE`
- `_command_openings`:
  - `DRIVE_ACCEL`: `accel = target_accel × min(1, elapsed/accel_ramp_time_s)`（クランプ）, `brake = 0`
  - `DRIVE_BRAKE`: `accel = 0`, `brake = target_brake × min(1, elapsed/brake_ramp_time_s)`（クランプ）
  - `MEASURE`: 従来どおり pattern の固定開度（CREEP の段階リリース）
- `_advance`:
  - `DRIVE_ACCEL`: `speed ≥ accel_to_brake_speed` / `elapsed ≥ pattern.hold_duration_s` /
    過速度・過G(デバウンス) → `DRIVE_BRAKE` へ（加速をやめ減速に移る＝安全側）
  - `DRIVE_BRAKE`: `speed ≤ STOP_SPEED` / `elapsed ≥ brake_stop_timeout_s` → 次パターン。
    過Gデバウンス成立時はブレーキランプを**現在値で頭打ち**にし（それ以上踏み増さない）減速Gを上限内に保つ
  - MEASURE（CREEP/CREEP_SETTLE）は現状維持
- `LearningLoopConfig` 改修:
  - 追加: `accel_ramp_time_s`(既定1.5), `brake_ramp_time_s`(既定1.5),
    `accel_to_brake_speed_frac`(=旧 spinup_target_speed_frac 0.5), `accel_hold_timeout_s`(=旧 spinup_timeout_s),
    `brake_stop_timeout_s`(=旧 return_timeout_s)
  - 削除: `spinup_accel_opening_pct` / `return_brake_opening_pct` / `return_brake_rate_pct_s`
  - 維持: `skip_consecutive_required`, `creep_settle_*`
- スケジューリング・連続ログ・2段階安全（スキップ型/非常停止型）・`stop_and_join` 自タスクガードは現状維持

### RobotController / ルーター / フロント

- 変更不要（arm/start/cancel、`current_*_opening` ライブ表示、autoTrain 連動はそのまま機能）

### テスト

- `tests/unit/test_learning_drive.py`: COMBINED zip 生成（全開度網羅・短い方巡回）、CREEP/CREEP_SETTLE 維持
- `tests/unit/test_learning_loop.py`: DRIVE_ACCEL→DRIVE_BRAKE 遷移、ランプ（elapsed に応じ 0→target）、
  accel_to_brake_speed 到達遷移、過速度・過G スキップ（デバウンス）、過電流/CAN で on_emergency、完了で on_complete 一度
- 既存テストの旧 ACCEL/BRAKE/SPINUP/RETURN 前提箇所を新仕様へ更新

### ドキュメント

- `docs/functional-design.md` UC6 / 走行パターン構成、`docs/glossary.md` 走行パターン（ACCEL/BRAKE→COMBINED, ランプ）を更新
- 引き継ぎ書（handover.md）§1 パターン構成・§5 チューニングパラメータを更新
