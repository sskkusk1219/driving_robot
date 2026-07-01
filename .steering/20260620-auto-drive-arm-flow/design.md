# 設計: 自動走行 arm フロー

## 方針

学習運転の arm/cancel ロジックは自動走行とほぼ同一（停車保持ブレーキ踏込 → 車速0待ち →
二段階走行前チェック）。`learning_manager` の有無チェックだけが学習固有なので、共通処理を
`RobotController._arm_drive()` / `_cancel_armed_drive()` に抽出し、学習・自動の双方から再利用する。

フロント側の `DriveMonitorScreen` は既に arm フロー（`driveArmPath` / `driveCancelPath` /
`confirmStartMessage`）を学習運転で利用しているため、自動走行画面に同じ props を渡すだけでよい。

## バックエンド変更（`src/app/robot_controller.py`）

- `_arm_drive(*, require_learning_manager)`: 既存 `arm_learning_drive` の本体を抽出。
  READY → PRE_CHECK。踏込前チェック（車速除外）→ 停車保持ブレーキ → 車速0待ち →
  踏込後チェック（位置除外）。失敗時はブレーキ原点復帰して READY ロールバック。
- `arm_learning_drive()` → `_arm_drive(require_learning_manager=True)` の薄いラッパへ。
- `arm_auto_drive()` → `_arm_drive(require_learning_manager=False)` を新規追加。
- `_cancel_armed_drive()`: PRE_CHECK → READY + ブレーキ原点復帰を抽出。
- `cancel_learning_drive()` / `cancel_auto_drive()` は共に `_cancel_armed_drive()` を呼ぶ。
- `start_auto_drive()`: 二経路に対応。
  - PRE_CHECK（arm 済み）の場合: セッション開始 → RUNNING へ遷移（走行前チェックは arm 済み）。
  - READY（直接呼び出し）の場合: 従来どおり `_run_pre_check_and_transition` で開始（後方互換）。

## API 変更（`src/web/routers/drive.py`）

- `POST /api/v1/drive/arm` → `arm_auto_drive()`（status='armed'、409/422 マッピング）
- `POST /api/v1/drive/cancel` → `cancel_auto_drive()`（status='cancelled'、409 マッピング）

## フロント変更（`src/web/static/js/screens/auto-drive.js`）

`AutoDriveScreen` から `DriveMonitorScreen` へ以下を渡す:
- `driveArmPath="/api/v1/drive/arm"`
- `driveCancelPath="/api/v1/drive/cancel"`
- `confirmStartMessage="自動走行を開始しますか？"`

## テスト

- `tests/unit/test_robot_controller.py`: `arm_auto_drive` / `start_auto_drive`(PRE_CHECK) /
  `cancel_auto_drive` のテストを追加。既存の READY 直接開始テストは後方互換で維持。
- `tests/unit/test_web_drive.py`: `/drive/arm` / `/drive/cancel` のテストを追加。
