---
name: handover-impl-tasklist
description: 申し送り事項実装タスクリスト
metadata:
  type: project
---

# タスクリスト: 申し送り事項の実装

## A. 再現性確保

- [x] A1-1 config/udev/99-driving-robot-actuators.rules.example を作成（テンプレート）
- [x] A1-2 scripts/setup_udev.sh を作成（--list / シリアル指定で設置・reload）
- [x] A1-3 .gitignore に実 .rules を除外（.example は管理）
- [x] A2 scripts/setup_db.py に calibration_data の UNIQUE 制約追加マイグレーション（冪等DOブロック）
- [x] A3 config/settings.toml.example の accel_port/brake_port を /dev/actuator_* に更新＋案内コメント

## B. キャリブレーション改善

- [x] B4-1 ActuatorDriver.wait_for_position_complete を実装（DSS1 PEND / DSSE MOVE）
- [x] B4-2 RobotController.ActuatorDriverProtocol に wait_for_position_complete を追加
- [x] B4-3 jog_axis で move_to_position 後に wait_for_position_complete してから read_position
- [x] B4-4 _StubActuator と test mock に wait_for_position_complete(no-op) を追加
- [x] B5-1 save_manual_calibration を「成功時のみ原点復帰＋READY、失敗時 CALIBRATING 維持」に変更
- [x] B5-2 test_robot_controller の保存失敗テストを新挙動に更新＋例外時テスト追加（+ jog 待ちテスト）

## 検証

- [x] pytest（unit 全件）pass → 437 passed
- [x] ruff check クリーン（変更ファイル。既存の app.py/modes.py/setup_db import は HEAD 既存問題で範囲外）
- [x] mypy クリーン
- [x] A2 setup_db.py 冪等実行を実機DBで確認（制約維持・エラーなし）
- [x] A1 setup_udev.sh --list を実機で確認（FTBB7KRI/FTAQUJJJ 検出）
- [x] 新コードで実HWサーバ再起動 → STANDBY 到達

## 実装後の振り返り

**実装完了日**: 2026-06-09

**結果**: 実機検証の申し送り5項目すべて実装・検証完了。

**実装内容**:
- A1: udev ルールをテンプレート(config/udev/*.example)＋設置スクリプト(scripts/setup_udev.sh)
  化。実 .rules は gitignore。--list で FTDI RS485 シリアル検出。
- A2: setup_db.py に calibration_data の UNIQUE(profile_id) を冪等追加する DO ブロック追加。
- A3: settings.toml.example を /dev/actuator_* 推奨に更新＋udev案内コメント。
- B4: ActuatorDriver.wait_for_position_complete（DSS1 PEND かつ DSSE 非 MOVE）を追加し、
  jog_axis で移動完了を待ってから read_position。DriveLoop は move_to_position のみ使うため非影響。
- B5: save_manual_calibration を成功時のみ原点復帰＋READY、失敗・例外時は CALIBRATING 維持・
  pending 保持に変更（リトライ導線）。

**計画との差分**: jog の不正軸テストも追加（テスト網羅性向上）。既存の lint 問題
（app.py 未使用 import 等）は HEAD 既存のため本タスクでは触らず。

**次回への改善提案**:
- wait_for_position_complete のタイムアウト時、jog は TimeoutError を伝播し API 500 となる。
  必要なら GUI 側でリトライ/警告表示の検討。
- 既存 lint 問題（src/web/app.py の未使用 import、modes.py の E402、setup_db の import 整列）の
  別タスクでのクリーンアップ。
</content>
