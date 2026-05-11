# 要求内容: コア機能（MVP）実装

## 背景

ハードウェアはまだ準備できていないが、ソフトウェアの主要機能（MVP）を実装し、
スタブモードで動作確認できる状態にする。

## 対象機能（PRD §コア機能）

以下は実装済み：
- ✅ 制御ループ (DriveLoop 50ms)
- ✅ CalibrationManager
- ✅ PreCheckRunner (6項目)
- ✅ SafetyMonitor
- ✅ RobotController (状態機械)
- ✅ 走行制御 Web API (`/api/v1/drive/`)
- ✅ DB スキーマ (setup_db.py DDL)
- ✅ ProfileRepository.save_calibration

以下が未実装（本タスクで実装）：
1. **車両プロファイル管理** - CRUD API
2. **走行モード管理** - CSV アップロード + CRUD API
3. **セッション一覧** - 走行履歴閲覧 API
4. **プロファイル選択** - RobotController への反映
5. **In-memory リポジトリ** - HW なしでテスト可能

## ハードウェアなしでの動作条件

- スタブモード (`DRIVING_ROBOT_USE_REAL_HW` 未設定) で動作
- DB なしでも In-memory リポジトリで動作
- DB ありの場合は PostgreSQL に永続化
